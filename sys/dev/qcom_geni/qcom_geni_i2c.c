/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 James Kane
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

/*
 * I2C controller on a Qualcomm GENI serial engine (QUP), as the firmware
 * leaves it: its I2C protocol firmware loaded, its clocks on and its bus
 * clock set up, which the driver keeps.  Transfers use the engine's FIFOs,
 * one byte to a FIFO word, and are polled: the buses carry small transfers
 * to chips such as RTCs and EEPROMs.  The sequence follows Linux's
 * i2c-qcom-geni.c: one engine command per message, a message but the last
 * holding the bus for a repeated start.
 *
 * ACPI doesn't describe the chips on the buses.  For boards that are known,
 * by their SMBIOS names and the bus's ACPI _UID, the chips are declared as
 * hints for the iicbus, which attaches them.
 */

#include "opt_acpi.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/time.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include <dev/iicbus/iicbus.h>
#include <dev/iicbus/iiconf.h>

#include <dev/qcom_geni/qcom_geni_reg.h>

#include "iicbus_if.h"

/* I2C protocol registers and commands. */
#define	GENI_I2C_TX_TRANS_LEN		0x026c
#define	GENI_I2C_RX_TRANS_LEN		0x0270
#define	GENI_I2C_WRITE			0x1
#define	GENI_I2C_READ			0x2
#define	 GENI_I2C_STOP_STRETCH		(1u << 2)
#define	 GENI_I2C_SLV_ADDR_SHIFT	9
#define	GENI_M_GP_LENGTH		0x0910
#define	GENI_TX_FIFO_WC(v)		((v) & 0xfffffff)

/* Main sequencer interrupt status bits, beyond those in the header. */
#define	GENI_M_CMD_OVERRUN		(1u << 1)
#define	GENI_M_ILLEGAL_CMD		(1u << 2)
#define	GENI_M_CMD_FAILURE		(1u << 3)
#define	GENI_M_GP_IRQ_1			(1u << 10)	/* NACK */
#define	GENI_M_GP_IRQ_3			(1u << 12)	/* bus protocol */
#define	GENI_M_GP_IRQ_4			(1u << 13)	/* arbitration lost */
#define	GENI_M_ERRORS			(GENI_M_CMD_OVERRUN | \
	GENI_M_ILLEGAL_CMD | GENI_M_CMD_FAILURE | GENI_M_GP_IRQ_1 | \
	GENI_M_GP_IRQ_3 | GENI_M_GP_IRQ_4)

#define	GENI_I2C_TIMEOUT_US		100000	/* for a command */

struct qcom_geni_i2c_softc {
	device_t		dev;
	struct resource		*mem;
	struct mtx		mtx;
	device_t		iicbus;
	u_int			tx_depth;	/* FIFO words */
};

#define	RD4(sc, r)	bus_read_4((sc)->mem, (r))
#define	WR4(sc, r, v)	bus_write_4((sc)->mem, (r), (v))

static char *qcom_geni_i2c_acpi_ids[] = { "QCOM0610", NULL };

/* The chips on a bus: the driver, its 8-bit address and its compatible. */
struct qcom_geni_i2c_chip {
	const char	*name;
	int		addr;
	const char	*compat;
};

static const struct qcom_geni_i2c_board {
	const char			*maker;
	const char			*product;
	u_int				uid;
	const struct qcom_geni_i2c_chip	*chips;
} qcom_geni_i2c_boards[] = {
	{
		/* An ST M41T11 RTC, which works as a DS1307. */
		.maker = "Radxa Computer Co., Ltd.",
		.product = "Radxa Dragon Q8B",
		.uid = 13,
		.chips = (const struct qcom_geni_i2c_chip[]){
			{ "ds13rtc", 0xd0, "dallas,ds1307" },
			{ NULL }
		},
	},
};

/* Declare the known chips on this bus, unless hints already do. */
static void
qcom_geni_i2c_board_hints(struct qcom_geni_i2c_softc *sc)
{
	const struct qcom_geni_i2c_board *b;
	const struct qcom_geni_i2c_chip *c;
	char *maker, *product, name[64], val[32];
	const char *at;
	ACPI_HANDLE h;
	UINT32 uid;
	u_int i;
	int unit;

	h = acpi_get_handle(sc->dev);
	if (h == NULL || ACPI_FAILURE(acpi_GetInteger(h, "_UID", &uid)))
		return;
	maker = kern_getenv("smbios.system.maker");
	product = kern_getenv("smbios.system.product");
	b = NULL;
	for (i = 0; maker != NULL && product != NULL &&
	    i < nitems(qcom_geni_i2c_boards); i++) {
		if (strcmp(maker, qcom_geni_i2c_boards[i].maker) == 0 &&
		    strcmp(product, qcom_geni_i2c_boards[i].product) == 0 &&
		    uid == qcom_geni_i2c_boards[i].uid) {
			b = &qcom_geni_i2c_boards[i];
			break;
		}
	}
	freeenv(maker);
	freeenv(product);
	if (b == NULL)
		return;

	for (c = b->chips; c->name != NULL; c++) {
		/* The first unit of the driver with no hints. */
		for (unit = 0; resource_string_value(c->name, unit, "at",
		    &at) == 0; unit++)
			;
		snprintf(val, sizeof(val), "iicbus%d",
		    device_get_unit(sc->iicbus));
		snprintf(name, sizeof(name), "hint.%s.%d.at", c->name, unit);
		kern_setenv(name, val);
		snprintf(val, sizeof(val), "%#x", c->addr);
		snprintf(name, sizeof(name), "hint.%s.%d.addr", c->name, unit);
		kern_setenv(name, val);
		snprintf(name, sizeof(name), "hint.%s.%d.compatible", c->name,
		    unit);
		kern_setenv(name, c->compat);
	}
}

/* Stop the command under way: cancel it, or failing that abort it. */
static void
qcom_geni_i2c_stop(struct qcom_geni_i2c_softc *sc)
{
	int us;

	WR4(sc, GENI_M_CMD_CTRL, GENI_M_CMD_CANCEL);
	for (us = 0; us < 1000 && (RD4(sc, GENI_M_IRQ_STATUS) &
	    GENI_M_CMD_CANCEL_DONE) == 0; us += 10)
		DELAY(10);
	if (us >= 1000) {
		WR4(sc, GENI_M_CMD_CTRL, GENI_M_CMD_ABORT);
		for (us = 0; us < 1000 && (RD4(sc, GENI_M_IRQ_STATUS) &
		    GENI_M_CMD_ABORT_DONE) == 0; us += 10)
			DELAY(10);
	}
	WR4(sc, GENI_M_IRQ_CLEAR, 0xffffffff);
}

/* One message: one command, the FIFOs fed and drained as it runs. */
static int
qcom_geni_i2c_msg(struct qcom_geni_i2c_softc *sc, struct iic_msg *msg,
    bool last)
{
	uint32_t param, st;
	bool rd;
	u_int done, n, us;
	int error;

	rd = (msg->flags & IIC_M_RD) != 0;
	param = (msg->slave >> 1) << GENI_I2C_SLV_ADDR_SHIFT;
	if (!last)
		param |= GENI_I2C_STOP_STRETCH;
	WR4(sc, GENI_M_IRQ_CLEAR, 0xffffffff);
	WR4(sc, rd ? GENI_I2C_RX_TRANS_LEN : GENI_I2C_TX_TRANS_LEN, msg->len);
	WR4(sc, GENI_M_CMD0, (rd ? GENI_I2C_READ : GENI_I2C_WRITE) <<
	    GENI_M_OPCODE_SHIFT | param);

	done = 0;
	error = 0;
	for (us = 0;; us += 5) {
		if (rd) {
			n = GENI_RX_FIFO_WC(RD4(sc, GENI_RX_FIFO_STATUS));
			for (; n > 0; n--) {
				st = RD4(sc, GENI_RX_FIFO);
				if (done < msg->len)
					msg->buf[done++] = st & 0xff;
			}
		} else {
			n = GENI_TX_FIFO_WC(RD4(sc, GENI_TX_FIFO_STATUS));
			for (; n < sc->tx_depth && done < msg->len; n++)
				WR4(sc, GENI_TX_FIFO, msg->buf[done++]);
		}
		st = RD4(sc, GENI_M_IRQ_STATUS);
		if ((st & GENI_M_ERRORS) != 0) {
			if ((st & GENI_M_GP_IRQ_1) != 0)
				error = IIC_ENOACK;
			else if ((st & GENI_M_GP_IRQ_4) != 0)
				error = IIC_EBUSBSY;
			else
				error = IIC_EBUSERR;
			break;
		}
		if ((st & GENI_M_CMD_DONE) != 0) {
			/* The last words may still be in the FIFO. */
			if (rd && done < msg->len &&
			    GENI_RX_FIFO_WC(RD4(sc, GENI_RX_FIFO_STATUS)) != 0)
				continue;
			if (done < msg->len)
				error = IIC_EBUSERR;
			break;
		}
		if (us >= GENI_I2C_TIMEOUT_US) {
			error = IIC_ETIMEOUT;
			break;
		}
		DELAY(5);
	}
	if (error != 0 && (st & GENI_M_CMD_DONE) == 0)
		qcom_geni_i2c_stop(sc);
	WR4(sc, GENI_M_IRQ_CLEAR, 0xffffffff);
	return (error);
}

static int
qcom_geni_i2c_transfer(device_t dev, struct iic_msg *msgs, uint32_t nmsgs)
{
	struct qcom_geni_i2c_softc *sc = device_get_softc(dev);
	uint32_t i;
	int error;

	error = 0;
	mtx_lock(&sc->mtx);
	for (i = 0; i < nmsgs && error == 0; i++) {
		/* 10-bit addresses, and messages without a start, aren't. */
		if ((msgs[i].flags & IIC_M_NOSTART) != 0) {
			error = IIC_ENOTSUPP;
			break;
		}
		error = qcom_geni_i2c_msg(sc, &msgs[i], i == nmsgs - 1);
	}
	mtx_unlock(&sc->mtx);
	return (error);
}

static int
qcom_geni_i2c_reset(device_t dev, u_char speed, u_char addr, u_char *oldaddr)
{
	/* The bus speed is the firmware's. */
	return (0);
}

static int
qcom_geni_i2c_probe(device_t dev)
{
	if (ACPI_ID_PROBE(device_get_parent(dev), dev, qcom_geni_i2c_acpi_ids,
	    NULL) > 0)
		return (ENXIO);
	device_set_desc(dev, "Qualcomm GENI I2C controller");
	return (BUS_PROBE_DEFAULT);
}

static int
qcom_geni_i2c_attach(device_t dev)
{
	struct qcom_geni_i2c_softc *sc = device_get_softc(dev);
	uint32_t proto;
	int rid;

	sc->dev = dev;
	rid = 0;
	sc->mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->mem == NULL) {
		device_printf(dev, "no registers\n");
		return (ENXIO);
	}

	/* The firmware must have set the engine up for I2C. */
	proto = GENI_FW_REV_PROTOCOL(RD4(sc, GENI_FW_REVISION_RO));
	if (proto != GENI_PROTOCOL_I2C) {
		if (bootverbose)
			device_printf(dev, "engine runs protocol %u\n", proto);
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem);
		return (ENXIO);
	}
	sc->tx_depth = GENI_HW_PARAM_FIFO_DEPTH(RD4(sc, GENI_HW_PARAM_0));
	if (sc->tx_depth == 0) {
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem);
		return (ENXIO);
	}
	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	/* FIFO mode, one byte to a word, most significant bit first, polled. */
	WR4(sc, GENI_DMA_MODE_EN, RD4(sc, GENI_DMA_MODE_EN) &
	    ~GENI_DMA_MODE_ENABLE);
	WR4(sc, GENI_TX_PACKING_CFG0, GENI_PACKING_1x8_MSB);
	WR4(sc, GENI_TX_PACKING_CFG1, 0);
	WR4(sc, GENI_RX_PACKING_CFG0, GENI_PACKING_1x8_MSB);
	WR4(sc, GENI_RX_PACKING_CFG1, 0);
	WR4(sc, GENI_BYTE_GRANULARITY, 0);
	WR4(sc, GENI_SE_IRQ_EN, 0);
	WR4(sc, GENI_M_IRQ_EN, 0);
	WR4(sc, GENI_S_IRQ_EN, 0);
	WR4(sc, GENI_TX_WATERMARK, 0);
	WR4(sc, GENI_M_IRQ_CLEAR, 0xffffffff);
	WR4(sc, GENI_S_IRQ_CLEAR, 0xffffffff);

	sc->iicbus = device_add_child(dev, "iicbus", DEVICE_UNIT_ANY);
	qcom_geni_i2c_board_hints(sc);
	bus_attach_children(dev);
	return (0);
}

static int
qcom_geni_i2c_detach(device_t dev)
{
	struct qcom_geni_i2c_softc *sc = device_get_softc(dev);
	int error;

	error = bus_generic_detach(dev);
	if (error != 0)
		return (error);
	mtx_destroy(&sc->mtx);
	bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem);
	return (0);
}

static device_method_t qcom_geni_i2c_methods[] = {
	DEVMETHOD(device_probe,		qcom_geni_i2c_probe),
	DEVMETHOD(device_attach,	qcom_geni_i2c_attach),
	DEVMETHOD(device_detach,	qcom_geni_i2c_detach),

	DEVMETHOD(iicbus_callback,	iicbus_null_callback),
	DEVMETHOD(iicbus_reset,		qcom_geni_i2c_reset),
	DEVMETHOD(iicbus_transfer,	qcom_geni_i2c_transfer),

	DEVMETHOD_END
};

static driver_t qcom_geni_i2c_driver = {
	"qcom_geni_i2c",
	qcom_geni_i2c_methods,
	sizeof(struct qcom_geni_i2c_softc),
};

DRIVER_MODULE(qcom_geni_i2c, acpi, qcom_geni_i2c_driver, 0, 0);
DRIVER_MODULE(iicbus, qcom_geni_i2c, iicbus_driver, 0, 0);
MODULE_DEPEND(qcom_geni_i2c, acpi, 1, 1, 1);
MODULE_DEPEND(qcom_geni_i2c, iicbus, IICBUS_MINVER, IICBUS_PREFVER,
    IICBUS_MAXVER);
MODULE_DEPEND(qcom_geni_i2c, ds13rtc, 1, 1, 1);	/* for the boards' RTCs */
MODULE_VERSION(qcom_geni_i2c, 1);
ACPI_PNP_INFO(qcom_geni_i2c_acpi_ids);
