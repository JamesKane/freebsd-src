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
 * Realtek RTS5453 USB Type-C power delivery controllers, as CIX Sky1 boards'
 * firmware describes them in ACPI (CIXH200D): each port's state (partner,
 * mode, orientation, data and power roles, DisplayPort hot plug) as
 * sysctls, logged as it changes.
 *
 * The controller runs the port itself; this only watches.  Polled, as GPIO
 * interrupts are not yet to be had: its event flag is acknowledged when
 * its state changes.
 */

#include "opt_acpi.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/sbuf.h>
#include <sys/sx.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include <dev/iicbus/iicbus.h>
#include <dev/iicbus/iiconf.h>

#include "iicbus_if.h"

#define	RTS5453_DATA_CTRL	0x50
#define	RTS5453_INT_ACK		(1 << 2)
#define	RTS5453_HPD_IRQ_ACK	(1 << 5)
#define	RTS5453_DATA_STATUS	0x5f	/* length, then the status */
#define	RTS5453_STATUS_LEN	6

/* Status byte 1 */
#define	RTS5453_ORIENTATION	(1 << 1)	/* reverse */
#define	RTS5453_USB2_CONN	(1 << 4)
#define	RTS5453_USB3_CONN	(1 << 5)
#define	RTS5453_DATA_ROLE	(1 << 7)	/* device */
/* Status byte 2 */
#define	RTS5453_DP_CONN		(1 << 0)
#define	RTS5453_DEBUG_ACC	(1 << 4)
#define	RTS5453_HPD_IRQ		(1 << 6)
#define	RTS5453_HPD_STATE	(1 << 7)
/* Status byte 3 */
#define	RTS5453_PWR_SOURCE	(1 << 0)
#define	RTS5453_PWR_SINK	(1 << 1)

static char *rts5453_ids[] = {
	"CIXH200D",		/* CIX Sky1 boards */
	NULL
};

struct rts5453_softc {
	device_t		dev;
	struct sx		lock;
	struct timeout_task	poll_task;
	int			poll_ms;
	bool			detaching;
	bool			valid;
	bool			read_failed;
	uint8_t			status[RTS5453_STATUS_LEN];
};

/*
 * Register reads and writes at the device's bus address, which ACPI gives
 * as 7 bits (acpi_iicbus) where FreeBSD's I2C drivers take 8.
 */
static uint16_t
rts5453_slave(device_t dev)
{
	return (iicbus_get_addr(dev) << 1);
}

static int
rts5453_readfrom(device_t dev, uint8_t reg, void *buf, uint16_t len)
{
	struct iic_msg msgs[2] = {
		{ rts5453_slave(dev), IIC_M_WR | IIC_M_NOSTOP, 1, &reg },
		{ rts5453_slave(dev), IIC_M_RD, len, buf },
	};

	return (iicbus_transfer_excl(dev, msgs, nitems(msgs), IIC_WAIT));
}

static int
rts5453_writeto(device_t dev, uint8_t reg, const void *buf, uint16_t len)
{
	uint8_t data[1 + 16];
	struct iic_msg msg = { rts5453_slave(dev), IIC_M_WR, len + 1, data };

	if (len > sizeof(data) - 1)
		return (IIC_EOVERFLOW);
	data[0] = reg;
	memcpy(&data[1], buf, len);
	return (iicbus_transfer_excl(dev, &msg, 1, IIC_WAIT));
}

static int
rts5453_probe(device_t dev)
{
	ACPI_HANDLE handle;
	int i;

	if ((handle = acpi_get_handle(dev)) == NULL)
		return (ENXIO);
	for (i = 0; rts5453_ids[i] != NULL; i++) {
		if (acpi_MatchHid(handle, rts5453_ids[i])) {
			device_set_desc(dev,
			    "Realtek RTS5453 USB Type-C PD controller");
			return (BUS_PROBE_DEFAULT);
		}
	}
	return (ENXIO);
}

static const char *
rts5453_mode(const uint8_t *st)
{
	if (st[2] & RTS5453_DEBUG_ACC)
		return ("debug accessory");
	if ((st[1] & RTS5453_USB3_CONN) && (st[2] & RTS5453_DP_CONN))
		return ("USB+DP");
	if (st[2] & RTS5453_DP_CONN)
		return ("DP");
	if (st[1] & (RTS5453_USB3_CONN | RTS5453_USB2_CONN))
		return ("USB");
	return ("none");
}

static const char *
rts5453_orientation(const uint8_t *st)
{
	if (!(st[1] & RTS5453_USB3_CONN) && !(st[2] & RTS5453_DP_CONN))
		return ("none");
	return ((st[1] & RTS5453_ORIENTATION) ? "reverse" : "normal");
}

static const char *
rts5453_data_role(const uint8_t *st)
{
	if (!(st[1] & (RTS5453_USB3_CONN | RTS5453_USB2_CONN)))
		return ("none");
	return ((st[1] & RTS5453_DATA_ROLE) ? "device" : "host");
}

static const char *
rts5453_power_role(const uint8_t *st)
{
	switch (st[3] & (RTS5453_PWR_SOURCE | RTS5453_PWR_SINK)) {
	case RTS5453_PWR_SOURCE:
		return ("source");
	case RTS5453_PWR_SINK:
		return ("sink");
	default:
		return ("none");
	}
}

static void
rts5453_ack(struct rts5453_softc *sc, bool hpd)
{
	uint8_t ctrl[3] = { 2, RTS5453_INT_ACK, 0 };

	if (hpd)
		ctrl[2] = RTS5453_HPD_IRQ_ACK;
	(void)rts5453_writeto(sc->dev, RTS5453_DATA_CTRL, ctrl, sizeof(ctrl));
}

static void
rts5453_poll(void *arg, int pending __unused)
{
	struct rts5453_softc *sc = arg;
	uint8_t st[RTS5453_STATUS_LEN];
	int error;

	sx_xlock(&sc->lock);
	if (sc->detaching) {
		sx_xunlock(&sc->lock);
		return;
	}
	error = rts5453_readfrom(sc->dev, RTS5453_DATA_STATUS, st,
	    sizeof(st));
	if (error != 0 && !sc->read_failed) {
		device_printf(sc->dev, "cannot read its status: %d\n", error);
		sc->read_failed = true;
	} else if (error == 0)
		sc->read_failed = false;
	if (error == 0 && (!sc->valid ||
	    memcmp(st + 1, sc->status + 1, sizeof(st) - 1) != 0)) {
		device_printf(sc->dev, "%s, %s, data %s, power %s%s\n",
		    rts5453_mode(st), rts5453_orientation(st),
		    rts5453_data_role(st), rts5453_power_role(st),
		    (st[2] & RTS5453_DP_CONN) ?
		    ((st[2] & RTS5453_HPD_STATE) ? ", HPD high" :
		    ", HPD low") : "");
		if (sc->valid)
			rts5453_ack(sc, (st[2] & RTS5453_HPD_IRQ) != 0);
		memcpy(sc->status, st, sizeof(st));
		sc->valid = true;
	}
	taskqueue_enqueue_timeout(taskqueue_thread, &sc->poll_task,
	    MAX(1, sc->poll_ms * hz / 1000));
	sx_xunlock(&sc->lock);
}

/* A field of the last status read, decoded (arg2 selects it). */
static int
rts5453_sysctl_field(SYSCTL_HANDLER_ARGS)
{
	struct rts5453_softc *sc = arg1;
	const char *s;
	char buf[32];

	sx_slock(&sc->lock);
	if (!sc->valid)
		s = "unknown";
	else {
		switch (arg2) {
		case 0:
			s = rts5453_mode(sc->status);
			break;
		case 1:
			s = rts5453_orientation(sc->status);
			break;
		case 2:
			s = rts5453_data_role(sc->status);
			break;
		case 3:
			s = rts5453_power_role(sc->status);
			break;
		case 4:
			s = !(sc->status[2] & RTS5453_DP_CONN) ? "none" :
			    (sc->status[2] & RTS5453_HPD_STATE) ? "high" :
			    "low";
			break;
		default:
			snprintf(buf, sizeof(buf), "%02x %02x %02x %02x %02x",
			    sc->status[1], sc->status[2], sc->status[3],
			    sc->status[4], sc->status[5]);
			s = buf;
			break;
		}
	}
	strlcpy(buf, s, sizeof(buf));
	sx_sunlock(&sc->lock);
	return (sysctl_handle_string(oidp, buf, sizeof(buf), req));
}

static int
rts5453_attach(device_t dev)
{
	static const struct {
		const char *name, *desc;
	} fields[] = {
		{ "mode", "Partner's mode: none, USB, DP, USB+DP" },
		{ "orientation", "Plug orientation" },
		{ "data_role", "Data role: host, device or none" },
		{ "power_role", "Power role: source, sink or none" },
		{ "hpd", "DisplayPort hot plug: high, low or none" },
		{ "status", "Raw status bytes" },
	};
	struct rts5453_softc *sc;
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid_list *tree;
	u_int i;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->poll_ms = 1000;
	sx_init(&sc->lock, device_get_nameunit(dev));
	TIMEOUT_TASK_INIT(taskqueue_thread, &sc->poll_task, 0, rts5453_poll,
	    sc);

	ctx = device_get_sysctl_ctx(dev);
	tree = SYSCTL_CHILDREN(device_get_sysctl_tree(dev));
	for (i = 0; i < nitems(fields); i++)
		SYSCTL_ADD_PROC(ctx, tree, OID_AUTO, fields[i].name,
		    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, i,
		    rts5453_sysctl_field, "A", fields[i].desc);
	SYSCTL_ADD_INT(ctx, tree, OID_AUTO, "poll_ms", CTLFLAG_RW,
	    &sc->poll_ms, 0, "Polling interval (ms)");

	/* The first poll once the bus is up. */
	taskqueue_enqueue_timeout(taskqueue_thread, &sc->poll_task, 1);
	return (0);
}

static int
rts5453_detach(device_t dev)
{
	struct rts5453_softc *sc = device_get_softc(dev);

	sx_xlock(&sc->lock);
	sc->detaching = true;
	sx_xunlock(&sc->lock);
	while (taskqueue_cancel_timeout(taskqueue_thread, &sc->poll_task,
	    NULL) != 0)
		taskqueue_drain_timeout(taskqueue_thread, &sc->poll_task);
	sx_destroy(&sc->lock);
	return (0);
}

static device_method_t rts5453_methods[] = {
	DEVMETHOD(device_probe,		rts5453_probe),
	DEVMETHOD(device_attach,	rts5453_attach),
	DEVMETHOD(device_detach,	rts5453_detach),
	DEVMETHOD_END
};

static driver_t rts5453_driver = {
	"rts5453",
	rts5453_methods,
	sizeof(struct rts5453_softc),
};

DRIVER_MODULE(rts5453, iicbus, rts5453_driver, NULL, NULL);
MODULE_DEPEND(rts5453, iicbus, IICBUS_MINVER, IICBUS_PREFVER, IICBUS_MAXVER);
MODULE_DEPEND(rts5453, acpi, 1, 1, 1);
MODULE_VERSION(rts5453, 1);
IICBUS_ACPI_PNP_INFO(rts5453_ids);
