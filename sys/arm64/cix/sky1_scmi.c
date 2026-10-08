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
 * SCMI for CIX Sky1 devices under ACPI.
 *
 * Sky1's firmware describes its SCMI agent as an ACPI device (CIXHA006)
 * holding devicetree-style _DSD properties: "mboxes", a sending and a
 * receiving CIX mailbox, and "shmem", shared memory that is the sending
 * mailbox's message registers.  FreeBSD's SCMI framework is devicetree
 * only, so this is a small client of its own: requests are written to the
 * shared memory, the doorbell rung, and the reply polled for (the platform
 * marks the channel free).  It offers the clock protocol to drivers, such
 * as the GPU's, whose clocks firmware leaves off.
 *
 * The firmware's AML has SCMI methods too (\_SB.PMMX), on another agent,
 * which answers NOT_FOUND for every clock.
 *
 * Power domains are TF-A's: SCMI POWER_STATE_SET over an SMC, with shared
 * memory of its own, as CIX's Linux does under ACPI.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/sx.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/bus.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>
#include <dev/psci/smccc.h>

#include "acpi_if.h"

#include <arm64/cix/cix_mbox.h>
#include <arm64/cix/sky1_scmi.h>

/* SCMI shared memory (SCMI 3.x, 5.1.2) */
#define	SHMEM_CHAN_STATUS	0x04
#define	 CHAN_FREE		(1u << 0)
#define	 CHAN_ERROR		(1u << 1)
#define	SHMEM_FLAGS		0x10
#define	SHMEM_LENGTH		0x14
#define	SHMEM_HEADER		0x18
#define	SHMEM_PAYLOAD		0x1c
#define	SHMEM_SIZE		0x80
#define	SHMEM_MAX_WORDS		((SHMEM_SIZE - SHMEM_PAYLOAD) / 4)

#define	SCMI_HDR(proto, msg, tok)	((msg) | (proto) << 10 | \
	((tok) & 0x3ff) << 18)
#define	SCMI_NOT_FOUND		(-4)

#define	SCMI_PROTO_BASE		0x10
#define	SCMI_PROTO_CLOCK	0x14
#define	 CLOCK_RATE_SET		5
#define	 CLOCK_RATE_GET		6
#define	 CLOCK_CONFIG_SET	7
#define	SCMI_BASE_DISCOVER_VENDOR	3

#define	SCMI_REPLY_TIMEOUT_US	300000	/* as Linux on Sky1 */

/* TF-A's SCMI over SMC (Sky1 power domains). */
#define	SKY1_SMC_SCMI		0xc2000001
#define	SKY1_SMC_SHMEM		0x84380000UL
#define	SCMI_PROTO_POWER	0x11
#define	 POWER_STATE_SET	4
#define	 POWER_STATE_ON		0
#define	 POWER_STATE_OFF	0x40000000

struct sky1_scmi_softc {
	device_t	dev;
	ACPI_HANDLE	tx_h;		/* "mboxes"[0] */
	device_t	tx;		/* its mailbox, once attached */
	struct sx	lock;
	u_int		token;
};

static struct sky1_scmi_softc *sky1_scmi_sc;
static struct sx sky1_smc_lock;
SX_SYSINIT(sky1_smc_lock, &sky1_smc_lock, "sky1 smc scmi");

static int
sky1_scmi_status(int32_t status)
{
	if (status == 0)
		return (0);
	return (status == SCMI_NOT_FOUND ? ENOENT : EIO);
}

int
sky1_scmi_request(uint32_t protocol, uint32_t msg, const uint32_t *tx,
    int ntx, uint32_t *rx, int nrx)
{
	struct sky1_scmi_softc *sc = sky1_scmi_sc;
	device_t mb;
	uint32_t st;
	int error, i, us;

	if (sc == NULL)
		return (ENXIO);
	/* A reply's status takes the payload's first word. */
	if (ntx > SHMEM_MAX_WORDS || nrx > SHMEM_MAX_WORDS - 1)
		return (EINVAL);
	sx_xlock(&sc->lock);
	/* The sending mailbox may attach after this driver. */
	if (sc->tx == NULL)
		sc->tx = cix_mbox_get(sc->tx_h);
	if ((mb = sc->tx) == NULL) {
		error = ENXIO;
		goto out;
	}
	if ((cix_mbox_msg_read(mb, SHMEM_CHAN_STATUS) & CHAN_FREE) == 0) {
		device_printf(sc->dev, "channel busy\n");
		error = EBUSY;
		goto out;
	}
	cix_mbox_msg_write(mb, SHMEM_FLAGS, 0);		/* polled */
	cix_mbox_msg_write(mb, SHMEM_HEADER,
	    SCMI_HDR(protocol, msg, ++sc->token));
	for (i = 0; i < ntx; i++)
		cix_mbox_msg_write(mb, SHMEM_PAYLOAD + 4 * i, tx[i]);
	cix_mbox_msg_write(mb, SHMEM_LENGTH, 4 + 4 * ntx);
	cix_mbox_msg_write(mb, SHMEM_CHAN_STATUS, 0);	/* the platform's */
	cix_mbox_ring(mb);
	for (us = 0; us < SCMI_REPLY_TIMEOUT_US; us += 10) {
		st = cix_mbox_msg_read(mb, SHMEM_CHAN_STATUS);
		if ((st & CHAN_FREE) != 0)
			break;
		DELAY(10);
	}
	if ((st & CHAN_FREE) == 0) {
		device_printf(sc->dev, "no reply to protocol %#x message %u\n",
		    protocol, msg);
		error = ETIMEDOUT;
		goto out;
	}
	if ((st & CHAN_ERROR) != 0) {
		error = EIO;
		goto out;
	}
	/* The reply: status, then the return values. */
	error = sky1_scmi_status(cix_mbox_msg_read(mb, SHMEM_PAYLOAD));
	for (i = 0; error == 0 && i < nrx; i++)
		rx[i] = cix_mbox_msg_read(mb, SHMEM_PAYLOAD + 4 * (i + 1));
out:
	sx_xunlock(&sc->lock);
	return (error);
}

int
sky1_scmi_clk_enable(uint32_t id, bool enable)
{
	uint32_t tx[2] = { id, enable ? 1 : 0 };

	return (sky1_scmi_request(SCMI_PROTO_CLOCK, CLOCK_CONFIG_SET, tx, 2,
	    NULL, 0));
}

int
sky1_scmi_clk_get_rate(uint32_t id, uint64_t *hz)
{
	uint32_t rx[2];
	int error;

	error = sky1_scmi_request(SCMI_PROTO_CLOCK, CLOCK_RATE_GET, &id, 1,
	    rx, 2);
	if (error == 0)
		*hz = (uint64_t)rx[1] << 32 | rx[0];
	return (error);
}

int
sky1_scmi_clk_set_rate(uint32_t id, uint64_t hz)
{
	/* Synchronous, rounding down. */
	uint32_t tx[4] = { 0, id, (uint32_t)hz, hz >> 32 };

	return (sky1_scmi_request(SCMI_PROTO_CLOCK, CLOCK_RATE_SET, tx, 4,
	    NULL, 0));
}

int
sky1_scmi_power_set(uint32_t domain, bool on)
{
	struct arm_smccc_res res;
	uint8_t *sh;
	int error, i;

#define	SH(o)	(*(volatile uint32_t *)(sh + (o)))
	sx_xlock(&sky1_smc_lock);
	sh = pmap_mapdev(SKY1_SMC_SHMEM, PAGE_SIZE);
	for (i = 0; i < 1000 && (SH(SHMEM_CHAN_STATUS) & CHAN_FREE) == 0; i++)
		DELAY(10);
	if ((SH(SHMEM_CHAN_STATUS) & CHAN_FREE) == 0) {
		error = EBUSY;
		goto out;
	}
	SH(SHMEM_CHAN_STATUS) = 0;
	SH(SHMEM_FLAGS) = 0;
	SH(SHMEM_HEADER) = SCMI_HDR(SCMI_PROTO_POWER, POWER_STATE_SET, 0);
	SH(SHMEM_PAYLOAD) = 0;				/* synchronous */
	SH(SHMEM_PAYLOAD + 4) = domain;
	SH(SHMEM_PAYLOAD + 8) = on ? POWER_STATE_ON : POWER_STATE_OFF;
	SH(SHMEM_LENGTH) = 16;
	dsb(sy);
	arm_smccc_smc(SKY1_SMC_SCMI, SKY1_SMC_SHMEM >> 12,
	    SKY1_SMC_SHMEM & PAGE_MASK, 0, 0, 0, 0, 0, &res);
	error = res.a0 != 0 ? EIO : sky1_scmi_status(SH(SHMEM_PAYLOAD));
out:
	pmap_unmapdev(sh, PAGE_SIZE);
	sx_xunlock(&sky1_smc_lock);
	return (error);
#undef SH
}

static int
sky1_scmi_probe(device_t dev)
{
	static char *ids[] = { "CIXHA006", NULL };
	int rv;

	if (acpi_disabled("sky1_scmi"))
		return (ENXIO);
	rv = ACPI_ID_PROBE(device_get_parent(dev), dev, ids, NULL);
	if (rv <= 0)
		device_set_desc(dev, "CIX Sky1 SCMI");
	return (rv);
}

static int
sky1_scmi_attach(device_t dev)
{
	struct sky1_scmi_softc *sc = device_get_softc(dev);
	const ACPI_OBJECT *mboxes, *ref;
	uint32_t vendor[4];
	char name[17];

	sc->dev = dev;
	if (sky1_scmi_sc != NULL)
		return (ENXIO);
	/* "mboxes": <sending mailbox, channel, receiving mailbox, channel>. */
	if (ACPI_FAILURE(ACPI_GET_PROPERTY(device_get_parent(dev), dev,
	    "mboxes", &mboxes)) || mboxes->Type != ACPI_TYPE_PACKAGE ||
	    mboxes->Package.Count < 2 ||
	    (ref = &mboxes->Package.Elements[0])->Type !=
	    ACPI_TYPE_LOCAL_REFERENCE) {
		device_printf(dev, "no mailboxes\n");
		return (ENXIO);
	}
	sc->tx_h = ref->Reference.Handle;
	sx_init(&sc->lock, "sky1 scmi");
	sky1_scmi_sc = sc;

	/* The sending mailbox is in an earlier pass: say hello. */
	if (sky1_scmi_request(SCMI_PROTO_BASE, SCMI_BASE_DISCOVER_VENDOR,
	    NULL, 0, vendor, 4) == 0) {
		memcpy(name, vendor, 16);
		name[16] = '\0';
		device_printf(dev, "vendor %s\n", name);
	} else
		device_printf(dev, "no reply from the platform yet\n");
	return (0);
}

static device_method_t sky1_scmi_methods[] = {
	DEVMETHOD(device_probe,		sky1_scmi_probe),
	DEVMETHOD(device_attach,	sky1_scmi_attach),
	DEVMETHOD_END
};

static driver_t sky1_scmi_driver = {
	"sky1_scmi",
	sky1_scmi_methods,
	sizeof(struct sky1_scmi_softc),
};

EARLY_DRIVER_MODULE(sky1_scmi, acpi, sky1_scmi_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);
