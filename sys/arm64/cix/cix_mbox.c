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
 * CIX Sky1 mailbox (ACPI CIXHA001).
 *
 * Each mailbox is one direction between two processors (_DSD
 * "cix,mbox_dir": 0 sends from here, 1 receives).  Its first 128 bytes are
 * message registers, which SCMI uses as its shared memory; a write to
 * DB_ACK rings the doorbell of a sending mailbox.  Only the doorbell
 * channel is supported, the one SCMI uses; the FIFO, register and fast
 * channels are not, nor interrupts: SCMI is polled.
 *
 * Register layout from CIX's Linux driver, used as reference only.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include <arm64/cix/cix_mbox.h>

#define	CIX_MBOX_MSG_SIZE	0x80		/* message registers */
#define	CIX_MBOX_DB_ACK		0x80
#define	 CIX_MBOX_DB_INT	(1u << 0)
#define	CIX_MBOX_INT_ENABLE	0xbc

struct cix_mbox_softc {
	device_t		dev;
	struct resource		*mem;
	uint32_t		dir;		/* CIX_MBOX_TX or _RX */
};

static int
cix_mbox_probe(device_t dev)
{
	static char *ids[] = { "CIXHA001", NULL };
	int rv;

	if (acpi_disabled("cix_mbox"))
		return (ENXIO);
	rv = ACPI_ID_PROBE(device_get_parent(dev), dev, ids, NULL);
	if (rv <= 0)
		device_set_desc(dev, "CIX mailbox");
	return (rv);
}

static int
cix_mbox_attach(device_t dev)
{
	struct cix_mbox_softc *sc = device_get_softc(dev);
	int rid;

	sc->dev = dev;
	if (device_get_property(dev, "cix,mbox_dir", &sc->dir,
	    sizeof(sc->dir), DEVICE_PROP_UINT32) <= 0 ||
	    (sc->dir != CIX_MBOX_TX && sc->dir != CIX_MBOX_RX)) {
		device_printf(dev, "no direction\n");
		return (ENXIO);
	}
	rid = 0;
	sc->mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->mem == NULL) {
		device_printf(dev, "cannot map registers\n");
		return (ENXIO);
	}
	if (bootverbose)
		device_printf(dev, "%s\n", sc->dir == CIX_MBOX_TX ? "send" :
		    "receive");
	return (0);
}

static int
cix_mbox_detach(device_t dev)
{
	struct cix_mbox_softc *sc = device_get_softc(dev);

	bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem);
	return (0);
}

device_t
cix_mbox_get(ACPI_HANDLE h)
{
	device_t dev;

	dev = acpi_get_device(h);
	if (dev == NULL || !device_is_attached(dev) ||
	    strcmp(device_get_name(dev), "cix_mbox") != 0)
		return (NULL);
	return (dev);
}

int
cix_mbox_dir(device_t dev)
{
	struct cix_mbox_softc *sc = device_get_softc(dev);

	return (sc->dir);
}

bus_addr_t
cix_mbox_base(device_t dev)
{
	struct cix_mbox_softc *sc = device_get_softc(dev);

	return (rman_get_start(sc->mem));
}

void
cix_mbox_ring(device_t dev)
{
	struct cix_mbox_softc *sc = device_get_softc(dev);

	KASSERT(sc->dir == CIX_MBOX_TX, ("%s: receiving mailbox", __func__));
	bus_barrier(sc->mem, 0, CIX_MBOX_MSG_SIZE, BUS_SPACE_BARRIER_WRITE);
	bus_write_4(sc->mem, CIX_MBOX_DB_ACK, CIX_MBOX_DB_INT);
}

uint32_t
cix_mbox_msg_read(device_t dev, bus_size_t off)
{
	struct cix_mbox_softc *sc = device_get_softc(dev);

	KASSERT(off < CIX_MBOX_MSG_SIZE, ("%s: offset %#jx", __func__,
	    (uintmax_t)off));
	return (bus_read_4(sc->mem, off));
}

void
cix_mbox_msg_write(device_t dev, bus_size_t off, uint32_t val)
{
	struct cix_mbox_softc *sc = device_get_softc(dev);

	KASSERT(off < CIX_MBOX_MSG_SIZE, ("%s: offset %#jx", __func__,
	    (uintmax_t)off));
	bus_write_4(sc->mem, off, val);
}

static device_method_t cix_mbox_methods[] = {
	DEVMETHOD(device_probe,		cix_mbox_probe),
	DEVMETHOD(device_attach,	cix_mbox_attach),
	DEVMETHOD(device_detach,	cix_mbox_detach),
	DEVMETHOD_END
};

static driver_t cix_mbox_driver = {
	"cix_mbox",
	cix_mbox_methods,
	sizeof(struct cix_mbox_softc),
};

EARLY_DRIVER_MODULE(cix_mbox, acpi, cix_mbox_driver, 0, 0,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_MIDDLE);
