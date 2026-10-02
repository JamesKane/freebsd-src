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
 * Arm SBSA generic watchdog.
 *
 * A refresh frame, whose WRR a write to refreshes the watchdog, and a
 * control frame: WCS enables it, WOR is the offset from the system
 * counter at which it next expires.  The first expiry raises WS0 and
 * starts the offset again; the second raises WS1, which the platform
 * wires to a reset.  As Linux does by default, WS0 is not used: the
 * offset is half the timeout, and WS1 resets.
 *
 * The offset register is 32 bits in architecture version 0, 48 in later
 * ones, counting at the system counter's rate: with a 1 GHz counter,
 * version 0 allows timeouts of up to about 8 seconds.
 *
 * Only ACPI (the GTDT's watchdog entries) is supported.
 */

#include "opt_acpi.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/eventhandler.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>
#include <sys/watchdog.h>

#include <machine/armreg.h>
#include <machine/bus.h>
#include <machine/resource.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

/* Refresh frame */
#define	SBSA_GWDT_WRR		0x000

/* Control frame */
#define	SBSA_GWDT_WCS		0x000
#define	 WCS_EN			(1u << 0)
#define	 WCS_WS0		(1u << 1)
#define	 WCS_WS1		(1u << 2)
#define	SBSA_GWDT_WOR_LO	0x008
#define	SBSA_GWDT_WOR_HI	0x00c		/* version 1 and later */
#define	SBSA_GWDT_WCV_LO	0x010
#define	SBSA_GWDT_WCV_HI	0x014

/* Both frames */
#define	SBSA_GWDT_W_IIDR	0xfcc
#define	 W_IIDR_ARCH(x)		(((x) >> 16) & 0xf)

#define	SBSA_GWDT_SIZE		0x1000

struct sbsa_gwdt_softc {
	device_t		dev;
	struct resource		*refresh;
	struct resource		*ctl;
	struct mtx		mtx;
	eventhandler_tag	ev_tag;
	uint64_t		freq;
	uint64_t		max_wor;
	uint64_t		wor;		/* the offset set, if enabled */
	bool			wor_refresh;	/* WRR does not refresh */
};

static void
sbsa_gwdt_set_wor(struct sbsa_gwdt_softc *sc, uint64_t wor)
{
	/* A write to WOR also refreshes the watchdog. */
	if (sc->max_wor > UINT32_MAX)
		bus_write_4(sc->ctl, SBSA_GWDT_WOR_HI, wor >> 32);
	bus_write_4(sc->ctl, SBSA_GWDT_WOR_LO, (uint32_t)wor);
}

static uint64_t
sbsa_gwdt_wcv(struct sbsa_gwdt_softc *sc)
{
	uint32_t hi, lo;

	do {
		hi = bus_read_4(sc->ctl, SBSA_GWDT_WCV_HI);
		lo = bus_read_4(sc->ctl, SBSA_GWDT_WCV_LO);
	} while (bus_read_4(sc->ctl, SBSA_GWDT_WCV_HI) != hi);
	return ((uint64_t)hi << 32 | lo);
}

/*
 * Whether a write to WRR refreshes the watchdog: on CIX Sky1 it does not
 * (the compare value stays), and the watchdog resets the system however
 * often it is refreshed so.  A write to WOR refreshes it too, so such a
 * watchdog is refreshed that way.  Checked with the watchdog enabled for
 * ten minutes, and disabled again.
 */
static bool
sbsa_gwdt_wrr_refreshes(struct sbsa_gwdt_softc *sc)
{
	uint64_t wcv;
	bool ok;

	sbsa_gwdt_set_wor(sc, MIN(sc->freq * 600, sc->max_wor));
	bus_write_4(sc->ctl, SBSA_GWDT_WCS, WCS_EN);
	wcv = sbsa_gwdt_wcv(sc);
	DELAY(100);
	bus_write_4(sc->refresh, SBSA_GWDT_WRR, 0);
	ok = sbsa_gwdt_wcv(sc) != wcv;
	bus_write_4(sc->ctl, SBSA_GWDT_WCS, 0);
	return (ok);
}

static void
sbsa_gwdt_fn(void *arg, u_int cmd, int *error)
{
	struct sbsa_gwdt_softc *sc = arg;
	uint64_t ns, wor;

	mtx_lock(&sc->mtx);
	cmd &= WD_INTERVAL;
	wor = 0;
	if (cmd != 0) {
		ns = (uint64_t)1 << cmd;
		/* Half the timeout to WS0, half again to WS1's reset. */
		wor = ns / 2 / 1000 * sc->freq / 1000000;
		if (wor == 0 || wor > sc->max_wor)
			wor = 0;
	}
	if (wor == 0) {
		/* Disabled, or a timeout this watchdog cannot keep. */
		bus_write_4(sc->ctl, SBSA_GWDT_WCS, 0);
		sc->wor = 0;
	} else if (wor == sc->wor) {
		if (sc->wor_refresh)
			sbsa_gwdt_set_wor(sc, wor);
		else
			bus_write_4(sc->refresh, SBSA_GWDT_WRR, 0);
		*error = 0;
	} else {
		sbsa_gwdt_set_wor(sc, wor);
		bus_write_4(sc->ctl, SBSA_GWDT_WCS, WCS_EN);
		sc->wor = wor;
		*error = 0;
	}
	mtx_unlock(&sc->mtx);
}

/*
 * Add a device for the GTDT's first non-secure watchdog.  Resources:
 * memory 0 is the refresh frame, memory 1 the control frame.
 */
static void
sbsa_gwdt_identify(driver_t *driver, device_t parent)
{
	ACPI_TABLE_GTDT *gtdt;
	ACPI_GTDT_HEADER *hdr;
	ACPI_GTDT_WATCHDOG *wd;
	vm_paddr_t physaddr;
	device_t dev;
	char *p, *end;
	u_int n;

	if (acpi_disabled("sbsa_gwdt") ||
	    device_find_child(parent, "sbsa_gwdt", DEVICE_UNIT_ANY) != NULL ||
	    (physaddr = acpi_find_table(ACPI_SIG_GTDT)) == 0 ||
	    (gtdt = acpi_map_table(physaddr, ACPI_SIG_GTDT)) == NULL)
		return;
	if (gtdt->Header.Revision < 2 || gtdt->PlatformTimerCount == 0)
		goto out;

	p = (char *)gtdt + gtdt->PlatformTimerOffset;
	end = (char *)gtdt + gtdt->Header.Length;
	for (n = 0; n < gtdt->PlatformTimerCount && p < end; n++,
	    p += hdr->Length) {
		hdr = (ACPI_GTDT_HEADER *)p;
		if (hdr->Length == 0)
			break;
		if (hdr->Type != ACPI_GTDT_TYPE_WATCHDOG)
			continue;
		wd = (ACPI_GTDT_WATCHDOG *)hdr;
		if ((wd->TimerFlags & ACPI_GTDT_WATCHDOG_SECURE) != 0)
			continue;
		dev = BUS_ADD_CHILD(parent, 0, "sbsa_gwdt", DEVICE_UNIT_ANY);
		if (dev == NULL)
			break;
		acpi_set_private(dev, (void *)1);
		bus_set_resource(dev, SYS_RES_MEMORY, 0,
		    wd->RefreshFrameAddress, SBSA_GWDT_SIZE);
		bus_set_resource(dev, SYS_RES_MEMORY, 1,
		    wd->ControlFrameAddress, SBSA_GWDT_SIZE);
		break;
	}
out:
	acpi_unmap_table(gtdt);
}

static int
sbsa_gwdt_probe(device_t dev)
{
	if (acpi_get_handle(dev) != NULL || acpi_get_private(dev) == NULL)
		return (ENXIO);
	device_set_desc(dev, "Arm SBSA generic watchdog");
	return (BUS_PROBE_NOWILDCARD);
}

static int
sbsa_gwdt_attach(device_t dev)
{
	struct sbsa_gwdt_softc *sc = device_get_softc(dev);
	uint32_t iidr, wcs;
	int rid;

	sc->dev = dev;
	rid = 0;
	sc->refresh = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	rid = 1;
	sc->ctl = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->refresh == NULL || sc->ctl == NULL) {
		device_printf(dev, "cannot allocate resources\n");
		goto fail;
	}
	sc->freq = READ_SPECIALREG(cntfrq_el0);
	if (sc->freq == 0) {
		device_printf(dev, "no system counter frequency\n");
		goto fail;
	}
	iidr = bus_read_4(sc->ctl, SBSA_GWDT_W_IIDR);
	sc->max_wor = W_IIDR_ARCH(iidr) >= 1 ? (1ul << 48) - 1 : UINT32_MAX;

	/*
	 * Firmware may leave it running, as UEFI does while it boots: stop
	 * it until watchdogd(8) or the kernel arms it.
	 */
	wcs = bus_read_4(sc->ctl, SBSA_GWDT_WCS);
	if ((wcs & WCS_EN) != 0) {
		device_printf(dev, "was enabled; disabling\n");
		bus_write_4(sc->ctl, SBSA_GWDT_WCS, 0);
	}

	sc->wor_refresh = !sbsa_gwdt_wrr_refreshes(sc);
	if (sc->wor_refresh)
		device_printf(dev, "refresh frame does not refresh; "
		    "refreshing through WOR\n");

	mtx_init(&sc->mtx, "sbsa_gwdt", NULL, MTX_DEF);
	sc->ev_tag = EVENTHANDLER_REGISTER(watchdog_list, sbsa_gwdt_fn, sc, 0);
	if (bootverbose)
		device_printf(dev, "architecture version %u, at most %ju s\n",
		    W_IIDR_ARCH(iidr), (uintmax_t)(sc->max_wor * 2 / sc->freq));
	return (0);
fail:
	if (sc->ctl != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 1, sc->ctl);
	if (sc->refresh != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->refresh);
	return (ENXIO);
}

static int
sbsa_gwdt_detach(device_t dev)
{
	struct sbsa_gwdt_softc *sc = device_get_softc(dev);

	EVENTHANDLER_DEREGISTER(watchdog_list, sc->ev_tag);
	bus_write_4(sc->ctl, SBSA_GWDT_WCS, 0);
	mtx_destroy(&sc->mtx);
	bus_release_resource(dev, SYS_RES_MEMORY, 1, sc->ctl);
	bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->refresh);
	return (0);
}

static device_method_t sbsa_gwdt_methods[] = {
	DEVMETHOD(device_identify,	sbsa_gwdt_identify),
	DEVMETHOD(device_probe,		sbsa_gwdt_probe),
	DEVMETHOD(device_attach,	sbsa_gwdt_attach),
	DEVMETHOD(device_detach,	sbsa_gwdt_detach),
	DEVMETHOD_END
};

static driver_t sbsa_gwdt_driver = {
	"sbsa_gwdt",
	sbsa_gwdt_methods,
	sizeof(struct sbsa_gwdt_softc),
};

DRIVER_MODULE(sbsa_gwdt, acpi, sbsa_gwdt_driver, 0, 0);
