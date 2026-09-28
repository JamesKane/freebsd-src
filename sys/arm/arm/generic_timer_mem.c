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
 * Arm memory-mapped generic timer.
 *
 * Besides each core's own timer, a system may have a memory-mapped timer
 * block: a control frame (CNTCTLBase) and one or more timer frames
 * (CNTBaseN), each with a physical timer compared against the system
 * counter.  Such a frame keeps running while cores are powered down, so
 * it can serve as a global event timer when the per-core timers stop in
 * deep idle states.
 *
 * Only ACPI (the GTDT's platform timers) is supported.
 */

#include "opt_acpi.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/timeet.h>

#include <machine/armreg.h>
#include <machine/bus.h>
#include <machine/resource.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

/* CNTCTLBase registers */
#define	CNTCTL_CNTTIDR		0x08
#define	 CNTTIDR_FRAME(n)	(0x1u << ((n) * 4))	/* frame exists */
#define	CNTCTL_CNTACR(n)	(0x40 + (n) * 4)
#define	 CNTACR_RPCT		(1u << 0)	/* read CNTPCT */
#define	 CNTACR_RFRQ		(1u << 2)	/* read CNTFRQ */
#define	 CNTACR_RWPT		(1u << 5)	/* use the physical timer */

/* CNTBaseN registers */
#define	CNTBASE_CNTFRQ		0x10
#define	CNTBASE_CNTP_TVAL	0x28
#define	CNTBASE_CNTP_CTL	0x2c		/* CNTP_CTL_* bits of armreg.h */

#define	GT_MEM_SIZE		0x1000

struct gt_mem_softc {
	struct resource		*ctl;
	struct resource		*frame;
	struct resource		*irq;
	void			*ih;
	struct eventtimer	et;
};

static int
gt_mem_start(struct eventtimer *et, sbintime_t first, sbintime_t period)
{
	struct gt_mem_softc *sc = et->et_priv;
	uint32_t counts;

	if (first == 0)
		return (EINVAL);
	counts = (et->et_frequency * first) >> 32;
	bus_write_4(sc->frame, CNTBASE_CNTP_TVAL, counts);
	bus_write_4(sc->frame, CNTBASE_CNTP_CTL, CNTP_CTL_ENABLE);
	return (0);
}

static int
gt_mem_stop(struct eventtimer *et)
{
	struct gt_mem_softc *sc = et->et_priv;

	bus_write_4(sc->frame, CNTBASE_CNTP_CTL, 0);
	return (0);
}

static int
gt_mem_intr(void *arg)
{
	struct gt_mem_softc *sc = arg;
	uint32_t ctl;

	ctl = bus_read_4(sc->frame, CNTBASE_CNTP_CTL);
	if ((ctl & CNTP_CTL_ISTATUS) == 0)
		return (FILTER_STRAY);
	bus_write_4(sc->frame, CNTBASE_CNTP_CTL, ctl | CNTP_CTL_IMASK);
	if (sc->et.et_active)
		sc->et.et_event_cb(&sc->et, sc->et.et_arg);
	return (FILTER_HANDLED);
}

/*
 * Add a device for the first non-secure, always-on timer frame of the
 * GTDT's timer blocks.  Resources: memory 0 is the block's CNTCTLBase,
 * memory 1 the frame's CNTBaseN; the frame number is the device's ivar.
 */
static void
gt_mem_acpi_identify(driver_t *driver, device_t parent)
{
	ACPI_TABLE_GTDT *gtdt;
	ACPI_GTDT_HEADER *hdr;
	ACPI_GTDT_TIMER_BLOCK *blk;
	ACPI_GTDT_TIMER_ENTRY *t;
	vm_paddr_t physaddr;
	device_t dev;
	char *p, *end;
	u_int i, n;

	if (acpi_disabled("gt_mem") ||
	    device_find_child(parent, "gt_mem", DEVICE_UNIT_ANY) != NULL ||
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
		if (hdr->Type != ACPI_GTDT_TYPE_TIMER_BLOCK)
			continue;
		blk = (ACPI_GTDT_TIMER_BLOCK *)hdr;
		t = (ACPI_GTDT_TIMER_ENTRY *)(p + blk->TimerOffset);
		for (i = 0; i < blk->TimerCount; i++, t++) {
			/*
			 * The ACPI bus maps a device's interrupts as
			 * level-triggered and active-high, so only such a
			 * timer can be used.
			 */
			if ((t->CommonFlags & ACPI_GTDT_GT_IS_SECURE_TIMER) ||
			    (t->CommonFlags & ACPI_GTDT_GT_ALWAYS_ON) == 0 ||
			    t->TimerInterrupt == 0 ||
			    (t->TimerFlags & (ACPI_GTDT_GT_IRQ_MODE |
			    ACPI_GTDT_GT_IRQ_POLARITY)) != 0)
				continue;
			dev = BUS_ADD_CHILD(parent, 0, "gt_mem",
			    DEVICE_UNIT_ANY);
			if (dev == NULL)
				goto out;
			acpi_set_private(dev,
			    (void *)(uintptr_t)(t->FrameNumber + 1));
			bus_set_resource(dev, SYS_RES_MEMORY, 0,
			    blk->BlockAddress, GT_MEM_SIZE);
			bus_set_resource(dev, SYS_RES_MEMORY, 1,
			    t->BaseAddress, GT_MEM_SIZE);
			bus_set_resource(dev, SYS_RES_IRQ, 0,
			    t->TimerInterrupt, 1);
			goto out;
		}
	}
out:
	acpi_unmap_table(gtdt);
}

static int
gt_mem_acpi_probe(device_t dev)
{
	if (acpi_get_handle(dev) != NULL || acpi_get_private(dev) == NULL)
		return (ENXIO);
	device_set_desc(dev, "ARM memory-mapped generic timer");
	return (BUS_PROBE_NOWILDCARD);
}

static int
gt_mem_acpi_attach(device_t dev)
{
	struct gt_mem_softc *sc = device_get_softc(dev);
	uint32_t acr, freq;
	int frame, rid;

	frame = (uintptr_t)acpi_get_private(dev) - 1;
	rid = 0;
	sc->ctl = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	rid = 1;
	sc->frame = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	rid = 0;
	sc->irq = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid, RF_ACTIVE);
	if (sc->ctl == NULL || sc->frame == NULL || sc->irq == NULL) {
		device_printf(dev, "cannot allocate resources\n");
		goto fail;
	}

	if ((bus_read_4(sc->ctl, CNTCTL_CNTTIDR) & CNTTIDR_FRAME(frame)) == 0) {
		device_printf(dev, "frame %d is not implemented\n", frame);
		goto fail;
	}
	/* Allow non-secure use of the frame's counter and physical timer. */
	acr = bus_read_4(sc->ctl, CNTCTL_CNTACR(frame));
	bus_write_4(sc->ctl, CNTCTL_CNTACR(frame),
	    acr | CNTACR_RPCT | CNTACR_RFRQ | CNTACR_RWPT);
	acr = bus_read_4(sc->ctl, CNTCTL_CNTACR(frame));
	if ((acr & CNTACR_RWPT) == 0) {
		device_printf(dev, "frame %d physical timer is not accessible\n",
		    frame);
		goto fail;
	}
	bus_write_4(sc->frame, CNTBASE_CNTP_CTL, 0);

	freq = bus_read_4(sc->frame, CNTBASE_CNTFRQ);
	if (freq == 0)
		freq = READ_SPECIALREG(cntfrq_el0);
	if (freq == 0) {
		device_printf(dev, "no counter frequency\n");
		goto fail;
	}

	if (bus_setup_intr(dev, sc->irq, INTR_TYPE_CLK, gt_mem_intr, NULL, sc,
	    &sc->ih) != 0) {
		device_printf(dev, "cannot set up interrupt\n");
		goto fail;
	}
	/*
	 * Take the interrupt on CPU 0: a core powered down in a deep idle
	 * state may not be woken by it, so CPU 0 stays out of such states
	 * while this timer is in use.
	 */
	if (bus_bind_intr(dev, sc->irq, 0) != 0)
		device_printf(dev, "cannot bind interrupt to CPU 0\n");

	/*
	 * A global timer: interrupts go to one CPU, which wakes the others
	 * with IPIs.  Rank it below the per-core timers; select it with
	 * kern.eventtimer.timer to allow idle states that stop those.
	 */
	sc->et.et_name = "ARM MMIO Timer";
	sc->et.et_flags = ET_FLAGS_ONESHOT;
	sc->et.et_quality = 500;
	sc->et.et_frequency = freq;
	sc->et.et_min_period = (0x00000010LLU << 32) / freq;
	sc->et.et_max_period = (0x7fffffffLLU << 32) / freq;
	sc->et.et_start = gt_mem_start;
	sc->et.et_stop = gt_mem_stop;
	sc->et.et_priv = sc;
	et_register(&sc->et);
	if (bootverbose)
		device_printf(dev, "frame %d, %u Hz\n", frame, freq);
	return (0);
fail:
	if (sc->irq != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq);
	if (sc->frame != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 1, sc->frame);
	if (sc->ctl != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->ctl);
	return (ENXIO);
}

static device_method_t gt_mem_acpi_methods[] = {
	DEVMETHOD(device_identify,	gt_mem_acpi_identify),
	DEVMETHOD(device_probe,		gt_mem_acpi_probe),
	DEVMETHOD(device_attach,	gt_mem_acpi_attach),
	DEVMETHOD_END
};

static driver_t gt_mem_acpi_driver = {
	"gt_mem",
	gt_mem_acpi_methods,
	sizeof(struct gt_mem_softc),
};

DRIVER_MODULE(gt_mem, acpi, gt_mem_acpi_driver, 0, 0);
