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
 * CIX Sky1 general purpose timer (ACPI CIXH1007).
 *
 * Sky1's cores' generic timers stop while the cores are powered down in
 * deep idle states, and its firmware describes no memory-mapped generic
 * timer.  This timer keeps running: two 32-bit timers, an even and an odd
 * one 0x1000 apart, chained into a 64-bit free-running counter with a
 * 64-bit compare, serve as a global one-shot event timer.
 *
 * The firmware leaves the timer's clock running; its rate is not described
 * in ACPI except through CIX's clock methods, so it is checked against the
 * system counter.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/timeet.h>

#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/resource.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

/* Registers of each timer; the odd timer's are at SKY1_GPT_ODD. */
#define	SKY1_GPT_COUNT		0x1c	/* free-running count */
#define	SKY1_GPT_COMP		0x20	/* free-running compare */
#define	SKY1_GPT_CTL		0x24
#define	 GPT_CTL_ENABLE		(1u << 17)
#define	 GPT_CTL_FREE_ENABLE	(1u << 15)
#define	 GPT_CTL_64BIT		(1u << 12)	/* odd: chain to the even */
#define	 GPT_CTL_INTR_ENABLE	(1u << 6)
#define	 GPT_CTL_MODE_FREERUN	0x2
#define	SKY1_GPT_INTR_STATUS	0x30
#define	SKY1_GPT_INTR_CLEAR	0x34
#define	 GPT_INTR_FREE		(1u << 5)	/* free-running compare */
#define	SKY1_GPT_ODD		0x1000

#define	SKY1_GPT_FREQ		25000000

struct sky1_gpt_softc {
	device_t		dev;
	struct resource		*mem;
	struct resource		*irq;
	void			*ih;
	bool			armed;	/* the odd timer's interrupt is on */
	struct eventtimer	et;
};

static uint64_t
sky1_gpt_count(struct sky1_gpt_softc *sc)
{
	uint32_t hi, lo, hi2;

	hi = bus_read_4(sc->mem, SKY1_GPT_ODD + SKY1_GPT_COUNT);
	for (;;) {
		lo = bus_read_4(sc->mem, SKY1_GPT_COUNT);
		hi2 = bus_read_4(sc->mem, SKY1_GPT_ODD + SKY1_GPT_COUNT);
		if (hi2 == hi)
			return ((uint64_t)hi << 32 | lo);
		hi = hi2;
	}
}

static void
sky1_gpt_set_compare(struct sky1_gpt_softc *sc, uint64_t comp)
{
	bus_write_4(sc->mem, SKY1_GPT_ODD + SKY1_GPT_COMP, comp >> 32);
	bus_write_4(sc->mem, SKY1_GPT_COMP, (uint32_t)comp);
}

static void
sky1_gpt_intr_enable(struct sky1_gpt_softc *sc, bool on)
{
	uint32_t ctl;

	ctl = bus_read_4(sc->mem, SKY1_GPT_ODD + SKY1_GPT_CTL);
	if (on)
		ctl |= GPT_CTL_INTR_ENABLE;
	else
		ctl &= ~GPT_CTL_INTR_ENABLE;
	bus_write_4(sc->mem, SKY1_GPT_ODD + SKY1_GPT_CTL, ctl);
}

static void
sky1_gpt_disarm(struct sky1_gpt_softc *sc)
{
	sc->armed = false;
	sky1_gpt_intr_enable(sc, false);
	bus_write_4(sc->mem, SKY1_GPT_ODD + SKY1_GPT_INTR_CLEAR,
	    GPT_INTR_FREE);
}

static int
sky1_gpt_start(struct eventtimer *et, sbintime_t first, sbintime_t period)
{
	struct sky1_gpt_softc *sc = et->et_priv;
	uint64_t comp, counts;

	if (first == 0)
		return (EINVAL);
	counts = (et->et_frequency * first) >> 32;
	/*
	 * Turn the interrupt on before setting the compare, so no match is
	 * missed; the previous compare is already behind the count.
	 */
	if (!sc->armed) {
		sky1_gpt_intr_enable(sc, true);
		sc->armed = true;
	}
	/*
	 * The compare may only match the count exactly: should the count
	 * pass it before it is set, set it further ahead.
	 */
	for (;;) {
		comp = sky1_gpt_count(sc) + counts;
		sky1_gpt_set_compare(sc, comp);
		if (sky1_gpt_count(sc) < comp)
			break;
		counts *= 2;
	}
	return (0);
}

static int
sky1_gpt_stop(struct eventtimer *et)
{
	sky1_gpt_disarm(et->et_priv);
	return (0);
}

static int
sky1_gpt_intr(void *arg)
{
	struct sky1_gpt_softc *sc = arg;

	if ((bus_read_4(sc->mem, SKY1_GPT_ODD + SKY1_GPT_INTR_STATUS) &
	    GPT_INTR_FREE) == 0)
		return (FILTER_STRAY);
	/*
	 * Only acknowledge: another CPU may already have set the next
	 * compare, and the compare matches once, so it stays armed.  Ack
	 * before the callback, which handles every event due by then.
	 */
	bus_write_4(sc->mem, SKY1_GPT_ODD + SKY1_GPT_INTR_CLEAR,
	    GPT_INTR_FREE);
	if (sc->et.et_active)
		sc->et.et_event_cb(&sc->et, sc->et.et_arg);
	return (FILTER_HANDLED);
}

static int
sky1_gpt_probe(device_t dev)
{
	static char *ids[] = { "CIXH1007", NULL };
	int rv;

	if (acpi_disabled("sky1_gpt"))
		return (ENXIO);
	rv = ACPI_ID_PROBE(device_get_parent(dev), dev, ids, NULL);
	if (rv <= 0)
		device_set_desc(dev, "CIX Sky1 general purpose timer");
	return (rv);
}

static int
sky1_gpt_attach(device_t dev)
{
	struct sky1_gpt_softc *sc = device_get_softc(dev);
	uint64_t c0, c1, t0, t1, freq;
	int rid;

	sc->dev = dev;
	rid = 0;
	sc->mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	rid = 0;
	sc->irq = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid, RF_ACTIVE);
	if (sc->mem == NULL || sc->irq == NULL) {
		device_printf(dev, "cannot allocate resources\n");
		goto fail;
	}

	/*
	 * Run both timers free, the odd one chained above the even one, with
	 * the enables set last.  The even timer's interrupt enable stays set;
	 * the odd one's arms the compare.
	 */
	bus_write_4(sc->mem, SKY1_GPT_CTL,
	    GPT_CTL_FREE_ENABLE | GPT_CTL_MODE_FREERUN | GPT_CTL_INTR_ENABLE);
	bus_write_4(sc->mem, SKY1_GPT_ODD + SKY1_GPT_CTL,
	    GPT_CTL_FREE_ENABLE | GPT_CTL_64BIT | GPT_CTL_MODE_FREERUN);
	bus_write_4(sc->mem, SKY1_GPT_CTL,
	    GPT_CTL_ENABLE | GPT_CTL_FREE_ENABLE | GPT_CTL_MODE_FREERUN |
	    GPT_CTL_INTR_ENABLE);
	bus_write_4(sc->mem, SKY1_GPT_ODD + SKY1_GPT_CTL,
	    GPT_CTL_ENABLE | GPT_CTL_FREE_ENABLE | GPT_CTL_64BIT |
	    GPT_CTL_MODE_FREERUN);
	sky1_gpt_disarm(sc);

	/* Check the rate against the system counter, over a millisecond. */
	t0 = READ_SPECIALREG(cntvct_el0);
	c0 = sky1_gpt_count(sc);
	DELAY(1000);
	t1 = READ_SPECIALREG(cntvct_el0);
	c1 = sky1_gpt_count(sc);
	if (t1 == t0 || c1 == c0) {
		device_printf(dev, "the counter is not running\n");
		goto fail;
	}
	freq = (c1 - c0) * READ_SPECIALREG(cntfrq_el0) / (t1 - t0);
	if (freq < SKY1_GPT_FREQ / 100 * 98 ||
	    freq > SKY1_GPT_FREQ / 100 * 102) {
		device_printf(dev, "counts at %ju Hz, not %u\n",
		    (uintmax_t)freq, SKY1_GPT_FREQ);
		goto fail;
	}

	if (bus_setup_intr(dev, sc->irq, INTR_TYPE_CLK, sky1_gpt_intr, NULL,
	    sc, &sc->ih) != 0) {
		device_printf(dev, "cannot set up interrupt\n");
		goto fail;
	}
	/*
	 * Take the interrupt on CPU 0, which stays out of states that power
	 * it down while a global timer is in use: such a state may not be
	 * woken by it.
	 */
	if (bus_bind_intr(dev, sc->irq, 0) != 0)
		device_printf(dev, "cannot bind interrupt to CPU 0\n");

	/*
	 * A global timer, as gt_mem: ranked below the per-core timers, and
	 * selected with kern.eventtimer.timer to allow idle states that stop
	 * them.
	 */
	sc->et.et_name = "Sky1 GPT";
	sc->et.et_flags = ET_FLAGS_ONESHOT;
	sc->et.et_quality = 500;
	sc->et.et_frequency = SKY1_GPT_FREQ;
	sc->et.et_min_period = (0x40LLU << 32) / SKY1_GPT_FREQ;
	sc->et.et_max_period = (0x7fffffffLLU << 32) / SKY1_GPT_FREQ;
	sc->et.et_start = sky1_gpt_start;
	sc->et.et_stop = sky1_gpt_stop;
	sc->et.et_priv = sc;
	et_register(&sc->et);
	return (0);
fail:
	if (sc->irq != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, 0, sc->irq);
	if (sc->mem != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem);
	return (ENXIO);
}

static device_method_t sky1_gpt_methods[] = {
	DEVMETHOD(device_probe,		sky1_gpt_probe),
	DEVMETHOD(device_attach,	sky1_gpt_attach),
	DEVMETHOD_END
};

static driver_t sky1_gpt_driver = {
	"sky1_gpt",
	sky1_gpt_methods,
	sizeof(struct sky1_gpt_softc),
};

DRIVER_MODULE(sky1_gpt, acpi, sky1_gpt_driver, 0, 0);
