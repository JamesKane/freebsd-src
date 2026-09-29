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
 * Qualcomm EPSS CPU frequency scaling.
 *
 * On Snapdragon SoCs with EPSS, each frequency domain (a cluster of cores)
 * has a register block holding a table of the performance levels firmware
 * set up, a register that selects the level for each core, and a register
 * reporting the level the domain actually runs at.  The hardware sets the
 * voltage and may run slower than requested when it limits power or
 * temperature.
 *
 * One cpufreq(4) driver attaches per domain, under the domain's first
 * core, and requests the same level for every core of the domain.
 *
 * With a devicetree, a core's "qcom,freq-domain" property points at its
 * domain's registers in a "qcom,cpufreq-epss" node.  Windows-on-Arm ACPI
 * tables describe neither, so under ACPI the domains of a known SoC,
 * recognized by the _HID of its power management device, come from a
 * table.  Firmware that provides _CPC should get a CPPC driver instead, so
 * a core with _CPC is left alone.
 */

#include "opt_acpi.h"
#include "opt_platform.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/cpu.h>
#include <sys/cpuset.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/pcpu.h>
#include <sys/rman.h>
#include <sys/smp.h>

#include <machine/bus.h>
#include <machine/cpu.h>
#include <machine/resource.h>

#ifdef DEV_ACPI
#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>
#endif

#ifdef FDT
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_subr.h>
#endif

#include "cpufreq_if.h"

#define	EPSS_ENABLE		0x000
#define	 EPSS_ENABLE_HW		(1u << 0)
#define	EPSS_DOMAIN_STATE	0x020
#define	 EPSS_STATE_LVAL_MASK	0xff		/* running L value */
#define	EPSS_DCVS_CTRL		0x0b0
#define	 EPSS_DCVS_PER_CORE	(1u << 0)
#define	EPSS_FREQ_LUT(i)	(0x100 + (i) * 4)
#define	 EPSS_LUT_SRC_SHIFT	30		/* 0: fixed PLL clock */
#define	 EPSS_LUT_CORES_SHIFT	16
#define	 EPSS_LUT_CORES_MASK	0x7
#define	 EPSS_LUT_CORES_TURBO	1
#define	 EPSS_LUT_LVAL_MASK	0xff
#define	EPSS_VOLT_LUT(i)	(0x200 + (i) * 4)
#define	 EPSS_VOLT_MV_MASK	0xfff
#define	EPSS_PERF_STATE(core)	(0x320 + (core) * 4)

#define	EPSS_SIZE		0x1000
#define	EPSS_LUT_MAX		40
#define	EPSS_MAX_CORES		8
#define	EPSS_XO_KHZ		19200	/* L values are multiples of this */
#define	EPSS_PLL_KHZ		300000	/* GPLL0 / 2, for LUT source 0 */

struct qcom_epss_domain {
	bus_addr_t	base;
	int		ncores;
	uint64_t	mpidr[EPSS_MAX_CORES];	/* in register order */
};

struct qcom_epss_level {
	int		khz;
	int		mv;
	int		index;			/* LUT row */
};

struct qcom_epss_softc {
	device_t		dev;
	struct resource		*mem;
	struct qcom_epss_domain	dom;
	bool			per_core;
	cpuset_t		cpus;
	struct qcom_epss_level	levels[EPSS_LUT_MAX];
	int			nlevels;
};

#ifdef DEV_ACPI
/* SC8280XP */
static const struct qcom_epss_domain qcom_epss_sc8280xp[] = {
	{ 0x18591000, 4, { 0x000, 0x100, 0x200, 0x300 } },
	{ 0x18592000, 4, { 0x400, 0x500, 0x600, 0x700 } },
};

static const struct {
	const char			*pep_hid;
	const struct qcom_epss_domain	*doms;
	int				ndoms;
} qcom_epss_acpi_socs[] = {
	{ "QCOM0617", qcom_epss_sc8280xp, nitems(qcom_epss_sc8280xp) },
};

static bool
qcom_epss_acpi_domain(device_t cpudev, struct qcom_epss_domain *dom,
    uint64_t *mpidr)
{
	ACPI_HANDLE cpu, h, pep;
	struct pcpu *pc;
	int c, d, s;

	cpu = acpi_get_handle(cpudev);
	if (cpu == NULL || acpi_disabled("qcom_epss") ||
	    ACPI_SUCCESS(AcpiGetHandle(cpu, "_CPC", &h)) ||
	    ACPI_FAILURE(AcpiGetHandle(NULL, "\\_SB.PEP0", &pep)))
		return (false);
	for (s = 0; s < nitems(qcom_epss_acpi_socs); s++)
		if (acpi_MatchHid(pep, qcom_epss_acpi_socs[s].pep_hid) ==
		    ACPI_MATCHHID_HID)
			break;
	if (s == nitems(qcom_epss_acpi_socs))
		return (false);

	/* acpi_cpu(4) records its device in the core's pcpu. */
	pc = NULL;
	CPU_FOREACH(c) {
		if (pcpu_find(c)->pc_device == cpudev) {
			pc = pcpu_find(c);
			break;
		}
	}
	if (pc == NULL)
		return (false);
	*mpidr = pc->pc_mpidr & CPU_AFF_MASK;
	for (d = 0; d < qcom_epss_acpi_socs[s].ndoms; d++) {
		*dom = qcom_epss_acpi_socs[s].doms[d];
		for (c = 0; c < dom->ncores; c++)
			if (dom->mpidr[c] == *mpidr)
				return (true);
	}
	return (false);
}
#endif

#ifdef FDT
/* A cpu node's "reg" is its MPIDR, in one or two cells. */
static bool
qcom_epss_fdt_mpidr(phandle_t node, uint64_t *mpidr)
{
	pcell_t reg[2];
	ssize_t len;

	len = OF_getencprop(node, "reg", reg, sizeof(reg));
	if (len == sizeof(reg))
		*mpidr = (uint64_t)reg[0] << 32 | reg[1];
	else if (len == sizeof(reg[0]))
		*mpidr = reg[0];
	else
		return (false);
	*mpidr &= CPU_AFF_MASK;
	return (true);
}

/*
 * The domain is reg entry N of the "qcom,cpufreq-epss" node named by
 * "qcom,freq-domain" = <&node N>; its cores are all those that name it.
 */
static bool
qcom_epss_fdt_domain(device_t cpudev, struct qcom_epss_domain *dom,
    uint64_t *mpidr)
{
	phandle_t cpu, cpus, epss, node;
	pcell_t fd[2], ofd[2];
	bus_size_t size;

	cpu = ofw_bus_get_node(cpudev);
	if (cpu <= 0 || OF_getencprop(cpu, "qcom,freq-domain", fd,
	    sizeof(fd)) != sizeof(fd) || !qcom_epss_fdt_mpidr(cpu, mpidr))
		return (false);
	epss = OF_node_from_xref(fd[0]);
	if (!ofw_bus_node_is_compatible(epss, "qcom,cpufreq-epss") ||
	    ofw_reg_to_paddr(epss, fd[1], &dom->base, &size, NULL) != 0)
		return (false);

	dom->ncores = 0;
	cpus = OF_parent(cpu);
	for (node = OF_child(cpus); node != 0; node = OF_peer(node)) {
		if (OF_getencprop(node, "qcom,freq-domain", ofd,
		    sizeof(ofd)) != sizeof(ofd) || ofd[0] != fd[0] ||
		    ofd[1] != fd[1])
			continue;
		if (dom->ncores == EPSS_MAX_CORES ||
		    !qcom_epss_fdt_mpidr(node, &dom->mpidr[dom->ncores]))
			return (false);
		dom->ncores++;
	}
	return (dom->ncores > 0);
}
#endif

/* Find the domain of the core that cpudev, a cpu(4) device, stands for. */
static bool
qcom_epss_domain(device_t cpudev, struct qcom_epss_domain *dom,
    uint64_t *mpidr)
{
#ifdef DEV_ACPI
	if (qcom_epss_acpi_domain(cpudev, dom, mpidr))
		return (true);
#endif
#ifdef FDT
	if (qcom_epss_fdt_domain(cpudev, dom, mpidr))
		return (true);
#endif
	return (false);
}

static void
qcom_epss_identify(driver_t *driver, device_t parent)
{
	struct qcom_epss_domain dom;
	device_t child;
	uint64_t mpidr;

	/* One driver per domain, under its first core. */
	if (device_find_child(parent, "qcom_epss", DEVICE_UNIT_ANY) != NULL ||
	    !qcom_epss_domain(parent, &dom, &mpidr) || dom.mpidr[0] != mpidr)
		return;
	child = BUS_ADD_CHILD(parent, 10, "qcom_epss", DEVICE_UNIT_ANY);
	if (child == NULL) {
		device_printf(parent, "cannot add qcom_epss\n");
		return;
	}
	/* acpi_cpu(4) allocates from a resource list; ofw_cpu passes up. */
	(void)bus_set_resource(child, SYS_RES_MEMORY, 0, dom.base, EPSS_SIZE);
}

static int
qcom_epss_probe(device_t dev)
{
	device_set_desc(dev, "Qualcomm EPSS CPU frequency control");
	return (BUS_PROBE_NOWILDCARD);
}

static const struct qcom_epss_level *
qcom_epss_find_khz(struct qcom_epss_softc *sc, int khz)
{
	const struct qcom_epss_level *best;
	int i;

	best = &sc->levels[0];
	for (i = 1; i < sc->nlevels; i++)
		if (abs(sc->levels[i].khz - khz) < abs(best->khz - khz))
			best = &sc->levels[i];
	return (best);
}

static void
qcom_epss_notify(struct qcom_epss_softc *sc, int khz)
{
	int cpu;

	CPU_FOREACH_ISSET(cpu, &sc->cpus)
		pcpu_find(cpu)->pc_clock = (uint64_t)khz * 1000;
}

static void
qcom_epss_to_setting(struct qcom_epss_softc *sc,
    const struct qcom_epss_level *l, struct cf_setting *set)
{
	memset(set, 0, sizeof(*set));
	set->freq = l->khz / 1000;
	set->volts = l->mv;
	set->power = CPUFREQ_VAL_UNKNOWN;
	set->lat = CPUFREQ_VAL_UNKNOWN;
	set->dev = sc->dev;
}

/* The level the domain actually runs at, which may be below the request. */
static int
qcom_epss_get(device_t dev, struct cf_setting *set)
{
	struct qcom_epss_softc *sc = device_get_softc(dev);
	const struct qcom_epss_level *l;
	uint32_t lval;

	if (set == NULL)
		return (EINVAL);
	lval = bus_read_4(sc->mem, EPSS_DOMAIN_STATE) & EPSS_STATE_LVAL_MASK;
	l = qcom_epss_find_khz(sc, lval * EPSS_XO_KHZ);
	qcom_epss_to_setting(sc, l, set);
	return (0);
}

static int
qcom_epss_set(device_t dev, const struct cf_setting *set)
{
	struct qcom_epss_softc *sc = device_get_softc(dev);
	const struct qcom_epss_level *l;
	int c;

	if (set == NULL)
		return (EINVAL);
	l = qcom_epss_find_khz(sc, set->freq * 1000);
	if (l->khz / 1000 != set->freq)
		return (EINVAL);
	for (c = 0; c < (sc->per_core ? sc->dom.ncores : 1); c++)
		bus_write_4(sc->mem, EPSS_PERF_STATE(c), l->index);
	qcom_epss_notify(sc, l->khz);
	return (0);
}

static int
qcom_epss_type(device_t dev, int *type)
{
	if (type == NULL)
		return (EINVAL);
	*type = CPUFREQ_TYPE_ABSOLUTE | CPUFREQ_FLAG_DOMAIN;
	return (0);
}

static int
qcom_epss_settings(device_t dev, struct cf_setting *sets, int *count)
{
	struct qcom_epss_softc *sc = device_get_softc(dev);
	int i;

	if (sets == NULL || count == NULL)
		return (EINVAL);
	if (*count < sc->nlevels) {
		*count = sc->nlevels;
		return (E2BIG);
	}
	for (i = 0; i < sc->nlevels; i++)
		qcom_epss_to_setting(sc, &sc->levels[i], &sets[i]);
	*count = sc->nlevels;
	return (0);
}

/*
 * Read the levels firmware set up.  The table ends where a row repeats
 * the previous one; rows marked as turbo are skipped.
 */
static void
qcom_epss_read_lut(struct qcom_epss_softc *sc)
{
	struct qcom_epss_level *l;
	uint32_t row;
	int i, khz, prev;

	prev = 0;
	for (i = 0; i < EPSS_LUT_MAX; i++) {
		row = bus_read_4(sc->mem, EPSS_FREQ_LUT(i));
		khz = (row >> EPSS_LUT_SRC_SHIFT) != 0 ?
		    (row & EPSS_LUT_LVAL_MASK) * EPSS_XO_KHZ : EPSS_PLL_KHZ;
		if (khz == prev)
			break;
		prev = khz;
		if (((row >> EPSS_LUT_CORES_SHIFT) & EPSS_LUT_CORES_MASK) ==
		    EPSS_LUT_CORES_TURBO)
			continue;
		l = &sc->levels[sc->nlevels++];
		l->khz = khz;
		l->mv = bus_read_4(sc->mem, EPSS_VOLT_LUT(i)) &
		    EPSS_VOLT_MV_MASK;
		l->index = i;
	}
}

static int
qcom_epss_attach(device_t dev)
{
	struct qcom_epss_softc *sc = device_get_softc(dev);
	struct cf_setting cur;
	struct pcpu *pc;
	uint64_t mpidr;
	int c, cpu, rid;

	sc->dev = dev;
	if (!qcom_epss_domain(device_get_parent(dev), &sc->dom, &mpidr))
		return (ENXIO);
	rid = 0;
	sc->mem = bus_alloc_resource(dev, SYS_RES_MEMORY, &rid, sc->dom.base,
	    sc->dom.base + EPSS_SIZE - 1, EPSS_SIZE, RF_ACTIVE);
	if (sc->mem == NULL) {
		device_printf(dev, "cannot map registers\n");
		return (ENXIO);
	}
	if ((bus_read_4(sc->mem, EPSS_ENABLE) & EPSS_ENABLE_HW) == 0) {
		device_printf(dev, "not enabled by firmware\n");
		goto fail;
	}
	sc->per_core = (bus_read_4(sc->mem, EPSS_DCVS_CTRL) &
	    EPSS_DCVS_PER_CORE) != 0;
	qcom_epss_read_lut(sc);
	if (sc->nlevels == 0) {
		device_printf(dev, "no performance levels\n");
		goto fail;
	}

	CPU_ZERO(&sc->cpus);
	CPU_FOREACH(cpu) {
		pc = pcpu_find(cpu);
		for (c = 0; c < sc->dom.ncores; c++)
			if ((pc->pc_mpidr & CPU_AFF_MASK) == sc->dom.mpidr[c])
				CPU_SET(cpu, &sc->cpus);
	}
	if (bootverbose)
		device_printf(dev, "%d levels, %d-%d MHz, %d cores%s\n",
		    sc->nlevels, sc->levels[0].khz / 1000,
		    sc->levels[sc->nlevels - 1].khz / 1000,
		    CPU_COUNT(&sc->cpus),
		    sc->per_core ? ", per-core requests" : "");

	qcom_epss_get(dev, &cur);
	qcom_epss_notify(sc, cur.freq * 1000);
	cpufreq_register(dev);
	return (0);
fail:
	bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem);
	return (ENXIO);
}

static int
qcom_epss_detach(device_t dev)
{
	struct qcom_epss_softc *sc = device_get_softc(dev);
	int error;

	error = cpufreq_unregister(dev);
	if (error != 0)
		return (error);
	bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->mem);
	return (0);
}

static device_method_t qcom_epss_methods[] = {
	DEVMETHOD(device_identify,	qcom_epss_identify),
	DEVMETHOD(device_probe,		qcom_epss_probe),
	DEVMETHOD(device_attach,	qcom_epss_attach),
	DEVMETHOD(device_detach,	qcom_epss_detach),

	DEVMETHOD(cpufreq_drv_get,	qcom_epss_get),
	DEVMETHOD(cpufreq_drv_set,	qcom_epss_set),
	DEVMETHOD(cpufreq_drv_type,	qcom_epss_type),
	DEVMETHOD(cpufreq_drv_settings,	qcom_epss_settings),

	DEVMETHOD_END
};

static driver_t qcom_epss_driver = {
	"qcom_epss",
	qcom_epss_methods,
	sizeof(struct qcom_epss_softc),
};

DRIVER_MODULE(qcom_epss, cpu, qcom_epss_driver, 0, 0);
MODULE_VERSION(qcom_epss, 1);
