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
 * ACPI Collaborative Processor Performance Control (CPPC, ACPI 6.5 8.4.6).
 *
 * A processor's _CPC describes an abstract performance scale and the
 * registers through which the OS asks for a performance level and reads
 * what it got.  Without _PSS, this is how firmware lets the OS choose CPU
 * frequency, as on many Arm servers and boards.
 *
 * One cpufreq(4) driver attaches per performance domain (_PSD), under the
 * domain's first processor, and offers levels 100 MHz apart, mapped onto
 * the performance scale through _CPC's lowest and nominal frequencies.
 * Setting a level writes the desired performance register of each
 * processor of the domain.  dev.acpi_cppc.N.delivered_mhz reads the
 * domain's actual frequency from the feedback counters.
 *
 * Registers in system memory are supported, and on arm64 the functional
 * fixed hardware counters (the activity monitors' core and constant
 * cycles), as Linux defines them; PCC registers are not yet.  Autonomous
 * selection is left as the firmware set it.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/cpu.h>
#include <sys/cpuset.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/pcpu.h>
#include <sys/proc.h>
#include <sys/sched.h>
#include <sys/smp.h>
#include <sys/sysctl.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/bus.h>
#include <machine/cpu.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include "cpufreq_if.h"

/* _CPC package entries (revision 3). */
#define	CPC_NUM_ENTRIES		0
#define	CPC_REVISION		1
#define	CPC_HIGHEST_PERF	2
#define	CPC_NOMINAL_PERF	3
#define	CPC_LOWEST_NL_PERF	4
#define	CPC_LOWEST_PERF		5
#define	CPC_DESIRED_PERF	7
#define	CPC_REFERENCE_CTR	13
#define	CPC_DELIVERED_CTR	14
#define	CPC_REFERENCE_PERF	20
#define	CPC_LOWEST_FREQ		21
#define	CPC_NOMINAL_FREQ	22
#define	CPC_ENTRIES_V3		23

/* _PSD package entries. */
#define	PSD_DOMAIN		2
#define	PSD_ENTRIES		5

/* Arm functional fixed hardware registers: activity monitor counters. */
#define	CPPC_FFH_CORE_CYCLES	0
#define	CPPC_FFH_CONST_CYCLES	1

#define	CPPC_STEP_MHZ		100
#define	CPPC_MAX_LEVELS		64

struct acpi_cppc_reg {
	int		 space;		/* ACPI_ADR_SPACE_*; -1: none */
	uint64_t	 addr;
	int		 width;
	int		 offset;
	vm_offset_t	 va;		/* system memory: mapped page */
};

struct acpi_cppc_softc {
	device_t		 dev;
	cpuset_t		 cpus;
	int			 head;		/* cpuid the driver is under */
	struct acpi_cppc_reg	*desired;	/* one per distinct register */
	int			 ndesired;
	struct acpi_cppc_reg	 ref_ctr;
	struct acpi_cppc_reg	 del_ctr;
	uint32_t		 highest;
	uint32_t		 nominal;
	uint32_t		 lowest;
	uint32_t		 ref_perf;
	uint32_t		 lowest_mhz;
	uint32_t		 nominal_mhz;
	int			 levels[CPPC_MAX_LEVELS];	/* MHz */
	int			 nlevels;
	int			 cur_mhz;
};

static MALLOC_DEFINE(M_ACPI_CPPC, "acpi_cppc", "ACPI CPPC");

static int
acpi_cppc_domain(ACPI_HANDLE h)
{
	ACPI_BUFFER buf;
	ACPI_OBJECT *pkg;
	int domain;

	domain = -1;
	buf.Pointer = NULL;
	buf.Length = ACPI_ALLOCATE_BUFFER;
	if (ACPI_FAILURE(AcpiEvaluateObject(h, "_PSD", NULL, &buf)))
		return (-1);
	pkg = buf.Pointer;
	if (ACPI_PKG_VALID(pkg, 1) &&
	    ACPI_PKG_VALID(&pkg->Package.Elements[0], PSD_ENTRIES))
		(void)acpi_PkgInt32(&pkg->Package.Elements[0], PSD_DOMAIN,
		    (uint32_t *)&domain);
	AcpiOsFree(buf.Pointer);
	return (domain);
}

static ACPI_HANDLE
acpi_cppc_cpu_handle(int cpu)
{
	device_t dev;

	dev = pcpu_find(cpu)->pc_device;
	return (dev != NULL ? acpi_get_handle(dev) : NULL);
}

/* Whether a processor has _CPC, and its frequency is not left to _PSS. */
static bool
acpi_cppc_present(ACPI_HANDLE h)
{
	ACPI_HANDLE tmp;

	return (h != NULL &&
	    ACPI_SUCCESS(AcpiGetHandle(h, "_CPC", &tmp)) &&
	    ACPI_FAILURE(AcpiGetHandle(h, "_PSS", &tmp)));
}

/*
 * The processors of the domain of the processor cpu (one without _PSD has
 * its own), and whether cpu is the domain's first.
 */
static bool
acpi_cppc_domain_cpus(int cpu, cpuset_t *cpus)
{
	ACPI_HANDLE h;
	int domain, i;

	CPU_ZERO(cpus);
	CPU_SET(cpu, cpus);
	domain = acpi_cppc_domain(acpi_cppc_cpu_handle(cpu));
	if (domain < 0)
		return (true);
	CPU_FOREACH(i) {
		if (i == cpu)
			continue;
		h = acpi_cppc_cpu_handle(i);
		if (acpi_cppc_present(h) && acpi_cppc_domain(h) == domain)
			CPU_SET(i, cpus);
	}
	return (CPU_FFS(cpus) - 1 == cpu);
}

static void
acpi_cppc_identify(driver_t *driver, device_t parent)
{
	cpuset_t cpus;
	int cpu;

	if (device_find_child(parent, "acpi_cppc", DEVICE_UNIT_ANY) != NULL ||
	    !acpi_cppc_present(acpi_get_handle(parent)))
		return;
	/* The processor's own id: its pcpu names it as the device. */
	CPU_FOREACH(cpu)
		if (pcpu_find(cpu)->pc_device == parent)
			break;
	if (cpu > mp_maxid || !acpi_cppc_domain_cpus(cpu, &cpus))
		return;
	if (BUS_ADD_CHILD(parent, 10, "acpi_cppc", DEVICE_UNIT_ANY) == NULL)
		device_printf(parent, "cannot add acpi_cppc\n");
}

static int
acpi_cppc_probe(device_t dev)
{
	if (resource_disabled("acpi_cppc", 0))
		return (ENXIO);
	device_set_desc(dev, "ACPI CPPC CPU frequency control");
	return (BUS_PROBE_NOWILDCARD);
}

/*
 * Read a _CPC entry that is a register, from its Generic Register
 * descriptor.  A system memory register at address 0 is an absent one.
 */
static int
acpi_cppc_get_reg(ACPI_OBJECT *pkg, int idx, struct acpi_cppc_reg *reg)
{
	ACPI_GENERIC_ADDRESS gas;
	ACPI_OBJECT *obj;

	reg->space = -1;
	reg->va = 0;
	obj = &pkg->Package.Elements[idx];
	if (obj->Type != ACPI_TYPE_BUFFER ||
	    obj->Buffer.Length < sizeof(gas) + 3)
		return (EINVAL);
	memcpy(&gas, obj->Buffer.Pointer + 3, sizeof(gas));
	if (gas.SpaceId == ACPI_ADR_SPACE_SYSTEM_MEMORY && gas.Address == 0)
		return (0);
	reg->space = gas.SpaceId;
	reg->addr = gas.Address;
	reg->width = gas.BitWidth;
	reg->offset = gas.BitOffset;
	return (0);
}

static int
acpi_cppc_map_reg(struct acpi_cppc_reg *reg)
{
	if (reg->space != ACPI_ADR_SPACE_SYSTEM_MEMORY)
		return (0);
	if ((reg->width != 32 && reg->width != 64) || reg->offset != 0 ||
	    (reg->addr & (reg->width / 8 - 1)) != 0)
		return (EOPNOTSUPP);
	reg->va = (vm_offset_t)pmap_mapdev(trunc_page(reg->addr), PAGE_SIZE) +
	    (reg->addr & PAGE_MASK);
	return (0);
}

static void
acpi_cppc_unmap_reg(struct acpi_cppc_reg *reg)
{
	if (reg->va != 0)
		pmap_unmapdev((void *)trunc_page(reg->va), PAGE_SIZE);
	reg->va = 0;
}

/* Read a register; functional fixed hardware ones on the current CPU. */
static int
acpi_cppc_read(const struct acpi_cppc_reg *reg, uint64_t *val)
{
	switch (reg->space) {
	case ACPI_ADR_SPACE_SYSTEM_MEMORY:
		if (reg->va == 0)
			return (ENXIO);
		*val = reg->width == 64 ? *(volatile uint64_t *)reg->va :
		    *(volatile uint32_t *)reg->va;
		return (0);
#ifdef __aarch64__
	case ACPI_ADR_SPACE_FIXED_HARDWARE:
		switch (reg->addr) {
		case CPPC_FFH_CORE_CYCLES:
			*val = READ_SPECIALREG(S3_3_C13_C4_0); /* AMEVCNTR00 */
			return (0);
		case CPPC_FFH_CONST_CYCLES:
			*val = READ_SPECIALREG(S3_3_C13_C4_1); /* AMEVCNTR01 */
			return (0);
		}
		return (EOPNOTSUPP);
#endif
	}
	return (EOPNOTSUPP);
}

static int
acpi_cppc_write(const struct acpi_cppc_reg *reg, uint64_t val)
{
	if (reg->space != ACPI_ADR_SPACE_SYSTEM_MEMORY || reg->va == 0)
		return (EOPNOTSUPP);
	if (reg->width == 64)
		*(volatile uint64_t *)reg->va = val;
	else
		*(volatile uint32_t *)reg->va = val;
	return (0);
}

/* A _CPC entry that is an integer, or a register to read once. */
static int
acpi_cppc_get_value(ACPI_OBJECT *pkg, int idx, uint32_t *val)
{
	struct acpi_cppc_reg reg;
	uint64_t v;
	int error;

	if (acpi_PkgInt32(pkg, idx, val) == 0)
		return (0);
	if ((error = acpi_cppc_get_reg(pkg, idx, &reg)) != 0)
		return (error);
	if (reg.space != ACPI_ADR_SPACE_SYSTEM_MEMORY)
		return (EOPNOTSUPP);
	if ((error = acpi_cppc_map_reg(&reg)) != 0)
		return (error);
	error = acpi_cppc_read(&reg, &v);
	acpi_cppc_unmap_reg(&reg);
	*val = v;
	return (error);
}

/* Map performance to frequency and back, through the lowest and nominal. */
static int
acpi_cppc_perf_to_mhz(struct acpi_cppc_softc *sc, uint64_t perf)
{
	if (sc->nominal == sc->lowest)
		return (perf * sc->nominal_mhz / sc->nominal);
	return (sc->lowest_mhz + ((int64_t)perf - sc->lowest) *
	    (int)(sc->nominal_mhz - sc->lowest_mhz) /
	    (int)(sc->nominal - sc->lowest));
}

static uint32_t
acpi_cppc_mhz_to_perf(struct acpi_cppc_softc *sc, int mhz)
{
	int64_t perf;

	if (sc->nominal == sc->lowest)
		perf = (int64_t)mhz * sc->nominal / sc->nominal_mhz;
	else
		perf = sc->lowest + ((int64_t)mhz - (int)sc->lowest_mhz) *
		    (int)(sc->nominal - sc->lowest) /
		    (int)(sc->nominal_mhz - sc->lowest_mhz);
	return (MAX(sc->lowest, MIN(sc->highest, perf)));
}

static void
acpi_cppc_to_setting(struct acpi_cppc_softc *sc, int mhz,
    struct cf_setting *set)
{
	memset(set, 0, sizeof(*set));
	set->freq = mhz;
	set->volts = CPUFREQ_VAL_UNKNOWN;
	set->power = CPUFREQ_VAL_UNKNOWN;
	set->lat = CPUFREQ_VAL_UNKNOWN;
	set->dev = sc->dev;
}

static void
acpi_cppc_notify(struct acpi_cppc_softc *sc, int mhz)
{
	int cpu;

	CPU_FOREACH_ISSET(cpu, &sc->cpus)
		pcpu_find(cpu)->pc_clock = (uint64_t)mhz * 1000000;
}

static int
acpi_cppc_get(device_t dev, struct cf_setting *set)
{
	struct acpi_cppc_softc *sc = device_get_softc(dev);

	if (set == NULL)
		return (EINVAL);
	acpi_cppc_to_setting(sc, sc->cur_mhz, set);
	return (0);
}

static int
acpi_cppc_set(device_t dev, const struct cf_setting *set)
{
	struct acpi_cppc_softc *sc = device_get_softc(dev);
	uint32_t perf;
	int error, i;

	if (set == NULL)
		return (EINVAL);
	for (i = 0; i < sc->nlevels; i++)
		if (sc->levels[i] == set->freq)
			break;
	if (i == sc->nlevels)
		return (EINVAL);
	perf = acpi_cppc_mhz_to_perf(sc, set->freq);
	for (i = 0; i < sc->ndesired; i++)
		if ((error = acpi_cppc_write(&sc->desired[i], perf)) != 0)
			return (error);
	sc->cur_mhz = set->freq;
	acpi_cppc_notify(sc, sc->cur_mhz);
	return (0);
}

static int
acpi_cppc_type(device_t dev, int *type)
{
	if (type == NULL)
		return (EINVAL);
	*type = CPUFREQ_TYPE_ABSOLUTE | CPUFREQ_FLAG_DOMAIN;
	return (0);
}

static int
acpi_cppc_settings(device_t dev, struct cf_setting *sets, int *count)
{
	struct acpi_cppc_softc *sc = device_get_softc(dev);
	int i;

	if (sets == NULL || count == NULL)
		return (EINVAL);
	if (*count < sc->nlevels) {
		*count = sc->nlevels;
		return (E2BIG);
	}
	for (i = 0; i < sc->nlevels; i++)
		acpi_cppc_to_setting(sc, sc->levels[i], &sets[i]);
	*count = sc->nlevels;
	return (0);
}

/*
 * Keep the processor busy for usec: DELAY() may idle it (WFET), and the
 * delivered counter only counts while it runs.
 */
static void
acpi_cppc_spin(int usec)
{
	sbintime_t end;

	end = sbinuptime() + ustosbt(usec);
	while (sbinuptime() < end)
		cpu_spinwait();
}

/*
 * The domain's frequency as the feedback counters see it over 10 ms of
 * the first processor kept busy: the reference counter counts at the
 * reference performance, the delivered counter at the delivered one.
 */
static int
acpi_cppc_delivered_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct acpi_cppc_softc *sc = arg1;
	uint64_t d0, d1, r0, r1;
	int error, mhz;

	if (sc->ref_ctr.space < 0 || sc->del_ctr.space < 0)
		return (EOPNOTSUPP);
	thread_lock(curthread);
	sched_bind(curthread, sc->head);
	thread_unlock(curthread);
	error = acpi_cppc_read(&sc->ref_ctr, &r0);
	if (error == 0)
		error = acpi_cppc_read(&sc->del_ctr, &d0);
	acpi_cppc_spin(10000);
	if (error == 0)
		error = acpi_cppc_read(&sc->ref_ctr, &r1);
	if (error == 0)
		error = acpi_cppc_read(&sc->del_ctr, &d1);
	thread_lock(curthread);
	sched_unbind(curthread);
	thread_unlock(curthread);
	if (error != 0)
		return (error);
	if (r1 == r0)
		mhz = 0;
#ifdef __aarch64__
	/*
	 * The constant cycle counter counts at the system counter's rate,
	 * which gives the frequency directly; firmware's reference
	 * performance need not match it (Sky1's is 1000, on a scale where
	 * 1 GHz is 3150).
	 */
	else if (sc->ref_ctr.space == ACPI_ADR_SPACE_FIXED_HARDWARE &&
	    sc->ref_ctr.addr == CPPC_FFH_CONST_CYCLES)
		mhz = (d1 - d0) * (READ_SPECIALREG(cntfrq_el0) / 1000) /
		    (r1 - r0) / 1000;
#endif
	else
		mhz = acpi_cppc_perf_to_mhz(sc,
		    (d1 - d0) * sc->ref_perf / (r1 - r0));
	return (sysctl_handle_int(oidp, &mhz, 0, req));
}

/* Add the desired performance register of the processor cpu, once. */
static int
acpi_cppc_add_desired(struct acpi_cppc_softc *sc, int cpu)
{
	ACPI_BUFFER buf;
	ACPI_OBJECT *pkg;
	struct acpi_cppc_reg reg;
	int error, i;

	buf.Pointer = NULL;
	buf.Length = ACPI_ALLOCATE_BUFFER;
	if (ACPI_FAILURE(AcpiEvaluateObject(acpi_cppc_cpu_handle(cpu), "_CPC",
	    NULL, &buf)))
		return (ENXIO);
	pkg = buf.Pointer;
	error = ACPI_PKG_VALID(pkg, CPC_DESIRED_PERF + 1) ?
	    acpi_cppc_get_reg(pkg, CPC_DESIRED_PERF, &reg) : EINVAL;
	AcpiOsFree(buf.Pointer);
	if (error != 0)
		return (error);
	if (reg.space != ACPI_ADR_SPACE_SYSTEM_MEMORY)
		return (EOPNOTSUPP);
	for (i = 0; i < sc->ndesired; i++)
		if (sc->desired[i].addr == reg.addr)
			return (0);
	if ((error = acpi_cppc_map_reg(&reg)) != 0)
		return (error);
	sc->desired[sc->ndesired++] = reg;
	return (0);
}

static int
acpi_cppc_attach(device_t dev)
{
	struct acpi_cppc_softc *sc = device_get_softc(dev);
	ACPI_BUFFER buf;
	ACPI_OBJECT *pkg;
	uint64_t cur;
	uint32_t nentries, rev;
	int cpu, error, max_mhz, mhz;

	sc->dev = dev;
	sc->head = cpu_get_pcpuid(dev);
	sc->ref_ctr.space = sc->del_ctr.space = -1;
	(void)acpi_cppc_domain_cpus(sc->head, &sc->cpus);

	buf.Pointer = NULL;
	buf.Length = ACPI_ALLOCATE_BUFFER;
	if (ACPI_FAILURE(AcpiEvaluateObject(acpi_cppc_cpu_handle(sc->head),
	    "_CPC", NULL, &buf)))
		return (ENXIO);
	pkg = buf.Pointer;
	error = EINVAL;
	if (!ACPI_PKG_VALID(pkg, CPC_ENTRIES_V3) ||
	    acpi_PkgInt32(pkg, CPC_NUM_ENTRIES, &nentries) != 0 ||
	    acpi_PkgInt32(pkg, CPC_REVISION, &rev) != 0 || rev < 3 ||
	    acpi_cppc_get_value(pkg, CPC_HIGHEST_PERF, &sc->highest) != 0 ||
	    acpi_cppc_get_value(pkg, CPC_NOMINAL_PERF, &sc->nominal) != 0 ||
	    acpi_cppc_get_value(pkg, CPC_LOWEST_PERF, &sc->lowest) != 0 ||
	    acpi_PkgInt32(pkg, CPC_LOWEST_FREQ, &sc->lowest_mhz) != 0 ||
	    acpi_PkgInt32(pkg, CPC_NOMINAL_FREQ, &sc->nominal_mhz) != 0) {
		device_printf(dev, "_CPC without performance and frequency "
		    "bounds (revision 3) is not supported\n");
		goto out;
	}
	if (acpi_cppc_get_value(pkg, CPC_REFERENCE_PERF, &sc->ref_perf) != 0 ||
	    sc->ref_perf == 0)
		sc->ref_perf = sc->nominal;
	(void)acpi_cppc_get_reg(pkg, CPC_REFERENCE_CTR, &sc->ref_ctr);
	(void)acpi_cppc_get_reg(pkg, CPC_DELIVERED_CTR, &sc->del_ctr);
	if (acpi_cppc_map_reg(&sc->ref_ctr) != 0 ||
	    acpi_cppc_map_reg(&sc->del_ctr) != 0)
		sc->ref_ctr.space = sc->del_ctr.space = -1;
	if (sc->lowest == 0 || sc->nominal < sc->lowest ||
	    sc->highest < sc->nominal || sc->lowest_mhz == 0 ||
	    sc->nominal_mhz < sc->lowest_mhz) {
		device_printf(dev, "inconsistent _CPC bounds\n");
		goto out;
	}

	sc->desired = malloc(sizeof(*sc->desired) * CPU_COUNT(&sc->cpus),
	    M_ACPI_CPPC, M_WAITOK | M_ZERO);
	CPU_FOREACH_ISSET(cpu, &sc->cpus) {
		if ((error = acpi_cppc_add_desired(sc, cpu)) != 0) {
			device_printf(dev, "CPU %d: unsupported desired "
			    "performance register\n", cpu);
			goto out;
		}
	}

	/* Levels 100 MHz apart from the highest down, and the lowest. */
	max_mhz = acpi_cppc_perf_to_mhz(sc, sc->highest);
	for (mhz = max_mhz; mhz > (int)sc->lowest_mhz &&
	    sc->nlevels < CPPC_MAX_LEVELS - 1; mhz -= CPPC_STEP_MHZ)
		sc->levels[sc->nlevels++] = mhz;
	sc->levels[sc->nlevels++] = sc->lowest_mhz;

	if (acpi_cppc_read(&sc->desired[0], &cur) == 0 && cur != 0)
		sc->cur_mhz = acpi_cppc_perf_to_mhz(sc, cur);
	else
		sc->cur_mhz = max_mhz;
	acpi_cppc_notify(sc, sc->cur_mhz);

	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
	    "delivered_mhz", CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    acpi_cppc_delivered_sysctl, "I",
	    "Frequency the feedback counters measure (MHz)");
	if (bootverbose)
		device_printf(dev, "%d CPUs, %d-%d MHz, performance %u-%u, "
		    "nominal %u\n", CPU_COUNT(&sc->cpus), sc->lowest_mhz,
		    max_mhz, sc->lowest, sc->highest, sc->nominal);
	cpufreq_register(dev);
	error = 0;
out:
	AcpiOsFree(buf.Pointer);
	if (error != 0) {
		for (cpu = 0; sc->desired != NULL && cpu < sc->ndesired; cpu++)
			acpi_cppc_unmap_reg(&sc->desired[cpu]);
		free(sc->desired, M_ACPI_CPPC);
		sc->desired = NULL;
	}
	return (error);
}

static int
acpi_cppc_detach(device_t dev)
{
	struct acpi_cppc_softc *sc = device_get_softc(dev);
	int error, i;

	if ((error = cpufreq_unregister(dev)) != 0)
		return (error);
	for (i = 0; i < sc->ndesired; i++)
		acpi_cppc_unmap_reg(&sc->desired[i]);
	free(sc->desired, M_ACPI_CPPC);
	return (0);
}

static device_method_t acpi_cppc_methods[] = {
	DEVMETHOD(device_identify,	acpi_cppc_identify),
	DEVMETHOD(device_probe,		acpi_cppc_probe),
	DEVMETHOD(device_attach,	acpi_cppc_attach),
	DEVMETHOD(device_detach,	acpi_cppc_detach),

	DEVMETHOD(cpufreq_drv_get,	acpi_cppc_get),
	DEVMETHOD(cpufreq_drv_set,	acpi_cppc_set),
	DEVMETHOD(cpufreq_drv_type,	acpi_cppc_type),
	DEVMETHOD(cpufreq_drv_settings,	acpi_cppc_settings),

	DEVMETHOD_END
};

static driver_t acpi_cppc_driver = {
	"acpi_cppc",
	acpi_cppc_methods,
	sizeof(struct acpi_cppc_softc),
};

DRIVER_MODULE(acpi_cppc, cpu, acpi_cppc_driver, 0, 0);
MODULE_DEPEND(acpi_cppc, acpi, 1, 1, 1);
