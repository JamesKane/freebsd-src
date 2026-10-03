/*
 * Copyright (c) 2026 Koine Yuusuke
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sys/cdefs.h>
#include "opt_global.h"
#include "opt_coreindex.h"

#include <sys/malloc.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/lock.h>
#include <sys/proc.h>
#include <sys/smp.h>
#include <sys/hmp.h>
#include <sys/sysctl.h>

#define	COREINDEX_NAME "coreindex"
#define	COREINDEX_HMP_PRIORITY 10

/* Check kernel build option with SMP */
#ifndef SMP
#error "coreindex requires the SMP option in the kernel configuraton."
#endif

/* Check kernel build option with HMP */
#ifndef HMP
#error "coreindex requires the HMP option in the kernel configuraton."
#endif

/*
 * Structure defines
 */
struct coreindex_instance {
	uint8_t		status;

	uint32_t	*capacity;

	struct {
		uint16_t	*performance;
		uint16_t	*efficiency;
	} score;

	struct {
		struct sysctl_ctx_list ctx;
		struct sysctl_oid *parent;
		struct sysctl_oid **core;
	} sysctl;
};

#define	COREINDEX_STATUS_LOADED	(1 << 0)	/* status bit0: coreindex loaded */

static struct coreindex_instance cidx_inst;

/*
 * Prototype defines
 */
static uint32_t coreindex_capacity_get_tunable_value(void);
static int coreindex_capacity_provider_probe(void);
static int coreindex_capacity_provider_init(void);
static uint16_t coreindex_score_get_tunable(char *nodename, uint16_t *valptr);
static int coreindex_score_provider_probe(void);
static int coreindex_score_provider_init(void);
static int coreindex_modevent(module_t mod, int type, void *data);
static int coreindex_register_sysctl(void);
static int coreindex_capacity_register_sysctl(void);
static int coreindex_score_set_sysctl(SYSCTL_HANDLER_ARGS);
static int coreindex_score_register_sysctl(void);

DPCPU_DECLARE(struct hmp_pcpu, hmp_pcpu);

static MALLOC_DEFINE(M_COREINDEX, COREINDEX_NAME, "Buffers for coreindex driver");

/*
 * HMP capacity provider
 */
static uint32_t
coreindex_capacity_get_tunable_value(void)
{
	char path[128];
	uint64_t value;
	uint32_t mincap = HMP_CAPACITY_SCALE;
	uint32_t maxcap = 0;

	/* Set the capacity value of the core for which tunable parameters are configured. */
	for (int i=0; i<mp_ncpus; i++) {
		/* Set temporary (not initialized) value */
		*(cidx_inst.capacity + i) = 0;

		/* Get tunable capacity value */
		snprintf(path, sizeof(path), "hw.coreindex.%d.capacity", i);
		if (TUNABLE_UINT64_FETCH(path, &value)) {
			if ((value > 0) && (value <= HMP_CAPACITY_SCALE))
				*(cidx_inst.capacity + i) = (uint32_t)value;

			if (mincap > *(cidx_inst.capacity + i))
				mincap = *(cidx_inst.capacity + i);

			if (maxcap < *(cidx_inst.capacity + i))
				maxcap = *(cidx_inst.capacity + i);
		}
	}

	/* Calc. default capacity value */
	if ((mincap == HMP_CAPACITY_SCALE) && (maxcap == 0))
		value = HMP_CAPACITY_SCALE;
	else
		value = (mincap + maxcap) / 2;

	/* Set the capacity value of the core for which tunable parameters are NOT configured. */
	for (int i=0; i<mp_ncpus; i++) {
		if (*(cidx_inst.capacity + i) > 0)
			continue;

		/* Get tunable capacity value */
		*(cidx_inst.capacity + i) = value;
	}

	return mincap;
}

static int
coreindex_capacity_provider_probe(void)
{
	return (0);
}

static int
coreindex_capacity_provider_init(void)
{
	struct hmp_pcpu *hp;
	uint32_t mincap;
	uint32_t *capacity;
	int cpu;

	/* Alloc CPU core perf. value */
	capacity = malloc((sizeof(uint32_t) * mp_ncpus), M_COREINDEX, M_NOWAIT | M_ZERO);
	if (capacity == NULL)
		return (-1);

	cidx_inst.capacity = capacity;

	/* Get tunable capacity value */
	mincap = coreindex_capacity_get_tunable_value();

	/* Regist sysctl interfaces */
	if (coreindex_register_sysctl()) {
		free(cidx_inst.capacity, M_COREINDEX);
		return (-2);
	}

	/* Regist sysctl capacity leaf */
	coreindex_capacity_register_sysctl();

	/* Set CPU core capacity for HMP */
	CPU_FOREACH(cpu) {
		hp = DPCPU_ID_PTR(cpu, hmp_pcpu);
		hp->capacity = HMP_CAPACITY_NORMAL_FROM(*(cidx_inst.capacity + cpu), mincap);
	}

	/* Set coreindex loaded flag */
	cidx_inst.status |= COREINDEX_STATUS_LOADED;

	return (0);
}

static struct hmp_capacity_provider coreindex_capacity_provider = {
	.name		= COREINDEX_NAME,
	.priority	= COREINDEX_HMP_PRIORITY,
	.probe		= coreindex_capacity_provider_probe,
	.init		= coreindex_capacity_provider_init,
};
HMP_CAPACITY_PROVIDER_DECLARE(coreindex, coreindex_capacity_provider);

/*
 * HMP socre provider
 */
static uint16_t
coreindex_score_get_tunable(char *nodename, uint16_t *valptr)
{
	char path[128];
	uint64_t value;
	uint16_t maxval = 0;

	for (int i=0; i<mp_ncpus; i++) {
		/* Get tunable score value */
		snprintf(path, sizeof(path), "hw.coreindex.%d.%s", i, nodename);
		if (TUNABLE_UINT64_FETCH(path, &value))
			*(valptr + i) = (uint16_t)((value > HMP_SCORE_MAX) ?  HMP_SCORE_DEFAULT : value);
		else
			*(valptr + i) = HMP_SCORE_DEFAULT;

		if (maxval < *(valptr + i))
			maxval = *(valptr + i);
	}

	return maxval;
}

static int
coreindex_score_provider_probe(void)
{
	return (0);
}

static int
coreindex_score_provider_init(void)
{
	struct hmp_pcpu *hp;
	uint16_t *performance;
	uint16_t *efficiency;
	int cpu;

	/* Check capacity provider regist status */
	if (!(cidx_inst.status & COREINDEX_STATUS_LOADED))
		return (-1);

	/* Alloc CPU core perf. value */
	performance = malloc((sizeof(uint16_t) * mp_ncpus), M_COREINDEX, M_NOWAIT | M_ZERO);
	if (performance == NULL)
		return (-2);

	cidx_inst.score.performance = performance;

	/* Alloc CPU core efficiency value */
	efficiency = malloc((sizeof(uint16_t) * mp_ncpus), M_COREINDEX, M_NOWAIT | M_ZERO);
	if (efficiency == NULL) {
		free(cidx_inst.score.performance, M_COREINDEX);
		return (-3);
	}
	cidx_inst.score.efficiency = efficiency;

	/* Get tunable capacity value */
	coreindex_score_get_tunable("performance", performance);
	coreindex_score_get_tunable("efficiency", efficiency);

	/* Regist sysctl score leaf */
	coreindex_score_register_sysctl();

	/* Set CPU core capacity for HMP */
	CPU_FOREACH(cpu) {
		hp = DPCPU_ID_PTR(cpu, hmp_pcpu);
		hmp_set_score(hp, HMP_SCORE_PERF, *(cidx_inst.score.performance + cpu));
		hmp_set_score(hp, HMP_SCORE_EFF, *(cidx_inst.score.efficiency + cpu));
	}

	printf("coreindex: CPU core performance/efficiency index setter\n");

	return (0);
}

static struct hmp_score_provider coreindex_score_provider = {
	.name		= COREINDEX_NAME,
	.priority	= COREINDEX_HMP_PRIORITY,
	.probe		= coreindex_score_provider_probe,
	.init		= coreindex_score_provider_init,
};
HMP_SCORE_PROVIDER_DECLARE(coreindex, coreindex_score_provider);

/*
 * Module Interfaces
 */
static int
coreindex_modevent(module_t mod, int type, void *data)
{
	switch (type) {
	case MOD_LOAD:
		/*
		 * If this driver is enabled in the /boot/loader.conf file, it will be
		 * initialized as a driver module after initialization by hmp(4).
		 * If an error is returned in this case, an error message will be recorded
		 * in dmesg due to double loading, even though it has been loaded successfully.
		 * Therefore, if it has been initialized by hmp(4), zero is returned, and it will
		 * be treated as having been loaded without errors.
		 */
		if (cidx_inst.status & COREINDEX_STATUS_LOADED)
			return (0);

		/*
		 * To allow enabling the driver from the /boot/loader.conf file, it is registered
		 * as a driver module, but loading via the kldload command is not permitted.
		 * Therefore, it returns an error.
		 */
		return (EOPNOTSUPP);

	default:
		return (EOPNOTSUPP);
	}
}

static moduledata_t coreindex_mod = {
	COREINDEX_NAME,		/* module name */
	coreindex_modevent,	/* event handler */
	NULL
};
DECLARE_MODULE(coreindex, coreindex_mod, SI_SUB_SMP + 2, SI_ORDER_ANY);

/*
 * Sysctls
 */
static int
coreindex_register_sysctl(void)
{
	char id[8];
	char desc[128];
	struct sysctl_oid *node;

	sysctl_ctx_init(&(cidx_inst.sysctl.ctx));

	/* Regist "hw.coreindex" parent node */
	cidx_inst.sysctl.parent = SYSCTL_ADD_NODE( &(cidx_inst.sysctl.ctx),
			SYSCTL_STATIC_CHILDREN(_hw), OID_AUTO, COREINDEX_NAME,
			CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
			"coreindex: CPU core performance/efficiency index setter");
	if (cidx_inst.sysctl.parent == NULL)
		return (-1);

	/* Alloc sysctl node memory */
	cidx_inst.sysctl.core = malloc((sizeof(struct sysctl_oid *) * mp_ncpus),
								M_COREINDEX, M_NOWAIT | M_ZERO);
	if (cidx_inst.sysctl.core == NULL) {
		sysctl_ctx_free(&(cidx_inst.sysctl.ctx));
		return (-2);
	}

	/* Regist "hw.coreindex.X" node each CPU core id */
	for (int i=0; i<mp_ncpus; i++) {
		snprintf(id, sizeof(id), "%d", i);
		snprintf(desc, sizeof(desc), "CPU core #%d performance/efficiency index", i);

		node = SYSCTL_ADD_NODE(&(cidx_inst.sysctl.ctx), SYSCTL_CHILDREN(cidx_inst.sysctl.parent),
					OID_AUTO, id, CTLFLAG_RD | CTLFLAG_MPSAFE, 0, desc);
		if (node == NULL) {
			sysctl_ctx_free(&(cidx_inst.sysctl.ctx));
			free(cidx_inst.sysctl.core, M_COREINDEX);
			return (-3);
		}

		*(cidx_inst.sysctl.core + i) = node;
	}

	return (0);
}

static int
coreindex_capacity_register_sysctl(void)
{
	char desc[128];

	/* Set sysctl capacity value */
	for (int i=0; i<mp_ncpus; i++) {
		snprintf(desc, sizeof(desc), "CPU core #%d capacity index", i);
		SYSCTL_ADD_U32( &(cidx_inst.sysctl.ctx), SYSCTL_CHILDREN(*(cidx_inst.sysctl.core+i)),
				OID_AUTO, "capacity", CTLFLAG_RDTUN, (cidx_inst.capacity + i), 0, desc);
	}

	return (0);
}

static int
coreindex_score_set_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct hmp_pcpu *hp;
	uint16_t *valptr;
	int cpu = arg2;
	int err, tmpval;

	valptr = (uint16_t *)arg1 + cpu;
	tmpval = *valptr;

	err = sysctl_handle_int(oidp, &tmpval, 0, req);
	if (err || req->newptr == NULL)
		return (err);

	if ((tmpval<0) || (tmpval>1024))
		return (EINVAL);

	*valptr = (uint16_t)(tmpval & 0xffff);

	hp = DPCPU_ID_PTR(cpu, hmp_pcpu);
	if ((uint16_t *)arg1 == cidx_inst.score.performance)
		hmp_set_score(hp, HMP_SCORE_PERF, tmpval);
	else if ((uint16_t *)arg1 == cidx_inst.score.efficiency)
		hmp_set_score(hp, HMP_SCORE_EFF, tmpval);

	return (0);
}

static int
coreindex_score_register_sysctl(void)
{
	char desc[128];

	/* Set sysctl capacity value */
	for (int i=0; i<mp_ncpus; i++) {
		snprintf(desc, sizeof(desc), "CPU core #%d performance score index", i);
		SYSCTL_ADD_PROC(&(cidx_inst.sysctl.ctx), SYSCTL_CHILDREN(*(cidx_inst.sysctl.core+i)),
				OID_AUTO, "performance", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,
				(void *)(cidx_inst.score.performance), i, coreindex_score_set_sysctl,"I", desc);

		snprintf(desc, sizeof(desc), "CPU core #%d efficiency score index", i);
		SYSCTL_ADD_PROC(&(cidx_inst.sysctl.ctx), SYSCTL_CHILDREN(*(cidx_inst.sysctl.core+i)),
				OID_AUTO, "efficiency", CTLTYPE_INT | CTLFLAG_RW | CTLFLAG_MPSAFE,
				(void *)(cidx_inst.score.efficiency), i, coreindex_score_set_sysctl,"I", desc);
	}

	return (0);
}

