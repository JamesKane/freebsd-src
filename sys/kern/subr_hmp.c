/*
 * Copyright (c) 2026 FreeBSD Foundation
 *
 * This software was developed by Minsoo Choo under sponsorship from the
 * FreeBSD Foundation.
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#include <sys/systm.h>
#include <sys/hmp.h>
#include <sys/kernel.h>
#include <sys/pcpu.h>
#include <sys/smp.h>
#include <sys/sysctl.h>

/* System-wide CPU capability information */
struct hmp hmp_state = {
	.has_scores	= false,
};

/* Per-CPU HMP state */
DPCPU_DEFINE(struct hmp_pcpu, hmp_pcpu);

/*
 * Provider interfaces
 *
 * Capacity and score providers live on independent lists and are selected
 * independently at boot.  Selection runs in two phases (see hmp_init):
 * capacity first, then scores.
 */
static SLIST_HEAD(, hmp_capacity_provider) hmp_cap_providers =
    SLIST_HEAD_INITIALIZER(hmp_cap_providers);
static SLIST_HEAD(, hmp_score_provider) hmp_score_providers =
    SLIST_HEAD_INITIALIZER(hmp_score_providers);

static struct hmp_capacity_provider *hmp_active_cap_provider;
static struct hmp_score_provider *hmp_active_score_provider;

void
hmp_capacity_provider_register(struct hmp_capacity_provider *p)
{
	KASSERT(p->name != NULL, ("hmp: capacity provider missing name"));
	KASSERT(p->probe != NULL && p->init != NULL,
	    ("hmp: capacity provider %s missing probe/init", p->name));

	SLIST_INSERT_HEAD(&hmp_cap_providers, p, link);
}

void
hmp_score_provider_register(struct hmp_score_provider *p)
{
	KASSERT(p->name != NULL, ("hmp: score provider missing name"));
	KASSERT(p->probe != NULL && p->init != NULL,
	    ("hmp: score provider %s missing probe/init", p->name));

	SLIST_INSERT_HEAD(&hmp_score_providers, p, link);
}

/*
 * Set the CPUs' capacities from perf[cpu], each CPU's relative throughput,
 * higher for faster, on one scale for all CPUs: for a capacity provider's
 * init().  The scheduler sees each CPU's new capacity as it is stored.
 */
int
hmp_capacity_set(const uint32_t *perf)
{
	struct hmp_pcpu *hp;
	uint32_t min;
	int cpu;

	min = UINT32_MAX;
	CPU_FOREACH(cpu) {
		if (perf[cpu] == 0)
			return (EINVAL);
		min = MIN(min, perf[cpu]);
	}
	/* hmp(4)'s capacity: HMP_CAPACITY_SCALE for the slowest CPU. */
	CPU_FOREACH(cpu) {
		hp = DPCPU_ID_PTR(cpu, hmp_pcpu);
		atomic_store_rel_32(&hp->capacity,
		    (uint64_t)min * HMP_CAPACITY_SCALE / perf[cpu]);
	}
	atomic_add_rel_int(&hmp_state.capacity_gen, 1);
	return (0);
}

/*
 * The same, later, for a provider that learns the capacities after hmp(4)
 * chose it: only the active capacity provider may.
 */
int
hmp_capacity_update(struct hmp_capacity_provider *p, const uint32_t *perf)
{
	if (!p->active)
		return (EPERM);
	return (hmp_capacity_set(perf));
}

/*
 * Default capacity provider.
 *
 * Always wins if nothing else probes: every CPU gets HMP_CAPACITY_DEFAULT,
 * which collapses HMP decisions down to "any CPU is fine."
 */

static int
hmp_default_cap_probe(void)
{
	return (0);
}

static int
hmp_default_cap_init(void)
{
	struct hmp_pcpu *hp;
	int cpu;

	CPU_FOREACH(cpu) {
		hp = DPCPU_ID_PTR(cpu, hmp_pcpu);
		hp->capacity = HMP_CAPACITY_DEFAULT;
	}

	return (0);
}

static struct hmp_capacity_provider hmp_default_cap_provider = {
	.name		= "default",
	.priority	= 0,
	.probe		= hmp_default_cap_probe,	/* always true */
	.init		= hmp_default_cap_init,
};
HMP_CAPACITY_PROVIDER_DECLARE(default, hmp_default_cap_provider);

/*
 * Default score provider.
 *
 * Always wins if nothing else probes: every CPU gets HMP_SCORE_DEFAULT,
 * which collapses HMP decisions down to "any CPU is fine."
 */

static int
hmp_default_score_probe(void)
{
	return (0);
}

static int
hmp_default_score_init(void)
{
	struct hmp_pcpu *hp;
	int cpu;

	CPU_FOREACH(cpu) {
		hp = DPCPU_ID_PTR(cpu, hmp_pcpu);
		hmp_set_score(hp, HMP_SCORE_PERF, HMP_SCORE_DEFAULT);
		hmp_set_score(hp, HMP_SCORE_EFF, HMP_SCORE_DEFAULT);
	}

	return (0);
}

static struct hmp_score_provider hmp_default_score_provider = {
	.name		= "default",
	.priority	= 0,
	.probe		= hmp_default_score_probe,	/* always true */
	.init		= hmp_default_score_init,
};
HMP_SCORE_PROVIDER_DECLARE(default, hmp_default_score_provider);

/*
 * Initialization
 */

static void
hmp_init_capacity(void)
{
	struct hmp_capacity_provider *p, *best;

	SLIST_FOREACH(p, &hmp_cap_providers, link) {
		if (p->probe() == 0)
			p->probed = true;
	}

	for (;;) {
		best = NULL;
		SLIST_FOREACH(p, &hmp_cap_providers, link) {
			if (!p->probed)
				continue;
			if (best == NULL || p->priority > best->priority)
				best = p;
		}
		if (best == NULL)
			break;
		if (best->init() == 0)
			break;
		best->probed = false;
	}

	KASSERT(best != NULL,
	    ("hmp: no capacity provider init succeeded, not even default"));
	best->active = true;
	hmp_active_cap_provider = best;
}

static void
hmp_init_scores(void)
{
	struct hmp_score_provider *p, *best;

	SLIST_FOREACH(p, &hmp_score_providers, link) {
		if (p->probe() == 0)
			p->probed = true;
	}

	for (;;) {
		best = NULL;
		SLIST_FOREACH(p, &hmp_score_providers, link) {
			if (!p->probed)
				continue;
			if (best == NULL || p->priority > best->priority)
				best = p;
		}
		if (best == NULL) {
			hmp_state.has_scores = false;
			return;
		}
		if (best->init() == 0)
			break;
		best->probed = false;
	}

	best->active = true;
	hmp_active_score_provider = best;
	hmp_state.has_scores = true;
}

static void
hmp_init(void *arg __unused)
{
	hmp_init_capacity();
	hmp_init_scores();
	/* For those who derived data from the capacities before. */
	atomic_add_rel_int(&hmp_state.capacity_gen, 1);
}
SYSINIT(hmp, SI_SUB_SMP + 1, SI_ORDER_ANY, hmp_init, NULL);

/*
 * Sysctls
 */

static int
hmp_sysctl_capacity(SYSCTL_HANDLER_ARGS)
{
	struct hmp_pcpu *hp;
	unsigned int v;

	hp = DPCPU_ID_PTR(arg2, hmp_pcpu);
	v = hp->capacity;
	return (sysctl_handle_int(oidp, &v, 0, req));
}

static int
hmp_sysctl_score(SYSCTL_HANDLER_ARGS)
{
	struct hmp_pcpu *hp;
	int cpu = arg2 & 0xffff;
	enum score_type st = (arg2 >> 16) & 0xffff;
	unsigned int v;

	hp = DPCPU_ID_PTR(cpu, hmp_pcpu);
	v = hmp_get_score(hp, st);
	return (sysctl_handle_int(oidp, &v, 0, req));
}

static int
hmp_sysctl_active_cap_provider(SYSCTL_HANDLER_ARGS)
{
	const char *name;

	name = (hmp_active_cap_provider != NULL) ?
	    hmp_active_cap_provider->name : "none";
	return (sysctl_handle_string(oidp, __DECONST(char *, name), 0, req));
}

static int
hmp_sysctl_active_score_provider(SYSCTL_HANDLER_ARGS)
{
	const char *name;

	name = (hmp_active_score_provider != NULL) ?
	    hmp_active_score_provider->name : "none";
	return (sysctl_handle_string(oidp, __DECONST(char *, name), 0, req));
}

static void
hmp_sysctl_add_provider(struct sysctl_oid_list *parent, const char *name,
    int *priority, bool *probed, bool *active)
{
	struct sysctl_oid *node;

	node = SYSCTL_ADD_NODE(NULL, parent, OID_AUTO, name,
	    CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, "HMP provider state");
	if (node == NULL)
		return;

	SYSCTL_ADD_INT(NULL, SYSCTL_CHILDREN(node), OID_AUTO, "priority",
	    CTLFLAG_RD, priority, 0, "Provider priority (higher wins)");
	SYSCTL_ADD_BOOL(NULL, SYSCTL_CHILDREN(node), OID_AUTO, "probed",
	    CTLFLAG_RD, probed, 0, "probe() returned true on this system");
	SYSCTL_ADD_BOOL(NULL, SYSCTL_CHILDREN(node), OID_AUTO, "active",
	    CTLFLAG_RD, active, 0, "Selected as the active provider");
}

static SYSCTL_NODE(_kern, OID_AUTO, hmp, CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
    "Heterogeneous multi-processing state");

SYSCTL_BOOL(_kern_hmp, OID_AUTO, has_scores, CTLFLAG_RD,
    &hmp_state.has_scores, 0,
    "True if a score provider is supplying dynamic perf/eff scores");

static void
hmp_sysctl_init(void *arg __unused)
{
	struct sysctl_oid *cpu_root, *cpu_node, *cpu_score_node;
	struct sysctl_oid *provider_root, *cap_root, *score_root;
	struct sysctl_oid_list *cpu_root_children, *cpu_children;
	struct sysctl_oid_list *cpu_score_children, *provider_children;
	struct hmp_capacity_provider *cp;
	struct hmp_score_provider *sp;
	char name[8];
	int cpu;

	/*
	 * Per-CPU tree: kern.hmp.cpu.<N>.{capacity,score.perf,score.eff,...}
	 */
	cpu_root = SYSCTL_ADD_NODE(NULL, SYSCTL_STATIC_CHILDREN(_kern_hmp),
	    OID_AUTO, "cpu", CTLFLAG_RD | CTLFLAG_MPSAFE, NULL,
	    "Per-CPU HMP state");
	if (cpu_root == NULL)
		return;
	cpu_root_children = SYSCTL_CHILDREN(cpu_root);

	CPU_FOREACH(cpu) {
		snprintf(name, sizeof(name), "%d", cpu);
		cpu_node = SYSCTL_ADD_NODE(NULL, cpu_root_children, OID_AUTO,
		    name, CTLFLAG_RD | CTLFLAG_MPSAFE, NULL,
		    "Per-CPU HMP state");
		if (cpu_node == NULL)
			continue;
		cpu_children = SYSCTL_CHILDREN(cpu_node);

		SYSCTL_ADD_PROC(NULL, cpu_children, OID_AUTO, "capacity",
		    CTLTYPE_UINT | CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, cpu,
		    hmp_sysctl_capacity, "IU",
		    "Static capacity (0-HMP_CAPACITY_SCALE)");

		cpu_score_node = SYSCTL_ADD_NODE(NULL, cpu_children, OID_AUTO,
		    "score", CTLFLAG_RD | CTLFLAG_MPSAFE, NULL,
		    "Per-CPU dynamic scores");
		if (cpu_score_node != NULL) {
			cpu_score_children = SYSCTL_CHILDREN(cpu_score_node);

			SYSCTL_ADD_PROC(NULL, cpu_score_children, OID_AUTO,
			    "perf",
			    CTLTYPE_UINT | CTLFLAG_RD | CTLFLAG_MPSAFE, NULL,
			    cpu | (HMP_SCORE_PERF << 16), hmp_sysctl_score,
			    "IU",
			    "Current performance score (dynamic)");

			SYSCTL_ADD_PROC(NULL, cpu_score_children, OID_AUTO,
			    "eff",
			    CTLTYPE_UINT | CTLFLAG_RD | CTLFLAG_MPSAFE, NULL,
			    cpu | (HMP_SCORE_EFF << 16), hmp_sysctl_score,
			    "IU",
			    "Current efficiency score (dynamic)");
		}
	}

	/*
	 * Provider tree.
	 *
	 * kern.hmp.provider.active_capacity
	 *     - name of the winning capacity provider (always set)
	 * kern.hmp.provider.active_score
	 *     - name of the winning score provider, or "" if none
	 * kern.hmp.provider.capacity.<n>.{priority,probed,active}
	 * kern.hmp.provider.score.<n>.{priority,probed,active}
	 *
	 * Every registered provider is listed on each side, not just the
	 * winner, so operators can see which providers were available and
	 * why one was chosen over another.
	 */
	provider_root = SYSCTL_ADD_NODE(NULL, SYSCTL_STATIC_CHILDREN(_kern_hmp),
	    OID_AUTO, "provider", CTLFLAG_RD | CTLFLAG_MPSAFE, NULL,
	    "Registered HMP providers");
	if (provider_root == NULL)
		return;
	provider_children = SYSCTL_CHILDREN(provider_root);

	SYSCTL_ADD_PROC(NULL, provider_children,
	    OID_AUTO, "active_capacity",
	    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, 0,
	    hmp_sysctl_active_cap_provider, "A",
	    "Name of the active HMP capacity provider");

	SYSCTL_ADD_PROC(NULL, provider_children,
	    OID_AUTO, "active_score",
	    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, 0,
	    hmp_sysctl_active_score_provider, "A",
	    "Name of the active HMP score provider (empty if none)");

	cap_root = SYSCTL_ADD_NODE(NULL, provider_children,
	    OID_AUTO, "capacity", CTLFLAG_RD | CTLFLAG_MPSAFE, NULL,
	    "Registered HMP capacity providers");
	if (cap_root != NULL) {
		SLIST_FOREACH(cp, &hmp_cap_providers, link) {
			hmp_sysctl_add_provider(SYSCTL_CHILDREN(cap_root),
			    cp->name, &cp->priority, &cp->probed, &cp->active);
		}
	}

	score_root = SYSCTL_ADD_NODE(NULL, provider_children,
	    OID_AUTO, "score", CTLFLAG_RD | CTLFLAG_MPSAFE, NULL,
	    "Registered HMP score providers");
	if (score_root != NULL) {
		SLIST_FOREACH(sp, &hmp_score_providers, link) {
			hmp_sysctl_add_provider(SYSCTL_CHILDREN(score_root),
			    sp->name, &sp->priority, &sp->probed, &sp->active);
		}
	}
}
SYSINIT(hmp_sysctl, SI_SUB_SMP + 2, SI_ORDER_ANY, hmp_sysctl_init, NULL);
