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
 * Qualcomm shared memory (SMEM), as the boot loader sets it up: a header
 * with a table of contents for the global heap, and a partition table 4 KB
 * from the end listing partitions private to a pair of hosts, and on
 * version 12 a global partition.  A partition is a header, then items
 * allocated upwards ("uncached", each a header and its data) and downwards
 * ("cached", the data and then its header).  Allocation takes a hardware
 * mutex shared with the other processors.
 *
 * ACPI doesn't describe it, so where it is comes from a table of SoCs,
 * which the DSDT's \_SB.SOID identifies.
 */

#include "opt_acpi.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/sx.h>

#include <machine/atomic.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include <dev/qcom_glink/qcom_smem.h>

#define	SMEM_ITEM_FIXED		8	/* allocated by the boot loader */
#define	SMEM_ITEMS		512
#define	SMEM_GLOBAL_HOST	0xfffe
#define	SMEM_SBL_VERSION	7	/* index of the boot loader's version */
#define	SMEM_HEAP_V11		11
#define	SMEM_PART_V12		12
#define	SMEM_MUTEX_LOCK		3	/* the hardware mutex SMEM uses */
#define	SMEM_MUTEX_APPS		1	/* our value in a held lock */

struct smem_header {
	uint32_t	proc_comm[16];
	uint32_t	version[32];
	uint32_t	initialized;
	uint32_t	free_offset;
	uint32_t	available;
	uint32_t	reserved;
	struct {
		uint32_t	allocated;
		uint32_t	offset;
		uint32_t	size;
		uint32_t	aux_base;
	} toc[SMEM_ITEMS];
};

#define	SMEM_PTABLE_MAGIC	0x434f5424	/* "$TOC" */
struct smem_ptable {
	uint32_t	magic;
	uint32_t	version;
	uint32_t	num_entries;
	uint32_t	reserved[5];
	struct smem_ptable_entry {
		uint32_t	offset;
		uint32_t	size;
		uint32_t	flags;
		uint16_t	host0;
		uint16_t	host1;
		uint32_t	cacheline;
		uint32_t	reserved[7];
	} entry[];
};

#define	SMEM_INFO_MAGIC		0x49494953	/* "SIII" */
struct smem_info {
	uint32_t	magic;
	uint32_t	size;
	uint32_t	base_addr;
	uint32_t	reserved;
	uint16_t	num_items;
};

#define	SMEM_PART_MAGIC		0x54525024	/* "$PRT" */
struct smem_partition_header {
	uint32_t	magic;
	uint16_t	host0;
	uint16_t	host1;
	uint32_t	size;
	uint32_t	offset_free_uncached;
	uint32_t	offset_free_cached;
	uint32_t	reserved[3];
};

#define	SMEM_CANARY		0xa5a5
struct smem_private_entry {
	uint16_t	canary;
	uint16_t	item;
	uint32_t	size;		/* of the data, padding included */
	uint16_t	padding_data;
	uint16_t	padding_hdr;
	uint32_t	reserved;
};

struct qcom_smem_soc {
	uint32_t	id;		/* \_SB.SOID */
	vm_paddr_t	base;
	vm_size_t	size;
	vm_paddr_t	mutex;		/* TCSR mutexes, 4 KB apart */
};

static const struct qcom_smem_soc qcom_smem_socs[] = {
	{ 449, 0x80900000, 0x200000, 0x1f40000 },	/* SC8280XP */
};

struct smem_part {
	char		*base;
	size_t		size;
	size_t		cacheline;
};

static struct {
	struct sx	lock;		/* initialisation */
	struct mtx	alloc_mtx;	/* our side of the hardware mutex */
	bool		tried;
	int		error;
	char		*base;
	size_t		size;
	volatile uint32_t *mutex;
	u_int		version;
	u_int		items;
	struct smem_part global;	/* version 12 */
	struct smem_ptable *ptable;
} smem;

SX_SYSINIT(qcom_smem_lock, &smem.lock, "qcom_smem");
MTX_SYSINIT(qcom_smem_alloc, &smem.alloc_mtx, "qcom_smem alloc", MTX_SPIN);

static const struct qcom_smem_soc *
qcom_smem_find_soc(void)
{
	UINT32 id;
	u_int i;

	if (ACPI_FAILURE(acpi_GetInteger(ACPI_ROOT_OBJECT, "\\_SB.SOID", &id)))
		return (NULL);
	for (i = 0; i < nitems(qcom_smem_socs); i++)
		if (qcom_smem_socs[i].id == id)
			return (&qcom_smem_socs[i]);
	return (NULL);
}

static bool
qcom_smem_part_ok(struct smem_part *p)
{
	struct smem_partition_header *ph = (void *)p->base;

	return (ph->magic == SMEM_PART_MAGIC && ph->size == p->size &&
	    ph->offset_free_uncached <= p->size &&
	    ph->offset_free_cached <= p->size);
}

/* The partition shared with host, or NULL. */
static bool
qcom_smem_find_part(u_int host, struct smem_part *p)
{
	struct smem_ptable_entry *e;
	u_int i;

	if (smem.ptable == NULL)
		return (false);
	for (i = 0; i < smem.ptable->num_entries; i++) {
		e = &smem.ptable->entry[i];
		if (e->offset == 0 || e->size == 0 ||
		    e->offset + e->size > smem.size)
			continue;
		if ((e->host0 == QCOM_SMEM_HOST_APPS && e->host1 == host) ||
		    (e->host1 == QCOM_SMEM_HOST_APPS && e->host0 == host) ||
		    (host == SMEM_GLOBAL_HOST && e->host0 == host &&
		    e->host1 == host)) {
			p->base = smem.base + e->offset;
			p->size = e->size;
			p->cacheline = MAX(e->cacheline, 8);
			return (qcom_smem_part_ok(p));
		}
	}
	return (false);
}

static int
qcom_smem_init_locked(void)
{
	const struct qcom_smem_soc *soc;
	struct smem_header *hdr;
	struct smem_info *info;
	struct smem_ptable *pt;

	soc = qcom_smem_find_soc();
	if (soc == NULL)
		return (ENXIO);
	/* Shared with processors that don't snoop our caches. */
	smem.base = pmap_mapdev_attr(soc->base, soc->size,
	    VM_MEMATTR_WRITE_COMBINING);
	smem.size = soc->size;
	smem.mutex = pmap_mapdev(soc->mutex + SMEM_MUTEX_LOCK * 0x1000, 4);

	hdr = (struct smem_header *)smem.base;
	smem.version = hdr->version[SMEM_SBL_VERSION] >> 16;
	if (!hdr->initialized || (smem.version != SMEM_HEAP_V11 &&
	    smem.version != SMEM_PART_V12)) {
		printf("qcom_smem: not set up (version %u)\n", smem.version);
		return (ENXIO);
	}
	smem.items = SMEM_ITEMS;
	pt = (struct smem_ptable *)(smem.base + smem.size - PAGE_SIZE);
	if (pt->magic == SMEM_PTABLE_MAGIC && pt->version == 1 &&
	    sizeof(*pt) + pt->num_entries * sizeof(pt->entry[0]) +
	    sizeof(*info) <= PAGE_SIZE) {
		smem.ptable = pt;
		info = (struct smem_info *)&pt->entry[pt->num_entries];
		if (info->magic == SMEM_INFO_MAGIC && info->num_items != 0)
			smem.items = info->num_items;
	}
	if (smem.version == SMEM_PART_V12 &&
	    !qcom_smem_find_part(SMEM_GLOBAL_HOST, &smem.global)) {
		printf("qcom_smem: no global partition\n");
		return (ENXIO);
	}
	return (0);
}

static int
qcom_smem_init(void)
{

	sx_slock(&smem.lock);
	if (smem.tried) {
		sx_sunlock(&smem.lock);
		return (smem.error);
	}
	sx_sunlock(&smem.lock);
	sx_xlock(&smem.lock);
	if (!smem.tried) {
		smem.error = qcom_smem_init_locked();
		smem.tried = true;
	}
	sx_xunlock(&smem.lock);
	return (smem.error);
}

static void *
qcom_smem_get_private(struct smem_part *p, u_int item, size_t *size)
{
	struct smem_partition_header *ph = (void *)p->base;
	struct smem_private_entry *e;
	char *end, *start;
	size_t ehsz;

	/* Upwards: the header, padding, then the data. */
	start = p->base + sizeof(*ph);
	end = p->base + ph->offset_free_uncached;
	for (e = (void *)start; (char *)e + sizeof(*e) <= end;
	    e = (void *)((char *)e + sizeof(*e) + e->padding_hdr + e->size)) {
		if (e->canary != SMEM_CANARY)
			return (NULL);
		if (e->item == item) {
			if ((char *)e + sizeof(*e) + e->padding_hdr + e->size >
			    end || e->padding_data > e->size)
				return (NULL);
			*size = e->size - e->padding_data;
			return ((char *)e + sizeof(*e) + e->padding_hdr);
		}
	}
	/* Downwards: the data, then the header, at cache line spacing. */
	ehsz = roundup2(sizeof(*e), p->cacheline);
	end = p->base + ph->offset_free_cached;
	for (e = (void *)(p->base + p->size - ehsz); (char *)e > end;
	    e = (void *)((char *)e - e->size - ehsz)) {
		if (e->canary != SMEM_CANARY)
			return (NULL);
		if (e->item == item) {
			if ((char *)e - e->size < end ||
			    e->padding_data > e->size)
				return (NULL);
			*size = e->size - e->padding_data;
			return ((char *)e - e->size);
		}
	}
	return (NULL);
}

int
qcom_smem_get(u_int host, u_int item, void **ptr, size_t *size)
{
	struct smem_part p;
	struct smem_header *hdr;
	void *v;
	int error;

	error = qcom_smem_init();
	if (error != 0)
		return (error);
	if (item >= smem.items)
		return (EINVAL);
	v = NULL;
	if (qcom_smem_find_part(host, &p))
		v = qcom_smem_get_private(&p, item, size);
	if (v == NULL && smem.version == SMEM_PART_V12)
		v = qcom_smem_get_private(&smem.global, item, size);
	if (v == NULL && smem.version == SMEM_HEAP_V11) {
		hdr = (struct smem_header *)smem.base;
		if (hdr->toc[item].allocated &&
		    hdr->toc[item].aux_base == 0 &&
		    hdr->toc[item].offset + hdr->toc[item].size <= smem.size) {
			*size = hdr->toc[item].size;
			v = smem.base + hdr->toc[item].offset;
		}
	}
	if (v == NULL)
		return (ENOENT);
	*ptr = v;
	return (0);
}

/* The hardware mutex: held when it reads back the value we wrote. */
static int
qcom_smem_lock_hw(void)
{
	int us;

	mtx_lock_spin(&smem.alloc_mtx);
	for (us = 0; us < 1000000; us++) {
		*smem.mutex = SMEM_MUTEX_APPS;
		if (*smem.mutex == SMEM_MUTEX_APPS)
			return (0);
		DELAY(1);
	}
	mtx_unlock_spin(&smem.alloc_mtx);
	return (ETIMEDOUT);
}

static void
qcom_smem_unlock_hw(void)
{

	*smem.mutex = 0;
	mtx_unlock_spin(&smem.alloc_mtx);
}

static int
qcom_smem_alloc_private(struct smem_part *p, u_int item, size_t size)
{
	struct smem_partition_header *ph = (void *)p->base;
	struct smem_private_entry *e;
	char *end, *cached;
	size_t asize;

	end = p->base + ph->offset_free_uncached;
	cached = p->base + ph->offset_free_cached;
	for (e = (void *)(p->base + sizeof(*ph)); (char *)e < end;
	    e = (void *)((char *)e + sizeof(*e) + e->padding_hdr + e->size)) {
		if (e->canary != SMEM_CANARY)
			return (EINVAL);
		if (e->item == item)
			return (EEXIST);
	}
	asize = sizeof(*e) + roundup2(size, 8);
	if ((char *)e + asize > cached)
		return (ENOSPC);
	e->canary = SMEM_CANARY;
	e->item = item;
	e->size = roundup2(size, 8);
	e->padding_data = e->size - size;
	e->padding_hdr = 0;
	/* The header before the free offset that publishes it. */
	wmb();
	ph->offset_free_uncached += asize;
	return (0);
}

int
qcom_smem_alloc(u_int host, u_int item, size_t size)
{
	struct smem_part p;
	int error;

	error = qcom_smem_init();
	if (error != 0)
		return (error);
	if (item < SMEM_ITEM_FIXED || item >= smem.items)
		return (EINVAL);
	if (!qcom_smem_find_part(host, &p)) {
		if (smem.version != SMEM_PART_V12)
			return (ENOTSUP);	/* the global heap: not yet */
		p = smem.global;
	}
	error = qcom_smem_lock_hw();
	if (error != 0)
		return (error);
	error = qcom_smem_alloc_private(&p, item, size);
	qcom_smem_unlock_hw();
	return (error);
}

static int
qcom_smem_modevent(module_t mod, int type, void *data)
{

	switch (type) {
	case MOD_LOAD:
		return (0);
	case MOD_UNLOAD:
		/* Mappings may be in use by drivers that called us. */
		return (smem.tried ? EBUSY : 0);
	default:
		return (EOPNOTSUPP);
	}
}

static moduledata_t qcom_smem_mod = { "qcom_smem", qcom_smem_modevent, NULL };
DECLARE_MODULE(qcom_smem, qcom_smem_mod, SI_SUB_DRIVERS, SI_ORDER_FIRST);
MODULE_DEPEND(qcom_smem, acpi, 1, 1, 1);
MODULE_VERSION(qcom_smem, 1);
