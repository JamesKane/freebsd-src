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
 * The Qualcomm "apps" SMMU, an Arm MMU-500, as the firmware leaves it:
 * enabled, faulting streams it doesn't match, with stream match entries
 * for the firmware's own devices pointing at context banks with
 * translation off.  Nothing in ACPI describes it (Windows leaves it to the
 * hypervisor), and FreeBSD doesn't otherwise drive it.
 *
 * A stream another processor needs into our memory gets what Linux gives
 * it: a free context bank translating through a page table of its own
 * (qcom_smmu(4)'s), a free stream-to-context entry pointing at it, and last
 * the stream match entry, with the ID and mask Linux's devicetree uses,
 * that makes it live.  The audio DSP's stream crashed the DSP, and with it
 * the SoC, through a bank with translation off.
 *
 * Where it is comes from a table of SoCs, which the DSDT's \_SB.SOID
 * identifies.
 */

#include "opt_acpi.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mutex.h>

#include <machine/atomic.h>

#include <vm/vm.h>
#include <vm/pmap.h>
#include <vm/vm_page.h>

#include <machine/vmparam.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include <machine/cpu.h>

#include <dev/qcom_smmu/qcom_smmu.h>
#include <dev/qcom_audio/qcom_apps_smmu.h>

#define	SMMU_IDR0		0x020
#define	 IDR0_NUMSMRG(x)	((x) & 0xff)
#define	SMMU_IDR1		0x024
#define	 IDR1_NUMCB(x)		((x) & 0xff)
#define	 IDR1_NUMPAGENDXB(x)	(((x) >> 28) & 0x7)
#define	 IDR1_PAGESIZE_64K	(1u << 31)
#define	SMMU_IDR2		0x028
#define	 IDR2_OAS(x)		(((x) >> 4) & 0xf)
#define	SMMU_SMR(n)		(0x800 + 4 * (n))
#define	 SMR_VALID		(1u << 31)
#define	 SMR_ID(x)		((x) & 0x7fff)
#define	 SMR_MASK(x)		(((x) >> 16) & 0x7fff)
#define	SMMU_S2CR(n)		(0xc00 + 4 * (n))
#define	 S2CR_TYPE(x)		(((x) >> 16) & 0x3)
#define	 S2CR_TYPE_TRANS	0
#define	 S2CR_TYPE_FAULT	2
#define	 S2CR_CBNDX(x)		((x) & 0xff)
#define	SMMU_CBAR(n)		(0x1000 + 4 * (n))	/* in GR1 */
#define	 CBAR_S1		(1u << 16 | 0xfu << 12 | 3u << 8)
#define	SMMU_CBA2R(n)		(0x1800 + 4 * (n))
#define	 CBA2R_VA64		0x1

/* A context bank's registers */
#define	CB_SCTLR		0x000
#define	 CB_SCTLR_M		(1u << 0)
#define	 CB_SCTLR_TRE		(1u << 1)
#define	 CB_SCTLR_AFE		(1u << 2)
#define	 CB_SCTLR_CFRE		(1u << 5)
#define	 CB_SCTLR_CFCFG		(1u << 7)	/* stall on a fault */
#define	 CB_SCTLR_ASIDPNE	(1u << 12)
#define	CB_TCR2			0x010
#define	 CB_TCR2_SEP_UPSTREAM	(7u << 15)
#define	CB_TTBR0		0x020
#define	 CB_TTBR_ASID(x)	((uint64_t)(x) << 48)
#define	CB_TCR			0x030
#define	 CB_TCR_T0SZ(x)		(x)
#define	 CB_TCR_IRGN0_WBWA	(1u << 8)
#define	 CB_TCR_ORGN0_WBWA	(1u << 10)
#define	 CB_TCR_SH0_IS		(3u << 12)
#define	 CB_TCR_EPD1		(1u << 23)
#define	CB_CONTEXTIDR		0x034
#define	CB_MAIR0		0x038
#define	CB_MAIR1		0x03c
#define	CB_FSR			0x058
#define	CB_TLBIASID		0x610
#define	CB_TLBSYNC		0x7f0
#define	CB_TLBSTATUS		0x7f4
#define	 CB_TLBSTATUS_ACTIVE	0x1

#define	IOVA_BASE		0x10000000ul	/* below 4 GB, as Linux's */
#define	IOVA_END		0x100000000ul	/* within 36 bits */

struct qcom_apps_smmu_soc {
	uint32_t	id;		/* \_SB.SOID */
	vm_paddr_t	base;
	vm_size_t	size;
};

static const struct qcom_apps_smmu_soc qcom_apps_smmu_socs[] = {
	{ 449, 0x15000000, 0x100000 },	/* SC8280XP */
};

struct qcom_apps_smmu_dom {
	uint16_t		sid;
	u_int			cb;
	struct qcom_smmu_pt	*pt;
	vm_paddr_t		l1;	/* the walk's first table */
	uint64_t		next;	/* the next I/O address to give */
	u_int			nmaps;
};

static struct mtx qcom_apps_smmu_mtx;
MTX_SYSINIT(qcom_apps_smmu, &qcom_apps_smmu_mtx, "qcom_apps_smmu", MTX_DEF);
static volatile uint32_t *qcom_apps_smmu_regs;
/* Stall faulting transactions, for looking at them: hw.qcom_apps_smmu_stall. */
static int qcom_apps_smmu_stall;
TUNABLE_INT("hw.qcom_apps_smmu_stall", &qcom_apps_smmu_stall);
static u_int qcom_apps_smmu_cbbase;

static const struct qcom_apps_smmu_soc *
qcom_apps_smmu_find_soc(void)
{
	UINT32 id;
	u_int i;

	if (ACPI_FAILURE(acpi_GetInteger(ACPI_ROOT_OBJECT, "\\_SB.SOID", &id)))
		return (NULL);
	for (i = 0; i < nitems(qcom_apps_smmu_socs); i++)
		if (qcom_apps_smmu_socs[i].id == id)
			return (&qcom_apps_smmu_socs[i]);
	return (NULL);
}

#define	RD(off)		(qcom_apps_smmu_regs[(off) / 4])
#define	WR(off, v)	(qcom_apps_smmu_regs[(off) / 4] = (v))
#define	CB(cb, reg)	(qcom_apps_smmu_cbbase + (cb) * 0x1000 + (reg))

/* Set up a translating bank for the stream, or find the one it has. */
int
qcom_apps_smmu_attach(uint16_t sid, struct qcom_apps_smmu_dom **dp)
{
	const struct qcom_apps_smmu_soc *soc;
	struct qcom_apps_smmu_dom *d;
	uint32_t idr1, smr, used[8];
	uint64_t mair, ttbr;
	u_int nsmr, ncb, i, cb, free_smr, own_smr;

	mtx_lock(&qcom_apps_smmu_mtx);
	if (qcom_apps_smmu_regs == NULL) {
		soc = qcom_apps_smmu_find_soc();
		if (soc == NULL) {
			mtx_unlock(&qcom_apps_smmu_mtx);
			return (ENXIO);
		}
		qcom_apps_smmu_regs = pmap_mapdev(soc->base, soc->size);
	}
	nsmr = MIN(IDR0_NUMSMRG(RD(SMMU_IDR0)), 128);
	idr1 = RD(SMMU_IDR1);
	ncb = MIN(IDR1_NUMCB(idr1), nitems(used) * 32);
	/* Context banks follow the global pages. */
	qcom_apps_smmu_cbbase = (1u << (IDR1_NUMPAGENDXB(idr1) + 1)) *
	    ((idr1 & IDR1_PAGESIZE_64K) != 0 ? 65536 : 4096);

	memset(used, 0, sizeof(used));
	free_smr = own_smr = nsmr;
	cb = ncb;
	for (i = 0; i < nsmr; i++) {
		smr = RD(SMMU_SMR(i));
		if ((smr & SMR_VALID) == 0) {
			if (free_smr == nsmr &&
			    S2CR_TYPE(RD(SMMU_S2CR(i))) == S2CR_TYPE_FAULT)
				free_smr = i;
			continue;
		}
		if ((SMR_ID(smr) & ~SMR_MASK(smr)) == (sid & ~SMR_MASK(smr))) {
			/*
			 * Set up by an earlier load of this driver, which
			 * leaves it: take its bank over.  Anything else
			 * already routing the stream is not ours to change.
			 */
			if (SMR_ID(smr) != sid || SMR_MASK(smr) != 0 ||
			    S2CR_TYPE(RD(SMMU_S2CR(i))) != S2CR_TYPE_TRANS) {
				mtx_unlock(&qcom_apps_smmu_mtx);
				return (EEXIST);
			}
			own_smr = i;
			cb = S2CR_CBNDX(RD(SMMU_S2CR(i)));
			continue;
		}
		if (S2CR_TYPE(RD(SMMU_S2CR(i))) == S2CR_TYPE_TRANS)
			used[S2CR_CBNDX(RD(SMMU_S2CR(i))) / 32] |=
			    1u << (S2CR_CBNDX(RD(SMMU_S2CR(i))) % 32);
	}
	if (own_smr != nsmr)
		free_smr = own_smr;
	else
		for (cb = 0; cb < ncb; cb++)
			if ((used[cb / 32] & (1u << (cb % 32))) == 0 &&
			    RD(SMMU_CBAR(cb)) == 0)
				break;
	if (free_smr == nsmr || cb == ncb) {
		mtx_unlock(&qcom_apps_smmu_mtx);
		return (ENOSPC);
	}
	mtx_unlock(&qcom_apps_smmu_mtx);

	d = malloc(sizeof(*d), M_DEVBUF, M_WAITOK | M_ZERO);
	d->sid = sid;
	d->cb = cb;
	d->pt = qcom_smmu_pt_create(0);
	d->next = IOVA_BASE;
	/*
	 * This SMMU takes 36-bit addresses (IDR2's IAS and UBS), as Linux
	 * sets it up: the walk starts at level 1, so the bank is given the
	 * level-1 table below the 48-bit table's first entry, which a first
	 * mapping creates.
	 */
	{
		uint64_t *root;
		vm_page_t m;

		m = vm_page_alloc_noobj(VM_ALLOC_WAITOK | VM_ALLOC_WIRED |
		    VM_ALLOC_ZERO);
		(void)qcom_smmu_map(d->pt, IOVA_BASE, VM_PAGE_TO_PHYS(m),
		    PAGE_SIZE, QCOM_SMMU_UNCACHED);
		qcom_smmu_unmap(d->pt, IOVA_BASE, PAGE_SIZE);
		vm_page_unwire_noq(m);
		vm_page_free(m);
		root = (uint64_t *)PHYS_TO_DMAP(qcom_smmu_pt_root(d->pt));
		d->l1 = root[0] & 0x0000fffffffff000ul;
	}

	mtx_lock(&qcom_apps_smmu_mtx);
	/* The bank, as qcom_smmu(4) sets the GPU's up; ASID is its index. */
	WR(CB(cb, CB_SCTLR), 0);
	WR(SMMU_CBA2R(cb), CBA2R_VA64);
	WR(SMMU_CBAR(cb), CBAR_S1);
	WR(CB(cb, CB_TCR2), IDR2_OAS(RD(SMMU_IDR2)) | CB_TCR2_SEP_UPSTREAM);
	WR(CB(cb, CB_TCR), CB_TCR_T0SZ(28) | CB_TCR_IRGN0_WBWA |
	    CB_TCR_ORGN0_WBWA | CB_TCR_SH0_IS | CB_TCR_EPD1);
	WR(CB(cb, CB_CONTEXTIDR), cb);
	ttbr = d->l1 | CB_TTBR_ASID(cb);
	WR(CB(cb, CB_TTBR0), (uint32_t)ttbr);
	WR(CB(cb, CB_TTBR0) + 4, (uint32_t)(ttbr >> 32));
	mair = READ_SPECIALREG(mair_el1);
	WR(CB(cb, CB_MAIR0), (uint32_t)mair);
	WR(CB(cb, CB_MAIR1), (uint32_t)(mair >> 32));
	WR(CB(cb, CB_FSR), RD(CB(cb, CB_FSR)));
	WR(CB(cb, CB_SCTLR), CB_SCTLR_M | CB_SCTLR_TRE | CB_SCTLR_AFE |
	    CB_SCTLR_CFRE | CB_SCTLR_ASIDPNE |
	    (qcom_apps_smmu_stall ? CB_SCTLR_CFCFG : 0));
	if (own_smr != nsmr) {
		/* Already routed here: drop what the old tables left. */
		WR(CB(cb, CB_TLBIASID), cb);
		WR(CB(cb, CB_TLBSYNC), 0);
		for (i = 0; i < 100000 &&
		    (RD(CB(cb, CB_TLBSTATUS)) & CB_TLBSTATUS_ACTIVE) != 0; i++)
			DELAY(1);
	} else {
		/* Then what points at it, then what makes it live. */
		WR(SMMU_S2CR(free_smr), cb);
		wmb();
		WR(SMMU_SMR(free_smr), SMR_VALID | sid);
		wmb();
	}
	mtx_unlock(&qcom_apps_smmu_mtx);
	printf("qcom_apps_smmu: stream %#x through context bank %u "
	    "(entry %u), translated\n", sid, cb, free_smr);
	*dp = d;
	return (0);
}

static void
qcom_apps_smmu_tlb_flush(struct qcom_apps_smmu_dom *d)
{
	int i;

	mtx_lock(&qcom_apps_smmu_mtx);
	WR(CB(d->cb, CB_TLBIASID), d->cb);
	WR(CB(d->cb, CB_TLBSYNC), 0);
	for (i = 0; i < 100000 &&
	    (RD(CB(d->cb, CB_TLBSTATUS)) & CB_TLBSTATUS_ACTIVE) != 0; i++)
		DELAY(1);
	mtx_unlock(&qcom_apps_smmu_mtx);
}

int
qcom_apps_smmu_map(struct qcom_apps_smmu_dom *d, vm_paddr_t pa, size_t size,
    uint64_t *iovap)
{
	uint64_t iova;
	int error;

	size = roundup2(size, PAGE_SIZE);
	mtx_lock(&qcom_apps_smmu_mtx);
	if (d->nmaps == 0)
		d->next = IOVA_BASE;
	iova = d->next;
	if (iova + size > IOVA_END) {
		mtx_unlock(&qcom_apps_smmu_mtx);
		return (ENOSPC);
	}
	d->next += size;
	d->nmaps++;
	mtx_unlock(&qcom_apps_smmu_mtx);
	/* Uncached: the CPU writes the buffers through a write-combining map. */
	error = qcom_smmu_map(d->pt, iova, pa, size, QCOM_SMMU_UNCACHED);
	if (error != 0) {
		mtx_lock(&qcom_apps_smmu_mtx);
		d->nmaps--;
		mtx_unlock(&qcom_apps_smmu_mtx);
		return (error);
	}
	*iovap = iova;
	return (0);
}

void
qcom_apps_smmu_unmap(struct qcom_apps_smmu_dom *d, uint64_t iova, size_t size)
{

	qcom_smmu_unmap(d->pt, iova, roundup2(size, PAGE_SIZE));
	qcom_apps_smmu_tlb_flush(d);
	mtx_lock(&qcom_apps_smmu_mtx);
	d->nmaps--;
	mtx_unlock(&qcom_apps_smmu_mtx);
}
