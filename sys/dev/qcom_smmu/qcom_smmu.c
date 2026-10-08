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
 * Qualcomm Adreno SMMU; see qcom_smmu.h.
 *
 * The SMMU is an Arm MMU-500 (SMMUv2 architecture).  Its register space is
 * a number of global pages (GR0: configuration, stream matching, global TLB
 * maintenance; GR1: context bank attributes), then one page per context
 * bank.  An incoming stream ID is matched by a stream match register (SMR)
 * and its stream-to-context register (S2CR) selects the context bank that
 * translates it.
 *
 * On Qualcomm SoCs a hypervisor owns stage 2 and the SMMU reports no stage 2
 * context banks.  Writing a context bank's CBAR with the stage 2 type makes
 * the hypervisor reset the SoC, so CBAR is only ever written with the
 * "stage 1, stage 2 bypass" type, and unused banks are only disabled.
 *
 * Page tables use the Armv8 stage 1 format with a 4 KB granule and a 48-bit
 * input address, like the CPU's, so memory attributes use the kernel's MAIR
 * indices.  The SMMU's table walks are cache coherent.
 */

#include "opt_acpi.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bitstring.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/sx.h>

#include <vm/vm.h>
#include <vm/pmap.h>
#include <vm/vm_page.h>

#include <machine/armreg.h>
#include <machine/atomic.h>
#include <machine/bus.h>
#include <machine/cpufunc.h>
#include <machine/pte.h>
#include <machine/vmparam.h>

#ifdef DEV_ACPI
#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>
#endif

#include <dev/qcom_smmu/qcom_smmu.h>

/* GR0: global configuration and stream matching. */
#define	SMMU_sCR0		0x000
#define	 SMMU_sCR0_CLIENTPD	(1u << 0)
#define	SMMU_ID0		0x020
#define	 SMMU_ID0_NUMSMRG(x)	((x) & 0xff)
#define	SMMU_ID1		0x024
#define	 SMMU_ID1_PAGESIZE	(1u << 31)
#define	 SMMU_ID1_NUMPAGENDXB(x) (((x) >> 28) & 0x7)
#define	 SMMU_ID1_NUMCB(x)	((x) & 0xff)
#define	SMMU_ID2		0x028
#define	 SMMU_ID2_OAS(x)	(((x) >> 4) & 0xf)
#define	 SMMU_ID2_UBS(x)	(((x) >> 8) & 0xf)
#define	SMMU_sGFSR		0x048
#define	SMMU_TLBIALLNSNH	0x068
#define	SMMU_TLBGSYNC		0x070
#define	SMMU_TLBGSTATUS		0x074
#define	 SMMU_TLB_SACTIVE	(1u << 0)
#define	SMMU_SMR(n)		(0x800 + (n) * 4)
#define	 SMMU_SMR_VALID		(1u << 31)
#define	 SMMU_SMR_MASK(m)	((uint32_t)(m) << 16)
#define	SMMU_S2CR(n)		(0xc00 + (n) * 4)
#define	 SMMU_S2CR_TRANS(cb)	(cb)		/* type 0: translate */
#define	 SMMU_S2CR_FAULT	(2u << 16)

/* GR1: context bank attributes. */
#define	SMMU_CBAR(n)		((n) * 4)
/* Stage 1 translation, stage 2 bypass; bypass NSH; write-back attributes. */
#define	 SMMU_CBAR_S1		(1u << 16 | 0xfu << 12 | 3u << 8)
#define	SMMU_CBFRSYNRA(n)	(0x400 + (n) * 4)
#define	SMMU_CBA2R(n)		(0x800 + (n) * 4)
#define	 SMMU_CBA2R_VA64	(1u << 0)

/* Context bank registers. */
#define	CB_SCTLR		0x000
#define	 CB_SCTLR_M		(1u << 0)
#define	 CB_SCTLR_TRE		(1u << 1)
#define	 CB_SCTLR_AFE		(1u << 2)
#define	 CB_SCTLR_CFRE		(1u << 5)
#define	 CB_SCTLR_CFIE		(1u << 6)
#define	 CB_SCTLR_ASIDPNE	(1u << 12)
#define	CB_TCR2			0x010
#define	 CB_TCR2_PASIZE(x)	(x)
#define	 CB_TCR2_SEP_UPSTREAM	(7u << 15)
#define	CB_TTBR0		0x020
#define	 CB_TTBR_ASID(a)	((uint64_t)(a) << 48)
#define	CB_TTBR1		0x028
#define	CB_TCR			0x030
#define	 CB_TCR_T0SZ(x)		(x)
#define	 CB_TCR_EPD0		(1u << 7)
#define	 CB_TCR_IRGN0_WBWA	(1u << 8)
#define	 CB_TCR_ORGN0_WBWA	(1u << 10)
#define	 CB_TCR_SH0_IS		(3u << 12)
#define	 CB_TCR_TG0_4K		(0u << 14)
#define	 CB_TCR_T1SZ(x)		((x) << 16)
#define	 CB_TCR_EPD1		(1u << 23)
#define	 CB_TCR_IRGN1_WBWA	(1u << 24)
#define	 CB_TCR_ORGN1_WBWA	(1u << 26)
#define	 CB_TCR_SH1_IS		(3u << 28)
#define	 CB_TCR_TG1_4K		(2u << 30)
#define	CB_CONTEXTIDR		0x034
#define	CB_MAIR0		0x038
#define	CB_MAIR1		0x03c
#define	CB_FSR			0x058
/* Fault bits; FSR also holds the (read-only) descriptor format. */
#define	 CB_FSR_FAULT		0xc00001fe
#define	CB_FAR			0x060
#define	CB_FSYNR0		0x068
#define	CB_FSYNR1		0x06c
#define	CB_TLBIASID		0x610
#define	CB_TLBSYNC		0x7f0
#define	CB_TLBSTATUS		0x7f4

#define	SMMU_POLL_US		10000
#define	QCOM_SMMU_FAULTS_PER_INTR	16

/* Page tables: 4 KB granule, four levels, 48-bit input addresses. */
#define	PT_VA_BITS		48
#define	PT_SPAN			(1ul << PT_VA_BITS)
#define	PT_UPPER_BASE		(~0ul << PT_VA_BITS)	/* upper tables' */
#define	PT_LEVELS		4
#define	PT_SHIFT(lvl)		(39 - 9 * (lvl))
#define	PT_INDEX(va, lvl)	(((va) >> PT_SHIFT(lvl)) & 0x1ff)
#define	PT_DESC			0x3		/* table or page */
#define	PT_ADDR_MASK		0x0000fffffffff000ul

CTASSERT(PAGE_SIZE == 4096);

struct qcom_smmu {
	device_t		dev;
	struct resource		*res;
	bool			claimed;
	bus_size_t		pgsize;
	u_int			ncb;
	u_int			nsmr;
	u_int			pasize;		/* TCR2.PASIZE encoding */
	bus_size_t		cb_base;	/* offset of bank 0 */
	bitstr_t		*cb_used;
	bitstr_t		*smr_used;
	struct qcom_smmu_cb *cbs;
};

struct qcom_smmu_cb {
	struct qcom_smmu	*sc;
	u_int			idx;
	bool			split;		/* TTBR1 holds an upper table */
	uint32_t		tcr;
	qcom_smmu_fault_fn	*fault_fn;
	void			*fault_arg;
	struct resource		*irq;
	int			irq_rid;
	void			*irq_cookie;
};

struct qcom_smmu_pt {
	struct sx		lock;
	struct mtx		mtx;	/* QCOM_SMMU_PT_NOSLEEP's */
	vm_page_t		root;
	bool			upper;
	bool			nosleep;
};

static MALLOC_DEFINE(M_QCOM_SMMU, "qcom_smmu",
    "Qualcomm Adreno SMMU");

/* Register access. */

static uint32_t
gr0_read(struct qcom_smmu *sc, bus_size_t reg)
{
	return (bus_read_4(sc->res, reg));
}

static void
gr0_write(struct qcom_smmu *sc, bus_size_t reg, uint32_t v)
{
	bus_write_4(sc->res, reg, v);
}

static uint32_t
gr1_read(struct qcom_smmu *sc, bus_size_t reg)
{
	return (bus_read_4(sc->res, sc->pgsize + reg));
}

static void
gr1_write(struct qcom_smmu *sc, bus_size_t reg, uint32_t v)
{
	bus_write_4(sc->res, sc->pgsize + reg, v);
}

static bus_size_t
cb_reg(struct qcom_smmu *sc, u_int idx, bus_size_t reg)
{
	return (sc->cb_base + idx * sc->pgsize + reg);
}

static uint32_t
cb_read(struct qcom_smmu *sc, u_int idx, bus_size_t reg)
{
	return (bus_read_4(sc->res, cb_reg(sc, idx, reg)));
}

static void
cb_write(struct qcom_smmu *sc, u_int idx, bus_size_t reg, uint32_t v)
{
	bus_write_4(sc->res, cb_reg(sc, idx, reg), v);
}

static void
cb_write8(struct qcom_smmu *sc, u_int idx, bus_size_t reg, uint64_t v)
{
	bus_write_8(sc->res, cb_reg(sc, idx, reg), v);
}

static int
smmu_poll(struct qcom_smmu *sc, bus_size_t reg, const char *what)
{
	int i;

	for (i = 0; i < SMMU_POLL_US; i++) {
		if ((bus_read_4(sc->res, reg) & SMMU_TLB_SACTIVE) == 0)
			return (0);
		DELAY(1);
	}
	device_printf(sc->dev, "%s timed out\n", what);
	return (ETIMEDOUT);
}

static int
smmu_tlb_inv_all(struct qcom_smmu *sc)
{
	gr0_write(sc, SMMU_TLBIALLNSNH, 0);
	gr0_write(sc, SMMU_TLBGSYNC, 0);
	return (smmu_poll(sc, SMMU_TLBGSTATUS, "TLB sync"));
}

/* Page tables. */

static uint64_t *
pt_table(vm_page_t m)
{
	return ((uint64_t *)PHYS_TO_DMAP(VM_PAGE_TO_PHYS(m)));
}

/* NULL only with nosleep, when no page is free. */
static vm_page_t
pt_alloc_table(bool nosleep)
{
	return (vm_page_alloc_noobj((nosleep ? 0 : VM_ALLOC_WAITOK) |
	    VM_ALLOC_WIRED | VM_ALLOC_ZERO));
}

static void
pt_lock(struct qcom_smmu_pt *pt)
{
	if (pt->nosleep)
		mtx_lock(&pt->mtx);
	else
		sx_xlock(&pt->lock);
}

static void
pt_unlock(struct qcom_smmu_pt *pt)
{
	if (pt->nosleep)
		mtx_unlock(&pt->mtx);
	else
		sx_xunlock(&pt->lock);
}

static void
pt_free_table(uint64_t *table, int lvl)
{
	vm_page_t m;
	u_int i;

	if (lvl < PT_LEVELS - 1) {
		for (i = 0; i < Ln_ENTRIES; i++) {
			if ((table[i] & PT_DESC) == PT_DESC)
				pt_free_table((uint64_t *)PHYS_TO_DMAP(
				    table[i] & PT_ADDR_MASK), lvl + 1);
		}
	}
	m = PHYS_TO_VM_PAGE(DMAP_TO_PHYS((vm_offset_t)table));
	vm_page_unwire_noq(m);
	vm_page_free(m);
}

struct qcom_smmu_pt *
qcom_smmu_pt_create(u_int flags)
{
	struct qcom_smmu_pt *pt;

	pt = malloc(sizeof(*pt), M_QCOM_SMMU, M_WAITOK | M_ZERO);
	sx_init(&pt->lock, "adreno smmu pt");
	mtx_init(&pt->mtx, "qcom smmu pt", NULL, MTX_DEF);
	pt->root = pt_alloc_table(false);
	pt->upper = (flags & QCOM_SMMU_PT_UPPER) != 0;
	pt->nosleep = (flags & QCOM_SMMU_PT_NOSLEEP) != 0;
	return (pt);
}

void
qcom_smmu_pt_destroy(struct qcom_smmu_pt *pt)
{
	if (pt == NULL)
		return;
	pt_free_table(pt_table(pt->root), 0);
	sx_destroy(&pt->lock);
	mtx_destroy(&pt->mtx);
	free(pt, M_QCOM_SMMU);
}

vm_paddr_t
qcom_smmu_pt_root(struct qcom_smmu_pt *pt)
{
	return (VM_PAGE_TO_PHYS(pt->root));
}

/* The last-level entry for va, allocating tables on the way if asked. */
static uint64_t *
pt_lookup(struct qcom_smmu_pt *pt, uint64_t va, bool alloc)
{
	uint64_t *table, *e;
	vm_page_t m;
	int lvl;

	if (pt->nosleep)
		mtx_assert(&pt->mtx, MA_OWNED);
	else
		sx_assert(&pt->lock, SA_XLOCKED);
	table = pt_table(pt->root);
	for (lvl = 0; lvl < PT_LEVELS - 1; lvl++) {
		e = &table[PT_INDEX(va, lvl)];
		if ((*e & PT_DESC) != PT_DESC) {
			if (!alloc)
				return (NULL);
			if ((m = pt_alloc_table(pt->nosleep)) == NULL)
				return (NULL);
			/* The walker is coherent: order the zeroing first. */
			dsb(ishst);
			*e = VM_PAGE_TO_PHYS(m) | PT_DESC;
		}
		table = (uint64_t *)PHYS_TO_DMAP(*e & PT_ADDR_MASK);
	}
	return (&table[PT_INDEX(va, PT_LEVELS - 1)]);
}

/*
 * Whether [va, va + size) lies in the range the table translates.  Lookups
 * index with bits 47:12 only, so upper tables need no other conversion.
 */
static bool
pt_range_ok(struct qcom_smmu_pt *pt, uint64_t va, size_t size)
{
	uint64_t base;

	base = pt->upper ? PT_UPPER_BASE : 0;
	return (size != 0 && va >= base && va - base < PT_SPAN &&
	    size - 1 <= PT_SPAN - 1 - (va - base));
}

int
qcom_smmu_map(struct qcom_smmu_pt *pt, uint64_t va,
    vm_paddr_t pa, size_t size, u_int flags)
{
	uint64_t attr, *e;
	size_t done;
	int error;

	if (((va | pa | size) & PAGE_MASK) != 0 || !pt_range_ok(pt, va, size))
		return (EINVAL);
	/*
	 * Non-global (ASID-tagged), shareable, executable: the GPU fetches
	 * shader code and the GMU its firmware through here.  Pages writable
	 * by unprivileged accesses are never executable by privileged ones,
	 * so privileged masters that run code from memory need QCOM_SMMU_PRIV.
	 */
	attr = ATTR_AF | ATTR_S1_nG | ATTR_SH(ATTR_SH_IS) | PT_DESC;
	if ((flags & QCOM_SMMU_PRIV) == 0)
		attr |= ATTR_S1_AP(ATTR_S1_AP_USER);
	if ((flags & QCOM_SMMU_READONLY) != 0)
		attr |= ATTR_S1_AP(ATTR_S1_AP_RO);
	attr |= ATTR_S1_IDX((flags & QCOM_SMMU_UNCACHED) != 0 ?
	    VM_MEMATTR_UNCACHEABLE : VM_MEMATTR_WRITE_BACK);

	error = 0;
	pt_lock(pt);
	for (done = 0; done < size; done += PAGE_SIZE) {
		e = pt_lookup(pt, va + done, true);
		if (e == NULL) {
			error = ENOMEM;
			break;
		}
		if ((*e & PT_DESC) == PT_DESC) {
			error = EEXIST;
			break;
		}
		*e = (pa + done) | attr;
	}
	dsb(ishst);
	pt_unlock(pt);
	if (error != 0 && done > 0)
		qcom_smmu_unmap(pt, va, done);
	return (error);
}

void
qcom_smmu_unmap(struct qcom_smmu_pt *pt, uint64_t va,
    size_t size)
{
	uint64_t *e;
	size_t done;

	/* Out of range, lookups would clear the entries of other addresses. */
	if (((va | size) & PAGE_MASK) != 0 || !pt_range_ok(pt, va, size)) {
		KASSERT(false, ("%s: bad range %#jx+%#zx", __func__,
		    (uintmax_t)va, size));
		return;
	}
	pt_lock(pt);
	for (done = 0; done < size; done += PAGE_SIZE) {
		e = pt_lookup(pt, va + done, false);
		if (e != NULL)
			*e = 0;
	}
	dsb(ishst);
	pt_unlock(pt);
}

vm_paddr_t
qcom_smmu_lookup(struct qcom_smmu_pt *pt, uint64_t va)
{
	uint64_t *e;
	vm_paddr_t pa;

	if (!pt_range_ok(pt, trunc_page(va), PAGE_SIZE))
		return (0);
	pt_lock(pt);
	e = pt_lookup(pt, trunc_page(va), false);
	pa = e != NULL && (*e & PT_DESC) == PT_DESC ?
	    (*e & PT_ADDR_MASK) | (va & PAGE_MASK) : 0;
	pt_unlock(pt);
	return (pa);
}

/* Context banks. */

/*
 * Point TTBR0 at a table, or with root 0, turn it off in a split bank.
 * Walks through TTBR0 are only enabled once it holds the table.
 */
static void
cb_program_ttbr0(struct qcom_smmu_cb *cb, vm_paddr_t root)
{
	if (cb->split && root == 0) {
		cb->tcr |= CB_TCR_EPD0;
		cb_write(cb->sc, cb->idx, CB_TCR, cb->tcr);
	}
	cb_write8(cb->sc, cb->idx, CB_TTBR0, root | CB_TTBR_ASID(cb->idx));
	if (cb->split && root != 0) {
		cb->tcr &= ~CB_TCR_EPD0;
		cb_write(cb->sc, cb->idx, CB_TCR, cb->tcr);
	}
}

int
qcom_smmu_cb_alloc(struct qcom_smmu *sc,
    struct qcom_smmu_pt *pt, struct qcom_smmu_cb **cbp)
{
	struct qcom_smmu_cb *cb;
	uint64_t mair;
	int error, idx;

	bit_ffc(sc->cb_used, sc->ncb, &idx);
	if (idx < 0)
		return (ENOSPC);
	bit_set(sc->cb_used, idx);
	cb = &sc->cbs[idx];
	cb->sc = sc;
	cb->idx = idx;
	/*
	 * A bank given an upper table is split: TTBR1 translates the top of
	 * the address space with it, and TTBR0, off until a table is set with
	 * qcom_smmu_cb_set_ttbr0(), the bottom.
	 */
	cb->split = pt->upper;
	cb->tcr = CB_TCR_T0SZ(64 - PT_VA_BITS) | CB_TCR_IRGN0_WBWA |
	    CB_TCR_ORGN0_WBWA | CB_TCR_SH0_IS | CB_TCR_TG0_4K;
	if (cb->split)
		cb->tcr |= CB_TCR_EPD0 | CB_TCR_T1SZ(64 - PT_VA_BITS) |
		    CB_TCR_IRGN1_WBWA | CB_TCR_ORGN1_WBWA | CB_TCR_SH1_IS |
		    CB_TCR_TG1_4K;
	else
		cb->tcr |= CB_TCR_EPD1;

	/* Attributes first, then the bank's registers (as Linux does). */
	cb_write(sc, idx, CB_SCTLR, 0);
	gr1_write(sc, SMMU_CBA2R(idx), SMMU_CBA2R_VA64);
	gr1_write(sc, SMMU_CBAR(idx), SMMU_CBAR_S1);
	cb_write(sc, idx, CB_TCR2, CB_TCR2_PASIZE(sc->pasize) |
	    CB_TCR2_SEP_UPSTREAM);
	cb_write(sc, idx, CB_TCR, cb->tcr);
	cb_write(sc, idx, CB_CONTEXTIDR, idx);
	/* The ASID is the bank's index; a split bank's TTBR0 starts off. */
	if (cb->split) {
		cb_write8(sc, idx, CB_TTBR0, CB_TTBR_ASID(idx));
		cb_write8(sc, idx, CB_TTBR1, qcom_smmu_pt_root(pt));
	} else
		cb_program_ttbr0(cb, qcom_smmu_pt_root(pt));
	mair = READ_SPECIALREG(mair_el1);
	cb_write(sc, idx, CB_MAIR0, (uint32_t)mair);
	cb_write(sc, idx, CB_MAIR1, (uint32_t)(mair >> 32));
	cb_write(sc, idx, CB_FSR, CB_FSR_FAULT);
	/* Faulting transactions are terminated, not stalled. */
	cb_write(sc, idx, CB_SCTLR, CB_SCTLR_M | CB_SCTLR_TRE | CB_SCTLR_AFE |
	    CB_SCTLR_CFRE | CB_SCTLR_ASIDPNE);
	error = qcom_smmu_cb_tlb_inv(cb);
	if (error != 0) {
		qcom_smmu_cb_free(cb);
		return (error);
	}
	*cbp = cb;
	return (0);
}

void
qcom_smmu_cb_free(struct qcom_smmu_cb *cb)
{
	struct qcom_smmu *sc;

	if (cb == NULL)
		return;
	sc = cb->sc;
	(void)qcom_smmu_cb_set_fault_handler(cb, NULL, NULL);
	cb_write(sc, cb->idx, CB_SCTLR, 0);
	(void)qcom_smmu_cb_tlb_inv(cb);
	bit_clear(sc->cb_used, cb->idx);
}

/*
 * Set a split bank's TTBR0 to the table with this root, which may belong to
 * another driver's page table code, or with root 0, turn TTBR0 off.
 */
int
qcom_smmu_cb_set_ttbr0(struct qcom_smmu_cb *cb, vm_paddr_t root)
{
	if (!cb->split || (root & ~PT_ADDR_MASK) != 0)
		return (EINVAL);
	cb_program_ttbr0(cb, root);
	return (qcom_smmu_cb_tlb_inv(cb));
}

u_int
qcom_smmu_cb_index(struct qcom_smmu_cb *cb)
{
	return (cb->idx);
}

int
qcom_smmu_cb_tlb_inv(struct qcom_smmu_cb *cb)
{
	struct qcom_smmu *sc = cb->sc;

	dsb(ishst);
	cb_write(sc, cb->idx, CB_TLBIASID, cb->idx);
	cb_write(sc, cb->idx, CB_TLBSYNC, 0);
	return (smmu_poll(sc, cb_reg(sc, cb->idx, CB_TLBSTATUS),
	    "context TLB sync"));
}

/* Read the bank's fault syndrome, if it has a fault recorded. */
static bool
cb_read_fault(struct qcom_smmu_cb *cb, struct qcom_smmu_fault *f)
{
	struct qcom_smmu *sc = cb->sc;

	f->fsr = cb_read(sc, cb->idx, CB_FSR) & CB_FSR_FAULT;
	if (f->fsr == 0)
		return (false);
	f->far = bus_read_8(sc->res, cb_reg(sc, cb->idx, CB_FAR));
	f->ttbr0 = bus_read_8(sc->res, cb_reg(sc, cb->idx, CB_TTBR0));
	f->fsynr0 = cb_read(sc, cb->idx, CB_FSYNR0);
	f->fsynr1 = cb_read(sc, cb->idx, CB_FSYNR1);
	f->contextidr = cb_read(sc, cb->idx, CB_CONTEXTIDR);
	f->cbfrsynra = gr1_read(sc, SMMU_CBFRSYNRA(cb->idx));
	return (true);
}

static void
cb_intr(void *arg)
{
	struct qcom_smmu_cb *cb = arg;
	struct qcom_smmu_fault f;
	int n;

	/*
	 * The interrupt is edge-triggered and stays asserted while FSR has
	 * any fault bit set, so handle faults until it reads clear, or a
	 * fault taken in the meantime would silence the interrupt for good.
	 * A master that keeps faulting only gets a few reported per
	 * interrupt.  Faults terminate, so nothing is left to resume.
	 */
	for (n = 0; n < QCOM_SMMU_FAULTS_PER_INTR && cb_read_fault(cb, &f);
	    n++) {
		cb->fault_fn(cb->fault_arg, &f);
		cb_write(cb->sc, cb->idx, CB_FSR, f.fsr);
	}
	if (n == QCOM_SMMU_FAULTS_PER_INTR)
		cb_write(cb->sc, cb->idx, CB_FSR, CB_FSR_FAULT);
}

/*
 * Call fn from an interrupt thread with each fault the bank takes, or with
 * fn NULL, stop.  On ACPI systems, IORT lists the SMMU's context interrupts
 * in bank order, and so do the SMMU device's interrupt resources.
 */
int
qcom_smmu_cb_set_fault_handler(struct qcom_smmu_cb *cb,
    qcom_smmu_fault_fn *fn, void *arg)
{
	struct qcom_smmu *sc = cb->sc;
	uint32_t sctlr;
	int error;

	sctlr = cb_read(sc, cb->idx, CB_SCTLR);
	if (cb->irq != NULL) {
		cb_write(sc, cb->idx, CB_SCTLR, sctlr & ~CB_SCTLR_CFIE);
		bus_teardown_intr(sc->dev, cb->irq, cb->irq_cookie);
		bus_release_resource(sc->dev, SYS_RES_IRQ, cb->irq_rid,
		    cb->irq);
		cb->irq = NULL;
	}
	cb->fault_fn = fn;
	cb->fault_arg = arg;
	if (fn == NULL)
		return (0);

	cb->irq_rid = cb->idx;
	cb->irq = bus_alloc_resource_any(sc->dev, SYS_RES_IRQ, &cb->irq_rid,
	    RF_ACTIVE);
	if (cb->irq == NULL)
		return (ENXIO);
	error = bus_setup_intr(sc->dev, cb->irq, INTR_TYPE_MISC | INTR_MPSAFE,
	    NULL, cb_intr, cb, &cb->irq_cookie);
	if (error != 0) {
		bus_release_resource(sc->dev, SYS_RES_IRQ, cb->irq_rid,
		    cb->irq);
		cb->irq = NULL;
		return (error);
	}
	cb_write(sc, cb->idx, CB_FSR, CB_FSR_FAULT);
	cb_write(sc, cb->idx, CB_SCTLR, sctlr | CB_SCTLR_CFIE);
	return (0);
}

/* Streams. */

int
qcom_smmu_attach_stream(struct qcom_smmu_cb *cb, uint16_t sid,
    uint16_t mask)
{
	struct qcom_smmu *sc = cb->sc;
	int n;

	bit_ffc(sc->smr_used, sc->nsmr, &n);
	if (n < 0)
		return (ENOSPC);
	bit_set(sc->smr_used, n);
	/* Route first, then make the match valid. */
	gr0_write(sc, SMMU_S2CR(n), SMMU_S2CR_TRANS(cb->idx));
	gr0_write(sc, SMMU_SMR(n), SMMU_SMR_VALID | SMMU_SMR_MASK(mask) | sid);
	return (0);
}

void
qcom_smmu_detach_stream(struct qcom_smmu *sc, uint16_t sid,
    uint16_t mask)
{
	uint32_t want;
	u_int n;

	want = SMMU_SMR_VALID | SMMU_SMR_MASK(mask) | sid;
	for (n = 0; n < sc->nsmr; n++) {
		if (bit_test(sc->smr_used, n) &&
		    gr0_read(sc, SMMU_SMR(n)) == want) {
			gr0_write(sc, SMMU_SMR(n), 0);
			gr0_write(sc, SMMU_S2CR(n), SMMU_S2CR_FAULT);
			bit_clear(sc->smr_used, n);
		}
	}
}

/* Setup. */

/* Read the SMMU's configuration; its owner has powered it. */
static int
qcom_smmu_read_config(struct qcom_smmu *sc)
{
	uint32_t id0, id1, id2;

	if (sc->cbs != NULL)
		return (0);
	id0 = gr0_read(sc, SMMU_ID0);
	id1 = gr0_read(sc, SMMU_ID1);
	id2 = gr0_read(sc, SMMU_ID2);
	sc->pgsize = (id1 & SMMU_ID1_PAGESIZE) != 0 ? 65536 : 4096;
	sc->cb_base = sc->pgsize << (SMMU_ID1_NUMPAGENDXB(id1) + 1);
	sc->ncb = SMMU_ID1_NUMCB(id1);
	sc->nsmr = SMMU_ID0_NUMSMRG(id0);
	sc->pasize = SMMU_ID2_OAS(id2);
	/* OAS and TCR2.PASIZE share an encoding; 5 is 48 bits, the most. */
	if (sc->pasize > 5 || SMMU_ID2_UBS(id2) < 5 || sc->ncb == 0 ||
	    sc->nsmr == 0 ||
	    sc->cb_base + sc->ncb * sc->pgsize > rman_get_size(sc->res) ||
	    (gr0_read(sc, SMMU_sCR0) & SMMU_sCR0_CLIENTPD) != 0) {
		device_printf(sc->dev, "unsupported SMMU (sCR0 %#x ID0 %#x "
		    "ID1 %#x ID2 %#x)\n", gr0_read(sc, SMMU_sCR0), id0, id1,
		    id2);
		return (ENXIO);
	}
	sc->cb_used = bit_alloc(sc->ncb, M_QCOM_SMMU, M_WAITOK);
	sc->smr_used = bit_alloc(sc->nsmr, M_QCOM_SMMU, M_WAITOK);
	sc->cbs = mallocarray(sc->ncb, sizeof(*sc->cbs), M_QCOM_SMMU,
	    M_WAITOK | M_ZERO);
	return (0);
}

/* Reset for a new owner: clear stream mapping, disable every bank. */
static int
qcom_smmu_reset(struct qcom_smmu *sc)
{
	u_int i;

	for (i = 0; i < sc->nsmr; i++) {
		gr0_write(sc, SMMU_SMR(i), 0);
		gr0_write(sc, SMMU_S2CR(i), SMMU_S2CR_FAULT);
	}
	for (i = 0; i < sc->ncb; i++) {
		cb_write(sc, i, CB_SCTLR, 0);
		cb_write(sc, i, CB_FSR, CB_FSR_FAULT);
	}
	bit_nclear(sc->cb_used, 0, sc->ncb - 1);
	bit_nclear(sc->smr_used, 0, sc->nsmr - 1);
	gr0_write(sc, SMMU_sGFSR, gr0_read(sc, SMMU_sGFSR));
	return (smmu_tlb_inv_all(sc));
}

/*
 * Claim the SMMU that ACPI's IORT shows is used by the consumer device
 * alone, and get the stream IDs the consumer's transactions carry on it.
 * SMMUs shared with other devices are never claimed: firmware set them up
 * and other drivers depend on that.  The SMMU must be powered.
 */
int
qcom_smmu_claim(device_t consumer, struct qcom_smmu **scp, u_int *sids,
    u_int *nsids)
{
#ifdef DEV_ACPI
	struct qcom_smmu *sc;
	ACPI_HANDLE h;
	devclass_t dc;
	device_t dev;
	uint64_t base;
	u_int idx, n;
	bool shared;
	int error, i;

	h = acpi_get_handle(consumer);
	dc = devclass_find("qcom_smmu");
	if (h == NULL || dc == NULL)
		return (ENXIO);
	for (idx = 0;; idx++) {
		n = *nsids;
		error = acpi_iort_named_smmu(h, idx, &base, &shared, sids, &n);
		if (error == ENOENT)
			return (ENXIO);
		if (error != 0)
			return (error);
		if (shared)
			continue;
		for (i = 0; i < devclass_get_maxunit(dc); i++) {
			dev = devclass_get_device(dc, i);
			if (dev == NULL || !device_is_attached(dev))
				continue;
			sc = device_get_softc(dev);
			if (rman_get_start(sc->res) != base)
				continue;
			if (sc->claimed)
				return (EBUSY);
			if ((error = qcom_smmu_read_config(sc)) != 0 ||
			    (error = qcom_smmu_reset(sc)) != 0)
				return (error);
			sc->claimed = true;
			*scp = sc;
			*nsids = n;
			return (0);
		}
	}
#else
	return (ENXIO);
#endif
}

/* Give the SMMU back, with its stream mapping cleared. */
void
qcom_smmu_release(struct qcom_smmu *sc)
{
	u_int i;

	if (sc == NULL || !sc->claimed)
		return;
	/* The banks' interrupts would outlive them. */
	for (i = 0; i < sc->ncb; i++)
		if (bit_test(sc->cb_used, i))
			(void)qcom_smmu_cb_set_fault_handler(&sc->cbs[i],
			    NULL, NULL);
	(void)qcom_smmu_reset(sc);
	sc->claimed = false;
}

#ifdef DEV_ACPI
static char *qcom_smmu_acpi_ids[] = { "QCOM0609", NULL };
#endif

static int
qcom_smmu_probe(device_t dev)
{
	int rv;

	rv = ENXIO;
#ifdef DEV_ACPI
	rv = ACPI_ID_PROBE(device_get_parent(dev), dev, qcom_smmu_acpi_ids,
	    NULL);
#endif
	if (rv <= 0)
		device_set_desc(dev, "Qualcomm MMU-500 SMMU");
	return (rv);
}

/*
 * Only maps the registers: the SMMU may not be powered yet, and it is left
 * as firmware set it up until a driver claims it.
 */
static int
qcom_smmu_attach(device_t dev)
{
	struct qcom_smmu *sc = device_get_softc(dev);
	int rid;

	sc->dev = dev;
	rid = 0;
	sc->res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->res == NULL) {
		device_printf(dev, "cannot map registers\n");
		return (ENXIO);
	}
	return (0);
}

static int
qcom_smmu_detach(device_t dev)
{
	struct qcom_smmu *sc = device_get_softc(dev);

	if (sc->claimed)
		return (EBUSY);
	free(sc->cbs, M_QCOM_SMMU);
	free(sc->cb_used, M_QCOM_SMMU);
	free(sc->smr_used, M_QCOM_SMMU);
	bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->res);
	return (0);
}

static device_method_t qcom_smmu_methods[] = {
	DEVMETHOD(device_probe,		qcom_smmu_probe),
	DEVMETHOD(device_attach,	qcom_smmu_attach),
	DEVMETHOD(device_detach,	qcom_smmu_detach),
	DEVMETHOD_END
};

static driver_t qcom_smmu_driver = {
	"qcom_smmu",
	qcom_smmu_methods,
	sizeof(struct qcom_smmu),
};

#ifdef DEV_ACPI
DRIVER_MODULE(qcom_smmu, acpi, qcom_smmu_driver, 0, 0);
#endif
MODULE_VERSION(qcom_smmu, 1);
