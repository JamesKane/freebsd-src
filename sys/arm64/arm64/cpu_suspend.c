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
 * Idle a core in a PSCI CPU_SUSPEND power state that may power it down.
 *
 * Powering down loses the core's registers.  cpu_suspend_save() records
 * what is needed before any C code runs again; PSCI then resumes the core
 * at cpu_suspend_resume_entry, which turns the MMU on, restores that state
 * and returns into cpu_suspend_psci(), which restores the rest.  State kept
 * outside the core (memory, caches, the GIC redistributor, the system
 * counter) is preserved by the platform.
 *
 * The idle thread calls this with interrupts disabled.  The core's generic
 * timer stops while it is powered down, so the caller must use a global
 * event timer.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/pcpu.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/armreg.h>
#include <machine/cpu.h>
#include <machine/cpu_suspend.h>
#include <machine/debug_monitor.h>
#include <machine/machdep.h>
#include <machine/vfp.h>

#include <dev/psci/psci.h>

/* cpu_suspend_save() and the resume path move these in pairs. */
#define	CS_PAIR(a, b)							\
	_Static_assert(offsetof(struct cpu_suspend_ctx, b) ==		\
	    offsetof(struct cpu_suspend_ctx, a) + 8, #a " and " #b)
CS_PAIR(cs_ttbr0, cs_ttbr1);
CS_PAIR(cs_tcr, cs_mair);
CS_PAIR(cs_sctlr, cs_vbar);
CS_PAIR(cs_tpidr_el1, cs_sp_el0);
CS_PAIR(cs_contextidr, cs_tpidr_el0);
CS_PAIR(cs_apia_lo, cs_apia_hi);

DPCPU_DEFINE_STATIC(struct cpu_suspend_ctx, cpu_suspend_ctx);

extern int has_pan;

/* Fixed once the boot CPU is identified; set by cpu_suspend_supported(). */
static bool cpu_suspend_ok;
static bool cpu_suspend_gicv3;
static vm_paddr_t cpu_suspend_entry_pa;

/*
 * Whether cpu_suspend_psci() can be used: PSCI is present and the kernel
 * runs at EL1 (when it runs at EL2 the EL2 state would be lost), and the
 * core has no state this code does not restore (SVE).
 */
bool
cpu_suspend_supported(void)
{
	uint64_t pfr0;

	if (cpu_suspend_entry_pa == 0) {
		get_kernel_reg(ID_AA64PFR0_EL1, &pfr0);
		cpu_suspend_ok = psci_present && !in_vhe() &&
		    ID_AA64PFR0_SVE_VAL(pfr0) == ID_AA64PFR0_SVE_NONE;
		cpu_suspend_gicv3 =
		    ID_AA64PFR0_GIC_VAL(pfr0) != ID_AA64PFR0_GIC_CPUIF_NONE;
		cpu_suspend_entry_pa =
		    pmap_kextract((vm_offset_t)cpu_suspend_resume_entry);
	}
	return (cpu_suspend_ok);
}

/*
 * Enter a PSCI CPU_SUSPEND power state.  Returns 0 once the core runs
 * again, whether it was powered down or the firmware returned without
 * doing so, or an error if the firmware refused the state.  Only to be
 * used once cpu_suspend_supported() has returned true.
 */
int
cpu_suspend_psci(uint32_t power_state)
{
	struct cpu_suspend_ctx *ctx;
	int error;

	/*
	 * The FP registers are lost, so no thread's state may be left in
	 * them: mark them unowned, so the next use reloads the state from
	 * the thread's PCB, and trap that use.
	 */
	PCPU_SET(fpcurthread, NULL);
	vfp_discard(NULL);

	ctx = DPCPU_PTR(cpu_suspend_ctx);
	ctx->cs_cpacr = READ_SPECIALREG(cpacr_el1);
	ctx->cs_cntkctl = READ_SPECIALREG(cntkctl_el1);
	if (cpu_suspend_gicv3) {
		ctx->cs_icc_sre = READ_SPECIALREG(icc_sre_el1);
		ctx->cs_icc_pmr = READ_SPECIALREG(icc_pmr_el1);
		ctx->cs_icc_bpr1 = READ_SPECIALREG(icc_bpr1_el1);
		ctx->cs_icc_ctlr = READ_SPECIALREG(icc_ctlr_el1);
		ctx->cs_icc_igrpen1 = READ_SPECIALREG(icc_igrpen1_el1);
	}
	ctx->cs_flags =
	    (READ_SPECIALREG(sctlr_el1) & SCTLR_EnIA) != 0 ? CS_FLAG_APIA : 0;

	if (cpu_suspend_save(ctx) == 0) {
		error = psci_cpu_suspend(power_state, cpu_suspend_entry_pa,
		    (unsigned long)ctx);
		/* The firmware returned: the core was not powered down. */
		return (error == PSCI_RETVAL_SUCCESS ? 0 : EINVAL);
	}

	/* Resumed from a power-down, with the MMU, stack and pcpu back. */
	if (has_pan)
		__asm __volatile(
		    ".arch_extension pan	\n"
		    "msr pan, #1		\n"
		    ".arch_extension nopan	\n");
	WRITE_SPECIALREG(cpacr_el1, ctx->cs_cpacr);
	WRITE_SPECIALREG(cntkctl_el1, ctx->cs_cntkctl);
	WRITE_SPECIALREG(oslar_el1, 0);
	dbg_register_sync(NULL);
	if (cpu_suspend_gicv3) {
		WRITE_SPECIALREG(icc_sre_el1, ctx->cs_icc_sre);
		isb();
		WRITE_SPECIALREG(icc_pmr_el1, ctx->cs_icc_pmr);
		WRITE_SPECIALREG(icc_bpr1_el1, ctx->cs_icc_bpr1);
		WRITE_SPECIALREG(icc_ctlr_el1, ctx->cs_icc_ctlr);
		WRITE_SPECIALREG(icc_igrpen1_el1, ctx->cs_icc_igrpen1);
	}
	isb();
	return (0);
}
