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

#ifndef _MACHINE_CPU_SUSPEND_H_
#define	_MACHINE_CPU_SUSPEND_H_

#define	CS_FLAG_APIA	0x1	/* cs_flags: the kernel APIA key is in use */

#ifndef LOCORE
/*
 * The context of a core that may be powered down in a PSCI CPU_SUSPEND
 * state.  The first part is saved by cpu_suspend_save() and restored in
 * assembly before any C code runs; its offsets come from genassym.c, and
 * fields the assembly moves in pairs must stay adjacent.
 */
struct cpu_suspend_ctx {
	uint64_t	cs_x[12];	/* x19-x30 */
	uint64_t	cs_sp;
	uint64_t	cs_ttbr0;
	uint64_t	cs_ttbr1;
	uint64_t	cs_tcr;
	uint64_t	cs_mair;
	uint64_t	cs_sctlr;
	uint64_t	cs_vbar;
	uint64_t	cs_tpidr_el1;
	uint64_t	cs_sp_el0;
	uint64_t	cs_contextidr;
	uint64_t	cs_tpidr_el0;
	uint64_t	cs_tpidrro_el0;
	uint64_t	cs_apia_lo;
	uint64_t	cs_apia_hi;
	uint64_t	cs_flags;
	/* Restored in C once the core runs kernel code again. */
	uint64_t	cs_cpacr;
	uint64_t	cs_cntkctl;
	uint64_t	cs_icc_sre;
	uint64_t	cs_icc_pmr;
	uint64_t	cs_icc_bpr1;
	uint64_t	cs_icc_ctlr;
	uint64_t	cs_icc_igrpen1;
};

#ifdef _KERNEL
int	cpu_suspend_psci(uint32_t power_state);
bool	cpu_suspend_supported(void);

int	cpu_suspend_save(struct cpu_suspend_ctx *);
void	cpu_suspend_resume_entry(void);
#endif
#endif /* !LOCORE */

#endif /* !_MACHINE_CPU_SUSPEND_H_ */
