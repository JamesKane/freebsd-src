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

/*
 * The context saved by cpu_suspend_save() and restored in assembly when a
 * core resumes from a powered-down PSCI CPU_SUSPEND state, before any C
 * code runs.  The offsets are into struct cpu_suspend_ctx.
 */
#define	CS_X19		0	/* x19-x30, in pairs */
#define	CS_SP		96
#define	CS_TTBR0	104
#define	CS_TTBR1	112
#define	CS_TCR		120
#define	CS_MAIR		128
#define	CS_SCTLR	136
#define	CS_VBAR		144
#define	CS_TPIDR_EL1	152
#define	CS_SP_EL0	160
#define	CS_CONTEXTIDR	168
#define	CS_TPIDR_EL0	176
#define	CS_TPIDRRO_EL0	184
#define	CS_APIA_LO	192
#define	CS_APIA_HI	200
#define	CS_FLAGS	208
#define	 CS_FLAG_APIA	0x1	/* the kernel APIA key is in use */
#define	CS_ASM_SIZE	216

#ifndef LOCORE
struct cpu_suspend_ctx {
	uint64_t	asm_regs[CS_ASM_SIZE / 8];
	/* Restored in C once the core runs kernel code again. */
	uint64_t	cpacr;
	uint64_t	cntkctl;
	uint64_t	mdscr;
	uint64_t	icc_sre;
	uint64_t	icc_pmr;
	uint64_t	icc_bpr1;
	uint64_t	icc_ctlr;
	uint64_t	icc_igrpen1;
};

#ifdef _KERNEL
int	cpu_suspend_psci(uint32_t power_state);
bool	cpu_suspend_supported(void);

int	cpu_suspend_save(struct cpu_suspend_ctx *);
void	cpu_suspend_resume_entry(void);
#endif
#endif /* !LOCORE */

#endif /* !_MACHINE_CPU_SUSPEND_H_ */
