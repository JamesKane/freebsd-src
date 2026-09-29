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

#ifndef _DEV_QCOM_SMMU_QCOM_SMMU_H_
#define	_DEV_QCOM_SMMU_QCOM_SMMU_H_

/*
 * Arm MMU-500 SMMUs on Qualcomm SoCs, as used by a driver that owns one,
 * such as the Adreno GPU's: stage 1 translation with page tables the
 * driver owns, and the stream mapping that sends the device's transactions
 * through them.  A driver claims the SMMU dedicated to its device.
 *
 * Page tables are separate from context banks, so the GPU can switch a
 * bank between tables (one per process).  Unmapping does not invalidate
 * TLBs; the caller invalidates every bank using the table.
 *
 * qcom_smmu_claim() returns every stream ID firmware lists for the device.
 * The hypervisor checks each stream match entry against the ID and mask
 * pairs it expects, and resets the SoC on any other, even one covering only
 * the device's IDs.  Route streams with the pairs Linux's devicetree uses for
 * the device; on SC8280XP the Adreno GPU's are (0, 0xc00) and (1, 0xc00).
 *
 * The SMMU's registers must be clocked and powered (see qcom_gpucc(4))
 * before it is claimed.
 * Calls on one SMMU must be serialized by the caller, except map and unmap,
 * which lock the page table.
 */

struct qcom_smmu;
struct qcom_smmu_cb;
struct qcom_smmu_pt;

/* Mapping flags. */
#define	QCOM_SMMU_READONLY	0x01
#define	QCOM_SMMU_UNCACHED	0x02
#define	QCOM_SMMU_PRIV		0x04	/* privileged accesses only */

int	qcom_smmu_claim(device_t consumer, struct qcom_smmu **scp,
	    u_int *sids, u_int *nsids);
void	qcom_smmu_release(struct qcom_smmu *sc);

struct qcom_smmu_pt *qcom_smmu_pt_create(void);
void	qcom_smmu_pt_destroy(struct qcom_smmu_pt *pt);
int	qcom_smmu_map(struct qcom_smmu_pt *pt, uint64_t va,
	    vm_paddr_t pa, size_t size, u_int flags);
void	qcom_smmu_unmap(struct qcom_smmu_pt *pt, uint64_t va,
	    size_t size);
vm_paddr_t qcom_smmu_pt_root(struct qcom_smmu_pt *pt);

int	qcom_smmu_cb_alloc(struct qcom_smmu *sc,
	    struct qcom_smmu_pt *pt, struct qcom_smmu_cb **cbp);
void	qcom_smmu_cb_free(struct qcom_smmu_cb *cb);
void	qcom_smmu_cb_set_pt(struct qcom_smmu_cb *cb,
	    struct qcom_smmu_pt *pt);
u_int	qcom_smmu_cb_asid(struct qcom_smmu_cb *cb);
int	qcom_smmu_cb_tlb_inv(struct qcom_smmu_cb *cb);
bool	qcom_smmu_cb_fault(struct qcom_smmu_cb *cb,
	    uint32_t *fsr, uint64_t *far, uint32_t *fsynr0);

int	qcom_smmu_attach_stream(struct qcom_smmu_cb *cb,
	    uint16_t sid, uint16_t mask);
void	qcom_smmu_detach_stream(struct qcom_smmu *sc,
	    uint16_t sid, uint16_t mask);

#endif /* _DEV_QCOM_SMMU_QCOM_SMMU_H_ */
