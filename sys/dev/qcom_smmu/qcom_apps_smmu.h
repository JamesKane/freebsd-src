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

#ifndef _DEV_QCOM_SMMU_QCOM_APPS_SMMU_H_
#define	_DEV_QCOM_SMMU_QCOM_APPS_SMMU_H_

/*
 * A stream through the Qualcomm "apps" SMMU, translated as Linux does it:
 * a context bank with its own page table, into which memory is mapped at
 * I/O virtual addresses below 4 GB that the device is given.
 */
struct qcom_apps_smmu_dom;

/* The stream, matched with mask (0 for exactly), through a bank of its own. */
int	qcom_apps_smmu_attach(uint16_t sid, uint16_t mask,
	    struct qcom_apps_smmu_dom **dp);
/*
 * Several streams through one bank.  QCOM_APPS_SMMU_IOMMU: the caller places
 * mappings (map_at, unmap_at), possibly with locks held; map and map_pages
 * then fail.
 */
#define	QCOM_APPS_SMMU_MAXSTREAMS	4
#define	QCOM_APPS_SMMU_IOMMU		0x1
int	qcom_apps_smmu_attach_streams(const uint16_t *sids,
	    const uint16_t *masks, u_int nstreams, u_int flags,
	    struct qcom_apps_smmu_dom **dp);
void	qcom_apps_smmu_detach(struct qcom_apps_smmu_dom *d);
int	qcom_apps_smmu_map(struct qcom_apps_smmu_dom *d, vm_paddr_t pa,
	    size_t size, uint64_t *iovap);
/* Pages, wherever they are, at consecutive I/O addresses; uncached. */
int	qcom_apps_smmu_map_pages(struct qcom_apps_smmu_dom *d, vm_page_t *ma,
	    u_int npages, uint64_t *iovap, u_int flags);
#define	QCOM_SMMU_CACHED	0x100	/* map_pages: write-back, snooped */
void	qcom_apps_smmu_unmap(struct qcom_apps_smmu_dom *d, uint64_t iova,
	    size_t size);
int	qcom_apps_smmu_map_at(struct qcom_apps_smmu_dom *d, uint64_t iova,
	    vm_page_t *ma, u_int npages, u_int flags);
void	qcom_apps_smmu_unmap_at(struct qcom_apps_smmu_dom *d, uint64_t iova,
	    size_t size);
/* What an I/O address translates to, or 0. */
vm_paddr_t qcom_apps_smmu_lookup(struct qcom_apps_smmu_dom *d,
	    uint64_t iova);

#endif /* _DEV_QCOM_SMMU_QCOM_APPS_SMMU_H_ */
