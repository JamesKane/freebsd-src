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

#ifndef _DEV_QCOM_SMMU_QCOM_APPS_IOMMU_H_
#define	_DEV_QCOM_SMMU_QCOM_APPS_IOMMU_H_

/*
 * The Qualcomm "apps" SMMU as an iommu(4) unit, for the devices its driver
 * names: a device claims its streams, and its busdma tag (from its parent's
 * bus_get_dma_tag(), which on ACPI asks iommu(4)) then translates them
 * through a context bank of its own, at I/O addresses within a window.
 * Nothing describes these streams to FreeBSD (Windows' ACPI leaves the SMMU
 * to the hypervisor), so the device's driver supplies them: the ID and mask
 * pairs Linux's devicetree uses, which the hypervisor checks.
 *
 * Claim before the device's first bus_get_dma_tag().  Needs options IOMMU.
 */

#define	QCOM_APPS_IOMMU_COHERENT	0x1	/* the device snoops the caches */

int	qcom_apps_iommu_claim(device_t dev, const uint16_t *sids,
	    const uint16_t *masks, u_int nstreams, uint64_t iova_start,
	    uint64_t iova_end, u_int flags);
void	qcom_apps_iommu_unclaim(device_t dev);

#endif /* _DEV_QCOM_SMMU_QCOM_APPS_IOMMU_H_ */
