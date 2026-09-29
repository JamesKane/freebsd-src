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

#ifndef _DEV_QCOM_SCM_QCOM_SCM_H_
#define	_DEV_QCOM_SCM_QCOM_SCM_H_

/*
 * Calls into the Qualcomm Secure Channel Manager, the TrustZone firmware
 * interface.  All functions may sleep and return an errno value; they fail
 * with ENXIO until the qcom_scm(4) driver has attached.
 */

/* Peripheral authentication service (PAS) IDs. */
#define	QCOM_SCM_PAS_GPU		13	/* the GPU's zap shader */

/* Services and commands, for qcom_scm_is_call_available(). */
#define	QCOM_SCM_SVC_BOOT		0x01
#define	QCOM_SCM_BOOT_SET_REMOTE_STATE	0x0a
#define	QCOM_SCM_SVC_PIL		0x02
#define	QCOM_SCM_PIL_PAS_INIT_IMAGE	0x01
#define	QCOM_SCM_PIL_PAS_MEM_SETUP	0x02
#define	QCOM_SCM_PIL_PAS_AUTH_AND_RESET	0x05
#define	QCOM_SCM_PIL_PAS_SHUTDOWN	0x06
#define	QCOM_SCM_PIL_PAS_IS_SUPPORTED	0x07
#define	QCOM_SCM_SVC_INFO		0x06
#define	QCOM_SCM_INFO_IS_CALL_AVAIL	0x01
#define	QCOM_SCM_SVC_MP			0x0c
#define	QCOM_SCM_MP_CP_SMMU_APERTURE_ID	0x1b

bool	qcom_scm_available(void);
bool	qcom_scm_is_call_available(uint32_t svc, uint32_t cmd);

bool	qcom_scm_pas_supported(uint32_t pas_id);
int	qcom_scm_pas_init_image(uint32_t pas_id, const void *metadata,
	    size_t len);
int	qcom_scm_pas_mem_setup(uint32_t pas_id, vm_paddr_t addr,
	    vm_size_t size);
int	qcom_scm_pas_auth_and_reset(uint32_t pas_id);
int	qcom_scm_pas_shutdown(uint32_t pas_id);
int	qcom_scm_set_remote_state(uint32_t state, uint32_t id);
bool	qcom_scm_set_gpu_smmu_aperture_is_available(void);
int	qcom_scm_set_gpu_smmu_aperture(u_int context_bank);

#endif /* _DEV_QCOM_SCM_QCOM_SCM_H_ */
