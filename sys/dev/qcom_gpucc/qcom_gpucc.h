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

#ifndef _DEV_QCOM_GPUCC_QCOM_GPUCC_H_
#define	_DEV_QCOM_GPUCC_QCOM_GPUCC_H_

/*
 * Clocks and power for the always-on (CX) side of a Qualcomm Adreno GPU:
 * the GCC clocks the GPU needs, the GPU clock controller's CX power domain,
 * and the clocks of the GMU, the GPU's power management microcontroller.
 * The GMU firmware then powers the GPU core (GX) itself.
 *
 * The GPU driver owns the clock controller's registers and passes them in,
 * as a resource and the controller's offset within it: ACPI firmware may
 * describe it only as part of a larger window.  The library maps the few GCC
 * registers it needs.  Calls must be serialized by the caller.
 */

struct qcom_gpucc;

struct qcom_gpucc *qcom_gpucc_create(device_t dev, struct resource *res,
	    bus_size_t offset);
void	qcom_gpucc_destroy(struct qcom_gpucc *sc);
int	qcom_gpucc_cx_enable(struct qcom_gpucc *sc);
void	qcom_gpucc_cx_disable(struct qcom_gpucc *sc);

#endif /* _DEV_QCOM_GPUCC_QCOM_GPUCC_H_ */
