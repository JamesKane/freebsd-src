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

#ifndef _DEV_QCOM_GLINK_QCOM_SMEM_H_
#define	_DEV_QCOM_GLINK_QCOM_SMEM_H_

/*
 * Qualcomm shared memory (SMEM): an allocate-only heap the SoC's processors
 * share, of numbered items, in partitions private to a pair of processors
 * ("hosts") and a global one.
 */

#define	QCOM_SMEM_HOST_APPS	0	/* this processor */
#define	QCOM_SMEM_HOST_ADSP	2
#define	QCOM_SMEM_HOST_CDSP	5

/* Find an item: its address and size, or ENOENT. */
int	qcom_smem_get(u_int host, u_int item, void **ptr, size_t *size);

/* Allocate an item, or EEXIST if it has been. */
int	qcom_smem_alloc(u_int host, u_int item, size_t size);

#endif /* _DEV_QCOM_GLINK_QCOM_SMEM_H_ */
