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

#ifndef _DEV_QCOM_AUDIO_QCOM_SWR_H_
#define	_DEV_QCOM_AUDIO_QCOM_SWR_H_

#define	QCOM_SWR_RX	0	/* playback */
#define	QCOM_SWR_TX	1	/* capture, and the codec's registers */

struct qcom_swr;

/* Bring a link up, or out of clock stop; the codec macros must be clocked. */
int	qcom_swr_up(u_int which, struct qcom_swr **sp);
/* Stop its clock, before the macros' go. */
int	qcom_swr_stop(struct qcom_swr *s);
void	qcom_swr_forget(void);
int	qcom_swr_wake_intr(device_t dev, u_int which, driver_intr_t *fn,
	    void *arg, struct resource **resp, void **cookiep);
bool	qcom_swr_wake_take(u_int which);
uint32_t qcom_swr_attached(struct qcom_swr *s);
uint64_t qcom_swr_dev_id(struct qcom_swr *s, u_int n);
int	qcom_swr_read(struct qcom_swr *s, u_int dev, uint16_t reg,
	    uint8_t *val);
int	qcom_swr_write(struct qcom_swr *s, u_int dev, uint16_t reg,
	    uint8_t val);
void	qcom_swr_mmio_write(struct qcom_swr *s, u_int reg, uint32_t val);
int	qcom_swr_broadcast(struct qcom_swr *s, uint16_t reg, uint8_t val);
int	qcom_swr_bank_switch(struct qcom_swr *s, uint16_t reg);

#endif /* _DEV_QCOM_AUDIO_QCOM_SWR_H_ */
