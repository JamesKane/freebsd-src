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

#ifndef _DEV_QCOM_AUDIO_QCOM_APM_H_
#define	_DEV_QCOM_AUDIO_QCOM_APM_H_

struct qcom_apm_play;

/*
 * A period has been consumed: its token, and the DSP's status for it (0 if
 * fine).  Called from the DSP link's thread, which must not wait on the DSP.
 */
typedef void qcom_apm_done_t(void *arg, u_int token, uint32_t status);

bool	qcom_apm_supported(void);

/* 48 kHz 16-bit stereo to the headphones, from a contiguous buffer. */
int	qcom_apm_play_open(vm_offset_t buf, size_t size, qcom_apm_done_t *done,
	    void *arg, struct qcom_apm_play **pp);
int	qcom_apm_play_write(struct qcom_apm_play *p, size_t off, size_t len,
	    u_int token);
int	qcom_apm_play_volume(struct qcom_apm_play *p, uint16_t gain);
void	qcom_apm_play_close(struct qcom_apm_play *p);

#endif /* _DEV_QCOM_AUDIO_QCOM_APM_H_ */
