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

#ifndef _DEV_QCOM_AUDIO_QCOM_PRM_H_
#define	_DEV_QCOM_AUDIO_QCOM_PRM_H_

/* The audio DSP's PRM (proxy resource manager): LPASS hardware votes. */
#define	QCOM_PRM_HW_LPASS	1	/* the LPASS core */
#define	QCOM_PRM_HW_DCODEC	2	/* the digital codec (the macros) */

/* LPASS clocks, by the DSP's IDs. */
#define	QCOM_PRM_CLK_TX_CORE_MCLK	0x30c
#define	QCOM_PRM_CLK_TX_CORE_NPL_MCLK	0x30d
#define	QCOM_PRM_CLK_RX_CORE_TX_MCLK	0x312
#define	QCOM_PRM_CLK_RX_CORE_TX_2X_MCLK	0x313

int	qcom_prm_hw_vote(uint32_t hw, bool on);
/* Run a clock at hz, or release it (hz 0). */
int	qcom_prm_clock(uint32_t id, uint32_t hz);

#endif /* _DEV_QCOM_AUDIO_QCOM_PRM_H_ */
