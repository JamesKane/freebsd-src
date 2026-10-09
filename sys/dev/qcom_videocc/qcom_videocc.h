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

#ifndef _DEV_QCOM_VIDEOCC_QCOM_VIDEOCC_H_
#define	_DEV_QCOM_VIDEOCC_QCOM_VIDEOCC_H_

/*
 * Clocks and power for a Qualcomm video codec (Venus/Iris): the RPMh rails
 * it runs from, the GCC clocks it needs, and the video clock controller's
 * PLL, core clocks and power domains.  Under ACPI the firmware hides all of
 * these behind the Windows power engine plug-in.
 *
 * The codec has two power domains, as Linux's driver sees them: the
 * controller (its CPU and interfaces: MVS0C) and the hardware (the codec
 * core: MVS0).  The controller's must be up before the hardware's.  Calls
 * must be serialized by the caller.
 */

struct qcom_videocc;

/* NULL if this SoC has no description, or the registers can't be mapped. */
struct qcom_videocc *qcom_videocc_create(device_t dev);
void	qcom_videocc_destroy(struct qcom_videocc *sc);
int	qcom_videocc_ctrl_enable(struct qcom_videocc *sc);
void	qcom_videocc_ctrl_disable(struct qcom_videocc *sc);
int	qcom_videocc_hw_enable(struct qcom_videocc *sc);
void	qcom_videocc_hw_disable(struct qcom_videocc *sc);
/* The core's power domain under the codec's control (hw) or ours. */
int	qcom_videocc_hw_set_hwmode(struct qcom_videocc *sc, bool hw);
/*
 * The core clock to the slowest level at least hz (the fastest, if none is),
 * with the rails it needs; the rate it would get; the rate it has.
 */
int	qcom_videocc_set_rate(struct qcom_videocc *sc, u_long hz);
u_long	qcom_videocc_round_rate(struct qcom_videocc *sc, u_long hz);
u_long	qcom_videocc_get_rate(struct qcom_videocc *sc);

#endif /* _DEV_QCOM_VIDEOCC_QCOM_VIDEOCC_H_ */
