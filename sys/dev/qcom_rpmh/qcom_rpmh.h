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

#ifndef _DEV_QCOM_RPMH_QCOM_RPMH_H_
#define	_DEV_QCOM_RPMH_QCOM_RPMH_H_

#define	QCOM_RPMH_ARC_MAX	(~0u)
#define	QCOM_RPMH_BCM_MAX	0x3fff

/* An active-only RPMh write, acknowledged.  Sleeps. */
int	qcom_rpmh_write(uint32_t addr, uint32_t data);
/* Vote a rail ("nsp.lvl") to a level index, or its highest. */
int	qcom_rpmh_arc_vote(const char *res, u_int hlvl);
/* Vote a BCM ("NSA0") to an average and peak bandwidth, in its units. */
int	qcom_rpmh_bcm_vote(const char *bcm, uint32_t avg, uint32_t peak);

#endif /* _DEV_QCOM_RPMH_QCOM_RPMH_H_ */
