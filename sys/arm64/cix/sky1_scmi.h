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

#ifndef _ARM64_CIX_SKY1_SCMI_H_
#define	_ARM64_CIX_SKY1_SCMI_H_

/*
 * SCMI on CIX Sky1: requests to the system control processor through the
 * OS agent's channel (ACPI CIXHA006), and power domains through TF-A.
 * Errors are errnos; the SCMI status of a refused request is EIO, or
 * ENOENT for NOT_FOUND.  May sleep.
 */

int	sky1_scmi_request(uint32_t protocol, uint32_t msg, const uint32_t *tx,
	    int ntx, uint32_t *rx, int nrx);

/* Clock protocol (0x14), by SCMI clock id (an ACPI CLKT entry's first). */
int	sky1_scmi_clk_enable(uint32_t id, bool enable);
int	sky1_scmi_clk_get_rate(uint32_t id, uint64_t *hz);
int	sky1_scmi_clk_set_rate(uint32_t id, uint64_t hz);

/* Power domain on or off, through TF-A (SCMI POWER_STATE_SET over SMC). */
int	sky1_scmi_power_set(uint32_t domain, bool on);

#endif /* !_ARM64_CIX_SKY1_SCMI_H_ */
