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

/*
 * The audio DSP's PRM (proxy resource manager), which powers and clocks
 * LPASS hardware for the application processors: here, the votes that
 * keep the LPASS core and the digital codec on, which Linux's codec macro
 * drivers hold through the "LPASS_HW_MACRO_VOTE" and "LPASS_HW_DCODEC_VOTE"
 * clocks, and the codec clocks themselves.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/sx.h>
#include <sys/sysctl.h>

#include <dev/qcom_audio/qcom_gpr.h>
#include <dev/qcom_audio/qcom_prm.h>

#define	PRM_CMD_REQUEST_HW_RSC		0x0100100f
#define	PRM_CMD_RSP_REQUEST_HW_RSC	0x02001002
#define	PRM_CMD_RELEASE_HW_RSC		0x01001010
#define	PRM_CMD_RSP_RELEASE_HW_RSC	0x02001003
#define	PARAM_ID_RSC_AUDIO_HW_CLK	0x0800102c
#define	PARAM_ID_RSC_HW_CORE		0x08001032

static struct sx qcom_prm_lock;
SX_SYSINIT(qcom_prm, &qcom_prm_lock, "qcom_prm");

/*
 * Send the PRM a request or release of one resource: the APM command
 * header, then a parameter of len bytes.
 */
static int
qcom_prm_rsc(uint32_t param, const uint32_t *data, size_t len, bool on)
{
	struct qcom_gpr_port *port;
	uint32_t req[8 + 6], rsp[2];
	size_t rlen;
	int error;

	KASSERT(len <= sizeof(req) - 8 * 4, ("PRM parameter too long"));
	memset(req, 0, sizeof(req));
	req[3] = 16 + len;		/* payload: the parameter */
	req[4] = QCOM_GPR_PORT_PRM;	/* module instance */
	req[5] = param;
	req[6] = len;
	memcpy(&req[8], data, len);
	sx_xlock(&qcom_prm_lock);
	error = qcom_gpr_port_open(QCOM_GPR_PORT_PRM, NULL, NULL, &port);
	if (error == 0) {
		rlen = sizeof(rsp);
		error = qcom_gpr_cmd(port, QCOM_GPR_PORT_PRM,
		    on ? PRM_CMD_REQUEST_HW_RSC : PRM_CMD_RELEASE_HW_RSC, req,
		    8 * 4 + len, on ? PRM_CMD_RSP_REQUEST_HW_RSC :
		    PRM_CMD_RSP_RELEASE_HW_RSC, rsp, &rlen);
		if (error == 0 && (rlen < sizeof(rsp) || rsp[1] != 0)) {
			printf("qcom_prm: %#x %#x %s: DSP error %#x\n", param,
			    data[0], on ? "on" : "off",
			    rlen >= sizeof(rsp) ? rsp[1] : 0);
			error = EIO;
		}
		qcom_gpr_port_close(port);
	}
	sx_xunlock(&qcom_prm_lock);
	return (error);
}

int
qcom_prm_hw_vote(uint32_t hw, bool on)
{

	return (qcom_prm_rsc(PARAM_ID_RSC_HW_CORE, &hw, sizeof(hw), on));
}

int
qcom_prm_clock(uint32_t id, uint32_t hz)
{
	/* One clock: its ID, rate, attributes (not coupled), default root. */
	uint32_t clk[5] = { 1, id, hz, 1, 0 };

	if (hz == 0)
		return (qcom_prm_rsc(PARAM_ID_RSC_AUDIO_HW_CLK, clk,
		    2 * sizeof(clk[0]), false));
	return (qcom_prm_rsc(PARAM_ID_RSC_AUDIO_HW_CLK, clk, sizeof(clk),
	    true));
}

static int
qcom_prm_vote_sysctl(SYSCTL_HANDLER_ARGS)
{
	int error, v = 0;

	error = sysctl_handle_int(oidp, &v, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	/* hw to vote on, -hw to release. */
	if (v == 0)
		return (EINVAL);
	return (qcom_prm_hw_vote(v > 0 ? v : -v, v > 0));
}

SYSCTL_NODE(_hw, OID_AUTO, qcom_prm, CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
    "The audio DSP's PRM");
SYSCTL_PROC(_hw_qcom_prm, OID_AUTO, vote, CTLTYPE_INT | CTLFLAG_RW |
    CTLFLAG_MPSAFE, NULL, 0, qcom_prm_vote_sysctl, "I",
    "Vote LPASS hardware on (1 core, 2 digital codec), or off (-1, -2)");
