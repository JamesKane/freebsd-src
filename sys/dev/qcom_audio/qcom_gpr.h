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

#ifndef _DEV_QCOM_AUDIO_QCOM_GPR_H_
#define	_DEV_QCOM_AUDIO_QCOM_GPR_H_

/*
 * Qualcomm GPR (generic packet router): packets between ports on the
 * application processors and the audio DSP's services, over GLINK.
 */

#define	QCOM_GPR_DOMAIN_ADSP		2
#define	QCOM_GPR_DOMAIN_APPS		3

/* The DSP's services' ports, which their clients use as theirs too. */
#define	QCOM_GPR_PORT_APM		1
#define	QCOM_GPR_PORT_PRM		2

/* A command's result: struct qcom_gpr_result. */
#define	QCOM_GPR_BASIC_RSP_RESULT	0x02001005
#define	QCOM_GPR_BASIC_EVT_ACCEPTED	0x02001006

struct qcom_gpr_hdr {
	uint32_t	w0;		/* version 3:0, header words 7:4, */
					/* packet bytes 31:8 */
	uint32_t	domains;	/* destination 7:0, source 15:8 */
	uint32_t	src_port;
	uint32_t	dst_port;
	uint32_t	token;
	uint32_t	opcode;
};

struct qcom_gpr_result {
	uint32_t	opcode;		/* of the command */
	uint32_t	status;		/* 0, or the DSP's error */
};

struct qcom_gpr_port;

/* A packet for the port that no command was waiting for. */
typedef void qcom_gpr_rx_t(void *arg, const struct qcom_gpr_hdr *hdr,
    const void *payload, size_t len);

int	qcom_gpr_port_open(uint32_t port, qcom_gpr_rx_t *rx, void *arg,
	    struct qcom_gpr_port **pp);
void	qcom_gpr_port_close(struct qcom_gpr_port *p);

/* Send a packet to dst_port on the DSP. */
int	qcom_gpr_send(struct qcom_gpr_port *p, uint32_t dst_port,
	    uint32_t opcode, uint32_t token, const void *payload, size_t len);

/*
 * Send a command and sleep for its answer: a packet with rsp_opcode,
 * whose payload is copied to rsp (up to *rsplen, which is set to its
 * size), or a basic result for the command, whose status is returned as
 * EIO if it isn't 0.  rsp_opcode 0 waits for the basic result only.
 */
int	qcom_gpr_cmd(struct qcom_gpr_port *p, uint32_t dst_port,
	    uint32_t opcode, const void *payload, size_t len,
	    uint32_t rsp_opcode, void *rsp, size_t *rsplen);

#endif /* _DEV_QCOM_AUDIO_QCOM_GPR_H_ */
