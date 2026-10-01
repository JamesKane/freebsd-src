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

#ifndef _DEV_QCOM_GLINK_QCOM_IPCC_H_
#define	_DEV_QCOM_GLINK_QCOM_IPCC_H_

/*
 * The Qualcomm inter-processor communication controller (IPCC): doorbells,
 * each a signal number of a client (a processor).
 */

#define	QCOM_IPCC_CLIENT_LPASS	3	/* the audio DSP */
#define	QCOM_IPCC_CLIENT_CDSP	6	/* the compute DSP */
#define	QCOM_IPCC_SIGNAL_GLINK	0

typedef void qcom_ipcc_handler_t(void *arg);

/* Call fn, in an interrupt thread, when client rings signal. */
int	qcom_ipcc_register(u_int client, u_int signal, qcom_ipcc_handler_t *fn,
	    void *arg);
void	qcom_ipcc_unregister(u_int client, u_int signal);

/* Ring client's signal. */
int	qcom_ipcc_send(u_int client, u_int signal);

#endif /* _DEV_QCOM_GLINK_QCOM_IPCC_H_ */
