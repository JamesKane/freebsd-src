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

#ifndef _DEV_QCOM_GLINK_QCOM_GLINK_H_
#define	_DEV_QCOM_GLINK_QCOM_GLINK_H_

/*
 * Qualcomm GLINK: named, message-oriented channels to a remote processor
 * over a pair of shared-memory rings (an "edge").
 */

struct qcom_glink_chan;

/*
 * A message has arrived.  Called from the edge's thread, which may sleep;
 * data is valid only during the call.
 */
typedef void qcom_glink_rx_t(void *arg, const void *data, size_t len);

/*
 * Open the channel named name on the edge labelled edge ("lpass": the audio
 * DSP), offering the remote nintents buffers of intent_size bytes to send
 * into.  Sleeps until the remote has opened it too, or for some seconds.
 */
int	qcom_glink_open(const char *edge, const char *name, size_t intent_size,
	    u_int nintents, qcom_glink_rx_t *rx, void *arg,
	    struct qcom_glink_chan **chp);

/* Send a message, sleeping for the remote's buffer or ring space. */
int	qcom_glink_send(struct qcom_glink_chan *ch, const void *data,
	    size_t len);

void	qcom_glink_close(struct qcom_glink_chan *ch);

#endif /* _DEV_QCOM_GLINK_QCOM_GLINK_H_ */
