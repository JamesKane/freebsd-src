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
 * Qualcomm GPR, the generic packet router: the audio DSP's services (the
 * APM, which runs audio graphs, and the PRM, which manages the audio
 * clocks and power) talk to ports on the application processors through
 * it, over the GLINK channel "adsp_apps".  A packet is a header (size,
 * source and destination domain and port, a token, an opcode) and a
 * payload; packets are routed to the port they're addressed to.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/condvar.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/queue.h>
#include <sys/sx.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>

#include <dev/qcom_glink/qcom_glink.h>
#include <dev/qcom_audio/qcom_gpr.h>

static MALLOC_DEFINE(M_GPR, "qcom_gpr", "Qualcomm GPR");

#define	GPR_VERSION		0
#define	GPR_HDR_WORDS		(sizeof(struct qcom_gpr_hdr) / 4)
#define	GPR_MAX_PKT		4096
#define	GPR_INTENT_SIZE		512	/* as Linux offers the DSP */
#define	GPR_INTENTS		20
#define	GPR_WAIT		(5 * hz)
#define	GPR_START_TRIES		60

/* A port the APM state sysctl asks from. */
#define	GPR_PORT_SYSCTL		0x7f
#define	APM_CMD_GET_SPF_STATE		0x01001021
#define	APM_CMD_RSP_GET_SPF_STATE	0x02001007

struct qcom_gpr_port {
	LIST_ENTRY(qcom_gpr_port) link;
	uint32_t	id;
	qcom_gpr_rx_t	*rx;
	void		*arg;
	struct sx	cmd_lock;	/* one command at a time */
	/* The command waiting for its answer, under gpr.mtx. */
	bool		waiting;
	bool		answered;
	uint32_t	token;
	uint32_t	opcode;
	uint32_t	rsp_opcode;
	void		*rsp;
	size_t		rsplen;
	int		status;
};

static struct {
	struct mtx	mtx;
	struct cv	cv;
	LIST_HEAD(, qcom_gpr_port) ports;
	struct qcom_glink_chan *ch;
	struct taskqueue *tq;		/* opening the channel may sleep long */
	struct timeout_task start_task;
	int		start_tries;
	uint32_t	next_token;
} gpr;

static struct qcom_gpr_port *
gpr_port_find(uint32_t id)
{
	struct qcom_gpr_port *p;

	LIST_FOREACH(p, &gpr.ports, link)
		if (p->id == id)
			return (p);
	return (NULL);
}

static void
gpr_rx(void *arg __unused, const void *data, size_t len)
{
	const struct qcom_gpr_hdr *h = data;
	const struct qcom_gpr_result *res;
	struct qcom_gpr_port *p;
	const void *payload;
	qcom_gpr_rx_t *rx;
	size_t hlen, plen;
	void *rxarg;

	if (len < sizeof(*h) || (h->w0 & 0xf) > GPR_VERSION + 1 ||
	    (h->w0 >> 8) != len) {
		printf("qcom_gpr: malformed packet of %zu bytes\n", len);
		return;
	}
	hlen = ((h->w0 >> 4) & 0xf) * 4;
	if (hlen < sizeof(*h) || hlen > len) {
		printf("qcom_gpr: malformed header\n");
		return;
	}
	payload = (const char *)data + hlen;
	plen = len - hlen;

	mtx_lock(&gpr.mtx);
	p = gpr_port_find(h->dst_port);
	if (p == NULL) {
		mtx_unlock(&gpr.mtx);
		printf("qcom_gpr: opcode %#x for unknown port %#x\n",
		    h->opcode, h->dst_port);
		return;
	}
	res = payload;
	if (p->waiting && !p->answered && h->token == p->token &&
	    ((p->rsp_opcode != 0 && h->opcode == p->rsp_opcode) ||
	    (h->opcode == QCOM_GPR_BASIC_RSP_RESULT &&
	    plen >= sizeof(*res) && res->opcode == p->opcode))) {
		if (h->opcode == QCOM_GPR_BASIC_RSP_RESULT) {
			p->status = res->status;
			p->rsplen = 0;
		} else {
			p->status = 0;
			p->rsplen = MIN(plen, p->rsplen);
			memcpy(p->rsp, payload, p->rsplen);
		}
		p->answered = true;
		cv_broadcast(&gpr.cv);
		mtx_unlock(&gpr.mtx);
		return;
	}
	rx = p->rx;
	rxarg = p->arg;
	mtx_unlock(&gpr.mtx);
	if (rx != NULL)
		rx(rxarg, h, payload, plen);
}

int
qcom_gpr_port_open(uint32_t id, qcom_gpr_rx_t *rx, void *arg,
    struct qcom_gpr_port **pp)
{
	struct qcom_gpr_port *p;

	p = malloc(sizeof(*p), M_GPR, M_WAITOK | M_ZERO);
	p->id = id;
	p->rx = rx;
	p->arg = arg;
	sx_init(&p->cmd_lock, "qcom_gpr cmd");
	mtx_lock(&gpr.mtx);
	if (gpr_port_find(id) != NULL) {
		mtx_unlock(&gpr.mtx);
		sx_destroy(&p->cmd_lock);
		free(p, M_GPR);
		return (EEXIST);
	}
	LIST_INSERT_HEAD(&gpr.ports, p, link);
	mtx_unlock(&gpr.mtx);
	*pp = p;
	return (0);
}

void
qcom_gpr_port_close(struct qcom_gpr_port *p)
{

	sx_xlock(&p->cmd_lock);
	mtx_lock(&gpr.mtx);
	LIST_REMOVE(p, link);
	mtx_unlock(&gpr.mtx);
	sx_xunlock(&p->cmd_lock);
	sx_destroy(&p->cmd_lock);
	free(p, M_GPR);
}

static int
gpr_send(struct qcom_gpr_port *p, uint32_t dst_port, uint32_t opcode,
    uint32_t token, const void *payload, size_t len)
{
	struct qcom_glink_chan *ch;
	struct qcom_gpr_hdr *h;
	size_t size;
	int error;

	size = sizeof(*h) + len;
	if (size > GPR_MAX_PKT)
		return (EMSGSIZE);
	mtx_lock(&gpr.mtx);
	ch = gpr.ch;
	mtx_unlock(&gpr.mtx);
	if (ch == NULL)
		return (ENOTCONN);
	h = malloc(size, M_GPR, M_WAITOK);
	h->w0 = GPR_VERSION | GPR_HDR_WORDS << 4 | size << 8;
	h->domains = QCOM_GPR_DOMAIN_ADSP | QCOM_GPR_DOMAIN_APPS << 8;
	h->src_port = p->id;
	h->dst_port = dst_port;
	h->token = token;
	h->opcode = opcode;
	if (len != 0)
		memcpy(h + 1, payload, len);
	error = qcom_glink_send(ch, h, size);
	free(h, M_GPR);
	return (error);
}

int
qcom_gpr_send(struct qcom_gpr_port *p, uint32_t dst_port, uint32_t opcode,
    uint32_t token, const void *payload, size_t len)
{

	return (gpr_send(p, dst_port, opcode, token, payload, len));
}

int
qcom_gpr_cmd(struct qcom_gpr_port *p, uint32_t dst_port, uint32_t opcode,
    const void *payload, size_t len, uint32_t rsp_opcode, void *rsp,
    size_t *rsplen)
{
	int error, status;

	sx_xlock(&p->cmd_lock);
	mtx_lock(&gpr.mtx);
	p->token = ++gpr.next_token;
	p->opcode = opcode;
	p->rsp_opcode = rsp_opcode;
	p->rsp = rsp;
	p->rsplen = rsplen != NULL ? *rsplen : 0;
	p->answered = false;
	p->waiting = true;
	mtx_unlock(&gpr.mtx);

	error = gpr_send(p, dst_port, opcode, p->token, payload, len);
	mtx_lock(&gpr.mtx);
	while (error == 0 && !p->answered)
		if (cv_timedwait(&gpr.cv, &gpr.mtx, GPR_WAIT) == EWOULDBLOCK &&
		    !p->answered)
			error = ETIMEDOUT;
	p->waiting = false;
	status = p->status;
	if (error == 0 && rsplen != NULL)
		*rsplen = p->rsplen;
	mtx_unlock(&gpr.mtx);
	sx_xunlock(&p->cmd_lock);
	if (error == 0 && status != 0) {
		printf("qcom_gpr: opcode %#x to port %#x: DSP error %#x\n",
		    opcode, dst_port, status);
		error = EIO;
	}
	return (error);
}

static int
gpr_apm_state_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct qcom_gpr_port *p;
	uint32_t cmd[4] = { 0 };	/* an APM command header, no payload */
	uint32_t state;
	size_t len;
	int error;

	error = qcom_gpr_port_open(GPR_PORT_SYSCTL, NULL, NULL, &p);
	if (error != 0)
		return (error);
	len = sizeof(state);
	state = 0;
	error = qcom_gpr_cmd(p, QCOM_GPR_PORT_APM, APM_CMD_GET_SPF_STATE, cmd,
	    sizeof(cmd), APM_CMD_RSP_GET_SPF_STATE, &state, &len);
	qcom_gpr_port_close(p);
	if (error != 0)
		return (error);
	return (sysctl_handle_32(oidp, &state, 0, req));
}

SYSCTL_NODE(_hw, OID_AUTO, qcom_gpr, CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
    "Qualcomm GPR");
SYSCTL_PROC(_hw_qcom_gpr, OID_AUTO, apm_state, CTLTYPE_U32 | CTLFLAG_RD |
    CTLFLAG_MPSAFE, NULL, 0, gpr_apm_state_sysctl, "IU",
    "The audio DSP's APM state (1: ready)");

/* The channel opens once GLINK's edge to the DSP is up. */
static void
gpr_start(void *arg __unused, int pending __unused)
{
	struct qcom_glink_chan *ch;
	int error;

	error = qcom_glink_open("lpass", "adsp_apps", GPR_INTENT_SIZE,
	    GPR_INTENTS, gpr_rx, NULL, &ch);
	if (error == 0) {
		mtx_lock(&gpr.mtx);
		gpr.ch = ch;
		cv_broadcast(&gpr.cv);
		mtx_unlock(&gpr.mtx);
		return;
	}
	if (++gpr.start_tries < GPR_START_TRIES)
		taskqueue_enqueue_timeout(gpr.tq, &gpr.start_task, hz);
	else
		printf("qcom_gpr: no channel to the DSP: %d\n", error);
}

static int
qcom_gpr_modevent(module_t mod, int type, void *data)
{
	struct qcom_glink_chan *ch;

	switch (type) {
	case MOD_LOAD:
		mtx_init(&gpr.mtx, "qcom_gpr", NULL, MTX_DEF);
		cv_init(&gpr.cv, "qcom_gpr");
		LIST_INIT(&gpr.ports);
		gpr.tq = taskqueue_create("qcom_gpr", M_WAITOK,
		    taskqueue_thread_enqueue, &gpr.tq);
		taskqueue_start_threads(&gpr.tq, 1, PWAIT, "qcom_gpr");
		TIMEOUT_TASK_INIT(gpr.tq, &gpr.start_task, 0, gpr_start, NULL);
		taskqueue_enqueue_timeout(gpr.tq, &gpr.start_task, 0);
		return (0);
	case MOD_UNLOAD:
		if (!LIST_EMPTY(&gpr.ports))
			return (EBUSY);
		while (taskqueue_cancel_timeout(gpr.tq, &gpr.start_task,
		    NULL) != 0)
			taskqueue_drain_timeout(gpr.tq, &gpr.start_task);
		taskqueue_free(gpr.tq);
		mtx_lock(&gpr.mtx);
		ch = gpr.ch;
		gpr.ch = NULL;
		mtx_unlock(&gpr.mtx);
		if (ch != NULL)
			qcom_glink_close(ch);
		cv_destroy(&gpr.cv);
		mtx_destroy(&gpr.mtx);
		return (0);
	default:
		return (EOPNOTSUPP);
	}
}

static moduledata_t qcom_gpr_mod = { "qcom_gpr", qcom_gpr_modevent, NULL };
DECLARE_MODULE(qcom_gpr, qcom_gpr_mod, SI_SUB_DRIVERS, SI_ORDER_ANY);
MODULE_DEPEND(qcom_gpr, qcom_glink, 1, 1, 1);
MODULE_VERSION(qcom_gpr, 1);
