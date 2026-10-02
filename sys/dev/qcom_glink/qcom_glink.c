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
 * Qualcomm GLINK over shared memory: named channels to a remote processor,
 * as Linux's qcom_glink_smem and qcom_glink_native drivers speak it.
 *
 * An edge is a pair of rings in SMEM, one each way, with a descriptor of
 * their tail and head offsets, and an IPCC doorbell each way.  Commands are
 * an 8-byte header (command, a 16-bit and a 32-bit parameter) and whatever
 * follows, padded to 8 bytes.  Both sides announce a version, then open
 * channels by name: each side's open is acknowledged by the other, and the
 * pair of channel ids identifies the channel.  Data goes into buffers the
 * receiver has advertised ("intents"), which it hands back when done.
 *
 * ACPI has the GLINK device (QCOM0684) but nothing about its edges, which
 * come from a table of SoCs that the DSDT's \_SB.SOID identifies.  An edge
 * starts once the remote has announced its version, which it does as soon
 * as it runs: waiting for that serves whether the remote boots before or
 * after this driver attaches.
 */

#include "opt_acpi.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/condvar.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/queue.h>
#include <sys/sbuf.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>

#include <machine/atomic.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include <dev/qcom_glink/qcom_glink.h>
#include <dev/qcom_glink/qcom_ipcc.h>
#include <dev/qcom_glink/qcom_smem.h>

static MALLOC_DEFINE(M_GLINK, "qcom_glink", "Qualcomm GLINK");

#define	GLINK_CMD_VERSION		0
#define	GLINK_CMD_VERSION_ACK		1
#define	GLINK_CMD_OPEN			2
#define	GLINK_CMD_CLOSE			3
#define	GLINK_CMD_OPEN_ACK		4
#define	GLINK_CMD_INTENT		5
#define	GLINK_CMD_RX_DONE		6
#define	GLINK_CMD_RX_INTENT_REQ		7
#define	GLINK_CMD_RX_INTENT_REQ_ACK	8
#define	GLINK_CMD_TX_DATA		9
#define	GLINK_CMD_CLOSE_ACK		11
#define	GLINK_CMD_TX_DATA_CONT		12
#define	GLINK_CMD_READ_NOTIF		13
#define	GLINK_CMD_RX_DONE_W_REUSE	14
#define	GLINK_CMD_SIGNALS		15

#define	GLINK_VERSION			1
#define	GLINK_FEATURE_INTENT_REUSE	0x1
#define	GLINK_NAME_SIZE			32

/* The SMEM items of an edge, in the partition shared with the remote. */
#define	GLINK_ITEM_DESC			478	/* tx tail, head, rx tail, head */
#define	GLINK_ITEM_TX_FIFO		479
#define	GLINK_ITEM_RX_FIFO		480
#define	GLINK_FIFO_SIZE			16384

#define	GLINK_TX_RESERVE		16	/* never full; a READ_NOTIF fits */
#define	GLINK_CHUNK			8192
#define	GLINK_WAIT			(10 * hz)
#define	GLINK_START_TRIES		300	/* a second apart */

struct glink_hdr {
	uint16_t	cmd;
	uint16_t	param1;
	uint32_t	param2;
} __packed;

struct glink_data_hdr {
	struct glink_hdr hdr;		/* param1 cid, param2 intent id */
	uint32_t	chunk;
	uint32_t	left;
} __packed;

struct qcom_glink_edge_conf {
	const char	*label;
	u_int		host;		/* SMEM host */
	u_int		ipcc_client;
	u_int		ipcc_signal;
};

struct qcom_glink_soc {
	uint32_t	id;		/* \_SB.SOID */
	const struct qcom_glink_edge_conf *edges;
	u_int		nedges;
};

static const struct qcom_glink_edge_conf sc8280xp_edges[] = {
	{ "lpass", QCOM_SMEM_HOST_ADSP, QCOM_IPCC_CLIENT_LPASS,
	    QCOM_IPCC_SIGNAL_GLINK },
	{ "cdsp", QCOM_SMEM_HOST_CDSP, QCOM_IPCC_CLIENT_CDSP,
	    QCOM_IPCC_SIGNAL_GLINK },
};

static const struct qcom_glink_soc qcom_glink_socs[] = {
	{ 449, sc8280xp_edges, nitems(sc8280xp_edges) },	/* SC8280XP */
};

struct glink_intent {
	TAILQ_ENTRY(glink_intent) link;
	uint32_t	id;
	size_t		size;
	size_t		offset;		/* ours: received so far */
	bool		reuse;
	bool		in_use;		/* the remote's: we're sending into it */
	uint8_t		*data;		/* ours */
};
TAILQ_HEAD(glink_intents, glink_intent);

struct glink_edge;

struct qcom_glink_chan {
	TAILQ_ENTRY(qcom_glink_chan) link;
	struct glink_edge *edge;
	char		name[GLINK_NAME_SIZE];
	uint16_t	lcid;		/* ours; 0 until we open it */
	uint16_t	rcid;		/* the remote's; 0 until it opens it */
	bool		open_acked;	/* the remote acknowledged our open */
	bool		remote_acked;	/* we acknowledged the remote's */
	struct thread	*rx_td;		/* in the client's rx callback */
	qcom_glink_rx_t	*rx;		/* the client, if any */
	void		*arg;
	struct glink_intents lintents;	/* we receive into */
	struct glink_intents rintents;	/* the remote receives into */
	uint32_t	next_liid;
	int		intent_req;	/* for a bigger intent: */
#define	INTENT_REQ_NONE		0
#define	INTENT_REQ_ASKED	1
#define	INTENT_REQ_GRANTED	2
#define	INTENT_REQ_REFUSED	3
	struct glink_intent *rx_partial; /* a message arriving in chunks */
};

struct glink_edge {
	struct qcom_glink_softc	*sc;
	const struct qcom_glink_edge_conf *conf;
	volatile uint32_t	*desc;
	uint8_t			*tx_fifo;
	uint8_t			*rx_fifo;
	size_t			tx_len;
	size_t			rx_len;
	struct mtx		mtx;
	struct cv		cv;	/* anything changed */
	struct task		rx_task;
	struct timeout_task	start_task;
	int			start_tries;
	bool			started;	/* the remote is talking */
	bool			up;		/* versions agreed */
	bool			dead;		/* the ring made no sense */
	bool			read_notif_sent;
	uint32_t		features;
	uint16_t		next_lcid;
	TAILQ_HEAD(, qcom_glink_chan) chans;
};

struct qcom_glink_softc {
	device_t		dev;
	struct taskqueue	*tq;
	struct glink_edge	*edges;
	u_int			nedges;
};

static struct qcom_glink_softc *qcom_glink_sc;

static char *qcom_glink_ids[] = { "QCOM0684", NULL };

#define	TX_TAIL(e)	((e)->desc[0])
#define	TX_HEAD(e)	((e)->desc[1])
#define	RX_TAIL(e)	((e)->desc[2])
#define	RX_HEAD(e)	((e)->desc[3])

static const struct qcom_glink_soc *
qcom_glink_find_soc(void)
{
	UINT32 id;
	u_int i;

	if (ACPI_FAILURE(acpi_GetInteger(ACPI_ROOT_OBJECT, "\\_SB.SOID", &id)))
		return (NULL);
	for (i = 0; i < nitems(qcom_glink_socs); i++)
		if (qcom_glink_socs[i].id == id)
			return (&qcom_glink_socs[i]);
	return (NULL);
}

/* The rings */

static size_t
glink_rx_avail(struct glink_edge *e)
{
	uint32_t head, tail;

	head = RX_HEAD(e);
	tail = RX_TAIL(e);
	/*
	 * The data no earlier than the head that published it.  The remote
	 * is outside our inner shareable domain: full-system barriers.
	 */
	rmb();
	if (head >= e->rx_len || tail >= e->rx_len)
		return (0);
	return (head >= tail ? head - tail : e->rx_len - tail + head);
}

static void
glink_rx_peek(struct glink_edge *e, void *buf, size_t off, size_t len)
{
	size_t n, tail;

	tail = RX_TAIL(e) + off;
	if (tail >= e->rx_len)
		tail -= e->rx_len;
	n = MIN(len, e->rx_len - tail);
	memcpy(buf, e->rx_fifo + tail, n);
	if (n < len)
		memcpy((char *)buf + n, e->rx_fifo, len - n);
}

static void
glink_rx_advance(struct glink_edge *e, size_t len)
{
	size_t tail;

	tail = RX_TAIL(e) + roundup2(len, 8);
	if (tail >= e->rx_len)
		tail -= e->rx_len;
	/* Done reading the data before the remote may reuse it. */
	mb();
	RX_TAIL(e) = tail;
}

static size_t
glink_tx_avail(struct glink_edge *e)
{
	uint32_t head, tail;
	size_t avail;

	head = TX_HEAD(e);
	tail = TX_TAIL(e);
	if (head >= e->tx_len || tail >= e->tx_len)
		return (0);
	avail = tail <= head ? e->tx_len - head + tail : tail - head;
	return (avail < GLINK_TX_RESERVE ? 0 : avail - GLINK_TX_RESERVE);
}

static size_t
glink_tx_write_one(struct glink_edge *e, size_t head, const void *data,
    size_t len)
{
	size_t n;

	n = MIN(len, e->tx_len - head);
	memcpy(e->tx_fifo + head, data, n);
	if (n < len)
		memcpy(e->tx_fifo, (const char *)data + n, len - n);
	head += len;
	if (head >= e->tx_len)
		head -= e->tx_len;
	return (head);
}

static void
glink_tx_write(struct glink_edge *e, const void *hdr, size_t hlen,
    const void *data, size_t dlen)
{
	size_t head;

	head = glink_tx_write_one(e, TX_HEAD(e), hdr, hlen);
	if (dlen != 0)
		head = glink_tx_write_one(e, head, data, dlen);
	head = roundup2(head, 8);
	if (head >= e->tx_len)
		head -= e->tx_len;
	/* The data before the head that publishes it, before the doorbell. */
	wmb();
	TX_HEAD(e) = head;
	wmb();
	qcom_ipcc_send(e->conf->ipcc_client, e->conf->ipcc_signal);
}

/*
 * Send a command and what follows it, waiting for ring space.  The
 * remote is asked once to tell us when it reads; its doorbell runs the
 * receive task, which wakes us, and a timeout covers a lost wakeup.
 */
static int
glink_tx(struct glink_edge *e, const void *hdr, size_t hlen, const void *data,
    size_t dlen)
{
	struct glink_hdr notif = { .cmd = GLINK_CMD_READ_NOTIF };
	size_t need;
	int waited;

	mtx_assert(&e->mtx, MA_OWNED);
	need = roundup2(hlen + dlen, 8);
	if (need >= e->tx_len - GLINK_TX_RESERVE)
		return (EINVAL);
	for (waited = 0; glink_tx_avail(e) < need; waited += hz / 100) {
		if (e->dead || waited >= GLINK_WAIT)
			return (EIO);
		if (!e->read_notif_sent) {
			/* It fits in the reserve. */
			e->read_notif_sent = true;
			glink_tx_write(e, &notif, sizeof(notif), NULL, 0);
		}
		cv_timedwait(&e->cv, &e->mtx, MAX(hz / 100, 1));
	}
	e->read_notif_sent = false;
	glink_tx_write(e, hdr, hlen, data, dlen);
	return (0);
}

static int
glink_send_cmd(struct glink_edge *e, uint16_t cmd, uint16_t param1,
    uint32_t param2)
{
	struct glink_hdr h = { cmd, param1, param2 };

	return (glink_tx(e, &h, sizeof(h), NULL, 0));
}

/* Channels and intents */

static struct qcom_glink_chan *
glink_chan_by_name(struct glink_edge *e, const char *name)
{
	struct qcom_glink_chan *ch;

	TAILQ_FOREACH(ch, &e->chans, link)
		if (strcmp(ch->name, name) == 0)
			return (ch);
	return (NULL);
}

static struct qcom_glink_chan *
glink_chan_by_lcid(struct glink_edge *e, u_int lcid)
{
	struct qcom_glink_chan *ch;

	TAILQ_FOREACH(ch, &e->chans, link)
		if (lcid != 0 && ch->lcid == lcid)
			return (ch);
	return (NULL);
}

static struct qcom_glink_chan *
glink_chan_by_rcid(struct glink_edge *e, u_int rcid)
{
	struct qcom_glink_chan *ch;

	TAILQ_FOREACH(ch, &e->chans, link)
		if (rcid != 0 && ch->rcid == rcid)
			return (ch);
	return (NULL);
}

static struct qcom_glink_chan *
glink_chan_alloc(struct glink_edge *e, const char *name)
{
	struct qcom_glink_chan *ch;

	ch = malloc(sizeof(*ch), M_GLINK, M_NOWAIT | M_ZERO);
	if (ch == NULL)
		return (NULL);
	ch->edge = e;
	strlcpy(ch->name, name, sizeof(ch->name));
	TAILQ_INIT(&ch->lintents);
	TAILQ_INIT(&ch->rintents);
	ch->next_liid = 1;
	TAILQ_INSERT_TAIL(&e->chans, ch, link);
	return (ch);
}

static void
glink_intents_free(struct glink_intents *list)
{
	struct glink_intent *in;

	while ((in = TAILQ_FIRST(list)) != NULL) {
		TAILQ_REMOVE(list, in, link);
		free(in->data, M_GLINK);
		free(in, M_GLINK);
	}
}

/* Forget a channel neither side has open any more. */
static void
glink_chan_gc(struct glink_edge *e, struct qcom_glink_chan *ch)
{

	if (ch->lcid != 0 || ch->rcid != 0 || ch->rx != NULL)
		return;
	TAILQ_REMOVE(&e->chans, ch, link);
	glink_intents_free(&ch->lintents);
	glink_intents_free(&ch->rintents);
	free(ch, M_GLINK);
}

/* Offer the remote a buffer of size bytes to send us a message in. */
static int
glink_intent_advertise(struct glink_edge *e, struct qcom_glink_chan *ch,
    size_t size, bool reuse)
{
	struct {
		struct glink_hdr hdr;	/* param1 lcid, param2 count */
		uint32_t	size;
		uint32_t	liid;
	} __packed cmd;
	struct glink_intent *in;

	in = malloc(sizeof(*in), M_GLINK, M_NOWAIT | M_ZERO);
	if (in == NULL)
		return (ENOMEM);
	in->data = malloc(MAX(size, 1), M_GLINK, M_NOWAIT);
	if (in->data == NULL) {
		free(in, M_GLINK);
		return (ENOMEM);
	}
	in->id = ch->next_liid++;
	in->size = size;
	in->reuse = reuse;
	TAILQ_INSERT_TAIL(&ch->lintents, in, link);
	cmd.hdr.cmd = GLINK_CMD_INTENT;
	cmd.hdr.param1 = ch->lcid;
	cmd.hdr.param2 = 1;
	cmd.size = size;
	cmd.liid = in->id;
	return (glink_tx(e, &cmd, sizeof(cmd), NULL, 0));
}

/* Received commands, with e->mtx held */

static void
glink_rx_version(struct glink_edge *e, u_int version, uint32_t features)
{

	if (version == 0)
		return;
	e->features &= features;
	(void)glink_send_cmd(e, GLINK_CMD_VERSION_ACK, GLINK_VERSION,
	    e->features);
}

static void
glink_rx_version_ack(struct glink_edge *e, u_int version, uint32_t features)
{

	if (version == 0) {
		device_printf(e->sc->dev, "%s: version refused\n",
		    e->conf->label);
		return;
	}
	if (features != e->features) {
		e->features &= features;
		(void)glink_send_cmd(e, GLINK_CMD_VERSION, GLINK_VERSION,
		    e->features);
		return;
	}
	if (!e->up)
		device_printf(e->sc->dev, "%s: up, GLINK version %u, "
		    "features %#x\n", e->conf->label, version, e->features);
	e->up = true;
}

static void
glink_rx_open(struct glink_edge *e, u_int rcid, const char *name)
{
	struct qcom_glink_chan *ch;

	ch = glink_chan_by_name(e, name);
	if (ch == NULL)
		ch = glink_chan_alloc(e, name);
	if (ch == NULL || glink_chan_by_rcid(e, rcid) != NULL) {
		device_printf(e->sc->dev, "%s: can't take channel %s (%u)\n",
		    e->conf->label, name, rcid);
		return;
	}
	ch->rcid = rcid;
	ch->remote_acked = false;
	if (bootverbose)
		device_printf(e->sc->dev, "%s: the remote opened %s\n",
		    e->conf->label, name);
	/*
	 * Open on our side already (the remote reopening it): acknowledge
	 * now.  While our own open is in progress, qcom_glink_open does.
	 */
	if (ch->rx != NULL && ch->open_acked &&
	    glink_send_cmd(e, GLINK_CMD_OPEN_ACK, rcid, 0) == 0)
		ch->remote_acked = true;
}

static void
glink_rx_close(struct glink_edge *e, u_int rcid)
{
	struct qcom_glink_chan *ch;

	ch = glink_chan_by_rcid(e, rcid);
	if (ch == NULL)
		return;
	(void)glink_send_cmd(e, GLINK_CMD_CLOSE_ACK, rcid, 0);
	ch->rcid = 0;
	ch->remote_acked = false;
	glink_intents_free(&ch->rintents);
	glink_chan_gc(e, ch);
}

static void
glink_rx_close_ack(struct glink_edge *e, u_int lcid)
{
	struct qcom_glink_chan *ch;

	ch = glink_chan_by_lcid(e, lcid);
	if (ch == NULL)
		return;
	ch->lcid = 0;
	ch->open_acked = false;
	glink_chan_gc(e, ch);
}

static void
glink_rx_intents(struct glink_edge *e, u_int rcid, const uint32_t *pairs,
    u_int count)
{
	struct qcom_glink_chan *ch;
	struct glink_intent *in;
	u_int i;

	ch = glink_chan_by_rcid(e, rcid);
	if (ch == NULL)
		return;
	for (i = 0; i < count; i++) {
		in = malloc(sizeof(*in), M_GLINK, M_NOWAIT | M_ZERO);
		if (in == NULL)
			break;
		in->size = pairs[2 * i];
		in->id = pairs[2 * i + 1];
		TAILQ_INSERT_TAIL(&ch->rintents, in, link);
	}
}

static void
glink_rx_done(struct glink_edge *e, u_int rcid, uint32_t iid, bool reuse)
{
	struct qcom_glink_chan *ch;
	struct glink_intent *in;

	ch = glink_chan_by_rcid(e, rcid);
	if (ch == NULL)
		return;
	TAILQ_FOREACH(in, &ch->rintents, link)
		if (in->id == iid)
			break;
	if (in == NULL)
		return;
	if (reuse)
		in->in_use = false;
	else {
		TAILQ_REMOVE(&ch->rintents, in, link);
		free(in, M_GLINK);
	}
}

static void
glink_rx_intent_req(struct glink_edge *e, u_int rcid, uint32_t size)
{
	struct qcom_glink_chan *ch;
	bool granted;

	ch = glink_chan_by_rcid(e, rcid);
	if (ch == NULL || ch->lcid == 0)
		return;
	granted = size <= 1024 * 1024 &&
	    glink_intent_advertise(e, ch, size, false) == 0;
	(void)glink_send_cmd(e, GLINK_CMD_RX_INTENT_REQ_ACK, ch->lcid,
	    granted);
}

/*
 * A chunk of a message into one of our intents.  Returns false if the
 * chunk isn't all in the ring yet.
 */
static bool
glink_rx_data(struct glink_edge *e, size_t avail)
{
	struct glink_data_hdr h;
	struct qcom_glink_chan *ch;
	struct glink_intent *in;
	qcom_glink_rx_t *rx;
	void *arg;
	bool reuse;

	if (avail < sizeof(h))
		return (false);
	glink_rx_peek(e, &h, 0, sizeof(h));
	if (avail < sizeof(h) + h.chunk)
		return (false);
	ch = glink_chan_by_rcid(e, h.hdr.param1);
	in = NULL;
	if (ch != NULL) {
		in = ch->rx_partial;
		if (in == NULL)
			TAILQ_FOREACH(in, &ch->lintents, link)
				if (in->id == h.hdr.param2)
					break;
	}
	if (in == NULL || in->size - in->offset < h.chunk) {
		device_printf(e->sc->dev, "%s: dropped %u bytes on %s\n",
		    e->conf->label, h.chunk, ch != NULL ? ch->name : "?");
		glink_rx_advance(e, sizeof(h) + h.chunk);
		return (true);
	}
	glink_rx_peek(e, in->data + in->offset, sizeof(h), h.chunk);
	in->offset += h.chunk;
	glink_rx_advance(e, sizeof(h) + h.chunk);
	if (h.left != 0) {
		ch->rx_partial = in;
		return (true);
	}
	ch->rx_partial = NULL;

	/*
	 * The client may send, so it's called without the lock.  The intent
	 * is off the channel's list meanwhile, out of a close's way, and the
	 * channel is marked busy, which a close waits for.
	 */
	TAILQ_REMOVE(&ch->lintents, in, link);
	rx = ch->rx;
	arg = ch->arg;
	if (rx != NULL) {
		ch->rx_td = curthread;
		mtx_unlock(&e->mtx);
		rx(arg, in->data, in->offset);
		mtx_lock(&e->mtx);
		ch->rx_td = NULL;
	}
	in->offset = 0;
	reuse = in->reuse && ch->lcid != 0;
	if (ch->lcid != 0)
		(void)glink_send_cmd(e, reuse ? GLINK_CMD_RX_DONE_W_REUSE :
		    GLINK_CMD_RX_DONE, ch->lcid, in->id);
	if (reuse)
		TAILQ_INSERT_TAIL(&ch->lintents, in, link);
	else {
		free(in->data, M_GLINK);
		free(in, M_GLINK);
	}
	cv_broadcast(&e->cv);
	return (true);
}

static void
glink_rx_task(void *arg, int pending __unused)
{
	struct glink_edge *e = arg;
	struct glink_hdr h;
	char name[GLINK_NAME_SIZE];
	uint32_t *pairs;
	size_t avail, len;

	mtx_lock(&e->mtx);
	while (e->started && !e->dead) {
		avail = glink_rx_avail(e);
		if (avail < sizeof(h))
			break;
		glink_rx_peek(e, &h, 0, sizeof(h));
		switch (h.cmd) {
		case GLINK_CMD_VERSION:
			glink_rx_advance(e, sizeof(h));
			glink_rx_version(e, h.param1, h.param2);
			continue;
		case GLINK_CMD_VERSION_ACK:
			glink_rx_advance(e, sizeof(h));
			glink_rx_version_ack(e, h.param1, h.param2);
			continue;
		case GLINK_CMD_OPEN:
			/* The upper half of param2 is a priority. */
			len = h.param2 & 0xffff;
			if (avail < sizeof(h) + len)
				break;
			memset(name, 0, sizeof(name));
			glink_rx_peek(e, name, sizeof(h),
			    MIN(len, sizeof(name) - 1));
			glink_rx_advance(e, sizeof(h) + len);
			glink_rx_open(e, h.param1, name);
			continue;
		case GLINK_CMD_CLOSE:
			glink_rx_advance(e, sizeof(h));
			glink_rx_close(e, h.param1);
			continue;
		case GLINK_CMD_CLOSE_ACK:
			glink_rx_advance(e, sizeof(h));
			glink_rx_close_ack(e, h.param1);
			continue;
		case GLINK_CMD_OPEN_ACK: {
			struct qcom_glink_chan *ch;

			glink_rx_advance(e, sizeof(h));
			ch = glink_chan_by_lcid(e, h.param1);
			if (ch != NULL)
				ch->open_acked = true;
			continue;
		}
		case GLINK_CMD_INTENT:
			len = sizeof(h) + h.param2 * 2 * sizeof(uint32_t);
			if (avail < len)
				break;
			pairs = malloc(MAX(len - sizeof(h), 1), M_GLINK,
			    M_NOWAIT);
			if (pairs == NULL)
				break;
			glink_rx_peek(e, pairs, sizeof(h), len - sizeof(h));
			glink_rx_advance(e, len);
			glink_rx_intents(e, h.param1, pairs, h.param2);
			free(pairs, M_GLINK);
			continue;
		case GLINK_CMD_RX_DONE:
		case GLINK_CMD_RX_DONE_W_REUSE:
			glink_rx_advance(e, sizeof(h));
			glink_rx_done(e, h.param1, h.param2,
			    h.cmd == GLINK_CMD_RX_DONE_W_REUSE);
			continue;
		case GLINK_CMD_RX_INTENT_REQ:
			glink_rx_advance(e, sizeof(h));
			glink_rx_intent_req(e, h.param1, h.param2);
			continue;
		case GLINK_CMD_RX_INTENT_REQ_ACK: {
			struct qcom_glink_chan *ch;

			glink_rx_advance(e, sizeof(h));
			ch = glink_chan_by_rcid(e, h.param1);
			if (ch != NULL)
				ch->intent_req = h.param2 != 0 ?
				    INTENT_REQ_GRANTED : INTENT_REQ_REFUSED;
			continue;
		}
		case GLINK_CMD_TX_DATA:
		case GLINK_CMD_TX_DATA_CONT:
			if (!glink_rx_data(e, avail))
				break;
			continue;
		case GLINK_CMD_READ_NOTIF:
			glink_rx_advance(e, sizeof(h));
			qcom_ipcc_send(e->conf->ipcc_client,
			    e->conf->ipcc_signal);
			continue;
		case GLINK_CMD_SIGNALS:
			glink_rx_advance(e, sizeof(h));
			continue;
		default:
			/* There's no telling where the next command is. */
			device_printf(e->sc->dev, "%s: unknown command %u; "
			    "the edge is dead\n", e->conf->label, h.cmd);
			e->dead = true;
			continue;
		}
		break;	/* the rest of a command is yet to come */
	}
	cv_broadcast(&e->cv);
	mtx_unlock(&e->mtx);
}

static void
glink_edge_intr(void *arg)
{
	struct glink_edge *e = arg;

	taskqueue_enqueue(e->sc->tq, &e->rx_task);
}

/* Starting an edge */

static int
glink_edge_item(struct glink_edge *e, u_int item, size_t size, void **p,
    size_t *len)
{
	int error;

	if (size != 0) {
		error = qcom_smem_alloc(e->conf->host, item, size);
		if (error != 0 && error != EEXIST)
			return (error);
	}
	return (qcom_smem_get(e->conf->host, item, p, len));
}

static void
glink_edge_start(void *arg, int pending __unused)
{
	struct glink_edge *e = arg;
	void *p;
	size_t len;
	int error;

	/*
	 * Only this task touches the ring pointers before the edge has
	 * started, so the SMEM lookups, which may sleep, go unlocked.
	 */
	if (e->desc == NULL) {
		if (glink_edge_item(e, GLINK_ITEM_DESC, 32, &p, &len) != 0 ||
		    len < 4 * sizeof(uint32_t))
			goto retry;
		e->desc = p;
	}
	if (e->tx_fifo == NULL) {
		if (glink_edge_item(e, GLINK_ITEM_TX_FIFO, GLINK_FIFO_SIZE, &p,
		    &len) != 0)
			goto retry;
		e->tx_fifo = p;
		e->tx_len = len;
	}
	/* The remote's ring is the remote's to allocate. */
	if (e->rx_fifo == NULL) {
		if (glink_edge_item(e, GLINK_ITEM_RX_FIFO, 0, &p, &len) != 0)
			goto retry;
		e->rx_fifo = p;
		e->rx_len = len;
	}
	/* Until the remote announces its version, it isn't listening. */
	if (glink_rx_avail(e) == 0)
		goto retry;
	error = qcom_ipcc_register(e->conf->ipcc_client, e->conf->ipcc_signal,
	    glink_edge_intr, e);
	if (error == ENXIO)
		goto retry;		/* no IPCC yet */
	if (error != 0) {
		device_printf(e->sc->dev, "%s: no doorbell: %d\n",
		    e->conf->label, error);
		return;
	}
	mtx_lock(&e->mtx);
	e->started = true;
	e->features = GLINK_FEATURE_INTENT_REUSE;
	(void)glink_send_cmd(e, GLINK_CMD_VERSION, GLINK_VERSION, e->features);
	mtx_unlock(&e->mtx);
	taskqueue_enqueue(e->sc->tq, &e->rx_task);
	return;
retry:
	if (++e->start_tries < GLINK_START_TRIES)
		taskqueue_enqueue_timeout(e->sc->tq, &e->start_task, hz);
	else
		device_printf(e->sc->dev, "%s: the remote never spoke\n",
		    e->conf->label);
}

/* The client interface */

static int
glink_wait(struct glink_edge *e, bool *cond)
{
	int waited;

	for (waited = 0; !*cond; waited += hz / 10) {
		if (e->dead || waited >= GLINK_WAIT)
			return (ETIMEDOUT);
		cv_timedwait(&e->cv, &e->mtx, MAX(hz / 10, 1));
	}
	return (0);
}

static struct glink_edge *
glink_edge_by_label(const char *label)
{
	struct qcom_glink_softc *sc = qcom_glink_sc;
	u_int i;

	if (sc == NULL)
		return (NULL);
	for (i = 0; i < sc->nedges; i++)
		if (strcmp(sc->edges[i].conf->label, label) == 0)
			return (&sc->edges[i]);
	return (NULL);
}

int
qcom_glink_open(const char *label, const char *name, size_t intent_size,
    u_int nintents, qcom_glink_rx_t *rx, void *arg,
    struct qcom_glink_chan **chp)
{
	struct {
		struct glink_hdr hdr;	/* param1 lcid, param2 name length */
		char		name[GLINK_NAME_SIZE];
	} __packed open;
	struct glink_edge *e;
	struct qcom_glink_chan *ch;
	size_t len;
	int error;
	u_int i;

	len = strlen(name) + 1;
	if (len > GLINK_NAME_SIZE || rx == NULL)
		return (EINVAL);
	e = glink_edge_by_label(label);
	if (e == NULL)
		return (ENXIO);
	mtx_lock(&e->mtx);
	error = glink_wait(e, &e->up);
	if (error != 0)
		goto out;
	ch = glink_chan_by_name(e, name);
	if (ch == NULL)
		ch = glink_chan_alloc(e, name);
	if (ch == NULL) {
		error = ENOMEM;
		goto out;
	}
	if (ch->rx != NULL || ch->lcid != 0) {
		error = EBUSY;
		goto out;
	}
	ch->rx = rx;
	ch->arg = arg;
	do {
		ch->lcid = ++e->next_lcid;
	} while (ch->lcid == 0 || glink_chan_by_lcid(e, ch->lcid) != ch);

	/* The remote's open first, if it has; ours either way. */
	if (ch->rcid != 0 && !ch->remote_acked) {
		error = glink_send_cmd(e, GLINK_CMD_OPEN_ACK, ch->rcid, 0);
		ch->remote_acked = error == 0;
	}
	if (error == 0) {
		open.hdr.cmd = GLINK_CMD_OPEN;
		open.hdr.param1 = ch->lcid;
		open.hdr.param2 = len;
		memset(open.name, 0, sizeof(open.name));
		strlcpy(open.name, name, sizeof(open.name));
		error = glink_tx(e, &open, sizeof(open.hdr) + len, NULL, 0);
	}
	if (error == 0)
		error = glink_wait(e, &ch->open_acked);
	if (error == 0 && !ch->remote_acked) {
		/*
		 * An open of our own: the remote's comes now, perhaps already
		 * with its acknowledgement of ours, and wants acknowledging.
		 */
		for (i = 0; ch->rcid == 0 && i < 100 && !e->dead; i++)
			cv_timedwait(&e->cv, &e->mtx, MAX(hz / 10, 1));
		if (ch->rcid == 0)
			error = ETIMEDOUT;
		else if (!ch->remote_acked) {
			error = glink_send_cmd(e, GLINK_CMD_OPEN_ACK, ch->rcid,
			    0);
			ch->remote_acked = error == 0;
		}
	}
	for (i = 0; error == 0 && i < nintents; i++)
		error = glink_intent_advertise(e, ch, intent_size, true);
	if (error != 0) {
		ch->rx = NULL;
		if (ch->open_acked)
			(void)glink_send_cmd(e, GLINK_CMD_CLOSE, ch->lcid, 0);
		else
			ch->lcid = 0;
		glink_chan_gc(e, ch);
		goto out;
	}
	*chp = ch;
out:
	mtx_unlock(&e->mtx);
	return (error);
}

int
qcom_glink_send(struct qcom_glink_chan *ch, const void *data, size_t len)
{
	struct glink_edge *e = ch->edge;
	struct glink_data_hdr h;
	struct glink_intent *in, *best;
	struct glink_hdr req;
	size_t chunk, off;
	uint32_t iid;
	int error, waited;

	mtx_lock(&e->mtx);
	for (waited = 0;; waited += hz / 100) {
		if (ch->rcid == 0 || e->dead) {
			error = ENOTCONN;
			goto out;
		}
		best = NULL;
		TAILQ_FOREACH(in, &ch->rintents, link)
			if (!in->in_use && in->size >= len &&
			    (best == NULL || in->size < best->size))
				best = in;
		if (best != NULL)
			break;
		if (waited >= GLINK_WAIT) {
			error = ETIMEDOUT;
			goto out;
		}
		/* Ask for one big enough, once, then wait for it. */
		if (ch->intent_req == INTENT_REQ_NONE) {
			ch->intent_req = INTENT_REQ_ASKED;
			req.cmd = GLINK_CMD_RX_INTENT_REQ;
			req.param1 = ch->lcid;
			req.param2 = len;
			error = glink_tx(e, &req, sizeof(req), NULL, 0);
			if (error != 0)
				goto out;
		} else if (ch->intent_req == INTENT_REQ_REFUSED) {
			ch->intent_req = INTENT_REQ_NONE;
			error = ENOBUFS;
			goto out;
		}
		cv_timedwait(&e->cv, &e->mtx, MAX(hz / 100, 1));
	}
	ch->intent_req = INTENT_REQ_NONE;
	best->in_use = true;
	/*
	 * By its ID from here: sending may sleep, and a close from the remote
	 * meanwhile frees its intents.
	 */
	iid = best->id;
	for (off = 0, error = 0; error == 0 && (off < len || len == 0);
	    off += chunk) {
		if (ch->rcid == 0 || e->dead) {
			error = ENOTCONN;
			break;
		}
		chunk = MIN(len - off, GLINK_CHUNK);
		h.hdr.cmd = off == 0 ? GLINK_CMD_TX_DATA :
		    GLINK_CMD_TX_DATA_CONT;
		h.hdr.param1 = ch->lcid;
		h.hdr.param2 = iid;
		h.chunk = chunk;
		h.left = len - off - chunk;
		error = glink_tx(e, &h, sizeof(h), (const char *)data + off,
		    chunk);
		if (len == 0)
			break;
	}
	if (error != 0)
		TAILQ_FOREACH(in, &ch->rintents, link)
			if (in->id == iid) {
				in->in_use = false;
				break;
			}
out:
	mtx_unlock(&e->mtx);
	return (error);
}

bool
qcom_glink_up(const char *label)
{
	struct glink_edge *e;
	bool up;

	e = glink_edge_by_label(label);
	if (e == NULL)
		return (false);
	mtx_lock(&e->mtx);
	up = e->up && !e->dead;
	mtx_unlock(&e->mtx);
	return (up);
}

bool
qcom_glink_announced(const char *label, const char *name)
{
	struct qcom_glink_chan *ch;
	struct glink_edge *e;
	bool announced;

	e = glink_edge_by_label(label);
	if (e == NULL)
		return (false);
	mtx_lock(&e->mtx);
	ch = glink_chan_by_name(e, name);
	announced = e->up && !e->dead && ch != NULL && ch->rcid != 0;
	mtx_unlock(&e->mtx);
	return (announced);
}

void
qcom_glink_close(struct qcom_glink_chan *ch)
{
	struct glink_edge *e = ch->edge;
	int i;

	mtx_lock(&e->mtx);
	ch->rx = NULL;
	/* Not while the rx task is in the client's callback, unless it's us. */
	while (ch->rx_td != NULL && ch->rx_td != curthread)
		cv_wait(&e->cv, &e->mtx);
	ch->rx_partial = NULL;
	if (ch->lcid != 0 && !e->dead &&
	    glink_send_cmd(e, GLINK_CMD_CLOSE, ch->lcid, 0) == 0)
		for (i = 0; ch->lcid != 0 && i < 50; i++)
			cv_timedwait(&e->cv, &e->mtx, MAX(hz / 10, 1));
	ch->lcid = 0;
	ch->open_acked = false;
	glink_intents_free(&ch->lintents);
	glink_intents_free(&ch->rintents);
	glink_chan_gc(e, ch);
	mtx_unlock(&e->mtx);
}

/* The device */

static int
glink_edge_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct glink_edge *e = arg1;
	struct qcom_glink_chan *ch;
	struct sbuf *sb;
	int error;

	/* Written under the lock, so into memory, copied out after. */
	sb = sbuf_new(NULL, NULL, 4096, SBUF_FIXEDLEN);
	mtx_lock(&e->mtx);
	sbuf_printf(sb, "%s", e->dead ? "dead" : e->up ? "up" :
	    e->started ? "negotiating" : "waiting for the remote");
	TAILQ_FOREACH(ch, &e->chans, link)
		sbuf_printf(sb, "\n%-24s local %u remote %u%s", ch->name,
		    ch->lcid, ch->rcid, ch->rx != NULL ? ", in use" : "");
	mtx_unlock(&e->mtx);
	sbuf_finish(sb);
	error = SYSCTL_OUT(req, sbuf_data(sb), sbuf_len(sb) + 1);
	sbuf_delete(sb);
	return (error);
}

static int
qcom_glink_probe(device_t dev)
{
	int rv;

	rv = ACPI_ID_PROBE(device_get_parent(dev), dev, qcom_glink_ids, NULL);
	if (rv > 0)
		return (rv);
	if (qcom_glink_find_soc() == NULL)
		return (ENXIO);
	device_set_desc(dev, "Qualcomm GLINK");
	return (rv);
}

static int
qcom_glink_attach(device_t dev)
{
	struct qcom_glink_softc *sc = device_get_softc(dev);
	const struct qcom_glink_soc *soc = qcom_glink_find_soc();
	struct glink_edge *e;
	u_int i;

	if (qcom_glink_sc != NULL)
		return (EEXIST);
	sc->dev = dev;
	sc->tq = taskqueue_create("qcom_glink", M_WAITOK,
	    taskqueue_thread_enqueue, &sc->tq);
	taskqueue_start_threads(&sc->tq, 1, PI_NET, "%s",
	    device_get_nameunit(dev));
	sc->nedges = soc->nedges;
	sc->edges = mallocarray(sc->nedges, sizeof(*sc->edges), M_GLINK,
	    M_WAITOK | M_ZERO);
	for (i = 0; i < sc->nedges; i++) {
		e = &sc->edges[i];
		e->sc = sc;
		e->conf = &soc->edges[i];
		mtx_init(&e->mtx, e->conf->label, "qcom_glink edge", MTX_DEF);
		cv_init(&e->cv, e->conf->label);
		TAILQ_INIT(&e->chans);
		TASK_INIT(&e->rx_task, 0, glink_rx_task, e);
		TIMEOUT_TASK_INIT(sc->tq, &e->start_task, 0, glink_edge_start,
		    e);
		SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
		    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO,
		    e->conf->label, CTLTYPE_STRING | CTLFLAG_RD |
		    CTLFLAG_MPSAFE, e, 0, glink_edge_sysctl, "A",
		    "The edge and its channels");
		taskqueue_enqueue_timeout(sc->tq, &e->start_task, 0);
	}
	qcom_glink_sc = sc;
	return (0);
}

static int
qcom_glink_detach(device_t dev)
{
	struct qcom_glink_softc *sc = device_get_softc(dev);
	struct qcom_glink_chan *ch;
	struct glink_edge *e;
	u_int i;

	for (i = 0; i < sc->nedges; i++) {
		e = &sc->edges[i];
		mtx_lock(&e->mtx);
		TAILQ_FOREACH(ch, &e->chans, link)
			if (ch->rx != NULL) {
				mtx_unlock(&e->mtx);
				return (EBUSY);
			}
		mtx_unlock(&e->mtx);
	}
	qcom_glink_sc = NULL;
	for (i = 0; i < sc->nedges; i++) {
		e = &sc->edges[i];
		while (taskqueue_cancel_timeout(sc->tq, &e->start_task,
		    NULL) != 0)
			taskqueue_drain_timeout(sc->tq, &e->start_task);
		if (e->started)
			qcom_ipcc_unregister(e->conf->ipcc_client,
			    e->conf->ipcc_signal);
		taskqueue_drain(sc->tq, &e->rx_task);
	}
	taskqueue_free(sc->tq);
	for (i = 0; i < sc->nedges; i++) {
		e = &sc->edges[i];
		while ((ch = TAILQ_FIRST(&e->chans)) != NULL) {
			TAILQ_REMOVE(&e->chans, ch, link);
			glink_intents_free(&ch->lintents);
			glink_intents_free(&ch->rintents);
			free(ch, M_GLINK);
		}
		cv_destroy(&e->cv);
		mtx_destroy(&e->mtx);
	}
	free(sc->edges, M_GLINK);
	return (0);
}

static device_method_t qcom_glink_methods[] = {
	DEVMETHOD(device_probe,		qcom_glink_probe),
	DEVMETHOD(device_attach,	qcom_glink_attach),
	DEVMETHOD(device_detach,	qcom_glink_detach),

	DEVMETHOD_END
};

static driver_t qcom_glink_driver = {
	"qcom_glink",
	qcom_glink_methods,
	sizeof(struct qcom_glink_softc),
};

DRIVER_MODULE(qcom_glink, acpi, qcom_glink_driver, 0, 0);
MODULE_DEPEND(qcom_glink, acpi, 1, 1, 1);
MODULE_DEPEND(qcom_glink, qcom_smem, 1, 1, 1);
MODULE_DEPEND(qcom_glink, qcom_ipcc, 1, 1, 1);
MODULE_VERSION(qcom_glink, 1);
