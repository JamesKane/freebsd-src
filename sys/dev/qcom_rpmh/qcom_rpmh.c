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
 * RPMh requests from the application processors, as Linux's rpmh-rsc and
 * rpmhpd drivers make them: through the apps RSC's DRV2, a command (an
 * RPMh resource address and its data) written into one of its "active"
 * TCSs, triggered in AMC mode, and waited for.  Only active-only requests
 * for now, which hold until changed: no sleep or wake sets, so a vote
 * stays in force when the CPUs sleep.
 *
 * On top, drivers' requests on ARC resources, the power rails, by level
 * (the command DB holds each rail's levels; the vote is the level's index),
 * and on BCMs, the interconnects' bandwidth nodes, aggregated as Linux's
 * rpmhpd and bcm-voter do (see below).
 *
 * The RSC isn't described by ACPI; where it is comes from a table of SoCs,
 * which the DSDT's \_SB.SOID identifies.
 */

#include "opt_acpi.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/queue.h>
#include <sys/sbuf.h>
#include <sys/sx.h>
#include <sys/sysctl.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/atomic.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include <dev/qcom_cmd_db/qcom_cmd_db.h>
#include <dev/qcom_rpmh/qcom_rpmh.h>

/* RSC v2.7 layout (Linux's rpmh_rsc_reg_offset_ver_2_7). */
#define	RSC_DRV_ID		0x00
#define	RSC_TCS_SIZE		672
#define	RSC_CMD_SIZE		20
#define	RSC_IRQ_ENABLE		0x00	/* from the TCS area, not per TCS */
#define	RSC_IRQ_STATUS		0x04
#define	RSC_IRQ_CLEAR		0x08
#define	RSC_TCS_CONTROL		0x14	/* per TCS */
#define	RSC_TCS_STATUS		0x18	/* nonzero: idle */
#define	RSC_TCS_CMD_ENABLE	0x1c
#define	RSC_CMD_MSGID		0x30	/* per command */
#define	RSC_CMD_ADDR		0x34
#define	RSC_CMD_DATA		0x38
#define	RSC_CMD_STATUS		0x3c

#define	TCS_AMC_MODE_ENABLE	(1u << 16)
#define	TCS_AMC_MODE_TRIGGER	(1u << 24)
#define	CMD_MSGID_LEN		8
#define	CMD_MSGID_RESP_REQ	(1u << 8)
#define	CMD_MSGID_WRITE		(1u << 16)

struct qcom_rpmh_soc {
	uint32_t	id;		/* \_SB.SOID */
	vm_paddr_t	drv;		/* the apps RSC's DRV2 */
	uint32_t	tcs_offset;
	u_int		active_first;	/* its active TCSs */
	u_int		nactive;
};

static const struct qcom_rpmh_soc qcom_rpmh_socs[] = {
	/* tcs-config: active 2, sleep 3, wake 3, control 1. */
	{ 449, 0x18220000, 0xd00, 0, 2 },	/* SC8280XP */
};

static struct {
	struct sx		lock;
	const struct qcom_rpmh_soc *soc;
	volatile uint32_t	*regs;	/* the TCS area */
} rpmh;

SX_SYSINIT(qcom_rpmh, &rpmh.lock, "qcom_rpmh");

#define	TCS_REG(t, r)		(rpmh.regs[((t) * RSC_TCS_SIZE + (r)) / 4])
#define	CMD_REG(t, c, r)						\
	(rpmh.regs[((t) * RSC_TCS_SIZE + (c) * RSC_CMD_SIZE + (r)) / 4])
#define	RSC_REG(r)		(rpmh.regs[(r) / 4])

static int
qcom_rpmh_init(void)
{
	UINT32 id;
	u_int i;

	sx_assert(&rpmh.lock, SA_XLOCKED);
	if (rpmh.regs != NULL)
		return (0);
	if (ACPI_FAILURE(acpi_GetInteger(ACPI_ROOT_OBJECT, "\\_SB.SOID", &id)))
		return (ENXIO);
	for (i = 0; i < nitems(qcom_rpmh_socs); i++)
		if (qcom_rpmh_socs[i].id == id)
			rpmh.soc = &qcom_rpmh_socs[i];
	if (rpmh.soc == NULL)
		return (ENXIO);
	rpmh.regs = pmap_mapdev(rpmh.soc->drv + rpmh.soc->tcs_offset,
	    RSC_TCS_SIZE * 16);
	return (0);
}

/* Leave the TCS as Linux's interrupt handler does: off, and free. */
static void
qcom_rpmh_tcs_reset(u_int t)
{

	TCS_REG(t, RSC_TCS_CONTROL) &= ~TCS_AMC_MODE_TRIGGER;
	TCS_REG(t, RSC_TCS_CONTROL) &= ~TCS_AMC_MODE_ENABLE;
	TCS_REG(t, RSC_TCS_CMD_ENABLE) = 0;
	RSC_REG(RSC_IRQ_CLEAR) = 1u << t;
	RSC_REG(RSC_IRQ_ENABLE) &= ~(1u << t);
}

/*
 * Send one active-only write, and wait for the resource to acknowledge it.
 * Sleeps.
 */
int
qcom_rpmh_write(uint32_t addr, uint32_t data)
{
	u_int i, t;
	int error;

	sx_xlock(&rpmh.lock);
	error = qcom_rpmh_init();
	if (error != 0) {
		sx_xunlock(&rpmh.lock);
		return (error);
	}
	/* A free active TCS: idle, and nothing of anyone's enabled in it. */
	for (t = rpmh.soc->active_first;
	    t < rpmh.soc->active_first + rpmh.soc->nactive; t++)
		if (TCS_REG(t, RSC_TCS_STATUS) != 0 &&
		    TCS_REG(t, RSC_TCS_CMD_ENABLE) == 0)
			break;
	if (t == rpmh.soc->active_first + rpmh.soc->nactive) {
		sx_xunlock(&rpmh.lock);
		return (EBUSY);
	}

	/* The completion latches in IRQ_STATUS only while enabled. */
	RSC_REG(RSC_IRQ_CLEAR) = 1u << t;
	RSC_REG(RSC_IRQ_ENABLE) |= 1u << t;
	CMD_REG(t, 0, RSC_CMD_MSGID) = CMD_MSGID_LEN | CMD_MSGID_WRITE |
	    CMD_MSGID_RESP_REQ;
	CMD_REG(t, 0, RSC_CMD_ADDR) = addr;
	CMD_REG(t, 0, RSC_CMD_DATA) = data;
	TCS_REG(t, RSC_TCS_CMD_ENABLE) = 1;
	wmb();
	TCS_REG(t, RSC_TCS_CONTROL) &= ~TCS_AMC_MODE_TRIGGER;
	TCS_REG(t, RSC_TCS_CONTROL) &= ~TCS_AMC_MODE_ENABLE;
	TCS_REG(t, RSC_TCS_CONTROL) = TCS_AMC_MODE_ENABLE;
	wmb();
	TCS_REG(t, RSC_TCS_CONTROL) = TCS_AMC_MODE_ENABLE |
	    TCS_AMC_MODE_TRIGGER;

	/* Rails take some time to settle: up to a second. */
	error = ETIMEDOUT;
	for (i = 0; i < 1000; i++) {
		if ((RSC_REG(RSC_IRQ_STATUS) & (1u << t)) != 0) {
			error = 0;
			break;
		}
		if (i < 100)
			DELAY(10);
		else
			pause("rpmh", MAX(hz / 1000, 1));
	}
	if (error != 0)
		printf("qcom_rpmh: %#x <- %#x: no answer (TCS %u status %#x, "
		    "command status %#x)\n", addr, data, t,
		    TCS_REG(t, RSC_TCS_STATUS), CMD_REG(t, 0, RSC_CMD_STATUS));
	qcom_rpmh_tcs_reset(t);
	sx_xunlock(&rpmh.lock);
	return (error);
}

/*
 * Votes, as Linux's rpmhpd and bcm-voter aggregate them: each driver holds
 * a request per resource, and what goes to RPMh is their aggregate: a
 * rail's highest level; a BCM's summed average and highest peak.  RPMh
 * keeps one vote per resource for the application processors, so a driver
 * writing its own would undo another's.
 *
 * What UEFI voted can't be read back, and some of it still matters: the
 * display, which UEFI set up, runs on it.  A "firmware" request on each such
 * resource stands in for it, at a level found to cover it (see the SoC
 * table).
 */
struct qcom_rpmh_req {
	TAILQ_ENTRY(qcom_rpmh_req) link;
	struct qcom_rpmh_res	*res;
	char			client[24];
	u_int			hlvl;		/* ARC: a level index; 0 none */
	uint32_t		avg, peak;	/* BCM, in its units */
};

struct qcom_rpmh_res {
	TAILQ_ENTRY(qcom_rpmh_res) link;
	TAILQ_HEAD(, qcom_rpmh_req) reqs;
	char			name[16];
	uint32_t		addr;
	bool			arc;		/* or a BCM */
	const uint16_t		*lv;		/* ARC: the levels (vlvl) */
	u_int			nlv;
	uint32_t		unit;		/* BCM: its units (cmd-db) */
	uint16_t		width;
	bool			sent_valid;
	uint32_t		sent;		/* ARC: index; BCM: the command */
};

/* What firmware's votes must keep covering: a rail's vlvl, a BCM's peak. */
struct qcom_rpmh_floor {
	uint32_t	soc_id;
	const char	*res;
	uint32_t	value;
	const char	*why;
};

static const struct qcom_rpmh_floor qcom_rpmh_floors[] = {
	/*
	 * SC8280XP: the display (DP2, MDP clock at 300 MHz from UEFI) needs
	 * MMCX at SVS, nominal for MDP to 500 MHz or HBR3; MX at SVS has run
	 * it.  Its path to memory is MM0, which RPMh keeps alive: Linux, with
	 * the same display, votes nothing on MM1 while the video codec idles.
	 */
	{ 449, "mmcx.lvl", 256, "display (UEFI)" },
	{ 449, "mx.lvl", 128, "display (UEFI)" },
};

static TAILQ_HEAD(, qcom_rpmh_res) qcom_rpmh_resources =
    TAILQ_HEAD_INITIALIZER(qcom_rpmh_resources);
static struct sx qcom_rpmh_votes_lock;
SX_SYSINIT(qcom_rpmh_votes, &qcom_rpmh_votes_lock, "qcom_rpmh votes");
static MALLOC_DEFINE(M_QCOM_RPMH, "qcom_rpmh", "Qualcomm RPMh votes");

/* The rail's lowest level index at or above vlvl; nlv if none is. */
static u_int
qcom_rpmh_arc_index(struct qcom_rpmh_res *res, u_int vlvl)
{
	u_int i;

	if (vlvl == 0)
		return (0);
	if (vlvl == QCOM_RPMH_ARC_MAX)
		return (res->nlv - 1);
	for (i = 0; i < res->nlv; i++)
		if (res->lv[i] >= vlvl)
			return (i);
	return (res->nlv);
}

static uint32_t
qcom_rpmh_bcm_cmd(uint32_t avg, uint32_t peak)
{
	/* Linux's BCM_TCS_CMD: commit, valid, x (average), y (peak). */
	return (1u << 30 | (avg != 0 || peak != 0 ? 1u << 29 : 0) |
	    avg << 14 | peak);
}

/* Send the aggregate of the resource's requests, if it changed. */
static int
qcom_rpmh_res_update(struct qcom_rpmh_res *res)
{
	struct qcom_rpmh_req *req;
	uint32_t data, avg, peak;
	int error;

	sx_assert(&qcom_rpmh_votes_lock, SA_XLOCKED);
	if (res->arc) {
		data = 0;
		TAILQ_FOREACH(req, &res->reqs, link)
			data = MAX(data, req->hlvl);
	} else {
		avg = peak = 0;
		TAILQ_FOREACH(req, &res->reqs, link) {
			avg = MIN(avg + req->avg, QCOM_RPMH_BCM_MAX);
			peak = MAX(peak, req->peak);
		}
		data = qcom_rpmh_bcm_cmd(avg, peak);
	}
	if (res->sent_valid && res->sent == data)
		return (0);
	if (bootverbose)
		printf("qcom_rpmh: %s (%#x) to %s %u\n", res->name, res->addr,
		    res->arc ? "level" : "command", data);
	error = qcom_rpmh_write(res->addr, data);
	if (error == 0) {
		res->sent = data;
		res->sent_valid = true;
	}
	return (error);
}

static struct qcom_rpmh_req *
qcom_rpmh_req_alloc(struct qcom_rpmh_res *res, const char *client)
{
	struct qcom_rpmh_req *req;

	req = malloc(sizeof(*req), M_QCOM_RPMH, M_WAITOK | M_ZERO);
	req->res = res;
	strlcpy(req->client, client, sizeof(req->client));
	TAILQ_INSERT_TAIL(&res->reqs, req, link);
	return (req);
}

/* The resource, looked up in the command DB on first use. */
static int
qcom_rpmh_res_find(const char *name, struct qcom_rpmh_res **resp)
{
	const struct qcom_rpmh_floor *f;
	struct qcom_rpmh_res *res;
	struct qcom_rpmh_req *req;
	const uint16_t *lv;
	uint32_t addr;
	size_t len;
	u_int n;
	UINT32 id;
	int type;

	sx_assert(&qcom_rpmh_votes_lock, SA_XLOCKED);
	TAILQ_FOREACH(res, &qcom_rpmh_resources, link)
		if (strcmp(res->name, name) == 0) {
			*resp = res;
			return (0);
		}
	if (qcom_cmd_db_ready() != 0)
		return (ENXIO);
	addr = qcom_cmd_db_read_addr(name);
	type = qcom_cmd_db_read_slave_id(name);
	if (addr == 0 || (type != QCOM_CMD_DB_HW_ARC &&
	    type != QCOM_CMD_DB_HW_BCM) || strlen(name) >= sizeof(res->name))
		return (ENOENT);
	lv = qcom_cmd_db_read_aux_data(name, &len);
	n = 0;
	if (type == QCOM_CMD_DB_HW_ARC) {
		if (lv == NULL)
			return (ENOENT);
		/* Zero padding after the first entry ends the list. */
		for (n = 1; n < len / 2 && lv[n] != 0; n++)
			;
	} else if (lv == NULL || len < 6)
		return (ENOENT);
	res = malloc(sizeof(*res), M_QCOM_RPMH, M_WAITOK | M_ZERO);
	TAILQ_INIT(&res->reqs);
	strlcpy(res->name, name, sizeof(res->name));
	res->addr = addr;
	res->arc = type == QCOM_CMD_DB_HW_ARC;
	if (res->arc) {
		res->lv = lv;
		res->nlv = n;
	} else {
		/* Linux's struct bcm_db: unit (le32), width (le16), vcd. */
		res->unit = le32dec(lv);
		res->width = le16dec((const uint8_t *)lv + 4);
		if (res->unit == 0)
			res->unit = 1;
	}
	TAILQ_INSERT_TAIL(&qcom_rpmh_resources, res, link);

	if (ACPI_SUCCESS(acpi_GetInteger(ACPI_ROOT_OBJECT, "\\_SB.SOID", &id)))
		for (f = qcom_rpmh_floors; f < qcom_rpmh_floors +
		    nitems(qcom_rpmh_floors); f++) {
			if (f->soc_id != id || strcmp(f->res, name) != 0)
				continue;
			req = qcom_rpmh_req_alloc(res, f->why);
			if (res->arc)
				req->hlvl = qcom_rpmh_arc_index(res, f->value);
			else
				req->peak = f->value;
		}
	*resp = res;
	return (0);
}

struct qcom_rpmh_req *
qcom_rpmh_req_get(const char *name, const char *client, int *errorp)
{
	struct qcom_rpmh_res *res;
	struct qcom_rpmh_req *req;
	int error;

	sx_xlock(&qcom_rpmh_votes_lock);
	error = qcom_rpmh_res_find(name, &res);
	req = error == 0 ? qcom_rpmh_req_alloc(res, client) : NULL;
	sx_xunlock(&qcom_rpmh_votes_lock);
	if (errorp != NULL)
		*errorp = error;
	return (req);
}

void
qcom_rpmh_req_put(struct qcom_rpmh_req *req)
{
	struct qcom_rpmh_res *res;

	if (req == NULL)
		return;
	sx_xlock(&qcom_rpmh_votes_lock);
	res = req->res;
	TAILQ_REMOVE(&res->reqs, req, link);
	free(req, M_QCOM_RPMH);
	(void)qcom_rpmh_res_update(res);
	sx_xunlock(&qcom_rpmh_votes_lock);
}

int
qcom_rpmh_req_level(struct qcom_rpmh_req *req, u_int vlvl)
{
	struct qcom_rpmh_res *res = req->res;
	u_int old;
	int error;

	if (!res->arc)
		return (EINVAL);
	sx_xlock(&qcom_rpmh_votes_lock);
	old = req->hlvl;
	req->hlvl = qcom_rpmh_arc_index(res, vlvl);
	if (req->hlvl >= res->nlv) {
		req->hlvl = old;
		error = EINVAL;
	} else if ((error = qcom_rpmh_res_update(res)) != 0)
		req->hlvl = old;
	sx_xunlock(&qcom_rpmh_votes_lock);
	return (error);
}

int
qcom_rpmh_req_bw(struct qcom_rpmh_req *req, uint32_t avg, uint32_t peak)
{
	struct qcom_rpmh_res *res = req->res;
	uint32_t oavg, opeak;
	int error;

	if (res->arc)
		return (EINVAL);
	sx_xlock(&qcom_rpmh_votes_lock);
	oavg = req->avg;
	opeak = req->peak;
	req->avg = MIN(avg, QCOM_RPMH_BCM_MAX);
	req->peak = MIN(peak, QCOM_RPMH_BCM_MAX);
	if ((error = qcom_rpmh_res_update(res)) != 0) {
		req->avg = oavg;
		req->peak = opeak;
	}
	sx_xunlock(&qcom_rpmh_votes_lock);
	return (error);
}

/* Linux's bcm_div(): small votes aren't lost. */
static uint64_t
qcom_rpmh_bcm_div(uint64_t num, uint32_t base)
{
	if (num != 0 && num < base)
		return (1);
	return (num / base);
}

int
qcom_rpmh_req_kbps(struct qcom_rpmh_req *req, uint32_t avg_kbps,
    uint32_t peak_kbps, u_int buswidth, u_int channels)
{
	struct qcom_rpmh_res *res = req->res;
	uint64_t x, y;

	if (res->arc || buswidth == 0 || channels == 0)
		return (EINVAL);
	/* As Linux's bcm_aggregate(), with its vote_scale of 1000. */
	x = qcom_rpmh_bcm_div((uint64_t)avg_kbps * res->width,
	    buswidth * channels);
	x = qcom_rpmh_bcm_div(x * 1000, res->unit);
	y = qcom_rpmh_bcm_div((uint64_t)peak_kbps * res->width, buswidth);
	y = qcom_rpmh_bcm_div(y * 1000, res->unit);
	return (qcom_rpmh_req_bw(req, MIN(x, QCOM_RPMH_BCM_MAX),
	    MIN(y, QCOM_RPMH_BCM_MAX)));
}

/*
 * By hand: a rail to its highest level ("nsp.lvl"), or a BCM's peak, held
 * by a "sysctl" request until the module goes (it doesn't).
 */
static int
qcom_rpmh_vote_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct qcom_rpmh_req *rq;
	char res[32];
	int error;

	res[0] = '\0';
	error = sysctl_handle_string(oidp, res, sizeof(res), req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	rq = qcom_rpmh_req_get(res, "sysctl", &error);
	if (rq == NULL)
		return (error);
	if (rq->res->arc)
		error = qcom_rpmh_req_level(rq, QCOM_RPMH_ARC_MAX);
	else
		error = qcom_rpmh_req_bw(rq, 0, QCOM_RPMH_BCM_MAX);
	return (error);
}

/* Every resource voted: its requests and what was sent. */
static int
qcom_rpmh_votes_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct qcom_rpmh_res *res;
	struct qcom_rpmh_req *rq;
	struct sbuf *sb;
	int error;

	sb = sbuf_new_for_sysctl(NULL, NULL, 256, req);
	sx_slock(&qcom_rpmh_votes_lock);
	TAILQ_FOREACH(res, &qcom_rpmh_resources, link) {
		sbuf_printf(sb, "\n%s", res->name);
		if (!res->sent_valid)
			sbuf_cat(sb, ": nothing sent");
		else if (res->arc)
			sbuf_printf(sb, ": level %u (vlvl %u)", res->sent,
			    res->lv[res->sent]);
		else
			sbuf_printf(sb, ": average %u, peak %u (unit %u, "
			    "width %u)", res->sent >> 14 & QCOM_RPMH_BCM_MAX,
			    res->sent & QCOM_RPMH_BCM_MAX, res->unit, res->width);
		TAILQ_FOREACH(rq, &res->reqs, link) {
			if (res->arc)
				sbuf_printf(sb, "\n  %-24s vlvl %u", rq->client,
				    rq->hlvl < res->nlv ? res->lv[rq->hlvl] : 0);
			else
				sbuf_printf(sb, "\n  %-24s average %u, peak %u",
				    rq->client, rq->avg, rq->peak);
		}
	}
	sx_sunlock(&qcom_rpmh_votes_lock);
	error = sbuf_finish(sb);
	sbuf_delete(sb);
	return (error);
}

SYSCTL_NODE(_hw, OID_AUTO, qcom_rpmh, CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
    "Qualcomm RPMh requests");
SYSCTL_PROC(_hw_qcom_rpmh, OID_AUTO, vote_max, CTLTYPE_STRING | CTLFLAG_WR |
    CTLFLAG_MPSAFE, NULL, 0, qcom_rpmh_vote_sysctl, "A",
    "Vote an ARC rail (\"nsp.lvl\") to its highest level, or a BCM "
    "(\"NSA0\") to its highest peak");
SYSCTL_PROC(_hw_qcom_rpmh, OID_AUTO, votes, CTLTYPE_STRING | CTLFLAG_RD |
    CTLFLAG_MPSAFE, NULL, 0, qcom_rpmh_votes_sysctl, "A",
    "The resources voted: their requests, and what was sent");

static int
qcom_rpmh_modevent(module_t mod, int type, void *data)
{

	switch (type) {
	case MOD_LOAD:
		return (0);
	case MOD_UNLOAD:
		/* Votes made stay; so would the mapping. */
		return (EBUSY);
	default:
		return (EOPNOTSUPP);
	}
}

static moduledata_t qcom_rpmh_mod = { "qcom_rpmh", qcom_rpmh_modevent, NULL };
DECLARE_MODULE(qcom_rpmh, qcom_rpmh_mod, SI_SUB_DRIVERS, SI_ORDER_ANY);
MODULE_DEPEND(qcom_rpmh, qcom_cmd_db, 1, 1, 1);
MODULE_VERSION(qcom_rpmh, 1);
