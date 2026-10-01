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
 * On top, votes on ARC resources, the power rails, by level: the command
 * DB holds each rail's levels, and the vote is the level's index; and on
 * BCMs, the interconnects' bandwidth nodes, as Linux's bcm-voter makes them.
 *
 * The RSC isn't described by ACPI; where it is comes from a table of SoCs,
 * which the DSDT's \_SB.SOID identifies.
 */

#include "opt_acpi.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
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
 * Vote the ARC resource (a rail, "nsp.lvl") to its level index hlvl, or to
 * its highest with QCOM_RPMH_ARC_MAX, as Linux's rpmhpd does for a power
 * domain's performance state.  The command DB lists the levels.
 */
int
qcom_rpmh_arc_vote(const char *res, u_int hlvl)
{
	const uint16_t *lv;
	size_t len;
	uint32_t addr;
	u_int n;

	if (qcom_cmd_db_ready() != 0)
		return (ENXIO);
	addr = qcom_cmd_db_read_addr(res);
	lv = qcom_cmd_db_read_aux_data(res, &len);
	if (addr == 0 || lv == NULL ||
	    qcom_cmd_db_read_slave_id(res) != QCOM_CMD_DB_HW_ARC)
		return (ENOENT);
	/* Zero padding after the first entry ends the list. */
	for (n = 1; n < len / 2 && lv[n] != 0; n++)
		;
	if (hlvl == QCOM_RPMH_ARC_MAX)
		hlvl = n - 1;
	if (hlvl >= n)
		return (EINVAL);
	if (bootverbose)
		printf("qcom_rpmh: %s (%#x) to level %u of %u (%u)\n", res,
		    addr, hlvl, n, lv[hlvl]);
	return (qcom_rpmh_write(addr, hlvl));
}

/* By hand: a rail to its highest level ("nsp.lvl"), or a BCM's peak. */
static int
qcom_rpmh_vote_sysctl(SYSCTL_HANDLER_ARGS)
{
	char res[32];
	int error;

	res[0] = '\0';
	error = sysctl_handle_string(oidp, res, sizeof(res), req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (qcom_cmd_db_read_slave_id(res) == QCOM_CMD_DB_HW_BCM)
		return (qcom_rpmh_bcm_vote(res, 0, QCOM_RPMH_BCM_MAX));
	return (qcom_rpmh_arc_vote(res, QCOM_RPMH_ARC_MAX));
}

SYSCTL_NODE(_hw, OID_AUTO, qcom_rpmh, CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
    "Qualcomm RPMh requests");
SYSCTL_PROC(_hw_qcom_rpmh, OID_AUTO, vote_max, CTLTYPE_STRING | CTLFLAG_WR |
    CTLFLAG_MPSAFE, NULL, 0, qcom_rpmh_vote_sysctl, "A",
    "Vote an ARC rail (\"nsp.lvl\") to its highest level, or a BCM "
    "(\"NSA0\") to its highest peak");

/*
 * Vote a BCM ("NSA0") to an average and peak bandwidth, in its own units
 * (Linux's BCM_TCS_CMD: commit, valid, x, y); QCOM_RPMH_BCM_MAX saturates.
 */
int
qcom_rpmh_bcm_vote(const char *bcm, uint32_t avg, uint32_t peak)
{
	uint32_t addr;

	if (qcom_cmd_db_ready() != 0)
		return (ENXIO);
	addr = qcom_cmd_db_read_addr(bcm);
	if (addr == 0 ||
	    qcom_cmd_db_read_slave_id(bcm) != QCOM_CMD_DB_HW_BCM)
		return (ENOENT);
	avg = MIN(avg, QCOM_RPMH_BCM_MAX);
	peak = MIN(peak, QCOM_RPMH_BCM_MAX);
	if (bootverbose)
		printf("qcom_rpmh: %s (%#x) to average %u, peak %u\n", bcm,
		    addr, avg, peak);
	return (qcom_rpmh_write(addr, 1u << 30 | (avg != 0 || peak != 0 ?
	    1u << 29 : 0) | avg << 14 | peak));
}

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
