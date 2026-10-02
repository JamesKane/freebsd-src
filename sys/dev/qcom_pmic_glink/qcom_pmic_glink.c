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
 * pmic_glink: the audio DSP's charger and USB-C services, over GLINK.
 * Here, its USB-C notifications, as Linux's pmic_glink_altmode driver
 * receives them: for each port, which way round the plug is and what its
 * lanes carry.  The USB-C PHYs' lanes are switched to match, so that
 * SuperSpeed works with the plug either way round: UEFI fixes them to the
 * normal orientation, under software control, and leaves them there.
 *
 * The PHYs (QMP USB43DP combo PHYs) come from a table of SoCs, which the
 * DSDT's \_SB.SOID identifies; ACPI describes neither them nor the service.
 */

#include "opt_acpi.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include <dev/qcom_glink/qcom_glink.h>

/* Messages: a header, then the owner's own. */
struct pmic_glink_hdr {
	uint32_t	owner;
	uint32_t	type;
	uint32_t	opcode;		/* USB-C: low byte; SVID in the top half */
} __packed;

#define	PMIC_GLINK_OWNER_USBC_PAN	32780
#define	PMIC_GLINK_REQ_RESP		1
#define	PMIC_GLINK_NOTIFY		2

#define	USBC_CMD_WRITE_REQ		0x15
#define	USBC_NOTIFY_IND			0x16
#define	ALTMODE_PAN_EN			0x10	/* send me notifications */
#define	ALTMODE_PAN_ACK			0x11	/* a port's notification handled */

/* DisplayPort alternate mode: its SVID, and its byte of the extended data. */
#define	USB_TYPEC_DP_SVID		0xff01
#define	DP_PIN_ASSIGNMENT(b)		((b) & 0x3f)	/* 1 A ... 6 F */
#define	DP_HPD_STATE(b)			(((b) >> 6) & 1)
#define	DP_HPD_IRQ(b)			(((b) >> 7) & 1)

struct usbc_write_req {
	struct pmic_glink_hdr hdr;
	uint32_t	cmd;
	uint32_t	arg;
	uint32_t	reserved;
} __packed;

struct usbc_notify {
	struct pmic_glink_hdr hdr;
	uint8_t		port_idx;
	uint8_t		orientation;	/* 0 normal, 1 reversed, else none */
	uint8_t		mux_ctrl;	/* 0 none, 1 USB3, 2 DP, 3 USB3+DP, ... */
	uint8_t		res;
	uint16_t	vid;
	uint16_t	svid;
	uint8_t		extended[8];	/* DisplayPort or tunneling */
	uint32_t	reserved;
} __packed;

#define	ORIENTATION_NORMAL	0
#define	ORIENTATION_REVERSE	1

/* QMP combo PHY: the common block's lane select, and the USB3 PCS. */
#define	QMP_COM_TYPEC_CTRL	0x0010
#define	SW_PORTSELECT_VAL	0x01	/* the reversed lanes */
#define	SW_PORTSELECT_MUX	0x02	/* software selects */
#define	QMP_USB3_PCS_SW_RESET	0x1400
#define	PCS_SW_RESET		0x01
#define	QMP_SIZE		0x2000

#define	PMIC_GLINK_PORTS	2
#define	PMIC_GLINK_INTENT_SIZE	1024
#define	PMIC_GLINK_INTENTS	8
#define	PMIC_GLINK_START_TRIES	120	/* a second apart, for the DSP */

struct pmic_glink_soc {
	uint32_t	id;		/* \_SB.SOID */
	vm_paddr_t	phy[PMIC_GLINK_PORTS];
};

static const struct pmic_glink_soc pmic_glink_socs[] = {
	{ 449, { 0x88eb000, 0x8903000 } },	/* SC8280XP */
};

static struct {
	struct mtx		mtx;
	const struct pmic_glink_soc *soc;
	volatile uint32_t	*phy[PMIC_GLINK_PORTS];
	int			orientation[PMIC_GLINK_PORTS]; /* -1 none */
	int			mux[PMIC_GLINK_PORTS];
	/* DisplayPort: pin assignment (-1 none) and HPD, as last told. */
	int			dp_pin[PMIC_GLINK_PORTS];
	int			dp_hpd[PMIC_GLINK_PORTS];
	u_int			ack_ports;	/* notifications to acknowledge */
	struct task		ack_task;
	struct qcom_glink_chan	*ch;
	struct taskqueue	*tq;
	struct timeout_task	start_task;
	int			start_tries;
} pg;

static const struct pmic_glink_soc *
pmic_glink_find_soc(void)
{
	UINT32 id;
	u_int i;

	if (ACPI_FAILURE(acpi_GetInteger(ACPI_ROOT_OBJECT, "\\_SB.SOID", &id)))
		return (NULL);
	for (i = 0; i < nitems(pmic_glink_socs); i++)
		if (pmic_glink_socs[i].id == id)
			return (&pmic_glink_socs[i]);
	return (NULL);
}

/*
 * Switch the port's PHY to the plug's lanes, as Linux's QMP combo PHY
 * driver does on an orientation change, and restart its USB3 PCS so that
 * the link trains on them.  (Linux reinitializes the whole PHY; the lane
 * tables are the same for both lane pairs, so the PCS restart is enough.)
 */
static void
pmic_glink_set_orientation(u_int port, int orientation)
{
	volatile uint32_t *phy;
	uint32_t sel;

	mtx_assert(&pg.mtx, MA_OWNED);
	if (orientation != ORIENTATION_NORMAL &&
	    orientation != ORIENTATION_REVERSE)
		return;		/* nothing in it: leave the lanes be */
	phy = pg.phy[port];
	sel = SW_PORTSELECT_MUX |
	    (orientation == ORIENTATION_REVERSE ? SW_PORTSELECT_VAL : 0);
	if ((phy[QMP_COM_TYPEC_CTRL / 4] & 0xff) == sel)
		return;
	phy[QMP_COM_TYPEC_CTRL / 4] = sel;
	phy[QMP_USB3_PCS_SW_RESET / 4] = PCS_SW_RESET;
	DELAY(10);
	phy[QMP_USB3_PCS_SW_RESET / 4] = 0;
	if (bootverbose)
		printf("qcom_pmic_glink: port %u: lanes %s\n", port,
		    orientation == ORIENTATION_REVERSE ? "reversed" : "normal");
}

static void
pmic_glink_rx(void *arg __unused, const void *data, size_t len)
{
	const struct pmic_glink_hdr *h = data;
	const struct usbc_notify *n = data;

	if (len < sizeof(*h) || h->owner != PMIC_GLINK_OWNER_USBC_PAN)
		return;
	switch (h->opcode & 0xff) {
	case USBC_CMD_WRITE_REQ:
		/* The acknowledgement of our request for notifications. */
		break;
	case USBC_NOTIFY_IND:
		if (len != sizeof(*n) || n->port_idx >= PMIC_GLINK_PORTS)
			break;
		mtx_lock(&pg.mtx);
		pg.orientation[n->port_idx] = n->orientation <=
		    ORIENTATION_REVERSE ? n->orientation : -1;
		pg.mux[n->port_idx] = n->mux_ctrl;
		if (h->opcode >> 16 == USB_TYPEC_DP_SVID) {
			pg.dp_pin[n->port_idx] =
			    DP_PIN_ASSIGNMENT(n->extended[0]);
			pg.dp_hpd[n->port_idx] = DP_HPD_STATE(n->extended[0]);
		} else
			pg.dp_pin[n->port_idx] = pg.dp_hpd[n->port_idx] = -1;
		pmic_glink_set_orientation(n->port_idx, n->orientation);
		/*
		 * Acknowledged once handled, as Linux does, from the task
		 * queue: sending here, on GLINK's receive thread, could wait
		 * for buffers announced on that thread.
		 */
		pg.ack_ports |= 1u << n->port_idx;
		mtx_unlock(&pg.mtx);
		taskqueue_enqueue(pg.tq, &pg.ack_task);
		if (bootverbose)
			printf("qcom_pmic_glink: port %u: orientation %u, "
			    "mux %u, svid %#x, extended %#x\n", n->port_idx,
			    n->orientation, n->mux_ctrl, h->opcode >> 16,
			    n->extended[0]);
		break;
	}
}

/* Open the channel once the DSP is up, and ask for the notifications. */
static void
pmic_glink_start(void *arg __unused, int pending __unused)
{
	struct usbc_write_req req;
	struct qcom_glink_chan *ch;
	u_int i;
	int error;

	if (qcom_glink_absent())
		return;
	/*
	 * Which SoC, here rather than at load: built into the kernel, this
	 * loads before ACPI is up.  Not one we know: nothing to do.
	 */
	if (pg.soc == NULL) {
		pg.soc = pmic_glink_find_soc();
		if (pg.soc == NULL) {
			if (++pg.start_tries < PMIC_GLINK_START_TRIES)
				taskqueue_enqueue_timeout(pg.tq,
				    &pg.start_task, hz);
			return;
		}
		for (i = 0; i < PMIC_GLINK_PORTS; i++)
			pg.phy[i] = pmap_mapdev(pg.soc->phy[i], QMP_SIZE);
	}
	/*
	 * Only once the DSP's service has opened its end: before, each try
	 * timed out and was closed, and a dozen such opens and closes of one
	 * channel while the ADSP came up left it, on some boots, answering
	 * nothing more on the edge (no audio).  Linux's client binds then.
	 */
	if (!qcom_glink_announced("lpass", "PMIC_RTR_ADSP_APPS")) {
		if (++pg.start_tries < PMIC_GLINK_START_TRIES)
			taskqueue_enqueue_timeout(pg.tq, &pg.start_task, hz);
		else
			printf("qcom_pmic_glink: the DSP's service never "
			    "came\n");
		return;
	}
	error = qcom_glink_open("lpass", "PMIC_RTR_ADSP_APPS",
	    PMIC_GLINK_INTENT_SIZE, PMIC_GLINK_INTENTS, pmic_glink_rx, NULL,
	    &ch);
	if (error != 0) {
		if (++pg.start_tries < PMIC_GLINK_START_TRIES)
			taskqueue_enqueue_timeout(pg.tq, &pg.start_task, hz);
		else
			printf("qcom_pmic_glink: no channel to the DSP: %d\n",
			    error);
		return;
	}
	mtx_lock(&pg.mtx);
	pg.ch = ch;
	mtx_unlock(&pg.mtx);
	memset(&req, 0, sizeof(req));
	req.hdr.owner = PMIC_GLINK_OWNER_USBC_PAN;
	req.hdr.type = PMIC_GLINK_REQ_RESP;
	req.hdr.opcode = USBC_CMD_WRITE_REQ;
	req.cmd = ALTMODE_PAN_EN;
	error = qcom_glink_send(ch, &req, sizeof(req));
	if (error != 0)
		printf("qcom_pmic_glink: no USB-C notifications: %d\n", error);
}

static void
pmic_glink_ack(void *arg __unused, int pending __unused)
{
	struct usbc_write_req req;
	struct qcom_glink_chan *ch;
	u_int i, ports;

	mtx_lock(&pg.mtx);
	ports = pg.ack_ports;
	pg.ack_ports = 0;
	ch = pg.ch;
	mtx_unlock(&pg.mtx);
	for (i = 0; ch != NULL && i < PMIC_GLINK_PORTS; i++) {
		if ((ports & (1u << i)) == 0)
			continue;
		memset(&req, 0, sizeof(req));
		req.hdr.owner = PMIC_GLINK_OWNER_USBC_PAN;
		req.hdr.type = PMIC_GLINK_REQ_RESP;
		req.hdr.opcode = USBC_CMD_WRITE_REQ;
		req.cmd = ALTMODE_PAN_ACK;
		req.arg = i;
		(void)qcom_glink_send(ch, &req, sizeof(req));
	}
}

static int
pmic_glink_port_sysctl(SYSCTL_HANDLER_ARGS)
{
	static const char *const mux[] = { "none", "usb3", "dp", "usb3+dp",
	    "tunneling" };
	char buf[64];
	u_int port = arg2;
	int hpd, o, m, pin;

	mtx_lock(&pg.mtx);
	o = pg.orientation[port];
	m = pg.mux[port];
	pin = pg.dp_pin[port];
	hpd = pg.dp_hpd[port];
	mtx_unlock(&pg.mtx);
	snprintf(buf, sizeof(buf), "%s, %s",
	    o == ORIENTATION_NORMAL ? "normal" :
	    o == ORIENTATION_REVERSE ? "reversed" : "unplugged",
	    m >= 0 && m < (int)nitems(mux) ? mux[m] : "unknown");
	/* DisplayPort: pin assignment C, D, ... and the sink's HPD. */
	if (pin > 0 && pin <= 6)
		snprintf(buf + strlen(buf), sizeof(buf) - strlen(buf),
		    ", dp pin %c, hpd %s", 'A' + pin - 1, hpd ? "high" : "low");
	return (sysctl_handle_string(oidp, buf, sizeof(buf), req));
}

SYSCTL_NODE(_hw, OID_AUTO, qcom_pmic_glink, CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
    "Qualcomm pmic_glink (USB-C)");
SYSCTL_PROC(_hw_qcom_pmic_glink, OID_AUTO, port0, CTLTYPE_STRING |
    CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, 0, pmic_glink_port_sysctl, "A",
    "USB-C port 0: the plug's orientation, and what its lanes carry");
SYSCTL_PROC(_hw_qcom_pmic_glink, OID_AUTO, port1, CTLTYPE_STRING |
    CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, 1, pmic_glink_port_sysctl, "A",
    "USB-C port 1: the plug's orientation, and what its lanes carry");

static int
qcom_pmic_glink_modevent(module_t mod, int type, void *data)
{
	struct qcom_glink_chan *ch;
	u_int i;

	switch (type) {
	case MOD_LOAD:
		mtx_init(&pg.mtx, "qcom_pmic_glink", NULL, MTX_DEF);
		for (i = 0; i < PMIC_GLINK_PORTS; i++)
			pg.orientation[i] = pg.mux[i] = pg.dp_pin[i] =
			    pg.dp_hpd[i] = -1;
		pg.tq = taskqueue_create("qcom_pmic_glink", M_WAITOK,
		    taskqueue_thread_enqueue, &pg.tq);
		taskqueue_start_threads(&pg.tq, 1, PWAIT, "qcom_pmic_glink");
		TIMEOUT_TASK_INIT(pg.tq, &pg.start_task, 0, pmic_glink_start,
		    NULL);
		TASK_INIT(&pg.ack_task, 0, pmic_glink_ack, NULL);
		taskqueue_enqueue_timeout(pg.tq, &pg.start_task, 0);
		return (0);
	case MOD_UNLOAD:
		taskqueue_drain_timeout(pg.tq, &pg.start_task);
		taskqueue_drain(pg.tq, &pg.ack_task);
		taskqueue_free(pg.tq);
		mtx_lock(&pg.mtx);
		ch = pg.ch;
		pg.ch = NULL;
		mtx_unlock(&pg.mtx);
		if (ch != NULL)
			qcom_glink_close(ch);
		for (i = 0; pg.soc != NULL && i < PMIC_GLINK_PORTS; i++)
			pmap_unmapdev(__DEVOLATILE(void *, pg.phy[i]), QMP_SIZE);
		mtx_destroy(&pg.mtx);
		return (0);
	default:
		return (EOPNOTSUPP);
	}
}

static moduledata_t qcom_pmic_glink_mod = { "qcom_pmic_glink",
    qcom_pmic_glink_modevent, NULL };
DECLARE_MODULE(qcom_pmic_glink, qcom_pmic_glink_mod, SI_SUB_DRIVERS,
    SI_ORDER_ANY);
MODULE_DEPEND(qcom_pmic_glink, qcom_glink, 1, 1, 1);
MODULE_VERSION(qcom_pmic_glink, 1);
