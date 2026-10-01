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
 * QMP, the Qualcomm message protocol to the AOSS (always-on subsystem):
 * a descriptor at the start of the AOSS's message RAM, with each side's
 * link and channel state and acknowledgements, and a mailbox each way.
 * We write a 64-byte message into ours, ring the AOSS through the IPCC,
 * and the AOSS clears its length when it has taken it.  This is how a
 * subsystem's image is declared loaded, which Linux does before starting
 * the audio DSP.
 *
 * The answers are polled for, so no interrupt is needed.  Where the message
 * RAM is comes from a table of SoCs, which the DSDT's \_SB.SOID identifies.
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

#include <machine/atomic.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include <dev/qcom_glink/qcom_aoss.h>
#include <dev/qcom_glink/qcom_ipcc.h>

#define	QMP_DESC_MAGIC			0x00
#define	QMP_DESC_VERSION		0x04
#define	QMP_DESC_UCORE_LINK_STATE	0x0c
#define	QMP_DESC_UCORE_LINK_STATE_ACK	0x10
#define	QMP_DESC_UCORE_CH_STATE		0x14
#define	QMP_DESC_UCORE_CH_STATE_ACK	0x18
#define	QMP_DESC_MCORE_LINK_STATE	0x24
#define	QMP_DESC_MCORE_LINK_STATE_ACK	0x28
#define	QMP_DESC_MCORE_CH_STATE		0x2c
#define	QMP_DESC_MCORE_CH_STATE_ACK	0x30
#define	QMP_DESC_MCORE_MBOX_SIZE	0x34
#define	QMP_DESC_MCORE_MBOX_OFFSET	0x38
#define	QMP_STATE_UP			0x0000ffff
#define	QMP_STATE_DOWN			0xffff0000
#define	QMP_MAGIC			0x4d41494c	/* "mail" */
#define	QMP_VERSION			1
#define	QMP_MSG_LEN			64

#define	QCOM_IPCC_CLIENT_AOP		0
#define	QCOM_IPCC_SIGNAL_QMP		0

struct qcom_aoss_soc {
	uint32_t	id;		/* \_SB.SOID */
	vm_paddr_t	msgram;
	vm_size_t	size;
};

static const struct qcom_aoss_soc qcom_aoss_socs[] = {
	{ 449, 0xc300000, 0x400 },	/* SC8280XP */
};

static struct {
	struct sx		lock;
	volatile uint32_t	*ram;
	size_t			size;
	uint32_t		offset;	/* our mailbox */
	uint32_t		mbox_size;
	bool			open;
} aoss;

SX_SYSINIT(qcom_aoss, &aoss.lock, "qcom_aoss");

#define	RD(off)		(aoss.ram[(off) / 4])
#define	WR(off, v)	(aoss.ram[(off) / 4] = (v))

static const struct qcom_aoss_soc *
qcom_aoss_find_soc(void)
{
	UINT32 id;
	u_int i;

	if (ACPI_FAILURE(acpi_GetInteger(ACPI_ROOT_OBJECT, "\\_SB.SOID", &id)))
		return (NULL);
	for (i = 0; i < nitems(qcom_aoss_socs); i++)
		if (qcom_aoss_socs[i].id == id)
			return (&qcom_aoss_socs[i]);
	return (NULL);
}

static void
qcom_aoss_kick(void)
{

	wmb();
	(void)qcom_ipcc_send(QCOM_IPCC_CLIENT_AOP, QCOM_IPCC_SIGNAL_QMP);
}

/* Wait up to a second for the word at off to read want. */
static int
qcom_aoss_wait(bus_size_t off, uint32_t want)
{
	int i;

	for (i = 0; i < 1000; i++) {
		if (RD(off) == want)
			return (0);
		pause("aoss", MAX(hz / 1000, 1));
	}
	return (ETIMEDOUT);
}

static int
qcom_aoss_open(void)
{
	const struct qcom_aoss_soc *soc;
	int error;

	sx_assert(&aoss.lock, SA_XLOCKED);
	if (aoss.open)
		return (0);
	if (aoss.ram == NULL) {
		soc = qcom_aoss_find_soc();
		if (soc == NULL)
			return (ENXIO);
		aoss.ram = pmap_mapdev(soc->msgram, soc->size);
		aoss.size = soc->size;
	}
	if (RD(QMP_DESC_MAGIC) != QMP_MAGIC ||
	    RD(QMP_DESC_VERSION) != QMP_VERSION)
		return (ENXIO);
	aoss.offset = RD(QMP_DESC_MCORE_MBOX_OFFSET);
	aoss.mbox_size = RD(QMP_DESC_MCORE_MBOX_SIZE);
	if (aoss.mbox_size < 4 + QMP_MSG_LEN ||
	    aoss.offset + aoss.mbox_size > aoss.size)
		return (ENXIO);

	/* Acknowledge the AOSS's link, bring ours up. */
	WR(QMP_DESC_UCORE_LINK_STATE_ACK, RD(QMP_DESC_UCORE_LINK_STATE));
	WR(QMP_DESC_MCORE_LINK_STATE, QMP_STATE_UP);
	qcom_aoss_kick();
	error = qcom_aoss_wait(QMP_DESC_MCORE_LINK_STATE_ACK, QMP_STATE_UP);
	if (error != 0) {
		printf("qcom_aoss: the AOSS didn't acknowledge the link\n");
		goto down;
	}
	/* Our channel up, then the AOSS's, acknowledged both ways. */
	WR(QMP_DESC_MCORE_CH_STATE, QMP_STATE_UP);
	qcom_aoss_kick();
	error = qcom_aoss_wait(QMP_DESC_UCORE_CH_STATE, QMP_STATE_UP);
	if (error != 0) {
		printf("qcom_aoss: the AOSS didn't open its channel\n");
		goto chdown;
	}
	WR(QMP_DESC_UCORE_CH_STATE_ACK, QMP_STATE_UP);
	qcom_aoss_kick();
	error = qcom_aoss_wait(QMP_DESC_MCORE_CH_STATE_ACK, QMP_STATE_UP);
	if (error != 0) {
		printf("qcom_aoss: the AOSS didn't acknowledge the channel\n");
		goto chdown;
	}
	aoss.open = true;
	printf("qcom_aoss: QMP link up\n");
	return (0);
chdown:
	WR(QMP_DESC_MCORE_CH_STATE, QMP_STATE_DOWN);
down:
	WR(QMP_DESC_MCORE_LINK_STATE, QMP_STATE_DOWN);
	qcom_aoss_kick();
	return (error);
}

int
qcom_aoss_send(const char *msg)
{
	uint32_t buf[QMP_MSG_LEN / 4];
	size_t len;
	u_int i;
	int error;

	len = strlen(msg);
	if (len >= QMP_MSG_LEN)
		return (EINVAL);
	memset(buf, 0, sizeof(buf));
	memcpy(buf, msg, len);
	sx_xlock(&aoss.lock);
	error = qcom_aoss_open();
	if (error == 0) {
		/* The message RAM takes 32-bit accesses only. */
		for (i = 0; i < nitems(buf); i++)
			WR(aoss.offset + 4 + 4 * i, buf[i]);
		WR(aoss.offset, QMP_MSG_LEN);
		(void)RD(aoss.offset);
		qcom_aoss_kick();
		error = qcom_aoss_wait(aoss.offset, 0);
		if (error != 0) {
			printf("qcom_aoss: the AOSS didn't take \"%s\"\n", msg);
			WR(aoss.offset, 0);
		}
	}
	sx_xunlock(&aoss.lock);
	return (error);
}

static int
qcom_aoss_send_sysctl(SYSCTL_HANDLER_ARGS)
{
	char msg[QMP_MSG_LEN];
	int error;

	msg[0] = '\0';
	error = sysctl_handle_string(oidp, msg, sizeof(msg), req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	return (qcom_aoss_send(msg));
}

SYSCTL_NODE(_hw, OID_AUTO, qcom_aoss, CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
    "Qualcomm AOSS");
SYSCTL_PROC(_hw_qcom_aoss, OID_AUTO, send, CTLTYPE_STRING | CTLFLAG_WR |
    CTLFLAG_MPSAFE, NULL, 0, qcom_aoss_send_sysctl, "A",
    "Send a QMP message to the AOSS");
