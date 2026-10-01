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
 * The LPASS SoundWire controllers (Qualcomm "SWRM" v1.6), which link the
 * codec macros to the WCD938x codec, as Linux's soundwire/qcom.c drives
 * them: the RX link carries playback; the TX link carries capture and
 * also the codec's register access.  Here, bringing a link up (its pins,
 * a reset, the frame shape, the clock), auto-enumeration of the devices on
 * it, and register reads and writes through the command FIFO, polled.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/sx.h>
#include <sys/sysctl.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <dev/qcom_audio/qcom_lpass_macro.h>
#include <dev/qcom_audio/qcom_swr.h>

/* Controller registers (v1.3 layout, which v1.6 keeps). */
#define	SWRM_COMP_HW_VERSION		0x000
#define	SWRM_COMP_CFG			0x004
#define	SWRM_COMP_CFG_ENABLE		0x01
#define	SWRM_COMP_CFG_IRQ_PULSE		0x02
#define	SWRM_COMP_STATUS		0x014
#define	SWRM_FRM_GEN_ENABLED		0x01
#define	SWRM_COMP_PARAMS		0x100
#define	SWRM_WR_FIFO_DEPTH(v)		(((v) >> 10) & 0x1f)
#define	SWRM_INTERRUPT_STATUS		0x200
#define	SWRM_INTERRUPT_MASK		0x204
#define	SWRM_INTERRUPT_CLEAR		0x208
#define	SWRM_INTERRUPT_CPU_EN		0x210
#define	SWRM_INTERRUPT_ALL		0x1ffff
#define	SWRM_INT_SPECIAL_CMD_DONE	0x400
#define	SWRM_CMD_FIFO_WR_CMD		0x300
#define	SWRM_CMD_FIFO_RD_CMD		0x304
#define	SWRM_CMD_FIFO_CMD		0x308
#define	SWRM_CMD_FIFO_FLUSH		0x1
#define	SWRM_CMD_FIFO_STATUS		0x30c
#define	SWRM_RD_FIFO_CNT(v)		(((v) >> 16) & 0x1f)
#define	SWRM_WR_FIFO_CNT(v)		(((v) >> 8) & 0x1f)
#define	SWRM_CMD_FIFO_CFG		0x314
#define	SWRM_CMD_RETRIES		0x7
#define	SWRM_CONTINUE_ON_IGNORE		0x80000000u
#define	SWRM_CMD_FIFO_RD_FIFO		0x318
#define	SWRM_RD_FIFO_CMD_ID(v)		(((v) >> 8) & 0xf)
#define	SWRM_ENUMERATOR_CFG		0x500
#define	SWRM_ENUM_DEV_ID_1(n)		(0x530 + 8 * (n))
#define	SWRM_ENUM_DEV_ID_2(n)		(0x534 + 8 * (n))
#define	SWRM_MCP_FRAME_CTRL_BANK(b)	(0x101c + 0x40 * (b))
#define	SWRM_MCP_BUS_CTRL		0x1044
#define	SWRM_MCP_BUS_CLK_START		0x02
#define	SWRM_MCP_CFG			0x1048
#define	SWRM_MCP_CFG_NO_PINGS(v)	((v) << 17)
#define	SWRM_MCP_CFG_NO_PINGS_MASK	(0x1f << 17)
#define	SWRM_MCP_SLV_STATUS		0x1090

/* A command: register, ID, device and data. */
#define	SWRM_CMD(data, dev, id, reg)					\
	((uint32_t)(reg) | (uint32_t)(id) << 16 | (uint32_t)(dev) << 20 | \
	(uint32_t)(data) << 24)
#define	SWRM_MAX_CMD_ID			14
#define	SWRM_BROADCAST_CMD_ID		15
#define	SWRM_BROADCAST_DEV		15

/* 50 rows (index 1) by 16 columns (index 7), as Linux uses on v1.6. */
#define	SWRM_FRAME_SHAPE		(7 | 1 << 3)

/* LPASS pins: configuration at 0x1000 per pin, slew rates in one word. */
#define	LPI_BASE			0x33c0000
#define	LPI_SIZE			0x20000
#define	LPI_SLEW			0x355a000
#define	LPI_CFG_MASK			0x1ff	/* pull, function, drive */
#define	LPI_PULL_NONE			0
#define	LPI_PULL_KEEPER			2

/* SC8280XP */
static const struct qcom_swr_link {
	const char	*name;
	vm_paddr_t	base;
	vm_paddr_t	cgcr;		/* its software clock gating reset */
	uint32_t	cgcr_bit;
	u_int		pins[3];	/* clock, data, data */
	u_int		slew[3];	/* each pin's slew-rate field */
} links[] = {
	[QCOM_SWR_RX] = { "RX", 0x3210000, 0x32a90a0, 0x2, { 3, 4, 5 },
	    { 8, 10, 12 } },
	[QCOM_SWR_TX] = { "TX", 0x3330000, 0x33ec010, 0x2, { 0, 1, 2 },
	    { 0, 2, 4 } },
};

struct qcom_swr {
	const struct qcom_swr_link *link;
	volatile uint32_t	*regs;
	u_int		wr_depth;
	uint8_t		wcmd_id;
	uint8_t		rcmd_id;
	bool		up;
};

static struct qcom_swr swr[nitems(links)];
static struct sx swr_lock;
SX_SYSINIT(qcom_swr, &swr_lock, "qcom_swr");

#define	RD(s, off)	((s)->regs[(off) / 4])
#define	WR(s, off, v)	((s)->regs[(off) / 4] = (v))

static void
swr_pins(const struct qcom_swr_link *l)
{
	volatile uint32_t *lpi, *slew;
	uint32_t cfg;
	u_int i;

	/* As the device tree has them: function 1, 2 mA, slew rate 1. */
	lpi = pmap_mapdev(LPI_BASE, LPI_SIZE);
	slew = pmap_mapdev(LPI_SLEW, PAGE_SIZE);
	for (i = 0; i < nitems(l->pins); i++) {
		cfg = 1 << 2 | (i == 0 ? LPI_PULL_NONE : LPI_PULL_KEEPER);
		lpi[l->pins[i] * 0x1000 / 4] =
		    (lpi[l->pins[i] * 0x1000 / 4] & ~LPI_CFG_MASK) | cfg;
		*slew = (*slew & ~(3u << l->slew[i])) | 1u << l->slew[i];
	}
	pmap_unmapdev(__DEVOLATILE(void *, slew), PAGE_SIZE);
	pmap_unmapdev(__DEVOLATILE(void *, lpi), LPI_SIZE);
}

static void
swr_cgcr_reset(const struct qcom_swr_link *l)
{
	volatile uint32_t *page, *r;

	page = pmap_mapdev(trunc_page(l->cgcr), PAGE_SIZE);
	r = page + (l->cgcr & PAGE_MASK) / 4;
	*r |= l->cgcr_bit;
	DELAY(1);
	*r &= ~l->cgcr_bit;
	pmap_unmapdev(__DEVOLATILE(void *, page), PAGE_SIZE);
}

static int
swr_wait(struct qcom_swr *s, u_int off, uint32_t mask, bool set)
{
	int i;

	for (i = 0; i < 100; i++) {
		if (((RD(s, off) & mask) != 0) == set)
			return (0);
		DELAY(500);
	}
	return (ETIMEDOUT);
}

/* Bring the link up, as Linux's qcom_swrm_init. */
static int
swr_init(struct qcom_swr *s)
{
	uint32_t v;

	swr_pins(s->link);
	swr_cgcr_reset(s->link);
	WR(s, SWRM_MCP_FRAME_CTRL_BANK(0), SWRM_FRAME_SHAPE);
	WR(s, SWRM_ENUMERATOR_CFG, 1);		/* auto-enumeration */
	WR(s, SWRM_INTERRUPT_MASK, SWRM_INTERRUPT_ALL);
	v = RD(s, SWRM_MCP_CFG);
	v = (v & ~SWRM_MCP_CFG_NO_PINGS_MASK) | SWRM_MCP_CFG_NO_PINGS(0x1f);
	WR(s, SWRM_MCP_CFG, v);
	WR(s, SWRM_MCP_BUS_CTRL, SWRM_MCP_BUS_CLK_START);
	WR(s, SWRM_CMD_FIFO_CFG, SWRM_CMD_RETRIES | SWRM_CONTINUE_ON_IGNORE);
	WR(s, SWRM_COMP_CFG, SWRM_COMP_CFG_ENABLE);
	WR(s, SWRM_COMP_CFG, SWRM_COMP_CFG_IRQ_PULSE);
	WR(s, SWRM_INTERRUPT_CLEAR, 0xffffffff);
	WR(s, SWRM_COMP_CFG, SWRM_COMP_CFG_IRQ_PULSE | SWRM_COMP_CFG_ENABLE);
	if (swr_wait(s, SWRM_COMP_STATUS, SWRM_FRM_GEN_ENABLED, true) != 0) {
		printf("qcom_swr: %s: no frames\n", s->link->name);
		return (ETIMEDOUT);
	}
	s->wr_depth = SWRM_WR_FIFO_DEPTH(RD(s, SWRM_COMP_PARAMS));
	return (0);
}

int
qcom_swr_up(u_int which, struct qcom_swr **sp)
{
	struct qcom_swr *s;
	int error;

	if (which >= nitems(links))
		return (EINVAL);
	sx_xlock(&swr_lock);
	s = &swr[which];
	if (!s->up) {
		s->link = &links[which];
		if (s->regs == NULL)
			s->regs = pmap_mapdev(s->link->base, 0x2000);
		error = swr_init(s);
		if (error != 0) {
			sx_xunlock(&swr_lock);
			return (error);
		}
		s->up = true;
		printf("qcom_swr: %s link up: version %#x, write FIFO %u\n",
		    s->link->name, RD(s, SWRM_COMP_HW_VERSION), s->wr_depth);
	}
	sx_xunlock(&swr_lock);
	*sp = s;
	return (0);
}

void
qcom_swr_down(void)
{
	u_int i;

	/* The macros stop the clocks; this just forgets the links. */
	sx_xlock(&swr_lock);
	for (i = 0; i < nitems(swr); i++)
		swr[i].up = false;
	sx_xunlock(&swr_lock);
}

/*
 * The devices attached: status 1 (attached) or 2 (alerting) in each
 * device number's two bits.
 */
uint32_t
qcom_swr_attached(struct qcom_swr *s)
{
	uint32_t st, mask;
	u_int n;

	st = RD(s, SWRM_MCP_SLV_STATUS);
	for (n = 1, mask = 0; n < 12; n++)
		if (((st >> (2 * n)) & 3) != 0)
			mask |= 1u << n;
	return (mask);
}

/* The 48-bit SoundWire ID of the device enumerated as number n. */
uint64_t
qcom_swr_dev_id(struct qcom_swr *s, u_int n)
{
	uint32_t v1, v2;

	v1 = RD(s, SWRM_ENUM_DEV_ID_1(n));
	v2 = RD(s, SWRM_ENUM_DEV_ID_2(n));
	/* As Linux assembles it from the two words' bytes. */
	return ((uint64_t)((v2 >> 8) & 0xff) | (uint64_t)(v2 & 0xff) << 8 |
	    (uint64_t)((v1 >> 24) & 0xff) << 16 |
	    (uint64_t)((v1 >> 16) & 0xff) << 24 |
	    (uint64_t)((v1 >> 8) & 0xff) << 32 | (uint64_t)(v1 & 0xff) << 40);
}

static uint8_t
swr_next_id(uint8_t *id)
{

	*id = *id < SWRM_MAX_CMD_ID ? *id + 1 : 0;
	return (*id);
}

static int
swr_wr_space(struct qcom_swr *s)
{
	int i;

	for (i = 0; i < 30; i++) {
		if (SWRM_WR_FIFO_CNT(RD(s, SWRM_CMD_FIFO_STATUS)) < s->wr_depth)
			return (0);
		DELAY(500);
	}
	return (EIO);
}

/* Write a byte to a device's register, which must be below 0x8000. */
int
qcom_swr_write(struct qcom_swr *s, u_int dev, uint16_t reg, uint8_t val)
{

	if (swr_wr_space(s) != 0)
		return (EIO);
	WR(s, SWRM_CMD_FIFO_WR_CMD, SWRM_CMD(val, dev, swr_next_id(&s->wcmd_id),
	    reg));
	return (0);
}

/* A controller register, for setting up its data ports. */
void
qcom_swr_mmio_write(struct qcom_swr *s, u_int reg, uint32_t val)
{

	WR(s, reg, val);
}

/*
 * Switch banks: broadcast the new frame shape to SCP_FRAMECTRL of the bank
 * not in use (reg), which every device and the controller take up at the
 * next frame, and wait for the controller to say it went out.
 */
int
qcom_swr_bank_switch(struct qcom_swr *s, uint16_t reg)
{
	int error;

	WR(s, SWRM_INTERRUPT_CLEAR, SWRM_INT_SPECIAL_CMD_DONE);
	if (swr_wr_space(s) != 0)
		return (EIO);
	WR(s, SWRM_CMD_FIFO_WR_CMD, SWRM_CMD(SWRM_FRAME_SHAPE,
	    SWRM_BROADCAST_DEV, SWRM_BROADCAST_CMD_ID, reg));
	error = swr_wait(s, SWRM_INTERRUPT_STATUS, SWRM_INT_SPECIAL_CMD_DONE,
	    true);
	WR(s, SWRM_INTERRUPT_CLEAR, RD(s, SWRM_INTERRUPT_STATUS));
	if (error != 0)
		printf("qcom_swr: %s: bank switch not done (status %#x)\n",
		    s->link->name, RD(s, SWRM_INTERRUPT_STATUS));
	return (error);
}

int
qcom_swr_read(struct qcom_swr *s, u_int dev, uint16_t reg, uint8_t *val)
{
	uint32_t cmd, d;
	int i, try;

	(void)swr_wr_space(s);
	cmd = SWRM_CMD(1, dev, swr_next_id(&s->rcmd_id), reg);
	for (try = 0; try < 3; try++) {
		if (try > 0) {
			DELAY(500);
			WR(s, SWRM_CMD_FIFO_CMD, SWRM_CMD_FIFO_FLUSH);
		}
		DELAY(100);
		WR(s, SWRM_CMD_FIFO_RD_CMD, cmd);
		DELAY(250);
		for (i = 0; i < 30 &&
		    SWRM_RD_FIFO_CNT(RD(s, SWRM_CMD_FIFO_STATUS)) == 0; i++)
			DELAY(500);
		d = RD(s, SWRM_CMD_FIFO_RD_FIFO);
		if (SWRM_RD_FIFO_CMD_ID(d) == s->rcmd_id) {
			*val = d & 0xff;
			return (0);
		}
	}
	return (EIO);
}

/*
 * Test: bring both links up and list what enumerated, with the SoundWire
 * IDs the devices report through their own registers too.
 */
static int
swr_probe_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct qcom_swr *s;
	uint32_t att;
	uint8_t id[6];
	u_int which, n, i;
	int error, v = 0;

	error = sysctl_handle_int(oidp, &v, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (v == 0) {
		qcom_swr_down();
		return (qcom_lpass_macro_rx(false));
	}
	error = qcom_lpass_macro_rx(true);
	if (error != 0)
		return (error);
	for (which = 0; which < nitems(links); which++) {
		error = qcom_swr_up(which, &s);
		if (error != 0)
			return (error);
		for (i = 0; i < 20 && (att = qcom_swr_attached(s)) == 0; i++)
			pause("swrenum", hz / 100);
		printf("qcom_swr: %s: devices %#x (status %#x)\n",
		    s->link->name, att, RD(s, SWRM_MCP_SLV_STATUS));
		for (n = 1; n < 12; n++) {
			if ((att & 1u << n) == 0)
				continue;
			for (i = 0; i < 6; i++)
				if (qcom_swr_read(s, n, 0x50 + i, &id[i]) != 0)
					id[i] = 0xee;
			printf("qcom_swr: %s: device %u: ID %012jx; "
			    "registers %02x%02x%02x%02x%02x%02x\n",
			    s->link->name, n, (uintmax_t)qcom_swr_dev_id(s, n),
			    id[0], id[1], id[2], id[3], id[4], id[5]);
		}
	}
	return (0);
}

SYSCTL_NODE(_hw, OID_AUTO, qcom_swr, CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
    "LPASS SoundWire links");
SYSCTL_PROC(_hw_qcom_swr, OID_AUTO, probe, CTLTYPE_INT | CTLFLAG_RW |
    CTLFLAG_MPSAFE, NULL, 0, swr_probe_sysctl, "I",
    "Bring the links up and list their devices (1), or take them down (0)");
