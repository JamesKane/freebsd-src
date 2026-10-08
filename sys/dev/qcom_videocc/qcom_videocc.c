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
 * Qualcomm video clock controller and the codec's power; see
 * qcom_videocc.h.
 *
 * Linux's references: videocc-sm8350.c (which the SC8280XP shares, with
 * its own offsets for a few registers), the GCC's video clocks, and the Iris
 * driver's power-on order.  The rates are fixed: video_pll0 at 1599 MHz,
 * which the controller clock divides by 2 (799.5 MHz) and the core clock by
 * 3 (533 MHz, the OPP Linux runs the codec at, which needs MMCX at turbo).
 *
 * RPMh keeps one vote per rail for the application processors, and nothing
 * here aggregates votes: the display runs on what UEFI voted for MMCX.  The
 * rails are therefore only ever raised, never lowered, and the video clock
 * controller is touched only once they are: it lives in MMCX, and reading
 * it with MMCX off resets the SoC.
 */

#include "opt_acpi.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/sx.h>
#include <sys/sysctl.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include <dev/qcom_rpmh/qcom_rpmh.h>
#include <dev/qcom_videocc/qcom_videocc.h>

/* Branch clock control register (CBCR). */
#define	CBCR_CLK_OFF		(1u << 31)
#define	CBCR_ARES		(1u << 2)	/* the block's reset */
#define	CBCR_CLK_ENABLE		(1u << 0)

/* Root clock generator (RCG2): command and configuration registers. */
#define	RCG_CMD_UPDATE		(1u << 0)
#define	RCG_CFG(cmd)		((cmd) + 4)
#define	RCG_CFG_SRC_SEL(s)	((s) << 8)
#define	RCG_CFG_SRC_DIV(d2)	((d2) - 1)	/* in half steps: 2 * div */
#define	RCG_CFG_MASK		0x71f

/* Power domain (GDSC) control register. */
#define	GDSC_PWR_ON		(1u << 31)
#define	GDSC_WAIT_MASK		0x00fff000
#define	GDSC_WAIT(rest, few, clkdis)	\
	((rest) << 20 | (few) << 16 | (clkdis) << 12)
#define	GDSC_RETAIN_FF		(1u << 11)
#define	GDSC_HW_CONTROL		(1u << 1)
#define	GDSC_SW_COLLAPSE	(1u << 0)

/* Lucid 5LPE PLL, from its base. */
#define	PLL_MODE		0x00
#define	 PLL_LOCK_DET		(1u << 31)
#define	 PLL_UPDATE_BYPASS	(1u << 23)
#define	 PLL_LATCH_INPUT	(1u << 14)
#define	 PLL_ACK_LATCH		(1u << 13)
#define	 PLL_RESET_N		(1u << 2)
#define	 PLL_OUTCTRL		(1u << 0)
#define	PLL_L_VAL		0x04
#define	PLL_CAL_L_VAL		0x08
#define	 PLL_CAL_VAL		0x44
#define	PLL_USER_CTL		0x0c
#define	 PLL_ENABLE_VOTE_RUN	(1u << 21)
#define	 PLL_OUT_MASK		0x7
#define	PLL_USER_CTL_U		0x10
#define	PLL_USER_CTL_U1		0x14
#define	PLL_CONFIG_CTL		0x18
#define	PLL_CONFIG_CTL_U	0x1c
#define	PLL_CONFIG_CTL_U1	0x20
#define	PLL_TEST_CTL		0x24
#define	PLL_TEST_CTL_U		0x28
#define	PLL_TEST_CTL_U1		0x2c
#define	PLL_OPMODE		0x38
#define	 PLL_STANDBY		0x0
#define	 PLL_RUN		0x1
#define	PLL_ALPHA_VAL		0x40

#define	POLL_US			2000
#define	RESET_US		400	/* as Linux's resets for these blocks */

struct qcom_videocc_pll_cfg {
	uint32_t	l;		/* rate = XO * (l + alpha / 2^16) */
	uint32_t	alpha;
	/* The rest as Linux writes them: zeros are left alone. */
	uint32_t	config_ctl, config_ctl_u, config_ctl_u1;
	uint32_t	user_ctl, user_ctl_u, user_ctl_u1;
	uint32_t	test_ctl, test_ctl_u, test_ctl_u1;
};

struct qcom_videocc_desc {
	uint32_t	soc_id;		/* \_SB.SOID */
	/* The GCC's video registers, and the video clock controller. */
	vm_paddr_t	gcc_base;
	vm_size_t	gcc_size;
	vm_paddr_t	cc_base;
	vm_size_t	cc_size;
	/* Rails, as Linux's OPP for the core clock needs them. */
	u_int		mx_vlvl;
	u_int		mmcx_vlvl;
	/* GCC (relative to gcc_base): always on, and the codec's bus. */
	uint32_t	gcc_ahb, gcc_xo, gcc_axi;
	/* Video CC: always on. */
	uint32_t	cc_ahb, cc_xo;
	/* The PLL, and the core clock source it feeds. */
	uint32_t	pll;
	const struct qcom_videocc_pll_cfg *pll_cfg;
	uint32_t	core_rcg;
	uint32_t	core_rcg_cfg;
	/* Controller and hardware: power domain, clock (with its reset). */
	uint32_t	ctrl_gdscr, ctrl_cbcr;
	uint32_t	hw_gdscr, hw_cbcr;
};

/* 1599 MHz: 19.2 MHz * (0x53 + 0x4800 / 2^16); otherwise as Linux's. */
static const struct qcom_videocc_pll_cfg sc8280xp_pll0 = {
	.l = 0x53,
	.alpha = 0x4800,
	.config_ctl = 0x20485699,
	.config_ctl_u = 0x00002261,
	.config_ctl_u1 = 0x2a9a699c,
	.user_ctl_u = 0x00000805,
	.test_ctl_u1 = 0x01800000,
};

static const struct qcom_videocc_desc qcom_videocc_socs[] = {
	{
		.soc_id = 449,			/* SC8280XP */
		.gcc_base = 0x128000,		/* GCC 0x100000 + 0x28000 */
		.gcc_size = 0x1000,
		.cc_base = 0xabf0000,
		.cc_size = 0x10000,
		.mx_vlvl = 256,			/* nominal */
		.mmcx_vlvl = 384,		/* turbo */
		.gcc_ahb = 0x004,
		.gcc_xo = 0x028,
		.gcc_axi = 0x010,		/* GCC_VIDEO_AXI0 */
		.cc_ahb = 0xe58,
		.cc_xo = 0xf34,
		.pll = 0x42c,			/* video_pll0 */
		.pll_cfg = &sc8280xp_pll0,
		/* mvs0_clk_src: video_pll0 (source 1), undivided. */
		.core_rcg = 0xb94,
		.core_rcg_cfg = RCG_CFG_SRC_SEL(1) | RCG_CFG_SRC_DIV(2),
		.ctrl_gdscr = 0xbf8,		/* MVS0C */
		.ctrl_cbcr = 0xc34,
		.hw_gdscr = 0xd18,		/* MVS0 */
		.hw_cbcr = 0xd34,
	},
};

struct qcom_videocc {
	device_t			dev;
	const struct qcom_videocc_desc	*desc;
	volatile uint32_t		*gcc;
	volatile uint32_t		*cc;
	bool				voted;
	bool				ctrl_on;
	bool				hw_on;
};

static MALLOC_DEFINE(M_QCOM_VIDEOCC, "qcom_videocc", "Qualcomm video clocks");

#define	VCC_PRINTF(sc, ...) do {					\
	if ((sc)->dev != NULL)						\
		device_printf((sc)->dev, __VA_ARGS__);			\
	else {								\
		printf("qcom_videocc: ");				\
		printf(__VA_ARGS__);					\
	}								\
} while (0)

/* Register access; "gcc" selects the GCC window over the video CC. */
static uint32_t
qcom_videocc_read(struct qcom_videocc *sc, bool gcc, uint32_t reg)
{
	return (gcc ? sc->gcc[reg / 4] : sc->cc[reg / 4]);
}

static void
qcom_videocc_write(struct qcom_videocc *sc, bool gcc, uint32_t reg,
    uint32_t v)
{
	if (gcc)
		sc->gcc[reg / 4] = v;
	else
		sc->cc[reg / 4] = v;
	wmb();
}

static void
qcom_videocc_set(struct qcom_videocc *sc, bool gcc, uint32_t reg,
    uint32_t clr, uint32_t set)
{
	qcom_videocc_write(sc, gcc, reg,
	    (qcom_videocc_read(sc, gcc, reg) & ~clr) | set);
}

static int
qcom_videocc_poll(struct qcom_videocc *sc, bool gcc, uint32_t reg,
    uint32_t mask, uint32_t want, const char *what)
{
	int i;

	for (i = 0; i < POLL_US; i++) {
		if ((qcom_videocc_read(sc, gcc, reg) & mask) == want)
			return (0);
		DELAY(1);
	}
	VCC_PRINTF(sc, "%s timed out (%#x = %#x)\n", what, reg,
	    qcom_videocc_read(sc, gcc, reg));
	return (ETIMEDOUT);
}

/* A branch clock on; halt: wait for it to run. */
static int
qcom_videocc_branch_enable(struct qcom_videocc *sc, bool gcc, uint32_t reg,
    bool halt, const char *name)
{
	int error;

	qcom_videocc_set(sc, gcc, reg, 0, CBCR_CLK_ENABLE);
	if (!halt)
		return (0);
	error = qcom_videocc_poll(sc, gcc, reg, CBCR_CLK_OFF, 0, name);
	if (error != 0)		/* don't leave it enabled */
		qcom_videocc_set(sc, gcc, reg, CBCR_CLK_ENABLE, 0);
	return (error);
}

static void
qcom_videocc_branch_disable(struct qcom_videocc *sc, bool gcc, uint32_t reg)
{
	qcom_videocc_set(sc, gcc, reg, CBCR_CLK_ENABLE, 0);
}

/* The rails up, once (see above: never lowered). */
static int
qcom_videocc_vote(struct qcom_videocc *sc)
{
	const struct qcom_videocc_desc *d = sc->desc;
	int error;

	if (sc->voted)
		return (0);
	error = qcom_rpmh_arc_vote_level("mx.lvl", d->mx_vlvl);
	if (error == 0)
		error = qcom_rpmh_arc_vote_level("mmcx.lvl", d->mmcx_vlvl);
	if (error != 0) {
		VCC_PRINTF(sc, "cannot vote the rails: %d\n", error);
		return (error);
	}
	sc->voted = true;
	return (0);
}

static bool
qcom_videocc_pll_running(struct qcom_videocc *sc)
{
	uint32_t p = sc->desc->pll;

	return ((qcom_videocc_read(sc, false, p + PLL_OPMODE) & PLL_RUN) != 0 &&
	    (qcom_videocc_read(sc, false, p + PLL_MODE) & PLL_OUTCTRL) != 0);
}

static void
qcom_videocc_pll_write(struct qcom_videocc *sc, uint32_t reg, uint32_t v)
{
	if (v != 0)
		qcom_videocc_write(sc, false, sc->desc->pll + reg, v);
}

/*
 * The PLL at its rate.  If firmware left it running, retune it through the
 * latch, as Linux's set_rate (stopping it could hang clocks it feeds);
 * otherwise configure it and start it, as Linux's configure and enable.
 */
static int
qcom_videocc_pll_enable(struct qcom_videocc *sc)
{
	const struct qcom_videocc_pll_cfg *c = sc->desc->pll_cfg;
	uint32_t p = sc->desc->pll;
	int error;

	if ((qcom_videocc_read(sc, false, p + PLL_USER_CTL) &
	    PLL_ENABLE_VOTE_RUN) != 0) {
		VCC_PRINTF(sc, "video PLL is in FSM mode\n");
		return (ENXIO);
	}
	if (qcom_videocc_pll_running(sc)) {
		if (qcom_videocc_read(sc, false, p + PLL_L_VAL) == c->l &&
		    qcom_videocc_read(sc, false, p + PLL_ALPHA_VAL) == c->alpha)
			return (0);
		qcom_videocc_write(sc, false, p + PLL_L_VAL, c->l);
		qcom_videocc_write(sc, false, p + PLL_ALPHA_VAL, c->alpha);
		qcom_videocc_set(sc, false, p + PLL_MODE, 0, PLL_LATCH_INPUT);
		DELAY(1);
		if ((qcom_videocc_read(sc, false, p + PLL_MODE) &
		    PLL_ACK_LATCH) == 0)
			VCC_PRINTF(sc, "video PLL didn't latch its rate\n");
		qcom_videocc_set(sc, false, p + PLL_MODE, PLL_LATCH_INPUT, 0);
		error = qcom_videocc_poll(sc, false, p + PLL_MODE,
		    PLL_LOCK_DET, PLL_LOCK_DET, "video PLL lock");
		DELAY(100);
		return (error);
	}

	/* Configure. */
	qcom_videocc_pll_write(sc, PLL_L_VAL, c->l);
	qcom_videocc_write(sc, false, p + PLL_CAL_L_VAL, PLL_CAL_VAL);
	qcom_videocc_pll_write(sc, PLL_ALPHA_VAL, c->alpha);
	qcom_videocc_pll_write(sc, PLL_CONFIG_CTL, c->config_ctl);
	qcom_videocc_pll_write(sc, PLL_CONFIG_CTL_U, c->config_ctl_u);
	qcom_videocc_pll_write(sc, PLL_CONFIG_CTL_U1, c->config_ctl_u1);
	qcom_videocc_pll_write(sc, PLL_USER_CTL, c->user_ctl);
	qcom_videocc_pll_write(sc, PLL_USER_CTL_U, c->user_ctl_u);
	qcom_videocc_pll_write(sc, PLL_USER_CTL_U1, c->user_ctl_u1);
	qcom_videocc_pll_write(sc, PLL_TEST_CTL, c->test_ctl);
	qcom_videocc_pll_write(sc, PLL_TEST_CTL_U, c->test_ctl_u);
	qcom_videocc_pll_write(sc, PLL_TEST_CTL_U1, c->test_ctl_u1);
	qcom_videocc_set(sc, false, p + PLL_MODE, 0, PLL_UPDATE_BYPASS);
	qcom_videocc_set(sc, false, p + PLL_MODE, PLL_OUTCTRL, 0);
	qcom_videocc_write(sc, false, p + PLL_OPMODE, PLL_STANDBY);
	qcom_videocc_set(sc, false, p + PLL_MODE, 0, PLL_RESET_N);

	/* Enable. */
	qcom_videocc_write(sc, false, p + PLL_OPMODE, PLL_RUN);
	error = qcom_videocc_poll(sc, false, p + PLL_MODE, PLL_LOCK_DET,
	    PLL_LOCK_DET, "video PLL lock");
	if (error != 0) {
		qcom_videocc_write(sc, false, p + PLL_OPMODE, PLL_STANDBY);
		return (error);
	}
	qcom_videocc_set(sc, false, p + PLL_USER_CTL, 0, PLL_OUT_MASK);
	qcom_videocc_set(sc, false, p + PLL_MODE, 0, PLL_OUTCTRL);
	return (0);
}

static void
qcom_videocc_pll_disable(struct qcom_videocc *sc)
{
	uint32_t p = sc->desc->pll;

	qcom_videocc_set(sc, false, p + PLL_MODE, PLL_OUTCTRL, 0);
	qcom_videocc_write(sc, false, p + PLL_OPMODE, PLL_STANDBY);
	qcom_videocc_set(sc, false, p + PLL_USER_CTL, PLL_OUT_MASK, 0);
}

static int
qcom_videocc_rcg_set(struct qcom_videocc *sc, uint32_t cmd, uint32_t cfg)
{
	qcom_videocc_set(sc, false, RCG_CFG(cmd), RCG_CFG_MASK, cfg);
	qcom_videocc_set(sc, false, cmd, 0, RCG_CMD_UPDATE);
	return (qcom_videocc_poll(sc, false, cmd, RCG_CMD_UPDATE, 0,
	    "core clock source"));
}

static int
qcom_videocc_gdsc_enable(struct qcom_videocc *sc, uint32_t gdscr,
    const char *name)
{
	int error;

	qcom_videocc_set(sc, false, gdscr, GDSC_WAIT_MASK,
	    GDSC_WAIT(0x2, 0x8, 0x2));
	qcom_videocc_set(sc, false, gdscr, GDSC_SW_COLLAPSE, 0);
	error = qcom_videocc_poll(sc, false, gdscr, GDSC_PWR_ON, GDSC_PWR_ON,
	    name);
	if (error != 0) {
		qcom_videocc_set(sc, false, gdscr, 0, GDSC_SW_COLLAPSE);
		return (error);
	}
	/* Clocks must not be enabled within 400 ns of powering the memories. */
	DELAY(1);
	qcom_videocc_set(sc, false, gdscr, 0, GDSC_RETAIN_FF);
	return (0);
}

static void
qcom_videocc_gdsc_disable(struct qcom_videocc *sc, uint32_t gdscr)
{
	qcom_videocc_set(sc, false, gdscr, GDSC_RETAIN_FF, GDSC_SW_COLLAPSE);
	DELAY(500);
}

/*
 * The controller: rails, always-on clocks, the PLL and the core clock's
 * source, the controller's power domain, the bus and controller resets
 * pulsed, then their clocks (Iris's power_on_controller).
 */
int
qcom_videocc_ctrl_enable(struct qcom_videocc *sc)
{
	const struct qcom_videocc_desc *d = sc->desc;
	int error;

	if (sc->ctrl_on)
		return (0);
	if ((error = qcom_videocc_vote(sc)) != 0)
		return (error);
	/* GCC's always-on video clocks: the video CC's register access. */
	qcom_videocc_set(sc, true, d->gcc_ahb, 0, CBCR_CLK_ENABLE);
	qcom_videocc_set(sc, true, d->gcc_xo, 0, CBCR_CLK_ENABLE);
	qcom_videocc_set(sc, false, d->cc_ahb, 0, CBCR_CLK_ENABLE);
	qcom_videocc_set(sc, false, d->cc_xo, 0, CBCR_CLK_ENABLE);
	if (bootverbose)
		VCC_PRINTF(sc, "video PLL mode %#x opmode %#x l %#x alpha %#x; "
		    "controller power %#x, core power %#x\n",
		    qcom_videocc_read(sc, false, d->pll + PLL_MODE),
		    qcom_videocc_read(sc, false, d->pll + PLL_OPMODE),
		    qcom_videocc_read(sc, false, d->pll + PLL_L_VAL),
		    qcom_videocc_read(sc, false, d->pll + PLL_ALPHA_VAL),
		    qcom_videocc_read(sc, false, d->ctrl_gdscr),
		    qcom_videocc_read(sc, false, d->hw_gdscr));

	if ((error = qcom_videocc_pll_enable(sc)) != 0)
		return (error);
	if ((error = qcom_videocc_rcg_set(sc, d->core_rcg,
	    d->core_rcg_cfg)) != 0)
		goto pll;
	if ((error = qcom_videocc_gdsc_enable(sc, d->ctrl_gdscr,
	    "controller power")) != 0)
		goto pll;

	qcom_videocc_set(sc, true, d->gcc_axi, 0, CBCR_ARES);
	qcom_videocc_set(sc, false, d->ctrl_cbcr, 0, CBCR_ARES);
	DELAY(RESET_US);
	qcom_videocc_set(sc, false, d->ctrl_cbcr, CBCR_ARES, 0);
	qcom_videocc_set(sc, true, d->gcc_axi, CBCR_ARES, 0);
	DELAY(RESET_US);

	/* The bus clock's status isn't checked (Linux's BRANCH_HALT_SKIP). */
	qcom_videocc_branch_enable(sc, true, d->gcc_axi, false, "bus clock");
	if ((error = qcom_videocc_branch_enable(sc, false, d->ctrl_cbcr, true,
	    "controller clock")) != 0)
		goto axi;
	sc->ctrl_on = true;
	return (0);

axi:
	qcom_videocc_branch_disable(sc, true, d->gcc_axi);
	qcom_videocc_gdsc_disable(sc, d->ctrl_gdscr);
pll:
	qcom_videocc_pll_disable(sc);
	return (error);
}

void
qcom_videocc_ctrl_disable(struct qcom_videocc *sc)
{
	const struct qcom_videocc_desc *d = sc->desc;

	if (!sc->ctrl_on)
		return;
	qcom_videocc_hw_disable(sc);
	qcom_videocc_branch_disable(sc, false, d->ctrl_cbcr);
	qcom_videocc_branch_disable(sc, true, d->gcc_axi);
	qcom_videocc_gdsc_disable(sc, d->ctrl_gdscr);
	qcom_videocc_pll_disable(sc);
	sc->ctrl_on = false;
}

/* The codec core: its power domain, then its clock. */
int
qcom_videocc_hw_enable(struct qcom_videocc *sc)
{
	const struct qcom_videocc_desc *d = sc->desc;
	int error;

	if (sc->hw_on)
		return (0);
	if (!sc->ctrl_on)
		return (ENXIO);
	if ((error = qcom_videocc_gdsc_enable(sc, d->hw_gdscr,
	    "core power")) != 0)
		return (error);
	if ((error = qcom_videocc_branch_enable(sc, false, d->hw_cbcr, true,
	    "core clock")) != 0) {
		qcom_videocc_gdsc_disable(sc, d->hw_gdscr);
		return (error);
	}
	sc->hw_on = true;
	return (0);
}

void
qcom_videocc_hw_disable(struct qcom_videocc *sc)
{
	const struct qcom_videocc_desc *d = sc->desc;

	if (!sc->hw_on)
		return;
	(void)qcom_videocc_hw_set_hwmode(sc, false);
	qcom_videocc_branch_disable(sc, false, d->hw_cbcr);
	qcom_videocc_gdsc_disable(sc, d->hw_gdscr);
	sc->hw_on = false;
}

/*
 * The core's power domain under the codec's control (its firmware powers
 * the core up and down), or back under ours, as Linux's gdsc_set_hwmode():
 * back under ours, it must be on.
 */
int
qcom_videocc_hw_set_hwmode(struct qcom_videocc *sc, bool hw)
{
	const struct qcom_videocc_desc *d = sc->desc;

	if (!sc->hw_on)
		return (ENXIO);
	qcom_videocc_set(sc, false, d->hw_gdscr, hw ? 0 : GDSC_HW_CONTROL,
	    hw ? GDSC_HW_CONTROL : 0);
	/* The controller takes a few cycles to see the change. */
	DELAY(1);
	if (hw)
		return (0);
	return (qcom_videocc_poll(sc, false, d->hw_gdscr, GDSC_PWR_ON,
	    GDSC_PWR_ON, "core power"));
}

static const struct qcom_videocc_desc *
qcom_videocc_find_soc(void)
{
	UINT32 id;
	u_int i;

	if (ACPI_FAILURE(acpi_GetInteger(ACPI_ROOT_OBJECT, "\\_SB.SOID", &id)))
		return (NULL);
	for (i = 0; i < nitems(qcom_videocc_socs); i++)
		if (qcom_videocc_socs[i].soc_id == id)
			return (&qcom_videocc_socs[i]);
	return (NULL);
}

struct qcom_videocc *
qcom_videocc_create(device_t dev)
{
	const struct qcom_videocc_desc *d;
	struct qcom_videocc *sc;

	if ((d = qcom_videocc_find_soc()) == NULL)
		return (NULL);
	sc = malloc(sizeof(*sc), M_QCOM_VIDEOCC, M_WAITOK | M_ZERO);
	sc->dev = dev;
	sc->desc = d;
	/* Firmware describes neither; map them by address. */
	sc->gcc = pmap_mapdev(d->gcc_base, d->gcc_size);
	sc->cc = pmap_mapdev(d->cc_base, d->cc_size);
	return (sc);
}

void
qcom_videocc_destroy(struct qcom_videocc *sc)
{
	if (sc == NULL)
		return;
	qcom_videocc_ctrl_disable(sc);
	pmap_unmapdev(__DEVOLATILE(void *, sc->gcc), sc->desc->gcc_size);
	pmap_unmapdev(__DEVOLATILE(void *, sc->cc), sc->desc->cc_size);
	free(sc, M_QCOM_VIDEOCC);
}

/*
 * By hand, for bring-up: power the codec, read its wrapper's version
 * (Venus's WRAPPER_HW_VERSION), and power it down again.  The state found
 * is printed under bootverbose (boot -v, or sysctl debug.bootverbose=1).
 */
#define	SC8280XP_CODEC_WRAPPER	0xaab0000	/* codec 0xaa00000 + 0xb0000 */

static uint32_t qcom_videocc_codec_version;
static struct sx qcom_videocc_test_lock;
SX_SYSINIT(qcom_videocc_test, &qcom_videocc_test_lock, "qcom_videocc test");

static int
qcom_videocc_test_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct qcom_videocc *sc;
	volatile uint32_t *w;
	int error, v;

	v = 0;
	error = sysctl_handle_int(oidp, &v, 0, req);
	if (error != 0 || req->newptr == NULL || v == 0)
		return (error);
	sx_xlock(&qcom_videocc_test_lock);
	if ((sc = qcom_videocc_create(NULL)) == NULL) {
		sx_xunlock(&qcom_videocc_test_lock);
		return (ENXIO);
	}
	if ((error = qcom_videocc_ctrl_enable(sc)) == 0 &&
	    (error = qcom_videocc_hw_enable(sc)) == 0) {
		w = pmap_mapdev(SC8280XP_CODEC_WRAPPER, PAGE_SIZE);
		qcom_videocc_codec_version = w[0];
		pmap_unmapdev(__DEVOLATILE(void *, w), PAGE_SIZE);
		VCC_PRINTF(sc, "codec wrapper version %#x (%u.%u)\n",
		    qcom_videocc_codec_version,
		    (qcom_videocc_codec_version >> 28) & 0x7,
		    (qcom_videocc_codec_version >> 16) & 0xfff);
	}
	qcom_videocc_destroy(sc);
	sx_xunlock(&qcom_videocc_test_lock);
	return (error);
}

SYSCTL_NODE(_hw, OID_AUTO, qcom_videocc, CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
    "Qualcomm video clocks");
SYSCTL_PROC(_hw_qcom_videocc, OID_AUTO, test,
    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE, NULL, 0,
    qcom_videocc_test_sysctl, "I",
    "Power the codec, read its version, power it down");
SYSCTL_UINT(_hw_qcom_videocc, OID_AUTO, codec_version, CTLFLAG_RD,
    &qcom_videocc_codec_version, 0, "The codec wrapper's version, as read");

MODULE_VERSION(qcom_videocc, 1);
MODULE_DEPEND(qcom_videocc, acpi, 1, 1, 1);
MODULE_DEPEND(qcom_videocc, qcom_rpmh, 1, 1, 1);
