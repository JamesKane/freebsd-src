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
 * Qualcomm GPU clock controller, CX side; see qcom_gpucc.h.
 *
 * Under ACPI the firmware hides clocks and power domains behind the Windows
 * power engine plug-in, so they are programmed here directly.  Only fixed
 * rates are needed: the GMU runs at 200 MHz, and the GMU firmware scales the
 * GPU core clock, whose PLL is in the GX domain.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/rman.h>

#include <machine/bus.h>

#include <dev/qcom_gpucc/qcom_gpucc.h>

/* Branch clock control register (CBCR). */
#define	CBCR_CLK_OFF		(1u << 31)
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
#define	GDSC_SW_COLLAPSE	(1u << 0)

#define	POLL_US			2000

struct qcom_gpucc_branch {
	const char	*name;
	bool		gcc;		/* in GCC rather than the GPU CC */
	uint32_t	reg;
	bool		halt_check;	/* CLK_OFF reflects the clock */
};

struct qcom_gpucc_rcg {
	const char	*name;
	uint32_t	cmd;
	uint32_t	cfg;
};

struct qcom_gpucc_desc {
	/* GCC window, and the GPLL0 outputs voted on for the GPU CC. */
	bus_addr_t	gcc_base;
	bus_size_t	gcc_size;
	uint32_t	gcc_gpll0_vote;
	uint32_t	gcc_gpll0_mask;
	/* GCC clocks needed before the CX domain is powered. */
	const struct qcom_gpucc_branch *pre;
	u_int		npre;
	/* The CX power domain. */
	uint32_t	cx_gdscr;
	uint32_t	cx_gds_hw_ctrl;
	const struct qcom_gpucc_rcg *rcgs;
	u_int		nrcgs;
	/* Clocks inside the CX domain, enabled in order. */
	const struct qcom_gpucc_branch *clks;
	u_int		nclks;
};

/* The GCC offsets below are relative to gcc_base. */
#define	SC8280XP_GCC_BASE	0x152000

static const struct qcom_gpucc_branch sc8280xp_pre[] = {
	{ "gcc_gpu_cfg_ahb",	true,	0x71004 - 0x52000, true },
	{ "gcc_ddrss_gpu_axi",	true,	0x7115c - 0x52000, false },
};

static const struct qcom_gpucc_rcg sc8280xp_rcgs[] = {
	/* 200 MHz: GPLL0 / 2 (source 6), divided by 1.5. */
	{ "gmu_clk_src", 0x1120, RCG_CFG_SRC_SEL(6) | RCG_CFG_SRC_DIV(3) },
	/* 200 MHz: GPLL0 (source 5), divided by 3. */
	{ "hub_clk_src", 0x117c, RCG_CFG_SRC_SEL(5) | RCG_CFG_SRC_DIV(6) },
};

static const struct qcom_gpucc_branch sc8280xp_clks[] = {
	{ "gcc_gpu_memnoc_gfx",	true,	0x71010 - 0x52000, true },
	{ "gpu_cc_cb",		false,	0x1170, false },
	{ "gpu_cc_cxo",		false,	0x109c, true },
	{ "gpu_cc_ahb",		false,	0x1078, true },
	{ "gpu_cc_cx_gmu",	false,	0x1098, true },
	{ "gpu_cc_hub_cx_int",	false,	0x1204, true },
	{ "gpu_cc_hlos1_vote_gpu_smmu", false, 0x5000, false },
};

static const struct qcom_gpucc_desc sc8280xp_desc = {
	.gcc_base = SC8280XP_GCC_BASE,
	.gcc_size = 0x20000,
	.gcc_gpll0_vote = 0,
	.gcc_gpll0_mask = (1u << 15) | (1u << 16),
	.pre = sc8280xp_pre,
	.npre = nitems(sc8280xp_pre),
	.cx_gdscr = 0x106c,
	.cx_gds_hw_ctrl = 0x1540,
	.rcgs = sc8280xp_rcgs,
	.nrcgs = nitems(sc8280xp_rcgs),
	.clks = sc8280xp_clks,
	.nclks = nitems(sc8280xp_clks),
};

struct qcom_gpucc {
	device_t			dev;
	const struct qcom_gpucc_desc	*desc;
	struct resource			*gpucc;
	bus_size_t			gpucc_off;
	struct resource			*gcc;
	int				gcc_rid;
	bool				cx_on;
};

static MALLOC_DEFINE(M_QCOM_GPUCC, "qcom_gpucc", "Qualcomm GPU clocks");

/* Register access; "gcc" selects the GCC window over the GPU CC. */
static uint32_t
qcom_gpucc_read(struct qcom_gpucc *sc, bool gcc, uint32_t reg)
{
	if (gcc)
		return (bus_read_4(sc->gcc, reg));
	return (bus_read_4(sc->gpucc, sc->gpucc_off + reg));
}

static void
qcom_gpucc_write(struct qcom_gpucc *sc, bool gcc, uint32_t reg, uint32_t v)
{
	if (gcc)
		bus_write_4(sc->gcc, reg, v);
	else
		bus_write_4(sc->gpucc, sc->gpucc_off + reg, v);
}

static void
qcom_gpucc_set(struct qcom_gpucc *sc, bool gcc, uint32_t reg, uint32_t clr,
    uint32_t set)
{
	qcom_gpucc_write(sc, gcc, reg,
	    (qcom_gpucc_read(sc, gcc, reg) & ~clr) | set);
}

static int
qcom_gpucc_poll(struct qcom_gpucc *sc, bool gcc, uint32_t reg, uint32_t mask,
    uint32_t want, const char *what)
{
	int i;

	for (i = 0; i < POLL_US; i++) {
		if ((qcom_gpucc_read(sc, gcc, reg) & mask) == want)
			return (0);
		DELAY(1);
	}
	device_printf(sc->dev, "%s timed out (%#x = %#x)\n", what, reg,
	    qcom_gpucc_read(sc, gcc, reg));
	return (ETIMEDOUT);
}

static int
qcom_gpucc_branch_enable(struct qcom_gpucc *sc,
    const struct qcom_gpucc_branch *b)
{
	qcom_gpucc_set(sc, b->gcc, b->reg, 0, CBCR_CLK_ENABLE);
	if (!b->halt_check)
		return (0);
	return (qcom_gpucc_poll(sc, b->gcc, b->reg, CBCR_CLK_OFF, 0, b->name));
}

static void
qcom_gpucc_branch_disable(struct qcom_gpucc *sc,
    const struct qcom_gpucc_branch *b)
{
	qcom_gpucc_set(sc, b->gcc, b->reg, CBCR_CLK_ENABLE, 0);
}

static int
qcom_gpucc_rcg_set(struct qcom_gpucc *sc, const struct qcom_gpucc_rcg *rcg)
{
	qcom_gpucc_set(sc, false, RCG_CFG(rcg->cmd), RCG_CFG_MASK, rcg->cfg);
	qcom_gpucc_set(sc, false, rcg->cmd, 0, RCG_CMD_UPDATE);
	return (qcom_gpucc_poll(sc, false, rcg->cmd, RCG_CMD_UPDATE, 0,
	    rcg->name));
}

static int
qcom_gpucc_gdsc_enable(struct qcom_gpucc *sc)
{
	const struct qcom_gpucc_desc *d = sc->desc;
	int error;

	qcom_gpucc_set(sc, false, d->cx_gdscr, GDSC_WAIT_MASK,
	    GDSC_WAIT(0x2, 0x8, 0x2));
	qcom_gpucc_set(sc, false, d->cx_gdscr, GDSC_SW_COLLAPSE, 0);
	/* The hardware controller takes a few XO cycles to update status. */
	DELAY(1);
	error = qcom_gpucc_poll(sc, false, d->cx_gds_hw_ctrl, GDSC_PWR_ON,
	    GDSC_PWR_ON, "cx_gdsc");
	if (error != 0)
		return (error);
	/* Clocks must not be enabled within 400 ns of powering the memories. */
	DELAY(1);
	qcom_gpucc_set(sc, false, d->cx_gdscr, 0, GDSC_RETAIN_FF);
	return (0);
}

static void
qcom_gpucc_gdsc_disable(struct qcom_gpucc *sc)
{
	const struct qcom_gpucc_desc *d = sc->desc;

	qcom_gpucc_set(sc, false, d->cx_gdscr, GDSC_RETAIN_FF,
	    GDSC_SW_COLLAPSE);
	/*
	 * The domain is voted: it stays up while other masters (the GMU) hold
	 * it, so don't wait for it to go down.  Linux waits this long so that
	 * an immediate re-enable finds it in a known state.
	 */
	DELAY(500);
}

int
qcom_gpucc_cx_enable(struct qcom_gpucc *sc)
{
	const struct qcom_gpucc_desc *d = sc->desc;
	u_int i;
	int error;

	if (sc->cx_on)
		return (0);
	qcom_gpucc_set(sc, true, d->gcc_gpll0_vote, 0, d->gcc_gpll0_mask);
	for (i = 0; i < d->npre; i++)
		if ((error = qcom_gpucc_branch_enable(sc, &d->pre[i])) != 0)
			goto fail_pre;
	if ((error = qcom_gpucc_gdsc_enable(sc)) != 0)
		goto fail_pre;
	for (i = 0; i < d->nrcgs; i++)
		if ((error = qcom_gpucc_rcg_set(sc, &d->rcgs[i])) != 0)
			goto fail_gdsc;
	for (i = 0; i < d->nclks; i++)
		if ((error = qcom_gpucc_branch_enable(sc, &d->clks[i])) != 0)
			goto fail_clks;
	sc->cx_on = true;
	return (0);

fail_clks:
	while (i-- > 0)
		qcom_gpucc_branch_disable(sc, &d->clks[i]);
fail_gdsc:
	qcom_gpucc_gdsc_disable(sc);
	i = d->npre;
fail_pre:
	while (i-- > 0)
		qcom_gpucc_branch_disable(sc, &d->pre[i]);
	qcom_gpucc_set(sc, true, d->gcc_gpll0_vote, d->gcc_gpll0_mask, 0);
	return (error);
}

void
qcom_gpucc_cx_disable(struct qcom_gpucc *sc)
{
	const struct qcom_gpucc_desc *d = sc->desc;
	u_int i;

	if (!sc->cx_on)
		return;
	for (i = d->nclks; i-- > 0;)
		qcom_gpucc_branch_disable(sc, &d->clks[i]);
	qcom_gpucc_gdsc_disable(sc);
	for (i = d->npre; i-- > 0;)
		qcom_gpucc_branch_disable(sc, &d->pre[i]);
	qcom_gpucc_set(sc, true, d->gcc_gpll0_vote, d->gcc_gpll0_mask, 0);
	sc->cx_on = false;
}

struct qcom_gpucc *
qcom_gpucc_create(device_t dev, struct resource *res, bus_size_t offset)
{
	const struct qcom_gpucc_desc *d = &sc8280xp_desc;
	struct qcom_gpucc *sc;

	sc = malloc(sizeof(*sc), M_QCOM_GPUCC, M_WAITOK | M_ZERO);
	sc->dev = dev;
	sc->desc = d;
	sc->gpucc = res;
	sc->gpucc_off = offset;
	/* Firmware doesn't describe the GCC; allocate its window by address. */
	sc->gcc_rid = 0x7fff;
	sc->gcc = bus_alloc_resource(dev, SYS_RES_MEMORY, &sc->gcc_rid,
	    d->gcc_base, d->gcc_base + d->gcc_size - 1, d->gcc_size,
	    RF_ACTIVE);
	if (sc->gcc == NULL) {
		device_printf(dev, "cannot map GCC registers\n");
		free(sc, M_QCOM_GPUCC);
		return (NULL);
	}
	return (sc);
}

void
qcom_gpucc_destroy(struct qcom_gpucc *sc)
{
	if (sc == NULL)
		return;
	qcom_gpucc_cx_disable(sc);
	bus_release_resource(sc->dev, SYS_RES_MEMORY, sc->gcc_rid, sc->gcc);
	free(sc, M_QCOM_GPUCC);
}

MODULE_VERSION(qcom_gpucc, 1);
