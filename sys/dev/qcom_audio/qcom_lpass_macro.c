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
 * The LPASS codec macros: the digital half of the codec, between the DSP's
 * codec DMA and the SoundWire links to the analog codecs.  Here, clocking
 * the RX (playback) macro and opening its headphone paths, and clocking the
 * RX and TX SoundWire controllers, as Linux's lpass-rx/tx/va-macro drivers
 * do: the DSP's PRM powers the macros and runs their clocks, and the VA
 * macro generates the frame sync the others count from.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/sx.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/bus.h>

#include <dev/qcom_audio/qcom_lpass_macro.h>
#include <dev/qcom_audio/qcom_prm.h>

/* SC8280XP */
#define	RX_MACRO_BASE		0x3200000
#define	TX_MACRO_BASE		0x3220000
#define	VA_MACRO_BASE		0x3370000
#define	MACRO_SIZE		0x1000
#define	MACRO_MCLK_HZ		19200000

#define	VA_MCLK_CONTROL		0x0000
#define	VA_MCLK_EN		0x01
#define	VA_FS_CNT_CONTROL	0x0004
#define	VA_FS_EN		0x01
#define	VA_FS_COUNTER_CLR	0x02
#define	VA_SWR_CONTROL		0x0008
#define	VA_SWR_CLK_EN		0x01
#define	VA_SWR_RESET		0x02
#define	VA_TOP_CFG0		0x0080
#define	VA_FS_BROADCAST_EN	0x02

#define	TX_MCLK_CONTROL		0x0000
#define	TX_MCLK_EN		0x01
#define	TX_FS_CNT_CONTROL	0x0004
#define	TX_FS_CNT_EN		0x01
#define	TX_SWR_CONTROL		0x0008
#define	TX_SWR_CLK_EN		0x01
#define	TX_SWR_RESET		0x02
#define	TX_TOP_FREQ_MCLK	0x0090
#define	TX_FREQ_MCLK_9P6	0x01

#define	RX_MCLK_CONTROL		0x0100
#define	RX_MCLK_EN		0x01
#define	RX_MCLK2_EN		0x02
#define	RX_FS_CNT_CONTROL	0x0104
#define	RX_FS_CNT_EN		0x01
#define	RX_FS_CNT_CLR		0x02
#define	RX_SWR_CONTROL		0x0108
#define	RX_SWR_CLK_EN		0x01
#define	RX_SWR_RESET		0x02

/*
 * The headphone paths, interpolators 0 and 1 (left and right), as Linux
 * leaves the registers while playing to them: what DAPM sets on the way
 * up, from the input muxes to each path's clock last.  Each with its
 * reset value, which putting the path down restores.
 */
static const struct macro_reg {
	uint16_t	off;
	uint8_t		on;
	uint8_t		dflt;
} rx_hph[] = {
	{ 0x028, 0x80, 0x00 },	/* TOP_HPHL_COMP_LUT */
	{ 0x03c, 0x80, 0x00 },	/* TOP_HPHR_COMP_LUT */
	{ 0x07c, 0x08, 0x00 },	/* TOP_DSD0_DEBUG_CFG3 */
	{ 0x08c, 0x08, 0x00 },	/* TOP_DSD1_DEBUG_CFG3 */
	{ 0x200, 0x01, 0x00 },	/* CLSH_CRC */
	{ 0x208, 0x00, 0x02 },	/* CLSH_DECAY_CTRL */
	{ 0x21c, 0x00, 0x01 },	/* CLSH_K1_MSB */
	{ 0x220, 0xc0, 0x00 },	/* CLSH_K1_LSB */
	{ 0x81c, 0x08, 0x28 },	/* COMPANDER0_CTL7 */
	{ 0x85c, 0x08, 0x28 },	/* COMPANDER1_CTL7 */
	{ 0xa2c, 0x50, 0x00 },	/* SIDETONE_IIR0_IIR_COEF_B1_CTL */
	{ 0xaac, 0x50, 0x00 },	/* SIDETONE_IIR1_IIR_COEF_B1_CTL */
	{ 0x180, 0x05, 0x00 },	/* INP_MUX_RX_INT0_CFG0: RX0 */
	{ 0x188, 0x06, 0x00 },	/* INP_MUX_RX_INT1_CFG0: RX1 */
	{ 0x404, 0x4c, 0x00 },	/* RX0_RX_PATH_CFG0 */
	{ 0x408, 0x65, 0x64 },	/* RX0_RX_PATH_CFG1 */
	{ 0x410, 0x03, 0x00 },	/* RX0_RX_PATH_CFG3 */
	{ 0x424, 0x0c, 0x08 },	/* RX0_RX_PATH_SEC1 */
	{ 0x42c, 0x14, 0x00 },	/* RX0_RX_PATH_SEC3 */
	{ 0x434, 0x02, 0x00 },	/* RX0_RX_PATH_SEC7 */
	{ 0x440, 0x09, 0x08 },	/* RX0_RX_PATH_DSM_CTL */
	{ 0x484, 0x4c, 0x00 },	/* RX1_RX_PATH_CFG0 */
	{ 0x488, 0x65, 0x64 },	/* RX1_RX_PATH_CFG1 */
	{ 0x490, 0x03, 0x00 },	/* RX1_RX_PATH_CFG3 */
	{ 0x4a4, 0x0c, 0x08 },	/* RX1_RX_PATH_SEC1 */
	{ 0x4ac, 0x14, 0x00 },	/* RX1_RX_PATH_SEC3 */
	{ 0x4b4, 0x02, 0x00 },	/* RX1_RX_PATH_SEC7 */
	{ 0x4c0, 0x09, 0x08 },	/* RX1_RX_PATH_DSM_CTL */
	{ 0x400, 0x24, 0x04 },	/* RX0_RX_PATH_CTL: clock on, 48 kHz */
	{ 0x480, 0x24, 0x04 },	/* RX1_RX_PATH_CTL */
};

static const uint32_t macro_clocks[] = {
	QCOM_PRM_CLK_TX_CORE_MCLK,	/* the VA macro's */
	QCOM_PRM_CLK_TX_CORE_NPL_MCLK,
	QCOM_PRM_CLK_RX_CORE_TX_MCLK,	/* the RX macro's */
	QCOM_PRM_CLK_RX_CORE_TX_2X_MCLK,
};

static struct sx macro_lock;
SX_SYSINIT(qcom_lpass_macro, &macro_lock, "qcom_lpass_macro");
static volatile uint32_t *va, *rx, *tx;
static bool macro_voted;	/* the LPASS core and digital codec held */
static bool macro_clocked;
static bool macro_swr_reset;	/* the SoundWire controllers started */
static bool macro_hph_on;

static void
macro_set(volatile uint32_t *m, u_int off, uint32_t mask, uint32_t v)
{

	m[off / 4] = (m[off / 4] & ~mask) | v;
}

static void
macro_clocks_off(u_int nclk)
{

	while (nclk > 0)
		(void)qcom_prm_clock(macro_clocks[--nclk], 0);
}

/*
 * The LPASS core and digital codec votes, taken the first time and then
 * held, as Linux's LPASS pin driver holds them: the macros' state and the
 * SoundWire pins, and so the codec's wake-up, live on while the clocks are
 * off.
 */
static int
macro_vote(void)
{
	int error;

	if (macro_voted)
		return (0);
	error = qcom_prm_hw_vote(QCOM_PRM_HW_LPASS, true);
	if (error != 0)
		return (error);
	error = qcom_prm_hw_vote(QCOM_PRM_HW_DCODEC, true);
	if (error != 0) {
		(void)qcom_prm_hw_vote(QCOM_PRM_HW_LPASS, false);
		return (error);
	}
	if (va == NULL) {
		va = pmap_mapdev(VA_MACRO_BASE, MACRO_SIZE);
		rx = pmap_mapdev(RX_MACRO_BASE, MACRO_SIZE);
		tx = pmap_mapdev(TX_MACRO_BASE, MACRO_SIZE);
	}
	macro_voted = true;
	return (0);
}

/*
 * The macros' clocks: the codec clocks from the PRM, the VA macro's frame
 * sync, the RX and TX macros' clocks and frame counters, and the clocks of
 * the SoundWire controllers behind them.  The macros' registers only answer
 * while these run.
 */
int
qcom_lpass_macro_clocks(bool on)
{
	u_int i;
	int error;

	sx_xlock(&macro_lock);
	if (on == macro_clocked) {
		sx_xunlock(&macro_lock);
		return (0);
	}
	if (!on) {
		KASSERT(!macro_hph_on, ("qcom_lpass_macro: paths still on"));
		macro_set(rx, RX_SWR_CONTROL, RX_SWR_CLK_EN, 0);
		macro_set(tx, TX_SWR_CONTROL, TX_SWR_CLK_EN, 0);
		macro_set(va, VA_SWR_CONTROL, VA_SWR_CLK_EN, 0);
		macro_set(tx, TX_FS_CNT_CONTROL, TX_FS_CNT_EN, 0);
		macro_set(tx, TX_MCLK_CONTROL, TX_MCLK_EN, 0);
		macro_set(rx, RX_FS_CNT_CONTROL, RX_FS_CNT_EN, 0);
		macro_set(rx, RX_FS_CNT_CONTROL, RX_FS_CNT_CLR, RX_FS_CNT_CLR);
		macro_set(rx, RX_MCLK_CONTROL, RX_MCLK_EN | RX_MCLK2_EN, 0);
		macro_set(va, VA_MCLK_CONTROL, VA_MCLK_EN, 0);
		macro_set(va, VA_FS_CNT_CONTROL, VA_FS_EN, 0);
		macro_set(va, VA_TOP_CFG0, VA_FS_BROADCAST_EN, 0);
		macro_clocks_off(nitems(macro_clocks));
		macro_clocked = false;
		sx_xunlock(&macro_lock);
		return (0);
	}

	/* Power first: until then, touching the macros hangs the bus. */
	error = macro_vote();
	if (error != 0) {
		sx_xunlock(&macro_lock);
		return (error);
	}
	for (i = 0; i < nitems(macro_clocks); i++) {
		error = qcom_prm_clock(macro_clocks[i], MACRO_MCLK_HZ);
		if (error != 0) {
			macro_clocks_off(i);
			sx_xunlock(&macro_lock);
			return (error);
		}
	}

	/* The VA macro's frame sync, which it broadcasts to the others. */
	macro_set(va, VA_MCLK_CONTROL, VA_MCLK_EN, VA_MCLK_EN);
	macro_set(va, VA_FS_CNT_CONTROL, VA_FS_EN | VA_FS_COUNTER_CLR,
	    VA_FS_EN | VA_FS_COUNTER_CLR);
	macro_set(va, VA_FS_CNT_CONTROL, VA_FS_EN | VA_FS_COUNTER_CLR,
	    VA_FS_EN);
	macro_set(va, VA_TOP_CFG0, VA_FS_BROADCAST_EN, VA_FS_BROADCAST_EN);

	/* The RX and TX macros' clocks and frame counters. */
	macro_set(rx, RX_MCLK_CONTROL, RX_MCLK_EN | RX_MCLK2_EN,
	    RX_MCLK_EN | RX_MCLK2_EN);
	macro_set(rx, RX_FS_CNT_CONTROL, RX_FS_CNT_CLR, 0);
	macro_set(rx, RX_FS_CNT_CONTROL, RX_FS_CNT_EN, RX_FS_CNT_EN);
	macro_set(tx, TX_TOP_FREQ_MCLK, TX_FREQ_MCLK_9P6, TX_FREQ_MCLK_9P6);
	macro_set(tx, TX_MCLK_CONTROL, TX_MCLK_EN, TX_MCLK_EN);
	macro_set(tx, TX_FS_CNT_CONTROL, TX_FS_CNT_EN, TX_FS_CNT_EN);

	/*
	 * The SoundWire controllers' clocks: the RX macro's for the RX link,
	 * the TX and VA macros' for the TX link.  The first time, each
	 * starts with its controller held in reset, as Linux's macro probes
	 * do; after that, as Linux's clock gates do, it just runs again, and
	 * the links resume where they stopped.
	 */
	if (!macro_swr_reset) {
		macro_set(rx, RX_SWR_CONTROL, RX_SWR_RESET, RX_SWR_RESET);
		macro_set(tx, TX_SWR_CONTROL, TX_SWR_RESET, TX_SWR_RESET);
		macro_set(va, VA_SWR_CONTROL, VA_SWR_RESET, VA_SWR_RESET);
	}
	macro_set(rx, RX_SWR_CONTROL, RX_SWR_CLK_EN, RX_SWR_CLK_EN);
	macro_set(tx, TX_SWR_CONTROL, TX_SWR_CLK_EN, TX_SWR_CLK_EN);
	macro_set(va, VA_SWR_CONTROL, VA_SWR_CLK_EN, VA_SWR_CLK_EN);
	if (!macro_swr_reset) {
		macro_set(rx, RX_SWR_CONTROL, RX_SWR_RESET, 0);
		macro_set(tx, TX_SWR_CONTROL, TX_SWR_RESET, 0);
		macro_set(va, VA_SWR_CONTROL, VA_SWR_RESET, 0);
		macro_swr_reset = true;
	}
	macro_clocked = true;
	sx_xunlock(&macro_lock);
	return (0);
}

/* The RX macro's headphone paths, which need the clocks. */
int
qcom_lpass_macro_hph(bool on)
{
	u_int i;

	sx_xlock(&macro_lock);
	if (!macro_clocked) {
		sx_xunlock(&macro_lock);
		return (ENXIO);
	}
	if (on != macro_hph_on) {
		if (on)
			for (i = 0; i < nitems(rx_hph); i++)
				rx[rx_hph[i].off / 4] = rx_hph[i].on;
		else
			for (i = nitems(rx_hph); i > 0; i--)
				rx[rx_hph[i - 1].off / 4] = rx_hph[i - 1].dflt;
		macro_hph_on = on;
	}
	sx_xunlock(&macro_lock);
	return (0);
}

/* Give everything up, for unloading. */
void
qcom_lpass_macro_release(void)
{

	(void)qcom_lpass_macro_hph(false);
	(void)qcom_lpass_macro_clocks(false);
	sx_xlock(&macro_lock);
	if (macro_voted) {
		(void)qcom_prm_hw_vote(QCOM_PRM_HW_DCODEC, false);
		(void)qcom_prm_hw_vote(QCOM_PRM_HW_LPASS, false);
		macro_voted = false;
		macro_swr_reset = false;
	}
	sx_xunlock(&macro_lock);
}
