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
 * the RX (playback) macro and opening its headphone paths, as Linux's
 * lpass-rx-macro and lpass-va-macro drivers do: the DSP's PRM powers them
 * and runs their clocks, and the VA macro generates the frame sync the RX
 * macro counts from.
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
#define	VA_MACRO_BASE		0x3370000
#define	MACRO_SIZE		0x1000
#define	MACRO_MCLK_HZ		19200000

#define	VA_MCLK_CONTROL		0x0000
#define	VA_MCLK_EN		0x01
#define	VA_FS_CNT_CONTROL	0x0004
#define	VA_FS_EN		0x01
#define	VA_FS_COUNTER_CLR	0x02
#define	VA_TOP_CFG0		0x0080
#define	VA_FS_BROADCAST_EN	0x02

#define	RX_MCLK_CONTROL		0x0100
#define	RX_MCLK_EN		0x01
#define	RX_MCLK2_EN		0x02
#define	RX_FS_CNT_CONTROL	0x0104
#define	RX_FS_CNT_EN		0x01
#define	RX_FS_CNT_CLR		0x02

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
static volatile uint32_t *va, *rx;
static bool macro_on;

static void
macro_set(volatile uint32_t *m, u_int off, uint32_t mask, uint32_t v)
{

	m[off / 4] = (m[off / 4] & ~mask) | v;
}

static void
macro_rx_down(u_int nclk, u_int nvote)
{

	while (nclk > 0)
		(void)qcom_prm_clock(macro_clocks[--nclk], 0);
	if (nvote > 1)
		(void)qcom_prm_hw_vote(QCOM_PRM_HW_DCODEC, false);
	if (nvote > 0)
		(void)qcom_prm_hw_vote(QCOM_PRM_HW_LPASS, false);
}

int
qcom_lpass_macro_rx(bool on)
{
	u_int i;
	int error;

	sx_xlock(&macro_lock);
	if (on == macro_on) {
		sx_xunlock(&macro_lock);
		return (0);
	}
	if (!on) {
		/* The macros' registers only answer while clocked. */
		for (i = nitems(rx_hph); i > 0; i--)
			rx[rx_hph[i - 1].off / 4] = rx_hph[i - 1].dflt;
		macro_set(rx, RX_FS_CNT_CONTROL, RX_FS_CNT_EN, 0);
		macro_set(rx, RX_FS_CNT_CONTROL, RX_FS_CNT_CLR, RX_FS_CNT_CLR);
		macro_set(rx, RX_MCLK_CONTROL, RX_MCLK_EN | RX_MCLK2_EN, 0);
		macro_set(va, VA_MCLK_CONTROL, VA_MCLK_EN, 0);
		macro_set(va, VA_FS_CNT_CONTROL, VA_FS_EN, 0);
		macro_set(va, VA_TOP_CFG0, VA_FS_BROADCAST_EN, 0);
		macro_rx_down(nitems(macro_clocks), 2);
		macro_on = false;
		sx_xunlock(&macro_lock);
		return (0);
	}

	/* Power first: until then, touching the macros hangs the bus. */
	error = qcom_prm_hw_vote(QCOM_PRM_HW_LPASS, true);
	if (error != 0) {
		sx_xunlock(&macro_lock);
		return (error);
	}
	error = qcom_prm_hw_vote(QCOM_PRM_HW_DCODEC, true);
	if (error != 0) {
		macro_rx_down(0, 1);
		sx_xunlock(&macro_lock);
		return (error);
	}
	for (i = 0; i < nitems(macro_clocks); i++) {
		error = qcom_prm_clock(macro_clocks[i], MACRO_MCLK_HZ);
		if (error != 0) {
			macro_rx_down(i, 2);
			sx_xunlock(&macro_lock);
			return (error);
		}
	}
	if (va == NULL) {
		va = pmap_mapdev(VA_MACRO_BASE, MACRO_SIZE);
		rx = pmap_mapdev(RX_MACRO_BASE, MACRO_SIZE);
	}

	/* The VA macro's frame sync, which it broadcasts to the others. */
	macro_set(va, VA_MCLK_CONTROL, VA_MCLK_EN, VA_MCLK_EN);
	macro_set(va, VA_FS_CNT_CONTROL, VA_FS_EN | VA_FS_COUNTER_CLR,
	    VA_FS_EN | VA_FS_COUNTER_CLR);
	macro_set(va, VA_FS_CNT_CONTROL, VA_FS_EN | VA_FS_COUNTER_CLR,
	    VA_FS_EN);
	macro_set(va, VA_TOP_CFG0, VA_FS_BROADCAST_EN, VA_FS_BROADCAST_EN);

	/* Then the RX macro's clocks and its frame counter. */
	macro_set(rx, RX_MCLK_CONTROL, RX_MCLK_EN | RX_MCLK2_EN,
	    RX_MCLK_EN | RX_MCLK2_EN);
	macro_set(rx, RX_FS_CNT_CONTROL, RX_FS_CNT_CLR, 0);
	macro_set(rx, RX_FS_CNT_CONTROL, RX_FS_CNT_EN, RX_FS_CNT_EN);
	for (i = 0; i < nitems(rx_hph); i++)
		rx[rx_hph[i].off / 4] = rx_hph[i].on;
	printf("qcom_lpass_macro: RX clocked: VA mclk %#x fs %#x top %#x, "
	    "RX mclk %#x fs %#x, paths %#x %#x\n", va[VA_MCLK_CONTROL / 4],
	    va[VA_FS_CNT_CONTROL / 4], va[VA_TOP_CFG0 / 4],
	    rx[RX_MCLK_CONTROL / 4], rx[RX_FS_CNT_CONTROL / 4],
	    rx[0x400 / 4], rx[0x480 / 4]);
	macro_on = true;
	sx_xunlock(&macro_lock);
	return (0);
}
