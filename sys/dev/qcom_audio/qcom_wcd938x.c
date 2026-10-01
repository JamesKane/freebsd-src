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
 * The WCD938x codec (the WCD9385 here): out of reset, its settings, and
 * its headphone output up and down, with the RX SoundWire link's ports set
 * up to carry the headphone samples to it.
 *
 * The register sequences are Linux's, as its wcd938x and SoundWire drivers
 * issue them for a 48 kHz stereo stream to the headphones: captured from
 * the controllers' command FIFOs while Linux played, in order, with the
 * pauses longer than 150 us that it made between them.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/gpio.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/sx.h>

#include <dev/qcom_audio/qcom_lpass_macro.h>
#include <dev/qcom_audio/qcom_swr.h>
#include <dev/qcom_audio/qcom_wcd938x.h>

#include "gpio_if.h"

/* Radxa Dragon Q8B: the codec's reset, active low. */
#define	WCD_RESET_PIN		106
/* Both halves enumerate as device 1 on their links. */
#define	WCD_DEV			1

struct wcd_op {
	uint8_t		op;
	uint16_t	reg;
	uint32_t	val;
};

enum {
	OP_CODEC,	/* a codec register, through the TX link */
	OP_RXDEV,	/* the codec's RX device's SoundWire registers */
	OP_RXMMIO,	/* the RX controller's registers */
	OP_BANK_SWITCH,	/* the RX link's frame control, broadcast */
	OP_DELAY,	/* microseconds */
};

#define	SEQ_CODEC(r, v)		{ OP_CODEC, (r), (v) }
#define	SEQ_RXDEV(r, v)		{ OP_RXDEV, (r), (v) }
#define	SEQ_RXMMIO(r, v)	{ OP_RXMMIO, (r), (v) }
#define	SEQ_BANK_SWITCH(r)	{ OP_BANK_SWITCH, (r), 0 }
#define	SEQ_DELAY(us)		{ OP_DELAY, 0, (us) }

/* The codec's settings, as Linux restores them on resume. */
static const struct wcd_op wcd_init[] = {
	SEQ_CODEC(0x3001, 0x80),	/* ANA_BIAS */
	SEQ_CODEC(0x3008, 0x02),	/* ANA_RX_SUPPLIES */
	SEQ_CODEC(0x3009, 0x00),	/* ANA_HPH */
	SEQ_CODEC(0x300f, 0x0a),	/* ANA_TX_CH2 */
	SEQ_CODEC(0x3014, 0x8d),	/* ANA_MBHC_MECH */
	SEQ_CODEC(0x3015, 0xb9),	/* ANA_MBHC_ELECT */
	SEQ_CODEC(0x301a, 0x18),	/* ANA_MBHC_BTN0 */
	SEQ_CODEC(0x301b, 0x30),	/* ANA_MBHC_BTN1 */
	SEQ_CODEC(0x301c, 0x48),	/* ANA_MBHC_BTN2 */
	SEQ_CODEC(0x301d, 0xa0),	/* ANA_MBHC_BTN3 */
	SEQ_CODEC(0x301e, 0xa0),	/* ANA_MBHC_BTN4 */
	SEQ_CODEC(0x301f, 0xa0),	/* ANA_MBHC_BTN5 */
	SEQ_CODEC(0x3020, 0xa0),	/* ANA_MBHC_BTN6 */
	SEQ_CODEC(0x3021, 0xa0),	/* ANA_MBHC_BTN7 */
	SEQ_CODEC(0x3029, 0x85),	/* BIAS_VBG_FINE_ADJ */
	SEQ_CODEC(0x306b, 0xfa),	/* MICB1_TEST_CTL_1 */
	SEQ_CODEC(0x306e, 0xfa),	/* MICB2_TEST_CTL_1 */
	SEQ_DELAY(517),
	SEQ_CODEC(0x3071, 0xfa),	/* MICB3_TEST_CTL_1 */
	SEQ_CODEC(0x3074, 0xfa),	/* MICB4_TEST_CTL_1 */
	SEQ_CODEC(0x308f, 0xfa),	/* TX_3_4_TEST_BLK_EN2 */
	SEQ_CODEC(0x3099, 0x02),	/* CLASSH_MODE_3 */
	SEQ_CODEC(0x30a5, 0xeb),	/* FLYBACK_VNEG_CTRL_1 */
	SEQ_CODEC(0x30d3, 0xaf),	/* HPH_L_EN */
	SEQ_CODEC(0x30d6, 0xaf),	/* HPH_R_EN */
	SEQ_CODEC(0x30d9, 0x19),	/* HPH_RDAC_CLK_CTL1 */
	SEQ_CODEC(0x30dd, 0xa8),	/* HPH_REFBUFF_UHQA_CTL */
	SEQ_CODEC(0x30e2, 0xd9),	/* HPH_SURGE_HPHLR_SURGE_EN */
	SEQ_CODEC(0x3103, 0xde),	/* SLEEP_CTL */
	SEQ_CODEC(0x3120, 0x82),	/* MBHC_NEW_CTL_1 */
	SEQ_CODEC(0x3121, 0x06),	/* MBHC_NEW_CTL_2 */
	SEQ_DELAY(515),
	SEQ_CODEC(0x3122, 0xe6),	/* MBHC_NEW_PLUG_DETECT_CTL */
	SEQ_CODEC(0x3128, 0x10),	/* AUX_AUXPA */
	SEQ_CODEC(0x312a, 0x00),	/* LDORXTX_CONFIG */
	SEQ_CODEC(0x3132, 0x00),	/* HPH_NEW_INT_RDAC_GAIN_CTL */
	SEQ_CODEC(0x3133, 0x82),	/* HPH_NEW_INT_RDAC_HD2_CTL_L */
	SEQ_CODEC(0x3140, 0x95),	/* HPH_NEW_INT_RDAC_HD2_CTL_L_NEW */
	SEQ_CODEC(0x3141, 0x95),	/* HPH_NEW_INT_RDAC_HD2_CTL_R_NEW */
	SEQ_CODEC(0x31b1, 0x08),	/* MBHC_NEW_INT_MECH_DET_CURRENT */
	SEQ_CODEC(0x31df, 0x08),	/* TX_COM_NEW_INT_TXFE_ICTRL_STG2MAIN_ULP */
	SEQ_CODEC(0x31e1, 0x14),	/* TX_COM_NEW_INT_TXFE_ICTRL_STG2CASC_ULP */
	SEQ_CODEC(0x3408, 0x10),	/* DIGITAL_CDC_ANA_CLK_CTL */
	SEQ_CODEC(0x3409, 0xf3),	/* DIGITAL_CDC_DIG_CLK_CTL */
	SEQ_CODEC(0x340d, 0xbc),	/* DIGITAL_CDC_RX0_CTL */
	SEQ_DELAY(516),
	SEQ_CODEC(0x340e, 0xbc),	/* DIGITAL_CDC_RX1_CTL */
	SEQ_CODEC(0x340f, 0xbc),	/* DIGITAL_CDC_RX2_CTL */
	SEQ_CODEC(0x3417, 0x1f),	/* DIGITAL_CDC_ANA_TX_CLK_CTL */
	SEQ_CODEC(0x344e, 0x0c),	/* DIGITAL_CDC_HPH_GAIN_CTL */
	SEQ_CODEC(0x345b, 0x06),	/* DIGITAL_CDC_DMIC_CTL */
	SEQ_CODEC(0x346b, 0x4c),	/* DIGITAL_INTR_MASK_0 */
	SEQ_CODEC(0x34d0, 0x55),	/* DIGITAL_TX_REQ_FB_CTL_0 */
	SEQ_CODEC(0x34d1, 0x44),	/* DIGITAL_TX_REQ_FB_CTL_1 */
	SEQ_CODEC(0x34d2, 0x11),	/* DIGITAL_TX_REQ_FB_CTL_2 */
	SEQ_CODEC(0x34d3, 0x00),	/* DIGITAL_TX_REQ_FB_CTL_3 */
	SEQ_CODEC(0x34d4, 0x00),	/* DIGITAL_TX_REQ_FB_CTL_4 */
};

/* The RX link carrying the headphone ports: both banks set up, channels on. */
static const struct wcd_op wcd_stream_on[] = {
	SEQ_RXDEV(0x0102, 0x00),	/* DP1_PORTCTRL */
	SEQ_RXDEV(0x0103, 0x01),	/* DP1_BLOCKCTRL1 */
	SEQ_RXDEV(0x0132, 0x03),	/* DP1_SAMPLECTRL1_B1 */
	SEQ_RXDEV(0x0134, 0x00),	/* DP1_OFFSETCTRL1_B1 */
	SEQ_RXDEV(0x0138, 0x01),	/* DP1_LANECTRL_B1 */
	SEQ_DELAY(370),
	SEQ_RXDEV(0x0202, 0x00),	/* DP2_PORTCTRL */
	SEQ_RXDEV(0x0203, 0x07),	/* DP2_BLOCKCTRL1 */
	SEQ_RXDEV(0x0232, 0x1f),	/* DP2_SAMPLECTRL1_B1 */
	SEQ_RXDEV(0x0234, 0x00),	/* DP2_OFFSETCTRL1_B1 */
	SEQ_RXDEV(0x0238, 0x00),	/* DP2_LANECTRL_B1 */
	SEQ_DELAY(370),
	SEQ_RXDEV(0x0402, 0x00),	/* DP4_PORTCTRL */
	SEQ_RXDEV(0x0403, 0xff),	/* DP4_BLOCKCTRL1 */
	SEQ_RXDEV(0x0432, 0x07),	/* DP4_SAMPLECTRL1_B1 */
	SEQ_RXDEV(0x0434, 0x01),	/* DP4_OFFSETCTRL1_B1 */
	SEQ_RXDEV(0x0438, 0x00),	/* DP4_LANECTRL_B1 */
	SEQ_RXMMIO(0x1164, 0x3),
	SEQ_RXMMIO(0x1168, 0x1),
	SEQ_RXMMIO(0x1174, 0xf0),
	SEQ_RXMMIO(0x112c, 0x1),
	SEQ_RXMMIO(0x1264, 0x1f),
	SEQ_RXMMIO(0x1268, 0x0),
	SEQ_RXMMIO(0x1274, 0x63),
	SEQ_RXMMIO(0x122c, 0x7),
	SEQ_RXMMIO(0x1464, 0x107),
	SEQ_RXMMIO(0x1468, 0x0),
	SEQ_RXMMIO(0x1474, 0xf0),
	SEQ_RXMMIO(0x102c, 0xffffffff),
	SEQ_RXDEV(0x00f0, 0x01),	/* SCP_BUSCLOCK_SCALE_B1 */
	SEQ_RXMMIO(0x105c, 0xf),
	SEQ_BANK_SWITCH(0x0070),	/* SCP_FRAMECTRL_B1 */
	SEQ_DELAY(887),
	SEQ_RXDEV(0x0102, 0x00),	/* DP1_PORTCTRL */
	SEQ_RXDEV(0x0103, 0x01),	/* DP1_BLOCKCTRL1 */
	SEQ_RXDEV(0x0122, 0x03),	/* DP1_SAMPLECTRL1_B0 */
	SEQ_RXDEV(0x0124, 0x00),	/* DP1_OFFSETCTRL1_B0 */
	SEQ_RXDEV(0x0128, 0x01),	/* DP1_LANECTRL_B0 */
	SEQ_DELAY(370),
	SEQ_RXDEV(0x0202, 0x00),	/* DP2_PORTCTRL */
	SEQ_RXDEV(0x0203, 0x07),	/* DP2_BLOCKCTRL1 */
	SEQ_RXDEV(0x0222, 0x1f),	/* DP2_SAMPLECTRL1_B0 */
	SEQ_RXDEV(0x0224, 0x00),	/* DP2_OFFSETCTRL1_B0 */
	SEQ_RXDEV(0x0228, 0x00),	/* DP2_LANECTRL_B0 */
	SEQ_DELAY(369),
	SEQ_RXDEV(0x0402, 0x00),	/* DP4_PORTCTRL */
	SEQ_RXDEV(0x0403, 0xff),	/* DP4_BLOCKCTRL1 */
	SEQ_RXDEV(0x0422, 0x07),	/* DP4_SAMPLECTRL1_B0 */
	SEQ_RXDEV(0x0424, 0x01),	/* DP4_OFFSETCTRL1_B0 */
	SEQ_RXDEV(0x0428, 0x00),	/* DP4_LANECTRL_B0 */
	SEQ_RXMMIO(0x1124, 0x3),
	SEQ_RXMMIO(0x1128, 0x1),
	SEQ_RXMMIO(0x1134, 0xf0),
	SEQ_RXMMIO(0x112c, 0x1),
	SEQ_RXMMIO(0x1224, 0x1f),
	SEQ_RXMMIO(0x1228, 0x0),
	SEQ_RXMMIO(0x1234, 0x63),
	SEQ_RXMMIO(0x122c, 0x7),
	SEQ_RXMMIO(0x1424, 0x107),
	SEQ_RXMMIO(0x1428, 0x0),
	SEQ_RXMMIO(0x1434, 0xf0),
	SEQ_RXMMIO(0x102c, 0xffffffff),
	SEQ_RXDEV(0x00e0, 0x01),	/* SCP_BUSCLOCK_SCALE_B0 */
	SEQ_RXDEV(0x0120, 0x03),	/* DP1_CHANNELEN_B0 */
	SEQ_RXDEV(0x0220, 0x01),	/* DP2_CHANNELEN_B0 */
	SEQ_RXDEV(0x0420, 0x01),	/* DP4_CHANNELEN_B0 */
	SEQ_RXMMIO(0x1124, 0x3000003),
	SEQ_RXMMIO(0x1224, 0x100001f),
	SEQ_RXMMIO(0x1424, 0x1000107),
	SEQ_RXMMIO(0x101c, 0xf),
	SEQ_BANK_SWITCH(0x0060),	/* SCP_FRAMECTRL_B0 */
};

/* Class-H, the headphone DACs and amplifiers up. */
static const struct wcd_op wcd_hph_on[] = {
	SEQ_CODEC(0x3408, 0x11),	/* DIGITAL_CDC_ANA_CLK_CTL */
	SEQ_CODEC(0x3008, 0x03),	/* ANA_RX_SUPPLIES */
	SEQ_CODEC(0x3408, 0x13),	/* DIGITAL_CDC_ANA_CLK_CTL */
	SEQ_CODEC(0x3136, 0x82),	/* HPH_NEW_INT_RDAC_HD2_CTL_R */
	SEQ_CODEC(0x3008, 0x01),	/* ANA_RX_SUPPLIES */
	SEQ_CODEC(0x3008, 0x05),	/* ANA_RX_SUPPLIES */
	SEQ_CODEC(0x30a8, 0x8f),	/* FLYBACK_VNEG_CTRL_4 */
	SEQ_CODEC(0x30af, 0xb0),	/* FLYBACK_VNEGDAC_CTRL_2 */
	SEQ_CODEC(0x3098, 0x1c),	/* CLASSH_MODE_2 */
	SEQ_CODEC(0x3008, 0x45),	/* ANA_RX_SUPPLIES */
	SEQ_CODEC(0x30af, 0xf0),	/* FLYBACK_VNEGDAC_CTRL_2 */
	SEQ_DELAY(670),
	SEQ_CODEC(0x3008, 0x4d),	/* ANA_RX_SUPPLIES */
	SEQ_CODEC(0x3008, 0xcd),	/* ANA_RX_SUPPLIES */
	SEQ_DELAY(516),
	SEQ_CODEC(0x3098, 0x3a),	/* CLASSH_MODE_2 */
	SEQ_DELAY(555),
	SEQ_CODEC(0x3009, 0x08),	/* ANA_HPH */
	SEQ_CODEC(0x3009, 0x28),	/* ANA_HPH */
	SEQ_CODEC(0x3465, 0x03),	/* DIGITAL_PDM_WD_CTL0 */
	SEQ_CODEC(0x30af, 0xb0),	/* FLYBACK_VNEGDAC_CTRL_2 */
	SEQ_CODEC(0x3098, 0x1c),	/* CLASSH_MODE_2 */
	SEQ_CODEC(0x3009, 0x38),	/* ANA_HPH */
	SEQ_CODEC(0x3466, 0x03),	/* DIGITAL_PDM_WD_CTL1 */
	SEQ_CODEC(0x3009, 0xf8),	/* ANA_HPH */
	SEQ_DELAY(20902),
	SEQ_CODEC(0x3008, 0xcf),	/* ANA_RX_SUPPLIES */
	SEQ_CODEC(0x346c, 0xdf),	/* DIGITAL_INTR_MASK_1 */
	SEQ_CODEC(0x346c, 0x9f),	/* DIGITAL_INTR_MASK_1 */
};

/* And down again. */
static const struct wcd_op wcd_hph_off[] = {
	SEQ_CODEC(0x346c, 0xbf),	/* DIGITAL_INTR_MASK_1 */
	SEQ_DELAY(21745),
	SEQ_CODEC(0x3009, 0x78),	/* ANA_HPH */
	SEQ_CODEC(0x346c, 0xff),	/* DIGITAL_INTR_MASK_1 */
	SEQ_DELAY(21729),
	SEQ_CODEC(0x3009, 0x38),	/* ANA_HPH */
	SEQ_DELAY(22135),
	SEQ_CODEC(0x3009, 0x18),	/* ANA_HPH */
	SEQ_CODEC(0x3465, 0x00),	/* DIGITAL_PDM_WD_CTL0 */
	SEQ_CODEC(0x3009, 0x10),	/* ANA_HPH */
	SEQ_CODEC(0x3008, 0xcb),	/* ANA_RX_SUPPLIES */
	SEQ_CODEC(0x30a8, 0x7f),	/* FLYBACK_VNEG_CTRL_4 */
	SEQ_CODEC(0x3008, 0xc3),	/* ANA_RX_SUPPLIES */
	SEQ_CODEC(0x3009, 0x00),	/* ANA_HPH */
	SEQ_CODEC(0x3466, 0x00),	/* DIGITAL_PDM_WD_CTL1 */
	SEQ_CODEC(0x3008, 0x83),	/* ANA_RX_SUPPLIES */
	SEQ_CODEC(0x30af, 0xf0),	/* FLYBACK_VNEGDAC_CTRL_2 */
	SEQ_DELAY(562),
	SEQ_CODEC(0x3008, 0x03),	/* ANA_RX_SUPPLIES */
	SEQ_DELAY(521),
	SEQ_CODEC(0x3098, 0x3a),	/* CLASSH_MODE_2 */
	SEQ_DELAY(577),
	SEQ_CODEC(0x3136, 0x81),	/* HPH_NEW_INT_RDAC_HD2_CTL_R */
	SEQ_CODEC(0x3008, 0x02),	/* ANA_RX_SUPPLIES */
	SEQ_CODEC(0x3408, 0x11),	/* DIGITAL_CDC_ANA_CLK_CTL */
	SEQ_CODEC(0x3408, 0x10),	/* DIGITAL_CDC_ANA_CLK_CTL */
};

/* The RX link's channels off, in both banks. */
static const struct wcd_op wcd_stream_off[] = {
	SEQ_RXDEV(0x0130, 0x00),	/* DP1_CHANNELEN_B1 */
	SEQ_RXDEV(0x0230, 0x00),	/* DP2_CHANNELEN_B1 */
	SEQ_RXDEV(0x0430, 0x00),	/* DP4_CHANNELEN_B1 */
	SEQ_RXMMIO(0x1164, 0x3),
	SEQ_RXMMIO(0x1264, 0x1f),
	SEQ_RXMMIO(0x1464, 0x107),
	SEQ_DELAY(393),
	SEQ_RXDEV(0x0102, 0x00),	/* DP1_PORTCTRL */
	SEQ_RXDEV(0x0103, 0x01),	/* DP1_BLOCKCTRL1 */
	SEQ_RXDEV(0x0132, 0x03),	/* DP1_SAMPLECTRL1_B1 */
	SEQ_RXDEV(0x0134, 0x00),	/* DP1_OFFSETCTRL1_B1 */
	SEQ_RXDEV(0x0138, 0x01),	/* DP1_LANECTRL_B1 */
	SEQ_DELAY(385),
	SEQ_RXDEV(0x0202, 0x00),	/* DP2_PORTCTRL */
	SEQ_RXDEV(0x0203, 0x07),	/* DP2_BLOCKCTRL1 */
	SEQ_RXDEV(0x0232, 0x1f),	/* DP2_SAMPLECTRL1_B1 */
	SEQ_RXDEV(0x0234, 0x00),	/* DP2_OFFSETCTRL1_B1 */
	SEQ_RXDEV(0x0238, 0x00),	/* DP2_LANECTRL_B1 */
	SEQ_DELAY(380),
	SEQ_RXDEV(0x0402, 0x00),	/* DP4_PORTCTRL */
	SEQ_RXDEV(0x0403, 0xff),	/* DP4_BLOCKCTRL1 */
	SEQ_RXDEV(0x0432, 0x07),	/* DP4_SAMPLECTRL1_B1 */
	SEQ_RXDEV(0x0434, 0x01),	/* DP4_OFFSETCTRL1_B1 */
	SEQ_RXDEV(0x0438, 0x00),	/* DP4_LANECTRL_B1 */
	SEQ_RXMMIO(0x1164, 0x3),
	SEQ_RXMMIO(0x1168, 0x1),
	SEQ_RXMMIO(0x1174, 0xf0),
	SEQ_RXMMIO(0x112c, 0x1),
	SEQ_RXMMIO(0x1264, 0x1f),
	SEQ_RXMMIO(0x1268, 0x0),
	SEQ_RXMMIO(0x1274, 0x63),
	SEQ_RXMMIO(0x122c, 0x7),
	SEQ_RXMMIO(0x1464, 0x107),
	SEQ_RXMMIO(0x1468, 0x0),
	SEQ_RXMMIO(0x1474, 0xf0),
	SEQ_RXMMIO(0x102c, 0xffffffff),
	SEQ_RXDEV(0x00f0, 0x01),	/* SCP_BUSCLOCK_SCALE_B1 */
	SEQ_RXMMIO(0x105c, 0xf),
	SEQ_BANK_SWITCH(0x0070),	/* SCP_FRAMECTRL_B1 */
	SEQ_DELAY(993),
	SEQ_RXDEV(0x0120, 0x00),	/* DP1_CHANNELEN_B0 */
	SEQ_RXDEV(0x0220, 0x00),	/* DP2_CHANNELEN_B0 */
	SEQ_RXDEV(0x0420, 0x00),	/* DP4_CHANNELEN_B0 */
	SEQ_RXMMIO(0x1124, 0x3),
	SEQ_RXMMIO(0x1224, 0x1f),
	SEQ_RXMMIO(0x1424, 0x107),
};

static struct sx wcd_lock;
SX_SYSINIT(qcom_wcd938x, &wcd_lock, "qcom_wcd938x");
static struct qcom_swr *wcd_tx, *wcd_rx;
static bool wcd_hph;

static int
wcd_run(const struct wcd_op *ops, size_t n)
{
	size_t i;
	int error;

	for (i = 0, error = 0; i < n && error == 0; i++) {
		switch (ops[i].op) {
		case OP_CODEC:
			error = qcom_swr_write(wcd_tx, WCD_DEV, ops[i].reg,
			    ops[i].val);
			break;
		case OP_RXDEV:
			error = qcom_swr_write(wcd_rx, WCD_DEV, ops[i].reg,
			    ops[i].val);
			break;
		case OP_RXMMIO:
			qcom_swr_mmio_write(wcd_rx, ops[i].reg, ops[i].val);
			break;
		case OP_BANK_SWITCH:
			error = qcom_swr_bank_switch(wcd_rx, ops[i].reg);
			break;
		case OP_DELAY:
			if (ops[i].val >= 10000)
				pause("wcd", howmany(ops[i].val * hz, 1000000));
			else
				DELAY(ops[i].val);
			break;
		}
	}
	if (error != 0)
		printf("qcom_wcd938x: step %zu (%#x): %d\n", i - 1,
		    ops[i - 1].reg, error);
	return (error);
}

static int
wcd_reset(void)
{
	device_t gpio;
	int error;

	gpio = devclass_get_device(devclass_find("gpio"), 0);
	if (gpio == NULL)
		return (ENXIO);
	error = GPIO_PIN_SETFLAGS(gpio, WCD_RESET_PIN, GPIO_PIN_OUTPUT);
	if (error == 0)
		error = GPIO_PIN_SET(gpio, WCD_RESET_PIN, 0);
	if (error != 0)
		return (error);
	pause("wcdrst", MAX(hz / 50, 1));
	error = GPIO_PIN_SET(gpio, WCD_RESET_PIN, 1);
	pause("wcdrst", MAX(hz / 50, 1));
	return (error);
}

static int
wcd_attached(struct qcom_swr *s)
{
	int i;

	for (i = 0; i < 50; i++) {
		if ((qcom_swr_attached(s) & 1u << WCD_DEV) != 0)
			return (0);
		pause("wcdenum", MAX(hz / 100, 1));
	}
	return (ENXIO);
}

/*
 * Bring the codec up: out of reset, the macros clocked, both links up with
 * the codec enumerated on each, and its settings.
 */
int
qcom_wcd938x_up(void)
{
	int error;

	sx_xlock(&wcd_lock);
	if (wcd_tx != NULL) {
		sx_xunlock(&wcd_lock);
		return (0);
	}
	error = wcd_reset();
	if (error == 0)
		error = qcom_lpass_macro_rx(true);
	if (error == 0)
		error = qcom_swr_up(QCOM_SWR_TX, &wcd_tx);
	if (error == 0)
		error = qcom_swr_up(QCOM_SWR_RX, &wcd_rx);
	if (error == 0)
		error = wcd_attached(wcd_tx);
	if (error == 0)
		error = wcd_attached(wcd_rx);
	if (error == 0)
		error = wcd_run(wcd_init, nitems(wcd_init));
	if (error != 0) {
		printf("qcom_wcd938x: not up: %d\n", error);
		wcd_tx = wcd_rx = NULL;
	} else
		printf("qcom_wcd938x: up\n");
	sx_xunlock(&wcd_lock);
	return (error);
}

void
qcom_wcd938x_down(void)
{

	qcom_wcd938x_hph(false);
	sx_xlock(&wcd_lock);
	if (wcd_tx != NULL) {
		qcom_swr_down();
		(void)qcom_lpass_macro_rx(false);
		wcd_tx = wcd_rx = NULL;
	}
	sx_xunlock(&wcd_lock);
}

/* The headphone output on, with the RX link carrying its ports, or off. */
int
qcom_wcd938x_hph(bool on)
{
	int error;

	sx_xlock(&wcd_lock);
	if (wcd_tx == NULL || on == wcd_hph) {
		sx_xunlock(&wcd_lock);
		return (wcd_tx == NULL && on ? ENXIO : 0);
	}
	if (on) {
		error = wcd_run(wcd_stream_on, nitems(wcd_stream_on));
		if (error == 0)
			error = wcd_run(wcd_hph_on, nitems(wcd_hph_on));
	} else {
		error = wcd_run(wcd_hph_off, nitems(wcd_hph_off));
		if (error == 0)
			error = wcd_run(wcd_stream_off, nitems(wcd_stream_off));
	}
	if (error == 0)
		wcd_hph = on;
	sx_xunlock(&wcd_lock);
	return (error);
}
