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

#ifndef _DEV_TCX_IF_TCXREG_H_
#define	_DEV_TCX_IF_TCXREG_H_

/*
 * Toshiba TC956x (TC9564) PCIe Ethernet endpoint.
 *
 * Each of the two PCI functions exposes the same three BARs:
 *   BAR0  bridge configuration, including the AXI-to-PCIe address
 *         translation table (TAMAP)
 *   BAR2  SRAM of the embedded Cortex-M3 (unused)
 *   BAR4  the complete SFR register space of the chip
 * Chip-wide registers (clocks, resets) are driven through function 0.
 * Each function drives its own XGMAC, XPCS, PMA and MSI generator.
 */

#define	TC956X_VENDOR_TOSHIBA		0x1179
#define	TC956X_DEVICE_ETH		0x0220

#define	TC956X_BAR_BRIDGE		PCIR_BAR(0)
#define	TC956X_BAR_SFR			PCIR_BAR(4)

/* BAR0: AXI slave 0 address translation table, four entries */
#define	TC956X_TAMAP_BASE		0x0800
#define	TC956X_TAMAP_STRIDE		0x20
#define	TC956X_TAMAP_NENTRIES		4
#define	TC956X_TAMAP_SRC_LO		0x00
#define	 TC956X_TAMAP_IMPL		(1u << 0)
#define	 TC956X_TAMAP_SIZE_SHIFT	1	/* window is 2^(size + 1) */
#define	TC956X_TAMAP_SRC_HI		0x04
#define	TC956X_TAMAP_TRSL_LO		0x08
#define	TC956X_TAMAP_TRSL_HI		0x0c
#define	TC956X_TAMAP_TRSL_PARAM		0x10

/*
 * The XGMAC reaches host memory through TAMAP entry 0: its AXI addresses
 * from TC956X_DMA_OFFSET up are sent to PCIe address 0 up, for a window
 * of 2^(TC956X_TAMAP_SIZE + 1) = 64GB.
 */
#define	TC956X_DMA_OFFSET		0x1000000000ULL
#define	TC956X_TAMAP_SIZE		35

/* BAR4: chip control */
#define	TC956X_NCID			0x0000
#define	 TC956X_NCID_REV_MASK		0xffu
#define	TC956X_NCLKCTRL(n)		((n) == 0 ? 0x1004 : 0x100c)
#define	TC956X_NRSTCTRL(n)		((n) == 0 ? 0x1008 : 0x1010)
#define	TC956X_NEMACCTL(mac)		(0x1070 + (mac) * 4)
#define	 TC956X_EMACCTL_SP_SEL_MASK	0xfu
#define	 TC956X_EMACCTL_SP_2500		4
#define	 TC956X_EMACCTL_SP_1000		5
#define	 TC956X_EMACCTL_SP_100		6
#define	 TC956X_EMACCTL_SP_10		7
#define	 TC956X_EMACCTL_PHY_INF_MASK	(0x3u << 4)
#define	 TC956X_EMACCTL_PHY_INF_PHYCLK	(0x1u << 4)
#define	 TC956X_EMACCTL_INV_SGM_SIGDET	(1u << 6)
#define	 TC956X_EMACCTL_LPIHWCLKEN	(1u << 8)
#define	 TC956X_EMACCTL_INIT_DONE	(1u << 21)

/* Chip-wide clock and reset bits, in NCLKCTRL(0) and NRSTCTRL(0) */
#define	TC956X_CLK_MSIGEN		(1u << 18)
#define	TC956X_RST_MSIGEN		(1u << 18)

/* Per-MAC clock and reset bits, in NCLKCTRL(mac) and NRSTCTRL(mac) */
#define	TC956X_CLK_MAC_TX		(1u << 7)
#define	TC956X_CLK_MAC_RX		(1u << 14)
#define	TC956X_CLK_MAC_RMII		(1u << 15)	/* MAC 1 only */
#define	TC956X_CLK_MAC_ALL		(1u << 31)
#define	TC956X_RST_MAC			(1u << 7)
#define	TC956X_RST_PMA			(1u << 30)
#define	TC956X_RST_XPCS			(1u << 31)

/*
 * MSI generator, one per function.  The DMA channel interrupts are level
 * sources; it sends one MSI and then holds off until MASK_CLR is written,
 * when it sends another if a source is still asserted.
 */
#define	TC956X_MSIGEN_BASE(mac)		(0xf000 + (mac) * 0x100)
#define	TC956X_MSI_OUT_EN		0x00	/* bit per source */
#define	TC956X_MSI_MASK_CLR		0x0c
#define	 TC956X_MSI_MASK_CLR_ALL	(1u << 0)
#define	TC956X_MSI_SRC_TX(c)		(3 + (c))	/* DMA channel c */
#define	TC956X_MSI_SRC_RX(c)		(11 + (c))

/* The QCA8081 PHY on each port, as wired on the Radxa Dragon Q8B */
#define	TC956X_PHY_ADDR			0x1c

/* XGMAC and the blocks behind it */
#define	TC956X_XGMAC_BASE(mac)		(0x40000 + (mac) * 0x8000)
#define	TC956X_XPCS_OFFSET		0x3a00
#define	TC956X_PMA_OFFSET		0x4000

/*
 * SerDes (PMA) registers, relative to the XGMAC base plus
 * TC956X_PMA_OFFSET.  PMA init powers the CML buffers and switches five
 * lanes' reference clock to the internal CLK_REF_I.
 */
#define	TC956X_PMA_CML_GL_PM_CFG0	0x01b8
#define	TC956X_PMA_NLANES		5
#define	TC956X_PMA_HWT_REFCK_R_EN(i)	(0x1080 + (i) * 0x14)
#define	TC956X_PMA_HWT_REFCK_TERM_EN(i)	(0x1090 + (i) * 0x14)
#define	TC956X_PMA_COMM_CFG_0_1(i)	(0x1888 + (i) * 8)
#define	 TC956X_PMA_COMM_CFG_REFCLK_I	((0xf7u << 9) | (1u << 8) | 0x04)

/*
 * XPCS registers are reached through a 1KB window at XGMAC base plus
 * TC956X_XPCS_OFFSET: an MMD register (mmd << 16 | reg) has its bits
 * 20:8 written to the viewport, and bits 7:0 select the 32-bit word.
 * MMD 31 starts with the clause 22 registers, laid out as in mii.h.
 */
#define	TC956X_XPCS_VIEWPORT		(0xff * 4)
#define	XPCS_MMD_PCS			3
#define	XPCS_MMD_VEND2			31
#define	XPCS_PCS_CTRL2			7	/* MMD 3 */
#define	 XPCS_PCS_TYPE_SEL_MODAL	4	/* reserved: honour mode bits */
#define	XPCS_PCS_STAT2			8	/* MMD 3 */
#define	 XPCS_PCS_STAT2_10GBR		(1u << 0)
#define	XPCS_VR_MII_DIG_CTRL1		0x8000	/* MMD 31 */
#define	 XPCS_DIG_CTRL1_2G5_EN		(1u << 2)
#define	 XPCS_DIG_CTRL1_MAC_AUTO_SW	(1u << 9)
#define	XPCS_VR_MII_AN_CTRL		0x8001	/* MMD 31 */
#define	 XPCS_AN_CTRL_PCS_MODE_MASK	(0x3u << 1)
#define	 XPCS_AN_CTRL_PCS_MODE_SGMII	(0x2u << 1)
#define	 XPCS_AN_CTRL_TX_CONFIG_PHY	(1u << 3)	/* else MAC side */
#define	XPCS_VR_MII_AN_INTR_STS		0x8002	/* MMD 31 */

/* XGMAC MAC registers, relative to TC956X_XGMAC_BASE() */
#define	XGMAC_TX_CONFIG			0x0000
#define	 XGMAC_TX_CONFIG_TE		(1u << 0)
#define	 XGMAC_TX_CONFIG_JD		(1u << 16)
#define	 XGMAC_TX_CONFIG_SS_SHIFT	29
#define	 XGMAC_TX_CONFIG_SS_MASK	(0x7u << 29)
#define	 XGMAC_SS_2500_GMII		0x2
#define	 XGMAC_SS_1000_GMII		0x3
#define	 XGMAC_SS_100_MII		0x4
#define	 XGMAC_SS_10_MII		0x7
#define	XGMAC_RX_CONFIG			0x0004
#define	 XGMAC_RX_CONFIG_RE		(1u << 0)
#define	 XGMAC_RX_CONFIG_ACS		(1u << 1)	/* strip pad/FCS */
#define	 XGMAC_RX_CONFIG_CST		(1u << 2)	/* strip FCS */
#define	 XGMAC_RX_CONFIG_GPSLCE		(1u << 6)
#define	 XGMAC_RX_CONFIG_WD		(1u << 7)
#define	 XGMAC_RX_CONFIG_IPC		(1u << 9)	/* checksum offload */
#define	 XGMAC_RX_CONFIG_GPSL_SHIFT	16
#define	 XGMAC_RX_CONFIG_GPSL_MAX	16368
#define	XGMAC_PACKET_FILTER		0x0008
#define	 XGMAC_FILTER_PR		(1u << 0)
#define	 XGMAC_FILTER_HMC		(1u << 2)	/* hash multicast */
#define	 XGMAC_FILTER_PM		(1u << 4)	/* all multicast */
#define	 XGMAC_FILTER_HPF		(1u << 10)	/* hash or perfect */
/*
 * Multicast hash filter: 64 bins in two registers, as HW_FEATURE1 on
 * the TC956x reports.  A frame's bin is the top six bits of the
 * complement of the big-endian Ethernet CRC of its destination address.
 */
#define	XGMAC_HASH_TABLE(n)		(0x0010 + (n) * 4)
#define	XGMAC_HASH_BITS_LOG2		6
#define	XGMAC_Q_TX_FLOW_CTRL(q)		(0x0070 + (q) * 4)
#define	 XGMAC_TX_FLOW_TFE		(1u << 1)
#define	 XGMAC_TX_FLOW_PT_SHIFT		16	/* pause time, 512 bit times */
#define	XGMAC_RX_FLOW_CTRL		0x0090
#define	 XGMAC_RX_FLOW_RFE		(1u << 0)
#define	XGMAC_RXQ_CTRL0			0x00a0
#define	 XGMAC_RXQ_EN_DCB		0x2	/* per queue, 2 bits each */
#define	XGMAC_INT_EN			0x00b4
#define	XGMAC_VERSION			0x0110
#define	 XGMAC_VERSION_SNPS_MASK	0xffu
#define	XGMAC_MDIO_ADDR			0x0200
#define	 XGMAC_MDIO_ADDR_PA_SHIFT	16	/* PHY (port) address */
#define	 XGMAC_MDIO_ADDR_DA_SHIFT	21	/* clause 45 device */
#define	 XGMAC_MDIO_ADDR_C22_REG_MASK	0x1fu
#define	XGMAC_MDIO_DATA			0x0204
#define	 XGMAC_MDIO_DATA_MASK		0xffffu
#define	 XGMAC_MDIO_CMD_WRITE		(1u << 16)
#define	 XGMAC_MDIO_CMD_READ		(3u << 16)
#define	 XGMAC_MDIO_CR_SHIFT		19	/* MDC clock divider */
#define	 XGMAC_MDIO_BUSY		(1u << 22)
#define	XGMAC_MDIO_C22P			0x0220	/* bit n: port n is clause 22 */
#define	XGMAC_ADDR_HIGH(n)		(0x0300 + (n) * 8)
#define	 XGMAC_ADDR_HIGH_AE		(1u << 31)
#define	XGMAC_ADDR_LOW(n)		(0x0304 + (n) * 8)

/* MAC management counters (MMC), 64-bit where noted */
#define	XGMAC_MMC_BASE			0x0800
#define	XGMAC_MMC_RX_PKT_GB		(XGMAC_MMC_BASE + 0x100)	/* 64 */
#define	XGMAC_MMC_RX_CRC_ERR		(XGMAC_MMC_BASE + 0x128)	/* 64 */
#define	XGMAC_MMC_RX_PAUSE		(XGMAC_MMC_BASE + 0x188)	/* 64 */
#define	XGMAC_MMC_RX_FIFOOVER_PKT	(XGMAC_MMC_BASE + 0x190)	/* 64 */
#define	XGMAC_MMC_RX_DISCARD_PKT_GB	(XGMAC_MMC_BASE + 0x1ac)	/* 64 */

/* MTL, per queue */
/* TQS and RQS, the queue sizes */
#define	XGMAC_MTL_QS(bytes)		(((bytes) / 256 - 1) << 16)
#define	XGMAC_MTL_TXQ_OPMODE(q)		(0x1100 + (q) * 0x80)
#define	 XGMAC_MTL_TSF			(1u << 1)
#define	 XGMAC_MTL_TXQEN_ENABLED	(0x2u << 2)
#define	XGMAC_MTL_TXQ_DEBUG(q)		(0x1108 + (q) * 0x80)
#define	 XGMAC_MTL_TXQ_NOT_EMPTY	(1u << 4)	/* TXQSTS */
#define	XGMAC_MTL_TC_ETS_CONTROL(q)	(0x1110 + (q) * 0x80)
#define	XGMAC_MTL_RXQ_OPMODE(q)		(0x1140 + (q) * 0x80)
#define	 XGMAC_MTL_RSF			(1u << 5)
#define	 XGMAC_MTL_EHFC			(1u << 7)	/* flow control */
#define	XGMAC_MTL_RXQ_MISSED(q)		(0x1144 + (q) * 0x80)
#define	XGMAC_MTL_RXQ_DEBUG(q)		(0x1148 + (q) * 0x80)
#define	 XGMAC_MTL_RXQ_NOT_EMPTY	((0x3fffu << 16) | (0x3u << 4))
/* Thresholds count down from full: FIFO size - (1KB + n * 512 bytes). */
#define	XGMAC_MTL_RXQ_FLOW_CONTROL(q)	(0x1150 + (q) * 0x80)
#define	 XGMAC_MTL_RFA_SHIFT		1	/* send PAUSE above this */
#define	 XGMAC_MTL_RFD_SHIFT		17	/* release it below this */

/* DMA */
#define	XGMAC_DMA_MODE			0x3000
#define	 XGMAC_DMA_MODE_SWR		(1u << 0)
#define	 XGMAC_DMA_MODE_INTM_MASK	(0x3u << 12)
#define	 XGMAC_DMA_MODE_INTM_PERCH	(0x1u << 12)
#define	XGMAC_DMA_SYSBUS_MODE		0x3004
#define	 XGMAC_SYSBUS_WR_OSR_SHIFT	24
#define	 XGMAC_SYSBUS_RD_OSR_SHIFT	16
#define	 XGMAC_SYSBUS_EAME		(1u << 11)
#define	 XGMAC_SYSBUS_BLEN4_128		0x7eu
#define	 XGMAC_SYSBUS_UNDEF		(1u << 0)
#define	XGMAC_TX_EDMA_CTRL		0x3040
#define	XGMAC_RX_EDMA_CTRL		0x3044
#define	 XGMAC_EDMA_PS_MAX		0x3fffffffu
#define	XGMAC_DMA_CH_CONTROL(c)		(0x3100 + (c) * 0x80)
#define	 XGMAC_DMA_CH_PBLX8		(1u << 16)
#define	XGMAC_DMA_CH_TX_CONTROL(c)	(0x3104 + (c) * 0x80)
#define	 XGMAC_DMA_CH_TXST		(1u << 0)
#define	 XGMAC_DMA_CH_TSE		(1u << 12)	/* allow TSO */
#define	XGMAC_DMA_CH_RX_CONTROL(c)	(0x3108 + (c) * 0x80)
#define	 XGMAC_DMA_CH_RXST		(1u << 0)
#define	 XGMAC_DMA_CH_RBSZ_SHIFT	1
#define	 XGMAC_DMA_CH_RBSZ_MASK		(0x3fffu << 1)
#define	 XGMAC_DMA_CH_PBL_SHIFT		16	/* TxPBL and RxPBL */
#define	XGMAC_DMA_CH_TXDESC_HADDR(c)	(0x3110 + (c) * 0x80)
#define	XGMAC_DMA_CH_TXDESC_LADDR(c)	(0x3114 + (c) * 0x80)
#define	XGMAC_DMA_CH_RXDESC_HADDR(c)	(0x3118 + (c) * 0x80)
#define	XGMAC_DMA_CH_RXDESC_LADDR(c)	(0x311c + (c) * 0x80)
#define	XGMAC_DMA_CH_TXDESC_TAIL(c)	(0x3124 + (c) * 0x80)
#define	XGMAC_DMA_CH_RXDESC_TAIL(c)	(0x312c + (c) * 0x80)
#define	XGMAC_DMA_CH_TXDESC_RING_LEN(c)	(0x3130 + (c) * 0x80)
#define	XGMAC_DMA_CH_RXDESC_RING_LEN(c)	(0x3134 + (c) * 0x80)
#define	 XGMAC_DMA_CH_OWRQ_SHIFT	24	/* XGMAC 3.01a erratum */
#define	XGMAC_DMA_CH_INT_EN(c)		(0x3138 + (c) * 0x80)
/*
 * RX interrupt watchdog: after a frame whose descriptor did not ask for an
 * interrupt, RI is raised RWT * 256 DMA clock cycles later.
 */
#define	XGMAC_DMA_CH_RX_WATCHDOG(c)	(0x313c + (c) * 0x80)
#define	 XGMAC_DMA_CH_RWT_MASK		0xffu
#define	XGMAC_DMA_CH_STATUS(c)		(0x3160 + (c) * 0x80)
#define	 XGMAC_DMA_CH_TI		(1u << 0)
#define	 XGMAC_DMA_CH_TPS		(1u << 1)	/* TX stopped */
#define	 XGMAC_DMA_CH_RI		(1u << 6)
#define	 XGMAC_DMA_CH_RBU		(1u << 7)
#define	 XGMAC_DMA_CH_FBE		(1u << 12)
#define	 XGMAC_DMA_CH_AIS		(1u << 14)
#define	 XGMAC_DMA_CH_NIS		(1u << 15)

/*
 * DMA descriptors, 16 bytes.  The same layout is used for TX and RX; the
 * meaning of the fields depends on direction and on who owns it.
 */
struct tcx_desc {
	uint32_t	des0;		/* buffer address, low */
	uint32_t	des1;		/* buffer address, high */
	uint32_t	des2;
	uint32_t	des3;
};

#define	TDES2_B1L_MASK			0x3fffu
#define	TDES2_MSS_MASK			0x3fffu		/* context */
#define	TDES2_IOC			(1u << 31)
#define	TDES3_FL_MASK			0x7fffu
#define	TDES3_TPL_MASK			0x3ffffu	/* TSO payload */
#define	TDES3_CIC_IP			(1u << 16)	/* IPv4 header */
#define	TDES3_CIC_FULL			(3u << 16)	/* and TCP/UDP */
#define	TDES3_TSE			(1u << 18)
#define	TDES3_THL_SHIFT			19		/* TCP header, words */
#define	TDES3_TCMSSV			(1u << 26)	/* context: MSS valid */
#define	TDES3_LD			(1u << 28)
#define	TDES3_FD			(1u << 29)
#define	TDES3_CTXT			(1u << 30)	/* context descriptor */
#define	TDES3_OWN			(1u << 31)

#define	RDES3_PL_MASK			0x3fffu
#define	RDES3_ES			(1u << 15)
#define	RDES3_L34T_SHIFT		20		/* packet type */
#define	RDES3_L34T_MASK			(0xfu << 20)
#define	 RDES3_L34T_IP4TCP		0x1
#define	 RDES3_L34T_IP4UDP		0x2
#define	 RDES3_L34T_IP6TCP		0x9
#define	 RDES3_L34T_IP6UDP		0xa
#define	RDES3_LD			(1u << 28)
#define	RDES3_IOC			(1u << 30)	/* read format */
#define	RDES3_OWN			(1u << 31)

#endif /* _DEV_TCX_IF_TCXREG_H_ */
