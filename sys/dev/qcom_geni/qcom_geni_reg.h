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

#ifndef	_DEV_QCOM_GENI_QCOM_GENI_REG_H_
#define	_DEV_QCOM_GENI_QCOM_GENI_REG_H_

/*
 * Qualcomm QUPv3 / GENI (Generic Interface) register definitions.
 *
 * A QUP wrapper hosts a number of serial engines (SEs).  Each SE runs a
 * protocol firmware (SPI, UART, I2C, ...) and exposes a common register
 * block with a primary ("M") sequencer, used for transmit, and a
 * secondary ("S") sequencer, used for UART receive.
 */

/* QUP wrapper registers */
#define	QUPV3_HW_VER			0x0004
#define	 QUPV3_HW_VER_MAJOR(v)		(((v) >> 28) & 0xf)
#define	 QUPV3_HW_VER_MINOR(v)		(((v) >> 16) & 0xfff)

/* Common serial engine registers */
#define	GENI_FORCE_DEFAULT_REG		0x0020
#define	 GENI_FORCE_DEFAULT		(1u << 0)
#define	GENI_OUTPUT_CTRL		0x0024
#define	 GENI_OUTPUT_CTRL_DEFAULT	0x7f
#define	GENI_CGC_CTRL			0x0028
#define	 GENI_CGC_CTRL_DEFAULT		0x7f
#define	GENI_STATUS			0x0040
#define	 GENI_STATUS_M_CMD_ACTIVE	(1u << 0)
#define	 GENI_STATUS_S_CMD_ACTIVE	(1u << 12)
#define	GENI_FW_REVISION_RO		0x0068
#define	 GENI_FW_REV_PROTOCOL(v)	(((v) >> 8) & 0xff)
#define	 GENI_PROTOCOL_SPI		1
#define	 GENI_PROTOCOL_UART		2
#define	 GENI_PROTOCOL_I2C		3
#define	GENI_BYTE_GRANULARITY		0x0254
#define	GENI_DMA_MODE_EN		0x0258
#define	 GENI_DMA_MODE_ENABLE		(1u << 0)
#define	GENI_TX_PACKING_CFG0		0x0260
#define	GENI_TX_PACKING_CFG1		0x0264
#define	GENI_RX_PACKING_CFG0		0x0284
#define	GENI_RX_PACKING_CFG1		0x0288

/*
 * FIFO packing vectors.  Each 10-bit vector describes one protocol word
 * taken from (or placed into) a 32-bit FIFO entry: bits 9:5 are the start
 * bit within the entry, bit 4 the direction (0 = LSB first), bits 3:1 the
 * word length minus one and bit 0 marks the last vector for an entry.
 * A single 8-bit word per FIFO entry, LSB first, is one vector of 0xf.
 */
#define	GENI_PACKING_1x8		0x0000000f

/* Primary (M) sequencer */
#define	GENI_M_CMD0			0x0600
#define	 GENI_M_OPCODE_SHIFT		27
#define	GENI_M_CMD_CTRL			0x0604
#define	 GENI_M_CMD_ABORT		(1u << 1)
#define	 GENI_M_CMD_CANCEL		(1u << 2)
#define	GENI_M_IRQ_STATUS		0x0610
#define	GENI_M_IRQ_EN			0x0614
#define	GENI_M_IRQ_CLEAR		0x0618
#define	 GENI_M_CMD_DONE		(1u << 0)
#define	 GENI_M_CMD_CANCEL_DONE		(1u << 4)
#define	 GENI_M_CMD_ABORT_DONE		(1u << 5)

/* Secondary (S) sequencer */
#define	GENI_S_CMD0			0x0630
#define	 GENI_S_OPCODE_SHIFT		27
#define	GENI_S_CMD_CTRL			0x0634
#define	 GENI_S_CMD_ABORT		(1u << 1)
#define	 GENI_S_CMD_CANCEL		(1u << 2)
#define	GENI_S_IRQ_STATUS		0x0640
#define	GENI_S_IRQ_EN			0x0644
#define	GENI_S_IRQ_CLEAR		0x0648
#define	 GENI_S_CMD_DONE		(1u << 0)
#define	 GENI_S_CMD_CANCEL_DONE		(1u << 4)
#define	 GENI_S_CMD_ABORT_DONE		(1u << 5)
#define	 GENI_S_GP_IRQ_0		(1u << 9)
#define	 GENI_S_GP_IRQ_1		(1u << 10)
#define	 GENI_S_GP_IRQ_2		(1u << 11)
#define	 GENI_S_GP_IRQ_3		(1u << 12)
#define	 GENI_S_RX_FIFO_WR_ERR		(1u << 25)
#define	 GENI_S_RX_FIFO_WATERMARK	(1u << 26)
#define	 GENI_S_RX_FIFO_LAST		(1u << 27)

/* FIFOs */
#define	GENI_TX_FIFO			0x0700
#define	GENI_RX_FIFO			0x0780
#define	GENI_TX_FIFO_STATUS		0x0800
#define	GENI_RX_FIFO_STATUS		0x0804
#define	 GENI_RX_FIFO_WC(v)		((v) & 0x1ffffff)
#define	GENI_TX_WATERMARK		0x080c
#define	GENI_RX_WATERMARK		0x0810
#define	GENI_RX_RFR_WATERMARK		0x0814

/* DMA and top-level interrupt control */
#define	GENI_DMA_TX_IRQ_CLR		0x0c44
#define	GENI_DMA_RX_IRQ_CLR		0x0d44
#define	GENI_GSI_EVENT_EN		0x0e18
#define	GENI_SE_IRQ_EN			0x0e1c
#define	 GENI_SE_IRQ_DMA_RX		(1u << 0)
#define	 GENI_SE_IRQ_DMA_TX		(1u << 1)
#define	 GENI_SE_IRQ_M			(1u << 2)
#define	 GENI_SE_IRQ_S			(1u << 3)
#define	GENI_HW_PARAM_0			0x0e24	/* TX FIFO parameters */
#define	GENI_HW_PARAM_1			0x0e28	/* RX FIFO parameters */
#define	 GENI_HW_PARAM_FIFO_DEPTH(v)	(((v) >> 16) & 0x3f)
#define	 GENI_HW_PARAM_FIFO_DEPTH_256(v) (((v) >> 16) & 0xff)
#define	GENI_DMA_GENERAL_CFG		0x0e30
#define	 GENI_DMA_GENERAL_CFG_CGC_ON	0x0f

/* UART protocol registers */
#define	GENI_UART_TX_TRANS_CFG		0x025c
#define	 GENI_UART_TX_PAR_EN		(1u << 0)
#define	 GENI_UART_CTS_MASK		(1u << 1)
#define	GENI_UART_TX_WORD_LEN		0x0268
#define	GENI_UART_TX_STOP_BIT_LEN	0x026c
#define	 GENI_UART_TX_STOP_BIT_LEN_1	0
#define	 GENI_UART_TX_STOP_BIT_LEN_2	2
#define	GENI_UART_TX_TRANS_LEN		0x0270
#define	GENI_UART_RX_TRANS_CFG		0x0280
#define	GENI_UART_RX_WORD_LEN		0x028c
#define	GENI_UART_RX_STALE_CNT		0x0294
#define	GENI_UART_TX_PARITY_CFG		0x02a4
#define	GENI_UART_RX_PARITY_CFG		0x02a8

/* UART sequencer opcodes */
#define	GENI_UART_M_START_TX		1
#define	GENI_UART_S_START_READ		1

#endif /* _DEV_QCOM_GENI_QCOM_GENI_REG_H_ */
