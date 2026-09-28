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
 * Qualcomm QUPv3 GENI UART driver.
 *
 * The serial engine runs in FIFO mode with one character per 32-bit FIFO
 * entry in each direction.  Transmission is a primary sequencer command
 * carrying a byte count; reception is a secondary sequencer "read"
 * command left running for as long as the port is in use.
 *
 * The bit rate is produced by dividing the serial engine clock, which is
 * owned by the global clock controller.  Until a clock driver can program
 * that clock, the rate configured by firmware is kept.
 */

#include "opt_acpi.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/conf.h>
#include <sys/kdb.h>

#include <machine/bus.h>

#include <dev/uart/uart.h>
#include <dev/uart/uart_cpu.h>
#include <dev/uart/uart_bus.h>

#ifdef DEV_ACPI
#include <contrib/dev/acpica/include/acpi.h>
#include <dev/uart/uart_cpu_acpi.h>
#endif

#include <dev/qcom_geni/qcom_geni_reg.h>

#include "uart_if.h"

#define	GETREG(bas, reg)	\
    bus_space_read_4((bas)->bst, (bas)->bsh, (reg))
#define	SETREG(bas, reg, value)	\
    bus_space_write_4((bas)->bst, (bas)->bsh, (reg), (value))

/* Upper bound on waiting for a sequencer, generous for slow bit rates. */
#define	GENI_UART_TIMEOUT_US		100000

/* FIFO depth, in entries, assumed when the hardware cannot be queried. */
#define	GENI_UART_DEF_FIFO_DEPTH	16

/* RX FIFO watermark, in entries. */
#define	GENI_UART_RX_WATERMARK		2

/* Flush a partially filled RX FIFO after 16 idle 10-bit character times. */
#define	GENI_UART_RX_STALE_BITS		(16 * 10)

#define	GENI_UART_S_IRQS						\
    (GENI_S_RX_FIFO_WATERMARK | GENI_S_RX_FIFO_LAST |			\
    GENI_S_RX_FIFO_WR_ERR | GENI_S_GP_IRQ_2 | GENI_S_GP_IRQ_3)

static bool
geni_uart_poll(struct uart_bas *bas, bus_size_t reg, uint32_t mask,
    uint32_t want)
{
	int timo;

	for (timo = GENI_UART_TIMEOUT_US; timo > 0; timo -= 2) {
		if ((GETREG(bas, reg) & mask) == want)
			return (true);
		DELAY(2);
	}
	return (false);
}

/*
 * Wait for a transmit command in progress to finish.  If the sequencer
 * does not make progress, cancel the command and, failing that, abort it.
 */
static void
geni_uart_tx_wait(struct uart_bas *bas)
{

	if (geni_uart_poll(bas, GENI_STATUS, GENI_STATUS_M_CMD_ACTIVE, 0))
		return;

	SETREG(bas, GENI_M_CMD_CTRL, GENI_M_CMD_CANCEL);
	if (!geni_uart_poll(bas, GENI_M_IRQ_STATUS, GENI_M_CMD_CANCEL_DONE,
	    GENI_M_CMD_CANCEL_DONE)) {
		SETREG(bas, GENI_M_CMD_CTRL, GENI_M_CMD_ABORT);
		geni_uart_poll(bas, GENI_M_IRQ_STATUS, GENI_M_CMD_ABORT_DONE,
		    GENI_M_CMD_ABORT_DONE);
		SETREG(bas, GENI_M_IRQ_CLEAR, GENI_M_CMD_ABORT_DONE);
	}
	SETREG(bas, GENI_M_IRQ_CLEAR, GENI_M_CMD_CANCEL_DONE);
}

static void
geni_uart_tx_start(struct uart_bas *bas, u_int len)
{

	SETREG(bas, GENI_UART_TX_TRANS_LEN, len);
	SETREG(bas, GENI_M_CMD0,
	    GENI_UART_M_START_TX << GENI_M_OPCODE_SHIFT);
}

static void
geni_uart_rx_start(struct uart_bas *bas)
{

	SETREG(bas, GENI_S_CMD0,
	    GENI_UART_S_START_READ << GENI_S_OPCODE_SHIFT);
}

static void
geni_uart_rx_stop(struct uart_bas *bas)
{

	SETREG(bas, GENI_S_CMD_CTRL, GENI_S_CMD_ABORT);
	geni_uart_poll(bas, GENI_S_CMD_CTRL, GENI_S_CMD_ABORT, 0);
	SETREG(bas, GENI_S_IRQ_CLEAR, GENI_S_CMD_DONE | GENI_S_CMD_ABORT_DONE);
	SETREG(bas, GENI_FORCE_DEFAULT_REG, GENI_FORCE_DEFAULT);
}

static u_int
geni_uart_rx_count(struct uart_bas *bas)
{

	return (GENI_RX_FIFO_WC(GETREG(bas, GENI_RX_FIFO_STATUS)));
}

static int
geni_uart_param(struct uart_bas *bas, int baudrate, int databits,
    int stopbits, int parity)
{
	uint32_t stop;

	if (databits < 5 || databits > 8)
		return (EINVAL);

	switch (stopbits) {
	case 1:
		stop = GENI_UART_TX_STOP_BIT_LEN_1;
		break;
	case 2:
		stop = GENI_UART_TX_STOP_BIT_LEN_2;
		break;
	default:
		return (EINVAL);
	}

	/*
	 * XXX Parity generation and checking are not supported yet: the
	 * encoding of the parity type in the parity configuration registers
	 * is not understood well enough to program it.
	 */
	if (parity != UART_PARITY_NONE)
		return (EINVAL);

	/* Ignore CTS; hardware flow control is not supported yet. */
	SETREG(bas, GENI_UART_TX_TRANS_CFG, GENI_UART_CTS_MASK);
	SETREG(bas, GENI_UART_TX_PARITY_CFG, 0);
	SETREG(bas, GENI_UART_RX_TRANS_CFG, 0);
	SETREG(bas, GENI_UART_RX_PARITY_CFG, 0);
	SETREG(bas, GENI_UART_TX_WORD_LEN, databits);
	SETREG(bas, GENI_UART_RX_WORD_LEN, databits);
	SETREG(bas, GENI_UART_TX_STOP_BIT_LEN, stop);

	return (0);
}

/*
 * Low-level UART interface.
 */
static int	geni_uart_probe(struct uart_bas *bas);
static void	geni_uart_init(struct uart_bas *bas, int, int, int, int);
static void	geni_uart_term(struct uart_bas *bas);
static void	geni_uart_putc(struct uart_bas *bas, int);
static int	geni_uart_rxready(struct uart_bas *bas);
static int	geni_uart_getc(struct uart_bas *bas, struct mtx *mtx);

static struct uart_ops uart_geni_ops = {
	.probe = geni_uart_probe,
	.init = geni_uart_init,
	.term = geni_uart_term,
	.putc = geni_uart_putc,
	.rxready = geni_uart_rxready,
	.getc = geni_uart_getc,
};

static int
geni_uart_probe(struct uart_bas *bas)
{

	if (GENI_FW_REV_PROTOCOL(GETREG(bas, GENI_FW_REVISION_RO)) !=
	    GENI_PROTOCOL_UART)
		return (ENXIO);

	return (0);
}

static void
geni_uart_init(struct uart_bas *bas, int baudrate, int databits,
    int stopbits, int parity)
{

	/* Let output queued by earlier boot stages drain first. */
	geni_uart_tx_wait(bas);
	geni_uart_rx_stop(bas);

	/* One character per FIFO entry, in both directions. */
	SETREG(bas, GENI_TX_PACKING_CFG0, GENI_PACKING_1x8);
	SETREG(bas, GENI_TX_PACKING_CFG1, 0);
	SETREG(bas, GENI_RX_PACKING_CFG0, GENI_PACKING_1x8);
	SETREG(bas, GENI_RX_PACKING_CFG1, 0);
	SETREG(bas, GENI_BYTE_GRANULARITY, 0);

	/* Mask and acknowledge every interrupt source. */
	SETREG(bas, GENI_GSI_EVENT_EN, 0);
	SETREG(bas, GENI_M_IRQ_EN, 0);
	SETREG(bas, GENI_S_IRQ_EN, 0);
	SETREG(bas, GENI_M_IRQ_CLEAR, 0xffffffff);
	SETREG(bas, GENI_S_IRQ_CLEAR, 0xffffffff);
	SETREG(bas, GENI_DMA_TX_IRQ_CLR, 0xffffffff);
	SETREG(bas, GENI_DMA_RX_IRQ_CLR, 0xffffffff);

	/* Ungate the engine clocks and return the I/O pins to defaults. */
	SETREG(bas, GENI_CGC_CTRL,
	    GETREG(bas, GENI_CGC_CTRL) | GENI_CGC_CTRL_DEFAULT);
	SETREG(bas, GENI_DMA_GENERAL_CFG,
	    GETREG(bas, GENI_DMA_GENERAL_CFG) | GENI_DMA_GENERAL_CFG_CGC_ON);
	SETREG(bas, GENI_OUTPUT_CTRL, GENI_OUTPUT_CTRL_DEFAULT);
	SETREG(bas, GENI_FORCE_DEFAULT_REG, GENI_FORCE_DEFAULT);

	/* FIFO mode, with the engine interrupts routed to the SE line. */
	SETREG(bas, GENI_DMA_MODE_EN,
	    GETREG(bas, GENI_DMA_MODE_EN) & ~GENI_DMA_MODE_ENABLE);
	SETREG(bas, GENI_SE_IRQ_EN, GENI_SE_IRQ_DMA_RX | GENI_SE_IRQ_DMA_TX |
	    GENI_SE_IRQ_M | GENI_SE_IRQ_S);

	SETREG(bas, GENI_RX_WATERMARK, GENI_UART_RX_WATERMARK);
	SETREG(bas, GENI_RX_RFR_WATERMARK, GENI_UART_DEF_FIFO_DEPTH - 2);
	SETREG(bas, GENI_UART_RX_STALE_CNT, GENI_UART_RX_STALE_BITS);

	geni_uart_param(bas, baudrate, databits, stopbits, parity);
	geni_uart_rx_start(bas);
}

static void
geni_uart_term(struct uart_bas *bas)
{

	geni_uart_tx_wait(bas);
}

static void
geni_uart_putc(struct uart_bas *bas, int c)
{

	/*
	 * Clearing command-done may consume the completion of a bus-layer
	 * transmission, but that transmission has finished by now and this
	 * command sets command-done again, which the bus layer then sees.
	 */
	geni_uart_tx_wait(bas);
	SETREG(bas, GENI_M_IRQ_CLEAR, GENI_M_CMD_DONE);
	geni_uart_tx_start(bas, 1);
	SETREG(bas, GENI_TX_FIFO, c & 0xff);
	geni_uart_poll(bas, GENI_M_IRQ_STATUS, GENI_M_CMD_DONE,
	    GENI_M_CMD_DONE);
}

static int
geni_uart_rxready(struct uart_bas *bas)
{

	if (geni_uart_rx_count(bas) != 0)
		return (1);

	/* Restart reception if the secondary sequencer has stopped. */
	if ((GETREG(bas, GENI_STATUS) & GENI_STATUS_S_CMD_ACTIVE) == 0)
		geni_uart_rx_start(bas);
	return (0);
}

static int
geni_uart_getc(struct uart_bas *bas, struct mtx *hwmtx)
{
	int c;

	uart_lock(hwmtx);
	while (!geni_uart_rxready(bas)) {
		uart_unlock(hwmtx);
		DELAY(4);
		uart_lock(hwmtx);
	}
	c = GETREG(bas, GENI_RX_FIFO) & 0xff;
	uart_unlock(hwmtx);

	return (c);
}

/*
 * High-level UART interface.
 */
struct geni_uart_softc {
	struct uart_softc	base;
	uint32_t		m_irq_en;
	uint32_t		s_irq_en;
};

static int	geni_uart_bus_probe(struct uart_softc *);
static int	geni_uart_bus_attach(struct uart_softc *);
static int	geni_uart_bus_flush(struct uart_softc *, int);
static int	geni_uart_bus_getsig(struct uart_softc *);
static int	geni_uart_bus_ioctl(struct uart_softc *, int, intptr_t);
static int	geni_uart_bus_ipend(struct uart_softc *);
static int	geni_uart_bus_param(struct uart_softc *, int, int, int, int);
static int	geni_uart_bus_receive(struct uart_softc *);
static int	geni_uart_bus_setsig(struct uart_softc *, int);
static int	geni_uart_bus_transmit(struct uart_softc *);
static void	geni_uart_bus_grab(struct uart_softc *);
static void	geni_uart_bus_ungrab(struct uart_softc *);

static kobj_method_t geni_uart_methods[] = {
	KOBJMETHOD(uart_probe,		geni_uart_bus_probe),
	KOBJMETHOD(uart_attach,		geni_uart_bus_attach),
	KOBJMETHOD(uart_flush,		geni_uart_bus_flush),
	KOBJMETHOD(uart_getsig,		geni_uart_bus_getsig),
	KOBJMETHOD(uart_ioctl,		geni_uart_bus_ioctl),
	KOBJMETHOD(uart_ipend,		geni_uart_bus_ipend),
	KOBJMETHOD(uart_param,		geni_uart_bus_param),
	KOBJMETHOD(uart_receive,	geni_uart_bus_receive),
	KOBJMETHOD(uart_setsig,		geni_uart_bus_setsig),
	KOBJMETHOD(uart_transmit,	geni_uart_bus_transmit),
	KOBJMETHOD(uart_grab,		geni_uart_bus_grab),
	KOBJMETHOD(uart_ungrab,		geni_uart_bus_ungrab),
	KOBJMETHOD_END
};

static int
geni_uart_bus_probe(struct uart_softc *sc)
{
	struct uart_bas *bas;

	bas = &sc->sc_bas;

	/*
	 * Under ACPI nothing says whether firmware left an engine clocked,
	 * and touching an unclocked engine faults.  Only take the engine
	 * firmware uses as its console.
	 */
	if (sc->sc_sysdev == NULL)
		return (ENXIO);

	if (geni_uart_probe(bas) != 0)
		return (ENXIO);

	/*
	 * The FIFO depth fields change width with the QUP version, which
	 * only the QUP wrapper reports; use a depth every version has.
	 */
	sc->sc_txfifosz = GENI_UART_DEF_FIFO_DEPTH;
	sc->sc_rxfifosz = GENI_UART_DEF_FIFO_DEPTH;

	device_set_desc(sc->sc_dev, "Qualcomm GENI UART");
	return (0);
}

static int
geni_uart_bus_attach(struct uart_softc *sc)
{
	struct geni_uart_softc *u;
	struct uart_bas *bas;

	u = (struct geni_uart_softc *)sc;
	bas = &sc->sc_bas;

	/* A console was already set up by the low-level layer. */
	if (sc->sc_sysdev == NULL)
		geni_uart_init(bas, 0, 8, 1, UART_PARITY_NONE);

	sc->sc_hwiflow = 0;
	sc->sc_hwoflow = 0;

	uart_lock(sc->sc_hwmtx);
	u->m_irq_en = 0;
	u->s_irq_en = GENI_UART_S_IRQS;
	SETREG(bas, GENI_M_IRQ_EN, u->m_irq_en);
	SETREG(bas, GENI_S_IRQ_EN, u->s_irq_en);
	uart_unlock(sc->sc_hwmtx);

	return (0);
}

static int
geni_uart_bus_transmit(struct uart_softc *sc)
{
	struct geni_uart_softc *u;
	struct uart_bas *bas;
	int i;

	u = (struct geni_uart_softc *)sc;
	bas = &sc->sc_bas;

	uart_lock(sc->sc_hwmtx);

	/* A console write may still be in flight. */
	geni_uart_tx_wait(bas);

	SETREG(bas, GENI_M_IRQ_CLEAR, GENI_M_CMD_DONE);
	geni_uart_tx_start(bas, sc->sc_txdatasz);
	for (i = 0; i < sc->sc_txdatasz; i++)
		SETREG(bas, GENI_TX_FIFO, sc->sc_txbuf[i]);

	/* Command completion means the last character has been sent. */
	u->m_irq_en |= GENI_M_CMD_DONE;
	SETREG(bas, GENI_M_IRQ_EN, u->m_irq_en);
	sc->sc_txbusy = 1;

	uart_unlock(sc->sc_hwmtx);

	return (0);
}

static int
geni_uart_bus_receive(struct uart_softc *sc)
{
	struct uart_bas *bas;

	bas = &sc->sc_bas;
	uart_lock(sc->sc_hwmtx);

	while (geni_uart_rx_count(bas) != 0) {
		if (uart_rx_full(sc)) {
			sc->sc_rxbuf[sc->sc_rxput] = UART_STAT_OVERRUN;
			break;
		}
		uart_rx_put(sc, GETREG(bas, GENI_RX_FIFO) & 0xff);
	}

	uart_unlock(sc->sc_hwmtx);

	return (0);
}

static int
geni_uart_bus_ipend(struct uart_softc *sc)
{
	struct geni_uart_softc *u;
	struct uart_bas *bas;
	uint32_t m_status, s_status;
	int ipend;

	u = (struct geni_uart_softc *)sc;
	bas = &sc->sc_bas;
	ipend = 0;

	uart_lock(sc->sc_hwmtx);

	m_status = GETREG(bas, GENI_M_IRQ_STATUS) & u->m_irq_en;
	s_status = GETREG(bas, GENI_S_IRQ_STATUS) & u->s_irq_en;
	if (m_status != 0)
		SETREG(bas, GENI_M_IRQ_CLEAR, m_status);
	if (s_status != 0)
		SETREG(bas, GENI_S_IRQ_CLEAR, s_status);

	if ((m_status & GENI_M_CMD_DONE) != 0) {
		u->m_irq_en &= ~GENI_M_CMD_DONE;
		SETREG(bas, GENI_M_IRQ_EN, u->m_irq_en);
		if (sc->sc_txbusy)
			ipend |= SER_INT_TXIDLE;
	}

	if ((s_status & GENI_S_RX_FIFO_WR_ERR) != 0)
		ipend |= SER_INT_OVERRUN;
	if ((s_status & (GENI_S_GP_IRQ_2 | GENI_S_GP_IRQ_3)) != 0)
		ipend |= SER_INT_BREAK;
	if ((s_status & (GENI_S_RX_FIFO_WATERMARK | GENI_S_RX_FIFO_LAST)) != 0)
		ipend |= SER_INT_RXREADY;

	uart_unlock(sc->sc_hwmtx);

	return (ipend);
}

static int
geni_uart_bus_param(struct uart_softc *sc, int baudrate, int databits,
    int stopbits, int parity)
{
	int error;

	uart_lock(sc->sc_hwmtx);
	geni_uart_tx_wait(&sc->sc_bas);
	error = geni_uart_param(&sc->sc_bas, baudrate, databits, stopbits,
	    parity);
	uart_unlock(sc->sc_hwmtx);

	return (error);
}

static int
geni_uart_bus_flush(struct uart_softc *sc, int what)
{
	struct uart_bas *bas;

	bas = &sc->sc_bas;
	if ((what & UART_FLUSH_RECEIVER) != 0) {
		uart_lock(sc->sc_hwmtx);
		while (geni_uart_rx_count(bas) != 0)
			(void)GETREG(bas, GENI_RX_FIFO);
		uart_unlock(sc->sc_hwmtx);
	}

	return (0);
}

static int
geni_uart_bus_getsig(struct uart_softc *sc)
{

	/* No modem control lines are supported; report them asserted. */
	return (SER_CTS | SER_DCD | SER_DSR);
}

static int
geni_uart_bus_setsig(struct uart_softc *sc, int sig)
{

	return (0);
}

static int
geni_uart_bus_ioctl(struct uart_softc *sc, int request, intptr_t data)
{

	return (EINVAL);
}

static void
geni_uart_bus_grab(struct uart_softc *sc)
{
	struct uart_bas *bas;

	/* Mask interrupts for polled operation; ungrab restores them. */
	bas = &sc->sc_bas;
	uart_lock(sc->sc_hwmtx);
	SETREG(bas, GENI_M_IRQ_EN, 0);
	SETREG(bas, GENI_S_IRQ_EN, 0);
	uart_unlock(sc->sc_hwmtx);
}

static void
geni_uart_bus_ungrab(struct uart_softc *sc)
{
	struct geni_uart_softc *u;
	struct uart_bas *bas;

	u = (struct geni_uart_softc *)sc;
	bas = &sc->sc_bas;
	uart_lock(sc->sc_hwmtx);
	SETREG(bas, GENI_M_IRQ_EN, u->m_irq_en);
	SETREG(bas, GENI_S_IRQ_EN, u->s_irq_en);
	uart_unlock(sc->sc_hwmtx);
}

static struct uart_class uart_geni_class = {
	"qcom_geni",
	geni_uart_methods,
	sizeof(struct geni_uart_softc),
	.uc_ops = &uart_geni_ops,
	.uc_range = 0x4000,
	.uc_rclk = 0,
	.uc_rshift = 0,
	.uc_riowidth = 4,
};

#ifdef DEV_ACPI
static struct acpi_spcr_compat_data acpi_spcr_compat_data[] = {
	{ &uart_geni_class,	ACPI_DBG2_SDM845_1_8432MHZ },
	{ &uart_geni_class,	ACPI_DBG2_SDM845_7_372MHZ },
	{ NULL, 0 },
};
UART_ACPI_SPCR_CLASS(acpi_spcr_compat_data);

static struct acpi_uart_compat_data acpi_compat_data[] = {
	{ "QCOM0616",	&uart_geni_class, 0, 0, 0, 0, "Qualcomm GENI UART" },
	{ NULL,		NULL,		  0, 0, 0, 0, NULL },
};
UART_ACPI_CLASS_AND_DEVICE(acpi_compat_data);
#endif
