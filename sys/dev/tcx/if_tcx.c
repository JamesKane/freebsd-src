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
 * Toshiba TC956x PCIe Ethernet (Synopsys XGMAC 3.01).
 *
 * The TC956x is a PCIe switch with an internal endpoint whose two PCI
 * functions each hold an XGMAC, an XPCS, a SerDes (PMA) and an MSI
 * generator.  Function 0 also owns the chip-wide clocks and resets and
 * the table that translates the MACs' 64GB DMA window onto PCIe.
 *
 * Each MAC reaches its PHY (a QCA8081 on the Radxa Dragon Q8B) over one
 * SerDes lane.  The PHY switches that lane between 2500BASE-X at 2.5G and
 * SGMII, with in-band autonegotiation, at lower speeds; on each change
 * the driver retunes the MAC's speed selector, restarts the PMA and
 * reconfigures the XPCS.  One TX and one RX DMA channel and a single MSI
 * are used.
 *
 * A block whose clock is off or whose reset is asserted may not answer a
 * read, and on Qualcomm PCIe hosts a failed read is an SError, so blocks
 * are only touched once the clock and reset registers say they run.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/mbuf.h>
#include <sys/module.h>
#include <sys/rman.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/sysctl.h>

#include <machine/atomic.h>
#include <machine/bus.h>
#include <machine/resource.h>

#include <net/ethernet.h>
#include <net/if.h>
#include <net/if_var.h>
#include <net/if_dl.h>
#include <net/if_media.h>
#include <net/iflib.h>

#include <netinet/in.h>
#include <netinet/ip.h>

#include <dev/mii/mii.h>
#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <dev/tcx/if_tcxreg.h>

#include "ifdi_if.h"

#define	TCX_NDESC_MIN		64
#define	TCX_NDESC_DEFAULT	512
#define	TCX_NDESC_MAX		1024
#define	TCX_TX_MAXSEGS		32
#define	TCX_TX_MAXSIZE		16384
#define	TCX_TSO_SIZE		IP_MAXPACKET	/* iflib adds a VLAN header */
#define	TCX_MAX_MTU		9000

/*
 * MTL FIFO per queue.  Linux gives each of its four queues 8KB of a 32KB
 * budget; with a single queue an 8KB receive FIFO overflows on 2.5G
 * bursts, so give it the whole budget.  In store-and-forward mode a frame
 * must fit in the transmit FIFO, so that holds a jumbo frame.
 */
#define	TCX_TX_FIFO_BYTES	16384
#define	TCX_RX_FIFO_BYTES	32768

/*
 * Flow control: ask for a pause while the receive FIFO still has room for
 * the frames already on their way, two of the largest or 8KB, whichever is
 * more, and release it 4KB lower.  The pause itself is as long as the MAC
 * allows; it is cancelled early by a zero-quanta PAUSE when the FIFO
 * drains.
 */
#define	TCX_FC_HEADROOM_MIN	8192
#define	TCX_FC_HYSTERESIS	4096
#define	TCX_PAUSE_TIME		0xffff

/*
 * Receive interrupt moderation.  Descriptors are refilled without asking
 * for an interrupt, except one in every rx_coal_frames if that is set, and
 * the RX watchdog raises one rx_riwt units after the first such frame.
 * A unit is 256 cycles of the 125MHz DMA clock, about 2us, so the default
 * holds a frame back by up to about 130us, near 8000 interrupts a second
 * while streaming.  An rx_riwt of 0 turns moderation off.
 */
#define	TCX_RX_RIWT_DEFAULT	64
#define	TCX_RX_COAL_FRAMES_DEFAULT 0

/*
 * Transmit interrupt moderation.  iflib asks for a completion interrupt
 * as often as every other packet when the ring is nearly empty; honour
 * the request only once tx_coal_frames descriptors have been queued since
 * the last one.  iflib also reclaims descriptors when it transmits and
 * from its timer, so completions are never stranded.
 */
#define	TCX_TX_COAL_FRAMES_DEFAULT 128
#define	TCX_PBL			32
#define	TCX_DMA_WIDTH		36	/* the translation window is 64GB */

/*
 * MDC divider.  The CSR clock is 125MHz and 0 divides it by 62, giving
 * 2MHz, which is what Linux uses on this chip.
 */
#define	TCX_MDIO_CR		0
#define	TCX_MDIO_TIMEOUT	10000	/* microseconds */
#define	TCX_SWR_TIMEOUT		100000	/* microseconds */
#define	TCX_PMA_TIMEOUT		1000000	/* microseconds */
#define	TCX_XPCS_RESET_TIMEOUT	600000	/* microseconds */

/*
 * The QCA8081's SerDes answers at the next MDIO address.  Its FIFO toward
 * the MAC must be taken out of reset on each link up and put back on link
 * down, as Linux does; while it is held no frames pass, although the
 * SerDes link and SGMII autonegotiation work.  Firmware only releases it
 * on ports that had a link when it ran.
 */
#define	QCA8081_SERDES_ADDR		(TC956X_PHY_ADDR + 1)
#define	QCA8081_SERDES_FIFO_CTRL	0x9072		/* MMD 1 */
#define	 QCA8081_SERDES_FIFO_RSTN	(1u << 11)

/* QCA8081 registers reached through the clause 22 MMD window */
#define	MII_MMDCTRL		0x0d
#define	 MMDCTRL_DATA_NOINC	0x4000
#define	MII_MMDDATA		0x0e
#define	MMD_AN			7
#define	MMD_AN_10GBT_CTRL	0x0020
#define	 MMD_AN_10GBT_ADV2_5G	0x0080

/*
 * Attach restarts each MAC, SerDes and PCS from reset, as Linux does, so
 * the result does not depend on what firmware left behind.  Set
 * hw.tcx.cold_init=0 to keep a MAC that firmware left running, for
 * example to keep the link while booting from the network.
 */
static int tcx_cold_init = 1;
TUNABLE_INT("hw.tcx.cold_init", &tcx_cold_init);

/* QCA8081 PHY-specific status register */
#define	QCA808X_PHY_SPEC_STATUS		0x11
#define	 QCA808X_SS_LINK		(1u << 10)
#define	 QCA808X_SS_DUPLEX		(1u << 13)
#define	 QCA808X_SS_SPEED_SHIFT		7
#define	 QCA808X_SS_SPEED_MASK		(0x7u << 7)
#define	 QCA808X_SS_SPEED_10		0
#define	 QCA808X_SS_SPEED_100		1
#define	 QCA808X_SS_SPEED_1000		2
#define	 QCA808X_SS_SPEED_2500		4

struct tcx_softc;

struct tcx_txq {
	struct tcx_softc	*sc;
	struct tcx_desc		*ring;
	uint64_t		paddr;
	qidx_t			cidx;	/* oldest descriptor not reclaimed */
	qidx_t			pidx;	/* next descriptor to fill */
	uint16_t		tso_mss; /* MSS last given to the DMA */
};

struct tcx_rxq {
	struct tcx_softc	*sc;
	struct tcx_desc		*ring;
	uint64_t		paddr;
	qidx_t			pidx;	/* next descriptor to refill */
};

struct tcx_softc {
	device_t		dev;
	if_ctx_t		ctx;
	if_softc_ctx_t		scctx;
	struct ifmedia		*media;
	struct resource		*bridge_res;
	struct resource		*sfr_res;
	int			mac;		/* PCI function, 0 or 1 */
	struct tcx_txq		txq;
	struct tcx_rxq		rxq;
	bool			link_up;
	bool			link_fdx;
	u_int			link_speed;	/* Mb/s */
	bool			link_txpause;	/* we may send PAUSE */
	bool			link_rxpause;	/* we obey PAUSE */
	u_int			serdes_speed;	/* what the SerDes is set to */
	int			link_reported;	/* LINK_STATE_* for iflib */
	u_int			speed_reported;
	bool			txpause_reported;
	bool			rxpause_reported;
	u_int			rx_riwt;	/* RX watchdog, 256 cycles */
	u_int			rx_coal_frames;	/* IOC every n; 0 never */
	u_int			rx_ioc_count;
	u_int			tx_coal_frames;	/* descriptors between IOCs */
	u_int			tx_since_ioc;
	u_long			stat_rx_err;	/* frames with ES set */
	uint32_t		rx_err_des3;	/* status of the last */
	u_long			stat_intr;	/* interrupts taken */
	u_long			stat_intr_rx;	/* ... with RI set */
	u_long			stat_intr_tx;	/* ... with TI set */
	u_long			stat_rbu;	/* RX buffer unavailable */
	u_long			stat_fbe;	/* fatal bus error */
};

#define	SFR_READ(sc, reg)	bus_read_4((sc)->sfr_res, (reg))
#define	SFR_WRITE(sc, reg, val)	bus_write_4((sc)->sfr_res, (reg), (val))
#define	BRIDGE_WRITE(sc, reg, val) bus_write_4((sc)->bridge_res, (reg), (val))
#define	MAC_READ(sc, reg)	\
	SFR_READ((sc), TC956X_XGMAC_BASE((sc)->mac) + (reg))
#define	MAC_WRITE(sc, reg, val)	\
	SFR_WRITE((sc), TC956X_XGMAC_BASE((sc)->mac) + (reg), (val))
#define	MSI_WRITE(sc, reg, val)	\
	SFR_WRITE((sc), TC956X_MSIGEN_BASE((sc)->mac) + (reg), (val))
#define	MSI_READ(sc, reg)	\
	SFR_READ((sc), TC956X_MSIGEN_BASE((sc)->mac) + (reg))

static device_register_t	tcx_register;

static ifdi_attach_pre_t	tcx_attach_pre;
static ifdi_attach_post_t	tcx_attach_post;
static ifdi_detach_t		tcx_detach;
static ifdi_tx_queues_alloc_t	tcx_tx_queues_alloc;
static ifdi_rx_queues_alloc_t	tcx_rx_queues_alloc;
static ifdi_queues_free_t	tcx_queues_free;
static ifdi_init_t		tcx_init;
static ifdi_stop_t		tcx_stop;
static ifdi_intr_enable_t	tcx_intr_enable;
static ifdi_intr_disable_t	tcx_intr_disable;
static ifdi_multi_set_t		tcx_multi_set;
static ifdi_mtu_set_t		tcx_mtu_set;
static ifdi_promisc_set_t	tcx_promisc_set;
static ifdi_timer_t		tcx_timer;
static ifdi_update_admin_status_t tcx_update_admin_status;
static ifdi_media_status_t	tcx_media_status;
static ifdi_media_change_t	tcx_media_change;
static ifdi_get_counter_t	tcx_get_counter;

static int	tcx_txd_encap(void *, if_pkt_info_t);
static void	tcx_txd_flush(void *, uint16_t, qidx_t);
static int	tcx_txd_credits_update(void *, uint16_t, bool);
static int	tcx_rxd_available(void *, uint16_t, qidx_t, qidx_t);
static int	tcx_rxd_pkt_get(void *, if_rxd_info_t);
static void	tcx_rxd_refill(void *, if_rxd_update_t);
static void	tcx_rxd_flush(void *, uint16_t, uint8_t, qidx_t);
static int	tcx_intr(void *);

static const char tcx_driver_version[] = "0.1";

static const pci_vendor_info_t tcx_vendor_info_array[] = {
	PVID(TC956X_VENDOR_TOSHIBA, TC956X_DEVICE_ETH,
	    "Toshiba TC956x Ethernet"),
	PVID_END
};

static device_method_t tcx_methods[] = {
	DEVMETHOD(device_register,	tcx_register),
	DEVMETHOD(device_probe,		iflib_device_probe),
	DEVMETHOD(device_attach,	iflib_device_attach),
	DEVMETHOD(device_detach,	iflib_device_detach),
	DEVMETHOD(device_shutdown,	iflib_device_shutdown),
	DEVMETHOD(device_suspend,	iflib_device_suspend),
	DEVMETHOD(device_resume,	iflib_device_resume),

	DEVMETHOD_END
};

static driver_t tcx_driver = {
	"tcx", tcx_methods, sizeof(struct tcx_softc)
};

DRIVER_MODULE(tcx, pci, tcx_driver, NULL, NULL);
MODULE_VERSION(tcx, 1);
IFLIB_PNP_INFO(pci, tcx, tcx_vendor_info_array);
MODULE_DEPEND(tcx, pci, 1, 1, 1);
MODULE_DEPEND(tcx, ether, 1, 1, 1);
MODULE_DEPEND(tcx, iflib, 1, 1, 1);

static device_method_t tcx_iflib_methods[] = {
	DEVMETHOD(ifdi_attach_pre,		tcx_attach_pre),
	DEVMETHOD(ifdi_attach_post,		tcx_attach_post),
	DEVMETHOD(ifdi_detach,			tcx_detach),
	DEVMETHOD(ifdi_tx_queues_alloc,		tcx_tx_queues_alloc),
	DEVMETHOD(ifdi_rx_queues_alloc,		tcx_rx_queues_alloc),
	DEVMETHOD(ifdi_queues_free,		tcx_queues_free),
	DEVMETHOD(ifdi_init,			tcx_init),
	DEVMETHOD(ifdi_stop,			tcx_stop),
	DEVMETHOD(ifdi_intr_enable,		tcx_intr_enable),
	DEVMETHOD(ifdi_intr_disable,		tcx_intr_disable),
	DEVMETHOD(ifdi_multi_set,		tcx_multi_set),
	DEVMETHOD(ifdi_mtu_set,			tcx_mtu_set),
	DEVMETHOD(ifdi_promisc_set,		tcx_promisc_set),
	DEVMETHOD(ifdi_timer,			tcx_timer),
	DEVMETHOD(ifdi_update_admin_status,	tcx_update_admin_status),
	DEVMETHOD(ifdi_media_status,		tcx_media_status),
	DEVMETHOD(ifdi_media_change,		tcx_media_change),
	DEVMETHOD(ifdi_get_counter,		tcx_get_counter),

	DEVMETHOD_END
};

static driver_t tcx_iflib_driver = {
	"tcx", tcx_iflib_methods, sizeof(struct tcx_softc)
};

static struct if_txrx tcx_txrx = {
	.ift_txd_encap = tcx_txd_encap,
	.ift_txd_flush = tcx_txd_flush,
	.ift_txd_credits_update = tcx_txd_credits_update,
	.ift_rxd_available = tcx_rxd_available,
	.ift_rxd_pkt_get = tcx_rxd_pkt_get,
	.ift_rxd_refill = tcx_rxd_refill,
	.ift_rxd_flush = tcx_rxd_flush,
	.ift_legacy_intr = tcx_intr,
};

static struct if_shared_ctx tcx_sctx_init = {
	.isc_magic = IFLIB_MAGIC,
	.isc_q_align = PAGE_SIZE,

	.isc_tx_maxsize = TCX_TX_MAXSIZE,
	.isc_tx_maxsegsize = PAGE_SIZE,
	.isc_tso_maxsize = TCX_TSO_SIZE + sizeof(struct ether_vlan_header),
	.isc_tso_maxsegsize = PAGE_SIZE,
	.isc_rx_maxsize = MJUMPAGESIZE,
	.isc_rx_maxsegsize = MJUMPAGESIZE,
	.isc_rx_nsegments = 1,

	.isc_admin_intrcnt = 0,
	.isc_nfl = 1,
	.isc_ntxqs = 1,
	.isc_nrxqs = 1,

	.isc_vendor_info = tcx_vendor_info_array,
	.isc_driver_version = tcx_driver_version,
	.isc_driver = &tcx_iflib_driver,

	/* We allocate our single MSI ourselves, see tcx_attach_pre(). */
	.isc_flags = IFLIB_SKIP_MSIX,

	.isc_ntxd_min = { TCX_NDESC_MIN },
	.isc_ntxd_max = { TCX_NDESC_MAX },
	.isc_ntxd_default = { TCX_NDESC_DEFAULT },
	.isc_nrxd_min = { TCX_NDESC_MIN },
	.isc_nrxd_max = { TCX_NDESC_MAX },
	.isc_nrxd_default = { TCX_NDESC_DEFAULT },
};

static void *
tcx_register(device_t dev)
{
	return (&tcx_sctx_init);
}

/*
 * Chip level
 */

static void
tcx_sfr_update(struct tcx_softc *sc, bus_size_t reg, uint32_t clr,
    uint32_t set)
{
	SFR_WRITE(sc, reg, (SFR_READ(sc, reg) & ~clr) | set);
}

static bool
tcx_msigen_running(struct tcx_softc *sc)
{
	return ((SFR_READ(sc, TC956X_NCLKCTRL(0)) & TC956X_CLK_MSIGEN) != 0 &&
	    (SFR_READ(sc, TC956X_NRSTCTRL(0)) & TC956X_RST_MSIGEN) == 0);
}

static bool
tcx_mac_running(struct tcx_softc *sc)
{
	return ((SFR_READ(sc, TC956X_NCLKCTRL(sc->mac)) &
	    TC956X_CLK_MAC_ALL) != 0 &&
	    (SFR_READ(sc, TC956X_NRSTCTRL(sc->mac)) & TC956X_RST_MAC) == 0);
}

/*
 * Chip-wide setup, done by function 0.  Firmware has been seen to leave
 * the translation table programmed already, but it is cheap to redo and
 * must be right before the MACs can DMA.
 */
static void
tcx_chip_init(struct tcx_softc *sc)
{
	bus_size_t e;
	int i;

	e = TC956X_TAMAP_BASE;
	BRIDGE_WRITE(sc, e + TC956X_TAMAP_SRC_LO,
	    (uint32_t)TC956X_DMA_OFFSET |
	    (TC956X_TAMAP_SIZE << TC956X_TAMAP_SIZE_SHIFT) | TC956X_TAMAP_IMPL);
	BRIDGE_WRITE(sc, e + TC956X_TAMAP_SRC_HI,
	    (uint32_t)(TC956X_DMA_OFFSET >> 32));
	BRIDGE_WRITE(sc, e + TC956X_TAMAP_TRSL_LO, 0);
	BRIDGE_WRITE(sc, e + TC956X_TAMAP_TRSL_HI, 0);
	BRIDGE_WRITE(sc, e + TC956X_TAMAP_TRSL_PARAM, 0);
	for (i = 1; i < TC956X_TAMAP_NENTRIES; i++) {
		e = TC956X_TAMAP_BASE + i * TC956X_TAMAP_STRIDE;
		BRIDGE_WRITE(sc, e + TC956X_TAMAP_SRC_LO, 0);
		BRIDGE_WRITE(sc, e + TC956X_TAMAP_SRC_HI, 0);
		BRIDGE_WRITE(sc, e + TC956X_TAMAP_TRSL_LO, 0);
		BRIDGE_WRITE(sc, e + TC956X_TAMAP_TRSL_HI, 0);
		BRIDGE_WRITE(sc, e + TC956X_TAMAP_TRSL_PARAM, 0);
	}

	/* Start the MSI generators; clock first, then release the reset. */
	tcx_sfr_update(sc, TC956X_NCLKCTRL(0), 0, TC956X_CLK_MSIGEN);
	tcx_sfr_update(sc, TC956X_NRSTCTRL(0), TC956X_RST_MSIGEN, 0);
}

/*
 * MDIO and PHY
 */

static int
tcx_mdio_wait(struct tcx_softc *sc)
{
	int i;

	for (i = 0; i < TCX_MDIO_TIMEOUT; i++) {
		if ((MAC_READ(sc, XGMAC_MDIO_DATA) & XGMAC_MDIO_BUSY) == 0)
			return (0);
		DELAY(1);
	}
	return (ETIMEDOUT);
}

/* Clause 22 read.  Returns the register value, or -1 on timeout. */
static int
tcx_mdio_read_c22(struct tcx_softc *sc, int phy, int reg)
{
	if (tcx_mdio_wait(sc) != 0)
		return (-1);

	MAC_WRITE(sc, XGMAC_MDIO_C22P, 1u << phy);
	MAC_WRITE(sc, XGMAC_MDIO_ADDR, (phy << XGMAC_MDIO_ADDR_PA_SHIFT) |
	    (reg & XGMAC_MDIO_ADDR_C22_REG_MASK));
	MAC_WRITE(sc, XGMAC_MDIO_DATA,
	    (TCX_MDIO_CR << XGMAC_MDIO_CR_SHIFT) | XGMAC_MDIO_CMD_READ |
	    XGMAC_MDIO_BUSY);

	if (tcx_mdio_wait(sc) != 0)
		return (-1);
	return (MAC_READ(sc, XGMAC_MDIO_DATA) & XGMAC_MDIO_DATA_MASK);
}

/* Clause 45 access; the port must not be marked clause 22 in C22P. */
static int
tcx_mdio_c45(struct tcx_softc *sc, int phy, int mmd, int reg, uint32_t cmd,
    uint16_t val)
{
	if (tcx_mdio_wait(sc) != 0)
		return (-1);

	MAC_WRITE(sc, XGMAC_MDIO_C22P,
	    MAC_READ(sc, XGMAC_MDIO_C22P) & ~(1u << phy));
	MAC_WRITE(sc, XGMAC_MDIO_ADDR, (phy << XGMAC_MDIO_ADDR_PA_SHIFT) |
	    (mmd << XGMAC_MDIO_ADDR_DA_SHIFT) | (reg & 0xffff));
	MAC_WRITE(sc, XGMAC_MDIO_DATA,
	    (TCX_MDIO_CR << XGMAC_MDIO_CR_SHIFT) | cmd | XGMAC_MDIO_BUSY | val);

	if (tcx_mdio_wait(sc) != 0)
		return (-1);
	return (MAC_READ(sc, XGMAC_MDIO_DATA) & XGMAC_MDIO_DATA_MASK);
}

/* Hold the PHY's SerDes FIFO in reset while the link is down. */
static void
tcx_phy_serdes_fifo(struct tcx_softc *sc, bool up)
{
	int v;

	v = tcx_mdio_c45(sc, QCA8081_SERDES_ADDR, 1, QCA8081_SERDES_FIFO_CTRL,
	    XGMAC_MDIO_CMD_READ, 0);
	if (v < 0)
		return;
	if (up)
		v |= QCA8081_SERDES_FIFO_RSTN;
	else
		v &= ~QCA8081_SERDES_FIFO_RSTN;
	tcx_mdio_c45(sc, QCA8081_SERDES_ADDR, 1, QCA8081_SERDES_FIFO_CTRL,
	    XGMAC_MDIO_CMD_WRITE, v);
}

/* Clause 22 write.  Returns 0 or ETIMEDOUT. */
static int
tcx_mdio_write_c22(struct tcx_softc *sc, int phy, int reg, uint16_t val)
{
	if (tcx_mdio_wait(sc) != 0)
		return (ETIMEDOUT);

	MAC_WRITE(sc, XGMAC_MDIO_C22P, 1u << phy);
	MAC_WRITE(sc, XGMAC_MDIO_ADDR, (phy << XGMAC_MDIO_ADDR_PA_SHIFT) |
	    (reg & XGMAC_MDIO_ADDR_C22_REG_MASK));
	MAC_WRITE(sc, XGMAC_MDIO_DATA,
	    (TCX_MDIO_CR << XGMAC_MDIO_CR_SHIFT) | XGMAC_MDIO_CMD_WRITE |
	    XGMAC_MDIO_BUSY | val);
	return (tcx_mdio_wait(sc));
}

/*
 * Make the PHY advertise what the MAC can do: symmetric and asymmetric
 * pause, and no half duplex, which the XGMAC does not support.  Firmware
 * or the PHY's defaults may differ, and changing the advertisement means
 * renegotiating, which drops the link for a few seconds.
 */
static void
tcx_phy_fix_advert(struct tcx_softc *sc)
{
	int anar, bmcr, gtcr, nanar, ngtcr;

	anar = tcx_mdio_read_c22(sc, TC956X_PHY_ADDR, MII_ANAR);
	gtcr = tcx_mdio_read_c22(sc, TC956X_PHY_ADDR, MII_100T2CR);
	bmcr = tcx_mdio_read_c22(sc, TC956X_PHY_ADDR, MII_BMCR);
	if (anar < 0 || gtcr < 0 || bmcr < 0)
		return;
	nanar = (anar & ~(ANAR_10 | ANAR_TX)) | ANAR_PAUSE_SYM |
	    ANAR_PAUSE_ASYM;
	ngtcr = gtcr & ~GTCR_ADV_1000THDX;
	if (nanar == anar && ngtcr == gtcr)
		return;

	device_printf(sc->dev, "updating PHY advertisement, "
	    "renegotiating\n");
	tcx_mdio_write_c22(sc, TC956X_PHY_ADDR, MII_ANAR, nanar);
	tcx_mdio_write_c22(sc, TC956X_PHY_ADDR, MII_100T2CR, ngtcr);
	tcx_mdio_write_c22(sc, TC956X_PHY_ADDR, MII_BMCR,
	    bmcr | BMCR_AUTOEN | BMCR_STARTNEG);
}

static int
tcx_phy_mmd_read(struct tcx_softc *sc, int mmd, int reg)
{
	tcx_mdio_write_c22(sc, TC956X_PHY_ADDR, MII_MMDCTRL, mmd);
	tcx_mdio_write_c22(sc, TC956X_PHY_ADDR, MII_MMDDATA, reg);
	tcx_mdio_write_c22(sc, TC956X_PHY_ADDR, MII_MMDCTRL,
	    MMDCTRL_DATA_NOINC | mmd);
	return (tcx_mdio_read_c22(sc, TC956X_PHY_ADDR, MII_MMDDATA));
}

static void
tcx_phy_mmd_write(struct tcx_softc *sc, int mmd, int reg, uint16_t val)
{
	tcx_mdio_write_c22(sc, TC956X_PHY_ADDR, MII_MMDCTRL, mmd);
	tcx_mdio_write_c22(sc, TC956X_PHY_ADDR, MII_MMDDATA, reg);
	tcx_mdio_write_c22(sc, TC956X_PHY_ADDR, MII_MMDCTRL,
	    MMDCTRL_DATA_NOINC | mmd);
	tcx_mdio_write_c22(sc, TC956X_PHY_ADDR, MII_MMDDATA, val);
}

/*
 * Advertise one speed, or all of them for IFM_AUTO, full duplex only, and
 * renegotiate.
 */
static int
tcx_phy_set_media(struct tcx_softc *sc, int subtype)
{
	int anar, gtcr, adv25;

	anar = tcx_mdio_read_c22(sc, TC956X_PHY_ADDR, MII_ANAR);
	gtcr = tcx_mdio_read_c22(sc, TC956X_PHY_ADDR, MII_100T2CR);
	adv25 = tcx_phy_mmd_read(sc, MMD_AN, MMD_AN_10GBT_CTRL);
	if (anar < 0 || gtcr < 0 || adv25 < 0)
		return (EIO);
	anar &= ~(ANAR_10 | ANAR_10_FD | ANAR_TX | ANAR_TX_FD);
	gtcr &= ~(GTCR_ADV_1000TFDX | GTCR_ADV_1000THDX);
	adv25 &= ~MMD_AN_10GBT_ADV2_5G;

	switch (subtype) {
	case IFM_AUTO:
		anar |= ANAR_10_FD | ANAR_TX_FD;
		gtcr |= GTCR_ADV_1000TFDX;
		adv25 |= MMD_AN_10GBT_ADV2_5G;
		break;
	case IFM_2500_T:
		adv25 |= MMD_AN_10GBT_ADV2_5G;
		break;
	case IFM_1000_T:
		gtcr |= GTCR_ADV_1000TFDX;
		break;
	case IFM_100_TX:
		anar |= ANAR_TX_FD;
		break;
	case IFM_10_T:
		anar |= ANAR_10_FD;
		break;
	default:
		return (EINVAL);
	}
	anar |= ANAR_PAUSE_SYM | ANAR_PAUSE_ASYM;

	tcx_mdio_write_c22(sc, TC956X_PHY_ADDR, MII_ANAR, anar);
	tcx_mdio_write_c22(sc, TC956X_PHY_ADDR, MII_100T2CR, gtcr);
	tcx_phy_mmd_write(sc, MMD_AN, MMD_AN_10GBT_CTRL, adv25);
	tcx_mdio_write_c22(sc, TC956X_PHY_ADDR, MII_BMCR,
	    BMCR_AUTOEN | BMCR_STARTNEG);
	return (0);
}

/* Resolve PAUSE use from both sides' advertisements (802.3 Annex 28B). */
static void
tcx_phy_resolve_pause(struct tcx_softc *sc)
{
	int anar, anlpar;

	sc->link_txpause = sc->link_rxpause = false;
	if (!sc->link_up || !sc->link_fdx)
		return;
	anar = tcx_mdio_read_c22(sc, TC956X_PHY_ADDR, MII_ANAR);
	anlpar = tcx_mdio_read_c22(sc, TC956X_PHY_ADDR, MII_ANLPAR);
	if (anar < 0 || anlpar < 0)
		return;

	if ((anar & ANAR_PAUSE_SYM) != 0 && (anlpar & ANLPAR_PAUSE_SYM) != 0)
		sc->link_txpause = sc->link_rxpause = true;
	else if ((anar & ANAR_PAUSE_ASYM) != 0 &&
	    (anlpar & ANLPAR_PAUSE_ASYM) != 0) {
		if ((anar & ANAR_PAUSE_SYM) != 0)
			sc->link_rxpause = true;
		else if ((anlpar & ANLPAR_PAUSE_SYM) != 0)
			sc->link_txpause = true;
	}
}

/* Read the link state from the PHY.  Returns false if the PHY is silent. */
static bool
tcx_phy_poll(struct tcx_softc *sc)
{
	int ss;

	ss = tcx_mdio_read_c22(sc, TC956X_PHY_ADDR, QCA808X_PHY_SPEC_STATUS);
	if (ss < 0)
		return (false);

	sc->link_up = (ss & QCA808X_SS_LINK) != 0;
	sc->link_fdx = (ss & QCA808X_SS_DUPLEX) != 0;
	switch ((ss & QCA808X_SS_SPEED_MASK) >> QCA808X_SS_SPEED_SHIFT) {
	case QCA808X_SS_SPEED_10:
		sc->link_speed = 10;
		break;
	case QCA808X_SS_SPEED_100:
		sc->link_speed = 100;
		break;
	case QCA808X_SS_SPEED_1000:
		sc->link_speed = 1000;
		break;
	case QCA808X_SS_SPEED_2500:
		sc->link_speed = 2500;
		break;
	default:
		sc->link_speed = 0;
		break;
	}
	tcx_phy_resolve_pause(sc);
	return (true);
}

/* Set the MAC's port speed and PAUSE use. */
static void
tcx_mac_set_speed(struct tcx_softc *sc)
{
	uint32_t ss, v;

	switch (sc->link_speed) {
	case 10:
		ss = XGMAC_SS_10_MII;
		break;
	case 100:
		ss = XGMAC_SS_100_MII;
		break;
	case 1000:
		ss = XGMAC_SS_1000_GMII;
		break;
	default:
		ss = XGMAC_SS_2500_GMII;
		break;
	}
	v = MAC_READ(sc, XGMAC_TX_CONFIG) & ~XGMAC_TX_CONFIG_SS_MASK;
	MAC_WRITE(sc, XGMAC_TX_CONFIG, v | (ss << XGMAC_TX_CONFIG_SS_SHIFT));

	MAC_WRITE(sc, XGMAC_Q_TX_FLOW_CTRL(0), sc->link_txpause ?
	    XGMAC_TX_FLOW_TFE | (TCX_PAUSE_TIME << XGMAC_TX_FLOW_PT_SHIFT) : 0);
	MAC_WRITE(sc, XGMAC_RX_FLOW_CTRL, sc->link_rxpause ?
	    XGMAC_RX_FLOW_RFE : 0);
}

/*
 * SerDes and PCS
 */

static void
tcx_delay(int us)
{
	if (cold || us < 1000)
		DELAY(us);
	else
		pause_sbt("tcxdly", ustosbt(us), 0, C_PREL(1));
}

static int
tcx_xpcs_read(struct tcx_softc *sc, int mmd, int reg)
{
	bus_size_t win;
	uint32_t csr;

	win = TC956X_XGMAC_BASE(sc->mac) + TC956X_XPCS_OFFSET;
	csr = (mmd << 16) | reg;
	SFR_WRITE(sc, win + TC956X_XPCS_VIEWPORT, csr >> 8);
	return (SFR_READ(sc, win + (csr & 0xff) * 4) & 0xffff);
}

static void
tcx_xpcs_write(struct tcx_softc *sc, int mmd, int reg, uint16_t val)
{
	bus_size_t win;
	uint32_t csr;

	win = TC956X_XGMAC_BASE(sc->mac) + TC956X_XPCS_OFFSET;
	csr = (mmd << 16) | reg;
	SFR_WRITE(sc, win + TC956X_XPCS_VIEWPORT, csr >> 8);
	SFR_WRITE(sc, win + (csr & 0xff) * 4, val);
}

static void
tcx_xpcs_update(struct tcx_softc *sc, int mmd, int reg, uint16_t clr,
    uint16_t set)
{
	tcx_xpcs_write(sc, mmd, reg,
	    (tcx_xpcs_read(sc, mmd, reg) & ~clr) | set);
}

static u_int
tcx_sp_sel_speed(uint32_t emacctl)
{
	switch (emacctl & TC956X_EMACCTL_SP_SEL_MASK) {
	case TC956X_EMACCTL_SP_2500:
		return (2500);
	case TC956X_EMACCTL_SP_1000:
		return (1000);
	case TC956X_EMACCTL_SP_100:
		return (100);
	case TC956X_EMACCTL_SP_10:
		return (10);
	default:
		return (0);
	}
}

/*
 * Restart the SerDes.  It takes its rate from the speed selector, so that
 * must be valid first: out of reset it holds 8, which is no SGMII rate,
 * and in-band autonegotiation then never completes.
 */
static int
tcx_pma_init(struct tcx_softc *sc, u_int speed)
{
	bus_size_t pma, emacctl;
	uint32_t sp_sel, v;
	int i;

	switch (speed) {
	case 2500:
		sp_sel = TC956X_EMACCTL_SP_2500;
		break;
	case 100:
		sp_sel = TC956X_EMACCTL_SP_100;
		break;
	case 10:
		sp_sel = TC956X_EMACCTL_SP_10;
		break;
	default:
		sp_sel = TC956X_EMACCTL_SP_1000;
		break;
	}
	emacctl = TC956X_NEMACCTL(sc->mac);
	v = SFR_READ(sc, emacctl);
	v &= ~(TC956X_EMACCTL_SP_SEL_MASK | TC956X_EMACCTL_PHY_INF_MASK |
	    TC956X_EMACCTL_INV_SGM_SIGDET);
	v |= sp_sel | TC956X_EMACCTL_PHY_INF_PHYCLK | TC956X_EMACCTL_LPIHWCLKEN;
	SFR_WRITE(sc, emacctl, v);

	/* The clock settings may only change with the PMA in reset. */
	tcx_sfr_update(sc, TC956X_NRSTCTRL(sc->mac), 0, TC956X_RST_PMA);
	pma = TC956X_XGMAC_BASE(sc->mac) + TC956X_PMA_OFFSET;
	SFR_WRITE(sc, pma + TC956X_PMA_CML_GL_PM_CFG0, 0);
	for (i = 0; i < TC956X_PMA_NLANES; i++) {
		SFR_WRITE(sc, pma + TC956X_PMA_HWT_REFCK_R_EN(i), 0);
		SFR_WRITE(sc, pma + TC956X_PMA_HWT_REFCK_TERM_EN(i), 0);
		SFR_WRITE(sc, pma + TC956X_PMA_COMM_CFG_0_1(i),
		    TC956X_PMA_COMM_CFG_REFCLK_I);
	}
	tcx_sfr_update(sc, TC956X_NRSTCTRL(sc->mac), TC956X_RST_PMA, 0);

	for (i = 0; i < TCX_PMA_TIMEOUT; i += 100) {
		if ((SFR_READ(sc, emacctl) & TC956X_EMACCTL_INIT_DONE) != 0)
			return (0);
		tcx_delay(100);
	}
	device_printf(sc->dev, "SerDes did not come up at %u Mb/s\n", speed);
	return (ETIMEDOUT);
}

/*
 * Set the XPCS up for 2500BASE-X, or for MAC-side SGMII with in-band
 * autonegotiation and automatic speed switching.
 */
static int
tcx_xpcs_config(struct tcx_softc *sc, bool sgmii)
{
	int bmcr, i;

	tcx_xpcs_write(sc, XPCS_MMD_VEND2, XPCS_MII_BMCR, XPCS_BMCR_RESET);
	for (i = 0; i < TCX_XPCS_RESET_TIMEOUT; i += 1000) {
		tcx_delay(1000);
		if ((tcx_xpcs_read(sc, XPCS_MMD_VEND2, XPCS_MII_BMCR) &
		    XPCS_BMCR_RESET) == 0)
			break;
	}
	if (i >= TCX_XPCS_RESET_TIMEOUT) {
		device_printf(sc->dev, "PCS reset timed out\n");
		return (ETIMEDOUT);
	}

	/*
	 * This PCS can do 10GBASE-R, which is its default and makes it
	 * ignore the mode bits.  A reserved type selects mode switching.
	 */
	if ((tcx_xpcs_read(sc, XPCS_MMD_PCS, XPCS_PCS_STAT2) &
	    XPCS_PCS_STAT2_10GBR) != 0)
		tcx_xpcs_write(sc, XPCS_MMD_PCS, XPCS_PCS_CTRL2,
		    XPCS_PCS_TYPE_SEL_MODAL);

	bmcr = tcx_xpcs_read(sc, XPCS_MMD_VEND2, XPCS_MII_BMCR);
	if (sgmii) {
		tcx_xpcs_write(sc, XPCS_MMD_VEND2, XPCS_MII_BMCR,
		    bmcr & ~XPCS_BMCR_ANENABLE);
		tcx_xpcs_update(sc, XPCS_MMD_VEND2, XPCS_VR_MII_AN_CTRL,
		    XPCS_AN_CTRL_PCS_MODE_MASK | XPCS_AN_CTRL_TX_CONFIG_PHY,
		    XPCS_AN_CTRL_PCS_MODE_SGMII);
		tcx_xpcs_update(sc, XPCS_MMD_VEND2, XPCS_VR_MII_DIG_CTRL1,
		    XPCS_DIG_CTRL1_2G5_EN, XPCS_DIG_CTRL1_MAC_AUTO_SW);
		tcx_xpcs_write(sc, XPCS_MMD_VEND2, XPCS_MII_BMCR,
		    bmcr | XPCS_BMCR_ANENABLE);
	} else {
		tcx_xpcs_update(sc, XPCS_MMD_VEND2, XPCS_VR_MII_DIG_CTRL1,
		    XPCS_DIG_CTRL1_MAC_AUTO_SW, XPCS_DIG_CTRL1_2G5_EN);
		tcx_xpcs_write(sc, XPCS_MMD_VEND2, XPCS_MII_BMCR,
		    (bmcr & ~(XPCS_BMCR_ANENABLE | XPCS_BMCR_SPEED100)) |
		    XPCS_BMCR_SPEED1000);
	}
	return (0);
}

/* Match the SerDes and PCS to the speed the PHY linked at. */
static int
tcx_serdes_config(struct tcx_softc *sc, u_int speed)
{
	int error;

	error = tcx_pma_init(sc, speed);
	if (error == 0 && (sc->serdes_speed == 0 ||
	    (sc->serdes_speed == 2500) != (speed == 2500)))
		error = tcx_xpcs_config(sc, speed != 2500);
	sc->serdes_speed = error == 0 ? speed : 0;
	return (error);
}

/*
 * Start a MAC from reset: clocks, then MAC reset, then the SerDes at a
 * valid rate, then the PCS.  The PHY's current speed is used if it has a
 * link; otherwise SGMII at 1G, which the next link change corrects.
 */
static int
tcx_mac_start(struct tcx_softc *sc)
{
	uint32_t clk;
	u_int speed;
	int error;

	/* Put the MAC, SerDes and PCS in reset, as Linux does. */
	tcx_sfr_update(sc, TC956X_NRSTCTRL(sc->mac), 0,
	    TC956X_RST_MAC | TC956X_RST_PMA | TC956X_RST_XPCS);

	clk = TC956X_CLK_MAC_TX | TC956X_CLK_MAC_RX | TC956X_CLK_MAC_ALL;
	if (sc->mac == 1)
		clk |= TC956X_CLK_MAC_RMII;
	tcx_sfr_update(sc, TC956X_NCLKCTRL(sc->mac), 0, clk);
	tcx_sfr_update(sc, TC956X_NRSTCTRL(sc->mac), TC956X_RST_MAC, 0);

	speed = (tcx_phy_poll(sc) && sc->link_up && sc->link_speed != 0) ?
	    sc->link_speed : 1000;
	error = tcx_pma_init(sc, speed);
	tcx_sfr_update(sc, TC956X_NRSTCTRL(sc->mac), TC956X_RST_XPCS, 0);
	if (error == 0)
		error = tcx_xpcs_config(sc, speed != 2500);
	sc->serdes_speed = error == 0 ? speed : 0;
	return (error);
}

/* Report the SerDes and PCS state, for debugging. */
static int
tcx_sysctl_serdes(SYSCTL_HANDLER_ARGS)
{
	struct tcx_softc *sc;
	char buf[256];

	sc = arg1;
	snprintf(buf, sizeof(buf), "EMACCTL 0x%08x, set for %u Mb/s; "
	    "PCS CTRL2 0x%04x, BMCR 0x%04x, BMSR 0x%04x, DIG_CTRL1 0x%04x, "
	    "AN_CTRL 0x%04x, AN_INTR_STS 0x%04x",
	    SFR_READ(sc, TC956X_NEMACCTL(sc->mac)), sc->serdes_speed,
	    tcx_xpcs_read(sc, XPCS_MMD_PCS, XPCS_PCS_CTRL2),
	    tcx_xpcs_read(sc, XPCS_MMD_VEND2, XPCS_MII_BMCR),
	    tcx_xpcs_read(sc, XPCS_MMD_VEND2, XPCS_MII_BMSR),
	    tcx_xpcs_read(sc, XPCS_MMD_VEND2, XPCS_VR_MII_DIG_CTRL1),
	    tcx_xpcs_read(sc, XPCS_MMD_VEND2, XPCS_VR_MII_AN_CTRL),
	    tcx_xpcs_read(sc, XPCS_MMD_VEND2, XPCS_VR_MII_AN_INTR_STS));
	return (sysctl_handle_string(oidp, buf, sizeof(buf), req));
}

static int
tcx_sysctl_rx_riwt(SYSCTL_HANDLER_ARGS)
{
	struct tcx_softc *sc;
	int error, v;

	sc = arg1;
	v = sc->rx_riwt;
	error = sysctl_handle_int(oidp, &v, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (v < 0 || v > XGMAC_DMA_CH_RWT_MASK)
		return (EINVAL);
	/*
	 * Descriptors already on the ring keep their setting, so when
	 * moderation is turned off the watchdog is left running for them;
	 * it does no harm once every frame interrupts.  The next init
	 * clears it.
	 */
	if (v != 0)
		MAC_WRITE(sc, XGMAC_DMA_CH_RX_WATCHDOG(0), v);
	sc->rx_riwt = v;
	return (0);
}

/* Writing 1 makes the next link check redo the SerDes and PCS setup. */
static int
tcx_sysctl_serdes_reset(SYSCTL_HANDLER_ARGS)
{
	struct tcx_softc *sc;
	int error, v;

	sc = arg1;
	v = 0;
	error = sysctl_handle_int(oidp, &v, 0, req);
	if (error != 0 || req->newptr == NULL)
		return (error);
	if (v != 0)
		sc->serdes_speed = 0;
	return (0);
}

/*
 * iflib attach and detach
 */

static int
tcx_attach_pre(if_ctx_t ctx)
{
	struct tcx_softc *sc;
	if_softc_ctx_t scctx;
	struct ether_addr ea;
	device_t dev;
	uint32_t hi, lo, v;
	int count, error, rid;

	sc = iflib_get_softc(ctx);
	dev = iflib_get_dev(ctx);
	scctx = iflib_get_softc_ctx(ctx);
	sc->ctx = ctx;
	sc->dev = dev;
	sc->scctx = scctx;
	sc->mac = pci_get_function(dev);
	if (sc->mac > 1)
		return (ENXIO);
	sc->rx_riwt = TCX_RX_RIWT_DEFAULT;
	sc->rx_coal_frames = TCX_RX_COAL_FRAMES_DEFAULT;
	sc->tx_coal_frames = TCX_TX_COAL_FRAMES_DEFAULT;

	pci_enable_busmaster(dev);

	rid = TC956X_BAR_BRIDGE;
	sc->bridge_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	rid = TC956X_BAR_SFR;
	sc->sfr_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->bridge_res == NULL || sc->sfr_res == NULL) {
		device_printf(dev, "cannot map registers\n");
		error = ENXIO;
		goto fail;
	}

	if (sc->mac == 0)
		tcx_chip_init(sc);
	else if (!tcx_msigen_running(sc)) {
		device_printf(dev, "function 0 has not set up the chip\n");
		error = ENXIO;
		goto fail;
	}

	/*
	 * The station address lives in an I2C EEPROM we cannot read yet.
	 * Keep the one firmware programmed, or make one up.  Read it before
	 * any MAC reset, which clears it, and only from a running MAC.
	 */
	hi = lo = 0;
	if (tcx_mac_running(sc)) {
		hi = MAC_READ(sc, XGMAC_ADDR_HIGH(0));
		lo = MAC_READ(sc, XGMAC_ADDR_LOW(0));
	}
	ea.octet[0] = lo;
	ea.octet[1] = lo >> 8;
	ea.octet[2] = lo >> 16;
	ea.octet[3] = lo >> 24;
	ea.octet[4] = hi;
	ea.octet[5] = hi >> 8;
	if ((hi & XGMAC_ADDR_HIGH_AE) == 0 || ETHER_IS_MULTICAST(ea.octet) ||
	    (lo == 0 && (hi & 0xffff) == 0))
		ether_gen_addr(iflib_get_ifp(ctx), &ea);
	iflib_set_mac(ctx, ea.octet);

	/*
	 * Keep a MAC that firmware left running, so that booting from the
	 * network does not drop the link; otherwise start it ourselves.
	 */
	v = SFR_READ(sc, TC956X_NEMACCTL(sc->mac));
	if (tcx_cold_init || !tcx_mac_running(sc) ||
	    (v & TC956X_EMACCTL_INIT_DONE) == 0) {
		if (tcx_mac_start(sc) != 0)
			device_printf(dev, "MAC %d did not start cleanly\n",
			    sc->mac);
	} else
		sc->serdes_speed = tcx_sp_sel_speed(v);

	v = MAC_READ(sc, XGMAC_VERSION);
	device_printf(dev, "revision 0x%02x, XGMAC 0x%02x, MAC %d\n",
	    SFR_READ(sc, TC956X_NCID) & TC956X_NCID_REV_MASK,
	    v & XGMAC_VERSION_SNPS_MASK, sc->mac);

	scctx->isc_txqsizes[0] = roundup2(scctx->isc_ntxd[0] *
	    sizeof(struct tcx_desc), PAGE_SIZE);
	scctx->isc_rxqsizes[0] = roundup2(scctx->isc_nrxd[0] *
	    sizeof(struct tcx_desc), PAGE_SIZE);
	scctx->isc_txd_size[0] = sizeof(struct tcx_desc);
	scctx->isc_rxd_size[0] = sizeof(struct tcx_desc);
	scctx->isc_tx_nsegments = TCX_TX_MAXSEGS;
	scctx->isc_tx_tso_segments_max = TCX_TX_MAXSEGS;
	scctx->isc_tx_tso_size_max = TCX_TSO_SIZE;
	scctx->isc_tx_tso_segsize_max = PAGE_SIZE;
	/* A TSO packet may need an MSS context and a split header buffer. */
	scctx->isc_tx_pad = 2;
	scctx->isc_ntxqsets_max = scctx->isc_ntxqsets = 1;
	scctx->isc_nrxqsets_max = scctx->isc_nrxqsets = 1;
	scctx->isc_capabilities = scctx->isc_capenable = IFCAP_VLAN_MTU |
	    IFCAP_HWCSUM | IFCAP_HWCSUM_IPV6 | IFCAP_TSO | IFCAP_JUMBO_MTU;
	scctx->isc_tx_csum_flags = CSUM_IP | CSUM_TCP | CSUM_UDP |
	    CSUM_IP6_TCP | CSUM_IP6_UDP | CSUM_IP_TSO | CSUM_IP6_TSO;
	scctx->isc_dma_width = TCX_DMA_WIDTH;
	scctx->isc_max_frame_size = ETHER_MAX_LEN + ETHER_VLAN_ENCAP_LEN;
	scctx->isc_txrx = &tcx_txrx;

	/*
	 * The function offers 32 MSI messages, and iflib only takes MSI
	 * when there is exactly one, so allocate it here.  Every interrupt
	 * source goes through the chip's MSI generator anyway.
	 */
	count = 1;
	error = pci_alloc_msi(dev, &count);
	if (error != 0 || count != 1) {
		device_printf(dev, "cannot allocate an MSI\n");
		if (error == 0) {
			pci_release_msi(dev);
			error = ENXIO;
		}
		goto fail;
	}
	scctx->isc_vectors = 1;
	scctx->isc_intr = IFLIB_INTR_MSI;

	return (0);

fail:
	tcx_detach(ctx);
	return (error);
}

/* Read a MAC register; arg2 is its offset, with bit 0 set for 64 bits. */
static int
tcx_sysctl_reg(SYSCTL_HANDLER_ARGS)
{
	struct tcx_softc *sc;
	uint64_t v;
	bus_size_t reg;

	sc = arg1;
	reg = arg2 & ~1;
	v = MAC_READ(sc, reg);
	if ((arg2 & 1) != 0)
		v |= (uint64_t)MAC_READ(sc, reg + 4) << 32;
	return (sysctl_handle_64(oidp, &v, 0, req));
}

static void
tcx_add_sysctls(struct tcx_softc *sc)
{
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid_list *list;
	static const struct {
		const char	*name;
		int		reg;
		const char	*descr;
	} regs[] = {
		{ "mmc_rx_frames", XGMAC_MMC_RX_PKT_GB | 1,
		    "Frames received, good and bad" },
		{ "mmc_rx_crc_errors", XGMAC_MMC_RX_CRC_ERR | 1,
		    "Frames received with a CRC error" },
		{ "mmc_rx_fifo_overflow", XGMAC_MMC_RX_FIFOOVER_PKT | 1,
		    "Frames dropped for RX FIFO overflow" },
		{ "mmc_rx_discard", XGMAC_MMC_RX_DISCARD_PKT_GB | 1,
		    "Frames discarded by the MAC" },
		{ "mtl_rxq0_missed", XGMAC_MTL_RXQ_MISSED(0),
		    "MTL RX queue 0 missed (31:16) and overflow (15:0), raw" },
	};
	int i;

	ctx = device_get_sysctl_ctx(sc->dev);
	list = SYSCTL_CHILDREN(device_get_sysctl_tree(sc->dev));
	for (i = 0; i < nitems(regs); i++)
		SYSCTL_ADD_PROC(ctx, list, OID_AUTO, regs[i].name,
		    CTLTYPE_U64 | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, regs[i].reg,
		    tcx_sysctl_reg, "QU", regs[i].descr);
	SYSCTL_ADD_ULONG(ctx, list, OID_AUTO, "rx_buf_unavail", CTLFLAG_RD,
	    &sc->stat_rbu, "RX DMA ran out of descriptors");
	SYSCTL_ADD_ULONG(ctx, list, OID_AUTO, "dma_bus_errors", CTLFLAG_RD,
	    &sc->stat_fbe, "Fatal DMA bus errors");
	SYSCTL_ADD_PROC(ctx, list, OID_AUTO, "rx_riwt",
	    CTLTYPE_INT | CTLFLAG_RWTUN | CTLFLAG_MPSAFE, sc, 0,
	    tcx_sysctl_rx_riwt, "I", "RX interrupt watchdog, in units of 256 "
	    "DMA clock cycles; 0 interrupts on every frame");
	SYSCTL_ADD_UINT(ctx, list, OID_AUTO, "rx_coal_frames", CTLFLAG_RWTUN,
	    &sc->rx_coal_frames, 0, "With rx_riwt set, also interrupt every "
	    "this many frames; 0 relies on the watchdog alone");
	SYSCTL_ADD_UINT(ctx, list, OID_AUTO, "tx_coal_frames", CTLFLAG_RWTUN,
	    &sc->tx_coal_frames, 0, "Ask for a TX completion interrupt at "
	    "most once per this many descriptors");
	SYSCTL_ADD_ULONG(ctx, list, OID_AUTO, "rx_errors", CTLFLAG_RD,
	    &sc->stat_rx_err, "Frames received with the error summary set");
	SYSCTL_ADD_U32(ctx, list, OID_AUTO, "rx_error_status", CTLFLAG_RD,
	    &sc->rx_err_des3, 0, "Descriptor status of the last such frame");
	SYSCTL_ADD_ULONG(ctx, list, OID_AUTO, "intr", CTLFLAG_RD,
	    &sc->stat_intr, "Interrupts taken");
	SYSCTL_ADD_ULONG(ctx, list, OID_AUTO, "intr_rx", CTLFLAG_RD,
	    &sc->stat_intr_rx, "Interrupts with receive completions");
	SYSCTL_ADD_ULONG(ctx, list, OID_AUTO, "intr_tx", CTLFLAG_RD,
	    &sc->stat_intr_tx, "Interrupts with transmit completions");
	SYSCTL_ADD_PROC(ctx, list, OID_AUTO, "serdes",
	    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    tcx_sysctl_serdes, "A", "SerDes and PCS state");
	SYSCTL_ADD_PROC(ctx, list, OID_AUTO, "serdes_reset",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE, sc, 0,
	    tcx_sysctl_serdes_reset, "I", "Redo the SerDes setup on the "
	    "next link check");
}

static int
tcx_attach_post(if_ctx_t ctx)
{
	struct tcx_softc *sc;

	sc = iflib_get_softc(ctx);
	tcx_add_sysctls(sc);
	sc->media = iflib_get_media(ctx);
	ifmedia_add(sc->media, IFM_ETHER | IFM_AUTO, 0, NULL);
	ifmedia_add(sc->media, IFM_ETHER | IFM_2500_T | IFM_FDX, 0, NULL);
	ifmedia_add(sc->media, IFM_ETHER | IFM_1000_T | IFM_FDX, 0, NULL);
	ifmedia_add(sc->media, IFM_ETHER | IFM_100_TX | IFM_FDX, 0, NULL);
	ifmedia_add(sc->media, IFM_ETHER | IFM_10_T | IFM_FDX, 0, NULL);
	ifmedia_set(sc->media, IFM_ETHER | IFM_AUTO);

	if (!tcx_phy_poll(sc)) {
		device_printf(sc->dev, "PHY at %d does not answer\n",
		    TC956X_PHY_ADDR);
		return (0);
	}
	tcx_phy_fix_advert(sc);
	return (0);
}

static int
tcx_detach(if_ctx_t ctx)
{
	struct tcx_softc *sc;

	sc = iflib_get_softc(ctx);
	/* The MSI and the chip-wide state are left to iflib and function 0. */
	if (sc->sfr_res != NULL) {
		bus_release_resource(sc->dev, SYS_RES_MEMORY, TC956X_BAR_SFR,
		    sc->sfr_res);
		sc->sfr_res = NULL;
	}
	if (sc->bridge_res != NULL) {
		bus_release_resource(sc->dev, SYS_RES_MEMORY,
		    TC956X_BAR_BRIDGE, sc->bridge_res);
		sc->bridge_res = NULL;
	}
	return (0);
}

static int
tcx_tx_queues_alloc(if_ctx_t ctx, caddr_t *vaddrs, uint64_t *paddrs,
    int ntxqs, int ntxqsets)
{
	struct tcx_softc *sc;

	sc = iflib_get_softc(ctx);
	MPASS(ntxqs == 1 && ntxqsets == 1);
	sc->txq.sc = sc;
	sc->txq.ring = (struct tcx_desc *)vaddrs[0];
	sc->txq.paddr = paddrs[0];
	return (0);
}

static int
tcx_rx_queues_alloc(if_ctx_t ctx, caddr_t *vaddrs, uint64_t *paddrs,
    int nrxqs, int nrxqsets)
{
	struct tcx_softc *sc;

	sc = iflib_get_softc(ctx);
	MPASS(nrxqs == 1 && nrxqsets == 1);
	sc->rxq.sc = sc;
	sc->rxq.ring = (struct tcx_desc *)vaddrs[0];
	sc->rxq.paddr = paddrs[0];
	return (0);
}

static void
tcx_queues_free(if_ctx_t ctx)
{
	struct tcx_softc *sc;

	sc = iflib_get_softc(ctx);
	sc->txq.ring = NULL;
	sc->rxq.ring = NULL;
}

/*
 * Start and stop
 */

/* Where the MAC sees host address pa, through the translation window. */
static inline uint64_t
tcx_dma_addr(bus_addr_t pa)
{
	return ((uint64_t)pa + TC956X_DMA_OFFSET);
}

static u_int
tcx_hash_maddr(void *arg, struct sockaddr_dl *sdl, u_int cnt)
{
	uint32_t *hash, crc, rev;
	int i;

	hash = arg;
	crc = ~ether_crc32_le(LLADDR(sdl), ETHER_ADDR_LEN);
	for (rev = 0, i = 0; i < 32; i++, crc >>= 1)
		rev = (rev << 1) | (crc & 1);
	rev >>= 32 - XGMAC_HASH_BITS_LOG2;
	hash[rev >> 5] |= 1u << (rev & 0x1f);
	return (1);
}

static void
tcx_set_filter(struct tcx_softc *sc, int flags)
{
	uint32_t hash[2], v;

	hash[0] = hash[1] = 0;
	v = XGMAC_FILTER_HPF;
	if ((flags & IFF_PROMISC) != 0)
		v |= XGMAC_FILTER_PR;
	if ((flags & IFF_ALLMULTI) != 0) {
		v |= XGMAC_FILTER_PM;
		hash[0] = hash[1] = 0xffffffff;
	} else {
		v |= XGMAC_FILTER_HMC;
		if_foreach_llmaddr(iflib_get_ifp(sc->ctx), tcx_hash_maddr,
		    hash);
	}
	MAC_WRITE(sc, XGMAC_HASH_TABLE(0), hash[0]);
	MAC_WRITE(sc, XGMAC_HASH_TABLE(1), hash[1]);
	MAC_WRITE(sc, XGMAC_PACKET_FILTER, v);
}

/*
 * The receive buffer size given to the DMA.  A frame larger than this,
 * such as a jumbo frame in 4KB clusters, spans several descriptors.
 */
static u_int
tcx_rx_bufsz(struct tcx_softc *sc)
{
	return (iflib_get_rx_mbuf_sz(sc->ctx));
}

/*
 * The MTL flow control thresholds, which count down from a full FIFO in
 * steps of 512 bytes after the first 1KB.
 */
static uint32_t
tcx_flow_thresholds(struct tcx_softc *sc)
{
	u_int headroom, rfa, rfd;

	headroom = MAX(TCX_FC_HEADROOM_MIN,
	    2 * sc->scctx->isc_max_frame_size);
	rfa = howmany(headroom - 1024, 512);
	rfd = rfa + TCX_FC_HYSTERESIS / 512;
	return ((rfa << XGMAC_MTL_RFA_SHIFT) | (rfd << XGMAC_MTL_RFD_SHIFT));
}

static void
tcx_init(if_ctx_t ctx)
{
	struct tcx_softc *sc;
	if_t ifp;
	const uint8_t *ea;
	uint64_t a;
	uint32_t bufsz, v;
	int i;

	sc = iflib_get_softc(ctx);
	ifp = iflib_get_ifp(ctx);

	/* Reset the DMA, MTL and MAC.  This does not touch the PCS or PMA. */
	MAC_WRITE(sc, XGMAC_DMA_MODE, XGMAC_DMA_MODE_SWR);
	for (i = 0; i < TCX_SWR_TIMEOUT; i += 10) {
		if ((MAC_READ(sc, XGMAC_DMA_MODE) & XGMAC_DMA_MODE_SWR) == 0)
			break;
		DELAY(10);
	}
	if (i >= TCX_SWR_TIMEOUT) {
		device_printf(sc->dev, "DMA reset timed out\n");
		iflib_init_failed(ctx);
		return;
	}

	/* DMA: bus behaviour as firmware and Linux set it. */
	MAC_WRITE(sc, XGMAC_DMA_SYSBUS_MODE,
	    (31u << XGMAC_SYSBUS_WR_OSR_SHIFT) |
	    (31u << XGMAC_SYSBUS_RD_OSR_SHIFT) | XGMAC_SYSBUS_EAME |
	    XGMAC_SYSBUS_BLEN4_128 | XGMAC_SYSBUS_UNDEF);
	MAC_WRITE(sc, XGMAC_TX_EDMA_CTRL, XGMAC_EDMA_PS_MAX);
	MAC_WRITE(sc, XGMAC_RX_EDMA_CTRL, XGMAC_EDMA_PS_MAX);
	/* Channel interrupts on their own lines, into the MSI generator. */
	v = MAC_READ(sc, XGMAC_DMA_MODE) & ~XGMAC_DMA_MODE_INTM_MASK;
	MAC_WRITE(sc, XGMAC_DMA_MODE, v | XGMAC_DMA_MODE_INTM_PERCH);

	/* MTL: one queue each way, store and forward. */
	MAC_WRITE(sc, XGMAC_MTL_TXQ_OPMODE(0), XGMAC_MTL_TSF |
	    XGMAC_MTL_TXQEN_ENABLED | XGMAC_MTL_QS(TCX_TX_FIFO_BYTES));
	MAC_WRITE(sc, XGMAC_MTL_TC_ETS_CONTROL(0), 0);
	MAC_WRITE(sc, XGMAC_MTL_RXQ_FLOW_CONTROL(0), tcx_flow_thresholds(sc));
	MAC_WRITE(sc, XGMAC_MTL_RXQ_OPMODE(0), XGMAC_MTL_RSF |
	    XGMAC_MTL_EHFC | XGMAC_MTL_QS(TCX_RX_FIFO_BYTES));

	/* MAC */
	ea = (const uint8_t *)if_getlladdr(ifp);
	MAC_WRITE(sc, XGMAC_ADDR_HIGH(0), XGMAC_ADDR_HIGH_AE |
	    (ea[5] << 8) | ea[4]);
	MAC_WRITE(sc, XGMAC_ADDR_LOW(0), ((uint32_t)ea[3] << 24) |
	    (ea[2] << 16) | (ea[1] << 8) | ea[0]);
	tcx_set_filter(sc, if_getflags(ifp));
	v = XGMAC_RX_CONFIG_ACS | XGMAC_RX_CONFIG_CST |
	    XGMAC_RX_CONFIG_GPSLCE | XGMAC_RX_CONFIG_WD |
	    (XGMAC_RX_CONFIG_GPSL_MAX << XGMAC_RX_CONFIG_GPSL_SHIFT);
	if ((if_getcapenable(ifp) & (IFCAP_RXCSUM | IFCAP_RXCSUM_IPV6)) != 0)
		v |= XGMAC_RX_CONFIG_IPC;
	MAC_WRITE(sc, XGMAC_RX_CONFIG, v);
	MAC_WRITE(sc, XGMAC_TX_CONFIG, XGMAC_TX_CONFIG_JD);
	tcx_phy_poll(sc);
	tcx_mac_set_speed(sc);
	MAC_WRITE(sc, XGMAC_RXQ_CTRL0, XGMAC_RXQ_EN_DCB);
	MAC_WRITE(sc, XGMAC_INT_EN, 0);

	/* DMA channel 0 */
	bufsz = tcx_rx_bufsz(sc);
	MAC_WRITE(sc, XGMAC_DMA_CH_CONTROL(0), XGMAC_DMA_CH_PBLX8);
	MAC_WRITE(sc, XGMAC_DMA_CH_TX_CONTROL(0),
	    (TCX_PBL << XGMAC_DMA_CH_PBL_SHIFT) | XGMAC_DMA_CH_TSE);
	MAC_WRITE(sc, XGMAC_DMA_CH_RX_CONTROL(0),
	    (TCX_PBL << XGMAC_DMA_CH_PBL_SHIFT) |
	    ((bufsz << XGMAC_DMA_CH_RBSZ_SHIFT) & XGMAC_DMA_CH_RBSZ_MASK));

	sc->txq.cidx = sc->txq.pidx = 0;
	sc->txq.tso_mss = 0;
	a = tcx_dma_addr(sc->txq.paddr);
	MAC_WRITE(sc, XGMAC_DMA_CH_TXDESC_HADDR(0), a >> 32);
	MAC_WRITE(sc, XGMAC_DMA_CH_TXDESC_LADDR(0), (uint32_t)a);
	MAC_WRITE(sc, XGMAC_DMA_CH_TXDESC_RING_LEN(0),
	    sc->scctx->isc_ntxd[0] - 1);
	MAC_WRITE(sc, XGMAC_DMA_CH_TXDESC_TAIL(0), (uint32_t)a);

	sc->rxq.pidx = 0;
	a = tcx_dma_addr(sc->rxq.paddr);
	MAC_WRITE(sc, XGMAC_DMA_CH_RXDESC_HADDR(0), a >> 32);
	MAC_WRITE(sc, XGMAC_DMA_CH_RXDESC_LADDR(0), (uint32_t)a);
	MAC_WRITE(sc, XGMAC_DMA_CH_RXDESC_RING_LEN(0),
	    (sc->scctx->isc_nrxd[0] - 1) | (3u << XGMAC_DMA_CH_OWRQ_SHIFT));
	MAC_WRITE(sc, XGMAC_DMA_CH_RXDESC_TAIL(0), (uint32_t)a);

	MAC_WRITE(sc, XGMAC_DMA_CH_RX_WATCHDOG(0),
	    sc->rx_riwt & XGMAC_DMA_CH_RWT_MASK);
	MAC_WRITE(sc, XGMAC_DMA_CH_STATUS(0), 0xffffffff);
	MAC_WRITE(sc, XGMAC_DMA_CH_INT_EN(0), XGMAC_DMA_CH_NIS |
	    XGMAC_DMA_CH_AIS | XGMAC_DMA_CH_FBE | XGMAC_DMA_CH_RI |
	    XGMAC_DMA_CH_TI);

	/*
	 * Start.  The receive ring is empty until iflib refills it, which
	 * moves the tail pointer and lets the RX DMA go.
	 */
	v = MAC_READ(sc, XGMAC_DMA_CH_TX_CONTROL(0));
	MAC_WRITE(sc, XGMAC_DMA_CH_TX_CONTROL(0), v | XGMAC_DMA_CH_TXST);
	v = MAC_READ(sc, XGMAC_TX_CONFIG);
	MAC_WRITE(sc, XGMAC_TX_CONFIG, v | XGMAC_TX_CONFIG_TE);
	v = MAC_READ(sc, XGMAC_DMA_CH_RX_CONTROL(0));
	MAC_WRITE(sc, XGMAC_DMA_CH_RX_CONTROL(0), v | XGMAC_DMA_CH_RXST);
	v = MAC_READ(sc, XGMAC_RX_CONFIG);
	MAC_WRITE(sc, XGMAC_RX_CONFIG, v | XGMAC_RX_CONFIG_RE);
}

static void
tcx_stop(if_ctx_t ctx)
{
	struct tcx_softc *sc;
	uint32_t v;

	sc = iflib_get_softc(ctx);
	MSI_WRITE(sc, TC956X_MSI_OUT_EN, 0);
	MAC_WRITE(sc, XGMAC_DMA_CH_INT_EN(0), 0);

	v = MAC_READ(sc, XGMAC_DMA_CH_TX_CONTROL(0));
	MAC_WRITE(sc, XGMAC_DMA_CH_TX_CONTROL(0), v & ~XGMAC_DMA_CH_TXST);
	v = MAC_READ(sc, XGMAC_TX_CONFIG);
	MAC_WRITE(sc, XGMAC_TX_CONFIG, v & ~XGMAC_TX_CONFIG_TE);
	v = MAC_READ(sc, XGMAC_RX_CONFIG);
	MAC_WRITE(sc, XGMAC_RX_CONFIG, v & ~XGMAC_RX_CONFIG_RE);
	v = MAC_READ(sc, XGMAC_DMA_CH_RX_CONTROL(0));
	MAC_WRITE(sc, XGMAC_DMA_CH_RX_CONTROL(0), v & ~XGMAC_DMA_CH_RXST);

	/* Let a transfer in flight finish before iflib frees the buffers. */
	DELAY(1000);
}

/*
 * Interrupts
 *
 * The DMA channel interrupts are level signals into the MSI generator.
 * It sends one MSI and then holds off until MASK_CLR is written, when it
 * sends another if a source is still asserted.
 */

static void
tcx_intr_enable(if_ctx_t ctx)
{
	struct tcx_softc *sc;

	sc = iflib_get_softc(ctx);
	MSI_WRITE(sc, TC956X_MSI_OUT_EN, (1u << TC956X_MSI_SRC_TX(0)) |
	    (1u << TC956X_MSI_SRC_RX(0)));
	MSI_WRITE(sc, TC956X_MSI_MASK_CLR, TC956X_MSI_MASK_CLR_ALL);
}

static void
tcx_intr_disable(if_ctx_t ctx)
{
	struct tcx_softc *sc;

	sc = iflib_get_softc(ctx);
	MSI_WRITE(sc, TC956X_MSI_OUT_EN, 0);
}

static int
tcx_intr(void *arg)
{
	struct tcx_softc *sc;
	uint32_t st;

	sc = arg;
	if (MSI_READ(sc, TC956X_MSI_INT_STS) == 0)
		return (FILTER_STRAY);

	MSI_WRITE(sc, TC956X_MSI_OUT_EN, 0);
	st = MAC_READ(sc, XGMAC_DMA_CH_STATUS(0));
	MAC_WRITE(sc, XGMAC_DMA_CH_STATUS(0), st);
	sc->stat_intr++;
	if ((st & XGMAC_DMA_CH_RI) != 0)
		sc->stat_intr_rx++;
	if ((st & XGMAC_DMA_CH_TI) != 0)
		sc->stat_intr_tx++;
	if ((st & XGMAC_DMA_CH_RBU) != 0)
		sc->stat_rbu++;
	if ((st & XGMAC_DMA_CH_FBE) != 0) {
		sc->stat_fbe++;
		device_printf(sc->dev, "DMA bus error, status 0x%08x\n", st);
	}

	/* iflib runs the queues and then calls tcx_intr_enable(). */
	return (FILTER_SCHEDULE_THREAD);
}

/*
 * Transmit
 */

/* Checksum insertion for the first descriptor of a packet. */
static uint32_t
tcx_tx_cic(uint32_t csum_flags)
{
	if ((csum_flags & (CSUM_TCP | CSUM_UDP | CSUM_IP6_TCP |
	    CSUM_IP6_UDP)) != 0)
		return (TDES3_CIC_FULL);
	if ((csum_flags & CSUM_IP) != 0)
		return (TDES3_CIC_IP);
	return (0);
}

static void
tcx_tx_desc(struct tcx_desc *d, uint64_t a, uint32_t des2, uint32_t des3)
{
	d->des0 = htole32((uint32_t)a);
	d->des1 = htole32((uint32_t)(a >> 32));
	d->des2 = htole32(des2);
	d->des3 = htole32(des3);
}

/*
 * A plain packet takes one descriptor per segment.  For TSO the first
 * descriptor must hold exactly the headers, so the first segment is split
 * if it carries payload too, and a context descriptor goes ahead of it
 * whenever the MSS changes.  isc_tx_pad reserves those two extra
 * descriptors.
 */
static int
tcx_txd_encap(void *arg, if_pkt_info_t pi)
{
	struct tcx_softc *sc;
	struct tcx_txq *q;
	bus_dma_segment_t *segs;
	uint64_t a;
	uint32_t des2, des3, first3, len;
	u_int hdrlen, ndesc, off;
	qidx_t pidx;
	int i, last, n;
	bool tso;

	sc = arg;
	q = &sc->txq;
	segs = pi->ipi_segs;
	n = sc->scctx->isc_ntxd[0];
	last = pi->ipi_nsegs - 1;
	pidx = pi->ipi_pidx;
	ndesc = 0;

	tso = (pi->ipi_csum_flags & (CSUM_IP_TSO | CSUM_IP6_TSO)) != 0;
	hdrlen = 0;
	if (tso) {
		hdrlen = pi->ipi_ehdrlen + pi->ipi_ip_hlen + pi->ipi_tcp_hlen;
		if (segs[0].ds_len < hdrlen)
			return (EFBIG);	/* iflib defragments and retries */
		if (pi->ipi_tso_segsz != q->tso_mss) {
			tcx_tx_desc(&q->ring[pidx], 0,
			    pi->ipi_tso_segsz & TDES2_MSS_MASK,
			    TDES3_OWN | TDES3_CTXT | TDES3_TCMSSV);
			q->tso_mss = pi->ipi_tso_segsz;
			pidx = (pidx + 1) % n;
			ndesc++;
		}
		first3 = TDES3_FD | TDES3_TSE |
		    ((pi->ipi_tcp_hlen / 4) << TDES3_THL_SHIFT) |
		    ((pi->ipi_len - hdrlen) & TDES3_TPL_MASK);
	} else
		first3 = TDES3_FD | tcx_tx_cic(pi->ipi_csum_flags) |
		    (pi->ipi_len & TDES3_FL_MASK);

	for (i = 0; i <= last; i++) {
		off = 0;
		do {
			a = tcx_dma_addr(segs[i].ds_addr + off);
			len = segs[i].ds_len - off;
			if (i == 0 && off == 0) {
				if (tso)
					len = hdrlen;
				des3 = TDES3_OWN | first3;
			} else
				des3 = TDES3_OWN | (tso ? 0 :
				    (pi->ipi_len & TDES3_FL_MASK));
			off += len;
			des2 = len & TDES2_B1L_MASK;
			if (i == last && off == segs[i].ds_len) {
				des3 |= TDES3_LD;
				sc->tx_since_ioc += ndesc + 1;
				if ((pi->ipi_flags & IPI_TX_INTR) != 0 &&
				    sc->tx_since_ioc >= sc->tx_coal_frames) {
					des2 |= TDES2_IOC;
					sc->tx_since_ioc = 0;
				}
			}
			tcx_tx_desc(&q->ring[pidx], a, des2, des3);
			pidx = (pidx + 1) % n;
			ndesc++;
		} while (off < segs[i].ds_len);
	}

	pi->ipi_new_pidx = pidx;
	q->pidx = pidx;
	return (0);
}

static void
tcx_txd_flush(void *arg, uint16_t qid, qidx_t pidx)
{
	struct tcx_softc *sc;

	sc = arg;
	/* The DMA works up to, but not including, the tail descriptor. */
	wmb();
	MAC_WRITE(sc, XGMAC_DMA_CH_TXDESC_TAIL(0),
	    (uint32_t)tcx_dma_addr(sc->txq.paddr +
	    pidx * sizeof(struct tcx_desc)));
}

static int
tcx_txd_credits_update(void *arg, uint16_t qid, bool clear)
{
	struct tcx_softc *sc;
	struct tcx_txq *q;
	qidx_t idx;
	int count, n;

	sc = arg;
	q = &sc->txq;
	n = sc->scctx->isc_ntxd[0];

	/* The DMA clears OWN in every descriptor it has finished with. */
	count = 0;
	for (idx = q->cidx; idx != q->pidx; idx = (idx + 1) % n) {
		if ((le32toh(q->ring[idx].des3) & TDES3_OWN) != 0)
			break;
		count++;
		if (!clear)
			return (1);
	}
	q->cidx = idx;
	return (count);
}

/*
 * Receive
 */

static int
tcx_rxd_available(void *arg, uint16_t qid, qidx_t idx, qidx_t budget)
{
	struct tcx_softc *sc;
	struct tcx_rxq *q;
	uint32_t des3;
	int count, n;

	sc = arg;
	q = &sc->rxq;
	n = sc->scctx->isc_nrxd[0];

	/* Descriptors from rxq.pidx on have not been handed to the DMA. */
	count = 0;
	for (; idx != q->pidx && count < budget; idx = (idx + 1) % n) {
		des3 = le32toh(q->ring[idx].des3);
		if ((des3 & RDES3_OWN) != 0)
			break;
		if ((des3 & RDES3_LD) != 0)
			count++;
	}
	return (count);
}

/*
 * Report what the MAC checked.  With checksum offload on, a bad IPv4
 * header or TCP/UDP checksum sets the error summary, as for other receive
 * errors.  Such frames are passed up unchecked, for the stack to judge,
 * rather than failing the whole receive as an error return would.
 */
static void
tcx_rx_csum(struct tcx_softc *sc, if_rxd_info_t ri, uint32_t des3)
{
	if ((des3 & RDES3_ES) != 0) {
		sc->stat_rx_err++;
		sc->rx_err_des3 = des3;
		return;
	}
	if ((if_getcapenable(ri->iri_ifp) &
	    (IFCAP_RXCSUM | IFCAP_RXCSUM_IPV6)) == 0)
		return;

	switch ((des3 & RDES3_L34T_MASK) >> RDES3_L34T_SHIFT) {
	case RDES3_L34T_IP4TCP:
	case RDES3_L34T_IP4UDP:
		if ((if_getcapenable(ri->iri_ifp) & IFCAP_RXCSUM) == 0)
			return;
		ri->iri_csum_flags = CSUM_IP_CHECKED | CSUM_IP_VALID;
		break;
	case RDES3_L34T_IP6TCP:
	case RDES3_L34T_IP6UDP:
		if ((if_getcapenable(ri->iri_ifp) & IFCAP_RXCSUM_IPV6) == 0)
			return;
		break;
	default:
		return;
	}
	ri->iri_csum_flags |= CSUM_DATA_VALID | CSUM_PSEUDO_HDR;
	ri->iri_csum_data = 0xffff;
}

static int
tcx_rxd_pkt_get(void *arg, if_rxd_info_t ri)
{
	struct tcx_softc *sc;
	struct tcx_rxq *q;
	uint32_t des3;
	u_int bufsz, len, pktlen;
	qidx_t idx;
	int i, n;

	sc = arg;
	q = &sc->rxq;
	n = sc->scctx->isc_nrxd[0];
	bufsz = tcx_rx_bufsz(sc);
	idx = ri->iri_cidx;
	len = 0;

	for (i = 0; i < IFLIB_MAX_RX_SEGS; i++) {
		des3 = le32toh(q->ring[idx].des3);
		ri->iri_frags[i].irf_flid = 0;
		ri->iri_frags[i].irf_idx = idx;
		idx = (idx + 1) % n;
		if ((des3 & RDES3_LD) != 0) {
			/* The last descriptor holds the whole length. */
			pktlen = des3 & RDES3_PL_MASK;
			ri->iri_frags[i].irf_len = pktlen - len;
			ri->iri_nfrags = i + 1;
			ri->iri_len = pktlen;
			tcx_rx_csum(sc, ri, des3);
			return (0);
		}
		ri->iri_frags[i].irf_len = bufsz;
		len += bufsz;
	}
	return (EBADMSG);
}

/* Does the next refilled receive descriptor ask for an interrupt? */
static bool
tcx_rx_want_ioc(struct tcx_softc *sc)
{
	if (sc->rx_riwt == 0)
		return (true);
	if (sc->rx_coal_frames == 0)
		return (false);
	if (++sc->rx_ioc_count < sc->rx_coal_frames)
		return (false);
	sc->rx_ioc_count = 0;
	return (true);
}

static void
tcx_rxd_refill(void *arg, if_rxd_update_t iru)
{
	struct tcx_softc *sc;
	struct tcx_desc *d;
	uint64_t a;
	qidx_t idx;
	int i, n;

	sc = arg;
	n = sc->scctx->isc_nrxd[0];
	idx = iru->iru_pidx;
	for (i = 0; i < iru->iru_count; i++) {
		d = &sc->rxq.ring[idx];
		a = tcx_dma_addr(iru->iru_paddrs[i]);
		d->des0 = htole32((uint32_t)a);
		d->des1 = htole32((uint32_t)(a >> 32));
		d->des2 = 0;
		d->des3 = htole32(RDES3_OWN |
		    (tcx_rx_want_ioc(sc) ? RDES3_IOC : 0));
		idx = (idx + 1) % n;
	}
}

static void
tcx_rxd_flush(void *arg, uint16_t qid, uint8_t flid, qidx_t pidx)
{
	struct tcx_softc *sc;

	sc = arg;
	sc->rxq.pidx = pidx;
	wmb();
	MAC_WRITE(sc, XGMAC_DMA_CH_RXDESC_TAIL(0),
	    (uint32_t)tcx_dma_addr(sc->rxq.paddr +
	    pidx * sizeof(struct tcx_desc)));
}

/*
 * Configuration and link state
 */

static void
tcx_multi_set(if_ctx_t ctx)
{
	struct tcx_softc *sc;

	sc = iflib_get_softc(ctx);
	tcx_set_filter(sc, if_getflags(iflib_get_ifp(ctx)));
}

static int
tcx_promisc_set(if_ctx_t ctx, int flags)
{
	struct tcx_softc *sc;

	sc = iflib_get_softc(ctx);
	tcx_set_filter(sc, flags);
	return (0);
}

static int
tcx_mtu_set(if_ctx_t ctx, uint32_t mtu)
{
	struct tcx_softc *sc;

	if (mtu > TCX_MAX_MTU)
		return (EINVAL);
	sc = iflib_get_softc(ctx);
	sc->scctx->isc_max_frame_size = mtu + ETHER_HDR_LEN + ETHER_CRC_LEN +
	    ETHER_VLAN_ENCAP_LEN;
	return (0);
}

static void
tcx_timer(if_ctx_t ctx, uint16_t qid)
{
	/* No link interrupt yet, so poll the PHY from the admin task. */
	if (qid == 0)
		iflib_admin_intr_deferred(ctx);
}

static void
tcx_update_admin_status(if_ctx_t ctx)
{
	struct tcx_softc *sc;
	int state;

	sc = iflib_get_softc(ctx);
	if (!tcx_phy_poll(sc))
		return;
	if (sc->link_up && sc->link_speed != 0 &&
	    sc->link_speed != sc->serdes_speed) {
		if (bootverbose)
			device_printf(sc->dev, "SerDes %u -> %u Mb/s\n",
			    sc->serdes_speed, sc->link_speed);
		tcx_serdes_config(sc, sc->link_speed);
		sc->link_reported = LINK_STATE_UNKNOWN;
	}

	/*
	 * Compare with what iflib was last told, not with the previous
	 * poll: attach and init poll the PHY too.
	 */
	state = sc->link_up ? LINK_STATE_UP : LINK_STATE_DOWN;
	if (state == sc->link_reported && (state == LINK_STATE_DOWN ||
	    (sc->link_speed == sc->speed_reported &&
	    sc->link_txpause == sc->txpause_reported &&
	    sc->link_rxpause == sc->rxpause_reported)))
		return;

	if (state != sc->link_reported)
		tcx_phy_serdes_fifo(sc, state == LINK_STATE_UP);
	sc->link_reported = state;
	sc->speed_reported = sc->link_speed;
	sc->txpause_reported = sc->link_txpause;
	sc->rxpause_reported = sc->link_rxpause;
	if (state == LINK_STATE_UP) {
		tcx_mac_set_speed(sc);
		iflib_link_state_change(ctx, LINK_STATE_UP,
		    IF_Mbps(sc->link_speed));
	} else
		iflib_link_state_change(ctx, LINK_STATE_DOWN, 0);
}

static void
tcx_media_status(if_ctx_t ctx, struct ifmediareq *ifmr)
{
	struct tcx_softc *sc;

	sc = iflib_get_softc(ctx);
	ifmr->ifm_status = IFM_AVALID;
	ifmr->ifm_active = IFM_ETHER;
	if (!sc->link_up) {
		ifmr->ifm_active |= IFM_NONE;
		return;
	}
	ifmr->ifm_status |= IFM_ACTIVE;
	switch (sc->link_speed) {
	case 10:
		ifmr->ifm_active |= IFM_10_T;
		break;
	case 100:
		ifmr->ifm_active |= IFM_100_TX;
		break;
	case 1000:
		ifmr->ifm_active |= IFM_1000_T;
		break;
	case 2500:
		ifmr->ifm_active |= IFM_2500_T;
		break;
	default:
		ifmr->ifm_active |= IFM_UNKNOWN;
		break;
	}
	ifmr->ifm_active |= sc->link_fdx ? IFM_FDX : IFM_HDX;
	if (sc->link_txpause)
		ifmr->ifm_active |= IFM_ETH_TXPAUSE;
	if (sc->link_rxpause)
		ifmr->ifm_active |= IFM_ETH_RXPAUSE;
}

static int
tcx_media_change(if_ctx_t ctx)
{
	struct tcx_softc *sc;

	sc = iflib_get_softc(ctx);
	if (IFM_TYPE(sc->media->ifm_media) != IFM_ETHER)
		return (EINVAL);
	return (tcx_phy_set_media(sc, IFM_SUBTYPE(sc->media->ifm_media)));
}

static uint64_t
tcx_get_counter(if_ctx_t ctx, ift_counter cnt)
{
	return (if_get_counter_default(iflib_get_ifp(ctx), cnt));
}
