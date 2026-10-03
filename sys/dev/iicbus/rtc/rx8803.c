/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2020 Alstom Group.
 * Copyright (c) 2020 Semihalf.
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <sys/cdefs.h>
#include "opt_platform.h"

#include "opt_acpi.h"
#include "opt_platform.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/clock.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>

#ifdef FDT
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#endif

#ifdef DEV_ACPI
#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>
#endif

#include <dev/iicbus/iiconf.h>
#include <dev/iicbus/iicbus.h>

#include "clock_if.h"
#include "iicbus_if.h"

#define	BIT(x)			(1 << (x))

#define RX8803_TIME		0x0
#define RX8803_FLAGS		0xE
#define RX8803_CTRL		0xF

#define RX8803_FLAGS_V1F	BIT(0)
#define RX8803_FLAGS_V2F	BIT(1)

#define RX8803_CTRL_DISABLE	BIT(0)

#define HALF_OF_SEC_NS		500000000
#define MAX_WRITE_LEN		16

struct rx8803_time {
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint8_t dow;
	uint8_t day;
	uint8_t mon;
	uint8_t year;
};

#ifdef FDT
static struct ofw_compat_data compat_data[] = {
	{"epson,rx8803", 1},
	{NULL,           0},
};
#endif

#ifdef DEV_ACPI
/* The RX8900 keeps time in the same registers. */
static char *rx8803_acpi_ids[] = {
	"RX008900",		/* CIX Sky1 boards */
	NULL
};
#endif


/*
 * Register reads and writes at the device's bus address: ACPI gives it as
 * 7 bits (acpi_iicbus), devicetree as the 8 FreeBSD's I2C drivers take.
 */
static uint16_t
rx8803_slave(device_t dev)
{
#ifdef DEV_ACPI
	if (acpi_get_handle(dev) != NULL)
		return (iicbus_get_addr(dev) << 1);
#endif
	return (iicbus_get_addr(dev));
}

static int
rx8803_readfrom(device_t dev, uint8_t reg, void *buf, uint16_t len)
{
	struct iic_msg msgs[2] = {
		{ rx8803_slave(dev), IIC_M_WR | IIC_M_NOSTOP, 1, &reg },
		{ rx8803_slave(dev), IIC_M_RD, len, buf },
	};

	return (iicbus_transfer_excl(dev, msgs, nitems(msgs), IIC_WAIT));
}

static int
rx8803_writeto(device_t dev, uint8_t reg, const void *buf, uint16_t len)
{
	uint8_t data[1 + 16];
	struct iic_msg msg = { rx8803_slave(dev), IIC_M_WR, len + 1, data };

	if (len > sizeof(data) - 1)
		return (IIC_EOVERFLOW);
	data[0] = reg;
	memcpy(&data[1], buf, len);
	return (iicbus_transfer_excl(dev, &msg, 1, IIC_WAIT));
}

static int rx8803_probe(device_t dev);
static int rx8803_attach(device_t dev);
static int rx8803_detach(device_t dev);

static int rx8803_gettime(device_t dev, struct timespec *ts);
static int rx8803_settime(device_t dev, struct timespec *ts);

static int rx8803_check_status(device_t dev);

static int
rx8803_check_status(device_t dev)
{
	uint8_t flags;
	int rc;

	rc = rx8803_readfrom(dev, RX8803_FLAGS, &flags, 1);
	if (rc != 0)
		return (rc);

	if (flags & RX8803_FLAGS_V2F) {
		device_printf(dev, "Low voltage flag set, date is incorrect\n");
		return (ENXIO);
	}

	return (0);
}

static int
rx8803_gettime(device_t dev, struct timespec *ts)
{
	struct rx8803_time data;
	struct bcd_clocktime bcd;
	int rc;

	rc = rx8803_check_status(dev);
	if (rc != 0)
		return (rc);

	rc = rx8803_readfrom(dev,
	    RX8803_TIME,
	    &data, sizeof(struct rx8803_time));
	if (rc != 0)
		return (rc);

	bcd.nsec = 0;
	bcd.sec = data.sec & 0x7F;
	bcd.min = data.min & 0x7F;
	bcd.hour = data.hour & 0x3F;
	bcd.dow = flsl(data.dow & 0x7F) - 1;
	bcd.day = data.day & 0x3F;
	bcd.mon = (data.mon & 0x1F);
	bcd.year = data.year;

	clock_dbgprint_bcd(dev, CLOCK_DBG_READ, &bcd);

	rc = clock_bcd_to_ts(&bcd, ts, false);
	return (rc);
}

static int
rx8803_settime(device_t dev, struct timespec *ts)
{
	struct rx8803_time data;
	struct bcd_clocktime bcd;
	uint8_t reg;
	int rc;

	ts->tv_sec -= utc_offset();
	clock_ts_to_bcd(ts, &bcd, false);
	clock_dbgprint_bcd(dev, CLOCK_DBG_WRITE, &bcd);

	data.sec = bcd.sec;
	data.min = bcd.min;
	data.hour = bcd.hour;
	data.dow = 1 << bcd.dow;
	data.day = bcd.day;
	data.mon = bcd.mon;
	data.year = bcd.year;

	if (ts->tv_nsec > HALF_OF_SEC_NS)
		data.sec++;

	/* First disable clock. */
	rc = rx8803_readfrom(dev, RX8803_CTRL, &reg, sizeof(uint8_t));
	if (rc != 0)
		return (rc);

	reg |= RX8803_CTRL_DISABLE;

	rc = rx8803_writeto(dev, RX8803_CTRL, &reg, sizeof(uint8_t));
	if (rc != 0)
		return (rc);

	/* Update the date. */
	rc = rx8803_writeto(dev,
	    RX8803_TIME,
	    &data, sizeof(struct rx8803_time));
	if (rc != 0)
		return (rc);

	/* Now restart it. */
	reg &= ~RX8803_CTRL_DISABLE;
	rc = rx8803_writeto(dev, RX8803_CTRL, &reg, sizeof(uint8_t));
	if (rc != 0)
		return (rc);

	/* Clear low voltage flags, as we have just updated the clock. */
	rc = rx8803_readfrom(dev, RX8803_FLAGS, &reg, sizeof(uint8_t));
	if (rc != 0)
		return (rc);

	reg &= ~(RX8803_FLAGS_V1F | RX8803_FLAGS_V2F);
	rc = rx8803_writeto(dev, RX8803_FLAGS, &reg, sizeof(uint8_t));
	return (rc);
}

static int
rx8803_probe(device_t dev)
{
#ifdef DEV_ACPI
	ACPI_HANDLE handle;
	int i;
#endif

#ifdef FDT
	if (ofw_bus_status_okay(dev) &&
	    ofw_bus_search_compatible(dev, compat_data)->ocd_data != 0) {
		device_set_desc(dev, "Epson RX8803 Real Time Clock");
		return (BUS_PROBE_GENERIC);
	}
#endif
#ifdef DEV_ACPI
	if ((handle = acpi_get_handle(dev)) != NULL) {
		for (i = 0; rx8803_acpi_ids[i] != NULL; i++) {
			if (!acpi_MatchHid(handle, rx8803_acpi_ids[i]))
				continue;
			device_set_desc(dev, "Epson RX8900 Real Time Clock");
			return (BUS_PROBE_DEFAULT);
		}
	}
#endif
	return (ENXIO);
}

static int
rx8803_attach(device_t dev)
{

	/* Set 1 sec resolution. */
	clock_register_flags(dev, 1000000, 0);
	clock_schedule(dev, 1);

	return (0);

}

static int
rx8803_detach(device_t dev)
{

	clock_unregister(dev);

	return (0);
}

static device_method_t rx8803_methods[] = {
	DEVMETHOD(device_probe, rx8803_probe),
	DEVMETHOD(device_attach, rx8803_attach),
	DEVMETHOD(device_detach, rx8803_detach),

	DEVMETHOD(clock_gettime, rx8803_gettime),
	DEVMETHOD(clock_settime, rx8803_settime),

	DEVMETHOD_END,
};

static driver_t rx8803_driver = {
	"rx8803",
	rx8803_methods,
	0,			/* We don't need softc for this one. */
};

DRIVER_MODULE(rx8803, iicbus, rx8803_driver, NULL, NULL);
MODULE_VERSION(rx8803, 1);
MODULE_DEPEND(rx8803, iicbus, IICBUS_MINVER, IICBUS_PREFVER, IICBUS_MAXVER);
#ifdef FDT
IICBUS_FDT_PNP_INFO(compat_data);
#endif
#ifdef DEV_ACPI
IICBUS_ACPI_PNP_INFO(rx8803_acpi_ids);
#endif
