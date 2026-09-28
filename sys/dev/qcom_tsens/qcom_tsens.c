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
 * Qualcomm TSENS v2 temperature sensor controller.
 *
 * Each controller has a TM block, which reports the last reading of each
 * sensor, and an SROT block, which holds its configuration.  Firmware
 * enables the sensors and selects readings in tenths of a degree Celsius;
 * this driver only reads them, exposes them as sysctls and powers the
 * system off if a sensor stays at or above a critical temperature.
 *
 * With a devicetree the controllers are described by "qcom,tsens-v2"
 * nodes.  Windows-on-Arm ACPI tables do not describe them at all (their
 * thermal zones read temperatures through Qualcomm's Windows power
 * driver), so under ACPI the controllers of a known SoC, recognized by the
 * _HID of that power driver's device, are added from a table.
 */

#include "opt_acpi.h"
#include "opt_platform.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/callout.h>
#include <sys/kernel.h>
#include <sys/limits.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/reboot.h>
#include <sys/rman.h>
#include <sys/sysctl.h>

#include <machine/bus.h>
#include <machine/resource.h>

#ifdef DEV_ACPI
#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>
#endif

#ifdef FDT
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#endif

/* SROT registers */
#define	TSENS_SROT_CTRL		0x04
#define	 TSENS_CTRL_EN		(1u << 0)
#define	 TSENS_CTRL_SENSOR_EN_SHIFT 3
#define	 TSENS_CTRL_SENSOR_EN_MASK 0xffff
#define	 TSENS_CTRL_RESULT_TEMP	(1u << 21)	/* readings in 0.1 C */

/* TM registers */
#define	TSENS_TM_STATUS(n)	(0xa0 + (n) * 4)
#define	 TSENS_STATUS_TEMP_BITS	12		/* signed */
#define	 TSENS_STATUS_VALID	(1u << 21)

#define	TSENS_TM_SIZE		0x200
#define	TSENS_SROT_SIZE		0x8
#define	TSENS_MAX_SENSORS	16
#define	TSENS_READ_TRIES	3

struct qcom_tsens_softc {
	device_t		dev;
	struct resource		*tm;
	struct resource		*srot;
	uint32_t		sensors;	/* mask of enabled sensors */
	struct callout		poll;
	int			hot_polls;
};

static SYSCTL_NODE(_hw, OID_AUTO, qcom_tsens, CTLFLAG_RD | CTLFLAG_MPSAFE, 0,
    "Qualcomm TSENS temperature sensors");

static int qcom_tsens_crit_temp = 110;
SYSCTL_INT(_hw_qcom_tsens, OID_AUTO, crit_temp, CTLFLAG_RWTUN,
    &qcom_tsens_crit_temp, 0,
    "Power off when a sensor reads this many degrees C or more; 0 disables");

static bool qcom_tsens_powering_off;

static int
qcom_tsens_read(struct qcom_tsens_softc *sc, int n, int *decic)
{
	uint32_t v;
	int i;

	for (i = 0; i < TSENS_READ_TRIES; i++) {
		v = bus_read_4(sc->tm, TSENS_TM_STATUS(n));
		if ((v & TSENS_STATUS_VALID) != 0) {
			*decic = (int32_t)(v << (32 - TSENS_STATUS_TEMP_BITS)) >>
			    (32 - TSENS_STATUS_TEMP_BITS);
			return (0);
		}
	}
	return (EAGAIN);
}

static int
qcom_tsens_sysctl_temp(SYSCTL_HANDLER_ARGS)
{
	struct qcom_tsens_softc *sc = arg1;
	int decic, error, val;

	error = qcom_tsens_read(sc, arg2, &decic);
	if (error != 0)
		return (error);
	val = decic + 2731;	/* deciKelvin, for the "IK" format */
	return (sysctl_handle_int(oidp, &val, 0, req));
}

static void
qcom_tsens_poll(void *arg)
{
	struct qcom_tsens_softc *sc = arg;
	int crit, decic, hottest, n, where;

	crit = qcom_tsens_crit_temp;
	if (crit <= 0 || qcom_tsens_powering_off)
		goto out;
	hottest = INT_MIN;
	where = -1;
	for (n = 0; n < TSENS_MAX_SENSORS; n++) {
		if ((sc->sensors & (1u << n)) == 0 ||
		    qcom_tsens_read(sc, n, &decic) != 0)
			continue;
		if (decic > hottest) {
			hottest = decic;
			where = n;
		}
	}
	/* Require two readings in a row, so a single glitch is ignored. */
	if (hottest < (int64_t)crit * 10) {
		sc->hot_polls = 0;
		goto out;
	}
	if (++sc->hot_polls < 2)
		goto out;
	qcom_tsens_powering_off = true;
	device_printf(sc->dev,
	    "sensor %d at %d.%d C, at or above %d C; powering off\n",
	    where, hottest / 10, hottest % 10, crit);
	shutdown_nice(RB_POWEROFF);
out:
	callout_schedule_sbt(&sc->poll, SBT_1S, SBT_1S / 2, 0);
}

/*
 * names[n] names sensor n in sysctl; sensors without a name are called
 * "sN".
 */
static int
qcom_tsens_attach_common(device_t dev, const char *const *names)
{
	struct qcom_tsens_softc *sc = device_get_softc(dev);
	struct sysctl_ctx_list *ctx;
	struct sysctl_oid_list *tree;
	char buf[16];
	uint32_t ctrl;
	int n, rid;

	sc->dev = dev;
	rid = 0;
	sc->tm = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	rid = 1;
	sc->srot = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
	    RF_ACTIVE);
	if (sc->tm == NULL || sc->srot == NULL) {
		device_printf(dev, "cannot map registers\n");
		goto fail;
	}

	ctrl = bus_read_4(sc->srot, TSENS_SROT_CTRL);
	if ((ctrl & TSENS_CTRL_EN) == 0 ||
	    (ctrl & TSENS_CTRL_RESULT_TEMP) == 0) {
		device_printf(dev,
		    "not set up by firmware (control 0x%08x)\n", ctrl);
		goto fail;
	}
	sc->sensors = (ctrl >> TSENS_CTRL_SENSOR_EN_SHIFT) &
	    TSENS_CTRL_SENSOR_EN_MASK;

	ctx = device_get_sysctl_ctx(dev);
	tree = SYSCTL_CHILDREN(device_get_sysctl_tree(dev));
	for (n = 0; n < TSENS_MAX_SENSORS; n++) {
		if ((sc->sensors & (1u << n)) == 0)
			continue;
		if (names != NULL && names[n] != NULL)
			strlcpy(buf, names[n], sizeof(buf));
		else
			snprintf(buf, sizeof(buf), "s%d", n);
		SYSCTL_ADD_PROC(ctx, tree, OID_AUTO, buf,
		    CTLTYPE_INT | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, n,
		    qcom_tsens_sysctl_temp, "IK", "Sensor temperature");
	}
	if (bootverbose)
		device_printf(dev, "%d sensors\n", bitcount32(sc->sensors));

	callout_init(&sc->poll, 1);
	/* Loose timing lets the controllers' polls share wakeups. */
	callout_reset_sbt(&sc->poll, SBT_1S, SBT_1S / 2, qcom_tsens_poll, sc,
	    0);
	return (0);
fail:
	if (sc->tm != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->tm);
	if (sc->srot != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY, 1, sc->srot);
	return (ENXIO);
}

static int
qcom_tsens_detach(device_t dev)
{
	struct qcom_tsens_softc *sc = device_get_softc(dev);

	callout_drain(&sc->poll);
	bus_release_resource(dev, SYS_RES_MEMORY, 0, sc->tm);
	bus_release_resource(dev, SYS_RES_MEMORY, 1, sc->srot);
	return (0);
}

#ifdef DEV_ACPI
struct qcom_tsens_acpi_hw {
	vm_paddr_t	tm;
	vm_paddr_t	srot;
	const char	*names[TSENS_MAX_SENSORS];
};

/* SC8280XP; sensor names follow the thermal zones of its devicetree. */
static const struct qcom_tsens_acpi_hw qcom_tsens_sc8280xp[] = {
	{ 0x0c263000, 0x0c222000, {
	    "aoss-0", "cpu0-0", "cpu1-0", "cpu2-0", "cpu3-0", "cpu4-0",
	    "cpu5-0", "cpu6-0", "cpu7-0", "cluster0", "nsp0-0", NULL,
	    "smss-0", "camss-0" } },
	{ 0x0c265000, 0x0c223000, {
	    "aoss-1", "cpu0-1", "cpu1-1", "cpu2-1", "cpu3-1", "cpu4-1",
	    "cpu5-1", "cpu6-1", "cpu7-1", "cluster1", "nsp0-2", NULL,
	    "smss-1", "camss-1", "pcie-0", "mem-0" } },
	{ 0x0c251000, 0x0c224000, {
	    "aoss-2", "gpuss-0", "gpuss-1", "gpuss-2", "gpuss-3", "pcie-1",
	    "mem-1", "audio", "video", "nsp0-1" } },
	{ 0x0c252000, 0x0c225000, {
	    "aoss-3", "gpuss-4", "gpuss-5", "gpuss-6", "gpuss-7" } },
};

static const struct {
	const char				*pep_hid;
	const struct qcom_tsens_acpi_hw		*hw;
	int					nhw;
} qcom_tsens_acpi_socs[] = {
	{ "QCOM0617", qcom_tsens_sc8280xp, nitems(qcom_tsens_sc8280xp) },
};

static void
qcom_tsens_acpi_identify(driver_t *driver, device_t parent)
{
	const struct qcom_tsens_acpi_hw *hw;
	ACPI_HANDLE pep;
	device_t child;
	int i, s;

	if (acpi_disabled("qcom_tsens") ||
	    ACPI_FAILURE(AcpiGetHandle(NULL, "\\_SB.PEP0", &pep)))
		return;
	for (s = 0; s < nitems(qcom_tsens_acpi_socs); s++)
		if (acpi_MatchHid(pep, qcom_tsens_acpi_socs[s].pep_hid) ==
		    ACPI_MATCHHID_HID)
			break;
	if (s == nitems(qcom_tsens_acpi_socs))
		return;

	for (i = 0; i < qcom_tsens_acpi_socs[s].nhw; i++) {
		if (device_find_child(parent, "qcom_tsens", i) != NULL)
			continue;
		hw = &qcom_tsens_acpi_socs[s].hw[i];
		child = BUS_ADD_CHILD(parent, 10, "qcom_tsens", i);
		if (child == NULL)
			continue;
		acpi_set_private(child, __DECONST(void *, hw));
		bus_set_resource(child, SYS_RES_MEMORY, 0, hw->tm,
		    TSENS_TM_SIZE);
		bus_set_resource(child, SYS_RES_MEMORY, 1, hw->srot,
		    TSENS_SROT_SIZE);
	}
}

static int
qcom_tsens_acpi_probe(device_t dev)
{
	/* Only the children added by qcom_tsens_acpi_identify(). */
	if (acpi_get_handle(dev) != NULL || acpi_get_private(dev) == NULL)
		return (ENXIO);
	device_set_desc(dev, "Qualcomm TSENS temperature sensors");
	return (BUS_PROBE_NOWILDCARD);
}

static int
qcom_tsens_acpi_attach(device_t dev)
{
	const struct qcom_tsens_acpi_hw *hw = acpi_get_private(dev);

	return (qcom_tsens_attach_common(dev, hw->names));
}

static device_method_t qcom_tsens_acpi_methods[] = {
	DEVMETHOD(device_identify,	qcom_tsens_acpi_identify),
	DEVMETHOD(device_probe,		qcom_tsens_acpi_probe),
	DEVMETHOD(device_attach,	qcom_tsens_acpi_attach),
	DEVMETHOD(device_detach,	qcom_tsens_detach),
	DEVMETHOD_END
};

static driver_t qcom_tsens_acpi_driver = {
	"qcom_tsens",
	qcom_tsens_acpi_methods,
	sizeof(struct qcom_tsens_softc),
};

DRIVER_MODULE(qcom_tsens_acpi, acpi, qcom_tsens_acpi_driver, 0, 0);
#endif /* DEV_ACPI */

#ifdef FDT
static struct ofw_compat_data qcom_tsens_compat[] = {
	{ "qcom,tsens-v2",	1 },
	{ NULL,			0 }
};

static int
qcom_tsens_fdt_probe(device_t dev)
{
	if (!ofw_bus_status_okay(dev) ||
	    ofw_bus_search_compatible(dev, qcom_tsens_compat)->ocd_data == 0)
		return (ENXIO);
	device_set_desc(dev, "Qualcomm TSENS temperature sensors");
	return (BUS_PROBE_DEFAULT);
}

/*
 * Name each sensor after the thermal zone that refers to it, less the
 * "-thermal" suffix.
 */
static void
qcom_tsens_fdt_names(device_t dev, char **names)
{
	phandle_t xref, zone, zones;
	pcell_t *cells;
	char *name, *p;
	ssize_t ncells;

	zones = OF_finddevice("/thermal-zones");
	if (zones == -1)
		return;
	xref = OF_xref_from_node(ofw_bus_get_node(dev));
	for (zone = OF_child(zones); zone != 0; zone = OF_peer(zone)) {
		ncells = OF_getencprop_alloc_multi(zone, "thermal-sensors",
		    sizeof(*cells), (void **)&cells);
		if (ncells < 2) {
			OF_prop_free(cells);
			continue;
		}
		if (cells[0] == xref && cells[1] < TSENS_MAX_SENSORS &&
		    names[cells[1]] == NULL &&
		    OF_getprop_alloc(zone, "name", (void **)&name) > 0) {
			if ((p = strstr(name, "-thermal")) != NULL)
				*p = '\0';
			names[cells[1]] = strdup(name, M_DEVBUF);
			OF_prop_free(name);
		}
		OF_prop_free(cells);
	}
}

static int
qcom_tsens_fdt_attach(device_t dev)
{
	char *names[TSENS_MAX_SENSORS] = { NULL };
	int error, n;

	qcom_tsens_fdt_names(dev, names);
	error = qcom_tsens_attach_common(dev, (const char *const *)names);
	for (n = 0; n < TSENS_MAX_SENSORS; n++)
		free(names[n], M_DEVBUF);
	return (error);
}

static device_method_t qcom_tsens_fdt_methods[] = {
	DEVMETHOD(device_probe,		qcom_tsens_fdt_probe),
	DEVMETHOD(device_attach,	qcom_tsens_fdt_attach),
	DEVMETHOD(device_detach,	qcom_tsens_detach),
	DEVMETHOD_END
};

static driver_t qcom_tsens_fdt_driver = {
	"qcom_tsens",
	qcom_tsens_fdt_methods,
	sizeof(struct qcom_tsens_softc),
};

DRIVER_MODULE(qcom_tsens_fdt, simplebus, qcom_tsens_fdt_driver, 0, 0);
#endif /* FDT */

MODULE_VERSION(qcom_tsens, 1);
