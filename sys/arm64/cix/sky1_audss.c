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
 * The CIX Sky1 audio subsystem (ACPI ADSS.ACLK, CIXH6061): powered up for the
 * HDA controller and the other audio devices, whose clock gates and reset
 * lines (the ACRU block, ACPI CIXHA018) it then provides.
 *
 * Up, in order, before any of its registers is touched (a register of a
 * powered-off block hangs the SoC):
 * - D0, whose power resource (PPRS) powers the subsystem and repairs its
 *   memories;
 * - its six SCMI clocks (CLKT audio_clk0-5), at their rates;
 * - its interconnect out of reset (RSTL "noc", RST0);
 * - the subsystem's address remap and bus timeout (its RCSU).
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/bus.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include <arm64/cix/sky1_audss.h>
#include <arm64/cix/sky1_scmi.h>

/* ACRU registers */
#define	ACRU_CLK_GATE		0x10
#define	ACRU_SW_RST		0x78	/* active low */

/* The subsystem's RCSU */
#define	AUDSS_RCSU_PA		0x07000000UL
#define	AUDSS_RCSU_SIZE		0x2000
#define	AUDSS_RCSU_REMAP	0x34
#define	AUDSS_RCSU_REMAP_VAL	0x20000000
#define	AUDSS_RCSU_TIMEOUT	0x1000
#define	AUDSS_RCSU_TIMEOUT_EN	(1U << 31)
#define	AUDSS_RCSU_TIMEOUT_VAL	0x78

/*
 * The interconnect's reset, RSTL "noc": RST0 line 0x1F, bit 1 of its
 * register 0x404, active low.
 */
#define	RST0_PA			0x16000000UL
#define	RST0_NOC_LINE		0x1f
#define	RST0_NOC_REG		0x404
#define	RST0_NOC_BIT		(1U << 1)

static const struct {
	const char	*name;
	uint64_t	 hz;
} sky1_audss_clks[] = {
	{ "audio_clk0", 294912000 },
	{ "audio_clk1", 344064000 },
	{ "audio_clk2", 270950400 },
	{ "audio_clk3", 316108800 },
	{ "audio_clk4", 800000000 },
	{ "audio_clk5",  48000000 },
};

struct sky1_audss_softc {
	device_t	 dev;
	struct mtx	 mtx;
	volatile uint32_t *acru;
	vm_size_t	 acru_size;
};

static struct sky1_audss_softc *sky1_audss_sc;

static char *sky1_audss_ids[] = { "CIXH6061", NULL };

/* An entry of a CLKT (id, name, device) or RSTL (controller, line, device,
 * name) package, by name: its integer element. */
static int
sky1_audss_table(device_t dev, const char *table, int name_idx, int val_idx,
    const char *name, uint32_t *val)
{
	ACPI_BUFFER buf;
	ACPI_OBJECT *pkg, *e;
	int error;
	u_int i;

	buf.Pointer = NULL;
	buf.Length = ACPI_ALLOCATE_BUFFER;
	if (ACPI_FAILURE(AcpiEvaluateObject(acpi_get_handle(dev),
	    __DECONST(char *, table), NULL, &buf)))
		return (ENOENT);
	pkg = buf.Pointer;
	error = ENOENT;
	for (i = 0; pkg->Type == ACPI_TYPE_PACKAGE && i < pkg->Package.Count;
	    i++) {
		e = &pkg->Package.Elements[i];
		if (e->Type != ACPI_TYPE_PACKAGE ||
		    e->Package.Count <= (u_int)MAX(name_idx, val_idx) ||
		    e->Package.Elements[val_idx].Type != ACPI_TYPE_INTEGER ||
		    e->Package.Elements[name_idx].Type != ACPI_TYPE_STRING ||
		    strcmp(e->Package.Elements[name_idx].String.Pointer,
		    name) != 0)
			continue;
		*val = e->Package.Elements[val_idx].Integer.Value;
		error = 0;
		break;
	}
	AcpiOsFree(buf.Pointer);
	return (error);
}

/* The ACRU block, through the _DSD audss_cru reference. */
static int
sky1_audss_map_acru(struct sky1_audss_softc *sc)
{
	const ACPI_OBJECT *obj;
	device_t acru;
	rman_res_t start, count;

	if (ACPI_FAILURE(acpi_GetProperty(sc->dev, "audss_cru", &obj)) ||
	    obj->Type != ACPI_TYPE_LOCAL_REFERENCE ||
	    (acru = acpi_get_device(obj->Reference.Handle)) == NULL ||
	    bus_get_resource(acru, SYS_RES_MEMORY, 0, &start, &count) != 0)
		return (ENXIO);
	sc->acru = pmap_mapdev(start, count);
	sc->acru_size = count;
	return (0);
}

static void
sky1_audss_noc_reset(void)
{
	volatile uint32_t *rst0;
	uint32_t v;

	rst0 = pmap_mapdev(RST0_PA, PAGE_SIZE);
	v = rst0[RST0_NOC_REG / 4];
	rst0[RST0_NOC_REG / 4] = v & ~RST0_NOC_BIT;
	DELAY(2);
	rst0[RST0_NOC_REG / 4] = v | RST0_NOC_BIT;
	pmap_unmapdev(__DEVOLATILE(void *, rst0), PAGE_SIZE);
}

static void
sky1_audss_rcsu(void)
{
	volatile uint32_t *rcsu;

	rcsu = pmap_mapdev(AUDSS_RCSU_PA, AUDSS_RCSU_SIZE);
	rcsu[AUDSS_RCSU_REMAP / 4] = AUDSS_RCSU_REMAP_VAL;
	rcsu[AUDSS_RCSU_TIMEOUT / 4] = AUDSS_RCSU_TIMEOUT_EN |
	    AUDSS_RCSU_TIMEOUT_VAL;
	pmap_unmapdev(__DEVOLATILE(void *, rcsu), AUDSS_RCSU_SIZE);
}

static int
sky1_audss_probe(device_t dev)
{
	int rv;

	rv = ACPI_ID_PROBE(device_get_parent(dev), dev, sky1_audss_ids, NULL);
	if (rv <= 0)
		device_set_desc(dev, "CIX Sky1 audio subsystem");
	return (rv);
}

static int
sky1_audss_attach(device_t dev)
{
	struct sky1_audss_softc *sc;
	ACPI_STATUS status;
	uint32_t id, line;
	uint64_t hz;
	int error;
	u_int i;

	sc = device_get_softc(dev);
	sc->dev = dev;
	mtx_init(&sc->mtx, "sky1 audss", NULL, MTX_SPIN);

	/* Everything it needs, before powering it up. */
	if ((error = sky1_audss_map_acru(sc)) != 0) {
		device_printf(dev, "no audss_cru\n");
		goto fail;
	}
	if (sky1_audss_table(dev, "RSTL", 3, 1, "noc", &line) != 0 ||
	    line != RST0_NOC_LINE) {
		device_printf(dev, "unexpected RSTL noc line\n");
		error = ENXIO;
		goto fail;
	}

	status = acpi_pwr_switch_consumer(acpi_get_handle(dev), ACPI_STATE_D0);
	if (ACPI_FAILURE(status)) {
		device_printf(dev, "cannot power up: %s\n",
		    AcpiFormatException(status));
		error = ENXIO;
		goto fail;
	}
	for (i = 0; i < nitems(sky1_audss_clks); i++) {
		if (sky1_audss_table(dev, "CLKT", 1, 0, sky1_audss_clks[i].name,
		    &id) != 0) {
			device_printf(dev, "no clock %s\n",
			    sky1_audss_clks[i].name);
			error = ENXIO;
			goto fail;
		}
		if ((error = sky1_scmi_clk_enable(id, true)) != 0 ||
		    (error = sky1_scmi_clk_set_rate(id,
		    sky1_audss_clks[i].hz)) != 0) {
			device_printf(dev, "clock %s (%#x): %d\n",
			    sky1_audss_clks[i].name, id, error);
			goto fail;
		}
		if (bootverbose && sky1_scmi_clk_get_rate(id, &hz) == 0)
			device_printf(dev, "%s: %ju Hz\n",
			    sky1_audss_clks[i].name, (uintmax_t)hz);
	}
	sky1_audss_noc_reset();
	sky1_audss_rcsu();

	sky1_audss_sc = sc;
	return (0);

fail:
	if (sc->acru != NULL)
		pmap_unmapdev(__DEVOLATILE(void *, sc->acru), sc->acru_size);
	mtx_destroy(&sc->mtx);
	return (error);
}

static int
sky1_audss_update(u_int reg, u_int bit, bool set)
{
	struct sky1_audss_softc *sc = sky1_audss_sc;
	uint32_t v;

	if (sc == NULL)
		return (ENXIO);
	if (bit >= 32)
		return (EINVAL);
	mtx_lock_spin(&sc->mtx);
	v = sc->acru[reg / 4];
	sc->acru[reg / 4] = set ? v | (1U << bit) : v & ~(1U << bit);
	mtx_unlock_spin(&sc->mtx);
	return (0);
}

int
sky1_audss_gate(u_int bit, bool on)
{
	return (sky1_audss_update(ACRU_CLK_GATE, bit, on));
}

int
sky1_audss_reset(u_int bit, bool assert)
{
	return (sky1_audss_update(ACRU_SW_RST, bit, !assert));
}

static device_method_t sky1_audss_methods[] = {
	DEVMETHOD(device_probe,		sky1_audss_probe),
	DEVMETHOD(device_attach,	sky1_audss_attach),
	DEVMETHOD_END
};

static driver_t sky1_audss_driver = {
	"sky1_audss",
	sky1_audss_methods,
	sizeof(struct sky1_audss_softc),
};

/* After SCMI (BUS_PASS_INTERRUPT), before the audio devices. */
EARLY_DRIVER_MODULE(sky1_audss, acpi, sky1_audss_driver, 0, 0,
    BUS_PASS_SUPPORTDEV + BUS_PASS_ORDER_MIDDLE);
MODULE_VERSION(sky1_audss, 1);
