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
 * The Cadence GPIO controller, as CIX Sky1's firmware describes it in ACPI
 * (CIXH1002, CIXH1003): up to 32 pins a controller.
 *
 * A pin the firmware routes to a peripheral (its bypass bit set) or drives
 * is left as it is until it is configured here: attaching changes nothing,
 * as the firmware's outputs (a converter's power-down line, say) matter.
 * Configuring a pin takes it from its peripheral.  No interrupts yet.
 */

#include "opt_acpi.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/gpio.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include <dev/gpio/gpiobusvar.h>

#ifdef __aarch64__
#include <arm64/cix/sky1_scmi.h>
#endif

#include "gpio_if.h"

#define	CDNS_GPIO_BYPASS_MODE		0x00	/* 1: the peripheral's */
#define	CDNS_GPIO_DIRECTION_MODE	0x04	/* 1: input */
#define	CDNS_GPIO_OUTPUT_EN		0x08
#define	CDNS_GPIO_OUTPUT_VALUE		0x0c
#define	CDNS_GPIO_INPUT_VALUE		0x10
#define	CDNS_GPIO_IRQ_MASK		0x14

#define	CDNS_GPIO_CAPS			(GPIO_PIN_INPUT | GPIO_PIN_OUTPUT)

struct cdns_gpio_softc {
	device_t		 dev;
	device_t		 busdev;
	struct mtx		 mtx;
	struct resource		*mem;
	int			 npins;
	int			 uid;
};

#define	RD4(sc, r)		bus_read_4((sc)->mem, (r))
#define	WR4(sc, r, v)		bus_write_4((sc)->mem, (r), (v))

static char *cdns_gpio_ids[] = {
	"CIXH1002",		/* CIX Sky1 (FCH) */
	"CIXH1003",		/* CIX Sky1 */
	NULL
};

static void
cdns_gpio_update(struct cdns_gpio_softc *sc, bus_size_t reg, uint32_t mask,
    bool set)
{
	uint32_t v;

	v = RD4(sc, reg);
	WR4(sc, reg, set ? v | mask : v & ~mask);
}

static device_t
cdns_gpio_get_bus(device_t dev)
{
	return (((struct cdns_gpio_softc *)device_get_softc(dev))->busdev);
}

static int
cdns_gpio_pin_max(device_t dev, int *maxpin)
{
	*maxpin = ((struct cdns_gpio_softc *)device_get_softc(dev))->npins - 1;
	return (0);
}

static int
cdns_gpio_pin_getname(device_t dev, uint32_t pin, char *name)
{
	struct cdns_gpio_softc *sc = device_get_softc(dev);

	if (pin >= (uint32_t)sc->npins)
		return (EINVAL);
	snprintf(name, GPIOMAXNAME, "gpio%d.%u", sc->uid, pin);
	return (0);
}

static int
cdns_gpio_pin_getcaps(device_t dev, uint32_t pin, uint32_t *caps)
{
	struct cdns_gpio_softc *sc = device_get_softc(dev);

	if (pin >= (uint32_t)sc->npins)
		return (EINVAL);
	*caps = CDNS_GPIO_CAPS;
	return (0);
}

/* A pin's direction; a peripheral's pin reports neither. */
static int
cdns_gpio_pin_getflags(device_t dev, uint32_t pin, uint32_t *flags)
{
	struct cdns_gpio_softc *sc = device_get_softc(dev);

	if (pin >= (uint32_t)sc->npins)
		return (EINVAL);
	mtx_lock(&sc->mtx);
	if (RD4(sc, CDNS_GPIO_BYPASS_MODE) & (1u << pin))
		*flags = 0;
	else if (RD4(sc, CDNS_GPIO_DIRECTION_MODE) & (1u << pin))
		*flags = GPIO_PIN_INPUT;
	else
		*flags = GPIO_PIN_OUTPUT;
	mtx_unlock(&sc->mtx);
	return (0);
}

static int
cdns_gpio_pin_setflags(device_t dev, uint32_t pin, uint32_t flags)
{
	struct cdns_gpio_softc *sc = device_get_softc(dev);
	uint32_t mask;

	if (pin >= (uint32_t)sc->npins)
		return (EINVAL);
	if ((flags & ~CDNS_GPIO_CAPS) != 0 ||
	    (flags & CDNS_GPIO_CAPS) == CDNS_GPIO_CAPS)
		return (EINVAL);
	if ((flags & CDNS_GPIO_CAPS) == 0)
		return (0);
	mask = 1u << pin;
	mtx_lock(&sc->mtx);
	if (flags & GPIO_PIN_OUTPUT) {
		cdns_gpio_update(sc, CDNS_GPIO_OUTPUT_EN, mask, true);
		cdns_gpio_update(sc, CDNS_GPIO_DIRECTION_MODE, mask, false);
	} else
		cdns_gpio_update(sc, CDNS_GPIO_DIRECTION_MODE, mask, true);
	/* Ours now, from its peripheral's. */
	cdns_gpio_update(sc, CDNS_GPIO_BYPASS_MODE, mask, false);
	mtx_unlock(&sc->mtx);
	return (0);
}

static int
cdns_gpio_pin_get(device_t dev, uint32_t pin, uint32_t *value)
{
	struct cdns_gpio_softc *sc = device_get_softc(dev);

	if (pin >= (uint32_t)sc->npins)
		return (EINVAL);
	*value = (RD4(sc, CDNS_GPIO_INPUT_VALUE) >> pin) & 1;
	return (0);
}

static int
cdns_gpio_pin_set(device_t dev, uint32_t pin, uint32_t value)
{
	struct cdns_gpio_softc *sc = device_get_softc(dev);

	if (pin >= (uint32_t)sc->npins)
		return (EINVAL);
	mtx_lock(&sc->mtx);
	cdns_gpio_update(sc, CDNS_GPIO_OUTPUT_VALUE, 1u << pin, value != 0);
	mtx_unlock(&sc->mtx);
	return (0);
}

static int
cdns_gpio_pin_toggle(device_t dev, uint32_t pin)
{
	struct cdns_gpio_softc *sc = device_get_softc(dev);

	if (pin >= (uint32_t)sc->npins)
		return (EINVAL);
	mtx_lock(&sc->mtx);
	WR4(sc, CDNS_GPIO_OUTPUT_VALUE,
	    RD4(sc, CDNS_GPIO_OUTPUT_VALUE) ^ (1u << pin));
	mtx_unlock(&sc->mtx);
	return (0);
}

static int
cdns_gpio_probe(device_t dev)
{
	int rv;

	if (acpi_disabled("cdns_gpio"))
		return (ENXIO);
	rv = ACPI_ID_PROBE(device_get_parent(dev), dev, cdns_gpio_ids, NULL);
	if (rv <= 0)
		device_set_desc(dev, "Cadence GPIO controller");
	return (rv);
}

/* Its clock on (CLKT, Sky1's SCMI) before its registers are touched. */
static int
cdns_gpio_clock(device_t dev)
{
#ifdef __aarch64__
	ACPI_BUFFER buf;
	ACPI_OBJECT *pkg, *e;
	int error;

	buf.Pointer = NULL;
	buf.Length = ACPI_ALLOCATE_BUFFER;
	if (ACPI_FAILURE(AcpiEvaluateObject(acpi_get_handle(dev), "CLKT",
	    NULL, &buf)))
		return (0);	/* none named: always on */
	pkg = buf.Pointer;
	error = ENXIO;
	if (pkg->Type == ACPI_TYPE_PACKAGE && pkg->Package.Count > 0) {
		e = &pkg->Package.Elements[0];
		if (e->Type == ACPI_TYPE_PACKAGE && e->Package.Count > 0 &&
		    e->Package.Elements[0].Type == ACPI_TYPE_INTEGER)
			error = sky1_scmi_clk_enable(
			    e->Package.Elements[0].Integer.Value, true);
	}
	AcpiOsFree(buf.Pointer);
	return (error);
#else
	return (0);
#endif
}

static int
cdns_gpio_attach(device_t dev)
{
	struct cdns_gpio_softc *sc;
	uint32_t npins;
	int error, rid;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->uid = device_get_unit(dev);
	if (device_get_property(dev, "ngpios", &npins, sizeof(npins),
	    DEVICE_PROP_UINT32) <= 0)
		npins = 32;
	if (npins == 0 || npins > 32) {
		device_printf(dev, "%u pins?\n", npins);
		return (ENXIO);
	}
	sc->npins = npins;
	if ((error = cdns_gpio_clock(dev)) != 0) {
		device_printf(dev, "cannot enable its clock: %d\n", error);
		return (error);
	}

	rid = 0;
	sc->mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->mem == NULL) {
		device_printf(dev, "cannot map registers\n");
		return (ENXIO);
	}
	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);

	if (bootverbose)
		device_printf(dev, "bypass %#x, input %#x, output enable %#x\n",
		    RD4(sc, CDNS_GPIO_BYPASS_MODE),
		    RD4(sc, CDNS_GPIO_DIRECTION_MODE),
		    RD4(sc, CDNS_GPIO_OUTPUT_EN));

	sc->busdev = gpiobus_add_bus(dev);
	if (sc->busdev == NULL) {
		mtx_destroy(&sc->mtx);
		bus_release_resource(dev, SYS_RES_MEMORY, rid, sc->mem);
		return (ENXIO);
	}
	bus_attach_children(dev);
	return (0);
}

static int
cdns_gpio_detach(device_t dev)
{
	struct cdns_gpio_softc *sc = device_get_softc(dev);

	gpiobus_detach_bus(dev);
	mtx_destroy(&sc->mtx);
	bus_release_resource(dev, SYS_RES_MEMORY, rman_get_rid(sc->mem),
	    sc->mem);
	return (0);
}

static device_method_t cdns_gpio_methods[] = {
	DEVMETHOD(device_probe,		cdns_gpio_probe),
	DEVMETHOD(device_attach,	cdns_gpio_attach),
	DEVMETHOD(device_detach,	cdns_gpio_detach),

	DEVMETHOD(gpio_get_bus,		cdns_gpio_get_bus),
	DEVMETHOD(gpio_pin_max,		cdns_gpio_pin_max),
	DEVMETHOD(gpio_pin_getname,	cdns_gpio_pin_getname),
	DEVMETHOD(gpio_pin_getcaps,	cdns_gpio_pin_getcaps),
	DEVMETHOD(gpio_pin_getflags,	cdns_gpio_pin_getflags),
	DEVMETHOD(gpio_pin_setflags,	cdns_gpio_pin_setflags),
	DEVMETHOD(gpio_pin_get,		cdns_gpio_pin_get),
	DEVMETHOD(gpio_pin_set,		cdns_gpio_pin_set),
	DEVMETHOD(gpio_pin_toggle,	cdns_gpio_pin_toggle),

	DEVMETHOD_END
};

static driver_t cdns_gpio_driver = {
	"gpio",
	cdns_gpio_methods,
	sizeof(struct cdns_gpio_softc),
};

DRIVER_MODULE(cdns_gpio, acpi, cdns_gpio_driver, NULL, NULL);
MODULE_DEPEND(cdns_gpio, acpi, 1, 1, 1);
MODULE_DEPEND(cdns_gpio, gpiobus, 1, 1, 1);
ACPI_PNP_INFO(cdns_gpio_ids);
