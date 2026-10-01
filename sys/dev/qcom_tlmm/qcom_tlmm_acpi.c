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
 * The GPIOs of a Qualcomm TLMM (top level mode multiplexer), as ACPI
 * describes it (QCOM060C), and as the firmware leaves it: every pin already
 * in the function and with the pull its board needs.
 *
 * The DSDT of these machines lists GPIO consumers from Qualcomm's reference
 * design, which a board may not share: on the Radxa Dragon Q8B it would make
 * outputs of a dozen pins the board doesn't use as Qualcomm's did.  Windows
 * configures a pin only when a driver opens it, but acpi_gpiobus configures
 * every pin listed when it attaches.  So until the bus has attached, pin
 * configuration is ignored; after, it is a consumer's, and it never changes
 * a pin's function.  Some pins belong to the secure world, and touching
 * their registers faults; they are not offered.
 *
 * Pin interrupts come through the TLMM's summary interrupt, the first of
 * its ACPI interrupts, for pins in the GPIO function only: the DSDT's
 * event pins are the reference design's too.
 */

#include "opt_acpi.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/gpio.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/intr.h>
#include <machine/resource.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <contrib/dev/acpica/include/accommon.h>

#include <dev/acpica/acpivar.h>
#include <dev/gpio/gpiobusvar.h>

#include "gpio_if.h"
#include "pic_if.h"

/* Each pin has a 4 KB page of registers. */
#define	TLMM_CFG(p)		((p) * 0x1000)
#define	 TLMM_CFG_PULL_MASK	0x3
#define	 TLMM_CFG_PULL_NONE	0x0
#define	 TLMM_CFG_PULL_DOWN	0x1
#define	 TLMM_CFG_PULL_UP	0x3
#define	 TLMM_CFG_FUNC_MASK	0x3c	/* 0: GPIO */
#define	 TLMM_CFG_OE		0x200
#define	TLMM_IO(p)		((p) * 0x1000 + 0x4)
#define	 TLMM_IO_IN		0x1
#define	 TLMM_IO_OUT		0x2
#define	TLMM_INTR_CFG(p)	((p) * 0x1000 + 0x8)
#define	 TLMM_INTR_ENABLE	0x001
#define	 TLMM_INTR_POL_HIGH	0x002	/* level high; set for all edges */
#define	 TLMM_INTR_DET_MASK	0x00c
#define	 TLMM_INTR_DET_LEVEL	0x000
#define	 TLMM_INTR_DET_RISING	0x004
#define	 TLMM_INTR_DET_FALLING	0x008
#define	 TLMM_INTR_DET_BOTH	0x00c
#define	 TLMM_INTR_RAW_STATUS	0x010	/* latch the status */
#define	 TLMM_INTR_TARGET_MASK	0x0e0
#define	 TLMM_INTR_TARGET_APPS	0x060	/* the application CPUs */
#define	TLMM_INTR_STATUS(p)	((p) * 0x1000 + 0xc)
#define	 TLMM_INTR_PENDING	0x1	/* write 0 to acknowledge */

#define	TLMM_CAPS	(GPIO_PIN_INPUT | GPIO_PIN_OUTPUT | GPIO_PIN_PULLUP | \
			    GPIO_PIN_PULLDOWN)
#define	TLMM_INTR_CAPS	(GPIO_INTR_LEVEL_LOW | GPIO_INTR_LEVEL_HIGH | \
			    GPIO_INTR_EDGE_RISING | GPIO_INTR_EDGE_FALLING | \
			    GPIO_INTR_EDGE_BOTH)

struct qcom_tlmm_acpi_soc {
	uint32_t	id;		/* \_SB.SOID */
	const char	*name;
	u_int		npins;
	const u_int	(*reserved)[2];	/* first, count; ends with 0, 0 */
};

static const u_int sc8280xp_reserved[][2] = {
	{ 74, 6 }, { 83, 4 }, { 125, 2 }, { 128, 2 }, { 0, 0 }
};

static const struct qcom_tlmm_acpi_soc qcom_tlmm_acpi_socs[] = {
	{ 449, "SC8280XP", 228, sc8280xp_reserved },
};

struct qcom_tlmm_acpi_isrc {
	struct intr_irqsrc	isrc;
	u_int			pin;
	uint32_t		mode;	/* GPIO_INTR_*, or CONFORM unset */
};

struct qcom_tlmm_acpi_softc {
	device_t		dev;
	device_t		busdev;
	struct resource		*mem;
	struct mtx		mtx;
	const struct qcom_tlmm_acpi_soc *soc;
	bool			ready;	/* consumers' configuration taken */
	struct resource		*irq;	/* summary interrupt, or NULL */
	void			*ih;
	struct mtx		intr_mtx;	/* the interrupt registers */
	struct qcom_tlmm_acpi_isrc *isrcs;
	uint64_t		*active;	/* pins with a handler */
};

static char *qcom_tlmm_acpi_ids[] = { "QCOM060C", NULL };

static const struct qcom_tlmm_acpi_soc *
qcom_tlmm_acpi_find_soc(void)
{
	UINT32 id;
	u_int i;

	if (ACPI_FAILURE(acpi_GetInteger(ACPI_ROOT_OBJECT, "\\_SB.SOID", &id)))
		return (NULL);
	for (i = 0; i < nitems(qcom_tlmm_acpi_socs); i++)
		if (qcom_tlmm_acpi_socs[i].id == id)
			return (&qcom_tlmm_acpi_socs[i]);
	return (NULL);
}

static bool
qcom_tlmm_acpi_valid(struct qcom_tlmm_acpi_softc *sc, uint32_t pin)
{
	const u_int (*r)[2];

	if (pin >= sc->soc->npins)
		return (false);
	for (r = sc->soc->reserved; (*r)[1] != 0; r++)
		if (pin >= (*r)[0] && pin < (*r)[0] + (*r)[1])
			return (false);
	return (true);
}

/* Interrupts */

#define	TLMM_ISRC(sc, pin)	(&(sc)->isrcs[(pin)].isrc)

static void
qcom_tlmm_acpi_intr_cfg(struct qcom_tlmm_acpi_softc *sc, u_int pin,
    uint32_t clear, uint32_t set)
{
	uint32_t v;

	mtx_lock_spin(&sc->intr_mtx);
	v = bus_read_4(sc->mem, TLMM_INTR_CFG(pin));
	bus_write_4(sc->mem, TLMM_INTR_CFG(pin), (v & ~clear) | set);
	mtx_unlock_spin(&sc->intr_mtx);
}

static void
qcom_tlmm_acpi_intr_ack(struct qcom_tlmm_acpi_softc *sc, u_int pin)
{

	bus_write_4(sc->mem, TLMM_INTR_STATUS(pin), 0);
}

static int
qcom_tlmm_acpi_intr(void *arg)
{
	struct qcom_tlmm_acpi_softc *sc = arg;
	struct trapframe *tf = curthread->td_intr_frame;
	uint64_t bits;
	u_int pin, w;

	for (w = 0; w < howmany(sc->soc->npins, 64); w++) {
		for (bits = atomic_load_64(&sc->active[w]); bits != 0;
		    bits &= bits - 1) {
			pin = w * 64 + ffsll(bits) - 1;
			if ((bus_read_4(sc->mem, TLMM_INTR_STATUS(pin)) &
			    TLMM_INTR_PENDING) == 0)
				continue;
			if (intr_isrc_dispatch(TLMM_ISRC(sc, pin), tf) != 0) {
				qcom_tlmm_acpi_intr_cfg(sc, pin,
				    TLMM_INTR_ENABLE, 0);
				qcom_tlmm_acpi_intr_ack(sc, pin);
				device_printf(sc->dev,
				    "stray interrupt on pin %u, disabled\n",
				    pin);
			}
		}
	}
	return (FILTER_HANDLED);
}

static int
qcom_tlmm_acpi_pic_map_intr(device_t dev, struct intr_map_data *data,
    struct intr_irqsrc **isrcp)
{
	struct qcom_tlmm_acpi_softc *sc = device_get_softc(dev);
	struct intr_map_data_gpio *gd;

	if (data->type != INTR_MAP_DATA_GPIO)
		return (ENOTSUP);
	gd = (struct intr_map_data_gpio *)data;
	if (!qcom_tlmm_acpi_valid(sc, gd->gpio_pin_num))
		return (EINVAL);
	*isrcp = TLMM_ISRC(sc, gd->gpio_pin_num);
	return (0);
}

static int
qcom_tlmm_acpi_pic_setup_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
	struct qcom_tlmm_acpi_softc *sc = device_get_softc(dev);
	struct qcom_tlmm_acpi_isrc *ti = (struct qcom_tlmm_acpi_isrc *)isrc;
	struct intr_map_data_gpio *gd;
	uint32_t cfg, mode;

	if (data == NULL || data->type != INTR_MAP_DATA_GPIO)
		return (ENOTSUP);
	gd = (struct intr_map_data_gpio *)data;
	mode = gd->gpio_intr_mode & GPIO_INTR_MASK;
	if (gd->gpio_pin_num != ti->pin || !powerof2(mode))
		return (EINVAL);
	if (isrc->isrc_handlers != 0)
		return (ti->mode == mode ? 0 : EINVAL);
	/* A pin in another function belongs to that function. */
	if ((bus_read_4(sc->mem, TLMM_CFG(ti->pin)) & TLMM_CFG_FUNC_MASK) != 0)
		return (EBUSY);

	switch (mode) {
	case GPIO_INTR_LEVEL_LOW:
		cfg = TLMM_INTR_DET_LEVEL;
		break;
	case GPIO_INTR_LEVEL_HIGH:
		cfg = TLMM_INTR_DET_LEVEL | TLMM_INTR_POL_HIGH;
		break;
	case GPIO_INTR_EDGE_RISING:
		cfg = TLMM_INTR_DET_RISING | TLMM_INTR_POL_HIGH;
		break;
	case GPIO_INTR_EDGE_FALLING:
		cfg = TLMM_INTR_DET_FALLING | TLMM_INTR_POL_HIGH;
		break;
	case GPIO_INTR_EDGE_BOTH:
		cfg = TLMM_INTR_DET_BOTH | TLMM_INTR_POL_HIGH;
		break;
	default:
		return (EINVAL);
	}
	ti->mode = mode;
	qcom_tlmm_acpi_intr_cfg(sc, ti->pin, TLMM_INTR_ENABLE |
	    TLMM_INTR_POL_HIGH | TLMM_INTR_DET_MASK | TLMM_INTR_TARGET_MASK,
	    cfg | TLMM_INTR_RAW_STATUS | TLMM_INTR_TARGET_APPS);
	/* Not for an edge from before. */
	qcom_tlmm_acpi_intr_ack(sc, ti->pin);
	atomic_set_64(&sc->active[ti->pin / 64], 1ull << (ti->pin % 64));
	return (0);
}

static int
qcom_tlmm_acpi_pic_teardown_intr(device_t dev, struct intr_irqsrc *isrc,
    struct resource *res, struct intr_map_data *data)
{
	struct qcom_tlmm_acpi_softc *sc = device_get_softc(dev);
	struct qcom_tlmm_acpi_isrc *ti = (struct qcom_tlmm_acpi_isrc *)isrc;

	if (isrc->isrc_handlers == 0) {
		atomic_clear_64(&sc->active[ti->pin / 64],
		    1ull << (ti->pin % 64));
		qcom_tlmm_acpi_intr_cfg(sc, ti->pin, TLMM_INTR_ENABLE |
		    TLMM_INTR_TARGET_MASK, TLMM_INTR_TARGET_MASK);
		qcom_tlmm_acpi_intr_ack(sc, ti->pin);
		ti->mode = GPIO_INTR_CONFORM;
	}
	return (0);
}

static void
qcom_tlmm_acpi_pic_enable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct qcom_tlmm_acpi_isrc *ti = (struct qcom_tlmm_acpi_isrc *)isrc;

	qcom_tlmm_acpi_intr_cfg(device_get_softc(dev), ti->pin, 0,
	    TLMM_INTR_ENABLE);
}

static void
qcom_tlmm_acpi_pic_disable_intr(device_t dev, struct intr_irqsrc *isrc)
{
	struct qcom_tlmm_acpi_isrc *ti = (struct qcom_tlmm_acpi_isrc *)isrc;

	qcom_tlmm_acpi_intr_cfg(device_get_softc(dev), ti->pin,
	    TLMM_INTR_ENABLE, 0);
}

static bool
qcom_tlmm_acpi_is_level(struct qcom_tlmm_acpi_isrc *ti)
{

	return ((ti->mode & (GPIO_INTR_LEVEL_LOW | GPIO_INTR_LEVEL_HIGH)) != 0);
}

static void
qcom_tlmm_acpi_pic_post_filter(device_t dev, struct intr_irqsrc *isrc)
{
	struct qcom_tlmm_acpi_isrc *ti = (struct qcom_tlmm_acpi_isrc *)isrc;

	qcom_tlmm_acpi_intr_ack(device_get_softc(dev), ti->pin);
}

/*
 * An edge is acknowledged before its thread runs, so the next one isn't
 * lost; a level is masked until the thread has dealt with its cause.
 */
static void
qcom_tlmm_acpi_pic_pre_ithread(device_t dev, struct intr_irqsrc *isrc)
{
	struct qcom_tlmm_acpi_isrc *ti = (struct qcom_tlmm_acpi_isrc *)isrc;

	if (qcom_tlmm_acpi_is_level(ti))
		qcom_tlmm_acpi_pic_disable_intr(dev, isrc);
	qcom_tlmm_acpi_intr_ack(device_get_softc(dev), ti->pin);
}

static void
qcom_tlmm_acpi_pic_post_ithread(device_t dev, struct intr_irqsrc *isrc)
{
	struct qcom_tlmm_acpi_isrc *ti = (struct qcom_tlmm_acpi_isrc *)isrc;

	if (qcom_tlmm_acpi_is_level(ti)) {
		qcom_tlmm_acpi_intr_ack(device_get_softc(dev), ti->pin);
		qcom_tlmm_acpi_pic_enable_intr(dev, isrc);
	}
}

/* The summary interrupt is the first of the device's interrupts. */
static int
qcom_tlmm_acpi_intr_attach(struct qcom_tlmm_acpi_softc *sc)
{
	const char *name = device_get_nameunit(sc->dev);
	u_int pin;
	int error, rid;

	mtx_init(&sc->intr_mtx, name, "TLMM interrupts", MTX_SPIN);
	sc->isrcs = mallocarray(sc->soc->npins, sizeof(*sc->isrcs), M_DEVBUF,
	    M_WAITOK | M_ZERO);
	sc->active = mallocarray(howmany(sc->soc->npins, 64),
	    sizeof(*sc->active), M_DEVBUF, M_WAITOK | M_ZERO);
	for (pin = 0; pin < sc->soc->npins; pin++) {
		sc->isrcs[pin].pin = pin;
		sc->isrcs[pin].mode = GPIO_INTR_CONFORM;
		error = intr_isrc_register(TLMM_ISRC(sc, pin), sc->dev, 0,
		    "%s,%u", name, pin);
		if (error != 0)
			goto fail;	/* the registered ones stay */
	}
	if (intr_pic_register(sc->dev, ACPI_GPIO_XREF) == NULL) {
		error = ENXIO;
		goto fail;
	}
	rid = 0;
	sc->irq = bus_alloc_resource_any(sc->dev, SYS_RES_IRQ, &rid,
	    RF_ACTIVE | RF_SHAREABLE);
	if (sc->irq == NULL) {
		error = ENXIO;
		goto fail;
	}
	error = bus_setup_intr(sc->dev, sc->irq, INTR_TYPE_MISC | INTR_MPSAFE,
	    qcom_tlmm_acpi_intr, NULL, sc, &sc->ih);
	if (error != 0) {
		bus_release_resource(sc->dev, SYS_RES_IRQ, rid, sc->irq);
		sc->irq = NULL;
		goto fail;
	}
	return (0);
fail:
	/* Registered interrupt sources and PICs can't be taken back. */
	return (error);
}

static int
qcom_tlmm_acpi_probe(device_t dev)
{
	int rv;

	if (acpi_disabled("gpio"))
		return (ENXIO);
	rv = ACPI_ID_PROBE(device_get_parent(dev), dev, qcom_tlmm_acpi_ids,
	    NULL);
	if (rv > 0)
		return (rv);
	if (qcom_tlmm_acpi_find_soc() == NULL)
		return (ENXIO);
	device_set_desc(dev, "Qualcomm TLMM GPIO controller");
	return (rv);
}

static int
qcom_tlmm_acpi_attach(device_t dev)
{
	struct qcom_tlmm_acpi_softc *sc = device_get_softc(dev);
	int rid;

	sc->dev = dev;
	sc->soc = qcom_tlmm_acpi_find_soc();
	rid = 0;
	sc->mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (sc->mem == NULL) {
		device_printf(dev, "can't map the registers\n");
		return (ENXIO);
	}
	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);
	/* Without the interrupt the pins still work, without interrupts. */
	if (qcom_tlmm_acpi_intr_attach(sc) != 0)
		device_printf(dev, "no pin interrupts\n");
	sc->busdev = gpiobus_add_bus(dev);
	if (sc->busdev == NULL) {
		if (sc->irq != NULL)
			return (ENXIO);	/* the PIC stays registered */
		mtx_destroy(&sc->mtx);
		bus_release_resource(dev, SYS_RES_MEMORY, rid, sc->mem);
		return (ENXIO);
	}
	bus_attach_children(dev);
	sc->ready = true;
	device_printf(dev, "%s, %u pins%s\n", sc->soc->name, sc->soc->npins,
	    sc->irq != NULL ? ", interrupts" : "");
	return (0);
}

static int
qcom_tlmm_acpi_detach(device_t dev)
{
	struct qcom_tlmm_acpi_softc *sc = device_get_softc(dev);
	int error;

	/* INTRNG can't deregister a PIC. */
	if (sc->irq != NULL)
		return (EBUSY);
	error = bus_generic_detach(dev);
	if (error != 0)
		return (error);
	mtx_destroy(&sc->mtx);
	bus_release_resource(dev, SYS_RES_MEMORY, rman_get_rid(sc->mem),
	    sc->mem);
	return (0);
}

static device_t
qcom_tlmm_acpi_get_bus(device_t dev)
{
	struct qcom_tlmm_acpi_softc *sc = device_get_softc(dev);

	return (sc->busdev);
}

static int
qcom_tlmm_acpi_pin_max(device_t dev, int *maxpin)
{
	struct qcom_tlmm_acpi_softc *sc = device_get_softc(dev);

	*maxpin = sc->soc->npins - 1;
	return (0);
}

static int
qcom_tlmm_acpi_pin_getname(device_t dev, uint32_t pin, char *name)
{
	struct qcom_tlmm_acpi_softc *sc = device_get_softc(dev);

	if (pin >= sc->soc->npins)
		return (EINVAL);
	snprintf(name, GPIOMAXNAME, "gpio%u", pin);
	return (0);
}

static int
qcom_tlmm_acpi_pin_getcaps(device_t dev, uint32_t pin, uint32_t *caps)
{
	struct qcom_tlmm_acpi_softc *sc = device_get_softc(dev);

	if (pin >= sc->soc->npins)
		return (EINVAL);
	*caps = !qcom_tlmm_acpi_valid(sc, pin) ? 0 :
	    sc->irq != NULL ? TLMM_CAPS | TLMM_INTR_CAPS : TLMM_CAPS;
	return (0);
}

static int
qcom_tlmm_acpi_pin_getflags(device_t dev, uint32_t pin, uint32_t *flags)
{
	struct qcom_tlmm_acpi_softc *sc = device_get_softc(dev);
	uint32_t cfg;

	if (!qcom_tlmm_acpi_valid(sc, pin))
		return (EINVAL);
	cfg = bus_read_4(sc->mem, TLMM_CFG(pin));
	*flags = (cfg & TLMM_CFG_OE) != 0 ? GPIO_PIN_OUTPUT : GPIO_PIN_INPUT;
	switch (cfg & TLMM_CFG_PULL_MASK) {
	case TLMM_CFG_PULL_UP:
		*flags |= GPIO_PIN_PULLUP;
		break;
	case TLMM_CFG_PULL_DOWN:
		*flags |= GPIO_PIN_PULLDOWN;
		break;
	}
	return (0);
}

static int
qcom_tlmm_acpi_pin_setflags(device_t dev, uint32_t pin, uint32_t flags)
{
	struct qcom_tlmm_acpi_softc *sc = device_get_softc(dev);
	uint32_t cfg;

	if (!qcom_tlmm_acpi_valid(sc, pin))
		return (EINVAL);
	if (!sc->ready)
		return (0);
	if ((flags & (GPIO_PIN_INPUT | GPIO_PIN_OUTPUT)) ==
	    (GPIO_PIN_INPUT | GPIO_PIN_OUTPUT) ||
	    (flags & (GPIO_PIN_PULLUP | GPIO_PIN_PULLDOWN)) ==
	    (GPIO_PIN_PULLUP | GPIO_PIN_PULLDOWN))
		return (EINVAL);
	mtx_lock(&sc->mtx);
	cfg = bus_read_4(sc->mem, TLMM_CFG(pin));
	if ((cfg & TLMM_CFG_FUNC_MASK) != 0) {
		mtx_unlock(&sc->mtx);
		return (EBUSY);
	}
	cfg &= ~TLMM_CFG_PULL_MASK;
	if ((flags & GPIO_PIN_PULLUP) != 0)
		cfg |= TLMM_CFG_PULL_UP;
	else if ((flags & GPIO_PIN_PULLDOWN) != 0)
		cfg |= TLMM_CFG_PULL_DOWN;
	/* A direction is changed only when one is given. */
	if ((flags & GPIO_PIN_OUTPUT) != 0)
		cfg |= TLMM_CFG_OE;
	else if ((flags & GPIO_PIN_INPUT) != 0)
		cfg &= ~TLMM_CFG_OE;
	bus_write_4(sc->mem, TLMM_CFG(pin), cfg);
	mtx_unlock(&sc->mtx);
	return (0);
}

static int
qcom_tlmm_acpi_pin_get(device_t dev, uint32_t pin, uint32_t *val)
{
	struct qcom_tlmm_acpi_softc *sc = device_get_softc(dev);

	if (!qcom_tlmm_acpi_valid(sc, pin))
		return (EINVAL);
	*val = (bus_read_4(sc->mem, TLMM_IO(pin)) & TLMM_IO_IN) != 0;
	return (0);
}

static int
qcom_tlmm_acpi_set_out(struct qcom_tlmm_acpi_softc *sc, uint32_t pin,
    int how)
{
	uint32_t io;

	if (!qcom_tlmm_acpi_valid(sc, pin))
		return (EINVAL);
	mtx_lock(&sc->mtx);
	if ((bus_read_4(sc->mem, TLMM_CFG(pin)) & TLMM_CFG_FUNC_MASK) != 0) {
		mtx_unlock(&sc->mtx);
		return (EBUSY);
	}
	io = bus_read_4(sc->mem, TLMM_IO(pin));
	if (how < 0)
		io ^= TLMM_IO_OUT;
	else if (how > 0)
		io |= TLMM_IO_OUT;
	else
		io &= ~TLMM_IO_OUT;
	bus_write_4(sc->mem, TLMM_IO(pin), io);
	mtx_unlock(&sc->mtx);
	return (0);
}

static int
qcom_tlmm_acpi_pin_set(device_t dev, uint32_t pin, unsigned int val)
{

	return (qcom_tlmm_acpi_set_out(device_get_softc(dev), pin,
	    val != 0 ? 1 : 0));
}

static int
qcom_tlmm_acpi_pin_toggle(device_t dev, uint32_t pin)
{

	return (qcom_tlmm_acpi_set_out(device_get_softc(dev), pin, -1));
}

static device_method_t qcom_tlmm_acpi_methods[] = {
	DEVMETHOD(device_probe,		qcom_tlmm_acpi_probe),
	DEVMETHOD(device_attach,	qcom_tlmm_acpi_attach),
	DEVMETHOD(device_detach,	qcom_tlmm_acpi_detach),

	DEVMETHOD(gpio_get_bus,		qcom_tlmm_acpi_get_bus),
	DEVMETHOD(gpio_pin_max,		qcom_tlmm_acpi_pin_max),
	DEVMETHOD(gpio_pin_getname,	qcom_tlmm_acpi_pin_getname),
	DEVMETHOD(gpio_pin_getcaps,	qcom_tlmm_acpi_pin_getcaps),
	DEVMETHOD(gpio_pin_getflags,	qcom_tlmm_acpi_pin_getflags),
	DEVMETHOD(gpio_pin_setflags,	qcom_tlmm_acpi_pin_setflags),
	DEVMETHOD(gpio_pin_get,		qcom_tlmm_acpi_pin_get),
	DEVMETHOD(gpio_pin_set,		qcom_tlmm_acpi_pin_set),
	DEVMETHOD(gpio_pin_toggle,	qcom_tlmm_acpi_pin_toggle),

	DEVMETHOD(pic_map_intr,		qcom_tlmm_acpi_pic_map_intr),
	DEVMETHOD(pic_setup_intr,	qcom_tlmm_acpi_pic_setup_intr),
	DEVMETHOD(pic_teardown_intr,	qcom_tlmm_acpi_pic_teardown_intr),
	DEVMETHOD(pic_enable_intr,	qcom_tlmm_acpi_pic_enable_intr),
	DEVMETHOD(pic_disable_intr,	qcom_tlmm_acpi_pic_disable_intr),
	DEVMETHOD(pic_post_filter,	qcom_tlmm_acpi_pic_post_filter),
	DEVMETHOD(pic_pre_ithread,	qcom_tlmm_acpi_pic_pre_ithread),
	DEVMETHOD(pic_post_ithread,	qcom_tlmm_acpi_pic_post_ithread),

	DEVMETHOD_END
};

static driver_t qcom_tlmm_acpi_driver = {
	"gpio",
	qcom_tlmm_acpi_methods,
	sizeof(struct qcom_tlmm_acpi_softc),
};

EARLY_DRIVER_MODULE(qcom_tlmm_acpi, acpi, qcom_tlmm_acpi_driver, NULL, NULL,
    BUS_PASS_INTERRUPT + BUS_PASS_ORDER_LATE);
MODULE_DEPEND(qcom_tlmm_acpi, acpi, 1, 1, 1);
MODULE_DEPEND(qcom_tlmm_acpi, gpiobus, 1, 1, 1);
