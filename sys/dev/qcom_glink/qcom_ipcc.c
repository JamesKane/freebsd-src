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
 * The Qualcomm inter-processor communication controller (IPCC): a doorbell
 * between processors, each a client, with numbered signals.  Ringing a
 * client's signal is a write of both; signals rung at us are read one at a
 * time until none is left.
 *
 * ACPI gives its interrupts (QCOM06C2) but not its registers, which come
 * from a table of SoCs that the DSDT's \_SB.SOID identifies.
 */

#include "opt_acpi.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include <dev/qcom_glink/qcom_ipcc.h>

#define	IPCC_CONFIG		0x08
#define	 IPCC_CLEAR_ON_READ	0x1
#define	IPCC_SEND_ID		0x0c
#define	IPCC_RECV_ID		0x10
#define	 IPCC_NONE_PENDING	0xffffffff
#define	IPCC_SIGNAL_ENABLE	0x14
#define	IPCC_SIGNAL_DISABLE	0x18
#define	IPCC_SIGNAL_CLEAR	0x1c

#define	IPCC_ID(client, signal)	((client) << 16 | (signal))
#define	IPCC_HANDLERS		8

struct qcom_ipcc_soc {
	uint32_t	id;		/* \_SB.SOID */
	vm_paddr_t	base;
};

static const struct qcom_ipcc_soc qcom_ipcc_socs[] = {
	{ 449, 0x408000 },	/* SC8280XP */
};

struct qcom_ipcc_softc {
	device_t		dev;
	struct resource		*mem;
	struct resource		*irq;
	void			*ih;
	struct mtx		mtx;	/* the handlers */
	struct {
		uint32_t		id;	/* 0: free */
		qcom_ipcc_handler_t	*fn;
		void			*arg;
	} handlers[IPCC_HANDLERS];
};

static struct qcom_ipcc_softc *qcom_ipcc_sc;

static char *qcom_ipcc_ids[] = { "QCOM06C2", NULL };

static const struct qcom_ipcc_soc *
qcom_ipcc_find_soc(void)
{
	UINT32 id;
	u_int i;

	if (ACPI_FAILURE(acpi_GetInteger(ACPI_ROOT_OBJECT, "\\_SB.SOID", &id)))
		return (NULL);
	for (i = 0; i < nitems(qcom_ipcc_socs); i++)
		if (qcom_ipcc_socs[i].id == id)
			return (&qcom_ipcc_socs[i]);
	return (NULL);
}

static void
qcom_ipcc_intr(void *arg)
{
	struct qcom_ipcc_softc *sc = arg;
	qcom_ipcc_handler_t *fn;
	uint32_t id;
	void *fnarg;
	u_int i;

	while ((id = bus_read_4(sc->mem, IPCC_RECV_ID)) != IPCC_NONE_PENDING) {
		bus_write_4(sc->mem, IPCC_SIGNAL_CLEAR, id);
		fn = NULL;
		mtx_lock(&sc->mtx);
		for (i = 0; i < IPCC_HANDLERS; i++)
			if (sc->handlers[i].id == id) {
				fn = sc->handlers[i].fn;
				fnarg = sc->handlers[i].arg;
				break;
			}
		mtx_unlock(&sc->mtx);
		if (fn != NULL)
			fn(fnarg);
		else
			device_printf(sc->dev, "client %u signal %u: nobody "
			    "listens\n", id >> 16, id & 0xffff);
	}
}

int
qcom_ipcc_register(u_int client, u_int signal, qcom_ipcc_handler_t *fn,
    void *arg)
{
	struct qcom_ipcc_softc *sc = qcom_ipcc_sc;
	uint32_t id = IPCC_ID(client, signal);
	int free, i;

	if (sc == NULL)
		return (ENXIO);
	mtx_lock(&sc->mtx);
	free = -1;
	for (i = 0; i < IPCC_HANDLERS; i++) {
		if (sc->handlers[i].id == id) {
			mtx_unlock(&sc->mtx);
			return (EEXIST);
		}
		if (sc->handlers[i].id == 0 && free < 0)
			free = i;
	}
	if (free < 0) {
		mtx_unlock(&sc->mtx);
		return (ENOSPC);
	}
	sc->handlers[free].fn = fn;
	sc->handlers[free].arg = arg;
	sc->handlers[free].id = id;
	mtx_unlock(&sc->mtx);
	bus_write_4(sc->mem, IPCC_SIGNAL_ENABLE, id);
	return (0);
}

void
qcom_ipcc_unregister(u_int client, u_int signal)
{
	struct qcom_ipcc_softc *sc = qcom_ipcc_sc;
	uint32_t id = IPCC_ID(client, signal);
	int i;

	if (sc == NULL)
		return;
	bus_write_4(sc->mem, IPCC_SIGNAL_DISABLE, id);
	mtx_lock(&sc->mtx);
	for (i = 0; i < IPCC_HANDLERS; i++)
		if (sc->handlers[i].id == id)
			sc->handlers[i].id = 0;
	mtx_unlock(&sc->mtx);
}

int
qcom_ipcc_send(u_int client, u_int signal)
{
	struct qcom_ipcc_softc *sc = qcom_ipcc_sc;

	if (sc == NULL)
		return (ENXIO);
	bus_write_4(sc->mem, IPCC_SEND_ID, IPCC_ID(client, signal));
	return (0);
}

static int
qcom_ipcc_probe(device_t dev)
{
	int rv;

	rv = ACPI_ID_PROBE(device_get_parent(dev), dev, qcom_ipcc_ids, NULL);
	if (rv > 0)
		return (rv);
	if (qcom_ipcc_find_soc() == NULL)
		return (ENXIO);
	device_set_desc(dev, "Qualcomm IPC controller");
	return (rv);
}

static int
qcom_ipcc_detach(device_t dev);

static int
qcom_ipcc_attach(device_t dev)
{
	struct qcom_ipcc_softc *sc = device_get_softc(dev);
	const struct qcom_ipcc_soc *soc = qcom_ipcc_find_soc();
	int error, rid;

	if (qcom_ipcc_sc != NULL)
		return (EEXIST);
	sc->dev = dev;
	mtx_init(&sc->mtx, device_get_nameunit(dev), NULL, MTX_DEF);
	/* The registers ACPI leaves out. */
	rid = 0;
	error = bus_set_resource(dev, SYS_RES_MEMORY, rid, soc->base, 0x1000);
	if (error == 0)
		sc->mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid,
		    RF_ACTIVE);
	if (sc->mem == NULL) {
		device_printf(dev, "can't map the registers\n");
		error = ENXIO;
		goto fail;
	}
	/* Signals stay pending until cleared, after we've read them. */
	bus_write_4(sc->mem, IPCC_CONFIG,
	    bus_read_4(sc->mem, IPCC_CONFIG) & ~IPCC_CLEAR_ON_READ);
	rid = 0;
	sc->irq = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid, RF_ACTIVE);
	if (sc->irq == NULL) {
		device_printf(dev, "can't get the interrupt\n");
		error = ENXIO;
		goto fail;
	}
	error = bus_setup_intr(dev, sc->irq, INTR_TYPE_MISC | INTR_MPSAFE,
	    NULL, qcom_ipcc_intr, sc, &sc->ih);
	if (error != 0)
		goto fail;
	qcom_ipcc_sc = sc;
	return (0);
fail:
	qcom_ipcc_detach(dev);
	return (error);
}

static int
qcom_ipcc_detach(device_t dev)
{
	struct qcom_ipcc_softc *sc = device_get_softc(dev);
	int i;

	if (qcom_ipcc_sc == sc) {
		for (i = 0; i < IPCC_HANDLERS; i++)
			if (sc->handlers[i].id != 0)
				return (EBUSY);
		qcom_ipcc_sc = NULL;
	}
	if (sc->ih != NULL)
		bus_teardown_intr(dev, sc->irq, sc->ih);
	if (sc->irq != NULL)
		bus_release_resource(dev, SYS_RES_IRQ, rman_get_rid(sc->irq),
		    sc->irq);
	if (sc->mem != NULL)
		bus_release_resource(dev, SYS_RES_MEMORY,
		    rman_get_rid(sc->mem), sc->mem);
	mtx_destroy(&sc->mtx);
	return (0);
}

static device_method_t qcom_ipcc_methods[] = {
	DEVMETHOD(device_probe,		qcom_ipcc_probe),
	DEVMETHOD(device_attach,	qcom_ipcc_attach),
	DEVMETHOD(device_detach,	qcom_ipcc_detach),

	DEVMETHOD_END
};

static driver_t qcom_ipcc_driver = {
	"qcom_ipcc",
	qcom_ipcc_methods,
	sizeof(struct qcom_ipcc_softc),
};

DRIVER_MODULE(qcom_ipcc, acpi, qcom_ipcc_driver, 0, 0);
MODULE_DEPEND(qcom_ipcc, acpi, 1, 1, 1);
MODULE_VERSION(qcom_ipcc, 1);
