/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2019 Val Packett <val@packett.cool>
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

#include "opt_acpi.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/condvar.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/rman.h>

#include <machine/bus.h>
#include <machine/resource.h>

#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>

#include <dev/usb/usb_core.h>
#include <dev/usb/usb_busdma.h>
#include <dev/usb/usb_process.h>

#include <dev/usb/usb_controller.h>
#include <dev/usb/usb_bus.h>
#include <dev/usb/controller/xhci.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include <dev/usb/controller/dwc3/dwc3.h>

#include "generic_xhci.h"
#include "acpi_bus_if.h"

static char *xhci_ids[] = {
	"PNP0D10",
	"PNP0D15",
	NULL,
};

/*
 * A USB Role Switch device (PNP0CA1) holds a dual-role controller's
 * registers; its host-role child, with _ADR 0, holds the host
 * controller's interrupts, the controller's own first.
 */
static char *urs_ids[] = {
	"PNP0CA1",
	NULL,
};

#define	URS_HOST_ADR	0

struct urs_irq {
	UINT32	gsiv;
	int	trig;
	int	pol;
	bool	found;
};

static ACPI_STATUS
urs_find_irq(ACPI_RESOURCE *res, void *context)
{
	struct urs_irq *irq = context;
	ACPI_RESOURCE_EXTENDED_IRQ *ext;

	/* A GIC interrupt consumed by the device, not a routed one. */
	if (res->Type != ACPI_RESOURCE_TYPE_EXTENDED_IRQ)
		return (AE_OK);
	ext = &res->Data.ExtendedIrq;
	if (ext->ProducerConsumer != ACPI_CONSUMER ||
	    ext->ResourceSource.StringLength != 0 || ext->InterruptCount == 0)
		return (AE_OK);
	irq->gsiv = ext->Interrupts[0];
	irq->trig = ext->Triggering == ACPI_EDGE_SENSITIVE ?
	    INTR_TRIGGER_EDGE : INTR_TRIGGER_LEVEL;
	irq->pol = ext->Polarity == ACPI_ACTIVE_LOW ?
	    INTR_POLARITY_LOW : INTR_POLARITY_HIGH;
	irq->found = true;
	return (AE_CTRL_TERMINATE);
}

/* Find the interrupt of the role switch's present host-role child. */
static int
urs_host_irq(device_t dev, struct urs_irq *irq)
{
	ACPI_HANDLE child;
	UINT32 adr, sta;

	child = NULL;
	while (ACPI_SUCCESS(AcpiGetNextObject(ACPI_TYPE_DEVICE,
	    acpi_get_handle(dev), child, &child))) {
		if (ACPI_FAILURE(acpi_GetInteger(child, "_ADR", &adr)) ||
		    adr != URS_HOST_ADR)
			continue;
		if (ACPI_SUCCESS(acpi_GetInteger(child, "_STA", &sta)) &&
		    !ACPI_DEVICE_PRESENT(sta))
			continue;
		irq->found = false;
		AcpiWalkResources(child, "_CRS", urs_find_irq, irq);
		if (irq->found)
			return (0);
	}
	return (ENXIO);
}

/* A role switch that is not also described as an xHCI controller. */
static bool
generic_xhci_acpi_is_urs(device_t dev)
{
	device_t bus = device_get_parent(dev);

	return (ACPI_ID_PROBE(bus, dev, xhci_ids, NULL) > 0 &&
	    ACPI_ID_PROBE(bus, dev, urs_ids, NULL) <= 0);
}

static int
generic_xhci_acpi_probe(device_t dev)
{
	struct urs_irq irq;

	if (ACPI_ID_PROBE(device_get_parent(dev), dev, xhci_ids, NULL) > 0 &&
	    (!generic_xhci_acpi_is_urs(dev) || urs_host_irq(dev, &irq) != 0))
		return (ENXIO);

	device_set_desc(dev, XHCI_HC_DEVSTR);

	return (BUS_PROBE_GENERIC);
}

/*
 * A role switch's controller is used as firmware left it, which must be
 * host mode.  If it is a Synopsys DWC_usb3x core, check that, and disable
 * a receive threshold firmware may have left enabled: on the SC8280XP's
 * USB-C controllers it cuts SuperSpeed reads to a third.  The core's
 * reset default has it disabled.
 */
static int
generic_xhci_acpi_urs_setup(device_t dev)
{
	struct resource *mem;
	uint32_t id, reg, sel;
	int error, rid;

	rid = 0;
	mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (mem == NULL)
		return (ENXIO);
	error = 0;
	if (rman_get_size(mem) <= DWC3_GSNPSID)
		goto out;
	id = DWC3_VERSION(bus_read_4(mem, DWC3_GSNPSID));
	if (id == DWC3_IP_ID)
		sel = DWC3_GRXTHRCFG_PKTCNTSEL;
	else if (id == DWC3_1_IP_ID || id == DWC3_2_IP_ID)
		sel = DWC31_GRXTHRCFG_PKTCNTSEL;
	else
		goto out;
	if ((bus_read_4(mem, DWC3_GCTL) & DWC3_GCTL_PRTCAPDIR_MASK) !=
	    DWC3_GCTL_PRTCAPDIR_HOST) {
		device_printf(dev, "controller is not in host mode\n");
		error = ENXIO;
		goto out;
	}
	reg = bus_read_4(mem, DWC3_GRXTHRCFG);
	if ((reg & sel) != 0) {
		if (bootverbose)
			device_printf(dev, "disabling DWC3 RX threshold "
			    "(GRXTHRCFG 0x%08x)\n", reg);
		bus_write_4(mem, DWC3_GRXTHRCFG, reg & ~sel);
	}
out:
	bus_release_resource(dev, SYS_RES_MEMORY, rid, mem);
	return (error);
}

static int
generic_xhci_acpi_attach(device_t dev)
{
	struct resource_list *rl;
	struct urs_irq irq;
	device_t bus;
	int error, irqno;

	if (generic_xhci_acpi_is_urs(dev)) {
		error = generic_xhci_acpi_urs_setup(dev);
		if (error != 0)
			return (error);
		if (urs_host_irq(dev, &irq) != 0)
			return (ENXIO);
		/*
		 * Map the child's interrupt with its own trigger and
		 * polarity; bus_set_resource() would look them up in the
		 * role switch's _CRS.
		 */
		bus = device_get_parent(dev);
		rl = BUS_GET_RESOURCE_LIST(bus, dev);
		if (rl == NULL)
			return (ENXIO);
		irqno = ACPI_BUS_MAP_INTR(bus, dev, irq.gsiv, irq.trig,
		    irq.pol);
		resource_list_add(rl, SYS_RES_IRQ, 0, irqno, irqno, 1);
	}
	return (generic_xhci_attach(dev));
}

static device_method_t xhci_acpi_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, generic_xhci_acpi_probe),
	DEVMETHOD(device_attach, generic_xhci_acpi_attach),

	DEVMETHOD_END
};

DEFINE_CLASS_1(xhci, xhci_acpi_driver, xhci_acpi_methods,
    sizeof(struct xhci_softc), generic_xhci_driver);

DRIVER_MODULE(xhci, acpi, xhci_acpi_driver, 0, 0);
MODULE_DEPEND(xhci, usb, 1, 1, 1);
