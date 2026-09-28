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

#include "generic_xhci.h"

static char *xhci_ids[] = {
	"PNP0D10",
	"PNP0D15",
	NULL,
};

/*
 * A USB Role Switch device holds a dual-role controller's registers; its
 * host-role child, with _ADR 0, holds the host controller's interrupts.
 */
static char *urs_ids[] = {
	"PNP0CA1",
	NULL,
};

#define	URS_HOST_ADR	0

struct urs_irq_arg {
	UINT32	gsiv;
	bool	found;
};

static ACPI_STATUS
urs_find_irq(ACPI_RESOURCE *res, void *context)
{
	struct urs_irq_arg *arg = context;

	if (arg->found)
		return (AE_OK);
	if (res->Type == ACPI_RESOURCE_TYPE_EXTENDED_IRQ &&
	    res->Data.ExtendedIrq.InterruptCount > 0) {
		arg->gsiv = res->Data.ExtendedIrq.Interrupts[0];
		arg->found = true;
	} else if (res->Type == ACPI_RESOURCE_TYPE_IRQ &&
	    res->Data.Irq.InterruptCount > 0) {
		arg->gsiv = res->Data.Irq.Interrupts[0];
		arg->found = true;
	}
	return (AE_OK);
}

/* Find the first interrupt of the role switch's host-role child. */
static int
urs_host_irq(device_t dev, UINT32 *gsiv)
{
	struct urs_irq_arg arg;
	ACPI_HANDLE child;
	UINT32 adr;

	child = NULL;
	while (ACPI_SUCCESS(AcpiGetNextObject(ACPI_TYPE_DEVICE,
	    acpi_get_handle(dev), child, &child))) {
		if (ACPI_FAILURE(acpi_GetInteger(child, "_ADR", &adr)) ||
		    adr != URS_HOST_ADR)
			continue;
		arg.found = false;
		if (ACPI_SUCCESS(AcpiWalkResources(child, "_CRS", urs_find_irq,
		    &arg)) && arg.found) {
			*gsiv = arg.gsiv;
			return (0);
		}
	}
	return (ENXIO);
}

static int
generic_xhci_acpi_probe(device_t dev)
{
	UINT32 gsiv;

	if (ACPI_ID_PROBE(device_get_parent(dev), dev, xhci_ids, NULL) >= 0) {
		if (ACPI_ID_PROBE(device_get_parent(dev), dev, urs_ids,
		    NULL) >= 0 || urs_host_irq(dev, &gsiv) != 0)
			return (ENXIO);
	}

	device_set_desc(dev, XHCI_HC_DEVSTR);

	return (BUS_PROBE_GENERIC);
}

/* Synopsys DWC_usb3x global registers, after the xHCI registers. */
#define	DWC3_GSNPSID		0xc120
#define	 DWC3_GSNPSID_USB3	0x5533
#define	 DWC3_GSNPSID_USB31	0x3331
#define	 DWC3_GSNPSID_USB32	0x3332
#define	DWC3_GRXTHRCFG		0xc10c
#define	 DWC3_GRXTHRCFG_PKTCNTSEL	(1u << 29)
#define	 DWC31_GRXTHRCFG_PKTCNTSEL	(1u << 26)
#define	DWC3_MIN_SIZE		0xc200

/*
 * Firmware may leave a DWC3 core's receive threshold enabled, which on
 * the SC8280XP's USB-C controllers cuts SuperSpeed reads to a third.
 * Disable it, as the core's reset default does.
 */
static void
generic_xhci_acpi_dwc3_fixup(device_t dev)
{
	struct resource *mem;
	uint32_t id, reg, sel;
	int rid;

	rid = 0;
	mem = bus_alloc_resource_any(dev, SYS_RES_MEMORY, &rid, RF_ACTIVE);
	if (mem == NULL)
		return;
	if (rman_get_size(mem) < DWC3_MIN_SIZE)
		goto out;
	id = bus_read_4(mem, DWC3_GSNPSID) >> 16;
	if (id == DWC3_GSNPSID_USB3)
		sel = DWC3_GRXTHRCFG_PKTCNTSEL;
	else if (id == DWC3_GSNPSID_USB31 || id == DWC3_GSNPSID_USB32)
		sel = DWC31_GRXTHRCFG_PKTCNTSEL;
	else
		goto out;
	reg = bus_read_4(mem, DWC3_GRXTHRCFG);
	if ((reg & sel) != 0) {
		if (bootverbose)
			device_printf(dev, "disabling DWC3 RX threshold "
			    "(GRXTHRCFG 0x%08x)\n", reg);
		bus_write_4(mem, DWC3_GRXTHRCFG, reg & ~sel);
	}
out:
	bus_release_resource(dev, SYS_RES_MEMORY, rid, mem);
}

static int
generic_xhci_acpi_attach(device_t dev)
{
	UINT32 gsiv;

	/*
	 * A role switch's interrupts belong to its host-role child.  The
	 * controller is used as firmware left it, which must be host mode.
	 */
	if (ACPI_ID_PROBE(device_get_parent(dev), dev, urs_ids, NULL) <= 0) {
		if (urs_host_irq(dev, &gsiv) != 0)
			return (ENXIO);
		bus_set_resource(dev, SYS_RES_IRQ, 0, gsiv, 1);
	}
	generic_xhci_acpi_dwc3_fixup(dev);
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
