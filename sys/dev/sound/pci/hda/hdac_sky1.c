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
 * The HDA controller of CIX Sky1 (ACPI CIXH6020), in its audio subsystem
 * (sky1_audss):
 * - it reaches memory through a window, CPU 0x90000000 at its address 0,
 *   2 GB, and does not snoop the CPU's caches;
 * - it takes only 32-bit register accesses;
 * - its response interrupt status (RIRBSTS.RINTFL) does not clear, so it
 *   runs polled, without interrupts;
 * - a board may configure its codec (pins and vendor settings) before the
 *   codec is probed, from the ACPI model name (_DSD cix,model).
 */

#ifdef HAVE_KERNEL_OPTION_HEADERS
#include "opt_snd.h"
#endif

#include <dev/sound/pcm/sound.h>

#include <sys/ctype.h>
#include <sys/taskqueue.h>

#include <machine/busdma_window.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include <dev/sound/pci/hda/hdac_private.h>
#include <dev/sound/pci/hda/hdac_reg.h>
#include <dev/sound/pci/hda/hda_reg.h>
#include <dev/sound/pci/hda/hdac.h>

#include <arm64/cix/sky1_audss.h>

#define	HDAC_SKY1_DMA_CPU_BASE	0x90000000UL
#define	HDAC_SKY1_DMA_BUS_BASE	0x0UL
#define	HDAC_SKY1_DMA_SIZE	0x80000000UL

/*
 * Orange Pi 6 Plus (ALC269VC): the codec's subsystem id, its pins
 * (internal speaker 0x14, headphone 0x15, mic 0x18, internal mic 0x19),
 * and the vendor's settings (processing coefficients of widget 0x20).
 */
static const uint32_t hdac_sky1_orangepi6plus_verbs[] = {
	/* Subsystem id 0x10ec129e */
	0x0017209e, 0x00172112, 0x001722ec, 0x00172310,
	0x0017ff00, 0x0017ff00, 0x0017ff00, 0x0017ff00,
	/* Pin configuration defaults */
	0x01271c00, 0x01271d00, 0x01271e00, 0x01271f40,	/* 0x12 */
	0x01471c10, 0x01471d01, 0x01471e17, 0x01471f90,	/* 0x14 */
	0x01571c1f, 0x01571d10, 0x01571e21, 0x01571f04,	/* 0x15 */
	0x01771cf0, 0x01771d11, 0x01771e11, 0x01771f41,	/* 0x17 */
	0x01871c20, 0x01871d10, 0x01871ea1, 0x01871f04,	/* 0x18 */
	0x01971c2f, 0x01971d01, 0x01971ea7, 0x01971f90,	/* 0x19 */
	0x01a71cf0, 0x01a71d11, 0x01a71e11, 0x01a71f41,	/* 0x1a */
	0x01b71cf0, 0x01b71d11, 0x01b71e11, 0x01b71f41,	/* 0x1b */
	0x01d71c05, 0x01d71d82, 0x01d71e53, 0x01d71f40,	/* 0x1d */
	0x01e71cf0, 0x01e71d11, 0x01e71e11, 0x01e71f41,	/* 0x1e */
	/* Widget 0x20 coefficients */
	0x02050018, 0x02040184, 0x0205001c, 0x02040800,
	0x02050024, 0x02040000, 0x02050004, 0x02040080,
	0x02050008, 0x02040300, 0x0205000c, 0x02043f00,
	0x02050015, 0x02048002, 0x02050015, 0x02048002,
	/* Widgets 0x0c and 0x0d */
	0x00c37080, 0x00270610, 0x00d37080, 0x00370610,
};

static const struct {
	const char	*model;
	const uint32_t	*verbs;
	int		 nverbs;
} hdac_sky1_boards[] = {
	{ "CIX SKY1 ORAPI 6P HDA", hdac_sky1_orangepi6plus_verbs,
	    nitems(hdac_sky1_orangepi6plus_verbs) },
};

static char *hdac_sky1_ids[] = { "CIXH6020", NULL };

static int
hdac_sky1_probe(device_t dev)
{
	int rv;

	rv = ACPI_ID_PROBE(device_get_parent(dev), dev, hdac_sky1_ids, NULL);
	if (rv <= 0)
		device_set_desc(dev, "CIX Sky1 HDA controller");
	return (rv);
}

static int
hdac_sky1_attach(device_t dev)
{
	struct hdac_softc *sc;
	const ACPI_OBJECT *obj;
	int error;
	u_int i;

	sc = device_get_softc(dev);

	/* Its clocks, and out of reset (the subsystem is up, or ENXIO). */
	if ((error = sky1_audss_gate(SKY1_AUDSS_HDA, true)) != 0) {
		device_printf(dev, "audio subsystem not up\n");
		return (error);
	}
	sky1_audss_reset(SKY1_AUDSS_HDA, true);
	DELAY(20);
	sky1_audss_reset(SKY1_AUDSS_HDA, false);

	error = bus_dma_window_tag_create(bus_get_dma_tag(dev),
	    HDAC_SKY1_DMA_CPU_BASE, HDAC_SKY1_DMA_BUS_BASE, HDAC_SKY1_DMA_SIZE,
	    &sc->dma_parent);
	if (error != 0)
		return (error);

	sc->flags |= HDAC_F_NOT_PCI | HDAC_F_RINTFL_STUCK;
	sc->quirks_off |= HDAC_QUIRK_MSI | HDAC_QUIRK_DMAPOS;
	sc->mem.mem_aligned = true;
	sc->polling = 1;

	if (ACPI_SUCCESS(acpi_GetProperty(dev, "cix,model", &obj)) &&
	    obj->Type == ACPI_TYPE_STRING) {
		for (i = 0; i < nitems(hdac_sky1_boards); i++) {
			if (strcmp(obj->String.Pointer,
			    hdac_sky1_boards[i].model) != 0)
				continue;
			sc->init_verbs = hdac_sky1_boards[i].verbs;
			sc->init_nverbs = hdac_sky1_boards[i].nverbs;
			break;
		}
		if (sc->init_verbs == NULL)
			device_printf(dev, "no codec configuration for %s\n",
			    obj->String.Pointer);
	}

	error = hdac_attach_common(dev);
	if (error != 0) {
		bus_dma_tag_destroy(sc->dma_parent);
		sc->dma_parent = NULL;
	}
	return (error);
}

static int
hdac_sky1_detach(device_t dev)
{
	struct hdac_softc *sc;
	int error;

	sc = device_get_softc(dev);
	error = hdac_detach(dev);
	if (error != 0)
		return (error);
	if (sc->dma_parent != NULL)
		bus_dma_tag_destroy(sc->dma_parent);
	sky1_audss_reset(SKY1_AUDSS_HDA, true);
	sky1_audss_gate(SKY1_AUDSS_HDA, false);
	return (0);
}

static device_method_t hdac_sky1_methods[] = {
	DEVMETHOD(device_probe,		hdac_sky1_probe),
	DEVMETHOD(device_attach,	hdac_sky1_attach),
	DEVMETHOD(device_detach,	hdac_sky1_detach),
	DEVMETHOD_END
};

DEFINE_CLASS_1(hdac, hdac_sky1_driver, hdac_sky1_methods,
    sizeof(struct hdac_softc), hdac_driver);

DRIVER_MODULE(hdac_sky1, acpi, hdac_sky1_driver, NULL, NULL);
MODULE_DEPEND(hdac_sky1, sky1_audss, 1, 1, 1);
