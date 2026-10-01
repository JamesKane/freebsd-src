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
 * Qualcomm DSP loader: starts the audio DSP (ADSP) and the compute DSP
 * (CDSP, home of the NPU) with the board's firmware, through the TrustZone
 * peripheral authentication service (PAS), as Linux's qcom_q6v5_pas and
 * mdt_loader do.
 *
 * The firmware is an ELF image with Qualcomm's program header flags.  Its
 * metadata, the ELF and program headers followed by the hash segment, goes
 * to TrustZone first; its loadable segments are then copied into the DSP's
 * reserved memory, relocated to its start, and TrustZone authenticates them
 * and releases the DSP from reset.  The firmware is board-specific: boards
 * are matched by their SMBIOS names, and each DSP by its ACPI ID.  It is
 * read once the root filesystem is mounted, from a firmware(9) module named
 * after its path in linux-firmware with '/', '.' and '-' turned into '_', or
 * from /boot/firmware, and once qcom_scm(4) has attached, which its module
 * may do after this one's.
 *
 * Nothing here talks to a DSP once it runs: its own power votes, its GLINK
 * services and its crash notifications are not handled, and it can't be
 * stopped, which needs its stop handshake; the driver stays attached.  On
 * the Radxa Dragon Q8B, Radxa's firmware controls the fan by itself;
 * without the ADSP the fan runs at full speed.
 */

#include "opt_acpi.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/elf32.h>
#include <sys/eventhandler.h>
#include <sys/firmware.h>
#include <sys/kernel.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/sbuf.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include <dev/qcom_glink/qcom_aoss.h>
#include <dev/qcom_rpmh/qcom_rpmh.h>
#include <dev/qcom_scm/qcom_scm.h>

/* Qualcomm's segment flags: the hash segment, and relocatable segments. */
#define	QCOM_MDT_TYPE_MASK	(7u << 24)
#define	QCOM_MDT_TYPE_HASH	(2u << 24)
#define	QCOM_MDT_RELOCATABLE	(1u << 27)

struct qcom_adsp_board {
	const char	*maker;		/* SMBIOS system maker and product */
	const char	*product;
	const char	*hid;		/* the DSP's ACPI ID */
	const char	*name;		/* adsp, cdsp */
	const char	*desc;
	const char	*firmware;	/* the path in linux-firmware */
	const char	*load_state;	/* the AOSS's name for it, if any */
	const char	*rail;		/* an RPMh rail it needs, if any */
	const char	*bcms[4];	/* its path to memory's BCMs */
	uint32_t	pas_id;
	vm_paddr_t	mem;		/* the reserved memory */
	vm_size_t	mem_size;
};

static const struct qcom_adsp_board qcom_adsp_boards[] = {
	{
		.maker = "Radxa Computer Co., Ltd.",
		.product = "Radxa Dragon Q8B",
		.hid = "QCOM061B",
		.name = "adsp",
		.desc = "Qualcomm audio DSP",
		.firmware = "qcom/sc8280xp/radxa/dragon-q8b/qcadsp8280.mbn",
		.load_state = "adsp",
		.pas_id = 1,
		.mem = 0x86c00000,
		.mem_size = 0x2000000,
	},
	{
		/* NSP0; the SoC's second NSP isn't used here. */
		.maker = "Radxa Computer Co., Ltd.",
		.product = "Radxa Dragon Q8B",
		.hid = "QCOM06B0",
		.name = "cdsp",
		.desc = "Qualcomm compute DSP",
		.firmware = "qcom/sc8280xp/qccdsp8280.mbn",
		.rail = "nsp.lvl",
		/* NSP-A NoC to the memory NoC; SH0 and MC0 are UEFI's. */
		.bcms = { "NSA1", "NSA0" },
		.pas_id = 18,
		.mem = 0x8a100000,
		.mem_size = 0x1e00000,
	},
};

enum qcom_adsp_state {
	QCOM_ADSP_WAITING,		/* for the root filesystem */
	QCOM_ADSP_STARTING,
	QCOM_ADSP_RUNNING,
	QCOM_ADSP_FAILED,
	QCOM_ADSP_STOPPED,
};

struct qcom_adsp_softc {
	device_t			dev;
	const struct qcom_adsp_board	*board;
	struct mtx			mtx;	/* state */
	enum qcom_adsp_state		state;
	int				error;	/* if failed */
	int				scm_wait; /* seconds left */
	struct timeout_task		start_task;
	eventhandler_tag		mountroot_tag;
};

static char *qcom_adsp_acpi_ids[] = { "QCOM061B", "QCOM06B0", NULL };

/* This board's entry for the DSP with ACPI ID hid. */
static const struct qcom_adsp_board *
qcom_adsp_find_board(const char *hid)
{
	const struct qcom_adsp_board *b;
	char *maker, *product;
	u_int i;

	maker = kern_getenv("smbios.system.maker");
	product = kern_getenv("smbios.system.product");
	b = NULL;
	for (i = 0; maker != NULL && product != NULL &&
	    i < nitems(qcom_adsp_boards); i++) {
		if (strcmp(maker, qcom_adsp_boards[i].maker) == 0 &&
		    strcmp(product, qcom_adsp_boards[i].product) == 0 &&
		    strcmp(hid, qcom_adsp_boards[i].hid) == 0) {
			b = &qcom_adsp_boards[i];
			break;
		}
	}
	freeenv(maker);
	freeenv(product);
	return (b);
}

/* The firmware, from its module or from /boot/firmware. */
static const struct firmware *
qcom_adsp_get_firmware(const char *path)
{
	const struct firmware *fw;
	char *name, *p;

	name = strdup(path, M_TEMP);
	for (p = name; *p != '\0'; p++)
		if (*p == '/' || *p == '.' || *p == '-')
			*p = '_';
	fw = firmware_get_flags(name, FIRMWARE_GET_NOWARN);
	free(name, M_TEMP);
	if (fw == NULL)
		fw = firmware_get(path);
	return (fw);
}

static bool
qcom_adsp_loadable(const Elf32_Phdr *ph)
{
	return (ph->p_type == PT_LOAD && ph->p_memsz != 0 &&
	    (ph->p_flags & QCOM_MDT_TYPE_MASK) != QCOM_MDT_TYPE_HASH);
}

/* Load the image into the DSP's memory and start it. */
static int
qcom_adsp_boot(struct qcom_adsp_softc *sc, const struct firmware *fw)
{
	const struct qcom_adsp_board *b = sc->board;
	const Elf32_Ehdr *eh;
	const Elf32_Phdr *ph;
	const char *data;
	char *md, *mem;
	size_t mdlen, hoff, off;
	uint32_t base, end;
	bool relocate;
	int error, hash, i;

	data = fw->data;
	eh = (const Elf32_Ehdr *)data;
	if (fw->datasize < sizeof(*eh) ||
	    memcmp(eh->e_ident, ELFMAG, SELFMAG) != 0 ||
	    eh->e_ident[EI_CLASS] != ELFCLASS32 ||
	    eh->e_phentsize != sizeof(Elf32_Phdr) || eh->e_phnum < 2 ||
	    eh->e_phoff + (size_t)eh->e_phnum * sizeof(Elf32_Phdr) >
	    fw->datasize)
		return (EFTYPE);
	ph = (const Elf32_Phdr *)(data + eh->e_phoff);

	/* The metadata: the headers, then the hash segment. */
	for (hash = 1; hash < eh->e_phnum; hash++)
		if ((ph[hash].p_flags & QCOM_MDT_TYPE_MASK) ==
		    QCOM_MDT_TYPE_HASH)
			break;
	if (hash == eh->e_phnum)
		return (EFTYPE);
	mdlen = ph[0].p_filesz + ph[hash].p_filesz;
	hoff = mdlen == fw->datasize ? ph[0].p_filesz : ph[hash].p_offset;
	if (ph[0].p_filesz > fw->datasize ||
	    hoff + ph[hash].p_filesz > fw->datasize)
		return (EFTYPE);

	/* Where the image goes: relocatable images start the memory. */
	relocate = false;
	base = UINT32_MAX;
	end = 0;
	for (i = 0; i < eh->e_phnum; i++) {
		if (!qcom_adsp_loadable(&ph[i]))
			continue;
		if ((ph[i].p_flags & QCOM_MDT_RELOCATABLE) != 0)
			relocate = true;
		base = MIN(base, ph[i].p_paddr);
		end = MAX(end, roundup2(ph[i].p_paddr + ph[i].p_memsz,
		    PAGE_SIZE));
	}
	if (end == 0)
		return (EFTYPE);
	if (!relocate)
		base = b->mem;
	for (i = 0; i < eh->e_phnum; i++) {
		if (!qcom_adsp_loadable(&ph[i]))
			continue;
		off = ph[i].p_paddr - base;
		if (ph[i].p_paddr < base || off + ph[i].p_memsz > b->mem_size ||
		    ph[i].p_filesz > ph[i].p_memsz || (ph[i].p_filesz != 0 &&
		    ph[i].p_offset + ph[i].p_filesz > fw->datasize))
			return (EFTYPE);
	}

	/*
	 * The DSP's rail at its highest level, as Linux's proxy vote holds
	 * it while the DSP boots.  Linux drops it once the DSP has taken
	 * over its own votes; here it stays.
	 */
	if (b->rail != NULL) {
		error = qcom_rpmh_arc_vote(b->rail, QCOM_RPMH_ARC_MAX);
		if (error != 0) {
			device_printf(sc->dev, "no vote for %s: %d\n", b->rail,
			    error);
			return (error);
		}
	}
	/* And its path to memory, at full peak, as Linux's proxy vote. */
	for (i = 0; i < (int)nitems(b->bcms) && b->bcms[i] != NULL; i++) {
		error = qcom_rpmh_bcm_vote(b->bcms[i], 0, QCOM_RPMH_BCM_MAX);
		if (error != 0) {
			device_printf(sc->dev, "no vote for %s: %d\n",
			    b->bcms[i], error);
			return (error);
		}
	}

	/*
	 * Tell the AOSS the image is going in, as Linux does before loading
	 * those DSPs that have a load state: it holds resources for the
	 * subsystem from then on.
	 */
	if (b->load_state != NULL) {
		char msg[64];

		snprintf(msg, sizeof(msg),
		    "{class: image, res: load_state, name: %s, val: on}",
		    b->load_state);
		error = qcom_aoss_send(msg);
		if (error != 0)
			device_printf(sc->dev, "AOSS load state: %d\n", error);
	}

	md = malloc(mdlen, M_TEMP, M_WAITOK);
	memcpy(md, data, ph[0].p_filesz);
	memcpy(md + ph[0].p_filesz, data + hoff, ph[hash].p_filesz);
	error = qcom_scm_pas_init_image(b->pas_id, md, mdlen);
	free(md, M_TEMP);
	if (error != 0) {
		device_printf(sc->dev, "firmware metadata refused: %d\n",
		    error);
		return (error);
	}
	if (relocate) {
		error = qcom_scm_pas_mem_setup(b->pas_id, b->mem, end - base);
		if (error != 0) {
			device_printf(sc->dev, "relocation refused: %d\n",
			    error);
			goto fail;
		}
	}

	mem = pmap_mapdev_attr(b->mem, b->mem_size,
	    VM_MEMATTR_WRITE_COMBINING);
	for (i = 0; i < eh->e_phnum; i++) {
		if (!qcom_adsp_loadable(&ph[i]))
			continue;
		off = ph[i].p_paddr - base;
		memcpy(mem + off, data + ph[i].p_offset, ph[i].p_filesz);
		memset(mem + off + ph[i].p_filesz, 0,
		    ph[i].p_memsz - ph[i].p_filesz);
	}
	dsb(sy);
	pmap_unmapdev(mem, b->mem_size);

	error = qcom_scm_pas_auth_and_reset(b->pas_id);
	if (error == 0)
		return (0);
	device_printf(sc->dev, "firmware authentication failed: %d\n", error);
fail:
	(void)qcom_scm_pas_shutdown(b->pas_id);
	return (error);
}

static void
qcom_adsp_start(void *arg, int pending __unused)
{
	struct qcom_adsp_softc *sc = arg;
	const struct firmware *fw;
	int error;

	mtx_lock(&sc->mtx);
	if (sc->state != QCOM_ADSP_WAITING) {
		mtx_unlock(&sc->mtx);
		return;
	}
	if (!qcom_scm_available() && sc->scm_wait-- > 0) {
		taskqueue_enqueue_timeout(taskqueue_thread, &sc->start_task,
		    hz);
		mtx_unlock(&sc->mtx);
		return;
	}
	sc->state = QCOM_ADSP_STARTING;
	mtx_unlock(&sc->mtx);

	fw = NULL;
	if (!qcom_scm_available()) {
		device_printf(sc->dev, "no secure channel manager\n");
		error = ENXIO;
	} else if ((fw = qcom_adsp_get_firmware(sc->board->firmware)) ==
	    NULL) {
		device_printf(sc->dev, "no firmware %s\n",
		    sc->board->firmware);
		error = ENOENT;
	} else {
		if (bootverbose)
			device_printf(sc->dev, "starting at %jd ms\n",
			    (intmax_t)(sbinuptime() / SBT_1MS));
		error = qcom_adsp_boot(sc, fw);
		firmware_put(fw, FIRMWARE_UNLOAD);
	}
	if (error == 0)
		device_printf(sc->dev, "running %s\n", sc->board->firmware);

	mtx_lock(&sc->mtx);
	sc->state = error == 0 ? QCOM_ADSP_RUNNING : QCOM_ADSP_FAILED;
	sc->error = error;
	mtx_unlock(&sc->mtx);
}

static void
qcom_adsp_mountroot(void *arg)
{
	struct qcom_adsp_softc *sc = arg;

	taskqueue_enqueue_timeout(taskqueue_thread, &sc->start_task, 0);
}

static int
qcom_adsp_state_sysctl(SYSCTL_HANDLER_ARGS)
{
	static const char *const names[] = {
		[QCOM_ADSP_WAITING] = "waiting for the root filesystem or SCM",
		[QCOM_ADSP_STARTING] = "starting",
		[QCOM_ADSP_RUNNING] = "running",
		[QCOM_ADSP_FAILED] = "failed",
		[QCOM_ADSP_STOPPED] = "stopped",
	};
	struct qcom_adsp_softc *sc = arg1;
	struct sbuf sb;
	int error;

	sbuf_new_for_sysctl(&sb, NULL, 64, req);
	mtx_lock(&sc->mtx);
	sbuf_cat(&sb, names[sc->state]);
	if (sc->state == QCOM_ADSP_FAILED)
		sbuf_printf(&sb, " (error %d)", sc->error);
	mtx_unlock(&sc->mtx);
	error = sbuf_finish(&sb);
	sbuf_delete(&sb);
	return (error);
}

static int
qcom_adsp_probe(device_t dev)
{
	const struct qcom_adsp_board *b;
	char *hid;

	if (ACPI_ID_PROBE(device_get_parent(dev), dev, qcom_adsp_acpi_ids,
	    &hid) > 0 || (b = qcom_adsp_find_board(hid)) == NULL)
		return (ENXIO);
	device_set_desc(dev, b->desc);
	return (BUS_PROBE_DEFAULT);
}

static int
qcom_adsp_attach(device_t dev)
{
	struct qcom_adsp_softc *sc = device_get_softc(dev);
	char *hid;

	sc->dev = dev;
	(void)ACPI_ID_PROBE(device_get_parent(dev), dev, qcom_adsp_acpi_ids,
	    &hid);
	sc->board = qcom_adsp_find_board(hid);
	mtx_init(&sc->mtx, "qcom_adsp", NULL, MTX_DEF);
	sc->state = QCOM_ADSP_WAITING;
	sc->scm_wait = 60;
	TIMEOUT_TASK_INIT(taskqueue_thread, &sc->start_task, 0,
	    qcom_adsp_start, sc);
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, "state",
	    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, sc, 0,
	    qcom_adsp_state_sysctl, "A", "The DSP's state");

	/* The firmware is on the root filesystem. */
	if (root_mounted())
		taskqueue_enqueue_timeout(taskqueue_thread, &sc->start_task,
		    0);
	else
		sc->mountroot_tag = EVENTHANDLER_REGISTER(mountroot,
		    qcom_adsp_mountroot, sc, EVENTHANDLER_PRI_ANY);
	return (0);
}

static int
qcom_adsp_detach(device_t dev)
{
	struct qcom_adsp_softc *sc = device_get_softc(dev);

	if (sc->mountroot_tag != NULL)
		EVENTHANDLER_DEREGISTER(mountroot, sc->mountroot_tag);
	/* A start still to come won't; one under way finishes first. */
	mtx_lock(&sc->mtx);
	if (sc->state == QCOM_ADSP_WAITING)
		sc->state = QCOM_ADSP_STOPPED;
	mtx_unlock(&sc->mtx);
	taskqueue_drain_timeout(taskqueue_thread, &sc->start_task);
	/*
	 * A running DSP must first be asked to stop, through the stop state
	 * in SMEM and its acknowledgement, which isn't done here: shutting it
	 * down straight away hangs the SoC.  It runs until reset.
	 */
	if (sc->state == QCOM_ADSP_RUNNING)
		return (EBUSY);
	mtx_destroy(&sc->mtx);
	return (0);
}

static device_method_t qcom_adsp_methods[] = {
	DEVMETHOD(device_probe,		qcom_adsp_probe),
	DEVMETHOD(device_attach,	qcom_adsp_attach),
	DEVMETHOD(device_detach,	qcom_adsp_detach),
	DEVMETHOD_END
};

static driver_t qcom_adsp_driver = {
	"qcom_adsp",
	qcom_adsp_methods,
	sizeof(struct qcom_adsp_softc),
};

DRIVER_MODULE(qcom_adsp, acpi, qcom_adsp_driver, 0, 0);
MODULE_DEPEND(qcom_adsp, acpi, 1, 1, 1);
MODULE_DEPEND(qcom_adsp, qcom_scm, 1, 1, 1);
MODULE_DEPEND(qcom_adsp, qcom_glink, 1, 1, 1);
MODULE_DEPEND(qcom_adsp, qcom_rpmh, 1, 1, 1);
MODULE_DEPEND(qcom_adsp, firmware, 1, 1, 1);
MODULE_VERSION(qcom_adsp, 1);
ACPI_PNP_INFO(qcom_adsp_acpi_ids);
