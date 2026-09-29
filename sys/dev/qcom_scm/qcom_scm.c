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
 * Qualcomm Secure Channel Manager (SCM): calls into the TrustZone firmware
 * using the ARM SMC calling convention, as SiP service calls.
 *
 * A call's function ID encodes a service and a command.  x1 describes the
 * arguments (their number and whether each is a value or a buffer), x2-x5
 * hold up to four of them, and with more than four, x5 instead holds the
 * physical address of a buffer with arguments 4 to 10.  The firmware may
 * return "interrupted" to let an interrupt be handled, and the call is then
 * resumed, or "busy", and it is retried later.
 *
 * Buffers passed to the firmware are physically contiguous, below 4 GB, and
 * written back from the caches first.
 */

#include "opt_acpi.h"
#include "opt_platform.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/sx.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/cpufunc.h>

#include <dev/psci/smccc.h>

#ifdef DEV_ACPI
#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>
#endif

#ifdef FDT
#include <dev/ofw/ofw_bus.h>
#include <dev/ofw/ofw_bus_subr.h>
#endif

#include <dev/qcom_scm/qcom_scm.h>

#define	SCM_FNID(svc, cmd)	(((svc) & 0xff) << 8 | ((cmd) & 0xff))
#define	SCM_CALL_ID(svc, cmd)						\
	SMCCC_FUNC_ID(SMCCC_YIELDING_CALL, SMCCC_64BIT_CALL,		\
	    SMCCC_SIP_SERVICE_CALLS, SCM_FNID((svc), (cmd)))

/* Argument descriptor (x1): count, then two type bits per argument. */
#define	SCM_ARG_VAL		0
#define	SCM_ARG_RO		1
#define	SCM_ARG_RW		2
#define	SCM_ARGINFO(n)		((n) & 0xf)
#define	SCM_ARGINFO_TYPE(i, t)	((t) << (4 + 2 * (i)))

#define	SCM_MAX_ARGS		10
#define	SCM_REG_ARGS		4	/* x2-x5 */
#define	SCM_FIRST_EXT_ARG	3	/* with more, x5 points at args 3-9 */

/* Status returned in x0. */
#define	SCM_SUCCESS		0
#define	SCM_INTERRUPTED		1
#define	SCM_ERROR		(-1)
#define	SCM_EINVAL_ARG		(-2)
#define	SCM_EINVAL_ADDR		(-3)
#define	SCM_EOPNOTSUPP		(-4)
#define	SCM_ENOMEM		(-5)
#define	SCM_EBUSY		(-12)

#define	SCM_BUSY_RETRIES	20
#define	SCM_BUSY_WAIT_MS	30

struct qcom_scm_desc {
	uint32_t	svc;
	uint32_t	cmd;
	uint32_t	arginfo;
	uint64_t	args[SCM_MAX_ARGS];
};

struct qcom_scm_softc {
	device_t	dev;
	struct sx	lock;
	uint64_t	*ext_args;	/* one page for arguments 4-10 */
};

static MALLOC_DEFINE(M_QCOM_SCM, "qcom_scm", "Qualcomm SCM buffers");

static struct qcom_scm_softc *qcom_scm_sc;

static void *
qcom_scm_buf_alloc(size_t len)
{
	return (contigmalloc(len, M_QCOM_SCM, M_WAITOK | M_ZERO, 0,
	    BUS_SPACE_MAXADDR_32BIT, PAGE_SIZE, 0));
}

static void
qcom_scm_buf_free(void *buf)
{
	free(buf, M_QCOM_SCM);
}

static int
qcom_scm_errno(register_t status)
{
	switch ((int)status) {
	case SCM_SUCCESS:
		return (0);
	case SCM_EINVAL_ARG:
	case SCM_EINVAL_ADDR:
		return (EINVAL);
	case SCM_EOPNOTSUPP:
		return (EOPNOTSUPP);
	case SCM_ENOMEM:
		return (ENOMEM);
	case SCM_EBUSY:
		return (EBUSY);
	default:
		return (EIO);
	}
}

/* Make a call; res, if not NULL, gets x1-x3. */
static int
qcom_scm_call(struct qcom_scm_softc *sc, const struct qcom_scm_desc *desc,
    uint64_t *res)
{
	struct arm_smccc_1_2_regs in, out;
	u_int i, nargs, retries;

	nargs = SCM_ARGINFO(desc->arginfo);
	if (nargs > SCM_MAX_ARGS)
		return (EINVAL);

	memset(&in, 0, sizeof(in));
	in.a1 = desc->arginfo;
	in.a2 = desc->args[0];
	in.a3 = desc->args[1];
	in.a4 = desc->args[2];
	in.a5 = desc->args[3];

	sx_xlock(&sc->lock);
	for (retries = 0;; retries++) {
		/* Another call may have used the page while we slept. */
		if (nargs > SCM_REG_ARGS) {
			for (i = SCM_FIRST_EXT_ARG; i < nargs; i++)
				sc->ext_args[i - SCM_FIRST_EXT_ARG] =
				    desc->args[i];
			cpu_dcache_wb_range(sc->ext_args, PAGE_SIZE);
			in.a5 = pmap_kextract((vm_offset_t)sc->ext_args);
		}
		in.a0 = SCM_CALL_ID(desc->svc, desc->cmd);
		in.a6 = 0;
		for (;;) {
			arm_smccc_1_2_smc(&in, &out);
			if (out.a0 != SCM_INTERRUPTED)
				break;
			/* Resume the call; x6 identifies it. */
			in.a0 = SCM_INTERRUPTED;
			in.a6 = out.a6;
		}
		if ((int)out.a0 != SCM_EBUSY || retries == SCM_BUSY_RETRIES)
			break;
		sx_xunlock(&sc->lock);
		pause("scmbusy", MSEC_2_TICKS(SCM_BUSY_WAIT_MS));
		sx_xlock(&sc->lock);
	}
	sx_xunlock(&sc->lock);

	if (res != NULL) {
		res[0] = out.a1;
		res[1] = out.a2;
		res[2] = out.a3;
	}
	return (qcom_scm_errno(out.a0));
}

static bool
qcom_scm_call_available(struct qcom_scm_softc *sc, uint32_t svc, uint32_t cmd)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_INFO,
		.cmd = QCOM_SCM_INFO_IS_CALL_AVAIL,
		.arginfo = SCM_ARGINFO(1),
		.args[0] = SCM_CALL_ID(svc, cmd) & ~(1u << 31 | 1u << 30),
	};
	uint64_t res[3];

	return (qcom_scm_call(sc, &desc, res) == 0 && res[0] != 0);
}

bool
qcom_scm_available(void)
{
	return (qcom_scm_sc != NULL);
}

bool
qcom_scm_is_call_available(uint32_t svc, uint32_t cmd)
{
	if (qcom_scm_sc == NULL)
		return (false);
	return (qcom_scm_call_available(qcom_scm_sc, svc, cmd));
}

bool
qcom_scm_pas_supported(uint32_t pas_id)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_PIL,
		.cmd = QCOM_SCM_PIL_PAS_IS_SUPPORTED,
		.arginfo = SCM_ARGINFO(1),
		.args[0] = pas_id,
	};
	uint64_t res[3];

	if (!qcom_scm_is_call_available(QCOM_SCM_SVC_PIL,
	    QCOM_SCM_PIL_PAS_IS_SUPPORTED))
		return (false);
	return (qcom_scm_call(qcom_scm_sc, &desc, res) == 0 && res[0] != 0);
}

/*
 * Start authenticating a firmware image: the metadata is its ELF and program
 * headers and the hash segment.  The firmware reads it from a copy we keep
 * for the duration of the call.
 */
int
qcom_scm_pas_init_image(uint32_t pas_id, const void *metadata, size_t len)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_PIL,
		.cmd = QCOM_SCM_PIL_PAS_INIT_IMAGE,
		.arginfo = SCM_ARGINFO(2) | SCM_ARGINFO_TYPE(1, SCM_ARG_RW),
		.args[0] = pas_id,
	};
	uint64_t res[3];
	void *buf;
	int error;

	if (qcom_scm_sc == NULL)
		return (ENXIO);
	buf = qcom_scm_buf_alloc(len);
	if (buf == NULL)
		return (ENOMEM);
	memcpy(buf, metadata, len);
	cpu_dcache_wb_range(buf, len);
	desc.args[1] = pmap_kextract((vm_offset_t)buf);
	error = qcom_scm_call(qcom_scm_sc, &desc, res);
	if (error == 0 && res[0] != 0)
		error = EIO;
	qcom_scm_buf_free(buf);
	return (error);
}

/* Tell the firmware where the image is loaded (for relocatable images). */
int
qcom_scm_pas_mem_setup(uint32_t pas_id, vm_paddr_t addr, vm_size_t size)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_PIL,
		.cmd = QCOM_SCM_PIL_PAS_MEM_SETUP,
		.arginfo = SCM_ARGINFO(3),
		.args = { pas_id, addr, size },
	};
	uint64_t res[3];
	int error;

	if (qcom_scm_sc == NULL)
		return (ENXIO);
	error = qcom_scm_call(qcom_scm_sc, &desc, res);
	return (error == 0 && res[0] != 0 ? EIO : error);
}

/* Authenticate the loaded image and start the peripheral. */
int
qcom_scm_pas_auth_and_reset(uint32_t pas_id)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_PIL,
		.cmd = QCOM_SCM_PIL_PAS_AUTH_AND_RESET,
		.arginfo = SCM_ARGINFO(1),
		.args[0] = pas_id,
	};
	uint64_t res[3];
	int error;

	if (qcom_scm_sc == NULL)
		return (ENXIO);
	error = qcom_scm_call(qcom_scm_sc, &desc, res);
	return (error == 0 && res[0] != 0 ? EIO : error);
}

int
qcom_scm_pas_shutdown(uint32_t pas_id)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_PIL,
		.cmd = QCOM_SCM_PIL_PAS_SHUTDOWN,
		.arginfo = SCM_ARGINFO(1),
		.args[0] = pas_id,
	};
	uint64_t res[3];
	int error;

	if (qcom_scm_sc == NULL)
		return (ENXIO);
	error = qcom_scm_call(qcom_scm_sc, &desc, res);
	return (error == 0 && res[0] != 0 ? EIO : error);
}

/* E.g. resume a zap shader loaded earlier: state 0, id 0. */
int
qcom_scm_set_remote_state(uint32_t state, uint32_t id)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_BOOT,
		.cmd = QCOM_SCM_BOOT_SET_REMOTE_STATE,
		.arginfo = SCM_ARGINFO(2),
		.args = { state, id },
	};
	uint64_t res[3];
	int error;

	if (qcom_scm_sc == NULL)
		return (ENXIO);
	error = qcom_scm_call(qcom_scm_sc, &desc, res);
	return (error == 0 && res[0] != 0 ? EIO : error);
}

bool
qcom_scm_set_gpu_smmu_aperture_is_available(void)
{
	return (qcom_scm_is_call_available(QCOM_SCM_SVC_MP,
	    QCOM_SCM_MP_CP_SMMU_APERTURE_ID));
}

/*
 * Let the GPU's command processor switch the page tables of an SMMU context
 * bank, for per-process GPU address spaces.
 */
int
qcom_scm_set_gpu_smmu_aperture(u_int context_bank)
{
	struct qcom_scm_desc desc = {
		.svc = QCOM_SCM_SVC_MP,
		.cmd = QCOM_SCM_MP_CP_SMMU_APERTURE_ID,
		.arginfo = SCM_ARGINFO(4),
		.args = { 0xffff0000 | (context_bank & 0xff), 0xffffffff,
		    0xffffffff, 0xffffffff },
	};

	if (qcom_scm_sc == NULL)
		return (ENXIO);
	return (qcom_scm_call(qcom_scm_sc, &desc, NULL));
}

#ifdef DEV_ACPI
static char *qcom_scm_acpi_ids[] = { "QCOM04DD", NULL };
#endif

#ifdef FDT
static struct ofw_compat_data qcom_scm_compat[] = {
	{ "qcom,scm",	1 },
	{ NULL,		0 }
};
#endif

static int
qcom_scm_probe(device_t dev)
{
	int rv;

	rv = ENXIO;
#ifdef DEV_ACPI
	if (ACPI_ID_PROBE(device_get_parent(dev), dev, qcom_scm_acpi_ids,
	    NULL) <= 0)
		rv = BUS_PROBE_DEFAULT;
#endif
#ifdef FDT
	if (rv != BUS_PROBE_DEFAULT && ofw_bus_status_okay(dev) &&
	    ofw_bus_search_compatible(dev, qcom_scm_compat)->ocd_data != 0)
		rv = BUS_PROBE_DEFAULT;
#endif
	if (rv == BUS_PROBE_DEFAULT)
		device_set_desc(dev, "Qualcomm Secure Channel Manager");
	return (rv);
}

static int
qcom_scm_attach(device_t dev)
{
	struct qcom_scm_softc *sc = device_get_softc(dev);

	if (qcom_scm_sc != NULL) {
		device_printf(dev, "already attached as %s\n",
		    device_get_nameunit(qcom_scm_sc->dev));
		return (ENXIO);
	}
	sc->dev = dev;
	sx_init(&sc->lock, "qcom_scm");
	sc->ext_args = qcom_scm_buf_alloc(PAGE_SIZE);
	if (sc->ext_args == NULL) {
		sx_destroy(&sc->lock);
		return (ENOMEM);
	}

	/*
	 * Only the 64-bit convention is supported on arm64.  Firmware that
	 * implements it reports its own availability query as available.
	 */
	if (!qcom_scm_call_available(sc, QCOM_SCM_SVC_INFO,
	    QCOM_SCM_INFO_IS_CALL_AVAIL)) {
		device_printf(dev, "firmware lacks the SMC64 convention\n");
		qcom_scm_buf_free(sc->ext_args);
		sx_destroy(&sc->lock);
		return (ENXIO);
	}
	qcom_scm_sc = sc;
	return (0);
}

static int
qcom_scm_detach(device_t dev)
{
	struct qcom_scm_softc *sc = device_get_softc(dev);

	/* Consumers depend on the module, so this only runs when unused. */
	qcom_scm_sc = NULL;
	qcom_scm_buf_free(sc->ext_args);
	sx_destroy(&sc->lock);
	return (0);
}

static device_method_t qcom_scm_methods[] = {
	DEVMETHOD(device_probe,		qcom_scm_probe),
	DEVMETHOD(device_attach,	qcom_scm_attach),
	DEVMETHOD(device_detach,	qcom_scm_detach),
	DEVMETHOD_END
};

static driver_t qcom_scm_driver = {
	"qcom_scm",
	qcom_scm_methods,
	sizeof(struct qcom_scm_softc),
};

#ifdef DEV_ACPI
DRIVER_MODULE(qcom_scm, acpi, qcom_scm_driver, 0, 0);
#endif
#ifdef FDT
DRIVER_MODULE(qcom_scm, simplebus, qcom_scm_driver, 0, 0);
DRIVER_MODULE(qcom_scm, ofwbus, qcom_scm_driver, 0, 0);
#endif
MODULE_VERSION(qcom_scm, 1);
