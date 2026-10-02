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
 * The SoC's identity, as the boot firmware leaves it in SMEM (socinfo,
 * item 137), under hw.soc: the attributes of Linux's /sys/devices/soc0
 * (family, machine, soc_id, revision), which linsysfs shows to Linux
 * programs; Qualcomm's runtimes pick their code by soc_id.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/sbuf.h>
#include <sys/sysctl.h>

#include <dev/qcom_glink/qcom_smem.h>

#define	SMEM_SOCINFO	137

struct socinfo {
	uint32_t	fmt;
	uint32_t	id;
	uint32_t	ver;		/* major << 16 | minor */
};

static const struct {
	uint32_t	id;
	const char	*name;
} socinfo_names[] = {
	{ 449, "SC8280XP" },
	{ 460, "SA8295P" },
	{ 461, "SA8540P" },
};

static int
socinfo_get(struct socinfo *si)
{
	void *p;
	size_t size;
	int error;

	error = qcom_smem_get(QCOM_SMEM_HOST_APPS, SMEM_SOCINFO, &p, &size);
	if (error != 0)
		return (ENOENT);
	if (size < sizeof(*si))
		return (ENOENT);
	memcpy(si, p, sizeof(*si));
	return (0);
}

enum { SOC_FAMILY, SOC_MACHINE, SOC_ID, SOC_REVISION };

static int
socinfo_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct socinfo si;
	struct sbuf sb;
	u_int i;
	int error;

	error = socinfo_get(&si);
	if (error != 0)
		return (error);
	sbuf_new_for_sysctl(&sb, NULL, 32, req);
	switch (arg2) {
	case SOC_FAMILY:
		sbuf_cat(&sb, "Snapdragon");
		break;
	case SOC_MACHINE:
		for (i = 0; i < nitems(socinfo_names); i++)
			if (socinfo_names[i].id == si.id)
				break;
		if (i == nitems(socinfo_names)) {
			sbuf_delete(&sb);
			return (ENOENT);
		}
		sbuf_cat(&sb, socinfo_names[i].name);
		break;
	case SOC_ID:
		sbuf_printf(&sb, "%u", si.id);
		break;
	case SOC_REVISION:
		sbuf_printf(&sb, "%u.%u", si.ver >> 16, si.ver & 0xffff);
		break;
	}
	error = sbuf_finish(&sb);
	sbuf_delete(&sb);
	return (error);
}

SYSCTL_NODE(_hw, OID_AUTO, soc, CTLFLAG_RD | CTLFLAG_MPSAFE, NULL,
    "The SoC, as Linux's /sys/devices/soc0 describes it");
SYSCTL_PROC(_hw_soc, OID_AUTO, family, CTLTYPE_STRING | CTLFLAG_RD |
    CTLFLAG_MPSAFE, NULL, SOC_FAMILY, socinfo_sysctl, "A", "SoC family");
SYSCTL_PROC(_hw_soc, OID_AUTO, machine, CTLTYPE_STRING | CTLFLAG_RD |
    CTLFLAG_MPSAFE, NULL, SOC_MACHINE, socinfo_sysctl, "A", "SoC name");
SYSCTL_PROC(_hw_soc, OID_AUTO, soc_id, CTLTYPE_STRING | CTLFLAG_RD |
    CTLFLAG_MPSAFE, NULL, SOC_ID, socinfo_sysctl, "A", "SoC ID");
SYSCTL_PROC(_hw_soc, OID_AUTO, revision, CTLTYPE_STRING | CTLFLAG_RD |
    CTLFLAG_MPSAFE, NULL, SOC_REVISION, socinfo_sysctl, "A",
    "SoC revision");
