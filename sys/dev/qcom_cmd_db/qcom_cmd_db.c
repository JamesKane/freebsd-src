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
 * Qualcomm Command DB; see qcom_cmd_db.h.
 *
 * The database is in memory reserved for it.  A devicetree describes it with
 * a "qcom,cmd-db" reserved-memory node.  ACPI firmware doesn't describe it,
 * but the AOP firmware also publishes its address and size in a dictionary
 * in its message RAM, at a location that depends on the SoC.
 *
 * Layout (little-endian): a header with a magic number and, for each of up
 * to eight resource types, the offsets of its entries and their data within
 * the data area that follows.  An entry has an eight-character name, an
 * address, and the offset and length of its data.
 */

#include "opt_acpi.h"
#include "opt_platform.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/endian.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/sbuf.h>
#include <sys/sx.h>
#include <sys/sysctl.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include <machine/machdep.h>

#ifdef DEV_ACPI
#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>
#endif

#ifdef FDT
#include <dev/ofw/openfirm.h>
#include <dev/ofw/ofw_bus_subr.h>
#include <dev/ofw/ofw_subr.h>
#endif

#include <dev/qcom_cmd_db/qcom_cmd_db.h>

#define	CMD_DB_MAGIC		0x0c0330db
#define	CMD_DB_MAGIC_OFF	4
#define	CMD_DB_NRSC		8
#define	CMD_DB_RSC_OFF		8	/* resource headers, 16 bytes each */
#define	CMD_DB_RSC_SIZE		16
#define	CMD_DB_DATA_OFF		144	/* after the headers and checksum */

/* Resource header fields. */
#define	RSC_HEADER_OFFSET	2
#define	RSC_DATA_OFFSET		4
#define	RSC_COUNT		6

/* Entry fields. */
#define	ENT_SIZE		24
#define	ENT_ID_LEN		8
#define	ENT_ADDR		16
#define	ENT_LEN			20
#define	ENT_DATA_OFFSET		22

#define	CMD_DB_SLAVE_ID(addr)	(((addr) >> 16) & 0xf)

static struct sx cmd_db_lock;
SX_SYSINIT(qcom_cmd_db, &cmd_db_lock, "qcom_cmd_db");

static const uint8_t	*cmd_db;	/* NULL until found */
static size_t		cmd_db_size;
static int		cmd_db_error = -1;	/* -1: not looked for yet */

#ifdef DEV_ACPI
/* Where the AOP publishes the database's address and size. */
static const struct {
	const char	*pep_hid;
	vm_paddr_t	dict;
} cmd_db_acpi_socs[] = {
	{ "QCOM0617", 0xc3f000c },	/* SC8280XP */
};

static int
cmd_db_find_acpi(vm_paddr_t *pa, size_t *size)
{
	volatile uint32_t *dict;
	ACPI_HANDLE pep;
	u_int i;

	if (ACPI_FAILURE(AcpiGetHandle(NULL, "\\_SB.PEP0", &pep)))
		return (ENOENT);
	for (i = 0; i < nitems(cmd_db_acpi_socs); i++)
		if (acpi_MatchHid(pep, cmd_db_acpi_socs[i].pep_hid) ==
		    ACPI_MATCHHID_HID)
			break;
	if (i == nitems(cmd_db_acpi_socs))
		return (ENOENT);
	dict = pmap_mapdev(cmd_db_acpi_socs[i].dict, 2 * sizeof(uint32_t));
	*pa = dict[0];
	*size = dict[1];
	pmap_unmapdev(__DEVOLATILE(void *, dict), 2 * sizeof(uint32_t));
	return (*pa != 0 && *size != 0 ? 0 : ENOENT);
}
#endif

#ifdef FDT
static int
cmd_db_find_fdt(vm_paddr_t *pa, size_t *size)
{
	phandle_t node, child;
	bus_addr_t addr;
	bus_size_t len;

	node = OF_finddevice("/reserved-memory");
	if (node == -1)
		return (ENOENT);
	for (child = OF_child(node); child != 0; child = OF_peer(child)) {
		if (!ofw_bus_node_is_compatible(child, "qcom,cmd-db"))
			continue;
		if (ofw_reg_to_paddr(child, 0, &addr, &len, NULL) != 0)
			return (ENXIO);
		*pa = addr;
		*size = len;
		return (0);
	}
	return (ENOENT);
}
#endif

/* Resource i's header, its first entry and its number of entries. */
static const uint8_t *
cmd_db_rsc(const uint8_t *db, u_int i, const uint8_t **ent, u_int *cnt)
{
	const uint8_t *rsc;

	rsc = db + CMD_DB_RSC_OFF + i * CMD_DB_RSC_SIZE;
	*ent = db + CMD_DB_DATA_OFF + le16dec(rsc + RSC_HEADER_OFFSET);
	*cnt = le16dec(rsc + RSC_COUNT);
	return (rsc);
}

/* Check that every resource's entries lie within the database. */
static bool
cmd_db_valid(const uint8_t *db, size_t size)
{
	const uint8_t *ent;
	u_int i, cnt;

	if (size < CMD_DB_DATA_OFF ||
	    le32dec(db + CMD_DB_MAGIC_OFF) != CMD_DB_MAGIC)
		return (false);
	for (i = 0; i < CMD_DB_NRSC; i++) {
		(void)cmd_db_rsc(db, i, &ent, &cnt);
		if ((size_t)(ent - db) + (size_t)cnt * ENT_SIZE > size)
			return (false);
	}
	return (true);
}

static int
cmd_db_init_locked(void)
{
	vm_paddr_t pa;
	size_t size;
	void *db;
	int error;

	sx_assert(&cmd_db_lock, SA_XLOCKED);
	if (cmd_db_error >= 0)
		return (cmd_db_error);
	error = ENOENT;
#ifdef DEV_ACPI
	if (arm64_bus_method == ARM64_BUS_ACPI)
		error = cmd_db_find_acpi(&pa, &size);
#endif
#ifdef FDT
	if (arm64_bus_method == ARM64_BUS_FDT)
		error = cmd_db_find_fdt(&pa, &size);
#endif
	if (error == 0) {
		db = pmap_mapdev_attr(pa, size, VM_MEMATTR_WRITE_BACK);
		if (cmd_db_valid(db, size)) {
			cmd_db = db;
			cmd_db_size = size;
		} else {
			printf("qcom_cmd_db: invalid database at %#jx\n",
			    (uintmax_t)pa);
			pmap_unmapdev(db, size);
			error = EINVAL;
		}
	}
	cmd_db_error = error;
	return (error);
}

int
qcom_cmd_db_ready(void)
{
	int error;

	sx_xlock(&cmd_db_lock);
	error = cmd_db_init_locked();
	sx_xunlock(&cmd_db_lock);
	return (error);
}

/* Find an entry by name; if rscp is not NULL, also its resource header. */
static const uint8_t *
cmd_db_find(const char *id, const uint8_t **rscp)
{
	char key[ENT_ID_LEN];
	const uint8_t *rsc, *ent;
	u_int i, j, cnt;

	if (qcom_cmd_db_ready() != 0)
		return (NULL);
	/* Names are zero padded, not terminated, when eight long. */
	memset(key, 0, sizeof(key));
	strncpy(key, id, sizeof(key));
	for (i = 0; i < CMD_DB_NRSC; i++) {
		rsc = cmd_db_rsc(cmd_db, i, &ent, &cnt);
		for (j = 0; j < cnt; j++, ent += ENT_SIZE) {
			if (memcmp(ent, key, ENT_ID_LEN) == 0) {
				if (rscp != NULL)
					*rscp = rsc;
				return (ent);
			}
		}
	}
	return (NULL);
}

uint32_t
qcom_cmd_db_read_addr(const char *id)
{
	const uint8_t *ent;

	ent = cmd_db_find(id, NULL);
	return (ent != NULL ? le32dec(ent + ENT_ADDR) : 0);
}

const void *
qcom_cmd_db_read_aux_data(const char *id, size_t *len)
{
	const uint8_t *ent, *rsc;
	size_t off;

	ent = cmd_db_find(id, &rsc);
	if (ent == NULL)
		return (NULL);
	off = CMD_DB_DATA_OFF + le16dec(rsc + RSC_DATA_OFFSET) +
	    le16dec(ent + ENT_DATA_OFFSET);
	*len = le16dec(ent + ENT_LEN);
	if (*len == 0 || off + *len > cmd_db_size)
		return (NULL);
	return (cmd_db + off);
}

int
qcom_cmd_db_read_slave_id(const char *id)
{
	const uint8_t *ent;

	ent = cmd_db_find(id, NULL);
	return (ent != NULL ? CMD_DB_SLAVE_ID(le32dec(ent + ENT_ADDR)) : -1);
}

/* hw.qcom_cmd_db.entries: every entry, for debugging. */
static int
cmd_db_sysctl_entries(SYSCTL_HANDLER_ARGS)
{
	const uint8_t *ent;
	struct sbuf sb;
	u_int i, j, cnt;
	int error;

	if ((error = qcom_cmd_db_ready()) != 0)
		return (error);
	sbuf_new_for_sysctl(&sb, NULL, 1024, req);
	for (i = 0; i < CMD_DB_NRSC; i++) {
		(void)cmd_db_rsc(cmd_db, i, &ent, &cnt);
		for (j = 0; j < cnt; j++, ent += ENT_SIZE)
			sbuf_printf(&sb, "\n%-8.8s addr %#07x type %u len %u",
			    (const char *)ent, le32dec(ent + ENT_ADDR),
			    CMD_DB_SLAVE_ID(le32dec(ent + ENT_ADDR)),
			    le16dec(ent + ENT_LEN));
	}
	error = sbuf_finish(&sb);
	sbuf_delete(&sb);
	return (error);
}

static SYSCTL_NODE(_hw, OID_AUTO, qcom_cmd_db, CTLFLAG_RD | CTLFLAG_MPSAFE,
    0, "Qualcomm Command DB");
SYSCTL_PROC(_hw_qcom_cmd_db, OID_AUTO, entries,
    CTLTYPE_STRING | CTLFLAG_RD | CTLFLAG_MPSAFE, NULL, 0,
    cmd_db_sysctl_entries, "A", "Command DB entries");

static int
cmd_db_modevent(module_t mod, int type, void *data)
{
	switch (type) {
	case MOD_LOAD:
		return (0);
	case MOD_UNLOAD:
		/* Consumers depend on the module, so nothing still uses it. */
		sx_xlock(&cmd_db_lock);
		if (cmd_db != NULL)
			pmap_unmapdev(__DECONST(void *, cmd_db), cmd_db_size);
		cmd_db = NULL;
		cmd_db_error = -1;
		sx_xunlock(&cmd_db_lock);
		return (0);
	default:
		return (EOPNOTSUPP);
	}
}

static moduledata_t cmd_db_mod = { "qcom_cmd_db", cmd_db_modevent, NULL };
DECLARE_MODULE(qcom_cmd_db, cmd_db_mod, SI_SUB_DRIVERS, SI_ORDER_ANY);
MODULE_VERSION(qcom_cmd_db, 1);
