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

#ifndef _DEV_QCOM_CMD_DB_QCOM_CMD_DB_H_
#define	_DEV_QCOM_CMD_DB_QCOM_CMD_DB_H_

/*
 * Qualcomm Command DB: the read-only database, written by the AOP firmware,
 * that maps RPMh resource names ("gfx.lvl", "SH0", ...) to their addresses
 * and holds data about them, such as the voltage levels of a rail.
 */

/* Resource types (the slave ID in bits 19:16 of an address). */
#define	QCOM_CMD_DB_HW_ARC	3	/* power rails, by level */
#define	QCOM_CMD_DB_HW_VRM	4	/* regulators */
#define	QCOM_CMD_DB_HW_BCM	5	/* bus bandwidth */

/* 0 if the database was found and is valid, else an errno value. */
int		qcom_cmd_db_ready(void);
/* A resource's RPMh address, or 0 if there is none. */
uint32_t	qcom_cmd_db_read_addr(const char *id);
/* A resource's data, valid for the life of the system; NULL if none. */
const void	*qcom_cmd_db_read_aux_data(const char *id, size_t *len);
/* A resource's type, or -1 if there is no such resource. */
int		qcom_cmd_db_read_slave_id(const char *id);

#endif /* _DEV_QCOM_CMD_DB_QCOM_CMD_DB_H_ */
