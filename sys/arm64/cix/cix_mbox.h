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

#ifndef _ARM64_CIX_CIX_MBOX_H_
#define	_ARM64_CIX_CIX_MBOX_H_

#define	CIX_MBOX_TX	0		/* "cix,mbox_dir": sends from here */
#define	CIX_MBOX_RX	1

/* The attached mailbox an ACPI reference names, or NULL. */
device_t	cix_mbox_get(ACPI_HANDLE h);
int		cix_mbox_dir(device_t dev);
/* Physical address of the message registers (an SCMI shared memory). */
bus_addr_t	cix_mbox_base(device_t dev);
/* Ring a sending mailbox's doorbell. */
void		cix_mbox_ring(device_t dev);
/* The 128 bytes of message registers, 32 bits at a time. */
uint32_t	cix_mbox_msg_read(device_t dev, bus_size_t off);
void		cix_mbox_msg_write(device_t dev, bus_size_t off, uint32_t val);

#endif /* !_ARM64_CIX_CIX_MBOX_H_ */
