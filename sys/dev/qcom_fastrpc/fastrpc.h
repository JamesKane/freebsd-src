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

#ifndef _DEV_QCOM_FASTRPC_FASTRPC_H_
#define	_DEV_QCOM_FASTRPC_FASTRPC_H_

/*
 * FastRPC: calls from user space into the Qualcomm DSPs, through
 * /dev/fastrpc-cdsp.  The structures and requests are Linux's
 * (include/uapi/misc/fastrpc.h), so that its user space ports unchanged.
 */

#include <sys/types.h>
#include <sys/ioccom.h>

struct fastrpc_invoke_args {
	uint64_t	ptr;
	uint64_t	length;
	int32_t		fd;
	uint32_t	attr;
};

struct fastrpc_invoke {
	uint32_t	handle;
	uint32_t	sc;
	uint64_t	args;		/* struct fastrpc_invoke_args[] */
};

struct fastrpc_init_create {
	uint32_t	filelen;	/* the DSP process's image */
	int32_t		filefd;
	uint32_t	attrs;
	uint32_t	siglen;
	uint64_t	file;
};

struct fastrpc_init_create_static {
	uint32_t	namelen;
	uint32_t	memlen;
	uint64_t	name;
};

struct fastrpc_alloc_dma_buf {
	int32_t		fd;
	uint32_t	flags;
	uint64_t	size;
};

struct fastrpc_req_mmap {
	int32_t		fd;
	uint32_t	flags;
	uint64_t	vaddrin;
	uint64_t	size;
	uint64_t	vaddrout;
};

struct fastrpc_mem_map {
	int32_t		version;
	int32_t		fd;
	int32_t		offset;
	uint32_t	flags;
	uint64_t	vaddrin;
	uint64_t	length;
	uint64_t	vaddrout;
	int32_t		attrs;
	int32_t		reserved[4];
};

struct fastrpc_req_munmap {
	uint64_t	vaddrout;
	uint64_t	size;
};

struct fastrpc_mem_unmap {
	int32_t		vesion;
	int32_t		fd;
	uint64_t	vaddr;
	uint64_t	length;
	int32_t		reserved[5];
};

struct fastrpc_ioctl_set_option {
	uint32_t	request_id;
	uint32_t	value;
	int32_t		reserved[6];
};

struct fastrpc_ioctl_capability {
	uint32_t	unused;
	uint32_t	attribute_id;
	uint32_t	capability;
	uint32_t	reserved[4];
};

/* init_create attrs */
#define	FASTRPC_MODE_DEBUG		(1 << 0)
#define	FASTRPC_MODE_PTRACE		(1 << 1)
#define	FASTRPC_MODE_CRC		(1 << 2)
#define	FASTRPC_MODE_UNSIGNED_MODULE	(1 << 3)
#define	FASTRPC_MODE_ADAPTIVE_QOS	(1 << 4)
#define	FASTRPC_MODE_SYSTEM_PROCESS	(1 << 5)
#define	FASTRPC_MODE_PRIVILEGED		(1 << 6)

#define	FASTRPC_POLL_MODE		1

#define	FASTRPC_IOCTL_ALLOC_DMA_BUFF	_IOWR('R', 1, struct fastrpc_alloc_dma_buf)
#define	FASTRPC_IOCTL_FREE_DMA_BUFF	_IOWR('R', 2, uint32_t)
#define	FASTRPC_IOCTL_INVOKE		_IOWR('R', 3, struct fastrpc_invoke)
#define	FASTRPC_IOCTL_INIT_ATTACH	_IO('R', 4)
#define	FASTRPC_IOCTL_INIT_CREATE	_IOWR('R', 5, struct fastrpc_init_create)
#define	FASTRPC_IOCTL_MMAP		_IOWR('R', 6, struct fastrpc_req_mmap)
#define	FASTRPC_IOCTL_MUNMAP		_IOWR('R', 7, struct fastrpc_req_munmap)
#define	FASTRPC_IOCTL_INIT_ATTACH_SNS	_IO('R', 8)
#define	FASTRPC_IOCTL_INIT_CREATE_STATIC _IOWR('R', 9, struct fastrpc_init_create_static)
#define	FASTRPC_IOCTL_MEM_MAP		_IOWR('R', 10, struct fastrpc_mem_map)
#define	FASTRPC_IOCTL_MEM_UNMAP		_IOWR('R', 11, struct fastrpc_mem_unmap)
#define	FASTRPC_IOCTL_SET_OPTION	_IOWR('R', 12, struct fastrpc_ioctl_set_option)
#define	FASTRPC_IOCTL_GET_DSP_INFO	_IOWR('R', 13, struct fastrpc_ioctl_capability)

#endif /* _DEV_QCOM_FASTRPC_FASTRPC_H_ */
