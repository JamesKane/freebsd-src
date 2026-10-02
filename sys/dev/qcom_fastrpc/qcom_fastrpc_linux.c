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
 * FastRPC for Linux programs: the Linux compatibility layer hands their
 * ioctls on /dev/fastrpc-* here, and they go to qcom_fastrpc unchanged
 * but for the encoding of the command, so that Linux builds of the
 * FastRPC library, and what is built on it (Qualcomm's QNN), run as is.
 * The structures are Linux's already, and the same on both.
 *
 * The buffers qcom_fastrpc hands out are shared memory objects where
 * Linux's are dma-bufs; of the dma-buf ioctls, QNN names its buffers
 * (DMA_BUF_SET_NAME, for debugging on Linux), and gives up on a buffer it
 * cannot name, so that is taken, on any shared memory object.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/capsicum.h>
#include <sys/conf.h>
#include <sys/file.h>
#include <sys/ioccom.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/proc.h>
#include <sys/sysproto.h>
#include <sys/vnode.h>

#include <machine/../linux/linux.h>
#include <machine/../linux/linux_proto.h>
#include <compat/linux/linux_ioctl.h>

/* Linux's command encoding: _IOC(dir, type, nr, size). */
#define	LINUX_IOC_SIZE(cmd)	(((cmd) >> 16) & 0x3fff)
#define	LINUX_IOC_DIR(cmd)	((cmd) >> 30)
#define	LINUX_IOC_WRITE		1	/* to the kernel: IOC_IN */
#define	LINUX_IOC_READ		2	/* from it: IOC_OUT */

/* All of type 'R'; others' descriptors pass on to other handlers. */
#define	FASTRPC_LINUX_MIN	0x5200
#define	FASTRPC_LINUX_MAX	0x52ff

LINUX_IOCTL_SET(fastrpc, FASTRPC_LINUX_MIN, FASTRPC_LINUX_MAX);

/* dma-buf's, type 'b': DMA_BUF_SET_NAME_A (a u32 pointer) and _B (u64). */
#define	DMABUF_LINUX_MIN	0x6200
#define	DMABUF_LINUX_MAX	0x62ff
#define	DMABUF_SET_NAME_A	0x40046201
#define	DMABUF_SET_NAME_B	0x40086201
#define	DMABUF_NAME_LEN		32

LINUX_IOCTL_SET(dmabuf, DMABUF_LINUX_MIN, DMABUF_LINUX_MAX);

static bool
fastrpc_linux_ours(struct thread *td, int fd)
{
	cap_rights_t rights;
	struct file *fp;
	struct cdev *dev;
	bool ours;

	if (fget(td, fd, cap_rights_init_one(&rights, CAP_IOCTL), &fp) != 0)
		return (false);
	ours = false;
	if (fp->f_type == DTYPE_VNODE && fp->f_vnode->v_type == VCHR &&
	    (dev = fp->f_vnode->v_rdev) != NULL)
		ours = strcmp(dev->si_devsw->d_name, "fastrpc") == 0;
	fdrop(fp, td);
	return (ours);
}

static int
fastrpc_linux_ioctl(struct thread *td, struct linux_ioctl_args *args)
{
	struct ioctl_args ia;
	u_long cmd;
	u_int dir, size;

	if (!fastrpc_linux_ours(td, args->fd))
		return (ENOIOCTL);
	dir = LINUX_IOC_DIR(args->cmd);
	size = LINUX_IOC_SIZE(args->cmd);
	if (size > IOCPARM_MAX)
		return (EINVAL);
	cmd = args->cmd & 0xffff;
	if (dir == 0)
		cmd |= IOC_VOID;
	else {
		cmd |= (u_long)size << 16;
		if (dir & LINUX_IOC_WRITE)
			cmd |= IOC_IN;
		if (dir & LINUX_IOC_READ)
			cmd |= IOC_OUT;
	}
	/* The usual path: copies in and out, and the DSP's result back. */
	ia.fd = args->fd;
	ia.com = cmd;
	ia.data = (caddr_t)(uintptr_t)args->arg;
	return (sys_ioctl(td, &ia));
}

static int
dmabuf_linux_ioctl(struct thread *td, struct linux_ioctl_args *args)
{
	cap_rights_t rights;
	struct file *fp;
	char name[DMABUF_NAME_LEN];
	size_t len;
	int error;

	if (args->cmd != DMABUF_SET_NAME_A && args->cmd != DMABUF_SET_NAME_B)
		return (ENOIOCTL);
	error = fget(td, args->fd, cap_rights_init_one(&rights, CAP_IOCTL),
	    &fp);
	if (error != 0)
		return (error);
	if (fp->f_type != DTYPE_SHM) {
		fdrop(fp, td);
		return (ENOIOCTL);
	}
	fdrop(fp, td);
	/* As Linux checks it; the name itself has no use here. */
	error = copyinstr((void *)(uintptr_t)args->arg, name, sizeof(name),
	    &len);
	return (error == ENAMETOOLONG ? EINVAL : error);
}

static int
fastrpc_linux_modevent(module_t mod __unused, int cmd __unused,
    void *arg __unused)
{

	return (0);
}

DEV_MODULE(qcom_fastrpc_linux, fastrpc_linux_modevent, NULL);
MODULE_DEPEND(qcom_fastrpc_linux, linux64elf, 1, 1, 1);
MODULE_VERSION(qcom_fastrpc_linux, 1);
