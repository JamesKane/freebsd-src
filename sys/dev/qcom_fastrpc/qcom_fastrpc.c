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
 * FastRPC: calls from user space into the Qualcomm compute DSP (CDSP), as
 * Linux's drivers/misc/fastrpc.c makes them, behind the same ioctls.
 *
 * Each open of /dev/fastrpc-cdsp takes a session: one of the DSP's compute
 * context banks, a stream of its own through the apps SMMU, which the
 * DSP's accesses for that process go through.  A call goes to the DSP over
 * the GLINK channel "fastrpcglink-apps-dsp" as a message naming a buffer:
 * the call's arguments described (a remote argument, a page list and a
 * page for each), followed by copies of the input and room for the output
 * buffers.  The buffer is mapped into the session's bank, and addresses
 * given to the DSP carry the session's number above bit 32.  The DSP
 * answers with the call's context and result.
 *
 * Buffers shared with the DSP (Linux's dma-bufs, from ALLOC_DMA_BUFF) are
 * anonymous POSIX shared memory objects here: the program maps them as any
 * other, and a call argument that names one by its descriptor is mapped,
 * for the call, into the session's bank, its pages wired and their cache
 * lines written back before and dropped after.
 *
 * Not yet: maps of memory into the DSP process, static processes and poll
 * mode.  A DSP result that
 * isn't 0 comes back as EIO (Linux returns it as the ioctl's value).
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/capsicum.h>
#include <sys/conf.h>
#include <sys/fcntl.h>
#include <sys/file.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mman.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/rwlock.h>
#include <sys/sx.h>
#include <sys/syscallsubr.h>
#include <sys/taskqueue.h>

#include <vm/vm.h>
#include <vm/pmap.h>
#include <vm/vm_extern.h>
#include <vm/vm_kern.h>
#include <vm/vm_map.h>
#include <vm/vm_object.h>
#include <vm/vm_page.h>
#include <vm/vm_pager.h>

#include <machine/cpufunc.h>

#include <machine/bus.h>

#include <dev/qcom_fastrpc/fastrpc.h>
#include <dev/qcom_glink/qcom_glink.h>
#include <dev/qcom_smmu/qcom_apps_smmu.h>

static MALLOC_DEFINE(M_FASTRPC, "qcom_fastrpc", "Qualcomm FastRPC");

#define	FASTRPC_ALIGN			128
#define	FASTRPC_MAX_FDLIST		16
#define	FASTRPC_MAX_CRCLIST		64
#define	FASTRPC_CTX_MAX			256
#define	FASTRPC_MAX_DSP_ATTRIBUTES	256
#define	FASTRPC_INIT_HANDLE		1
#define	FASTRPC_DSP_UTILITIES_HANDLE	2
#define	FASTRPC_INIT_FILELEN_MAX	(2 * 1024 * 1024)
#define	FASTRPC_DSP_UNSUPPORTED_API	0x80000414
#define	FASTRPC_DSP_PD_NOTIFY_CTX	0xabcdabcdu

#define	FASTRPC_RMID_INIT_ATTACH	0
#define	FASTRPC_RMID_INIT_RELEASE	1
#define	FASTRPC_RMID_INIT_MMAP		4
#define	FASTRPC_RMID_INIT_MUNMAP	5
#define	FASTRPC_RMID_INIT_CREATE	6
#define	FASTRPC_RMID_INIT_CREATE_ATTR	7

#define	ROOT_PD				0
#define	USER_PD				1
#define	SENSORS_PD			2

#define	SC_INBUFS(sc)		(((sc) >> 16) & 0xff)
#define	SC_OUTBUFS(sc)		(((sc) >> 8) & 0xff)
#define	SC_INHANDLES(sc)	(((sc) >> 4) & 0x0f)
#define	SC_OUTHANDLES(sc)	((sc) & 0x0f)
#define	SC_LENGTH(sc)		(SC_INBUFS(sc) + SC_OUTBUFS(sc) +	\
				SC_INHANDLES(sc) + SC_OUTHANDLES(sc))
#define	SCALARS(method, in, out)					\
	(((method) & 0x1f) << 24 | ((in) & 0xff) << 16 | ((out) & 0xff) << 8)

/* Context IDs: a slot in bits 15:8 and a sequence above, the PD below. */
#define	CTXID_SLOT(id)		(((id) >> 8) & 0xff)
#define	CTXID_SEQ(id)		((id) >> 16)
#define	CTXID(slot, seq)	((uint64_t)(slot) << 8 | (uint64_t)(seq) << 16)

/* On the wire. */
struct fastrpc_msg {
	int32_t		client_id;
	int32_t		tid;
	uint64_t	ctx;
	uint32_t	handle;
	uint32_t	sc;
	uint64_t	addr;
	uint64_t	size;
};

struct fastrpc_invoke_rsp {
	uint64_t	ctx;
	int32_t		retval;
	uint32_t	pad;
};

struct fastrpc_remote_arg {		/* a buffer, or a handle */
	uint64_t	pv;
	uint64_t	len;
};

struct fastrpc_invoke_buf {
	uint32_t	num;
	uint32_t	pgidx;
};

struct fastrpc_phy_page {
	uint64_t	addr;
	uint64_t	size;
};

/* SC8280XP NSP0's compute banks: their numbers and streams (mask 0x420). */
static const struct fastrpc_bank {
	uint8_t		reg;
	uint16_t	sid;
} fastrpc_banks[] = {
	{ 1, 0x3181 }, { 2, 0x3182 }, { 3, 0x3183 }, { 4, 0x3184 },
	{ 5, 0x3185 }, { 6, 0x3186 }, { 7, 0x3187 }, { 8, 0x3188 },
	{ 11, 0x318c }, { 12, 0x318d }, { 13, 0x318e }, { 14, 0x318f },
};
#define	FASTRPC_SID_MASK	0x420
#define	FASTRPC_SESSIONS	nitems(fastrpc_banks)

struct fastrpc_session {
	const struct fastrpc_bank *bank;
	struct qcom_apps_smmu_dom *dom;	/* attached on first use, kept */
	bool		used;
};

/* Memory the DSP reaches through a session's bank. */
struct fastrpc_buf {
	void		*va;
	size_t		size;
	uint64_t	iova;
	uint64_t	daddr;		/* as the DSP knows it */
	uint64_t	raddr;		/* in the DSP process, if given it */
	TAILQ_ENTRY(fastrpc_buf) link;
};

/* MMAP's kind: pages for the DSP process's own heap. */
#define	FASTRPC_MMAP_ADD_PAGES	0x1000
#define	FASTRPC_MMAP_MAX	(256 * 1024 * 1024)

struct fastrpc_user {
	struct fastrpc_session *sess;
	int		client_id;
	int		pd;
	bool		attached;	/* to a process on the DSP */
	struct fastrpc_buf *init_mem;
	TAILQ_HEAD(, fastrpc_buf) mmaps; /* given the DSP process, by MMAP */
	struct sx	lock;
};

struct fastrpc_ctx {
	uint64_t	ctxid;
	int		retval;
	bool		done;
	/*
	 * A call given up on, by a signal or (the driver's own) a timeout:
	 * the DSP may still use its buffers, which go when it answers.
	 */
	bool		abandoned;
	struct fastrpc_session *sess;
	struct fastrpc_buf *buf;
	struct fastrpc_shbuf *sbs;
	u_int		nsbs;
	STAILQ_ENTRY(fastrpc_ctx) link;
};

static struct {
	struct mtx		mtx;	/* below, and the contexts */
	struct qcom_glink_chan	*ch;
	struct cdev		*cdev;
	struct fastrpc_session	sess[FASTRPC_SESSIONS];
	struct fastrpc_ctx	*ctx[FASTRPC_CTX_MAX];
	u_int			next_slot;
	uint64_t		seq;
	uint32_t		*attrs;	/* the DSP's, once asked */
	struct taskqueue	*tq;
	struct timeout_task	start_task;
	struct task		reap_task;	/* abandoned calls answered */
	STAILQ_HEAD(, fastrpc_ctx) reap;
	int			start_tries;
} frpc;

#define	FASTRPC_START_TRIES	300	/* a second apart, for the DSP */

/* Buffers */

static void
fastrpc_buf_free(struct fastrpc_session *s, struct fastrpc_buf *b)
{

	if (b == NULL)
		return;
	qcom_apps_smmu_unmap(s->dom, b->iova, b->size);
	kmem_free(b->va, b->size);
	free(b, M_FASTRPC);
}

static int
fastrpc_buf_alloc(struct fastrpc_session *s, size_t size,
    struct fastrpc_buf **bp)
{
	struct fastrpc_buf *b;
	int error;

	b = malloc(sizeof(*b), M_FASTRPC, M_WAITOK | M_ZERO);
	b->size = round_page(size);
	/* Uncached, as the bank maps it: no cache maintenance needed. */
	b->va = kmem_alloc_contig(b->size, M_WAITOK | M_ZERO, 0,
	    BUS_SPACE_MAXADDR, PAGE_SIZE, 0, VM_MEMATTR_WRITE_COMBINING);
	if (b->va == NULL) {
		free(b, M_FASTRPC);
		return (ENOMEM);
	}
	error = qcom_apps_smmu_map(s->dom, vtophys(b->va), b->size, &b->iova);
	if (error != 0) {
		kmem_free(b->va, b->size);
		free(b, M_FASTRPC);
		return (error);
	}
	b->daddr = b->iova | (uint64_t)s->bank->reg << 32;
	*bp = b;
	return (0);
}

/*
 * A shared buffer for a call: the shm object the descriptor names, the
 * pages under [ptr, ptr + len) of the program's map of it, wired and
 * mapped into the session's bank.
 */
struct fastrpc_shbuf {
	struct file	*fp;
	vm_page_t	*ma;
	u_int		npages;
	uint64_t	iova;
	uint64_t	daddr;		/* of the first page, as the DSP knows it */
};

static void
fastrpc_shbuf_put(struct fastrpc_session *s, struct fastrpc_shbuf *sb)
{
	u_int i;

	if (sb->npages != 0 && sb->iova != 0)
		qcom_apps_smmu_unmap(s->dom, sb->iova, ptoa(sb->npages));
	for (i = 0; i < sb->npages && sb->ma != NULL; i++)
		if (sb->ma[i] != NULL)
			vm_page_unwire(sb->ma[i], PQ_ACTIVE);
	free(sb->ma, M_FASTRPC);
	if (sb->fp != NULL)
		fdrop(sb->fp, curthread);
	memset(sb, 0, sizeof(*sb));
}

/* Write back, or (after the DSP wrote) drop, the pages' cache lines. */
static void
fastrpc_shbuf_sync(struct fastrpc_shbuf *sb, bool after)
{
	void *va;
	u_int i;

	for (i = 0; i < sb->npages; i++) {
		va = (void *)PHYS_TO_DMAP(VM_PAGE_TO_PHYS(sb->ma[i]));
		if (after)
			cpu_dcache_inv_range(va, PAGE_SIZE);
		else
			cpu_dcache_wbinv_range(va, PAGE_SIZE);
	}
}

static int
fastrpc_shbuf_get(struct fastrpc_session *s, int fd, uint64_t ptr,
    uint64_t len, struct fastrpc_shbuf *sb)
{
	struct thread *td = curthread;
	struct shmfd *shm;
	vm_map_entry_t entry;
	vm_map_t map;
	vm_object_t obj;
	vm_ooffset_t off;
	cap_rights_t rights;
	u_int i;
	int error;

	memset(sb, 0, sizeof(*sb));
	error = fget(td, fd, cap_rights_init_one(&rights, CAP_MMAP), &sb->fp);
	if (error != 0)
		return (error);
	if (sb->fp->f_type != DTYPE_SHM) {
		fastrpc_shbuf_put(s, sb);
		return (EINVAL);
	}
	shm = sb->fp->f_data;
	obj = shm->shm_object;

	/* Where in the object the program's pointer is. */
	map = &td->td_proc->p_vmspace->vm_map;
	vm_map_lock_read(map);
	if (!vm_map_lookup_entry(map, ptr, &entry) ||
	    entry->object.vm_object != obj ||
	    ptr + len > entry->end) {
		vm_map_unlock_read(map);
		fastrpc_shbuf_put(s, sb);
		return (EINVAL);
	}
	off = entry->offset + (ptr - entry->start);
	vm_map_unlock_read(map);
	if (off + len > (vm_ooffset_t)shm->shm_size) {
		fastrpc_shbuf_put(s, sb);
		return (EINVAL);
	}

	sb->npages = atop(round_page(off + len) - trunc_page(off));
	sb->ma = malloc(sb->npages * sizeof(*sb->ma), M_FASTRPC,
	    M_WAITOK | M_ZERO);
	for (i = 0; i < sb->npages; i++) {
		error = vm_page_grab_valid_unlocked(&sb->ma[i], obj,
		    OFF_TO_IDX(off) + i,
		    VM_ALLOC_WIRED | VM_ALLOC_NOBUSY | VM_ALLOC_NORMAL);
		if (error != VM_PAGER_OK) {
			sb->ma[i] = NULL;
			fastrpc_shbuf_put(s, sb);
			return (EFAULT);
		}
	}
	error = qcom_apps_smmu_map_pages(s->dom, sb->ma, sb->npages,
	    &sb->iova);
	if (error != 0) {
		sb->iova = 0;
		fastrpc_shbuf_put(s, sb);
		return (error);
	}
	sb->daddr = sb->iova | (uint64_t)s->bank->reg << 32;
	return (0);
}

/* Calls */

/* The DSP's answer; from the GLINK rx task. */
static void
fastrpc_rx(void *arg __unused, const void *data, size_t len)
{
	const struct fastrpc_invoke_rsp *rsp = data;
	struct fastrpc_ctx *c;
	u_int slot;

	if (len < offsetof(struct fastrpc_invoke_rsp, pad) ||
	    rsp->ctx == FASTRPC_DSP_PD_NOTIFY_CTX)
		return;
	slot = CTXID_SLOT(rsp->ctx);
	mtx_lock(&frpc.mtx);
	c = frpc.ctx[slot];
	if (c != NULL && CTXID_SEQ(c->ctxid) == CTXID_SEQ(rsp->ctx)) {
		c->retval = rsp->retval;
		c->done = true;
		if (c->abandoned) {
			frpc.ctx[slot] = NULL;
			STAILQ_INSERT_TAIL(&frpc.reap, c, link);
			taskqueue_enqueue(frpc.tq, &frpc.reap_task);
		} else
			wakeup(c);
	} else
		printf("qcom_fastrpc: answer for no call (%#jx)\n",
		    (uintmax_t)rsp->ctx);
	mtx_unlock(&frpc.mtx);
}

static int
fastrpc_ctx_alloc(struct fastrpc_ctx *c)
{
	u_int i, slot;

	mtx_lock(&frpc.mtx);
	for (i = 0; i < FASTRPC_CTX_MAX - 1; i++) {
		slot = 1 + (frpc.next_slot + i) % (FASTRPC_CTX_MAX - 1);
		if (frpc.ctx[slot] == NULL) {
			frpc.next_slot = slot;
			frpc.ctx[slot] = c;
			c->ctxid = CTXID(slot, ++frpc.seq);
			mtx_unlock(&frpc.mtx);
			return (0);
		}
	}
	mtx_unlock(&frpc.mtx);
	return (EAGAIN);
}

/*
 * Call handle on the DSP with args, described by sc.  kernel: the
 * arguments' pointers are the kernel's, and the call is the driver's own
 * (client 0).  Sleeps.
 */
static int
fastrpc_invoke_args(struct fastrpc_user *fl, bool kernel, uint32_t handle,
    uint32_t sc, struct fastrpc_invoke_args *args)
{
	struct fastrpc_session *s = fl->sess;
	struct fastrpc_remote_arg *rpra;
	struct fastrpc_invoke_buf *list;
	struct fastrpc_phy_page *pages;
	struct fastrpc_ctx *c;
	struct fastrpc_buf *buf;
	struct fastrpc_shbuf *sbs;
	struct fastrpc_msg msg;
	uint64_t off, pg0, pg1;
	size_t metalen, size;
	u_int i, inbufs, nbufs, nscalars;
	char *base;
	int error;

	inbufs = SC_INBUFS(sc);
	nbufs = inbufs + SC_OUTBUFS(sc);
	nscalars = SC_LENGTH(sc);
	for (i = nbufs; i < nscalars; i++)
		if (args[i].fd > 0)
			return (EOPNOTSUPP);	/* handles to buffers: not yet */
	/*
	 * Buffers by descriptor, in the calling process's table and map,
	 * kernel calls' too (the image INIT_CREATE passes).
	 */
	sbs = malloc(MAX(nbufs, 1) * sizeof(*sbs), M_FASTRPC,
	    M_WAITOK | M_ZERO);
	for (i = 0; i < nbufs; i++) {
		if (args[i].fd <= 0 || args[i].length == 0)
			continue;
		error = fastrpc_shbuf_get(s, args[i].fd, args[i].ptr,
		    args[i].length, &sbs[i]);
		if (error != 0)
			goto out_sbs;
		fastrpc_shbuf_sync(&sbs[i], false);
	}

	/* The description, then each buffer at its own aligned offset. */
	metalen = (sizeof(*rpra) + sizeof(*list) + sizeof(*pages)) *
	    nscalars + sizeof(uint64_t) * FASTRPC_MAX_FDLIST +
	    sizeof(uint32_t) * FASTRPC_MAX_CRCLIST + sizeof(uint32_t);
	size = roundup2(metalen, FASTRPC_ALIGN);
	for (i = 0; i < nbufs; i++) {
		if (sbs[i].npages != 0)
			continue;
		if (args[i].length > FASTRPC_INIT_FILELEN_MAX * 4) {
			error = EINVAL;
			goto out_sbs;
		}
		size = roundup2(size, FASTRPC_ALIGN) + args[i].length;
	}
	error = fastrpc_buf_alloc(s, size, &buf);
	if (error != 0)
		goto out_sbs;
	base = buf->va;
	rpra = (struct fastrpc_remote_arg *)base;
	list = (struct fastrpc_invoke_buf *)(rpra + nscalars);
	pages = (struct fastrpc_phy_page *)(list + nscalars);

	off = roundup2(metalen, FASTRPC_ALIGN);
	for (i = 0; i < nbufs; i++) {
		rpra[i].len = args[i].length;
		list[i].num = args[i].length != 0;
		list[i].pgidx = i;
		if (args[i].length == 0)
			continue;
		if (sbs[i].npages != 0) {
			/* In place: the program's address, the pages' own. */
			rpra[i].pv = args[i].ptr;
			pages[i].addr = sbs[i].daddr;
			pages[i].size = ptoa(sbs[i].npages);
			continue;
		}
		off = roundup2(off, FASTRPC_ALIGN);
		/* The DSP takes the offset in the page from pv. */
		rpra[i].pv = (uint64_t)(uintptr_t)(base + off);
		pg0 = trunc_page(off);
		pg1 = trunc_page(off + args[i].length - 1);
		pages[i].addr = buf->daddr + pg0;
		pages[i].size = pg1 - pg0 + PAGE_SIZE;
		if (i < inbufs) {
			if (kernel)
				memcpy(base + off,
				    (void *)(uintptr_t)args[i].ptr,
				    args[i].length);
			else if ((error = copyin((void *)(uintptr_t)
			    args[i].ptr, base + off, args[i].length)) != 0)
				goto out;
		}
		off += args[i].length;
	}
	for (i = nbufs; i < nscalars; i++) {
		/* Handles: an fd, its offset and length (none here). */
		rpra[i].pv = (uint64_t)(uint32_t)args[i].fd |
		    (uint64_t)(uint32_t)args[i].ptr << 32;
		rpra[i].len = args[i].length;
		list[i].num = args[i].length != 0;
		list[i].pgidx = i;
	}
	wmb();

	c = malloc(sizeof(*c), M_FASTRPC, M_WAITOK | M_ZERO);
	error = fastrpc_ctx_alloc(c);
	if (error != 0) {
		free(c, M_FASTRPC);
		goto out;
	}
	memset(&msg, 0, sizeof(msg));
	msg.client_id = kernel ? 0 : fl->client_id;
	msg.tid = curthread->td_tid;
	msg.ctx = c->ctxid | fl->pd;
	msg.handle = handle;
	msg.sc = sc;
	msg.addr = buf->daddr;
	msg.size = round_page(size);
	error = qcom_glink_send(frpc.ch, &msg, sizeof(msg));

	/*
	 * A program's call waits for as long as it takes (a listener's waits
	 * for the DSP's next request), unless a signal comes; the driver's
	 * own give up after ten seconds.
	 */
	mtx_lock(&frpc.mtx);
	if (error == 0) {
		while (!c->done && error == 0)
			error = msleep(c, &frpc.mtx, kernel ? 0 : PCATCH,
			    "frpc", kernel ? 10 * hz : 0);
		if (c->done)
			error = 0;
		else {
			/* The DSP may still use the buffers: they're its. */
			c->abandoned = true;
			c->sess = s;
			c->buf = buf;
			c->sbs = sbs;
			c->nsbs = nbufs;
			mtx_unlock(&frpc.mtx);
			if (kernel)
				printf("qcom_fastrpc: no answer from the DSP "
				    "to %#x/%#x\n", handle, sc);
			return (error == EWOULDBLOCK ? ETIMEDOUT : EINTR);
		}
	}
	frpc.ctx[CTXID_SLOT(c->ctxid)] = NULL;
	mtx_unlock(&frpc.mtx);
	if (error == 0 && c->retval != 0) {
		if (bootverbose || kernel)
			printf("qcom_fastrpc: DSP result %#x for %#x/%#x\n",
			    (uint32_t)c->retval, handle, sc);
		error = (uint32_t)c->retval == FASTRPC_DSP_UNSUPPORTED_API ?
		    EOPNOTSUPP : EIO;
	}
	free(c, M_FASTRPC);
	if (error != 0)
		goto out;
	rmb();

	/* The output buffers back. */
	off = roundup2(metalen, FASTRPC_ALIGN);
	for (i = 0; i < nbufs; i++) {
		if (sbs[i].npages != 0) {
			if (i >= inbufs)
				fastrpc_shbuf_sync(&sbs[i], true);
			continue;
		}
		off = roundup2(off, FASTRPC_ALIGN);
		if (i >= inbufs && args[i].length != 0) {
			if (kernel)
				memcpy((void *)(uintptr_t)args[i].ptr,
				    base + off, args[i].length);
			else if ((error = copyout(base + off,
			    (void *)(uintptr_t)args[i].ptr,
			    args[i].length)) != 0)
				break;
		}
		off += args[i].length;
	}
out:
	fastrpc_buf_free(s, buf);
out_sbs:
	for (i = 0; i < nbufs; i++)
		if (sbs[i].fp != NULL)
			fastrpc_shbuf_put(s, &sbs[i]);
	free(sbs, M_FASTRPC);
	return (error);
}

/* Free what abandoned calls held, once the DSP has answered them. */
static void
fastrpc_reap(void *arg __unused, int pending __unused)
{
	struct fastrpc_ctx *c;
	u_int i;

	mtx_lock(&frpc.mtx);
	while ((c = STAILQ_FIRST(&frpc.reap)) != NULL) {
		STAILQ_REMOVE_HEAD(&frpc.reap, link);
		mtx_unlock(&frpc.mtx);
		fastrpc_buf_free(c->sess, c->buf);
		for (i = 0; i < c->nsbs; i++)
			if (c->sbs[i].fp != NULL)
				fastrpc_shbuf_put(c->sess, &c->sbs[i]);
		free(c->sbs, M_FASTRPC);
		free(c, M_FASTRPC);
		mtx_lock(&frpc.mtx);
	}
	mtx_unlock(&frpc.mtx);
}

/* Requests */

static int
fastrpc_init_attach(struct fastrpc_user *fl, int pd)
{
	struct fastrpc_invoke_args args[1];
	int32_t client_id = fl->client_id;
	int error;

	memset(args, 0, sizeof(args));
	args[0].ptr = (uintptr_t)&client_id;
	args[0].length = sizeof(client_id);
	args[0].fd = -1;
	fl->pd = pd;
	error = fastrpc_invoke_args(fl, true, FASTRPC_INIT_HANDLE,
	    SCALARS(FASTRPC_RMID_INIT_ATTACH, 1, 0), args);
	if (error == 0)
		fl->attached = true;
	return (error);
}

static int
fastrpc_init_create(struct fastrpc_user *fl, struct fastrpc_init_create *init)
{
	struct fastrpc_invoke_args args[6];
	struct fastrpc_phy_page pages[1];
	struct {
		int32_t		client_id;
		uint32_t	namelen;
		uint32_t	filelen;
		uint32_t	pageslen;
		uint32_t	attrs;
		uint32_t	siglen;
	} inbuf;
	struct fastrpc_buf *imem;
	char name[MAXCOMLEN + 1];
	void *file;
	size_t memlen;
	int error;

	if (init->filelen > FASTRPC_INIT_FILELEN_MAX)
		return (EINVAL);
	if (fl->attached)
		return (EBUSY);
	file = NULL;
	/* The image: in a shared buffer the caller names, or copied. */
	if (init->filelen != 0 && init->filefd <= 0) {
		file = malloc(init->filelen, M_FASTRPC, M_WAITOK);
		error = copyin((void *)(uintptr_t)init->file, file,
		    init->filelen);
		if (error != 0) {
			free(file, M_FASTRPC);
			return (error);
		}
	}
	/* The process's memory on the DSP, which it keeps. */
	memlen = roundup2(MAX(FASTRPC_INIT_FILELEN_MAX, init->filelen * 4),
	    1024 * 1024);
	error = fastrpc_buf_alloc(fl->sess, memlen, &imem);
	if (error != 0) {
		free(file, M_FASTRPC);
		return (error);
	}
	strlcpy(name, curproc->p_comm, sizeof(name));
	inbuf.client_id = fl->client_id;
	inbuf.namelen = strlen(name) + 1;
	inbuf.filelen = init->filelen;
	inbuf.pageslen = 1;
	inbuf.attrs = init->attrs;
	inbuf.siglen = init->siglen;
	pages[0].addr = imem->daddr;
	pages[0].size = imem->size;
	memset(args, 0, sizeof(args));
	args[0].ptr = (uintptr_t)&inbuf;
	args[0].length = sizeof(inbuf);
	args[1].ptr = (uintptr_t)name;
	args[1].length = inbuf.namelen;
	args[2].ptr = file != NULL ? (uintptr_t)file : init->file;
	args[2].length = init->filelen;
	args[3].ptr = (uintptr_t)pages;
	args[3].length = sizeof(pages);
	args[4].ptr = (uintptr_t)&inbuf.attrs;
	args[4].length = sizeof(inbuf.attrs);
	args[5].ptr = (uintptr_t)&inbuf.siglen;
	args[5].length = sizeof(inbuf.siglen);
	for (int i = 0; i < 6; i++)
		args[i].fd = -1;
	if (file == NULL && init->filelen != 0)
		args[2].fd = init->filefd;
	fl->pd = USER_PD;
	error = fastrpc_invoke_args(fl, true, FASTRPC_INIT_HANDLE,
	    SCALARS(init->attrs != 0 ? FASTRPC_RMID_INIT_CREATE_ATTR :
	    FASTRPC_RMID_INIT_CREATE, 4, 0), args);
	free(file, M_FASTRPC);
	if (error != 0) {
		fastrpc_buf_free(fl->sess, imem);
		fl->pd = ROOT_PD;
		return (error);
	}
	fl->init_mem = imem;
	fl->attached = true;
	return (0);
}

static int
fastrpc_get_dsp_info(struct fastrpc_user *fl,
    struct fastrpc_ioctl_capability *cap)
{
	struct fastrpc_invoke_args args[2];
	uint32_t *attrs, len;
	int error;

	if (cap->attribute_id >= FASTRPC_MAX_DSP_ATTRIBUTES)
		return (EOVERFLOW);
	mtx_lock(&frpc.mtx);
	attrs = frpc.attrs;
	mtx_unlock(&frpc.mtx);
	if (attrs == NULL) {
		/* Attribute 0 is user space's; the DSP fills the rest. */
		attrs = malloc(FASTRPC_MAX_DSP_ATTRIBUTES * sizeof(*attrs),
		    M_FASTRPC, M_WAITOK | M_ZERO);
		len = FASTRPC_MAX_DSP_ATTRIBUTES - 1;
		memset(args, 0, sizeof(args));
		args[0].ptr = (uintptr_t)&len;
		args[0].length = sizeof(len);
		args[0].fd = -1;
		args[1].ptr = (uintptr_t)&attrs[1];
		args[1].length = len * sizeof(*attrs);
		args[1].fd = -1;
		error = fastrpc_invoke_args(fl, true,
		    FASTRPC_DSP_UTILITIES_HANDLE, SCALARS(0, 1, 1), args);
		if (error != 0) {
			free(attrs, M_FASTRPC);
			return (error);
		}
		mtx_lock(&frpc.mtx);
		if (frpc.attrs == NULL)
			frpc.attrs = attrs;
		else {
			free(attrs, M_FASTRPC);
			attrs = frpc.attrs;
		}
		mtx_unlock(&frpc.mtx);
	}
	cap->capability = attrs[cap->attribute_id];
	return (0);
}

/*
 * A buffer to share with the DSP: an anonymous shared memory object of the
 * size, its descriptor in the caller's table.
 */
static int
fastrpc_alloc_shm(struct thread *td, struct fastrpc_alloc_dma_buf *ab)
{
	int error, fd;

	if (ab->size == 0 || ab->size > (uint64_t)1 << 32)
		return (EINVAL);
	error = kern_shm_open2(td, SHM_ANON, O_RDWR | O_CLOEXEC, 0600, 0,
	    NULL, "fastrpc", NULL);
	if (error != 0)
		return (error);
	/* The descriptor comes back where the ioctl's own result goes. */
	fd = td->td_retval[0];
	td->td_retval[0] = 0;
	error = kern_ftruncate(td, fd, round_page(ab->size));
	if (error != 0) {
		(void)kern_close(td, fd);
		return (error);
	}
	ab->fd = fd;
	return (0);
}

/*
 * Pages the DSP process asks for (its loader's, its heap's), by way of
 * the library: allocated here and given it, never mapped for the program.
 */
static int
fastrpc_req_mmap(struct fastrpc_user *fl, struct fastrpc_req_mmap *req)
{
	struct {
		int32_t		pgid;
		uint32_t	flags;
		uint64_t	vaddr;
		int32_t		num;
	} msg;		/* laid out as Linux's, unpacked */
	struct fastrpc_phy_page page;
	struct fastrpc_invoke_args args[3];
	struct fastrpc_buf *b;
	uint64_t raddr;
	int error;

	if (req->flags != FASTRPC_MMAP_ADD_PAGES || req->vaddrin != 0 ||
	    req->size == 0 || req->size > FASTRPC_MMAP_MAX)
		return (EINVAL);
	error = fastrpc_buf_alloc(fl->sess, req->size, &b);
	if (error != 0)
		return (error);
	msg.pgid = fl->client_id;
	msg.flags = req->flags;
	msg.vaddr = 0;
	msg.num = sizeof(page);
	page.addr = b->daddr;
	page.size = b->size;
	memset(args, 0, sizeof(args));
	args[0].ptr = (uintptr_t)&msg;
	args[0].length = sizeof(msg);
	args[1].ptr = (uintptr_t)&page;
	args[1].length = sizeof(page);
	args[2].ptr = (uintptr_t)&raddr;
	args[2].length = sizeof(raddr);
	for (int i = 0; i < 3; i++)
		args[i].fd = -1;
	error = fastrpc_invoke_args(fl, true, FASTRPC_INIT_HANDLE,
	    SCALARS(FASTRPC_RMID_INIT_MMAP, 2, 1), args);
	if (error != 0) {
		/* A timeout leaves the pages the DSP's. */
		if (error != ETIMEDOUT)
			fastrpc_buf_free(fl->sess, b);
		return (error);
	}
	b->raddr = raddr;
	mtx_lock(&frpc.mtx);
	TAILQ_INSERT_TAIL(&fl->mmaps, b, link);
	mtx_unlock(&frpc.mtx);
	req->vaddrout = raddr;
	return (0);
}

static int
fastrpc_req_munmap(struct fastrpc_user *fl, struct fastrpc_req_munmap *req)
{
	struct {
		int32_t		pgid;
		uint64_t	vaddr;
		uint64_t	size;
	} msg;		/* laid out as Linux's, unpacked */
	struct fastrpc_invoke_args args[1];
	struct fastrpc_buf *b;
	int error;

	mtx_lock(&frpc.mtx);
	TAILQ_FOREACH(b, &fl->mmaps, link)
		if (b->raddr == req->vaddrout && b->size == req->size)
			break;
	if (b != NULL)
		TAILQ_REMOVE(&fl->mmaps, b, link);
	mtx_unlock(&frpc.mtx);
	if (b == NULL)
		return (EINVAL);
	msg.pgid = fl->client_id;
	msg.vaddr = b->raddr;
	msg.size = b->size;
	memset(args, 0, sizeof(args));
	args[0].ptr = (uintptr_t)&msg;
	args[0].length = sizeof(msg);
	args[0].fd = -1;
	error = fastrpc_invoke_args(fl, true, FASTRPC_INIT_HANDLE,
	    SCALARS(FASTRPC_RMID_INIT_MUNMAP, 1, 0), args);
	if (error == 0)
		fastrpc_buf_free(fl->sess, b);
	else if (error != ETIMEDOUT) {
		/* Still the DSP's: keep it, for close to free. */
		mtx_lock(&frpc.mtx);
		TAILQ_INSERT_TAIL(&fl->mmaps, b, link);
		mtx_unlock(&frpc.mtx);
	}
	return (error);
}

/* The device */

static void
fastrpc_dtor(void *data)
{
	struct fastrpc_user *fl = data;
	struct fastrpc_invoke_args args[1];
	struct fastrpc_buf *b;
	int32_t client_id = fl->client_id;

	if (fl->attached) {
		memset(args, 0, sizeof(args));
		args[0].ptr = (uintptr_t)&client_id;
		args[0].length = sizeof(client_id);
		args[0].fd = -1;
		(void)fastrpc_invoke_args(fl, true, FASTRPC_INIT_HANDLE,
		    SCALARS(FASTRPC_RMID_INIT_RELEASE, 1, 0), args);
	}
	fastrpc_buf_free(fl->sess, fl->init_mem);
	/* The DSP process is gone, and with it its use of these. */
	while ((b = TAILQ_FIRST(&fl->mmaps)) != NULL) {
		TAILQ_REMOVE(&fl->mmaps, b, link);
		fastrpc_buf_free(fl->sess, b);
	}
	mtx_lock(&frpc.mtx);
	fl->sess->used = false;
	mtx_unlock(&frpc.mtx);
	sx_destroy(&fl->lock);
	free(fl, M_FASTRPC);
}

static int
fastrpc_open(struct cdev *dev, int oflags, int devtype, struct thread *td)
{
	struct fastrpc_session *s;
	struct fastrpc_user *fl;
	u_int i;
	int error;

	mtx_lock(&frpc.mtx);
	if (frpc.ch == NULL) {
		mtx_unlock(&frpc.mtx);
		return (ENXIO);
	}
	s = NULL;
	for (i = 0; i < FASTRPC_SESSIONS; i++)
		if (!frpc.sess[i].used) {
			s = &frpc.sess[i];
			s->used = true;
			break;
		}
	mtx_unlock(&frpc.mtx);
	if (s == NULL)
		return (EBUSY);
	if (s->dom == NULL) {
		error = qcom_apps_smmu_attach(s->bank->sid, FASTRPC_SID_MASK,
		    &s->dom);
		if (error != 0) {
			printf("qcom_fastrpc: no stream %#x: %d\n",
			    s->bank->sid, error);
			mtx_lock(&frpc.mtx);
			s->used = false;
			mtx_unlock(&frpc.mtx);
			return (error);
		}
	}
	fl = malloc(sizeof(*fl), M_FASTRPC, M_WAITOK | M_ZERO);
	fl->sess = s;
	fl->client_id = i + 1;		/* any nonzero ID, as Linux's */
	fl->pd = ROOT_PD;
	sx_init(&fl->lock, "fastrpc user");
	TAILQ_INIT(&fl->mmaps);
	error = devfs_set_cdevpriv(fl, fastrpc_dtor);
	if (error != 0)
		fastrpc_dtor(fl);
	return (error);
}

static int
fastrpc_ioctl(struct cdev *dev, u_long cmd, caddr_t data, int fflag,
    struct thread *td)
{
	struct fastrpc_invoke_args *args;
	struct fastrpc_invoke *inv;
	struct fastrpc_user *fl;
	u_int n;
	int error;

	error = devfs_get_cdevpriv((void **)&fl);
	if (error != 0)
		return (error);
	/*
	 * Calls run side by side, as on Linux: a listener thread waits in
	 * one for the DSP's requests while others call.  Only setting up
	 * the process takes the lock.
	 */
	switch (cmd) {
	case FASTRPC_IOCTL_INIT_ATTACH:
	case FASTRPC_IOCTL_INIT_ATTACH_SNS:
	case FASTRPC_IOCTL_INIT_CREATE:
		sx_xlock(&fl->lock);
		break;
	}
	switch (cmd) {
	case FASTRPC_IOCTL_INVOKE:
		inv = (struct fastrpc_invoke *)data;
		if (inv->handle == FASTRPC_INIT_HANDLE) {
			error = EPERM;
			break;
		}
		n = SC_LENGTH(inv->sc);
		args = n == 0 ? NULL : malloc(n * sizeof(*args), M_FASTRPC,
		    M_WAITOK);
		error = n == 0 ? 0 : copyin((void *)(uintptr_t)inv->args,
		    args, n * sizeof(*args));
		if (error == 0)
			error = fastrpc_invoke_args(fl, false, inv->handle,
			    inv->sc, args);
		free(args, M_FASTRPC);
		break;
	case FASTRPC_IOCTL_INIT_ATTACH:
		error = fastrpc_init_attach(fl, ROOT_PD);
		break;
	case FASTRPC_IOCTL_INIT_ATTACH_SNS:
		error = fastrpc_init_attach(fl, SENSORS_PD);
		break;
	case FASTRPC_IOCTL_INIT_CREATE:
		error = fastrpc_init_create(fl,
		    (struct fastrpc_init_create *)data);
		break;
	case FASTRPC_IOCTL_GET_DSP_INFO:
		error = fastrpc_get_dsp_info(fl,
		    (struct fastrpc_ioctl_capability *)data);
		break;
	case FASTRPC_IOCTL_SET_OPTION:
		error = EOPNOTSUPP;	/* poll mode: not on this SoC */
		break;
	case FASTRPC_IOCTL_ALLOC_DMA_BUFF:
		error = fastrpc_alloc_shm(td,
		    (struct fastrpc_alloc_dma_buf *)data);
		break;
	case FASTRPC_IOCTL_FREE_DMA_BUFF:
		/* Closing the descriptor frees it. */
		error = 0;
		break;
	case FASTRPC_IOCTL_MMAP:
		error = fastrpc_req_mmap(fl, (struct fastrpc_req_mmap *)data);
		break;
	case FASTRPC_IOCTL_MUNMAP:
		error = fastrpc_req_munmap(fl,
		    (struct fastrpc_req_munmap *)data);
		break;
	case FASTRPC_IOCTL_MEM_MAP:
	case FASTRPC_IOCTL_MEM_UNMAP:
	case FASTRPC_IOCTL_INIT_CREATE_STATIC:
		error = EOPNOTSUPP;
		break;
	default:
		error = ENOTTY;
		break;
	}
	if (sx_xlocked(&fl->lock))
		sx_xunlock(&fl->lock);
	return (error);
}

static struct cdevsw fastrpc_cdevsw = {
	.d_version =	D_VERSION,
	.d_name =	"fastrpc",
	.d_open =	fastrpc_open,
	.d_ioctl =	fastrpc_ioctl,
};

/* The channel, once the CDSP is up; then the device. */
static void
fastrpc_start(void *arg __unused, int pending __unused)
{
	struct qcom_glink_chan *ch;
	int error;

	error = qcom_glink_open("cdsp", "fastrpcglink-apps-dsp", 1024, 8,
	    fastrpc_rx, NULL, &ch);
	if (error != 0) {
		if (++frpc.start_tries < FASTRPC_START_TRIES)
			taskqueue_enqueue_timeout(frpc.tq, &frpc.start_task,
			    hz);
		else
			printf("qcom_fastrpc: no channel to the CDSP: %d\n",
			    error);
		return;
	}
	mtx_lock(&frpc.mtx);
	frpc.ch = ch;
	mtx_unlock(&frpc.mtx);
	frpc.cdev = make_dev(&fastrpc_cdevsw, 0, UID_ROOT, GID_WHEEL, 0660,
	    "fastrpc-cdsp");
	printf("qcom_fastrpc: fastrpc-cdsp: %zu sessions\n",
	    (size_t)FASTRPC_SESSIONS);
}

static int
qcom_fastrpc_modevent(module_t mod, int type, void *data)
{
	u_int i;

	switch (type) {
	case MOD_LOAD:
		mtx_init(&frpc.mtx, "qcom_fastrpc", NULL, MTX_DEF);
		for (i = 0; i < FASTRPC_SESSIONS; i++)
			frpc.sess[i].bank = &fastrpc_banks[i];
		frpc.tq = taskqueue_create("qcom_fastrpc", M_WAITOK,
		    taskqueue_thread_enqueue, &frpc.tq);
		taskqueue_start_threads(&frpc.tq, 1, PWAIT, "qcom_fastrpc");
		TIMEOUT_TASK_INIT(frpc.tq, &frpc.start_task, 0, fastrpc_start,
		    NULL);
		TASK_INIT(&frpc.reap_task, 0, fastrpc_reap, NULL);
		STAILQ_INIT(&frpc.reap);
		taskqueue_enqueue_timeout(frpc.tq, &frpc.start_task, 0);
		return (0);
	case MOD_UNLOAD:
		/* The SMMU banks have no detach; keep them, and the module. */
		return (EBUSY);
	default:
		return (EOPNOTSUPP);
	}
}

static moduledata_t qcom_fastrpc_mod = { "qcom_fastrpc",
    qcom_fastrpc_modevent, NULL };
DECLARE_MODULE(qcom_fastrpc, qcom_fastrpc_mod, SI_SUB_DRIVERS, SI_ORDER_ANY);
MODULE_DEPEND(qcom_fastrpc, qcom_glink, 1, 1, 1);
MODULE_DEPEND(qcom_fastrpc, qcom_smmu, 1, 1, 1);
MODULE_VERSION(qcom_fastrpc, 1);
