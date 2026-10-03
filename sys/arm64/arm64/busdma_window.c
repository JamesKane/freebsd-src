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
 * busdma for devices behind a fixed DMA window (see <machine/busdma_window.h>):
 * a window of CPU physical addresses [cpu_base, cpu_base + size) the device
 * sees at bus addresses [bus_base, bus_base + size).
 *
 * Memory from bus_dmamem_alloc() comes from inside the window, and
 * uncacheable for BUS_DMA_COHERENT (the devices this is for do not snoop):
 * syncing it is a barrier.  Other buffers, which must lie inside the window
 * (nothing bounces), are synced by cache maintenance.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/memdesc.h>
#include <sys/mutex.h>

#include <vm/vm.h>
#include <vm/vm_extern.h>
#include <vm/vm_kern.h>
#include <vm/vm_page.h>
#include <vm/pmap.h>

#include <machine/atomic.h>
#include <machine/bus.h>
#include <machine/busdma_window.h>
#include <machine/cpufunc.h>
#include <machine/md_var.h>
#include <arm64/include/bus_dma_impl.h>

static MALLOC_DEFINE(M_BUSDMA_WINDOW, "busdma_window", "busdma window metadata");

struct bus_dma_tag_window {
	struct bus_dma_tag_common common;
	vm_paddr_t		 cpu_base;
	bus_addr_t		 bus_base;
	bus_size_t		 size;
	bus_dma_segment_t	*segments;
};

/* A cacheable range a map was loaded with, to sync. */
struct window_sync {
	vm_paddr_t	pa;
	bus_size_t	len;
};

struct bus_dmamap_window {
	bool			 dmamem;	/* from bus_dmamem_alloc() */
	bool			 cacheable;	/* its memory */
	int			 nsync;
	struct window_sync	 sync[];
};

extern struct bus_dma_impl bus_dma_window_impl;

static struct bus_dma_tag_window *
to_window(bus_dma_tag_t dmat)
{
	return ((struct bus_dma_tag_window *)dmat);
}

static int
window_tag_init(struct bus_dma_tag_window *tag)
{
	tag->segments = malloc(sizeof(bus_dma_segment_t) *
	    MIN(tag->common.nsegments, 4096), M_BUSDMA_WINDOW,
	    M_NOWAIT | M_ZERO);
	return (tag->segments == NULL ? ENOMEM : 0);
}

int
bus_dma_window_tag_create(bus_dma_tag_t parent, vm_paddr_t cpu_base,
    bus_addr_t bus_base, bus_size_t size, bus_dma_tag_t *dmat)
{
	struct bus_dma_tag_common *pc;
	struct bus_dma_tag_window *tag;
	int error;

	*dmat = NULL;
	pc = (struct bus_dma_tag_common *)parent;
	error = common_bus_dma_tag_create(NULL, 1, 0, BUS_SPACE_MAXADDR,
	    BUS_SPACE_MAXADDR, BUS_SPACE_MAXSIZE, BUS_SPACE_UNRESTRICTED,
	    BUS_SPACE_MAXSIZE, pc != NULL ? pc->flags : 0, NULL, NULL,
	    sizeof(*tag), (void **)&tag);
	if (error != 0)
		return (error);
	tag->common.impl = &bus_dma_window_impl;
	tag->cpu_base = cpu_base;
	tag->bus_base = bus_base;
	tag->size = size;
	if ((error = window_tag_init(tag)) != 0) {
		free(tag, M_DEVBUF);
		return (error);
	}
	*dmat = (bus_dma_tag_t)tag;
	return (0);
}

static int
window_tag_create(bus_dma_tag_t parent, bus_size_t alignment,
    bus_addr_t boundary, bus_addr_t lowaddr, bus_addr_t highaddr,
    bus_size_t maxsize, int nsegments, bus_size_t maxsegsz, int flags,
    bus_dma_lock_t *lockfunc, void *lockfuncarg, bus_dma_tag_t *dmat)
{
	struct bus_dma_tag_window *ptag, *tag;
	int error;

	ptag = to_window(parent);
	error = common_bus_dma_tag_create(&ptag->common, alignment, boundary,
	    lowaddr, highaddr, maxsize, nsegments, maxsegsz, flags, lockfunc,
	    lockfuncarg, sizeof(*tag), (void **)&tag);
	if (error != 0)
		return (error);
	tag->cpu_base = ptag->cpu_base;
	tag->bus_base = ptag->bus_base;
	tag->size = ptag->size;
	if ((error = window_tag_init(tag)) != 0) {
		free(tag, M_DEVBUF);
		return (error);
	}
	*dmat = (bus_dma_tag_t)tag;
	return (0);
}

static int
window_tag_destroy(bus_dma_tag_t dmat)
{
	struct bus_dma_tag_window *tag = to_window(dmat);

	free(tag->segments, M_BUSDMA_WINDOW);
	free(tag, M_DEVBUF);
	return (0);
}

static int
window_tag_set_domain(bus_dma_tag_t dmat __unused)
{
	return (0);
}

static bool
window_id_mapped(bus_dma_tag_t dmat __unused, vm_paddr_t buf __unused,
    bus_size_t buflen __unused)
{
	return (false);
}

/* The CPU addresses the tag's device reaches: in the window, below lowaddr. */
static void
window_cpu_range(struct bus_dma_tag_window *tag, vm_paddr_t *low,
    vm_paddr_t *high)
{
	bus_addr_t bus_high;

	bus_high = tag->bus_base + tag->size - 1;
	if (tag->common.lowaddr < bus_high)
		bus_high = tag->common.lowaddr;
	*low = tag->cpu_base;
	*high = tag->cpu_base + (bus_high - tag->bus_base);
}

static struct bus_dmamap_window *
window_map_alloc(struct bus_dma_tag_window *tag, int mflags)
{
	return (malloc(sizeof(struct bus_dmamap_window) +
	    sizeof(struct window_sync) * MIN(tag->common.nsegments, 4096),
	    M_BUSDMA_WINDOW, mflags | M_ZERO));
}

static int
window_map_create(bus_dma_tag_t dmat, int flags __unused, bus_dmamap_t *mapp)
{
	struct bus_dmamap_window *map;

	map = window_map_alloc(to_window(dmat), M_NOWAIT);
	if (map == NULL)
		return (ENOMEM);
	map->cacheable = true;
	*mapp = (bus_dmamap_t)map;
	return (0);
}

static int
window_map_destroy(bus_dma_tag_t dmat __unused, bus_dmamap_t map)
{
	free(map, M_BUSDMA_WINDOW);
	return (0);
}

static int
window_mem_alloc(bus_dma_tag_t dmat, void **vaddr, int flags,
    bus_dmamap_t *mapp)
{
	struct bus_dma_tag_window *tag = to_window(dmat);
	struct bus_dmamap_window *map;
	vm_memattr_t attr;
	vm_paddr_t low, high;
	int mflags;

	mflags = (flags & BUS_DMA_NOWAIT) != 0 ? M_NOWAIT : M_WAITOK;
	if ((flags & BUS_DMA_ZERO) != 0)
		mflags |= M_ZERO;
	map = window_map_alloc(tag, mflags & ~M_ZERO);
	if (map == NULL)
		return (ENOMEM);
	map->dmamem = true;
	map->cacheable = (flags & (BUS_DMA_COHERENT | BUS_DMA_NOCACHE)) == 0;
	attr = map->cacheable ? VM_MEMATTR_DEFAULT : VM_MEMATTR_UNCACHEABLE;
	window_cpu_range(tag, &low, &high);
	*vaddr = kmem_alloc_contig(tag->common.maxsize, mflags, low, high,
	    MAX(tag->common.alignment, PAGE_SIZE), tag->common.boundary, attr);
	if (*vaddr == NULL) {
		free(map, M_BUSDMA_WINDOW);
		return (ENOMEM);
	}
	*mapp = (bus_dmamap_t)map;
	return (0);
}

static void
window_mem_free(bus_dma_tag_t dmat, void *vaddr, bus_dmamap_t map)
{
	kmem_free(vaddr, to_window(dmat)->common.maxsize);
	free(map, M_BUSDMA_WINDOW);
}

/* Add [pa, pa + len), inside the window, as bus segments. */
static int
window_load_range(struct bus_dma_tag_window *tag,
    struct bus_dmamap_window *map, vm_paddr_t pa, bus_size_t len,
    bus_dma_segment_t *segs, int *segp)
{
	bus_addr_t baddr, bmask;
	bus_size_t sgsize;
	int seg;

	if (pa < tag->cpu_base || pa + len > tag->cpu_base + tag->size)
		return (EFBIG);
	if (map->cacheable && map->nsync < (int)tag->common.nsegments) {
		map->sync[map->nsync].pa = pa;
		map->sync[map->nsync].len = len;
		map->nsync++;
	}
	baddr = pa - tag->cpu_base + tag->bus_base;
	if (baddr + len - 1 > tag->common.lowaddr)
		return (EFBIG);
	bmask = ~(tag->common.boundary - 1);
	seg = *segp;
	while (len > 0) {
		sgsize = MIN(len, tag->common.maxsegsz);
		if (tag->common.boundary != 0)
			sgsize = MIN(sgsize, tag->common.boundary -
			    (baddr & (tag->common.boundary - 1)));
		if (seg >= 0 && segs[seg].ds_addr + segs[seg].ds_len ==
		    baddr && segs[seg].ds_len + sgsize <=
		    tag->common.maxsegsz && (tag->common.boundary == 0 ||
		    (segs[seg].ds_addr & bmask) == (baddr & bmask)))
			segs[seg].ds_len += sgsize;
		else {
			if (++seg >= (int)tag->common.nsegments)
				return (EFBIG);
			segs[seg].ds_addr = baddr;
			segs[seg].ds_len = sgsize;
		}
		baddr += sgsize;
		len -= sgsize;
	}
	*segp = seg;
	return (0);
}

static int
window_load_phys(bus_dma_tag_t dmat, bus_dmamap_t map, vm_paddr_t buf,
    bus_size_t buflen, int flags __unused, bus_dma_segment_t *segs, int *segp)
{
	struct bus_dma_tag_window *tag = to_window(dmat);

	return (window_load_range(tag, (struct bus_dmamap_window *)map, buf,
	    buflen, segs != NULL ? segs : tag->segments, segp));
}

static int
window_load_buffer(bus_dma_tag_t dmat, bus_dmamap_t map, void *buf,
    bus_size_t buflen, pmap_t pmap, int flags __unused,
    bus_dma_segment_t *segs, int *segp)
{
	struct bus_dma_tag_window *tag = to_window(dmat);
	vm_offset_t va;
	vm_paddr_t pa;
	bus_size_t len;
	int error;

	if (segs == NULL)
		segs = tag->segments;
	va = (vm_offset_t)buf;
	while (buflen > 0) {
		len = MIN(buflen, PAGE_SIZE - (va & PAGE_MASK));
		pa = pmap == kernel_pmap ? pmap_kextract(va) :
		    pmap_extract(pmap, va);
		error = window_load_range(tag, (struct bus_dmamap_window *)map,
		    pa, len, segs, segp);
		if (error != 0)
			return (error);
		va += len;
		buflen -= len;
	}
	return (0);
}

static int
window_load_ma(bus_dma_tag_t dmat, bus_dmamap_t map, struct vm_page **ma,
    bus_size_t tlen, int ma_offs, int flags, bus_dma_segment_t *segs,
    int *segp)
{
	vm_paddr_t pa;
	bus_size_t len;
	int error, i;

	for (i = 0; tlen > 0; i++, ma_offs = 0) {
		len = MIN(tlen, PAGE_SIZE - ma_offs);
		pa = VM_PAGE_TO_PHYS(ma[i]) + ma_offs;
		error = window_load_phys(dmat, map, pa, len, flags, segs,
		    segp);
		if (error != 0)
			return (error);
		tlen -= len;
	}
	return (0);
}

static void
window_map_waitok(bus_dma_tag_t dmat __unused, bus_dmamap_t map __unused,
    struct memdesc *mem __unused, bus_dmamap_callback_t *callback __unused,
    void *callback_arg __unused)
{
}

static bus_dma_segment_t *
window_map_complete(bus_dma_tag_t dmat, bus_dmamap_t map __unused,
    bus_dma_segment_t *segs, int nsegs __unused, int error __unused)
{
	return (segs != NULL ? segs : to_window(dmat)->segments);
}

static void
window_map_unload(bus_dma_tag_t dmat __unused, bus_dmamap_t map)
{
	((struct bus_dmamap_window *)map)->nsync = 0;
}

static void
window_map_sync(bus_dma_tag_t dmat __unused, bus_dmamap_t mapp,
    bus_dmasync_op_t op)
{
	struct bus_dmamap_window *map = (struct bus_dmamap_window *)mapp;
	void *va;
	int i;

	if (map == NULL || !map->cacheable) {
		dsb(sy);
		return;
	}
	for (i = 0; i < map->nsync; i++) {
		va = (void *)PHYS_TO_DMAP(map->sync[i].pa);
		if ((op & BUS_DMASYNC_PREREAD) != 0)
			cpu_dcache_wbinv_range(va, map->sync[i].len);
		else if ((op & BUS_DMASYNC_PREWRITE) != 0)
			cpu_dcache_wb_range(va, map->sync[i].len);
		if ((op & BUS_DMASYNC_POSTREAD) != 0)
			cpu_dcache_inv_range(va, map->sync[i].len);
	}
	dsb(sy);
}

struct bus_dma_impl bus_dma_window_impl = {
	.tag_create = window_tag_create,
	.tag_destroy = window_tag_destroy,
	.tag_set_domain = window_tag_set_domain,
	.id_mapped = window_id_mapped,
	.map_create = window_map_create,
	.map_destroy = window_map_destroy,
	.mem_alloc = window_mem_alloc,
	.mem_free = window_mem_free,
	.load_ma = window_load_ma,
	.load_phys = window_load_phys,
	.load_buffer = window_load_buffer,
	.map_waitok = window_map_waitok,
	.map_complete = window_map_complete,
	.map_unload = window_map_unload,
	.map_sync = window_map_sync,
};
