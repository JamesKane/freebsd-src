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
 * The Qualcomm "apps" SMMU as an iommu(4) unit; see qcom_apps_iommu.h.
 *
 * One domain per context, as the arm64 iommu(4) glue makes them, and one
 * context bank per domain (qcom_apps_smmu(4)'s, with its own page table).
 * The domain's addresses are the SMMU's 36 bits, less what the claim leaves
 * out of its window: reserved in the domain's address allocator.
 */

#include "opt_acpi.h"
#include "opt_iommu.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/bus.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/memdesc.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/queue.h>
#include <sys/sysctl.h>
#include <sys/taskqueue.h>
#include <sys/tree.h>

#include <vm/vm.h>
#include <vm/pmap.h>
#include <vm/vm_page.h>

#include <machine/bus.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include <dev/qcom_smmu/qcom_smmu.h>
#include <dev/qcom_smmu/qcom_apps_smmu.h>
#include <dev/qcom_smmu/qcom_apps_iommu.h>

#ifdef IOMMU
#include <dev/pci/pcivar.h>
#include <dev/iommu/iommu.h>
#include <dev/iommu/busdma_iommu.h>
#include <arm64/iommu/iommu.h>

#include "iommu_if.h"

#define	QCOM_APPS_IOMMU_END	(1ul << 36)	/* the SMMU's input range */

struct qcom_apps_iommu_claim {
	LIST_ENTRY(qcom_apps_iommu_claim) next;
	device_t	dev;
	u_int		nstreams;
	uint16_t	sids[QCOM_APPS_SMMU_MAXSTREAMS];
	uint16_t	masks[QCOM_APPS_SMMU_MAXSTREAMS];
	uint64_t	iova_start;
	uint64_t	iova_end;
	u_int		flags;
};

struct qcom_apps_iommu_domain {
	struct iommu_domain		iodom;	/* first */
	LIST_ENTRY(qcom_apps_iommu_domain) next;
	LIST_HEAD(, qcom_apps_iommu_ctx) ctxs;
	struct qcom_apps_smmu_dom	*d;
};

struct qcom_apps_iommu_ctx {
	struct iommu_ctx		ioctx;	/* first */
	LIST_ENTRY(qcom_apps_iommu_ctx)	next;
	device_t			dev;
	struct qcom_apps_iommu_domain	*domain;
};

struct qcom_apps_iommu_softc {
	struct iommu_unit		iommu;	/* first */
	device_t			dev;
	LIST_HEAD(, qcom_apps_iommu_domain) domains;
};

static MALLOC_DEFINE(M_QCOM_APPS_IOMMU, "qcom_apps_iommu",
    "Qualcomm apps SMMU as an IOMMU");

static struct mtx qcom_apps_iommu_claims_mtx;
MTX_SYSINIT(qcom_apps_iommu_claims, &qcom_apps_iommu_claims_mtx,
    "qcom_apps_iommu claims", MTX_DEF);
static LIST_HEAD(, qcom_apps_iommu_claim) qcom_apps_iommu_claims =
    LIST_HEAD_INITIALIZER(qcom_apps_iommu_claims);
static struct qcom_apps_iommu_softc *qcom_apps_iommu_sc;

/* SoCs with an apps SMMU this driver knows (\_SB.SOID). */
static const uint32_t qcom_apps_iommu_socs[] = {
	449,		/* SC8280XP */
};

static struct qcom_apps_iommu_claim *
qcom_apps_iommu_find_claim(device_t dev)
{
	struct qcom_apps_iommu_claim *c;

	mtx_assert(&qcom_apps_iommu_claims_mtx, MA_OWNED);
	LIST_FOREACH(c, &qcom_apps_iommu_claims, next)
		if (c->dev == dev)
			return (c);
	return (NULL);
}

int
qcom_apps_iommu_claim(device_t dev, const uint16_t *sids,
    const uint16_t *masks, u_int nstreams, uint64_t iova_start,
    uint64_t iova_end, u_int flags)
{
	struct qcom_apps_iommu_claim *c;
	u_int i;

	if (qcom_apps_iommu_sc == NULL)
		return (ENXIO);
	if (nstreams == 0 || nstreams > QCOM_APPS_SMMU_MAXSTREAMS ||
	    iova_start >= iova_end || iova_end > QCOM_APPS_IOMMU_END)
		return (EINVAL);
	c = malloc(sizeof(*c), M_QCOM_APPS_IOMMU, M_WAITOK | M_ZERO);
	c->dev = dev;
	c->nstreams = nstreams;
	for (i = 0; i < nstreams; i++) {
		c->sids[i] = sids[i];
		c->masks[i] = masks[i];
	}
	c->iova_start = round_page(iova_start);
	c->iova_end = trunc_page(iova_end);
	c->flags = flags;
	mtx_lock(&qcom_apps_iommu_claims_mtx);
	if (qcom_apps_iommu_find_claim(dev) != NULL) {
		mtx_unlock(&qcom_apps_iommu_claims_mtx);
		free(c, M_QCOM_APPS_IOMMU);
		return (EEXIST);
	}
	LIST_INSERT_HEAD(&qcom_apps_iommu_claims, c, next);
	mtx_unlock(&qcom_apps_iommu_claims_mtx);
	return (0);
}

void
qcom_apps_iommu_unclaim(device_t dev)
{
	struct qcom_apps_iommu_claim *c;

	mtx_lock(&qcom_apps_iommu_claims_mtx);
	c = qcom_apps_iommu_find_claim(dev);
	if (c != NULL)
		LIST_REMOVE(c, next);
	mtx_unlock(&qcom_apps_iommu_claims_mtx);
	free(c, M_QCOM_APPS_IOMMU);
}

/* iommu(4) interface */

static int
qcom_apps_iommu_find(device_t dev, device_t child)
{
	bool found;

	mtx_lock(&qcom_apps_iommu_claims_mtx);
	found = qcom_apps_iommu_find_claim(child) != NULL;
	mtx_unlock(&qcom_apps_iommu_claims_mtx);
	return (found ? 0 : ENOENT);
}

static int
qcom_apps_iommu_map(device_t dev, struct iommu_domain *iodom,
    vm_offset_t va, vm_page_t *ma, vm_size_t size, vm_prot_t prot)
{
	struct qcom_apps_iommu_domain *domain;

	domain = (struct qcom_apps_iommu_domain *)iodom;
	if (domain->d == NULL)
		return (ENXIO);
	return (qcom_apps_smmu_map_at(domain->d, va, ma, atop(size),
	    (iodom->flags & IOMMU_DOMAIN_NONCOHERENT) != 0 ? 0 :
	    QCOM_SMMU_CACHED));
}

static int
qcom_apps_iommu_unmap(device_t dev, struct iommu_domain *iodom,
    vm_offset_t va, bus_size_t size)
{
	struct qcom_apps_iommu_domain *domain;

	domain = (struct qcom_apps_iommu_domain *)iodom;
	if (domain->d != NULL)
		qcom_apps_smmu_unmap_at(domain->d, va, size);
	return (0);
}

static struct iommu_domain *
qcom_apps_iommu_domain_alloc(device_t dev, struct iommu_unit *iommu)
{
	struct qcom_apps_iommu_softc *sc;
	struct qcom_apps_iommu_domain *domain;

	sc = device_get_softc(dev);
	domain = malloc(sizeof(*domain), M_QCOM_APPS_IOMMU, M_WAITOK | M_ZERO);
	LIST_INIT(&domain->ctxs);
	domain->iodom.end = QCOM_APPS_IOMMU_END;
	IOMMU_LOCK(iommu);
	LIST_INSERT_HEAD(&sc->domains, domain, next);
	IOMMU_UNLOCK(iommu);
	return (&domain->iodom);
}

static void
qcom_apps_iommu_domain_free(device_t dev, struct iommu_domain *iodom)
{
	struct qcom_apps_iommu_domain *domain;

	domain = (struct qcom_apps_iommu_domain *)iodom;
	LIST_REMOVE(domain, next);
	if (domain->d != NULL)
		qcom_apps_smmu_detach(domain->d);
	free(domain, M_QCOM_APPS_IOMMU);
}

static struct iommu_ctx *
qcom_apps_iommu_ctx_alloc(device_t dev, struct iommu_domain *iodom,
    device_t child, bool disabled)
{
	struct qcom_apps_iommu_domain *domain;
	struct qcom_apps_iommu_ctx *ctx;

	domain = (struct qcom_apps_iommu_domain *)iodom;
	ctx = malloc(sizeof(*ctx), M_QCOM_APPS_IOMMU, M_WAITOK | M_ZERO);
	ctx->dev = child;
	ctx->domain = domain;
	IOMMU_DOMAIN_LOCK(iodom);
	LIST_INSERT_HEAD(&domain->ctxs, ctx, next);
	IOMMU_DOMAIN_UNLOCK(iodom);
	return (&ctx->ioctx);
}

/* The claim's streams through a bank, and its window. */
static int
qcom_apps_iommu_ctx_init(device_t dev, struct iommu_ctx *ioctx)
{
	struct qcom_apps_iommu_claim c, *cp;
	struct qcom_apps_iommu_ctx *ctx;
	struct iommu_domain *iodom;
	int error;

	ctx = (struct qcom_apps_iommu_ctx *)ioctx;
	iodom = &ctx->domain->iodom;
	mtx_lock(&qcom_apps_iommu_claims_mtx);
	cp = qcom_apps_iommu_find_claim(ctx->dev);
	if (cp != NULL)
		c = *cp;
	mtx_unlock(&qcom_apps_iommu_claims_mtx);
	if (cp == NULL)
		return (ENOENT);

	error = qcom_apps_smmu_attach_streams(c.sids, c.masks, c.nstreams,
	    QCOM_APPS_SMMU_IOMMU, &ctx->domain->d);
	if (error != 0) {
		device_printf(dev, "%s: streams: error %d\n",
		    device_get_nameunit(ctx->dev), error);
		return (error);
	}
	if (c.iova_start > 0)
		error = iommu_gas_reserve_region_extend(iodom, 0, c.iova_start);
	/*
	 * A reservation must end below the domain's end, whose last page
	 * stays outside it; the device's own tags limit its addresses too.
	 */
	if (error == 0 && c.iova_end < iodom->end - IOMMU_PAGE_SIZE)
		error = iommu_gas_reserve_region_extend(iodom, c.iova_end,
		    iodom->end - IOMMU_PAGE_SIZE);
	if (error != 0) {
		device_printf(dev, "%s: cannot reserve outside %#jx-%#jx: %d\n",
		    device_get_nameunit(ctx->dev), (uintmax_t)c.iova_start,
		    (uintmax_t)c.iova_end, error);
		return (error);
	}
	/* Mapped write-back and shareable, without cache maintenance. */
	if ((c.flags & QCOM_APPS_IOMMU_COHERENT) != 0)
		iodom->flags &= ~IOMMU_DOMAIN_NONCOHERENT;
	return (0);
}

static void
qcom_apps_iommu_ctx_free(device_t dev, struct iommu_ctx *ioctx)
{
	struct qcom_apps_iommu_ctx *ctx;

	ctx = (struct qcom_apps_iommu_ctx *)ioctx;
	LIST_REMOVE(ctx, next);
	free(ctx, M_QCOM_APPS_IOMMU);
}

static struct iommu_ctx *
qcom_apps_iommu_ctx_lookup(device_t dev, device_t child)
{
	struct qcom_apps_iommu_softc *sc;
	struct qcom_apps_iommu_domain *domain;
	struct qcom_apps_iommu_ctx *ctx;

	sc = device_get_softc(dev);
	IOMMU_ASSERT_LOCKED(&sc->iommu);
	LIST_FOREACH(domain, &sc->domains, next) {
		IOMMU_DOMAIN_LOCK(&domain->iodom);
		LIST_FOREACH(ctx, &domain->ctxs, next) {
			if (ctx->dev == child) {
				IOMMU_DOMAIN_UNLOCK(&domain->iodom);
				return (&ctx->ioctx);
			}
		}
		IOMMU_DOMAIN_UNLOCK(&domain->iodom);
	}
	return (NULL);
}

/* Children (the test's) get their tags as ACPI's children do. */
static bus_dma_tag_t
qcom_apps_iommu_get_dma_tag(device_t dev, device_t child)
{
	bus_dma_tag_t tag;

	tag = iommu_get_dma_tag(dev, child);
	return (tag != NULL ? tag : bus_get_dma_tag(dev));
}

/*
 * By hand, for bring-up: a child claims the SC8280XP video codec's streams
 * and window, as its driver will, and a buffer is allocated and loaded
 * through its tag; each page's I/O address must be in the window and
 * translate, in the bank's table, to the page.  Then everything is let go.
 */
#define	TEST_SIZE	(256 * 1024)

struct qcom_apps_iommu_test_seg {
	bus_addr_t	addr;
	bus_size_t	len;
	int		nsegs;
};

static void
qcom_apps_iommu_test_cb(void *arg, bus_dma_segment_t *segs, int nsegs,
    int error)
{
	struct qcom_apps_iommu_test_seg *s = arg;

	if (error != 0 || nsegs < 1)
		return;
	s->addr = segs[0].ds_addr;
	s->len = segs[0].ds_len;
	s->nsegs = nsegs;
}

static int
qcom_apps_iommu_test(struct qcom_apps_iommu_softc *sc)
{
	static const uint16_t sids[] = { 0x2a00, 0x2a07 };
	static const uint16_t masks[] = { 0x400, 0x400 };
	struct qcom_apps_iommu_test_seg seg;
	struct qcom_apps_iommu_ctx *ctx;
	bus_dma_tag_t parent, tag;
	bus_dmamap_t map;
	device_t child;
	vm_paddr_t pa, want;
	uint8_t *buf;
	u_int i, bad;
	int error;

	bus_topo_lock();
	child = device_add_child(sc->dev, "qcom_apps_iommu_test",
	    DEVICE_UNIT_ANY);
	bus_topo_unlock();
	if (child == NULL)
		return (ENOMEM);
	error = qcom_apps_iommu_claim(child, sids, masks, nitems(sids),
	    0x25800000, 0xe0000000, 0);
	if (error != 0)
		goto out;
	parent = bus_get_dma_tag(child);
	if (parent == bus_get_dma_tag(sc->dev)) {
		device_printf(sc->dev, "test: not translated\n");
		error = ENXIO;
		goto unclaim;
	}
	error = bus_dma_tag_create(parent, PAGE_SIZE, 0, 0xdfffffff,
	    BUS_SPACE_MAXADDR, NULL, NULL, TEST_SIZE, 1, TEST_SIZE, 0, NULL,
	    NULL, &tag);
	if (error != 0)
		goto free_ctx;
	error = bus_dmamem_alloc(tag, (void **)&buf, BUS_DMA_WAITOK |
	    BUS_DMA_ZERO, &map);
	if (error != 0)
		goto tag;
	memset(&seg, 0, sizeof(seg));
	error = bus_dmamap_load(tag, map, buf, TEST_SIZE,
	    qcom_apps_iommu_test_cb, &seg, BUS_DMA_NOWAIT);
	if (error != 0 || seg.nsegs != 1 || seg.len != TEST_SIZE) {
		device_printf(sc->dev, "test: load error %d, %d segments\n",
		    error, seg.nsegs);
		error = error != 0 ? error : EIO;
		goto mem;
	}
	ctx = (struct qcom_apps_iommu_ctx *)
	    ((struct bus_dma_tag_iommu *)parent)->ctx;
	bad = 0;
	for (i = 0; i < TEST_SIZE; i += PAGE_SIZE) {
		want = pmap_kextract((vm_offset_t)buf + i);
		pa = qcom_apps_smmu_lookup(ctx->domain->d, seg.addr + i);
		if (pa != want) {
			if (bad++ < 4)
				device_printf(sc->dev, "test: %#jx -> %#jx, "
				    "not %#jx\n", (uintmax_t)(seg.addr + i),
				    (uintmax_t)pa, (uintmax_t)want);
		}
	}
	device_printf(sc->dev, "test: %u KB at I/O %#jx-%#jx (window "
	    "0x25800000-0xe0000000): %u of %u pages translate right, %s\n",
	    TEST_SIZE / 1024, (uintmax_t)seg.addr,
	    (uintmax_t)(seg.addr + TEST_SIZE - 1), TEST_SIZE / PAGE_SIZE - bad,
	    TEST_SIZE / PAGE_SIZE,
	    (ctx->ioctx.flags & IOMMU_CTX_NONCOHERENT) != 0 ? "non-coherent" :
	    "coherent");
	if (bad != 0 || seg.addr < 0x25800000 ||
	    seg.addr + TEST_SIZE > 0xe0000000)
		error = EIO;
	bus_dmamap_unload(tag, map);
	if (qcom_apps_smmu_lookup(ctx->domain->d, seg.addr) != 0) {
		device_printf(sc->dev, "test: still mapped after unload\n");
		error = EIO;
	}
mem:
	bus_dmamem_free(tag, buf, map);
tag:
	bus_dma_tag_destroy(tag);
free_ctx:
	/* The tag's reference on the context: the last, which frees it. */
	IOMMU_LOCK(&sc->iommu);
	iommu_free_ctx_locked(&sc->iommu,
	    ((struct bus_dma_tag_iommu *)parent)->ctx);
unclaim:
	qcom_apps_iommu_unclaim(child);
out:
	bus_topo_lock();
	device_delete_child(sc->dev, child);
	bus_topo_unlock();
	return (error);
}

static int
qcom_apps_iommu_test_sysctl(SYSCTL_HANDLER_ARGS)
{
	struct qcom_apps_iommu_softc *sc = arg1;
	int error, v;

	v = 0;
	error = sysctl_handle_int(oidp, &v, 0, req);
	if (error != 0 || req->newptr == NULL || v == 0)
		return (error);
	return (qcom_apps_iommu_test(sc));
}

/* The device */

static void
qcom_apps_iommu_identify(driver_t *driver, device_t parent)
{
	UINT32 id;
	u_int i;

	if (device_find_child(parent, "qcom_apps_iommu", DEVICE_UNIT_ANY) !=
	    NULL)
		return;
	if (ACPI_FAILURE(acpi_GetInteger(ACPI_ROOT_OBJECT, "\\_SB.SOID", &id)))
		return;
	for (i = 0; i < nitems(qcom_apps_iommu_socs); i++)
		if (qcom_apps_iommu_socs[i] == id)
			break;
	if (i < nitems(qcom_apps_iommu_socs))
		BUS_ADD_CHILD(parent, 10, "qcom_apps_iommu", 0);
}

static int
qcom_apps_iommu_probe(device_t dev)
{
	device_set_desc(dev, "Qualcomm apps SMMU, as an IOMMU");
	return (BUS_PROBE_NOWILDCARD);
}

static int
qcom_apps_iommu_attach(device_t dev)
{
	struct qcom_apps_iommu_softc *sc;

	sc = device_get_softc(dev);
	sc->dev = dev;
	LIST_INIT(&sc->domains);
	sc->iommu.dev = dev;
	sc->iommu.unit = device_get_unit(dev);
	iommu_register(&sc->iommu);
	/* Only claimed devices are in scope: translate them. */
	sc->iommu.dma_enabled = 1;
	qcom_apps_iommu_sc = sc;
	SYSCTL_ADD_PROC(device_get_sysctl_ctx(dev),
	    SYSCTL_CHILDREN(device_get_sysctl_tree(dev)), OID_AUTO, "test",
	    CTLTYPE_INT | CTLFLAG_WR | CTLFLAG_MPSAFE, sc, 0,
	    qcom_apps_iommu_test_sysctl, "I",
	    "Map a buffer through the video codec's streams and check it");
	return (0);
}

static int
qcom_apps_iommu_detach(device_t dev)
{
	struct qcom_apps_iommu_softc *sc;

	sc = device_get_softc(dev);
	if (!LIST_EMPTY(&sc->domains) ||
	    !LIST_EMPTY(&qcom_apps_iommu_claims))
		return (EBUSY);
	qcom_apps_iommu_sc = NULL;
	iommu_unregister(&sc->iommu);
	return (0);
}

static device_method_t qcom_apps_iommu_methods[] = {
	DEVMETHOD(device_identify,	qcom_apps_iommu_identify),
	DEVMETHOD(device_probe,		qcom_apps_iommu_probe),
	DEVMETHOD(device_attach,	qcom_apps_iommu_attach),
	DEVMETHOD(device_detach,	qcom_apps_iommu_detach),

	DEVMETHOD(bus_get_dma_tag,	qcom_apps_iommu_get_dma_tag),

	DEVMETHOD(iommu_find,		qcom_apps_iommu_find),
	DEVMETHOD(iommu_map,		qcom_apps_iommu_map),
	DEVMETHOD(iommu_unmap,		qcom_apps_iommu_unmap),
	DEVMETHOD(iommu_domain_alloc,	qcom_apps_iommu_domain_alloc),
	DEVMETHOD(iommu_domain_free,	qcom_apps_iommu_domain_free),
	DEVMETHOD(iommu_ctx_alloc,	qcom_apps_iommu_ctx_alloc),
	DEVMETHOD(iommu_ctx_init,	qcom_apps_iommu_ctx_init),
	DEVMETHOD(iommu_ctx_free,	qcom_apps_iommu_ctx_free),
	DEVMETHOD(iommu_ctx_lookup,	qcom_apps_iommu_ctx_lookup),

	DEVMETHOD_END
};

static driver_t qcom_apps_iommu_driver = {
	"qcom_apps_iommu",
	qcom_apps_iommu_methods,
	sizeof(struct qcom_apps_iommu_softc),
};

DRIVER_MODULE(qcom_apps_iommu, acpi, qcom_apps_iommu_driver, NULL, NULL);
MODULE_DEPEND(qcom_apps_iommu, acpi, 1, 1, 1);
MODULE_DEPEND(qcom_apps_iommu, qcom_smmu, 1, 1, 1);

#else /* !IOMMU */

/* Without options IOMMU nothing translates: no device is ever claimed. */
int
qcom_apps_iommu_claim(device_t dev, const uint16_t *sids,
    const uint16_t *masks, u_int nstreams, uint64_t iova_start,
    uint64_t iova_end, u_int flags)
{
	return (ENXIO);
}

void
qcom_apps_iommu_unclaim(device_t dev)
{
}

#endif /* IOMMU */

MODULE_VERSION(qcom_apps_iommu, 1);
