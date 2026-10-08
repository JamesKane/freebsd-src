/*-
 * Copyright (c) 2010 Isilon Systems, Inc.
 * Copyright (c) 2010 iX Systems, Inc.
 * Copyright (c) 2010 Panasas, Inc.
 * Copyright (c) 2013-2015 Mellanox Technologies, Ltd.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice unmodified, this list of conditions, and the following
 *    disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/pci.h>

#include <sys/param.h>
#include <sys/bus.h>
#include <sys/lock.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/rman.h>
#include <sys/interrupt.h>

struct irq_ent {
	struct list_head	links;
	struct list_head	all;	/* lkpi_irq_all */
	struct device	*dev;
	struct resource	*res;
	void		*arg;
	irqreturn_t	(*handler)(int, void *);
	irqreturn_t	(*thread_handler)(int, void *);
	void		*tag;
	unsigned int	irq;
	bool		masked;	/* disable_irq_nosync() from its handler */
};

/*
 * Every requested IRQ, for disable_irq_nosync() from an interrupt handler,
 * where neither the devices' sx lock nor tearing the handler down may sleep.
 */
static struct mtx lkpi_irq_all_mtx;
MTX_SYSINIT(lkpi_irq_all, &lkpi_irq_all_mtx, "lkpi irqs", MTX_DEF);
static LIST_HEAD(lkpi_irq_all);

/* The PCI or platform device an IRQ number belongs to. */
static struct device *
lkpi_find_irq_dev(unsigned int irq)
{
	struct device *dev;

	dev = lkpi_pci_find_irq_dev(irq);
	if (dev == NULL)
		dev = lkpi_platform_find_irq_dev(irq);
	return (dev);
}

/* The rid of the IRQ on the device's FreeBSD device, or -1. */
static int
lkpi_irq_rid(struct device *dev, unsigned int irq)
{
	struct resource_list *rl;
	struct resource_list_entry *rle;

	if (dev_is_pci(dev)) {
		/* check for MSI- or MSIX- interrupt */
		if (irq >= dev->irq_start && irq < dev->irq_end)
			return (irq - dev->irq_start + 1);
		else
			return (0);
	}
	rl = BUS_GET_RESOURCE_LIST(device_get_parent(dev->bsddev),
	    dev->bsddev);
	if (rl == NULL)
		return (-1);
	STAILQ_FOREACH(rle, rl, link)
		if (rle->type == SYS_RES_IRQ && rle->start == irq)
			return (rle->rid);
	return (-1);
}

static inline struct irq_ent *
lkpi_irq_ent(struct device *dev, unsigned int irq)
{
	struct irq_ent *irqe;

	list_for_each_entry(irqe, &dev->irqents, links)
		if (irqe->irq == irq)
			return (irqe);

	return (NULL);
}

static void
lkpi_irq_handler(void *ent)
{
	struct irq_ent *irqe;

	if (linux_set_current_flags(curthread, M_NOWAIT))
		return;

	/*
	 * Without a primary handler, Linux's default one wakes the thread.
	 * Here the interrupt thread runs it, with the interrupt masked until
	 * it returns, as with IRQF_ONESHOT.
	 */
	irqe = ent;
	if ((irqe->handler == NULL ||
	    irqe->handler(irqe->irq, irqe->arg) == IRQ_WAKE_THREAD) &&
	    irqe->thread_handler != NULL) {
		THREAD_SLEEPING_OK();
		irqe->thread_handler(irqe->irq, irqe->arg);
		THREAD_NO_SLEEPING();
	}
}

/*
 * Interrupt handlers run in the network epoch, which forbids sleeping, except
 * those with a threaded handler: that may sleep, as on Linux.
 */
static inline int
lkpi_irq_flags(struct irq_ent *irqe)
{
	if (irqe->thread_handler != NULL)
		return (INTR_TYPE_MISC | INTR_MPSAFE);
	return (INTR_TYPE_NET | INTR_MPSAFE);
}

static inline void
lkpi_irq_release(struct device *dev, struct irq_ent *irqe)
{
	if (irqe->tag != NULL)
		bus_teardown_intr(dev->bsddev, irqe->res, irqe->tag);
	if (irqe->res != NULL)
		bus_release_resource(dev->bsddev, SYS_RES_IRQ,
		    rman_get_rid(irqe->res), irqe->res);
	list_del(&irqe->links);
	mtx_lock(&lkpi_irq_all_mtx);
	list_del(&irqe->all);
	mtx_unlock(&lkpi_irq_all_mtx);
}

static void
lkpi_devm_irq_release(struct device *dev, void *p)
{
	struct irq_ent *irqe;

	if (dev == NULL || p == NULL)
		return;

	irqe = p;
	lkpi_irq_release(dev, irqe);
}

int
lkpi_request_irq(struct device *xdev, unsigned int irq,
    irq_handler_t handler, irq_handler_t thread_handler,
    unsigned long flags, const char *name, void *arg)
{
	struct resource *res;
	struct irq_ent *irqe;
	struct device *dev;
	unsigned resflags;
	int error;
	int rid;

	dev = lkpi_find_irq_dev(irq);
	if (dev == NULL)
		return -ENXIO;
	if (xdev != NULL && xdev != dev)
		return -ENXIO;
	rid = lkpi_irq_rid(dev, irq);
	if (rid < 0)
		return (-ENXIO);
	resflags = RF_ACTIVE;
	if ((flags & IRQF_SHARED) != 0)
		resflags |= RF_SHAREABLE;
	res = bus_alloc_resource_any(dev->bsddev, SYS_RES_IRQ, &rid, resflags);
	if (res == NULL)
		return (-ENXIO);
	if (xdev != NULL)
		irqe = lkpi_devres_alloc(lkpi_devm_irq_release, sizeof(*irqe),
		    GFP_KERNEL | __GFP_ZERO);
	else
		irqe = kzalloc(sizeof(*irqe), GFP_KERNEL);
	irqe->dev = dev;
	irqe->res = res;
	irqe->arg = arg;
	irqe->handler = handler;
	irqe->thread_handler = thread_handler;
	irqe->irq = irq;

	/* With IRQF_NO_AUTOEN, the handler is set up by enable_irq(). */
	if ((flags & IRQF_NO_AUTOEN) == 0) {
		error = bus_setup_intr(dev->bsddev, res, lkpi_irq_flags(irqe),
		    NULL, lkpi_irq_handler, irqe, &irqe->tag);
		if (error)
			goto errout;
	}
	list_add(&irqe->links, &dev->irqents);
	mtx_lock(&lkpi_irq_all_mtx);
	list_add(&irqe->all, &lkpi_irq_all);
	mtx_unlock(&lkpi_irq_all_mtx);
	if (xdev != NULL)
		devres_add(xdev, irqe);

	return 0;

errout:
	bus_release_resource(dev->bsddev, SYS_RES_IRQ, rid, irqe->res);
	if (xdev != NULL)
		devres_free(irqe);
	else
		kfree(irqe);
	return (-error);
}

int
lkpi_enable_irq(unsigned int irq)
{
	struct irq_ent *irqe;
	struct device *dev;

	/* Masked by its handler: still set up, so just unmasked. */
	mtx_lock(&lkpi_irq_all_mtx);
	list_for_each_entry(irqe, &lkpi_irq_all, all)
		if (irqe->irq == irq && irqe->masked) {
			irqe->masked = false;
			mtx_unlock(&lkpi_irq_all_mtx);
			return (0);
		}
	mtx_unlock(&lkpi_irq_all_mtx);

	dev = lkpi_find_irq_dev(irq);
	if (dev == NULL)
		return -EINVAL;
	irqe = lkpi_irq_ent(dev, irq);
	if (irqe == NULL || irqe->tag != NULL)
		return -EINVAL;
	return -bus_setup_intr(dev->bsddev, irqe->res, lkpi_irq_flags(irqe),
	    NULL, lkpi_irq_handler, irqe, &irqe->tag);
}

void
lkpi_disable_irq(unsigned int irq)
{
	struct irq_ent *irqe;
	struct device *dev;

	/*
	 * From an interrupt handler (Linux's threaded drivers' primary
	 * handlers call disable_irq_nosync()): the interrupt thread keeps its
	 * source masked until it returns, and runs the threaded handler,
	 * which enables it again, before then.  So only note it.
	 */
	if ((curthread->td_pflags & TDP_ITHREAD) != 0) {
		mtx_lock(&lkpi_irq_all_mtx);
		list_for_each_entry(irqe, &lkpi_irq_all, all)
			if (irqe->irq == irq) {
				irqe->masked = true;
				break;
			}
		mtx_unlock(&lkpi_irq_all_mtx);
		return;
	}

	dev = lkpi_find_irq_dev(irq);
	if (dev == NULL)
		return;
	irqe = lkpi_irq_ent(dev, irq);
	if (irqe == NULL)
		return;
	if (irqe->tag != NULL)
		bus_teardown_intr(dev->bsddev, irqe->res, irqe->tag);
	irqe->tag = NULL;
}

int
lkpi_bind_irq_to_cpu(unsigned int irq, int cpu_id)
{
	struct irq_ent *irqe;
	struct device *dev;

	dev = lkpi_find_irq_dev(irq);
	if (dev == NULL)
		return (-ENOENT);

	irqe = lkpi_irq_ent(dev, irq);
	if (irqe == NULL)
		return (-ENOENT);

	return (-bus_bind_intr(dev->bsddev, irqe->res, cpu_id));
}

void
lkpi_free_irq(unsigned int irq, void *device __unused)
{
	struct irq_ent *irqe;
	struct device *dev;

	dev = lkpi_find_irq_dev(irq);
	if (dev == NULL)
		return;
	irqe = lkpi_irq_ent(dev, irq);
	if (irqe == NULL)
		return;
	lkpi_irq_release(dev, irqe);
	kfree(irqe);
}

void
lkpi_devm_free_irq(struct device *xdev, unsigned int irq, void *p __unused)
{
	struct device *dev;
	struct irq_ent *irqe;

	dev = lkpi_find_irq_dev(irq);
	if (dev == NULL)
		return;
	if (xdev != dev)
		return;
	irqe = lkpi_irq_ent(dev, irq);
	if (irqe == NULL)
		return;
	lkpi_irq_release(dev, irqe);
	lkpi_devres_unlink(dev, irqe);
	lkpi_devres_free(irqe);
	return;
}
