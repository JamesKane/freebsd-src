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
 * The platform bus: devices that are on no bus of their own, and the
 * drivers bound to them.  Devices are matched to drivers by the driver's
 * id_table, or else by name, as Linux does when there is no device tree
 * match; a driver is tried for every unbound device whenever a device or a
 * driver is added, which also retries probes that were deferred.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/sx.h>

#include <linux/device.h>
#include <linux/err.h>
#include <linux/idr.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/list.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/string.h>

const struct bus_type platform_bus_type = { .name = "platform" };

/* Serializes the lists and binding; probe() may add devices or drivers. */
static struct sx lkpi_platform_lock;
SX_SYSINIT_FLAGS(lkpi_platform, &lkpi_platform_lock, "lkpi platform bus",
    SX_RECURSE);

static LIST_HEAD(lkpi_platform_devices);
static LIST_HEAD(lkpi_platform_drivers);
static DEFINE_IDA(lkpi_platform_ida);

/* A device made by platform_device_alloc(), with a copy of its name. */
struct lkpi_platform_object {
	struct platform_device	pdev;
	char			name[];
};

static const struct platform_device_id *
lkpi_platform_match_id(const struct platform_device_id *id,
    const struct platform_device *pdev)
{
	for (; id->name[0] != '\0'; id++)
		if (strcmp(pdev->name, id->name) == 0)
			return (id);
	return (NULL);
}

static bool
lkpi_platform_match(struct platform_device *pdev,
    struct platform_driver *pdrv)
{
	const struct platform_device_id *id;

	if (pdrv->id_table != NULL) {
		id = lkpi_platform_match_id(pdrv->id_table, pdev);
		if (id == NULL)
			return (false);
		pdev->id_entry = id;
		return (true);
	}
	return (strcmp(pdev->name, pdrv->driver.name) == 0);
}

/* Unbind, releasing what the driver got with devm_*(), as Linux does. */
static void
lkpi_platform_unbind(struct platform_device *pdev)
{
	lkpi_devres_release_free_list(&pdev->dev);
	dev_set_drvdata(&pdev->dev, NULL);
	pdev->dev.driver = NULL;
	pdev->id_entry = NULL;
}

static int
lkpi_platform_probe(struct platform_device *pdev,
    struct platform_driver *pdrv)
{
	int error;

	pdev->dev.driver = &pdrv->driver;
	error = pdrv->probe != NULL ? pdrv->probe(pdev) : 0;
	pdev->lkpi_deferred = error == -EPROBE_DEFER;
	if (error != 0) {
		if (error != -ENODEV && error != -ENXIO &&
		    error != -EPROBE_DEFER)
			dev_err(&pdev->dev, "probe of %s by %s failed: %d\n",
			    dev_name(&pdev->dev), pdrv->driver.name, error);
		lkpi_platform_unbind(pdev);
	}
	return (error);
}

static void
lkpi_platform_remove(struct platform_device *pdev)
{
	struct platform_driver *pdrv;

	pdrv = to_platform_driver(pdev->dev.driver);
	if (pdrv->remove != NULL)
		pdrv->remove(pdev);
	lkpi_platform_unbind(pdev);
}

static bool
lkpi_platform_try(struct platform_device *pdev, struct platform_driver *pdrv)
{
	return (pdev->dev.driver == NULL && lkpi_platform_match(pdev, pdrv) &&
	    lkpi_platform_probe(pdev, pdrv) == 0);
}

/*
 * Bind what a new device or driver makes bindable, as Linux does: a new
 * device tries the drivers, a new driver the unbound devices.  Then the
 * probes that were deferred are tried again: a bound device may be what one
 * was waiting for, so start again after each bind.  A probe that failed
 * otherwise is not retried: probing again would find the device as the
 * failed probe left it (panthor's GPU, with its clocks off).
 */
static void
lkpi_platform_probe_all(struct platform_device *newdev,
    struct platform_driver *newdrv)
{
	struct platform_device *pdev;
	struct platform_driver *pdrv;

	sx_assert(&lkpi_platform_lock, SA_XLOCKED);
	list_for_each_entry(pdev, &lkpi_platform_devices, lkpi_link) {
		if (newdev != NULL && pdev != newdev)
			continue;
		list_for_each_entry(pdrv, &lkpi_platform_drivers, lkpi_link) {
			if (newdrv != NULL && pdrv != newdrv)
				continue;
			if (lkpi_platform_try(pdev, pdrv))
				break;
		}
	}
again:
	list_for_each_entry(pdev, &lkpi_platform_devices, lkpi_link) {
		if (!pdev->lkpi_deferred)
			continue;
		list_for_each_entry(pdrv, &lkpi_platform_drivers, lkpi_link)
			if (lkpi_platform_try(pdev, pdrv))
				goto again;
	}
}

/* Drivers */

int
platform_driver_register(struct platform_driver *pdrv)
{
	sx_xlock(&lkpi_platform_lock);
	list_add_tail(&pdrv->lkpi_link, &lkpi_platform_drivers);
	lkpi_platform_probe_all(NULL, pdrv);
	sx_xunlock(&lkpi_platform_lock);
	return (0);
}

void
platform_driver_unregister(struct platform_driver *pdrv)
{
	struct platform_device *pdev;

	sx_xlock(&lkpi_platform_lock);
	list_for_each_entry(pdev, &lkpi_platform_devices, lkpi_link)
		if (pdev->dev.driver == &pdrv->driver)
			lkpi_platform_remove(pdev);
	list_del(&pdrv->lkpi_link);
	sx_xunlock(&lkpi_platform_lock);
}

int
platform_driver_probe(struct platform_driver *pdrv,
    int (*probe)(struct platform_device *))
{
	pdrv->probe = probe;
	return (platform_driver_register(pdrv));
}

/* Devices */

static void
lkpi_platform_object_release(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);

	kfree(dev->platform_data);
	kfree(pdev->resource);
	kfree(container_of(pdev, struct lkpi_platform_object, pdev));
}

static void
lkpi_platform_device_init(struct platform_device *pdev)
{
	kobject_init(&pdev->dev.kobj, &linux_dev_ktype);
	INIT_LIST_HEAD(&pdev->lkpi_link);
}

struct platform_device *
platform_device_alloc(const char *name, int id)
{
	struct lkpi_platform_object *obj;

	obj = kzalloc(sizeof(*obj) + strlen(name) + 1, GFP_KERNEL);
	if (obj == NULL)
		return (NULL);
	strcpy(obj->name, name);
	obj->pdev.name = obj->name;
	obj->pdev.id = id;
	obj->pdev.dev.release = lkpi_platform_object_release;
	lkpi_platform_device_init(&obj->pdev);
	return (&obj->pdev);
}

/*
 * Add a device: it stands for the FreeBSD device its creator set in
 * dev.bsddev, or else for its parent's, and can do DMA through it.
 */
int
platform_device_add(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	device_t bsddev;
	int error, id;

	bsddev = dev->bsddev;
	if (bsddev == NULL)
		bsddev = dev->parent != NULL ? dev->parent->bsddev :
		    linux_root_device.bsddev;
	lkpi_device_init(dev, dev->parent, bsddev);
	dev->bus = &platform_bus_type;

	if (pdev->id == PLATFORM_DEVID_AUTO) {
		id = ida_alloc(&lkpi_platform_ida, GFP_KERNEL);
		if (id < 0)
			return (id);
		pdev->id = id;
		pdev->id_auto = true;
	}
	if (pdev->id == PLATFORM_DEVID_NONE)
		error = kobject_set_name(&dev->kobj, "%s", pdev->name);
	else if (pdev->id_auto)
		error = kobject_set_name(&dev->kobj, "%s.%d.auto", pdev->name,
		    pdev->id);
	else
		error = kobject_set_name(&dev->kobj, "%s.%d", pdev->name,
		    pdev->id);
	if (error == 0 && dev->dma_priv == NULL)
		error = linux_dma_dev_init(dev);
	if (error != 0) {
		if (pdev->id_auto) {
			ida_free(&lkpi_platform_ida, pdev->id);
			pdev->id = PLATFORM_DEVID_AUTO;
			pdev->id_auto = false;
		}
		return (error);
	}

	sx_xlock(&lkpi_platform_lock);
	list_add_tail(&pdev->lkpi_link, &lkpi_platform_devices);
	lkpi_platform_probe_all(pdev, NULL);
	sx_xunlock(&lkpi_platform_lock);
	return (0);
}

void
platform_device_del(struct platform_device *pdev)
{
	sx_xlock(&lkpi_platform_lock);
	if (pdev->dev.driver != NULL)
		lkpi_platform_remove(pdev);
	list_del_init(&pdev->lkpi_link);
	sx_xunlock(&lkpi_platform_lock);

	linux_dma_dev_uninit(&pdev->dev);
	if (pdev->id_auto) {
		ida_free(&lkpi_platform_ida, pdev->id);
		pdev->id = PLATFORM_DEVID_AUTO;
		pdev->id_auto = false;
	}
}

void
platform_device_put(struct platform_device *pdev)
{
	if (pdev != NULL)
		put_device(&pdev->dev);
}

/* Register a device its creator allocated; dev.release must free it. */
int
platform_device_register(struct platform_device *pdev)
{
	lkpi_platform_device_init(pdev);
	return (platform_device_add(pdev));
}

void
platform_device_unregister(struct platform_device *pdev)
{
	platform_device_del(pdev);
	platform_device_put(pdev);
}

struct platform_device *
platform_device_register_full(const struct platform_device_info *info)
{
	struct platform_device *pdev;
	int error;

	pdev = platform_device_alloc(info->name, info->id);
	if (pdev == NULL)
		return (ERR_PTR(-ENOMEM));
	pdev->dev.parent = info->parent;
	error = -ENOMEM;
	if (info->num_res != 0) {
		pdev->resource = kmemdup(info->res,
		    info->num_res * sizeof(*info->res), GFP_KERNEL);
		if (pdev->resource == NULL)
			goto fail;
		pdev->num_resources = info->num_res;
	}
	if (info->data != NULL) {
		pdev->dev.platform_data = kmemdup(info->data, info->size_data,
		    GFP_KERNEL);
		if (pdev->dev.platform_data == NULL)
			goto fail;
	}
	error = platform_device_add(pdev);
	if (error == 0)
		return (pdev);
fail:
	platform_device_put(pdev);
	return (ERR_PTR(error));
}

/* Resources */

static unsigned long
lkpi_resource_type(const struct resource *r)
{
	return (r->flags & (IORESOURCE_MEM | IORESOURCE_IO | IORESOURCE_IRQ));
}

struct resource *
platform_get_resource(struct platform_device *pdev, unsigned int type,
    unsigned int num)
{
	u32 i;

	for (i = 0; i < pdev->num_resources; i++)
		if (lkpi_resource_type(&pdev->resource[i]) == type &&
		    num-- == 0)
			return (&pdev->resource[i]);
	return (NULL);
}

struct resource *
platform_get_resource_byname(struct platform_device *pdev, unsigned int type,
    const char *name)
{
	u32 i;

	for (i = 0; i < pdev->num_resources; i++)
		if (lkpi_resource_type(&pdev->resource[i]) == type &&
		    pdev->resource[i].name != NULL &&
		    strcmp(pdev->resource[i].name, name) == 0)
			return (&pdev->resource[i]);
	return (NULL);
}

/*
 * A platform device's IRQ numbers are those of its FreeBSD device's IRQ
 * resources: the device whose IRQ resource irq is.
 */
struct device *
lkpi_platform_find_irq_dev(unsigned int irq)
{
	struct platform_device *pdev;
	struct device *found = NULL;
	u32 i;

	/* Exclusive: drivers request IRQs from probe(), with it held. */
	sx_xlock(&lkpi_platform_lock);
	list_for_each_entry(pdev, &lkpi_platform_devices, lkpi_link) {
		for (i = 0; i < pdev->num_resources; i++) {
			if (lkpi_resource_type(&pdev->resource[i]) ==
			    IORESOURCE_IRQ && pdev->resource[i].start == irq) {
				found = &pdev->dev;
				goto out;
			}
		}
	}
out:
	sx_xunlock(&lkpi_platform_lock);
	return (found);
}

int
platform_get_irq(struct platform_device *pdev, unsigned int num)
{
	struct resource *r;

	r = platform_get_resource(pdev, IORESOURCE_IRQ, num);
	return (r != NULL ? (int)r->start : -ENXIO);
}

int
platform_get_irq_byname(struct platform_device *pdev, const char *name)
{
	struct resource *r;

	r = platform_get_resource_byname(pdev, IORESOURCE_IRQ, name);
	return (r != NULL ? (int)r->start : -ENXIO);
}

void __iomem *
devm_ioremap_resource(struct device *dev, const struct resource *res)
{
	void __iomem *p;

	if (res == NULL)
		return (IOMEM_ERR_PTR(-EINVAL));
	p = devm_ioremap(dev, res->start, resource_size(res));
	return (p != NULL ? p : IOMEM_ERR_PTR(-ENOMEM));
}

void __iomem *
devm_platform_ioremap_resource(struct platform_device *pdev, unsigned int num)
{
	return (devm_ioremap_resource(&pdev->dev,
	    platform_get_resource(pdev, IORESOURCE_MEM, num)));
}

void __iomem *
devm_platform_ioremap_resource_byname(struct platform_device *pdev,
    const char *name)
{
	return (devm_ioremap_resource(&pdev->dev,
	    platform_get_resource_byname(pdev, IORESOURCE_MEM, name)));
}
