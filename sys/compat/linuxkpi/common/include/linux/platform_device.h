/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2020-2022 Bjoern A. Zeeb
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

#ifndef	_LINUXKPI_LINUX_PLATFORM_DEVICE_H
#define	_LINUXKPI_LINUX_PLATFORM_DEVICE_H

#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/ioport.h>
#include <linux/list.h>
#include <linux/mod_devicetable.h>

#define	PLATFORM_DEVID_NONE	(-1)
#define	PLATFORM_DEVID_AUTO	(-2)

/*
 * Devices on no bus of their own, as in Linux.  LinuxKPI matches them to
 * drivers by id_table or by name; drivers that match only by device tree
 * need the device to have the driver's name.  Each stands for a FreeBSD
 * device: the one its creator sets in dev.bsddev, else its parent's.
 */
struct platform_device {
	const char		*name;
	int			id;
	bool			id_auto;
	struct device		dev;
	u32			num_resources;
	struct resource		*resource;
	const struct platform_device_id *id_entry;

	struct list_head	lkpi_link;	/* LinuxKPI private */
};

struct platform_device_info {
	struct device		*parent;
	const char		*name;
	int			id;
	const struct resource	*res;
	unsigned int		num_res;
	const void		*data;
	size_t			size_data;
	u64			dma_mask;
};

struct platform_driver {
	int	(*probe)(struct platform_device *);
	void	(*remove)(struct platform_device *);
	void	(*shutdown)(struct platform_device *);
	struct device_driver driver;
	const struct platform_device_id *id_table;

	struct list_head	lkpi_link;	/* LinuxKPI private */
};

extern const struct bus_type platform_bus_type;

#define	to_platform_device(d)	container_of((d), struct platform_device, dev)
#define	to_platform_driver(d)	container_of((d), struct platform_driver, driver)

static inline bool
dev_is_platform(const struct device *dev)
{
	return (dev != NULL && dev->bus == &platform_bus_type);
}

int	platform_driver_register(struct platform_driver *pdrv);
void	platform_driver_unregister(struct platform_driver *pdrv);
int	platform_driver_probe(struct platform_driver *pdrv,
	    int (*probe)(struct platform_device *));

struct platform_device *platform_device_alloc(const char *name, int id);
int	platform_device_add(struct platform_device *pdev);
void	platform_device_del(struct platform_device *pdev);
void	platform_device_put(struct platform_device *pdev);
int	platform_device_register(struct platform_device *pdev);
void	platform_device_unregister(struct platform_device *pdev);
struct platform_device *platform_device_register_full(
	    const struct platform_device_info *info);

struct resource *platform_get_resource(struct platform_device *pdev,
	    unsigned int type, unsigned int num);
struct resource *platform_get_resource_byname(struct platform_device *pdev,
	    unsigned int type, const char *name);
int	platform_get_irq(struct platform_device *pdev, unsigned int num);
int	platform_get_irq_byname(struct platform_device *pdev,
	    const char *name);
void __iomem *devm_platform_ioremap_resource(struct platform_device *pdev,
	    unsigned int num);
void __iomem *devm_platform_ioremap_resource_byname(
	    struct platform_device *pdev, const char *name);

static inline struct platform_device *
platform_device_register_simple(const char *name, int id,
    const struct resource *res, unsigned int num)
{
	struct platform_device_info info = {
		.name = name,
		.id = id,
		.res = res,
		.num_res = num,
	};

	return (platform_device_register_full(&info));
}

static inline void *
platform_get_drvdata(const struct platform_device *pdev)
{
	return (dev_get_drvdata(&pdev->dev));
}

static inline void
platform_set_drvdata(struct platform_device *pdev, void *data)
{
	dev_set_drvdata(&pdev->dev, data);
}

#define	module_platform_driver(_drv)					\
static int __init							\
_drv##_init(void)							\
{									\
	return (platform_driver_register(&(_drv)));			\
}									\
module_init(_drv##_init);						\
static void __exit							\
_drv##_exit(void)							\
{									\
	platform_driver_unregister(&(_drv));				\
}									\
module_exit(_drv##_exit)

#endif	/* _LINUXKPI_LINUX_PLATFORM_DEVICE_H */
