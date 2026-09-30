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
 * The component framework, as in Linux.  A master (an aggregate device,
 * named by its parent device) lists the components it needs; whenever a
 * master or a component is added, masters whose components are all present
 * are bound.  What a master or a component gets with devm_*() while bound
 * is released when it is unbound, and a match list is freed with the
 * parent's devres.
 */

#include <sys/param.h>
#include <sys/systm.h>

#include <linux/component.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/string.h>

struct component;

struct component_match_array {
	void		*data;
	int		(*compare)(struct device *, void *);
	int		(*compare_typed)(struct device *, int, void *);
	void		(*release)(struct device *, void *);
	struct component *component;
	bool		duplicate;
};

struct component_match {
	size_t		alloc;
	size_t		num;
	struct component_match_array *compare;
};

struct aggregate_device {
	struct list_head node;
	bool		bound;
	const struct component_master_ops *ops;
	struct device	*parent;
	struct component_match *match;
};

struct component {
	struct list_head node;
	struct aggregate_device *adev;
	bool		bound;
	const struct component_ops *ops;
	int		subcomponent;
	struct device	*dev;
};

static DEFINE_MUTEX(component_mutex);
static LIST_HEAD(component_list);
static LIST_HEAD(aggregate_devices);

/* Match lists */

static void
devm_component_match_release(struct device *parent, void *res)
{
	struct component_match *match = res;
	size_t i;

	for (i = 0; i < match->num; i++)
		if (match->compare[i].release != NULL)
			match->compare[i].release(parent,
			    match->compare[i].data);
	kfree(match->compare);
}

static void
component_match_add_entry(struct device *parent,
    struct component_match **matchptr,
    void (*release)(struct device *, void *),
    int (*compare)(struct device *, void *),
    int (*compare_typed)(struct device *, int, void *), void *data)
{
	struct component_match *match = *matchptr;
	struct component_match_array *new;
	size_t alloc;

	if (IS_ERR(match))
		return;
	if (match == NULL) {
		match = devres_alloc(devm_component_match_release,
		    sizeof(*match), GFP_KERNEL);
		if (match == NULL) {
			*matchptr = ERR_PTR(-ENOMEM);
			return;
		}
		devres_add(parent, match);
		*matchptr = match;
	}
	if (match->num == match->alloc) {
		alloc = match->alloc + 16;
		new = krealloc(match->compare, alloc * sizeof(*new),
		    GFP_KERNEL);
		if (new == NULL) {
			*matchptr = ERR_PTR(-ENOMEM);
			return;
		}
		match->compare = new;
		match->alloc = alloc;
	}
	match->compare[match->num] = (struct component_match_array){
		.data = data,
		.compare = compare,
		.compare_typed = compare_typed,
		.release = release,
	};
	match->num++;
}

void
component_match_add_release(struct device *parent,
    struct component_match **matchptr,
    void (*release)(struct device *, void *),
    int (*compare)(struct device *, void *), void *compare_data)
{
	component_match_add_entry(parent, matchptr, release, compare, NULL,
	    compare_data);
}

void
component_match_add_typed(struct device *parent,
    struct component_match **matchptr,
    int (*compare_typed)(struct device *, int, void *), void *compare_data)
{
	component_match_add_entry(parent, matchptr, NULL, NULL, compare_typed,
	    compare_data);
}

int
component_compare_dev(struct device *dev, void *data)
{
	return (dev == data);
}

int
component_compare_dev_name(struct device *dev, void *data)
{
	return (strcmp(dev_name(dev), data) == 0);
}

/* Binding */

static struct aggregate_device *
find_aggregate_device(struct device *parent,
    const struct component_master_ops *ops)
{
	struct aggregate_device *adev;

	list_for_each_entry(adev, &aggregate_devices, node)
		if (adev->parent == parent && (ops == NULL || adev->ops == ops))
			return (adev);
	return (NULL);
}

static struct component *
find_component(struct aggregate_device *adev,
    struct component_match_array *mc)
{
	struct component *c;

	list_for_each_entry(c, &component_list, node) {
		if (c->adev != NULL && c->adev != adev)
			continue;
		if (mc->compare != NULL && c->subcomponent == 0 &&
		    mc->compare(c->dev, mc->data))
			return (c);
		if (mc->compare_typed != NULL && c->subcomponent != 0 &&
		    mc->compare_typed(c->dev, c->subcomponent, mc->data))
			return (c);
	}
	return (NULL);
}

/* Find each component the master matches; 0 once it has them all. */
static int
find_components(struct aggregate_device *adev)
{
	struct component_match *match = adev->match;
	struct component *c;
	size_t i, j;

	for (i = 0; i < match->num; i++) {
		if (match->compare[i].component != NULL)
			continue;
		c = find_component(adev, &match->compare[i]);
		if (c == NULL)
			return (-ENXIO);
		/* A component matched twice is bound and unbound once. */
		for (j = 0; j < i; j++)
			if (match->compare[j].component == c)
				match->compare[i].duplicate = true;
		match->compare[i].component = c;
		c->adev = adev;
	}
	return (0);
}

static void
remove_component(struct aggregate_device *adev, struct component *c)
{
	size_t i;

	for (i = 0; i < adev->match->num; i++) {
		if (adev->match->compare[i].component == c) {
			adev->match->compare[i].component = NULL;
			adev->match->compare[i].duplicate = false;
		}
	}
}

/*
 * Bind the master if it has all its components; 1 if it did.  With
 * component set, only if that component is one of them.
 */
static int
try_to_bring_up_aggregate_device(struct aggregate_device *adev,
    struct component *component)
{
	int error;

	if (adev->bound || find_components(adev) != 0)
		return (0);
	if (component != NULL && component->adev != adev)
		return (0);
	if (devres_open_group(adev->parent, adev, GFP_KERNEL) == NULL)
		return (-ENOMEM);
	error = adev->ops->bind(adev->parent);
	if (error != 0) {
		devres_release_group(adev->parent, adev);
		return (error);
	}
	devres_close_group(adev->parent, adev);
	adev->bound = true;
	return (1);
}

static int
try_to_bring_up_masters(struct component *component)
{
	struct aggregate_device *adev;
	int error;

	list_for_each_entry(adev, &aggregate_devices, node) {
		if (adev->bound)
			continue;
		error = try_to_bring_up_aggregate_device(adev, component);
		if (error != 0)
			return (error);
	}
	return (0);
}

static void
take_down_aggregate_device(struct aggregate_device *adev)
{
	if (!adev->bound)
		return;
	adev->ops->unbind(adev->parent);
	devres_release_group(adev->parent, adev);
	adev->bound = false;
}

static void
free_aggregate_device(struct aggregate_device *adev)
{
	struct component *c;
	size_t i;

	list_del(&adev->node);
	for (i = 0; i < adev->match->num; i++) {
		c = adev->match->compare[i].component;
		if (c != NULL)
			c->adev = NULL;
	}
	kfree(adev);
}

/* Masters */

int
component_master_add_with_match(struct device *parent,
    const struct component_master_ops *ops, struct component_match *match)
{
	struct aggregate_device *adev;
	int error;

	if (IS_ERR_OR_NULL(match))
		return (match == NULL ? -EINVAL : PTR_ERR(match));
	adev = kzalloc(sizeof(*adev), GFP_KERNEL);
	if (adev == NULL)
		return (-ENOMEM);
	adev->parent = parent;
	adev->ops = ops;
	adev->match = match;

	mutex_lock(&component_mutex);
	list_add(&adev->node, &aggregate_devices);
	error = try_to_bring_up_aggregate_device(adev, NULL);
	/* As in Linux, a master whose bind fails is not kept. */
	if (error < 0)
		free_aggregate_device(adev);
	mutex_unlock(&component_mutex);
	return (error < 0 ? error : 0);
}

void
component_master_del(struct device *parent,
    const struct component_master_ops *ops)
{
	struct aggregate_device *adev;

	mutex_lock(&component_mutex);
	adev = find_aggregate_device(parent, ops);
	if (adev != NULL) {
		take_down_aggregate_device(adev);
		free_aggregate_device(adev);
	}
	mutex_unlock(&component_mutex);
}

/* Components */

static int
component_bind(struct component *c, struct aggregate_device *adev,
    void *data)
{
	int error;

	if (devres_open_group(c->dev, c, GFP_KERNEL) == NULL)
		return (-ENOMEM);
	error = c->ops->bind(c->dev, adev->parent, data);
	if (error != 0) {
		devres_release_group(c->dev, c);
		if (error != -EPROBE_DEFER)
			dev_err(adev->parent, "failed to bind %s: %d\n",
			    dev_name(c->dev), error);
		return (error);
	}
	devres_close_group(c->dev, c);
	c->bound = true;
	return (0);
}

static void
component_unbind(struct component *c, struct aggregate_device *adev,
    void *data)
{
	if (!c->bound)
		return;
	if (c->ops->unbind != NULL)
		c->ops->unbind(c->dev, adev->parent, data);
	c->bound = false;
	devres_release_group(c->dev, c);
}

/* Called by the master's bind with component_mutex held. */
int
component_bind_all(struct device *parent, void *data)
{
	struct aggregate_device *adev;
	struct component *c;
	size_t i;
	int error;

	adev = find_aggregate_device(parent, NULL);
	if (adev == NULL)
		return (-EINVAL);
	for (i = 0; i < adev->match->num; i++) {
		if (adev->match->compare[i].duplicate)
			continue;
		c = adev->match->compare[i].component;
		error = component_bind(c, adev, data);
		if (error != 0) {
			while (i-- > 0) {
				if (!adev->match->compare[i].duplicate)
					component_unbind(
					    adev->match->compare[i].component,
					    adev, data);
			}
			return (error);
		}
	}
	return (0);
}

/* Called by the master's unbind with component_mutex held. */
void
component_unbind_all(struct device *parent, void *data)
{
	struct aggregate_device *adev;
	struct component *c;
	size_t i;

	adev = find_aggregate_device(parent, NULL);
	if (adev == NULL)
		return;
	for (i = adev->match->num; i-- > 0;) {
		if (adev->match->compare[i].duplicate)
			continue;
		c = adev->match->compare[i].component;
		if (c != NULL)
			component_unbind(c, adev, data);
	}
}

static int
component_add_common(struct device *dev, const struct component_ops *ops,
    int subcomponent)
{
	struct component *c;
	int error;

	c = kzalloc(sizeof(*c), GFP_KERNEL);
	if (c == NULL)
		return (-ENOMEM);
	c->ops = ops;
	c->dev = dev;
	c->subcomponent = subcomponent;

	mutex_lock(&component_mutex);
	list_add_tail(&c->node, &component_list);
	error = try_to_bring_up_masters(c);
	/* As in Linux, a component whose master fails to bind is not kept. */
	if (error < 0) {
		if (c->adev != NULL)
			remove_component(c->adev, c);
		list_del(&c->node);
		kfree(c);
	}
	mutex_unlock(&component_mutex);
	return (error < 0 ? error : 0);
}

int
component_add(struct device *dev, const struct component_ops *ops)
{
	return (component_add_common(dev, ops, 0));
}

int
component_add_typed(struct device *dev, const struct component_ops *ops,
    int subcomponent)
{
	if (subcomponent == 0)
		return (-EINVAL);
	return (component_add_common(dev, ops, subcomponent));
}

void
component_del(struct device *dev, const struct component_ops *ops)
{
	struct component *c;

	mutex_lock(&component_mutex);
	list_for_each_entry(c, &component_list, node) {
		if (c->dev != dev || c->ops != ops)
			continue;
		if (c->adev != NULL) {
			take_down_aggregate_device(c->adev);
			remove_component(c->adev, c);
		}
		list_del(&c->node);
		kfree(c);
		break;
	}
	mutex_unlock(&component_mutex);
}
