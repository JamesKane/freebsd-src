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
 * Runtime power management.  A device's usage count is raised by gets and
 * lowered by puts: the first get resumes the device through its driver's
 * runtime_resume, and the last put suspends it through runtime_suspend, at
 * once or, with autosuspend, once it has been idle for the autosuspend
 * delay.  Everything is synchronous except that deferred suspend.
 *
 * A device whose runtime PM is disabled is only counted, and is not
 * suspended.  PCI devices are always active: LinuxKPI does not put PCI
 * functions into low-power states, so their drivers' runtime PM callbacks
 * are never called, and the calls answer as for a device in use.
 */

#include <sys/param.h>
#include <sys/systm.h>

#include <linux/device.h>
#include <linux/jiffies.h>
#include <linux/mutex.h>
#include <linux/pci.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

struct lkpi_rpm {
	struct device	*dev;
	struct mutex	lock;		/* serializes callbacks and state */
	int		usage;
	int		disable_depth;	/* > 0: disabled */
	bool		active;
	bool		runtime_auto;	/* false after pm_runtime_forbid() */
	bool		no_callbacks;
	bool		autosuspend;
	bool		needs_force_resume;
	int		delay_ms;	/* autosuspend delay */
	unsigned long	last_busy;	/* jiffies */
	struct delayed_work suspend_work;
};

static DEFINE_MUTEX(lkpi_rpm_alloc_lock);

static void lkpi_rpm_suspend_work(struct work_struct *work);

/*
 * The device's runtime PM state, locked, made on first use; NULL for a PCI
 * device, or with no memory, when calls answer as for a device in use.
 */
static struct lkpi_rpm *
lkpi_rpm_get(struct device *dev)
{
	struct lkpi_rpm *r;

	if (dev_is_pci(dev))
		return (NULL);
	r = dev->power.lkpi_rpm;
	if (r == NULL) {
		mutex_lock(&lkpi_rpm_alloc_lock);
		r = dev->power.lkpi_rpm;
		if (r == NULL) {
			r = kzalloc(sizeof(*r), GFP_KERNEL);
			if (r == NULL) {
				mutex_unlock(&lkpi_rpm_alloc_lock);
				return (NULL);
			}
			r->dev = dev;
			r->disable_depth = 1;	/* as in Linux */
			r->runtime_auto = true;
			r->last_busy = jiffies;
			mutex_init(&r->lock);
			INIT_DELAYED_WORK(&r->suspend_work,
			    lkpi_rpm_suspend_work);
			dev->power.lkpi_rpm = r;
		}
		mutex_unlock(&lkpi_rpm_alloc_lock);
	}
	mutex_lock(&r->lock);
	return (r);
}

void
lkpi_pm_runtime_release(struct device *dev)
{
	struct lkpi_rpm *r = dev->power.lkpi_rpm;

	if (r == NULL)
		return;
	cancel_delayed_work_sync(&r->suspend_work);
	mutex_destroy(&r->lock);
	kfree(r);
	dev->power.lkpi_rpm = NULL;
}

static const struct dev_pm_ops *
lkpi_rpm_ops(struct lkpi_rpm *r)
{
	if (r->no_callbacks || r->dev->driver == NULL)
		return (NULL);
	return (r->dev->driver->pm);
}

static int
lkpi_rpm_resume(struct lkpi_rpm *r)
{
	const struct dev_pm_ops *ops = lkpi_rpm_ops(r);
	int error;

	if (r->active)
		return (0);
	if (r->disable_depth == 0 && ops != NULL &&
	    ops->runtime_resume != NULL) {
		error = ops->runtime_resume(r->dev);
		if (error != 0)
			return (error);
	}
	r->active = true;
	return (0);
}

static void
lkpi_rpm_suspend(struct lkpi_rpm *r)
{
	const struct dev_pm_ops *ops = lkpi_rpm_ops(r);

	if (!r->active || r->disable_depth > 0 || !r->runtime_auto)
		return;
	/* As in Linux, a device whose suspend fails stays active. */
	if (ops != NULL && ops->runtime_suspend != NULL &&
	    ops->runtime_suspend(r->dev) != 0)
		return;
	r->active = false;
}

/*
 * The device has become idle.  Suspend it, unless it uses autosuspend and
 * now is false: then suspend it once it has been idle for the delay, or,
 * with a negative delay, not at all.
 */
static void
lkpi_rpm_idle(struct lkpi_rpm *r, bool now)
{
	long left;

	if (!r->active)
		return;
	if (now || !r->autosuspend) {
		lkpi_rpm_suspend(r);
		return;
	}
	if (r->delay_ms < 0)
		return;
	left = (long)(r->last_busy + msecs_to_jiffies(r->delay_ms) - jiffies);
	if (left > 0)
		mod_delayed_work(system_wq, &r->suspend_work, left);
	else
		lkpi_rpm_suspend(r);
}

static void
lkpi_rpm_suspend_work(struct work_struct *work)
{
	struct lkpi_rpm *r = container_of(to_delayed_work(work),
	    struct lkpi_rpm, suspend_work);

	mutex_lock(&r->lock);
	/* A get may have come in; a mark_last_busy() moves the deadline. */
	if (r->usage == 0)
		lkpi_rpm_idle(r, false);
	mutex_unlock(&r->lock);
}

/* Gets and puts */

int
pm_runtime_get_sync(struct device *dev)
{
	struct lkpi_rpm *r;
	int error;

	if ((r = lkpi_rpm_get(dev)) == NULL)
		return (0);
	/* The work rechecks the count under the lock we hold. */
	cancel_delayed_work(&r->suspend_work);
	r->usage++;
	/* Also retries a resume that failed for an earlier get. */
	error = lkpi_rpm_resume(r);
	mutex_unlock(&r->lock);
	return (error);		/* the count stays up, as in Linux */
}

int
pm_runtime_resume_and_get(struct device *dev)
{
	int error;

	error = pm_runtime_get_sync(dev);
	if (error < 0) {
		pm_runtime_put_noidle(dev);
		return (error);
	}
	return (0);
}

void
pm_runtime_get_noresume(struct device *dev)
{
	struct lkpi_rpm *r;

	if ((r = lkpi_rpm_get(dev)) == NULL)
		return;
	r->usage++;
	mutex_unlock(&r->lock);
}

int
pm_runtime_get_if_in_use(struct device *dev)
{
	struct lkpi_rpm *r;
	int ret;

	if ((r = lkpi_rpm_get(dev)) == NULL)
		return (1);
	ret = r->active && r->usage > 0;
	if (ret)
		r->usage++;
	mutex_unlock(&r->lock);
	return (ret);
}

int
lkpi_pm_runtime_get_if_active(struct device *dev)
{
	struct lkpi_rpm *r;
	int ret;

	if ((r = lkpi_rpm_get(dev)) == NULL)
		return (1);
	ret = r->active;
	if (ret)
		r->usage++;
	mutex_unlock(&r->lock);
	return (ret);
}

static int
lkpi_rpm_put(struct device *dev, bool idle, bool now)
{
	struct lkpi_rpm *r;

	if ((r = lkpi_rpm_get(dev)) == NULL)
		return (0);
	if (r->usage > 0 && --r->usage == 0 && idle)
		lkpi_rpm_idle(r, now);
	mutex_unlock(&r->lock);
	return (0);
}

int
pm_runtime_put_sync(struct device *dev)
{
	return (lkpi_rpm_put(dev, true, false));
}

int
pm_runtime_put_sync_suspend(struct device *dev)
{
	return (lkpi_rpm_put(dev, true, true));
}

int
pm_runtime_put_autosuspend(struct device *dev)
{
	return (lkpi_rpm_put(dev, true, false));
}

void
pm_runtime_put_noidle(struct device *dev)
{
	(void)lkpi_rpm_put(dev, false, false);
}

/* Resume or suspend without changing the count. */

int
pm_runtime_resume(struct device *dev)
{
	struct lkpi_rpm *r;
	int error;

	if ((r = lkpi_rpm_get(dev)) == NULL)
		return (0);
	error = lkpi_rpm_resume(r);
	mutex_unlock(&r->lock);
	return (error);
}

static int
lkpi_rpm_suspend_idle(struct device *dev, bool now)
{
	struct lkpi_rpm *r;

	if ((r = lkpi_rpm_get(dev)) == NULL)
		return (0);
	if (r->usage == 0)
		lkpi_rpm_idle(r, now);
	mutex_unlock(&r->lock);
	return (0);
}

int
pm_runtime_suspend(struct device *dev)
{
	return (lkpi_rpm_suspend_idle(dev, true));
}

int
pm_runtime_autosuspend(struct device *dev)
{
	return (lkpi_rpm_suspend_idle(dev, false));
}

/* Autosuspend */

void
pm_runtime_mark_last_busy(struct device *dev)
{
	struct lkpi_rpm *r;

	if ((r = lkpi_rpm_get(dev)) == NULL)
		return;
	r->last_busy = jiffies;
	mutex_unlock(&r->lock);
}

void
pm_runtime_use_autosuspend(struct device *dev)
{
	struct lkpi_rpm *r;

	if ((r = lkpi_rpm_get(dev)) == NULL)
		return;
	r->autosuspend = true;
	mutex_unlock(&r->lock);
}

/* As in Linux, an idle device suspends at once when autosuspend stops. */
void
pm_runtime_dont_use_autosuspend(struct device *dev)
{
	struct lkpi_rpm *r;

	if ((r = lkpi_rpm_get(dev)) == NULL)
		return;
	r->autosuspend = false;
	cancel_delayed_work(&r->suspend_work);
	if (r->usage == 0)
		lkpi_rpm_idle(r, true);
	mutex_unlock(&r->lock);
}

void
pm_runtime_set_autosuspend_delay(struct device *dev, int ms)
{
	struct lkpi_rpm *r;

	if ((r = lkpi_rpm_get(dev)) == NULL)
		return;
	r->delay_ms = ms;
	mutex_unlock(&r->lock);
}

/* When the device may autosuspend, in ns since boot, or 0. */
u64
pm_runtime_autosuspend_expiration(struct device *dev)
{
	struct lkpi_rpm *r;
	u64 ns = 0;

	if ((r = lkpi_rpm_get(dev)) == NULL)
		return (0);
	if (r->autosuspend && r->delay_ms >= 0)
		ns = (u64)jiffies_to_usecs(r->last_busy +
		    msecs_to_jiffies(r->delay_ms)) * NSEC_PER_USEC;
	mutex_unlock(&r->lock);
	return (ns);
}

/* State */

void
pm_runtime_enable(struct device *dev)
{
	struct lkpi_rpm *r;

	if ((r = lkpi_rpm_get(dev)) == NULL)
		return;
	if (r->disable_depth > 0)
		r->disable_depth--;
	mutex_unlock(&r->lock);
}

void
pm_runtime_disable(struct device *dev)
{
	struct lkpi_rpm *r;

	if ((r = lkpi_rpm_get(dev)) == NULL)
		return;
	r->disable_depth++;
	cancel_delayed_work(&r->suspend_work);
	mutex_unlock(&r->lock);
}

bool
pm_runtime_enabled(struct device *dev)
{
	struct lkpi_rpm *r;
	bool enabled;

	if ((r = lkpi_rpm_get(dev)) == NULL)
		return (false);
	enabled = r->disable_depth == 0;
	mutex_unlock(&r->lock);
	return (enabled);
}

/* Allow or forbid runtime suspend, as user space does in Linux. */
void
pm_runtime_allow(struct device *dev)
{
	struct lkpi_rpm *r;

	if ((r = lkpi_rpm_get(dev)) == NULL)
		return;
	if (!r->runtime_auto) {
		r->runtime_auto = true;
		if (r->usage > 0 && --r->usage == 0)
			lkpi_rpm_idle(r, false);
	}
	mutex_unlock(&r->lock);
}

void
pm_runtime_forbid(struct device *dev)
{
	struct lkpi_rpm *r;

	if ((r = lkpi_rpm_get(dev)) == NULL)
		return;
	if (r->runtime_auto) {
		r->runtime_auto = false;
		r->usage++;
		(void)lkpi_rpm_resume(r);
	}
	mutex_unlock(&r->lock);
}

void
pm_runtime_no_callbacks(struct device *dev)
{
	struct lkpi_rpm *r;

	if ((r = lkpi_rpm_get(dev)) == NULL)
		return;
	r->no_callbacks = true;
	mutex_unlock(&r->lock);
}

int
pm_runtime_set_active(struct device *dev)
{
	struct lkpi_rpm *r;

	if ((r = lkpi_rpm_get(dev)) == NULL)
		return (0);
	r->active = true;
	mutex_unlock(&r->lock);
	return (0);
}

void
pm_runtime_set_suspended(struct device *dev)
{
	struct lkpi_rpm *r;

	if ((r = lkpi_rpm_get(dev)) == NULL)
		return;
	r->active = false;
	mutex_unlock(&r->lock);
}

bool
pm_runtime_active(struct device *dev)
{
	struct lkpi_rpm *r;
	bool active;

	if ((r = lkpi_rpm_get(dev)) == NULL)
		return (true);
	active = r->active || r->disable_depth > 0;
	mutex_unlock(&r->lock);
	return (active);
}

bool
pm_runtime_suspended(struct device *dev)
{
	struct lkpi_rpm *r;
	bool suspended;

	if ((r = lkpi_rpm_get(dev)) == NULL)
		return (false);
	suspended = !r->active && r->disable_depth == 0;
	mutex_unlock(&r->lock);
	return (suspended);
}

bool
pm_runtime_status_suspended(struct device *dev)
{
	struct lkpi_rpm *r;
	bool suspended;

	if ((r = lkpi_rpm_get(dev)) == NULL)
		return (false);
	suspended = !r->active;
	mutex_unlock(&r->lock);
	return (suspended);
}

/*
 * System sleep, and drivers taking a device down: as in Linux, disable
 * runtime PM and suspend the device if it is active, and on resume, power
 * it again if it was in use.
 */
int
pm_runtime_force_suspend(struct device *dev)
{
	const struct dev_pm_ops *ops;
	struct lkpi_rpm *r;
	int error;

	if ((r = lkpi_rpm_get(dev)) == NULL)
		return (0);
	r->disable_depth++;
	cancel_delayed_work(&r->suspend_work);
	if (r->active) {
		ops = lkpi_rpm_ops(r);
		if (ops != NULL && ops->runtime_suspend != NULL) {
			error = ops->runtime_suspend(dev);
			if (error != 0) {
				r->disable_depth--;
				mutex_unlock(&r->lock);
				return (error);
			}
		}
		r->active = false;
		r->needs_force_resume = r->usage > 0;
	}
	mutex_unlock(&r->lock);
	return (0);
}

int
pm_runtime_force_resume(struct device *dev)
{
	const struct dev_pm_ops *ops;
	struct lkpi_rpm *r;
	int error = 0;

	if ((r = lkpi_rpm_get(dev)) == NULL)
		return (0);
	if (!r->active && r->needs_force_resume) {
		ops = lkpi_rpm_ops(r);
		if (ops != NULL && ops->runtime_resume != NULL)
			error = ops->runtime_resume(dev);
		if (error == 0) {
			r->active = true;
			r->needs_force_resume = false;
		}
	}
	if (r->disable_depth > 0)
		r->disable_depth--;
	mutex_unlock(&r->lock);
	return (error);
}
