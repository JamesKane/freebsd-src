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
 * Runtime power management (linux_pm_runtime.c): the first get resumes a
 * device through its driver's runtime_resume and the last put suspends it,
 * at once or, with autosuspend, once it has been idle for the delay.  Calls
 * are synchronous except autosuspend.  PCI devices are always active:
 * LinuxKPI does not put PCI functions into low-power states.
 */
#ifndef _LINUXKPI_LINUX_PM_RUNTIME_H_
#define	_LINUXKPI_LINUX_PM_RUNTIME_H_

#include <linux/device.h>
#include <linux/pm.h>

int	pm_runtime_get_sync(struct device *dev);
int	pm_runtime_resume_and_get(struct device *dev);
void	pm_runtime_get_noresume(struct device *dev);
int	pm_runtime_get_if_in_use(struct device *dev);
int	lkpi_pm_runtime_get_if_active(struct device *dev);
int	pm_runtime_put_sync(struct device *dev);
int	pm_runtime_put_sync_suspend(struct device *dev);
int	pm_runtime_put_autosuspend(struct device *dev);
void	pm_runtime_put_noidle(struct device *dev);
int	pm_runtime_resume(struct device *dev);
int	pm_runtime_suspend(struct device *dev);
int	pm_runtime_autosuspend(struct device *dev);

void	pm_runtime_mark_last_busy(struct device *dev);
void	pm_runtime_use_autosuspend(struct device *dev);
void	pm_runtime_dont_use_autosuspend(struct device *dev);
void	pm_runtime_set_autosuspend_delay(struct device *dev, int ms);
u64	pm_runtime_autosuspend_expiration(struct device *dev);

void	pm_runtime_enable(struct device *dev);
void	pm_runtime_disable(struct device *dev);
bool	pm_runtime_enabled(struct device *dev);
void	pm_runtime_allow(struct device *dev);
void	pm_runtime_forbid(struct device *dev);
void	pm_runtime_no_callbacks(struct device *dev);
int	pm_runtime_set_active(struct device *dev);
void	pm_runtime_set_suspended(struct device *dev);
bool	pm_runtime_active(struct device *dev);
bool	pm_runtime_suspended(struct device *dev);
bool	pm_runtime_status_suspended(struct device *dev);

int	pm_runtime_force_suspend(struct device *dev);
int	pm_runtime_force_resume(struct device *dev);

/* Free the device's runtime PM state; from its release. */
void	lkpi_pm_runtime_release(struct device *dev);

#if defined(LINUXKPI_VERSION) && LINUXKPI_VERSION < 60900
#define	pm_runtime_get_if_active(dev, all)				\
	lkpi_pm_runtime_get_if_active(dev)
#else
#define	pm_runtime_get_if_active(dev)	lkpi_pm_runtime_get_if_active(dev)
#endif

/* Asynchronous calls are synchronous; autosuspend is still deferred. */
#define	pm_runtime_get(dev)		pm_runtime_get_sync(dev)
#define	pm_runtime_put(dev)		pm_runtime_put_sync(dev)
#define	pm_runtime_put_sync_autosuspend(dev)				\
	pm_runtime_put_autosuspend(dev)
#define	pm_runtime_idle(dev)		pm_runtime_suspend(dev)

#endif	/* _LINUXKPI_LINUX_PM_RUNTIME_H_ */
