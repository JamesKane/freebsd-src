/*
 * Copyright (c) 2025 The FreeBSD Foundation
 *
 * SPDX-License-Identifier: BSD-2-Clause
 */

#ifndef	_LINUXKPI_LINUX_STRING_HELPERS_H_
#define	_LINUXKPI_LINUX_STRING_HELPERS_H_

#include <linux/string_choices.h>
#include <linux/sched.h>
#include <linux/string.h>

/* Linux quotes the task's command line; FreeBSD's task has its name. */
static inline char *
kstrdup_quotable_cmdline(struct task_struct *task, gfp_t gfp)
{
	return (kstrdup(task->comm, gfp));
}

#endif
