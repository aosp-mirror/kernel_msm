/*
 *  servicefs.h - a small filesystem for service namespaces.
 *
 *  Copyright (C) 2015 Corey Tabaka <eieio@google.com>
 *  Copyright (C) 2015 Google, Inc.
 *
 *  Based on debugfs:
 *  Copyright (C) 2004 Greg Kroah-Hartman <greg@kroah.com>
 *  Copyright (C) 2004 IBM Inc.
 *
 *	This program is free software; you can redistribute it and/or
 *	modify it under the terms of the GNU General Public License version
 *	2 as published by the Free Software Foundation.
 *
 */

#ifndef _SERVICEFS_H_
#define _SERVICEFS_H_

#include <linux/fs.h>

#include <linux/types.h>

struct file_operations;

#if defined(CONFIG_SERVICE_FS)

/* declared over in file.c */
extern const struct file_operations servicefs_file_operations;
extern const struct inode_operations servicefs_link_operations;

bool servicefs_initialized(void);

#else

#include <linux/err.h>

static inline bool servicefs_initialized(void)
{
	return false;
}

#endif

#endif
