/*
 * Copyright 2019 Google, Inc
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#ifndef __LOGBUFFER_H_
#define __LOGBUFFER_H_

struct logbuffer;
void logbuffer_log(struct logbuffer *instance, const char *fmt, ...);

/*
 * Registers a new log buffer entry.
 * param name: name of the file in the /d/logbuffer/ directory.
 * returns the pointer to the logbuffer metadata.
 */
struct logbuffer *debugfs_logbuffer_register(char *name);

void debugfs_logbuffer_unregister(struct logbuffer *instance);
#endif /* __LOGBUFFER_H_ */
