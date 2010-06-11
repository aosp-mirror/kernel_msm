/* Copyright (c) 2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include "video_core_type.h"
#include "vid_frame_scheduler_utils.h"

/**
 * SCHED_ASSERT () - This function is a wrapper to underlying ASSERT
 * @val: value to be checked for
 * function.
 * DEPENDENCIES: None
 * Returns none
 */
SCHED_INLINE void SCHED_ASSERT(int val)
{

}				/* end of SCHED_ASSERT */

/**
 * SCHED_MIN () - This function will find minimum of two values
 * @x: value 1
 * @y: value 2
 * DEPENDENCIES: None
 * Returns none
 */
SCHED_INLINE int SCHED_MIN(int x, int y)
{
	if (x < y)
		return x;
	else
		return y;

}				/* end of SCHED_MIN */

/**
 * SCHED_MALLOC () - This function is a wrapper to underlying malloc
 * @size: memory size to be allocated
 * function
 * DEPENDENCIES: None
 * Returns none
 */
SCHED_INLINE void *SCHED_MALLOC(int size)
{
	return kmalloc(size, GFP_KERNEL);
}				/* end of SCHED_MALLOC */

/**
 * SCHED_FREE () - This function is a wrapper to underlying memory free
 * @ptr: memory to be freed
 * function
 * DEPENDENCIES: None
 * Returns none
 */
SCHED_INLINE void SCHED_FREE(void *ptr)
{
	kfree(ptr);
}				/* end of SCHED_FREE */

/**
 * SCHED_MEMSET () - This function is a wrapper to underlying memory set
 * @ptr: ptr to memory
 * @val: value to be set
 * @size: memory size to be set
 * function
 * DEPENDENCIES: None
 * Returns none
 */
SCHED_INLINE void *SCHED_MEMSET(void *ptr, int val, int size)
{
	return memset(ptr, val, size);
}				/* end of SCHED_MEMSET */

/**
 * SCHED_GET_CURRENT_TIME () - This function is a wrapper to underlying get time
 * @pn_time: ptr time value in milliseconds
 * function
 * DEPENDENCIES: None
 * Returns SCHED_S_OK on success
 */
SCHED_INLINE enum sched_status SCHED_GET_CURRENT_TIME(u32 *pn_time)
{
	struct timeval tv;
	do_gettimeofday(&tv);
	*pn_time = (tv.tv_sec * 1000) + (tv.tv_usec / 1000);
	return SCHED_S_OK;

}				/* end of SCHED_GET_CURRENT_TIME */

/**
 * SCHED_CRITSEC_CREATE () - This function is a wrapper to creating a critical
 * @cs: ptr to a critical section type
 * section
 * DEPENDENCIES: None
 * Returns SCHED_S_OK on success
 */
SCHED_INLINE enum sched_status SCHED_CRITSEC_CREATE(u32 **cs)
{
	return SCHED_S_OK;

}				/* end of SCHED_CRITSEC_CREATE */

/**
 * SCHED_CRITSEC_RELEASE () - This function is a wrapper to releasing a critical
 * @cs: critical section handle type
 * section resource
 * DEPENDENCIES: None
 * Returns SCHED_S_OK on success
 */
SCHED_INLINE enum sched_status SCHED_CRITSEC_RELEASE(u32 *cs)
{
	return SCHED_S_OK;

}				/* end of SCHED_CRITSEC_RELEASE */

/**
 * SCHED_CRITSEC_ENTER () - This function is a wrapper to enter a critical
 * @cs: critical section handle type
 * section
 * DEPENDENCIES: None
 * Returns SCHED_S_OK on success
 */
SCHED_INLINE enum sched_status SCHED_CRITSEC_ENTER(u32 *cs)
{
	return SCHED_S_OK;

}				/* end of SCHED_CRITSEC_ENTER */

/**
 * SCHED_CRITSEC_LEAVE () - This function is a wrapper to leave a critical
 * @cs: critical section handle type
 * section
 * DEPENDENCIES: None
 * Returns SCHED_S_OK on success
 */
SCHED_INLINE enum sched_status SCHED_CRITSEC_LEAVE(u32 *cs)
{
	return SCHED_S_OK;

}				/* end of SCHED_CRITSEC_LEAVE */
