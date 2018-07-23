/* Copyright (C) 2017 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include "ds_ioctl.h"
#include "linux_ds_ioctl.h"
#include "ds_constants.h"
#include "ds_generic.h"
#include "ds_interrupt.h"
#include "ds_logging.h"
#include "ds_page_table.h"
#include <linux/fs.h>
#include <linux/uaccess.h>

#ifdef DS_KERNEL_TRACE_SUPPORT
#define CREATE_TRACE_POINTS
#include <trace/events/ds_ioctl.h>
#else
#define trace_ds_ioctl_entry(x, ...)
#define trace_ds_ioctl_exit(x)
#define trace_ds_ioctl_integer_data(x)
#define trace_ds_ioctl_eventfd_data(x, ...)
#define trace_ds_ioctl_page_table_data(x, ...)
#define trace_ds_ioctl_config_coherent_allocator(x, ...)
#endif

static long ds_ioctl_copy_to_userspace(ulong arg, u64 value);
static uint ds_ioctl_check_permissions(struct file *filp, uint cmd);
static int ds_set_event_fd(struct ds_dev *dev, ulong arg);
static int ds_read_page_table_size(struct ds_dev *ds_dev, ulong arg);
static int ds_read_simple_page_table_size(struct ds_dev *ds_dev, ulong arg);
static int ds_partition_page_table(struct ds_dev *ds_dev, ulong arg);
static int ds_map_buffers(struct ds_dev *ds_dev, ulong arg);
static int ds_unmap_buffers(struct ds_dev *ds_dev, ulong arg);
static int ds_config_coherent_allocator(struct ds_dev *ds_dev, ulong arg);

/*
 * ds_handle_ioctl: standard ioctl dispatch function.
 * @filp: File structure pointer describing this node usage session.
 * @cmd: ioctl number to handle.
 * @arg: ioctl-specific data pointer.
 *
 * Standard ioctl dispatcher; forwards operations to individual handlers.
 */
long ds_handle_ioctl(struct file *filp, uint cmd, ulong arg)
{
	struct ds_dev *ds_dev;
	int retval;

	ds_dev = (struct ds_dev *)filp->private_data;
	trace_ds_ioctl_entry(ds_dev->dev_info.name, cmd);

	if (!ds_ioctl_check_permissions(filp, cmd)) {
		trace_ds_ioctl_exit(-EPERM);
		ds_log_error(ds_dev, "ioctl cmd=%x noperm.", cmd);
		return -EPERM;
	}

	/* Tracing happens in this switch statement for all ioctls with
	 * an integer argrument, but ioctls with a struct argument
	 * that needs copying and decoding, that tracing is done within
	 * the handler call.
	 */
	switch (cmd) {
	case DS_IOCTL_RESET:
		trace_ds_ioctl_integer_data(arg);
		retval = ds_reset(ds_dev, arg);
		break;
	case DS_IOCTL_SET_EVENTFD:
		retval = ds_set_event_fd(ds_dev, arg);
		break;
	case DS_IOCTL_CLEAR_EVENTFD:
		trace_ds_ioctl_integer_data(arg);
		retval = ds_interrupt_clear_eventfd(
			ds_dev->interrupt_data, (int)arg);
		break;
	case DS_IOCTL_PARTITION_PAGE_TABLE:
		trace_ds_ioctl_integer_data(arg);
		retval = ds_partition_page_table(ds_dev, arg);
		break;
	case DS_IOCTL_NUMBER_PAGE_TABLES:
		trace_ds_ioctl_integer_data(ds_dev->num_page_tables);
		retval = ds_ioctl_copy_to_userspace(
			arg, ds_dev->num_page_tables);
		break;
	case DS_IOCTL_PAGE_TABLE_SIZE:
		retval = ds_read_page_table_size(ds_dev, arg);
		break;
	case DS_IOCTL_SIMPLE_PAGE_TABLE_SIZE:
		retval = ds_read_simple_page_table_size(ds_dev, arg);
		break;
	case DS_IOCTL_MAP_BUFFER:
		retval = ds_map_buffers(ds_dev, arg);
		break;
	case DS_IOCTL_CONFIG_COHERENT_ALLOCATOR:
		retval = ds_config_coherent_allocator(ds_dev, arg);
		break;
	case DS_IOCTL_UNMAP_BUFFER:
		retval = ds_unmap_buffers(ds_dev, arg);
		break;
	case DS_IOCTL_CLEAR_INTERRUPT_COUNTS:
		/* Clear interrupt counts doesn't take an integer arg, so use 0.
		 */
		trace_ds_ioctl_integer_data(0);
		retval = ds_interrupt_reset_counts(ds_dev);
		break;
	default:
		/* If we don't understand the ioctl, the best we can do is trace
		 * the arg.
		 */
		trace_ds_ioctl_integer_data(arg);
		ds_log_warn(ds_dev,
			"Unknown ioctl cmd=0x%x not caught by ds_is_supported_ioctl",
			cmd);
		retval = -EINVAL;
		break;
	}

	trace_ds_ioctl_exit(retval);
	return retval;
}

/*
 * ds_is_supported_ioctl: Determines if an ioctl is part of the standard
 * DS framework.
 * @cmd: The ioctl number to handle.
 *
 * Returns 1 if the ioctl is supported and 0 otherwise.
 */
long ds_is_supported_ioctl(uint cmd)
{
	switch (cmd) {
	case DS_IOCTL_RESET:
	case DS_IOCTL_SET_EVENTFD:
	case DS_IOCTL_CLEAR_EVENTFD:
	case DS_IOCTL_PARTITION_PAGE_TABLE:
	case DS_IOCTL_NUMBER_PAGE_TABLES:
	case DS_IOCTL_PAGE_TABLE_SIZE:
	case DS_IOCTL_SIMPLE_PAGE_TABLE_SIZE:
	case DS_IOCTL_MAP_BUFFER:
	case DS_IOCTL_UNMAP_BUFFER:
	case DS_IOCTL_CLEAR_INTERRUPT_COUNTS:
	case DS_IOCTL_CONFIG_COHERENT_ALLOCATOR:
		return 1;
	default:
		return 0;
	}
}

/*
 * ds_ioctl_check_permissions: permission checker for DS ioctls.
 * @filp: File structure pointer describing this node usage session.
 * @cmd: ioctl number to handle.
 *
 * Standard permissions checker.
 */
static uint ds_ioctl_check_permissions(struct file *filp, uint cmd)
{
	uint alive, root, device_owner;
	fmode_t read, write;
	struct ds_dev *ds_dev = (struct ds_dev *)filp->private_data;

	alive = (ds_dev->status == DS_STATUS_ALIVE);
	if (!alive) {
		ds_nodev_error("ds_ioctl_check_permissions alive %d "
			       "status %d.",
			alive, ds_dev->status);
	}

	root = capable(CAP_SYS_ADMIN);
	read = filp->f_mode & FMODE_READ;
	write = filp->f_mode & FMODE_WRITE;
	device_owner = (ds_dev->dev_info.ownership.is_owned &&
			current->tgid == ds_dev->dev_info.ownership.owner);

	switch (cmd) {
	case DS_IOCTL_RESET:
	case DS_IOCTL_CLEAR_INTERRUPT_COUNTS:
		return root || (write && device_owner);

	case DS_IOCTL_PAGE_TABLE_SIZE:
	case DS_IOCTL_SIMPLE_PAGE_TABLE_SIZE:
	case DS_IOCTL_NUMBER_PAGE_TABLES:
		return root || read;

	case DS_IOCTL_PARTITION_PAGE_TABLE:
	case DS_IOCTL_CONFIG_COHERENT_ALLOCATOR:
		return alive && (root || (write && device_owner));

	case DS_IOCTL_MAP_BUFFER:
	case DS_IOCTL_UNMAP_BUFFER:
		return alive && (root || (write && device_owner));

	case DS_IOCTL_CLEAR_EVENTFD:
	case DS_IOCTL_SET_EVENTFD:
		return alive && (root || (write && device_owner));
	}

	return 0; /* unknown permissions */
}

/*
 * ds_ioctl_copy_to_userspace: Copy a value back to userspace.
 * @arg: Pointer to a u64 in userspace.
 * @value: Value to copy.
 */
static long ds_ioctl_copy_to_userspace(ulong arg, u64 value)
{
	if (copy_to_user((void __user *)arg, &value, sizeof(uint64_t)))
		return -EFAULT;

	return 0;
}

/*
 * ds_set_event_fd: Associate an eventfd with an interrupt.
 * @ds_dev: Pointer to the current ds_dev we're using.
 * @arg: Pointer to ds_interrupt_eventfd struct in userspace.
 */
static int ds_set_event_fd(struct ds_dev *ds_dev, ulong arg)
{
	struct ds_interrupt_eventfd die;

	if (copy_from_user(&die, (void __user *)arg,
		    sizeof(struct ds_interrupt_eventfd))) {
		return -EFAULT;
	}

	trace_ds_ioctl_eventfd_data(die.interrupt, die.event_fd);

	return ds_interrupt_set_eventfd(
		ds_dev->interrupt_data, die.interrupt, die.event_fd);
}

/*
 * ds_read_page_table_size: reads the size of the page table.
 * @ds_dev: Pointer to the current ds_dev we're using.
 * @arg: Pointer to ds_page_table_ioctl struct in userspace.
 *
 */
static int ds_read_page_table_size(struct ds_dev *ds_dev, ulong arg)
{
	int ret = 0;
	struct ds_page_table_ioctl ibuf;

	if (copy_from_user(&ibuf, (void __user *)arg,
		    sizeof(struct ds_page_table_ioctl)))
		return -EFAULT;

	if (ibuf.page_table_index >= ds_dev->num_page_tables)
		return -EFAULT;

	ibuf.size = ds_page_table_num_entries(
		ds_dev->page_table[ibuf.page_table_index]);

	trace_ds_ioctl_page_table_data(ibuf.page_table_index, ibuf.size,
		ibuf.host_address, ibuf.device_address);

	if (copy_to_user((void __user *)arg, &ibuf, sizeof(ibuf)))
		return -EFAULT;

	return ret;
}

/*
 * ds_read_simple page_table_size: reads the size of the simple page table.
 * @ds_dev: Pointer to the current ds_dev we're using.
 * @arg: Pointer to ds_page_table_ioctl struct in userspace.
 *
 */
static int ds_read_simple_page_table_size(struct ds_dev *ds_dev, ulong arg)
{
	int ret = 0;
	struct ds_page_table_ioctl ibuf;

	if (copy_from_user(&ibuf, (void __user *)arg,
		    sizeof(struct ds_page_table_ioctl)))
		return -EFAULT;

	if (ibuf.page_table_index >= ds_dev->num_page_tables)
		return -EFAULT;

	ibuf.size = ds_page_table_num_simple_entries(
		ds_dev->page_table[ibuf.page_table_index]);

	trace_ds_ioctl_page_table_data(ibuf.page_table_index, ibuf.size,
		ibuf.host_address, ibuf.device_address);

	if (copy_to_user((void __user *)arg, &ibuf, sizeof(ibuf)))
		return -EFAULT;

	return ret;
}

/*
 * ds_partition_page_table: sets up the page table when ioctl is called.
 * @ds_dev: Pointer to the current ds_dev we're using.
 * @arg: Pointer to ds_page_table_ioctl struct in userspace.
 *
 */
static int ds_partition_page_table(struct ds_dev *ds_dev, ulong arg)
{
	int ret;
	struct ds_page_table_ioctl ibuf;
	uint max_page_table_size;

	if (copy_from_user(&ibuf, (void __user *)arg,
		    sizeof(struct ds_page_table_ioctl)))
		return -EFAULT;

	trace_ds_ioctl_page_table_data(ibuf.page_table_index, ibuf.size,
		ibuf.host_address, ibuf.device_address);

	if (ibuf.page_table_index >= ds_dev->num_page_tables)
		return -EFAULT;
	max_page_table_size = ds_page_table_max_size(
		ds_dev->page_table[ibuf.page_table_index]);

	if (ibuf.size > max_page_table_size) {
		ds_log_error(ds_dev,
			"Partition request 0x%llx too large, max is 0x%x.",
			ibuf.size, max_page_table_size);
		return -EINVAL;
	}

	mutex_lock(&ds_dev->mutex);

	ret = ds_page_table_partition(
		ds_dev->page_table[ibuf.page_table_index], ibuf.size);
	mutex_unlock(&ds_dev->mutex);

	return ret;
}

/*
 * ds_map_buffers: Maps a userspace buffer to a device virtual address.
 * @ds_dev: Pointer to the current ds_dev we're using.
 * @arg: Pointer to a ds_page_table_ioctl struct in userspace.
 */
static int ds_map_buffers(struct ds_dev *ds_dev, ulong arg)
{
	struct ds_page_table_ioctl ibuf;

	if (copy_from_user(&ibuf, (void __user *)arg,
		    sizeof(struct ds_page_table_ioctl)))
		return -EFAULT;

	trace_ds_ioctl_page_table_data(ibuf.page_table_index, ibuf.size,
		ibuf.host_address, ibuf.device_address);

	if (ibuf.page_table_index >= ds_dev->num_page_tables)
		return -EFAULT;

	if (ds_page_table_are_addrs_bad(
		    ds_dev->page_table[ibuf.page_table_index],
		    ibuf.host_address, ibuf.device_address, ibuf.size))
		return -EINVAL;

	return ds_page_table_map(ds_dev->page_table[ibuf.page_table_index],
		ibuf.host_address, ibuf.device_address, ibuf.size / PAGE_SIZE);
}

/*
 * ds_unmap_buffers: Unmaps a userspace buffer to a device virtual address.
 * @dev: Pointer to the current ds_dev we're using.
 * @arg: Pointer to a ds_page_table_ioctl struct in userspace.
 */
static int ds_unmap_buffers(struct ds_dev *ds_dev, ulong arg)
{
	struct ds_page_table_ioctl ibuf;

	if (copy_from_user(&ibuf, (void __user *)arg,
		    sizeof(struct ds_page_table_ioctl)))
		return -EFAULT;

	trace_ds_ioctl_page_table_data(ibuf.page_table_index, ibuf.size,
		ibuf.host_address, ibuf.device_address);

	if (ibuf.page_table_index >= ds_dev->num_page_tables)
		return -EFAULT;

	if (ds_page_table_is_dev_addr_bad(
		    ds_dev->page_table[ibuf.page_table_index],
		    ibuf.device_address, ibuf.size))
		return -EINVAL;

	ds_page_table_unmap(ds_dev->page_table[ibuf.page_table_index],
		ibuf.device_address, ibuf.size / PAGE_SIZE);

	return 0;
}

/*
 * ds_config_coherent_allocator: Tell the driver to reserve structures
 * for coherent allocation, and allocate or free the corresponding memory.
 * @dev: Pointer to the current ds_dev we're using.
 * @arg: Pointer to a ds_coherent_alloc_config_ioctl struct in userspace.
 */
static int ds_config_coherent_allocator(struct ds_dev *ds_dev, ulong arg)
{
	int ret;
	struct ds_coherent_alloc_config_ioctl ibuf;

	if (copy_from_user(&ibuf, (void __user *)arg,
		    sizeof(struct ds_coherent_alloc_config_ioctl)))
		return -EFAULT;

	trace_ds_ioctl_config_coherent_allocator(
		ibuf.enable, ibuf.size, ibuf.dma_address);

	if (ibuf.page_table_index >= ds_dev->num_page_tables)
		return -EFAULT;

	if (ibuf.size > PAGE_SIZE * MAX_NUM_COHERENT_PAGES) {
		ibuf.size = PAGE_SIZE * MAX_NUM_COHERENT_PAGES;
		return -ENOMEM;
	}

	if (ibuf.enable == 0) {
		ret = ds_free_coherent_memory(ds_dev, ibuf.size,
			ibuf.dma_address, ibuf.page_table_index);
	} else {
		ret = ds_alloc_coherent_memory(ds_dev, ibuf.size,
			&ibuf.dma_address, ibuf.page_table_index);
	}
	if (copy_to_user((void __user *)arg, &ibuf, sizeof(ibuf)))
		return -EFAULT;
	return ret;
}
