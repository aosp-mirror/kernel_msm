/*
 * (C) Copyright Advanced Micro Devices, Inc. 2002, 2007
 * Copyright (c) 2008-2009 QUALCOMM USA, INC.
 * 
 * All source code in this file is licensed under the following license
 * 
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can find it at http://www.fsf.org
 */
#ifndef _MSM_KGSL_H
#define _MSM_KGSL_H

/*context flags */
#define KGSL_CONTEXT_SAVE_GMEM		1
#define KGSL_CONTEXT_NO_GMEM_ALLOC	2

/* generic flag values */
#define KGSL_FLAGS_NORMALMODE  0x00000000
#define KGSL_FLAGS_SAFEMODE    0x00000001
#define KGSL_FLAGS_INITIALIZED0 0x00000002
#define KGSL_FLAGS_INITIALIZED 0x00000004
#define KGSL_FLAGS_STARTED     0x00000008
#define KGSL_FLAGS_ACTIVE      0x00000010
#define KGSL_FLAGS_RESERVED0   0x00000020
#define KGSL_FLAGS_RESERVED1   0x00000040
#define KGSL_FLAGS_RESERVED2   0x00000080

/* device id */
enum kgsl_deviceid {
	KGSL_DEVICE_ANY		= 0x00000000,
	KGSL_DEVICE_YAMATO	= 0x00000001,
	KGSL_DEVICE_G12		= 0x00000002,
	KGSL_DEVICE_MAX		= 0x00000002
};

struct kgsl_devinfo {

	unsigned int device_id;
	/* chip revision id
	* coreid:8 majorrev:8 minorrev:8 patch:8
	*/
	unsigned int chip_id;
	unsigned int mmu_enabled;
	unsigned int gmem_gpubaseaddr;
	/* if gmem_hostbaseaddr is NULL, we would know its not mapped into
	 * mmio space */
	unsigned int gmem_hostbaseaddr;
	unsigned int gmem_sizebytes;
};

/* this structure defines the region of memory that can be mmap()ed from this
   driver. The timestamp fields are volatile because they are written by the
   GPU
*/
struct kgsl_devmemstore {
	volatile unsigned int soptimestamp;
	unsigned int sbz;
	volatile unsigned int eoptimestamp;
	unsigned int sbz2;
	volatile unsigned int ts_cmp_enable;
	unsigned int sbz3;
	volatile unsigned int ref_wait_ts;
	unsigned int sbz4;
};

#define KGSL_DEVICE_MEMSTORE_OFFSET(field) \
	offsetof(struct kgsl_devmemstore, field)


/* timestamp id*/
enum kgsl_timestamp_type {
	KGSL_TIMESTAMP_CONSUMED = 0x00000001, /* start-of-pipeline timestamp */
	KGSL_TIMESTAMP_RETIRED  = 0x00000002, /* end-of-pipeline timestamp*/
	KGSL_TIMESTAMP_MAX      = 0x00000002,
};

/* property types - used with kgsl_device_getproperty */
enum kgsl_property_type {
	KGSL_PROP_DEVICE_INFO     = 0x00000001,
	KGSL_PROP_DEVICE_SHADOW   = 0x00000002,
	KGSL_PROP_DEVICE_POWER    = 0x00000003,
	KGSL_PROP_SHMEM           = 0x00000004,
	KGSL_PROP_SHMEM_APERTURES = 0x00000005,
	KGSL_PROP_MMU_ENABLE 	  = 0x00000006,
	KGSL_PROP_INTERRUPT_WAITS = 0x00000007,
};

struct kgsl_shadowprop {
	unsigned int gpuaddr;
	unsigned int size;
	unsigned int flags; /* contains KGSL_FLAGS_ values */
};

/* ioctls */
#define KGSL_IOC_TYPE 0x09

/* get misc info about the GPU
   type should be a value from enum kgsl_property_type
   value points to a structure that varies based on type
   sizebytes is sizeof() that structure
   for KGSL_PROP_DEVICE_INFO, use struct kgsl_devinfo
   this structure contaings hardware versioning info.
   for KGSL_PROP_DEVICE_SHADOW, use struct kgsl_shadowprop
   this is used to find mmap() offset and sizes for mapping
   struct kgsl_memstore into userspace.
*/
struct kgsl_device_getproperty {
	unsigned int type;
	void  *value;
	unsigned int sizebytes;
};

#define IOCTL_KGSL_DEVICE_GETPROPERTY \
	_IOWR(KGSL_IOC_TYPE, 0x2, struct kgsl_device_getproperty)


/* read a GPU register.
   offsetwords it the 32 bit word offset from the beginning of the
   GPU register space.
 */
struct kgsl_device_regread {
	unsigned int offsetwords;
	unsigned int value; /* output param */
};

#define IOCTL_KGSL_DEVICE_REGREAD \
	_IOWR(KGSL_IOC_TYPE, 0x3, struct kgsl_device_regread)


/* block until the GPU has executed past a given timestamp
 * timeout is in milliseconds.
 */
struct kgsl_device_waittimestamp {
	unsigned int timestamp;
	unsigned int timeout;
};

#define IOCTL_KGSL_DEVICE_WAITTIMESTAMP \
	_IOW(KGSL_IOC_TYPE, 0x6, struct kgsl_device_waittimestamp)


/* issue indirect commands to the GPU.
 * drawctxt_id must have been created with IOCTL_KGSL_DRAWCTXT_CREATE
 * ibaddr and sizedwords must specify a subset of a buffer created
 * with IOCTL_KGSL_SHAREDMEM_FROM_PMEM
 * flags may be a mask of KGSL_CONTEXT_ values
 * timestamp is a returned counter value which can be passed to
 * other ioctls to determine when the commands have been executed by
 * the GPU.
 */
struct kgsl_ringbuffer_issueibcmds {
	unsigned int drawctxt_id;
	unsigned int ibaddr;
	unsigned int sizedwords;
	unsigned int timestamp; /*output param */
	unsigned int flags;
};

#define IOCTL_KGSL_RINGBUFFER_ISSUEIBCMDS \
	_IOWR(KGSL_IOC_TYPE, 0x10, struct kgsl_ringbuffer_issueibcmds)

/* read the most recently executed timestamp value
 * type should be a value from enum kgsl_timestamp_type
 */
struct kgsl_cmdstream_readtimestamp {
	unsigned int type;
	unsigned int timestamp; /*output param */
};

#define IOCTL_KGSL_CMDSTREAM_READTIMESTAMP \
	_IOR(KGSL_IOC_TYPE, 0x11, struct kgsl_cmdstream_readtimestamp)

/* free memory when the GPU reaches a given timestamp.
 * gpuaddr specify a memory region created by a
 * IOCTL_KGSL_SHAREDMEM_FROM_PMEM call
 * type should be a value from enum kgsl_timestamp_type
 */
struct kgsl_cmdstream_freememontimestamp {
	unsigned int gpuaddr;
	unsigned int type;
	unsigned int timestamp;
};

#define IOCTL_KGSL_CMDSTREAM_FREEMEMONTIMESTAMP \
	_IOR(KGSL_IOC_TYPE, 0x12, struct kgsl_cmdstream_freememontimestamp)

/* create a draw context, which is used to preserve GPU state.
 * The flags field may contain a mask KGSL_CONTEXT_*  values
 */
struct kgsl_drawctxt_create {
	unsigned int flags;
	unsigned int drawctxt_id; /*output param */
};

#define IOCTL_KGSL_DRAWCTXT_CREATE \
	_IOWR(KGSL_IOC_TYPE, 0x13, struct kgsl_drawctxt_create)

/* destroy a draw context */
struct kgsl_drawctxt_destroy {
	unsigned int drawctxt_id;
};

#define IOCTL_KGSL_DRAWCTXT_DESTROY \
	_IOW(KGSL_IOC_TYPE, 0x14, struct kgsl_drawctxt_destroy)

/* add a block of pmem or fb into the GPU address space */
struct kgsl_sharedmem_from_pmem {
	int pmem_fd;
	unsigned int gpuaddr;	/*output param */
	unsigned int len;
	unsigned int offset;
};

#define IOCTL_KGSL_SHAREDMEM_FROM_PMEM \
	_IOWR(KGSL_IOC_TYPE, 0x20, struct kgsl_sharedmem_from_pmem)

/* remove memory from the GPU's address space */
struct kgsl_sharedmem_free {
	unsigned int gpuaddr;
};

#define IOCTL_KGSL_SHAREDMEM_FREE \
	_IOW(KGSL_IOC_TYPE, 0x21, struct kgsl_sharedmem_free)

struct kgsl_gmem_desc {
	unsigned int x;
	unsigned int y;
	unsigned int width;
	unsigned int height;
	unsigned int pitch;
};

struct kgsl_buffer_desc {
	void 		*hostptr;
	unsigned int	gpuaddr;
	int		size;
	unsigned int	format;
	unsigned int  	pitch;
	unsigned int  	enabled;
};

struct kgsl_bind_gmem_shadow {
	unsigned int drawctxt_id;
	struct kgsl_gmem_desc gmem_desc;
	unsigned int shadow_x;
	unsigned int shadow_y;
	struct kgsl_buffer_desc shadow_buffer;
	unsigned int buffer_id;
};

#define IOCTL_KGSL_DRAWCTXT_BIND_GMEM_SHADOW \
    _IOW(KGSL_IOC_TYPE, 0x22, struct kgsl_bind_gmem_shadow)

/* add a block of memory into the GPU address space */
struct kgsl_sharedmem_from_vmalloc {
	unsigned int gpuaddr;	/*output param */
	unsigned int hostptr;
	/* If set from user space then will attempt to
	 * allocate even if low watermark is crossed */
	int force_no_low_watermark;
};

#define IOCTL_KGSL_SHAREDMEM_FROM_VMALLOC \
	_IOWR(KGSL_IOC_TYPE, 0x23, struct kgsl_sharedmem_from_vmalloc)

#define IOCTL_KGSL_SHAREDMEM_FLUSH_CACHE \
	_IOW(KGSL_IOC_TYPE, 0x24, struct kgsl_sharedmem_free)

struct kgsl_drawctxt_set_bin_base_offset {
	unsigned int drawctxt_id;
	unsigned int offset;
};

#define IOCTL_KGSL_DRAWCTXT_SET_BIN_BASE_OFFSET \
	_IOW(KGSL_IOC_TYPE, 0x25, struct kgsl_drawctxt_set_bin_base_offset)

#endif /* _MSM_KGSL_H */
