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
#ifndef __GSL_DRAWCTXT_H
#define __GSL_DRAWCTXT_H

/* Flags */

#define CTXT_FLAGS_NOT_IN_USE		0x00000000
#define CTXT_FLAGS_IN_USE			0x00000001

/* state shadow memory allocated */
#define CTXT_FLAGS_STATE_SHADOW		0x00000010

/* gmem shadow memory allocated */
#define CTXT_FLAGS_GMEM_SHADOW		0x00000100
/* gmem must be copied to shadow */
#define CTXT_FLAGS_GMEM_SAVE		0x00000200
/* gmem can be restored from shadow */
#define CTXT_FLAGS_GMEM_RESTORE		0x00000400
/* shader must be copied to shadow */
#define CTXT_FLAGS_SHADER_SAVE		0x00002000
/* shader can be restored from shadow */
#define CTXT_FLAGS_SHADER_RESTORE	0x00004000

#include "kgsl_sharedmem.h"
#include "yamato_reg.h"

#define KGSL_MAX_GMEM_SHADOW_BUFFERS	2

struct kgsl_device;

/*  types */

/* draw context */
struct gmem_shadow_t {
	struct kgsl_memdesc gmemshadow;	/* Shadow buffer address */

	/* 256 KB GMEM surface = 4 bytes-per-pixel x 256 pixels/row x
	* 256 rows. */
	/* width & height must be a multiples of 32, in case tiled textures
	 * are used. */
	enum COLORFORMATX format;
	unsigned int size;	/* Size of surface used to store GMEM */
	unsigned int width;	/* Width of surface used to store GMEM */
	unsigned int height;	/* Height of surface used to store GMEM */
	unsigned int pitch;	/* Pitch of surface used to store GMEM */
	int offset;
	unsigned int offset_x;
	unsigned int offset_y;
	unsigned int gmem_offset_x;
	unsigned int gmem_offset_y;
	unsigned int gmem_pitch;	/* Pitch value used for GMEM */
	unsigned int *gmem_save_commands;
	unsigned int *gmem_restore_commands;
	unsigned int gmem_save[3];
	unsigned int gmem_restore[3];
	struct kgsl_memdesc quad_vertices;
	struct kgsl_memdesc quad_texcoords;
};

struct kgsl_drawctxt {
	uint32_t         flags;
	struct kgsl_pagetable *pagetable;
	struct kgsl_memdesc       gpustate;
	unsigned int        reg_save[3];
	unsigned int        reg_restore[3];
	unsigned int        shader_save[3];
	unsigned int        shader_fixup[3];
	unsigned int        shader_restore[3];
	unsigned int		chicken_restore[3];
	unsigned int 	    bin_base_offset;
	/* Information of the GMEM shadow that is created in context create */
	struct gmem_shadow_t context_gmem_shadow;
	/* User defined GMEM shadow buffers */
	struct gmem_shadow_t user_gmem_shadow[KGSL_MAX_GMEM_SHADOW_BUFFERS];
};


int kgsl_drawctxt_create(struct kgsl_device *, struct kgsl_pagetable *,
			  unsigned int flags,
			  unsigned int *drawctxt_id);

int kgsl_drawctxt_destroy(struct kgsl_device *device, unsigned int drawctxt_id);

int kgsl_drawctxt_init(struct kgsl_device *device);

int kgsl_drawctxt_close(struct kgsl_device *device);

void kgsl_drawctxt_switch(struct kgsl_device *device,
				struct kgsl_drawctxt *drawctxt,
				unsigned int flags);
int kgsl_drawctxt_bind_gmem_shadow(struct kgsl_device *device,
			unsigned int drawctxt_id,
			const struct kgsl_gmem_desc *gmem_desc,
			unsigned int shadow_x,
			unsigned int shadow_y,
			const struct kgsl_buffer_desc
			*shadow_buffer, unsigned int buffer_id);

int kgsl_drawctxt_set_bin_base_offset(struct kgsl_device *device,
					unsigned int drawctxt_id,
					unsigned int offset);

#endif  /* __GSL_DRAWCTXT_H */
