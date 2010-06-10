/* arch/arm/mach-msm/qdsp5v2/adsp.h
 *
 * Copyright (C) 2010 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef _MSM_ADSP_5V2_H_
#define _MSM_ADSP_5V2_H_

struct msm_adsp_module;

typedef void (*msm_adsp_callback)(unsigned id, void *event,
				  size_t len, void *cookie);


int msm_adsp_get(const char *name, struct msm_adsp_module **module,
		 msm_adsp_callback callback, void *cookie);

void msm_adsp_put(struct msm_adsp_module *module);

/* find queue index for a named module command queue */
int msm_adsp_lookup_queue(struct msm_adsp_module *module, const char *name);

int msm_adsp_enable(struct msm_adsp_module *module);
int msm_adsp_disable(struct msm_adsp_module *module);

/* write is safe to call from atomic context.  All other msm_adsp_*
 * calls may block.
 */
int msm_adsp_write(struct msm_adsp_module *module, unsigned queue_idx,
		   void *data, size_t len);

#endif
