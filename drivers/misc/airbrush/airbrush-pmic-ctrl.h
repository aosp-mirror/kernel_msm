/*
 * Copyright (C) 2018 Samsung Electronics Co., Ltd.
 *
 * Authors:
 *	Raman Kumar Banka <raman.k2@samsung.com>
 *
 * PMIC rails controller for airbrush state manager.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 */
#ifndef _AIRBRUSH_PMIC_CTRL_H_
#define _AIRBRUSH_PMIC_CTRL_H_

int ab_blk_pw_rails_enable(struct ab_state_context *sc,
				   block_name_t blk_name);
int ab_blk_pw_rails_disable(struct ab_state_context *sc,
				   block_name_t blk_name);

int ab_pmic_on(struct ab_state_context *ab_ctx);
int ab_get_pmic_resources(struct ab_state_context *ab_ctx);

#endif // _AIRBRUSH_PMIC_CTRL_H_
