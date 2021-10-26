/* SPDX-License-Identifier: GPL-2.0 */
/*
 * GXP doorbell interface.
 *
 * Copyright (C) 2020 Google LLC
 */
#ifndef __GXP_DOORBELL_H__
#define __GXP_DOORBELL_H__

#include "gxp-internal.h"

void gxp_doorbell_set_listening_core(struct gxp_dev *gxp, u32 doorbell_num,
				     uint core);
void gxp_doorbell_set(struct gxp_dev *gxp, u32 doorbell_num);
void gxp_doorbell_clear(struct gxp_dev *gxp, u32 doorbell_num);
u32 gxp_doorbell_status(struct gxp_dev *gxp, u32 doorbell_num);

#endif /* __GXP_DOORBELL_H__ */
