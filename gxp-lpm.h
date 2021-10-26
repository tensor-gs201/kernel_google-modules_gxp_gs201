/* SPDX-License-Identifier: GPL-2.0 */
/*
 * GXP power management interface.
 *
 * Copyright (C) 2020 Google LLC
 */
#ifndef __GXP_LPM_H__
#define __GXP_LPM_H__

#include <linux/types.h>

#include "gxp.h"

enum lpm_psm_csrs {
	LPM_REG_ENABLE_STATE_0 = 0x080,
	LPM_REG_ENABLE_STATE_1 = 0x180,
	LPM_REG_ENABLE_STATE_2 = 0x280,
	LPM_REG_ENABLE_STATE_3 = 0x380,
};

enum lpm_state {
	LPM_ACTIVE_STATE = 0,
	LPM_CG_STATE = 1,
	LPM_PG_W_RET_STATE = 2,
	LPM_PG_STATE = 3,
};

#define LPM_STATE_TABLE_SIZE (LPM_REG_ENABLE_STATE_1 - LPM_REG_ENABLE_STATE_0)

#define LPM_INSTRUCTION_OFFSET 0x00000944
#define LPM_INSTRUCTION_MASK 0x03000000
#define LPM_TOP_PSM 4
#define LPM_HW_MODE 0
#define LPM_SW_PSM_MODE 1

#define LPM_CFG_SW_PS_TARGET_OFFSET 2

#define CORE_WAKEUP_DOORBELL 0

#define AUR_DVFS_DOMAIN 17
#define AUR_DVFS_DEBUG_REQ (1 << 31)
#define AUR_DEBUG_CORE_FREQ (AUR_DVFS_DEBUG_REQ | (3 << 27))

#define AUR_DVFS_MIN_STATE 178000

/*
 * Initializes the power manager for the first time after block power up.
 * The function needs to be called once after a block power up event.
 */
void gxp_lpm_init(struct gxp_dev *gxp);
/*
 * Destroys the power manager in preparation for a block shutdown.
 * The function needs to be called once before a block shutdown event.
 */
void gxp_lpm_destroy(struct gxp_dev *gxp);
/*
 * Turns on the power manager for a specific core. i.e. powers up the core.
 */
int gxp_lpm_up(struct gxp_dev *gxp, uint core);
/*
 * Turns off the power manager for a specific core. i.e. powers down the core.
 */
void gxp_lpm_down(struct gxp_dev *gxp, uint core);
/*
 * Sets the block-level DVFS state. This function can be called at any point
 * after block power on.
 */
int gxp_blk_set_state(struct gxp_dev *gxp, unsigned long state);
/*
 * Returns the current DVFS state of the Aurora block.
 */
int gxp_blk_get_state(struct gxp_dev *gxp);

static inline u32 lpm_read_32(struct gxp_dev *gxp, uint reg_offset)
{
	uint offset = GXP_LPM_BASE + reg_offset;

	return gxp_read_32(gxp, offset);
}

static inline void lpm_write_32(struct gxp_dev *gxp, uint reg_offset, u32 value)
{
	uint offset = GXP_LPM_BASE + reg_offset;

	gxp_write_32(gxp, offset, value);
}

static inline u32 lpm_read_32_psm(struct gxp_dev *gxp, uint psm,
				  uint reg_offset)
{
	uint offset =
		GXP_LPM_PSM_0_BASE + (GXP_LPM_PSM_SIZE * psm) + reg_offset;

	return gxp_read_32(gxp, offset);
}

static inline void lpm_write_32_psm(struct gxp_dev *gxp, uint psm,
				    uint reg_offset, u32 value)
{
	uint offset =
		GXP_LPM_PSM_0_BASE + (GXP_LPM_PSM_SIZE * psm) + reg_offset;

	gxp_write_32(gxp, offset, value);
}

#endif /* __GXP_LPM_H__ */
