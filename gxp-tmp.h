/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Temporary configuration file fore GXP.
 *
 * Copyright (C) 2021 Google LLC
 */
#ifndef __GXP_TMP_H__
#define __GXP_TMP_H__

/* TODO (b/176979630): Delete gxp.tmp.h. Move definitions to gxp-config.h */

#if !IS_ENABLED(CONFIG_GXP_TEST)

#define AURORA_SCRATCHPAD_OFF 0x00F00000 /* Last 1M of ELF load region */
#define AURORA_SCRATCHPAD_LEN 0x00100000 /* 1M */

#else /* CONFIG_GXP_TEST */
/* Firmware memory is shrinked in unit tests. */
#define AURORA_SCRATCHPAD_OFF 0x000F0000
#define AURORA_SCRATCHPAD_LEN 0x00010000

#endif /* CONFIG_GXP_TEST */

#define Q7_ALIVE_MAGIC	0x55555555

#define LPM_BLOCK		0x040000
#define DOORBELL_BLOCK		0x0C0000
#define SYNC_BARRIER_BLOCK	0x00100000

#define DOORBELL_BASE(_x_)	((_x_) << 12)
#define DOORBELL_COUNT		32
#define DOORBELL_STATUS_OFFSET	0x0
#define DOORBELL_SET_OFFSET	0x4
#define DOORBELL_CLEAR_OFFSET	0x8
#define DOORBELL_EN_ALL_MASK	0xFFFFFFFF

#define SYNC_BARRIER_BASE(_x_)	((_x_) << 12)
#define SYNC_BARRIER_FREE_VALUE	0xF
#define SYNC_BARRIER_COUNT	16

#define CORE_PSM_BASE(_core_) ((_core_ + 1) << 12)
#define TOP_PSM_BASE		0x5000

#define PSM_INIT_DONE_MASK		0x80
#define PSM_CURR_STATE_MASK		0x0F
#define PSM_STATE_VALID_MASK	0x10

#define PSM_HW_MODE	0x0
#define PSM_START	0x1

#define PSM_STATE_ACTIVE		0x0
#define PSM_STATE_CLK_GATED		0x1

#define PROVINO_IXBAR1_ARL_CTRL	0x1818
#define PROVINO_IXBAR1_ARL_EN	(0x1 << 31)

#define DISABLE	0x0
#define ENABLE	0x1

#define CORE_SCRATCHPAD_BASE(_core_) (_core_ << 16)
#define SCRATCHPAD_MSG_OFFSET(_msg_) (_msg_  <<  2)

enum aurora_msg {
	MSG_CORE_ALIVE,
	MSG_TOP_ACCESS_OK,
	MSG_SCRATCHPAD_MAX,
};

#endif /* __GXP_TMP_H__ */
