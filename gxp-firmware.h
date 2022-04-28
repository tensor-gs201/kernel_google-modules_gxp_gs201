/* SPDX-License-Identifier: GPL-2.0 */
/*
 * GXP firmware loader.
 *
 * Copyright (C) 2020 Google LLC
 */
#ifndef __GXP_FIRMWARE_H__
#define __GXP_FIRMWARE_H__

#include <linux/bitops.h>

#include "gxp-internal.h"

#if !IS_ENABLED(CONFIG_GXP_TEST)

#define AURORA_SCRATCHPAD_OFF 0x00F00000 /* Last 1M of ELF load region */
#define AURORA_SCRATCHPAD_LEN 0x00100000 /* 1M */

#else /* CONFIG_GXP_TEST */
/* Firmware memory is shrunk in unit tests. */
#define AURORA_SCRATCHPAD_OFF 0x000F0000
#define AURORA_SCRATCHPAD_LEN 0x00010000

#endif /* CONFIG_GXP_TEST */

#define Q7_ALIVE_MAGIC	0x55555555

#define CORE_SCRATCHPAD_BASE(_core_) (_core_ << 16)
#define SCRATCHPAD_MSG_OFFSET(_msg_) (_msg_  <<  2)

enum aurora_msg {
	MSG_CORE_ALIVE,
	MSG_TOP_ACCESS_OK,
	MSG_SCRATCHPAD_MAX,
};

static inline bool gxp_is_fw_running(struct gxp_dev *gxp, uint core)
{
	return (gxp->firmware_running & BIT(core)) != 0;
}

/*
 * Initializes the firmware loading/unloading subsystem. This includes
 * initializing the LPM and obtaining the memory regions needed to load the FW.
 * The function needs to be called once after a block power up event.
 */
int gxp_fw_init(struct gxp_dev *gxp);
/*
 * Tears down the firmware loading/unloading subsystem in preparation for a
 * block-level shutdown event. To be called once before a block shutdown.
 */
void gxp_fw_destroy(struct gxp_dev *gxp);
/*
 * Loads the firmware for the specified core in system memory and powers up the
 * core to start FW execution.
 */
int gxp_firmware_run(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
		     uint virt_core, uint core);
/*
 * Shuts down the specified core.
 */
void gxp_firmware_stop(struct gxp_dev *gxp, struct gxp_virtual_device *vd,
		       uint virt_core, uint core);

#endif /* __GXP_FIRMWARE_H__ */
