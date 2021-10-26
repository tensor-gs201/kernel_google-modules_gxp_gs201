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
int gxp_firmware_run(struct gxp_dev *gxp, uint core);
/*
 * Shuts down the specified core.
 */
void gxp_firmware_stop(struct gxp_dev *gxp, uint core);

#endif /* __GXP_FIRMWARE_H__ */
