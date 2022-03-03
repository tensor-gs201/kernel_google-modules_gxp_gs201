/* SPDX-License-Identifier: GPL-2.0 */
/*
 * GXP power management.
 *
 * Copyright (C) 2021 Google LLC
 */
#ifndef __GXP_PM_H__
#define __GXP_PM_H__

#include "gxp-internal.h"
#include <linux/refcount.h>
#include <soc/google/exynos_pm_qos.h>

#define AUR_DVFS_MIN_STATE 178000

enum aur_power_state {
	AUR_OFF = 0,
	AUR_UUD = 178000,
	AUR_SUD = 373000,
	AUR_UD = 750000,
	AUR_NOM = 1160000,
};

enum aur_memory_power_state {
	AUR_MEM_UNDEFINED = 0,
	AUR_MEM_MIN = 1,
	AUR_MEM_VERY_LOW = 2,
	AUR_MEM_LOW = 3,
	AUR_MEM_HIGH = 4,
	AUR_MEM_VERY_HIGH = 5,
	AUR_MEM_MAX = 6,
};

#define AUR_NUM_POWER_STATE 5
#define AUR_NUM_MEMORY_POWER_STATE (AUR_MAX_ALLOW_MEMORY_STATE + 1)

#define AUR_INIT_DVFS_STATE AUR_UUD
#define AUR_MAX_ALLOW_STATE AUR_NOM

#define AUR_MAX_ALLOW_MEMORY_STATE AUR_MEM_MAX

struct gxp_pm_device_ops {
	int (*pre_blk_powerup)(struct gxp_dev *gxp);
	int (*post_blk_powerup)(struct gxp_dev *gxp);
	int (*pre_blk_poweroff)(struct gxp_dev *gxp);
	int (*post_blk_poweroff)(struct gxp_dev *gxp);
};

struct gxp_set_acpm_state_work {
	struct work_struct work;
	struct gxp_dev *gxp;
	unsigned long state;
};

struct gxp_power_manager {
	struct gxp_dev *gxp;
	struct mutex pm_lock;
	int pwr_state_req_count[AUR_NUM_POWER_STATE];
	uint mem_pwr_state_req_count[AUR_NUM_MEMORY_POWER_STATE];
	int curr_state;
	int curr_memory_state;
	refcount_t blk_wake_ref;
	struct gxp_pm_device_ops *ops;
	struct gxp_set_acpm_state_work set_acpm_rate_work;
	struct workqueue_struct *wq;
	/* INT/MIF requests for memory bandwidth */
	struct exynos_pm_qos_request int_min;
	struct exynos_pm_qos_request mif_min;
};

/**
 * gxp_pm_blk_on() - Turn on the power for BLK_AUR
 * @gxp: The GXP device to turn on
 *
 * Return:
 * * 0       - BLK ON successfully
 * * -ENODEV - Cannot find PM interface
 */
int gxp_pm_blk_on(struct gxp_dev *gxp);

/**
 * gxp_pm_blk_off() - Turn off the power for BLK_AUR
 * @gxp: The GXP device to turn off
 *
 * Return:
 * * 0       - BLK OFF successfully
 * * -ENODEV - Cannot find PM interface
 * * -EBUSY  - Wakelock is held, blk is still busy
 */
int gxp_pm_blk_off(struct gxp_dev *gxp);

/**
 * gxp_pm_get_blk_state() - Get the blk power state
 * @gxp: The GXP device to sample state
 *
 * Return:
 * * state   - State number represented in kHZ, or 0 if OFF
 */
int gxp_pm_get_blk_state(struct gxp_dev *gxp);

/**
 * gxp_pm_core_on() - Turn on a core on GXP device
 * @gxp: The GXP device to operate
 * @core: The core ID to turn on
 *
 * Return:
 * * 0       - Core on process finished successfully
 * * -ETIMEDOUT - Core on process timed-out.
 */
int gxp_pm_core_on(struct gxp_dev *gxp, uint core);

/**
 * gxp_pm_core_off() - Turn off a core on GXP device
 * @gxp: The GXP device to operate
 * @core: The core ID to turn off
 *
 * Return:
 * * 0       - Core off process finished successfully
 */
int gxp_pm_core_off(struct gxp_dev *gxp, uint core);

/**
 * gxp_pm_acquire_blk_wakelock() - Acquire blk wakelock
 * to make sure block won't shutdown.
 *
 * Can be called multiple times and it will increase
 * reference count.
 *
 * @gxp: The GXP device to operate
 *
 * Return:
 * * 0       - Wakelock acquired
 */
int gxp_pm_acquire_blk_wakelock(struct gxp_dev *gxp);

/**
 * gxp_pm_release_blk_wakelock() - Release blk wakelock.
 *
 * Can be called multiple times and it will decrease
 * reference count till 0.
 *
 * @gxp: The GXP device to operate
 *
 * Return:
 * * 0       - Wakelock released
 * * -EIO    - No wakelock is currently held
 */
int gxp_pm_release_blk_wakelock(struct gxp_dev *gxp);

/**
 * gxp_pm_req_state() - API to request a desired power state.
 * @gxp: The GXP device to operate
 * @state: The requested state
 *
 * Return:
 * * 0       - Voting registered
 * * -EINVAL - Invalid requested state
 */
int gxp_pm_req_state(struct gxp_dev *gxp, enum aur_power_state state);

/**
 * gxp_pm_init() - API for initialize PM
 * interface for GXP, should only be called
 * once per probe
 * @gxp: The GXP device to operate
 *
 * Return:
 * * 0       - Initialization finished successfully
 * * -ENOMEM - Cannot get memory to finish init.
 */
int gxp_pm_init(struct gxp_dev *gxp);

/**
 * gxp_pm_destroy() - API for removing
 * the power management interface
 * @gxp: The GXP device to operate
 *
 * Return:
 * * 0       - Remove finished successfully
 */
int gxp_pm_destroy(struct gxp_dev *gxp);

/**
 * gxp_pm_blk_set_state_acpm() - API for setting the block-level DVFS state.
 * This function can be called at any point after block power on.
 * @gxp: The GXP device to operate
 * @state: State number in khz that need to be set.
 *         Supported state is in enum aur_power_state,
 *         if experiment is needed for unsupported state
 *         please refer to Lassen's ECT table.
 *
 * Return:
 * * 0       - Set finished successfully
 * * Other   - Set state encounter issue in exynos_acpm_set_rate
 */
int gxp_pm_blk_set_state_acpm(struct gxp_dev *gxp, unsigned long state);

/**
 * gxp_pm_blk_get_state_acpm() - API for getting
 * the current DVFS state of the Aurora block.
 * @gxp: The GXP device to operate
 *
 * Return:
 * * State   - State number in Khz from ACPM
 */
int gxp_pm_blk_get_state_acpm(struct gxp_dev *gxp);

/**
 * gxp_pm_update_requested_power_state() - API for a GXP client to vote for a
 * requested state.
 * @gxp: The GXP device to operate.
 * @origin_state: An existing old requested state, will be cleared. If this is
 * the first vote, pass AUR_OFF.
 * @requested_state: The new requested state.
 *
 * Return:
 * * 0       - Voting registered
 * * -EINVAL - Invalid original state or requested state
 */
int gxp_pm_update_requested_power_state(struct gxp_dev *gxp,
					enum aur_power_state origin_state,
					enum aur_power_state requested_state);

/**
 * gxp_pm_update_requested_memory_power_state() - API for a GXP client to vote for a
 * requested memory power state.
 * @gxp: The GXP device to operate.
 * @origin_state: An existing old requested state, will be cleared. If this is
 * the first vote, pass AUR_MEM_UNDEFINED.
 * @requested_state: The new requested state.
 *
 * Return:
 * * 0       - Voting registered
 * * -EINVAL - Invalid original state or requested state
 */
int gxp_pm_update_requested_memory_power_state(
	struct gxp_dev *gxp, enum aur_memory_power_state origin_state,
	enum aur_memory_power_state requested_state);

#endif /* __GXP_PM_H__ */
