/* SPDX-License-Identifier: GPL-2.0 */
/*
 * GXP power management.
 *
 * Copyright (C) 2021 Google LLC
 */
#ifndef __GXP_PM_H__
#define __GXP_PM_H__

#include <soc/google/exynos_pm_qos.h>

#include "gxp-internal.h"

#define AUR_DVFS_MIN_RATE 178000
static const uint aur_power_state2rate[] = { 0,	     178000,  373000,
					     750000, 1160000, 178000 };

enum aur_power_state {
	AUR_OFF = 0,
	AUR_UUD = 1,
	AUR_SUD = 2,
	AUR_UD = 3,
	AUR_NOM = 4,
	AUR_READY = 5,
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

enum aur_power_cmu_mux_state {
	AUR_CMU_MUX_LOW = 0,
	AUR_CMU_MUX_NORMAL = 1,
};

#define AUR_NUM_POWER_STATE (AUR_MAX_ALLOW_STATE + 1)
#define AUR_NUM_MEMORY_POWER_STATE (AUR_MAX_ALLOW_MEMORY_STATE + 1)

#define AUR_INIT_DVFS_STATE AUR_UUD

/*
 * These macros mean the maximum valid enum value of aur_power_state and
 * aur_memory_power_state, not necessarily the state with the maximum power
 * level.
 */
#define AUR_MAX_ALLOW_STATE AUR_READY
#define AUR_MAX_ALLOW_MEMORY_STATE AUR_MEM_MAX

/*
 * The bit to indicate non-aggressor vote for `exynos_acpm_set_rate`.
 * Lower 3 byte of frequency parameter of `exynos_acpm_set_rate` will still be
 * the requested rate.
 */
#define AUR_NON_AGGRESSOR_BIT 24

#define AUR_NUM_POWER_STATE_WORKER 16

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
	unsigned long prev_state;
	bool aggressor_vote;
	bool using;
};

struct gxp_req_pm_qos_work {
	struct work_struct work;
	struct gxp_dev *gxp;
	s32 int_val;
	s32 mif_val;
	bool using;
};

struct gxp_power_manager {
	struct gxp_dev *gxp;
	struct mutex pm_lock;
	uint pwr_state_req_count[AUR_NUM_POWER_STATE];
	uint non_aggressor_pwr_state_req_count[AUR_NUM_POWER_STATE];
	uint mem_pwr_state_req_count[AUR_NUM_MEMORY_POWER_STATE];
	bool curr_aggressor_vote;
	int curr_state;
	int curr_memory_state;
	struct gxp_pm_device_ops *ops;
	struct gxp_set_acpm_state_work
		set_acpm_state_work[AUR_NUM_POWER_STATE_WORKER];
	/* Serializes searching for an open worker in set_acpm_state_work[] */
	struct mutex set_acpm_state_work_lock;
	struct gxp_req_pm_qos_work req_pm_qos_work[AUR_NUM_POWER_STATE_WORKER];
	/* Serializes searching for an open worker in req_pm_qos_work[] */
	struct mutex req_pm_qos_work_lock;
	struct workqueue_struct *wq;
	/* INT/MIF requests for memory bandwidth */
	struct exynos_pm_qos_request int_min;
	struct exynos_pm_qos_request mif_min;
	int force_noc_mux_normal_count;
	/* Max frequency that the thermal driver/ACPM will allow in Hz */
	unsigned long thermal_limit;
	u64 blk_switch_count;
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
 * gxp_pm_get_blk_switch_count() - Get the blk switch count number
 * @gxp: The GXP device to switch the blk
 *
 * Return:
 * * count   - Switch count number after the module initialization.
 */
int gxp_pm_get_blk_switch_count(struct gxp_dev *gxp);

/**
 * gxp_pm_core_on() - Turn on a core on GXP device
 * @gxp: The GXP device to operate
 * @core: The core ID to turn on
 * @verbose: A boolean flag to indicate whether to print the log
 *
 * Return:
 * * 0       - Core on process finished successfully
 * * -ETIMEDOUT - Core on process timed-out.
 */
int gxp_pm_core_on(struct gxp_dev *gxp, uint core, bool verbose);

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
 * gxp_pm_init() - API for initialize PM interface for GXP, should only be
 * called once per probe
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
 * gxp_pm_blk_set_rate_acpm() - API for setting the block-level DVFS rate.
 * This function can be called at any point after block power on.
 * @gxp: The GXP device to operate
 * @rate: Rate number in khz that need to be set.
 *         Supported rate is in aur_power_state2rate,
 *         if experiment is needed for unsupported rate
 *         please refer to Lassen's ECT table.
 *
 * Return:
 * * 0       - Set finished successfully
 * * Other   - Set rate encounter issue in exynos_acpm_set_rate
 */
int gxp_pm_blk_set_rate_acpm(struct gxp_dev *gxp, unsigned long rate);

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
 * gxp_pm_update_requested_power_states() - API for a GXP client to vote for a
 * requested power state and a requested memory power state.
 * @gxp: The GXP device to operate.
 * @origin_state: An existing old requested state, will be cleared. If this is
 *                the first vote, pass AUR_OFF.
 * @origin_requested_aggressor: Specify whether the existing vote was requested with
 *                              aggressor flag.
 * @requested_state: The new requested state.
 * @requested_aggressor: Specify whether the new vote is requested with aggressor
 *                       flag. Will take no effect if the @requested state is
 *                       AUR_OFF.
 * @origin_mem_state: An existing old requested state, will be cleared. If this is
 *                the first vote, pass AUR_MEM_UNDEFINED.
 * @requested_mem_state: The new requested state.
 *
 * Return:
 * * 0       - Voting registered
 * * -EINVAL - Invalid original state or requested state
 */

int gxp_pm_update_requested_power_states(
	struct gxp_dev *gxp, enum aur_power_state origin_state,
	bool origin_requested_aggressor, enum aur_power_state requested_state,
	bool requested_aggressor, enum aur_memory_power_state origin_mem_state,
	enum aur_memory_power_state requested_mem_state);

/*
 * gxp_pm_force_cmu_noc_user_mux_normal() - Force PLL_CON0_NOC_USER MUX switch to the
 * normal state. This is required to guarantee LPM works when the core is starting the
 * firmware.
 */
void gxp_pm_force_cmu_noc_user_mux_normal(struct gxp_dev *gxp);

/*
 * gxp_pm_check_cmu_noc_user_mux() - Check PLL_CON0_NOC_USER MUX state modified
 * by gxp_pm_force_cmu_noc_user_mux_normal(). If the requested state is
 * AUR_READY, should set it to AUR_CMU_MUX_LOW.
 */
void gxp_pm_check_cmu_noc_user_mux(struct gxp_dev *gxp);

/**
 * gxp_pm_set_thermal_limit() - Notify the power manager of a thermal limit
 * @gxp: The GXP device the limit is set for
 * @thermal_limit: The highest frequency, in Hz, the thermal limit allows
 *
 * The power management code will only use this information for logging.
 */
void gxp_pm_set_thermal_limit(struct gxp_dev *gxp, unsigned long thermal_limit);

#endif /* __GXP_PM_H__ */
