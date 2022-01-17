// SPDX-License-Identifier: GPL-2.0
/*
 * GXP power management.
 *
 * Copyright (C) 2021 Google LLC
 */

#include <linux/io.h>
#include <linux/pm_runtime.h>
#include <linux/refcount.h>
#include <linux/types.h>

#ifdef CONFIG_GXP_CLOUDRIPPER
#include <linux/acpm_dvfs.h>
#endif

#include "gxp-bpm.h"
#include "gxp-doorbell.h"
#include "gxp-internal.h"
#include "gxp-lpm.h"
#include "gxp-pm.h"

static struct gxp_pm_device_ops gxp_aur_ops = {
	.pre_blk_powerup = NULL,
	.post_blk_powerup = NULL,
	.pre_blk_poweroff = NULL,
	.post_blk_poweroff = NULL,
};

static int gxp_pm_blkpwr_up(struct gxp_dev *gxp)
{
	int ret = 0;

#if defined(CONFIG_GXP_CLOUDRIPPER) && !defined(CONFIG_GXP_TEST)
	/*
	 * This function is equivalent to pm_runtime_get_sync, but will prevent
	 * the pm_runtime refcount from increasing if the call fails. It also
	 * only returns either 0 for success or an errno on failure.
	 */
	ret = pm_runtime_resume_and_get(gxp->dev);
	if (ret)
		dev_err(gxp->dev, "%s: pm_runtime_resume_and_get returned %d\n",
			__func__, ret);
#endif
	return ret;
}

static int gxp_pm_blkpwr_down(struct gxp_dev *gxp)
{
	int ret = 0;

#if defined(CONFIG_GXP_CLOUDRIPPER) && !defined(CONFIG_GXP_TEST)
	/*
	 * Need to put TOP LPM into active state before blk off
	 * b/189396709
	 */
	lpm_write_32_psm(gxp, LPM_TOP_PSM, LPM_REG_ENABLE_STATE_1, 0x0);
	lpm_write_32_psm(gxp, LPM_TOP_PSM, LPM_REG_ENABLE_STATE_2, 0x0);
	ret = pm_runtime_put_sync(gxp->dev);
	if (ret)
		/*
		 * pm_runtime_put_sync() returns the device's usage counter.
		 * Negative values indicate an error, while any positive values
		 * indicate the device is still in use somewhere. The only
		 * expected value here is 0, indicating no remaining users.
		 */
		dev_err(gxp->dev, "%s: pm_runtime_put_sync returned %d\n",
			__func__, ret);
#endif
	return ret;
}

int gxp_pm_blk_set_state_acpm(struct gxp_dev *gxp, unsigned long state)
{
	int ret = 0;

#if defined(CONFIG_GXP_CLOUDRIPPER)
	ret = exynos_acpm_set_rate(AUR_DVFS_DOMAIN, state);
	dev_dbg(gxp->dev, "%s: state %lu, ret %d\n", __func__, state, ret);
#endif
	return ret;
}

int gxp_pm_blk_get_state_acpm(struct gxp_dev *gxp)
{
	int ret = 0;

#if defined(CONFIG_GXP_CLOUDRIPPER)
	ret = exynos_acpm_get_rate(AUR_DVFS_DOMAIN, AUR_DEBUG_CORE_FREQ);
	dev_dbg(gxp->dev, "%s: state %d\n", __func__, ret);
#endif
	return ret;
}

int gxp_pm_blk_on(struct gxp_dev *gxp)
{
	int ret = 0;

	if (WARN_ON(!gxp->power_mgr)) {
		dev_err(gxp->dev, "%s: No PM found\n", __func__);
		return -ENODEV;
	}

	mutex_lock(&gxp->power_mgr->pm_lock);
	ret = gxp_pm_blkpwr_up(gxp);
	if (!ret) {
		gxp_pm_blk_set_state_acpm(gxp, AUR_INIT_DVFS_STATE);
		gxp->power_mgr->curr_state = AUR_INIT_DVFS_STATE;
	}

	/* Startup TOP's PSM */
	gxp_lpm_init(gxp);

	mutex_unlock(&gxp->power_mgr->pm_lock);

	return ret;
}

int gxp_pm_blk_off(struct gxp_dev *gxp)
{
	int ret = 0;

	if (WARN_ON(!gxp->power_mgr)) {
		dev_err(gxp->dev, "%s: No PM found\n", __func__);
		return -ENODEV;
	}
	mutex_lock(&gxp->power_mgr->pm_lock);
	if (refcount_read(&(gxp->power_mgr->blk_wake_ref))) {
		dev_err(gxp->dev, "%s: Wake lock not released\n", __func__);
		mutex_unlock(&gxp->power_mgr->pm_lock);
		return -EBUSY;
	}

	/* Shutdown TOP's PSM */
	gxp_lpm_destroy(gxp);

	ret = gxp_pm_blkpwr_down(gxp);
	if (!ret)
		gxp->power_mgr->curr_state = AUR_OFF;
	mutex_unlock(&gxp->power_mgr->pm_lock);
	return ret;
}

int gxp_pm_get_blk_state(struct gxp_dev *gxp)
{
	int ret;

	if (!gxp->power_mgr) {
		dev_err(gxp->dev, "%s: No PM found\n", __func__);
		return -ENODEV;
	}
	mutex_lock(&gxp->power_mgr->pm_lock);
	ret = gxp->power_mgr->curr_state;
	mutex_unlock(&gxp->power_mgr->pm_lock);

	return ret;
}

int gxp_pm_core_on(struct gxp_dev *gxp, uint core)
{
	int ret = 0;

	/*
	 * Check if TOP LPM is already on.
	 */
	WARN_ON(!gxp_lpm_is_initialized(gxp, LPM_TOP_PSM));

	mutex_lock(&gxp->power_mgr->pm_lock);
	ret = gxp_lpm_up(gxp, core);
	if (ret) {
		dev_err(gxp->dev, "%s: Core %d on fail\n", __func__, core);
		mutex_unlock(&gxp->power_mgr->pm_lock);
		return ret;
	}

	gxp->power_mgr->pwr_state_req[core] = gxp->power_mgr->curr_state;
	mutex_unlock(&gxp->power_mgr->pm_lock);

	dev_notice(gxp->dev, "%s: Core %d up\n", __func__, core);
	return ret;
}

int gxp_pm_core_off(struct gxp_dev *gxp, uint core)
{
	/*
	 * Check if TOP LPM is already on.
	 */
	WARN_ON(!gxp_lpm_is_initialized(gxp, LPM_TOP_PSM));

	mutex_lock(&gxp->power_mgr->pm_lock);
	gxp_lpm_down(gxp, core);
	gxp->power_mgr->pwr_state_req[core] = AUR_OFF;
	mutex_unlock(&gxp->power_mgr->pm_lock);
	/*
	 * TODO: b/199467568 If all cores are off shutdown blk
	 */
	dev_notice(gxp->dev, "%s: Core %d down\n", __func__, core);
	return 0;
}

int gxp_pm_get_core_state(struct gxp_dev *gxp, uint core)
{
	int ret;

	mutex_lock(&gxp->power_mgr->pm_lock);
	ret = gxp->power_mgr->pwr_state_req[core];
	mutex_unlock(&gxp->power_mgr->pm_lock);

	return ret;
}

int gxp_pm_req_state(struct gxp_dev *gxp, uint core, enum aur_power_state state)
{
	int i;
	unsigned long curr_max_state = AUR_OFF;

	if (core >= GXP_NUM_CORES) {
		dev_err(gxp->dev, "Invalid core num %d\n", core);
		return -EINVAL;
	}

	if (state > AUR_MAX_ALLOW_STATE) {
		dev_err(gxp->dev, "Invalid state %d\n", state);
		return -EINVAL;
	}
	mutex_lock(&gxp->power_mgr->pm_lock);
	gxp->power_mgr->pwr_state_req[core] = state;
	for (i = 0; i < GXP_NUM_CORES; i++) {
		if (gxp->power_mgr->pwr_state_req[i] >= curr_max_state)
			curr_max_state = gxp->power_mgr->pwr_state_req[i];
	}

	if (state == AUR_OFF)
		gxp_pm_core_off(gxp, core);
	if (curr_max_state != gxp->power_mgr->curr_state &&
	    curr_max_state > AUR_OFF) {
		gxp_pm_blk_set_state_acpm(gxp, curr_max_state);
		gxp->power_mgr->curr_state = curr_max_state;
	} else {
		/*
		 * TODO: b/199467568 If all cores are off shutdown blk
		 */
	}
	mutex_unlock(&gxp->power_mgr->pm_lock);

	return 0;
}

int gxp_pm_acquire_blk_wakelock(struct gxp_dev *gxp)
{
	mutex_lock(&gxp->power_mgr->pm_lock);
	refcount_inc(&(gxp->power_mgr->blk_wake_ref));
	dev_dbg(gxp->dev, "Blk wakelock ref count: %d\n",
		   refcount_read(&(gxp->power_mgr->blk_wake_ref)));
	mutex_unlock(&gxp->power_mgr->pm_lock);
	return 0;
}

int gxp_pm_release_blk_wakelock(struct gxp_dev *gxp)
{
	mutex_lock(&gxp->power_mgr->pm_lock);
	if (refcount_read(&(gxp->power_mgr->blk_wake_ref))) {
		refcount_dec(&(gxp->power_mgr->blk_wake_ref));
	} else {
		dev_err(gxp->dev, "Blk wakelock is already zero\n");
		WARN_ON(1);
		mutex_unlock(&gxp->power_mgr->pm_lock);
		return -EIO;
	}
	mutex_unlock(&gxp->power_mgr->pm_lock);
	dev_notice(gxp->dev, "Release blk wakelock\n");
	return 0;
}

int gxp_pm_init(struct gxp_dev *gxp)
{
	struct gxp_power_manager *mgr;
	int i;

	mgr = devm_kzalloc(gxp->dev, sizeof(*mgr), GFP_KERNEL);
	if (!mgr)
		return -ENOMEM;
	mgr->gxp = gxp;
	mutex_init(&mgr->pm_lock);
	mgr->curr_state = AUR_OFF;
	refcount_set(&(mgr->blk_wake_ref), 0);
	for (i = 0; i < GXP_NUM_CORES; i++)
		mgr->pwr_state_req[i] = AUR_OFF;
	mgr->ops = &gxp_aur_ops;
	gxp->power_mgr = mgr;

#if defined(CONFIG_GXP_CLOUDRIPPER) && !defined(CONFIG_GXP_TEST)
	pm_runtime_enable(gxp->dev);
#endif

	return 0;
}

int gxp_pm_destroy(struct gxp_dev *gxp)
{
	struct gxp_power_manager *mgr;

#if defined(CONFIG_GXP_CLOUDRIPPER) && !defined(CONFIG_GXP_TEST)
	pm_runtime_disable(gxp->dev);
#endif

	mgr = gxp->power_mgr;
	mutex_destroy(&mgr->pm_lock);
	return 0;
}
