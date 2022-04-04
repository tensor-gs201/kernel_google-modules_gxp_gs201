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
#include <linux/workqueue.h>

#ifdef CONFIG_GXP_CLOUDRIPPER
#include <linux/acpm_dvfs.h>
#endif
#include <soc/google/exynos_pm_qos.h>

#include "gxp-bpm.h"
#include "gxp-doorbell.h"
#include "gxp-internal.h"
#include "gxp-lpm.h"
#include "gxp-pm.h"

/*
 * The order of this array decides the voting priority, should be increasing in
 * frequencies.
 */
static const enum aur_power_state aur_state_array[] = { AUR_OFF, AUR_READY,
							AUR_UUD, AUR_SUD,
							AUR_UD,	 AUR_NOM };
static const uint aur_memory_state_array[] = {
	AUR_MEM_UNDEFINED, AUR_MEM_MIN,	      AUR_MEM_VERY_LOW, AUR_MEM_LOW,
	AUR_MEM_HIGH,	   AUR_MEM_VERY_HIGH, AUR_MEM_MAX
};

/*
 * TODO(b/177692488): move frequency values into chip-specific config.
 * TODO(b/221168126): survey how these value are derived from. Below
 * values are copied from the implementation in TPU firmware for PRO,
 * i.e. google3/third_party/darwinn/firmware/janeiro/power_manager.cc.
 */
static const s32 aur_memory_state2int_table[] = { 0,	  0,	  0,	 200000,
						  332000, 465000, 533000 };
static const s32 aur_memory_state2mif_table[] = { 0,	   0,	    0,
						  1014000, 1352000, 2028000,
						  3172000 };

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
	/* Remove our vote for INT/MIF state (if any) */
	exynos_pm_qos_update_request(&gxp->power_mgr->int_min, 0);
	exynos_pm_qos_update_request(&gxp->power_mgr->mif_min, 0);
	return ret;
}

static int gxp_pm_blk_set_state_acpm(struct gxp_dev *gxp, unsigned long state, bool aggressor)
{
	unsigned long rate;

	rate = aur_power_state2rate[state];
	if (!aggressor)
		rate |= BIT(AUR_NON_AGGRESSOR_BIT);
	return gxp_pm_blk_set_rate_acpm(gxp, rate);
}

int gxp_pm_blk_set_rate_acpm(struct gxp_dev *gxp, unsigned long rate)
{
	int ret = 0;

#if defined(CONFIG_GXP_CLOUDRIPPER)
	ret = exynos_acpm_set_rate(AUR_DVFS_DOMAIN, rate);
	dev_dbg(gxp->dev, "%s: rate %lu, ret %d\n", __func__, rate, ret);
#endif
	return ret;
}

static void set_cmu_mux_state(struct gxp_dev *gxp, u32 val)
{
	writel(val << 4, gxp->cmu.vaddr + PLL_CON0_PLL_AUR);
	writel(val << 4, gxp->cmu.vaddr + PLL_CON0_NOC_USER);
}

static void gxp_pm_blk_set_state_acpm_async(struct work_struct *work)
{
	struct gxp_set_acpm_state_work *set_acpm_state_work =
		container_of(work, struct gxp_set_acpm_state_work, work);

	mutex_lock(&set_acpm_state_work->gxp->power_mgr->pm_lock);
	/*
	 * This prev_state may be out of date with the manager's current state,
	 * but we don't need curr_state here. curr_state is the last scheduled
	 * state, while prev_state was the last actually requested state. It's
	 * true because all request are executed synchronously and executed in
	 * FIFO order.
	 */
	if (set_acpm_state_work->prev_state == AUR_READY)
		set_cmu_mux_state(set_acpm_state_work->gxp, AUR_CMU_MUX_NORMAL);
	else if (set_acpm_state_work->state == AUR_READY)
		set_cmu_mux_state(set_acpm_state_work->gxp, AUR_CMU_MUX_LOW);
	gxp_pm_blk_set_state_acpm(set_acpm_state_work->gxp,
				  set_acpm_state_work->state,
				  set_acpm_state_work->aggressor_vote);
	mutex_unlock(&set_acpm_state_work->gxp->power_mgr->pm_lock);
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
		gxp_pm_blk_set_state_acpm(gxp, AUR_INIT_DVFS_STATE,
					  true /*aggressor*/);
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
	if (gxp->power_mgr->curr_state == AUR_OFF) {
		mutex_unlock(&gxp->power_mgr->pm_lock);
		return ret;
	}
	/*
	 * Before the block is off, CMUMUX cannot be low. Otherwise, powering on
	 * cores will fail later.
	 */
	if (gxp->power_mgr->curr_state == AUR_READY)
		set_cmu_mux_state(gxp, AUR_CMU_MUX_NORMAL);

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
	mutex_unlock(&gxp->power_mgr->pm_lock);
	/*
	 * TODO: b/199467568 If all cores are off shutdown blk
	 */
	dev_notice(gxp->dev, "%s: Core %d down\n", __func__, core);
	return 0;
}

static int gxp_pm_req_state_locked(struct gxp_dev *gxp,
				   enum aur_power_state state,
				   bool aggressor_vote)
{
	if (state > AUR_MAX_ALLOW_STATE) {
		dev_err(gxp->dev, "Invalid state %d\n", state);
		return -EINVAL;
	}
	if (state != gxp->power_mgr->curr_state ||
	    aggressor_vote != gxp->power_mgr->curr_aggressor_vote) {
		if (state == AUR_OFF) {
			dev_warn(gxp->dev,
				 "It is not supported to request AUR_OFF\n");
		} else {
			gxp->power_mgr->set_acpm_state_work.gxp = gxp;
			gxp->power_mgr->set_acpm_state_work.state = state;
			gxp->power_mgr->set_acpm_state_work.aggressor_vote =
				aggressor_vote;
			gxp->power_mgr->set_acpm_state_work.prev_state =
				gxp->power_mgr->curr_state;
			queue_work(gxp->power_mgr->wq,
				   &gxp->power_mgr->set_acpm_state_work.work);
		}
		gxp->power_mgr->curr_state = state;
		gxp->power_mgr->curr_aggressor_vote = aggressor_vote;
	}

	return 0;
}

/* Caller must hold pm_lock */
static void gxp_pm_revoke_power_state_vote(struct gxp_dev *gxp,
					   enum aur_power_state revoked_state,
					   bool origin_requested_aggressor)
{
	unsigned int i;
	uint *pwr_state_req_count;

	if (revoked_state == AUR_OFF)
		return;
	if (origin_requested_aggressor)
		pwr_state_req_count = gxp->power_mgr->pwr_state_req_count;
	else
		pwr_state_req_count =
			gxp->power_mgr->non_aggressor_pwr_state_req_count;

	for (i = 0; i < AUR_NUM_POWER_STATE; i++) {
		if (aur_state_array[i] == revoked_state) {
			if (pwr_state_req_count[i] == 0)
				dev_err(gxp->dev, "Invalid state %d\n",
					revoked_state);
			else
				pwr_state_req_count[i]--;
			return;
		}
	}
}

/* Caller must hold pm_lock */
static void gxp_pm_vote_power_state(struct gxp_dev *gxp,
				    enum aur_power_state state,
				    bool requested_aggressor)
{
	unsigned int i;
	uint *pwr_state_req_count;

	if (state == AUR_OFF)
		return;
	if (requested_aggressor)
		pwr_state_req_count = gxp->power_mgr->pwr_state_req_count;
	else
		pwr_state_req_count =
			gxp->power_mgr->non_aggressor_pwr_state_req_count;

	for (i = 0; i < AUR_NUM_POWER_STATE; i++) {
		if (aur_state_array[i] == state) {
			pwr_state_req_count[i]++;
			return;
		}
	}
}

/* Caller must hold pm_lock */
static void gxp_pm_get_max_voted_power_state(struct gxp_dev *gxp,
					     unsigned long *state,
					     bool *aggressor_vote)
{
	int i;

	*state = AUR_OFF;
	for (i = AUR_NUM_POWER_STATE - 1; i >= 0; i--) {
		if (gxp->power_mgr->pwr_state_req_count[i] > 0) {
			*aggressor_vote = true;
			*state = aur_state_array[i];
			break;
		}
	}
	if (*state == AUR_OFF) {
		/* No aggressor vote, check non-aggressor vote counts */
		*aggressor_vote = false;
		for (i = AUR_NUM_POWER_STATE - 1; i >= 0; i--) {
			if (gxp->power_mgr->non_aggressor_pwr_state_req_count[i] > 0) {
				*state = aur_state_array[i];
				break;
			}
		}
	}
}

int gxp_pm_update_requested_power_state(struct gxp_dev *gxp,
					enum aur_power_state origin_state,
					bool origin_requested_aggressor,
					enum aur_power_state requested_state,
					bool requested_aggressor)
{
	int ret;
	unsigned long max_state = AUR_OFF;
	bool aggressor_vote = false;

	mutex_lock(&gxp->power_mgr->pm_lock);
	gxp_pm_revoke_power_state_vote(gxp, origin_state, origin_requested_aggressor);
	gxp_pm_vote_power_state(gxp, requested_state, requested_aggressor);
	gxp_pm_get_max_voted_power_state(gxp, &max_state, &aggressor_vote);
	ret = gxp_pm_req_state_locked(gxp, max_state, aggressor_vote);
	mutex_unlock(&gxp->power_mgr->pm_lock);
	return ret;
}

static int gxp_pm_req_pm_qos(struct gxp_dev *gxp, s32 int_val, s32 mif_val)
{
	exynos_pm_qos_update_request(&gxp->power_mgr->int_min, int_val);
	exynos_pm_qos_update_request(&gxp->power_mgr->mif_min, mif_val);
	return 0;
}

static void gxp_pm_req_pm_qos_async(struct work_struct *work)
{
	struct gxp_req_pm_qos_work *req_pm_qos_work =
		container_of(work, struct gxp_req_pm_qos_work, work);

	mutex_lock(&req_pm_qos_work->gxp->power_mgr->pm_lock);
	gxp_pm_req_pm_qos(req_pm_qos_work->gxp, req_pm_qos_work->int_val,
			  req_pm_qos_work->mif_val);
	mutex_unlock(&req_pm_qos_work->gxp->power_mgr->pm_lock);
}

static int gxp_pm_req_memory_state_locked(struct gxp_dev *gxp, enum aur_memory_power_state state)
{
	s32 int_val = 0, mif_val = 0;

	if (state > AUR_MAX_ALLOW_MEMORY_STATE) {
		dev_err(gxp->dev, "Invalid memory state %d\n", state);
		return -EINVAL;
	}
	if (state != gxp->power_mgr->curr_memory_state) {
		gxp->power_mgr->curr_memory_state = state;
		int_val = aur_memory_state2int_table[state];
		mif_val = aur_memory_state2mif_table[state];
		gxp->power_mgr->req_pm_qos_work.gxp = gxp;
		gxp->power_mgr->req_pm_qos_work.int_val = int_val;
		gxp->power_mgr->req_pm_qos_work.mif_val = mif_val;
		queue_work(gxp->power_mgr->wq,
			   &gxp->power_mgr->req_pm_qos_work.work);
	}

	return 0;
}

/* Caller must hold pm_lock */
static void
gxp_pm_revoke_memory_power_state_vote(struct gxp_dev *gxp,
				      enum aur_memory_power_state revoked_state)
{
	unsigned int i;

	if (revoked_state == AUR_MEM_UNDEFINED)
		return;
	for (i = 0; i < AUR_NUM_MEMORY_POWER_STATE; i++) {
		if (aur_memory_state_array[i] == revoked_state) {
			if (gxp->power_mgr->mem_pwr_state_req_count[i] == 0)
				dev_err_ratelimited(
					gxp->dev,
					"Invalid memory state %d with zero count\n",
					revoked_state);
			else
				gxp->power_mgr->mem_pwr_state_req_count[i]--;
			return;
		}
	}
}

/* Caller must hold pm_lock */
static void gxp_pm_vote_memory_power_state(struct gxp_dev *gxp,
				    enum aur_memory_power_state state)
{
	unsigned int i;

	if (state == AUR_MEM_UNDEFINED)
		return;
	for (i = 0; i < AUR_NUM_MEMORY_POWER_STATE; i++) {
		if (aur_memory_state_array[i] == state) {
			gxp->power_mgr->mem_pwr_state_req_count[i]++;
			return;
		}
	}
}

/* Caller must hold pm_lock */
static unsigned long gxp_pm_get_max_voted_memory_power_state(struct gxp_dev *gxp)
{
	int i;
	unsigned long state = AUR_MEM_UNDEFINED;

	for (i = AUR_NUM_MEMORY_POWER_STATE - 1; i >= 0; i--) {
		if (gxp->power_mgr->mem_pwr_state_req_count[i] > 0) {
			state = aur_memory_state_array[i];
			break;
		}
	}
	return state;
}

int gxp_pm_update_requested_memory_power_state(
	struct gxp_dev *gxp, enum aur_memory_power_state origin_state,
	enum aur_memory_power_state requested_state)
{
	int ret;
	unsigned long max_state;

	mutex_lock(&gxp->power_mgr->pm_lock);
	gxp_pm_revoke_memory_power_state_vote(gxp, origin_state);
	gxp_pm_vote_memory_power_state(gxp, requested_state);
	max_state = gxp_pm_get_max_voted_memory_power_state(gxp);
	ret = gxp_pm_req_memory_state_locked(gxp, max_state);
	mutex_unlock(&gxp->power_mgr->pm_lock);
	return ret;
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

	mgr = devm_kzalloc(gxp->dev, sizeof(*mgr), GFP_KERNEL);
	if (!mgr)
		return -ENOMEM;
	mgr->gxp = gxp;
	mutex_init(&mgr->pm_lock);
	mgr->curr_state = AUR_OFF;
	mgr->curr_memory_state = AUR_MEM_UNDEFINED;
	mgr->curr_aggressor_vote = true;
	refcount_set(&(mgr->blk_wake_ref), 0);
	mgr->ops = &gxp_aur_ops;
	gxp->power_mgr = mgr;
	INIT_WORK(&mgr->set_acpm_state_work.work, gxp_pm_blk_set_state_acpm_async);
	INIT_WORK(&mgr->req_pm_qos_work.work, gxp_pm_req_pm_qos_async);
	gxp->power_mgr->wq =
		create_singlethread_workqueue("gxp_power_work_queue");

#if defined(CONFIG_GXP_CLOUDRIPPER) && !defined(CONFIG_GXP_TEST)
	pm_runtime_enable(gxp->dev);
#endif
	exynos_pm_qos_add_request(&mgr->int_min, PM_QOS_DEVICE_THROUGHPUT, 0);
	exynos_pm_qos_add_request(&mgr->mif_min, PM_QOS_BUS_THROUGHPUT, 0);

	return 0;
}

int gxp_pm_destroy(struct gxp_dev *gxp)
{
	struct gxp_power_manager *mgr;

	mgr = gxp->power_mgr;
	exynos_pm_qos_remove_request(&mgr->int_min);
	exynos_pm_qos_remove_request(&mgr->mif_min);
#if defined(CONFIG_GXP_CLOUDRIPPER) && !defined(CONFIG_GXP_TEST)
	pm_runtime_disable(gxp->dev);
#endif
	destroy_workqueue(mgr->wq);
	mutex_destroy(&mgr->pm_lock);
	return 0;
}
