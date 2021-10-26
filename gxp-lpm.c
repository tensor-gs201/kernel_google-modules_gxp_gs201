// SPDX-License-Identifier: GPL-2.0
/*
 * GXP power management interface.
 *
 * Copyright (C) 2021 Google LLC
 */

#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/types.h>
#include <linux/pm_runtime.h>

#ifdef CONFIG_GXP_CLOUDRIPPER
#include <linux/acpm_dvfs.h>
#endif

#include "gxp-bpm.h"
#include "gxp-doorbell.h"
#include "gxp-internal.h"
#include "gxp-lpm.h"
#include "gxp-tmp.h"

static void enable_state(struct gxp_dev *gxp, uint psm, uint state)
{
	uint offset = LPM_REG_ENABLE_STATE_0 + (LPM_STATE_TABLE_SIZE * state);

	/* PS0 should always be enabled */
	WARN_ON(state == 0);

	/* Disable all low power states */
	lpm_write_32_psm(gxp, psm, LPM_REG_ENABLE_STATE_1, 0x0);
	lpm_write_32_psm(gxp, psm, LPM_REG_ENABLE_STATE_2, 0x0);
	lpm_write_32_psm(gxp, psm, LPM_REG_ENABLE_STATE_3, 0x0);

	/* Enable the requested low power state */
	lpm_write_32_psm(gxp, psm, offset, 0x1);
}

static bool is_initialized(struct gxp_dev *gxp, uint psm)
{
	u32 status = lpm_read_32_psm(gxp, psm, PSM_STATUS_OFFSET);

	/*
	 * state_valid bit goes active and stays high forever the first time you
	 * write the start register
	 */
	if (status & PSM_STATE_VALID_MASK)
		return true;

	return false;
}

static uint get_state(struct gxp_dev *gxp, uint psm)
{
	u32 status = lpm_read_32_psm(gxp, psm, PSM_STATUS_OFFSET);

	return status & PSM_CURR_STATE_MASK;
}

int gxp_blk_set_state(struct gxp_dev *gxp, unsigned long state)
{
	int ret = 0;

#ifdef CONFIG_GXP_CLOUDRIPPER
	ret = exynos_acpm_set_rate(AUR_DVFS_DOMAIN, state);
	dev_dbg(gxp->dev, "%s: state %lu, ret %d\n", __func__, state, ret);
#endif
	return ret;
}

int gxp_blk_get_state(struct gxp_dev *gxp)
{
	int ret = 0;

#ifdef CONFIG_GXP_CLOUDRIPPER
	ret = exynos_acpm_get_rate(AUR_DVFS_DOMAIN, AUR_DEBUG_CORE_FREQ);
	dev_dbg(gxp->dev, "%s: state %d\n", __func__, ret);
#endif
	return ret;
}

static int set_state_internal(struct gxp_dev *gxp, uint psm, uint target_state)
{
	u32 val;
	int i = 10000;

	/* Set SW sequencing mode and PS target */
	val = LPM_SW_PSM_MODE;
	val |= target_state << LPM_CFG_SW_PS_TARGET_OFFSET;
	lpm_write_32_psm(gxp, psm, PSM_CFG_OFFSET, val);

	/* Start the SW sequence */
	lpm_write_32_psm(gxp, psm, PSM_START_OFFSET, 0x1);

	/* Wait for LPM init done (0x60041688) */
	while (i && !(lpm_read_32_psm(gxp, psm, PSM_STATUS_OFFSET)
		      & PSM_INIT_DONE_MASK)) {
		cpu_relax();
		i--;
	}

	if (!i) {
		dev_err(gxp->dev, "Failed to switch to PS%u\n", target_state);
		return -EIO;
	}

	return 0;
}

static int set_state(struct gxp_dev *gxp, uint psm, uint target_state)
{
	uint curr_state = get_state(gxp, psm);

	if (curr_state == target_state)
		return 0;

	dev_warn(gxp->dev, "Forcing a transition to PS%u on core%u, status: %x\n",
		 target_state, psm,
		 lpm_read_32_psm(gxp, psm, PSM_STATUS_OFFSET));

	enable_state(gxp, psm, target_state);

	if ((curr_state != LPM_ACTIVE_STATE)
	    && (target_state != LPM_ACTIVE_STATE)) {
		/* Switch to PS0 before switching to a low power state. */
		set_state_internal(gxp, psm, LPM_ACTIVE_STATE);
	}

	set_state_internal(gxp, psm, target_state);

	dev_warn(gxp->dev, "Finished forced transition on core %u.  target: PS%u, actual: PS%u, status: %x\n",
		 psm, target_state, get_state(gxp, psm),
		 lpm_read_32_psm(gxp, psm, PSM_STATUS_OFFSET));

	/* Set HW sequencing mode */
	lpm_write_32_psm(gxp, psm, PSM_CFG_OFFSET, LPM_HW_MODE);

	return 0;
}

static int psm_enable(struct gxp_dev *gxp, uint psm)
{
	int i = 10000;

	/* Return early if LPM is already initialized */
	if (is_initialized(gxp, psm)) {
		if (psm != LPM_TOP_PSM) {
			/* Ensure core is in PS2 */
			return set_state(gxp, psm, LPM_PG_W_RET_STATE);
		}

		return 0;
	}

	/* Write PSM start bit */
	lpm_write_32_psm(gxp, psm, PSM_START_OFFSET, PSM_START);
	msleep(20 * GXP_TIME_DELAY_FACTOR);

	/* Wait for LPM init done (0x60041688) */
	while (i && !(lpm_read_32_psm(gxp, psm, PSM_STATUS_OFFSET)
		      & PSM_INIT_DONE_MASK)) {
		cpu_relax();
		i--;
	}

	if (!i)
		return 1;

	/* Set PSM to HW mode (0x60041680) */
	lpm_write_32_psm(gxp, psm, PSM_CFG_OFFSET, PSM_HW_MODE);

	return 0;
}

void gxp_lpm_init(struct gxp_dev *gxp)
{
	u32 val;

	/*
	 * Some LPM signals are not looped back in the current FPGA
	 * implementation, causing the PSM to time out waiting for a handshake
	 * signal from the host.
	 * TODO: This is to be fixed in the next version of FPGA build
	 * WORKAROUND: Patch LPM instruction to bypass the timeout for now
	 * FIXME: The patch is only for ML2.5 build, and is incompatible to
	 * other builds
	 */
	val = lpm_read_32(gxp, LPM_INSTRUCTION_OFFSET);
	val &= (~LPM_INSTRUCTION_MASK);
	lpm_write_32(gxp, LPM_INSTRUCTION_OFFSET, val);

	/* Local Access Path should not be enabled */
#if 0
	/*
	 * Enable CNOC to DNOC path in Provino for direct TOP access from Q7
	 * cores.
	 */
	val = gxp_read_32(gxp, PROVINO_IXBAR1_ARL_CTRL);
	val |= PROVINO_IXBAR1_ARL_EN;
	gxp_write_32(gxp, PROVINO_IXBAR1_ARL_CTRL, val);
#endif

	/* Enable Top PSM */
	dev_notice(gxp->dev, "Enabling Top PSM...\n");
	if (psm_enable(gxp, LPM_TOP_PSM)) {
		dev_notice(gxp->dev, "Timed out!\n");
		return;
	}
	dev_notice(gxp->dev, "Enabled\n");
}

void gxp_lpm_destroy(struct gxp_dev *gxp)
{
	/* (b/171063370) Put Top PSM in ACTIVE state before block shutdown */
	dev_notice(gxp->dev, "Kicking Top PSM out of ACG\n");

	/* Disable all low-power states for TOP */
	lpm_write_32_psm(gxp, LPM_TOP_PSM, LPM_REG_ENABLE_STATE_1, 0x0);
	lpm_write_32_psm(gxp, LPM_TOP_PSM, LPM_REG_ENABLE_STATE_2, 0x0);
}

int gxp_lpm_up(struct gxp_dev *gxp, uint core)
{
	/* Clear wakeup doorbell */
	gxp_doorbell_clear(gxp, CORE_WAKEUP_DOORBELL);

	/* Enable core PSM */
	dev_notice(gxp->dev, "Enabling Core%u PSM...\n", core);
	if (psm_enable(gxp, core)) {
		dev_notice(gxp->dev, "Timed out!\n");
		return 0;
	}
	dev_notice(gxp->dev, "Enabled\n");

	/* Enable PS1 (Clk Gated) */
	enable_state(gxp, core, LPM_CG_STATE);

	gxp_bpm_start(gxp, core);

#ifdef CONFIG_GXP_USE_SW_MAILBOX
	/*
	 * Enable doorbells [28-31] for SW mailbox.
	 * TODO (b/182526648): Enable doorbells required for SW mailbox in the
	 * driver's alloc function.
	 */
	gxp_write_32_core(gxp, core, GXP_REG_COMMON_INT_MASK_0, BIT(31 - core));
#endif  // CONFIG_GXP_USE_SW_MAILBOX

	return 0;
}

void gxp_lpm_down(struct gxp_dev *gxp, uint core)
{
	/* Enable PS2 (Pwr Gated w/Ret) */
	enable_state(gxp, core, LPM_PG_W_RET_STATE);

	/* Set wakeup doorbell to trigger an automatic transition to PS2 */
	gxp_doorbell_set_listening_core(gxp, CORE_WAKEUP_DOORBELL, core);
	gxp_doorbell_set(gxp, CORE_WAKEUP_DOORBELL);
	msleep(25 * GXP_TIME_DELAY_FACTOR);

	/* Reset doorbell mask */
	gxp_write_32_core(gxp, core, GXP_REG_COMMON_INT_MASK_0, 0);

	/* Ensure core is in PS2 */
	set_state(gxp, core, LPM_PG_W_RET_STATE);
}
