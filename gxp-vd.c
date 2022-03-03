// SPDX-License-Identifier: GPL-2.0
/*
 * GXP virtual device manager.
 *
 * Copyright (C) 2021 Google LLC
 */

#include <linux/bitops.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "gxp-firmware.h"
#include "gxp-firmware-data.h"
#include "gxp-internal.h"
#include "gxp-vd.h"

int gxp_vd_init(struct gxp_dev *gxp)
{
	uint core;
	int ret;

	init_rwsem(&gxp->vd_semaphore);

	/* All cores start as free */
	for (core = 0; core < GXP_NUM_CORES; core++)
		gxp->core_to_vd[core] = NULL;

	ret = gxp_fw_init(gxp);

	return ret;
}

void gxp_vd_destroy(struct gxp_dev *gxp)
{
	down_write(&gxp->vd_semaphore);

	gxp_fw_destroy(gxp);

	up_write(&gxp->vd_semaphore);
}

struct gxp_virtual_device *gxp_vd_allocate(struct gxp_dev *gxp, u16 requested_cores)
{
	struct gxp_virtual_device *vd;

	/* Assumes 0 < requested_cores <= GXP_NUM_CORES */
	if (requested_cores == 0 || requested_cores > GXP_NUM_CORES)
		return ERR_PTR(-EINVAL);

	vd = kzalloc(sizeof(*vd), GFP_KERNEL);
	if (!vd)
		return ERR_PTR(-ENOMEM);

	vd->gxp = gxp;
	vd->num_cores = requested_cores;

	/*
	 * TODO(b/209083969) Initialize VD aux domain here to support VD
	 * suspend/resume and mapping without a VIRTUAL_DEVICE wakelock.
	 */

	return vd;
}

void gxp_vd_release(struct gxp_virtual_device *vd)
{
	/*
	 * TODO(b/209083969) Cleanup VD aux domain once it's created in
	 * gxp_vd_allocate().
	 */

	kfree(vd);
}

/* Caller must hold gxp->vd_semaphore for writing */
int gxp_vd_start(struct gxp_virtual_device *vd)
{
	struct gxp_dev *gxp = vd->gxp;
	uint core;
	uint available_cores = 0;
	uint cores_remaining = vd->num_cores;
	uint core_list = 0;
	int ret = 0;

	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (gxp->core_to_vd[core] == NULL) {
			if (available_cores < vd->num_cores)
				core_list |= BIT(core);
			available_cores++;
		}
	}

	if (available_cores < vd->num_cores) {
		dev_err(gxp->dev, "Insufficient available cores. Available: %u. Requested: %u\n",
			available_cores, vd->num_cores);
		return -EBUSY;
	}

	vd->fw_app = gxp_fw_data_create_app(gxp, core_list);

	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (cores_remaining == 0)
			break;

		if (core_list & BIT(core)) {
			ret = gxp_firmware_run(gxp, core);
			if (ret) {
				dev_err(gxp->dev, "Failed to run firmware on core %u\n",
					core);
				goto out_vd_stop;
			}
			gxp->core_to_vd[core] = vd;
			cores_remaining--;
		}
	}

	if (cores_remaining != 0) {
		dev_err(gxp->dev,
			"Internal error: Failed to start %u requested cores. %u cores remaining\n",
			vd->num_cores, cores_remaining);
		/*
		 * Should never reach here. Previously verified that enough
		 * cores are available.
		 */
		WARN_ON(true);
		ret = -EIO;
		goto out_vd_stop;
	}

	return ret;

out_vd_stop:
	gxp_vd_stop(vd);
	return ret;

}

/* Caller must hold gxp->vd_semaphore for writing */
void gxp_vd_stop(struct gxp_virtual_device *vd)
{
	struct gxp_dev *gxp = vd->gxp;
	uint core;

	/*
	 * Put all cores in the VD into reset so they can not wake each other up
	 */
	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (gxp->core_to_vd[core] == vd) {
			gxp_write_32_core(
				gxp, core, GXP_REG_ETM_PWRCTL,
				1 << GXP_REG_ETM_PWRCTL_CORE_RESET_SHIFT);
		}
	}

	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (gxp->core_to_vd[core] == vd) {
			gxp->core_to_vd[core] = NULL;
			gxp_firmware_stop(gxp, core);
		}
	}

	if (vd->fw_app) {
		gxp_fw_data_destroy_app(gxp, vd->fw_app);
		vd->fw_app = NULL;
	}
}

/*
 * Helper function for use in both `gxp_vd_virt_core_to_phys_core()` and
 * `gxp_vd_virt_core_list_to_phys_core_list()`.
 *
 * Caller must have locked `gxp->vd_semaphore` for reading.
 */
static int virt_core_to_phys_core_locked(struct gxp_virtual_device *vd,
					 u16 virt_core)
{
	struct gxp_dev *gxp = vd->gxp;
	uint phys_core;
	uint virt_core_index = 0;

	for (phys_core = 0; phys_core < GXP_NUM_CORES; phys_core++) {
		if (gxp->core_to_vd[phys_core] == vd) {
			if (virt_core_index == virt_core) {
				/* Found virtual core */
				return phys_core;
			}

			virt_core_index++;
		}
	}

	dev_dbg(gxp->dev, "No mapping for virtual core %u\n", virt_core);
	return -EINVAL;
}

int gxp_vd_virt_core_to_phys_core(struct gxp_virtual_device *vd, u16 virt_core)
{
	struct gxp_dev *gxp = vd->gxp;
	int ret;

	down_read(&gxp->vd_semaphore);
	ret = virt_core_to_phys_core_locked(vd, virt_core);
	up_read(&gxp->vd_semaphore);

	return ret;
}

uint gxp_vd_virt_core_list_to_phys_core_list(struct gxp_virtual_device *vd,
					     u16 virt_core_list)
{
	struct gxp_dev *gxp = vd->gxp;
	uint phys_core_list = 0;
	uint virt_core = 0;
	int phys_core;

	down_read(&gxp->vd_semaphore);

	while (virt_core_list) {
		/*
		 * Get the next virt core by finding the index of the first
		 * set bit in the core list.
		 *
		 * Subtract 1 since `ffs()` returns a 1-based index. Since
		 * virt_core_list cannot be 0 at this point, no need to worry
		 * about wrap-around.
		 */
		virt_core = ffs(virt_core_list) - 1;

		/* Any invalid virt cores invalidate the whole list */
		phys_core = virt_core_to_phys_core_locked(vd, virt_core);
		if (phys_core < 0) {
			phys_core_list = 0;
			goto out;
		}

		phys_core_list |= BIT(phys_core);
		virt_core_list &= ~BIT(virt_core);
	}

out:
	up_read(&gxp->vd_semaphore);

	return phys_core_list;
}
