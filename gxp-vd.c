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

#include "gxp-dma.h"
#include "gxp-firmware.h"
#include "gxp-firmware-data.h"
#include "gxp-internal.h"
#include "gxp-vd.h"

int gxp_vd_init(struct gxp_dev *gxp)
{
	uint core;
	int ret;

	mutex_init(&gxp->vd_lock);
	mutex_lock(&gxp->vd_lock);

	/* Mark all cores as free */
	for (core = 0; core < GXP_NUM_CORES; core++)
		gxp->core_to_client[core] = NULL;

	ret = gxp_fw_init(gxp);
	mutex_unlock(&gxp->vd_lock);
	return ret;
}

void gxp_vd_destroy(struct gxp_dev *gxp)
{
	mutex_lock(&gxp->vd_lock);

	gxp_fw_destroy(gxp);

	mutex_unlock(&gxp->vd_lock);
}

/* Caller must hold gxp->vd_lock */
static void gxp_vd_release(struct gxp_client *client)
{
	uint core;
	struct gxp_dev *gxp = client->gxp;

	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (gxp->core_to_client[core] == client) {
			gxp->core_to_client[core] = NULL;
			gxp_firmware_stop(gxp, core);
		}
	}
	if (client->app) {
		gxp_fw_data_destroy_app(gxp, client->app);
		client->app = NULL;
	}
}

/* Caller must hold gxp->vd_lock */
int gxp_vd_allocate(struct gxp_client *client, u16 requested_cores)
{
	struct gxp_dev *gxp = client->gxp;
	uint core;
	int available_cores = 0;
	int cores_remaining = requested_cores;
	uint core_list = 0;
	int ret = 0;

	/* Assumes 0 < requested_cores <= GXP_NUM_CORES */
	WARN_ON(requested_cores == 0 || requested_cores > GXP_NUM_CORES);
	/* Assumes client has not called gxp_vd_allocate */
	WARN_ON(client->vd_allocated);

	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (gxp->core_to_client[core] == NULL) {
			if (available_cores < requested_cores)
				core_list |= BIT(core);
			available_cores++;
		}
	}

	if (available_cores < requested_cores) {
		dev_err(gxp->dev, "Insufficient available cores. Available: %d. Requested: %u\n",
			available_cores, requested_cores);
		return -EBUSY;
	}

	client->app = gxp_fw_data_create_app(gxp, core_list);

	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (cores_remaining == 0)
			break;

		if (core_list & BIT(core)) {
			ret = gxp_firmware_run(gxp, core);
			if (ret) {
				dev_err(gxp->dev, "Failed to run firmware on core %u\n",
					core);
				goto out_vd_release;
			}
			gxp->core_to_client[core] = client;
			cores_remaining--;
		}
	}

	if (cores_remaining != 0) {
		dev_err(gxp->dev, "Internal error: Failed to allocate %u requested cores. %d cores remaining\n",
			requested_cores, cores_remaining);
		/*
		 * Should never reach here. Previously verified that enough
		 * cores are available.
		 */
		WARN_ON(true);
		ret = -EIO;
		goto out_vd_release;
	}

	client->vd_allocated = true;
	return ret;

out_vd_release:
	gxp_vd_release(client);
	return ret;
}

int gxp_vd_virt_core_to_phys_core(struct gxp_client *client, u16 virt_core)
{
	struct gxp_dev *gxp = client->gxp;
	uint phys_core;
	uint virt_core_index = 0;

	mutex_lock(&gxp->vd_lock);

	if (!client->vd_allocated) {
		mutex_unlock(&gxp->vd_lock);
		dev_dbg(gxp->dev, "Client has not allocated a virtual device\n");
		return -EINVAL;
	}

	for (phys_core = 0; phys_core < GXP_NUM_CORES; phys_core++) {
		if (gxp->core_to_client[phys_core] == client) {
			if (virt_core_index == virt_core) {
				/* Found virtual core */
				mutex_unlock(&gxp->vd_lock);
				return phys_core;
			}

			virt_core_index++;
		}
	}

	mutex_unlock(&gxp->vd_lock);
	dev_dbg(gxp->dev, "No mapping for virtual core %u\n", virt_core);
	return -EINVAL;
}

uint gxp_vd_virt_core_list_to_phys_core_list(struct gxp_client *client,
					     u16 virt_core_list)
{
	uint phys_core_list = 0;
	uint virt_core = 0;
	int phys_core;

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
		phys_core = gxp_vd_virt_core_to_phys_core(client, virt_core);
		if (phys_core < 0)
			return 0;

		phys_core_list |= BIT(phys_core);
		virt_core_list &= ~BIT(virt_core);
	}

	return phys_core_list;
}

struct gxp_client *gxp_client_create(struct gxp_dev *gxp)
{
	struct gxp_client *client;

	client = kmalloc(sizeof(*client), GFP_KERNEL);
	if (!client)
		return ERR_PTR(-ENOMEM);

	client->gxp = gxp;
	client->vd_allocated = false;
	client->app = NULL;
	client->tpu_mbx_allocated = false;
	return client;
}

void gxp_client_destroy(struct gxp_client *client)
{
	struct gxp_dev *gxp = client->gxp;

	mutex_lock(&gxp->vd_lock);

#ifdef CONFIG_ANDROID
	/*
	 * Unmap TPU buffers, if the mapping is already removed, this
	 * is a no-op.
	 */
	gxp_dma_unmap_tpu_buffer(gxp, client->mbx_desc);
#endif  // CONFIG_ANDROID
	gxp_vd_release(client);

	mutex_unlock(&gxp->vd_lock);

	kfree(client);
}
