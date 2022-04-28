// SPDX-License-Identifier: GPL-2.0
/*
 * GXP virtual device manager.
 *
 * Copyright (C) 2021 Google LLC
 */

#include <linux/bitops.h>
#include <linux/slab.h>

#include "gxp-dma.h"
#include "gxp-firmware.h"
#include "gxp-firmware-data.h"
#include "gxp-internal.h"
#include "gxp-mailbox.h"
#include "gxp-telemetry.h"
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

struct gxp_virtual_device *gxp_vd_allocate(struct gxp_dev *gxp,
					   u16 requested_cores)
{
	struct gxp_virtual_device *vd;
	int i;
	int err = 0;

	/* Assumes 0 < requested_cores <= GXP_NUM_CORES */
	if (requested_cores == 0 || requested_cores > GXP_NUM_CORES)
		return ERR_PTR(-EINVAL);

	vd = kzalloc(sizeof(*vd), GFP_KERNEL);
	if (!vd)
		return ERR_PTR(-ENOMEM);

	vd->gxp = gxp;
	vd->num_cores = requested_cores;

	vd->core_domains =
		kcalloc(requested_cores, sizeof(*vd->core_domains), GFP_KERNEL);
	if (!vd->core_domains) {
		err = -ENOMEM;
		goto error_free_vd;
	}
	for (i = 0; i < requested_cores; i++) {
		vd->core_domains[i] = iommu_domain_alloc(gxp->dev->bus);
		if (!vd->core_domains[i])
			goto error_free_domains;
	}

	vd->mailbox_resp_queues = kcalloc(
		vd->num_cores, sizeof(*vd->mailbox_resp_queues), GFP_KERNEL);
	if (!vd->mailbox_resp_queues) {
		err = -ENOMEM;
		goto error_free_domains;
	}

	for (i = 0; i < vd->num_cores; i++) {
		INIT_LIST_HEAD(&vd->mailbox_resp_queues[i].queue);
		spin_lock_init(&vd->mailbox_resp_queues[i].lock);
		init_waitqueue_head(&vd->mailbox_resp_queues[i].waitq);
	}

	return vd;

error_free_domains:
	for (i -= 1; i >= 0; i--)
		iommu_domain_free(vd->core_domains[i]);
	kfree(vd->core_domains);
error_free_vd:
	kfree(vd);

	return err ? ERR_PTR(err) : NULL;
}

void gxp_vd_release(struct gxp_virtual_device *vd)
{
	struct gxp_async_response *cur, *nxt;
	int i;
	unsigned long flags;

	/* Cleanup any unconsumed responses */
	for (i = 0; i < vd->num_cores; i++) {
		/*
		 * Since VD is releasing, it is not necessary to lock here.
		 * Do it anyway for consistency.
		 */
		spin_lock_irqsave(&vd->mailbox_resp_queues[i].lock, flags);
		list_for_each_entry_safe(cur, nxt,
					 &vd->mailbox_resp_queues[i].queue,
					 list_entry) {
			list_del(&cur->list_entry);
			kfree(cur);
		}
		spin_unlock_irqrestore(&vd->mailbox_resp_queues[i].lock, flags);
	}

	for (i = 0; i < vd->num_cores; i++)
		iommu_domain_free(vd->core_domains[i]);
	kfree(vd->core_domains);
	kfree(vd->mailbox_resp_queues);
	kfree(vd);
}

static void map_telemetry_buffers(struct gxp_dev *gxp,
				  struct gxp_virtual_device *vd, uint virt_core,
				  uint core)
{
	if (gxp->telemetry_mgr->logging_buff_data)
		gxp_dma_map_allocated_coherent_buffer(
			gxp,
			gxp->telemetry_mgr->logging_buff_data->buffers[core],
			vd, BIT(virt_core),
			gxp->telemetry_mgr->logging_buff_data->size,
			gxp->telemetry_mgr->logging_buff_data
				->buffer_daddrs[core],
			0);
	if (gxp->telemetry_mgr->tracing_buff_data)
		gxp_dma_map_allocated_coherent_buffer(
			gxp,
			gxp->telemetry_mgr->tracing_buff_data->buffers[core],
			vd, BIT(virt_core),
			gxp->telemetry_mgr->tracing_buff_data->size,
			gxp->telemetry_mgr->tracing_buff_data
				->buffer_daddrs[core],
			0);
}

static void unmap_telemetry_buffers(struct gxp_dev *gxp,
				    struct gxp_virtual_device *vd,
				    uint virt_core, uint core)
{
	if (gxp->telemetry_mgr->logging_buff_data)
		gxp_dma_unmap_allocated_coherent_buffer(
			gxp, vd, BIT(virt_core),
			gxp->telemetry_mgr->logging_buff_data->size,
			gxp->telemetry_mgr->logging_buff_data
				->buffer_daddrs[core]);
	if (gxp->telemetry_mgr->tracing_buff_data)
		gxp_dma_unmap_allocated_coherent_buffer(
			gxp, vd, BIT(virt_core),
			gxp->telemetry_mgr->tracing_buff_data->size,
			gxp->telemetry_mgr->tracing_buff_data
				->buffer_daddrs[core]);
}

/* Caller must hold gxp->vd_semaphore for writing */
int gxp_vd_start(struct gxp_virtual_device *vd)
{
	struct gxp_dev *gxp = vd->gxp;
	uint core;
	uint available_cores = 0;
	uint cores_remaining = vd->num_cores;
	uint core_list = 0;
	uint virt_core = 0;
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
			gxp->core_to_vd[core] = vd;
			cores_remaining--;
			gxp_dma_domain_attach_device(gxp, vd, virt_core, core);
			gxp_dma_map_core_resources(gxp, vd, virt_core, core);
			map_telemetry_buffers(gxp, vd, virt_core, core);
			ret = gxp_firmware_run(gxp, vd, virt_core, core);
			if (ret) {
				dev_err(gxp->dev, "Failed to run firmware on core %u\n",
					core);
				/*
				 * out_vd_stop will only clean up the cores that
				 * had their firmware start successfully, so we
				 * need to clean up `core` here.
				 */
				unmap_telemetry_buffers(gxp, vd, virt_core,
							core);
				gxp_dma_unmap_core_resources(gxp, vd, virt_core,
							     core);
				gxp_dma_domain_detach_device(gxp, vd,
							     virt_core);
				gxp->core_to_vd[core] = NULL;
				goto out_vd_stop;
			}
			virt_core++;
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
	uint virt_core = 0;

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
			gxp_firmware_stop(gxp, vd, virt_core, core);
			unmap_telemetry_buffers(gxp, vd, virt_core, core);
			gxp_dma_unmap_core_resources(gxp, vd, virt_core, core);
			gxp_dma_domain_detach_device(gxp, vd, virt_core);
			gxp->core_to_vd[core] = NULL;
			virt_core++;
		}
	}

	if (vd->fw_app) {
		gxp_fw_data_destroy_app(gxp, vd->fw_app);
		vd->fw_app = NULL;
	}
}

/* Caller must have locked `gxp->vd_semaphore` for reading */
int gxp_vd_virt_core_to_phys_core(struct gxp_virtual_device *vd, u16 virt_core)
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

/* Caller must have locked `gxp->vd_semaphore` for reading */
uint gxp_vd_virt_core_list_to_phys_core_list(struct gxp_virtual_device *vd,
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
		phys_core = gxp_vd_virt_core_to_phys_core(vd, virt_core);
		if (phys_core < 0)
			return 0;

		phys_core_list |= BIT(phys_core);
		virt_core_list &= ~BIT(virt_core);
	}

	return phys_core_list;
}

/* Caller must have locked `gxp->vd_semaphore` for reading */
int gxp_vd_phys_core_to_virt_core(struct gxp_virtual_device *vd,
						u16 phys_core)
{
	struct gxp_dev *gxp = vd->gxp;
	int virt_core = 0;
	uint core;

	if (gxp->core_to_vd[phys_core] != vd) {
		virt_core = -EINVAL;
		goto out;
	}

	/*
	 * A core's virtual core ID == the number of physical cores in the same
	 * virtual device with a lower physical core ID than its own.
	 */
	for (core = 0; core < phys_core; core++) {
		if (gxp->core_to_vd[core] == vd)
			virt_core++;
	}
out:
	return virt_core;
}
