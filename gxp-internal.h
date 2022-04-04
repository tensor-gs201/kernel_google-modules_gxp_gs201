/* SPDX-License-Identifier: GPL-2.0 */
/*
 * GXP driver common internal definitions.
 *
 * Copyright (C) 2021 Google LLC
 */
#ifndef __GXP_INTERNAL_H__
#define __GXP_INTERNAL_H__

#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iommu.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/rwsem.h>
#include <linux/spinlock.h>

#include "gxp-config.h"
#include "gxp-tmp.h"

/* Holds Client's TPU mailboxes info used during mapping */
struct gxp_tpu_mbx_desc {
	uint phys_core_list;
	size_t cmdq_size, respq_size;
};

/* ioremapped resource */
struct gxp_mapped_resource {
	void __iomem *vaddr;		 /* starting virtual address */
	phys_addr_t paddr;		 /* starting physical address */
	dma_addr_t daddr;		 /* starting device address */
	resource_size_t size;		 /* size in bytes */
};

struct mailbox_resp_list {
	struct list_head list;
	struct gxp_response *resp;
};

/* Structure to hold TPU device info */
struct gxp_tpu_dev {
	struct device *dev;
	phys_addr_t mbx_paddr;
};

/* Forward declarations from submodules */
struct gxp_client;
struct gxp_mailbox_manager;
struct gxp_debug_dump_manager;
struct gxp_mapping_root;
struct gxp_dma_manager;
struct gxp_fw_data_manager;
struct gxp_power_manager;
struct gxp_telemetry_manager;
struct gxp_thermal_manager;
struct gxp_wakelock_manager;

struct gxp_dev {
	struct device *dev;		 /* platform bus device */
	struct miscdevice misc_dev;	 /* misc device structure */
	struct dentry *d_entry;		 /* debugfs dir for this device */
	struct gxp_mapped_resource regs; /* ioremapped CSRs */
	struct gxp_mapped_resource mbx[GXP_NUM_CORES]; /* mailbox CSRs */
	struct gxp_mapped_resource fwbufs[GXP_NUM_CORES]; /* FW carveout */
	struct gxp_mapped_resource fwdatabuf; /* Shared FW data carveout */
	struct gxp_mapped_resource coredumpbuf; /* core dump carveout */
	struct gxp_mapped_resource cmu; /* CMU CSRs */
	struct gxp_mailbox_manager *mailbox_mgr;
	struct gxp_power_manager *power_mgr;
	/*
	 * TODO(b/182416287): This should be a rb_tree of lists keyed by
	 * virtual device. For now, keep an array of one list per physical core
	 */
	struct list_head mailbox_resp_queues[GXP_NUM_CORES];
	wait_queue_head_t mailbox_resp_waitqs[GXP_NUM_CORES];
	spinlock_t mailbox_resps_lock;
	struct gxp_debug_dump_manager *debug_dump_mgr;
	struct gxp_mapping_root *mappings;	/* tree of user mappings */
	u32 firmware_running;		 /* firmware status bitmap */
	/*
	 * Reader/writer lock protecting usage of virtual cores assigned to
	 * physical cores.
	 * A writer is any function creating or destroying a virtual core, or
	 * running or stopping one on a physical core.
	 * A reader is any function making use of or interacting with a virtual
	 * core without starting or stopping it on a physical core.
	 */
	/*
	 * TODO(b/216862052) vd_semaphore also currently protects client state.
	 *                   A separate per-client lock should be introduced
	 *                   instead, as part of support for creating VDs
	 *                   without running them on physical cores.
	 */
	struct rw_semaphore vd_semaphore;
	struct gxp_virtual_device *core_to_vd[GXP_NUM_CORES];
	struct gxp_client *debugfs_client;
	struct mutex debugfs_client_lock;
	bool debugfs_wakelock_held;
	struct gxp_thermal_manager *thermal_mgr;
	struct gxp_dma_manager *dma_mgr;
	struct gxp_fw_data_manager *data_mgr;
	struct gxp_tpu_dev tpu_dev;
	struct gxp_telemetry_manager *telemetry_mgr;
	struct gxp_wakelock_manager *wakelock_mgr;
	/*
	 * Pointer to GSA device for firmware authentication.
	 * May be NULL if the chip does not support firmware authentication
	 */
	struct device *gsa_dev;
	u32 memory_per_core;
};

/* GXP device IO functions */

static inline u32 gxp_read_32(struct gxp_dev *gxp, uint reg_offset)
{
	return readl(gxp->regs.vaddr + reg_offset);
}

static inline void gxp_write_32(struct gxp_dev *gxp, uint reg_offset, u32 value)
{
	writel(value, gxp->regs.vaddr + reg_offset);
}

static inline u32 gxp_read_32_core(struct gxp_dev *gxp, uint core,
				   uint reg_offset)
{
	uint offset = GXP_CORE_0_BASE + (GXP_CORE_SIZE * core) + reg_offset;

	return gxp_read_32(gxp, offset);
}

static inline void gxp_write_32_core(struct gxp_dev *gxp, uint core,
				     uint reg_offset, u32 value)
{
	uint offset = GXP_CORE_0_BASE + (GXP_CORE_SIZE * core) + reg_offset;

	gxp_write_32(gxp, offset, value);
}

static inline void gxp_acquire_sync_barrier(struct gxp_dev *gxp, uint index)
{
	uint barrier_reg_offset;

	if (index >= SYNC_BARRIER_COUNT) {
		dev_err(gxp->dev,
			"Attempt to acquire non-existent sync barrier: %d\n",
			index);
		return;
	}

	barrier_reg_offset = SYNC_BARRIER_BLOCK + SYNC_BARRIER_BASE(index);
	while (gxp_read_32(gxp, barrier_reg_offset) !=
	       SYNC_BARRIER_FREE_VALUE) {
		/*
		 * Sleep for the minimum amount.
		 * msleep(1~20) may not do what the caller intends, and will
		 * often sleep longer (~20 ms actual sleep for any value given
		 * in the 1~20ms range).
		 */
		msleep(20);
	}
}

static inline void gxp_release_sync_barrier(struct gxp_dev *gxp, uint index)
{
	uint barrier_reg_offset;

	if (index >= SYNC_BARRIER_COUNT) {
		dev_err(gxp->dev,
			"Attempt to acquire non-existent sync barrier: %d\n",
			index);
		return;
	}

	barrier_reg_offset = SYNC_BARRIER_BLOCK + SYNC_BARRIER_BASE(index);
	gxp_write_32(gxp, barrier_reg_offset, 1);
}
static inline u32 gxp_read_sync_barrier_shadow(struct gxp_dev *gxp, uint index)
{
	uint barrier_reg_offset;

	if (index >= SYNC_BARRIER_COUNT) {
		dev_err(gxp->dev,
			"Attempt to read non-existent sync barrier: %0u\n",
			index);
		return 0;
	}

	barrier_reg_offset = SYNC_BARRIER_BLOCK + SYNC_BARRIER_BASE(index) +
			     SYNC_BARRIER_SHADOW_OFFSET;

	return gxp_read_32(gxp, barrier_reg_offset);
}

static inline int gxp_acquire_rmem_resource(struct gxp_dev *gxp,
					    struct resource *r, char *phandle)
{
	int ret;
	struct device_node *np;

	np = of_parse_phandle(gxp->dev->of_node, phandle, 0);
	if (IS_ERR_OR_NULL(np)) {
		dev_err(gxp->dev, "Failed to find \"%s\" reserved memory\n",
			phandle);
		return -ENODEV;
	}

	ret = of_address_to_resource(np, 0, r);
	of_node_put(np);

	return ret;
}

#endif /* __GXP_INTERNAL_H__ */
