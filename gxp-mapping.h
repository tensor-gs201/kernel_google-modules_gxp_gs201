/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Records the mapped device addresses.
 *
 * Copyright (C) 2020 Google LLC
 */
#ifndef __GXP_MAPPING_H__
#define __GXP_MAPPING_H__

#include <linux/dma-direction.h>
#include <linux/mutex.h>
#include <linux/rbtree.h>
#include <linux/scatterlist.h>
#include <linux/types.h>

#include "gxp-internal.h"

struct gxp_mapping_root {
	struct rb_root rb;
	struct mutex lock;
};

struct gxp_mapping {
	struct rb_node node;
	/*
	 * User-space address of the mapped buffer.
	 * If this value is 0, it indicates this mapping is for a dma-buf and
	 * should not be used if a regular buffer mapping was expected.
	 */
	u64 host_address;
	uint virt_core_list;
	struct gxp_virtual_device *vd;
	/*
	 * `device_address` and `size` are the base address and size of the
	 * user buffer a mapping represents.
	 *
	 * Due to alignment requirements from hardware, the actual IOVA space
	 * allocated may be larger and start at a different address, but that
	 * information is contained in the scatter-gather table, `sgt` below.
	 */
	dma_addr_t device_address;
	size_t size;
	uint gxp_dma_flags;
	enum dma_data_direction dir;
	struct sg_table sgt;
	u32 map_count;
};

int gxp_mapping_init(struct gxp_dev *gxp);
struct gxp_mapping *gxp_mapping_create(struct gxp_dev *gxp,
				       struct gxp_virtual_device *vd,
				       uint virt_core_list, u64 user_address,
				       size_t size, u32 flags,
				       enum dma_data_direction dir);
void gxp_mapping_destroy(struct gxp_dev *gxp, struct gxp_mapping *mapping);
int gxp_mapping_sync(struct gxp_dev *gxp, struct gxp_mapping *mapping,
		     u32 offset, u32 size, bool for_cpu);
int gxp_mapping_put(struct gxp_dev *gxp, struct gxp_mapping *map);
struct gxp_mapping *gxp_mapping_get(struct gxp_dev *gxp,
				    dma_addr_t device_address);
struct gxp_mapping *gxp_mapping_get_host(struct gxp_dev *gxp, u64 host_address);
void gxp_mapping_remove(struct gxp_dev *gxp, struct gxp_mapping *map);

#endif /* __GXP_MAPPING_H__ */
