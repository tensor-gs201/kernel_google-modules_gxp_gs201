/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Support for using dma-bufs.
 *
 * Copyright (C) 2022 Google LLC
 */
#ifndef __GXP_DMABUF_H__
#define __GXP_DMABUF_H__

#include <linux/dma-direction.h>
#include <linux/types.h>

#include "gxp-internal.h"
#include "gxp-mapping.h"

/**
 * gxp_dmabuf_map() - Map a dma-buf for access by the specified physical cores
 * @gxp: The GXP device to map the dma-buf for
 * @core_list: A bitfield enumerating the physical cores the mapping is for
 * @fd: A file descriptor for the dma-buf to be mapped
 * @flags: The type of mapping to create; Currently unused
 * @direction: DMA direction
 *
 * Return: The structure that was created and is being tracked to describe the
 *         mapping of the dma-buf. Returns ERR_PTR on failure.
 */
struct gxp_mapping *gxp_dmabuf_map(struct gxp_dev *gxp, uint core_list, int fd,
				   u32 flags, enum dma_data_direction dir);

/**
 * gxp_dmabuf_unmap - Unmap a dma-buf previously mapped with `gxp_dmabuf_map()`
 * @gxp: The GXP device the dma-buf was mapped for.
 * @device_address: The IOVA the dma-buf was mapped to. Should be obtained from
 *                  the `device_address` field of the `struct gxp_mapping`
 *                  returned by `gxp_dmabuf_map()`
 */
void gxp_dmabuf_unmap(struct gxp_dev *gxp, dma_addr_t device_address);

#endif /* __GXP_DMABUF_H__ */
