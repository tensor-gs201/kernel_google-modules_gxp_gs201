/* SPDX-License-Identifier: GPL-2.0 */
/*
 * GXP DMA interface.
 *
 * Copyright (C) 2021 Google LLC
 */
#ifndef __GXP_DMA_H__
#define __GXP_DMA_H__

#include <linux/dma-direction.h>
#include <linux/dma-mapping.h>
#include <linux/types.h>
#ifdef CONFIG_ANDROID
#include <soc/google/tpu-ext.h>
#endif

#include "gxp-internal.h"

struct gxp_dma_manager {
	struct rb_root mapping_tree;
};

/*
 * Error value to be returned in place of a dma_addr_t when a mapping fails.
 *
 * On newer kernels, this is defined in <linux/dma-mapping.h>. Redefined here
 * for older kernels, so clients can check for this value without worrying
 * which kernel version they're compiled for.
 */
#ifndef DMA_MAPPING_ERROR
#define DMA_MAPPING_ERROR (~(dma_addr_t)0)
#endif

/**
 * gxp_dma_init() - Initialize the GXP DMA subsystem
 * @gxp: The GXP device to initialize DMA for
 *
 * Return:
 * * 0       - DMA initialized successfully
 * * -EIO    - Failed to initialize underlying IOMMU hardware
 * * -ENODEV - The necessary hardware or device tree entries are missing
 * * -ENOMEM - Insufficient memory is available to initialize the interface
 */
int gxp_dma_init(struct gxp_dev *gxp);

/**
 * gxp_dma_exit() - Tear down the GXP DMA subsystem and release hardware
 * @gxp: The GXP device to tear down DMA for
 */
void gxp_dma_exit(struct gxp_dev *gxp);

/**
 * gxp_dma_map_resources() - Map the various buffers/registers with fixed IOVAs
 * @gxp: The GXP device to setup the mappings for
 *
 * GXP firmware expects several buffers and registers to be mapped to fixed
 * locations in their IOVA space. This function initializes all those mappings.
 *
 * This function must not be called until after all the `vaddr` and `size`
 * fields of every `struct gxp_mapped_resource` inside of @gxp have been
 * initialized.
 *
 * Return:
 * * 0    - Mappings created successfully
 * * -EIO - Failed to create one or more of the mappings
 */
int gxp_dma_map_resources(struct gxp_dev *gxp);

/**
 * gxp_dma_unmap_resources() - Unmap the IOVAs mapped by gxp_dma_map_resources
 * @gxp: The GXP device that was passed to gxp_dma_map_resources()
 *
 * GXP firmware expects several buffers and registers to be mapped to fixed
 * locations in their IOVA space. This function releases all those mappings.
 *
 * This function should be called after gxp_dma_map_resources().
 */
void gxp_dma_unmap_resources(struct gxp_dev *gxp);

#ifdef CONFIG_ANDROID
/**
 * gxp_dma_map_tpu_buffer() - Map the tpu mbx queue buffers with fixed IOVAs
 * @gxp: The GXP device to setup the mappings for
 * @core_list: A bitfield enumerating the physical cores the mapping is for
 * @mbx_info: Structure holding TPU-DSP mailbox queue buffer information
 *
 * Return:
 * * 0    - Mappings created successfully
 * * -EIO - Failed to create the mappings
 */
int gxp_dma_map_tpu_buffer(struct gxp_dev *gxp, uint core_list,
			   struct edgetpu_ext_mailbox_info *mbx_info);

/**
 * gxp_dma_unmap_tpu_buffer() - Unmap IOVAs mapped by gxp_dma_map_tpu_buffer()
 * @gxp: The GXP device that was passed to gxp_dma_map_tpu_buffer()
 * @mbx_desc: Structure holding info for already mapped TPU-DSP mailboxes.
 */
void gxp_dma_unmap_tpu_buffer(struct gxp_dev *gxp,
			      struct gxp_tpu_mbx_desc mbx_desc);
#endif  // CONFIG_ANDROID

/**
 * gxp_dma_alloc_coherent() - Allocate and map a coherent buffer for a GXP core
 * @gxp: The GXP device to map the allocated buffer for
 * @core_list: A bitfield enumerating the physical cores the mapping is for
 * @size: The size of the buffer to be allocated, in bytes
 * @dma_handle: Reference to a variable to be set to the allocated IOVA
 * @flag: The type of memory to allocate (see kmalloc)
 * @gxp_dma_flags: The type of mapping to create; Currently unused
 *
 * Return: Kernel virtual address of the allocated/mapped buffer
 */
void *gxp_dma_alloc_coherent(struct gxp_dev *gxp, uint core_list, size_t size,
			     dma_addr_t *dma_handle, gfp_t flag,
			     uint gxp_dma_flags);
/**
 * gxp_dma_free_coherent() - Unmap and free a coherent buffer
 * @gxp: The GXP device the buffer was allocated and mapped for
 * @core_list: A bitfield enumerating the physical cores the mapping is for
 * @size: The size of the buffer, in bytes, passed to `gxp_dma_alloc()`
 * @cpu_addr: The kernel virtual address returned by `gxp_dma_alloc()`
 * @dma_handle: The device IOVA, set by `gxp_dma_alloc()`
 *
 * If the buffer has been mirror-mapped via `gxp_dma_mirror_map()`, the buffer
 * will not be freed until all mappings have been unmapped.
 */
void gxp_dma_free_coherent(struct gxp_dev *gxp, uint core_list, size_t size,
			   void *cpu_addr, dma_addr_t dma_handle);

/**
 * gxp_dma_map_single() - Create a mapping for a kernel buffer
 * @gxp: The GXP device to map the buffer for
 * @core_list: A bitfield enumerating the physical cores the mapping is for
 * @cpu_addr: The kernel virtual address of the buffer to map
 * @size: The size of the buffer to map, in bytes
 * @direction: DMA direction
 * @attrs: The same set of flags used by the base DMA API
 * @gxp_dma_flags: The type of mapping to create; Currently unused
 *
 * Return: The IOVA the buffer was mapped to
 */
dma_addr_t gxp_dma_map_single(struct gxp_dev *gxp, uint core_list,
			      void *cpu_addr, size_t size,
			      enum dma_data_direction direction,
			      unsigned long attrs, uint gxp_dma_flags);
/**
 * gxp_dma_unmap_single() - Unmap a kernel buffer
 * @gxp: The GXP device the buffer was mapped for
 * @core_list: A bitfield enumerating the physical cores the mapping is for
 * @dma_addr: The device IOVA, returned by `gxp_dma_map_single()`
 * @size: The size of the mapping, which was passed to `gxp_dma_map_single()`
 * @direction: DMA direction; same as passed to `gxp_dma_map_single()`
 * @attrs: The same set of flags used by the base DMA API
 */
void gxp_dma_unmap_single(struct gxp_dev *gxp, uint core_list,
			  dma_addr_t dma_addr, size_t size,
			  enum dma_data_direction direction,
			  unsigned long attrs);

/**
 * gxp_dma_map_page() - Create a mapping for a physical page of memory
 * @gxp: The GXP device to map the page for
 * @core_list: A bitfield enumerating the physical cores the mapping is for
 * @page: The `struct page` of the physical page to create a mapping for
 * @offset: The offset into @page to begin the mapping at
 * @size: The number of bytes in @page to map
 * @direction: DMA direction
 * @attrs: The same set of flags used by the base DMA API
 * @gxp_dma_flags: The type of mapping to create; Currently unused
 *
 * Return: The IOVA the page was mapped to
 */
dma_addr_t gxp_dma_map_page(struct gxp_dev *gxp, uint core_list,
			    struct page *page, unsigned long offset,
			    size_t size, enum dma_data_direction direction,
			    unsigned long attrs, uint gxp_dma_flags);
/**
 * gxp_dma_unmap_page() - Unmap a physical page of memory
 * @gxp: The GXP device the page was mapped for
 * @core_list: A bitfield enumerating the physical cores the mapping is for
 * @dma_addr: The device IOVA, returned by `gxp_dma_map_page()`
 * @size: The size of the mapping, which was passed to `gxp_dma_map_page()`
 * @direction: DMA direction; Same as passed to `gxp_dma_map_page()`
 * @attrs: The same set of flags used by the base DMA API
 */
void gxp_dma_unmap_page(struct gxp_dev *gxp, uint core_list,
			dma_addr_t dma_addr, size_t size,
			enum dma_data_direction direction, unsigned long attrs);

/**
 * gxp_dma_map_resource() - Create a mapping for an MMIO resource
 * @gxp: The GXP device to map the resource for
 * @core_list: A bitfield enumerating the physical cores the mapping is for
 * @phys_addr: The physical address of the MMIO resource to map
 * @size: The size of the MMIO region to map, in bytes
 * @direction: DMA direction
 * @attrs: The same set of flags used by the base DMA API
 * @gxp_dma_flags: The type of mapping to create; Currently unused
 *
 * Return: The IOVA the MMIO resource was mapped to
 */
dma_addr_t gxp_dma_map_resource(struct gxp_dev *gxp, uint core_list,
				phys_addr_t phys_addr, size_t size,
				enum dma_data_direction direction,
				unsigned long attrs, uint gxp_dma_flags);
/**
 * gxp_dma_unmap_resource() - Unmap an MMIO resource
 * @gxp: The GXP device the MMIO resource was mapped for
 * @core_list: A bitfield enumerating the physical cores the mapping is for
 * @dma_addr: The device IOVA, returned by `gxp_dma_map_resource()`
 * @size: The size of the mapping, which was passed to `gxp_dma_map_resource()`
 * @direction: DMA direction; Same as passed to `gxp_dma_map_resource()`
 * @attrs: The same set of flags used by the base DMA API
 */
void gxp_dma_unmap_resource(struct gxp_dev *gxp, uint core_list,
			    dma_addr_t dma_addr, size_t size,
			    enum dma_data_direction direction,
			    unsigned long attrs);

/**
 * gxp_dma_map_sg() - Create a mapping for a scatter-gather list
 * @gxp: The GXP device to map the scatter-gather list for
 * @core_list: A bitfield enumerating the physical cores the mapping is for
 * @sg: The scatter-gather list of the buffer to be mapped
 * @nents: The number of entries in @sg
 * @direction: DMA direction
 * @attrs: The same set of flags used by the base DMA API
 * @gxp_dma_flags: The type of mapping to create; Currently unused
 *
 * Return: The number of scatter-gather entries mapped to
 */
int gxp_dma_map_sg(struct gxp_dev *gxp, uint core_list, struct scatterlist *sg,
		   int nents, enum dma_data_direction direction,
		   unsigned long attrs, uint gxp_dma_flags);
/**
 * gxp_dma_unmap_sg() - Unmap a scatter-gather list
 * @gxp: The GXP device the scatter-gather list was mapped for
 * @core_list: A bitfield enumerating the physical cores the mapping is for
 * @sg: The scatter-gather list to unmap; The same one passed to
 *      `gxp_dma_map_sg()`
 * @nents: The number of entries in @sg; Same value passed to `gxp_dma_map_sg()`
 * @direction: DMA direction; Same as passed to `gxp_dma_map_sg()`
 * @attrs: The same set of flags used by the base DMA API
 */
void gxp_dma_unmap_sg(struct gxp_dev *gxp, uint core_list,
		      struct scatterlist *sg, int nents,
		      enum dma_data_direction direction, unsigned long attrs);

/**
 * gxp_dma_sync_single_for_cpu() - Sync buffer for reading by the CPU
 * @gxp: The GXP device the mapping was created for
 * @dma_handle: The device IOVA, obtained from one of the `gxp_dma_map_*` APIs
 * @size: The size of the mapped region to sync
 * @direction: DMA direction
 */
void gxp_dma_sync_single_for_cpu(struct gxp_dev *gxp, dma_addr_t dma_handle,
				 size_t size,
				 enum dma_data_direction direction);
/**
 * gxp_dma_sync_single_for_device() - Sync buffer for reading by the device
 * @gxp: The GXP device the mapping was created for
 * @dma_handle: The device IOVA, obtained from one of the `gxp_dma_map_*` APIs
 * @size: The size of the mapped region to sync
 * @direction: DMA direction
 */
void gxp_dma_sync_single_for_device(struct gxp_dev *gxp, dma_addr_t dma_handle,
				    size_t size,
				    enum dma_data_direction direction);

/**
 * gxp_dma_sync_sg_for_cpu() - Sync sg list for reading by the  CPU
 * @gxp: The GXP device the mapping was created for
 * @sg: The mapped scatter-gather list to be synced
 * @nents: The number of entries in @sg
 * @direction: DMA direction
 */
void gxp_dma_sync_sg_for_cpu(struct gxp_dev *gxp, struct scatterlist *sg,
			     int nents, enum dma_data_direction direction);
/**
 * gxp_dma_sync_sg_for_device() - Sync sg list for reading by the device
 * @gxp: The GXP device the mapping was created for
 * @sg: The mapped scatter-gather list to be synced
 * @nents: The number of entries in @sg
 * @direction: DMA direction
 */
void gxp_dma_sync_sg_for_device(struct gxp_dev *gxp, struct scatterlist *sg,
				int nents, enum dma_data_direction direction);

#endif /* __GXP_DMA_H__ */
