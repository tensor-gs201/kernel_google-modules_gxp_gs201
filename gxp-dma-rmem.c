// SPDX-License-Identifier: GPL-2.0
/*
 * GXP DMA implemented via reserved memory carveouts.
 *
 * Copyright (C) 2021 Google LLC
 */

#include <linux/genalloc.h>
#include <linux/highmem.h>
#include <linux/mm_types.h>
#include <linux/mutex.h>
#include <linux/rbtree.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "gxp-config.h"
#include "gxp-dma.h"
#include "gxp-internal.h"
#include "gxp-mapping.h"

struct gxp_dma_bounce_buffer {
	struct rb_node node;
	dma_addr_t dma_handle;
	struct page *page;
	size_t size;
	unsigned long offset;
	void *buf;
};

struct gxp_dma_rmem_manager {
	struct gxp_dma_manager dma_mgr;
	struct gxp_mapped_resource poolbuf;
	struct gen_pool *pool;
	struct rb_root bounce_buffers;
	struct mutex bounce_lock;
};

/* RB Tree Management Functions for the Bounce Buffer tree */

static int bounce_buffer_put(struct gxp_dma_rmem_manager *mgr,
			     struct gxp_dma_bounce_buffer *bounce)
{
	struct rb_node **link;
	struct rb_node *parent = NULL;
	struct gxp_dma_bounce_buffer *this;

	link = &mgr->bounce_buffers.rb_node;

	mutex_lock(&mgr->bounce_lock);

	while (*link) {
		parent = *link;
		this = rb_entry(parent, struct gxp_dma_bounce_buffer, node);

		if (this->dma_handle > bounce->dma_handle)
			link = &(*link)->rb_left;
		else if (this->dma_handle < bounce->dma_handle)
			link = &(*link)->rb_right;
		else
			goto out;
	}

	rb_link_node(&bounce->node, parent, link);
	rb_insert_color(&bounce->node, &mgr->bounce_buffers);

	mutex_unlock(&mgr->bounce_lock);

	return 0;

out:
	mutex_unlock(&mgr->bounce_lock);
	return -EINVAL;
}

static struct gxp_dma_bounce_buffer *
bounce_buffer_get(struct gxp_dma_rmem_manager *mgr, dma_addr_t dma_handle)
{
	struct rb_node *node;
	struct gxp_dma_bounce_buffer *this;

	mutex_lock(&mgr->bounce_lock);

	node = mgr->bounce_buffers.rb_node;

	while (node) {
		this = rb_entry(node, struct gxp_dma_bounce_buffer, node);

		if (this->dma_handle > dma_handle) {
			node = node->rb_left;
		} else if (this->dma_handle + this->size <= dma_handle) {
			node = node->rb_right;
		} else {
			mutex_unlock(&mgr->bounce_lock);
			return this;
		}
	}

	mutex_unlock(&mgr->bounce_lock);

	return NULL;
}

static void bounce_buffer_remove(struct gxp_dma_rmem_manager *mgr,
				 struct gxp_dma_bounce_buffer *bounce)
{
	rb_erase(&bounce->node, &mgr->bounce_buffers);
}

/* gxp-dma.h Interface */

int gxp_dma_init(struct gxp_dev *gxp)
{
	struct gxp_dma_rmem_manager *mgr;
	struct resource r;
	int ret;

	mgr = devm_kzalloc(gxp->dev, sizeof(*mgr), GFP_KERNEL);
	if (!mgr)
		return -ENOMEM;

	/* Map the reserved memory for the pool from the device tree */
	if (gxp_acquire_rmem_resource(gxp, &r, "gxp-pool-region")) {
		dev_err(gxp->dev, "Unable to acquire pool reserved memory\n");
		return -ENODEV;
	}

	mgr->poolbuf.paddr = r.start;
	mgr->poolbuf.size = resource_size(&r);
	mgr->poolbuf.vaddr = devm_memremap(gxp->dev, mgr->poolbuf.paddr,
					   mgr->poolbuf.size, MEMREMAP_WC);
	if (IS_ERR_OR_NULL(mgr->poolbuf.vaddr)) {
		dev_err(gxp->dev, "Failed to map pool\n");
		return -ENODEV;
	}

	/* Create the gen pool for mappings/coherent allocations */
	mgr->pool = devm_gen_pool_create(gxp->dev, PAGE_SHIFT, -1, "gxp-pool");
	if (!mgr->pool) {
		dev_err(gxp->dev, "Failed to create memory pool\n");
		return -ENOMEM;
	}

	ret = gen_pool_add_virt(mgr->pool, (unsigned long)mgr->poolbuf.vaddr,
				mgr->poolbuf.paddr, mgr->poolbuf.size, -1);
	if (ret) {
		dev_err(gxp->dev, "Failed to add memory to pool (ret = %d)\n",
			ret);
		return ret;
	}

	mgr->dma_mgr.mapping_tree = RB_ROOT;
	mgr->bounce_buffers = RB_ROOT;

	gxp->dma_mgr = &(mgr->dma_mgr);

	return 0;
}

void gxp_dma_exit(struct gxp_dev *gxp)
{
	/* no cleanup */
}

int gxp_dma_map_resources(struct gxp_dev *gxp)
{
	unsigned int core;

	/* all resources are accessed via PA if there's no iommu */
	for (core = 0; core < GXP_NUM_CORES; core++) {
		gxp->mbx[core].daddr = gxp->mbx[core].paddr;
		gxp->fwbufs[core].daddr = gxp->fwbufs[core].paddr;
	}
	gxp->regs.daddr = gxp->regs.paddr;
	gxp->coredumpbuf.daddr = gxp->coredumpbuf.paddr;

	return 0;
}

void gxp_dma_unmap_resources(struct gxp_dev *gxp)
{
	/* no mappings to undo */
}

void *gxp_dma_alloc_coherent(struct gxp_dev *gxp, uint core_list, size_t size,
			     dma_addr_t *dma_handle, gfp_t flag,
			     uint gxp_dma_flags)
{
	/* Allocate the buffer from the cache-coherent pool */
	struct gxp_dma_rmem_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_rmem_manager, dma_mgr);
	void *vaddr = (void *)gen_pool_alloc(mgr->pool, size);

	if (!vaddr) {
		dev_err(gxp->dev, "Unable to allocate coherent buffer\n");
		return NULL;
	}

	/*
	 * On SysMMU-less systems, all GXP cores access DRAM directly, so set
	 * the dma_handle to the buffer's physical address.
	 */
	if (dma_handle) {
		*dma_handle =
			gen_pool_virt_to_phys(mgr->pool, (unsigned long)vaddr);

		if (*dma_handle == -1) {
			dev_err(gxp->dev,
				"Unable to get dma_addr_t for coherent buffer\n");
			gen_pool_free(mgr->pool, (unsigned long)vaddr, size);
			return NULL;
		}
	}

	/* `core_list` is unused, since no SysMMU means there's no mappings */
	return vaddr;
}

void gxp_dma_free_coherent(struct gxp_dev *gxp, uint core_list, size_t size,
			   void *cpu_addr, dma_addr_t dma_handle)
{
	/* No unmapping required since there's no SysMMU */

	/* Clean up the buffer */
	struct gxp_dma_rmem_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_rmem_manager, dma_mgr);
	gen_pool_free(mgr->pool, (unsigned long)cpu_addr, size);
}

dma_addr_t gxp_dma_map_single(struct gxp_dev *gxp, uint core_list,
			      void *cpu_addr, size_t size,
			      enum dma_data_direction direction,
			      unsigned long attrs, uint gxp_dma_flags)
{
	return gxp_dma_map_page(gxp, core_list, virt_to_page(cpu_addr),
				offset_in_page(cpu_addr), size, direction,
				attrs, gxp_dma_flags);
}

void gxp_dma_unmap_single(struct gxp_dev *gxp, uint core_list,
			  dma_addr_t dma_addr, size_t size,
			  enum dma_data_direction direction,
			  unsigned long attrs)
{
	return gxp_dma_unmap_page(gxp, core_list, dma_addr, size, direction,
				  attrs);
}

dma_addr_t gxp_dma_map_page(struct gxp_dev *gxp, uint core_list,
			    struct page *page, unsigned long offset,
			    size_t size, enum dma_data_direction direction,
			    unsigned long attrs, uint gxp_dma_flags)
{
	struct gxp_dma_rmem_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_rmem_manager, dma_mgr);
	struct gxp_dma_bounce_buffer *bounce;
	void *page_buf;
	int ret;

	bounce = kzalloc(sizeof(struct gxp_dma_bounce_buffer), GFP_KERNEL);
	if (!bounce) {
		dev_err(gxp->dev,
			"Failed to allocate tracking struct for mapping page\n");
		return DMA_MAPPING_ERROR;
	}

	bounce->offset = offset;
	bounce->size = size;
	bounce->page = page;

	bounce->buf = (void *)gen_pool_alloc(mgr->pool, size);
	if (!bounce->buf) {
		dev_err(gxp->dev,
			"Failed to allocate bounce buffer for mapping page\n");
		goto pool_alloc_error;
	}

	bounce->dma_handle =
		gen_pool_virt_to_phys(mgr->pool, (unsigned long)bounce->buf);
	if (bounce->dma_handle == -1) {
		dev_err(gxp->dev, "Unable to get dma_addr_t for mapped page\n");
		goto error;
	}

	page_buf = kmap(page);
	if (!page_buf) {
		dev_err(gxp->dev,
			"Failed to map page for copying to bounce buffer\n");
		goto error;
	}
	memcpy(bounce->buf, page_buf + offset, size);
	kunmap(page);

	ret = bounce_buffer_put(mgr, bounce);
	if (ret) {
		dev_err(gxp->dev,
			"Unable to put bounce buffer!\n");
		goto error;
	}

	return bounce->dma_handle;

error:
	gen_pool_free(mgr->pool, (unsigned long)bounce->buf, bounce->size);
pool_alloc_error:
	kfree(bounce);
	return DMA_MAPPING_ERROR;
}

void gxp_dma_unmap_page(struct gxp_dev *gxp, uint core_list,
			dma_addr_t dma_addr, size_t size,
			enum dma_data_direction direction, unsigned long attrs)
{
	struct gxp_dma_rmem_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_rmem_manager, dma_mgr);
	struct gxp_dma_bounce_buffer *bounce =
		bounce_buffer_get(mgr, dma_addr);
	void *page_buf = NULL;

	if (!bounce || !bounce->page) {
		dev_err(gxp->dev, "No page to unmap for IOVA %pad\n",
			&dma_addr);
		return;
	}

	bounce_buffer_remove(mgr, bounce);

	page_buf = kmap(bounce->page);
	if (!page_buf) {
		dev_warn(
			gxp->dev,
			"Failed to map page for copying from bounce buffer on unmap\n");
	} else {
		memcpy(page_buf + bounce->offset, bounce->buf, bounce->size);
	}
	kunmap(bounce->page);

	gen_pool_free(mgr->pool, (unsigned long)bounce->buf, bounce->size);
	kfree(bounce);
}

dma_addr_t gxp_dma_map_resource(struct gxp_dev *gxp, uint core_list,
				phys_addr_t phys_addr, size_t size,
				enum dma_data_direction direction,
				unsigned long attrs, uint gxp_dma_flags)
{
	dev_warn(gxp->dev, "%s: not yet supported!\n", __func__);
	return 0;
}

void gxp_dma_unmap_resource(struct gxp_dev *gxp, uint core_list,
			    dma_addr_t dma_addr, size_t size,
			    enum dma_data_direction direction,
			    unsigned long attrs)
{
	dev_warn(gxp->dev, "%s: not yet supported!\n", __func__);
}

int gxp_dma_map_sg(struct gxp_dev *gxp, uint core_list, struct scatterlist *sg,
		   int nents, enum dma_data_direction direction,
		   unsigned long attrs, uint gxp_dma_flags)
{
	struct gxp_dma_rmem_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_rmem_manager, dma_mgr);
	struct gxp_dma_bounce_buffer *bounce;
	struct scatterlist *s;
	size_t size_so_far = 0;
	int i;

	bounce = kzalloc(sizeof(struct gxp_dma_bounce_buffer), GFP_KERNEL);
	if (!bounce) {
		dev_err(gxp->dev,
			"Failed to allocate tracking struct for mapping sg\n");
		return 0;
	}

	for_each_sg(sg, s, nents, i)
		bounce->size += s->length;

	bounce->buf = (void *)gen_pool_alloc(mgr->pool, bounce->size);
	if (!bounce->buf) {
		dev_err(gxp->dev,
			"Failed to allocate bounce buffer for mapping sg\n");
		goto pool_alloc_error;
	}

	sg_copy_to_buffer(sg, nents, bounce->buf, bounce->size);

	for_each_sg(sg, s, nents, i) {
		s->dma_length = s->length;
		s->dma_address = gen_pool_virt_to_phys(
			mgr->pool, (unsigned long)bounce->buf) + size_so_far;
		size_so_far += s->length;

		if (s->dma_address == -1) {
			dev_err(gxp->dev,
				"Failed to get dma_addr_t while mapping sg\n");
			goto error;
		}
	}

	bounce->dma_handle = sg->dma_address;
	/* SGs use the SG's internal page and offset values */
	bounce->page = NULL;
	bounce->offset = 0;

	if (bounce_buffer_put(mgr, bounce)) {
		dev_err(gxp->dev, "Unable to put bounce buffer for sg!\n");
		goto error;
	}

	return nents;

error:
	/* TODO is this necessary? */
	for_each_sg(sg, s, nents, i) {
		s->dma_length = 0;
		s->dma_address = 0;
	}

	gen_pool_free(mgr->pool, (unsigned long) bounce->buf, bounce->size);

pool_alloc_error:
	kfree(bounce);

	return 0;
}

void gxp_dma_unmap_sg(struct gxp_dev *gxp, uint core_list,
		      struct scatterlist *sg, int nents,
		      enum dma_data_direction direction, unsigned long attrs)
{
	struct gxp_dma_rmem_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_rmem_manager, dma_mgr);
	struct gxp_dma_bounce_buffer *bounce =
		bounce_buffer_get(mgr, sg->dma_address);
	struct scatterlist *s;
	int i;

	if (!bounce || bounce->page) {
		dev_err(gxp->dev, "No sg to unmap for IOVA %pad\n",
			&sg->dma_address);
		return;
	}

	if (!(attrs & DMA_ATTR_SKIP_CPU_SYNC))
		gxp_dma_sync_sg_for_cpu(gxp, sg, nents, direction);

	bounce_buffer_remove(mgr, bounce);

	/* TODO is this necessary? */
	for_each_sg(sg, s, nents, i) {
		s->dma_length = 0;
		s->dma_address = 0;
	}

	gen_pool_free(mgr->pool, (unsigned long)bounce->buf, bounce->size);
	kfree(bounce);
}

void gxp_dma_sync_single_for_cpu(struct gxp_dev *gxp, dma_addr_t dma_handle,
				 size_t size, enum dma_data_direction direction)
{
	struct gxp_dma_rmem_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_rmem_manager, dma_mgr);
	struct gxp_dma_bounce_buffer *bounce =
		bounce_buffer_get(mgr, dma_handle);
	void *page_buf = NULL;
	unsigned long addr_offset;

	if (!bounce || !bounce->page) {
		dev_err(gxp->dev, "No single mapping to sync for IOVA %pad\n",
			&dma_handle);
		return;
	}

	addr_offset = dma_handle - bounce->dma_handle;

	/* Copy the contents of the bounce buffer back to the mapped page */
	page_buf = kmap(bounce->page);
	if (!page_buf) {
		dev_warn(gxp->dev,
			 "Failed to map page for syncing from bounce buffer\n");
		return;
	}
	memcpy(page_buf + bounce->offset + addr_offset,
	       bounce->buf + addr_offset, bounce->size);
	kunmap(bounce->page);
}

void gxp_dma_sync_single_for_device(struct gxp_dev *gxp, dma_addr_t dma_handle,
				    size_t size,
				    enum dma_data_direction direction)
{
	struct gxp_dma_rmem_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_rmem_manager, dma_mgr);
	struct gxp_dma_bounce_buffer *bounce =
		bounce_buffer_get(mgr, dma_handle);
	void *page_buf = NULL;
	unsigned long addr_offset;

	if (!bounce || !bounce->page) {
		dev_err(gxp->dev, "No single mapping to sync for IOVA %pad\n",
			&dma_handle);
		return;
	}

	addr_offset = dma_handle - bounce->dma_handle;

	/* Copy the latest contents of the mapped page to the bounce buffer*/
	page_buf = kmap(bounce->page);
	if (!page_buf) {
		dev_warn(gxp->dev,
			 "Failed to map page for syncing to bounce buffer\n");
		return;
	}
	memcpy(bounce->buf + addr_offset,
	       page_buf + bounce->offset + addr_offset, bounce->size);
	kunmap(bounce->page);
}

void gxp_dma_sync_sg_for_cpu(struct gxp_dev *gxp, struct scatterlist *sg,
			     int nents, enum dma_data_direction direction)
{
	struct gxp_dma_rmem_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_rmem_manager, dma_mgr);
	struct gxp_dma_bounce_buffer *bounce =
		bounce_buffer_get(mgr, sg->dma_address);
	void *page_buf;
	struct scatterlist *s;
	unsigned int i;

	if (!bounce || bounce->page) {
		dev_err(gxp->dev, "No mapping to sync for sg\n");
		return;
	}

	for_each_sg(sg, s, nents, i) {
		page_buf = kmap(sg_page(s));
		if (!page_buf) {
			dev_warn(gxp->dev, "Failed to map page for sg sync\n");
			continue;
		}
		memcpy(page_buf + s->offset,
		       bounce->buf + (s->dma_address - bounce->dma_handle),
		       s->dma_length);
		kunmap(sg_page(s));
	}
}

void gxp_dma_sync_sg_for_device(struct gxp_dev *gxp, struct scatterlist *sg,
				int nents, enum dma_data_direction direction)
{
	struct gxp_dma_rmem_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_rmem_manager, dma_mgr);
	struct gxp_dma_bounce_buffer *bounce =
		bounce_buffer_get(mgr, sg->dma_address);
	void *page_buf;
	struct scatterlist *s;
	unsigned int i;

	if (!bounce || bounce->page) {
		dev_err(gxp->dev, "No mapping to sync for sg\n");
		return;
	}

	for_each_sg(sg, s, nents, i) {
		page_buf = kmap(sg_page(s));
		if (!page_buf) {
			dev_warn(gxp->dev, "Failed to map page for sg sync\n");
			continue;
		}
		memcpy(bounce->buf + (s->dma_address - bounce->dma_handle),
		       page_buf + s->offset, s->dma_length);
		kunmap(sg_page(s));
	}
}
