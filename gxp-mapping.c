// SPDX-License-Identifier: GPL-2.0
/*
 * Records the mapped device addresses.
 *
 * Copyright (C) 2021 Google LLC
 */

#include <linux/dma-mapping.h>
#include <linux/mm.h>
#include <linux/slab.h>

#include "gxp-dma.h"
#include "gxp-internal.h"
#include "gxp-mapping.h"
#include "mm-backport.h"

int gxp_mapping_init(struct gxp_dev *gxp)
{
	gxp->mappings =
		devm_kzalloc(gxp->dev, sizeof(*gxp->mappings), GFP_KERNEL);
	if (!gxp->mappings)
		return -ENOMEM;

	gxp->mappings->rb = RB_ROOT;
	mutex_init(&gxp->mappings->lock);

	return 0;
}

struct gxp_mapping *gxp_mapping_create(struct gxp_dev *gxp, uint core_list,
				       u64 user_address, size_t size, u32 flags,
				       enum dma_data_direction dir)
{
	struct gxp_mapping *mapping = NULL;
	uint num_pages = 0;
	struct page **pages;
	ulong offset;
	int ret, i;
	struct vm_area_struct *vma;
	unsigned int foll_flags = FOLL_LONGTERM | FOLL_WRITE;

	/*
	 * The host pages might be read-only and could fail if we attempt to pin
	 * it with FOLL_WRITE.
	 * default to read/write if find_extend_vma returns NULL
	 */
	vma = find_extend_vma(current->mm, user_address & PAGE_MASK);
	if (vma && !(vma->vm_flags & VM_WRITE)) {
		foll_flags &= ~FOLL_WRITE;
		if (dir != DMA_TO_DEVICE) {
			dev_err(gxp->dev,
				"Unable to map read-only pages as anything but DMA_TO_DEVICE\n");
			return ERR_PTR(-EINVAL);
		}
	}

	/* Pin the user pages */
	offset = user_address & (PAGE_SIZE - 1);
	if (unlikely((size + offset) / PAGE_SIZE >= UINT_MAX - 1 ||
		     size + offset < size))
		return ERR_PTR(-ENOMEM);
	num_pages = (size + offset) / PAGE_SIZE;
	if ((size + offset) % PAGE_SIZE)
		num_pages++;

	pages = kcalloc(num_pages, sizeof(*pages), GFP_KERNEL);
	if (!pages)
		return ERR_PTR(-ENOMEM);

	/* Provide protection around `pin_user_pages_fast` since it fails if
	 * called by more than one thread simultaneously.
	 */
	mutex_lock(&gxp->mappings->lock);
	ret = pin_user_pages_fast(user_address & PAGE_MASK, num_pages,
				  foll_flags, pages);
	mutex_unlock(&gxp->mappings->lock);
	if (ret < 0 || ret < num_pages) {
		dev_dbg(gxp->dev,
			"Get user pages failed: user_add=%pK, num_pages=%u, ret=%d\n",
			(void *)user_address, num_pages, ret);
		num_pages = ret < 0 ? 0 : ret;
		ret = ret >= 0 ? -EFAULT : ret;
		goto error_unpin_pages;
	}

	/* Initialize mapping book-keeping */
	mapping = kzalloc(sizeof(*mapping), GFP_KERNEL);
	if (!mapping) {
		ret = -ENOMEM;
		goto error_unpin_pages;
	}
	mapping->host_address = user_address;
	mapping->core_list = core_list;
	mapping->size = size;
	mapping->map_count = 1;
	mapping->gxp_dma_flags = flags;
	mapping->dir = dir;
	ret = sg_alloc_table_from_pages(&mapping->sgt, pages, num_pages, 0,
					num_pages * PAGE_SIZE, GFP_KERNEL);
	if (ret) {
		dev_dbg(gxp->dev, "Failed to alloc sgt for mapping (ret=%d)\n",
			ret);
		goto error_free_sgt;
	}

	/* map the user pages */
	ret = gxp_dma_map_sg(gxp, mapping->core_list, mapping->sgt.sgl,
			     mapping->sgt.nents, mapping->dir,
			     DMA_ATTR_SKIP_CPU_SYNC, mapping->gxp_dma_flags);
	if (!ret) {
		dev_dbg(gxp->dev, "Failed to map sgt (ret=%d)\n", ret);
		ret = -EINVAL;
		goto error_free_sgt;
	}
	mapping->sgt.nents = ret;
	mapping->device_address =
		sg_dma_address(mapping->sgt.sgl) + offset;

	kfree(pages);
	return mapping;

error_free_sgt:
	sg_free_table(&mapping->sgt);
	kfree(mapping);
error_unpin_pages:
	for (i = 0; i < num_pages; i++)
		unpin_user_page(pages[i]);
	kfree(pages);

	return ERR_PTR(ret);
}

void gxp_mapping_destroy(struct gxp_dev *gxp, struct gxp_mapping *mapping)
{
	struct sg_page_iter sg_iter;
	struct page *page;

	/*
	 * Unmap the user pages
	 *
	 * Normally on unmap, the entire mapping is synced back to the CPU.
	 * Since mappings are made at a page granularity regardless of the
	 * underlying buffer's size, they can cover other data as well. If a
	 * user requires a mapping be synced before unmapping, they are
	 * responsible for calling `gxp_mapping_sync()` before hand.
	 */
	gxp_dma_unmap_sg(gxp, mapping->core_list, mapping->sgt.sgl,
			 mapping->sgt.orig_nents, mapping->dir,
			 DMA_ATTR_SKIP_CPU_SYNC);

	/* Unpin the user pages */
	for_each_sg_page(mapping->sgt.sgl, &sg_iter, mapping->sgt.orig_nents,
			 0) {
		page = sg_page_iter_page(&sg_iter);
		if (mapping->dir == DMA_FROM_DEVICE ||
		    mapping->dir == DMA_BIDIRECTIONAL) {
			set_page_dirty(page);
		}

		unpin_user_page(page);
	}

	/* Free the mapping book-keeping */
	sg_free_table(&mapping->sgt);
	kfree(mapping);
}

int gxp_mapping_sync(struct gxp_dev *gxp, struct gxp_mapping *mapping,
		     u32 offset, u32 size, bool for_cpu)
{
	struct scatterlist *sg, *start_sg = NULL, *end_sg = NULL;
	int nelems = 0, cur_offset = 0, ret = 0, i;
	u64 start, end;
	unsigned int start_diff = 0, end_diff = 0;

	/*
	 * Valid input requires
	 * - size > 0 (offset + size != offset)
	 * - offset + size does not overflow (offset + size > offset)
	 * - the mapped range falls within [0 : mapping->size]
	 */
	if (offset + size <= offset ||
	    offset + size > mapping->size)
		return -EINVAL;

	/*
	 * Mappings are created at a PAGE_SIZE granularity, however other data
	 * which is not part of the mapped buffer may be present in the first
	 * and last pages of the buffer's scattergather list.
	 *
	 * To ensure only the intended data is actually synced, iterate through
	 * the scattergather list, to find the first and last `scatterlist`s
	 * which contain the range of the buffer to sync.
	 *
	 * After those links are found, change their offset/lengths so that
	 * `dma_map_sg_for_*()` will only sync the requested region.
	 */
	start = (mapping->host_address & ~PAGE_MASK) + offset;
	end = start + size;
	for_each_sg(mapping->sgt.sgl, sg, mapping->sgt.orig_nents, i) {
		if (end <= cur_offset)
			break;
		if (cur_offset <= start && start < cur_offset + sg->length) {
			start_sg = sg;
			start_diff = start - cur_offset;
		}
		if (start_sg)
			nelems++;
		cur_offset += sg->length;
		end_sg = sg;
	}
	end_diff = cur_offset - end;

	/* Make sure a valid starting scatterlist was found for the start */
	if (!start_sg)
		return -EINVAL;

	/*
	 * Since the scatter-gather list of the mapping is modified while it is
	 * being synced, only one sync for a given mapping can occur at a time.
	 * Rather than maintain a mutex for every mapping, lock the mapping list
	 * mutex, making all syncs mutually exclusive.
	 */
	mutex_lock(&gxp->mappings->lock);

	start_sg->offset += start_diff;
	start_sg->dma_address += start_diff;
	start_sg->length -= start_diff;
	start_sg->dma_length -= start_diff;
	end_sg->length -= end_diff;
	end_sg->dma_length -= end_diff;

	if (for_cpu)
		gxp_dma_sync_sg_for_cpu(gxp, start_sg, nelems, mapping->dir);
	else
		gxp_dma_sync_sg_for_device(gxp, start_sg, nelems, mapping->dir);

	/*
	 * Return the start and end scatterlists' offset/lengths to their
	 * original values for the next time they need to be synced/unmapped.
	 */
	end_sg->length += end_diff;
	end_sg->dma_length += end_diff;
	start_sg->offset -= start_diff;
	start_sg->dma_address -= start_diff;
	start_sg->length += start_diff;
	start_sg->dma_length += start_diff;

	mutex_unlock(&gxp->mappings->lock);

	return ret;
}

int gxp_mapping_put(struct gxp_dev *gxp, struct gxp_mapping *map)
{
	struct rb_node **link;
	struct rb_node *parent = NULL;
	u64 device_address = map->device_address;
	struct gxp_mapping *this;

	link = &gxp->mappings->rb.rb_node;

	mutex_lock(&gxp->mappings->lock);

	/* Figure out where to put new node */
	while (*link) {
		parent = *link;
		this = rb_entry(parent, struct gxp_mapping, node);

		if (this->device_address > device_address)
			link = &(*link)->rb_left;
		else if (this->device_address < device_address)
			link = &(*link)->rb_right;
		else
			goto out;
	}

	/* Add new node and rebalance tree. */
	rb_link_node(&map->node, parent, link);
	rb_insert_color(&map->node, &gxp->mappings->rb);

	mutex_unlock(&gxp->mappings->lock);

	return 0;

out:
	mutex_unlock(&gxp->mappings->lock);
	dev_err(gxp->dev, "Duplicate mapping: 0x%llx", map->device_address);
	return -EINVAL;
}

struct gxp_mapping *gxp_mapping_get(struct gxp_dev *gxp, u64 device_address)
{
	struct rb_node *node;
	struct gxp_mapping *this;

	mutex_lock(&gxp->mappings->lock);

	node = gxp->mappings->rb.rb_node;

	while (node) {
		this = rb_entry(node, struct gxp_mapping, node);

		if (this->device_address > device_address) {
			node = node->rb_left;
		} else if (this->device_address < device_address) {
			node = node->rb_right;
		} else {
			mutex_unlock(&gxp->mappings->lock);
			return this;  /* Found it */
		}
	}

	mutex_unlock(&gxp->mappings->lock);

	dev_err(gxp->dev, "Mapping not found: 0x%llx", device_address);
	return NULL;
}

struct gxp_mapping *gxp_mapping_get_host(struct gxp_dev *gxp, u64 host_address)
{
	struct rb_node *node;
	struct gxp_mapping *this;

	mutex_lock(&gxp->mappings->lock);

	/* Iterate through the elements in the rbtree */
	for (node = rb_first(&gxp->mappings->rb); node; node = rb_next(node)) {
		this = rb_entry(node, struct gxp_mapping, node);
		if (this->host_address == host_address) {
			mutex_unlock(&gxp->mappings->lock);
			return this;
		}
	}

	mutex_unlock(&gxp->mappings->lock);

	return NULL;
}

void gxp_mapping_remove(struct gxp_dev *gxp, struct gxp_mapping *map)
{
	rb_erase(&map->node, &gxp->mappings->rb);
}
