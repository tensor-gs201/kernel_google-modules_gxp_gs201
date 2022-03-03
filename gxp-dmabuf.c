// SPDX-License-Identifier: GPL-2.0
/*
 * Support for using dma-bufs.
 *
 * Copyright (C) 2022 Google LLC
 */

#include <linux/dma-buf.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>

#include "gxp-dma.h"
#include "gxp-dmabuf.h"

struct gxp_dmabuf_mapping {
	struct gxp_mapping mapping;
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attachment;
	/*
	 * For normal mappings, the `sg_table` is embedded directly in the
	 * `gxp_mapping` and populated by `sg_alloc_table_from_pages()`.
	 * For dma-bufs however, a pointer to the `sg_table` is returned by
	 * `dma_buf_map_attachment()`.
	 *
	 * Rather than manage the memory of `gxp_mapping`'s `sg_table`
	 * independently so it can contain a pointer, dma-bufs store their
	 * `sg_table` pointer here and ignore `mapping->sgt`.
	 */
	struct sg_table *sgt;
};

struct gxp_mapping *gxp_dmabuf_map(struct gxp_dev *gxp, uint core_list, int fd,
				   u32 flags, enum dma_data_direction dir)
{
	struct dma_buf *dmabuf;
	struct dma_buf_attachment *attachment;
	struct sg_table *sgt;
	struct gxp_dmabuf_mapping *dmabuf_mapping;
	int ret = 0;

	dmabuf = dma_buf_get(fd);
	if (IS_ERR(dmabuf)) {
		dev_err(gxp->dev, "Failed to get dma-buf to map (ret=%ld)\n",
			PTR_ERR(dmabuf));
		return ERR_CAST(dmabuf);
	}

	attachment = dma_buf_attach(dmabuf, gxp->dev);
	if (IS_ERR(attachment)) {
		dev_err(gxp->dev, "Failed to attach dma-buf to map (ret=%ld)\n",
			PTR_ERR(attachment));
		ret = PTR_ERR(attachment);
		goto err_attach;
	}

	sgt = gxp_dma_map_dmabuf_attachment(gxp, core_list, attachment, dir);
	if (IS_ERR(sgt)) {
		dev_err(gxp->dev,
			"Failed to map dma-buf attachment (ret=%ld)\n",
			PTR_ERR(sgt));
		ret = PTR_ERR(sgt);
		goto err_map_attachment;
	}

	dmabuf_mapping = kzalloc(sizeof(*dmabuf_mapping), GFP_KERNEL);
	if (!dmabuf_mapping) {
		ret = -ENOMEM;
		goto err_alloc_mapping;
	}

	/* dma-buf mappings are indicated by a host_address of 0 */
	dmabuf_mapping->mapping.host_address = 0;
	dmabuf_mapping->mapping.core_list = core_list;
	dmabuf_mapping->mapping.device_address = sg_dma_address(sgt->sgl);
	dmabuf_mapping->mapping.dir = dir;
	dmabuf_mapping->dmabuf = dmabuf;
	dmabuf_mapping->attachment = attachment;
	dmabuf_mapping->sgt = sgt;
	ret = gxp_mapping_put(gxp, &dmabuf_mapping->mapping);
	if (ret) {
		dev_err(gxp->dev,
			"Failed to store mapping for dma-buf (ret=%d)\n", ret);
		goto err_put_mapping;
	}

	return &dmabuf_mapping->mapping;

err_put_mapping:
	kfree(dmabuf_mapping);
err_alloc_mapping:
	gxp_dma_unmap_dmabuf_attachment(gxp, core_list, attachment, sgt, dir);
err_map_attachment:
	dma_buf_detach(dmabuf, attachment);
err_attach:
	dma_buf_put(dmabuf);
	return ERR_PTR(ret);
}

void gxp_dmabuf_unmap(struct gxp_dev *gxp, dma_addr_t device_address)
{
	struct gxp_dmabuf_mapping *dmabuf_mapping;
	struct gxp_mapping *mapping;

	/*
	 * Fetch and remove the internal mapping records.
	 * If host_address is not 0, the provided device_address belongs to a
	 * non-dma-buf mapping.
	 */
	mapping = gxp_mapping_get(gxp, device_address);
	if (IS_ERR_OR_NULL(mapping) || mapping->host_address) {
		dev_warn(gxp->dev, "No dma-buf mapped for given IOVA\n");
		return;
	}

	gxp_mapping_remove(gxp, mapping);

	/* Unmap and detach the dma-buf */
	dmabuf_mapping =
		container_of(mapping, struct gxp_dmabuf_mapping, mapping);

	gxp_dma_unmap_dmabuf_attachment(gxp, mapping->core_list,
					dmabuf_mapping->attachment,
					dmabuf_mapping->sgt, mapping->dir);
	dma_buf_detach(dmabuf_mapping->dmabuf, dmabuf_mapping->attachment);
	dma_buf_put(dmabuf_mapping->dmabuf);

	kfree(dmabuf_mapping);
}
