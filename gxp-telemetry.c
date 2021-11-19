// SPDX-License-Identifier: GPL-2.0
/*
 * GXP telemetry support
 *
 * Copyright (C) 2021 Google LLC
 */

#include <linux/slab.h>

#include "gxp-config.h"
#include "gxp-dma.h"
#include "gxp-firmware-data.h"
#include "gxp-telemetry.h"

int gxp_telemetry_init(struct gxp_dev *gxp)
{
	struct gxp_telemetry_manager *mgr;

	mgr = devm_kzalloc(gxp->dev, sizeof(*mgr), GFP_KERNEL);
	if (!mgr)
		return -ENOMEM;

	mutex_init(&mgr->lock);

	gxp->telemetry_mgr = mgr;

	return 0;
}

/* Wrapper struct to be used by the telemetry vma_ops. */
struct telemetry_vma_data {
	struct gxp_dev *gxp;
	struct buffer_data *data;
	u8 type;
};

static void gxp_telemetry_vma_open(struct vm_area_struct *vma)
{
	struct gxp_dev *gxp;
	struct buffer_data *data;

	gxp = ((struct telemetry_vma_data *)vma->vm_private_data)->gxp;
	data = ((struct telemetry_vma_data *)vma->vm_private_data)->data;

	mutex_lock(&gxp->telemetry_mgr->lock);

	refcount_inc(&data->ref_count);

	mutex_unlock(&gxp->telemetry_mgr->lock);
}

static void gxp_telemetry_vma_close(struct vm_area_struct *vma)
{
	struct gxp_dev *gxp;
	struct buffer_data *data;
	u8 type;
	int i;

	gxp = ((struct telemetry_vma_data *)vma->vm_private_data)->gxp;
	data = ((struct telemetry_vma_data *)vma->vm_private_data)->data;
	type = ((struct telemetry_vma_data *)vma->vm_private_data)->type;

	mutex_lock(&gxp->telemetry_mgr->lock);

	if (refcount_dec_and_test(&data->ref_count)) {
		if (data->enabled)
			gxp_telemetry_disable(gxp, type);

		for (i = 0; i < GXP_NUM_CORES; i++)
			gxp_dma_free_coherent(gxp, BIT(i), data->size,
					      data->buffers[i],
					      data->buffer_daddrs[i]);
		switch (type) {
		case GXP_TELEMETRY_TYPE_LOGGING:
			gxp->telemetry_mgr->logging_buff_data = NULL;
			break;
		case GXP_TELEMETRY_TYPE_TRACING:
			gxp->telemetry_mgr->tracing_buff_data = NULL;
			break;
		default:
			dev_warn(gxp->dev, "%s called with invalid type %u\n",
				 __func__, type);
		}
		kfree(data);
		kfree(vma->vm_private_data);
	}

	mutex_unlock(&gxp->telemetry_mgr->lock);
}

static const struct vm_operations_struct gxp_telemetry_vma_ops = {
	.open = gxp_telemetry_vma_open,
	.close = gxp_telemetry_vma_close,
};

/**
 * check_telemetry_type_availability() - Checks if @type is valid and whether
 *                                       buffers of that type already exists.
 * @gxp: The GXP device to check availability for
 * @type: Either `GXP_TELEMETRY_TYPE_LOGGING` or `GXP_TELEMETRY_TYPE_TRACING`
 *
 * Caller must hold the telemetry_manager's lock.
 *
 * Return:
 * * 0       - @type is valid and can have new buffers created
 * * -EBUSY  - Buffers already exist for @type
 * * -EINVAL - @type is not a valid telemetry type
 */
static int check_telemetry_type_availability(struct gxp_dev *gxp, u8 type)
{
	switch (type) {
	case GXP_TELEMETRY_TYPE_LOGGING:
		if (gxp->telemetry_mgr->logging_buff_data)
			return -EBUSY;
		break;
	case GXP_TELEMETRY_TYPE_TRACING:
		if (gxp->telemetry_mgr->tracing_buff_data)
			return -EBUSY;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/**
 * allocate_telemetry_buffers() - Allocate and populate a `struct buffer_data`,
 *                                including allocating and mapping one coherent
 *                                buffer of @size bytes per core.
 * @gxp: The GXP device to allocate the buffers for
 * @size: The size of buffer to allocate for each core
 *
 * Caller must hold the telemetry_manager's lock.
 *
 * Return: A pointer to the `struct buffer_data` if successful, NULL otherwise
 */
static struct buffer_data *allocate_telemetry_buffers(struct gxp_dev *gxp,
						      size_t size)
{
	struct buffer_data *data;
	int i;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return NULL;

	/* Allocate cache-coherent buffers for logging/tracing to */
	for (i = 0; i < GXP_NUM_CORES; i++) {
		data->buffers[i] =
			gxp_dma_alloc_coherent(gxp, BIT(i), size,
					       &data->buffer_daddrs[i],
					       GFP_KERNEL, 0);
		if (!data->buffers[i])
			goto err_alloc;
	}
	data->size = size;
	refcount_set(&data->ref_count, 1);
	data->enabled = false;

	return data;

err_alloc:
	for (; i > 0; i--) {
		gxp_dma_free_coherent(gxp, BIT(i - 1), size,
				      data->buffers[i - 1],
				      data->buffer_daddrs[i - 1]);
	}
	kfree(data);

	return NULL;
}

/**
 * remap_telemetry_buffers() - Remaps a set of telemetry buffers into a
 *                             user-space vm_area.
 * @gxp: The GXP device the buffers were allocated for
 * @vma: A vm area to remap the buffers into
 * @data: The data describing a set of telemetry buffers to remap
 *
 * Caller must hold the telemetry_manager's lock.
 *
 * Return:
 * * 0         - Success
 * * otherwise - Error returned by `remap_pfn_range()`
 */
static int remap_telemetry_buffers(struct gxp_dev *gxp,
				   struct vm_area_struct *vma,
				   struct buffer_data *data)
{
	unsigned long orig_pgoff = vma->vm_pgoff;
	int i;
	unsigned long offset;
	phys_addr_t phys;
	int ret = 0;

	/* mmap the buffers */
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);
	vma->vm_flags |= VM_DONTCOPY | VM_DONTEXPAND | VM_DONTDUMP;
	vma->vm_pgoff = 0;

	for (i = 0; i < GXP_NUM_CORES; i++) {
		/*
		 * Remap each core's buffer a page at a time, in case it is not
		 * physically contiguous.
		 */
		for (offset = 0; offset < data->size; offset += PAGE_SIZE) {
			/*
			 * `virt_to_phys()` does not work on memory allocated
			 * by `dma_alloc_coherent()`, so we have to use
			 * `iommu_iova_to_phys()` instead. Since all buffers
			 * are mapped to the default domain as well as any per-
			 * core domains, we can use it here to get the physical
			 * address of any valid IOVA, regardless of its core.
			 */
			phys = iommu_iova_to_phys(
				iommu_get_domain_for_dev(gxp->dev),
				data->buffer_daddrs[i] + offset);
			ret = remap_pfn_range(
				vma, vma->vm_start + data->size * i + offset,
				phys >> PAGE_SHIFT, PAGE_SIZE,
				vma->vm_page_prot);
			if (ret)
				goto out;
		}
	}

out:
	vma->vm_pgoff = orig_pgoff;
	vma->vm_ops = &gxp_telemetry_vma_ops;

	return ret;
}

int gxp_telemetry_mmap_buffers(struct gxp_dev *gxp, u8 type,
			       struct vm_area_struct *vma)
{
	int ret = 0;
	struct telemetry_vma_data *vma_data;
	size_t total_size = vma->vm_end - vma->vm_start;
	size_t size = total_size / GXP_NUM_CORES;
	struct buffer_data *data;
	int i;

	if (!gxp->telemetry_mgr)
		return -ENODEV;

	/* Total size must divide evenly into 1 page-aligned buffer per core */
	if (!total_size || !IS_ALIGNED(total_size, PAGE_SIZE * GXP_NUM_CORES))
		return -EINVAL;

	mutex_lock(&gxp->telemetry_mgr->lock);

	ret = check_telemetry_type_availability(gxp, type);
	if (ret)
		goto err;

	vma_data = kmalloc(sizeof(*vma_data), GFP_KERNEL);
	if (!vma_data) {
		ret = -ENOMEM;
		goto err;
	}

	data = allocate_telemetry_buffers(gxp, size);
	if (!data) {
		ret = -ENOMEM;
		goto err_free_vma_data;
	}

	ret = remap_telemetry_buffers(gxp, vma, data);
	if (ret)
		goto err_free_buffers;

	vma_data->gxp = gxp;
	vma_data->data = data;
	vma_data->type = type;
	vma->vm_private_data = vma_data;

	/* Save book-keeping on the buffers in the telemetry manager */
	if (type == GXP_TELEMETRY_TYPE_LOGGING)
		gxp->telemetry_mgr->logging_buff_data = data;
	else /* type == GXP_TELEMETRY_TYPE_TRACING */
		gxp->telemetry_mgr->tracing_buff_data = data;

	mutex_unlock(&gxp->telemetry_mgr->lock);

	return 0;

err_free_buffers:
	for (i = 0; i < GXP_NUM_CORES; i++)
		gxp_dma_free_coherent(gxp, BIT(i), data->size, data->buffers[i],
				      data->buffer_daddrs[i]);
	kfree(data);

err_free_vma_data:
	kfree(vma_data);

err:
	mutex_unlock(&gxp->telemetry_mgr->lock);
	return ret;
}

int gxp_telemetry_enable(struct gxp_dev *gxp, u8 type)
{
	struct buffer_data *data;
	int ret = 0;

	mutex_lock(&gxp->telemetry_mgr->lock);

	switch (type) {
	case GXP_TELEMETRY_TYPE_LOGGING:
		data = gxp->telemetry_mgr->logging_buff_data;
		break;
	case GXP_TELEMETRY_TYPE_TRACING:
		data = gxp->telemetry_mgr->tracing_buff_data;
		break;
	default:
		ret = -EINVAL;
		goto out;
	}

	if (!data) {
		ret = -ENXIO;
		goto out;
	}

	/* Populate the buffer fields in firmware-data */
	gxp_fw_data_set_telemetry_descriptors(
		gxp, type, (u32 *)data->buffer_daddrs, data->size);

	/* TODO(b/202937192) To be done in a future CL */
	/* Notify any running cores that firmware-data was updated */

	data->enabled = true;

out:
	mutex_unlock(&gxp->telemetry_mgr->lock);

	return ret;
}

int gxp_telemetry_disable(struct gxp_dev *gxp, u8 type)
{
	struct buffer_data *data;
	int ret = 0;
	u32 null_daddrs[GXP_NUM_CORES] = {0};

	mutex_lock(&gxp->telemetry_mgr->lock);

	/* Cleanup telemetry manager's book-keeping */
	switch (type) {
	case GXP_TELEMETRY_TYPE_LOGGING:
		data = gxp->telemetry_mgr->logging_buff_data;
		break;
	case GXP_TELEMETRY_TYPE_TRACING:
		data = gxp->telemetry_mgr->tracing_buff_data;
		break;
	default:
		ret = -EINVAL;
		goto out;
	}

	if (!data) {
		ret = -ENXIO;
		goto out;
	}

	if (!data->enabled)
		goto out;

	/* Clear the log buffer fields in firmware-data */
	gxp_fw_data_set_telemetry_descriptors(gxp, type, null_daddrs, 0);

	/* TODO(b/202937192) To be done in a future CL */
	/* Notify any running cores that firmware-data was updated */
	/* Wait for ACK from firmware */

	data->enabled = false;

out:
	mutex_unlock(&gxp->telemetry_mgr->lock);

	return ret;
}
