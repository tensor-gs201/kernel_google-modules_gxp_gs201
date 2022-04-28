// SPDX-License-Identifier: GPL-2.0
/*
 * GXP telemetry support
 *
 * Copyright (C) 2021 Google LLC
 */

#include <linux/slab.h>
#include <linux/wait.h>

#include "gxp-config.h"
#include "gxp-dma.h"
#include "gxp-firmware.h"
#include "gxp-firmware-data.h"
#include "gxp-host-device-structs.h"
#include "gxp-notification.h"
#include "gxp-telemetry.h"
#include "gxp-vd.h"

static inline bool is_core_telemetry_enabled(struct gxp_dev *gxp, uint core,
					     u8 type)
{
	u32 device_status =
		gxp_fw_data_get_telemetry_device_status(gxp, core, type);

	return device_status & GXP_TELEMETRY_DEVICE_STATUS_ENABLED;
}

static void telemetry_status_notification_work(struct work_struct *work)
{
	struct gxp_telemetry_work *telem_work =
		container_of(work, struct gxp_telemetry_work, work);
	struct gxp_dev *gxp = telem_work->gxp;
	uint core = telem_work->core;
	struct gxp_telemetry_manager *mgr = telem_work->gxp->telemetry_mgr;

	/* Wake any threads waiting on an telemetry disable ACK */
	wake_up(&mgr->waitq);

	/* Signal the appropriate eventfd for any active telemetry types */
	mutex_lock(&mgr->lock);

	if (is_core_telemetry_enabled(gxp, core, GXP_TELEMETRY_TYPE_LOGGING) &&
	    mgr->logging_efd)
		eventfd_signal(mgr->logging_efd, 1);

	if (is_core_telemetry_enabled(gxp, core, GXP_TELEMETRY_TYPE_TRACING) &&
	    mgr->tracing_efd)
		eventfd_signal(mgr->tracing_efd, 1);

	mutex_unlock(&mgr->lock);
}

int gxp_telemetry_init(struct gxp_dev *gxp)
{
	struct gxp_telemetry_manager *mgr;
	uint i;

	mgr = devm_kzalloc(gxp->dev, sizeof(*mgr), GFP_KERNEL);
	if (!mgr)
		return -ENOMEM;

	mutex_init(&mgr->lock);
	for (i = 0; i < GXP_NUM_CORES; i++) {
		INIT_WORK(&mgr->notification_works[i].work,
			  telemetry_status_notification_work);
		mgr->notification_works[i].gxp = gxp;
		mgr->notification_works[i].core = i;

	}
	init_waitqueue_head(&mgr->waitq);

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

/*
 * Forward declaration of telemetry_disable_locked() so that
 * gxp_telemetry_vma_close() can invoke the locked version without having to
 * release `telemetry_mgr->lock` and calling gxp_telemetry_disable().
 */
static int telemetry_disable_locked(struct gxp_dev *gxp, u8 type);

static void gxp_telemetry_vma_close(struct vm_area_struct *vma)
{
	struct gxp_dev *gxp;
	struct buffer_data *data;
	u8 type;
	int i;
	uint virt_core;
	struct gxp_virtual_device *vd;

	gxp = ((struct telemetry_vma_data *)vma->vm_private_data)->gxp;
	data = ((struct telemetry_vma_data *)vma->vm_private_data)->data;
	type = ((struct telemetry_vma_data *)vma->vm_private_data)->type;

	mutex_lock(&gxp->telemetry_mgr->lock);

	down_read(&gxp->vd_semaphore);
	if (refcount_dec_and_test(&data->ref_count)) {
		if (data->host_status & GXP_TELEMETRY_HOST_STATUS_ENABLED)
			telemetry_disable_locked(gxp, type);

		for (i = 0; i < GXP_NUM_CORES; i++) {
			vd = gxp->core_to_vd[i];
			if (vd != NULL) {
				virt_core =
					gxp_vd_phys_core_to_virt_core(vd, i);
				gxp_dma_free_coherent(gxp, vd, BIT(virt_core),
						      data->size,
						      data->buffers[i],
						      data->buffer_daddrs[i]);
			} else {
				gxp_dma_free_coherent(gxp, NULL, 0, data->size,
						      data->buffers[i],
						      data->buffer_daddrs[i]);
			}
		}
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
	up_read(&gxp->vd_semaphore);

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
 * Caller must hold gxp->vd_semaphore for reading.
 *
 * Return: A pointer to the `struct buffer_data` if successful, NULL otherwise
 */
static struct buffer_data *allocate_telemetry_buffers(struct gxp_dev *gxp,
						      size_t size)
{
	struct buffer_data *data;
	int i;
	uint virt_core;
	struct gxp_virtual_device *vd;

	data = kzalloc(sizeof(*data), GFP_KERNEL);
	if (!data)
		return NULL;

	/* Allocate cache-coherent buffers for logging/tracing to */
	for (i = 0; i < GXP_NUM_CORES; i++) {
		/*
		 * If the core is not allocated, we cannot map the buffer on
		 * that core.
		 */
		vd = gxp->core_to_vd[i];
		if (vd != NULL) {
			virt_core = gxp_vd_phys_core_to_virt_core(vd, i);
			data->buffers[i] = gxp_dma_alloc_coherent(
				gxp, vd, BIT(virt_core), size,
				&data->buffer_daddrs[i], GFP_KERNEL, 0);
		} else {
			data->buffers[i] =
				gxp_dma_alloc_coherent(gxp, NULL, 0, size,
						       &data->buffer_daddrs[i],
						       GFP_KERNEL, 0);
		}
		if (!data->buffers[i])
			goto err_alloc;
	}
	data->size = size;
	refcount_set(&data->ref_count, 1);

	return data;

err_alloc:
	for (; i > 0; i--) {
		vd = gxp->core_to_vd[i-1];
		if (vd != NULL) {
			virt_core = gxp_vd_phys_core_to_virt_core(vd, i);
			gxp_dma_free_coherent(gxp, vd, BIT(virt_core), size,
					      data->buffers[i - 1],
					      data->buffer_daddrs[i - 1]);
		} else {
			gxp_dma_free_coherent(gxp, NULL, 0, size,
					      data->buffers[i - 1],
					      data->buffer_daddrs[i - 1]);
		}
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
	struct gxp_virtual_device *vd;
	uint virt_core;

	if (!gxp->telemetry_mgr)
		return -ENODEV;

	/* Total size must divide evenly into 1 page-aligned buffer per core */
	if (!total_size || !IS_ALIGNED(total_size, PAGE_SIZE * GXP_NUM_CORES))
		return -EINVAL;

	mutex_lock(&gxp->telemetry_mgr->lock);
	down_read(&gxp->vd_semaphore);

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

	up_read(&gxp->vd_semaphore);
	mutex_unlock(&gxp->telemetry_mgr->lock);

	return 0;

err_free_buffers:
	for (i = 0; i < GXP_NUM_CORES; i++) {
		vd = gxp->core_to_vd[i];
		if (vd != NULL) {
			virt_core = gxp_vd_phys_core_to_virt_core(vd, i);
			gxp_dma_free_coherent(gxp, vd, BIT(virt_core),
					      data->size, data->buffers[i],
					      data->buffer_daddrs[i]);
		} else {
			gxp_dma_free_coherent(gxp, NULL, 0, data->size,
					      data->buffers[i],
					      data->buffer_daddrs[i]);
		}
	}
	kfree(data);

err_free_vma_data:
	kfree(vma_data);

err:
	up_read(&gxp->vd_semaphore);
	mutex_unlock(&gxp->telemetry_mgr->lock);
	return ret;
}

int gxp_telemetry_enable(struct gxp_dev *gxp, u8 type)
{
	struct buffer_data *data;
	int ret = 0;
	uint core;

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
	data->host_status |= GXP_TELEMETRY_HOST_STATUS_ENABLED;
	gxp_fw_data_set_telemetry_descriptors(gxp, type, data->host_status,
					      data->buffer_daddrs, data->size);

	/* Notify any running cores that firmware-data was updated */
	down_read(&gxp->vd_semaphore);
	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (gxp_is_fw_running(gxp, core))
			gxp_notification_send(gxp, core,
					      CORE_NOTIF_TELEMETRY_STATUS);
	}
	up_read(&gxp->vd_semaphore);

out:
	mutex_unlock(&gxp->telemetry_mgr->lock);

	return ret;
}

/**
 * notify_core_and_wait_for_disable() - Notify a core that telemetry state has
 *                                      been changed by the host and wait for
 *                                      the core to stop using telemetry.
 * @gxp: The GXP device telemetry is changing for
 * @core: The core in @gxp to notify of the telemetry state change
 * @type: Either `GXP_TELEMETRY_TYPE_LOGGING` or `GXP_TELEMETRY_TYPE_TRACING`
 *
 * Caller must hold @gxp's virtual device lock
 *
 * Return:
 * * 0      - Firmware on @core is no longer using telemetry of @type
 * * -ENXIO - Firmware on @core is unresponsive
 */
static int notify_core_and_wait_for_disable(struct gxp_dev *gxp, uint core,
					    u8 type)
{
	uint retries_left = 50;

	gxp_notification_send(gxp, core, CORE_NOTIF_TELEMETRY_STATUS);

	/* Wait for ACK from firmware */
	while (is_core_telemetry_enabled(gxp, core, type) &&
	       gxp_is_fw_running(gxp, core) && retries_left) {
		/* Release vd_semaphore while waiting */
		up_read(&gxp->vd_semaphore);

		/*
		 * The VD lock must be held to check if firmware is running, so
		 * the wait condition is only whether the firmware data has been
		 * updated to show the core disabling telemetry.
		 *
		 * If a core does stop running firmware while this function is
		 * asleep, it will be seen at the next timeout.
		 */
		wait_event_timeout(gxp->telemetry_mgr->waitq,
				   !is_core_telemetry_enabled(gxp, core, type),
				   msecs_to_jiffies(10));
		retries_left--;

		down_read(&gxp->vd_semaphore);
	}

	/*
	 * If firmware has stopped running altogether, that is sufficient to be
	 * considered disabled. If firmware is started on this core again, it
	 * is responsible for clearing its status.
	 */
	if (unlikely(is_core_telemetry_enabled(gxp, core, type) &&
		     gxp_is_fw_running(gxp, core)))
		return -ENXIO;

	return 0;
}

/**
 * telemetry_disable_locked() - Helper function to break out the actual
 *                              process of disabling telemetry so that it
 *                              can be invoked by internal functions that are
 *                              already holding the telemetry lock.
 * @gxp: The GXP device to disable either logging or tracing for
 * @type: Either `GXP_TELEMETRY_TYPE_LOGGING` or `GXP_TELEMETRY_TYPE_TRACING`
 *
 * Caller must hold `telemetry_mgr->lock`.
 *
 * Return:
 * * 0       - Success
 * * -EINVAL - The @type provided is not valid
 * * -ENXIO  - Buffers for @type have not been created/mapped yet
 */
static int telemetry_disable_locked(struct gxp_dev *gxp, u8 type)
{
	struct buffer_data *data;
	int ret = 0;
	dma_addr_t null_daddrs[GXP_NUM_CORES] = {0};
	uint core;

	/* Cleanup telemetry manager's book-keeping */
	switch (type) {
	case GXP_TELEMETRY_TYPE_LOGGING:
		data = gxp->telemetry_mgr->logging_buff_data;
		break;
	case GXP_TELEMETRY_TYPE_TRACING:
		data = gxp->telemetry_mgr->tracing_buff_data;
		break;
	default:
		return -EINVAL;
	}

	if (!data)
		return -ENXIO;

	if (!(data->host_status & GXP_TELEMETRY_HOST_STATUS_ENABLED))
		return 0;

	/* Clear the log buffer fields in firmware-data */
	data->host_status &= ~GXP_TELEMETRY_HOST_STATUS_ENABLED;
	gxp_fw_data_set_telemetry_descriptors(gxp, type, data->host_status,
					      null_daddrs, 0);

	/* Notify any running cores that firmware-data was updated */
	down_read(&gxp->vd_semaphore);
	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (gxp_is_fw_running(gxp, core)) {
			ret = notify_core_and_wait_for_disable(gxp, core, type);
			if (ret)
				dev_warn(
					gxp->dev,
					"%s: core%u failed to disable telemetry (type=%u, ret=%d)\n",
					__func__, core, type, ret);
		}
	}
	up_read(&gxp->vd_semaphore);

	return 0;
}

int gxp_telemetry_disable(struct gxp_dev *gxp, u8 type)
{
	int ret;

	mutex_lock(&gxp->telemetry_mgr->lock);

	ret = telemetry_disable_locked(gxp, type);

	mutex_unlock(&gxp->telemetry_mgr->lock);

	return ret;
}

int gxp_telemetry_register_eventfd(struct gxp_dev *gxp, u8 type, int fd)
{
	struct eventfd_ctx *new_ctx;
	struct eventfd_ctx **ctx_to_set = NULL;
	int ret = 0;

	new_ctx = eventfd_ctx_fdget(fd);
	if (IS_ERR(new_ctx))
		return PTR_ERR(new_ctx);

	mutex_lock(&gxp->telemetry_mgr->lock);

	switch (type) {
	case GXP_TELEMETRY_TYPE_LOGGING:
		ctx_to_set = &gxp->telemetry_mgr->logging_efd;
		break;
	case GXP_TELEMETRY_TYPE_TRACING:
		ctx_to_set = &gxp->telemetry_mgr->tracing_efd;
		break;
	default:
		ret = -EINVAL;
		goto out;
	}

	if (*ctx_to_set) {
		dev_warn(gxp->dev,
			 "Replacing existing telemetry eventfd (type=%u)\n",
			 type);
		eventfd_ctx_put(*ctx_to_set);
	}

	*ctx_to_set = new_ctx;

out:
	mutex_unlock(&gxp->telemetry_mgr->lock);
	return ret;
}

int gxp_telemetry_unregister_eventfd(struct gxp_dev *gxp, u8 type)
{
	int ret = 0;

	mutex_lock(&gxp->telemetry_mgr->lock);

	switch (type) {
	case GXP_TELEMETRY_TYPE_LOGGING:
		eventfd_ctx_put(gxp->telemetry_mgr->logging_efd);
		gxp->telemetry_mgr->logging_efd = NULL;
		break;
	case GXP_TELEMETRY_TYPE_TRACING:
		eventfd_ctx_put(gxp->telemetry_mgr->tracing_efd);
		gxp->telemetry_mgr->tracing_efd = NULL;
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&gxp->telemetry_mgr->lock);

	return ret;
}

struct work_struct *gxp_telemetry_get_notification_handler(struct gxp_dev *gxp,
							   uint core)
{
	struct gxp_telemetry_manager *mgr = gxp->telemetry_mgr;

	if (!mgr || core >= GXP_NUM_CORES)
		return NULL;

	return &mgr->notification_works[core].work;
}
