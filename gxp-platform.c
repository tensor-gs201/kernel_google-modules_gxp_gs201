// SPDX-License-Identifier: GPL-2.0
/*
 * Platform device driver for GXP.
 *
 * Copyright (C) 2021 Google LLC
 */

#ifdef CONFIG_ANDROID
#include <linux/platform_data/sscoredump.h>
#endif

#include <linux/acpi.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/genalloc.h>
#include <linux/kthread.h>
#include <linux/log2.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/uaccess.h>
#ifdef CONFIG_ANDROID
#include <soc/google/tpu-ext.h>
#endif

#include "gxp.h"
#include "gxp-debug-dump.h"
#include "gxp-debugfs.h"
#include "gxp-dma.h"
#include "gxp-firmware.h"
#include "gxp-firmware-data.h"
#include "gxp-internal.h"
#include "gxp-mailbox.h"
#include "gxp-mailbox-driver.h"
#include "gxp-mapping.h"
#include "gxp-pm.h"
#include "gxp-telemetry.h"
#include "gxp-vd.h"

#ifdef CONFIG_ANDROID
static struct sscd_platform_data gxp_sscd_pdata;

static void gxp_sscd_release(struct device *dev)
{
	pr_debug("%s\n", __func__);
}

static struct platform_device gxp_sscd_dev = {
	.name = GXP_DRIVER_NAME,
	.driver_override = SSCD_NAME,
	.id = -1,
	.dev = {
		.platform_data = &gxp_sscd_pdata,
		.release = gxp_sscd_release,
	},
};
#endif  // CONFIG_ANDROID

static int gxp_open(struct inode *inode, struct file *file)
{
	struct gxp_client *client;
	struct gxp_dev *gxp = container_of(file->private_data, struct gxp_dev,
					   misc_dev);

	client = gxp_client_create(gxp);
	if (IS_ERR(client))
		return PTR_ERR(client);

	file->private_data = client;
	return 0;
}

static int gxp_release(struct inode *inode, struct file *file)
{
	struct gxp_client *client = file->private_data;

	/*
	 * TODO (b/184572070): Unmap buffers and drop mailbox responses
	 * belonging to the client
	 */
	gxp_client_destroy(client);
	return 0;
}

static inline enum dma_data_direction mapping_flags_to_dma_dir(u32 flags)
{
	switch (flags & 0x3) {
	case 0x0: /* 0b00 */
		return DMA_BIDIRECTIONAL;
	case 0x1: /* 0b01 */
		return DMA_TO_DEVICE;
	case 0x2: /* 0b10 */
		return DMA_FROM_DEVICE;
	}

	return DMA_NONE;
}

static int gxp_map_buffer(struct gxp_client *client,
			  struct gxp_map_ioctl __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	struct gxp_map_ioctl ibuf;
	struct gxp_mapping *map;
	int ret = 0;
	uint phys_core_list;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	phys_core_list = gxp_vd_virt_core_list_to_phys_core_list(
		client, ibuf.virtual_core_list);
	if (phys_core_list == 0)
		return -EINVAL;

	if (ibuf.size == 0)
		return -EINVAL;

	if (ibuf.host_address % L1_CACHE_BYTES || ibuf.size % L1_CACHE_BYTES) {
		dev_err(gxp->dev,
			"Mapped buffers must be cache line aligned and padded.\n");
		return -EINVAL;
	}

#ifndef CONFIG_GXP_HAS_SYSMMU
	/*
	 * TODO(b/193272602) On systems without a SysMMU, all attempts to map
	 * the same buffer must use the same mapping/bounce buffer or cores
	 * may corrupt each others' updates to the buffer. Once mirror mapping
	 * is supported, and a buffer can be mapped to multiple cores at once,
	 * attempting to remap a buffer can be considered an error and this
	 * check removed.
	 */
	/* Check if this buffer has already been mapped */
	map = gxp_mapping_get_host(gxp, ibuf.host_address);
	if (map) {
		ibuf.device_address = map->device_address;
		if (copy_to_user(argp, &ibuf, sizeof(ibuf)))
			return -EFAULT;

		map->map_count++;
		return ret;
	}
#endif

	map = gxp_mapping_create(gxp, phys_core_list, ibuf.host_address,
				 ibuf.size, /*gxp_dma_flags=*/0,
				 mapping_flags_to_dma_dir(ibuf.flags));
	if (IS_ERR(map))
		return PTR_ERR(map);

	ret = gxp_mapping_put(gxp, map);
	if (ret)
		goto error_destroy;

	ibuf.device_address = map->device_address;

	if (copy_to_user(argp, &ibuf, sizeof(ibuf))) {
		ret = -EFAULT;
		goto error_remove;
	}

	return ret;

error_remove:
	gxp_mapping_remove(gxp, map);
error_destroy:
	gxp_mapping_destroy(gxp, map);
	devm_kfree(gxp->dev, (void *)map);
	return ret;
}

static int gxp_unmap_buffer(struct gxp_client *client,
			    struct gxp_map_ioctl __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	struct gxp_map_ioctl ibuf;
	struct gxp_mapping *map;
	int ret = 0;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	map = gxp_mapping_get(gxp, ibuf.device_address);
	if (!map)
		return -EINVAL;

	WARN_ON(map->host_address != ibuf.host_address);
	if (--(map->map_count))
		return ret;

	gxp_mapping_remove(gxp, map);
	gxp_mapping_destroy(gxp, map);

	return ret;
}

static int gxp_sync_buffer(struct gxp_client *client,
			   struct gxp_sync_ioctl __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	struct gxp_sync_ioctl ibuf;
	struct gxp_mapping *map;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	map = gxp_mapping_get(gxp, ibuf.device_address);
	if (!map)
		return -EINVAL;

	return gxp_mapping_sync(gxp, map, ibuf.offset, ibuf.size,
				ibuf.flags == GXP_SYNC_FOR_CPU);
}

static int gxp_mailbox_command(struct gxp_client *client,
			       struct gxp_mailbox_command_ioctl __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	struct gxp_mailbox_command_ioctl ibuf;
	struct gxp_command cmd;
	struct buffer_descriptor buffer;
	int phys_core;
	int ret = 0;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf))) {
		dev_err(gxp->dev,
			"Unable to copy ioctl data from user-space\n");
		return -EFAULT;
	}

	phys_core = gxp_vd_virt_core_to_phys_core(client, ibuf.virtual_core_id);
	if (phys_core < 0) {
		dev_err(gxp->dev,
			"Mailbox command failed: Invalid virtual core id (%u)\n",
			ibuf.virtual_core_id);
		return -EINVAL;
	}

	if (!gxp_is_fw_running(gxp, phys_core)) {
		dev_err(gxp->dev,
			"Cannot process mailbox command for core %d when firmware isn't running\n",
			phys_core);
		return -EINVAL;
	}

	if (gxp->mailbox_mgr == NULL || gxp->mailbox_mgr->mailboxes == NULL ||
	    gxp->mailbox_mgr->mailboxes[phys_core] == NULL) {
		dev_err(gxp->dev, "Mailbox not initialized for core %d\n",
			phys_core);
		return -EIO;
	}

	/* Pack the command structure */
	buffer.address = ibuf.device_address;
	buffer.size = ibuf.size;
	buffer.flags = ibuf.flags;
	/* cmd.seq is assigned by mailbox implementation */
	cmd.code = GXP_MBOX_CODE_DISPATCH; /* All IOCTL commands are dispatch */
	cmd.priority = 0; /* currently unused */
	cmd.buffer_descriptor = buffer;

	ret = gxp_mailbox_execute_cmd_async(
		gxp->mailbox_mgr->mailboxes[phys_core], &cmd,
		&gxp->mailbox_resp_queues[phys_core], &gxp->mailbox_resps_lock,
		&gxp->mailbox_resp_waitqs[phys_core]);
	if (ret) {
		dev_err(gxp->dev, "Failed to enqueue mailbox command (ret=%d)\n",
			ret);
		return ret;
	}

	ibuf.sequence_number = cmd.seq;
	if (copy_to_user(argp, &ibuf, sizeof(ibuf))) {
		dev_err(gxp->dev, "Failed to copy back sequence number!\n");
		return -EFAULT;
	}

	return 0;
}

static int gxp_mailbox_response(struct gxp_client *client,
				struct gxp_mailbox_response_ioctl __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	u16 virtual_core_id;
	struct gxp_mailbox_response_ioctl ibuf;
	struct gxp_async_response *resp_ptr;
	int phys_core;
	unsigned long flags;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	virtual_core_id = ibuf.virtual_core_id;
	phys_core = gxp_vd_virt_core_to_phys_core(client, virtual_core_id);
	if (phys_core < 0) {
		dev_err(gxp->dev, "Mailbox response failed: Invalid virtual core id (%u)\n",
			virtual_core_id);
		return -EINVAL;
	}

	if (!gxp_is_fw_running(gxp, phys_core)) {
		dev_err(gxp->dev, "Cannot process mailbox response for core %d when firmware isn't running\n",
			phys_core);
		return -EINVAL;
	}

	spin_lock_irqsave(&gxp->mailbox_resps_lock, flags);

	/*
	 * No timeout is required since commands have a hard timeout after
	 * which the command is abandoned and a response with a failure
	 * status is added to the mailbox_resps queue.
	 *
	 * The "exclusive" version of wait_event is used since each wake
	 * corresponds to the addition of exactly one new response to be
	 * consumed. Therefore, only one waiting response ioctl can ever
	 * proceed per wake event.
	 */
	wait_event_exclusive_cmd(
		gxp->mailbox_resp_waitqs[phys_core],
		!list_empty(&(gxp->mailbox_resp_queues[phys_core])),
		/* Release the lock before sleeping */
		spin_unlock_irqrestore(&gxp->mailbox_resps_lock, flags),
		/* Reacquire the lock after waking */
		spin_lock_irqsave(&gxp->mailbox_resps_lock, flags));

	resp_ptr = list_first_entry(&(gxp->mailbox_resp_queues[phys_core]),
				    struct gxp_async_response, list_entry);

	/* Pop the front of the response list */
	list_del(&(resp_ptr->list_entry));

	spin_unlock_irqrestore(&gxp->mailbox_resps_lock, flags);

	ibuf.sequence_number = resp_ptr->resp.seq;
	switch (resp_ptr->resp.status) {
	case GXP_RESP_OK:
		ibuf.error_code = GXP_RESPONSE_ERROR_NONE;
		/* retval is only valid if status == GXP_RESP_OK */
		ibuf.cmd_retval = resp_ptr->resp.retval;
		break;
	case GXP_RESP_CANCELLED:
		ibuf.error_code = GXP_RESPONSE_ERROR_TIMEOUT;
		break;
	default:
		/* No other status values are valid at this point */
		WARN(true, "Completed response had invalid status %hu",
		     resp_ptr->resp.status);
		ibuf.error_code = GXP_RESPONSE_ERROR_INTERNAL;
		break;
	}

	/*
	 * We must be absolutely sure the timeout work has been cancelled
	 * and/or completed before freeing the `gxp_async_response`.
	 * There are 3 possible cases when we arrive at this point:
	 *   1) The response arrived normally and the timeout was cancelled
	 *   2) The response timedout and its timeout handler finished
	 *   3) The response handler and timeout handler raced, and the response
	 *      handler "cancelled" the timeout handler while it was already in
	 *      progress.
	 *
	 * This call handles case #3, and ensures any in-process timeout
	 * handler (which may reference the `gxp_async_response`) has
	 * been able to exit cleanly.
	 */
	cancel_delayed_work_sync(&resp_ptr->timeout_work);
	kfree(resp_ptr);

	if (copy_to_user(argp, &ibuf, sizeof(ibuf)))
		return -EFAULT;

	return 0;
}

static int gxp_get_specs(struct gxp_client *client,
			 struct gxp_specs_ioctl __user *argp)
{
	struct gxp_specs_ioctl ibuf;

	ibuf.core_count = GXP_NUM_CORES;
	ibuf.version_major = 0;
	ibuf.version_minor = 0;
	ibuf.version_build = 1;
	ibuf.threads_per_core = 1;
	ibuf.memory_per_core = 0;

	if (copy_to_user(argp, &ibuf, sizeof(ibuf)))
		return -EFAULT;

	return 0;
}

static int gxp_allocate_vd(struct gxp_client *client,
			   struct gxp_virtual_device_ioctl __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	struct gxp_virtual_device_ioctl ibuf;
	int ret = 0;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	if (ibuf.core_count == 0 || ibuf.core_count > GXP_NUM_CORES) {
		dev_err(gxp->dev, "Invalid core count (%u)\n", ibuf.core_count);
		return -EINVAL;
	}

	mutex_lock(&gxp->vd_lock);
	if (client->vd_allocated) {
		mutex_unlock(&gxp->vd_lock);
		dev_err(gxp->dev, "Virtual device was already allocated for client\n");
		return -EINVAL;
	}

	ret = gxp_vd_allocate(client, ibuf.core_count);
	mutex_unlock(&gxp->vd_lock);

	return ret;
}

static int
gxp_etm_trace_start_command(struct gxp_client *client,
			    struct gxp_etm_trace_start_ioctl __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	struct gxp_etm_trace_start_ioctl ibuf;
	int phys_core;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	ibuf.trace_ram_enable &= ETM_TRACE_LSB_MASK;
	ibuf.atb_enable &= ETM_TRACE_LSB_MASK;

	if (!ibuf.trace_ram_enable && !ibuf.atb_enable)
		return -EINVAL;

	if (!(ibuf.sync_msg_period == 0 ||
	    (ibuf.sync_msg_period <= ETM_TRACE_SYNC_MSG_PERIOD_MAX &&
	     ibuf.sync_msg_period >= ETM_TRACE_SYNC_MSG_PERIOD_MIN &&
	     is_power_of_2(ibuf.sync_msg_period))))
		return -EINVAL;

	if (ibuf.pc_match_mask_length > ETM_TRACE_PC_MATCH_MASK_LEN_MAX)
		return -EINVAL;

	phys_core = gxp_vd_virt_core_to_phys_core(client, ibuf.virtual_core_id);
	if (phys_core < 0) {
		dev_err(gxp->dev, "Trace start failed: Invalid virtual core id (%u)\n",
			ibuf.virtual_core_id);
		return -EINVAL;
	}

	/*
	 * TODO (b/185260919): Pass the etm trace configuration to system FW
	 * once communication channel between kernel and system FW is ready
	 * (b/185819530).
	 */

	return 0;
}

static int gxp_etm_trace_sw_stop_command(struct gxp_client *client,
					 __u16 __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	u16 virtual_core_id;
	int phys_core;

	if (copy_from_user(&virtual_core_id, argp, sizeof(virtual_core_id)))
		return -EFAULT;


	phys_core = gxp_vd_virt_core_to_phys_core(client, virtual_core_id);
	if (phys_core < 0) {
		dev_err(gxp->dev, "Trace stop via software trigger failed: Invalid virtual core id (%u)\n",
			virtual_core_id);
		return -EINVAL;
	}

	/*
	 * TODO (b/185260919): Pass the etm stop signal to system FW once
	 * communication channel between kernel and system FW is ready
	 * (b/185819530).
	 */

	return 0;
}

static int gxp_etm_trace_cleanup_command(struct gxp_client *client,
					 __u16 __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	u16 virtual_core_id;
	int phys_core;

	if (copy_from_user(&virtual_core_id, argp, sizeof(virtual_core_id)))
		return -EFAULT;

	phys_core = gxp_vd_virt_core_to_phys_core(client, virtual_core_id);
	if (phys_core < 0) {
		dev_err(gxp->dev, "Trace cleanup failed: Invalid virtual core id (%u)\n",
			virtual_core_id);
		return -EINVAL;
	}

	/*
	 * TODO (b/185260919): Pass the etm clean up signal to system FW once
	 * communication channel between kernel and system FW is ready
	 * (b/185819530).
	 */

	return 0;
}

static int
gxp_etm_get_trace_info_command(struct gxp_client *client,
			       struct gxp_etm_get_trace_info_ioctl __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	struct gxp_etm_get_trace_info_ioctl ibuf;
	int phys_core;
	u32 *trace_header;
	u32 *trace_data;
	int ret = 0;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	if (ibuf.type > 1)
		return -EINVAL;

	phys_core = gxp_vd_virt_core_to_phys_core(client, ibuf.virtual_core_id);
	if (phys_core < 0) {
		dev_err(gxp->dev, "Get trace info failed: Invalid virtual core id (%u)\n",
			ibuf.virtual_core_id);
		return -EINVAL;
	}

	trace_header = kzalloc(GXP_TRACE_HEADER_SIZE, GFP_KERNEL);
	trace_data = kzalloc(GXP_TRACE_RAM_SIZE, GFP_KERNEL);

	/*
	 * TODO (b/185260919): Get trace information from system FW once
	 * communication channel between kernel and system FW is ready
	 * (b/185819530).
	 */

	if (copy_to_user((void __user *)ibuf.trace_header_addr, trace_header,
			 GXP_TRACE_HEADER_SIZE)) {
		ret = -EFAULT;
		goto out;
	}

	if (ibuf.type == 1) {
		if (copy_to_user((void __user *)ibuf.trace_data_addr,
				 trace_data, GXP_TRACE_RAM_SIZE)) {
			ret = -EFAULT;
			goto out;
		}
	}

out:
	kfree(trace_header);
	kfree(trace_data);

	return ret;
}

static int gxp_enable_telemetry(struct gxp_client *client,
				__u8 __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	__u8 type;

	if (copy_from_user(&type, argp, sizeof(type)))
		return -EFAULT;

	if (type != GXP_TELEMETRY_TYPE_LOGGING &&
	    type != GXP_TELEMETRY_TYPE_TRACING)
		return -EINVAL;

	return gxp_telemetry_enable(gxp, type);
}

static int gxp_disable_telemetry(struct gxp_client *client, __u8 __user *argp)
{
	struct gxp_dev *gxp = client->gxp;
	__u8 type;

	if (copy_from_user(&type, argp, sizeof(type)))
		return -EFAULT;

	if (type != GXP_TELEMETRY_TYPE_LOGGING &&
	    type != GXP_TELEMETRY_TYPE_TRACING)
		return -EINVAL;

	return gxp_telemetry_disable(gxp, type);
}

static int gxp_map_tpu_mbx_queue(struct gxp_client *client,
				 struct gxp_tpu_mbx_queue_ioctl __user *argp)
{
#ifdef CONFIG_ANDROID
	struct gxp_dev *gxp = client->gxp;
	struct edgetpu_ext_mailbox_info *mbx_info;
	struct gxp_tpu_mbx_queue_ioctl ibuf;
	struct edgetpu_ext_client_info gxp_tpu_info;
	u32 phys_core_list = 0;
	u32 virtual_core_list;
	u32 core_count;
	int ret = 0;

	if (!gxp->tpu_dev.mbx_paddr) {
		dev_err(gxp->dev, "%s: TPU is not available for interop\n",
			__func__);
		return -EINVAL;
	}

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	virtual_core_list = ibuf.virtual_core_list;
	core_count = hweight_long(virtual_core_list);
	phys_core_list = gxp_vd_virt_core_list_to_phys_core_list(
		client, virtual_core_list);
	if (!phys_core_list) {
		dev_err(gxp->dev, "%s: invalid virtual core list 0x%x\n",
			__func__, virtual_core_list);
		return -EINVAL;
	}

	mbx_info =
		kmalloc(sizeof(struct edgetpu_ext_mailbox_info) + core_count *
			sizeof(struct edgetpu_ext_mailbox_descriptor),
			GFP_KERNEL);
	if (!mbx_info)
		return -ENOMEM;

	mutex_lock(&gxp->vd_lock);

	if (client->tpu_mbx_allocated) {
		dev_err(gxp->dev, "%s: Mappings already exist for TPU mailboxes\n",
			__func__);
		ret = -EBUSY;
		goto error;
	}

	gxp_tpu_info.tpu_fd = ibuf.tpu_fd;
	gxp_tpu_info.mbox_map = phys_core_list;
	gxp_tpu_info.attr = (struct edgetpu_mailbox_attr __user *)ibuf.attr_ptr;
	ret = edgetpu_ext_driver_cmd(gxp->tpu_dev.dev,
				     EDGETPU_EXTERNAL_CLIENT_TYPE_DSP,
				     ALLOCATE_EXTERNAL_MAILBOX, &gxp_tpu_info,
				     mbx_info);
	if (ret) {
		dev_err(gxp->dev, "%s: Failed to allocate ext tpu mailboxes %d\n",
			__func__, ret);
		goto error;
	}
	/* Align queue size to page size for iommu map. */
	mbx_info->cmdq_size = ALIGN(mbx_info->cmdq_size, PAGE_SIZE);
	mbx_info->respq_size = ALIGN(mbx_info->respq_size, PAGE_SIZE);

	ret = gxp_dma_map_tpu_buffer(gxp, phys_core_list, mbx_info);
	if (ret) {
		dev_err(gxp->dev, "%s: failed to map TPU mailbox buffer %d\n",
			__func__, ret);
		edgetpu_ext_driver_cmd(gxp->tpu_dev.dev,
				       EDGETPU_EXTERNAL_CLIENT_TYPE_DSP,
				       FREE_EXTERNAL_MAILBOX, &gxp_tpu_info,
				       NULL);
		goto error;
	}
	client->mbx_desc.phys_core_list = phys_core_list;
	client->mbx_desc.cmdq_size = mbx_info->cmdq_size;
	client->mbx_desc.respq_size = mbx_info->respq_size;
	client->tpu_mbx_allocated = true;

error:
	mutex_unlock(&gxp->vd_lock);

	kfree(mbx_info);
	return ret;
#else
	return -ENODEV;
#endif
}

static int gxp_unmap_tpu_mbx_queue(struct gxp_client *client,
				   struct gxp_tpu_mbx_queue_ioctl __user *argp)
{
#ifdef CONFIG_ANDROID
	struct gxp_dev *gxp = client->gxp;
	struct gxp_tpu_mbx_queue_ioctl ibuf;
	struct edgetpu_ext_client_info gxp_tpu_info;
	int ret = 0;

	if (copy_from_user(&ibuf, argp, sizeof(ibuf)))
		return -EFAULT;

	mutex_lock(&gxp->vd_lock);

	if (!client->tpu_mbx_allocated) {
		dev_err(gxp->dev, "%s: No mappings exist for TPU mailboxes\n",
			__func__);
		ret = -EINVAL;
		goto out;
	}

	gxp_dma_unmap_tpu_buffer(gxp, client->mbx_desc);

	gxp_tpu_info.tpu_fd = ibuf.tpu_fd;
	ret = edgetpu_ext_driver_cmd(gxp->tpu_dev.dev,
				     EDGETPU_EXTERNAL_CLIENT_TYPE_DSP,
				     FREE_EXTERNAL_MAILBOX, &gxp_tpu_info,
				     NULL);
	if (ret) {
		dev_err(gxp->dev, "%s: Failed to free ext tpu mailboxes %d\n",
			__func__, ret);
		goto out;
	}
	client->tpu_mbx_allocated = false;

out:
	mutex_unlock(&gxp->vd_lock);

	return ret;
#else
	return -ENODEV;
#endif
}

static long gxp_ioctl(struct file *file, uint cmd, ulong arg)
{
	struct gxp_client *client = file->private_data;
	void __user *argp = (void __user *)arg;
	long ret;

	switch (cmd) {
	case GXP_MAP_BUFFER:
		ret = gxp_map_buffer(client, argp);
		break;
	case GXP_UNMAP_BUFFER:
		ret = gxp_unmap_buffer(client, argp);
		break;
	case GXP_SYNC_BUFFER:
		ret = gxp_sync_buffer(client, argp);
		break;
	case GXP_MAILBOX_COMMAND:
		ret = gxp_mailbox_command(client, argp);
		break;
	case GXP_MAILBOX_RESPONSE:
		ret = gxp_mailbox_response(client, argp);
		break;
	case GXP_GET_SPECS:
		ret = gxp_get_specs(client, argp);
		break;
	case GXP_ALLOCATE_VIRTUAL_DEVICE:
		ret = gxp_allocate_vd(client, argp);
		break;
	case GXP_ETM_TRACE_START_COMMAND:
		ret = gxp_etm_trace_start_command(client, argp);
		break;
	case GXP_ETM_TRACE_SW_STOP_COMMAND:
		ret = gxp_etm_trace_sw_stop_command(client, argp);
		break;
	case GXP_ETM_TRACE_CLEANUP_COMMAND:
		ret = gxp_etm_trace_cleanup_command(client, argp);
		break;
	case GXP_ETM_GET_TRACE_INFO_COMMAND:
		ret = gxp_etm_get_trace_info_command(client, argp);
		break;
	case GXP_ENABLE_TELEMETRY:
		ret = gxp_enable_telemetry(client, argp);
		break;
	case GXP_DISABLE_TELEMETRY:
		ret = gxp_disable_telemetry(client, argp);
		break;
	case GXP_MAP_TPU_MBX_QUEUE:
		ret = gxp_map_tpu_mbx_queue(client, argp);
		break;
	case GXP_UNMAP_TPU_MBX_QUEUE:
		ret = gxp_unmap_tpu_mbx_queue(client, argp);
		break;
	default:
		ret = -ENOTTY; /* unknown command */
	}

	return ret;
}

static int gxp_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct gxp_client *client = file->private_data;

	if (!client)
		return -ENODEV;

	switch (vma->vm_pgoff << PAGE_SHIFT) {
	case GXP_MMAP_LOG_BUFFER_OFFSET:
		return gxp_telemetry_mmap_buffers(client->gxp,
						  GXP_TELEMETRY_TYPE_LOGGING,
						  vma);
	case GXP_MMAP_TRACE_BUFFER_OFFSET:
		return gxp_telemetry_mmap_buffers(client->gxp,
						  GXP_TELEMETRY_TYPE_TRACING,
						  vma);
	default:
		return -EINVAL;
	}
}

static const struct file_operations gxp_fops = {
	.owner = THIS_MODULE,
	.llseek = no_llseek,
	.mmap = gxp_mmap,
	.open = gxp_open,
	.release = gxp_release,
	.unlocked_ioctl = gxp_ioctl,
};

static int gxp_platform_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gxp_dev *gxp;
	struct resource *r;
	phys_addr_t offset, base_addr;
	struct device_node *np;
	struct platform_device *tpu_pdev;
	int ret;
	int i __maybe_unused;
	bool tpu_found __maybe_unused;

	gxp = devm_kzalloc(dev, sizeof(*gxp), GFP_KERNEL);
	if (!gxp)
		return -ENOMEM;

	platform_set_drvdata(pdev, gxp);
	gxp->dev = dev;

	gxp->misc_dev.minor = MISC_DYNAMIC_MINOR;
	gxp->misc_dev.name = "gxp";
	gxp->misc_dev.fops = &gxp_fops;

	ret = misc_register(&gxp->misc_dev);
	if (ret) {
		dev_err(dev, "Failed to register misc device (ret = %d)\n",
			ret);
		devm_kfree(dev, (void *)gxp);
		return ret;
	}

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (IS_ERR_OR_NULL(r)) {
		dev_err(dev, "Failed to get memory resource\n");
		ret = -ENODEV;
		goto err;
	}

	gxp->regs.paddr = r->start;
	gxp->regs.size = resource_size(r);
	gxp->regs.vaddr = devm_ioremap_resource(dev, r);
	if (IS_ERR_OR_NULL(gxp->regs.vaddr)) {
		dev_err(dev, "Failed to map registers\n");
		ret = -ENODEV;
		goto err;
	}

#if defined(CONFIG_GXP_CLOUDRIPPER) && !defined(CONFIG_GXP_TEST)
	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret) {
		dev_err(dev, "pm_runtime_get_sync returned %d\n", ret);
		goto err;
	}
#endif

#ifndef CONFIG_GXP_USE_SW_MAILBOX
	for (i = 0; i < GXP_NUM_CORES; i++) {
		r = platform_get_resource(pdev, IORESOURCE_MEM, i + 1);
		if (IS_ERR_OR_NULL(r)) {
			dev_err(dev, "Failed to get mailbox%d resource\n", i);
			ret = -ENODEV;
			goto err;
		}

		gxp->mbx[i].paddr = r->start;
		gxp->mbx[i].size = resource_size(r);
		gxp->mbx[i].vaddr = devm_ioremap_resource(dev, r);
		if (IS_ERR_OR_NULL(gxp->mbx[i].vaddr)) {
			dev_err(dev, "Failed to map mailbox%d registers\n", i);
			ret = -ENODEV;
			goto err;
		}
	}

	tpu_found = true;
	/* Get TPU device from device tree */
	np = of_parse_phandle(dev->of_node, "tpu-device", 0);
	if (IS_ERR_OR_NULL(np)) {
		dev_warn(dev, "No tpu-device in device tree\n");
		tpu_found = false;
	}
	tpu_pdev = of_find_device_by_node(np);
	if (!tpu_pdev) {
		dev_err(dev, "TPU device not found\n");
		tpu_found = false;
	}
	/* get tpu mailbox register base */
	ret = of_property_read_u64_index(np, "reg", 0, &base_addr);
	of_node_put(np);
	if (ret) {
		dev_warn(dev, "Unable to get tpu-device base address\n");
		tpu_found = false;
	}
	/* get gxp-tpu mailbox register offset */
	ret = of_property_read_u64(dev->of_node, "gxp-tpu-mbx-offset",
				   &offset);
	if (ret) {
		dev_warn(dev, "Unable to get tpu-device mailbox offset\n");
		tpu_found = false;
	}
	if (tpu_found) {
		gxp->tpu_dev.dev = &tpu_pdev->dev;
		get_device(gxp->tpu_dev.dev);
		gxp->tpu_dev.mbx_paddr = base_addr + offset;
	} else {
		dev_warn(dev, "TPU will not be available for interop\n");
		gxp->tpu_dev.mbx_paddr = 0;
	}
#endif  // !CONFIG_GXP_USE_SW_MAILBOX

	ret = gxp_dma_init(gxp);
	if (ret) {
		dev_err(dev, "Failed to initialize GXP DMA interface\n");
		goto err_put_tpu_dev;
	}

	gxp->mailbox_mgr = gxp_mailbox_create_manager(gxp, GXP_NUM_CORES);
	if (IS_ERR_OR_NULL(gxp->mailbox_mgr)) {
		dev_err(dev, "Failed to create mailbox manager\n");
		ret = -ENOMEM;
		goto err_dma_exit;
	}
	spin_lock_init(&gxp->mailbox_resps_lock);

#ifdef CONFIG_ANDROID
	ret = gxp_debug_dump_init(gxp, &gxp_sscd_dev, &gxp_sscd_pdata);
#else
	ret = gxp_debug_dump_init(gxp, NULL, NULL);
#endif  // !CONFIG_ANDROID
	if (ret) {
		dev_err(dev, "Failed to initialize debug dump\n");
		gxp_debug_dump_exit(gxp);
	}

	ret = gxp_mapping_init(gxp);
	if (ret) {
		dev_err(dev, "Failed to initialize mapping (ret=%d)\n", ret);
		goto err_debug_dump_exit;
	}

	ret = gxp_vd_init(gxp);
	if (ret) {
		dev_err(dev,
			"Failed to initialize virtual device manager (ret=%d)\n",
			ret);
		goto err_debug_dump_exit;
	}

	ret = gxp_dma_map_resources(gxp);
	if (ret) {
		dev_err(dev, "Failed to map resources for GXP cores (ret=%d)\n",
			ret);
		goto err_vd_destroy;
	}

	gxp_fw_data_init(gxp);
	gxp_telemetry_init(gxp);
	gxp_create_debugfs(gxp);
	gxp_pm_init(gxp);
	dev_dbg(dev, "Probe finished\n");

	return 0;

err_vd_destroy:
	gxp_vd_destroy(gxp);
err_debug_dump_exit:
	gxp_debug_dump_exit(gxp);
err_dma_exit:
	gxp_dma_exit(gxp);
err_put_tpu_dev:
#ifndef CONFIG_GXP_USE_SW_MAILBOX
	put_device(gxp->tpu_dev.dev);
#endif
err:
	misc_deregister(&gxp->misc_dev);
	devm_kfree(dev, (void *)gxp);
	return ret;
}

static int gxp_platform_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct gxp_dev *gxp = platform_get_drvdata(pdev);

	gxp_debug_dump_exit(gxp);
	gxp_remove_debugfs(gxp);
	gxp_fw_data_destroy(gxp);
	gxp_vd_destroy(gxp);
	gxp_dma_unmap_resources(gxp);
	gxp_dma_exit(gxp);
#ifndef CONFIG_GXP_USE_SW_MAILBOX
	put_device(gxp->tpu_dev.dev);
#endif
	misc_deregister(&gxp->misc_dev);

#ifdef CONFIG_GXP_CLOUDRIPPER
	// Request to power off BLK_AUR
	gxp_pm_blk_off(gxp);
	pm_runtime_disable(dev);
	gxp_pm_destroy(gxp);
#endif

	devm_kfree(dev, (void *)gxp);

	return 0;
}

#ifdef CONFIG_OF
static const struct of_device_id gxp_of_match[] = {
	{ .compatible = "google,gxp", },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(of, gxp_of_match);
#endif

#ifdef CONFIG_ACPI
static const struct acpi_device_id gxp_acpi_match[] = {
	{ "CXRP0001", 0 },
	{ /* end of list */ },
};
MODULE_DEVICE_TABLE(acpi, gxp_acpi_match);
#endif

static struct platform_driver gxp_platform_driver = {
	.probe = gxp_platform_probe,
	.remove = gxp_platform_remove,
	.driver = {
			.name = GXP_DRIVER_NAME,
			.of_match_table = of_match_ptr(gxp_of_match),
			.acpi_match_table = ACPI_PTR(gxp_acpi_match),
		},
};

static int __init gxp_platform_init(void)
{
#ifdef CONFIG_ANDROID
	/* Registers SSCD platform device */
	if (platform_device_register(&gxp_sscd_dev))
		pr_err("Unable to register SSCD platform device\n");
#endif
	return platform_driver_register(&gxp_platform_driver);
}

static void __exit gxp_platform_exit(void)
{
	platform_driver_unregister(&gxp_platform_driver);
#ifdef CONFIG_ANDROID
	platform_device_unregister(&gxp_sscd_dev);
#endif
}

MODULE_DESCRIPTION("Google GXP platform driver");
MODULE_LICENSE("GPL v2");
module_init(gxp_platform_init);
module_exit(gxp_platform_exit);
