// SPDX-License-Identifier: GPL-2.0
/*
 * GXP debugfs support.
 *
 * Copyright (C) 2021 Google LLC
 */

#include "gxp.h"
#include "gxp-debug-dump.h"
#include "gxp-debugfs.h"
#include "gxp-firmware.h"
#include "gxp-firmware-data.h"
#include "gxp-internal.h"
#include "gxp-pm.h"
#include "gxp-mailbox.h"
#include "gxp-telemetry.h"
#include "gxp-vd.h"
#include "gxp-wakelock.h"

static int gxp_debugfs_lpm_test(void *data, u64 val)
{
	struct gxp_dev *gxp = (struct gxp_dev *) data;

	dev_info(gxp->dev, "%llu\n", val);

	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(gxp_lpm_test_fops, NULL, gxp_debugfs_lpm_test,
			 "%llu\n");

static int gxp_debugfs_mailbox(void *data, u64 val)
{
	int core;
	struct gxp_command cmd;
	struct gxp_response resp;
	struct gxp_dev *gxp = (struct gxp_dev *)data;

	core = val / 1000;
	if (core >= GXP_NUM_CORES) {
		dev_notice(gxp->dev,
			   "Mailbox for core %d doesn't exist.\n", core);
		return -EINVAL;
	}

	if (gxp->mailbox_mgr == NULL ||
	    gxp->mailbox_mgr->mailboxes[core] == NULL) {
		dev_notice(gxp->dev,
			   "Unable to send mailbox command -- mailbox %d not ready\n",
			   core);
		return -EINVAL;
	}

	cmd.code = (u16) val;
	cmd.priority = 0;
	cmd.buffer_descriptor.address = 0;
	cmd.buffer_descriptor.size = 0;
	cmd.buffer_descriptor.flags = 0;

	down_read(&gxp->vd_semaphore);
	gxp_mailbox_execute_cmd(gxp->mailbox_mgr->mailboxes[core], &cmd, &resp);
	up_read(&gxp->vd_semaphore);

	dev_info(gxp->dev,
		"Mailbox Command Sent: cmd.code=%d, resp.status=%d, resp.retval=%d\n",
		cmd.code, resp.status, resp.retval);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(gxp_mailbox_fops, NULL, gxp_debugfs_mailbox, "%llu\n");

static int gxp_debugfs_pingpong(void *data, u64 val)
{
	int core;
	struct gxp_command cmd;
	struct gxp_response resp;
	struct gxp_dev *gxp = (struct gxp_dev *)data;

	core = val / 1000;
	if (core >= GXP_NUM_CORES) {
		dev_notice(gxp->dev,
			   "Mailbox for core %d doesn't exist.\n", core);
		return -EINVAL;
	}

	if (gxp->mailbox_mgr == NULL ||
	    gxp->mailbox_mgr->mailboxes[core] == NULL) {
		dev_notice(
			gxp->dev,
			"Unable to send mailbox pingpong -- mailbox %d not ready\n",
			core);
		return -EINVAL;
	}

	cmd.code = GXP_MBOX_CODE_PINGPONG;
	cmd.priority = 0;
	cmd.buffer_descriptor.address = 0;
	cmd.buffer_descriptor.size = 0;
	cmd.buffer_descriptor.flags = (u32) val;

	down_read(&gxp->vd_semaphore);
	gxp_mailbox_execute_cmd(gxp->mailbox_mgr->mailboxes[core], &cmd, &resp);
	up_read(&gxp->vd_semaphore);

	dev_info(
		gxp->dev,
		"Mailbox Pingpong Sent to core %d: val=%d, resp.status=%d, resp.retval=%d\n",
		core, cmd.buffer_descriptor.flags, resp.status, resp.retval);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(gxp_pingpong_fops, NULL, gxp_debugfs_pingpong,
			 "%llu\n");

static int gxp_firmware_run_set(void *data, u64 val)
{
	struct gxp_dev *gxp = (struct gxp_dev *) data;
	struct gxp_client *client_to_delete;
	int ret = 0;

	down_write(&gxp->vd_semaphore);

	if (val) {
		if (gxp->debugfs_client) {
			dev_err(gxp->dev, "Firmware already running!\n");
			ret = -EIO;
			goto out;
		}

		/* Cannot run firmware without a wakelock */
		gxp_wakelock_acquire(gxp);

		/*
		 * Cleanup any bad state or corruption the device might've
		 * caused
		 */
		gxp_fw_data_destroy(gxp);
		gxp_fw_data_init(gxp);

		gxp->debugfs_client = gxp_client_create(gxp);
		if (IS_ERR(gxp->debugfs_client)) {
			dev_err(gxp->dev, "Failed to create client\n");
			ret = PTR_ERR(gxp->debugfs_client);
			gxp->debugfs_client = NULL;
			gxp_wakelock_release(gxp);
			goto out;
		}

		ret = gxp_vd_allocate(gxp->debugfs_client, GXP_NUM_CORES);
		if (ret) {
			dev_err(gxp->dev, "Failed to allocate VD\n");
			gxp_client_destroy(gxp->debugfs_client);
			gxp->debugfs_client = NULL;
			gxp_wakelock_release(gxp);
			goto out;
		}
	} else {
		if (!gxp->debugfs_client) {
			dev_err(gxp->dev, "Firmware not running!\n");
			ret = -EIO;
			goto out;
		}
		client_to_delete = gxp->debugfs_client;
		gxp->debugfs_client = NULL;

		up_write(&gxp->vd_semaphore);

		gxp_client_destroy(client_to_delete);
		gxp_wakelock_release(gxp);

		/* Return here, since vd_semaphore has already been unlocked */
		return ret;
	}

out:
	up_write(&gxp->vd_semaphore);

	return ret;
}

static int gxp_firmware_run_get(void *data, u64 *val)
{
	struct gxp_dev *gxp = (struct gxp_dev *) data;

	*val = gxp->firmware_running;
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(gxp_firmware_run_fops, gxp_firmware_run_get,
			 gxp_firmware_run_set, "%llx\n");

static int gxp_wakelock_set(void *data, u64 val)
{
	struct gxp_dev *gxp = (struct gxp_dev *)data;
	int ret = 0;

	if (val > 0) {
		/* Wakelock Acquire */
		if (gxp->debugfs_wakelock_held) {
			dev_warn(gxp->dev,
				 "Debugfs wakelock is already held.\n");
			return -EBUSY;
		}

		ret = gxp_wakelock_acquire(gxp);
		if (ret)
			dev_err(gxp->dev,
				"Failed to acquire debugfs wakelock ret=%d\n",
				ret);
		else
			gxp->debugfs_wakelock_held = true;
	} else {
		/* Wakelock Release */
		if (!gxp->debugfs_wakelock_held) {
			dev_warn(gxp->dev, "Debugfs wakelock not held.\n");
			return -EIO;
		}

		gxp_wakelock_release(gxp);
		gxp->debugfs_wakelock_held = false;
	}

	return ret;
}

DEFINE_DEBUGFS_ATTRIBUTE(gxp_wakelock_fops, NULL, gxp_wakelock_set, "%llx\n");

static int gxp_blk_powerstate_set(void *data, u64 val)
{
	struct gxp_dev *gxp = (struct gxp_dev *)data;
	int ret = 0;

	if (gxp_pm_get_blk_state(gxp) == AUR_OFF) {
		dev_warn(
			gxp->dev,
			"Cannot set block power state when the block is off. Obtain a wakelock to power it on.\n");
		return -ENODEV;
	}

	if (val >= AUR_DVFS_MIN_STATE) {
		ret = gxp_pm_blk_set_state_acpm(gxp, val);
	} else {
		ret = -EINVAL;
		dev_err(gxp->dev, "Incorrect state %llu\n", val);
	}
	return ret;
}

static int gxp_blk_powerstate_get(void *data, u64 *val)
{
	struct gxp_dev *gxp = (struct gxp_dev *)data;

	if (gxp_pm_get_blk_state(gxp) == AUR_OFF) {
		dev_warn(
			gxp->dev,
			"Cannot get block power state when the block is off.\n");
		return -ENODEV;
	}

	*val = gxp_pm_blk_get_state_acpm(gxp);
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(gxp_blk_powerstate_fops, gxp_blk_powerstate_get,
			 gxp_blk_powerstate_set, "%llx\n");

static int gxp_debugfs_coredump(void *data, u64 val)
{
	return gxp_debugfs_mailbox(data, GXP_MBOX_CODE_COREDUMP);
}
DEFINE_DEBUGFS_ATTRIBUTE(gxp_coredump_fops, NULL, gxp_debugfs_coredump,
			 "%llu\n");

static int gxp_log_buff_set(void *data, u64 val)
{
	struct gxp_dev *gxp = (struct gxp_dev *)data;
	int i;
	u64 **buffers;
	u64 *ptr;

	mutex_lock(&gxp->telemetry_mgr->lock);

	if (!gxp->telemetry_mgr->logging_buff_data) {
		dev_err(gxp->dev, "%s: Logging buffer has not been created\n",
			__func__);
		mutex_unlock(&gxp->telemetry_mgr->lock);
		return -ENODEV;
	}

	buffers = (u64 **)gxp->telemetry_mgr->logging_buff_data->buffers;
	for (i = 0; i < GXP_NUM_CORES; i++) {
		ptr = buffers[i];
		*ptr = val;
	}
	dev_dbg(gxp->dev,
		"%s: log buff first bytes: [0] = %llu, [1] = %llu, [2] = %llu, [3] = %llu (val=%llu)\n",
		__func__, *buffers[0], *buffers[1], *buffers[2], *buffers[3],
		val);

	mutex_unlock(&gxp->telemetry_mgr->lock);

	return 0;
}

static int gxp_log_buff_get(void *data, u64 *val)
{
	struct gxp_dev *gxp = (struct gxp_dev *)data;
	u64 **buffers;

	mutex_lock(&gxp->telemetry_mgr->lock);

	if (!gxp->telemetry_mgr->logging_buff_data) {
		dev_err(gxp->dev, "%s: Logging buffer has not been created\n",
			__func__);
		mutex_unlock(&gxp->telemetry_mgr->lock);
		return -ENODEV;
	}

	buffers = (u64 **)gxp->telemetry_mgr->logging_buff_data->buffers;
	dev_dbg(gxp->dev,
		"%s: log buff first bytes: [0] = %llu, [1] = %llu, [2] = %llu, [3] = %llu\n",
		__func__, *buffers[0], *buffers[1], *buffers[2], *buffers[3]);

	*val = *buffers[0];

	mutex_unlock(&gxp->telemetry_mgr->lock);

	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(gxp_log_buff_fops, gxp_log_buff_get, gxp_log_buff_set,
			 "%llu\n");

static int gxp_log_eventfd_signal_set(void *data, u64 val)
{
	struct gxp_dev *gxp = (struct gxp_dev *)data;
	int ret = 0;

	mutex_lock(&gxp->telemetry_mgr->lock);

	if (!gxp->telemetry_mgr->logging_efd) {
		ret = -ENODEV;
		goto out;
	}

	ret = eventfd_signal(gxp->telemetry_mgr->logging_efd, 1);

out:
	mutex_unlock(&gxp->telemetry_mgr->lock);

	return ret;
}

DEFINE_DEBUGFS_ATTRIBUTE(gxp_log_eventfd_signal_fops, NULL,
			 gxp_log_eventfd_signal_set, "%llu\n");

void gxp_create_debugfs(struct gxp_dev *gxp)
{
	gxp->d_entry = debugfs_create_dir("gxp", NULL);
	if (IS_ERR_OR_NULL(gxp->d_entry))
		return;

	gxp->debugfs_wakelock_held = false;

	debugfs_create_file("lpm_test", 0200, gxp->d_entry, gxp,
			    &gxp_lpm_test_fops);
	debugfs_create_file("mailbox", 0200, gxp->d_entry, gxp,
			    &gxp_mailbox_fops);
	debugfs_create_file("pingpong", 0200, gxp->d_entry, gxp,
			    &gxp_pingpong_fops);
	debugfs_create_file("firmware_run", 0600, gxp->d_entry, gxp,
			    &gxp_firmware_run_fops);
	debugfs_create_file("wakelock", 0200, gxp->d_entry, gxp,
			    &gxp_wakelock_fops);
	debugfs_create_file("blk_powerstate", 0600, gxp->d_entry, gxp,
			    &gxp_blk_powerstate_fops);
	debugfs_create_file("coredump", 0200, gxp->d_entry, gxp,
			    &gxp_coredump_fops);
	debugfs_create_file("log", 0600, gxp->d_entry, gxp, &gxp_log_buff_fops);
	debugfs_create_file("log_eventfd", 0200, gxp->d_entry, gxp,
			    &gxp_log_eventfd_signal_fops);
}

void gxp_remove_debugfs(struct gxp_dev *gxp)
{
	if (gxp->debugfs_client)
		gxp_client_destroy(gxp->debugfs_client);

	debugfs_remove_recursive(gxp->d_entry);
}
