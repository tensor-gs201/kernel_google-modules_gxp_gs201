// SPDX-License-Identifier: GPL-2.0
/*
 * GXP debug dump handler
 *
 * Copyright (C) 2020 Google LLC
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/workqueue.h>

#if IS_ENABLED(CONFIG_SUBSYSTEM_COREDUMP)
#include <linux/platform_data/sscoredump.h>
#endif

#include "gxp-debug-dump.h"
#include "gxp-doorbell.h"
#include "gxp-internal.h"
#include "gxp-lpm.h"
#include "gxp-tmp.h"

#define GXP_COREDUMP_PENDING 0xF
#define KERNEL_INIT_DUMP_TIMEOUT (10000 * GXP_TIME_DELAY_FACTOR)

/* Enum indicating the debug dump request reason. */
enum gxp_debug_dump_init_type {
	DEBUG_DUMP_FW_INIT,
	DEBUG_DUMP_KERNEL_INIT
};

enum gxp_common_segments_idx {
	GXP_COMMON_REGISTERS_IDX,
	GXP_LPM_REGISTERS_IDX
};

static void gxp_debug_dump_cache_invalidate(struct gxp_dev *gxp)
{
	/* Debug dump carveout is currently coherent. NO-OP. */
	return;
}

static void gxp_debug_dump_cache_flush(struct gxp_dev *gxp)
{
	/* Debug dump carveout is currently coherent. NO-OP. */
	return;
}


static void
gxp_get_common_registers(struct gxp_dev *gxp, struct gxp_seg_header *seg_header,
			 struct gxp_common_registers *common_regs)
{
	int i;
	u32 addr;

	dev_dbg(gxp->dev, "Getting common registers\n");

	strscpy(seg_header->name, "Common Registers", sizeof(seg_header->name));
	seg_header->valid = 1;
	seg_header->size = sizeof(*common_regs);

	/* Get Aurora Top registers */
	common_regs->aurora_revision =
		gxp_read_32(gxp, GXP_REG_AURORA_REVISION);
	common_regs->common_int_pol_0 =
		gxp_read_32(gxp, GXP_REG_COMMON_INT_POL_0);
	common_regs->common_int_pol_1 =
		gxp_read_32(gxp, GXP_REG_COMMON_INT_POL_1);
	common_regs->dedicated_int_pol =
		gxp_read_32(gxp, GXP_REG_DEDICATED_INT_POL);
	common_regs->raw_ext_int = gxp_read_32(gxp, GXP_REG_RAW_EXT_INT);

	for (i = 0; i < CORE_PD_COUNT; i++) {
		common_regs->core_pd[i] =
			gxp_read_32(gxp, GXP_REG_CORE_PD + CORE_PD_BASE(i));
	}

	common_regs->global_counter_low =
		gxp_read_32(gxp, GXP_REG_GLOBAL_COUNTER_LOW);
	common_regs->global_counter_high =
		gxp_read_32(gxp, GXP_REG_GLOBAL_COUNTER_HIGH);
	common_regs->wdog_control = gxp_read_32(gxp, GXP_REG_WDOG_CONTROL);
	common_regs->wdog_value = gxp_read_32(gxp, GXP_REG_WDOG_VALUE);

	for (i = 0; i < TIMER_COUNT; i++) {
		addr = GXP_REG_TIMER_COMPARATOR + TIMER_BASE(i);
		common_regs->timer[i].comparator =
			gxp_read_32(gxp, addr + TIMER_COMPARATOR_OFFSET);
		common_regs->timer[i].control =
			gxp_read_32(gxp, addr + TIMER_CONTROL_OFFSET);
		common_regs->timer[i].value =
			gxp_read_32(gxp, addr + TIMER_VALUE_OFFSET);
	}

	/* Get Doorbell registers */
	for (i = 0; i < DOORBELL_COUNT; i++)
		common_regs->doorbell[i] = gxp_doorbell_status(gxp, i);

	/* Get Sync Barrier registers */
	for (i = 0; i < SYNC_BARRIER_COUNT; i++)
		common_regs->sync_barrier[i] =
			gxp_read_sync_barrier_shadow(gxp, i);

	dev_dbg(gxp->dev, "Done getting common registers\n");
}

static void gxp_get_lpm_psm_registers(struct gxp_dev *gxp,
				      struct gxp_lpm_psm_registers *psm_regs,
				      int psm)
{
	struct gxp_lpm_state_table_registers *state_table_regs;
	int i, j;
	uint offset;

	/* Get State Table registers */
	for (i = 0; i < PSM_STATE_TABLE_COUNT; i++) {
		state_table_regs = &psm_regs->state_table[i];

		/* Get Trans registers */
		for (j = 0; j < PSM_TRANS_COUNT; j++) {
			offset = PSM_STATE_TABLE_BASE(i) + PSM_TRANS_BASE(j);
			state_table_regs->trans[j].next_state =
				lpm_read_32_psm(gxp, psm, offset +
						PSM_NEXT_STATE_OFFSET);
			state_table_regs->trans[j].seq_addr =
				lpm_read_32_psm(gxp, psm, offset +
						PSM_SEQ_ADDR_OFFSET);
			state_table_regs->trans[j].timer_val =
				lpm_read_32_psm(gxp, psm, offset +
						PSM_TIMER_VAL_OFFSET);
			state_table_regs->trans[j].timer_en =
				lpm_read_32_psm(gxp, psm, offset +
						PSM_TIMER_EN_OFFSET);
			state_table_regs->trans[j].trigger_num =
				lpm_read_32_psm(gxp, psm, offset +
						PSM_TRIGGER_NUM_OFFSET);
			state_table_regs->trans[j].trigger_en =
				lpm_read_32_psm(gxp, psm, offset +
						PSM_TRIGGER_EN_OFFSET);
		}

		state_table_regs->enable_state =
			lpm_read_32_psm(gxp, psm, PSM_STATE_TABLE_BASE(i) +
					PSM_ENABLE_STATE_OFFSET);
	}

	/* Get DMEM registers */
	for (i = 0; i < PSM_DATA_COUNT; i++) {
		offset = PSM_DMEM_BASE(i) + PSM_DATA_OFFSET;
		psm_regs->data[i] = lpm_read_32_psm(gxp, psm, offset);
	}

	psm_regs->cfg = lpm_read_32_psm(gxp, psm, PSM_CFG_OFFSET);
	psm_regs->status = lpm_read_32_psm(gxp, psm, PSM_STATUS_OFFSET);

	/* Get Debug CSR registers */
	psm_regs->debug_cfg = lpm_read_32_psm(gxp, psm, PSM_DEBUG_CFG_OFFSET);
	psm_regs->break_addr = lpm_read_32_psm(gxp, psm, PSM_BREAK_ADDR_OFFSET);
	psm_regs->gpin_lo_rd = lpm_read_32_psm(gxp, psm, PSM_GPIN_LO_RD_OFFSET);
	psm_regs->gpin_hi_rd = lpm_read_32_psm(gxp, psm, PSM_GPIN_HI_RD_OFFSET);
	psm_regs->gpout_lo_rd =
		lpm_read_32_psm(gxp, psm, PSM_GPOUT_LO_RD_OFFSET);
	psm_regs->gpout_hi_rd =
		lpm_read_32_psm(gxp, psm, PSM_GPOUT_HI_RD_OFFSET);
	psm_regs->debug_status =
		lpm_read_32_psm(gxp, psm, PSM_DEBUG_STATUS_OFFSET);
}

static void
gxp_get_lpm_registers(struct gxp_dev *gxp, struct gxp_seg_header *seg_header,
		      struct gxp_lpm_registers *lpm_regs)
{
	int i;
	uint offset;

	dev_dbg(gxp->dev, "Getting LPM registers\n");

	strscpy(seg_header->name, "LPM Registers", sizeof(seg_header->name));
	seg_header->valid = 1;
	seg_header->size = sizeof(*lpm_regs);

	/* Get LPM Descriptor registers */
	lpm_regs->lpm_version = lpm_read_32(gxp, LPM_VERSION_OFFSET);
	lpm_regs->trigger_csr_start =
		lpm_read_32(gxp, TRIGGER_CSR_START_OFFSET);
	lpm_regs->imem_start = lpm_read_32(gxp, IMEM_START_OFFSET);
	lpm_regs->lpm_config = lpm_read_32(gxp, LPM_CONFIG_OFFSET);

	for (i = 0; i < PSM_DESCRIPTOR_COUNT; i++) {
		offset = PSM_DESCRIPTOR_OFFSET + PSM_DESCRIPTOR_BASE(i);
		lpm_regs->psm_descriptor[i] = lpm_read_32(gxp, offset);
	}

	/* Get Trigger CSR registers */
	for (i = 0; i < EVENTS_EN_COUNT; i++) {
		offset = EVENTS_EN_OFFSET + EVENTS_EN_BASE(i);
		lpm_regs->events_en[i] = lpm_read_32(gxp, offset);
	}

	for (i = 0; i < EVENTS_INV_COUNT; i++) {
		offset = EVENTS_INV_OFFSET + EVENTS_INV_BASE(i);
		lpm_regs->events_inv[i] = lpm_read_32(gxp, offset);
	}

	lpm_regs->function_select = lpm_read_32(gxp, FUNCTION_SELECT_OFFSET);
	lpm_regs->trigger_status = lpm_read_32(gxp, TRIGGER_STATUS_OFFSET);
	lpm_regs->event_status = lpm_read_32(gxp, EVENT_STATUS_OFFSET);

	/* Get IMEM registers */
	for (i = 0; i < OPS_COUNT; i++) {
		offset = OPS_OFFSET + OPS_BASE(i);
		lpm_regs->ops[i] = lpm_read_32(gxp, offset);
	}

	/* Get PSM registers */
	for (i = 0; i < PSM_COUNT; i++)
		gxp_get_lpm_psm_registers(gxp, &lpm_regs->psm_regs[i], i);

	dev_dbg(gxp->dev, "Done getting LPM registers\n");
}

static void gxp_get_common_dump(struct gxp_dev *gxp,
				struct gxp_seg_header *common_seg_header,
				struct gxp_common_dump_data *common_dump_data)
{
	gxp_get_common_registers(gxp,
				 &common_seg_header[GXP_COMMON_REGISTERS_IDX],
				 &common_dump_data->common_regs);
	gxp_get_lpm_registers(gxp, &common_seg_header[GXP_LPM_REGISTERS_IDX],
			      &common_dump_data->lpm_regs);

	dev_dbg(gxp->dev, "Segment Header for Common Segment\n");
	dev_dbg(gxp->dev, "Name: %s, Size: 0x%0x bytes, Valid :%0x\n",
		common_seg_header->name, common_seg_header->size,
		common_seg_header->valid);
	dev_dbg(gxp->dev, "Register aurora_revision: 0x%0x\n",
		common_dump_data->common_regs.aurora_revision);
}

static void gxp_handle_debug_dump(struct gxp_dev *gxp,
				  struct gxp_core_dump *core_dump,
				  struct gxp_common_dump *common_dump,
				  enum gxp_debug_dump_init_type init_type,
				  uint core_bits)
{
	struct gxp_core_dump_header *core_dump_header;
	struct gxp_core_header *core_header;
	int i;
#if IS_ENABLED(CONFIG_SUBSYSTEM_COREDUMP)
	struct sscd_platform_data *pdata =
		(struct sscd_platform_data *)gxp->debug_dump_mgr->sscd_pdata;
	struct sscd_segment *segs;
	int segs_num = GXP_NUM_COMMON_SEGMENTS;
	int seg_idx = 0;
	int core_dump_num = 0;
	int j;
	void *data_addr;

	for (i = 0; i < GXP_NUM_CORES; i++) {
		if (core_bits & BIT(i))
			core_dump_num++;
	}

	/*
	 * segs_num include the common segments, core segments for each core,
	 * core header for each core
	 */
	if (init_type == DEBUG_DUMP_FW_INIT)
		segs_num += GXP_NUM_CORE_SEGMENTS + 1;
	else
		segs_num += GXP_NUM_CORE_SEGMENTS * core_dump_num +
			    core_dump_num;

	segs = kmalloc_array(segs_num, sizeof(struct sscd_segment),
			     GFP_KERNEL);
	if (!segs)
		goto out;

	/* Common */
	data_addr = &common_dump->common_dump_data.common_regs;
	for (i = 0; i < GXP_NUM_COMMON_SEGMENTS; i++) {
		segs[seg_idx].addr = data_addr;
		segs[seg_idx].size = common_dump->seg_header[i].size;
		data_addr += segs[seg_idx].size;
		seg_idx++;
	}
#endif  // CONFIG_SUBSYSTEM_COREDUMP

	/* Core */
	for (i = 0; i < GXP_NUM_CORES; i++) {
		if ((core_bits & BIT(i)) == 0)
			continue;

		core_dump_header = &core_dump->core_dump_header[i];
		core_header = &core_dump_header->core_header;
		if (!core_header->dump_available) {
			dev_err(gxp->dev,
				"Core dump should have been available\n");
			goto out;
		}

#if IS_ENABLED(CONFIG_SUBSYSTEM_COREDUMP)
		/* Core Header */
		segs[seg_idx].addr = core_header;
		segs[seg_idx].size = sizeof(struct gxp_core_header);
		seg_idx++;

		data_addr = &core_dump->dump_data[i *
						  core_header->core_dump_size /
						  sizeof(u32)];

		for (j = 0; j < GXP_NUM_CORE_SEGMENTS; j++) {
			segs[seg_idx].addr = data_addr;
			segs[seg_idx].size =
				core_dump_header->seg_header[j].size;
			data_addr += segs[seg_idx].size;
			seg_idx++;
		}

		dev_notice(gxp->dev, "Passing dump data to SSCD daemon\n");
		if (!pdata->sscd_report) {
			dev_err(gxp->dev,
				"Failed to generate coredump\n");
			goto out;
		}

		mutex_lock(&gxp->debug_dump_mgr->sscd_lock);
		if (pdata->sscd_report(gxp->debug_dump_mgr->sscd_dev, segs,
				       segs_num, SSCD_FLAGS_ELFARM64HDR,
				       "gxp debug dump")) {
			dev_err(gxp->dev,
				"Unable to send the report to SSCD daemon\n");
			mutex_unlock(&gxp->debug_dump_mgr->sscd_lock);
			goto out;
		}

		/*
		 * This delay is needed to ensure there's sufficient time
		 * in between sscd_report() being called, as the file name of
		 * the core dump files generated by the SSCD daemon includes a
		 * time format with a seconds precision.
		 */
		msleep(1000);
		mutex_unlock(&gxp->debug_dump_mgr->sscd_lock);
#endif  // CONFIG_SUBSYSTEM_COREDUMP

		/* This bit signals that core dump has been processed */
		core_header->dump_available = 0;

		if (init_type == DEBUG_DUMP_FW_INIT)
			goto out;
	}

out:
#if IS_ENABLED(CONFIG_SUBSYSTEM_COREDUMP)
	kfree(segs);
#endif
	return;
}

static int gxp_generate_coredump(struct gxp_dev *gxp,
				 enum gxp_debug_dump_init_type init_type,
				 uint core_bits)
{
	struct gxp_core_dump *core_dump;
	struct gxp_common_dump *common_dump;
	struct gxp_seg_header *common_seg_header;
	struct gxp_common_dump_data *common_dump_data;

	if (!gxp->debug_dump_mgr->core_dump) {
		dev_err(gxp->dev, "Core dump not allocated\n");
		return -EINVAL;
	}

	if (core_bits == 0) {
		dev_err(gxp->dev, "The number of core dumps requested is 0.\n");
		return -EINVAL;
	} else if (core_bits > GXP_COREDUMP_PENDING) {
		dev_err(gxp->dev,
			"The number of core dumps requested (%0x) is greater than expected (%0x)\n",
			core_bits, GXP_COREDUMP_PENDING);
		return -EINVAL;
	}

	gxp_debug_dump_cache_invalidate(gxp);

	core_dump = gxp->debug_dump_mgr->core_dump;
	common_dump = kmalloc(sizeof(*common_dump), GFP_KERNEL);
	if (!common_dump)
		return -ENOMEM;

	common_seg_header = common_dump->seg_header;
	common_dump_data = &common_dump->common_dump_data;

	gxp_get_common_dump(gxp, common_seg_header, common_dump_data);

	gxp_handle_debug_dump(gxp, core_dump, common_dump, init_type,
			      core_bits);

	/* Mark the common segments as read */
	common_seg_header->valid = 0;

	gxp_debug_dump_cache_flush(gxp);

	return 0;
}

static void gxp_wait_kernel_init_dump_work(struct work_struct *work)
{
	struct gxp_debug_dump_manager *mgr =
		container_of(work, struct gxp_debug_dump_manager,
			     wait_kernel_init_dump_work);

	wait_event_timeout(mgr->kernel_init_dump_waitq,
			   mgr->kernel_init_dump_pending ==
			   GXP_COREDUMP_PENDING,
			   msecs_to_jiffies(KERNEL_INIT_DUMP_TIMEOUT));

	mutex_lock(&mgr->lock);
	gxp_generate_coredump(mgr->gxp, DEBUG_DUMP_KERNEL_INIT,
			      mgr->kernel_init_dump_pending);
	mgr->kernel_init_dump_pending = 0;
	mutex_unlock(&mgr->lock);
}

void gxp_debug_dump_process_dump(struct work_struct *work)
{
	struct gxp_debug_dump_work *debug_dump_work =
		container_of(work, struct gxp_debug_dump_work, work);

	uint core_id = debug_dump_work->core_id;
	struct gxp_dev *gxp = debug_dump_work->gxp;
	struct gxp_debug_dump_manager *mgr;
	struct gxp_core_dump *core_dump;
	struct gxp_core_dump_header *core_dump_header;
	struct gxp_core_header *core_header;
	int *kernel_init_dump_pending;

	mgr = gxp->debug_dump_mgr;
	if (!mgr) {
		dev_err(gxp->dev,
			"gxp->debug_dump_mgr has not been initialized\n");
		return;
	}

	core_dump = mgr->core_dump;
	if (!core_dump) {
		dev_err(gxp->dev,
			"mgr->core_dump has not been initialized\n");
		return;
	}

	core_dump_header = &core_dump->core_dump_header[core_id];
	core_header = &core_dump_header->core_header;
	kernel_init_dump_pending = &mgr->kernel_init_dump_pending;

	switch (core_header->dump_req_reason) {
	case DEBUG_DUMP_FW_INIT:
		gxp_generate_coredump(gxp, DEBUG_DUMP_FW_INIT, BIT(core_id));
		break;
	case DEBUG_DUMP_KERNEL_INIT:
		mutex_lock(&mgr->lock);
		if (*kernel_init_dump_pending == 0)
			schedule_work(&mgr->wait_kernel_init_dump_work);
		*kernel_init_dump_pending |= BIT(core_id);
		wake_up(&mgr->kernel_init_dump_waitq);
		mutex_unlock(&mgr->lock);
		break;
	}
}

struct work_struct *gxp_debug_dump_get_notification_handler(struct gxp_dev *gxp,
							    uint core)
{
	struct gxp_debug_dump_manager *mgr = gxp->debug_dump_mgr;

	if (!mgr)
		return NULL;

	return &mgr->debug_dump_works[core].work;
}

int gxp_debug_dump_init(struct gxp_dev *gxp, void *sscd_dev, void *sscd_pdata)
{
	struct resource r;
	struct gxp_debug_dump_manager *mgr;
	struct gxp_core_dump_header *core_dump_header;
	int core;

	mgr = devm_kzalloc(gxp->dev, sizeof(*mgr), GFP_KERNEL);
	if (!mgr)
		return -ENOMEM;
	gxp->debug_dump_mgr = mgr;
	mgr->gxp = gxp;

	/* Find and map the memory reserved for the debug dump */
	if (gxp_acquire_rmem_resource(gxp, &r, "gxp-debug-dump-region")) {
		dev_err(gxp->dev,
			"Unable to acquire debug dump reserved memory\n");
		return -ENODEV;
	}
	gxp->coredumpbuf.paddr = r.start;
	gxp->coredumpbuf.size = resource_size(&r);
	/*
	 * TODO (b/193069216) allocate a dynamic buffer and let
	 * `gxp_dma_map_resources()` map it to the expected paddr
	 */
	/*
	 * TODO (b/200169232) Using memremap until devm_memremap is added to
	 * the GKI ABI
	 */
	gxp->coredumpbuf.vaddr = memremap(gxp->coredumpbuf.paddr,
					  gxp->coredumpbuf.size, MEMREMAP_WC);
	if (IS_ERR(gxp->coredumpbuf.vaddr)) {
		dev_err(gxp->dev, "Failed to map core dump\n");
		return -ENODEV;
	}
	mgr->core_dump = (struct gxp_core_dump *)gxp->coredumpbuf.vaddr;

	for (core = 0; core < GXP_NUM_CORES; core++) {
		core_dump_header = &mgr->core_dump->core_dump_header[core];
		core_dump_header->core_header.dump_available = 0;

		mgr->debug_dump_works[core].gxp = gxp;
		mgr->debug_dump_works[core].core_id = core;
		INIT_WORK(&mgr->debug_dump_works[core].work,
			  gxp_debug_dump_process_dump);
	}

	/* No need for a DMA handle since the carveout is coherent */
	mgr->debug_dump_dma_handle = 0;
	mgr->kernel_init_dump_pending = 0;
	mgr->sscd_dev = sscd_dev;
	mgr->sscd_pdata = sscd_pdata;
	mutex_init(&mgr->lock);
	mutex_init(&mgr->sscd_lock);

	INIT_WORK(&mgr->wait_kernel_init_dump_work,
		  gxp_wait_kernel_init_dump_work);

	init_waitqueue_head(&mgr->kernel_init_dump_waitq);

	return 0;
}

void gxp_debug_dump_exit(struct gxp_dev *gxp)
{
	struct gxp_debug_dump_manager *mgr = gxp->debug_dump_mgr;

	if (!mgr) {
		dev_dbg(gxp->dev, "Debug dump manager was not allocated\n");
		return;
	}

	cancel_work_sync(&mgr->wait_kernel_init_dump_work);
	/* TODO (b/200169232) Remove this once we're using devm_memremap */
	memunmap(gxp->coredumpbuf.vaddr);

	mutex_destroy(&mgr->lock);
	mutex_destroy(&mgr->sscd_lock);
	devm_kfree(mgr->gxp->dev, mgr);
	gxp->debug_dump_mgr = NULL;
}
