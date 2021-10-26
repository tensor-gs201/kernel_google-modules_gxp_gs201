// SPDX-License-Identifier: GPL-2.0
/*
 * GXP kernel-userspace interface definitions.
 *
 * Copyright (C) 2020 Google LLC
 */

#include <linux/kthread.h>

#include "gxp-tmp.h"
#include "gxp-mailbox.h"
#include "gxp-mailbox-driver.h"
#include "gxp-mailbox-regs.h"

/* Doorbells for emulating Mailbox Interrupts to device */
#define MBOX_DOORBELL_INDEX(__core__) (31 - __core__)
#define MBOX_SET_INT_TO_DEVICE(__mailbox__)                                    \
	writel(ENABLE, __mailbox__->gxp->regs.vaddr + DOORBELL_BLOCK +         \
			       DOORBELL_BASE(MBOX_DOORBELL_INDEX(              \
				       __mailbox__->core_id)) +                \
			       DOORBELL_SET_OFFSET)
#define MBOX_CLEAR_INT_TO_DEVICE(__mailbox__)                                  \
	writel(ENABLE, __mailbox__->gxp->regs.vaddr + DOORBELL_BLOCK +         \
			       DOORBELL_BASE(MBOX_DOORBELL_INDEX(              \
				       __mailbox__->core_id)) +                \
			       DOORBELL_CLEAR_OFFSET)

/* Emulated Mailbox Register Macros */
#define MBOX_CSR_SCRATCHPAD_OFFSET 0x100

#define MBOX_ACCESS_SYNC_BARRIER 15 /* Start at the end of the sync barriers */

/* Register Access */

static u32 csr_read(struct gxp_mailbox *mailbox, uint reg_offset)
{
	u32 read_value;

	gxp_acquire_sync_barrier(mailbox->gxp, MBOX_ACCESS_SYNC_BARRIER);

	switch (reg_offset) {
	case MBOX_MCUCTLR_OFFSET:
	case MBOX_INTGR0_OFFSET:
	case MBOX_INTMR0_OFFSET:
	case MBOX_INTSR0_OFFSET:
	case MBOX_INTMSR0_OFFSET:
	case MBOX_INTGR1_OFFSET:
	case MBOX_INTMR1_OFFSET:
	case MBOX_INTSR1_OFFSET:
	case MBOX_INTMSR1_OFFSET:
		break;
	case MBOX_INTCR0_OFFSET:
	case MBOX_INTCR1_OFFSET:
		dev_notice(mailbox->gxp->dev,
			   "Attempt to read write-only mailbox CSR 0x%X\n",
			   reg_offset);
		read_value =  0;
		goto csr_read_exit;
	default:
		dev_notice(mailbox->gxp->dev,
			   "Attempt to read non-existent mailbox CSR 0x%X\n",
			   reg_offset);
		read_value = 0;
		goto csr_read_exit;
	}

	read_value = readl(mailbox->csr_reg_base + reg_offset);

csr_read_exit:
	gxp_release_sync_barrier(mailbox->gxp, MBOX_ACCESS_SYNC_BARRIER);

	return read_value;
}

static void csr_write(struct gxp_mailbox *mailbox, uint reg_offset, u32 value)
{
	bool send_interrupt = false;
	u32 gen_val, clear_val, mask_val, status_val;

	gxp_acquire_sync_barrier(mailbox->gxp, MBOX_ACCESS_SYNC_BARRIER);

	/* Emulate any side effects for CSR writes */
	switch (reg_offset) {
	case MBOX_MCUCTLR_OFFSET:
		/* side effects not implemented */
		break;
	case MBOX_INTGR0_OFFSET:
		/*
		 * 1. Set interrupt status register
		 */
		writel(value, mailbox->csr_reg_base + MBOX_INTSR0_OFFSET);
		/*
		 * 2. Check interrupt mask-status and clear registers
		 */
		mask_val = readl(mailbox->csr_reg_base + MBOX_INTMSR0_OFFSET);
		clear_val = readl(mailbox->csr_reg_base + MBOX_INTCR0_OFFSET);

		if ((value & mask_val) & clear_val) {
			/*
			 * 3. Update the clear register to reflect outgoing
			 *    interrupts
			 *
			 * A 0-bit in the clear register indicates an interrupt
			 * waiting to be serviced, and therefore masked from
			 * further generation.
			 *
			 * Set the bits of any newly-generated sources to 0.
			 * The only bits which shold remain set are those
			 * already 1 in the clear register and not being set
			 * (i.e. 0 in value & mask_val).
			 */
			writel(~(value & mask_val) & clear_val,
			       mailbox->csr_reg_base + MBOX_INTCR0_OFFSET);
			/*
			 * 4. If set interrupts aren't masked, trigger HW
			 *    interrupt
			 */
			send_interrupt = true;
		}
		break;
	case MBOX_INTCR0_OFFSET:
		/*
		 * 1. Clear interrupt generation register
		 */
		gen_val = readl(mailbox->csr_reg_base + MBOX_INTGR0_OFFSET);
		writel(~value & gen_val,
		       mailbox->csr_reg_base + MBOX_INTGR0_OFFSET);
		/*
		 * 2. Clear interrupt status register
		 */
		status_val = readl(mailbox->csr_reg_base + MBOX_INTSR0_OFFSET);
		writel(~value & status_val,
		       mailbox->csr_reg_base + MBOX_INTSR0_OFFSET);
		/*
		 * 3. Update the clear register unmask any cleared interrupts
		 *
		 * A 1 written to any bit should re-enable that interrupt,
		 * meaning the new value written should be 1 as well. OR.
		 */
		clear_val = readl(mailbox->csr_reg_base + MBOX_INTCR0_OFFSET);
		writel(value | clear_val,
		       mailbox->csr_reg_base + MBOX_INTCR0_OFFSET);
		/*
		 * 4. Clear outgoing HW interrupt
		 */
		MBOX_CLEAR_INT_TO_DEVICE(mailbox);

		/*
		 * Value written to MBOX_INTCR0_OFFSET is not the actual
		 * value stored in memory, so bail here.
		 */
		goto csr_write_exit;
	case MBOX_INTMR0_OFFSET:
		/*
		 * Update the interrupt mask status register
		 * In this register 1 = masked, but in the mask status register
		 * 1 = enabled, so the inverse value must be written.
		 */
		writel(~value, mailbox->csr_reg_base + MBOX_INTMSR0_OFFSET);
		break;
	case MBOX_INTGR1_OFFSET:
		dev_notice(
			mailbox->gxp->dev,
			"Writing to-host interrupt from host. Is this a mistake?\n");
		/*
		 * 1. Set interrupt status register
		 */
		writel(value, mailbox->csr_reg_base + MBOX_INTSR1_OFFSET);
		/*
		 * 2. Check interrupt mask-status and clear registers
		 */
		mask_val = readl(mailbox->csr_reg_base + MBOX_INTMSR1_OFFSET);
		clear_val = readl(mailbox->csr_reg_base + MBOX_INTCR1_OFFSET);

		if ((value & mask_val) & clear_val) {
			/*
			 * 3. Update the clear register to reflect outgoing
			 *    interrupts
			 *
			 * A 0-bit in the clear register indicates an interrupt
			 * waiting to be serviced, and therefore masked from
			 * further generation.
			 *
			 * Set the bits of any newly-generated sources to 0.
			 * The only bits which shold remain set are those
			 * already 1 in the clear register and not being set
			 * (i.e. 0 in value & mask_val).
			 */
			writel(~(value & mask_val) & clear_val,
			       mailbox->csr_reg_base + MBOX_INTCR1_OFFSET);
			/*
			 * 4. If set interrupts aren't masked, trigger HW
			 *    interrupt
			 */
			/*
			 * Software mailboxes don't have a to-host interrupt,
			 * so the host polls the status register and no
			 * further action is required.
			 */
		}
		break;
	case MBOX_INTCR1_OFFSET:
		/*
		 * 1. Clear interrupt generation register
		 */
		gen_val = readl(mailbox->csr_reg_base + MBOX_INTGR1_OFFSET);
		writel(~value & gen_val,
		       mailbox->csr_reg_base + MBOX_INTGR1_OFFSET);
		/*
		 * 2. Clear interrupt status register
		 */
		status_val = readl(mailbox->csr_reg_base + MBOX_INTSR1_OFFSET);
		writel(~value & status_val,
		       mailbox->csr_reg_base + MBOX_INTSR1_OFFSET);
		/*
		 * 3. Update the clear register unmask any cleared interrupts
		 *
		 * A 1 written to any bit should re-enable that interrupt,
		 * meaning the new value written should be 1 as well. OR.
		 */
		clear_val = readl(mailbox->csr_reg_base + MBOX_INTCR1_OFFSET);
		writel(value | clear_val,
		       mailbox->csr_reg_base + MBOX_INTCR1_OFFSET);
		/*
		 * 4. Clear outgoing HW interrupt
		 */
		/*
		 * Software mailboxes don't have a to-host interrupt, so the
		 * host polls the status register and no further action is
		 * required.
		 */

		/*
		 * Value written to MBOX_INTCR1_OFFSET is not the actual
		 * value stored in memory, so bail here.
		 */
		goto csr_write_exit;
	case MBOX_INTMR1_OFFSET:
		/*
		 * Update the interrupt mask status register
		 * In this register 1 = masked, but in the mask status register
		 * 1 = enabled, so the inverse value must be written.
		 */
		writel(~value, mailbox->csr_reg_base + MBOX_INTMSR1_OFFSET);
		break;
	case MBOX_INTSR0_OFFSET:
	case MBOX_INTMSR0_OFFSET:
	case MBOX_INTSR1_OFFSET:
	case MBOX_INTMSR1_OFFSET:
		dev_notice(mailbox->gxp->dev,
			   "Attempt to write read-only mailbox CSR 0x%X\n",
			   reg_offset);
		goto csr_write_exit;
	default:
		dev_notice(mailbox->gxp->dev,
			   "Attempt to write non-existent mailbox CSR 0x%X\n",
			   reg_offset);
		goto csr_write_exit;
	}

	writel(value, mailbox->csr_reg_base + reg_offset);

csr_write_exit:
	if (send_interrupt)
		MBOX_SET_INT_TO_DEVICE(mailbox);

	gxp_release_sync_barrier(mailbox->gxp, MBOX_ACCESS_SYNC_BARRIER);
}

static u32 data_read(struct gxp_mailbox *mailbox, uint reg_offset)
{
	u32 read_value;

	gxp_acquire_sync_barrier(mailbox->gxp, MBOX_ACCESS_SYNC_BARRIER);

	read_value = readl(mailbox->data_reg_base + reg_offset);

	gxp_release_sync_barrier(mailbox->gxp, MBOX_ACCESS_SYNC_BARRIER);

	return read_value;
}

static void data_write(struct gxp_mailbox *mailbox, uint reg_offset, u32 value)
{
	gxp_acquire_sync_barrier(mailbox->gxp, MBOX_ACCESS_SYNC_BARRIER);

	writel(value, mailbox->data_reg_base + reg_offset);

	gxp_release_sync_barrier(mailbox->gxp, MBOX_ACCESS_SYNC_BARRIER);
}

/* IRQ Handling */

static int poll_int_thread(void *data)
{
	u32 status_value, mask_value, masked_status_value;
	struct gxp_mailbox *mailbox = (struct gxp_mailbox *)data;

	while (!kthread_should_stop()) {
		gxp_acquire_sync_barrier(mailbox->gxp,
					 MBOX_ACCESS_SYNC_BARRIER);
		status_value =
			readl(mailbox->csr_reg_base + MBOX_INTSR1_OFFSET);
		mask_value = readl(mailbox->csr_reg_base + MBOX_INTMSR1_OFFSET);
		gxp_release_sync_barrier(mailbox->gxp,
					 MBOX_ACCESS_SYNC_BARRIER);

		masked_status_value = status_value & mask_value;
		if (masked_status_value) {
			if (masked_status_value & ~mailbox->debug_dump_int_mask)
				mailbox->handle_irq(mailbox);

			if (masked_status_value & mailbox->debug_dump_int_mask)
				mailbox->handle_debug_dump_irq(mailbox);

			gxp_mailbox_clear_host_interrupt(
				mailbox, status_value & mask_value);
		}

		/* TODO(b/177701517): Polling frequency is untuned.*/
		msleep(200);
	}

	return 0;
}

/* gxp-mailbox-driver.h interface */

void gxp_mailbox_driver_init(struct gxp_mailbox *mailbox)
{
	/* Clear and unmask all to-device interrupts */
	csr_write(mailbox, MBOX_INTCR0_OFFSET, 0xFFFFFFFF);
	csr_write(mailbox, MBOX_INTMR0_OFFSET, 0x00000000);
	/* Clear and unmask all to-host interrupts */
	csr_write(mailbox, MBOX_INTCR1_OFFSET, 0xFFFFFFFF);
	csr_write(mailbox, MBOX_INTMR1_OFFSET, 0x00000000);

	/* Setup a polling thread to check for to-host "interrupts" */
	mailbox->to_host_poll_task =
		kthread_run(poll_int_thread, mailbox,
			    "gxp_poll_mailbox%d_to_host", mailbox->core_id);

	if (IS_ERR(mailbox->to_host_poll_task)) {
		dev_err(mailbox->gxp->dev,
			"Failed to start polling for incoming updates from mailbox %d\n",
			mailbox->core_id);
	}
}

void gxp_mailbox_driver_exit(struct gxp_mailbox *mailbox)
{
	if (!IS_ERR_OR_NULL(mailbox->to_host_poll_task))
		kthread_stop(mailbox->to_host_poll_task);
}

void __iomem *gxp_mailbox_get_csr_base(struct gxp_dev *gxp, uint index)
{
	return gxp->fwbufs[index].vaddr + AURORA_SCRATCHPAD_OFF +
	       MBOX_CSR_SCRATCHPAD_OFFSET;
}

void __iomem *gxp_mailbox_get_data_base(struct gxp_dev *gxp, uint index)
{
	return gxp->fwbufs[index].vaddr + AURORA_SCRATCHPAD_OFF +
	       MBOX_CSR_SCRATCHPAD_OFFSET + MBOX_DATA_REG_BASE;
}

/* gxp-mailbox-driver.h: CSR-based calls */

void gxp_mailbox_reset_hw(struct gxp_mailbox *mailbox)
{
	csr_write(mailbox, MBOX_MCUCTLR_OFFSET, 1);
}

void gxp_mailbox_generate_device_interrupt(struct gxp_mailbox *mailbox,
					   u32 int_mask)
{
	csr_write(mailbox, MBOX_INTGR0_OFFSET, int_mask);
}

u32 gxp_mailbox_get_device_mask_status(struct gxp_mailbox *mailbox)
{
	return csr_read(mailbox, MBOX_INTMSR0_OFFSET);
}

void gxp_mailbox_clear_host_interrupt(struct gxp_mailbox *mailbox, u32 int_mask)
{
	csr_write(mailbox, MBOX_INTCR1_OFFSET, int_mask);
}

void gxp_mailbox_mask_host_interrupt(struct gxp_mailbox *mailbox, u32 int_mask)
{
	csr_write(mailbox, MBOX_INTMR1_OFFSET, int_mask);
}

u32 gxp_mailbox_get_host_mask_status(struct gxp_mailbox *mailbox)
{
	return csr_read(mailbox, MBOX_INTMSR1_OFFSET);
}

/* gxp-mailbox-driver.h: Data register-based callss */

void gxp_mailbox_write_status(struct gxp_mailbox *mailbox, u32 status)
{
	data_write(mailbox, MBOX_STATUS_OFFSET, status);
}

void gxp_mailbox_write_descriptor(struct gxp_mailbox *mailbox,
				  dma_addr_t descriptor_addr)
{
	data_write(mailbox, MBOX_DESCRIPTOR_ADDR_OFFSET, (u32)descriptor_addr);
}

void gxp_mailbox_write_cmd_queue_tail(struct gxp_mailbox *mailbox, u16 val)
{
	u32 current_resp_head =
		data_read(mailbox, MBOX_CMD_TAIL_RESP_HEAD_OFFSET) &
		RESP_HEAD_MASK;
	u32 new_cmd_tail = (u32)val << CMD_TAIL_SHIFT;

	data_write(mailbox, MBOX_CMD_TAIL_RESP_HEAD_OFFSET,
		   new_cmd_tail | current_resp_head);
}

void gxp_mailbox_write_resp_queue_head(struct gxp_mailbox *mailbox, u16 val)
{
	u32 current_cmd_tail =
		data_read(mailbox, MBOX_CMD_TAIL_RESP_HEAD_OFFSET) &
		CMD_TAIL_MASK;
	u32 new_resp_head = (u32)val << RESP_HEAD_SHIFT;

	data_write(mailbox, MBOX_CMD_TAIL_RESP_HEAD_OFFSET,
		   current_cmd_tail | new_resp_head);
}

u16 gxp_mailbox_read_cmd_queue_head(struct gxp_mailbox *mailbox)
{
	u32 reg_val = data_read(mailbox, MBOX_CMD_HEAD_RESP_TAIL_OFFSET);

	return (u16)((reg_val & CMD_HEAD_MASK) >> CMD_HEAD_SHIFT);
}

u16 gxp_mailbox_read_resp_queue_tail(struct gxp_mailbox *mailbox)
{
	u32 reg_val = data_read(mailbox, MBOX_CMD_HEAD_RESP_TAIL_OFFSET);

	return (u16)((reg_val & RESP_TAIL_MASK) >> RESP_TAIL_SHIFT);
}

void gxp_mailbox_write_cmd_queue_head(struct gxp_mailbox *mailbox, u16 val)
{
	u32 current_resp_tail =
		data_read(mailbox, MBOX_CMD_HEAD_RESP_TAIL_OFFSET) &
		RESP_TAIL_MASK;
	u32 new_cmd_head = (u32)val << CMD_HEAD_SHIFT;

	data_write(mailbox, MBOX_CMD_HEAD_RESP_TAIL_OFFSET,
		   new_cmd_head | current_resp_tail);
}

void gxp_mailbox_write_resp_queue_tail(struct gxp_mailbox *mailbox, u16 val)
{
	u32 current_cmd_head =
		data_read(mailbox, MBOX_CMD_HEAD_RESP_TAIL_OFFSET) &
		CMD_HEAD_MASK;
	u32 new_resp_tail = (u32)val << RESP_TAIL_SHIFT;

	data_write(mailbox, MBOX_CMD_HEAD_RESP_TAIL_OFFSET,
		   current_cmd_head | new_resp_tail);
}

u16 gxp_mailbox_read_cmd_queue_tail(struct gxp_mailbox *mailbox)
{
	u32 reg_val = data_read(mailbox, MBOX_CMD_TAIL_RESP_HEAD_OFFSET);

	return (u16)((reg_val & CMD_TAIL_MASK) >> CMD_TAIL_SHIFT);
}

u16 gxp_mailbox_read_resp_queue_head(struct gxp_mailbox *mailbox)
{
	u32 reg_val = data_read(mailbox, MBOX_CMD_TAIL_RESP_HEAD_OFFSET);

	return (u16)((reg_val & RESP_HEAD_MASK) >> RESP_HEAD_SHIFT);
}
