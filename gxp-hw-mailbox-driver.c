// SPDX-License-Identifier: GPL-2.0
/*
 * GXP hardware-based mailbox driver implementation.
 *
 * Copyright (C) 2021 Google LLC
 */

#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/kthread.h>
#include <linux/of_irq.h>

#include "gxp-tmp.h"
#include "gxp-mailbox.h"
#include "gxp-mailbox-driver.h"
#include "gxp-mailbox-regs.h"

static u32 csr_read(struct gxp_mailbox *mailbox, uint reg_offset)
{
	return readl(mailbox->csr_reg_base + reg_offset);
}

static void csr_write(struct gxp_mailbox *mailbox, uint reg_offset, u32 value)
{
	writel(value, mailbox->csr_reg_base + reg_offset);
}

static u32 data_read(struct gxp_mailbox *mailbox, uint reg_offset)
{
	return readl(mailbox->data_reg_base + reg_offset);
}

static void data_write(struct gxp_mailbox *mailbox, uint reg_offset,
			    u32 value)
{
	writel(value, mailbox->data_reg_base + reg_offset);
}

/* IRQ Handling */

/* Interrupt to signal a response from the device to host */
#define MBOX_DEVICE_TO_HOST_RESPONSE_IRQ_MASK	BIT(0)

static irqreturn_t mailbox_irq_handler(int irq, void *arg)
{
	u32 masked_status;
	struct gxp_mailbox *mailbox = (struct gxp_mailbox *) arg;

	/* Contains only the non-masked, pending interrupt bits */
	masked_status = gxp_mailbox_get_host_mask_status(mailbox);

	/* Clear all pending IRQ bits */
	gxp_mailbox_clear_host_interrupt(mailbox, masked_status);

	if (masked_status & MBOX_DEVICE_TO_HOST_RESPONSE_IRQ_MASK) {
		mailbox->handle_irq(mailbox);
		masked_status &= ~MBOX_DEVICE_TO_HOST_RESPONSE_IRQ_MASK;
	}

	if (masked_status & mailbox->debug_dump_int_mask) {
		mailbox->handle_debug_dump_irq(mailbox);
		masked_status &= ~mailbox->debug_dump_int_mask;
	}

	if (masked_status)
		pr_err_ratelimited(
			"mailbox%d: received unknown interrupt bits 0x%x\n",
			mailbox->core_id, masked_status);

	return IRQ_HANDLED;
}

static void register_irq(struct gxp_mailbox *mailbox)
{
	int err;
	unsigned int virq;

	virq = irq_of_parse_and_map(mailbox->gxp->dev->of_node,
				    mailbox->core_id);
	if (!virq) {
		pr_err("Unable to parse interrupt for core %d from the DT\n",
		       mailbox->core_id);
		return;
	}

	err = request_irq(virq, mailbox_irq_handler, /*flags=*/ 0,
			  "aurora_mbx_irq", (void *) mailbox);
	if (err) {
		pr_err("Unable to register IRQ num=%d; error=%d\n", virq, err);
		return;
	}

	mailbox->interrupt_virq = virq;
	pr_debug("Core %d's mailbox interrupt registered as IRQ %u.\n",
		  mailbox->core_id, virq);
}

static void unregister_irq(struct gxp_mailbox *mailbox)
{
	if (mailbox->interrupt_virq) {
		pr_debug("Freeing IRQ %d\n", mailbox->interrupt_virq);
		free_irq(mailbox->interrupt_virq, mailbox);
		mailbox->interrupt_virq = 0;
	}
}

/* gxp-mailbox-driver.h interface */

void gxp_mailbox_driver_init(struct gxp_mailbox *mailbox)
{
	register_irq(mailbox);
	return;
}

void gxp_mailbox_driver_exit(struct gxp_mailbox *mailbox)
{
	unregister_irq(mailbox);
	return;
}

void __iomem *gxp_mailbox_get_csr_base(struct gxp_dev *gxp, uint index)
{
	return gxp->mbx[index].vaddr;
}

void __iomem *gxp_mailbox_get_data_base(struct gxp_dev *gxp, uint index)
{
	return gxp->mbx[index].vaddr + 0x80;
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

/* gxp-mailbox-driver.h: Data register-based calls */

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
