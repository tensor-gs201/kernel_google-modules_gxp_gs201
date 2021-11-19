// SPDX-License-Identifier: GPL-2.0
/*
 * GXP firmware loader.
 *
 * Copyright (C) 2021 Google LLC
 */

#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/elf.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/slab.h>
#include <linux/types.h>

#include "gxp-bpm.h"
#include "gxp-debug-dump.h"
#include "gxp-doorbell.h"
#include "gxp-firmware.h"
#include "gxp-internal.h"
#include "gxp-lpm.h"
#include "gxp-mailbox.h"
#include "gxp-tmp.h"

/* TODO (b/176984045): Clean up gxp-firmware.c */

/* Files need to be copied to /lib/firmware */
#define Q7_ELF_FILE0	"gxp_fw_core0"
#define Q7_ELF_FILE1	"gxp_fw_core1"
#define Q7_ELF_FILE2	"gxp_fw_core2"
#define Q7_ELF_FILE3	"gxp_fw_core3"

static const struct firmware *fw[GXP_NUM_CORES];
static void __iomem *aurora_base;

static char *fw_elf[] = {Q7_ELF_FILE0, Q7_ELF_FILE1, Q7_ELF_FILE2,
			 Q7_ELF_FILE3};

static int elf_load_segments(struct gxp_dev *gxp, const struct firmware *fw,
			     u8 core)
{
	struct elf32_hdr *ehdr;
	struct elf32_phdr *phdr;
	int i, ret = 0;
	const u8 *elf_data = fw->data;

	ehdr = (struct elf32_hdr *)elf_data;
	phdr = (struct elf32_phdr *)(elf_data + ehdr->e_phoff);

	if ((ehdr->e_ident[EI_MAG0] != ELFMAG0) ||
	    (ehdr->e_ident[EI_MAG1] != ELFMAG1) ||
	    (ehdr->e_ident[EI_MAG2] != ELFMAG2) ||
	    (ehdr->e_ident[EI_MAG3] != ELFMAG3)) {
		dev_err(gxp->dev, "Cannot load FW! Invalid ELF format.\n");
		return -EINVAL;
	}

	/* go through the available ELF segments */
	for (i = 0; i < ehdr->e_phnum; i++, phdr++) {
		u64 da = phdr->p_paddr;
		u32 memsz = phdr->p_memsz;
		u32 filesz = phdr->p_filesz;
		u32 offset = phdr->p_offset;
		void *ptr;

		if (phdr->p_type != PT_LOAD)
			continue;

		if (!phdr->p_flags)
			continue;

		if (!memsz)
			continue;

		if (!((da >= (u32)gxp->fwbufs[core].daddr) &&
		   ((da + memsz) <= ((u32)gxp->fwbufs[core].daddr +
				     (u32)gxp->fwbufs[core].size - 1)))) {
			/*
			 * Some BSS data may be referenced from TCM, and can be
			 * skipped while loading
			 */
			dev_err(gxp->dev, "Segment out of bounds: da 0x%llx mem 0x%x. Skipping...\n",
				da, memsz);
			continue;
		}

		dev_notice(gxp->dev, "phdr: type %d da 0x%llx memsz 0x%x filesz 0x%x\n",
			   phdr->p_type, da, memsz, filesz);

		if (filesz > memsz) {
			dev_err(gxp->dev, "Bad phdr filesz 0x%x memsz 0x%x\n",
				filesz, memsz);
			ret = -EINVAL;
			break;
		}

		if (offset + filesz > fw->size) {
			dev_err(gxp->dev, "Truncated fw: need 0x%x avail 0x%zx\n",
				offset + filesz, fw->size);
			ret = -EINVAL;
			break;
		}

		/* grab the kernel address for this device address */
		ptr = gxp->fwbufs[core].vaddr + (da - gxp->fwbufs[core].daddr);
		if (!ptr) {
			dev_err(gxp->dev, "Bad phdr: da 0x%llx mem 0x%x\n",
				da, memsz);
			ret = -EINVAL;
			break;
		}

		/* put the segment where the remote processor expects it */
		if (phdr->p_filesz)
			memcpy_toio(ptr, elf_data + phdr->p_offset, filesz);

		/*
		 * Zero out remaining memory for this segment.
		 */
		if (memsz > filesz)
			memset(ptr + filesz, 0, memsz - filesz);
	}

	return ret;
}

static int gxp_firmware_load(struct gxp_dev *gxp, uint core)
{
	u32 reset_vec, offset;
	void __iomem *core_scratchpad_base;
	int ret;

	dev_notice(gxp->dev, "Loading Q7 ELF file %s\n", fw_elf[core]);
	ret = request_firmware(&fw[core], fw_elf[core], NULL);
	if (ret < 0) {
		dev_err(gxp->dev, "Loading ELF failed (ret=%d)\n", ret);
		return ret;
	}
	dev_notice(gxp->dev, "Q7 ELF file loaded\n");

	/*
	 * Currently, the Q7 FW needs to be statically linked to a base
	 * address where it would be loaded in memory. This requires the
	 * address (where the FW is to be loaded in DRAM) to be
	 * pre-defined, and hence not allocate-able dynamically (using
	 * the kernel's memory management system). Therefore, we are
	 * memremapping a static address and loading the FW there, while
	 * also having compiled the FW with this as the base address
	 * (used by the linker).
	 *
	 * FIXME: This should be fixed by compiling the FW in a
	 * re-locateable way so that it is independent of the load
	 * address, and then using the standard kernel APIs
	 * (kmalloc/dma_alloc_coherrent) to allocate memory and load the
	 * FW.
	 */
	/*
	 * TODO (b/193069216) allocate a dynamic buffer and let
	 * `gxp_dma_map_resources()` map it to the expected paddr
	 */
	gxp->fwbufs[core].vaddr = memremap(gxp->fwbufs[core].paddr,
					   gxp->fwbufs[core].size, MEMREMAP_WC);
	if (!(gxp->fwbufs[core].vaddr)) {
		dev_err(gxp->dev, "FW buf memremap failed\n");
		return -EINVAL;
	}

	/* Load firmware to System RAM */
	ret = elf_load_segments(gxp, fw[core], core);
	if (ret) {
		dev_err(gxp->dev, "Unable to load elf file\n");
		return ret;
	}

	memset(gxp->fwbufs[core].vaddr + AURORA_SCRATCHPAD_OFF, 0,
	       AURORA_SCRATCHPAD_LEN);

	core_scratchpad_base = gxp->fwbufs[core].vaddr + AURORA_SCRATCHPAD_OFF +
			       CORE_SCRATCHPAD_BASE(core);
	offset = SCRATCHPAD_MSG_OFFSET(MSG_CORE_ALIVE);
	writel(0, core_scratchpad_base + offset);
	offset = SCRATCHPAD_MSG_OFFSET(MSG_TOP_ACCESS_OK);
	writel(0, core_scratchpad_base + offset);

	/* TODO(b/188970444): Cleanup logging of addresses */
	dev_notice(gxp->dev,
		   "ELF loaded at virtual: %pK and physical: 0x%llx\n",
		   gxp->fwbufs[core].vaddr, gxp->fwbufs[core].paddr);

	/* Program reset vector */
	reset_vec = gxp_read_32_core(gxp, core,
				     GXP_REG_ALT_RESET_VECTOR);
	dev_notice(gxp->dev, "Current Aurora reset vector for core %u: 0x%x\n",
		   core, reset_vec);
	gxp_write_32_core(gxp, core, GXP_REG_ALT_RESET_VECTOR,
			  gxp->fwbufs[core].daddr);
	dev_notice(gxp->dev, "New Aurora reset vector for core %u: 0x%llx\n",
		   core, gxp->fwbufs[core].daddr);

	/* Configure bus performance monitors */
	gxp_bpm_configure(gxp, core, INST_BPM_OFFSET, BPM_EVENT_READ_XFER);
	gxp_bpm_configure(gxp, core, DATA_BPM_OFFSET, BPM_EVENT_WRITE_XFER);

	return 0;
}

static int gxp_firmware_handshake(struct gxp_dev *gxp, uint core)
{
	u32 offset;
	u32 psm_status;
	u32 expected_top_value;
	void __iomem *core_psm_base, *core_scratchpad_base, *addr;
	uint state;
	int ctr;

	/* Raise wakeup doorbell */
	dev_notice(gxp->dev, "Raising doorbell %d interrupt\n",
		   CORE_WAKEUP_DOORBELL);
	gxp_doorbell_set_listening_core(gxp, CORE_WAKEUP_DOORBELL, core);
	gxp_doorbell_set(gxp, CORE_WAKEUP_DOORBELL);

	/* Wait for core to come up */
	dev_notice(gxp->dev, "Waiting for core %u to power up...\n",
		   core);
	core_psm_base = ((u8 *)aurora_base) + LPM_BLOCK
			+ CORE_PSM_BASE(core);
	ctr = 1000;
	while (ctr) {
		addr = core_psm_base + PSM_STATUS_OFFSET;
		psm_status = (u32) readl(addr); /* 0x60041688 */
		if (psm_status & PSM_STATE_VALID_MASK) {
			state = psm_status & PSM_CURR_STATE_MASK;
			if ((state == PSM_STATE_ACTIVE)
			    || (state == PSM_STATE_CLK_GATED))
				break;
		}
		cpu_relax();
		ctr--;
	}

	if (!ctr) {
		dev_notice(gxp->dev, "Failed!\n");
		return -ETIMEDOUT;
	}
	dev_notice(gxp->dev, "Powered up!\n");

	/* Wait for 500ms. Then check if Q7 core is alive */
	dev_notice(gxp->dev, "Waiting for core %u to respond...\n",
		   core);

	core_scratchpad_base = gxp->fwbufs[core].vaddr + AURORA_SCRATCHPAD_OFF;

	ctr = 500;
	offset = SCRATCHPAD_MSG_OFFSET(MSG_CORE_ALIVE);
	while (ctr--) {
		if (readl(core_scratchpad_base + offset) == Q7_ALIVE_MAGIC)
			break;
		msleep(1 * GXP_TIME_DELAY_FACTOR);
	}
	/*
	 * Currently, the hello_world FW writes a magic number
	 * (Q7_ALIVE_MAGIC) to offset MSG_CORE_ALIVE in the scratchpad
	 * space as an alive message
	 */
	if (readl(core_scratchpad_base + offset) != Q7_ALIVE_MAGIC) {
		dev_err(gxp->dev, "Core %u did not respond!\n", core);
		return -EIO;
	} else {
		dev_notice(gxp->dev, "Core %u is alive!\n", core);
	}

	/*
	 * Currently, the hello_world FW reads the INT_MASK0 register
	 * (written by the driver) to validate TOP access. The value
	 * read is echoed back by the FW to offset MSG_TOP_ACCESS_OK in
	 * the scratchpad space, which must be compared to the value
	 * written in the INT_MASK0 register by the driver for
	 * confirmation.
	 */
	/* TODO (b/182528386): Fix handshake for verifying TOP access */
	offset = SCRATCHPAD_MSG_OFFSET(MSG_TOP_ACCESS_OK);
	expected_top_value = BIT(0);
#ifdef CONFIG_GXP_USE_SW_MAILBOX
	expected_top_value |= BIT(31 - core);
#endif  // CONFIG_GXP_USE_SW_MAILBOX
	if (readl(core_scratchpad_base + offset) != expected_top_value) {
		dev_err(gxp->dev, "TOP access from core %u failed!\n", core);
		return -EIO;
	} else {
		dev_notice(gxp->dev, "TOP access from core %u successful!\n",
			   core);
	}

	/* Stop bus performance monitors */
	gxp_bpm_stop(gxp, core);
	dev_notice(gxp->dev, "Core%u Instruction read transactions: 0x%x\n",
		   core, gxp_bpm_read_counter(gxp, core, INST_BPM_OFFSET));
	dev_notice(gxp->dev, "Core%u Data write transactions: 0x%x\n", core,
		   gxp_bpm_read_counter(gxp, core, DATA_BPM_OFFSET));

	return 0;
}

static void gxp_firmware_unload(struct gxp_dev *gxp, uint core)
{
	if (gxp->fwbufs[core].vaddr) {
		memunmap(gxp->fwbufs[core].vaddr);
		gxp->fwbufs[core].vaddr = NULL;
	}

	if (fw[core]) {
		release_firmware(fw[core]);
		fw[core] = NULL;
	}
}

int gxp_fw_init(struct gxp_dev *gxp)
{
	u32 ver, proc_id;
	uint core;
	struct resource r;
	int ret;

	aurora_base = gxp->regs.vaddr;

	ver = gxp_read_32(gxp, GXP_REG_AURORA_REVISION);
	dev_notice(gxp->dev, "Aurora version: 0x%x\n", ver);

	for (core = 0; core < GXP_NUM_CORES; core++) {
		proc_id = gxp_read_32_core(gxp, core, GXP_REG_PROCESSOR_ID);
		dev_notice(gxp->dev, "Aurora core %u processor ID: 0x%x\n",
			   core, proc_id);
	}

	ret = gxp_acquire_rmem_resource(gxp, &r, "gxp-fw-region");
	if (ret) {
		dev_err(gxp->dev,
			"Unable to acquire firmware reserved memory\n");
		return ret;
	}

	for (core = 0; core < GXP_NUM_CORES; core++) {
		gxp->fwbufs[core].size =
			(resource_size(&r) / GXP_NUM_CORES) & PAGE_MASK;
		gxp->fwbufs[core].paddr =
			r.start + (core * gxp->fwbufs[core].size);
		/*
		 * Firmware buffers are not mapped into kernel VA space until
		 * firmware is ready to be loaded.
		 */
	}

	ret = gxp_acquire_rmem_resource(gxp, &r, "gxp-scratchpad-region");
	if (ret) {
		dev_err(gxp->dev,
			"Unable to acquire shared FW data reserved memory\n");
		return ret;
	}
	gxp->fwdatabuf.size = resource_size(&r);
	gxp->fwdatabuf.paddr = r.start;
	/*
	 * Scratchpad region is not mapped until the firmware data is
	 * initialized.
	 */

	gxp->firmware_running = 0;
	gxp_lpm_init(gxp);
	return 0;
}

void gxp_fw_destroy(struct gxp_dev *gxp)
{
	gxp_lpm_destroy(gxp);
}

int gxp_firmware_run(struct gxp_dev *gxp, uint core)
{
	int ret = 0;

	if (gxp->firmware_running & BIT(core)) {
		dev_err(gxp->dev, "Firmware is already running on core %u\n",
			core);
		return -EBUSY;
	}

	ret = gxp_firmware_load(gxp, core);
	if (ret) {
		dev_err(gxp->dev, "Failed to load firmware on core %u\n", core);
		return ret;
	}

	ret = gxp_lpm_up(gxp, core);
	if (ret) {
		dev_err(gxp->dev, "Failed to power up core %u\n", core);
		goto out_firmware_unload;
	}

	ret = gxp_firmware_handshake(gxp, core);
	if (ret) {
		dev_err(gxp->dev, "Firmware handshake failed on core %u\n",
			core);
		/* TODO (b/176984045): Undo gxp_lpm_up() */
		goto out_firmware_unload;
	}

	/* Initialize ioctl response queues */
	INIT_LIST_HEAD(&(gxp->mailbox_resp_queues[core]));
	init_waitqueue_head(&(gxp->mailbox_resp_waitqs[core]));

	/* Initialize mailbox */
	gxp->mailbox_mgr->mailboxes[core] = gxp_mailbox_alloc(gxp->mailbox_mgr,
							      core);
	if (IS_ERR(gxp->mailbox_mgr->mailboxes[core])) {
		dev_err(gxp->dev,
			"Unable to allocate mailbox (core=%u, ret=%ld)\n", core,
			PTR_ERR(gxp->mailbox_mgr->mailboxes[core]));
		ret = PTR_ERR(gxp->mailbox_mgr->mailboxes[core]);
		gxp->mailbox_mgr->mailboxes[core] = NULL;
		goto out_firmware_unload;
	}

	gxp_mailbox_register_debug_handler(gxp->mailbox_mgr->mailboxes[core],
					   gxp_debug_dump_process_dump,
					   GXP_DEBUG_DUMP_INT_MASK);

	/*
	 * Wait until the FW consumes the new mailbox register values before
	 * allowing messages to be sent thus manipulating the mailbox pointers.
	 */
	msleep(25 * GXP_TIME_DELAY_FACTOR);
	gxp->firmware_running |= BIT(core);

	return ret;

out_firmware_unload:
	gxp_firmware_unload(gxp, core);
	return ret;
}

void gxp_firmware_stop(struct gxp_dev *gxp, uint core)
{
	if (!(gxp->firmware_running & BIT(core)))
		dev_err(gxp->dev, "Firmware is not running on core %u\n", core);

	gxp->firmware_running &= ~BIT(core);

	gxp_mailbox_release(gxp->mailbox_mgr,
			    gxp->mailbox_mgr->mailboxes[core]);
	dev_notice(gxp->dev, "Mailbox %u released\n", core);

	gxp_lpm_down(gxp, core);
	gxp_firmware_unload(gxp, core);
}
