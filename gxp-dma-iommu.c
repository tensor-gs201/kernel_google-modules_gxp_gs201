// SPDX-License-Identifier: GPL-2.0
/*
 * GXP DMA implemented via IOMMU.
 *
 * Copyright (C) 2021 Google LLC
 */

#include <linux/dma-mapping.h>
#include <linux/iommu.h>
#include <linux/platform_device.h>
#include <linux/scatterlist.h>
#include <linux/slab.h>

#include "gxp-config.h"
#include "gxp-dma.h"
#include "gxp-dma-iommu.h"
#include "gxp-iova.h"
#include "gxp-mapping.h"

struct gxp_dma_iommu_manager {
	struct gxp_dma_manager dma_mgr;
	struct iommu_domain *default_domain;
	struct iommu_domain *core_domains[GXP_NUM_CORES];
	int core_vids[GXP_NUM_CORES];
	void __iomem *idma_ssmt_base;
	void __iomem *inst_data_ssmt_base;
};

/**
 * dma_info_to_prot - Translate DMA API directions and attributes to IOMMU API
 *                    page flags.
 * @dir: Direction of DMA transfer
 * @coherent: Is the DMA master cache-coherent?
 * @attrs: DMA attributes for the mapping
 *
 * From drivers/iommu/dma-iommu.c
 *
 * Return: corresponding IOMMU API page protection flags
 */
static int dma_info_to_prot(enum dma_data_direction dir, bool coherent,
			    unsigned long attrs)
{
	int prot = coherent ? IOMMU_CACHE : 0;

	if (attrs & DMA_ATTR_PRIVILEGED)
		prot |= IOMMU_PRIV;
	switch (dir) {
	case DMA_BIDIRECTIONAL:
		return prot | IOMMU_READ | IOMMU_WRITE;
	case DMA_TO_DEVICE:
		return prot | IOMMU_READ;
	case DMA_FROM_DEVICE:
		return prot | IOMMU_WRITE;
	default:
		return 0;
	}
}

/* SSMT handling */

#define INST_SID_FOR_CORE(_x_) ((1 << 6) | ((_x_) << 4) | (0 << 3))
#define DATA_SID_FOR_CORE(_x_) ((1 << 6) | ((_x_) << 4) | (1 << 3))
#define IDMA_SID_FOR_CORE(_x_) ((1 << 6) | ((_x_) << 4))

static inline void ssmt_set_vid_for_sid(void __iomem *ssmt, int vid, u8 sid)
{
	/* NS_READ_STREAM_VID_<sid> */
	writel(vid, (ssmt) + 0x1000u + (0x4u * (sid)));
	/* NS_WRITE_STREAM_VID_<sid> */
	writel(vid, (ssmt) + 0x1200u + (0x4u * (sid)));
}

static inline int ssmt_init(struct gxp_dev *gxp,
			    struct gxp_dma_iommu_manager *mgr)
{
	struct platform_device *pdev =
		container_of(gxp->dev, struct platform_device, dev);
	struct resource *r;

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, "ssmt_idma");
	if (!r) {
		dev_err(gxp->dev, "Failed to find IDMA SSMT register base\n");
		return -EINVAL;
	}

	mgr->idma_ssmt_base = devm_ioremap_resource(gxp->dev, r);
	if (IS_ERR(mgr->idma_ssmt_base)) {
		dev_err(gxp->dev,
			"Failed to map IDMA SSMT register base (%ld)\n",
			PTR_ERR(mgr->idma_ssmt_base));
		return PTR_ERR(mgr->idma_ssmt_base);
	}

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM,
					 "ssmt_inst_data");
	if (!r) {
		dev_err(gxp->dev,
			"Failed to find instruction/data SSMT register base\n");
		return -EINVAL;
	}

	mgr->inst_data_ssmt_base = devm_ioremap_resource(gxp->dev, r);
	if (IS_ERR(mgr->inst_data_ssmt_base)) {
		dev_err(gxp->dev,
			"Failed to map instruction/data SSMT register base (%ld)\n",
			PTR_ERR(mgr->inst_data_ssmt_base));
		return PTR_ERR(mgr->inst_data_ssmt_base);
	}

	return 0;
}

/* Fault handler */

static int sysmmu_fault_handler(struct iommu_fault *fault, void *token)
{
	struct gxp_dev *gxp = (struct gxp_dev *)token;

	switch (fault->type) {
	case IOMMU_FAULT_DMA_UNRECOV:
		dev_err(gxp->dev, "Unrecoverable IOMMU fault!\n");
		break;
	case IOMMU_FAULT_PAGE_REQ:
		dev_err(gxp->dev, "IOMMU page request fault!\n");
		break;
	default:
		dev_err(gxp->dev, "Unexpected IOMMU fault type (%d)\n",
			fault->type);
		return -EAGAIN;
	}

	/*
	 * Normally the iommu driver should fill out the `event` struct for
	 * unrecoverable errors, and the `prm` struct for page request faults.
	 * The SysMMU driver, instead, always fills out the `event` struct.
	 *
	 * Note that the `fetch_addr` and `perm` fields are never filled out,
	 * so we skip printing them.
	 */
	dev_err(gxp->dev, "reason = %08X\n", fault->event.reason);
	dev_err(gxp->dev, "flags = %08X\n", fault->event.flags);
	dev_err(gxp->dev, "pasid = %08X\n", fault->event.pasid);
	dev_err(gxp->dev, "addr = %llX\n", fault->event.addr);

	// Tell the IOMMU driver to carry on
	return -EAGAIN;
}

/* gxp-dma.h Interface */

int gxp_dma_init(struct gxp_dev *gxp)
{
	struct gxp_dma_iommu_manager *mgr;
	unsigned int core;
	int ret;

	/* GXP can only address 32-bit IOVAs */
	ret = dma_set_mask_and_coherent(gxp->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(gxp->dev, "Failed to set DMA mask\n");
		return ret;
	}

	mgr = devm_kzalloc(gxp->dev, sizeof(*mgr), GFP_KERNEL);
	if (!mgr)
		return -ENOMEM;

/* TODO(b/201505925): remove this and prepare a of_node in unittests */
/* SSMT is not supported in unittests */
#ifndef CONFIG_GXP_TEST
	ret = ssmt_init(gxp, mgr);
	if (ret) {
		dev_err(gxp->dev, "Failed to find SSMT\n");
		return ret;
	}
#endif

	mgr->default_domain = iommu_get_domain_for_dev(gxp->dev);
	if (!mgr->default_domain) {
		dev_err(gxp->dev, "Failed to find default IOMMU domain\n");
		return -EIO;
	}

	if (iommu_register_device_fault_handler(gxp->dev, sysmmu_fault_handler,
						gxp)) {
		dev_err(gxp->dev, "Failed to register iommu fault handler\n");
		return -EIO;
	}

	iommu_dev_enable_feature(gxp->dev, IOMMU_DEV_FEAT_AUX);
	if (!iommu_dev_feature_enabled(gxp->dev, IOMMU_DEV_FEAT_AUX)) {
		dev_err(gxp->dev, "Failed to enable aux support in SysMMU\n");
		goto err_unreg_fault_handler;
	}

	for (core = 0; core < GXP_NUM_CORES; core++) {
		mgr->core_domains[core] = iommu_domain_alloc(gxp->dev->bus);
		if (iommu_aux_attach_device(mgr->core_domains[core],
					    gxp->dev)) {
			iommu_domain_free(mgr->core_domains[core]);
			goto err_detach_aux_domains;
		}
		mgr->core_vids[core] =
			iommu_aux_get_pasid(mgr->core_domains[core], gxp->dev);
		dev_notice(gxp->dev, "SysMMU: core%u assigned vid %d\n", core,
			   mgr->core_vids[core]);
/* SSMT is not supported in unittests */
#ifndef CONFIG_GXP_TEST
		/*
		 * TODO(b/194347483) SSMT programming must happen each time
		 * BLK_AURORA is powered on, but currently BLK_AURORA is
		 * turned on at probe and left on until removal.
		 */
		ssmt_set_vid_for_sid(mgr->idma_ssmt_base, mgr->core_vids[core],
				     IDMA_SID_FOR_CORE(core));
		ssmt_set_vid_for_sid(mgr->inst_data_ssmt_base,
				     mgr->core_vids[core],
				     INST_SID_FOR_CORE(core));
		ssmt_set_vid_for_sid(mgr->inst_data_ssmt_base,
				     mgr->core_vids[core],
				     DATA_SID_FOR_CORE(core));
#endif
	}

	gxp->dma_mgr = &(mgr->dma_mgr);

	return 0;

err_detach_aux_domains:
	/* Detach and free any aux domains successfully setup */
	for (core -= 1; core >= 0; core--) {
		iommu_aux_detach_device(mgr->core_domains[core], gxp->dev);
		iommu_domain_free(mgr->core_domains[core]);
	}

err_unreg_fault_handler:
	if (iommu_unregister_device_fault_handler(gxp->dev))
		dev_err(gxp->dev,
			"Failed to unregister SysMMU fault handler\n");

	return -EIO;
}

void gxp_dma_exit(struct gxp_dev *gxp)
{
	struct gxp_dma_iommu_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_iommu_manager, dma_mgr);
	unsigned int core;

	for (core = 0; core < GXP_NUM_CORES; core++) {
		iommu_aux_detach_device(mgr->core_domains[core], gxp->dev);
		iommu_domain_free(mgr->core_domains[core]);
	}

	if (iommu_unregister_device_fault_handler(gxp->dev))
		dev_err(gxp->dev,
			"Failed to unregister SysMMU fault handler\n");
}

#define SYNC_BARRIERS_SIZE       0x100000
#define SYNC_BARRIERS_TOP_OFFSET 0x100000
#define EXT_TPU_MBX_SIZE         0x2000

/* Offset from mailbox base to the device interface that needs to be mapped */
#define MAILBOX_DEVICE_INTERFACE_OFFSET 0x10000

int gxp_dma_map_resources(struct gxp_dev *gxp)
{
	struct gxp_dma_iommu_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_iommu_manager, dma_mgr);
	unsigned int core;
	int ret = 0;

	for (core = 0; core < GXP_NUM_CORES; core++) {
		ret = iommu_map(mgr->core_domains[core], GXP_IOVA_AURORA_TOP,
				gxp->regs.paddr, gxp->regs.size,
				IOMMU_READ | IOMMU_WRITE);
		if (ret)
			goto err;
		/*
		 * Firmware expects to access the sync barriers at a separate
		 * address, lower than the rest of the AURORA_TOP registers.
		 */
		ret = iommu_map(mgr->core_domains[core], GXP_IOVA_SYNC_BARRIERS,
				gxp->regs.paddr + SYNC_BARRIERS_TOP_OFFSET,
				SYNC_BARRIERS_SIZE, IOMMU_READ | IOMMU_WRITE);
		if (ret)
			goto err;
		ret = iommu_map(mgr->core_domains[core], GXP_IOVA_MAILBOX(core),
				gxp->mbx[core].paddr +
					MAILBOX_DEVICE_INTERFACE_OFFSET,
				gxp->mbx[core].size, IOMMU_READ | IOMMU_WRITE);
		if (ret)
			goto err;
		/*
		 * TODO(b/202213606): Map FW regions of all cores in a VD for
		 * each other at VD creation.
		 */
		ret = iommu_map(mgr->core_domains[core], GXP_IOVA_FIRMWARE(0),
				gxp->fwbufs[0].paddr,
				gxp->fwbufs[0].size * GXP_NUM_CORES,
				IOMMU_READ | IOMMU_WRITE);
		if (ret)
			goto err;
		ret = iommu_map(mgr->core_domains[core], GXP_IOVA_CORE_DUMP,
				gxp->coredumpbuf.paddr, gxp->coredumpbuf.size,
				IOMMU_READ | IOMMU_WRITE);
		if (ret)
			goto err;
		ret = iommu_map(mgr->core_domains[core], GXP_IOVA_FW_DATA,
				gxp->fwdatabuf.paddr, gxp->fwdatabuf.size,
				IOMMU_READ | IOMMU_WRITE);
		if (ret)
			goto err;
		/* Only map the TPU mailboxes if they were found on probe */
		if (gxp->tpu_dev.mbx_paddr) {
			ret = iommu_map(
				mgr->core_domains[core],
				GXP_IOVA_EXT_TPU_MBX + core * EXT_TPU_MBX_SIZE,
				gxp->tpu_dev.mbx_paddr +
					core * EXT_TPU_MBX_SIZE,
				EXT_TPU_MBX_SIZE, IOMMU_READ | IOMMU_WRITE);
			if (ret)
				goto err;
		}
		gxp->mbx[core].daddr = GXP_IOVA_MAILBOX(core);
		gxp->fwbufs[core].daddr = GXP_IOVA_FIRMWARE(core);
	}
	gxp->regs.daddr = GXP_IOVA_AURORA_TOP;
	gxp->coredumpbuf.daddr = GXP_IOVA_CORE_DUMP;
	gxp->fwdatabuf.daddr = GXP_IOVA_FW_DATA;

	return ret;

err:
	/*
	 * Attempt to unmap all resources.
	 * Any resource that hadn't been mapped yet will cause `iommu_unmap()`
	 * to return immediately, so its safe to try to unmap everything.
	 */
	gxp_dma_unmap_resources(gxp);
	return ret;
}

void gxp_dma_unmap_resources(struct gxp_dev *gxp)
{
	struct gxp_dma_iommu_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_iommu_manager, dma_mgr);
	unsigned int core;

	for (core = 0; core < GXP_NUM_CORES; core++) {
		iommu_unmap(mgr->core_domains[core], GXP_IOVA_AURORA_TOP,
			    gxp->regs.size);
		iommu_unmap(mgr->core_domains[core], GXP_IOVA_SYNC_BARRIERS,
			    SYNC_BARRIERS_SIZE);
		iommu_unmap(mgr->core_domains[core], GXP_IOVA_MAILBOX(core),
			    gxp->mbx[core].size);
		/*
		 * TODO(b/202213606): A core should only have access to the FW
		 * of other cores if they're in the same VD, and have the FW
		 * region unmapped on VD destruction.
		 */
		iommu_unmap(mgr->core_domains[core], GXP_IOVA_FIRMWARE(0),
			    gxp->fwbufs[0].size * GXP_NUM_CORES);
		iommu_unmap(mgr->core_domains[core], GXP_IOVA_CORE_DUMP,
			    gxp->coredumpbuf.size);
		iommu_unmap(mgr->core_domains[core], GXP_IOVA_FW_DATA,
			    gxp->fwdatabuf.size);
		/* Only unmap the TPU mailboxes if they were found on probe */
		if (gxp->tpu_dev.mbx_paddr) {
			iommu_unmap(mgr->core_domains[core],
				    GXP_IOVA_EXT_TPU_MBX +
					    core * EXT_TPU_MBX_SIZE,
				    EXT_TPU_MBX_SIZE);
		}
	}
}

static inline struct sg_table *
alloc_sgt_for_buffer(void *ptr, size_t size,
		     struct iommu_domain *domain,
		     dma_addr_t daddr)
{
	struct sg_table *sgt;
	ulong offset;
	uint num_ents;
	int ret;
	struct scatterlist *next;
	size_t size_in_page;
	struct page *page;
	void *va_base = ptr;

	/* Calculate the number of entries needed in the table */
	offset = offset_in_page(va_base);
	if (unlikely((size + offset) / PAGE_SIZE >= UINT_MAX - 1 ||
		     size + offset < size))
		return ERR_PTR(-EINVAL);
	num_ents = (size + offset) / PAGE_SIZE;
	if ((size + offset) % PAGE_SIZE)
		num_ents++;

	/* Allocate and setup the table for filling out */
	sgt = kmalloc(sizeof(*sgt), GFP_KERNEL);
	if (!sgt)
		return ERR_PTR(-ENOMEM);

	ret = sg_alloc_table(sgt, num_ents, GFP_KERNEL);
	if (ret) {
		kfree(sgt);
		return ERR_PTR(ret);
	}
	next = sgt->sgl;

	/*
	 * Fill in the first scatterlist entry.
	 * This is the only one which may start at a non-page-aligned address.
	 */
	size_in_page = size > (PAGE_SIZE - offset_in_page(ptr)) ?
			       PAGE_SIZE - offset_in_page(ptr) :
				     size;
	page = phys_to_page(iommu_iova_to_phys(domain, daddr));
	sg_set_page(next, page, size_in_page, offset_in_page(ptr));
	size -= size_in_page;
	ptr += size_in_page;
	next = sg_next(next);

	while (size > 0) {
		/*
		 * Fill in and link the next scatterlist entry.
		 * `ptr` is now page-aligned, so it is only necessary to check
		 * if this entire page is part of the buffer, or if the buffer
		 * ends part way through the page (which means this is the last
		 * entry in the list).
		 */
		size_in_page = size > PAGE_SIZE ? PAGE_SIZE : size;
		page = phys_to_page(iommu_iova_to_phys(
			domain, daddr + (unsigned long long)(ptr - va_base)));
		sg_set_page(next, page, size_in_page, 0);

		size -= size_in_page;
		ptr += size_in_page;
		next = sg_next(next);
	}

	return sgt;
}

#ifdef CONFIG_ANDROID
int gxp_dma_map_tpu_buffer(struct gxp_dev *gxp, uint core_list,
			   struct edgetpu_ext_mailbox_info *mbx_info)
{
	struct gxp_dma_iommu_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_iommu_manager, dma_mgr);
	uint orig_core_list = core_list;
	u64 queue_iova;
	int core;
	int ret;
	int i = 0;

	while (core_list) {
		phys_addr_t cmdq_pa = mbx_info->mailboxes[i].cmdq_pa;
		phys_addr_t respq_pa = mbx_info->mailboxes[i++].respq_pa;

		core = ffs(core_list) - 1;
		core_list &= ~BIT(core);
		queue_iova = GXP_IOVA_TPU_MBX_BUFFER(core);
		ret = iommu_map(mgr->core_domains[core], queue_iova,
				cmdq_pa, mbx_info->cmdq_size, IOMMU_WRITE);
		if (ret)
			goto error;
		ret = iommu_map(mgr->core_domains[core],
				queue_iova + mbx_info->cmdq_size, respq_pa,
				mbx_info->respq_size, IOMMU_READ);
		if (ret) {
			iommu_unmap(mgr->core_domains[core], queue_iova,
				    mbx_info->cmdq_size);
			goto error;
		}
	}
	return 0;

error:
	core_list ^= orig_core_list;
	while (core_list) {
		core = ffs(core_list) - 1;
		core_list &= ~BIT(core);
		queue_iova = GXP_IOVA_TPU_MBX_BUFFER(core);
		iommu_unmap(mgr->core_domains[core], queue_iova,
			    mbx_info->cmdq_size);
		iommu_unmap(mgr->core_domains[core], queue_iova +
			    mbx_info->cmdq_size, mbx_info->respq_size);
	}
	return ret;
}

void gxp_dma_unmap_tpu_buffer(struct gxp_dev *gxp,
			      struct gxp_tpu_mbx_desc mbx_desc)
{
	struct gxp_dma_iommu_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_iommu_manager, dma_mgr);
	uint core_list = mbx_desc.phys_core_list;
	u64 queue_iova;
	int core;

	while (core_list) {
		core = ffs(core_list) - 1;
		core_list &= ~BIT(core);
		queue_iova = GXP_IOVA_TPU_MBX_BUFFER(core);
		iommu_unmap(mgr->core_domains[core], queue_iova,
			    mbx_desc.cmdq_size);
		iommu_unmap(mgr->core_domains[core], queue_iova +
			    mbx_desc.cmdq_size, mbx_desc.respq_size);
	}
}
#endif  // CONFIG_ANDROID

void *gxp_dma_alloc_coherent(struct gxp_dev *gxp, uint core_list, size_t size,
			     dma_addr_t *dma_handle, gfp_t flag,
			     uint gxp_dma_flags)
{
	struct gxp_dma_iommu_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_iommu_manager, dma_mgr);
	void *buf;
	struct sg_table *sgt;
	dma_addr_t daddr;
	int core;
	int ret;

	size = size < PAGE_SIZE ? PAGE_SIZE : size;

	/* Allocate a coherent buffer in the default domain */
	buf = dma_alloc_coherent(gxp->dev, size, &daddr, flag);
	if (!buf) {
		dev_err(gxp->dev, "Failed to allocate coherent buffer\n");
		return NULL;
	}

	sgt = alloc_sgt_for_buffer(buf, size, mgr->default_domain, daddr);
	if (IS_ERR(sgt)) {
		dev_err(gxp->dev,
			"Failed to allocate sgt for coherent buffer\n");
		dma_free_coherent(gxp->dev, size, buf, daddr);
		return NULL;
	}

	/* Create identical mappings in the specified cores' domains */
	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (!(core_list & BIT(core)))
			continue;

		ret = iommu_map_sg(mgr->core_domains[core], daddr, sgt->sgl,
				   sgt->nents, IOMMU_READ | IOMMU_WRITE);
		if (ret != size)
			goto err;
	}

	if (dma_handle)
		*dma_handle = daddr;

	sg_free_table(sgt);
	kfree(sgt);

	return buf;

err:
	for (core -= 1; core >= 0; core--)
		iommu_unmap(mgr->core_domains[core], daddr, size);
	dma_free_coherent(gxp->dev, size, buf, daddr);

	sg_free_table(sgt);
	kfree(sgt);

	return NULL;
}

void gxp_dma_free_coherent(struct gxp_dev *gxp, uint core_list, size_t size,
			   void *cpu_addr, dma_addr_t dma_handle)
{
	struct gxp_dma_iommu_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_iommu_manager, dma_mgr);
	int core;

	size = size < PAGE_SIZE ? PAGE_SIZE : size;

	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (!(core_list & BIT(core)))
			continue;

		if (size !=
		    iommu_unmap(mgr->core_domains[core], dma_handle, size))
			dev_warn(gxp->dev, "Failed to unmap coherent buffer\n");
	}

	dma_free_coherent(gxp->dev, size, cpu_addr, dma_handle);
}

dma_addr_t gxp_dma_map_single(struct gxp_dev *gxp, uint core_list,
			      void *cpu_addr, size_t size,
			      enum dma_data_direction direction,
			      unsigned long attrs, uint gxp_dma_flags)
{
	struct gxp_dma_iommu_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_iommu_manager, dma_mgr);
	dma_addr_t daddr;
	phys_addr_t paddr;
	int prot = dma_info_to_prot(direction, 0, attrs);
	int core;

	daddr = dma_map_single_attrs(gxp->dev, cpu_addr, size, direction,
				     attrs);
	if (dma_mapping_error(gxp->dev, daddr))
		return DMA_MAPPING_ERROR;

	paddr = iommu_iova_to_phys(mgr->default_domain, daddr);
	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (!(core_list & BIT(core)))
			continue;

		if (iommu_map(mgr->core_domains[core], daddr, paddr, size,
			      prot))
			goto err;
	}

	return daddr;

err:
	for (core -= 1; core >= 0; core--)
		iommu_unmap(mgr->core_domains[core], daddr, size);
	dma_unmap_single_attrs(gxp->dev, daddr, size, direction,
			       DMA_ATTR_SKIP_CPU_SYNC);
	return DMA_MAPPING_ERROR;
}

void gxp_dma_unmap_single(struct gxp_dev *gxp, uint core_list,
			  dma_addr_t dma_addr, size_t size,
			  enum dma_data_direction direction,
			  unsigned long attrs)
{
	struct gxp_dma_iommu_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_iommu_manager, dma_mgr);
	int core;

	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (!(core_list & BIT(core)))
			continue;
		if (size !=
		    iommu_unmap(mgr->core_domains[core], dma_addr, size))
			dev_warn(gxp->dev, "Failed to unmap single\n");
	}

	dma_unmap_single_attrs(gxp->dev, dma_addr, size, direction, attrs);
}

dma_addr_t gxp_dma_map_page(struct gxp_dev *gxp, uint core_list,
			    struct page *page, unsigned long offset,
			    size_t size, enum dma_data_direction direction,
			    unsigned long attrs, uint gxp_dma_flags)
{
	struct gxp_dma_iommu_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_iommu_manager, dma_mgr);
	dma_addr_t daddr;
	phys_addr_t paddr;
	int prot = dma_info_to_prot(direction, 0, attrs);
	int core;

	daddr = dma_map_page_attrs(gxp->dev, page, offset, size, direction,
				   attrs);
	if (dma_mapping_error(gxp->dev, daddr))
		return DMA_MAPPING_ERROR;

	paddr = iommu_iova_to_phys(mgr->default_domain, daddr);
	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (!(core_list & BIT(core)))
			continue;

		if (iommu_map(mgr->core_domains[core], daddr, paddr, size,
			      prot))
			goto err;
	}

	return daddr;

err:
	for (core -= 1; core >= 0; core--)
		iommu_unmap(mgr->core_domains[core], daddr, size);
	dma_unmap_page_attrs(gxp->dev, daddr, size, direction,
				DMA_ATTR_SKIP_CPU_SYNC);
	return DMA_MAPPING_ERROR;
}

void gxp_dma_unmap_page(struct gxp_dev *gxp, uint core_list,
			dma_addr_t dma_addr, size_t size,
			enum dma_data_direction direction, unsigned long attrs)
{
	struct gxp_dma_iommu_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_iommu_manager, dma_mgr);
	int core;

	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (!(core_list & BIT(core)))
			continue;
		if (size !=
		    iommu_unmap(mgr->core_domains[core], dma_addr, size))
			dev_warn(gxp->dev, "Failed to unmap page\n");
	}

	dma_unmap_page_attrs(gxp->dev, dma_addr, size, direction, attrs);
}

dma_addr_t gxp_dma_map_resource(struct gxp_dev *gxp, uint core_list,
				phys_addr_t phys_addr, size_t size,
				enum dma_data_direction direction,
				unsigned long attrs, uint gxp_dma_flags)
{
	struct gxp_dma_iommu_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_iommu_manager, dma_mgr);
	dma_addr_t daddr;
	int prot = dma_info_to_prot(direction, 0, attrs);
	int core;

	daddr = dma_map_resource(gxp->dev, phys_addr, size, direction, attrs);
	if (dma_mapping_error(gxp->dev, daddr))
		return DMA_MAPPING_ERROR;

	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (!(core_list & BIT(core)))
			continue;

		if (iommu_map(mgr->core_domains[core], daddr, phys_addr, size,
			      prot))
			goto err;
	}

	return daddr;

err:
	for (core -= 1; core >= 0; core--)
		iommu_unmap(mgr->core_domains[core], daddr, size);
	dma_unmap_resource(gxp->dev, daddr, size, direction,
			   DMA_ATTR_SKIP_CPU_SYNC);
	return DMA_MAPPING_ERROR;
}

void gxp_dma_unmap_resource(struct gxp_dev *gxp, uint core_list,
			    dma_addr_t dma_addr, size_t size,
			    enum dma_data_direction direction,
			    unsigned long attrs)
{
	struct gxp_dma_iommu_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_iommu_manager, dma_mgr);
	int core;

	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (!(core_list & BIT(core)))
			continue;
		if (size !=
		    iommu_unmap(mgr->core_domains[core], dma_addr, size))
			dev_warn(gxp->dev, "Failed to unmap resource\n");
	}

	dma_unmap_resource(gxp->dev, dma_addr, size, direction, attrs);
}

int gxp_dma_map_sg(struct gxp_dev *gxp, uint core_list, struct scatterlist *sg,
		   int nents, enum dma_data_direction direction,
		   unsigned long attrs, uint gxp_dma_flags)
{
	struct gxp_dma_iommu_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_iommu_manager, dma_mgr);
	int nents_mapped;
	dma_addr_t daddr;
	int prot = dma_info_to_prot(direction, 0, attrs);
	int core;
	/* Variables needed to cleanup if an error occurs */
	struct scatterlist *s;
	int i;
	size_t size = 0;

	nents_mapped = dma_map_sg_attrs(gxp->dev, sg, nents, direction, attrs);
	if (!nents_mapped)
		return 0;

	daddr = sg_dma_address(sg);

	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (!(core_list & BIT(core)))
			continue;

		if (!iommu_map_sg(mgr->core_domains[core], daddr, sg, nents,
				  prot))
			goto err;
	}

	return nents_mapped;

err:
	for_each_sg(sg, s, nents, i) {
		size += sg_dma_len(s);
	}

	for (core -= 1; core >= 0; core--)
		iommu_unmap(mgr->core_domains[core], daddr, size);
	dma_unmap_sg_attrs(gxp->dev, sg, nents, direction, attrs);
	return 0;
}

void gxp_dma_unmap_sg(struct gxp_dev *gxp, uint core_list,
		      struct scatterlist *sg, int nents,
		      enum dma_data_direction direction, unsigned long attrs)
{
	struct gxp_dma_iommu_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_iommu_manager, dma_mgr);
	struct scatterlist *s;
	int i;
	size_t size = 0;
	int core;

	for_each_sg(sg, s, nents, i) {
		size += sg_dma_len(s);
	}

	for (core = 0; core < GXP_NUM_CORES; core++) {
		if (!(core_list & BIT(core)))
			continue;
		if (!iommu_unmap(mgr->core_domains[core], sg_dma_address(sg),
				 size))
			dev_warn(gxp->dev, "Failed to unmap sg\n");
	}

	dma_unmap_sg_attrs(gxp->dev, sg, nents, direction, attrs);
}

void gxp_dma_sync_single_for_cpu(struct gxp_dev *gxp, dma_addr_t dma_handle,
				 size_t size,
				 enum dma_data_direction direction)
{
	/* Syncing is not domain specific. Just call through to DMA API */
	dma_sync_single_for_cpu(gxp->dev, dma_handle, size, direction);
}

void gxp_dma_sync_single_for_device(struct gxp_dev *gxp, dma_addr_t dma_handle,
				    size_t size,
				    enum dma_data_direction direction)
{
	/* Syncing is not domain specific. Just call through to DMA API */
	dma_sync_single_for_device(gxp->dev, dma_handle, size, direction);
}

void gxp_dma_sync_sg_for_cpu(struct gxp_dev *gxp, struct scatterlist *sg,
			     int nents, enum dma_data_direction direction)
{
	/* Syncing is not domain specific. Just call through to DMA API */
	dma_sync_sg_for_cpu(gxp->dev, sg, nents, direction);
}

void gxp_dma_sync_sg_for_device(struct gxp_dev *gxp, struct scatterlist *sg,
				int nents, enum dma_data_direction direction)
{
	/* Syncing is not domain specific. Just call through to DMA API */
	dma_sync_sg_for_device(gxp->dev, sg, nents, direction);
}

#ifdef CONFIG_GXP_TEST
/*
 * gxp-dma-iommu.h interface
 * These APIs expose gxp-dma-iommu implementation details for unit testing.
 * They are not meant to be used by other components fo the driver.
 */

struct iommu_domain *gxp_dma_iommu_get_default_domain(struct gxp_dev *gxp)
{
	struct gxp_dma_iommu_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_iommu_manager, dma_mgr);

	if (!mgr)
		return ERR_PTR(-ENODEV);

	return mgr->default_domain;
}

struct iommu_domain *gxp_dma_iommu_get_core_domain(struct gxp_dev *gxp,
						   uint core)
{
	struct gxp_dma_iommu_manager *mgr = container_of(
		gxp->dma_mgr, struct gxp_dma_iommu_manager, dma_mgr);

	if (!mgr)
		return ERR_PTR(-ENODEV);

	if (core >= GXP_NUM_CORES)
		return ERR_PTR(-EINVAL);

	return mgr->core_domains[core];
}
#endif  // CONFIG_GXP_TEST
