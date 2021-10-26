/* SPDX-License-Identifier: GPL-2.0 */
/*
 * GXP DMA IOMMU-specific interface.
 *
 * Copyright (C) 2021 Google LLC
 */
#ifndef __GXP_DMA_IOMMU_H__
#define __GXP_DMA_IOMMU_H__

#include <linux/iommu.h>

#include "gxp-internal.h"

#ifdef CONFIG_GXP_TEST
struct iommu_domain *gxp_dma_iommu_get_default_domain(struct gxp_dev *gxp);
struct iommu_domain *gxp_dma_iommu_get_core_domain(struct gxp_dev *gxp,
						   uint core);
#endif

#endif /* __GXP_DMA_IOMMU_H__ */
