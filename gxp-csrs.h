/* SPDX-License-Identifier: GPL-2.0 */
/*
 * GXP CSR definitions.
 *
 * Copyright (C) 2021 Google LLC
 */
#ifndef __GXP_CSRS_H__
#define __GXP_CSRS_H__

#define GXP_REG_DOORBELLS_SET_WRITEMASK 0x1
#define GXP_REG_DOORBELLS_CLEAR_WRITEMASK 0x1

#define GXP_CMU_OFFSET 0x200000

enum gxp_csrs {
	GXP_REG_LPM_VERSION = 0x40000,
	GXP_REG_LPM_PSM_0 = 0x41000,
	GXP_REG_LPM_PSM_1 = 0x42000,
	GXP_REG_LPM_PSM_2 = 0x43000,
	GXP_REG_LPM_PSM_3 = 0x44000,
	GXP_REG_LPM_PSM_4 = 0x45000,
	GXP_REG_AURORA_REVISION = 0x80000,
	GXP_REG_COMMON_INT_POL_0 = 0x81000,
	GXP_REG_COMMON_INT_POL_1 = 0x81004,
	GXP_REG_DEDICATED_INT_POL = 0x81008,
	GXP_REG_RAW_EXT_INT = 0x82000,
	GXP_REG_CORE_PD = 0x82800,
	GXP_REG_GLOBAL_COUNTER_LOW = 0x83000,
	GXP_REG_GLOBAL_COUNTER_HIGH = 0x83004,
	GXP_REG_WDOG_CONTROL = 0x86000,
	GXP_REG_WDOG_VALUE = 0x86008,
	GXP_REG_TIMER_COMPARATOR = 0x90000,
	GXP_REG_TIMER_CONTROL = 0x90004,
	GXP_REG_TIMER_VALUE = 0x90008,
	GXP_REG_DOORBELL_0_STATUS = 0xC0000,
	GXP_REG_DOORBELL_0_SET = 0xC0004,
	GXP_REG_DOORBELL_0_CLEAR = 0xC0008,
	GXP_REG_DOORBELL_1_STATUS = 0xC1000,
	GXP_REG_DOORBELL_1_SET = 0xC1004,
	GXP_REG_DOORBELL_1_CLEAR = 0xC1008,
	GXP_REG_CORE_0_INST_BPM = 0x200000,
	GXP_REG_CORE_1_INST_BPM = 0x210000,
	GXP_REG_CORE_2_INST_BPM = 0x220000,
	GXP_REG_CORE_3_INST_BPM = 0x230000,
};

#define GXP_REG_COMMON_INT_MASK_0_DOORBELLS_MASK 0xFFFFFFFF
#define GXP_REG_ETM_PWRCTL_CORE_RESET_SHIFT	16

enum gxp_core_csrs {
	GXP_REG_INST_BPM = 0x0000,
	GXP_REG_PROFILING_CONDITION = 0x4000,
	GXP_REG_PROCESSOR_ID = 0x4004,
	GXP_REG_ALT_RESET_VECTOR = 0x4008,
	GXP_REG_COMMON_INT_MASK_0 = 0x4010,
	GXP_REG_ETM_PWRCTL = 0xB020,
};

#define DOORBELL_COUNT 32

#define SYNC_BARRIER_COUNT 16
#define SYNC_BARRIER_SHADOW_OFFSET 0x800

#define CORE_PD_BASE(_x_)	((_x_) << 2)
#define CORE_PD_COUNT		4

#define TIMER_BASE(_x_)		((_x_) << 12)
#define TIMER_COMPARATOR_OFFSET	0x0
#define TIMER_CONTROL_OFFSET	0x4
#define TIMER_VALUE_OFFSET	0x8
#define TIMER_COUNT		8

/* CMU offset */
#define PLL_CON0_PLL_AUR 0x100
#define PLL_CON0_NOC_USER 0x610

/* LPM Registers */
#define LPM_VERSION_OFFSET		0x0
#define TRIGGER_CSR_START_OFFSET	0x4
#define IMEM_START_OFFSET		0x8
#define LPM_CONFIG_OFFSET		0xC
#define PSM_DESCRIPTOR_OFFSET		0x10
#define EVENTS_EN_OFFSET		0x100
#define EVENTS_INV_OFFSET		0x140
#define FUNCTION_SELECT_OFFSET		0x180
#define TRIGGER_STATUS_OFFSET		0x184
#define EVENT_STATUS_OFFSET		0x188
#define OPS_OFFSET			0x800
#define PSM_DESCRIPTOR_BASE(_x_)	((_x_) << 2)
#define PSM_DESCRIPTOR_COUNT		5
#define EVENTS_EN_BASE(_x_)		((_x_) << 2)
#define EVENTS_EN_COUNT			16
#define EVENTS_INV_BASE(_x_)		((_x_) << 2)
#define EVENTS_INV_COUNT		16
#define OPS_BASE(_x_)			((_x_) << 2)
#define OPS_COUNT			128
#define PSM_COUNT			5
#define PSM_STATE_TABLE_BASE(_x_)	((_x_) << 8)
#define PSM_STATE_TABLE_COUNT		6
#define PSM_TRANS_BASE(_x_)		((_x_) << 5)
#define PSM_TRANS_COUNT			4
#define PSM_DMEM_BASE(_x_)		((_x_) << 2)
#define PSM_DATA_COUNT			32
#define PSM_NEXT_STATE_OFFSET		0x0
#define PSM_SEQ_ADDR_OFFSET		0x4
#define PSM_TIMER_VAL_OFFSET		0x8
#define PSM_TIMER_EN_OFFSET		0xC
#define PSM_TRIGGER_NUM_OFFSET		0x10
#define PSM_TRIGGER_EN_OFFSET		0x14
#define PSM_ENABLE_STATE_OFFSET		0x80
#define PSM_DATA_OFFSET			0x600
#define PSM_CFG_OFFSET			0x680
#define PSM_START_OFFSET		0x684
#define PSM_STATUS_OFFSET		0x688
#define PSM_DEBUG_CFG_OFFSET		0x68C
#define PSM_BREAK_ADDR_OFFSET		0x694
#define PSM_GPIN_LO_RD_OFFSET		0x6A0
#define PSM_GPIN_HI_RD_OFFSET		0x6A4
#define PSM_GPOUT_LO_RD_OFFSET		0x6B0
#define PSM_GPOUT_HI_RD_OFFSET		0x6B4
#define PSM_DEBUG_STATUS_OFFSET		0x6B8

#endif /* __GXP_CSRS_H__ */
