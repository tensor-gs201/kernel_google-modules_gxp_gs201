/* SPDX-License-Identifier: GPL-2.0 */
/*
 * GXP firmware data manager.
 * A sub-module responsible for managing the resources/data regions shared
 * between the GXP driver and FW.
 *
 * Copyright (C) 2021 Google LLC
 */
#ifndef __GXP_FIRMWARE_DATA_H__
#define __GXP_FIRMWARE_DATA_H__

#include "gxp-internal.h"

/**
 * gxp_fw_data_init() - Initializes the FW data manager submodule.
 * @gxp: The parent GXP device
 *
 * Return:
 * 0       - Successfully initialized submodule
 * -ENOMEM - Insufficient memory to create the submodule
 * -ENODEV - Failed to locate the shared driver-device region
 * -Other  - Error codes propagated from internal functions.
 */
int gxp_fw_data_init(struct gxp_dev *gxp);

/**
 * gxp_fw_data_create_app() - Allocates HW and memory resources needed to create
 *                            a GXP device application (1:1 with a GXP driver
 *                            virtual device) used by the specified physical
 *                            cores.
 * @gxp: The parent GXP device
 * @core_list: A bitmap of the physical cores used in this application
 *
 * Return:
 * ptr     - A pointer of the newly created application handle, an error pointer
 *           (PTR_ERR) otherwise.
 * -ENOMEM - Insufficient memory to create the application
 */
void *gxp_fw_data_create_app(struct gxp_dev *gxp, uint core_list);

/**
 * gxp_fw_data_destroy_app() - Deallocates the HW and memory resources used by
 *                             the specified application.
 * @gxp: The parent GXP device
 * @application: The handle to the application to deallocate
 */
void gxp_fw_data_destroy_app(struct gxp_dev *gxp, void *application);

/**
 * gxp_fw_data_destroy() - Destroys the FW data manager submodule and free all
 *                         its resources.
 * @gxp: The parent GXP device
 */
void gxp_fw_data_destroy(struct gxp_dev *gxp);

#endif /* __GXP_FIRMWARE_DATA_H__ */
