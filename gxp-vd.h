/* SPDX-License-Identifier: GPL-2.0 */
/*
 * GXP virtual device manager.
 *
 * Copyright (C) 2021 Google LLC
 */
#ifndef __GXP_VD_H__
#define __GXP_VD_H__

#include <linux/types.h>

#include "gxp-internal.h"


/*
 * TODO(b/193180931) cleanup the relationship between the internal GXP modules.
 * For example, whether or not gxp_vd owns the gxp_fw module, and if so, if
 * other modules are expected to access the gxp_fw directly or only via gxp_vd.
 */
/*
 * Initializes the device management subsystem and allocates resources for it.
 * This is expected to be called once per driver lifecycle.
 */
int gxp_vd_init(struct gxp_dev *gxp);
/*
 * Tears down the device management subsystem.
 * This is expected to be called once per driver lifecycle.
 */
void gxp_vd_destroy(struct gxp_dev *gxp);
/*
 * Allocates a virtual device on the requested cores for the specified client.
 * This will also load the FW on, and boot up, the requested cores.
 */
int gxp_vd_allocate(struct gxp_client *client, u16 requested_cores);
/*
 * Returns the physical core ID for the specified virtual_core belonging to
 * this virtual device.
 */
int gxp_vd_virt_core_to_phys_core(struct gxp_client *client, u16 virt_core);
/*
 * Converts a bitfield of virtual core IDs to a bitfield of physical core IDs.
 *
 * If the virtual list contains any invalid IDs, the entire physical ID list
 * will be considered invalid and this function will return 0.
 */
uint gxp_vd_virt_core_list_to_phys_core_list(struct gxp_client *client,
					     u16 virt_core_list);
/*
 * Allocates and initializes a client container to represent a virtual device.
 */
struct gxp_client *gxp_client_create(struct gxp_dev *gxp);
/*
 * Frees up the client container representing a virtual device.
 */
void gxp_client_destroy(struct gxp_client *client);

#endif /* __GXP_VD_H__ */
