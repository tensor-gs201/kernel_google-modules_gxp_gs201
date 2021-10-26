/* SPDX-License-Identifier: GPL-2.0 */
/*
 * GXP kernel-userspace interface definitions.
 *
 * Copyright (C) 2020 Google LLC
 */
#ifndef __GXP_H__
#define __GXP_H__

#include <linux/ioctl.h>
#include <linux/types.h>

#define GXP_IOCTL_BASE 0xEE

/* GXP map flag macros */

/* The mask for specifying DMA direction in GXP map flag */
#define GXP_MAP_DIR_MASK		3
/* The targeted DMA direction for the buffer */
#define GXP_MAP_DMA_BIDIRECTIONAL	0
#define GXP_MAP_DMA_TO_DEVICE		1
#define GXP_MAP_DMA_FROM_DEVICE		2

struct gxp_map_ioctl {
	/*
	 * Bitfield indicating which virtual cores to map the buffer for.
	 * To map for virtual core X, set bit X in this field, i.e. `1 << X`.
	 *
	 * This field is not used by the unmap IOCTL, which always unmaps a
	 * buffer for all cores it had been mapped for.
	 */
	__u16 virtual_core_list;
	__u64 host_address;	/* virtual address in the process space */
	__u32 size;		/* size of mapping in bytes */
	/*
	 * Flags indicating mapping attribute requests from the runtime.
	 * Set RESERVED bits to 0 to ensure backwards compatibility.
	 *
	 * Bitfields:
	 *   [1:0]   - DMA_DIRECTION:
	 *               00 = DMA_BIDIRECTIONAL (host/device can write buffer)
	 *               01 = DMA_TO_DEVICE     (host can write buffer)
	 *               10 = DMA_FROM_DEVICE   (device can write buffer)
	 *             Note: DMA_DIRECTION is the direction in which data moves
	 *             from the host's perspective.
	 *   [31:2]  - RESERVED
	 */
	__u32 flags;
	__u64 device_address;	/* returned device address */
};

/* Map host buffer. */
#define GXP_MAP_BUFFER \
	_IOWR(GXP_IOCTL_BASE, 0, struct gxp_map_ioctl)

/*
 * Un-map host buffer previously mapped by GXP_MAP_BUFFER.
 *
 * Only the @device_address field will be used. Other fields will be fetched
 * from the kernel's internal records. It is recommended to use the argument
 * that was passed in GXP_MAP_BUFFER to un-map the buffer.
 */
#define GXP_UNMAP_BUFFER \
	_IOW(GXP_IOCTL_BASE, 1, struct gxp_map_ioctl)

/* GXP sync flag macros */
#define GXP_SYNC_FOR_DEVICE		(0)
#define GXP_SYNC_FOR_CPU		(1)

struct gxp_sync_ioctl {
	/*
	 * The starting address of the buffer to be synchronized. Must be a
	 * device address returned by GXP_MAP_BUFFER.
	 */
	__u64 device_address;
	/* size in bytes to be sync'ed */
	__u32 size;
	/*
	 * offset in bytes at which the sync operation is to begin from the
	 * start of the buffer
	 */
	__u32 offset;
	/*
	 * Flags indicating sync operation requested from the runtime.
	 * Set RESERVED bits to 0 to ensure backwards compatibility.
	 *
	 * Bitfields:
	 *   [0:0]   - Sync direction. Sync for device or CPU.
	 *               0 = sync for device
	 *               1 = sync for CPU
	 *   [31:1]  - RESERVED
	 */
	__u32 flags;
};

/*
 * Sync buffer previously mapped by GXP_MAP_BUFFER.
 *
 * EINVAL: If a mapping for @device_address is not found.
 * EINVAL: If @size equals 0.
 * EINVAL: If @offset plus @size exceeds the mapping size.
 */
#define GXP_SYNC_BUFFER \
	_IOW(GXP_IOCTL_BASE, 2, struct gxp_sync_ioctl)

struct gxp_mailbox_command_ioctl {
	/*
	 * Input:
	 * The virtual core to dispatch the command to.
	 */
	__u16 virtual_core_id;
	/*
	 * Output:
	 * The sequence number assigned to this command. The caller can use
	 * this value to match responses fetched via `GXP_MAILBOX_RESPONSE`
	 * with this command.
	 */
	__u64 sequence_number;
	/*
	 * Input:
	 * Device address to the buffer containing a GXP command. The user
	 * should have obtained this address from the GXP_MAP_BUFFER ioctl.
	 */
	__u64 device_address;
	/*
	 * Input:
	 * Size of the buffer at `device_address` in bytes.
	 */
	__u32 size;
	/*
	 * Input:
	 * Flags describing the command, for use by the GXP device.
	 */
	__u32 flags;
};

/* Push element to the mailbox commmand queue. */
#define GXP_MAILBOX_COMMAND \
	_IOW(GXP_IOCTL_BASE, 3, struct gxp_mailbox_command_ioctl)

/* GXP mailbox response error code values */
#define GXP_RESPONSE_ERROR_NONE         (0)
#define GXP_RESPONSE_ERROR_INTERNAL     (1)
#define GXP_RESPONSE_ERROR_TIMEOUT      (2)

struct gxp_mailbox_response_ioctl {
	/*
	 * Input:
	 * The virtual core to fetch a response from.
	 */
	__u16 virtual_core_id;
	/*
	 * Output:
	 * Sequence number indicating which command this response is for.
	 */
	__u64 sequence_number;
	/*
	 * Output:
	 * Driver error code.
	 * Indicates if the response was obtained successfully,
	 * `GXP_RESPONSE_ERROR_NONE`, or what error prevented the command
	 * from completing successfully.
	 */
	__u16 error_code;
	/*
	 * Output:
	 * Value returned by firmware in response to a command.
	 * Only valid if `error_code` == GXP_RESPONSE_ERROR_NONE
	 */
	__u32 cmd_retval;
};

/*
 * Pop element from the mailbox response queue. Blocks until mailbox response
 * is available.
 */
#define GXP_MAILBOX_RESPONSE \
	_IOWR(GXP_IOCTL_BASE, 4, struct gxp_mailbox_response_ioctl)

struct gxp_specs_ioctl {
	__u8 core_count;
	__u16 version_major;
	__u16 version_minor;
	__u16 version_build;
	__u8 threads_per_core;
	__u32 memory_per_core;		/* measured in kB */
};

/* Query system specs. */
#define GXP_GET_SPECS \
	_IOR(GXP_IOCTL_BASE, 5, struct gxp_specs_ioctl)

struct gxp_virtual_device_ioctl {
	/*
	 * Input:
	 * The number of cores requested for the virtual device.
	 */
	__u8 core_count;
	/*
	 * Input:
	 * The number of threads requested per core.
	 */
	__u16 threads_per_core;
	/*
	 * Input:
	 * The amount of memory requested per core, in kB.
	 */
	__u32 memory_per_core;
	/*
	 * Output:
	 * The ID assigned to the virtual device and shared with its cores.
	 */
	__u32 vdid;
};

/* Allocate virtual device. */
#define GXP_ALLOCATE_VIRTUAL_DEVICE \
	_IOWR(GXP_IOCTL_BASE, 6, struct gxp_virtual_device_ioctl)

#endif /* __GXP_H__ */
