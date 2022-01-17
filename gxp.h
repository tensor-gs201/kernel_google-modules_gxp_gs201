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

/*
 * mmap offsets for logging and tracing buffers
 * Requested size will be divided evenly among all cores. The whole buffer
 * must be page-aligned, and the size of each core's buffer must be a multiple
 * of PAGE_SIZE.
 */
#define GXP_MMAP_LOG_BUFFER_OFFSET	0x10000
#define GXP_MMAP_TRACE_BUFFER_OFFSET	0x20000

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

#define ETM_TRACE_LSB_MASK 0x1
#define ETM_TRACE_SYNC_MSG_PERIOD_MIN 8
#define ETM_TRACE_SYNC_MSG_PERIOD_MAX 256
#define ETM_TRACE_PC_MATCH_MASK_LEN_MAX 31

/*
 * For all *_enable and pc_match_sense fields, only the least significant bit is
 * considered. All other bits are ignored.
 */
struct gxp_etm_trace_start_ioctl {
	__u16 virtual_core_id;
	__u8 trace_ram_enable; /* Enables local trace memory. */
	/* When set, trace output is sent out on the ATB interface. */
	__u8 atb_enable;
	/* Enables embedding timestamp information in trace messages. */
	__u8 timestamp_enable;
	/*
	 * Determines the rate at which synchronization messages are
	 * automatically emitted in the output trace.
	 * Valid values: 0, 8, 16, 32, 64, 128, 256
	 * Eg. A value of 16 means 1 synchronization message will be emitted
	 * every 16 messages.
	 * A value of 0 means no synchronization messages will be emitted.
	 */
	__u16 sync_msg_period;
	__u8 pc_match_enable; /* PC match causes Stop trigger. */
	/*
	 * 32-bit address to compare to processor PC when pc_match_enable = 1.
	 * A match for a given executed instruction triggers trace stop.
	 * Note: trigger_pc is ignored when pc_match_enable = 0.
	 */
	__u32 trigger_pc;
	/*
	 * Indicates how many of the lower bits of trigger_pc to ignore.
	 * Valid values: 0 to 31
	 * Note: pc_match_mask_length is ignored when pc_match_enable = 0.
	 */
	__u8 pc_match_mask_length;
	/* When 0, match when the processor's PC is in-range of trigger_pc and
	 * mask. When 1, match when the processor's PC is out-of-range of
	 * trigger_pc and mask.
	 * Note: pc_match_sense is ignored when pc_match_enable = 0.
	 */
	__u8 pc_match_sense;
};

/* Configure ETM trace registers and start ETM tracing. */
#define GXP_ETM_TRACE_START_COMMAND \
	_IOW(GXP_IOCTL_BASE, 7, struct gxp_etm_trace_start_ioctl)

/*
 * Halts trace generation via a software trigger. The virtual core id is passed
 * in as an input.
 */
#define GXP_ETM_TRACE_SW_STOP_COMMAND \
	_IOW(GXP_IOCTL_BASE, 8, __u16)

/*
 * Users should call this IOCTL after tracing has been stopped for the last
 * trace session of the core. Otherwise, there is a risk of having up to 3 bytes
 * of trace data missing towards the end of the trace session.
 * This is a workaround for b/180728272 and b/181623511.
 * The virtual core id is passed in as an input.
 */
#define GXP_ETM_TRACE_CLEANUP_COMMAND \
	_IOW(GXP_IOCTL_BASE, 9, __u16)

#define GXP_TRACE_HEADER_SIZE 256
#define GXP_TRACE_RAM_SIZE 4096
struct gxp_etm_get_trace_info_ioctl {
	/*
	 * Input:
	 * The virtual core to fetch a response from.
	 */
	__u16 virtual_core_id;
	/*
	 * Input:
	 * The type of data to retrieve.
	 * 0: Trace Header only
	 * 1: Trace Header + Trace Data in Trace RAM
	 */
	__u8 type;
	/*
	 * Input:
	 * Trace header user space address to contain trace header information
	 * that is used for decoding the trace.
	 */
	__u64 trace_header_addr;
        /*
         * Input:
	 * Trace data user space address to contain Trace RAM data.
	 * Note: trace_data field will be empty if type == 0
	 */
        __u64 trace_data_addr;
};

/* Retrieves trace header and/or trace data for decoding purposes. */
#define GXP_ETM_GET_TRACE_INFO_COMMAND \
	_IOWR(GXP_IOCTL_BASE, 10, struct gxp_etm_get_trace_info_ioctl)

#define GXP_TELEMETRY_TYPE_LOGGING	(0)
#define GXP_TELEMETRY_TYPE_TRACING	(1)

/*
 * Enable either logging or software tracing for all cores.
 * Accepts either `GXP_TELEMETRY_TYPE_LOGGING` or `GXP_TELEMETRY_TYPE_TRACING`
 * to specify whether logging or software tracing is to be enabled.
 *
 * Buffers for logging or tracing must have already been mapped via an `mmap()`
 * call with the respective offset and initialized by the client, prior to
 * calling this ioctl.
 *
 * If firmware is already running on any cores, they will be signaled to begin
 * logging/tracing to their buffers. Any cores booting after this call will
 * begin logging/tracing as soon as their firmware is able to.
 */
#define GXP_ENABLE_TELEMETRY _IOWR(GXP_IOCTL_BASE, 11, __u8)

/*
 * Disable either logging or software tracing for all cores.
 * Accepts either `GXP_TELEMETRY_TYPE_LOGGING` or `GXP_TELEMETRY_TYPE_TRACING`
 * to specify whether logging or software tracing is to be disabled.
 *
 * This call will block until any running cores have been notified and ACKed
 * that they have disabled the specified telemetry type.
 */
#define GXP_DISABLE_TELEMETRY _IOWR(GXP_IOCTL_BASE, 12, __u8)

struct gxp_tpu_mbx_queue_ioctl {
	__u32 tpu_fd; /* TPU virtual device group fd */
	/*
	 * Bitfield indicating which virtual cores to allocate and map the
	 * buffers for.
	 * To map for virtual core X, set bit X in this field, i.e. `1 << X`.
	 *
	 * This field is not used by the unmap IOCTL, which always unmaps the
	 * buffers for all cores it had been mapped for.
	 */
	__u32 virtual_core_list;
	/*
	 * The user address of an edgetpu_mailbox_attr struct, containing
	 * cmd/rsp queue size, mailbox priority and other relevant info.
	 * This structure is defined in edgetpu.h in the TPU driver.
	 */
	__u64 attr_ptr;
};

/*
 * Map TPU-DSP mailbox cmd/rsp queue buffers.
 */
#define GXP_MAP_TPU_MBX_QUEUE \
	_IOW(GXP_IOCTL_BASE, 13, struct gxp_tpu_mbx_queue_ioctl)

/*
 * Un-map TPU-DSP mailbox cmd/rsp queue buffers previously mapped by
 * GXP_MAP_TPU_MBX_QUEUE.
 *
 * Only the @tpu_fd field will be used. Other fields will be fetched
 * from the kernel's internal records. It is recommended to use the argument
 * that was passed in GXP_MAP_TPU_MBX_QUEUE to un-map the buffers.
 */
#define GXP_UNMAP_TPU_MBX_QUEUE \
	_IOW(GXP_IOCTL_BASE, 14, struct gxp_tpu_mbx_queue_ioctl)

struct gxp_register_telemetry_eventfd_ioctl {
	/*
	 * File-descriptor obtained via eventfd().
	 *
	 * Not used during the unregister step; the driver will unregister
	 * whichever eventfd it has currently registered for @type, if any.
	 */
	__u32 eventfd;
	/*
	 * Either `GXP_TELEMETRY_TYPE_LOGGING` or `GXP_TELEMETRY_TYPE_TRACING`.
	 * The driver will signal @eventfd whenever any core signals a
	 * telemetry state change while this type of telemetry is active.
	 */
	__u8 type;
};

#define GXP_REGISTER_TELEMETRY_EVENTFD                                         \
	_IOW(GXP_IOCTL_BASE, 15, struct gxp_register_telemetry_eventfd_ioctl)

#define GXP_UNREGISTER_TELEMETRY_EVENTFD                                       \
	_IOW(GXP_IOCTL_BASE, 16, struct gxp_register_telemetry_eventfd_ioctl)

/*
 * Reads the 2 global counter registers in AURORA_TOP and combines them to
 * return the full 64-bit value of the counter.
 */
#define GXP_READ_GLOBAL_COUNTER _IOR(GXP_IOCTL_BASE, 17, __u64)

#endif /* __GXP_H__ */
