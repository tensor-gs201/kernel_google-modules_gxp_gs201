# SPDX-License-Identifier: GPL-2.0
#
# Makefile for GXP driver.
#

obj-$(CONFIG_GXP) += gxp.o

gxp-objs +=	\
		gxp-bpm.o \
		gxp-debug-dump.o \
		gxp-debugfs.o \
		gxp-doorbell.o \
		gxp-firmware.o \
		gxp-firmware-data.o \
		gxp-lpm.o \
		gxp-mailbox.o \
		gxp-mapping.o \
		gxp-platform.o \
		gxp-range-alloc.o \
		gxp-vd.o

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

# If building via make directly, specify target platform by adding
#     "GXP_PLATFORM=<target>"
# With one of the following values:
#     - CLOUDRIPPER
#     - ZEBU
#     - IP_ZEBU
# Defaults to building for CLOUDRIPPER if not otherwise specified.
GXP_PLATFORM ?= CLOUDRIPPER

# Default to using the HW mailbox and SysMMU
GXP_SW_MAILBOX ?= 0
GXP_HAS_SYSMMU ?= 1

# Setup the linked mailbox implementation and definitions.
ifeq ($(GXP_SW_MAILBOX),1)
	ccflags-y += -DCONFIG_GXP_USE_SW_MAILBOX
	gxp-objs += gxp-sw-mailbox-driver.o
else
	gxp-objs += gxp-hw-mailbox-driver.o
endif

# Setup which version of the gxp-dma interface is used.
ifeq ($(GXP_HAS_SYSMMU),1)
	ccflags-y += -DCONFIG_GXP_HAS_SYSMMU
	gxp-objs += gxp-dma-iommu.o
else
	gxp-objs += gxp-dma-rmem.o
endif

ccflags-y += -DCONFIG_GXP_$(GXP_PLATFORM)

KBUILD_OPTIONS += CONFIG_GXP=m

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) W=1 $(KBUILD_OPTIONS) $(@)
