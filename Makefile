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
		gxp-mb-notification.o \
		gxp-platform.o \
		gxp-range-alloc.o \
		gxp-pm.o \
		gxp-telemetry.o \
		gxp-thermal.o \
		gxp-vd.o \
		gxp-wakelock.o

KERNEL_SRC ?= /lib/modules/$(shell uname -r)/build
M ?= $(shell pwd)

# Obtain the current git commit hash for logging on probe
GIT_PATH=$(shell cd $(KERNEL_SRC); readlink -e $(M))
ifeq ($(shell git --git-dir=$(GIT_PATH)/.git rev-parse --is-inside-work-tree),true)
        GIT_REPO_STATE=$(shell (git --git-dir=$(GIT_PATH)/.git --work-tree=$(GIT_PATH) status --porcelain | grep -q .) && echo -dirty)
        ccflags-y       += -DGIT_REPO_TAG=\"$(shell git --git-dir=$(GIT_PATH)/.git rev-parse --short HEAD)$(GIT_REPO_STATE)\"
else
        ccflags-y       += -DGIT_REPO_TAG=\"Not\ a\ git\ repository\"
endif

# If building via make directly, specify target platform by adding
#     "GXP_PLATFORM=<target>"
# With one of the following values:
#     - CLOUDRIPPER
#     - ZEBU
#     - IP_ZEBU
# Defaults to building for CLOUDRIPPER if not otherwise specified.
GXP_PLATFORM ?= CLOUDRIPPER

# Test against the build closet to production mode, choose CLOUDRIPPER.
ifdef CONFIG_GXP_TEST
	GXP_PLATFORM = CLOUDRIPPER
endif

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
	# For gem5, need to adopt dma interface without aux domain.
	ifeq ($(GXP_PLATFORM), GEM5)
		gxp-objs += gxp-dma-iommu-gem5.o
	else
		gxp-objs += gxp-dma-iommu.o
	endif
else
	gxp-objs += gxp-dma-rmem.o
endif

ccflags-y += -DCONFIG_GXP_$(GXP_PLATFORM)

KBUILD_OPTIONS += CONFIG_GXP=m

ifdef CONFIG_GXP_TEST
subdir-ccflags-y        += -Wall -Werror
obj-y           += unittests/
include $(srctree)/drivers/gxp/unittests/Makefile.include
$(call include_test_path, $(gxp-objs))
endif

# Access TPU driver's exported symbols.
KBUILD_EXTRA_SYMBOLS += ../google-modules/edgetpu/janeiro/drivers/edgetpu/Module.symvers

modules modules_install clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M) W=1 $(KBUILD_OPTIONS) $(@)
