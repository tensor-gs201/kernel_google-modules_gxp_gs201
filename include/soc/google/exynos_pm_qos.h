/* SPDX-License-Identifier: GPL-2.0 */
#ifndef _LINUX_EXYNOS_PM_QOS_H
#define _LINUX_EXYNOS_PM_QOS_H
/* interface for the exynos_pm_qos_power infrastructure of the linux kernel.
 *
 * Mark Gross <mgross@linux.intel.com>
 */

#include <linux/plist.h>
#include <linux/workqueue.h>

enum {
	EXYNOS_PM_QOS_RESERVED = 0,
	PM_QOS_CLUSTER0_FREQ_MIN,
	PM_QOS_CLUSTER0_FREQ_MAX,
	PM_QOS_CLUSTER1_FREQ_MIN,
	PM_QOS_CLUSTER1_FREQ_MAX,
	PM_QOS_CLUSTER2_FREQ_MIN,
	PM_QOS_CLUSTER2_FREQ_MAX,
	PM_QOS_DEVICE_THROUGHPUT,
	PM_QOS_INTCAM_THROUGHPUT,
	PM_QOS_DEVICE_THROUGHPUT_MAX,
	PM_QOS_INTCAM_THROUGHPUT_MAX,
	PM_QOS_BUS_THROUGHPUT,
	PM_QOS_BUS_THROUGHPUT_MAX,
	PM_QOS_DISPLAY_THROUGHPUT,
	PM_QOS_DISPLAY_THROUGHPUT_MAX,
	PM_QOS_CAM_THROUGHPUT,
	PM_QOS_CAM_THROUGHPUT_MAX,
	PM_QOS_MFC_THROUGHPUT,
	PM_QOS_MFC_THROUGHPUT_MAX,
	PM_QOS_TNR_THROUGHPUT,
	PM_QOS_TNR_THROUGHPUT_MAX,
	PM_QOS_BO_THROUGHPUT,
	PM_QOS_BO_THROUGHPUT_MAX,
	PM_QOS_GPU_THROUGHPUT_MIN,
	PM_QOS_GPU_THROUGHPUT_MAX,
	EXYNOS_PM_QOS_NUM_CLASSES,
};

struct exynos_pm_qos_request {
	struct plist_node node;
	int exynos_pm_qos_class;
	struct delayed_work work; /* for exynos_pm_qos_update_request_timeout */
	const char *func;
	unsigned int line;
};

static inline void exynos_pm_qos_add_request(struct exynos_pm_qos_request *req,
					     int exynos_pm_qos_class, s32 value)
{
}

static inline void
exynos_pm_qos_update_request(struct exynos_pm_qos_request *req, s32 new_value)
{
}

static inline void
exynos_pm_qos_remove_request(struct exynos_pm_qos_request *req)
{
}

#endif /* _LINUX_EXYNOS_PM_QOS_H */
