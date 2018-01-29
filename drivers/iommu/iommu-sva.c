/*
 * Track processes address spaces bound to devices and allocate PASIDs.
 *
 * Copyright (C) 2018 ARM Ltd.
 * Author: Jean-Philippe Brucker <jean-philippe.brucker@arm.com>
 *
 * SPDX-License-Identifier: GPL-2.0
 */

#include <linux/iommu.h>

/**
 * iommu_sva_device_init() - Initialize Shared Virtual Addressing for a device
 * @dev: the device
 * @features: bitmask of features that need to be initialized
 * @max_pasid: max PASID value supported by the device
 *
 * Users of the bind()/unbind() API must call this function to initialize all
 * features required for SVA.
 *
 * - If the device should support multiple address spaces (e.g. PCI PASID),
 *   IOMMU_SVA_FEAT_PASID must be requested.
 *
 *   By default the PASID allocated during bind() is limited by the IOMMU
 *   capacity, and by the device PASID width defined in the PCI capability or in
 *   the firmware description. Setting @max_pasid to a non-zero value smaller
 *   than this limit overrides it.
 *
 * - If the device should support I/O Page Faults (e.g. PCI PRI),
 *   IOMMU_SVA_FEAT_IOPF must be requested.
 *
 * The device should not be be performing any DMA while this function is
 * running.
 *
 * Return 0 if initialization succeeded, or an error.
 */
int iommu_sva_device_init(struct device *dev, unsigned long features,
			  unsigned int max_pasid)
{
	int ret;
	unsigned int min_pasid = 0;
	struct iommu_param *dev_param = dev->iommu_param;
	struct iommu_domain *domain = iommu_get_domain_for_dev(dev);

	if (!domain || !dev_param || !domain->ops->sva_device_init)
		return -ENODEV;

	/*
	 * IOMMU driver updates the limits depending on the IOMMU and device
	 * capabilities.
	 */
	ret = domain->ops->sva_device_init(dev, features, &min_pasid,
					   &max_pasid);
	if (ret)
		return ret;

	/* FIXME: racy. Next version should have a mutex (same as fault handler) */
	dev_param->sva_features = features;
	dev_param->min_pasid = min_pasid;
	dev_param->max_pasid = max_pasid;

	return 0;
}
EXPORT_SYMBOL_GPL(iommu_sva_device_init);

/**
 * iommu_sva_device_shutdown() - Shutdown Shared Virtual Addressing for a device
 * @dev: the device
 *
 * Disable SVA. The device should not be performing any DMA while this function
 * is running.
 */
int iommu_sva_device_shutdown(struct device *dev)
{
	struct iommu_param *dev_param = dev->iommu_param;
	struct iommu_domain *domain = iommu_get_domain_for_dev(dev);

	if (!domain)
		return -ENODEV;

	if (domain->ops->sva_device_shutdown)
		domain->ops->sva_device_shutdown(dev);

	dev_param->sva_features = 0;
	dev_param->min_pasid = 0;
	dev_param->max_pasid = 0;

	return 0;
}
EXPORT_SYMBOL_GPL(iommu_sva_device_shutdown);
