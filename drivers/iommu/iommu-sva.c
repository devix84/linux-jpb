/*
 * Track processes address spaces bound to devices and allocate PASIDs.
 *
 * Copyright (C) 2018 ARM Ltd.
 * Author: Jean-Philippe Brucker <jean-philippe.brucker@arm.com>
 *
 * SPDX-License-Identifier: GPL-2.0
 */

#include <linux/iommu.h>

/* TODO: stub for the fault queue. Remove later. */
#define iommu_fault_queue_flush(...)

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

	__iommu_sva_unbind_dev_all(dev);

	if (domain->ops->sva_device_shutdown)
		domain->ops->sva_device_shutdown(dev);

	dev_param->sva_features = 0;
	dev_param->min_pasid = 0;
	dev_param->max_pasid = 0;

	return 0;
}
EXPORT_SYMBOL_GPL(iommu_sva_device_shutdown);

/**
 * iommu_sva_bind_device() - Bind a process address space to a device
 * @dev: the device
 * @mm: the mm to bind, caller must hold a reference to it
 * @pasid: valid address where the PASID will be stored
 * @flags: bond properties (IOMMU_SVA_FEAT_*)
 * @drvdata: private data passed to the mm exit handler
 *
 * Create a bond between device and task, allowing the device to access the mm
 * using the returned PASID. A subsequent bind() for the same device and mm will
 * reuse the bond (and return the same PASID), but users will have to call
 * unbind() twice.
 *
 * Callers should have taken care of setting up SVA for this device with
 * iommu_sva_device_init() beforehand. They may also be notified of the bond
 * disappearing, for example when the last task that uses the mm dies, by
 * registering a notifier with iommu_register_mm_exit_handler().
 *
 * If IOMMU_SVA_FEAT_PASID is requested, a PASID is allocated and returned.
 * TODO: The alternative, binding the non-PASID context to an mm, isn't
 * supported at the moment because existing IOMMU domain types initialize the
 * non-PASID context for iommu_map()/unmap() or bypass. This requires a new
 * domain type.
 *
 * If IOMMU_SVA_FEAT_IOPF is not requested, the caller must pin down all
 * mappings shared with the device. mlock() isn't sufficient, as it doesn't
 * prevent minor page faults (e.g. copy-on-write). TODO: !IOPF isn't allowed at
 * the moment.
 *
 * On success, 0 is returned and @pasid contains a valid ID. Otherwise, an error
 * is returned.
 */
int iommu_sva_bind_device(struct device *dev, struct mm_struct *mm, int *pasid,
			  unsigned long flags, void *drvdata)
{
	struct iommu_domain *domain;
	struct iommu_param *dev_param = dev->iommu_param;

	domain = iommu_get_domain_for_dev(dev);
	if (!domain)
		return -EINVAL;

	if (!pasid)
		return -EINVAL;

	if (!dev_param || (flags & ~dev_param->sva_features))
		return -EINVAL;

	if (flags != (IOMMU_SVA_FEAT_PASID | IOMMU_SVA_FEAT_IOPF))
		return -EINVAL;

	return -ENOSYS; /* TODO */
}
EXPORT_SYMBOL_GPL(iommu_sva_bind_device);

/**
 * iommu_sva_unbind_device() - Remove a bond created with iommu_sva_bind_device
 * @dev: the device
 * @pasid: the pasid returned by bind()
 *
 * Remove bond between device and address space identified by @pasid. Users
 * should not call unbind() if the corresponding mm exited (as the PASID might
 * have been reallocated to another process.)
 *
 * The device must not be issuing any more transaction for this PASID. All
 * outstanding page requests for this PASID must have been flushed to the IOMMU.
 *
 * Returns 0 on success, or an error value
 */
int iommu_sva_unbind_device(struct device *dev, int pasid)
{
	struct iommu_domain *domain;

	domain = iommu_get_domain_for_dev(dev);
	if (WARN_ON(!domain))
		return -EINVAL;

	/*
	 * Caller stopped the device from issuing PASIDs, now make sure they are
	 * out of the fault queue.
	 */
	iommu_fault_queue_flush(dev);

	return -ENOSYS; /* TODO */
}
EXPORT_SYMBOL_GPL(iommu_sva_unbind_device);

/**
 * __iommu_sva_unbind_dev_all() - Detach all address spaces from this device
 *
 * When detaching @device from a domain, IOMMU drivers should use this helper.
 */
void __iommu_sva_unbind_dev_all(struct device *dev)
{
	iommu_fault_queue_flush(dev);

	/* TODO */
}
EXPORT_SYMBOL_GPL(__iommu_sva_unbind_dev_all);
