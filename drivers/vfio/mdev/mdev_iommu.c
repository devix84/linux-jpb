// SPDX-License-Identifier: GPL-2.0

#include <linux/device.h>
#include <linux/module.h>
#include <linux/uuid.h>

#include <linux/iommu.h>
#include <linux/mdev.h>

#include "mdev_private.h"

/**
 * mdev_get_pasid - Allocate a PASID for the given mdev
 */
struct io_mm *mdev_get_pasid(struct mdev_device *mdev)
{
	int ret;
	struct io_mm *io_mm;
	struct iommu_domain *domain;
	struct device *dev = &mdev->dev;
	struct mdev_parent *parent = mdev->parent;

	if (!parent->ops->set_pasid)
		return NULL;

	domain = iommu_get_domain_for_dev(parent->dev);
	if (!domain)
		return NULL;

	/*
	 * We assume that when initializing mdev the parent also setup sva with
	 * iommu_sva_device_init. We could do it here, but since the parent
	 * device may offer non-mdev mode as well, parent may want to initialize
	 * SVA with additional features (e.g. PRI).
	 */
	ret = iommu_sva_alloc_pasid(parent->dev, &io_mm);
	if (ret) {
		dev_err(parent->dev, "could not alloc PASID for %s: %d\n",
			dev_name(dev), ret);
		return NULL;
	}

	ret = parent->ops->set_pasid(mdev, io_mm->pasid);
	if (ret)
		goto err_free_pasid;

	dev_info(parent->dev, "allocated PASID %d for mdev %s\n", io_mm->pasid,
		 dev_name(dev));

	return io_mm;

err_free_pasid:
	iommu_sva_free_pasid(parent->dev, io_mm);
	return NULL;
}
EXPORT_SYMBOL_GPL(mdev_get_pasid);

/**
 * mdev_put_pasid - Free PASID obtainer with mdev_get_pasid
 */
void mdev_put_pasid(struct mdev_device *mdev, struct io_mm *io_mm)
{
	struct mdev_parent *parent = mdev->parent;

	if (parent->ops->clear_pasid)
		parent->ops->clear_pasid(mdev, io_mm->pasid);

	iommu_sva_free_pasid(parent->dev, io_mm);
}
EXPORT_SYMBOL_GPL(mdev_put_pasid);
