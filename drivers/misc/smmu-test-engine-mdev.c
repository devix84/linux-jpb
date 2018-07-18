// SPDX-License-Identifier: GPL-2.0
/*
 * Sub-assignment of SMMUv3 Test Engine frames
 *
 * Copyright (C) 2018 ARM Limited
 */

#define pr_fmt(fmt) "smmute-mdev: " fmt

#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/eventfd.h>
#include <linux/fs.h>
#include <linux/kernel.h>
#include <linux/mdev.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/uaccess.h>
#include <uapi/linux/vfio.h>

#include <linux/smmu-test-engine.h>

#define to_smmute_device(dev) dev_get_drvdata(dev)

struct smmute_mdev {
	struct smmute_device	*smmute;
	struct mdev_device	*mdev;
	size_t			page_nr;
	unsigned long		msi_vector;
	struct eventfd_ctx	*msi_trigger;
};

static irqreturn_t smmute_mdev_msi_handler(int irq, void *arg)
{
	struct smmute_mdev *smmute_mdev = arg;

	eventfd_signal(smmute_mdev->msi_trigger, 1);
	return IRQ_HANDLED;
}

#define smmute_mdev_for_each_frame(i, smmute_mdev) \
	for (i = smmute_mdev->page_nr * SMMUTE_FRAMES_PER_PAGE; \
	     i < (smmute_mdev->page_nr + 1) * SMMUTE_FRAMES_PER_PAGE; \
	     i++)

static int smmute_mdev_set_pasid(struct mdev_device *mdev, int pasid)
{
	struct smmute_mdev *smmute_mdev = mdev_get_drvdata(mdev);
	struct smmute_pframe *frame;
	int i;

	/* Initialize all privileged frames in this mdev's page */
	smmute_mdev_for_each_frame(i, smmute_mdev) {
		frame = smmute_privileged_frame(smmute_mdev->smmute->pairs, i);

		writel_relaxed(pasid, &frame->substreamid);
	}
	return 0;
}

static int smmute_mdev_clear_pasid(struct mdev_device *mdev, int unused)
{
	/*
	 * Prevent userspace from doing DMA when no PASID is allocated. Setting
	 * the SSID register to 0 or ~0U wouldn't have the desired effect - it
	 * would allows user to access no-pasid address space, which is owned by
	 * the kernel. To disable DMA, write a PASID that is not allocatable,
	 * and causes the test engine to fault with ENGINE_FRAME_MISCONFIGURED.
	 */
	return smmute_mdev_set_pasid(mdev, 0x100000);
}

static int smmute_mdev_set_msi(struct smmute_mdev *smmute_mdev)
{
	int i;
	struct smmute_uframe *frame;

	smmute_mdev_for_each_frame(i, smmute_mdev) {
		/*
		 * DESIGN BUG: MSI must not be under user control! These should
		 * be in the privileged frame. Instead we have to poke the user
		 * frame and hope that the application doesn't misbehave. Yuck!
		 */
		frame = smmute_user_frame(smmute_mdev->smmute->pairs, i);

		writel_relaxed(1, &frame->msiaddress);
		writel_relaxed(smmute_mdev->msi_vector, &frame->msidata);
		writel_relaxed(0, &frame->msiattr);
	}
	return 0;
}

static int smmute_mdev_clear_msi(struct smmute_mdev *smmute_mdev)
{
	int i;
	struct smmute_uframe *frame;

	smmute_mdev_for_each_frame(i, smmute_mdev) {
		frame = smmute_user_frame(smmute_mdev->smmute->pairs, i);

		writel_relaxed(0, &frame->msiaddress);
	}
	return 0;
}

static long smmute_mdev_get_info(struct smmute_mdev *smmute_mdev,
				 struct vfio_device_info *info)
{
	info->flags		= VFIO_DEVICE_FLAGS_PLATFORM;
	info->num_regions	= 1;
	info->num_irqs		= 1;
	return 0;
}

static long smmute_mdev_get_region_info(struct smmute_mdev *smmute_mdev,
					struct vfio_region_info *info)
{
	if (info->index != 0)
		return -EINVAL;

	info->flags		= VFIO_REGION_INFO_FLAG_MMAP;
	info->cap_offset	= 0;
	info->size		= 0x10000;
	info->offset		= 0;
	return 0;
}

static long smmute_mdev_get_irq_info(struct smmute_mdev *smmute_mdev,
				     struct vfio_irq_info *info)
{
	if (info->index != 0)
		return -EINVAL;

	info->flags		= VFIO_IRQ_INFO_EVENTFD;
	info->count		= 1;
	return 0;
}

static long smmute_mdev_set_irqs(struct mdev_device *mdev,
				 void __user *arg)
{
	int ret;
	struct device *dev = mdev_dev(mdev);
	struct smmute_mdev *smmute_mdev = mdev_get_drvdata(mdev);
	struct pci_dev *pdev = to_pci_dev(mdev_parent_dev(mdev));
	/* This is just a demo, we don't support disabling or masking */
	u32 flag_mask = VFIO_IRQ_SET_ACTION_TRIGGER | VFIO_IRQ_SET_DATA_EVENTFD;
	struct eventfd_ctx *trigger;
	struct {
		struct vfio_irq_set	hdr;
		s32			eventfd;
	} irq_set;

	if (copy_from_user(&irq_set, arg, sizeof(irq_set)))
		return -EFAULT;

	if (irq_set.hdr.argsz < sizeof(irq_set))
		return -EINVAL;

	if (irq_set.hdr.index != 0 || irq_set.hdr.start != 0 ||
	    irq_set.hdr.count != 1)
		return -EINVAL;

	if (irq_set.hdr.flags != flag_mask) {
		dev_err(dev, "invalid IRQ flags\n");
		return -EINVAL;
	}

	if (smmute_mdev->msi_trigger)
		return -EEXIST;

	trigger = eventfd_ctx_fdget(irq_set.eventfd);
	if (IS_ERR(trigger)) {
		dev_err(dev, "could not get eventfd %d\n", irq_set.eventfd);
		return PTR_ERR(trigger);
	}

	ret = pci_irq_vector(pdev, smmute_mdev->msi_vector);
	if (ret < 0) {
		dev_err(dev, "could not get IRQ for vector %lu\n",
			smmute_mdev->msi_vector);
		goto err_put_fd;
	}

	ret = request_irq(ret, smmute_mdev_msi_handler, 0, dev_name(dev),
			  smmute_mdev);
	if (ret) {
		dev_err(dev, "could not request MSI %lu\n",
			smmute_mdev->msi_vector);
		goto err_put_fd;
	}

	ret = smmute_mdev_set_msi(smmute_mdev);
	if (ret)
		goto err_free_irq;

	smmute_mdev->msi_trigger = trigger;

	return 0;

err_free_irq:
	free_irq(pci_irq_vector(pdev, smmute_mdev->msi_vector), smmute_mdev);
err_put_fd:
	eventfd_ctx_put(trigger);
	return ret;
}

static long smmute_mdev_ioctl(struct mdev_device *mdev, unsigned int cmd,
			      unsigned long arg)
{
	int ret;
	unsigned long minsz;
	struct smmute_mdev *smmute_mdev;

	smmute_mdev = mdev_get_drvdata(mdev);

	switch (cmd) {
	case VFIO_DEVICE_GET_INFO:
	{
		struct vfio_device_info info;

		minsz = offsetofend(struct vfio_device_info, num_irqs);

		if (copy_from_user(&info, (void __user *)arg, minsz))
			return -EFAULT;

		if (info.argsz < minsz)
			return -EINVAL;

		ret = smmute_mdev_get_info(smmute_mdev, &info);
		if (ret)
			return ret;

		if (copy_to_user((void __user *)arg, &info, minsz))
			return -EFAULT;

		return 0;
	}
	case VFIO_DEVICE_GET_REGION_INFO:
	{
		struct vfio_region_info info;

		minsz = offsetofend(struct vfio_region_info, offset);

		if (copy_from_user(&info, (void __user *)arg, minsz))
			return -EFAULT;

		if (info.argsz < minsz)
			return -EINVAL;

		ret = smmute_mdev_get_region_info(smmute_mdev, &info);
		if (ret)
			return ret;

		if (copy_to_user((void __user *)arg, &info, minsz))
			return -EFAULT;

		return 0;
	}
	case VFIO_DEVICE_GET_IRQ_INFO:
	{
		struct vfio_irq_info info;

		minsz = offsetofend(struct vfio_irq_info, count);

		if (copy_from_user(&info, (void __user *)arg, minsz))
			return -EFAULT;

		if (info.argsz < minsz)
			return -EINVAL;

		ret = smmute_mdev_get_irq_info(smmute_mdev, &info);
		if (ret)
			return ret;

		if (copy_to_user((void __user *)arg, &info, minsz))
			return -EFAULT;

		return 0;
	}
	case VFIO_DEVICE_SET_IRQS:
	{
		struct vfio_irq_set hdr;

		minsz = offsetofend(struct vfio_irq_set, count);

		if (copy_from_user(&hdr, (void __user *)arg, minsz))
			return -EFAULT;

		if (hdr.argsz < minsz)
			return -EINVAL;

		ret = smmute_mdev_set_irqs(mdev, (void __user *)arg);
		return ret;
	}
	default:
		return -ENOTTY;
	}
}

static int smmute_mdev_mmap(struct mdev_device *mdev, struct vm_area_struct *vma)
{
	struct smmute_mdev *smmute_mdev = mdev_get_drvdata(mdev);
	struct pci_dev *pdev = to_pci_dev(smmute_mdev->smmute->dev);

	size_t len = vma->vm_end - vma->vm_start;
	size_t pg_shift = 16;

	/*
	 * We have a full 64k page of frames allocated by _create. User and
	 * privileged pages are interleaved, so multiply the offset by 2.
	 */
	off_t page_off = smmute_mdev->page_nr << (pg_shift + 1);
	unsigned long page_addr = pci_resource_start(pdev, 0) + page_off;

	if (vma->vm_end < vma->vm_start)
		return -EINVAL;
	if (len != 1 << pg_shift)
		return -EINVAL;
	if ((vma->vm_flags & VM_SHARED) == 0)
		return -EINVAL;
	if (vma->vm_pgoff != 0)
		return -EINVAL;

	vma->vm_page_prot = pgprot_device(vma->vm_page_prot);

	return remap_pfn_range(vma, vma->vm_start, page_addr >> PAGE_SHIFT, len,
			       vma->vm_page_prot);
}

static int smmute_mdev_open(struct mdev_device *mdev)
{
	return 0;
}

static void smmute_mdev_close(struct mdev_device *mdev)
{
	struct smmute_mdev *smmute_mdev = mdev_get_drvdata(mdev);
	struct device *parent = mdev_parent_dev(mdev);

	if (smmute_mdev->msi_trigger) {
		int irq = pci_irq_vector(to_pci_dev(parent),
					 smmute_mdev->msi_vector);

		smmute_mdev_clear_msi(smmute_mdev);

		if (!WARN_ON(irq < 0))
			free_irq(irq, smmute_mdev);
		eventfd_ctx_put(smmute_mdev->msi_trigger);
		smmute_mdev->msi_trigger = NULL;
	}
}

static int smmute_mdev_create(struct kobject *kobj, struct mdev_device *mdev)
{
	struct device *dev = mdev_parent_dev(mdev);
	struct smmute_mdev *smmute_mdev;
	struct smmute_device *smmute;
	int ret;

	if (!dev_is_pci(dev))
		return -ENODEV;

	smmute = to_smmute_device(dev);
	if (WARN_ON(!smmute))
		return -ENODEV;

	smmute_mdev = kzalloc(sizeof(*smmute), GFP_KERNEL);
	if (!smmute_mdev)
		return -ENOMEM;

	mutex_lock(&smmute->mdev.mutex);
	/* Allocate a page of 512 frames for this mdev */
	ret = ida_simple_get(&smmute->mdev.pages, smmute->mdev.pages_first,
			    smmute->mdev.pages_max, GFP_KERNEL);
	if (ret < 0) {
		mutex_unlock(&smmute->mdev.mutex);
		goto err_free;
	}
	smmute_mdev->page_nr = ret;
	smmute->mdev.pages_free--;
	mutex_unlock(&smmute->mdev.mutex);

	smmute_mdev->smmute = smmute;
	smmute_mdev->mdev = mdev;

	/* Use the MSI vector corresponding to this page number */
	smmute_mdev->msi_vector = smmute->nr_msix_entries +
		(smmute_mdev->page_nr - smmute->mdev.pages_first);

	mdev_set_drvdata(mdev, smmute_mdev);

	smmute_mdev_clear_pasid(mdev, 0);
	smmute_mdev_clear_msi(smmute_mdev);

	dev_info(dev, "created mdev %s\n", dev_name(mdev_dev(mdev)));
	dev_info(dev, " with page %zu frames 0x%zx-0x%zx MSI %lu\n",
		 smmute_mdev->page_nr, smmute_mdev->page_nr *
		 SMMUTE_FRAMES_PER_PAGE, (smmute_mdev->page_nr + 1) *
		 SMMUTE_FRAMES_PER_PAGE - 1, smmute_mdev->msi_vector);

	return 0;

err_free:
	kfree(smmute_mdev);
	return ret;
}

static int smmute_mdev_remove(struct mdev_device *mdev)
{
	struct device *dev = mdev_parent_dev(mdev);
	struct smmute_mdev *smmute_mdev = mdev_get_drvdata(mdev);
	struct smmute_device *smmute = to_smmute_device(dev);

	dev_info(dev, "removing mdev %s\n", dev_name(mdev_dev(mdev)));

	mutex_lock(&smmute->mdev.mutex);
	ida_simple_remove(&smmute->mdev.pages, smmute_mdev->page_nr);
	smmute->mdev.pages_free++;
	mutex_unlock(&smmute->mdev.mutex);

	kfree(smmute_mdev);

	return 0;
}

static ssize_t name_show(struct kobject *kobj, struct device *dev, char *buf)
{
	return sprintf(buf, "SMMU Test Engine platform\n");
}
MDEV_TYPE_ATTR_RO(name);

static ssize_t available_instances_show(struct kobject *kobj,
					struct device *dev, char *buf)
{
	struct smmute_device *smmute = to_smmute_device(dev);

	return sprintf(buf, "%zu\n", smmute->mdev.pages_free);
}
MDEV_TYPE_ATTR_RO(available_instances);

static ssize_t device_api_show(struct kobject *kobj, struct device *dev,
			       char *buf)
{
	return sprintf(buf, "%s\n", VFIO_DEVICE_API_PLATFORM_STRING);
}
MDEV_TYPE_ATTR_RO(device_api);

static struct attribute *smmute_mdev_types_attrs[] = {
	&mdev_type_attr_name.attr,
	&mdev_type_attr_device_api.attr,
	&mdev_type_attr_available_instances.attr,
	NULL,
};

static struct attribute_group smmute_mdev_type_group = {
	.name	= "platform",
	.attrs	= smmute_mdev_types_attrs,
};

static struct attribute_group *smmute_mdev_type_groups[] = {
	&smmute_mdev_type_group,
	NULL,
};

static struct mdev_parent_ops smmute_mdev_fops = {
	.owner			= THIS_MODULE,
	.supported_type_groups	= smmute_mdev_type_groups,
	.create			= smmute_mdev_create,
	.remove			= smmute_mdev_remove,
	.open			= smmute_mdev_open,
	.release		= smmute_mdev_close,
	.ioctl			= smmute_mdev_ioctl,
	.mmap			= smmute_mdev_mmap,
	.set_pasid		= smmute_mdev_set_pasid,
	.clear_pasid		= smmute_mdev_clear_pasid,
};

/**
 * smmute_mdev_add - Add mediated smmute devices.
 * @count: number of mediated devices
 */
int smmute_mdev_add(struct smmute_device *smmute, size_t count)
{
	int ret;

	ret = mdev_register_device(smmute->dev, &smmute_mdev_fops);
	if (ret)
		pr_err("mdev_register failed: %d\n", ret);

	smmute->mdev.pages_first = smmute->nr_pairs - count;
	smmute->mdev.pages_max = smmute->nr_pairs;
	smmute->mdev.pages_free = count;
	ida_init(&smmute->mdev.pages);
	mutex_init(&smmute->mdev.mutex);
	return ret;
}
