// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for the SMMUv3 test engine
 *
 * Copyright (C) 2016 ARM Limited
 */

//#define DEBUG
//#define DEBUG_USER_FRAMES

#include <linux/cdev.h>
#include <linux/delay.h>
#include <linux/dma-iommu.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/iommu.h>
#include <linux/irq.h>
#include <linux/kobject.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/pagemap.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/printk.h>
#include <linux/rbtree.h>
#include <linux/sched/mm.h>
#include <linux/splice.h>
#include <linux/uaccess.h>

#include <linux/smmu-test-engine.h>

#define CREATE_TRACE_POINTS
#include <trace/events/smmute.h>

#define SMMUTE_MAX_MSIS		8

static void smmute_dma_release(struct kobject *kobj);
static void smmute_dma_put(struct smmute_dma *dma);
struct kobj_type smmute_dma_ktype = {
	.release = smmute_dma_release,
};

static void smmute_task_release(struct kobject *);
static struct kobj_type smmute_task_ktype = {
	.release	= smmute_task_release,
};

static void smmute_transaction_release(struct kobject *kobj);
struct kobj_type smmute_transaction_ktype = {
	.release = smmute_transaction_release,
};

static void smmute_fd_release(struct kobject *kobj);
struct kobj_type smmute_file_desc_ktype = {
	.release		= smmute_fd_release,
};

static int smmute_major;
static DEFINE_IDA(smmute_minor_ida);
static struct class *smmute_class;
static struct cdev smmute_cdev;

static DEFINE_MUTEX(smmute_devices_mutex);
static LIST_HEAD(smmute_devices);

static atomic64_t smmute_transactions_ida = ATOMIC_INIT(0);
static atomic64_t smmute_dma_ida = ATOMIC_INIT(0);

static const char *smmute_get_command_name(enum smmute_cmd command)
{
	switch (command) {
	case ENGINE_FRAME_MISCONFIGURED:
		return "ENGINE_FRAME_MISCONFIGURED";
	case ENGINE_ERROR:
		return "ENGINE_ERROR";
	case ENGINE_NO_FRAME:
		return "ENGINE_NO_FRAME";
	case ENGINE_HALTED:
		return "ENGINE_HALTED";
	case ENGINE_MEMCPY:
		return "ENGINE_MEMCPY";
	case ENGINE_SUM64:
		return "ENGINE_SUM64";
	case ENGINE_RAND48:
		return "ENGINE_RAND48";
	default:
		return "UNKNOWN";
	}
}

static enum smmute_cmd smmute_ioctl_to_command(long cmd)
{
	switch (cmd) {
	case SMMUTE_IOCTL_MEMCPY:
		return ENGINE_MEMCPY;
	case SMMUTE_IOCTL_RAND48:
		return ENGINE_RAND48;
	case SMMUTE_IOCTL_SUM64:
		return ENGINE_SUM64;
	default:
		pr_err("unknown command %ld\n", cmd);
		return ENGINE_ERROR;
	}
}

__attribute__((unused))
static void smmute_uframe_dump(size_t idx, struct smmute_uframe *frame)
{
	size_t i;

	pr_info("--------- User frame #%05zu ----------\n"
		" cmd                   = %s\n"
		" uctrl                 = 0x%x\n"
		" count_launch          = %u\n"
		" count_ret             = %u\n"
		" msi addr              = 0x%llx\n"
		" msi data              = 0x%x\n"
		" msi attr              = 0x%x\n"
		" attr                  = 0x%x\n"
		" seed                  = 0x%x\n"
		" begin                 = 0x%llx\n"
		" end                   = 0x%llx\n"
		" stride                = 0x%llu\n"
		" user data             =\n",
		idx,
		smmute_get_command_name(readl_relaxed(&frame->cmd)),
		readl_relaxed(&frame->uctrl),
		readl_relaxed(&frame->count_of_transactions_launched),
		readl_relaxed(&frame->count_of_transactions_returned),
		readq_relaxed(&frame->msiaddress),
		readl_relaxed(&frame->msidata),
		readl_relaxed(&frame->msiattr),
		readl_relaxed(&frame->attributes),
		readl_relaxed(&frame->seed),
		readq_relaxed(&frame->begin),
		readq_relaxed(&frame->end_incl),
		readq_relaxed(&frame->stride));

	for (i = 0; i < 4; i++)
		pr_info("  0x%016llx %016llx\n",
			readq_relaxed(frame->udata + i),
			readq(frame->udata + i + 1));
	pr_info("--------------------------------------\n");
}

static void smmute_transaction_set_state(struct smmute_transaction *transaction,
					 enum smmute_transaction_state state);

#define smmute_get_msi_vector(smmute, idx)					\
	(dev_is_pci((smmute)->dev) ? (smmute)->msix_entries[idx].vector :	\
	 (smmute)->plat_msi_entries[idx].vector)

static irqreturn_t smmute_msi_handler(int irq, void *opaque)
{
	u32 hwstate;
	struct smmute_msi_pool *pool = opaque;
	struct smmute_transaction *tsac, *next;

	spin_lock(&pool->lock);
	/* Signal all finished transactions */
	list_for_each_entry_safe(tsac, next, &pool->transactions, msi_head) {
		hwstate = readl_relaxed(&tsac->uframe->cmd);
		if (atomic_read(&tsac->state) == TRANSACTION_INFLIGHT &&
		    hwstate != tsac->command) {
			list_del_init(&tsac->msi_head);
			smmute_transaction_set_state(tsac, TRANSACTION_NOTIFIED);
			wake_up_interruptible(&tsac->fd->transaction_wait);
		}
	}
	spin_unlock(&pool->lock);

	return IRQ_HANDLED;
}

/**
 * smmute_msi_alloc - allocate an MSI for a transaction
 *
 * return the allocated MSI number (>= 0), or an error
 */
static int smmute_msi_alloc(struct smmute_device *smmute,
			     struct smmute_transaction *tsac)
{
	struct smmute_msi_pool *pool;
	unsigned long nr_msis = smmute->nr_msix_entries;

	mutex_lock(&smmute->resources_mutex);
	pool = &smmute->msi_pools[smmute->current_pool];

	spin_lock_irq(&pool->lock);
	list_add_tail(&tsac->msi_head, &pool->transactions);
	spin_unlock_irq(&pool->lock);

	tsac->msi = smmute->current_pool;
	smmute->current_pool = (smmute->current_pool + 1) % nr_msis;
	mutex_unlock(&smmute->resources_mutex);

	return tsac->msi;
}

static bool smmute_msi_free(struct smmute_device *smmute,
			    struct smmute_transaction *tsac)
{
	bool deleted = false;
	struct smmute_msi_pool *pool = &smmute->msi_pools[tsac->msi];

	spin_lock_irq(&pool->lock);
	/* Clean up if the MSI handler didn't already */
	if (!list_empty(&tsac->msi_head)) {
		deleted = true;
		list_del_init(&tsac->msi_head);
	}
	spin_unlock_irq(&pool->lock);

	return deleted;
}

/**
 * smmute_frame_alloc - reserve a pair of frames for a transaction
 */
static long smmute_frame_alloc(struct smmute_device *smmute)
{
	long frame;
	unsigned long nr_frames = smmute->nr_pairs * SMMUTE_FRAMES_PER_PAGE;

	mutex_lock(&smmute->resources_mutex);

	frame = find_first_zero_bit(smmute->reserved_frames, nr_frames);
	if (frame == nr_frames) {
		dev_err(smmute->dev, "no frame available\n");
		mutex_unlock(&smmute->resources_mutex);
		return -EBUSY;
	}
	set_bit(frame, smmute->reserved_frames);

	mutex_unlock(&smmute->resources_mutex);

	return frame;
}

/**
 * smmute_transaction_alloc - Allocate resources to perform one transaction
 *
 * Allocates
 * - a transaction struct, linked to the device
 * - one MSI, and request the associated IRQ if necessary
 * - one pair of frames
 *
 * Caller must intialize everything else:
 * - set command, size, stride, seed, attr
 * - attach DMA (one or two regions, depending on the command)
 * - launch the transaction
 */
static struct smmute_transaction *
smmute_transaction_alloc(struct smmute_file_desc *fd)
{
	int ret;
	long msi, frame;
	struct smmute_transaction *transaction;
	struct smmute_device *smmute = fd->smmute;

	transaction = kmem_cache_alloc(smmute->transaction_cache,
				       GFP_KERNEL | __GFP_ZERO);
	if (!transaction)
		return ERR_PTR(-ENOMEM);

	frame = smmute_frame_alloc(smmute);
	if (frame < 0) {
		ret = frame;
		goto err_free_transaction;
	}

	atomic_set(&transaction->state, TRANSACTION_INVALID);

	transaction->id = atomic64_inc_return(&smmute_transactions_ida);
	transaction->frame = frame;
	transaction->fd = fd;
	transaction->uframe = smmute_user_frame(smmute->pairs, frame);

	msi = smmute_msi_alloc(smmute, transaction);
	if (msi < 0) {
		ret = msi;
		goto err_release_frame;
	}

	ret = kobject_init_and_add(&transaction->kobj, &smmute_transaction_ktype,
				   &fd->kobj, "%llu", transaction->id);
	if (ret) {
		dev_err(smmute->dev, "could not create kobject\n");
		goto err_release_msi;
	}

	mutex_lock(&fd->transactions_mutex);
	list_add(&transaction->list, &fd->transactions);
	mutex_unlock(&fd->transactions_mutex);

	dev_dbg(smmute->dev, "allocated transaction %p\n", transaction);

	return transaction;

err_release_msi:
	smmute_msi_free(smmute, transaction);

err_release_frame:
	clear_bit(frame, smmute->reserved_frames);

err_free_transaction:
	kmem_cache_free(smmute->transaction_cache, transaction);

	return ERR_PTR(ret);
}

/* Called by kobject_cleanup */
static void smmute_transaction_release(struct kobject *kobj)
{
	struct smmute_transaction *transaction = container_of(kobj,
			struct smmute_transaction, kobj);
	struct smmute_file_desc *fd = transaction->fd;
	struct smmute_device *smmute = fd->smmute;

	smmute_transaction_set_state(transaction, TRANSACTION_INVALID);

	smmute_dma_put(transaction->dma_in);
	smmute_dma_put(transaction->dma_out);

	smmute_msi_free(smmute, transaction);
	clear_bit(transaction->frame, smmute->reserved_frames);

	list_del(&transaction->list);

	dev_dbg(smmute->dev, "freed transaction     %p\n", transaction);

	kmem_cache_free(smmute->transaction_cache, transaction);
}

static void smmute_transaction_free(struct smmute_file_desc *fd,
				    struct smmute_transaction *transaction)
{
	mutex_lock(&fd->transactions_mutex);
	kobject_put(&transaction->kobj);
	mutex_unlock(&fd->transactions_mutex);
}

/* Not upstream yet */
#define iommu_sva_device_init(...)		(-ENOSYS)
#define iommu_sva_device_shutdown(...)
#define iommu_register_mm_exit_handler(...)	(-ENOSYS)
#define iommu_unregister_mm_exit_handler(...)
#define iommu_sva_bind_device(...)		(-ENOSYS)
#define iommu_sva_unbind_device(...)		(-ENOSYS)

__maybe_unused
static int smmute_mm_exit_handler(struct device *dev, int pasid, void *data)
{
	/* data is smmu_task passed to bind. */
	dev_dbg(dev, "PASID %d exit\n", pasid);

	return 0;
}

static int smmute_task_get(struct smmute_file_desc *fd,
			   struct smmute_task **out_task)
{
	int ret = 0;
	int ssid;
	struct pid *current_pid = get_task_pid(current, PIDTYPE_PID);
	struct smmute_device *smmute = fd->smmute;
	struct smmute_task *smmute_task;
	struct mm_struct *mm;

	pid_t pidnr;

	if (!smmute->sva)
		return -ENODEV;

	mm = get_task_mm(current);
	if (!mm)
		return -EINVAL;

	mutex_lock(&fd->task_mutex);

	list_for_each_entry(smmute_task, &fd->tasks, fd_head) {
		if (smmute_task->pid == current_pid) {
			kobject_get(&smmute_task->kobj);
			goto out_unlock;
		}
	}

	smmute_task = kzalloc(sizeof(*smmute_task), GFP_KERNEL);
	if (!smmute_task) {
		ret = -ENOMEM;
		goto out_unlock;
	}

	ret = iommu_sva_bind_device(smmute->dev, mm, &ssid, IOMMU_SVA_FEAT_IOPF,
				    smmute_task);
	if (ret) {
		kfree(smmute_task);
		smmute_task = NULL;
		goto out_unlock;
	}

	smmute_task->smmute	= smmute;
	smmute_task->ssid	= ssid;
	smmute_task->pid	= get_pid(current_pid);
	smmute_task->fd		= fd;

	smmute_task->kobj.kset	= smmute->tasks;

	pidnr = pid_vnr(smmute_task->pid);

	ret = kobject_init_and_add(&smmute_task->kobj, &smmute_task_ktype, NULL,
				   "%u", pidnr);
	if (ret) {
		put_pid(current_pid);
		smmute_task_release(&smmute_task->kobj);
		smmute_task = NULL;
	} else {
		list_add(&smmute_task->fd_head, &fd->tasks);
	}

out_unlock:
	mutex_unlock(&fd->task_mutex);

	mmput(mm);

	if (out_task)
		*out_task = smmute_task;

	put_pid(current_pid);

	return ret;
}

void smmute_task_put_all(struct smmute_file_desc *fd)
{
	struct smmute_task *smmute_task, *next;

	mutex_lock(&fd->task_mutex);

	list_for_each_entry_safe(smmute_task, next, &fd->tasks, fd_head) {
		kobject_put(&smmute_task->kobj);
	}

	mutex_unlock(&fd->task_mutex);
}

/*
 * Put reference of the task passed as argument if non-null, otherwise put the
 * reference to the current task.
 */
void smmute_task_put(struct smmute_file_desc *fd, struct smmute_task *smmute_task)
{
	mutex_lock(&fd->task_mutex);

	if (smmute_task) {
		kobject_put(&smmute_task->kobj);
	} else {
		struct pid *current_pid = get_task_pid(current, PIDTYPE_PID);

		list_for_each_entry(smmute_task, &fd->tasks, fd_head) {
			if (smmute_task->pid != current_pid)
				continue;

			kobject_put(&smmute_task->kobj);
			break;
		}

		put_pid(current_pid);
	}

	mutex_unlock(&fd->task_mutex);
}

void smmute_task_release(struct kobject *kobj)
{
	int ret;
	struct smmute_task *smmute_task = container_of(kobj, struct smmute_task, kobj);

	ret = iommu_sva_unbind_device(smmute_task->smmute->dev, smmute_task->ssid);
	if (ret)
		pr_warn("iommu_sva_unbind_dev failed with %d\n", ret);

	/* We're holding fd->task_mutex (see smmute_task_put) */
	list_del(&smmute_task->fd_head);

	put_pid(smmute_task->pid);
	kfree(smmute_task);
}

static struct smmute_dma *
smmute_dma_alloc_struct(struct smmute_file_desc *fd)
{
	int ret;
	struct smmute_dma *dma;
	struct smmute_device *smmute = fd->smmute;

	dma = kmem_cache_alloc(smmute->dma_regions_cache,
			       GFP_KERNEL | __GFP_ZERO);
	if (!dma)
		return NULL;

	dma->id = atomic64_inc_return(&smmute_dma_ida);
	dma->smmute = smmute;
	dma->kobj.kset = fd->dma_regions;

	ret = kobject_init_and_add(&dma->kobj, &smmute_dma_ktype, NULL, "%llu",
				   dma->id);
	if (ret) {
		kmem_cache_free(smmute->dma_regions_cache, dma);
		return NULL;
	}

	return dma;
}

static void smmute_transaction_attach_dma(struct smmute_transaction *transaction,
					  struct smmute_dma *dma,
					  int direction)
{
	switch (direction) {
	case DMA_TO_DEVICE:
		transaction->dma_in = dma;
		break;
	case DMA_FROM_DEVICE:
		transaction->dma_out = dma;
		break;
	default:
		BUG();
	}

	/*
	 * Non-SVM:
	 * - mmap allocates smmute_dma, takes ref
	 * - attach_dma takes ref
	 * - transaction_release drops ref
	 * - munmap drops ref, frees smmute_dma
	 *
	 * SVM:
	 * - user_dma_get allocates smmute_dma, takes ref
	 * - transaction_release drops ref, frees smmute_dma
	 *
	 * Not very nice, I know, but it's the simplest
	 */
	if (!dma->task)
		kobject_get(&dma->kobj);
}

/* Called by kobject_cleanup */
static void smmute_dma_release(struct kobject *kobj)
{
	struct smmute_dma *dma = container_of(kobj, struct smmute_dma, kobj);
	struct smmute_device *smmute = dma->smmute;

	if (dma->task)
		smmute_task_put(dma->task->fd, dma->task);

	if (dma->type & SMMUTE_DMA_KERNEL) {
		iommu_dma_unmap_page(smmute->dev, dma->iova, dma->size,
				     DMA_TO_DEVICE, 0);

		if (dma->kernel.do_put_pages) {
			int i;
			for (i = 0; i < dma->kernel.nr_pages; i++) {
				put_page(dma->kernel.pages[i]);
			}
		}

		if (dma->kernel.pages)
			kvfree(dma->kernel.pages);

	} else if (dma->type & SMMUTE_DMA_USER) {
		dma_free_attrs(smmute->dev, dma->size, dma->user.kaddr,
			       dma->iova, 0);
	}

	kmem_cache_free(smmute->dma_regions_cache, dma);
}

static void smmute_dma_put(struct smmute_dma *dma)
{
	if (dma)
		kobject_put(&dma->kobj);
}

/**
 * smmute_dma_get_user - find DMA region mmap'd in current address space
 *
 * @fd: file descriptor used when mapping that region
 * @addr: user pointer inside a DMA region
 * @size: size of the buffer
 * @off: if the region is found, this will contain the offset between @addr and
 *       the beginning of the region
 * @sva: use userspace pointer for device DMA
 */
static struct smmute_dma *
smmute_dma_get_user(struct smmute_file_desc *fd, void __user *addr, size_t size,
		    off_t *off, bool sva)
{
	int ret;
	struct smmute_dma *dma;
	struct smmute_task *smmute_task;
	struct smmute_dma *found_dma = NULL;

	struct mm_struct *mm = current->mm;

	if (sva) {
		/* Allocate if necessary and get a ref to the smmute_task */
		ret = smmute_task_get(fd, &smmute_task);
		if (ret) {
			dev_err(fd->smmute->dev, "unable to get task\n");
			return NULL;
		}

		dma = smmute_dma_alloc_struct(fd);
		if (!dma)
			return NULL;

		dma->iova = (unsigned long)addr;
		dma->size = size;
		dma->task = smmute_task;

		return dma;
	}

	mutex_lock(&fd->user_dma_mutex);
	list_for_each_entry(dma, &fd->user_dma, user.list) {
		if (dma->user.mm != mm)
			continue;

		if (addr >= dma->user.uaddr && addr < (dma->user.uaddr +
					dma->size)) {
			found_dma = dma;
			*off = addr - dma->user.uaddr;
			break;
		}
	}
	mutex_unlock(&fd->user_dma_mutex);

	return found_dma;
}

static struct smmute_file_desc *smmute_fd_alloc(struct smmute_device *smmute)
{
	int ret;
	struct smmute_file_desc *fd;

	fd = kmem_cache_alloc(smmute->file_desc_cache, GFP_KERNEL | __GFP_ZERO);
	if (!fd)
		return NULL;

	fd->id = atomic64_inc_return(&smmute->files_ida);

	mutex_init(&fd->user_dma_mutex);
	INIT_LIST_HEAD(&fd->user_dma);
	INIT_LIST_HEAD(&fd->transactions);
	mutex_init(&fd->pending_pages_mutex);
	INIT_LIST_HEAD(&fd->pending_pages);
	mutex_init(&fd->task_mutex);

	init_waitqueue_head(&fd->transaction_wait);

	fd->kobj.kset = smmute->files;

	ret = kobject_init_and_add(&fd->kobj, &smmute_file_desc_ktype, NULL,
				   "%llu", fd->id);
	if (ret)
		goto err_free_fd;

	mutex_init(&fd->transactions_mutex);

	fd->smmute = smmute;

	fd->dma_regions = kset_create_and_add("dma", NULL, &fd->kobj);
	if (!fd->dma_regions)
		goto err_release;

	return fd;

err_release:
	kobject_put(&fd->kobj);

err_free_fd:
	kmem_cache_free(smmute->file_desc_cache, fd);

	return NULL;
}

void smmute_fd_release(struct kobject *kobj)
{
	struct smmute_file_desc *fd = container_of(kobj, struct smmute_file_desc, kobj);
	struct smmute_device *smmute = fd->smmute;

	mutex_destroy(&fd->transactions_mutex);

	kmem_cache_free(smmute->file_desc_cache, fd);
}

/**
 * smmute_user_frame_init - Initialise a user frame
 *
 * With informations taken from a transaction and its parameters, initialise a
 * user frame in the PCI device.
 *
 * Returns a pointer to that frame on success, an error pointer on failure.
 */
static struct smmute_uframe *
smmute_user_frame_init(struct smmute_device *smmute,
		       struct smmute_transaction *transaction)
{
	size_t i = 0;
	struct smmute_uframe *frame = transaction->uframe;
	dma_addr_t iova_in = transaction->dma_in->iova + transaction->offset_in;
	dma_addr_t iova_out = 0;

	if (transaction->dma_out)
		iova_out = transaction->dma_out->iova + transaction->offset_out;

	if (transaction->flags & SMMUTE_FLAG_FAULT) {
		iova_in = ~iova_in;
		iova_out = ~iova_out;
	}

	/*
	 * cmd must be ENGINE_HALTED, ENGINE_ERROR or
	 * ENGINE_FRAME_MISCONFIGURED for the rest of the structure to be
	 * writeable
	 */
	writel_relaxed(ENGINE_HALTED, &frame->cmd);

	/* Get dma region associated with this virtual address */
	if (!transaction->dma_in)
		return ERR_PTR(-EINVAL);

	writel_relaxed(0, &frame->uctrl);
	writeq_relaxed(iova_in, &frame->begin);
	writeq_relaxed(iova_in + transaction->size - 1, &frame->end_incl);
	writel_relaxed(transaction->attr, &frame->attributes);
	writel_relaxed(transaction->seed, &frame->seed);
	if (!transaction->stride)
		transaction->stride = 1;
	writeq_relaxed(transaction->stride, &frame->stride);

	if (dev_is_pci(smmute->dev)) {
		writeq_relaxed(1, &frame->msiaddress);
		writel_relaxed(transaction->msi, &frame->msidata);
		writel_relaxed(0, &frame->msiattr);
	} else {
		struct smmute_msi_info *entry =
			&smmute->plat_msi_entries[transaction->msi];
		writeq_relaxed(entry->doorbell, &frame->msiaddress);
		writel_relaxed(entry->data, &frame->msidata);
		writel_relaxed(SMMUTE_ATTR_DEVICE, &frame->msiattr);
	}

	if (transaction->dma_out) {
		writeq_relaxed(iova_out, frame->udata);
		/* Skip first udata */
		i = 1;
	}

	for (; i < 8; i++)
		writeq_relaxed(0, frame->udata + i);

	return frame;
}

/**
 * smmute_priv_frame_init - initialise privileged frame for a transaction
 */
static struct smmute_pframe *
smmute_priv_frame_init(struct smmute_device *smmute,
		       struct smmute_transaction *transaction)
{
	u32 ssid = 0, sid = 0;
	u32 pctrl = SMMUTE_PCTRL_NS;
	struct smmute_pframe *frame;
	struct iommu_fwspec *fwspec = smmute->dev->iommu_fwspec;

	if (transaction->dma_in->task) {
		ssid = transaction->dma_in->task->ssid;
	}

	if (transaction->dma_out && transaction->dma_out->task) {
		BUG_ON(ssid != transaction->dma_out->task->ssid);
	}

	frame = smmute_privileged_frame(smmute->pairs, transaction->frame);

	if (dev_is_pci(smmute->dev) && to_pci_dev(smmute->dev)->ats_enabled)
		pctrl |= SMMUTE_PCTRL_ATS_EN;

	/*
	 * For a platform device, retrieve the stream ID. As this ID isn't
	 * virtualizable, we can't assign the platform device to a guest
	 */
	if (fwspec && fwspec->num_ids)
		sid = fwspec->ids[0];

	writel_relaxed(pctrl, &frame->pctrl);
	writel_relaxed(0, &frame->downstream_port_index);
	writel_relaxed(sid, &frame->streamid); /* Ignored for PCI */
	writel_relaxed(ssid ? ssid : SMMUTE_NO_SUBSTREAMID, &frame->substreamid);

	return frame;
}

static const char *smmute_transaction_state_name(enum smmute_transaction_state state)
{
	switch (state) {
	case TRANSACTION_READY:
		return "READY";
	case TRANSACTION_REGISTERED:
		return "REGISTERED";
	case TRANSACTION_INFLIGHT:
		return "INFLIGHT";
	case TRANSACTION_NOTIFIED:
		return "NOTIFIED";
	case TRANSACTION_FINISHED:
		return "FINISHED";
	case TRANSACTION_INVALID:
		return "INVALID";
	}

	return "???";
}

static void smmute_transaction_set_state(struct smmute_transaction *transaction,
					 enum smmute_transaction_state state)
{
	enum smmute_transaction_state prev_state;
	enum smmute_transaction_state expect = TRANSACTION_INVALID;

	switch (state) {
	case TRANSACTION_READY:
		expect = TRANSACTION_INVALID;
		trace_smmute_transaction_ready(transaction);
		break;

	case TRANSACTION_REGISTERED:
		expect = TRANSACTION_READY;
		break;

	case TRANSACTION_INFLIGHT:
		expect = TRANSACTION_REGISTERED;
		trace_smmute_transaction_launch(transaction);
		break;

	case TRANSACTION_NOTIFIED:
		expect = TRANSACTION_INFLIGHT | TRANSACTION_FINISHED;
		trace_smmute_transaction_notify(transaction);
		break;

	case TRANSACTION_FINISHED:
		expect = TRANSACTION_NOTIFIED | TRANSACTION_FINISHED |
			 TRANSACTION_INFLIGHT;
		trace_smmute_transaction_finish(transaction);
		break;

	case TRANSACTION_INVALID:
		/*
		 * Don't WARN when unregistering a transaction that didn't
		 * succeed. INVALID can be reached from any state.
		 */
		expect = -1U;
		trace_smmute_transaction_retire(transaction);
		break;
	}

	prev_state = atomic_xchg(&transaction->state, state);

	WARN(!(prev_state & expect),
	     "Transaction %llu state was %s (%x), expected %x, new %s (%x)",
	     transaction->id, smmute_transaction_state_name(prev_state),
	     prev_state, expect, smmute_transaction_state_name(state), state);
}

/**
 * smmute_transaction_launch - start a transaction
 *
 * It is the caller's responsibility to keep track of the transaction and query
 * its status periodically. Once the transaction finished, it will (hopefully)
 * trigger an MSI, which will set the 'finished' status
 *
 * If start is false, only fill the engine frame, but don't write the command.
 *
 * Return 0 when the transaction was successfully launched.
 */
static int smmute_transaction_launch(struct smmute_device *smmute,
				     struct smmute_transaction *transaction,
				     bool start)
{
	struct smmute_uframe *user_frame;
	struct smmute_pframe *priv_frame;

	smmute_transaction_set_state(transaction, TRANSACTION_READY);

	priv_frame = smmute_priv_frame_init(smmute, transaction);
	if (IS_ERR(priv_frame)) {
		dev_dbg(smmute->dev, "init_priv_frame\n");
		return PTR_ERR(priv_frame);
	}

	user_frame = smmute_user_frame_init(smmute, transaction);
	if (IS_ERR(user_frame)) {
		dev_dbg(smmute->dev, "init_user_frame\n");
		return PTR_ERR(user_frame);
	}

	smmute_transaction_set_state(transaction, TRANSACTION_REGISTERED);
	if (!start)
		return 0;

	/*
	 * Start the workload. Assume frame is mapped with Dev-nGnRE
	 * attributes, through pci_iomap.
	 */
	smmute_transaction_set_state(transaction, TRANSACTION_INFLIGHT);
	writel_relaxed(transaction->command, &user_frame->cmd);

	if (readl_relaxed(&user_frame->cmd) != transaction->command) {
		/*
		 * If write "failed", no MSI will be generated. Set 'finished' and
		 * run away. result_get will handle the mess.
		 */
		smmute_transaction_set_state(transaction, TRANSACTION_FINISHED);
	}

	return 0;
}

/**
 * smmute_result_get - get transaction result
 *
 * When blocking is not set, sleep and wait for the transaction to finish.
 * Fill 'result' with status and resulting value (in case of a SUM op).
 *
 * return -EAGAIN if the transaction is not finished and 'blocking' is false
 * return 0 on success, which means that the transaction can be freed.
 */
static int smmute_result_get(struct smmute_file_desc *fd,
			     struct smmute_transaction *transaction,
			     struct smmute_transaction_result *result,
			     bool blocking)
{
	int ret;
	u32 status;
	struct smmute_uframe *frame;
	struct smmute_device *smmute = fd->smmute;

retry_wait:
	if (blocking) {
		/*
		 * Set a timeout to check periodically if the transaction
		 * finished without generating an MSI (due to a broken MSI
		 * setup, most likely.)
		 */
		ret = wait_event_interruptible_timeout(fd->transaction_wait,
				atomic_read(&transaction->state) != TRANSACTION_INFLIGHT,
				SMMUTE_POLL_DELAY);

		if (ret == -ERESTARTSYS) {
			/* task interrupted by a signal */
			return ret;
		}
	}

	frame = transaction->uframe;
	status = readl_relaxed(&frame->cmd);
	if (status == transaction->command) {
		/* Transaction is still running */
		if (blocking)
			goto retry_wait;
		else
			return -EAGAIN;
	}

	smmute_transaction_set_state(transaction, TRANSACTION_FINISHED);

	result->value = 0;

	switch (status) {
	case ENGINE_HALTED:
		result->status = 0;
		result->value = readq_relaxed(frame->udata + 1);
		break;
	case ENGINE_ERROR:
		result->status = EIO;
		result->value = readq_relaxed(frame->udata + 2);
		break;
	case ENGINE_FRAME_MISCONFIGURED:
		result->status = EINVAL;
		break;
	default:
		result->status = EFAULT;
		break;
	}

	/*
	 * There is a small chance of getting false positives here. If the MSI
	 * is masked in the MSI-X table (being serviced by the handler), then
	 * the TestEngine sets MSI_ABORTED. I could observe this when MSIs were
	 * handed in a thread.
	 */
	if (readl_relaxed(&frame->uctrl) & SMMUTE_UCTRL_MSI_ABORTED)
		dev_warn(smmute->dev, "MSI aborted\n");

#ifdef DEBUG_USER_FRAMES
	smmute_uframe_dump(transaction->frame, frame);
#endif

	return 0;
}

/**
 * __smmute_result_get_bulk - get all transaction results for a given file
 *
 * Merge results into the 'final' struct.
 *
 * If 'pos' is not NULL, start iteration there instead of the root.
 * In case of error, return the failed transaction in *pos, allowing the caller
 * to inspect it further, and then call us again
 *
 * If one transaction failed, stop merging and return -EINTR, with that
 * transaction in 'pos' and its status inside 'final'. It allows the caller to
 * signal any error before calling this function again. The partial result is
 * kept.
 *
 * Return -EAGAIN if a transaction is still running, and that transaction in
 * 'pos'. 'final' will contain a partial result.
 *
 * Return 0 on success. In that case, *pos is NULL
 */
static int __smmute_result_get_bulk(struct smmute_file_desc *fd,
				    struct smmute_transaction_result *final,
				    struct smmute_transaction **pos)
{
	int ret = 0;
	int command = -1;
	struct smmute_transaction *transaction, *n;
	struct smmute_transaction_result result;
	struct device *dev = fd->smmute->dev;

	BUG_ON(!pos);

	mutex_lock(&fd->transactions_mutex);

	if (*pos)
		transaction = *pos;
	else
		transaction = list_first_entry(&fd->transactions,
					struct smmute_transaction, list);

	list_for_each_entry_safe_from(transaction, n, &fd->transactions, list) {
		if (command != -1 && command != transaction->command) {
			dev_warn(dev, "mismatched transaction type\n");
			continue;
		}

		command = transaction->command;

		*pos = transaction;

		ret = smmute_result_get(fd, transaction, &result,
					final->blocking);
		if (ret)
			break;

		final->status = result.status;

		if (result.status != 0) {
			/*
			 * TODO: result.value contains the faulting address.
			 * Find a way to pass this information back to the
			 * caller. Can't use final->value since it might trash a
			 * partial result. I'd rather introduce an additional
			 * result.fault_addr field.
			 */
			ret = -EINTR;
			break;
		}

		if (command == ENGINE_SUM64)
			final->value += result.value;

		if (!final->keep) {
			/* transaction_free without taking the lock */
			kobject_put(&transaction->kobj);
			*pos = NULL;
		}
	}

	mutex_unlock(&fd->transactions_mutex);

	return ret;
}

/**
 * smmute_result_get_bulk - collect all transactions of a given file
 *
 * Wait for all transactions to finish; only report failures with printk.
 * This function will sleep if something is running.
 *
 * Transactions will only be freed if final->keep is false
 *
 * return 0 when all transactions associated to this file have been queried
 * return -EAGAIN if a transaction is running and final->blocking is false.
 *        'final' will contain a partial result.
 */
static int smmute_result_get_bulk(struct smmute_file_desc *fd,
				  struct smmute_transaction_result *final)
{
	int ret;
	struct smmute_transaction *partial = NULL;
	struct smmute_transaction *tmp;

	final->value = 0;
	final->status = 0;

	while (true) {
		ret = __smmute_result_get_bulk(fd, final, &partial);
		if (ret != -EINTR)
			break;

		dev_err(fd->smmute->dev, "transaction %llu failed with %d\n",
			partial->id, final->status);
		tmp = partial;
		partial = list_next_entry(partial, list);
		smmute_transaction_free(fd, tmp);

		/*
		 * Continue iteration from 'partial'. If end of list is
		 * reached, __smmute_result_get_bulk returns with the
		 * result.
		 */
	}

	return ret;
}

void smmute_vm_close(struct vm_area_struct *vma)
{
	struct smmute_file_desc *fd;
	struct file *file = vma->vm_file;
	struct smmute_dma *dma = vma->vm_private_data;

	BUG_ON(!file);

	fd = file->private_data;

	BUG_ON(!fd);
	BUG_ON(!dma);

	mutex_lock(&fd->user_dma_mutex);
	list_del(&dma->user.list);
	mutex_unlock(&fd->user_dma_mutex);

	smmute_dma_put(dma);
}


struct vm_operations_struct smmute_vm_ops = {
	.close		= smmute_vm_close,
};

/**
 * smmute_open - allocate the resources needed by a file descriptor
 */
static int smmute_open(struct inode *inode, struct file *file)
{
	struct smmute_device *smmute;
	struct smmute_file_desc *fd;

	mutex_lock(&smmute_devices_mutex);
	list_for_each_entry(smmute, &smmute_devices, list) {
		if (smmute->minor == iminor(inode))
			break;
	}
	mutex_unlock(&smmute_devices_mutex);

	if (unlikely(&smmute->list == &smmute_devices)) {
		/* device not found */
		return -ENOENT;
	}

	fd = smmute_fd_alloc(smmute);
	if (!fd)
		return -ENOMEM;

	fd->file = file;
	file->private_data = fd;
	INIT_LIST_HEAD(&fd->tasks);

	return 0;
}

static int smmute_mmap(struct file *file, struct vm_area_struct *vma)
{
	int ret;
	void *kaddr;
	struct smmute_dma *dma;
	struct smmute_file_desc *fd = file->private_data;
	struct device *dev = fd->smmute->dev;
	size_t size = vma->vm_end - vma->vm_start;

	dma = smmute_dma_alloc_struct(fd);
	if (!dma)
		return -ENOMEM;

	/* TODO: Argh! How do we specify prot flags without re-implementing the
	 * whole lot? It currently just assumes RW (DMA_BIDIRECTIONAL) */
	kaddr = dma_alloc_attrs(dev, size, &dma->iova, GFP_USER, 0);
	if (!kaddr) {
		ret = -ENOMEM;
		goto err_free_struct_dma;
	}

	dma->size = size;
	dma->user.kaddr = kaddr;
	dma->user.uaddr = (void *)vma->vm_start;
	dma->user.mm = vma->vm_mm;

	ret = dma_mmap_attrs(dev, vma, kaddr, dma->iova, size, 0);
	if (ret)
		goto err_free_attrs;

	dma->type = SMMUTE_DMA_USER;

	vma->vm_private_data = dma;
	vma->vm_ops = &smmute_vm_ops;

	mutex_lock(&fd->user_dma_mutex);
	list_add(&dma->user.list, &fd->user_dma);
	mutex_unlock(&fd->user_dma_mutex);

	return 0;

err_free_attrs:
	dma_free_attrs(dev, size, kaddr, dma->iova, 0);

err_free_struct_dma:
	smmute_dma_put(dma);

	return ret;
}

/*
 * smmute_dma_map_frame - map PCI config space with the iommu
 */
static struct smmute_dma *smmute_dma_map_frame(struct smmute_file_desc *fd,
					     size_t frame_idx)
{
	int ret;
	size_t size = sizeof(struct smmute_uframe);
	struct smmute_dma *dma;
	phys_addr_t phys_base, phys;
	dma_addr_t iova;

	struct smmute_device *smmute = fd->smmute;
	struct device *dev = smmute->dev;

	if (!dev_is_pci(dev))
		return NULL; /* TODO: platform */

	dma = smmute_dma_alloc_struct(fd);
	if (!dma)
		return NULL;

	/* Find out physical address of BAR0 */
	phys_base = pci_resource_start(to_pci_dev(dev), 0);

	/* Physical address of user frame */
	phys = (phys_addr_t)smmute_user_frame(phys_base, frame_idx);
	if (phys > pci_resource_end(to_pci_dev(dev), 0)) {
		dev_err(dev, "frame %zu is out of bounds\n", frame_idx);
		ret = -EFAULT;
		goto err_free;
	}

	iova = dma_map_resource(dev, phys, size, DMA_FROM_DEVICE, 0);
	if (dma_mapping_error(dev, iova)) {
		dev_err(dev, "mapping error\n");
		goto err_free;
	}

	dma->iova = iova;
	dma->size = size;

	return dma;

err_free:
	smmute_dma_put(dma);
	return ERR_PTR(ret);
}

/**
 * smmute_p2p_prepare - prepare secondary transaction
 *
 * Allocate a secondary transaction, connect transactions 1 and 2 by creating a
 * DMA mapping of one frame and using it as output region for the other.
 */
static int smmute_p2p_prepare(struct smmute_file_desc *fd,
			      struct smmute_transaction *transaction_1,
			      struct smmute_p2p_params *params)
{
	int ret;
	enum smmute_cmd *command_1, *command_2;
	struct smmute_dma *dma;
	struct smmute_transaction *transaction_2;

	transaction_2 = smmute_transaction_alloc(fd);
	if (!transaction_2)
		return -ENOMEM;

	dma = smmute_dma_get_user(fd, (void *)params->secondary.input_start,
				  params->secondary.size,
				  &transaction_2->offset_in,
				  params->secondary.flags & SMMUTE_FLAG_SVA);
	if (!dma) {
		ret = -ESRCH;
		goto err_free_transaction;
	}

	smmute_transaction_attach_dma(transaction_2, dma, DMA_TO_DEVICE);

	/*
	 * Note that for user mem, we're stuck here. There is no simple way to
	 * create mappings of physical stuff into userspace, so this will fail.
	 * Would be good to create a transaction "read unprivileged, write
	 * privileged"
	 *
	 * drive-by idea dump: vm_iomap_memory
	 */
	dma = smmute_dma_map_frame(fd, transaction_2->frame);
	if (IS_ERR(dma)) {
		dev_err(fd->smmute->dev, "failed to map frame %zu\n",
			transaction_2->frame);
		ret = PTR_ERR(dma);
		goto err_free_transaction;
	}
	smmute_transaction_attach_dma(transaction_1, dma, DMA_FROM_DEVICE);

	dma = smmute_dma_map_frame(fd, transaction_1->frame);
	if (IS_ERR(dma)) {
		dev_err(fd->smmute->dev, "failed to map frame %zu\n",
			transaction_1->frame);
		ret = PTR_ERR(dma);
		goto err_free_transaction;
	}
	smmute_transaction_attach_dma(transaction_2, dma, DMA_FROM_DEVICE);

	transaction_2->stride = params->secondary.stride;
	transaction_2->seed = params->secondary.seed;
	transaction_2->attr = params->secondary.attr;
	transaction_2->size = params->secondary.size;

	if (params->primary.size > 8) {
		/* Command must be written in one 64-bit access */
		dev_err(fd->smmute->dev, "erroneous size %u\n",
			params->primary.size);
		ret = -EINVAL;
		goto err_free_transaction;
	}

	/*
	 * Register the transaction in the engine (fill the frame) but don't
	 * start it.
	 */
	ret = smmute_transaction_launch(fd->smmute, transaction_2, false);
	if (ret) {
		dev_err(fd->smmute->dev, "init transaction 2 failed\n");
		goto err_free_transaction;
	}

	if (params->secondary.flags & SMMUTE_FLAG_SVA) {
		command_1 = (void *)transaction_1->dma_in->iova +
			    transaction_1->offset_in;
		command_2 = (void *)transaction_2->dma_in->iova +
			    transaction_2->offset_in;

		put_user(ENGINE_MEMCPY, command_1);
		put_user(smmute_ioctl_to_command(params->command), command_2);
	} else {
		command_1 = transaction_1->dma_in->user.kaddr + transaction_1->offset_in;
		command_2 = transaction_2->dma_in->user.kaddr + transaction_2->offset_in;

		*command_1 = ENGINE_MEMCPY;
		*command_2 = smmute_ioctl_to_command(params->command);
	}


	/* for get_result, to match against the current frame status */
	transaction_2->command = ENGINE_MEMCPY;

	/* fake state change to avoid surprising the IRQ thread */
	smmute_transaction_set_state(transaction_2, TRANSACTION_INFLIGHT);

	/* pass secondary ID back to user */
	params->secondary.transaction_id = transaction_2->id;

	return 0;

err_free_transaction:
	smmute_transaction_free(fd, transaction_2);

	return ret;
}

static long smmute_transaction_ioctl(struct smmute_file_desc *fd,
				     unsigned int cmd, void *argp)
{
	long ret;
	size_t size;
	struct smmute_dma *dma;
	union smmute_transaction_params params;
	struct smmute_transaction *transaction;
	union smmute_transaction_params __user *up = argp;

	switch (cmd) {
	case SMMUTE_IOCTL_MEMCPY:
		size = sizeof(params.memcpy);
		break;
	case SMMUTE_IOCTL_P2P:
		size = sizeof(params.p2p);
		break;
	default:
		size = sizeof(params.common);
	}

	ret = copy_from_user(&params, up, size);
	if (ret)
		return -EFAULT;

	if (params.common.flags & ~SMMUTE_FLAG_MASK)
		return -EINVAL;

	transaction = smmute_transaction_alloc(fd);
	if (IS_ERR(transaction))
		return PTR_ERR(transaction);

	dma = smmute_dma_get_user(fd, (void *)params.common.input_start,
				  params.common.size, &transaction->offset_in,
				  params.common.flags & SMMUTE_FLAG_SVA);
	if (!dma) {
		ret = -ESRCH;
		goto err_free_transaction;
	}

	smmute_transaction_attach_dma(transaction, dma, DMA_TO_DEVICE);

	transaction->stride = params.common.stride;
	transaction->seed = params.common.seed;
	transaction->attr = params.common.attr;
	transaction->size = params.common.size;
	transaction->flags = params.common.flags;

	switch (cmd) {
	case SMMUTE_IOCTL_MEMCPY:
		transaction->command = ENGINE_MEMCPY;

		dma = smmute_dma_get_user(fd,
					  (void *)params.memcpy.output_start,
					  params.common.size,
					  &transaction->offset_out,
					  params.common.flags & SMMUTE_FLAG_SVA);
		if (!dma) {
			ret = -ESRCH;
			break;
		}

		smmute_transaction_attach_dma(transaction, dma, DMA_FROM_DEVICE);
		break;
	case SMMUTE_IOCTL_SUM64:
		transaction->command = ENGINE_SUM64;
		break;
	case SMMUTE_IOCTL_RAND48:
		transaction->command = ENGINE_RAND48;
		break;
	case SMMUTE_IOCTL_P2P:
		transaction->command = ENGINE_MEMCPY;

		if (params.p2p.secondary.flags & ~SMMUTE_FLAG_MASK)
			return -EINVAL;

		ret = smmute_p2p_prepare(fd, transaction, &params.p2p);
		if (ret)
			goto err_free_transaction;
		break;
	default:
		ret = -EINVAL;
	}

	if (ret)
		goto err_free_transaction;

	ret = smmute_transaction_launch(fd->smmute, transaction, true);
	if (ret)
		goto err_free_transaction;

	params.common.transaction_id = transaction->id;
	ret = copy_to_user(up, &params, size);
	if (ret) {
		/*
		 * Transaction is launched, but we can't inform the user. Let's
		 * forget about it and let smmute_release deal with the mess.
		 */
		ret = -EFAULT;
	}

	return 0;

err_free_transaction:
	smmute_transaction_free(fd, transaction);

	return ret;
}

static long smmute_result_ioctl(struct smmute_file_desc *fd, void *argp)
{
	long ret;
	struct smmute_transaction *transaction;
	struct smmute_transaction_result result;

	ret = copy_from_user(&result, argp, sizeof(result));
	if (ret)
		return -EFAULT;

	ret = -ENOENT;
	mutex_lock(&fd->transactions_mutex);
	list_for_each_entry(transaction, &fd->transactions, list) {
		if (transaction->id == result.transaction_id) {
			ret = 0;
			break;
		}
	}
	mutex_unlock(&fd->transactions_mutex);
	if (ret)
		return ret;

	/* Will sleep if result.blocking is true */
	ret = smmute_result_get(fd, transaction, &result, result.blocking);
	if (ret)
		return ret;

	if (!result.keep)
		smmute_transaction_free(fd, transaction);

	ret = copy_to_user(argp, &result, sizeof(result));
	if (ret)
		return -EFAULT;

	return 0;
}

static long smmute_check_version(struct smmute_file_desc *fd, void __user *argp)
{
	int ret;
	struct smmute_version version;

	ret = copy_from_user(&version, argp, sizeof(version));
	if (ret)
		return -EFAULT;

	if (version.major > SMMUTE_VERSION_MAJOR ||
	    (version.major == SMMUTE_VERSION_MAJOR &&
	     version.minor > SMMUTE_VERSION_MINOR)) {
		dev_dbg(fd->smmute->dev,
			"user version %u.%u incompatible with our %u.%u\n",
			version.major, version.minor, SMMUTE_VERSION_MAJOR,
			SMMUTE_VERSION_MINOR);
		return -ENODEV;
	}

	version.major = SMMUTE_VERSION_MAJOR;
	version.minor = SMMUTE_VERSION_MINOR;

	ret = copy_to_user(argp, &version, sizeof(version));
	if (ret)
		return -EFAULT;

	return 0;
}

static long smmute_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void *argp = (void __user *)arg;
	struct smmute_file_desc *fd = file->private_data;

	switch (cmd) {
	case SMMUTE_IOCTL_VERSION:
		return smmute_check_version(fd, argp);
	case SMMUTE_IOCTL_GET_RESULT:
		return smmute_result_ioctl(fd, argp);
	case SMMUTE_IOCTL_BIND_TASK:
		return smmute_task_get(fd, NULL);
	case SMMUTE_IOCTL_UNBIND_TASK:
		smmute_task_put(fd, NULL);
		return 0;
	default:
		return smmute_transaction_ioctl(fd, cmd, argp);
	}
}

/*
 * Generic buffer ops are sufficient for the moment, we'll refine them as we go.
 */
static const struct pipe_buf_operations smmute_pipe_buf_ops = {
	.can_merge = 0,
	.confirm = generic_pipe_buf_confirm,
	.release = generic_pipe_buf_release,
	.steal = generic_pipe_buf_steal,
	.get = generic_pipe_buf_get,
};

static void smmute_spd_release_page(struct splice_pipe_desc *spd, unsigned int i)
{
	put_page(spd->pages[i]);
}

/**
 * smmute_splice_dma_collect
 *
 * Collect pages prepared by a previous call to splice_write, gather them into
 * one big iova-contiguous region.
 */
static int smmute_splice_dma_collect(struct smmute_file_desc *fd,
				     struct smmute_dma *dma)
{
	int ret;
	size_t i;
	size_t page_nr = 0;
	ssize_t collected_size = 0;
	struct smmute_dma_page *dma_page, *n;
	struct sg_table sgt;

	dma->kernel.pages = kzalloc(sizeof(struct page *) * dma->kernel.nr_pages,
				    GFP_KERNEL);

	mutex_lock(&fd->pending_pages_mutex);
	list_for_each_entry_safe(dma_page, n, &fd->pending_pages, list) {
		if (collected_size + dma_page->size > dma->size)
			/* won't fit in pipe */
			continue;

		collected_size += dma_page->size;
		dma->kernel.pages[page_nr++] = dma_page->page;

		list_del(&dma_page->list);
		kfree(dma_page);

		if (collected_size >= dma->size - (PAGE_SIZE - 1))
			/* can't stuff any more page down the pipe */
			break;
	}
	mutex_unlock(&fd->pending_pages_mutex);

	if (!collected_size) {
		dev_err(fd->smmute->dev, "nothing collected\n");
		ret = -EAGAIN;
		goto err_free_pages;
	}

	/*
	 * Shrink pages to what we actually have. No need to shrink the page
	 * pointer array, it won't waste too much space anyway.
	 */
	dma->kernel.nr_pages = page_nr;
	dma->size = collected_size;

	ret = sg_alloc_table_from_pages(&sgt, dma->kernel.pages,
			dma->kernel.nr_pages, 0, dma->size, GFP_KERNEL);
	if (ret)
		goto err_release_pages;

	ret = dma_map_sg_attrs(fd->smmute->dev, sgt.sgl, sgt.orig_nents,
			       DMA_FROM_DEVICE, 0);
	if (ret == 0) {
		ret = -EFAULT;
		goto err_free_table;
	}

	/* TODO: check if the IOVA space is actually contiguous. */
	dma->iova = sg_dma_address(sgt.sgl);

	sg_free_table(&sgt);

	return 0;

err_free_table:
	sg_free_table(&sgt);

err_release_pages:
	for (i = 0; i < dma->kernel.nr_pages; i++) {
		put_page(dma->kernel.pages[i]);
	}

err_free_pages:
	kvfree(dma->kernel.pages);

	return ret;
}

/**
 * smmute_splice_dma_alloc - allocate and map DMA for a splice op
 *
 * 'dma' must have members kernel.nr_pages and size set.
 */
static int smmute_splice_dma_alloc(struct smmute_file_desc *fd,
				   struct smmute_dma *dma)

{
	size_t page_nr;
	struct page **pages;

	pages = iommu_dma_alloc(fd->smmute->dev, dma->size, GFP_USER, 0,
			IOMMU_WRITE | IOMMU_CACHE, &dma->iova, NULL);
	if (!pages)
		return -ENOMEM;


	/* Inspect what we got from the IOMMU DMA allocator (could have
	 * returned high-order pages) */
	for (page_nr = 0; page_nr < dma->kernel.nr_pages; page_nr++) {
		if (!pages[page_nr])
			break;
	}

	/* FIXME: We can't use compound pages at the moment, since the
	 * pipe works on PAGE_SIZE (but I want to see it die
	 * first, and then think about making the pipe flexible.) */
	BUG_ON(page_nr != dma->kernel.nr_pages);

	dma->kernel.pages = pages;
	dma->kernel.nr_pages = page_nr;

	return 0;
}

/**
 * smmute_splice_read - fill a pipe with data produced by smmute
 *
 * If we work as a devpipe, collect existing pages and build one big DMA
 * region. Launch a MEMCPY transaction from this region to a new one.
 * Otherwise, allocate a new region, and launch a RAND
 */
static ssize_t smmute_splice_read(struct file *file, loff_t *ppos,
				  struct pipe_inode_info *pipe, size_t len,
				  unsigned int flags)
{
	ssize_t ret;
	size_t i;
	size_t off, nr_pages, buf_len, partial_len;
	unsigned int buffers = pipe->buffers;
	unsigned long attr = SMMUTE_ATTR_WBRAWA_SH;

	struct smmute_dma *dma_in, *dma_out = NULL;
	struct smmute_transaction *transaction;
	struct smmute_transaction_result result;
	struct partial_page *partial;
	struct splice_pipe_desc spd = {
		.ops = &smmute_pipe_buf_ops,
		.spd_release = &smmute_spd_release_page,
		.nr_pages_max = buffers,
	};
	struct smmute_file_desc *fd = file->private_data;
	struct device *dev = fd->smmute->dev;
	struct iommu_domain *domain = iommu_get_domain_for_dev(dev);

	bool is_devpipe = flags & SPLICE_F_MOVE; // XXX HACK

	if (!domain)
		return -EINVAL;

	off = *ppos & ~PAGE_MASK;
	/* Limit to the number of buffers allocated by the pipe */
	buf_len = min_t(size_t, buffers * PAGE_SIZE,
			PAGE_ALIGN(len));
	/* Maximum number of pages required to complete this transaction */
	nr_pages = buf_len >> PAGE_SHIFT;

	/* Cap transaction size to buffer size */
	len = min_t(size_t, len, buf_len);

	transaction = smmute_transaction_alloc(fd);
	if (IS_ERR(transaction))
		return PTR_ERR(transaction);

	transaction->seed	= 0;
	/* TODO: transaction offset (DMA is always page aligned) */
	transaction->size	= len;
	transaction->stride	= 1;
	transaction->attr	= SMMUTE_TRANSACTION_ATTR(attr, attr);

	dma_in = smmute_dma_alloc_struct(fd);
	if (!dma_in) {
		ret = -ENOMEM;
		goto out_free_transaction;
	}

	dma_in->size = buf_len;
	dma_in->kernel.nr_pages = nr_pages;
	if (is_devpipe)
		ret = smmute_splice_dma_collect(fd, dma_in);
	else
		ret = smmute_splice_dma_alloc(fd, dma_in);

	if (ret)
		goto out_free_transaction;

	dma_in->type = SMMUTE_DMA_KERNEL;

	smmute_transaction_attach_dma(transaction, dma_in, DMA_TO_DEVICE);

	if (is_devpipe) {
		/* Reserve an output region for DMA */
		dma_out = smmute_dma_alloc_struct(fd);
		if (!dma_out) {
			ret = -ENOMEM;
			goto out_free_transaction;
		}

		dma_out->size = dma_in->size;
		dma_out->kernel.nr_pages = dma_in->kernel.nr_pages;

		ret = smmute_splice_dma_alloc(fd, dma_out);
		if (ret)
			goto out_free_transaction;

		dma_out->type = SMMUTE_DMA_KERNEL;

		smmute_transaction_attach_dma(transaction, dma_out,
					      DMA_FROM_DEVICE);

		spd.nr_pages = dma_out->kernel.nr_pages;
		spd.pages = dma_out->kernel.pages;

		transaction->command = ENGINE_MEMCPY;
	} else {
		spd.nr_pages = dma_in->kernel.nr_pages;
		spd.pages = dma_in->kernel.pages;

		transaction->command = ENGINE_RAND48;
	}

	ret = smmute_transaction_launch(fd->smmute, transaction, true);
	if (ret) {
		dev_err(dev, "launch_transaction failed\n");
		goto out_free_transaction;
	}

	/* Sleep until transaction finishes */
	ret = smmute_result_get(fd, transaction, &result, true);
	if (ret) {
		dev_err(dev, "get_result failed\n");
		goto out_free_transaction;
	} else if (result.status != 0) {
		dev_err(dev, "bad result: %d\n", result.status);
		goto out_free_transaction;
	}

	partial = kzalloc(sizeof(struct partial_page) * spd.nr_pages, GFP_KERNEL);
	if (!partial) {
		ret = -ENOMEM;
		goto out_free_transaction;
	}

	spd.partial = partial;
	partial_len = len;

	dev_dbg(dev, "splice_read(loff=%lld, sz=0x%zx/0x%zx, %zu pages, f=%x)\n",
			*ppos, len, buf_len, nr_pages, flags);

	for (i = 0; i < spd.nr_pages; i++) {
		size_t this_len = min_t(size_t, partial_len,
					PAGE_SIZE - off);

		partial[i].offset = off;
		partial[i].len = this_len;

		partial_len -= this_len;
		off = 0;
	}

	ret = splice_to_pipe(pipe, &spd);
	if (ret > 0)
		*ppos += ret;
	/* TODO: otherwise, keep pages around for next run... */


	kfree(partial);

out_free_transaction:
	smmute_transaction_free(fd, transaction);

	smmute_dma_put(dma_in);
	smmute_dma_put(dma_out);

	return ret;
}

/**
 * pipe_to_devpipe - store buffer page for later
 */
static int pipe_to_devpipe(struct pipe_inode_info *pipe,
			   struct pipe_buffer *buf, struct splice_desc *sd)
{
	struct file *file = sd->u.file;
	struct smmute_file_desc *fd = file->private_data;

	struct smmute_dma_page *dma_page;

	dma_page = kzalloc(sizeof(struct smmute_dma_page), GFP_KERNEL);
	if (!dma_page)
		return -ENOMEM;

	/*
	 * Increase page refcount, allowing it to survive until
	 * smmute_splice_dma_collect takes over
	 */
	get_page(buf->page);

	dma_page->page = buf->page;
	dma_page->size = PAGE_SIZE;

	mutex_lock(&fd->pending_pages_mutex);
	list_add(&dma_page->list, &fd->pending_pages);
	mutex_unlock(&fd->pending_pages_mutex);

	return sd->len;
}

/**
 * pipe_to_smmute - launch a sum transaction on a buffer page
 */
static int pipe_to_smmute(struct pipe_inode_info *pipe,
			  struct pipe_buffer *buf, struct splice_desc *sd)
{
	int ret;
	size_t aligned_len;
	unsigned long attr = SMMUTE_ATTR_WBRAWA_SH;
	dma_addr_t dma_addr;
	struct smmute_dma *dma;
	struct smmute_transaction *transaction;
	struct file *file = sd->u.file;
	struct smmute_file_desc *fd = file->private_data;
	struct device *dev = fd->smmute->dev;

	/*
	 * Map DMA to read buffer. The test engine requires a size aligned on 8
	 * bytes for SUM64 commands.
	 */
	aligned_len = ALIGN(sd->len, 8);
	dma_addr = iommu_dma_map_page(dev, buf->page, 0, aligned_len,
				      IOMMU_CACHE | IOMMU_READ);
	if (dma_mapping_error(dev, dma_addr))
		return -EFAULT;

	dma = smmute_dma_alloc_struct(fd);
	if (!dma) {
		ret = -ENOMEM;
		goto err_unmap_page;
	}

	dma->type = SMMUTE_DMA_KERNEL;

	/* Until we implement merging, this is a single page mapping */
	dma->kernel.pages = kmalloc(sizeof(struct page *), GFP_KERNEL);
	if (!dma->kernel.pages) {
		ret = -ENOMEM;
		goto err_release_struct_dma;
	}

	dma->kernel.pages[0] = buf->page;
	/* TODO: use buf->len and buf->offset */
	dma->kernel.nr_pages = 1;
	dma->size = PAGE_SIZE;
	dma->iova = dma_addr;

	/*
	 * Take a reference to the page. When we return from this function, the
	 * page will be released once by buf->ops->release, and the final
	 * release will be done after the transaction finished
	 */
	get_page(buf->page);

	dma->kernel.do_put_pages = 1;

	/* TODO: merge transactions */
	transaction = smmute_transaction_alloc(fd);
	if (IS_ERR(transaction)) {
		ret = PTR_ERR(transaction);
		goto err_release_struct_dma;
	}

	transaction->command	= ENGINE_SUM64;
	transaction->seed	= 0;
	transaction->size	= aligned_len;
	transaction->stride	= 1;
	transaction->attr	= SMMUTE_TRANSACTION_ATTR(attr, attr);

	smmute_transaction_attach_dma(transaction, dma, DMA_TO_DEVICE);

	ret = smmute_transaction_launch(fd->smmute, transaction, true);
	if (ret) {
		dev_info(dev, "failed to launch transaction\n");
		goto err_free_transaction;
	}

	/*
	 * Drop one reference, but dma will be released when collecting the
	 * transaction.
	 */
	smmute_dma_put(dma);

	return sd->len;

err_free_transaction:
	smmute_transaction_free(fd, transaction);

err_release_struct_dma:
	smmute_dma_put(dma);

	return ret;

err_unmap_page:
	iommu_dma_unmap_page(dev, dma_addr, sd->len, DMA_TO_DEVICE, 0);

	return ret;
}

/**
 * smmute_splice_write - fetch pages from pipe
 */
static ssize_t smmute_splice_write(struct pipe_inode_info *pipe,
				   struct file *out, loff_t *ppos, size_t len,
				   unsigned int flags)
{
	int ret;
	ssize_t count;
	struct smmute_file_desc *fd = out->private_data;
	struct device *dev = fd->smmute->dev;
	struct iommu_domain *domain = iommu_get_domain_for_dev(dev);
	struct smmute_transaction_result result = {
		.blocking = true,
		.keep = false,
	};
	bool is_devpipe = flags & SPLICE_F_MOVE; // XXX hack

	if (!domain)
		return -EINVAL;

	dev_dbg(dev, "splice_write(loff=%lld, sz=0x%zx, f=%x)\n",
		*ppos, len, flags);

	/* In devpipe mode, we store the pages for next splice_read call */
	if (is_devpipe)
		return splice_from_pipe(pipe, out, ppos, len, flags,
					pipe_to_devpipe);

	count = splice_from_pipe(pipe, out, ppos, len, flags, pipe_to_smmute);
	if (count <= 0)
		/* We might lose a bunch of transactions here. They will be
		 * collected when we close the file. */
		return count;

	/*
	 * Wait for all transactions. This could be non-blocking if
	 * SPLICE_F_NONBLOCK is set, but we will need to store partial results.
	 */
	ret = smmute_result_get_bulk(fd, &result);

	if (ret) {
		dev_err(dev, "bulk result failed with %d\n", ret);
		return ret;
	} else if (result.status) {
		ret = result.status;
		dev_err(dev, "transaction error %d\n", ret);
		return result.status;
	}

	/* TODO: store this and let user collect it with an ioctl */
	/* pr_info("success: val = %llx\n", result.value); */

	return count;
}

/**
 * smmute_release - release a file descriptor
 *
 * Free all transactions associated to it. We *have* to wait until they
 * finished, otherwise we might end up with the device sending spurious MSIs
 * and writing to unmapped pages, which is much more critical than zombifying a
 * process.
 *
 * FIXME: are we allowed to sleep here? What happens when we return an error?
 *        What happens when we loop indefinitely?
 */
static int smmute_release(struct inode *inode, struct file *file)
{
	int ret;
	struct smmute_transaction_result result = {
		.keep = 0,
		.blocking = 1,
	};
	struct smmute_dma_page *dma_page, *np;

	struct smmute_file_desc *fd = file->private_data;
	struct device *dev = fd->smmute->dev;

	/* Collect garbage */
	ret = smmute_result_get_bulk(fd, &result);
	if (ret)
		dev_err(dev, "release: get_bulk returned %d\n", ret);

	mutex_lock(&fd->pending_pages_mutex);
	list_for_each_entry_safe(dma_page, np, &fd->pending_pages, list) {
		dev_warn(dev, "releasing orphan page %p\n", dma_page->page);

		put_page(dma_page->page);
		list_del(&dma_page->list);
		kfree(dma_page);
	}
	mutex_unlock(&fd->pending_pages_mutex);

	smmute_task_put_all(fd);

	kobject_put(&fd->kobj);
	kset_unregister(fd->dma_regions);

	return 0;
}

static const struct file_operations smmute_fops = {
	.unlocked_ioctl		= smmute_ioctl,
	.mmap			= smmute_mmap,
	.open			= smmute_open,
	.release		= smmute_release,

	.splice_read		= smmute_splice_read,
	.splice_write		= smmute_splice_write,
};

static int smmute_pci_msi_enable(struct pci_dev *pdev)
{
	int ret, i;
	int nr_msis;
	struct smmute_device *smmute;
	struct msix_entry *entries;

	smmute = pci_get_drvdata(pdev);
	if (!smmute)
		return -EINVAL;

	dev_dbg(&pdev->dev, "max number of MSI-X vectors: %d\n",
			pci_msix_vec_count(pdev));

	nr_msis = min_t(size_t, SMMUTE_MAX_MSIS,
			smmute->nr_pairs * SMMUTE_FRAMES_PER_PAGE);
	entries = devm_kmalloc(&pdev->dev, sizeof(struct msix_entry) * nr_msis,
			       GFP_KERNEL);

	if (!entries) {
		dev_err(&pdev->dev, "could not allocate MSI-X entries\n");
		return -ENOMEM;
	}

	smmute->msix_entries = entries;

	for (i = 0; i < nr_msis; i++)
		entries[i].entry = i;

	ret = pci_enable_msix_range(pdev, entries, 1, nr_msis);
	if (ret <= 0) {
		devm_kfree(&pdev->dev, entries);
		return ret;
	}

	smmute->nr_msix_entries = ret;
	dev_dbg(&pdev->dev, "requested %d MSIs, got %d\n", nr_msis, ret);

	return 0;
}

static int smmute_init_msi_pool(struct smmute_device *smmute)
{
	int vec, ret, i;
	struct smmute_msi_pool *pool;
	int nr_pools = smmute->nr_msix_entries;

	smmute->msi_pools = devm_kcalloc(smmute->dev, nr_pools, sizeof(*pool),
					 GFP_KERNEL);
	if (!smmute->msi_pools)
		return -ENOMEM;

	for (i = 0; i < nr_pools; i++) {
		pool = &smmute->msi_pools[i];
		spin_lock_init(&pool->lock);
		INIT_LIST_HEAD(&pool->transactions);

		vec = smmute_get_msi_vector(smmute, i);
		ret = request_irq(vec, smmute_msi_handler, 0,
				  dev_name(smmute->dev), pool);
		if (ret)
			break;
	}

	return ret;
}

static void smmute_free_msi_pool(struct smmute_device *smmute)
{
	int i;

	/*
	 * Other resources are managed (freed automatically), but we don't use
	 * devm for MSIs, because they have to be unregistered before MSIs are
	 * freed by pci_disable_msix.
	 */
	for (i = 0; i < smmute->nr_msix_entries; i++)
		free_irq(smmute_get_msi_vector(smmute, i),
			 &smmute->msi_pools[i]);
}

static int smmute_common_probe(struct smmute_device *smmute)
{
	int minor;
	size_t nr_frames;
	int ret = -ENOMEM;
	int cache_flags = 0;
	struct device *dev = smmute->dev;

#ifdef DEBUG
	/* prevents merging caches, allows to get stats from /proc/slabinfo */
	cache_flags = SLAB_POISON | SLAB_CONSISTENCY_CHECKS;
#endif

	mutex_init(&smmute->resources_mutex);

	nr_frames = smmute->nr_pairs * SMMUTE_FRAMES_PER_PAGE;
	smmute->reserved_frames = devm_kzalloc(dev, BITS_TO_LONGS(nr_frames),
					       GFP_KERNEL);
	if (!smmute->reserved_frames)
		return ret;

	smmute->transaction_cache = kmem_cache_create("smmute_transactions",
			sizeof(struct smmute_transaction), 0, cache_flags, NULL);
	if (!smmute->transaction_cache)
		goto err_free_frames;

	smmute->dma_regions_cache = kmem_cache_create("smmute_dma_regions",
			sizeof(struct smmute_dma), 0, cache_flags, NULL);
	if (!smmute->dma_regions_cache)
		goto err_destroy_transaction_cache;

	smmute->file_desc_cache = kmem_cache_create("smmute_file_descs",
			sizeof(struct smmute_file_desc), 0, cache_flags, NULL);
	if (!smmute->file_desc_cache)
		goto err_destroy_dma_cache;

	minor = ida_simple_get(&smmute_minor_ida, 0, SMMUTE_MAX_DEVICES,
			       GFP_KERNEL);
	if (minor < 0) {
		dev_dbg(dev, "idr_alloc failed with %d\n", minor);
		goto err_destroy_fd_cache;
	}

	smmute->chrdev = device_create(smmute_class, dev,
			MKDEV(smmute_major, minor), smmute,
			"smmute%d", minor);
	if (IS_ERR(smmute->chrdev)) {
		dev_err(dev, "unable to create char dev (%d, %d)\n",
			smmute_major, minor);
		ret = PTR_ERR(smmute->chrdev);
		goto err_free_minor;
	}
	smmute->minor = minor;

	atomic64_set(&smmute->files_ida, 0);
	smmute->files = kset_create_and_add("files", NULL, &smmute->chrdev->kobj);
	if (!smmute->files)
		goto err_device_destroy;

	smmute->tasks = kset_create_and_add("tasks", NULL, &smmute->chrdev->kobj);
	if (!smmute->tasks)
		goto err_release_files;

	ret = iommu_sva_device_init(dev, IOMMU_SVA_FEAT_IOPF, 0,
				    smmute_mm_exit_handler);
	if (ret)
		dev_warn(dev, "failed to initialize SVA (%d)\n", ret);
	else
		smmute->sva = true;

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret) {
		dev_warn(dev, "failed to set requested DMA mask\n");
		/* 32-bit is the default anyway. This is useless. */
		ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
		if (ret) {
			dev_err(dev, "failed to set DMA mask\n");
			goto err_release_tasks;
		}
	}

	ret = smmute_init_msi_pool(smmute);
	if (ret)
		goto err_release_tasks;

	mutex_lock(&smmute_devices_mutex);
	list_add(&smmute->list, &smmute_devices);
	mutex_unlock(&smmute_devices_mutex);

	dev_info(dev, "has %zux2 pages of %zu frames\n", smmute->nr_pairs,
		 SMMUTE_FRAMES_PER_PAGE);

	/* TODO: self-test */

	return 0;

err_release_tasks:
	kset_unregister(smmute->tasks);
err_release_files:
	kset_unregister(smmute->files);
err_device_destroy:
	device_destroy(smmute_class, MKDEV(smmute_major, smmute->minor));
err_free_minor:
	ida_simple_remove(&smmute_minor_ida, minor);
err_destroy_fd_cache:
	kmem_cache_destroy(smmute->file_desc_cache);
err_destroy_dma_cache:
	kmem_cache_destroy(smmute->dma_regions_cache);
err_destroy_transaction_cache:
	kmem_cache_destroy(smmute->transaction_cache);
err_free_frames:
	devm_kfree(smmute->dev, smmute->reserved_frames);

	return ret;
}

static void smmute_common_remove(struct smmute_device *smmute)
{
	mutex_lock(&smmute_devices_mutex);
	list_del(&smmute->list);
	mutex_unlock(&smmute_devices_mutex);

	if (smmute->sva)
		iommu_sva_device_shutdown(smmute->dev);

	smmute_free_msi_pool(smmute);

	kset_unregister(smmute->tasks);
	kset_unregister(smmute->files);

	device_destroy(smmute_class, MKDEV(smmute_major, smmute->minor));

	ida_simple_remove(&smmute_minor_ida, smmute->minor);

	kmem_cache_destroy(smmute->transaction_cache);
	kmem_cache_destroy(smmute->dma_regions_cache);
	kmem_cache_destroy(smmute->file_desc_cache);

	devm_kfree(smmute->dev, smmute->reserved_frames);
}

static int smmute_pci_probe(struct pci_dev *pdev, const struct pci_device_id *devid)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct smmute_device *smmute;

	smmute = devm_kzalloc(dev, sizeof(struct smmute_device), GFP_KERNEL);
	if (!smmute) {
		dev_err(dev, "failed to allocate smmute device");
		return -ENOMEM;
	}

	pci_set_drvdata(pdev, smmute);
	smmute->dev = &pdev->dev;

	ret = pci_enable_device(pdev);
	if (ret) {
		dev_err(dev, "failed to enable device\n");
		goto err_free_device;
	}

	ret = pci_request_regions(pdev, DRV_NAME);
	if (ret) {
		dev_err(dev, "failed to obtain resources\n");
		goto err_disable_device;
	}

	smmute->pairs = pci_iomap(pdev, 0, 0);
	if (!smmute->pairs) {
		dev_err(&pdev->dev, "pci_iomap failed");
		goto err_release_regions;
	}

	smmute->nr_pairs = pci_resource_len(pdev, 0) / sizeof(*smmute->pairs);

	pci_set_master(pdev);

	ret = smmute_pci_msi_enable(pdev);
	if (ret)
		goto err_unmap_pairs;

	ret = smmute_common_probe(smmute);
	if (ret)
		goto err_disable_msi;

	return 0;

err_disable_msi:
	pci_disable_msix(pdev);
	devm_kfree(dev, smmute->msix_entries);
err_unmap_pairs:
	pci_iounmap(pdev, smmute->pairs);
err_release_regions:
	pci_release_regions(pdev);
err_disable_device:
	pci_disable_device(pdev);
err_free_device:
	devm_kfree(dev, smmute);

	return ret;
}

static void smmute_pci_remove(struct pci_dev *pdev)
{
	struct smmute_device *smmute = pci_get_drvdata(pdev);

	/* TODO: cancel all in-flight transactions */

	if (smmute) {
		pci_iounmap(pdev, smmute->pairs);
		smmute_common_remove(smmute);
	}

	pci_disable_msix(pdev);
	pci_disable_device(pdev);
	pci_release_regions(pdev);
}

static void smmute_plat_write_msi_msg(struct msi_desc *desc,
				      struct msi_msg *msg)
{
	struct smmute_msi_info *entry;
	struct device *dev = msi_desc_to_dev(desc);
	struct smmute_device *smmute = dev_get_drvdata(dev);

	if (desc->platform.msi_index >= smmute->nr_msix_entries) {
		dev_err(dev, "invalid MSI index\n");
		return;
	}

	entry = &smmute->plat_msi_entries[desc->platform.msi_index];
	entry->doorbell = (((u64)msg->address_hi) << 32) | msg->address_lo;
	entry->data = msg->data;
}

static int smmute_plat_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct smmute_msi_info *msi_entries;
	struct smmute_device *smmute;
	struct msi_desc *msi_desc;
	struct resource *mem;
	unsigned long nr_msis = SMMUTE_MAX_MSIS;
	int ret = -ENOMEM;
	int i = 0;

	smmute = devm_kzalloc(dev, sizeof(struct smmute_device), GFP_KERNEL);
	if (!smmute)
		return ret;

	smmute->dev = dev;
	platform_set_drvdata(pdev, smmute);

	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!mem) {
		dev_err(dev, "unable to get resource\n");
		goto err_free_device;
	}

	smmute->pairs = devm_ioremap_resource(&pdev->dev, mem);
	if (IS_ERR(smmute->pairs)) {
		ret = PTR_ERR(smmute->pairs);
		dev_err(dev, "unable to map resource\n");
		goto err_free_device;
	}

	smmute->nr_pairs = resource_size(mem) / sizeof(*smmute->pairs);

	ret = platform_msi_domain_alloc_irqs(dev, nr_msis,
					     smmute_plat_write_msi_msg);
	if (ret) {
		dev_err(dev, "cannot alloc IRQs\n");
		goto err_unmap_resource;
	}

	ret = -ENOMEM;
	msi_entries = devm_kmalloc(dev, sizeof(struct smmute_msi_info) *
				   nr_msis, GFP_KERNEL);
	if (!msi_entries)
		goto err_free_irqs;

	for_each_msi_entry(msi_desc, dev)
		msi_entries[i++].vector = msi_desc->irq;

	smmute->plat_msi_entries = msi_entries;
	smmute->nr_msix_entries = nr_msis;

	dev_dbg(dev, "has %d SIDs\n", dev->iommu_fwspec->num_ids);

	ret = smmute_common_probe(smmute);
	if (ret)
		goto err_free_entries;

	return 0;

err_free_entries:
	devm_kfree(dev, msi_entries);
err_free_irqs:
	platform_msi_domain_free_irqs(dev);
err_unmap_resource:
	devm_iounmap(dev, smmute->pairs);
err_free_device:
	devm_kfree(dev, smmute);

	return ret;
}

static int smmute_plat_remove(struct platform_device *pdev)
{
	struct smmute_device *smmute = platform_get_drvdata(pdev);
	struct device *dev;

	if (!smmute)
		return 0;

	dev = smmute->dev;

	smmute_common_remove(smmute);

	platform_msi_domain_free_irqs(dev);

	devm_iounmap(dev, smmute->pairs);

	return 0;
}

static const struct pci_device_id smmute_id_table[] = {
	{ PCI_DEVICE(VENDOR_ID, DEVICE_ID) },
	{ 0 }
};
MODULE_DEVICE_TABLE(pci, smmute_id_table);

static struct pci_driver smmute_driver = {
	.name		= DRV_NAME,
	.id_table	= smmute_id_table,
	.probe		= smmute_pci_probe,
	.remove		= smmute_pci_remove,
};

static const struct of_device_id smmute_of_table[] = {
	{ .compatible = "arm,smmute" },
	{ }
};
MODULE_DEVICE_TABLE(of, smmute_of_table);

static struct platform_driver smmute_plat_driver = {
	.driver = {
		.name = DRV_NAME,
		.of_match_table = of_match_ptr(smmute_of_table),
	},
	.probe = smmute_plat_probe,
	.remove = smmute_plat_remove,
};

module_platform_driver(smmute_plat_driver);

static int __init smmute_init(void)
{
	int ret;
	dev_t dev_id;

	ret = alloc_chrdev_region(&dev_id, SMMUTE_FIRST_MINOR,
			SMMUTE_MAX_DEVICES, "smmute");
	if (ret) {
		pr_err(DRV_NAME ": alloc_chrdev_region\n");
		return ret;
	}

	smmute_major = MAJOR(dev_id);

	smmute_class = class_create(THIS_MODULE, "smmute");
	if (IS_ERR(smmute_class)) {
		pr_err(DRV_NAME ": class_create\n");
		ret = PTR_ERR(smmute_class);
		goto out_unregister_chrdev;
	}

	cdev_init(&smmute_cdev, &smmute_fops);
	ret = cdev_add(&smmute_cdev, dev_id, SMMUTE_MAX_DEVICES);
	if (ret) {
		pr_err(DRV_NAME ": cdev_add\n");
		goto out_class_destroy;
	}

	ret = pci_register_driver(&smmute_driver);
	if (ret) {
		pr_err(DRV_NAME ": pci_register_driver\n");
		goto out_cdev_del;
	}

	return 0;

out_cdev_del:
	cdev_del(&smmute_cdev);

out_class_destroy:
	class_destroy(smmute_class);

out_unregister_chrdev:
	unregister_chrdev_region(dev_id, SMMUTE_MAX_DEVICES);

	return ret;
}

static void __exit smmute_exit(void)
{
	dev_t dev_id = MKDEV(smmute_major, 0);

	pci_unregister_driver(&smmute_driver);

	cdev_del(&smmute_cdev);
	class_destroy(smmute_class);
	unregister_chrdev_region(dev_id, SMMUTE_MAX_DEVICES);
}

module_init(smmute_init);
module_exit(smmute_exit);

MODULE_DESCRIPTION("Driver for the SMMU Test Engine");
MODULE_AUTHOR("Jean-Philippe Brucker <jean-philippe.brucker@arm.com>");
MODULE_LICENSE("GPL v2");
