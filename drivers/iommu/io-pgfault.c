/*
 * Handle device page faults
 *
 * Copyright (C) 2018 ARM Ltd.
 * Author: Jean-Philippe Brucker <jean-philippe.brucker@arm.com>
 *
 * SPDX-License-Identifier: GPL-2.0
 */

#include <linux/iommu.h>
#include <linux/list.h>
#include <linux/sched/mm.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

static struct workqueue_struct *iommu_fault_queue;
static DECLARE_RWSEM(iommu_fault_queue_sem);
static refcount_t iommu_fault_queue_refs = REFCOUNT_INIT(0);
static BLOCKING_NOTIFIER_HEAD(iommu_fault_queue_flush_notifiers);

/* Used to store incomplete fault groups */
static LIST_HEAD(iommu_partial_faults);
static DEFINE_SPINLOCK(iommu_partial_faults_lock);

struct iommu_fault_context {
	struct device			*dev;
	struct iommu_fault_event	evt;
	struct list_head		head;
};

struct iommu_fault_group {
	struct iommu_domain		*domain;
	struct iommu_fault_context	last_fault;
	struct list_head		faults;
	struct work_struct		work;
};

/*
 * iommu_fault_complete() - Finish handling a fault
 *
 * Send a response if necessary and pass on the sanitized status code
 */
static int iommu_fault_complete(struct iommu_domain *domain, struct device *dev,
				struct iommu_fault_event *evt, int status)
{
	struct page_response_msg resp = {
		.addr		= evt->addr,
		.pasid		= evt->pasid,
		.pasid_present	= evt->pasid_valid,
		.page_req_group_id = evt->page_req_group_id,
		.type		= IOMMU_PAGE_GROUP_RESP,
		.private_data	= evt->iommu_private,
	};

	/*
	 * There is no "handling" an unrecoverable fault, so the only valid
	 * return values are 0 or an error.
	 */
	if (evt->type == IOMMU_FAULT_DMA_UNRECOV)
		return status > 0 ? 0 : status;

	/* Someone took ownership of the fault and will complete it later */
	if (status == IOMMU_PAGE_RESP_HANDLED)
		return 0;

	/*
	 * There was an internal error with handling the recoverable fault. Try
	 * to complete the fault if possible.
	 */
	if (status < 0)
		status = IOMMU_PAGE_RESP_INVALID;

	if (WARN_ON(!domain->ops->page_response))
		/*
		 * The IOMMU driver shouldn't have submitted recoverable faults
		 * if it cannot receive a response.
		 */
		return -EINVAL;

	resp.resp_code = status;
	return domain->ops->page_response(domain, dev, &resp);
}

static int iommu_fault_handle_single(struct iommu_fault_context *fault)
{
	struct mm_struct *mm;
	struct vm_area_struct *vma;
	unsigned int access_flags = 0;
	int ret = IOMMU_PAGE_RESP_INVALID;
	unsigned int fault_flags = FAULT_FLAG_REMOTE;
	struct iommu_fault_event *evt = &fault->evt;

	if (!evt->pasid_valid)
		return ret;

	/*
	 * Special case: PASID Stop Marker (LRW = 0b100) doesn't expect a
	 * response. A Stop Marker may be generated when disabling a PASID
	 * (issuing a PASID stop request) in some PCI devices.
	 *
	 * When the mm_exit() callback returns from the device driver, no page
	 * request is generated for this PASID anymore and outstanding ones have
	 * been pushed to the IOMMU (as per PCIe 4.0r1.0 - 6.20.1 and 10.4.1.2 -
	 * Managing PASID TLP Prefix Usage). Some PCI devices will wait for all
	 * outstanding page requests to come back with a response before
	 * completing the PASID stop request. Others do not wait for page
	 * responses, and instead issue this Stop Marker that tells us when the
	 * PASID can be reallocated.
	 *
	 * We ignore the Stop Marker because:
	 * a. Page requests, which are posted requests, have been flushed to the
	 *    IOMMU when mm_exit() returns,
	 * b. We flush all fault queues after mm_exit() returns and before
	 *    freeing the PASID.
	 *
	 * So even though the Stop Marker might be issued by the device *after*
	 * the stop request completes, outstanding faults will have been dealt
	 * with by the time we free the PASID.
	 */
	if (evt->last_req &&
	    !(evt->prot & (IOMMU_FAULT_READ | IOMMU_FAULT_WRITE)))
		return IOMMU_PAGE_RESP_HANDLED;

	mm = iommu_sva_find(evt->pasid);
	if (!mm)
		return ret;

	down_read(&mm->mmap_sem);

	vma = find_extend_vma(mm, evt->addr);
	if (!vma)
		/* Unmapped area */
		goto out_put_mm;

	if (evt->prot & IOMMU_FAULT_READ)
		access_flags |= VM_READ;

	if (evt->prot & IOMMU_FAULT_WRITE) {
		access_flags |= VM_WRITE;
		fault_flags |= FAULT_FLAG_WRITE;
	}

	if (evt->prot & IOMMU_FAULT_EXEC) {
		access_flags |= VM_EXEC;
		fault_flags |= FAULT_FLAG_INSTRUCTION;
	}

	if (!(evt->prot & IOMMU_FAULT_PRIV))
		fault_flags |= FAULT_FLAG_USER;

	if (access_flags & ~vma->vm_flags)
		/* Access fault */
		goto out_put_mm;

	ret = handle_mm_fault(vma, evt->addr, fault_flags);
	ret = ret & VM_FAULT_ERROR ? IOMMU_PAGE_RESP_INVALID :
		IOMMU_PAGE_RESP_SUCCESS;

out_put_mm:
	up_read(&mm->mmap_sem);

	/*
	 * If the process exits while we're handling the fault on its mm, we
	 * can't do mmput(). exit_mmap() would release the MMU notifier, calling
	 * iommu_notifier_release(), which has to flush the fault queue that
	 * we're executing on... So mmput_async() moves the release of the mm to
	 * another thread, if we're the last user.
	 */
	mmput_async(mm);

	return ret;
}

static void iommu_fault_handle_group(struct work_struct *work)
{
	struct iommu_fault_group *group;
	struct iommu_fault_context *fault, *next;
	int status = IOMMU_PAGE_RESP_SUCCESS;

	group = container_of(work, struct iommu_fault_group, work);

	list_for_each_entry_safe(fault, next, &group->faults, head) {
		struct iommu_fault_event *evt = &fault->evt;
		/*
		 * Errors are sticky: don't handle subsequent faults in the
		 * group if there is an error.
		 */
		if (status == IOMMU_PAGE_RESP_SUCCESS)
			status = iommu_fault_handle_single(fault);

		if (!evt->last_req)
			kfree(fault);
	}

	iommu_fault_complete(group->domain, group->last_fault.dev,
			     &group->last_fault.evt, status);
	kfree(group);
}

static int iommu_queue_fault(struct iommu_domain *domain, struct device *dev,
			     struct iommu_fault_event *evt)
{
	struct iommu_fault_group *group;
	struct iommu_fault_context *fault, *next;

	if (!iommu_fault_queue)
		return -ENOSYS;

	if (!evt->last_req) {
		fault = kzalloc(sizeof(*fault), GFP_KERNEL);
		if (!fault)
			return -ENOMEM;

		fault->evt = *evt;
		fault->dev = dev;

		/* Non-last request of a group. Postpone until the last one */
		spin_lock(&iommu_partial_faults_lock);
		list_add_tail(&fault->head, &iommu_partial_faults);
		spin_unlock(&iommu_partial_faults_lock);

		return IOMMU_PAGE_RESP_HANDLED;
	}

	group = kzalloc(sizeof(*group), GFP_KERNEL);
	if (!group)
		return -ENOMEM;

	group->last_fault.evt = *evt;
	group->last_fault.dev = dev;
	group->domain = domain;
	INIT_LIST_HEAD(&group->faults);
	list_add(&group->last_fault.head, &group->faults);
	INIT_WORK(&group->work, iommu_fault_handle_group);

	/* See if we have pending faults for this group */
	spin_lock(&iommu_partial_faults_lock);
	list_for_each_entry_safe(fault, next, &iommu_partial_faults, head) {
		if (fault->evt.page_req_group_id == evt->page_req_group_id &&
		    fault->dev == dev) {
			list_del(&fault->head);
			/* Insert *before* the last fault */
			list_add(&fault->head, &group->faults);
		}
	}
	spin_unlock(&iommu_partial_faults_lock);

	queue_work(iommu_fault_queue, &group->work);

	/* Postpone the fault completion */
	return IOMMU_PAGE_RESP_HANDLED;
}

/**
 * iommu_report_device_fault() - Handle fault in device driver or mm
 *
 * If the device driver expressed interest in handling fault, report it through
 * the callback. If the fault is recoverable, try to page in the address.
 */
int iommu_report_device_fault(struct device *dev, struct iommu_fault_event *evt)
{
	int ret = -ENOSYS;
	struct iommu_domain *domain = iommu_get_domain_for_dev(dev);

	if (!domain)
		return -ENODEV;

	/*
	 * if upper layers showed interest and installed a fault handler,
	 * invoke it.
	 */
	if (iommu_has_device_fault_handler(dev)) {
		struct iommu_fault_param *param = dev->iommu_param->fault_param;

		return param->handler(evt, param->data);
	}

	/* If the handler is blocking, handle fault in the workqueue */
	if (evt->type == IOMMU_FAULT_PAGE_REQ)
		ret = iommu_queue_fault(domain, dev, evt);

	return iommu_fault_complete(domain, dev, evt, ret);
}
EXPORT_SYMBOL_GPL(iommu_report_device_fault);

/**
 * iommu_fault_queue_register() - register an IOMMU driver to the fault queue
 * @flush_notifier: a notifier block that is called before the fault queue is
 * flushed. The IOMMU driver should commit all faults that are pending in its
 * low-level queues at the time of the call, into the fault queue. The notifier
 * takes a device pointer as argument, hinting what endpoint is causing the
 * flush. When the device is NULL, all faults should be committed.
 */
int iommu_fault_queue_register(struct notifier_block *flush_notifier)
{
	/*
	 * The WQ is unordered because the low-level handler enqueues faults by
	 * group. PRI requests within a group have to be ordered, but once
	 * that's dealt with, the high-level function can handle groups out of
	 * order.
	 */
	down_write(&iommu_fault_queue_sem);
	if (!iommu_fault_queue) {
		iommu_fault_queue = alloc_workqueue("iommu_fault_queue",
						    WQ_UNBOUND, 0);
		if (iommu_fault_queue)
			refcount_set(&iommu_fault_queue_refs, 1);
	} else {
		refcount_inc(&iommu_fault_queue_refs);
	}
	up_write(&iommu_fault_queue_sem);

	if (!iommu_fault_queue)
		return -ENOMEM;

	if (flush_notifier)
		blocking_notifier_chain_register(&iommu_fault_queue_flush_notifiers,
						 flush_notifier);

	return 0;
}
EXPORT_SYMBOL_GPL(iommu_fault_queue_register);

/**
 * iommu_fault_queue_flush() - Ensure that all queued faults have been
 * processed.
 * @dev: the endpoint whose faults need to be flushed. If NULL, flush all
 *       pending faults.
 *
 * Users must call this function when releasing a PASID, to ensure that all
 * pending faults affecting this PASID have been handled, and won't affect the
 * address space of a subsequent process that reuses this PASID.
 */
void iommu_fault_queue_flush(struct device *dev)
{
	blocking_notifier_call_chain(&iommu_fault_queue_flush_notifiers, 0, dev);

	down_read(&iommu_fault_queue_sem);
	/*
	 * Don't flush the partial faults list. All PRGs with the PASID are
	 * complete and have been submitted to the queue.
	 */
	if (iommu_fault_queue)
		flush_workqueue(iommu_fault_queue);
	up_read(&iommu_fault_queue_sem);
}
EXPORT_SYMBOL_GPL(iommu_fault_queue_flush);

/**
 * iommu_fault_queue_unregister() - Unregister an IOMMU driver from the fault
 * queue.
 * @flush_notifier: same parameter as iommu_fault_queue_register
 */
void iommu_fault_queue_unregister(struct notifier_block *flush_notifier)
{
	down_write(&iommu_fault_queue_sem);
	if (refcount_dec_and_test(&iommu_fault_queue_refs)) {
		destroy_workqueue(iommu_fault_queue);
		iommu_fault_queue = NULL;
	}
	up_write(&iommu_fault_queue_sem);

	if (flush_notifier)
		blocking_notifier_chain_unregister(&iommu_fault_queue_flush_notifiers,
						   flush_notifier);
}
EXPORT_SYMBOL_GPL(iommu_fault_queue_unregister);
