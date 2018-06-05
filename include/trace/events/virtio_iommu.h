/* SPDX-License-Identifier: GPL-2.0 */
/*
 * virtio_iommu trace points
 */
#if !defined(_TRACE_VIOMMU_H) || defined(TRACE_HEADER_MULTI_READ)
#define _TRACE_VIOMMU_H

#include <linux/tracepoint.h>

#undef  TRACE_SYSTEM
#define TRACE_SYSTEM virtio_iommu

TRACE_EVENT(viommu_map,
	TP_PROTO(struct device *iommu_dev, u32 domain_id, u64 vstart, u64 vend,
		 u64 phys, u32 prot),
	TP_ARGS(iommu_dev, domain_id, vstart, vend, phys, prot),
	TP_STRUCT__entry(
		__string(iommu_dev, dev_name(iommu_dev))
		__field(u32, domain_id)
		__field(u64, vstart)
		__field(u64, vend)
		__field(u64, phys)
		__field(u32, prot)
	),
	TP_fast_assign(
		__assign_str(iommu_dev, dev_name(iommu_dev));
		__entry->domain_id	= domain_id;
		__entry->vstart		= vstart;
		__entry->vend		= vend;
		__entry->phys		= phys;
		__entry->prot		= prot;
	),
	TP_printk("%s D%u 0x%llx-0x%llx -> 0x%llx (0x%x)",
		  __get_str(iommu_dev), __entry->domain_id, __entry->vstart,
		  __entry->vend, __entry->phys, __entry->prot)
);

TRACE_EVENT(viommu_unmap,
	TP_PROTO(struct device *iommu_dev, u32 domain_id, u64 vstart, u64 vend),
	TP_ARGS(iommu_dev, domain_id, vstart, vend),
	TP_STRUCT__entry(
		__string(iommu_dev, dev_name(iommu_dev))
		__field(u32, domain_id)
		__field(u64, vstart)
		__field(u64, vend)
	),
	TP_fast_assign(
		__assign_str(iommu_dev, dev_name(iommu_dev));
		__entry->domain_id	= domain_id;
		__entry->vstart		= vstart;
		__entry->vend		= vend;
	),
	TP_printk("%s D%u 0x%llx-0x%llx", __get_str(iommu_dev),
		  __entry->domain_id, __entry->vstart, __entry->vend)
);

#endif /* _TRACE_VIOMMU_H */

#include <trace/define_trace.h>
