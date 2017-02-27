/*
 * IOMMU API for ARM architected SMMUv3 implementations.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Copyright (C) 2015 ARM Limited
 *
 * Author: Will Deacon <will.deacon@arm.com>
 *
 * This driver is powered by bad coffee and bombay mix.
 */

#include <linux/acpi.h>
#include <linux/acpi_iort.h>
#include <linux/cpufeature.h>
#include <linux/delay.h>
#include <linux/dma-iommu.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/mmu_context.h>
#include <linux/mmu_notifier.h>
#include <linux/module.h>
#include <linux/msi.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_iommu.h>
#include <linux/of_platform.h>
#include <linux/pci.h>
#include <linux/pci-ats.h>
#include <linux/platform_device.h>

#include <linux/amba/bus.h>

#include "io-pgtable.h"
#include "io-pgtable-arm.h"

/* MMIO registers */
#define ARM_SMMU_IDR0			0x0
#define IDR0_ST_LVL_SHIFT		27
#define IDR0_ST_LVL_MASK		0x3
#define IDR0_ST_LVL_2LVL		(1 << IDR0_ST_LVL_SHIFT)
#define IDR0_STALL_MODEL_SHIFT		24
#define IDR0_STALL_MODEL_MASK		0x3
#define IDR0_STALL_MODEL_STALL		(0 << IDR0_STALL_MODEL_SHIFT)
#define IDR0_STALL_MODEL_FORCE		(2 << IDR0_STALL_MODEL_SHIFT)
#define IDR0_TTENDIAN_SHIFT		21
#define IDR0_TTENDIAN_MASK		0x3
#define IDR0_TTENDIAN_LE		(2 << IDR0_TTENDIAN_SHIFT)
#define IDR0_TTENDIAN_BE		(3 << IDR0_TTENDIAN_SHIFT)
#define IDR0_TTENDIAN_MIXED		(0 << IDR0_TTENDIAN_SHIFT)
#define IDR0_CD2L			(1 << 19)
#define IDR0_VMID16			(1 << 18)
#define IDR0_PRI			(1 << 16)
#define IDR0_SEV			(1 << 14)
#define IDR0_MSI			(1 << 13)
#define IDR0_ASID16			(1 << 12)
#define IDR0_ATS			(1 << 10)
#define IDR0_HYP			(1 << 9)
#define IDR0_BTM			(1 << 5)
#define IDR0_COHACC			(1 << 4)
#define IDR0_TTF_SHIFT			2
#define IDR0_TTF_MASK			0x3
#define IDR0_TTF_AARCH64		(2 << IDR0_TTF_SHIFT)
#define IDR0_TTF_AARCH32_64		(3 << IDR0_TTF_SHIFT)
#define IDR0_S1P			(1 << 1)
#define IDR0_S2P			(1 << 0)

#define ARM_SMMU_IDR1			0x4
#define IDR1_TABLES_PRESET		(1 << 30)
#define IDR1_QUEUES_PRESET		(1 << 29)
#define IDR1_REL			(1 << 28)
#define IDR1_CMDQ_SHIFT			21
#define IDR1_CMDQ_MASK			0x1f
#define IDR1_EVTQ_SHIFT			16
#define IDR1_EVTQ_MASK			0x1f
#define IDR1_PRIQ_SHIFT			11
#define IDR1_PRIQ_MASK			0x1f
#define IDR1_SSID_SHIFT			6
#define IDR1_SSID_MASK			0x1f
#define IDR1_SID_SHIFT			0
#define IDR1_SID_MASK			0x3f

#define ARM_SMMU_IDR5			0x14
#define IDR5_STALL_MAX_SHIFT		16
#define IDR5_STALL_MAX_MASK		0xffff
#define IDR5_GRAN64K			(1 << 6)
#define IDR5_GRAN16K			(1 << 5)
#define IDR5_GRAN4K			(1 << 4)
#define IDR5_OAS_SHIFT			0
#define IDR5_OAS_MASK			0x7
#define IDR5_OAS_32_BIT			(0 << IDR5_OAS_SHIFT)
#define IDR5_OAS_36_BIT			(1 << IDR5_OAS_SHIFT)
#define IDR5_OAS_40_BIT			(2 << IDR5_OAS_SHIFT)
#define IDR5_OAS_42_BIT			(3 << IDR5_OAS_SHIFT)
#define IDR5_OAS_44_BIT			(4 << IDR5_OAS_SHIFT)
#define IDR5_OAS_48_BIT			(5 << IDR5_OAS_SHIFT)

#define ARM_SMMU_CR0			0x20
#define CR0_ATSCHK			(1 << 4)
#define CR0_CMDQEN			(1 << 3)
#define CR0_EVTQEN			(1 << 2)
#define CR0_PRIQEN			(1 << 1)
#define CR0_SMMUEN			(1 << 0)

#define ARM_SMMU_CR0ACK			0x24

#define ARM_SMMU_CR1			0x28
#define CR1_SH_NSH			0
#define CR1_SH_OSH			2
#define CR1_SH_ISH			3
#define CR1_CACHE_NC			0
#define CR1_CACHE_WB			1
#define CR1_CACHE_WT			2
#define CR1_TABLE_SH_SHIFT		10
#define CR1_TABLE_OC_SHIFT		8
#define CR1_TABLE_IC_SHIFT		6
#define CR1_QUEUE_SH_SHIFT		4
#define CR1_QUEUE_OC_SHIFT		2
#define CR1_QUEUE_IC_SHIFT		0

#define ARM_SMMU_CR2			0x2c
#define CR2_PTM				(1 << 2)
#define CR2_RECINVSID			(1 << 1)
#define CR2_E2H				(1 << 0)

#define ARM_SMMU_GBPA			0x44
#define GBPA_ABORT			(1 << 20)
#define GBPA_UPDATE			(1 << 31)

#define ARM_SMMU_IRQ_CTRL		0x50
#define IRQ_CTRL_EVTQ_IRQEN		(1 << 2)
#define IRQ_CTRL_PRIQ_IRQEN		(1 << 1)
#define IRQ_CTRL_GERROR_IRQEN		(1 << 0)

#define ARM_SMMU_IRQ_CTRLACK		0x54

#define ARM_SMMU_GERROR			0x60
#define GERROR_SFM_ERR			(1 << 8)
#define GERROR_MSI_GERROR_ABT_ERR	(1 << 7)
#define GERROR_MSI_PRIQ_ABT_ERR		(1 << 6)
#define GERROR_MSI_EVTQ_ABT_ERR		(1 << 5)
#define GERROR_MSI_CMDQ_ABT_ERR		(1 << 4)
#define GERROR_PRIQ_ABT_ERR		(1 << 3)
#define GERROR_EVTQ_ABT_ERR		(1 << 2)
#define GERROR_CMDQ_ERR			(1 << 0)
#define GERROR_ERR_MASK			0xfd

#define ARM_SMMU_GERRORN		0x64

#define ARM_SMMU_GERROR_IRQ_CFG0	0x68
#define ARM_SMMU_GERROR_IRQ_CFG1	0x70
#define ARM_SMMU_GERROR_IRQ_CFG2	0x74

#define ARM_SMMU_STRTAB_BASE		0x80
#define STRTAB_BASE_RA			(1UL << 62)
#define STRTAB_BASE_ADDR_SHIFT		6
#define STRTAB_BASE_ADDR_MASK		0x3ffffffffffUL

#define ARM_SMMU_STRTAB_BASE_CFG	0x88
#define STRTAB_BASE_CFG_LOG2SIZE_SHIFT	0
#define STRTAB_BASE_CFG_LOG2SIZE_MASK	0x3f
#define STRTAB_BASE_CFG_SPLIT_SHIFT	6
#define STRTAB_BASE_CFG_SPLIT_MASK	0x1f
#define STRTAB_BASE_CFG_FMT_SHIFT	16
#define STRTAB_BASE_CFG_FMT_MASK	0x3
#define STRTAB_BASE_CFG_FMT_LINEAR	(0 << STRTAB_BASE_CFG_FMT_SHIFT)
#define STRTAB_BASE_CFG_FMT_2LVL	(1 << STRTAB_BASE_CFG_FMT_SHIFT)

#define ARM_SMMU_CMDQ_BASE		0x90
#define ARM_SMMU_CMDQ_PROD		0x98
#define ARM_SMMU_CMDQ_CONS		0x9c

#define ARM_SMMU_EVTQ_BASE		0xa0
#define ARM_SMMU_EVTQ_PROD		0x100a8
#define ARM_SMMU_EVTQ_CONS		0x100ac
#define ARM_SMMU_EVTQ_IRQ_CFG0		0xb0
#define ARM_SMMU_EVTQ_IRQ_CFG1		0xb8
#define ARM_SMMU_EVTQ_IRQ_CFG2		0xbc

#define ARM_SMMU_PRIQ_BASE		0xc0
#define ARM_SMMU_PRIQ_PROD		0x100c8
#define ARM_SMMU_PRIQ_CONS		0x100cc
#define ARM_SMMU_PRIQ_IRQ_CFG0		0xd0
#define ARM_SMMU_PRIQ_IRQ_CFG1		0xd8
#define ARM_SMMU_PRIQ_IRQ_CFG2		0xdc

/* Common MSI config fields */
#define MSI_CFG0_ADDR_SHIFT		2
#define MSI_CFG0_ADDR_MASK		0x3fffffffffffUL
#define MSI_CFG2_SH_SHIFT		4
#define MSI_CFG2_SH_NSH			(0UL << MSI_CFG2_SH_SHIFT)
#define MSI_CFG2_SH_OSH			(2UL << MSI_CFG2_SH_SHIFT)
#define MSI_CFG2_SH_ISH			(3UL << MSI_CFG2_SH_SHIFT)
#define MSI_CFG2_MEMATTR_SHIFT		0
#define MSI_CFG2_MEMATTR_DEVICE_nGnRE	(0x1 << MSI_CFG2_MEMATTR_SHIFT)

#define Q_IDX(q, p)			((p) & ((1 << (q)->max_n_shift) - 1))
#define Q_WRP(q, p)			((p) & (1 << (q)->max_n_shift))
#define Q_OVERFLOW_FLAG			(1 << 31)
#define Q_OVF(q, p)			((p) & Q_OVERFLOW_FLAG)
#define Q_ENT(q, p)			((q)->base +			\
					 Q_IDX(q, p) * (q)->ent_dwords)

#define Q_BASE_RWA			(1UL << 62)
#define Q_BASE_ADDR_SHIFT		5
#define Q_BASE_ADDR_MASK		0xfffffffffffUL
#define Q_BASE_LOG2SIZE_SHIFT		0
#define Q_BASE_LOG2SIZE_MASK		0x1fUL

/*
 * Stream table.
 *
 * Linear: Enough to cover 1 << IDR1.SIDSIZE entries
 * 2lvl: 128k L1 entries,
 *       256 lazy entries per table (each table covers a PCI bus)
 */
#define STRTAB_L1_SZ_SHIFT		20
#define STRTAB_SPLIT			8

#define STRTAB_L1_DESC_DWORDS		1
#define STRTAB_L1_DESC_SPAN_SHIFT	0
#define STRTAB_L1_DESC_SPAN_MASK	0x1fUL
#define STRTAB_L1_DESC_L2PTR_SHIFT	6
#define STRTAB_L1_DESC_L2PTR_MASK	0x3ffffffffffUL

#define STRTAB_STE_DWORDS		8
#define STRTAB_STE_0_V			(1UL << 0)
#define STRTAB_STE_0_CFG_SHIFT		1
#define STRTAB_STE_0_CFG_MASK		0x7UL
#define STRTAB_STE_0_CFG_ABORT		(0UL << STRTAB_STE_0_CFG_SHIFT)
#define STRTAB_STE_0_CFG_BYPASS		(4UL << STRTAB_STE_0_CFG_SHIFT)
#define STRTAB_STE_0_CFG_S1_TRANS	(5UL << STRTAB_STE_0_CFG_SHIFT)
#define STRTAB_STE_0_CFG_S2_TRANS	(6UL << STRTAB_STE_0_CFG_SHIFT)

#define STRTAB_STE_0_S1FMT_SHIFT	4
#define STRTAB_STE_0_S1FMT_LINEAR	(0UL << STRTAB_STE_0_S1FMT_SHIFT)
#define STRTAB_STE_0_S1FMT_4K_L2	(1UL << STRTAB_STE_0_S1FMT_SHIFT)
#define STRTAB_STE_0_S1FMT_64K_L2	(2UL << STRTAB_STE_0_S1FMT_SHIFT)
#define STRTAB_STE_0_S1CTXPTR_SHIFT	6
#define STRTAB_STE_0_S1CTXPTR_MASK	0x3ffffffffffUL
#define STRTAB_STE_0_S1CDMAX_SHIFT	59
#define STRTAB_STE_0_S1CDMAX_MASK	0x1fUL

#define STRTAB_STE_1_S1DSS_SHIFT	0
#define STRTAB_STE_1_S1DSS_MASK		0x3UL
#define STRTAB_STE_1_S1DSS_TERMINATE	(0x0 << STRTAB_STE_1_S1DSS_SHIFT)
#define STRTAB_STE_1_S1DSS_BYPASS	(0x1 << STRTAB_STE_1_S1DSS_SHIFT)
#define STRTAB_STE_1_S1DSS_SSID0	(0x2 << STRTAB_STE_1_S1DSS_SHIFT)

#define STRTAB_STE_1_S1C_CACHE_NC	0UL
#define STRTAB_STE_1_S1C_CACHE_WBRA	1UL
#define STRTAB_STE_1_S1C_CACHE_WT	2UL
#define STRTAB_STE_1_S1C_CACHE_WB	3UL
#define STRTAB_STE_1_S1C_SH_NSH		0UL
#define STRTAB_STE_1_S1C_SH_OSH		2UL
#define STRTAB_STE_1_S1C_SH_ISH		3UL
#define STRTAB_STE_1_S1CIR_SHIFT	2
#define STRTAB_STE_1_S1COR_SHIFT	4
#define STRTAB_STE_1_S1CSH_SHIFT	6

#define STRTAB_STE_1_PPAR		(1UL << 18)

#define STRTAB_STE_1_S1STALLD		(1UL << 27)

#define STRTAB_STE_1_EATS_ABT		0UL
#define STRTAB_STE_1_EATS_TRANS		1UL
#define STRTAB_STE_1_EATS_S1CHK		2UL
#define STRTAB_STE_1_EATS_SHIFT		28

#define STRTAB_STE_1_STRW_NSEL1		0UL
#define STRTAB_STE_1_STRW_EL2		2UL
#define STRTAB_STE_1_STRW_SHIFT		30

#define STRTAB_STE_1_SHCFG_INCOMING	1UL
#define STRTAB_STE_1_SHCFG_SHIFT	44

#define STRTAB_STE_2_S2VMID_SHIFT	0
#define STRTAB_STE_2_S2VMID_MASK	0xffffUL
#define STRTAB_STE_2_VTCR_SHIFT		32
#define STRTAB_STE_2_VTCR_MASK		0x7ffffUL
#define STRTAB_STE_2_S2AA64		(1UL << 51)
#define STRTAB_STE_2_S2ENDI		(1UL << 52)
#define STRTAB_STE_2_S2PTW		(1UL << 54)
#define STRTAB_STE_2_S2R		(1UL << 58)

#define STRTAB_STE_3_S2TTB_SHIFT	4
#define STRTAB_STE_3_S2TTB_MASK		0xfffffffffffUL

/*
 * Context descriptor
 *
 * Linear: when less than 1024 SSIDs are supported
 * 2lvl: at most 1024 L1 entrie,
 *	 1024 lazy entries per table.
 */
#define CTXDESC_SPLIT			10
#define CTXDESC_NUM_L2_ENTRIES		(1 << CTXDESC_SPLIT)

#define CTXDESC_L1_DESC_DWORD		1
#define CTXDESC_L1_DESC_VALID		1
#define CTXDESC_L1_DESC_L2PTR_SHIFT	12
#define CTXDESC_L1_DESC_L2PTR_MASK	0xfffffffffUL

#define CTXDESC_CD_DWORDS		8
#define CTXDESC_CD_0_TCR_T0SZ_SHIFT	0
#define ARM64_TCR_T0SZ_SHIFT		0
#define ARM64_TCR_T0SZ_MASK		0x1fUL
#define CTXDESC_CD_0_TCR_TG0_SHIFT	6
#define ARM64_TCR_TG0_SHIFT		14
#define ARM64_TCR_TG0_MASK		0x3UL
#define CTXDESC_CD_0_TCR_IRGN0_SHIFT	8
#define ARM64_TCR_IRGN0_SHIFT		8
#define ARM64_TCR_IRGN0_MASK		0x3UL
#define CTXDESC_CD_0_TCR_ORGN0_SHIFT	10
#define ARM64_TCR_ORGN0_SHIFT		10
#define ARM64_TCR_ORGN0_MASK		0x3UL
#define CTXDESC_CD_0_TCR_SH0_SHIFT	12
#define ARM64_TCR_SH0_SHIFT		12
#define ARM64_TCR_SH0_MASK		0x3UL
#define CTXDESC_CD_0_TCR_EPD0_SHIFT	14
#define ARM64_TCR_EPD0_SHIFT		7
#define ARM64_TCR_EPD0_MASK		0x1UL
#define CTXDESC_CD_0_TCR_EPD1_SHIFT	30
#define ARM64_TCR_EPD1_SHIFT		23
#define ARM64_TCR_EPD1_MASK		0x1UL

#define CTXDESC_CD_0_ENDI		(1UL << 15)
#define CTXDESC_CD_0_V			(1UL << 31)

#define CTXDESC_CD_0_TCR_IPS_SHIFT	32
#define ARM64_TCR_IPS_SHIFT		32
#define ARM64_TCR_IPS_MASK		0x7UL
#define CTXDESC_CD_0_TCR_TBI0_SHIFT	38
#define ARM64_TCR_TBI0_SHIFT		37
#define ARM64_TCR_TBI0_MASK		0x1UL

#define CTXDESC_CD_0_AA64		(1UL << 41)
#define CTXDESC_CD_0_R			(1UL << 45)
#define CTXDESC_CD_0_A			(1UL << 46)
#define CTXDESC_CD_0_ASET_SHIFT		47
#define CTXDESC_CD_0_ASET_SHARED	(0UL << CTXDESC_CD_0_ASET_SHIFT)
#define CTXDESC_CD_0_ASET_PRIVATE	(1UL << CTXDESC_CD_0_ASET_SHIFT)
#define CTXDESC_CD_0_ASID_SHIFT		48
#define CTXDESC_CD_0_ASID_MASK		0xffffUL

#define CTXDESC_CD_1_TTB0_SHIFT		4
#define CTXDESC_CD_1_TTB0_MASK		0xfffffffffffUL

#define CTXDESC_CD_3_MAIR_SHIFT		0

/* Convert between AArch64 (CPU) TCR format and SMMU CD format */
#define ARM_SMMU_TCR2CD(tcr, fld)					\
	(((tcr) >> ARM64_TCR_##fld##_SHIFT & ARM64_TCR_##fld##_MASK)	\
	 << CTXDESC_CD_0_TCR_##fld##_SHIFT)

/* Command queue */
#define CMDQ_ENT_DWORDS			2
#define CMDQ_MAX_SZ_SHIFT		8

#define CMDQ_ERR_SHIFT			24
#define CMDQ_ERR_MASK			0x7f
#define CMDQ_ERR_CERROR_NONE_IDX	0
#define CMDQ_ERR_CERROR_ILL_IDX		1
#define CMDQ_ERR_CERROR_ABT_IDX		2
#define CMDQ_ERR_CERROR_ATC_INV_IDX	3

#define CMDQ_0_OP_SHIFT			0
#define CMDQ_0_OP_MASK			0xffUL
#define CMDQ_0_SSV			(1UL << 11)

#define CMDQ_PREFETCH_0_SSID_SHIFT	12
#define CMDQ_PREFETCH_0_SSID_MASK	0xfffffUL
#define CMDQ_PREFETCH_0_SID_SHIFT	32
#define CMDQ_PREFETCH_1_SIZE_SHIFT	0
#define CMDQ_PREFETCH_1_ADDR_MASK	~0xfffUL

#define CMDQ_CFGI_0_SSID_SHIFT		12
#define CMDQ_CFGI_0_SSID_MASK		0xfffffUL
#define CMDQ_CFGI_0_SID_SHIFT		32
#define CMDQ_CFGI_0_SID_MASK		0xffffffffUL
#define CMDQ_CFGI_1_LEAF		(1UL << 0)
#define CMDQ_CFGI_1_RANGE_SHIFT		0
#define CMDQ_CFGI_1_RANGE_MASK		0x1fUL

#define CMDQ_TLBI_0_VMID_SHIFT		32
#define CMDQ_TLBI_0_ASID_SHIFT		48
#define CMDQ_TLBI_1_LEAF		(1UL << 0)
#define CMDQ_TLBI_1_VA_MASK		~0xfffUL
#define CMDQ_TLBI_1_IPA_MASK		0xfffffffff000UL

#define CMDQ_ATC_0_SSID_SHIFT		12
#define CMDQ_ATC_0_SSID_MASK		0xfffffUL
#define CMDQ_ATC_0_SID_SHIFT		32
#define CMDQ_ATC_0_SID_MASK		0xffffffffUL
#define CMDQ_ATC_0_GLOBAL		(1UL << 9)
#define CMDQ_ATC_1_SIZE_SHIFT		0
#define CMDQ_ATC_1_SIZE_MASK		0x3fUL
#define CMDQ_ATC_1_ADDR_MASK		~0xfffUL

#define CMDQ_PRI_0_SSID_SHIFT		12
#define CMDQ_PRI_0_SSID_MASK		0xfffffUL
#define CMDQ_PRI_0_SID_SHIFT		32
#define CMDQ_PRI_0_SID_MASK		0xffffffffUL
#define CMDQ_PRI_1_GRPID_SHIFT		0
#define CMDQ_PRI_1_GRPID_MASK		0x1ffUL
#define CMDQ_PRI_1_RESP_SHIFT		12
#define CMDQ_PRI_1_RESP_DENY		(0UL << CMDQ_PRI_1_RESP_SHIFT)
#define CMDQ_PRI_1_RESP_FAIL		(1UL << CMDQ_PRI_1_RESP_SHIFT)
#define CMDQ_PRI_1_RESP_SUCC		(2UL << CMDQ_PRI_1_RESP_SHIFT)

#define CMDQ_SYNC_0_CS_SHIFT		12
#define CMDQ_SYNC_0_CS_NONE		(0UL << CMDQ_SYNC_0_CS_SHIFT)
#define CMDQ_SYNC_0_CS_SEV		(2UL << CMDQ_SYNC_0_CS_SHIFT)

/* Event queue */
#define EVTQ_ENT_DWORDS			4
#define EVTQ_MAX_SZ_SHIFT		7

#define EVTQ_0_ID_SHIFT			0
#define EVTQ_0_ID_MASK			0xffUL

/* PRI queue */
#define PRIQ_ENT_DWORDS			2
#define PRIQ_MAX_SZ_SHIFT		8

#define PRIQ_0_SID_SHIFT		0
#define PRIQ_0_SID_MASK			0xffffffffUL
#define PRIQ_0_SSID_SHIFT		32
#define PRIQ_0_SSID_MASK		0xfffffUL
#define PRIQ_0_PERM_PRIV		(1UL << 58)
#define PRIQ_0_PERM_EXEC		(1UL << 59)
#define PRIQ_0_PERM_READ		(1UL << 60)
#define PRIQ_0_PERM_WRITE		(1UL << 61)
#define PRIQ_0_PRG_LAST			(1UL << 62)
#define PRIQ_0_SSID_V			(1UL << 63)

#define PRIQ_1_PRG_IDX_SHIFT		0
#define PRIQ_1_PRG_IDX_MASK		0x1ffUL
#define PRIQ_1_ADDR_SHIFT		12
#define PRIQ_1_ADDR_MASK		0xfffffffffffffUL

/* High-level queue structures */
#define ARM_SMMU_POLL_TIMEOUT_US	100

#define MSI_IOVA_BASE			0x8000000
#define MSI_IOVA_LENGTH			0x100000

static bool disable_bypass;
module_param_named(disable_bypass, disable_bypass, bool, S_IRUGO);
MODULE_PARM_DESC(disable_bypass,
	"Disable bypass streams such that incoming transactions from devices that are not attached to an iommu domain will report an abort back to the device and will not be allowed to pass through the SMMU.");

static bool disable_ats_check;
module_param_named(disable_ats_check, disable_ats_check, bool, S_IRUGO);
MODULE_PARM_DESC(disable_ats_check,
	"By default, the SMMU checks whether each incoming transaction marked as translated is allowed by the stream configuration. This option disables the check.");

enum fault_status {
	/* Non-paging error. SMMU will not handle any fault from this device */
	ARM_SMMU_FAULT_DENY,
	/* Page fault is permanent, device shouldn't retry this access */
	ARM_SMMU_FAULT_FAIL,
	/* Fault has been handled, the access should be retried */
	ARM_SMMU_FAULT_SUCC,
	/* Do not send any reply to the device */
	ARM_SMMU_FAULT_IGNORE,
};

enum arm_smmu_msi_index {
	EVTQ_MSI_INDEX,
	GERROR_MSI_INDEX,
	PRIQ_MSI_INDEX,
	ARM_SMMU_MAX_MSIS,
};

static phys_addr_t arm_smmu_msi_cfg[ARM_SMMU_MAX_MSIS][3] = {
	[EVTQ_MSI_INDEX] = {
		ARM_SMMU_EVTQ_IRQ_CFG0,
		ARM_SMMU_EVTQ_IRQ_CFG1,
		ARM_SMMU_EVTQ_IRQ_CFG2,
	},
	[GERROR_MSI_INDEX] = {
		ARM_SMMU_GERROR_IRQ_CFG0,
		ARM_SMMU_GERROR_IRQ_CFG1,
		ARM_SMMU_GERROR_IRQ_CFG2,
	},
	[PRIQ_MSI_INDEX] = {
		ARM_SMMU_PRIQ_IRQ_CFG0,
		ARM_SMMU_PRIQ_IRQ_CFG1,
		ARM_SMMU_PRIQ_IRQ_CFG2,
	},
};

struct arm_smmu_cmdq_ent {
	/* Common fields */
	u8				opcode;
	bool				substream_valid;

	/* Command-specific fields */
	union {
		#define CMDQ_OP_PREFETCH_CFG	0x1
		struct {
			u32			sid;
			u32			ssid;
			u8			size;
			u64			addr;
		} prefetch;

		#define CMDQ_OP_CFGI_STE	0x3
		#define CMDQ_OP_CFGI_ALL	0x4
		#define CMDQ_OP_CFGI_CD		0x5
		#define CMDQ_OP_CFGI_CD_ALL	0x6
		struct {
			u32			sid;
			u32			ssid;
			union {
				bool		leaf;
				u8		span;
			};
		} cfgi;

		#define CMDQ_OP_TLBI_NH_ASID	0x11
		#define CMDQ_OP_TLBI_NH_VA	0x12
		#define CMDQ_OP_TLBI_EL2_ALL	0x20
		#define CMDQ_OP_TLBI_EL2_ASID	0x21
		#define CMDQ_OP_TLBI_EL2_VA	0x22
		#define CMDQ_OP_TLBI_S12_VMALL	0x28
		#define CMDQ_OP_TLBI_S2_IPA	0x2a
		#define CMDQ_OP_TLBI_NSNH_ALL	0x30
		struct {
			u16			asid;
			u16			vmid;
			bool			leaf;
			u64			addr;
		} tlbi;

		#define CMDQ_OP_ATC_INV		0x40
		struct {
			u32			sid;
			u32			ssid;
			u64			addr;
			u8			size;
			bool			global;
		} atc;

		#define CMDQ_OP_PRI_RESP	0x41
		struct {
			u32			sid;
			u32			ssid;
			u16			grpid;
			enum fault_status	resp;
		} pri;

		#define CMDQ_OP_CMD_SYNC	0x46
	};
};

struct arm_smmu_queue {
	int				irq; /* Wired interrupt */

	__le64				*base;
	dma_addr_t			base_dma;
	u64				q_base;

	size_t				ent_dwords;
	u32				max_n_shift;
	u32				prod;
	u32				cons;

	u32 __iomem			*prod_reg;
	u32 __iomem			*cons_reg;
};

struct arm_smmu_cmdq {
	struct arm_smmu_queue		q;
	spinlock_t			lock;
};

struct arm_smmu_evtq {
	struct arm_smmu_queue		q;
	u32				max_stalls;
};

struct arm_smmu_priq {
	struct arm_smmu_queue		q;

	u64				batch;
	wait_queue_head_t		wq;
};

/* High-level stream table and context descriptor structures */
struct arm_smmu_strtab_l1_desc {
	u8				span;

	__le64				*l2ptr;
	dma_addr_t			l2ptr_dma;
};

struct arm_smmu_s1_cfg {
	u16				asid;
	u64				ttbr;
	u64				tcr;
	u64				mair;
};

struct arm_smmu_s2_cfg {
	u16				vmid;
	u64				vttbr;
	u64				vtcr;
};

struct arm_smmu_cd_table {
	__le64				*cdptr;
	dma_addr_t			cdptr_dma;

	unsigned long			*context_map;
};

struct arm_smmu_cd_cfg {
	bool				linear;

	union {
		struct arm_smmu_cd_table table;
		struct {
			__le64		*ptr;
			dma_addr_t	ptr_dma;

			struct arm_smmu_cd_table *tables;
			unsigned long	cur_table;
		} l1;
	};

	size_t				num_entries;
};

struct arm_smmu_strtab_ent {
	bool				valid;

	bool				bypass;	/* Overrides s1/s2 config */
	struct arm_smmu_cd_cfg		cd_cfg;
	struct arm_smmu_s1_cfg		*s1_cfg;
	struct arm_smmu_s2_cfg		*s2_cfg;

	bool				prg_response_needs_ssid;
};

struct arm_smmu_strtab_cfg {
	__le64				*strtab;
	dma_addr_t			strtab_dma;
	struct arm_smmu_strtab_l1_desc	*l1_desc;
	unsigned int			num_l1_ents;

	u64				strtab_base;
	u32				strtab_base_cfg;
};

/* An SMMUv3 instance */
struct arm_smmu_device {
	struct device			*dev;
	void __iomem			*base;

#define ARM_SMMU_FEAT_2_LVL_STRTAB	(1 << 0)
#define ARM_SMMU_FEAT_2_LVL_CDTAB	(1 << 1)
#define ARM_SMMU_FEAT_TT_LE		(1 << 2)
#define ARM_SMMU_FEAT_TT_BE		(1 << 3)
#define ARM_SMMU_FEAT_PRI		(1 << 4)
#define ARM_SMMU_FEAT_ATS		(1 << 5)
#define ARM_SMMU_FEAT_SEV		(1 << 6)
#define ARM_SMMU_FEAT_MSI		(1 << 7)
#define ARM_SMMU_FEAT_COHERENCY		(1 << 8)
#define ARM_SMMU_FEAT_TRANS_S1		(1 << 9)
#define ARM_SMMU_FEAT_TRANS_S2		(1 << 10)
#define ARM_SMMU_FEAT_STALLS		(1 << 11)
#define ARM_SMMU_FEAT_HYP		(1 << 12)
#define ARM_SMMU_FEAT_E2H		(1 << 13)
#define ARM_SMMU_FEAT_BTM		(1 << 14)
#define ARM_SMMU_FEAT_SVM		(1 << 15)
	u32				features;

#define ARM_SMMU_OPT_SKIP_PREFETCH	(1 << 0)
	u32				options;

	struct arm_smmu_cmdq		cmdq;
	struct arm_smmu_evtq		evtq;
	struct arm_smmu_priq		priq;

	int				gerr_irq;

	unsigned long			ias; /* IPA */
	unsigned long			oas; /* PA */
	unsigned long			pgsize_bitmap;

#define ARM_SMMU_MAX_ASIDS		(1 << 16)
	unsigned int			asid_bits;
	DECLARE_BITMAP(asid_map, ARM_SMMU_MAX_ASIDS);

#define ARM_SMMU_MAX_VMIDS		(1 << 16)
	unsigned int			vmid_bits;
	DECLARE_BITMAP(vmid_map, ARM_SMMU_MAX_VMIDS);

	unsigned int			ssid_bits;
	unsigned int			sid_bits;

	struct arm_smmu_strtab_cfg	strtab_cfg;

	/* IOMMU core code handle */
	struct iommu_device		iommu;

	spinlock_t			contexts_lock;
	struct rb_root			streams;
	struct list_head		tasks;

	struct workqueue_struct		*fault_queue;

	struct list_head		domains;
	struct mutex			domains_mutex;
};

struct arm_smmu_stream {
	u32				id;
	struct arm_smmu_master_data	*master;
	struct rb_node			node;
};

/* SMMU private data for each master */
struct arm_smmu_master_data {
	struct arm_smmu_device		*smmu;
	struct arm_smmu_strtab_ent	ste;

	struct device			*dev;
	struct list_head		group_head;

	struct arm_smmu_stream		*streams;
	struct rb_root			contexts;

	bool				can_fault;
	u32				avail_contexts;
	struct work_struct		sweep_contexts;
#define STALE_CONTEXTS_LIMIT(master)	((master)->avail_contexts / 4)
	u32				stale_contexts;

	const struct iommu_svm_ops	*svm_ops;
};

/* SMMU private data for an IOMMU domain */
enum arm_smmu_domain_stage {
	ARM_SMMU_DOMAIN_S1 = 0,
	ARM_SMMU_DOMAIN_S2,
	ARM_SMMU_DOMAIN_NESTED,
};

struct arm_smmu_domain {
	struct arm_smmu_device		*smmu;
	struct mutex			init_mutex; /* Protects smmu pointer */

	struct io_pgtable_ops		*pgtbl_ops;
	spinlock_t			pgtbl_lock;

	enum arm_smmu_domain_stage	stage;
	union {
		struct arm_smmu_s1_cfg	s1_cfg;
		struct arm_smmu_s2_cfg	s2_cfg;
	};

	struct iommu_domain		domain;

	struct list_head		groups;
	spinlock_t			groups_lock;

	struct list_head		list; /* For domain search by ASID */
};

struct arm_smmu_fault {
	struct arm_smmu_device		*smmu;
	u32				sid;
	u32				ssid;
	bool				ssv;
	u16				grpid;

	u64				iova;
	bool				read;
	bool				write;
	bool				exec;
	bool				priv;

	bool				last;

	struct work_struct		work;
};

struct arm_smmu_pri_group {
	u16				index;
	enum fault_status		resp;

	struct list_head		list;
};

struct arm_smmu_task {
	struct pid			*pid;

	struct arm_smmu_device		*smmu;
	struct list_head		smmu_head;

	struct list_head		contexts;

	struct arm_smmu_s1_cfg		s1_cfg;

	struct mmu_notifier		mmu_notifier;
	struct mm_struct		*mm;

	struct list_head		prgs;

	struct kref			kref;
};

struct arm_smmu_context {
	u32				ssid;

	struct arm_smmu_task		*task;
	struct arm_smmu_master_data	*master;
	void				*priv;

	struct list_head		task_head;
	struct rb_node			master_node;
	struct list_head		flush_head;

	struct kref			kref;

#define ARM_SMMU_CONTEXT_STALE		(1 << 0)
#define ARM_SMMU_CONTEXT_INVALIDATED	(1 << 1)
#define ARM_SMMU_CONTEXT_FREE		(ARM_SMMU_CONTEXT_STALE |\
					 ARM_SMMU_CONTEXT_INVALIDATED)
	atomic64_t			state;
};

struct arm_smmu_group {
	struct arm_smmu_domain		*domain;
	struct list_head		domain_head;

	struct list_head		devices;
	spinlock_t			devices_lock;

	bool				ats_enabled;
};

struct arm_smmu_option_prop {
	u32 opt;
	const char *prop;
};

static struct arm_smmu_option_prop arm_smmu_options[] = {
	{ ARM_SMMU_OPT_SKIP_PREFETCH, "hisilicon,broken-prefetch-cmd" },
	{ 0, NULL},
};

static struct arm_smmu_domain *to_smmu_domain(struct iommu_domain *dom)
{
	return container_of(dom, struct arm_smmu_domain, domain);
}

static struct kmem_cache *arm_smmu_fault_cache;

#define to_smmu_group iommu_group_get_iommudata

static void parse_driver_options(struct arm_smmu_device *smmu)
{
	int i = 0;

	do {
		if (of_property_read_bool(smmu->dev->of_node,
						arm_smmu_options[i].prop)) {
			smmu->options |= arm_smmu_options[i].opt;
			dev_notice(smmu->dev, "option %s\n",
				arm_smmu_options[i].prop);
		}
	} while (arm_smmu_options[++i].opt);
}

static int arm_smmu_bitmap_alloc(unsigned long *map, int span)
{
	int idx, size = 1 << span;

	do {
		idx = find_first_zero_bit(map, size);
		if (idx == size)
			return -ENOSPC;
	} while (test_and_set_bit(idx, map));

	return idx;
}

static void arm_smmu_bitmap_free(unsigned long *map, int idx)
{
	clear_bit(idx, map);
}

/* Low-level queue manipulation functions */
static bool queue_full(struct arm_smmu_queue *q)
{
	return Q_IDX(q, q->prod) == Q_IDX(q, q->cons) &&
	       Q_WRP(q, q->prod) != Q_WRP(q, q->cons);
}

static bool queue_empty(struct arm_smmu_queue *q)
{
	return Q_IDX(q, q->prod) == Q_IDX(q, q->cons) &&
	       Q_WRP(q, q->prod) == Q_WRP(q, q->cons);
}

static void queue_sync_cons(struct arm_smmu_queue *q)
{
	q->cons = readl_relaxed(q->cons_reg);
}

static void queue_inc_cons(struct arm_smmu_queue *q)
{
	u32 cons = (Q_WRP(q, q->cons) | Q_IDX(q, q->cons)) + 1;

	q->cons = Q_OVF(q, q->cons) | Q_WRP(q, cons) | Q_IDX(q, cons);
	writel(q->cons, q->cons_reg);
}

static void queue_sync_cons_ovf(struct arm_smmu_queue *q)
{
	/* Acknowledge overflow condition if any */
	if (Q_OVF(q, q->prod) == Q_OVF(q, q->cons))
		return;

	q->cons = Q_OVF(q, q->prod) | Q_WRP(q, q->cons) | Q_IDX(q, q->cons);
	writel(q->cons, q->cons_reg);
}

static int queue_sync_prod(struct arm_smmu_queue *q)
{
	int ret = 0;
	u32 prod = readl_relaxed(q->prod_reg);

	if (Q_OVF(q, prod) != Q_OVF(q, q->prod))
		ret = -EOVERFLOW;

	q->prod = prod;
	return ret;
}

static void queue_inc_prod(struct arm_smmu_queue *q)
{
	u32 prod = (Q_WRP(q, q->prod) | Q_IDX(q, q->prod)) + 1;

	q->prod = Q_OVF(q, q->prod) | Q_WRP(q, prod) | Q_IDX(q, prod);
	writel(q->prod, q->prod_reg);
}

/*
 * Wait for the SMMU to consume items. If drain is true, wait until the queue
 * is empty. Otherwise, wait until there is at least one free slot.
 */
static int queue_poll_cons(struct arm_smmu_queue *q, bool drain, bool wfe)
{
	ktime_t timeout = ktime_add_us(ktime_get(), ARM_SMMU_POLL_TIMEOUT_US);

	while (queue_sync_cons(q), (drain ? !queue_empty(q) : queue_full(q))) {
		if (ktime_compare(ktime_get(), timeout) > 0)
			return -ETIMEDOUT;

		if (wfe) {
			wfe();
		} else {
			cpu_relax();
			udelay(1);
		}
	}

	return 0;
}

static void queue_write(__le64 *dst, u64 *src, size_t n_dwords)
{
	int i;

	for (i = 0; i < n_dwords; ++i)
		*dst++ = cpu_to_le64(*src++);
}

static int queue_insert_raw(struct arm_smmu_queue *q, u64 *ent)
{
	if (queue_full(q))
		return -ENOSPC;

	queue_write(Q_ENT(q, q->prod), ent, q->ent_dwords);
	queue_inc_prod(q);
	return 0;
}

static void queue_read(__le64 *dst, u64 *src, size_t n_dwords)
{
	int i;

	for (i = 0; i < n_dwords; ++i)
		*dst++ = le64_to_cpu(*src++);
}

static int queue_remove_raw(struct arm_smmu_queue *q, u64 *ent)
{
	if (queue_empty(q))
		return -EAGAIN;

	queue_read(ent, Q_ENT(q, q->cons), q->ent_dwords);
	queue_inc_cons(q);
	return 0;
}

/* High-level queue accessors */
static int arm_smmu_cmdq_build_cmd(u64 *cmd, struct arm_smmu_cmdq_ent *ent)
{
	memset(cmd, 0, CMDQ_ENT_DWORDS << 3);
	cmd[0] |= (ent->opcode & CMDQ_0_OP_MASK) << CMDQ_0_OP_SHIFT;

	switch (ent->opcode) {
	case CMDQ_OP_TLBI_EL2_ALL:
	case CMDQ_OP_TLBI_NSNH_ALL:
		break;
	case CMDQ_OP_PREFETCH_CFG:
		cmd[0] |= ent->substream_valid ? CMDQ_0_SSV : 0;
		cmd[0] |= (u64)ent->prefetch.sid << CMDQ_PREFETCH_0_SID_SHIFT;
		cmd[0] |= ent->prefetch.ssid << CMDQ_PREFETCH_0_SSID_SHIFT;
		cmd[1] |= ent->prefetch.size << CMDQ_PREFETCH_1_SIZE_SHIFT;
		cmd[1] |= ent->prefetch.addr & CMDQ_PREFETCH_1_ADDR_MASK;
		break;
	case CMDQ_OP_CFGI_CD:
		cmd[0] |= ent->cfgi.ssid << CMDQ_CFGI_0_SSID_SHIFT;
		/* pass through */
	case CMDQ_OP_CFGI_STE:
		cmd[0] |= (u64)ent->cfgi.sid << CMDQ_CFGI_0_SID_SHIFT;
		cmd[1] |= ent->cfgi.leaf ? CMDQ_CFGI_1_LEAF : 0;
		break;
	case CMDQ_OP_CFGI_CD_ALL:
		cmd[0] |= (u64)ent->cfgi.sid << CMDQ_CFGI_0_SID_SHIFT;
		break;
	case CMDQ_OP_CFGI_ALL:
		/* Cover the entire SID range */
		cmd[1] |= CMDQ_CFGI_1_RANGE_MASK << CMDQ_CFGI_1_RANGE_SHIFT;
		break;
	case CMDQ_OP_TLBI_NH_VA:
	case CMDQ_OP_TLBI_EL2_VA:
		cmd[0] |= (u64)ent->tlbi.asid << CMDQ_TLBI_0_ASID_SHIFT;
		cmd[1] |= ent->tlbi.leaf ? CMDQ_TLBI_1_LEAF : 0;
		cmd[1] |= ent->tlbi.addr & CMDQ_TLBI_1_VA_MASK;
		break;
	case CMDQ_OP_TLBI_S2_IPA:
		cmd[0] |= (u64)ent->tlbi.vmid << CMDQ_TLBI_0_VMID_SHIFT;
		cmd[1] |= ent->tlbi.leaf ? CMDQ_TLBI_1_LEAF : 0;
		cmd[1] |= ent->tlbi.addr & CMDQ_TLBI_1_IPA_MASK;
		break;
	case CMDQ_OP_TLBI_NH_ASID:
		cmd[0] |= (u64)ent->tlbi.asid << CMDQ_TLBI_0_ASID_SHIFT;
		/* Fallthrough */
	case CMDQ_OP_TLBI_S12_VMALL:
		cmd[0] |= (u64)ent->tlbi.vmid << CMDQ_TLBI_0_VMID_SHIFT;
		break;
	case CMDQ_OP_TLBI_EL2_ASID:
		cmd[0] |= (u64)ent->tlbi.asid << CMDQ_TLBI_0_ASID_SHIFT;
		break;
	case CMDQ_OP_ATC_INV:
		cmd[0] |= ent->substream_valid ? CMDQ_0_SSV : 0;
		cmd[0] |= ent->atc.global ? CMDQ_ATC_0_GLOBAL : 0;
		cmd[0] |= ent->atc.ssid << CMDQ_ATC_0_SSID_SHIFT;
		cmd[0] |= (u64)ent->atc.sid << CMDQ_ATC_0_SID_SHIFT;
		cmd[1] |= ent->atc.size << CMDQ_ATC_1_SIZE_SHIFT;
		cmd[1] |= ent->atc.addr & CMDQ_ATC_1_ADDR_MASK;
		break;
	case CMDQ_OP_PRI_RESP:
		cmd[0] |= ent->substream_valid ? CMDQ_0_SSV : 0;
		cmd[0] |= ent->pri.ssid << CMDQ_PRI_0_SSID_SHIFT;
		cmd[0] |= (u64)ent->pri.sid << CMDQ_PRI_0_SID_SHIFT;
		cmd[1] |= ent->pri.grpid << CMDQ_PRI_1_GRPID_SHIFT;
		switch (ent->pri.resp) {
		case ARM_SMMU_FAULT_DENY:
			cmd[1] |= CMDQ_PRI_1_RESP_DENY;
			break;
		case ARM_SMMU_FAULT_FAIL:
			cmd[1] |= CMDQ_PRI_1_RESP_FAIL;
			break;
		case ARM_SMMU_FAULT_SUCC:
			cmd[1] |= CMDQ_PRI_1_RESP_SUCC;
			break;
		default:
			return -EINVAL;
		}
		break;
	case CMDQ_OP_CMD_SYNC:
		cmd[0] |= CMDQ_SYNC_0_CS_SEV;
		break;
	default:
		return -ENOENT;
	}

	return 0;
}

static void arm_smmu_cmdq_skip_err(struct arm_smmu_device *smmu)
{
	static const char *cerror_str[] = {
		[CMDQ_ERR_CERROR_NONE_IDX]	= "No error",
		[CMDQ_ERR_CERROR_ILL_IDX]	= "Illegal command",
		[CMDQ_ERR_CERROR_ABT_IDX]	= "Abort on command fetch",
		[CMDQ_ERR_CERROR_ATC_INV_IDX]	= "ATC invalidate timeout",
	};

	int i;
	u64 cmd[CMDQ_ENT_DWORDS];
	struct arm_smmu_queue *q = &smmu->cmdq.q;
	u32 cons = readl_relaxed(q->cons_reg);
	u32 idx = cons >> CMDQ_ERR_SHIFT & CMDQ_ERR_MASK;
	struct arm_smmu_cmdq_ent cmd_sync = {
		.opcode = CMDQ_OP_CMD_SYNC,
	};

	dev_err(smmu->dev, "CMDQ error (cons 0x%08x): %s\n", cons,
		idx < ARRAY_SIZE(cerror_str) ?  cerror_str[idx] : "Unknown");

	switch (idx) {
	case CMDQ_ERR_CERROR_ABT_IDX:
		dev_err(smmu->dev, "retrying command fetch\n");
	case CMDQ_ERR_CERROR_NONE_IDX:
		return;
	case CMDQ_ERR_CERROR_ATC_INV_IDX:
		/*
		 * CMD_SYNC failed because of ATC Invalidation completion
		 * timeout. CONS is still pointing at the CMD_SYNC. Ensure other
		 * operations complete by re-submitting the CMD_SYNC, cowardly
		 * ignoring the ATC error.
		 */
	case CMDQ_ERR_CERROR_ILL_IDX:
		/* Fallthrough */
	default:
		break;
	}

	/*
	 * We may have concurrent producers, so we need to be careful
	 * not to touch any of the shadow cmdq state.
	 */
	queue_read(cmd, Q_ENT(q, cons), q->ent_dwords);
	dev_err(smmu->dev, "skipping command in error state:\n");
	for (i = 0; i < ARRAY_SIZE(cmd); ++i)
		dev_err(smmu->dev, "\t0x%016llx\n", (unsigned long long)cmd[i]);

	/* Convert the erroneous command into a CMD_SYNC */
	if (arm_smmu_cmdq_build_cmd(cmd, &cmd_sync)) {
		dev_err(smmu->dev, "failed to convert to CMD_SYNC\n");
		return;
	}

	queue_write(Q_ENT(q, cons), cmd, q->ent_dwords);
}

static void arm_smmu_cmdq_issue_cmd(struct arm_smmu_device *smmu,
				    struct arm_smmu_cmdq_ent *ent)
{
	u64 cmd[CMDQ_ENT_DWORDS];
	unsigned long flags;
	bool wfe = !!(smmu->features & ARM_SMMU_FEAT_SEV);
	struct arm_smmu_queue *q = &smmu->cmdq.q;

	if (arm_smmu_cmdq_build_cmd(cmd, ent)) {
		dev_warn(smmu->dev, "ignoring unknown CMDQ opcode 0x%x\n",
			 ent->opcode);
		return;
	}

	spin_lock_irqsave(&smmu->cmdq.lock, flags);
	while (queue_insert_raw(q, cmd) == -ENOSPC) {
		if (queue_poll_cons(q, false, wfe))
			dev_err_ratelimited(smmu->dev, "CMDQ timeout\n");
	}

	if (ent->opcode == CMDQ_OP_CMD_SYNC && queue_poll_cons(q, true, wfe))
		dev_err_ratelimited(smmu->dev, "CMD_SYNC timeout\n");
	spin_unlock_irqrestore(&smmu->cmdq.lock, flags);
}

static void arm_smmu_fault_reply(struct arm_smmu_fault *fault,
				 enum fault_status resp)
{
	struct arm_smmu_cmdq_ent cmd = {
		.opcode			= CMDQ_OP_PRI_RESP,
		.substream_valid	= fault->ssv,
		.pri			= {
			.sid	= fault->sid,
			.ssid	= fault->ssid,
			.grpid	= fault->grpid,
			.resp	= resp,
		},
	};

	if (!fault->last || resp == ARM_SMMU_FAULT_IGNORE)
		return;

	arm_smmu_cmdq_issue_cmd(fault->smmu, &cmd);
	cmd.opcode = CMDQ_OP_CMD_SYNC;
	arm_smmu_cmdq_issue_cmd(fault->smmu, &cmd);
}

/* Context descriptor manipulation functions */
static void arm_smmu_sync_cd(struct arm_smmu_master_data *master, u32 ssid,
			     bool leaf)
{
	size_t i;
	struct arm_smmu_device *smmu = master->smmu;
	struct iommu_fwspec *fwspec = master->dev->iommu_fwspec;
	struct arm_smmu_cmdq_ent cmd = {
		.opcode = CMDQ_OP_CFGI_CD,
		.cfgi   = {
			.ssid   = ssid,
			.leaf   = leaf,
		},
	};

	for (i = 0; i < fwspec->num_ids; i++) {
		cmd.cfgi.sid = fwspec->ids[i];
		arm_smmu_cmdq_issue_cmd(smmu, &cmd);
	}

	cmd.opcode = CMDQ_OP_CMD_SYNC;
	arm_smmu_cmdq_issue_cmd(smmu, &cmd);
}

static __u64 *arm_smmu_get_cd_ptr(struct arm_smmu_cd_cfg *cfg, u32 ssid)
{
	unsigned long idx;
	struct arm_smmu_cd_table *l1_desc;

	if (cfg->linear)
		return cfg->table.cdptr + ssid * CTXDESC_CD_DWORDS;

	idx = ssid >> CTXDESC_SPLIT;
	if (idx >= cfg->num_entries)
		return NULL;

	l1_desc = &cfg->l1.tables[idx];
	if (!l1_desc->cdptr)
		return NULL;

	idx = ssid & ((1 << CTXDESC_SPLIT) - 1);

	return l1_desc->cdptr + idx * CTXDESC_CD_DWORDS;
}

static u64 arm_smmu_cpu_tcr_to_cd(struct arm_smmu_device *smmu, u64 tcr)
{
	u64 val = 0;

	/* Repack the TCR. Just care about TTBR0 for now */
	val |= ARM_SMMU_TCR2CD(tcr, T0SZ);
	val |= ARM_SMMU_TCR2CD(tcr, TG0);
	val |= ARM_SMMU_TCR2CD(tcr, IRGN0);
	val |= ARM_SMMU_TCR2CD(tcr, ORGN0);
	val |= ARM_SMMU_TCR2CD(tcr, SH0);
	val |= ARM_SMMU_TCR2CD(tcr, EPD0);
	val |= ARM_SMMU_TCR2CD(tcr, EPD1);
	val |= ARM_SMMU_TCR2CD(tcr, IPS);
	if (!(smmu->features & ARM_SMMU_FEAT_ATS))
		val |= ARM_SMMU_TCR2CD(tcr, TBI0);

	return val;
}

static void arm_smmu_write_cd_l1_desc(__le64 *dst,
				      struct arm_smmu_cd_table *table)
{
	u64 val = (table->cdptr_dma & CTXDESC_L1_DESC_L2PTR_MASK
		  << CTXDESC_L1_DESC_L2PTR_SHIFT) | CTXDESC_L1_DESC_VALID;

	*dst = cpu_to_le64(val);
}

static void arm_smmu_write_ctx_desc(struct arm_smmu_master_data *master,
				    u32 ssid, struct arm_smmu_s1_cfg *cfg)
{
	u64 val;
	bool cd_live;
	struct arm_smmu_device *smmu = master->smmu;
	struct arm_smmu_cd_cfg *descs_cfg = &master->ste.cd_cfg;
	__u64 *cdptr = arm_smmu_get_cd_ptr(descs_cfg, ssid);

	/*
	 * This function handles the following cases:
	 *
	 * (1) Install primary CD, for normal DMA traffic (SSID = 0). In this
	 *     case, invalidation is performed when installing the STE.
	 * (2) Install a secondary CD, for SID+SSID traffic, followed by an
	 *     invalidation.
	 * (3) Update ASID of primary CD. This is allowed by atomically writing
	 *     the first 64 bits of the CD, followed by invalidation of the old
	 *     entry and mappings.
	 * (4) Remove a secondary CD and invalidate it.
	 * (5) Remove primary CD. The STE is cleared and invalidated beforehand,
	 *     so this CD is already unreachable and invalidated.
	 */

	if (WARN_ON(!cdptr))
		return;

	val = le64_to_cpu(cdptr[0]);
	cd_live = !!(val & CTXDESC_CD_0_V);

	if (!cfg) {
		/* (4) and (5) */
		cdptr[0] = 0;
		if (ssid && cd_live)
			arm_smmu_sync_cd(master, ssid, true);
		return;
	}

	if (cd_live) {
		/* (3) */
		val &= ~(CTXDESC_CD_0_ASID_MASK << CTXDESC_CD_0_ASID_SHIFT);
		val |= (u64)cfg->asid << CTXDESC_CD_0_ASID_SHIFT;

		cdptr[0] = cpu_to_le64(val);
		/*
		 * Until CD+TLB invalidation, both ASIDs may be used for tagging
		 * this substream's traffic
		 */

	} else {
		/* (1) and (2) */
		cdptr[1] = cpu_to_le64(cfg->ttbr & CTXDESC_CD_1_TTB0_MASK
				       << CTXDESC_CD_1_TTB0_SHIFT);
		cdptr[2] = 0;
		cdptr[3] = cpu_to_le64(cfg->mair << CTXDESC_CD_3_MAIR_SHIFT);

		if (ssid)
			/*
			 * STE is live, and the SMMU might fetch this CD at any
			 * time. Ensure it observes the rest of the CD before we
			 * enable it.
			 */
			arm_smmu_sync_cd(master, ssid, true);

		val = arm_smmu_cpu_tcr_to_cd(smmu, cfg->tcr) |
#ifdef __BIG_ENDIAN
		      CTXDESC_CD_0_ENDI |
#endif
		      CTXDESC_CD_0_R | CTXDESC_CD_0_A |
		      (ssid ? CTXDESC_CD_0_ASET_SHARED :
			      CTXDESC_CD_0_ASET_PRIVATE) |
		      CTXDESC_CD_0_AA64 |
		      (u64)cfg->asid << CTXDESC_CD_0_ASID_SHIFT |
		      CTXDESC_CD_0_V;

		cdptr[0] = cpu_to_le64(val);

	}

	if (ssid || cd_live)
		arm_smmu_sync_cd(master, ssid, true);
}

static int arm_smmu_alloc_cd_leaf_table(struct arm_smmu_device *smmu,
					struct arm_smmu_cd_table *desc,
					size_t num_entries)
{
	size_t size = num_entries * (CTXDESC_CD_DWORDS << 3);

	desc->context_map = devm_kzalloc(smmu->dev, BITS_TO_LONGS(num_entries) *
					 sizeof(long), GFP_ATOMIC);
	if (!desc->context_map)
		return -ENOMEM;

	desc->cdptr = dmam_alloc_coherent(smmu->dev, size, &desc->cdptr_dma,
					  GFP_ATOMIC | __GFP_ZERO);
	if (!desc->cdptr) {
		devm_kfree(smmu->dev, desc->context_map);
		return -ENOMEM;
	}

	return 0;
}

static void arm_smmu_free_cd_leaf_table(struct arm_smmu_device *smmu,
					struct arm_smmu_cd_table *desc,
					size_t num_entries)
{
	size_t size = num_entries * (CTXDESC_CD_DWORDS << 3);

	dmam_free_coherent(smmu->dev, size, desc->cdptr, desc->cdptr_dma);
	devm_kfree(smmu->dev, desc->context_map);
}

static int arm_smmu_alloc_cd_tables(struct arm_smmu_master_data *master,
				    int nr_ssids)
{
	int ret;
	size_t num_leaf_entries, size = 0;
	struct arm_smmu_cd_table *leaf_table;
	struct arm_smmu_device *smmu = master->smmu;
	struct arm_smmu_cd_cfg *cfg = &master->ste.cd_cfg;

	if (cfg->num_entries) {
		/*
		 * Messy master initialization. arm_smmu_add_device already
		 * moaned about it, let's ignore it.
		 */
		return nr_ssids;
	}

	nr_ssids = clamp_val(nr_ssids, 1, 1 << smmu->ssid_bits);
	if (WARN_ON_ONCE(!is_power_of_2(nr_ssids)))
		nr_ssids = 1;

	if (nr_ssids <= (1 << CTXDESC_SPLIT)) {
		/* Fits in a single table */
		cfg->linear = true;
		cfg->num_entries = num_leaf_entries = nr_ssids;
		leaf_table = &cfg->table;
	} else {
		/*
		 * SSID[S1CDmax-1:10] indexes 1st-level table, SSID[9:0] indexes
		 * 2nd-level
		 */
		cfg->linear = false;
		cfg->num_entries = nr_ssids / CTXDESC_NUM_L2_ENTRIES;

		cfg->l1.tables = devm_kzalloc(smmu->dev,
					      sizeof(struct arm_smmu_cd_table) *
					      cfg->num_entries, GFP_KERNEL);
		if (!cfg->l1.tables)
			return -ENOMEM;

		size = cfg->num_entries * (CTXDESC_L1_DESC_DWORD << 3);
		cfg->l1.ptr = dmam_alloc_coherent(smmu->dev, size,
						  &cfg->l1.ptr_dma,
						  GFP_KERNEL | __GFP_ZERO);
		if (!cfg->l1.ptr) {
			devm_kfree(smmu->dev, cfg->l1.tables);
			return -ENOMEM;
		}

		num_leaf_entries = CTXDESC_NUM_L2_ENTRIES;
		leaf_table = cfg->l1.tables;
	}

	ret = arm_smmu_alloc_cd_leaf_table(smmu, leaf_table, num_leaf_entries);
	if (ret) {
		if (!cfg->linear) {
			dmam_free_coherent(smmu->dev, size, cfg->l1.ptr,
					   cfg->l1.ptr_dma);
			devm_kfree(smmu->dev, cfg->l1.tables);
		}

		cfg->num_entries = 0;
		return ret;
	}

	if (!cfg->linear)
		arm_smmu_write_cd_l1_desc(cfg->l1.ptr, leaf_table);

	/* SSID 0 corresponds to default context */
	set_bit(0, leaf_table->context_map);

	return nr_ssids;
}

static void arm_smmu_free_cd_tables(struct arm_smmu_master_data *master)
{
	size_t i, size;
	struct arm_smmu_device *smmu = master->smmu;
	struct arm_smmu_cd_cfg *cfg = &master->ste.cd_cfg;

	if (!cfg->num_entries)
		return;

	if (cfg->linear) {
		arm_smmu_free_cd_leaf_table(smmu, &cfg->table, cfg->num_entries);
	} else {
		for (i = 0; i < cfg->num_entries; i++) {
			struct arm_smmu_cd_table *desc = &cfg->l1.tables[i];

			if (!desc->cdptr)
				continue;

			arm_smmu_free_cd_leaf_table(smmu, desc,
						    CTXDESC_NUM_L2_ENTRIES);
		}

		size = cfg->num_entries * (CTXDESC_L1_DESC_DWORD << 3);
		dmam_free_coherent(smmu->dev, size, cfg->l1.ptr, cfg->l1.ptr_dma);

		devm_kfree(smmu->dev, cfg->l1.tables);
	}

	cfg->num_entries = 0;
}

static int arm_smmu_alloc_cd(struct arm_smmu_master_data *master)
{
	int ssid;
	int i, ret;
	struct arm_smmu_cd_cfg *cfg = &master->ste.cd_cfg;

	if (cfg->linear)
		return arm_smmu_bitmap_alloc(cfg->table.context_map,
					     ilog2(cfg->num_entries));

	/* Find first leaf table with an empty slot, or allocate a new leaf */
	for (i = cfg->l1.cur_table; i < cfg->num_entries; i++) {
		struct arm_smmu_cd_table *table = &cfg->l1.tables[i];

		if (!table->cdptr) {
			__le64 *l1ptr = cfg->l1.ptr + i * CTXDESC_L1_DESC_DWORD;

			ret = arm_smmu_alloc_cd_leaf_table(master->smmu, table,
							   CTXDESC_NUM_L2_ENTRIES);
			if (ret)
				return ret;

			arm_smmu_write_cd_l1_desc(l1ptr, table);
			arm_smmu_sync_cd(master, i << CTXDESC_SPLIT, false);
		}

		ssid = arm_smmu_bitmap_alloc(table->context_map, CTXDESC_SPLIT);
		if (ssid < 0)
			continue;

		cfg->l1.cur_table = i;
		return i << CTXDESC_SPLIT | ssid;
	}

	return -ENOSPC;
}

static void arm_smmu_free_cd(struct arm_smmu_master_data *master, u32 ssid)
{
	unsigned long l1_idx, idx;
	struct arm_smmu_cd_cfg *cfg = &master->ste.cd_cfg;

	if (cfg->linear) {
		arm_smmu_bitmap_free(cfg->table.context_map, ssid);
		return;
	}

	l1_idx = ssid >> CTXDESC_SPLIT;
	idx = ssid & ((1 << CTXDESC_SPLIT) - 1);
	arm_smmu_bitmap_free(cfg->l1.tables[l1_idx].context_map, idx);

	/* Prepare next allocation */
	if (cfg->l1.cur_table > idx)
		cfg->l1.cur_table = idx;
}

/* Stream table manipulation functions */
static void
arm_smmu_write_strtab_l1_desc(__le64 *dst, struct arm_smmu_strtab_l1_desc *desc)
{
	u64 val = 0;

	val |= (desc->span & STRTAB_L1_DESC_SPAN_MASK)
		<< STRTAB_L1_DESC_SPAN_SHIFT;
	val |= desc->l2ptr_dma &
	       STRTAB_L1_DESC_L2PTR_MASK << STRTAB_L1_DESC_L2PTR_SHIFT;

	*dst = cpu_to_le64(val);
}

static void arm_smmu_sync_ste_for_sid(struct arm_smmu_device *smmu, u32 sid)
{
	struct arm_smmu_cmdq_ent cmd = {
		.opcode	= CMDQ_OP_CFGI_STE,
		.cfgi	= {
			.sid	= sid,
			.leaf	= true,
		},
	};

	arm_smmu_cmdq_issue_cmd(smmu, &cmd);
	cmd.opcode = CMDQ_OP_CMD_SYNC;
	arm_smmu_cmdq_issue_cmd(smmu, &cmd);
}

static void arm_smmu_write_strtab_ent(struct arm_smmu_device *smmu, u32 sid,
				      __le64 *dst, struct arm_smmu_strtab_ent *ste)
{
	/*
	 * This is hideously complicated, but we only really care about
	 * three cases at the moment:
	 *
	 * 1. Invalid (all zero) -> bypass  (init)
	 * 2. Bypass -> translation (attach)
	 * 3. Translation -> bypass (detach)
	 *
	 * Given that we can't update the STE atomically and the SMMU
	 * doesn't read the thing in a defined order, that leaves us
	 * with the following maintenance requirements:
	 *
	 * 1. Update Config, return (init time STEs aren't live)
	 * 2. Write everything apart from dword 0, sync, write dword 0, sync
	 * 3. Update Config, sync
	 */
	u64 val = le64_to_cpu(dst[0]);
	bool ste_live = false;
	struct arm_smmu_cmdq_ent prefetch_cmd = {
		.opcode		= CMDQ_OP_PREFETCH_CFG,
		.prefetch	= {
			.sid	= sid,
		},
	};

	if (val & STRTAB_STE_0_V) {
		u64 cfg;

		cfg = val & STRTAB_STE_0_CFG_MASK << STRTAB_STE_0_CFG_SHIFT;
		switch (cfg) {
		case STRTAB_STE_0_CFG_BYPASS:
			break;
		case STRTAB_STE_0_CFG_S1_TRANS:
		case STRTAB_STE_0_CFG_S2_TRANS:
			ste_live = true;
			break;
		case STRTAB_STE_0_CFG_ABORT:
			if (disable_bypass)
				break;
		default:
			BUG(); /* STE corruption */
		}
	}

	/* Nuke the existing STE_0 value, as we're going to rewrite it */
	val = ste->valid ? STRTAB_STE_0_V : 0;

	if (ste->bypass) {
		val |= disable_bypass ? STRTAB_STE_0_CFG_ABORT
				      : STRTAB_STE_0_CFG_BYPASS;
		dst[0] = cpu_to_le64(val);
		dst[1] = cpu_to_le64(STRTAB_STE_1_SHCFG_INCOMING
			 << STRTAB_STE_1_SHCFG_SHIFT);
		dst[2] = 0; /* Nuke the VMID */
		if (ste_live)
			arm_smmu_sync_ste_for_sid(smmu, sid);
		return;
	}

	if (ste->s1_cfg) {
		dma_addr_t s1ctxptr;
		unsigned int s1cdmax = ilog2(ste->cd_cfg.num_entries);

		if (ste->cd_cfg.linear) {
			s1ctxptr = ste->cd_cfg.table.cdptr_dma;
		} else {
			s1cdmax += CTXDESC_SPLIT;
			s1ctxptr = ste->cd_cfg.l1.ptr_dma;
		}

		BUG_ON(ste_live);

		dst[1] = cpu_to_le64(
			 STRTAB_STE_1_S1DSS_SSID0 |
			 STRTAB_STE_1_S1C_CACHE_WBRA
			 << STRTAB_STE_1_S1CIR_SHIFT |
			 STRTAB_STE_1_S1C_CACHE_WBRA
			 << STRTAB_STE_1_S1COR_SHIFT |
			 STRTAB_STE_1_S1C_SH_ISH << STRTAB_STE_1_S1CSH_SHIFT |
			 (smmu->features & ARM_SMMU_FEAT_E2H ?
			  STRTAB_STE_1_STRW_EL2 : STRTAB_STE_1_STRW_NSEL1) <<
			 STRTAB_STE_1_STRW_SHIFT);

		if (ste->prg_response_needs_ssid)
			dst[1] |= STRTAB_STE_1_PPAR;

		if (smmu->features & ARM_SMMU_FEAT_STALLS)
			dst[1] |= cpu_to_le64(STRTAB_STE_1_S1STALLD);

		val |= (s1ctxptr & STRTAB_STE_0_S1CTXPTR_MASK
		        << STRTAB_STE_0_S1CTXPTR_SHIFT) |
			(u64)(s1cdmax & STRTAB_STE_0_S1CDMAX_MASK)
			<< STRTAB_STE_0_S1CDMAX_SHIFT |
			(ste->cd_cfg.linear ? STRTAB_STE_0_S1FMT_LINEAR :
			   STRTAB_STE_0_S1FMT_64K_L2) |
			STRTAB_STE_0_CFG_S1_TRANS;
	}

	if (ste->s2_cfg) {
		BUG_ON(ste_live);
		dst[2] = cpu_to_le64(
			 ste->s2_cfg->vmid << STRTAB_STE_2_S2VMID_SHIFT |
			 (ste->s2_cfg->vtcr & STRTAB_STE_2_VTCR_MASK)
			  << STRTAB_STE_2_VTCR_SHIFT |
#ifdef __BIG_ENDIAN
			 STRTAB_STE_2_S2ENDI |
#endif
			 STRTAB_STE_2_S2PTW | STRTAB_STE_2_S2AA64 |
			 STRTAB_STE_2_S2R);

		dst[3] = cpu_to_le64(ste->s2_cfg->vttbr &
			 STRTAB_STE_3_S2TTB_MASK << STRTAB_STE_3_S2TTB_SHIFT);

		val |= STRTAB_STE_0_CFG_S2_TRANS;
	}

	if (IS_ENABLED(CONFIG_PCI_ATS) && !ste_live)
		dst[1] |= cpu_to_le64(STRTAB_STE_1_EATS_TRANS
				      << STRTAB_STE_1_EATS_SHIFT);

	arm_smmu_sync_ste_for_sid(smmu, sid);
	dst[0] = cpu_to_le64(val);
	arm_smmu_sync_ste_for_sid(smmu, sid);

	/* It's likely that we'll want to use the new STE soon */
	if (!(smmu->options & ARM_SMMU_OPT_SKIP_PREFETCH))
		arm_smmu_cmdq_issue_cmd(smmu, &prefetch_cmd);
}

static void arm_smmu_init_bypass_stes(u64 *strtab, unsigned int nent)
{
	unsigned int i;
	struct arm_smmu_strtab_ent ste = {
		.valid	= true,
		.bypass	= true,
	};

	for (i = 0; i < nent; ++i) {
		arm_smmu_write_strtab_ent(NULL, -1, strtab, &ste);
		strtab += STRTAB_STE_DWORDS;
	}
}

static int arm_smmu_init_l2_strtab(struct arm_smmu_device *smmu, u32 sid)
{
	size_t size;
	void *strtab;
	struct arm_smmu_strtab_cfg *cfg = &smmu->strtab_cfg;
	struct arm_smmu_strtab_l1_desc *desc = &cfg->l1_desc[sid >> STRTAB_SPLIT];

	if (desc->l2ptr)
		return 0;

	size = 1 << (STRTAB_SPLIT + ilog2(STRTAB_STE_DWORDS) + 3);
	strtab = &cfg->strtab[(sid >> STRTAB_SPLIT) * STRTAB_L1_DESC_DWORDS];

	desc->span = STRTAB_SPLIT + 1;
	desc->l2ptr = dmam_alloc_coherent(smmu->dev, size, &desc->l2ptr_dma,
					  GFP_KERNEL | __GFP_ZERO);
	if (!desc->l2ptr) {
		dev_err(smmu->dev,
			"failed to allocate l2 stream table for SID %u\n",
			sid);
		return -ENOMEM;
	}

	arm_smmu_init_bypass_stes(desc->l2ptr, 1 << STRTAB_SPLIT);
	arm_smmu_write_strtab_l1_desc(strtab, desc);
	return 0;
}

/* IRQ and event handlers */
static irqreturn_t arm_smmu_evtq_thread(int irq, void *dev)
{
	int i;
	struct arm_smmu_device *smmu = dev;
	struct arm_smmu_queue *q = &smmu->evtq.q;
	u64 evt[EVTQ_ENT_DWORDS];

	do {
		while (!queue_remove_raw(q, evt)) {
			u8 id = evt[0] >> EVTQ_0_ID_SHIFT & EVTQ_0_ID_MASK;

			dev_info(smmu->dev, "event 0x%02x received:\n", id);
			for (i = 0; i < ARRAY_SIZE(evt); ++i)
				dev_info(smmu->dev, "\t0x%016llx\n",
					 (unsigned long long)evt[i]);

		}

		/*
		 * Not much we can do on overflow, so scream and pretend we're
		 * trying harder.
		 */
		if (queue_sync_prod(q) == -EOVERFLOW)
			dev_err(smmu->dev, "EVTQ overflow detected -- events lost\n");
	} while (!queue_empty(q));

	/* Sync our overflow flag, as we believe we're up to speed */
	queue_sync_cons_ovf(q);
	return IRQ_HANDLED;
}

static void arm_smmu_handle_fault(struct work_struct *work);

static void arm_smmu_handle_ppr(struct arm_smmu_device *smmu, u64 *evt)
{
	struct arm_smmu_fault *fault;
	struct arm_smmu_fault params = {
		.smmu	= smmu,

		.sid	= evt[0] >> PRIQ_0_SID_SHIFT & PRIQ_0_SID_MASK,
		.ssv	= evt[0] & PRIQ_0_SSID_V,
		.ssid	= evt[0] >> PRIQ_0_SSID_SHIFT & PRIQ_0_SSID_MASK,
		.last	= evt[0] & PRIQ_0_PRG_LAST,
		.grpid	= evt[1] >> PRIQ_1_PRG_IDX_SHIFT & PRIQ_1_PRG_IDX_MASK,

		.iova	= evt[1] & PRIQ_1_ADDR_MASK << PRIQ_1_ADDR_SHIFT,
		.read	= evt[0] & PRIQ_0_PERM_READ,
		.write	= evt[0] & PRIQ_0_PERM_WRITE,
		.exec	= evt[0] & PRIQ_0_PERM_EXEC,
		.priv	= evt[0] & PRIQ_0_PERM_PRIV,
	};

	fault = kmem_cache_alloc(arm_smmu_fault_cache, GFP_KERNEL);
	if (!fault) {
		/* Out of memory, tell the device to retry later */
		arm_smmu_fault_reply(&params, ARM_SMMU_FAULT_SUCC);
		return;
	}

	*fault = params;
	INIT_WORK(&fault->work, arm_smmu_handle_fault);
	queue_work(smmu->fault_queue, &fault->work);
}

static irqreturn_t arm_smmu_priq_thread(int irq, void *dev)
{
	struct arm_smmu_device *smmu = dev;
	struct arm_smmu_queue *q = &smmu->priq.q;
	size_t queue_size = 1 << q->max_n_shift;
	u64 evt[PRIQ_ENT_DWORDS];
	size_t i = 0;

	spin_lock(&smmu->priq.wq.lock);

	do {
		while (!queue_remove_raw(q, evt)) {
			spin_unlock(&smmu->priq.wq.lock);
			arm_smmu_handle_ppr(smmu, evt);
			spin_lock(&smmu->priq.wq.lock);
			if (++i == queue_size) {
				smmu->priq.batch++;
				wake_up_locked(&smmu->priq.wq);
				i = 0;
			}
		}

		if (queue_sync_prod(q) == -EOVERFLOW)
			dev_err(smmu->dev, "PRIQ overflow detected -- requests lost\n");
	} while (!queue_empty(q));

	/* Sync our overflow flag, as we believe we're up to speed */
	queue_sync_cons_ovf(q);

	smmu->priq.batch++;
	wake_up_locked(&smmu->priq.wq);

	spin_unlock(&smmu->priq.wq.lock);

	return IRQ_HANDLED;
}

static irqreturn_t arm_smmu_cmdq_sync_handler(int irq, void *dev)
{
	/* We don't actually use CMD_SYNC interrupts for anything */
	return IRQ_HANDLED;
}

static int arm_smmu_device_disable(struct arm_smmu_device *smmu);

static irqreturn_t arm_smmu_gerror_handler(int irq, void *dev)
{
	u32 gerror, gerrorn, active;
	struct arm_smmu_device *smmu = dev;

	gerror = readl_relaxed(smmu->base + ARM_SMMU_GERROR);
	gerrorn = readl_relaxed(smmu->base + ARM_SMMU_GERRORN);

	active = gerror ^ gerrorn;
	if (!(active & GERROR_ERR_MASK))
		return IRQ_NONE; /* No errors pending */

	dev_warn(smmu->dev,
		 "unexpected global error reported (0x%08x), this could be serious\n",
		 active);

	if (active & GERROR_SFM_ERR) {
		dev_err(smmu->dev, "device has entered Service Failure Mode!\n");
		arm_smmu_device_disable(smmu);
	}

	if (active & GERROR_MSI_GERROR_ABT_ERR)
		dev_warn(smmu->dev, "GERROR MSI write aborted\n");

	if (active & GERROR_MSI_PRIQ_ABT_ERR)
		dev_warn(smmu->dev, "PRIQ MSI write aborted\n");

	if (active & GERROR_MSI_EVTQ_ABT_ERR)
		dev_warn(smmu->dev, "EVTQ MSI write aborted\n");

	if (active & GERROR_MSI_CMDQ_ABT_ERR) {
		dev_warn(smmu->dev, "CMDQ MSI write aborted\n");
		arm_smmu_cmdq_sync_handler(irq, smmu->dev);
	}

	if (active & GERROR_PRIQ_ABT_ERR)
		dev_err(smmu->dev, "PRIQ write aborted -- events may have been lost\n");

	if (active & GERROR_EVTQ_ABT_ERR)
		dev_err(smmu->dev, "EVTQ write aborted -- events may have been lost\n");

	if (active & GERROR_CMDQ_ERR)
		arm_smmu_cmdq_skip_err(smmu);

	writel(gerror, smmu->base + ARM_SMMU_GERRORN);
	return IRQ_HANDLED;
}

/* IO_PGTABLE API */
static void __arm_smmu_tlb_sync(struct arm_smmu_device *smmu)
{
	struct arm_smmu_cmdq_ent cmd;

	cmd.opcode = CMDQ_OP_CMD_SYNC;
	arm_smmu_cmdq_issue_cmd(smmu, &cmd);
}

static void arm_smmu_tlb_sync(void *cookie)
{
	struct arm_smmu_domain *smmu_domain = cookie;
	__arm_smmu_tlb_sync(smmu_domain->smmu);
}

static void arm_smmu_tlb_inv_context(void *cookie)
{
	struct arm_smmu_domain *smmu_domain = cookie;
	struct arm_smmu_device *smmu = smmu_domain->smmu;
	struct arm_smmu_cmdq_ent cmd;

	if (smmu_domain->stage == ARM_SMMU_DOMAIN_S1) {
		cmd.opcode	= smmu->features & ARM_SMMU_FEAT_E2H ?
				  CMDQ_OP_TLBI_EL2_ASID : CMDQ_OP_TLBI_NH_ASID;
		cmd.tlbi.asid	= smmu_domain->s1_cfg.asid;
		cmd.tlbi.vmid	= 0;
	} else {
		cmd.opcode	= CMDQ_OP_TLBI_S12_VMALL;
		cmd.tlbi.vmid	= smmu_domain->s2_cfg.vmid;
	}

	arm_smmu_cmdq_issue_cmd(smmu, &cmd);
	__arm_smmu_tlb_sync(smmu);
}

static void arm_smmu_tlb_inv_range_nosync(unsigned long iova, size_t size,
					  size_t granule, bool leaf, void *cookie)
{
	struct arm_smmu_domain *smmu_domain = cookie;
	struct arm_smmu_device *smmu = smmu_domain->smmu;
	struct arm_smmu_cmdq_ent cmd = {
		.tlbi = {
			.leaf	= leaf,
			.addr	= iova,
		},
	};

	if (smmu_domain->stage == ARM_SMMU_DOMAIN_S1) {
		cmd.opcode	= smmu->features & ARM_SMMU_FEAT_E2H ?
				  CMDQ_OP_TLBI_EL2_VA : CMDQ_OP_TLBI_NH_VA;
		cmd.tlbi.asid	= smmu_domain->s1_cfg.asid;
	} else {
		cmd.opcode	= CMDQ_OP_TLBI_S2_IPA;
		cmd.tlbi.vmid	= smmu_domain->s2_cfg.vmid;
	}

	do {
		arm_smmu_cmdq_issue_cmd(smmu, &cmd);
		cmd.tlbi.addr += granule;
	} while (size -= granule);
}

static const struct iommu_gather_ops arm_smmu_gather_ops = {
	.tlb_flush_all	= arm_smmu_tlb_inv_context,
	.tlb_add_flush	= arm_smmu_tlb_inv_range_nosync,
	.tlb_sync	= arm_smmu_tlb_sync,
};

static void arm_smmu_atc_invalidate_to_cmd(struct arm_smmu_device *smmu,
					   unsigned long iova, size_t size,
					   struct arm_smmu_cmdq_ent *cmd)
{
	size_t log2_span;
	size_t span_mask;
	size_t smmu_grain;
	/* ATC invalidates are always on 4096 bytes pages */
	size_t inval_grain_shift = 12;
	unsigned long iova_start, iova_end;
	unsigned long page_start, page_end;

	smmu_grain	= 1ULL << __ffs(smmu->pgsize_bitmap);

	/* In case parameters are not aligned on PAGE_SIZE */
	iova_start	= round_down(iova, smmu_grain);
	iova_end	= round_up(iova + size, smmu_grain) - 1;

	page_start	= iova_start >> inval_grain_shift;
	page_end	= iova_end >> inval_grain_shift;

	/*
	 * Find the smallest power of two that covers the range. Most
	 * significant differing bit between start and end address indicates the
	 * required span, ie. fls(start ^ end). For example:
	 *
	 * We want to invalidate pages [8; 11]. This is already the ideal range:
	 *		x = 0b1000 ^ 0b1011 = 0b11
	 *		span = 1 << fls(x) = 4
	 *
	 * To invalidate pages [7; 10], we need to invalidate [0; 15]:
	 *		x = 0b0111 ^ 0b1010 = 0b1101
	 *		span = 1 << fls(x) = 16
	 */
	log2_span	= fls_long(page_start ^ page_end);
	span_mask	= (1ULL << log2_span) - 1;

	page_start	&= ~span_mask;

	*cmd = (struct arm_smmu_cmdq_ent) {
		.opcode	= CMDQ_OP_ATC_INV,
		.atc	= {
			.addr = page_start << inval_grain_shift,
			.size = log2_span,
		}
	};
}

static int arm_smmu_atc_invalidate_master(struct arm_smmu_master_data *master,
					  struct arm_smmu_cmdq_ent *cmd)
{
	int i;
	struct iommu_fwspec *fwspec = master->dev->iommu_fwspec;
	struct pci_dev *pdev = to_pci_dev(master->dev);

	if (!pdev->ats_enabled)
		return 0;

	for (i = 0; i < fwspec->num_ids; i++) {
		cmd->atc.sid = fwspec->ids[i];

		dev_dbg(master->smmu->dev,
			"ATC invalidate %#x:%#x:%#llx-%#llx, esz=%d\n",
			cmd->atc.sid, cmd->atc.ssid, cmd->atc.addr,
			cmd->atc.addr + (1 << (cmd->atc.size + 12)) - 1,
			cmd->atc.size);

		arm_smmu_cmdq_issue_cmd(master->smmu, cmd);
	}

	return 0;
}

static size_t arm_smmu_atc_invalidate_domain(struct arm_smmu_domain *smmu_domain,
					     unsigned long iova, size_t size)
{
	unsigned long flags;
	struct arm_smmu_cmdq_ent cmd = {0};
	struct arm_smmu_group *smmu_group;
	struct arm_smmu_master_data *master;
	struct arm_smmu_device *smmu = smmu_domain->smmu;
	struct arm_smmu_cmdq_ent sync_cmd = {
		.opcode = CMDQ_OP_CMD_SYNC,
	};

	spin_lock_irqsave(&smmu_domain->groups_lock, flags);

	list_for_each_entry(smmu_group, &smmu_domain->groups, domain_head) {
		if (!smmu_group->ats_enabled)
			continue;

		/* Initialise command lazily */
		if (!cmd.opcode)
			arm_smmu_atc_invalidate_to_cmd(smmu, iova, size, &cmd);

		spin_lock(&smmu_group->devices_lock);

		list_for_each_entry(master, &smmu_group->devices, group_head)
			arm_smmu_atc_invalidate_master(master, &cmd);

		/*
		 * TODO: ensure we do a sync whenever we have sent ats_queue_depth
		 * invalidations to the same device.
		 */
		arm_smmu_cmdq_issue_cmd(smmu, &sync_cmd);

		spin_unlock(&smmu_group->devices_lock);
	}

	spin_unlock_irqrestore(&smmu_domain->groups_lock, flags);

	return size;
}

static size_t arm_smmu_atc_invalidate_task(struct arm_smmu_task *smmu_task,
					   unsigned long iova, size_t size)
{
	struct arm_smmu_cmdq_ent cmd;
	struct arm_smmu_context *smmu_context;
	struct arm_smmu_device *smmu = smmu_task->smmu;
	struct arm_smmu_cmdq_ent sync_cmd = {
		.opcode = CMDQ_OP_CMD_SYNC,
	};

	arm_smmu_atc_invalidate_to_cmd(smmu, iova, size, &cmd);
	cmd.substream_valid = true;

	spin_lock(&smmu->contexts_lock);

	list_for_each_entry(smmu_context, &smmu_task->contexts, task_head) {
		cmd.atc.ssid = smmu_context->ssid;
		arm_smmu_atc_invalidate_master(smmu_context->master, &cmd);
	}

	spin_unlock(&smmu->contexts_lock);

	arm_smmu_cmdq_issue_cmd(smmu, &sync_cmd);

	return size;
}

static size_t arm_smmu_atc_invalidate_context(struct arm_smmu_context *smmu_context,
					      unsigned long iova, size_t size)
{
	struct arm_smmu_cmdq_ent cmd;
	struct arm_smmu_device *smmu = smmu_context->master->smmu;
	struct arm_smmu_cmdq_ent sync_cmd = {
		.opcode = CMDQ_OP_CMD_SYNC,
	};

	arm_smmu_atc_invalidate_to_cmd(smmu, iova, size, &cmd);

	cmd.substream_valid = true;
	cmd.atc.ssid = smmu_context->ssid;

	arm_smmu_atc_invalidate_master(smmu_context->master, &cmd);
	arm_smmu_cmdq_issue_cmd(smmu, &sync_cmd);

	return size;
}

/* IOMMU API */
static bool arm_smmu_capable(enum iommu_cap cap)
{
	switch (cap) {
	case IOMMU_CAP_CACHE_COHERENCY:
		return true;
	case IOMMU_CAP_NOEXEC:
		return true;
	default:
		return false;
	}
}

static struct arm_smmu_context *
arm_smmu_attach_task(struct arm_smmu_task *smmu_task,
		     struct arm_smmu_master_data *master)
{
	int ssid;
	int ret = 0;
	struct arm_smmu_context *smmu_context, *ctx;
	struct arm_smmu_device *smmu = master->smmu;
	struct rb_node **new_node, *parent_node = NULL;

	smmu_context = kzalloc(sizeof(*smmu_context), GFP_KERNEL);
	if (!smmu_context)
		return ERR_PTR(-ENOMEM);

	smmu_context->task = smmu_task;
	smmu_context->master = master;
	kref_init(&smmu_context->kref);

	spin_lock(&smmu->contexts_lock);

	/* Allocate a context descriptor and SSID */
	ssid = arm_smmu_alloc_cd(master);
	if (ssid <= 0) {
		if (WARN_ON_ONCE(ssid == 0))
			ret = -EEXIST;
		else
			ret = ssid;
		goto err_free_context;
	}

	smmu_context->ssid = ssid;

	arm_smmu_write_ctx_desc(master, ssid, &smmu_task->s1_cfg);

	list_add(&smmu_context->task_head, &smmu_task->contexts);

	/* Insert into master context list */
	new_node = &(master->contexts.rb_node);
	while (*new_node) {
		ctx = rb_entry(*new_node, struct arm_smmu_context,
			       master_node);
		parent_node = *new_node;
		if (ctx->ssid > ssid) {
			new_node = &((*new_node)->rb_left);
		} else if (ctx->ssid < ssid) {
			new_node = &((*new_node)->rb_right);
		} else {
			dev_warn(master->dev, "context %u already exists\n",
				 ctx->ssid);
			ret = -EEXIST;
			goto err_remove_context;
		}
	}

	rb_link_node(&smmu_context->master_node, parent_node, new_node);
	rb_insert_color(&smmu_context->master_node, &master->contexts);

	spin_unlock(&smmu->contexts_lock);

	return smmu_context;

err_remove_context:
	list_del(&smmu_context->task_head);
	arm_smmu_write_ctx_desc(master, ssid, NULL);
	arm_smmu_free_cd(master, ssid);

err_free_context:
	spin_unlock(&smmu->contexts_lock);

	kfree(smmu_context);

	return ERR_PTR(ret);
}

/* Caller must hold contexts_lock */
static void arm_smmu_free_context(struct kref *kref)
{
	struct arm_smmu_master_data *master;
	struct arm_smmu_context *smmu_context;

	smmu_context = container_of(kref, struct arm_smmu_context, kref);

	WARN_ON_ONCE(smmu_context->task);

	master = smmu_context->master;

	arm_smmu_free_cd(master, smmu_context->ssid);

	rb_erase(&smmu_context->master_node, &master->contexts);

	kfree(smmu_context);
}

static void _arm_smmu_put_context(struct arm_smmu_context *smmu_context)
{
	kref_put(&smmu_context->kref, arm_smmu_free_context);
}

static void arm_smmu_put_context(struct arm_smmu_device *smmu,
				 struct arm_smmu_context *smmu_context)
{
	spin_lock(&smmu->contexts_lock);
	_arm_smmu_put_context(smmu_context);
	spin_unlock(&smmu->contexts_lock);
}

/*
 * Find context associated to a (@sid, @ssid) pair. If found, take a reference
 * to the context and return it. Otherwise, return NULL. If a non-NULL master
 * is provided, search context by @ssid, ignoring argument @sid.
 */
static struct arm_smmu_context *
arm_smmu_get_context_by_id(struct arm_smmu_device *smmu,
			   struct arm_smmu_master_data *master,
			   u32 sid, u32 ssid)
{
	struct rb_node *node;
	struct arm_smmu_stream *stream;
	struct arm_smmu_context *cur_context, *smmu_context = NULL;

	spin_lock(&smmu->contexts_lock);

	if (!master) {
		node = smmu->streams.rb_node;
		while (node) {
			stream = rb_entry(node, struct arm_smmu_stream, node);
			if (stream->id < sid) {
				node = node->rb_right;
			} else if (stream->id > sid) {
				node = node->rb_left;
			} else {
				master = stream->master;
				break;
			}
		}
	}

	if (!master)
		goto out_unlock;

	node = master->contexts.rb_node;
	while (node) {
		cur_context = rb_entry(node, struct arm_smmu_context,
				       master_node);

		if (cur_context->ssid < ssid) {
			node = node->rb_right;
		} else if (cur_context->ssid > ssid) {
			node = node->rb_left;
		} else {
			smmu_context = cur_context;
			kref_get(&smmu_context->kref);
			break;
		}
	}

out_unlock:
	spin_unlock(&smmu->contexts_lock);

	return smmu_context;
}

static struct arm_smmu_task *mn_to_task(struct mmu_notifier *mn)
{
	return container_of(mn, struct arm_smmu_task, mmu_notifier);
}

static void arm_smmu_notifier_invalidate_range(struct mmu_notifier *mn,
					       struct mm_struct *mm,
					       unsigned long start,
					       unsigned long end)
{
	struct arm_smmu_task *smmu_task = mn_to_task(mn);

	arm_smmu_atc_invalidate_task(smmu_task, start, end - start);
}

static void arm_smmu_notifier_invalidate_page(struct mmu_notifier *mn,
					      struct mm_struct *mm,
					      unsigned long address)
{
	arm_smmu_notifier_invalidate_range(mn, mm, address, address + PAGE_SIZE);
}

static int arm_smmu_notifier_clear_flush_young(struct mmu_notifier *mn,
					       struct mm_struct *mm,
					       unsigned long start,
					       unsigned long end)
{
	arm_smmu_notifier_invalidate_range(mn, mm, start, end);

	return 0;
}

static const struct mmu_notifier_ops arm_smmu_mmu_notifier_ops = {
	.invalidate_page	= arm_smmu_notifier_invalidate_page,
	.invalidate_range	= arm_smmu_notifier_invalidate_range,
	.clear_flush_young	= arm_smmu_notifier_clear_flush_young,
};

static int arm_smmu_context_share(struct arm_smmu_task *smmu_task, int asid)
{
	int ret = 0;
	int new_asid;
	unsigned long flags;
	struct arm_smmu_group *smmu_group;
	struct arm_smmu_master_data *master;
	struct arm_smmu_device *smmu = smmu_task->smmu;
	struct arm_smmu_domain *tmp_domain, *smmu_domain = NULL;
	struct arm_smmu_cmdq_ent cmd = {
		.opcode = smmu->features & ARM_SMMU_FEAT_E2H ?
			  CMDQ_OP_TLBI_EL2_ASID : CMDQ_OP_TLBI_NH_ASID,
	};

	mutex_lock(&smmu->domains_mutex);

	if (!test_and_set_bit(asid, smmu->asid_map))
		goto out_unlock;

	/* ASID is used by a domain. Try to replace it with a new one. */
	new_asid = arm_smmu_bitmap_alloc(smmu->asid_map, smmu->asid_bits);
	if (new_asid < 0) {
		ret = new_asid;
		goto out_unlock;
	}

	list_for_each_entry(tmp_domain, &smmu->domains, list) {
		if (tmp_domain->stage != ARM_SMMU_DOMAIN_S1 ||
		    tmp_domain->s1_cfg.asid != asid)
			continue;

		smmu_domain = tmp_domain;
		break;
	}

	/*
	 * We didn't find the domain that owns this ASID. It is a bug, since we
	 * hold domains_mutex
	 */
	if (WARN_ON(!smmu_domain)) {
		ret = -ENOSPC;
		goto out_unlock;
	}

	/*
	 * Race with smmu_unmap; TLB invalidations will start targeting the
	 * new ASID, which isn't assigned yet. We'll do an invalidate-all on
	 * the old ASID later, so it doesn't matter.
	 */
	smmu_domain->s1_cfg.asid = new_asid;

	/*
	 * Update ASID and invalidate CD in all associated masters. There will
	 * be some overlapping between use of both ASIDs, until we invalidate
	 * the TLB.
	 */
	spin_lock_irqsave(&smmu_domain->groups_lock, flags);

	list_for_each_entry(smmu_group, &smmu_domain->groups, domain_head) {
		spin_lock(&smmu_group->devices_lock);
		list_for_each_entry(master, &smmu_group->devices, group_head) {
			arm_smmu_write_ctx_desc(master, 0, &smmu_domain->s1_cfg);
		}
		spin_unlock(&smmu_group->devices_lock);
	}

	spin_unlock_irqrestore(&smmu_domain->groups_lock, flags);

	/* Invalidate TLB entries previously associated with that domain */
	cmd.tlbi.asid = asid;
	arm_smmu_cmdq_issue_cmd(smmu, &cmd);
	cmd.opcode = CMDQ_OP_CMD_SYNC;
	arm_smmu_cmdq_issue_cmd(smmu, &cmd);

out_unlock:
	mutex_unlock(&smmu->domains_mutex);

	return ret;
}

static int arm_smmu_init_task_pgtable(struct arm_smmu_task *smmu_task)
{
	int ret;
	int asid;
	unsigned long tcr;
	unsigned long reg, par;
	struct arm_smmu_s1_cfg *cfg = &smmu_task->s1_cfg;

	/* Pin ASID on the CPU side */
	asid = mm_context_get(smmu_task->mm);
	if (!asid)
		return -ENOSPC;

	ret = arm_smmu_context_share(smmu_task, asid);
	if (ret) {
		mm_context_put(smmu_task->mm);
		return ret;
	}

	tcr = TCR_T0SZ(VA_BITS) | TCR_IRGN0_WBWA | TCR_ORGN0_WBWA |
		TCR_SH0_INNER | ARM_LPAE_TCR_EPD1;

	switch (PAGE_SIZE) {
	case SZ_4K:
		tcr |= TCR_TG0_4K;
		break;
	case SZ_16K:
		tcr |= TCR_TG0_16K;
		break;
	case SZ_64K:
		tcr |= TCR_TG0_64K;
		break;
	default:
		WARN_ON(1);
		return -EFAULT;
	}

	reg = read_system_reg(SYS_ID_AA64MMFR0_EL1);
	par = cpuid_feature_extract_unsigned_field(reg, ID_AA64MMFR0_PARANGE_SHIFT);
	tcr |= par << ARM_LPAE_TCR_IPS_SHIFT;

	/* Enable this by default, it will be filtered when writing the CD */
	tcr |= TCR_TBI0;

	cfg->asid	= asid;
	cfg->ttbr	= virt_to_phys(smmu_task->mm->pgd);
	/*
	 * MAIR value is pretty much constant and global, so we can just get it
	 * from the current CPU register
	 */
	cfg->mair	= read_sysreg(mair_el1);
	cfg->tcr	= tcr;

	return 0;
}

static void arm_smmu_free_task_pgtable(struct arm_smmu_task *smmu_task)
{
	struct arm_smmu_device *smmu = smmu_task->smmu;

	mm_context_put(smmu_task->mm);

	arm_smmu_bitmap_free(smmu->asid_map, smmu_task->s1_cfg.asid);
}

static struct arm_smmu_task *arm_smmu_alloc_task(struct arm_smmu_device *smmu,
						 struct task_struct *task)
{
	int ret;
	struct mm_struct *mm;
	struct arm_smmu_task *smmu_task;

	mm = get_task_mm(task);
	if (!mm)
		return ERR_PTR(-EINVAL);

	smmu_task = kzalloc(sizeof(*smmu_task), GFP_KERNEL);
	if (!smmu_task) {
		ret = -ENOMEM;
		goto err_put_mm;
	}

	smmu_task->smmu = smmu;
	smmu_task->pid = get_task_pid(task, PIDTYPE_PID);
	smmu_task->mmu_notifier.ops = &arm_smmu_mmu_notifier_ops;
	smmu_task->mm = mm;
	INIT_LIST_HEAD(&smmu_task->contexts);
	INIT_LIST_HEAD(&smmu_task->prgs);
	kref_init(&smmu_task->kref);

	ret = arm_smmu_init_task_pgtable(smmu_task);
	if (ret)
		goto err_free_task;

	/*
	 * TODO: check conflicts between task mappings and reserved HW
	 * mappings. It is unclear which reserved mappings might be affected
	 * because, for instance, devices are unlikely to send MSIs tagged with
	 * PASIDs so we (probably) don't need to carve out MSI regions from the
	 * task address space. Clarify this.
	 */

	ret = mmu_notifier_register(&smmu_task->mmu_notifier, mm);
	if (ret)
		goto err_free_pgtable;

	spin_lock(&smmu->contexts_lock);
	list_add(&smmu_task->smmu_head, &smmu->tasks);
	spin_unlock(&smmu->contexts_lock);

	/* A reference to mm is kept by the notifier */
	mmput(mm);

	return smmu_task;

err_free_pgtable:
	arm_smmu_free_task_pgtable(smmu_task);

err_free_task:
	put_pid(smmu_task->pid);
	kfree(smmu_task);

err_put_mm:
	mmput(mm);

	return ERR_PTR(ret);
}

/* Caller must hold contexts_lock */
static void arm_smmu_free_task(struct kref *kref)
{
	struct arm_smmu_device *smmu;
	struct arm_smmu_task *smmu_task;
	struct arm_smmu_master_data *master;
	struct arm_smmu_pri_group *prg, *next_prg;
	struct arm_smmu_context *smmu_context, *next;

	smmu_task = container_of(kref, struct arm_smmu_task, kref);
	smmu = smmu_task->smmu;

	if (WARN_ON_ONCE(!list_empty(&smmu_task->contexts))) {
		list_for_each_entry_safe(smmu_context, next,
					 &smmu_task->contexts, task_head) {
			master = smmu_context->master;

			arm_smmu_write_ctx_desc(master, smmu_context->ssid, NULL);
			smmu_context->task = NULL;
			list_del(&smmu_context->task_head);
		}
	}

	list_del(&smmu_task->smmu_head);

	/*
	 * Release the lock temporarily to unregister the notifier. This is safe
	 * because the task is not accessible anymore.
	 */
	spin_unlock(&smmu->contexts_lock);

	/* Unpin ASID */
	arm_smmu_free_task_pgtable(smmu_task);

	mmu_notifier_unregister(&smmu_task->mmu_notifier, smmu_task->mm);

	list_for_each_entry_safe(prg, next_prg, &smmu_task->prgs, list)
		list_del(&prg->list);

	put_pid(smmu_task->pid);
	kfree(smmu_task);

	spin_lock(&smmu->contexts_lock);
}

static void _arm_smmu_put_task(struct arm_smmu_task *smmu_task)
{
	kref_put(&smmu_task->kref, arm_smmu_free_task);
}

/* Caller must hold contexts_lock */
static void arm_smmu_detach_task(struct arm_smmu_context *smmu_context)
{
	struct arm_smmu_task *smmu_task = smmu_context->task;

	smmu_context->task = NULL;
	list_del(&smmu_context->task_head);
	_arm_smmu_put_task(smmu_task);

	arm_smmu_write_ctx_desc(smmu_context->master, smmu_context->ssid, NULL);
}

static void arm_smmu_put_task(struct arm_smmu_device *smmu,
			      struct arm_smmu_task *smmu_task)
{
	spin_lock(&smmu->contexts_lock);
	_arm_smmu_put_task(smmu_task);
	spin_unlock(&smmu->contexts_lock);
}

static int arm_smmu_handle_mm_fault(struct arm_smmu_device *smmu,
				    struct mm_struct *mm,
				    struct arm_smmu_fault *fault)
{
	int ret;
	struct vm_area_struct *vma;
	unsigned long access_flags = 0;
	unsigned long fault_flags = FAULT_FLAG_USER | FAULT_FLAG_REMOTE;

	/*
	 * We're holding smmu_task, which holds the mmu notifier, so mm is
	 * guaranteed to be here, but mm_users might still drop to zero when
	 * the task exits.
	 */
	if (!mmget_not_zero(mm)) {
		dev_dbg(smmu->dev, "mm dead\n");
		return -EINVAL;
	}

	down_read(&mm->mmap_sem);

	vma = find_extend_vma(mm, fault->iova);
	if (!vma) {
		ret = -ESRCH;
		dev_dbg(smmu->dev, "VMA not found\n");
		goto out_release;
	}

	if (fault->read)
		access_flags |= VM_READ;

	if (fault->write) {
		access_flags |= VM_WRITE;
		fault_flags |= FAULT_FLAG_WRITE;
	}

	if (fault->exec) {
		access_flags |= VM_EXEC;
		fault_flags |= FAULT_FLAG_INSTRUCTION;
	}

	if (access_flags & ~vma->vm_flags) {
		ret = -EFAULT;
		dev_dbg(smmu->dev, "access flags mismatch\n");
		goto out_release;
	}

	ret = handle_mm_fault(vma, fault->iova, fault_flags);
	dev_dbg(smmu->dev, "handle_mm_fault(%#x:%#x:%#llx, %#lx) -> %#x\n",
		fault->sid, fault->ssid, fault->iova, fault_flags, ret);

	ret = ret & VM_FAULT_ERROR ? -EFAULT : 0;

out_release:
	up_read(&mm->mmap_sem);
	mmput(mm);

	return ret;
}

static enum fault_status _arm_smmu_handle_fault(struct arm_smmu_fault *fault)
{
	struct arm_smmu_task *smmu_task = NULL;
	struct arm_smmu_device *smmu = fault->smmu;
	struct arm_smmu_context *smmu_context = NULL;
	enum fault_status resp = ARM_SMMU_FAULT_FAIL;
	struct arm_smmu_pri_group *prg = NULL, *tmp_prg;

	if (!fault->ssv)
		return ARM_SMMU_FAULT_DENY;

	if (fault->priv)
		return resp;

	smmu_context = arm_smmu_get_context_by_id(smmu, NULL, fault->sid,
						  fault->ssid);
	if (!smmu_context) {
		dev_dbg(smmu->dev, "unable to find context %#x:%#x\n",
			fault->sid, fault->ssid);
		/*
		 * Note that we don't have prg_response_needs_ssid yet. Reply
		 * might be inconsistent with what the device expects.
		 */
		return resp;
	}

	if (fault->last && !fault->read && !fault->write) {
		/* Special case: stop marker invalidates the PASID */
		u64 val = atomic64_fetch_or(ARM_SMMU_CONTEXT_INVALIDATED,
					    &smmu_context->state);
		if (val == ARM_SMMU_CONTEXT_STALE) {
			spin_lock(&smmu->contexts_lock);
			_arm_smmu_put_context(smmu_context);
			smmu_context->master->stale_contexts--;
			spin_unlock(&smmu->contexts_lock);
		}

		/* No reply expected */
		resp = ARM_SMMU_FAULT_IGNORE;
		goto out_put_context;
	}

	fault->ssv = smmu_context->master->ste.prg_response_needs_ssid;

	spin_lock(&smmu->contexts_lock);
	smmu_task = smmu_context->task;
	if (smmu_task)
		kref_get(&smmu_task->kref);
	spin_unlock(&smmu->contexts_lock);

	if (!smmu_task)
		/* Stale context */
		goto out_put_context;

	list_for_each_entry(tmp_prg, &smmu_task->prgs, list) {
		if (tmp_prg->index == fault->grpid) {
			prg = tmp_prg;
			break;
		}
	}

	if (!prg && !fault->last) {
		prg = kzalloc(sizeof(*prg), GFP_KERNEL);
		if (!prg) {
			resp = ARM_SMMU_FAULT_SUCC;
			goto out_put_task;
		}

		prg->index = fault->grpid;
		list_add(&prg->list, &smmu_task->prgs);
	} else if (prg && prg->resp != ARM_SMMU_FAULT_SUCC) {
		resp = prg->resp;
		goto out_put_task;
	}

	if (!arm_smmu_handle_mm_fault(smmu, smmu_task->mm, fault))
		resp = ARM_SMMU_FAULT_SUCC;

	if (prg) {
		if (fault->last) {
			list_del(&prg->list);
			kfree(prg);
		} else {
			prg->resp = resp;
		}
	}

out_put_task:
	arm_smmu_put_task(smmu, smmu_task);

out_put_context:
	arm_smmu_put_context(smmu, smmu_context);

	return resp;
}

static void arm_smmu_handle_fault(struct work_struct *work)
{
	enum fault_status resp;
	struct arm_smmu_fault *fault = container_of(work, struct arm_smmu_fault,
						    work);

	resp = _arm_smmu_handle_fault(fault);
	if (resp != ARM_SMMU_FAULT_SUCC && resp != ARM_SMMU_FAULT_IGNORE)
		dev_info_ratelimited(fault->smmu->dev, "%s fault:\n"
			"\t0x%08x.0x%05x: [%u%s] %sprivileged %s%s%s access at iova "
			"0x%016llx\n",
			resp == ARM_SMMU_FAULT_DENY ? "unexpected" : "unhandled",
			fault->sid, fault->ssid, fault->grpid,
			fault->last ? "L" : "", fault->priv ? "" : "un",
			fault->read ? "R" : "", fault->write ? "W" : "",
			fault->exec ? "X" : "", fault->iova);

	arm_smmu_fault_reply(fault, resp);

	kfree(fault);
}

static void arm_smmu_sweep_contexts(struct work_struct *work)
{
	u64 batch;
	int ret, i = 0;
	struct arm_smmu_priq *priq;
	struct arm_smmu_device *smmu;
	struct arm_smmu_master_data *master;
	struct arm_smmu_context *smmu_context, *tmp;
	struct list_head flush_list = LIST_HEAD_INIT(flush_list);

	master = container_of(work, struct arm_smmu_master_data, sweep_contexts);
	smmu = master->smmu;
	priq = &smmu->priq;

	spin_lock(&smmu->contexts_lock);
	dev_dbg(smmu->dev, "Sweeping contexts %u/%u\n",
		master->stale_contexts, master->avail_contexts);

	rbtree_postorder_for_each_entry_safe(smmu_context, tmp,
					     &master->contexts, master_node) {
		u64 val = atomic64_cmpxchg(&smmu_context->state,
					   ARM_SMMU_CONTEXT_STALE,
					   ARM_SMMU_CONTEXT_FREE);
		if (val != ARM_SMMU_CONTEXT_STALE)
			continue;

		/*
		 * We volunteered for deleting this context by setting the state
		 * atomically. This guarantees that no one else writes to its
		 * flush_head field.
		 */
		list_add(&smmu_context->flush_head, &flush_list);
	}
	spin_unlock(&smmu->contexts_lock);

	if (list_empty(&flush_list))
		return;

	/*
	 * Now wait until the priq thread finishes a batch, or until the queue
	 * is empty. After that, we are certain that the last references to this
	 * context have been flushed to the fault work queue. Note that we don't
	 * handle overflows on priq->batch. If it occurs, just wait for the
	 * queue to be empty.
	 */
	spin_lock(&priq->wq.lock);
	if (queue_sync_prod(&priq->q) == -EOVERFLOW)
		dev_err(smmu->dev, "PRIQ overflow detected -- requests lost\n");
	batch = priq->batch;
	ret = wait_event_interruptible_locked(priq->wq, queue_empty(&priq->q) ||
					      priq->batch >= batch + 2);
	spin_unlock(&priq->wq.lock);

	if (ret) {
		/* Woops, rollback. */
		spin_lock(&smmu->contexts_lock);
		list_for_each_entry(smmu_context, &flush_list, flush_head)
			atomic64_xchg(&smmu_context->state,
				      ARM_SMMU_CONTEXT_STALE);
		spin_unlock(&smmu->contexts_lock);
		return;
	}

	flush_workqueue(smmu->fault_queue);

	spin_lock(&smmu->contexts_lock);
	list_for_each_entry_safe(smmu_context, tmp, &flush_list, flush_head) {
		_arm_smmu_put_context(smmu_context);
		i++;
	}

	master->stale_contexts -= i;
	spin_unlock(&smmu->contexts_lock);
}

static bool arm_smmu_master_supports_svm(struct arm_smmu_master_data *master)
{
	return dev_is_pci(master->dev) && master->can_fault &&
		master->avail_contexts;
}

static int arm_smmu_set_svm_ops(struct device *dev,
				const struct iommu_svm_ops *svm_ops)
{
	struct arm_smmu_master_data *master;

	if (!dev->iommu_fwspec)
		return -EINVAL;

	master = dev->iommu_fwspec->iommu_priv;
	if (!master)
		return -EINVAL;

	master->svm_ops = svm_ops;

	return 0;
}

static int arm_smmu_invalidate_context(struct arm_smmu_context *smmu_context)
{
	struct arm_smmu_master_data *master = smmu_context->master;

	if (!master->svm_ops || !master->svm_ops->invalidate_pasid)
		return 0;

	return master->svm_ops->invalidate_pasid(master->dev,
						 smmu_context->ssid,
						 smmu_context->priv);
}

static int arm_smmu_bind_task(struct device *dev, struct task_struct *task,
			      int *pasid, int flags, void *priv)
{
	int ret = 0;
	struct pid *pid;
	struct iommu_group *group;
	struct arm_smmu_device *smmu;
	struct arm_smmu_group *smmu_group;
	struct arm_smmu_domain *smmu_domain;
	struct arm_smmu_master_data *master;
	struct arm_smmu_task *smmu_task = NULL, *cur_task;
	struct arm_smmu_context *smmu_context = NULL, *cur_context;

	if (!dev->iommu_fwspec)
		return -EINVAL;

	master = dev->iommu_fwspec->iommu_priv;
	if (!master)
		return -EINVAL;

	if (!arm_smmu_master_supports_svm(master))
		return -EINVAL;

	smmu = master->smmu;

	group = iommu_group_get(dev);
	smmu_group = to_smmu_group(group);

	smmu_domain = smmu_group->domain;
	if (!smmu_domain) {
		iommu_group_put(group);
		return -EINVAL;
	}

	if (smmu_domain->stage != ARM_SMMU_DOMAIN_S1) {
		/* We do not support stage-2 SVM yet... */
		iommu_group_put(group);
		return -ENOSYS;
	}

	iommu_group_put(group);

	pid = get_task_pid(task, PIDTYPE_PID);

	spin_lock(&smmu->contexts_lock);

	list_for_each_entry(cur_task, &smmu->tasks, smmu_head) {
		if (cur_task->pid == pid) {
			kref_get(&cur_task->kref);
			smmu_task = cur_task;
			break;
		}
	}

	if (smmu_task) {
		list_for_each_entry(cur_context, &smmu_task->contexts,
				    task_head) {
			if (cur_context->master->dev == dev) {
				smmu_context = cur_context;
				_arm_smmu_put_task(cur_task);
				break;
			}
		}
	}
	spin_unlock(&smmu->contexts_lock);

	put_pid(pid);

	if (smmu_context)
		/* We don't support nested bind/unbind calls */
		return -EEXIST;

	if (!smmu_task) {
		smmu_task = arm_smmu_alloc_task(smmu, task);
		if (IS_ERR(smmu_task))
			return -PTR_ERR(smmu_task);
	}

	smmu_context = arm_smmu_attach_task(smmu_task, master);
	if (IS_ERR(smmu_context)) {
		arm_smmu_put_task(smmu, smmu_task);
		return PTR_ERR(smmu_context);
	}

	smmu_context->priv = priv;

	*pasid = smmu_context->ssid;
	dev_dbg(dev, "bound to task %d with PASID %d\n", pid_vnr(pid), *pasid);

	return ret;
}

static int arm_smmu_unbind_task(struct device *dev, int pasid, int flags)
{
	int ret;
	unsigned long val;
	unsigned int pasid_state;
	bool put_context = false;
	struct arm_smmu_device *smmu;
	struct arm_smmu_master_data *master;
	struct arm_smmu_context *smmu_context = NULL;

	if (!dev->iommu_fwspec)
		return -EINVAL;

	master = dev->iommu_fwspec->iommu_priv;
	if (!master)
		return -EINVAL;

	smmu = master->smmu;

	smmu_context = arm_smmu_get_context_by_id(smmu, master, 0, pasid);
	if (!smmu_context)
		return -ESRCH;

	dev_dbg(dev, "unbind PASID %d\n", pasid);

	pasid_state = flags & (IOMMU_PASID_FLUSHED | IOMMU_PASID_CLEAN);
	if (!pasid_state)
		pasid_state = arm_smmu_invalidate_context(smmu_context);

	if (!pasid_state) {
		/* PASID is in use, we can't do anything. */
		ret = -EBUSY;
		goto err_put_context;
	}

	/*
	 * There isn't any "ATC invalidate all by PASID" command. If this isn't
	 * good enough, we'll need fine-grained invalidation for each vma.
	 */
	arm_smmu_atc_invalidate_context(smmu_context, 0, -1);

	val = atomic64_fetch_or(ARM_SMMU_CONTEXT_STALE, &smmu_context->state);
	if (val == ARM_SMMU_CONTEXT_INVALIDATED || !master->can_fault) {
		/* We already received a stop marker for this context. */
		put_context = true;
	} else if (pasid_state & IOMMU_PASID_CLEAN) {
		/* We are allowed to free the PASID now! */
		val = atomic64_fetch_or(ARM_SMMU_CONTEXT_INVALIDATED,
					&smmu_context->state);
		if (val == ARM_SMMU_CONTEXT_STALE)
			put_context = true;
	}

	spin_lock(&smmu->contexts_lock);
	if (smmu_context->task)
		arm_smmu_detach_task(smmu_context);

	/* Release the ref we got earlier in this function */
	_arm_smmu_put_context(smmu_context);

	if (put_context)
		_arm_smmu_put_context(smmu_context);
	else if (++master->stale_contexts >= STALE_CONTEXTS_LIMIT(master))
		queue_work(system_long_wq, &master->sweep_contexts);
	spin_unlock(&smmu->contexts_lock);

	return 0;

err_put_context:
	arm_smmu_put_context(smmu, smmu_context);

	return ret;
}

static struct iommu_domain *arm_smmu_domain_alloc(unsigned type)
{
	struct arm_smmu_domain *smmu_domain;

	if (type != IOMMU_DOMAIN_UNMANAGED && type != IOMMU_DOMAIN_DMA)
		return NULL;

	/*
	 * Allocate the domain and initialise some of its data structures.
	 * We can't really do anything meaningful until we've added a
	 * master.
	 */
	smmu_domain = kzalloc(sizeof(*smmu_domain), GFP_KERNEL);
	if (!smmu_domain)
		return NULL;

	if (type == IOMMU_DOMAIN_DMA &&
	    iommu_get_dma_cookie(&smmu_domain->domain)) {
		kfree(smmu_domain);
		return NULL;
	}

	mutex_init(&smmu_domain->init_mutex);
	spin_lock_init(&smmu_domain->pgtbl_lock);
	INIT_LIST_HEAD(&smmu_domain->groups);
	spin_lock_init(&smmu_domain->groups_lock);

	return &smmu_domain->domain;
}

static void arm_smmu_domain_free(struct iommu_domain *domain)
{
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct arm_smmu_device *smmu = smmu_domain->smmu;

	iommu_put_dma_cookie(domain);
	free_io_pgtable_ops(smmu_domain->pgtbl_ops);

	mutex_lock(&smmu->domains_mutex);

	if (smmu_domain->stage == ARM_SMMU_DOMAIN_S1) {
		struct arm_smmu_s1_cfg *cfg = &smmu_domain->s1_cfg;
		if (cfg->asid) {
			arm_smmu_bitmap_free(smmu->asid_map, cfg->asid);

			list_del(&smmu_domain->list);
		}
	} else {
		struct arm_smmu_s2_cfg *cfg = &smmu_domain->s2_cfg;
		if (cfg->vmid)
			arm_smmu_bitmap_free(smmu->vmid_map, cfg->vmid);
	}

	mutex_unlock(&smmu->domains_mutex);

	kfree(smmu_domain);
}

static int arm_smmu_domain_finalise_s1(struct arm_smmu_domain *smmu_domain,
				       struct io_pgtable_cfg *pgtbl_cfg)
{
	int asid;
	struct arm_smmu_device *smmu = smmu_domain->smmu;
	struct arm_smmu_s1_cfg *cfg = &smmu_domain->s1_cfg;

	asid = arm_smmu_bitmap_alloc(smmu->asid_map, smmu->asid_bits);
	if (asid < 0)
		return asid;

	cfg->asid	= (u16)asid;
	cfg->ttbr	= pgtbl_cfg->arm_lpae_s1_cfg.ttbr[0];
	cfg->tcr	= pgtbl_cfg->arm_lpae_s1_cfg.tcr;
	cfg->mair	= pgtbl_cfg->arm_lpae_s1_cfg.mair[0];

	list_add(&smmu_domain->list, &smmu->domains);

	return 0;
}

static int arm_smmu_domain_finalise_s2(struct arm_smmu_domain *smmu_domain,
				       struct io_pgtable_cfg *pgtbl_cfg)
{
	int vmid;
	struct arm_smmu_device *smmu = smmu_domain->smmu;
	struct arm_smmu_s2_cfg *cfg = &smmu_domain->s2_cfg;

	vmid = arm_smmu_bitmap_alloc(smmu->vmid_map, smmu->vmid_bits);
	if (vmid < 0)
		return vmid;

	cfg->vmid	= (u16)vmid;
	cfg->vttbr	= pgtbl_cfg->arm_lpae_s2_cfg.vttbr;
	cfg->vtcr	= pgtbl_cfg->arm_lpae_s2_cfg.vtcr;
	return 0;
}

static int arm_smmu_domain_finalise(struct iommu_domain *domain)
{
	int ret;
	unsigned long ias, oas;
	enum io_pgtable_fmt fmt;
	struct io_pgtable_cfg pgtbl_cfg;
	struct io_pgtable_ops *pgtbl_ops;
	int (*finalise_stage_fn)(struct arm_smmu_domain *,
				 struct io_pgtable_cfg *);
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct arm_smmu_device *smmu = smmu_domain->smmu;

	/* Restrict the stage to what we can actually support */
	if (!(smmu->features & ARM_SMMU_FEAT_TRANS_S1))
		smmu_domain->stage = ARM_SMMU_DOMAIN_S2;
	if (!(smmu->features & ARM_SMMU_FEAT_TRANS_S2))
		smmu_domain->stage = ARM_SMMU_DOMAIN_S1;

	switch (smmu_domain->stage) {
	case ARM_SMMU_DOMAIN_S1:
		ias = VA_BITS;
		oas = smmu->ias;
		fmt = ARM_64_LPAE_S1;
		finalise_stage_fn = arm_smmu_domain_finalise_s1;
		break;
	case ARM_SMMU_DOMAIN_NESTED:
	case ARM_SMMU_DOMAIN_S2:
		ias = smmu->ias;
		oas = smmu->oas;
		fmt = ARM_64_LPAE_S2;
		finalise_stage_fn = arm_smmu_domain_finalise_s2;
		break;
	default:
		return -EINVAL;
	}

	pgtbl_cfg = (struct io_pgtable_cfg) {
		.pgsize_bitmap	= smmu->pgsize_bitmap,
		.ias		= ias,
		.oas		= oas,
		.tlb		= &arm_smmu_gather_ops,
		.iommu_dev	= smmu->dev,
	};

	pgtbl_ops = alloc_io_pgtable_ops(fmt, &pgtbl_cfg, smmu_domain);
	if (!pgtbl_ops)
		return -ENOMEM;

	domain->pgsize_bitmap = pgtbl_cfg.pgsize_bitmap;
	domain->geometry.aperture_end = (1UL << ias) - 1;
	domain->geometry.force_aperture = true;
	smmu_domain->pgtbl_ops = pgtbl_ops;

	ret = finalise_stage_fn(smmu_domain, &pgtbl_cfg);
	if (ret < 0)
		free_io_pgtable_ops(pgtbl_ops);

	return ret;
}

static __le64 *arm_smmu_get_step_for_sid(struct arm_smmu_device *smmu, u32 sid)
{
	__le64 *step;
	struct arm_smmu_strtab_cfg *cfg = &smmu->strtab_cfg;

	if (smmu->features & ARM_SMMU_FEAT_2_LVL_STRTAB) {
		struct arm_smmu_strtab_l1_desc *l1_desc;
		int idx;

		/* Two-level walk */
		idx = (sid >> STRTAB_SPLIT) * STRTAB_L1_DESC_DWORDS;
		l1_desc = &cfg->l1_desc[idx];
		idx = (sid & ((1 << STRTAB_SPLIT) - 1)) * STRTAB_STE_DWORDS;
		step = &l1_desc->l2ptr[idx];
	} else {
		/* Simple linear lookup */
		step = &cfg->strtab[sid * STRTAB_STE_DWORDS];
	}

	return step;
}

static int arm_smmu_install_ste_for_dev(struct iommu_fwspec *fwspec)
{
	int i;
	struct arm_smmu_master_data *master = fwspec->iommu_priv;
	struct arm_smmu_device *smmu = master->smmu;

	for (i = 0; i < fwspec->num_ids; ++i) {
		u32 sid = fwspec->ids[i];
		__le64 *step = arm_smmu_get_step_for_sid(smmu, sid);

		arm_smmu_write_strtab_ent(smmu, sid, step, &master->ste);
	}

	return 0;
}

static void arm_smmu_group_release(void *smmu_group)
{
	kfree(smmu_group);
}

static struct arm_smmu_group *arm_smmu_group_alloc(struct iommu_group *group)
{
	struct arm_smmu_group *smmu_group = to_smmu_group(group);

	if (smmu_group)
		return smmu_group;

	smmu_group = kzalloc(sizeof(*smmu_group), GFP_KERNEL);
	if (!smmu_group)
		return NULL;

	INIT_LIST_HEAD(&smmu_group->devices);
	spin_lock_init(&smmu_group->devices_lock);

	iommu_group_set_iommudata(group, smmu_group, arm_smmu_group_release);

	return smmu_group;
}

static void arm_smmu_detach_dev(struct device *dev)
{
	struct arm_smmu_master_data *master = dev->iommu_fwspec->iommu_priv;
	struct arm_smmu_device *smmu = master->smmu;
	struct arm_smmu_context *smmu_context;
	struct rb_node *node, *next;
	int new_stale_contexts = 0;

	mutex_lock(&smmu->domains_mutex);

	master->ste.bypass = true;
	if (arm_smmu_install_ste_for_dev(dev->iommu_fwspec) < 0)
		dev_warn(dev, "failed to install bypass STE\n");

	arm_smmu_write_ctx_desc(master, 0, NULL);

	mutex_unlock(&smmu->domains_mutex);

	if (!master->ste.valid)
		return;

	/* Try to clean the contexts. */
	spin_lock(&smmu->contexts_lock);
	for (node = rb_first(&master->contexts); node; node = next) {
		u64 val;
		int pasid_state = 0;

		smmu_context = rb_entry(node, struct arm_smmu_context,
					master_node);
		next = rb_next(node);

		val = atomic64_fetch_or(ARM_SMMU_CONTEXT_STALE,
					&smmu_context->state);
		if (val == ARM_SMMU_CONTEXT_FREE)
			/* Someone else is waiting to free this context */
			continue;

		if (!(val & ARM_SMMU_CONTEXT_STALE)) {
			pasid_state = arm_smmu_invalidate_context(smmu_context);
			if (!pasid_state) {
				/*
				 * This deserves a slap, since there still
				 * might be references to that PASID hanging
				 * around downstream of the SMMU and we can't
				 * do anything about it.
				 */
				dev_warn(dev, "PASID %u was still bound!\n",
					 smmu_context->ssid);
			}

			if (smmu_context->task)
				arm_smmu_detach_task(smmu_context);
			else
				dev_warn(dev, "bound without a task?!");

			new_stale_contexts++;
		}

		if (!(val & ARM_SMMU_CONTEXT_INVALIDATED) && master->can_fault &&
		    !(pasid_state & IOMMU_PASID_CLEAN)) {
			/*
			 * We can't free the context yet, its PASID might still
			 * be waiting in the pipe.
			 */
			continue;
		}

		val = atomic64_fetch_or(ARM_SMMU_CONTEXT_INVALIDATED,
					&smmu_context->state);
		if (val == ARM_SMMU_CONTEXT_FREE)
			continue;

		_arm_smmu_put_context(smmu_context);
		new_stale_contexts--;
	}

	master->stale_contexts += new_stale_contexts;
	if (master->stale_contexts)
		queue_work(system_long_wq, &master->sweep_contexts);
	spin_unlock(&smmu->contexts_lock);
}

static int arm_smmu_attach_dev(struct iommu_domain *domain, struct device *dev)
{
	int ret = 0;
	unsigned long flags;
	struct iommu_group *group;
	struct arm_smmu_device *smmu;
	struct arm_smmu_group *smmu_group;
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct arm_smmu_master_data *master;
	struct arm_smmu_strtab_ent *ste;

	if (!dev->iommu_fwspec)
		return -ENOENT;

	master = dev->iommu_fwspec->iommu_priv;
	smmu = master->smmu;
	ste = &master->ste;

	/*
	 * When adding devices, this is the first occasion we have to create the
	 * smmu_group and attach it to iommu_group.
	 */
	group = iommu_group_get(dev);
	smmu_group = arm_smmu_group_alloc(group);
	if (!smmu_group) {
		iommu_group_put(group);
		return -ENOMEM;
	}

	/*
	 * Already attached to a different domain? This happens when we're
	 * switching from default domain to unmanaged domain, and back. We
	 * assume here that, when switching from old domain to new domain, old
	 * domain doesn't have any live mapping anymore. This is an important
	 * requirement because here we remove the group-domain link when we
	 * re-attach the first device in a group. Other devices in that group
	 * might still be attached to the old domain, and will be reattached in
	 * a moment.
	 *
	 * We also take this path when attaching for the very first time, just
	 * after the STE is initialized.
	 */
	if (!ste->bypass) {
		struct arm_smmu_domain *other_domain = smmu_group->domain;

		if (other_domain) {
			spin_lock_irqsave(&other_domain->groups_lock, flags);
			list_del(&smmu_group->domain_head);
			spin_unlock_irqrestore(&other_domain->groups_lock, flags);

			smmu_group->domain = NULL;
		}
		arm_smmu_detach_dev(dev);
	}

	mutex_lock(&smmu->domains_mutex);
	mutex_lock(&smmu_domain->init_mutex);

	if (!smmu_domain->smmu) {
		smmu_domain->smmu = smmu;
		ret = arm_smmu_domain_finalise(domain);
		if (ret) {
			smmu_domain->smmu = NULL;
			goto out_unlock;
		}
	} else if (smmu_domain->smmu != smmu) {
		dev_err(dev,
			"cannot attach to SMMU %s (upstream of %s)\n",
			dev_name(smmu_domain->smmu->dev),
			dev_name(smmu->dev));
		ret = -ENXIO;
		goto out_unlock;
	}

	if (!smmu_group->domain) {
		smmu_group->domain = smmu_domain;

		spin_lock_irqsave(&smmu_domain->groups_lock, flags);
		list_add(&smmu_group->domain_head, &smmu_domain->groups);
		spin_unlock_irqrestore(&smmu_domain->groups_lock, flags);
	}

	ste->bypass = false;
	ste->valid = true;

	if (smmu_domain->stage == ARM_SMMU_DOMAIN_S1) {
		ste->s1_cfg = &smmu_domain->s1_cfg;
		ste->s2_cfg = NULL;
		arm_smmu_write_ctx_desc(master, 0, ste->s1_cfg);
	} else {
		ste->s1_cfg = NULL;
		ste->s2_cfg = &smmu_domain->s2_cfg;
	}

	ret = arm_smmu_install_ste_for_dev(dev->iommu_fwspec);
	if (ret < 0)
		ste->valid = false;

out_unlock:
	mutex_unlock(&smmu_domain->init_mutex);
	mutex_unlock(&smmu->domains_mutex);

	iommu_group_put(group);

	return ret;
}

static int arm_smmu_map(struct iommu_domain *domain, unsigned long iova,
			phys_addr_t paddr, size_t size, int prot)
{
	int ret;
	unsigned long flags;
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct io_pgtable_ops *ops = smmu_domain->pgtbl_ops;

	if (!ops)
		return -ENODEV;

	spin_lock_irqsave(&smmu_domain->pgtbl_lock, flags);
	ret = ops->map(ops, iova, paddr, size, prot);
	spin_unlock_irqrestore(&smmu_domain->pgtbl_lock, flags);
	return ret;
}

static size_t
arm_smmu_unmap(struct iommu_domain *domain, unsigned long iova, size_t size)
{
	size_t ret;
	unsigned long flags;
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct io_pgtable_ops *ops = smmu_domain->pgtbl_ops;

	if (!ops)
		return 0;

	spin_lock_irqsave(&smmu_domain->pgtbl_lock, flags);
	ret = ops->unmap(ops, iova, size);
	if (ret)
		ret = arm_smmu_atc_invalidate_domain(smmu_domain, iova, size);
	spin_unlock_irqrestore(&smmu_domain->pgtbl_lock, flags);

	return ret;
}

static phys_addr_t
arm_smmu_iova_to_phys(struct iommu_domain *domain, dma_addr_t iova)
{
	phys_addr_t ret;
	unsigned long flags;
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);
	struct io_pgtable_ops *ops = smmu_domain->pgtbl_ops;

	if (!ops)
		return 0;

	spin_lock_irqsave(&smmu_domain->pgtbl_lock, flags);
	ret = ops->iova_to_phys(ops, iova);
	spin_unlock_irqrestore(&smmu_domain->pgtbl_lock, flags);

	return ret;
}

static struct platform_driver arm_smmu_driver;

static int arm_smmu_match_node(struct device *dev, void *data)
{
	return dev->fwnode == data;
}

static
struct arm_smmu_device *arm_smmu_get_by_fwnode(struct fwnode_handle *fwnode)
{
	struct device *dev = driver_find_device(&arm_smmu_driver.driver, NULL,
						fwnode, arm_smmu_match_node);
	put_device(dev);
	return dev ? dev_get_drvdata(dev) : NULL;
}

static bool arm_smmu_sid_in_range(struct arm_smmu_device *smmu, u32 sid)
{
	unsigned long limit = smmu->strtab_cfg.num_l1_ents;

	if (smmu->features & ARM_SMMU_FEAT_2_LVL_STRTAB)
		limit *= 1UL << STRTAB_SPLIT;

	return sid < limit;
}

/*
 * Returns -ENOSYS if ATS is not supported either by the device or by the SMMU
 */
static int arm_smmu_enable_ats(struct arm_smmu_master_data *master)
{
	int ret;
	size_t stu;
	struct pci_dev *pdev;
	struct arm_smmu_device *smmu = master->smmu;

	if (!(smmu->features & ARM_SMMU_FEAT_ATS) || !dev_is_pci(master->dev))
		return -ENOSYS;

	pdev = to_pci_dev(master->dev);

#ifdef CONFIG_PCI_ATS
	if (!pdev->ats_cap)
		return -ENOSYS;
#else
	return -ENOSYS;
#endif

	/* Smallest Translation Unit: log2 of the smallest supported granule */
	stu = __ffs(smmu->pgsize_bitmap);

	ret = pci_enable_ats(pdev, stu);
	if (ret) {
		dev_err(&pdev->dev, "cannot enable ATS: %d\n", ret);
		return ret;
	}

	dev_dbg(&pdev->dev, "enabled ATS with STU = %zu\n", stu);

	return 0;
}

static void arm_smmu_disable_ats(struct arm_smmu_master_data *master)
{
	struct pci_dev *pdev;

	if (!dev_is_pci(master->dev))
		return;

	pdev = to_pci_dev(master->dev);

	if (!pdev->ats_enabled)
		return;

	pci_disable_ats(pdev);
}

static int arm_smmu_enable_ssid(struct arm_smmu_master_data *master)
{
	int ret;
	int features;
	int nr_ssids;
	struct pci_dev *pdev;

	if (!dev_is_pci(master->dev))
		return -ENOSYS;

	pdev = to_pci_dev(master->dev);

	features = pci_pasid_features(pdev);
	if (features < 0)
		return -ENOSYS;

	nr_ssids = pci_max_pasids(pdev);

	dev_dbg(&pdev->dev, "device supports %#x SSIDs [%s%s]\n", nr_ssids,
		(features & PCI_PASID_CAP_EXEC) ? "x" : "",
		(features & PCI_PASID_CAP_PRIV) ? "p" : "");

	ret = pci_enable_pasid(pdev, features);
	return ret ? ret : nr_ssids;
}

static void arm_smmu_disable_ssid(struct arm_smmu_master_data *master)
{
	struct pci_dev *pdev;

	if (!dev_is_pci(master->dev))
		return;

	pdev = to_pci_dev(master->dev);

	if (!pdev->pasid_enabled)
		return;

	pci_disable_pasid(pdev);
}

static int arm_smmu_enable_pri(struct arm_smmu_master_data *master)
{
	int ret, pos;
	struct pci_dev *pdev;
	size_t max_requests = 64;
	struct arm_smmu_device *smmu = master->smmu;

	/* Do not enable PRI if SVM isn't supported */
	unsigned long feat_mask = ARM_SMMU_FEAT_PRI | ARM_SMMU_FEAT_SVM;

	if ((smmu->features & feat_mask) != feat_mask || !dev_is_pci(master->dev))
		return -ENOSYS;

	pdev = to_pci_dev(master->dev);

	pos = pci_find_ext_capability(pdev, PCI_EXT_CAP_ID_PRI);
	if (!pos)
		return -ENOSYS;

	ret = pci_reset_pri(pdev);
	if (ret)
		return ret;

	ret = pci_enable_pri(pdev, max_requests);
	if (ret) {
		dev_err(master->dev, "cannot enable PRI: %d\n", ret);
		return ret;
	}

	master->can_fault = true;
	master->ste.prg_response_needs_ssid = pci_prg_resp_requires_prefix(pdev);

	dev_dbg(master->dev, "enabled PRI");

	return 0;
}

static void arm_smmu_disable_pri(struct arm_smmu_master_data *master)
{
	struct pci_dev *pdev;

	if (!master->can_fault || !dev_is_pci(master->dev))
		return;

	pdev = to_pci_dev(master->dev);

	pci_disable_pri(pdev);

	master->can_fault = false;
}

static int arm_smmu_insert_master(struct arm_smmu_device *smmu,
				  struct arm_smmu_master_data *master)
{
	int i;
	int ret = 0;
	struct arm_smmu_stream *new_stream, *cur_stream;
	struct rb_node **new_node, *parent_node = NULL;
	struct iommu_fwspec *fwspec = master->dev->iommu_fwspec;

	master->streams = kcalloc(fwspec->num_ids,
				  sizeof(struct arm_smmu_stream), GFP_KERNEL);
	if (!master->streams)
		return -ENOMEM;

	spin_lock(&smmu->contexts_lock);
	for (i = 0; i < fwspec->num_ids && !ret; i++) {
		new_stream = &master->streams[i];
		new_stream->id = fwspec->ids[i];
		new_stream->master = master;

		new_node = &(smmu->streams.rb_node);
		while (*new_node) {
			cur_stream = rb_entry(*new_node, struct arm_smmu_stream,
					      node);
			parent_node = *new_node;
			if (cur_stream->id > new_stream->id) {
				new_node = &((*new_node)->rb_left);
			} else if (cur_stream->id < new_stream->id) {
				new_node = &((*new_node)->rb_right);
			} else {
				dev_warn(master->dev,
					 "stream %u already in tree\n",
					 cur_stream->id);
				ret = -EINVAL;
				break;
			}
		}

		if (!ret) {
			rb_link_node(&new_stream->node, parent_node, new_node);
			rb_insert_color(&new_stream->node, &smmu->streams);
		}
	}
	spin_unlock(&smmu->contexts_lock);

	return ret;
}

static struct iommu_ops arm_smmu_ops;

static int arm_smmu_add_device(struct device *dev)
{
	int i, ret;
	int nr_ssids;
	bool ats_enabled;
	unsigned long flags;
	struct arm_smmu_device *smmu;
	struct arm_smmu_group *smmu_group;
	struct arm_smmu_master_data *master;
	struct iommu_fwspec *fwspec = dev->iommu_fwspec;
	struct iommu_group *group;

	if (!fwspec || fwspec->ops != &arm_smmu_ops)
		return -ENODEV;
	/*
	 * We _can_ actually withstand dodgy bus code re-calling add_device()
	 * without an intervening remove_device()/of_xlate() sequence, but
	 * we're not going to do so quietly...
	 */
	if (WARN_ON_ONCE(fwspec->iommu_priv)) {
		master = fwspec->iommu_priv;
		smmu = master->smmu;
	} else {
		smmu = arm_smmu_get_by_fwnode(fwspec->iommu_fwnode);
		if (!smmu)
			return -ENODEV;
		master = kzalloc(sizeof(*master), GFP_KERNEL);
		if (!master)
			return -ENOMEM;

		master->smmu = smmu;
		master->dev = dev;
		fwspec->iommu_priv = master;

		master->contexts = RB_ROOT;

		INIT_WORK(&master->sweep_contexts, arm_smmu_sweep_contexts);
	}

	/* Check the SIDs are in range of the SMMU and our stream table */
	for (i = 0; i < fwspec->num_ids; i++) {
		u32 sid = fwspec->ids[i];

		if (!arm_smmu_sid_in_range(smmu, sid))
			return -ERANGE;

		/* Ensure l2 strtab is initialised */
		if (smmu->features & ARM_SMMU_FEAT_2_LVL_STRTAB) {
			ret = arm_smmu_init_l2_strtab(smmu, sid);
			if (ret)
				return ret;
		}
	}

	/* PCIe PASID must be enabled before ATS */
	nr_ssids = arm_smmu_enable_ssid(master);
	if (nr_ssids <= 0)
		nr_ssids = 1;

	nr_ssids = arm_smmu_alloc_cd_tables(master, nr_ssids);
	if (nr_ssids < 0) {
		ret = nr_ssids;
		goto err_disable_ssid;
	}

	/* SSID0 is reserved */
	master->avail_contexts = nr_ssids - 1;

	ats_enabled = !arm_smmu_enable_ats(master);
	if (ats_enabled)
		arm_smmu_enable_pri(master);

	if (arm_smmu_master_supports_svm(master))
		arm_smmu_insert_master(smmu, master);

	group = iommu_group_get_for_dev(dev);
	if (IS_ERR(group)) {
		ret = PTR_ERR(group);
		goto err_disable_ats;
	}

	smmu_group = to_smmu_group(group);

	smmu_group->ats_enabled |= ats_enabled;

	spin_lock_irqsave(&smmu_group->devices_lock, flags);
	list_add(&master->group_head, &smmu_group->devices);
	spin_unlock_irqrestore(&smmu_group->devices_lock, flags);

	iommu_group_put(group);
	iommu_device_link(&smmu->iommu, dev);

	return 0;

err_disable_ats:
	arm_smmu_disable_pri(master);
	arm_smmu_disable_ats(master);

	arm_smmu_free_cd_tables(master);

err_disable_ssid:
	arm_smmu_disable_ssid(master);

	return ret;
}

static void arm_smmu_remove_device(struct device *dev)
{
	struct iommu_fwspec *fwspec = dev->iommu_fwspec;
	struct arm_smmu_context *smmu_context;
	struct arm_smmu_master_data *master;
	struct arm_smmu_group *smmu_group;
	struct arm_smmu_device *smmu;
	struct rb_node *node, *next;
	struct iommu_group *group;
	unsigned long flags;
	u64 val;
	int i;

	if (!fwspec || fwspec->ops != &arm_smmu_ops)
		return;

	master = fwspec->iommu_priv;
	smmu = master->smmu;
	if (master && master->ste.valid)
		arm_smmu_detach_dev(dev);

	if (master) {
		cancel_work_sync(&master->sweep_contexts);

		spin_lock(&smmu->contexts_lock);

		for (node = rb_first(&master->contexts); node; node = next) {
			smmu_context = rb_entry(node, struct arm_smmu_context,
						master_node);
			next = rb_next(node);

			/*
			 * Force removal of remaining contexts. They were marked
			 * stale by detach_dev, but haven't been invalidated
			 * since. Page requests might be pending but we can't
			 * afford to wait for them anymore. Bad things will
			 * happen.
			 */
			dev_warn(dev, "PASID %u wasn't invalidated\n",
				 smmu_context->ssid);
			val = atomic64_xchg(&smmu_context->state,
					    ARM_SMMU_CONTEXT_FREE);
			if (val != ARM_SMMU_CONTEXT_FREE)
				_arm_smmu_put_context(smmu_context);
		}

		if (master->streams) {
			for (i = 0; i < fwspec->num_ids; i++)
				rb_erase(&master->streams[i].node,
					 &smmu->streams);

			kfree(master->streams);
		}

		spin_unlock(&smmu->contexts_lock);

		group = iommu_group_get(dev);
		smmu_group = to_smmu_group(group);

		spin_lock_irqsave(&smmu_group->devices_lock, flags);
		list_del(&master->group_head);
		spin_unlock_irqrestore(&smmu_group->devices_lock, flags);

		iommu_group_put(group);

		arm_smmu_disable_pri(master);
		/* PCIe PASID must be disabled after ATS */
		arm_smmu_disable_ats(master);
		arm_smmu_disable_ssid(master);

		arm_smmu_free_cd_tables(master);
	}

	iommu_group_remove_device(dev);
	iommu_device_unlink(&smmu->iommu, dev);
	kfree(master);
	iommu_fwspec_free(dev);
}

static struct iommu_group *arm_smmu_device_group(struct device *dev)
{
	struct iommu_group *group;

	/*
	 * We don't support devices sharing stream IDs other than PCI RID
	 * aliases, since the necessary ID-to-device lookup becomes rather
	 * impractical given a potential sparse 32-bit stream ID space.
	 */
	if (dev_is_pci(dev))
		group = pci_device_group(dev);
	else
		group = generic_device_group(dev);

	return group;
}

static int arm_smmu_domain_get_attr(struct iommu_domain *domain,
				    enum iommu_attr attr, void *data)
{
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);

	switch (attr) {
	case DOMAIN_ATTR_NESTING:
		*(int *)data = (smmu_domain->stage == ARM_SMMU_DOMAIN_NESTED);
		return 0;
	default:
		return -ENODEV;
	}
}

static int arm_smmu_domain_set_attr(struct iommu_domain *domain,
				    enum iommu_attr attr, void *data)
{
	int ret = 0;
	struct arm_smmu_domain *smmu_domain = to_smmu_domain(domain);

	mutex_lock(&smmu_domain->init_mutex);

	switch (attr) {
	case DOMAIN_ATTR_NESTING:
		if (smmu_domain->smmu) {
			ret = -EPERM;
			goto out_unlock;
		}

		if (*(int *)data)
			smmu_domain->stage = ARM_SMMU_DOMAIN_NESTED;
		else
			smmu_domain->stage = ARM_SMMU_DOMAIN_S1;

		break;
	default:
		ret = -ENODEV;
	}

out_unlock:
	mutex_unlock(&smmu_domain->init_mutex);
	return ret;
}

static int arm_smmu_of_xlate(struct device *dev, struct of_phandle_args *args)
{
	return iommu_fwspec_add_ids(dev, args->args, 1);
}

static void arm_smmu_get_resv_regions(struct device *dev,
				      struct list_head *head)
{
	struct iommu_resv_region *region;
	int prot = IOMMU_WRITE | IOMMU_NOEXEC | IOMMU_MMIO;

	region = iommu_alloc_resv_region(MSI_IOVA_BASE, MSI_IOVA_LENGTH,
					 prot, IOMMU_RESV_MSI);
	if (!region)
		return;

	list_add_tail(&region->list, head);
}

static void arm_smmu_put_resv_regions(struct device *dev,
				      struct list_head *head)
{
	struct iommu_resv_region *entry, *next;

	list_for_each_entry_safe(entry, next, head, list)
		kfree(entry);
}

static struct iommu_ops arm_smmu_ops = {
	.capable		= arm_smmu_capable,
	.domain_alloc		= arm_smmu_domain_alloc,
	.domain_free		= arm_smmu_domain_free,
	.set_svm_ops		= arm_smmu_set_svm_ops,
	.bind_task		= arm_smmu_bind_task,
	.unbind_task		= arm_smmu_unbind_task,
	.attach_dev		= arm_smmu_attach_dev,
	.map			= arm_smmu_map,
	.unmap			= arm_smmu_unmap,
	.map_sg			= default_iommu_map_sg,
	.iova_to_phys		= arm_smmu_iova_to_phys,
	.add_device		= arm_smmu_add_device,
	.remove_device		= arm_smmu_remove_device,
	.device_group		= arm_smmu_device_group,
	.domain_get_attr	= arm_smmu_domain_get_attr,
	.domain_set_attr	= arm_smmu_domain_set_attr,
	.of_xlate		= arm_smmu_of_xlate,
	.get_resv_regions	= arm_smmu_get_resv_regions,
	.put_resv_regions	= arm_smmu_put_resv_regions,
	.pgsize_bitmap		= -1UL, /* Restricted during device attach */
};

/* Probing and initialisation functions */
static int arm_smmu_init_one_queue(struct arm_smmu_device *smmu,
				   struct arm_smmu_queue *q,
				   unsigned long prod_off,
				   unsigned long cons_off,
				   size_t dwords)
{
	size_t qsz = ((1 << q->max_n_shift) * dwords) << 3;

	q->base = dmam_alloc_coherent(smmu->dev, qsz, &q->base_dma, GFP_KERNEL);
	if (!q->base) {
		dev_err(smmu->dev, "failed to allocate queue (0x%zx bytes)\n",
			qsz);
		return -ENOMEM;
	}

	q->prod_reg	= smmu->base + prod_off;
	q->cons_reg	= smmu->base + cons_off;
	q->ent_dwords	= dwords;

	q->q_base  = Q_BASE_RWA;
	q->q_base |= q->base_dma & Q_BASE_ADDR_MASK << Q_BASE_ADDR_SHIFT;
	q->q_base |= (q->max_n_shift & Q_BASE_LOG2SIZE_MASK)
		     << Q_BASE_LOG2SIZE_SHIFT;

	q->prod = q->cons = 0;
	return 0;
}

static int arm_smmu_init_queues(struct arm_smmu_device *smmu)
{
	int ret;

	/* cmdq */
	spin_lock_init(&smmu->cmdq.lock);
	ret = arm_smmu_init_one_queue(smmu, &smmu->cmdq.q, ARM_SMMU_CMDQ_PROD,
				      ARM_SMMU_CMDQ_CONS, CMDQ_ENT_DWORDS);
	if (ret)
		return ret;

	/* evtq */
	ret = arm_smmu_init_one_queue(smmu, &smmu->evtq.q, ARM_SMMU_EVTQ_PROD,
				      ARM_SMMU_EVTQ_CONS, EVTQ_ENT_DWORDS);
	if (ret)
		return ret;

	/* priq */
	if (!(smmu->features & ARM_SMMU_FEAT_PRI))
		return 0;

	init_waitqueue_head(&smmu->priq.wq);
	smmu->priq.batch = 0;

	return arm_smmu_init_one_queue(smmu, &smmu->priq.q, ARM_SMMU_PRIQ_PROD,
				       ARM_SMMU_PRIQ_CONS, PRIQ_ENT_DWORDS);
}

static int arm_smmu_init_l1_strtab(struct arm_smmu_device *smmu)
{
	unsigned int i;
	struct arm_smmu_strtab_cfg *cfg = &smmu->strtab_cfg;
	size_t size = sizeof(*cfg->l1_desc) * cfg->num_l1_ents;
	void *strtab = smmu->strtab_cfg.strtab;

	cfg->l1_desc = devm_kzalloc(smmu->dev, size, GFP_KERNEL);
	if (!cfg->l1_desc) {
		dev_err(smmu->dev, "failed to allocate l1 stream table desc\n");
		return -ENOMEM;
	}

	for (i = 0; i < cfg->num_l1_ents; ++i) {
		arm_smmu_write_strtab_l1_desc(strtab, &cfg->l1_desc[i]);
		strtab += STRTAB_L1_DESC_DWORDS << 3;
	}

	return 0;
}

static int arm_smmu_init_strtab_2lvl(struct arm_smmu_device *smmu)
{
	void *strtab;
	u64 reg;
	u32 size, l1size;
	struct arm_smmu_strtab_cfg *cfg = &smmu->strtab_cfg;

	/* Calculate the L1 size, capped to the SIDSIZE. */
	size = STRTAB_L1_SZ_SHIFT - (ilog2(STRTAB_L1_DESC_DWORDS) + 3);
	size = min(size, smmu->sid_bits - STRTAB_SPLIT);
	cfg->num_l1_ents = 1 << size;

	size += STRTAB_SPLIT;
	if (size < smmu->sid_bits)
		dev_warn(smmu->dev,
			 "2-level strtab only covers %u/%u bits of SID\n",
			 size, smmu->sid_bits);

	l1size = cfg->num_l1_ents * (STRTAB_L1_DESC_DWORDS << 3);
	strtab = dmam_alloc_coherent(smmu->dev, l1size, &cfg->strtab_dma,
				     GFP_KERNEL | __GFP_ZERO);
	if (!strtab) {
		dev_err(smmu->dev,
			"failed to allocate l1 stream table (%u bytes)\n",
			size);
		return -ENOMEM;
	}
	cfg->strtab = strtab;

	/* Configure strtab_base_cfg for 2 levels */
	reg  = STRTAB_BASE_CFG_FMT_2LVL;
	reg |= (size & STRTAB_BASE_CFG_LOG2SIZE_MASK)
		<< STRTAB_BASE_CFG_LOG2SIZE_SHIFT;
	reg |= (STRTAB_SPLIT & STRTAB_BASE_CFG_SPLIT_MASK)
		<< STRTAB_BASE_CFG_SPLIT_SHIFT;
	cfg->strtab_base_cfg = reg;

	return arm_smmu_init_l1_strtab(smmu);
}

static int arm_smmu_init_strtab_linear(struct arm_smmu_device *smmu)
{
	void *strtab;
	u64 reg;
	u32 size;
	struct arm_smmu_strtab_cfg *cfg = &smmu->strtab_cfg;

	size = (1 << smmu->sid_bits) * (STRTAB_STE_DWORDS << 3);
	strtab = dmam_alloc_coherent(smmu->dev, size, &cfg->strtab_dma,
				     GFP_KERNEL | __GFP_ZERO);
	if (!strtab) {
		dev_err(smmu->dev,
			"failed to allocate linear stream table (%u bytes)\n",
			size);
		return -ENOMEM;
	}
	cfg->strtab = strtab;
	cfg->num_l1_ents = 1 << smmu->sid_bits;

	/* Configure strtab_base_cfg for a linear table covering all SIDs */
	reg  = STRTAB_BASE_CFG_FMT_LINEAR;
	reg |= (smmu->sid_bits & STRTAB_BASE_CFG_LOG2SIZE_MASK)
		<< STRTAB_BASE_CFG_LOG2SIZE_SHIFT;
	cfg->strtab_base_cfg = reg;

	arm_smmu_init_bypass_stes(strtab, cfg->num_l1_ents);
	return 0;
}

static int arm_smmu_init_strtab(struct arm_smmu_device *smmu)
{
	u64 reg;
	int ret;

	if (smmu->features & ARM_SMMU_FEAT_2_LVL_STRTAB)
		ret = arm_smmu_init_strtab_2lvl(smmu);
	else
		ret = arm_smmu_init_strtab_linear(smmu);

	if (ret)
		return ret;

	/* Set the strtab base address */
	reg  = smmu->strtab_cfg.strtab_dma &
	       STRTAB_BASE_ADDR_MASK << STRTAB_BASE_ADDR_SHIFT;
	reg |= STRTAB_BASE_RA;
	smmu->strtab_cfg.strtab_base = reg;

	/* Allocate the first VMID for stage-2 bypass STEs */
	set_bit(0, smmu->vmid_map);
	return 0;
}

static int arm_smmu_init_structures(struct arm_smmu_device *smmu)
{
	int ret;

	mutex_init(&smmu->domains_mutex);
	spin_lock_init(&smmu->contexts_lock);
	smmu->streams = RB_ROOT;
	INIT_LIST_HEAD(&smmu->tasks);
	INIT_LIST_HEAD(&smmu->domains);

	ret = arm_smmu_init_queues(smmu);
	if (ret)
		return ret;

	if (smmu->features & ARM_SMMU_FEAT_SVM &&
	    smmu->features & ARM_SMMU_FEAT_PRI) {
		/*
		 * Ensure strict ordering of the queue. We can't go reordering
		 * page faults willy nilly since they work in groups, with a
		 * flag "last" denoting when we should send a PRI response.
		 */
		smmu->fault_queue = alloc_ordered_workqueue("smmu_fault_queue", 0);
		if (!smmu->fault_queue)
			return -ENOMEM;
	}

	return arm_smmu_init_strtab(smmu);
}

static int arm_smmu_write_reg_sync(struct arm_smmu_device *smmu, u32 val,
				   unsigned int reg_off, unsigned int ack_off)
{
	u32 reg;

	writel_relaxed(val, smmu->base + reg_off);
	return readl_relaxed_poll_timeout(smmu->base + ack_off, reg, reg == val,
					  1, ARM_SMMU_POLL_TIMEOUT_US);
}

/* GBPA is "special" */
static int arm_smmu_update_gbpa(struct arm_smmu_device *smmu, u32 set, u32 clr)
{
	int ret;
	u32 reg, __iomem *gbpa = smmu->base + ARM_SMMU_GBPA;

	ret = readl_relaxed_poll_timeout(gbpa, reg, !(reg & GBPA_UPDATE),
					 1, ARM_SMMU_POLL_TIMEOUT_US);
	if (ret)
		return ret;

	reg &= ~clr;
	reg |= set;
	writel_relaxed(reg | GBPA_UPDATE, gbpa);
	return readl_relaxed_poll_timeout(gbpa, reg, !(reg & GBPA_UPDATE),
					  1, ARM_SMMU_POLL_TIMEOUT_US);
}

static void arm_smmu_free_msis(void *data)
{
	struct device *dev = data;
	platform_msi_domain_free_irqs(dev);
}

static void arm_smmu_write_msi_msg(struct msi_desc *desc, struct msi_msg *msg)
{
	phys_addr_t doorbell;
	struct device *dev = msi_desc_to_dev(desc);
	struct arm_smmu_device *smmu = dev_get_drvdata(dev);
	phys_addr_t *cfg = arm_smmu_msi_cfg[desc->platform.msi_index];

	doorbell = (((u64)msg->address_hi) << 32) | msg->address_lo;
	doorbell &= MSI_CFG0_ADDR_MASK << MSI_CFG0_ADDR_SHIFT;

	writeq_relaxed(doorbell, smmu->base + cfg[0]);
	writel_relaxed(msg->data, smmu->base + cfg[1]);
	writel_relaxed(MSI_CFG2_MEMATTR_DEVICE_nGnRE, smmu->base + cfg[2]);
}

static void arm_smmu_setup_msis(struct arm_smmu_device *smmu)
{
	struct msi_desc *desc;
	int ret, nvec = ARM_SMMU_MAX_MSIS;
	struct device *dev = smmu->dev;

	/* Clear the MSI address regs */
	writeq_relaxed(0, smmu->base + ARM_SMMU_GERROR_IRQ_CFG0);
	writeq_relaxed(0, smmu->base + ARM_SMMU_EVTQ_IRQ_CFG0);

	if (smmu->features & ARM_SMMU_FEAT_PRI)
		writeq_relaxed(0, smmu->base + ARM_SMMU_PRIQ_IRQ_CFG0);
	else
		nvec--;

	if (!(smmu->features & ARM_SMMU_FEAT_MSI))
		return;

	/* Allocate MSIs for evtq, gerror and priq. Ignore cmdq */
	ret = platform_msi_domain_alloc_irqs(dev, nvec, arm_smmu_write_msi_msg);
	if (ret) {
		dev_warn(dev, "failed to allocate MSIs\n");
		return;
	}

	for_each_msi_entry(desc, dev) {
		switch (desc->platform.msi_index) {
		case EVTQ_MSI_INDEX:
			smmu->evtq.q.irq = desc->irq;
			break;
		case GERROR_MSI_INDEX:
			smmu->gerr_irq = desc->irq;
			break;
		case PRIQ_MSI_INDEX:
			smmu->priq.q.irq = desc->irq;
			break;
		default:	/* Unknown */
			continue;
		}
	}

	/* Add callback to free MSIs on teardown */
	devm_add_action(dev, arm_smmu_free_msis, dev);
}

static int arm_smmu_setup_irqs(struct arm_smmu_device *smmu)
{
	int ret, irq;
	u32 irqen_flags = IRQ_CTRL_EVTQ_IRQEN | IRQ_CTRL_GERROR_IRQEN;

	/* Disable IRQs first */
	ret = arm_smmu_write_reg_sync(smmu, 0, ARM_SMMU_IRQ_CTRL,
				      ARM_SMMU_IRQ_CTRLACK);
	if (ret) {
		dev_err(smmu->dev, "failed to disable irqs\n");
		return ret;
	}

	arm_smmu_setup_msis(smmu);

	/* Request interrupt lines */
	irq = smmu->evtq.q.irq;
	if (irq) {
		ret = devm_request_threaded_irq(smmu->dev, irq, NULL,
						arm_smmu_evtq_thread,
						IRQF_ONESHOT,
						"arm-smmu-v3-evtq", smmu);
		if (ret < 0)
			dev_warn(smmu->dev, "failed to enable evtq irq\n");
	}

	irq = smmu->cmdq.q.irq;
	if (irq) {
		ret = devm_request_irq(smmu->dev, irq,
				       arm_smmu_cmdq_sync_handler, 0,
				       "arm-smmu-v3-cmdq-sync", smmu);
		if (ret < 0)
			dev_warn(smmu->dev, "failed to enable cmdq-sync irq\n");
	}

	irq = smmu->gerr_irq;
	if (irq) {
		ret = devm_request_irq(smmu->dev, irq, arm_smmu_gerror_handler,
				       0, "arm-smmu-v3-gerror", smmu);
		if (ret < 0)
			dev_warn(smmu->dev, "failed to enable gerror irq\n");
	}

	if (smmu->features & ARM_SMMU_FEAT_PRI) {
		irq = smmu->priq.q.irq;
		if (irq) {
			ret = devm_request_threaded_irq(smmu->dev, irq, NULL,
							arm_smmu_priq_thread,
							IRQF_ONESHOT,
							"arm-smmu-v3-priq",
							smmu);
			if (ret < 0)
				dev_warn(smmu->dev,
					 "failed to enable priq irq\n");
			else
				irqen_flags |= IRQ_CTRL_PRIQ_IRQEN;
		}
	}

	/* Enable interrupt generation on the SMMU */
	ret = arm_smmu_write_reg_sync(smmu, irqen_flags,
				      ARM_SMMU_IRQ_CTRL, ARM_SMMU_IRQ_CTRLACK);
	if (ret)
		dev_warn(smmu->dev, "failed to enable irqs\n");

	return 0;
}

static int arm_smmu_device_disable(struct arm_smmu_device *smmu)
{
	int ret;

	ret = arm_smmu_write_reg_sync(smmu, 0, ARM_SMMU_CR0, ARM_SMMU_CR0ACK);
	if (ret)
		dev_err(smmu->dev, "failed to clear cr0\n");

	return ret;
}

static int arm_smmu_device_reset(struct arm_smmu_device *smmu, bool bypass)
{
	int ret;
	u32 reg, enables;
	struct arm_smmu_cmdq_ent cmd;

	/* Clear CR0 and sync (disables SMMU and queue processing) */
	reg = readl_relaxed(smmu->base + ARM_SMMU_CR0);
	if (reg & CR0_SMMUEN)
		dev_warn(smmu->dev, "SMMU currently enabled! Resetting...\n");

	ret = arm_smmu_device_disable(smmu);
	if (ret)
		return ret;

	/* CR1 (table and queue memory attributes) */
	reg = (CR1_SH_ISH << CR1_TABLE_SH_SHIFT) |
	      (CR1_CACHE_WB << CR1_TABLE_OC_SHIFT) |
	      (CR1_CACHE_WB << CR1_TABLE_IC_SHIFT) |
	      (CR1_SH_ISH << CR1_QUEUE_SH_SHIFT) |
	      (CR1_CACHE_WB << CR1_QUEUE_OC_SHIFT) |
	      (CR1_CACHE_WB << CR1_QUEUE_IC_SHIFT);
	writel_relaxed(reg, smmu->base + ARM_SMMU_CR1);

	/* CR2 (random crap) */
	reg = CR2_RECINVSID;

	if (smmu->features & ARM_SMMU_FEAT_E2H)
		reg |= CR2_E2H;

	if (!(smmu->features & ARM_SMMU_FEAT_BTM))
		reg |= CR2_PTM;

	writel_relaxed(reg, smmu->base + ARM_SMMU_CR2);

	/* Stream table */
	writeq_relaxed(smmu->strtab_cfg.strtab_base,
		       smmu->base + ARM_SMMU_STRTAB_BASE);
	writel_relaxed(smmu->strtab_cfg.strtab_base_cfg,
		       smmu->base + ARM_SMMU_STRTAB_BASE_CFG);

	/* Command queue */
	writeq_relaxed(smmu->cmdq.q.q_base, smmu->base + ARM_SMMU_CMDQ_BASE);
	writel_relaxed(smmu->cmdq.q.prod, smmu->base + ARM_SMMU_CMDQ_PROD);
	writel_relaxed(smmu->cmdq.q.cons, smmu->base + ARM_SMMU_CMDQ_CONS);

	enables = CR0_CMDQEN;
	ret = arm_smmu_write_reg_sync(smmu, enables, ARM_SMMU_CR0,
				      ARM_SMMU_CR0ACK);
	if (ret) {
		dev_err(smmu->dev, "failed to enable command queue\n");
		return ret;
	}

	/* Invalidate any cached configuration */
	cmd.opcode = CMDQ_OP_CFGI_ALL;
	arm_smmu_cmdq_issue_cmd(smmu, &cmd);
	cmd.opcode = CMDQ_OP_CMD_SYNC;
	arm_smmu_cmdq_issue_cmd(smmu, &cmd);

	/* Invalidate any stale TLB entries */
	if (smmu->features & ARM_SMMU_FEAT_HYP) {
		cmd.opcode = CMDQ_OP_TLBI_EL2_ALL;
		arm_smmu_cmdq_issue_cmd(smmu, &cmd);
	}

	cmd.opcode = CMDQ_OP_TLBI_NSNH_ALL;
	arm_smmu_cmdq_issue_cmd(smmu, &cmd);
	cmd.opcode = CMDQ_OP_CMD_SYNC;
	arm_smmu_cmdq_issue_cmd(smmu, &cmd);

	/* Event queue */
	writeq_relaxed(smmu->evtq.q.q_base, smmu->base + ARM_SMMU_EVTQ_BASE);
	writel_relaxed(smmu->evtq.q.prod, smmu->base + ARM_SMMU_EVTQ_PROD);
	writel_relaxed(smmu->evtq.q.cons, smmu->base + ARM_SMMU_EVTQ_CONS);

	enables |= CR0_EVTQEN;
	ret = arm_smmu_write_reg_sync(smmu, enables, ARM_SMMU_CR0,
				      ARM_SMMU_CR0ACK);
	if (ret) {
		dev_err(smmu->dev, "failed to enable event queue\n");
		return ret;
	}

	/* PRI queue */
	if (smmu->features & ARM_SMMU_FEAT_PRI) {
		writeq_relaxed(smmu->priq.q.q_base,
			       smmu->base + ARM_SMMU_PRIQ_BASE);
		writel_relaxed(smmu->priq.q.prod,
			       smmu->base + ARM_SMMU_PRIQ_PROD);
		writel_relaxed(smmu->priq.q.cons,
			       smmu->base + ARM_SMMU_PRIQ_CONS);

		enables |= CR0_PRIQEN;
		ret = arm_smmu_write_reg_sync(smmu, enables, ARM_SMMU_CR0,
					      ARM_SMMU_CR0ACK);
		if (ret) {
			dev_err(smmu->dev, "failed to enable PRI queue\n");
			return ret;
		}
	}

	if (smmu->features & ARM_SMMU_FEAT_ATS && !disable_ats_check) {
		enables |= CR0_ATSCHK;
		ret = arm_smmu_write_reg_sync(smmu, enables, ARM_SMMU_CR0,
					      ARM_SMMU_CR0ACK);
		if (ret) {
			dev_err(smmu->dev, "failed to enable ATS check\n");
			return ret;
		}
	}

	ret = arm_smmu_setup_irqs(smmu);
	if (ret) {
		dev_err(smmu->dev, "failed to setup irqs\n");
		return ret;
	}


	/* Enable the SMMU interface, or ensure bypass */
	if (!bypass || disable_bypass) {
		enables |= CR0_SMMUEN;
	} else {
		ret = arm_smmu_update_gbpa(smmu, 0, GBPA_ABORT);
		if (ret) {
			dev_err(smmu->dev, "GBPA not responding to update\n");
			return ret;
		}
	}
	ret = arm_smmu_write_reg_sync(smmu, enables, ARM_SMMU_CR0,
				      ARM_SMMU_CR0ACK);
	if (ret) {
		dev_err(smmu->dev, "failed to enable SMMU interface\n");
		return ret;
	}

	return 0;
}

static bool arm_smmu_supports_svm(struct arm_smmu_device *smmu)
{
	unsigned long reg, fld;
	unsigned long oas;
	unsigned long asid_bits;

	u32 feat_mask = ARM_SMMU_FEAT_BTM | ARM_SMMU_FEAT_COHERENCY;

	if ((smmu->features & feat_mask) != feat_mask)
		return false;

	if (!smmu->ssid_bits)
		return false;

	if (!(smmu->pgsize_bitmap & PAGE_SIZE))
		return false;

	/*
	 * Get the smallest PA size of all CPUs (sanitized by cpufeature). We're
	 * not even pretending to support AArch32 here.
	 */
	reg = read_system_reg(SYS_ID_AA64MMFR0_EL1);
	fld = cpuid_feature_extract_unsigned_field(reg, ID_AA64MMFR0_PARANGE_SHIFT);
	switch (fld) {
	case 0x0:
		oas = 32;
		break;
	case 0x1:
		oas = 36;
		break;
	case 0x2:
		oas = 40;
		break;
	case 0x3:
		oas = 42;
		break;
	case 0x4:
		oas = 44;
		break;
	case 0x5:
		oas = 48;
		break;
	default:
		return false;
	}

	/* abort if MMU outputs addresses greater than what we support. */
	if (smmu->oas < oas)
		return false;

	/* We can support bigger ASIDs than the CPU, but not smaller */
	fld = cpuid_feature_extract_unsigned_field(reg, ID_AA64MMFR0_ASID_SHIFT);
	asid_bits = fld ? 16 : 8;
	if (smmu->asid_bits < asid_bits)
		return false;

	return true;
}

static int arm_smmu_device_hw_probe(struct arm_smmu_device *smmu)
{
	u32 reg;
	bool coherent = smmu->features & ARM_SMMU_FEAT_COHERENCY;
	bool vhe = cpus_have_cap(ARM64_HAS_VIRT_HOST_EXTN);

	/* IDR0 */
	reg = readl_relaxed(smmu->base + ARM_SMMU_IDR0);

	/* 2-level structures */
	if ((reg & IDR0_ST_LVL_MASK << IDR0_ST_LVL_SHIFT) == IDR0_ST_LVL_2LVL)
		smmu->features |= ARM_SMMU_FEAT_2_LVL_STRTAB;

	if (reg & IDR0_CD2L)
		smmu->features |= ARM_SMMU_FEAT_2_LVL_CDTAB;

	/*
	 * Translation table endianness.
	 * We currently require the same endianness as the CPU, but this
	 * could be changed later by adding a new IO_PGTABLE_QUIRK.
	 */
	switch (reg & IDR0_TTENDIAN_MASK << IDR0_TTENDIAN_SHIFT) {
	case IDR0_TTENDIAN_MIXED:
		smmu->features |= ARM_SMMU_FEAT_TT_LE | ARM_SMMU_FEAT_TT_BE;
		break;
#ifdef __BIG_ENDIAN
	case IDR0_TTENDIAN_BE:
		smmu->features |= ARM_SMMU_FEAT_TT_BE;
		break;
#else
	case IDR0_TTENDIAN_LE:
		smmu->features |= ARM_SMMU_FEAT_TT_LE;
		break;
#endif
	default:
		dev_err(smmu->dev, "unknown/unsupported TT endianness!\n");
		return -ENXIO;
	}

	/* Boolean feature flags */
	if (IS_ENABLED(CONFIG_PCI_PRI) && reg & IDR0_PRI)
		smmu->features |= ARM_SMMU_FEAT_PRI;

	if (IS_ENABLED(CONFIG_PCI_ATS) && reg & IDR0_ATS)
		smmu->features |= ARM_SMMU_FEAT_ATS;

	if (reg & IDR0_SEV)
		smmu->features |= ARM_SMMU_FEAT_SEV;

	if (reg & IDR0_MSI)
		smmu->features |= ARM_SMMU_FEAT_MSI;

	if (reg & IDR0_HYP) {
		smmu->features |= ARM_SMMU_FEAT_HYP;
		if (vhe)
			smmu->features |= ARM_SMMU_FEAT_E2H;
	}

	/*
	 * If the CPU is using VHE, but the SMMU doesn't support it, the SMMU
	 * will create TLB entries for NH-EL1 world and will miss the
	 * broadcasted TLB invalidations that target EL2-E2H world. Don't enable
	 * BTM in that case.
	 */
	if (reg & IDR0_BTM && (!vhe || reg & IDR0_HYP))
		smmu->features |= ARM_SMMU_FEAT_BTM;

	/*
	 * The coherency feature as set by FW is used in preference to the ID
	 * register, but warn on mismatch.
	 */
	if (!!(reg & IDR0_COHACC) != coherent)
		dev_warn(smmu->dev, "IDR0.COHACC overridden by dma-coherent property (%s)\n",
			 coherent ? "true" : "false");

	switch (reg & IDR0_STALL_MODEL_MASK << IDR0_STALL_MODEL_SHIFT) {
	case IDR0_STALL_MODEL_STALL:
		/* Fallthrough */
	case IDR0_STALL_MODEL_FORCE:
		smmu->features |= ARM_SMMU_FEAT_STALLS;
	}

	if (reg & IDR0_S1P)
		smmu->features |= ARM_SMMU_FEAT_TRANS_S1;

	if (reg & IDR0_S2P)
		smmu->features |= ARM_SMMU_FEAT_TRANS_S2;

	if (!(reg & (IDR0_S1P | IDR0_S2P))) {
		dev_err(smmu->dev, "no translation support!\n");
		return -ENXIO;
	}

	/* We only support the AArch64 table format at present */
	switch (reg & IDR0_TTF_MASK << IDR0_TTF_SHIFT) {
	case IDR0_TTF_AARCH32_64:
		smmu->ias = 40;
		/* Fallthrough */
	case IDR0_TTF_AARCH64:
		break;
	default:
		dev_err(smmu->dev, "AArch64 table format not supported!\n");
		return -ENXIO;
	}

	/* ASID/VMID sizes */
	smmu->asid_bits = reg & IDR0_ASID16 ? 16 : 8;
	smmu->vmid_bits = reg & IDR0_VMID16 ? 16 : 8;

	/* IDR1 */
	reg = readl_relaxed(smmu->base + ARM_SMMU_IDR1);
	if (reg & (IDR1_TABLES_PRESET | IDR1_QUEUES_PRESET | IDR1_REL)) {
		dev_err(smmu->dev, "embedded implementation not supported\n");
		return -ENXIO;
	}

	/* Queue sizes, capped at 4k */
	smmu->cmdq.q.max_n_shift = min((u32)CMDQ_MAX_SZ_SHIFT,
				       reg >> IDR1_CMDQ_SHIFT & IDR1_CMDQ_MASK);
	if (!smmu->cmdq.q.max_n_shift) {
		/* Odd alignment restrictions on the base, so ignore for now */
		dev_err(smmu->dev, "unit-length command queue not supported\n");
		return -ENXIO;
	}

	smmu->evtq.q.max_n_shift = min((u32)EVTQ_MAX_SZ_SHIFT,
				       reg >> IDR1_EVTQ_SHIFT & IDR1_EVTQ_MASK);
	smmu->priq.q.max_n_shift = min((u32)PRIQ_MAX_SZ_SHIFT,
				       reg >> IDR1_PRIQ_SHIFT & IDR1_PRIQ_MASK);

	/* SID/SSID sizes */
	smmu->ssid_bits = reg >> IDR1_SSID_SHIFT & IDR1_SSID_MASK;
	smmu->sid_bits = reg >> IDR1_SID_SHIFT & IDR1_SID_MASK;

	/*
	 * If the SMMU supports fewer bits than would fill a single L2 stream
	 * table, use a linear table instead.
	 */
	if (smmu->sid_bits <= STRTAB_SPLIT)
		smmu->features &= ~ARM_SMMU_FEAT_2_LVL_STRTAB;

	/* IDR5 */
	reg = readl_relaxed(smmu->base + ARM_SMMU_IDR5);

	/* Maximum number of outstanding stalls */
	smmu->evtq.max_stalls = reg >> IDR5_STALL_MAX_SHIFT
				& IDR5_STALL_MAX_MASK;

	/* Page sizes */
	if (reg & IDR5_GRAN64K)
		smmu->pgsize_bitmap |= SZ_64K | SZ_512M;
	if (reg & IDR5_GRAN16K)
		smmu->pgsize_bitmap |= SZ_16K | SZ_32M;
	if (reg & IDR5_GRAN4K)
		smmu->pgsize_bitmap |= SZ_4K | SZ_2M | SZ_1G;

	if (arm_smmu_ops.pgsize_bitmap == -1UL)
		arm_smmu_ops.pgsize_bitmap = smmu->pgsize_bitmap;
	else
		arm_smmu_ops.pgsize_bitmap |= smmu->pgsize_bitmap;

	/* Output address size */
	switch (reg & IDR5_OAS_MASK << IDR5_OAS_SHIFT) {
	case IDR5_OAS_32_BIT:
		smmu->oas = 32;
		break;
	case IDR5_OAS_36_BIT:
		smmu->oas = 36;
		break;
	case IDR5_OAS_40_BIT:
		smmu->oas = 40;
		break;
	case IDR5_OAS_42_BIT:
		smmu->oas = 42;
		break;
	case IDR5_OAS_44_BIT:
		smmu->oas = 44;
		break;
	default:
		dev_info(smmu->dev,
			"unknown output address size. Truncating to 48-bit\n");
		/* Fallthrough */
	case IDR5_OAS_48_BIT:
		smmu->oas = 48;
	}

	/* Set the DMA mask for our table walker */
	if (dma_set_mask_and_coherent(smmu->dev, DMA_BIT_MASK(smmu->oas)))
		dev_warn(smmu->dev,
			 "failed to set DMA mask for table walker\n");

	smmu->ias = max(smmu->ias, smmu->oas);

	if (arm_smmu_supports_svm(smmu))
		smmu->features |= ARM_SMMU_FEAT_SVM;

	dev_info(smmu->dev, "ias %lu-bit, oas %lu-bit (features 0x%08x)\n",
		 smmu->ias, smmu->oas, smmu->features);
	return 0;
}

#ifdef CONFIG_ACPI
static int arm_smmu_device_acpi_probe(struct platform_device *pdev,
				      struct arm_smmu_device *smmu)
{
	struct acpi_iort_smmu_v3 *iort_smmu;
	struct device *dev = smmu->dev;
	struct acpi_iort_node *node;

	node = *(struct acpi_iort_node **)dev_get_platdata(dev);

	/* Retrieve SMMUv3 specific data */
	iort_smmu = (struct acpi_iort_smmu_v3 *)node->node_data;

	if (iort_smmu->flags & ACPI_IORT_SMMU_V3_COHACC_OVERRIDE)
		smmu->features |= ARM_SMMU_FEAT_COHERENCY;

	return 0;
}
#else
static inline int arm_smmu_device_acpi_probe(struct platform_device *pdev,
					     struct arm_smmu_device *smmu)
{
	return -ENODEV;
}
#endif

static int arm_smmu_device_dt_probe(struct platform_device *pdev,
				    struct arm_smmu_device *smmu)
{
	struct device *dev = &pdev->dev;
	u32 cells;
	int ret = -EINVAL;

	if (of_property_read_u32(dev->of_node, "#iommu-cells", &cells))
		dev_err(dev, "missing #iommu-cells property\n");
	else if (cells != 1)
		dev_err(dev, "invalid #iommu-cells value (%d)\n", cells);
	else
		ret = 0;

	parse_driver_options(smmu);

	if (of_dma_is_coherent(dev->of_node))
		smmu->features |= ARM_SMMU_FEAT_COHERENCY;

	return ret;
}

static int arm_smmu_device_probe(struct platform_device *pdev)
{
	int irq, ret;
	struct resource *res;
	resource_size_t ioaddr;
	struct arm_smmu_device *smmu;
	struct device *dev = &pdev->dev;
	bool bypass;

	smmu = devm_kzalloc(dev, sizeof(*smmu), GFP_KERNEL);
	if (!smmu) {
		dev_err(dev, "failed to allocate arm_smmu_device\n");
		return -ENOMEM;
	}
	smmu->dev = dev;

	/* Base address */
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (resource_size(res) + 1 < SZ_128K) {
		dev_err(dev, "MMIO region too small (%pr)\n", res);
		return -EINVAL;
	}
	ioaddr = res->start;

	smmu->base = devm_ioremap_resource(dev, res);
	if (IS_ERR(smmu->base))
		return PTR_ERR(smmu->base);

	/* Interrupt lines */
	irq = platform_get_irq_byname(pdev, "eventq");
	if (irq > 0)
		smmu->evtq.q.irq = irq;

	irq = platform_get_irq_byname(pdev, "priq");
	if (irq > 0)
		smmu->priq.q.irq = irq;

	irq = platform_get_irq_byname(pdev, "cmdq-sync");
	if (irq > 0)
		smmu->cmdq.q.irq = irq;

	irq = platform_get_irq_byname(pdev, "gerror");
	if (irq > 0)
		smmu->gerr_irq = irq;

	if (dev->of_node) {
		ret = arm_smmu_device_dt_probe(pdev, smmu);
	} else {
		ret = arm_smmu_device_acpi_probe(pdev, smmu);
		if (ret == -ENODEV)
			return ret;
	}

	/* Set bypass mode according to firmware probing result */
	bypass = !!ret;

	/* Probe the h/w */
	ret = arm_smmu_device_hw_probe(smmu);
	if (ret)
		return ret;

	/* Initialise in-memory data structures */
	ret = arm_smmu_init_structures(smmu);
	if (ret)
		return ret;

	/* Record our private device structure */
	platform_set_drvdata(pdev, smmu);

	/* Reset the device */
	ret = arm_smmu_device_reset(smmu, bypass);
	if (ret)
		return ret;

	/* And we're up. Go go go! */
	ret = iommu_device_sysfs_add(&smmu->iommu, dev, NULL,
				     "smmu3.%pa", &ioaddr);
	if (ret)
		return ret;

	iommu_device_set_ops(&smmu->iommu, &arm_smmu_ops);
	iommu_device_set_fwnode(&smmu->iommu, dev->fwnode);

	ret = iommu_device_register(&smmu->iommu);

#ifdef CONFIG_PCI
	if (pci_bus_type.iommu_ops != &arm_smmu_ops) {
		pci_request_acs();
		ret = bus_set_iommu(&pci_bus_type, &arm_smmu_ops);
		if (ret)
			return ret;
	}
#endif
#ifdef CONFIG_ARM_AMBA
	if (amba_bustype.iommu_ops != &arm_smmu_ops) {
		ret = bus_set_iommu(&amba_bustype, &arm_smmu_ops);
		if (ret)
			return ret;
	}
#endif
	if (platform_bus_type.iommu_ops != &arm_smmu_ops) {
		ret = bus_set_iommu(&platform_bus_type, &arm_smmu_ops);
		if (ret)
			return ret;
	}
	return 0;
}

static int arm_smmu_device_remove(struct platform_device *pdev)
{
	struct arm_smmu_device *smmu = platform_get_drvdata(pdev);

	arm_smmu_device_disable(smmu);
	return 0;
}

static struct of_device_id arm_smmu_of_match[] = {
	{ .compatible = "arm,smmu-v3", },
	{ },
};
MODULE_DEVICE_TABLE(of, arm_smmu_of_match);

static struct platform_driver arm_smmu_driver = {
	.driver	= {
		.name		= "arm-smmu-v3",
		.of_match_table	= of_match_ptr(arm_smmu_of_match),
	},
	.probe	= arm_smmu_device_probe,
	.remove	= arm_smmu_device_remove,
};

static int __init arm_smmu_init(void)
{
	static bool registered;
	int ret = 0;

	if (!registered) {
		arm_smmu_fault_cache = KMEM_CACHE(arm_smmu_fault, 0);
		if (!arm_smmu_fault_cache)
			return -ENOMEM;

		ret = platform_driver_register(&arm_smmu_driver);
		registered = !ret;
	}
	return ret;
}

static void __exit arm_smmu_exit(void)
{
	return platform_driver_unregister(&arm_smmu_driver);
}

subsys_initcall(arm_smmu_init);
module_exit(arm_smmu_exit);

static int __init arm_smmu_of_init(struct device_node *np)
{
	int ret = arm_smmu_init();

	if (ret)
		return ret;

	if (!of_platform_device_create(np, NULL, platform_bus_type.dev_root))
		return -ENODEV;

	return 0;
}
IOMMU_OF_DECLARE(arm_smmuv3, "arm,smmu-v3", arm_smmu_of_init);

#ifdef CONFIG_ACPI
static int __init acpi_smmu_v3_init(struct acpi_table_header *table)
{
	if (iort_node_match(ACPI_IORT_NODE_SMMU_V3))
		return arm_smmu_init();

	return 0;
}
IORT_ACPI_DECLARE(arm_smmu_v3, ACPI_SIG_IORT, acpi_smmu_v3_init);
#endif

MODULE_DESCRIPTION("IOMMU API for ARM architected SMMUv3 implementations");
MODULE_AUTHOR("Will Deacon <will.deacon@arm.com>");
MODULE_LICENSE("GPL v2");
