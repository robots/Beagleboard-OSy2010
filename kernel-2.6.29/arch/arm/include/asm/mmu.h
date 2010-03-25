#ifndef __ARM_MMU_H
#define __ARM_MMU_H

#ifdef CONFIG_MMU

typedef struct {
#ifdef CONFIG_CPU_HAS_ASID
	unsigned int id;
#endif
#ifdef CONFIG_ARM_FCSE
	unsigned long pid;
#ifdef CONFIG_ARM_FCSE_BEST_EFFORT
	unsigned big;
	unsigned high_pages;
	unsigned shared_dirty_pages;
#endif /* CONFIG_ARM_FCSE_BEST_EFFORT */
	cpumask_t cpu_tlb_mask;
#endif
	unsigned int kvm_seq;
} mm_context_t;

#ifdef CONFIG_CPU_HAS_ASID
#define ASID(mm)	((mm)->context.id & 255)
#else
#define ASID(mm)	(0)
#endif

#else

/*
 * From nommu.h:
 *  Copyright (C) 2002, David McCullough <davidm@snapgear.com>
 *  modified for 2.6 by Hyok S. Choi <hyok.choi@samsung.com>
 */
typedef struct {
	unsigned long		end_brk;
} mm_context_t;

#endif

#endif
