/*
 *  arch/arm/include/asm/mmu_context.h
 *
 *  Copyright (C) 1996 Russell King.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 *  Changelog:
 *   27-06-1996	RMK	Created
 */
#ifndef __ASM_ARM_MMU_CONTEXT_H
#define __ASM_ARM_MMU_CONTEXT_H

#include <linux/compiler.h>
#include <linux/sched.h>
#include <asm/cacheflush.h>
#include <asm/cachetype.h>
#include <asm/proc-fns.h>
#include <asm-generic/mm_hooks.h>
#include <asm/fcse.h>

void __check_kvm_seq(struct mm_struct *mm);

#ifdef CONFIG_CPU_HAS_ASID

/*
 * On ARMv6, we have the following structure in the Context ID:
 *
 * 31                         7          0
 * +-------------------------+-----------+
 * |      process ID         |   ASID    |
 * +-------------------------+-----------+
 * |              context ID             |
 * +-------------------------------------+
 *
 * The ASID is used to tag entries in the CPU caches and TLBs.
 * The context ID is used by debuggers and trace logic, and
 * should be unique within all running processes.
 */
#define ASID_BITS		8
#define ASID_MASK		((~0) << ASID_BITS)
#define ASID_FIRST_VERSION	(1 << ASID_BITS)

extern unsigned int cpu_last_asid;

void __init_new_context(struct task_struct *tsk, struct mm_struct *mm);
void __new_context(struct mm_struct *mm);

static inline void check_context(struct mm_struct *mm)
{
	if (unlikely((mm->context.id ^ cpu_last_asid) >> ASID_BITS))
		__new_context(mm);

	if (unlikely(mm->context.kvm_seq != init_mm.context.kvm_seq))
		__check_kvm_seq(mm);
}

#define init_new_context(tsk,mm)	(__init_new_context(tsk,mm),0)

#else

static inline void check_context(struct mm_struct *mm)
{
	if (unlikely(mm->context.kvm_seq != init_mm.context.kvm_seq))
		__check_kvm_seq(mm);
}

static inline int
init_new_context(struct task_struct *tsk, struct mm_struct *mm)
{
#ifdef CONFIG_ARM_FCSE
	int pid;

	cpus_clear(mm->context.cpu_tlb_mask);
#ifdef CONFIG_ARM_FCSE_BEST_EFFORT
	if (!mm->context.big) {
		pid = fcse_pid_alloc();
		mm->context.pid = pid << FCSE_PID_SHIFT;
	} else {
		/* We are normally forking a process vith a virtual address
		   space larger than 32 MB, so its pid should be 0. */
		BUG_ON(mm->context.pid);
		fcse_pid_reference(0);
	}
	/* If we are forking, set_pte_at will restore the correct high pages
	   count, and shared writable pages are write-protected again. */
	mm->context.high_pages = 0;
	mm->context.shared_dirty_pages = 0;
#else /* CONFIG_ARM_FCSE_GUARANTEED */
	pid = fcse_pid_alloc();
	if (pid < 0)
		return pid;
	mm->context.pid = pid << FCSE_PID_SHIFT;
#endif /* CONFIG_ARM_FCSE_GUARANTEED */
#endif /* CONFIG_ARM_FCSE */
	return 0;
}

#endif

static inline void destroy_context(struct mm_struct *mm)
{
#ifdef CONFIG_ARM_FCSE
#ifdef CONFIG_ARM_FCSE_BEST_EFFORT
	BUG_ON(mm->context.high_pages);
	BUG_ON(mm->context.shared_dirty_pages);
#endif /* CONFIG_ARM_FCSE_BEST_EFFORT */
	fcse_pid_free(mm->context.pid >> FCSE_PID_SHIFT);
#endif /* CONFIG_ARM_FCSE */
}

/*
 * This is called when "tsk" is about to enter lazy TLB mode.
 *
 * mm:  describes the currently active mm context
 * tsk: task which is entering lazy tlb
 * cpu: cpu number which is entering lazy tlb
 *
 * tsk->mm will be NULL
 */
static inline void
enter_lazy_tlb(struct mm_struct *mm, struct task_struct *tsk)
{
}

/*
 * This is the actual mm switch as far as the scheduler
 * is concerned.  No registers are touched.  We avoid
 * calling the CPU specific function when the mm hasn't
 * actually changed.
 */
static inline void
switch_mm(struct mm_struct *prev, struct mm_struct *next,
	  struct task_struct *tsk)
{
#ifdef CONFIG_MMU
	unsigned int cpu = smp_processor_id_hw();

#ifdef CONFIG_SMP
	/* check for possible thread migration */
	if (!cpus_empty(next->cpu_vm_mask) && !cpu_isset(cpu, next->cpu_vm_mask))
		__flush_icache_all();
#endif
	if (!cpu_test_and_set(cpu, fcse_tlb_mask(next)) || prev != next) {
		fcse_cpu_set_vm_mask(cpu, next);
		check_context(next);
#if defined(CONFIG_IPIPE)
		if (ipipe_root_domain_p)
			do {
				/* mark mm state as undefined. */
				per_cpu(ipipe_active_mm, cpu) = NULL;
				barrier();
				fcse_pid_set(next->context.pid);
				cpu_switch_mm(next->pgd, next,
					      fcse_needs_flush(prev, next));
				barrier();
				prev = xchg(&per_cpu(ipipe_active_mm, cpu),
					    next);
			} while (test_and_clear_thread_flag(TIF_MMSWITCH_INT));
		else
#endif /* CONFIG_IPIPE */
			{
				fcse_pid_set(next->context.pid);
				cpu_switch_mm(next->pgd, next,
					      fcse_needs_flush(prev, next));
			}
		if (cache_is_vivt() && prev)
			cpu_clear(cpu, fcse_tlb_mask(prev));
	}
#endif
}

#define deactivate_mm(tsk,mm)	do { } while (0)
#define activate_mm(prev,next)	switch_mm(prev, next, NULL)

#endif
