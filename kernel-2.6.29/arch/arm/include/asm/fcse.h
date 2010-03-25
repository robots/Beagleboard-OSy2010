/*
 * Filename:    arch/arm/include/asm/fcse.h
 * Description: ARM Process ID (PID) includes for Fast Address Space Switching
 *              (FASS) in ARM Linux.
 * Created:     14/10/2001
 * Changes:     19/02/2002 - Macros added.
 *              03/08/2007 - Adapted to kernel 2.6.21 (ssm)
 *              Feb 2008   - Simplified a bit (rco)
 *
 * Copyright:   (C) 2001, 2002 Adam Wiggins <awiggins@cse.unsw.edu.au>
 *              (C) 2007 Sebastian Smolorz <ssm@emlix.com>
 *              (C) 2008 Richard Cochran
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of teh GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef __ASM_ARM_FCSE_H
#define __ASM_ARM_FCSE_H

#ifdef CONFIG_ARM_FCSE

#include <linux/mm_types.h>	/* For struct mm_struct */

#define FCSE_PID_SHIFT 25

/* Size of PID relocation area */
#define FCSE_PID_TASK_SIZE (1UL << FCSE_PID_SHIFT)

/* Mask to get rid of PID from relocated address */
#define FCSE_PID_MASK (FCSE_PID_TASK_SIZE - 1)

#define fcse_tlb_mask(mm) ((mm)->context.cpu_tlb_mask)
#define fcse_cpu_set_vm_mask(cpu, mm) cpu_set(cpu, (mm)->cpu_vm_mask)

/* Sets the CPU's PID Register */
static inline void fcse_pid_set(unsigned long pid)
{
	__asm__ __volatile__ ("mcr p15, 0, %0, c13, c0, 0"
			      : /* */ : "r" (pid) : "memory", "cc");
}

/* Returns the state of the CPU's PID Register */
static inline unsigned long fcse_pid_get(void)
{
	unsigned long pid;
	__asm__ __volatile__("mrc p15, 0, %0, c13, c0, 0" 
			     : "=&r" (pid) : /* */ : "cc");
	return pid & ~FCSE_PID_MASK;
}

static inline unsigned long fcse_mva_to_va(unsigned long mva)
{
	unsigned long pid = fcse_pid_get();
	if (pid && (pid == (mva & ~FCSE_PID_MASK)))
		return mva & FCSE_PID_MASK;
	return mva;
}

static inline unsigned long
fcse_va_to_mva(struct mm_struct *mm, unsigned long va)
{
	if (va < FCSE_PID_TASK_SIZE)
		return mm->context.pid | va;
	return va;
}

int fcse_pid_alloc(void);
void fcse_pid_free(unsigned pid);
void fcse_pid_reference(unsigned pid);
#ifdef CONFIG_ARM_FCSE_BEST_EFFORT
int fcse_needs_flush(struct mm_struct *prev, struct mm_struct *next);
void fcse_notify_flush_all(void);
void fcse_relocate_mm_to_null_pid(struct mm_struct *mm);
#else /* CONFIG_ARM_FCSE_GUARANTEED */
#define fcse_needs_flush(prev, next) (0)
#define fcse_notify_flush_all() do { } while (0)
#endif /* CONFIG_ARM_FCSE_GUARANTEED */

#else /* CONFIG_ARM_FCSE */
#define fcse_pid_set(pid) do { } while (0)
#define fcse_mva_to_va(x) (x)
#define fcse_va_to_mva(mm, x) ({ (void)(mm); (x); })
#define fcse_tlb_mask(mm) ((mm)->cpu_vm_mask)
#define fcse_cpu_set_vm_mask(cpu, mm) do { } while (0)
#define fcse_needs_flush(prev, next) (1)
#define fcse_notify_flush_all() do { } while (0)
#endif

#endif /* __ASM_ARM_FCSE_H */
