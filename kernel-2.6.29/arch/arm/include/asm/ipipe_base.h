/* -*- linux-c -*-
 * arch/arm/include/asm/ipipe_base.h
 *
 * Copyright (C) 2007 Gilles Chanteperdrix.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, Inc., 675 Mass Ave, Cambridge MA 02139,
 * USA; either version 2 of the License, or (at your option) any later
 * version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __ARM_IPIPE_BASE_H
#define __ARM_IPIPE_BASE_H

#include <linux/threads.h>
#include <asm/irq.h>

#define IPIPE_NR_XIRQS		NR_IRQS
#define IPIPE_IRQ_ISHIFT	5	/* 25 for 32bits arch. */

/* ARM traps */
#define IPIPE_TRAP_ACCESS	 0	/* Data or instruction access exception */
#define IPIPE_TRAP_SECTION	 1	/* Section fault */
#define IPIPE_TRAP_DABT		 2	/* Generic data abort */
#define IPIPE_TRAP_UNKNOWN	 3	/* Unknown exception */
#define IPIPE_TRAP_BREAK	 4	/* Instruction breakpoint */
#define IPIPE_TRAP_FPU		 5	/* Floating point exception */
#define IPIPE_TRAP_VFP		 6	/* VFP floating point exception */
#define IPIPE_TRAP_UNDEFINSTR	 7	/* Undefined instruction */
#define IPIPE_TRAP_ALIGNMENT	 8	/* Unaligned access exception */
#define IPIPE_NR_FAULTS		 9

/* Pseudo-vectors used for kernel events */
#define IPIPE_FIRST_EVENT	IPIPE_NR_FAULTS
#define IPIPE_EVENT_SYSCALL	(IPIPE_FIRST_EVENT)
#define IPIPE_EVENT_SCHEDULE	(IPIPE_FIRST_EVENT + 1)
#define IPIPE_EVENT_SIGWAKE	(IPIPE_FIRST_EVENT + 2)
#define IPIPE_EVENT_SETSCHED	(IPIPE_FIRST_EVENT + 3)
#define IPIPE_EVENT_INIT	(IPIPE_FIRST_EVENT + 4)
#define IPIPE_EVENT_EXIT	(IPIPE_FIRST_EVENT + 5)
#define IPIPE_EVENT_CLEANUP	(IPIPE_FIRST_EVENT + 6)
#define IPIPE_LAST_EVENT	IPIPE_EVENT_CLEANUP
#define IPIPE_NR_EVENTS		(IPIPE_LAST_EVENT + 1)

#ifndef __ASSEMBLY__

#include <asm/irqflags.h>

#ifdef CONFIG_SMP
#error "SMP not implemented."
#define __ipipe_root_status ipipe_root_cpudom_var(status)

#else /* !CONFIG_SMP */

#ifdef CONFIG_VFP
#define __IPIPE_FEATURE_VFP_SAFE 1
#endif

#if __GNUC__ >= 4
/* Alias to ipipe_root_cpudom_var(status) */
extern unsigned long __ipipe_root_status;
#else
extern unsigned long *const __ipipe_root_status_addr;
#define __ipipe_root_status	(*__ipipe_root_status_addr)
#endif

static inline void __ipipe_stall_root(void)
{
	unsigned long flags;

	local_irq_save_hw(flags);
	__ipipe_root_status |= 1;
	local_irq_restore_hw(flags);
}

static inline unsigned __ipipe_test_root(void)
{
	return __ipipe_root_status & 1;
}

static inline unsigned __ipipe_test_and_stall_root(void)
{
	unsigned long flags, res;

	local_irq_save_hw(flags);
	res = __ipipe_root_status;
	__ipipe_root_status = res | 1;
	local_irq_restore_hw(flags);

	return res & 1;
}

#endif	/* CONFIG_SMP */

#endif /* !__ASSEMBLY__ */

#endif /* __ARM_IPIPE_BASE_H */
