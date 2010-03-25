/* -*- linux-c -*-
 * linux/arch/arm/kernel/ipipe.c
 *
 * Copyright (C) 2002-2005 Philippe Gerum.
 * Copyright (C) 2004 Wolfgang Grandegger (Adeos/arm port over 2.4).
 * Copyright (C) 2005 Heikki Lindholm (PowerPC 970 fixes).
 * Copyright (C) 2005 Stelian Pop.
 * Copyright (C) 2006-2008 Gilles Chanteperdrix.
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
 *
 * Architecture-dependent I-PIPE support for ARM.
 */

#include <linux/kernel.h>
#include <linux/smp.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/kallsyms.h>
#include <linux/kprobes.h>
#include <asm/system.h>
#include <asm/atomic.h>
#include <asm/hardirq.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/unistd.h>
#include <asm/mach/irq.h>
#include <asm/mmu_context.h>

/* Next tick date (timebase value). */
DEFINE_PER_CPU(struct pt_regs, __ipipe_tick_regs);
DEFINE_PER_CPU(struct mm_struct *,ipipe_active_mm);
EXPORT_PER_CPU_SYMBOL(ipipe_active_mm);
#ifdef __IPIPE_FEATURE_PIC_MUTE
__ipipe_irqbits_t __ipipe_irqbits;
IPIPE_DEFINE_SPINLOCK(__ipipe_irqbits_lock);
#endif /* __IPIPE_FEATURE_PIC_MUTE */

extern struct irq_desc irq_desc[];
asmlinkage void asm_do_IRQ(unsigned int irq, struct pt_regs *regs);

#ifdef CONFIG_SMP

static cpumask_t __ipipe_cpu_sync_map;

static cpumask_t __ipipe_cpu_lock_map;

static IPIPE_DEFINE_SPINLOCK(__ipipe_cpu_barrier);

static atomic_t __ipipe_critical_count = ATOMIC_INIT(0);

static void (*__ipipe_cpu_sync) (void);

/* Always called with hw interrupts off. */


void __ipipe_do_critical_sync(unsigned irq)
{
	int cpu = ipipe_processor_id();

	cpu_set(cpu, __ipipe_cpu_sync_map);

	/*
	 * Now we are in sync with the lock requestor running on another
	 * CPU. Enter a spinning wait until he releases the global
	 * lock.
	 */
	spin_lock(&__ipipe_cpu_barrier);

	/* Got it. Now get out. */

	if (__ipipe_cpu_sync)
		/* Call the sync routine if any. */
		__ipipe_cpu_sync();

	spin_unlock(&__ipipe_cpu_barrier);

	cpu_clear(cpu, __ipipe_cpu_sync_map);
}

#endif	/* CONFIG_SMP */

/*
 * ipipe_trigger_irq() -- Push the interrupt at front of the pipeline
 * just like if it has been actually received from a hw source. Also
 * works for virtual interrupts.
 */
int ipipe_trigger_irq(unsigned irq)
{
	unsigned long flags;

	if (irq >= IPIPE_NR_IRQS ||
	    (ipipe_virtual_irq_p(irq)
	     && !test_bit(irq - IPIPE_VIRQ_BASE, &__ipipe_virtual_irq_map)))
		return -EINVAL;

	local_irq_save_hw(flags);

	__ipipe_handle_irq(irq, NULL);

	local_irq_restore_hw(flags);

	return 1;
}

int ipipe_get_sysinfo(struct ipipe_sysinfo *info)
{
	info->ncpus = num_online_cpus();
	info->cpufreq = ipipe_cpu_freq();
	info->archdep.tmirq = __ipipe_mach_timerint;
	info->archdep.tmfreq = info->cpufreq;
        __ipipe_mach_get_tscinfo(&info->archdep.tsc);

	return 0;
}

static void __ipipe_ack_irq(unsigned irq, struct irq_desc *desc)
{
	desc->ipipe_ack(irq, desc);
}

static void __ipipe_ack_timerirq(unsigned irq, struct irq_desc *desc)
{
	desc->ipipe_ack(irq, desc);
	__ipipe_mach_acktimer();
	desc->ipipe_end(irq, desc);
}

void __ipipe_enable_irqdesc(struct ipipe_domain *ipd, unsigned irq)
{
	unsigned long flags;
	irq_desc[irq].status &= ~IRQ_DISABLED;
#ifdef __IPIPE_FEATURE_PIC_MUTE
	if (ipd == &ipipe_root)
		return;
	
	spin_lock_irqsave(&__ipipe_irqbits_lock, flags);
	__ipipe_irqbits[irq / BITS_PER_LONG] &= ~(1 << (irq % BITS_PER_LONG));
	spin_unlock_irqrestore(&__ipipe_irqbits_lock, flags);
	printk("__ipipe_irqbits(after en)[%u]: 0x%08lx\n",
	       irq / BITS_PER_LONG, __ipipe_irqbits[irq / BITS_PER_LONG]);
#else
	(void) flags;
#endif /* __IPIPE_FEATURE_PIC_MUTE */
}
EXPORT_SYMBOL(__ipipe_enable_irqdesc);

#ifdef __IPIPE_FEATURE_PIC_MUTE
void __ipipe_disable_irqdesc(struct ipipe_domain *ipd, unsigned irq)
{
	unsigned long flags;

	if (ipd == &ipipe_root)
		return;
	
	spin_lock_irqsave(&__ipipe_irqbits_lock, flags);
	__ipipe_irqbits[irq / BITS_PER_LONG] |= 1 << (irq % BITS_PER_LONG);
	spin_unlock_irqrestore(&__ipipe_irqbits_lock, flags);
	printk("__ipipe_irqbits(after dis)[%u]: 0x%08lx\n",
	       irq / BITS_PER_LONG, __ipipe_irqbits[irq / BITS_PER_LONG]);
}
EXPORT_SYMBOL(__ipipe_disable_irqdesc);
#endif /* __IPIPE_FEATURE_PIC_MUTE */

/*
 * __ipipe_enable_pipeline() -- We are running on the boot CPU, hw
 * interrupts are off, and secondary CPUs are still lost in space.
 */
void __ipipe_enable_pipeline(void)
{
	unsigned long flags;
	unsigned irq;

/* We do not want "wfi" to be called in arm926ejs based processor, as
   this causes the I-cache to be disabled when idle. */
#ifdef CONFIG_CPU_ARM926T
	disable_hlt();
#endif

	flags = ipipe_critical_enter(NULL);

	/* First, virtualize all interrupts from the root domain. */

	for (irq = 0; irq < NR_IRQS; irq++)
		ipipe_virtualize_irq(ipipe_root_domain,
				     irq,
				     (ipipe_irq_handler_t)&asm_do_IRQ, NULL,
				     ((irq == __ipipe_mach_timerint)
				      ? &__ipipe_ack_timerirq
				      : &__ipipe_ack_irq),
				     IPIPE_HANDLE_MASK | IPIPE_PASS_MASK);

	ipipe_critical_exit(flags);
}

/*
 * ipipe_critical_enter() -- Grab the superlock excluding all CPUs
 * but the current one from a critical section. This lock is used when
 * we must enforce a global critical section for a single CPU in a
 * possibly SMP system whichever context the CPUs are running.
 */
unsigned long ipipe_critical_enter(void (*syncfn) (void))
{
	unsigned long flags;

	local_irq_save_hw(flags);

#ifdef CONFIG_SMP
	if (num_online_cpus() > 1) {	/* We might be running a SMP-kernel on a UP box... */
		int cpu = ipipe_processor_id();
		cpumask_t lock_map;

		if (!cpu_test_and_set(cpu, __ipipe_cpu_lock_map)) {
			while (cpu_test_and_set(BITS_PER_LONG - 1,
						__ipipe_cpu_lock_map)) {
				int n = 0;
				do {
					cpu_relax();
				} while (++n < cpu);
			}

			spin_lock(&__ipipe_cpu_barrier);

			__ipipe_cpu_sync = syncfn;

			/* Send the sync IPI to all processors but the current one. */
			send_IPI_allbutself(IPIPE_CRITICAL_VECTOR);

			cpus_andnot(lock_map, cpu_online_map,
				    __ipipe_cpu_lock_map);

			while (!cpus_equal(__ipipe_cpu_sync_map, lock_map))
				cpu_relax();
		}

		atomic_inc(&__ipipe_critical_count);
	}
#endif	/* CONFIG_SMP */

	return flags;
}

/* ipipe_critical_exit() -- Release the superlock. */

void ipipe_critical_exit(unsigned long flags)
{
#ifdef CONFIG_SMP
	if (num_online_cpus() > 1) {	/* We might be running a SMP-kernel on a UP box... */
		if (atomic_dec_and_test(&__ipipe_critical_count)) {
			spin_unlock(&__ipipe_cpu_barrier);

			while (!cpus_empty(__ipipe_cpu_sync_map))
				cpu_relax();

			cpu_clear(ipipe_processor_id(), __ipipe_cpu_lock_map);
			cpu_clear(BITS_PER_LONG - 1, __ipipe_cpu_lock_map);
		}
	}
#endif	/* CONFIG_SMP */

	local_irq_restore_hw(flags);
}

#ifdef CONFIG_PREEMPT

asmlinkage void __sched __ipipe_preempt_schedule_irq(void)
{
	extern asmlinkage void __sched preempt_schedule_irq(void);
	struct ipipe_percpu_domain_data *p;
	unsigned long flags;
	/*
	 * preempt_schedule_irq expects the root stage to be stalled.
	 */
	BUG_ON(!irqs_disabled_hw());
	local_irq_save(flags);
	local_irq_enable_hw();
	preempt_schedule_irq(); /* Ok, may reschedule now. */
	local_irq_disable_hw();

	/*
	 * Flush any pending interrupt that may have been logged after
	 * preempt_schedule_irq() stalled the root stage before
	 * returning to us, and now.
	 */
	p = ipipe_root_cpudom_ptr();
	if (unlikely(p->irqpend_himask != 0)) {
		add_preempt_count(PREEMPT_ACTIVE);
		clear_bit(IPIPE_STALL_FLAG, &p->status);
		__ipipe_sync_pipeline(IPIPE_IRQMASK_ANY);
		sub_preempt_count(PREEMPT_ACTIVE);
	}

	__local_irq_restore_nosync(flags);
}

#endif	/* CONFIG_PREEMPT */

asmlinkage int __ipipe_check_root(void)
{
	return ipipe_root_domain_p;
}

asmlinkage int __ipipe_check_root_interruptible(void)
{
        return ipipe_root_domain_p && !__ipipe_test_root();
}

__kprobes int 
__ipipe_switch_to_notifier_call_chain(struct atomic_notifier_head *nh,
				      unsigned long val, void *v)
{
        unsigned long flags;
        int rc;

        local_irq_save(flags);
        rc = atomic_notifier_call_chain(nh, val, v);
        __local_irq_restore_nosync(flags);

        return rc;
}

asmlinkage int __ipipe_syscall_root(unsigned long scno, struct pt_regs *regs)
{
	unsigned long flags, origr7;

	/* We use r7 to pass the syscall number to the other domains */
	origr7 = regs->ARM_r7;
	regs->ARM_r7 = __NR_SYSCALL_BASE + scno;
	/*
	 * This routine either returns:
	 * 0 -- if the syscall is to be passed to Linux;
	 * >0 -- if the syscall should not be passed to Linux, and no
	 * tail work should be performed;
	 * <0 -- if the syscall should not be passed to Linux but the
	 * tail work has to be performed (for handling signals etc).
	 */

	if (__ipipe_syscall_watched_p(current, regs->ARM_r7) &&
	    __ipipe_event_monitored_p(IPIPE_EVENT_SYSCALL) &&
	    __ipipe_dispatch_event(IPIPE_EVENT_SYSCALL,regs) > 0) {
		if (ipipe_root_domain_p && !in_atomic()) {
			/*
			 * Sync pending VIRQs before _TIF_NEED_RESCHED
			 * is tested.
			 */
			local_irq_save_hw(flags);
			if ((ipipe_root_cpudom_var(irqpend_himask) & IPIPE_IRQMASK_VIRT) != 0)
				__ipipe_sync_pipeline(IPIPE_IRQMASK_VIRT);
			local_irq_restore_hw(flags);
			regs->ARM_r7 = origr7;
			return -1;
		}
		regs->ARM_r7 = origr7;
		return 1;
	}

	regs->ARM_r7 = origr7;
	return 0;
}

/*
 * __ipipe_handle_irq() -- IPIPE's generic IRQ handler. An optimistic
 * interrupt protection log is maintained here for each domain. Hw
 * interrupts are off on entry.
 */
int __ipipe_handle_irq(int irq, struct pt_regs *regs)
{
	struct ipipe_domain *this_domain, *next_domain;
	struct list_head *head, *pos;
	int m_ack;

	m_ack = (regs == NULL);

	if (irq >= IPIPE_NR_IRQS) {
		printk(KERN_ERR "I-pipe: spurious interrupt %d\n", irq);
		goto finalize_nosync;
	}

	this_domain = ipipe_current_domain;

	if (test_bit(IPIPE_STICKY_FLAG, &this_domain->irqs[irq].control))
		head = &this_domain->p_link;
        else {
                head = __ipipe_pipeline.next;
                next_domain = list_entry(head, struct ipipe_domain, p_link);
                if (likely(test_bit(IPIPE_WIRED_FLAG, &next_domain->irqs[irq].control))) {
                        if (!m_ack && next_domain->irqs[irq].acknowledge != NULL)
                                next_domain->irqs[irq].acknowledge(irq, irq_desc + irq);
                        __ipipe_dispatch_wired(next_domain, irq);
                        goto finalize_nosync;
                }
        }

	/* Ack the interrupt. */

	pos = head;

	while (pos != &__ipipe_pipeline) {
		next_domain = list_entry(pos, struct ipipe_domain, p_link);

		/*
		 * For each domain handling the incoming IRQ, mark it
		 * as pending in its log.
		 */
		if (test_bit(IPIPE_HANDLE_FLAG, &next_domain->irqs[irq].control)) {
			/*
			 * Domains that handle this IRQ are polled for
			 * acknowledging it by decreasing priority
			 * order. The interrupt must be made pending
			 * _first_ in the domain's status flags before
			 * the PIC is unlocked.
			 */
			__ipipe_set_irq_pending(next_domain, irq);

			if (!m_ack && next_domain->irqs[irq].acknowledge) {
                                next_domain->irqs[irq].acknowledge(irq, irq_desc + irq);
                                m_ack = 1;
                        }
		}

		/*
		 * If the domain does not want the IRQ to be passed
		 * down the interrupt pipe, exit the loop now.
		 */

		if (!test_bit(IPIPE_PASS_FLAG, &next_domain->irqs[irq].control))
			break;

		pos = next_domain->p_link.next;
	}

	/*
	 * If the interrupt preempted the head domain, then do not
	 * even try to walk the pipeline, unless an interrupt is
	 * pending for it.
	 */
	if (test_bit(IPIPE_AHEAD_FLAG, &this_domain->flags) &&
	    ipipe_head_cpudom_var(irqpend_himask) == 0)
		goto finalize_nosync;

	/*
	 * Now walk the pipeline, yielding control to the highest
	 * priority domain that has pending interrupt(s) or
	 * immediately to the current domain if the interrupt has been
	 * marked as 'sticky'. This search does not go beyond the
	 * current domain in the pipeline.
	 */

	__ipipe_walk_pipeline(head);

finalize_nosync:
        if (!ipipe_root_domain_p || __ipipe_test_root())
                return 0;

#ifdef CONFIG_SMP
	/*
	 * Prevent a spurious rescheduling from being triggered on
	 * preemptible kernels along the way out through
	 * ret_from_intr.
	 */
        if (!regs)
                __set_bit(IPIPE_STALL_FLAG, &ipipe_root_cpudom_var(status));
#endif	/* CONFIG_SMP */

        return 1;
}

asmlinkage int __ipipe_grab_irq(int irq, struct pt_regs *regs)
{
        int status;

#ifdef irq_finish
	/* AT91 specific workaround */
        irq_finish(irq);
#endif /* irq_finish */

	if (irq == __ipipe_mach_timerint) {
                /*
		 * Given our deferred dispatching model for regular IRQs, we
                 * only record CPU regs for the last timer interrupt, so that
                 * the timer handler charges CPU times properly. It is assumed
                 * that other interrupt handlers don't actually care for such
                 * information.
		 */
		__raw_get_cpu_var(__ipipe_tick_regs).ARM_cpsr =
                        (ipipe_root_domain_p
                         ? regs->ARM_cpsr
                         : regs->ARM_cpsr | PSR_I_BIT);
		__raw_get_cpu_var(__ipipe_tick_regs).ARM_pc = regs->ARM_pc;
	}

#ifdef CONFIG_IPIPE_TRACE_IRQSOFF
	ipipe_trace_begin(regs->ARM_ORIG_r0);
#endif

	if (__ipipe_mach_irq_mux_p(irq)) {
                __ipipe_mach_demux_irq(irq, regs);
                status = ipipe_root_domain_p && !__ipipe_test_root();
        } else
		status = __ipipe_handle_irq(irq, regs);

#ifdef CONFIG_IPIPE_TRACE_IRQSOFF
	ipipe_trace_end(regs->ARM_ORIG_r0);
#endif

        return status;
}

EXPORT_SYMBOL_GPL(show_stack);
#ifndef MULTI_CPU
EXPORT_SYMBOL_GPL(cpu_do_switch_mm);
#endif
EXPORT_SYMBOL_GPL(__check_kvm_seq);
#if defined(CONFIG_SMP) || defined(CONFIG_DEBUG_SPINLOCK)
EXPORT_SYMBOL_GPL(tasklist_lock);
#endif /* CONFIG_SMP || CONFIG_DEBUG_SPINLOCK */

#ifdef CONFIG_CPU_HAS_ASID
EXPORT_SYMBOL_GPL(__new_context);
EXPORT_SYMBOL_GPL(cpu_last_asid);
#endif /* CONFIG_CPU_HAS_ASID */

