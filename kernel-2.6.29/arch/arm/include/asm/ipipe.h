/* -*- linux-c -*-
 * arch/arm/include/asm/ipipe.h
 *
 * Copyright (C) 2002-2005 Philippe Gerum.
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
 */

#ifndef __ARM_IPIPE_H
#define __ARM_IPIPE_H

#ifdef CONFIG_IPIPE

#include <linux/ipipe_percpu.h>
#include <mach/irqs.h>		/* For __IPIPE_FEATURE_PIC_MUTE */

#define IPIPE_ARCH_STRING	"1.13-05"
#define IPIPE_MAJOR_NUMBER	1
#define IPIPE_MINOR_NUMBER	13
#define IPIPE_PATCH_NUMBER	5

#ifdef CONFIG_SMP
#error "I-pipe/arm: SMP not yet implemented"
#define ipipe_processor_id()	(current_thread_info()->cpu)
#else /* !CONFIG_SMP */
#define ipipe_processor_id()	0
#endif	/* CONFIG_SMP */

#define smp_processor_id_hw() ipipe_processor_id()

#ifdef CONFIG_IPIPE_WANT_PREEMPTIBLE_SWITCH

#define prepare_arch_switch(next)			\
	do {						\
		local_irq_enable_hw();			\
		ipipe_schedule_notify(current, next);	\
	} while(0)

#define task_hijacked(p)						\
	({								\
		int x = !ipipe_root_domain_p;				\
		clear_bit(IPIPE_SYNC_FLAG, &ipipe_root_cpudom_var(status)); \
		x;							\
	})

#else /* !CONFIG_IPIPE_WANT_PREEMPTIBLE_SWITCH */

#define prepare_arch_switch(next)			\
	do {                                            \
		ipipe_schedule_notify(current ,next);   \
		local_irq_disable_hw();                 \
	} while(0)

#define task_hijacked(p)						\
	({								\
		int x = !ipipe_root_domain_p;                           \
		__clear_bit(IPIPE_SYNC_FLAG, &ipipe_root_cpudom_var(status)); \
		if (!x) local_irq_enable_hw(); x;			\
	})

#endif /* !CONFIG_IPIPE_WANT_PREEMPTIBLE_SWITCH */

extern unsigned long arm_return_addr(int level);

#define BROKEN_BUILTIN_RETURN_ADDRESS
#define __BUILTIN_RETURN_ADDRESS0 arm_return_addr(0)
#define __BUILTIN_RETURN_ADDRESS1 arm_return_addr(1)


struct ipipe_domain;

#define IPIPE_TSC_TYPE_NONE	   0
#define IPIPE_TSC_TYPE_FREERUNNING 1
#define IPIPE_TSC_TYPE_DECREMENTER 2

struct __ipipe_tscinfo {
	unsigned type;
	union {
		struct {
			unsigned *counter; /* Hw counter physical address */
			unsigned mask; /* Significant bits in the hw counter. */
			unsigned long long *tsc; /* 64 bits tsc value. */
		} fr;
		struct {
			unsigned *counter; /* Hw counter physical address */
			unsigned mask; /* Significant bits in the hw counter. */
			unsigned *last_cnt; /* Counter value when updating
						tsc value. */
			unsigned long long *tsc; /* 64 bits tsc value. */
		} dec;
	} u;
};

struct ipipe_sysinfo {

	int ncpus;		/* Number of CPUs on board */
	u64 cpufreq;		/* CPU frequency (in Hz) */

	/* Arch-dependent block */

	struct {
		unsigned tmirq;	/* Timer tick IRQ */
		u64 tmfreq;	/* Timer frequency */
		struct __ipipe_tscinfo tsc; /* exported data for u.s. tsc */
	} archdep;
};

DECLARE_PER_CPU(struct mm_struct *,ipipe_active_mm);
/* arch specific stuff */
extern void *__ipipe_tsc_area;
extern int __ipipe_mach_timerint;
extern int __ipipe_mach_timerstolen;
extern unsigned int __ipipe_mach_ticks_per_jiffy;
extern void __ipipe_mach_acktimer(void);
extern unsigned long long __ipipe_mach_get_tsc(void);
extern void __ipipe_mach_set_dec(unsigned long);
extern void __ipipe_mach_release_timer(void);
extern unsigned long __ipipe_mach_get_dec(void);
extern void __ipipe_mach_demux_irq(unsigned irq, struct pt_regs *regs);
void __ipipe_mach_get_tscinfo(struct __ipipe_tscinfo *info);
int __ipipe_check_tickdev(const char *devname);

#define ipipe_read_tsc(t)		do { t = __ipipe_mach_get_tsc(); } while (0)
#define __ipipe_read_timebase()		__ipipe_mach_get_tsc()

#define ipipe_cpu_freq()	(HZ * __ipipe_mach_ticks_per_jiffy)
#define ipipe_tsc2ns(t) \
({ \
	unsigned long long delta = (t)*1000; \
	do_div(delta, ipipe_cpu_freq() / 1000000 + 1); \
	(unsigned long)delta; \
})
#define ipipe_tsc2us(t) \
({ \
	unsigned long long delta = (t); \
	do_div(delta, ipipe_cpu_freq() / 1000000 + 1); \
	(unsigned long)delta; \
})

/* Private interface -- Internal use only */

#define __ipipe_check_platform()	do { } while(0)

#define __ipipe_init_platform()		do { } while(0)

#define __ipipe_enable_irq(irq)	irq_desc[irq].chip->enable(irq)

#define __ipipe_disable_irq(irq)	irq_desc[irq].chip->disable(irq)

#define __ipipe_hook_critical_ipi(ipd) do { } while(0)

void __ipipe_enable_irqdesc(struct ipipe_domain *ipd, unsigned irq);

#ifndef __IPIPE_FEATURE_PIC_MUTE
#define __ipipe_disable_irqdesc(ipd, irq) do { } while (0)
#else /* __IPIPE_FEATURE_PIC_MUTE */

typedef unsigned long
__ipipe_irqbits_t[(NR_IRQS + BITS_PER_LONG - 1) / BITS_PER_LONG];
extern __ipipe_irqbits_t __ipipe_irqbits;

void __ipipe_disable_irqdesc(struct ipipe_domain *ipd, unsigned irq);

void ipipe_mute_pic(void);

void ipipe_unmute_pic(void);
#endif /* __IPIPE_FEATURE_PIC_MUTE */

void __ipipe_enable_pipeline(void);

void __ipipe_do_critical_sync(unsigned irq,
			      void *cookie);

DECLARE_PER_CPU(struct pt_regs, __ipipe_tick_regs);

int __ipipe_handle_irq(int irq,
		       struct pt_regs *regs);

#define ipipe_update_tick_evtdev(evtdev) do { } while (0)

#define __ipipe_tick_irq	__ipipe_mach_timerint

static inline unsigned long __ipipe_ffnz(unsigned long ul)
{
	return ffs(ul) - 1;
}

/* When running handlers, enable hw interrupts for all domains but the
 * one heading the pipeline, so that IRQs can never be significantly
 * deferred for the latter. */
#define __ipipe_run_isr(ipd, irq)					\
do {									\
	if (!__ipipe_pipeline_head_p(ipd))				\
		local_irq_enable_hw();					\
	if (ipd == ipipe_root_domain) {					\
		if (likely(!ipipe_virtual_irq_p(irq)))			\
			((void (*)(unsigned, struct pt_regs *))		\
			 ipd->irqs[irq].handler) (irq,			\
						  &__raw_get_cpu_var(__ipipe_tick_regs)); \
		else {							\
			irq_enter();					\
			ipd->irqs[irq].handler(irq,ipd->irqs[irq].cookie); \
			irq_exit();					\
		}							\
	} else {							\
		__clear_bit(IPIPE_SYNC_FLAG, &ipipe_cpudom_var(ipd, status)); \
		ipd->irqs[irq].handler(irq,ipd->irqs[irq].cookie);	\
		__set_bit(IPIPE_SYNC_FLAG, &ipipe_cpudom_var(ipd, status)); \
	}								\
	local_irq_disable_hw();						\
} while(0)

#define __ipipe_syscall_watched_p(p, sc)				\
	(((p)->flags & PF_EVNOTIFY) || (unsigned long)sc >= __ARM_NR_BASE + 64)

#define __ipipe_root_tick_p(regs) (!raw_irqs_disabled_flags(regs->ARM_cpsr))

#else /* !CONFIG_IPIPE */

#define task_hijacked(p)		0

#define ipipe_update_tick_evtdev(evtdev)	do { } while (0)

#define smp_processor_id_hw()		smp_processor_id()

#endif /* CONFIG_IPIPE */

#endif	/* !__ARM_IPIPE_H */
