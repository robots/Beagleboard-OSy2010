/*
 * arch/arm/mach-pxa/time.c
 *
 * PXA clocksource, clockevents, and OST interrupt handlers.
 * Copyright (c) 2007 by Bill Gatliff <bgat@billgatliff.com>.
 *
 * Derived from Nicolas Pitre's PXA timer handler Copyright (c) 2001
 * by MontaVista Software, Inc.  (Nico, your code rocks!)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/clockchips.h>
#include <linux/sched.h>
#include <linux/cnt32_to_63.h>

#include <asm/div64.h>
#include <asm/mach/irq.h>
#include <asm/mach/time.h>
#include <mach/hardware.h>
#include <mach/pxa-regs.h>

#ifdef CONFIG_IPIPE
#ifdef CONFIG_NO_IDLE_HZ
#error "dynamic tick timer not yet supported with IPIPE"
#endif /* CONFIG_NO_IDLE_HZ */
int __ipipe_mach_timerint = IRQ_OST0;
EXPORT_SYMBOL(__ipipe_mach_timerint);

int __ipipe_mach_timerstolen = 0;
EXPORT_SYMBOL(__ipipe_mach_timerstolen);

unsigned int __ipipe_mach_ticks_per_jiffy = LATCH;
EXPORT_SYMBOL(__ipipe_mach_ticks_per_jiffy);

static int pxa_timer_initialized;

union tsc_reg {
#ifdef __BIG_ENDIAN
	struct {
		unsigned long high;
		unsigned long low;
	};
#else /* __LITTLE_ENDIAN */
	struct {
		unsigned long low;
		unsigned long high;
	};
#endif /* __LITTLE_ENDIAN */
	unsigned long long full;
};

#ifdef CONFIG_SMP
static union tsc_reg tsc[NR_CPUS];

void __ipipe_mach_get_tscinfo(struct __ipipe_tscinfo *info)
{
	info->type = IPIPE_TSC_TYPE_NONE;
}

#else /* !CONFIG_SMP */
static union tsc_reg *tsc;

void __ipipe_mach_get_tscinfo(struct __ipipe_tscinfo *info)
{
	info->type = IPIPE_TSC_TYPE_FREERUNNING;
	info->u.fr.counter = (unsigned *) 0x40A00010;
	info->u.fr.mask = 0xffffffff;
	info->u.fr.tsc = &tsc->full;
}
#endif /* !CONFIG_SMP */

static void ipipe_mach_update_tsc(void);

#endif /* CONFIG_IPIPE */

/*
 * This is PXA's sched_clock implementation. This has a resolution
 * of at least 308 ns and a maximum value of 208 days.
 *
 * The return value is guaranteed to be monotonic in that range as
 * long as there is always less than 582 seconds between successive
 * calls to sched_clock() which should always be the case in practice.
 */

#define OSCR2NS_SCALE_FACTOR 10

static unsigned long oscr2ns_scale;

static void __init set_oscr2ns_scale(unsigned long oscr_rate)
{
	unsigned long long v = 1000000000ULL << OSCR2NS_SCALE_FACTOR;
	do_div(v, oscr_rate);
	oscr2ns_scale = v;
	/*
	 * We want an even value to automatically clear the top bit
	 * returned by cnt32_to_63() without an additional run time
	 * instruction. So if the LSB is 1 then round it up.
	 */
	if (oscr2ns_scale & 1)
		oscr2ns_scale++;
}

unsigned long long sched_clock(void)
{
	unsigned long long v = cnt32_to_63(OSCR);
	return (v * oscr2ns_scale) >> OSCR2NS_SCALE_FACTOR;
}


#define MIN_OSCR_DELTA 16

static irqreturn_t
pxa_ost0_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *c = dev_id;

	/* Disarm the compare/match, signal the event. */
#ifndef CONFIG_IPIPE
	OIER &= ~OIER_E0;
	OSSR = OSSR_M0;
#else /* CONFIG_IPIPE */
	ipipe_mach_update_tsc();
#endif /* CONFIG_IPIPE */
	c->event_handler(c);

	return IRQ_HANDLED;
}

static int
pxa_osmr0_set_next_event(unsigned long delta, struct clock_event_device *dev)
{
	unsigned long flags, next, oscr;

	raw_local_irq_save(flags);
	next = OSCR + delta;
	OIER |= OIER_E0;
	OSMR0 = next;
	oscr = OSCR;
	raw_local_irq_restore(flags);

	return (signed)(next - oscr) <= MIN_OSCR_DELTA ? -ETIME : 0;
}

static void
pxa_osmr0_set_mode(enum clock_event_mode mode, struct clock_event_device *dev)
{
	unsigned long irqflags;

	switch (mode) {
	case CLOCK_EVT_MODE_ONESHOT:
		raw_local_irq_save(irqflags);
		OIER &= ~OIER_E0;
		OSSR = OSSR_M0;
		raw_local_irq_restore(irqflags);
		break;

	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
		/* initializing, released, or preparing for suspend */
		raw_local_irq_save(irqflags);
		OIER &= ~OIER_E0;
		OSSR = OSSR_M0;
		raw_local_irq_restore(irqflags);
		break;

	case CLOCK_EVT_MODE_RESUME:
	case CLOCK_EVT_MODE_PERIODIC:
		break;
	}
}

static struct clock_event_device ckevt_pxa_osmr0 = {
	.name		= "osmr0",
	.features	= CLOCK_EVT_FEAT_ONESHOT,
	.shift		= 32,
	.rating		= 200,
	.set_next_event	= pxa_osmr0_set_next_event,
	.set_mode	= pxa_osmr0_set_mode,
};

#ifdef CONFIG_IPIPE
int __ipipe_check_tickdev(const char *devname)
{
	return !strcmp(devname, ckevt_pxa_osmr0.name);
}
#endif /* CONFIG_IPIPE */

static cycle_t pxa_read_oscr(void)
{
	return OSCR;
}

static struct clocksource cksrc_pxa_oscr0 = {
	.name           = "oscr0",
	.rating         = 200,
	.read           = pxa_read_oscr,
	.mask           = CLOCKSOURCE_MASK(32),
	.shift          = 20,
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static struct irqaction pxa_ost0_irq = {
	.name		= "ost0",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= pxa_ost0_interrupt,
	.dev_id		= &ckevt_pxa_osmr0,
};

static void __init pxa_timer_init(void)
{
	unsigned long clock_tick_rate = get_clock_tick_rate();

	OIER = 0;
	OSSR = OSSR_M0 | OSSR_M1 | OSSR_M2 | OSSR_M3;

	set_oscr2ns_scale(clock_tick_rate);

	ckevt_pxa_osmr0.mult =
		div_sc(clock_tick_rate, NSEC_PER_SEC, ckevt_pxa_osmr0.shift);
	ckevt_pxa_osmr0.max_delta_ns =
		clockevent_delta2ns(0x7fffffff, &ckevt_pxa_osmr0);
	ckevt_pxa_osmr0.min_delta_ns =
		clockevent_delta2ns(MIN_OSCR_DELTA * 2, &ckevt_pxa_osmr0) + 1;
	ckevt_pxa_osmr0.cpumask = cpumask_of(0);

	cksrc_pxa_oscr0.mult =
		clocksource_hz2mult(clock_tick_rate, cksrc_pxa_oscr0.shift);

	setup_irq(IRQ_OST0, &pxa_ost0_irq);

#ifdef CONFIG_IPIPE
#ifndef CONFIG_SMP
	tsc = (union tsc_reg *) __ipipe_tsc_area;
	barrier();
#endif /* CONFIG_SMP */

	pxa_timer_initialized = 1;
#endif /* CONFIG_IPIPE */

	clocksource_register(&cksrc_pxa_oscr0);
	clockevents_register_device(&ckevt_pxa_osmr0);
}

#ifdef CONFIG_PM
static unsigned long osmr[4], oier, oscr;

static void pxa_timer_suspend(void)
{
	osmr[0] = OSMR0;
	osmr[1] = OSMR1;
	osmr[2] = OSMR2;
	osmr[3] = OSMR3;
	oier = OIER;
	oscr = OSCR;
}

static void pxa_timer_resume(void)
{
	/*
	 * Ensure that we have at least MIN_OSCR_DELTA between match
	 * register 0 and the OSCR, to guarantee that we will receive
	 * the one-shot timer interrupt.  We adjust OSMR0 in preference
	 * to OSCR to guarantee that OSCR is monotonically incrementing.
	 */
	if (osmr[0] - oscr < MIN_OSCR_DELTA)
		osmr[0] += MIN_OSCR_DELTA;

	OSMR0 = osmr[0];
	OSMR1 = osmr[1];
	OSMR2 = osmr[2];
	OSMR3 = osmr[3];
	OIER = oier;
	OSCR = oscr;
}
#else
#define pxa_timer_suspend NULL
#define pxa_timer_resume NULL
#endif

struct sys_timer pxa_timer = {
	.init		= pxa_timer_init,
	.suspend	= pxa_timer_suspend,
	.resume		= pxa_timer_resume,
};

#ifdef CONFIG_IPIPE
void __ipipe_mach_acktimer(void)
{
	OSSR = OSSR_M0;  /* Clear match on timer 0 */
	OIER &= ~OIER_E0;
}

static void ipipe_mach_update_tsc(void)
{
	union tsc_reg *local_tsc;
	unsigned long stamp, flags;

	local_irq_save_hw(flags);
	local_tsc = &tsc[ipipe_processor_id()];
	stamp = OSCR;
	if (unlikely(stamp < local_tsc->low))
		/* 32 bit counter wrapped, increment high word. */
		local_tsc->high++;
	local_tsc->low = stamp;
	local_irq_restore_hw(flags);
}

notrace unsigned long long __ipipe_mach_get_tsc(void)
{
	if (likely(pxa_timer_initialized)) {
		union tsc_reg *local_tsc, result;
		unsigned long stamp;

		local_tsc = &tsc[ipipe_processor_id()];

		__asm__ ("ldmia %1, %M0\n":
			 "=r"(result.full): "r"(local_tsc), "m"(*local_tsc));
		barrier();
		stamp = OSCR;
		if (unlikely(stamp < result.low))
			/* 32 bit counter wrapped, increment high word. */
			result.high++;
		result.low = stamp;

		return result.full;
	}

        return 0;
}
EXPORT_SYMBOL(__ipipe_mach_get_tsc);

/*
 * Reprogram the timer
 */

void __ipipe_mach_set_dec(unsigned long delay)
{
	if (delay > MIN_OSCR_DELTA) {
		unsigned long flags;

		local_irq_save_hw(flags);
		OSMR0 = delay + OSCR;
		OIER |= OIER_E0;
		local_irq_restore_hw(flags);
	} else
		ipipe_trigger_irq(IRQ_OST0);
}
EXPORT_SYMBOL(__ipipe_mach_set_dec);

void __ipipe_mach_release_timer(void)
{
	pxa_osmr0_set_mode(ckevt_pxa_osmr0.mode, &ckevt_pxa_osmr0);
	if (ckevt_pxa_osmr0.mode == CLOCK_EVT_MODE_ONESHOT)
		pxa_osmr0_set_next_event(LATCH, &ckevt_pxa_osmr0);
}
EXPORT_SYMBOL(__ipipe_mach_release_timer);

unsigned long __ipipe_mach_get_dec(void)
{
	return OSMR0 - OSCR;
}
#endif /* CONFIG_IPIPE */
