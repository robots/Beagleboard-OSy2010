/*
 * linux/arch/arm/mach-sa1100/time.c
 *
 * Copyright (C) 1998 Deborah Wallach.
 * Twiddles  (C) 1999 Hugo Fiennes <hugo@empeg.com>
 *
 * 2000/03/29 (C) Nicolas Pitre <nico@cam.org>
 *	Rewritten: big cleanup, much simpler, better HZ accuracy.
 *
 */
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/timex.h>
#include <linux/clockchips.h>

#include <asm/mach/time.h>
#include <mach/hardware.h>

#define MIN_OSCR_DELTA 2

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

static int sa1100_timer_initialized;

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
	info->u.fr.counter = (unsigned *)0x90000010;
	info->u.fr.mask = 0xffffffff;
	info->u.fr.tsc = &tsc->full;
}
#endif /* !CONFIG_SMP */

static void ipipe_mach_update_tsc(void);

#endif /* CONFIG_IPIPE */

static irqreturn_t sa1100_ost0_interrupt(int irq, void *dev_id)
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
sa1100_osmr0_set_next_event(unsigned long delta, struct clock_event_device *c)
{
	unsigned long flags, next, oscr;

	raw_local_irq_save(flags);
	OIER |= OIER_E0;
	next = OSCR + delta;
	OSMR0 = next;
	oscr = OSCR;
	raw_local_irq_restore(flags);

	return (signed)(next - oscr) <= MIN_OSCR_DELTA ? -ETIME : 0;
}

static void
sa1100_osmr0_set_mode(enum clock_event_mode mode, struct clock_event_device *c)
{
	unsigned long flags;

	switch (mode) {
	case CLOCK_EVT_MODE_ONESHOT:
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
		raw_local_irq_save(flags);
		OIER &= ~OIER_E0;
		OSSR = OSSR_M0;
		raw_local_irq_restore(flags);
		break;

	case CLOCK_EVT_MODE_RESUME:
	case CLOCK_EVT_MODE_PERIODIC:
		break;
	}
}

static struct clock_event_device ckevt_sa1100_osmr0 = {
	.name		= "osmr0",
	.features	= CLOCK_EVT_FEAT_ONESHOT,
	.shift		= 32,
	.rating		= 200,
	.set_next_event	= sa1100_osmr0_set_next_event,
	.set_mode	= sa1100_osmr0_set_mode,
};

#ifdef CONFIG_IPIPE
int __ipipe_check_tickdev(const char *devname)
{
	return !strcmp(devname, ckevt_sa1100_osmr0.name);
}
#endif /* CONFIG_IPIPE */

static cycle_t sa1100_read_oscr(void)
{
	return OSCR;
}

static struct clocksource cksrc_sa1100_oscr = {
	.name		= "oscr",
	.rating		= 200,
	.read		= sa1100_read_oscr,
	.mask		= CLOCKSOURCE_MASK(32),
	.shift		= 20,
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static struct irqaction sa1100_timer_irq = {
	.name		= "ost0",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= sa1100_ost0_interrupt,
	.dev_id		= &ckevt_sa1100_osmr0,
};

static void __init sa1100_timer_init(void)
{
	OIER = 0;		/* disable any timer interrupts */
	OSSR = 0xf;		/* clear status on all timers */

	ckevt_sa1100_osmr0.mult =
		div_sc(3686400, NSEC_PER_SEC, ckevt_sa1100_osmr0.shift);
	ckevt_sa1100_osmr0.max_delta_ns =
		clockevent_delta2ns(0x7fffffff, &ckevt_sa1100_osmr0);
	ckevt_sa1100_osmr0.min_delta_ns =
		clockevent_delta2ns(MIN_OSCR_DELTA * 2, &ckevt_sa1100_osmr0) + 1;
	ckevt_sa1100_osmr0.cpumask = cpumask_of(0);

	cksrc_sa1100_oscr.mult =
		clocksource_hz2mult(CLOCK_TICK_RATE, cksrc_sa1100_oscr.shift);

	setup_irq(IRQ_OST0, &sa1100_timer_irq);

#ifdef CONFIG_IPIPE
#ifndef CONFIG_SMP
	tsc = (union tsc_reg *) __ipipe_tsc_area;
	barrier();
#endif /* CONFIG_SMP */

	sa1100_timer_initialized = 1;
#endif /* CONFIG_IPIPE */

	clocksource_register(&cksrc_sa1100_oscr);
	clockevents_register_device(&ckevt_sa1100_osmr0);
}

#ifdef CONFIG_PM
unsigned long osmr[4], oier;

static void sa1100_timer_suspend(void)
{
	osmr[0] = OSMR0;
	osmr[1] = OSMR1;
	osmr[2] = OSMR2;
	osmr[3] = OSMR3;
	oier = OIER;
}

static void sa1100_timer_resume(void)
{
	OSSR = 0x0f;
	OSMR0 = osmr[0];
	OSMR1 = osmr[1];
	OSMR2 = osmr[2];
	OSMR3 = osmr[3];
	OIER = oier;

	/*
	 * OSMR0 is the system timer: make sure OSCR is sufficiently behind
	 */
	OSCR = OSMR0 - LATCH;
}
#else
#define sa1100_timer_suspend NULL
#define sa1100_timer_resume NULL
#endif

struct sys_timer sa1100_timer = {
	.init		= sa1100_timer_init,
	.suspend	= sa1100_timer_suspend,
	.resume		= sa1100_timer_resume,
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
	if (likely(sa1100_timer_initialized)) {
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
	sa1100_osmr0_set_mode(ckevt_sa1100_osmr0.mode, &ckevt_sa1100_osmr0);
	if (ckevt_sa1100_osmr0.mode == CLOCK_EVT_MODE_ONESHOT)
		sa1100_osmr0_set_next_event(LATCH, &ckevt_sa1100_osmr0);
}
EXPORT_SYMBOL(__ipipe_mach_release_timer);

unsigned long __ipipe_mach_get_dec(void)
{
	return OSMR0 - OSCR;
}
#endif /* CONFIG_IPIPE */
