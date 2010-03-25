/*
 *  linux/arch/arm/mach-imx/time.c
 *
 *  Copyright (C) 2000-2001 Deep Blue Solutions
 *  Copyright (C) 2002 Shane Nay (shane@minirl.com)
 *  Copyright (C) 2006-2007 Pavel Pisa (ppisa@pikron.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/time.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>
#include <linux/clk.h>
#include <linux/io.h>

#include <mach/hardware.h>
#include <asm/leds.h>
#include <asm/irq.h>
#include <asm/mach/time.h>

#ifdef CONFIG_IPIPE
#ifdef CONFIG_NO_IDLE_HZ
#error "dynamic tick timer not yet supported with IPIPE"
#endif				/* CONFIG_NO_IDLE_HZ */
int __ipipe_mach_timerint = TIM1_INT;
EXPORT_SYMBOL(__ipipe_mach_timerint);

int __ipipe_mach_timerstolen = 0;
EXPORT_SYMBOL(__ipipe_mach_timerstolen);

unsigned int __ipipe_mach_ticks_per_jiffy = LATCH;
EXPORT_SYMBOL(__ipipe_mach_ticks_per_jiffy);

static int imx_timer_initialized;
union tsc_reg {
#ifdef __BIG_ENDIAN
	struct {
		unsigned long high;
		unsigned long low;
	};
#else				/* __LITTLE_ENDIAN */
	struct {
		unsigned long low;
		unsigned long high;
	};
#endif				/* __LITTLE_ENDIAN */
	unsigned long long full;
};
#ifdef CONFIG_SMP
static union tsc_reg tsc[NR_CPUS];

void __ipipe_mach_get_tscinfo(struct __ipipe_tscinfo *info)
{
	info->type = IPIPE_TSC_TYPE_NONE;
}
#else				/* !CONFIG_SMP */
static union tsc_reg *tsc;

void __ipipe_mach_get_tscinfo(struct __ipipe_tscinfo *info)
{
	info->type = IPIPE_TSC_TYPE_FREERUNNING;
	info->u.fr.counter = (unsigned *)(0x10 + IMX_TIM1_BASE);
	info->u.fr.mask = 0xffffffff;
	info->u.fr.tsc = &tsc->full;
}
#endif				/* !CONFIG_SMP */

static void ipipe_mach_update_tsc(void);

#endif				/* CONFIG_IPIPE */

/* Use timer 1 as system timer */
#define TIMER_BASE IMX_TIM1_BASE

static struct clock_event_device clockevent_imx;
static enum clock_event_mode clockevent_mode = CLOCK_EVT_MODE_UNUSED;

/*
 * IRQ handler for the timer
 */
static irqreturn_t
imx_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &clockevent_imx;
#ifndef CONFIG_IPIPE
	uint32_t tstat;
	irqreturn_t ret = IRQ_NONE;

	/* clear the interrupt */
	tstat = IMX_TSTAT(TIMER_BASE);
	IMX_TSTAT(TIMER_BASE) = 0;

	if (tstat & TSTAT_COMP) {
		evt->event_handler(evt);
		ret = IRQ_HANDLED;
	}

	return ret;
#else /* CONFIG_IPIPE */
	ipipe_mach_update_tsc();
	evt->event_handler(evt);
	return  IRQ_HANDLED;
#endif /* CONFIG_IPIPE */
}

static struct irqaction imx_timer_irq = {
	.name		= "i.MX Timer Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= imx_timer_interrupt,
};

/*
 * Set up timer hardware into expected mode and state.
 */
static void __init imx_timer_hardware_init(void)
{
	/*
	 * Initialise to a known state (all timers off, and timing reset)
	 */
	IMX_TCTL(TIMER_BASE) = 0;
	IMX_TPRER(TIMER_BASE) = 0;

	IMX_TCTL(TIMER_BASE) = TCTL_FRR | TCTL_CLK_PCLK1 | TCTL_TEN;
}

cycle_t imx_get_cycles(void)
{
	return IMX_TCN(TIMER_BASE);
}

static struct clocksource clocksource_imx = {
	.name 		= "imx_timer1",
	.rating		= 200,
	.read		= imx_get_cycles,
	.mask		= 0xFFFFFFFF,
	.shift 		= 20,
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

static int __init imx_clocksource_init(unsigned long rate)
{
	clocksource_imx.mult =
		clocksource_hz2mult(rate, clocksource_imx.shift);
	clocksource_register(&clocksource_imx);

	return 0;
}

static int imx_set_next_event(unsigned long evt,
				  struct clock_event_device *unused)
{
	unsigned long tcmp;

	tcmp = IMX_TCN(TIMER_BASE) + evt;
	IMX_TCMP(TIMER_BASE) = tcmp;

	return (int32_t)(tcmp - IMX_TCN(TIMER_BASE)) < 0 ? -ETIME : 0;
}

#ifdef DEBUG
static const char *clock_event_mode_label[]={
	[CLOCK_EVT_MODE_PERIODIC] = "CLOCK_EVT_MODE_PERIODIC",
	[CLOCK_EVT_MODE_ONESHOT]  = "CLOCK_EVT_MODE_ONESHOT",
	[CLOCK_EVT_MODE_SHUTDOWN] = "CLOCK_EVT_MODE_SHUTDOWN",
	[CLOCK_EVT_MODE_UNUSED]   = "CLOCK_EVT_MODE_UNUSED"
};
#endif /*DEBUG*/

static void imx_set_mode(enum clock_event_mode mode, struct clock_event_device *evt)
{
	unsigned long flags;

	/*
	 * The timer interrupt generation is disabled at least
	 * for enough time to call imx_set_next_event()
	 */
	local_irq_save_hw(flags);
	/* Disable interrupt in GPT module */
	IMX_TCTL(TIMER_BASE) &= ~TCTL_IRQEN;
	if (mode != clockevent_mode) {
		/* Set event time into far-far future */
		IMX_TCMP(TIMER_BASE) = IMX_TCN(TIMER_BASE) - 3;
		/* Clear pending interrupt */
		IMX_TSTAT(TIMER_BASE) &= ~TSTAT_COMP;
	}

#ifdef DEBUG
	printk(KERN_INFO "imx_set_mode: changing mode from %s to %s\n",
		clock_event_mode_label[clockevent_mode], clock_event_mode_label[mode]);
#endif /*DEBUG*/

	/* Remember timer mode */
	clockevent_mode = mode;
	local_irq_restore_hw(flags);

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		printk(KERN_ERR "imx_set_mode: Periodic mode is not supported for i.MX\n");
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		/*
		 * Do not put overhead of interrupt enable/disable into
		 * imx_set_next_event(), the core has about 4 minutes
		 * to call imx_set_next_event() or shutdown clock after
		 * mode switching
		 */
		local_irq_save_hw(flags);
		IMX_TCTL(TIMER_BASE) |= TCTL_IRQEN;
		local_irq_restore_hw(flags);
		break;
	case CLOCK_EVT_MODE_SHUTDOWN:
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_RESUME:
		/* Left event sources disabled, no more interrupts appears */
		break;
	}
}

#ifdef CONFIG_IPIPE
int __ipipe_check_tickdev(const char *devname)
{
	return !strcmp(devname, clockevent_imx.name);
}
#endif /* CONFIG_IPIPE */

static struct clock_event_device clockevent_imx = {
	.name		= "imx_timer1",
	.features	= CLOCK_EVT_FEAT_ONESHOT,
	.shift		= 32,
	.set_mode	= imx_set_mode,
	.set_next_event	= imx_set_next_event,
	.rating		= 200,
};

static int __init imx_clockevent_init(unsigned long rate)
{
	clockevent_imx.mult = div_sc(rate, NSEC_PER_SEC,
					clockevent_imx.shift);
	clockevent_imx.max_delta_ns =
		clockevent_delta2ns(0xfffffffe, &clockevent_imx);
	clockevent_imx.min_delta_ns =
		clockevent_delta2ns(0xf, &clockevent_imx);

	clockevent_imx.cpumask = cpumask_of(0);

	clockevents_register_device(&clockevent_imx);

	return 0;
}

extern int imx_clocks_init(void);

static void __init imx_timer_init(void)
{
	struct clk *clk;
	unsigned long rate;

	imx_clocks_init();

	clk = clk_get(NULL, "perclk1");
	clk_enable(clk);
	rate = clk_get_rate(clk);

	imx_timer_hardware_init();
	imx_clocksource_init(rate);

	imx_clockevent_init(rate);

#ifdef CONFIG_IPIPE
#ifndef CONFIG_SMP
	tsc = (union tsc_reg *)__ipipe_tsc_area;
	barrier();
#endif				/* CONFIG_SMP */
	imx_timer_initialized = 1;

#endif				/* CONFIG_IPIPE */
	/*
	 * Make irqs happen for the system timer
	 */
	setup_irq(TIM1_INT, &imx_timer_irq);
}

struct sys_timer imx_timer = {
	.init		= imx_timer_init,
};

#ifdef CONFIG_IPIPE
void __ipipe_mach_acktimer(void)
{
	unsigned tstat = IMX_TSTAT(TIMER_BASE);
	IMX_TSTAT(TIMER_BASE) = 0;
}

static void ipipe_mach_update_tsc(void)
{
	union tsc_reg *local_tsc;
	unsigned long stamp, flags;

	local_irq_save_hw(flags);
	local_tsc = &tsc[ipipe_processor_id()];
	stamp = IMX_TCN(TIMER_BASE);
	if (unlikely(stamp < local_tsc->low))
		/* 32 bit counter wrapped, increment high word. */
		local_tsc->high++;
	local_tsc->low = stamp;
	local_irq_restore_hw(flags);
}

notrace unsigned long long __ipipe_mach_get_tsc(void)
{
	if (likely(imx_timer_initialized)) {
		union tsc_reg *local_tsc, result;
		unsigned long stamp;

		local_tsc = &tsc[ipipe_processor_id()];

		__asm__("ldmia %1, %M0\n":
			"=r"(result.full): "r"(local_tsc), "m"(*local_tsc));
		barrier();
		stamp = IMX_TCN(TIMER_BASE);
		if (unlikely(stamp < result.low))
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
	unsigned long flags;
	if (delay > 8) {
		local_irq_save_hw(flags);
		IMX_TCMP(TIMER_BASE) = IMX_TCN(TIMER_BASE) + delay;
		local_irq_restore_hw(flags);
	} else
		ipipe_trigger_irq(TIM1_INT);
}

EXPORT_SYMBOL(__ipipe_mach_set_dec);

void __ipipe_mach_release_timer(void)
{
	__ipipe_mach_set_dec(__ipipe_mach_ticks_per_jiffy);
}

EXPORT_SYMBOL(__ipipe_mach_release_timer);

unsigned long __ipipe_mach_get_dec(void)
{
	return IMX_TCMP(TIMER_BASE) - IMX_TCN(TIMER_BASE);
}
#endif				/* CONFIG_IPIPE */
