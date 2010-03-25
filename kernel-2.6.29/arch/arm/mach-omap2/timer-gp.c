/*
 * linux/arch/arm/mach-omap2/timer-gp.c
 *
 * OMAP2 GP timer support.
 *
 * Update to use new clocksource/clockevent layers
 * Author: Kevin Hilman, MontaVista Software, Inc. <source@mvista.com>
 * Copyright (C) 2007 MontaVista Software, Inc.
 *
 * Original driver:
 * Copyright (C) 2005 Nokia Corporation
 * Author: Paul Mundt <paul.mundt@nokia.com>
 *         Juha Yrjölä <juha.yrjola@nokia.com>
 * OMAP Dual-mode timer framework support by Timo Teras
 *
 * Some parts based off of TI's 24xx code:
 *
 *   Copyright (C) 2004 Texas Instruments, Inc.
 *
 * Roughly modelled after the OMAP1 MPU timer code.
 *
 * This file is subject to the terms and conditions of the GNU General Public
 * License. See the file "COPYING" in the main directory of this archive
 * for more details.
 */
#include <linux/init.h>
#include <linux/time.h>
#include <linux/interrupt.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>

#include <asm/mach/time.h>
#include <mach/dmtimer.h>

static struct omap_dm_timer *gptimer;
static struct clock_event_device clockevent_gpt;
#ifndef CONFIG_OMAP_32K_TIMER
static struct omap_dm_timer *gpt_clocksource;
#endif /* !CONFIG_OMAP_32K_TIMER */

#ifdef CONFIG_IPIPE
#ifdef CONFIG_NO_IDLE_HZ
#error "dynamic tick timer not yet supported with IPIPE"
#endif /* CONFIG_NO_IDLE_HZ */
int __ipipe_mach_timerint;
EXPORT_SYMBOL(__ipipe_mach_timerint);

int __ipipe_mach_timerstolen;
EXPORT_SYMBOL(__ipipe_mach_timerstolen);

unsigned int __ipipe_mach_ticks_per_jiffy;
EXPORT_SYMBOL(__ipipe_mach_ticks_per_jiffy);

static int omap2_timer_initialized;

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
	info->u.fr.counter = (unsigned *)
		omap_dm_timer_get_phys_counter_addr(gpt_clocksource);
	info->u.fr.mask = 0xffffffff;
	info->u.fr.tsc = &tsc->full;
}
#endif /* !CONFIG_SMP */

static void ipipe_mach_update_tsc(void);

#endif /* CONFIG_IPIPE */
static irqreturn_t omap2_gp_timer_interrupt(int irq, void *dev_id)
{
	struct clock_event_device *evt = &clockevent_gpt;
#ifndef CONFIG_IPIPE
	struct omap_dm_timer *gpt = (struct omap_dm_timer *)dev_id;

	omap_dm_timer_write_status(gpt, OMAP_TIMER_INT_OVERFLOW);
#else /* CONFIG_IPIPE */
	ipipe_mach_update_tsc();
#endif /* CONFIG_IPIPE */

	evt->event_handler(evt);
	return IRQ_HANDLED;
}

static struct irqaction omap2_gp_timer_irq = {
	.name		= "gp timer",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= omap2_gp_timer_interrupt,
};

static int omap2_gp_timer_set_next_event(unsigned long cycles,
					 struct clock_event_device *evt)
{
	omap_dm_timer_set_load_start(gptimer, 0, 0xffffffff - cycles);

	return 0;
}

static void omap2_gp_timer_set_mode(enum clock_event_mode mode,
				    struct clock_event_device *evt)
{
	u32 period;

	omap_dm_timer_stop(gptimer);

	switch (mode) {
	case CLOCK_EVT_MODE_PERIODIC:
		period = clk_get_rate(omap_dm_timer_get_fclk(gptimer)) / HZ;
		period -= 1;

		omap_dm_timer_set_load_start(gptimer, 1, 0xffffffff - period);
		break;
	case CLOCK_EVT_MODE_ONESHOT:
		break;
	case CLOCK_EVT_MODE_UNUSED:
	case CLOCK_EVT_MODE_SHUTDOWN:
	case CLOCK_EVT_MODE_RESUME:
		break;
	}
}

static struct clock_event_device clockevent_gpt = {
	.name		= "gp timer",
	.features       = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
	.shift		= 32,
	.set_next_event	= omap2_gp_timer_set_next_event,
	.set_mode	= omap2_gp_timer_set_mode,
};

#ifdef CONFIG_IPIPE
int __ipipe_check_tickdev(const char *devname)
{
	return !strcmp(devname, clockevent_gpt.name);
}
#endif /* CONFIG_IPIPE */

static void __init omap2_gp_clockevent_init(void)
{
	u32 tick_rate;

	gptimer = omap_dm_timer_request_specific(CONFIG_OMAP_TICK_GPTIMER);
	BUG_ON(gptimer == NULL);

#if defined(CONFIG_OMAP_32K_TIMER)
	omap_dm_timer_set_source(gptimer, OMAP_TIMER_SRC_32_KHZ);
#else
	omap_dm_timer_set_source(gptimer, OMAP_TIMER_SRC_SYS_CLK);
#endif
	tick_rate = clk_get_rate(omap_dm_timer_get_fclk(gptimer));

	pr_info("OMAP clockevent source: GPTIMER%d at %u Hz\n",
		CONFIG_OMAP_TICK_GPTIMER, tick_rate);

	omap2_gp_timer_irq.dev_id = (void *)gptimer;

#ifdef CONFIG_IPIPE
	__ipipe_mach_timerint = omap_dm_timer_get_irq(gptimer);
	__ipipe_mach_ticks_per_jiffy = (tick_rate + HZ / 2) / HZ;
#endif /* CONFIG_IPIPE */

	setup_irq(omap_dm_timer_get_irq(gptimer), &omap2_gp_timer_irq);
	omap_dm_timer_set_int_enable(gptimer, OMAP_TIMER_INT_OVERFLOW);

	clockevent_gpt.mult = div_sc(tick_rate, NSEC_PER_SEC,
				     clockevent_gpt.shift);
	clockevent_gpt.max_delta_ns =
		clockevent_delta2ns(0xffffffff, &clockevent_gpt);
	clockevent_gpt.min_delta_ns =
		clockevent_delta2ns(3, &clockevent_gpt);
		/* Timer internal resynch latency. */
	clockevent_gpt.cpumask = cpumask_of(0);
	clockevents_register_device(&clockevent_gpt);
}

#ifdef CONFIG_OMAP_32K_TIMER
/* 
 * When 32k-timer is enabled, don't use GPTimer for clocksource
 * instead, just leave default clocksource which uses the 32k
 * sync counter.  See clocksource setup in see plat-omap/common.c. 
 */

static inline void __init omap2_gp_clocksource_init(void) {}
#else
/*
 * clocksource
 */
static cycle_t clocksource_read_cycles(void)
{
	return (cycle_t)omap_dm_timer_read_counter(gpt_clocksource);
}

static struct clocksource clocksource_gpt = {
	.name		= "gp timer",
	.rating		= 300,
	.read		= clocksource_read_cycles,
	.mask		= CLOCKSOURCE_MASK(32),
	.shift		= 24,
	.flags		= CLOCK_SOURCE_IS_CONTINUOUS,
};

/* Setup free-running counter for clocksource */
static void __init omap2_gp_clocksource_init(void)
{
	static struct omap_dm_timer *gpt;
	u32 tick_rate, tick_period;
	static char err1[] __initdata = KERN_ERR
		"%s: failed to request dm-timer\n";
	static char err2[] __initdata = KERN_ERR
		"%s: can't register clocksource!\n";

	gpt = omap_dm_timer_request();
	if (!gpt)
		printk(err1, clocksource_gpt.name);
	gpt_clocksource = gpt;

	omap_dm_timer_set_source(gpt, OMAP_TIMER_SRC_SYS_CLK);
	tick_rate = clk_get_rate(omap_dm_timer_get_fclk(gpt));
	tick_period = (tick_rate / HZ) - 1;

	omap_dm_timer_set_load_start(gpt, 1, 0);

	clocksource_gpt.mult =
		clocksource_khz2mult(tick_rate/1000, clocksource_gpt.shift);

#ifdef CONFIG_IPIPE
#ifndef CONFIG_SMP
	tsc = (union tsc_reg *) __ipipe_tsc_area;
	barrier();
#endif /* CONFIG_SMP */

	omap2_timer_initialized = 1;
#endif /* CONFIG_IPIPE */

	if (clocksource_register(&clocksource_gpt))
		printk(err2, clocksource_gpt.name);
}
#endif

static void __init omap2_gp_timer_init(void)
{
	omap_dm_timer_init();

	omap2_gp_clockevent_init();
	omap2_gp_clocksource_init();
}

struct sys_timer omap_timer = {
	.init	= omap2_gp_timer_init,
};

#ifdef CONFIG_IPIPE
void __ipipe_mach_acktimer(void)
{
	omap_dm_timer_write_status(gptimer, OMAP_TIMER_INT_OVERFLOW);
	omap_dm_timer_read_status(gptimer);
}

static void ipipe_mach_update_tsc(void)
{
	union tsc_reg *local_tsc;
	unsigned long stamp, flags;

	if (likely(omap2_timer_initialized)) {
		local_irq_save_hw(flags);
		local_tsc = &tsc[ipipe_processor_id()];
		stamp = omap_dm_timer_read_counter(gpt_clocksource);
		if (unlikely(stamp < local_tsc->low))
			/* 32 bit counter wrapped, increment high word. */
			local_tsc->high++;
		local_tsc->low = stamp;
		local_irq_restore_hw(flags);
	}
}

notrace unsigned long long __ipipe_mach_get_tsc(void)
{
	if (likely(omap2_timer_initialized)) {
		union tsc_reg *local_tsc, result;
		unsigned long stamp;

		local_tsc = &tsc[ipipe_processor_id()];

		__asm__ ("ldmia %1, %M0\n" :
			 "=r"(result.full) : "r"(local_tsc), "m"(*local_tsc));
		barrier();
		stamp = omap_dm_timer_read_counter(gpt_clocksource);
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
	if (delay > 3)
		omap_dm_timer_set_load_start(gptimer, 0, 0xffffffff - delay);
	else
		ipipe_trigger_irq(__ipipe_mach_timerint);
}
EXPORT_SYMBOL(__ipipe_mach_set_dec);

void __ipipe_mach_release_timer(void)
{
	struct clock_event_device *ckdev = &clockevent_gpt;
	ckdev->set_mode(ckdev->mode, ckdev);
	if (ckdev->mode == CLOCK_EVT_MODE_ONESHOT)
		ckdev->set_next_event(__ipipe_mach_ticks_per_jiffy, ckdev);
}
EXPORT_SYMBOL(__ipipe_mach_release_timer);

unsigned long __ipipe_mach_get_dec(void)
{
	return 0xffffffff - omap_dm_timer_read_counter(gptimer);
}
#endif /* CONFIG_IPIPE */
