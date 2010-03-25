/*
 *  linux/arch/arm/mach-integrator/core.c
 *
 *  Copyright (C) 2000-2003 Deep Blue Solutions Ltd
 *  Copyright (C) 2005 Stelian Pop.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2, as
 * published by the Free Software Foundation.
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/sched.h>
#include <linux/smp.h>
#include <linux/termios.h>
#include <linux/amba/bus.h>
#include <linux/amba/serial.h>
#include <linux/io.h>

#include <asm/clkdev.h>
#include <mach/clkdev.h>
#include <mach/hardware.h>
#include <asm/irq.h>
#include <asm/hardware/arm_timer.h>
#include <mach/cm.h>
#include <asm/system.h>
#include <asm/leds.h>
#include <asm/mach/time.h>

#include "common.h"

static struct amba_pl010_data integrator_uart_data;

static struct amba_device rtc_device = {
	.dev		= {
		.init_name = "mb:15",
	},
	.res		= {
		.start	= INTEGRATOR_RTC_BASE,
		.end	= INTEGRATOR_RTC_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	.irq		= { IRQ_RTCINT, NO_IRQ },
	.periphid	= 0x00041030,
};

static struct amba_device uart0_device = {
	.dev		= {
		.init_name = "mb:16",
		.platform_data = &integrator_uart_data,
	},
	.res		= {
		.start	= INTEGRATOR_UART0_BASE,
		.end	= INTEGRATOR_UART0_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	.irq		= { IRQ_UARTINT0, NO_IRQ },
	.periphid	= 0x0041010,
};

static struct amba_device uart1_device = {
	.dev		= {
		.init_name = "mb:17",
		.platform_data = &integrator_uart_data,
	},
	.res		= {
		.start	= INTEGRATOR_UART1_BASE,
		.end	= INTEGRATOR_UART1_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	.irq		= { IRQ_UARTINT1, NO_IRQ },
	.periphid	= 0x0041010,
};

static struct amba_device kmi0_device = {
	.dev		= {
		.init_name = "mb:18",
	},
	.res		= {
		.start	= KMI0_BASE,
		.end	= KMI0_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	.irq		= { IRQ_KMIINT0, NO_IRQ },
	.periphid	= 0x00041050,
};

static struct amba_device kmi1_device = {
	.dev		= {
		.init_name = "mb:19",
	},
	.res		= {
		.start	= KMI1_BASE,
		.end	= KMI1_BASE + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	.irq		= { IRQ_KMIINT1, NO_IRQ },
	.periphid	= 0x00041050,
};

static struct amba_device *amba_devs[] __initdata = {
	&rtc_device,
	&uart0_device,
	&uart1_device,
	&kmi0_device,
	&kmi1_device,
};

/*
 * These are fixed clocks.
 */
static struct clk clk24mhz = {
	.rate	= 24000000,
};

static struct clk uartclk = {
	.rate	= 14745600,
};

static struct clk_lookup lookups[] __initdata = {
	{	/* UART0 */
		.dev_id		= "mb:16",
		.clk		= &uartclk,
	}, {	/* UART1 */
		.dev_id		= "mb:17",
		.clk		= &uartclk,
	}, {	/* KMI0 */
		.dev_id		= "mb:18",
		.clk		= &clk24mhz,
	}, {	/* KMI1 */
		.dev_id		= "mb:19",
		.clk		= &clk24mhz,
	}, {	/* MMCI - IntegratorCP */
		.dev_id		= "mb:1c",
		.clk		= &uartclk,
	}
};

static int __init integrator_init(void)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(lookups); i++)
		clkdev_add(&lookups[i]);

	for (i = 0; i < ARRAY_SIZE(amba_devs); i++) {
		struct amba_device *d = amba_devs[i];
		amba_device_register(d, &iomem_resource);
	}

	return 0;
}

arch_initcall(integrator_init);

/*
 * On the Integrator platform, the port RTS and DTR are provided by
 * bits in the following SC_CTRLS register bits:
 *        RTS  DTR
 *  UART0  7    6
 *  UART1  5    4
 */
#define SC_CTRLC	(IO_ADDRESS(INTEGRATOR_SC_BASE) + INTEGRATOR_SC_CTRLC_OFFSET)
#define SC_CTRLS	(IO_ADDRESS(INTEGRATOR_SC_BASE) + INTEGRATOR_SC_CTRLS_OFFSET)

static void integrator_uart_set_mctrl(struct amba_device *dev, void __iomem *base, unsigned int mctrl)
{
	unsigned int ctrls = 0, ctrlc = 0, rts_mask, dtr_mask;

	if (dev == &uart0_device) {
		rts_mask = 1 << 4;
		dtr_mask = 1 << 5;
	} else {
		rts_mask = 1 << 6;
		dtr_mask = 1 << 7;
	}

	if (mctrl & TIOCM_RTS)
		ctrlc |= rts_mask;
	else
		ctrls |= rts_mask;

	if (mctrl & TIOCM_DTR)
		ctrlc |= dtr_mask;
	else
		ctrls |= dtr_mask;

	__raw_writel(ctrls, SC_CTRLS);
	__raw_writel(ctrlc, SC_CTRLC);
}

static struct amba_pl010_data integrator_uart_data = {
	.set_mctrl = integrator_uart_set_mctrl,
};

#define CM_CTRL	IO_ADDRESS(INTEGRATOR_HDR_BASE) + INTEGRATOR_HDR_CTRL_OFFSET

static DEFINE_SPINLOCK(cm_lock);

/**
 * cm_control - update the CM_CTRL register.
 * @mask: bits to change
 * @set: bits to set
 */
void cm_control(u32 mask, u32 set)
{
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&cm_lock, flags);
	val = readl(CM_CTRL) & ~mask;
	writel(val | set, CM_CTRL);
	spin_unlock_irqrestore(&cm_lock, flags);
}

EXPORT_SYMBOL(cm_control);

/*
 * Where is the timer (VA)?
 */
#define TIMER0_VA_BASE (IO_ADDRESS(INTEGRATOR_CT_BASE)+0x00000000)
#define TIMER1_VA_BASE (IO_ADDRESS(INTEGRATOR_CT_BASE)+0x00000100)
#define TIMER2_VA_BASE (IO_ADDRESS(INTEGRATOR_CT_BASE)+0x00000200)
#define VA_IC_BASE     IO_ADDRESS(INTEGRATOR_IC_BASE) 

/*
 * How long is the timer interval?
 */
#define TICKS2USECS(x)	((x) / TICKS_PER_uSEC)

static unsigned long timer_reload;
static unsigned long timer_interval;
static unsigned long timer_lxlost;
static int tscok;

#ifdef CONFIG_IPIPE
int __ipipe_mach_timerint = IRQ_TIMERINT1;
static unsigned long long __ipipe_mach_tsc;
static IPIPE_DEFINE_SPINLOCK(timer_lock);

int __ipipe_mach_timerstolen = 0;
EXPORT_SYMBOL(__ipipe_mach_timerstolen);

void __ipipe_mach_get_tscinfo(struct __ipipe_tscinfo *info)
{
	info->type = IPIPE_TSC_TYPE_NONE;
}
#endif

/*
 * Called with IRQ disabled from do_gettimeofday().
 */
static inline unsigned long integrator_getticksoffset(void)
{
	unsigned long ticks;

	if (!tscok)
		return 0;

	ticks = readl(TIMER1_VA_BASE + TIMER_VALUE) & 0xffff;

	if (ticks > timer_reload)
		ticks = 0xffff + timer_reload - ticks;
	else
		ticks = timer_reload - ticks;

	if (timer_interval < 0x10000)
		return ticks;
	else if (timer_interval < 0x100000)
		return ticks * 16;
	else
		return ticks * 256;
}

/*
 * Returns number of ms since last clock interrupt.  Note that interrupts
 * will have been disabled by do_gettimeoffset()
 */
unsigned long integrator_gettimeoffset(void)
{
	/*
	 * Convert the ticks to usecs
	 */
	return TICKS2USECS(timer_lxlost + integrator_getticksoffset());
}

/*
 * IRQ handler for the timer
 */
static irqreturn_t
integrator_timer_interrupt(int irq, void *dev_id)
{
	timer_lxlost = 0;

#ifdef CONFIG_IPIPE
	/*
	 * If Linux is the only domain, ack the timer and reprogram it
	 */
	if (!__ipipe_mach_timerstolen) {
		__ipipe_mach_tsc += integrator_getticksoffset();
#else
		writel(1, TIMER1_VA_BASE + TIMER_INTCLR);
#endif

		writel(timer_reload, TIMER1_VA_BASE + TIMER_LOAD);
#ifdef CONFIG_IPIPE
	}
#endif
	timer_tick();

	return IRQ_HANDLED;
}

static struct irqaction integrator_timer_irq = {
	.name		= "Integrator Timer Tick",
	.flags		= IRQF_DISABLED | IRQF_TIMER | IRQF_IRQPOLL,
	.handler	= integrator_timer_interrupt,
};

static inline void set_dec(unsigned long reload)
{
	unsigned int ctrl = TIMER_CTRL_ENABLE | TIMER_CTRL_IE;

	timer_reload = reload;
	timer_interval = reload;

	if (timer_reload >= 0x100000) {
		timer_reload >>= 8;
		ctrl |= TIMER_CTRL_DIV256;
	} else if (timer_reload >= 0x010000) {
		timer_reload >>= 4;
		ctrl |= TIMER_CTRL_DIV16;
	}

	writel(ctrl, TIMER1_VA_BASE + TIMER_CTRL);
	writel(timer_reload, TIMER1_VA_BASE + TIMER_LOAD);
}

/*
 * Set up timer interrupt, and return the current time in seconds.
 */
void __init integrator_time_init(unsigned long reload, unsigned int ctrl)
{
	/*
	 * Initialise to a known state (all timers off)
	 */
	writel(0, TIMER0_VA_BASE + TIMER_CTRL);
	writel(0, TIMER1_VA_BASE + TIMER_CTRL);
	writel(0, TIMER2_VA_BASE + TIMER_CTRL);

	set_dec(reload);

	/*
	 * Make irqs happen for the system timer
	 */
	setup_irq(IRQ_TIMERINT1, &integrator_timer_irq);

	tscok = 1;
}

#ifdef CONFIG_IPIPE

void __ipipe_mach_acktimer(void)
{
	writel(1, TIMER1_VA_BASE + TIMER_INTCLR);
}

notrace unsigned long long __ipipe_mach_get_tsc(void)
{
	unsigned long long result;
	unsigned long flags;

	local_irq_save_hw_notrace(flags);
	spin_lock(&timer_lock);
	result = __ipipe_mach_tsc + integrator_getticksoffset();
	spin_unlock(&timer_lock);
	local_irq_restore_hw_notrace(flags);
	return result;
}
EXPORT_SYMBOL(__ipipe_mach_get_tsc);

void __ipipe_mach_set_dec(unsigned long reload)
{
	unsigned long ticks;
	unsigned long flags;

	spin_lock_irqsave(&timer_lock, flags);
	ticks = integrator_getticksoffset();
	__ipipe_mach_tsc += ticks;
	timer_lxlost += ticks;

	set_dec(reload);
	spin_unlock_irqrestore(&timer_lock, flags);
}
EXPORT_SYMBOL(__ipipe_mach_set_dec);

void __ipipe_mach_release_timer(void)
{
       __ipipe_mach_set_dec(__ipipe_mach_ticks_per_jiffy);
}
EXPORT_SYMBOL(__ipipe_mach_release_timer);

unsigned long __ipipe_mach_get_dec(void)
{
	return readl(TIMER1_VA_BASE + TIMER_VALUE);
}
#endif /* CONFIG_IPIPE */
