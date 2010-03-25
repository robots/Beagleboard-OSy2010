/*
 * arch/arm/mach-at91/include/mach/irqs.h
 *
 *  Copyright (C) 2004 SAN People
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#ifndef __ASM_ARCH_IRQS_H
#define __ASM_ARCH_IRQS_H

#include <linux/io.h>
#include <mach/at91_aic.h>

#define NR_AIC_IRQS 32


/*
 * Acknowledge interrupt with AIC after interrupt has been handled.
 *   (by kernel/irq.c)
 */
#define irq_finish(irq) do { at91_sys_write(AT91_AIC_EOICR, 0); } while (0)


/*
 * IRQ interrupt symbols are the AT91xxx_ID_* symbols
 * for IRQs handled directly through the AIC, or else the AT91_PIN_*
 * symbols in gpio.h for ones handled indirectly as GPIOs.
 * We make provision for 5 banks of GPIO.
 */
#define	NR_IRQS		(NR_AIC_IRQS + (5 * 32))

/* FIQ is AIC source 0. */
#define FIQ_START AT91_ID_FIQ

#if defined(CONFIG_IPIPE) && !defined(__ASSEMBLY__)
extern unsigned __ipipe_at91_gpio_banks;

#if defined(CONFIG_ARCH_AT91RM9200)
#define __ipipe_mach_irq_mux_p(irq)					\
	((unsigned) (irq - AT91RM9200_ID_PIOA) < __ipipe_at91_gpio_banks)

#elif defined(CONFIG_ARCH_AT91SAM9260) || defined(CONFIG_ARCH_AT91SAM9G20)
#define __ipipe_mach_irq_mux_p(irq)					\
	((unsigned) (irq - AT91SAM9260_ID_PIOA) < __ipipe_at91_gpio_banks)

#elif defined(CONFIG_ARCH_AT91SAM9261)
#define __ipipe_mach_irq_mux_p(irq)					\
	((unsigned) (irq - AT91SAM9261_ID_PIOA) < __ipipe_at91_gpio_banks)

#elif defined(CONFIG_ARCH_AT91SAM9263)
#define __ipipe_mach_irq_mux_p(irq)					\
	((unsigned) (irq - AT91SAM9263_ID_PIOA) < __ipipe_at91_gpio_banks)

#elif defined(CONFIG_ARCH_AT91SAM9RL)
#define __ipipe_mach_irq_mux_p(irq)					\
	((unsigned) (irq - AT91SAM9RL_ID_PIOA) < __ipipe_at91_gpio_banks)

#elif defined(CONFIG_ARCH_AT91X40)
#define __ipipe_mach_irq_mux_p(irq)					\
	((unsigned) (irq - AT91X40_ID_PIOA) < __ipipe_at91_gpio_banks)

#endif /* CONFIG_ARCH_AT91X40 */

/* #define __IPIPE_FEATURE_PIC_MUTE */

#endif /* CONFIG_IPIPE && !__ASSEMBLY__ */

#endif
