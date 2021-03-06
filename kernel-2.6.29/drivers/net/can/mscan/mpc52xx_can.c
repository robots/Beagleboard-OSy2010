/*
 * CAN bus driver for the Freescale MPC52xx embedded CPU.
 *
 * Copyright (C) 2004-2005 Andrey Volkov <avolkov@varma-el.com>,
 *                         Varma Electronics Oy
 * Copyright (C) 2008-2009 Wolfgang Grandegger <wg@grandegger.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the version 2 of the GNU General Public License
 * as published by the Free Software Foundation
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/netdevice.h>
#include <linux/can.h>
#include <linux/can/dev.h>
#include <linux/of_platform.h>
#include <sysdev/fsl_soc.h>
#include <linux/io.h>
#include <asm/mpc52xx.h>

#include "mscan.h"


#define DRV_NAME "mpc52xx_can"

static struct of_device_id mpc52xx_cdm_ids[] __devinitdata = {
	{ .compatible = "fsl,mpc5200-cdm", },
	{ .compatible = "fsl,mpc5200b-cdm", },
	{}
};

/*
 * Get the frequency of the external oscillator clock connected
 * to the SYS_XTAL_IN pin, or retrun 0 if it cannot be determined.
 */
static unsigned int  __devinit mpc52xx_can_xtal_freq(struct device_node *np)
{
	struct mpc52xx_cdm  __iomem *cdm;
	struct device_node *np_cdm;
	unsigned int freq;
	u32 val;

	freq = mpc52xx_find_ipb_freq(np);
	if (!freq)
		return 0;

	/*
	 * Detemine SYS_XTAL_IN frequency from the clock domain settings
	 */
	np_cdm = of_find_matching_node(NULL, mpc52xx_cdm_ids);
	cdm = of_iomap(np_cdm, 0);
	of_node_put(np_cdm);
	if (!np_cdm) {
		printk(KERN_ERR "%s() failed abnormally\n", __func__);
		return 0;
	}

	if (in_8(&cdm->ipb_clk_sel) & 0x1)
		freq *= 2;
	val  = in_be32(&cdm->rstcfg);
	if (val & (1 << 5))
		freq *= 8;
	else
		freq *= 4;
	if (val & (1 << 6))
		freq /= 12;
	else
		freq /= 16;

	iounmap(cdm);

	return freq;
}

/*
 * Get frequency of the MSCAN clock source
 *
 * Either the oscillator clock (SYS_XTAL_IN) or the IP bus clock (IP_CLK)
 * can be selected. According to the MPC5200 user's manual, the oscillator
 * clock is the better choice as it has less jitter but due to a hardware
 * bug, it can not be selected for the old MPC5200 Rev. A chips.
 */

static unsigned int  __devinit mpc52xx_can_clock_freq(struct device_node *np,
						      int clock_src)
{
	unsigned int pvr;

	pvr = mfspr(SPRN_PVR);

	if (clock_src == MSCAN_CLKSRC_BUS || pvr == 0x80822011)
		return mpc52xx_find_ipb_freq(np);

	return mpc52xx_can_xtal_freq(np);
}

static int __devinit mpc52xx_can_probe(struct of_device *ofdev,
				       const struct of_device_id *id)
{
	struct device_node *np = ofdev->node;
	struct net_device *dev;
	struct can_priv *priv;
	struct resource res;
	void __iomem *base;
	int err, irq, res_size, clock_src;

	err = of_address_to_resource(np, 0, &res);
	if (err) {
		dev_err(&ofdev->dev, "invalid address\n");
		return err;
	}

	res_size = res.end - res.start + 1;

	if (!request_mem_region(res.start, res_size, DRV_NAME)) {
		dev_err(&ofdev->dev, "couldn't request %#x..%#x\n",
			res.start, res.end);
		return -EBUSY;
	}

	base = ioremap_nocache(res.start, res_size);
	if (!base) {
		dev_err(&ofdev->dev, "couldn't ioremap %#x..%#x\n",
			res.start, res.end);
		err = -ENOMEM;
		goto exit_release_mem;
	}

	irq = irq_of_parse_and_map(np, 0);
	if (irq == NO_IRQ) {
		dev_err(&ofdev->dev, "no irq found\n");
		err = -ENODEV;
		goto exit_unmap_mem;
	}

	dev = alloc_mscandev();
	if (!dev) {
		err = -ENOMEM;
		goto exit_dispose_irq;
	}

	dev->base_addr = (unsigned long)base;
	dev->irq = irq;

	priv = netdev_priv(dev);

	/*
	 * Either the oscillator clock (SYS_XTAL_IN) or the IP bus clock
	 * (IP_CLK) can be selected as MSCAN clock source. According to
	 * the MPC5200 user's manual, the oscillator clock is the better
	 * choice as it has less jitter. For this reason, it is selected
	 * by default.
	 */
	if (of_get_property(np, "clock-ipb", NULL))
		clock_src = MSCAN_CLKSRC_BUS;
	else
		clock_src = MSCAN_CLKSRC_XTAL;
	priv->bittiming.clock = mpc52xx_can_clock_freq(np, clock_src);
	if (!priv->bittiming.clock) {
		dev_err(&ofdev->dev, "couldn't get MSCAN clock frequency\n");
		err = -ENODEV;
		goto exit_free_mscan;
	}

	SET_NETDEV_DEV(dev, &ofdev->dev);

	err = register_mscandev(dev, clock_src);
	if (err) {
		dev_err(&ofdev->dev, "registering %s failed (err=%d)\n",
			DRV_NAME, err);
		goto exit_free_mscan;
	}

	dev_set_drvdata(&ofdev->dev, dev);

	dev_info(&ofdev->dev, "MSCAN at 0x%lx, irq %d, clock %dHZ\n",
		 dev->base_addr, dev->irq, priv->bittiming.clock);

	return 0;

exit_free_mscan:
	free_candev(dev);
exit_dispose_irq:
	irq_dispose_mapping(irq);
exit_unmap_mem:
	iounmap(base);
exit_release_mem:
	release_mem_region(res.start, res_size);

	return err;
}

static int __devexit mpc52xx_can_remove(struct of_device *ofdev)
{
	struct net_device *dev = dev_get_drvdata(&ofdev->dev);
	struct device_node *np = ofdev->node;
	struct resource res;

	dev_set_drvdata(&ofdev->dev, NULL);

	unregister_mscandev(dev);
	iounmap((void __iomem *)dev->base_addr);
	irq_dispose_mapping(dev->irq);
	free_candev(dev);

	of_address_to_resource(np, 0, &res);
	release_mem_region(res.start, res.end - res.start + 1);

	return 0;
}

static struct mscan_regs saved_regs;
static int mpc52xx_can_suspend(struct of_device *ofdev, pm_message_t state)
{
	struct net_device *dev = dev_get_drvdata(&ofdev->dev);
	struct mscan_regs *regs = (struct mscan_regs *)dev->base_addr;

	_memcpy_fromio(&saved_regs, regs, sizeof(*regs));

	return 0;
}

static int mpc52xx_can_resume(struct of_device *ofdev)
{
	struct net_device *dev = dev_get_drvdata(&ofdev->dev);
	struct mscan_regs *regs = (struct mscan_regs *)dev->base_addr;

	regs->canctl0 |= MSCAN_INITRQ;
	while ((regs->canctl1 & MSCAN_INITAK) == 0)
		udelay(10);

	regs->canctl1 = saved_regs.canctl1;
	regs->canbtr0 = saved_regs.canbtr0;
	regs->canbtr1 = saved_regs.canbtr1;
	regs->canidac = saved_regs.canidac;

	/* restore masks, buffers etc. */
	_memcpy_toio(&regs->canidar1_0, (void *)&saved_regs.canidar1_0,
		     sizeof(*regs) - offsetof(struct mscan_regs, canidar1_0));

	regs->canctl0 &= ~MSCAN_INITRQ;
	regs->cantbsel = saved_regs.cantbsel;
	regs->canrier = saved_regs.canrier;
	regs->cantier = saved_regs.cantier;
	regs->canctl0 = saved_regs.canctl0;

	return 0;
}

static struct of_device_id __devinitdata mpc52xx_can_table[] = {
	{.compatible = "fsl,mpc5200-mscan"},
	{.compatible = "fsl,mpc5200b-mscan"},
	{},
};

static struct of_platform_driver mpc52xx_can_driver = {
	.owner = THIS_MODULE,
	.name = "mpc52xx_can",
	.probe = mpc52xx_can_probe,
	.remove = __devexit_p(mpc52xx_can_remove),
	.suspend = mpc52xx_can_suspend,
	.resume = mpc52xx_can_resume,
	.match_table = mpc52xx_can_table,
};

static int __init mpc52xx_can_init(void)
{
	return of_register_platform_driver(&mpc52xx_can_driver);
}
module_init(mpc52xx_can_init);

static void __exit mpc52xx_can_exit(void)
{
	return of_unregister_platform_driver(&mpc52xx_can_driver);
};
module_exit(mpc52xx_can_exit);

MODULE_AUTHOR("Wolfgang Grandegger <wg@grandegger.com>");
MODULE_DESCRIPTION("Freescale MPC5200 CAN driver");
MODULE_LICENSE("GPL v2");
