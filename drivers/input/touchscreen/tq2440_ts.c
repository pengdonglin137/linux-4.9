/*************************************

NAME:tq2440_ts.c
COPYRIGHT:www.embedsky.net

 *************************************/
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/init.h>
#include <linux/serio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <asm/io.h>
#include <asm/irq.h>

#include <plat/regs-adc.h>
#include <mach/regs-gpio.h>

/* For ts.dev.id.version */
#define S3C2410TSVERSION	0x0101

#define WAIT4INT(x)  (((x)<<8) | \
	S3C2410_ADCTSC_YM_SEN | S3C2410_ADCTSC_YP_SEN | S3C2410_ADCTSC_XP_SEN | \
	S3C2410_ADCTSC_XY_PST(3))

#define AUTOPST	     (S3C2410_ADCTSC_YM_SEN | S3C2410_ADCTSC_YP_SEN | S3C2410_ADCTSC_XP_SEN | \
	S3C2410_ADCTSC_AUTO_PST | S3C2410_ADCTSC_XY_PST(0))

static char *tq2440ts_name = "TQ2440 TouchScreen";

static	struct input_dev *idev;
static	long xp;
static	long yp;
static	int count;

static void __iomem *base_addr;

static void touch_timer_fire(unsigned long data)
{
	u32 data0;
	u32 data1;
	int updown;

	data0 = readl(base_addr+S3C2410_ADCDAT0);
	data1 = readl(base_addr+S3C2410_ADCDAT1);

	updown = (!(data0 & S3C2410_ADCDAT0_UPDOWN)) && (!(data1 & S3C2410_ADCDAT0_UPDOWN));

	if (updown) {
		if (count != 0) {
			long tmp;

			tmp = xp;
			xp = yp;
			yp = tmp;

			xp >>= 2;
			yp >>= 2;

			input_report_abs(idev, ABS_X, xp);
			input_report_abs(idev, ABS_Y, yp);

			input_report_key(idev, BTN_TOUCH, 1);
			input_report_abs(idev, ABS_PRESSURE, 1);
			input_sync(idev);
		}

		xp = 0;
		yp = 0;
		count = 0;

		writel(S3C2410_ADCTSC_PULL_UP_DISABLE | AUTOPST, base_addr+S3C2410_ADCTSC);
		writel(readl(base_addr+S3C2410_ADCCON) | S3C2410_ADCCON_ENABLE_START, base_addr+S3C2410_ADCCON);
	} else {
		count = 0;

		input_report_key(idev, BTN_TOUCH, 0);
		input_report_abs(idev, ABS_PRESSURE, 0);
		input_sync(idev);

		writel(WAIT4INT(0), base_addr+S3C2410_ADCTSC);
	}
}

static struct timer_list touch_timer =
TIMER_INITIALIZER(touch_timer_fire, 0, 0);

static irqreturn_t stylus_updown(int irq, void *dev_id)
{
	u32 data0;
	u32 data1;
	int updown;

	data0 = readl(base_addr+S3C2410_ADCDAT0);
	data1 = readl(base_addr+S3C2410_ADCDAT1);

	updown = (!(data0 & S3C2410_ADCDAT0_UPDOWN)) && (!(data1 & S3C2410_ADCDAT0_UPDOWN));

	if (updown)
		touch_timer_fire(0);

	return IRQ_HANDLED;
}

static irqreturn_t stylus_action(int irq, void *dev_id)
{
	u32 data0;
	u32 data1;

	data0 = readl(base_addr+S3C2410_ADCDAT0);
	data1 = readl(base_addr+S3C2410_ADCDAT1);

	xp += data0 & S3C2410_ADCDAT0_XPDATA_MASK;
	yp += data1 & S3C2410_ADCDAT1_YPDATA_MASK;
	count++;

	if (count < (1<<2)) {
		writel(S3C2410_ADCTSC_PULL_UP_DISABLE | AUTOPST, base_addr+S3C2410_ADCTSC);
		writel(readl(base_addr+S3C2410_ADCCON) | S3C2410_ADCCON_ENABLE_START, base_addr+S3C2410_ADCCON);
	} else {
		mod_timer(&touch_timer, jiffies+1);
		writel(WAIT4INT(1), base_addr+S3C2410_ADCTSC);
	}

	return IRQ_HANDLED;
}

static int tq2440ts_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct device_node *node = dev->of_node;
	struct clk	*adc_clock;
	struct resource *tsmem, *irq;
	struct input_dev *input_dev;
	int ret;

	if (!node) {
		dev_dbg(dev, "of_node is NULL\n");
		return -EINVAL;
	}

	adc_clock = devm_clk_get(dev, "adc");
	dev_dbg(dev, "adc_clock: %p\n", adc_clock);
	if (IS_ERR(adc_clock)) {
		dev_err(dev, "cannot get clock\n");
		return -ENOENT;
	}
	clk_prepare(adc_clock);
	clk_enable(adc_clock);

	dev_dbg(dev, "get mem\n");
	tsmem = platform_get_resource_byname(pdev, IORESOURCE_MEM, "adc_ts_physical");
	if (!tsmem) {
		dev_dbg(dev, "get mem resource failed.\n");
		ret = -EINVAL;
		goto err;
	}

	base_addr = devm_ioremap_resource(dev, tsmem);
	if (IS_ERR(base_addr)) {
		dev_dbg(dev, "ioremap failed.\n");
		ret = PTR_ERR(base_addr);
		goto err;
	}

	writel(S3C2410_ADCCON_PRSCEN | S3C2410_ADCCON_PRSCVL(0xFF),base_addr+S3C2410_ADCCON);
	writel(0xffff,  base_addr+S3C2410_ADCDLY);
	writel(WAIT4INT(0), base_addr+S3C2410_ADCTSC);

	input_dev = devm_input_allocate_device(dev);
	if (!input_dev) {
		dev_dbg(dev, "ioremap failed.\n");
		ret = -ENOMEM;
		goto err;
	}

	idev = input_dev;
	idev->evbit[0] = BIT(EV_SYN) | BIT(EV_KEY) | BIT(EV_ABS);


	__set_bit(EV_SYN, idev->evbit);
	__set_bit(EV_KEY, idev->evbit);
	__set_bit(EV_ABS, idev->evbit);
	__set_bit(BTN_TOUCH, idev->keybit);

	input_set_abs_params(idev, ABS_X, 0, 0x3FF, 0, 0);
	input_set_abs_params(idev, ABS_Y, 0, 0x3FF, 0, 0);
	input_set_abs_params(idev, ABS_PRESSURE, 0, 1, 0, 0);

	idev->name = tq2440ts_name;
	idev->id.bustype = BUS_RS232;
	idev->id.vendor = 0xDEAD;
	idev->id.product = 0xBEEF;
	idev->id.version = S3C2410TSVERSION;

	irq = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "int_ts");
	if (!irq) {
		dev_err(dev, "get irq resource int_ts failed.\n");
		ret = -EINVAL;
		goto err;
	}
	ret = devm_request_irq(dev, irq->start, stylus_updown, IRQF_ONESHOT, "int_ts", NULL);
	if (ret < 0){
		dev_err(dev, "request irq tsirq %d failed.\n", irq->start);
		goto err;
	}

	irq = platform_get_resource_byname(pdev, IORESOURCE_IRQ, "int_adc_s");
	if (!irq) {
		dev_err(dev, "get irq resource int_adc_s failed.\n");
		ret = -EINVAL;
		goto err;
	}
	ret = devm_request_irq(dev, irq->start, stylus_action, IRQF_ONESHOT, "int_adc_s", NULL);
	if (ret < 0) {
		dev_err(dev, "request irq adcirq %d failed.\n", irq->start);
		goto err;
	}

	dev_info(dev, "%s successfully loaded\n", tq2440ts_name);
	input_register_device(idev);

	return 0;
err:
	clk_disable(adc_clock);
	return ret;
}

static const struct of_device_id tq2440ts_match[] = {
	{ .compatible = "tq2440,ts", .data = (void *)0 },
	{},
};

static struct platform_driver tq2440ts_driver = {
	.probe		= tq2440ts_probe,
	.driver		= {
		.name	= "tq2440ts",
		.of_match_table = of_match_ptr(tq2440ts_match),
	},
};

static int __init tq2440ts_init(void)
{
	return platform_driver_register(&tq2440ts_driver);
}

static void __exit tq2440ts_exit(void)
{
	platform_driver_unregister(&tq2440ts_driver);
}

module_init(tq2440ts_init);
module_exit(tq2440ts_exit);

MODULE_LICENSE("GPL");
