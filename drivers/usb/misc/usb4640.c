/*
 * Driver for SMSC USB4640 USB 2.0 hub controller driver
 *
 */

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/platform_data/usb4640.h>

struct usb4640 {
	enum usb4640_mode	mode;
	struct device		*dev;
	int	gpio_reset;
};

static int usb4640_reset(struct usb4640 *hub, int state)
{
	if (gpio_is_valid(hub->gpio_reset))
		gpio_set_value_cansleep(hub->gpio_reset, state);

	/* Wait 1ms for hub logic to stabilize */
	if (state)
		usleep_range(1, 10);

	return 0;
}

static int usb4640_connect(struct usb4640 *hub)
{
	struct device *dev = hub->dev;

	usb4640_reset(hub, 1);
	hub->mode = USB4640_MODE_HUB;
	dev_info(dev, "switched to HUB mode\n");

	return 0;
}

static int usb4640_switch_mode(struct usb4640 *hub, enum usb4640_mode mode)
{
	struct device *dev = hub->dev;
	int err = 0;

	switch (mode) {
	case USB4640_MODE_HUB:
		err = usb4640_connect(hub);
		break;

	case USB4640_MODE_STANDBY:
		usb4640_reset(hub, 0);
		dev_info(dev, "switched to STANDBY mode\n");
		break;

	default:
		dev_err(dev, "unknown mode is requested\n");
		err = -EINVAL;
		break;
	}

	return err;
}


static int usb4640_probe(struct usb4640 *hub)
{
	struct device *dev = hub->dev;
	struct usb4640_platform_data *pdata = dev_get_platdata(dev);
	struct device_node *np = dev->of_node;
	int err;
	u32 mode = USB4640_MODE_HUB;

	if (pdata) {
		hub->gpio_reset		= pdata->gpio_reset;
		hub->mode		= pdata->initial_mode;
	} else if (np) {
		hub->gpio_reset = of_get_named_gpio(np, "reset-gpios", 0);
		if (hub->gpio_reset == -EPROBE_DEFER)
			return -EPROBE_DEFER;
		of_property_read_u32(np, "initial-mode", &mode);
		hub->mode = mode;
	}

	if (gpio_is_valid(hub->gpio_reset)) {
		err = devm_gpio_request_one(dev, hub->gpio_reset,
				GPIOF_OUT_INIT_LOW, "usb4640 reset");
		if (err) {
			dev_err(dev,
				"unable to request GPIO %d as reset pin (%d)\n",
				hub->gpio_reset, err);
			return err;
		}
	} else
		dev_err(dev, "reset gpio is invalid: %d\n", hub->gpio_reset);

	usb4640_switch_mode(hub, hub->mode);

	dev_info(dev, "%s: probed in %s mode\n", __func__,
			(hub->mode == USB4640_MODE_HUB) ? "hub" : "standby");

	return 0;
}

static int usb4640_platform_probe(struct platform_device *pdev)
{
	struct usb4640 *hub;

	hub = devm_kzalloc(&pdev->dev, sizeof(struct usb4640), GFP_KERNEL);
	if (!hub)
		return -ENOMEM;
	hub->dev = &pdev->dev;

	return usb4640_probe(hub);
}

#ifdef CONFIG_OF
static const struct of_device_id usb4640_of_match[] = {
	{ .compatible = "smsc,usb4640", },
	{},
};
MODULE_DEVICE_TABLE(of, usb4640_of_match);
#endif

static struct platform_driver usb4640_platform_driver = {
	.driver = {
		.name = USB4640_NAME,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(usb4640_of_match),
#endif
	},
	.probe		= usb4640_platform_probe,
};

static int __init usb4640_init(void)
{
	int err;

	err = platform_driver_register(&usb4640_platform_driver);
	if (err != 0)
		pr_err("usb4640: Failed to register platform driver: %d\n",
		       err);

	return 0;
}
module_init(usb4640_init);

static void __exit usb4640_exit(void)
{
	platform_driver_unregister(&usb4640_platform_driver);
}
module_exit(usb4640_exit);

MODULE_DESCRIPTION("USB4640 USB HUB driver");
MODULE_LICENSE("GPL");
