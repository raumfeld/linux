/*
 * Driver to select GPIO lines as wakeup sources from DT.
 *
 * Copyright 2013 Daniel Mack
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/pm.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>

struct gpio_wakeup_priv {
	unsigned int count;
	unsigned int irq[0];
};

static irqreturn_t gpio_wakeup_isr(int irq, void *dev_id)
{
	return IRQ_HANDLED;
}

static int gpio_wakeup_probe(struct platform_device *pdev)
{
	int ret, count, i;
	struct gpio_wakeup_priv *priv;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;

	if (!np)
		return -EINVAL;

	count = of_gpio_count(np);
	if (count == 0)
		return -EINVAL;

	priv = devm_kzalloc(dev, sizeof(*priv) +
				 sizeof(int) * count, GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	for (i = 0; i < count; i++) {
		unsigned int gpio, irq;

		priv->irq[i] = -EINVAL;

		gpio = of_get_gpio(np, i);
		if (gpio < 0) {
			dev_warn(dev, "Unable to get gpio #%d\n", i);
			continue;
		}

		irq = gpio_to_irq(gpio);
		if (irq < 0) {
			dev_warn(dev, "Can't map GPIO %d to an IRQ\n", gpio);
			continue;
		}

		ret = devm_gpio_request_one(dev, gpio, GPIOF_IN, pdev->name);
		if (ret < 0) {
			dev_warn(dev, "Unable to request GPIO %d: %d\n",
				 gpio, ret);
			continue;
		}

		ret = devm_request_irq(dev, irq, gpio_wakeup_isr,
				       IRQF_TRIGGER_RISING |
				       IRQF_TRIGGER_FALLING,
				       pdev->name, NULL);
		if (ret < 0) {
			dev_warn(dev, "Unable to request IRQ %d\n", irq);
			continue;
		}

		disable_irq(irq);
		priv->irq[i] = irq;

		dev_info(dev, "Adding GPIO %d (IRQ %d) to wakeup sources\n",
			 gpio, irq);
	}

	priv->count = count;
	device_init_wakeup(dev, 1);
	platform_set_drvdata(pdev, priv);

	return 0;
}

static int gpio_wakeup_suspend(struct device *dev)
{
	struct gpio_wakeup_priv *priv = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < priv->count; i++)
		if (priv->irq[i] >= 0) {
			enable_irq(priv->irq[i]);
			enable_irq_wake(priv->irq[i]);
		}

	return 0;
}

static int gpio_wakeup_resume(struct device *dev)
{
	struct gpio_wakeup_priv *priv = dev_get_drvdata(dev);
	int i;

	for (i = 0; i < priv->count; i++)
		if (priv->irq[i] >= 0) {
			disable_irq_wake(priv->irq[i]);
			disable_irq(priv->irq[i]);
		}

	return 0;
}

static SIMPLE_DEV_PM_OPS(gpio_wakeup_pm_ops,
			 gpio_wakeup_suspend, gpio_wakeup_resume);

static struct of_device_id gpio_wakeup_of_match[] = {
	{ .compatible = "gpio-wakeup", },
	{ },
};
MODULE_DEVICE_TABLE(of, gpio_wakeup_of_match);

static struct platform_driver gpio_wakeup_driver = {
	.probe	= gpio_wakeup_probe,
	.driver	= {
		.name	= "gpio-wakeup",
		.owner	= THIS_MODULE,
		.pm	= &gpio_wakeup_pm_ops,
		.of_match_table = of_match_ptr(gpio_wakeup_of_match),
	}
};

static int __init gpio_wakeup_init(void)
{
	return platform_driver_register(&gpio_wakeup_driver);
}

static void __exit gpio_wakeup_exit(void)
{
	platform_driver_unregister(&gpio_wakeup_driver);
}

late_initcall(gpio_wakeup_init);
module_exit(gpio_wakeup_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Daniel Mack <zonque@gmail.com>");
MODULE_DESCRIPTION("Driver to wake up systems from GPIOs");
MODULE_ALIAS("platform:gpio-wakeup");
