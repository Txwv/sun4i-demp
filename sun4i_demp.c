// SPDX-License-Identifier: GPL-2.0+

/*
 * Copyright (c) 2020 Luc Verhaegen <libv@skynet.be>
 */

#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>

#define MODULE_NAME	"sun4i-demp"

struct demp {
	struct device *dev;
};

static int demp_probe(struct platform_device *platform_dev)
{
	struct device *dev = &platform_dev->dev;
	struct demp *demp;

	dev_info(dev, "%s();\n", __func__);

	demp = devm_kzalloc(dev, sizeof(struct demp), GFP_KERNEL);
	if (!demp)
		return -ENOMEM;
	demp->dev = dev;

	platform_set_drvdata(platform_dev, demp);

	return 0;
}

static int demp_remove(struct platform_device *platform_dev)
{
	struct device *dev = &platform_dev->dev;

	dev_info(dev, "%s();\n", __func__);

	return 0;
}

/* We are currently only testing on sun7i, but should work for sun4i as well */
static const struct of_device_id demp_of_match[] = {
	{ .compatible = "allwinner,sun4i-a10-demp", },
	{ .compatible = "allwinner,sun7i-a20-demp", },
	{},
};
MODULE_DEVICE_TABLE(of, demp_of_match);

static struct platform_driver demp_platform_driver = {
	.probe = demp_probe,
	.remove = demp_remove,
	.driver = {
		.name = MODULE_NAME,
		.of_match_table = of_match_ptr(demp_of_match),
	},
};

module_platform_driver(demp_platform_driver);
MODULE_DESCRIPTION("Allwinner A10/A20 Mixer Processor V4L2 driver");
MODULE_AUTHOR("Luc Verhaegen <libv@skynet.be>");
MODULE_LICENSE("GPL v2");
