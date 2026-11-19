// SPDX-License-Identifier: GPL-2.0-only
/*
 * i.MX28 PSWITCH power button input driver
 *
 * On the SHARP Brain (PW-SH6) the power button is not part of the
 * keyboard matrix; it drives the i.MX28 PSWITCH pin. The pin level
 * is exposed as a two-bit comparator readout in HW_POWER_STS
 * (bits 21:20, i.MX28 RM section 11.12.13).
 *
 * The idle level is captured at probe time and any deviation from
 * it is reported as a KEY_POWER press, so the driver does not
 * depend on the board's PSWITCH polarity.
 */

#include <linux/input.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>

#define HW_POWER_STS		0x0c0
#define POWER_STS_PSWITCH_MASK	(0x3 << 20)
#define POWER_STS_PSWITCH_SHIFT	20

struct mxs_pswitch {
	struct input_dev *input;
	struct regmap *regmap;
	unsigned int idle;
	bool pressed;
};

static void mxs_pswitch_poll(struct input_dev *input)
{
	struct mxs_pswitch *sw = input_get_drvdata(input);
	unsigned int sts, level;
	int err;

	err = regmap_read(sw->regmap, HW_POWER_STS, &sts);
	if (err)
		return;

	level = (sts & POWER_STS_PSWITCH_MASK) >> POWER_STS_PSWITCH_SHIFT;
	if (!sw->pressed && level != sw->idle) {
		sw->pressed = true;
		input_report_key(input, KEY_POWER, 1);
		input_sync(input);
	} else if (sw->pressed && level == sw->idle) {
		sw->pressed = false;
		input_report_key(input, KEY_POWER, 0);
		input_sync(input);
	}
}

static int mxs_pswitch_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mxs_pswitch *sw;
	unsigned int sts;
	int err;

	sw = devm_kzalloc(dev, sizeof(*sw), GFP_KERNEL);
	if (!sw)
		return -ENOMEM;

	sw->regmap = dev_get_regmap(dev->parent, NULL);
	if (!sw->regmap)
		return dev_err_probe(dev, -EINVAL, "no parent regmap\n");

	err = regmap_read(sw->regmap, HW_POWER_STS, &sts);
	if (err)
		return dev_err_probe(dev, err, "failed to read HW_POWER_STS\n");
	sw->idle = (sts & POWER_STS_PSWITCH_MASK) >> POWER_STS_PSWITCH_SHIFT;

	sw->input = devm_input_allocate_device(dev);
	if (!sw->input)
		return -ENOMEM;

	sw->input->name = "mxs-pswitch";
	sw->input->id.bustype = BUS_HOST;
	input_set_drvdata(sw->input, sw);
	input_set_capability(sw->input, EV_KEY, KEY_POWER);

	err = input_setup_polling(sw->input, mxs_pswitch_poll);
	if (err)
		return dev_err_probe(dev, err, "failed to setup polling\n");
	input_set_poll_interval(sw->input, 100);

	err = input_register_device(sw->input);
	if (err)
		return dev_err_probe(dev, err, "failed to register input device\n");

	platform_set_drvdata(pdev, sw);
	dev_info(dev, "PSWITCH idle level %u\n", sw->idle);
	return 0;
}

static const struct of_device_id mxs_pswitch_of_match[] = {
	{ .compatible = "fsl,imx28-pswitch" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mxs_pswitch_of_match);

static struct platform_driver mxs_pswitch_driver = {
	.driver = {
		.name = "mxs-pswitch",
		.of_match_table = of_match_ptr(mxs_pswitch_of_match),
	},
	.probe = mxs_pswitch_probe,
};
module_platform_driver(mxs_pswitch_driver);

MODULE_DESCRIPTION("i.MX28 PSWITCH power button input driver");
MODULE_LICENSE("GPL v2");
