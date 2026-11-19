// SPDX-License-Identifier: GPL-2.0-only
/*
 * SHARP Brain keyboard (I2C) driver
 *
 * Copyright 2021 Takumi Sueda
 *
 * Author: Takumi Sueda <puhitaku@gmail.com>
 *
 */

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>

#define DEV_NAME "brain-kbd-gpio"

#define BK_CMD_KEYCODE 0x04

#define BK_IS_PRESSED(val) ((~val & 0x40) >> 6)

struct bk_gpio_data {
	struct input_dev *inputdev;
	struct gpio_desc* in[8];
	struct gpio_desc* out[7];
	unsigned int km[7][7];
	unsigned int km_symbol[7][7];
	int kmlen;
	int kmlen_symbol;
	unsigned int sym_key_bank;
	unsigned int sym_key_num;

	bool symbol;
	ulong last_in[7];
	bool pressed[7][7];
	/* keycode actually reported for each cell, UINT_MAX if none */
	unsigned int reported[7][7];
};

static void bk_gpio_read_keys(struct input_dev *inputdev, ulong* result)
{
	struct bk_gpio_data *kbd = input_get_drvdata(inputdev);
	struct device *dev = &inputdev->dev;
	int try, i, err;
	ulong in[10][7];

	for (try = 0; try < ARRAY_SIZE(in); try++) {
		for (i = 0; i < ARRAY_SIZE(in[0]); i++) {
			gpiod_set_value(kbd->out[i], 1);
			udelay(100);
			in[try][i] = 0;
			err = gpiod_get_array_value(8, kbd->in, NULL, &in[try][i]);
			if (err) {
				dev_err(dev, "failed to get array value: %d\n", err);
			}
			in[try][i] = ~(((in[try][i] ^ (in[try][i] >> 1)) & 0x1f) ^ (in[try][i] >> 1)) & 0x7f;
			gpiod_set_value(kbd->out[i], 0);
		}

		if (try < 3) {
			continue;
		}

		if (
			memcmp(in[try-2], in[try-1], sizeof(in[0])) == 0 &&
			memcmp(in[try-1], in[try], sizeof(in[0])) == 0
		) {
			for (i = 0; i < ARRAY_SIZE(in[0]); i++) {
				result[i] = in[try][i];
			}
			break;
		}

		// not to exceed 10ms in total
		mdelay(1);
	}

	if (try == ARRAY_SIZE(in)) {
		dev_dbg(dev, "failed to debounce: exceeded %d tries\n", ARRAY_SIZE(in));
		for (i = 0; i < ARRAY_SIZE(in[0]); i++) {
			result[i] = 0;
		}
	}
}

/*
 * The matrix has no diodes, so pressing three corners of a rectangle
 * makes the fourth corner read as pressed (ghosting). A new press that
 * completes a rectangle in the raw scan is therefore ambiguous: it may
 * be a real key or a ghost. Suppress it until the combination changes.
 */
static bool bk_is_ghost(bool cur[7][7], int row, int col)
{
	int i, j;

	for (i = 0; i < 7; i++) {
		if (i == row)
			continue;
		for (j = 0; j < 7; j++) {
			if (j == col)
				continue;
			if (cur[row][j] && cur[i][col] && cur[i][j])
				return true;
		}
	}
	return false;
}

static void bk_gpio_poll(struct input_dev *inputdev)
{
	struct bk_gpio_data *kbd = input_get_drvdata(inputdev);
	struct device *dev = &inputdev->dev;
	int i, j;
	ulong in[7], agg;
	bool cur[7][7], real[7][7];

	bk_gpio_read_keys(inputdev, in);

	if (memcmp(in, kbd->last_in, sizeof(in)) == 0) {
		goto skip;
	}

	dev_dbg(
		dev,
		"%02lx %02lx %02lx %02lx %02lx %02lx %02lx",
		in[0], in[1], in[2], in[3], in[4], in[5], in[6]
	);

	agg = 0;
	for (i = 0; i < 7; i++)
		agg |= in[i];

	for (i = 0; i < 7; i++)
		for (j = 0; j < 7; j++) {
			cur[i][j] = ((in[j] >> i) & 1) == 0;
			/*
			 * The scan drives one column high while every other
			 * column stays low, and a row is pulled low only by
			 * a contact in a low column -- so a row bit in in[j]
			 * is set only by a contact in a column other than
			 * j, and cur on its own reads pressed at idle
			 * (in[] == 0).  A cell is really pressed only when
			 * its bit is clear in its own column's read and its
			 * row is seen pulled low in some other column's read
			 * (the agg test the pre-ghost-fix driver used).
			 * Without it releases never fire: after the first
			 * key, pressed[] latches for the whole phantom set
			 * and every later key is swallowed.
			 */
			real[i][j] = cur[i][j] && ((agg >> i) & 1);
		}

	/* Releases first, so state is settled before new presses are
	 * evaluated (and before the ghost check looks at the matrix). */
	for (i = 0; i < 7; i++) {
		for (j = 0; j < 7; j++) {
			if (!kbd->pressed[i][j] || real[i][j])
				continue;

			kbd->pressed[i][j] = false;

			if (i == kbd->sym_key_bank && j == kbd->sym_key_num) {
				kbd->symbol = false;
			} else if (kbd->reported[i][j] != UINT_MAX) {
				dev_dbg(dev, "R: %04x\n", kbd->reported[i][j]);
				input_report_key(inputdev, kbd->reported[i][j], 0);
				kbd->reported[i][j] = UINT_MAX;
			}
		}
	}

	for (i = 0; i < 7; i++) {
		for (j = 0; j < 7; j++) {
			unsigned int code;

			if (kbd->pressed[i][j] || !real[i][j])
				continue;

			if (i == kbd->sym_key_bank && j == kbd->sym_key_num) {
				dev_dbg(dev, "symbol\n");
				kbd->pressed[i][j] = true;
				kbd->symbol = true;
				continue;
			}

			if (bk_is_ghost(cur, i, j)) {
				dev_dbg(dev, "ghost suppressed at %d %d\n", i, j);
				continue;
			}

			code = kbd->symbol ? kbd->km_symbol[i][j] : kbd->km[i][j];
			if (code == UINT_MAX)
				continue;

			dev_dbg(dev, "P: %04x\n", code);
			kbd->pressed[i][j] = true;
			kbd->reported[i][j] = code;
			input_report_key(inputdev, code, 1);
		}
	}

	input_sync(inputdev);
skip:
	memcpy(kbd->last_in, in, sizeof(in));
}

static int bk_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct bk_gpio_data *kbd;
	int i, j, count, err, cells, len, offset;
	struct gpio_desc *gpio;
	u32 keydef[3];

	kbd = devm_kzalloc(dev, sizeof(*kbd), GFP_KERNEL);
	if (!kbd) {
		return -ENOMEM;
	}

	count = gpiod_count(dev, "matrix-in");
	if (count != 8) {
		dev_err(dev, "expected length of matrix-in-gpios is 8, not %d\n", count);
		return -EINVAL;
	}

	for (i = 0; i < gpiod_count(dev, "matrix-in"); i++) {
		gpio = devm_gpiod_get_index(dev, "matrix-in", i, GPIOD_IN);
		if (IS_ERR(gpio)) {
			dev_err(dev, "failed to get input gpio %d\n", i);
			return -EINVAL;
		}
		kbd->in[i] = gpio;
	}

	count = gpiod_count(dev, "matrix-out");
	if (count != 7) {
		dev_err(dev, "expected length of matrix-out-gpios is 7, not %d\n", count);
		return -EINVAL;
	}

	for (i = 0; i < gpiod_count(dev, "matrix-out"); i++) {
		gpio = devm_gpiod_get_index(dev, "matrix-out", i, GPIOD_OUT_LOW);
		if (IS_ERR(gpio)) {
			dev_err(dev, "failed to get output gpio %d\n", i);
			return -EINVAL;
		}
		gpiod_set_value(gpio, 0);
		kbd->out[i] = gpio;
	}

	// Init input device

	kbd->inputdev = devm_input_allocate_device(dev);
	if (!kbd->inputdev) {
		dev_err(dev, "failed to allocate inpute device\n");
		return -ENOMEM;
	}

	input_set_drvdata(kbd->inputdev, kbd);

	kbd->inputdev->name = DEV_NAME;
	kbd->inputdev->id.bustype = BUS_HOST;

	__set_bit(EV_KEY, kbd->inputdev->evbit); /* FIXME: is it really necessary? */
	__set_bit(EV_REP, kbd->inputdev->evbit); /* autorepeat */

	// Parse keymap

	cells = 3;
	if (!of_get_property(dev->of_node, "keymap", &len)) {
		dev_err(dev, "DT node has no keymap\n");
		return -EINVAL;
	}

	len /= sizeof(u32) * cells;
	kbd->kmlen = len;

	for (i = 0; i < 7; i++) {
		for (j = 0; j < 7; j++) {
			// sentinel value to indicate unused mapping
			kbd->km[i][j] = UINT_MAX;
		}
	}

	for (i = 0; i < len; i++) {
		offset = i * cells;
		for (j = 0; j < 3; j++) {
			if (of_property_read_u32_index(dev->of_node, "keymap",
						       offset + j, keydef + j)) {
				dev_err(dev,
					"could not read DT property (brain keycode)\n");
				return -EINVAL;
			}
		}
		kbd->km[keydef[0] & 0xff][keydef[1] & 0xff] = keydef[2];
		dev_dbg(dev, "normal: brain: %x %x, kernel: %x",
			keydef[0], keydef[1], keydef[2]);

		input_set_capability(kbd->inputdev, EV_KEY, keydef[2]);
	}

	if (!of_get_property(dev->of_node, "keymap-symbol", &len)) {
		dev_err(dev, "DT node has no keymap (symbol)\n");
		return -EINVAL;
	}

	len /= sizeof(u32) * cells;
	kbd->kmlen_symbol = len;

	for (i = 0; i < 7; i++) {
		for (j = 0; j < 7; j++) {
			// sentinel value to indicate unused mapping
			kbd->km_symbol[i][j] = UINT_MAX;
		}
	}

	for (i = 0; i < len; i++) {
		offset = i * cells;
		for (j = 0; j < 3; j++) {
			if (of_property_read_u32_index(dev->of_node, "keymap-symbol",
						       offset + j, keydef + j)) {
				dev_err(dev,
					"could not read DT property (brain keycode)\n");
				return -EINVAL;
			}
		}
		kbd->km_symbol[keydef[0] & 0xff][keydef[1] & 0xff] = keydef[2];
		dev_dbg(dev, "symbol: brain: %02x %02x, kernel: %02x",
			keydef[0], keydef[1], keydef[2]);

		input_set_capability(kbd->inputdev, EV_KEY, keydef[2]);
	}

	if (of_property_read_u32_index(dev->of_node, "symbol-key", 0, &kbd->sym_key_bank)) {
		dev_err(dev, "could not read symbol key bank\n");
		return -EINVAL;
	}
	dev_dbg(dev, "sym_key_bank = %d\n", kbd->sym_key_bank);

	if (of_property_read_u32_index(dev->of_node, "symbol-key", 1, &kbd->sym_key_num)) {
		dev_err(dev, "could not read symbol key num\n");
		return -EINVAL;
	}
	dev_dbg(dev, "sym_key_num = %d\n", kbd->sym_key_num);

	for (i = 0; i < 7; i++) {
		for (j = 0; j < 7; j++) {
			kbd->pressed[i][j] = false;
			kbd->reported[i][j] = UINT_MAX;
		}
	}

	err = input_setup_polling(kbd->inputdev, bk_gpio_poll);
	if (err) {
		dev_err(dev, "failed to setupp poling func: %d\n",
			err);
		return err;
	}

	input_set_poll_interval(kbd->inputdev, 100);

	err = input_register_device(kbd->inputdev);
	if (err) {
		dev_err(dev, "failed to register input device: %d\n",
			err);
		return err;
	}

	return 0;
}

static const struct of_device_id bk_gpio_of_match[] = {
	{
		.compatible = "sharp,brain-kbd-gpio",
	},
	{ /* sentinel */ },
};

static struct platform_driver bk_gpio_driver = {
	.driver =  {
		.name = DEV_NAME,
		.of_match_table = of_match_ptr(bk_gpio_of_match),
	},
	.probe = bk_gpio_probe,
};
module_platform_driver(bk_gpio_driver);

MODULE_AUTHOR("Takumi Sueda <puhitaku@gmail.com>");
MODULE_DESCRIPTION("SHARP Brain keyboard driver");
MODULE_LICENSE("GPL v2");
