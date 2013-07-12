/*
 * clk-max9485.c: MAX9485 Programmable Audio Clock Generator
 *
 * (c) 2013, Daniel Mack <zonque@gmail.com>
 *
 * References:
 *   MAX9485 Datasheet
 *     http://www.maximintegrated.com/datasheet/index.mvp/id/4421
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/rational.h>
#include <linux/i2c.h>
#include <linux/of_platform.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <asm/div64.h>

/* this chip has only one register of 8 bit width. */

#define MAX9485_MCLK_ENABLE		(1 << 7)
#define MAX9485_CLKOUT2_ENABLE		(1 << 6)
#define MAX9485_CLKOUT1_ENABLE		(1 << 5)
#define MAX9485_SR_DOUBLE		(1 << 4)
#define MAX9485_FREQ_SCALE(X)		((X) << 2)
#define MAX9485_FREQ_SCALE_MASK		(0x3 << 2)
#define MAX9485_FREQ_SELECT(X)		(X)
#define MAX9485_FREQ_SELECT_MASK	(0x3)

struct max9485_hw_data {
	struct clk_hw	hw;
	u8		index;
};

struct max9485_driver_data {
	struct regmap *regmap;
	struct max9485_hw_data mclk_out;
	struct max9485_hw_data clk_out1;
	struct max9485_hw_data clk_out2;
	struct clk_onecell_data onecell;
};

/*
 * MAX9485 i2c regmap
 */
static struct regmap_config max9485_regmap_config = {
	.reg_bits = 0,
	.val_bits = 8,
	.max_register = 0,
};

/* MCLK OUT */
static int max9485_mclk_out_prepare(struct clk_hw *hw)
{
	struct max9485_driver_data *drvdata =
		container_of(hw, struct max9485_driver_data, mclk_out);

	return regmap_update_bits(drvdata->regmap, 0,
				  MAX9485_MCLK_ENABLE,
				  MAX9485_MCLK_ENABLE);
}

static int max9485_mclk_out_unprepare(struct clk_hw *hw)
{
	struct max9485_driver_data *drvdata =
		container_of(hw, struct max9485_driver_data, mclk_out);

	return regmap_update_bits(drvdata->regmap, 0,
				  MAX9485_MCLK_ENABLE, 0);
}

static int max9485_mclk_out_set_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long parent_rate)
{
	/*
	 * The MCLK output can only provide the
	 * same rate than the input clock
	 */
	if (rate != 27000000)
		return -EINVAL;

	return 0;
}

static u8 max9485_mclk_out_get_parent(struct clk_hw *hw)
{
	return 0;
}

static const struct clk_ops max9485_mclk_out_ops = {
	.prepare	= max9485_mclk_out_prepare,
	.unprepare	= max9485_mclk_out_unprepare,
	.set_rate	= max9485_mclk_out_set_rate,
	.get_parent	= max9485_mclk_out_get_parent,
};

/* CLKOUT1/2 */

static unsigned char max9485_clkout_get_parent(struct clk_hw *hw)
{
	return 0;
}

static long max9485_clkout_round_rate(struct clk_hw *hw, unsigned long rate,
				      unsigned long *parent_rate)
{
	return 0;
}

static int max9485_clkout_set_rate(struct clk_hw *hw, unsigned long rate,
				   unsigned long parent_rate)
{
	struct max9485_hw_data *hwdata = (struct max9485_hw_data *) hw;
	struct max9485_driver_data *drvdata;

	if (hwdata->index == 0)
		drvdata = container_of(hw, struct max9485_driver_data, clk_out1);
	else
		drvdata = container_of(hw, struct max9485_driver_data, clk_out2);

	return 0;
}

static int max9485_clkout_prepare(struct clk_hw *hw)
{
	struct max9485_hw_data *hwdata = (struct max9485_hw_data *) hw;
	struct max9485_driver_data *drvdata;

	if (hwdata->index == 0) {
		drvdata = container_of(hw, struct max9485_driver_data, clk_out1);
		return regmap_update_bits(drvdata->regmap, 0,
					  MAX9485_CLKOUT1_ENABLE,
					  MAX9485_CLKOUT1_ENABLE);
	} else {
		drvdata = container_of(hw, struct max9485_driver_data, clk_out2);
		return regmap_update_bits(drvdata->regmap, 0,
					  MAX9485_CLKOUT2_ENABLE,
					  MAX9485_CLKOUT2_ENABLE);
	}
}

static int max9485_clkout_unprepare(struct clk_hw *hw)
{
	struct max9485_hw_data *hwdata = (struct max9485_hw_data *) hw;
	struct max9485_driver_data *drvdata;

	if (hwdata->index == 0) {
		drvdata = container_of(hw, struct max9485_driver_data, clk_out1);
		return regmap_update_bits(drvdata->regmap, 0,
					  MAX9485_CLKOUT1_ENABLE, 0);
	} else {
		drvdata = container_of(hw, struct max9485_driver_data, clk_out2);
		return regmap_update_bits(drvdata->regmap, 0,
					  MAX9485_CLKOUT2_ENABLE, 0);
	}
}

static const struct clk_ops max9485_clkout_ops = {
	.get_parent	= max9485_clkout_get_parent,
	.set_rate	= max9485_clkout_set_rate,
	.prepare	= max9485_clkout_prepare,
	.unprepare	= max9485_clkout_unprepare,
};

#ifdef CONFIG_OF
static const struct of_device_id max9485_dt_ids[] = {
	{ .compatible = "silabs,max9485a", }
};
MODULE_DEVICE_TABLE(of, max9485_dt_ids);
#endif

static int max9485_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct max9485_driver_data *drvdata;
	struct clk *clk;
	int ret, n;

	drvdata = devm_kzalloc(&client->dev, sizeof(*drvdata), GFP_KERNEL);
	if (drvdata == NULL) {
		dev_err(&client->dev, "unable to allocate driver data\n");
		return -ENOMEM;
	}

	i2c_set_clientdata(client, drvdata);

	drvdata->regmap = devm_regmap_init_i2c(client, &max9485_regmap_config);
	if (IS_ERR(drvdata->regmap)) {
		dev_err(&client->dev, "failed to allocate register map\n");
		return PTR_ERR(drvdata->regmap);
	}

	clk = devm_clk_register(&client->dev, &drvdata->mclk_out);
	ret = of_clk_add_provider(client->dev.of_node, of_clk_src_onecell_get,
				  &drvdata->onecell);
	if (ret) {
		dev_err(&client->dev, "unable to add clk provider\n");
		return ret;
	}

	return 0;
}

static const struct i2c_device_id max9485_i2c_ids[] = {
	{ "max9485", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, max9485_i2c_ids);

static struct i2c_driver max9485_driver = {
	.driver = {
		.name = "max9485",
		.of_match_table = of_match_ptr(max9485_dt_ids),
	},
	.probe = max9485_i2c_probe,
	.id_table = max9485_i2c_ids,
};
module_i2c_driver(max9485_driver);

MODULE_AUTHOR("Daniel Mack <zonque@gmail.com>");
MODULE_DESCRIPTION("MAX9485 Programmable Audio Clock Generator");
MODULE_LICENSE("GPL");
