/*
 * clk-si5351.c: Silicon Laboratories Si5351A/B/C I2C Clock Generator
 *
 * Sebastian Hesselbarth <sebastian.hesselbarth@gmail.com>
 * Rabeeh Khoury <rabeeh@solid-run.com>
 *
 * References:
 * [1] "Si5351A/B/C Data Sheet"
 *     http://www.silabs.com/Support%20Documents/TechnicalDocs/Si5351.pdf
 * [2] "Manually Generating an Si5351 Register Map"
 *     http://www.silabs.com/Support%20Documents/TechnicalDocs/AN619.pdf
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/clk-private.h>
#include <linux/clkdev.h>
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

#include "clk-si5351.h"

enum si5351_variant {
	SI5351_VARIANT_A = 1,
	SI5351_VARIANT_A3 = 2,
	SI5351_VARIANT_B = 3,
	SI5351_VARIANT_C = 4,
};

struct si5351_driver_data;

struct si5351_parameters {
	unsigned long	p1;
	unsigned long	p2;
	unsigned long	p3;
	int		valid;
};

struct si5351_hw_data {
	struct clk_hw			hw;
	struct si5351_driver_data	*drvdata;
	struct si5351_parameters	params;
	unsigned char			num;
};

struct si5351_driver_data {
	enum si5351_variant	variant;
	struct i2c_client	*client;
	struct regmap		*regmap;
	struct clk_onecell_data onecell;

	struct clk		*pxtal;
	struct clk_hw		xtal;
	struct clk		*pclkin;
	struct clk_hw		clkin;

	struct si5351_hw_data	pll[2];
	struct si5351_hw_data	*msynth;
	struct si5351_hw_data	*clkout;
};

static const char const *si5351_input_names[] = {
	"xtal", "clkin"
};
static const char const *si5351_pll_names[] = {
	"plla", "pllb", "vxco"
};
static const char const *si5351_msynth_names[] = {
	"ms0", "ms1", "ms2", "ms3", "ms4", "ms5", "ms6", "ms7"
};
static const char const *si5351_clkout_names[] = {
	"clk0", "clk1", "clk2", "clk3", "clk4", "clk5", "clk6", "clk7"
};

/*
 * Si5351 i2c regmap
 */
static inline unsigned char si5351_reg_read(struct si5351_driver_data *drvdata,
	unsigned char reg)
{
	unsigned int val;
	int ret;

	ret = regmap_read(drvdata->regmap, reg, &val);
	if (ret) {
		dev_err(&drvdata->client->dev,
			"unable to read from reg%02x\n", reg);
		return 0;
	}

	return (unsigned char)val;
}

static inline int si5351_bulk_read(struct si5351_driver_data *drvdata,
	unsigned char reg, unsigned char count, unsigned char *buf)
{
	return regmap_bulk_read(drvdata->regmap, reg, buf, count);
}

static inline int si5351_reg_write(struct si5351_driver_data *drvdata,
	unsigned char reg, unsigned char val)
{
	return regmap_write(drvdata->regmap, reg, val);
}

static inline int si5351_bulk_write(struct si5351_driver_data *drvdata,
	unsigned char reg, unsigned char count, const unsigned char *buf)
{
	return regmap_raw_write(drvdata->regmap, reg, buf, count);
}

static inline int si5351_set_bits(struct si5351_driver_data *drvdata,
	unsigned char reg, unsigned char mask, unsigned char val)
{
	return regmap_update_bits(drvdata->regmap, reg, mask, val);
}

static inline unsigned char si5351_msynth_params_address(int num)
{
	if (num > 5)
		return SI5351_CLK6_PARAMETERS + (num - 6);
	return SI5351_CLK0_PARAMETERS + (SI5351_PARAMETERS_LENGTH * num);
}

static void si5351_read_parameters(struct si5351_driver_data *drvdata,
	unsigned char reg, struct si5351_parameters *params)
{
	unsigned char buf[SI5351_PARAMETERS_LENGTH];

	switch (reg) {
	case SI5351_CLK6_PARAMETERS:
	case SI5351_CLK7_PARAMETERS:
		buf[0] = si5351_reg_read(drvdata, reg);
		params->p1 = buf[0];
		params->p2 = 0;
		params->p3 = 1;
		break;
	default:
		si5351_bulk_read(drvdata, reg, SI5351_PARAMETERS_LENGTH, buf);
		params->p1 = ((buf[2] & 0x03) << 16) | (buf[3] << 8) | buf[4];
		params->p2 = ((buf[5] & 0x0f) << 16) | (buf[6] << 8) | buf[7];
		params->p3 = ((buf[5] & 0xf0) << 12) | (buf[0] << 8) | buf[1];
	}
	params->valid = 1;
}

static void si5351_write_parameters(struct si5351_driver_data *drvdata,
	unsigned char reg, struct si5351_parameters *params)
{
	unsigned char buf[SI5351_PARAMETERS_LENGTH];

	switch (reg) {
	case SI5351_CLK6_PARAMETERS:
	case SI5351_CLK7_PARAMETERS:
		buf[0] = params->p1 & 0xff;
		si5351_reg_write(drvdata, reg, buf[0]);
		break;
	default:
		buf[0] = ((params->p3 & 0x0ff00) >> 8) & 0xff;
		buf[1] = params->p3 & 0xff;
		/* save rdiv and divby4 */
		buf[2] = si5351_reg_read(drvdata, reg + 2) & ~0x03;
		buf[2] |= ((params->p1 & 0x30000) >> 16) & 0x03;
		buf[3] = ((params->p1 & 0x0ff00) >> 8) & 0xff;
		buf[4] = params->p1 & 0xff;
		buf[5] = ((params->p3 & 0xf0000) >> 12) |
			((params->p2 & 0xf0000) >> 16);
		buf[6] = ((params->p2 & 0x0ff00) >> 8) & 0xff;
		buf[7] = params->p2 & 0xff;
		si5351_bulk_write(drvdata, reg, SI5351_PARAMETERS_LENGTH, buf);
	}
}

static bool si5351_regmap_is_volatile(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case SI5351_DEVICE_STATUS:
	case SI5351_INTERRUPT_STATUS:
	case SI5351_PLL_RESET:
		return true;
	}
	return false;
}

static bool si5351_regmap_is_writeable(struct device *dev, unsigned int reg)
{
	/* reserved registers */
	if (reg >= 4 && reg <= 8)
		return false;
	if (reg >= 10 && reg <= 14)
		return false;
	if (reg >= 173 && reg <= 176)
		return false;
	if (reg >= 178 && reg <= 182)
		return false;
	/* read-only */
	if (reg == SI5351_DEVICE_STATUS)
		return false;
	return true;
}

static struct regmap_config si5351_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.cache_type = REGCACHE_RBTREE,
	.max_register = 187,
	.writeable_reg = si5351_regmap_is_writeable,
	.volatile_reg = si5351_regmap_is_volatile,
};

/*
 * Si5351 xtal clock input
 */
static int si5351_xtal_prepare(struct clk_hw *hw)
{
	struct si5351_driver_data *drvdata =
		container_of(hw, struct si5351_driver_data, xtal);
	si5351_set_bits(drvdata, SI5351_FANOUT_ENABLE,
			SI5351_XTAL_ENABLE, SI5351_XTAL_ENABLE);
	return 0;
}

static void si5351_xtal_unprepare(struct clk_hw *hw)
{
	struct si5351_driver_data *drvdata =
		container_of(hw, struct si5351_driver_data, xtal);
	si5351_set_bits(drvdata, SI5351_FANOUT_ENABLE,
			SI5351_XTAL_ENABLE, 0);
}

static const struct clk_ops si5351_xtal_ops = {
	.prepare = si5351_xtal_prepare,
	.unprepare = si5351_xtal_unprepare,
};

/*
 * Si5351 clkin clock input (Si5351C only)
 */
static int si5351_clkin_prepare(struct clk_hw *hw)
{
	struct si5351_driver_data *drvdata =
		container_of(hw, struct si5351_driver_data, clkin);
	si5351_set_bits(drvdata, SI5351_FANOUT_ENABLE,
			SI5351_CLKIN_ENABLE, SI5351_CLKIN_ENABLE);
	return 0;
}

static void si5351_clkin_unprepare(struct clk_hw *hw)
{
	struct si5351_driver_data *drvdata =
		container_of(hw, struct si5351_driver_data, clkin);
	si5351_set_bits(drvdata, SI5351_FANOUT_ENABLE,
			SI5351_CLKIN_ENABLE, 0);
}

/*
 * CMOS clock source constraints:
 * The input frequency range of the PLL is 10Mhz to 40MHz.
 * If CLKIN is >40MHz, the input divider must be used.
 */
static unsigned long si5351_clkin_recalc_rate(struct clk_hw *hw,
					      unsigned long parent_rate)
{
	struct si5351_driver_data *drvdata =
		container_of(hw, struct si5351_driver_data, clkin);
	unsigned long rate;
	unsigned char idiv;

	rate = parent_rate;
	if (parent_rate > 160000000) {
		idiv = SI5351_CLKIN_DIV_8;
		rate /= 8;
	} else if (parent_rate > 80000000) {
		idiv = SI5351_CLKIN_DIV_4;
		rate /= 4;
	} else if (parent_rate > 40000000) {
		idiv = SI5351_CLKIN_DIV_2;
		rate /= 2;
	} else {
		idiv = SI5351_CLKIN_DIV_1;
	}

	si5351_set_bits(drvdata, SI5351_PLL_INPUT_SOURCE,
			SI5351_CLKIN_DIV_MASK, idiv);

	dev_dbg(&drvdata->client->dev, "%s - clkin div = %d, rate = %lu\n",
		__func__, (1 << (idiv >> 6)), rate);

	return rate;
}

static const struct clk_ops si5351_clkin_ops = {
	.prepare = si5351_clkin_prepare,
	.unprepare = si5351_clkin_unprepare,
	.recalc_rate = si5351_clkin_recalc_rate,
};

/*
 * Si5351 vxco clock input (Si5351B only)
 */

static int si5351_vxco_prepare(struct clk_hw *hw)
{
	struct si5351_hw_data *hwdata =
		container_of(hw, struct si5351_hw_data, hw);

	dev_warn(&hwdata->drvdata->client->dev, "VXCO currently unsupported\n");

	return 0;
}

static void si5351_vxco_unprepare(struct clk_hw *hw)
{
}

static unsigned long si5351_vxco_recalc_rate(struct clk_hw *hw,
					     unsigned long parent_rate)
{
	return 0;
}

static int si5351_vxco_set_rate(struct clk_hw *hw, unsigned long rate,
				unsigned long parent)
{
	return 0;
}

static const struct clk_ops si5351_vxco_ops = {
	.prepare = si5351_vxco_prepare,
	.unprepare = si5351_vxco_unprepare,
	.recalc_rate = si5351_vxco_recalc_rate,
	.set_rate = si5351_vxco_set_rate,
};

/*
 * Si5351 pll a/b
 *
 * Feedback Multisynth Divider Equations [2]
 *
 * fVCO = fIN * (a + b/c)
 *
 * with 15 + 0/1048575 <= (a + b/c) <= 90 + 0/1048575 and
 * fIN = fXTAL or fIN = fCLKIN/CLKIN_DIV
 *
 * Feedback Multisynth Register Equations
 *
 * (1) MSNx_P1[17:0] = 128 * a + floor(128 * b/c) - 512
 * (2) MSNx_P2[19:0] = 128 * b - c * floor(128 * b/c) = (128*b) mod c
 * (3) MSNx_P3[19:0] = c
 *
 * Transposing (2) yields: (4) floor(128 * b/c) = (128 * b / MSNx_P2)/c
 *
 * Using (4) on (1) yields:
 * MSNx_P1 = 128 * a + (128 * b/MSNx_P2)/c - 512
 * MSNx_P1 + 512 + MSNx_P2/c = 128 * a + 128 * b/c
 *
 * a + b/c = (MSNx_P1 + MSNx_P2/MSNx_P3 + 512)/128
 *         = (MSNx_P1*MSNx_P3 + MSNx_P2 + 512*MSNx_P3)/(128*MSNx_P3)
 *
 */
static inline int _si5351_pll_reparent(struct si5351_driver_data *drvdata,
				       unsigned char num, unsigned char parent)
{
	if (num > 2 ||
	    (drvdata->variant == SI5351_VARIANT_B && num > 1))
		return -EINVAL;

	if (drvdata->variant != SI5351_VARIANT_C && parent > 0)
		return -EINVAL;

	return clk_set_parent(drvdata->pll[num].hw.clk, (parent) ?
			      drvdata->clkin.clk : drvdata->xtal.clk);
}

static unsigned char si5351_pll_get_parent(struct clk_hw *hw)
{
	struct si5351_hw_data *hwdata =
		container_of(hw, struct si5351_hw_data, hw);
	unsigned char mask = (hwdata->num == 0) ?
		SI5351_PLLA_SOURCE : SI5351_PLLB_SOURCE;
	unsigned char val;

	val = si5351_reg_read(hwdata->drvdata, SI5351_PLL_INPUT_SOURCE);

	return (val & mask) ? 1 : 0;
}

static int si5351_pll_set_parent(struct clk_hw *hw, u8 index)
{
	struct si5351_hw_data *hwdata =
		container_of(hw, struct si5351_hw_data, hw);
	unsigned char mask = (hwdata->num == 0) ?
		SI5351_PLLA_SOURCE : SI5351_PLLB_SOURCE;

	if (hwdata->drvdata->variant != SI5351_VARIANT_C &&
	    index > 0)
		return -EPERM;

	if (index > 1)
		return -EINVAL;

	si5351_set_bits(hwdata->drvdata, SI5351_PLL_INPUT_SOURCE,
			mask, (index) ? 0 : mask);

	return 0;
}

static unsigned long si5351_pll_recalc_rate(struct clk_hw *hw,
					    unsigned long parent_rate)
{
	struct si5351_hw_data *hwdata =
		container_of(hw, struct si5351_hw_data, hw);
	unsigned char reg = (hwdata->num == 0) ?
		SI5351_PLLA_PARAMETERS : SI5351_PLLB_PARAMETERS;
	unsigned long long rate;

	if (!hwdata->params.valid)
		si5351_read_parameters(hwdata->drvdata, reg, &hwdata->params);

	if (hwdata->params.p3 == 0)
		return parent_rate;

	/* fVCO = fIN * (P1*P3 + 512*P3 + P2)/(128*P3) */
	rate  = hwdata->params.p1 * hwdata->params.p3;
	rate += 512 * hwdata->params.p3;
	rate += hwdata->params.p2;
	rate *= parent_rate;
	do_div(rate, 128 * hwdata->params.p3);

	dev_dbg(&hwdata->drvdata->client->dev,
		"%s - %s: p1 = %lu, p2 = %lu, p3 = %lu, parent_rate = %lu, rate = %lu\n",
		__func__, hwdata->hw.clk->name,
		hwdata->params.p1, hwdata->params.p2, hwdata->params.p3,
		parent_rate, (unsigned long)rate);

	return (unsigned long)rate;
}

static long si5351_pll_round_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long *parent_rate)
{
	struct si5351_hw_data *hwdata =
		container_of(hw, struct si5351_hw_data, hw);
	unsigned long rfrac, denom, a, b, c;
	unsigned long long lltmp;

	if (rate < SI5351_PLL_VCO_MIN)
		rate = SI5351_PLL_VCO_MIN;
	if (rate > SI5351_PLL_VCO_MAX)
		rate = SI5351_PLL_VCO_MAX;

	/* determine integer part of feedback equation */
	a = rate / *parent_rate;

	if (a < SI5351_PLL_A_MIN)
		rate = *parent_rate * SI5351_PLL_A_MIN;
	if (a > SI5351_PLL_A_MAX)
		rate = *parent_rate * SI5351_PLL_A_MAX;

	/* find best approximation for b/c = fVCO mod fIN */
	denom = 1000 * 1000;
	lltmp = rate % (*parent_rate);
	lltmp *= denom;
	do_div(lltmp, *parent_rate);
	rfrac = (unsigned long)lltmp;

	b = 0;
	c = 1;
	if (rfrac)
		rational_best_approximation(rfrac, denom,
				    SI5351_PLL_B_MAX, SI5351_PLL_C_MAX, &b, &c);

	/* calculate parameters */
	hwdata->params.p3  = c;
	hwdata->params.p2  = (128 * b) % c;
	hwdata->params.p1  = 128 * a;
	hwdata->params.p1 += (128 * b / c);
	hwdata->params.p1 -= 512;

	/* recalculate rate by fIN * (a + b/c) */
	lltmp  = *parent_rate;
	lltmp *= b;
	do_div(lltmp, c);

	rate  = (unsigned long)lltmp;
	rate += *parent_rate * a;

	dev_dbg(&hwdata->drvdata->client->dev,
		"%s - %s: a = %lu, b = %lu, c = %lu, parent_rate = %lu, rate = %lu\n",
		__func__, hwdata->hw.clk->name, a, b, c, *parent_rate, rate);

	return rate;
}

static int si5351_pll_set_rate(struct clk_hw *hw, unsigned long rate,
			       unsigned long parent_rate)
{
	struct si5351_hw_data *hwdata =
		container_of(hw, struct si5351_hw_data, hw);
	unsigned char reg = (hwdata->num == 0) ?
		SI5351_PLLA_PARAMETERS : SI5351_PLLB_PARAMETERS;
	unsigned char val;

	/* write multisynth parameters */
	si5351_write_parameters(hwdata->drvdata, reg, &hwdata->params);

	/* plla/pllb ctrl is in clk6/clk7 ctrl registers */
	si5351_set_bits(hwdata->drvdata, SI5351_CLK6_CTRL + hwdata->num,
		SI5351_CLK_INTEGER_MODE,
		(hwdata->params.p2 == 0) ? SI5351_CLK_INTEGER_MODE : 0);

	/* reset pll */
	val = (hwdata->num == 0) ? SI5351_PLL_RESET_A : SI5351_PLL_RESET_B;
	si5351_set_bits(hwdata->drvdata, SI5351_PLL_RESET, val, val);

	dev_dbg(&hwdata->drvdata->client->dev,
		"%s - %s: p1 = %lu, p2 = %lu, p3 = %lu, parent_rate = %lu, rate = %lu\n",
		__func__, hwdata->hw.clk->name,
		hwdata->params.p1, hwdata->params.p2, hwdata->params.p3,
		parent_rate, rate);

	return 0;
}

static const struct clk_ops si5351_pll_ops = {
	.set_parent = si5351_pll_set_parent,
	.get_parent = si5351_pll_get_parent,
	.recalc_rate = si5351_pll_recalc_rate,
	.round_rate = si5351_pll_round_rate,
	.set_rate = si5351_pll_set_rate,
};

/*
 * Si5351 multisync divider
 *
 * for fOUT <= 150 MHz:
 *
 * fOUT = (fIN * (a + b/c)) / CLKOUTDIV
 *
 * with 6 + 0/1048575 <= (a + b/c) <= 1800 + 0/1048575 and
 * fIN = fVCO0, fVCO1, fXTAL or fCLKIN/CLKIN_DIV
 *
 * Output Clock Multisynth Register Equations
 *
 * MSx_P1[17:0] = 128 * a + floor(128 * b/c) - 512
 * MSx_P2[19:0] = 128 * b - c * floor(128 * b/c) = (128*b) mod c
 * MSx_P3[19:0] = c
 *
 * MS[6,7] are integer (P1) divide only, P2 = 0, P3 = 0
 *
 * for 150MHz < fOUT <= 160MHz:
 *
 * MSx_P1 = 0, MSx_P2 = 0, MSx_P3 = 1, MSx_INT = 1, MSx_DIVBY4 = 11b
 */
static inline void _si5351_msynth_set_pll_master(
	struct si5351_driver_data *drvdata, unsigned char num, int is_master)
{
	if (num > 8 ||
	    (drvdata->variant == SI5351_VARIANT_A3 && num > 3))
		return;

	if (is_master)
		drvdata->msynth[num].hw.clk->flags |= CLK_SET_RATE_PARENT;
	else
		drvdata->msynth[num].hw.clk->flags &= ~CLK_SET_RATE_PARENT;
}

static inline int _si5351_msynth_reparent(struct si5351_driver_data *drvdata,
				  unsigned char num, unsigned char parent)
{
	if (parent > 2)
		return -EINVAL;

	if (num > 8 ||
	    (drvdata->variant == SI5351_VARIANT_A3 && num > 3))
		return -EINVAL;

	return clk_set_parent(drvdata->msynth[num].hw.clk,
			      drvdata->pll[parent].hw.clk);
}

static unsigned char si5351_msynth_get_parent(struct clk_hw *hw)
{
	struct si5351_hw_data *hwdata =
		container_of(hw, struct si5351_hw_data, hw);
	unsigned char val;

	val = si5351_reg_read(hwdata->drvdata, SI5351_CLK0_CTRL + hwdata->num);

	return (val & SI5351_CLK_PLL_SELECT) ? 1 : 0;
}

static int si5351_msynth_set_parent(struct clk_hw *hw, u8 index)
{
	struct si5351_hw_data *hwdata =
		container_of(hw, struct si5351_hw_data, hw);

	si5351_set_bits(hwdata->drvdata, SI5351_CLK0_CTRL + hwdata->num,
			SI5351_CLK_PLL_SELECT,
			(index) ? SI5351_CLK_PLL_SELECT : 0);

	return 0;
}

static unsigned long si5351_msynth_recalc_rate(struct clk_hw *hw,
					       unsigned long parent_rate)
{
	struct si5351_hw_data *hwdata =
		container_of(hw, struct si5351_hw_data, hw);
	unsigned char reg = si5351_msynth_params_address(hwdata->num);
	unsigned long long rate;
	unsigned long m;

	if (!hwdata->params.valid)
		si5351_read_parameters(hwdata->drvdata, reg, &hwdata->params);

	if (hwdata->params.p3 == 0)
		return parent_rate;

	/*
	 * multisync0-5: fOUT = (128 * P3 * fIN) / (P1*P3 + P2 + 512*P3)
	 * multisync6-7: fOUT = fIN / P1
	 */
	rate = parent_rate;
	if (hwdata->num > 5) {
		m = hwdata->params.p1;
	} else if ((si5351_reg_read(hwdata->drvdata, reg + 2) &
		    SI5351_OUTPUT_CLK_DIVBY4) == SI5351_OUTPUT_CLK_DIVBY4) {
		m = 4;
	} else {
		rate *= 128 * hwdata->params.p3;
		m = hwdata->params.p1 * hwdata->params.p3;
		m += hwdata->params.p2;
		m += 512 * hwdata->params.p3;
	}

	if (m == 0)
		return 0;
	do_div(rate, m);

	dev_dbg(&hwdata->drvdata->client->dev,
		"%s - %s: p1 = %lu, p2 = %lu, p3 = %lu, m = %lu, parent_rate = %lu, rate = %lu\n",
		__func__, hwdata->hw.clk->name,
		hwdata->params.p1, hwdata->params.p2, hwdata->params.p3,
		m, parent_rate, (unsigned long)rate);

	return (unsigned long)rate;
}

static long si5351_msynth_round_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long *parent_rate)
{
	struct si5351_hw_data *hwdata =
		container_of(hw, struct si5351_hw_data, hw);
	unsigned long long lltmp;
	unsigned long a, b, c;
	int divby4;

	/* multisync6-7 can only handle freqencies < 150MHz */
	if (hwdata->num >= 6 && rate > SI5351_MULTISYNTH67_MAX_FREQ)
		rate = SI5351_MULTISYNTH67_MAX_FREQ;

	/* multisync frequency is 1MHz .. 160MHz */
	if (rate > SI5351_MULTISYNTH_MAX_FREQ)
		rate = SI5351_MULTISYNTH_MAX_FREQ;
	if (rate < SI5351_MULTISYNTH_MIN_FREQ)
		rate = SI5351_MULTISYNTH_MIN_FREQ;

	divby4 = 0;
	if (rate > SI5351_MULTISYNTH_DIVBY4_FREQ)
		divby4 = 1;

	/* multisync can set pll */
	if (hwdata->hw.clk->flags & CLK_SET_RATE_PARENT) {
		/*
		 * find largest integer divider for max
		 * vco frequency and given target rate
		 */
		if (divby4 == 0) {
			lltmp = SI5351_PLL_VCO_MAX;
			do_div(lltmp, rate);
			a = (unsigned long)lltmp;
		} else
			a = 4;

		b = 0;
		c = 1;

		*parent_rate = a * rate;
	} else {
		unsigned long rfrac, denom;

		/* disable divby4 */
		if (divby4) {
			rate = SI5351_MULTISYNTH_DIVBY4_FREQ;
			divby4 = 0;
		}

		/* determine integer part of divider equation */
		a = *parent_rate / rate;
		if (a < SI5351_MULTISYNTH_A_MIN)
			a = SI5351_MULTISYNTH_A_MIN;
		if (hwdata->num >= 6 && a > SI5351_MULTISYNTH67_A_MAX)
			a = SI5351_MULTISYNTH67_A_MAX;
		else if (a > SI5351_MULTISYNTH_A_MAX)
			a = SI5351_MULTISYNTH_A_MAX;

		/* find best approximation for b/c = fVCO mod fOUT */
		denom = 1000 * 1000;
		lltmp = (*parent_rate) % rate;
		lltmp *= denom;
		do_div(lltmp, rate);
		rfrac = (unsigned long)lltmp;

		b = 0;
		c = 1;
		if (rfrac)
			rational_best_approximation(rfrac, denom,
			    SI5351_MULTISYNTH_B_MAX, SI5351_MULTISYNTH_C_MAX,
			    &b, &c);
	}

	/* recalculate rate by fOUT = fIN / (a + b/c) */
	lltmp  = *parent_rate;
	lltmp *= c;
	do_div(lltmp, a * c + b);
	rate  = (unsigned long)lltmp;

	/* calculate parameters */
	if (divby4) {
		hwdata->params.p3 = 1;
		hwdata->params.p2 = 0;
		hwdata->params.p1 = 0;
	} else {
		hwdata->params.p3  = c;
		hwdata->params.p2  = (128 * b) % c;
		hwdata->params.p1  = 128 * a;
		hwdata->params.p1 += (128 * b / c);
		hwdata->params.p1 -= 512;
	}

	dev_dbg(&hwdata->drvdata->client->dev,
		"%s - %s: a = %lu, b = %lu, c = %lu, divby4 = %d, parent_rate = %lu, rate = %lu\n",
		__func__, hwdata->hw.clk->name, a, b, c, divby4, *parent_rate,
		rate);

	return rate;
}

static int si5351_msynth_set_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long parent_rate)
{
	struct si5351_hw_data *hwdata =
		container_of(hw, struct si5351_hw_data, hw);
	unsigned char reg = si5351_msynth_params_address(hwdata->num);
	int divby4 = 0;

	/* write multisynth parameters */
	si5351_write_parameters(hwdata->drvdata, reg, &hwdata->params);

	if (rate > SI5351_MULTISYNTH_DIVBY4_FREQ)
		divby4 = 1;

	/* enable/disable integer mode and divby4 on multisynth0-5 */
	if (hwdata->num < 6) {
		si5351_set_bits(hwdata->drvdata, reg + 2,
				SI5351_OUTPUT_CLK_DIVBY4,
				(divby4) ? SI5351_OUTPUT_CLK_DIVBY4 : 0);
		si5351_set_bits(hwdata->drvdata, SI5351_CLK0_CTRL + hwdata->num,
			SI5351_CLK_INTEGER_MODE,
			(hwdata->params.p2 == 0) ? SI5351_CLK_INTEGER_MODE : 0);
	}

	dev_dbg(&hwdata->drvdata->client->dev,
		"%s - %s: p1 = %lu, p2 = %lu, p3 = %lu, divby4 = %d, parent_rate = %lu, rate = %lu\n",
		__func__, hwdata->hw.clk->name,
		hwdata->params.p1, hwdata->params.p2, hwdata->params.p3,
		divby4, parent_rate, rate);

	return 0;
}

static const struct clk_ops si5351_msynth_ops = {
	.set_parent = si5351_msynth_set_parent,
	.get_parent = si5351_msynth_get_parent,
	.recalc_rate = si5351_msynth_recalc_rate,
	.round_rate = si5351_msynth_round_rate,
	.set_rate = si5351_msynth_set_rate,
};

/*
 * Si5351 clkout divider
 */
static int _si5351_clkout_set_drive_strength(struct si5351_driver_data *drvdata,
				     unsigned char num, unsigned char drive)
{
	if (num > 8 ||
	    (drvdata->variant == SI5351_VARIANT_A3 && num > 3))
		return -EINVAL;

	switch (drive) {
	case 2:
		drive = SI5351_CLK_DRIVE_2MA;
		break;
	case 4:
		drive = SI5351_CLK_DRIVE_4MA;
		break;
	case 6:
		drive = SI5351_CLK_DRIVE_6MA;
		break;
	case 8:
		drive = SI5351_CLK_DRIVE_8MA;
		break;
	default:
		return -EINVAL;
	}

	si5351_set_bits(drvdata, SI5351_CLK0_CTRL + num,
			SI5351_CLK_DRIVE_MASK, drive);

	return 0;
}

static inline int _si5351_clkout_reparent(struct si5351_driver_data *drvdata,
				  unsigned char num, unsigned char parent)
{
	struct clk *pclk;

	if (num > 8 ||
	    (drvdata->variant == SI5351_VARIANT_A3 && num > 3))
		return -EINVAL;

	drvdata->clkout[num].hw.clk->flags &= ~CLK_SET_RATE_PARENT;
	switch (parent) {
	case 0:
		pclk = drvdata->msynth[num].hw.clk;
		drvdata->clkout[num].hw.clk->flags |= CLK_SET_RATE_PARENT;
		break;
	case 1:
		pclk = drvdata->msynth[0].hw.clk;
		if (num >= 4)
			pclk = drvdata->msynth[4].hw.clk;
		break;
	case 2:
		pclk = drvdata->xtal.clk;
		break;
	case 3:
		if (drvdata->variant != SI5351_VARIANT_C)
			return -EINVAL;
		pclk = drvdata->clkin.clk;
		break;
	default:
		return -EINVAL;
	}
	return clk_set_parent(drvdata->clkout[num].hw.clk, pclk);
}

static int si5351_clkout_prepare(struct clk_hw *hw)
{
	struct si5351_hw_data *hwdata =
		container_of(hw, struct si5351_hw_data, hw);

	si5351_set_bits(hwdata->drvdata, SI5351_CLK0_CTRL + hwdata->num,
			SI5351_CLK_POWERDOWN, 0);
	si5351_set_bits(hwdata->drvdata, SI5351_OUTPUT_ENABLE_CTRL,
			(1 << hwdata->num), 0);
	return 0;
}

static void si5351_clkout_unprepare(struct clk_hw *hw)
{
	struct si5351_hw_data *hwdata =
		container_of(hw, struct si5351_hw_data, hw);

	si5351_set_bits(hwdata->drvdata, SI5351_CLK0_CTRL + hwdata->num,
			SI5351_CLK_POWERDOWN, SI5351_CLK_POWERDOWN);
	si5351_set_bits(hwdata->drvdata, SI5351_OUTPUT_ENABLE_CTRL,
			(1 << hwdata->num), (1 << hwdata->num));
}

static u8 si5351_clkout_get_parent(struct clk_hw *hw)
{
	struct si5351_hw_data *hwdata =
		container_of(hw, struct si5351_hw_data, hw);
	int index = 0;
	unsigned char val;

	val = si5351_reg_read(hwdata->drvdata, SI5351_CLK0_CTRL + hwdata->num);
	switch (val & SI5351_CLK_INPUT_MASK) {
	case SI5351_CLK_INPUT_MULTISYNTH_N:
		index = 0;
		break;
	case SI5351_CLK_INPUT_MULTISYNTH_0_4:
		index = 1;
		break;
	case SI5351_CLK_INPUT_XTAL:
		index = 2;
		break;
	case SI5351_CLK_INPUT_CLKIN:
		index = 3;
		break;
	}

	return index;
}

static int si5351_clkout_set_parent(struct clk_hw *hw, u8 index)
{
	struct si5351_hw_data *hwdata =
		container_of(hw, struct si5351_hw_data, hw);
	unsigned val;

	val = 0;
	hw->clk->flags &= ~CLK_SET_RATE_PARENT;
	switch (index) {
	case 0:
		hw->clk->flags |= CLK_SET_RATE_PARENT;
		val = SI5351_CLK_INPUT_MULTISYNTH_N;
		break;
	case 1:
		/* clk0/clk4 can only connect to its own multisync */
		if (hwdata->num == 0 || hwdata->num == 4)
			val = SI5351_CLK_INPUT_MULTISYNTH_N;
		else
			val = SI5351_CLK_INPUT_MULTISYNTH_0_4;
		break;
	case 2:
		val = SI5351_CLK_INPUT_XTAL;
		break;
	case 3:
		val = SI5351_CLK_INPUT_CLKIN;
		break;
	}
	si5351_set_bits(hwdata->drvdata, SI5351_CLK0_CTRL + hwdata->num,
			SI5351_CLK_INPUT_MASK, val);

	return 0;
}

static unsigned long si5351_clkout_recalc_rate(struct clk_hw *hw,
					       unsigned long parent_rate)
{
	struct si5351_hw_data *hwdata =
		container_of(hw, struct si5351_hw_data, hw);
	unsigned char reg;
	unsigned char rdiv;

	if (hwdata->num > 5)
		reg = si5351_msynth_params_address(hwdata->num) + 2;
	else
		reg = SI5351_CLK6_7_OUTPUT_DIVIDER;

	rdiv = si5351_reg_read(hwdata->drvdata, reg);
	if (hwdata->num == 6) {
		rdiv &= SI5351_OUTPUT_CLK6_DIV_MASK;
	} else {
		rdiv &= SI5351_OUTPUT_CLK_DIV_MASK;
		rdiv >>= SI5351_OUTPUT_CLK_DIV_SHIFT;
	}

	return parent_rate >> rdiv;
}

static long si5351_clkout_round_rate(struct clk_hw *hw, unsigned long rate,
				     unsigned long *parent_rate)
{
	struct si5351_hw_data *hwdata =
		container_of(hw, struct si5351_hw_data, hw);
	unsigned char rdiv;

	/* clkout6/7 can only handle output freqencies < 150MHz */
	if (hwdata->num >= 6 && rate > SI5351_CLKOUT67_MAX_FREQ)
		rate = SI5351_CLKOUT67_MAX_FREQ;

	/* clkout freqency is 8kHz - 160MHz */
	if (rate > SI5351_CLKOUT_MAX_FREQ)
		rate = SI5351_CLKOUT_MAX_FREQ;
	if (rate < SI5351_CLKOUT_MIN_FREQ)
		rate = SI5351_CLKOUT_MIN_FREQ;

	/* request frequency if multisync master */
	if (hwdata->hw.clk->flags & CLK_SET_RATE_PARENT) {
		/* use r divider for frequencies below 1MHz */
		rdiv = SI5351_OUTPUT_CLK_DIV_1;
		while (rate < SI5351_MULTISYNTH_MIN_FREQ &&
		       rdiv < SI5351_OUTPUT_CLK_DIV_128) {
			rdiv += 1;
			rate *= 2;
		}
		*parent_rate = rate;
	} else {
		unsigned long new_rate, new_err, err;

		/* round to closed rdiv */
		rdiv = SI5351_OUTPUT_CLK_DIV_1;
		new_rate = *parent_rate;
		err = abs(new_rate - rate);
		do {
			new_rate >>= 1;
			new_err = abs(new_rate - rate);
			if (new_err > err || rdiv == SI5351_OUTPUT_CLK_DIV_128)
				break;
			rdiv++;
			err = new_err;
		} while (1);
	}
	rate = *parent_rate >> rdiv;

	dev_dbg(&hwdata->drvdata->client->dev,
		"%s - %s: rdiv = %u, parent_rate = %lu, rate = %lu\n",
		__func__, hwdata->hw.clk->name, (1 << rdiv), *parent_rate,
		rate);

	return rate;
}

static int si5351_clkout_set_rate(struct clk_hw *hw, unsigned long rate,
				  unsigned long parent_rate)
{
	struct si5351_hw_data *hwdata =
		container_of(hw, struct si5351_hw_data, hw);
	unsigned long new_rate, new_err, err;
	unsigned char rdiv;

	/* round to closed rdiv */
	rdiv = SI5351_OUTPUT_CLK_DIV_1;
	new_rate = parent_rate;
	err = abs(new_rate - rate);
	do {
		new_rate >>= 1;
		new_err = abs(new_rate - rate);
		if (new_err > err || rdiv == SI5351_OUTPUT_CLK_DIV_128)
			break;
		rdiv++;
		err = new_err;
	} while (1);

	/* powerdown clkout */
	si5351_set_bits(hwdata->drvdata, SI5351_CLK0_CTRL + hwdata->num,
			SI5351_CLK_POWERDOWN, SI5351_CLK_POWERDOWN);

	/* write output divider */
	switch (hwdata->num) {
	case 6:
		si5351_set_bits(hwdata->drvdata, SI5351_CLK6_7_OUTPUT_DIVIDER,
				SI5351_OUTPUT_CLK6_DIV_MASK, rdiv);
		break;
	case 7:
		si5351_set_bits(hwdata->drvdata, SI5351_CLK6_7_OUTPUT_DIVIDER,
				SI5351_OUTPUT_CLK_DIV_MASK,
				rdiv << SI5351_OUTPUT_CLK_DIV_SHIFT);
		break;
	default:
		si5351_set_bits(hwdata->drvdata,
				si5351_msynth_params_address(hwdata->num) + 2,
				SI5351_OUTPUT_CLK_DIV_MASK,
				rdiv << SI5351_OUTPUT_CLK_DIV_SHIFT);
	}

	/* powerup clkout */
	si5351_set_bits(hwdata->drvdata, SI5351_CLK0_CTRL + hwdata->num,
			SI5351_CLK_POWERDOWN, 0);

	dev_dbg(&hwdata->drvdata->client->dev,
		"%s - %s: rdiv = %u, parent_rate = %lu, rate = %lu\n",
		__func__, hwdata->hw.clk->name, (1 << rdiv), parent_rate, rate);

	return 0;
}

static const struct clk_ops si5351_clkout_ops = {
	.prepare = si5351_clkout_prepare,
	.unprepare = si5351_clkout_unprepare,
	.set_parent = si5351_clkout_set_parent,
	.get_parent = si5351_clkout_get_parent,
	.recalc_rate = si5351_clkout_recalc_rate,
	.round_rate = si5351_clkout_round_rate,
	.set_rate = si5351_clkout_set_rate,
};

/*
 * Si5351 i2c probe and DT
 */
static void si5351_dt_setup(
	struct i2c_client *client, struct si5351_driver_data *drvdata)
{
	struct device_node *np = client->dev.of_node;
	struct property *prop;
	const __be32 *p;
	unsigned int num, val;

	if (np == NULL)
		return;

	/*
	 * property silabs,pll-source : <num src>, [<..>]
	 * allow to selectively set pll source
	 */
	of_property_for_each_u32(client->dev.of_node, "silabs,pll-source",
				 prop, p, num) {
		if (num >= 2) {
			dev_err(&client->dev,
				"invalid pll %d on pll-source prop\n", num);
			break;
		}

		p = of_prop_next_u32(prop, p, &val);
		if (!p)
			break;

		if (_si5351_pll_reparent(drvdata, num, val))
			dev_warn(&client->dev,
				 "unable to reparent pll %d to %d\n",
				 num, val);
	}

	for_each_child_of_node(client->dev.of_node, np) {
		if (of_property_read_u32(np, "reg", &num)) {
			dev_err(&client->dev, "missing reg property of %s\n",
				np->full_name);
			continue;
		}

		if (of_property_read_bool(np, "silabs,pll-master"))
			_si5351_msynth_set_pll_master(drvdata, num, 1);

		if (!of_property_read_u32(np, "silabs,drive-strength", &val)) {
			if (_si5351_clkout_set_drive_strength(drvdata,
							      num, val))
				dev_warn(&client->dev,
					 "unable to set drive strength of %d to %d\n",
					 num, val);
		}

		if (!of_property_read_u32(np, "silabs,multisynth-source",
					  &val)) {
			if (_si5351_msynth_reparent(drvdata, num, val))
				dev_warn(&client->dev,
					 "unable to reparent multisynth %d to %d\n",
					 num, val);
		}

		if (!of_property_read_u32(np, "silabs,clock-source", &val)) {
			if (_si5351_clkout_reparent(drvdata, num, val))
				dev_warn(&client->dev,
					 "unable to reparent clockout %d to %d\n",
					 num, val);
		}

		if (!of_property_read_u32(np, "clock-frequency", &val))
			clk_set_rate(drvdata->onecell.clks[num], val);
	}
}

static const struct of_device_id si5351_dt_ids[] = {
	{ .compatible = "silabs,si5351a", .data = (void *)SI5351_VARIANT_A, },
	{ .compatible = "silabs,si5351a-msop",
					 .data = (void *)SI5351_VARIANT_A3, },
	{ .compatible = "silabs,si5351b", .data = (void *)SI5351_VARIANT_B, },
	{ .compatible = "silabs,si5351c", .data = (void *)SI5351_VARIANT_C, },
	{ }
};
MODULE_DEVICE_TABLE(of, si5351_dt_ids);

static int si5351_dt_parse(
	struct i2c_client *client, struct si5351_driver_data *drvdata)
{
	struct device_node *np = client->dev.of_node;
	const struct of_device_id *match;

	if (np == NULL)
		return -EINVAL;

	match = of_match_node(si5351_dt_ids, np);
	if (match == NULL)
		return -EINVAL;

	drvdata->variant = (enum si5351_variant)match->data;
	drvdata->pxtal = of_clk_get(np, 0);
	drvdata->pclkin = of_clk_get(np, 1);

	return 0;
}

static int si5351_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct si5351_driver_data *drvdata;
	struct clk_init_data init;
	struct clk *clk;
	const char *parent_names[4];
	u8 num_parents, num_clocks;
	int ret, n;

	drvdata = devm_kzalloc(&client->dev, sizeof(*drvdata), GFP_KERNEL);
	if (drvdata == NULL) {
		dev_err(&client->dev, "unable to allocate driver data\n");
		return -ENOMEM;
	}

	ret = si5351_dt_parse(client, drvdata);
	if (ret)
		return ret;

	i2c_set_clientdata(client, drvdata);
	drvdata->client = client;
	drvdata->regmap = devm_regmap_init_i2c(client, &si5351_regmap_config);
	if (IS_ERR(drvdata->regmap)) {
		dev_err(&client->dev, "failed to allocate register map\n");
		return PTR_ERR(drvdata->regmap);
	}

	/* Disable interrupts */
	si5351_reg_write(drvdata, SI5351_INTERRUPT_MASK, 0xf0);
	/* Set disabled output drivers to drive low */
	si5351_reg_write(drvdata, SI5351_CLK3_0_DISABLE_STATE, 0x00);
	si5351_reg_write(drvdata, SI5351_CLK7_4_DISABLE_STATE, 0x00);
	/* Ensure pll select is on XTAL for Si5351A/B */
	if (drvdata->variant != SI5351_VARIANT_C)
		si5351_set_bits(drvdata, SI5351_PLL_INPUT_SOURCE,
				SI5351_PLLA_SOURCE | SI5351_PLLB_SOURCE, 0);

	/* register xtal input clock gate */
	memset(&init, 0, sizeof(init));
	init.name = si5351_input_names[0];
	init.ops = &si5351_xtal_ops;
	init.flags = 0;
	if (!IS_ERR(drvdata->pxtal)) {
		init.parent_names = &drvdata->pxtal->name;
		init.num_parents = 1;
	}
	drvdata->xtal.init = &init;
	clk = devm_clk_register(&client->dev, &drvdata->xtal);
	if (IS_ERR(clk)) {
		dev_err(&client->dev, "unable to register %s\n", init.name);
		return PTR_ERR(clk);
	}

	/* register clkin input clock gate */
	if (drvdata->variant == SI5351_VARIANT_C) {
		memset(&init, 0, sizeof(init));
		init.name = si5351_input_names[1];
		init.ops = &si5351_clkin_ops;
		if (!IS_ERR(drvdata->pclkin)) {
			init.parent_names = &drvdata->pclkin->name;
			init.num_parents = 1;
		}
		drvdata->clkin.init = &init;
		clk = devm_clk_register(&client->dev, &drvdata->clkin);
		if (IS_ERR(clk)) {
			dev_err(&client->dev, "unable to register %s\n",
				init.name);
			return PTR_ERR(clk);
		}
	}

	/* Si5351C allows to mux either xtal or clkin to PLL input */
	num_parents = (drvdata->variant == SI5351_VARIANT_C) ? 2 : 1;
	parent_names[0] = si5351_input_names[0];
	parent_names[1] = si5351_input_names[1];

	/* register PLLA */
	drvdata->pll[0].num = 0;
	drvdata->pll[0].drvdata = drvdata;
	drvdata->pll[0].hw.init = &init;
	memset(&init, 0, sizeof(init));
	init.name = si5351_pll_names[0];
	init.ops = &si5351_pll_ops;
	init.flags = 0;
	init.parent_names = parent_names;
	init.num_parents = num_parents;
	clk = devm_clk_register(&client->dev, &drvdata->pll[0].hw);
	if (IS_ERR(clk)) {
		dev_err(&client->dev, "unable to register %s\n", init.name);
		return -EINVAL;
	}

	/* register PLLB or VXCO (Si5351B) */
	drvdata->pll[1].num = 1;
	drvdata->pll[1].drvdata = drvdata;
	drvdata->pll[1].hw.init = &init;
	memset(&init, 0, sizeof(init));
	if (drvdata->variant == SI5351_VARIANT_B) {
		init.name = si5351_pll_names[2];
		init.ops = &si5351_vxco_ops;
		init.flags = CLK_IS_ROOT;
		init.parent_names = NULL;
		init.num_parents = 0;
	} else {
		init.name = si5351_pll_names[1];
		init.ops = &si5351_pll_ops;
		init.flags = 0;
		init.parent_names = parent_names;
		init.num_parents = num_parents;
	}
	clk = devm_clk_register(&client->dev, &drvdata->pll[1].hw);
	if (IS_ERR(clk)) {
		dev_err(&client->dev, "unable to register %s\n", init.name);
		return -EINVAL;
	}

	/* register clk multisync and clk out divider */
	num_clocks = (drvdata->variant == SI5351_VARIANT_A3) ? 3 : 8;
	parent_names[0] = si5351_pll_names[0];
	if (drvdata->variant == SI5351_VARIANT_B)
		parent_names[1] = si5351_pll_names[2];
	else
		parent_names[1] = si5351_pll_names[1];

	drvdata->msynth = devm_kzalloc(&client->dev, num_clocks *
				       sizeof(*drvdata->msynth), GFP_KERNEL);
	drvdata->clkout = devm_kzalloc(&client->dev, num_clocks *
				       sizeof(*drvdata->clkout), GFP_KERNEL);

	drvdata->onecell.clk_num = num_clocks;
	drvdata->onecell.clks = devm_kzalloc(&client->dev,
		num_clocks * sizeof(*drvdata->onecell.clks), GFP_KERNEL);

	if (WARN_ON(!drvdata->msynth || !drvdata->clkout ||
		    !drvdata->onecell.clks))
		return -ENOMEM;

	for (n = 0; n < num_clocks; n++) {
		drvdata->msynth[n].num = n;
		drvdata->msynth[n].drvdata = drvdata;
		drvdata->msynth[n].hw.init = &init;
		memset(&init, 0, sizeof(init));
		init.name = si5351_msynth_names[n];
		init.ops = &si5351_msynth_ops;
		init.flags = 0;
		init.parent_names = parent_names;
		init.num_parents = 2;
		clk = devm_clk_register(&client->dev, &drvdata->msynth[n].hw);
		if (IS_ERR(clk)) {
			dev_err(&client->dev, "unable to register %s\n",
				init.name);
			return -EINVAL;
		}
	}

	num_parents = (drvdata->variant == SI5351_VARIANT_C) ? 4 : 3;
	parent_names[2] = si5351_input_names[0];
	parent_names[3] = si5351_input_names[1];
	for (n = 0; n < num_clocks; n++) {
		parent_names[0] = si5351_msynth_names[n];
		parent_names[1] = (n < 4) ? si5351_msynth_names[0] :
			si5351_msynth_names[4];

		drvdata->clkout[n].num = n;
		drvdata->clkout[n].drvdata = drvdata;
		drvdata->clkout[n].hw.init = &init;
		memset(&init, 0, sizeof(init));
		init.name = si5351_clkout_names[n];
		init.ops = &si5351_clkout_ops;
		init.flags = 0;
		init.parent_names = parent_names;
		init.num_parents = num_parents;
		clk = devm_clk_register(&client->dev, &drvdata->clkout[n].hw);
		if (IS_ERR(clk)) {
			dev_err(&client->dev, "unable to register %s\n",
				init.name);
			return -EINVAL;
		}
		drvdata->onecell.clks[n] = clk;
	}

	/* setup clock setup from DT */
	si5351_dt_setup(client, drvdata);

	ret = of_clk_add_provider(client->dev.of_node, of_clk_src_onecell_get,
				  &drvdata->onecell);
	if (ret) {
		dev_err(&client->dev, "unable to add clk provider\n");
		return ret;
	}

	return 0;
}

static const struct i2c_device_id si5351_i2c_ids[] = {
	{ "silabs,si5351", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, si5351_i2c_ids);

static struct i2c_driver si5351_driver = {
	.driver = {
		.name = "si5351",
		.of_match_table = si5351_dt_ids,
	},
	.probe = si5351_i2c_probe,
	.id_table = si5351_i2c_ids,
};
module_i2c_driver(si5351_driver);

MODULE_AUTHOR("Sebastian Hesselbarth <sebastian.hesselbarth@gmail.de");
MODULE_DESCRIPTION("Silicon Labs Si5351A/B/C clock generator driver");
MODULE_LICENSE("GPL");
