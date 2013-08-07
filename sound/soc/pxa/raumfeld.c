/*
 * raumfeld_audio.c  --  SoC audio for Raumfeld audio devices
 *
 * Copyright (c) 2013 Daniel Mack <zonque@gmail.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include <sound/pcm.h>
#include <sound/soc.h>

#include "pxa-ssp.h"

struct snd_soc_raumfeld_pxa3xx {
	struct snd_soc_card	card;

	int			mclk_reset;

	/* FIXME: use looked up clk from DT */
	struct clk		*mclk;
};

static struct i2c_client *max9486_client;
static struct i2c_board_info max9486_i2c_info = {
	I2C_BOARD_INFO("max9485", 0x63),
};

/* FIXME: move this crap to a serious driver */

#define MAX9485_MCLK_FREQ_112896 0x22
#define MAX9485_MCLK_FREQ_122880 0x23
#define MAX9485_MCLK_FREQ_225792 0x32
#define MAX9485_MCLK_FREQ_245760 0x33

static void set_max9485_clk(char clk)
{
	int ret = i2c_master_send(max9486_client, &clk, 1);
	printk("%s(): ret %d\n", __func__, ret);
}

static void raumfeld_enable_audio(struct snd_soc_raumfeld_pxa3xx *priv, bool en)
{
	if (en) {
		gpio_set_value(111, 1);

		/* wait some time to let the clocks become stable */
		msleep(100);
	} else {
		gpio_set_value(111, 0);
	}
}

/* CS4270 */
static int raumfeld_cs4270_startup(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	/* set freq to 0 to enable all possible codec sample rates */
	return snd_soc_dai_set_sysclk(codec_dai, 0, 0, 0);
}

static void raumfeld_cs4270_shutdown(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	/* set freq to 0 to enable all possible codec sample rates */
	snd_soc_dai_set_sysclk(codec_dai, 0, 0, 0);
}

static int raumfeld_cs4270_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	unsigned int fmt, clk = 0;
	int ret = 0;

	switch (params_rate(params)) {
	case 44100:
		set_max9485_clk(MAX9485_MCLK_FREQ_112896);
		clk = 11289600;
		break;
	case 48000:
		set_max9485_clk(MAX9485_MCLK_FREQ_122880);
		clk = 12288000;
		break;
	case 88200:
		set_max9485_clk(MAX9485_MCLK_FREQ_225792);
		clk = 22579200;
		break;
	case 96000:
		set_max9485_clk(MAX9485_MCLK_FREQ_245760);
		clk = 24576000;
		break;
	default:
		return -EINVAL;
	}

	fmt = SND_SOC_DAIFMT_I2S |
	      SND_SOC_DAIFMT_NB_NF |
	      SND_SOC_DAIFMT_CBS_CFS;

printk(" >>> %s() :%d\n", __func__, __LINE__);
	/* setup the CODEC DAI */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt);
	if (ret < 0)
		return ret;

printk(" >>> %s() :%d\n", __func__, __LINE__);
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, clk, 0);
	if (ret < 0)
		return ret;

	/* setup the CPU DAI */
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_clkdiv(cpu_dai, PXA_SSP_DIV_SCR, 4);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(cpu_dai, PXA_SSP_CLK_EXT, clk, 1);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops raumfeld_cs4270_ops = {
	.startup = raumfeld_cs4270_startup,
	.shutdown = raumfeld_cs4270_shutdown,
	.hw_params = raumfeld_cs4270_hw_params,
};

static int raumfeld_analog_suspend(struct snd_soc_card *card)
{
	struct snd_soc_raumfeld_pxa3xx *priv = snd_soc_card_get_drvdata(card);

	raumfeld_enable_audio(priv, false);
	return 0;
}

static int raumfeld_analog_resume(struct snd_soc_card *card)
{
	struct snd_soc_raumfeld_pxa3xx *priv = snd_soc_card_get_drvdata(card);

	raumfeld_enable_audio(priv, true);
	return 0;
}

/* AK4104 */

static int raumfeld_ak4104_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	int fmt, ret = 0, clk = 0;

	switch (params_rate(params)) {
	case 44100:
		set_max9485_clk(MAX9485_MCLK_FREQ_112896);
		clk = 11289600;
		break;
	case 48000:
		set_max9485_clk(MAX9485_MCLK_FREQ_122880);
		clk = 12288000;
		break;
	case 88200:
		set_max9485_clk(MAX9485_MCLK_FREQ_225792);
		clk = 22579200;
		break;
	case 96000:
		set_max9485_clk(MAX9485_MCLK_FREQ_245760);
		clk = 24576000;
		break;
	default:
		return -EINVAL;
	}

	fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF;

	/* setup the CODEC DAI */
	ret = snd_soc_dai_set_fmt(codec_dai, fmt | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	/* setup the CPU DAI */
	ret = snd_soc_dai_set_pll(cpu_dai, 0, 0, 0, clk);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_fmt(cpu_dai, fmt | SND_SOC_DAIFMT_CBS_CFS);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_clkdiv(cpu_dai, PXA_SSP_DIV_SCR, 4);
	if (ret < 0)
		return ret;

	ret = snd_soc_dai_set_sysclk(cpu_dai, PXA_SSP_CLK_EXT, clk, 1);
	if (ret < 0)
		return ret;

	return 0;
}

static struct snd_soc_ops raumfeld_ak4104_ops = {
	.hw_params = raumfeld_ak4104_hw_params,
};

static const struct of_device_id snd_soc_pxa3xx_raumfeld_of_ids[] = {
	{ .compatible	= "raumfeld,pxa3xx-audio" },
	{ }
};

static int snd_soc_pxa3xx_raumfeld_probe(struct platform_device *pdev)
{
	struct snd_soc_raumfeld_pxa3xx *priv;
	struct device *dev = &pdev->dev;
	struct snd_soc_dai_link *link;
	struct device_node *node;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->card.dev		= dev;
	priv->card.suspend_post	= raumfeld_analog_suspend,
	priv->card.resume_pre	= raumfeld_analog_resume,

	snd_soc_of_parse_card_name(&priv->card, "raumfeld,card-name");

	node = of_get_child_by_name(dev->of_node, "links");
	if (node) {
		struct device_node *child;

		/* iterate over child nodes */
		priv->card.num_links = of_get_child_count(node);
		if (priv->card.num_links == 0)
			return -EINVAL;

		priv->card.dai_link =
			devm_kzalloc(dev, priv->card.num_links * sizeof(*link),
				     GFP_KERNEL);
		if (!priv->card.dai_link)
			return -ENOMEM;

		link = priv->card.dai_link;

		for_each_child_of_node(node, child) {
			link->platform_of_node = of_parse_phandle(child, "raumfeld,platform", 0);
			link->codec_of_node = of_parse_phandle(child, "raumfeld,codec", 0);
			link->cpu_of_node = of_parse_phandle(child, "raumfeld,cpu", 0);

			of_property_read_string(child, "raumfeld,name",
						&link->name);
			of_property_read_string(child, "raumfeld,stream-name",
						&link->stream_name);
			of_property_read_string(child, "raumfeld,codec-dai-name",
						&link->codec_dai_name);

			/* FIXME: combine that to one common ops */
			if (strcmp(link->name, "analog") == 0)
				link->ops = &raumfeld_cs4270_ops;
			else
				link->ops = &raumfeld_ak4104_ops;

			link++;
		}
	}

	platform_set_drvdata(pdev, &priv->card);
	snd_soc_card_set_drvdata(&priv->card, priv);

	ret = snd_soc_register_card(&priv->card);
	if (ret < 0) {
		dev_err(dev, "error registering card (%d)\n", ret);
		return ret;
	}

	/* FIXME */
	priv->mclk_reset = devm_gpio_request_one(&pdev->dev, 111,
						 GPIOF_INIT_LOW, "mclk reset");
	raumfeld_enable_audio(priv, true);

	max9486_client = i2c_new_device(i2c_get_adapter(1),
					&max9486_i2c_info);

	if (!max9486_client)
		return -ENOMEM;

	set_max9485_clk(MAX9485_MCLK_FREQ_122880);


	return 0;
}

static int snd_soc_pxa3xx_raumfeld_remove(struct platform_device *pdev)
{
	struct snd_soc_card *card = platform_get_drvdata(pdev);
	struct snd_soc_raumfeld_pxa3xx *priv = snd_soc_card_get_drvdata(card);

	raumfeld_enable_audio(priv, false);
	i2c_unregister_device(max9486_client);
	return 0;
}

static struct platform_driver snd_soc_pxa3xx_raumfeld_driver = {
	.driver = {
		.owner	  = THIS_MODULE,
		.name	   = "snd-soc-pxa3xx-raumfeld",
		.of_match_table = snd_soc_pxa3xx_raumfeld_of_ids,
	},
	.probe  = snd_soc_pxa3xx_raumfeld_probe,
	.remove = snd_soc_pxa3xx_raumfeld_remove,
};

module_platform_driver(snd_soc_pxa3xx_raumfeld_driver);

/* Module information */
MODULE_AUTHOR("Daniel Mack <zonque@gmail.com>");
MODULE_DESCRIPTION("Raumfeld PXA3xx audio SoC");
MODULE_LICENSE("GPL");
