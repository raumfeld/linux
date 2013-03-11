/*
 * ASoC driver for StreamUnlimited S800/Raumfeld AM33xx based audio devices
 *
 *  (c) 2013 Daniel Mack <daniel@zonque.org>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/pinctrl/consumer.h>
#include <linux/clk.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define RAUMFELD_DAI_FMT \
	(SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS | SND_SOC_DAIFMT_NB_NF)

#define DATA_WORD_WIDTH 32

struct snd_soc_am33xx_s800 {
	struct snd_soc_card	card;
	struct clk 		*mclk;
	unsigned int		mclk_rate;
	signed int		drift;
};

static int am33xx_s800_set_mclk(struct snd_soc_am33xx_s800 *priv)
{
	int ret;
	unsigned int drift;
	int sgn = priv->drift > 0 ? 1:-1;
	signed long comp, clk;

	drift = priv->drift * sgn;
	comp = ((priv->mclk_rate / DATA_WORD_WIDTH) * drift ) / (1000000ULL / DATA_WORD_WIDTH) ;
	comp *= sgn;
	clk = priv->mclk_rate - comp;

	ret = clk_set_rate(priv->mclk, clk);
	if (ret < 0)
		return ret;

	return 0;
}

static int am33xx_s800_i2s_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *card = codec_dai->card;
	struct snd_soc_am33xx_s800 *priv = snd_soc_card_get_drvdata(card);
	unsigned int clk, rate = params_rate(params);
	int ret;

	clk = priv->mclk_rate = (rate % 16000 == 0) ? 24576000 : 22579200;

	ret = am33xx_s800_set_mclk(priv);
	if (ret < 0)
		return ret;

	/* propagate the clock rate */
	ret = snd_soc_dai_set_sysclk(cpu_dai, 0, clk, SND_SOC_CLOCK_IN);
	if (ret < 0)
		return ret;

	/* intentionally ignore errors - the codec driver may not care */
	snd_soc_dai_set_sysclk(codec_dai, 0, clk, SND_SOC_CLOCK_IN);

	/* MCLK divider */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, 0, 1);
	if (ret < 0)
		return ret;

	/* BCLK divider */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, 1, clk / (rate * 2 * DATA_WORD_WIDTH));
	if (ret < 0)
		return ret;

	/* BCLK-to-LRCLK divider */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, 2, 2 * DATA_WORD_WIDTH);
	if (ret < 0)
		return ret;

	return 0;
}

static int am33xx_s800_i2c_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_card *card = codec_dai->card;
	struct snd_soc_am33xx_s800 *priv = snd_soc_card_get_drvdata(card);

	priv->mclk_rate = 0;

	return 0;
}

static struct snd_soc_ops am33xx_s800_dai_link_ops = {
	.hw_params	= am33xx_s800_i2s_hw_params,
	.hw_free	= am33xx_s800_i2c_hw_free,
};

static int am33xx_s800_drift_info(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->value.integer.min = -500;	/* +/- 500ppm */
	uinfo->value.integer.max = 500;
	uinfo->count = 1;

	return 0;
}

static int am33xx_s800_drift_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
        struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct snd_soc_am33xx_s800 *priv = snd_soc_card_get_drvdata(card);

	ucontrol->value.integer.value[0] = priv->drift;

	return 0;
}

static int am33xx_s800_drift_put(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
        struct snd_soc_card *card =  snd_kcontrol_chip(kcontrol);
	struct snd_soc_am33xx_s800 *priv = snd_soc_card_get_drvdata(card);
	int ret;

	if (ucontrol->value.integer.value[0] == priv->drift)
		return 0;

        priv->drift = ucontrol->value.integer.value[0];

	if (priv->mclk_rate) {
		ret = am33xx_s800_set_mclk(priv);
		if (ret < 0)
			dev_warn(card->dev,
				 "Unable to set clock rate: %d\n", ret);
	}

        return 1;
}

static const struct snd_kcontrol_new am33xx_s800_controls[] = {
	{
		.iface 	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name	= "Drift compensator",
		.info	= am33xx_s800_drift_info,
		.get	= am33xx_s800_drift_get,
		.put	= am33xx_s800_drift_put,
	},
};

static const struct of_device_id snd_soc_am33xx_s800_match[] = {
	{ .compatible	= "sue,am33xx-generic-audio" },
	{ }
};

static int snd_soc_am33xx_s800_probe(struct platform_device *pdev)
{
	int ret;
	unsigned int dai_fmt;
	struct device *dev = &pdev->dev;
	struct device_node *child, *np = dev->of_node;
	struct snd_soc_am33xx_s800 *priv;
	struct snd_soc_dai_link *link;
	struct pinctrl *pinctrl;
        const struct of_device_id *of_id =
                        of_match_device(snd_soc_am33xx_s800_match, dev);

	if (!of_id)
		return -ENODEV;

	priv = devm_kzalloc(dev, sizeof(struct snd_soc_am33xx_s800),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS;

	if (of_get_property(np, "sue,invert-wclk", NULL))
		dai_fmt |= SND_SOC_DAIFMT_NB_IF;
	else
		dai_fmt |= SND_SOC_DAIFMT_NB_NF;

	/* request pin mux */
	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl))
		dev_warn(dev, "pins are not configured from the driver\n");

	priv->mclk = of_clk_get(np, 0);
	if (IS_ERR(priv->mclk)) {
		dev_err(dev, "failed to get MCLK\n");
		return -EPROBE_DEFER;
	}

	/* machine controls */
	priv->card.controls = am33xx_s800_controls;
	priv->card.num_controls = ARRAY_SIZE(am33xx_s800_controls);

	priv->card.dev = dev;
	snd_soc_of_parse_card_name(&priv->card, "sue,card-name");

	/* iterate over child nodes */
	priv->card.num_links = of_get_child_count(np);
	if (priv->card.num_links == 0)
		return -EINVAL;

	priv->card.dai_link =
		devm_kzalloc(dev, priv->card.num_links * sizeof(*link),
			     GFP_KERNEL);
	if (!priv->card.dai_link)
		return -ENOMEM;

	link = priv->card.dai_link;

	for_each_child_of_node(np, child) {
		link->platform_of_node = of_parse_phandle(child, "sue,platform", 0);
		link->codec_of_node = of_parse_phandle(child, "sue,codec", 0);

		of_property_read_string(child, "sue,name",
					&link->name);
		of_property_read_string(child, "sue,stream-name",
					&link->stream_name);
		of_property_read_string(child, "sue,cpu-dai-name",
					&link->cpu_dai_name);
		of_property_read_string(child, "sue,codec-dai-name",
					&link->codec_dai_name);

		link->ops = &am33xx_s800_dai_link_ops;
		link->dai_fmt = dai_fmt;

		link++;
	}

	platform_set_drvdata(pdev, &priv->card);
	snd_soc_card_set_drvdata(&priv->card, priv);

	ret = snd_soc_register_card(&priv->card);
	if (ret < 0)
		dev_err(dev, "error registering card (%d)\n", ret);

	return 0;
}

static int snd_soc_am33xx_s800_remove(struct platform_device *pdev)
{
	return 0;
}

static struct platform_driver snd_soc_am33xx_s800_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= "snd-soc-am33xx-s800",
		.of_match_table	= snd_soc_am33xx_s800_match,
	},
	.probe	= snd_soc_am33xx_s800_probe,
	.remove	= snd_soc_am33xx_s800_remove,
};

module_platform_driver(snd_soc_am33xx_s800_driver);

MODULE_AUTHOR("Daniel Mack <daniel@zonque.org>");
MODULE_DESCRIPTION("Stream Unlimited S800 / Raumfeld ASoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:snd-soc-am33xx-s800");
