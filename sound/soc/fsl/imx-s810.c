/*
 * ASoC driver for StreamUnlimited S810/Teufel i.MX based audio devices
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
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#define MCLK_48k	24576000
#define MCLK_44k1	22579200

#define IMX7D_SAI_PLL_48k	884736000UL
#define IMX7D_SAI_PLL_44k1	812851200UL

struct snd_soc_imx_s810 {
	struct snd_soc_card	card;
	struct clk 		*mclk;
	struct clk		*mclk_rx;
	unsigned int		mclk_rate;
	unsigned int		mclk_rate_current;
	unsigned int		mclk_rate_rx;
	s32			drift;
	int			passive_mode_gpio;
	int			cb_reset_gpio;
	int			amp_overheat_gpio;
	int			amp_overcurrent_gpio;
	struct snd_kcontrol	*amp_overheat_kctl;
	struct regulator	*regulator;
	const char		*serial_config; /* I (I2S only), D (DSD only), M (I2S and DSD), S (SPDIF), - (do not use) */

	struct pinctrl *pinctrl;
	struct pinctrl_state *pinctrl_state_pcm, *pinctrl_state_dsd;

	/* i.MX7D specific */
	struct clk *pllclk;
	u32 nominal_pll_rate;
};

/*
 * This function applies the drift in ppm to the current PLL value.
 * If no PLL is specified nothing happens.
 */
static int imx_s810_apply_drift(struct snd_soc_card *card)
{
	int ret;
	s32 sgn, comp, drift;
	u32 clk_rate;
	struct snd_soc_imx_s810 *priv = snd_soc_card_get_drvdata(card);

	if (IS_ERR(priv->pllclk))
		return 0;

	drift = priv->drift;
	sgn = drift > 0 ? 1 : -1;

	drift = abs(drift);
	comp = DIV_ROUND_CLOSEST_ULL((u64)priv->nominal_pll_rate * drift, 1000000UL);

	clk_rate = priv->nominal_pll_rate - (comp * sgn);

	dev_dbg(card->dev, "drift is %d ppm, new PLL rate is %u\n", priv->drift, clk_rate);

	ret = clk_set_rate(priv->pllclk, clk_rate);
	if (ret)
		dev_warn(card->dev, "failed to set PLL rate %d\n", ret);

	return 0;
}

static int imx_s810_set_mclk(struct snd_soc_imx_s810 *priv, int stream)
{
	struct clk *mclk;
	unsigned long mclk_rate;
	int ret;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		mclk = priv->mclk;
		mclk_rate = priv->mclk_rate_current;

	} else {
		mclk = priv->mclk_rx;
		mclk_rate = priv->mclk_rate_rx;
	}

	ret = clk_set_rate(mclk, mclk_rate);
	if (ret < 0)
		return ret;

	ret = clk_prepare_enable(mclk);
	if (ret < 0)
		return ret;

	return 0;
}

static int snd_soc_imx_s810_set_control(struct snd_card *card,
					   const char *name,
					   const char *value)
{
	struct snd_ctl_elem_id id;
	struct snd_kcontrol *ctl;
	struct snd_ctl_elem_value val;
	struct snd_ctl_elem_info *info;
	int i, ret = 0;

	memset(&id, 0, sizeof(id));
	memset(&val, 0, sizeof(val));

	id.iface = SNDRV_CTL_ELEM_IFACE_MIXER;

	strlcpy(id.name, name, sizeof(id.name));

	ctl = snd_ctl_find_id(card, &id);
	if (!ctl) {
		dev_warn(card->dev, "Unknown control name '%s'\n", name);
		return -ENOENT;
	}

	if (!ctl->put || !ctl->info) {
		dev_warn(card->dev, "Control '%s' not writable\n", name);
		return -ENOENT;
	}

	info = kzalloc(sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	ret = ctl->info(ctl, info);
	if (ret < 0) {
		dev_warn(card->dev, "Unable to get info for '%s'\n", name);
		goto exit_free_info;
	}

	if (info->type != SNDRV_CTL_ELEM_TYPE_ENUMERATED) {
		dev_warn(card->dev, "Control '%s' is not an enum\n", name);
		ret = -EINVAL;
		goto exit_free_info;
	}

	for (i = 0; i < info->value.enumerated.items; i++) {
		info->value.enumerated.item = i;
		ctl->info(ctl, info);

		if (strcmp(info->value.enumerated.name, value) != 0)
			continue;

		val.value.enumerated.item[0] = i;

		ret = ctl->put(ctl, &val);
		if (ret < 0) {
			dev_warn(card->dev, "Unable to write control '%s'\n",
				 name);
			goto exit_free_info;
		}

		dev_warn(card->dev, "Control default '%s' -> '%s'\n",
			 name, value);

		goto exit_free_info;
	}

	dev_warn(card->dev, "Enum '%s' has no entry '%s'\n", name, value);

exit_free_info:
	kfree(info);
	return ret;
}

static int imx_s810_common_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params,
					bool is_spdif)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *card = codec_dai->component->card;
	struct snd_soc_imx_s810 *priv = snd_soc_card_get_drvdata(card);
	unsigned int mclk, rate;
	int ret;
	int clk_id, div_mclk, div_bclk, div_lrclk;

	const unsigned int base_44100_clks = 11025;
	const unsigned int base_48000_clks = 12000;

	rate = params_rate(params);

	mclk = (rate % 16000 == 0) ? MCLK_48k : MCLK_44k1;

	if (priv->mclk_rate) {
		if (priv->mclk_rate % base_48000_clks == 0)
			mclk = (rate % 8000 == 0) ? priv->mclk_rate :
				priv->mclk_rate / 48000 * 44100;
		else if (priv->mclk_rate % base_44100_clks == 0)
			mclk = (rate % 8000 != 0) ? priv->mclk_rate :
				priv->mclk_rate / 44100 * 48000;
	}

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		priv->mclk_rate_current = mclk;

		clk_id = 0;
		div_mclk = 0;
		div_bclk = 1;
		div_lrclk = 2;

	} else {
		priv->mclk_rate_rx = mclk;

		clk_id = 1;
		div_mclk = 10;
		div_bclk = 11;
		div_lrclk = 12;
	}

	/* if the codec is MCLK master then do not configure our MCLK source */
	if ((rtd->dai_link->dai_fmt & SND_SOC_DAIFMT_CMM) == 0) {
		if (IS_ERR(priv->pllclk)) {
			dev_warn(card->dev, "no PLL clk available\n");
		} else {
			u32 pllrate = 0;

			/*
			 * Depending on the audio rate we want to use different PLL rates to
			 * to divide the PLL rate down without any error.
			 */
			if ((rate % 8000) == 0)
				pllrate = IMX7D_SAI_PLL_48k;
			else
				pllrate = IMX7D_SAI_PLL_44k1;

			ret = clk_set_rate(priv->pllclk, pllrate);
			if (ret)
				dev_warn(card->dev, "failed to set PLL rate %d\n", ret);

			dev_info(card->dev, "Audio pll set to %u\n", pllrate);
			priv->nominal_pll_rate = clk_get_rate(priv->pllclk);
		}

		ret = imx_s810_set_mclk(priv, substream->stream);
		if (ret < 0) {
			dev_warn(card->dev, "Unsupported MCLK source : %d\n", ret);
			return ret;
		}

		if (!IS_ERR(priv->pllclk)) {
			priv->drift = 0;
			imx_s810_apply_drift(card);
		}
	}

	/* CPU MLCK */
	ret = snd_soc_dai_set_sysclk(cpu_dai, clk_id, mclk, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_warn(card->dev, "Unsupported cpu dai MCLK : %d\n", ret);
		return ret;
	}

	/* Codec MCLK */
	/* intentionally ignore errors - the codec driver may not care, at least give a warning */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, mclk, SND_SOC_CLOCK_IN);

	return 0;
}

static int imx_s810_i2s_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	return imx_s810_common_hw_params(substream, params, false);
}

static struct snd_soc_ops imx_s810_i2s_dai_link_ops = {
	.hw_params	= imx_s810_i2s_hw_params,
};

static int imx_s810_spdif_hw_params(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *params)
{
	return imx_s810_common_hw_params(substream, params, true);
}

static struct snd_soc_ops imx_s810_spdif_dai_link_ops = {
	.hw_params	= imx_s810_spdif_hw_params,
};

static int imx_s810_drift_info(struct snd_kcontrol *kcontrol,
				      struct snd_ctl_elem_info *uinfo)
{
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->value.integer.min = -500;	/* +/- 500ppm */
	uinfo->value.integer.max = 500;
	uinfo->count = 1;

	return 0;
}

static int imx_s810_drift_get(struct snd_kcontrol *kcontrol,
				 struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct snd_soc_imx_s810 *priv = snd_soc_card_get_drvdata(card);

	ucontrol->value.integer.value[0] = priv->drift;

	return 0;
}

static int imx_s810_drift_put(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct snd_soc_imx_s810 *priv = snd_soc_card_get_drvdata(card);

	if (ucontrol->value.integer.value[0] == priv->drift)
		return 0;

	priv->drift = ucontrol->value.integer.value[0];

	imx_s810_apply_drift(card);

        return 1;
}

static const struct snd_kcontrol_new imx_s810_controls[] = {
	{
		.iface 	= SNDRV_CTL_ELEM_IFACE_MIXER,
		.name	= "Drift compensator",
		.info	= imx_s810_drift_info,
		.get	= imx_s810_drift_get,
		.put	= imx_s810_drift_put,
	},
};

static int imx_s810_passive_mode_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct snd_soc_imx_s810 *priv = snd_soc_card_get_drvdata(card);

	ucontrol->value.integer.value[0] =
		!gpio_get_value_cansleep(priv->passive_mode_gpio);
	return 0;
}

static int imx_s810_passive_mode_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct snd_soc_imx_s810 *priv = snd_soc_card_get_drvdata(card);

	gpio_set_value(priv->passive_mode_gpio,
		       !ucontrol->value.integer.value[0]);
        return 1;
}

static const struct snd_kcontrol_new imx_s810_passive_mode_control =
	SOC_SINGLE_BOOL_EXT("Passive mode", 0,
			    imx_s810_passive_mode_get,
			    imx_s810_passive_mode_put);

static int imx_s810_amp_overheat_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct snd_soc_imx_s810 *priv = snd_soc_card_get_drvdata(card);

	ucontrol->value.integer.value[0] =
		!gpio_get_value_cansleep(priv->amp_overheat_gpio);

	return 0;
}

static const struct snd_kcontrol_new imx_s810_amp_overheat_control =
	SOC_SINGLE_BOOL_EXT("Amplifier Overheat Sensor", 0,
			    imx_s810_amp_overheat_get,
			    NULL);

static irqreturn_t imx_s810_amp_overheat_irq(int irq, void *data)
{
	struct snd_soc_imx_s810 *priv = data;

	snd_ctl_notify(priv->card.snd_card, SNDRV_CTL_EVENT_MASK_VALUE,
		       &priv->amp_overheat_kctl->id);

	return IRQ_HANDLED;
}

static irqreturn_t imx_s810_amp_overcurrent_irq(int irq, void *data)
{
	struct snd_soc_imx_s810 *priv = data;

	dev_warn(priv->card.dev, "Amplifier signaled overcurrent/shutdown condition");

	return IRQ_HANDLED;
}

static const struct of_device_id snd_soc_imx_s810_match[] = {
	{ .compatible	= "sue,imx7-s810-audio" },
	{ }
};

static int snd_soc_imx_s810_probe(struct platform_device *pdev)
{
	int ret;
	unsigned int dai_fmt;
	struct device *dev = &pdev->dev;
	struct device_node *top_node, *node;
	struct snd_soc_imx_s810 *priv;
	struct snd_soc_dai_link *link;
        const struct of_device_id *of_id =
                        of_match_device(snd_soc_imx_s810_match, dev);

	top_node = dev->of_node;

	if (!of_id)
		return -ENODEV;

	priv = devm_kzalloc(dev, sizeof(struct snd_soc_imx_s810),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF;

	priv->pllclk = devm_clk_get(&pdev->dev, "pll");
	if (IS_ERR(priv->pllclk))
		dev_dbg(&pdev->dev, "could not get PLL clock: %ld\n", PTR_ERR(priv->pllclk));

	/* Get the default rate on boot */
	if (!IS_ERR(priv->pllclk))
		priv->nominal_pll_rate = clk_get_rate(priv->pllclk);

	priv->mclk = devm_clk_get(&pdev->dev, "mclk");
	if (IS_ERR(priv->mclk)) {
		dev_err(dev, "failed to get MCLK\n");
		return -EPROBE_DEFER;
	}

	/* this is a hack to temporarily disable the MCLK in test mode */
	if (of_get_property(top_node, "sue,disable-clk", NULL)) {
		clk_prepare_enable(priv->mclk);
		clk_disable_unprepare(priv->mclk);
		return 0;
	}

	/* machine controls */
	priv->card.controls = imx_s810_controls;
	priv->card.num_controls = ARRAY_SIZE(imx_s810_controls);

	priv->card.dev = dev;
	snd_soc_of_parse_card_name(&priv->card, "sue,card-name");

	node = of_get_child_by_name(top_node, "links");
	if (node) {
		struct device_node *child;

		/* iterate over child nodes */
		priv->card.num_links = of_get_child_count(node);
		if (priv->card.num_links == 0) {
			dev_err(dev, "Faild to find any links in device tree\n");
			return -EINVAL;
		}

		priv->card.dai_link =
			devm_kzalloc(dev, priv->card.num_links * sizeof(*link),
				     GFP_KERNEL);
		if (!priv->card.dai_link)
			return -ENOMEM;

		link = priv->card.dai_link;

		for_each_child_of_node(node, child) {
			unsigned int dai_fmt_link = 0;

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

			if (of_get_property(child, "sue,codec-is-bfclk-master", NULL))
				dai_fmt_link |= SND_SOC_DAIFMT_CBM_CFM;
			else
				dai_fmt_link |= SND_SOC_DAIFMT_CBS_CFS;

			if (of_get_property(child, "sue,codec-is-mclk-master", NULL))
				dai_fmt_link |= SND_SOC_DAIFMT_CMM;


			if (of_get_property(child, "sue,spdif", NULL))
				link->ops = &imx_s810_spdif_dai_link_ops;
			else
				link->ops = &imx_s810_i2s_dai_link_ops;

			link->dai_fmt = dai_fmt | dai_fmt_link;
			link++;
		}
	} else {
		dev_err(dev, "Faild to find links node in device tree\n");
		return -EINVAL;
	}

	platform_set_drvdata(pdev, &priv->card);
	snd_soc_card_set_drvdata(&priv->card, priv);

	if (priv->regulator) {
		ret = regulator_enable(priv->regulator);
		if (ret < 0) {
			dev_err(dev, "error enabling regulator\n");
			return ret;
		}
	}

	if (of_property_read_u32(top_node, "mclk_rate", &priv->mclk_rate)) {
		dev_err(dev, "Invalid value for mclk_rate in device tree!\n");
		return -EINVAL;
	}

	// TODO: Maybe disable MCLK again if snd_soc_register_card() fails?
	if (of_get_property(top_node, "sue,early-mclk", NULL)) {
		u32 pllrate = IMX7D_SAI_PLL_48k;
		dev_info(dev, "enabling early MCLK\n");

		clk_set_rate(priv->pllclk, pllrate);
		priv->mclk_rate_current = priv->mclk_rate;
		imx_s810_set_mclk(priv, SNDRV_PCM_STREAM_PLAYBACK);
	}

	priv->cb_reset_gpio = of_get_named_gpio(top_node, "sue,cb-reset-gpio", 0);
	if (gpio_is_valid(priv->cb_reset_gpio)) {
		ret = devm_gpio_request_one(dev, priv->cb_reset_gpio, GPIOF_OUT_INIT_LOW, "Carrier board reset GPIO");

		if (ret == 0) {
			usleep_range(1000, 5000);
			gpio_set_value(priv->cb_reset_gpio, 1);
			usleep_range(1000, 5000);
		}

		if (ret < 0)
			priv->cb_reset_gpio = -EINVAL;
	}

	of_property_read_string(top_node, "sue,serial-config", &priv->serial_config);
	if (priv->serial_config) {
		dev_info(dev, "Found serial config %s \n", priv->serial_config);
	} else {
		dev_warn(dev, "No serial config\n");
	}

	ret = snd_soc_register_card(&priv->card);
	if (ret < 0) {
		dev_err(dev, "error registering card (%d)\n", ret);

		if (priv->regulator)
			regulator_disable(priv->regulator);

		return ret;
	}

	node = of_get_child_by_name(top_node, "control-defaults");
	if (node) {
		struct device_node *child;

		for_each_child_of_node(node, child) {
			const char *name, *value;

			of_property_read_string(child, "sue,control-name", &name);
			of_property_read_string(child, "sue,control-value", &value);

			snd_soc_imx_s810_set_control(priv->card.snd_card,
							name, value);
		}
	}

	priv->passive_mode_gpio = of_get_named_gpio(top_node, "sue,passive-mode-gpio", 0);
	if (gpio_is_valid(priv->passive_mode_gpio)) {
		ret = devm_gpio_request_one(dev, priv->passive_mode_gpio,
					    GPIOF_OUT_INIT_HIGH,
					    "Audio Passive Mode");

		if (ret == 0) {
			struct snd_kcontrol *kc =
				snd_ctl_new1(&imx_s810_passive_mode_control, priv);
			ret = snd_ctl_add(priv->card.snd_card, kc);
			if (ret < 0)
				dev_warn(dev, "Failed to add passive mode control: %d\n", ret);
		}

		if (ret < 0)
			priv->passive_mode_gpio = -EINVAL;
	}

	priv->amp_overheat_gpio = of_get_named_gpio(top_node, "sue,amp-overheat-gpio", 0);
	if (gpio_is_valid(priv->amp_overheat_gpio)) {
		ret = devm_gpio_request_one(dev, priv->amp_overheat_gpio,
					    GPIOF_IN, "Amplifier Overheat");

		if (ret == 0) {
			unsigned int irq_flags = IRQF_TRIGGER_RISING |
						 IRQF_TRIGGER_FALLING |
						 IRQF_ONESHOT;

			ret = devm_request_threaded_irq(
					dev, gpio_to_irq(priv->amp_overheat_gpio),
					NULL, imx_s810_amp_overheat_irq,
					irq_flags, "Amplifier Overheat", priv);
			if (ret < 0)
				dev_warn(dev, "Unable to request amp overheat IRQ: %d\n", ret);
		}

		if (ret == 0) {
			priv->amp_overheat_kctl =
				snd_ctl_new1(&imx_s810_amp_overheat_control, priv);

			ret = snd_ctl_add(priv->card.snd_card, priv->amp_overheat_kctl);
			if (ret < 0)
				dev_warn(dev, "Failed to add amp overheat control: %d\n", ret);
		}

		if (ret < 0)
			priv->amp_overheat_gpio = -EINVAL;
	}

	priv->amp_overcurrent_gpio = of_get_named_gpio(top_node, "sue,amp-overcurrent-gpio", 0);
	if (gpio_is_valid(priv->amp_overcurrent_gpio)) {
		ret = devm_gpio_request_one(dev, priv->amp_overcurrent_gpio,
					    GPIOF_IN, "Amplifier Over-current");

		if (ret == 0) {
			unsigned int irq_flags = IRQF_TRIGGER_RISING |
						 IRQF_TRIGGER_FALLING |
						 IRQF_ONESHOT;

			ret = request_threaded_irq(gpio_to_irq(priv->amp_overcurrent_gpio),
						   NULL, imx_s810_amp_overcurrent_irq,
						   irq_flags, "Amplifier Overcurrent", priv);
			if (ret < 0)
				dev_warn(dev, "Unable to request amp overcurrent IRQ: %d\n", ret);
		}
	}

	return 0;
}

static int snd_soc_imx_s810_remove(struct platform_device *pdev)
{
	struct snd_soc_imx_s810 *priv = platform_get_drvdata(pdev);

	snd_soc_unregister_card(&priv->card);

	if (priv->regulator)
		regulator_disable(priv->regulator);

	return 0;
}

static int snd_soc_imx_s810_suspend(struct device *dev)
{
        struct snd_soc_card *card = dev_get_drvdata(dev);
	struct snd_soc_imx_s810 *priv = snd_soc_card_get_drvdata(card);

	pinctrl_pm_select_sleep_state(dev);

	if (priv->regulator)
		regulator_disable(priv->regulator);

	return snd_soc_suspend(dev);
}

static void snd_soc_imx_s810_shutdown(struct platform_device *pdev)
{
	pinctrl_pm_select_sleep_state(&pdev->dev);
}

static int snd_soc_imx_s810_resume(struct device *dev)
{
	struct snd_soc_card *card = dev_get_drvdata(dev);
	struct snd_soc_imx_s810 *priv = snd_soc_card_get_drvdata(card);
	int ret;

	if (priv->regulator) {
		ret = regulator_enable(priv->regulator);
		if (ret < 0) {
			dev_err(dev, "unable to enable regulator: %d\n", ret);
			return ret;
		}
	}

	pinctrl_pm_select_default_state(dev);

	return snd_soc_resume(dev);
}

const struct dev_pm_ops snd_soc_imx_s810_pm_ops = {
	.suspend = snd_soc_imx_s810_suspend,
	.resume = snd_soc_imx_s810_resume,
	.freeze = snd_soc_suspend,
	.thaw = snd_soc_resume,
	.poweroff = snd_soc_poweroff,
	.restore = snd_soc_resume,
};

static struct platform_driver snd_soc_imx_s810_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= "snd-soc-imx-s810",
		.of_match_table	= snd_soc_imx_s810_match,
		.pm		= &snd_soc_imx_s810_pm_ops,
	},
	.probe		= snd_soc_imx_s810_probe,
	.remove		= snd_soc_imx_s810_remove,
	.shutdown	= snd_soc_imx_s810_shutdown,
};

module_platform_driver(snd_soc_imx_s810_driver);

MODULE_AUTHOR("Daniel Mack <daniel@zonque.org>");
MODULE_DESCRIPTION("Stream Unlimited S810 / Teufel ASoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:snd-soc-imx-s810");
