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

#define DATA_WORD_WIDTH 32

#define MCLK_48k	24576000
#define MCLK_44k1	22579200

#define IMX7D_SAI_PLL_48k	884736000UL
#define IMX7D_SAI_PLL_44k1	812851200UL

struct snd_soc_am33xx_s800 {
	struct snd_soc_card	card;
	struct clk 		*mclk;
	struct clk		*mclk_rx;
	unsigned int		mclk_rate;
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
static int am33xx_s800_apply_drift(struct snd_soc_card *card)
{
	int ret;
	s32 sgn, comp, drift;
	u32 clk_rate;
	struct snd_soc_am33xx_s800 *priv = snd_soc_card_get_drvdata(card);

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

static int am33xx_s800_setup_mcasp(struct snd_pcm_substream *substream, snd_pcm_format_t format)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *card = codec_dai->component->card;
	struct snd_soc_am33xx_s800 *priv = snd_soc_card_get_drvdata(card);
	const char *serial_config = priv->serial_config;

	int n_i2s = 0;
	int n_dsd = 0;
	int n_spdif = 0;
	int i;
	int tx_slots[4];
	int nch = 0;
	int ret;
	bool is_dsd = (format == SNDRV_PCM_FORMAT_DSD_U8);

	if (!serial_config) {
		dev_warn(card->dev, "Serial configuration is empty, skipping reconfiguration\n");
		return 0;
	}

	for (i = 0; i < 4; i++) {
		switch (serial_config[i]) {
			case 'I':
				if (is_dsd) continue;
				n_i2s++;
				break;
			case 'D':
				if (!is_dsd) continue;
				n_dsd++;
				break;
			case 'M':
				n_i2s++;
				n_dsd++;
				break;
			case 'S':
				n_spdif++;
				break;
			case '-':
				continue;
				break;
			default:
				dev_warn(card->dev, "Invalid character '%c' in serial config\n", serial_config[i]);
				return -EINVAL;
		}
		tx_slots[nch++] = i;
	}

	if (n_spdif > 0 && (n_i2s + n_dsd) != 0) {
		dev_warn(card->dev, "SPDIF is not compatiable with other formats\n");
		return -EINVAL;
	}

	if (n_dsd == 0 && is_dsd) {
		dev_warn(card->dev, "No pins defined for DSD, yet DSD is detected!\n");
		return -EINVAL;
	}

	if (is_dsd) {
		ret = pinctrl_select_state(priv->pinctrl, priv->pinctrl_state_dsd);
		if (ret < 0)
			dev_warn(card->dev, "could not select dsd pins\n");
	} else {
		ret = pinctrl_select_state(priv->pinctrl, priv->pinctrl_state_pcm);
		if (ret < 0)
			dev_warn(card->dev, "could not select pcm pins\n");
	}

	ret = snd_soc_dai_set_channel_map(cpu_dai, nch, tx_slots, 0, NULL);
	if (ret < 0) {
		dev_warn(card->dev, "Failed to reconfigure channel map\n");
		return ret;
	}

	return 0;
}

static int am33xx_s800_set_mclk(struct snd_soc_am33xx_s800 *priv, int stream)
{
	struct clk *mclk;
	unsigned long mclk_rate;
	int ret;

	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		mclk = priv->mclk;
		mclk_rate = priv->mclk_rate;

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

static int snd_soc_am33xx_s800_set_control(struct snd_card *card,
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

static int am33xx_s800_common_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *params,
					bool is_spdif)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_dai *cpu_dai = rtd->cpu_dai;
	struct snd_soc_card *card = codec_dai->component->card;
	struct snd_soc_am33xx_s800 *priv = snd_soc_card_get_drvdata(card);
	unsigned int mclk, rate, bclk;
#ifdef CONFIG_SND_SOC_STREAM_AM33XX
	unsigned int bclk_div = is_spdif ? 4 : 2;
#endif /* CONFIG_SND_SOC_STREAM_AM33XX */
	int ret;
	int clk_id, div_mclk, div_bclk, div_lrclk;

	rate = params_rate(params);
	mclk = (rate % 16000 == 0) ? MCLK_48k : MCLK_44k1;

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		priv->mclk_rate = mclk;

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


		ret = am33xx_s800_set_mclk(priv, substream->stream);
		if (ret < 0) {
			dev_warn(card->dev, "Unsupported MCLK source : %d\n", ret);
			return ret;
		}

		if (!IS_ERR(priv->pllclk)) {
			priv->drift = 0;
			am33xx_s800_apply_drift(card);
		}
	}

	/* Reconfigure McASP serializers */
	ret = am33xx_s800_setup_mcasp(substream, params_format(params));
	if (ret < 0) {
		dev_warn(card->dev, "Unsupported mcasp serial config : %d\n", ret);
		return ret;
	}

	/* CPU MLCK */
	ret = snd_soc_dai_set_sysclk(cpu_dai, clk_id, mclk, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_warn(card->dev, "Unsupported cpu dai MCLK : %d\n", ret);
		return ret;
	}

	/* Codec MCLK */
	ret = snd_soc_dai_set_sysclk(codec_dai, 0, mclk, SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_warn(card->dev, "Unsupported codec dai MLCK : %d\n", ret);
		/* intentionally ignore errors - the codec driver may not care, at least give a warning */
	}

	if (params_format(params) == SNDRV_PCM_FORMAT_DSD_U8) {
		bclk = rate * 8;
	} else {
		bclk = rate * 2 * DATA_WORD_WIDTH;
	}


#ifdef CONFIG_SND_SOC_STREAM_AM33XX
	/* CPU MCLK divider */
	ret = snd_soc_dai_set_clkdiv(cpu_dai, div_mclk, 1);
	if (ret < 0) {
		dev_warn(card->dev, "Unsupported cpu dai MCLK divider : %d\n", ret);
		return ret;
	}

	/* CPU BCLK-to-LRCLK divider */
	if (params_format(params) == SNDRV_PCM_FORMAT_DSD_U8) {
		/* Clock rate for DSD matches bitrate */
		ret = snd_soc_dai_set_clkdiv(cpu_dai, div_lrclk, 0);
	} else {
		ret = snd_soc_dai_set_clkdiv(cpu_dai, div_lrclk, 2 * DATA_WORD_WIDTH);
	}

	if (ret < 0) {
		dev_warn(card->dev, "Unsupported cpu dai BCLK/LRCLK divider : %d\n", ret);
 		return ret;
	}

	/* CPU BCLK divider */
	if (params_format(params) == SNDRV_PCM_FORMAT_DSD_U8) {
		ret = snd_soc_dai_set_clkdiv(cpu_dai, div_bclk, mclk / bclk);
	} else {
		ret = snd_soc_dai_set_clkdiv(cpu_dai, div_bclk, mclk / (rate * bclk_div * DATA_WORD_WIDTH));
	}
	if (ret < 0) {
		dev_warn(card->dev, "Unsupported cpu dai BCLK divider : %d\n", ret);
		return ret;
	}

#endif /* CONFIG_SND_SOC_STREAM_AM33XX */
	dev_info(card->dev, "Configured common HW params, RATE %d, MCLK %d, BCLK %d", rate, mclk, bclk);

	return 0;
}

static int am33xx_s800_i2s_hw_params(struct snd_pcm_substream *substream,
				     struct snd_pcm_hw_params *params)
{
	return am33xx_s800_common_hw_params(substream, params, false);
}

static int am33xx_s800_common_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_soc_card *card = codec_dai->component->card;
	struct snd_soc_am33xx_s800 *priv = snd_soc_card_get_drvdata(card);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		priv->mclk_rate = 0;
	} else {
		priv->mclk_rate_rx = 0;
	}

	return 0;
}

static struct snd_soc_ops am33xx_s800_i2s_dai_link_ops = {
	.hw_params	= am33xx_s800_i2s_hw_params,
	.hw_free	= am33xx_s800_common_hw_free,
};

static int am33xx_s800_spdif_hw_params(struct snd_pcm_substream *substream,
				       struct snd_pcm_hw_params *params)
{
	return am33xx_s800_common_hw_params(substream, params, true);
}

static struct snd_soc_ops am33xx_s800_spdif_dai_link_ops = {
	.hw_params	= am33xx_s800_spdif_hw_params,
	.hw_free	= am33xx_s800_common_hw_free,
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
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct snd_soc_am33xx_s800 *priv = snd_soc_card_get_drvdata(card);

	ucontrol->value.integer.value[0] = priv->drift;

	return 0;
}

static int am33xx_s800_drift_put(struct snd_kcontrol *kcontrol,
				     struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct snd_soc_am33xx_s800 *priv = snd_soc_card_get_drvdata(card);

	if (ucontrol->value.integer.value[0] == priv->drift)
		return 0;

	priv->drift = ucontrol->value.integer.value[0];

	am33xx_s800_apply_drift(card);

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

static int am33xx_s800_passive_mode_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct snd_soc_am33xx_s800 *priv = snd_soc_card_get_drvdata(card);

	ucontrol->value.integer.value[0] =
		!gpio_get_value_cansleep(priv->passive_mode_gpio);
	return 0;
}

static int am33xx_s800_passive_mode_put(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct snd_soc_am33xx_s800 *priv = snd_soc_card_get_drvdata(card);

	gpio_set_value(priv->passive_mode_gpio,
		       !ucontrol->value.integer.value[0]);
        return 1;
}

static const struct snd_kcontrol_new am33xx_s800_passive_mode_control =
	SOC_SINGLE_BOOL_EXT("Passive mode", 0,
			    am33xx_s800_passive_mode_get,
			    am33xx_s800_passive_mode_put);

static int am33xx_s800_amp_overheat_get(struct snd_kcontrol *kcontrol,
					struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
	struct snd_soc_am33xx_s800 *priv = snd_soc_card_get_drvdata(card);

	ucontrol->value.integer.value[0] =
		!gpio_get_value_cansleep(priv->amp_overheat_gpio);

	return 0;
}

static const struct snd_kcontrol_new am33xx_s800_amp_overheat_control =
	SOC_SINGLE_BOOL_EXT("Amplifier Overheat Sensor", 0,
			    am33xx_s800_amp_overheat_get,
			    NULL);

static irqreturn_t am33xx_s800_amp_overheat_irq(int irq, void *data)
{
	struct snd_soc_am33xx_s800 *priv = data;

	snd_ctl_notify(priv->card.snd_card, SNDRV_CTL_EVENT_MASK_VALUE,
		       &priv->amp_overheat_kctl->id);

	return IRQ_HANDLED;
}

static irqreturn_t am33xx_s800_amp_overcurrent_irq(int irq, void *data)
{
	struct snd_soc_am33xx_s800 *priv = data;

	dev_warn(priv->card.dev, "Amplifier signaled overcurrent/shutdown condition");

	return IRQ_HANDLED;
}

static const struct of_device_id snd_soc_am33xx_s800_match[] = {
	{ .compatible	= "sue,am33xx-generic-audio" },
	{ }
};

static int snd_soc_am33xx_s800_probe(struct platform_device *pdev)
{
	int ret;
	unsigned int dai_fmt;
	struct device *dev = &pdev->dev;
	struct device_node *top_node, *node;
	struct snd_soc_am33xx_s800 *priv;
	struct snd_soc_dai_link *link;
        const struct of_device_id *of_id =
                        of_match_device(snd_soc_am33xx_s800_match, dev);

	top_node = dev->of_node;

	if (!of_id)
		return -ENODEV;

	priv = devm_kzalloc(dev, sizeof(struct snd_soc_am33xx_s800),
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

	priv->mclk_rx = devm_clk_get(&pdev->dev, "mclk_rx");
	if (IS_ERR(priv->mclk_rx)) {
		dev_err(dev, "failed to get MCLK RX\n");
		return -EPROBE_DEFER;
	}
#ifdef CONFIG_SND_SOC_STREAM_AM33XX
	/* request pin mux */
	priv->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(priv->pinctrl))
		dev_warn(dev, "pins are not configured from the driver\n");

	priv->pinctrl_state_pcm = pinctrl_lookup_state(priv->pinctrl, "pcm");
	if (IS_ERR(priv->pinctrl_state_pcm)) {
		dev_warn(dev, "pcm pin lookup failed, retrying with default\n");

		priv->pinctrl_state_pcm = pinctrl_lookup_state(priv->pinctrl, PINCTRL_STATE_DEFAULT);
		if (IS_ERR(priv->pinctrl_state_pcm))
			dev_warn(dev, "default pins also not found, check your devicetree\n");
	}

	priv->pinctrl_state_dsd = pinctrl_lookup_state(priv->pinctrl, "dsd");
	if (IS_ERR(priv->pinctrl_state_dsd))
		dev_warn(dev, "dsd pin lookup failed\n");

	/* Enable pcm pins as default */
	ret = pinctrl_select_state(priv->pinctrl, priv->pinctrl_state_pcm);
	if (ret < 0)
		dev_warn(dev, "could not select pcm pins\n");
#endif /* CONFIG_SND_SOC_STREAM_AM33XX */

	priv->regulator = devm_regulator_get(dev, "vd");
	if (IS_ERR(priv->regulator)) {
		dev_err(dev, "failed to get regulator\n");
		return -EPROBE_DEFER;
	}

	/* this is a hack to temporarily disable the MCLK in test mode */
	if (of_get_property(top_node, "sue,disable-clk", NULL)) {
		clk_prepare_enable(priv->mclk);
		clk_disable_unprepare(priv->mclk);
		return 0;
	}

	/* machine controls */
	priv->card.controls = am33xx_s800_controls;
	priv->card.num_controls = ARRAY_SIZE(am33xx_s800_controls);

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
				link->ops = &am33xx_s800_spdif_dai_link_ops;
			else
				link->ops = &am33xx_s800_i2s_dai_link_ops;

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
	// TODO: Maybe disable MCLK again if snd_soc_register_card() fails?
	if (of_get_property(top_node, "sue,early-mclk", NULL)) {
		u32 pllrate = IMX7D_SAI_PLL_48k;
		dev_info(dev, "enabling early MCLK\n");

		priv->mclk_rate = MCLK_48k;

		clk_set_rate(priv->pllclk, pllrate);
		am33xx_s800_set_mclk(priv, SNDRV_PCM_STREAM_PLAYBACK);
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

			snd_soc_am33xx_s800_set_control(priv->card.snd_card,
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
				snd_ctl_new1(&am33xx_s800_passive_mode_control, priv);
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
					NULL, am33xx_s800_amp_overheat_irq,
					irq_flags, "Amplifier Overheat", priv);
			if (ret < 0)
				dev_warn(dev, "Unable to request amp overheat IRQ: %d\n", ret);
		}

		if (ret == 0) {
			priv->amp_overheat_kctl =
				snd_ctl_new1(&am33xx_s800_amp_overheat_control, priv);

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
						   NULL, am33xx_s800_amp_overcurrent_irq,
						   irq_flags, "Amplifier Overcurrent", priv);
			if (ret < 0)
				dev_warn(dev, "Unable to request amp overcurrent IRQ: %d\n", ret);
		}
	}

	return 0;
}

static int snd_soc_am33xx_s800_remove(struct platform_device *pdev)
{
	struct snd_soc_am33xx_s800 *priv = platform_get_drvdata(pdev);

	snd_soc_unregister_card(&priv->card);
	regulator_disable(priv->regulator);

	return 0;
}

static int snd_soc_am33xx_s800_suspend(struct device *dev)
{
        struct snd_soc_card *card = dev_get_drvdata(dev);
	struct snd_soc_am33xx_s800 *priv = snd_soc_card_get_drvdata(card);

	pinctrl_pm_select_sleep_state(dev);
	regulator_disable(priv->regulator);

	return snd_soc_suspend(dev);
}

static void snd_soc_am33xx_s800_shutdown(struct platform_device *pdev)
{
	pinctrl_pm_select_sleep_state(&pdev->dev);
}

static int snd_soc_am33xx_s800_resume(struct device *dev)
{
	struct snd_soc_card *card = dev_get_drvdata(dev);
	struct snd_soc_am33xx_s800 *priv = snd_soc_card_get_drvdata(card);
	int ret;

	ret = regulator_enable(priv->regulator);
	if (ret < 0) {
		dev_err(dev, "unable to enable regulator: %d\n", ret);
		return ret;
	}

	pinctrl_pm_select_default_state(dev);

	return snd_soc_resume(dev);
}

const struct dev_pm_ops snd_soc_am33xx_s800_pm_ops = {
	.suspend = snd_soc_am33xx_s800_suspend,
	.resume = snd_soc_am33xx_s800_resume,
	.freeze = snd_soc_suspend,
	.thaw = snd_soc_resume,
	.poweroff = snd_soc_poweroff,
	.restore = snd_soc_resume,
};

static struct platform_driver snd_soc_am33xx_s800_driver = {
	.driver = {
		.owner		= THIS_MODULE,
		.name		= "snd-soc-am33xx-s800",
		.of_match_table	= snd_soc_am33xx_s800_match,
		.pm		= &snd_soc_am33xx_s800_pm_ops,
	},
	.probe		= snd_soc_am33xx_s800_probe,
	.remove		= snd_soc_am33xx_s800_remove,
	.shutdown	= snd_soc_am33xx_s800_shutdown,
};

module_platform_driver(snd_soc_am33xx_s800_driver);

MODULE_AUTHOR("Daniel Mack <daniel@zonque.org>");
MODULE_DESCRIPTION("Stream Unlimited S800 / Raumfeld ASoC Interface");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:snd-soc-am33xx-s800");
