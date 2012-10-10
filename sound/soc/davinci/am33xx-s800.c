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

struct snd_soc_am33xx_s800 {
	struct snd_soc_card	card;
	struct clk 		*mclk;
	unsigned int		mclk_rate;
	signed int		drift;
	int			passive_mode_gpio;
	int			amp_overheat_gpio;
	int			amp_overcurrent_gpio;
	int			amp_reset_gpio;
	int			amp_reset_delay_ms;
	struct snd_kcontrol	*amp_overheat_kctl;
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

static const struct snd_soc_dapm_route ams_delta_audio_map[] = {
	{ "PWM1", "SDIN1 Left Mux", "SDIN1 right" },
};

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
			dev_warn(card->dev,
				 "Unable to write control '%s'\n",
				 name);
			goto exit_free_info;
		}

		dev_warn(card->dev, "Control default '%s' -> '%s'\n",
			 name, value);

		goto exit;
	}

	dev_warn(card->dev, "Enum '%s' has no entry '%s'\n", name, value);

exit_free_info:
	kfree(info);
exit:
	return ret;
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
        struct snd_soc_card *card = snd_kcontrol_chip(kcontrol);
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
	struct pinctrl *pinctrl;
        const struct of_device_id *of_id =
                        of_match_device(snd_soc_am33xx_s800_match, dev);

	top_node = dev->of_node;

	if (!of_id)
		return -ENODEV;

	priv = devm_kzalloc(dev, sizeof(struct snd_soc_am33xx_s800),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_CBS_CFS;

	if (of_get_property(top_node, "sue,invert-wclk", NULL))
		dai_fmt |= SND_SOC_DAIFMT_NB_IF;
	else
		dai_fmt |= SND_SOC_DAIFMT_NB_NF;

	/* request pin mux */
	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl))
		dev_warn(dev, "pins are not configured from the driver\n");

	priv->mclk = of_clk_get(top_node, 0);
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
	priv->card.controls = am33xx_s800_controls;
	priv->card.num_controls = ARRAY_SIZE(am33xx_s800_controls);

	priv->card.dev = dev;
	snd_soc_of_parse_card_name(&priv->card, "sue,card-name");

	node = of_get_child_by_name(top_node, "links");
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
	}

	platform_set_drvdata(pdev, &priv->card);
	snd_soc_card_set_drvdata(&priv->card, priv);

	ret = snd_soc_register_card(&priv->card);
	if (ret < 0) {
		dev_err(dev, "error registering card (%d)\n", ret);
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

			ret = request_threaded_irq(gpio_to_irq(priv->amp_overheat_gpio),
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

	priv->amp_reset_gpio = of_get_named_gpio(top_node, "sue,amp-reset-gpio", 0);
	if (gpio_is_valid(priv->amp_reset_gpio)) {
		ret = devm_gpio_request_one(dev, priv->amp_reset_gpio,
					    GPIOF_OUT_INIT_HIGH,
					    "Audio Amplifier Reset");
		if (ret < 0)
			priv->amp_reset_gpio = -EINVAL;

		of_property_read_u32(top_node, "sue,amp-reset-delay-ms",
				     &priv->amp_reset_delay_ms);
	}

	return 0;
}

static void snd_soc_am33xx_s800_shutdown_amp(struct device *dev,
					     struct snd_soc_am33xx_s800 *priv)
{
	pinctrl_pm_select_sleep_state(dev);

	if (gpio_is_valid(priv->amp_reset_gpio)) {
		gpio_set_value(priv->amp_reset_gpio, 0);
		msleep(priv->amp_reset_delay_ms);
	}
}

static int snd_soc_am33xx_s800_remove(struct platform_device *pdev)
{
	struct snd_soc_am33xx_s800 *priv = platform_get_drvdata(pdev);

	snd_soc_am33xx_s800_shutdown_amp(&pdev->dev, priv);
	snd_soc_unregister_card(&priv->card);

	return 0;
}

static int snd_soc_am33xx_s800_suspend(struct device *dev)
{
        struct snd_soc_card *card = dev_get_drvdata(dev);
	struct snd_soc_am33xx_s800 *priv = snd_soc_card_get_drvdata(card);

	snd_soc_am33xx_s800_shutdown_amp(dev, priv);

	return snd_soc_suspend(dev);
}

static void snd_soc_am33xx_s800_shutdown(struct platform_device *pdev)
{
	struct snd_soc_am33xx_s800 *priv = platform_get_drvdata(pdev);

	snd_soc_am33xx_s800_shutdown_amp(&pdev->dev, priv);
}

static int snd_soc_am33xx_s800_resume(struct device *dev)
{
	struct snd_soc_card *card = dev_get_drvdata(dev);
	struct snd_soc_am33xx_s800 *priv = snd_soc_card_get_drvdata(card);

	if (gpio_is_valid(priv->amp_reset_gpio))
		gpio_set_value(priv->amp_reset_gpio, 1);

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
