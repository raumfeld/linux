/*
 * pcm_dummy.c  --  dummy PCM ALSA SoC Audio driver
 *
 * Copyright 2014 Streamunlimited
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/spi/spi.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#define DRIVER_NAME "pcm-dummy"

struct pcm_dummy_private {
	struct device  *dev;	/* attached platform device */

	unsigned gpio_nmute;
	unsigned gpio_nreset;
};

static int pcm_dummy_set_dai_fmt(struct snd_soc_dai *codec_dai, unsigned int format)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct pcm_dummy_private *ctx = snd_soc_codec_get_drvdata(codec);

	dev_dbg(ctx->dev, "pcm_dummy_set_dai_fmt() called, format = %d\n", format);
	return 0;
}

static int pcm_dummy_digital_mute(struct snd_soc_dai *codec_dai, int mute)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct pcm_dummy_private *ctx = snd_soc_codec_get_drvdata(codec);

	dev_dbg(ctx->dev, "pcm_dummy_digital_mute() called: %s\n", mute ? "muted" : "unmuted");
	return 0;
}

static int pcm_dummy_set_dai_sysclk(struct snd_soc_dai *codec_dai, int clk_id, unsigned int freq, int dir)
{
	struct snd_soc_codec *codec = codec_dai->codec;
	struct pcm_dummy_private *ctx = snd_soc_codec_get_drvdata(codec);

	dev_dbg(ctx->dev, "pcm_dummy_set_dai_sysclk() called: clk_id = %d, freq = %d, dir = %d\n", clk_id, freq, dir);
	return 0;
}

static const struct snd_soc_dai_ops pcm_dummy_dai_ops = {
	.set_fmt	= pcm_dummy_set_dai_fmt,
	.set_sysclk	= pcm_dummy_set_dai_sysclk,
	.digital_mute	= pcm_dummy_digital_mute,
};

#define PCM_DUMMY_RATES (\
		SNDRV_PCM_RATE_5512  | SNDRV_PCM_RATE_8000 |\
		SNDRV_PCM_RATE_11025 | SNDRV_PCM_RATE_16000 |\
		SNDRV_PCM_RATE_22050 | SNDRV_PCM_RATE_32000 |\
		SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |\
		SNDRV_PCM_RATE_64000 | SNDRV_PCM_RATE_88200 |\
		SNDRV_PCM_RATE_96000 | SNDRV_PCM_RATE_176400 |\
		SNDRV_PCM_RATE_192000 | SNDRV_PCM_RATE_352800 |\
		SNDRV_PCM_RATE_384000 | SNDRV_PCM_RATE_705600 |\
		SNDRV_PCM_RATE_768000 | SNDRV_PCM_RATE_1411200 |\
		SNDRV_PCM_RATE_1536000 | SNDRV_PCM_RATE_2822400 |\
		SNDRV_PCM_RATE_3072000)

#define PCM_DUMMY_FORMATS (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S24_3LE | SNDRV_PCM_FMTBIT_S32_LE | SNDRV_PCM_FMTBIT_DSD_U8)

struct snd_soc_dai_driver pcm_dummy_dai = {
	.name = "Dummy PCM Codec",
	.playback = {
		.stream_name	= "Playback",
		.channels_min	= 2,
		.channels_max	= 2,
		.rates		= SNDRV_PCM_RATE_CONTINUOUS,
		.rate_min	= 5512,
		.rate_max	= 3072000,
		.formats	= PCM_DUMMY_FORMATS,
	},
	.capture = {
		.stream_name	= "Capture",
		.channels_min	= 2,
		.channels_max	= 2,
		.rates		= SNDRV_PCM_RATE_CONTINUOUS,
		.rate_min	= 5512,
		.rate_max	= 3072000,
		.formats	= PCM_DUMMY_FORMATS,
	},
	.ops = &pcm_dummy_dai_ops,
	.symmetric_rates = 1,
};

#ifdef CONFIG_OF
static const struct of_device_id pcm_dummy_dt_ids[] = {
	{ .compatible = "sue,pcm_dummy", },
	{ }
};
MODULE_DEVICE_TABLE(of, pcm_dummy_dt_ids);
#endif

static int pcm_dummy_soc_probe(struct snd_soc_codec *codec)
{
	struct pcm_dummy_private *ctx = snd_soc_codec_get_drvdata(codec);

	dev_dbg(ctx->dev, "pcm_dummy_soc_probe() called\n");
	return 0;
}

static int pcm_dummy_soc_remove(struct snd_soc_codec *codec)
{
	struct pcm_dummy_private *ctx = snd_soc_codec_get_drvdata(codec);

	dev_dbg(ctx->dev, "pcm_dummy_soc_remove() called\n");
	return 0;
}

int pcm_dummy_soc_suspend(struct snd_soc_codec *codec)
{
	struct pcm_dummy_private *ctx = snd_soc_codec_get_drvdata(codec);

	dev_dbg(ctx->dev, "pcm_dummy_soc_suspend() called\n");
	return 0;
}

int pcm_dummy_soc_resume(struct snd_soc_codec *codec)
{
	struct pcm_dummy_private *ctx = snd_soc_codec_get_drvdata(codec);

	dev_dbg(ctx->dev, "pcm_dummy_soc_resume() called\n");
	return 0;
}

static struct snd_soc_codec_driver soc_codec_dev_pcm_dummy = {
	.probe			= pcm_dummy_soc_probe,
	.remove			= pcm_dummy_soc_remove,
	.suspend		= pcm_dummy_soc_suspend,
	.resume			= pcm_dummy_soc_resume,
	.reg_cache_default	= NULL,
	.reg_cache_size		= 0,
	.reg_word_size		= 0,
};

static int pcm_dummy_probe(struct platform_device *pDev)
{
	struct device *dev = &pDev->dev;
	struct pcm_dummy_private *pdata = NULL;
	const struct of_device_id *match = NULL;
	int ret = 0;

	dev_dbg(dev, "pcm_dummy_probe(): starting\n");

	match = of_match_device(pcm_dummy_dt_ids, dev);

	if (match == NULL) {
		dev_err(dev, "%s(%d): failed!\n", __func__, __LINE__);
		return -EINVAL;
	}

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	pdata->dev = dev;

	platform_set_drvdata(pDev, pdata);

	#ifdef CONFIG_OF
	pdata->gpio_nreset = of_get_named_gpio(dev->of_node, "reset-gpio", 0);
	pdata->gpio_nmute  = of_get_named_gpio(dev->of_node, "mute-gpio", 0);
	#endif


	/* TODO: release GPIOs on module remove */
	if (gpio_is_valid(pdata->gpio_nreset)) {
		ret = gpio_request_one(pdata->gpio_nreset, GPIOF_OUT_INIT_HIGH, "Dummy codec reset GPIO");

		if (ret < 0)
			dev_warn(dev, "failed to request reset gpio: %d\n", ret);
		else {
			/* Reset codec */
			gpio_direction_output(pdata->gpio_nreset, 1);
			udelay(1000);
			gpio_set_value(pdata->gpio_nreset, 0);

			/* Give the codec time to wake up */
			udelay(1000);
		}
	}

	/* Enable the amplifier */
	if (gpio_is_valid(pdata->gpio_nmute)) {
		ret = gpio_request_one(pdata->gpio_nmute, GPIOF_OUT_INIT_HIGH, "Dummy codec amplifier mute GPIO");

		if (ret < 0)
			dev_warn(dev, "failed to request mute gpio: %d\n", ret);
		else
			gpio_set_value(pdata->gpio_nmute, 0);
	}

	ret = snd_soc_register_codec(dev, &soc_codec_dev_pcm_dummy, &pcm_dummy_dai, 1);

	if (ret < 0) {
		dev_err(dev, "pcm_dummy_probe(): failed with error %d\n", ret);
		return ret;
	}

	dev_dbg(dev, "pcm_dummy_probe(): successfully finished\n");
	return 0;
}

static int pcm_dummy_remove(struct platform_device *pDev)
{
	struct pcm_dummy_private *pData = platform_get_drvdata(pDev);

	dev_dbg(&pDev->dev, "pcm_dummy_remove(): starting\n");

	kfree(pData);
	platform_set_drvdata(pDev, NULL);

	dev_dbg(&pDev->dev, "pcm_dummy_remove(): successfully finished\n");
	return 0;
}

static struct platform_driver pcm_dummy_platform_driver = {
	.probe	= pcm_dummy_probe,
	.remove	= pcm_dummy_remove,
	.driver	= {
		.name = DRIVER_NAME,
		.owner = THIS_MODULE,
		.of_match_table = of_match_ptr(pcm_dummy_dt_ids),
	},
};

module_platform_driver(pcm_dummy_platform_driver);

MODULE_DESCRIPTION("ASoC PCM dummy driver");
MODULE_AUTHOR("Lee Page");
MODULE_LICENSE("GPL");
