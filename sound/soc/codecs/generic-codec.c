/*
 * Generic I2S codec driver
 *
 * Copyright (C) 2024 comma.ai
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <sound/soc.h>
#include <sound/pcm.h>

static struct snd_soc_dai_driver generic_dai = {
	.name = "HiFi",
	.playback = {
		.stream_name  = "HiFi Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates        = SNDRV_PCM_RATE_48000,
		.formats      = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name  = "HiFi Capture",
		.channels_min = 2,
		.channels_max = 2,
		.rates        = SNDRV_PCM_RATE_48000,
		.formats      = SNDRV_PCM_FMTBIT_S16_LE,
	},
    .ops = &(const struct snd_soc_dai_ops){},
};

static const struct snd_soc_dapm_widget generic_dapm_widgets[] = {
	SND_SOC_DAPM_INPUT("IN_L"),
	SND_SOC_DAPM_INPUT("IN_R"),

	SND_SOC_DAPM_OUTPUT("OUT_L"),
	SND_SOC_DAPM_OUTPUT("OUT_R"),
};

static const struct snd_soc_dapm_route generic_dapm_routes[] = {
	{ "OUT_L", NULL, "HiFi Playback" },
	{ "OUT_R", NULL, "HiFi Playback" },

	{ "HiFi Capture", NULL, "IN_L" },
	{ "HiFi Capture", NULL, "IN_R" },
};

static const struct snd_soc_component_driver generic_component = {
	.dapm_widgets = generic_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(generic_dapm_widgets),
	.dapm_routes = generic_dapm_routes,
	.num_dapm_routes = ARRAY_SIZE(generic_dapm_routes),
};

static int generic_codec_probe(struct platform_device *pdev)
{
	int ret;

	ret = devm_snd_soc_register_component(&pdev->dev, &generic_component,
					      &generic_dai, 1);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register generic codec: %d\n",
			ret);
		return ret;
	}

	dev_info(&pdev->dev, "Registered generic codec\n");
	return 0;
}

static const struct of_device_id generic_codec_of_match[] = {
	{ .compatible = "commaai,generic-codec" },
	{}
};
MODULE_DEVICE_TABLE(of, generic_codec_of_match);

static struct platform_driver generic_codec_driver = {
	.probe = generic_codec_probe,
	.driver = {
		.name           = "generic-codec",
		.of_match_table = generic_codec_of_match,
	},
};
module_platform_driver(generic_codec_driver);

MODULE_DESCRIPTION("Generic codec driver");
MODULE_AUTHOR("Robbe Derks <robbe@comma.ai>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:generic-codec");