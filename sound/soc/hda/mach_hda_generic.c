/*
 *  mach_hda_generic.c: HD-A generic machine driver
 *
 *  Copyright (C) 2014 Intel Corp
 *  Author: Vinod Koul <vinod.koul@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <sound/soc.h>
#include <sound/soc-hda-bus.h>

struct snd_soc_dai_link mach_hda_generic_dailink[] = {
	{
		.name = "Speaker Playback",
		.stream_name = "Audio1",
		.cpu_dai_name = "System Pin",
		.platform_name = "0000:02:18.0",
		.codec_name = "codec#000.0",
		.codec_dai_name = "ALC286-AIF1",
	},
	{
		.name = "Headphone Playback",
		.stream_name = "Audio2",
		.cpu_dai_name = "System Pin",
		.platform_name = "0000:02:18.0",
		.codec_name = "codec#000.0",
		.codec_dai_name = "ALC286-AIF2",
	},
	{
		.name = "Capture",
		.stream_name = "Record1",
		.cpu_dai_name = "System Pin",
		.platform_name = "0000:02:18.0",
		.codec_name = "codec#000.0",
		.codec_dai_name = "ALC286-AIF3",
	},
#if 0
	{
		.name = "HDMI Playback 1",
		.stream_name = "HDMI PB1",
		.cpu_dai_name = "System Pin",
		.platform_name = "0000:02:18.0",
		.codec_name = "codec#003.3",
		.codec_dai_name = "intel-hdmi-hif1",
	},
	{
		.name = "HDMI Playback 2",
		.stream_name = "HDMI PB2",
		.cpu_dai_name = "System Pin",
		.platform_name = "0000:02:18.0",
		.codec_name = "codec#003.3",
		.codec_dai_name = "intel-hdmi-hif2",
	},
#endif
};

/* SoC card */
static struct snd_soc_card snd_soc_card_hda = {
	.name = "mach_hda_generic",
	.dai_link = mach_hda_generic_dailink,
	.num_links = ARRAY_SIZE(mach_hda_generic_dailink),
};

static int mach_generic_probe(struct snd_soc_hda_device *hda)
{
	int ret_val;

	/* register the soc card */
	snd_soc_card_hda.dev = &hda->dev;
	ret_val = snd_soc_register_card(&snd_soc_card_hda);
	if (ret_val)
		dev_err(&hda->dev, "snd_soc_register_card failed %d\n", ret_val);

	return ret_val;
}

static int mach_generic_remove(struct snd_soc_hda_device *hda)
{
	return snd_soc_unregister_card(&snd_soc_card_hda);
}

static const struct snd_soc_hda_device_id mach_generic_ids[] = {
{ .id = -1, .addr = 0xFF, .name = "mach_hda_generic", NULL},
{},
};

static struct snd_soc_hda_driver mach_generic = {
	.driver	= {
		.name	= "HDAudio",
		.owner	= THIS_MODULE,
	},
	.id_table = mach_generic_ids,
	.probe = mach_generic_probe,
	.remove = mach_generic_remove,
};

static int __init hda_mach_generic_init(void)
{
	return snd_soc_hda_driver_register(&mach_generic);
}

module_init(hda_mach_generic_init);

static void __exit hda_mach_generic_exit(void)
{
	snd_soc_hda_driver_unregister(&mach_generic);
}
module_exit(hda_mach_generic_exit);

MODULE_DESCRIPTION("ASoC HDA generic Machine driver");
MODULE_AUTHOR("Vinod Koul <vinod.koul@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("snd-mach_hda_generic");
