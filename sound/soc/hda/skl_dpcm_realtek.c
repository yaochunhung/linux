/*
 *  skl_dpcm_realtek.c - ASOC Machine driver for SKL platform
 *
 *  Copyright (C) 2014 Intel Corp
 *  Author: Samreen Nilofer <samreen.nilofer@intel.com>
 *
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
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <sound/soc.h>

static const struct snd_soc_dapm_route skl_map[] = {
	/* SWM map link the SWM outs to codec AIF */
	{ "Speaker", NULL, "HD-Codec Tx"},
	{ "Headphone", NULL, "HD-Codec Tx"},
	{ "HD-Codec Tx", NULL, "codec1_out"},
	{ "HD-Codec Tx", NULL, "codec0_out"},

	{ "codec0_in", NULL, "HD-Codec Rx"},
	{ "codec1_in", NULL, "HD-Codec Rx"},
	{ "HD-Codec Rx", NULL, "Record"},

	{ "dmic01_hifi", NULL, "DMIC01 Rx"},
	{ "dmic23_hifi", NULL, "DMIC23 Rx"},
	{ "DMIC01 Rx", NULL, "Dummy Capture"},
	{ "DMIC23 Rx", NULL, "Dummy Capture"}

	/*{ "hif1", NULL, "iDisp Tx"},
	{ "iDisp Tx", NULL, "iDisp_out"},*/

};

struct snd_soc_dai_link skl_audio_msic_dailink[] = {
	{
		.name = "Skl Audio HS Port",
		.stream_name = "Audio Headset",
		.cpu_dai_name = "System Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:1f.3",
		.ignore_suspend = 1,
		.dynamic = 1,
	},
	{
		.name = "Skl Audio SPK Port",
		.stream_name = "Audio speaker",
		.cpu_dai_name = "System Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:1f.3",
		.ignore_suspend = 1,
		.dynamic = 1,
	},
	{
		.name = "Skl Audio Record Port",
		.stream_name = "Audio Record",
		.cpu_dai_name = "System Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:1f.3",
		.ignore_suspend = 1,
		.dynamic = 1,
	},
	{
		.name = "Skl DB Audio Port",
		.stream_name = "Deep Buffer Audio",
		.cpu_dai_name = "Deepbuffer Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:1f.3",
		.ignore_suspend = 1,
		.dynamic = 1,
	},
	{
		.name = "Skl LL Audio Port",
		.stream_name = "Low Latency Audio",
		.cpu_dai_name = "LowLatency Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:1f.3",
		.ignore_suspend = 1,
		.dynamic = 1,
	},
	{
		.name = "Skl Compress Port",
		.stream_name = "Compress Audio",
		.cpu_dai_name = "Compress Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:1f.3",
		.dynamic = 1,
	},
	{
		.name = "Skl VOIP Port",
		.stream_name = "Voip",
		.cpu_dai_name = "VOIP Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:1f.3",
		.init = NULL,
		.ignore_suspend = 1,
		.dynamic = 1,
	},
/*	{
		.name = "Skl HDMI Port",
		.stream_name = "Hdmi",
		.cpu_dai_name = "System Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:1f.3",
		.init = NULL,
		.ignore_suspend = 1,
		.dynamic = 1,
	},*/
	{
		.name = "Skl Audio Refrence cap",
		.stream_name = "refcap",
		.cpu_dai_name = "Refrence Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:1f.3",
		.init = NULL,
		.ignore_suspend = 1,
		.dynamic = 1,
	},

	/* back ends */
	{
		.name = "HDA-Codec HS",
		.be_id = 1,
		.cpu_dai_name = "HD-Codec Pin",
		.codec_name = "codec#000.0",
		.codec_dai_name = "ALC286-AIF2",
		.platform_name = "0000:00:1f.3",
		.ignore_suspend = 1,
		.no_pcm = 1,
	},
	{
		.name = "HDA-Codec SPK",
		.be_id = 2,
		.cpu_dai_name = "HD-Codec Pin",
		.codec_name = "codec#000.0",
		.codec_dai_name = "ALC286-AIF2",
		.platform_name = "0000:00:1f.3",
		.ignore_suspend = 1,
		.no_pcm = 1,
	},
	{
		.name = "HDA-Codec Record",
		.be_id = 3,
		.cpu_dai_name = "HD-Codec Pin",
		.codec_name = "codec#000.0",
		.codec_dai_name = "ALC286-AIF3",
		.platform_name = "0000:00:1f.3",
		.ignore_suspend = 1,
		.no_pcm = 1,
	},
	{
		.name = "SSP1-BTFM",
		.be_id = 4,
		.cpu_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "snd-soc-dummy",
		.ignore_suspend = 1,
		.no_pcm = 1,
	},
	/*{
		.name = "iDisp",
		.be_id = 5,
		.cpu_dai_name = "iDisp Pin",
		.codec_name = "codec#001.1",
		.codec_dai_name = "intel-hdmi-hif1",
		.platform_name = "0000:00:1f.3",
		.ignore_suspend = 1,
		.no_pcm = 1,
	},*/
	{
	.name = "dmic01",
	.be_id = 5,
	.cpu_dai_name = "DMIC01 Pin",
	.codec_name = "snd-soc-dummy",
	.codec_dai_name = "snd-soc-dummy-dai",
	.platform_name = "0000:00:1f.3",
	.ignore_suspend = 1,
	.no_pcm = 1,
	},
	{
	.name = "dmic23",
	.be_id = 6,
	.cpu_dai_name = "DMIC23 Pin",
	.codec_name = "snd-soc-dummy",
	.codec_dai_name = "snd-soc-dummy-dai",
	.platform_name = "0000:00:1f.3",
	.ignore_suspend = 1,
	.no_pcm = 1,
	},
};

/* SoC card */
static struct snd_soc_card snd_soc_card_skl = {
	.name = "skl-audio",
	.dai_link = skl_audio_msic_dailink,
	.num_links = ARRAY_SIZE(skl_audio_msic_dailink),
	.dapm_routes = skl_map,
	.num_dapm_routes = ARRAY_SIZE(skl_map),
};

static int snd_skl_realtek_mc_probe(struct platform_device *pdev)
{
	int ret_val = 0;
	dev_dbg(&pdev->dev, "Entry %s\n", __func__);

	/* register the soc card */
	snd_soc_card_skl.dev = &pdev->dev;
	ret_val = snd_soc_register_card(&snd_soc_card_skl);
	if (ret_val) {
		pr_err("snd_soc_register_card failed %d\n", ret_val);
		goto unalloc;
	}
	platform_set_drvdata(pdev, &snd_soc_card_skl);
	dev_info(&pdev->dev, "%s successful\n", __func__);
	return ret_val;

unalloc:
	return ret_val;
}

static int snd_skl_realtek_mc_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);

	dev_dbg(&pdev->dev, "In %s\n", __func__);

	snd_soc_unregister_card(soc_card);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

static struct platform_driver snd_skl_realtek_mc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "morg_florida",
	},
	.probe = snd_skl_realtek_mc_probe,
	.remove = snd_skl_realtek_mc_remove,
};

static int snd_skl_realtek_driver_init(void)
{
	pr_info("Morganfield Machine Driver registerd\n");
	return platform_driver_register(&snd_skl_realtek_mc_driver);
}
module_init(snd_skl_realtek_driver_init);

static void snd_skl_realtek_driver_exit(void)
{
	pr_debug("In %s\n", __func__);
	platform_driver_unregister(&snd_skl_realtek_mc_driver);
}
module_exit(snd_skl_realtek_driver_exit)

MODULE_DESCRIPTION("ASoC Skylake Machine driver");
MODULE_AUTHOR("Samreen Nilofer <samreen.nilofer@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:skl_audio");
