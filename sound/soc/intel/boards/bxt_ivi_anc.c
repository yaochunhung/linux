/*
 * Intel Broxton-P I2S Machine Driver
 *
 * Copyright (C) 2014-2015, Intel Corporation. All rights reserved.
 *
 * Modified from:
 *   Intel Skylake I2S Machine driver
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <sound/pcm_params.h>

#include "../../codecs/rt298.h"

static struct snd_soc_jack broxton_headset;
/* Headset jack detection DAPM pins */
static struct snd_soc_jack_pin broxton_headset_pins[] = {
	{
		.pin = "Mic Jack",
		.mask = SND_JACK_MICROPHONE,
	},
	{
		.pin = "Headphone Jack",
		.mask = SND_JACK_HEADPHONE,
	},
};

static const struct snd_soc_pcm_stream bxtn_dai_params_codec = {
	.formats = SNDRV_PCM_FMTBIT_S24_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 8,
	.channels_max = 8,
};

static const struct snd_kcontrol_new broxton_controls[] = {
	SOC_DAPM_PIN_SWITCH("Speaker"),
	SOC_DAPM_PIN_SWITCH("Headphone Jack"),
	SOC_DAPM_PIN_SWITCH("Mic Jack"),
};

static const struct snd_soc_dapm_widget broxton_widgets[] = {
	SND_SOC_DAPM_HP("Headphone Jack", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_MIC("Mic Jack", NULL),
	SND_SOC_DAPM_MIC("DMIC2", NULL),
	SND_SOC_DAPM_MIC("SoC DMIC", NULL),
	SND_SOC_DAPM_SPK("DummySpeaker1", NULL),
	SND_SOC_DAPM_SPK("DummySpeaker2", NULL),
	SND_SOC_DAPM_SPK("DummySpeaker4", NULL),
	SND_SOC_DAPM_MIC("DummyMIC0", NULL),
	SND_SOC_DAPM_MIC("DummyMIC2", NULL),
	SND_SOC_DAPM_MIC("DummyMIC4", NULL),
};

static const struct snd_soc_dapm_route broxton_rt298_map[] = {
	/* speaker */
	{"Speaker", NULL, "SPOR"},
	{"Speaker", NULL, "SPOL"},

	/* HP jack connectors - unknown if we have jack detect */
	{"Headphone Jack", NULL, "HPO Pin"},

	/* other jacks */
	{"MIC1", NULL, "Mic Jack"},

	/* digital mics */
	{"DMIC1 Pin", NULL, "DMIC2"},
	{"DMIC AIF", NULL, "SoC DMIC"},

	/* CODEC BE connections */
	{"AIF1 Playback", NULL, "ssp5 Tx"},
	{"ssp5 Tx", NULL, "codec0_out"},
	{"ssp5 Tx", NULL, "codec_pt_out"},

	{"codec_pt_in", NULL, "ssp5 Rx" },
	{"codec0_in", NULL, "ssp5 Rx" },
	{"ssp5 Rx", NULL, "AIF1 Capture" },

	{"dmic01_hifi", NULL, "DMIC01 Rx" },
	{"dmic23_hifi", NULL, "DMIC23 Rx" },
	{"DMIC01 Rx", NULL, "Capture" },
	{"DMIC23 Rx", NULL, "Capture" },

	{"8ch_pt_in3", NULL, "ssp0 Rx" },
	{"ssp0 Rx", NULL, "Dummy Capture" },
	{"Dummy Capture", NULL, "DummyMIC0"},

	{"DummySpeaker1", NULL, "Dummy Playback1"},
	{"Dummy Playback1", NULL, "ssp1 Tx"},
	{"ssp1 Tx", NULL, "8ch_pt_out2"},

	{"DummySpeaker2", NULL, "Dummy Playback2"},
	{"Dummy Playback2", NULL, "ssp2 Tx"},
	{"ssp2 Tx", NULL, "8ch_pt_out3"},

	{"8ch_pt_in2", NULL, "ssp2 Rx" },
	{"ssp2 Rx", NULL, "Dummy Capture2" },
	{"Dummy Capture2", NULL, "DummyMIC2"},

	{"DummySpeaker4", NULL, "Dummy Playback4"},
	{"Dummy Playback4", NULL, "ssp4 Tx"},
	{"ssp4 Tx", NULL, "8ch_pt_out"},

	{"8ch_pt_in", NULL, "ssp4 Rx" },
	{"ssp4 Rx", NULL, "Dummy Capture4" },
	{"Dummy Capture4", NULL, "DummyMIC4"},

	/* (ANC) Codec1_in - pipe */
	{ "codec1_in", NULL, "ssp0 Rx" },
	{ "ssp0 Rx", NULL, "Dummy Capture" },

	/* CodecX_in - pipe */
	{ "codecX_in", NULL, "ssp2 Rx" },
	{ "ssp2 Rx", NULL, "Dummy Capture2" },

	/* Media1_out  Path */
	{ "Dummy Playback2", NULL, "ssp2 Tx"},
	{ "ssp2 Tx", NULL, "media1_out"},
};

static int broxton_rt298_codec_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_codec *codec = rtd->codec;
	int ret = 0;

	ret = snd_soc_card_jack_new(rtd->card, "Headset",
		SND_JACK_HEADSET | SND_JACK_BTN_0,
		&broxton_headset,
		broxton_headset_pins, ARRAY_SIZE(broxton_headset_pins));

	if (ret)
		return ret;

	rt298_mic_detect(codec, &broxton_headset);
	return 0;
}


static int broxton_ssp5_fixup(struct snd_soc_pcm_runtime *rtd,
			struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_mask *fmt = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);

	/* The ADSP will covert the FE rate to 48k, stereo */
	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;

	/* set SSP5 to 24 bit */
	snd_mask_none(fmt);
	snd_mask_set(fmt, SNDRV_PCM_FORMAT_S24_LE);
	return 0;
}

static int broxton_rt298_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	int ret;

	ret = snd_soc_dai_set_sysclk(codec_dai, RT298_SCLK_S_PLL, 19200000,
		SND_SOC_CLOCK_IN);
	if (ret < 0) {
		dev_err(rtd->dev, "can't set codec sysclk configuration\n");
		return ret;
	}

	return ret;
}

static int bxtp_ssp0_gpio_init(struct snd_soc_pcm_runtime *rtd)
{
	char *gpio_addr;
	u32 gpio_value1 = 0x40900500;
	u32 gpio_value2 = 0x44000600;

	gpio_addr = (void *)ioremap_nocache(0xd0c40610, 0x30);
	if (gpio_addr == NULL)
		return(-EIO);

	memcpy_toio(gpio_addr + 0x8, &gpio_value1, sizeof(gpio_value1));
	memcpy_toio(gpio_addr + 0x10, &gpio_value2, sizeof(gpio_value2));
	memcpy_toio(gpio_addr + 0x18, &gpio_value2, sizeof(gpio_value2));
	memcpy_toio(gpio_addr + 0x20, &gpio_value2, sizeof(gpio_value2));

	iounmap(gpio_addr);
	return 0;
}

static int bxtp_ssp1_gpio_init(struct snd_soc_pcm_runtime *rtd)
{

	char *gpio_addr;
	u32 gpio_value1 = 0x44000400;

	gpio_addr = (void *)ioremap_nocache(0xd0c40660, 0x30);
	if (gpio_addr == NULL)
		return(-EIO);

	memcpy_toio(gpio_addr + 0x8, &gpio_value1, sizeof(gpio_value1));
	memcpy_toio(gpio_addr + 0x10, &gpio_value1, sizeof(gpio_value1));
	memcpy_toio(gpio_addr + 0x18, &gpio_value1, sizeof(gpio_value1));
	memcpy_toio(gpio_addr + 0x20, &gpio_value1, sizeof(gpio_value1));

	iounmap(gpio_addr);
	return 0;
}

static int bxtp_ssp4_gpio_init(struct snd_soc_pcm_runtime *rtd)
{

	char *gpio_addr;
	u32 gpio_value1 = 0x44000A00;
	u32 gpio_value2 = 0x44000800;

	gpio_addr = (void *)ioremap_nocache(0xd0c705A0, 0x30);
	if (gpio_addr == NULL)
		return(-EIO);

	memcpy_toio(gpio_addr, &gpio_value1, sizeof(gpio_value1));
	memcpy_toio(gpio_addr + 0x8, &gpio_value1, sizeof(gpio_value1));
	memcpy_toio(gpio_addr + 0x10, &gpio_value1, sizeof(gpio_value1));
	memcpy_toio(gpio_addr + 0x18, &gpio_value2, sizeof(gpio_value2));

	iounmap(gpio_addr);
	return 0;

}

static struct snd_soc_ops broxton_rt298_ops = {
	.hw_params = broxton_rt298_hw_params,
};

/* broxton digital audio interface glue - connects codec <--> CPU */
static struct snd_soc_dai_link broxton_rt298_dais[] = {
	/* Front End DAI links */
	{
		.name = "Bxt Audio Port",
		.stream_name = "Audio",
		.cpu_dai_name = "System Pin",
		.platform_name = "0000:00:0e.0",
		.nonatomic = 1,
		.dynamic = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.dpcm_playback = 1,
	},
	{	/* Passthrough stereo PB & Cap to Codec */
		.name = "Bxt Audio Port 2",
		.stream_name = "Stereo-Passthrough",
		.cpu_dai_name = "System Pin 2",
		.platform_name = "0000:00:0e.0",
		.nonatomic = 1,
		.dynamic = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai2",
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	{	/* Passthrough 8-ch PB & Cap to dummy codec */
		.name = "Bxt Audio Port 3",
		.stream_name = "8-ch Dummy PT SSP4",
		.cpu_dai_name = "System Pin 3",
		.platform_name = "0000:00:0e.0",
		.nonatomic = 1,
		.dynamic = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai4",
		.trigger = {SND_SOC_DPCM_TRIGGER_POST,
			SND_SOC_DPCM_TRIGGER_POST},
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
        {       /* Passthrough 8-ch PB & Cap to dummy codec */
                .name = "Bxt Audio Port 4",
                .stream_name = "8-ch Dummy PT 2 SSP1",
                .cpu_dai_name = "System Pin 4",
                .platform_name = "0000:00:0e.0",
                .nonatomic = 1,
                .dynamic = 1,
                .codec_name = "snd-soc-dummy",
                .codec_dai_name = "snd-soc-dummy-dai1",
                .trigger = {SND_SOC_DPCM_TRIGGER_POST,
                        SND_SOC_DPCM_TRIGGER_POST},
                .dpcm_playback = 1,
        },
        {       /* Passthrough 8-ch PB & Cap to dummy codec */
                .name = "Bxt Audio Port 5",
                .stream_name = "8-ch Dummy PT 3 SSP2",
                .cpu_dai_name = "System Pin 5",
                .platform_name = "0000:00:0e.0",
                .nonatomic = 1,
                .dynamic = 1,
                .codec_name = "snd-soc-dummy",
                .codec_dai_name = "snd-soc-dummy-dai2",
                .trigger = {SND_SOC_DPCM_TRIGGER_POST,
                        SND_SOC_DPCM_TRIGGER_POST},
                .dpcm_playback = 1,
                .dpcm_capture = 1,
        },
	{
		.name = "Bxt Audio Capture Port",
		.stream_name = "Audio Record",
		.cpu_dai_name = "System Pin",
		.platform_name = "0000:00:0e.0",
		.nonatomic = 1,
		.dynamic = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
		.dpcm_capture = 1,
	},
        {
                .name = "Bxt Audio Port 6",
                .stream_name = "Audio Record 2 SSP0",
                .cpu_dai_name = "System Pin 6",
                .platform_name = "0000:00:0e.0",
                .nonatomic = 1,
                .dynamic = 1,
                .codec_name = "snd-soc-dummy",
                .codec_dai_name = "snd-soc-dummy-dai",
                .trigger = {SND_SOC_DPCM_TRIGGER_POST, SND_SOC_DPCM_TRIGGER_POST},
                .dpcm_capture = 1,
        },
	{
		.name = "Bxt Audio Reference cap",
		.stream_name = "refcap",
		.cpu_dai_name = "Reference Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.dpcm_capture = 1,
		.ignore_suspend = 1,
		.nonatomic = 1,
		.dynamic = 1,
	},
	{
		.name = "Bxt Compress Probe playback",
		.stream_name = "Probe Playback",
		.cpu_dai_name = "Compress Probe0 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.nonatomic = 1,
	},
	{
		.name = "Bxt Compress Probe capture",
		.stream_name = "Probe Capture",
		.cpu_dai_name = "Compress Probe1 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.init = NULL,
		.nonatomic = 1,
	},
	/* Trace Buffer DAI links */
	{
		.name = "Bxt Trace Buffer0",
		.stream_name = "Core 0 Trace Buffer",
		.cpu_dai_name = "TraceBuffer0 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.capture_only = true,
		.ignore_suspend = 1,
	},
	{
		.name = "Bxt Trace Buffer1",
		.stream_name = "Core 1 Trace Buffer",
		.cpu_dai_name = "TraceBuffer1 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:00:0e.0",
		.capture_only = true,
		.ignore_suspend = 1,
	},
		/* CODEC<->CODEC link */
	{
		.name = "Bxtn SSP0 Port",
		.stream_name = "Bxtn SSP0",
		.cpu_dai_name = "SSP0 Pin",
		.platform_name = "0000:00:0e.0",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.params = &bxtn_dai_params_codec,
		.dsp_loopback = true,
		.dai_fmt = SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
	},

	{
		.name = "Bxtn SSP2 port",
		.stream_name = "Bxtn SSP2",
		.cpu_dai_name = "SSP2 Pin",
		.platform_name = "0000:00:0e.0",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai2",
		.params = &bxtn_dai_params_codec,
		.dsp_loopback = true,
		.dai_fmt = SND_SOC_DAIFMT_DSP_A |
			SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
	},

	/* Back End DAI links */
	{
		/* SSP5 - Codec */
		.name = "SSP5-Codec",
		.be_id = 0,
		.cpu_dai_name = "SSP5 Pin",
		.platform_name = "0000:00:0e.0",
		.no_pcm = 1,
		.codec_name = "i2c-INT343A:00",
		.codec_dai_name = "rt298-aif1",
		.init = broxton_rt298_codec_init,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			SND_SOC_DAIFMT_CBS_CFS,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.be_hw_params_fixup = broxton_ssp5_fixup,
		.ops = &broxton_rt298_ops,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
	{
		/* SSP4 - Codec */
		.name = "SSP4-Codec",
		.be_id = 3,
		.cpu_dai_name = "SSP4 Pin",
		.platform_name = "0000:00:0e.0",
		.no_pcm = 1,
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai4",
                .init = bxtp_ssp4_gpio_init,
		.ignore_suspend = 1,
		.ignore_pmdown_time = 1,
		.dpcm_playback = 1,
		.dpcm_capture = 1,
	},
        {
                /* SSP1 - Codec */
                .name = "SSP1-Codec",
                .be_id = 4,
                .cpu_dai_name = "SSP1 Pin",
                .platform_name = "0000:00:0e.0",
                .no_pcm = 1,
                .codec_name = "snd-soc-dummy",
                .codec_dai_name = "snd-soc-dummy-dai1",
                .init = bxtp_ssp1_gpio_init,
                .ignore_suspend = 1,
                .ignore_pmdown_time = 1,
                .dpcm_playback = 1,
        },
        {
                /* SSP2 - Codec */
                .name = "SSP2-Codec",
                .be_id = 5,
                .cpu_dai_name = "SSP2 Pin",
                .platform_name = "0000:00:0e.0",
                .no_pcm = 1,
                .codec_name = "snd-soc-dummy",
                .codec_dai_name = "snd-soc-dummy-dai2",
                .ignore_suspend = 1,
                .ignore_pmdown_time = 1,
                .dpcm_playback = 1,
                .dpcm_capture = 1,
        },
        {
                /* SSP0 - Codec */
                .name = "SSP0-Codec",
                .be_id = 6,
                .cpu_dai_name = "SSP0 Pin",
                .platform_name = "0000:00:0e.0",
                .no_pcm = 1,
                .codec_name = "snd-soc-dummy",
                .codec_dai_name = "snd-soc-dummy-dai",
                .init = bxtp_ssp0_gpio_init,
                .ignore_suspend = 1,
                .ignore_pmdown_time = 1,
                .dpcm_capture = 1,
        },
	{
		.name = "dmic01",
		.be_id = 1,
		.cpu_dai_name = "DMIC01 Pin",
		.codec_name = "dmic-codec",
		.codec_dai_name = "dmic-hifi",
		.platform_name = "0000:00:0e.0",
		.ignore_suspend = 1,
		.dpcm_capture = 1,
		.no_pcm = 1,
	},
	{
		.name = "dmic23",
		.be_id = 2,
		.cpu_dai_name = "DMIC23 Pin",
		.codec_name = "dmic-codec",
		.codec_dai_name = "dmic-hifi",
		.platform_name = "0000:00:0e.0",
		.ignore_suspend = 1,
		.dpcm_capture = 1,
		.no_pcm = 1,
	},
};

/* broxton audio machine driver for SPT + RT298S */
static struct snd_soc_card broxton_rt298 = {
	.name = "broxton-rt298",
	.owner = THIS_MODULE,
	.dai_link = broxton_rt298_dais,
	.num_links = ARRAY_SIZE(broxton_rt298_dais),
	.controls = broxton_controls,
	.num_controls = ARRAY_SIZE(broxton_controls),
	.dapm_widgets = broxton_widgets,
	.num_dapm_widgets = ARRAY_SIZE(broxton_widgets),
	.dapm_routes = broxton_rt298_map,
	.num_dapm_routes = ARRAY_SIZE(broxton_rt298_map),
	.fully_routed = false,
};

static int broxton_audio_probe(struct platform_device *pdev)
{
	broxton_rt298.dev = &pdev->dev;

	return snd_soc_register_card(&broxton_rt298);
}

static int broxton_audio_remove(struct platform_device *pdev)
{
	snd_soc_unregister_card(&broxton_rt298);
	return 0;
}

static struct platform_driver broxton_audio = {
	.probe = broxton_audio_probe,
	.remove = broxton_audio_remove,
	.driver = {
		.name = "bxt_alc298s_i2s",
		.pm = &snd_soc_pm_ops,
	},
};

module_platform_driver(broxton_audio)

/* Module information */
MODULE_AUTHOR("Ramesh Babu <Ramesh.Babu@intel.com>");
MODULE_AUTHOR("Senthilnathan Veppur <senthilnathanx.veppur@intel.com>");
MODULE_DESCRIPTION("Intel SST Audio for Broxton");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:bxt_alc298s_i2s");
