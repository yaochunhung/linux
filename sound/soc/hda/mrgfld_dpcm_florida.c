/*
 *  mrgfld_dpcm_florida.c - ASOC Machine driver for Intel Morganfield platform
 *
 *  Copyright (C) 2014 Wolfson Micro
 *  Copyright (C) 2014 Intel Corp
 *  Author: Samreen Nilofer <samreen.nilofer@intel.com>
 *
 * Based on
 *	moor_dpcm_florida.c - ASOC Machine driver for Intel Moorefield MID platform
 *  Copyright (C) 2014 Wolfson Micro
 *  Copyright (C) 2014 Intel Corp
 *  Author: Nikesh Oswal <Nikesh.Oswal@wolfsonmicro.com>
 *  	    Praveen Diwakar <praveen.diwakar@intel.com>
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
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/async.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/jack.h>
#include <linux/input.h>
#include <sound/soc-hda-bus.h>

#ifdef OSC_PMIC
#include <asm/intel_scu_pmic.h>
#include <asm/intel_mid_rpmsg.h>
#include <asm/intel-mid.h>
#endif /* OSC_PMIC */

#include <linux/mfd/arizona/registers.h>
#include "../codecs/florida.h"

/* Codec PLL output clk rate */
#define CODEC_SYSCLK_RATE			49152000
/* Input clock to codec at MCLK1 PIN */
#define CODEC_IN_MCLK1_RATE			19200000
/* Input clock to codec at MCLK2 PIN */
#define CODEC_IN_MCLK2_RATE			32768
/* Input bit clock to codec */
#define CODEC_IN_BCLK_RATE			4800000

/*  define to select between MCLK1 and MCLK2 input to codec as its clock */
#define CODEC_IN_MCLK1				1
#define CODEC_IN_MCLK2				2
#define CODEC_IN_BCLK				3

/* TODO: check the OSC and PMIC stuff for Morganfield */
/* Register address for OSC Clock */
#define MERR_OSC_CLKOUT_CTRL0_REG_ADDR  0xFF00BC04
/* Size of osc clock register */
#define MERR_OSC_CLKOUT_CTRL0_REG_SIZE  4

static int bt_debug;
module_param(bt_debug, int, S_IRUGO);
MODULE_PARM_DESC(bt_debug, "0: bt_debug disable, 1: debug bt in wideband mode, 2: debug bt in narrow band mode");


struct mrgfld_mc_private {
	u8		pmic_id;
	void __iomem    *osc_clk0_reg;
};
static const struct snd_soc_pcm_stream bxtn_florida_dai_params_codec_default = {
	.formats = SNDRV_PCM_FMTBIT_S24_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
};

static const struct snd_soc_pcm_stream bxtn_florida_dai_params_codec_bt_wb = {
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rate_min = 16000,
	.rate_max = 16000,
	.channels_min = 1,
	.channels_max = 1,
};
static const struct snd_soc_pcm_stream bxtn_florida_dai_params_codec_bt_nb = {
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rate_min = 8000,
	.rate_max = 8000,
	.channels_min = 1,
	.channels_max = 1,
};

static const struct snd_soc_pcm_stream bxtn_florida_dai_params_modem = {
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
};
static const struct snd_soc_pcm_stream bxtn_florida_dai_params_bt = {
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
};

/* set_osc_clk0-	enable/disables the osc clock0
 * addr:		address of the register to write to
 * enable:		bool to enable or disable the clock
 */
static inline void set_soc_osc_clk0(void __iomem *addr, bool enable)
{
	u32 osc_clk_ctrl;

	osc_clk_ctrl = readl(addr);
	if (enable)
		osc_clk_ctrl |= BIT(31);
	else
		osc_clk_ctrl &= ~(BIT(31));

	pr_debug("%s: enable:%d val 0x%x\n", __func__, enable, osc_clk_ctrl);

	writel(osc_clk_ctrl, addr);
}

static inline struct snd_soc_codec *morg_florida_get_codec(struct snd_soc_card *card)
{
	bool found = false;
	struct snd_soc_codec *codec;

	list_for_each_entry(codec, &card->codec_dev_list, card_list) {
		if (!strstr(codec->name, "florida-codec")) {
			pr_debug("codec was %s", codec->name);
			continue;
		} else {
			found = true;
			break;
		}
	}
	if (found == false) {
		pr_err("%s: cant find codec", __func__);
		return NULL;
	}
	return codec;
}

static struct snd_soc_dai *morg_florida_get_codec_dai(struct snd_soc_card *card, const char *dai_name)
{
	int i;
	for (i = 0; i < card->num_rtd; i++) {
		if (!strcmp(card->rtd[i].codec_dai->name, dai_name))
			return card->rtd[i].codec_dai;
	}
	pr_err("%s: unable to find codec dai\n", __func__);
	/* this should never occur */
	WARN_ON(1);
	return NULL;
}

static struct snd_soc_dai *mrgfld_florida_get_cpu_dai(struct snd_soc_card *card,
							const char *dai_name)
{
	int i;
	for (i = 0; i < card->num_rtd; i++) {
		if (!strcmp(card->rtd[i].cpu_dai->name, dai_name))
			return card->rtd[i].cpu_dai;
	}
	pr_err("%s: unable to find cpu dai\n", __func__);
	/* this should never occur */
	WARN_ON(1);
	return NULL;
}

/* Function to switch the input clock for codec,  When audio is in
 * progress input clock to codec will be through MCLK1 which is 19.2MHz
 * while in off state input clock to codec will be through 32KHz through
 * MCLK2
 * card	: Sound card structure
 * src	: Input clock source to codec
 */

static int morg_florida_set_codec_clk(struct snd_soc_codec *florida_codec, int src)
{
	int ret;

	pr_debug("morg_florida_set_codec_clk: source %d\n", src);

	/*reset FLL1*/
	snd_soc_codec_set_pll(florida_codec, FLORIDA_FLL1_REFCLK,
				ARIZONA_FLL_SRC_NONE, 0, 0);
	snd_soc_codec_set_pll(florida_codec, FLORIDA_FLL1,
				ARIZONA_FLL_SRC_NONE, 0, 0);

	switch (src) {
	case CODEC_IN_MCLK1:
		/* Turn ON the PLL to generate required sysclk rate
		 * from MCLK1 */
		ret = snd_soc_codec_set_pll(florida_codec, FLORIDA_FLL1,
				ARIZONA_CLK_SRC_MCLK1, CODEC_IN_MCLK1_RATE,
				CODEC_SYSCLK_RATE);
		if (ret != 0) {
			dev_err(florida_codec->dev, "Failed to enable FLL1 with Ref(MCLK) Clock Loop: %d\n", ret);
			return ret;
		}
		break;
	case CODEC_IN_BCLK:
		/* Turn ON the PLL to generate required sysclk rate
		 * from BCLK */
		ret = snd_soc_codec_set_pll(florida_codec, FLORIDA_FLL1,
				ARIZONA_CLK_SRC_AIF1BCLK, CODEC_IN_BCLK_RATE,
				CODEC_SYSCLK_RATE);
		if (ret != 0) {
			dev_err(florida_codec->dev, "Failed to enable FLL1 with Ref Clock Loop: %d\n", ret);
			return ret;
		}

		break;
	default:
		return -EINVAL;
	}

	/*Switch to PLL*/
	ret = snd_soc_codec_set_sysclk(florida_codec,
			ARIZONA_CLK_SYSCLK, ARIZONA_CLK_SRC_FLL1,
			CODEC_SYSCLK_RATE, SND_SOC_CLOCK_IN);
	if (ret != 0) {
		dev_err(florida_codec->dev, "Failed to set SYSCLK to FLL1: %d\n", ret);
		return ret;
	}

	return 0;
}

#define PMIC_ID_ADDR		0x00
#define PMIC_CHIP_ID_A0_VAL	0xC0

#ifdef OSC_PMIC
static int morg_florida_set_vflex_vsel(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *k, int event)
{
#define VFLEXCNT		0xAB
#define VFLEXVSEL_5V		0x01
#define VFLEXVSEL_B0_VSYS_PT	0x80	/* B0: Vsys pass-through */
#define VFLEXVSEL_A0_4P5V	0x41	/* A0: 4.5V */

	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct morg_8281_mc_private *ctx = snd_soc_card_get_drvdata(card);

	u8 vflexvsel, pmic_id = ctx->pmic_id;
	int retval = 0;

	pr_debug("%s: ON? %d\n", __func__, SND_SOC_DAPM_EVENT_ON(event));

	vflexvsel = (pmic_id == PMIC_CHIP_ID_A0_VAL) ? VFLEXVSEL_A0_4P5V : VFLEXVSEL_B0_VSYS_PT;
	pr_debug("pmic_id %#x vflexvsel %#x\n", pmic_id,
		SND_SOC_DAPM_EVENT_ON(event) ? VFLEXVSEL_5V : vflexvsel);

	/*FIXME: seems to be issue with bypass mode in MOOR, for now
		force the bias off volate as VFLEXVSEL_5V */
	if ((INTEL_MID_BOARD(1, PHONE, MOFD)) ||
			(INTEL_MID_BOARD(1, TABLET, MOFD)))
		vflexvsel = VFLEXVSEL_5V;

	if (SND_SOC_DAPM_EVENT_ON(event))
		retval = intel_scu_ipc_iowrite8(VFLEXCNT, VFLEXVSEL_5V);
	else if (SND_SOC_DAPM_EVENT_OFF(event))
		retval = intel_scu_ipc_iowrite8(VFLEXCNT, vflexvsel);
	if (retval)
		pr_err("Error writing to VFLEXCNT register\n");

	return retval;
}
#endif

static int mrgfld_clock_control(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *k, int  event)
{

	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_card *card = dapm->card;
	struct snd_soc_codec *florida_codec = morg_florida_get_codec(card);
	int ret = 0;

	if (!florida_codec) {
		pr_err("%s: florida codec not found\n", __func__);
		return -EINVAL;
	}
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		pr_info("%s %d Event On\n", __func__, __LINE__);
		/* TODO: Ideally MCLK should be used to drive codec PLL
		 * currently we are using BCLK
		 */
		ret = morg_florida_set_codec_clk(florida_codec, CODEC_IN_BCLK);
	} else {
		pr_info("%s %d Event Off\n", __func__, __LINE__);
		/* TODO: Switch to 32K clock for saving power. */
		pr_info("Currently we are not switching to 32K PMIC clock\n");
	}
	return ret;

}
static const struct snd_soc_dapm_widget morg_widgets[] = {
	SND_SOC_DAPM_HP("Headphones", NULL),
	SND_SOC_DAPM_SPK("Ext Spk", NULL),
	SND_SOC_DAPM_SPK("EP", NULL),
	SND_SOC_DAPM_MIC("AMIC", NULL),
	SND_SOC_DAPM_MIC("DMIC", NULL),
	SND_SOC_DAPM_SUPPLY("Platform Clock", SND_SOC_NOPM, 0, 0,
			mrgfld_clock_control, SND_SOC_DAPM_PRE_PMU|
			SND_SOC_DAPM_POST_PMD),

#ifdef OSC_PMIC
	SND_SOC_DAPM_SUPPLY("VFLEXCNT", SND_SOC_NOPM, 0, 0,
			morg_florida_set_vflex_vsel,
			SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
#endif
};

static const struct snd_soc_dapm_route morg_map[] = {
	/*Headphones*/
	{ "Headphones", NULL, "HPOUT1L" },
	{ "Headphones", NULL, "HPOUT1R" },

	/*Speakers*/
	{"Ext Spk", NULL, "SPKOUTLP"},
	{"Ext Spk", NULL, "SPKOUTLN"},
	{"Ext Spk", NULL, "SPKOUTRP"},
	{"Ext Spk", NULL, "SPKOUTRN"},

	/*Earpiece*/
	{ "EP", NULL, "HPOUT3L" },
	{ "EP", NULL, "HPOUT3R" },

	/*On Board DMIC*/
	{"IN2L", NULL, "DMIC"},
	{"IN2R", NULL, "DMIC"},
	/* TODO: Currently MICBIAS is set according to WM8281 AOB for MOFD */
	{"DMIC", NULL, "MICBIAS3"},


	/* TODO: Currently MICBIAS is set according to WM8281 AOB for MOFD */
	{ "AMIC", NULL, "MICBIAS2" },
	{ "AMIC", NULL, "MICBIAS1" },
	{ "IN1L", NULL, "AMIC" },
	{ "IN1R", NULL, "AMIC" },

	/* SWM map link the SWM outs to codec AIF */
	{ "AIF1 Playback", NULL, "ssp0 Tx"},
	{ "ssp0 Tx", NULL, "codec1_out"},
	{ "ssp0 Tx", NULL, "codec0_out"},

	{ "codec0_in", NULL, "ssp0 Rx" },
	{ "codec1_in", NULL, "ssp0 Rx" },
	{ "dmic01_hifi", NULL, "DMIC01 Rx" },
	{ "dmic23_hifi", NULL, "DMIC23 Rx" },

	{ "ssp0 Rx", NULL, "AIF1 Capture" },
	{ "DMIC01 Rx", NULL, "Dummy Capture" },
	{ "DMIC23 Rx", NULL, "Dummy Capture" },

	/* TODO: map for rest of the ports */

	{ "hif1", NULL, "iDisp Tx"},
	{ "iDisp Tx", NULL, "iDisp_out"},

	/* Modem Path */
	{ "Dummy Playback", NULL, "ssp2 Tx"},
	{ "ssp2 Tx", NULL, "modem0_out"},

	{ "modem0_in", NULL, "ssp2 Rx" },
	{ "ssp2 Rx", NULL, "Dummy Capture" },

	/* Bt Path */
	{ "Dummy Playback", NULL, "ssp1 Tx"},
	{ "ssp1 Tx", NULL, "bt_out"},

	{ "bt_in", NULL, "ssp1 Rx" },
	{ "ssp1 Rx", NULL, "Dummy Capture" },


#ifdef OSC_PMIC
	{ "Dummy Playback", NULL, "VFLEXCNT"},
	{ "Dummy Capture", NULL, "VFLEXCNT"},
#endif

	{"Headphones", NULL, "Platform Clock"},
	{"AMIC", NULL, "Platform Clock"},
	{"DMIC", NULL, "Platform Clock"},
	{"Ext Spk", NULL, "Platform Clock"},
	{"EP", NULL, "Platform Clock"},
};

static const struct snd_kcontrol_new morg_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphones"),
	SOC_DAPM_PIN_SWITCH("Ext Spk"),
	SOC_DAPM_PIN_SWITCH("EP"),
	SOC_DAPM_PIN_SWITCH("AMIC"),
	SOC_DAPM_PIN_SWITCH("DMIC"),
};

static int morg_florida_init(struct snd_soc_pcm_runtime *runtime)
{
	int ret;
	unsigned int fmt;
	struct snd_soc_card *card = runtime->card;
	struct snd_soc_dai *florida_dai = morg_florida_get_codec_dai(card, "florida-aif1");
	int slot_width;


	pr_info("Entry %s\n", __func__);
	switch (bt_debug) {
	case 0:
		slot_width = 24;
		break;
	case 1:
	case 2:
		slot_width = 16;
		break;
	default:
		slot_width = 24;
	}
	pr_info("Slot width for codec = %d\n", slot_width);

	ret = snd_soc_dai_set_tdm_slot(florida_dai, 0, 0, 4, slot_width);
	/* slot width is set as 25, SNDRV_PCM_FORMAT_S32_LE */
	if (ret < 0) {
		pr_err("can't set codec pcm format %d\n", ret);
		return ret;
	}

	/* bit clock inverse not required */
	fmt =   SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_NB_NF
		| SND_SOC_DAIFMT_CBS_CFS;
	ret = snd_soc_dai_set_fmt(florida_dai, fmt);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}

	card->dapm.idle_bias_off = true;

	ret = snd_soc_add_card_controls(card, morg_controls,
					ARRAY_SIZE(morg_controls));
	if (ret) {
		pr_err("unable to add card controls\n");
		return ret;
	}
	return 0;
}

#if 0
static unsigned int rates_8000_16000[] = {
	8000,
	16000,
};
static struct snd_pcm_hw_constraint_list constraints_8000_16000 = {
	.count = ARRAY_SIZE(rates_8000_16000),
	.list  = rates_8000_16000,
};
#endif
static unsigned int rates_48000[] = {
	48000,
	16000,
	8000,
};

static struct snd_pcm_hw_constraint_list constraints_48000 = {
	.count = ARRAY_SIZE(rates_48000),
	.list  = rates_48000,
};

static int morg_florida_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE,
			&constraints_48000);
}

#if 0
static int morg_florida_8k_16k_startup(struct snd_pcm_substream *substream)
{
	return snd_pcm_hw_constraint_list(substream->runtime, 0,
			SNDRV_PCM_HW_PARAM_RATE,
			&constraints_8000_16000);
}
#endif
static struct snd_soc_ops morg_florida_ops = {
	.startup = morg_florida_startup,
};
#if 0
static struct snd_soc_ops morg_florida_8k_16k_ops = {
	.startup = morg_florida_8k_16k_startup,
	.hw_params = morg_florida_hw_params,
};
#endif
static struct snd_soc_compr_ops morg_compr_ops = {
	.set_params = NULL,
};
static int mrgfld_florida_modem_fixup(struct snd_soc_pcm_runtime *rtd,
			    struct snd_pcm_hw_params *params)
{
	struct snd_soc_dai *be_cpu_dai;
	int fmt;
	int ret = 0;

	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);

	pr_debug("Invoked %s for dailink %s\n", __func__, rtd->dai_link->name);

	/* The DSP will covert the FE rate to 48k, stereo, 24bits */
	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;

	snd_mask_none(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT));
	snd_mask_set(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT),
						SNDRV_PCM_FORMAT_S16_LE);


	be_cpu_dai = rtd->cpu_dai;

	/* bit clock inverse not required */
	fmt =   SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF
		| SND_SOC_DAIFMT_CBM_CFM;
	ret = snd_soc_dai_set_fmt(be_cpu_dai, fmt);
	if (ret < 0) {
		pr_err("can't set cpu DAI configuration %d\n", ret);
		return ret;
	}

	return ret;

}

static int mrgfld_florida_btfm_fixup(struct snd_soc_pcm_runtime *rtd,
			    struct snd_pcm_hw_params *params)
{
	struct snd_soc_dai *be_cpu_dai;
	int fmt;
	int ret = 0;
	int slot_width = 16;
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);

	pr_debug("Invoked %s for dailink %s\n", __func__, rtd->dai_link->name);
	slot_width = 24;
	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;
	snd_mask_none(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT));
	snd_mask_set(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT),
						SNDRV_PCM_FORMAT_S24_LE);

	pr_info("Slot width = %d\n", slot_width);
	pr_info("param width set to:0x%x\n",
				snd_pcm_format_width(params_format(params)));

	be_cpu_dai = rtd->cpu_dai;

	/* SoC SSP is master */
	fmt =   SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_NB_NF
		| SND_SOC_DAIFMT_CBM_CFM;
	ret = snd_soc_dai_set_fmt(be_cpu_dai, fmt);
	if (ret < 0) {
		pr_err("can't set cpu DAI configuration %d\n", ret);
		return ret;
	}

	ret = snd_soc_dai_set_tdm_slot(be_cpu_dai, 0, 0, 2, slot_width);
	return ret;

}
static int morg_florida_codec_fixup(struct snd_soc_pcm_runtime *rtd,
			    struct snd_pcm_hw_params *params)
{
	struct snd_soc_dai *be_cpu_dai;
	int slot_width = 24;
	int ret = 0;
	int fmt;
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);

	pr_debug("Invoked %s for dailink %s\n", __func__, rtd->dai_link->name);
	slot_width = 24;
	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;
	snd_mask_none(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT));
	snd_mask_set(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT),
						SNDRV_PCM_FORMAT_S24_LE);


	pr_info("param width set to:0x%x\n",
			snd_pcm_format_width(params_format(params)));
	pr_info("Slot width = %d\n", slot_width);

	be_cpu_dai = rtd->cpu_dai;
	ret = snd_soc_dai_set_tdm_slot(be_cpu_dai, 0, 0, 4, slot_width);
	if (ret < 0) {
		pr_err("can't set cpu dai pcm format %d\n", ret);
		return ret;
	}

	/* bit clock inverse not required */
	fmt =   SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_NB_NF
		| SND_SOC_DAIFMT_CBM_CFM;
	ret = snd_soc_dai_set_fmt(be_cpu_dai, fmt);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}

	return ret;
}

static int mrgfld_codec_loop_fixup(struct snd_soc_dai_link *dai_link,
		struct snd_soc_dai *dai, struct snd_pcm_hw_params *params)
{
	int ret;
	unsigned int fmt;
	int slot_width;
	struct snd_soc_card *card = dai->card;
	struct snd_soc_dai *florida_dai = morg_florida_get_codec_dai(card, "florida-aif1");
	struct snd_soc_dai *cpu_dai;
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);

	pr_debug("Invoked %s for dailink %s\n", __func__, dai_link->name);

	switch (bt_debug) {
	case 0:
		slot_width = 24;
		rate->min = rate->max = 48000;
		channels->min = channels->max = 2;
		snd_mask_none(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT));
		snd_mask_set(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT),
						SNDRV_PCM_FORMAT_S24_LE);
		break;
	case 1:
		slot_width = 16;
		rate->min = rate->max = 16000;
		channels->min = channels->max = 1;
		snd_mask_none(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT));
		snd_mask_set(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT),
						SNDRV_PCM_FORMAT_S16_LE);

	case 2:
		slot_width = 16;
		rate->min = rate->max = 8000;
		channels->min = channels->max = 1;
		snd_mask_none(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT));
		snd_mask_set(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT),
						SNDRV_PCM_FORMAT_S16_LE);
		break;
	default:
		slot_width = 24;
		rate->min = rate->max = 48000;
		channels->min = channels->max = 2;
		snd_mask_none(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT));
		snd_mask_set(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT),
						SNDRV_PCM_FORMAT_S24_LE);

	}
	pr_info("Slot width = %d\n", slot_width);
	pr_info("param width set to:0x%x\n",
				snd_pcm_format_width(params_format(params)));

	cpu_dai = mrgfld_florida_get_cpu_dai(card, dai_link->cpu_dai_name);
	if (!cpu_dai) {
		pr_err("%s: %s CPU dai not found\n", __func__,
							dai_link->cpu_dai_name);
		return -EINVAL;
	}

	/* Set up tdm params for codec */
	ret = snd_soc_dai_set_tdm_slot(florida_dai, 0, 0, 4, slot_width);
	if (ret < 0) {
		pr_err("can't set codec pcm format %d\n", ret);
		return ret;
	}

	/* Setup fmt for codec */
	fmt =   SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_NB_NF
		| SND_SOC_DAIFMT_CBS_CFS;
	ret = snd_soc_dai_set_fmt(florida_dai, fmt);
	if (ret < 0) {
		pr_err("can't set codec DAI configuration %d\n", ret);
		return ret;
	}

	/* Setup FMT for SSP on SoC side */
	fmt =   SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_NB_NF
		| SND_SOC_DAIFMT_CBM_CFM;
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret < 0) {
		pr_err("can't set format for CPU dai %s error %d\n",
							cpu_dai->name, ret);
		return ret;
	}

	/* Setup tdm slot for SSP on SoC side */
	ret = snd_soc_dai_set_tdm_slot(cpu_dai, 0, 0, 4, slot_width);
	if (ret < 0) {
		pr_err("Can't set the slot config for CPU dai %s error %d\n",
							cpu_dai->name, ret);
		return ret;
	}

	return ret;
}
static int mrgfld_modem_loop_fixup(struct snd_soc_dai_link *dai_link,
		struct snd_soc_dai *dai, struct snd_pcm_hw_params *params)
{
	int ret;
	unsigned int fmt;
	struct snd_soc_card *card = dai->card;
	struct snd_soc_dai *cpu_dai;
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);

	pr_debug("Invoked %s for dailink %s\n", __func__, dai_link->name);

	/* The DSP will covert the FE rate to 48k, stereo, 24bits */
	rate->min = rate->max = 48000;
	channels->min = channels->max = 2;

	snd_mask_none(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT));
	snd_mask_set(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT),
						SNDRV_PCM_FORMAT_S16_LE);

	cpu_dai = mrgfld_florida_get_cpu_dai(card, dai_link->cpu_dai_name);
	if (!cpu_dai) {
		pr_err("%s: %s CPU dai not found\n", __func__,
						dai_link->cpu_dai_name);
		return -EINVAL;
	}

	/* SoC SSP is master, both BCLK and FS */
	fmt =   SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF
		| SND_SOC_DAIFMT_CBM_CFM;
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret < 0) {
		pr_err("can't set format for CPU dai %s error %d\n",
							cpu_dai->name, ret);
		return ret;
	}
	/* TDM slot configuration is not required for I2S mode */

	return ret;

}

static int mrgfld_btfm_loop_fixup(struct snd_soc_dai_link *dai_link,
		struct snd_soc_dai *dai, struct snd_pcm_hw_params *params)
{
	int ret;
	unsigned int fmt;
	int slot_width;
	struct snd_soc_card *card = dai->card;
	struct snd_soc_dai *cpu_dai;
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);

	pr_debug("Invoked %s for dailink %s\n", __func__, dai_link->name);
	switch (bt_debug) {
	case 0:
		slot_width = 24;
		rate->min = rate->max = 48000;
		channels->min = channels->max = 2;
		snd_mask_none(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT));
		snd_mask_set(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT),
						SNDRV_PCM_FORMAT_S24_LE);
		break;
	case 1:
		slot_width = 16;
		rate->min = rate->max = 16000;
		channels->min = channels->max = 1;
		snd_mask_none(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT));
		snd_mask_set(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT),
						SNDRV_PCM_FORMAT_S16_LE);

	case 2:
		slot_width = 16;
		rate->min = rate->max = 8000;
		channels->min = channels->max = 1;
		snd_mask_none(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT));
		snd_mask_set(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT),
						SNDRV_PCM_FORMAT_S16_LE);
		break;
	default:
		slot_width = 24;
		rate->min = rate->max = 48000;
		channels->min = channels->max = 2;
		snd_mask_none(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT));
		snd_mask_set(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT),
						SNDRV_PCM_FORMAT_S24_LE);

	}
	pr_info("Slot width = %d\n", slot_width);
	pr_info("param width set to:0x%x\n",
				snd_pcm_format_width(params_format(params)));

	cpu_dai = mrgfld_florida_get_cpu_dai(card, dai_link->cpu_dai_name);
	if (!cpu_dai) {
		pr_err("%s: %s CPU dai not found\n", __func__,
						dai_link->cpu_dai_name);
		return -EINVAL;
	}


	/* SoC SSP is master, both BCLK and FS */
	fmt =   SND_SOC_DAIFMT_DSP_A | SND_SOC_DAIFMT_IB_NF
		| SND_SOC_DAIFMT_CBM_CFM;
	ret = snd_soc_dai_set_fmt(cpu_dai, fmt);
	if (ret < 0) {
		pr_err("can't set format for CPU dai %s error %d\n",
							cpu_dai->name, ret);
		return ret;
	}

	ret = snd_soc_dai_set_tdm_slot(cpu_dai, 0, 0, 2, slot_width);
	if (ret < 0) {
		pr_err("Can't set the slot config for CPU dai %s error %d\n",
							cpu_dai->name, ret);
		return ret;
	}

	return ret;
}


static const struct snd_soc_pcm_stream morg_florida_dai_params = {
	.formats = SNDRV_PCM_FMTBIT_S24_LE,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 4,
	.channels_max = 4,
};

struct snd_soc_dai_link morg_florida_msic_dailink[] = {
	{
		.name = "Bxtn Audio Port",
		.stream_name = "Audio",
		.cpu_dai_name = "System Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:02:18.0",
		.init = morg_florida_init,
		.ignore_suspend = 1,
		.dynamic = 1,
		.ops = &morg_florida_ops,
	},
	{
		.name = "Bxtn DB Audio Port",
		.stream_name = "Deep Buffer Audio",
		.cpu_dai_name = "Deepbuffer Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:02:18.0",
		.ignore_suspend = 1,
		.dynamic = 1,
		.ops = &morg_florida_ops,
	},
	{
		.name = "Bxtn LL Audio Port",
		.stream_name = "Low Latency Audio",
		.cpu_dai_name = "LowLatency Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:02:18.0",
		.ignore_suspend = 1,
		.dynamic = 1,
		.ops = &morg_florida_ops,
	},
	{
		.name = "Bxtn Compress Port",
		.stream_name = "Compress Audio",
		.cpu_dai_name = "Compress Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:02:18.0",
		.dynamic = 1,
		.compr_ops = &morg_compr_ops,
	},
	{
		.name = "Bxtn VOIP Port",
		.stream_name = "Voip",
		.cpu_dai_name = "VOIP Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:02:18.0",
		.init = NULL,
		.ignore_suspend = 1,
/*		.ops = &morg_florida_8k_16k_ops,*/
		.dynamic = 1,
	},
	{
		.name = "Bxtn HDMI Port",
		.stream_name = "Hdmi",
		.cpu_dai_name = "System Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:02:18.0",
		.init = NULL,
		.ignore_suspend = 1,
		.dynamic = 1,
	},
	{
		.name = "Bxtn Audio Refrence cap",
		.stream_name = "refcap",
		.cpu_dai_name = "Refrence Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:02:18.0",
		.init = NULL,
		.ignore_suspend = 1,
		.dynamic = 1,
	},
	/* CODEC<->CODEC link */
	{
		.name = "Bxtn Codec-Loop Port",
		.stream_name = "Bxtn Codec-Loop",
		.cpu_dai_name = "SSP0 Pin",
		.platform_name = "0000:02:18.0",
		.codec_name = "florida-codec",
		/* .params is set when card gets register, based on module param */
		.codec_dai_name = "florida-aif1",
		.dsp_loopback = true,
		.be_fixup = mrgfld_codec_loop_fixup,
	},
	{
		.name = "Bxtn Modem-Loop Port",
		.stream_name = "Bxtn Modem-Loop",
		.cpu_dai_name = "SSP2 Pin",
		.platform_name = "0000:02:18.0",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.params = &bxtn_florida_dai_params_modem,
		.dsp_loopback = true,
		.be_fixup = mrgfld_modem_loop_fixup,
	},
	{
		.name = "Bxtn BTFM-Loop Port",
		.stream_name = "Bxtn BTFM-Loop",
		.cpu_dai_name = "SSP1 Pin",
		.platform_name = "0000:02:18.0",
		.codec_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.params = &bxtn_florida_dai_params_bt,
		.dsp_loopback = true,
		.be_fixup = mrgfld_btfm_loop_fixup,
	},

	/* back ends */
	{
		.name = "SSP0-Codec",
		.be_id = 1,
		.cpu_dai_name = "SSP0 Pin",
		.codec_name = "florida-codec",
		.codec_dai_name = "florida-aif1",
		.platform_name = "0000:02:18.0",
		.be_hw_params_fixup = morg_florida_codec_fixup,
		.ignore_suspend = 1,
		.no_pcm = 1,
	},
	{
		.name = "SSP1-BTFM",
		.be_id = 2,
		.cpu_dai_name = "SSP1 Pin",
		.platform_name = "0000:02:18.0",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.ignore_suspend = 1,
		.be_hw_params_fixup = mrgfld_florida_btfm_fixup,
		.no_pcm = 1,
	},
	{
		.name = "SSP0-Modem",
		.be_id = 3,
		.cpu_dai_name = "snd-soc-dummy-dai",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "snd-soc-dummy",
		.ignore_suspend = 1,
		.be_hw_params_fixup = mrgfld_florida_modem_fixup,
		.no_pcm = 1,
	},
	{
		.name = "iDisp",
		.be_id = 4,
		.cpu_dai_name = "iDisp Pin",
		.codec_name = "codec#001.1",
		.codec_dai_name = "intel-hdmi-hif1",
		.platform_name = "0000:02:18.0",
		.ignore_suspend = 1,
		.no_pcm = 1,
	},
	{
		.name = "dmic01",
		.be_id = 5,
		.cpu_dai_name = "DMIC01 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:02:18.0",
		.ignore_suspend = 1,
		.no_pcm = 1,
	},
	{
		.name = "dmic23",
		.be_id = 6,
		.cpu_dai_name = "DMIC23 Pin",
		.codec_name = "snd-soc-dummy",
		.codec_dai_name = "snd-soc-dummy-dai",
		.platform_name = "0000:02:18.0",
		.ignore_suspend = 1,
		.no_pcm = 1,
	},
};

#ifdef CONFIG_PM_SLEEP
static int snd_morg_florida_prepare(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	return snd_soc_suspend(dev);
}

static void snd_morg_florida_complete(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	snd_soc_resume(dev);

	return;
}

static int snd_morg_florida_poweroff(struct device *dev)
{
	pr_debug("In %s\n", __func__);
	return snd_soc_poweroff(dev);
}
#else
#define snd_morg_florida_prepare NULL
#define snd_morg_florida_complete NULL
#define snd_morg_florida_poweroff NULL
#endif

#ifdef OSC_PMIC
static int morg_florida_mc_late_probe(struct snd_soc_card *card)
{
	int ret;
	struct snd_soc_dai *florida_dai = morg_florida_get_codec_dai(card, "florida-aif1");
	struct snd_soc_codec *florida_codec = morg_florida_get_codec(card);

	ret = snd_soc_dai_set_sysclk(florida_dai,  ARIZONA_CLK_SYSCLK, 0, 0);
	if (ret != 0) {
		dev_err(card->rtd[0].codec->dev, "Failed to set codec dai clk domain: %d\n", ret);
		return ret;
	}

	/*Configure SAMPLE_RATE_1 and ASYNC_SAMPLE_RATE_1 by default to
	48KHz these values can be changed in runtime by corresponding
	DAI hw_params callback */
	snd_soc_update_bits(florida_codec, ARIZONA_SAMPLE_RATE_1,
		ARIZONA_SAMPLE_RATE_1_MASK, 0x03);
	snd_soc_update_bits(florida_codec, ARIZONA_ASYNC_SAMPLE_RATE_1,
		ARIZONA_ASYNC_SAMPLE_RATE_MASK, 0x03);

	return 0;
}
#endif
/* SoC card */
static struct snd_soc_card snd_soc_card_morg = {
	.name = "florida-audio",
	.dai_link = morg_florida_msic_dailink,
	.num_links = ARRAY_SIZE(morg_florida_msic_dailink),
#ifdef OSC_PMIC
	.late_probe = morg_florida_mc_late_probe,
#endif
	.dapm_widgets = morg_widgets,
	.num_dapm_widgets = ARRAY_SIZE(morg_widgets),
	.dapm_routes = morg_map,
	.num_dapm_routes = ARRAY_SIZE(morg_map),
};

static int snd_morg_florida_mc_probe(struct platform_device *pdev)
{
	int i, ret_val = 0;
	struct mrgfld_mc_private *drv;
	struct snd_soc_dai_link *dai_link = NULL;

	pr_debug("Entry %s\n", __func__);
	drv = devm_kzalloc(&pdev->dev, sizeof(*drv), GFP_ATOMIC);
	if (!drv) {
		pr_err("allocation failed\n");
		return -ENOMEM;
	}

#ifdef OSC_PMIC
	/* ioremap the register */
	drv->osc_clk0_reg = devm_ioremap_nocache(&pdev->dev,
					MERR_OSC_CLKOUT_CTRL0_REG_ADDR,
					MERR_OSC_CLKOUT_CTRL0_REG_SIZE);
	if (!drv->osc_clk0_reg) {
		pr_err("osc clk0 ctrl ioremap failed\n");
		ret_val = -1;
		goto unalloc;
	}

	ret_val = intel_scu_ipc_ioread8(PMIC_ID_ADDR, &drv->pmic_id);
	if (ret_val) {
		pr_err("Error reading PMIC ID register\n");
		goto unalloc;
	}
#endif
	/* Override the params based on bt debug param */
	for (i = 0; i < ARRAY_SIZE(morg_florida_msic_dailink); i++) {
		if (!(strcmp("Bxtn Codec-Loop Port", morg_florida_msic_dailink[i].name))) {
			dai_link = &morg_florida_msic_dailink[i];
			pr_info("Codec Codec link found\n");
		}
	}
	if (dai_link) {
		switch (bt_debug) {
		case 0:
			dai_link->params =
				&bxtn_florida_dai_params_codec_default;
			pr_info("Default codec params selected\n");
			break;
		case 1:
			dai_link->params =
				&bxtn_florida_dai_params_codec_bt_wb;
			pr_info("BT Wideband codec params selected\n");
			break;
		case 2:
			dai_link->params =
				&bxtn_florida_dai_params_codec_bt_nb;
			pr_info("BT Narrowband codec params selected\n");
			break;
		default:
			dai_link->params =
				&bxtn_florida_dai_params_codec_default;
			pr_info("Default codec params selected\n");
		}
	}
	/* register the soc card */
	snd_soc_card_morg.dev = &pdev->dev;
	snd_soc_card_set_drvdata(&snd_soc_card_morg, drv);
	ret_val = snd_soc_register_card(&snd_soc_card_morg);
	if (ret_val) {
		pr_err("snd_soc_register_card failed %d\n", ret_val);
		goto unalloc;
	}
	platform_set_drvdata(pdev, &snd_soc_card_morg);
	pr_info("%s successful\n", __func__);
	return ret_val;

unalloc:
	devm_kfree(&pdev->dev, drv);
	return ret_val;
}

static int snd_morg_florida_mc_remove(struct platform_device *pdev)
{
	struct snd_soc_card *soc_card = platform_get_drvdata(pdev);
	struct mrgfld_mc_private *drv = snd_soc_card_get_drvdata(soc_card);

	pr_debug("In %s\n", __func__);

	devm_kfree(&pdev->dev, drv);
	snd_soc_card_set_drvdata(soc_card, NULL);
	snd_soc_unregister_card(soc_card);
	platform_set_drvdata(pdev, NULL);
	return 0;
}

const struct dev_pm_ops snd_morg_florida_mc_pm_ops = {
	.prepare = snd_morg_florida_prepare,
	.complete = snd_morg_florida_complete,
	.poweroff = snd_morg_florida_poweroff,
};

static struct platform_driver snd_morg_florida_mc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = "morg_florida",
#ifdef OSC_PMIC
		.pm = &snd_morg_florida_mc_pm_ops,
#endif
	},
	.probe = snd_morg_florida_mc_probe,
	.remove = snd_morg_florida_mc_remove,
};

static int snd_morg_florida_driver_init(void)
{
	pr_info("Morganfield Machine Driver morg_florida: wm8280 registerd\n");
	return platform_driver_register(&snd_morg_florida_mc_driver);
}
module_init(snd_morg_florida_driver_init);

static void snd_morg_florida_driver_exit(void)
{
	pr_debug("In %s\n", __func__);
	platform_driver_unregister(&snd_morg_florida_mc_driver);
}
module_exit(snd_morg_florida_driver_exit)

MODULE_DESCRIPTION("ASoC Morganfield Machine driver");
MODULE_AUTHOR("Samreen Nilofer <samreen.nilofer@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:morg_florida");
