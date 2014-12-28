/*
 *  soc-hda-pcm.c -ASOC HDA Platform driver file implementing PCM functionality
 *
 *  Copyright (C) 2014 Intel Corp
 *  Author:  Jeeja KP <jeeja.kp@intel.com>
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
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 */
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/hda_controller.h>
#include <sound/hda_dma.h>
#include <sound/hda_codec.h>
#include <sound/soc-hda-dma.h>
#include "soc-hda-controller.h"

inline struct azx_dev *
soc_azx_assign_device(struct azx *chip, struct snd_pcm_substream *substream)
{
	int dev, i, nums;
	struct azx_dev *res = NULL;
	/* make a non-zero unique key for the substream */
	int key = (substream->pcm->device << 16) | (substream->number << 2) |
			(substream->stream + 1);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		dev = chip->playback_index_offset;
		nums = chip->playback_streams;
	} else {
		dev = chip->capture_index_offset;
		nums = chip->capture_streams;
	}
	for (i = 0; i < nums; i++, dev++) {
		struct azx_dev *azx_dev = &chip->azx_dev[dev];
		if (!azx_dev->opened) {
			if (azx_dev->assigned_key == key) {
				azx_dev->opened = 1;
				azx_dev->assigned_key = key;
				return azx_dev;
			}
			if (!res)
				res = azx_dev;
		}
	}
	if (res) {
		res->opened = 1;
		res->assigned_key = key;
	}
	return res;
}

/* release the assigned stream */
static inline void soc_azx_release_device(struct azx_dev *azx_dev)
{
	azx_dev->opened = 0;
}

static inline struct azx_dev *
soc_azx_assign_link_device(struct azx *chip, bool is_playback)
{
	int i, nums, dev;
	struct azx_dev *link_dev = NULL;

	if (is_playback) {
		nums = chip->playback_streams;
		dev = chip->playback_index_offset;
	} else {
		nums = chip->capture_streams;
		dev = chip->capture_index_offset;
	}
	dsp_lock(link_dev);
	for (i = 0; i < nums; i++, dev++) {
		link_dev = &chip->link_dev[dev];
		if (!link_dev->opened && !dsp_is_locked(link_dev)) {
			link_dev->opened = 1;
			dsp_unlock(link_dev);
			return link_dev;
		}
	}
	dsp_unlock(link_dev);

	return link_dev;
}

/* release the assigned stream */
static inline void azx_link_release_device(struct azx_dev *link_dev)
{
	link_dev->opened = 0;
}

/* Get DMA id seperate 0-based numeration
 * for playback and capture streams
 */
static int azx_get_dma_id(struct azx *chip, struct azx_dev *azx_dev)
{
	return azx_dev->index >= chip->playback_index_offset ?
		azx_dev->index - chip->playback_index_offset :
		azx_dev->index - chip->capture_index_offset;
}

static struct azx *get_chip_ctx(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_pcm_substream_chip(substream);
	struct device *dev = rtd->cpu_dai->dev;
	struct azx *chip = dev_get_drvdata(dev);
	return chip;
}

static struct azx_dai_config *hda_be_get_dai_config(struct snd_soc_dai *dai)
{
	struct azx *chip = dev_get_drvdata(dai->dev);

	return &chip->dai_config[dai->id - 1];
}

/*
 * PCM support
 */
static int soc_hda_pcm_open(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct azx *chip = dev_get_drvdata(dai->dev);
	struct azx_dev *azx_dev;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long flags;
	struct snd_soc_hda_dma_params *dma_params;

	dev_dbg(chip->dev, "%s: %s\n", __func__, dai->name);
	mutex_lock(&chip->open_mutex);
	pm_runtime_get_sync(dai->dev);
	/*dsp is running*/
	if (!azx_is_dsp_running(chip)) {
		mutex_unlock(&chip->open_mutex);
		return -EAGAIN;
	}

	azx_dev = soc_azx_assign_device(chip, substream);
	if (azx_dev == NULL) {
		mutex_unlock(&chip->open_mutex);
		return -EBUSY;
	}
	if (chip->ppcap_offset)
		azx_dma_decouple(chip, azx_dev, true);
	azx_set_pcm_constrains(chip, runtime);

	/* disable WALLCLOCK timestamps for capture streams
	   until we figure out how to handle digital inputs */
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		runtime->hw.info &= ~SNDRV_PCM_INFO_HAS_WALL_CLOCK;

	spin_lock_irqsave(&chip->reg_lock, flags);
	azx_dev->substream = substream;
	azx_dev->running = 0;
	azx_dev->prepared = 0;
	spin_unlock_irqrestore(&chip->reg_lock, flags);
	runtime->private_data = azx_dev;

	dma_params = kzalloc(sizeof(*dma_params), GFP_KERNEL);
	dma_params->stream_tag = azx_dev->stream_tag;
	snd_soc_dai_set_dma_data(dai, substream, (void *)dma_params);

	dev_dbg(chip->dev, "stream tag set in dma params=%d\n",
				 dma_params->stream_tag);
	snd_pcm_set_sync(substream);
	mutex_unlock(&chip->open_mutex);
	return 0;
}
static int soc_hda_pcm_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = snd_pcm_substream_chip(substream);
	struct azx *chip = dev_get_drvdata(dai->dev);
	struct azx_dev *azx_dev = get_azx_dev(substream);
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned int format_val = 0;
	int ret = 0;
	struct snd_soc_hda_dma_params *dma_params;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	dev_dbg(chip->dev, "%s: %s\n", __func__, dai->name);
	if (azx_dev->prepared) {
		dev_dbg(chip->dev, "already stream is prepared - returning\n");
		return 0;
	}

	if (chip->ppcap_offset) {
		format_val = snd_hda_calc_stream_format(runtime->rate,

						runtime->channels,
						runtime->format,
						32, 0);

	} else {
		dma_params  = (struct snd_soc_hda_dma_params *)
			snd_soc_dai_get_dma_data(codec_dai, substream);

		if (!dma_params)
			format_val = dma_params->format;
	}

	dev_dbg(chip->dev, "stream_tag=%d formatvalue=%d\n",
				azx_dev->stream_tag, format_val);
	azx_stream_reset(chip, azx_dev);

	ret = azx_set_device_params(chip, substream, format_val);

	if (!ret)
		azx_dev->prepared = 1;
	return ret;
}

static void soc_hda_update_params(struct snd_pcm_hw_params *params,
					struct azx_dai_config *dai_config)
{
	dai_config->s_fmt = snd_pcm_format_width(params_format(params));
	dai_config->num_channels = params_channels(params);
	dai_config->sampling_rate = params_rate(params);
}

static int soc_hda_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct azx *chip = dev_get_drvdata(dai->dev);
	struct azx_dev *azx_dev = get_azx_dev(substream);
	int ret, dma_id;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct azx_dai_config *dai_config = hda_be_get_dai_config(dai);

	dev_dbg(chip->dev, "%s: %s\n", __func__, dai->name);
	ret = chip->ops->substream_alloc_pages(chip, substream,
					  params_buffer_bytes(params));

	memset(substream->runtime->dma_area, 0, params_buffer_bytes(params));

	dev_dbg(chip->dev, "format_val, rate=%d, ch=%d, format=%d\n",
			runtime->rate, runtime->channels, runtime->format);

	dma_id = azx_get_dma_id(chip, azx_dev);
	dev_dbg(chip->dev, "dma_id=%d\n", dma_id);

	if (chip->ppcap_offset) {
		hda_sst_set_copier_hw_params(dai, params,
			 substream->stream, true);
		hda_sst_set_copier_dma_id(dai, dma_id, substream->stream, true);
	}

	soc_hda_update_params(params, dai_config);

	dev_dbg(dai->dev, "%s: %s\n", __func__, dai->name);
	dev_dbg(dai->dev, "%s Sampling Format = %d\n",
				dai->name, dai_config->s_fmt);
	dev_dbg(dai->dev, "%s channels = %d\n",
				dai->name, dai_config->num_channels);
	dev_dbg(dai->dev, "%s rate = %d\n",
				dai->name, dai_config->sampling_rate);

	return ret;
}

static void soc_hda_pcm_close(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct azx *chip = dev_get_drvdata(dai->dev);
	struct azx_dev *azx_dev = get_azx_dev(substream);
	unsigned long flags;
	struct snd_soc_hda_dma_params *dma_params;

	dev_dbg(chip->dev, "%s: %s\n", __func__, dai->name);
	mutex_lock(&chip->open_mutex);
	spin_lock_irqsave(&chip->reg_lock, flags);
	azx_dev->substream = NULL;
	azx_dev->running = 0;
	spin_unlock_irqrestore(&chip->reg_lock, flags);
	soc_azx_release_device(azx_dev);
	dma_params  = (struct snd_soc_hda_dma_params *)
			snd_soc_dai_get_dma_data(dai, substream);
	pm_runtime_put_sync(dai->dev);
	kfree(dma_params);
	mutex_unlock(&chip->open_mutex);
}

static int soc_hda_pcm_hw_free(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct azx *chip = dev_get_drvdata(dai->dev);
	struct azx_dev *azx_dev = get_azx_dev(substream);

	dev_dbg(chip->dev, "%s: %s\n", __func__, dai->name);

	/*reset BDL address */
	azx_reset_device(chip, azx_dev);

	azx_dev->prepared = 0;

	return chip->ops->substream_free_pages(chip, substream);
}

static int hda_be_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	dev_dbg(dai->dev, "%s: %s\n", __func__, dai->name);
	/*will set the be copier spefic config here
	now the default values will be taken from DFW */
	return 0;
}

static int hda_be_ssp_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct azx_dai_config *dai_config = hda_be_get_dai_config(dai);
	struct azx_ssp_dai_config *ssp_cfg = &dai_config->ssp_dai_config;

	soc_hda_update_params(params, dai_config);

	if (strncmp(dai->name, "SSP0 Pin", strlen(dai->name)) == 0)
		ssp_cfg->i2s_instance = 0;
	else if (strncmp(dai->name, "SSP1 Pin", strlen(dai->name)) == 0)
		ssp_cfg->i2s_instance = 1;
	else if (strncmp(dai->name, "SSP2 Pin", strlen(dai->name)) == 0)
		ssp_cfg->i2s_instance = 2;

	dai_config->dai_type = AZX_DAI_TYPE_SSP;
	dev_dbg(dai->dev, "%s: %s\n", __func__, dai->name);
	dev_dbg(dai->dev, "%s Sampling Format = %d\n",
				dai->name, dai_config->s_fmt);
	dev_dbg(dai->dev, "%s channels = %d\n",
				dai->name, dai_config->num_channels);
	dev_dbg(dai->dev, "%s rate = %d\n",
				dai->name, dai_config->sampling_rate);

	return 0;
}

static int hda_be_ssp_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct azx_dai_config *cfg = hda_be_get_dai_config(dai);
	struct azx_ssp_dai_config *ssp_cfg = &cfg->ssp_dai_config;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
		ssp_cfg->ssp_mode = HDA_SSP_MODE_DSP_A;
		break;
	case SND_SOC_DAIFMT_DSP_B:
		ssp_cfg->ssp_mode = HDA_SSP_MODE_DSP_B;
		break;
	case SND_SOC_DAIFMT_I2S:
		ssp_cfg->ssp_mode = HDA_SSP_MODE_I2S;
		break;
	default:
		dev_err(dai->dev, "%s: Unsupport SSP mode for dai %s\n",
				__func__, dai->name);
		return -EINVAL;
	}
	dev_dbg(dai->dev, "%s SSP Mode = %d\n", dai->name, ssp_cfg->ssp_mode);
	switch (fmt & SND_SOC_DAIFMT_MASTER_MASK) {
	case SND_SOC_DAIFMT_CBS_CFS:
		ssp_cfg->fs_slave = 1;
		ssp_cfg->bclk_slave = 1;
		break;
	case SND_SOC_DAIFMT_CBS_CFM:
		ssp_cfg->fs_slave = 0;
		ssp_cfg->bclk_slave = 1;
		break;
	case SND_SOC_DAIFMT_CBM_CFS:
		ssp_cfg->fs_slave = 1;
		ssp_cfg->bclk_slave = 0;
		break;
	case SND_SOC_DAIFMT_CBM_CFM:
		ssp_cfg->fs_slave = 0;
		ssp_cfg->bclk_slave = 0;
		break;
	default:
		dev_err(dai->dev, "Unsupported master mode for dai %s\n",
								dai->name);
		return -EINVAL;

	}
	dev_dbg(dai->dev, "%s Master mode fs = %d bclk= %d\n", dai->name,
		ssp_cfg->fs_slave, ssp_cfg->bclk_slave);
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		ssp_cfg->fs_invert = 0;
		ssp_cfg->bclk_invert = 0;
		break;
	case SND_SOC_DAIFMT_IB_IF:
		ssp_cfg->fs_invert = 1;
		ssp_cfg->bclk_invert = 1;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		ssp_cfg->fs_invert = 0;
		ssp_cfg->bclk_invert = 1;
		break;
	case SND_SOC_DAIFMT_NB_IF:
		ssp_cfg->fs_invert = 1;
		ssp_cfg->bclk_invert = 0;
		break;
	default:
		dev_err(dai->dev, "%s: Unsupport fs and blck invert mode for dai %s\n",
				__func__, dai->name);
		return -EINVAL;
	}
	dev_dbg(dai->dev, "%s Clock inversion fs = %d bclk= %d\n", dai->name,
		ssp_cfg->fs_invert, ssp_cfg->bclk_invert);
	return 0;
}

static int hda_be_ssp_set_tdm_slot(struct snd_soc_dai *dai,
		unsigned int tx_mask, unsigned int rx_mask,
		int slots, int slot_width)
{
	struct azx_dai_config *cfg = hda_be_get_dai_config(dai);
	struct azx_ssp_dai_config *ssp_cfg = &cfg->ssp_dai_config;

	if (tx_mask || rx_mask) {
		dev_err(dai->dev, "%s Channel remapping not supported %s\n",
			__func__, dai->name);
		return -EINVAL;
	}
	ssp_cfg->slots = slots;
	ssp_cfg->slot_width = slot_width;

	dev_dbg(dai->dev, "%s num of slots = %d slot_width = %d\n", dai->name,
			ssp_cfg->slots, ssp_cfg->slot_width);

	return 0;
}

static int hda_be_ssp_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct azx *chip = dev_get_drvdata(dai->dev);

	pm_runtime_get_sync(dai->dev);
	/*dsp is running*/
	if (azx_is_dsp_running(chip))
		return 0;
	return -EAGAIN;
}

static void hda_be_ssp_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	pm_runtime_put_sync(dai->dev);
}

static int hda_be_dmic_prepare(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct snd_pcm_hw_params params = {0};
	struct snd_interval *channels, *rate;
	struct azx *chip = dev_get_drvdata(dai->dev);

	dev_dbg(chip->dev, "%s: %s\n", __func__, dai->name);

	channels = hw_param_interval(&params, SNDRV_PCM_HW_PARAM_CHANNELS);
	channels->min = channels->max = substream->runtime->channels;
	rate = hw_param_interval(&params, SNDRV_PCM_HW_PARAM_RATE);
	rate->min = rate->max = substream->runtime->rate;
	snd_mask_set(&params.masks[SNDRV_PCM_HW_PARAM_FORMAT -
					SNDRV_PCM_HW_PARAM_FIRST_MASK],
					substream->runtime->format);
	hda_sst_set_be_dmic_config(dai, &params, substream->stream);
	return 0;
}

static int hda_be_dmic_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct azx *chip = dev_get_drvdata(dai->dev);

	pm_runtime_get_sync(dai->dev);
	/*dsp is running*/
	if (azx_is_dsp_running(chip))
		return 0;
	return -EAGAIN;
}

static void hda_be_dmic_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	pm_runtime_put_sync(dai->dev);
}

static int soc_hda_pcm_trigger(struct snd_pcm_substream *substream, int cmd,
		struct snd_soc_dai *dai)
{
	struct azx *chip = dev_get_drvdata(dai->dev);

	dev_dbg(chip->dev, "In %s cmd=%d\n", __func__, cmd);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		hda_sst_set_fe_pipeline_state(dai, true, substream->stream);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
		hda_sst_set_fe_pipeline_state(dai, false, substream->stream);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int soc_hda_be_link_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = snd_pcm_substream_chip(substream);
	struct azx_dev *link_dev;
	struct azx *chip = dev_get_drvdata(dai->dev);
	int dma_id;
	struct snd_soc_hda_dma_params *dma_params;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	pr_debug("%s\n", __func__);
	link_dev = soc_azx_assign_link_device(chip, substream);
	if (link_dev == NULL) {
		mutex_unlock(&chip->open_mutex);
		return -EBUSY;
	}

	snd_soc_dai_set_dma_data(dai, substream, (void *)link_dev);

	/*set the stream tag in the codec dai dma params */
	dma_params  = (struct snd_soc_hda_dma_params *)
			snd_soc_dai_get_dma_data(codec_dai, substream);
	dma_params->stream_tag =  link_dev->stream_tag;
	snd_soc_dai_set_dma_data(codec_dai, substream, (void *)dma_params);
	dma_id = azx_get_dma_id(chip, link_dev);
	hda_sst_set_copier_dma_id(dai, dma_id, substream->stream, false);

	return 0;
}

int find_link_index(struct azx *chip, struct snd_soc_codec *codec)
{
	int i = 0, j = 0;
	for (i = 0; i <= chip->link_count; i++) {
		for (j = 0; j < 16; j++) {
			if (strlen(chip->azx_link[i].codec_name[j]) == 0)
				break;
			if (!strcmp(chip->azx_link[i].codec_name[j], codec->name))
				return i;
		}
	}
	return -EINVAL;
}

static int soc_hda_be_link_pcm_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = snd_pcm_substream_chip(substream);
	struct azx *chip = dev_get_drvdata(dai->dev);
	struct azx_dev *link_dev = snd_soc_dai_get_dma_data(dai, substream);
	unsigned int format_val;
	int ret = 0;
	int index = 0;
	struct snd_soc_hda_dma_params *dma_params;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;
	struct snd_pcm_hw_params *params;
	struct snd_interval *channels, *rate;
	u32 __iomem *addr;
	u32 value;

	dev_dbg(chip->dev, "%s: %s\n", __func__, dai->name);
	if (link_dev->prepared) {
		dev_dbg(chip->dev, "already stream is prepared - returning\n");
		return 0;
	}
	params  = devm_kzalloc(chip->dev, sizeof(*params), GFP_KERNEL);
	if (params == NULL)
		return -ENOMEM;

	channels = hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS);
	channels->min = channels->max = substream->runtime->channels;
	rate = hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE);
	rate->min = rate->max = substream->runtime->rate;
	snd_mask_set(&params->masks[SNDRV_PCM_HW_PARAM_FORMAT -
					SNDRV_PCM_HW_PARAM_FIRST_MASK],
					substream->runtime->format);


	dma_params  = (struct snd_soc_hda_dma_params *)
			snd_soc_dai_get_dma_data(codec_dai, substream);
	format_val = dma_params->format;
	dev_dbg(chip->dev, "stream_tag=%d formatvalue=%d codec_dai_name=%s\n",
				link_dev->stream_tag, format_val, codec_dai->name);
	azx_link_dma_reset(chip, link_dev);

	ret = azx_link_dma_set_stream_id(chip, link_dev);
	if (ret) {
		dev_err(chip->dev, "Failed to set stream tag to link DMA");
		return -EIO;
	}
	ret = azx_link_dma_set_format(chip, link_dev, format_val);
	if (ret) {
		dev_err(chip->dev, "Failed to set format to link DMA");
		return -EIO;
	}

	/*TODO: verify the mapping */
	index = find_link_index(chip, rtd->codec);
	if (index < 0)
		return index;
	dev_dbg(chip->dev, "using link: %d\n", index);
	addr = (chip->remap_addr + chip->azx_link[index].losidv_offset);
	value = (azx_ml_readl(chip, addr) | (1 << link_dev->stream_tag));
	azx_ml_writel(chip, addr, value);
	if (!ret)
		link_dev->prepared = 1;
	return ret;
}

static int soc_hda_be_link_pcm_trigger(struct snd_pcm_substream *substream,
	int cmd, struct snd_soc_dai *dai)
{
	struct azx *chip = dev_get_drvdata(dai->dev);
	struct azx_dev *link_dev = snd_soc_dai_get_dma_data(dai, substream);

	dev_dbg(chip->dev, "In %s cmd=%d\n", __func__, cmd);
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		azx_link_dma_run_ctrl(chip, link_dev, true);
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
		azx_link_dma_run_ctrl(chip, link_dev, false);
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int soc_hda_be_link_hw_free(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct azx *chip = dev_get_drvdata(dai->dev);
	struct snd_soc_pcm_runtime *rtd = snd_pcm_substream_chip(substream);
	struct azx_dev *link_dev = snd_soc_dai_get_dma_data(dai, substream);
	u32 __iomem *addr;
	u32 value;
	int index = 0;

	dev_dbg(chip->dev, "%s: %s\n", __func__, dai->name);

	link_dev->stream_tag = 0;
	link_dev->prepared = 0;

	/*TODO: verify the mapping */
	index = find_link_index(chip, rtd->codec);
	if (index < 0)
		return index;
	dev_dbg(chip->dev, "using link: %d\n", index);

	addr = (chip->remap_addr + chip->azx_link[index].losidv_offset);
	value = (azx_ml_readl(chip, addr) & ~(1 << link_dev->stream_tag));
	azx_ml_writel(chip, addr, value);
	azx_link_release_device(link_dev);
	return 0;
}

static int soc_hda_be_link_startup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct azx *chip = dev_get_drvdata(dai->dev);

	pm_runtime_get_sync(dai->dev);
	/*dsp is running*/
	if (azx_is_dsp_running(chip))
		return 0;
	return -EAGAIN;
}

static void soc_hda_be_link_shutdown(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	pm_runtime_put_sync(dai->dev);
}

static struct snd_soc_dai_ops hda_pcm_dai_ops = {
	.startup = soc_hda_pcm_open,
	.shutdown = soc_hda_pcm_close,
	.prepare = soc_hda_pcm_prepare,
	.hw_params = soc_hda_pcm_hw_params,
	.hw_free = soc_hda_pcm_hw_free,
	.trigger = soc_hda_pcm_trigger,
};

static struct snd_soc_dai_ops hda_be_dmic_dai_ops = {
	.startup = hda_be_dmic_startup,
	.prepare = hda_be_dmic_prepare,
	.shutdown = hda_be_dmic_shutdown,
};

static struct snd_soc_dai_ops hda_be_dai_ops = {
	.hw_params = hda_be_hw_params,
};

static struct snd_soc_dai_ops hda_be_ssp_dai_ops = {
	.startup = hda_be_ssp_startup,
	.hw_params = hda_be_ssp_hw_params,
	.shutdown = hda_be_ssp_shutdown,
	.set_fmt = hda_be_ssp_set_fmt,
	.set_tdm_slot = hda_be_ssp_set_tdm_slot,
};

static struct snd_soc_dai_ops hda_be_link_dai_ops = {
	.startup = soc_hda_be_link_startup,
	.prepare = soc_hda_be_link_pcm_prepare,
	.hw_params = soc_hda_be_link_hw_params,
	.hw_free = soc_hda_be_link_hw_free,
	.trigger = soc_hda_be_link_pcm_trigger,
	.shutdown = soc_hda_be_link_shutdown,
};

static struct snd_soc_dai_driver soc_hda_platform_dai[] = {
{
	.name = "System Pin",
	.ops = &hda_pcm_dai_ops,
	.playback = {
		.stream_name = "System Playback",
		.channels_min = HDA_MONO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_8000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "System Capture",
		.channels_min = HDA_MONO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
},
{
	.name = "Refrence Pin",
	.ops = &hda_pcm_dai_ops,
	.capture = {
		.stream_name = "Refrence Capture",
		.channels_min = HDA_MONO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
},
{
	.name = "Deepbuffer iDisp Pin",
	.ops = &hda_pcm_dai_ops,
	.playback = {
		.stream_name = "Deepbuffer iDisp Playback",
		.channels_min = HDA_MONO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_44100|SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = "Deepbuffer Pin",
	.ops = &hda_pcm_dai_ops,
	.playback = {
		.stream_name = "Deepbuffer Playback",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = "LowLatency Pin",
	.ops = &hda_pcm_dai_ops,
	.playback = {
		.stream_name = "Low Latency Playback",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = "Compress Pin",
	.compress_dai = 1,
	.playback = {
		.stream_name = "Compress Playback",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = "VOIP Pin",
	.ops = &hda_pcm_dai_ops,
	.playback = {
		.stream_name = "VOIP Playback",
		.channels_min = HDA_MONO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "VOIP Capture",
		.channels_min = HDA_MONO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
/*BE CPU  Dais */
{
	.name = "SSP0 Pin",
	.ops = &hda_be_ssp_dai_ops,
	.playback = {
		.stream_name = "ssp0 Tx",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "ssp0 Rx",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = "SSP1 Pin",
	.ops = &hda_be_ssp_dai_ops,
	.playback = {
		.stream_name = "ssp1 Tx",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "ssp1 Rx",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = "SSP2 Pin",
	.ops = &hda_be_ssp_dai_ops,
	.playback = {
		.stream_name = "ssp2 Tx",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "ssp2 Rx",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = "iDisp Pin",
	.ops = &hda_be_link_dai_ops,
	.playback = {
		.stream_name = "iDisp Tx",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_8000|SNDRV_PCM_RATE_16000|SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = "DMIC01 Pin",
	.ops = &hda_be_dmic_dai_ops,
	.capture = {
		.stream_name = "DMIC01 Rx",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
},
{
	.name = "DMIC23 Pin",
	.ops = &hda_be_dmic_dai_ops,
	.capture = {
		.stream_name = "DMIC23 Rx",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_48000 | SNDRV_PCM_RATE_16000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE,
	},
},
{
	.name = "Slimbus Pin",
	.ops = &hda_be_dai_ops,
	.playback = {
		.stream_name = "Slimbus Tx",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "Slimbus Rx",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
{
	.name = "HD-Codec Pin",
	.ops = &hda_be_link_dai_ops,
	.playback = {
		.stream_name = "HD-Codec Tx",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "HD-Codec Rx",
		.channels_min = HDA_STEREO,
		.channels_max = HDA_STEREO,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
};

static int soc_hda_platform_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *dai_link = rtd->dai_link;

	dev_dbg(rtd->cpu_dai->dev, "In %s:%s\n", __func__,
					dai_link->cpu_dai_name);

	runtime = substream->runtime;
	snd_soc_set_runtime_hwparams(substream, &azx_pcm_hw);
	return 0;
}

static int soc_hda_platform_close(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *dai_link = rtd->dai_link;

	dev_dbg(rtd->cpu_dai->dev, "In %s:%s\n", __func__,
					dai_link->cpu_dai_name);

	return 0;
}

static cycle_t soc_hda_cc_read(const struct cyclecounter *cc)
{
	struct azx_dev *azx_dev =
		container_of(cc, struct azx_dev, azx_cc);
	struct snd_pcm_substream *substream = azx_dev->substream;
	struct azx *chip = get_chip_ctx(substream);

	return azx_readl(chip, WALLCLK);
}

void azx_timecounter_init(struct snd_pcm_substream *substream,
					bool force, cycle_t last)
{
	struct azx_dev *azx_dev = get_azx_dev(substream);
	struct timecounter *tc = &azx_dev->azx_tc;
	struct cyclecounter *cc = &azx_dev->azx_cc;
	u64 nsec;

	cc->read = soc_hda_cc_read;
	cc->mask = CLOCKSOURCE_MASK(32);

	/*
	* Converting from 24 MHz to ns means applying a 125/3 factor.
	* To avoid any saturation issues in intermediate operations,
	* the 125 factor is applied first. The division is applied
	* last after reading the timecounter value.
	* Applying the 1/3 factor as part of the multiplication
	* requires at least 20 bits for a decent precision, however
	* overflows occur after about 4 hours or less, not a option.
	*/

	cc->mult = 125; /* saturation after 195 years */
	cc->shift = 0;

	nsec = 0; /* audio time is elapsed time since trigger */
	timecounter_init(tc, cc, nsec);
	if (force)
		/*
		* force timecounter to use predefined value,
		* used for synchronized starts
		*/
		tc->cycle_last = last;
	}

static int soc_hda_platform_pcm_trigger(struct snd_pcm_substream *substream,
					int cmd)
{
	struct azx *chip = get_chip_ctx(substream);
	struct azx_dev *azx_dev;
	struct snd_pcm_substream *s;
	int rstart = 0, start, nsync = 0, sbits = 0;
	int nwait, timeout;
	unsigned long cookie;

	azx_dev = get_azx_dev(substream);
	/*FIXME trace_azx_pcm_trigger(chip, azx_dev, cmd);*/

	dev_dbg(chip->dev, "In %s cmd=%d\n", __func__, cmd);
	if (!azx_dev->prepared)
		return -EPIPE;
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		rstart = 1;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		start = 1;
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
		start = 0;
		break;
	default:
	return -EINVAL;
	}
	snd_pcm_group_for_each_entry(s, substream) {
		if (s->pcm->card != substream->pcm->card)
			continue;
		azx_dev = get_azx_dev(s);
		sbits |= 1 << azx_dev->index;
		nsync++;
		snd_pcm_trigger_done(s, substream);
	}

	spin_lock_irqsave(&chip->reg_lock, cookie);

	/* first, set SYNC bits of corresponding streams */
	if (chip->driver_caps & AZX_DCAPS_OLD_SSYNC)
		azx_writel(chip, OLD_SSYNC,
			azx_readl(chip, OLD_SSYNC) | sbits);
	else
		azx_writel(chip, SSYNC, azx_readl(chip, SSYNC) | sbits);

	snd_pcm_group_for_each_entry(s, substream) {
		if (s->pcm->card != substream->pcm->card)
			continue;
		azx_dev = get_azx_dev(s);
		if (start) {
			azx_dev->start_wallclk = azx_readl(chip, WALLCLK);
			if (!rstart)
				azx_dev->start_wallclk -=
						azx_dev->period_wallclk;
			azx_stream_start(chip, azx_dev);
		} else {
			azx_stream_stop(chip, azx_dev);
		}
		azx_dev->running = start;
	}
	spin_unlock_irqrestore(&chip->reg_lock, cookie);
	if (start) {
		/* wait until all FIFOs get ready */
		for (timeout = 5000; timeout; timeout--) {
			nwait = 0;
			snd_pcm_group_for_each_entry(s, substream) {
				if (s->pcm->card != substream->pcm->card)
					continue;
				azx_dev = get_azx_dev(s);
				if (!(azx_sd_readb(chip, azx_dev, SD_STS) &
				      SD_STS_FIFO_READY))
					nwait++;
			}
			if (!nwait)
				break;
			cpu_relax();
		}
	} else {
		/* wait until all RUN bits are cleared */
		for (timeout = 5000; timeout; timeout--) {
			nwait = 0;
			snd_pcm_group_for_each_entry(s, substream) {
				if (s->pcm->card != substream->pcm->card)
					continue;
				azx_dev = get_azx_dev(s);
				if (azx_sd_readb(chip, azx_dev, SD_CTL) &
				    SD_CTL_DMA_START)
					nwait++;
			}
			if (!nwait)
				break;
			cpu_relax();
		}
	}
	spin_lock_irqsave(&chip->reg_lock, cookie);
	/* reset SYNC bits */
	if (chip->driver_caps & AZX_DCAPS_OLD_SSYNC)
		azx_writel(chip, OLD_SSYNC,
				azx_readl(chip, OLD_SSYNC) & ~sbits);
	else
		azx_writel(chip, SSYNC, azx_readl(chip, SSYNC) & ~sbits);

	if (start) {
		azx_timecounter_init(substream, 0, 0);
		if (nsync > 1) {
			cycle_t cycle_last;

			/* same start cycle for master and group */
			azx_dev = get_azx_dev(substream);
			cycle_last = azx_dev->azx_tc.cycle_last;
			snd_pcm_group_for_each_entry(s, substream) {
				if (s->pcm->card != substream->pcm->card)
					continue;
				azx_timecounter_init(s, 1, cycle_last);
			}
		}
	}
	spin_unlock_irqrestore(&chip->reg_lock, cookie);
	return 0;
}

static int soc_hda_platform_pcm_dsp_trigger(struct snd_pcm_substream *substream,
		int cmd)
{
	struct azx *chip = get_chip_ctx(substream);
	struct azx_dev *azx_dev = get_azx_dev(substream);
	int rstart = 0, start;
	unsigned long cookie;

	dev_dbg(chip->dev, "In %s cmd=%d\n", __func__, cmd);
	if (!azx_dev->prepared)
		return -EPIPE;
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		rstart = 1;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		start = 1;
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
		start = 0;
		break;
	default:
	return -EINVAL;
	}

	spin_lock_irqsave(&chip->reg_lock, cookie);

	if (start) {
		azx_dev->start_wallclk = azx_readl(chip, WALLCLK);
		if (!rstart)
			azx_dev->start_wallclk -= azx_dev->period_wallclk;
		azx_stream_start(chip, azx_dev);
	} else {
			azx_stream_stop(chip, azx_dev);
	}
	azx_dev->running = start;

	if (start)
		azx_timecounter_init(substream, 0, 0);
	spin_unlock_irqrestore(&chip->reg_lock, cookie);
	return 0;
}

static snd_pcm_uframes_t soc_hda_platform_pcm_pointer
			(struct snd_pcm_substream *substream)
{
	struct azx *chip = get_chip_ctx(substream);
	struct azx_dev *azx_dev = get_azx_dev(substream);

	dev_dbg(chip->dev, "In %s\n", __func__);

	return bytes_to_frames(substream->runtime,
			       azx_get_position(chip, azx_dev, false));
}

#ifdef CONFIG_X86
static int soc_hda_platform_pcm_mmap(struct snd_pcm_substream *substream,
			struct vm_area_struct *area)
{
	struct azx *chip = get_chip_ctx(substream);

#ifdef CONFIG_X86
	if (!azx_snoop(chip))
		area->vm_page_prot = pgprot_writecombine(area->vm_page_prot);
#endif
	return snd_pcm_lib_default_mmap(substream, area);
}
#else
#define soc_azx_pcm_mmap	NULL
#endif

static int soc_hda_platform_get_wallclock_tstamp(struct snd_pcm_substream *substream,
				struct timespec *ts)
{
	struct azx_dev *azx_dev = get_azx_dev(substream);
	u64 nsec;

	nsec = timecounter_read(&azx_dev->azx_tc);
	nsec = div_u64(nsec, 3); /* can be optimized */
	/*FIXME nsec = azx_adjust_codec_delay(substream, nsec);*/

	*ts = ns_to_timespec(nsec);

	return 0;
}

static struct snd_pcm_ops soc_hda_platform_ops = {
	.open = soc_hda_platform_open,
	.close = soc_hda_platform_close,
	.ioctl = snd_pcm_lib_ioctl,
	.trigger = soc_hda_platform_pcm_trigger,
	.pointer = soc_hda_platform_pcm_pointer,
	.wall_clock = soc_hda_platform_get_wallclock_tstamp,
	.mmap = soc_hda_platform_pcm_mmap,
	.page = snd_pcm_sgbuf_ops_page,
};

static void soc_hda_pcm_free(struct snd_pcm *pcm)
{
	snd_pcm_lib_preallocate_free_for_all(pcm);
}

#define MAX_PREALLOC_SIZE	(32 * 1024 * 1024)

static int soc_hda_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_dai *dai = rtd->cpu_dai;
	struct azx *chip = dev_get_drvdata(dai->dev);
	struct snd_pcm *pcm = rtd->pcm;
	unsigned int size;

	int retval = 0;

	dev_dbg(dai->dev, "In %s\n", __func__);
	if (dai->driver->playback.channels_min ||
			dai->driver->capture.channels_min) {
			/* buffer pre-allocation */
			size = CONFIG_SND_HDA_PREALLOC_SIZE * 1024;
			if (size > MAX_PREALLOC_SIZE)
				size = MAX_PREALLOC_SIZE;
			retval = snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV_SG,
					      snd_dma_pci_data(chip->pci),
					      size, MAX_PREALLOC_SIZE);
		if (retval) {
			dev_err(dai->dev, "dma buffer allocationf fail\n");
			return retval;
		}
	}
	return retval;
}

static int soc_hda_soc_probe(struct snd_soc_platform *platform)
{
	struct azx *chip = dev_get_drvdata(platform->dev);
	int ret = 0;

	dev_dbg(chip->dev, "Enter:%s\n", __func__);

	if (chip->ppcap_offset)
		ret = hda_sst_dsp_control_init(platform, chip);
	return ret;
}

static int soc_hda_soc_remove(struct snd_soc_platform *platform)
{
	dev_dbg(platform->dev, "Enter:%s\n", __func__);
	return 0;
}

static struct snd_soc_platform_driver soc_hda_platform_drv  = {
	.probe		= soc_hda_soc_probe,
	.remove		= soc_hda_soc_remove,
	.ops		= &soc_hda_platform_ops,
	.pcm_new	= soc_hda_pcm_new,
	.pcm_free	= soc_hda_pcm_free,
	.read		= soc_hda_soc_read,
	.write		= soc_hda_soc_write,
};

static const struct snd_soc_component_driver soc_hda_component = {
	.name           = "pcm",
};


int soc_hda_platform_register(struct device *dev)
{
	struct hda_platform_info *pinfo;
	struct azx *chip = dev_get_drvdata(dev);
	struct snd_soc_azx *sazx = container_of(chip,
				struct snd_soc_azx, hda_azx);
	int i, ret = 0;
	struct snd_soc_dai_driver *dai_driver;

	pinfo  = devm_kzalloc(chip->dev, sizeof(*pinfo), GFP_KERNEL);
	if (pinfo == NULL) {
		dev_err(chip->dev, "kzalloc failed\n");
		return -ENOMEM;
	}

	chip->dai_config = devm_kzalloc(chip->dev, sizeof(struct azx_dai_config) *
				(ARRAY_SIZE(soc_hda_platform_dai)), GFP_KERNEL);

	if (chip->dai_config == NULL) {
		dev_err(chip->dev, "kzalloc failed\n");
		return -ENOMEM;
	}
	pinfo->dev = dev;
	INIT_LIST_HEAD(&pinfo->ppl_list);
	INIT_LIST_HEAD(&pinfo->ppl_start_list);

	sazx->pinfo  = pinfo;
	if (chip->ppcap_offset)
		soc_hda_platform_ops.trigger = soc_hda_platform_pcm_dsp_trigger;

	ret = snd_soc_register_platform(dev,
					 &soc_hda_platform_drv);
	if (ret) {
		dev_err(dev, "registering soc platform failed\n");
		return ret;
	}
	for (i = 0; i < ARRAY_SIZE(soc_hda_platform_dai); i++) {
		dai_driver = &soc_hda_platform_dai[i];
		dai_driver->id = i + 1;
	}
	ret = snd_soc_register_component(dev, &soc_hda_component,
				soc_hda_platform_dai,
				ARRAY_SIZE(soc_hda_platform_dai));
	if (ret) {
		dev_err(dev, "registering cpu dais failed\n");
		snd_soc_unregister_platform(dev);
	}

	dev_dbg(chip->dev, "In%s registration successful\n", __func__);

	return ret;

}

int soc_hda_platform_unregister(struct device *dev)
{
	snd_soc_unregister_component(dev);
	snd_soc_unregister_platform(dev);
	return 0;
}
