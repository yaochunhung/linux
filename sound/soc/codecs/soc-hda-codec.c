/*
 * ASoC High Definition Audio Codec interface Definitions
 *
 * Copyright (c) 2014 Intel Corporation
 *
 * Author(s): Lakshmi Vinnakota <lakshmi.n.vinnakota@intel.com>
 *
 *
 *  This driver is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This driver is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <sound/core.h>
#include <sound/jack.h>
#include <sound/pcm.h>
#include <sound/asoundef.h>
#include <sound/hda_codec.h>
#include <sound/hda_auto_parser.h>
#include <sound/hda_controls.h>
#include <sound/hda_jack.h>
#include <sound/hda_generic.h>
#include "soc-hda-codec.h"

/*
 * codec destructor
 */
void snd_soc_hda_codec_free(struct snd_soc_hda_codec *codec)
{
	if (!codec)
		return;
	snd_hda_codec_free(&codec->hdac);

	if (codec->ops.free)
		codec->ops.free(codec);
	kfree(codec);
}

/*
 * Hook for executing codec commands on the hda bus
 */
static int soc_hda_exec_verb(struct hda_codec *codec, unsigned int cmd, int flags,
		unsigned int *res)
{
	struct snd_soc_hda_codec *shc = container_of(codec,
						struct snd_soc_hda_codec, hdac);

	return snd_soc_codec_exec_verb(shc->hdev, cmd, flags, res);
}

/*
 * Hook for executing snd_pci_quirks lookup
 */
static const struct snd_pci_quirk *
soc_quirk_lookup(struct hda_codec *codec, const struct snd_pci_quirk *list)
{
	struct snd_soc_hda_codec *shc = container_of(codec,
						struct snd_soc_hda_codec, hdac);

	return snd_soc_hda_quirk_lookup(shc->hdev, list);
}

/**
 * snd_soc_hda_codec_new - create a ASoC HDA codec
 * @hdev: the pointer to the hda device for the codec
 * @codec_addr: the codec address
 * @codecp: the pointer to store the generated codec
 *
 * Returns 0 if successful, or a negative error code.
 */
int snd_soc_hda_codec_new(struct snd_soc_hda_device *hdev, unsigned int codec_addr,
				struct snd_soc_hda_codec **codecp)
{
	struct snd_soc_hda_codec *codec;
	int err;

	if (snd_BUG_ON(codec_addr > HDA_MAX_CODEC_ADDRESS))
		return -EINVAL;

	if ((codecp == NULL) || (hdev == NULL))
		return -EINVAL;

	codec = kzalloc(sizeof(*codec), GFP_KERNEL);
	if (codec == NULL) {
		snd_printk(KERN_ERR "can't allocate struct snd_soc_hda_codec\n");
		return -ENOMEM;
	}

	codec->hdev = hdev;
	codec->hdac.dev = &hdev->dev;
	codec->map_cur_idx = 0;
	codec->hdac.exec_cmd = &soc_hda_exec_verb;
	codec->hdac.quirk_lookup = &soc_quirk_lookup;

	err = snd_hda_codec_init(codec_addr, &codec->hdac);

	if (err) {
		kfree(codec);
		return err;
	}
	codec->hdac.modelname = kstrdup("generic", GFP_KERNEL);

	*codecp = codec;
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_hda_codec_new);

/**
 * snd_soc_hda_codec_configure - configure the HD-audio codec
 * @codec: the HDA codec
 *
 * Start parsing of the given codec tree and (re-)initialize the whole
 * patch instance.
 *
 * Returns 0 if successful or a negative error code.
 */
int snd_soc_hda_codec_configure(struct snd_soc_hda_codec *codec)
{
	int err;

	/* call the default parser */
	err = snd_hda_codec_configure(&codec->hdac);
	if (err < 0)
		printk(KERN_ERR "hda-codec: No codec parser is available\n");

	return err;
}
EXPORT_SYMBOL_GPL(snd_soc_hda_codec_configure);

/**
 * snd_soc_hda_codec_reset - Clear all objects assigned to the codec
 * @codec: HD-audio codec
 *
 * This frees the all PCM and control elements assigned to the codec, and
 * clears the caches and restores the pin default configurations.
 *
 * When a device is being used, it returns -EBSY.  If successfully freed,
 * returns zero.
 */
int snd_soc_hda_codec_reset(struct snd_soc_hda_codec *codec)
{
	/* OK, let it free */
	snd_hda_codec_reset(&codec->hdac);
	if (codec->ops.free)
		codec->ops.free(codec);
	memset(&codec->ops, 0, sizeof(codec->ops));

	return 0;
}

int snd_soc_hda_codec_build_controls(struct snd_soc_hda_codec *codec)
{
	int err = 0;

	if (codec->ops.init)
		err = codec->ops.init(codec);
	if (!err && codec->ops.build_controls)
		err = codec->ops.build_controls(codec);
	if (err < 0)
		return err;

/* FIXME: chmaps should be added for pcm devices of this codec in
 * Machine driver. Jackpoll interval has to be implemented as module parameter
 * in Realtek codec driver.*/
#if 0
	err = add_std_chmaps(codec);
	if (err < 0)
		return err;
	if (codec->hdac->jackpoll_interval)
		hda_jackpoll_work(&codec->hdac->jackpoll_work.work);
	else
#endif
	snd_hda_jack_report_sync(&codec->hdac); /* call at the last init point */
	snd_hda_sync_power_up_states(&codec->hdac);

	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_hda_codec_build_controls);

/* get dai map entry for the given dai*/
int snd_soc_hda_get_dai_map(struct snd_soc_hda_codec *codec,
		struct snd_soc_dai *dai)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(codec->nid_list); i++) {
		if (!strcmp(codec->nid_list[i].dai_name, dai->name)) {
			return i;
		}
	}
	return -1;
}
EXPORT_SYMBOL_GPL(snd_soc_hda_get_dai_map);

/* create dai map entry for the given dai, nid*/

int snd_soc_hda_add_dai_map(struct snd_soc_hda_codec *codec,
		char *dai_name, hda_nid_t nid, u32 maxbps)
{
	unsigned int index = codec->map_cur_idx;
	struct hda_dai_map *p = &codec->nid_list[index];

	p->dai_name = kstrdup(dai_name, GFP_KERNEL);
	p->nid = nid;
	p->maxbps = maxbps;
	soc_codec_dbg(codec, "in add-dai-map at idx %d: name %s, nid %d, maxbps %d\n",
			index, p->dai_name, p->nid, p->maxbps);
	index++;
	if (index > ARRAY_SIZE(codec->nid_list))
			return -1;
	codec->map_cur_idx = index;
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_hda_add_dai_map);

static int create_hda_soc_dai(struct snd_soc_hda_codec *codec,
	struct snd_soc_dai_driver *dentry, hda_nid_t nid, char *dai_name,
	char *stream_name, int direction, int max_chan)
{
	u32 rates, bps;
	u64 formats;
	int ret, i;
	unsigned int rate_max = 0, rate_min = 0;
	struct hda_cvt_setup *cvtp;
	static unsigned int rate_pcm[] = {
                8000, 11025, 16000, 22050, 32000, 44100, 48000, 88200,
                96000, 176400, 192000, 384000
        };

	dentry->name = kstrdup(dai_name, GFP_KERNEL);
	ret = snd_hda_query_supported_pcm(&codec->hdac, nid,
				&rates, &formats, &bps);
	if (ret)
		return ret;

	soc_codec_dbg(codec, "nid:%d rates 0x%x, formats 0x%x, bps 0x%x\n",
			      nid, rates, (unsigned int)formats, bps);

	for (i = 0; i < ARRAY_SIZE(rate_pcm); i++) {
		if (rates & (1 << i)) {
			rate_min = rate_pcm[i];
			break;
		}
	}
	for (i = ARRAY_SIZE(rate_pcm) - 1; i >= 0; i--) {
		if (rates & (1 << i)) {
			rate_max = rate_pcm[i];
			break;
		}
	}
	soc_codec_dbg(codec, "min_rate %d, max_rate %d\n", rate_min, rate_max);

	if (!direction) {
		dentry->playback.stream_name = kstrdup(stream_name,
								GFP_KERNEL);
		dentry->playback.formats = formats;
		dentry->playback.rates = rates;
		dentry->playback.rate_max = rate_max;
		dentry->playback.rate_min = rate_min;
		dentry->playback.channels_min = 1;
		dentry->playback.channels_max = max_chan;
	} else {
		dentry->capture.stream_name = kstrdup(stream_name,
							GFP_KERNEL);
		dentry->capture.formats = formats;
		dentry->capture.rates = rates;
		dentry->capture.rate_min = rate_min;
		dentry->capture.rate_max = rate_max;
		dentry->capture.channels_min = 1;
		dentry->capture.channels_max = max_chan;

	}
	cvtp = snd_hda_get_cvt_setup(&codec->hdac, nid);
	if (!cvtp)
		return -1;
	soc_codec_dbg(codec, "%s nid %d\n", stream_name, nid);
	ret = snd_soc_hda_add_dai_map(codec, dai_name, nid, bps);
	if (ret)
		return -1;
	return 0;
}

int snd_soc_hda_build_codec_dais(struct snd_soc_hda_codec *codec,
		struct snd_soc_dai_driver *daip, int *num_dai,
		struct snd_soc_dai_ops *ops)
{
	struct hda_gen_spec *spec = codec->hdac.spec;
	struct auto_pin_cfg *cfg = &spec->autocfg;
	int ret, j, i = 0;
	char name[30];
	hda_nid_t nid;
	struct hda_dai_map *p;

	*num_dai = (spec->num_all_dacs - 1) + spec->num_adc_nids;
	soc_codec_dbg(codec, "dacs %d, adcs %d, dais %d\n",
			spec->num_all_dacs, spec->num_adc_nids, *num_dai);

	if (spec->multiout.num_dacs > 0) {
		if (cfg->speaker_outs) {
			/* Create Speaker Dai */
			for (j = 0; j < cfg->speaker_outs; j++) {
				if (spec->multiout.extra_out_nid[j])
					nid = spec->multiout.extra_out_nid[j];
				else
					continue;
				soc_codec_dbg(codec, "creating speaker dai with nid %d\n", nid);
				snprintf(name, sizeof(name), "%s-AIF%d",
						soc_hda_get_device_id(codec->hdev)->name, (i+1));

				ret = create_hda_soc_dai(codec, &daip[i], nid, name,
					"Speaker", SNDRV_PCM_STREAM_PLAYBACK,
					spec->multiout.max_channels);
				if (ret)
					return ret;
				daip[i].ops = ops;
				i++;
			}
		}
		if (cfg->hp_outs && (cfg->line_out_type == AUTO_PIN_HP_OUT)) {
			/*Create Headphone dai */
			for (j = 0; j < cfg->hp_outs; j++) {
				if (spec->multiout.dac_nids[j])
					nid = spec->multiout.dac_nids[j];
				else
					continue;
				soc_codec_dbg(codec, "creating headset dai with nid %d\n",
						nid);
				snprintf(name, sizeof(name), "%s-AIF%d",
				     soc_hda_get_device_id(codec->hdev)->name, (i+1));
				ret = create_hda_soc_dai(codec, &daip[i], nid, name,
					"Headphone", SNDRV_PCM_STREAM_PLAYBACK,
					spec->multiout.max_channels);
				if (ret)
					return ret;
				daip[i].ops = ops;
				i++;
			}
		}
	}
	if (spec->num_adc_nids) {
		/* Create Capture Dai */
		for (j = 0; j < spec->num_adc_nids; j++) {
			if (spec->adc_nids[j])
				nid = spec->adc_nids[j];
			else
				continue;
			soc_codec_dbg(codec, "creating capture dai with nid %d\n",
					nid);
			snprintf(name, sizeof(name), "%s-AIF%d",
			     soc_hda_get_device_id(codec->hdev)->name, (i+1));
			ret = create_hda_soc_dai(codec, &daip[i], nid, name,
					"Record", SNDRV_PCM_STREAM_CAPTURE,
					spec->multiout.max_channels);
			if (ret)
				return ret;
			daip[i].ops = ops;
			i++;
		}
	}

	if (i > *num_dai)
		return -1;
	soc_codec_dbg(codec, "printing dai map\n");
	for (i = 0; i < ARRAY_SIZE(codec->nid_list); i++) {
		p = &codec->nid_list[i];
		soc_codec_dbg(codec, " %d: dai_name: %s, nid %d, bps %d\n",
					i, p->dai_name, p->nid, p->maxbps);
	}
	return 0;
}
EXPORT_SYMBOL_GPL(snd_soc_hda_build_codec_dais);

MODULE_DESCRIPTION("ASoC HDA codec core");
MODULE_LICENSE("GPL v2");
