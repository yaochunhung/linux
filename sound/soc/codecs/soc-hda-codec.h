/*
 *  ASoC High Definition Audio Codec interface
 *
 * Copyright (c) 2014 Intel Corporation
 *
 * Author(s): Lakshmi Vinnakota <lakshmi.n.vinnakota@intel.com>
 *
 *  This program is free software; you can redistribute it and/or modify it
 *  under the terms of the GNU General Public License as published by the Free
 *  Software Foundation; either version 2 of the License, or (at your option)
 *  any later version.
 *
 *  This program is distributed in the hope that it will be useful, but WITHOUT
 *  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 *  FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 *  more details.
 *
 */

#ifndef __SOUND_SOC_HDA_CODEC_H
#define __SOUND_SOC_HDA_CODEC_H

#include <sound/info.h>
#include <sound/control.h>
#include <sound/soc.h>
#include <sound/hda_codec.h>
#include <sound/soc-hda-bus.h>

struct snd_soc_hda_codec;

/* ops common to all codec drivers */
struct snd_soc_hda_codec_ops {
	int (*build_controls)(struct snd_soc_hda_codec *codec);
	int (*init)(struct snd_soc_hda_codec *codec);
	void (*free)(struct snd_soc_hda_codec *codec);
};

struct hda_dai_map {
	char *dai_name;
	hda_nid_t nid;
	u32	maxbps;
};

#define HDA_MAX_CODEC_DAIS 16

/**
 * struct snd_soc_hda_codec -  ASoC HDA codec information
 *
 * @hdev - pointer the hda codec device on the soc-hda-bus
 * @scodec - pointer to soc codec
 * @nid_list - the dai map which matches the dai-name with the nid
 * @map_cur_idx - the idx in use in dai_map
 * @ops - the soc hda codec ops common to all codec drivers
 * @hdac - an instance of the common hda_codec structure
 *
 **/
struct snd_soc_hda_codec {

	struct snd_soc_hda_device *hdev;
	struct snd_soc_codec *scodec;
	/*soc-dai to nid map*/
	struct hda_dai_map nid_list[HDA_MAX_CODEC_DAIS];
	unsigned int map_cur_idx;

	/* codec ops */
	struct snd_soc_hda_codec_ops ops;

	struct hda_codec hdac; /* Generic Codec Lib structure */
};

/*
 * constructors
 */
int snd_soc_hda_codec_new(struct snd_soc_hda_device *hdev,
		unsigned int codec_addr, struct snd_soc_hda_codec **codecp);
void snd_soc_hda_codec_free(struct snd_soc_hda_codec *codec);
int snd_soc_hda_codec_configure(struct snd_soc_hda_codec *codec);
int snd_soc_hda_codec_build_controls(struct snd_soc_hda_codec *codec);
int snd_soc_hda_build_codec_dais(struct snd_soc_hda_codec *codec,
		struct snd_soc_dai_driver *daip, int *num_dai,
		struct snd_soc_dai_ops *ops);

int snd_soc_hda_add_dai_map(struct snd_soc_hda_codec *codec,
	char *dai_name, hda_nid_t nid, u32 maxbps);
int snd_soc_hda_get_dai_map(struct snd_soc_hda_codec *codec,
		struct snd_soc_dai *dai);

#define soc_codec_err(codec, fmt, args...) dev_err(&(codec)->hdev->dev, fmt, ##args)
#define soc_codec_warn(codec, fmt, args...) dev_warn(&(codec)->hdev->dev, fmt, ##args)
#define soc_codec_info(codec, fmt, args...) dev_info(&(codec)->hdev->dev, fmt, ##args)
#define soc_codec_dbg(codec, fmt, args...) dev_dbg(&(codec)->hdev->dev, fmt, ##args)
#endif /* __SOUND_SOC_HDA_CODEC_H */
