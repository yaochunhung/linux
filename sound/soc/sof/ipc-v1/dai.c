// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
//
// This file is provided under a dual BSD/GPLv2 license.  When using or
// redistributing this file, you may do so under either license.
//
// Copyright(c) 2019 Intel Corporation. All rights reserved.
//
// Author: Liam Girdwood <liam.r.girdwood@linux.intel.com>
//
// Legacy IPC Support. IPC flavour is detected by drivers on FW boot.
//

#include <linux/mutex.h>
#include <linux/types.h>
#include <sound/pcm_params.h>

#include <sound/sof-ipc-v1/stream.h> /* needs to be included before control.h */
#include <sound/sof-ipc-v1/control.h>
#include <sound/sof-ipc-v1/dai.h>
#include <sound/sof-ipc-v1/info.h>
#include <sound/sof-ipc-v1/pm.h>
#include <sound/sof-ipc-v1/topology.h>
#include <sound/sof-ipc-v1/trace.h>
#include <sound/sof-ipc-v1/xtensa.h>

#include "ipc-v1.h"

/*
 * DAI
 */

/* fixup the BE DAI link to match any values from topology */
int dai_link_fixup(struct snd_sof_dev *sdev,
		   struct snd_soc_pcm_runtime *rtd,
		   struct snd_pcm_hw_params *params)
{
	struct snd_interval *rate = hw_param_interval(params,
			SNDRV_PCM_HW_PARAM_RATE);
	struct snd_interval *channels = hw_param_interval(params,
						SNDRV_PCM_HW_PARAM_CHANNELS);
	struct snd_mask *fmt = hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT);
	struct snd_sof_dai *dai =
		snd_sof_find_dai(sdev, (char *)rtd->dai_link->name);
	struct sof_ipc_dai_config *dai_config;
	struct sof_ipc_comp_dai *comp_dai;

	/* no topology exists for this BE, try a common configuration */
	if (!dai) {
		dev_warn(sdev->dev, "warning: no topology found for BE DAI %s config\n",
			 rtd->dai_link->name);

		/*  set 48k, stereo, 16bits by default */
		rate->min = 48000;
		rate->max = 48000;

		channels->min = 2;
		channels->max = 2;

		snd_mask_none(fmt);
		snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S16_LE);

		return 0;
	}

	comp_dai = dai->ipc_comp_dai;
	dai_config = dai->ipc_dai_config;

	/* read format from topology */
	snd_mask_none(fmt);

	switch (comp_dai->config.frame_fmt) {
	case SOF_IPC_FRAME_S16_LE:
		snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S16_LE);
		break;
	case SOF_IPC_FRAME_S24_4LE:
		snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S24_LE);
		break;
	case SOF_IPC_FRAME_S32_LE:
		snd_mask_set_format(fmt, SNDRV_PCM_FORMAT_S32_LE);
		break;
	default:
		dev_err(sdev->dev, "error: No available DAI format!\n");
		return -EINVAL;
	}

	/* read rate and channels from topology */
	switch (dai_config->type) {
	case SOF_DAI_INTEL_SSP:
		rate->min = dai_config->ssp.fsync_rate;
		rate->max = dai_config->ssp.fsync_rate;
		channels->min = dai_config->ssp.tdm_slots;
		channels->max = dai_config->ssp.tdm_slots;

		dev_dbg(sdev->dev,
			"rate_min: %d rate_max: %d\n", rate->min, rate->max);
		dev_dbg(sdev->dev,
			"channels_min: %d channels_max: %d\n",
			channels->min, channels->max);

		break;
	case SOF_DAI_INTEL_DMIC:
		/* DMIC only supports 16 or 32 bit formats */
		if (comp_dai->config.frame_fmt == SOF_IPC_FRAME_S24_4LE) {
			dev_err(sdev->dev,
				"error: invalid fmt %d for DAI type %d\n",
				comp_dai->config.frame_fmt,
				dai_config->type);
		}
		break;
	case SOF_DAI_INTEL_HDA:
		/* do nothing for HDA dai_link */
		break;
	case SOF_DAI_INTEL_ALH:
		/* do nothing for ALH dai_link */
		break;
	default:
		dev_err(sdev->dev, "error: invalid DAI type %d\n",
			dai_config->type);
		break;
	}

	return 0;
}

int dai_hda_stream_config(struct snd_sof_dev *sdev,
			  struct hdac_stream *hstream, void *params)
{
	struct sof_intel_hda_dev *hda = sdev->pdata->hw_pdata;
	struct sof_ipc_stream_params *ipc_params = params;
	struct fw_version *v = &sdev->fw_version;

	/* update no_stream_position flag for ipc params */
	if (hda && hda->no_ipc_position) {
		/* For older ABIs set host_period_bytes to zero to inform
		 * FW we don't want position updates. Newer versions use
		 * no_stream_position for this purpose.
		 */
		if (v->abi_version < SOF_ABI_VER(3, 10, 0))
			ipc_params->host_period_bytes = 0;
		else
			ipc_params->no_stream_position = 1;
	}

	ipc_params->stream_tag = hstream->stream_tag;

	return 0;
}

/* Send DAI_CONFIG IPC to the DAI that matches the dai_name and direction */
int dai_hda_link_config(struct snd_sof_dev *sdev, void *hdastream,
			const char *dai_name, int channel, int dir)
{
	struct sof_intel_hda_stream *hda_stream = hdastream;
	struct sof_ipc_dai_config *config;
	struct sof_ipc_comp_dai *comp_dai;
	struct snd_sof_dai *sof_dai;
	struct sof_ipc_reply reply;
	int ret = 0;

	list_for_each_entry(sof_dai, &hda_stream->sdev->dai_list, list) {
		if (!sof_dai->cpu_dai_name)
			continue;

		comp_dai = sof_dai->ipc_comp_dai;

		if (!strcmp(dai_name, sof_dai->cpu_dai_name) &&
		    dir == comp_dai->direction) {
			config = sof_dai->ipc_dai_config;

			if (!config) {
				dev_err(hda_stream->sdev->dev,
					"error: no config for DAI %s\n",
					sof_dai->name);
				return -EINVAL;
			}

			/* update config with stream tag */
			config->hda.link_dma_ch = channel;

			/* send IPC */
			ret = sof_ipc_tx_message(hda_stream->sdev->ipc,
						 config->hdr.cmd,
						 config,
						 config->hdr.size,
						 &reply, sizeof(reply));

			if (ret < 0)
				dev_err(hda_stream->sdev->dev,
					"error: failed to set dai config for %s\n",
					sof_dai->name);
			return ret;
		}
	}

	return -EINVAL;
}

