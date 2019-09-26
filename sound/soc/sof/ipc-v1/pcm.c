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
 * PCM
 */

int pcm_open(struct snd_sof_dev *sdev, struct snd_pcm_substream *substream,
	     struct snd_sof_pcm *spcm)
{
	struct sof_ipc_stream_posn *posn =
		spcm->stream[substream->stream].ipc_posn;
	posn->host_posn = 0;
	posn->dai_posn = 0;

	return 0;
}

int pcm_close(struct snd_sof_dev *sdev, struct snd_pcm_substream *substream,
	      struct snd_sof_pcm *spcm)
{
	return 0;
}

int pcm_trigger(struct snd_sof_dev *sdev,
		struct snd_pcm_substream *substream,
		struct snd_sof_pcm *spcm, int cmd, bool *reset_needed)
{
	struct sof_ipc_stream stream;
	struct sof_ipc_reply reply;
	bool ipc_first = false;
	int ret = 0;

	stream.hdr.size = sizeof(stream);
	stream.hdr.cmd = SOF_IPC_GLB_STREAM_MSG;
	stream.comp_id = spcm->stream[substream->stream].comp_id;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		stream.hdr.cmd |= SOF_IPC_STREAM_TRIG_PAUSE;
		ipc_first = true;
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		stream.hdr.cmd |= SOF_IPC_STREAM_TRIG_RELEASE;
		break;
	case SNDRV_PCM_TRIGGER_RESUME:
		/* set up hw_params */
		ret = snd_sof_pcm_prepare(substream);
		if (ret < 0) {
			dev_err(sdev->dev,
				"error: failed to set up hw_params upon resume\n");
			return ret;
		}

		/* fallthrough */
	case SNDRV_PCM_TRIGGER_START:
		stream.hdr.cmd |= SOF_IPC_STREAM_TRIG_START;
		break;
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
		stream.hdr.cmd |= SOF_IPC_STREAM_TRIG_STOP;
		ipc_first = true;
		*reset_needed = true;
		break;
	default:
		dev_err(sdev->dev, "error: unhandled trigger cmd %d\n", cmd);
		return -EINVAL;
	}

	/*
	 * DMA and IPC sequence is different for start and stop. Need to send
	 * STOP IPC before stop DMA
	 */
	if (!ipc_first)
		snd_sof_pcm_platform_trigger(sdev, substream, cmd);

	/* send IPC to the DSP */
	ret = sof_ipc_tx_message(sdev->ipc, stream.hdr.cmd, &stream,
				 sizeof(stream), &reply, sizeof(reply));

	/* need to STOP DMA even if STOP IPC failed */
	if (ipc_first)
		snd_sof_pcm_platform_trigger(sdev, substream, cmd);

	return ret;
}

int pcm_free(struct snd_sof_dev *sdev, struct snd_pcm_substream *substream,
	     struct snd_sof_pcm *spcm)
{
	struct sof_ipc_stream stream;
	struct sof_ipc_reply reply;
	int ret;

	stream.hdr.size = sizeof(stream);
	stream.hdr.cmd = SOF_IPC_GLB_STREAM_MSG | SOF_IPC_STREAM_PCM_FREE;
	stream.comp_id = spcm->stream[substream->stream].comp_id;

	/* send IPC to the DSP */
	ret = sof_ipc_tx_message(sdev->ipc, stream.hdr.cmd, &stream,
				 sizeof(stream), &reply, sizeof(reply));

	return ret;
}


int pcm_dsp_params(struct snd_sof_pcm *spcm,
		   struct snd_pcm_substream *substream,
		   const struct sof_ipc_pcm_params_reply *reply)
{
	struct snd_sof_dev *sdev = spcm->sdev;
	/* validate offset */
	int ret = snd_sof_ipc_pcm_params(sdev, substream, reply->posn_offset);

	if (ret < 0)
		dev_err(sdev->dev, "error: got wrong reply for PCM %d\n",
			spcm->pcm.pcm_id);

	return ret;
}

/* Create DMA buffer page table for DSP */
static int create_page_table(struct snd_pcm_substream *substream,
			     unsigned char *dma_area, size_t size)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_component *component =
		snd_soc_rtdcom_lookup(rtd, SOF_COMPONENT_NAME);
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(component);
	struct snd_sof_pcm *spcm;
	struct snd_dma_buffer *dmab = snd_pcm_get_dma_buf(substream);
	int stream = substream->stream;

	spcm = snd_sof_find_spcm_dai(sdev, rtd);
	if (!spcm)
		return -EINVAL;

	return snd_sof_create_page_table(sdev, dmab,
		spcm->stream[stream].page_table.area, size);
}

int pcm_hw_params(struct snd_sof_dev *sdev,
		  struct snd_pcm_substream *substream,
		  struct snd_pcm_hw_params *params,
		  struct snd_sof_pcm *spcm, int new_buffer)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct sof_ipc_pcm_params pcm;
	struct sof_ipc_pcm_params_reply ipc_params_reply;
	int ret;


	memset(&pcm, 0, sizeof(pcm));

	if (new_buffer) {
		/*
		 * new_buffer means the buffer is changed
		 * create compressed page table for audio firmware
		 * new_buffer == 0 means the buffer is not changed
		 * so no need to regenerate the page table
		 */
		ret = create_page_table(substream, runtime->dma_area,
					runtime->dma_bytes);
		if (ret < 0)
			return ret;
	}

	/* number of pages should be rounded up */
	pcm.params.buffer.pages = PFN_UP(runtime->dma_bytes);

	/* set IPC PCM parameters */
	pcm.hdr.size = sizeof(pcm);
	pcm.hdr.cmd = SOF_IPC_GLB_STREAM_MSG | SOF_IPC_STREAM_PCM_PARAMS;
	pcm.comp_id = spcm->stream[substream->stream].comp_id;
	pcm.params.hdr.size = sizeof(pcm.params);
	pcm.params.buffer.phy_addr =
		spcm->stream[substream->stream].page_table.addr;
	pcm.params.buffer.size = runtime->dma_bytes;
	pcm.params.direction = substream->stream;
	pcm.params.sample_valid_bytes = params_width(params) >> 3;
	pcm.params.buffer_fmt = SOF_IPC_BUFFER_INTERLEAVED;
	pcm.params.rate = params_rate(params);
	pcm.params.channels = params_channels(params);
	pcm.params.host_period_bytes = params_period_bytes(params);

	/* container size */
	ret = snd_pcm_format_physical_width(params_format(params));
	if (ret < 0)
		return ret;
	pcm.params.sample_container_bytes = ret >> 3;

	/* format */
	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S16:
		pcm.params.frame_fmt = SOF_IPC_FRAME_S16_LE;
		break;
	case SNDRV_PCM_FORMAT_S24:
		pcm.params.frame_fmt = SOF_IPC_FRAME_S24_4LE;
		break;
	case SNDRV_PCM_FORMAT_S32:
		pcm.params.frame_fmt = SOF_IPC_FRAME_S32_LE;
		break;
	case SNDRV_PCM_FORMAT_FLOAT:
		pcm.params.frame_fmt = SOF_IPC_FRAME_FLOAT;
		break;
	default:
		return -EINVAL;
	}

	/* firmware already configured host stream */
	ret = snd_sof_pcm_platform_hw_params(sdev,
					     substream,
					     params,
					     &pcm.params);
	if (ret < 0) {
		dev_err(sdev->dev, "error: platform hw params failed\n");
		return ret;
	}

	dev_dbg(sdev->dev, "stream_tag %d", pcm.params.stream_tag);

	/* send IPC to the DSP */
	ret = sof_ipc_tx_message(sdev->ipc, pcm.hdr.cmd, &pcm, sizeof(pcm),
				 &ipc_params_reply, sizeof(ipc_params_reply));
	if (ret < 0) {
		dev_err(sdev->dev, "error: hw params ipc failed for stream %d\n",
			pcm.params.stream_tag);
		return ret;
	}

	ret = pcm_dsp_params(spcm, substream, &ipc_params_reply);
	if (ret < 0)
		return ret;

	return ret;
}

