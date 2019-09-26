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
 * PM
 */

int pm_restore_kcontrol(struct snd_sof_dev *sdev,
			struct snd_sof_control *scontrol)
{
	int ipc_cmd;
	int ctrl_type;
	int ret = 0;

	/* reset readback offset for scontrol after resuming */
	scontrol->readback_offset = 0;

	/* notify DSP of kcontrol values */
	switch (scontrol->ipc_ctrl_cmd) {
	case SOF_CTRL_CMD_VOLUME:
	case SOF_CTRL_CMD_ENUM:
	case SOF_CTRL_CMD_SWITCH:
		ipc_cmd = SOF_IPC_COMP_SET_VALUE;
		ctrl_type = SOF_CTRL_TYPE_VALUE_CHAN_SET;
		ret = snd_sof_ipc_set_get_comp_data(sdev->ipc, scontrol,
						    ipc_cmd, ctrl_type,
						    scontrol->ipc_ctrl_cmd,
						    true);
		break;
	case SOF_CTRL_CMD_BINARY:
		ipc_cmd = SOF_IPC_COMP_SET_DATA;
		ctrl_type = SOF_CTRL_TYPE_DATA_SET;
		ret = snd_sof_ipc_set_get_comp_data(sdev->ipc, scontrol,
						    ipc_cmd, ctrl_type,
						    scontrol->ipc_ctrl_cmd,
						    true);
		break;

	default:
		break;
	}

	return ret;
}

int pm_restore_pipeline(struct snd_sof_dev *sdev,
			struct snd_sof_widget *swidget)
{
	struct snd_sof_dai *dai;
	struct sof_ipc_pipe_new *pipeline;
	struct sof_ipc_comp_dai *comp_dai;
	struct sof_ipc_cmd_hdr *hdr;
	struct sof_ipc_comp_reply r;
	int ret;

	/* skip if there is no private data */
	if (!swidget->private)
		return 0;;

	switch (swidget->id) {
	case snd_soc_dapm_dai_in:
	case snd_soc_dapm_dai_out:
		dai = swidget->private;
		comp_dai = dai->ipc_comp_dai;
		ret = sof_ipc_tx_message(sdev->ipc,
					 comp_dai->comp.hdr.cmd,
					 comp_dai, sizeof(*comp_dai),
					 &r, sizeof(r));
		break;
	case snd_soc_dapm_scheduler:

		/*
		 * During suspend, all DSP cores are powered off.
		 * Therefore upon resume, create the pipeline comp
		 * and power up the core that the pipeline is
		 * scheduled on.
		 */
		pipeline = swidget->private;
		ret = sof_load_pipeline_ipc(sdev, pipeline, &r);
		break;
	default:
		hdr = swidget->private;
		ret = sof_ipc_tx_message(sdev->ipc, hdr->cmd,
					 swidget->private, hdr->size,
					 &r, sizeof(r));
		break;
	}

	return ret;
}


int pm_restore_connection(struct snd_sof_dev *sdev,
			  struct snd_sof_route *sroute)
{
	struct sof_ipc_pipe_comp_connect *connect;
	struct sof_ipc_reply reply;
	int ret;

	/* skip if there's no private data */
	if (!sroute->private)
		return 0;

	connect = sroute->private;

	/* send ipc */
	ret = sof_ipc_tx_message(sdev->ipc,
				 connect->hdr.cmd,
				 connect, sizeof(*connect),
				 &reply, sizeof(reply));

	return ret;
}

int pm_restore_dai_link(struct snd_sof_dev *sdev,
			struct snd_sof_dai *dai)
{
	struct sof_ipc_reply reply;
	struct sof_ipc_dai_config *config = dai->ipc_dai_config;
	int ret = 0;

	if (!config) {
		dev_err(sdev->dev, "error: no config for DAI %s\n",
			dai->name);
		return 0;
	}

	/*
	 * The link DMA channel would be invalidated for running
	 * streams but not for streams that were in the PAUSED
	 * state during suspend. So invalidate it here before setting
	 * the dai config in the DSP.
	 */
	if (config->type == SOF_DAI_INTEL_HDA)
		config->hda.link_dma_ch = DMA_CHAN_INVALID;

	ret = sof_ipc_tx_message(sdev->ipc,
				 config->hdr.cmd, config,
				 config->hdr.size,
				 &reply, sizeof(reply));

	return ret;
}

int pm_set_state(struct snd_sof_dev *sdev, int cmd)
{
	struct sof_ipc_pm_ctx pm_ctx;
	struct sof_ipc_reply reply;

	memset(&pm_ctx, 0, sizeof(pm_ctx));

	switch (cmd) {
	case SOF_PM_CTX_SAVE:
		cmd = SOF_IPC_PM_CTX_SAVE;
		break;
	case SOF_PM_CTX_RESTORE:
		cmd = SOF_IPC_PM_CTX_RESTORE;
		break;
	default:
		dev_err(sdev->dev, "error: unkow PM cmd %d\n", cmd);
		return -EINVAL;
	}

	/* configure ctx save ipc message */
	pm_ctx.hdr.size = sizeof(pm_ctx);
	pm_ctx.hdr.cmd = SOF_IPC_GLB_PM_MSG | cmd;

	/* send ctx save ipc to dsp */
	return sof_ipc_tx_message(sdev->ipc, pm_ctx.hdr.cmd, &pm_ctx,
				 sizeof(pm_ctx), &reply, sizeof(reply));
}

