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
 * Trace
 */

int trace_enable(struct snd_sof_dev *sdev)
{
	struct ipc_data *idata = sdev->ipc_private;
	struct sof_ipc_fw_ready *ready = &idata->fw_ready;
	struct sof_ipc_fw_version *v = &ready->version;
	struct sof_ipc_dma_trace_params_ext params;
	struct sof_ipc_reply ipc_reply;
	int ret;

	if (!sdev->dtrace_is_supported)
		return 0;

	if (sdev->dtrace_is_enabled || !sdev->dma_trace_pages)
		return -EINVAL;

	/* set IPC parameters */
	params.hdr.cmd = SOF_IPC_GLB_TRACE_MSG;
	/* PARAMS_EXT is only supported from ABI 3.7.0 onwards */
	if (v->abi_version >= SOF_ABI_VER(3, 7, 0)) {
		params.hdr.size = sizeof(struct sof_ipc_dma_trace_params_ext);
		params.hdr.cmd |= SOF_IPC_TRACE_DMA_PARAMS_EXT;
		params.timestamp_ns = ktime_get(); /* in nanosecond */
	} else {
		params.hdr.size = sizeof(struct sof_ipc_dma_trace_params);
		params.hdr.cmd |= SOF_IPC_TRACE_DMA_PARAMS;
	}
	params.buffer.phy_addr = sdev->dmatp.addr;
	params.buffer.size = sdev->dmatb.bytes;
	params.buffer.pages = sdev->dma_trace_pages;
	params.stream_tag = 0;

	sdev->host_offset = 0;
	sdev->dtrace_draining = false;

	ret = snd_sof_dma_trace_init(sdev, &params.stream_tag);
	if (ret < 0) {
		dev_err(sdev->dev,
			"error: fail in snd_sof_dma_trace_init %d\n", ret);
		return ret;
	}
	dev_dbg(sdev->dev, "stream_tag: %d\n", params.stream_tag);

	/* send IPC to the DSP */
	ret = sof_ipc_tx_message(sdev->ipc,
				 params.hdr.cmd, &params, sizeof(params),
				 &ipc_reply, sizeof(ipc_reply));
	if (ret < 0) {
		dev_err(sdev->dev,
			"error: can't set params for DMA for trace %d\n", ret);
		goto trace_release;
	}

	ret = snd_sof_dma_trace_trigger(sdev, SNDRV_PCM_TRIGGER_START);
	if (ret < 0) {
		dev_err(sdev->dev,
			"error: snd_sof_dma_trace_trigger: start: %d\n", ret);
		goto trace_release;
	}

	sdev->dtrace_is_enabled = true;

	return 0;

trace_release:
	snd_sof_dma_trace_release(sdev);
	return ret;
}

int trace_update_pos(struct snd_sof_dev *sdev,
		     struct sof_ipc_dma_trace_posn *posn)
{
	if (!sdev->dtrace_is_supported)
		return 0;

	if (sdev->dtrace_is_enabled && sdev->host_offset != posn->host_offset) {
		sdev->host_offset = posn->host_offset;
		wake_up(&sdev->trace_sleep);
	}

	if (posn->overflow != 0)
		dev_err(sdev->dev,
			"error: DSP trace buffer overflow %u bytes. Total messages %d\n",
			posn->overflow, posn->messages);

	return 0;
}

void trace_process_message(struct snd_sof_dev *sdev, u32 msg_id)
{
	struct sof_ipc_dma_trace_posn posn;

	switch (msg_id) {
	case SOF_IPC_TRACE_DMA_POSITION:
		/* read back full message */
		snd_sof_ipc_msg_data(sdev, NULL, &posn, sizeof(posn));
		trace_update_pos(sdev, &posn);
		break;
	default:
		dev_err(sdev->dev, "error: unhandled trace message %x\n",
			msg_id);
		break;
	}
}
