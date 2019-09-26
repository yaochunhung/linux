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

#if IS_ENABLED(CONFIG_SND_SOC_SOF_DEBUG_VERBOSE_IPC)
static void ipc_log_header(struct snd_sof_dev *sdev, u8 *text, u32 cmd)
{
	u8 *str;
	u8 *str2 = NULL;
	u32 glb;
	u32 type;

	glb = cmd & SOF_GLB_TYPE_MASK;
	type = cmd & SOF_CMD_TYPE_MASK;

	switch (glb) {
	case SOF_IPC_GLB_REPLY:
		str = "GLB_REPLY"; break;
	case SOF_IPC_GLB_COMPOUND:
		str = "GLB_COMPOUND"; break;
	case SOF_IPC_GLB_TPLG_MSG:
		str = "GLB_TPLG_MSG";
		switch (type) {
		case SOF_IPC_TPLG_COMP_NEW:
			str2 = "COMP_NEW"; break;
		case SOF_IPC_TPLG_COMP_FREE:
			str2 = "COMP_FREE"; break;
		case SOF_IPC_TPLG_COMP_CONNECT:
			str2 = "COMP_CONNECT"; break;
		case SOF_IPC_TPLG_PIPE_NEW:
			str2 = "PIPE_NEW"; break;
		case SOF_IPC_TPLG_PIPE_FREE:
			str2 = "PIPE_FREE"; break;
		case SOF_IPC_TPLG_PIPE_CONNECT:
			str2 = "PIPE_CONNECT"; break;
		case SOF_IPC_TPLG_PIPE_COMPLETE:
			str2 = "PIPE_COMPLETE"; break;
		case SOF_IPC_TPLG_BUFFER_NEW:
			str2 = "BUFFER_NEW"; break;
		case SOF_IPC_TPLG_BUFFER_FREE:
			str2 = "BUFFER_FREE"; break;
		default:
			str2 = "unknown type"; break;
		}
		break;
	case SOF_IPC_GLB_PM_MSG:
		str = "GLB_PM_MSG";
		switch (type) {
		case SOF_IPC_PM_CTX_SAVE:
			str2 = "CTX_SAVE"; break;
		case SOF_IPC_PM_CTX_RESTORE:
			str2 = "CTX_RESTORE"; break;
		case SOF_IPC_PM_CTX_SIZE:
			str2 = "CTX_SIZE"; break;
		case SOF_IPC_PM_CLK_SET:
			str2 = "CLK_SET"; break;
		case SOF_IPC_PM_CLK_GET:
			str2 = "CLK_GET"; break;
		case SOF_IPC_PM_CLK_REQ:
			str2 = "CLK_REQ"; break;
		case SOF_IPC_PM_CORE_ENABLE:
			str2 = "CORE_ENABLE"; break;
		default:
			str2 = "unknown type"; break;
		}
		break;
	case SOF_IPC_GLB_COMP_MSG:
		str = "GLB_COMP_MSG";
		switch (type) {
		case SOF_IPC_COMP_SET_VALUE:
			str2 = "SET_VALUE"; break;
		case SOF_IPC_COMP_GET_VALUE:
			str2 = "GET_VALUE"; break;
		case SOF_IPC_COMP_SET_DATA:
			str2 = "SET_DATA"; break;
		case SOF_IPC_COMP_GET_DATA:
			str2 = "GET_DATA"; break;
		default:
			str2 = "unknown type"; break;
		}
		break;
	case SOF_IPC_GLB_STREAM_MSG:
		str = "GLB_STREAM_MSG";
		switch (type) {
		case SOF_IPC_STREAM_PCM_PARAMS:
			str2 = "PCM_PARAMS"; break;
		case SOF_IPC_STREAM_PCM_PARAMS_REPLY:
			str2 = "PCM_REPLY"; break;
		case SOF_IPC_STREAM_PCM_FREE:
			str2 = "PCM_FREE"; break;
		case SOF_IPC_STREAM_TRIG_START:
			str2 = "TRIG_START"; break;
		case SOF_IPC_STREAM_TRIG_STOP:
			str2 = "TRIG_STOP"; break;
		case SOF_IPC_STREAM_TRIG_PAUSE:
			str2 = "TRIG_PAUSE"; break;
		case SOF_IPC_STREAM_TRIG_RELEASE:
			str2 = "TRIG_RELEASE"; break;
		case SOF_IPC_STREAM_TRIG_DRAIN:
			str2 = "TRIG_DRAIN"; break;
		case SOF_IPC_STREAM_TRIG_XRUN:
			str2 = "TRIG_XRUN"; break;
		case SOF_IPC_STREAM_POSITION:
			str2 = "POSITION"; break;
		case SOF_IPC_STREAM_VORBIS_PARAMS:
			str2 = "VORBIS_PARAMS"; break;
		case SOF_IPC_STREAM_VORBIS_FREE:
			str2 = "VORBIS_FREE"; break;
		default:
			str2 = "unknown type"; break;
		}
		break;
	case SOF_IPC_FW_READY:
		str = "FW_READY"; break;
	case SOF_IPC_GLB_DAI_MSG:
		str = "GLB_DAI_MSG";
		switch (type) {
		case SOF_IPC_DAI_CONFIG:
			str2 = "CONFIG"; break;
		case SOF_IPC_DAI_LOOPBACK:
			str2 = "LOOPBACK"; break;
		default:
			str2 = "unknown type"; break;
		}
		break;
	case SOF_IPC_GLB_TRACE_MSG:
		str = "GLB_TRACE_MSG"; break;
	case SOF_IPC_GLB_TEST_MSG:
		str = "GLB_TEST_MSG";
		switch (type) {
		case SOF_IPC_TEST_IPC_FLOOD:
			str2 = "IPC_FLOOD"; break;
		default:
			str2 = "unknown type"; break;
		}
		break;
	default:
		str = "unknown GLB command"; break;
	}

	if (str2)
		dev_dbg(sdev->dev, "%s: 0x%x: %s: %s\n", text, cmd, str, str2);
	else
		dev_dbg(sdev->dev, "%s: 0x%x: %s\n", text, cmd, str);
}
#else
static void ipc_log_header(struct snd_sof_dev *sdev, u8 *text, u32 cmd)
{
	if ((cmd & SOF_GLB_TYPE_MASK) != SOF_IPC_GLB_TRACE_MSG)
		dev_dbg(sdev->dev, "%s: 0x%x\n", text, cmd);
}
#endif

/*
 * IPC Rx
 */

static void ipc_msgs_rx(struct snd_sof_dev *sdev)
{
	struct sof_ipc_cmd_hdr hdr;
	u32 cmd, type;
	int err = 0;

	/* read back header */
	snd_sof_ipc_msg_data(sdev, NULL, &hdr, sizeof(hdr));
	ipc_log_header(sdev, "ipc rx", hdr.cmd);

	cmd = hdr.cmd & SOF_GLB_TYPE_MASK;
	type = hdr.cmd & SOF_CMD_TYPE_MASK;

	/* check message type */
	switch (cmd) {
	case SOF_IPC_GLB_REPLY:
		dev_err(sdev->dev, "error: ipc reply unknown\n");
		break;
	case SOF_IPC_FW_READY:
		/* check for FW boot completion */
		if (!sdev->boot_complete) {
			err = sof_ops(sdev)->fw_ready(sdev, cmd);
			if (err < 0) {
				/*
				 * this indicates a mismatch in ABI
				 * between the driver and fw
				 */
				dev_err(sdev->dev, "error: ABI mismatch %d\n",
					err);
			} else {
				/* firmware boot completed OK */
				sdev->boot_complete = true;
			}

			/* wake up firmware loader */
			wake_up(&sdev->boot_wait);
		}
		break;
	case SOF_IPC_GLB_COMPOUND:
	case SOF_IPC_GLB_TPLG_MSG:
	case SOF_IPC_GLB_PM_MSG:
	case SOF_IPC_GLB_COMP_MSG:
		break;
	case SOF_IPC_GLB_STREAM_MSG:
		/* need to pass msg id into the function */
		stream_notification(sdev, hdr.cmd);
		break;
	case SOF_IPC_GLB_TRACE_MSG:
		trace_process_message(sdev, type);
		break;
	default:
		dev_err(sdev->dev, "error: unknown DSP message 0x%x\n", cmd);
		break;
	}

	ipc_log_header(sdev, "ipc rx done", hdr.cmd);
}

static void ipc_get_reply(struct snd_sof_dev *sdev)
{
	struct snd_sof_ipc_msg *msg = sdev->msg;
	struct sof_ipc_reply reply;
	struct sof_ipc_cmd_hdr *hdr;
	int ret = 0;

	/*
	 * Sometimes, there is unexpected reply ipc arriving. The reply
	 * ipc belongs to none of the ipcs sent from driver.
	 * In this case, the driver must ignore the ipc.
	 */
	if (!msg) {
		dev_warn(sdev->dev, "unexpected ipc interrupt raised!\n");
		return;
	}

	hdr = msg->msg_data;
	if (hdr->cmd == (SOF_IPC_GLB_PM_MSG | SOF_IPC_PM_CTX_SAVE)) {
		/*
		 * memory windows are powered off before sending IPC reply,
		 * so we can't read the mailbox for CTX_SAVE reply.
		 */
		reply.error = 0;
		reply.hdr.cmd = SOF_IPC_GLB_REPLY;
		reply.hdr.size = sizeof(reply);
		memcpy(msg->reply_data, &reply, sizeof(reply));
		goto out;
	}

	/* get IPC reply from DSP in the mailbox */
	sof_mailbox_read(sdev, sdev->host_box.offset, &reply,
			 sizeof(reply));

	if (reply.error < 0) {
		memcpy(msg->reply_data, &reply, sizeof(reply));
		ret = reply.error;
	} else {
		/* reply correct size ? */
		if (reply.hdr.size != msg->reply_size) {
			dev_err(sdev->dev, "error: reply expected %zu got %u bytes\n",
				msg->reply_size, reply.hdr.size);
			ret = -EINVAL;
		}

		/* read the message */
		if (msg->reply_size > 0)
			sof_mailbox_read(sdev, sdev->host_box.offset,
					 msg->reply_data, msg->reply_size);
	}

out:
	msg->reply_error = ret;

}

static bool ipc_is_sof(struct snd_sof_dev *sdev, u32 mask, u32 msg)
{
	return (msg & (mask | 0xf << 9)) != msg ||
		(msg & mask) != mask;
}

/*
 * IPC layer enumeration.
 */

static int ipc_tx_wait_done(struct snd_sof_dev *sdev, struct snd_sof_ipc *ipc,
			    struct snd_sof_ipc_msg *msg, void *reply_data)
{
	struct sof_ipc_cmd_hdr *hdr = msg->msg_data;
	int ret;

	/* wait for DSP IPC completion */
	ret = wait_event_timeout(msg->waitq, msg->ipc_complete,
				 msecs_to_jiffies(sdev->ipc_timeout));

	if (ret == 0) {
		dev_err(sdev->dev, "error: ipc timed out for 0x%x size %d\n",
			hdr->cmd, hdr->size);
		snd_sof_handle_fw_exception(ipc->sdev);
		ret = -ETIMEDOUT;
	} else {
		/* copy the data returned from DSP */
		ret = msg->reply_error;
		if (msg->reply_size)
			memcpy(reply_data, msg->reply_data, msg->reply_size);
		if (ret < 0)
			dev_err(sdev->dev, "error: ipc error for 0x%x size %zu\n",
				hdr->cmd, msg->reply_size);
		else
			snd_sof_ipc_log_header(sdev, "ipc tx succeeded",
					       hdr->cmd);
	}

	return ret;
}

static int ipc_mailbox_init(struct snd_sof_dev *sdev, u32 dspbox,
			     size_t dspbox_size, u32 hostbox,
			     size_t hostbox_size)
{
	sdev->dsp_box.offset = dspbox;
	sdev->dsp_box.size = dspbox_size;
	sdev->host_box.offset = hostbox;
	sdev->host_box.size = hostbox_size;
	return 0;
}

static int ipc_ping(struct snd_sof_dev *sdev)
{
	struct sof_ipc_cmd_hdr hdr;
	struct sof_ipc_reply reply;

	/* configure test IPC */
	hdr.cmd = SOF_IPC_GLB_TEST_MSG | SOF_IPC_TEST_IPC_FLOOD;
	hdr.size = sizeof(hdr);

	return sof_ipc_tx_message(sdev->ipc, hdr.cmd, &hdr, hdr.size,
				  &reply, sizeof(reply));
}

const struct ipc_ops legacy_ipc_ops = {
	/* IPC */
	.ipc_mailbox_init	= ipc_mailbox_init,
	.ipc_msgs_rx		= ipc_msgs_rx,
	.ipc_log_header		= ipc_log_header,
	.ipc_get_reply		= ipc_get_reply,
	.ipc_is_valid		= ipc_is_sof,
	.ipc_tx_wait_done	= ipc_tx_wait_done,
	.ipc_ping		= ipc_ping,

	/* stream */
	.stream_update_posn	= stream_update_posn,
	.stream_notification	= stream_notification,	
	.stream_dai_posn_bytes	= stream_dai_posn_bytes,
	.stream_host_posn_bytes	= stream_host_posn_bytes,

	/* DAI */
	.dai_hda_link_config	= dai_hda_link_config,
	.dai_link_fixup		= dai_link_fixup,
	.dai_hda_stream_config	= dai_hda_stream_config,

	/* controls */
	.kctrl_volume_to_ipc	= kctrl_volume_to_ipc,
	.kctrl_ipc_to_volume	= kctrl_ipc_to_volume,
	.kctrl_value_to_ipc	= kctrl_value_to_ipc,
	.kctrl_ipc_to_value	= kctrl_ipc_to_value,
	.kctrl_set_get_comp_data= kctrl_set_get_comp_data,
	.kctrl_bytes_get	= kctrl_bytes_get,
	.kctrl_bytes_put	= kctrl_bytes_put,
	.kctrl_bytes_ext_get	= kctrl_bytes_ext_get,
	.kctrl_bytes_ext_put	= kctrl_bytes_ext_put,

	/* topology */
	.complete_pipeline	= tplg_complete_pipeline,

	/* PM */
	.pm_restore_kcontrol	= pm_restore_kcontrol,
	.pm_restore_pipeline	= pm_restore_pipeline,
	.pm_restore_connection	= pm_restore_connection,
	.pm_restore_dai_link	= pm_restore_dai_link,
	.pm_set_state		= pm_set_state,

	/* trace */
	.trace_enable		= trace_enable,

	/* PCM */
	.pcm_open		= pcm_open,
	.pcm_close		= pcm_close,
	.pcm_trigger		= pcm_trigger,
	.pcm_free		= pcm_free,
	.pcm_hw_params		= pcm_hw_params,

	/* debug */
	.panic_get_registers	= panic_get_registers,
	.panic_is_exception	= panic_is_exception,
	.panic_get_status	= panic_get_status,
};


/*
 * IPC Firmware ready, feature enumeration and boot.
 */

static int get_ext_windows(struct snd_sof_dev *sdev,
			   struct sof_ipc_ext_data_hdr *ext_hdr)
{
	struct sof_ipc_window *w =
		container_of(ext_hdr, struct sof_ipc_window, ext_hdr);

	if (w->num_windows == 0 || w->num_windows > SOF_IPC_MAX_ELEMS)
		return -EINVAL;

	/* keep a local copy of the data */
	sdev->info_window = kmemdup(w, struct_size(w, window, w->num_windows),
				    GFP_KERNEL);
	if (!sdev->info_window)
		return -ENOMEM;

	return 0;
}

/* parse the extended FW boot data structures from FW boot message */
int snd_sof_fw_parse_ext_data(struct snd_sof_dev *sdev, u32 bar, u32 offset)
{
	struct sof_ipc_ext_data_hdr *ext_hdr;
	void *ext_data;
	int ret = 0;

	ext_data = kzalloc(PAGE_SIZE, GFP_KERNEL);
	if (!ext_data)
		return -ENOMEM;

	/* get first header */
	snd_sof_dsp_block_read(sdev, bar, offset, ext_data,
			       sizeof(*ext_hdr));
	ext_hdr = ext_data;

	while (ext_hdr->hdr.cmd == SOF_IPC_FW_READY) {
		/* read in ext structure */
		offset += sizeof(*ext_hdr);
		snd_sof_dsp_block_read(sdev, bar, offset,
				   (void *)((u8 *)ext_data + sizeof(*ext_hdr)),
				   ext_hdr->hdr.size - sizeof(*ext_hdr));

		dev_dbg(sdev->dev, "found ext header type %d size 0x%x\n",
			ext_hdr->type, ext_hdr->hdr.size);

		/* process structure data */
		switch (ext_hdr->type) {
		case SOF_IPC_EXT_DMA_BUFFER:
			break;
		case SOF_IPC_EXT_WINDOW:
			ret = get_ext_windows(sdev, ext_hdr);
			break;
		default:
			break;
		}

		if (ret < 0) {
			dev_err(sdev->dev, "error: failed to parse ext data type %d\n",
				ext_hdr->type);
			break;
		}

		/* move to next header */
		offset += ext_hdr->hdr.size;
		snd_sof_dsp_block_read(sdev, bar, offset, ext_data,
				       sizeof(*ext_hdr));
		ext_hdr = ext_data;
	}

	kfree(ext_data);
	return ret;
}
EXPORT_SYMBOL(snd_sof_fw_parse_ext_data);

static void sof_get_windows(struct snd_sof_dev *sdev)
{
	struct sof_ipc_window_elem *elem;
	u32 outbox_offset = 0;
	u32 stream_offset = 0;
	u32 inbox_offset = 0;
	u32 outbox_size = 0;
	u32 stream_size = 0;
	u32 inbox_size = 0;
	int window_offset;
	int bar;
	int i;

	if (!sdev->info_window) {
		dev_err(sdev->dev, "error: have no window info\n");
		return;
	}

	bar = snd_sof_dsp_get_bar_index(sdev, SOF_FW_BLK_TYPE_SRAM);
	if (bar < 0) {
		dev_err(sdev->dev, "error: have no bar mapping\n");
		return;
	}

	for (i = 0; i < sdev->info_window->num_windows; i++) {
		elem = &sdev->info_window->window[i];

		window_offset = snd_sof_dsp_get_window_offset(sdev, elem->id);
		if (window_offset < 0) {
			dev_warn(sdev->dev, "warn: no offset for window %d\n",
				 elem->id);
			continue;
		}

		switch (elem->type) {
		case SOF_IPC_REGION_UPBOX:
			inbox_offset = window_offset + elem->offset;
			inbox_size = elem->size;
			snd_sof_debugfs_io_item(sdev,
						sdev->bar[bar] +
						inbox_offset,
						elem->size, "inbox",
						SOF_DEBUGFS_ACCESS_D0_ONLY);
			break;
		case SOF_IPC_REGION_DOWNBOX:
			outbox_offset = window_offset + elem->offset;
			outbox_size = elem->size;
			snd_sof_debugfs_io_item(sdev,
						sdev->bar[bar] +
						outbox_offset,
						elem->size, "outbox",
						SOF_DEBUGFS_ACCESS_D0_ONLY);
			break;
		case SOF_IPC_REGION_TRACE:
			snd_sof_debugfs_io_item(sdev,
						sdev->bar[bar] +
						window_offset +
						elem->offset,
						elem->size, "etrace",
						SOF_DEBUGFS_ACCESS_D0_ONLY);
			break;
		case SOF_IPC_REGION_DEBUG:
			snd_sof_debugfs_io_item(sdev,
						sdev->bar[bar] +
						window_offset +
						elem->offset,
						elem->size, "debug",
						SOF_DEBUGFS_ACCESS_D0_ONLY);
			break;
		case SOF_IPC_REGION_STREAM:
			stream_offset = window_offset + elem->offset;
			stream_size = elem->size;
			snd_sof_debugfs_io_item(sdev,
						sdev->bar[bar] +
						stream_offset,
						elem->size, "stream",
						SOF_DEBUGFS_ACCESS_D0_ONLY);
			break;
		case SOF_IPC_REGION_REGS:
			snd_sof_debugfs_io_item(sdev,
						sdev->bar[bar] +
						window_offset +
						elem->offset,
						elem->size, "regs",
						SOF_DEBUGFS_ACCESS_D0_ONLY);
			break;
		case SOF_IPC_REGION_EXCEPTION:
			sdev->dsp_oops_offset = window_offset + elem->offset;
			snd_sof_debugfs_io_item(sdev,
						sdev->bar[bar] +
						window_offset +
						elem->offset,
						elem->size, "exception",
						SOF_DEBUGFS_ACCESS_D0_ONLY);
			break;
		default:
			dev_err(sdev->dev, "error: get illegal window info\n");
			return;
		}
	}

	if (outbox_size == 0 || inbox_size == 0) {
		dev_err(sdev->dev, "error: get illegal mailbox window\n");
		return;
	}

	snd_sof_dsp_mailbox_init(sdev, inbox_offset, inbox_size,
				 outbox_offset, outbox_size);
	sdev->stream_box.offset = stream_offset;
	sdev->stream_box.size = stream_size;

	dev_dbg(sdev->dev, " mailbox upstream 0x%x - size 0x%x\n",
		inbox_offset, inbox_size);
	dev_dbg(sdev->dev, " mailbox downstream 0x%x - size 0x%x\n",
		outbox_offset, outbox_size);
	dev_dbg(sdev->dev, " stream region 0x%x - size 0x%x\n",
		stream_offset, stream_size);
}

int ipc_legacy_v1_validate(struct snd_sof_dev *sdev, u32 token, int bar,
			   int offset)
{
	struct sof_ipc_fw_ready ready;
	struct sof_ipc_fw_version *v = &ready.version;
	struct ipc_data *idata;

	/* copy data from the DSP FW ready offset */
	sof_block_read(sdev, bar, offset, &ready, sizeof(ready));

	/* first validate */
	if (ready.hdr.cmd != SOF_IPC_FW_READY)
		return -EINVAL;

	/* ok initially matches, now check version */
	dev_info(sdev->dev,
		 "Firmware info: version %d:%d:%d-%s\n",  v->major, v->minor,
		 v->micro, v->tag);
	dev_info(sdev->dev,
		 "Firmware: ABI %d:%d:%d Kernel ABI %d:%d:%d\n",
		 SOF_ABI_VERSION_MAJOR(v->abi_version),
		 SOF_ABI_VERSION_MINOR(v->abi_version),
		 SOF_ABI_VERSION_PATCH(v->abi_version),
		 SOF_ABI_MAJOR, SOF_ABI_MINOR, SOF_ABI_PATCH);

	if (SOF_ABI_VERSION_INCOMPATIBLE(SOF_ABI_VERSION, v->abi_version)) {
		dev_err(sdev->dev, "error: incompatible FW ABI version\n");
		return -EINVAL;
	}

	if (v->abi_version > SOF_ABI_VERSION) {
		if (!IS_ENABLED(CONFIG_SND_SOC_SOF_STRICT_ABI_CHECKS)) {
			dev_warn(sdev->dev, "warn: FW ABI is more recent than kernel\n");
		} else {
			dev_err(sdev->dev, "error: FW ABI is more recent than kernel\n");
			return -EINVAL;
		}
	}

	if (ready.flags & SOF_IPC_INFO_BUILD) {
		dev_info(sdev->dev,
			 "Firmware debug build %d on %s-%s - options:\n"
			 " GDB: %s\n"
			 " lock debug: %s\n"
			 " lock vdebug: %s\n",
			 v->build, v->date, v->time,
			 (ready.flags & SOF_IPC_INFO_GDB) ?
				"enabled" : "disabled",
			 (ready.flags & SOF_IPC_INFO_LOCKS) ?
				"enabled" : "disabled",
			 (ready.flags & SOF_IPC_INFO_LOCKSV) ?
				"enabled" : "disabled");
	}

	/* copy the fw_version into debugfs at first boot */
	idata = devm_kzalloc(sdev->dev, sizeof(*idata), GFP_KERNEL);
	if (!idata)
		return -ENOMEM;
	memcpy(&idata->fw_version, v, sizeof(*v));
	sdev->ipc_private = idata;

	/* setup ops for IPC and topology */
	sdev->tplg_ops = &sof_legacy_v1_tplg_ops;
	sdev->ipc_ops = &legacy_ipc_ops;

	/* now check for extended data */
	snd_sof_fw_parse_ext_data(sdev, bar, offset +
				  sizeof(struct sof_ipc_fw_ready));

	sof_get_windows(sdev);

	return 0;
}
EXPORT_SYMBOL(ipc_legacy_v1_validate);
