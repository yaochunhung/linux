/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * Copyright(c) 2018 Intel Corporation. All rights reserved.
 *
 * Author: Liam Girdwood <liam.r.girdwood@linux.intel.com>
 */

#ifndef __SOUND_SOC_SOF_IPC_OPS_H
#define __SOUND_SOC_SOF_IPC_OPS_H

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/types.h>
#include <sound/pcm.h>
#include "sof-priv.h"

#define sof_ops_ipc(sdev) \
	((sdev)->ipc_ops)

/* Mandatory operations are verified during probing */


/*
 * IPC & topology flavour abstraction.
 */
static inline void snd_sof_ipc_log_header(struct snd_sof_dev *sdev, u8 *text,
					  u32 cmd)
{
	sof_ops_ipc(sdev)->ipc_log_header(sdev, text, cmd);
}

static inline void snd_sof_ipc_msgs_rx(struct snd_sof_dev *sdev)
{
	sof_ops_ipc(sdev)->ipc_msgs_rx(sdev);
}

static inline int snd_sof_ipc_tx_wait_done(struct snd_sof_dev *sdev,
					   struct snd_sof_ipc *ipc,
					   struct snd_sof_ipc_msg *msg,
					   void *reply_data)
{
	return sof_ops_ipc(sdev)->ipc_tx_wait_done(sdev, ipc, msg,
						   reply_data);
}

static inline int snd_sof_ipc_ping(struct snd_sof_dev *sdev)
{
	return sof_ops_ipc(sdev)->ipc_ping(sdev);
}

static inline int snd_sof_dsp_mailbox_init(struct snd_sof_dev *sdev, u32 dspbox,
				size_t dspbox_size, u32 hostbox,
				size_t hostbox_size)
{
	return sof_ops_ipc(sdev)->ipc_mailbox_init(sdev, dspbox, dspbox_size,
						   hostbox, hostbox_size);
}

static inline void snd_sof_ipc_get_reply(struct snd_sof_dev *sdev)
{
	sof_ops_ipc(sdev)->ipc_get_reply(sdev);
}

static inline bool snd_sof_ipc_is_valid(struct snd_sof_dev *sdev, u32 mask,
					u32 msg)
{
	return sof_ops_ipc(sdev)->ipc_is_valid(sdev, mask, msg);
}

/*
 * Kcontrol APIs.
 */
static inline void snd_sof_volume_to_ipc(struct snd_sof_dev *sdev,
					 void *control_data,
					 unsigned int channel,
					 unsigned int value, u32 *volume_map,
					 int size)
{
	sof_ops_ipc(sdev)->kctrl_volume_to_ipc(sdev, control_data, channel,
					       value, volume_map, size);
}

static inline u32 snd_sof_ipc_to_volume(struct snd_sof_dev *sdev, 
					void *control_data, u32 channel,
					u32 *volume_map,
					int size)
{
	return sof_ops_ipc(sdev)->kctrl_ipc_to_volume(sdev, control_data,
						      channel, volume_map,
						      size);
}

static inline void snd_sof_value_to_ipc(struct snd_sof_dev *sdev,
					 void *control_data,
					 unsigned int channel,
					 unsigned int value)
{
	sof_ops_ipc(sdev)->kctrl_value_to_ipc(sdev, control_data, channel,
					      value);
}

static inline u32 snd_sof_ipc_to_value(struct snd_sof_dev *sdev,
					void *control_data, u32 channel)
{
	return sof_ops_ipc(sdev)->kctrl_ipc_to_value(sdev, control_data,
						     channel);
}

static inline int snd_sof_control_bytes_get(struct snd_sof_dev *sdev,
					    struct snd_kcontrol *kcontrol,
					    struct snd_ctl_elem_value *ucontrol)
{
	return sof_ops_ipc(sdev)->kctrl_bytes_get(sdev, kcontrol, ucontrol);
}

static inline int snd_sof_control_bytes_put(struct snd_sof_dev *sdev,
					    struct snd_kcontrol *kcontrol,
					    struct snd_ctl_elem_value *ucontrol)
{
	return sof_ops_ipc(sdev)->kctrl_bytes_put(sdev, kcontrol, ucontrol);
}

static inline int snd_sof_control_bytes_ext_get(struct snd_sof_dev *sdev,
						struct snd_kcontrol *kcontrol,
						unsigned int __user *binary_data,
						unsigned int size)
{
	return sof_ops_ipc(sdev)->kctrl_bytes_ext_get(sdev, kcontrol,
						      binary_data, size);
}

static inline int snd_sof_control_bytes_ext_put(struct snd_sof_dev *sdev,
						struct snd_kcontrol *kcontrol,
						struct snd_ctl_tlv *header,
						const unsigned int __user *binary_data,
						unsigned int size)
{
	return sof_ops_ipc(sdev)->kctrl_bytes_ext_put(sdev, kcontrol,
						      header, binary_data,
						      size);
}

static inline int snd_sof_ipc_set_get_comp_data(struct snd_sof_ipc *ipc,
						struct snd_sof_control *scontrol,
						u32 ipc_cmd,
						u64 ctrl_type,
						u64 ctrl_cmd,
						bool send)
{
	return sof_ops_ipc(ipc->sdev)->kctrl_set_get_comp_data(ipc, scontrol,
							       ipc_cmd,
							       ctrl_type,
							       ctrl_cmd,
							       send);
}

/*
 * PM ops.
 */

static inline int snd_sof_restore_pipeline(struct snd_sof_dev *sdev,
			    struct snd_sof_widget *swidget)
{
	return sof_ops_ipc(sdev)->pm_restore_pipeline(sdev, swidget);
}

static inline int snd_sof_restore_connection(struct snd_sof_dev *sdev,
			      struct snd_sof_route *sroute)
{
	return sof_ops_ipc(sdev)->pm_restore_connection(sdev, sroute);
}

static inline int snd_sof_restore_dai_link(struct snd_sof_dev *sdev,
			    struct snd_sof_dai *dai)
{
	return sof_ops_ipc(sdev)->pm_restore_dai_link(sdev, dai);
}

static inline int snd_sof_restore_kcontrol(struct snd_sof_dev *sdev,
			    struct snd_sof_control *scontrol)
{
	return sof_ops_ipc(sdev)->pm_restore_kcontrol(sdev, scontrol);
} 

static inline int snd_sof_send_pm_ipc(struct snd_sof_dev *sdev, int cmd)
{
	return sof_ops_ipc(sdev)->pm_set_state(sdev, cmd);
}

/*
 * Trace
 */

static inline int snd_sof_init_trace_ipc(struct snd_sof_dev *sdev)
{
	return sof_ops_ipc(sdev)->trace_enable(sdev);
}


/*
 * PCM
 */

static inline int snd_sof_pcm_open(struct snd_sof_dev *sdev,
				   struct snd_pcm_substream *substream,
				   struct snd_sof_pcm *spcm)
{
	return sof_ops_ipc(sdev)->pcm_open(sdev, substream, spcm);
}

static inline int snd_sof_pcm_close(struct snd_sof_dev *sdev,
				    struct snd_pcm_substream *substream,
				    struct snd_sof_pcm *spcm)
{
	return sof_ops_ipc(sdev)->pcm_close(sdev, substream, spcm);
}

static inline int snd_sof_pcm_trigger(struct snd_sof_dev *sdev,
				      struct snd_pcm_substream *substream,
				      struct snd_sof_pcm *spcm, int cmd,
				      bool *reset_needed)
{
	return sof_ops_ipc(sdev)->pcm_trigger(sdev, substream, spcm, cmd,
					      reset_needed);
}

static inline int snd_sof_pcm_free(struct snd_sof_dev *sdev,
				      struct snd_pcm_substream *substream,
				      struct snd_sof_pcm *spcm)
{
	return sof_ops_ipc(sdev)->pcm_free(sdev, substream, spcm);
}

static inline int snd_sof_pcm_hw_params(struct snd_sof_dev *sdev,
					struct snd_pcm_substream *substream,
			 		struct snd_pcm_hw_params *params,
			 		struct snd_sof_pcm *spcm,
					int new_buffer)
{
	return sof_ops_ipc(sdev)->pcm_hw_params(sdev, substream, params, spcm,
						new_buffer);
}

/*
 * Debug/Panic.
 */
static inline void snd_sof_panic_get_status(struct snd_sof_dev *sdev,
					    u32 panic_code,
					    u32 tracep_code,
					    struct snd_sof_oops *oops)
{
	sof_ops_ipc(sdev)->panic_get_status(sdev, panic_code, tracep_code,
					    oops);
}

static inline void snd_sof_oops_get_registers(struct snd_sof_dev *sdev,
					      struct snd_sof_oops *oops)
{
	sof_ops_ipc(sdev)->panic_get_registers(sdev, oops);
}


static inline int snd_sof_is_exception(struct snd_sof_dev *sdev, u32 token)
{
	return sof_ops_ipc(sdev)->panic_is_exception(sdev, token);
}

/*
 * DAI.
 */
static inline int snd_sof_set_hda_stream_config(struct snd_sof_dev *sdev,
						struct hdac_stream *hstream,
						void *params)
{
	return sof_ops_ipc(sdev)->dai_hda_stream_config(sdev, hstream, params);
}

static inline int snd_sof_set_hda_link_config(struct snd_sof_dev *sdev,
					      void *hdastream,
					      const char *dai_name,
					      int channel, int dir)
{
	return sof_ops_ipc(sdev)->dai_hda_link_config(sdev, hdastream,
						      dai_name, channel, dir);
}

static inline int snd_sof_dai_link_fixup(struct snd_sof_dev *sdev,
					 struct snd_soc_pcm_runtime *rtd,
					 struct snd_pcm_hw_params *params)
{
	return sof_ops_ipc(sdev)->dai_link_fixup(sdev, rtd, params);
}

/*
 * Stream.
 */
static inline int snd_sof_host_posn_bytes(struct snd_sof_dev *sdev, void *posn)
{
	return sof_ops_ipc(sdev)->stream_host_posn_bytes(sdev, posn);
}

static inline int snd_sof_dai_posn_bytes(struct snd_sof_dev *sdev, void *posn)
{
	return sof_ops_ipc(sdev)->stream_dai_posn_bytes(sdev, posn);
}

static inline int snd_sof_ipc_stream_posn(struct snd_sof_dev *sdev,
					  struct snd_sof_pcm *spcm,
					  int direction,
					  void *posn)
{
	return sof_ops_ipc(sdev)->stream_update_posn(sdev, spcm, direction, posn);
}

static inline void snd_sof_ipc_stream_message(struct snd_sof_dev *sdev, u32 msg_cmd)
{
	sof_ops_ipc(sdev)->stream_notification(sdev, msg_cmd);
}

#endif
