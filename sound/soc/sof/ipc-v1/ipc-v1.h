/* SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause) */
/*
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * Copyright(c) 2019 Intel Corporation. All rights reserved.
 *
 * Author: Liam Girdwood <liam.r.girdwood@linux.intel.com>
 */

#ifndef __SOUND_SOC_SOF_IPC_V1_IPC_V1_H
#define __SOUND_SOC_SOF_IPC_V1_IPC_V1_H

#include "../sof-priv.h"
#include "../ops.h"
#include "../ipc-ops.h"
#include "../intel/hda.h"

#define EXCEPT_MAX_HDR_SIZE	0x400

struct sof_ipc_ctrl_data_params {
	size_t msg_bytes;
	size_t hdr_bytes;
	size_t pl_size;
	size_t elems;
	u32 num_msg;
	u8 *src;
	u8 *dst;
};

struct ipc_data {
	struct sof_ipc_fw_ready fw_ready;
	struct sof_ipc_fw_version fw_version;
};

/*
 * IPC Topology Ops - outside of parser.
 */
int tplg_complete_pipeline(struct snd_sof_dev *sdev,
			   struct snd_sof_widget *swidget);

/*
 * Trace IPC specific ops.
 */
void trace_process_message(struct snd_sof_dev *sdev, u32 msg_id);
int trace_update_pos(struct snd_sof_dev *sdev,
		     struct sof_ipc_dma_trace_posn *posn);
int trace_enable(struct snd_sof_dev *sdev);

/*
 * PM IPC specific ops.
 */
int pm_restore_kcontrol(struct snd_sof_dev *sdev,
			struct snd_sof_control *scontrol);
int pm_restore_pipeline(struct snd_sof_dev *sdev,
			struct snd_sof_widget *swidget);
int pm_restore_connection(struct snd_sof_dev *sdev,
			  struct snd_sof_route *sroute);
int pm_restore_dai_link(struct snd_sof_dev *sdev,
			struct snd_sof_dai *dai);
int pm_set_state(struct snd_sof_dev *sdev, int cmd);

/*
 * PM IPC specific ops.
 */
int pcm_open(struct snd_sof_dev *sdev, struct snd_pcm_substream *substream,
	     struct snd_sof_pcm *spcm);
int pcm_close(struct snd_sof_dev *sdev, struct snd_pcm_substream *substream,
	      struct snd_sof_pcm *spcm);
int pcm_trigger(struct snd_sof_dev *sdev,
		struct snd_pcm_substream *substream,
		struct snd_sof_pcm *spcm, int cmd, bool *reset_needed);
int pcm_hw_params(struct snd_sof_dev *sdev,
		  struct snd_pcm_substream *substream,
		  struct snd_pcm_hw_params *params,
		  struct snd_sof_pcm *spcm, int new_buffer);
int pcm_dsp_params(struct snd_sof_pcm *spcm,
		   struct snd_pcm_substream *substream,
		   const struct sof_ipc_pcm_params_reply *reply);
int pcm_free(struct snd_sof_dev *sdev, struct snd_pcm_substream *substream,
	     struct snd_sof_pcm *spcm);

/*
 * Panic/Debug IPC specific ops.
 */
void panic_get_status(struct snd_sof_dev *sdev, u32 panic_code,
		      u32 tracep_code, struct snd_sof_oops *oops);
void panic_get_registers(struct snd_sof_dev *sdev,
			 struct snd_sof_oops *oops);
int panic_is_exception(struct snd_sof_dev *sdev, u32 token);

/*
 * Kcontrol specific IPC ops.
 */
void kctrl_volume_to_ipc(struct snd_sof_dev *sdev, void *control_data,
			 unsigned int channel, unsigned int value,
			 u32 *volume_map, int size);
u32 kctrl_ipc_to_volume(struct snd_sof_dev *sdev, void *control_data,
			u32 channel, u32 *volume_map, int size);
void kctrl_value_to_ipc(struct snd_sof_dev *sdev, void *control_data,
			unsigned int channel, unsigned int value);
u32 kctrl_ipc_to_value(struct snd_sof_dev *sdev, void *control_data,
		       u32 channel);
int kctrl_bytes_get(struct snd_sof_dev *sdev, struct snd_kcontrol *kcontrol,
		    struct snd_ctl_elem_value *ucontrol);
int kctrl_bytes_put(struct snd_sof_dev *sdev, struct snd_kcontrol *kcontrol,
		    struct snd_ctl_elem_value *ucontrol);
int kctrl_bytes_ext_get(struct snd_sof_dev *sdev,
			struct snd_kcontrol *kcontrol,
			unsigned int __user *binary_data,
			unsigned int size);
int kctrl_bytes_ext_put(struct snd_sof_dev *sdev,
			struct snd_kcontrol *kcontrol,
			struct snd_ctl_tlv *header,
			const unsigned int __user *binary_data,
			unsigned int size);
int kctrl_set_get_comp_data(struct snd_sof_ipc *ipc,
			    struct snd_sof_control *scontrol, u32 ipc_cmd,
			    int ctrl_type, int ctrl_cmd, bool send);

/*
 * Stream specific IPC ops.
 */
int stream_dai_posn_bytes(struct snd_sof_dev *sdev, void *posn);
int stream_host_posn_bytes(struct snd_sof_dev *sdev, void *posn);
int stream_update_posn(struct snd_sof_dev *sdev,
		       struct snd_sof_pcm *spcm, int direction,
		       void *posn);
void stream_notification(struct snd_sof_dev *sdev, u32 msg_cmd);

/*
 * DAI specific ops.
 */
int dai_link_fixup(struct snd_sof_dev *sdev,
		   struct snd_soc_pcm_runtime *rtd,
		   struct snd_pcm_hw_params *params);
int dai_hda_stream_config(struct snd_sof_dev *sdev,
			  struct hdac_stream *hstream, void *params);
int dai_hda_link_config(struct snd_sof_dev *sdev, void *hdastream,
			const char *dai_name, int channel, int dir);

/*
 * Topology IPC ops.
 */

int tplg_complete_pipeline(struct snd_sof_dev *sdev,
			   struct snd_sof_widget *swidget);

int sof_load_pipeline_ipc(struct snd_sof_dev *sdev,
			  struct sof_ipc_pipe_new *pipeline,
			  struct sof_ipc_comp_reply *r);
#endif
