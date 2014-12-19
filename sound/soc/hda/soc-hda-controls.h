/*
 *  soc-hda-controls.h - Intel HDA Platform controls header file
 *
 *  Copyright (C) 2014 Intel Corp
 *  Author: Jeeja KP <jeeja.kp@intel.com>
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

#ifndef __SOC_HDA_CONTROLS_H__
#define __SOC_HDA_CONTROLS_H__

#include <linux/types.h>
#include "soc-hda-vendor.h"

#define BITS_PER_BYTE 8

/* Maximum number of coefficients up down mixer module */
#define UP_DOWN_MIXER_MAX_COEFF		6

enum channel_index {
	CHANNEL_LEFT = 0,
	CHANNEL_CENTER = 1,
	CHANNEL_RIGHT = 2,
	CHANNEL_LEFT_SURROUND = 3,
	CHANNEL_CENTER_SURROUND = 3,
	CHANNEL_RIGHT_SURROUND = 4,
	CHANNEL_LFE = 7,
	CHANNEL_INVALID = 0xF,
};

enum bit_depth {
	DEPTH_8BIT = 8,
	DEPTH_16BIT = 16,
	DEPTH_24BIT = 24, /**< Default. */
	DEPTH_32BIT = 32,
	DEPTH_INVALID
};

enum interleaving_style {
	INTERLEAVING_PER_CHANNEL = 0, /* [s1_ch1...s1_chN,...,sM_ch1...sM_chN] */
	INTERLEAVING_PER_SAMPLE = 1, /* [s1_ch1...sM_ch1,...,s1_chN...sM_chN] */
};

enum sampling_frequency {
	FS_8000HZ = 8000,
	FS_11025HZ = 11025,
	FS_12000HZ = 12000, /** Mp3, AAC, SRC only. */
	FS_16000HZ = 16000,
	FS_22050HZ = 22050,
	FS_24000HZ = 24000, /** Mp3, AAC, SRC only. */
	FS_32000HZ = 32000,
	FS_44100HZ = 44100,
	FS_48000HZ = 48000, /**< Default. */
	FS_64000HZ = 64000, /** AAC, SRC only. */
	FS_88200HZ = 88200, /** AAC, SRC only. */
	FS_96000HZ = 96000, /** AAC, SRC only. */
	FS_128000HZ = 128000, /** SRC only. */
	FS_176400HZ = 176400, /** SRC only. */
	FS_192000HZ = 192000, /** SRC only. */
	FS_INVALID
};

enum widget_type {
	HDA_SST_WIDGET_VMIXER = 1,
	HDA_SST_WIDGET_MIXER = 2,
	HDA_SST_WIDGET_PGA = 3,
	HDA_SST_WIDGET_MUX = 4
};

struct audio_data_format {
	enum sampling_frequency sampling_frequency;
	enum bit_depth bit_depth;
	u32 channel_map;
	enum channel_config channel_config;
	enum interleaving_style interleaving_style;
	u8 number_of_channels;
	u8 valid_bit_depth;
	u8 sample_type;
	u8 reserved[1];
} __packed;

struct sst_base_module_config {
	u32 cps;
	u32 ibs;
	u32 obs;
	u32 is_pages;
	struct audio_data_format audio_fmt;
};

struct sst_copiergateway_cfg {
	u32 node_id;
	u32 dma_buffer_size;
	u32 config_length;
	u32 config_data[1]; /*not mandatory; required only for DMIC/I2S */
} __packed;

struct sst_copier_module_config {
	struct sst_base_module_config base_module_config;
	struct audio_data_format out_fmt;
	u32 cpr_feature_mask;
	struct sst_copiergateway_cfg gtw_cfg;
} __packed;

struct sst_src_config {
	enum sampling_frequency s_freq;
} __packed;

struct sst_src_module_config {
	struct sst_base_module_config base_module_config;
	struct sst_src_config src_config;
} __packed;

struct sst_up_down_mixer_module_config {
	struct sst_base_module_config base_module_config;
	enum channel_config out_channel_config;
	/* This should be set to 1 if user coefficients are required */
	u32 coeff_sel;
	/* Pass the user coeff in this array */
	s32 coeff[UP_DOWN_MIXER_MAX_COEFF];
} __packed;

enum dma_type {
	HDA_HOST_OUTPUT_CLASS = 0,
	HDA_HOST_INPUT_CLASS = 1,
	HDA_HOST_INOUT_CLASS = 2,
	HDA_LINK_OUTPUT_CLASS = 8,
	HDA_LINK_INPUT_CLASS = 9,
	HDA_LINK_INOUT_CLASS = 0xA,
	DMIC_LINK_INPUT_CLASS = 0xB,
	I2S_LINK_OUTPUT_CLASS = 0xC,
	I2S_LINK_INPUT_CLASS = 0xD,
	SLIMBUS_LINK_OUTPUT_CLASS = 0xE,
	SLIMBUS_LINK_INPUT_CLASS = 0xF
};

union ssp_dma_node {
	u8 val;
	struct {
		u8 dual_mono:1;
		u8 time_slot:3;
		u8 i2s_instance:4;
	} dma_node;
};

union connector_node_id {
	u32 val;
	struct {
		u32 dma_id:8; /* DMA engine ID */
		u32 dma_type:4;
		u32 rsvd:20;
	} node;
};

enum stream_state {
	RESET_STATE = 0,
	STOP_STATE = 1,
	PAUSE_STATE = 1,
	RUN_STATE = 2
};

enum fw_pipeline_type {
	DECODE = 0,
	MIXER = 1,
	REFERENCE = 2,
	CAPTURE = 3
};

enum core_affinity {
	CORE_0 = 0,
	CORE_1,
	CORE_max
};

struct module_format {
	u32 channels;
	u32 sampling_freq;
	u32 bit_depth;
	u32 valid_bit_depth;
	u32 channel_config;
};

struct module_instance_id {
	u32 module_id;
	u32 instance_id;
};

struct specific_config {
	u32 caps_size;
	u32 *caps;
};

enum pipe_state {
	INVALID = 0,
	CREATED = 1,
	STARTED = 2
};

struct sst_pipe {
	u8 ppl_id;
	u8 pipe_type;
	u16 conn_type;
	u32 memory_pages;
	enum pipe_state state;
};


enum sst_module_state {
	UNINIT = 0,
	INIT_DONE = 1,
	LOADED = 2,
	UNLOADED = 3,
	BIND_DONE = 4
};

struct module_config {
	struct module_instance_id id;
	struct module_format in_fmt;
	struct module_format out_fmt;
	u8 max_in_queue;
	u8 max_out_queue;
	u8 in_queue_mask;
	u8 out_queue_mask;
	u8 in_queue;
	u8 out_queue;
	u32 mcps;
	u32 ibs;
	u32 obs;
	u8 is_loadable;
	u8 core_id;
	u8 conn_type;
	u8 dev_type;
	u8 dma_id;
	u8 time_slot;
	u32 params_fixup;
	u32 converter;
	enum hw_connection_type hw_conn_type;
	enum sst_module_state m_state;
	struct sst_pipe *pipe;
	struct specific_config formats_config;
};

struct hda_sst_algo_data {
	enum algo_kcontrol_type type;
	u32 max;
	char *params;
};

struct sst_pipeline {
	struct sst_pipe *pipe;
	struct list_head node;
};

int hda_sst_create_pipeline(struct sst_dsp_ctx *ctx, struct sst_pipe *pipe);
int hda_sst_run_pipe(struct sst_dsp_ctx *ctx, struct sst_pipe *pipe);
int hda_sst_pause_pipe(struct sst_dsp_ctx *ctx, struct sst_pipe *pipe);
int hda_sst_delete_pipe(struct sst_dsp_ctx *ctx, struct sst_pipe *pipe);
int hda_sst_stop_pipe(struct sst_dsp_ctx *ctx, struct sst_pipe *pipe);
int hda_sst_init_module(struct sst_dsp_ctx *ctx, struct module_config *module_config,
	struct hda_sst_algo_data *ac);
int hda_sst_bind_unbind_modules(struct sst_dsp_ctx *ctx, struct module_config
	*src_module, struct module_config *dst_module, bool bind);
enum bit_depth hda_sst_get_bit_depth(struct snd_pcm_hw_params *params);
#endif
