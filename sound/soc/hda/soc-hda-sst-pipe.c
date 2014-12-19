/*
 *  soc_hda_sst_pipe.c - HDA DSP interface functions for FW pipe configuration
 *
 *  Copyright (C) 2014 Intel Corp
 *  Author:Rafal Redzimski <rafal.f.redzimski@intel.com>
 *	   Jeeja KP <jeeja.kp@intel.com>
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
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#include <linux/slab.h>
#include <sound/core.h>
#include <sound/pcm_params.h>
#include <sound/pcm.h>
#include <sound/soc-hda-sst-dsp.h>
#include <sound/soc-hda-sst-ipc.h>
#include "soc-hda-controls.h"

enum bit_depth hda_sst_get_bit_depth(struct snd_pcm_hw_params *params)
{

	switch (params_format(params)) {
	case SNDRV_PCM_FORMAT_S8:
		return DEPTH_8BIT;
	case SNDRV_PCM_FORMAT_S16_LE:
		return DEPTH_16BIT;
	case SNDRV_PCM_FORMAT_S24_LE:
		return DEPTH_24BIT;
	case SNDRV_PCM_FORMAT_S32_LE:
		return DEPTH_32BIT;
	default:
		return DEPTH_INVALID;
	}
}

static u32 hda_sst_create_channel_map(enum channel_config channel_config)
{
	u32 config;

	switch (channel_config) {
	case CHANNEL_CONFIG_MONO:
		config =  (0xFFFFFFF0 | CHANNEL_CENTER);
	break;
	case CHANNEL_CONFIG_STEREO:
		config = (0xFFFFFF00 | CHANNEL_LEFT
			| (CHANNEL_RIGHT << 4));
	break;
	case CHANNEL_CONFIG_2_1:
		config = (0xFFFFF000 | CHANNEL_LEFT
			| (CHANNEL_RIGHT << 4)
			| (CHANNEL_LFE << 8));
	break;
	case CHANNEL_CONFIG_3_0:
		config =  (0xFFFFF000 | CHANNEL_LEFT
			| (CHANNEL_CENTER << 4)
			| (CHANNEL_RIGHT << 8));
	break;
	case CHANNEL_CONFIG_3_1:
		config = (0xFFFF0000 | CHANNEL_LEFT
			| (CHANNEL_CENTER << 4)
			| (CHANNEL_RIGHT << 8)
			| (CHANNEL_LFE << 12));
	break;
	case CHANNEL_CONFIG_QUATRO:
		config = (0xFFFF0000 | CHANNEL_LEFT
			| (CHANNEL_RIGHT << 4)
			| (CHANNEL_LEFT_SURROUND << 8)
			| (CHANNEL_RIGHT_SURROUND << 12));
	break;
	case CHANNEL_CONFIG_4_0:
		config = (0xFFFF0000 | CHANNEL_LEFT
			| (CHANNEL_CENTER << 4)
			| (CHANNEL_RIGHT << 8)
			| (CHANNEL_CENTER_SURROUND << 12));
	break;
	case CHANNEL_CONFIG_5_0:
		config = (0xFFF00000 | CHANNEL_LEFT
			| (CHANNEL_CENTER << 4)
			| (CHANNEL_RIGHT << 8)
			| (CHANNEL_LEFT_SURROUND << 12)
			| (CHANNEL_RIGHT_SURROUND << 16));
	break;
	case CHANNEL_CONFIG_5_1:
		config = (0xFF000000 | CHANNEL_CENTER
			| (CHANNEL_LEFT << 4)
			| (CHANNEL_RIGHT << 8)
			| (CHANNEL_LEFT_SURROUND << 12)
			| (CHANNEL_RIGHT_SURROUND << 16)
			| (CHANNEL_LFE << 20));
	break;
	case CHANNEL_CONFIG_DUAL_MONO:
		config = (0xFFFFFF00 | CHANNEL_LEFT
			| (CHANNEL_LEFT << 4));
	break;
	case CHANNEL_CONFIG_I2S_DUAL_STEREO_0:
		config = (0xFFFFFF00 | CHANNEL_LEFT
			| (CHANNEL_RIGHT << 4));
	break;
	case CHANNEL_CONFIG_I2S_DUAL_STEREO_1:
		config = (0xFFFF00FF | (CHANNEL_LEFT << 8)
			| (CHANNEL_RIGHT << 12));
	break;
	default:
		config =  0xFFFFFFFF;
	}
	return config;
}

/*
 * Fills in base_module_config structure and sets config_size.
 * base_module_config is sent as input buffer with INIT_INSTANCE IPC msg
 */
static void hda_sst_set_base_module_format(struct sst_dsp_ctx *ctx,
	struct module_config *mconfig,
	struct sst_base_module_config *base_module_config, u16 *config_size)
{
	struct module_format *format = &mconfig->in_fmt;
	u32 *buffer = NULL;
	int i = 0;

	base_module_config->audio_fmt.number_of_channels =
		(u8)format->channels;

	base_module_config->audio_fmt.sampling_frequency = format->sampling_freq;
	base_module_config->audio_fmt.bit_depth = format->bit_depth;
	base_module_config->audio_fmt.valid_bit_depth = format->valid_bit_depth;
	base_module_config->audio_fmt.channel_config = format->channel_config;

	dev_dbg(ctx->dev, "bit_depth=%x valid_bd=%x ch_config=%x\n", format->bit_depth,
			format->valid_bit_depth, format->channel_config);

	base_module_config->audio_fmt.channel_map =
		hda_sst_create_channel_map(
			base_module_config->audio_fmt.channel_config);

	base_module_config->audio_fmt.interleaving_style =
		INTERLEAVING_PER_CHANNEL;

	base_module_config->cps = mconfig->mcps;
	base_module_config->ibs = mconfig->ibs;
	base_module_config->obs = mconfig->obs;
	base_module_config->is_pages = 0;

	*config_size = sizeof(*base_module_config);

	dev_dbg(ctx->dev, "Base module config size: %d bytes\n", *config_size);
	for (i = 0, buffer = (u32 *) base_module_config;
		i < (*config_size)/sizeof(u32);
		i++, buffer++)
		dev_dbg(ctx->dev, "0x%x: %08x\n", i, *buffer);

}

/*
 * Copies copier capabilities into copier module and updates copier module
 * config size.
 * In the end, config_size is the sum of the following:
 * (a) sizeof(struct sst_copier_module_config)
 * (b) copier capabilities size
 */
static void hda_sst_copy_copier_caps(struct module_config *mconfig,
	struct sst_copier_module_config *cpr_mconfig,
	u16 *config_size)
{
	if (mconfig->formats_config.caps_size == 0)
		return;

	memcpy(cpr_mconfig->gtw_cfg.config_data,
		mconfig->formats_config.caps,
		mconfig->formats_config.caps_size);

	cpr_mconfig->gtw_cfg.config_length =
		 (mconfig->formats_config.caps_size) / 4;
	*config_size += mconfig->formats_config.caps_size;
}

static void hda_sst_setup_cpr_gateway_cfg(struct sst_dsp_ctx *ctx,
	struct module_config *mconfig,
	struct sst_copier_module_config *cpr_mconfig, u16 *config_size)
{
	union connector_node_id node_id = {0};

	switch (mconfig->dev_type) {
	case DEVICE_BT:
		node_id.node.dma_type = (SOURCE == mconfig->hw_conn_type) ?
			I2S_LINK_OUTPUT_CLASS : I2S_LINK_INPUT_CLASS;
		node_id.node.dma_id = mconfig->dma_id;
		break;
	case DEVICE_I2S:
		node_id.node.dma_type = (SOURCE == mconfig->hw_conn_type) ?
			I2S_LINK_OUTPUT_CLASS : I2S_LINK_INPUT_CLASS;
		node_id.node.dma_id = mconfig->dma_id +
					 (mconfig->time_slot << 1);
		break;
	case DEVICE_DMIC:
		node_id.node.dma_type = DMIC_LINK_INPUT_CLASS;
		node_id.node.dma_id = mconfig->dma_id +
					 (mconfig->time_slot);
		break;
	case DEVICE_SLIMBUS:
		node_id.node.dma_type = (SOURCE == mconfig->hw_conn_type) ?
			SLIMBUS_LINK_OUTPUT_CLASS : SLIMBUS_LINK_INPUT_CLASS;
		node_id.node.dma_id = mconfig->dma_id;
		break;
	case DEVICE_HDALINK:
		node_id.node.dma_type = (SOURCE == mconfig->hw_conn_type) ?
			HDA_LINK_OUTPUT_CLASS : HDA_LINK_INPUT_CLASS;
		node_id.node.dma_id = mconfig->dma_id;
		break;
	default:
		node_id.node.dma_type = (SOURCE == mconfig->hw_conn_type) ?
			HDA_HOST_OUTPUT_CLASS : HDA_HOST_INPUT_CLASS;
		node_id.node.dma_id = mconfig->dma_id;
		break;
	}

	cpr_mconfig->gtw_cfg.node_id = node_id.val;

	if (SOURCE == mconfig->hw_conn_type)
		cpr_mconfig->gtw_cfg.dma_buffer_size = 2 * mconfig->obs;
	else
		cpr_mconfig->gtw_cfg.dma_buffer_size = 2 * mconfig->ibs;
	cpr_mconfig->cpr_feature_mask = 0;
	cpr_mconfig->gtw_cfg.config_length  = 0;

	*config_size += sizeof(cpr_mconfig->gtw_cfg) +
			(cpr_mconfig->cpr_feature_mask) + sizeof(u32);
	hda_sst_copy_copier_caps(mconfig, cpr_mconfig, config_size);
	dev_dbg(ctx->dev, "Copier module config:\n");
	dev_dbg(ctx->dev, "length: 0x%x\n", cpr_mconfig->gtw_cfg.config_length);
	return;
}

static void hda_sst_setup_cpr_out_format(struct sst_dsp_ctx *ctx,
	struct module_config *mconfig,
	struct sst_copier_module_config *cpr_mconfig, u16 *config_size)
{
	struct module_format *format = &mconfig->out_fmt;

	cpr_mconfig->out_fmt.number_of_channels =
		(u8)format->channels;

	cpr_mconfig->out_fmt.sampling_frequency = format->sampling_freq;

	cpr_mconfig->out_fmt.bit_depth = format->bit_depth;

	cpr_mconfig->out_fmt.valid_bit_depth = format->valid_bit_depth;

	cpr_mconfig->out_fmt.channel_config = format->channel_config;

	cpr_mconfig->out_fmt.channel_map =
		hda_sst_create_channel_map(cpr_mconfig->out_fmt.channel_config);
	cpr_mconfig->out_fmt.interleaving_style = INTERLEAVING_PER_CHANNEL;

	*config_size += sizeof(cpr_mconfig->out_fmt);

	dev_dbg(ctx->dev, "copier out format chan=%d fre=%d bitdepth=%d\n",
		cpr_mconfig->out_fmt.number_of_channels,
		 format->sampling_freq, format->bit_depth);
	return;
}

static int hda_sst_set_src_format(struct sst_dsp_ctx *ctx,
	struct module_config *mconfig,
	struct sst_src_module_config *src_mconfig,
	u16 *config_size)
{
	int i = 0;
	u32 *byte;
	struct module_format *fmt = &mconfig->out_fmt;

	hda_sst_set_base_module_format(ctx,
		mconfig,
		(struct sst_base_module_config *)src_mconfig,
		config_size);

	src_mconfig->src_config.s_freq = fmt->sampling_freq;

	*config_size += sizeof(struct sst_src_config);

	dev_dbg(ctx->dev, "SRC module config size: %d bytes\n", *config_size);
	for (i = 0, byte = (u32 *) src_mconfig;
		i < (*config_size)/sizeof(u32);
		i++, byte++) {
		dev_dbg(ctx->dev, "0x%x: %08x\n", i, *byte);
	}
	return 0;
}

static int hda_sst_set_updown_mixer_format(struct sst_dsp_ctx *ctx,
	struct module_config *mconfig,
	struct sst_up_down_mixer_module_config *mixer_mconfig,
	u16 *config_size)
{
	int i = 0;
	u32 *byte;
	struct module_format *fmt = &mconfig->out_fmt;

	hda_sst_set_base_module_format(ctx,
		mconfig,
		(struct sst_base_module_config *)mixer_mconfig,
		config_size);
	mixer_mconfig->out_channel_config = fmt->channel_config;

	*config_size += sizeof(enum channel_config);

	/* Select F/W defatult coefficient */
	mixer_mconfig->coeff_sel = 0x0;
	*config_size += sizeof(mixer_mconfig->coeff_sel);

	/* User coeff, dont care since we are selecting F/W defaults */
	for (i = 0; i < UP_DOWN_MIXER_MAX_COEFF; i++)
		mixer_mconfig->coeff[i] = 0xDEADBEEF;

	*config_size += sizeof(mixer_mconfig->coeff);

	dev_dbg(ctx->dev, "Up Down mixer module config size: %d bytes\n",
								*config_size);
	for (i = 0, byte = (u32 *) mixer_mconfig;
		i < (*config_size)/sizeof(u32);
		i++, byte++) {
		dev_dbg(ctx->dev, "0x%x: %08x\n", i, *byte);
	}
	return 0;
}

/*
 * copier_module_config is sent as input buffer with INIT_INSTANCE IPC msg
 */
static int hda_sst_set_copier_format(struct sst_dsp_ctx *ctx,
	struct module_config *mconfig,
	struct sst_copier_module_config *cpr_mconfig,
	u16 *config_size)
{
	int ret = 0;
	int i = 0;
	u32 *byte;

	hda_sst_set_base_module_format(ctx,
		mconfig,
		(struct sst_base_module_config *)cpr_mconfig,
		config_size);

	hda_sst_setup_cpr_out_format(ctx, mconfig, cpr_mconfig, config_size);
	hda_sst_setup_cpr_gateway_cfg(ctx, mconfig, cpr_mconfig, config_size);

	dev_dbg(ctx->dev, "Copier module config size: %d bytes\n", *config_size);
	for (i = 0, byte = (u32 *) cpr_mconfig;
		i < (*config_size)/sizeof(u32);
		i++, byte++) {
		dev_dbg(ctx->dev, "0x%x: %08x\n", i, *byte);
	}

	return ret;
}

static int hda_sst_set_module_format(struct sst_dsp_ctx *ctx,
	struct module_config *module_config,
	u16 *module_config_size,
	void **param_data)
{
	int ret = 0;

	/* Buffer size varies for different modules. For safety, allocate
	 * buffer of size WINDOW1_SIZE */
	*param_data = kzalloc(WINDOW1_SIZE, GFP_KERNEL);

	if (NULL == *param_data)
		return -ENOMEM;

	memset(*param_data, 0, WINDOW1_SIZE);

	if (module_config->id.module_id == COPIER_MODULE)
		ret = hda_sst_set_copier_format(ctx, module_config,
			*param_data, module_config_size);
	else if (module_config->id.module_id == SRCINT_MODULE)
		ret = hda_sst_set_src_format(ctx, module_config,
			*param_data, module_config_size);
	else if (module_config->id.module_id == UPDWMIX_MODULE)
		ret = hda_sst_set_updown_mixer_format(ctx, module_config,
			*param_data, module_config_size);
	else
		hda_sst_set_base_module_format(ctx,
			module_config, *param_data, module_config_size);

	return ret;
}

/*
 * Allocates queue in the link pipeline (mix in/out module)
 */
static u8 hda_sst_alloc_queue(u8 *queue_mask, u8 max_queue)
{
	u8 i = 0;

	while (*queue_mask & (1llu << i)) {
		i++;
		if (i == max_queue) {
			i = 0;
			break;
		}
	}

	*queue_mask |= (1llu << i);

	return i;
}

static void hda_sst_free_queue(u8 *queue_mask, u8 queue_index)
{
	*queue_mask &= 0;
}

int hda_sst_init_module(struct sst_dsp_ctx *ctx, struct module_config *mconfig,
	struct hda_sst_algo_data *ac)
{
	u16 module_config_size = 0;
	void *param_data = NULL;
	int ret = 0;
	struct init_instance_msg msg;

	dev_dbg(ctx->dev, "%s: module_id = %d instance=%d\n", __func__,
		 mconfig->id.module_id, mconfig->id.instance_id);

	ret = hda_sst_set_module_format(ctx, mconfig,
			&module_config_size, &param_data);
	if (ret < 0) {
		dev_err(ctx->dev, "Failed to set module format ret=%d\n", ret);
		goto exit;
	}

	msg.module_id = mconfig->id.module_id;
	msg.instance_id = mconfig->id.instance_id;
	msg.ppl_instance_id = mconfig->pipe->ppl_id;
	msg.param_data_size = module_config_size;
	msg.core_id = mconfig->core_id;

	if (mconfig->pipe->state != CREATED) {
		dev_err(ctx->dev, "Pipe not created for Module state= %d pipe_id= %d\n",
				 mconfig->pipe->state, mconfig->pipe->ppl_id);
		return -EIO;
	}

	ret = ipc_init_instance(ctx->ipc, &msg, param_data);
	if (ret < 0) {
		dev_err(ctx->dev, "Failed to init instance ret=%d\n", ret);
		goto exit;
	}
	mconfig->m_state = INIT_DONE;
	/*FIXME need to check on msg extension for passing algo data in init */

exit:
	if (param_data != NULL)
		kfree(param_data);

	return ret;
}

int hda_sst_bind_unbind_modules(struct sst_dsp_ctx *ctx, struct module_config
	*src_module, struct module_config *dst_module, bool bind)
{
	int ret = 0;
	struct bind_unbind_msg msg;


	dev_dbg(ctx->dev, "%s: src module_id = %d  src_instance=%d dst_module=%d \
		dst_instacne=%d\n", __func__, src_module->id.module_id,
		src_module->id.instance_id, dst_module->id.module_id,
		dst_module->id.instance_id);

	dev_dbg(ctx->dev, "src_module state = %d dst module state = %d\n",
		src_module->m_state, dst_module->m_state);
	/*if module is not bind, then don't send unbind */
	if (!bind) {
		if (src_module->m_state != BIND_DONE)
			return ret;
		/* if intra module unbind, check if both modules are BIND,
		then send unbind */
		if ((src_module->pipe->ppl_id != dst_module->pipe->ppl_id) &&
			dst_module->m_state != BIND_DONE)
			return ret;
	} else if (src_module->m_state != INIT_DONE &&
			 dst_module->m_state != INIT_DONE)
		return ret;

	msg.module_id = src_module->id.module_id;
	msg.instance_id = src_module->id.instance_id;
	msg.dst_module_id = dst_module->id.module_id;
	msg.dst_instance_id = dst_module->id.instance_id;
	if (bind) {
		src_module->out_queue = hda_sst_alloc_queue(&src_module->out_queue_mask,
				 src_module->max_out_queue);
		dst_module->in_queue = hda_sst_alloc_queue(&dst_module->in_queue_mask,
				 dst_module->max_in_queue);
	} else {

		src_module->out_queue_mask = 0;
		dst_module->in_queue_mask = 0;
	}
	dev_dbg(ctx->dev, "in quque = %d out_queue =%d\n",
		 src_module->out_queue, dst_module->in_queue);
	msg.src_queue = src_module->out_queue;
	msg.dst_queue = dst_module->in_queue;
	msg.bind = bind;

	ret = ipc_bind_unbind(ctx->ipc, &msg);

	if (!ret) {
		if (bind)
			src_module->m_state = BIND_DONE;
		else
			src_module->m_state = UNINIT;
	}
	return ret;
}

static int hda_sst_set_pipe_state(struct sst_dsp_ctx *ctx, struct sst_pipe *pipe,
	enum pipeline_state state)
{
	int ret = 0;

	dev_dbg(ctx->dev, "%s: pipe_satate = %d\n", __func__, state);
	ret = ipc_set_pipeline_state(ctx->ipc, pipe->ppl_id, state);
	return ret;
}

/*
 * Creates pipeline, by sending IPC messages to FW
 */
int hda_sst_create_pipeline(struct sst_dsp_ctx *ctx, struct sst_pipe *pipe)
{
	int ret = 0;

	dev_dbg(ctx->dev, "%s: pipe_id = %d\n", __func__, pipe->ppl_id);

	ret = ipc_create_pipeline(ctx->ipc, pipe->memory_pages,
				pipe->pipe_type, pipe->ppl_id);
	if (ret < 0) {
		dev_err(ctx->dev, "Failed to create pipeline\n");
		return ret;
	}
	pipe->state = CREATED;
	return ret;
}

/*
 * Sets pipe state to RUNNING
 */
int hda_sst_run_pipe(struct sst_dsp_ctx *ctx, struct sst_pipe *pipe)
{
	int ret = 0;

	dev_dbg(ctx->dev, "%s: pipe = %d\n", __func__, pipe->ppl_id);

	/* If pipe was not created in FW, do not try to pause or delete */
	if (pipe->state < CREATED)
		return ret;

	/* Pipe has to be paused before it is started */
	ret = hda_sst_set_pipe_state(ctx, pipe, PPL_PAUSED);
	if (ret < 0) {
		dev_err(ctx->dev, "Failed to pause pipe\n");
		return ret;
	}
	ret = hda_sst_set_pipe_state(ctx, pipe, PPL_RUNNING);
	if (ret < 0) {
		dev_err(ctx->dev, "Failed to start pipe\n");
		return ret;
	}

	pipe->state = STARTED;

	return ret;
}

/*
 * Sets pipe state to PAUSED in FW, stops DMA engines and releases resources
 */
int hda_sst_delete_pipe(struct sst_dsp_ctx *ctx, struct sst_pipe *pipe)
{
	int ret = 0;

	dev_dbg(ctx->dev, "%s: pipe = %d\n", __func__, pipe->ppl_id);

	/* If pipe is not started, do not try to stop the pipe in FW. */
	if (pipe->state < STARTED)
		goto delete_pipe;

	ret = hda_sst_set_pipe_state(ctx, pipe, PPL_PAUSED);
	if (ret < 0) {
		dev_err(ctx->dev, "Failed to stop pipeline\n");
		return ret;
	}

delete_pipe:
	/* If pipe was not created in FW, do not try to delete it */
	if (pipe->state < CREATED)
		return ret;

	ret = ipc_delete_pipeline(ctx->ipc, pipe->ppl_id);
	if (ret < 0) {
		dev_err(ctx->dev, "Failed to delete pipeline\n");
		return ret;
	}
	return ret;
}

int hda_sst_stop_pipe(struct sst_dsp_ctx *ctx, struct sst_pipe *pipe)
{
	int ret = 0;

	dev_dbg(ctx->dev, "In %s pipe=%d\n", __func__, pipe->ppl_id);

	/* If pipe was not created in FW, do not try to pause or delete */
	if (pipe->state < CREATED)
		return ret;
	ret = hda_sst_set_pipe_state(ctx, pipe, PPL_PAUSED);
	if (ret < 0) {
		dev_dbg(ctx->dev, "Failed to stop pipe\n");
		return ret;
	}
	pipe->state = CREATED;

	return ret;
}
