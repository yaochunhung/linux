/*
 *  Common functionality for for HD Audio.
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
 */

#ifndef __SOUND_SOC_HDA_CONTROLLER_H
#define __SOUND_SOC_HDA_CONTROLLER_H

#include <sound/core.h>
#include <sound/initval.h>
#include <sound/soc.h>
#include <sound/hda_bus.h>
#include <sound/hda_controller.h>
#include <sound/soc-hda-sst-dsp.h>
#include <sound/soc-hda-bus.h>
#include "soc-hda-controls.h"

#define HDA_MONO 1
#define HDA_STEREO 2

#define HDA_SSP_MODE_I2S	0
#define HDA_SSP_MODE_DSP_A	1
#define HDA_SSP_MODE_DSP_B	2
#define HDA_SSP_MAX_SLOTS	8
struct hda_platform_info;

struct snd_soc_azx {
	struct azx hda_azx;
	struct snd_soc_hda_bus *sbus;
	struct hda_platform_info *pinfo;
	struct sst_dsp_ctx *dsp;
	struct platform_device *i2s_pdev;
};

struct fw_resource {
	u32 max_mcps;
	u32 max_mem;
	u32 mcps;
	u32 mem;
};

struct hda_platform_info {
	struct azx *chip;
	struct device *dev;
	u32 *widget;
	struct fw_resource resource;
	struct list_head ppl_list;
	struct list_head ppl_start_list;
};

unsigned int azx_get_position(struct azx *chip, struct azx_dev *azx_dev,
				bool with_check);
int soc_hda_platform_unregister(struct device *dev);
int soc_hda_platform_register(struct device *dev);
unsigned int soc_hda_soc_read(struct snd_soc_platform *platform,
		 unsigned int reg);
int soc_hda_soc_write(struct snd_soc_platform *platform,
	unsigned int reg, unsigned int val);
void hda_sst_set_copier_dma_id(struct snd_soc_dai *dai, int dma_id, int stream,
		bool is_fe);
int hda_sst_set_fe_pipeline_state(struct snd_soc_dai *dai,
			 bool start, int stream);
void hda_sst_set_copier_hw_params(struct snd_soc_dai *dai,
	struct snd_pcm_hw_params *params, int stream, bool is_fe);
void hda_sst_set_be_dmic_config(struct snd_soc_dai *dai,
	struct snd_pcm_hw_params *params, int stream);
void hda_sst_set_be_ssp_config(struct snd_soc_dai *dai, int stream);
int hda_sst_dsp_control_init(struct snd_soc_platform *platform,
	struct azx *chip);
int dsp_register(struct azx *chip);
void azx_dsp_unregister(struct azx *chip);
int azx_load_i2s_machine(struct azx *chip);
void azx_i2s_machine_device_unregister(struct azx *chip);
int azx_load_dsp_prepare(struct device *dev, unsigned int format,
				unsigned int byte_size,
				struct snd_dma_buffer *bufp);
void azx_load_dsp_trigger(struct device *dev, bool start);
void azx_load_dsp_cleanup(struct device *dev,
				 struct snd_dma_buffer *dmab);

int azx_alloc_dma_buf(struct device *dev,
		struct snd_dma_buffer *dmab, u32 size);
int azx_free_dma_buf(struct device *dev,
		struct snd_dma_buffer *dmab);

int azx_load_dsp_init(struct device *dev);
bool azx_is_dsp_running(struct azx *chip);
int azx_calculate_ssp_regs(struct sst_dsp_ctx *ctx,
			struct azx_dai_config *cfg, void *ssp_regs);

#endif /* __SOUND_HDA_CONTROLLER_H */
