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
#include <sound/hda_bus.h>
#include <sound/hda_controller.h>
#include <sound/soc-hda-sst-dsp.h>
#include <sound/soc-hda-bus.h>

struct snd_soc_azx {
	struct azx hda_azx;
	struct snd_soc_hda_bus *sbus;
	struct sst_dsp_ctx *dsp;
	struct platform_device *i2s_pdev;
};

unsigned int azx_get_position(struct azx *chip, struct azx_dev *azx_dev,
				bool with_check);
int soc_hda_platform_unregister(struct device *dev);
int soc_hda_platform_register(struct device *dev);

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

#endif /* __SOUND_HDA_CONTROLLER_H */
