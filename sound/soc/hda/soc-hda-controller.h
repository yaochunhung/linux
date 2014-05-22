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
#include <sound/soc-hda-bus.h>

struct snd_soc_azx {
	struct azx hda_azx;
	struct snd_soc_hda_bus *sbus;
};

unsigned int azx_get_position(struct azx *chip, struct azx_dev *azx_dev,
				bool with_check);
int soc_hda_platform_unregister(struct device *dev);
int soc_hda_platform_register(struct device *dev);
#endif /* __SOUND_HDA_CONTROLLER_H */
