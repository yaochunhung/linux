/*
 *  soc-hda-dma.h - Generic dma header
 *
 *  Copyright (c) 2014 Intel Corporation
 *  Author: Jeeja KP <jeeja.kp@intel.com>
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
 *
 */
#ifndef _SOC_HDA_DMA_H_
#define _SOC_HDA_DMA_H_

struct snd_soc_hda_dma_params {
	u32 format;
	u8 stream_tag;
};
#endif /* _SOC__HDA_DMA_H__*/
