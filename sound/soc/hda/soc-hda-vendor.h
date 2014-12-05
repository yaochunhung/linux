/*
 *  soc-hda-vendor.h - Intel dsp fw private data
 *
 *  Copyright (C) 2014 Intel Corp
 *  Author: Jeeja KP <jeeja.kp@intel.com>
 *	    Nilofer, Samreen <samreen.nilofer@intel.com>
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
 */

#ifndef __SOC_HDA_VENDOR_H__
#define __SOC_HDA_VENDOR_H__

/* mixer register ids here */
#define HDA_SST_MIX(x)		(x)

#define HDA_SST_MIX_MEDIA0		HDA_SST_MIX(0)
#define HDA_SST_MIX_MEDIA1		HDA_SST_MIX(1)
#define HDA_SST_MIX_MEDIA2		HDA_SST_MIX(2)
#define HDA_SST_MIX_MEDIA3		HDA_SST_MIX(3)
#define HDA_SST_MIX_MFX_LOOP0		HDA_SST_MIX(4)
#define HDA_SST_MIX_SPEECH		HDA_SST_MIX(5)
#define HDA_SST_MIX_TONE		HDA_SST_MIX(6)
#define HDA_SST_MIX_SIDETONE		HDA_SST_MIX(7)
#define HDA_SST_MIX_MODEM0		HDA_SST_MIX(8)
#define HDA_SST_MIX_MODEM1		HDA_SST_MIX(9)
#define HDA_SST_MIX_FM			HDA_SST_MIX(10)
#define HDA_SST_MIX_BT			HDA_SST_MIX(11)
#define HDA_SST_MIX_DMIC01_HIFI		HDA_SST_MIX(12)
#define HDA_SST_MIX_DMIC23_HIFI		HDA_SST_MIX(13)
#define HDA_SST_MIX_DMIC01_VOICE	HDA_SST_MIX(14)
#define HDA_SST_MIX_DMIC23_VOICE	HDA_SST_MIX(15)
#define HDA_SST_MIX_CODEC0		HDA_SST_MIX(16)
#define HDA_SST_MIX_CODEC1		HDA_SST_MIX(17)
#define HDA_SST_MIX_EFX_LOOP0		HDA_SST_MIX(18)
#define HDA_SST_MIX_EFX_LOOP1		HDA_SST_MIX(19)
#define HDA_SST_MIX_EFX_LOOP2		HDA_SST_MIX(20)
#define HDA_SST_MIX_EFX_LOOP3		HDA_SST_MIX(21)
#define HDA_SST_MIX_IDISP		HDA_SST_MIX(22)
#define HDA_SST_MIX_SNS2		HDA_SST_MIX(23)

#define HDA_SST_REG_LAST            (HDA_SST_MIX_SNS2)

/* last entry defines array size */
#define HDA_SST_NUM_WIDGETS         (HDA_SST_REG_LAST + 1)

/* in each mixer register we will define one bit for each input */
#define HDA_SST_MIX_IP(x)		(x)

#define HDA_SST_IP_MEDIA0		HDA_SST_MIX_IP(0)
#define HDA_SST_IP_MEDIA1		HDA_SST_MIX_IP(1)
#define HDA_SST_IP_MEDIA2		HDA_SST_MIX_IP(2)
#define HDA_SST_IP_MEDIA3		HDA_SST_MIX_IP(3)
#define HDA_SST_IP_MFX_LOOP0		HDA_SST_MIX_IP(4)
#define HDA_SST_IP_SPEECH		HDA_SST_MIX_IP(5)
#define HDA_SST_IP_TONE			HDA_SST_MIX_IP(6)
#define HDA_SST_IP_SIDETONE		HDA_SST_MIX_IP(7)
#define HDA_SST_IP_MODEM0		HDA_SST_MIX_IP(8)
#define HDA_SST_IP_MODEM1		HDA_SST_MIX_IP(9)
#define HDA_SST_IP_FM			HDA_SST_MIX_IP(10)
#define HDA_SST_IP_BT			HDA_SST_MIX_IP(11)
#define HDA_SST_IP_DMIC01_HIFI		HDA_SST_MIX_IP(12)
#define HDA_SST_IP_DMIC23_HIFI		HDA_SST_MIX_IP(13)
#define HDA_SST_IP_DMIC01_VOICE		HDA_SST_MIX_IP(14)
#define HDA_SST_IP_DMIC23_VOICE		HDA_SST_MIX_IP(15)
#define HDA_SST_IP_CODEC0		HDA_SST_MIX_IP(16)
#define HDA_SST_IP_CODEC1		HDA_SST_MIX_IP(17)
#define HDA_SST_IP_EFX_LOOP0		HDA_SST_MIX_IP(18)
#define HDA_SST_IP_EFX_LOOP1		HDA_SST_MIX_IP(19)
#define HDA_SST_IP_EFX_LOOP2		HDA_SST_MIX_IP(20)
#define HDA_SST_IP_EFX_LOOP3		HDA_SST_MIX_IP(21)


/* Default types range from 0~12. type can range from 0 to 0xff
 * SST types start at higher to avoid any overlapping in future */
/*fixme move all SST DSP to a common file */
#define SOC_CONTROL_TYPE_HDA_SST_ALGO_PARAMS		1000
#define SOC_CONTROL_TYPE_HDA_SST_MUX		1001
#define SOC_CONTROL_TYPE_HDA_SST_MIX		1002
#define SOC_CONTROL_TYPE_HDA_SST_BYTE		1003

/* Define kcontrol index */
#define SOC_CONTROL_IO_HDA_SST_ALGO_PARAMS\
	SOC_CONTROL_ID(SOC_CONTROL_TYPE_HDA_SST_ALGO_PARAMS, \
		SOC_CONTROL_TYPE_HDA_SST_ALGO_PARAMS, \
		SOC_CONTROL_TYPE_BYTES_EXT)

#define SOC_CONTROL_IO_HDA_SST_MIX\
	SOC_CONTROL_ID(SOC_CONTROL_TYPE_HDA_SST_MIX, \
		SOC_CONTROL_TYPE_HDA_SST_MIX, \
		SOC_CONTROL_TYPE_VOLSW)

#define SOC_CONTROL_IO_HDA_SST_MUX\
	SOC_CONTROL_ID(SOC_CONTROL_TYPE_HDA_SST_MUX, \
		SOC_CONTROL_TYPE_HDA_SST_MUX, \
		SOC_CONTROL_TYPE_HDA_SST_MUX)

/* Define kcontrol index */
#define SOC_CONTROL_IO_HDA_SST_BYTES_EXT\
	SOC_CONTROL_ID(SOC_CONTROL_TYPE_HDA_SST_BYTE, \
		SOC_CONTROL_TYPE_HDA_SST_BYTE, \
		SOC_CONTROL_TYPE_BYTES_EXT)

#define HDA_SST_CFG_MAX	900 /* size of copier cfg*/

/* Event types goes here */
/* Reserve event type 0 for no event handlers */
enum hda_sst_event_types {
	HDA_SST_EVENT_TYPE_NONE = 0,
	HDA_SST_SET_MIXER,
	HDA_SST_SET_MUX,
	HDA_SST_SET_VMIXER,
	HDA_SST_SET_PGA
};

enum enhanced_pipe_type {
	/* render pipes */
	REFERENCE_STREAM = 0, /* loopback pipe */
	RAW_RENDER_STREAM,
	SYSTEM_STREAM,
	OFFLOAD_STREAM,
	RENDER_LINK_STREAM,
	/* capture pipes */
	SPEECH_STREAM,
	VOICE_DEFAULT_STREAM,
	VOICE_RAW_STREAM,
	CAMERA_RAW_STREAM,
	CAPTURE_LINK_STREAM,
	MAX_PIPES
};

enum pipe_type {
	PIPE_TYPE_RAW = 0,
	PIPE_TYPE_DEFAULT = 1,
	PIPE_TYPE_OFFLOAD = 2,
	PIPE_TYPE_LOOPBACK = 3,
	PIPE_TYPE_RENDER_LINK = 4,
	PIPE_TYPE_CAPTURE_LINK = 5,
};

enum channel_config {
	CHANNEL_CONFIG_MONO = 0, /**< One channel only. */
	CHANNEL_CONFIG_STEREO = 1, /**< L & R. */
	CHANNEL_CONFIG_2_1 = 2, /**< L, R & LFE; PCM only. */
	CHANNEL_CONFIG_3_0 = 3, /**< L, C & R; MP3 & AAC only. */
	CHANNEL_CONFIG_3_1 = 4, /**< L, C, R & LFE; PCM only. */
	CHANNEL_CONFIG_QUATRO = 5, /**< L, R, Ls & Rs; PCM only. */
	CHANNEL_CONFIG_4_0 = 6, /**< L, C, R & Cs; MP3 & AAC only. */
	CHANNEL_CONFIG_5_0 = 7, /**< L, C, R, Ls & Rs. */
	CHANNEL_CONFIG_5_1 = 8, /**< L, C, R, Ls, Rs & LFE. */
	CHANNEL_CONFIG_DUAL_MONO = 9, /**< One channel replicated in two. */
	CHANNEL_CONFIG_I2S_DUAL_STEREO_0 = 10, /*Stereo(L,R) in 4 slots,
						1st stream:[ L, R, -, - ] */
	CHANNEL_CONFIG_I2S_DUAL_STEREO_1 = 11, /*Stereo(L,R) in 4 slots,
						2nd stream:[ -, -, L, R ] */
	CHANNEL_CONFIG_INVALID
};

enum module_type {
	BASE_FW_MODULE = 0,
	MIX_IN_MODULE,
	MIX_OUT_MODULE,
	COPIER_MODULE,
	PEAK_VOL_MODULE,
	SRC_MODULE,
	PCM_MODULE,
	HISTORY_BUFFER_MODULE,
	STREAM_FX_MODULE,
	ENDPOINT_FX_MODULE,
	MAX_MODULE
};

enum dev_type {
	DEVICE_BT = 0x0,
	DEVICE_DMIC = 0x1,
	DEVICE_I2S = 0x2,
	DEVICE_SLIMBUS = 0x3,
	DEVICE_HDALINK = 0x4,
	DEVICE_NONE
};

enum module_conn_type {
	CONN_TYPE_NONE = 0,
	CONN_TYPE_FE,
	CONN_TYPE_BE
};

enum hw_connection_type {
	NONE = 0,
	SOURCE = 1,
	SINK = 2
};

enum algo_kcontrol_type {
	HDA_SST_ALGO_PARAMS,
	HDA_SST_ALGO_BYPASS
};

struct hda_dfw_module_fmt {
	u32 channels;
	u32 freq;
	u32 bit_depth;
	u32 valid_bit_depth;
	u32 ch_cfg;
} __packed;

struct hda_dfw_module_caps {
	u32 caps_size;
	u32 caps[HDA_SST_CFG_MAX];
};

struct hda_dfw_pipe {
	u8 pipe_id;
	u8 pipe_type;
	u16 conn_type;
	u32 memory_pages;
} __packed;

struct hda_dfw_module {
	u16 module_id;
	u16 instance_id;
	u32 max_mcps;
	u8 core_id;
	u8 max_in_queue;
	u8 max_out_queue;
	u8 is_loadable;
	u8 conn_type;
	u8 dev_type;
	u8 hw_conn_type;
	u8 time_slot;
	u32 obs;
	u32 ibs;
	u32 params_fixup;
	u32 converter;
	struct hda_dfw_pipe pipe;
	struct hda_dfw_module_fmt in_fmt;
	struct hda_dfw_module_fmt out_fmt;
	struct hda_dfw_module_caps caps;
} __packed;

struct hda_dfw_algo_data {
	u32 max;
	char *params;
} __packed;

#endif
