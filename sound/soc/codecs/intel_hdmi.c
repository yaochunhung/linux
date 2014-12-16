/*
 *  intel_hdmi.c - ASoc HDA-HDMI codec driver for Intel platforms
 *
 *  Copyright (C) 2014 Intel Corp
 *  Copyright (c) 2014 Wu Fengguang <wfg@linux.intel.com>
 *
 * Author: Samreen Nilofer <samreen.nilofer@intel.com>
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
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <sound/core.h>
#include <sound/jack.h>
#include <sound/asoundef.h>
#include <sound/tlv.h>
#include <sound/pcm_params.h>
#include <sound/hda_codec.h>
#include <sound/hda_auto_parser.h>
#include <sound/hda_controls.h>
#include <sound/hda_hdmi.h>
#include <sound/hda_jack.h>
#include <sound/soc-hda-bus.h>
#include <sound/soc-hda-dma.h>
#include "soc-hda-codec.h"

static bool static_hdmi_pcm;
module_param(static_hdmi_pcm, bool, 0644);
MODULE_PARM_DESC(static_hdmi_pcm, "Don't restrict PCM parameters per ELD info");

struct hdmi_spec_per_cvt {
	hda_nid_t cvt_nid;
	int assigned;
	unsigned int channels_min;
	unsigned int channels_max;
	u32 rates;
	u64 formats;
	unsigned int maxbps;
};

/* max. connections to a widget */
#define HDA_MAX_CONNECTIONS     32

struct hdmi_spec_per_pin {
	hda_nid_t pin_nid;
	int num_mux_nids;
	hda_nid_t mux_nids[HDA_MAX_CONNECTIONS];
	int mux_idx;
	hda_nid_t cvt_nid;

	struct hda_codec *codec;
	struct hdmi_eld sink_eld;
	struct mutex lock;
	struct delayed_work work;
	struct snd_kcontrol *eld_ctl;
	int repoll_count;
	bool setup; /* the stream has been set up by prepare callback */
	int channels; /* current number of channels */
	bool non_pcm;
	bool chmap_set;         /* channel-map override by ALSA API? */
	unsigned char chmap[8]; /* ALSA API channel-map */
	char pcm_name[8];       /* filled in build_pcm callbacks */

#ifdef CONFIG_PROC_FS
	struct snd_info_entry *proc_entry;
#endif
};

struct cea_channel_speaker_allocation;

/* operations used by generic code that can be overridden by patches */
struct hdmi_ops {
	int (*pin_get_eld)(struct hda_codec *codec, hda_nid_t pin_nid,
				unsigned char *buf, int *eld_size);

	/* get and set channel assigned to each HDMI ASP (audio sample packet) slot */
	int (*pin_get_slot_channel)(struct hda_codec *codec, hda_nid_t pin_nid,
				int asp_slot);
	int (*pin_set_slot_channel)(struct hda_codec *codec, hda_nid_t pin_nid,
				int asp_slot, int channel);

	void (*pin_setup_infoframe)(struct hda_codec *codec, hda_nid_t pin_nid,
				int ca, int active_channels, int conn_type);

	/* enable/disable HBR (HD passthrough) */
	int (*pin_hbr_setup)(struct hda_codec *codec, hda_nid_t pin_nid, bool hbr);

	int (*setup_stream)(struct hda_codec *codec, hda_nid_t cvt_nid,
				hda_nid_t pin_nid, u32 stream_tag, int format);

	/* Helpers for producing the channel map TLVs. These can be overridden
	 * for devices that have non-standard mapping requirements. */
	int (*chmap_cea_alloc_validate_get_type)(struct cea_channel_speaker_allocation *cap,
				int channels);
	void (*cea_alloc_to_tlv_chmap)(struct cea_channel_speaker_allocation *cap,
				unsigned int *chmap, int channels);

	/* check that the user-given chmap is supported */
	int (*chmap_validate)(int ca, int channels, unsigned char *chmap);
};

struct hdmi_dai_pin_map {
	char dai_name[20];
	hda_nid_t nid;
	int pin_idx;
};

struct hdmi_spec {
	int num_cvts;
	struct snd_array cvts; /* struct hdmi_spec_per_cvt */
	hda_nid_t cvt_nids[4];

	int num_pins;
	struct snd_array pins; /* struct hdmi_spec_per_pin */

	struct hdmi_dai_pin_map dai_pin_map[16]; /* struct hda_dai_pin_map */
	unsigned int channels_max; /* max over all cvts */

	struct hdmi_eld temp_eld;
	struct hdmi_ops ops;
	bool dyn_pin_out;

	struct workqueue_struct *workq;
};


struct hdmi_patch {
	int (*patch)(struct hda_codec *codec);
};

struct hdmi_audio_infoframe {
	u8 type; /* 0x84 */
	u8 ver;  /* 0x01 */
	u8 len;  /* 0x0a */

	u8 checksum;

	u8 CC02_CT47;	/* CC in bits 0:2, CT in 4:7 */
	u8 SS01_SF24;
	u8 CXT04;
	u8 CA;
	u8 LFEPBL01_LSV36_DM_INH7;
};

struct dp_audio_infoframe {
	u8 type; /* 0x84 */
	u8 len;  /* 0x1b */
	u8 ver;  /* 0x11 << 2 */

	u8 CC02_CT47;	/* match with HDMI infoframe from this on */
	u8 SS01_SF24;
	u8 CXT04;
	u8 CA;
	u8 LFEPBL01_LSV36_DM_INH7;
};

union audio_infoframe {
	struct hdmi_audio_infoframe hdmi;
	struct dp_audio_infoframe dp;
	u8 bytes[0];
};

/*
 * CEA speaker placement:
 *
 *        FLH       FCH        FRH
 *  FLW    FL  FLC   FC   FRC   FR   FRW
 *
 *                                  LFE
 *                     TC
 *
 *          RL  RLC   RC   RRC   RR
 *
 * The Left/Right Surround channel _notions_ LS/RS in SMPTE 320M corresponds to
 * CEA RL/RR; The SMPTE channel _assignment_ C/LFE is swapped to CEA LFE/FC.
 */
enum cea_speaker_placement {
	FL  = (1 <<  0),	/* Front Left           */
	FC  = (1 <<  1),	/* Front Center         */
	FR  = (1 <<  2),	/* Front Right          */
	FLC = (1 <<  3),	/* Front Left Center    */
	FRC = (1 <<  4),	/* Front Right Center   */
	RL  = (1 <<  5),	/* Rear Left            */
	RC  = (1 <<  6),	/* Rear Center          */
	RR  = (1 <<  7),	/* Rear Right           */
	RLC = (1 <<  8),	/* Rear Left Center     */
	RRC = (1 <<  9),	/* Rear Right Center    */
	LFE = (1 << 10),	/* Low Frequency Effect */
	FLW = (1 << 11),	/* Front Left Wide      */
	FRW = (1 << 12),	/* Front Right Wide     */
	FLH = (1 << 13),	/* Front Left High      */
	FCH = (1 << 14),	/* Front Center High    */
	FRH = (1 << 15),	/* Front Right High     */
	TC  = (1 << 16),	/* Top Center           */
};

/*
 * ELD SA bits in the CEA Speaker Allocation data block
 */
static int eld_speaker_allocation_bits[] = {
	[0] = FL | FR,
	[1] = LFE,
	[2] = FC,
	[3] = RL | RR,
	[4] = RC,
	[5] = FLC | FRC,
	[6] = RLC | RRC,
	/* the following are not defined in ELD yet */
	[7] = FLW | FRW,
	[8] = FLH | FRH,
	[9] = TC,
	[10] = FCH,
};

struct cea_channel_speaker_allocation {
	int ca_index;
	int speakers[8];

	/* derived values, just for convenience */
	int channels;
	int spk_mask;
};

/*
 * ALSA sequence is:
 *
 *       surround40   surround41   surround50   surround51   surround71
 * ch0   front left   =            =            =            =
 * ch1   front right  =            =            =            =
 * ch2   rear left    =            =            =            =
 * ch3   rear right   =            =            =            =
 * ch4                LFE          center       center       center
 * ch5                                          LFE          LFE
 * ch6                                                       side left
 * ch7                                                       side right
 *
 * surround71 = {FL, FR, RLC, RRC, FC, LFE, RL, RR}
 */
static int hdmi_channel_mapping[0x32][8] = {
	/* stereo */
	[0x00] = { 0x00, 0x11, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7 },
	/* 2.1 */
	[0x01] = { 0x00, 0x11, 0x22, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7 },
	/* Dolby Surround */
	[0x02] = { 0x00, 0x11, 0x23, 0xf2, 0xf4, 0xf5, 0xf6, 0xf7 },
	/* surround40 */
	[0x08] = { 0x00, 0x11, 0x24, 0x35, 0xf3, 0xf2, 0xf6, 0xf7 },
	/* 4ch */
	[0x03] = { 0x00, 0x11, 0x23, 0x32, 0x44, 0xf5, 0xf6, 0xf7 },
	/* surround41 */
	[0x09] = { 0x00, 0x11, 0x24, 0x35, 0x42, 0xf3, 0xf6, 0xf7 },
	/* surround50 */
	[0x0a] = { 0x00, 0x11, 0x24, 0x35, 0x43, 0xf2, 0xf6, 0xf7 },
	/* surround51 */
	[0x0b] = { 0x00, 0x11, 0x24, 0x35, 0x43, 0x52, 0xf6, 0xf7 },
	/* 7.1 */
	[0x13] = { 0x00, 0x11, 0x26, 0x37, 0x43, 0x52, 0x64, 0x75 },
};

/*
 * This is an ordered list!
 *
 * The preceding ones have better chances to be selected by
 * hdmi_channel_allocation().
 */
static struct cea_channel_speaker_allocation channel_allocations[] = {
	/*		  channel:   7     6    5    4    3     2    1    0  */
	{ .ca_index = 0x00,  .speakers = {   0,    0,   0,   0,   0,    0,  FR,  FL } },
					 /* 2.1 */
	{ .ca_index = 0x01,  .speakers = {   0,    0,   0,   0,   0,  LFE,  FR,  FL } },
					 /* Dolby Surround */
	{ .ca_index = 0x02,  .speakers = {   0,    0,   0,   0,  FC,    0,  FR,  FL } },
					 /* surround40 */
	{ .ca_index = 0x08,  .speakers = {   0,    0,  RR,  RL,   0,    0,  FR,  FL } },
					 /* surround41 */
	{ .ca_index = 0x09,  .speakers = {   0,    0,  RR,  RL,   0,  LFE,  FR,  FL } },
					 /* surround50 */
	{ .ca_index = 0x0a,  .speakers = {   0,    0,  RR,  RL,  FC,    0,  FR,  FL } },
					 /* surround51 */
	{ .ca_index = 0x0b,  .speakers = {   0,    0,  RR,  RL,  FC,  LFE,  FR,  FL } },
					 /* 6.1 */
	{ .ca_index = 0x0f,  .speakers = {   0,   RC,  RR,  RL,  FC,  LFE,  FR,  FL } },
					 /* surround71 */
	{ .ca_index = 0x13,  .speakers = { RRC,  RLC,  RR,  RL,  FC,  LFE,  FR,  FL } },

	{ .ca_index = 0x03,  .speakers = {   0,    0,   0,   0,  FC,  LFE,  FR,  FL } },
	{ .ca_index = 0x04,  .speakers = {   0,    0,   0,  RC,   0,    0,  FR,  FL } },
	{ .ca_index = 0x05,  .speakers = {   0,    0,   0,  RC,   0,  LFE,  FR,  FL } },
	{ .ca_index = 0x06,  .speakers = {   0,    0,   0,  RC,  FC,    0,  FR,  FL } },
	{ .ca_index = 0x07,  .speakers = {   0,    0,   0,  RC,  FC,  LFE,  FR,  FL } },
	{ .ca_index = 0x0c,  .speakers = {   0,   RC,  RR,  RL,   0,    0,  FR,  FL } },
	{ .ca_index = 0x0d,  .speakers = {   0,   RC,  RR,  RL,   0,  LFE,  FR,  FL } },
	{ .ca_index = 0x0e,  .speakers = {   0,   RC,  RR,  RL,  FC,    0,  FR,  FL } },
	{ .ca_index = 0x10,  .speakers = { RRC,  RLC,  RR,  RL,   0,    0,  FR,  FL } },
	{ .ca_index = 0x11,  .speakers = { RRC,  RLC,  RR,  RL,   0,  LFE,  FR,  FL } },
	{ .ca_index = 0x12,  .speakers = { RRC,  RLC,  RR,  RL,  FC,    0,  FR,  FL } },
	{ .ca_index = 0x14,  .speakers = { FRC,  FLC,   0,   0,   0,    0,  FR,  FL } },
	{ .ca_index = 0x15,  .speakers = { FRC,  FLC,   0,   0,   0,  LFE,  FR,  FL } },
	{ .ca_index = 0x16,  .speakers = { FRC,  FLC,   0,   0,  FC,    0,  FR,  FL } },
	{ .ca_index = 0x17,  .speakers = { FRC,  FLC,   0,   0,  FC,  LFE,  FR,  FL } },
	{ .ca_index = 0x18,  .speakers = { FRC,  FLC,   0,  RC,   0,    0,  FR,  FL } },
	{ .ca_index = 0x19,  .speakers = { FRC,  FLC,   0,  RC,   0,  LFE,  FR,  FL } },
	{ .ca_index = 0x1a,  .speakers = { FRC,  FLC,   0,  RC,  FC,    0,  FR,  FL } },
	{ .ca_index = 0x1b,  .speakers = { FRC,  FLC,   0,  RC,  FC,  LFE,  FR,  FL } },
	{ .ca_index = 0x1c,  .speakers = { FRC,  FLC,  RR,  RL,   0,    0,  FR,  FL } },
	{ .ca_index = 0x1d,  .speakers = { FRC,  FLC,  RR,  RL,   0,  LFE,  FR,  FL } },
	{ .ca_index = 0x1e,  .speakers = { FRC,  FLC,  RR,  RL,  FC,    0,  FR,  FL } },
	{ .ca_index = 0x1f,  .speakers = { FRC,  FLC,  RR,  RL,  FC,  LFE,  FR,  FL } },
	{ .ca_index = 0x20,  .speakers = {   0,  FCH,  RR,  RL,  FC,    0,  FR,  FL } },
	{ .ca_index = 0x21,  .speakers = {   0,  FCH,  RR,  RL,  FC,  LFE,  FR,  FL } },
	{ .ca_index = 0x22,  .speakers = {  TC,    0,  RR,  RL,  FC,    0,  FR,  FL } },
	{ .ca_index = 0x23,  .speakers = {  TC,    0,  RR,  RL,  FC,  LFE,  FR,  FL } },
	{ .ca_index = 0x24,  .speakers = { FRH,  FLH,  RR,  RL,   0,    0,  FR,  FL } },
	{ .ca_index = 0x25,  .speakers = { FRH,  FLH,  RR,  RL,   0,  LFE,  FR,  FL } },
	{ .ca_index = 0x26,  .speakers = { FRW,  FLW,  RR,  RL,   0,    0,  FR,  FL } },
	{ .ca_index = 0x27,  .speakers = { FRW,  FLW,  RR,  RL,   0,  LFE,  FR,  FL } },
	{ .ca_index = 0x28,  .speakers = {  TC,   RC,  RR,  RL,  FC,    0,  FR,  FL } },
	{ .ca_index = 0x29,  .speakers = {  TC,   RC,  RR,  RL,  FC,  LFE,  FR,  FL } },
	{ .ca_index = 0x2a,  .speakers = { FCH,   RC,  RR,  RL,  FC,    0,  FR,  FL } },
	{ .ca_index = 0x2b,  .speakers = { FCH,   RC,  RR,  RL,  FC,  LFE,  FR,  FL } },
	{ .ca_index = 0x2c,  .speakers = {  TC,  FCH,  RR,  RL,  FC,    0,  FR,  FL } },
	{ .ca_index = 0x2d,  .speakers = {  TC,  FCH,  RR,  RL,  FC,  LFE,  FR,  FL } },
	{ .ca_index = 0x2e,  .speakers = { FRH,  FLH,  RR,  RL,  FC,    0,  FR,  FL } },
	{ .ca_index = 0x2f,  .speakers = { FRH,  FLH,  RR,  RL,  FC,  LFE,  FR,  FL } },
	{ .ca_index = 0x30,  .speakers = { FRW,  FLW,  RR,  RL,  FC,    0,  FR,  FL } },
	{ .ca_index = 0x31,  .speakers = { FRW,  FLW,  RR,  RL,  FC,  LFE,  FR,  FL } },
};

/*
 * HDMI routines
 */
#define get_pin(spec, idx) \
	((struct hdmi_spec_per_pin *)snd_array_elem(&spec->pins, idx))
#define get_cvt(spec, idx) \
	((struct hdmi_spec_per_cvt  *)snd_array_elem(&spec->cvts, idx))
#define get_dai_pin_map(spec, idx) \
	((struct hdmi_dai_pin_map *)(&spec->dai_pin_map[idx]))

static int pin_nid_to_pin_index(struct hda_codec *codec, hda_nid_t pin_nid)
{
	struct hdmi_spec *spec = codec->spec;
	int pin_idx;

	for (pin_idx = 0; pin_idx < spec->num_pins; pin_idx++) {
		if (get_pin(spec, pin_idx)->pin_nid == pin_nid)
			return pin_idx;
	}
	snd_printd("%s: pin nid %d not registered\n", __func__, pin_nid);

	return -EINVAL;
}

static int dai_map_to_pin_index(struct hda_codec *codec,
				const char *dai_name)
{
	struct hdmi_spec *spec = codec->spec;
	int pin_idx;

	for (pin_idx = 0; pin_idx < spec->num_pins; pin_idx++) {
		struct hdmi_dai_pin_map *map = get_dai_pin_map(spec, pin_idx);

		if (!strcmp(map->dai_name, dai_name)) {
			snd_printd("matched idx %d\n", pin_idx);
			return pin_idx;
		}
	}
	snd_printk(KERN_WARNING "dai name %s not registered\n", dai_name);

	return -EINVAL;
}

static int cvt_nid_to_cvt_index(struct hda_codec *codec, hda_nid_t cvt_nid)
{
	struct hdmi_spec *spec = codec->spec;
	int cvt_idx;

	for (cvt_idx = 0; cvt_idx < spec->num_cvts; cvt_idx++)
		if (get_cvt(spec, cvt_idx)->cvt_nid == cvt_nid)
			return cvt_idx;

	snd_printk(KERN_WARNING "HDMI: cvt nid %d not registered\n", cvt_nid);
	return -EINVAL;
}

static int hdmi_eld_ctl_info(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_info *uinfo)
{
	struct hda_codec *codec = snd_kcontrol_chip(kcontrol);
	struct hdmi_spec *spec = codec->spec;
	struct hdmi_spec_per_pin *per_pin;
	struct hdmi_eld *eld;
	int pin_idx;

	uinfo->type = SNDRV_CTL_ELEM_TYPE_BYTES;

	pin_idx = kcontrol->private_value;
	per_pin = get_pin(spec, pin_idx);
	eld = &per_pin->sink_eld;

	mutex_lock(&per_pin->lock);
	uinfo->count = eld->eld_valid ? eld->eld_size : 0;
	mutex_unlock(&per_pin->lock);

	return 0;
}

static int hdmi_eld_ctl_get(struct snd_kcontrol *kcontrol,
			struct snd_ctl_elem_value *ucontrol)
{
	struct hda_codec *codec = snd_kcontrol_chip(kcontrol);
	struct hdmi_spec *spec = codec->spec;
	struct hdmi_spec_per_pin *per_pin;
	struct hdmi_eld *eld;
	int pin_idx;

	pin_idx = kcontrol->private_value;
	per_pin = get_pin(spec, pin_idx);
	eld = &per_pin->sink_eld;

	mutex_lock(&per_pin->lock);
	if (eld->eld_size > ARRAY_SIZE(ucontrol->value.bytes.data)) {
		mutex_unlock(&per_pin->lock);
		snd_BUG();
		return -EINVAL;
	}

	memset(ucontrol->value.bytes.data, 0,
	       ARRAY_SIZE(ucontrol->value.bytes.data));
	if (eld->eld_valid)
		memcpy(ucontrol->value.bytes.data, eld->eld_buffer,
		       eld->eld_size);
	mutex_unlock(&per_pin->lock);

	return 0;
}

static struct snd_kcontrol_new eld_bytes_ctl = {
	.access = SNDRV_CTL_ELEM_ACCESS_READ | SNDRV_CTL_ELEM_ACCESS_VOLATILE,
	.iface = SNDRV_CTL_ELEM_IFACE_PCM,
	.info = hdmi_eld_ctl_info,
	.get = hdmi_eld_ctl_get,
};

static int hdmi_create_eld_ctl(struct hda_codec *codec, int pin_idx)
{
	struct snd_kcontrol *kctl;
	struct hdmi_spec *spec = codec->spec;
	char str[32] = "ELD";
	int err;

	snd_printd("In %s pin_idx %d\n", __func__, pin_idx);

	sprintf(str + strlen(str), "%s:%d", str, pin_idx);
	eld_bytes_ctl.name = str;

	snd_printd("eld name %s\n", eld_bytes_ctl.name);

	kctl = snd_ctl_new1(&eld_bytes_ctl, codec);
	if (!kctl)
		return -ENOMEM;
	kctl->private_value = pin_idx;

	snd_printd("Pin nid %d\n", get_pin(spec, pin_idx)->pin_nid);

	err = snd_hda_ctl_add(codec, get_pin(spec, pin_idx)->pin_nid, kctl);
	if (err < 0)
		return err;

	get_pin(spec, pin_idx)->eld_ctl = kctl;
	return 0;
}

static void hdmi_set_dip_index(struct hda_codec *codec, hda_nid_t pin_nid,
				int packet_index, int byte_index)
{
	int val;

	val = (packet_index << 5) | (byte_index & 0x1f);

	snd_hda_codec_write(codec, pin_nid, 0, AC_VERB_SET_HDMI_DIP_INDEX, val);
}

static void hdmi_write_dip_byte(struct hda_codec *codec, hda_nid_t pin_nid,
				unsigned char val)
{
	snd_hda_codec_write(codec, pin_nid, 0, AC_VERB_SET_HDMI_DIP_DATA, val);
}

static void hdmi_init_pin(struct hda_codec *codec, hda_nid_t pin_nid)
{
	/* Unmute */
	if (get_wcaps(codec, pin_nid) & AC_WCAP_OUT_AMP)
		snd_hda_codec_write(codec, pin_nid, 0,
				AC_VERB_SET_AMP_GAIN_MUTE, AMP_OUT_UNMUTE);
	/* Enable pin out: some machines with GM965 gets broken output when
	 * the pin is disabled or changed while using with HDMI
	 */
	snd_hda_codec_write(codec, pin_nid, 0,
			    AC_VERB_SET_PIN_WIDGET_CONTROL, PIN_OUT);
}

static int hdmi_get_channel_count(struct hda_codec *codec, hda_nid_t cvt_nid)
{
	return 1 + snd_hda_codec_read(codec, cvt_nid, 0,
					AC_VERB_GET_CVT_CHAN_COUNT, 0);
}

static void hdmi_set_channel_count(struct hda_codec *codec,
				   hda_nid_t cvt_nid, int chs)
{
	if (chs != hdmi_get_channel_count(codec, cvt_nid))
		snd_hda_codec_write(codec, cvt_nid, 0,
				    AC_VERB_SET_CVT_CHAN_COUNT, chs - 1);
}

/*
 * ELD proc files
 */

#ifdef CONFIG_PROC_FS
static void print_eld_info(struct snd_info_entry *entry,
				struct snd_info_buffer *buffer)
{
	struct hdmi_spec_per_pin *per_pin = entry->private_data;

	mutex_lock(&per_pin->lock);
	snd_hdmi_print_eld_info(&per_pin->sink_eld, buffer);
	mutex_unlock(&per_pin->lock);
}

static void write_eld_info(struct snd_info_entry *entry,
			   struct snd_info_buffer *buffer)
{
	struct hdmi_spec_per_pin *per_pin = entry->private_data;

	mutex_lock(&per_pin->lock);
	snd_hdmi_write_eld_info(&per_pin->sink_eld, buffer);
	mutex_unlock(&per_pin->lock);
}

static int eld_proc_new(struct hdmi_spec_per_pin *per_pin, int index)
{
	char name[32];
	struct hda_codec *codec = per_pin->codec;
	struct snd_info_entry *entry;
	int err;

	snd_printd("%s with config\n", __func__);
	snprintf(name, sizeof(name), "eld#%d.%d", codec->addr, index);
	err = snd_card_proc_new(codec->card, name, &entry);
	if (err < 0)
		return err;

	snd_info_set_text_ops(entry, per_pin, print_eld_info);
	entry->c.text.write = write_eld_info;
	entry->mode |= S_IWUSR;
	per_pin->proc_entry = entry;

	return 0;
}

static void eld_proc_free(struct hdmi_spec_per_pin *per_pin)
{
	if (per_pin->proc_entry) {
		snd_device_free(per_pin->codec->card, per_pin->proc_entry);
		per_pin->proc_entry = NULL;
	}
}
#else
static inline int eld_proc_new(struct hdmi_spec_per_pin *per_pin,
			       int index)
{
	snd_printd("%s without config\n", __func__);
	return 0;
}
static inline void eld_proc_free(struct hdmi_spec_per_pin *per_pin)
{
}
#endif

/*
 * Channel mapping routines
 */

/*
 * Compute derived values in channel_allocations[].
 */
static void init_channel_allocations(void)
{
	int i, j;
	struct cea_channel_speaker_allocation *p;

	for (i = 0; i < ARRAY_SIZE(channel_allocations); i++) {
		p = channel_allocations + i;
		p->channels = 0;
		p->spk_mask = 0;
		for (j = 0; j < ARRAY_SIZE(p->speakers); j++)
			if (p->speakers[j]) {
				p->channels++;
				p->spk_mask |= p->speakers[j];
			}
	}
}

static int get_channel_allocation_order(int ca)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(channel_allocations); i++) {
		if (channel_allocations[i].ca_index == ca)
			break;
	}
	return i;
}

/*
 * The transformation takes two steps:
 *
 *	eld->spk_alloc => (eld_speaker_allocation_bits[]) => spk_mask
 *	      spk_mask => (channel_allocations[])         => ai->CA
 *
 * TODO: it could select the wrong CA from multiple candidates.
*/
static int hdmi_channel_allocation(struct hdmi_eld *eld, int channels)
{
	int i;
	int ca = 0;
	int spk_mask = 0;
	char buf[SND_PRINT_CHANNEL_ALLOCATION_ADVISED_BUFSIZE];

	/*
	 * CA defaults to 0 for basic stereo audio
	 */
	if (channels <= 2)
		return 0;

	/*
	 * expand ELD's speaker allocation mask
	 *
	 * ELD tells the speaker mask in a compact(paired) form,
	 * expand ELD's notions to match the ones used by Audio InfoFrame.
	 */
	for (i = 0; i < ARRAY_SIZE(eld_speaker_allocation_bits); i++) {
		if (eld->info.spk_alloc & (1 << i))
			spk_mask |= eld_speaker_allocation_bits[i];
	}

	/* search for the first working match in the CA table */
	for (i = 0; i < ARRAY_SIZE(channel_allocations); i++) {
		if (channels == channel_allocations[i].channels &&
		    (spk_mask & channel_allocations[i].spk_mask) ==
				channel_allocations[i].spk_mask) {
			ca = channel_allocations[i].ca_index;
			break;
		}
	}

	if (!ca) {
		/* if there was no match, select the regular ALSA channel
		 * allocation with the matching number of channels */
		for (i = 0; i < ARRAY_SIZE(channel_allocations); i++) {
			if (channels == channel_allocations[i].channels) {
				ca = channel_allocations[i].ca_index;
				break;
			}
		}
	}

	snd_print_channel_allocation(eld->info.spk_alloc, buf, sizeof(buf));
	snd_printd("HDMI: select CA 0x%x for %d-channel allocation: %s\n",
		    ca, channels, buf);

	return ca;
}

static void hdmi_debug_channel_mapping(struct hda_codec *codec,
				       hda_nid_t pin_nid)
{
#ifdef CONFIG_SND_DEBUG_VERBOSE
	struct hdmi_spec *spec = codec->spec;
	int i;
	int channel;

	for (i = 0; i < 8; i++) {
		channel = spec->ops.pin_get_slot_channel(codec, pin_nid, i);
		snd_printd("HDMI: ASP channel %d => slot %d\n",
							channel, i);
	}
#endif
}

static void hdmi_std_setup_channel_mapping(struct hda_codec *codec,
				       hda_nid_t pin_nid,
				       bool non_pcm,
				       int ca)
{
	struct hdmi_spec *spec;
	struct cea_channel_speaker_allocation *ch_alloc;
	int i;
	int err;
	int order;
	int non_pcm_mapping[8];

	spec  = codec->spec;
	order = get_channel_allocation_order(ca);
	ch_alloc = &channel_allocations[order];

	if (hdmi_channel_mapping[ca][1] == 0) {
		int hdmi_slot = 0;
		/* fill actual channel mappings in ALSA channel (i) order */
		for (i = 0; i < ch_alloc->channels; i++) {
			while (!ch_alloc->speakers[7 - hdmi_slot] && !WARN_ON(hdmi_slot >= 8))
				hdmi_slot++; /* skip zero slots */

			hdmi_channel_mapping[ca][i] = (i << 4) | hdmi_slot++;
		}
		/* fill the rest of the slots with ALSA channel 0xf */
		for (hdmi_slot = 0; hdmi_slot < 8; hdmi_slot++)
			if (!ch_alloc->speakers[7 - hdmi_slot])
				hdmi_channel_mapping[ca][i++] = (0xf << 4) | hdmi_slot;
	}

	if (non_pcm) {
		for (i = 0; i < ch_alloc->channels; i++)
			non_pcm_mapping[i] = (i << 4) | i;
		for (; i < 8; i++)
			non_pcm_mapping[i] = (0xf << 4) | i;
	}

	for (i = 0; i < 8; i++) {
		int slotsetup = non_pcm ? non_pcm_mapping[i] : hdmi_channel_mapping[ca][i];
		int hdmi_slot = slotsetup & 0x0f;
		int channel = (slotsetup & 0xf0) >> 4;
		err = spec->ops.pin_set_slot_channel(codec, pin_nid, hdmi_slot, channel);
		if (err) {
			snd_printk(KERN_ERR "HDMI: channel mapping failed\n");
			break;
		}
	}
}

struct channel_map_table {
	unsigned char map;		/* ALSA API channel map position */
	int spk_mask;			/* speaker position bit mask */
};

static struct channel_map_table map_tables[] = {
	{ SNDRV_CHMAP_FL,	FL },
	{ SNDRV_CHMAP_FR,	FR },
	{ SNDRV_CHMAP_RL,	RL },
	{ SNDRV_CHMAP_RR,	RR },
	{ SNDRV_CHMAP_LFE,	LFE },
	{ SNDRV_CHMAP_FC,	FC },
	{ SNDRV_CHMAP_RLC,	RLC },
	{ SNDRV_CHMAP_RRC,	RRC },
	{ SNDRV_CHMAP_RC,	RC },
	{ SNDRV_CHMAP_FLC,	FLC },
	{ SNDRV_CHMAP_FRC,	FRC },
	{ SNDRV_CHMAP_TFL,	FLH },
	{ SNDRV_CHMAP_TFR,	FRH },
	{ SNDRV_CHMAP_FLW,	FLW },
	{ SNDRV_CHMAP_FRW,	FRW },
	{ SNDRV_CHMAP_TC,	TC },
	{ SNDRV_CHMAP_TFC,	FCH },
	{} /* terminator */
};
/* from ALSA API channel position to speaker bit mask */
static int to_spk_mask(unsigned char c)
{
	struct channel_map_table *t = map_tables;
	for (; t->map; t++) {
		if (t->map == c)
			return t->spk_mask;
	}
	return 0;
}

/* from ALSA API channel position to CEA slot */
static int to_cea_slot(int ordered_ca, unsigned char pos)
{
	int mask = to_spk_mask(pos);
	int i;

	if (mask) {
		for (i = 0; i < 8; i++) {
			if (channel_allocations[ordered_ca].speakers[7 - i] == mask)
				return i;
		}
	}

	return -1;
}

/* from speaker bit mask to ALSA API channel position */
static int spk_to_chmap(int spk)
{
	struct channel_map_table *t = map_tables;
	for (; t->map; t++) {
		if (t->spk_mask == spk)
			return t->map;
	}
	return 0;
}

/* from CEA slot to ALSA API channel position */
static int from_cea_slot(int ordered_ca, unsigned char slot)
{
	int mask = channel_allocations[ordered_ca].speakers[7 - slot];

	return spk_to_chmap(mask);
}

/* get the CA index corresponding to the given ALSA API channel map */
static int hdmi_manual_channel_allocation(int chs, unsigned char *map)
{
	int i, spks = 0, spk_mask = 0;

	for (i = 0; i < chs; i++) {
		int mask = to_spk_mask(map[i]);
		if (mask) {
			spk_mask |= mask;
			spks++;
		}
	}

	for (i = 0; i < ARRAY_SIZE(channel_allocations); i++) {
		if ((chs == channel_allocations[i].channels ||
		     spks == channel_allocations[i].channels) &&
		    (spk_mask & channel_allocations[i].spk_mask) ==
				channel_allocations[i].spk_mask)
			return channel_allocations[i].ca_index;
	}
	return -1;
}

/* set up the channel slots for the given ALSA API channel map */
static int hdmi_manual_setup_channel_mapping(struct hda_codec *codec,
					     hda_nid_t pin_nid,
					     int chs, unsigned char *map,
						int ca)
{
	struct hdmi_spec *spec = codec->spec;
	int ordered_ca = get_channel_allocation_order(ca);
	int alsa_pos, hdmi_slot;
	int assignments[8] = {[0 ... 7] = 0xf};

	for (alsa_pos = 0; alsa_pos < chs; alsa_pos++) {

		hdmi_slot = to_cea_slot(ordered_ca, map[alsa_pos]);

		if (hdmi_slot < 0)
			continue; /* unassigned channel */

		assignments[hdmi_slot] = alsa_pos;
	}

	for (hdmi_slot = 0; hdmi_slot < 8; hdmi_slot++) {
		int err;

		err = spec->ops.pin_set_slot_channel(codec, pin_nid, hdmi_slot,
						     assignments[hdmi_slot]);
		if (err)
			return -EINVAL;
	}
	return 0;
}

/* store ALSA API channel map from the current default map */
static void hdmi_setup_fake_chmap(unsigned char *map, int ca)
{
	int i;
	int ordered_ca = get_channel_allocation_order(ca);
	for (i = 0; i < 8; i++) {
		if (i < channel_allocations[ordered_ca].channels)
			map[i] = from_cea_slot(ordered_ca, hdmi_channel_mapping[ca][i] & 0x0f);
		else
			map[i] = 0;
	}
}

static void hdmi_setup_channel_mapping(struct hda_codec *codec,
				       hda_nid_t pin_nid, bool non_pcm, int ca,
				       int channels, unsigned char *map,
				       bool chmap_set)
{
	if (!non_pcm && chmap_set) {
		hdmi_manual_setup_channel_mapping(codec, pin_nid,
						  channels, map, ca);
	} else {
		hdmi_std_setup_channel_mapping(codec, pin_nid, non_pcm, ca);
		hdmi_setup_fake_chmap(map, ca);
	}
	hdmi_debug_channel_mapping(codec, pin_nid);
}

static int hdmi_pin_set_slot_channel(struct hda_codec *codec, hda_nid_t pin_nid,
				     int asp_slot, int channel)
{
	return snd_hda_codec_write(codec, pin_nid, 0,
				   AC_VERB_SET_HDMI_CHAN_SLOT,
				   (channel << 4) | asp_slot);
}

static int hdmi_pin_get_slot_channel(struct hda_codec *codec, hda_nid_t pin_nid,
				     int asp_slot)
{
	return (snd_hda_codec_read(codec, pin_nid, 0,
				   AC_VERB_GET_HDMI_CHAN_SLOT,
				   asp_slot) & 0xf0) >> 4;
}

/*
 * Audio InfoFrame routines
 */

/*
 * Enable Audio InfoFrame Transmission
 */
static void hdmi_start_infoframe_trans(struct hda_codec *codec,
				       hda_nid_t pin_nid)
{
	hdmi_set_dip_index(codec, pin_nid, 0x0, 0x0);
	snd_hda_codec_write(codec, pin_nid, 0, AC_VERB_SET_HDMI_DIP_XMIT,
						AC_DIPXMIT_BEST);
}

/*
 * Disable Audio InfoFrame Transmission
 */
static void hdmi_stop_infoframe_trans(struct hda_codec *codec,
				      hda_nid_t pin_nid)
{
	hdmi_set_dip_index(codec, pin_nid, 0x0, 0x0);
	snd_hda_codec_write(codec, pin_nid, 0, AC_VERB_SET_HDMI_DIP_XMIT,
						AC_DIPXMIT_DISABLE);
}

static void hdmi_debug_dip_size(struct hda_codec *codec, hda_nid_t pin_nid)
{
#ifdef CONFIG_SND_DEBUG_VERBOSE
	int i;
	int size;

	size = snd_hdmi_get_eld_size(codec, pin_nid);
	snd_printd("HDMI: ELD buf size is %d\n", size);

	for (i = 0; i < 8; i++) {
		size = snd_hda_codec_read(codec, pin_nid, 0,
						AC_VERB_GET_HDMI_DIP_SIZE, i);
		snd_printd("HDMI: DIP GP[%d] buf size is %d\n", i, size);
	}
#endif
}

static void hdmi_clear_dip_buffers(struct hda_codec *codec, hda_nid_t pin_nid)
{
#ifdef BE_PARANOID
	int i, j;
	int size;
	int pi, bi;
	for (i = 0; i < 8; i++) {
		size = snd_hda_codec_read(codec, pin_nid, 0,
						AC_VERB_GET_HDMI_DIP_SIZE, i);
		if (size == 0)
			continue;

		hdmi_set_dip_index(codec, pin_nid, i, 0x0);
		for (j = 1; j < 1000; j++) {
			hdmi_write_dip_byte(codec, pin_nid, 0x0);
			hdmi_get_dip_index(codec, pin_nid, &pi, &bi);
			if (pi != i)
				snd_printd(KERN_INFO "dip index %d: %d != %d\n",
						bi, pi, i);
			if (bi == 0) /* byte index wrapped around */
				break;
		}
		snd_printd(KERN_INFO
			"HDMI: DIP GP[%d] buf reported size=%d, written=%d\n",
			i, size, j);
	}
#endif
}

static void hdmi_checksum_audio_infoframe(struct hdmi_audio_infoframe *hdmi_ai)
{
	u8 *bytes = (u8 *)hdmi_ai;
	u8 sum = 0;
	int i;

	hdmi_ai->checksum = 0;

	for (i = 0; i < sizeof(*hdmi_ai); i++)
		sum += bytes[i];

	hdmi_ai->checksum = -sum;
}

static void hdmi_fill_audio_infoframe(struct hda_codec *codec,
				      hda_nid_t pin_nid,
				      u8 *dip, int size)
{
	int i;

	hdmi_debug_dip_size(codec, pin_nid);
	hdmi_clear_dip_buffers(codec, pin_nid); /* be paranoid */

	hdmi_set_dip_index(codec, pin_nid, 0x0, 0x0);
	for (i = 0; i < size; i++)
		hdmi_write_dip_byte(codec, pin_nid, dip[i]);
}

static bool hdmi_infoframe_uptodate(struct hda_codec *codec, hda_nid_t pin_nid,
				    u8 *dip, int size)
{
	u8 val;
	int i;

	snd_printd("Enter %s\n", __func__);
	if (snd_hda_codec_read(codec, pin_nid, 0, AC_VERB_GET_HDMI_DIP_XMIT, 0)
							    != AC_DIPXMIT_BEST)
		return false;

	hdmi_set_dip_index(codec, pin_nid, 0x0, 0x0);
	for (i = 0; i < size; i++) {
		val = snd_hda_codec_read(codec, pin_nid, 0,
					 AC_VERB_GET_HDMI_DIP_DATA, 0);
		if (val != dip[i])
			return false;
	}

	snd_printd("Exit %s\n", __func__);

	return true;
}

static void hdmi_pin_setup_infoframe(struct hda_codec *codec,
				     hda_nid_t pin_nid,
				     int ca, int active_channels,
				     int conn_type)
{
	union audio_infoframe ai;

	memset(&ai, 0, sizeof(ai));
	if (conn_type == 0) { /* HDMI */
		struct hdmi_audio_infoframe *hdmi_ai = &ai.hdmi;

		hdmi_ai->type		= 0x84;
		hdmi_ai->ver		= 0x01;
		hdmi_ai->len		= 0x0a;
		hdmi_ai->CC02_CT47	= active_channels - 1;
		hdmi_ai->CA		= ca;
		hdmi_checksum_audio_infoframe(hdmi_ai);
	} else if (conn_type == 1) { /* DisplayPort */
		struct dp_audio_infoframe *dp_ai = &ai.dp;

		dp_ai->type		= 0x84;
		dp_ai->len		= 0x1b;
		dp_ai->ver		= 0x11 << 2;
		dp_ai->CC02_CT47	= active_channels - 1;
		dp_ai->CA		= ca;
	} else {
		snd_printk(KERN_ERR "HDMI: unknown connection type at pin %d\n",
			    pin_nid);
		return;
	}

	/*
	 * sizeof(ai) is used instead of sizeof(*hdmi_ai) or
	 * sizeof(*dp_ai) to avoid partial match/update problems when
	 * the user switches between HDMI/DP monitors.
	 */
	if (!hdmi_infoframe_uptodate(codec, pin_nid, ai.bytes,
					sizeof(ai))) {
		snd_printd("hdmi_pin_setup_infoframe: pin=%d channels=%d ca=0x%02x\n",
			    pin_nid,
			    active_channels, ca);
		hdmi_stop_infoframe_trans(codec, pin_nid);
		hdmi_fill_audio_infoframe(codec, pin_nid,
					    ai.bytes, sizeof(ai));
		hdmi_start_infoframe_trans(codec, pin_nid);
	}
}

static void hdmi_setup_audio_infoframe(struct hda_codec *codec,
				       struct hdmi_spec_per_pin *per_pin,
				       bool non_pcm)
{
	struct hdmi_spec *spec = codec->spec;
	hda_nid_t pin_nid = per_pin->pin_nid;
	int channels = per_pin->channels;
	int active_channels;
	struct hdmi_eld *eld;
	int ca, ordered_ca;

	if (!channels)
		return;

	eld = &per_pin->sink_eld;
	if (!eld->monitor_present)
		return;

	if (!non_pcm && per_pin->chmap_set)
		ca = hdmi_manual_channel_allocation(channels, per_pin->chmap);
	else
		ca = hdmi_channel_allocation(eld, channels);
	if (ca < 0)
		ca = 0;

	ordered_ca = get_channel_allocation_order(ca);
	active_channels = channel_allocations[ordered_ca].channels;

	hdmi_set_channel_count(codec, per_pin->cvt_nid, active_channels);

	/*
	 * always configure channel mapping, it may have been changed by the
	 * user in the meantime
	 */
	hdmi_setup_channel_mapping(codec, pin_nid, non_pcm, ca,
				   channels, per_pin->chmap,
				   per_pin->chmap_set);

	spec->ops.pin_setup_infoframe(codec, pin_nid, ca, active_channels,
				      eld->info.conn_type);

	per_pin->non_pcm = non_pcm;
}

/*
 * Unsolicited events
 */
static bool hdmi_present_sense(struct hdmi_spec_per_pin *per_pin, int repoll);

static void jack_callback(struct hda_codec *codec, struct hda_jack_tbl *jack)
{
	struct hdmi_spec *spec = codec->spec;
	int pin_idx = pin_nid_to_pin_index(codec, jack->nid);
	if (pin_idx < 0)
		return;

	if (hdmi_present_sense(get_pin(spec, pin_idx), 1))
		snd_hda_jack_report_sync(codec);
}

static void hdmi_intrinsic_event(struct hda_codec *codec, unsigned int res)
{
	int tag = res >> AC_UNSOL_RES_TAG_SHIFT;
	struct hda_jack_tbl *jack;
	int dev_entry = (res & AC_UNSOL_RES_DE) >> AC_UNSOL_RES_DE_SHIFT;

	jack = snd_hda_jack_tbl_get_from_tag(codec, tag);
	if (!jack)
		return;
	jack->jack_dirty = 1;

	snd_printd("HDMI hot plug event: Codec=%d Pin=%d Device=%d Inactive=%d Presence_Detect=%d ELD_Valid=%d\n",
		codec->addr, jack->nid, dev_entry, !!(res & AC_UNSOL_RES_IA),
		!!(res & AC_UNSOL_RES_PD), !!(res & AC_UNSOL_RES_ELDV));

	jack_callback(codec, jack);
}

static void hdmi_non_intrinsic_event(struct hda_codec *codec, unsigned int res)
{
	int tag = res >> AC_UNSOL_RES_TAG_SHIFT;
	int subtag = (res & AC_UNSOL_RES_SUBTAG) >> AC_UNSOL_RES_SUBTAG_SHIFT;
	int cp_state = !!(res & AC_UNSOL_RES_CP_STATE);
	int cp_ready = !!(res & AC_UNSOL_RES_CP_READY);

	snd_printd("HDMI CP event: CODEC=%d TAG=%d SUBTAG=0x%x CP_STATE=%d CP_READY=%d\n",
		codec->addr,
		tag,
		subtag,
		cp_state,
		cp_ready);
	/* TODO */
	if (cp_state)
		;
	if (cp_ready)
		;
}

static void intel_hdmi_unsol_event(struct snd_soc_hda_device *hdev, unsigned int res)
{
	struct snd_soc_hda_codec *hcodec = snd_soc_hda_get_drvdata(hdev);
	struct hda_codec *codec = &hcodec->hdac;
	int tag = res >> AC_UNSOL_RES_TAG_SHIFT;
	int subtag = (res & AC_UNSOL_RES_SUBTAG) >> AC_UNSOL_RES_SUBTAG_SHIFT;

	if (!snd_hda_jack_tbl_get_from_tag(codec, tag)) {
		snd_printd(KERN_INFO "Unexpected HDMI event tag 0x%x\n", tag);
		return;
	}

	if (subtag == 0)
		hdmi_intrinsic_event(codec, res);
	else
		hdmi_non_intrinsic_event(codec, res);
}

/*
 * HDA/HDMI auto parsing
 */
static int hdmi_read_pin_conn(struct hda_codec *codec, int pin_idx)
{
	struct hdmi_spec *spec = codec->spec;
	struct hdmi_spec_per_pin *per_pin = get_pin(spec, pin_idx);
	hda_nid_t pin_nid = per_pin->pin_nid;

	if (!(get_wcaps(codec, pin_nid) & AC_WCAP_CONN_LIST)) {
		snd_printk(KERN_WARNING
			   "HDMI: pin %d wcaps %#x "
			   "does not support connection list\n",
			   pin_nid, get_wcaps(codec, pin_nid));
		return -EINVAL;
	}

	per_pin->num_mux_nids = snd_hda_get_connections(codec, pin_nid,
							per_pin->mux_nids,
							HDA_MAX_CONNECTIONS);

	return 0;
}

static bool hdmi_present_sense(struct hdmi_spec_per_pin *per_pin, int repoll)
{
	struct hda_jack_tbl *jack;
	struct hda_codec *codec = per_pin->codec;
	struct hdmi_spec *spec = codec->spec;
	struct hdmi_eld *eld = &spec->temp_eld;
	struct hdmi_eld *pin_eld = &per_pin->sink_eld;
	hda_nid_t pin_nid = per_pin->pin_nid;
	/*
	 * Always execute a GetPinSense verb here, even when called from
	 * hdmi_intrinsic_event; for some NVIDIA HW, the unsolicited
	 * response's PD bit is not the real PD value, but indicates that
	 * the real PD value changed. An older version of the HD-audio
	 * specification worked this way. Hence, we just ignore the data in
	 * the unsolicited response to avoid custom WARs.
	 */
	int present;
	bool update_eld = false;
	bool eld_changed = false;
	bool ret;

	present = snd_hda_pin_sense(codec, pin_nid);

	mutex_lock(&per_pin->lock);
	pin_eld->monitor_present = !!(present & AC_PINSENSE_PRESENCE);
	if (pin_eld->monitor_present)
		eld->eld_valid  = !!(present & AC_PINSENSE_ELDV);
	else
		eld->eld_valid = false;

	snd_printd("HDMI status: Codec=%d Pin=%d Presence_Detect=%d ELD_Valid=%d\n",
		codec->addr, pin_nid, pin_eld->monitor_present, eld->eld_valid);

	if (eld->eld_valid) {
		if (spec->ops.pin_get_eld(codec, pin_nid, eld->eld_buffer,
						     &eld->eld_size) < 0)
			eld->eld_valid = false;
		else {
			memset(&eld->info, 0, sizeof(struct parsed_hdmi_eld));
			if (snd_hdmi_parse_eld(&eld->info, eld->eld_buffer,
						    eld->eld_size) < 0)
				eld->eld_valid = false;
		}

		if (eld->eld_valid) {
			snd_hdmi_show_eld(&eld->info);
			update_eld = true;
		} else if (repoll) {
			queue_delayed_work(spec->workq, &per_pin->work,
				msecs_to_jiffies(300));
			goto unlock;
		}
	}

	if (pin_eld->eld_valid && !eld->eld_valid) {
		update_eld = true;
		eld_changed = true;
	}
	if (update_eld) {
		bool old_eld_valid = pin_eld->eld_valid;
		pin_eld->eld_valid = eld->eld_valid;
		eld_changed = pin_eld->eld_size != eld->eld_size ||
			      memcmp(pin_eld->eld_buffer, eld->eld_buffer,
				     eld->eld_size) != 0;
		if (eld_changed)
			memcpy(pin_eld->eld_buffer, eld->eld_buffer,
			       eld->eld_size);
		pin_eld->eld_size = eld->eld_size;
		pin_eld->info = eld->info;

		/*
		 * Re-setup pin and infoframe. This is needed e.g. when
		 * - sink is first plugged-in (infoframe is not set up if !monitor_present)
		 * - transcoder can change during stream playback on Haswell
		 */
		if (eld->eld_valid && !old_eld_valid && per_pin->setup)
			hdmi_setup_audio_infoframe(codec, per_pin,
						   per_pin->non_pcm);
	}

	if (eld_changed)
		snd_ctl_notify(codec->card,
			       SNDRV_CTL_EVENT_MASK_VALUE | SNDRV_CTL_EVENT_MASK_INFO,
			       &per_pin->eld_ctl->id);
 unlock:
	ret = !repoll || !pin_eld->monitor_present || pin_eld->eld_valid;

	jack = snd_hda_jack_tbl_get(codec, pin_nid);
	if (jack)
		jack->block_report = !ret;

	mutex_unlock(&per_pin->lock);

	return ret;
}

static void hdmi_repoll_eld(struct work_struct *work)
{
	struct hdmi_spec_per_pin *per_pin =
	container_of(to_delayed_work(work), struct hdmi_spec_per_pin, work);

	if (per_pin->repoll_count++ > 6)
		per_pin->repoll_count = 0;

	if (hdmi_present_sense(per_pin, per_pin->repoll_count))
		snd_hda_jack_report_sync(per_pin->codec);
}

/*
 * Callbacks
 */

/* HBR should be Non-PCM, 8 channels */
#define is_hbr_format(format) \
	((format & AC_FMT_TYPE_NON_PCM) && (format & AC_FMT_CHAN_MASK) == 7)

static int hdmi_pin_hbr_setup(struct hda_codec *codec, hda_nid_t pin_nid,
			      bool hbr)
{
	int pinctl, new_pinctl;

	if (snd_hda_query_pin_caps(codec, pin_nid) & AC_PINCAP_HBR) {
		pinctl = snd_hda_codec_read(codec, pin_nid, 0,
					    AC_VERB_GET_PIN_WIDGET_CONTROL, 0);

		if (pinctl < 0)
			return hbr ? -EINVAL : 0;

		new_pinctl = pinctl & ~AC_PINCTL_EPT;
		if (hbr)
			new_pinctl |= AC_PINCTL_EPT_HBR;
		else
			new_pinctl |= AC_PINCTL_EPT_NATIVE;

		snd_printd("hdmi_pin_hbr_setup: NID=0x%x, %spinctl=0x%x\n",
			    pin_nid,
			    pinctl == new_pinctl ? "" : "new-",
			    new_pinctl);

		if (pinctl != new_pinctl)
			snd_hda_codec_write(codec, pin_nid, 0,
					    AC_VERB_SET_PIN_WIDGET_CONTROL,
					    new_pinctl);
	} else if (hbr)
		return -EINVAL;

	return 0;
}

static int hdmi_setup_stream(struct hda_codec *codec, hda_nid_t cvt_nid,
			      hda_nid_t pin_nid, u32 stream_tag, int format)
{
	struct hdmi_spec *spec = codec->spec;
	int err;

	snd_printd("Enter %s\n", __func__);
	snd_printd("cvt nid %d pnid %d stream %d format 0x%x\n", cvt_nid, pin_nid, stream_tag, format);

	err = spec->ops.pin_hbr_setup(codec, pin_nid, is_hbr_format(format));

	if (err) {
		snd_printk(KERN_ERR "hdmi_setup_stream: HBR is not supported\n");
		return err;
	}

	snd_hda_codec_setup_stream(codec, cvt_nid, stream_tag, 0, format);
	return 0;
}

static int hdmi_choose_cvt(struct hda_codec *codec,
			int pin_idx, int *cvt_id, int *mux_id)
{
	struct hdmi_spec *spec = codec->spec;
	struct hdmi_spec_per_pin *per_pin;
	struct hdmi_spec_per_cvt *per_cvt = NULL;
	int cvt_idx, mux_idx = 0;

	per_pin = get_pin(spec, pin_idx);

	/* Dynamically assign converter to stream */
	for (cvt_idx = 0; cvt_idx < spec->num_cvts; cvt_idx++) {
		per_cvt = get_cvt(spec, cvt_idx);

		/* Must not already be assigned */
		if (per_cvt->assigned)
			continue;
		/* Must be in pin's mux's list of converters */
		for (mux_idx = 0; mux_idx < per_pin->num_mux_nids; mux_idx++)
			if (per_pin->mux_nids[mux_idx] == per_cvt->cvt_nid)
				break;
		/* Not in mux list */
		if (mux_idx == per_pin->num_mux_nids)
			continue;
		break;
	}

	/* No free converters */
	if (cvt_idx == spec->num_cvts)
		return -ENODEV;

	per_pin->mux_idx = mux_idx;

	if (cvt_id)
		*cvt_id = cvt_idx;
	if (mux_id)
		*mux_id = mux_idx;

	return 0;
}

/* update PCM info based on ELD */
int hdmi_eld_update_pcm_info(struct parsed_hdmi_eld *e, struct snd_pcm_runtime *rtd,
				struct hdmi_spec_per_cvt *per_cvt)
{
	u32 rates;
	u64 formats;
	unsigned int maxbps;
	unsigned int channels_max;
	int i;

	/* assume basic audio support (the basic audio flag is not in ELD;
	 * however, all audio capable sinks are required to support basic
	 * audio) */
	rates = SNDRV_PCM_RATE_32000 | SNDRV_PCM_RATE_44100 |
		SNDRV_PCM_RATE_48000;
	formats = SNDRV_PCM_FMTBIT_S16_LE;
	maxbps = 16;
	channels_max = 2;
	for (i = 0; i < e->sad_count; i++) {
		struct cea_sad *a = &e->sad[i];
		rates |= a->rates;
		if (a->channels > channels_max)
			channels_max = a->channels;
		if (a->format == 1) { /* AUDIO_CODING_TYPE_LPCM */
			if (a->sample_bits & AC_SUPPCM_BITS_20) {
				formats |= SNDRV_PCM_FMTBIT_S32_LE;
				if (maxbps < 20)
					maxbps = 20;
			}
			if (a->sample_bits & AC_SUPPCM_BITS_24) {
				formats |= SNDRV_PCM_FMTBIT_S32_LE;
				if (maxbps < 24)
					maxbps = 24;
			}
		}
	}

	/* Check for valid params */
	if (per_cvt->channels_min > min(per_cvt->channels_max, channels_max) ||
		    !rates || !formats)
		return -ENODEV;

	/* Store the updated parameters */
	rtd->hw.channels_min = per_cvt->channels_min;
	rtd->hw.channels_max = min(per_cvt->channels_max, channels_max);
	rtd->hw.formats &= formats;
	rtd->hw.rates &= rates;

	snd_printd("%s: rate 0x%x formats 0x%llx\n", __func__, rates, formats);
	snd_printd("mbps 0x%x ch_max 0x%x\n", maxbps, channels_max);

	return 0;
}

static int hdmi_pcm_open(struct snd_pcm_substream *substream,
			struct snd_soc_dai *dai)
{
	struct snd_soc_hda_codec *hcodec = snd_soc_dai_get_drvdata(dai);
	struct hda_codec *codec = &hcodec->hdac;
	struct hdmi_spec *spec = codec->spec;

	struct snd_pcm_runtime *runtime = substream->runtime;
	int pin_idx, cvt_idx, mux_idx = 0;
	struct hdmi_spec_per_pin *per_pin;
	struct hdmi_eld *eld;
	struct hdmi_spec_per_cvt *per_cvt = NULL;
	struct hdmi_dai_pin_map *map;
	int err;

	snd_printd("In %s\n", __func__);

	/* Validate */
	pin_idx = dai_map_to_pin_index(codec, dai->name);
	if (snd_BUG_ON(pin_idx < 0))
		return -EINVAL;

	per_pin = get_pin(spec, pin_idx);
	eld = &per_pin->sink_eld;

	err = hdmi_choose_cvt(codec, pin_idx, &cvt_idx, &mux_idx);
	if (err < 0)
		return err;

	per_cvt = get_cvt(spec, cvt_idx);

	/* Claim converter */
	per_cvt->assigned = 1;
	per_pin->cvt_nid = per_cvt->cvt_nid;
	map = get_dai_pin_map(spec, pin_idx);
	map->nid = per_cvt->cvt_nid;

	snd_printd("pin id %d cvt nid %d\n", pin_idx, map->nid);

	snd_hda_codec_write_cache(codec, per_pin->pin_nid, 0,
			    AC_VERB_SET_CONNECT_SEL,
			    mux_idx);
	snd_hda_spdif_ctls_assign(codec, pin_idx, per_cvt->cvt_nid);

	snd_printd("cvt ch min 0x%x max 0x%x\n", per_cvt->channels_min,
					per_cvt->channels_max);

	snd_printd("rates 0x%x formats 0x%llx\n", per_cvt->rates,
					per_cvt->formats);

	err = hdmi_eld_update_pcm_info(&eld->info, runtime, per_cvt);
	if (err < 0) {
			per_cvt->assigned = 0;
			map->nid = 0;
			snd_hda_spdif_ctls_unassign(codec, pin_idx);
			return err;
	}

	snd_pcm_hw_constraint_step(substream->runtime, 0,
				   SNDRV_PCM_HW_PARAM_CHANNELS, 2);
	return 0;
}

static void hdmi_pcm_close(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_hda_codec *hcodec = snd_soc_dai_get_drvdata(dai);
	struct hda_codec *codec = &hcodec->hdac;
	struct hdmi_spec *spec = codec->spec;

	int cvt_idx, pin_idx;
	struct hdmi_spec_per_cvt *per_cvt;
	struct hdmi_spec_per_pin *per_pin;
	struct hdmi_dai_pin_map *map;

	snd_printd("In %s\n", __func__);

	/* Validate */
	pin_idx = dai_map_to_pin_index(codec, dai->name);
	if (snd_BUG_ON(pin_idx < 0))
		return;

	map = get_dai_pin_map(spec, pin_idx);
	if (map->nid) {
		cvt_idx = cvt_nid_to_cvt_index(codec, map->nid);
		if (snd_BUG_ON(cvt_idx < 0))
			return;
		per_cvt = get_cvt(spec, cvt_idx);

		snd_BUG_ON(!per_cvt->assigned);
		per_cvt->assigned = 0;
		map->nid = 0;

		per_pin = get_pin(spec, pin_idx);

		snd_hda_spdif_ctls_unassign(codec, pin_idx);

		mutex_lock(&per_pin->lock);
		per_pin->chmap_set = false;
		memset(per_pin->chmap, 0, sizeof(per_pin->chmap));

		per_pin->setup = false;
		per_pin->channels = 0;
		mutex_unlock(&per_pin->lock);
	}

	return;
}

static int hdmi_set_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *hparams, struct snd_soc_dai *dai)
{
	struct snd_soc_hda_codec *hcodec = snd_soc_dai_get_drvdata(dai);
	struct hda_codec *codec = &hcodec->hdac;
	struct hdmi_spec *spec = codec->spec;

	struct snd_soc_hda_dma_params *dd;
	struct hdmi_dai_pin_map *map;
	int pin_idx, format;

	struct hda_spdif_out *spdif;
	unsigned short ctls;

	snd_printd("In %s\n", __func__);

	/* Validate */
	pin_idx = dai_map_to_pin_index(codec, dai->name);
	if (snd_BUG_ON(pin_idx < 0))
		return -EINVAL;

	map = get_dai_pin_map(spec, pin_idx);

	mutex_lock(&codec->spdif_mutex);
	spdif = snd_hda_spdif_out_of_nid(codec, map->nid);
	ctls  = spdif ? spdif->ctls : 0;
	mutex_unlock(&codec->spdif_mutex);

	if (map->nid) {
		dd = kzalloc(sizeof(*dd), GFP_KERNEL);
		dd->format = snd_hda_calc_stream_format(params_rate(hparams),
			params_channels(hparams), params_format(hparams),
			24, 0);
		snd_soc_dai_set_dma_data(dai, substream, (void *)dd);
	}
	return 0;
}

static bool check_non_pcm_per_cvt(struct hda_codec *codec, hda_nid_t cvt_nid)
{
	struct hda_spdif_out *spdif;
	bool non_pcm;

	mutex_lock(&codec->spdif_mutex);
	spdif = snd_hda_spdif_out_of_nid(codec, cvt_nid);
	non_pcm = !!(spdif->status & IEC958_AES0_NONAUDIO);
	mutex_unlock(&codec->spdif_mutex);
	return non_pcm;
}

static int hdmi_playback_prepare(struct snd_pcm_substream *substream,
				struct snd_soc_dai *dai)
{
	struct snd_soc_hda_codec *hcodec = snd_soc_dai_get_drvdata(dai);
	struct hda_codec *codec = &hcodec->hdac;
	struct hdmi_spec *spec = codec->spec;
	struct snd_soc_pcm_runtime *rtd = snd_pcm_substream_chip(substream);

	hda_nid_t cvt_nid, pin_nid;
	int pin_idx, cvt_idx;
	struct hdmi_spec_per_pin *per_pin;
	bool non_pcm;
	struct hdmi_dai_pin_map *map;
	struct snd_soc_dai *cpu_dai;
	struct snd_soc_hda_dma_params *dd;
	struct hdmi_spec_per_cvt *per_cvt;

	snd_printd("In %s\n", __func__);

	/* Validate */
	pin_idx = dai_map_to_pin_index(codec, dai->name);
	if (snd_BUG_ON(pin_idx < 0))
		return -EINVAL;

	map = get_dai_pin_map(spec, pin_idx);
	cvt_nid = map->nid;
	per_pin = get_pin(spec, pin_idx);
	pin_nid = per_pin->pin_nid;

	snd_printd("pin id %d pin nid %d cvt nid %d\n", pin_idx, pin_nid, cvt_nid);

	non_pcm = check_non_pcm_per_cvt(codec, cvt_nid);

	cvt_idx = cvt_nid_to_cvt_index(codec, cvt_nid);
	if (snd_BUG_ON(cvt_idx < 0))
			return -EINVAL;
	per_cvt = get_cvt(spec, cvt_idx);

	mutex_lock(&per_pin->lock);
	per_pin->channels = substream->runtime->channels;
	per_pin->setup = true;

	cpu_dai = rtd->cpu_dai;
	dd = (struct snd_soc_hda_dma_params *)snd_soc_dai_get_dma_data(dai, substream);
	snd_printd("stream tag from cpu dai %d format in cvt 0x%llx\n", dd->stream_tag,
				dd->format);

	hdmi_setup_audio_infoframe(codec, per_pin, non_pcm);
	mutex_unlock(&per_pin->lock);

	return spec->ops.setup_stream(codec, cvt_nid, pin_nid, dd->stream_tag, dd->format);
}

static int hdmi_playback_cleanup(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_hda_codec *hcodec = snd_soc_dai_get_drvdata(dai);
	struct hda_codec *codec = &hcodec->hdac;
	struct hdmi_spec *spec = codec->spec;
	struct snd_soc_hda_dma_params *dd;

	int pin_idx;
	struct hdmi_dai_pin_map *map;

	/* Validate */
	pin_idx = dai_map_to_pin_index(codec, dai->name);
	if (snd_BUG_ON(pin_idx < 0))
		return -EINVAL;

	map = get_dai_pin_map(spec, pin_idx);

	__snd_hda_codec_cleanup_stream(codec, map->nid, 1);

	dd = (struct snd_soc_hda_dma_params *)snd_soc_dai_get_dma_data(dai, substream);
	kfree(dd);

	return 0;
}

static int hdmi_create_dai_pin_map(struct hda_codec *codec)
{
	struct hdmi_spec *spec = codec->spec;
	int pin_idx;

	for (pin_idx = 0; pin_idx < spec->num_pins; pin_idx++) {
		struct hdmi_dai_pin_map *map;
		struct hdmi_spec_per_pin *per_pin;

		per_pin = get_pin(spec, pin_idx);

		snd_printd("per_pin->num_mux %d\n", per_pin->num_mux_nids);

		map = get_dai_pin_map(spec, pin_idx);

		sprintf(map->dai_name, "intel-hdmi-hif%d", pin_idx + 1);

		map->pin_idx = pin_idx; /* save the pin id */
	}

	return 0;
}

static int hdmi_add_pin(struct hda_codec *codec, hda_nid_t pin_nid)
{
	struct hdmi_spec *spec = codec->spec;
	unsigned int caps, config;
	struct hdmi_spec_per_pin *per_pin;
	int err, pin_idx;

	caps = snd_hda_query_pin_caps(codec, pin_nid);
	if (!(caps & (AC_PINCAP_HDMI | AC_PINCAP_DP)))
		return 0;

	config = snd_hda_codec_get_pincfg(codec, pin_nid);
	if (get_defcfg_connect(config) == AC_JACK_PORT_NONE)
		return 0;

	pin_idx = spec->num_pins;
	per_pin = snd_array_new(&spec->pins);
	if (!per_pin)
		return -ENOMEM;

	per_pin->pin_nid = pin_nid;
	per_pin->non_pcm = false;

	err = hdmi_read_pin_conn(codec, pin_idx);
	if (err < 0)
		return err;

	spec->num_pins++;

	return 0;
}

static int hdmi_add_cvt(struct hda_codec *codec, hda_nid_t cvt_nid)
{
	struct hdmi_spec *spec = codec->spec;
	struct hdmi_spec_per_cvt *per_cvt;
	unsigned int chans;
	int err;

	chans = get_wcaps(codec, cvt_nid);
	chans = get_wcaps_channels(chans);

	per_cvt = snd_array_new(&spec->cvts);
	if (!per_cvt)
		return -ENOMEM;

	per_cvt->cvt_nid = cvt_nid;
	per_cvt->channels_min = 2;
	if (chans <= 16) {
		per_cvt->channels_max = chans;
		if (chans > spec->channels_max)
			spec->channels_max = chans;
	}

	err = snd_hda_query_supported_pcm(codec, cvt_nid,
			&per_cvt->rates,
			&per_cvt->formats,
			&per_cvt->maxbps);
	if (err < 0)
		return err;

	if (spec->num_cvts < ARRAY_SIZE(spec->cvt_nids))
		spec->cvt_nids[spec->num_cvts] = cvt_nid;
	spec->num_cvts++;

	return 0;
}

static int hdmi_parse_codec(struct hda_codec *codec)
{
	hda_nid_t nid;
	int i, nodes;

	struct hdmi_spec *spec = codec->spec;

	snd_printd("In %s\n", __func__);

	nodes = snd_hda_get_sub_nodes(codec, codec->afg, &nid);
	if (!nid || nodes < 0) {
		snd_printk(KERN_WARNING "HDMI: failed to get afg sub nodes\n");
		return -EINVAL;
	}

	snd_printd("Number of nodes %d\n", nodes);

	for (i = 0; i < nodes; i++, nid++) {
		unsigned int caps;
		unsigned int type;

		caps = get_wcaps(codec, nid);
		type = get_wcaps_type(caps);

		if (!(caps & AC_WCAP_DIGITAL))
			continue;

		switch (type) {
		case AC_WID_AUD_OUT:
			hdmi_add_cvt(codec, nid);
			break;
		case AC_WID_PIN:
			hdmi_add_pin(codec, nid);
			break;
		}
	}

	snd_printd("num of cvt %d num pins %d\n", spec->num_cvts, spec->num_pins);

	hdmi_create_dai_pin_map(codec);

	return 0;
}

static void hdmi_array_init(struct hdmi_spec *spec, int nums)
{
	snd_array_init(&spec->pins, sizeof(struct hdmi_spec_per_pin), nums);
	snd_array_init(&spec->cvts, sizeof(struct hdmi_spec_per_cvt), nums);
}

static void hdmi_array_free(struct hdmi_spec *spec)
{
	snd_array_free(&spec->pins);
	snd_array_free(&spec->cvts);
}

/*
 * ALSA API channel-map control callbacks
 */
static int hdmi_chmap_ctl_info(struct snd_kcontrol *kcontrol,
			       struct snd_ctl_elem_info *uinfo)
{
	struct snd_pcm_chmap *info = snd_kcontrol_chip(kcontrol);
	struct hda_codec *codec = info->private_data;
	struct hdmi_spec *spec = codec->spec;
	uinfo->type = SNDRV_CTL_ELEM_TYPE_INTEGER;
	uinfo->count = spec->channels_max;
	uinfo->value.integer.min = 0;
	uinfo->value.integer.max = SNDRV_CHMAP_LAST;
	return 0;
}

static int hdmi_chmap_cea_alloc_validate_get_type(struct cea_channel_speaker_allocation *cap,
						  int channels)
{
	/* If the speaker allocation matches the channel count, it is OK.*/
	if (cap->channels != channels)
		return -1;

	/* all channels are remappable freely */
	return SNDRV_CTL_TLVT_CHMAP_VAR;
}

static void hdmi_cea_alloc_to_tlv_chmap(struct cea_channel_speaker_allocation *cap,
					unsigned int *chmap, int channels)
{
	int count = 0;
	int c;

	for (c = 7; c >= 0; c--) {
		int spk = cap->speakers[c];
		if (!spk)
			continue;

		chmap[count++] = spk_to_chmap(spk);
	}

	WARN_ON(count != channels);
}

static int hdmi_chmap_ctl_tlv(struct snd_kcontrol *kcontrol, int op_flag,
			      unsigned int size, unsigned int __user *tlv)
{
	struct snd_pcm_chmap *info = snd_kcontrol_chip(kcontrol);
	struct hda_codec *codec = info->private_data;
	struct hdmi_spec *spec = codec->spec;
	unsigned int __user *dst;
	int chs, count = 0;

	if (size < 8)
		return -ENOMEM;
	if (put_user(SNDRV_CTL_TLVT_CONTAINER, tlv))
		return -EFAULT;
	size -= 8;
	dst = tlv + 2;
	for (chs = 2; chs <= spec->channels_max; chs++) {
		int i;
		struct cea_channel_speaker_allocation *cap;
		cap = channel_allocations;
		for (i = 0; i < ARRAY_SIZE(channel_allocations); i++, cap++) {
			int chs_bytes = chs * 4;
			int type = spec->ops.chmap_cea_alloc_validate_get_type(cap, chs);
			unsigned int tlv_chmap[8];

			if (type < 0)
				continue;
			if (size < 8)
				return -ENOMEM;
			if (put_user(type, dst) ||
			    put_user(chs_bytes, dst + 1))
				return -EFAULT;
			dst += 2;
			size -= 8;
			count += 8;
			if (size < chs_bytes)
				return -ENOMEM;
			size -= chs_bytes;
			count += chs_bytes;
			spec->ops.cea_alloc_to_tlv_chmap(cap, tlv_chmap, chs);
			if (copy_to_user(dst, tlv_chmap, chs_bytes))
				return -EFAULT;
			dst += chs;
		}
	}
	if (put_user(count, tlv + 1))
		return -EFAULT;
	return 0;
}

static int hdmi_chmap_ctl_get(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_pcm_chmap *info = snd_kcontrol_chip(kcontrol);
	struct hda_codec *codec = info->private_data;
	struct hdmi_spec *spec = codec->spec;
	int pin_idx = kcontrol->private_value;
	struct hdmi_spec_per_pin *per_pin = get_pin(spec, pin_idx);
	int i;

	for (i = 0; i < ARRAY_SIZE(per_pin->chmap); i++)
		ucontrol->value.integer.value[i] = per_pin->chmap[i];
	return 0;
}

static int hdmi_chmap_ctl_put(struct snd_kcontrol *kcontrol,
			      struct snd_ctl_elem_value *ucontrol)
{
	struct snd_pcm_chmap *info = snd_kcontrol_chip(kcontrol);
	struct hda_codec *codec = info->private_data;
	struct hdmi_spec *spec = codec->spec;
	int pin_idx = kcontrol->private_value;
	struct hdmi_spec_per_pin *per_pin = get_pin(spec, pin_idx);
	unsigned int ctl_idx;
	struct snd_pcm_substream *substream;
	unsigned char chmap[8];
	int i, ca, prepared = 0;

	ctl_idx = snd_ctl_get_ioffidx(kcontrol, &ucontrol->id);
	substream = snd_pcm_chmap_substream(info, ctl_idx);
	if (!substream || !substream->runtime)
		return 0; /* just for avoiding error from alsactl restore */
	switch (substream->runtime->status->state) {
	case SNDRV_PCM_STATE_OPEN:
	case SNDRV_PCM_STATE_SETUP:
		break;
	case SNDRV_PCM_STATE_PREPARED:
		prepared = 1;
		break;
	default:
		return -EBUSY;
	}
	memset(chmap, 0, sizeof(chmap));
	for (i = 0; i < ARRAY_SIZE(chmap); i++)
		chmap[i] = ucontrol->value.integer.value[i];
	if (!memcmp(chmap, per_pin->chmap, sizeof(chmap)))
		return 0;
	ca = hdmi_manual_channel_allocation(ARRAY_SIZE(chmap), chmap);
	if (ca < 0)
		return -EINVAL;

	mutex_lock(&per_pin->lock);
	per_pin->chmap_set = true;
	memcpy(per_pin->chmap, chmap, sizeof(chmap));
	if (prepared)
		hdmi_setup_audio_infoframe(codec, per_pin, per_pin->non_pcm);
	mutex_unlock(&per_pin->lock);
	return 0;
}

static int intel_hdmi_build_jack(struct hda_codec *codec, int pin_idx)
{
	char hdmi_str[32] = "HDMI/DP";
	struct hdmi_spec *spec = codec->spec;
	struct hdmi_spec_per_pin *per_pin = get_pin(spec, pin_idx);

	sprintf(hdmi_str + strlen(hdmi_str), ", pin=%d", pin_idx);

	if (!is_jack_detectable(codec, per_pin->pin_nid))
		strncat(hdmi_str, " Phantom",
			sizeof(hdmi_str) - strlen(hdmi_str) - 1);

	return snd_hda_jack_add_kctl(codec, per_pin->pin_nid, hdmi_str, 0);
}

struct snd_pcm *get_pcm(struct snd_soc_card *card, const char *dai_name)
{

	int num_rtd = card->num_rtd;
	int i;

	snd_printd("%s: num %d\n", __func__, num_rtd);

	for (i = 0; i < num_rtd; i++) {
		if (!strcmp(card->rtd[i].dai_link->codec_dai_name, dai_name))
			return card->rtd[i].pcm;
	}

	snd_printk(KERN_ERR"Pcm pointer not found....\n");
	return NULL;
}

/* FIXME: Channel Map controls needs to created for Multichannel support */
int hdmi_add_ch_map(struct snd_soc_codec *scodec, struct snd_soc_dai *dai)
{

	struct snd_soc_hda_codec *hcodec = snd_soc_codec_get_drvdata(scodec);
	struct hda_codec *codec = &hcodec->hdac;

	struct snd_pcm *pcm = NULL;
	struct snd_pcm_chmap *chmap;
	struct snd_kcontrol *kctl;
	int i;
	int err;
	int pin_idx;

	snd_printd("In %s : dai name %s\n", __func__, dai->name);

	/* Validate */
	pin_idx = dai_map_to_pin_index(codec, dai->name);
	if (snd_BUG_ON(pin_idx < 0))
		return -EINVAL;

	pcm = get_pcm(scodec->card, dai->name);
	/* add channel maps */
	if (!pcm)
		return -EINVAL;

	err = snd_pcm_add_chmap_ctls(pcm,
				SNDRV_PCM_STREAM_PLAYBACK,
				NULL, 0, pin_idx, &chmap);
	if (err < 0 && err != -EBUSY)
		return err;

	/* override handlers */
	chmap->private_data = codec;
	kctl = chmap->kctl;
	for (i = 0; i < kctl->count; i++)
		kctl->vd[i].access |= SNDRV_CTL_ELEM_ACCESS_WRITE;
	kctl->info = hdmi_chmap_ctl_info;
	kctl->get = hdmi_chmap_ctl_get;
	kctl->put = hdmi_chmap_ctl_put;
	kctl->tlv.c = hdmi_chmap_ctl_tlv;

	return 0;
}

static int intel_hdmi_build_controls(struct snd_soc_hda_codec *hcodec)
{
	struct hda_codec *codec = &hcodec->hdac;
	struct hdmi_spec *spec = codec->spec;
	int err, pin_idx;

	snd_printd("In %s num_pins %d\n", __func__, spec->num_pins);

	for (pin_idx = 0; pin_idx < spec->num_pins; pin_idx++) {
		struct hdmi_spec_per_pin *per_pin = get_pin(spec, pin_idx);

		err = intel_hdmi_build_jack(codec, pin_idx);
		if (err < 0)
			return err;

		snd_printd("pin_idx %d nid %d\n", pin_idx, per_pin->pin_nid);

		err = snd_hda_create_dig_out_ctls(codec,
				per_pin->pin_nid,
				per_pin->mux_nids[0],
				HDA_PCM_TYPE_HDMI);
		if (err < 0)
			return err;

		snd_hda_spdif_ctls_unassign(codec, pin_idx);

		/* add control for ELD Bytes */
		err = hdmi_create_eld_ctl(codec, pin_idx);
		if (err < 0)
			return err;

		hdmi_present_sense(per_pin, 0);
	}

	return 0;
}

static int generic_hdmi_init_per_pins(struct hda_codec *codec)
{
	struct hdmi_spec *spec = codec->spec;
	int pin_idx;

	spec->workq = create_singlethread_workqueue("HDMI-AudioWQ");
	if (!spec->workq) {
		snd_printk(KERN_ERR"%s: failed to create WQ\n", __func__);
		return -ENOMEM;
	}
	for (pin_idx = 0; pin_idx < spec->num_pins; pin_idx++) {
		struct hdmi_spec_per_pin *per_pin = get_pin(spec, pin_idx);

		snd_printd("per_ pin init %d\n", pin_idx);
		per_pin->codec = codec;
		mutex_init(&per_pin->lock);
		INIT_DELAYED_WORK(&per_pin->work, hdmi_repoll_eld);
		eld_proc_new(per_pin, pin_idx);
	}
	return 0;
}

static int intel_hdmi_init(struct snd_soc_hda_codec *codec)
{
	struct hda_codec *hc = &codec->hdac;
	struct hdmi_spec *spec = hc->spec;
	int pin_idx;

	snd_printd("In %s\n", __func__);

	for (pin_idx = 0; pin_idx < spec->num_pins; pin_idx++) {
		struct hdmi_spec_per_pin *per_pin = get_pin(spec, pin_idx);
		hda_nid_t pin_nid = per_pin->pin_nid;

		hdmi_init_pin(hc, pin_nid);
		snd_hda_jack_detect_enable(hc, pin_nid, pin_nid);
	}
	return 0;
}

static void intel_hdmi_free(struct snd_soc_hda_codec *codec)
{
	struct hda_codec *hc = &codec->hdac;
	struct hdmi_spec *spec = hc->spec;
	int pin_idx;

	for (pin_idx = 0; pin_idx < spec->num_pins; pin_idx++) {
		struct hdmi_spec_per_pin *per_pin = get_pin(spec, pin_idx);

		cancel_delayed_work(&per_pin->work);
		eld_proc_free(per_pin);
	}

	flush_workqueue(spec->workq);
	hdmi_array_free(spec);
	kfree(spec);
}

static struct snd_soc_hda_codec_ops intel_hdmi_patch_ops = {
	.build_controls		= intel_hdmi_build_controls,
	.init			= intel_hdmi_init,
	.free			= intel_hdmi_free,
};

static const struct hdmi_ops generic_standard_hdmi_ops = {
	.pin_get_eld				= snd_hdmi_get_eld,
	.pin_get_slot_channel			= hdmi_pin_get_slot_channel,
	.pin_set_slot_channel			= hdmi_pin_set_slot_channel,
	.pin_setup_infoframe			= hdmi_pin_setup_infoframe,
	.pin_hbr_setup				= hdmi_pin_hbr_setup,
	.setup_stream				= hdmi_setup_stream,
	.chmap_cea_alloc_validate_get_type	= hdmi_chmap_cea_alloc_validate_get_type,
	.cea_alloc_to_tlv_chmap			= hdmi_cea_alloc_to_tlv_chmap,
};

static int patch_intel_hdmi(struct hda_codec *codec)
{
	struct hdmi_spec *spec;
	int err = 0;

	snd_printd("In %s\n", __func__);

	spec = kzalloc(sizeof(*spec), GFP_KERNEL);
	if (spec == NULL)
		return -ENOMEM;

	codec->spec = spec;
	spec->ops = generic_standard_hdmi_ops;
	hdmi_array_init(spec, 4);

	if (hdmi_parse_codec(codec) < 0) {
		codec->spec = NULL;
		kfree(spec);
		return -EINVAL;
	}

	err = generic_hdmi_init_per_pins(codec);
	if (err < 0) {
		codec->spec = NULL;
		kfree(spec);
		return -EINVAL;
	}

	init_channel_allocations();

	return 0;
}

/*
 * called from hda_codec.c for generic HDMI support
 */
int snd_hda_parse_hdmi_codec(struct hda_codec *codec)
{
	return patch_intel_hdmi(codec);
}
EXPORT_SYMBOL_GPL(snd_hda_parse_hdmi_codec);

static int hdmi_codec_probe(struct snd_soc_codec *codec)
{
	struct snd_soc_hda_codec *hdac = snd_soc_codec_get_drvdata(codec);
	struct hdmi_patch *hdmi_drv_data;
	int ret;

	snd_printd("%s called\n", __func__);

	hdac->scodec = codec;

	/* Imp: Store the card pointer in hda_codec */
	hdac->hdac.card = codec->card->snd_card;

	hdmi_drv_data = (struct hdmi_patch *)soc_hda_get_device_id(hdac->hdev)->driver_data;
	hdmi_drv_data->patch(&hdac->hdac);

	ret = snd_soc_hda_codec_build_controls(hdac);
	if (ret)
		return ret;

	snd_hda_codec_proc_new(&hdac->hdac);

	/* FIXME: add codec controls here dynamically */
	/* dapm widgets and routes are added dynamically */

	pm_runtime_enable(&hdac->hdev->dev);
	pm_runtime_set_suspended(&hdac->hdev->dev);

	return 0;
}

static int hdmi_codec_remove(struct snd_soc_codec *codec)
{
	struct snd_soc_hda_codec *hdac = snd_soc_codec_get_drvdata(codec);

	pm_runtime_disable(&hdac->hdev->dev);
	return 0;
}

static struct snd_soc_dapm_widget hdmi_dapm_widgets[] = {
	SND_SOC_DAPM_OUTPUT("hif1 Output"),
};

static struct snd_soc_dapm_route hdmi_dapm_route[] = {
	{"hif1 Output", NULL, "hif1"},
};
static struct snd_soc_codec_driver hdmi_hda_codec = {
	.probe		= hdmi_codec_probe,
	.remove		= hdmi_codec_remove,
	.idle_bias_off	= true,
	.dapm_widgets = hdmi_dapm_widgets,
	.num_dapm_widgets = ARRAY_SIZE(hdmi_dapm_widgets),
	.dapm_routes = hdmi_dapm_route,
	.num_dapm_routes = ARRAY_SIZE(hdmi_dapm_route),

	/* dapm widgets and routes are added dynamically */
};

static struct snd_soc_dai_ops hdmi_dai_ops = {
	.startup = hdmi_pcm_open,
	.shutdown = hdmi_pcm_close,
	.hw_params = hdmi_set_hw_params,
	.prepare = hdmi_playback_prepare,
	.hw_free = hdmi_playback_cleanup,
};



static struct snd_soc_dai_driver hdmi_dais[] = {
	{	.name = "intel-hdmi-hif1",
		.playback = {
			.stream_name = "hif1",
			.channels_min = 2,
			.channels_max = 8,
			.rates = SNDRV_PCM_RATE_32000 |
				SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |
				SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE,

		},
		.ops = &hdmi_dai_ops,
	},

	{	.name = "intel-hdmi-hif2",
		.playback = {
			.stream_name = "hif2",
			.channels_min = 2,
			.channels_max = 8,
			.rates = SNDRV_PCM_RATE_32000 |
				SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000 |
				SNDRV_PCM_RATE_88200 | SNDRV_PCM_RATE_96000 |
				SNDRV_PCM_RATE_176400 | SNDRV_PCM_RATE_192000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S20_3LE |
				SNDRV_PCM_FMTBIT_S24_LE | SNDRV_PCM_FMTBIT_S32_LE,
		},
		.ops = &hdmi_dai_ops,
	},
};

static int intel_hdmi_dev_probe(struct snd_soc_hda_device *hdev)
{
	struct snd_soc_hda_codec *hdcodec;
	int ret;

	snd_printd("%s called\n", __func__);

	ret = snd_soc_hda_codec_new(hdev, hdev->addr, &hdcodec);
	if (ret)
		return -ENOMEM;

	hdcodec->ops = intel_hdmi_patch_ops;
	hdcodec->hdac.vendor_name = kstrdup("Intel", GFP_KERNEL);
	hdcodec->hdac.chip_name = kstrdup(soc_hda_get_device_id(hdev)->name, GFP_KERNEL);
	snd_printd("%s %s\n", hdcodec->hdac.vendor_name, hdcodec->hdac.chip_name);

	hdcodec->hdev = hdev;

	/* FIXME: Dynamic Dais here */

	snd_soc_hda_set_drvdata(hdev, hdcodec);

	/* ASoC specific initialization */
	ret = snd_soc_register_codec(&hdev->dev, &hdmi_hda_codec,
			hdmi_dais, ARRAY_SIZE(hdmi_dais));

	return ret;
}

static int intel_hdmi_dev_remove(struct snd_soc_hda_device *hdev)
{
	snd_soc_unregister_codec(&hdev->dev);
	return 0;
}

static void intel_hdmi_dev_shutdown(struct snd_soc_hda_device *hdev)
{
	/* FIXME: what needs to be done here */
}

static struct hdmi_patch cougarpoint_drv_data = {
	.patch = &patch_intel_hdmi,
};

static struct hdmi_patch pantherpoint_drv_data = {
	.patch = &patch_intel_hdmi,
};
static const struct snd_soc_hda_device_id hdmi_list[] = {
	{ .id = 0x80862805, .addr = 0, .name = "CougarPoint HDMI", (kernel_ulong_t)&cougarpoint_drv_data},
	{ .id = 0x80862806, .addr = 0, .name = "PantherPoint HDMI", (kernel_ulong_t)&pantherpoint_drv_data},
	{ .id = 0x8086280a, .addr = 0, .name = "CougarPoint HDMI", (kernel_ulong_t)&cougarpoint_drv_data},
	{ .id = 0x80862809, .addr = 0, .name = "Skylake HDMI", (kernel_ulong_t)&cougarpoint_drv_data},
	{},
};

#ifdef CONFIG_PM
static void hdmi_resume(struct snd_soc_hda_codec *codec)
{

	snd_printd("%s Enter\n", __func__);

	/* power up only AFG here */
	snd_hda_set_power_state(&codec->hdac, codec->hdac.afg, AC_PWRST_D0);

	intel_hdmi_init(codec);
	snd_hda_codec_resume_amp(&codec->hdac);
	snd_hda_codec_resume_cache(&codec->hdac);

	snd_hda_power_up(&codec->hdac);
}
#endif

static int hdmi_runtime_suspend(struct device *dev)
{
	struct snd_soc_hda_device *hdev = to_soc_hda_device(dev);
	struct snd_soc_hda_codec *codec = snd_soc_hda_get_drvdata(hdev);

	snd_printd("%s Enter\n", __func__);

	snd_hda_power_down(&codec->hdac);
	/* power down only AFG here */
	snd_hda_set_power_state(&codec->hdac, codec->hdac.afg, AC_PWRST_D3);
	return 0;
}

static int hdmi_runtime_resume(struct device *dev)
{
	struct snd_soc_hda_device *hdev = to_soc_hda_device(dev);
	struct snd_soc_hda_codec *codec = snd_soc_hda_get_drvdata(hdev);

	snd_printd("%s Enter\n", __func__);
	/* power up only AFG here */
	snd_hda_set_power_state(&codec->hdac, codec->hdac.afg, AC_PWRST_D0);

	snd_hda_codec_resume_amp(&codec->hdac);
	snd_hda_codec_resume_cache(&codec->hdac);

	snd_hda_power_up(&codec->hdac);
	return 0;
}

static int hdmi_runtime_idle(struct device *dev)
{
	snd_printd("%s Enter\n", __func__);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int hdmi_pm_suspend(struct device *dev)
{
	snd_printd("%s Enter\n", __func__);
	return 0;
}

static int hdmi_pm_resume(struct device  *dev)
{
	struct snd_soc_hda_device *hdev = to_soc_hda_device(dev);
	struct snd_soc_hda_codec *codec = snd_soc_hda_get_drvdata(hdev);

	snd_printd("%s Enter\n", __func__);
	hdmi_resume(codec);
	return 0;
}
#endif

static const struct dev_pm_ops hdmi_rtpm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(hdmi_pm_suspend, hdmi_pm_resume)
	SET_RUNTIME_PM_OPS(hdmi_runtime_suspend,
			hdmi_runtime_resume, hdmi_runtime_idle)
};

static struct snd_soc_hda_driver hdmi_hda_driver = {
	.driver = {
		.name   = "HDMI HDA Codec",
		.owner  = THIS_MODULE,
		.pm     = &hdmi_rtpm_ops,
	},
	.id_table       = hdmi_list,
	.probe          = intel_hdmi_dev_probe,
	.remove         = intel_hdmi_dev_remove,
	.shutdown       = intel_hdmi_dev_shutdown,
	.unsol_event	= intel_hdmi_unsol_event,
};

int __init hdmi_init(void)
{
	int ret;

	snd_printd("In %s\n", __func__);
	ret = snd_soc_hda_driver_register(&hdmi_hda_driver);
	return ret;
}

void __exit hdmi_exit(void)
{
	snd_printd("In %s\n", __func__);
	snd_soc_hda_driver_unregister(&hdmi_hda_driver);
}

module_init(hdmi_init);
module_exit(hdmi_exit);

MODULE_ALIAS("snd-hda-codec-id:80862*");
MODULE_ALIAS("snd-hda-codec-intelhdmi");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("HDMI HD codec");
