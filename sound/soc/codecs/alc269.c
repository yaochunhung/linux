/*
 *  alc269.c - ASoc HDA codec driver for Realtek ALC series
 *
 * Copyright (C) 2014 Intel Corp
 * Author: Lakshmi Vinnakota <lakshmi.n.vinnakota@intel.com>
 *
 * Copyright (c) 2004 Kailang Yang <kailang@realtek.com.tw>
 *                    PeiSen Hou <pshou@realtek.com.tw>
 *                    Takashi Iwai <tiwai@suse.de>
 *                    Jonathan Woithe <jwoithe@just42.net>
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
#include <linux/dmi.h>
#include <linux/pm_runtime.h>
#include <sound/core.h>
#include <sound/jack.h>
#include <sound/soc.h>
#include <sound/soc-dapm.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/hda_codec.h>
#include <sound/hda_auto_parser.h>
#include <sound/hda_controls.h>
#include <sound/hda_jack.h>
#include <sound/hda_generic.h>
#include <sound/hda_beep.h>
#include <sound/soc-hda-bus.h>
#include <sound/soc-hda-dma.h>
#include "soc-hda-codec.h"

/* unsol event tags */
#define ALC_DCVOL_EVENT		0x08

/* for GPIO Poll */
#define GPIO_MASK	0x03

/* extra amp-initialization sequence types */
enum {
	ALC_INIT_NONE,
	ALC_INIT_DEFAULT,
	ALC_INIT_GPIO1,
	ALC_INIT_GPIO2,
	ALC_INIT_GPIO3,
};

enum {
	ALC_HEADSET_MODE_UNKNOWN,
	ALC_HEADSET_MODE_UNPLUGGED,
	ALC_HEADSET_MODE_HEADSET,
	ALC_HEADSET_MODE_MIC,
	ALC_HEADSET_MODE_HEADPHONE,
};

enum {
	ALC_HEADSET_TYPE_UNKNOWN,
	ALC_HEADSET_TYPE_CTIA,
	ALC_HEADSET_TYPE_OMTP,
};

struct alc_customize_define {
	unsigned int  sku_cfg;
	unsigned char port_connectivity;
	unsigned char check_sum;
	unsigned char customization;
	unsigned char external_amp;
	unsigned int  enable_pcbeep:1;
	unsigned int  platform_type:1;
	unsigned int  swap:1;
	unsigned int  override:1;
	unsigned int  fixup:1; /* Means that this sku is set by driver, not read from hw */
};

struct alc_spec {
	struct hda_gen_spec gen; /* must be at head */

	/* codec parameterization */
	const struct snd_kcontrol_new *mixers[5];	/* mixer arrays */
	unsigned int num_mixers;
	unsigned int beep_amp;	/* beep amp value, set via set_beep_amp() */

	struct alc_customize_define cdefine;
	unsigned int parse_flags; /* flag for snd_hda_parse_pin_defcfg() */

	/* inverted dmic fix */
	unsigned int inv_dmic_fixup:1; /* has inverted digital-mic workaround */
	unsigned int inv_dmic_muted:1; /* R-ch of inv d-mic is muted? */
	hda_nid_t inv_dmic_pin;

	/* mute LED for HP laptops, see alc269_fixup_mic_mute_hook() */
	int mute_led_polarity;
	hda_nid_t mute_led_nid;

	unsigned int gpio_led; /* used for alc269_fixup_hp_gpio_led() */

	hda_nid_t headset_mic_pin;
	hda_nid_t headphone_mic_pin;
	int current_headset_mode;
	int current_headset_type;

	/* hooks */
	void (*init_hook)(struct snd_soc_hda_codec *codec);
#ifdef CONFIG_PM
	void (*power_hook)(struct snd_soc_hda_codec *codec);
#endif
	void (*shutup)(struct snd_soc_hda_codec *codec);

	int init_amp;
	int codec_variant;	/* flag for other variants */

	/* for PLL fix */
	hda_nid_t pll_nid;
	unsigned int pll_coef_idx, pll_coef_bit;
	unsigned int coef0;
};

struct alc_patch {
	int (*patch)(struct snd_soc_hda_codec *codec);
};

/*
 * Append the given mixer and verb elements for the later use
 * The mixer array is referred in build_controls(), and init_verbs are
 * called in init().
 */
static void add_mixer(struct alc_spec *spec, const struct snd_kcontrol_new *mix)
{
	if (snd_BUG_ON(spec->num_mixers >= ARRAY_SIZE(spec->mixers)))
		return;
	spec->mixers[spec->num_mixers++] = mix;
}

/* Debug functions */
static int write_amp_vals(struct snd_soc_hda_codec *shc, hda_nid_t nid,
		int ch, int dir, int index, int val)
{
	u32 parm;
	struct hda_codec *codec = &shc->hdac;

	parm = ch ? AC_AMP_SET_RIGHT : AC_AMP_SET_LEFT;
	parm |= dir == HDA_OUTPUT ? AC_AMP_SET_OUTPUT : AC_AMP_SET_INPUT;
	parm |= index << AC_AMP_SET_INDEX_SHIFT;
	parm |= val;
	snd_hda_codec_write(codec, nid, 0, AC_VERB_SET_AMP_GAIN_MUTE, parm);
	codec_dbg(codec, "amp value written %d\n", parm);
	return 0;
}

static int read_amp_vals(struct snd_soc_hda_codec *shc, hda_nid_t nid, int indices)
{
	int i, val;
	struct hda_codec *codec = &shc->hdac;

	for (i = 0; i < indices; i++) {
		codec_dbg(codec, "nid %d: [", nid);
		val = snd_hda_codec_read(codec, nid, 0,
					 AC_VERB_GET_AMP_GAIN_MUTE,
					 AC_AMP_GET_LEFT | HDA_INPUT | i);
		codec_dbg(codec, "0x%02x", val);
		val = snd_hda_codec_read(codec, 0xc, 0,
					 AC_VERB_GET_AMP_GAIN_MUTE,
					 AC_AMP_GET_RIGHT | HDA_INPUT | i);
		codec_dbg(codec, " 0x%02x] \n", val);
	}
	return 0;
}

/*
 * GPIO setup tables, used in initialization
 */
/* Enable GPIO mask and set output */
static const struct hda_verb alc_gpio1_init_verbs[] = {
	{0x01, AC_VERB_SET_GPIO_MASK, 0x01},
	{0x01, AC_VERB_SET_GPIO_DIRECTION, 0x01},
	{0x01, AC_VERB_SET_GPIO_DATA, 0x01},
	{ }
};

static const struct hda_verb alc_gpio2_init_verbs[] = {
	{0x01, AC_VERB_SET_GPIO_MASK, 0x02},
	{0x01, AC_VERB_SET_GPIO_DIRECTION, 0x02},
	{0x01, AC_VERB_SET_GPIO_DATA, 0x02},
	{ }
};

static const struct hda_verb alc_gpio3_init_verbs[] = {
	{0x01, AC_VERB_SET_GPIO_MASK, 0x03},
	{0x01, AC_VERB_SET_GPIO_DIRECTION, 0x03},
	{0x01, AC_VERB_SET_GPIO_DATA, 0x03},
	{ }
};

/*
 * Fix hardware PLL issue
 * On some codecs, the analog PLL gating control must be off while
 * the default value is 1.
 */
static void alc_fix_pll(struct hda_codec *codec)
{
	struct alc_spec *spec = codec->spec;
	unsigned int val;

	if (!spec->pll_nid)
		return;
	snd_hda_codec_write(codec, spec->pll_nid, 0, AC_VERB_SET_COEF_INDEX,
			    spec->pll_coef_idx);
	val = snd_hda_codec_read(codec, spec->pll_nid, 0,
				 AC_VERB_GET_PROC_COEF, 0);
	snd_hda_codec_write(codec, spec->pll_nid, 0, AC_VERB_SET_COEF_INDEX,
			    spec->pll_coef_idx);
	snd_hda_codec_write(codec, spec->pll_nid, 0, AC_VERB_SET_PROC_COEF,
			    val & ~(1 << spec->pll_coef_bit));
}

static void alc_fix_pll_init(struct hda_codec *codec, hda_nid_t nid,
			     unsigned int coef_idx, unsigned int coef_bit)
{
	struct alc_spec *spec = codec->spec;
	spec->pll_nid = nid;
	spec->pll_coef_idx = coef_idx;
	spec->pll_coef_bit = coef_bit;
	alc_fix_pll(codec);
}

/* update the master volume per volume-knob's unsol event */
static void alc_update_knob_master(struct hda_codec *codec, struct hda_jack_tbl *jack)
{
	unsigned int val;
	struct snd_kcontrol *kctl;
	struct snd_ctl_elem_value *uctl;

	kctl = snd_hda_find_mixer_ctl(codec, "Master Playback Volume");
	if (!kctl)
		return;
	uctl = kzalloc(sizeof(*uctl), GFP_KERNEL);
	if (!uctl)
		return;
	val = snd_hda_codec_read(codec, jack->nid, 0,
				 AC_VERB_GET_VOLUME_KNOB_CONTROL, 0);
	val &= HDA_AMP_VOLMASK;
	uctl->value.integer.value[0] = val;
	uctl->value.integer.value[1] = val;
	kctl->put(kctl, uctl);
	kfree(uctl);
}

/* turn on/off EAPD control (only if available) */
static void set_eapd(struct hda_codec *codec, hda_nid_t nid, int on)
{
	if (get_wcaps_type(get_wcaps(codec, nid)) != AC_WID_PIN)
		return;
	if (snd_hda_query_pin_caps(codec, nid) & AC_PINCAP_EAPD) {
		snd_hda_codec_write(codec, nid, 0, AC_VERB_SET_EAPD_BTLENABLE,
				    on ? 2 : 0);
		codec_dbg(codec, "In %s: EAPD control %d for nid %d\n",
					__func__, on, nid);
	}
}

static void alc_auto_setup_eapd(struct hda_codec *codec, bool on)
{
	/* We currently only handle front, HP */
	static hda_nid_t pins[] = {
		0x0f, 0x10, 0x14, 0x15, 0
	};
	hda_nid_t *p;
	for (p = pins; *p; p++)
		set_eapd(codec, *p, on);
}

/* generic EAPD initialization */
static void alc_auto_init_amp(struct hda_codec *codec, int type)
{
	unsigned int tmp;

	alc_auto_setup_eapd(codec, true);
	switch (type) {
	case ALC_INIT_GPIO1:
		snd_hda_sequence_write(codec, alc_gpio1_init_verbs);
		break;
	case ALC_INIT_GPIO2:
		snd_hda_sequence_write(codec, alc_gpio2_init_verbs);
		break;
	case ALC_INIT_GPIO3:
		snd_hda_sequence_write(codec, alc_gpio3_init_verbs);
		break;
	default:
		break;
	}
}

/* Could be any non-zero and even value. When used as fixup, tells
 * the driver to ignore any present sku defines.
 */
#define ALC_FIXUP_SKU_IGNORE (2)

static void alc_fixup_sku_ignore(struct hda_codec *codec,
				 const struct hda_fixup *fix, int action)
{
	struct alc_spec *spec = codec->spec;
	if (action == HDA_FIXUP_ACT_PRE_PROBE) {
		spec->cdefine.fixup = 1;
		spec->cdefine.sku_cfg = ALC_FIXUP_SKU_IGNORE;
	}
}

static int alc_auto_parse_customize_define(struct hda_codec *codec)
{
	unsigned int ass, tmp, i;
	unsigned nid = 0;
	struct alc_spec *spec = codec->spec;

	spec->cdefine.enable_pcbeep = 1; /* assume always enabled */

	if (spec->cdefine.fixup) {
		ass = spec->cdefine.sku_cfg;
		if (ass == ALC_FIXUP_SKU_IGNORE)
			return -1;
		goto do_sku;
	}

	ass = codec->subsystem_id & 0xffff;
#if 0
	if (ass != codec->bus->pci->subsystem_device && (ass & 1))
		goto do_sku;
#endif
	nid = 0x1d;
	if (codec->vendor_id == 0x10ec0260)
		nid = 0x17;
	ass = snd_hda_codec_get_pincfg(codec, nid);

	if (!(ass & 1)) {
		printk(KERN_INFO "snd_soc_hda_codec: %s: SKU not ready 0x%08x\n",
		       codec->chip_name, ass);
		return -1;
	}

	/* check sum */
	tmp = 0;
	for (i = 1; i < 16; i++) {
		if ((ass >> i) & 1)
			tmp++;
	}
	if (((ass >> 16) & 0xf) != tmp)
		return -1;

	spec->cdefine.port_connectivity = ass >> 30;
	spec->cdefine.enable_pcbeep = (ass & 0x100000) >> 20;
	spec->cdefine.check_sum = (ass >> 16) & 0xf;
	spec->cdefine.customization = ass >> 8;
do_sku:
	spec->cdefine.sku_cfg = ass;
	spec->cdefine.external_amp = (ass & 0x38) >> 3;
	spec->cdefine.platform_type = (ass & 0x4) >> 2;
	spec->cdefine.swap = (ass & 0x2) >> 1;
	spec->cdefine.override = ass & 0x1;

	codec_dbg(codec, "SKU: Nid=0x%x sku_cfg=0x%08x\n",
		   nid, spec->cdefine.sku_cfg);
	codec_dbg(codec, "SKU: port_connectivity=0x%x\n",
		   spec->cdefine.port_connectivity);
	codec_dbg(codec, "SKU: enable_pcbeep=0x%x\n", spec->cdefine.enable_pcbeep);
	codec_dbg(codec, "SKU: check_sum=0x%08x\n", spec->cdefine.check_sum);
	codec_dbg(codec, "SKU: customization=0x%08x\n", spec->cdefine.customization);
	codec_dbg(codec, "SKU: external_amp=0x%x\n", spec->cdefine.external_amp);
	codec_dbg(codec, "SKU: platform_type=0x%x\n", spec->cdefine.platform_type);
	codec_dbg(codec, "SKU: swap=0x%x\n", spec->cdefine.swap);
	codec_dbg(codec, "SKU: override=0x%x\n", spec->cdefine.override);

	return 0;
}

/* return the position of NID in the list, or -1 if not found */
static int find_idx_in_nid_list(hda_nid_t nid, const hda_nid_t *list, int nums)
{
	int i;
	for (i = 0; i < nums; i++)
		if (list[i] == nid)
			return i;
	return -1;
}
/* return true if the given NID is found in the list */
static bool found_in_nid_list(hda_nid_t nid, const hda_nid_t *list, int nums)
{
	return find_idx_in_nid_list(nid, list, nums) >= 0;
}

/* check subsystem ID and set up device-specific initialization;
 * return 1 if initialized, 0 if invalid SSID
 */
/* 32-bit subsystem ID for BIOS loading in HD Audio codec.
 *	31 ~ 16 :	Manufacture ID
 *	15 ~ 8	:	SKU ID
 *	7  ~ 0	:	Assembly ID
 *	port-A --> pin 39/41, port-E --> pin 14/15, port-D --> pin 35/36
 */
static int alc_subsystem_id(struct hda_codec *codec,
			    hda_nid_t porta, hda_nid_t porte,
			    hda_nid_t portd, hda_nid_t porti)
{
	unsigned int ass, tmp, i;
	unsigned nid;
	struct alc_spec *spec = codec->spec;

	if (spec->cdefine.fixup) {
		ass = spec->cdefine.sku_cfg;
		if (ass == ALC_FIXUP_SKU_IGNORE)
			return 0;
		goto do_sku;
	}

	ass = codec->subsystem_id & 0xffff;

	if (ass & 1)
		goto do_sku;

	/* invalid SSID, check the special NID pin defcfg instead */
	/*
	 * 31~30	: port connectivity
	 * 29~21	: reserve
	 * 20		: PCBEEP input
	 * 19~16	: Check sum (15:1)
	 * 15~1		: Custom
	 * 0		: override
	*/
	nid = 0x1d;
	if (codec->vendor_id == 0x10ec0260)
		nid = 0x17;
	ass = snd_hda_codec_get_pincfg(codec, nid);
	codec_dbg(codec, "realtek: No valid SSID, "
		   "checking pincfg 0x%08x for NID 0x%x\n",
		   ass, nid);
	if (!(ass & 1))
		return 0;
	if ((ass >> 30) != 1)	/* no physical connection */
		return 0;

	/* check sum */
	tmp = 0;
	for (i = 1; i < 16; i++) {
		if ((ass >> i) & 1)
			tmp++;
	}
	if (((ass >> 16) & 0xf) != tmp)
		return 0;
do_sku:
	codec_dbg(codec, "realtek: Enabling init ASM_ID=0x%04x CODEC_ID=%08x\n",
		   ass & 0xffff, codec->vendor_id);
	/*
	 * 0 : override
	 * 1 :	Swap Jack
	 * 2 : 0 --> Desktop, 1 --> Laptop
	 * 3~5 : External Amplifier control
	 * 7~6 : Reserved
	*/
	tmp = (ass & 0x38) >> 3;	/* external Amp control */
	switch (tmp) {
	case 1:
		spec->init_amp = ALC_INIT_GPIO1;
		break;
	case 3:
		spec->init_amp = ALC_INIT_GPIO2;
		break;
	case 7:
		spec->init_amp = ALC_INIT_GPIO3;
		break;
	case 5:
	default:
		spec->init_amp = ALC_INIT_DEFAULT;
		break;
	}

	/* is laptop or Desktop and enable the function "Mute internal speaker
	 * when the external headphone out jack is plugged"
	 */
	if (!(ass & 0x8000))
		return 1;
	/*
	 * 10~8 : Jack location
	 * 12~11: Headphone out -> 00: PortA, 01: PortE, 02: PortD, 03: Resvered
	 * 14~13: Resvered
	 * 15   : 1 --> enable the function "Mute internal speaker
	 *	        when the external headphone out jack is plugged"
	 */
	if (!spec->gen.autocfg.hp_pins[0] &&
	    !(spec->gen.autocfg.line_out_pins[0] &&
	      spec->gen.autocfg.line_out_type == AUTO_PIN_HP_OUT)) {
		hda_nid_t nid;
		tmp = (ass >> 11) & 0x3;	/* HP to chassis */
		if (tmp == 0)
			nid = porta;
		else if (tmp == 1)
			nid = porte;
		else if (tmp == 2)
			nid = portd;
		else if (tmp == 3)
			nid = porti;
		else
			return 1;
		if (found_in_nid_list(nid, spec->gen.autocfg.line_out_pins,
				      spec->gen.autocfg.line_outs))
			return 1;
		spec->gen.autocfg.hp_pins[0] = nid;
	}
	return 1;
}

/* Check the validity of ALC subsystem-id
 * ports contains an array of 4 pin NIDs for port-A, E, D and I */
static void alc_ssid_check(struct hda_codec *codec, const hda_nid_t *ports)
{
	if (!alc_subsystem_id(codec, ports[0], ports[1], ports[2], ports[3])) {
		struct alc_spec *spec = codec->spec;
		codec_dbg(codec, "realtek: "
			   "Enable default setup for auto mode as fallback\n");
		spec->init_amp = ALC_INIT_DEFAULT;
	}
}

/*
 * COEF access helper functions
 */
static int alc_read_coef_idx(struct hda_codec *codec,
			unsigned int coef_idx)
{
	unsigned int val;
	snd_hda_codec_write(codec, 0x20, 0, AC_VERB_SET_COEF_INDEX,
				coef_idx);
	val = snd_hda_codec_read(codec, 0x20, 0,
				AC_VERB_GET_PROC_COEF, 0);
	return val;
}

static void alc_write_coef_idx(struct hda_codec *codec, unsigned int coef_idx,
							unsigned int coef_val)
{
	snd_hda_codec_write(codec, 0x20, 0, AC_VERB_SET_COEF_INDEX,
			    coef_idx);
	snd_hda_codec_write(codec, 0x20, 0, AC_VERB_SET_PROC_COEF,
			    coef_val);
}

/* a special bypass for COEF 0; read the cached value at the second time */
static unsigned int alc_get_coef0(struct hda_codec *codec)
{
	struct alc_spec *spec = codec->spec;
	if (!spec->coef0)
		spec->coef0 = alc_read_coef_idx(codec, 0);
	return spec->coef0;
}
/*
 * Common callbacks
 */

#ifdef CONFIG_SND_HDA_INPUT_BEEP
#define set_beep_amp(spec, nid, idx, dir) \
	((spec)->beep_amp = HDA_COMPOSE_AMP_VAL(nid, 3, idx, dir))

static const struct snd_pci_quirk beep_white_list[] = {
	SND_PCI_QUIRK(0x8086, 0xd613, "Intel", 1),
	{}
};

static inline int has_cdefine_beep(struct hda_codec *codec)
{
	struct alc_spec *spec = codec->spec;
	const struct snd_pci_quirk *q;
	q = codec->quirk_lookup(codec, beep_white_list);
	if (q)
		return q->value;
	return spec->cdefine.enable_pcbeep;
}

/* additional beep mixers; the actual parameters are overwritten at build */
static const struct snd_kcontrol_new alc_beep_mixer[] = {
	HDA_CODEC_VOLUME("Beep Playback Volume", 0, 0, HDA_INPUT),
	HDA_CODEC_MUTE_BEEP("Beep Playback Switch", 0, 0, HDA_INPUT),
	{ } /* end */
};
#else
#define set_beep_amp(spec, nid, idx, dir) /* NOP */
#define has_cdefine_beep(codec)		0
#endif

static int alc_build_controls(struct snd_soc_hda_codec *shc)
{
	struct hda_codec *codec = &shc->hdac;
	struct alc_spec *spec = codec->spec;
	int i, err;

	err = snd_hda_gen_build_controls(codec);
	if (err < 0)
		return err;

	for (i = 0; i < spec->num_mixers; i++) {
		err = snd_hda_add_new_ctls(codec, spec->mixers[i]);
		if (err < 0)
			return err;
	}

#ifdef CONFIG_SND_HDA_INPUT_BEEP
	/* create beep controls if needed */
	if (spec->beep_amp) {
		const struct snd_kcontrol_new *knew;
		for (knew = alc_beep_mixer; knew->name; knew++) {
			struct snd_kcontrol *kctl;
			kctl = snd_ctl_new1(knew, codec);
			if (!kctl)
				return -ENOMEM;
			kctl->private_value = spec->beep_amp;
			err = snd_hda_ctl_add(codec, 0, kctl);
			if (err < 0)
				return err;
		}
	}
#endif
	snd_hda_apply_fixup(codec, HDA_FIXUP_ACT_BUILD);
	return 0;
}

static int alc_init(struct snd_soc_hda_codec *shc)
{
	struct hda_codec *codec = &shc->hdac;
	struct alc_spec *spec = codec->spec;

	if (spec->init_hook)
		spec->init_hook(shc);

	alc_fix_pll(codec);
	alc_auto_init_amp(codec, spec->init_amp);

	snd_hda_gen_init(codec);

	snd_hda_apply_fixup(codec, HDA_FIXUP_ACT_INIT);

	return 0;
}

static void alc_free(struct snd_soc_hda_codec *codec)
{
	snd_hda_gen_free(&codec->hdac);
}

/*
 */
static struct snd_soc_hda_codec_ops alc_codec_ops = {
	.build_controls = alc_build_controls,
	.init = alc_init,
	.free = alc_free,
};

/* parse the BIOS configuration and set up the alc_spec */
/* return 1 if successful, 0 if the proper config is not found,
 * or a negative error code
 */
static int alc_parse_auto_config(struct hda_codec *codec,
				 const hda_nid_t *ignore_nids,
				 const hda_nid_t *ssid_nids)
{
	struct alc_spec *spec = codec->spec;
	struct auto_pin_cfg *cfg = &spec->gen.autocfg;
	int err;

	err = snd_hda_parse_pin_defcfg(codec, cfg, ignore_nids,
				       spec->parse_flags);
	if (err < 0)
		return err;

	if (ssid_nids)
		alc_ssid_check(codec, ssid_nids);

	err = snd_hda_gen_parse_auto_config(codec, cfg);
	if (err < 0)
		return err;

	return 1;
}

/* common preparation job for alc_spec */
static int alc_alloc_spec(struct hda_codec *codec, hda_nid_t mixer_nid)
{
	struct alc_spec *spec = kzalloc(sizeof(*spec), GFP_KERNEL);

	if (!spec)
		return -ENOMEM;
	codec->spec = spec;

	snd_hda_gen_spec_init(&spec->gen);
	spec->gen.mixer_nid = mixer_nid;
	spec->gen.own_eapd_ctl = 1;
	codec->single_adc_amp = 1;
	return 0;
}


/*
 * ALC269
 */

/* different alc269-variants */
enum {
	ALC269_TYPE_ALC269VA,
	ALC269_TYPE_ALC269VB,
	ALC269_TYPE_ALC269VC,
	ALC269_TYPE_ALC269VD,
	ALC269_TYPE_ALC280,
	ALC269_TYPE_ALC282,
	ALC269_TYPE_ALC284,
	ALC269_TYPE_ALC286,
};

/*
 * BIOS auto configuration
 */
static int alc269_parse_auto_config(struct snd_soc_hda_codec *codec)
{
	static const hda_nid_t alc269_ignore[] = { 0x1d, 0 };
	static const hda_nid_t alc269_ssids[] = { 0, 0x1b, 0x14, 0x21 };
	static const hda_nid_t alc269va_ssids[] = { 0x15, 0x1b, 0x14, 0 };
	struct alc_spec *spec = codec->hdac.spec;
	const hda_nid_t *ssids;

	switch (spec->codec_variant) {
	case ALC269_TYPE_ALC269VA:
	case ALC269_TYPE_ALC269VC:
	case ALC269_TYPE_ALC280:
	case ALC269_TYPE_ALC284:
		ssids = alc269va_ssids;
		break;
	case ALC269_TYPE_ALC269VB:
	case ALC269_TYPE_ALC269VD:
	case ALC269_TYPE_ALC282:
	case ALC269_TYPE_ALC286:
		ssids = alc269_ssids;
		break;
	default:
		ssids = alc269_ssids;
		break;
	}

	return alc_parse_auto_config(&codec->hdac, alc269_ignore, ssids);
}

static void alc269vb_toggle_power_output(struct hda_codec *codec, int power_up)
{
	int val = alc_read_coef_idx(codec, 0x04);
	if (power_up)
		val |= 1 << 11;
	else
		val &= ~(1 << 11);
	alc_write_coef_idx(codec, 0x04, val);
}

static void alc269_shutup(struct snd_soc_hda_codec *codec)
{
	struct alc_spec *spec = codec->hdac.spec;

	if (spec->codec_variant != ALC269_TYPE_ALC269VB)
		return;

	if (spec->codec_variant == ALC269_TYPE_ALC269VB)
		alc269vb_toggle_power_output(&codec->hdac, 0);
	if (spec->codec_variant == ALC269_TYPE_ALC269VB &&
			(alc_get_coef0(&codec->hdac) & 0x00ff) == 0x018) {
		msleep(150);
	}
	snd_hda_shutup_pins(codec);
}

#ifdef CONFIG_PM
static int alc269_resume(struct snd_soc_hda_codec *shc)
{
	struct hda_codec *codec = &shc->hdac;
	struct alc_spec *spec = codec->spec;

	if (spec->codec_variant == ALC269_TYPE_ALC269VB)
		alc269vb_toggle_power_output(codec, 0);
	if (spec->codec_variant == ALC269_TYPE_ALC269VB &&
			(alc_get_coef0(codec) & 0x00ff) == 0x018) {
		msleep(150);
	}

	shc->ops.init(shc);

	if (spec->codec_variant == ALC269_TYPE_ALC269VB)
		alc269vb_toggle_power_output(codec, 1);
	if (spec->codec_variant == ALC269_TYPE_ALC269VB &&
			(alc_get_coef0(codec) & 0x00ff) == 0x017) {
		msleep(200);
	}

	snd_hda_codec_resume_amp(codec);
	snd_hda_codec_resume_cache(codec);
	snd_hda_gen_check_power_status(codec, 0x01);
	return 0;
}
#endif /* CONFIG_PM */

static void alc269_fixup_pincfg_no_hp_to_lineout(struct hda_codec *codec,
						 const struct hda_fixup *fix, int action)
{
	struct alc_spec *spec = codec->spec;

	if (action == HDA_FIXUP_ACT_PRE_PROBE)
		spec->parse_flags = HDA_PINCFG_NO_HP_FIXUP;
}

static void alc269_fixup_hweq(struct hda_codec *codec,
			       const struct hda_fixup *fix, int action)
{
	int coef;

	if (action != HDA_FIXUP_ACT_INIT)
		return;
	coef = alc_read_coef_idx(codec, 0x1e);
	alc_write_coef_idx(codec, 0x1e, coef | 0x80);
}

static void alc269_fixup_headset_mic(struct hda_codec *codec,
				       const struct hda_fixup *fix, int action)
{
	struct alc_spec *spec = codec->spec;
	if (action == HDA_FIXUP_ACT_PRE_PROBE)
		spec->parse_flags |= HDA_PINCFG_HEADSET_MIC;
}

static void alc271_fixup_dmic(struct hda_codec *codec,
			      const struct hda_fixup *fix, int action)
{
	static const struct hda_verb verbs[] = {
		{0x20, AC_VERB_SET_COEF_INDEX, 0x0d},
		{0x20, AC_VERB_SET_PROC_COEF, 0x4000},
		{}
	};
	unsigned int cfg;

	if (strcmp(codec->chip_name, "ALC271X") &&
	    strcmp(codec->chip_name, "ALC269VB"))
		return;
	cfg = snd_hda_codec_get_pincfg(codec, 0x12);
	if (get_defcfg_connect(cfg) == AC_JACK_PORT_FIXED)
		snd_hda_sequence_write(codec, verbs);
}

static void alc269_fixup_stereo_dmic(struct hda_codec *codec,
				     const struct hda_fixup *fix, int action)
{
	int coef;

	if (action != HDA_FIXUP_ACT_INIT)
		return;
	/* The digital-mic unit sends PDM (differential signal) instead of
	 * the standard PCM, thus you can't record a valid mono stream as is.
	 * Below is a workaround specific to ALC269 to control the dmic
	 * signal source as mono.
	 */
	coef = alc_read_coef_idx(codec, 0x07);
	alc_write_coef_idx(codec, 0x07, coef | 0x80);
}

static void alc269_quanta_automute(struct hda_codec *codec)
{
	snd_hda_gen_update_outputs(codec);

	snd_hda_codec_write(codec, 0x20, 0,
			AC_VERB_SET_COEF_INDEX, 0x0c);
	snd_hda_codec_write(codec, 0x20, 0,
			AC_VERB_SET_PROC_COEF, 0x680);

	snd_hda_codec_write(codec, 0x20, 0,
			AC_VERB_SET_COEF_INDEX, 0x0c);
	snd_hda_codec_write(codec, 0x20, 0,
			AC_VERB_SET_PROC_COEF, 0x480);
}

static void alc269_fixup_quanta_mute(struct hda_codec *codec,
				     const struct hda_fixup *fix, int action)
{
	struct alc_spec *spec = codec->spec;
	if (action != HDA_FIXUP_ACT_PROBE)
		return;
	spec->gen.automute_hook = alc269_quanta_automute;
}

static void alc269_x101_hp_automute_hook(struct hda_codec *codec,
					 struct hda_jack_tbl *jack)
{
	struct alc_spec *spec = codec->spec;
	int vref;
	msleep(200);
	snd_hda_gen_hp_automute(codec, jack);

	vref = spec->gen.hp_jack_present ? PIN_VREF80 : 0;
	msleep(100);
	snd_hda_codec_write(codec, 0x18, 0, AC_VERB_SET_PIN_WIDGET_CONTROL,
			    vref);
	msleep(500);
	snd_hda_codec_write(codec, 0x18, 0, AC_VERB_SET_PIN_WIDGET_CONTROL,
			    vref);
}

static void alc269_fixup_x101_headset_mic(struct hda_codec *codec,
				     const struct hda_fixup *fix, int action)
{
	struct alc_spec *spec = codec->spec;
	if (action == HDA_FIXUP_ACT_PRE_PROBE) {
		spec->parse_flags |= HDA_PINCFG_HEADSET_MIC;
		spec->gen.hp_automute_hook = alc269_x101_hp_automute_hook;
	}
}


/* update mute-LED according to the speaker mute state via mic VREF pin */
static void alc269_fixup_mic_mute_hook(void *private_data, int enabled)
{
	struct snd_soc_hda_codec *codec = private_data;
	struct alc_spec *spec = codec->hdac.spec;
	unsigned int pinval;

	if (spec->mute_led_polarity)
		enabled = !enabled;
	pinval = AC_PINCTL_IN_EN |
		(enabled ? AC_PINCTL_VREF_HIZ : AC_PINCTL_VREF_80);
	if (spec->mute_led_nid)
		snd_hda_set_pin_ctl_cache(&codec->hdac, spec->mute_led_nid, pinval);
}

static void alc269_fixup_hp_mute_led(struct hda_codec *codec,
				     const struct hda_fixup *fix, int action)
{
	struct alc_spec *spec = codec->spec;
	const struct dmi_device *dev = NULL;

	if (action != HDA_FIXUP_ACT_PRE_PROBE)
		return;

	while ((dev = dmi_find_device(DMI_DEV_TYPE_OEM_STRING, NULL, dev))) {
		int pol, pin;
		if (sscanf(dev->name, "HP_Mute_LED_%d_%x", &pol, &pin) != 2)
			continue;
		if (pin < 0x0a || pin >= 0x10)
			break;
		spec->mute_led_polarity = pol;
		spec->mute_led_nid = pin - 0x0a + 0x18;
		spec->gen.vmaster_mute.hook = alc269_fixup_mic_mute_hook;
		spec->gen.vmaster_mute_enum = 1;
		codec_dbg(codec, "Detected mute LED for %x:%d\n", spec->mute_led_nid,
			   spec->mute_led_polarity);
		break;
	}
}

static void alc269_fixup_hp_mute_led_mic1(struct hda_codec *codec,
				const struct hda_fixup *fix, int action)
{
	struct alc_spec *spec = codec->spec;
	if (action == HDA_FIXUP_ACT_PRE_PROBE) {
		spec->mute_led_polarity = 0;
		spec->mute_led_nid = 0x18;
		spec->gen.vmaster_mute.hook = alc269_fixup_mic_mute_hook;
		spec->gen.vmaster_mute_enum = 1;
	}
}

static void alc269_fixup_hp_mute_led_mic2(struct hda_codec *codec,
				const struct hda_fixup *fix, int action)
{
	struct alc_spec *spec = codec->spec;
	if (action == HDA_FIXUP_ACT_PRE_PROBE) {
		spec->mute_led_polarity = 0;
		spec->mute_led_nid = 0x19;
		spec->gen.vmaster_mute.hook = alc269_fixup_mic_mute_hook;
		spec->gen.vmaster_mute_enum = 1;
	}
}

/* turn on/off mute LED per vmaster hook */
static void alc269_fixup_hp_gpio_mute_hook(void *private_data, int enabled)
{
	struct snd_soc_hda_codec *codec = private_data;
	struct alc_spec *spec = codec->hdac.spec;
	unsigned int oldval = spec->gpio_led;

	if (enabled)
		spec->gpio_led &= ~0x08;
	else
		spec->gpio_led |= 0x08;
	if (spec->gpio_led != oldval)
		snd_hda_codec_write(&codec->hdac, 0x01, 0, AC_VERB_SET_GPIO_DATA,
				    spec->gpio_led);
}

/* turn on/off mic-mute LED per capture hook */
static void alc269_fixup_hp_gpio_mic_mute_hook(struct hda_codec *codec,
			       struct snd_ctl_elem_value *ucontrol)
{
	struct alc_spec *spec = codec->spec;
	unsigned int oldval = spec->gpio_led;

	if (!ucontrol)
		return;

	if (ucontrol->value.integer.value[0] ||
	    ucontrol->value.integer.value[1])
		spec->gpio_led &= ~0x10;
	else
		spec->gpio_led |= 0x10;
	if (spec->gpio_led != oldval)
		snd_hda_codec_write(codec, 0x01, 0, AC_VERB_SET_GPIO_DATA,
				    spec->gpio_led);
}

static void alc269_fixup_hp_gpio_led(struct hda_codec *codec,
				const struct hda_fixup *fix, int action)
{
	struct alc_spec *spec = codec->spec;
	static const struct hda_verb gpio_init[] = {
		{ 0x01, AC_VERB_SET_GPIO_MASK, 0x18 },
		{ 0x01, AC_VERB_SET_GPIO_DIRECTION, 0x18 },
		{}
	};

	if (action == HDA_FIXUP_ACT_PRE_PROBE) {
		spec->gen.vmaster_mute.hook = alc269_fixup_hp_gpio_mute_hook;
		spec->gen.cap_sync_hook = alc269_fixup_hp_gpio_mic_mute_hook;
		spec->gpio_led = 0;
		snd_hda_add_verbs(codec, gpio_init);
	}
}

enum {
	ALC269_FIXUP_SONY_VAIO,
	ALC275_FIXUP_SONY_VAIO_GPIO2,
	ALC269_FIXUP_DELL_M101Z,
	ALC269_FIXUP_SKU_IGNORE,
	ALC269_FIXUP_ASUS_G73JW,
	ALC269_FIXUP_LENOVO_EAPD,
	ALC275_FIXUP_SONY_HWEQ,
	ALC271_FIXUP_DMIC,
	ALC269_FIXUP_PCM_44K,
	ALC269_FIXUP_STEREO_DMIC,
	ALC269_FIXUP_HEADSET_MIC,
	ALC269_FIXUP_QUANTA_MUTE,
	ALC269_FIXUP_LIFEBOOK,
	ALC269_FIXUP_AMIC,
	ALC269_FIXUP_DMIC,
	ALC269VB_FIXUP_AMIC,
	ALC269VB_FIXUP_DMIC,
	ALC269_FIXUP_HP_MUTE_LED,
	ALC269_FIXUP_HP_MUTE_LED_MIC1,
	ALC269_FIXUP_HP_MUTE_LED_MIC2,
	ALC269_FIXUP_HP_GPIO_LED,
	ALC269_FIXUP_INV_DMIC,
	ALC269_FIXUP_LENOVO_DOCK,
	ALC286_FIXUP_SONY_MIC_NO_PRESENCE,
	ALC269_FIXUP_PINCFG_NO_HP_TO_LINEOUT,
	ALC269_FIXUP_DELL1_MIC_NO_PRESENCE,
	ALC269_FIXUP_DELL2_MIC_NO_PRESENCE,
	ALC269_FIXUP_HEADSET_MODE,
	ALC269_FIXUP_HEADSET_MODE_NO_HP_MIC,
	ALC269_FIXUP_ASUS_X101_FUNC,
	ALC269_FIXUP_ASUS_X101_VERB,
	ALC269_FIXUP_ASUS_X101,
	ALC271_FIXUP_AMIC_MIC2,
	ALC271_FIXUP_HP_GATE_MIC_JACK,
	ALC269_FIXUP_ACER_AC700,
	ALC269_FIXUP_LIMIT_INT_MIC_BOOST,
};

static const struct hda_fixup alc269_fixups[] = {
	[ALC269_FIXUP_SONY_VAIO] = {
		.type = HDA_FIXUP_PINCTLS,
		.v.pins = (const struct hda_pintbl[]) {
			{0x19, PIN_VREFGRD},
			{}
		}
	},
	[ALC275_FIXUP_SONY_VAIO_GPIO2] = {
		.type = HDA_FIXUP_VERBS,
		.v.verbs = (const struct hda_verb[]) {
			{0x01, AC_VERB_SET_GPIO_MASK, 0x04},
			{0x01, AC_VERB_SET_GPIO_DIRECTION, 0x04},
			{0x01, AC_VERB_SET_GPIO_DATA, 0x00},
			{ }
		},
		.chained = true,
		.chain_id = ALC269_FIXUP_SONY_VAIO
	},
	[ALC269_FIXUP_SKU_IGNORE] = {
		.type = HDA_FIXUP_FUNC,
		.v.func = alc_fixup_sku_ignore,
	},
	[ALC275_FIXUP_SONY_HWEQ] = {
		.type = HDA_FIXUP_FUNC,
		.v.func = alc269_fixup_hweq,
		.chained = true,
		.chain_id = ALC275_FIXUP_SONY_VAIO_GPIO2
	},
	[ALC269_FIXUP_HEADSET_MIC] = {
		.type = HDA_FIXUP_FUNC,
		.v.func = alc269_fixup_headset_mic,
	},
	[ALC286_FIXUP_SONY_MIC_NO_PRESENCE] = {
		.type = HDA_FIXUP_PINS,
		.v.pins = (const struct hda_pintbl[]) {
			{ 0x18, 0x01a1913c }, /* use as headset mic, without its own jack detect */
			{ }
		},
		.chained = true,
		.chain_id = ALC269_FIXUP_HEADSET_MIC
	},
};

static const struct snd_pci_quirk alc269_fixup_tbl[] = {
	SND_PCI_QUIRK(0x104d, 0x90b5, "Sony VAIO Pro 11", ALC286_FIXUP_SONY_MIC_NO_PRESENCE),
	SND_PCI_QUIRK(0x104d, 0x90b6, "Sony VAIO Pro 13", ALC286_FIXUP_SONY_MIC_NO_PRESENCE),
	SND_PCI_QUIRK(0x104d, 0x9073, "Sony VAIO", ALC275_FIXUP_SONY_VAIO_GPIO2),
	SND_PCI_QUIRK(0x104d, 0x907b, "Sony VAIO", ALC275_FIXUP_SONY_HWEQ),
	SND_PCI_QUIRK(0x104d, 0x9084, "Sony VAIO", ALC275_FIXUP_SONY_HWEQ),
	SND_PCI_QUIRK_VENDOR(0x104d, "Sony VAIO", ALC269_FIXUP_SONY_VAIO),
	{}
};

static const struct hda_model_fixup alc269_fixup_models[] = {
	{.id = ALC269_FIXUP_AMIC, .name = "laptop-amic"},
	{.id = ALC269_FIXUP_DMIC, .name = "laptop-dmic"},
	{.id = ALC269_FIXUP_STEREO_DMIC, .name = "alc269-dmic"},
	{.id = ALC271_FIXUP_DMIC, .name = "alc271-dmic"},
	{.id = ALC269_FIXUP_INV_DMIC, .name = "inv-dmic"},
	{.id = ALC269_FIXUP_LENOVO_DOCK, .name = "lenovo-dock"},
	{.id = ALC269_FIXUP_HP_GPIO_LED, .name = "hp-gpio-led"},
	{.id = ALC269_FIXUP_DELL1_MIC_NO_PRESENCE, .name = "dell-headset-multi"},
	{.id = ALC269_FIXUP_DELL2_MIC_NO_PRESENCE, .name = "dell-headset-dock"},
	{}
};


static void alc269_fill_coef(struct snd_soc_hda_codec *codec)
{
	struct hda_codec *hc = &codec->hdac;
	struct alc_spec *spec = hc->spec;
	int val;

	if (spec->codec_variant != ALC269_TYPE_ALC269VB)
		return;

	if ((alc_get_coef0(hc) & 0x00ff) < 0x015) {
		alc_write_coef_idx(hc, 0xf, 0x960b);
		alc_write_coef_idx(hc, 0xe, 0x8817);
	}

	if ((alc_get_coef0(hc) & 0x00ff) == 0x016) {
		alc_write_coef_idx(hc, 0xf, 0x960b);
		alc_write_coef_idx(hc, 0xe, 0x8814);
	}

	if ((alc_get_coef0(hc) & 0x00ff) == 0x017) {
		val = alc_read_coef_idx(hc, 0x04);
		/* Power up output pin */
		alc_write_coef_idx(hc, 0x04, val | (1<<11));
	}

	if ((alc_get_coef0(hc) & 0x00ff) == 0x018) {
		val = alc_read_coef_idx(hc, 0xd);
		if ((val & 0x0c00) >> 10 != 0x1) {
			/* Capless ramp up clock control */
			alc_write_coef_idx(hc, 0xd, val | (1<<10));
		}
		val = alc_read_coef_idx(hc, 0x17);
		if ((val & 0x01c0) >> 6 != 0x4) {
			/* Class D power on reset */
			alc_write_coef_idx(hc, 0x17, val | (1<<7));
		}
	}

	val = alc_read_coef_idx(hc, 0xd); /* Class D */
	alc_write_coef_idx(hc, 0xd, val | (1<<14));

	val = alc_read_coef_idx(hc, 0x4); /* HP */
	alc_write_coef_idx(hc, 0x4, val | (1<<11));
}


static int patch_alc269(struct snd_soc_hda_codec *codec)
{
	struct alc_spec *spec;
	int err;
	struct hda_codec *hc = &codec->hdac;

	err = alc_alloc_spec(hc, 0x0b);
	if (err < 0)
		return err;

	spec = hc->spec;

	spec->gen.shared_mic_vref_pin = 0x18;


	alc_auto_parse_customize_define(hc);

	if (has_cdefine_beep(hc))
		spec->gen.beep_nid = 0x01;

	switch (hc->vendor_id) {
	case 0x10ec0269:
		spec->codec_variant = ALC269_TYPE_ALC269VA;
		switch (alc_get_coef0(hc) & 0x00f0) {
#if 0
		case 0x0010:
			if (codec->bus->pci->subsystem_vendor == 0x1025 &&
			    spec->cdefine.platform_type == 1)
				err = alc_codec_rename(codec, "ALC271X");
			spec->codec_variant = ALC269_TYPE_ALC269VB;
			break;
		case 0x0020:
			if (codec->bus->pci->subsystem_vendor == 0x17aa &&
			    codec->bus->pci->subsystem_device == 0x21f3)
				err = alc_codec_rename(codec, "ALC3202");
			spec->codec_variant = ALC269_TYPE_ALC269VC;
			break;
#endif
		case 0x0030:
			spec->codec_variant = ALC269_TYPE_ALC269VD;
			break;
		default:
			alc_fix_pll_init(hc, 0x20, 0x04, 15);
		}
		if (err < 0)
			goto error;
		spec->init_hook = alc269_fill_coef;
		alc269_fill_coef(codec);
		break;
	case 0x10ec0286:
		spec->codec_variant = ALC269_TYPE_ALC286;
		break;
	}

	/* automatic parse from the BIOS config */
	err = alc269_parse_auto_config(codec);
	if (err < 0)
		goto error;

	if (!spec->gen.no_analog && spec->gen.beep_nid)
		set_beep_amp(spec, 0x0b, 0x04, HDA_INPUT);

	/* FIXME: how to set this variant in driver ops */
#if 0
#ifdef CONFIG_PM
	codec->ops.resume = alc269_resume;
#endif
#endif
	if (!spec->shutup)
		spec->shutup = alc269_shutup;

	snd_hda_apply_fixup(hc, HDA_FIXUP_ACT_PROBE);

	return 0;

 error:
	alc_free(codec);
	return err;
}

static int power_ev(struct snd_soc_dapm_widget *w,
		  struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct snd_soc_hda_codec *hdac = snd_soc_codec_get_drvdata(codec);
	hda_nid_t nid = w->shift;

	if (SND_SOC_DAPM_EVENT_ON(event))
		snd_hda_set_power_state(&hdac->hdac, nid, AC_PWRST_D0);
	else
		snd_hda_set_power_state(&hdac->hdac, nid, AC_PWRST_D3);

	return 0;
}

static const struct snd_soc_dapm_widget alc_cvt_widgets[] = {
SND_SOC_DAPM_DAC_E("SPKDAC", NULL, SND_SOC_NOPM, 3, 0,
	power_ev, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_DAC_E("HPDAC", NULL, SND_SOC_NOPM, 2, 0,
	power_ev, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
SND_SOC_DAPM_ADC_E("ADC1", NULL, SND_SOC_NOPM, 0x11, 0,
	power_ev, SND_SOC_DAPM_PRE_PMU | SND_SOC_DAPM_POST_PMD),
};

void pin_amp_event(struct snd_soc_hda_codec *hdac, hda_nid_t nid,
				int event)
{
	if (SND_SOC_DAPM_EVENT_ON(event)) {
		/*Enable EAPD */
		snd_hda_set_power_state(&hdac->hdac, nid, AC_PWRST_D0);
		set_eapd(&hdac->hdac, nid, true);
		msleep(200);
	} else {
		/*Disable EAPD */
		set_eapd(&hdac->hdac, nid, false);
		msleep(200);
		snd_hda_set_power_state(&hdac->hdac, nid, AC_PWRST_D3);
	}

}

int hp_amp_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *k, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct snd_soc_hda_codec *hdac = snd_soc_codec_get_drvdata(codec);
	struct alc_spec *spec = hdac->hdac.spec;
	hda_nid_t nid = spec->gen.autocfg.hp_pins[0];

	pin_amp_event(hdac, nid, event);
	return 0;
}

int spk_amp_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *k, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct snd_soc_hda_codec *hdac = snd_soc_codec_get_drvdata(codec);
	struct alc_spec *spec = hdac->hdac.spec;
	hda_nid_t nid = spec->gen.autocfg.speaker_pins[0];

	pin_amp_event(hdac, nid, event);
	return 0;
}

static int mic_ev(struct snd_soc_dapm_widget *w,
		  struct snd_kcontrol *kcontrol, int event)
{
	struct snd_soc_codec *codec = w->codec;
	struct snd_soc_hda_codec *hdac = snd_soc_codec_get_drvdata(codec);
	struct alc_spec *spec = hdac->hdac.spec;
	hda_nid_t nid = 0x12; /*FIXME: how to get mic pin nid */

	if (SND_SOC_DAPM_EVENT_ON(event))
		snd_hda_set_power_state(&hdac->hdac, nid, AC_PWRST_D0);
	else
		snd_hda_set_power_state(&hdac->hdac, nid, AC_PWRST_D3);

	return 0;
}

static const struct snd_soc_dapm_widget alc_dapm_muxers[] = {
	SND_SOC_DAPM_PGA("Mux0x22", SND_SOC_NOPM, 0x22, 0, NULL, 0),
};

static const struct snd_soc_dapm_widget alc_dapm_mixers[] = {
	SND_SOC_DAPM_PGA("Mixer0xc", SND_SOC_NOPM, 0xc, 0, NULL, 0),
	SND_SOC_DAPM_PGA("Mixer0xd", SND_SOC_NOPM, 0xd, 0, NULL, 0),
};

static const struct snd_soc_dapm_widget alc_pin_widgets[] = {
	SND_SOC_DAPM_HP("HPOUT", hp_amp_event),
	SND_SOC_DAPM_SPK("SPKOUT", spk_amp_event),
	SND_SOC_DAPM_MIC("Mic", mic_ev),
};

static const struct snd_soc_dapm_route alc_intercon[] = {
	{"HPOUT", NULL, "Headphone"},
	{"SPKOUT", NULL, "Speaker"},
	{"Record", NULL, "Mic"},
/* Headphone path */
	{"Mixer0xc", NULL, "HPOUT"},
	{"HPDAC", NULL, "Mixer0xc"},
/* Speaker path */
	{"Mixer0xd", NULL, "SPKOUT"},
	{"SPKDAC", NULL, "Mixer0xd"},
/* capture path */
	{"Mic", NULL, "Mux0x22"},
	{"Mux0x22", NULL, "ADC1"},
};
/* codec registration */
static int alc_codec_probe(struct snd_soc_codec *codec)
{
	struct snd_soc_hda_codec *hdac = snd_soc_codec_get_drvdata(codec);
	struct alc_patch *alc_drv_data;
	int ret, i, val;

	hdac->scodec = codec;

	/* Imp: Store the card pointer in hda_codec */
	hdac->hdac.card = codec->card->snd_card;

	alc_drv_data = (struct alc_patch *)soc_hda_get_device_id(hdac->hdev)->driver_data;
	alc_drv_data->patch(hdac);

	/* add codec controls here dynamically */
	ret = snd_soc_hda_codec_build_controls(hdac);

	if (ret)
		return ret;
	snd_hda_codec_proc_new(&hdac->hdac);
	/* FIXME: dapm widgets and routes to be added dynamically */
	snd_soc_dapm_new_controls(&codec->dapm, alc_cvt_widgets, ARRAY_SIZE(alc_cvt_widgets));
	snd_soc_dapm_new_controls(&codec->dapm, alc_pin_widgets, ARRAY_SIZE(alc_pin_widgets));
	snd_soc_dapm_new_controls(&codec->dapm, alc_dapm_mixers, ARRAY_SIZE(alc_dapm_mixers));
	snd_soc_dapm_new_controls(&codec->dapm, alc_dapm_muxers, ARRAY_SIZE(alc_dapm_muxers));
	snd_soc_dapm_add_routes(&codec->dapm, alc_intercon, ARRAY_SIZE(alc_intercon));
	snd_soc_dapm_new_widgets(codec->card);

	pm_runtime_enable(&hdac->hdev->dev);
	pm_runtime_set_suspended(&hdac->hdev->dev);
	return 0;

}

static int alc_codec_remove(struct snd_soc_codec *codec)
{
	pr_debug("codec_remove called\n");
	return 0;
}

static struct snd_soc_codec_driver alc_hda_codec = {
	.probe		= alc_codec_probe,
	.remove		= alc_codec_remove,
	.idle_bias_off	= true,
/* dapm widgets and routes are added dynamically */
};

static int alc_set_hw_params(struct snd_pcm_substream *substream,
	struct snd_pcm_hw_params *hparams, struct snd_soc_dai *dai)
{
	struct snd_soc_hda_codec *codec = snd_soc_dai_get_drvdata(dai);
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_hda_dma_params *dd;
	struct hda_dai_map *map;
	int ret, idx;
	int format_id;

	dev_dbg(dai->dev, "In %s\n", __func__);

	idx = snd_soc_hda_get_dai_map(codec, dai);
	if (idx < 0)
		return -1;
	map = &codec->nid_list[idx];


	dd = kzalloc(sizeof(*dd), GFP_KERNEL);
	dd->format = snd_hda_calc_stream_format(params_rate(hparams),
			params_channels(hparams), params_format(hparams),
			map->maxbps, 0);
	dev_dbg(dai->dev, "hda format val = 0x%x\n", dd->format);
	snd_soc_dai_set_dma_data(dai, substream, (void *)dd);
	return 0;

}

static int alc_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_hda_codec *codec = snd_soc_dai_get_drvdata(dai);
	struct snd_soc_pcm_runtime *rtd = snd_pcm_substream_chip(substream);
	struct hda_dai_map *map;
	struct hda_cvt_setup *cvt;
	struct snd_soc_hda_dma_params *dd;
	int idx;

	dev_dbg(dai->dev, "In %s\n", __func__);
	if (substream == NULL)
		return -1;

	idx = snd_soc_hda_get_dai_map(codec, dai);
	if (idx < 0)
		return -1;
	map = &codec->nid_list[idx];

	cvt = snd_hda_get_cvt_setup(&codec->hdac, map->nid);
	if (cvt == NULL)
		return -1;

	dd = (struct snd_soc_hda_dma_params *)
				snd_soc_dai_get_dma_data(dai, substream);
	dev_dbg(dai->dev, "stream tag from cpu dai %di format=%d dainame=%s\n",
				dd->stream_tag, dd->format, dai->name);
	snd_hda_codec_setup_stream(&codec->hdac, map->nid, dd->stream_tag, 0, dd->format);
	return 0;

}


static int alc_hw_free(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_hda_codec *codec = snd_soc_dai_get_drvdata(dai);
	struct snd_soc_pcm_runtime *rtd = snd_pcm_substream_chip(substream);
	struct hda_dai_map *map;
	struct snd_soc_hda_dma_params *dd;
	int idx;

	if (substream == NULL)
		return -1;

	dev_dbg(dai->dev, "In %s\n", __func__);

	idx = snd_soc_hda_get_dai_map(codec, dai);
	if (idx < 0)
		return -1;
	map = &codec->nid_list[idx];
	__snd_hda_codec_cleanup_stream(&codec->hdac, map->nid, 1);

	dd = (struct snd_soc_hda_dma_params *)
				snd_soc_dai_get_dma_data(dai, substream);

	kfree(dd);
	return 0;
}

static struct snd_soc_dai_ops alc_dai_ops = {
	.prepare = alc_prepare,
	.hw_params = alc_set_hw_params,
	.hw_free = alc_hw_free,
};

static struct snd_soc_dai_driver dyn_dais[AUTO_CFG_MAX_OUTS+AUTO_CFG_MAX_INS];
static int alc_dev_probe(struct snd_soc_hda_device *hdev)
{
	struct snd_soc_hda_codec *shc;
	int ret, num_dais, i;

	ret = snd_soc_hda_codec_new(hdev, hdev->addr, &shc);
	if (ret)
		return -ENOMEM;

	shc->ops = alc_codec_ops;
	shc->hdac.vendor_name = kstrdup("Realtek", GFP_KERNEL);
	shc->hdac.chip_name = kstrdup(soc_hda_get_device_id(hdev)->name, GFP_KERNEL);

	snd_hda_pick_fixup(&shc->hdac, alc269_fixup_models,
		       alc269_fixup_tbl, alc269_fixups);
	snd_hda_apply_fixup(&shc->hdac, HDA_FIXUP_ACT_PRE_PROBE);

	snd_soc_hda_codec_configure(shc);

	ret = snd_soc_hda_build_codec_dais(shc, dyn_dais, &num_dais,
			&alc_dai_ops);
	if (ret)
		return ret;

	snd_soc_hda_set_drvdata(hdev, shc);

	/* ASoC specific initialization */
	ret = snd_soc_register_codec(&hdev->dev, &alc_hda_codec,
			dyn_dais, num_dais);
	return ret;
}

static int alc_dev_remove(struct snd_soc_hda_device *hdev)
{
	snd_soc_unregister_codec(&hdev->dev);
	pm_runtime_disable(&hdev->dev);
	return 0;
}

static void alc_dev_shutdown(struct snd_soc_hda_device *hdev)
{
	/* FIXME: what needs to be done here */
}

#ifdef CONFIG_PM
static int alc_suspend(struct snd_soc_hda_device *hdev, pm_message_t mesg)
{
	struct snd_soc_hda_codec *codec = snd_soc_hda_get_drvdata(hdev);

	/* FIXME: chk if anything else needs to be done here */
	return 0;
}

static int alc_resume(struct snd_soc_hda_device *hdev)
{
	struct snd_soc_hda_codec *codec = snd_soc_hda_get_drvdata(hdev);
	msleep(150); /* to avoid pop noise */
	/* FIXME: is the below seq required ? */
#if 0
	codec->ops.init(codec);
	snd_hda_codec_resume_amp(codec);
	snd_hda_codec_resume_cache(codec);
#endif
	return 0;
}


static int alc_runtime_suspend(struct device *dev)
{
	struct snd_soc_hda_device *hdev = to_soc_hda_device(dev);
	struct snd_soc_hda_codec *codec = snd_soc_hda_get_drvdata(hdev);
	/* power down only AFG here */
	snd_hda_set_power_state(&codec->hdac, codec->hdac.afg, AC_PWRST_D3);
	return 0;
}

static int alc_runtime_resume(struct device *dev)
{
	struct snd_soc_hda_device *hdev = to_soc_hda_device(dev);
	struct snd_soc_hda_codec *codec = snd_soc_hda_get_drvdata(hdev);

	/* power up only AFG here */
	snd_hda_set_power_state(&codec->hdac, codec->hdac.afg, AC_PWRST_D0);
	msleep(150); /* to avoid pop noise */
	/* FIXME: is the below seq required? */
#if 0
	snd_hda_codec_resume_amp(codec);
	snd_hda_codec_resume_cache(codec);
#endif
	return 0;

}

static int alc_runtime_idle(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops alc_rtpm_ops = {
	SET_RUNTIME_PM_OPS(alc_runtime_suspend,
			alc_runtime_resume, alc_runtime_idle)
};
#endif /*__CONFIG_PM__ */

static void alc_jack_unsol_event(struct snd_soc_hda_device *hdev, unsigned int res)
{
	struct snd_soc_hda_codec *codec = snd_soc_hda_get_drvdata(hdev);
	snd_hda_jack_unsol_event(&codec->hdac, res);
}

static struct alc_patch alc269_drv_data = {
	.patch = &patch_alc269,
};

static struct alc_patch alc275_drv_data = {
	.patch = &patch_alc269,
};

static struct alc_patch alc286_drv_data = {
	.patch = &patch_alc269,
};

static const struct snd_soc_hda_device_id realtek_list[] = {
{ .id = 0x10ec0269, .addr = 0, .name = "ALC269", (kernel_ulong_t)&alc269_drv_data},
{ .id = 0x10ec0275, .addr = 0, .name = "ALC275", (kernel_ulong_t)&alc275_drv_data},
{ .id = 0x10ec0286, .addr = 0, .name = "ALC286", (kernel_ulong_t)&alc286_drv_data},
{},
};

static struct snd_soc_hda_driver realtek_hda_driver = {
	.driver	= {
		.name	= "Realtek-HDA-Codec",
		.owner	= THIS_MODULE,
		.pm	= &alc_rtpm_ops,
	},
	.id_table	= realtek_list,
	.probe		= alc_dev_probe,
	.remove		= alc_dev_remove,
	.shutdown	= alc_dev_shutdown,
#ifdef CONFIG_PM
	.suspend	= alc_suspend,
	.resume		= alc_resume,
#endif
	.unsol_event	= alc_jack_unsol_event,
};

int __init realtek_init(void)
{
	int ret;

	ret = snd_soc_hda_driver_register(&realtek_hda_driver);
	return ret;
}

void __exit realtek_exit(void)
{
	snd_soc_hda_driver_unregister(&realtek_hda_driver);
}

module_init(realtek_init);
module_exit(realtek_exit);

MODULE_ALIAS("snd-hda-codec-id:10ec*");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Realtek-HDA-codec");
