/*
 *  soc-hda-control.c -HD Audio Platform component ALSA controls for BXT
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
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#include <linux/slab.h>
#include <linux/types.h>
#include <sound/soc.h>
#include <sound/tlv.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <linux/firmware.h>
#include <sound/soc-fw.h>
#include <sound/hda_controller.h>
#include <sound/soc-hda-sst-dsp.h>
#include "soc-hda-controller.h"
#include "soc-hda-controls.h"
#include "soc-hda-vendor.h"

#define CH_FIXUP		(1 << 0)
#define RATE_FIXUP		(1 << 1)
#define FMT_FIXUP		(1 << 2)

#define CH_FIXUP_MASK		(1 << 0)
#define RATE_FIXUP_MASK		(1 << 1)
#define FMT_FIXUP_MASK		(1 << 2)


#define CH_CONVERTER		(1 << 0)
#define RATE_CONVERTER		(1 << 1)
#define FMT_CONVERTER		(1 << 2)

#define CH_CONVERTER_MASK	(1 << 0)
#define RATE_CONVERTER_MASK	(1 << 1)
#define FMT_CONVERTER_MASK	(1 << 2)

#define REGS_OFFSET_CPR_BLOB	8

static int hda_sst_src_bind_unbind_modules(struct snd_soc_dapm_widget *w,
			struct sst_dsp_ctx *ctx, bool bind, bool is_pipe);

static int hda_sst_algo_control_get(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct soc_bytes_ext *sb = (void *) kcontrol->private_value;
	struct hda_sst_algo_data *bc = (struct hda_sst_algo_data *)sb->pvt_data;
	struct snd_soc_dapm_context *dapm = snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct azx *chip = dev_get_drvdata(dapm->dev);

	dev_dbg(chip->dev, "In%s control_name=%s\n", __func__, kcontrol->id.name);
	switch (bc->type) {
	case HDA_SST_ALGO_PARAMS:
		if (bc->params)
			memcpy(ucontrol->value.bytes.data, bc->params, bc->max);
		break;
	default:
		dev_err(chip->dev, "Invalid Input- algo type:%d\n", bc->type);
		return -EINVAL;

	}
	return 0;
}

static int hda_sst_algo_control_set(struct snd_kcontrol *kcontrol,
				struct snd_ctl_elem_value *ucontrol)
{
	struct snd_soc_dapm_context *dapm = snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct azx *chip = dev_get_drvdata(dapm->dev);
	struct soc_bytes_ext *sb = (void *) kcontrol->private_value;
	struct hda_sst_algo_data *bc = (struct hda_sst_algo_data *)sb->pvt_data;

	dev_dbg(chip->dev, "in %s control_name=%s\n", __func__, kcontrol->id.name);
	switch (bc->type) {
	case HDA_SST_ALGO_PARAMS:
		if (bc->params)
			memcpy(bc->params, ucontrol->value.bytes.data, bc->max);
		break;
	default:
		dev_err(chip->dev, "Invalid Input- algo type:%ld\n",
			ucontrol->value.integer.value[0]);
		return -EINVAL;
	}
	/*if (w->power)
		send set config data
	*/
	return 0;
}

unsigned int soc_hda_soc_read(struct snd_soc_platform *platform, unsigned int reg)
{
	struct azx *chip  = snd_soc_platform_get_drvdata(platform);
	struct snd_soc_azx *schip =
			container_of(chip, struct snd_soc_azx, hda_azx);
	struct hda_platform_info *pinfo = schip->pinfo;

	dev_dbg(chip->dev, "%s: reg[%d] = %#x\n", __func__, reg, pinfo->widget[reg]);
	BUG_ON(reg > (HDA_SST_NUM_WIDGETS - 1));
	return pinfo->widget[reg];
}

int soc_hda_soc_write(struct snd_soc_platform *platform,
	unsigned int reg, unsigned int val)
{
	struct azx *chip  = snd_soc_platform_get_drvdata(platform);
	struct snd_soc_azx *schip =
			container_of(chip, struct snd_soc_azx, hda_azx);
	struct hda_platform_info *pinfo = schip->pinfo;

	dev_dbg(platform->dev, "%s:reg[%d]= %#x\n", __func__, reg, pinfo->widget[reg]);
	BUG_ON(reg > (HDA_SST_NUM_WIDGETS - 1));
	pinfo->widget[reg] = val;
	return 0;
}

unsigned int hda_sst_reg_read(struct hda_platform_info *pinfo, unsigned int reg,
	unsigned int shift, unsigned int max)
{
	unsigned int mask = (1 << fls(max)) - 1;

	return (pinfo->widget[reg] >> shift) & mask;
}

unsigned int hda_sst_reg_write(struct hda_platform_info *pinfo, unsigned int reg,
	unsigned int shift, unsigned int max, unsigned int val)
{
	unsigned int mask = (1 << fls(max)) - 1;

	val &= mask;
	val <<= shift;
	pinfo->widget[reg] &= ~(mask << shift);
	pinfo->widget[reg] |= val;
	return val;
}

int hda_sst_mix_put(struct snd_kcontrol *kcontrol,
	struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
				(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_dapm_context *dapm = snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct azx *chip  = dev_get_drvdata(dapm->dev);
	struct snd_soc_azx *schip =
			container_of(chip, struct snd_soc_azx, hda_azx);
	struct hda_platform_info *pinfo = schip->pinfo;
	unsigned int mask = (1 << fls(mc->max)) - 1;
	unsigned int val;
	int connect;
	struct snd_soc_dapm_update update;

	dev_dbg(chip->dev, "%s called set %#lx\n", __func__, ucontrol->value.integer.value[0]);
	val = hda_sst_reg_write(pinfo, mc->reg, mc->shift, mc->max,
				ucontrol->value.integer.value[0]);
	connect = !!val;

	dapm_kcontrol_set_value(kcontrol,  val);
	update.kcontrol = kcontrol;
	update.reg = mc->reg;
	update.mask = mask << mc->shift;
	update.val = val;

	snd_soc_dapm_mixer_update_power(dapm, kcontrol, connect, &update);
	return 0;
}

int hda_sst_mix_get(struct snd_kcontrol *kcontrol,
		struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *mc =
			(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_soc_dapm_context *dapm = snd_soc_dapm_kcontrol_dapm(kcontrol);
	struct azx *chip  = dev_get_drvdata(dapm->dev);
	struct snd_soc_azx *schip =
			container_of(chip, struct snd_soc_azx, hda_azx);
	struct hda_platform_info *pinfo = schip->pinfo;

	ucontrol->value.integer.value[0] = !!hda_sst_reg_read(pinfo, mc->reg, mc->shift, mc->max);
	return 0;
}

static bool is_last_pipeline(struct snd_soc_dapm_widget *w,
		struct device *dev)
{
	struct module_config *smodule = w->priv;
	struct module_config *module = NULL;
	struct snd_soc_dapm_path *p = NULL;

	dev_dbg(dev, "%s: widget = %s\n", __func__, w->name);

	if (smodule == NULL)
		return 0;
	if (smodule->conn_type == SOURCE) {
		list_for_each_entry(p, &w->sources, list_sink) {
			if (p->connected && !p->connected(w, p->sink)
				&& (p->source->priv == NULL))
				continue;

			if (p->connect && p->source->power) {
				module = p->source->priv;
				if (module->pipe->conn_type == CONN_TYPE_BE)
					return true;
				else
					return false;
			}
		}
	}

	if (smodule->conn_type == SINK) {
		list_for_each_entry(p, &w->sinks, list_source) {
			if (p->connected && !p->connected(w, p->sink)
				&& (p->sink->priv == NULL))
				continue;
			if (p->connect && p->sink->power) {
				module = p->sink->priv;
				if (module->pipe->conn_type == CONN_TYPE_BE)
					return true;
				else
					return false;
			}
		}
	}

	return false;
}

static int is_hda_widget_type(struct snd_soc_dapm_widget *w)
{
	return ((w->id == snd_soc_dapm_dai_link) ||
		(w->id == snd_soc_dapm_dai_in) ||
		(w->id == snd_soc_dapm_aif_in) ||
		(w->id == snd_soc_dapm_aif_out) ||
		(w->id == snd_soc_dapm_dai_out)) ? 1 : 0;

}

static int hda_sst_src_bind_unbind_modules(struct snd_soc_dapm_widget *w,
			struct sst_dsp_ctx *ctx,  bool bind, bool is_pipe)
{
	struct module_config *sink_module = w->priv;
	struct module_config *src_module = NULL;
	struct snd_soc_dapm_path *p = NULL;
	int ret = 0;

	dev_dbg(ctx->dev, "%s: widget = %s conn_type=%d\n", __func__, w->name, sink_module->hw_conn_type);
	list_for_each_entry(p, &w->sources, list_sink) {
		if (p->connected && !p->connected(w, p->sink)
			&& (p->source->priv == NULL)
			&& (is_hda_widget_type(w)))
			continue;
		dev_dbg(ctx->dev, "%s: SINK for  sink widget = %s\n", __func__, w->name);
		if (p->connect && (p->source->new_power == bind) &&
			(p->source->priv != NULL) &&
			 (!is_hda_widget_type(p->source))) {
			src_module = p->source->priv;
			dev_dbg(ctx->dev, "%s: SINK for  sink  widget id  = %d src_id=%d\n", __func__, w->id , p->source->id);
			dev_dbg(ctx->dev, "%s: SINK widget = %s\n", __func__, p->source->name);
			dev_dbg(ctx->dev, "%s: wirgdet source =%s\n", __func__, p->sink->name);
			if (!is_pipe && (sink_module->pipe->ppl_id == src_module->pipe->ppl_id)) {
				dev_dbg(ctx->dev, "%s: wirgdet source id =%s\n", __func__, p->sink->name);
				if (p->source->id == snd_soc_dapm_mixer) {
					if (sink_module->hw_conn_type == SINK)
						ret = hda_sst_bind_unbind_modules(ctx, src_module, sink_module, bind);
					else
						ret = hda_sst_bind_unbind_modules(ctx, src_module, sink_module, bind);
				} else {
					if (sink_module->hw_conn_type == SINK)
						ret = hda_sst_bind_unbind_modules(ctx, src_module, sink_module, bind);
				}
			} else if (is_pipe && (sink_module->pipe->ppl_id != src_module->pipe->ppl_id)) {
				dev_dbg(ctx->dev, "%s: wirgdet source id =%s sink=%s sink_pipe=%d src_pipe=%d\n", __func__, p->sink->name,
					w->name, src_module->pipe->ppl_id, sink_module->pipe->ppl_id);
				if (sink_module->hw_conn_type == SINK)
					ret = hda_sst_bind_unbind_modules(ctx, src_module, sink_module, bind);
				else
					ret = hda_sst_bind_unbind_modules(ctx, src_module, sink_module, bind);
			}
		}
	}

	list_for_each_entry(p, &w->sinks, list_source) {
		if (p->connected && !p->connected(w, p->sink)
			&& (p->sink->priv == NULL)
			&& (is_hda_widget_type(w)))
			continue;

		dev_dbg(ctx->dev, "%s: SRC for  sink widget = %s\n", __func__, w->name);
		if (p->connect && (p->sink->new_power == bind) &&
			(p->sink->priv != NULL) &&
			(!is_hda_widget_type(p->sink))) {
			src_module = p->sink->priv;
			dev_dbg(ctx->dev, "%s: SRC for  sink  widget id  = %d src_id=%d\n", __func__, w->id , p->sink->id);
			dev_dbg(ctx->dev, "%s: SRC wirgdet source id = %s\n", __func__, p->source->name);
			dev_dbg(ctx->dev, "%s: wirgdet sink = %s\n", __func__, p->sink->name);
			if (!is_pipe && (sink_module->pipe->ppl_id == src_module->pipe->ppl_id)) {
				dev_dbg(ctx->dev, "%s: wirgdet sink id = %s\n", __func__, p->sink->name);
				if (p->sink->id == snd_soc_dapm_mixer) {
					if (sink_module->hw_conn_type == SOURCE)
						ret = hda_sst_bind_unbind_modules(ctx, src_module, sink_module, bind);
					else
						ret = hda_sst_bind_unbind_modules(ctx, src_module, sink_module, bind);
				} else {
					if (sink_module->hw_conn_type == SOURCE)
						ret = hda_sst_bind_unbind_modules(ctx, sink_module, src_module, bind);
				}
			} else if (is_pipe && (sink_module->pipe->ppl_id != src_module->pipe->ppl_id)) {
				if (sink_module->hw_conn_type == SOURCE)
					ret = hda_sst_bind_unbind_modules(ctx, sink_module, src_module, bind);
				else
					ret = hda_sst_bind_unbind_modules(ctx, sink_module, src_module, bind);
			}
		}
	}

	return ret;
}

static bool hda_sst_is_pipe_mem_available(struct hda_platform_info *pinfo,
	struct sst_dsp_ctx *ctx, struct module_config *mconfig)
{
	dev_dbg(ctx->dev, "%s: module_id =%d instance=%d\n", __func__, mconfig->id.module_id, mconfig->id.instance_id);
	pinfo->resource.mem += mconfig->pipe->memory_pages;

	if (pinfo->resource.mem > pinfo->resource.max_mem) {
		dev_err(ctx->dev, "exceeds ppl memory available=%d > mem=%d\n",
				pinfo->resource.max_mem, pinfo->resource.mem);
		/* FIXME pinfo->resource.mem -= mconfig->pipe->memory_pages;
		return false;*/
	}
	return true;
}

static bool hda_sst_is_pipe_mcps_available(struct hda_platform_info *pinfo,
	struct sst_dsp_ctx *ctx, struct module_config *mconfig)
{
	dev_dbg(ctx->dev, "%s: module_id = %d instance=%d\n", __func__,
			mconfig->id.module_id, mconfig->id.instance_id);
	pinfo->resource.mcps += mconfig->mcps;

	if (pinfo->resource.mcps > pinfo->resource.max_mcps) {
		dev_err(ctx->dev, "exceeds ppl memory available=%d > mem=%d\n",
				pinfo->resource.max_mcps, pinfo->resource.mcps);
		/*FIXME pinfo->resource.mcps -= mconfig->mcps;
		return false*/
	}
	return true;
}
static struct snd_soc_dai *hda_find_dai_in(struct list_head *sinks)
{
	struct snd_soc_dapm_path *p;
	struct snd_soc_dai *dai = NULL;
	list_for_each_entry(p, sinks, list_source) {
		if (p->connect) {
			if (p->sink->id == snd_soc_dapm_dai_in ||
					p->sink->id == snd_soc_dapm_dai_out) {
				dai = p->sink->priv;
				return dai;
			}
			dai = hda_find_dai_in(&p->sink->sinks);
			if (dai)
				return dai;
		}
	}
	return dai;
}

static struct snd_soc_dai *hda_find_dai_out(struct list_head *sources)
{
	struct snd_soc_dapm_path *p;
	struct snd_soc_dai *dai = NULL;
	list_for_each_entry(p, sources, list_sink) {
		if (p->connect) {
			if (p->source->id == snd_soc_dapm_dai_in ||
					p->source->id == snd_soc_dapm_dai_out) {
				dai = p->source->priv;
				break;
			}
			dai = hda_find_dai_out(&p->source->sources);
			if (dai)
				return dai;
		}
	}
	return dai;
}

static struct azx_dai_config *hda_sst_get_dai_config(struct snd_soc_dapm_widget *w,
		struct module_config *mconfig, struct sst_dsp_ctx *ctx)
{
	struct snd_soc_dai *dai = NULL;
	struct azx *chip = NULL;
	if (mconfig->hw_conn_type == SOURCE) {
		if (mconfig->pipe->conn_type == CONN_TYPE_BE)
			dai = hda_find_dai_in(&w->sinks);
		else if (mconfig->pipe->conn_type == CONN_TYPE_FE)
			dai = hda_find_dai_out(&w->sources);
	} else if (mconfig->hw_conn_type == SINK) {
		if (mconfig->pipe->conn_type == CONN_TYPE_BE)
			dai = hda_find_dai_out(&w->sources);
		else if (mconfig->pipe->conn_type == CONN_TYPE_FE)
			dai = hda_find_dai_in(&w->sinks);
	}
	if (!dai) {
		dev_dbg(ctx->dev, "Dai not found for widget %s\n", w->name);
		return NULL;
	}
	dev_dbg(ctx->dev, "Dai found %s for widget %s\n",
			dai->name, w->name);
	chip = dev_get_drvdata(dai->dev);
	return &chip->dai_config[dai->id - 1];

}

static void hda_dump_mconfig(struct sst_dsp_ctx *ctx,
					struct module_config *mcfg)
{
	dev_dbg(ctx->dev, "Dumping Mconfig\n");
	dev_dbg(ctx->dev, "Input Format:\n");
	dev_dbg(ctx->dev, "channels = %d\n", mcfg->in_fmt.channels);
	dev_dbg(ctx->dev, "sampling_freq = %d\n", mcfg->in_fmt.sampling_freq);
	dev_dbg(ctx->dev, "channel_config = %d\n", mcfg->in_fmt.channel_config);
	dev_dbg(ctx->dev, "valid bit depth = %d\n", mcfg->in_fmt.valid_bit_depth);
	dev_dbg(ctx->dev, "Output Format:\n");
	dev_dbg(ctx->dev, "channels = %d\n", mcfg->out_fmt.channels);
	dev_dbg(ctx->dev, "sampling_freq = %d\n", mcfg->out_fmt.sampling_freq);
	dev_dbg(ctx->dev, "valid bit depth = %d\n", mcfg->out_fmt.valid_bit_depth);
	dev_dbg(ctx->dev, "channel_config = %d\n", mcfg->out_fmt.channel_config);
}
static void hda_dump_dai_config(struct sst_dsp_ctx *ctx,
					struct azx_dai_config *cfg)
{
	struct azx_ssp_dai_config *ssp_cfg = &cfg->ssp_dai_config;
	dev_dbg(ctx->dev, "Dumping SSP config\n");
	dev_dbg(ctx->dev, "slot_width = %d\n", ssp_cfg->slot_width);
	dev_dbg(ctx->dev, "Slot = %d\n", ssp_cfg->slots);
	dev_dbg(ctx->dev, "ssp_mode = %d\n", ssp_cfg->ssp_mode);
	dev_dbg(ctx->dev, "sampling rate = %d\n", cfg->sampling_rate);
	dev_dbg(ctx->dev, "s_fmt = %d\n", cfg->s_fmt);
	dev_dbg(ctx->dev, "bclk_invert = %d\n", ssp_cfg->bclk_invert);
	dev_dbg(ctx->dev, "fs_invert = %d\n", ssp_cfg->fs_invert);
	dev_dbg(ctx->dev, "num_channels = %d\n", cfg->num_channels);
	dev_dbg(ctx->dev, "fs_slave = %d\n", ssp_cfg->fs_slave);
	dev_dbg(ctx->dev, "bclk_slave = %d\n", ssp_cfg->bclk_slave);
}

static void hda_update_mconfig(struct module_format *fmt,
						struct azx_dai_config *ssp_cfg,
						int params_fixup)
{
	if (params_fixup & RATE_FIXUP_MASK)
		fmt->sampling_freq = ssp_cfg->sampling_rate;
	if (params_fixup & CH_FIXUP_MASK)
		fmt->channels = ssp_cfg->num_channels;
	if (params_fixup & CH_FIXUP_MASK)
		fmt->channels = ssp_cfg->num_channels;

}

static void hda_update_ch_config(struct module_config *m_cfg)
{
	struct module_format *in_fmt, *out_fmt;

	in_fmt = &m_cfg->in_fmt;
	out_fmt = &m_cfg->out_fmt;

	if (in_fmt->channels == 2) {
		in_fmt->channel_config =  CHANNEL_CONFIG_STEREO;
		if (out_fmt->channels == 1)
			out_fmt->channel_config = CHANNEL_CONFIG_MONO;
		if (out_fmt->channels == 2)
			out_fmt->channel_config = CHANNEL_CONFIG_STEREO;
	} else if (in_fmt->channels == 1) {
		in_fmt->channel_config =  CHANNEL_CONFIG_MONO;
			if (out_fmt->channels == 1)
				out_fmt->channel_config = CHANNEL_CONFIG_MONO;
			if (out_fmt->channels == 2)
				out_fmt->channel_config = CHANNEL_CONFIG_STEREO;
	}

}

static void hda_sst_update_cpr_ssp_id(struct snd_soc_dapm_widget *w,
		struct module_config *mconfig, struct sst_dsp_ctx *ctx)
{
	char *cpr, *ssp;
	union ssp_dma_node dma_id;
	bool id_found = 1;
	cpr = strstr(w->name, "cpr");
	if (cpr) {
		dma_id.val = mconfig->dma_id;
		id_found = 0;
		dev_dbg(ctx->dev, "%s: Copier found widget =%s sname = %s\n", __func__, w->name, w->sname);
		ssp = strstr(w->name, "codec");
		if (ssp) {
			dev_dbg(ctx->dev, "%s SSP instance set to 0", __func__);
			dma_id.dma_node.i2s_instance = 0;
			mconfig->dma_id = dma_id.val;
			id_found = 1;
		}
		ssp = strstr(w->name, "modem0");
		if (ssp) {
			dev_dbg(ctx->dev, "%s SSP instance set to 2", __func__);
			dma_id.dma_node.i2s_instance = 2;
			mconfig->dma_id = dma_id.val;
			id_found = 1;
		}
		ssp = strstr(w->name, "bt");
		if (ssp) {
			dev_dbg(ctx->dev, "%s SSP instance set to 1", __func__);
			dma_id.dma_node.i2s_instance = 1;
			mconfig->dma_id = dma_id.val;
			id_found = 1;
		}
	}
	if (cpr && !id_found)
		dev_dbg(ctx->dev, "Valid SSP id not found for cpr %s\n", w->name);

}
static void hda_update_buffer_size(struct sst_dsp_ctx *ctx,
				struct module_config *mcfg)
{

	mcfg->ibs = (mcfg->in_fmt.sampling_freq / 1000) *
				(mcfg->in_fmt.channels) *
				(mcfg->in_fmt.bit_depth >> 3);

	mcfg->obs = (mcfg->out_fmt.sampling_freq / 1000) *
				(mcfg->out_fmt.channels) *
				(mcfg->out_fmt.bit_depth >> 3);
}

static void hda_sst_configure_widget(struct snd_soc_dapm_widget *w,
	int w_type, struct sst_dsp_ctx *ctx, struct hda_platform_info *pinfo)
{
	struct module_config *m_cfg = w->priv;
	struct azx_dai_config *dai_config;
	union ssp_dma_node dma_id;
	unsigned int *regs = NULL;

	dai_config = hda_sst_get_dai_config(w, m_cfg, ctx);
	if (!dai_config)
		return;

	if (m_cfg->id.module_id == COPIER_MODULE &&
		m_cfg->pipe->conn_type == CONN_TYPE_BE &&
		dai_config->dai_type == AZX_DAI_TYPE_SSP) {
		dma_id.dma_node.i2s_instance =
				dai_config->ssp_dai_config.i2s_instance;
		m_cfg->dma_id = dma_id.val;
		regs = m_cfg->formats_config.caps;
	}
	if (!m_cfg->params_fixup)
		return;

	dev_dbg(ctx->dev, "Mconfig for widget  %s BEFORE updation\n", w->name);
	hda_dump_mconfig(ctx, m_cfg);

	/* Based on whether the widget is in FE pipe or BE PIPE and playback direction
	 * or capture direction, fixups applied will be changed
	 */
	if  ((m_cfg->pipe->conn_type == CONN_TYPE_FE &&
		(m_cfg->hw_conn_type  == SINK)) ||
		(m_cfg->pipe->conn_type == CONN_TYPE_BE &&
		m_cfg->hw_conn_type  == SOURCE)) {
		hda_update_mconfig(&m_cfg->out_fmt, dai_config,
						m_cfg->params_fixup);
		hda_update_mconfig(&m_cfg->in_fmt, dai_config,
				(~m_cfg->converter) & m_cfg->params_fixup);
	}
	if  ((m_cfg->pipe->conn_type == CONN_TYPE_BE &&
		(m_cfg->hw_conn_type  == SINK)) ||
		(m_cfg->pipe->conn_type == CONN_TYPE_FE &&
		m_cfg->hw_conn_type  == SOURCE)) {
		hda_update_mconfig(&m_cfg->in_fmt, dai_config,
						m_cfg->params_fixup);
		hda_update_mconfig(&m_cfg->out_fmt, dai_config,
				(~m_cfg->converter) & m_cfg->params_fixup);
	}

	hda_update_ch_config(m_cfg);
	hda_update_buffer_size(ctx, m_cfg);
	if (regs)
		azx_calculate_ssp_regs(ctx, dai_config,
				&regs[REGS_OFFSET_CPR_BLOB]);
	dev_dbg(ctx->dev, "Mconfig for widget  %s AFTER updation\n", w->name);
	hda_dump_mconfig(ctx, m_cfg);
	hda_dump_dai_config(ctx, dai_config);

}

static int hda_sst_dapm_pre_pmu_event(struct snd_soc_dapm_widget *w,
	int w_type, struct sst_dsp_ctx *ctx, struct hda_platform_info *pinfo)
{
	int ret = 0;
	struct module_config *mconfig = w->priv;

	dev_dbg(ctx->dev, "%s: widget =%s type=%d\n", __func__, w->name, w_type);

	/*check resource available */
	if (!hda_sst_is_pipe_mcps_available(pinfo, ctx, mconfig))
		return -1;

	hda_sst_configure_widget(w, w_type, ctx, pinfo);
	hda_sst_update_cpr_ssp_id(w, mconfig, ctx);
	if (w_type == HDA_SST_WIDGET_VMIXER ||
		w_type == HDA_SST_WIDGET_MIXER) {

		if (!hda_sst_is_pipe_mem_available(pinfo, ctx, mconfig))
			return -ENOMEM;

		ret = hda_sst_create_pipeline(ctx, mconfig->pipe);
		if (ret < 0)
			return ret;
		hda_sst_update_cpr_ssp_id(w, mconfig, ctx);
		ret = hda_sst_init_module(ctx, mconfig, NULL);
		if (ret < 0)
			return ret;
	}

	if (w_type == HDA_SST_WIDGET_PGA) {
		/*if (mconfig->is_loadable) {
			ret = sst_load_module(ctx, mconfig);
			goto err;
		}*/

		if (w->num_kcontrols == 1) {
			struct soc_bytes_ext *sb = (void *) w->kcontrol_news[0].private_value;
			struct hda_sst_algo_data *bc = (struct hda_sst_algo_data *)sb->pvt_data;

			ret = hda_sst_init_module(ctx, mconfig, bc);
		} else
			ret = hda_sst_init_module(ctx, mconfig, NULL);
		if (ret < 0)
			return ret;

		/*bind modules */
		ret = hda_sst_src_bind_unbind_modules(w, ctx, true, false);
		if (ret < 0)
			return ret;
	}

	return ret;
}

static int hda_sst_dapm_post_pmu_event(struct snd_soc_dapm_widget *w,
	int w_type, struct sst_dsp_ctx *ctx, struct hda_platform_info *pinfo)
{
	struct module_config *mconfig = w->priv;
	struct sst_pipeline *ppl, *__ppl;
	int ret = 0;
	int bind_done = 0;
	enum sst_module_state prev_m_state;

	dev_dbg(ctx->dev, "%s: widget = %s\n", __func__, w->name);

	/*bind modules pipes*/
	if (w_type == HDA_SST_WIDGET_PGA) {
		/*bind module */
		prev_m_state = mconfig->m_state;
		ret = hda_sst_src_bind_unbind_modules(w, ctx, true, true);
		if (mconfig->m_state == BIND_DONE && prev_m_state != BIND_DONE)
			bind_done = 1;
	}

	if (w_type == HDA_SST_WIDGET_VMIXER ||
		w_type == HDA_SST_WIDGET_MIXER) {
		if (mconfig->pipe->conn_type != CONN_TYPE_FE) {
			/*if module is not a FE then add to ppl_start list,
			 *to send the run pipe when be is reached */
			ppl = kzalloc(sizeof(*ppl), GFP_KERNEL);
			if (!ppl) {
				dev_err(ctx->dev, "kzalloc block failed\n");
					return -ENOMEM;
			}
			ppl->pipe = mconfig->pipe;
			list_add(&ppl->node, &pinfo->ppl_start_list);
		}
	}

	if ((w_type == HDA_SST_WIDGET_PGA)
	&& (bind_done)) {
		list_for_each_entry_safe(ppl, __ppl, &pinfo->ppl_start_list, node) {
			list_del(&ppl->node);

			dev_dbg(ctx->dev, "%s: set to run pipe_id =%d\n", __func__, ppl->pipe->ppl_id);
			ret = hda_sst_run_pipe(ctx, ppl->pipe);
			kfree(ppl);
			if (ret < 0)
				return ret;
		}
		bind_done = 0;
	}
	return ret;
}

static int hda_sst_dapm_pre_pmd_event(struct snd_soc_dapm_widget *w,
	int w_type, struct sst_dsp_ctx *ctx, struct hda_platform_info *pinfo)
{
	struct module_config *mconfig = w->priv;
	int ret = 0;

	dev_dbg(ctx->dev, "****%s: widget = %s\n", __func__, w->name);

	if (w_type == HDA_SST_WIDGET_PGA) {
		if (mconfig->pipe->conn_type != CONN_TYPE_FE) {
			ret = hda_sst_stop_pipe(ctx, mconfig->pipe);
			if (ret < 0)
				return ret;
		}
		ret = hda_sst_src_bind_unbind_modules(w, ctx, false, true);
		return ret;
	}

	return ret;
}

static int hda_sst_dapm_post_pmd_event(struct snd_soc_dapm_widget *w,
	int w_type, struct sst_dsp_ctx *ctx, struct hda_platform_info *pinfo)
{
	struct module_config *mconfig = w->priv;
	int ret = 0;

	dev_dbg(ctx->dev, "%s: widget = %s\n", __func__, w->name);

	pinfo->resource.mcps -= mconfig->mcps;
	if (w_type == HDA_SST_WIDGET_PGA) {
		ret = hda_sst_src_bind_unbind_modules(w, ctx, false, false);
		if (ret < 0)
			return ret;
		/*if(mconfig->is_loadable)
			sst_unload_module(mconfig); */
	}

	if (w_type == HDA_SST_WIDGET_VMIXER ||
		w_type == HDA_SST_WIDGET_MIXER) {
		ret = hda_sst_delete_pipe(ctx, mconfig->pipe);
		pinfo->resource.mem -= mconfig->pipe->memory_pages;
	}
	return ret;
}

static int hda_sst_event_handler(struct snd_soc_dapm_widget *w,
				int event, int w_type)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct azx *chip  = dev_get_drvdata(dapm->dev);
	struct snd_soc_azx *schip =
			container_of(chip, struct snd_soc_azx, hda_azx);
	struct hda_platform_info *pinfo = schip->pinfo;
	struct sst_dsp_ctx *ctx = schip->dsp;

	dev_dbg(dapm->dev, "%s: widget = %s\n", __func__, w->name);

	switch (event) {
	case SND_SOC_DAPM_PRE_PMU:
		return hda_sst_dapm_pre_pmu_event(w, w_type, ctx, pinfo);
	break;
	case SND_SOC_DAPM_POST_PMU:
		return hda_sst_dapm_post_pmu_event(w, w_type, ctx, pinfo);
	break;
	case SND_SOC_DAPM_PRE_PMD:
		return hda_sst_dapm_pre_pmd_event(w, w_type, ctx, pinfo);
	break;
	case SND_SOC_DAPM_POST_PMD:
		return hda_sst_dapm_post_pmd_event(w, w_type, ctx, pinfo);
	break;
	}

	return 0;
}

static int hda_sst_vmixer_event(struct snd_soc_dapm_widget *w,
		struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;

	dev_dbg(dapm->dev, "%s: widget = %s\n", __func__, w->name);
	return hda_sst_event_handler(w, event, HDA_SST_WIDGET_VMIXER);
}

static int hda_sst_mixer_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;

	dev_dbg(dapm->dev, "%s: widget = %s\n", __func__, w->name);

	return hda_sst_event_handler(w, event, HDA_SST_WIDGET_MIXER);
}

static int hda_sst_mux_event(struct snd_soc_dapm_widget *w,
			struct snd_kcontrol *k, int event)
{
	struct snd_soc_dapm_context *dapm = w->dapm;

	dev_dbg(dapm->dev, "%s: widget = %s\n", __func__, w->name);

	return hda_sst_event_handler(w, event, HDA_SST_WIDGET_MUX);
}

static int hda_sst_pga_event(struct snd_soc_dapm_widget *w,
				struct snd_kcontrol *k, int event)

{
	struct snd_soc_dapm_context *dapm = w->dapm;

	dev_dbg(dapm->dev, "%s: widget = %s\n", __func__, w->name);
	return hda_sst_event_handler(w, event, HDA_SST_WIDGET_PGA);
}

const struct snd_soc_fw_kcontrol_ops control_ops[] = {
	{SOC_CONTROL_IO_HDA_SST_ALGO_PARAMS, hda_sst_algo_control_get, hda_sst_algo_control_set, snd_soc_bytes_info_ext},
	{SOC_CONTROL_IO_HDA_SST_MIX, hda_sst_mix_get, hda_sst_mix_put, snd_soc_info_volsw},
};

const struct snd_soc_fw_widget_events hda_sst_widget_ops[] = {
	{HDA_SST_SET_MIXER, hda_sst_mixer_event},
	{HDA_SST_SET_MUX, hda_sst_mux_event},
	{HDA_SST_SET_VMIXER, hda_sst_vmixer_event},
	{HDA_SST_SET_PGA, hda_sst_pga_event},
};

static int hda_sst_copy_algo_control(struct device *dev,
		struct soc_bytes_ext *be, struct snd_soc_fw_bytes_ext *mbe)
{
	struct hda_sst_algo_data *ac;
	struct hda_dfw_algo_data *fw_ac = (struct hda_dfw_algo_data *)mbe->pvt_data;
	ac = devm_kzalloc(dev, sizeof(*ac), GFP_KERNEL);
	if (!ac) {
		dev_err(dev, "kzalloc failed\n");
		return -ENOMEM;
	}

	/* Fill private data */
	ac->max = fw_ac->max;
	if (fw_ac->params) {
		ac->params = (char *) devm_kzalloc(dev, fw_ac->max, GFP_KERNEL);
		if (ac->params == NULL) {
			dev_err(dev, "kzalloc failed\n");
			return -ENOMEM;
		} else {
			memcpy(ac->params, fw_ac->params, fw_ac->max);
		}
	}
	be->pvt_data  = (char *)ac;
	be->pvt_data_len = sizeof(struct hda_sst_algo_data) + ac->max;
	return 0;
}

int hda_sst_fw_kcontrol_find_io(struct device *dev,
		u32 io_type, const struct snd_soc_fw_kcontrol_ops *ops,
		int num_ops, unsigned long sm, unsigned long mc)
{
	int i;

	for (i = 0; i < num_ops; i++) {
		if ((SOC_CONTROL_GET_ID_PUT(ops[i].id) ==
			SOC_CONTROL_GET_ID_PUT(io_type) && ops[i].put)
			&& (SOC_CONTROL_GET_ID_GET(ops[i].id) ==
			SOC_CONTROL_GET_ID_GET(io_type) && ops[i].get)) {
			switch (SOC_CONTROL_GET_ID_PUT(ops[i].id)) {
			case SOC_CONTROL_TYPE_HDA_SST_ALGO_PARAMS:
				hda_sst_copy_algo_control(dev, (struct soc_bytes_ext *)sm,
						(struct snd_soc_fw_bytes_ext *)mc);
				break;
			default:
				break;
			}
		}
	}

	return 0;
}

static struct sst_pipe *hda_sst_add_pipe(struct device *dev,
	struct hda_platform_info *pinfo, struct hda_dfw_pipe *dfw_pipe)
{
	struct sst_pipeline  *ppl;
	struct sst_pipe *pipe;

	list_for_each_entry(ppl, &pinfo->ppl_list, node) {
		if (ppl->pipe->ppl_id == dfw_pipe->pipe_id)
			return ppl->pipe;
	}
	ppl = devm_kzalloc(dev, sizeof(*ppl), GFP_KERNEL);
	if (!ppl) {
		dev_err(dev, "kzalloc block failed\n");
		return NULL;
	}

	pipe  = devm_kzalloc(dev, sizeof(*pipe), GFP_KERNEL);
	if (!ppl) {
		dev_err(dev, "kzalloc block failed\n");
		return NULL;
	}

	pipe->ppl_id = dfw_pipe->pipe_id;
	pipe->memory_pages = dfw_pipe->memory_pages;
	pipe->pipe_type = dfw_pipe->pipe_type;
	pipe->conn_type = dfw_pipe->conn_type;
	pipe->state = INVALID;

	ppl->pipe = pipe;
	list_add(&ppl->node, &pinfo->ppl_list);

	return ppl->pipe;
}

static int hda_sst_widget_load(struct snd_soc_platform *platform,
		struct snd_soc_dapm_widget *w, struct snd_soc_fw_dapm_widget *fw_w)
{
	int ret;
	struct azx *chip  = snd_soc_platform_get_drvdata(platform);
	struct snd_soc_azx *schip =
		container_of(chip, struct snd_soc_azx, hda_azx);
	struct hda_platform_info *pinfo = schip->pinfo;
	struct module_config *mconfig;
	struct sst_pipe *pipe;
	struct hda_dfw_module *dfw_config = (struct hda_dfw_module *)fw_w->pvt_data;

	dev_dbg(chip->dev,
		"In%s pvt_data_len=%d\n", __func__, fw_w->pvt_data_len);
	if (!fw_w->pvt_data_len)
		goto bind_event;

	mconfig = devm_kzalloc(chip->dev, sizeof(*mconfig), GFP_KERNEL);

	if (!mconfig)
		return -ENOMEM;

	dev_dbg(chip->dev, "In%s copying widget private daat\n", __func__);

	dev_dbg(chip->dev,
		"In%s module id = %d\n", __func__,  dfw_config->module_id);
	dev_dbg(chip->dev,
	"In%s module instance = %d\n", __func__,  dfw_config->instance_id);
	dev_dbg(chip->dev, "In%s mcps = %d\n", __func__,  dfw_config->max_mcps);
	dev_dbg(chip->dev, "In%s ibs = %d\n", __func__,  dfw_config->ibs);
	dev_dbg(chip->dev, "In%s obs = %d\n", __func__,  dfw_config->obs);
	dev_dbg(chip->dev, "In%s core_id=%d\n", __func__,  dfw_config->core_id);
	w->priv = (void *)mconfig;
	mconfig->id.module_id = dfw_config->module_id;
	mconfig->id.instance_id = dfw_config->instance_id;
	mconfig->mcps = dfw_config->max_mcps;
	mconfig->ibs = dfw_config->ibs;
	mconfig->obs = dfw_config->obs;
	mconfig->core_id = dfw_config->core_id;
	mconfig->max_in_queue = dfw_config->max_in_queue;
	mconfig->max_out_queue = dfw_config->max_out_queue;
	mconfig->is_loadable = dfw_config->is_loadable;
	mconfig->conn_type = dfw_config->conn_type;
	mconfig->in_fmt.channels  = dfw_config->in_fmt.channels;
	mconfig->in_fmt.sampling_freq = dfw_config->in_fmt.freq;
	mconfig->in_fmt.bit_depth = dfw_config->in_fmt.bit_depth;
	mconfig->in_fmt.valid_bit_depth = dfw_config->in_fmt.valid_bit_depth;
	mconfig->in_fmt.channel_config = dfw_config->in_fmt.ch_cfg;
	mconfig->out_fmt.channels  = dfw_config->out_fmt.channels;
	mconfig->out_fmt.sampling_freq = dfw_config->out_fmt.freq;
	mconfig->out_fmt.bit_depth = dfw_config->out_fmt.bit_depth;
	mconfig->out_fmt.valid_bit_depth = dfw_config->out_fmt.valid_bit_depth;
	mconfig->out_fmt.channel_config = dfw_config->out_fmt.ch_cfg;
	mconfig->params_fixup = dfw_config->params_fixup;
	mconfig->converter = dfw_config->converter;
	pipe =	hda_sst_add_pipe(chip->dev, pinfo, &dfw_config->pipe);


	if (pipe)
		mconfig->pipe = pipe;
	mconfig->dev_type =  dfw_config->dev_type;
	mconfig->hw_conn_type = dfw_config->hw_conn_type;
	mconfig->time_slot =  dfw_config->time_slot;
	mconfig->formats_config.caps_size = dfw_config->caps.caps_size;

	if (mconfig->formats_config.caps_size == 0)
		goto bind_event;

	mconfig->formats_config.caps = (u32 *)devm_kzalloc(chip->dev,
				mconfig->formats_config.caps_size, GFP_KERNEL);

	if (mconfig->formats_config.caps == NULL) {
		dev_err(chip->dev, "kzalloc failed\n");
		return -ENOMEM;
	} else
		memcpy(mconfig->formats_config.caps, dfw_config->caps.caps,
						 dfw_config->caps.caps_size);
	pinfo->resource.max_mcps += mconfig->mcps;
	pinfo->resource.max_mem += mconfig->pipe->memory_pages;

bind_event:
	ret = snd_soc_fw_widget_bind_event(fw_w->event_type, w,
			hda_sst_widget_ops, ARRAY_SIZE(hda_sst_widget_ops));
	if (ret) {
		dev_err(chip->dev, "%s: No matching event handlers found for %d\n",
					__func__, fw_w->event_type);
		return -EINVAL;
	}

	return 0;
}

static int hda_sst_pvt_load(struct snd_soc_platform *platform,
		u32 io_type, unsigned long sm, unsigned long mc)
{
	return hda_sst_fw_kcontrol_find_io(platform->dev, io_type,
			control_ops, ARRAY_SIZE(control_ops), sm, mc);
}


static struct module_config *hda_sst_get_module_by_dir(
	struct snd_soc_dapm_widget *w, int dir, char *m_type)
{
	struct snd_soc_dapm_context *dapm = w->dapm;
	struct snd_soc_dapm_widget *w1 = NULL;
	struct snd_soc_dapm_path *p = NULL;
	struct module_config *mconfig = NULL;

	/* get the source modules  dir = 0 source module, dir = 1 sink modules*/
	if (dir == 0) {
		dev_dbg(dapm->dev, "Stream name=%s\n", w->name);
		if (list_empty(&w->sources))
			return mconfig;

		list_for_each_entry(p, &w->sources, list_sink) {
			if (p->connected && !p->connected(w, p->source) &&
				!is_hda_widget_type(p->source) &&
				(strstr(p->source->name, m_type) == NULL))
				continue;

			if (p->connect && p->source->priv) {
				dev_dbg(dapm->dev, "module widget=%s\n", p->source->name);
				return p->source->priv;
			}
			w1 = p->source;
		}
	} else {
		dev_dbg(dapm->dev, "Stream name=%s\n", w->name);
		if (list_empty(&w->sinks))
			return mconfig;

		list_for_each_entry(p, &w->sinks, list_source) {
			if (p->connected && !p->connected(w, p->sink) &&
				!is_hda_widget_type(p->sink) &&
				(strstr(p->sink->name, m_type) == NULL))
				continue;

			if (p->connect && p->sink->priv) {
				dev_dbg(dapm->dev, "module widget=%s\n", p->sink->name);
				return p->sink->priv;
			}
			w1 = p->sink;
		}
	}
	if (w1 != NULL)
		mconfig = hda_sst_get_module_by_dir(w1, dir, m_type);
	return mconfig;
}

static struct module_config *hda_sst_get_module(struct snd_soc_dai *dai,
	int stream, bool is_fe, char *m_type)
{
	struct snd_soc_dapm_widget *w;
	int dir = 0;

	dev_dbg(dai->dev, "%s: enter, dai-name=%s dir=%d\n", __func__, dai->name, stream);

	/*if FE - Playback, then parse sink list , Capture then source list
	if BE - Playback, then parse source list , Capture then sink list */
	if (stream == SNDRV_PCM_STREAM_PLAYBACK) {
		w = dai->playback_widget;
		(is_fe) ? (dir = 1) : (dir = 0);
	} else {
		w = dai->capture_widget;
		(is_fe) ? (dir = 0) : (dir = 1);
	}
	return hda_sst_get_module_by_dir(w, dir, m_type);
}

static void hda_set_module_params(struct module_config *mconfig,
	struct snd_pcm_hw_params *params, bool is_in_fmt)
{

	struct module_format *format = NULL;

	if (is_in_fmt)
		format = &mconfig->in_fmt;
	else
		format = &mconfig->out_fmt;
	/*set the hw_params */
	format->sampling_freq = params_rate(params);
	format->channels = params_channels(params);
	format->valid_bit_depth = hda_sst_get_bit_depth(params);
	if (format->valid_bit_depth == DEPTH_16BIT)
		format->bit_depth = format->valid_bit_depth;
	else if (format->valid_bit_depth == DEPTH_24BIT)
		format->bit_depth = DEPTH_32BIT;;
	if (is_in_fmt) {
		mconfig->ibs = (format->sampling_freq / 1000) *
				(format->channels) *
				(format->bit_depth >> 3);
	} else {
		mconfig->obs = (format->sampling_freq / 1000) *
				(format->channels) *
				(format->bit_depth >> 3);
	}
}

void hda_sst_set_copier_hw_params(struct snd_soc_dai *dai,
	struct snd_pcm_hw_params *params, int stream, bool is_fe)
{
	struct module_config *mconfig = NULL;
	bool in_fmt;

	dev_dbg(dai->dev,
		"%s: enter, dai-name=%s dir=%d\n", __func__, dai->name, stream);
	if (stream == SNDRV_PCM_STREAM_PLAYBACK)
		in_fmt = true;
	else
		in_fmt = false;
	mconfig = hda_sst_get_module(dai, stream, is_fe, "cpr");
	if (mconfig != NULL)
		hda_set_module_params(mconfig, params, in_fmt);
	return;
}

void hda_sst_set_copier_dma_id(struct snd_soc_dai *dai, int dma_id, int stream,
		bool is_fe)
{
	struct module_config *mconfig = NULL;

	dev_dbg(dai->dev,
		 "%s: enter, dai-name=%s dir=%d\n", __func__,
		dai->name, stream);
	mconfig = hda_sst_get_module(dai, stream, is_fe, "cpr");
	if (mconfig != NULL)
		mconfig->dma_id = dma_id;
	return;
}

/*set BE copier I2s,DMIC, SLIMBUS config*/
void hda_sst_set_be_copier_caps(struct snd_soc_dai *dai,
	struct specific_config *configs, int stream)
{
	struct module_config *mconfig = NULL;

	dev_dbg(dai->dev, "%s: enter, dai-name=%s\n", __func__, dai->name);
	mconfig = hda_sst_get_module(dai, stream, false, "cpr");
	if (mconfig != NULL && configs->caps_size != 0) {
		memcpy(mconfig->formats_config.caps,
		configs->caps,
		configs->caps_size);

		mconfig->formats_config.caps_size = configs->caps_size;
	}
	return;
}

void hda_sst_set_be_dmic_config(struct snd_soc_dai *dai,
	struct snd_pcm_hw_params *params, int stream)
{
	struct module_config *mconfig = NULL;
	u32 outctrl;

	dev_dbg(dai->dev, "%s: enter, dai-name=%s\n", __func__, dai->name);
	mconfig = hda_sst_get_module(dai, stream, false, "cpr");
	if (mconfig != NULL && mconfig->formats_config.caps_size != 0) {
		/*FIXME need to fix based on the FW dmic interface struct.
		the parameter to set here should to set the DMIC mode.
		currenlt;y using timeslot */
		if (strncmp(dai->name, "DMIC23 Pin", strlen(dai->name)) == 0) {
			mconfig->time_slot = 1;
			outctrl = mconfig->formats_config.caps[3];
		} else
			outctrl = mconfig->formats_config.caps[2];

		if (hda_sst_get_bit_depth(params)  == DEPTH_16BIT)
			outctrl &= ~BIT(19);
		else if (hda_sst_get_bit_depth(params) == DEPTH_24BIT)
			outctrl |= BIT(19);
		if (strncmp(dai->name, "DMIC23 Pin", strlen(dai->name)) == 0)
			mconfig->formats_config.caps[3] = outctrl;
		else
			mconfig->formats_config.caps[2] = outctrl;
		dev_dbg(dai->dev, "%s: outctrl =%x\n", __func__, outctrl);
		hda_set_module_params(mconfig, params, true);

		/*FIXME need to fix based on the FW dmic interface struct.
		the parameter to set here should to set the DMIC mode.
		currenlt;y using timeslot */
		if (strncmp(dai->name, "DMIC23 Pin", strlen(dai->name)) == 0)
			mconfig->time_slot = 1;

	}
}

int hda_sst_set_fe_pipeline_state(struct snd_soc_dai *dai, bool start,
		 int stream)
{
	struct azx *chip = dev_get_drvdata(dai->dev);
	struct snd_soc_azx *schip =
			container_of(chip, struct snd_soc_azx, hda_azx);
	struct sst_dsp_ctx *ctx = schip->dsp;

	struct module_config *mconfig = NULL;
	int ret = 0;

	dev_dbg(dai->dev, "%s: enter, dai-name=%s dir=%d\n", __func__,
		 dai->name, stream);
	mconfig = hda_sst_get_module(dai, stream, true, "cpr");
	if (mconfig != NULL) {
		if (start)
			ret = hda_sst_run_pipe(ctx, mconfig->pipe);
		else
			ret = hda_sst_stop_pipe(ctx, mconfig->pipe);
	}

	return ret;
}

static struct snd_soc_fw_platform_ops soc_fw_ops = {
	.widget_load = hda_sst_widget_load,
	.pvt_load = hda_sst_pvt_load,
	.io_ops = control_ops,
	.io_ops_count = ARRAY_SIZE(control_ops),
};

int hda_sst_dsp_control_init(struct snd_soc_platform *platform,
		 struct azx *chip)
{
	int ret = 0;
	const struct firmware *fw;
	struct snd_soc_azx *schip =
		container_of(chip, struct snd_soc_azx, hda_azx);
	struct hda_platform_info *pinfo = schip->pinfo;

	dev_dbg(chip->dev, "In%s\n", __func__);

	pinfo->widget = devm_kzalloc(chip->dev,
				   HDA_SST_NUM_WIDGETS * sizeof(*pinfo->widget),
				   GFP_KERNEL);
	if (!pinfo->widget) {
		dev_err(chip->dev, "%s: kzalloc failed\n", __func__);
		return -ENOMEM;
	}

	dev_dbg(chip->dev, "In%s req firmware topology bin\n",  __func__);
	ret = request_firmware(&fw, "dfw_sst.bin", chip->dev);
	if (fw == NULL) {
		dev_err(chip->dev, "config firmware request failed with %d\n", ret);
		return ret;
	}

	dev_dbg(chip->dev, "In%s soc fw load_platform\n", __func__);
	/* Index is for each config load */
	ret = snd_soc_fw_load_platform(platform, &soc_fw_ops, fw, 0);
	if (ret < 0) {
		dev_err(chip->dev, "Control load failed%d\n", ret);
		return -EINVAL;
	}
	return ret;
}
