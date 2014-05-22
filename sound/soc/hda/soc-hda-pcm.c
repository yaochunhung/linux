/*
 *  soc-hda-pcm.c -ASOC HDA Platform driver file implementing PCM functionality
 *
 *  Copyright (C) 2014 Intel Corp
 *  Author:  Jeeja KP <jeeja.kp@intel.com>
 *
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
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/pci.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/hda_controller.h>
#include <sound/soc-hda-dma.h>
#include "soc-hda-controller.h"

inline struct azx_dev *
soc_azx_assign_device(struct azx *chip, struct snd_pcm_substream *substream)
{
	int dev, i, nums;
	struct azx_dev *res = NULL;
	/* make a non-zero unique key for the substream */
	int key = (substream->pcm->device << 16) | (substream->number << 2) |
			(substream->stream + 1);

	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK) {
		dev = chip->playback_index_offset;
		nums = chip->playback_streams;
	} else {
		dev = chip->capture_index_offset;
		nums = chip->capture_streams;
	}
	for (i = 0; i < nums; i++, dev++) {
		struct azx_dev *azx_dev = &chip->azx_dev[dev];
		if (!azx_dev->opened) {
			if (azx_dev->assigned_key == key) {
				azx_dev->opened = 1;
				azx_dev->assigned_key = key;
				return azx_dev;
			}
			if (!res)
				res = azx_dev;
		}
	}
	if (res) {
		res->opened = 1;
		res->assigned_key = key;
	}
	return res;
}

/* release the assigned stream */
static inline void soc_azx_release_device(struct azx_dev *azx_dev)
{
	azx_dev->opened = 0;
}

static struct azx *get_chip_ctx(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = snd_pcm_substream_chip(substream);
	struct snd_soc_platform *platform = rtd->platform;
	struct azx *chip = dev_get_drvdata(platform->dev);
	return chip;
}

/*
 * PCM support
 */
static int soc_hda_pcm_open(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct azx *chip = get_chip_ctx(substream);
	struct azx_dev *azx_dev;
	struct snd_pcm_runtime *runtime = substream->runtime;
	unsigned long flags;
	struct snd_soc_hda_dma_params *dma_params;

	dev_dbg(chip->dev, "%s: %s\n", __func__, dai->name);
	mutex_lock(&chip->open_mutex);
	azx_dev = soc_azx_assign_device(chip, substream);
	if (azx_dev == NULL) {
		mutex_unlock(&chip->open_mutex);
		return -EBUSY;
	}

	azx_set_pcm_constrains(chip, runtime);

	/* disable WALLCLOCK timestamps for capture streams
	   until we figure out how to handle digital inputs */
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		runtime->hw.info &= ~SNDRV_PCM_INFO_HAS_WALL_CLOCK;

	spin_lock_irqsave(&chip->reg_lock, flags);
	azx_dev->substream = substream;
	azx_dev->running = 0;
	spin_unlock_irqrestore(&chip->reg_lock, flags);
	runtime->private_data = azx_dev;

	dma_params = kzalloc(sizeof(*dma_params), GFP_KERNEL);
	dma_params->stream_tag = azx_dev->stream_tag;
	snd_soc_dai_set_dma_data(dai, substream, (void *)dma_params);

	dev_dbg(chip->dev, "stream tag set in dma params=%d\n",
				 dma_params->stream_tag);
	snd_pcm_set_sync(substream);
	mutex_unlock(&chip->open_mutex);
	return 0;
}
static int soc_hda_pcm_prepare(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct snd_soc_pcm_runtime *rtd = snd_pcm_substream_chip(substream);
	struct azx *chip = get_chip_ctx(substream);
	struct azx_dev *azx_dev = get_azx_dev(substream);
	unsigned int format_val;
	int err;
	struct snd_soc_hda_dma_params *dma_params;
	struct snd_soc_dai *codec_dai = rtd->codec_dai;

	dev_dbg(chip->dev, "%s: %s\n", __func__, dai->name);
	if (azx_dev->prepared) {
		dev_dbg(chip->dev, "already stream is prepared - returning\n");
		return 0;
	}

	dma_params  = (struct snd_soc_hda_dma_params *)
			snd_soc_dai_get_dma_data(codec_dai, substream);
	format_val = dma_params->format;

	dev_dbg(chip->dev, "stream_tag=%d formatvalue=%d\n",
				azx_dev->stream_tag, format_val);
	azx_stream_reset(chip, azx_dev);

	err = azx_set_device_params(chip, substream, format_val);

	if (!err)
		azx_dev->prepared = 1;
	return err;
}

static int soc_hda_pcm_hw_params(struct snd_pcm_substream *substream,
				struct snd_pcm_hw_params *params,
				struct snd_soc_dai *dai)
{
	struct azx *chip = get_chip_ctx(substream);
	int ret;
	struct snd_pcm_runtime *runtime = substream->runtime;

	dev_dbg(chip->dev, "%s: %s\n", __func__, dai->name);
	ret = chip->ops->substream_alloc_pages(chip, substream,
					  params_buffer_bytes(params));

	memset(substream->runtime->dma_area, 0, params_buffer_bytes(params));

	dev_dbg(chip->dev, "format_val, rate=%d, ch=%d, format=%d\n",
			runtime->rate, runtime->channels, runtime->format);

	return ret;
}

static void soc_hda_pcm_close(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct azx *chip = get_chip_ctx(substream);
	struct azx_dev *azx_dev = get_azx_dev(substream);
	unsigned long flags;
	struct snd_soc_hda_dma_params *dma_params;

	dev_dbg(chip->dev, "%s: %s\n", __func__, dai->name);
	mutex_lock(&chip->open_mutex);
	spin_lock_irqsave(&chip->reg_lock, flags);
	azx_dev->substream = NULL;
	azx_dev->running = 0;
	spin_unlock_irqrestore(&chip->reg_lock, flags);
	soc_azx_release_device(azx_dev);
	dma_params  = (struct snd_soc_hda_dma_params *)
			snd_soc_dai_get_dma_data(dai, substream);
	kfree(dma_params);
	mutex_unlock(&chip->open_mutex);
}

static int soc_hda_pcm_hw_free(struct snd_pcm_substream *substream,
		struct snd_soc_dai *dai)
{
	struct azx *chip = get_chip_ctx(substream);
	struct azx_dev *azx_dev = get_azx_dev(substream);

	dev_dbg(chip->dev, "%s: %s\n", __func__, dai->name);

	/*reset BDL address */
	azx_reset_device(chip, azx_dev);

	azx_dev->prepared = 0;

	return chip->ops->substream_free_pages(chip, substream);
}

static struct snd_soc_dai_ops hda_pcm_dai_ops = {
	.startup = soc_hda_pcm_open,
	.shutdown = soc_hda_pcm_close,
	.prepare = soc_hda_pcm_prepare,
	.hw_params = soc_hda_pcm_hw_params,
	.hw_free = soc_hda_pcm_hw_free,
};

static struct snd_soc_dai_driver soc_hda_platform_dai[] = {
{
	.name = "System Pin",
	.ops = &hda_pcm_dai_ops,
	.playback = {
		.stream_name = "System Playback",
		.channels_min = 2,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
	.capture = {
		.stream_name = "System Capture",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE,
	},
},
};

static int soc_hda_platform_open(struct snd_pcm_substream *substream)
{
	struct snd_pcm_runtime *runtime;
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *dai_link = rtd->dai_link;

	dev_dbg(rtd->platform->dev, "In %s:%s\n", __func__,
					dai_link->cpu_dai_name);

	runtime = substream->runtime;
	runtime->hw = azx_pcm_hw;
	return 0;
}

static int soc_hda_platform_close(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_dai_link *dai_link = rtd->dai_link;

	dev_dbg(rtd->platform->dev, "In %s:%s\n", __func__,
					dai_link->cpu_dai_name);

	return 0;
}

static cycle_t soc_hda_cc_read(const struct cyclecounter *cc)
{
	struct azx_dev *azx_dev =
		container_of(cc, struct azx_dev, azx_cc);
	struct snd_pcm_substream *substream = azx_dev->substream;
	struct azx *chip = get_chip_ctx(substream);

	return azx_readl(chip, WALLCLK);
}

void azx_timecounter_init(struct snd_pcm_substream *substream,
					bool force, cycle_t last)
{
	struct azx_dev *azx_dev = get_azx_dev(substream);
	struct timecounter *tc = &azx_dev->azx_tc;
	struct cyclecounter *cc = &azx_dev->azx_cc;
	u64 nsec;

	cc->read = soc_hda_cc_read;
	cc->mask = CLOCKSOURCE_MASK(32);

	/*
	* Converting from 24 MHz to ns means applying a 125/3 factor.
	* To avoid any saturation issues in intermediate operations,
	* the 125 factor is applied first. The division is applied
	* last after reading the timecounter value.
	* Applying the 1/3 factor as part of the multiplication
	* requires at least 20 bits for a decent precision, however
	* overflows occur after about 4 hours or less, not a option.
	*/

	cc->mult = 125; /* saturation after 195 years */
	cc->shift = 0;

	nsec = 0; /* audio time is elapsed time since trigger */
	timecounter_init(tc, cc, nsec);
	if (force)
		/*
		* force timecounter to use predefined value,
		* used for synchronized starts
		*/
		tc->cycle_last = last;
	}

static int soc_hda_platform_pcm_trigger(struct snd_pcm_substream *substream,
					int cmd)
{
	struct azx *chip = get_chip_ctx(substream);
	struct azx_dev *azx_dev;
	struct snd_pcm_substream *s;
	int rstart = 0, start, nsync = 0, sbits = 0;
	int nwait, timeout;

	azx_dev = get_azx_dev(substream);
	/*FIXME trace_azx_pcm_trigger(chip, azx_dev, cmd);*/

	dev_dbg(chip->dev, "In %s cmd=%d\n", __func__, cmd);
	if (!azx_dev->prepared)
		return -EPIPE;
	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		rstart = 1;
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
	case SNDRV_PCM_TRIGGER_RESUME:
		start = 1;
		break;
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_STOP:
		start = 0;
		break;
	default:
	return -EINVAL;
	}
	snd_pcm_group_for_each_entry(s, substream) {
		if (s->pcm->card != substream->pcm->card)
			continue;
		azx_dev = get_azx_dev(s);
		sbits |= 1 << azx_dev->index;
		nsync++;
		snd_pcm_trigger_done(s, substream);
	}

	spin_lock(&chip->reg_lock);

	/* first, set SYNC bits of corresponding streams */
	if (chip->driver_caps & AZX_DCAPS_OLD_SSYNC)
		azx_writel(chip, OLD_SSYNC,
			azx_readl(chip, OLD_SSYNC) | sbits);
	else
		azx_writel(chip, SSYNC, azx_readl(chip, SSYNC) | sbits);

	snd_pcm_group_for_each_entry(s, substream) {
		if (s->pcm->card != substream->pcm->card)
			continue;
		azx_dev = get_azx_dev(s);
		if (start) {
			azx_dev->start_wallclk = azx_readl(chip, WALLCLK);
			if (!rstart)
				azx_dev->start_wallclk -=
						azx_dev->period_wallclk;
			azx_stream_start(chip, azx_dev);
		} else {
			azx_stream_stop(chip, azx_dev);
		}
		azx_dev->running = start;
	}
	spin_unlock(&chip->reg_lock);
	if (start) {
		/* wait until all FIFOs get ready */
		for (timeout = 5000; timeout; timeout--) {
			nwait = 0;
			snd_pcm_group_for_each_entry(s, substream) {
				if (s->pcm->card != substream->pcm->card)
					continue;
				azx_dev = get_azx_dev(s);
				if (!(azx_sd_readb(chip, azx_dev, SD_STS) &
				      SD_STS_FIFO_READY))
					nwait++;
			}
			if (!nwait)
				break;
			cpu_relax();
		}
	} else {
		/* wait until all RUN bits are cleared */
		for (timeout = 5000; timeout; timeout--) {
			nwait = 0;
			snd_pcm_group_for_each_entry(s, substream) {
				if (s->pcm->card != substream->pcm->card)
					continue;
				azx_dev = get_azx_dev(s);
				if (azx_sd_readb(chip, azx_dev, SD_CTL) &
				    SD_CTL_DMA_START)
					nwait++;
			}
			if (!nwait)
				break;
			cpu_relax();
		}
	}
	spin_lock(&chip->reg_lock);
	/* reset SYNC bits */
	if (chip->driver_caps & AZX_DCAPS_OLD_SSYNC)
		azx_writel(chip, OLD_SSYNC,
				azx_readl(chip, OLD_SSYNC) & ~sbits);
	else
		azx_writel(chip, SSYNC, azx_readl(chip, SSYNC) & ~sbits);

	if (start) {
		azx_timecounter_init(substream, 0, 0);
		if (nsync > 1) {
			cycle_t cycle_last;

			/* same start cycle for master and group */
			azx_dev = get_azx_dev(substream);
			cycle_last = azx_dev->azx_tc.cycle_last;
			snd_pcm_group_for_each_entry(s, substream) {
				if (s->pcm->card != substream->pcm->card)
					continue;
				azx_timecounter_init(s, 1, cycle_last);
			}
		}
	}
	spin_unlock(&chip->reg_lock);
	return 0;
}

static snd_pcm_uframes_t soc_hda_platform_pcm_pointer
			(struct snd_pcm_substream *substream)
{
	struct azx *chip = get_chip_ctx(substream);
	struct azx_dev *azx_dev = get_azx_dev(substream);

	return bytes_to_frames(substream->runtime,
			       azx_get_position(chip, azx_dev, false));
}

#ifdef CONFIG_X86
static int soc_hda_platform_pcm_mmap(struct snd_pcm_substream *substream,
			struct vm_area_struct *area)
{
	struct azx *chip = get_chip_ctx(substream);

#ifdef CONFIG_X86
	if (!azx_snoop(chip))
		area->vm_page_prot = pgprot_writecombine(area->vm_page_prot);
#endif
	return snd_pcm_lib_default_mmap(substream, area);
}
#else
#define soc_azx_pcm_mmap	NULL
#endif

static int soc_hda_platform_get_wallclock_tstamp(struct snd_pcm_substream *substream,
				struct timespec *ts)
{
	struct azx_dev *azx_dev = get_azx_dev(substream);
	u64 nsec;

	nsec = timecounter_read(&azx_dev->azx_tc);
	nsec = div_u64(nsec, 3); /* can be optimized */
	/*FIXME nsec = azx_adjust_codec_delay(substream, nsec);*/

	*ts = ns_to_timespec(nsec);

	return 0;
}

static struct snd_pcm_ops soc_hda_platform_ops = {
	.open = soc_hda_platform_open,
	.close = soc_hda_platform_close,
	.ioctl = snd_pcm_lib_ioctl,
	.trigger = soc_hda_platform_pcm_trigger,
	.pointer = soc_hda_platform_pcm_pointer,
	.wall_clock = soc_hda_platform_get_wallclock_tstamp,
	.mmap = soc_hda_platform_pcm_mmap,
	.page = snd_pcm_sgbuf_ops_page,
};

static void soc_hda_pcm_free(struct snd_pcm *pcm)
{
	snd_pcm_lib_preallocate_free_for_all(pcm);
}

#define MAX_PREALLOC_SIZE	(32 * 1024 * 1024)

static int soc_hda_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_platform *platform = rtd->platform;
	struct azx *chip = dev_get_drvdata(platform->dev);
	struct snd_soc_dai *dai = rtd->cpu_dai;
	struct snd_pcm *pcm = rtd->pcm;
	unsigned int size;

	int retval = 0;

	dev_dbg(chip->dev, "In %s\n", __func__);
	if (dai->driver->playback.channels_min ||
			dai->driver->capture.channels_min) {
			/* buffer pre-allocation */
			size = CONFIG_SND_HDA_PREALLOC_SIZE * 1024;
			if (size > MAX_PREALLOC_SIZE)
				size = MAX_PREALLOC_SIZE;
			retval = snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV_SG,
					      snd_dma_pci_data(chip->pci),
					      size, MAX_PREALLOC_SIZE);
		if (retval) {
			dev_err(chip->dev, "dma buffer allocationf fail\n");
			return retval;
		}
	}
	return retval;
}

static int soc_hda_soc_probe(struct snd_soc_platform *platform)
{
	dev_dbg(platform->dev, "Enter:%s\n", __func__);
	/*need to load controls here */
	return 0;
}

static int soc_hda_soc_remove(struct snd_soc_platform *platform)
{
	dev_dbg(platform->dev, "Enter:%s\n", __func__);
	return 0;
}

unsigned int soc_hda_soc_read(struct snd_soc_platform *platform,
				unsigned int reg)
{
	dev_dbg(platform->dev, "Enter:%s\n", __func__);
	return 0;
}

int soc_hda_soc_write(struct snd_soc_platform *platform,
			unsigned int reg, unsigned int val)
{
	dev_dbg(platform->dev, "Enter:%s\n", __func__);
	return 0;
}

static struct snd_soc_platform_driver soc_hda_platform_drv  = {
	.probe		= soc_hda_soc_probe,
	.remove		= soc_hda_soc_remove,
	.ops		= &soc_hda_platform_ops,
	.pcm_new	= soc_hda_pcm_new,
	.pcm_free	= soc_hda_pcm_free,
	.read		= soc_hda_soc_read,
	.write		= soc_hda_soc_write,
};

static const struct snd_soc_component_driver soc_hda_component = {
	.name           = "pcm",
};


int soc_hda_platform_register(struct device *dev)
{
	int ret = 0;

	ret = snd_soc_register_platform(dev,
					 &soc_hda_platform_drv);
	if (ret) {
		dev_err(dev, "registering soc platform failed\n");
		return ret;
	}
	ret = snd_soc_register_component(dev, &soc_hda_component,
				soc_hda_platform_dai,
				ARRAY_SIZE(soc_hda_platform_dai));
	if (ret) {
		dev_err(dev, "registering cpu dais failed\n");
		snd_soc_unregister_platform(dev);
	}

	return ret;

}

int soc_hda_platform_unregister(struct device *dev)
{

	snd_soc_unregister_component(dev);
	snd_soc_unregister_platform(dev);
	return 0;
}
