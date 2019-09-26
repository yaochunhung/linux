// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
//
// This file is provided under a dual BSD/GPLv2 license.  When using or
// redistributing this file, you may do so under either license.
//
// Copyright(c) 2018 Intel Corporation. All rights reserved.
//
// Author: Liam Girdwood <liam.r.girdwood@linux.intel.com>
//
// PCM Layer, interface between ALSA and IPC.
//

#include <linux/pm_runtime.h>
#include <sound/pcm_params.h>
#include <sound/sof.h>
#include "sof-priv.h"
#include "ops.h"
#include "ipc-ops.h"

/*
 * sof pcm period elapse work
 */
static void sof_pcm_period_elapsed_work(struct work_struct *work)
{
	struct snd_sof_pcm_stream *sps =
		container_of(work, struct snd_sof_pcm_stream,
			     period_elapsed_work);

	snd_pcm_period_elapsed(sps->substream);
}

/*
 * sof pcm period elapse, this could be called at irq thread context.
 */
void snd_sof_pcm_period_elapsed(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_component *component =
		snd_soc_rtdcom_lookup(rtd, SOF_COMPONENT_NAME);
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(component);
	struct snd_sof_pcm *spcm;

	spcm = snd_sof_find_spcm_dai(sdev, rtd);
	if (!spcm) {
		dev_err(sdev->dev,
			"error: period elapsed for unknown stream!\n");
		return;
	}

	/*
	 * snd_pcm_period_elapsed() can be called in interrupt context
	 * before IRQ_HANDLED is returned. Inside snd_pcm_period_elapsed(),
	 * when the PCM is done draining or xrun happened, a STOP IPC will
	 * then be sent and this IPC will hit IPC timeout.
	 * To avoid sending IPC before the previous IPC is handled, we
	 * schedule delayed work here to call the snd_pcm_period_elapsed().
	 */
	schedule_work(&spcm->stream[substream->stream].period_elapsed_work);
}
EXPORT_SYMBOL(snd_sof_pcm_period_elapsed);

/* this may get called several times by oss emulation */
static int sof_pcm_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_component *component =
		snd_soc_rtdcom_lookup(rtd, SOF_COMPONENT_NAME);
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(component);
	struct snd_sof_pcm *spcm;
	int ret;
	int new_buffer;

	/* nothing to do for BE */
	if (rtd->dai_link->no_pcm)
		return 0;

	spcm = snd_sof_find_spcm_dai(sdev, rtd);
	if (!spcm)
		return -EINVAL;

	dev_dbg(sdev->dev, "pcm: hw params stream %d dir %d\n",
		spcm->pcm.pcm_id, substream->stream);

	/* allocate audio buffer pages */
	new_buffer = snd_pcm_lib_malloc_pages(substream, params_buffer_bytes(params));
	if (new_buffer < 0) {
		dev_err(sdev->dev, "error: could not allocate %d bytes for PCM %d\n",
			params_buffer_bytes(params), spcm->pcm.pcm_id);
		return new_buffer;
	}

	ret = snd_sof_pcm_hw_params(sdev, substream, params, spcm, new_buffer);
	if (ret < 0) {
		dev_err(sdev->dev, "error: hw_params config failed\n");
		return ret;
	}

	spcm->prepared[substream->stream] = true;

	/* save pcm hw_params */
	memcpy(&spcm->params[substream->stream], params, sizeof(*params));

	return ret;
}

static int sof_pcm_free(struct snd_pcm_substream *substream,
			struct snd_sof_dev *sdev,
			struct snd_sof_pcm *spcm)
{
	int ret;

	ret = snd_sof_pcm_free(sdev, substream, spcm);
	if (!ret)
		spcm->prepared[substream->stream] = false;

	return ret;
}

static int sof_pcm_hw_free(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_component *component =
		snd_soc_rtdcom_lookup(rtd, SOF_COMPONENT_NAME);
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(component);
	struct snd_sof_pcm *spcm;
	int ret, err = 0;

	/* nothing to do for BE */
	if (rtd->dai_link->no_pcm)
		return 0;

	spcm = snd_sof_find_spcm_dai(sdev, rtd);
	if (!spcm)
		return -EINVAL;

	dev_dbg(sdev->dev, "pcm: free stream %d dir %d\n", spcm->pcm.pcm_id,
		substream->stream);

	if (spcm->prepared[substream->stream]) {
		ret = sof_pcm_free(substream, sdev, spcm);
		if (ret < 0)
			err = ret;
	}

	snd_pcm_lib_free_pages(substream);

	cancel_work_sync(&spcm->stream[substream->stream].period_elapsed_work);

	ret = snd_sof_pcm_platform_hw_free(sdev, substream);
	if (ret < 0) {
		dev_err(sdev->dev, "error: platform hw free failed\n");
		err = ret;
	}

	return err;
}

/* can be called externally to restart paused or suspended streams */
int snd_sof_pcm_prepare(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_component *component =
		snd_soc_rtdcom_lookup(rtd, SOF_COMPONENT_NAME);
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(component);
	struct snd_sof_pcm *spcm;
	int ret;

	/* nothing to do for BE */
	if (rtd->dai_link->no_pcm)
		return 0;

	spcm = snd_sof_find_spcm_dai(sdev, rtd);
	if (!spcm)
		return -EINVAL;

	if (spcm->prepared[substream->stream])
		return 0;

	dev_dbg(sdev->dev, "pcm: prepare stream %d dir %d\n", spcm->pcm.pcm_id,
		substream->stream);

	/* set hw_params */
	ret = sof_pcm_hw_params(substream, &spcm->params[substream->stream]);
	if (ret < 0) {
		dev_err(sdev->dev, "error: set pcm hw_params after resume\n");
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL(snd_sof_pcm_prepare);

/*
 * FE dai link trigger actions are always executed in non-atomic context because
 * they involve IPC's.
 */
static int sof_pcm_trigger(struct snd_pcm_substream *substream, int cmd)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_component *component =
		snd_soc_rtdcom_lookup(rtd, SOF_COMPONENT_NAME);
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(component);
	struct snd_sof_pcm *spcm;
	bool reset_hw_params = false;
	int ret;

	/* nothing to do for BE */
	if (rtd->dai_link->no_pcm)
		return 0;

	spcm = snd_sof_find_spcm_dai(sdev, rtd);
	if (!spcm)
		return -EINVAL;

	dev_dbg(sdev->dev, "pcm: trigger stream %d dir %d cmd %d\n",
		spcm->pcm.pcm_id, substream->stream, cmd);

	/* free PCM if reset_hw_params is set and the STOP IPC is successful */
	ret = snd_sof_pcm_trigger(sdev, substream, spcm, cmd, &reset_hw_params);
	if (!ret && reset_hw_params)
		ret = sof_pcm_free(substream, sdev, spcm);

	return ret;
}

static snd_pcm_uframes_t sof_pcm_pointer(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_component *component =
		snd_soc_rtdcom_lookup(rtd, SOF_COMPONENT_NAME);
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(component);
	struct snd_sof_pcm *spcm;
	snd_pcm_uframes_t host, dai;

	/* nothing to do for BE */
	if (rtd->dai_link->no_pcm)
		return 0;

	/* use dsp ops pointer callback directly if set */
	if (sof_ops(sdev)->pcm_pointer)
		return sof_ops(sdev)->pcm_pointer(sdev, substream);

	spcm = snd_sof_find_spcm_dai(sdev, rtd);
	if (!spcm)
		return -EINVAL;

	/* read position from DSP */
	host = bytes_to_frames(substream->runtime,
			       snd_sof_host_posn_bytes(sdev,
			       spcm->stream[substream->stream].ipc_posn));
	dai = bytes_to_frames(substream->runtime, snd_sof_host_posn_bytes(sdev,
			      spcm->stream[substream->stream].ipc_posn));

	dev_dbg(sdev->dev, "PCM: stream %d dir %d DMA position %lu DAI position %lu\n",
		spcm->pcm.pcm_id, substream->stream, host, dai);

	return host;
}

static int sof_pcm_open(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_component *component =
		snd_soc_rtdcom_lookup(rtd, SOF_COMPONENT_NAME);
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(component);
	struct snd_sof_pcm *spcm;
	struct snd_soc_tplg_stream_caps *caps;
	int ret;

	/* nothing to do for BE */
	if (rtd->dai_link->no_pcm)
		return 0;

	spcm = snd_sof_find_spcm_dai(sdev, rtd);
	if (!spcm)
		return -EINVAL;

	dev_dbg(sdev->dev, "pcm: open stream %d dir %d\n", spcm->pcm.pcm_id,
		substream->stream);

	INIT_WORK(&spcm->stream[substream->stream].period_elapsed_work,
		  sof_pcm_period_elapsed_work);

	caps = &spcm->pcm.caps[substream->stream];

	/* set any runtime constraints based on topology */
	snd_pcm_hw_constraint_step(substream->runtime, 0,
				   SNDRV_PCM_HW_PARAM_BUFFER_BYTES,
				   le32_to_cpu(caps->period_size_min));
	snd_pcm_hw_constraint_step(substream->runtime, 0,
				   SNDRV_PCM_HW_PARAM_PERIOD_BYTES,
				   le32_to_cpu(caps->period_size_min));

	/* set runtime config */
	runtime->hw.info = SNDRV_PCM_INFO_MMAP |
			  SNDRV_PCM_INFO_MMAP_VALID |
			  SNDRV_PCM_INFO_INTERLEAVED |
			  SNDRV_PCM_INFO_PAUSE |
			  SNDRV_PCM_INFO_NO_PERIOD_WAKEUP;
	runtime->hw.formats = le64_to_cpu(caps->formats);
	runtime->hw.period_bytes_min = le32_to_cpu(caps->period_size_min);
	runtime->hw.period_bytes_max = le32_to_cpu(caps->period_size_max);
	runtime->hw.periods_min = le32_to_cpu(caps->periods_min);
	runtime->hw.periods_max = le32_to_cpu(caps->periods_max);

	/*
	 * caps->buffer_size_min is not used since the
	 * snd_pcm_hardware structure only defines buffer_bytes_max
	 */
	runtime->hw.buffer_bytes_max = le32_to_cpu(caps->buffer_size_max);

	dev_dbg(sdev->dev, "period min %zd max %zd bytes\n",
		runtime->hw.period_bytes_min,
		runtime->hw.period_bytes_max);
	dev_dbg(sdev->dev, "period count %d max %d\n",
		runtime->hw.periods_min,
		runtime->hw.periods_max);
	dev_dbg(sdev->dev, "buffer max %zd bytes\n",
		runtime->hw.buffer_bytes_max);

	/* set wait time - TODO: come from topology */
	substream->wait_time = 500;

	ret = snd_sof_pcm_open(sdev, substream, spcm);
	if (ret < 0)
		dev_err(sdev->dev, "error: ipc pcm open failed %d\n", ret);

	spcm->stream[substream->stream].substream = substream;
	spcm->prepared[substream->stream] = false;

	ret = snd_sof_pcm_platform_open(sdev, substream);
	if (ret < 0)
		dev_err(sdev->dev, "error: pcm open failed %d\n", ret);

	return ret;
}

static int sof_pcm_close(struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = substream->private_data;
	struct snd_soc_component *component =
		snd_soc_rtdcom_lookup(rtd, SOF_COMPONENT_NAME);
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(component);
	struct snd_sof_pcm *spcm;
	int err;
	int ret;

	/* nothing to do for BE */
	if (rtd->dai_link->no_pcm)
		return 0;

	spcm = snd_sof_find_spcm_dai(sdev, rtd);
	if (!spcm)
		return -EINVAL;

	dev_dbg(sdev->dev, "pcm: close stream %d dir %d\n", spcm->pcm.pcm_id,
		substream->stream);

	err = snd_sof_pcm_platform_close(sdev, substream);
	if (err < 0) {
		dev_err(sdev->dev, "error: pcm close failed %d\n",
			err);
		/*
		 * keep going, no point in preventing the close
		 * from happening
		 */
	}

	ret = snd_sof_pcm_close(sdev, substream, spcm);
	if (ret < 0)
		dev_err(sdev->dev, "error: ipc pcm closed failed %d\n", ret);

	return 0;
}

static struct snd_pcm_ops sof_pcm_ops = {
	.open		= sof_pcm_open,
	.close		= sof_pcm_close,
	.ioctl		= snd_pcm_lib_ioctl,
	.hw_params	= sof_pcm_hw_params,
	.prepare	= snd_sof_pcm_prepare,
	.hw_free	= sof_pcm_hw_free,
	.trigger	= sof_pcm_trigger,
	.pointer	= sof_pcm_pointer,
	.page		= snd_pcm_sgbuf_ops_page,
};

/*
 * Pre-allocate playback/capture audio buffer pages.
 * no need to explicitly release memory preallocated by sof_pcm_new in pcm_free
 * snd_pcm_lib_preallocate_free_for_all() is called by the core.
 */
static int sof_pcm_new(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *component =
		snd_soc_rtdcom_lookup(rtd, SOF_COMPONENT_NAME);
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(component);
	struct snd_sof_pcm *spcm;
	struct snd_pcm *pcm = rtd->pcm;
	struct snd_soc_tplg_stream_caps *caps;
	int stream = SNDRV_PCM_STREAM_PLAYBACK;

	/* find SOF PCM for this RTD */
	spcm = snd_sof_find_spcm_dai(sdev, rtd);
	if (!spcm) {
		dev_warn(sdev->dev, "warn: can't find PCM with DAI ID %d\n",
			 rtd->dai_link->id);
		return 0;
	}

	dev_dbg(sdev->dev, "creating new PCM %s\n", spcm->pcm.pcm_name);

	/* do we need to pre-allocate playback audio buffer pages */
	if (!spcm->pcm.playback)
		goto capture;

	caps = &spcm->pcm.caps[stream];

	/* pre-allocate playback audio buffer pages */
	dev_dbg(sdev->dev, "spcm: allocate %s playback DMA buffer size 0x%x max 0x%x\n",
		caps->name, caps->buffer_size_min, caps->buffer_size_max);

	snd_pcm_lib_preallocate_pages(pcm->streams[stream].substream,
				      SNDRV_DMA_TYPE_DEV_SG, sdev->dev,
				      le32_to_cpu(caps->buffer_size_min),
				      le32_to_cpu(caps->buffer_size_max));
capture:
	stream = SNDRV_PCM_STREAM_CAPTURE;

	/* do we need to pre-allocate capture audio buffer pages */
	if (!spcm->pcm.capture)
		return 0;

	caps = &spcm->pcm.caps[stream];

	/* pre-allocate capture audio buffer pages */
	dev_dbg(sdev->dev, "spcm: allocate %s capture DMA buffer size 0x%x max 0x%x\n",
		caps->name, caps->buffer_size_min, caps->buffer_size_max);

	snd_pcm_lib_preallocate_pages(pcm->streams[stream].substream,
				      SNDRV_DMA_TYPE_DEV_SG, sdev->dev,
				      le32_to_cpu(caps->buffer_size_min),
				      le32_to_cpu(caps->buffer_size_max));

	return 0;
}

static int sof_pcm_probe(struct snd_soc_component *component)
{
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(component);
	struct snd_sof_pdata *plat_data = sdev->pdata;
	const char *tplg_filename;
	int ret;

	/* load the default topology */
	sdev->component = component;

	tplg_filename = devm_kasprintf(sdev->dev, GFP_KERNEL,
				       "%s/%s",
				       plat_data->tplg_filename_prefix,
				       plat_data->tplg_filename);
	if (!tplg_filename)
		return -ENOMEM;

	ret = snd_sof_load_topology(sdev, tplg_filename);
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed to load DSP topology %d\n",
			ret);
		return ret;
	}

	/*
	 * Some platforms in SOF, ex: BYT, may not have their platform PM
	 * callbacks set. Increment the usage count so as to
	 * prevent the device from entering runtime suspend.
	 */
	if (!sof_ops(sdev)->runtime_suspend || !sof_ops(sdev)->runtime_resume)
		pm_runtime_get_noresume(sdev->dev);

	return ret;
}

static void sof_pcm_remove(struct snd_soc_component *component)
{
	/* remove topology */
	snd_soc_tplg_component_remove(component, SND_SOC_TPLG_INDEX_ALL);
}

static int pcm_dai_link_fixup(struct snd_soc_pcm_runtime *rtd,
			      struct snd_pcm_hw_params *params)
{
	struct snd_soc_component *component =
		snd_soc_rtdcom_lookup(rtd, SOF_COMPONENT_NAME);
	struct snd_sof_dev *sdev = snd_soc_component_get_drvdata(component);

	return snd_sof_dai_link_fixup(sdev, rtd, params);
}

void snd_sof_new_platform_drv(struct snd_sof_dev *sdev)
{
	struct snd_soc_component_driver *pd = &sdev->plat_drv;
	struct snd_sof_pdata *plat_data = sdev->pdata;
	const char *drv_name;

	drv_name = plat_data->machine->drv_name;

	pd->name = "sof-audio-component";
	pd->probe = sof_pcm_probe;
	pd->remove = sof_pcm_remove;
	pd->ops	= &sof_pcm_ops;
#if IS_ENABLED(CONFIG_SND_SOC_SOF_COMPRESS)
	pd->compr_ops = &sof_compressed_ops;
#endif
	pd->pcm_new = sof_pcm_new;
	pd->ignore_machine = drv_name;
	pd->be_hw_params_fixup = pcm_dai_link_fixup;
	pd->be_pcm_base = SOF_BE_PCM_BASE;
	pd->use_dai_pcm_id = true;
	pd->topology_name_prefix = "sof";

	 /* increment module refcount when a pcm is opened */
	pd->module_get_upon_open = 1;
}
