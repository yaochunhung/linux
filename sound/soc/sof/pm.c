// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
//
// This file is provided under a dual BSD/GPLv2 license.  When using or
// redistributing this file, you may do so under either license.
//
// Copyright(c) 2018 Intel Corporation. All rights reserved.
//
// Author: Liam Girdwood <liam.r.girdwood@linux.intel.com>
//

#include "ops.h"
#include "sof-priv.h"
#include "ipc-ops.h"

static int sof_restore_kcontrols(struct snd_sof_dev *sdev)
{
	struct snd_sof_control *scontrol;
	int ret = 0;

	/* restore kcontrol values */
	list_for_each_entry(scontrol, &sdev->kcontrol_list, list) {

		ret = snd_sof_restore_kcontrol(sdev, scontrol);
		if (ret < 0) {
			dev_err(sdev->dev,
				"error: failed kcontrol value set for widget: %d\n",
				scontrol->comp_id);

			return ret;
		}
		
	}

	return 0;
}

static int sof_restore_pipelines(struct snd_sof_dev *sdev)
{
	struct snd_sof_widget *swidget;
	struct snd_sof_route *sroute;
	struct snd_sof_dai *dai;
	int ret;

	/* restore pipeline components */
	list_for_each_entry_reverse(swidget, &sdev->widget_list, list) {

		ret = snd_sof_restore_pipeline(sdev, swidget);
		if (ret < 0) {
			dev_err(sdev->dev,
				"error: failed to load widget type %d with ID: %d\n",
				swidget->widget->id, swidget->comp_id);

			return ret;
		}
	}

	/* restore pipeline connections */
	list_for_each_entry_reverse(sroute, &sdev->route_list, list) {

		ret = snd_sof_restore_connection(sdev, sroute);
		if (ret < 0) {
			dev_err(sdev->dev,
				"error: failed to load route sink %s control %s source %s\n",
				sroute->route->sink,
				sroute->route->control ? sroute->route->control
					: "none",
				sroute->route->source);

			return ret;
		}
	}

	/* restore dai links */
	list_for_each_entry_reverse(dai, &sdev->dai_list, list) {

		ret = snd_sof_restore_dai_link(sdev, dai);
		if (ret < 0) {
			dev_err(sdev->dev,
				"error: failed to set dai config for %s\n",
				dai->name);

			return ret;
		}
	}

	/* complete pipeline */
	list_for_each_entry(swidget, &sdev->widget_list, list) {
		switch (swidget->id) {
		case snd_soc_dapm_scheduler:
			swidget->complete =
				snd_sof_complete_pipeline(sdev, swidget);
			break;
		default:
			break;
		}
	}

	/* restore pipeline kcontrols */
	ret = sof_restore_kcontrols(sdev);
	if (ret < 0)
		dev_err(sdev->dev,
			"error: restoring kcontrols after resume\n");

	return ret;
}



static int sof_set_hw_params_upon_resume(struct snd_sof_dev *sdev)
{
	struct snd_pcm_substream *substream;
	struct snd_sof_pcm *spcm;
	snd_pcm_state_t state;
	int dir;

	/*
	 * SOF requires hw_params to be set-up internally upon resume.
	 * So, set the flag to indicate this for those streams that
	 * have been suspended.
	 */
	list_for_each_entry(spcm, &sdev->pcm_list, list) {
		for (dir = 0; dir <= SNDRV_PCM_STREAM_CAPTURE; dir++) {
			substream = spcm->stream[dir].substream;
			if (!substream || !substream->runtime)
				continue;

			state = substream->runtime->status->state;
			if (state == SNDRV_PCM_STATE_SUSPENDED)
				spcm->prepared[dir] = false;
		}
	}

	/* set internal flag for BE */
	return snd_sof_dsp_hw_params_upon_resume(sdev);
}

#if IS_ENABLED(CONFIG_SND_SOC_SOF_DEBUG_ENABLE_DEBUGFS_CACHE)
static void sof_cache_debugfs(struct snd_sof_dev *sdev)
{
	struct snd_sof_dfsentry *dfse;

	list_for_each_entry(dfse, &sdev->dfsentry_list, list) {

		/* nothing to do if debugfs buffer is not IO mem */
		if (dfse->type == SOF_DFSENTRY_TYPE_BUF)
			continue;

		/* cache memory that is only accessible in D0 */
		if (dfse->access_type == SOF_DEBUGFS_ACCESS_D0_ONLY)
			memcpy_fromio(dfse->cache_buf, dfse->io_mem,
				      dfse->size);
	}
}
#endif

static int sof_resume(struct device *dev, bool runtime_resume)
{
	struct snd_sof_dev *sdev = dev_get_drvdata(dev);
	int ret;

	/* do nothing if dsp resume callbacks are not set */
	if (!sof_ops(sdev)->resume || !sof_ops(sdev)->runtime_resume)
		return 0;

	/*
	 * if the runtime_resume flag is set, call the runtime_resume routine
	 * or else call the system resume routine
	 */
	if (runtime_resume)
		ret = snd_sof_dsp_runtime_resume(sdev);
	else
		ret = snd_sof_dsp_resume(sdev);
	if (ret < 0) {
		dev_err(sdev->dev,
			"error: failed to power up DSP after resume\n");
		return ret;
	}

	/* load the firmware */
	ret = snd_sof_load_firmware(sdev);
	if (ret < 0) {
		dev_err(sdev->dev,
			"error: failed to load DSP firmware after resume %d\n",
			ret);
		return ret;
	}

	/* boot the firmware */
	ret = snd_sof_run_firmware(sdev);
	if (ret < 0) {
		dev_err(sdev->dev,
			"error: failed to boot DSP firmware after resume %d\n",
			ret);
		return ret;
	}

	/* resume DMA trace, only need send ipc */
	ret = snd_sof_init_trace_ipc(sdev);
	if (ret < 0) {
		/* non fatal */
		dev_warn(sdev->dev,
			 "warning: failed to init trace after resume %d\n",
			 ret);
	}

	/* restore pipelines */
	ret = sof_restore_pipelines(sdev);
	if (ret < 0) {
		dev_err(sdev->dev,
			"error: failed to restore pipeline after resume %d\n",
			ret);
		return ret;
	}

	/* notify DSP of system resume */
	ret = snd_sof_send_pm_ipc(sdev, SOF_PM_CTX_RESTORE);
	if (ret < 0)
		dev_err(sdev->dev,
			"error: ctx_restore ipc error during resume %d\n",
			ret);

	/* initialize default D0 sub-state */
	sdev->d0_substate = SOF_DSP_D0I0;

	return ret;
}

static int sof_suspend(struct device *dev, bool runtime_suspend)
{
	struct snd_sof_dev *sdev = dev_get_drvdata(dev);
	int ret;

	/* do nothing if dsp suspend callback is not set */
	if (!sof_ops(sdev)->suspend)
		return 0;

	/* release trace */
	snd_sof_release_trace(sdev);

	/* set restore_stream for all streams during system suspend */
	if (!runtime_suspend) {
		ret = sof_set_hw_params_upon_resume(sdev);
		if (ret < 0) {
			dev_err(sdev->dev,
				"error: setting hw_params flag during suspend %d\n",
				ret);
			return ret;
		}
	}

#if IS_ENABLED(CONFIG_SND_SOC_SOF_DEBUG_ENABLE_DEBUGFS_CACHE)
	/* cache debugfs contents during runtime suspend */
	if (runtime_suspend)
		sof_cache_debugfs(sdev);
#endif
	/* notify DSP of upcoming power down */
	ret = snd_sof_send_pm_ipc(sdev, SOF_PM_CTX_SAVE);
	if (ret == -EBUSY || ret == -EAGAIN) {
		/*
		 * runtime PM has logic to handle -EBUSY/-EAGAIN so
		 * pass these errors up
		 */
		dev_err(sdev->dev,
			"error: ctx_save ipc error during suspend %d\n",
			ret);
		return ret;
	} else if (ret < 0) {
		/* FW in unexpected state, continue to power down */
		dev_warn(sdev->dev,
			 "ctx_save ipc error %d, proceeding with suspend\n",
			 ret);
	}

	/* power down all DSP cores */
	if (runtime_suspend)
		ret = snd_sof_dsp_runtime_suspend(sdev);
	else
		ret = snd_sof_dsp_suspend(sdev);
	if (ret < 0)
		dev_err(sdev->dev,
			"error: failed to power down DSP during suspend %d\n",
			ret);

	return ret;
}

int snd_sof_runtime_suspend(struct device *dev)
{
	return sof_suspend(dev, true);
}
EXPORT_SYMBOL(snd_sof_runtime_suspend);

int snd_sof_runtime_idle(struct device *dev)
{
	struct snd_sof_dev *sdev = dev_get_drvdata(dev);

	return snd_sof_dsp_runtime_idle(sdev);
}
EXPORT_SYMBOL(snd_sof_runtime_idle);

int snd_sof_runtime_resume(struct device *dev)
{
	return sof_resume(dev, true);
}
EXPORT_SYMBOL(snd_sof_runtime_resume);

int snd_sof_resume(struct device *dev)
{
	return sof_resume(dev, false);
}
EXPORT_SYMBOL(snd_sof_resume);

int snd_sof_suspend(struct device *dev)
{
	return sof_suspend(dev, false);
}
EXPORT_SYMBOL(snd_sof_suspend);
