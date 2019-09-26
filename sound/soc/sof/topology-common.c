// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
//
// This file is provided under a dual BSD/GPLv2 license.  When using or
// redistributing this file, you may do so under either license.
//
// Copyright(c) 2018 Intel Corporation. All rights reserved.
//
// Author: Liam Girdwood <liam.r.girdwood@linux.intel.com>
//

#include <linux/firmware.h>
#include <sound/tlv.h>
#include <sound/pcm_params.h>
#include <uapi/sound/sof/tokens.h>
#include "sof-priv.h"
#include "ops.h"

int snd_sof_load_topology(struct snd_sof_dev *sdev, const char *file)
{
	const struct firmware *fw;
	int ret;

	dev_dbg(sdev->dev, "loading topology:%s\n", file);

	ret = request_firmware(&fw, file, sdev->dev);
	if (ret < 0) {
		dev_err(sdev->dev, "error: tplg request firmware %s failed err: %d\n",
			file, ret);
		return ret;
	}

	ret = snd_soc_tplg_component_load(sdev->component,
					  sdev->tplg_ops, fw,
					  SND_SOC_TPLG_INDEX_ALL);
	if (ret < 0) {
		dev_err(sdev->dev, "error: tplg component load failed %d\n",
			ret);
		ret = -EINVAL;
	}

	release_firmware(fw);
	return ret;
}
EXPORT_SYMBOL(snd_sof_load_topology);
