// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
//
// This file is provided under a dual BSD/GPLv2 license.  When using or
// redistributing this file, you may do so under either license.
//
// Copyright(c) 2018 Intel Corporation. All rights reserved.
//
// Author: Liam Girdwood <liam.r.girdwood@linux.intel.com>
//

/* Mixer Controls */

#include <linux/pm_runtime.h>
#include "sof-priv.h"
#include "ops.h"
#include "ipc-ops.h"

int snd_sof_volume_get(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *sm =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_sof_control *scontrol = sm->dobj.private;
	struct snd_sof_dev *sdev = scontrol->sdev;
	void *cdata = scontrol->ipc_control_data;
	unsigned int i, channels = scontrol->num_channels;

	/* read back each channel */
	for (i = 0; i < channels; i++)
		ucontrol->value.integer.value[i] =
			snd_sof_ipc_to_volume(sdev, cdata, i,
					      scontrol->volume_table,
					      sm->max + 1);

	return 0;
}
EXPORT_SYMBOL(snd_sof_volume_get);

int snd_sof_volume_put(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *sm =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_sof_control *scontrol = sm->dobj.private;
	struct snd_sof_dev *sdev = scontrol->sdev;
	void *cdata = scontrol->ipc_control_data;
	unsigned int i, channels = scontrol->num_channels;

	/* update each channel */
	for (i = 0; i < channels; i++) {
		snd_sof_volume_to_ipc(sdev, cdata, i,
				      ucontrol->value.integer.value[i],
				      scontrol->volume_table, sm->max + 1);
	}

	/* notify DSP of mixer updates */
	if (pm_runtime_active(sdev->dev))
		snd_sof_ipc_set_get_comp_data(sdev->ipc, scontrol,
					      SOF_KCTRL_COMP_SET_VALUE,
					      SOF_KCTRL_CHAN_GET,
					      SOF_KCTRL_CMD_VOLUME,
					      true);

	return 0;
}
EXPORT_SYMBOL(snd_sof_volume_put);

int snd_sof_switch_get(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *sm =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_sof_control *scontrol = sm->dobj.private;
	struct snd_sof_dev *sdev = scontrol->sdev;
	void *cdata = scontrol->ipc_control_data;
	unsigned int i, channels = scontrol->num_channels;

	/* read back each channel */
	for (i = 0; i < channels; i++)
		ucontrol->value.integer.value[i] =
			snd_sof_ipc_to_value(sdev, cdata, i);

	return 0;
}
EXPORT_SYMBOL(snd_sof_switch_get);

int snd_sof_switch_put(struct snd_kcontrol *kcontrol,
		       struct snd_ctl_elem_value *ucontrol)
{
	struct soc_mixer_control *sm =
		(struct soc_mixer_control *)kcontrol->private_value;
	struct snd_sof_control *scontrol = sm->dobj.private;
	struct snd_sof_dev *sdev = scontrol->sdev;
	void *cdata = scontrol->ipc_control_data;
	unsigned int i, channels = scontrol->num_channels;

	/* update each channel */
	for (i = 0; i < channels; i++) {
		snd_sof_value_to_ipc(sdev, cdata, i,
				      ucontrol->value.integer.value[i]);
	}

	/* notify DSP of mixer updates */
	if (pm_runtime_active(sdev->dev))
		snd_sof_ipc_set_get_comp_data(sdev->ipc, scontrol,
					      SOF_KCTRL_COMP_SET_VALUE,
					      SOF_KCTRL_CHAN_GET,
					      SOF_KCTRL_CMD_SWITCH,
					      true);

	return 0;
}
EXPORT_SYMBOL(snd_sof_switch_put);

int snd_sof_enum_get(struct snd_kcontrol *kcontrol,
		     struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *se =
		(struct soc_enum *)kcontrol->private_value;
	struct snd_sof_control *scontrol = se->dobj.private;
	struct snd_sof_dev *sdev = scontrol->sdev;
	void *cdata = scontrol->ipc_control_data;
	unsigned int i, channels = scontrol->num_channels;

	/* read back each channel */
	for (i = 0; i < channels; i++)
		ucontrol->value.integer.value[i] =
			snd_sof_ipc_to_value(sdev, cdata, i);

	return 0;
}
EXPORT_SYMBOL(snd_sof_enum_get);

int snd_sof_enum_put(struct snd_kcontrol *kcontrol,
		     struct snd_ctl_elem_value *ucontrol)
{
	struct soc_enum *se =
		(struct soc_enum *)kcontrol->private_value;
	struct snd_sof_control *scontrol = se->dobj.private;
	struct snd_sof_dev *sdev = scontrol->sdev;
	void *cdata = scontrol->ipc_control_data;
	unsigned int i, channels = scontrol->num_channels;

	/* update each channel */
	for (i = 0; i < channels; i++) {
		snd_sof_value_to_ipc(sdev, cdata, i,
				     ucontrol->value.integer.value[i]);
	}

	/* notify DSP of enum updates */
	if (pm_runtime_active(sdev->dev))
		snd_sof_ipc_set_get_comp_data(sdev->ipc, scontrol,
					      SOF_KCTRL_COMP_SET_VALUE,
					      SOF_KCTRL_CHAN_GET,
					      SOF_KCTRL_CMD_ENUM,
					      true);

	return 0;
}
EXPORT_SYMBOL(snd_sof_enum_put);

int snd_sof_bytes_get(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct soc_bytes_ext *be =
		(struct soc_bytes_ext *)kcontrol->private_value;
	struct snd_sof_control *scontrol = be->dobj.private;
	struct snd_sof_dev *sdev = scontrol->sdev;
	int ret = 0;

	if (be->max > sizeof(ucontrol->value.bytes.data)) {
		dev_err_ratelimited(sdev->dev,
				    "error: data max %d exceeds ucontrol data array size\n",
				    be->max);
		return -EINVAL;
	}

	ret = snd_sof_control_bytes_get(sdev, kcontrol, ucontrol);
	if (ret < 0)
		dev_err(sdev->dev, "error: failed to get bytes\n");
	return ret;
}
EXPORT_SYMBOL(snd_sof_bytes_get);

int snd_sof_bytes_put(struct snd_kcontrol *kcontrol,
		      struct snd_ctl_elem_value *ucontrol)
{
	struct soc_bytes_ext *be =
		(struct soc_bytes_ext *)kcontrol->private_value;
	struct snd_sof_control *scontrol = be->dobj.private;
	struct snd_sof_dev *sdev = scontrol->sdev;
	int ret;

	if (be->max > sizeof(ucontrol->value.bytes.data)) {
		dev_err_ratelimited(sdev->dev,
				    "error: data max %d exceeds ucontrol data array size\n",
				    be->max);
		return -EINVAL;
	}

	ret = snd_sof_control_bytes_put(sdev, kcontrol, ucontrol);
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed to put bytes\n");
		return ret;
	}

	/* notify DSP of byte control updates */
	if (pm_runtime_active(sdev->dev))
		snd_sof_ipc_set_get_comp_data(sdev->ipc, scontrol,
					      SOF_KCTRL_COMP_SET_DATA,
					      SOF_KCTRL_DATA_SET,
					      scontrol->ipc_ctrl_cmd,
					      true);

	return ret;
}
EXPORT_SYMBOL(snd_sof_bytes_put);

int snd_sof_bytes_ext_put(struct snd_kcontrol *kcontrol,
			  const unsigned int __user *binary_data,
			  unsigned int size)
{
	struct soc_bytes_ext *be =
		(struct soc_bytes_ext *)kcontrol->private_value;
	struct snd_sof_control *scontrol = be->dobj.private;
	struct snd_sof_dev *sdev = scontrol->sdev;
	struct snd_ctl_tlv header;
	const struct snd_ctl_tlv __user *tlvd =
		(const struct snd_ctl_tlv __user *)binary_data;
	int ret;

	/*
	 * The beginning of bytes data contains a header from where
	 * the length (as bytes) is needed to know the correct copy
	 * length of data from tlvd->tlv.
	 */
	if (copy_from_user(&header, tlvd, sizeof(const struct snd_ctl_tlv)))
		return -EFAULT;

	/* be->max is coming from topology */
	if (header.length > be->max) {
		dev_err_ratelimited(sdev->dev, "error: Bytes data size %d exceeds max %d.\n",
				    header.length, be->max);
		return -EINVAL;
	}

	/* Check that header id matches the command */
	if (header.numid != scontrol->ipc_ctrl_cmd) {
		dev_err_ratelimited(sdev->dev,
				    "error: incorrect numid %d\n",
				    header.numid);
		return -EINVAL;
	}

	ret = snd_sof_control_bytes_ext_put(sdev, kcontrol, &header,
					    binary_data, size);
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed to put bytes\n");
		return ret;
	}

	/* notify DSP of byte control updates */
	if (pm_runtime_active(sdev->dev))
		snd_sof_ipc_set_get_comp_data(sdev->ipc, scontrol,
					      SOF_KCTRL_COMP_SET_DATA,
					      SOF_KCTRL_DATA_SET,
					      scontrol->ipc_ctrl_cmd,
					      true);

	return 0;
}
EXPORT_SYMBOL(snd_sof_bytes_ext_put);

int snd_sof_bytes_ext_get(struct snd_kcontrol *kcontrol,
			  unsigned int __user *binary_data,
			  unsigned int size)
{
	struct soc_bytes_ext *be =
		(struct soc_bytes_ext *)kcontrol->private_value;
	struct snd_sof_control *scontrol = be->dobj.private;
	struct snd_sof_dev *sdev = scontrol->sdev;
	int ret = 0;

	/*
	 * Decrement the limit by ext bytes header size to
	 * ensure the user space buffer is not exceeded.
	 */
	size -= sizeof(const struct snd_ctl_tlv);

	ret = snd_sof_control_bytes_ext_get(sdev, kcontrol, binary_data, size);
	if (ret < 0)
		dev_err(sdev->dev, "error: failed to put bytes\n");

	return ret;
}
EXPORT_SYMBOL(snd_sof_bytes_ext_get);
