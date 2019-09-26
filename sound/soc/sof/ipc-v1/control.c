// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
//
// This file is provided under a dual BSD/GPLv2 license.  When using or
// redistributing this file, you may do so under either license.
//
// Copyright(c) 2019 Intel Corporation. All rights reserved.
//
// Author: Liam Girdwood <liam.r.girdwood@linux.intel.com>
//
// Legacy IPC Support. IPC flavour is detected by drivers on FW boot.
//

#include <linux/mutex.h>
#include <linux/types.h>
#include <sound/pcm_params.h>

#include <sound/sof-ipc-v1/stream.h> /* needs to be included before control.h */
#include <sound/sof-ipc-v1/control.h>
#include <sound/sof-ipc-v1/dai.h>
#include <sound/sof-ipc-v1/info.h>
#include <sound/sof-ipc-v1/pm.h>
#include <sound/sof-ipc-v1/topology.h>
#include <sound/sof-ipc-v1/trace.h>
#include <sound/sof-ipc-v1/xtensa.h>

#include "ipc-v1.h"

/*
 * Control Level
 */

void kctrl_volume_to_ipc(struct snd_sof_dev *sdev, void *control_data,
			 unsigned int channel, unsigned int value,
			 u32 *volume_map, int size)
{
	struct sof_ipc_ctrl_data *cdata = control_data;

	cdata->chanv[channel].channel = channel;

	if (value >= size)
		cdata->chanv[channel].value = volume_map[size - 1];
	else
		cdata->chanv[channel].value = volume_map[value];
}

u32 kctrl_ipc_to_volume(struct snd_sof_dev *sdev, void *control_data,
			u32 channel, u32 *volume_map, int size)
{
	struct sof_ipc_ctrl_data *cdata = control_data;
	unsigned int value = cdata->chanv[channel].value;
	int i;

	for (i = 0; i < size; i++) {
		if (volume_map[i] >= value)
			return i;
	}

	return i - 1;
}


void kctrl_value_to_ipc(struct snd_sof_dev *sdev, void *control_data,
			unsigned int channel, unsigned int value)
{
	struct sof_ipc_ctrl_data *cdata = control_data;

	cdata->chanv[channel].channel = channel;
	cdata->chanv[channel].value = value;
}

u32 kctrl_ipc_to_value(struct snd_sof_dev *sdev, void *control_data,
		       u32 channel)
{
	struct sof_ipc_ctrl_data *cdata = control_data;
	return cdata->chanv[channel].value;
}

int kctrl_bytes_get(struct snd_sof_dev *sdev, struct snd_kcontrol *kcontrol,
		    struct snd_ctl_elem_value *ucontrol)
{
	struct soc_bytes_ext *be =
		(struct soc_bytes_ext *)kcontrol->private_value;
	struct snd_sof_control *scontrol = be->dobj.private;
	struct sof_ipc_ctrl_data *cdata = scontrol->ipc_control_data;
	struct sof_abi_hdr *data = cdata->data;
	size_t size;
	int ret = 0;

	size = data->size + sizeof(*data);
	if (size > be->max) {
		dev_err_ratelimited(sdev->dev,
				    "error: DSP sent %zu bytes max is %d\n",
				    size, be->max);
		ret = -EINVAL;
		goto out;
	}

	/* copy back to kcontrol */
	memcpy(ucontrol->value.bytes.data, data, size);

out:
	return ret;
}

int kctrl_bytes_put(struct snd_sof_dev *sdev, struct snd_kcontrol *kcontrol,
		    struct snd_ctl_elem_value *ucontrol)
{
	struct soc_bytes_ext *be =
		(struct soc_bytes_ext *)kcontrol->private_value;
	struct snd_sof_control *scontrol = be->dobj.private;
	struct sof_ipc_ctrl_data *cdata = scontrol->ipc_control_data;
	struct sof_abi_hdr *data = cdata->data;
	size_t size = data->size + sizeof(*data);

	if (size > be->max) {
		dev_err_ratelimited(sdev->dev,
				    "error: size too big %zu bytes max is %d\n",
				    size, be->max);
		return -EINVAL;
	}

	/* copy from kcontrol */
	memcpy(data, ucontrol->value.bytes.data, size);

	return 0;
}

int kctrl_bytes_ext_get(struct snd_sof_dev *sdev,
			struct snd_kcontrol *kcontrol,
			unsigned int __user *binary_data,
			unsigned int size)
{
	struct soc_bytes_ext *be =
		(struct soc_bytes_ext *)kcontrol->private_value;
	struct snd_sof_control *scontrol = be->dobj.private;
	struct sof_ipc_ctrl_data *cdata = scontrol->ipc_control_data;
	struct snd_ctl_tlv header;
	struct snd_ctl_tlv __user *tlvd =
		(struct snd_ctl_tlv __user *)binary_data;
	int data_size;
	int ret = 0;

	/* set the ABI header values */
	cdata->data->magic = SOF_ABI_MAGIC;
	cdata->data->abi = SOF_ABI_VERSION;

	/* Prevent read of other kernel data or possibly corrupt response */
	data_size = cdata->data->size + sizeof(const struct sof_abi_hdr);

	/* check data size doesn't exceed max coming from topology */
	if (data_size > be->max) {
		dev_err_ratelimited(sdev->dev, "error: user data size %d exceeds max size %d.\n",
				    data_size, be->max);
		ret = -EINVAL;
		goto out;
	}

	header.numid = scontrol->ipc_ctrl_cmd;
	header.length = data_size;
	if (copy_to_user(tlvd, &header, sizeof(const struct snd_ctl_tlv))) {
		ret = -EFAULT;
		goto out;
	}

	if (copy_to_user(tlvd->tlv, cdata->data, data_size))
		ret = -EFAULT;

out:
	return ret;
}

int kctrl_bytes_ext_put(struct snd_sof_dev *sdev,
			struct snd_kcontrol *kcontrol,
			struct snd_ctl_tlv *header,
			const unsigned int __user *binary_data,
			unsigned int size)
{
	struct soc_bytes_ext *be =
		(struct soc_bytes_ext *)kcontrol->private_value;
	struct snd_sof_control *scontrol = be->dobj.private;
	struct sof_ipc_ctrl_data *cdata = scontrol->ipc_control_data;
	const struct snd_ctl_tlv __user *tlvd =
		(const struct snd_ctl_tlv __user *)binary_data;

	if (copy_from_user(cdata->data, tlvd->tlv, header->length))
		return -EFAULT;

	if (cdata->data->magic != SOF_ABI_MAGIC) {
		dev_err_ratelimited(sdev->dev,
				    "error: Wrong ABI magic 0x%08x.\n",
				    cdata->data->magic);
		return -EINVAL;
	}

	if (SOF_ABI_VERSION_INCOMPATIBLE(SOF_ABI_VERSION, cdata->data->abi)) {
		dev_err_ratelimited(sdev->dev, "error: Incompatible ABI version 0x%08x.\n",
				    cdata->data->abi);
		return -EINVAL;
	}

	if (cdata->data->size + sizeof(const struct sof_abi_hdr) > be->max) {
		dev_err_ratelimited(sdev->dev, "error: Mismatch in ABI data size (truncated?).\n");
		return -EINVAL;
	}

	return 0;
}

static int copy_params(enum sof_ipc_ctrl_type ctrl_type,
		       struct sof_ipc_ctrl_data *src,
		       struct sof_ipc_ctrl_data *dst,
		       struct sof_ipc_ctrl_data_params *sparams)
{
	switch (ctrl_type) {
	case SOF_CTRL_TYPE_VALUE_CHAN_GET:
	case SOF_CTRL_TYPE_VALUE_CHAN_SET:
		sparams->src = (u8 *)src->chanv;
		sparams->dst = (u8 *)dst->chanv;
		break;
	case SOF_CTRL_TYPE_VALUE_COMP_GET:
	case SOF_CTRL_TYPE_VALUE_COMP_SET:
		sparams->src = (u8 *)src->compv;
		sparams->dst = (u8 *)dst->compv;
		break;
	case SOF_CTRL_TYPE_DATA_GET:
	case SOF_CTRL_TYPE_DATA_SET:
		sparams->src = (u8 *)src->data->data;
		sparams->dst = (u8 *)dst->data->data;
		break;
	default:
		return -EINVAL;
	}

	/* calculate payload size and number of messages */
	sparams->pl_size = SOF_IPC_MSG_MAX_SIZE - sparams->hdr_bytes;
	sparams->num_msg = DIV_ROUND_UP(sparams->msg_bytes, sparams->pl_size);

	return 0;
}

static int copy_large_ctrl_data(struct snd_sof_dev *sdev,
				struct sof_ipc_ctrl_data *cdata,
				struct sof_ipc_ctrl_data_params *sparams,
				bool send)
{
	struct sof_ipc_ctrl_data *partdata;
	size_t send_bytes;
	size_t offset = 0;
	size_t msg_bytes;
	size_t pl_size;
	int err;
	int i;

	/* allocate max ipc size because we have at least one */
	partdata = kzalloc(SOF_IPC_MSG_MAX_SIZE, GFP_KERNEL);
	if (!partdata)
		return -ENOMEM;

	if (send)
		err = copy_params(cdata->type, cdata, partdata, sparams);
	else
		err = copy_params(cdata->type, partdata, cdata, sparams);
	if (err < 0)
		return err;

	msg_bytes = sparams->msg_bytes;
	pl_size = sparams->pl_size;

	/* copy the header data */
	memcpy(partdata, cdata, sparams->hdr_bytes);

	/* Serialise IPC TX */
	mutex_lock(&sdev->ipc->tx_mutex);

	/* copy the payload data in a loop */
	for (i = 0; i < sparams->num_msg; i++) {
		send_bytes = min(msg_bytes, pl_size);
		partdata->num_elems = send_bytes;
		partdata->rhdr.hdr.size = sparams->hdr_bytes + send_bytes;
		partdata->msg_index = i;
		msg_bytes -= send_bytes;
		partdata->elems_remaining = msg_bytes;

		if (send)
			memcpy(sparams->dst, sparams->src + offset, send_bytes);

		err = sof_ipc_tx_message_unlocked(sdev->ipc,
						  partdata->rhdr.hdr.cmd,
						  partdata,
						  partdata->rhdr.hdr.size,
						  partdata,
						  partdata->rhdr.hdr.size);
		if (err < 0)
			break;

		if (!send)
			memcpy(sparams->dst + offset, sparams->src, send_bytes);

		offset += pl_size;
	}

	mutex_unlock(&sdev->ipc->tx_mutex);

	kfree(partdata);
	return err;
}

/*
 * IPC get()/set() for kcontrols.
 */
int kctrl_set_get_comp_data(struct snd_sof_ipc *ipc,
			    struct snd_sof_control *scontrol, u32 ipc_cmd,
			    int ctrl_type, int ctrl_cmd, bool send)
{
	struct sof_ipc_ctrl_data *cdata = scontrol->ipc_control_data;
	struct snd_sof_dev *sdev = ipc->sdev;
	struct ipc_data *idata = sdev->ipc_private;
	struct sof_ipc_fw_ready *ready = &idata->fw_ready;
	struct sof_ipc_fw_version *v = &ready->version;
	struct sof_ipc_ctrl_data_params sparams;
	size_t send_bytes;
	int err;

	/* read or write firmware volume */
	if (scontrol->readback_offset != 0) {
		/* write/read value header via mmaped region */
		send_bytes = sizeof(struct sof_ipc_ctrl_value_chan) *
		cdata->num_elems;
		if (send)
			snd_sof_dsp_block_write(sdev, sdev->mmio_bar,
						scontrol->readback_offset,
						cdata->chanv, send_bytes);

		else
			snd_sof_dsp_block_read(sdev, sdev->mmio_bar,
					       scontrol->readback_offset,
					       cdata->chanv, send_bytes);
		return 0;
	}

	/* translate cmd to IPC value */
	switch (ipc_cmd) {
	case SOF_KCTRL_COMP_SET_DATA:
		ipc_cmd = SOF_IPC_COMP_SET_DATA;
		break;
	case SOF_KCTRL_COMP_GET_DATA:
		ipc_cmd = SOF_IPC_COMP_GET_DATA;
		break;
	case SOF_KCTRL_COMP_SET_VALUE:
		ipc_cmd = SOF_IPC_COMP_SET_VALUE;
		break;
	case SOF_KCTRL_COMP_GET_VALUE:
		ipc_cmd = SOF_IPC_COMP_GET_VALUE;
		break;
	default:
		dev_err(sdev->dev, "error: unknown control cmd %d\n", ipc_cmd);
		return -EINVAL;
	}

	/* translate type */
	switch (ctrl_type) {
	case SOF_KCTRL_CHAN_GET:
		ctrl_type = SOF_CTRL_TYPE_VALUE_CHAN_GET;
		break;
	case SOF_KCTRL_CHAN_SET:
		ctrl_type = SOF_CTRL_TYPE_VALUE_CHAN_SET;
		break;
	case SOF_KCTRL_COMP_GET:
		ctrl_type = SOF_CTRL_TYPE_VALUE_COMP_GET;
		break;
	case SOF_KCTRL_COMP_SET:
		ctrl_type = SOF_CTRL_TYPE_VALUE_COMP_SET;
		break;
	case SOF_KCTRL_DATA_GET:
		ctrl_type = SOF_CTRL_TYPE_DATA_GET;
		break;
	case SOF_KCTRL_DATA_SET:
		ctrl_type = SOF_CTRL_TYPE_DATA_SET;
		break;
	default:
		dev_err(sdev->dev, "error: unknown ctrl_type %d\n", ctrl_type);
		return -EINVAL;
	}

	/* translate cmd */
	switch (ctrl_cmd) {
	case SOF_KCTRL_CMD_VOLUME:
		ctrl_cmd = SOF_CTRL_CMD_VOLUME;
		break;
	case SOF_KCTRL_CMD_ENUM:
		ctrl_cmd = SOF_CTRL_CMD_ENUM;
		break;
	case SOF_KCTRL_CMD_SWITCH:
		ctrl_cmd = SOF_CTRL_CMD_SWITCH;
		break;
	case SOF_KCTRL_CMD_BINARY:
		ctrl_cmd = SOF_CTRL_CMD_BINARY;
		break;
	default:
		dev_err(sdev->dev, "error: unknown ctrl_cmd %d\n", ctrl_type);
		return -EINVAL;
	}


	cdata->rhdr.hdr.cmd = SOF_IPC_GLB_COMP_MSG | ipc_cmd;
	cdata->cmd = ctrl_cmd;
	cdata->type = ctrl_type;
	cdata->comp_id = scontrol->comp_id;
	cdata->msg_index = 0;

	/* calculate header and data size */
	switch (cdata->type) {
	case SOF_CTRL_TYPE_VALUE_CHAN_GET:
	case SOF_CTRL_TYPE_VALUE_CHAN_SET:
		sparams.msg_bytes = scontrol->num_channels *
			sizeof(struct sof_ipc_ctrl_value_chan);
		sparams.hdr_bytes = sizeof(struct sof_ipc_ctrl_data);
		sparams.elems = scontrol->num_channels;
		break;
	case SOF_CTRL_TYPE_VALUE_COMP_GET:
	case SOF_CTRL_TYPE_VALUE_COMP_SET:
		sparams.msg_bytes = scontrol->num_channels *
			sizeof(struct sof_ipc_ctrl_value_comp);
		sparams.hdr_bytes = sizeof(struct sof_ipc_ctrl_data);
		sparams.elems = scontrol->num_channels;
		break;
	case SOF_CTRL_TYPE_DATA_GET:
	case SOF_CTRL_TYPE_DATA_SET:
		sparams.msg_bytes = cdata->data->size;
		sparams.hdr_bytes = sizeof(struct sof_ipc_ctrl_data) +
			sizeof(struct sof_abi_hdr);
		sparams.elems = cdata->data->size;
		break;
	default:
		return -EINVAL;
	}

	cdata->rhdr.hdr.size = sparams.msg_bytes + sparams.hdr_bytes;
	cdata->num_elems = sparams.elems;
	cdata->elems_remaining = 0;

	/* send normal size ipc in one part */
	if (cdata->rhdr.hdr.size <= SOF_IPC_MSG_MAX_SIZE) {
		err = sof_ipc_tx_message(sdev->ipc, cdata->rhdr.hdr.cmd, cdata,
					 cdata->rhdr.hdr.size, cdata,
					 cdata->rhdr.hdr.size);

		if (err < 0)
			dev_err(sdev->dev, "error: set/get ctrl ipc comp %d\n",
				cdata->comp_id);

		return err;
	}

	/* data is bigger than max ipc size, chop into smaller pieces */
	dev_dbg(sdev->dev, "large ipc size %u, control size %u\n",
		cdata->rhdr.hdr.size, scontrol->size);

	/* large messages is only supported from ABI 3.3.0 onwards */
	if (v->abi_version < SOF_ABI_VER(3, 3, 0)) {
		dev_err(sdev->dev, "error: incompatible FW ABI version\n");
		return -EINVAL;
	}

	err = copy_large_ctrl_data(sdev, cdata, &sparams, send);
	if (err < 0)
		dev_err(sdev->dev, "error: set/get large ctrl ipc comp %d\n",
			cdata->comp_id);

	return err;
}


