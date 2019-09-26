// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
//
// This file is provided under a dual BSD/GPLv2 license.  When using or
// redistributing this file, you may do so under either license.
//
// Copyright(c) 2018 Intel Corporation. All rights reserved.
//
// Author: Liam Girdwood <liam.r.girdwood@linux.intel.com>
//
// Generic IPC layer that can work over MMIO and SPI/I2C. PHY layer provided
// by platform driver code.
//

#include <linux/mutex.h>
#include <linux/types.h>

#include "sof-priv.h"
#include "ops.h"
#include "ipc-ops.h"


/*
 * IPC message Tx/Rx message handling.
 */

/* send IPC message from host to DSP */
int sof_ipc_tx_message_unlocked(struct snd_sof_ipc *ipc, u32 header,
				void *msg_data, size_t msg_bytes,
				void *reply_data, size_t reply_bytes)
{
	struct snd_sof_dev *sdev = ipc->sdev;
	struct snd_sof_ipc_msg *msg;
	int ret;

	if (ipc->disable_ipc_tx)
		return -ENODEV;

	/*
	 * The spin-lock is also still needed to protect message objects against
	 * other atomic contexts.
	 */
	spin_lock_irq(&sdev->ipc_lock);

	/* initialise the message */
	msg = &ipc->msg;

	msg->header = header;
	msg->msg_size = msg_bytes;
	msg->reply_size = reply_bytes;
	msg->reply_error = 0;

	/* attach any data */
	if (msg_bytes)
		memcpy(msg->msg_data, msg_data, msg_bytes);

	sdev->msg = msg;

	ret = snd_sof_dsp_send_msg(sdev, msg);
	/* Next reply that we receive will be related to this message */
	if (!ret)
		msg->ipc_complete = false;

	spin_unlock_irq(&sdev->ipc_lock);

	if (ret < 0) {
		/* So far IPC TX never fails, consider making the above void */
		dev_err_ratelimited(sdev->dev,
				    "error: ipc tx failed with error %d\n",
				    ret);
		return ret;
	}

	snd_sof_ipc_log_header(sdev, "ipc tx", msg->header);

	/* now wait for completion */
	if (!ret)
		ret = snd_sof_ipc_tx_wait_done(sdev, ipc, msg, reply_data);

	return ret;
}
EXPORT_SYMBOL(sof_ipc_tx_message_unlocked);

/* send IPC message from host to DSP */
int sof_ipc_tx_message(struct snd_sof_ipc *ipc, u32 header,
		       void *msg_data, size_t msg_bytes, void *reply_data,
		       size_t reply_bytes)
{
	int ret;

	if (msg_bytes > SOF_IPC_MSG_MAX_SIZE ||
	    reply_bytes > SOF_IPC_MSG_MAX_SIZE)
		return -ENOBUFS;

	/* Serialise IPC TX */
	mutex_lock(&ipc->tx_mutex);

	ret = sof_ipc_tx_message_unlocked(ipc, header, msg_data, msg_bytes,
					  reply_data, reply_bytes);

	mutex_unlock(&ipc->tx_mutex);

	return ret;
}
EXPORT_SYMBOL(sof_ipc_tx_message);

/* handle reply message from DSP */
int snd_sof_ipc_reply(struct snd_sof_dev *sdev, u32 msg_id)
{
	struct snd_sof_ipc_msg *msg = &sdev->ipc->msg;

	if (msg->ipc_complete) {
		dev_err(sdev->dev, "error: no reply expected, received 0x%x",
			msg_id);
		return -EINVAL;
	}

	/* wake up and return the error if we have waiters on this message ? */
	msg->ipc_complete = true;
	wake_up(&msg->waitq);

	return 0;
}
EXPORT_SYMBOL(snd_sof_ipc_reply);

struct snd_sof_ipc *snd_sof_ipc_init(struct snd_sof_dev *sdev)
{
	struct snd_sof_ipc *ipc;
	struct snd_sof_ipc_msg *msg;

	/* check if mandatory ops required for ipc are defined */
	if (!sof_ops(sdev)->fw_ready) {
		dev_err(sdev->dev, "error: ipc mandatory ops not defined\n");
		return NULL;
	}

	ipc = devm_kzalloc(sdev->dev, sizeof(*ipc), GFP_KERNEL);
	if (!ipc)
		return NULL;

	mutex_init(&ipc->tx_mutex);
	ipc->sdev = sdev;
	msg = &ipc->msg;

	/* indicate that we aren't sending a message ATM */
	msg->ipc_complete = true;

	/* pre-allocate message data */
	msg->msg_data = devm_kzalloc(sdev->dev, SOF_IPC_MSG_MAX_SIZE,
				     GFP_KERNEL);
	if (!msg->msg_data)
		return NULL;

	msg->reply_data = devm_kzalloc(sdev->dev, SOF_IPC_MSG_MAX_SIZE,
				       GFP_KERNEL);
	if (!msg->reply_data)
		return NULL;

	init_waitqueue_head(&msg->waitq);

	return ipc;
}
EXPORT_SYMBOL(snd_sof_ipc_init);

void snd_sof_ipc_free(struct snd_sof_dev *sdev)
{
	struct snd_sof_ipc *ipc = sdev->ipc;

	/* disable sending of ipc's */
	mutex_lock(&ipc->tx_mutex);
	ipc->disable_ipc_tx = true;
	mutex_unlock(&ipc->tx_mutex);
}
EXPORT_SYMBOL(snd_sof_ipc_free);
