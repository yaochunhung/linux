/*
 *  Intel bxt IPC Support
 *
 * Copyright (C) 2014, Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/list.h>
#include <linux/device.h>
#include <linux/wait.h>
#include <linux/spinlock.h>
#include <linux/workqueue.h>
#include <linux/export.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/kthread.h>
#include <linux/firmware.h>
#include <linux/dma-mapping.h>
#include <linux/debugfs.h>
#include <linux/irqreturn.h>
#include <sound/soc-hda-sst-dsp.h>
#include <sound/soc-hda-sst-ipc.h>

#define IXC_STATUS_BITS 24

/* Global Message - Generic */
#define IPC_GLB_TYPE_SHIFT	24
#define IPC_GLB_TYPE_MASK	(0xf << IPC_GLB_TYPE_SHIFT)
#define IPC_GLB_TYPE(x)		((x) << IPC_GLB_TYPE_SHIFT)

/* Global Message - Reply */
#define IPC_GLB_REPLY_STATUS_SHIFT	24
#define IPC_GLB_REPLY_STATUS_MASK	((0x1 << IPC_GLB_REPLY_STATUS_SHIFT) - 1)
#define IPC_GLB_REPLY_STATUS(x)		((x) << IPC_GLB_REPLY_STATUS_SHIFT)

#define IPC_TIMEOUT_MSECS	3000

#define IPC_EMPTY_LIST_SIZE	8

#define IPC_MSG_TARGET_SHIFT	30
#define IPC_MSG_TARGET_MASK	0x1
#define IPC_MSG_TARGET(x)	(((x) & IPC_MSG_TARGET_MASK) \
				<< IPC_MSG_TARGET_SHIFT)

#define IPC_MSG_DIR_SHIFT	29
#define IPC_MSG_DIR_MASK	0x1
#define IPC_MSG_DIR(x)		(((x) & IPC_MSG_DIR_MASK) \
				<< IPC_MSG_DIR_SHIFT)
/*Global Notofication Message*/
#define IPC_GLB_NOTIFI_TYPE_SHIFT	16
#define IPC_GLB_NOTIFI_TYPE_MASK	0xFF
#define IPC_GLB_NOTIFI_TYPE(x)		(((x) >> IPC_GLB_NOTIFI_TYPE_SHIFT) \
					& IPC_GLB_NOTIFI_TYPE_MASK)

#define IPC_GLB_NOTIFI_MSG_TYPE_SHIFT	24
#define IPC_GLB_NOTIFI_MSG_TYPE_MASK	0x1F
#define IPC_GLB_NOTIFI_MSG_TYPE(x)	(((x) >> IPC_GLB_NOTIFI_MSG_TYPE_SHIFT) & IPC_GLB_NOTIFI_MSG_TYPE_MASK)

#define IPC_GLB_NOTIFI_RSP_SHIFT	29
#define IPC_GLB_NOTIFI_RSP_MASK		0x1
#define IPC_GLB_NOTIFI_RSP_TYPE(x)	(((x) >> IPC_GLB_NOTIFI_RSP_SHIFT) \
					& IPC_GLB_NOTIFI_RSP_MASK)

/** Pipeline operations **/
/* Create pipeline message */
#define IPC_PPL_MEM_SIZE_SHIFT	0
#define IPC_PPL_MEM_SIZE_MASK	0x7FF
#define IPC_PPL_MEM_SIZE(x)	(((x) & IPC_PPL_MEM_SIZE_MASK) \
				<< IPC_PPL_MEM_SIZE_SHIFT)

#define IPC_PPL_TYPE_SHIFT	11
#define IPC_PPL_TYPE_MASK	0x1F
#define IPC_PPL_TYPE(x)		(((x) & IPC_PPL_TYPE_MASK) \
				<< IPC_PPL_TYPE_SHIFT)

#define IPC_INSTANCE_ID_SHIFT	16
#define IPC_INSTANCE_ID_MASK	0xFF
#define IPC_INSTANCE_ID(x)	(((x) & IPC_INSTANCE_ID_MASK) \
				<< IPC_INSTANCE_ID_SHIFT)

/* Set pipeline state message */
#define IPC_PPL_STATE_SHIFT	0
#define IPC_PPL_STATE_MASK	0x1F
#define IPC_PPL_STATE(x)	(((x) & IPC_PPL_STATE_MASK) \
				<< IPC_PPL_STATE_SHIFT)

/** Module operations **/
/* primary register */
#define IPC_MODULE_ID_SHIFT	0
#define IPC_MODULE_ID_MASK	0xFFFF
#define IPC_MODULE_ID(x)	(((x) & IPC_MODULE_ID_MASK) \
				<< IPC_MODULE_ID_SHIFT)

#define IPC_MODULE_INSTANCE_ID_SHIFT	16
#define IPC_MODULE_INSTANCE_ID_MASK	0xFF
#define IPC_MODULE_INSTANCE_ID(x)	(((x) & IPC_MODULE_INSTANCE_ID_MASK) \
					<< IPC_MODULE_INSTANCE_ID_SHIFT)

/* Init instance message */
/* extension register */
#define IPC_PARAM_BLOCK_SIZE_SHIFT	0
#define IPC_PARAM_BLOCK_SIZE_MASK	0xFFFF
#define IPC_PARAM_BLOCK_SIZE(x)		(((x) & IPC_PARAM_BLOCK_SIZE_MASK) \
					<< IPC_PARAM_BLOCK_SIZE_SHIFT)

#define IPC_PPL_INSTANCE_ID_SHIFT	16
#define IPC_PPL_INSTANCE_ID_MASK	0xFF
#define IPC_PPL_INSTANCE_ID(x)		(((x) & IPC_PPL_INSTANCE_ID_MASK) \
					<< IPC_PPL_INSTANCE_ID_SHIFT)

#define IPC_CORE_ID_SHIFT	24
#define IPC_CORE_ID_MASK	0x1F
#define IPC_CORE_ID(x)		(((x) & IPC_CORE_ID_MASK) \
				<< IPC_CORE_ID_SHIFT)

/* Bind/Unbind message */
/* extension register */
#define IPC_DST_MODULE_ID_SHIFT	0
#define IPC_DST_MODULE_ID(x)	(((x) & IPC_MODULE_ID_MASK) \
				<< IPC_DST_MODULE_ID_SHIFT)

#define IPC_DST_MODULE_INSTANCE_ID_SHIFT 16
#define IPC_DST_MODULE_INSTANCE_ID(x)	(((x) & IPC_MODULE_INSTANCE_ID_MASK) \
					<< IPC_DST_MODULE_INSTANCE_ID_SHIFT)

#define IPC_DST_QUEUE_SHIFT	24
#define IPC_DST_QUEUE_MASK	0x7
#define IPC_DST_QUEUE(x)	(((x) & IPC_DST_QUEUE_MASK) \
				<< IPC_DST_QUEUE_SHIFT)

#define IPC_SRC_QUEUE_SHIFT	27
#define IPC_SRC_QUEUE_MASK	0x7
#define IPC_SRC_QUEUE(x)	(((x) & IPC_SRC_QUEUE_MASK) \
				<< IPC_SRC_QUEUE_SHIFT)

/*Save pipeline messgae extension register */
#define IPC_DMA_ID_SHIFT	0
#define IPC_DMA_ID_MASK		0x1F
#define IPC_DMA_ID(x)		(((x) & IPC_DMA_ID_MASK) \
				<< IPC_DMA_ID_SHIFT)

#define W0_SIZE			2048
#define W1_SIZE			4096

enum ipc_msg_target {
	IPC_FW_GEN_MSG = 0,
	IPC_MODULE_MSG = 1
};

enum ipc_msg_direction {
	IPC_MSG_REQUEST = 0,
	IPC_MSG_REPLY = 1
};

/* Global Message Types */
enum ipc_glb_type {
	IPC_GLB_GET_FW_VERSION = 0, /**< Retrieves firmware version */
	IPC_GLB_LOAD_MULTIPLE_MODULES = 15,
	IPC_GLB_UNLOAD_MULTIPLE_MODULES = 16,
	IPC_GLB_CREATE_PIPELINE = 17,
	IPC_GLB_DELETE_PIPELINE = 18,
	IPC_GLB_SET_PIPELINE_STATE = 19,
	IPC_GLB_GET_PIPELINE_STATE = 20,
	IPC_GLB_GET_PIPELINE_CONTEXT_SIZE = 21,
	IPC_GLB_SAVE_PIPELINE = 22,
	IPC_GLB_RESTORE_PIPELINE = 23,
	IPC_GLB_NOTIFICATION = 26,
	IPC_GLB_MAX_IPC_MESSAGE_TYPE = 31 /**< Maximum message number */
};

enum ipc_glb_reply {
	IPC_GLB_REPLY_SUCCESS = 0,

	IPC_GLB_REPLY_UNKNOWN_MESSAGE_TYPE = 1,
	IPC_GLB_REPLY_ERROR_INVALID_PARAM = 2,

	IPC_GLB_REPLY_BUSY = 3,
	IPC_GLB_REPLY_PENDING = 4,
	IPC_GLB_REPLY_FAILURE = 5,
	IPC_GLB_REPLY_INVALID_REQUEST = 6,

	IPC_GLB_REPLY_OUT_OF_MEMORY = 7,
	IPC_GLB_REPLY_OUT_OF_MIPS = 8,

	IPC_GLB_REPLY_INVALID_RESOURCE_ID = 9,
	IPC_GLB_REPLY_INVALID_RESOURCE_STATE = 10,

	IPC_GLB_REPLY_MOD_MGMT_ERROR = 100,
	IPC_GLB_REPLY_MOD_LOAD_CL_FAILED = 101,
	IPC_GLB_REPLY_MOD_LOAD_INVALID_HASH = 102,

	IPC_GLB_REPLY_MOD_UNLOAD_INST_EXIST = 103,
	IPC_GLB_REPLY_MOD_NOT_INITIALIZED = 104,

	IPC_GLB_REPLY_INVALID_CONFIG_PARAM_ID = 120,
	IPC_GLB_REPLY_INVALID_CONFIG_DATA_LEN = 121,
	IPC_GLB_REPLY_GATEWAY_NOT_INITIALIZED = 140,
	IPC_GLB_REPLY_GATEWAY_NOT_EXIST = 141,

	IPC_GLB_REPLY_PIPELINE_NOT_INITIALIZED = 160,
	IPC_GLB_REPLY_PIPELINE_NOT_EXIST = 161,
	IPC_GLB_REPLY_PIPELINE_SAVE_FAILED = 162,
	IPC_GLB_REPLY_PIPELINE_RESTORE_FAILED = 163,

	IPC_MAX_STATUS = ((1<<IXC_STATUS_BITS)-1)
};

enum ipc_notification_type {
	IPC_GLB_NOTIFCATION_GLITCH = 0,
	IPC_GLB_NOTIFCATION_OVERRUN = 1,
	IPC_GLB_NOTIFCATION_UNDERRUN = 2,
	IPC_GLB_NOTIFCATION_END_STREAM = 3,
	IPC_GLB_NOTIFCATION_PHRASE_DETECTED = 4,
	IPC_GLB_NOTIFCATION_RESOURCE_EVENT = 5,
	IPC_GLB_NOTIFCATION_LOG_BUFFER_STATUS = 6,
	IPC_GLB_NOTIFCATION_TIMESTAMP_CAPTURED = 7,
	IPC_GLB_NOTIFCATION_FW_READY = 8
};

/* Module Message Types */
enum ipc_module_msg {
	IPC_MODULE_INIT_INSTANCE = 0,
	IPC_MODULE_CONFIG_GET = 1,
	IPC_MODULE_CONFIG_SET = 2,
	IPC_MODULE_LARGE_CONFIG_GET = 3,
	IPC_MODULE_LARGE_CONFIG_SET = 4,
	IPC_MODULE_BIND = 5,
	IPC_MODULE_UNBIND = 6,
	IPC_MODULE_SET_DX = 7
};

struct ipc_message {
	struct list_head list;
	struct header header;
	char tx_data[WINDOW1_SIZE];
	size_t tx_size;
	char rx_data[WINDOW0_UP_SIZE];
	size_t rx_size;

	wait_queue_head_t waitq;
	bool complete;
	bool wait;
	int errno;
};

/* locks held by caller */
static struct ipc_message *ipc_msg_get_empty(struct ipc *ipc)
{
	struct ipc_message *msg = NULL;

	if (!list_empty(&ipc->empty_list)) {
		msg = list_first_entry(&ipc->empty_list,
			struct ipc_message, list);
		list_del(&msg->list);
	}

	return msg;
}

/* locks held by caller */
static void ipc_msg_put_empty(struct ipc *ipc,
	struct ipc_message *msg)
{
	list_add_tail(&msg->list, &ipc->empty_list);
}

static int ipc_tx_wait_done(struct ipc *ipc, struct ipc_message *msg,
	void *rx_data)
{
	int ret;
	unsigned long irq_flags;

	/* wait for DSP completion (in all cases atm inc pending) */
	ret = wait_event_timeout(msg->waitq, msg->complete,
		msecs_to_jiffies(IPC_TIMEOUT_MSECS));

	spin_lock_irqsave(&ipc->ipc_lock, irq_flags);

	if (ret == 0)
		ret = -ETIMEDOUT;
	else {
	/* copy the data returned from DSP */
		if (msg->rx_size) {
			if (rx_data)
				memcpy(rx_data, msg->rx_data, msg->rx_size);
			else
				dev_err(ipc->dev, "error: no output buffer");
		}
		ret = msg->errno;
	}

	ipc_msg_put_empty(ipc, msg);

	spin_unlock_irqrestore(&ipc->ipc_lock, irq_flags);
	return ret;
}

static int ipc_tx_message(struct ipc *ipc, struct header header,
	void *tx_data, size_t tx_bytes, void *rx_data, size_t rx_bytes,
	int wait)
{
	struct ipc_message *msg;
	unsigned long irq_flags;

	spin_lock_irqsave(&ipc->ipc_lock, irq_flags);

	msg = ipc_msg_get_empty(ipc);
	dev_dbg(ipc->dev, "msg pointer send %p", msg);
	if (msg == NULL) {
		spin_unlock_irqrestore(&ipc->ipc_lock, irq_flags);
		return -EBUSY;
	}

	if (tx_bytes)
		memcpy(msg->tx_data, tx_data, tx_bytes);

	msg->header.primary = header.primary;
	msg->header.extension = header.extension;
	msg->tx_size = tx_bytes;
	msg->rx_size = rx_bytes;
	msg->wait = wait;
	msg->errno = 0;
	msg->complete = false;

	list_add_tail(&msg->list, &ipc->tx_list);

	spin_unlock_irqrestore(&ipc->ipc_lock, irq_flags);

	queue_kthread_work(&ipc->kworker, &ipc->kwork);

	if (wait)
		return ipc_tx_wait_done(ipc, msg, rx_data);
	else
		return 0;
}

int ipc_tx_message_wait(struct ipc *ipc,
	struct header header, void *tx_data, size_t tx_bytes, void *rx_data,
	size_t rx_bytes)
{
	return ipc_tx_message(ipc, header, tx_data, tx_bytes, rx_data,
	rx_bytes, 1);
}

static inline int ipc_tx_message_nowait(struct ipc *ipc,
	struct header header, void *tx_data, size_t tx_bytes)
{
	return ipc_tx_message(ipc, header, tx_data, tx_bytes, NULL, 0, 0);
}

static void ipc_tx_msgs(struct kthread_work *work)
{
	struct ipc *ipc =
		container_of(work, struct ipc, kwork);
	struct ipc_message *msg;
	unsigned long irq_flags;
	u32 hipci;


	spin_lock_irqsave(&ipc->ipc_lock, irq_flags);
	if (list_empty(&ipc->tx_list)) {
		spin_unlock_irqrestore(&ipc->ipc_lock, irq_flags);
		return;
	}

	/* if the DSP is busy we will TX messages after IRQ */
	hipci = sst_readl(ipc->dsp, HIPCI);
	if (hipci & HDA_ADSP_REG_HIPCI_BUSY) {
		dev_dbg(ipc->dev, "ipc_tx_msgs dsp busy HIPCI(48)=%x\n", hipci);
		spin_unlock_irqrestore(&ipc->ipc_lock, irq_flags);
		return;
	}

	msg = list_first_entry(&ipc->tx_list, struct ipc_message, list);

	list_move(&msg->list, &ipc->rx_list);

	/* send the message */
	sst_mailbox_write(ipc->dsp, msg->tx_data, msg->tx_size);
	sst_writel(ipc->dsp, HIPCIE,
		msg->header.extension);
	sst_writel(ipc->dsp, HIPCI,
		msg->header.primary | HDA_ADSP_REG_HIPCI_BUSY);

	dev_dbg(ipc->dev, "sending msg HIPCI(48)=%x\n",
		 sst_readl(ipc->dsp, HIPCI));
	spin_unlock_irqrestore(&ipc->ipc_lock, irq_flags);
}

static inline void ipc_tx_msg_reply_complete(struct ipc *ipc,
	struct ipc_message *msg)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&ipc->ipc_lock, irq_flags);
	msg->complete = true;

	if (!msg->wait) {
		ipc_msg_put_empty(ipc, msg);
		spin_unlock_irqrestore(&ipc->ipc_lock, irq_flags);
	} else {
		spin_unlock_irqrestore(&ipc->ipc_lock, irq_flags);
		wake_up(&msg->waitq);
	}
}

static struct ipc_message *ipc_reply_find_msg(struct ipc *ipc, u32 header)
{
	struct ipc_message *msg = NULL;
	unsigned long irq_flags;

	spin_lock_irqsave(&ipc->ipc_lock, irq_flags);

	if (list_empty(&ipc->rx_list)) {
		dev_err(ipc->dev, "ipc: rx list is empty but received 0x%x\n",
			header);
		goto out;
	}

	msg = list_first_entry(&ipc->rx_list, struct ipc_message, list);

out:
	spin_unlock_irqrestore(&ipc->ipc_lock, irq_flags);
	return msg;
}

static void ipc_reply_remove(struct ipc *ipc, struct ipc_message *msg)
{
	unsigned long irq_flags;

	spin_lock_irqsave(&ipc->ipc_lock, irq_flags);
	if (list_empty(&ipc->rx_list)) {
		dev_dbg(ipc->dev, "empty rx list");
		goto out;
	}
	list_del(&msg->list);

out:
	spin_unlock_irqrestore(&ipc->ipc_lock, irq_flags);
}

static int ipc_process_notification(struct ipc *ipc, struct header header)
{
	if (IPC_GLB_NOTIFI_MSG_TYPE(header.primary) ==
			IPC_GLB_NOTIFI_MSG_TYPE(header.primary)) {
		switch (IPC_GLB_NOTIFI_TYPE(header.primary)) {
		case IPC_GLB_NOTIFCATION_GLITCH:
			break;
		case IPC_GLB_NOTIFCATION_OVERRUN:
			break;
		case IPC_GLB_NOTIFCATION_UNDERRUN:
			dev_dbg(ipc->dev, "FW UNDERRUN\n");
			break;
		case IPC_GLB_NOTIFCATION_END_STREAM:
			break;
		case IPC_GLB_NOTIFCATION_PHRASE_DETECTED:
			break;
		case IPC_GLB_NOTIFCATION_RESOURCE_EVENT:
			dev_dbg(ipc->dev, "MCPS Budget Violation\n");
			break;
		case IPC_GLB_NOTIFCATION_LOG_BUFFER_STATUS:
			break;
		case IPC_GLB_NOTIFCATION_TIMESTAMP_CAPTURED:
			break;
		case IPC_GLB_NOTIFCATION_FW_READY:
			ipc->boot_complete = true;
			wake_up(&ipc->boot_wait);
			break;
		default:
			dev_err(ipc->dev, "ipc: error received unexpected msg=%x", header.primary);
			break;
		}
	}
	return 0;
}

static void ipc_process_reply(struct ipc *ipc, struct header header)
{
	struct ipc_message *msg;
	u32 reply = header.primary & IPC_GLB_REPLY_STATUS_MASK;
	msg = ipc_reply_find_msg(ipc, header.primary);
	if (msg == NULL) {
		dev_dbg(ipc->dev, "ipc: rx list is empty\n");
		return;
	}

	/* first process the header */
	switch (reply) {
	case IPC_GLB_REPLY_SUCCESS:
		dev_dbg(ipc->dev, "ipc FW reply: success\n");
		break;
	case IPC_GLB_REPLY_OUT_OF_MEMORY:
		msg->errno = -ENOMEM;
		break;
	case IPC_GLB_REPLY_BUSY:
		msg->errno = -EBUSY;
		break;
	case IPC_GLB_REPLY_FAILURE:
	case IPC_GLB_REPLY_INVALID_REQUEST:
	case IPC_GLB_REPLY_ERROR_INVALID_PARAM:
	case IPC_GLB_REPLY_PIPELINE_NOT_INITIALIZED:
		msg->errno = -EINVAL;
		break;
	default:
		dev_err(ipc->dev, "ipc reply: 0x%x", reply);
		msg->errno = -EINVAL;
		break;
	}

	dev_err(ipc->dev, "********ipc FW reply: 0x%x", reply);
	if (msg->errno != IPC_GLB_REPLY_SUCCESS)
		dev_err(ipc->dev, "ipc FW reply: 0x%x reply=0x%x", msg->errno, reply);

	ipc_reply_remove(ipc, msg);
	ipc_tx_msg_reply_complete(ipc, msg);
}

irqreturn_t sst_irq_thread_handler(int irq, void *context)
{
	struct sst_dsp_ctx *dsp = (struct sst_dsp_ctx *)context;
	struct ipc *ipc = dsp->ipc;
	struct header header = {0};
	u32 hipcie, hipct, hipcte;
	int ipc_irq = 0;

	hipcie = sst_readl(dsp, HIPCIE);
	hipct = sst_readl(dsp, HIPCT);

	/* reply message from DSP */
	if (hipcie & HDA_ADSP_REG_HIPCIE_DONE) {
		sst_updatel_bits(dsp, HDA_ADSP_REG_HIPCCTL,
			HDA_ADSP_REG_HIPCCTL_DONE, 0);
		/* clear DONE bit - tell DSP we have completed the operation */

		sst_updatel_bits(dsp, HDA_ADSP_REG_HIPCIE,
			HDA_ADSP_REG_HIPCIE_DONE, HDA_ADSP_REG_HIPCIE_DONE);

		ipc_irq = 1;

		/* unmask Done interrupt */
		sst_updatel_bits(dsp, HDA_ADSP_REG_HIPCCTL,
			HDA_ADSP_REG_HIPCCTL_DONE, HDA_ADSP_REG_HIPCCTL_DONE);
	}

	/* to do: new message from DSP */
	if (hipct & HDA_ADSP_REG_HIPCT_BUSY) {
		dev_dbg(dsp->dev, "IPC irq: Firmware respond");
		hipcte = sst_readl(dsp, HIPCTE);
		header.primary = hipct;
		header.extension = hipcte;

		if (header.primary & 0x20000000) {
			/* Handle Immediate reply from DSP Core */
			ipc_process_reply(ipc, header);
		} else {
			trace_printk("IPC irq: Notification from firmware\n");
			ipc_process_notification(ipc, header);
		}
		/* clear  busy interrupt */
		sst_updatel_bits(dsp, HDA_ADSP_REG_HIPCT,
			HDA_ADSP_REG_HIPCT_BUSY, HDA_ADSP_REG_HIPCT_BUSY);
		ipc_irq = 1;
	}

	if (ipc_irq == 0)
		return IRQ_NONE;

	ipc_int_enable(dsp);
	/* continue to send any remaining messages... */
	queue_kthread_work(&ipc->kworker, &ipc->kwork);
	return IRQ_HANDLED;
}

static int ipc_msg_empty_list_init(struct ipc *ipc)
{
	struct ipc_message *msg;
	int i;
	msg = kzalloc(sizeof(*msg) * IPC_EMPTY_LIST_SIZE, GFP_KERNEL);
	if (msg == NULL)
		return -ENOMEM;

	for (i = 0; i < IPC_EMPTY_LIST_SIZE; i++) {
		init_waitqueue_head(&msg[i].waitq);
		list_add(&msg[i].list, &ipc->empty_list);
	}

	return 0;
}

void ipc_int_enable(struct sst_dsp_ctx *ctx)
{
	sst_updatel_bits(ctx, HDA_ADSP_REG_ADSPIC,
			ADSPIC_IPC, ADSPIC_IPC);
}

void ipc_int_disable(struct sst_dsp_ctx *ctx)
{
	sst_updatel_bits(ctx, HDA_ADSP_REG_ADSPIC,
			ADSPIC_IPC, 0);
}

void ipc_op_int_enable(struct sst_dsp_ctx *ctx)
{
	/* enable IPC DONE interrupt */
	sst_updatel_bits(ctx, HDA_ADSP_REG_HIPCCTL,
		HDA_ADSP_REG_HIPCCTL_DONE, HDA_ADSP_REG_HIPCCTL_DONE);
	/* Enable IPC BUSY interrupt */
	sst_updatel_bits(ctx, HDA_ADSP_REG_HIPCCTL,
		HDA_ADSP_REG_HIPCCTL_BUSY, HDA_ADSP_REG_HIPCCTL_BUSY);
}

bool ipc_int_status(struct sst_dsp_ctx *ctx)
{
	return sst_readl(ctx, ADSPIS) & ADSPIS_IPC;
}


struct ipc *ipc_init(struct device *dev,
	struct sst_dsp_ctx *dsp)
{
	struct ipc *ipc;
	int err;

	dev_dbg(dev, "In %s\n", __func__);

	ipc = kzalloc(sizeof(*ipc), GFP_KERNEL);
	if (ipc == NULL)
		return NULL;

	ipc->dev = dev;
	ipc->dsp = dsp;

	INIT_LIST_HEAD(&ipc->tx_list);
	INIT_LIST_HEAD(&ipc->rx_list);
	INIT_LIST_HEAD(&ipc->empty_list);
	spin_lock_init(&ipc->ipc_lock);
	init_waitqueue_head(&ipc->wait_txq);
	init_waitqueue_head(&ipc->boot_wait);

	err = ipc_msg_empty_list_init(ipc);
	if (err < 0)
		goto list_err;

	/* start the IPC message thread */
	init_kthread_worker(&ipc->kworker);
	ipc->tx_thread = kthread_run(kthread_worker_fn,
					&ipc->kworker,
					dev_name(ipc->dev));
	if (IS_ERR(ipc->tx_thread)) {
		dev_err(ipc->dev, "error failed to create message TX task\n");
		goto list_err;
	}
	init_kthread_work(&ipc->kwork, ipc_tx_msgs);

	return ipc;

list_err:
	kfree(ipc);
	return NULL;
}

void ipc_free(struct ipc *ipc)
{
	/* Disable  IPC DONE interrupt */
	sst_updatel_bits(ipc->dsp, HDA_ADSP_REG_HIPCCTL,
		HDA_ADSP_REG_HIPCCTL_DONE, 0);
	/* Disable IPC BUSY interrupt */
	sst_updatel_bits(ipc->dsp, HDA_ADSP_REG_HIPCCTL,
		HDA_ADSP_REG_HIPCCTL_BUSY, 0);

	kfree(ipc);
}

int ipc_create_pipeline(struct ipc *ipc, u16 ppl_mem_size,
	u8 ppl_type, u8 instance_id)
{
	struct header header = {0};
	int ret = 0;

	header.primary = IPC_MSG_TARGET(IPC_FW_GEN_MSG);
	header.primary |= IPC_MSG_DIR(IPC_MSG_REQUEST);
	header.primary |= IPC_GLB_TYPE(IPC_GLB_CREATE_PIPELINE);
	header.primary |= IPC_INSTANCE_ID(instance_id);
	header.primary |= IPC_PPL_TYPE(ppl_type);
	header.primary |= IPC_PPL_MEM_SIZE(ppl_mem_size);

	dev_dbg(ipc->dev, "In %s header=%d\n", __func__, header.primary);
	ret = ipc_tx_message_wait(ipc, header, NULL, 0, NULL, 0);
	if (ret < 0) {
		dev_err(ipc->dev, "ipc: create pipeline failed, err: 0x%x\n", ret);
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(ipc_create_pipeline);

int ipc_delete_pipeline(struct ipc *ipc, u8 instance_id)
{
	struct header header = {0};
	int ret = 0;

	header.primary = IPC_MSG_TARGET(IPC_FW_GEN_MSG);
	header.primary |= IPC_MSG_DIR(IPC_MSG_REQUEST);
	header.primary |= IPC_GLB_TYPE(IPC_GLB_DELETE_PIPELINE);
	header.primary |= IPC_INSTANCE_ID(instance_id);

	dev_dbg(ipc->dev, "In %s header=%d\n", __func__, header.primary);
	ret = ipc_tx_message_wait(ipc, header, NULL, 0, NULL, 0);
	if (ret < 0) {
		dev_err(ipc->dev, "ipc: delete pipeline failed\n");
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(ipc_delete_pipeline);

int ipc_set_pipeline_state(struct ipc *ipc, u8 instance_id,
	enum pipeline_state state)
{
	struct header header = {0};
	int ret = 0;

	header.primary = IPC_MSG_TARGET(IPC_FW_GEN_MSG);
	header.primary |= IPC_MSG_DIR(IPC_MSG_REQUEST);
	header.primary |= IPC_GLB_TYPE(IPC_GLB_SET_PIPELINE_STATE);
	header.primary |= IPC_INSTANCE_ID(instance_id);
	header.primary |= IPC_PPL_STATE(state);

	dev_dbg(ipc->dev, "In %s header=%d\n", __func__, header.primary);
	ret = ipc_tx_message_wait(ipc, header, NULL, 0, NULL, 0);
	if (ret < 0) {
		dev_err(ipc->dev, "ipc: set pipeline state failed\n");
		return ret;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(ipc_set_pipeline_state);

int ipc_save_pipeline(struct ipc *ipc, u8 instance_id, int dma_id)
{
	struct header header = {0};
	int ret = 0;

	header.primary = IPC_MSG_TARGET(IPC_FW_GEN_MSG);
	header.primary |= IPC_MSG_DIR(IPC_MSG_REQUEST);
	header.primary |= IPC_GLB_TYPE(IPC_GLB_SAVE_PIPELINE);
	header.primary |= IPC_INSTANCE_ID(instance_id);

	header.extension = IPC_DMA_ID(dma_id);
	dev_dbg(ipc->dev, "In %s header=%d\n", __func__, header.primary);
	ret = ipc_tx_message_wait(ipc, header, NULL, 0, NULL, 0);
	if (ret < 0) {
		dev_err(ipc->dev, "ipc: save pipeline failed, err: 0x%x\n", ret);
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(ipc_save_pipeline);

int ipc_restore_pipeline(struct ipc *ipc, u8 instance_id)
{
	struct header header = {0};
	int ret = 0;

	header.primary = IPC_MSG_TARGET(IPC_FW_GEN_MSG);
	header.primary |= IPC_MSG_DIR(IPC_MSG_REQUEST);
	header.primary |= IPC_GLB_TYPE(IPC_GLB_RESTORE_PIPELINE);
	header.primary |= IPC_INSTANCE_ID(instance_id);

	dev_dbg(ipc->dev, "In %s header=%d\n", __func__, header.primary);
	ret = ipc_tx_message_wait(ipc, header, NULL, 0, NULL, 0);
	if (ret < 0) {
		dev_err(ipc->dev, "ipc: restore  pipeline failed, err: 0x%x\n", ret);
		return ret;
	}

	return ret;
}
EXPORT_SYMBOL_GPL(ipc_restore_pipeline);

int ipc_set_dx(struct ipc *ipc, u8 instance_id, u16 module_id,
		struct dxstate_info *dx)
{
	struct header header = {0};
	int ret = 0;

	header.primary = IPC_MSG_TARGET(IPC_MODULE_MSG);
	header.primary |= IPC_MSG_DIR(IPC_MSG_REQUEST);
	header.primary |= IPC_GLB_TYPE(IPC_MODULE_SET_DX);
	header.primary |= IPC_MODULE_INSTANCE_ID(instance_id);
	header.primary |= IPC_MODULE_ID(module_id);

	dev_dbg(ipc->dev, "In %s primary =%x ext=%x\n", __func__,
			 header.primary, header.extension);
	ret = ipc_tx_message_wait(ipc, header, (void *)dx, sizeof(dx),
		NULL, 0);
	if (ret < 0) {
		dev_err(ipc->dev, "ipc: set dx failed\n");
		return ret;
	}

	return ret;
}

int ipc_init_instance(struct ipc *ipc, struct init_instance_msg *msg,
	void *param_data)
{
	struct header header = {0};
	 /* param_block_size must be in dwords */
	u16 param_block_size = msg->param_data_size / sizeof(u32);
	int ret = 0;
	u32 *buffer = (u32 *)param_data;
	int i = 0;

	dev_dbg(ipc->dev, "param size: %d bytes\n", msg->param_data_size);
	for (i = 0; i < param_block_size; ++i)
		dev_dbg(ipc->dev, "%x\n ", buffer[i]);

	header.primary = IPC_MSG_TARGET(IPC_MODULE_MSG);
	header.primary |= IPC_MSG_DIR(IPC_MSG_REQUEST);
	header.primary |= IPC_GLB_TYPE(IPC_MODULE_INIT_INSTANCE);
	header.primary |= IPC_MODULE_INSTANCE_ID(msg->instance_id);
	header.primary |= IPC_MODULE_ID(msg->module_id);

	header.extension = IPC_CORE_ID(msg->core_id);
	header.extension |= IPC_PPL_INSTANCE_ID(msg->ppl_instance_id);
	header.extension |= IPC_PARAM_BLOCK_SIZE(param_block_size);

	dev_dbg(ipc->dev, "In %s primary =%x ext=%x\n", __func__,
			 header.primary, header.extension);
	ret = ipc_tx_message_wait(ipc, header, param_data, msg->param_data_size,
		NULL, 0);

	if (ret < 0) {
		dev_err(ipc->dev, "ipc: init instance failed\n");
		return ret;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(ipc_init_instance);

int ipc_bind_unbind(struct ipc *ipc, struct bind_unbind_msg *msg)
{
	struct header header = {0};
	u8 bind_unbind = msg->bind ? IPC_MODULE_BIND : IPC_MODULE_UNBIND;
	int ret = 0;

	header.primary = IPC_MSG_TARGET(IPC_MODULE_MSG);
	header.primary |= IPC_MSG_DIR(IPC_MSG_REQUEST);
	header.primary |= IPC_GLB_TYPE(bind_unbind);
	header.primary |= IPC_MODULE_INSTANCE_ID(msg->instance_id);
	header.primary |= IPC_MODULE_ID(msg->module_id);

	header.extension = IPC_DST_MODULE_ID(msg->dst_module_id);
	header.extension |= IPC_DST_MODULE_INSTANCE_ID(msg->dst_instance_id);
	header.extension |= IPC_DST_QUEUE(msg->dst_queue);
	header.extension |= IPC_SRC_QUEUE(msg->src_queue);

	dev_dbg(ipc->dev, "In %s hdr=%x ext=%x\n", __func__, header.primary,
			 header.extension);
	ret = ipc_tx_message_wait(ipc, header, NULL, 0, NULL, 0);
	if (ret < 0) {
		dev_err(ipc->dev, "ipc: bind/unbind failed\n");
		return ret;
	}
	return ret;
}
EXPORT_SYMBOL_GPL(ipc_bind_unbind);
