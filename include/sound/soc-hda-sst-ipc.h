/*
 *  Intel SST IPC Support
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

#ifndef __HDA_SST_IPC_H
#define __HDA_SST_IPC_H

#include <linux/kthread.h>
#include <linux/irqreturn.h>


struct sst_dsp_ctx;

enum pipeline_state {
	PPL_INVALID_STATE = 0,
	PPL_UNINITIALIZED = 1,
	PPL_RESET = 2,
	PPL_PAUSED = 3,
	PPL_RUNNING = 4,
	PPL_ERROR_STOP = 5,
	PPL_SAVED = 6,
	PPL_RESTORED = 7
};

struct header {
	u32 primary;
	u32 extension;
};

struct ipc *ipc_init(
	struct device *dev,
	struct sst_dsp_ctx *dsp);

/* IPC data */
struct ipc {
	struct device *dev;
	struct sst_dsp_ctx *dsp;

/* IPC messaging */
	struct list_head tx_list;
	struct list_head rx_list;
	struct list_head empty_list;
	wait_queue_head_t wait_txq;
	spinlock_t ipc_lock;
	struct task_struct *tx_thread;
	struct kthread_worker kworker;
	struct kthread_work kwork;

	/* boot */
	wait_queue_head_t boot_wait;
	bool boot_complete;
	bool shutdown;
};

struct init_instance_msg {
	u32 module_id;
	u32 instance_id;
	u16 param_data_size;
	u8 ppl_instance_id;
	u8 core_id;
};

struct bind_unbind_msg {
	u32 module_id;
	u32 instance_id;
	u32 dst_module_id;
	u32 dst_instance_id;
	u8 src_queue;
	u8 dst_queue;
	bool bind;
};

#define IPC_BOOT_MSECS          3000

struct dxstate_info {
	u32 core_mask;
	u32 dx_mask;
};

#define ADSP_IPC_D3_MASK   0
#define ADSP_IPC_D0_MASK   3

irqreturn_t sst_irq_thread_handler(int irq, void *context);

int ipc_tx_message_wait(struct ipc *sst_ipc,
	struct header header, void *tx_data, size_t tx_bytes, void *rx_data,
	size_t rx_bytes);

int ipc_create_pipeline(struct ipc *sst_ipc, u16 ppl_mem_size,
	u8 ppl_type, u8 instance_id);

int ipc_delete_pipeline(struct ipc *sst_ipc, u8 instance_id);

int ipc_set_pipeline_state(struct ipc *sst_ipc, u8 instance_id,
	enum pipeline_state state);

int ipc_init_instance(struct ipc *sst_ipc, struct init_instance_msg *msg,
	void *param_data);

int ipc_bind_unbind(struct ipc *sst_ipc, struct bind_unbind_msg *msg);

int ipc_set_dx(struct ipc *ipc, u8 instance_id, u16 module_id,
		struct dxstate_info *dx);
void ipc_int_enable(struct sst_dsp_ctx *dsp);
void ipc_op_int_enable(struct sst_dsp_ctx *ctx);
void ipc_int_disable(struct sst_dsp_ctx *dsp);

bool ipc_int_status(struct sst_dsp_ctx *dsp);
void ipc_free(struct ipc *ipc);

#endif /* __HDA_SST_IPC_H */
