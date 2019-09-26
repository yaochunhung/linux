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
 * FW Panic/fault handling.
 */

struct ipc_panic_msg {
	u32 id;
	const char *msg;
};

/* standard FW panic types */
static const struct ipc_panic_msg panic_msg[] = {
	{SOF_IPC_PANIC_MEM, "out of memory"},
	{SOF_IPC_PANIC_WORK, "work subsystem init failed"},
	{SOF_IPC_PANIC_IPC, "IPC subsystem init failed"},
	{SOF_IPC_PANIC_ARCH, "arch init failed"},
	{SOF_IPC_PANIC_PLATFORM, "platform init failed"},
	{SOF_IPC_PANIC_TASK, "scheduler init failed"},
	{SOF_IPC_PANIC_EXCEPTION, "runtime exception"},
	{SOF_IPC_PANIC_DEADLOCK, "deadlock"},
	{SOF_IPC_PANIC_STACK, "stack overflow"},
	{SOF_IPC_PANIC_IDLE, "can't enter idle"},
	{SOF_IPC_PANIC_WFI, "invalid wait state"},
	{SOF_IPC_PANIC_ASSERT, "assertion failed"},
};

/*
 * helper to be called from .dbg_dump callbacks. No error code is
 * provided, it's left as an exercise for the caller of .dbg_dump
 * (typically IPC or loader)
 */
void panic_get_status(struct snd_sof_dev *sdev, u32 panic_code,
		      u32 tracep_code, struct snd_sof_oops *oops)
{
	struct sof_ipc_panic_info *panic_info = oops->ipc_panic;
	u32 code;
	int i;

	/* is firmware dead ? */
	if ((panic_code & SOF_IPC_PANIC_MAGIC_MASK) != SOF_IPC_PANIC_MAGIC) {
		dev_err(sdev->dev, "error: unexpected fault 0x%8.8x trace 0x%8.8x\n",
			panic_code, tracep_code);
		return; /* no fault ? */
	}

	code = panic_code & (SOF_IPC_PANIC_MAGIC_MASK | SOF_IPC_PANIC_CODE_MASK);

	for (i = 0; i < ARRAY_SIZE(panic_msg); i++) {
		if (panic_msg[i].id == code) {
			dev_err(sdev->dev, "error: %s\n", panic_msg[i].msg);
			dev_err(sdev->dev, "error: trace point %8.8x\n",
				tracep_code);
			goto out;
		}
	}

	/* unknown error */
	dev_err(sdev->dev, "error: unknown reason %8.8x\n", panic_code);
	dev_err(sdev->dev, "error: trace point %8.8x\n", tracep_code);

out:
	dev_err(sdev->dev, "error: panic at %s:%d\n",
		panic_info->filename, panic_info->linenum);
	sof_oops(sdev, oops);
	sof_stack(sdev, oops, oops->stack, oops->stack_words);
}

void panic_get_registers(struct snd_sof_dev *sdev,
			 struct snd_sof_oops *oops)
{
	struct sof_ipc_dsp_oops_xtensa *xoops;
	struct sof_ipc_panic_info *panic_info;
	u32 offset = sdev->dsp_oops_offset;

	/* allocate space for data - caller frees non NULL */
	xoops = kzalloc(sizeof(*xoops), GFP_KERNEL);
	if (!xoops)
		return;
	oops->arch_oops = xoops;
	panic_info = kzalloc(sizeof(*panic_info), GFP_KERNEL);
	if (!panic_info)
		return;
	oops->ipc_panic = panic_info;

	/* first read regsisters */
	sof_mailbox_read(sdev, offset, xoops, sizeof(*xoops));

	/* note: variable AR register array is not read */

	/* then get panic info */
	if (xoops->arch_hdr.totalsize > EXCEPT_MAX_HDR_SIZE) {
		dev_err(sdev->dev, "invalid header size 0x%x. FW oops is bogus\n",
			xoops->arch_hdr.totalsize);
		return;
	}
	offset += xoops->arch_hdr.totalsize;
	sof_block_read(sdev, oops->panic_mmio_bar, offset, panic_info,
		       sizeof(*panic_info));

	/* then get the stack */
	offset += sizeof(*panic_info);
	sof_block_read(sdev, oops->stack_mmio_bar, offset, oops->stack,
		       oops->stack_words * sizeof(u32));
}

int panic_is_exception(struct snd_sof_dev *sdev, u32 token)
{
	return ((token & SOF_IPC_PANIC_MAGIC_MASK) == SOF_IPC_PANIC_MAGIC);
}

