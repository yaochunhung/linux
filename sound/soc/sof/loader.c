// SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)
//
// This file is provided under a dual BSD/GPLv2 license.  When using or
// redistributing this file, you may do so under either license.
//
// Copyright(c) 2018 Intel Corporation. All rights reserved.
//
// Author: Liam Girdwood <liam.r.girdwood@linux.intel.com>
//
// Generic firmware loader.
//

#include <linux/firmware.h>
#include <sound/sof.h>
#include "ops.h"
#include "ipc-ops.h"

struct fw_ipc_flavour {
	const char *name;
	int (*ipc_validate)(struct snd_sof_dev *sdev, u32 token, int bar,
			    int offset);
};

/* supported IPC falvours */
static const struct fw_ipc_flavour ipc_flavour[] = {
#if IS_ENABLED(CONFIG_SND_SOC_SOF_IPC_LEGACY_SUPPORT_V1)
	{"legacy-v1", ipc_legacy_v1_validate},
#endif
};

/* check for ABI compatibility and create memory windows on first boot */
int sof_fw_ready(struct snd_sof_dev *sdev, u32 msg_id)
{
	int offset;
	int bar;
	int ret;
	int i;

	/* mailbox must be on 4k boundary */
	offset = snd_sof_dsp_get_mailbox_offset(sdev);
	if (offset < 0) {
		dev_err(sdev->dev, "error: have no mailbox offset\n");
		return offset;
	}

	bar = snd_sof_dsp_get_bar_index(sdev, SOF_FW_BLK_TYPE_SRAM);
	if (bar < 0) {
		dev_err(sdev->dev, "error: have no bar mapping\n");
		return -EINVAL;
	}

	dev_dbg(sdev->dev, "ipc: DSP is ready 0x%8.8x offset 0x%x\n",
		msg_id, offset);

	/* no need to re-check version/ABI for subsequent boots */
	if (!sdev->first_boot)
		return 0;

	/* detect IPC flavour used by firmware */
	for (i = 0; i < ARRAY_SIZE(ipc_flavour); i++) {
		dev_info(sdev->dev, "ipc: trying %s\n", ipc_flavour[i].name);

		/* does IPC match this flavor */
		ret = ipc_flavour[i].ipc_validate(sdev, msg_id, bar, offset);
		if (ret < 0)
			continue;	/* try next match */

		return 0;	/* match found */
	}

	dev_err(sdev->dev, "error: no matching ipc flavour found\n");
	return -EINVAL;	
}
EXPORT_SYMBOL(sof_fw_ready);

/* generic module parser for mmaped DSPs */
int snd_sof_parse_module_memcpy(struct snd_sof_dev *sdev,
				struct snd_sof_mod_hdr *module)
{
	struct snd_sof_blk_hdr *block;
	int count, bar;
	u32 offset;
	size_t remaining;

	dev_dbg(sdev->dev, "new module size 0x%x blocks 0x%x type 0x%x\n",
		module->size, module->num_blocks, module->type);

	block = (struct snd_sof_blk_hdr *)((u8 *)module + sizeof(*module));

	/* module->size doesn't include header size */
	remaining = module->size;
	for (count = 0; count < module->num_blocks; count++) {
		/* check for wrap */
		if (remaining < sizeof(*block)) {
			dev_err(sdev->dev, "error: not enough data remaining\n");
			return -EINVAL;
		}

		/* minus header size of block */
		remaining -= sizeof(*block);

		if (block->size == 0) {
			dev_warn(sdev->dev,
				 "warning: block %d size zero\n", count);
			dev_warn(sdev->dev, " type 0x%x offset 0x%x\n",
				 block->type, block->offset);
			continue;
		}

		switch (block->type) {
		case SOF_FW_BLK_TYPE_RSRVD0:
		case SOF_FW_BLK_TYPE_ROM...SOF_FW_BLK_TYPE_RSRVD14:
			continue;	/* not handled atm */
		case SOF_FW_BLK_TYPE_IRAM:
		case SOF_FW_BLK_TYPE_DRAM:
		case SOF_FW_BLK_TYPE_SRAM:
			offset = block->offset;
			bar = snd_sof_dsp_get_bar_index(sdev, block->type);
			if (bar < 0) {
				dev_err(sdev->dev,
					"error: no BAR mapping for block type 0x%x\n",
					block->type);
				return bar;
			}
			break;
		default:
			dev_err(sdev->dev, "error: bad type 0x%x for block 0x%x\n",
				block->type, count);
			return -EINVAL;
		}

		dev_dbg(sdev->dev,
			"block %d type 0x%x size 0x%x ==>  offset 0x%x\n",
			count, block->type, block->size, offset);

		/* checking block->size to avoid unaligned access */
		if (block->size % sizeof(u32)) {
			dev_err(sdev->dev, "error: invalid block size 0x%x\n",
				block->size);
			return -EINVAL;
		}
		snd_sof_dsp_block_write(sdev, bar, offset,
					block + 1, block->size);

		if (remaining < block->size) {
			dev_err(sdev->dev, "error: not enough data remaining\n");
			return -EINVAL;
		}

		/* minus body size of block */
		remaining -= block->size;
		/* next block */
		block = (struct snd_sof_blk_hdr *)((u8 *)block + sizeof(*block)
			+ block->size);
	}

	return 0;
}
EXPORT_SYMBOL(snd_sof_parse_module_memcpy);

static int check_header(struct snd_sof_dev *sdev, const struct firmware *fw)
{
	struct snd_sof_fw_header *header;

	/* Read the header information from the data pointer */
	header = (struct snd_sof_fw_header *)fw->data;

	/* verify FW sig */
	if (strncmp(header->sig, SND_SOF_FW_SIG, SND_SOF_FW_SIG_SIZE) != 0) {
		dev_err(sdev->dev, "error: invalid firmware signature\n");
		return -EINVAL;
	}

	/* check size is valid */
	if (fw->size != header->file_size + sizeof(*header)) {
		dev_err(sdev->dev, "error: invalid filesize mismatch got 0x%zx expected 0x%zx\n",
			fw->size, header->file_size + sizeof(*header));
		return -EINVAL;
	}

	dev_dbg(sdev->dev, "header size=0x%x modules=0x%x abi=0x%x size=%zu\n",
		header->file_size, header->num_modules,
		header->abi, sizeof(*header));

	return 0;
}

static int load_modules(struct snd_sof_dev *sdev, const struct firmware *fw)
{
	struct snd_sof_fw_header *header;
	struct snd_sof_mod_hdr *module;
	int (*load_module)(struct snd_sof_dev *sof_dev,
			   struct snd_sof_mod_hdr *hdr);
	int ret, count;
	size_t remaining;

	header = (struct snd_sof_fw_header *)fw->data;
	load_module = sof_ops(sdev)->load_module;
	if (!load_module)
		return -EINVAL;

	/* parse each module */
	module = (struct snd_sof_mod_hdr *)((u8 *)(fw->data) + sizeof(*header));
	remaining = fw->size - sizeof(*header);
	/* check for wrap */
	if (remaining > fw->size) {
		dev_err(sdev->dev, "error: fw size smaller than header size\n");
		return -EINVAL;
	}

	for (count = 0; count < header->num_modules; count++) {
		/* check for wrap */
		if (remaining < sizeof(*module)) {
			dev_err(sdev->dev, "error: not enough data remaining\n");
			return -EINVAL;
		}

		/* minus header size of module */
		remaining -= sizeof(*module);

		/* module */
		ret = load_module(sdev, module);
		if (ret < 0) {
			dev_err(sdev->dev, "error: invalid module %d\n", count);
			return ret;
		}

		if (remaining < module->size) {
			dev_err(sdev->dev, "error: not enough data remaining\n");
			return -EINVAL;
		}

		/* minus body size of module */
		remaining -=  module->size;
		module = (struct snd_sof_mod_hdr *)((u8 *)module
			+ sizeof(*module) + module->size);
	}

	return 0;
}

int snd_sof_load_firmware_raw(struct snd_sof_dev *sdev)
{
	struct snd_sof_pdata *plat_data = sdev->pdata;
	const char *fw_filename;
	int ret;

	/* set code loading condition to true */
	sdev->code_loading = 1;

	/* Don't request firmware again if firmware is already requested */
	if (plat_data->fw)
		return 0;

	fw_filename = kasprintf(GFP_KERNEL, "%s/%s",
				plat_data->fw_filename_prefix,
				plat_data->fw_filename);
	if (!fw_filename)
		return -ENOMEM;

	ret = request_firmware(&plat_data->fw, fw_filename, sdev->dev);

	if (ret < 0) {
		dev_err(sdev->dev, "error: request firmware %s failed err: %d\n",
			fw_filename, ret);
	}

	kfree(fw_filename);

	return ret;
}
EXPORT_SYMBOL(snd_sof_load_firmware_raw);

int snd_sof_load_firmware_memcpy(struct snd_sof_dev *sdev)
{
	struct snd_sof_pdata *plat_data = sdev->pdata;
	int ret;

	ret = snd_sof_load_firmware_raw(sdev);
	if (ret < 0)
		return ret;

	/* make sure the FW header and file is valid */
	ret = check_header(sdev, plat_data->fw);
	if (ret < 0) {
		dev_err(sdev->dev, "error: invalid FW header\n");
		goto error;
	}

	/* prepare the DSP for FW loading */
	ret = snd_sof_dsp_reset(sdev);
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed to reset DSP\n");
		goto error;
	}

	/* parse and load firmware modules to DSP */
	ret = load_modules(sdev, plat_data->fw);
	if (ret < 0) {
		dev_err(sdev->dev, "error: invalid FW modules\n");
		goto error;
	}

	return 0;

error:
	release_firmware(plat_data->fw);
	plat_data->fw = NULL;
	return ret;

}
EXPORT_SYMBOL(snd_sof_load_firmware_memcpy);

int snd_sof_load_firmware(struct snd_sof_dev *sdev)
{
	dev_dbg(sdev->dev, "loading firmware\n");

	if (sof_ops(sdev)->load_firmware)
		return sof_ops(sdev)->load_firmware(sdev);
	return 0;
}
EXPORT_SYMBOL(snd_sof_load_firmware);

int snd_sof_run_firmware(struct snd_sof_dev *sdev)
{
	int ret;
	int init_core_mask;

	init_waitqueue_head(&sdev->boot_wait);
	sdev->boot_complete = false;

	/* create read-only fw_version debugfs to store boot version info */
	if (sdev->first_boot) {
		ret = snd_sof_debugfs_buf_item(sdev, &sdev->fw_version,
					       sizeof(sdev->fw_version),
					       "fw_version", 0444);
		/* errors are only due to memory allocation, not debugfs */
		if (ret < 0) {
			dev_err(sdev->dev, "error: snd_sof_debugfs_buf_item failed\n");
			return ret;
		}
	}

	/* perform pre fw run operations */
	ret = snd_sof_dsp_pre_fw_run(sdev);
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed pre fw run op\n");
		return ret;
	}

	dev_dbg(sdev->dev, "booting DSP firmware\n");

	/* boot the firmware on the DSP */
	ret = snd_sof_dsp_run(sdev);
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed to reset DSP\n");
		return ret;
	}

	init_core_mask = ret;

	/* now wait for the DSP to boot */
	ret = wait_event_timeout(sdev->boot_wait, sdev->boot_complete,
				 msecs_to_jiffies(sdev->boot_timeout));
	if (ret == 0) {
		dev_err(sdev->dev, "error: firmware boot failure\n");
		snd_sof_dsp_dbg_dump(sdev, SOF_DBG_REGS | SOF_DBG_MBOX |
			SOF_DBG_TEXT | SOF_DBG_PCI);
		/* after this point FW_READY msg should be ignored */
		sdev->boot_complete = true;
		return -EIO;
	}

	dev_info(sdev->dev, "firmware boot complete\n");

	/* perform post fw run operations */
	ret = snd_sof_dsp_post_fw_run(sdev);
	if (ret < 0) {
		dev_err(sdev->dev, "error: failed post fw run op\n");
		return ret;
	}

	/* fw boot is complete. Update the active cores mask */
	sdev->enabled_cores_mask = init_core_mask;

	return 0;
}
EXPORT_SYMBOL(snd_sof_run_firmware);

void snd_sof_fw_unload(struct snd_sof_dev *sdev)
{
	/* TODO: support module unloading at runtime */
}
EXPORT_SYMBOL(snd_sof_fw_unload);
