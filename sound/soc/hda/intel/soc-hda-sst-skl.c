/*
 *  soc_hda_sst-skl.c - HDA DSP library functions for SKL platform
 *
 *  Copyright (C) 2014 Intel Corp
 *  Author:Rafal Redzimski <rafal.f.redzimski@intel.com>
 *		 Jeeja KP <jeeja.kp@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <sound/soc-hda-sst-ipc.h>
#include <sound/soc-hda-sst-dsp.h>
#include "soc-hda-sst-skl-fw.h"

#define SKL_FW_ROM_BASEFW_ENTERED_TIMEOUT		300
#define SKL_ROM_INIT_DONE_TIMEOUT 1000
#define SKL_FWLOAD_DONE_TIMEOUT 10000

/* Intel® HD Audio L2 local SRAM Windoww 0*/
#define SKL_HDA_ADSP_SRAM0_BASE		0x8000

/* Firmware status window */
#define SKL_HDA_ADSP_REG_FW_STATUS	SKL_HDA_ADSP_SRAM0_BASE

static int sst_skl_load_base_firmware(struct sst_dsp_ctx *ctx);

struct sst_ops skl_ops = {
	/*.parse_fw = sst_skl_parse_fw_image,
	.set_state_D0 = sst_skl_set_dsp_D0,
	.set_state_D3 = sst_skl_set_dsp_D3,*/
	.load_fw = sst_skl_load_base_firmware,
};

static bool check_fw_status(struct sst_dsp_ctx *ctx, u32 status)
{
	u32 cur_sts;
	cur_sts = sst_readl_alt(ctx, SKL_HDA_ADSP_REG_FW_STATUS) & FW_STATUS_MASK;
	return (cur_sts == status) ? true : false;
}

int sst_skl_init(struct device *dev, void __iomem *mmio_base, int irq,
		struct sst_dsp_loader_ops dsp_ops, struct sst_dsp_ctx **dsp)
{
	struct sst_dsp_ctx *ctx;
	int ret = 0;

	ctx = devm_kzalloc(dev, sizeof(*ctx), GFP_KERNEL);
	if (ctx == NULL)
		return -ENOMEM;
	ctx->mmio_base = mmio_base;
	ctx->irq = irq;
	ctx->dev = dev;

	dev_dbg(dev, "mmio_base: 0x%p\n", ctx->mmio_base);
	ctx->dsp_ops = dsp_ops;
	ctx->window.w0stat = mmio_base + SKL_HDA_ADSP_SRAM0_BASE;
	ctx->window.w0up = mmio_base + SKL_HDA_ADSP_SRAM0_BASE + WINDOW0_STAT_SIZE;
	ctx->window.w1 = mmio_base + HDA_ADSP_SRAM1_BASE;
	ctx->window.w0stat_size = WINDOW0_STAT_SIZE;
	ctx->window.w0up_size = WINDOW0_UP_SIZE;
	ctx->window.w1_size = WINDOW1_SIZE;

	ctx->ops = skl_ops;

	ret = sst_dsp_init(ctx);
	if (ret < 0)
		return ret;

	ret = ctx->ops.load_fw(ctx);
	if (ret < 0) {
		dev_err(dev, "Load base fw failed : %x", ret);
		return ret;
	}

	if (dsp)
		*dsp = ctx;
	return 0;
}
EXPORT_SYMBOL_GPL(sst_skl_init);

static int sst_skl_transfer_firmware(struct sst_dsp_ctx  *ctx,
		const void *basefw,
		u32 base_fw_size, u32 page_count)
{
	int ret = 0, i = 0;
	struct snd_dma_buffer dmab_fw;
	struct snd_dma_buffer dmab_bdl;
	u32 *bdl;

	dev_dbg(ctx->dev, "In sst_skl_transfer_firmware fw size=%d page_count=%d\n", base_fw_size, page_count);
	/*FIXME need to move this to soc-hda-dsp.c*/
	ret = ctx->dsp_ops.alloc_dma_buf(ctx->dev, &dmab_fw, base_fw_size);
	if (ret < 0) {
		dev_err(ctx->dev, "Alloc buffer for base fw failed: %x", ret);
		goto sst_transfer_firmware_failed;
	}

	ret = ctx->dsp_ops.alloc_dma_buf(ctx->dev,
						&dmab_bdl, PAGE_SIZE);
	if (ret < 0) {
		dev_err(ctx->dev, "Alloc buffer for blde failed: %x", ret);
		goto sst_transfer_firmware_failed;
	}

	memcpy(dmab_fw.area, basefw, base_fw_size);

	/* Setup bdle */
	bdl = (u32 *)dmab_bdl.area;
	for (i = 0; i < page_count; i++) {
		phys_addr_t addr = virt_to_phys(dmab_fw.area + i * PAGE_SIZE);
		bdl[0] = cpu_to_le32(lower_32_bits(addr));
		bdl[1] = cpu_to_le32(upper_32_bits(addr));
		bdl[2] = cpu_to_le32(PAGE_SIZE);
		bdl[3] = 0;
		bdl += 4;
	}

	/* Setup controller*/
	sst_writel(ctx, CL_SD_BDLPL, CL_SD_BDLPLBA(dmab_bdl.addr));
	sst_writel(ctx, CL_SD_BDLPU, CL_SD_BDLPUBA(dmab_bdl.addr));

	sst_writel(ctx, CL_SD_CBL, base_fw_size);
	sst_writel(ctx, CL_SD_LVI, page_count - 1);

	sst_updatel(ctx, CL_SPBFIFO_SPBFCCTL,
				SPIBE_MASK, CL_SPBFIFO_SPBFCCTL_SPIBE(1));
	sst_writel(ctx, CL_SPBFIFO_SPIB, base_fw_size);

	sst_updatel(ctx, CL_SD_CTL, IOCE_MASK, CL_SD_CTL_IOCE(1));
	sst_updatel(ctx, CL_SD_CTL, FEIE_MASK, CL_SD_CTL_FEIE(1));
	sst_updatel(ctx, CL_SD_CTL, DEIE_MASK, CL_SD_CTL_DEIE(1));
	sst_updatel(ctx, CL_SD_CTL, STRM_MASK,
					CL_SD_CTL_STRM(FW_CL_STREAM_NUMER));

	sst_updatel(ctx, CL_SD_CTL, RUN_MASK, CL_SD_CTL_RUN(1));

	ret = sst_register_poll(ctx,
			SKL_HDA_ADSP_REG_FW_STATUS,
			FW_STATUS_MASK,
			FW_ROM_BASEFW_ENTERED,
			SKL_FW_ROM_BASEFW_ENTERED_TIMEOUT,
			"Firmware boot");

	/* Reset controller*/
	sst_updatel(ctx, CL_SD_CTL, RUN_MASK, CL_SD_CTL_RUN(0));

	sst_updatel(ctx, CL_SD_CTL, IOCE_MASK, CL_SD_CTL_IOCE(0));
	sst_updatel(ctx, CL_SD_CTL, FEIE_MASK, CL_SD_CTL_FEIE(0));
	sst_updatel(ctx, CL_SD_CTL, DEIE_MASK, CL_SD_CTL_DEIE(0));
	sst_updatel(ctx, CL_SD_CTL, STRM_MASK, CL_SD_CTL_STRM(0));

	sst_writel(ctx, CL_SD_BDLPL, CL_SD_BDLPLBA(0));
	sst_writel(ctx, CL_SD_BDLPU, 0);

	sst_writel(ctx, CL_SD_CBL, 0);
	sst_writel(ctx, CL_SD_LVI, 0);

	sst_updatel(ctx, CL_SPBFIFO_SPBFCCTL,
				SPIBE_MASK,  CL_SPBFIFO_SPBFCCTL_SPIBE(0));
	sst_writel(ctx, CL_SPBFIFO_SPIB, 0);

sst_transfer_firmware_failed:
	ctx->dsp_ops.free_dma_buf(ctx->dev, &dmab_bdl);
	ctx->dsp_ops.free_dma_buf(ctx->dev, &dmab_fw);
	return ret;
}

static int sst_skl_load_base_firmware(struct sst_dsp_ctx *ctx)
{
	int ret = 0, i;
	const struct firmware *fw = NULL;
	u32 fw_preload_page_count = 0;
	u32 base_fw_size = 0;
	struct sst_fw_image_manifest *manifest;
	u32 reg;

	ctx->ipc->boot_complete = false;
	ret = sst_boot_dsp(ctx);
	if (ret < 0) {
		dev_err(ctx->dev, "Boot dsp core failed ret: %d", ret);
		return ret;
	}

	ret = request_firmware(&fw, "dsp_fw_release.bin", ctx->dev);
	if (ret < 0) {
		dev_err(ctx->dev, "Request firmware failed %d\n", ret);
		sst_disable_dsp_core(ctx);
		return -EIO;
	}

	/*enable Interrupt */
	ipc_int_enable(ctx);
	ipc_op_int_enable(ctx);

	/*check ROM Status */
	for (i = SKL_ROM_INIT_DONE_TIMEOUT; i > 0; --i) {
		if (check_fw_status(ctx, FW_ROM_INIT_DONE)) {
			dev_dbg(ctx->dev, "ROM loaded, we can continue with FW loading\n");
			break;
		}
		mdelay(1);
	}
	if (!i) {
		reg = sst_readl_alt(ctx, SKL_HDA_ADSP_REG_FW_STATUS);
		dev_err(ctx->dev, "Timeout waiting for ROM init done, reg:0x%x\n", reg);
		ret = -EIO;
		goto sst_load_base_firmware_failed;
	}

	manifest = (struct sst_fw_image_manifest *)fw->data;
	fw_preload_page_count =
		manifest->adsp_fw_bin_desc.header.preload_page_count;
	base_fw_size = fw_preload_page_count * PAGE_SIZE;

	if (base_fw_size > fw->size) {
		dev_err(ctx->dev, "Preloaded base fw size is bigger then whole fw image");
		ret = -EIO;
		goto sst_load_base_firmware_failed;
	}

	ret = sst_skl_transfer_firmware(ctx, fw->data,
			base_fw_size, fw_preload_page_count);
	if (ret < 0) {
		dev_err(ctx->dev, "Transfer firmware failed%d\n", ret);
		goto sst_load_base_firmware_failed;
	} else {
		dev_dbg(ctx->dev, "Download firmware successfull%d\n", ret);
		/*FIXME - remove once firmware implementation is done
		ret = wait_event_timeout(ctx->ipc->boot_wait, ctx->ipc->boot_complete,
						msecs_to_jiffies(IPC_BOOT_MSECS));
		if (ret == 0) {
			dev_err(ctx->dev, "DSP boot failed, FW Ready timed-out\n");
			ret = -EIO;
			goto sst_load_base_firmware_failed;
		} */
	}
	release_firmware(fw);
	return 0;

sst_load_base_firmware_failed:
	sst_disable_dsp_core(ctx);
	release_firmware(fw);
	return ret;
}
