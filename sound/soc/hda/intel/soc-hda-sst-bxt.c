/*
 *  soc_hda_sst-bxt.c - HDA DSP library functions for BXT platform
 *
 *  Copyright (C) 2014 Intel Corp
 *  Author:Rafal Redzimski <rafal.f.redzimski@intel.com>
 *	   Jeeja KP <jeeja.kp@intel.com>
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
#include <asm/cacheflush.h>
#include <sound/soc-hda-sst-ipc.h>
#include <sound/soc-hda-sst-dsp.h>
#include <sound/soc-hda-sst-ipc.h>
#include "soc-hda-sst-bxt-fw.h"

#define BXT_FW_ROM_BASEFW_ENTERED_TIMEOUT	300
#define BXT_ROM_INIT_HIPCIE_TIMEOUT	1000
#define BXT_ROM_INIT_DONE_TIMEOUT	1000
#define BXT_FWLOAD_DONE_TIMEOUT		10000
#define BXT_IMR_MEMSIZE			0x400000  /*4MB*/

#define BXT_IPC_PURGE_FW	0x01004000
#define BXT_FWSTS_FW_READY	0x2000012
#define BXT_FW_ROM_BASEFW_ENTERED	0x5

#define BXT_HDA_ADSP_SRAM0_BASE	0x80000

/* Firmware status window */
#define BXT_HDA_ADSP_REG_FW_STATUS	BXT_HDA_ADSP_SRAM0_BASE

static int sst_bxt_load_base_firmware(struct sst_dsp_ctx *ctx);
static int sst_bxt_set_dsp_D0(struct sst_dsp_ctx *ctx);
static int sst_bxt_set_dsp_D3(struct sst_dsp_ctx *ctx);

static struct sst_ops bxt_ops = {
	/*.parse_fw = sst_bxt_parse_fw_image,*/
	.set_state_D0 = sst_bxt_set_dsp_D0,
	.set_state_D3 = sst_bxt_set_dsp_D3,
	.load_fw = sst_bxt_load_base_firmware,
};


int sst_bxt_init(struct device *dev, void __iomem *mmio_base, int irq,
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
	ctx->window.w0stat = mmio_base + BXT_HDA_ADSP_SRAM0_BASE;
	ctx->window.w0up = mmio_base + BXT_HDA_ADSP_SRAM0_BASE + WINDOW0_STAT_SIZE;
	ctx->window.w1 = mmio_base + BXT_HDA_ADSP_SRAM0_BASE + WINDOW1_SIZE;
	ctx->window.w0stat_size = WINDOW0_STAT_SIZE;
	ctx->window.w0up_size = WINDOW0_UP_SIZE;
	ctx->window.w1_size = WINDOW1_SIZE;

	ctx->ops = bxt_ops;

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
EXPORT_SYMBOL_GPL(sst_bxt_init);

static int sst_bxt_prepare_fw(struct sst_dsp_ctx *ctx)
{

	int ret = 0, i = 0;
	u32 pages, reg;
	u32 fw_size = BXT_IMR_MEMSIZE;

	ret = ctx->dsp_ops.alloc_dma_buf(ctx->dev, &ctx->dsp_fw_buf, fw_size);

	if (ret < 0) {
		dev_err(ctx->dev, "Alloc buffer for base fw failed: %x\n", ret);
		/*goto err;*/
	}

	pages = (fw_size + PAGE_SIZE - 1) >> PAGE_SHIFT;

	dev_dbg(ctx->dev, "sst_bxt_prepare_fw pages=%d\n", pages);
	set_memory_wc((unsigned long)ctx->dsp_fw_buf.area, pages);

	writeq(virt_to_phys(ctx->dsp_fw_buf.area) + 1,
		 ctx->mmio_base + HDA_ADSP_REG_ADSPCS_IMR_CACHED_TLB_START);
	writeq(virt_to_phys(ctx->dsp_fw_buf.area) + 1,
		 ctx->mmio_base + HDA_ADSP_REG_ADSPCS_IMR_UNCACHED_TLB_START);

	memset(ctx->dsp_fw_buf.area, 0, fw_size);

	dev_dbg(ctx->dev, "IMR register val=%lx\n",
		 readq(ctx->mmio_base + HDA_ADSP_REG_ADSPCS_IMR_CACHED_TLB_START));
	dev_dbg(ctx->dev, "sst_bxt_prepare_fw fw_size=%x\n", fw_size);

	/* Purge FW request */
	sst_writel(ctx, HIPCI, HDA_ADSP_REG_HIPCI_BUSY | BXT_IPC_PURGE_FW);

	ret = sst_enable_dsp_core(ctx);
	if (ret < 0) {
		dev_err(ctx->dev, "Boot dsp core failed ret: %d\n", ret);
		ret = -EIO;
		goto base_fw_load_failed;
	}

	for (i = BXT_ROM_INIT_HIPCIE_TIMEOUT; i > 0; --i) {
		reg = sst_readl(ctx, HIPCIE);

		if (reg & HDA_ADSP_REG_HIPCIE_DONE) {
			sst_updatel_bits(ctx, HDA_ADSP_REG_HIPCIE,
					HDA_ADSP_REG_HIPCIE_DONE,
					HDA_ADSP_REG_HIPCIE_DONE);
			break;
		}

		mdelay(1);
	}
	if (!i) {
		dev_err(ctx->dev, "Timeout waiting for HIPCIE done, reg: 0x%x\n", reg);
		/*FIXME */
		sst_updatel_bits(ctx, HDA_ADSP_REG_HIPCIE,
				HDA_ADSP_REG_HIPCIE_DONE,
				HDA_ADSP_REG_HIPCIE_DONE);
	}

	dev_dbg(ctx->dev, "******HIPCIE reg: 0x%x\n", reg);

	/*enable Interrupt */
	ipc_int_enable(ctx);
	ipc_op_int_enable(ctx);

	for (i = BXT_ROM_INIT_DONE_TIMEOUT; i > 0; --i) {
		if (FW_ROM_INIT_DONE ==
			(sst_readl_alt(ctx, BXT_HDA_ADSP_REG_FW_STATUS) &
				FW_STATUS_MASK)) {
				dev_err(ctx->dev, "ROM loaded, we can continue with FW loading\n");
			break;
		}
		mdelay(1);
	}
	if (!i) {
		dev_err(ctx->dev, "Timeout waiting for ROM init done, reg: 0x%x\n", reg);
		ret = -EIO;
		goto base_fw_load_failed;
	}
	return ret;
base_fw_load_failed:
	sst_disable_dsp_core(ctx);
	return ret;
}

static int sst_transfer_fw_host_dma(struct sst_dsp_ctx *ctx, const void *fwdata,
		u32 fwsize)
{
	int ret = 0;
	struct snd_dma_buffer dmab;

	dev_dbg(ctx->dev, "starting to preapre host dma fwsize=%x\n", fwsize);
	ret = ctx->dsp_ops.prepare(ctx->dev, 0x40, fwsize, &dmab);
	if (ret < 0) {
		dev_err(ctx->dev, "Failed to prepare DMA engine for FW loading, err: %x\n", ret);
		return ret;
	}

	memcpy(dmab.area, fwdata, fwsize);

	ctx->dsp_ops.trigger(ctx->dev, true);

	mdelay(1000);
	ret = sst_register_poll(ctx, BXT_HDA_ADSP_REG_FW_STATUS, FW_STATUS_MASK,
			BXT_FW_ROM_BASEFW_ENTERED,
			BXT_FW_ROM_BASEFW_ENTERED_TIMEOUT,
			"Firmware boot");

	ctx->dsp_ops.trigger(ctx->dev, false);
	ctx->dsp_ops.cleanup(ctx->dev, &dmab);

	return ret;
}

#define INSTANCE_ID 0
#define FW_MODULE_ID 0

static int sst_bxt_set_dsp_D0(struct sst_dsp_ctx *ctx)
{
	int ret = 0;
	struct dxstate_info dx;

	dev_dbg(ctx->dev, "In %s:\n", __func__);

	ctx->ipc->boot_complete = false;
	dx.core_mask = DSP_CORES_MASK;
	dx.dx_mask = ADSP_IPC_D0_MASK;

	dev_dbg(ctx->dev, "core mask=%x dx_mask=%x\n",
				dx.core_mask, dx.dx_mask);
	ret = sst_enable_dsp_core(ctx);
	if (ret < 0) {
		dev_err(ctx->dev, "enable dsp core failed ret: %d\n", ret);
		return ret;
	}

	/*enable interrupt*/
	ipc_int_enable(ctx);
	ipc_op_int_enable(ctx);

	ret = wait_event_timeout(ctx->ipc->boot_wait, ctx->ipc->boot_complete,
					msecs_to_jiffies(IPC_BOOT_MSECS));
	if (ret == 0) {
		dev_err(ctx->dev, "ipc: error DSP boot timeout\n");
		return -EIO;
	}

	ret = ipc_set_dx(ctx->ipc, INSTANCE_ID, FW_MODULE_ID, &dx);
	if (ret < 0) {
		dev_err(ctx->dev, "Failed to set DSP to D0 state\n");
		sst_disable_dsp_core(ctx);
		return ret;
	}
	return ret;
}

static int sst_bxt_set_dsp_D3(struct sst_dsp_ctx *ctx)
{
	int ret = 0;
	struct dxstate_info dx;

	dev_dbg(ctx->dev, "In %s:\n", __func__);

	dx.core_mask = DSP_CORES_MASK;
	dx.dx_mask = ADSP_IPC_D3_MASK;

	dev_dbg(ctx->dev, "core mask=%x dx_mask=%x\n",
				 dx.core_mask, dx.dx_mask);
	ret = ipc_set_dx(ctx->ipc, INSTANCE_ID, FW_MODULE_ID, &dx);
	if (ret < 0) {
		dev_err(ctx->dev, "Failed to set DSP to D0 state\n");
		return ret;
	}

	ret = sst_disable_dsp_core(ctx);
	if (ret < 0) {
		dev_err(ctx->dev, "disbale dsp core failed ret: %d\n", ret);
		ret = -EIO;
	}
	return 0;
}

static int sst_bxt_load_base_firmware(struct sst_dsp_ctx *ctx)
{
	int ret = 0;
	const struct firmware *fw = NULL;
	u32 fw_preload_page_count = 0;
	u32 base_fw_size = 0;
	struct sst_fw_image_manifest *manifest;
	ctx->ipc->boot_complete = false;

	ret = request_firmware(&fw, "dsp_fw_release.bin", ctx->dev);
	if (ret < 0) {
		dev_err(ctx->dev, "Request firmware failed %d\n", ret);
		goto sst_load_base_firmware_failed;
	}

	ret = sst_bxt_prepare_fw(ctx);
	if (ret < 0)
		goto sst_load_base_firmware_failed;

	manifest = (struct sst_fw_image_manifest *)fw->data;
	fw_preload_page_count =
		manifest->adsp_fw_bin_desc.header.preload_page_count;
	base_fw_size = fw_preload_page_count * PAGE_SIZE;

	if (base_fw_size > fw->size) {
		dev_err(ctx->dev, "Preloaded base fw size is bigger then whole fw image");
		ret = -EIO;
		goto sst_load_base_firmware_failed;
	}

	ret = sst_transfer_fw_host_dma(ctx, fw->data, fw->size);
	if (ret < 0) {
		dev_err(ctx->dev, "Transfer firmware failed %d\n", ret);
		sst_disable_dsp_core(ctx);
	} else {
		dev_dbg(ctx->dev, "Firmware download successfull\n");
		ret = wait_event_timeout(ctx->ipc->boot_wait, ctx->ipc->boot_complete,
						msecs_to_jiffies(IPC_BOOT_MSECS));
		if (ret == 0) {
			dev_err(ctx->dev, "DSP boot failed, FW Ready timed-out\n");
			sst_disable_dsp_core(ctx);
			ret = -EIO;
		}
	}

sst_load_base_firmware_failed:
	release_firmware(fw);
	return ret;
}
