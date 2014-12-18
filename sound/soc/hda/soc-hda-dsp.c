/*
 *  soc_hda_dsp.c - HDA dsp Inteface function for HDA devices
 *
 *  Copyright (C) 2014 Intel Corp
 *  Author: Jeeja KP<jeeja.kp@intel.com>
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

#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <sound/hda_register.h>
#include <sound/hda_controller.h>
#include <sound/hda_dma.h>
#include <sound/soc-hda-sst-dsp.h>
#include "soc-hda-controller.h"

int dsp_register(struct azx *chip)
{
	void __iomem *mmio_base;
	struct sst_dsp_loader_ops dsp_ops;
	int irq = chip->pci->irq;
	struct snd_soc_azx *schip =
			container_of(chip, struct snd_soc_azx, hda_azx);
	int ret = 0;

	azx_ppcap_int_enable(chip, true);

	/*read the BAR of the ADSP MMIO */
	mmio_base = pci_ioremap_bar(chip->pci, 4);
	if (mmio_base == NULL) {
		dev_err(chip->dev, "ioremap error\n");
		return -ENXIO;
	}
	dsp_ops.init = azx_load_dsp_init;
	dsp_ops.prepare = azx_load_dsp_prepare;
	dsp_ops.trigger = azx_load_dsp_trigger;
	dsp_ops.cleanup = azx_load_dsp_cleanup;
	dsp_ops.alloc_dma_buf  = azx_alloc_dma_buf;
	dsp_ops.free_dma_buf  = azx_free_dma_buf;

	if (chip->pci->device == 0x0a98)
		ret = sst_bxt_init(chip->dev, mmio_base, irq,
			dsp_ops, &schip->dsp);
	else if (chip->pci->device == 0x9d70)
		ret = sst_skl_init(chip->dev, mmio_base, irq,
			dsp_ops, &schip->dsp);

	dev_dbg(chip->dev, "dsp registeration status=%d\n", ret);
	return ret;
}

void azx_dsp_unregister(struct azx *chip)
{
	struct snd_soc_azx *schip =
			container_of(chip, struct snd_soc_azx, hda_azx);

	sst_dsp_free(schip->dsp);
	azx_ppcap_int_enable(chip, false);
	if (schip->dsp->mmio_base)
		iounmap(schip->dsp->mmio_base);
	return;
}

int azx_load_i2s_machine(struct azx *chip)
{
	struct platform_device *pdev;
	int ret;
	struct snd_soc_azx *schip =
			container_of(chip, struct snd_soc_azx, hda_azx);

	pdev = platform_device_alloc("morg_florida", -1);
	if (pdev == NULL) {
		dev_err(chip->dev, "failed device alloc failed\n");
		return -EIO;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		dev_err(chip->dev, "failed to add i2s machine device\n");
		platform_device_put(pdev);
		return -1;
	}
	schip->i2s_pdev = pdev;
	return 0;
}

void azx_i2s_machine_device_unregister(struct azx *chip)
{
	struct snd_soc_azx *schip =
			container_of(chip, struct snd_soc_azx, hda_azx);

	if (schip->i2s_pdev)
		platform_device_unregister(schip->i2s_pdev);
}

/* use the first stream for loading DSP */
static struct azx_dev *
azx_get_dsp_loader_dev(struct azx *chip)
{
	dev_dbg(chip->dev, "playback index =%x\n", chip->playback_index_offset);
	return &chip->azx_dev[chip->playback_index_offset];
}

int azx_load_dsp_prepare(struct device *dev, unsigned int format,
				unsigned int byte_size,
				struct snd_dma_buffer *bufp)
{
	struct pci_dev *pci = to_pci_dev(dev);
	struct azx *chip  = pci_get_drvdata(pci);
	u32 *bdl;
	struct azx_dev *azx_dev;
	int ret;
	unsigned long cookie;


	dev_dbg(chip->dev, "format: %x, byte_size: %x, bufp:%p", format, byte_size, bufp);

	azx_dev = azx_get_dsp_loader_dev(chip);

	dsp_lock(azx_dev);
	spin_lock_irqsave(&chip->reg_lock, cookie);
	if (azx_dev->running || azx_dev->locked) {
		spin_unlock_irqrestore(&chip->reg_lock, cookie);
		ret = -EBUSY;
		goto unlock;
	}
	azx_dev->prepared = 0;
	chip->saved_azx_dev = *azx_dev;
	azx_dev->locked = 1;
	spin_unlock_irqrestore(&chip->reg_lock, cookie);

	ret = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV_SG,
				snd_dma_pci_data(chip->pci),
				byte_size, bufp);
	if (ret < 0)
		goto err_alloc;

	mark_pages_wc(chip, bufp, true);
	azx_dev->bufsize = byte_size;
	azx_dev->period_bytes = byte_size;
	azx_dev->format_val = format;

	if (chip->ppcap_offset) {
		azx_writel_andor(
			chip,
			chip->ppcap_offset + ICH6_REG_PP_PPCTL,
			~0x1FFF,
			0x1FFF);
	/*	azx_dma_decouple(chip, azx_dev, true);*/
	}

	/* reset BDL address */
	azx_sd_writel(chip, azx_dev, SD_BDLPL, 0);
	azx_sd_writel(chip, azx_dev, SD_BDLPU, 0);

	azx_dev->frags = 0;

	bdl = (u32 *)azx_dev->bdl.area;

	ret = setup_bdle(chip, bufp, azx_dev, &bdl, 0, byte_size, 0, 0);
	if (ret < 0)
		goto error;

	azx_setup_controller(chip, azx_dev, 1);

	if (chip->ppcap_offset && chip->spbcap_offset)
		azx_spbcap_one_enable(chip, true, azx_dev->index);

	dsp_unlock(azx_dev);
	dev_dbg(chip->dev, "stream_tag: 0x%x", azx_dev->stream_tag);
	return azx_dev->stream_tag;

 error:
	mark_pages_wc(chip, bufp, false);
	snd_dma_free_pages(bufp);
 err_alloc:
	spin_lock_irqsave(&chip->reg_lock, cookie);
	if (azx_dev->opened)
		*azx_dev = chip->saved_azx_dev;
	azx_dev->locked = 0;
	spin_unlock_irqrestore(&chip->reg_lock, cookie);
 unlock:
	dsp_unlock(azx_dev);
	return ret;
}

void azx_dev_reg_dump(struct azx *chip, struct azx_dev *azx_dev)
{
	dev_dbg(chip->dev, "*******azx_dev index: %x SD_CTL:%x SD_STS:%x \
		SD_LPIB:%x SD_CBL:%x SD_LVI:%x SD_FIFOW:%x SD_FIFOSIZE:%x \
		SD_FMT:%x SD_FIFOL:%x SD_BDPL:%x SD_BDLPU:%x\n",
	azx_dev->index,
	azx_sd_readl(chip, azx_dev, SD_CTL) & ((1 << 24) - 1),
	azx_sd_readb(chip, azx_dev, SD_STS),
	azx_sd_readl(chip, azx_dev, SD_LPIB),
	azx_sd_readl(chip, azx_dev, SD_CBL),
	azx_sd_readw(chip, azx_dev, SD_LVI),
	azx_sd_readw(chip, azx_dev, SD_FIFOW),
	azx_sd_readw(chip, azx_dev, SD_FIFOSIZE),
	azx_sd_readw(chip, azx_dev, SD_FORMAT),
	azx_sd_readw(chip, azx_dev, SD_FIFOL),
	azx_sd_readl(chip, azx_dev, SD_BDLPL),
	azx_sd_readl(chip, azx_dev, SD_BDLPU));

	if (azx_readl_alt(chip, (chip->ppcap_offset + ICH6_REG_PP_PPCTL)) &
		PPCTL_PROCEN(azx_dev->index)) {
		dev_dbg(chip->dev, "azx_dev index: %x decoupled,\nPPHCLLPL:%x,\nPPHCLLPU:%x,"
			"\nPPHC_LDPL:%x,\nPPHC_LDPU:%x",
		azx_dev->index,
		azx_pphc_readl(chip, azx_dev, 0),
		azx_pphc_readl(chip, azx_dev, 4),
		azx_pphc_readl(chip, azx_dev, 8),
		azx_pphc_readl(chip, azx_dev, 0xC));

		dev_dbg(chip->dev, "azx_dev index: %x,\nPPLCCTL:%x,\nPPLCFMT:%x,"
			"\nPPLCLLPL:%x,\nPPLCLLPU:%x",
			azx_dev->index,
			azx_pplc_readl(chip, azx_dev, REG_PPLCCTL),
			azx_pplc_readl(chip, azx_dev, REG_PPLCFMT),
			azx_pplc_readl(chip, azx_dev, REG_PPLCLLPL),
			azx_pplc_readl(chip, azx_dev, REG_PPLCLLPU));
	} else {
		dev_dbg(chip->dev, "azx_dev index: %x Not decoupled", azx_dev->index);
	}
}

void azx_load_dsp_trigger(struct device *dev, bool start)
{
	struct azx *chip = dev_get_drvdata(dev);
	struct azx_dev *azx_dev = azx_get_dsp_loader_dev(chip);
	int i = 20;

	dev_dbg(chip->dev, "In%s start=%d\n", __func__, start);
	if (start) {
		azx_dev_reg_dump(chip, azx_dev);
		azx_stream_start(chip, azx_dev);

		udelay(100);

		azx_writel_alt(
			chip,
			chip->spbcap_offset +
			(ICH6_REG_SPB_XBASE + ICH6_REG_SPB_XINTERVAL * azx_dev->index),
			azx_sd_readl(chip, azx_dev, SD_CBL));
		do {
			dev_dbg(chip->dev, "LPIB position=%x\n", azx_sd_readl(chip, azx_dev, SD_LPIB));
			i--;
		} while (i > 0);

	} else
		azx_stream_stop(chip, azx_dev);
	azx_dev->running = start;
}

void azx_load_dsp_cleanup(struct device *dev,
				 struct snd_dma_buffer *dmab)
{
	struct azx *chip = dev_get_drvdata(dev);
	struct azx_dev *azx_dev = azx_get_dsp_loader_dev(chip);
	unsigned long cookie;

	if (!dmab->area || !azx_dev->locked)
		return;

	dsp_lock(azx_dev);
	/* reset BDL address */
	azx_stream_reset(chip, azx_dev);

	azx_sd_writel(chip, azx_dev, SD_BDLPL, 0);
	azx_sd_writel(chip, azx_dev, SD_BDLPU, 0);
	azx_sd_writel(chip, azx_dev, SD_CTL, 0);
	azx_dev->bufsize = 0;
	azx_dev->period_bytes = 0;
	azx_dev->format_val = 0;

	mark_pages_wc(chip, dmab, false);
	snd_dma_free_pages(dmab);
	dmab->area = NULL;

	spin_lock_irqsave(&chip->reg_lock, cookie);
	if (chip->ppcap_offset)
		azx_writel_andor(
			chip,
			chip->ppcap_offset + ICH6_REG_PP_PPCTL,
			~0x1FFF,
			0);

	if (azx_dev->opened)
		*azx_dev = chip->saved_azx_dev;
	azx_dev->locked = 0;
	spin_unlock_irqrestore(&chip->reg_lock, cookie);
	dsp_unlock(azx_dev);
}

int azx_alloc_dma_buf(struct device *dev,
		struct snd_dma_buffer *dmab, u32 size)
{
	int err = 0;
	struct pci_dev *pci = to_pci_dev(dev);
	struct azx *chip  = pci_get_drvdata(pci);

	if (chip == NULL)
		return -1;

	dev_dbg(chip->dev, "In%s size=%x\n", __func__, size);
	err = snd_dma_alloc_pages(SNDRV_DMA_TYPE_DEV,
				snd_dma_pci_data(chip->pci), size, dmab);
	return err;
}

int azx_free_dma_buf(struct device *dev,
		struct snd_dma_buffer *dmab)
{
	if (dmab == NULL)
		return 0;
	snd_dma_free_pages(dmab);
	return 0;
}

int azx_load_dsp_init(struct device *dev)
{
	struct pci_dev *pci = to_pci_dev(dev);
	struct azx *chip  = pci_get_drvdata(pci);
	int i;

	azx_stop_chip(chip);
	azx_enter_link_reset(chip);

	azx_exit_link_reset(chip);

	for (i = 0; i < chip->num_streams; i++)
		azx_stream_reset(chip, (chip->azx_dev + i));
	return 0;
}
