/*
 *  soc-hda-core.c - Implementation of primary ASoC Intel HD Audio driver
 *
 *  Copyright (C) 2014 Intel Corp
 *  Author: Jeeja KP <jeeja.kp@intel.com>
 *
 *  Copyright (c) 2004 Takashi Iwai <tiwai@suse.de>
 *                     PeiSen Hou <pshou@realtek.com.tw>
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
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/pci.h>
#include <linux/mutex.h>
#include <linux/reboot.h>
#include <linux/io.h>
#include <linux/pm_runtime.h>
#include <linux/clocksource.h>
#include <linux/time.h>
#include <linux/completion.h>

#ifdef CONFIG_X86
/* for snoop control */
#include <asm/pgtable.h>
#include <asm/cacheflush.h>
#endif
#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>
#include <linux/vgaarb.h>
#include <linux/vga_switcheroo.h>
#include <sound/hda_register.h>
#include <sound/hda_controller.h>
#include <sound/hda_dma.h>
#include <sound/hda_verbs.h>
#include <sound/soc-hda-bus.h>
#include "soc-hda-controller.h"

static char *model;
static int position_fix = -1;
static int bdl_pos_adj = -1;
static int probe_mask = -1;
static int probe_only;
static int jackpoll_ms;
static bool single_cmd;
static int enable_msi = -1;



#ifdef CONFIG_SND_HDA_INPUT_BEEP
static bool beep_mode = CONFIG_SND_HDA_INPUT_BEEP_MODE;
#endif


module_param(model, charp, 0444);
MODULE_PARM_DESC(model, "Use the given board model.");
module_param(position_fix, int, 0444);
MODULE_PARM_DESC(position_fix, "DMA pointer read method."
		 "(-1 = system default, 0 = auto, 1 = LPIB, 2 = POSBUF, 3 = VIACOMBO, 4 = COMBO).");
module_param(bdl_pos_adj, int, 0644);
MODULE_PARM_DESC(bdl_pos_adj, "BDL position adjustment offset.");
module_param(probe_mask, int, 0444);
MODULE_PARM_DESC(probe_mask, "Bitmask to probe codecs (default = -1).");
module_param(probe_only, int, 0444);
MODULE_PARM_DESC(probe_only, "Only probing and no codec initialization.");
module_param(jackpoll_ms, int, 0444);
MODULE_PARM_DESC(jackpoll_ms, "Ms between polling for jack events (default = 0, using unsol events only)");
module_param(single_cmd, bool, 0444);
MODULE_PARM_DESC(single_cmd, "Use single command to communicate with codecs "
		 "(for debugging only).");
module_param(enable_msi, bint, 0444);
MODULE_PARM_DESC(enable_msi, "Enable Message Signaled Interrupt (MSI)");

#ifdef CONFIG_SND_HDA_INPUT_BEEP
module_param(beep_mode, bool, 0444);
MODULE_PARM_DESC(beep_mode, "Select HDA Beep registration mode "
			    "(0=off, 1=on) (default=1).");
#endif

#ifdef CONFIG_PM

/* reset the HD-audio controller in power save mode.
 * this may give more power-saving, but will take longer time to
 * wake up.
 */
static bool power_save_controller = 1;
module_param(power_save_controller, bool, 0644);
MODULE_PARM_DESC(power_save_controller, "Reset controller in power save mode.");
#endif /* CONFIG_PM */

static int align_buffer_size = -1;
module_param(align_buffer_size, bint, 0644);
MODULE_PARM_DESC(align_buffer_size,
		"Force buffer and period sizes to be multiple of 128 bytes.");

#ifdef CONFIG_X86
static bool hda_snoop = true;
module_param_named(snoop, hda_snoop, bool, 0444);
MODULE_PARM_DESC(snoop, "Enable/disable snooping");
#else
#define hda_snoop		true
#endif


MODULE_LICENSE("GPL v2");
MODULE_SUPPORTED_DEVICE("{{Intel, ICH6},"
			 "{Intel, ICH6M},"
			 "{Intel, ICH7},"
			 "{Intel, ESB2},"
			 "{Intel, ICH8},"
			 "{Intel, ICH9},"
			 "{Intel, ICH10},"
			 "{Intel, PCH},"
			 "{Intel, CPT},"
			 "{Intel, PPT},"
			 "{Intel, LPT},"
			 "{Intel, LPT_LP},"
			 "{Intel, HPT},"
			 "{Intel, PBG},"
			 "{Intel, SCH},");
MODULE_DESCRIPTION("Intel HDA driver");

#if defined(CONFIG_PM) && defined(CONFIG_VGA_SWITCHEROO)
#if IS_ENABLED(CONFIG_SND_HDA_CODEC_HDMI)
#define SUPPORT_VGA_SWITCHEROO
#endif
#endif

/* driver types */
enum {
	AZX_DRIVER_ICH,
	AZX_DRIVER_PCH,
	AZX_DRIVER_SCH,
	AZX_DRIVER_HDMI,
	AZX_DRIVER_GENERIC,
	AZX_NUM_DRIVERS, /* keep this as last entry */
};

/* quirks for Intel PCH */
#define AZX_DCAPS_INTEL_PCH_NOPM \
	(AZX_DCAPS_SCH_SNOOP | AZX_DCAPS_BUFSIZE | \
	 AZX_DCAPS_COUNT_LPIB_DELAY)

#define AZX_DCAPS_INTEL_PCH \
	(AZX_DCAPS_INTEL_PCH_NOPM | AZX_DCAPS_PM_RUNTIME)

#define AZX_DCAPS_INTEL_HASWELL \
	(AZX_DCAPS_SCH_SNOOP | AZX_DCAPS_ALIGN_BUFSIZE | \
	 AZX_DCAPS_COUNT_LPIB_DELAY | AZX_DCAPS_PM_RUNTIME | \
	 AZX_DCAPS_I915_POWERWELL)

/*
 * VGA-switcher support
 */
#ifdef SUPPORT_VGA_SWITCHEROO
#define use_vga_switcheroo(chip)	((chip)->use_vga_switcheroo)
#else
#define use_vga_switcheroo(chip)	0
#endif
#ifdef CONFIG_X86
void __mark_pages_wc(struct azx *chip, struct snd_dma_buffer *dmab, bool on)
{
	int pages;

	if (azx_snoop(chip))
		return;
	if (!dmab || !dmab->area || !dmab->bytes)
		return;

#ifdef CONFIG_SND_DMA_SGBUF
	if (dmab->dev.type == SNDRV_DMA_TYPE_DEV_SG) {
		struct snd_sg_buf *sgbuf = dmab->private_data;
		if (on)
			set_pages_array_wc(sgbuf->page_table, sgbuf->pages);
		else
			set_pages_array_wb(sgbuf->page_table, sgbuf->pages);
		return;
	}
#endif

	pages = (dmab->bytes + PAGE_SIZE - 1) >> PAGE_SHIFT;
	if (on)
		set_memory_wc((unsigned long)dmab->area, pages);
	else
		set_memory_wb((unsigned long)dmab->area, pages);
}
#endif

static int azx_acquire_irq(struct azx *chip, int do_disconnect);

/*
 * initialize the PCI registers
 */
/* update bits in a PCI register byte */
static void update_pci_byte(struct pci_dev *pci, unsigned int reg,
			    unsigned char mask, unsigned char val)
{
	unsigned char data;

	pci_read_config_byte(pci, reg, &data);
	data &= ~mask;
	data |= (val & mask);
	pci_write_config_byte(pci, reg, data);
}

static void azx_init_pci(struct azx *chip)
{

	/* Clear bits 0-2 of PCI register TCSEL (at offset 0x44)
	 * TCSEL == Traffic Class Select Register, which sets PCI express QOS
	 * Ensuring these bits are 0 clears playback static on some HD Audio
	 * codecs.
	 * The PCI register TCSEL is defined in the Intel manuals.
	 */
	if (!(chip->driver_caps & AZX_DCAPS_NO_TCSEL)) {
		dev_dbg(chip->dev, "Clearing TCSEL\n");
		update_pci_byte(chip->pci, ICH6_PCIREG_TCSEL, 0x07, 0);
	}

	/* Enable SCH/PCH snoop if needed */
	if (chip->driver_caps & AZX_DCAPS_SCH_SNOOP) {
		unsigned short snoop;
		pci_read_config_word(chip->pci, INTEL_SCH_HDA_DEVC, &snoop);
		if ((!azx_snoop(chip) && !(snoop & INTEL_SCH_HDA_DEVC_NOSNOOP)) ||
		    (azx_snoop(chip) && (snoop & INTEL_SCH_HDA_DEVC_NOSNOOP))) {
			snoop &= ~INTEL_SCH_HDA_DEVC_NOSNOOP;
			if (!azx_snoop(chip))
				snoop |= INTEL_SCH_HDA_DEVC_NOSNOOP;
			pci_write_config_word(chip->pci, INTEL_SCH_HDA_DEVC, snoop);
			pci_read_config_word(chip->pci,
				INTEL_SCH_HDA_DEVC, &snoop);
		}
		dev_dbg(chip->dev, "SCH snoop: %s\n",
			(snoop & INTEL_SCH_HDA_DEVC_NOSNOOP) ?
			"Disabled" : "Enabled");
	}
}

static int azx_position_ok(struct azx *chip, struct azx_dev *azx_dev);

/* called from IRQ */
static int azx_position_check(struct azx *chip, struct azx_dev *azx_dev)
{
	int ok;

	ok = azx_position_ok(chip, azx_dev);
	if (ok == 1) {
		azx_dev->irq_pending = 0;
		return ok;
	} else if (ok == 0 && chip->bus && chip->bus->workq) {
		/* bogus IRQ, process it later */
		azx_dev->irq_pending = 1;
		queue_work(chip->bus->workq, &chip->irq_pending_work);
	}
	return 0;
}

unsigned int azx_get_position(struct azx *chip, struct azx_dev *azx_dev,
				     bool with_check)
{
	int delay = 0;

	/*FIXME get the codec delay */
	return azx_calculate_position(chip, azx_dev, with_check, delay);
}

/*
 * Check whether the current DMA position is acceptable for updating
 * periods.  Returns non-zero if it's OK.
 *
 * Many HD-audio controllers appear pretty inaccurate about
 * the update-IRQ timing.  The IRQ is issued before actually the
 * data is processed.  So, we need to process it afterwords in a
 * workqueue.
 */
int azx_position_ok(struct azx *chip, struct azx_dev *azx_dev)
{
	u32 wallclk;
	unsigned int pos;

	wallclk = azx_readl(chip, WALLCLK) - azx_dev->start_wallclk;
	if (wallclk < (azx_dev->period_wallclk * 2) / 3)
		return -1;	/* bogus (too early) interrupt */

	pos = azx_get_position(chip, azx_dev, true);

	if (WARN_ONCE(!azx_dev->period_bytes,
		      "hda-intel: zero azx_dev->period_bytes"))
		return -1; /* this shouldn't happen! */
	if (wallclk < (azx_dev->period_wallclk * 5) / 4 &&
	    pos % azx_dev->period_bytes > azx_dev->period_bytes / 2)
		/* NG - it's below the first next period boundary */
		return chip->bdl_pos_adj[chip->dev_index] ? 0 : -1;
	azx_dev->start_wallclk += wallclk;
	return 1; /* OK, it's fine */
}

/*
 * The work for pending PCM period updates.
 */
static void azx_irq_pending_work(struct work_struct *work)
{
	struct azx *chip = container_of(work, struct azx, irq_pending_work);
	int i, pending, ok;
	unsigned long cookie;

	if (!chip->irq_pending_warned) {
		dev_info(chip->dev,
			 "IRQ timing workaround is activated. Suggest a bigger bdl_pos_adj.\n");
		chip->irq_pending_warned = 1;
	}

	for (;;) {
		pending = 0;
		spin_lock_irqsave(&chip->reg_lock, cookie);
		for (i = 0; i < chip->num_streams; i++) {
			struct azx_dev *azx_dev = &chip->azx_dev[i];
			if (!azx_dev->irq_pending ||
			    !azx_dev->substream ||
			    !azx_dev->running)
				continue;
			ok = azx_position_ok(chip, azx_dev);
			if (ok > 0) {
				azx_dev->irq_pending = 0;
				spin_unlock_irqrestore(&chip->reg_lock, cookie);
				snd_pcm_period_elapsed(azx_dev->substream);
				spin_lock_irqsave(&chip->reg_lock, cookie);
			} else if (ok < 0) {
				pending = 0;	/* too early */
			} else
				pending++;
		}
		spin_unlock_irqrestore(&chip->reg_lock, cookie);
		if (!pending)
			return;
		msleep(1);
	}
}

/* clear irq_pending flags and assure no on-going workq */
static void azx_clear_irq_pending(struct azx *chip)
{
	int i;
	unsigned long cookie;

	spin_lock_irqsave(&chip->reg_lock, cookie);
	for (i = 0; i < chip->num_streams; i++)
		chip->azx_dev[i].irq_pending = 0;
	spin_unlock_irqrestore(&chip->reg_lock, cookie);
}

static int azx_acquire_irq(struct azx *chip, int do_disconnect)
{
	unsigned long flags = 0;

	if (chip->ppcap_offset)
		flags = IRQF_SHARED;
	else
		flags = chip->msi ? 0 : IRQF_SHARED;

	if (request_threaded_irq(chip->pci->irq, azx_interrupt,
			azx_threaded_handler, flags,
			KBUILD_MODNAME, chip)) {
		dev_err(chip->dev,
			"unable to grab IRQ %d, disabling device\n",
			chip->pci->irq);
		/*FIXME if (do_disconnect)
			snd_card_disconnect(chip->card);*/
		return -1;
	}
	chip->irq = chip->pci->irq;
	pci_intx(chip->pci, !chip->msi);
	return 0;
}

#if defined(CONFIG_PM_SLEEP) || defined(SUPPORT_VGA_SWITCHEROO)
/*
 * power management
 */
static int azx_suspend(struct device *dev)
{
	struct pci_dev *pci = to_pci_dev(dev);
	struct azx *chip  = pci_get_drvdata(pci);

	if (chip->disabled)
		return 0;

	azx_clear_irq_pending(chip);
	azx_stop_chip(chip);
	azx_enter_link_reset(chip);
	if (chip->irq >= 0) {
		free_irq(chip->irq, chip);
		chip->irq = -1;
	}
	if (chip->msi)
		pci_disable_msi(chip->pci);
	pci_disable_device(pci);
	pci_save_state(pci);
	pci_set_power_state(pci, PCI_D3hot);
	return 0;
}

static int azx_resume(struct device *dev)
{
	struct pci_dev *pci = to_pci_dev(dev);
	struct azx *chip = pci_get_drvdata(pci);

	if (chip->disabled)
		return 0;

	pci_set_power_state(pci, PCI_D0);
	pci_restore_state(pci);

	if (pci_enable_device(pci) < 0) {
		dev_err(dev, "hda-intel: pci_enable_device failed, "
		       "disabling device\n");
		/*FIXME snd_card_disconnect(card);*/
		return -EIO;
	}
	pci_set_master(pci);
	if (chip->msi)
		if (pci_enable_msi(pci) < 0)
			chip->msi = 0;
	if (azx_acquire_irq(chip, 1) < 0)
		return -EIO;
	azx_init_pci(chip);

	azx_init_chip(chip, 1);

	return 0;
}
#endif /* CONFIG_PM_SLEEP || SUPPORT_VGA_SWITCHEROO */

#ifdef CONFIG_PM_RUNTIME
static int azx_runtime_suspend(struct device *dev)
{
	struct pci_dev *pci = to_pci_dev(dev);
	struct azx *chip = pci_get_drvdata(pci);

	if (chip->disabled)
		return 0;
	if (!(chip->driver_caps & AZX_DCAPS_PM_RUNTIME))
		return 0;

	/* enable controller wake up event */
	azx_writew(chip, WAKEEN, azx_readw(chip, WAKEEN) |
		  STATESTS_INT_MASK);

	azx_stop_chip(chip);
	azx_enter_link_reset(chip);
	azx_clear_irq_pending(chip);
	return 0;
}

static int azx_runtime_resume(struct device *dev)
{
	struct pci_dev *pci = to_pci_dev(dev);
	struct azx *chip = pci_get_drvdata(pci);
	int status;

	if (chip->disabled)
		return 0;

	if (!(chip->driver_caps & AZX_DCAPS_PM_RUNTIME))
		return 0;

	/*FIXME
	if (chip->driver_caps & AZX_DCAPS_I915_POWERWELL)
		hda_display_power(true);*/

	/* Read STATESTS before controller reset */
	status = azx_readw(chip, STATESTS);

	azx_init_pci(chip);
	azx_init_chip(chip, true);
	/* disable controller Wake Up event*/
	azx_writew(chip, WAKEEN, azx_readw(chip, WAKEEN) &
			~STATESTS_INT_MASK);

	return 0;
}

static int azx_runtime_idle(struct device *dev)
{
	struct pci_dev *pci = to_pci_dev(dev);
	struct snd_soc_azx *schip = pci_get_drvdata(pci);
	struct azx *chip = &schip->hda_azx;

	if (chip->disabled)
		return 0;

	if (!power_save_controller ||
	    !(chip->driver_caps & AZX_DCAPS_PM_RUNTIME))
		return -EBUSY;

	return 0;
}

#endif /* CONFIG_PM_RUNTIME */

#ifdef CONFIG_PM
static const struct dev_pm_ops azx_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(azx_suspend, azx_resume)
	SET_RUNTIME_PM_OPS(azx_runtime_suspend, azx_runtime_resume, azx_runtime_idle)
};

#define AZX_PM_OPS	&azx_pm
#else
#define AZX_PM_OPS	NULL
#endif /* CONFIG_PM */

/*
 * reboot notifier for hang-up problem at power-down
 */
static int azx_halt(struct notifier_block *nb, unsigned long event, void *buf)
{
	/*FIME struct azx *chip = container_of(nb, struct azx, reboot_notifier);
	snd_hda_bus_reboot_notify(chip->bus);
	azx_stop_chip(chip); */
	return NOTIFY_OK;
}

static void azx_notifier_register(struct azx *chip)
{
	chip->reboot_notifier.notifier_call = azx_halt;
	register_reboot_notifier(&chip->reboot_notifier);
}

static void azx_notifier_unregister(struct azx *chip)
{
	if (chip->reboot_notifier.notifier_call)
		unregister_reboot_notifier(&chip->reboot_notifier);
}

static int azx_probe_continue(struct azx *chip);

/*FIXME */
#if 0
#ifdef SUPPORT_VGA_SWITCHEROO
static struct pci_dev *get_bound_vga(struct pci_dev *pci);

static void azx_vs_set_state(struct pci_dev *pci,
			     enum vga_switcheroo_state state)
{
	struct snd_card *card = pci_get_drvdata(pci);
	struct azx *chip = card->private_data;
	bool disabled;

	wait_for_completion(&chip->probe_wait);
	if (chip->init_failed)
		return;

	disabled = (state == VGA_SWITCHEROO_OFF);
	if (chip->disabled == disabled)
		return;

	if (!chip->bus) {
		chip->disabled = disabled;
		if (!disabled) {
			dev_info(chip->dev,
				 "Start delayed initialization\n");
			if (azx_probe_continue(chip) < 0) {
				dev_err(chip->dev, "initialization error\n");
				chip->init_failed = true;
			}
		}
	} else {
		dev_info(chip->dev, "%s via VGA-switcheroo\n",
			 disabled ? "Disabling" : "Enabling");
		if (disabled) {
			pm_runtime_put_sync_suspend(card->dev);
			azx_suspend(card->dev);
			/* when we get suspended by vga switcheroo we end up in D3cold,
			 * however we have no ACPI handle, so pci/acpi can't put us there,
			 * put ourselves there */
			pci->current_state = PCI_D3cold;
			chip->disabled = true;
			if (snd_hda_lock_devices(chip->bus))
				dev_warn(chip->dev,
					 "Cannot lock devices!\n");
		} else {
			snd_hda_unlock_devices(chip->bus);
			pm_runtime_get_noresume(card->dev);
			chip->disabled = false;
			azx_resume(card->dev);
		}
	}
}

static bool azx_vs_can_switch(struct pci_dev *pci)
{
	struct snd_card *card = pci_get_drvdata(pci);
	struct azx *chip = card->private_data;

	wait_for_completion(&chip->probe_wait);
	if (chip->init_failed)
		return false;
	if (chip->disabled || !chip->bus)
		return true;
	if (snd_hda_lock_devices(chip->bus))
		return false;
	snd_hda_unlock_devices(chip->bus);
	return true;
}

static void init_vga_switcheroo(struct azx *chip)
{
	struct pci_dev *p = get_bound_vga(chip->pci);
	if (p) {
		dev_info(chip->dev,
			 "Handle VGA-switcheroo audio client\n");
		chip->use_vga_switcheroo = 1;
		pci_dev_put(p);
	}
}

static const struct vga_switcheroo_client_ops azx_vs_ops = {
	.set_gpu_state = azx_vs_set_state,
	.can_switch = azx_vs_can_switch,
};

static int register_vga_switcheroo(struct azx *chip)
{
	int err;

	if (!chip->use_vga_switcheroo)
		return 0;
	/* FIXME: currently only handling DIS controller
	 * is there any machine with two switchable HDMI audio controllers?
	 */
	err = vga_switcheroo_register_audio_client(chip->pci, &azx_vs_ops,
						    VGA_SWITCHEROO_DIS,
						    chip->bus != NULL);
	if (err < 0)
		return err;
	chip->vga_switcheroo_registered = 1;

	/* register as an optimus hdmi audio power domain */
	vga_switcheroo_init_domain_pm_optimus_hdmi_audio(chip->dev,
							 &chip->hdmi_pm_domain);
	return 0;
}
#else
#define init_vga_switcheroo(chip)		/* NOP */
#define register_vga_switcheroo(chip)		0
#define check_hdmi_disabled(pci)	false
#endif /* SUPPORT_VGA_SWITCHER */
#endif

/*
 * destructor
 */
static int azx_free(struct azx *chip)
{
	struct pci_dev *pci = chip->pci;
	int i;

	if ((chip->driver_caps & AZX_DCAPS_PM_RUNTIME)
			&& chip->running)
		pm_runtime_get_noresume(&pci->dev);
	azx_notifier_unregister(chip);

	chip->init_failed = 1; /* to be sure */
	complete_all(&chip->probe_wait);

	/*FIXME
	if (use_vga_switcheroo(chip)) {
		if (chip->disabled && chip->bus)
			snd_hda_unlock_devices(chip->bus);
		if (chip->vga_switcheroo_registered)
			vga_switcheroo_unregister_client(chip->pci);
	}*/

	if (chip->initialized) {
		azx_clear_irq_pending(chip);
		for (i = 0; i < chip->num_streams; i++)
			azx_stream_stop(chip, &chip->azx_dev[i]);
		azx_stop_chip(chip);
	}

	if (chip->irq >= 0)
		free_irq(chip->irq, (void *)chip);
	if (chip->msi)
		pci_disable_msi(chip->pci);
	if (chip->remap_addr)
		iounmap(chip->remap_addr);

	azx_free_stream_pages(chip);
	if (chip->region_requested)
		pci_release_regions(chip->pci);
	pci_disable_device(chip->pci);
	kfree(chip->azx_dev);
	/*FIXME if (chip->driver_caps & AZX_DCAPS_I915_POWERWELL) {
		hda_display_power(false);
		hda_i915_exit();
	}*/
	kfree(chip);

	return 0;
}

/*FIXME */
#if 0
#ifdef SUPPORT_VGA_SWITCHEROO
/*
 * Check of disabled HDMI controller by vga-switcheroo
 */
static struct pci_dev *get_bound_vga(struct pci_dev *pci)
{
	struct pci_dev *p;

	/* check only discrete GPU */
	switch (pci->vendor) {
	case PCI_VENDOR_ID_ATI:
	case PCI_VENDOR_ID_AMD:
	case PCI_VENDOR_ID_NVIDIA:
		if (pci->devfn == 1) {
			p = pci_get_domain_bus_and_slot(pci_domain_nr(pci->bus),
							pci->bus->number, 0);
			if (p) {
				if ((p->class >> 8) == PCI_CLASS_DISPLAY_VGA)
					return p;
				pci_dev_put(p);
			}
		}
		break;
	}
	return NULL;
}

static bool check_hdmi_disabled(struct pci_dev *pci)
{
	bool vga_inactive = false;
	struct pci_dev *p = get_bound_vga(pci);

	if (p) {
		if (vga_switcheroo_get_client_state(p) == VGA_SWITCHEROO_OFF)
			vga_inactive = true;
		pci_dev_put(p);
	}
	return vga_inactive;
}
#endif /* SUPPORT_VGA_SWITCHEROO */
#endif

/*
 * white/black-listing for position_fix
 */
static struct snd_pci_quirk position_fix_list[] = {
	SND_PCI_QUIRK(0x1028, 0x01cc, "Dell D820", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x1028, 0x01de, "Dell Precision 390", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x103c, 0x306d, "HP dv3", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x1043, 0x813d, "ASUS P5AD2", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x1043, 0x81b3, "ASUS", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x1043, 0x81e7, "ASUS M2V", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x104d, 0x9069, "Sony VPCS11V9E", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x10de, 0xcb89, "Macbook Pro 7,1", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x1297, 0x3166, "Shuttle", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x1458, 0xa022, "ga-ma770-ud3", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x1462, 0x1002, "MSI Wind U115", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x1565, 0x8218, "Biostar Microtech", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x1849, 0x0888, "775Dual-VSTA", POS_FIX_LPIB),
	SND_PCI_QUIRK(0x8086, 0x2503, "DG965OT AAD63733-203", POS_FIX_LPIB),
	{}
};

static int check_position_fix(struct azx *chip, int fix)
{
	const struct snd_pci_quirk *q;

	switch (fix) {
	case POS_FIX_AUTO:
	case POS_FIX_LPIB:
	case POS_FIX_POSBUF:
	case POS_FIX_VIACOMBO:
	case POS_FIX_COMBO:
		return fix;
	}

	q = snd_pci_quirk_lookup(chip->pci, position_fix_list);
	if (q) {
		dev_info(chip->dev,
			 "position_fix set to %d for device %04x:%04x\n",
			 q->value, q->subvendor, q->subdevice);
		return q->value;
	}

	/* Check VIA/ATI HD Audio Controller exist */
	if (chip->driver_caps & AZX_DCAPS_POSFIX_VIA) {
		dev_dbg(chip->dev, "Using VIACOMBO position fix\n");
		return POS_FIX_VIACOMBO;
	}
	if (chip->driver_caps & AZX_DCAPS_POSFIX_LPIB) {
		dev_dbg(chip->dev, "Using LPIB position fix\n");
		return POS_FIX_LPIB;
	}
	return POS_FIX_AUTO;
}

/*
 * black-lists for probe_mask
 */
static struct snd_pci_quirk probe_mask_list[] = {
	/* Thinkpad often breaks the controller communication when accessing
	 * to the non-working (or non-existing) modem codec slot.
	 */
	SND_PCI_QUIRK(0x1014, 0x05b7, "Thinkpad Z60", 0x01),
	SND_PCI_QUIRK(0x17aa, 0x2010, "Thinkpad X/T/R60", 0x01),
	SND_PCI_QUIRK(0x17aa, 0x20ac, "Thinkpad X/T/R61", 0x01),
	/* broken BIOS */
	SND_PCI_QUIRK(0x1028, 0x20ac, "Dell Studio Desktop", 0x01),
	/* including bogus ALC268 in slot#2 that conflicts with ALC888 */
	SND_PCI_QUIRK(0x17c0, 0x4085, "Medion MD96630", 0x01),
	/* forced codec slots */
	SND_PCI_QUIRK(0x1043, 0x1262, "ASUS W5Fm", 0x103),
	SND_PCI_QUIRK(0x1046, 0x1262, "ASUS W5F", 0x103),
	/* WinFast VP200 H (Teradici) user reported broken communication */
	SND_PCI_QUIRK(0x3a21, 0x040d, "WinFast VP200 H", 0x101),
	{}
};

#define AZX_FORCE_CODEC_MASK	0x100

static void check_probe_mask(struct azx *chip)
{
	const struct snd_pci_quirk *q;

	chip->codec_probe_mask = probe_mask;
	if (chip->codec_probe_mask == -1) {
		q = snd_pci_quirk_lookup(chip->pci, probe_mask_list);
		if (q) {
			dev_info(chip->dev,
				 "probe_mask set to 0x%x for device %04x:%04x\n",
			 q->value, q->subvendor, q->subdevice);
			 chip->codec_probe_mask = q->value;
		}
	}

	/* check forced option */
	if (chip->codec_probe_mask != -1 &&
	    (chip->codec_probe_mask & AZX_FORCE_CODEC_MASK)) {
		chip->codec_mask = chip->codec_probe_mask & 0xff;
		dev_info(chip->dev, "codec_mask forced to 0x%x\n",
		       chip->codec_mask);
	}
}

/*
 * white/black-list for enable_msi
 */
static struct snd_pci_quirk msi_black_list[] = {
	SND_PCI_QUIRK(0x1043, 0x81f2, "ASUS", 0), /* Athlon64 X2 + nvidia */
	SND_PCI_QUIRK(0x1043, 0x81f6, "ASUS", 0), /* nvidia */
	SND_PCI_QUIRK(0x1043, 0x822d, "ASUS", 0), /* Athlon64 X2 + nvidia MCP55 */
	SND_PCI_QUIRK(0x1179, 0xfb44, "Toshiba Satellite C870", 0), /* AMD Hudson */
	SND_PCI_QUIRK(0x1849, 0x0888, "ASRock", 0), /* Athlon64 X2 + nvidia */
	SND_PCI_QUIRK(0xa0a0, 0x0575, "Aopen MZ915-M", 0), /* ICH6 */
	{}
};

static void check_msi(struct azx *chip)
{
	const struct snd_pci_quirk *q;

	if (enable_msi >= 0) {
		chip->msi = !!enable_msi;
		return;
	}
	chip->msi = 1;	/* enable MSI as default */
	q = snd_pci_quirk_lookup(chip->pci, msi_black_list);
	if (q) {
		dev_info(chip->dev,
			 "msi for device %04x:%04x set to %d\n",
		       q->subvendor, q->subdevice, q->value);
		chip->msi = q->value;
		return;
	}

	/* NVidia chipsets seem to cause troubles with MSI */
	if (chip->driver_caps & AZX_DCAPS_NO_MSI) {
		dev_info(chip->dev, "Disabling MSI\n");
		chip->msi = 0;
	}
}

/* check the snoop mode availability */
static void azx_check_snoop_available(struct azx *chip)
{
	bool snoop = chip->snoop;

	if (snoop != chip->snoop) {
		dev_info(chip->dev, "Force to %s mode\n",
			 snoop ? "snoop" : "non-snoop");
		chip->snoop = snoop;
	}
}

static void azx_probe_work(struct work_struct *work)
{
	azx_probe_continue(container_of(work, struct azx, probe_work));
}

/*load module */
static int azx_load_generic_mach(struct device *dev)
{
	struct snd_soc_hda_device *pdev;
	int ret;

	pdev = snd_soc_hda_device_alloc("mach_hda_generic", 0xFF, -1);
	if (!pdev) {
		dev_dbg(dev, "failed to allocate hda device\n");
		return -1;
	}

	ret = snd_soc_hda_device_add(pdev);
	if (ret) {
		dev_err(dev, "failed to add generic machine device\n");
		snd_soc_hda_device_put(pdev);
		return -1;
	}

	request_module("snd-mach_hda_generic");

	return 0;
}

static int azx_add_codec_device(int addr, unsigned int id, struct device *dev)
{
	struct snd_soc_hda_device *pdev;
	char name[10];
	int ret;

	snprintf(name, sizeof(name), "codec#%03x", addr);

	pdev = snd_soc_hda_device_alloc(name, addr, id);
	if (!pdev) {
		dev_err(dev, "failed to allocate hda device\n");
		return -1;
	}

	ret = snd_soc_hda_device_add(pdev);
	if (ret) {
		dev_err(dev, "failed to add hda codec device\n");
		snd_soc_hda_device_put(pdev);
		return -1;
	}
	return 0;
}

/*
 * Probe the given codec address
 */
static int probe_codec(struct azx *chip, int addr)
{
	unsigned int cmd = (addr << 28) | (AC_NODE_ROOT << 20) |
		(AC_VERB_PARAMETERS << 8) | AC_PAR_VENDOR_ID;
	unsigned int res;
	int ret;
	char name[32];

	mutex_lock(&chip->bus->cmd_mutex);
	chip->probing = 1;
	azx_send_cmd(chip->bus, cmd);
	res = azx_get_response(chip->bus, addr);
	chip->probing = 0;
	mutex_unlock(&chip->bus->cmd_mutex);
	if (res == -1)
		return -EIO;
	dev_dbg(chip->dev, "%s: codec #%d probed OK\n", pci_name(chip->pci), addr);

	/*load codec */
	snprintf(name, sizeof(name), "snd-hda-codec-id:%08x", res);

	request_module(name);

	ret = azx_add_codec_device(addr, res, chip->dev);

	ret = azx_map_codec_to_link(chip, addr);

	return ret;
}

static int azx_bus_create(struct azx *chip)
{
	struct snd_soc_hda_bus_ops bus_ops;
	struct snd_soc_azx *schip =
			container_of(chip, struct snd_soc_azx, hda_azx);
	int err;

	bus_ops.command = azx_send_cmd;
	bus_ops.get_response = azx_get_response;
	/*FIXME bus_ops.bus_reset = azx_bus_reset;*/

	err = snd_soc_hda_bus_init(chip->pci, schip, bus_ops, &schip->sbus);
	if (err < 0)
		return err;

	chip->bus = &schip->sbus->bus;
	if (chip->driver_caps & AZX_DCAPS_RIRB_DELAY) {
		dev_info(chip->dev, "%s: Enable delay in RIRB handling\n", pci_name(chip->pci));
		chip->bus->needs_damn_long_delay = 1;
	}

	return 0;
}

/* Codec initialization */
int azx_codec_create(struct azx *chip, const char *model)
{
	int c, max_slots;

	if (chip->driver_caps & AZX_DCAPS_RIRB_DELAY) {
		dev_dbg(chip->dev, "Enable delay in RIRB handling\n");
		chip->bus->needs_damn_long_delay = 1;
	}

	max_slots = AZX_DEFAULT_CODECS;

	/* First try to probe all given codec slots */
	for (c = 0; c < max_slots; c++) {
		if ((chip->codec_mask & (1 << c)) & chip->codec_probe_mask) {
			if (probe_codec(chip, c) < 0 &&
				!chip->ppcap_offset) {
				/* Some BIOSen give you wrong codec addresses
				 * that don't exist
				 */
				dev_warn(chip->dev,
					 "Codec #%d probe error; disabling it...\n", c);
				chip->codec_mask &= ~(1 << c);
				/* More badly, accessing to a non-existing
				 * codec often screws up the controller chip,
				 * and disturbs the further communications.
				 * Thus if an error occurs during probing,
				 * better to reset the controller chip to
				 * get back to the sanity state.
				 */
				azx_stop_chip(chip);
				azx_init_chip(chip, true);
			}
		}
	}

	/* AMD chipsets often cause the communication stalls upon certain
	 * sequence like the pin-detection.  It seems that forcing the synced
	 * access works around the stall.  Grrr...
	 */
	if (chip->driver_caps & AZX_DCAPS_SYNC_WRITE) {
		dev_dbg(chip->dev, "Enable sync_write for stable communication\n");
		chip->bus->sync_write = 1;
		chip->bus->allow_bus_reset = 1;
	}

	return 0;
}
/*
 * constructor
 */
static int azx_create(struct pci_dev *pci,
		      unsigned int driver_caps,
		      const struct hda_controller_ops *hda_ops,
		      struct snd_soc_azx **rchip)
{
	struct snd_soc_azx *schip;
	struct azx *chip;

	int err;

	*rchip = NULL;

	err = pci_enable_device(pci);
	if (err < 0)
		return err;

	schip = kzalloc(sizeof(*schip), GFP_KERNEL);
	if (!schip) {
		dev_err(&pci->dev, "Cannot allocate chip\n");
		pci_disable_device(pci);
		return -ENOMEM;
	}
	chip = &schip->hda_azx;

	spin_lock_init(&chip->reg_lock);
	mutex_init(&chip->open_mutex);
	chip->pci = pci;
	chip->dev = &pci->dev;
	chip->ops = hda_ops;
	chip->irq = -1;
	chip->driver_caps = driver_caps;
	chip->driver_type = driver_caps & 0xff;
	check_msi(chip);
	chip->dev_index = 0;
	INIT_WORK(&chip->irq_pending_work, azx_irq_pending_work);

	/*FIXME  - need to check why this is required
	init_vga_switcheroo(chip);
	*/
	init_completion(&chip->probe_wait);

	chip->position_fix[0] = chip->position_fix[1] =
		check_position_fix(chip, position_fix);
	/* combo mode uses LPIB for playback */
	if (chip->position_fix[0] == POS_FIX_COMBO) {
		chip->position_fix[0] = POS_FIX_LPIB;
		chip->position_fix[1] = POS_FIX_AUTO;
	}

	check_probe_mask(chip);

	chip->single_cmd = single_cmd;
	chip->snoop = hda_snoop;
	azx_check_snoop_available(chip);

	if (bdl_pos_adj < 0) {
		switch (chip->driver_type) {
		case AZX_DRIVER_ICH:
		case AZX_DRIVER_PCH:
			bdl_pos_adj = 0;
			break;
		default:
			bdl_pos_adj = 32;
			break;
		}
	}
	chip->bdl_pos_adj = &bdl_pos_adj;

	/* continue probing in work context as may trigger request module */
	INIT_WORK(&chip->probe_work, azx_probe_work);
	*rchip = schip;
	return 0;
}

static int azx_first_init(struct azx *chip)
{
	struct pci_dev *pci = chip->pci;
	int err;
	unsigned short gcap;

	err = pci_request_regions(pci, "ICH HD audio");
	if (err < 0)
		return err;
	chip->region_requested = 1;

	chip->addr = pci_resource_start(pci, 0);
	chip->remap_addr = pci_ioremap_bar(pci, 0);
	if (chip->remap_addr == NULL) {
		dev_err(chip->dev, "ioremap error\n");
		return -ENXIO;
	}

	azx_parse_capabilities(chip);
	if (chip->msi)
		if (pci_enable_msi(pci) < 0)
			chip->msi = 0;

	if (azx_acquire_irq(chip, 0) < 0)
		return -EBUSY;

	pci_set_master(pci);
	synchronize_irq(chip->irq);

	gcap = azx_readw(chip, GCAP);
	dev_dbg(chip->dev, "chipset global capabilities = 0x%x\n", gcap);

	/* disable 64bit DMA address on some devices */
	if (chip->driver_caps & AZX_DCAPS_NO_64BIT) {
		dev_dbg(chip->dev, "Disabling 64bit DMA\n");
		gcap &= ~ICH6_GCAP_64OK;
	}

	/* disable buffer size rounding to 128-byte multiples if supported */
	if (align_buffer_size >= 0)
		chip->align_buffer_size = !!align_buffer_size;
	else {
		if (chip->driver_caps & AZX_DCAPS_BUFSIZE)
			chip->align_buffer_size = 0;
		else if (chip->driver_caps & AZX_DCAPS_ALIGN_BUFSIZE)
			chip->align_buffer_size = 1;
		else
			chip->align_buffer_size = 1;
	}

	/* allow 64bit DMA address if supported by H/W */
	if ((gcap & ICH6_GCAP_64OK) && !pci_set_dma_mask(pci, DMA_BIT_MASK(64)))
		pci_set_consistent_dma_mask(pci, DMA_BIT_MASK(64));
	else {
		pci_set_dma_mask(pci, DMA_BIT_MASK(32));
		pci_set_consistent_dma_mask(pci, DMA_BIT_MASK(32));
	}

	/* read number of streams from GCAP register instead of using
	 * hardcoded value
	 */
	chip->capture_streams = (gcap >> 8) & 0x0f;
	chip->playback_streams = (gcap >> 12) & 0x0f;
	if (!chip->playback_streams && !chip->capture_streams) {
		/* gcap didn't give any info, switching to old method */

		chip->playback_streams = ICH6_NUM_PLAYBACK;
		chip->capture_streams = ICH6_NUM_CAPTURE;
	}
	chip->capture_index_offset = 0;
	chip->playback_index_offset = chip->capture_streams;
	chip->num_streams = chip->playback_streams + chip->capture_streams;
	chip->azx_dev = kcalloc(chip->num_streams, sizeof(*chip->azx_dev),
				GFP_KERNEL);
	if (!chip->azx_dev) {
		dev_err(chip->dev, "cannot malloc azx_dev\n");
		return -ENOMEM;
	}

	err = azx_alloc_stream_pages(chip);
	if (err < 0)
		return err;

	if (chip->ppcap_offset) {
		chip->link_dev = kcalloc(chip->num_streams,
					sizeof(*chip->link_dev),
					GFP_KERNEL);
		if (!chip->link_dev) {
			dev_err(chip->dev, "cannot malloc azx_dev\n");
			return -ENOMEM;
		}
	}

	/* initialize streams */
	azx_init_stream(chip);

	if (chip->ppcap_offset)
		azx_set_dma_decouple_mode(chip);

	/* initialize chip */
	azx_init_pci(chip);
	azx_init_chip(chip, (probe_only & 2) == 0);

	/* codec detection */
	if (!chip->codec_mask && !chip->ppcap_offset) {
		dev_err(chip->dev, "no codecs found!\n");
		return -ENODEV;
	}

	return 0;
}

/*
 * HDA controller ops.
 */

/* PCI register access. */
static void pci_azx_writel(u32 value, u32 __iomem *addr)
{
	writel(value, addr);
}

static u32 pci_azx_readl(u32 __iomem *addr)
{
	return readl(addr);
}

static void pci_azx_writew(u16 value, u16 __iomem *addr)
{
	writew(value, addr);
}

static u16 pci_azx_readw(u16 __iomem *addr)
{
	return readw(addr);
}

static void pci_azx_writeb(u8 value, u8 __iomem *addr)
{
	writeb(value, addr);
}

static u8 pci_azx_readb(u8 __iomem *addr)
{
	return readb(addr);
}

static int disable_msi_reset_irq(struct azx *chip)
{
	int err;

	free_irq(chip->irq, chip);
	chip->irq = -1;
	pci_disable_msi(chip->pci);
	chip->msi = 0;
	err = azx_acquire_irq(chip, 1);
	if (err < 0)
		return err;

	return 0;
}

/* DMA page allocation helpers.  */
static int dma_alloc_pages(struct azx *chip,
			   int type,
			   size_t size,
			   struct snd_dma_buffer *buf)
{
	int err;

	err = snd_dma_alloc_pages(type,
				  chip->dev,
				  size, buf);
	if (err < 0)
		return err;
	mark_pages_wc(chip, buf, true);
	return 0;
}

static void dma_free_pages(struct azx *chip, struct snd_dma_buffer *buf)
{
	mark_pages_wc(chip, buf, false);
	snd_dma_free_pages(buf);
}

static int substream_alloc_pages(struct azx *chip,
				 struct snd_pcm_substream *substream,
				 size_t size)
{
	struct azx_dev *azx_dev = get_azx_dev(substream);
	int ret;

	mark_runtime_wc(chip, azx_dev, substream, false);
	azx_dev->bufsize = 0;
	azx_dev->period_bytes = 0;
	azx_dev->format_val = 0;
	ret = snd_pcm_lib_malloc_pages(substream, size);
	if (ret < 0)
		return ret;
	mark_runtime_wc(chip, azx_dev, substream, true);
	return 0;
}

static int substream_free_pages(struct azx *chip,
				struct snd_pcm_substream *substream)
{
	struct azx_dev *azx_dev = get_azx_dev(substream);
	mark_runtime_wc(chip, azx_dev, substream, false);
	return snd_pcm_lib_free_pages(substream);
}

static const struct hda_controller_ops pci_hda_ops = {
	.reg_writel = pci_azx_writel,
	.reg_readl = pci_azx_readl,
	.reg_writew = pci_azx_writew,
	.reg_readw = pci_azx_readw,
	.reg_writeb = pci_azx_writeb,
	.reg_readb = pci_azx_readb,
	.disable_msi_reset_irq = disable_msi_reset_irq,
	.dma_alloc_pages = dma_alloc_pages,
	.dma_free_pages = dma_free_pages,
	.substream_alloc_pages = substream_alloc_pages,
	.substream_free_pages = substream_free_pages,
	.position_check = azx_position_check,
};

static int azx_probe(struct pci_dev *pci,
		     const struct pci_device_id *pci_id)
{
	struct snd_soc_azx *schip;
	struct azx *chip;
	bool schedule_probe;
	int err;

	err = azx_create(pci, pci_id->driver_data, &pci_hda_ops, &schip);
	if (err < 0)
		goto out_free;

	chip = &schip->hda_azx;

	/**FIXME
	err = register_vga_switcheroo(chip);
	if (err < 0) {
		snd_printk(KERN_ERR SFX
			   "%s: Error registering VGA-switcheroo client\n", pci_name(pci));
		goto out_free;
	}

	if (check_hdmi_disabled(pci)) {
		snd_printk(KERN_INFO SFX "%s: VGA controller is disabled\n",
			   pci_name(pci));
		snd_printk(KERN_INFO SFX "%s: Delaying initialization\n", pci_name(pci));
		chip->disabled = true;
	}
	*/
	schedule_probe = !chip->disabled;

	if (schedule_probe)
		schedule_work(&chip->probe_work);
	if (chip->disabled)
		complete_all(&chip->probe_wait);
	return 0;
out_free:
	return err;
}
static int azx_probe_continue(struct azx *chip)
{
	struct pci_dev *pci = chip->pci;
	int err;

	err = azx_first_init(chip);
	if (err < 0)
		goto out_free;

#ifdef CONFIG_SND_HDA_INPUT_BEEP
	chip->beep_mode = beep_mode;
#endif

	pci_set_drvdata(pci, chip);

	/*create new hda_bus */
	err = azx_bus_create(chip);
	if (err < 0)
		goto out_free;

	/* check if dsp is there */
	if (chip->ppcap_offset) {
		err = dsp_register(chip);
		if (err < 0) {
			dev_dbg(chip->dev, "error failed to register dsp\n");
			goto out_free;
		}
		/*FIXME machine name need to be read from DSDT entry*/
		err = azx_load_i2s_machine(chip);
		if (err < 0)
			goto out_free;
	}

	/*register platfrom dai and controls */
	err = soc_hda_platform_register(&pci->dev);
	if (err < 0)
		goto out_free;
	/* create codec instances */
	err = azx_codec_create(chip, model);
	if (err < 0)
		goto out_unregister;

	pci_set_drvdata(pci, chip);

	err = azx_load_generic_mach(&pci->dev);
	if (err < 0)
		goto out_unregister;

	chip->running = 1;
	azx_notifier_register(chip);

	if ((chip->driver_caps & AZX_DCAPS_PM_RUNTIME) || chip->use_vga_switcheroo)
		pm_runtime_put_noidle(&pci->dev);

	complete_all(&chip->probe_wait);
	return 0;

out_unregister:
	soc_hda_platform_unregister(&pci->dev);
out_free:
	chip->init_failed = 1;
	pci_set_drvdata(pci, NULL);
	return err;
}

static void azx_remove(struct pci_dev *pci)
{
	struct azx *chip  = pci_get_drvdata(pci);

	printk("***********azx_remove=***********\n");
	printk("****In %s", __func__);

	chip = pci_get_drvdata(pci);
	printk("chip->init_failed %d", chip->init_failed);

	if (chip->init_failed)
		return;
	if (pci_dev_run_wake(pci))
		pm_runtime_get_noresume(&pci->dev);

	soc_hda_platform_unregister(&pci->dev);
	dev_dbg(chip->dev, "In %s platform component removed", __func__);
	azx_i2s_machine_device_unregister(chip);
	dev_dbg(chip->dev, "In %s machine device removed", __func__);
	azx_dsp_unregister(chip);
	dev_dbg(chip->dev, "In %s dsp unregistered", __func__);
	snd_soc_hda_bus_release();
	dev_dbg(chip->dev, "In %s bus unregistered", __func__);
	azx_free(chip);
	dev_dbg(chip->dev, "In %s azx_free", __func__);
	pci_set_drvdata(pci, NULL);
}

/* PCI IDs */
static DEFINE_PCI_DEVICE_TABLE(azx_ids) = {
	/* CPT */
	{ PCI_DEVICE(0x8086, 0x1c20),
	  .driver_data = AZX_DRIVER_PCH | AZX_DCAPS_INTEL_PCH_NOPM },
	/* PBG */
	{ PCI_DEVICE(0x8086, 0x1d20),
	  .driver_data = AZX_DRIVER_PCH | AZX_DCAPS_INTEL_PCH_NOPM },
	/* Panther Point */
	{ PCI_DEVICE(0x8086, 0x1e20),
	  .driver_data = AZX_DRIVER_PCH | AZX_DCAPS_INTEL_PCH_NOPM },
	/* Lynx Point */
	{ PCI_DEVICE(0x8086, 0x8c20),
	  .driver_data = AZX_DRIVER_PCH | AZX_DCAPS_INTEL_PCH },
	/* Wellsburg */
	{ PCI_DEVICE(0x8086, 0x8d20),
	  .driver_data = AZX_DRIVER_PCH | AZX_DCAPS_INTEL_PCH },
	{ PCI_DEVICE(0x8086, 0x8d21),
	  .driver_data = AZX_DRIVER_PCH | AZX_DCAPS_INTEL_PCH },
	/* Lynx Point-LP */
	{ PCI_DEVICE(0x8086, 0x9c20),
	  .driver_data = AZX_DRIVER_PCH | AZX_DCAPS_INTEL_PCH },
	/* Lynx Point-LP */
	{ PCI_DEVICE(0x8086, 0x9c21),
	  .driver_data = AZX_DRIVER_PCH | AZX_DCAPS_INTEL_PCH },
	/* Haswell */
	{ PCI_DEVICE(0x8086, 0x0a0c),
	  .driver_data = AZX_DRIVER_SCH | AZX_DCAPS_INTEL_PCH },
	{ PCI_DEVICE(0x8086, 0x0c0c),
	  .driver_data = AZX_DRIVER_SCH | AZX_DCAPS_INTEL_PCH },
	{ PCI_DEVICE(0x8086, 0x0d0c),
	  .driver_data = AZX_DRIVER_SCH | AZX_DCAPS_INTEL_PCH },
	/* 5 Series/3400 */
	{ PCI_DEVICE(0x8086, 0x3b56),
	  .driver_data = AZX_DRIVER_SCH | AZX_DCAPS_INTEL_PCH_NOPM },
	/* Poulsbo */
	{ PCI_DEVICE(0x8086, 0x811b),
	  .driver_data = AZX_DRIVER_SCH | AZX_DCAPS_INTEL_PCH_NOPM },
	/* Oaktrail */
	{ PCI_DEVICE(0x8086, 0x080a),
	  .driver_data = AZX_DRIVER_SCH | AZX_DCAPS_INTEL_PCH_NOPM },
	/* ICH */
	{ PCI_DEVICE(0x8086, 0x2668),
	  .driver_data = AZX_DRIVER_ICH | AZX_DCAPS_OLD_SSYNC |
	  AZX_DCAPS_BUFSIZE },  /* ICH6 */
	{ PCI_DEVICE(0x8086, 0x27d8),
	  .driver_data = AZX_DRIVER_ICH | AZX_DCAPS_OLD_SSYNC |
	  AZX_DCAPS_BUFSIZE },  /* ICH7 */
	{ PCI_DEVICE(0x8086, 0x269a),
	  .driver_data = AZX_DRIVER_ICH | AZX_DCAPS_OLD_SSYNC |
	  AZX_DCAPS_BUFSIZE },  /* ESB2 */
	{ PCI_DEVICE(0x8086, 0x284b),
	  .driver_data = AZX_DRIVER_ICH | AZX_DCAPS_OLD_SSYNC |
	  AZX_DCAPS_BUFSIZE },  /* ICH8 */
	{ PCI_DEVICE(0x8086, 0x293e),
	  .driver_data = AZX_DRIVER_ICH | AZX_DCAPS_OLD_SSYNC |
	  AZX_DCAPS_BUFSIZE },  /* ICH9 */
	{ PCI_DEVICE(0x8086, 0x293f),
	  .driver_data = AZX_DRIVER_ICH | AZX_DCAPS_OLD_SSYNC |
	  AZX_DCAPS_BUFSIZE },  /* ICH9 */
	{ PCI_DEVICE(0x8086, 0x3a3e),
	  .driver_data = AZX_DRIVER_ICH | AZX_DCAPS_OLD_SSYNC |
	  AZX_DCAPS_BUFSIZE },  /* ICH10 */
	{ PCI_DEVICE(0x8086, 0x3a6e),
	  .driver_data = AZX_DRIVER_ICH | AZX_DCAPS_OLD_SSYNC |
	  AZX_DCAPS_BUFSIZE },  /* ICH10 */
	/* ValleyView FIXME
	{ PCI_DEVICE(0x8086, 0x0f04),
	  .driver_data = AZX_DRIVER_SCH | AZX_DCAPS_INTEL_VLV2 },*/
	/* Generic Intel */
	{ PCI_DEVICE(PCI_VENDOR_ID_INTEL, PCI_ANY_ID),
	  .class = PCI_CLASS_MULTIMEDIA_HD_AUDIO << 8,
	  .class_mask = 0xffffff,
	  .driver_data = AZX_DRIVER_ICH | AZX_DCAPS_BUFSIZE },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, azx_ids);

/* pci_driver definition */
static struct pci_driver azx_driver = {
	.name = KBUILD_MODNAME,
	.id_table = azx_ids,
	.probe = azx_probe,
	.remove = azx_remove,
	.driver = {
		.pm = AZX_PM_OPS,
	},
};

module_pci_driver(azx_driver);
