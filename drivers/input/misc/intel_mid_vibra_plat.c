/*
 * platform_vibra.c: Vibra platform  data initilization file
 *
 * Copyright (C) 2014 Intel Corporation
 * Author: Mythri PK <mythri.p.k@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/gpio.h>
#include <linux/sfi.h>
#include <linux/platform_device.h>
#include <linux/input/intel_mid_vibra.h>

static struct mid_vibra_data bxtn_vibra_pdata = {
	.time_divisor = 0x40,
	.base_unit = 0x80,
	.alt_fn = 1,
	.ext_drv = 0,
	.name = "drv2603",
	.use_gpio_en = false,
};

static struct mid_vibra_data *get_vibra_platform_data(struct pci_dev *pdev)
{
	struct mid_vibra_data *pdata = NULL;
	pdata = &bxtn_vibra_pdata;
#if 0
	if (pdata != NULL) {
		pdata->gpio_en = get_gpio_by_name("haptics_en");
		pdata->gpio_pwm = get_gpio_by_name("haptics_pwm");
	}
#endif
	return pdata;
}

static void vibra_pci_early_quirks(struct pci_dev *pci_dev)
{
	pci_dev->dev.platform_data = get_vibra_platform_data(pci_dev);
}

DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_VIBRA,
					vibra_pci_early_quirks);
