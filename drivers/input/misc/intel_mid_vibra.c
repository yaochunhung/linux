/*
 *  intel_mid_vibra.c - Intel vibrator driver for Bxtn
 *
 *  Copyright (C) 2011-2015 Intel Corp
 *  Author: KP, Jeeja <jeeja.kp@intel.com>
 *  Author: Vinod Koul <vinod.koul@intel.com>
 *  Modified by: Mythri P K <mythri.p.k@intel.com>
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


#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pci.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/acpi.h>
#include <linux/pm_runtime.h>
#include "intel_mid_vibra.h"
#include <trace/events/power.h>
#include "mid_vibra.h"

union sst_pwmctrl_reg {
	struct {
		u32 pwmtd:8;
		u32 pwmbu:22;
		u32 pwmswupdate:1;
		u32 pwmenable:1;
	} part;
	u32 full;
};

static int vibra_soc_pwm_configure(struct vibra_info *info, bool enable)
{
	union sst_pwmctrl_reg pwmctrl;

	if (enable) {
		/* lnw_gpio_set_alt(info->gpio_pwm, info->alt_fn);
		 * Check if applicable
		 */

		/* 1. Enable the PWM by setting PWM enable bit to 1 */
		pwmctrl.full = readl(info->shim);
		dev_dbg(info->dev, "Vibra:Read pwmctrl %x\n", readl(info->shim));
		pwmctrl.part.pwmenable = 1;
		writel(pwmctrl.full, info->shim);

		/* 2. Read the PWM register to make sure there is no pending
		 * update.
		 */
		pwmctrl.full = readl(info->shim);
		dev_dbg(info->dev, "Read pwmctrl %x\n", pwmctrl.full);

		/* check pwnswupdate bit */
		if (pwmctrl.part.pwmswupdate)
			return -EBUSY;
		/* Base unit == 1*/
		pwmctrl.part.pwmswupdate = 0x1;

		/* validate values input */
		if (*info->base_unit > info->max_base_unit)
			*info->base_unit = info->max_base_unit;
		if (*info->duty_cycle > info->max_duty_cycle)
			*info->duty_cycle = info->max_duty_cycle;
		pwmctrl.part.pwmbu = *info->base_unit;
		pwmctrl.part.pwmtd = *info->duty_cycle;
		writel(pwmctrl.full,  info->shim);
		dev_dbg(info->dev, "Read pwmctrl %x\n", pwmctrl.full);
	} else { /*disable PWM block */
		/* lnw_gpio_set_alt(info->gpio_pwm, 0);
		 * Check if applicable
		 */

		/*1. setting PWM enable bit to 0 */
		pwmctrl.full = readl(info->shim);
		pwmctrl.part.pwmenable = 0;
		writel(pwmctrl.full,  info->shim);
	}
	return 0;
}
/* Enable vibra */
static void vibra_drv2603_enable(struct vibra_info *info)
{
	dev_dbg(info->dev, "%s: Enable", __func__);
	mutex_lock(&info->lock);
	pm_runtime_get_sync(info->dev);

	/* Enable the EN line */
	vibra_gpio_set_value(info, 1);

	/* Wait for 850us per spec, give 100us buffer */
	usleep_range(950, 1000);

	/* Enable the Trigger line */
	info->pwm_configure(info, true);

	info->enabled = true;
	mutex_unlock(&info->lock);
}
static void vibra_disable(struct vibra_info *info)
{
	dev_dbg(info->dev, "%s: Disable", __func__);
	mutex_lock(&info->lock);
	vibra_gpio_set_value_cansleep(info, 0);
	info->enabled = false;
	info->pwm_configure(info, false);
	pm_runtime_put(info->dev);
	mutex_unlock(&info->lock);
}


/*******************************************************************************
 * SYSFS                                                                       *
 ******************************************************************************/

static ssize_t vibra_show_vibrator(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct vibra_info *info = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", info->enabled);

}

static ssize_t vibra_set_vibrator(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t len)
{
	long vibrator_enable;
	struct vibra_info *info = dev_get_drvdata(dev);

	if (kstrtol(buf, 0, &vibrator_enable))
		return -EINVAL;
	if (vibrator_enable == info->enabled)
		return len;
	else if (vibrator_enable == 0)
		info->disable(info);
	else if (vibrator_enable == 1)
		info->enable(info);
	else
		return -EINVAL;
	return len;
}

unsigned long mid_vibra_base_unit;
unsigned long mid_vibra_duty_cycle;

static DEVICE_ATTR(vibrator, S_IRUGO | S_IWUSR,
		   vibra_show_vibrator, vibra_set_vibrator);
static DEVICE_ULONG_ATTR(pwm_baseunit, S_IRUGO | S_IWUSR, mid_vibra_base_unit);
static DEVICE_ULONG_ATTR(pwm_ontime_div, S_IRUGO | S_IWUSR, mid_vibra_duty_cycle);

static struct attribute *vibra_attrs[] = {
	&dev_attr_vibrator.attr,
	&dev_attr_pwm_baseunit.attr.attr,
	&dev_attr_pwm_ontime_div.attr.attr,
	0,
};

static const struct attribute_group vibra_attr_group = {
	.attrs = vibra_attrs,
};

/*** Module ***/
#if CONFIG_PM
static int intel_vibra_runtime_suspend(struct device *dev)
{
	struct vibra_info *info = dev_get_drvdata(dev);

	dev_dbg(dev, "In %s\n", __func__);
	info->pwm_configure(info, false);
	return 0;
}

static int intel_vibra_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "In %s\n", __func__);
	return 0;
}

static void intel_vibra_complete(struct device *dev)
{
	dev_dbg(dev, "In %s\n", __func__);
	intel_vibra_runtime_resume(dev);
}

static const struct dev_pm_ops intel_mid_vibra_pm_ops = {
	.prepare = intel_vibra_runtime_suspend,
	.complete = intel_vibra_complete,
	.runtime_suspend = intel_vibra_runtime_suspend,
	.runtime_resume = intel_vibra_runtime_resume,
};
#endif

struct vibra_info *mid_vibra_setup(struct device *dev, struct mid_vibra_data *data)
{
	struct vibra_info *info;

	dev_dbg(dev, "probe data divisor %x, base %x, alt_fn %d ext_drv %d, name: %s",
		data->time_divisor, data->base_unit, data->alt_fn, data->ext_drv, data->name);

	info =  devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(dev, "%s: no memory for driver context", __func__);
		return NULL;
	}

	info->alt_fn = data->alt_fn;
	info->ext_drv = data->ext_drv;
	info->gpio_en = data->gpio_en;
	info->gpio_pwm = data->gpio_pwm;
	info->name = data->name;
	info->use_gpio_en = data->use_gpio_en;

	info->dev = dev;
	mutex_init(&info->lock);
	info->vibra_attr_group = &vibra_attr_group;

	mid_vibra_base_unit = data->base_unit;
	mid_vibra_duty_cycle = data->time_divisor;
	info->base_unit = &mid_vibra_base_unit;
	info->duty_cycle = &mid_vibra_duty_cycle;

	info->enable = vibra_drv2603_enable;
	info->disable = vibra_disable;

	return info;
}

static int intel_mid_vibra_probe(struct pci_dev *pci,
			const struct pci_device_id *pci_id)
{
	struct vibra_info *info;
	struct device *dev = &pci->dev;
	struct mid_vibra_data *data;
	int ret;

	dev_dbg(&pci->dev, "Probe for DID %x\n", pci->device);

	data = pci->dev.platform_data;
	if (!data) {
		dev_err(&pci->dev, "Failed to get vibrator platform data\n");
		return -ENODEV;
	}

	info = mid_vibra_setup(dev, data);
	if (!info)
		return -ENODEV;

	info->pwm_configure = vibra_soc_pwm_configure;

	info->max_base_unit = INTEL_VIBRA_MAX_BASEUNIT;
	info->max_duty_cycle = INTEL_VIBRA_MAX_TIMEDIVISOR;

	if (info->use_gpio_en) {
		dev_dbg(&pci->dev, "using gpios en: %d, pwm %d",
				info->gpio_en, info->gpio_pwm);
		ret = gpio_request_one(info->gpio_en, GPIOF_DIR_OUT,
				"VIBRA ENABLE");
		if (ret != 0) {
			dev_err(dev, "gpio_request(%d) fails:%d\n",
					info->gpio_en, ret);
			goto out;
		}
	}
	/* Init the device */
	ret = pci_enable_device(pci);
	if (ret) {
		dev_err(dev, "device can't be enabled\n");
		goto do_freegpio_vibra_enable;
	}
	ret = pci_request_regions(pci, INTEL_VIBRA_DRV_NAME);
	if (ret)
		goto do_disable_device;
	pci_dev_get(pci);

	/* vibra Shim */
	info->shim =  pci_ioremap_bar(pci, 0);
	if (!info->shim) {
		dev_err(dev, "ioremap failed for vibra driver\n");
		goto do_release_regions;
	}
	dev_dbg(dev, "Base reg: %#x", (unsigned int) pci_resource_start(pci, 0));
	ret = sysfs_create_group(&dev->kobj, info->vibra_attr_group);
	if (ret) {
		dev_err(dev, "could not register sysfs files\n");
		goto do_unmap_shim;
	}

	pci_set_drvdata(pci, info);
	pm_runtime_allow(&pci->dev);
	pm_runtime_put_noidle(&pci->dev);
	return ret;

do_unmap_shim:
	iounmap(info->shim);
do_release_regions:
	pci_release_regions(pci);
do_disable_device:
	pci_disable_device(pci);
do_freegpio_vibra_enable:
	vibra_gpio_free(info);
out:
	return ret;
}

static void intel_mid_vibra_remove(struct pci_dev *pci)
{
	struct vibra_info *info = pci_get_drvdata(pci);
	vibra_gpio_free(info);
	sysfs_remove_group(&info->dev->kobj, info->vibra_attr_group);
	iounmap(info->shim);
	pci_release_regions(pci);
	pci_disable_device(pci);
	pci_set_drvdata(pci, NULL);
}

/* PCI Routines */
static DEFINE_PCI_DEVICE_TABLE(intel_vibra_ids) = {
	{ PCI_VDEVICE(INTEL, PCI_DEVICE_ID_INTEL_VIBRA), 0},
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, intel_vibra_ids);

static struct pci_driver vibra_driver = {
	.name = INTEL_VIBRA_DRV_NAME,
	.id_table = intel_vibra_ids,
	.probe = intel_mid_vibra_probe,
	.remove = intel_mid_vibra_remove,
#ifdef CONFIG_PM
	.driver = {
		.pm = &intel_mid_vibra_pm_ops,
	},
#endif
};

/**
* intel_mid_vibra_init - Module init function
*
* Registers with PCI
* Registers platform
* Init all data strutures
*/
static int __init intel_mid_vibra_init(void)
{
	int ret = 0;

	/* Register with PCI */
	ret = pci_register_driver(&vibra_driver);
	if (ret)
		pr_err("PCI register failed\n");

	return ret;
}

/**
* intel_mid_vibra_exit - Module exit function
*
* Unregisters with PCI
* Unregisters platform
* Frees all data strutures
*/
static void __exit intel_mid_vibra_exit(void)
{
	pci_unregister_driver(&vibra_driver);
	return;
}

late_initcall(intel_mid_vibra_init);
module_exit(intel_mid_vibra_exit);

MODULE_ALIAS("pci:intel_mid_vibra");
MODULE_DESCRIPTION("Intel(R) MID Vibra driver");
MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("KP Jeeja <jeeja.kp@intel.com>");
