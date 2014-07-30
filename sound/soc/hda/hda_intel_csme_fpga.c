/*
 * Intel HDA CSME FPGA driver
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

#include <linux/module.h>
#include <linux/pci.h>
#include <linux/debugfs.h>
#include <linux/kthread.h>
#include <linux/delay.h>
#include <asm/uaccess.h>

#define csme_FPGA_ROM_BAR_IDX 0

#define CSME_REG_8 (8*4)
#define CSME_REG_9 (9*4)
#define CSME_REG_10 (10*4)
#define CSME_REG_11 (11*4)


#define CSME_IPC_VALIDATE_REQUEST 0x00002003
#define CSME_IPC_IMR_REUQEST 0x00000002


struct task_struct *thread;

struct csme_dev {
	void __iomem *mmio_base;
	struct pci_dev *pdev;
};

static u32 csme_readl(
	struct csme_dev *csmedev,
	u32 offset)
{
	u32 val;
	val = readl((csmedev)->mmio_base + offset);

	return val;
}

static void csme_writel(
	struct csme_dev *csmedev,
	u32 offset,
	u32 val)
{
	writel(val, (csmedev)->mmio_base + offset);
}

static const struct file_operations csme_fpga_rom_fops = {
	.owner = THIS_MODULE,
};


/* send response */
static void csme_imr_resp(struct csme_dev *csmedev)
{
	printk(KERN_INFO "%s: entry\n", __func__);
	
	csme_writel(csmedev, CSME_REG_11, 0x00400082);
	csme_writel(csmedev, CSME_REG_10, 0x80000001);

	printk(KERN_INFO "%s: exit\n", __func__);
}

/* send validate image response */
static void csme_validate_resp(struct csme_dev *csmedev)
{
	printk(KERN_INFO "%s: entry\n", __func__);
	csme_writel(csmedev, CSME_REG_10, 0x80000001);
	csme_writel(csmedev, CSME_REG_11, 0x00000083);

	printk(KERN_INFO "%s: exit\n", __func__);
}

/* clear cse */
static void csme_clear_cse_done(struct csme_dev *csmedev)
{
	u32 reg, i;

	printk(KERN_INFO "%s: entry\n", __func__);
	
	for (i = 100000; i > 0; i--) {
		reg = csme_readl(csmedev, CSME_REG_11);
		if((reg & 0x40000000) == 0x40000000) {
			printk(KERN_INFO "%s: clearing cse done\n", __func__);
			csme_writel(csmedev, CSME_REG_11, 0x40000000);
			break;
		}
		mdelay(1);
	}
	
	if(!i)
		printk(KERN_ERR "%s: Timeout\n", __func__);

	printk(KERN_INFO "%s: exit\n", __func__);
}

/* wait for response thread */
static int csme_ipc_thread(void *dev)
{
	u32 csme_reg_8;
	u32 csme_reg_9;
	u32 reg;

	/* get struct */
	struct csme_dev *csmedev = (struct csme_dev*) dev;

	printk(KERN_INFO "%s: entry\n", __func__);

	/* polling */
	while(true) {
		csme_reg_8 = csme_readl(csmedev, CSME_REG_8);
		if((csme_reg_8 & 0x80000000) == 0x80000000) {
			if(csme_reg_8 == 0x90000001) {
				csme_reg_9 = csme_readl(csmedev, CSME_REG_9);
				csme_writel(csmedev, CSME_REG_8, 0x80000000);
				switch(csme_reg_9)
				{
					case CSME_IPC_IMR_REUQEST:
						csme_imr_resp(csmedev);
						csme_clear_cse_done(csmedev);
					break;
					case CSME_IPC_VALIDATE_REQUEST:
						csme_validate_resp(csmedev);
						csme_clear_cse_done(csmedev);
					break;
					default:
						printk("%s: Unrecognized message: reg:0x%x\n",
							__func__, csme_reg_9);
				}
			}
		}
		msleep(1);
	}

	printk(KERN_INFO "%s: exit\n", __func__);
	return 0;
}

/* cse_wait_for_imr_request */
static int csme_fpga_probe(struct pci_dev *pdev,
		const struct pci_device_id *pci_id)
{
	struct csme_dev *csmedev;
	int ret = 0;

	printk(KERN_INFO "%s: entry\n", __func__);

	/* device enable */
	ret = pci_enable_device(pdev);
	if (ret) {
		dev_err(&pdev->dev, "%s: pci_enable_device failed: %d\n",
			__func__, ret);
		return ret;
	}

	/* allocation */
	csmedev = kzalloc(sizeof(struct csme_dev), GFP_KERNEL);
	if (!csmedev) {
		dev_err(&pdev->dev, "%s: kzalloc failed, aborting.\n",
			__func__);
		ret = -ENOMEM;
		goto err_alloc;
	}

	/* mmio mapping */
	csmedev->mmio_base
		= pci_ioremap_bar(pdev, csme_FPGA_ROM_BAR_IDX);
	if (!csmedev->mmio_base) {
		dev_err(&pdev->dev, "%s: ioremap failed, aborting\n",
			__func__);
		ret = -ENXIO;
		goto err_map;
	}

	csmedev->pdev = pdev;
	pci_set_drvdata(pdev, csmedev);

	/* start thread */
	thread = kthread_create(csme_ipc_thread,
			(void*)csmedev,"thread");

	if (!thread) {
		printk(KERN_INFO "Failed to create dsp thread\n");
		ret = -ENOMEM;
		goto err_map;
	}

	wake_up_process(thread);

	printk(KERN_INFO "%s: exit\n", __func__);

	return ret;

err_map:
	if (csmedev->mmio_base) {
		iounmap(csmedev->mmio_base);
		csmedev->mmio_base = NULL;
	}
	kfree(csmedev);
err_alloc:
	pci_disable_device(pdev);
	return ret;
}

static void csme_fpga_remove(struct pci_dev *pdev)
{
	struct csme_dev *csmedev = pci_get_drvdata(pdev);

	printk(KERN_INFO "csme_fpga_remove_in");

	/* thread stop */
	kthread_stop(thread);

	/* unmap PCI memory space, mapped during device init. */
	if (csmedev->mmio_base) {
		iounmap(csmedev->mmio_base);
		csmedev->mmio_base = NULL;
	}
	pci_disable_device(pdev);
	kfree(csmedev);

	printk(KERN_INFO "csme_fpga_remove_out");
}

/* PCI IDs */
static DEFINE_PCI_DEVICE_TABLE(csme_fpga_ids) = {
	{ PCI_DEVICE(0x8086, 0x0200) },
	{ 0, }
};
MODULE_DEVICE_TABLE(pci, csme_fpga_ids);

/* pci_driver definition */
static struct pci_driver csme_fpga_driver = {
	.name = KBUILD_MODNAME,
	.id_table = csme_fpga_ids,
	.probe = csme_fpga_probe,
	.remove = csme_fpga_remove,
};

module_pci_driver(csme_fpga_driver);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Intel HDA CSME FPGA driver");
