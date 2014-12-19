/*
 *  soc-hda-sst-debug.c - Intel SoC azx driver debugfs support
 *
 *  Copyright (C) 2014 Intel Corp
 *  Authors:	Hardik Shah <hardik.t.shah@intel.com>
 *
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
 *
 */

#include <linux/fs.h>
#include <linux/pci.h>
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/pm_runtime.h>
#include <linux/uaccess.h>
#include <sound/soc-hda-sst-dsp.h>
#include <sound/soc-hda-sst-ipc.h>


struct sst_dsp_debug_ops {
	const char *name;
	const struct file_operations *fops;
	umode_t mode;
};

static ssize_t sst_dsp_debug_ipc_enable_read(struct file *file, char __user *user_buf,
				   size_t count, loff_t *ppos)
{
	struct sst_dsp_ctx *dsp = file->private_data;
	struct sst_dsp_debug *debug = &dsp->sst_dsp_debug;

	char *status;

	status = debug->ipc_debug_enable ? "enabled\n" : "disabled\n";
	return simple_read_from_buffer(user_buf, count, ppos,
			status, strlen(status));
}

static ssize_t sst_dsp_debug_ipc_enable_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct sst_dsp_ctx *dsp = file->private_data;
	struct sst_dsp_debug *debug = &dsp->sst_dsp_debug;
	char buf[16];
	int sz = min(count, sizeof(buf)-1);

	if (copy_from_user(buf, user_buf, sz))
		return -EFAULT;
	buf[sz] = 0;
	if (!strncmp(buf, "enable\n", sz)) {
		debug->ipc_debug_enable = 1;
		sz = 6; /* strlen("enable") */
	} else if (!strncmp(buf, "disable\n", sz)) {
		debug->ipc_debug_enable = 0;
		sz = 7; /* strlen("disable") */
	} else
		return -EINVAL;
	return sz;
}

static ssize_t sst_dsp_debug_ipc_count_read(struct file *file, char __user *user_buf,
				   size_t count, loff_t *ppos)
{
	struct sst_dsp_ctx *dsp = file->private_data;
	struct sst_dsp_debug *debug = &dsp->sst_dsp_debug;
	char ipc_count[16];

	sprintf(ipc_count, "%d\n", debug->ipc_debug_count);
	return simple_read_from_buffer(user_buf, count, ppos,
			ipc_count, strlen(ipc_count));
}

static ssize_t sst_dsp_debug_ipc_count_write(struct file *file,
		const char __user *user_buf, size_t count, loff_t *ppos)
{
	struct sst_dsp_ctx *dsp = file->private_data;
	struct sst_dsp_debug *debug = &dsp->sst_dsp_debug;
	char buf[16];
	char *start = buf;
	int ret = 0, ipc_debug_count;

	size_t buf_size = min(count, sizeof(buf)-1);
	if (copy_from_user(buf, user_buf, buf_size))
		return -EFAULT;
	buf[buf_size] = 0;

	ret = kstrtou32(start, 10, &ipc_debug_count);
	if (ret) {
		pr_err("kstrtoul failed, ret_val = %d\n", ret);
		goto end;
	}
	debug->ipc_debug_count = ipc_debug_count;
	ret = buf_size;
end:
	return ret;
}


static const struct file_operations dsp_debug_ipc_enable_ops = {
	.open = simple_open,
	.read = sst_dsp_debug_ipc_enable_read,
	.write = sst_dsp_debug_ipc_enable_write,
	.llseek = default_llseek,
};

static const struct file_operations dsp_debug_ipc_count_ops = {
	.open = simple_open,
	.read = sst_dsp_debug_ipc_count_read,
	.write = sst_dsp_debug_ipc_count_write,
	.llseek = default_llseek,
};



static const struct sst_dsp_debug_ops dsp_common_dbg_entries[] = {
	{"enable_ipc_debug", &dsp_debug_ipc_enable_ops, 0600},
	{"ipc_debug_count", &dsp_debug_ipc_count_ops, 0600},
};

void dsp_debugfs_create_files(struct sst_dsp_ctx *dsp,
			const struct sst_dsp_debug_ops *entries, int size)
{
	int i;

	for (i = 0; i < size; i++) {
		struct dentry *dentry;
		const struct sst_dsp_debug_ops *entry = &entries[i];

		dentry = debugfs_create_file(entry->name, entry->mode,
				dsp->sst_dsp_debug.root, dsp, entry->fops);
		if (dentry == NULL) {
			pr_err("Failed to create %s file\n", entry->name);
			return;
		}
	}
}


int sst_dsp_debugfs_init(struct sst_dsp_ctx *dsp)
{
	struct sst_dsp_debug *debug = &dsp->sst_dsp_debug;

	debug->root = debugfs_create_dir("snd_soc_dsp", NULL);
	if (IS_ERR(debug->root) || !debug->root) {
		pr_err("Failed to create debugfs directory\n");
		return -EINVAL;
	}
	dsp_debugfs_create_files(dsp, dsp_common_dbg_entries,
				ARRAY_SIZE(dsp_common_dbg_entries));
	return 0;
}
int sst_dsp_debugfs_uninit(struct sst_dsp_ctx *dsp)
{
	debugfs_remove_recursive(dsp->sst_dsp_debug.root);
	return 0;
}
