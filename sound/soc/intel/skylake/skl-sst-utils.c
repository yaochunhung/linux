/*
 *  skl-sst-utils.c - SKL sst utils functions
 *
 *  Copyright (C) 2016 Intel Corp
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as version 2, as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */

#include <linux/device.h>
#include <linux/slab.h>
#include <linux/uuid.h>
#include <linux/devcoredump.h>
#include <linux/pci.h>
#include "skl-sst-dsp.h"
#include "../common/sst-dsp.h"
#include "../common/sst-dsp-priv.h"
#include "skl-sst-ipc.h"
#include "skl-topology.h"


#define UUID_STR_SIZE 37
#define DEFAULT_HASH_SHA256_LEN 32
#define TYPE1_HEADER_ROW 7

/* FW Extended Manifest Header id = $AE1 */
#define SKL_EXT_MANIFEST_HEADER_MAGIC   0x31454124

struct skl_dfw_module_mod {
	char name[100];
	struct skl_dfw_module skl_dfw_mod;
};

struct UUID {
	u8 id[16];
};

union seg_flags {
	u32 ul;
	struct {
		u32 contents : 1;
		u32 alloc    : 1;
		u32 load     : 1;
		u32 read_only : 1;
		u32 code     : 1;
		u32 data     : 1;
		u32 _rsvd0   : 2;
		u32 type     : 4;
		u32 _rsvd1   : 4;
		u32 length   : 16;
	} r;
} __packed;

struct segment_desc {
	union seg_flags flags;
	u32 v_base_addr;
	u32 file_offset;
};

struct module_type {
	u32 load_type  : 4;
	u32 auto_start : 1;
	u32 domain_ll  : 1;
	u32 domain_dp  : 1;
	u32 rsvd       : 25;
} __packed;

struct adsp_module_entry {
	u32 struct_id;
	u8  name[8];
	struct UUID uuid;
	struct module_type type;
	u8  hash1[DEFAULT_HASH_SHA256_LEN];
	u32 entry_point;
	u16 cfg_offset;
	u16 cfg_count;
	u32 affinity_mask;
	u16 instance_max_count;
	u16 instance_bss_size;
	struct segment_desc segments[3];
} __packed;

struct adsp_fw_hdr {
	u32 id;
	u32 len;
	u8  name[8];
	u32 preload_page_count;
	u32 fw_image_flags;
	u32 feature_mask;
	u16 major;
	u16 minor;
	u16 hotfix;
	u16 build;
	u32 num_modules;
	u32 hw_buf_base;
	u32 hw_buf_length;
	u32 load_offset;
} __packed;

struct uuid_module {
	uuid_le uuid;
	int id;
	int is_loadable;

	struct list_head list;
	u8 hash1[DEFAULT_HASH_SHA256_LEN];
};

struct skl_ext_manifest_hdr {
	u32 id;
	u32 len;
	u16 version_major;
	u16 version_minor;
	u32 entries;
};

int snd_skl_get_module_info(struct skl_sst *ctx,
			struct skl_module_cfg *mconfig)
{
	struct uuid_module *module;
	uuid_le *uuid_mod;

	uuid_mod = (uuid_le *)mconfig->guid;

	if (list_empty(&ctx->uuid_list)) {
		dev_err(ctx->dev, "Module list is empty\n");
		return -EINVAL;
	}

	list_for_each_entry(module, &ctx->uuid_list, list) {
		if (uuid_le_cmp(*uuid_mod, module->uuid) == 0) {
			mconfig->id.module_id = module->id;
			mconfig->is_loadable = module->is_loadable;

			return 0;
		}
	}

	return -EINVAL;
}
EXPORT_SYMBOL_GPL(snd_skl_get_module_info);

void fw_exception_dump_read(struct skl_sst *ctx)
{
	int num_mod = 0, size, size_core_dump;
	struct uuid_module *module, *module1;
	void *coredump;
	void *fw_reg_addr, *offset;
	struct pci_dev *pci = to_pci_dev(ctx->dsp->dev);
	u16 length0, bus_dev_id, length1, length2;
	u32 crash_dump_ver;
	struct type0_crash_data *type0_data;
	struct type1_crash_data *type1_data;
	struct type2_crash_data *type2_data;
	struct adsp_type1_crash_data *type1_mod_info;

	if (list_empty(&ctx->uuid_list)) {
		dev_err(ctx->dev, "Module list is empty\n");
		return;
	}

	list_for_each_entry(module1, &ctx->uuid_list, list) {
		num_mod++;
	}

	length0 = (sizeof(struct fw_version)
		+ sizeof(struct sw_version) + sizeof(u32)*2)/sizeof(u32);
	length1 = num_mod * TYPE1_HEADER_ROW;
	length2 = MAX_FW_REG_SZ/sizeof(u32);

	size = (num_mod * sizeof(u32) * TYPE1_HEADER_ROW);
	size_core_dump = sizeof(struct type0_crash_data) +
				sizeof(struct type1_crash_data) +
				sizeof(struct type2_crash_data) + size;

	coredump = vzalloc(size_core_dump);
	if (!coredump) {
		dev_err(ctx->dsp->dev, "failed to allocate memory\n");
		return;
	}

	offset = coredump;
	bus_dev_id = pci->device;
	crash_dump_ver = 0x1;

	type0_data = (struct type0_crash_data *) offset;
	type0_data->type = TYPE0_EXCEPTION;
	type0_data->length = length0;
	type0_data->crash_dump_ver = crash_dump_ver;
	type0_data->bus_dev_id = bus_dev_id;
	offset += sizeof(struct type0_crash_data);

	type1_data = (struct type1_crash_data *) offset;
	type1_data->type = TYPE1_EXCEPTION;
	type1_data->length = length1;
	type1_mod_info = offset + sizeof(struct type1_crash_data);

	list_for_each_entry(module, &ctx->uuid_list, list) {
		memcpy(type1_mod_info->mod_uuid, &(module->uuid),
					(sizeof(type1_mod_info->mod_uuid)));
		memcpy(type1_mod_info->hash1, &(module->hash1),
					(sizeof(type1_mod_info->hash1)));
		memcpy(&type1_mod_info->mod_id, &(module->id),
					(sizeof(type1_mod_info->mod_id)));
		type1_mod_info ++;
	}

	offset += sizeof(struct type1_crash_data) + size;

	type2_data = (struct type2_crash_data *) offset;
	type2_data->type = TYPE2_EXCEPTION;
	type2_data->length = length2;
	fw_reg_addr = ctx->dsp->mailbox.in_base - ctx->dsp->addr.w0_stat_sz;
	memcpy_fromio(type2_data->fwreg, fw_reg_addr, MAX_FW_REG_SZ);

	dev_coredumpv(ctx->dsp->dev, coredump,
			(size_core_dump), GFP_KERNEL);

}
EXPORT_SYMBOL_GPL(fw_exception_dump_read);

/*
 * Parse the firmware binary to get the UUID, module id
 * and loadable flags
 */
int snd_skl_parse_uuids(struct sst_dsp *ctx, const struct firmware *fw,
			unsigned int offset, int index)
{
	struct adsp_fw_hdr *adsp_hdr;
	struct adsp_module_entry *mod_entry;
	int i, num_entry;
	uuid_le *uuid_bin;
	const char *buf;
	struct skl_sst *skl = ctx->thread_context;
	struct uuid_module *module;
	struct firmware stripped_fw;
	unsigned int safe_file;

	/* Get the FW pointer to derive ADSP header */
	stripped_fw.data = fw->data;
	stripped_fw.size = fw->size;

	skl_dsp_strip_extended_manifest(&stripped_fw);

	buf = stripped_fw.data;

	/* check if we have enough space in file to move to header */
	safe_file = sizeof(*adsp_hdr) + offset;
	if (stripped_fw.size <= safe_file) {
		dev_err(ctx->dev, "Small fw file size, No space for hdr\n");
		return -EINVAL;
	}

	adsp_hdr = (struct adsp_fw_hdr *)(buf + offset);

	/* check 1st module entry is in file */
	safe_file += adsp_hdr->len + sizeof(*mod_entry);
	if (stripped_fw.size <= safe_file) {
		dev_err(ctx->dev, "Small fw file size, No module entry\n");
		return -EINVAL;
	}

	mod_entry = (struct adsp_module_entry *)
		(buf + offset + adsp_hdr->len);

	num_entry = adsp_hdr->num_modules;

	/* check all entries are in file */
	safe_file += num_entry * sizeof(*mod_entry);
	if (stripped_fw.size <= safe_file) {
		dev_err(ctx->dev, "Small fw file size, No modules\n");
		return -EINVAL;
	}


	/*
	 * Read the UUID(GUID) from FW Manifest.
	 *
	 * The 16 byte UUID format is: XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXX
	 * Populate the UUID table to store module_id and loadable flags
	 * for the module.
	 */

	for (i = 0; i < num_entry; i++, mod_entry++) {
		module = kzalloc(sizeof(*module), GFP_KERNEL);
		if (!module)
			return -ENOMEM;

		uuid_bin = (uuid_le *)mod_entry->uuid.id;
		memcpy(&module->uuid, uuid_bin, sizeof(module->uuid));

		module->id = (i | (index << 12));
		module->is_loadable = mod_entry->type.load_type;
		memcpy(&module->hash1, mod_entry->hash1,
					sizeof(module->hash1));


		list_add_tail(&module->list, &skl->uuid_list);

		dev_dbg(ctx->dev,
			"Adding uuid :%pUL   mod id: %d  Loadable: %d\n",
			&module->uuid, module->id, module->is_loadable);
	}

	return 0;
}

void skl_freeup_uuid_list(struct skl_sst *ctx)
{
	struct uuid_module *uuid, *_uuid;

	list_for_each_entry_safe(uuid, _uuid, &ctx->uuid_list, list) {
		list_del(&uuid->list);
		kfree(uuid);
	}
}

/*
 * some firmware binary contains some extended manifest. This needs
 * to be stripped in that case before we load and use that image.
 *
 * Get the module id for the module by checking
 * the table for the UUID for the module
 */
int skl_dsp_strip_extended_manifest(struct firmware *fw)
{
	struct skl_ext_manifest_hdr *hdr;

	/* check if fw file is greater than header we are looking */
	if (fw->size < sizeof(hdr)) {
		pr_err("%s: Firmware file small, no hdr\n", __func__);
		return -EINVAL;
	}

	hdr = (struct skl_ext_manifest_hdr *)fw->data;

	if (hdr->id == SKL_EXT_MANIFEST_HEADER_MAGIC) {
		fw->size -= hdr->len;
		fw->data += hdr->len;
	}

	return 0;
}
