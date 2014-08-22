/*
 * Intel SST Firmware Loader
 *
 * Copyright (C) 2013, Intel Corporation. All rights reserved.
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

#ifndef SST_FIRMWARE_SKYLAKE_H_
#define SST_FIRMWARE_SKYLAKE_H_

#include <linux/types.h>
#include <linux/bug.h>

#define RSA_KEY_DEFAULT_LENGTH 256
#define RSA_EXPONENT_DEFAULT_LENGTH 4

#define SHA256_HASH_DEFAULT_LENGTH 32

#define MODULE_SIGNATURE_DEFAULT_SIZE 256
#define MODULE_NAME_DEFAULT_LENGTH 8
#define MODULE_UUID_DEFAULT_LENGTH 4

#define CSS_HEADER_LENGTH 8192

struct sst_segment_flags {
	uint32_t contents:1;
	uint32_t alloc:1;
	uint32_t load:1;
	uint32_t readonly:1;
	uint32_t code:1;
	uint32_t data:1;
	uint32_t rsvd0:2;
	uint32_t type:4;
	uint32_t rsvd1:4;
	uint32_t length:16;
} __packed;

enum sst_segment_type {
	ST_TEXT,
	ST_RODATA,
	ST_BSS
};

struct sst_segment_desc {
	struct sst_segment_flags flags;
	uint32_t v_base_addr;
	uint32_t file_offset;
} __packed;

struct sst_module_cfg {
	uint32_t par[4];
	uint32_t ip_pages;
	uint32_t cps;
	uint32_t ibs;
	uint32_t obs;
	uint32_t module_flags;
	uint32_t cpc;
	uint32_t obls;
} __packed;

enum sst_module_type {
	MT_BUILTIN,
	MT_LOADABLE
};

struct sst_module_entry {
	uint32_t struct_id;
	uint8_t name[MODULE_NAME_DEFAULT_LENGTH];
	uint32_t uuid[MODULE_UUID_DEFAULT_LENGTH];
	enum sst_module_type type;
	uint8_t hash[SHA256_HASH_DEFAULT_LENGTH];
	uint32_t entry_point;
	uint16_t cfg_offset;
	uint16_t cfg_count;
	uint32_t affinity_mask;
	uint16_t instance_max_count;
	uint16_t instance_bss_size;
	struct sst_segment_desc segments[3];
} __packed;

struct sst_adsp_fw_binary_header {
	uint32_t header_id;
	uint32_t header_len;
	uint8_t name[MODULE_NAME_DEFAULT_LENGTH];
	uint32_t preload_page_count;
	uint32_t fw_image_size;
	uint32_t feature_mask;
	uint16_t major_version;
	uint16_t minor_version;
	uint16_t hotfix_version;
	uint16_t build_version;
	uint32_t num_module_entries;
	uint32_t hw_buf_base_addr;
	uint32_t hw_buf_length;
	uint32_t load_offset;
} __packed;

struct sst_adsp_fw_binary_desc {
	struct sst_adsp_fw_binary_header header;
	struct sst_module_entry module_entries[1];
	/* address of module_cfg depend on
	 *  sizeof(struct sst_module_entry) * header.num_module_entries
	 */
	struct sst_module_cfg module_cfg[1];
} __packed;

struct sst_css_module_id {
	uint32_t _res_ls_bits:31;
	uint32_t debug_manifest:1;
} __packed;

struct sst_css_header {
	uint32_t module_type;
	uint32_t header_len;
	uint32_t header_version;
	struct sst_css_module_id module_id;
	uint32_t module_vendor;
	uint32_t date;
	uint32_t size;
	uint32_t key_size;
	uint32_t modules_size;
	uint32_t exponent_size;
	uint32_t reserved[22];
} __packed;

struct sst_manifest_rsa_keys {
	uint8_t modules[RSA_KEY_DEFAULT_LENGTH];
	uint8_t exponent[RSA_EXPONENT_DEFAULT_LENGTH];
} __packed;

struct sst_manifest_crypto_block {
	struct sst_manifest_rsa_keys pub_key;
	uint8_t signature[MODULE_SIGNATURE_DEFAULT_SIZE];
} __packed;

struct sst_fw_image_manifest {
	struct sst_css_header header;
	struct sst_manifest_crypto_block crypto_block;
	uint8_t css_header[CSS_HEADER_LENGTH];
	struct sst_adsp_fw_binary_desc adsp_fw_bin_desc;
} __packed;


static inline void sst_fw_structs_check_sizes(void)
{
	BUILD_BUG_ON(sizeof(struct sst_segment_flags) != 4);
	BUILD_BUG_ON(sizeof(struct sst_segment_desc) != 12);
	BUILD_BUG_ON(sizeof(struct sst_module_cfg) != 44);
	BUILD_BUG_ON(sizeof(struct sst_module_entry) != 116);
	BUILD_BUG_ON(sizeof(struct sst_adsp_fw_binary_header) != 48);
	BUILD_BUG_ON(sizeof(struct sst_adsp_fw_binary_desc) != 208);
	BUILD_BUG_ON(sizeof(struct sst_css_module_id) != 4);
	BUILD_BUG_ON(sizeof(struct sst_css_header) != 128);
	BUILD_BUG_ON(sizeof(struct sst_manifest_rsa_keys) != 260);
	BUILD_BUG_ON(sizeof(struct sst_manifest_crypto_block) != 516);
}

#endif /* HDA_SST_FIRMWARE_H_ */
