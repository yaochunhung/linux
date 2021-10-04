// SPDX-License-Identifier: (GPL-2.0-only OR BSD-3-Clause)
//
// Copyright(c) 2021 Mediatek Corporation. All rights reserved.
//
// Author: Bo Pan <bo.pan@mediatek.com>
//		YC Hung <yc.hung@mediatek.com>
//
// The Mediatek ADSP PCM implementation
//

#include "adsp_helper.h"

int adsp_genpool_create(struct adsp_mem_pool *mem_pool,
			struct adsp_mem *mem_info)
{
	int ret;

	mem_pool->dram_pool = gen_pool_create(SOF_MIN_ADSP_SHIFT, -1);
	if (!mem_pool->dram_pool)
		return -ENOMEM;

	if (!mem_info->va_addr || !mem_info->size) {
		ret = -EINVAL;
		goto out;
	}

	ret = gen_pool_add_virt(mem_pool->dram_pool, mem_info->va_addr,
				mem_info->phy_addr, mem_info->size, -1);
	if (ret)
		goto out;

	mem_pool->dram = mem_info;

	return 0;

out:
	gen_pool_destroy(mem_pool->dram_pool);
	mem_pool->dram_pool = NULL;
	return ret;
}
EXPORT_SYMBOL_NS(adsp_genpool_create, SND_SOC_SOF_MTK_COMMON);

int adsp_genpool_destroy(struct adsp_mem_pool *mem_pool)
{
	gen_pool_destroy(mem_pool->dram_pool);
	mem_pool->dram_pool = NULL;
	mem_pool->dram = NULL;
	return 0;
}
EXPORT_SYMBOL_NS(adsp_genpool_destroy, SND_SOC_SOF_MTK_COMMON);

int adsp_genpool_alloc(struct adsp_mem_pool *mem_pool, unsigned char **vaddr,
		       dma_addr_t *paddr, unsigned int size)
{
	if (!mem_pool || !mem_pool->dram_pool)
		return -EINVAL;

	*vaddr = (unsigned char *)gen_pool_dma_alloc(mem_pool->dram_pool,
						     size, paddr);
	if (!*vaddr)
		return -ENOMEM;

	return 0;
}

int adsp_genpool_alloc_align(struct adsp_mem_pool *mem_pool,
			     unsigned char **vaddr, dma_addr_t *paddr,
			     unsigned int size, int align)
{
	if (!mem_pool || !mem_pool->dram_pool)
		return -EINVAL;

	*vaddr = (unsigned char *)gen_pool_dma_alloc_align(mem_pool->dram_pool,
							   size, paddr, align);
	if (!*vaddr)
		return -ENOMEM;

	return 0;
}

int adsp_genpool_free(struct adsp_mem_pool *mem_pool,
		      unsigned char *vaddr, size_t size)
{
	if (!mem_pool || !mem_pool->dram_pool)
		return -EINVAL;

	if (*vaddr)
		gen_pool_free(mem_pool->dram_pool, (unsigned long)vaddr, size);

	return 0;
}

int mtk_adsp_pcm_hw_params(struct snd_sof_dev *sdev,
			   struct snd_pcm_substream *substream,
			   struct snd_pcm_hw_params *params,
			   struct sof_ipc_stream_params *ipc_params)
{
	struct snd_pcm_runtime *runtime = substream->runtime;
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_soc_component *scomp = sdev->component;
	struct device *dev = sdev->dev;
	struct snd_sof_pcm *spcm;
	struct snd_dma_buffer *dmab;
	struct snd_dma_buffer *pg_table;
	struct snd_dma_buffer *pg_table_backup;
	struct adsp_priv *priv = sdev->pdata->hw_pdata;
	struct adsp_mem_pool *mem_pool = &priv->mem_pool;
	size_t dma_bytes;
	int ret;

	dma_bytes = runtime->dma_bytes;

	spcm = snd_sof_find_spcm_dai(scomp, rtd);
	if (!spcm) {
		dev_err(sdev->dev, "%s : can't find PCM with DAI ID %d\n",
			__func__, rtd->dai_link->id);
		return -EINVAL;
	}

	pg_table = &spcm->stream[substream->stream].page_table;
	if (runtime->buffer_changed && substream->managed_buffer_alloc) {
		/* free dma pages allocated by common layer
		 * if page table private data is not NULL,
		 * this hw_params may be trigged by PCM xrun
		 */
		if (pg_table->private_data)
			return 0;

		snd_pcm_lib_free_pages(substream);

		/* prepare dma buf struct for dsp runtime dma and page table*/
		dmab = kzalloc(sizeof(*dmab) * 2, GFP_KERNEL);
		if (!dmab)
			return -ENOMEM;
		pg_table_backup = dmab + 1;
		/* backup original page table address */
		pg_table_backup->area = pg_table->area;
		pg_table_backup->addr = pg_table->addr;
		pg_table->private_data = pg_table_backup;
		ret = adsp_genpool_alloc(mem_pool, &pg_table->area,
					 &pg_table->addr, pg_table->bytes);
		if (ret)
			goto out;

		pg_table->addr = priv->ap2adsp_addr(pg_table->addr, priv->adsp);
		dmab->dev.type = SNDRV_DMA_TYPE_DEV;
		dmab->dev.dev = substream->pcm->card->dev;
		dmab->bytes = dma_bytes;
		/* allocate dma pages from dsp share buffer */
		ret = adsp_genpool_alloc_align(mem_pool, &dmab->area, &dmab->addr,
					       dma_bytes, PAGE_SIZE);
		if (ret)
			goto out;

		dmab->addr = priv->ap2adsp_addr(dmab->addr, priv->adsp);
		snd_pcm_set_runtime_buffer(substream, dmab);
		snd_sof_create_page_table(dev, dmab,
					  pg_table->area, dma_bytes);

		/* replace ipc parameters for dma buffer */
		ipc_params->buffer.phy_addr = pg_table->addr;
	}

	return 0;
out:
	if (dmab->area)
		adsp_genpool_free(mem_pool, dmab->area, dmab->bytes);

	if (pg_table_backup->area && pg_table->area) {
		adsp_genpool_free(mem_pool, pg_table->area,
				  pg_table->bytes);
		pg_table->area = pg_table_backup->area;
		pg_table->addr = pg_table_backup->addr;
		pg_table->private_data = NULL;
	}

	kfree(dmab);

	return ret;
}
EXPORT_SYMBOL_NS(mtk_adsp_pcm_hw_params, SND_SOC_SOF_MTK_COMMON);

int mtk_adsp_pcm_hw_free(struct snd_sof_dev *sdev,
			 struct snd_pcm_substream *substream)
{
	struct snd_soc_pcm_runtime *rtd = asoc_substream_to_rtd(substream);
	struct snd_soc_component *scomp = sdev->component;
	struct snd_dma_buffer *dmab;
	struct snd_dma_buffer *pg_table;
	struct snd_dma_buffer *pg_table_backup;
	struct adsp_priv *priv = sdev->pdata->hw_pdata;
	struct adsp_mem_pool *mem_pool = &priv->mem_pool;
	struct snd_sof_pcm *spcm;

	spcm = snd_sof_find_spcm_dai(scomp, rtd);
	if (!spcm) {
		dev_err(sdev->dev, "%s : can't find PCM with DAI ID %d\n",
			__func__, rtd->dai_link->id);
		return -EINVAL;
	}

	pg_table = &spcm->stream[substream->stream].page_table;
	if (substream->managed_buffer_alloc) {
		pg_table_backup = pg_table->private_data;
		dmab = snd_pcm_get_dma_buf(substream);
		if (dmab && dmab->area)
			adsp_genpool_free(mem_pool, dmab->area, dmab->bytes);
		snd_pcm_set_runtime_buffer(substream, NULL);
		if (pg_table_backup && pg_table->area) {
			adsp_genpool_free(mem_pool, pg_table->area,
					  pg_table->bytes);
			pg_table->area = pg_table_backup->area;
			pg_table->addr = pg_table_backup->addr;
			pg_table->private_data = NULL;
		}

		kfree(dmab);
	}
	return 0;
}
EXPORT_SYMBOL_NS(mtk_adsp_pcm_hw_free, SND_SOC_SOF_MTK_COMMON);

MODULE_LICENSE("Dual BSD/GPL");

