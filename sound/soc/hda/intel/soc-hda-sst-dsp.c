/*
 *  soc_hda_sst-dsp.c - HDA DSP library generic function
 *
 *  Copyright (C) 2014 Intel Corp
 *  Author:Rafal Redzimski <rafal.f.redzimski@intel.com>
 *		Jeeja KP <jeeja.kp@intel.com>
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

#include <linux/io.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/spinlock_types.h>
#include <linux/mutex.h>
#include <sound/core.h>
#include <sound/pcm.h>
#include <sound/memalloc.h>
#include <sound/soc-hda-sst-dsp.h>
#include <sound/soc-hda-sst-ipc.h>

#define DSP_CORE_POWER_UP_TIMEOUT		50
#define DSP_CORE_POWER_DOWN_TIMEOUT		50
#define DSP_CORE_SET_RESET_STATE_TIMEOUT	50
#define DSP_CORE_UNSET_RESET_STATE_TIMEOUT	50

u8 sst_readb_traced(struct sst_dsp_ctx *dsp, u32 offset)
{
	u8 val;

	val = readb((dsp)->mmio_base + offset);
	dev_dbg(dsp->dev, "offset=%x val=%x\n", offset, val);

	return val;
}

u16 sst_readw_traced(struct sst_dsp_ctx *dsp, u32 offset)
{
	u16 val;

	val = readw((dsp)->mmio_base + offset);
	dev_dbg(dsp->dev, "offset=%x val=%x\n", offset, val);

	return val;
}

u32 sst_readl_traced(struct sst_dsp_ctx *dsp, u32 offset)
{
	u32 val;

	val = readl((dsp)->mmio_base + offset);
	dev_dbg(dsp->dev, "offset=%x val=%x\n", offset, val);

	return val;
}

void sst_writeb_traced(struct sst_dsp_ctx *dsp, u32 offset, u8 val)
{
	writeb(val, (dsp)->mmio_base + offset);
	dev_dbg(dsp->dev, "offset=%x val=%x\n", offset, val);
}

void sst_writew_traced(struct sst_dsp_ctx *dsp, u32 offset, u16 val)
{
	dev_dbg(dsp->dev, "offset=%x val=%x\n", offset, val);
	writew(val, (dsp)->mmio_base + offset);
}

void sst_writel_traced(struct sst_dsp_ctx *dsp, u32 offset, u32 val)
{
	dev_dbg(dsp->dev, "offset=%x val=%x\n", offset, val);
	writel(val, (dsp)->mmio_base + offset);
}

/* generic functions */
static irqreturn_t sst_interrupt(int irq, void *dev_id);
static int sst_acquire_irq(struct sst_dsp_ctx *dsp);

void sst_updatel_bits(
	struct sst_dsp_ctx *ctx,
	u32 offset, u32 mask, u32 value)
{
	u32 val;

	val = (readl((ctx)->mmio_base + offset) & ~mask) | (value & mask);

	writel(val, (ctx)->mmio_base + offset);

	dev_dbg(ctx->dev, "offset=%x val=%x\n", offset, val);
}

static void sst_updatel_bits_locked(
	struct sst_dsp_ctx  *ctx,
	u32 offset, u32 mask, u32 value)
{
	unsigned long flags;
	spin_lock_irqsave(&ctx->reg_lock, flags);
	sst_updatel_bits(ctx, offset, mask, value);
	spin_unlock_irqrestore(&ctx->reg_lock, flags);
}

static inline void _sst_memcpy_toio_32(volatile u32 __iomem *dest,
	u32 *src, size_t bytes)
{
	int i, words = bytes >> 2;

	for (i = 0; i < words; i++)
		writel(src[i], dest + i);
}

static inline void _sst_memcpy_fromio_32(u32 *dest,
	const volatile __iomem u32 *src, size_t bytes)
{
	int i, words = bytes >> 2;

	for (i = 0; i < words; i++)
		dest[i] = readl(src + i);
}

void sst_mailbox_write(struct sst_dsp_ctx  *ctx, void *msg, size_t bytes)
{
	_sst_memcpy_toio_32(ctx->window.w1, msg, bytes);
}

void sst_mailbox_read(struct sst_dsp_ctx  *ctx, void *msg, size_t bytes)
{
	_sst_memcpy_fromio_32(msg, ctx->window.w0up, bytes);
}

int sst_register_poll(struct sst_dsp_ctx  *ctx, u32 offset, u32 mask,
			 u32 expected_value, u32 timeout, char *operation)
{
	int time = 0;
	int ret = 0;
	u32 reg;


	/* check if set state successful */
	for (time = 0; time < timeout; time++) {
		if ((sst_readl_alt(ctx, offset) & mask) == expected_value)
			break;

		mdelay(1);
	}
	reg = sst_readl_alt(ctx, offset);
	dev_dbg(ctx->dev, "FW Status = reg:0x%x time=%d ret=%d\n", reg, time, (time < timeout));
	ret = time < timeout ? 0 : -ETIME;

	return ret;
}

static int sst_dsp_core_set_reset_state(struct sst_dsp_ctx  *ctx)
{
	int ret = 0;

	/* update bits */
	sst_updatel_locked(ctx, ADSPCS, CRST_MASK,
				ADSPCS_CRST(DSP_CORES_MASK));
	/* poll with timeout to check if operation successful */
	ret = sst_register_poll(ctx,
			HDA_ADSP_REG_ADSPCS,
			ADSPCS_CRST_MASK,
			ADSPCS_CRST(DSP_CORES_MASK),
			DSP_CORE_SET_RESET_STATE_TIMEOUT,
			"Set reset");

	snd_BUG_ON((sst_readl(ctx, ADSPCS) & ADSPCS_CRST(DSP_CORES_MASK)) !=
			ADSPCS_CRST(DSP_CORES_MASK));

	return ret;
}

static int sst_dsp_core_unset_reset_state(struct sst_dsp_ctx  *ctx)
{
	int ret = 0;

	/* update bits */
	sst_updatel_locked(ctx, ADSPCS, CRST_MASK, 0);

	/* poll with timeout to check if operation successful */
	ret = sst_register_poll(ctx,
			HDA_ADSP_REG_ADSPCS,
			ADSPCS_CRST_MASK,
			0,
			DSP_CORE_UNSET_RESET_STATE_TIMEOUT,
			"Unset reset");

	snd_BUG_ON((sst_readl(ctx, ADSPCS) & ADSPCS_CRST(DSP_CORES_MASK)) !=
			0);

	return ret;
}

static int sst_dsp_core_power_up(struct sst_dsp_ctx  *ctx)
{
	int ret = 0;

	dev_dbg(ctx->dev, "In %s\n", __func__);
	/* update bits */
	sst_updatel_locked(ctx,
			ADSPCS, SPA_MASK, ADSPCS_SPA(DSP_CORES_MASK));

	/* poll with timeout to check if operation successful */
	ret = sst_register_poll(ctx,
			HDA_ADSP_REG_ADSPCS,
			ADSPCS_CPA_MASK,
			ADSPCS_CPA(DSP_CORES_MASK),
			DSP_CORE_POWER_UP_TIMEOUT,
			"Power up");

	snd_BUG_ON((sst_readl(ctx, ADSPCS) & ADSPCS_CPA(DSP_CORES_MASK)) !=
			ADSPCS_CPA(DSP_CORES_MASK));

	return ret;
}

static int sst_dsp_core_power_down(struct sst_dsp_ctx  *ctx)
{
	int ret = 0;

	/* update bits */
	sst_updatel_locked(ctx, ADSPCS, SPA_MASK, 0);

	/* poll with timeout to check if operation successful */
	ret = sst_register_poll(ctx,
			HDA_ADSP_REG_ADSPCS,
			ADSPCS_SPA_MASK,
			0,
			DSP_CORE_POWER_DOWN_TIMEOUT,
			"Power down");

	return ret;
}

static bool sst_is_dsp_core_enable(struct sst_dsp_ctx  *ctx)
{
	int val = 0;
	bool is_enable;

	val = sst_readl(ctx, ADSPCS);

	is_enable = ((val & ADSPCS_CPA(DSP_CORES_MASK)) &&
			(val & ADSPCS_SPA(DSP_CORES_MASK)) &&
			!(val & ADSPCS_CRST(DSP_CORES_MASK)) &&
			!(val & ADSPCS_CSTALL(DSP_CORES_MASK)));

	dev_dbg(ctx->dev, "DSP core is enabled=%d\n", is_enable);
	return is_enable;
}


int sst_enable_dsp_core(struct sst_dsp_ctx  *ctx)
{
	int ret = 0;

	dev_dbg(ctx->dev, "In %s\n", __func__);

	/* power up */
	ret = sst_dsp_core_power_up(ctx);
	if (ret < 0) {
		dev_dbg(ctx->dev, "dsp core power up failed\n");
		return ret;
	}

	/* unset reset state */
	ret = sst_dsp_core_unset_reset_state(ctx);
	if (ret < 0) {
		dev_dbg(ctx->dev, "dsp core reset failed\n");
		return ret;
	}

	/* run core */
	sst_writel(ctx, ADSPCS, sst_readl(ctx, ADSPCS) &
			~ADSPCS_CSTALL(DSP_CORES_MASK));

	snd_BUG_ON(!sst_is_dsp_core_enable(ctx));

	return ret;
}

int sst_disable_dsp_core(struct sst_dsp_ctx  *ctx)
{
	int ret = 0;

	dev_dbg(ctx->dev, "In %s\n", __func__);

	/* stall core */
	sst_writel(ctx, ADSPCS, sst_readl(ctx, ADSPCS) |
			ADSPCS_CSTALL(DSP_CORES_MASK));

	/* set reset state */
	ret = sst_dsp_core_set_reset_state(ctx);
	if (ret < 0) {
		dev_err(ctx->dev, "dsp core reset failed\n");
		return ret;
	}

	/* power down core*/
	ret = sst_dsp_core_power_down(ctx);
	if (ret < 0) {
		dev_err(ctx->dev, "dsp core power down failed\n");
		return ret;
	}

	snd_BUG_ON(sst_is_dsp_core_enable(ctx));

	return ret;
}

static int sst_reset_dsp_core(struct sst_dsp_ctx  *ctx)
{
	int ret = 0;

	dev_dbg(ctx->dev, "In %s\n", __func__);

	/* stall core */
	sst_writel(ctx, ADSPCS, sst_readl(ctx, ADSPCS) &
			ADSPCS_CSTALL(DSP_CORES_MASK));

	/* set reset state */
	ret = sst_dsp_core_set_reset_state(ctx);
	if (ret < 0) {
		dev_err(ctx->dev, "dsp reset failed\n");
		return ret;
	}

	/* unset reset state */
	ret = sst_dsp_core_unset_reset_state(ctx);
	if (ret < 0) {
		dev_dbg(ctx->dev, "dsp unset reset failes\n");
		return ret;
	}

	/* run core */
	sst_writel(ctx, ADSPCS, sst_readl(ctx, ADSPCS) &
			~ADSPCS_CSTALL(DSP_CORES_MASK));

	snd_BUG_ON(sst_is_dsp_core_enable(ctx));

	return ret;
}

int sst_boot_dsp(struct sst_dsp_ctx *ctx)
{
	int ret = 0;

	dev_dbg(ctx->dev, "In %s\n", __func__);

	if (sst_is_dsp_core_enable(ctx)) {
		/* Dsp core powered up - simply reset core */
		dev_dbg(ctx->dev, "dsp core is already enabled, so reset the dap core\n");
		ret = sst_reset_dsp_core(ctx);
	} else {
		/*disable and enable to make sure DSP is invalid state */
		ret = sst_disable_dsp_core(ctx);

		if (ret < 0) {
			dev_err(ctx->dev, "dsp disable core failes\n");
			return ret;
		}
		ret = sst_enable_dsp_core(ctx);
	}

	return ret;
}

int sst_dsp_init(struct sst_dsp_ctx *ctx)
{
	int ret = 0;

	/* initialize IPC */
	ctx->ipc = ipc_init(ctx->dev, ctx);
	if (ctx->ipc == NULL)
		ret = -ENODEV;

	/* Now let's request the IRQ */
	sst_acquire_irq(ctx);

	return ret;
}

int sst_dsp_free(struct sst_dsp_ctx *dsp)
{
	int ret = 0;

	ipc_int_disable(dsp);

	free_irq(dsp->irq, dsp);
	ipc_free(dsp->ipc);
	kfree(dsp);
	return ret;
}
EXPORT_SYMBOL_GPL(sst_dsp_free);

int sst_dsp_set_power_state(struct sst_dsp_ctx *ctx, int state)
{
	int ret = 0;

	if (state == SST_DSP_POWER_D0)
		ret = ctx->ops.set_state_D0(ctx);
	else if (state == SST_DSP_POWER_D3)
		ret = ctx->ops.set_state_D3(ctx);
	else
		dev_err(ctx->dev, "Power State=%x not supported", state);
	return ret;
}
EXPORT_SYMBOL_GPL(sst_dsp_set_power_state);

/*
 * interrupt handler
 */
static irqreturn_t sst_interrupt(int irq, void *dev_id)
{
	struct sst_dsp_ctx *ctx = (struct sst_dsp_ctx *) dev_id;
	u32 val;
	irqreturn_t result = IRQ_NONE;

	spin_lock(&ctx->reg_lock);

	val = sst_readl(ctx, ADSPIS);

	if (val & ADSPIS_IPC) {
		ipc_int_disable(ctx);
		result = IRQ_WAKE_THREAD;
	}

	spin_unlock(&ctx->reg_lock);
	return result;
}

static int sst_acquire_irq(struct sst_dsp_ctx *ctx)
{
	if (request_threaded_irq(ctx->irq, sst_interrupt,
			sst_irq_thread_handler, IRQF_SHARED,
			KBUILD_MODNAME, ctx)) {
		dev_err(ctx->dev, "unable to grab threaded IRQ %d, disabling device\n", ctx->irq);
		return -1;
	}
	return 0;
}

MODULE_DESCRIPTION("HDA SST/IPC Library");
MODULE_LICENSE("GPL v2");
