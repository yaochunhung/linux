/*
 *  soc-hda-ssp-config.c - ASOC HDA SSP register configuration functionality
 *
 *  Copyright (C) 2014 Intel Corp
 *  Author:  Hardik Shah <hardik.t.shah@intel.com>
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
#include "intel-ssp-config.h"
#include "soc-hda-controller.h"

#define HDA_SSP_MAX_FREQ_192 19200000

struct hda_ssp_regs {
	u32 hda_ssc0;
	u32 hda_ssc1;
	u32 hda_ssto;
	u32 hda_sspsp;
	u32 hda_sstsa;
	u32 hda_ssrsa;
	u32 hda_ssc2;
	u32 hda_sspsp2;
	u32 hda_ssc3;
	u32 hda_ssioc;
};


static void azx_set_default_ssp_regs_v1(struct hda_ssp_regs *regs)
{
	regs->hda_ssc0 = 0;
	regs->hda_ssc0 = (HDA_SSC0_FRF_DEF_V1 << HDA_SSC0_FRF_SHIFT) |
			(HDA_SSC0_ECS_DEF_V1 << HDA_SSC0_ECS_SHIFT) |
			(HDA_SSC0_SSE_DEF_V1 << HDA_SSC0_SSE_SHIFT) |
			(HDA_SSC0_NCS_DEF_V1 << HDA_SSC0_NCS_SHIFT) |
			(HDA_SSC0_RIM_DEF_V1 << HDA_SSC0_RIM_SHIFT) |
			(HDA_SSC0_ACS_DEF_V1 << HDA_SSC0_ACS_SHIFT) |
			(HDA_SSC0_MOD_DEF_V1 << HDA_SSC0_MOD_SHIFT);

	regs->hda_ssc1 = 0;
	regs->hda_ssc1 = (HDA_SSC1_RIE_DEF_V1 << HDA_SSC1_RIE_SHIFT) |
			(HDA_SSC1_TIE_DEF_V1 << HDA_SSC1_TIE_SHIFT) |
			(HDA_SSC1_LBM_DEF_V1 << HDA_SSC1_LBM_SHIFT) |
			(HDA_SSC1_SPO_DEF_V1 << HDA_SSC1_SPO_SHIFT) |
			(HDA_SSC1_SPH_DEF_V1 << HDA_SSC1_SPH_SHIFT) |
			(HDA_SSC1_MWDS_DEF_V1 << HDA_SSC1_MWDS_SHIFT) |
			(HDA_SSC1_EFWR_DEF_V1 << HDA_SSC1_EFWR_SHIFT) |
			(HDA_SSC1_STRF_DEF_V1 << HDA_SSC1_STRF_SHIFT) |
			(HDA_SSC1_IFS_DEF_V1 << HDA_SSC1_IFS_SHIFT) |
			(HDA_SSC1_PINTE_DEF_V1 << HDA_SSC1_PINTE_SHIFT) |
			(HDA_SSC1_TINTE_DEF_V1 << HDA_SSC1_TINTE_SHIFT) |
			(HDA_SSC1_RSRE_DEF_V1 << HDA_SSC1_RSRE_SHIFT) |
			(HDA_SSC1_TSRE_DEF_V1 << HDA_SSC1_TSRE_SHIFT) |
			(HDA_SSC1_TRAIL_DEF_V1 << HDA_SSC1_TRAIL_SHIFT) |
			(HDA_SSC1_RWOT_DEF_V1 << HDA_SSC1_RWOT_SHIFT) |
			(HDA_SSC1_ECRB_DEF_V1 << HDA_SSC1_ECRB_SHIFT) |
			(HDA_SSC1_ECRA_DEF_V1 << HDA_SSC1_ECRA_SHIFT) |
			(HDA_SSC1_SCFR_DEF_V1 << HDA_SSC1_SCFR_SHIFT) |
			(HDA_SSC1_EBCEI_DEF_V1 << HDA_SSC1_EBCEI_SHIFT) |
			(HDA_SSC1_TTE_DEF_V1 << HDA_SSC1_TTE_SHIFT) |
			(HDA_SSC1_TTELP_DEF_V1 << HDA_SSC1_TTELP_SHIFT);

	regs->hda_ssto = 0;
	regs->hda_ssto = HDA_SST0_TIMEOUT_DEF_V1 << HDA_SST0_TIMEOUT_SHIFT;

	regs->hda_sspsp = 0;
	regs->hda_sspsp = (HDA_SSPSP_ETDS_DEF_V1 << HDA_SSPSP_ETDS_SHIFT) |
			(HDA_SSPSP_STRTDLY_DEF_V1 << HDA_SSPSP_STRTDLY_SHIFT) |
			(HDA_SSPSP_DMYSTRT_DEF_V1 << HDA_SSPSP_DMYSTRT_SHIFT) |
			(HDA_SSPSP_SFRMDLY_DEF_V1 << HDA_SSPSP_SFRMDLY_SHIFT) |
			(HDA_SSPSP_DMYSTOP_DEF_V1 << HDA_SSPSP_DMYSTOP_SHIFT) |
			(HDA_SSPSP_EDMYSTOP_DEF_V1 << HDA_SSPSP_EDMYSTOP_SHIFT);

	regs->hda_sstsa = 0;
	regs->hda_sstsa = (HDA_SSTSA_TXEN_DEF_V1 << HDA_SSTSA_TXEN_SHIFT) |
			(HDA_SSTSA_TTSA_DEF_V1 << HDA_SSTSA_TTSA_SHIFT);

	regs->hda_ssrsa = 0;
	regs->hda_ssrsa = (HDA_SSRSA_RXEN_DEF_V1 << HDA_SSRSA_RXEN_SHIFT) |
			(HDA_SSTSA_RTSA_DEF_V1 << HDA_SSRSA_RTSA_SHIFT);

	regs->hda_ssc2 = 0;
	regs->hda_ssc2 = (HDA_SSC2_TURM0_DEF_V1 << HDA_SSC2_TURM0_SHIFT) |
		(HDA_SSC2_TURM1_DEF_V1 << HDA_SSC2_TURM1_SHIFT) |
		(HDA_SSC2_PSPMB2BFMFD_DEF_V1 << HDA_SSC2_PSPMB2BFMFD_SHIFT) |
		(HDA_SSC2_PSPSRWFDFD_DEF_V1 << HDA_SSC2_PSPSRWFDFD_SHIFT) |
		(HDA_SSC2_PSPSTWFDFD_DEF_V1 << HDA_SSC2_PSPSTWFDFD_SHIFT) |
		(HDA_SSC2_PSPMFSRTPFD_DEF_V1 << HDA_SSC2_PSPMFSRTPFD_SHIFT) |
		(HDA_SSC2_ROFD_DEF_V1 << HDA_SSC2_ROFD_SHIFT) |
		(HDA_SSC2_C2DFFD_DEF_V1 << HDA_SSC2_C2DFFD_SHIFT) |
		(HDA_SSC2_TSAFD_DEF_V1 << HDA_SSC2_TSAFD_SHIFT) |
		(HDA_SSC2_RSAFD_DEF_V1 << HDA_SSC2_RSAFD_SHIFT) |
		(HDA_SSC2_RMFGFD_DEF_V1 << HDA_SSC2_RMFGFD_SHIFT) |
		(HDA_SSC2_TURFD_DEF_V1 << HDA_SSC2_TURFD_SHIFT) |
		(HDA_SSC2_TUDFD_DEF_V1 << HDA_SSC2_TUDFD_SHIFT) |
		(HDA_SSC2_BCEFD_DEF_V1 << HDA_SSC2_BCEFD_SHIFT) |
		(HDA_SSC2_SDFD_DEF_V1 << HDA_SSC2_SDFD_SHIFT) |
		(HDA_SSC2_SDHFD_DEF_V1 << HDA_SSC2_SDHFD_SHIFT) |
		(HDA_SSC2_SDPM_DEF_V1 << HDA_SSC2_SDPM_SHIFT) |
		(HDA_SSC2_LJDFD_DEF_V1 << HDA_SSC2_LJDFD_SHIFT) |
		(HDA_SSC2_MMRATF_DEF_V1 << HDA_SSC2_MMRATF_SHIFT) |
		(HDA_SSC2_SMTATF_DEF_V1 << HDA_SSC2_SMTATF_SHIFT);

	regs->hda_ssc3 = 0;
	regs->hda_ssc3 = (HDA_SSC3_TFT_DEF_V1 << HDA_SSC3_TFT_SHIFT) |
			(HDA_SSC3_RFT_DEF_V1 << HDA_SSC3_RFT_SHIFT);

	regs->hda_ssioc = 0;
	regs->hda_ssioc = (HDA_SSIOC_RXDPDEB_DEF_V1 << HDA_SSIOC_RXDPDEB_SHIFT) |
		(HDA_SSIOC_TXDPDEB_DEF_V1 << HDA_SSIOC_TXDPDEB_SHIFT) |
		(HDA_SSIOC_SFRMPDEB_DEF_V1 << HDA_SSIOC_SFRMPDEB_SHIFT) |
		(HDA_SSIOC_SCLKPDEB_DEF_V1 << HDA_SSIOC_SCLKPDEB_SHIFT) |
		(HDA_SSIOC_SFCR_DEF_V1 << HDA_SSIOC_SFCR_SHIFT) |
		(HDA_SSIOC_SCOE_DEF_V1 << HDA_SSIOC_SCOE_SHIFT);

	regs->hda_sspsp2 = 0;

}

static void azx_print_ssp_regs(struct sst_dsp_ctx *ctx,
						struct hda_ssp_regs *regs)
{
	dev_dbg(ctx->dev, "ssc0\t\t %x\n", regs->hda_ssc0);
	dev_dbg(ctx->dev, "ssc1\t\t %x\n", regs->hda_ssc1);
	dev_dbg(ctx->dev, "sst0\t\t %x\n", regs->hda_ssto);
	dev_dbg(ctx->dev, "sspsp\t\t %x\n", regs->hda_sspsp);
	dev_dbg(ctx->dev, "sstsa\t\t %x\n", regs->hda_sstsa);
	dev_dbg(ctx->dev, "ssrsa\t\t %x\n", regs->hda_ssrsa);
	dev_dbg(ctx->dev, "ssc2\t\t %x\n", regs->hda_ssc2);
	dev_dbg(ctx->dev, "sspsp2\t\t %x\n", regs->hda_sspsp2);
	dev_dbg(ctx->dev, "ssc3\t\t %x\n", regs->hda_ssc3);
	dev_dbg(ctx->dev, "ssioc\t\t %x\n", regs->hda_ssioc);
}

static int azx_find_ssp_clk_divisor(struct sst_dsp_ctx *ctx, int fs,
			int slots, int s_fmt, int *div, int *dummy_bits)
{
	int divisor, mod;
	int found = 0;
	int req_bclk;
	int dummy = 0;

	dev_dbg(ctx->dev, "fs = %d, slots = %d s_fmt = %d\n", fs, slots, s_fmt);
	req_bclk = fs * slots * s_fmt;
	if (req_bclk == 0) {
		dev_err(ctx->dev, "Required bit clock = 0\n");
		return -EINVAL;
	}
	if (req_bclk > HDA_SSP_MAX_FREQ_192) {
		dev_err(ctx->dev, "Bit clock is %d, greater than max SSP freq\n", req_bclk);
		return -EINVAL;
	}
	dummy = 0;
	/* Find the lowest bit clock possible for particular configuration */
	do {
		req_bclk = ((s_fmt * fs * slots) + (fs * dummy));
		mod = HDA_SSP_MAX_FREQ_192 % req_bclk;
		divisor = HDA_SSP_MAX_FREQ_192 / req_bclk;
		if (!mod) {
			found = 1;
			break;
		}
		/* Dont add odd number of dummy bits, since I2S requires
		 * dummy bit after each slot/channel
		 */
		dummy += 2;
		if (dummy > (HDA_SSPSP2_FEP_MASK >> HDA_SSPSP2_FEP_SHIFT)) {
			dev_err(ctx->dev, "Dummy bit greater than what SSP can support\n");
			return -EINVAL;
		}

	} while (divisor > 0);
	if (found) {
		*div = divisor;
		*dummy_bits = dummy;
		return 0;
	}
	dev_err(ctx->dev, "Suitable divisor not found for SSP configuration\n");

	return -EINVAL;
}
int azx_calculate_ssp_regs(struct sst_dsp_ctx *ctx, struct azx_dai_config *cfg,
						void *ssp_regs)
{
	struct hda_ssp_regs regs;
	int ret = 0;
	int div, dummy;
	int dss, edss;
	int edmystop, dmystop;
	struct azx_ssp_dai_config *ssp_cfg = &cfg->ssp_dai_config;

	azx_set_default_ssp_regs_v1(&regs);
	dev_dbg(ctx->dev, "Default value of registers set to:\n");
	azx_print_ssp_regs(ctx, &regs);

	if (ssp_cfg->fs_slave)
		regs.hda_ssc1 |= HDA_SSC1_SFRMDIR_MASK;
	if (ssp_cfg->bclk_slave)
		regs.hda_ssc1 |= HDA_SSC1_SCLKDIR_MASK;
	if (!ssp_cfg->fs_invert)
		regs.hda_sspsp |= ((0x1 << HDA_SSPSP_SFRMP_SHIFT) &
				HDA_SSPSP_SFRMP_MASK);
	if (ssp_cfg->bclk_invert)
		regs.hda_sspsp |= ((0x1 << HDA_SSPSP_SCMODE_SHIFT) &
				HDA_SSPSP_SCMODE_MASK);

	if (ssp_cfg->ssp_mode == HDA_SSP_MODE_I2S)
		/* I2S mode, means 2 slots */
		ssp_cfg->slots = 2;

	ret = azx_find_ssp_clk_divisor(ctx, cfg->sampling_rate, ssp_cfg->slots,
					cfg->s_fmt, &div, &dummy);
	if (ret) {
		dev_err(ctx->dev, "Clock disior not possible\n");
		return -EINVAL;
	}
	/* Program dividier */
	regs.hda_ssc0 |= ((div - 1) << HDA_SSC0_SRC_SHIFT) & HDA_SSC0_SCR_MASK;


	if (ssp_cfg->slot_width != cfg->s_fmt) {
		dev_err(ctx->dev, "slot_width must be equal to format bits\n");
		return -EINVAL;
	}

	if (cfg->s_fmt > 16) {
		edss = 1;
		dss = cfg->s_fmt - 16;
	} else {
		edss = 0;
		dss = cfg->s_fmt;
	}
	dss -= 1;
	regs.hda_ssc0 |= ((dss << HDA_SSC0_DSS_SHIFT) & HDA_SSC0_DSS_MASK);
	regs.hda_ssc0 |= ((edss << HDA_SSC0_EDSS_SHIFT) & HDA_SSC0_EDSS_MASK);

	if (ssp_cfg->ssp_mode != HDA_SSP_MODE_I2S) {
		regs.hda_sspsp2 |= (dummy << HDA_SSPSP2_FEP_SHIFT) &
							HDA_SSPSP2_FEP_MASK;
		regs.hda_sspsp |= (0x1 << HDA_SSPSP_SFRMWDTH_SHIFT) &
					HDA_SSPSP_SFRMWDTH_MASK;
		if (ssp_cfg->ssp_mode == HDA_SSP_MODE_DSP_A)
			regs.hda_sspsp |= (0x1 << HDA_SSPSP_FSRT_SHIFT) &
					HDA_SSPSP_FSRT_MASK;
		else
			regs.hda_sspsp &= ~(HDA_SSPSP_FSRT_MASK);
	} else {
		/* Divide dummy bits / 2 */
		regs.hda_sspsp |= (0x1 << HDA_SSPSP_FSRT_SHIFT) &
				HDA_SSPSP_FSRT_MASK;
		dummy = dummy >> 1;
		dmystop = dummy & 0x3;
		edmystop = (dummy >> 2);
		regs.hda_sspsp |= (dmystop << HDA_SSPSP_DMYSTOP_SHIFT) &
							HDA_SSPSP_DMYSTOP_MASK;
		regs.hda_sspsp |= (edmystop << HDA_SSPSP_EDMYSTOP_SHIFT) &
						HDA_SSPSP_EDMYSTOP_MASK;
		regs.hda_sspsp |= ((cfg->s_fmt + dummy) << HDA_SSPSP_SFRMWDTH_SHIFT) &
							HDA_SSPSP_SFRMWDTH_MASK;
		ssp_cfg->slots = 2;
	}
	if (ssp_cfg->slots > HDA_SSP_MAX_SLOTS) {
		dev_err(ctx->dev, "Max slots = %d\n", HDA_SSP_MAX_SLOTS);
		return -EINVAL;
	}
	regs.hda_ssc0 |= ((ssp_cfg->slots - 1) << HDA_SSC0_FRDC_SHIFT) &
			HDA_SSC0_FRDC_MASK;

	memcpy(ssp_regs, &regs, sizeof(struct hda_ssp_regs));
	dev_dbg(ctx->dev, "Registers updated to\n");
	azx_print_ssp_regs(ctx, &regs);
	return 0;

}
