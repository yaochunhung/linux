/*Add header */
#ifndef __HDA_SST_DSP_H__
#define __HDA_SST_DSP_H__

#include <linux/spinlock.h>
#include <sound/memalloc.h>

#define sst_writel_andor(ctx, reg, mask_and, mask_or) \
	sst_writel_traced( \
	ctx, \
	reg, \
	(sst_readl_traced(ctx, (reg)) & (mask_and) | (mask_or)))

#define sst_readb(ctx, reg) \
	sst_readb_traced(ctx, HDA_ADSP_REG_##reg)
#define sst_readw(ctx, reg) \
	sst_readw_traced(ctx, HDA_ADSP_REG_##reg)
#define sst_readl(ctx, reg) \
	sst_readl_traced(ctx, HDA_ADSP_REG_##reg)
#define sst_readl_alt(ctx, reg) \
	sst_readl_traced(ctx, reg)

#define sst_writeb(ctx, reg, value) \
	sst_writeb_traced(ctx, HDA_ADSP_REG_##reg, (value))
#define sst_writew(ctx, reg, value) \
	sst_writew_traced(ctx, HDA_ADSP_REG_##reg, (value))
#define sst_writel(ctx, reg, value) \
	sst_writel_traced(ctx, HDA_ADSP_REG_##reg, (value))

#if 0
#define sst_writel_andor(ctx, reg, mask_and, mask_or) \
	sst_writel( \
		ctx, \
		reg, \
		(sst_readl(ctx, (reg)) & (mask_and) | (mask_or)))

#define sst_readb(ctx, reg) \
	readb((ctx)->mmio_base + HDA_ADSP_REG_##reg);
#define sst_readw(ctx, reg) \
	readw((ctx)->mmio_base + HDA_ADSP_REG_##reg)
#define sst_readl(ctx, reg) \
	readl((ctx)->mmio_base + HDA_ADSP_REG_##reg)
#define sst_readl_alt(ctx, reg) \
	readl((ctx)->mmio_base + reg)

#define sst_writeb(ctx, reg, value) \
	writeb(value, (ctx)->mmio_base + HDA_ADSP_REG_##reg)
#define sst_writew(ctx, reg, value) \
	writew(value, (ctx)->mmio_base + HDA_ADSP_REG_##reg)
#define sst_writel(ctx, reg, value) \
	writel(value, (ctx)->mmio_base + HDA_ADSP_REG_##reg)
#endif

#define sst_updatel(ctx, reg, mask, value) \
	sst_updatel_bits(ctx, HDA_ADSP_REG_##reg, reg##_##mask, (value))

#define sst_updatel_locked(ctx, reg, mask, value) \
	sst_updatel_bits_locked(ctx, HDA_ADSP_REG_##reg, \
		reg##_##mask, (value))

/* Intel® HD Audio General DSP Registers */
#define HDA_ADSP_GEN_BASE		0x0
#define HDA_ADSP_REG_ADSPCS		(HDA_ADSP_GEN_BASE + 0x04)
#define HDA_ADSP_REG_ADSPIC		(HDA_ADSP_GEN_BASE + 0x08)
#define HDA_ADSP_REG_ADSPIS		(HDA_ADSP_GEN_BASE + 0x0C)
#define HDA_ADSP_REG_ADSPIC2		(HDA_ADSP_GEN_BASE + 0x10)
#define HDA_ADSP_REG_ADSPIS2		(HDA_ADSP_GEN_BASE + 0x14)

/* Intel® HD Audio Inter-Processor Communication Registers */
#define HDA_ADSP_IPC_BASE		0x40
#define HDA_ADSP_REG_HIPCT		(HDA_ADSP_IPC_BASE + 0x00)
#define HDA_ADSP_REG_HIPCTE		(HDA_ADSP_IPC_BASE + 0x04)
#define HDA_ADSP_REG_HIPCI		(HDA_ADSP_IPC_BASE + 0x08)
#define HDA_ADSP_REG_HIPCIE		(HDA_ADSP_IPC_BASE + 0x0C)
#define HDA_ADSP_REG_HIPCCTL		(HDA_ADSP_IPC_BASE + 0x10)

/*  HIPCI */
#define HDA_ADSP_REG_HIPCI_BUSY		(0x1 << 31)

/* HIPCIE */
#define HDA_ADSP_REG_HIPCIE_DONE	(0x1 << 30)

/* HIPCCTL */
#define HDA_ADSP_REG_HIPCCTL_DONE	(0x1 << 1)
#define HDA_ADSP_REG_HIPCCTL_BUSY       (0x1 << 0)

/* HIPCT */
#define HDA_ADSP_REG_HIPCT_BUSY		(0x1 << 31)

/* Intel® HD Audio Code Loader DMA Registers */
#define HDA_ADSP_LOADER_BASE	0x80

/* Intel® HD Audio L2 local SRAM Window 1 */
#define HDA_ADSP_SRAM1_BASE		0xA000

/* Intel® HD Audio L2 local SRAM Window 2 */
#define HDA_ADSP_SRAM2_BASE		0xC000

/* Intel® HD Audio L2 local SRAM Window 3 */
#define HDA_ADSP_SRAM3_BASE		0xE000

#define HDA_ADSP_MMIO_LEN		0x10000

#define WINDOW0_STAT_SIZE		0x800

#define WINDOW0_UP_SIZE			0x800

#define WINDOW1_SIZE			0x1000

#define FW_STATUS_MASK			0xf

#define FW_ROM_INIT			0x0
#define FW_ROM_INIT_DONE		0x1
#define FW_ROM_MEM_PWR_DONE		0x2
#define FW_ROM_MANIFEST_LOADED		0X3
#define FW_ROM_MANIFEST_VERIFIED	0x4
#define FW_ROM_FEAT_MASK_VERIFIED	0x5
#define FW_ROM_BASEFW_FOUND		0x6
#define FW_ROM_BASEFW_BA_VALID		0x7
#define FW_ROM_BASEFW_TOTAL_PWR		0x8
#define FW_ROM_BASEFW_TEXT_LOADED	0x9
#define FW_ROM_BASEFW_TEXT_HASHED	0xa
#define FW_ROM_BASEFW_RODA_LOADED	0xb
#define FW_ROM_BASEFW_LOAD_HASHED	0xc
#define FW_ROM_BASEFW_HASH_VER		0xd
#define FW_ROM_BASEFW_START_FOUND	0xe
#define FW_ROM_BASEFW_ENTERED		0xf

#define FW_CL_STREAM_NUMER	0xf

#define DMA_ADDRESS_128_BITS_ALIGNMENT 7
#define BDL_ALIGN(x)	(x >> DMA_ADDRESS_128_BITS_ALIGNMENT)

#define ADSPIC_IPC                      1
#define ADSPIS_IPC			1

/* Code Loader -
 * Stream Registers */
#define HDA_ADSP_REG_CL_SD_CTL			(HDA_ADSP_LOADER_BASE + 0x00)
#define HDA_ADSP_REG_CL_SD_STS			(HDA_ADSP_LOADER_BASE + 0x03)
#define HDA_ADSP_REG_CL_SD_LPIB			(HDA_ADSP_LOADER_BASE + 0x04)
#define HDA_ADSP_REG_CL_SD_CBL			(HDA_ADSP_LOADER_BASE + 0x08)
#define HDA_ADSP_REG_CL_SD_LVI			(HDA_ADSP_LOADER_BASE + 0x0c)
#define HDA_ADSP_REG_CL_SD_FIFOW		(HDA_ADSP_LOADER_BASE + 0x0e)
#define HDA_ADSP_REG_CL_SD_FIFOSIZE		(HDA_ADSP_LOADER_BASE + 0x10)
#define HDA_ADSP_REG_CL_SD_FORMAT		(HDA_ADSP_LOADER_BASE + 0x12)
#define HDA_ADSP_REG_CL_SD_FIFOL		(HDA_ADSP_LOADER_BASE + 0x14)
#define HDA_ADSP_REG_CL_SD_BDLPL		(HDA_ADSP_LOADER_BASE + 0x18)
#define HDA_ADSP_REG_CL_SD_BDLPU		(HDA_ADSP_LOADER_BASE + 0x1c)

/* Code Loader -
 * Software Position Based FIFO Capability Registers */
#define HDA_ADSP_REG_CL_SPBFIFO				(HDA_ADSP_LOADER_BASE + 0x20)
#define HDA_ADSP_REG_CL_SPBFIFO_SPBFCH		(HDA_ADSP_REG_CL_SPBFIFO + 0x0)
#define HDA_ADSP_REG_CL_SPBFIFO_SPBFCCTL	(HDA_ADSP_REG_CL_SPBFIFO + 0x4)
#define HDA_ADSP_REG_CL_SPBFIFO_SPIB		(HDA_ADSP_REG_CL_SPBFIFO + 0x8)
#define HDA_ADSP_REG_CL_SPBFIFO_MAXFIFOS	(HDA_ADSP_REG_CL_SPBFIFO + 0xc)


/* Code Loader -
 * Stream Descriptor x Control */
/* Stream Reset */
#define CL_SD_CTL_SRST_SHIFT	0
#define CL_SD_CTL_SRST_MASK		(1 << CL_SD_CTL_SRST_SHIFT)
#define CL_SD_CTL_SRST(x) \
	((x << CL_SD_CTL_SRST_SHIFT) & CL_SD_CTL_SRST_MASK)

/* Stream Run */
#define CL_SD_CTL_RUN_SHIFT		1
#define CL_SD_CTL_RUN_MASK		(1 << CL_SD_CTL_RUN_SHIFT)
#define CL_SD_CTL_RUN(x) \
	((x << CL_SD_CTL_RUN_SHIFT) & CL_SD_CTL_RUN_MASK)

/* Interrupt On Completion Enable */
#define CL_SD_CTL_IOCE_SHIFT	2
#define CL_SD_CTL_IOCE_MASK		(1 << CL_SD_CTL_IOCE_SHIFT)
#define CL_SD_CTL_IOCE(x) \
	((x << CL_SD_CTL_IOCE_SHIFT) & CL_SD_CTL_IOCE_MASK)

/* FIFO Error Interrupt Enable */
#define CL_SD_CTL_FEIE_SHIFT	3
#define CL_SD_CTL_FEIE_MASK		(1 << CL_SD_CTL_FEIE_SHIFT)
#define CL_SD_CTL_FEIE(x) \
	((x << CL_SD_CTL_FEIE_SHIFT) & CL_SD_CTL_FEIE_MASK)

/* Descriptor Error Interrupt Enable */
#define CL_SD_CTL_DEIE_SHIFT	4
#define CL_SD_CTL_DEIE_MASK		(1 << CL_SD_CTL_DEIE_SHIFT)
#define CL_SD_CTL_DEIE(x) \
	((x << CL_SD_CTL_DEIE_SHIFT) & CL_SD_CTL_DEIE_MASK)

/* FIFO Limit Change */
#define CL_SD_CTL_FIFOLC_SHIFT	5
#define CL_SD_CTL_FIFOLC_MASK	(1 << CL_SD_CTL_FIFOLC_SHIFT)
#define CL_SD_CTL_FIFOLC(x) \
	((x << CL_SD_CTL_FIFOLC_SHIFT) & CL_SD_CTL_FIFOLC_MASK)

/* Stripe Control */
#define CL_SD_CTL_STRIPE_SHIFT	16
#define CL_SD_CTL_STRIPE_MASK	(0x3 << CL_SD_CTL_STRIPE_SHIFT)
#define CL_SD_CTL_STRIPE(x) \
	((x << CL_SD_CTL_STRIPE_SHIFT) & CL_SD_CTL_STRIPE_MASK)

/* Traffic Priority */
#define CL_SD_CTL_TP_SHIFT		18
#define CL_SD_CTL_TP_MASK		(1 << CL_SD_CTL_TP_SHIFT)
#define CL_SD_CTL_TP(x) \
	((x << CL_SD_CTL_TP_SHIFT) & CL_SD_CTL_TP_MASK)

/* Bidirectional Direction Control */
#define CL_SD_CTL_DIR_SHIFT		19
#define CL_SD_CTL_DIR_MASK		(1 << CL_SD_CTL_DIR_SHIFT)
#define CL_SD_CTL_DIR(x) \
	((x << CL_SD_CTL_DIR_SHIFT) & CL_SD_CTL_DIR_MASK)

/* Stream Number */
#define CL_SD_CTL_STRM_SHIFT	20
#define CL_SD_CTL_STRM_MASK		(0xf << CL_SD_CTL_STRM_SHIFT)
#define CL_SD_CTL_STRM(x) \
	((x << CL_SD_CTL_STRM_SHIFT) & CL_SD_CTL_STRM_MASK)


/* Code Loader -
 * Stream Descriptor x Status */
/* Buffer Completion Interrupt Status */
#define CL_SD_STS_BCIS(x)		CL_SD_CTL_IOCE(x)

/* FIFO Error */
#define CL_SD_STS_FIFOE(x)		CL_SD_CTL_FEIE(x)

/* Descriptor Error */
#define CL_SD_STS_DESE(x)		CL_SD_CTL_DEIE(x)

/* FIFO Ready */
#define CL_SD_STS_FIFORDY(x)	CL_SD_CTL_FIFOLC(x)


/* Code Loader -
 * Stream Descriptor x Last Valid Index */
#define CL_SD_LVI_SHIFT		0
#define CL_SD_LVI_MASK		(0xff << CL_SD_LVI_SHIFT)
#define CL_SD_LVI(x)		((x << CL_SD_LVI_SHIFT) & CL_SD_LVI_MASK)


/* Code Loader -
 * Stream Descriptor x FIFO Eviction Watermark */
#define CL_SD_FIFOW_SHIFT	0
#define CL_SD_FIFOW_MASK	(0x7 << CL_SD_FIFOW_SHIFT)
#define CL_SD_FIFOW(x)		((x << CL_SD_FIFOW_SHIFT) & CL_SD_FIFOW_MASK)


/* Code Loader -
 * Stream Descriptor x Buffer Descriptor List Pointer Lower Base Address */
/* Protect */
#define CL_SD_BDLPLBA_PROT_SHIFT	0
#define CL_SD_BDLPLBA_PROT_MASK		(1 << CL_SD_BDLPLBA_PROT_SHIFT)
#define CL_SD_BDLPLBA_PROT(x) \
	((x << CL_SD_BDLPLBA_PROT_SHIFT) & CL_SD_BDLPLBA_PROT_MASK)

/* Buffer Descriptor List Lower Base Address */
#define CL_SD_BDLPLBA_SHIFT	7
#define CL_SD_BDLPLBA_MASK	(0x1ffffff << CL_SD_BDLPLBA_SHIFT)
#define CL_SD_BDLPLBA(x) \
	((BDL_ALIGN(lower_32_bits(x)) << CL_SD_BDLPLBA_SHIFT) & CL_SD_BDLPLBA_MASK)

/* Buffer Descriptor List Upper Base Address */
#define CL_SD_BDLPUBA_SHIFT	0
#define CL_SD_BDLPUBA_MASK	(0xffffffff << CL_SD_BDLPUBA_SHIFT)
#define CL_SD_BDLPUBA(x) \
	((upper_32_bits(x) << CL_SD_BDLPUBA_SHIFT) & CL_SD_BDLPUBA_MASK)

/* Code Loader - Software Position Based FIFO
 * Capability Registers x Software Position Based FIFO Header */

/* Next Capability Pointer */
#define CL_SPBFIFO_SPBFCH_PTR_SHIFT	0
#define CL_SPBFIFO_SPBFCH_PTR_MASK	(0xff << CL_SPBFIFO_SPBFCH_PTR_SHIFT)
#define CL_SPBFIFO_SPBFCH_PTR(x) \
	((x << CL_SPBFIFO_SPBFCH_PTR_SHIFT) & CL_SPBFIFO_SPBFCH_PTR_MASK)

/* Capability Identifier */
#define CL_SPBFIFO_SPBFCH_ID_SHIFT	16
#define CL_SPBFIFO_SPBFCH_ID_MASK	(0xfff << CL_SPBFIFO_SPBFCH_ID_SHIFT)
#define CL_SPBFIFO_SPBFCH_ID(x) \
	((x << CL_SPBFIFO_SPBFCH_ID_SHIFT) & CL_SPBFIFO_SPBFCH_ID_MASK)

/* Capability Version */
#define CL_SPBFIFO_SPBFCH_VER_SHIFT	28
#define CL_SPBFIFO_SPBFCH_VER_MASK	(0xf << CL_SPBFIFO_SPBFCH_VER_SHIFT)
#define CL_SPBFIFO_SPBFCH_VER(x) \
	((x << CL_SPBFIFO_SPBFCH_VER_SHIFT) & CL_SPBFIFO_SPBFCH_VER_MASK)


/* Code Loader -
   Software Position Based FIFO Control */
/* Software Position in Buffer Enable */
#define CL_SPBFIFO_SPBFCCTL_SPIBE_SHIFT	0
#define CL_SPBFIFO_SPBFCCTL_SPIBE_MASK	(1 << CL_SPBFIFO_SPBFCCTL_SPIBE_SHIFT)
#define CL_SPBFIFO_SPBFCCTL_SPIBE(x) \
	((x << CL_SPBFIFO_SPBFCCTL_SPIBE_SHIFT) & CL_SPBFIFO_SPBFCCTL_SPIBE_MASK)


/* ADSPCS - Audio DSP Control & Status */
#define DSP_CORES 2
#define DSP_CORES_MASK ((1 << DSP_CORES) - 1)

/* Core Reset - asserted high */
#define ADSPCS_CRST_SHIFT	0
#define ADSPCS_CRST_MASK	(DSP_CORES_MASK << ADSPCS_CRST_SHIFT)
#define ADSPCS_CRST(x)		((x << ADSPCS_CRST_SHIFT) & ADSPCS_CRST_MASK)

/* Core run/stall - when set to '1' core is stalled */
#define ADSPCS_CSTALL_SHIFT	8
#define ADSPCS_CSTALL_MASK	(DSP_CORES_MASK << ADSPCS_CSTALL_SHIFT)
#define ADSPCS_CSTALL(x)	((x << ADSPCS_CSTALL_SHIFT) & ADSPCS_CSTALL_MASK)

/* Set Power Active - when set to '1' turn cores on */
#define ADSPCS_SPA_SHIFT	16
#define ADSPCS_SPA_MASK		(DSP_CORES_MASK << ADSPCS_SPA_SHIFT)
#define ADSPCS_SPA(x)		((x << ADSPCS_SPA_SHIFT) & ADSPCS_SPA_MASK)

/* Current Power Active - power status of cores, set by hardware */
#define ADSPCS_CPA_SHIFT	24
#define ADSPCS_CPA_MASK		(DSP_CORES_MASK << ADSPCS_CPA_SHIFT)
#define ADSPCS_CPA(x)		((x << ADSPCS_CPA_SHIFT) & ADSPCS_CPA_MASK)


/* Broxton specificdefinitions */

#define HDA_ADSP_REG_ADSPCS_IMR_CACHED_TLB_START 0x100
#define HDA_ADSP_REG_ADSPCS_IMR_UNCACHED_TLB_START 0x200

#define SST_DSP_POWER_D0              0x0  /* full On */
#define SST_DSP_POWER_D3              0x3  /* Off */

/* sst definition */
struct ipc;
struct sst_dsp_ctx;

struct sst_window {
	void __iomem *w0stat;
	void __iomem *w0up;
	void __iomem *w1;
	size_t w0stat_size;
	size_t w0up_size;
	size_t w1_size;
};

struct sst_dsp_loader_ops {
	int (*init) (struct device *dev);
	int (*prepare) (struct device *dev, unsigned int format,
				unsigned int byte_size,
				struct snd_dma_buffer *bufp);
	void (*trigger) (struct device *dev, bool start);

	void (*cleanup) (struct device *dev,
				 struct snd_dma_buffer *dmab);
	int (*alloc_dma_buf) (struct device *dev,
		struct snd_dma_buffer *dmab, u32 size);
	int (*free_dma_buf) (struct device *dev,
		struct snd_dma_buffer *dmab);
};

struct sst_ops {
	int (*load_fw)(struct sst_dsp_ctx  *ctx);
	/* FW module parser/loader */
	int (*parse_fw)(struct sst_dsp_ctx *ctx);
	int (*set_state_D0) (struct sst_dsp_ctx *ctx);
	int (*set_state_D3) (struct sst_dsp_ctx *ctx);
};

struct sst_dsp_ctx {
	struct device *dev;
	struct ipc *ipc;
	void __iomem *mmio_base;
	struct sst_window window;
	spinlock_t reg_lock;
	int irq;
	struct sst_dsp_loader_ops  dsp_ops;
	struct sst_ops ops;
	struct snd_dma_buffer dsp_fw_buf;
};

int sst_load_skl_base_firmware(struct sst_dsp_ctx *ctx);
int sst_load_bxt_base_firmware(struct sst_dsp_ctx *ctx);

void sst_updatel_bits(struct sst_dsp_ctx *ctx, u32 offset, u32 mask, u32 value);

void sst_mailbox_write(struct sst_dsp_ctx *ctx, void *message, size_t bytes);
void sst_mailbox_read(struct sst_dsp_ctx *ctx, void *message, size_t bytes);

int sst_dsp_init(struct sst_dsp_ctx *ctx);
int sst_skl_init(struct device *dev, void __iomem *mmio_base, int irq,
		struct sst_dsp_loader_ops dsp_ops, struct sst_dsp_ctx **dsp);
int sst_bxt_init(struct device *dev, void __iomem *mmio_base, int irq,
		struct sst_dsp_loader_ops dsp_ops, struct sst_dsp_ctx **dsp);
int sst_dsp_free(struct sst_dsp_ctx *dsp);
u8 sst_readb_traced(
	struct sst_dsp_ctx *ctx,
	u32 offset);

u16 sst_readw_traced(
	struct sst_dsp_ctx *ctx,
	u32 offset);

u32 sst_readl_traced(
	struct sst_dsp_ctx *ctx,
	u32 offset);

void sst_writeb_traced(
	struct sst_dsp_ctx *ctx,
	u32 offset,
	u8 val);

void sst_writew_traced(
	struct sst_dsp_ctx *ctx,
	u32 offset, u16 val);

void sst_writel_traced(
	struct sst_dsp_ctx *ctx,
	u32 offset,
	u32 val);

int sst_boot_dsp(struct sst_dsp_ctx *ctx);

int sst_register_poll(struct sst_dsp_ctx  *ctx, u32 offset, u32 mask,
			 u32 expected_value, u32 timeout, char *operation);
int sst_enable_dsp_core(struct sst_dsp_ctx  *ctx);
int sst_disable_dsp_core(struct sst_dsp_ctx  *ctx);
int sst_dsp_set_power_state(struct sst_dsp_ctx *ctx, int state);

#endif /*__HDA_SST_DSP_H__*/
