ccflags-y += -DDEBUG
#ccflags-y += -Werror

snd-soc-hda-bus-objs := soc-hda-bus.o

snd-soc-hda-intel-objs := intel-ssp-config.o soc-hda-core.o soc-hda-pcm.o soc-hda-sst-pipe.o soc-hda-controls.o soc-hda-dsp.o

snd-mach_hda_generic-objs := mach_hda_generic.o

snd-mrgfld_dpcm_florida-objs := mrgfld_dpcm_florida.o

snd-skl_dpcm_realtek-objs := skl_dpcm_realtek.o

# DSP library
obj-$(CONFIG_SND_SOC_HDA_SST_DSP) += intel/

# common driver
obj-$(CONFIG_SND_SOC_HDA_BUS) += snd-soc-hda-bus.o
obj-$(CONFIG_SND_SOC_HDA_INTEL) += snd-soc-hda-intel.o

obj-$(CONFIG_SND_SOC_HDA_GENERIC_MACH) += snd-mach_hda_generic.o

obj-$(CONFIG_SND_SOC_HDA_MRGFLD_MACH) += snd-mrgfld_dpcm_florida.o

obj-$(CONFIG_SND_SOC_HDA_SKL_MACH) += snd-skl_dpcm_realtek.o

# for intel skylake fpga device
#snd-hda-intel-skl-fpga-objs := hda_intel_skl_fpga.o
#obj-$(CONFIG_SND_HDA_INTEL_SKL_FPGA) += snd-hda-intel-skl-fpga.o

snd-hda-intel-bxt-fpga-objs := hda_intel_bxt_fpga.o
obj-$(CONFIG_SND_HDA_INTEL_SKL_FPGA) += snd-hda-intel-bxt-fpga.o

snd-hda-intel-csme-fpga-objs := hda_intel_csme_fpga.o
obj-$(CONFIG_SND_HDA_INTEL_SKL_FPGA) += snd-hda-intel-csme-fpga.o
