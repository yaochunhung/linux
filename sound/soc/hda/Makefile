ccflags-y += -DDEBUG
#ccflags-y += -Werror

snd-soc-hda-bus-objs := soc-hda-bus.o

snd-soc-hda-intel-objs := soc-hda-core.o soc-hda-pcm.o soc-hda-dsp.o

snd-mach_hda_generic-objs := mach_hda_generic.o

# DSP library
obj-$(CONFIG_SND_SOC_HDA_SST_DSP) += intel/

# common driver
obj-$(CONFIG_SND_SOC_HDA_BUS) += snd-soc-hda-bus.o
obj-$(CONFIG_SND_SOC_HDA_INTEL) += snd-soc-hda-intel.o

obj-$(CONFIG_SND_SOC_HDA_GENERIC_MACH) += snd-mach_hda_generic.o
