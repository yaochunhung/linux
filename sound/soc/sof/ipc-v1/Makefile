# SPDX-License-Identifier: (GPL-2.0 OR BSD-3-Clause)

ccflags-y += -DDEBUG

snd-sof-ipc-v1-objs := topology.o dai.o stream.o pm.o pcm.o trace.o\
		       control.o debug.o ipc.o
obj-$(CONFIG_SND_SOC_SOF_IPC_LEGACY_SUPPORT_V1) = snd-sof-ipc-v1.o
