
#ifndef _HDA_BUS_H_
#define _HDA_BUS_H_

/*FIXME move to common header file
#define HDA_RW_NO_RESPONSE_FALLBACK (1 << 0)
*/

struct hda_bus;
struct hda_bus_unsolicited;

/* bus operators */
struct hda_bus_ops {
	/* send a single command */
	int (*command)(struct hda_bus *bus, unsigned int cmd);
	/* get a response from the last command */
	unsigned int (*get_response)(struct hda_bus *bus, unsigned int addr);
	/* free the private data */
	void (*private_free)(struct hda_bus *);
	/* attach a PCM stream */
	int (*attach_pcm)(struct hda_bus *bus, struct hda_codec *codec,
	struct hda_pcm *pcm);
	/* reset bus for retry verb */
	void (*bus_reset)(struct hda_bus *bus);
#ifdef CONFIG_PM
	/* notify power-up/down from codec to controller */
	void (*pm_notify)(struct hda_bus *bus, bool power_up);
#endif
#ifdef CONFIG_SND_HDA_DSP_LOADER
	/* prepare DSP transfer */
	int (*load_dsp_prepare)(struct hda_bus *bus, unsigned int format,
					unsigned int byte_size,
	struct snd_dma_buffer *bufp);
	/* start/stop DSP transfer */
	void (*load_dsp_trigger)(struct hda_bus *bus, bool start);
	/* clean up DSP transfer */
	void (*load_dsp_cleanup)(struct hda_bus *bus,
					struct snd_dma_buffer *dmab);
#endif
};

/* template to pass to the bus constructor */
struct hda_bus_template {
	void *private_data;
	struct pci_dev *pci;
	const char *modelname;
	int *power_save;
	struct hda_bus_ops ops;
};

/*
 * codec bus
 *
 * each controller needs to creata a hda_bus to assign the accessor.
 * A hda_bus contains several codecs in the list codec_list.
 */
struct hda_bus {
	struct snd_card *card;

	/* copied from template */
	void *private_data;
	struct pci_dev *pci;
	const char *modelname;
	int *power_save;
	struct hda_bus_ops ops;

	/* codec linked list */
	struct list_head codec_list;
	unsigned int num_codecs;
	/* link caddr -> codec */
	struct hda_codec *caddr_tbl[HDA_MAX_CODEC_ADDRESS + 1];

	struct mutex cmd_mutex;
	struct mutex prepare_mutex;

	 /* unsolicited event queue */
	 struct hda_bus_unsolicited *unsol;
	 char workq_name[16];
	 struct workqueue_struct *workq; /* common workqueue for codecs */

	 /* assigned PCMs */
	DECLARE_BITMAP(pcm_dev_bits, SNDRV_PCM_DEVICES);

	/* misc op flags */
	unsigned int needs_damn_long_delay:1;
	unsigned int allow_bus_reset:1; /* allow bus reset at fatal error */
	unsigned int sync_write:1;      /* sync after verb write */
	/* status for codec/controller */
	unsigned int shutdown:1;        /* being unloaded */
	unsigned int rirb_error:1;      /* error in codec communication */
	unsigned int response_reset:1;  /* controller was reset */
	unsigned int in_reset:1;        /* during reset operation */
	unsigned int power_keep_link_on:1; /* don't power off HDA link */
	unsigned int no_response_fallback:1; /* don't fallback at RIRB error */

	int primary_dig_out_type;       /* primary digital out PCM type */
};

#endif /* _HDA_BUS_H_ */
