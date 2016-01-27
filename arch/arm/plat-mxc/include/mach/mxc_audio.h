struct mxc_audio_platform_data {
	int ssi_num;
	int src_port;
	int ext_port;

	int intr_id_hp;
	int ext_ram;
	struct clk *ssi_clk[2];

	int hp_irq;
	int (*hp_status) (void);

	int sysclk;

	int (*init) (void);	/* board specific init */
	int (*amp_enable) (int enable);
	void (*finit) (void);	/* board specific finit */
	void *priv;		/* used by board specific functions */
};

