enum jeita_model {
	MODEL_DISABLE = 0,
	MODEL_COLD,
	MODEL_HOT,
};

void ext_gauge_init(struct fg_chip *chip);
void fg_recharge_mode_detection(struct fg_chip *chip);
void ext_fg_jeita_compensation(struct fg_chip *chip, int temp);
void ext_fg_soc_compensation(struct fg_chip *chip, int *msoc);
bool ext_fg_restart_need(struct fg_chip *chip);
void ext_fg_jeita_compensation_init(struct fg_chip *chip);
void ext_fg_jeita_enable(struct fg_chip *chip);
void ext_fg_read_dt(struct fg_chip *chip);
void ext_fg_init(struct fg_chip *chip);

