void ext_smb2_read_dt(struct smb_charger *chg);
void ext_smblib_init(struct smb_charger *chg);
void ext_smblib_usb_plugin(struct smb_charger *chg, bool enable);

int ext_smb2_fake_charger_icon(struct smb_charger *chg);
void ext_smb2_init_hw(struct smb_charger *chg);
int ext_smblib_input_current_limit(struct smb_charger *chg,
				struct smb_chg_param *param, int val_u);
void ext_smblib_power_ok(struct smb_charger *chg);
void ext_smblib_usbicl_restart(struct smb_charger *chg);
void ext_smb2_force_disable_hvdcp(struct smb_charger *chg, u8 *val);
