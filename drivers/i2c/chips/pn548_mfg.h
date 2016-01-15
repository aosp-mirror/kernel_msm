#define FTM_NFC_CPLC 0

#define MAX_NFC_DATA_SIZE (600)
#define MAX_FW_BLOCK_SIZE (512)
#define MAX_NFC_RETRY (100)
#define MAX_READ_TIME (5)
#define NFC_READ_DELAY (10)
#define MAX_QUP_DATA_LEN (224)
#define MAX_CMD_LEN 257



typedef struct RF_entry_def {
	unsigned int RF_ID;
	unsigned int RF_Protocol;
	unsigned int RF_Technology;
} RF_Entry;

static struct device_info_def {
	unsigned int padding_exist;
	unsigned char padding;
	unsigned long fwVersion;
	unsigned int HW_model;
	unsigned int NCI_version;
	unsigned long NFCC_Features;
	unsigned char manufactor;

	unsigned char FW_Major;
	unsigned char FW_Minor;

	unsigned int protocol_set;
	unsigned int intf_set;
	unsigned int target_rf_id;
	unsigned int activated_INTF;
	unsigned int NTF_count;
	RF_Entry NTF_queue[15];
} gDevice_info;

typedef struct control_msg_pack_def {
	uint8_t cmd[MAX_CMD_LEN];
	uint8_t exp_resp_content[MAX_CMD_LEN];
	uint8_t exp_ntf[MAX_CMD_LEN];
} control_msg_pack;

static control_msg_pack nfc_version_script[] = {
	{
		.cmd = { 4, 0x20, 0x00, 0x01, 0x01 },
		.exp_resp_content = { 1, 0x00 },
		.exp_ntf = { 0 },
	},
	{
		{ 3, 0x20, 0x01, 0x00 },
		{ 1, 0x00 },
		{ 0 },
	},
};

static control_msg_pack select_rf_target[] = {
	{
		{ 6, 0x21, 0x04, 0x03, 0x01, 0x04, 0x02 },
		{ 1, 0x00 },
		{ 0 },
	},
};

static control_msg_pack nfc_card_script[] ={
	{
		.cmd = { 4, 0x20, 0x00, 0x01, 0x01 },
		.exp_resp_content = { 1, 0x00 },
		.exp_ntf = { 0 },
	},
	{
		{ 3, 0x20, 0x01, 0x00 },
		{ 1, 0x00 },
		{ 0 },
	},
	{
		{ 3, 0x2F, 0x02, 0x00 },
		{ 0 },
		{ 0 },
	},
	{
		{ 8, 0x20, 0x02, 0x05, 0x01, 0xA0, 0x03, 0x01, 0x08 },
		{ 0 },
		{ 0 },
	},
	{
		.cmd = { 4, 0x20, 0x00, 0x01, 0x00 },
		.exp_resp_content = { 1, 0x00 },
		.exp_ntf = { 0 },
	},
	{
		{ 3, 0x20, 0x01, 0x00 },
		{ 1, 0x00 },
		{ 0 },
	},
	{
		{ 3, 0x2F, 0x02, 0x00 },
		{ 0 },
		{ 0 },
	},
	{
		{ 15, 0x21, 0x01, 0x0C, 0x00, 0x02, 0x00, 0x03, 0x00, 0x05, 0x01, 0x01, 0x03, 0x00, 0x01, 0x04 },
		{ 1, 0x00 },
		{ 0 },
	},
	{
		{ 12, 0x21, 0x03, 0x09, 0x04, 0x81, 0x01, 0x82, 0x01, 0x83, 0x01, 0x85, 0x01 },
		{ 1, 0x00 },
		{ 1, 0x01 }
	}
};

static control_msg_pack nfc_standby_enble_script[] = {
	{
		.cmd = { 4, 0x20, 0x00, 0x01, 0x01 },
		.exp_resp_content = { 1, 0x00 },
		.exp_ntf = { 0 },
	},
	{
		{ 3, 0x20, 0x01, 0x00 },
		{ 1, 0x00 },
		{ 0 },
	},
	{
		/*Enable Standby mode*/
		{ 4, 0x2F, 0x00, 0x01, 0x01 },
		{ 1, 0x00 },
		{ 0 },
	},
	{
		/*Set the Internal VEN to VEN = 0 state*/
		{ 8, 0x20, 0x02, 0x05, 0x01, 0xA0, 0x07, 0x01, 0x02 },
		{ 1, 0x00 },
		{ 0 },
	},
};

static control_msg_pack nfc_reader_script[] ={
	{
		.cmd = { 4, 0x20, 0x00, 0x01, 0x01 },
		.exp_resp_content = { 1, 0x00 },
		.exp_ntf = { 0 },
	},
	{
		{ 3, 0x20, 0x01, 0x00 },
		{ 1, 0x00 },
		{ 0 },
	},
	{
		{ 3, 0x2F, 0x02, 0x00 },
		{ 0 },
		{ 0 },
	},
	{
		{ 8, 0x20, 0x02, 0x05, 0x01, 0xA0, 0x03, 0x01, 0x08 },
		{ 2, 0x00, 0x00 },
		{ 0 },
	},
	{   /*NXP_CORE_CONF_EXTN*/
		{ 0x0A,
        0x20, 0x02, 0x07, 0x01, 
        0xA0, 0x0E, 0x03, 0x56, 0x24, 0x0A
        },
		{ 0 },
		{ 0 },
	},
	{
		.cmd = { 4, 0x20, 0x00, 0x01, 0x00 },
		.exp_resp_content = { 1, 0x00 },
		.exp_ntf = { 0 },
	},
	{
		{ 3, 0x20, 0x01, 0x00 },
		{ 1, 0x00 },
		{ 0 },
	},
	{
		{ 3, 0x2F, 0x02, 0x00 },
		{ 0 },
		{ 0 },
	},
	/*{
		{ 13, 0x20, 0x02, 0x0A, 0x01, 0xA0, 0x4E, 0x06, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF },
		{ 2, 0x00, 0x00 },
		{ 0 },
	},*/
	{
		{ 7, 0x21, 0x00, 0x04, 0x01, 0x04, 0x01, 0x02 },
		{ 1, 0x00 },
		{ 0 },
	},
	{
		{ 8, 0x21, 0x03, 0x05, 0x02, 0x00, 0x01, 0x01, 0x01 },
		{ 1, 0x00 },
		{ 1, 0x01 }
	},
};

 
#if FTM_NFC_CPLC
// removed
#endif //FTM_NFC_CPLC

