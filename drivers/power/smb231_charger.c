
#define pr_fmt(fmt) "SMB231 %s: " fmt, __func__
#include <linux/i2c.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/power_supply.h>
#include <linux/mutex.h>

#include <linux/timer.h>

#define SHOW_INFO


/* SMB231 Registers */
#define INPUT_CURRENT_LIMIT		0x00
#define PRECHG_TERM_CURRENT	0x01
#define VARIOUS_CHG_CTRL		0x02
#define VARIOUS_CHG_CTRL_FASTCHG_CURRENT_BIT	(BIT(2) | BIT(1)  | BIT(0))
#define FASTCHG_CURRENT		0x03
#define OTHER_CHG				0x04
#define HARD_LIMIT_TEMP			0x05
#define CHG_CTRL				0x06
#define VARIOUS_SYSTEM_CHG		0x07
#define LIMIT_CELL_TEMP			0x08
#define INT_OUT_CTRL				0x09
#define I2C_STATUS				0x0A

#define CMD_A_REG				0x30
#define CMD_A_CHG_ENABLE_BIT					BIT(1)
#define CMD_A_VOLATILE_W_BIT					BIT(7)
#define CMD_B_REG				0x31

#define IRQ_A_REG				0x38
#define IRQ_B_REG				0x39
#define IRQ_B_MISSING_BATTERY_STATUS_BIT			BIT(4)
#define IRQ_B_LOW_BATTERY_STATUS_BIT			BIT(2)
#define IRQ_B_PRE_FAST_CHG_STATUS_BIT			BIT(0)
#define IRQ_C_REG				0x3A
#define IRQ_C_TERM_CHG_CURRENT_HIT_BIT			BIT(0)
#define IRQ_D_REG				0x3B

#define STATUS_A_REG			0x3C
#define STATUS_B_REG			0x3D
#define STATUS_B_HOLD_OFF_STATUS_BIT			BIT(3)
#define STATUS_B_CHG_STATUS_BIT					(BIT(2) | BIT(1))
#define STATUS_B_CHG_STATUS_PRE_CHG_BIT			BIT(1)
#define STATUS_B_CHG_STATUS_FAST_CHG_BIT		BIT(2)
#define STATUS_B_CHG_STATUS_TAPER_CHG_BIT		(BIT(2) | BIT(1))
#define STATUS_B_CHG_ENABLE_STATUS_BIT			BIT(0)
#define STATUS_C_REG			0x3E

#define AUTO_INPUT_CURRENT_REG	0x3F
#define MAX_REG_INFO_COUNT	3
struct smb231_charger {
	int int_smb231_i2c_enable;

	struct i2c_client	*client;
	struct device		*dev;
	struct power_supply	*usb_psy;
	struct power_supply	batt_psy;

	/* Timer */
	struct timer_list timer;	
	struct delayed_work timer_work;
	struct workqueue_struct    *workqueue;	 

	/* register info timer */
	struct timer_list register_info_timer;	
	struct delayed_work register_info_timer_work;
	struct workqueue_struct *register_info_workqueue;
};

static struct smb231_charger *g_chip;
static int reg_info_count;
static bool reg_info_timer_add;
static int smb231_fastchg_current_table[] = {
	100,
	250,
	300,
	370,
	500,
	600,
	700,
	1000
};

static int smb231_read_reg(struct smb231_charger *chip, int reg, u8 *val)
{
	s32 ret;

	ret = i2c_smbus_read_byte_data(chip->client, reg);
	if (ret < 0) {
		pr_err("read fail. addr=0x%02x, ret=%d\n", reg, ret);
		return ret;
	} else {
		*val = ret;
	}

	return 0;
}

static int smb231_write_reg(struct smb231_charger *chip, int reg, u8 val)
{
	s32 ret;

	ret = i2c_smbus_write_byte_data(chip->client, reg, val);
	if (ret < 0) {
		pr_err("write fail. addr=0x%02x, val=0x%02x, ret=%d\n", reg, val, ret);
		return ret;
	}

	return 0;
}

static int smb231_masked_write(struct smb231_charger *chip, int reg,
							u8 mask, u8 val)
{
	int rc;
	u8 temp;

	rc = smb231_read_reg(chip, reg, &temp);
	if (rc) {
		pr_err("read fail. addr=0x%02x, rc=%d\n", reg, rc);
		return rc;
	}
	temp &= ~mask;
	temp |= val & mask;
	rc = smb231_write_reg(chip, reg, temp);
	if (rc) {
		pr_err("write fail. addr=0x%02x, mask=0x%02x, val=0x%02x, rc=%d\n", reg, mask, val, rc);
		return rc;
	}

	return 0;
}

static int smb231_enable_volatile_writes(struct smb231_charger *chip)
{
	int rc;

	rc = smb231_masked_write(chip, CMD_A_REG, CMD_A_VOLATILE_W_BIT, CMD_A_VOLATILE_W_BIT);
	if (rc)
		dev_err(chip->dev, "Couldn't write VOLATILE_W_PERM_BIT rc=%d\n", rc);

	return rc;
}

static enum power_supply_property smb231_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_CHARGING_ENABLED,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static int smb231_get_prop_batt_status(struct smb231_charger *chip)
{
	int rc;
	u8 reg = 0;

	if (chip->int_smb231_i2c_enable == 0)
		return POWER_SUPPLY_STATUS_UNKNOWN;

	rc = smb231_read_reg(chip, STATUS_B_REG, &reg);
	if (rc) {
		pr_err("STATUS_B_REG read fail. rc=%d\n", rc);
		return POWER_SUPPLY_STATUS_UNKNOWN;
	}

	if (reg & STATUS_B_HOLD_OFF_STATUS_BIT)
		return POWER_SUPPLY_STATUS_NOT_CHARGING;

	if (reg & STATUS_B_CHG_STATUS_BIT)
		return POWER_SUPPLY_STATUS_CHARGING;

	return POWER_SUPPLY_STATUS_DISCHARGING;
}

static int smb231_get_prop_batt_present(struct smb231_charger *chip)
{
	int rc;
	u8 reg = 0;

	if (chip->int_smb231_i2c_enable == 0)
		return true;

	rc = smb231_read_reg(chip, IRQ_B_REG, &reg);
	if (rc)
		pr_err("STATUS_B_REG read fail. rc=%d\n", rc);

	return  (reg & IRQ_B_MISSING_BATTERY_STATUS_BIT) ? false : true;
}

static int smb231_get_prop_charge_type(struct smb231_charger *chip)
{
	int rc;
	u8 reg;

	if (chip->int_smb231_i2c_enable == 0)
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;

	reg = 0;
	rc = smb231_read_reg(chip, IRQ_C_REG, &reg);
	if (rc)
		pr_err("IRQ_C_REG read fail. rc=%d\n", rc);
	else if ((reg & IRQ_C_TERM_CHG_CURRENT_HIT_BIT) ? true : false)
		return POWER_SUPPLY_STATUS_FULL;

	reg = 0;
	rc = smb231_read_reg(chip, STATUS_B_REG, &reg);
	if (rc) {
		pr_err("STATUS_B_REG read fail. rc=%d\n", rc);
		return POWER_SUPPLY_CHARGE_TYPE_UNKNOWN;
	}

	reg &= STATUS_B_CHG_STATUS_BIT;

	if (reg == STATUS_B_CHG_STATUS_TAPER_CHG_BIT)
		return POWER_SUPPLY_CHARGE_TYPE_TAPER;
	else if (reg == STATUS_B_CHG_STATUS_FAST_CHG_BIT)
		return POWER_SUPPLY_CHARGE_TYPE_FAST;
	else if (reg == STATUS_B_CHG_STATUS_PRE_CHG_BIT)
		return POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
	else
		return POWER_SUPPLY_CHARGE_TYPE_NONE;
}

static int smb231_get_charging_status(struct smb231_charger *chip)
{
	int rc;
	u8 reg = 0;

	if (chip->int_smb231_i2c_enable == 0)
		return 0;

	rc = smb231_read_reg(chip, STATUS_B_REG, &reg);
	if (rc) {
		pr_err("STATUS_B_REG read fail. rc=%d\n", rc);
		return 0;
	}

	return (reg & STATUS_B_CHG_ENABLE_STATUS_BIT) ? 1 : 0;
}

static int smb231_charging(struct smb231_charger *chip, int enable)
{
	int rc = 0;
	u8 reg = 0;

	rc = smb231_read_reg(chip, OTHER_CHG, &reg);
	if (rc)
		return rc;
	reg &= BIT(5);

	if (enable) {
		rc = smb231_masked_write(chip, CMD_A_REG, CMD_A_CHG_ENABLE_BIT, reg ? 0 : CMD_A_CHG_ENABLE_BIT);
		if (rc)
			pr_err("enable fail, enable polarity=0x%02x, rc=%d\n", reg, rc);
	} else {
		rc = smb231_masked_write(chip, CMD_A_REG, CMD_A_CHG_ENABLE_BIT, reg ? CMD_A_CHG_ENABLE_BIT : 0);
		if (rc)
			pr_err("disable fail, enable polarity=0x%02x, rc=%d\n", reg, rc);
	}
		
	return rc;
}

static int smb231_get_fastchg_current(struct smb231_charger *chip)
{
	u8 reg;
	int rc;

	rc = smb231_read_reg(chip, VARIOUS_CHG_CTRL, &reg);
	if (rc)
		return rc;

	reg &= VARIOUS_CHG_CTRL_FASTCHG_CURRENT_BIT;

	return smb231_fastchg_current_table[reg];
}

static int smb231_set_fastchg_current(struct smb231_charger *chip, int current_ma)
{
	u8 reg;
	int i, rc, level;
	int fastchg_current_arr_size = ARRAY_SIZE(smb231_fastchg_current_table);

	level = fastchg_current_arr_size - 1;

	for (i = 0; i < fastchg_current_arr_size; i++) {
		if (current_ma <= smb231_fastchg_current_table[i]) {
			level = i;
			break;
		}
	}

	reg = level & VARIOUS_CHG_CTRL_FASTCHG_CURRENT_BIT;
	rc = smb231_masked_write(chip, VARIOUS_CHG_CTRL, VARIOUS_CHG_CTRL_FASTCHG_CURRENT_BIT, reg);
	if (rc)
		pr_err("set current fail, rc=%d\n", rc);

	return rc;
}

int smb231_reg_info(struct smb231_charger *chip)
{
#ifdef SHOW_INFO
	int rc;
	u8 reg, addr = 0;

	pr_err("=====  smb231_reg_info  ===== \n");
	for(addr = INPUT_CURRENT_LIMIT ; addr <= I2C_STATUS ; addr++) {
		reg = 0;
		rc = smb231_read_reg(chip, addr, &reg);
		if (rc) {
			pr_err("Read fail. addr=0x%02x\n", addr);
			return rc;
		} else {
			pr_err("Reg 0x%02x=0x%02x\n", addr, reg);
		}
	}

	for(addr = CMD_A_REG ; addr <= CMD_B_REG ; addr++) {
		reg = 0;
		rc = smb231_read_reg(chip, addr, &reg);
		if (rc) {
			pr_err("Read fail. addr=0x%02x\n", addr);
			return rc;
		} else {
			pr_err("Reg 0x%02x=0x%02x\n", addr, reg);
		}
	}

	for(addr = IRQ_A_REG ; addr <= AUTO_INPUT_CURRENT_REG ; addr++) {
		reg = 0;
		rc = smb231_read_reg(chip, addr, &reg);
		if (rc) {
			pr_err("Read fail. addr=0x%02x\n", addr);
			return rc;
		} else {
			pr_err("Reg 0x%02x=0x%02x\n", addr, reg);
		}
	}
#endif
	return 0;
}

static int smb231_set_usb_chg_current(struct smb231_charger *chip)
{
	int rc;

	//AC input limit = 500 mA
	rc = smb231_masked_write(chip, INPUT_CURRENT_LIMIT, (BIT(4) | BIT(3) | BIT(2)), BIT(3));
	if (rc) 
		return rc;

	//Pre-charge current = 20 mA
	rc = smb231_masked_write(chip, PRECHG_TERM_CURRENT, (BIT(7) | BIT(6)), 0);
	if (rc) 
		return rc;

	//Float voltage (V) = 4.36 V
	rc = smb231_masked_write(chip, FASTCHG_CURRENT, (BIT(4) | BIT(3) | BIT(2) | BIT(1) | BIT(0)), (BIT(4) | BIT(2) | BIT(1)));
	if (rc) 
		return rc;

	//Charger enable polarity = Active high
	rc = smb231_masked_write(chip, OTHER_CHG, BIT(5), 0);
	if (rc) 
		return rc;

	//System voltage = VBATT tracking (VBATT + 250mV)
	rc = smb231_masked_write(chip, OTHER_CHG, (BIT(1) | BIT(0)), BIT(1));
	if (rc) 
		return rc;

	//Soft temp limit behavior = Charge current and float voltage compensation
	rc = smb231_masked_write(chip, HARD_LIMIT_TEMP, (BIT(5) | BIT(4)), (BIT(5) | BIT(4)));
	if (rc) 
		return rc;

	//Float voltage compensation = VFLT - 160mV
	rc = smb231_masked_write(chip, HARD_LIMIT_TEMP, (BIT(3) | BIT(2)), BIT(3));
	if (rc) 
		return rc;

	//System voltage threshold for initiating charge current reduction = VBATT tracking (100mV)
	rc = smb231_masked_write(chip, CHG_CTRL, (BIT(5) | BIT(4)), BIT(5));
	if (rc) 
		return rc;
//==== 9K  ========
/*
	//R1 = 9 K, R2 = 0 K, RT = 10 K, Beta = 3435
	//Hard limit cold temperature alarm trip point = 0 C
	rc = smb231_masked_write(chip, LIMIT_CELL_TEMP, (BIT(7) | BIT(6)), BIT(7));
	if (rc) 
		return rc;

	//Hard limit hot temperature alarm trip point = 61 C
	rc = smb231_masked_write(chip, LIMIT_CELL_TEMP, (BIT(5) | BIT(4)), BIT(4));
	if (rc) 
		return rc;

	//Soft limit cold temperature alarm trip point = 11 C
	rc = smb231_masked_write(chip, LIMIT_CELL_TEMP, (BIT(3) | BIT(2)), BIT(2));
	if (rc) 
		return rc;

	//Soft limit hot temperature alarm trip point = 45 C
	rc = smb231_masked_write(chip, LIMIT_CELL_TEMP, (BIT(1) | BIT(0)), 0);
	if (rc) 
		return rc;
*/

	//==== 10K  ========
	//Hard limit cold temperature alarm trip point = 0 C
	rc = smb231_masked_write(chip, LIMIT_CELL_TEMP, (BIT(7) | BIT(6)), BIT(7));
	if (rc) 
		return rc;

	//Hard limit hot temperature alarm trip point = 60 C
	rc = smb231_masked_write(chip, LIMIT_CELL_TEMP, (BIT(5) | BIT(4)), BIT(5));
	if (rc) 
		return rc;

	//Soft limit cold temperature alarm trip point = 10 C
	rc = smb231_masked_write(chip, LIMIT_CELL_TEMP, (BIT(3) | BIT(2)), BIT(2));
	if (rc) 
		return rc;

	//Soft limit hot temperature alarm trip point = 45 C
	rc = smb231_masked_write(chip, LIMIT_CELL_TEMP, (BIT(1) | BIT(0)), BIT(0));
	if (rc) 
		return rc;


	//Battery charge enable = Enable
	rc = smb231_masked_write(chip, CMD_A_REG, BIT(1), BIT(1));
	if (rc) 
		return rc;

	return 0;
}

static int smb231_batt_property_is_writeable(struct power_supply *psy, enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		return 1;
	default:
		break;
	}

	return 0;
}

static int smb231_battery_set_property(struct power_supply *psy, enum power_supply_property prop, const union power_supply_propval *val)
{
	int rc = 0;
	struct smb231_charger *chip = container_of(psy, struct smb231_charger, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		rc = smb231_charging(chip, val->intval);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		rc = smb231_set_fastchg_current(chip, val->intval / 1000);
		break;
	default:
		return -EINVAL;
	}

	return rc;
}

static int smb231_battery_get_property(struct power_supply *psy, enum power_supply_property prop, union power_supply_propval *val)
{
	struct smb231_charger *chip = container_of(psy, struct smb231_charger, batt_psy);

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = smb231_get_prop_batt_status(chip);
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = smb231_get_prop_batt_present(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGING_ENABLED:
		val->intval = smb231_get_charging_status(chip);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		val->intval = smb231_get_prop_charge_type(chip);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		val->intval = smb231_get_fastchg_current(chip);
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = "SMB231";
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static void smb231_external_power_changed(struct power_supply *psy)
{
	int rc, charger_exist = 0;
	struct smb231_charger *chip = container_of(psy, struct smb231_charger, batt_psy);

	rc = smb231_enable_volatile_writes(chip);
	if (rc)
		charger_exist = 0;
	else
		charger_exist = 1;

	//To avoid this function is called multiple times in 1 seconds
	if((chip->int_smb231_i2c_enable) ^ charger_exist)
		chip->int_smb231_i2c_enable = charger_exist;
	else	
		return;

	if(!charger_exist) {
		if(reg_info_timer_add) {
			del_timer(&g_chip->register_info_timer);
			reg_info_timer_add = false;
		}
		return;
	}

	rc = smb231_set_usb_chg_current(chip);
	if (rc) {
		chip->int_smb231_i2c_enable = 0;
		return;
	}

	reg_info_count = 0;
	reg_info_timer_add = true;
	add_timer(&g_chip->register_info_timer); 

	power_supply_changed(&chip->batt_psy);
}

void smb231_timer_read_register(unsigned long dev)
{
	queue_delayed_work(g_chip->workqueue, &g_chip->timer_work, 0);
}

void smb231_timer_work_set_register(struct work_struct *work)
{
	int rc;

	rc = smb231_enable_volatile_writes(g_chip);
	if (rc)
		return;

	rc = smb231_set_usb_chg_current(g_chip);
	if (rc)
		return;

	g_chip->int_smb231_i2c_enable = 1;

	power_supply_changed(&g_chip->batt_psy);
}


void smb231_register_info_timer_workqueue(unsigned long dev)
{
	queue_delayed_work(g_chip->register_info_workqueue, &g_chip->register_info_timer_work, 0);
}

void smb231_register_info_timer_work(struct work_struct *work)
{
	if ((g_chip->int_smb231_i2c_enable == 1) && (reg_info_count < MAX_REG_INFO_COUNT)) {
		reg_info_count++;
		smb231_reg_info(g_chip);

		g_chip->register_info_timer.expires = jiffies + 10*HZ; 	/*10 sec */
		reg_info_timer_add = true;
		add_timer(&g_chip->register_info_timer); 
	}
	else
		reg_info_timer_add = false;
}

static int smb231_charger_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	int rc;
	struct smb231_charger *chip;
	struct power_supply *usb_psy;
	u8 reg = 0;

	usb_psy = power_supply_get_by_name("usb");
	if (!usb_psy) {
		dev_dbg(&client->dev, "USB psy not found; deferring probe\n");
		return -EPROBE_DEFER;
	}

	chip = devm_kzalloc(&client->dev, sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&client->dev, "Couldn't allocate memory\n");
		return -ENOMEM;
	}

	pr_err("Enter !!!\n");

	chip->client = client;
	chip->dev = &client->dev;
	chip->usb_psy = usb_psy;

	i2c_set_clientdata(client, chip);

	chip->batt_psy.name		= "charger";
	chip->batt_psy.type		= POWER_SUPPLY_TYPE_BATTERY;
	chip->batt_psy.get_property	= smb231_battery_get_property;
	chip->batt_psy.set_property	= smb231_battery_set_property;
	chip->batt_psy.property_is_writeable		= smb231_batt_property_is_writeable;
	chip->batt_psy.properties				= smb231_battery_properties;
	chip->batt_psy.num_properties			= ARRAY_SIZE(smb231_battery_properties);
	chip->batt_psy.external_power_changed	= smb231_external_power_changed;

	rc = power_supply_register(chip->dev, &chip->batt_psy);
	if (rc < 0) {
		dev_err(&client->dev, "Couldn't register batt psy rc=%d\n", rc);
		return rc;
	}

	//Init timier work
	g_chip = chip;
	INIT_DELAYED_WORK(&g_chip->timer_work, smb231_timer_work_set_register);
	g_chip->workqueue = create_singlethread_workqueue("smb231_work");
	if (g_chip->workqueue == NULL)
		pr_err("failed to create work queue\n");

	//init timer
	init_timer(&g_chip->timer);
	g_chip->timer.function = smb231_timer_read_register;
	g_chip->timer.expires = jiffies + 10*HZ;
	add_timer(&g_chip->timer); 

	//Init register info timer work
	reg_info_count = 0;
	INIT_DELAYED_WORK(&chip->register_info_timer_work, smb231_register_info_timer_work);
       chip->register_info_workqueue = create_singlethread_workqueue("smb231_info_work");
       if (chip->register_info_workqueue == NULL)
               pr_err("failed to create register info work queue\n");
	else {
		init_timer(&chip->register_info_timer);
		chip->register_info_timer.function = smb231_register_info_timer_workqueue;
		chip->register_info_timer.expires = jiffies + 5*HZ; 	/*5 sec */
	}
	
	//Init i2c status
	rc = smb231_read_reg(chip, STATUS_B_REG, &reg);
	if (rc)
		chip->int_smb231_i2c_enable = 0; //read fail
	else
		chip->int_smb231_i2c_enable = 1;

	return 0;
}

static int smb231_charger_remove(struct i2c_client *client)
{
	struct smb231_charger *chip = i2c_get_clientdata(client);

	power_supply_unregister(&chip->batt_psy);

	return 0;
}

static int smb231_suspend(struct device *dev)
{
	pr_err("Resume\n");

	return 0;
}

static int smb231_resume(struct device *dev)
{
	pr_err("Suspend\n");

	return 0;
}

static const struct dev_pm_ops smb231_pm_ops = {
	.suspend	= smb231_suspend,
	.resume	= smb231_resume,
};

static struct of_device_id smb231_match_table[] = {
	{ .compatible = "qcom,smb231-charger",},
	{ },
};

static const struct i2c_device_id smb231_charger_id[] = {
	{"smb231-charger", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, smb231_charger_id);

static struct i2c_driver smb231_charger_driver = {
	.driver		= {
		.name			= "smb231-charger",
		.owner			= THIS_MODULE,
		.of_match_table	= smb231_match_table,
		.pm				= &smb231_pm_ops,
	},
	.probe		= smb231_charger_probe,
	.remove		= smb231_charger_remove,
	.id_table		= smb231_charger_id,
};

module_i2c_driver(smb231_charger_driver);

MODULE_AUTHOR("SMB231-CEI");
MODULE_DESCRIPTION("SMB231 charger CEI");
MODULE_LICENSE("GPL");
