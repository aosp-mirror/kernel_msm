/******************************************************************************
 *
 *  This is the implementation file for the PN548 NFC customization Functions
 *
 ******************************************************************************/

#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/types.h>
#include "pn548_htc.h"

#if NFC_GET_BOOTMODE
#include <linux/htc_flags.h>
#endif //NFC_GET_BOOTMODE

#if NFC_READ_RFSKUID
#define HAS_NFC_CHIP 0x7000000
#endif //NFC_READ_RFSKUID

#define D(x...)	\
	if (is_debug) \
		printk(KERN_DEBUG "[NFC] " x)
#define I(x...) printk(KERN_INFO "[NFC] " x)
#define E(x...) printk(KERN_ERR "[NFC] [Err] " x)


// for off mode charging ++
#if NFC_OFF_MODE_CHARGING_LOAD_SWITCH
static unsigned int   pvdd_gpio;
#endif //NFC_OFF_MODE_CHARGING_LOAD_SWITCH
// for off mode charging --


/******************************************************************************
 *
 *  Function pn548_htc_check_rfskuid:
 *  Return With(1)/Without(0) NFC chip if this SKU can get RFSKUID in kernal
 *  Return is_alive(original value) by default.
 *
 ******************************************************************************/
int pn548_htc_check_rfskuid(int in_is_alive){
#if NFC_READ_RFSKUID
	int nfc_rfbandid_size = 0;
	int i;
	unsigned int *nfc_rfbandid_info;
	struct device_node *nfc_rfbandid;
	nfc_rfbandid = of_find_node_by_path("/chosen/mfg");
	if (nfc_rfbandid){
		nfc_rfbandid_info = (unsigned int *) of_get_property(nfc_rfbandid,"skuid.rf_id",&nfc_rfbandid_size);
		if (nfc_rfbandid_info == NULL)
		{
			E("%s:Get skuid.rf_id fail keep NFC by default",__func__);
			return 1;
		}
	}else {
		E("%s:Get skuid.rf_id fail (can't find /chosen/mfg) keep NFC by default",__func__);
		return 1;
	}
	if(nfc_rfbandid_size != 32) {  //32bytes = 4 bytes(int) * 8 rfbandid_info
		E("%s:Get skuid.rf_id size error keep NFC by default",__func__);
		return 1;
	}

	for ( i = 0; i < 8; i++) {
		if (nfc_rfbandid_info[i] == HAS_NFC_CHIP) {
			I("%s: Check skuid.rf_id done device has NFC chip",__func__);
			return 1;
		}
	}
	I("%s: Check skuid.rf_id done remove NFC",__func__);
	return 0;
#else //NFC_READ_RFSKUID
	return in_is_alive;
#endif //NFC_READ_RFSKUID
}


/******************************************************************************
 *
 *  Function pn548_htc_get_bootmode:
 *  Return  NFC_BOOT_MODE_NORMAL            0
 *          NFC_BOOT_MODE_FTM               1
 *          NFC_BOOT_MODE_DOWNLOAD          2
 *          NFC_BOOT_MODE_OFF_MODE_CHARGING 5
 *  Return 	NFC_BOOT_MODE_NORMAL by default
 *          if there's no bootmode infomation available
 ******************************************************************************/
int pn548_htc_get_bootmode(void) {
	char sbootmode[30] = "default";
#if NFC_GET_BOOTMODE
	strlcpy(sbootmode,htc_get_bootmode(),sizeof(sbootmode));
#endif  //NFC_GET_BOOTMODE
	if (strcmp(sbootmode, "offmode_charging") == 0) {
		I("%s: Check bootmode done NFC_BOOT_MODE_OFF_MODE_CHARGING\n",__func__);
		return NFC_BOOT_MODE_OFF_MODE_CHARGING;
	} else if (strcmp(sbootmode, "ftm") == 0) {
		I("%s: Check bootmode done NFC_BOOT_MODE_FTM\n",__func__);
		return NFC_BOOT_MODE_FTM;
	} else if (strcmp(sbootmode, "download") == 0) {
		I("%s: Check bootmode done NFC_BOOT_MODE_DOWNLOAD\n",__func__);
		return NFC_BOOT_MODE_DOWNLOAD;
	} else {
		I("%s: Check bootmode done NFC_BOOT_MODE_NORMAL mode = %s\n",__func__,sbootmode);
		return NFC_BOOT_MODE_NORMAL;
	}
}


/******************************************************************************
 *
 *  Function pn548_htc_get_bootmode:
 *  Get platform required GPIO number from device tree
 *  For Power off sequence and OFF_MODE_CHARGING
 *
 ******************************************************************************/
void pn548_htc_parse_dt(struct device *dev) {
#if NFC_OFF_MODE_CHARGING_LOAD_SWITCH
	struct device_node *dt = dev->of_node;
	pvdd_gpio = of_get_named_gpio_flags(dt, "nxp,pvdd-gpio",0, NULL);
	I("%s: pvdd_gpio:%d\n", __func__, pvdd_gpio);
#endif
}

/******************************************************************************
 *
 *  Function pn548_htc_off_mode_charging
 *  Turn of NFC_PVDD when bootmode = NFC_BOOT_MODE_OFF_MODE_CHARGING
 *
 ******************************************************************************/
void pn548_htc_off_mode_charging (void) {
#if NFC_OFF_MODE_CHARGING_LOAD_SWITCH
	I("%s: Turn off NFC_PVDD \n", __func__);
	gpio_set_value(pvdd_gpio, 0);
#endif
}

