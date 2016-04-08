/*
 * cyttsp5_devtree.c
 * Cypress TrueTouch(TM) Standard Product V5 Device Tree Support Module.
 * For use with Cypress Txx5xx parts.
 * Supported parts include:
 * TMA5XX
 *
 * Copyright (C) 2013-2014 Cypress Semiconductor
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Contact Cypress Semiconductor at www.cypress.com <ttdrivers@cypress.com>
 *
 */

#include <linux/device.h>
#include <linux/err.h>
#include <linux/of_device.h>
#include <linux/slab.h>
#include <linux/of_gpio.h>

/* cyttsp */
#include "cyttsp5_regs.h"
#include "cyttsp5_platform.h"

//#define ENABLE_VIRTUAL_KEYS

#define MAX_NAME_LENGTH		64

enum cyttsp5_device_type {
	DEVICE_MT,
	DEVICE_BTN,
	DEVICE_PROXIMITY,
	DEVICE_TYPE_MAX,
};

struct cyttsp5_device_pdata_func {
	void * (*create_and_get_pdata)(struct device_node *);
	void (*free_pdata)(void *);
};

struct cyttsp5_pdata_ptr {
	void **pdata;
};

#ifdef ENABLE_VIRTUAL_KEYS
static struct kobject *board_properties_kobj;

struct cyttsp5_virtual_keys {
	struct kobj_attribute kobj_attr;
	u16 *data;
	int size;
};
#endif

struct cyttsp5_extended_mt_platform_data {
	struct cyttsp5_mt_platform_data pdata;
#ifdef ENABLE_VIRTUAL_KEYS
	struct cyttsp5_virtual_keys vkeys;
#endif
};

static inline int get_inp_dev_name(struct device_node *dev_node,
		const char **inp_dev_name)
{
	return of_property_read_string(dev_node, "cy,inp_dev_name",
			inp_dev_name);
}

static s16 *create_and_get_u16_array(struct device_node *dev_node,
		const char *name, int *size)
{
	const __be32 *values;
	s16 *val_array;
	int len;
	int sz;
	int rc;
	int i;

	values = of_get_property(dev_node, name, &len);
	if (values == NULL)
		return NULL;

	sz = len / sizeof(u32);
	tp_log_debug("%s: %s size:%d\n", __func__, name, sz);

	val_array = kcalloc(sz, sizeof(s16), GFP_KERNEL);
	if (!val_array) {
		rc = -ENOMEM;
		goto fail;
	}

	for (i = 0; i < sz; i++)
		val_array[i] = (s16)be32_to_cpup(values++);

	*size = sz;

	return val_array;

fail:
	return ERR_PTR(rc);
}

static struct touch_framework *create_and_get_touch_framework(
		struct device_node *dev_node)
{
	struct touch_framework *frmwrk = NULL;
	s16 *abs = NULL;
	int size = 0;
	int rc= 0;

	abs = create_and_get_u16_array(dev_node, "cy,abs", &size);
	if (IS_ERR_OR_NULL(abs)) {
		tp_log_err("%s %d:[cy,abs] array get fail.\n", __func__, __LINE__);
		return (void *)abs;
	}

	/* Check for valid abs size */
	if (size % CY_NUM_ABS_SET) {
		tp_log_err("%s %d:Check abs size fail, size = %d, num = %d.\n",
					__func__, __LINE__, size, CY_NUM_ABS_SET);
		rc = -EINVAL;
		goto fail_free_abs;
	}

	frmwrk = kzalloc(sizeof(*frmwrk), GFP_KERNEL);
	if (!frmwrk) {
		tp_log_err("%s %d:kzalloc fail.\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto fail_free_abs;
	}

	frmwrk->abs = abs;
	frmwrk->size = size;

	return frmwrk;

fail_free_abs:
	kfree(abs);

	return ERR_PTR(rc);
}

static void free_touch_framework(struct touch_framework *frmwrk)
{
	kfree(frmwrk->abs);
	kfree(frmwrk);
}

#ifdef ENABLE_VIRTUAL_KEYS
#define VIRTUAL_KEY_ELEMENT_SIZE	5
static ssize_t virtual_keys_show(struct kobject *kobj,
		struct kobj_attribute *attr, char *buf)
{
	struct cyttsp5_virtual_keys *vkeys = container_of(attr,
		struct cyttsp5_virtual_keys, kobj_attr);
	u16 *data = vkeys->data;
	int size = vkeys->size;
	int index;
	int i;

	index = 0;
	for (i = 0; i < size; i += VIRTUAL_KEY_ELEMENT_SIZE)
		index += scnprintf(buf + index, CY_MAX_PRBUF_SIZE - index,
			"0x01:%d:%d:%d:%d:%d\n",
			data[i], data[i+1], data[i+2], data[i+3], data[i+4]);

	return index;
}

static int setup_virtual_keys(struct device_node *dev_node,
		const char *inp_dev_name, struct cyttsp5_virtual_keys *vkeys)
{
	char *name;
	u16 *data;
	int size;
	int rc;

	data = create_and_get_u16_array(dev_node, "cy,virtual_keys", &size);
	if (data == NULL)
		return 0;
	else if (IS_ERR(data)) {
		rc = PTR_ERR(data);
		goto fail;
	}

	/* Check for valid virtual keys size */
	if (size % VIRTUAL_KEY_ELEMENT_SIZE) {
		rc = -EINVAL;
		goto fail_free_data;
	}

	name = kzalloc(MAX_NAME_LENGTH, GFP_KERNEL);
	if (!name) {
		rc = -ENOMEM;
		goto fail_free_data;
	}

	snprintf(name, MAX_NAME_LENGTH, "virtualkeys.%s", inp_dev_name);

	vkeys->data = data;
	vkeys->size = size;

	/* TODO: Instantiate in board file and export it */
	if (board_properties_kobj == NULL)
		board_properties_kobj =	tp_get_virtual_key_obj("board_properties");
	if (board_properties_kobj == NULL) {
		tp_log_err("%s: Cannot get board_properties kobject!\n", __func__);
		rc = -EINVAL;
		goto fail_free_name;
	}

	/* Initialize dynamic SysFs attribute */
	sysfs_attr_init(&vkeys->kobj_attr.attr);
	vkeys->kobj_attr.attr.name = name;
	vkeys->kobj_attr.attr.mode = S_IRUGO;
	vkeys->kobj_attr.show = virtual_keys_show;

	rc = sysfs_create_file(board_properties_kobj, &vkeys->kobj_attr.attr);
	if (rc)
		goto fail_del_kobj;

	return 0;

fail_del_kobj:
	kobject_del(board_properties_kobj);
fail_free_name:
	kfree(name);
	vkeys->kobj_attr.attr.name = NULL;
fail_free_data:
	kfree(data);
	vkeys->data = NULL;
fail:
	return rc;
}

static void free_virtual_keys(struct cyttsp5_virtual_keys *vkeys)
{
	if (board_properties_kobj)
		sysfs_remove_file(board_properties_kobj,
			&vkeys->kobj_attr.attr);


	kobject_del(board_properties_kobj);
	board_properties_kobj = NULL;

	kfree(vkeys->data);
	kfree(vkeys->kobj_attr.attr.name);
}
#endif

static void *create_and_get_mt_pdata(struct device_node *dev_node)
{
	struct cyttsp5_extended_mt_platform_data *ext_pdata = NULL;
	struct cyttsp5_mt_platform_data *pdata= NULL;
	u32 value = 0;
	int rc = 0;

	ext_pdata = kzalloc(sizeof(*ext_pdata), GFP_KERNEL);
	if (!ext_pdata) {
		tp_log_err("%s %d:kzalloc fail\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto fail;
	}

	pdata = &ext_pdata->pdata;
	rc = get_inp_dev_name(dev_node, &pdata->inp_dev_name);
	if (rc) {
		tp_log_err("%s %d:get_inp_dev_name fail, rc = %d\n",
					__func__, __LINE__, rc);
		goto fail_free_pdata;
	}

	/* Optional fields */
	rc = of_property_read_u32(dev_node, "cy,flags", &value);
	if (!rc) {
		pdata->flags = value;
		tp_log_info("%s %d:pdata->flags is %d\n", __func__, __LINE__, value);
	} else {
		tp_log_err("%s %d:[cy,flags] read fail, rc = %d\n",
					__func__, __LINE__, rc);
	}

	rc = of_property_read_u32(dev_node, "cy,vkeys_x", &value);
	if (!rc) {
		pdata->vkeys_x = value;
		tp_log_info("%s %d:pdata->vkeys_x is %d\n", __func__, __LINE__, value);
	} else {
		tp_log_err("%s %d:[cy,vkeys_x] read fail, rc = %d\n",
					__func__, __LINE__, rc);
	}

	rc = of_property_read_u32(dev_node, "cy,vkeys_y", &value);
	if (!rc) {
		pdata->vkeys_y = value;
		tp_log_info("%s %d:pdata->vkeys_y is %d\n", __func__, __LINE__, value);
	} else {
		tp_log_err("%s %d:[cy,vkeys_y] read fail, rc = %d\n",
					__func__, __LINE__, rc);
	}

	rc = of_property_read_u32(dev_node, "cy,x_max", &value);
	if (!rc) {
		pdata->x_max = value;
		tp_log_info("%s %d:pdata->x_max is %d\n", __func__, __LINE__, value);
	} else {
		tp_log_err("%s %d:[cy,x_max] not found,will ignore, rc = %d\n",
					__func__, __LINE__, rc);
	}

	rc = of_property_read_u32(dev_node, "cy,y_max", &value);
	if (!rc) {
		pdata->y_max = value;
		tp_log_info("%s %d:pdata->y_max is %d\n", __func__, __LINE__, value);
	} else {
		tp_log_err("%s %d:[cy,y_max] not found,will ignore, rc = %d\n",
					__func__, __LINE__, rc);
	}

	rc = of_property_read_u32(dev_node, "cy,p_max", &value);
	if (!rc) {
		pdata->p_max = value;
		tp_log_info("%s %d:pdata->p_max is %d\n", __func__, __LINE__, value);
	} else {
		tp_log_err("%s %d:[cy,p_max] not found,will ignore, rc = %d\n",
					__func__, __LINE__, rc);
	}

	rc = of_property_read_u32(dev_node, "cy,slots_max", &value);
	if (!rc) {
		pdata->slots_max = value;
		tp_log_info("%s %d:pdata->slots_max is %d\n", __func__, __LINE__, value);
	} else {
		tp_log_err("%s %d:[cy,slots_max] not found,will ignore, rc = %d\n",
					__func__, __LINE__, rc);
	}

	/* Required fields */
	pdata->frmwrk = create_and_get_touch_framework(dev_node);
	if (pdata->frmwrk == NULL) {
		tp_log_err("%s %d:pdata->frmwrk is NULL\n", __func__, __LINE__);
		rc = -EINVAL;
		goto fail_free_pdata;
	} else if (IS_ERR(pdata->frmwrk)) {
		rc = PTR_ERR(pdata->frmwrk);
		tp_log_err("%s %d:pdata->frmwrk is error, rc = %d\n", __func__, __LINE__, rc);
		goto fail_free_pdata;
	}
#ifdef ENABLE_VIRTUAL_KEYS
	rc = setup_virtual_keys(dev_node, pdata->inp_dev_name,
			&ext_pdata->vkeys);
	if (rc) {
		tp_log_err("%s: Cannot setup virtual keys!\n", __func__);
		goto fail_free_pdata;
	}
#endif
	return pdata;

fail_free_pdata:
	kfree(ext_pdata);
fail:
	return ERR_PTR(rc);
}

static void free_mt_pdata(void *pdata)
{
	struct cyttsp5_mt_platform_data *mt_pdata =
		(struct cyttsp5_mt_platform_data *)pdata;
	struct cyttsp5_extended_mt_platform_data *ext_mt_pdata =
		container_of(mt_pdata,
			struct cyttsp5_extended_mt_platform_data, pdata);

	free_touch_framework(mt_pdata->frmwrk);
#ifdef ENABLE_VIRTUAL_KEYS
	free_virtual_keys(&ext_mt_pdata->vkeys);
#endif
	kfree(ext_mt_pdata);
}

static void *create_and_get_btn_pdata(struct device_node *dev_node)
{
	struct cyttsp5_btn_platform_data *pdata = NULL;
	int rc = 0;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		tp_log_err("%s %d:kzalloc\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto fail;
	}

	rc = get_inp_dev_name(dev_node, &pdata->inp_dev_name);
	if (rc) {
		tp_log_err("%s %d:get_inp_dev_name fail, \n", __func__, __LINE__);
		goto fail_free_pdata;
	}

	return pdata;

fail_free_pdata:
	kfree(pdata);
fail:
	return ERR_PTR(rc);
}

static void free_btn_pdata(void *pdata)
{
	struct cyttsp5_btn_platform_data *btn_pdata =
		(struct cyttsp5_btn_platform_data *)pdata;

	kfree(btn_pdata);
}

static void *create_and_get_proximity_pdata(struct device_node *dev_node)
{
	struct cyttsp5_proximity_platform_data *pdata = NULL;
	int rc = 0;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		tp_log_err("%s %d:kzalloc fail.\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto fail;
	}

	rc = get_inp_dev_name(dev_node, &pdata->inp_dev_name);
	if (rc) {
		tp_log_err("%s %d:get_inp_dev_name fail.\n", __func__, __LINE__);
		goto fail_free_pdata;
	}

	pdata->frmwrk = create_and_get_touch_framework(dev_node);
	if (pdata->frmwrk == NULL) {
		tp_log_err("%s %d:create_and_get_touch_framework fail.\n", __func__, __LINE__);
		rc = -EINVAL;
		goto fail_free_pdata;
	} else if (IS_ERR(pdata->frmwrk)) {
		tp_log_err("%s %d:create_and_get_touch_framework fail.\n", __func__, __LINE__);
		rc = PTR_ERR(pdata->frmwrk);
		goto fail_free_pdata;
	}

	return pdata;

fail_free_pdata:
	kfree(pdata);
fail:
	return ERR_PTR(rc);
}

static void free_proximity_pdata(void *pdata)
{
	struct cyttsp5_proximity_platform_data *proximity_pdata =
		(struct cyttsp5_proximity_platform_data *)pdata;

	free_touch_framework(proximity_pdata->frmwrk);

	kfree(proximity_pdata);
}

static struct cyttsp5_device_pdata_func device_pdata_funcs[DEVICE_TYPE_MAX] = {
	[DEVICE_MT] = {
		.create_and_get_pdata = create_and_get_mt_pdata,
		.free_pdata = free_mt_pdata,
	},
	[DEVICE_BTN] = {
		.create_and_get_pdata = create_and_get_btn_pdata,
		.free_pdata = free_btn_pdata,
	},
	[DEVICE_PROXIMITY] = {
		.create_and_get_pdata = create_and_get_proximity_pdata,
		.free_pdata = free_proximity_pdata,
	},
};

static struct cyttsp5_pdata_ptr pdata_ptr[DEVICE_TYPE_MAX];

static const char *device_names[DEVICE_TYPE_MAX] = {
	[DEVICE_MT] = "cy,mt",
	[DEVICE_BTN] = "cy,btn",
	[DEVICE_PROXIMITY] = "cy,proximity",
};

static void set_pdata_ptr(struct cyttsp5_platform_data *pdata)
{
	pdata_ptr[DEVICE_MT].pdata = (void **)&pdata->mt_pdata;
	pdata_ptr[DEVICE_BTN].pdata = (void **)&pdata->btn_pdata;
	pdata_ptr[DEVICE_PROXIMITY].pdata = (void **)&pdata->prox_pdata;
}

static int get_device_type(struct device_node *dev_node,
		enum cyttsp5_device_type *type)
{
	const char *name = NULL;
	enum cyttsp5_device_type t = DEVICE_MT;
	int rc = 0;

	rc = of_property_read_string(dev_node, "name", &name);
	if (rc) {
		tp_log_err("%s %d:[name] read fail\n", __func__, __LINE__);
		return rc;
	}

	for (t = 0; t < DEVICE_TYPE_MAX; t++) {
		if (!strncmp(name, device_names[t], MAX_NAME_LENGTH)) {
			*type = t;
			return 0;
		}
	}

	return -EINVAL;
}

static inline void *create_and_get_device_pdata(struct device_node *dev_node,
		enum cyttsp5_device_type type)
{
	return device_pdata_funcs[type].create_and_get_pdata(dev_node);
}

static inline void free_device_pdata(enum cyttsp5_device_type type)
{
	device_pdata_funcs[type].free_pdata(*pdata_ptr[type].pdata);
}

/*****************************************************************
Parameters    :  core_node
                 name
Return        :  a struct of touch_settings
Description   :  parse touch_settings information from dts
*****************************************************************/
static struct touch_settings *create_and_get_touch_setting(
		struct device_node *core_node, const char *name)
{
	struct touch_settings *setting = NULL;
	char *tag_name = NULL;
	u32 tag_value = 0;
	u16 *data = NULL;
	int size = 0;
	int rc = 0;

	if (NULL == core_node || NULL == name) {
		tp_log_err("%s %d:core_node or name is missing\n",
					__func__, __LINE__);
	#ifdef CONFIG_64BIT
		tp_log_err("%s %d:core_node = %ld, name = %ld\n", __func__,
					__LINE__, (long)core_node, (long)name);
	#else
		tp_log_err("%s %d:core_node = %d, name = %d\n", __func__,
					__LINE__, (int)core_node, (int)name);
	#endif
		return NULL;
	}

	data = create_and_get_u16_array(core_node, name, &size);
	if (IS_ERR_OR_NULL(data)) {
		tp_log_err("%s %d:create and get %s fail.\n",
					__func__, __LINE__, name);
		return (void *)data;
	}

	tp_log_debug("%s %d: Touch setting:'%s' size:%d\n",
				__func__, __LINE__, name, size);
	setting = kzalloc(sizeof(*setting), GFP_KERNEL);
	if (!setting) {
		tp_log_err("%s %d:kzalloc fail.\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto fail_free_data;
	}

	setting->data = (u8 *)data;
	setting->size = size;

	tag_name = kzalloc(MAX_NAME_LENGTH, GFP_KERNEL);
	if (!tag_name) {
		tp_log_err("%s %d:kzalloc fail.\n",	__func__, __LINE__);
		rc = -ENOMEM;
		goto fail_free_setting;
	}

	snprintf(tag_name, MAX_NAME_LENGTH, "%s-tag", name);

	rc = of_property_read_u32(core_node, tag_name, &tag_value);
	if (!rc) {
		tp_log_debug("%s %d:[%s] is %d.\n", __func__, __LINE__, name, tag_value);
		setting->tag = tag_value;
	} else {
		tp_log_err("%s %d:[%s] read fail, rc = %d.\n", __func__, __LINE__, name, rc);
	}

	kfree(tag_name);

	return setting;

fail_free_setting:
	kfree(setting);
fail_free_data:
	kfree(data);

	return ERR_PTR(rc);
}

static struct cyttsp5_power_config *create_and_get_power_config(struct device_node *core_node)
{
	struct cyttsp5_power_config * power_config = NULL;
	int rc = 0;
	u32 value = 0;

	power_config = kzalloc(sizeof(*power_config), GFP_KERNEL);
	if (!power_config) {
		tp_log_err("%s %d:kzalloc fail.\n", __func__, __LINE__);
		return NULL;
	}

	rc = of_property_read_u32(core_node, "cy,vdd-type", &value);
	if (rc) {
		tp_log_err("%s %d: [cy,vdd-type] read fail, rc = %d.\n", __func__, __LINE__, rc);
		goto fail_free;
	} else {
		power_config->vdd_type = (u8)value;
		tp_log_info("%s %d: [cy,vdd-type] is %d.\n", __func__, __LINE__, power_config->vdd_type);
	}

	rc = of_property_read_u32(core_node, "cy,vbus-type", &value);
	if (rc) {
		tp_log_err("%s %d: [cy,vbus-type] read fail, rc = %d.\n", __func__, __LINE__, rc);
		goto fail_free;
	} else {
		power_config->vbus_type = (u8)value;
		tp_log_info("%s %d: [cy,vbus-type] is %d.\n", __func__, __LINE__, power_config->vbus_type);
	}

	rc = of_property_read_string(core_node, "cy,vdd", &power_config->vdd_name);
	if (rc){
		tp_log_warning("%s %d:[cy,vdd] read fail, rc = %d.\n", __func__, __LINE__, rc);
		power_config->vdd_name = NULL;
	}

	rc = of_property_read_string(core_node, "cy,vbus", &power_config->vbus_name);
	if (rc){
			tp_log_warning("%s %d:[cy,vbus] read fail, rc = %d.\n", __func__, __LINE__, rc);
			power_config->vbus_name = NULL;
	}

	if (of_find_property(core_node, "cy,vdd-en-gpio", NULL)) {
		power_config->vdd_en_gpio = of_get_named_gpio_flags(core_node, "cy,vdd-en-gpio", 0, NULL);
		tp_log_info("%s %d:[cy,vdd-en-gpio] is %d.\n", __func__, __LINE__, power_config->vdd_en_gpio);
	} else {
		power_config->vdd_en_gpio = -1;
		tp_log_warning("%s %d:[cy,vdd-en-gpio] read fail.\n", __func__, __LINE__);
	}

	if (of_find_property(core_node, "cy,vbus-en-gpio", NULL)) {
		power_config->vbus_en_gpio = of_get_named_gpio_flags(core_node, "cy,vbus-en-gpio", 0, NULL);
		tp_log_info("%s %d:[cy,vbus-en-gpio] is %d.\n", __func__, __LINE__, power_config->vbus_en_gpio);
	} else {
		power_config->vbus_en_gpio = -1;
		tp_log_warning("%s %d:[cy,vbus-en-gpio] read fail.\n", __func__, __LINE__);
	}

	power_config->vdd_reg = NULL;
	power_config->vbus_reg = NULL;

	return power_config;

fail_free:
	kfree(power_config);
	return NULL;
}

static void free_touch_setting(struct touch_settings *setting)
{
	if (setting) {
		kfree(setting->data);
		kfree(setting);
	}
}
static char *touch_setting_names[CY_IC_GRPNUM_NUM] = {
	NULL,				/* CY_IC_GRPNUM_RESERVED */
	NULL,		/* CY_IC_GRPNUM_CMD_REGS */
	NULL,		/* CY_IC_GRPNUM_TCH_REP */
	NULL,		/* CY_IC_GRPNUM_DATA_REC */
	NULL,		/* CY_IC_GRPNUM_TEST_REC */
	NULL,		/* CY_IC_GRPNUM_PCFG_REC */
	NULL,	/* CY_IC_GRPNUM_TCH_PARM_VAL */
	NULL,	/* CY_IC_GRPNUM_TCH_PARM_SIZE */
	NULL,				/* CY_IC_GRPNUM_RESERVED1 */
	NULL,				/* CY_IC_GRPNUM_RESERVED2 */
	NULL,		/* CY_IC_GRPNUM_OPCFG_REC */
	NULL,		/* CY_IC_GRPNUM_DDATA_REC */
	NULL,		/* CY_IC_GRPNUM_MDATA_REC */
	NULL,		/* CY_IC_GRPNUM_TEST_REGS */
	"cy,btn_keys",		/* CY_IC_GRPNUM_BTN_KEYS */
	NULL,				/* CY_IC_GRPNUM_TTHE_REGS */
};

static struct cyttsp5_wakeup_keys *create_and_get_wakeup_keys(struct device_node *dev_node)
{
	struct cyttsp5_wakeup_keys *wakeup_keys;
	u16 *keys;
	int size;
	int rc;

	keys = create_and_get_u16_array(dev_node, "cy,easy_wakeup_gesture_keys", &size);
	if (IS_ERR_OR_NULL(keys))
		return (void *)keys;

	/* Check for valid abs size */
	if (size > GESTURE_MAX) {
		rc = -EINVAL;
		tp_log_err("%s:fail to check size of keys,line=%d\n", __func__,__LINE__);
		goto fail_free_keys;
	}

	wakeup_keys = kzalloc(sizeof(*wakeup_keys), GFP_KERNEL);
	if (wakeup_keys == NULL) {
		rc = -ENOMEM;
		tp_log_err("%s:fail to allocate memory,line=%d\n", __func__,__LINE__);
		goto fail_free_keys;
	}

	wakeup_keys->keys = keys;
	wakeup_keys->size = size;

	return wakeup_keys;

fail_free_keys:
	kfree(keys);

	return ERR_PTR(rc);
}

/**
 * get_of_u32_val() - get the u32 type value fro the DTSI.
 * @np: device node
 * @name: The DTSI key name
 * @default_value: If the name is not matched, return a default value.
 */
static u32 get_of_u32_val(struct device_node *np,
	const char *name,u32 default_val)
{
	u32 tmp= 0;
	int err = 0;
	err = of_property_read_u32(np, name, &tmp);
	if (!err)
		return tmp;
	else
		return default_val;
}
/*****************************************************************
Parameters    :  core_node
Return        :  a cyttsp5_core_platform_data struct pointer
				 or a error number less than zero
Description   :  init gpio, read dts configer....
*****************************************************************/
static struct cyttsp5_core_platform_data *create_and_get_core_pdata(
		struct device_node *core_node)
{
	struct cyttsp5_core_platform_data *pdata = NULL;
	u32 value = 0;
	int rc = 0;
	int i= 0;

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		tp_log_err("%s %d: kzalloc core_pdata fail.\n", __func__, __LINE__);
		rc = -ENOMEM;
		goto fail;
	}
	/* read irq_gpio */
	if (of_find_property(core_node, "cy,irq-gpio", NULL)) {
		pdata->irq_gpio = of_get_named_gpio_flags(core_node, "cy,irq-gpio", 0, NULL);
		tp_log_info("%s %d:irq gpio is %d.\n", __func__, __LINE__, pdata->irq_gpio);
	} else {
		/*invalid irq GPIO*/
		pdata->irq_gpio = -1;
		tp_log_err("%s %d:[cy,irq-gpio] read fail.\n", __func__, __LINE__);
	}

	if (of_find_property(core_node, "cy,reset-gpio", NULL)) {
		pdata->rst_gpio = of_get_named_gpio_flags(core_node, "cy,reset-gpio", 0, NULL);
		tp_log_info("%s %d:reset gpio is %d.\n", __func__, __LINE__, pdata->rst_gpio);
	} else {
		/*invalid reset GPIO*/
		pdata->rst_gpio = -1;
		tp_log_err("%s %d:[cy,reset-gpio] read fail.\n", __func__, __LINE__);
	}

	rc = of_property_read_u32(core_node, "cy,hid_desc_register", &value);
	if (rc) {
		tp_log_err("%s %d: [cy,hid_desc_register] read fail, rc = %d.\n",
					__func__, __LINE__, rc);
		rc = -ENODEV;
		goto fail_free;
	} else {
		pdata->hid_desc_register = value;
		tp_log_info("%s %d: pdata->hid_desc_register = %d.\n",
					__func__, __LINE__, value);
	}

	pdata->level_irq_udelay = get_of_u32_val(core_node, "cy,level_irq_udelay", 0);

	pdata->vendor_id = get_of_u32_val(core_node, "cy,vendor_id", 0);

	pdata->product_id = get_of_u32_val(core_node, "cy,product_id", 0);

	pdata->flags = get_of_u32_val(core_node, "cy,flags", 0);
	tp_log_info("%s %d:pdata->flags = %d.\n", __func__, __LINE__, pdata->flags);
	pdata->easy_wakeup_gesture = get_of_u32_val(core_node, "cy,easy_wakeup_gesture", 0);

	pdata->easy_wakeup_supported_gestures = get_of_u32_val(core_node,
			"cy,easy_wakeup_supported_gestures", 0);

	pdata->double_tap_enabled = get_of_u32_val(core_node, "cy,double_tap_enabled", 0);

	pdata->dtz_x0 = get_of_u32_val(core_node, "cy,dtz_x0", 0);

	pdata->dtz_y0 = get_of_u32_val(core_node, "cy,dtz_y0", 0);

	pdata->dtz_x1 = get_of_u32_val(core_node, "cy,dtz_x1", 0);

	pdata->dtz_y1 = get_of_u32_val(core_node, "cy,dtz_y1", 0);

	pdata->glove_support = get_of_u32_val(core_node, "cy,glove_support", 0);
	pdata->holster_support = get_of_u32_val(core_node, "cy,holster_support", 0);
	pdata->mmi_test_support = get_of_u32_val(core_node, "cy,mmi_test_support", 0);
	pdata->wakeup_keys = create_and_get_wakeup_keys(core_node);

	pdata->fw_upgrade_start_ver = get_of_u32_val(core_node, "cy,fw_upgrade_start_ver", 0);

	pdata->fw_update_logic = get_of_u32_val(core_node, "cy,fw_uptate_logic", 0);

	pdata->input_register_early= get_of_u32_val(core_node, "cy,input_register_early", 0);
	tp_log_info("%s:input_register_early = %d\n",__func__,pdata->input_register_early);

	/* parse product name */
	rc = of_property_read_string(core_node, "cy,product_name", &pdata->product_name);
	if (rc){
		tp_log_err("%s %d:Read name product_name fail, rc = %d\n", __func__, __LINE__, rc);
		pdata->product_name = "Unknow";
	}

	/* parse product name */
	rc = of_property_read_string(core_node, "cy,chip_name", &pdata->chip_name);
	if (rc){
		tp_log_info("%s %d:Read name chip_name fail rc = %d\n", __func__, __LINE__, rc);
        pdata->chip_name = NULL;
	} else {
        tp_log_info("%s :chip_name = %s\n", __func__, pdata->chip_name);
	}

	pdata->power_config = create_and_get_power_config(core_node);
	if (NULL == pdata->power_config) {
		tp_log_err("%s %d: power_config is null!.\n", __func__,__LINE__);
		goto fail_free;
	}

	/* read configer from dts */
	for (i = 0; (unsigned int)i < ARRAY_SIZE(touch_setting_names); i++) {
		if (touch_setting_names[i] == NULL) {
			continue;
		}

		pdata->sett[i] = create_and_get_touch_setting(core_node, touch_setting_names[i]);
		if (IS_ERR(pdata->sett[i])) {
			tp_log_err("%s %d: pdata->sett[i] = %d.\n",	__func__, __LINE__, (int)PTR_ERR(pdata->sett[i]));
			rc = PTR_ERR(pdata->sett[i]);
			goto fail_free_sett;
		} else if (pdata->sett[i] == NULL) {
			tp_log_debug("%s: No data for setting '%s'\n", __func__, touch_setting_names[i]);
		}
	}

	tp_log_info("%s: irq_gpio:%d rst_gpio:%d\n"
		"hid_desc_register:%d level_irq_udelay:%d vendor_id:%d product_id:%d\n"
		"flags:%d easy_wakeup_gesture:%d\n dtz_x0:%d dtz_x1:%d dtz_y0:%d dtz_y1:%d\n"
		"easy_wakeup_supported_gestures=%d double_tap_enabled=%d\n", __func__,
		pdata->irq_gpio, pdata->rst_gpio,
		pdata->hid_desc_register,
		pdata->level_irq_udelay, pdata->vendor_id, pdata->product_id,
		pdata->flags, pdata->easy_wakeup_gesture,
		pdata->dtz_x0,pdata->dtz_x1,pdata->dtz_y0,pdata->dtz_y1,
		pdata->easy_wakeup_supported_gestures,pdata->double_tap_enabled);

	pdata->xres = cyttsp5_xres;
	pdata->init = cyttsp5_init;
	pdata->power = cyttsp5_power;
	pdata->detect = cyttsp5_detect;
	pdata->irq_stat = cyttsp5_irq_stat;

	tp_log_info("%s %d:", __func__, __LINE__);
	return pdata;

fail_free_sett:
	for (i--; i >= 0; i--)
		free_touch_setting(pdata->sett[i]);

	kfree(pdata->power_config);
	pdata->power_config = NULL;
fail_free:
	kfree(pdata);
fail:
	return ERR_PTR(rc);
}

static void free_core_pdata(void *pdata)
{
	struct cyttsp5_core_platform_data *core_pdata = pdata;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(touch_setting_names); i++)
		free_touch_setting(core_pdata->sett[i]);
	kfree(core_pdata);
}

/*****************************************************************
Parameters    :  adap_dev
Return        :
Description   :
*****************************************************************/
int cyttsp5_devtree_create_and_get_pdata(struct device *adap_dev)
{
	struct cyttsp5_platform_data *pdata = NULL;
	struct device_node *core_node = NULL;
	struct device_node *dev_node = NULL;
	struct device_node *dev_node_fail = NULL;
	enum cyttsp5_device_type type = DEVICE_MT;
	int count = 0;
	int rc = 0;

	/* if do not find dts file, return no device */
	if (!adap_dev->of_node) {
		tp_log_err("%s %d: adap_dev->of_node is null.\n", __func__, __LINE__);
		return -ENODEV;
	}

	pdata = kzalloc(sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		tp_log_err("%s %d: kzalloc pdata memory fail.\n", __func__, __LINE__);
		return -ENOMEM;
	}

	adap_dev->platform_data = pdata;
	set_pdata_ptr(pdata);

	/* There should be only one core node */
	for_each_child_of_node(adap_dev->of_node, core_node) {
		const char *name;

		rc = of_property_read_string(core_node, "name", &name);
		if (!rc) {
			tp_log_info("%s %d:core_node name:%s\n", __func__, __LINE__, name);
		} else {
			tp_log_err("%s %d:core_node name get fail, rc = %d.\n",
						__func__, __LINE__, rc);
		}

		pdata->core_pdata = create_and_get_core_pdata(core_node);
		if (IS_ERR(pdata->core_pdata)) {
			tp_log_info("%s %d:create_and_get_core_pdata fail.\n", __func__, __LINE__);
			rc = PTR_ERR(pdata->core_pdata);
			break;
		}

		/* Increment reference count */
		of_node_get(core_node);

		for_each_child_of_node(core_node, dev_node) {
			count++;
			rc = get_device_type(dev_node, &type);
			if (rc) {
				tp_log_err("%s %d:get_device_type fail, rc = %d.\n",
							__func__, __LINE__, rc);
				break;
			}

			*pdata_ptr[type].pdata
				= create_and_get_device_pdata(dev_node, type);
			if (IS_ERR(*pdata_ptr[type].pdata)) {
				rc = PTR_ERR(*pdata_ptr[type].pdata);
				tp_log_err("%s %d:create_and_get_device_pdata fail, rc = %d\n", __func__, __LINE__, rc);
			}

			if (rc)
				break;
			/* Increment reference count */
			of_node_get(dev_node);
		}

		if (rc) {
			free_core_pdata(pdata->core_pdata);
			of_node_put(core_node);
			for_each_child_of_node(core_node, dev_node_fail) {
				if (dev_node == dev_node_fail) {
				#ifdef CONFIG_64BIT
					tp_log_err("%s %d:dev match, dev_node = %ld, dev_node_fail = %ld\n",
							__func__, __LINE__, (long)dev_node, (long)dev_node_fail);
				#else
					tp_log_err("%s %d:dev match, dev_node = %d, dev_node_fail = %d\n",
							__func__, __LINE__, (int)dev_node, (int)dev_node_fail);
				#endif
					break;
				}

				rc = get_device_type(dev_node, &type);
				if (rc) {
					tp_log_err("%s %d:get_device_type fail, rc = %d\n",__func__, __LINE__, rc);
					break;
				}
				free_device_pdata(type);
				of_node_put(dev_node);
			}
			break;
		}
		pdata->loader_pdata = &_cyttsp5_loader_platform_data;
	}

	pr_info("%s: %d child node(s) found\n", __func__, count);

	return rc;
}
EXPORT_SYMBOL_GPL(cyttsp5_devtree_create_and_get_pdata);

int cyttsp5_devtree_clean_pdata(struct device *adap_dev)
{
	struct cyttsp5_platform_data *pdata;
	struct device_node *core_node, *dev_node;
	enum cyttsp5_device_type type;
	int rc = 0;

	if (!adap_dev->of_node)
		return 0;

	pdata = dev_get_platdata(adap_dev);
	if (NULL == pdata) {
		return 0;
	}

	set_pdata_ptr(pdata);
	for_each_child_of_node(adap_dev->of_node, core_node) {
		if (pdata->core_pdata != NULL) {
			free_core_pdata(pdata->core_pdata);
		}

		of_node_put(core_node);
		for_each_child_of_node(core_node, dev_node) {
			rc = get_device_type(dev_node, &type);
			if (rc)
				break;
			free_device_pdata(type);
			of_node_put(dev_node);
		}
	}

	return rc;
}
EXPORT_SYMBOL_GPL(cyttsp5_devtree_clean_pdata);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Product DeviceTree Driver");
MODULE_AUTHOR("Cypress Semiconductor <ttdrivers@cypress.com>");
