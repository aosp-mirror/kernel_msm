/******************************************************************************
 * @file parameter.c
 *
 * parameter.c
 *
 * @version 0.0.1
 * @authors btok
 *
 *****************************************************************************//*
 * Copyright (2014), Cypress Semiconductor Corporation. All rights reserved.
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
 *****************************************************************************/
#include "parameter.h"
#include "csv.h"
#include "../cyttsp5_regs.h"
#include "../cyttsp5_core.h"
#include <linux/slab.h>
#include <linux/uaccess.h>

struct parameter_enumeration {
    struct parameter_enumeration *next;
    int index;
    union parameter_value *value;
};

struct parameter {
    struct parameter *next;
    enum parameter_id id;
    enum parameter_type type;
    uint16_t address;
    uint16_t size;
    uint32_t mask;
    struct parameter_enumeration *enumerations;
};

struct parameter_id_mapping {
    enum parameter_id parameter_id;
    char *parameter_name;
};

struct parameter_type_mapping {
    enum parameter_type parameter_type;
    char *parameter_type_name;
};

enum parser_state {
    IDLE,
    NAME_PARSED,
    TYPE_PARSED,
    ADDRESS_PARSED,
    SIZE_PARSED,
    MASK_PARSED,
    ENUM_PARSING,
    END,
    ERROR,
};

struct parser_data {
    enum parser_state state;
    struct parameter *current_parameter;
    int error;
};

static struct parameter *parameters;

static struct parameter_id_mapping parameter_id_mappings[PARAMETER_ID_MAX] = {
    {
        .parameter_id = TX_NUM,
        .parameter_name = "TX_NUM",
    },
    {
        .parameter_id = RX_NUM,
        .parameter_name = "RX_NUM",
    },
    {
        .parameter_id = BUTTON_NUM,
        .parameter_name = "BUTTON_NUM",
    },
    {
        .parameter_id = SCANNING_MODE_BUTTON,
        .parameter_name = "SCANNING_MODE_BUTTON",
    },
    {
        .parameter_id = TX_PERIOD_MUTUAL,
        .parameter_name = "TX_PERIOD_MUTUAL",
    },
    {
        .parameter_id = TX_PERIOD_SELF,
        .parameter_name = "TX_PERIOD_SELF",
    },
    {
        .parameter_id = TX_PERIOD_BTN_MUTUAL,
        .parameter_name = "TX_PERIOD_BTN_MUTUAL",
    },
    {
        .parameter_id = TX_PERIOD_BTN_SELF,
        .parameter_name = "TX_PERIOD_BTN_SELF",
    },
    {
        .parameter_id = MTX_ORDER,
        .parameter_name = "MTX_ORDER",
    },
    {
        .parameter_id = VDDA_MODE,
        .parameter_name = "VDDA_MODE",
    },
    {
        .parameter_id = SCALING_FACTOR_MUTUAL,
        .parameter_name = "SCALING_FACTOR_MUTUAL",
    },
    {
        .parameter_id = SCALING_FACTOR_SELF,
        .parameter_name = "SCALING_FACTOR_SELF",
    },
    {
        .parameter_id = SCALING_FACTOR_BUTTON_MUTUAL,
        .parameter_name = "SCALING_FACTOR_BUTTON_MUTUAL",
    },
    {
        .parameter_id = SCALING_FACTOR_BUTTON_SELF,
        .parameter_name = "SCALING_FACTOR_BUTTON_SELF",
    },
    {
        .parameter_id = BALANCING_TARGET_MUTUAL,
        .parameter_name = "BALANCING_TARGET_MUTUAL",
    },
    {
        .parameter_id = BALANCING_TARGET_SELF,
        .parameter_name = "BALANCING_TARGET_SELF",
    },
    {
        .parameter_id = BALANCING_TARGET_BUTTON_MUTUAL,
        .parameter_name = "BALANCING_TARGET_BUTTON_MUTUAL",
    },
    {
        .parameter_id = BALANCING_TARGET_BUTTON_SELF,
        .parameter_name = "BALANCING_TARGET_BUTTON_SELF",
    },
    {
        .parameter_id = GIDAC_MULT,
        .parameter_name = "GIDAC_MULT",
    },
    {
        .parameter_id = RXDAC,
        .parameter_name = "RXDAC",
    },
    {
        .parameter_id = REF_SCALE,
        .parameter_name = "REF_SCALE",
    },
    {
        .parameter_id = ACT_LFT_EN,
        .parameter_name = "ACT_LFT_EN",
    },
    {
        .parameter_id = BL_H20_RJCT,
        .parameter_name = "BL_H20_RJCT",
    },
    {
        .parameter_id = TX_PUMP_VOLTAGE,
        .parameter_name = "TX_PUMP_VOLTAGE",
    },
    {
        .parameter_id = SENSOR_ASSIGNMENT,
        .parameter_name = "SENSOR_ASSIGNMENT",
    },
    {
        .parameter_id = GIDAC_LSB_CONFIG,
        .parameter_name = "GIDAC_LSB_CONFIG",
    },
    {
        .parameter_id = INT_CAP_MUTUAL,
        .parameter_name = "INT_CAP_MUTUAL",
    },
    {
        .parameter_id = INT_CAP_SELF,
        .parameter_name = "INT_CAP_SELF",
    },
    {
        .parameter_id = INT_CAP_BUTTON_MUTUAL,
        .parameter_name = "INT_CAP_BUTTON_MUTUAL",
    },
    {
        .parameter_id = INT_CAP_BUTTON_SELF,
        .parameter_name = "INT_CAP_BUTTON_SELF",
    },
    {
        .parameter_id = TX_VOLTAGE_MUTUAL,
        .parameter_name = "TX_VOLTAGE_MUTUAL",
    },
};

static struct parameter_type_mapping
        parameter_type_mappings[PARAMETER_TYPE_MAX] = {
    {
        .parameter_type = INTEGER,
        .parameter_type_name = "integer",
    },
    {
        .parameter_type = FLOAT,
        .parameter_type_name = "float",
    },
    {
        .parameter_type = STRING,
        .parameter_type_name = "string",
    },
};

static int find_parameter_id(char *parameter_name, enum parameter_id *id)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(parameter_id_mappings); i++)
        if (!strcmp(parameter_id_mappings[i].parameter_name,
                parameter_name)) {
            *id = parameter_id_mappings[i].parameter_id;
            return 0;
        }

    return -EINVAL;
}

static int find_parameter_type(char *parameter_type_name,
        enum parameter_type *type)
{
    int i;

    for (i = 0; i < ARRAY_SIZE(parameter_type_mappings); i++)
        if (!strcmp(parameter_type_mappings[i].parameter_type_name,
                parameter_type_name)) {
            *type = parameter_type_mappings[i].parameter_type;
            return 0;
        }

    return -EINVAL;
}

static struct parameter *find_parameter(enum parameter_id id,
        struct parameter **last_parameter)
{
    struct parameter *parameter = parameters;

    if (last_parameter)
        *last_parameter = NULL;

    while (parameter) {
        if (parameter->id == id)
            return parameter;
        if (last_parameter)
            *last_parameter = parameter;
        parameter = parameter->next;
    }

    return NULL;
}

static struct parameter_enumeration *find_parameter_enumeration(
        struct parameter *parameter, int index,
        struct parameter_enumeration **last_parameter_enumeration)
{
    struct parameter_enumeration *parameter_enumeration =
                    parameter->enumerations;

    if (last_parameter_enumeration)
        *last_parameter_enumeration = NULL;

    while (parameter_enumeration) {
        if (parameter_enumeration->index == index)
            return parameter_enumeration;
        if (last_parameter_enumeration)
            *last_parameter_enumeration = parameter_enumeration;
        parameter_enumeration = parameter_enumeration->next;
    }

    return NULL;
}

static int add_parameter(enum parameter_id id)
{
    struct parameter *parameter, *last_parameter;

    parameter = find_parameter(id, &last_parameter);
    if (parameter)
        return -EEXIST;

    parameter = kzalloc(sizeof(struct parameter), GFP_KERNEL);
    if (!parameter)
        return -ENOMEM;

    memset(parameter, 0, sizeof(struct parameter));
    parameter->id = id;
    parameter->next = NULL;
    parameter->enumerations = NULL;

    if (last_parameter)
        last_parameter->next = parameter;
    else
        parameters = parameter;

    return 0;
}

static int add_parameter_enumeration(enum parameter_id id, int index,
        union parameter_value *value)
{
    struct parameter *parameter;
    struct parameter_enumeration *parameter_enumeration;
    struct parameter_enumeration *last_parameter_enumeration;

    parameter = find_parameter(id, NULL);
    if (!parameter)
        return -EINVAL;

    parameter_enumeration = find_parameter_enumeration(parameter, index,
                    &last_parameter_enumeration);
    if(parameter_enumeration)
        return -EEXIST;

    parameter_enumeration = kzalloc(sizeof(struct parameter_enumeration), GFP_KERNEL);
    if (!parameter_enumeration)
        return -ENOMEM;

    parameter_enumeration->index = index;
    parameter_enumeration->value = value;
    parameter_enumeration->next = NULL;

    if (last_parameter_enumeration)
        last_parameter_enumeration->next = parameter_enumeration;
    else
        parameter->enumerations = parameter_enumeration;

    return 0;
}

static void parser_set_state(struct parser_data *data, enum parser_state state)
{
    if (state == IDLE) {
        data->current_parameter = NULL;
        data->error = 0;
    }

    data->state = state;
}
static void parser_init(struct parser_data *data)
{
    parser_set_state(data, IDLE);
}

static int parse_enumeration(char *field, enum parameter_type type, int *index,
        union parameter_value **value)
{
    char *value_str;

    if (sscanf(field, "%d", index) != 1)
        return -EINVAL;

    value_str = strchr(field, ':');
    if (!value_str)
        return -EINVAL;

    value_str++;

    *value = kzalloc(sizeof(union parameter_value), GFP_KERNEL);
    if (!*value)
        return -ENOMEM;

    switch (type) {
    case INTEGER:
        if (sscanf(value_str, "%d", &(*value)->integer) != 1) {
            kfree(*value);
            return -EINVAL;
        }
        break;

    case FLOAT:
        if (sscanf(value_str, "%d", &(*value)->flt) != 1) {
            kfree(*value);
            return -EINVAL;
        }
        break;

    case STRING:
        if (strlen(value_str) == 0) {
            kfree(*value);
            return -EINVAL;
        }
        strncpy((*value)->string, value_str, MAX_STRING_LENGTH - 1);
        break;
	default:
		break;
	}
    return 0;
}

static void free_parameter_enumerations(struct parameter *parameter)
{
    struct parameter_enumeration *enumeration = parameter->enumerations;

    while (enumeration) {
        struct parameter_enumeration *enumeration_temp;
        kfree(enumeration->value);

        enumeration_temp = enumeration->next;
        kfree(enumeration);
        enumeration = enumeration_temp;
    }
}

static void free_parameters(void)
{
    struct parameter *parameter = parameters;

    while (parameter) {
        struct parameter *parameter_temp;
        free_parameter_enumerations(parameter);

        parameter_temp = parameter->next;
        kfree(parameter);
        parameter = parameter_temp;
    }

    parameters = NULL;
}

static void field_read_callback(void *s, size_t len, void *d)
{
    struct parser_data *data = (struct parser_data *)d;
    char *field = (char *)s;
    enum parameter_id parameter_id;
    enum parameter_type parameter_type;
    union parameter_value *value;
    int address;
    int size;
    int mask;
    int index;
    int ret;

    if (!strcmp(field, ""))
        return;

    tp_log_debug("%s: field:%s\n", __func__, field);

    switch (data->state) {
    case IDLE:
        ret = find_parameter_id(field, &parameter_id);
        if (ret)
            return;

        tp_log_debug("%s: Parameter ID:%d\n", __func__, parameter_id);
        ret = add_parameter(parameter_id);
        if (ret) {
            tp_log_err("%s: Unable to add parameter, ID:%d\n",
                    __func__, parameter_id);
            goto error;
        }

        data->current_parameter = find_parameter(parameter_id, NULL);
        if (!data->current_parameter){
            tp_log_err("%s: Unable to add parameter, ID:%d\n",
                    __func__, parameter_id);
            ret = -EINVAL;
            goto error;
        }

        parser_set_state(data, NAME_PARSED);
        break;

    case NAME_PARSED:
        ret = find_parameter_type(field, &parameter_type);
        if (ret)
            goto error;

        tp_log_debug("%s: Parameter Type:%d\n", __func__,
                parameter_type);
        data->current_parameter->type = parameter_type;
        parser_set_state(data, TYPE_PARSED);
        break;

    case TYPE_PARSED:
        if (sscanf(field, "0x%X", &address) != 1) {
            ret = -EINVAL;
            goto error;
        }
        tp_log_debug("%s: Parameter Address:0x%X\n", __func__,
                address);
        data->current_parameter->address = (uint16_t)address;
        parser_set_state(data, ADDRESS_PARSED);
        break;

    case ADDRESS_PARSED:
        if (sscanf(field, "%d", &size) != 1) {
            ret = -EINVAL;
            goto error;
        }
        tp_log_debug("%s: Parameter Size:%d\n", __func__,
                size);
        data->current_parameter->size = (uint16_t)size;
        parser_set_state(data, SIZE_PARSED);
        break;

    case SIZE_PARSED:
        if (sscanf(field, "0x%X", &mask) != 1) {
            ret = -EINVAL;
            goto error;
        }
        tp_log_debug("%s: Parameter Mask:%X\n", __func__,
                mask);
        data->current_parameter->mask = (uint32_t)mask;
        parser_set_state(data, MASK_PARSED);
        break;

    case MASK_PARSED:
    case ENUM_PARSING:
        ret = parse_enumeration(field, data->current_parameter->type,
                    &index, &value);
        if (ret) {
            tp_log_err("%s: Parameter Enumeration Index:%d\n", __func__,
                    index);
            goto error;
        }
#ifdef VDEBUG
        switch (data->current_parameter->type) {
        case INTEGER:
            tp_log_debug("Integer: %d\n", value->integer);
            break;
        case FLOAT:
            tp_log_debug("Float: %d\n", value->flt);
            break;
        case STRING:
            tp_log_debug("String: %s\n", value->string);
            break;
        }
#endif
        ret = add_parameter_enumeration(data->current_parameter->id,
                index, value);
        if (ret) {
            tp_log_err("%s: Unable to add parameter enumeratioon, ID:%d\n",
                    __func__, data->current_parameter->id);
            kfree(value);
            goto error;
        }

        parser_set_state(data, ENUM_PARSING);
        break;

	default:
		break;
    }

    return;

error:
    data->error = ret;
    parser_set_state(data, ERROR);
}

static void end_of_record_callback(int c, void *d)
{
    struct parser_data *data = (struct parser_data *)d;

    tp_log_debug("%s: Enter\n", __func__);
    switch (data->state) {
    case IDLE:
    case ERROR:
        return;

    case NAME_PARSED:
    case TYPE_PARSED:
    case ADDRESS_PARSED:
        tp_log_err("%s: Incomplete parameter field, ID:%d\n",
                __func__, data->current_parameter->id);
        data->error = -EINVAL;
        parser_set_state(data, ERROR);
        return;
    default:
        break;
    }

    parser_set_state(data, IDLE);
}

int parameter_init(struct file *file)
{
    struct csv_parser p;
    uint8_t buffer[512] = {0};
    size_t bytes_read;
    struct parser_data data;
    int ret;

    if (!file)
        return -EINVAL;

    ret = csv_init(&p, CSV_APPEND_NULL);
    if (ret) {
        goto exit;
    }

    parser_init(&data);

    while ((bytes_read = vfs_read(file, buffer, sizeof(buffer), &(file->f_pos))) > 0) {
        if (data.state == ERROR) {
            ret = data.error;
            goto finish;
        }

        if (csv_parse(&p, buffer, bytes_read, field_read_callback,
                end_of_record_callback, &data) != bytes_read) {
            tp_log_err("%s, Error parsing file: %s\n",
                    __func__, csv_strerror(csv_error(&p)));
            ret = -csv_error(&p);
            goto finish;
        }
    }

    csv_fini(&p, field_read_callback, end_of_record_callback, &data);

    parser_set_state(&data, END);

finish:
    csv_free(&p);

exit:
    if (ret)
        free_parameters();

    return ret;
}

void parameter_exit()
{
    free_parameters();
}

int parameter_get_info(enum parameter_id id, uint16_t *address, uint16_t *size,
        uint32_t *mask, enum parameter_type *type)
{
    struct parameter *parameter;

    parameter = find_parameter(id, NULL);
    if (!parameter)
        return -EINVAL;

    if (address)
        *address = parameter->address;
    if (size)
        *size = parameter->size;
    if (mask)
        *mask = parameter->mask;
    if (type)
        *type = parameter->type;

    return 0;
}

int parameter_get_enumerated_value(enum parameter_id id, int index,
        const union parameter_value **value)
{
    struct parameter *parameter;
    struct parameter_enumeration *parameter_enumeration;

    parameter = find_parameter(id, NULL);
    if (!parameter)
        return -EINVAL;

    parameter_enumeration = find_parameter_enumeration(parameter,
                    index, NULL);

    if (value) {
        if (parameter_enumeration) {
            *value = parameter_enumeration->value;
        } else {
            *value = NULL;
        }
    }
    return 0;
}
