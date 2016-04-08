/******************************************************************************
 * @file cm_cp_test.c
 *
 * cm_cp_test.c
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

#include <linux/device.h>
#include "pip.h"
#include "lookup.h"
#include "parameter.h"
#include "configuration.h"
#include "result.h"
#include "cm_cp_test.h"
#include "../cyttsp5_regs.h"
#include "../cyttsp5_core.h"

#define IDAC_LSB_DEFAULT	80
#define MTX_SUM_DEFAULT		1
#define CLK_DEFAULT		48
#define VREF_DEFAULT     2

#define CONVERSION_CONST	1000
#define MAX_READ_LENGTH     100


#define ABS(x)			(((x) < 0) ? -(x) : (x))

static int8_t mtx_sum_lookup_table[] =
    {MTX_SUM_LOOKUP_TABLE};
static int mtx_sum_lookup(uint8_t mtx_order, int *mtx_sum)
{
    if (IS_MTX_ORDER_VALID(mtx_order)) {
        *mtx_sum = (int)mtx_sum_lookup_table[
                GET_MTX_SUM_OFFSET(mtx_order)];
        return 0;
    }

    return -EINVAL;
};
static int get_configuration_parameter(enum parameter_id id,
        enum parameter_type *parameter_type,
        union parameter_value *parameter_value)
{
    uint16_t parameter_address;
    uint16_t parameter_size;
    uint32_t parameter_mask;
    const union parameter_value *enumerated_value;
    uint16_t row_number;
    uint16_t row_offset;
    uint16_t read_length;
    uint8_t data[ROW_SIZE];
    uint32_t value;
    int ret;

    tp_log_debug("%s,id:%d\n", __func__, id);
    ret = parameter_get_info(id, &parameter_address, &parameter_size,
            &parameter_mask, parameter_type);
    if (ret) {
        tp_log_err("%s:Unable to get parameter info!\n",__func__);
        goto exit;
    }

    row_number = parameter_address / ROW_SIZE;
    row_offset = parameter_address % ROW_SIZE;

    ret = pip_read_data_block(row_number, row_offset + parameter_size,
            CONFIG_BLOCK_ID, &read_length, data);
    if (ret) {
        tp_log_err("%s:Unable to read data block!\n",__func__);
        goto exit;
    }

    tp_log_info("%s: data[0..%d]:\n", __func__, read_length - 1);

    value = data[row_offset];
    if (parameter_size == 2 || parameter_size == 4)
        value += data[row_offset + 1] << 8;
    if (parameter_size == 4) {
        value += data[row_offset + 2] << 16;
        value += data[row_offset + 3] << 24;
    }

    if (parameter_mask) {
        value &= parameter_mask;
        while ((parameter_mask & 0x01) == 0) {
            value >>= 1;
            parameter_mask >>= 1;
        }
    }

    ret = parameter_get_enumerated_value(id, (int)value,
            &enumerated_value);
    if (ret) {
        tp_log_err("%s:Unable to get parameter enumerated value!\n",__func__);
        goto exit;
    }

    if (enumerated_value)
        memcpy(parameter_value, enumerated_value,
            sizeof(union parameter_value));
    else if (!enumerated_value && *parameter_type == INTEGER)
        parameter_value->integer = (int32_t)value;
    else {
        tp_log_err("%s:Unable to get parameter value!\n",__func__);
        ret = -EINVAL;
    }

exit:
    return ret;
}

/*
 * Wrapper function for pip_retrieve_scan to call it multiple times
 * to gather data if necessary
 */
static int retrieve_panel_scan(uint16_t read_offset, uint16_t read_length,
        uint8_t data_id, uint16_t *actual_read_length,
        uint8_t *data_format, uint8_t *data)
{
    uint16_t actual_read_len;
    uint16_t myread = 0;
    uint16_t total_read_len = 0;
    int ret = 0;

    while (read_length > 0) {
        if (read_length > MAX_READ_LENGTH){
            myread = MAX_READ_LENGTH;
        } else {
            myread = read_length;
        }
        ret = pip_retrieve_panel_scan(read_offset, myread,
                data_id, &actual_read_len, data_format, data);
        if (ret) {
            tp_log_err("%s:Unable to retrieve panel scan!\n",__func__);
            goto exit;
        }
        if (actual_read_len == 0)
            break;
        tp_log_debug("%s: read_offset: %d\n", __func__, read_offset);
        tp_log_debug("%s: actual_read_len: %d\n", __func__,
                actual_read_len);
        tp_log_debug("%s: read_length: %d\n", __func__, read_length);
        tp_log_debug("%s: data_format: %02X\n", __func__,
                *data_format);

        read_offset += actual_read_len;
        total_read_len += actual_read_len;

        data += actual_read_len * GET_ELEMENT_SIZE(*data_format);

        read_length -= actual_read_len;
    }

    *actual_read_length = total_read_len;

exit:
    return ret;
}

static int32_t get_element(uint8_t element_size, uint8_t sign, uint8_t *data)
{
    if (element_size == 1) {
        if (sign == SIGN_UNSIGNED)
            return *data;
        else
            return (int8_t)*data;
    } else if (element_size == 2) {
        if (sign == SIGN_UNSIGNED)
            return get_unaligned_le16(data);
        else
            return (int16_t)get_unaligned_le16(data);
    } else if (element_size == 4) {
        if (sign == SIGN_UNSIGNED)
            return get_unaligned_le32(data);
        else
            return (int)get_unaligned_le32(data);
    }

    return 0;
}

static int retrieve_panel_raw_data(uint8_t data_id, uint16_t read_offset,
        uint16_t read_length, uint8_t *data_format, int32_t *raw_data)
{
    uint16_t actual_read_length;
    uint8_t element_size;
    uint8_t sign;
    uint8_t *data;
    int ret;
    int i;

    data = kzalloc(read_length * sizeof(uint32_t), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    ret = retrieve_panel_scan(read_offset, read_length, data_id,
            &actual_read_length, data_format, data);
    if (ret) {
        tp_log_err("%s:Unable to retrieve panel raw data!\n",__func__);
        goto free;
    }

    if (read_length != actual_read_length) {
        ret = -EINVAL;
        goto free;
    }

    element_size = GET_ELEMENT_SIZE(*data_format);
    if (element_size != 1 && element_size != 2 && element_size != 4) {
        ret = -EINVAL;
        goto free;
    }

    sign = GET_SIGN(*data_format);

    for (i = 0; i < read_length; i++)
        raw_data[i] = get_element(element_size, sign,
                    &data[i << (element_size / 2)]);

free:
    kfree(data);

    return ret;
}

static int retrieve_data_structure(uint16_t read_offset, uint16_t read_length,
        uint8_t data_id, uint16_t *actual_read_length,
        uint8_t *data_format, uint8_t *data)
{
    uint16_t actual_read_len;
    uint16_t total_read_len = 0;
    uint16_t myread = 0;
    int ret = 0;
    tp_log_info("%s called\n", __func__);
    while (read_length > 0) {
        if(read_length > MAX_READ_LENGTH) {
                myread = MAX_READ_LENGTH;
        } else {
                myread = read_length;
        }
        ret = pip_retrieve_data_structure(read_offset, myread,
                data_id, &actual_read_len, data_format, data);
        if (ret) {
            tp_log_err("%s:Unable to retrieve panel scan!\n",__func__);
            goto exit;
        }
        //mdelay(5000);
        if (actual_read_len == 0)
            break;
        tp_log_debug("%s: read_offset: %d\n", __func__, read_offset);
        tp_log_debug("%s: actual_read_len: %d\n", __func__,
                actual_read_len);
        tp_log_debug("%s: read_length: %d\n", __func__, read_length);
        tp_log_debug("%s: data_format: %02X\n", __func__,
                *data_format);

        read_offset += actual_read_len;
        total_read_len += actual_read_len;

        data += actual_read_len * GET_ELEMENT_SIZE(*data_format);

        read_length -= actual_read_len;
    }

    *actual_read_length = total_read_len;

exit:
    return ret;
}

static int retrieve_panel_localpwc(uint8_t data_id, uint16_t read_offset,
        uint16_t read_length, uint8_t *data_format, int32_t *sensor_localpwc)
{
    uint16_t actual_read_length;
    uint8_t element_size;
    uint8_t sign;
    uint8_t *data;
    int ret;
    int i;

    data = kzalloc(read_length * sizeof(uint32_t), GFP_KERNEL);
    if (!data)
        return -ENOMEM;

    ret = retrieve_data_structure(read_offset, read_length, data_id,
            &actual_read_length, data_format, data);
    if (ret) {
        tp_log_err("%s:Unable to retrieve panel pwc data!\n",__func__);
        goto free;
    }

    if (read_length != actual_read_length) {
        ret = -EINVAL;
        goto free;
    }

    element_size = GET_ELEMENT_SIZE(*data_format);
    if (element_size != 1 && element_size != 2 && element_size != 4) {
        ret = -EINVAL;
        goto free;
    }

    sign = GET_SIGN(*data_format);

    for (i = 0; i < read_length; i++)
        sensor_localpwc[i] = get_element(element_size, sign,
                    &data[i << (element_size / 2)]);

free:
    kfree(data);

    return ret;
}

static int calculate_cm(int sensor_rawdata, int sensor_localpwc, int int_cap_mutual, int idac_lsb, int vref,
      int giadc_val, int scaling_factor_mutual, int clk, int vtx, int mtx_sum)
{
    int qidac = 4 * idac_lsb * giadc_val * sensor_localpwc/clk;
    int qint = int_cap_mutual * vref* sensor_rawdata * 100000/scaling_factor_mutual/2048;
    int cm_sensor = (qidac - qint) / (vtx);

    tp_log_debug("sensor_rawdata:%d ,sensor_localpwc :%d, qidac :%d, qint : %d, cm_sensor :%d\n",
           sensor_rawdata, sensor_localpwc,qidac, qint ,cm_sensor);

    return cm_sensor;
}

static void calculate_gradient_row(struct gd_sensor *gd_sensor_row_head,
            uint16_t row_num, int exclude_row_edge,int exclude_col_edge) {
    int i = 0;
    int cm_min_cur = 0;
    int cm_max_cur = 0;
    int cm_ave_cur = 0;
    int cm_ave_next = 0;
    int cm_ave_prev = 0;
    struct gd_sensor *p = gd_sensor_row_head;
    if( exclude_row_edge) {
        for(i = 0; i < row_num; i++) {
            if(!exclude_col_edge ) {
                cm_ave_cur = (p + i)->cm_ave;
                cm_min_cur =  (p + i)->cm_min;
                cm_max_cur =  (p + i)->cm_max;
                if(i < ( row_num-1)) {
                    cm_ave_next = (p + i+1)->cm_ave;
                }
                if(i > 0) {
                    cm_ave_prev = (p + i-1)->cm_ave;
                }
            } else {
                cm_ave_cur = (p + i)->cm_ave_exclude_edge;
                cm_min_cur =  (p + i)->cm_min_exclude_edge;
                cm_max_cur =  (p + i)->cm_max_exclude_edge;
                if(i < ( row_num-1)) {
                     cm_ave_next = (p + i+1)->cm_ave_exclude_edge;
                }
                if(i > 0) {
                    cm_ave_prev = (p + i-1)->cm_ave_exclude_edge;
                }
            }
            if(cm_ave_cur == 0) {
                cm_ave_cur = 1;
            }
            if((i == 0) || (i == (row_num-1))) {
                (p + i)->gradient_val = (cm_max_cur -cm_min_cur) *100/ cm_ave_cur;
            } else if(i==1){
                (p + i)->gradient_val = (cm_max_cur -cm_min_cur + ABS(cm_ave_cur -cm_ave_next )) *100/ cm_ave_cur;
            } else {
                (p + i)->gradient_val = (cm_max_cur -cm_min_cur + ABS(cm_ave_cur -cm_ave_prev )) *100/ cm_ave_cur;
            }
        }
    } else if(!exclude_row_edge) {
        for(i = 0; i < row_num; i++) {
                if(!exclude_col_edge ) {
                    cm_ave_cur = (p + i)->cm_ave;
                    cm_min_cur =  (p + i)->cm_min;
                    cm_max_cur =  (p + i)->cm_max;
                    if(i < ( row_num-1)) {
                        cm_ave_next = (p + i+1)->cm_ave;
                    }
                    if(i > 0) {
                        cm_ave_prev = (p + i-1)->cm_ave;
                    }
                } else {
                    cm_ave_cur = (p + i)->cm_ave_exclude_edge;
                    cm_min_cur =  (p + i)->cm_min_exclude_edge;
                    cm_max_cur =  (p + i)->cm_max_exclude_edge;
                    if(i < ( row_num-1)) {
                         cm_ave_next = (p + i+1)->cm_ave_exclude_edge;
                    }
                    if(i > 0) {
                        cm_ave_prev = (p + i-1)->cm_ave_exclude_edge;
                    }
                }
                if(cm_ave_cur == 0) {
                    cm_ave_cur = 1;
                }
                 if(i <= 1) {
                    (p + i)->gradient_val = (cm_max_cur -cm_min_cur + ABS(cm_ave_cur -cm_ave_next )) *100/ cm_ave_cur;
                } else {
                    (p + i)->gradient_val = (cm_max_cur -cm_min_cur + ABS(cm_ave_cur -cm_ave_prev )) *100/ cm_ave_cur;
                }
            }
    }
}
static void calculate_gradient_col(struct gd_sensor *gd_sensor_row_head,
            uint16_t col_num, int  exclude_row_edge,int  exclude_col_edge) {
    int i = 0;
    int cm_min_cur = 0;
    int cm_max_cur = 0;
    int cm_ave_cur = 0;
    int cm_ave_next = 0;
    int cm_ave_prev = 0;
    struct gd_sensor *p = gd_sensor_row_head;
    if( !exclude_col_edge) {
        for(i = 0; i < col_num; i++) {
            if(!exclude_row_edge ) {
                cm_ave_cur = (p + i)->cm_ave;
                cm_min_cur =  (p + i)->cm_min;
                cm_max_cur =  (p + i)->cm_max;
                if(i < ( col_num-1)) {
                    cm_ave_next = (p + i+1)->cm_ave;
                }
                if(i > 0) {
                    cm_ave_prev = (p + i-1)->cm_ave;
                }
            } else {
                cm_ave_cur = (p + i)->cm_ave_exclude_edge;
                cm_min_cur =  (p + i)->cm_min_exclude_edge;
                cm_max_cur =  (p + i)->cm_max_exclude_edge;
                if(i < ( col_num-1)) {
                     cm_ave_next = (p + i+1)->cm_ave_exclude_edge;
                }
                if(i > 0) {
                    cm_ave_prev = (p + i-1)->cm_ave_exclude_edge;
                }
            }
            if(cm_ave_cur == 0) {
                cm_ave_cur = 1;
            }
            if(i <= 1){
                (p + i)->gradient_val = (cm_max_cur -cm_min_cur + ABS(cm_ave_cur -cm_ave_next )) *100/ cm_ave_cur;
            } else {
                (p + i)->gradient_val = (cm_max_cur -cm_min_cur + ABS(cm_ave_cur -cm_ave_prev )) *100/ cm_ave_cur;
            }
        }
    } else if(exclude_col_edge) {
        for(i = 0; i < col_num; i++) {
                if(!exclude_row_edge ) {
                    cm_ave_cur = (p + i)->cm_ave;
                    cm_min_cur =  (p + i)->cm_min;
                    cm_max_cur =  (p + i)->cm_max;
                    if(i < ( col_num-1)) {
                         cm_ave_next = (p + i+1)->cm_ave;
                    }
                    if(i > 0) {
                        cm_ave_prev = (p + i-1)->cm_ave;
                    }
                } else {
                    cm_ave_cur = (p + i)->cm_ave_exclude_edge;
                    cm_min_cur =  (p + i)->cm_min_exclude_edge;
                    cm_max_cur =  (p + i)->cm_max_exclude_edge;
                    if(i < ( col_num-1)) {
                        cm_ave_next =  (p + i+1)->cm_ave_exclude_edge;
                    }
                    if(i > 0) {
                        cm_ave_prev = (p + i-1)->cm_ave_exclude_edge;
                    }
                }
                if(cm_ave_cur == 0) {
                    cm_ave_cur = 1;
                }

                if((i == 0) || (i == (col_num -1))) {
                    (p + i)->gradient_val = (cm_max_cur -cm_min_cur ) *100/ cm_ave_cur;
                }
                 else if(i == 1){
                    (p + i)->gradient_val = (cm_max_cur -cm_min_cur + ABS(cm_ave_cur -cm_ave_next )) *100/ cm_ave_cur;
                } else {
                    (p + i)->gradient_val = (cm_max_cur -cm_min_cur + ABS(cm_ave_cur -cm_ave_prev )) *100/ cm_ave_cur;
                }
            }
    }

}



static void fill_gd_sensor_table(struct gd_sensor * head, int index, uint16_t cm_max, uint16_t cm_min,
    int cm_ave, uint16_t cm_max_exclude_edge, uint16_t cm_min_exclude_edge, int cm_ave_exclude_edge) {
        (head + index)->cm_max = cm_max;
        (head + index)->cm_min = cm_min;
        (head + index)->cm_ave = cm_ave;
        (head + index)->cm_ave_exclude_edge= cm_ave_exclude_edge;
        (head + index)->cm_max_exclude_edge= cm_max_exclude_edge;
        (head + index)->cm_min_exclude_edge= cm_min_exclude_edge;
}


static int get_cm_uniformity_test_results(int vdda, uint16_t tx_num,
        uint16_t rx_num,  uint16_t button_num, bool skip_cm_button,
        int32_t **sensor_raw_data,
        int **cm_sensor_data,
        int *cm_sensor_average)
{
    union parameter_value parameter_value;
    enum parameter_type parameter_type;
    int *sensor_localpwc = NULL;
    uint16_t sensor_element_num = 0;
    uint8_t data_format = 0;
    int gidac_val = 0;
    int max_gidac = 0;
    int vref = VREF_DEFAULT;
    char* vdda_mode;
    uint32_t scaling_factor_mutual;
    int int_cap_mutual;
    int vtx;
    char *tx_voltage_mutual;
    int idac_lsb = IDAC_LSB_DEFAULT;
    char *idac_lsb_config;
    int mtx_sum = MTX_SUM_DEFAULT;
    uint8_t mtx_oder;
    int clk = CLK_DEFAULT;
    int ret;
    int i;
    int j;

    sensor_element_num = rx_num * tx_num;
    tp_log_info("%s, get_cm_uniformity_test_results called\n", __func__);
    *sensor_raw_data = kzalloc(sensor_element_num * sizeof(int32_t), GFP_KERNEL);
    *cm_sensor_data = kzalloc(sensor_element_num * sizeof(int),GFP_KERNEL);
    sensor_localpwc = kzalloc((sensor_element_num + tx_num)* sizeof(int),GFP_KERNEL);
    if (!*sensor_raw_data || !*cm_sensor_data || !sensor_localpwc) {
        ret = -ENOMEM;
        goto exit;
    }

    tp_log_info("%s: get GIDAC_LSB_CONFIG\n", __func__);

    /* Step 1: get GIDAC_LSB_CONFIG */
    ret = get_configuration_parameter(GIDAC_LSB_CONFIG, &parameter_type,
                &parameter_value);
    if (ret || parameter_type != STRING) {
        tp_log_err("%s:Unable to get GIDAC_LSB_CONFIG!\n",__func__);
        goto restore_multi_tx;
    }
    idac_lsb_config = parameter_value.string;
    sscanf(idac_lsb_config, "%d", &idac_lsb);
    tp_log_info("%s,idac_lsb: %d\n", __func__, idac_lsb);

    /* Step 2: get INT_CAP_MUTUAL */
    tp_log_info("%s: get INT_CAP_MUTUAL\n", __func__);
    ret = get_configuration_parameter(INT_CAP_MUTUAL, &parameter_type,
                    &parameter_value);
    if (ret || parameter_type != INTEGER) {
        tp_log_err("%s:Unable to get INT_CAP_MUTUAL!\n",__func__);
        goto restore_multi_tx;
    }
    int_cap_mutual = parameter_value.integer;
    tp_log_info("%s,int_cap_mutual: %d\n", __func__, int_cap_mutual);

    /* step 3 Get CDC:VDDA_MODE */
    ret = get_configuration_parameter(VDDA_MODE, &parameter_type,
                &parameter_value);
    if (ret || parameter_type != STRING) {
        tp_log_err("%s:Unable to get vdda_mode!\n",__func__);
        goto restore_multi_tx;
    }
    vdda_mode = parameter_value.string;
    tp_log_info("%s: vdda_mode: %s\n", __func__, vdda_mode);

    if (!strcmp(vdda_mode, VDDA_MODE_BYPASS)) {
        if (vdda != 0)
            vtx = vdda;
        else {
            tp_log_err("%s:VDDA cannot be zero when VDDA_MODE is bypass!\n",__func__);
            ret = -EINVAL;
            goto restore_multi_tx;
        }
    } else if (!strcmp(vdda_mode, VDDA_MODE_PUMP)) {
        /* Get CDC:TX_PUMP_VOLTAGE */
        ret = get_configuration_parameter(TX_VOLTAGE_MUTUAL,
                &parameter_type, &parameter_value);
        if (ret || parameter_type != STRING) {
            tp_log_err("%s:Unable to get tx_pump_voltage!\n", __func__);
            goto restore_multi_tx;
        }
        tx_voltage_mutual = parameter_value.string;
        sscanf(tx_voltage_mutual, "%d", &vtx);
    } else {
        tp_log_err("%s:Invalid VDDA_MODE: %s!\n",__func__, vdda_mode);
        ret = -EINVAL;
        goto restore_multi_tx;
    }

    tp_log_info("%s: vtx: %d\n", __func__, vtx);
    /* step 5 Get CDC:SCALING_FACTOR_MUTUAL */
    ret = get_configuration_parameter(SCALING_FACTOR_MUTUAL,
                &parameter_type, &parameter_value);
    if (ret || parameter_type != INTEGER) {
        tp_log_err("%s:Unable to get scaling_factor_mutual!\n", __func__);
        goto restore_multi_tx;
    }

    scaling_factor_mutual = parameter_value.integer;
    tp_log_info("%s: scaling_factor_mutual: %d\n", __func__,
            scaling_factor_mutual);

    /* step 6 Get CDC:MTXSUM */
    ret = get_configuration_parameter(MTX_ORDER,
                &parameter_type, &parameter_value);
    if (ret || parameter_type != INTEGER) {
        tp_log_err("%s:Unable to get mtx_sum!\n", __func__);
        goto restore_multi_tx;
    }

    mtx_oder= parameter_value.integer;
    ret = mtx_sum_lookup(mtx_oder, &mtx_sum);
    if(ret) {
        tp_log_err("%s, Unable to lookup sum\n", __func__);
        goto restore_multi_tx;
    }

    tp_log_info("%s: mtx_sum: %d\n", __func__,
            mtx_sum);


    /* Step 7: Perform calibration */
    ret = pip_calibrate_idacs(0);
    if (ret) {
        tp_log_err("%s:Unable to calibrate IDACs!\n",__func__);
        goto restore_multi_tx;
    }
    if(button_num > 0)
    {
        ret = pip_calibrate_idacs(1);
        if (ret) {
            tp_log_err("%s Unable to calibrate button IDACs!\n",__func__);
            goto restore_multi_tx;
        }
    }
    /*step 8a:resume Panel Scan*/
    ret = pip_resume_scanning();
    if (ret) {
        tp_log_err("%s:Unable to resume panel scan!\n",__func__);
        goto restore_multi_tx;
    }
    /*step 8b:sleep 300ms*/
    mdelay(300);
    /*step 8c:suspend Panel Scan*/
    ret = pip_suspend_scanning();
    if (ret) {
        tp_log_err("%s:Unable to suspend panel scan!\n",__func__);
        goto restore_multi_tx;
    }

    /* Step 8d: Execute Panel Scan */
    ret = pip_execute_panel_scan();
    if (ret) {
        tp_log_err("%s:Unable to execute panel scan!\n",__func__);
        goto restore_multi_tx;
    }

    /* Step 8e: Retrieve Panel Scan raw data */
    ret = retrieve_panel_localpwc(MUTUAL_LOCAL_PWC_DATA_ID, 0,
            sensor_element_num + tx_num, &data_format, sensor_localpwc);
    if (ret) {
        tp_log_err("%s:Unable to retrieve panel localpwc!\n",__func__);
        goto restore_multi_tx;
    }

    tp_log_info("%s: sensor_localpwc[0..%d]:\n", __func__,
            sensor_element_num + tx_num - 1);
    max_gidac = sensor_localpwc[0];
    tp_log_info("global idac:");
    for( i = 0 ; i < tx_num ; i++) {
        if( sensor_localpwc[i] > max_gidac) {
            max_gidac = sensor_localpwc[i];
        }
        printk("%5d", sensor_localpwc[i]);
    }
    printk("\n");

    for( i = 0 ; i < sensor_element_num; i++) {
        printk("%5d", sensor_localpwc[i + tx_num]);
        if((i + 1)%rx_num  == 0)
            printk("\n");
    }


    if((max_gidac % mtx_sum) ==0 ) {
        gidac_val = (max_gidac /mtx_sum);
    } else {
        gidac_val = (max_gidac /mtx_sum)+1;
    }
    if(gidac_val == 0) {
        gidac_val =10;
        tp_log_info("%s: reseting gidac_val from 0 to 10\n", __func__);
    }
    tp_log_info("%s, max global idac:%d\n", __func__, max_gidac);

    /*step 8a:resume Panel Scan*/
    ret = pip_resume_scanning();
    if (ret) {
        tp_log_err("%s:Unable to resume panel scan!\n",__func__);
        goto restore_multi_tx;
    }

    /*step 8b:sleep 300ms*/
    mdelay(300);
    /*step 8c:suspend Panel Scan*/
    ret = pip_suspend_scanning();
    if (ret) {
        tp_log_err("%s:Unable to suspend panel scan!\n",__func__);
        goto restore_multi_tx;
    }

    /* Step 8d: Execute Panel Scan */
    ret = pip_execute_panel_scan();
    if (ret) {
        tp_log_err("%s:Unable to execute panel scan!\n",__func__);
        goto restore_multi_tx;
    }

    /* Step 8e: Retrieve Panel Scan raw data */
    ret = retrieve_panel_raw_data(MUTUAL_CAP_RAW_DATA_ID, 0,
            sensor_element_num, &data_format, *sensor_raw_data);
    if (ret) {
        tp_log_err("%s:Unable to retrieve panel raw data!\n",__func__);
        goto restore_multi_tx;
    }

    tp_log_info("%s: sensor_raw_data[0..%d]:\n", __func__,
            sensor_element_num - 1);
    for (i = 0; i < tx_num; i++) {
        for (j = 0; j < rx_num; j++)
        {
            printk("%5d ",
                (*sensor_raw_data)[i * rx_num + j]);
        }
        printk("\n");
    }

    tp_log_info("gidac_val:%d, mtx_sum:%d, vref:%d\n",gidac_val, mtx_sum, vref);
    tp_log_info("int_cap_mutual:%d, idac_lsb:%d, scaling_factor_mutual:%d, vtx:%d\n",
        int_cap_mutual, idac_lsb, scaling_factor_mutual, vtx);
    tp_log_info("begin calculate_cm\n");
    /* Step 5 and 6: Calculate Cm_sensor and Cm_sensor_ave */
    *cm_sensor_average = 0;

    for (i = 0; i < sensor_element_num; i++) {
        (*cm_sensor_data)[i] = calculate_cm((*sensor_raw_data)[i], sensor_localpwc[tx_num + i], int_cap_mutual,
                              idac_lsb, vref, gidac_val, scaling_factor_mutual ,
                              clk, vtx, mtx_sum);
        *cm_sensor_average += (*cm_sensor_data)[i];
    }
    *cm_sensor_average /= sensor_element_num;

    tp_log_info("%s: cm_sensor_data[0..%d]:\n", __func__,
            sensor_element_num - 1);
    for (i = 0; i < tx_num; i++) {
        for (j = 0; j < rx_num; j++)
        {
            printk("%5d", (*cm_sensor_data)[i * rx_num + j]);
        }
        printk("\n");
    }
    tp_log_info("%s: cm_sensor_average: %d\n", __func__,
            *cm_sensor_average);

    if (!skip_cm_button)   //not support button
        tp_log_err("%s, cm_button not support\n", __func__);

restore_multi_tx:

    /* Step 14: Perform calibration */
    ret = pip_calibrate_idacs(0);
    if (ret) {
        tp_log_err("%s:Unable to calibrate IDACs!\n",__func__);
        goto exit;
    }
    if(button_num > 0)
    {
        ret = pip_calibrate_idacs(1);
        if (ret) {
            tp_log_err("%s:Unable to calibrate button IDACs!\n",__func__);
            goto exit;
        }
    }

exit:
    if(sensor_localpwc) {
        kfree(sensor_localpwc);
        sensor_localpwc = NULL;
    }
    return ret;
}

static int calculate_cp(int sensor_self_rawdata, int sensor_self_localpwc, int int_cap_self, int idac_lsb, int vref,
      int giadc_val, int scaling_factor_self, int clk, int vtx, int mtx_sum)
{
    int qidac = 4 * idac_lsb * giadc_val * sensor_self_localpwc/clk;
    int qint = int_cap_self * vref* sensor_self_rawdata * 100000/scaling_factor_self/2048;
    int cp_sensor = (qidac + qint) / (vtx);

    tp_log_debug("sensor_self_rawdata:%d ,sensor_self_localpwc :%d, qidac :%d, qint : %d, cp_sensor :%d\n",
           sensor_self_rawdata, sensor_self_localpwc,qidac, qint ,cp_sensor);

    return cp_sensor;
}

static int get_cp_uniformity_test_results(int vdda, uint16_t tx_num,
        uint16_t rx_num,  uint16_t button_num, bool skip_cp_button,
        bool skip_cp_sensor,
        int32_t **sensor_tx_raw_data, int32_t **sensor_rx_raw_data,
        int **cp_sensor_tx_data, int **cp_sensor_rx_data,
        int *cp_sensor_average)
{
    union parameter_value parameter_value;
    enum parameter_type parameter_type;
    int *sensor_self_localpwc = NULL;
    uint16_t sensor_element_num;
    uint8_t data_format;
    int gidac_rx_val;
    int gidac_tx_val;
    int vref = VREF_DEFAULT;
    uint32_t scaling_factor_self;
    int int_cap_self;
    int idac_lsb = IDAC_LSB_DEFAULT;
    char *idac_lsb_config;
    int mtx_sum = MTX_SUM_DEFAULT;
    uint8_t mtx_oder;
    int clk = CLK_DEFAULT;
    int ret = 0;
    int rxdac;
    int i;

    if(skip_cp_sensor) {
        tp_log_err("%s,skip_cp_sensor,return now\n", __func__);
        return ret;
    }
    tp_log_info("%s, get_cp_uniformity_test_results called\n", __func__);
    sensor_element_num = rx_num + tx_num;

    *sensor_tx_raw_data = kzalloc(tx_num * sizeof(int32_t), GFP_KERNEL);
    *sensor_rx_raw_data = kzalloc(rx_num * sizeof(int32_t), GFP_KERNEL);

    *cp_sensor_tx_data = kzalloc(tx_num * sizeof(int32_t), GFP_KERNEL);
    *cp_sensor_rx_data = kzalloc(rx_num * sizeof(int32_t), GFP_KERNEL);
    sensor_self_localpwc = kzalloc((sensor_element_num + tx_num)* sizeof(int),GFP_KERNEL);
    if (!(*sensor_tx_raw_data) || !(*sensor_rx_raw_data) || !sensor_self_localpwc ||
        !(*cp_sensor_tx_data) || !(*cp_sensor_tx_data)) {
        ret = -ENOMEM;
        goto exit;
    }

    tp_log_info("%s: get GIDAC_LSB_CONFIG\n", __func__);

    /* Step 1: get GIDAC_LSB_CONFIG */
    ret = get_configuration_parameter(GIDAC_LSB_CONFIG, &parameter_type,
                &parameter_value);
    if (ret || parameter_type != STRING) {
        tp_log_err("%s:Unable to get GIDAC_LSB_CONFIG!\n",__func__);
        goto restore_multi_tx;
    }
    idac_lsb_config = parameter_value.string;
    sscanf(idac_lsb_config, "%d", &idac_lsb);
    tp_log_info("%s,idac_lsb: %d\n", __func__, idac_lsb);

    /* Step 2: get INT_CAP_MUTUAL */
    tp_log_info("%s: get INT_CAP_SELF\n", __func__);

    ret = get_configuration_parameter(INT_CAP_SELF, &parameter_type,
                    &parameter_value);
    if (ret || parameter_type != INTEGER) {
        tp_log_err("%s:Unable to get INT_CAP_SELF!\n",__func__);
        goto restore_multi_tx;
    }
    int_cap_self = parameter_value.integer;
    tp_log_info("%s,int_cap_self: %d\n", __func__, int_cap_self);

    ret = get_configuration_parameter(RXDAC,
                &parameter_type, &parameter_value);
    if (ret || parameter_type != INTEGER) {
        tp_log_err("%s:Unable to get rxdac!\n", __func__);
        goto restore_multi_tx;
    }

    rxdac= parameter_value.integer + 5;//according mfg_test . 1.5*10
    tp_log_info("%s: rxdac: %d\n", __func__,
            rxdac);

    /* step 5 Get CDC:SCALING_FACTOR_MUTUAL */
    ret = get_configuration_parameter(SCALING_FACTOR_SELF,
                &parameter_type, &parameter_value);
    if (ret || parameter_type != INTEGER) {
        tp_log_err("%s:Unable to get scaling_factor_self!\n", __func__);
        goto restore_multi_tx;
    }

    scaling_factor_self = parameter_value.integer;
    tp_log_info("%s: scaling_factor_self: %d\n", __func__,
            scaling_factor_self);

    /* step 6 Get CDC:MTXSUM */
    ret = get_configuration_parameter(MTX_ORDER,
                &parameter_type, &parameter_value);
    if (ret || parameter_type != INTEGER) {
        tp_log_err("%s:Unable to get mtx_sum!\n", __func__);
        goto restore_multi_tx;
    }

    mtx_oder= parameter_value.integer;
    ret = mtx_sum_lookup(mtx_oder, &mtx_sum);
    if(ret) {
        tp_log_err("%s, Unable to lookup sum\n", __func__);
        goto restore_multi_tx;
    }
    tp_log_info("%s: mtx_sum: %d\n", __func__, mtx_sum);

    /* Step 7: Perform calibration */
    ret = pip_calibrate_idacs(0);
    if (ret) {
        tp_log_err("%s:Unable to calibrate IDACs!\n",__func__);
        goto restore_multi_tx;
    }
    if(button_num > 0)
    {
        ret = pip_calibrate_idacs(1);
        if (ret) {
            tp_log_err("%s Unable to calibrate button IDACs!\n",__func__);
            goto restore_multi_tx;
        }
    }

    /*step 8a:resume Panel Scan*/
    ret = pip_resume_scanning();
    if (ret) {
        tp_log_err("%s:Unable to resume panel scan!\n",__func__);
        goto restore_multi_tx;
    }

    /*step 8b:sleep 300ms*/
    mdelay(300);
    /*step 8c:suspend Panel Scan*/
    ret = pip_suspend_scanning();
    if (ret) {
        tp_log_err("%s:Unable to suspend panel scan!\n",__func__);
        goto restore_multi_tx;
    }

    /* Step 8d: Execute Panel Scan */
    ret = pip_execute_panel_scan();
    if (ret) {
        tp_log_err("%s:Unable to execute panel scan!\n",__func__);
        goto restore_multi_tx;
    }

    /* Step 8e: Retrieve Panel Scan raw data */
    ret = retrieve_panel_raw_data(SELF_CAP_RAW_DATA_ID, 0,
            rx_num, &data_format, *sensor_rx_raw_data);
    if (ret) {
        tp_log_err("%s:Unable to retrieve panel rx raw data!\n",__func__);
        goto restore_multi_tx;
    }

    ret = retrieve_panel_raw_data(SELF_CAP_RAW_DATA_ID, rx_num,
            tx_num, &data_format, *sensor_tx_raw_data);
    if (ret) {
        tp_log_err("%s:Unable to retrieve panel tx raw data!\n",__func__);
        goto restore_multi_tx;
    }

    tp_log_info("%s: sensor_self raw_data[0..%d]:\n", __func__,
            sensor_element_num - 1);
    for (i = 0; i < rx_num; i++)
        printk("%6d", (*sensor_rx_raw_data)[i]);
    printk("\n");
    for (i = 0; i < tx_num; i++)
        printk("%6d", (*sensor_tx_raw_data)[i]);
    printk("\n");

    /*step 8a:resume Panel Scan*/
    ret = pip_resume_scanning();
    if (ret) {
        tp_log_err("%s:Unable to resume panel scan!\n",__func__);
        goto restore_multi_tx;
    }

    /*step 8b:sleep 300ms*/
    mdelay(300);
    /*step 8c:suspend Panel Scan*/
    ret = pip_suspend_scanning();
    if (ret) {
        tp_log_err("%s:Unable to suspend panel scan!\n",__func__);
        goto restore_multi_tx;
    }

    /* Step 8d: Execute Panel Scan */
    ret = pip_execute_panel_scan();
    if (ret) {
        tp_log_err("%s:Unable to execute panel scan!\n",__func__);
        goto restore_multi_tx;
    }

    /* Step 8e: Retrieve Panel Scan raw data */
    ret = retrieve_panel_localpwc(SELF_LOCAL_PWC_DATA_ID, 0,
            sensor_element_num  + 2, &data_format, sensor_self_localpwc);
    if (ret) {
        tp_log_err("%s:Unable to retrieve panel localpwc!\n",__func__);
        goto restore_multi_tx;
    }

    tp_log_info("%s: sensor_self_localpwc[0..%d]:\n", __func__,
            sensor_element_num + 2 - 1);
    printk("RX_idac:%d, TX_idac:%d\n", sensor_self_localpwc[0], sensor_self_localpwc[1]);
    for( i = 2 ; i < sensor_element_num + 2; i++) {
        printk("%6d", sensor_self_localpwc[i]);
        if((i - 1)%rx_num  == 0)
            printk("\n");
    }

    gidac_rx_val = sensor_self_localpwc[0];
    gidac_tx_val = sensor_self_localpwc[1];
    tp_log_info("mtx_sum:%d, vref:%d\n", mtx_sum, vref);
    tp_log_info("int_cap_self:%d, idac_lsb:%d, scaling_factor_self:%d, vtx:%d\n",
        int_cap_self, idac_lsb, scaling_factor_self, rxdac);
    tp_log_info("begin calculate_cp\n");
    /* Step 5 and 6: Calculate Cm_sensor and Cm_sensor_ave */
    *cp_sensor_average = 0;
    for (i = 0; i < rx_num; i++) {
        (*cp_sensor_rx_data)[i] = calculate_cp((*sensor_rx_raw_data)[i], sensor_self_localpwc[2 + i], int_cap_self,
                              idac_lsb, vref, gidac_rx_val, scaling_factor_self ,
                              clk, rxdac, mtx_sum);
        *cp_sensor_average += (*cp_sensor_rx_data)[i];
    }
    for (i = 0; i < tx_num; i++) {
        (*cp_sensor_tx_data)[i] = calculate_cp((*sensor_tx_raw_data)[i], sensor_self_localpwc[2 + rx_num + i], int_cap_self,
                              idac_lsb, vref, gidac_tx_val, scaling_factor_self ,
                              clk, rxdac, mtx_sum);
        *cp_sensor_average += (*cp_sensor_tx_data)[i];
    }

    *cp_sensor_average /= sensor_element_num;
    tp_log_info("%s: cp_sensor_data[0..%d]:\n", __func__,
            sensor_element_num - 1);
    for (i = 0; i < rx_num; i++)
        printk("%6d", (*cp_sensor_rx_data)[i]);
        printk("\n");
    for (i = 0; i < tx_num; i++)
        printk("%6d", (*cp_sensor_tx_data)[i]);
    printk("\n");

    tp_log_info("%s: cp_sensor_average: %d\n", __func__,
            *cp_sensor_average);

    if (!skip_cp_button)   //not support button
        tp_log_err("%s, cp_button not support\n", __func__);

restore_multi_tx:
    /* Step 14: Perform calibration */
    ret = pip_calibrate_idacs(0);
    if (ret) {
        tp_log_err("%s:Unable to calibrate IDACs!\n",__func__);
        goto exit;
    }
    if(button_num > 0)
    {
        ret = pip_calibrate_idacs(1);
        if (ret) {
            tp_log_err("%s:Unable to calibrate button IDACs!\n",__func__);
            goto exit;
        }
    }

exit:
    if(sensor_self_localpwc) {
        kfree(sensor_self_localpwc);
        sensor_self_localpwc = NULL;
    }
    return ret;
}

static int get_extra_parameter(uint32_t* sensor_assignment,uint32_t * config_ver,uint32_t * revision_ctrl,uint32_t * device_id_high,uint32_t * device_id_low){
    union parameter_value parameter_value;
    enum parameter_type parameter_type;
    struct pip_sysinfo sysinfo;
    int i;
    int ret;
    *device_id_high=0;
    *device_id_low=0;
    /* Get Device Setup:SENSOR_ASSIGNMENT */
    ret = get_configuration_parameter(SENSOR_ASSIGNMENT, &parameter_type,
                &parameter_value);
    if (ret || parameter_type != STRING) {
        tp_log_err("%s:Unable to get sensor assignment!\n",__func__);
        goto exit;
    }
    *sensor_assignment = !strcmp(parameter_value.string,RX_IS_Y);
    tp_log_info("%s: sensor_assignment: %d\n", __func__, *sensor_assignment);

    /* Get config version and revision control version */
    ret=pip_get_system_information(&sysinfo);
    if(ret){
        tp_log_err("%s:Unable to get system information!\n",__func__);
        goto exit;
    }
    *revision_ctrl=sysinfo.cy_data.fw_revctrl;
    *config_ver =sysinfo.cy_data.fw_ver_conf;
    tp_log_info("%s: config_ver: %d\n", __func__, *config_ver);
    tp_log_info("%s: revision_ctrl: %d\n", __func__, *revision_ctrl);
    for(i=0;i<4;i++){
        *device_id_low=(*device_id_low<<8)+sysinfo.cy_data.mfg_id[i];
    }
    for(i=4;i<8;i++){
        *device_id_high=(*device_id_high<<8)+sysinfo.cy_data.mfg_id[i];
    }
    tp_log_info("%s: device_id_low: 0x%x\n", __func__, *device_id_low);
    tp_log_info("%s: device_id_high: 0x%x\n", __func__, *device_id_high);
    return 0;
exit:
    return ret;
}
static int check_tests(uint32_t *tx_num, uint32_t *rx_num, uint32_t *button_num,
        bool *skip_cm_button, bool *skip_cp_sensor,
        bool *skip_cp_button)
{
    union parameter_value parameter_value;
    enum parameter_type parameter_type;
    uint32_t act_lft_en_enabled;
    uint32_t bl_h2o_rjct_enabled;
    char *scanning_mode_button;
    int ret;

    /* Get CDC:TX_NUM */
    ret = get_configuration_parameter(TX_NUM, &parameter_type,
                &parameter_value);
    if (ret || parameter_type != INTEGER) {
        tp_log_err("%s:Unable to get tx_num!\n",__func__);
        goto exit;
    }

    *tx_num = parameter_value.integer;
    tp_log_info("%s: tx_num: %d\n", __func__, *tx_num);

    /* Get CDC:RX_NUM */
    ret = get_configuration_parameter(RX_NUM, &parameter_type,
                &parameter_value);
    if (ret || parameter_type != INTEGER) {
        tp_log_err("%s:Unable to get rx_num!\n",__func__);
        goto exit;
    }

    *rx_num = parameter_value.integer;
    tp_log_info("%s: rx_num: %d\n", __func__, *rx_num);

    /* Get CDC:BUTTON_NUM */
    ret = get_configuration_parameter(BUTTON_NUM, &parameter_type,
                &parameter_value);
    if (ret || parameter_type != INTEGER) {
        tp_log_err("%s:Unable to get button_num!\n",__func__);
        goto exit;
    }

    *button_num = parameter_value.integer;
    tp_log_info("%s: button_num: %d\n", __func__, *button_num);

    /* Get FingerTracking:ACT_LFT_EN */
    ret = get_configuration_parameter(ACT_LFT_EN, &parameter_type,
                &parameter_value);
    if (ret || parameter_type != STRING) {
        tp_log_err("%s:Unable to get act_lft_en!\n",__func__);
        goto exit;
    }

    act_lft_en_enabled = !strcmp(parameter_value.string,
                ACT_LFT_EN_ENABLED);
    tp_log_info("%s: act_lft_en: %d\n", __func__, act_lft_en_enabled);

    /* Get ScanFiltering:BL_H20_RJCT */
    ret = get_configuration_parameter(BL_H20_RJCT, &parameter_type,
                &parameter_value);
    if (ret || parameter_type != STRING) {
        tp_log_err("%s:Unable to get bl_h2o_rjct!\n",__func__);
        goto exit;
    }

    bl_h2o_rjct_enabled = !strcmp(parameter_value.string,
                BL_H20_RJCT_ENABLED);
    tp_log_info("%s: bl_h2o_rjct: %d\n", __func__, bl_h2o_rjct_enabled);

    /* Get CDC:SCANNING_MODE_BUTTON */
    ret = get_configuration_parameter(SCANNING_MODE_BUTTON,
                &parameter_type, &parameter_value);
    if (ret < 0|| parameter_type != STRING) {
        tp_log_err("%s:Unable to get scanning_mode_button! ret = %d\n",__func__, ret);
        goto exit;
    }
    scanning_mode_button = parameter_value.string;
    tp_log_info("%s: scanning_mode_button: %s\n", __func__,
            scanning_mode_button);

    *skip_cm_button = false;
    *skip_cp_button = false;
    *skip_cp_sensor = false;

    if (*button_num == 0) {
        *skip_cm_button = true;
        *skip_cp_button = true;
    } else if (!strcmp(scanning_mode_button,
                SCANNING_MODE_BUTTON_MUTUAL))
        *skip_cp_button = true;
    else if (!strcmp(scanning_mode_button, SCANNING_MODE_BUTTON_SELF))
        *skip_cm_button = true;

    if (!act_lft_en_enabled && !bl_h2o_rjct_enabled)
        *skip_cp_sensor = true;

exit:
    return ret;
}

static int validate_cm_test_results(struct configuration *configuration,uint32_t button_num,
        struct result *result, uint32_t tx_num, uint32_t rx_num, bool skip_cm_button,
        int *cm_sensor_data, int sensor_assignment,
        struct gd_sensor **gd_sensor_col, struct gd_sensor **gd_sensor_row,
        bool *pass)
{
    int ret = 0;
    int index = 0;
    int i, j;
    uint32_t configuration_sensor_num;
    uint32_t configuration_button_num;
    int sensor_data_min;
    int sensor_data_max;
    int sensor_num;
    int cm_max ;
    int cm_min ;
    int cm_ave ;
    int cm_max_exclude_edge ;
    int cm_min_exclude_edge ;
    int cm_ave_exclude_edge ;
    int cm_data ;

    if(!configuration || !result || !cm_sensor_data ) {
        tp_log_err("%s, param invalid\n", __func__);
        ret = -EINVAL;
        goto exit;
    }
    sensor_num = tx_num *rx_num;
    tp_log_info("%s, sensor_num= %d\n", __func__, sensor_num);
    configuration_sensor_num =
                configuration->cm_min_max_table_sensor_size / 2;
    configuration_button_num =
                configuration->cm_min_max_table_button_size / 2;
    tp_log_debug("%s, configuration_sensor_num= %d\n", __func__, configuration_sensor_num);
    if (configuration_sensor_num != sensor_num) {
        tp_log_err("%s: Configuration and Device number of sensors mismatch! (Configuration:%d, Device:%d)\n",
            __func__, configuration_sensor_num,
                sensor_num);
            ret = -EINVAL;
            goto exit;
    }

    if (!skip_cm_button && (button_num != configuration_button_num)) {
        tp_log_err("%s: Configuration and Device number of buttons mismatch! (Configuration:%d, Device:%d)\n",
            __func__, configuration_button_num,
                button_num);
            ret = -EINVAL;
            goto exit;
    }

    /*max min test*/
    result->cm_sensor_validation_pass = true;
    for(i = 0; i < tx_num; i++) {
        for(j = 0; j < rx_num; j++) {
            index = i * rx_num + j;
            sensor_data_min = configuration->cm_min_max_table_sensor[2 * index];
            sensor_data_max = configuration->cm_min_max_table_sensor[2 * index + 1];
            if(cm_sensor_data[index] < sensor_data_min||
                cm_sensor_data[index] > sensor_data_max) {
                tp_log_err("%s, cm_sensor min_max failed,sensor[%d][%d] = %d, overflow [%d, %d]\n",
                    __func__ , i, j, cm_sensor_data[index], sensor_data_min, sensor_data_max);
                result->cm_sensor_validation_pass = false;
            }
        }
    }

    tp_log_info("cm_sensor_validation_pass: %d\n", result->cm_sensor_validation_pass);

    *gd_sensor_col = (struct gd_sensor *)kzalloc(tx_num * sizeof(struct gd_sensor), GFP_KERNEL);
    if(*gd_sensor_col == NULL) {
        tp_log_err("failed to malloc for gd_sensor_col\n");
        goto exit;
    }
    *gd_sensor_row =  (struct gd_sensor *)kzalloc(rx_num * sizeof(struct gd_sensor), GFP_KERNEL);
    if(*gd_sensor_row == NULL) {
        tp_log_err("failed to malloc for gd_sensor_row\n");
        goto exit;
    }
    for (j = 0; j < rx_num; j++) {
        /*re-initialize for a new row*/
        cm_max = (cm_sensor_data)[j];
        cm_min =  cm_max;
        cm_ave = 0;
        cm_max_exclude_edge = (cm_sensor_data)[rx_num + j];
        cm_min_exclude_edge =  cm_max_exclude_edge;
        cm_ave_exclude_edge = 0;
        for (i = 0; i < tx_num; i++) {
            cm_data = (cm_sensor_data)[i * rx_num + j];
            if (cm_data > cm_max)
                cm_max = cm_data;
            if ( cm_data < cm_min)
                cm_min = cm_data;
            cm_ave += cm_data;
            /*calculate exclude edge data*/
            if((i >  0) && (i < (tx_num-1))) {
                if (cm_data > cm_max_exclude_edge)
                    cm_max_exclude_edge = cm_data;
                if ( cm_data < cm_min_exclude_edge)
                    cm_min_exclude_edge = cm_data;
                cm_ave_exclude_edge += cm_data;
            }
        }
        cm_ave /= tx_num;
        cm_ave_exclude_edge /= (tx_num-2);
        fill_gd_sensor_table(*gd_sensor_row, j, cm_max, cm_min, cm_ave,
            cm_max_exclude_edge, cm_min_exclude_edge, cm_ave_exclude_edge);
    }

    result->cm_sensor_row_delta_pass = true;
    calculate_gradient_row(*gd_sensor_row, rx_num,
            configuration->cm_excluding_row_edge, configuration->cm_excluding_col_edge);
    for (i = 0;i < configuration->cm_gradient_check_row_size; i++) {
        if((*gd_sensor_row + i)->gradient_val > configuration->cm_gradient_check_row[i]) {
            tp_log_err("%s cm_max_table_gradient_row_percent[%d]:%d, gradient_val:%d\n", __func__, i, configuration->cm_gradient_check_row[i], (*gd_sensor_row + i)->gradient_val );
            tp_log_debug("%s:row[%d] cm_max:%d cm_min:%d cm_ave:%d cm_max_exclude_edge:%d  cm_min_exclude_edge:%d cm_ave_exclude_edge:%d gradient_val=%d \n",
            __func__, i,
            (*gd_sensor_row + i)->cm_max,
            (*gd_sensor_row + i)->cm_min,
            (*gd_sensor_row + i)->cm_ave,
            (*gd_sensor_row + i)->cm_max_exclude_edge,
            (*gd_sensor_row + i)->cm_min_exclude_edge,
            (*gd_sensor_row + i)->cm_ave_exclude_edge,
            (*gd_sensor_row + i)->gradient_val);
            result->cm_sensor_row_delta_pass = false;
        }
    }
    printk("gd_sensor_row:\n");
    for (i = 0;i < configuration->cm_gradient_check_row_size; i++) {
        printk("%6d", (*gd_sensor_row + i)->gradient_val);
    }
    printk("\n");

    for (i = 0; i < tx_num; i++) {
        /*re-initialize for a new col*/
        cm_max = (cm_sensor_data)[i * rx_num];
        cm_min = cm_max;
        cm_ave = 0;
        cm_max_exclude_edge = (cm_sensor_data)[i * rx_num + 1];
        cm_min_exclude_edge = cm_max_exclude_edge;
        cm_ave_exclude_edge = 0;
        for (j = 0; j < rx_num; j++) {
            cm_data = (cm_sensor_data)[i * rx_num + j];
            if (cm_data > cm_max)
                cm_max = cm_data;
            if ( cm_data < cm_min)
                cm_min = cm_data;
            cm_ave += cm_data;
            /*calculate exclude edge data*/
            if((j > 0) && (j < (rx_num-1))) {
                if (cm_data > cm_max_exclude_edge)
                    cm_max_exclude_edge = cm_data;
                if ( cm_data < cm_min_exclude_edge)
                    cm_min_exclude_edge = cm_data;
                cm_ave_exclude_edge += cm_data;
            }
        }
        cm_ave /= rx_num;
        cm_ave_exclude_edge /= (rx_num-2);
        fill_gd_sensor_table(*gd_sensor_col, i, cm_max, cm_min, cm_ave,
            cm_max_exclude_edge, cm_min_exclude_edge, cm_ave_exclude_edge);
    }
    result->cm_sensor_col_delta_pass = true;
    calculate_gradient_col(*gd_sensor_col, tx_num,
            configuration->cm_excluding_row_edge, configuration->cm_excluding_col_edge);
    for (i = 0;i < configuration->cm_gradient_check_col_size; i++) {
        if((*gd_sensor_col + i)->gradient_val > configuration->cm_gradient_check_col[i]) {
            tp_log_err("%s cm_max_table_gradient_col_percent[%d]:%d, gradient_val:%d\n", __func__, i, configuration->cm_gradient_check_col[i], (*gd_sensor_col + i)->gradient_val );
            tp_log_debug("%s:col[%d] cm_max:%d cm_min:%d cm_ave:%d cm_max_exclude_edge:%d  cm_min_exclude_edge:%d cm_ave_exclude_edge:%d gradient_val=%d \n",
            __func__, i,
            (*gd_sensor_col + i)->cm_max,
            (*gd_sensor_col + i)->cm_min,
            (*gd_sensor_col + i)->cm_ave,
            (*gd_sensor_col + i)->cm_max_exclude_edge,
            (*gd_sensor_col + i)->cm_min_exclude_edge,
            (*gd_sensor_col + i)->cm_ave_exclude_edge,
            (*gd_sensor_col + i)->gradient_val);
            result->cm_sensor_col_delta_pass = false;
        }
    }
    printk("gd_sensor_col:\n");
    for (i = 0;i < configuration->cm_gradient_check_col_size; i++) {
        printk("%6d", (*gd_sensor_col + i)->gradient_val);
    }
    printk("\n");

    result->cm_test_pass = result->cm_sensor_validation_pass &&
                           result->cm_sensor_col_delta_pass &&
                           result->cm_sensor_row_delta_pass;

    *pass = result->cm_test_pass;
    tp_log_info("cm_test:%d, result cm_test:%d\n", *pass, result->cm_test_pass);
    if(!skip_cm_button)
        goto exit;
exit:
    return ret;
}

static int validate_cp_test_results(struct configuration* configuration, struct result* result,
                int tx_num, int rx_num, int button_num,
                bool skip_cp_sensor, bool skip_cp_button,
                int *cp_sensor_rx_data, int *cp_sensor_tx_data,
                bool *cp_test_pass)

{
    int ret = 0;
    int i;
    uint32_t configuration_rx_sensor_num;
    uint32_t configuration_tx_sensor_num;
    uint32_t configuration_button_num;
    int max_rx_sensor, min_rx_sensor;
    int max_tx_sensor, min_tx_sensor;
    int sensor_num = 0;

    if(!configuration || !result || !cp_sensor_rx_data ||!cp_sensor_tx_data) {
        tp_log_err("%s, param invalid\n", __func__);
        ret = -EINVAL;
        goto exit;
    }
    if(skip_cp_sensor) {
        result->cp_rx_validation_pass = true;
        result->cp_rx_validation_pass = true;
        tp_log_info("%s, skip cp sensor,return pass\n", __func__);
        goto exit;
    }
    sensor_num = tx_num  + rx_num;
    tp_log_info("%s, sensor_num= %d\n", __func__, sensor_num);
    configuration_rx_sensor_num =
                configuration->cp_min_max_table_rx_size / 2;
    configuration_tx_sensor_num =
                configuration->cp_min_max_table_tx_size / 2;
    configuration_button_num =
                configuration->cm_min_max_table_button_size / 2;
    tp_log_info("%s, configuration_rx_num:%d , tx_num: %d\n",
            __func__, configuration_rx_sensor_num, configuration_tx_sensor_num);
    if (configuration_rx_sensor_num != rx_num || configuration_tx_sensor_num != tx_num) {
        tp_log_err("%s: Configuration and Device number of sensors mismatch!\n", __func__);
        tp_log_err("%s:configuration:rx:%d, tx:%d Device:rx:%d, tx:%d\n",
            __func__, configuration_rx_sensor_num , configuration_tx_sensor_num,
                rx_num, tx_num );
        ret = -EINVAL;
        goto exit;
    }
    if (!skip_cp_button && (button_num != configuration_button_num)) {
        tp_log_err("%s: Configuration and Device number of buttons mismatch! (Configuration:%d, Device:%d)\n",
            __func__, configuration_button_num,
                button_num);
        ret = -EINVAL;
        goto exit;
    }

    /*rx min_max check*/
    result->cp_rx_validation_pass = true;
    for(i = 0; i < rx_num; i++){
        min_rx_sensor = configuration->cp_min_max_table_rx[2*i];
        max_rx_sensor = configuration->cp_min_max_table_rx[2*i + 1];
        if(cp_sensor_rx_data[i] > max_rx_sensor ||
            cp_sensor_rx_data[i] < min_rx_sensor) {
            tp_log_err("%s, rx[%d],cp_data[%d],over range[%d, %d]\n",
                __func__, i, cp_sensor_rx_data[i],
                min_rx_sensor, max_rx_sensor);
            result->cp_rx_validation_pass = false;
        }
    }

    /*tx min_max check*/
    result->cp_tx_validation_pass = true;
    for(i = 0; i < tx_num; i++){
        min_tx_sensor = configuration->cp_min_max_table_tx[2*i];
        max_tx_sensor = configuration->cp_min_max_table_tx[2*i + 1];
        if(cp_sensor_tx_data[i] > max_tx_sensor ||
            cp_sensor_tx_data[i] < min_tx_sensor) {
            tp_log_err("%s, tx[%d],cp_data[%d],over range[%d, %d]\n",
                __func__, i, cp_sensor_tx_data[i],
                min_tx_sensor, max_tx_sensor);
            result->cp_tx_validation_pass = false;
        }
    }
    if(!skip_cp_button)
        tp_log_err("%s, not support cp_button\n", __func__);
exit:
    result->cp_test_pass = result->cp_tx_validation_pass &&
                           result->cp_rx_validation_pass;
    *cp_test_pass = result->cp_test_pass;
    tp_log_info("cp_test:%d\n", *cp_test_pass);
    return ret;
}

int cm_cp_test_run(char *device_path, struct file *parameter_file,
        struct file *configuration_file, struct seq_file *result_file, int vdda,
        bool run_cm_test, bool run_cp_test, bool *cm_test_pass,
        bool *cp_test_pass)
{
    struct configuration *configuration;
    struct result *result;
    uint32_t tx_num, rx_num, button_num;
    uint32_t sensor_assignment=0;//if tx is column, then sensor_assignment is 1;
    uint32_t config_ver,revision_ctrl,device_id_high,device_id_low;
    bool skip_cm_button, skip_cp_button, skip_cp_sensor;
    int32_t *cm_sensor_raw_data = NULL;
    int *cm_sensor_data = NULL;
    struct gd_sensor *cm_gradient_col = NULL, *cm_gradient_row = NULL;
    int cm_sensor_average;
    int32_t *cp_sensor_rx_raw_data = NULL, *cp_sensor_tx_raw_data = NULL;
    int *cp_sensor_rx_data = NULL, *cp_sensor_tx_data = NULL;
    int cp_sensor_average;
    int ret = 0;

    tp_log_info("%s: begin\n" ,__func__ );
    configuration = kzalloc(sizeof(struct configuration), GFP_KERNEL);
    result = kzalloc(sizeof(struct result), GFP_KERNEL);
    if (!configuration || !result) {
        ret = -ENOMEM;
        tp_log_err("%s: malloc configuration result fail\n" ,__func__ );
        goto exit;
    }

    memset(configuration, 0, sizeof(struct configuration));
    memset(result, 0, sizeof(struct result));

    ret = parameter_init(parameter_file);
    if (ret) {
        tp_log_err("%s: Fail initing parameters!\n" ,__func__ );
        goto exit;
    }

    ret = configuration_get(configuration_file, configuration);
    if (ret) {
        tp_log_err("%s: Fail getting configuration\n" ,__func__ );
        goto parameter_exit;
    }

    ret = pip_init(device_path, HID_DESCRIPTOR_REGISTER);
    if (ret) {
        tp_log_err("%s: Unable to init pip!\n" ,__func__ );
        goto parameter_exit;
    }

    /* Suspend scanning */
    ret = pip_suspend_scanning();
    if (ret) {
        tp_log_err("%s: Unable to suspend scanning!\n" ,__func__ );
        goto pip_exit;
    }

    ret = check_tests(&tx_num, &rx_num, &button_num,
            &skip_cm_button, &skip_cp_sensor, &skip_cp_button);
    if (ret) {
        tp_log_err("%s: Unable to check_tests !\n" ,__func__ );
        goto resume_scanning;
    }
    /*add get_extra_parameter item*/
    ret=get_extra_parameter(&sensor_assignment,&config_ver,&revision_ctrl,&device_id_high,&device_id_low);
    if (ret) {
        tp_log_err("%s:Unable to get_extra_parameter\n",__func__);
    }
    else{
        tp_log_info("%s: get_extra_parameter pass\n", __func__);
        result->sensor_assignment=sensor_assignment;
        result->config_ver=config_ver;
        result->revision_ctrl=revision_ctrl;
        result->device_id_high=device_id_high;
        result->device_id_low=device_id_low;
    }
    /*add short test item*/
    result->short_test_pass=true;
    ret = pip_short_test();
    if (ret) {
        result->short_test_pass=false;
        tp_log_err("%s:Unable to do short test\n",__func__);
        goto resume_scanning;
    }
    else{
        tp_log_info("%s: short tets pass\n", __func__);
    }
    if (run_cp_test && skip_cp_sensor)
        tp_log_info("%s:Cp sensor test is skipped\n",__func__);
    if (run_cp_test && skip_cp_button)
        tp_log_info("%s:Cp button test is skipped\n",__func__);
    if (run_cm_test && skip_cm_button)
        tp_log_info("%s:Cm button test is skipped\n",__func__);

    if (run_cm_test) {
        ret = get_cm_uniformity_test_results(vdda, tx_num, rx_num,
                button_num, skip_cm_button,
                &cm_sensor_raw_data,
                &cm_sensor_data,
                &cm_sensor_average);
        if (ret) {
            tp_log_err("%s:Unable to run Cm uniformity test!\n",__func__);
            goto err_get_cm_test_results;
        }

        ret = validate_cm_test_results(configuration,button_num,result,
            tx_num,rx_num,skip_cm_button,cm_sensor_data,sensor_assignment,
            &cm_gradient_col, &cm_gradient_row,
            cm_test_pass);
        if (ret) {
            tp_log_err("%s:Unable to validate_cm_test_results!\n",__func__);
            goto err_validate_cm_test_results;
        }

        result->cm_sensor_data = cm_sensor_data;
        result->cm_sensor_raw_data = cm_sensor_raw_data;
        result->cm_gradient_col = cm_gradient_col;
        result->cm_gradient_row = cm_gradient_row;
    }
    if (run_cp_test) {
        ret = get_cp_uniformity_test_results(vdda,tx_num,
        rx_num, button_num, skip_cp_button,
        skip_cp_sensor,
        &cp_sensor_tx_raw_data, &cp_sensor_rx_raw_data,
        &cp_sensor_tx_data, &cp_sensor_rx_data,
        &cp_sensor_average);
        if (ret) {
            tp_log_err("%s:Unable to run Cp uniformity check test!\n",__func__);
            goto err_get_cp_test_results;
        }

        if (!skip_cp_sensor) {
            result->cp_sensor_rx_data = cp_sensor_rx_data;
            result->cp_sensor_tx_data = cp_sensor_tx_data;
            result->cp_sensor_rx_raw_data = cp_sensor_rx_raw_data;
            result->cp_sensor_tx_raw_data = cp_sensor_tx_raw_data;
            result->cp_sensor_rx_average =
                    cp_sensor_average;
            result->cp_sensor_tx_average =
                    cp_sensor_average;
        }
        ret = validate_cp_test_results(configuration, result, tx_num,
                rx_num, button_num, skip_cp_sensor,
                skip_cp_button, cp_sensor_rx_data,
                cp_sensor_tx_data,
                cp_test_pass);
        if (ret) {
            tp_log_err("%s:Fail validating Cp test results!\n",__func__);
            goto free_buffers;
        }
    }


    result->tx_num = tx_num;
    result->rx_num = rx_num;
    result->button_num = button_num;

    result->cm_test_run = run_cm_test;
    result->cp_test_run = run_cp_test;
    result->test_summary= result->cm_test_pass && result->cp_test_pass;

    tp_log_info("%s,get over\n", __func__);

    ret = result_save(result_file, configuration, result);
    if (ret) {
        tp_log_err("%s:Fail saving result\n",__func__);
        goto free_buffers;
    }

free_buffers:
err_get_cp_test_results:

    if (run_cp_test) {
        if (!skip_cp_sensor) {
            kfree(cp_sensor_rx_raw_data);
            cp_sensor_rx_raw_data = NULL;

            kfree(cp_sensor_tx_raw_data);
            cp_sensor_tx_raw_data = NULL;

            kfree(cp_sensor_rx_data);
            cp_sensor_rx_data = NULL;

            kfree(cp_sensor_tx_data);
            cp_sensor_tx_data = NULL;
        }
    }
err_validate_cm_test_results:
err_get_cm_test_results:
    if (run_cm_test) {
        kfree(cm_sensor_raw_data);
        cm_sensor_raw_data = NULL;

        kfree(cm_sensor_data);
        cm_sensor_data = NULL;

        kfree(cm_gradient_col);
        cm_gradient_col = NULL;

        kfree(cm_gradient_row);
        cm_gradient_row = NULL;
    }

resume_scanning:
    if (pip_resume_scanning()) {
        tp_log_err("%s:Unable to resume scanning!\n",__func__);
        goto pip_exit;
    }

pip_exit:
    pip_exit();

parameter_exit:
    parameter_exit();

exit:
    kfree(result);
    result = NULL;

    kfree(configuration);
    configuration = NULL;

    return ret;
}
