#ifndef _MACH_MSM_QUP_H
#define _MACH_MSM_QUP_H

struct msm_qup_i2c_platform_data {
        int clk_freq;
        uint32_t rmutex;
        const char *rsl_id;
        uint32_t pm_lat;
        int pri_clk;
        int pri_dat;
        int aux_clk;
        int aux_dat;
        void (*msm_i2c_config_gpio)(int iface, int config_type);
};

#endif
