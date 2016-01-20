/* CwMcuSensor.c - driver file for HTC SensorHUB
 *
 * Copyright (C) 2014 HTC Ltd.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */
#if 0
#include <linux/vibtrig.h>
#endif

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>

#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/pm.h>
#include <linux/pm_runtime.h>
#include <linux/CwMcuSensor.h>
#include <linux/CwMcuSensor_bus.h>
#include <linux/gpio.h>
#include <linux/wakelock.h>
#ifdef CONFIG_OF
#include <linux/of_gpio.h>
#endif
#include <linux/mutex.h>
#include <linux/spi/spi.h>
#include <linux/workqueue.h>

#include <linux/regulator/consumer.h>

#include <linux/firmware.h>

#include <linux/notifier.h>

#include <linux/sensor_hub.h>

#include <linux/iio/buffer.h>
#include <linux/iio/iio.h>
#include <linux/iio/sysfs.h>
#include <linux/iio/trigger.h>
#include <linux/iio/trigger_consumer.h>
#include <linux/iio/kfifo_buf.h>
#include <linux/irq_work.h>
#include <linux/rtc.h>
#include <linux/ioctl.h>
#include <linux/shub_ctrl.h>
#include <linux/completion.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#if 0
#include <linux/htc_flags.h>
#endif

#ifdef CONFIG_FB
#include <linux/notifier.h>
#include <linux/fb.h>
#endif //CONFIG_FB

#ifdef CONFIG_AK8789_HALLSENSOR
#include <linux/hall_sensor.h>
#endif

/*#include <mach/gpiomux.h>*/
#define D(x...) pr_debug("[S_HUB][CW_MCU] " x)
#define I(x...) pr_info("[S_HUB][CW_MCU] " x)
#define W(x...) pr_warn("[S_HUB][CW_MCU] " x)
#define E(x...) pr_err("[S_HUB][CW_MCU] " x)

#ifdef CONFIG_AK8789_HALLSENSOR
#define HALL_RETRY_TIMES 5
#endif

#define RETRY_TIMES 20
#define LATCH_TIMES  1
#define LATCH_ERROR_NO (-110)
#define ACTIVE_RETRY_TIMES 10
#define DPS_MAX			(1 << (16 - 1))
/* ========================================================================= */

#define TOUCH_LOG_DELAY		5000
#define CWMCU_BATCH_TIMEOUT_MIN 200
#define MS_TO_PERIOD (1000 * 99 / 100)

/* ========================================================================= */
#define rel_significant_motion REL_WHEEL

#define ACC_CALIBRATOR_LEN 3
#define ACC_CALIBRATOR_RL_LEN 12
#define MAG_CALIBRATOR_LEN 26
#define GYRO_CALIBRATOR_LEN 3
#define LIGHT_CALIBRATOR_LEN 4
#define PROXIMITY_CALIBRATOR_LEN 6
#define PRESSURE_CALIBRATOR_LEN 4

#define REPORT_EVENT_COMMON_LEN 3
#define REPORT_EVENT_MOTION_LEN 6
#define REPORT_EVENT_PROXIMITY_LEN 9

#define FW_VER_INFO_LEN 31
#define FW_VER_HEADER_LEN 7
#define FW_VER_COUNT 6
#define FW_I2C_LEN_LIMIT 256

#define ENABLE_LIST_GROUP_NUM 4
#define REACTIVATE_PERIOD (10*HZ)
#define RESET_PERIOD (30*HZ)
#define SYNC_ACK_MAGIC  0x66
#define EXHAUSTED_MAGIC 0x77

#define CRASH_REASON_NUM 4

#define CALIBRATION_DATA_PATH "/calibration_data"
#define G_SENSOR_FLASH_DATA "gs_flash"
#define GYRO_SENSOR_FLASH_DATA "gyro_flash"
#define LIGHT_SENSOR_FLASH_DATA "als_flash"
#define PROX_SENSOR_FLASH_DATA "ps_flash"
#define BARO_SENSOR_FLASH_DATA "bs_flash"

#define MCU_CHIP_MODE_APPLICATION   0
#define MCU_CHIP_MODE_BOOTLOADER    1

#define MCU_CRASH_REASON_WATCH_DOG 0x000d
#define MCU_CRASH_REASON_NO_DATA 0x0005
#define MCU_CRASH_REASON_UNKNOWN 0xffff

typedef enum {
    MCU_STATE_UNKNOWN = 0,
    MCU_STATE_SHUB_INIT,
    MCU_STATE_SHUB_RUN,
    MCU_STATE_DLOAD,
    MCU_STATE_BOOTLOADER,
    MCU_STATE_MAX,
} MCU_STATE;

static MCU_STATE s_mcu_state = MCU_STATE_UNKNOWN;
#define MCU_IN_UNKNOWN() (MCU_STATE_UNKNOWN == s_mcu_state)
#define MCU_IN_DLOAD() (MCU_STATE_DLOAD == s_mcu_state)
#define MCU_IN_SHUB() (MCU_STATE_SHUB_INIT == s_mcu_state || MCU_STATE_SHUB_RUN == s_mcu_state)
#define MCU_IN_SHUB_INIT() (MCU_STATE_SHUB_INIT == s_mcu_state)
#define MCU_IN_SHUB_RUN() (MCU_STATE_SHUB_RUN == s_mcu_state)
#define MCU_IN_BOOTLOADER() (MCU_STATE_BOOTLOADER == s_mcu_state)
static DEFINE_MUTEX(s_activated_i2c_lock);
static DECLARE_COMPLETION(s_mcu_enter_shub_run);

#ifdef SHUB_DLOAD_SUPPORT
#define MCU2CPU_STATUS_GPIO_LEVEL_SHUB 0
#define MCU2CPU_STATUS_GPIO_LEVEL_DLOAD 1
static DECLARE_COMPLETION(s_mcu_ramdump_avail);
static void mcu_enable_disable_dload_mode(bool en);
#endif //SHUB_DLOAD_SUPPORT

#ifdef SHUB_LOGGING_SUPPORT
static DECLARE_COMPLETION(s_mcu_log_avail);
static void mcu_set_log_mask(u32 log_mask);
static void mcu_set_log_level(u32 log_level);
#endif //SHUB_LOGGING_SUPPORT

#ifdef SHUB_EVENT_SUPPORT
static DECLARE_COMPLETION(s_mcu_event_avail);
#endif //SHUB_EVENT_SUPPORT

#ifdef CONFIG_CWSTM32_DEBUG  /* Remove this from defconfig when release */

static int DEBUG_FLAG_GSENSOR;
static int DEBUG_FLAG_LIGHT_POLLING;
module_param(DEBUG_FLAG_GSENSOR, int, 0600);
module_param(DEBUG_FLAG_LIGHT_POLLING, int, 0600);

#else

#define DEBUG_FLAG_GSENSOR 0
#define DEBUG_FLAG_LIGHT_POLLING 0

#endif

static int DEBUG_DISABLE;
module_param(DEBUG_DISABLE, int, 0660);

MODULE_PARM_DESC(DEBUG_DISABLE, "disable " CWMCU_I2C_NAME " driver") ;

static u8 g_touch_solution = 0xff;
static u8 g_touch_status = 0xff;

static int p_status = 9;
static u32 adc_table[10] = {0};

struct cwmcu_data {
	struct i2c_client *client;
	struct cwmcu_bus_client *mcu_client;
	struct device *mcu_dev;
	atomic_t delay;

	/* mutex_lock protect:
	 * cw_set_pseudo_irq(indio_dev, state);
	 * iio_push_to_buffers(mcu_data->indio_dev, event);
	 */
	struct mutex mutex_lock;

	/* group_i2c_lock protect:
	 * set_calibrator_en(),
	 * set_k_value(),
	 * get_light_polling(),
	 * CWMCU_i2c_multi_write()
	 */
	struct mutex group_i2c_lock;

	/* power_mode_lock protect:
	 * mcu_data->power_on_counter
	 */
	struct mutex power_mode_lock;
	struct iio_trigger  *trig;
	atomic_t pseudo_irq_enable;
	atomic_t als_first_polling_event;
	struct mutex lock;

	struct timeval now;
	struct class *sensor_class;
	struct device *sensor_dev;
	struct class *bma250_class;
	struct device *bma250_dev;
	struct class *gs_cali_class;
	struct device *gs_cali_dev;
	u8	acceleration_axes;
	u8	magnetic_axes;
	u8	gyro_axes;

	u64	enabled_list; /* Bit mask for sensor enable status */
	u64	batched_list; /* Bit mask for FIFO usage, 32MSB is wake up */

	/* report time */
	s64	sensors_time[num_sensors];
	s64	time_diff[num_sensors];
	s32	report_period[num_sensors]; /* Microseconds * 0.99 */
	u64	update_list;
	int	pending_flush[num_sensors];
	s64	batch_timeout[num_sensors];
	int	IRQ;
	struct delayed_work	work;
	struct work_struct	one_shot_work;
	/* Remember to add flag in cwmcu_resume() when add new flag */
	bool w_activated_i2c;
	bool w_re_init;
	bool w_flush_fifo;
	bool w_clear_fifo;
	bool w_clear_fifo_running;
	bool w_kick_start_mcu;
	bool w_mcu_state_change;
#ifdef CONFIG_AK8789_HALLSENSOR
	bool w_hall_inform_mcu;

	u8 dotview_st;
#endif
	bool w_batch_read;
	bool iio_work_done;
	atomic_t suspended;
	atomic_t enter_suspend;
	atomic_t critical_sect;
	bool is_block_i2c;

	u32 gpio_wake_mcu;
	u32 gpio_reset;
	u32 gpio_chip_mode;
	int gpio_chip_mode_level;
	u32 gpio_mcu_irq;
	u32 gpio_mcu_status;
	int gpio_mcu_status_level;
	s32 gs_chip_layout;
	s32 touch_enable;
	s32 vibrate_ms;
	u32 gs_kvalue;
	s16 gs_kvalue_R1;
	s16 gs_kvalue_R2;
	s16 gs_kvalue_R3;
	s16 gs_kvalue_L1;
	s16 gs_kvalue_L2;
	s16 gs_kvalue_L3;
	u32 gy_kvalue;
	u8  als_goldl;
	u8  als_goldh;
	u8  als_polling;
	u8  als_lux_ratio_n;
	u8  als_lux_ratio_d;
	u32 *als_levels;
	u32 als_kvalue;
        u32 ps_kvalue;
        u32 ps_kheader;
        u16 ps_thd_fixed;
        u16 ps_thd_add;
	u32 bs_kvalue;
	u8  bs_kheader;
	u8  gs_calibrated;
	u8  ps_calibrated;
	u8  ls_calibrated;
	u8  bs_calibrated;
	u8  gy_calibrated;

	s32 i2c_total_retry;
	s32 i2c_latch_retry;
	unsigned long i2c_jiffies;
	unsigned long reset_jiffies;
	unsigned long suspend_jiffies;
	unsigned long resume_jiffies;
	unsigned long batch_fifo_read_start_jiffies;
	unsigned long batch_fifo_read_end_jiffies;
	unsigned long enqueue_iio_irq_work_jiffies;

	int disable_access_count;

	s32 iio_data[6];
	struct iio_dev *indio_dev;
	struct irq_work iio_irq_work;

	/* power status */
	int power_on_counter;

	u16 light_last_data[REPORT_EVENT_COMMON_LEN];
	u64 time_base;
	u64 wake_fifo_time_base;
	u64 step_counter_base;

	struct workqueue_struct *mcu_wq;
	struct wake_lock gesture_motion_wake_lock;
	struct wake_lock significant_wake_lock;
	struct wake_lock any_motion_wake_lock;
	struct wake_lock batch_read_wake_lock;
	struct wake_lock ps_read_wake_lock;
#ifdef SHUB_DLOAD_SUPPORT
	struct wake_lock mcu_dload_wake_lock;
#endif //SHUB_DLOAD_SUPPORT
	struct wake_lock report_wake_lock;
	struct wake_lock one_shot_wake_lock;

	int fw_update_status;

	bool mcu_bootup;
	bool mcu_sensor_ready;
	bool dload_mode_enabled;

	u32 gesture_motion_param;
	int power_key_pressed;

#ifdef SHUB_LOGGING_SUPPORT
	uint32_t mcu_log_mask;
	uint32_t mcu_log_level;
#endif //SHUB_LOGGING_SUPPORT
	struct pinctrl *pinctrl;
	struct pinctrl_state *gpio_state_init;

#ifdef CONFIG_FB
	struct notifier_block fb_notif;
	struct delayed_work delay_work_register_fb;
	bool is_display_on;
#endif //CONFIG_FB

	u16 jenkins_build_ver;
	u16 crash_reason;
	u32 crash_count[CRASH_REASON_NUM]; /* index 0 is used for exception */

	struct regulator *v_sr_2v85;
	struct regulator *v_srio_1v8;
};

static struct cwmcu_data *s_mcu_data = NULL;
#if 0
static struct vib_trigger *vib_trigger = NULL;
#endif

BLOCKING_NOTIFIER_HEAD(double_tap_notifier_list);

static int CWMCU_i2c_read(struct cwmcu_data *mcu_data,
			u8 reg_addr, void *data, u8 len);
static int CWMCU_i2c_read_power(struct cwmcu_data *mcu_data,
			 u8 reg_addr, void *data, u8 len);
static int CWMCU_i2c_write(struct cwmcu_data *mcu_data,
			u8 reg_addr, const void *data, u8 len, bool mutex_lock_i2c);
static int CWMCU_i2c_write_power(struct cwmcu_data *mcu_data,
			u8 reg_addr, const void *data, u8 len);
static int CWMCU_i2c_write_block(struct cwmcu_data *sensor,
			  u8 reg_addr, u8 *data, u8 len);
static int firmware_odr(struct cwmcu_data *mcu_data, int sensors_id,
			int delay_ms);
static void cwmcu_batch_read(struct cwmcu_data *mcu_data);
static bool reset_hub(struct cwmcu_data *mcu_data, bool force_reset);
int is_continuous_sensor(int sensors_id);

#ifdef SHUB_DLOAD_SUPPORT
static int mcu_set_reboot_state(u32 state);
static int mcu_dload_i2c_read(u8 cmd, u8 *data, u8 len);
static int mcu_dload_dump_backup_registers(void);
static int mcu_dload_dump_exception_buffer(struct cwmcu_data *mcu_data);

static inline int MCU2CPU_STATUS_GPIO_LEVEL(struct cwmcu_data *mcu_data)
{
    if ((MCU_IN_BOOTLOADER()) || (gpio_get_value_cansleep(mcu_data->gpio_reset) == 0)) {
        return MCU2CPU_STATUS_GPIO_LEVEL_SHUB;
    }

    if (gpio_is_valid(mcu_data->gpio_mcu_status)) {
        return gpio_get_value(mcu_data->gpio_mcu_status);
    }
    else {
        int ret;
        u32 mcu_status_read = 0xff;

        E("gpio_mcu_status is invalid !!!\n");

        ret = mcu_dload_i2c_read(CW_I2C_REG_REBOOT_MODE, (u8*)&mcu_status_read, sizeof(mcu_status_read));
        if (ret > 0) {
            I("MCU2CPU_STATUS_GPIO_LEVEL_DLOAD\n");
            return MCU2CPU_STATUS_GPIO_LEVEL_DLOAD;
        }
        else  {
            I("MCU2CPU_STATUS_GPIO_LEVEL_SHUB\n");
            return MCU2CPU_STATUS_GPIO_LEVEL_SHUB;
        }
    }
}
#endif //SHUB_DLOAD_SUPPORT

static void gpio_make_falling_edge(int gpio)
{
	if (!gpio_get_value(gpio))
		gpio_set_value(gpio, 1);
	gpio_set_value(gpio, 0);
}

static void mcu_chip_mode_set(struct cwmcu_data *mcu_data, int onoff)
{
    if (mcu_data) {
        gpio_direction_output(mcu_data->gpio_chip_mode, onoff);
        mcu_data->gpio_chip_mode_level = onoff;
    }
}

static int mcu_chip_mode_get(struct cwmcu_data *mcu_data)
{
    if (mcu_data)
        return mcu_data->gpio_chip_mode_level;
    else
        return -1;
}

static inline void mcu_boot_status_reset(struct cwmcu_data *mcu_data)
{
	mcu_data->mcu_bootup = false;
	mcu_data->mcu_sensor_ready = false;
}

static void mcu_state_enter_unknown(struct cwmcu_data *mcu_data)
{
    struct timespec ts;
    struct rtc_time tm;

    //get current time
    getnstimeofday(&ts);
    rtc_time_to_tm(ts.tv_sec, &tm);

    reinit_completion(&s_mcu_enter_shub_run);
    s_mcu_state = MCU_STATE_UNKNOWN;
    E("%s[%d]: s_mcu_state enter MCU_STATE_UNKNOWN at (%d-%02d-%02d %02d:%02d:%02d.%09lu UTC) !!!!!!!!!!\n", __func__, __LINE__,
        tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
}

static void mcu_state_enter_shub_init(struct cwmcu_data *mcu_data)
{
    struct timespec ts;
    struct rtc_time tm;

    //get current time
    getnstimeofday(&ts);
    rtc_time_to_tm(ts.tv_sec, &tm);

    I("%s[%d]: s_mcu_state enter MCU_STATE_SHUB_INIT at (%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n", __func__, __LINE__,
        tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
    s_mcu_state = MCU_STATE_SHUB_INIT;

    mcu_data->w_re_init = true;
    queue_work(s_mcu_data->mcu_wq, &s_mcu_data->one_shot_work);
}

static void mcu_state_enter_shub_run(struct cwmcu_data *mcu_data)
{
    struct timespec ts;
    struct rtc_time tm;

    //get current time
    getnstimeofday(&ts);
    rtc_time_to_tm(ts.tv_sec, &tm);

    I("%s[%d]: s_mcu_state enter MCU_STATE_SHUB_RUN at (%d-%02d-%02d %02d:%02d:%02d.%09lu UTC)\n", __func__, __LINE__,
                    tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec, ts.tv_nsec);
    s_mcu_state = MCU_STATE_SHUB_RUN;
    complete_all(&s_mcu_enter_shub_run);
}

#ifdef SHUB_DLOAD_SUPPORT
static u16 to_crash_count_index(u16 crash_reason)
{
	switch (crash_reason) {
	case MCU_CRASH_REASON_WATCH_DOG:
		return 1;
	case MCU_CRASH_REASON_NO_DATA:
		return 2;
	case MCU_CRASH_REASON_UNKNOWN:
		return 3;
	default:
		return 0;
	}
}

static void mcu_state_enter_dload(struct cwmcu_data *mcu_data)
{
    struct timespec ts;
    struct rtc_time tm;
    int ret;
    u16 idx;

    if (!mcu_data)
        return;

    if (!mcu_data->dload_mode_enabled) {
        E("%s[%d]: set mcu to SHUB mode\n", __func__, __LINE__);
        ret = mcu_set_reboot_state(MCU_SYS_STATUS_SHUB);
        if (ret >= 0) {
            if (mcu_data) {
                mutex_lock(&s_activated_i2c_lock);
                reset_hub(mcu_data, true);
                mutex_unlock(&s_activated_i2c_lock);
            }
        }
        return;
    }

    I("%s[%d]", __func__, __LINE__);

    ret = mcu_dload_dump_backup_registers();
    if (ret < 0) {
        E("%s[%d]: mcu_dload_dump_backup_registers fails, ret = %d\n", __func__, __LINE__, ret);
        return;
    }

    mcu_data->crash_reason = MCU_CRASH_REASON_UNKNOWN;  //reset crash reason to unknwon
    ret = mcu_dload_dump_exception_buffer(mcu_data);
    if (ret < 0) {
        E("%s[%d]: mcu_dload_dump_exception_buffer fails, ret = %d\n", __func__, __LINE__, ret);
        return;
    }

    //get current time
    getnstimeofday(&ts);
    rtc_time_to_tm(ts.tv_sec, &tm);

    mutex_lock(&s_activated_i2c_lock);
    reinit_completion(&s_mcu_enter_shub_run);
    s_mcu_state = MCU_STATE_DLOAD;
    mutex_unlock(&s_activated_i2c_lock);

    I("%s[%d]: wake_lock_timeout:mcu_dload_wake_lock\n", __func__, __LINE__);
    wake_lock_timeout(&mcu_data->mcu_dload_wake_lock,
        15 * HZ);

    idx = to_crash_count_index(mcu_data->crash_reason);;
    idx = (idx < CRASH_REASON_NUM) ? idx : 0;

    mcu_data->crash_count[idx]++;

    if ((mcu_data->crash_reason == MCU_CRASH_REASON_UNKNOWN) &&
	((mcu_data->crash_count[idx] % 100) == 1)) {
	E("%s[%d]: Silent recover, reason[0x%x], ver[%d], "
	  "crash_count[%d] = %d, s_mcu_state enter MCU_STATE_DLOAD at "
	  "(%d-%02d-%02d %02d:%02d:%02d.%09lu UTC) !!!!!!!!!!\n",
	  __func__, __LINE__,
	  mcu_data->crash_reason,
	  mcu_data->jenkins_build_ver, idx, mcu_data->crash_count[idx],
	  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min,
	  tm.tm_sec, ts.tv_nsec);
    } else {
	I("%s[%d]: Silent recover - Enter dload DBG, reason[0x%x], ver[%d], "
	  "crash_count[%d] = %d, s_mcu_state enter MCU_STATE_DLOAD at "
	  "(%d-%02d-%02d %02d:%02d:%02d.%09lu UTC) !!!!!!!!!!\n",
	  __func__, __LINE__,
	  mcu_data->crash_reason,
	  mcu_data->jenkins_build_ver, idx, mcu_data->crash_count[idx],
	  tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min,
	  tm.tm_sec, ts.tv_nsec);
    }

    mcu_boot_status_reset(mcu_data);
    complete(&s_mcu_ramdump_avail);
}
#endif //SHUB_DLOAD_SUPPORT

static void do_mcu_state_transition(struct cwmcu_data *mcu_data)
{
    MCU_STATE pre_mcu_state = s_mcu_state;
#ifdef SHUB_DLOAD_SUPPORT
    int mcu_status_level = mcu_data->gpio_mcu_status_level;
#endif //SHUB_DLOAD_SUPPORT

    D("%s enter, s_mcu_state:0x%x\n", __func__, pre_mcu_state);

DO_STAT_TRANS:
    switch (s_mcu_state) {
    case MCU_STATE_UNKNOWN:
        {
            #ifdef SHUB_DLOAD_SUPPORT
            if (MCU2CPU_STATUS_GPIO_LEVEL_DLOAD == mcu_status_level) {
                mcu_state_enter_dload(mcu_data);
            }
            #endif //SHUB_DLOAD_SUPPORT

            if (mcu_data->mcu_bootup) {
                mcu_data->mcu_bootup = false;
                mcu_state_enter_shub_init(mcu_data);
            }
        }
        break;

    case MCU_STATE_SHUB_INIT:
        {
            #ifdef SHUB_DLOAD_SUPPORT
            if (MCU2CPU_STATUS_GPIO_LEVEL_DLOAD == mcu_status_level) {
                mcu_state_enter_dload(mcu_data);
            }
            #endif //SHUB_DLOAD_SUPPORT

            //if abnormal bootup
            if (mcu_data->mcu_bootup) {
                mcu_state_enter_unknown(mcu_data);
                goto DO_STAT_TRANS;
            }

            if (mcu_data->mcu_sensor_ready) {
                mcu_data->mcu_sensor_ready = false;
                mcu_state_enter_shub_run(mcu_data);
            }
        }
        break;

    case MCU_STATE_SHUB_RUN:
        {
            #ifdef SHUB_DLOAD_SUPPORT
            if (MCU2CPU_STATUS_GPIO_LEVEL_DLOAD == mcu_status_level) {
                mcu_state_enter_dload(mcu_data);
            }
            #endif //SHUB_DLOAD_SUPPORT

            //if abnormal bootup
            if (mcu_data->mcu_bootup) {
                mcu_state_enter_unknown(mcu_data);
                goto DO_STAT_TRANS;
            }
        }
        break;

#ifdef SHUB_DLOAD_SUPPORT
    case MCU_STATE_DLOAD:
        if ((MCU2CPU_STATUS_GPIO_LEVEL_SHUB == mcu_status_level)) {
            I("%s[%d]: wake_unlock:mcu_dload_wake_lock\n", __func__, __LINE__);
            wake_unlock(&mcu_data->mcu_dload_wake_lock);
            mcu_state_enter_unknown(mcu_data);
            goto DO_STAT_TRANS;
        }
        break;
#endif //SHUB_DLOAD_SUPPORT

    case MCU_STATE_BOOTLOADER:
        break;

    default:
        E("%s[%d]: incorrect s_mcu_state = 0x%x\n", __func__, __LINE__, s_mcu_state);
        break;
    }

    if (pre_mcu_state != s_mcu_state) {
        I("%s s_mcu_state change 0x%x -> 0x%x\n", __func__, pre_mcu_state, s_mcu_state);
    }

    D("%s exit, s_mcu_state:0x%x\n", __func__, s_mcu_state);
}

static void cwmcu_powermode_switch(struct cwmcu_data *mcu_data, int onoff)
{
	mutex_lock(&mcu_data->power_mode_lock);
	if (onoff) {
		if (mcu_data->power_on_counter == 0) {
			gpio_make_falling_edge(mcu_data->gpio_wake_mcu);
			udelay(10);
			gpio_set_value(mcu_data->gpio_wake_mcu, 1);
			udelay(10);
			gpio_set_value(mcu_data->gpio_wake_mcu, 0);
			D("%s: 11 onoff = %d\n", __func__, onoff);
			usleep_range(500, 600);
		}
		mcu_data->power_on_counter++;
	} else {
		mcu_data->power_on_counter--;
		if (mcu_data->power_on_counter <= 0) {
			mcu_data->power_on_counter = 0;
			gpio_set_value(mcu_data->gpio_wake_mcu, 1);
			D("%s: 22 onoff = %d\n", __func__, onoff);
		}
	}
	mutex_unlock(&mcu_data->power_mode_lock);
	D("%s: onoff = %d, power_counter = %d\n", __func__, onoff,
	  mcu_data->power_on_counter);
}

static int cw_send_event(struct cwmcu_data *mcu_data, u8 id, u16 *data,
			 s64 timestamp)
{
	u8 event[21];/* Sensor HAL uses fixed 21 bytes */

	event[0] = id;
	memcpy(&event[1], data, sizeof(u16)*3);
	memset(&event[7], 0, sizeof(u16)*3);
	memcpy(&event[13], &timestamp, sizeof(s64));

	D("%s: active_scan_mask = 0x%p, masklength = %u, data(x, y, z) ="
	  "(%d, %d, %d)\n",
	  __func__, mcu_data->indio_dev->active_scan_mask,
	  mcu_data->indio_dev->masklength,
	  *(s16 *)&event[1], *(s16 *)&event[3], *(s16 *)&event[5]);

	if (mcu_data->indio_dev && mcu_data->indio_dev->active_scan_mask &&
	    (!bitmap_empty(mcu_data->indio_dev->active_scan_mask,
			   mcu_data->indio_dev->masklength))) {
		mutex_lock(&mcu_data->mutex_lock);
		if (atomic_read(&mcu_data->pseudo_irq_enable)
		    && ((!mcu_data->w_clear_fifo_running)
		     || (!is_continuous_sensor(id))))
			iio_push_to_buffers(mcu_data->indio_dev, event);
		else {
			D(
			  "%s: Drop data(0, 1, 2, 3) = "
			  "(0x%x, 0x%x, 0x%x, 0x%x)\n", __func__,
			  data[0], data[1], data[2], data[3]);
		}
		mutex_unlock(&mcu_data->mutex_lock);

		if (!mcu_data->indio_dev) {
			E("%s: mcu_data->indio_dev == NULL!!\n", __func__);
		}

		return 0;
	} else
		D("%s: Event might be missing\n", __func__);

	return -EIO;
}

static int cw_send_event_special(struct cwmcu_data *mcu_data, u8 id, u16 *data,
				 u16 *bias, s64 timestamp)
{
	u8 event[1+(2*sizeof(u16)*REPORT_EVENT_COMMON_LEN)+sizeof(timestamp)];

	event[0] = id;
	memcpy(&event[1], data, sizeof(u16)*REPORT_EVENT_COMMON_LEN);
	memcpy(&event[1+sizeof(u16)*REPORT_EVENT_COMMON_LEN], bias,
	       sizeof(u16)*REPORT_EVENT_COMMON_LEN);
	memcpy(&event[1+(2*sizeof(u16)*REPORT_EVENT_COMMON_LEN)], &timestamp,
	       sizeof(timestamp));

	if (mcu_data->indio_dev && mcu_data->indio_dev->active_scan_mask &&
	    (!bitmap_empty(mcu_data->indio_dev->active_scan_mask,
			   mcu_data->indio_dev->masklength))) {
		mutex_lock(&mcu_data->mutex_lock);
		if (atomic_read(&mcu_data->pseudo_irq_enable)
		    && mcu_data->indio_dev
		    && ((!mcu_data->w_clear_fifo_running)
		     || (!is_continuous_sensor(id))))
			iio_push_to_buffers(mcu_data->indio_dev, event);
		else {
			D(
			  "%s: Drop data(0, 1, 2, 3) = "
			  "(0x%x, 0x%x, 0x%x, 0x%x)\n", __func__,
			  data[0], data[1], data[2], data[3]);
		}
		mutex_unlock(&mcu_data->mutex_lock);

		if (!mcu_data->indio_dev) {
			E("%s: mcu_data->indio_dev == NULL!!\n", __func__);
		}

		return 0;
	} else
		D("%s: Event might be missing\n", __func__);

	return -EIO;
}

static int cwmcu_get_calibrator_status(struct cwmcu_data *mcu_data,
				       u8 sensor_id, u8 *data)
{
	int error_msg = 0;

	if (sensor_id == CW_ACCELERATION)
		error_msg = CWMCU_i2c_read_power(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_STATUS_ACC,
				data, 1);
	else if (sensor_id == CW_MAGNETIC)
		error_msg = CWMCU_i2c_read_power(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_STATUS_MAG,
				data, 1);
	else if (sensor_id == CW_GYRO)
		error_msg = CWMCU_i2c_read_power(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_STATUS_GYRO,
				data, 1);

	return error_msg;
}

static int cwmcu_get_calibrator(struct cwmcu_data *mcu_data, u8 sensor_id,
				s8 *data, u8 len)
{
	int error_msg = 0;

	if ((sensor_id == CW_ACCELERATION) && (len == ACC_CALIBRATOR_LEN))
		error_msg = CWMCU_i2c_read_power(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_ACC,
				data, len);
	else if ((sensor_id == CW_MAGNETIC) && (len == MAG_CALIBRATOR_LEN))
		error_msg = CWMCU_i2c_read_power(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_MAG,
				data, len);
	else if ((sensor_id == CW_GYRO) && (len == GYRO_CALIBRATOR_LEN))
		error_msg = CWMCU_i2c_read_power(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_GYRO,
				data, len);
	else if ((sensor_id == CW_LIGHT) && (len == LIGHT_CALIBRATOR_LEN))
		error_msg = CWMCU_i2c_read_power(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_LIGHT,
				data, len);
        else if ((sensor_id == CW_PROXIMITY) && (len == PROXIMITY_CALIBRATOR_LEN))
                error_msg = CWMCU_i2c_read_power(mcu_data,
                                CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_PROXIMITY,
                                data, len);
	else if ((sensor_id == CW_PRESSURE) && (len == PRESSURE_CALIBRATOR_LEN))
		error_msg = CWMCU_i2c_read_power(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_PRESSURE,
				data, len);
	else
		E("%s: invalid arguments, sensor_id = %u, len = %u\n",
		  __func__, sensor_id, len);

	I("sensors_id = %u, calibrator data = (%d, %d, %d)\n", sensor_id,
	  data[0], data[1], data[2]);
	return error_msg;
}

static ssize_t set_calibrator_en(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 data;
	u8 data2;
	unsigned long sensors_id;
	int error;

	error = kstrtoul(buf, 10, &sensors_id);
	if (error) {
		E("%s: kstrtoul fails, error = %d\n", __func__, error);
		return error;
	}

	/* sensor_id at least should between 0 ~ 31 */
	data = (u8)sensors_id;
	D("%s: data(sensors_id) = %u\n", __func__, data);

	cwmcu_powermode_switch(mcu_data, 1);

	mutex_lock(&mcu_data->group_i2c_lock);

	switch (data) {
	case 1:
		error = CWMCU_i2c_read(mcu_data, G_SENSORS_STATUS, &data2, 1);
		if (error < 0)
			goto i2c_fail;
		data = data2 | 16;
		error = CWMCU_i2c_write(mcu_data, G_SENSORS_STATUS, &data, 1, 1);
		if (error < 0)
			goto i2c_fail;
		break;
	case 2:
		error = CWMCU_i2c_read(mcu_data, ECOMPASS_SENSORS_STATUS,
				       &data2, 1);
		if (error < 0)
			goto i2c_fail;
		data = data2 | 16;
		error = CWMCU_i2c_write(mcu_data, ECOMPASS_SENSORS_STATUS,
					&data, 1, 1);
		if (error < 0)
			goto i2c_fail;
		break;
	case 4:
		error = CWMCU_i2c_read(mcu_data, GYRO_SENSORS_STATUS,
				       &data2, 1);
		if (error < 0)
			goto i2c_fail;
		data = data2 | 16;
		error = CWMCU_i2c_write(mcu_data, GYRO_SENSORS_STATUS,
					&data, 1, 1);
		if (error < 0)
			goto i2c_fail;
		break;
	case 7:
		error = CWMCU_i2c_read(mcu_data, LIGHT_SENSORS_STATUS,
				       &data2, 1);
		if (error < 0)
			goto i2c_fail;
		data = data2 | 16;
		error = CWMCU_i2c_write(mcu_data, LIGHT_SENSORS_STATUS,
					&data, 1, 1);
		if (error < 0)
			goto i2c_fail;
		break;
        case 8:
                error =  CWMCU_i2c_read(mcu_data, PROXIMITY_SENSORS_STATUS,
                                        &data2, 1);
		if (error < 0)
			goto i2c_fail;
		data = data2 | 16;
		error = CWMCU_i2c_write(mcu_data, PROXIMITY_SENSORS_STATUS,
					&data,1, 1);
		if (error < 0)
			goto i2c_fail;
		break;
	case 9:
		data = 2; /* X- R calibration */
		error = CWMCU_i2c_write(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_TARGET_ACC,
				&data, 1, 1);
		error = CWMCU_i2c_read(mcu_data, G_SENSORS_STATUS, &data2, 1);
		if (error < 0)
			goto i2c_fail;
		data = data2 | 16;
		error = CWMCU_i2c_write(mcu_data, G_SENSORS_STATUS, &data, 1, 1);
		if (error < 0)
			goto i2c_fail;
		break;
	case 10:
		data = 1; /* X+ L calibration */
		error = CWMCU_i2c_write(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_TARGET_ACC,
				&data, 1, 1);
		error = CWMCU_i2c_read(mcu_data, G_SENSORS_STATUS, &data2, 1);
		if (error < 0)
			goto i2c_fail;
		data = data2 | 16;
		error = CWMCU_i2c_write(mcu_data, G_SENSORS_STATUS, &data, 1, 1);
		if (error < 0)
			goto i2c_fail;
		break;
	case 11:
		data = 0; /* Z+ */
		error = CWMCU_i2c_write(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_TARGET_ACC,
				&data, 1, 1);
		if (error < 0)
			goto i2c_fail;
		break;
	case 12:
		mcu_data->step_counter_base = 0;
		D("%s: Reset step counter\n", __func__);
		break;
	default:
		mutex_unlock(&mcu_data->group_i2c_lock);
		cwmcu_powermode_switch(mcu_data, 0);
		E("%s: Improper sensor_id = %u\n", __func__, data);
		return -EINVAL;
	}

	error = count;

i2c_fail:
	mutex_unlock(&mcu_data->group_i2c_lock);

	cwmcu_powermode_switch(mcu_data, 0);

	D("%s--: data2 = 0x%x, rc = %d\n", __func__, data2, error);
	return error;
}

static void print_hex_data(char *buf, u32 index, u8 *data, size_t len)
{
	int i;
	int rc;
	char *buf_start;
	size_t buf_remaining =
		3*EXCEPTION_BLOCK_LEN; /* 3 characters per data */

	buf_start = buf;

	for (i = 0; i < len; i++) {
		rc = scnprintf(buf, buf_remaining, "%02x%c", data[i],
				(i == len - 1) ? '\0' : ' ');
		buf += rc;
		buf_remaining -= rc;
	}

	printk(KERN_ERR "[S_HUB][CW_MCU] Exception Buffer[%04d] = %.*s\n",
			index * EXCEPTION_BLOCK_LEN,
			(int)(buf - buf_start),
			buf_start);
}

static ssize_t sprint_data(char *buf, s8 *data, ssize_t len)
{
	int i;
	int rc;
	size_t buf_remaining = PAGE_SIZE;

	for (i = 0; i < len; i++) {
		rc = scnprintf(buf, buf_remaining, "%d%c", data[i],
				(i == len - 1) ? '\n' : ' ');
		buf += rc;
		buf_remaining -= rc;
	}
	return PAGE_SIZE - buf_remaining;
}

static ssize_t show_calibrator_status_acc(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 data[6] = {0};

	if (cwmcu_get_calibrator_status(mcu_data, CW_ACCELERATION, data) >= 0)
		return scnprintf(buf, PAGE_SIZE, "0x%x\n", data[0]);

	return scnprintf(buf, PAGE_SIZE, "0x1\n");
}

static ssize_t show_calibrator_status_mag(struct device *dev,
					  struct device_attribute *attr,
					  char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 data[6] = {0};

	if (cwmcu_get_calibrator_status(mcu_data, CW_MAGNETIC, data) >= 0)
		return scnprintf(buf, PAGE_SIZE, "0x%x\n", data[0]);

	return scnprintf(buf, PAGE_SIZE, "0x1\n");
}

static ssize_t show_calibrator_status_gyro(struct device *dev,
					   struct device_attribute *attr,
					   char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 data[6] = {0};

	if (cwmcu_get_calibrator_status(mcu_data, CW_GYRO, data) >= 0)
		return scnprintf(buf, PAGE_SIZE, "0x%x\n", data[0]);

	return scnprintf(buf, PAGE_SIZE, "0x1\n");
}

static ssize_t set_k_value(struct cwmcu_data *mcu_data, const char *buf,
			   size_t count, u8 reg_addr, u8 len)
{
	int i;
	long data_temp[len];
	char *str_buf;
	char *running;
	int error;

	D(
	  "%s: count = %lu, strlen(buf) = %lu, PAGE_SIZE = %lu,"
	  " reg_addr = 0x%x\n",
	  __func__, count, strlen(buf), PAGE_SIZE, reg_addr);

	memset(data_temp, 0, len);

	str_buf = kstrndup(buf, count, GFP_KERNEL);
	if (str_buf == NULL) {
		E("%s: cannot allocate buffer\n", __func__);
		return -ENOMEM;
	}
	running = str_buf;

	for (i = 0; i < len; i++) {
		int error;
		char *token;

		token = strsep(&running, " ");

		if (token == NULL) {
			D("%s: i = %d\n", __func__, i);
			break;
		} else {
			if (reg_addr ==
			    CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PRESSURE)
				error = kstrtol(token, 16, &data_temp[i]);
			else
				error = kstrtol(token, 10, &data_temp[i]);
			if (error) {
				E("%s: kstrtol fails, error = %d, i = %d\n",
				  __func__, error, i);
				kfree(str_buf);
				return error;
			}
		}
	}
	kfree(str_buf);

	D("Set calibration by attr (%ld, %ld, %ld), len = %u, reg_addr = 0x%x\n"
	  , data_temp[0], data_temp[1], data_temp[2], len, reg_addr);

	cwmcu_powermode_switch(mcu_data, 1);

	mutex_lock(&mcu_data->group_i2c_lock);
	for (i = 0; i < len; i++) {
		u8 data = (u8)(data_temp[i]);
		/* Firmware can't write multi bytes */
		error = CWMCU_i2c_write(mcu_data, reg_addr, &data, 1, 1);
		if (error < 0) {
			mutex_unlock(&mcu_data->group_i2c_lock);
			cwmcu_powermode_switch(mcu_data, 0);
			E("%s: error = %d, i = %d\n", __func__, error, i);
			return -EIO;
		}
	}
	mutex_unlock(&mcu_data->group_i2c_lock);

	cwmcu_powermode_switch(mcu_data, 0);

	return count;
}

static ssize_t set_k_value_acc_f(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);

	return set_k_value(mcu_data, buf, count,
			   CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_ACC,
			   ACC_CALIBRATOR_LEN);
}


static ssize_t set_k_value_mag_f(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);

	return set_k_value(mcu_data, buf, count,
			   CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_MAG,
			   MAG_CALIBRATOR_LEN);
}

static ssize_t set_k_value_gyro_f(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);

	return set_k_value(mcu_data, buf, count,
			   CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_GYRO,
			   GYRO_CALIBRATOR_LEN);
}

static ssize_t set_k_value_proximity_f(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf, size_t count)
{
        struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
        int data_temp[3] = {0};
        int i = 0;
        u8 data[10] = {0};
        sscanf(buf, "%x %x\n", &data_temp[0], &data_temp[1]);
	data_temp[2]= mcu_data->ps_thd_add;

        I("%s: set proximity calibration\n", __func__);
        for(i = 0; i < 3; i++){
                data[i] = (u8)(s8) data_temp[i] & 0xFF;
                I("%s: data[%d] = %d, data_temp[%d] = %d\n"
			,__func__, 2*i, data[i], 2*i, data_temp[i]);
		CWMCU_i2c_write_power(mcu_data,
			CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,
				&data[i], 1);
                data[i] = (u8)(s8) (data_temp[i] >> 8) & 0xFF;
                I("%s: data[%d] = %d, data_temp[%d] = %d\n"
			,__func__, 2*i + 1,data[i] , 2*i + 1, data_temp[i]);
		CWMCU_i2c_write_power(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,
				&data[i], 1);
        }

	return count;
}

static ssize_t set_k_value_light_f(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf, size_t count)
{
        struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
        u8 datal = 0, datah = 0;
        u8 als_goldh = (mcu_data->als_goldh == 0) ? 0x0A : (mcu_data->als_goldh);
        u8 als_goldl = (mcu_data->als_goldl == 0) ? 0x38 : (mcu_data->als_goldl);
        int data_temp = 0;
        sscanf(buf, "%x\n", &data_temp);
        D("set light calibration\n");
        CWMCU_i2c_write_power(mcu_data,
                        CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_LIGHT,
                        &als_goldl, 1);
        CWMCU_i2c_write_power(mcu_data,
                        CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_LIGHT,
                        &als_goldh, 1);
        datal = data_temp & 0xFF;
        CWMCU_i2c_write_power(mcu_data,
                        CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_LIGHT,
                        &datal, 1);
        datah = (data_temp >> 8) & 0xFF;
        CWMCU_i2c_write_power(mcu_data,
                        CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_LIGHT,
                        &datah, 1);
        return count;
}

static ssize_t set_k_value_barometer_f(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);

	return set_k_value(mcu_data, buf, count,
			   CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PRESSURE,
			   PRESSURE_CALIBRATOR_LEN);
}

static ssize_t set_ps_canc(struct device *dev,struct device_attribute *attr,const char *buf, size_t count){

	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	int code = 0;
	u8 PS_canc1, PS_canc2 = 0;
        u8 PS_th1, PS_th2 = 0;
        u8 PS_th_add1, PS_th_add2 = 0;

        sscanf(buf, "%d\n", &code);

        if(code == 1) {
		if (((mcu_data->ps_kvalue >> 16) & 0xFFFF) == 0xFFFF) {
                        PS_canc1 = 0x00;
                        PS_canc2 = 0x00;
                        CWMCU_i2c_write_power(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_canc1,1);
                        CWMCU_i2c_write_power(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_canc2,1);
                        PS_th1 = (mcu_data->ps_kvalue >> 8) & 0xFF;
                        PS_th2 = 0x00;
                        CWMCU_i2c_write_power(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_th1,1);
                        CWMCU_i2c_write_power(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_th2,1);
                        PS_th_add1 = mcu_data->ps_thd_add & 0xFF;
                        PS_th_add2 = (mcu_data->ps_thd_add >> 8) & 0xFF;
                        CWMCU_i2c_write_power(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_th_add1,1);
                        CWMCU_i2c_write_power(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_th_add2,1);
                } else {
                        PS_canc1 = 0x00;
                        PS_canc2 = 0x00;
                        CWMCU_i2c_write_power(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_canc1,1);
                        CWMCU_i2c_write_power(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_canc2,1);
                        PS_th1 = (mcu_data->ps_kvalue & 0xFF0000) >> 16;
                        PS_th2 = (mcu_data->ps_kvalue & 0xFF000000) >> 24;
                        CWMCU_i2c_write_power(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_th1,1);
                        CWMCU_i2c_write_power(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_th2,1);
                        PS_th_add1 = mcu_data->ps_thd_add & 0xFF;
                        PS_th_add2 = (mcu_data->ps_thd_add >> 8) & 0xFF;
                        CWMCU_i2c_write_power(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_th_add1,1);
                        CWMCU_i2c_write_power(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_th_add2,1);
                }
                I("Recover PS_canc1 = %d, PS_canc2 = %d, PS_th1 = %d PS_th2 = %d, PS_th_add1 = %d, PS_th_add2 = %d\n",
				 PS_canc1, PS_canc2, PS_th1, PS_th2, PS_th_add1, PS_th_add2);
        } else {
                if (((mcu_data->ps_kvalue >> 16) & 0xFFFF) == 0xFFFF) {
                        PS_canc1 = (mcu_data->ps_kvalue) & 0xFF;
                        PS_canc2 = 0x00;
                        CWMCU_i2c_write_power(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_canc1,1);
                        CWMCU_i2c_write_power(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_canc2,1);
                        PS_th1 = (mcu_data->ps_kvalue >>  8) & 0xFF;
                        PS_th2 = 0x00;
                        CWMCU_i2c_write_power(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_th1,1);
                        CWMCU_i2c_write_power(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_th2,1);
                        PS_th_add1 = mcu_data->ps_thd_add & 0xFF;
                        PS_th_add2 = (mcu_data->ps_thd_add >> 8) & 0xFF;
                        CWMCU_i2c_write_power(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_th_add1,1);
                        CWMCU_i2c_write_power(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_th_add2,1);
                } else {
                        PS_canc1 = mcu_data->ps_kvalue & 0xFF;
                        PS_canc2 = ((mcu_data->ps_kvalue) & 0xFF00) >> 8;
                        CWMCU_i2c_write_power(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_canc1,1);
                        CWMCU_i2c_write_power(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_canc2,1);
                        PS_th1 = (mcu_data->ps_kvalue & 0xFF0000) >> 16;
                        PS_th2 = (mcu_data->ps_kvalue & 0xFF000000) >> 24;
                        CWMCU_i2c_write_power(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_th1,1);
                        CWMCU_i2c_write_power(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_th2,1);
                        PS_th_add1 = mcu_data->ps_thd_add & 0xFF;
                        PS_th_add2 = (mcu_data->ps_thd_add >> 8) & 0xFF;
                        CWMCU_i2c_write_power(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_th_add1,1);
                        CWMCU_i2c_write_power(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,&PS_th_add2,1);
                }
                I("Recover PS_canc1 = %d, PS_canc2 = %d, PS_th1 = %d PS_th2 = %d, PS_th_add1 = %d, PS_th_add2 = %d\n",
				 PS_canc1, PS_canc2, PS_th1, PS_th2, PS_th_add1, PS_th_add2);
        }
        return count;
}

static ssize_t get_k_value(struct cwmcu_data *mcu_data, int type, char *buf,
			   char *data, unsigned len)
{
	if (cwmcu_get_calibrator(mcu_data, type, data, len) < 0)
		memset(data, 0, len);

	return sprint_data(buf, data, len);
}

static ssize_t get_k_value_acc_f(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 data[ACC_CALIBRATOR_LEN];

	return get_k_value(mcu_data, CW_ACCELERATION, buf, data, sizeof(data));
}

static ssize_t get_k_value_acc_rl_f(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 data[ACC_CALIBRATOR_RL_LEN] = {0};

	if (CWMCU_i2c_read_power(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_RESULT_RL_ACC
			   , data, sizeof(data)) >= 0) {

		if (DEBUG_FLAG_GSENSOR == 1) {
			int i;

			for (i = 0; i < sizeof(data); i++)
				D("data[%d]: %u\n", i, data[i]);
		}

		mcu_data->gs_kvalue_L1 = ((s8)data[1] << 8) | data[0];
		mcu_data->gs_kvalue_L2 = ((s8)data[3] << 8) | data[2];
		mcu_data->gs_kvalue_L3 = ((s8)data[5] << 8) | data[4];
		mcu_data->gs_kvalue_R1 = ((s8)data[7] << 8) | data[6];
		mcu_data->gs_kvalue_R2 = ((s8)data[9] << 8) | data[8];
		mcu_data->gs_kvalue_R3 = ((s8)data[11] << 8) | data[10];
	}

	return sprint_data(buf, data, sizeof(data));
}

static ssize_t ap_get_k_value_acc_rl_f(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d %d %d %d %d %d\n",
			 (s16)mcu_data->gs_kvalue_L1,
			 (s16)mcu_data->gs_kvalue_L2,
			 (s16)mcu_data->gs_kvalue_L3,
			 (s16)mcu_data->gs_kvalue_R1,
			 (s16)mcu_data->gs_kvalue_R2,
			 (s16)mcu_data->gs_kvalue_R3);
}

static ssize_t get_k_value_mag_f(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 data[MAG_CALIBRATOR_LEN];

	return get_k_value(mcu_data, CW_MAGNETIC, buf, data, sizeof(data));
}

static ssize_t get_k_value_gyro_f(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 data[GYRO_CALIBRATOR_LEN];

	return get_k_value(mcu_data, CW_GYRO, buf, data, sizeof(data));
}

static ssize_t get_k_value_light_f(struct device *dev,
				   struct device_attribute *attr, char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 data[LIGHT_CALIBRATOR_LEN] = {0};

	if (cwmcu_get_calibrator(mcu_data, CW_LIGHT, data, sizeof(data)) < 0) {
		E("%s: Get LIGHT Calibrator fails\n", __func__);
		return -EIO;
	}
	return scnprintf(buf, PAGE_SIZE, "%x %x %x %x\n", data[0], data[1],
			 data[2], data[3]);
}

static ssize_t get_k_value_proximity_f(struct device *dev,
                                       struct device_attribute *attr, char *buf)
{
        struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
        u8 data[PROXIMITY_CALIBRATOR_LEN] = {0};

        if(cwmcu_get_calibrator(mcu_data, CW_PROXIMITY, data, sizeof(data)) < 0) {
                E("%s: Get PROXIMITY Calibrator fails\n", __func__);
                return -EIO;
	}
        return scnprintf(buf, PAGE_SIZE, "%x %x %x %x\n", data[0], data[1],
                        data[2], data[3]);
}

static ssize_t get_k_value_barometer_f(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 data[PRESSURE_CALIBRATOR_LEN];

	return get_k_value(mcu_data, CW_PRESSURE, buf, data, sizeof(data));
}

static ssize_t sensor_placement_store(struct device *dev,
                        struct device_attribute *attr,
                        const char *buf,
                        size_t count)
{
	char *str_buf;
	char *running;
	long sensors_id = 0;
	long placement = 0;
	uint8_t position, cali_data_from_fw;
	int i, rc;
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);

	str_buf = kstrndup(buf, count, GFP_KERNEL);
	if (str_buf == NULL) {
		E("%s: cannot allocate buffer\n", __func__);
		return -ENOMEM;
	}
	running = str_buf;

	for (i = 0; i < 2; i++) {
		int error;
		char *token;

		token = strsep(&running, " ");

		if (token && (i == 0))
			error = kstrtol(token, 10, &sensors_id);
		else {
			if (token == NULL) {
				placement = sensors_id;
				sensors_id = 0;
			} else
				error = kstrtol(token, 10, &placement);
		}
		if (error) {
			E("%s: kstrtol fails, error = %d, i = %d\n",
				__func__, error, i);
			kfree(str_buf);
			return error;
		}
	}
	switch(sensors_id){
		case(1):
			position = GENSOR_POSITION;
			break;
		case(2):
			position = COMPASS_POSITION;
			break;
		case(3):
			position = GYRO_POSITION;
			break;
		default:
			E("Sensor id %d is not in range, please in range 1-3\n", (int)sensors_id);
			return count;
	}

	for (i = 0; i < 3; i++) {
		CWMCU_i2c_write_power(mcu_data, position,
				(uint8_t*)&placement,
				1);
		rc = CWMCU_i2c_read_power(mcu_data, position,
			       &cali_data_from_fw, 1);
		if (rc >= 0) {
			if (cali_data_from_fw == placement)
				break;
			else {
				I("%s: cali_data_from_fw = 0x%x, "
				  "sensors_axes = 0x%x\n",
				  __func__, cali_data_from_fw,
				  (unsigned int)placement);
			}
		} else {
			I("%s: sensors_id %d  i2c read fails, rc = %d\n",
			  __func__, (int)sensors_id, rc);
		}
	}

	return count;
}


static ssize_t trigger_mcu_crash(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    int error;
    u8 data;
    long data_temp = 0;

    error = kstrtol(buf, 10, &data_temp);
    if (error) {
        E("%s: kstrtol fails, error = %d\n", __func__, error);
        return error;
    }

    data = data_temp;

    I("%s(%d): crash_type:%d\n", __func__, __LINE__, data);
    if (s_mcu_data) {
        error = CWMCU_i2c_write_power(s_mcu_data, CW_I2C_REG_TRIGGER_CRASH, &data, 1);
        if (error < 0) {
            E("%s: error = %d\n", __func__, error);
            return -EIO;
        }
    }

    return count;
}

static ssize_t trigger_mcu_wakeup(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    int error;
    u8 data;
    long data_temp = 0;

    error = kstrtol(buf, 10, &data_temp);
    if (error) {
        E("%s: kstrtol fails, error = %d\n", __func__, error);
        return error;
    }

    data = data_temp;

    I("%s: mcu_wakeup:%d\n", __func__, data);
    if (s_mcu_data) {
        cwmcu_powermode_switch(s_mcu_data, !!data);
    }

    return count;
}

static ssize_t set_vibrate_ms(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    int error;
    u8 data;
    long data_temp = 0;

    error = kstrtol(buf, 10, &data_temp);
    if (error) {
        E("%s: kstrtol fails, error = %d\n", __func__, error);
        return error;
    }

    data = data_temp;

    if (!s_mcu_data) {
        W("%s: probe not completed\n", __func__);
        return -1;
    }

    I("%s: vibrate_ms%d\n", __func__, data);
    s_mcu_data->vibrate_ms = data;

    return count;
}

static ssize_t sensors_self_test_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
    int error;
    u8 sensors_id;
	u8 sensors_status_reg;
	u8 data = 0;
    long data_temp = 0;

    error = kstrtol(buf, 10, &data_temp);
    if (error) {
        E("%s: kstrtol fails, error = %d\n", __func__, error);
        return error;
    }

    sensors_id = data_temp;
	switch(sensors_id){
		case(0):
			sensors_status_reg = G_SENSORS_STATUS;
			break;
		case(1):
			sensors_status_reg = ECOMPASS_SENSORS_STATUS;
			break;
		case(2):
			sensors_status_reg = GYRO_SENSORS_STATUS;
			break;
		default:
			E("Sensor id %d is not in range, please in range 0-2\n", (int)sensors_id);
			return count;
	}

	if (s_mcu_data) {
		data = 0x20;
        error = CWMCU_i2c_write_power(s_mcu_data, sensors_status_reg, &data, 1);
        if (error < 0) {
            E("%s: error = %d\n", __func__, error);
            return -EIO;
        }
    }

    return count;
}

static ssize_t sensors_self_test_show(struct device *dev,
                                 struct device_attribute *attr,
                                 char *buf)
{
	int error;
	u8 test_result[3] = {0};

	if (s_mcu_data) {
		error = CWMCU_i2c_read_power(s_mcu_data, G_SENSORS_STATUS, &test_result[0], 1);
        if (error < 0) {
            E("%s: error = %d\n", __func__, error);
            return -EIO;
        }
		error = CWMCU_i2c_read_power(s_mcu_data, ECOMPASS_SENSORS_STATUS, &test_result[1], 1);
        if (error < 0) {
            E("%s: error = %d\n", __func__, error);
            return -EIO;
        }
		error = CWMCU_i2c_read_power(s_mcu_data, GYRO_SENSORS_STATUS, &test_result[2], 1);
        if (error < 0) {
            E("%s: error = %d\n", __func__, error);
            return -EIO;
        }
    }
	return snprintf(buf, PAGE_SIZE, "Sensors self test result[0x%02x 0x%02x 0x%02x]\n", test_result[0], test_result[1], test_result[2]);
}

static int CWMCU_i2c_read_power(struct cwmcu_data *mcu_data,
			 u8 reg_addr, void *data, u8 len)
{
	int ret;

	cwmcu_powermode_switch(mcu_data, 1);
	ret = CWMCU_i2c_read(mcu_data, reg_addr, data, len);
	cwmcu_powermode_switch(mcu_data, 0);
	return ret;
}

static int CWMCU_i2c_write_power(struct cwmcu_data *mcu_data,
				 u8 reg_addr, const void *data, u8 len)
{
	int ret;

	cwmcu_powermode_switch(mcu_data, 1);
	ret = CWMCU_i2c_write(mcu_data, reg_addr, data, len, 1);
	cwmcu_powermode_switch(mcu_data, 0);
	return ret;
}

static int CWMCU_i2c_write_block_power(struct cwmcu_data *mcu_data,
				 u8 reg_addr, u8 *data, u8 len)
{
	int ret;

	cwmcu_powermode_switch(mcu_data, 1);
	ret = CWMCU_i2c_write_block(mcu_data, reg_addr, data, len);
	cwmcu_powermode_switch(mcu_data, 0);
	return ret;
}

static ssize_t get_light_kadc(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 data[4] = {0};
	u16 light_gadc;
	u16 light_kadc;

	CWMCU_i2c_read_power(mcu_data, LIGHT_SENSORS_CALIBRATION_DATA, data,
			     sizeof(data));

	light_gadc = (data[1] << 8) | data[0];
	light_kadc = (data[3] << 8) | data[2];
	return scnprintf(buf, PAGE_SIZE, "gadc = 0x%x, kadc = 0x%x", light_gadc,
			 light_kadc);
}

static ssize_t sensor_hw_id_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 gs_hw_id, mag_hw_id, gyro_hw_id = 0;

	CWMCU_i2c_read_power(mcu_data, CW_I2C_REG_GSENSOR_HW_ID, &gs_hw_id, sizeof(gs_hw_id));
	CWMCU_i2c_read_power(mcu_data, CW_I2C_REG_COMPASS_HW_ID, &mag_hw_id, sizeof(mag_hw_id));
	CWMCU_i2c_read_power(mcu_data, CW_I2C_REG_GYRO_HW_ID, &gyro_hw_id, sizeof(gyro_hw_id));

	return scnprintf(buf, PAGE_SIZE,
			 "Sensor hw_id, acc = %d, mag = %d, gyro = %d\n",
			 gs_hw_id, mag_hw_id, gyro_hw_id);
}

static ssize_t get_firmware_version(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 firmware_version[FW_VER_COUNT] = {0};

	CWMCU_i2c_read_power(mcu_data, FIRMWARE_VERSION, firmware_version,
			     sizeof(firmware_version));

	return scnprintf(buf, PAGE_SIZE,
			 "Firmware Architecture version %u, "
			 "Sense version %u, Cywee lib version %u,"
			 " Water number %u"
			 ", Active Engine %u, Project Mapping %u\n",
			 firmware_version[0], firmware_version[1],
			 firmware_version[2], firmware_version[3],
			 firmware_version[4], firmware_version[5]);
}

static ssize_t get_firmware_info(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 firmware_version[FW_VER_COUNT] = {0};
	u8 firmware_info[6] = {0};

	CWMCU_i2c_read_power(mcu_data, FIRMWARE_VERSION, firmware_version,
			sizeof(firmware_version));

	CWMCU_i2c_read_power(mcu_data, FIRMWARE_INFO, firmware_info,
			sizeof(firmware_info));

	return scnprintf(buf, PAGE_SIZE,
			 "%03u.%03u.%03u.%03u.%03u.%03u: "
			 "Jenkins build number %u, "
			 "Build time(hh:mm) %02u:%02u, "
			 "CW branch %u, CW mcu type %u\n",
			 firmware_version[0], firmware_version[1],
			 firmware_version[2], firmware_version[3],
			 firmware_version[4], firmware_version[5],
			 (firmware_info[0] << 8) | firmware_info[1],
			 firmware_info[2], firmware_info[3],
			 firmware_info[4], firmware_info[5]);
}

#ifdef CONFIG_AK8789_HALLSENSOR
static int hallsensor_status_handler_func(struct notifier_block *this,
	unsigned long status, void *unused)
{
	int pole = 0, pole_value = 0;

	if (!s_mcu_data) {
		E("%s: s_mcu_data == NULL\n", __func__);
		return NOTIFY_OK;
	}

	if (MCU_IN_DLOAD() || MCU_IN_BOOTLOADER()) {
		I("%s skip, s_mcu_state:%d\n", __func__, s_mcu_state);
		return NOTIFY_OK;
	}

	pole_value = status & 0x01;
	pole = (status & 0x02) >> HALL_POLE_BIT;
	I("%s: %s[%s]\n",
	  __func__,
	  pole ? "att_s" : "att_n",
	  pole_value ? "Near" : "Far");

	if (pole == HALL_POLE_S) {
		u8 dotview_status;
		int rc;

		if (pole_value == HALL_FAR)
			dotview_status = DOTVIEW_NO_COVER;
		else
			dotview_status = DOTVIEW_COVER;

		s_mcu_data->dotview_st = dotview_status;

		rc = CWMCU_i2c_write_power(s_mcu_data,
					   CW_I2C_REG_DOTVIEW_STATUS,
					   &dotview_status, 1);
		if (rc) {
			E("%s: CWMCU_i2c_write fails, rc = %d\n",
			  __func__, rc);

			s_mcu_data->w_hall_inform_mcu = true;
			queue_work(s_mcu_data->mcu_wq,
				   &s_mcu_data->one_shot_work);
		}
	}

	return NOTIFY_OK;
}

static struct notifier_block hallsensor_status_handler = {
	.notifier_call = hallsensor_status_handler_func,
};
#endif

static ssize_t get_barometer(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 data[6] = {0};

	CWMCU_i2c_read_power(mcu_data, CWSTM32_READ_Pressure, data,
			     sizeof(data));

	return scnprintf(buf, PAGE_SIZE, "%x %x %x %x\n", data[0], data[1],
					 data[2], data[3]);
}

static ssize_t get_ps_canc(struct device *dev, struct device_attribute *attr,
                               char *buf)
{
        struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
        u16 PS_canc, PS_th;
        u8 data[6] = {0};
        int ret = 0;

        ret = CWMCU_i2c_read_power(mcu_data, CW_I2C_REG_SENSORS_CALIBRATOR_GET_DATA_PROXIMITY,
							 data, 6);
        PS_canc = (data[1] << 8) | data[0];
        PS_th   = (data[3] << 8) | data[2];
        D("INTE_PS1_CANC = (0x%04X),  INTE_PS2_CANC = (0x%04X)\n", PS_canc, PS_th);

	if (((mcu_data->ps_kheader & (0x50 << 24)) == (0x50 << 24)) &&
		 ((mcu_data->ps_kheader & (0x53 << 16)) == (0x53 << 16))) {
                ret = snprintf(buf, PAGE_SIZE, "P-sensor calibrated,"
                                "INTE_PS1_CANC = (0x%04X), "
                                "INTE_PS2_CANC = (0x%04X)\n",
                                PS_canc, PS_th);
        } else
                ret = snprintf(buf, PAGE_SIZE, "P-sensor NOT calibrated,"
                                "INTE_PS1_CANC = (0x%04X), "
                                "INTE_PS2_CANC = (0x%04X)\n",
                                PS_canc, PS_th);
        return ret;
}

static ssize_t get_proximity(struct device *dev, struct device_attribute *attr,
                            char *buf)
{
        struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
        u8 data[REPORT_EVENT_PROXIMITY_LEN] = {0};
        u16 proximity_adc;

        CWMCU_i2c_read_power(mcu_data, CWSTM32_READ_Proximity,
				 data, sizeof(data));

        proximity_adc = (data[2] << 8) | data[1];
        return snprintf(buf, PAGE_SIZE, "%x %x \n",
				 data[0], proximity_adc);
}

static ssize_t get_acceleration_polling(struct device *dev, struct device_attribute *attr,
                                char *buf)
{
        struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
        u8 data[REPORT_EVENT_MOTION_LEN] = {0};
	u8 acceleration_enable;
	u64 sensors_bit;
	int rc;

	/*TODO: Enable light sensor*/
        sensors_bit = 1LL << CW_ACCELERATION;
        mcu_data->enabled_list &= ~sensors_bit;
        mcu_data->enabled_list |= sensors_bit;

        acceleration_enable = (u8)(mcu_data->enabled_list);
        rc = CWMCU_i2c_write_power(mcu_data, CWSTM32_ENABLE_REG,
				 &acceleration_enable, 1);
        if (rc) {
                E("%s: CWMCU_i2c_write fails, rc = %d\n",
                  __func__, rc);
		mutex_unlock(&mcu_data->group_i2c_lock);
                return -EIO;
        }

        CWMCU_i2c_read_power(mcu_data, CWSTM32_READ_Acceleration,
				 data, sizeof(data));

        return snprintf(buf, PAGE_SIZE, "G-sensor raw data [%x %x %x %x %x %x]\n",
				 data[0], data[1], data[2], data[3], data[4], data[5]);
}

static ssize_t get_magnetic_polling(struct device *dev, struct device_attribute *attr,
                                char *buf)
{
        struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
        u8 data[REPORT_EVENT_MOTION_LEN] = {0};
	u8 magnetic_enable;
	u64 sensors_bit;
	int rc;

	/*TODO: Enable light sensor*/
        sensors_bit = 1LL << CW_MAGNETIC;
        mcu_data->enabled_list &= ~sensors_bit;
        mcu_data->enabled_list |= sensors_bit;

        magnetic_enable = (u8)(mcu_data->enabled_list);
        rc = CWMCU_i2c_write_power(mcu_data, CWSTM32_ENABLE_REG,
				 &magnetic_enable, 1);
        if (rc) {
                E("%s: CWMCU_i2c_write fails, rc = %d\n",
                  __func__, rc);
		mutex_unlock(&mcu_data->group_i2c_lock);
                return -EIO;
        }

        CWMCU_i2c_read_power(mcu_data, CWSTM32_READ_Magnetic,
				 data, sizeof(data));

        return snprintf(buf, PAGE_SIZE, "Compass raw data [%x %x %x %x %x %x]\n",
				 data[0], data[1], data[2], data[3], data[4], data[5]);
}

static ssize_t get_gyro_polling(struct device *dev, struct device_attribute *attr,
                                char *buf)
{
        struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
        u8 data[REPORT_EVENT_MOTION_LEN] = {0};
	u8 gyro_enable;
	u64 sensors_bit;
	int rc;

	/*TODO: Enable light sensor*/
        sensors_bit = 1LL << CW_GYRO;
        mcu_data->enabled_list &= ~sensors_bit;
        mcu_data->enabled_list |= sensors_bit;

        gyro_enable = (u8)(mcu_data->enabled_list);
        rc = CWMCU_i2c_write_power(mcu_data, CWSTM32_ENABLE_REG,
				 &gyro_enable, 1);
        if (rc) {
                E("%s: CWMCU_i2c_write fails, rc = %d\n",
                  __func__, rc);
		mutex_unlock(&mcu_data->group_i2c_lock);
                return -EIO;
        }

        CWMCU_i2c_read_power(mcu_data, CWSTM32_READ_Gyro,
				 data, sizeof(data));

        return snprintf(buf, PAGE_SIZE, "Gyro sensor raw data [%x %x %x %x %x %x]\n",
				 data[0], data[1], data[2], data[3], data[4], data[5]);
}

static ssize_t get_proximity_polling(struct device *dev, struct device_attribute *attr,
                                char *buf)
{
        struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
        u8 data[REPORT_EVENT_PROXIMITY_LEN] = {0};
        u8 data_polling_enable = 0;
	u8 proximity_enable;
        u16 proximity_adc;
	u64 sensors_bit;
	int rc;

        sensors_bit = CW_MCU_INT_BIT_PROXIMITY;
        mcu_data->enabled_list &= ~sensors_bit;
        mcu_data->enabled_list |= sensors_bit;

        proximity_enable = (u8)(mcu_data->enabled_list);
        rc = CWMCU_i2c_write_power(mcu_data, CWSTM32_ENABLE_REG,
				 &proximity_enable, 1);
        if (rc) {
                E("%s: CWMCU_i2c_write fails, rc = %d\n",
                  __func__, rc);
		mutex_unlock(&mcu_data->group_i2c_lock);
                return -EIO;
        }

        data_polling_enable = CW_MCU_BIT_PROXIMITY_POLLING;

        CWMCU_i2c_write_power(mcu_data, PROXIMITY_SENSORS_STATUS,
				 &data_polling_enable, 1);
        CWMCU_i2c_read_power(mcu_data, CWSTM32_READ_Proximity,
				 data, sizeof(data));
        proximity_adc = (data[2] << 8) | data[1];

        return snprintf(buf, PAGE_SIZE, "ADC[0x%02X] status is %d\n",
				 proximity_adc, data[0]);
}

static ssize_t get_light_polling(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 data[REPORT_EVENT_COMMON_LEN] = {0};
	u8 data_polling_enable;
	u8 light_enable;
	u16 light_adc;
	u64 sensors_bit;
	int rc;

	mutex_lock(&mcu_data->group_i2c_lock);

        /*TODO: Enable light sensor*/
        sensors_bit = CW_MCU_INT_BIT_LIGHT;
        mcu_data->enabled_list &= ~sensors_bit;
        mcu_data->enabled_list |= sensors_bit;

        light_enable = (u8)(mcu_data->enabled_list);
        rc = CWMCU_i2c_write_power(mcu_data, CWSTM32_ENABLE_REG,
				 &light_enable, 1);
        if (rc) {
                E("%s: CWMCU_i2c_write fails, rc = %d\n",
                  __func__, rc);
		mutex_unlock(&mcu_data->group_i2c_lock);
                return -EIO;
        }

	data_polling_enable = CW_MCU_BIT_LIGHT_POLLING;

	rc = CWMCU_i2c_write_power(mcu_data, LIGHT_SENSORS_STATUS,
			&data_polling_enable, 1);
	if (rc < 0) {
		mutex_unlock(&mcu_data->group_i2c_lock);
		E("%s: write fail, rc = %d\n", __func__, rc);
		return rc;
	}
	rc = CWMCU_i2c_read_power(mcu_data, CWSTM32_READ_Light,
				 data, sizeof(data));
	if (rc < 0) {
		mutex_unlock(&mcu_data->group_i2c_lock);
		E("%s: read fail, rc = %d\n", __func__, rc);
		return rc;
	}
	mutex_unlock(&mcu_data->group_i2c_lock);

	light_adc = (data[2] << 8) | data[1];

	return scnprintf(buf, PAGE_SIZE, "ADC[0x%04X] => level %u\n",
				 light_adc, data[0]);
}

static ssize_t get_ls_mechanism(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	return snprintf(buf, PAGE_SIZE, "%u %u\n", mcu_data->als_polling
					, mcu_data->als_lux_ratio_d);
}

static ssize_t read_mcu_data(struct device *dev, struct device_attribute *attr,
			     const char *buf, size_t count)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	int i;
	u8 reg_addr;
	u8 len;
	long data_temp[2] = {0};
	u8 mcu_rdata[128] = {0};
	char *str_buf;
	char *running;

	str_buf = kstrndup(buf, count, GFP_KERNEL);
	if (str_buf == NULL) {
		E("%s: cannot allocate buffer\n", __func__);
		return -ENOMEM;
	}
	running = str_buf;

	for (i = 0; i < ARRAY_SIZE(data_temp); i++) {
		int error;
		char *token;

		token = strsep(&running, " ");

		if (token && (i == 0))
			error = kstrtol(token, 16, &data_temp[i]);
		else {
			if (token == NULL) {
				data_temp[i] = 1;
				D("%s: token 2 missing\n", __func__);
				break;
			} else
				error = kstrtol(token, 10, &data_temp[i]);
		}
		if (error) {
			E("%s: kstrtol fails, error = %d, i = %d\n",
			  __func__, error, i);
			kfree(str_buf);
			return error;
		}
	}
	kfree(str_buf);

	/* TESTME for changing array to variable */
	reg_addr = (u8)(data_temp[0]);
	len = (u8)(data_temp[1]);

	if (len < sizeof(mcu_rdata)) {
		CWMCU_i2c_read_power(mcu_data, reg_addr, mcu_rdata, len);

		for (i = 0; i < len; i++)
			I("read mcu reg_addr = 0x%x, reg[%u] = 0x%x\n",
				reg_addr, (reg_addr + i), mcu_rdata[i]);
	} else
		E("%s: len = %u, out of range\n", __func__, len);

	return count;
}

static inline bool retry_exhausted(struct cwmcu_data *mcu_data)
{
	return ((mcu_data->i2c_total_retry > RETRY_TIMES) ||
		(mcu_data->i2c_latch_retry >= LATCH_TIMES));
}

static inline void retry_reset(struct cwmcu_data *mcu_data)
{
	mcu_data->i2c_total_retry = 0;
	mcu_data->i2c_latch_retry = 0;
}

static int CWMCU_i2c_write(struct cwmcu_data *mcu_data,
			  u8 reg_addr, const void *data, u8 len, bool mutex_lock_i2c)
{
	s32 write_res;
	int i;
	const u8 *u8_data = data;

	if (MCU_IN_DLOAD() || MCU_IN_BOOTLOADER()) {
		I("%s[%d], s_mcu_state:%d, return %d\n", __func__, __LINE__, s_mcu_state, -ENOTCONN);
		if (!in_interrupt())msleep(100);
		return -ENOTCONN;
	}

	if (DEBUG_DISABLE) {
		mcu_data->disable_access_count++;
		if ((mcu_data->disable_access_count % 100) == 0)
			I("%s: DEBUG_DISABLE = %d\n", __func__, DEBUG_DISABLE);
		return len;
	}

	if (mcu_data->is_block_i2c) {
		if (time_after(jiffies,
			       mcu_data->reset_jiffies + RESET_PERIOD)) {
			gpio_direction_input(mcu_data->gpio_reset);
			I("%s: gpio_reset = %d\n", __func__,
			  gpio_get_value_cansleep(mcu_data->gpio_reset));

			if (mcu_chip_mode_get(mcu_data) ==
			    MCU_CHIP_MODE_BOOTLOADER)
				msleep(100);
			else {
				/* HUB need at least 500ms to be ready */
				usleep_range(500000, 1000000);
			}

			mcu_data->is_block_i2c = false;
		}
		return len;
	}

	if (atomic_read(&mcu_data->suspended))
		return len;
	if(mutex_lock_i2c)
		mutex_lock(&s_activated_i2c_lock);
	if (retry_exhausted(mcu_data)) {
		if(mutex_lock_i2c)
			mutex_unlock(&s_activated_i2c_lock);
		D("%s: mcu_data->i2c_total_retry = %d, i2c_latch_retry = %d\n",
		  __func__,
		  mcu_data->i2c_total_retry, mcu_data->i2c_latch_retry);
		/* Try to recover HUB in low CPU utilization */
		mcu_data->w_activated_i2c = true;
		queue_work(mcu_data->mcu_wq, &mcu_data->one_shot_work);
		return -EIO;
	}

	for (i = 0; i < len; i++) {
		while (!retry_exhausted(mcu_data)) {
			write_res = CWMCU_do_write(mcu_data->mcu_client,
						  reg_addr, u8_data[i]);
			if (write_res >= 0) {
				retry_reset(mcu_data);
				break;
			}
			gpio_make_falling_edge(mcu_data->gpio_wake_mcu);
			if (write_res == LATCH_ERROR_NO)
				mcu_data->i2c_latch_retry++;
			mcu_data->i2c_total_retry++;
			E(
			  "%s: i2c write error, write_res = %d, total_retry ="
			  " %d, latch_retry = %d, addr = 0x%x, val = 0x%x\n",
			  __func__, write_res, mcu_data->i2c_total_retry,
			  mcu_data->i2c_latch_retry, reg_addr, u8_data[i]);
		}

		if (retry_exhausted(mcu_data)) {
			if(mutex_lock_i2c)
				mutex_unlock(&s_activated_i2c_lock);
			E("%s: mcu_data->i2c_total_retry = %d, "
			  "i2c_latch_retry = %d, EIO\n", __func__,
			  mcu_data->i2c_total_retry, mcu_data->i2c_latch_retry);
			return -EIO;
		}
	}
	if(mutex_lock_i2c)
		mutex_unlock(&s_activated_i2c_lock);
	return 0;
}

static int CWMCU_i2c_write_block(struct cwmcu_data *mcu_data,
			  u8 reg_addr, u8 *data, u8 len)
{
	s32 write_res;

	if (MCU_IN_DLOAD() || MCU_IN_BOOTLOADER()) {
		I("%s[%d], s_mcu_state:%d, return %d\n", __func__, __LINE__, s_mcu_state, -ENOTCONN);
		if (!in_interrupt())msleep(100);
		return -ENOTCONN;
	}

	if (DEBUG_DISABLE) {
		mcu_data->disable_access_count++;
		if ((mcu_data->disable_access_count % 100) == 0)
			I("%s: DEBUG_DISABLE = %d\n", __func__, DEBUG_DISABLE);
		return len;
	}

	if (mcu_data->is_block_i2c) {
		if (time_after(jiffies,
			       mcu_data->reset_jiffies + RESET_PERIOD)) {
			gpio_direction_input(mcu_data->gpio_reset);
			I("%s: gpio_reset = %d\n", __func__,
			  gpio_get_value_cansleep(mcu_data->gpio_reset));

			if (mcu_chip_mode_get(mcu_data) ==
			    MCU_CHIP_MODE_BOOTLOADER)
				msleep(100);
			else {
				/* HUB need at least 500ms to be ready */
				usleep_range(500000, 1000000);
			}

			mcu_data->is_block_i2c = false;
		}
		return len;
	}

	if (atomic_read(&mcu_data->suspended))
		return len;

	mutex_lock(&s_activated_i2c_lock);
	if (retry_exhausted(mcu_data)) {
		mutex_unlock(&s_activated_i2c_lock);
		D("%s: mcu_data->i2c_total_retry = %d, i2c_latch_retry = %d\n",
		  __func__,
		  mcu_data->i2c_total_retry, mcu_data->i2c_latch_retry);
		/* Try to recover HUB in low CPU utilization */
		mcu_data->w_activated_i2c = true;
		queue_work(mcu_data->mcu_wq, &mcu_data->one_shot_work);
		return -EIO;
	}

	if (1) {
		while (!retry_exhausted(mcu_data)) {
			write_res = CWMCU_do_write_block(mcu_data->mcu_client,
						  reg_addr, len, data);
			if (write_res >= 0) {
				retry_reset(mcu_data);
				break;
			}
			gpio_make_falling_edge(mcu_data->gpio_wake_mcu);
			if (write_res == LATCH_ERROR_NO)
				mcu_data->i2c_latch_retry++;
			mcu_data->i2c_total_retry++;
			E(
			  "%s: i2c write error, write_res = %d, total_retry ="
			  " %d, latch_retry = %d\n", __func__, write_res,
				mcu_data->i2c_total_retry,
				mcu_data->i2c_latch_retry);
		}

		if (retry_exhausted(mcu_data)) {
			mutex_unlock(&s_activated_i2c_lock);
			E("%s: mcu_data->i2c_total_retry = %d, "
			  "i2c_latch_retry = %d, EIO\n", __func__,
			  mcu_data->i2c_total_retry, mcu_data->i2c_latch_retry);
			return -EIO;
		}
	}

	mutex_unlock(&s_activated_i2c_lock);

	return 0;
}

static int CWMCU_i2c_multi_write(struct cwmcu_data *mcu_data,
			  u8 reg_addr, const void *data, u8 len)
{
	int rc, i;
	const u8 *u8_data = data;

	mutex_lock(&mcu_data->group_i2c_lock);

	for (i = 0; i < len; i++) {
		rc = CWMCU_i2c_write(mcu_data, reg_addr, &u8_data[i], 1, 1);
		if (rc) {
			mutex_unlock(&mcu_data->group_i2c_lock);
			E("%s: CWMCU_i2c_write fails, rc = %d, i = %d\n",
			  __func__, rc, i);
			return -EIO;
		}
	}

	mutex_unlock(&mcu_data->group_i2c_lock);
	return 0;
}

static int cwmcu_set_g_sensor_kvalue(struct cwmcu_data *mcu_data)
{
	u8 *gs_data = (u8 *)&mcu_data->gs_kvalue; /* gs_kvalue is u32 */
	int rc = 0;

	if (gs_data[3] == 0x67) {
		__be32 be32_gs_data = cpu_to_be32(mcu_data->gs_kvalue);
		gs_data = (u8 *)&be32_gs_data;

		rc = CWMCU_i2c_write_power(mcu_data,
			CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_ACC,
			gs_data + 1, ACC_CALIBRATOR_LEN);
		mcu_data->gs_calibrated = 1;
		I("Set g-sensor kvalue (x, y, z) = (0x%x, 0x%x, 0x%x)\n",
			gs_data[1], gs_data[2], gs_data[3]);
	}
	return rc;
}

static int cwmcu_set_sensor_kvalue(struct cwmcu_data *mcu_data)
{
	/* Write single Byte because firmware can't write multi bytes now */

	u8 *gy_data = (u8 *)&mcu_data->gy_kvalue; /* gy_kvalue is u32 */
	u8 *bs_data = (u8 *)&mcu_data->bs_kvalue; /* bs_kvalue is u32 */
        u8 als_goldh = (mcu_data->als_goldh == 0) ? 0x0A : (mcu_data->als_goldh);
        u8 als_goldl = (mcu_data->als_goldl == 0) ? 0x38 : (mcu_data->als_goldl);
	u8 als_datal = 0, als_datah = 0, als_levell = 0, als_levelh = 0;
	u8 ps_cancl = 0, ps_canch = 0, ps_thdl = 0, ps_thdh = 0, ps_thd_addl = 0, ps_thd_addh = 0;
	int i = 0;
	u8 firmware_version[FW_VER_COUNT] = {0};
	u8 firmware_info[6] = {0};
	u8 gs_hw_id, mag_hw_id, gyro_hw_id = 0;

	mcu_data->gs_calibrated = 0;
	mcu_data->gy_calibrated = 0;
	mcu_data->ls_calibrated = 0;
	mcu_data->bs_calibrated = 0;

	CWMCU_i2c_read_power(mcu_data, FIRMWARE_VERSION, firmware_version,
		       sizeof(firmware_version));

	I(
	  "Firmware Architecture version %u, Sense version %u,"
	  " Cywee lib version %u, Water number %u"
	  ", Active Engine %u, Project Mapping %u\n",
		firmware_version[0], firmware_version[1], firmware_version[2],
		firmware_version[3], firmware_version[4], firmware_version[5]);

	CWMCU_i2c_read_power(mcu_data, FIRMWARE_INFO, firmware_info,
			sizeof(firmware_info));
	I(
        "Jenkins build number %u, "
        "Build time(hh:mm) %02u:%02u, "
        "CW branch %u, CW mcu type %u\n",
        (firmware_info[0] << 8) | firmware_info[1],
        firmware_info[2], firmware_info[3],
        firmware_info[4], firmware_info[5]);
	mcu_data->jenkins_build_ver = (firmware_info[0] << 8) | firmware_info[1];

	CWMCU_i2c_read_power(mcu_data, CW_I2C_REG_GSENSOR_HW_ID, &gs_hw_id, sizeof(gs_hw_id));
	CWMCU_i2c_read_power(mcu_data, CW_I2C_REG_COMPASS_HW_ID, &mag_hw_id, sizeof(mag_hw_id));
	CWMCU_i2c_read_power(mcu_data, CW_I2C_REG_GYRO_HW_ID, &gyro_hw_id, sizeof(gyro_hw_id));
	I("Sensor hw_id, acc = %d, mag = %d, gyro = %d\n", gs_hw_id, mag_hw_id, gyro_hw_id);

	cwmcu_set_g_sensor_kvalue(mcu_data);

	if (gy_data[3] == 0x67) {
		__be32 be32_gy_data = cpu_to_be32(mcu_data->gy_kvalue);
		gy_data = (u8 *)&be32_gy_data;

		CWMCU_i2c_write(mcu_data,
			CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_GYRO,
			gy_data + 1, GYRO_CALIBRATOR_LEN, 1);
		mcu_data->gy_calibrated = 1;
		D("Set gyro-sensor kvalue (x, y, z) = (0x%x, 0x%x, 0x%x)\n",
			gy_data[1], gy_data[2], gy_data[3]);
	}

	if ((mcu_data->als_kvalue & 0x6DA50000) == 0x6DA50000) {
		CWMCU_i2c_write_power(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_LIGHT,
				&als_goldl, 1);
		CWMCU_i2c_write_power(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_LIGHT,
				&als_goldh, 1);
		als_datal = (mcu_data->als_kvalue) & 0xFF;
		CWMCU_i2c_write_power(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_LIGHT,
				&als_datal, 1);
		als_datah = (mcu_data->als_kvalue >>  8) & 0xFF;
		CWMCU_i2c_write_power(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_LIGHT,
				&als_datah, 1);
		mcu_data->ls_calibrated = 1;
		I("Set light-sensor kvalue is %x %x, gvalue is %x %x\n"
				, als_datah, als_datal, als_goldh, als_goldl);
	}
	else {
		CWMCU_i2c_write_power(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_LIGHT,
				&als_goldl, 1);
		CWMCU_i2c_write_power(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_LIGHT,
				&als_goldh, 1);
		als_datal = als_goldl;
		CWMCU_i2c_write_power(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_LIGHT,
				&als_datal, 1);
		als_datah = als_goldh;
		CWMCU_i2c_write_power(mcu_data,
				CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_LIGHT,
				&als_datah, 1);
		mcu_data->ls_calibrated = 0;
		I("Set light-sensor kvalue is %x %x, gvalue is %x %x\n"
				, als_datah, als_datal, als_goldh, als_goldl);
	}

	for (i = 0; i<10; i++) {
                als_levell = *(mcu_data->als_levels + i) & 0xFF;
                als_levelh = (*(mcu_data->als_levels + i) >> 8) & 0xFF;
                CWMCU_i2c_write_power(mcu_data,
                                CW_I2C_REG_SENSORS_SET_LEVEL_LIGHT,
                                &als_levell, 1);
                CWMCU_i2c_write_power(mcu_data,
                                CW_I2C_REG_SENSORS_SET_LEVEL_LIGHT,
                                &als_levelh, 1);
               I("Set light-sensor level is %d\n", ((als_levelh << 8) | als_levell));
	}

	if((mcu_data->ps_kheader & 0x50530000) == 0x50530000) {
		if (((mcu_data->ps_kvalue >> 16) & 0xFFFF) == 0xFFFF) {
			ps_cancl = (mcu_data->ps_kvalue) & 0xFF;
                        ps_canch = 0x00;
			CWMCU_i2c_write_power(mcu_data,
					CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,
					&ps_cancl, 1);
			CWMCU_i2c_write_power(mcu_data,
					CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,
                                       	&ps_canch, 1);

                       	if( (mcu_data->ps_kvalue & 0xFF00) != 0X0000) {
				ps_thdl = (mcu_data->ps_kvalue >> 8) & 0xFF;
                               	ps_thdh = 0x00;
                       	}
                       	else {
				ps_thdl = mcu_data->ps_thd_fixed & 0xFF;
                               	ps_thdh = 0x00;
                       	}
			CWMCU_i2c_write_power(mcu_data,
					CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,
					&ps_thdl, 1);
			CWMCU_i2c_write_power(mcu_data,
					CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,
					&ps_thdh, 1);
			ps_thd_addl = mcu_data->ps_thd_add & 0xFF;
			ps_thd_addh = (mcu_data->ps_thd_add >> 8) & 0xFF;
                        CWMCU_i2c_write(mcu_data,
					CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,
					&ps_thd_addl, 1, 1);
                        CWMCU_i2c_write(mcu_data,
					CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,
					&ps_thd_addh, 1, 1);
                        I("set proximity-sensor kvalue is %x %x %x %x, "
                               "ps_thd_add is %x\n",
                               ps_cancl, ps_canch, ps_thdl, ps_thdh,
                               ((ps_thd_addh << 8) | ps_thd_addl) );
#if 0
                        I("set proximity-sensor kvalue is %x %x %x %x\n",
                               ps_cancl, ps_canch, ps_thdl, ps_thdh);

                       	CWMCU_i2c_read(mcu_data,
                                       CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,
                                       debug_data, 6);
                        I("[AUSTIN DEBUG]  canc: 0x%X, thd: 0x%X, thd_add: 0x%X\n",
                               ((debug_data[1] << 8) | debug_data[0]),
                               ((debug_data[3] << 8) | debug_data[2]),
                               ((debug_data[5] << 8) | debug_data[4]) );
#endif
                } else {
                        ps_cancl = mcu_data->ps_kvalue & 0xFF;
                        ps_canch = ((mcu_data->ps_kvalue) & 0xFF00) >> 8;
                        CWMCU_i2c_write_power(mcu_data,
                                       CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,
                                       &ps_cancl, 1);
                        CWMCU_i2c_write_power(mcu_data,
                                       CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,
                                       &ps_canch, 1);
                       	if((mcu_data->ps_kvalue & 0xFFFF0000) != 0X00000000) {
                       	        ps_thdl = (mcu_data->ps_kvalue >> 16) & 0xFF;
                       	        ps_thdh = (mcu_data->ps_kvalue >> 24) & 0xFF;;
                       	}
                       	else {
                       	        ps_thdl = mcu_data->ps_thd_fixed & 0xFF;
                       	        ps_thdh = (mcu_data->ps_thd_fixed >> 8) & 0xFF;
                       	}
                        CWMCU_i2c_write_power(mcu_data,
                                       	CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,
                                       	&ps_thdl, 1);
                        CWMCU_i2c_write_power(mcu_data,
                                       	CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,
                                       	&ps_thdh, 1);
			ps_thd_addl = mcu_data->ps_thd_add & 0xFF;
			ps_thd_addh = (mcu_data->ps_thd_add >> 8) & 0xFF;
                        CWMCU_i2c_write(mcu_data,
                                       	CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,
					&ps_thd_addl, 1, 1);
                        CWMCU_i2c_write(mcu_data,
                                       	CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,
					&ps_thd_addh, 1, 1);
                        I("set proximity-sensor kvalue is %x %x %x %x, "
                               "ps_thd_add is %x\n",
                               ps_cancl, ps_canch, ps_thdl, ps_thdh,
                               ((ps_thd_addh << 8) | ps_thd_addl) );
#if 0
                        I("set proximity-sensor kvalue is %x %x %x %x\n",
                               ps_cancl, ps_canch, ps_thdl, ps_thdh);

                       	CWMCU_i2c_read(mcu_data,
                                       CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PROXIMITY,
                                       debug_data, 6);
                        I("[AUSTIN DEBUG]  canc: 0x%X, thd: 0x%X, thd_add: 0x%X\n",
                               ((debug_data[1] << 8) | debug_data[0]),
                               ((debug_data[3] << 8) | debug_data[2]),
                               ((debug_data[5] << 8) | debug_data[4]) );
#endif
                }
                mcu_data->ps_calibrated = 1;
        }

	if (mcu_data->bs_kheader == 0x67) {
		__be32 be32_bs_data = cpu_to_be32(mcu_data->bs_kvalue);

		CWMCU_i2c_write(mcu_data,
			CW_I2C_REG_SENSORS_CALIBRATOR_SET_DATA_PRESSURE,
			&be32_bs_data, PRESSURE_CALIBRATOR_LEN, 1);
		mcu_data->bs_calibrated = 1;
		D(
		  "Set barometer kvalue (a, b, c, d) = "
		  "(0x%x, 0x%x, 0x%x, 0x%x)\n",
		  bs_data[3], bs_data[2], bs_data[1], bs_data[0]);
	}
	I("Sensor calibration matrix is (gs %u gy %u ls %u bs %u)\n",
		mcu_data->gs_calibrated, mcu_data->gy_calibrated,
		mcu_data->ls_calibrated, mcu_data->bs_calibrated);

	return 0;
}


static int cwmcu_set_touch_solution(struct cwmcu_data *mcu_data)
{
	if (g_touch_solution == TOUCH_SOLUTION_MAXIM1187X ||
	  g_touch_solution == TOUCH_SOLUTION_SYNAPTICS3351) {
		I("[TP] %s: solution = %d\n", __func__, g_touch_solution);
		CWMCU_i2c_write_power(s_mcu_data, TOUCH_SOLUTION_REGISTER, &g_touch_solution, 1);
	} else {
		W("[TP] %s: unexpected solution = %d\n", __func__, g_touch_solution);
	}
	if(g_touch_status != 0xff)
		CWMCU_i2c_write_power(s_mcu_data, TOUCH_STATUS_REGISTER, &g_touch_status, 1);
	I("[TP] %s: status = %d\n", __func__, g_touch_status);

	return 0;
}


static int cwmcu_sensor_placement(struct cwmcu_data *mcu_data)
{
	int i, rc;
	u8 cali_data_from_fw;

	I("Set Sensor Placement\n");

	return 0; // use firmware placement only

	for (i = 0; i < 3; i++) {
		CWMCU_i2c_write_power(mcu_data, GENSOR_POSITION,
				&mcu_data->acceleration_axes,
				1);
		rc = CWMCU_i2c_read_power(mcu_data, GENSOR_POSITION,
			       &cali_data_from_fw, 1);
		if (rc >= 0) {
			if (cali_data_from_fw == mcu_data->acceleration_axes)
				break;
			else {
				I("%s: cali_data_from_fw = 0x%x, "
				  "acceleration_axes = 0x%x\n",
				  __func__, cali_data_from_fw,
				  mcu_data->acceleration_axes);
			}
		} else {
			I("%s: GENSOR_POSITION i2c read fails, rc = %d\n",
			  __func__, rc);
		}
	}

	for (i = 0; i < 3; i++) {
		CWMCU_i2c_write_power(mcu_data, COMPASS_POSITION,
				&mcu_data->magnetic_axes, 1);
		rc = CWMCU_i2c_read_power(mcu_data, COMPASS_POSITION,
			       &cali_data_from_fw, 1);
		if (rc >= 0) {
			if (cali_data_from_fw == mcu_data->magnetic_axes)
				break;
			else {
				I("%s: cali_data_from_fw = 0x%x, "
				  "magnetic_axes = 0x%x\n",
				  __func__, cali_data_from_fw,
				  mcu_data->magnetic_axes);
			}
		} else {
			I("%s: COMPASS_POSITION i2c read fails, rc = %d\n",
			  __func__, rc);
		}
	}

	for (i = 0; i < 3; i++) {
		CWMCU_i2c_write_power(mcu_data, GYRO_POSITION,
				&mcu_data->gyro_axes, 1);
		rc = CWMCU_i2c_read_power(mcu_data, GYRO_POSITION,
			       &cali_data_from_fw, 1);
		if (rc >= 0) {
			if (cali_data_from_fw == mcu_data->gyro_axes)
				break;
			else {
				I("%s: cali_data_from_fw = 0x%x, "
				  "gyro_axes = 0x%x\n",
				  __func__, cali_data_from_fw,
				  mcu_data->gyro_axes);
			}
		} else {
			I("%s: GYRO_POSITION i2c read fails, rc = %d\n",
			  __func__, rc);
		}
	}
	return 0;
}

static void cwmcu_i2c_write_group(struct cwmcu_data *mcu_data, u8 write_addr,
				  u32 enable_list)
{
	int i;
	__le32 buf = cpu_to_le32(enable_list);
	u8 *data = (u8 *)&buf;

	for (i = 0; i < sizeof(buf); ++i) {
		D("%s: write_addr = 0x%x, write_val = 0x%x\n",
		  __func__, write_addr + i, data[i]);
		CWMCU_i2c_write(mcu_data, write_addr + i, data + i, 1, 1);
	}
}

static int cwmcu_restore_status(struct cwmcu_data *mcu_data)
{
	int i, rc;
	u8 data;
	u8 reg_value = 0;
	int delay_ms;
	u8 *data_p;

	D("Restore status\n");

	data_p = (u8 *)&mcu_data->gesture_motion_param;

	I("%s: setting gesture motion parameter = (0x%x, 0x%x, 0x%x, 0x%x)\n",
	  __func__, data_p[0], data_p[1], data_p[2], data_p[3]);

	for (i = 0; i < GESTURE_MOTION_UPDATE_ATTRIBUTE_LEN; i++) {
		CWMCU_i2c_write(mcu_data, GESTURE_MOTION_UPDATE_ATTRIBUTE,
				&data_p[i], 1, 1);
	}

	cwmcu_i2c_write_group(mcu_data, CWSTM32_ENABLE_REG,
			      mcu_data->enabled_list
			      | (mcu_data->enabled_list >> 32));

	D("%s: enable_list = 0x%llx\n", __func__, mcu_data->enabled_list);

	for (i = 0; i < CW_SENSORS_ID_TOTAL; i++) {
		delay_ms = mcu_data->report_period[i] / MS_TO_PERIOD;

		rc = firmware_odr(mcu_data, i, delay_ms);
		if (rc) {
			E("%s: firmware_odr fails, rc = %d, i = %d\n",
			  __func__, rc, i);
			return -EIO;
		}
	}

	cwmcu_i2c_write_group(mcu_data, CW_BATCH_ENABLE_REG,
			      mcu_data->batched_list);
	cwmcu_i2c_write_group(mcu_data, CW_WAKE_UP_BATCH_ENABLE_REG,
			      mcu_data->batched_list >> 32);
#ifdef MCU_WARN_MSGS
	reg_value = 1;
	rc = CWMCU_i2c_write_power(mcu_data, CW_I2C_REG_WARN_MSG_ENABLE,
			     &reg_value, 1);
	if (rc) {
		E("%s: CWMCU_i2c_write(WARN_MSG) fails, rc = %d, i = %d\n",
		  __func__, rc, i);
		return -EIO;
	}
	D("%s: WARN_MSGS enabled\n", __func__);
#endif

	reg_value = 1;
	rc = CWMCU_i2c_write_power(mcu_data, CW_I2C_REG_WATCH_DOG_ENABLE,
			     &reg_value, 1);
	if (rc) {
		E("%s: CWMCU_i2c_write(WATCH_DOG) fails, rc = %d\n",
		  __func__, rc);
		return -EIO;
	}
	D("%s: Watch dog enabled\n", __func__);

	/* Inform SensorHUB that CPU is going to resume */
	data = 1;
	CWMCU_i2c_write(mcu_data, CW_CPU_STATUS_REG, &data, 1, 1);
	D("%s: write_addr = 0x%x, write_data = 0x%x\n", __func__,
	  CW_CPU_STATUS_REG, data);

#ifdef CONFIG_AK8789_HALLSENSOR
	for (i = 0; i < HALL_RETRY_TIMES; i++) {
		rc = CWMCU_i2c_write_power(s_mcu_data,
				   CW_I2C_REG_DOTVIEW_STATUS,
				   &s_mcu_data->dotview_st, 1);
		if (rc) {
			E(
			  "%s: CWMCU_i2c_write HALL fails, rc = %d, "
			  "i = %d\n", __func__, rc, i);
			msleep(10);
		} else
			break;
	}
#endif

	return 0;
}

static void mcu_set_display_state(bool on_off)
{
    int ret;
    u8 display_state = (u8)on_off;
    ret = CWMCU_i2c_write_block_power(s_mcu_data, CW_I2C_REG_DISPLAY_STATE, (u8*)&display_state, sizeof(display_state));
    I("%s(%d): display_state:0x%x ret:%d\n", __func__, __LINE__, display_state, ret);
}

static int mcu_time_sync(void)
{
    MCU_TIME_SYNC_T mcu_time_sync ;
    struct timespec ts_rtc;
    struct rtc_time tm;
    struct timespec ts_boot;
    ktime_t ktime_boot;

    //boot time
    ktime_boot = ktime_get_boottime();
    ts_boot = ns_to_timespec(ktime_to_ns(ktime_boot));
    mcu_time_sync.boot_sec = ts_boot.tv_sec;
    mcu_time_sync.boot_nsec = ts_boot.tv_nsec;

    //rtc
    getnstimeofday(&ts_rtc);
    rtc_time_to_tm(ts_rtc.tv_sec, &tm);
    mcu_time_sync.year = tm.tm_year + 1900;
    mcu_time_sync.month = tm.tm_mon + 1;
    mcu_time_sync.day = tm.tm_mday;
    mcu_time_sync.hour = tm.tm_hour;
    mcu_time_sync.minute = tm.tm_min;
    mcu_time_sync.second= tm.tm_sec;
    mcu_time_sync.hour_format_24 = 1;

    D("%s[%d]: boot_time:%u.%u\n", __func__, __LINE__, mcu_time_sync.boot_sec, mcu_time_sync.boot_nsec);

    I("%s[%d]: %d-%02d-%02d %02d:%02d:%02d UTC\n", __func__, __LINE__,
        mcu_time_sync.year, mcu_time_sync.month, mcu_time_sync.day, mcu_time_sync.hour, mcu_time_sync.minute, mcu_time_sync.second);

    CWMCU_i2c_write_block_power(s_mcu_data, CW_I2C_REG_MCU_TIME, (u8*)&mcu_time_sync, sizeof(mcu_time_sync));

    return 0;
}

/* Returns the number of read bytes on success */
static int CWMCU_i2c_read(struct cwmcu_data *mcu_data,
			 u8 reg_addr, void *data, u8 len)
{
	s32 rc = 0;
	u8 *u8_data = data;

	if (MCU_IN_DLOAD() || MCU_IN_BOOTLOADER()) {
		I("%s[%d], s_mcu_state:%d, return %d\n", __func__, __LINE__, s_mcu_state, -ENOTCONN);
		if (!in_interrupt()) msleep(100);
		return -ENOTCONN;
	}

	if (DEBUG_DISABLE) {
		mcu_data->disable_access_count++;
		if ((mcu_data->disable_access_count % 100) == 0)
			I("%s: DEBUG_DISABLE = %d\n", __func__, DEBUG_DISABLE);
		return len;
	}

	if (mcu_data->is_block_i2c) {
		if (time_after(jiffies,
			       mcu_data->reset_jiffies + RESET_PERIOD)) {
			gpio_direction_input(mcu_data->gpio_reset);
			I("%s: gpio_reset = %d\n", __func__,
			  gpio_get_value_cansleep(mcu_data->gpio_reset));

			if (mcu_chip_mode_get(mcu_data) ==
			    MCU_CHIP_MODE_BOOTLOADER)
				msleep(100);
			else {
				/* HUB need at least 500ms to be ready */
				usleep_range(500000, 1000000);
			}

			mcu_data->is_block_i2c = false;
		}
		return len;
	}

	if (atomic_read(&mcu_data->suspended))
		return len;

	mutex_lock(&s_activated_i2c_lock);
	if (retry_exhausted(mcu_data)) {
		memset(u8_data, 0, len); /* Assign data to 0 when chip NACK */

		/* Try to recover HUB in low CPU utilization */
		D(
		  "%s: mcu_data->i2c_total_retry = %d, "
		  "mcu_data->i2c_latch_retry = %d\n", __func__,
		  mcu_data->i2c_total_retry,
		  mcu_data->i2c_latch_retry);
		mcu_data->w_activated_i2c = true;
		queue_work(mcu_data->mcu_wq, &mcu_data->one_shot_work);

		mutex_unlock(&s_activated_i2c_lock);
		return len;
	}

	while (!retry_exhausted(mcu_data)) {
		rc = CWMCU_do_read(mcu_data->mcu_client, reg_addr,
						   len, u8_data);
		if (rc == len) {
			retry_reset(mcu_data);
			break;
		} else {
			gpio_make_falling_edge(mcu_data->gpio_wake_mcu);
			mcu_data->i2c_total_retry++;
			if (rc == LATCH_ERROR_NO)
				mcu_data->i2c_latch_retry++;
			E("%s: i2c read error, rc = %d, total_retry = %d, latch_retry = %d, read_addr = 0x%x\n",
			  __func__,
			  rc, mcu_data->i2c_total_retry,
			  mcu_data->i2c_latch_retry,
			  reg_addr);
		}
	}

	if (retry_exhausted(mcu_data)) {
		E("%s: total_retry = %d, latch_retry = %d, return\n",
		  __func__, mcu_data->i2c_total_retry,
		  mcu_data->i2c_latch_retry);
	}

	mutex_unlock(&s_activated_i2c_lock);
	return rc;
}

static bool reset_hub(struct cwmcu_data *mcu_data, bool force_reset)
{
	if (force_reset || time_after(jiffies, mcu_data->reset_jiffies + RESET_PERIOD)) {
		gpio_direction_output(mcu_data->gpio_reset, 0);
		I("%s: gpio_reset = %d\n", __func__,
		  gpio_get_value_cansleep(mcu_data->gpio_reset));
		usleep_range(10000, 15000);
		gpio_direction_input(mcu_data->gpio_reset);
		I("%s: gpio_reset = %d\n", __func__,
		  gpio_get_value_cansleep(mcu_data->gpio_reset));

		retry_reset(mcu_data);
		mcu_boot_status_reset(mcu_data);
		mcu_data->i2c_jiffies = jiffies;

		if (mcu_chip_mode_get(mcu_data) == MCU_CHIP_MODE_BOOTLOADER) {
			msleep(100);
		}
		else {
			/* HUB need at least 500ms to be ready */
			usleep_range(500000, 1000000);
		}

		mcu_data->is_block_i2c = false;
	} else {
		gpio_direction_output(mcu_data->gpio_reset, 0);
		I("%s: else: gpio_reset = %d\n", __func__,
		  gpio_get_value_cansleep(mcu_data->gpio_reset));
		mcu_data->is_block_i2c = true;
	}

	mcu_data->reset_jiffies = jiffies;
	return !mcu_data->is_block_i2c;
}

/* This informs firmware for Output Data Rate of each sensor.
 * Need powermode held by caller */
static int firmware_odr(struct cwmcu_data *mcu_data, int sensors_id,
			int delay_ms)
{
	u8 reg_addr;
	u8 reg_value;
	int rc;

	switch (sensors_id) {
	case CW_ACCELERATION:
		reg_addr = ACCE_UPDATE_RATE;
		break;
	case CW_MAGNETIC:
		reg_addr = MAGN_UPDATE_RATE;
		break;
	case CW_GYRO:
		reg_addr = GYRO_UPDATE_RATE;
		break;
	case CW_ORIENTATION:
		reg_addr = ORIE_UPDATE_RATE;
		break;
	case CW_ROTATIONVECTOR:
		reg_addr = ROTA_UPDATE_RATE;
		break;
	case CW_LINEARACCELERATION:
		reg_addr = LINE_UPDATE_RATE;
		break;
	case CW_GRAVITY:
		reg_addr = GRAV_UPDATE_RATE;
		break;
	case CW_MAGNETIC_UNCALIBRATED:
		reg_addr = MAGN_UNCA_UPDATE_RATE;
		break;
	case CW_GYROSCOPE_UNCALIBRATED:
		reg_addr = GYRO_UNCA_UPDATE_RATE;
		break;
	case CW_GAME_ROTATION_VECTOR:
		reg_addr = GAME_ROTA_UPDATE_RATE;
		break;
	case CW_GEOMAGNETIC_ROTATION_VECTOR:
		reg_addr = GEOM_ROTA_UPDATE_RATE;
		break;
	case CW_SIGNIFICANT_MOTION:
		reg_addr = SIGN_UPDATE_RATE;
		break;
	case CW_PRESSURE:
		reg_addr = PRESSURE_UPDATE_RATE;
		break;
	case CW_STEP_COUNTER:
		reg_addr = STEP_COUNTER_UPDATE_PERIOD;
		break;
	case CW_ACCELERATION_W:
		reg_addr = ACCE_WAKE_UPDATE_RATE;
		break;
	case CW_MAGNETIC_W:
		reg_addr = MAGN_WAKE_UPDATE_RATE;
		break;
	case CW_GYRO_W:
		reg_addr = GYRO_WAKE_UPDATE_RATE;
		break;
	case CW_PRESSURE_W:
		reg_addr = PRESSURE_WAKE_UPDATE_RATE;
		break;
	case CW_ORIENTATION_W:
		reg_addr = ORIE_WAKE_UPDATE_RATE;
		break;
	case CW_ROTATIONVECTOR_W:
		reg_addr = ROTA_WAKE_UPDATE_RATE;
		break;
	case CW_LINEARACCELERATION_W:
		reg_addr = LINE_WAKE_UPDATE_RATE;
		break;
	case CW_GRAVITY_W:
		reg_addr = GRAV_WAKE_UPDATE_RATE;
		break;
	case CW_MAGNETIC_UNCALIBRATED_W:
		reg_addr = MAGN_UNCA_WAKE_UPDATE_RATE;
		break;
	case CW_GYROSCOPE_UNCALIBRATED_W:
		reg_addr = GYRO_UNCA_WAKE_UPDATE_RATE;
		break;
	case CW_GAME_ROTATION_VECTOR_W:
		reg_addr = GAME_ROTA_WAKE_UPDATE_RATE;
		break;
	case CW_GEOMAGNETIC_ROTATION_VECTOR_W:
		reg_addr = GEOM_ROTA_WAKE_UPDATE_RATE;
		break;
	case CW_STEP_COUNTER_W:
		reg_addr = STEP_COUNTER_UPDATE_PERIOD;
		break;
	default:
		reg_addr = 0;
		if ((CW_ACCELERATION <= sensors_id) &&
		    (sensors_id < num_sensors)) {
			D("%s: Only report_period changed, sensors_id = %d,"
			  " delay_us = %6d\n",
			  __func__, sensors_id,
			  mcu_data->report_period[sensors_id]);
		} else {
			I("%s: Invalid sensors_id = %d\n", __func__,
			  sensors_id);
		}
		return 0;
	}

	if (!(mcu_data->enabled_list & (1LL << sensors_id)) &&
	    (delay_ms >= 300))
		reg_value = UPDATE_RATE_NONE;
	else if (delay_ms >= 200)
		reg_value = UPDATE_RATE_NORMAL;
	else if (delay_ms >= 100)
		reg_value = UPDATE_RATE_RATE_10Hz;
	else if (delay_ms >= 60)
		reg_value = UPDATE_RATE_UI;
	else if (delay_ms >= 40)
		reg_value = UPDATE_RATE_RATE_25Hz;
	else if (delay_ms >= 20)
		reg_value = UPDATE_RATE_GAME;
	else
		reg_value = UPDATE_RATE_FASTEST;

	if ((sensors_id != CW_STEP_COUNTER) && (sensors_id != CW_LIGHT) &&
	    (sensors_id != CW_STEP_COUNTER_W) && (sensors_id != CW_PROXIMITY)) {
		D("%s: reg_addr = 0x%x, reg_value = 0x%x\n",
		  __func__, reg_addr, reg_value);

		rc = CWMCU_i2c_write(mcu_data, reg_addr, &reg_value, 1, 1);
		if (rc) {
			E("%s: CWMCU_i2c_write fails, rc = %d\n", __func__, rc);
			return -EIO;
		}
	} else {
		__le32 period_data;

		period_data = cpu_to_le32(delay_ms);

		D("%s: reg_addr = 0x%x, period_data = 0x%x\n",
		  __func__, reg_addr, period_data);

		rc = CWMCU_i2c_multi_write(mcu_data, reg_addr,
					   &period_data,
					   sizeof(period_data));
		if (rc) {
			E("%s: CWMCU_i2c_multi_write fails, rc = %d\n",
			  __func__, rc);
			return -EIO;
		}
	}

	return 0;
}

int is_continuous_sensor(int sensors_id)
{
	switch (sensors_id) {
	case CW_ACCELERATION:
	case CW_MAGNETIC:
	case CW_GYRO:
	case CW_PRESSURE:
	case CW_ORIENTATION:
	case CW_ROTATIONVECTOR:
	case CW_LINEARACCELERATION:
	case CW_GRAVITY:
	case CW_MAGNETIC_UNCALIBRATED:
	case CW_GYROSCOPE_UNCALIBRATED:
	case CW_GAME_ROTATION_VECTOR:
	case CW_GEOMAGNETIC_ROTATION_VECTOR:
	case CW_ACCELERATION_W:
	case CW_MAGNETIC_W:
	case CW_GYRO_W:
	case CW_PRESSURE_W:
	case CW_ORIENTATION_W:
	case CW_ROTATIONVECTOR_W:
	case CW_LINEARACCELERATION_W:
	case CW_GRAVITY_W:
	case CW_MAGNETIC_UNCALIBRATED_W:
	case CW_GYROSCOPE_UNCALIBRATED_W:
	case CW_GAME_ROTATION_VECTOR_W:
	case CW_GEOMAGNETIC_ROTATION_VECTOR_W:
		return 1;
		break;
	default:
		return 0;
		break;
	}
}

static void setup_delay(struct cwmcu_data *mcu_data)
{
	u8 i;
	int delay_ms;
	int delay_candidate_ms;

	delay_candidate_ms = CWMCU_NO_POLLING_DELAY;
	for (i = 0; i < CW_SENSORS_ID_TOTAL; i++) {
		D("%s: batch_timeout[%d] = %lld\n", __func__, i,
		  mcu_data->batch_timeout[i]);
		if ((mcu_data->enabled_list & (1LL << i)) &&
		    (is_continuous_sensor(i) || i == CW_LIGHT) &&
		    (mcu_data->batch_timeout[i] == 0)) {
			D("%s: report_period[%d] = %d\n", __func__, i,
			  mcu_data->report_period[i]);

			/* report_period is actual delay(us) * 0.99), convert to
			 * microseconds */
			delay_ms = mcu_data->report_period[i] /
					MS_TO_PERIOD;
			if (delay_ms > CWMCU_MAX_DELAY)
				delay_ms = CWMCU_MAX_DELAY;

			if (delay_candidate_ms > delay_ms)
				delay_candidate_ms = delay_ms;
		}
	}

	if (delay_candidate_ms != atomic_read(&mcu_data->delay)) {
		cancel_delayed_work_sync(&mcu_data->work);
		if (mcu_data->enabled_list & IIO_SENSORS_MASK) {
			atomic_set(&mcu_data->delay, delay_candidate_ms);
			queue_delayed_work(mcu_data->mcu_wq, &mcu_data->work,
					   0);
		} else
			atomic_set(&mcu_data->delay, CWMCU_MAX_DELAY + 1);
	}

	I("%s: Minimum delay = %dms\n", __func__,
	  atomic_read(&mcu_data->delay));

}

static int handle_batch_list(struct cwmcu_data *mcu_data, int sensors_id,
			     bool is_wake)
{
	int rc;
	u8 i;
	u8 data;
	u64 sensors_bit;
	u8 write_addr;

	if ((sensors_id == CW_LIGHT) || (sensors_id == CW_PROXIMITY) || (sensors_id == CW_SIGNIFICANT_MOTION))
		return 0;

	sensors_bit = (1LL << sensors_id);
	mcu_data->batched_list &= ~sensors_bit;
	mcu_data->batched_list |= (mcu_data->enabled_list & sensors_bit)
					? sensors_bit : 0;

	D("%s: sensors_bit = 0x%llx, batched_list = 0x%llx\n", __func__,
	  sensors_bit, mcu_data->batched_list);

	i = (sensors_id / 8);
	data = (u8)(mcu_data->batched_list >> (i*8));

	write_addr = (is_wake) ? CW_WAKE_UP_BATCH_ENABLE_REG :
				 CW_BATCH_ENABLE_REG;

	if (i > 3)
		i = (i - 4);

	D("%s: Writing, addr = 0x%x, data = 0x%x\n", __func__,
	  (write_addr+i), data);

	rc = CWMCU_i2c_write_power(mcu_data, write_addr+i, &data, 1);
	if (rc)
		E("%s: CWMCU_i2c_write fails, rc = %d\n",
		  __func__, rc);

	return rc;
}

static int setup_batch_timeout(struct cwmcu_data *mcu_data, bool is_wake)
{
	__le32 timeout_data;
	s64 current_timeout;
	u32 continuous_sensor_count;
	u8 i;
	u8 write_addr;
	int rc;
	int scan_limit;

	current_timeout = 0;
	if (is_wake) {
		i = CW_ACCELERATION_W;
		scan_limit = CW_SENSORS_ID_TOTAL;
	} else {
		i = CW_ACCELERATION;
		scan_limit = CW_SENSORS_ID_FW;
	}
	for (continuous_sensor_count = 0; i < scan_limit; i++) {
		if (mcu_data->batch_timeout[i] != 0) {
			if ((current_timeout >
			     mcu_data->batch_timeout[i]) ||
			    (current_timeout == 0)) {
				current_timeout =
					mcu_data->batch_timeout[i];
			}
			D("sensorid = %d, current_timeout = %lld\n",
			  i, current_timeout);
		} else
			continuous_sensor_count++;
	}

	if (continuous_sensor_count == scan_limit)
		current_timeout = 0;

	timeout_data = cpu_to_le32(current_timeout);

	write_addr = (is_wake) ? CWSTM32_WAKE_UP_BATCH_MODE_TIMEOUT :
				 CWSTM32_BATCH_MODE_TIMEOUT;

	D(
	  "%s: Writing, write_addr = 0x%x, current_timeout = %lld,"
	  " timeout_data = 0x%x\n",
	  __func__, write_addr, current_timeout, timeout_data);

	cwmcu_powermode_switch(mcu_data, 1);
	rc = CWMCU_i2c_multi_write(mcu_data, write_addr,
				   &timeout_data,
				   sizeof(timeout_data));
	cwmcu_powermode_switch(mcu_data, 0);
	if (rc)
		E("%s: CWMCU_i2c_write fails, rc = %d\n", __func__, rc);

	return rc;
}
#if defined(CONFIG_SYNC_TOUCH_STATUS)
int touch_status(u8 status)
{
	int ret = -1;

	g_touch_status = status;

	if (!s_mcu_data) {
		W("[TP] %s: probe not completed\n", __func__);
		return ret;
	}

	I("[TP] %s: status = %d\n", __func__, status);
	if(g_touch_status != 0xff)
		ret = CWMCU_i2c_write_power(s_mcu_data, TOUCH_STATUS_REGISTER, &status, 1);
	return 0;
}
#endif

int touch_solution(u8 solution)
{
	int ret = -1;

	g_touch_solution = solution;

	if (!s_mcu_data) {
		W("[TP] %s: probe not completed\n", __func__);
		return ret;
	}

	if (MCU_IN_SHUB()) {
		cwmcu_set_touch_solution(s_mcu_data);
	} else {
		W("[TP] %s: hub not ready\n", __func__);
	}
	return 0;
}

static u64 report_step_counter(struct cwmcu_data *mcu_data, u32 fw_step,
			       u64 timestamp, bool is_wake)
{
	u16 u16_data_buff[REPORT_EVENT_COMMON_LEN * 2];
	u64 step_counter_buff;

	mcu_data->sensors_time[CW_STEP_COUNTER] = 0;

	step_counter_buff = mcu_data->step_counter_base + fw_step;

	u16_data_buff[0] = step_counter_buff & 0xFFFF;
	u16_data_buff[1] = (step_counter_buff >> 16) & 0xFFFF;
	u16_data_buff[2] = 0;
	u16_data_buff[3] = (step_counter_buff >> 32) & 0xFFFF;
	u16_data_buff[4] = (step_counter_buff >> 48) & 0xFFFF;
	u16_data_buff[5] = 0;

	cw_send_event_special(mcu_data, (is_wake) ? CW_STEP_COUNTER_W
						  : CW_STEP_COUNTER,
			      u16_data_buff,
			      u16_data_buff + REPORT_EVENT_COMMON_LEN,
			      timestamp);

	return step_counter_buff;
}

static ssize_t active_set(struct device *dev, struct device_attribute *attr,
			  const char *buf, size_t count)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	long enabled = 0;
	long sensors_id = 0;
	u8 data;
	u8 i;
	char *str_buf;
	char *running;
	u64 sensors_bit;
	int rc;
	bool is_wake;
	bool non_wake_bit;
	bool wake_bit;
	u32 write_list;

	atomic_set(&mcu_data->critical_sect, 1);

	str_buf = kstrndup(buf, count, GFP_KERNEL);
	if (str_buf == NULL) {
		E("%s: cannot allocate buffer\n", __func__);
		atomic_set(&mcu_data->critical_sect, 0);
		return -ENOMEM;
	}
	running = str_buf;

	for (i = 0; i < 2; i++) {
		int error;
		char *token;

		token = strsep(&running, " ");

		if (token && (i == 0))
			error = kstrtol(token, 10, &sensors_id);
		else {
			if (token == NULL) {
				enabled = sensors_id;
				sensors_id = 0;
			} else
				error = kstrtol(token, 10, &enabled);
		}
		if (error) {
			E("%s: kstrtol fails, error = %d, i = %d\n",
				__func__, error, i);
			kfree(str_buf);
			atomic_set(&mcu_data->critical_sect, 0);
			return error;
		}
	}
	kfree(str_buf);

	if (!s_mcu_data) {
		W("%s: probe not completed\n", __func__);
		atomic_set(&mcu_data->critical_sect, 0);
		return -EBUSY;
	}

	if ((!mcu_data->touch_enable) && sensors_id == HTC_GESTURE_MOTION){
		atomic_set(&mcu_data->critical_sect, 0);
		return 0;
	}

	if ((sensors_id >= CW_SENSORS_ID_TOTAL) ||
	    (sensors_id < 0)
	   ) {
		atomic_set(&mcu_data->critical_sect, 0);
		E("%s: Invalid sensors_id = %ld\n", __func__, sensors_id);
		return -EINVAL;
	}

	if ((sensors_id == HTC_ANY_MOTION) && mcu_data->power_key_pressed) {
		atomic_set(&mcu_data->critical_sect, 0);
		I("%s: Any_Motion && power_key_pressed\n", __func__);
		return count;
	}

	sensors_bit = 1LL << sensors_id;

	is_wake = (sensors_id >= CW_ACCELERATION_W) &&
		  (sensors_id <= CW_STEP_COUNTER_W);
	if (is_wake) {
		wake_bit = (mcu_data->enabled_list & sensors_bit);
		non_wake_bit = (mcu_data->enabled_list & (sensors_bit >> 32));
	} else {
		wake_bit = (mcu_data->enabled_list & (sensors_bit << 32));
		non_wake_bit = (mcu_data->enabled_list & sensors_bit);
	}

	mcu_data->enabled_list &= ~sensors_bit;
	mcu_data->enabled_list |= enabled ? sensors_bit : 0;

	/* clean batch parameters if sensor turn off */
	if (!enabled) {
		mcu_data->batch_timeout[sensors_id] = 0;
		mcu_data->batched_list &= ~sensors_bit;
		mcu_data->sensors_time[sensors_id] = 0;
		setup_batch_timeout(mcu_data, is_wake);
		mcu_data->report_period[sensors_id] =
				DEFAULT_DELAY_US * MS_TO_PERIOD;
		mcu_data->pending_flush[sensors_id] = 0;
	} else {
		do_gettimeofday(&mcu_data->now);
		mcu_data->sensors_time[sensors_id] =
			(mcu_data->now.tv_sec * NS_PER_US) +
			mcu_data->now.tv_usec;
	}

	write_list = mcu_data->enabled_list | (mcu_data->enabled_list >> 32);

	i = ((is_wake) ? (sensors_id - 32) : sensors_id) / 8;
	data = (u8)(write_list >> (i*8));

	if (enabled
	    ? !(wake_bit | non_wake_bit)
	    : (wake_bit ^ non_wake_bit)) {
		D("%s: Writing: CWSTM32_ENABLE_REG+i = 0x%x, data = 0x%x\n",
		  __func__, CWSTM32_ENABLE_REG+i, data);
		rc = CWMCU_i2c_write_power(mcu_data, CWSTM32_ENABLE_REG+i,
					   &data, 1);
		if (rc) {
			atomic_set(&mcu_data->critical_sect, 0);
			E("%s: CWMCU_i2c_write fails, rc = %d\n",
			  __func__, rc);
			return -EIO;
		}

		/* Disabling Step counter and no other step counter enabled */
		if (((sensors_id == CW_STEP_COUNTER) ||
		     (sensors_id == CW_STEP_COUNTER_W))
		    && !enabled
		    && !(mcu_data->enabled_list & STEP_COUNTER_MASK)) {
			__le32 data[3];

			rc = CWMCU_i2c_read_power(mcu_data,
					    CWSTM32_READ_STEP_COUNTER,
					    data, sizeof(data));
			if (rc >= 0) {
				mcu_data->step_counter_base +=
							le32_to_cpu(data[2]);
				D("%s: Record step = %llu\n",
				  __func__, mcu_data->step_counter_base);
			} else {
				D("%s: Step Counter i2c read fails, rc = %d\n",
				  __func__, rc);
			}
		}

		if (!enabled
		    && (!(mcu_data->enabled_list & IIO_CONTINUOUS_MASK))) {
			mutex_lock(&mcu_data->mutex_lock);
			mcu_data->w_clear_fifo_running = true;
			mcu_data->w_clear_fifo = true;
			mutex_unlock(&mcu_data->mutex_lock);
			queue_work(mcu_data->mcu_wq, &mcu_data->one_shot_work);
		}

	}

	if (mcu_data->als_polling && sensors_id == CW_LIGHT) {
		mcu_data->report_period[light] =
			 CWMCU_LIGHT_POLLING_DELAY * MS_TO_PERIOD;
		D("%s: Initial lightsensor report_period = %d\n", __func__,
					 mcu_data->report_period[light]);
	}

	cwmcu_powermode_switch(mcu_data, 1);
	rc = firmware_odr(mcu_data, sensors_id,
			  mcu_data->report_period[sensors_id] / MS_TO_PERIOD);
	cwmcu_powermode_switch(mcu_data, 0);
	if (rc) {
		E("%s: firmware_odr fails, rc = %d\n", __func__, rc);
	}

        if (sensors_id == CW_PROXIMITY) {
                I("%s: Initial proximitysensor\n", __func__);
                if (!enabled)
                        p_status = 1;
                else
                        p_status = 9;
        }

	if ((sensors_id == CW_LIGHT) && (!!enabled)) {
		/*
		   This is kept from STM-383 driver for debugging purpose.
		   Basically, the MCU has covered the mechanism to initilize a
		   light sensor level by triggering an interrupt to driver.
		*/
		if (mcu_data->als_polling) {
			/*
			   Flag up for light sensor first polling event to make
			   sure the first reporting after enabled is 100ms after.
			*/
			atomic_set(&mcu_data->als_first_polling_event, 1);
			I("%s: Initial lightsensor = 0x%04X%04X (not report)\n",
			  __func__, mcu_data->light_last_data[1], mcu_data->light_last_data[0]);
		} else {
			I("%s: Initial lightsensor = %d\n",
			  __func__, mcu_data->light_last_data[0]);
			cw_send_event(mcu_data, CW_LIGHT, &mcu_data->light_last_data[0], 0);
		}
	}

	setup_delay(mcu_data);

	rc = handle_batch_list(mcu_data, sensors_id, is_wake);
	if (rc) {
		atomic_set(&mcu_data->critical_sect, 0);
		E("%s: handle_batch_list fails, rc = %d\n", __func__,
		  rc);
		return rc;
	}

	atomic_set(&mcu_data->critical_sect, 0);

	I("%s: sensors_id = %ld, enable = %ld, enable_list = 0x%llx\n",
		__func__, sensors_id, enabled, mcu_data->enabled_list);

	return count;
}

static ssize_t active_show(struct device *dev, struct device_attribute *attr,
			   char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	int i;
	u8 data[4];

	for (i=0; i<4; i++) {
		CWMCU_i2c_read_power(mcu_data, CWSTM32_ENABLE_REG+i, &data[i], sizeof(data[i]));
	}

	return scnprintf(buf, PAGE_SIZE, "0x%llX, mcu=0x%08X\n",
			 mcu_data->enabled_list, cpu_to_le32p((__u32 *)&data));
}

static ssize_t interval_show(struct device *dev, struct device_attribute *attr,
			     char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d\n", atomic_read(&mcu_data->delay));
}

static ssize_t interval_set(struct device *dev, struct device_attribute *attr,
			    const char *buf, size_t count)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	long val = 0;
	long sensors_id = 0;
	int i, rc;
	char *str_buf;
	char *running;

	str_buf = kstrndup(buf, count, GFP_KERNEL);
	if (str_buf == NULL) {
		E("%s: cannot allocate buffer\n", __func__);
		return -ENOMEM;
	}
	running = str_buf;

	for (i = 0; i < 2; i++) {
		int error;
		char *token;

		token = strsep(&running, " ");

		if (token && (i == 0))
			error = kstrtol(token, 10, &sensors_id);
		else {
			if (token == NULL) {
				val = 66;
				D("%s: delay set to 66\n", __func__);
			} else
				error = kstrtol(token, 10, &val);
		}
		if (error) {
			E("%s: kstrtol fails, error = %d, i = %d\n",
				__func__, error, i);
			kfree(str_buf);
			return error;
		}
	}
	kfree(str_buf);

	if ((sensors_id < 0) || (sensors_id >= num_sensors)) {
		D("%s: Invalid sensors_id = %ld\n", __func__, sensors_id);
		return -EINVAL;
	}

	if (mcu_data->report_period[sensors_id] != val * MS_TO_PERIOD) {
		/* period is actual delay(us) * 0.99 */

		mcu_data->report_period[sensors_id] = val * MS_TO_PERIOD;

		setup_delay(mcu_data);

		cwmcu_powermode_switch(mcu_data, 1);

		rc = firmware_odr(mcu_data, sensors_id, val);
		cwmcu_powermode_switch(mcu_data, 0);
		if (rc) {
			E("%s: firmware_odr fails, rc = %d\n", __func__, rc);
			return rc;
		}
	}

	return count;
}


static ssize_t batch_set(struct device *dev,
		     struct device_attribute *attr,
		     const char *buf, size_t count)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	s64 timeout = 0;
	int sensors_id = 0, flag = 0, delay_ms = 0;
	u8 i;
	int retry;
	int rc;
	char *token;
	char *str_buf;
	char *running;
	long input_val;
	unsigned long long input_val_l;
	bool need_update_fw_odr;
	s32 period;
	bool is_wake;

	atomic_set(&mcu_data->critical_sect, 1);

	if (!s_mcu_data) {
		atomic_set(&mcu_data->critical_sect, 0);
		W("%s: probe not completed\n", __func__);
		return -1;
	}

	for (retry = 0; retry < ACTIVE_RETRY_TIMES; retry++) {
		if (atomic_read(&mcu_data->suspended)) {
			D("%s: suspended, retry = %d\n",
				__func__, retry);
			usleep_range(5000, 10000);
		} else
			break;
	}
	if (retry >= ACTIVE_RETRY_TIMES) {
		atomic_set(&mcu_data->critical_sect, 0);
		D("%s: resume not completed, retry = %d, retry fails!\n",
			__func__, retry);
		return -ETIMEDOUT;
	}

	str_buf = kstrndup(buf, count, GFP_KERNEL);
	if (str_buf == NULL) {
		atomic_set(&mcu_data->critical_sect, 0);
		E("%s: cannot allocate buffer\n", __func__);
		return -1;
	}
	running = str_buf;

	for (i = 0; i < 4; i++) {
		token = strsep(&running, " ");
		if (token == NULL) {
			E("%s: token = NULL, i = %d\n", __func__, i);
			break;
		}

		switch (i) {
		case 0:
			rc = kstrtol(token, 10, &input_val);
			sensors_id = (int)input_val;
			break;
		case 1:
			rc = kstrtol(token, 10, &input_val);
			flag = (int)input_val;
			break;
		case 2:
			rc = kstrtol(token, 10, &input_val);
			delay_ms = (int)input_val;
			break;
		case 3:
			rc = kstrtoull(token, 10, &input_val_l);
			timeout = (s64)input_val_l;
			break;
		default:
			E("%s: Unknown i = %d\n", __func__, i);
			break;
		}

		if (rc) {
			atomic_set(&mcu_data->critical_sect, 0);
			E("%s: kstrtol fails, rc = %d, i = %d\n",
			  __func__, rc, i);
			kfree(str_buf);
			return rc;
		}
	}
	kfree(str_buf);

	D("%s: sensors_id = 0x%x, flag = %d, delay_ms = %d, timeout = %lld\n",
	  __func__, sensors_id, flag, delay_ms, timeout);

	is_wake = (CW_ACCELERATION_W <= sensors_id) &&
				       (sensors_id <= CW_STEP_COUNTER_W);

	/* period is actual delay(us) * 0.99 */
	period = delay_ms * MS_TO_PERIOD;
	need_update_fw_odr = mcu_data->report_period[sensors_id] != period;

	D("%s: period = %d, report_period[%d] = %d\n",
	  __func__, period, sensors_id, mcu_data->report_period[sensors_id]);
	mcu_data->report_period[sensors_id] = period;

	mcu_data->report_period[sensors_id] = ((mcu_data->als_polling) && (sensors_id == CW_LIGHT))
				? (CWMCU_LIGHT_POLLING_DELAY * MS_TO_PERIOD)
				: period;

	switch (sensors_id) {
	case CW_ACCELERATION:
	case CW_MAGNETIC:
	case CW_GYRO:
	case CW_PRESSURE:
	case CW_ORIENTATION:
	case CW_ROTATIONVECTOR:
	case CW_LINEARACCELERATION:
	case CW_GRAVITY:
	case CW_MAGNETIC_UNCALIBRATED:
	case CW_GYROSCOPE_UNCALIBRATED:
	case CW_GAME_ROTATION_VECTOR:
	case CW_GEOMAGNETIC_ROTATION_VECTOR:
	case CW_STEP_DETECTOR:
	case CW_STEP_COUNTER:
	case CW_ACCELERATION_W:
	case CW_MAGNETIC_W:
	case CW_GYRO_W:
	case CW_PRESSURE_W:
	case CW_ORIENTATION_W:
	case CW_ROTATIONVECTOR_W:
	case CW_LINEARACCELERATION_W:
	case CW_GRAVITY_W:
	case CW_MAGNETIC_UNCALIBRATED_W:
	case CW_GYROSCOPE_UNCALIBRATED_W:
	case CW_GAME_ROTATION_VECTOR_W:
	case CW_GEOMAGNETIC_ROTATION_VECTOR_W:
	case CW_STEP_DETECTOR_W:
	case CW_STEP_COUNTER_W:
		break;
	case CW_LIGHT:
	case CW_SIGNIFICANT_MOTION:
	default:
		atomic_set(&mcu_data->critical_sect, 0);
		D("%s: Batch not supported for this sensor_id = 0x%x\n",
		  __func__, sensors_id);
		return count;
	}

	mcu_data->batch_timeout[sensors_id] = timeout;

	setup_delay(mcu_data);

	rc = setup_batch_timeout(mcu_data, is_wake);
	if (rc) {
		atomic_set(&mcu_data->critical_sect, 0);
		E("%s: setup_batch_timeout fails, rc = %d\n", __func__, rc);
		return rc;
	}

	if ((need_update_fw_odr == true) &&
	    (mcu_data->enabled_list & (1LL << sensors_id))) {
		int odr_sensors_id;

		odr_sensors_id = (is_wake) ? (sensors_id + 32) : sensors_id;

		cwmcu_powermode_switch(mcu_data, 1);
		rc = firmware_odr(mcu_data, odr_sensors_id, delay_ms);
		cwmcu_powermode_switch(mcu_data, 0);
		if (rc) {
			E("%s: firmware_odr fails, rc = %d\n", __func__, rc);
		}
	}

	atomic_set(&mcu_data->critical_sect, 0);

	I(
	  "%s: sensors_id = %d, timeout = %lld, batched_list = 0x%llx,"
	  " delay_ms = %d\n",
	  __func__, sensors_id, timeout, mcu_data->batched_list,
	  delay_ms);

	return (rc) ? rc : count;
}

static ssize_t batch_show(struct device *dev, struct device_attribute *attr,
		      char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u64 timestamp = 0;
	struct timespec kt;
	u64 k_timestamp;

	kt = current_kernel_time();

	CWMCU_i2c_read_power(mcu_data, CW_I2C_REG_MCU_TIME, &timestamp,
			     sizeof(timestamp));

	le64_to_cpus(&timestamp);

	k_timestamp = (u64)(kt.tv_sec*NSEC_PER_SEC) + (u64)kt.tv_nsec;

	return scnprintf(buf, PAGE_SIZE, "%llu", timestamp);
}


static ssize_t flush_show(struct device *dev, struct device_attribute *attr,
		      char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	int ret;
	u8 data[4] = {0};

	ret = CWMCU_i2c_read_power(mcu_data, CWSTM32_BATCH_MODE_DATA_COUNTER,
			     data, sizeof(data));
	if (ret < 0)
		D("%s: Read Counter fail, ret = %d\n", __func__, ret);

	D("%s: DEBUG: Queue counter = %d\n", __func__,
	  *(u32 *)&data[0]);

	return scnprintf(buf, PAGE_SIZE, "Queue counter = %d\n",
			 *(u32 *)&data[0]);
}

static void cwmcu_send_flush(struct cwmcu_data *mcu_data, int id)
{
	u8 type = CW_META_DATA;
	u16 data[REPORT_EVENT_COMMON_LEN];
	s64 timestamp = 0;
	int rc;

	data[0] = (u16)id;
	data[1] = data[2] = 0;

	I("%s: flush sensor: %d!!\n", __func__, id);

	rc = cw_send_event(mcu_data, type, data, timestamp);
	if (rc < 0)
		E("%s: send_event fails, rc = %d\n", __func__, rc);
}

static ssize_t flush_set(struct device *dev, struct device_attribute *attr,
		     const char *buf, size_t count)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 data;
	unsigned long handle;
	int rc;

	atomic_set(&mcu_data->critical_sect, 1);

	rc = kstrtoul(buf, 10, &handle);
	if (rc) {
		atomic_set(&mcu_data->critical_sect, 0);
		E("%s: kstrtoul fails, rc = %d\n", __func__, rc);
		return rc;
	}

	D("%s: handle = %lu\n", __func__, handle);

	data = handle;

	if ((handle == CW_SIGNIFICANT_MOTION) ||(handle == CW_PROXIMITY)) {
		mutex_lock(&mcu_data->lock);
		cwmcu_send_flush(mcu_data, handle);
		mutex_unlock(&mcu_data->lock);
	} else {
		mcu_data->pending_flush[handle]++;
	}

	mcu_data->w_flush_fifo = true;
	queue_work(mcu_data->mcu_wq, &mcu_data->one_shot_work);

	atomic_set(&mcu_data->critical_sect, 0);

	return count;
}

/* Return if META is read out */
static bool report_iio(struct cwmcu_data *mcu_data, int *i, u8 *data,
		       __le64 *data64, u32 *event_count, bool is_wake)
{
	s32 ret;
	u8 data_buff;
	u16 data_event[REPORT_EVENT_COMMON_LEN];
	u16 bias_event[REPORT_EVENT_COMMON_LEN];
	u16 timestamp_event;
	u64 *handle_time_base;
	bool is_meta_read = false;

	if (is_wake) {
		wake_lock_timeout(&mcu_data->report_wake_lock,
				  msecs_to_jiffies(200));
	}

	if (data[0] == CW_META_DATA) {
		__le16 *data16 = (__le16 *)(data + 1);

		data_event[0] = le16_to_cpup(data16 + 1);
		cw_send_event(mcu_data, data[0], data_event, 0);
		mcu_data->pending_flush[data_event[0]]--;
		I(
		  "total count = %u, current_count = %d, META from firmware,"
		  " event_id = %d\n", *event_count, *i, data_event[0]);
		is_meta_read = true;
	} else if (data[0] == CW_TIME_BASE) {
		u64 timestamp;

		timestamp = le64_to_cpup(data64 + 1);

		handle_time_base = (is_wake) ? &mcu_data->wake_fifo_time_base :
					       &mcu_data->time_base;

		D(
		  "total count = %u, current_count = %d, CW_TIME_BASE = %llu,"
		  " is_wake = %d\n", *event_count, *i, timestamp, is_wake);
		*handle_time_base = timestamp;

	} else if (data[0] == CW_STEP_DETECTOR) {
		__le16 *data16 = (__le16 *)(data + 1);

		timestamp_event = le16_to_cpup(data16);

		data_event[0] = 1;
		handle_time_base = (is_wake) ?
					&mcu_data->wake_fifo_time_base :
					&mcu_data->time_base;
		cw_send_event(mcu_data,
			      (is_wake)
				? CW_STEP_DETECTOR_W
				: CW_STEP_DETECTOR
			      , data_event
			      , timestamp_event + *handle_time_base);
		if (DEBUG_FLAG_GSENSOR == 1) {
			I(
			  "Batch data: total count = %u, current count = %d, "
			  "STEP_DETECTOR%s, timediff = %d, time_base = %llu,"
			  " r_time = %llu\n"
			  , *event_count, *i, (is_wake) ? "_W" : ""
			  , timestamp_event
			  , *handle_time_base
			  , *handle_time_base + timestamp_event
			  );
		}

	} else if (data[0] == CW_STEP_COUNTER) {
		__le16 *data16 = (__le16 *)(data + 1);
		__le32 *data32 = (__le32 *)(data + 3);

		timestamp_event = le16_to_cpup(data16);

		handle_time_base = (is_wake) ?
					&mcu_data->wake_fifo_time_base :
					&mcu_data->time_base;
		report_step_counter(mcu_data,
				    le32_to_cpu(*data32),
				    timestamp_event + *handle_time_base,
				    is_wake);
		if (DEBUG_FLAG_GSENSOR == 1) {
			I(
			  "Batch data: total count = %u, current count = %d, "
			  "STEP_COUNTER%s, step = %d, "
			  "timediff = %d, time_base = %llu, r_time = %llu\n"
			  , *event_count, *i, (is_wake) ? "_W" : ""
			  , le32_to_cpu(*data32)
			  , timestamp_event
			  , *handle_time_base
			  , *handle_time_base + timestamp_event
			  );
		}

	} else if ((data[0] == CW_MAGNETIC_UNCALIBRATED_BIAS) ||
		   (data[0] == CW_GYROSCOPE_UNCALIBRATED_BIAS)) {
		__le16 *data16 = (__le16 *)(data + 1);
		u8 read_addr;

		data_buff = (data[0] == CW_MAGNETIC_UNCALIBRATED_BIAS) ?
				CW_MAGNETIC_UNCALIBRATED :
				CW_GYROSCOPE_UNCALIBRATED;
		data_buff += (is_wake) ? 32 : 0;

		bias_event[0] = le16_to_cpup(data16 + 1);
		bias_event[1] = le16_to_cpup(data16 + 2);
		bias_event[2] = le16_to_cpup(data16 + 3);

		read_addr = (is_wake) ? CWSTM32_WAKE_UP_BATCH_MODE_DATA_QUEUE :
					CWSTM32_BATCH_MODE_DATA_QUEUE;
		ret = CWMCU_i2c_read(mcu_data, read_addr, data, 9);
		if (ret >= 0) {
			(*i)++;
			timestamp_event = le16_to_cpup(data16);
			data_event[0] = le16_to_cpup(data16 + 1);
			data_event[1] = le16_to_cpup(data16 + 2);
			data_event[2] = le16_to_cpup(data16 + 3);

			handle_time_base = (is_wake) ?
						&mcu_data->wake_fifo_time_base :
						&mcu_data->time_base;

			if (DEBUG_FLAG_GSENSOR == 1) {
				I(
				  "Batch data: total count = %u, current "
				  "count = %d, event_id = %d, data(x, y, z) = "
				  "(%d, %d, %d), bias(x, y,  z) = "
				  "(%d, %d, %d), "
				  "timediff = %d, time_base = %llu, r_time = %llu\n"
				  , *event_count, *i, data_buff
				  , data_event[0], data_event[1], data_event[2]
				  , bias_event[0], bias_event[1], bias_event[2]
				  , timestamp_event
				  , *handle_time_base
				  , *handle_time_base + timestamp_event
				  );
			}

			cw_send_event_special(mcu_data, data_buff,
					      data_event,
					      bias_event,
					      timestamp_event +
					      *handle_time_base);
		} else {
			E("Read Uncalibrated data fails, ret = %d\n", ret);
		}
	} else {
		__le16 *data16 = (__le16 *)(data + 1);

		timestamp_event = le16_to_cpup(data16);
		data_event[0] = le16_to_cpup(data16 + 1);
		data_event[1] = le16_to_cpup(data16 + 2);
		data_event[2] = le16_to_cpup(data16 + 3);

		data[0] += (is_wake) ? 32 : 0;

		handle_time_base = (is_wake) ?
					&mcu_data->wake_fifo_time_base :
					&mcu_data->time_base;

		if (DEBUG_FLAG_GSENSOR == 1) {
			I(
			  "Batch data: total count = %u, current count = %d, "
			  "event_id = %d, data(x, y, z) = (%d, %d, %d), "
			  "timediff = %d, time_base = %llu, r_time = %llu\n"
			  , *event_count, *i, data[0]
			  , data_event[0], data_event[1], data_event[2]
			  , timestamp_event
			  , *handle_time_base
			  , *handle_time_base + timestamp_event
			  );
		}

		if ((data[0] == CW_MAGNETIC) || (data[0] == CW_ORIENTATION)) {
			int rc;
			u8 accuracy;
			u16 bias_event[REPORT_EVENT_COMMON_LEN] = {0};

			rc = CWMCU_i2c_read(mcu_data,
					    CW_I2C_REG_SENSORS_ACCURACY_MAG,
					    &accuracy, 1);
			if (rc < 0) {
				E(
				  "%s: read ACCURACY_MAG fails, rc = "
				  "%d\n", __func__, rc);
				accuracy = 3;
			}
			bias_event[0] = accuracy;

			cw_send_event_special(mcu_data, data[0], data_event,
					      bias_event,
					      timestamp_event +
					      *handle_time_base);
		} else {
			cw_send_event(mcu_data, data[0], data_event,
				      timestamp_event + *handle_time_base);
		}
	}
	return is_meta_read;
}

/* Return if META is read out */
static bool cwmcu_batch_fifo_read(struct cwmcu_data *mcu_data, int queue_id)
{
	s32 ret;
	int i, nodata_count = 0;
	u32 *event_count;
	u8 event_count_data[4] = {0};
	u8 reg_addr;
	bool is_meta_read = false;

	mcu_data->batch_fifo_read_start_jiffies = jiffies;

	mutex_lock(&mcu_data->lock);

	reg_addr = (queue_id)
			? CWSTM32_WAKE_UP_BATCH_MODE_DATA_COUNTER
			: CWSTM32_BATCH_MODE_DATA_COUNTER;

	ret = CWMCU_i2c_read(mcu_data, reg_addr, event_count_data,
			     sizeof(event_count_data));
	if (ret < 0) {
		D(
		  "Read Batched data Counter fail, ret = %d, queue_id"
		  " = %d\n", ret, queue_id);
	}

	event_count = (u32 *)(&event_count_data[0]);

	if (*event_count > MAX_EVENT_COUNT) {
		I("%s: event_count = %u, strange, queue_id = %d\n",
		  __func__, *event_count, queue_id);
		*event_count = 0;
	}

	D("%s: event_count = %u, queue_id = %d\n", __func__,
	  *event_count, queue_id);

	reg_addr = (queue_id) ? CWSTM32_WAKE_UP_BATCH_MODE_DATA_QUEUE :
		 CWSTM32_BATCH_MODE_DATA_QUEUE;

	if (*event_count > 500) I("%s:++ event_count = %d, queue_id = %d\n", __func__, *event_count, queue_id);

	for (i = 0; i < *event_count; i++) {
		__le64 data64[2];
		u8 *data = (u8 *)data64;

		if (atomic_read(&mcu_data->enter_suspend)) {
			I("Drop batch event because system suspend\n");
			break;
		} else if (atomic_read(&mcu_data->critical_sect)) {
			I("%s: Stop reading batch events because "
			  "cri = %d, sleep 10ms\n", __func__,
			  atomic_read(&mcu_data->critical_sect));
			msleep(10);
		}

		data = data + 7;

		ret = CWMCU_i2c_read(mcu_data, reg_addr, data, 9);
		if (ret >= 0) {
			/* check if there are no data from queue */
			if (data[0] != CWMCU_NODATA) {
				is_meta_read = report_iio(mcu_data, &i, data,
							  data64,
							  event_count,
							  queue_id);
				nodata_count = 0;
			} else {
				nodata_count++;
			}
		} else {
			E("Read Queue fails, ret = %d, queue_id = %d\n",
			  ret, queue_id);
		}

		if (nodata_count > 5) {
			I("%s: too many NO_DATA, mcu is clearing fifo\n", __func__);
			break;
		}
	}

	if (*event_count > 500) I("%s:-- event_count = %d, queue_id = %d\n", __func__, *event_count, queue_id);

	mutex_unlock(&mcu_data->lock);
	mcu_data->batch_fifo_read_end_jiffies = jiffies;

	return is_meta_read;
}

/* cwmcu_powermode_switch() must be held by caller */
static void cwmcu_batch_read(struct cwmcu_data *mcu_data)
{
	int j;
	u32 *non_wake_batch_list = (u32 *)&mcu_data->batched_list;
	u32 *wake_batch_list = (non_wake_batch_list + 1);

	D("%s++: batched_list = 0x%llx\n", __func__, mcu_data->batched_list);

	for (j = 0; j < 2; j++) {
		if ((!(*non_wake_batch_list) && (j == 0)) ||
		    (!(*wake_batch_list) && (j == 1))) {
			D(
			  "%s++: nw_batched_list = 0x%x, w_batched_list = 0x%x,"
			  " j = %d, continue\n",
			   __func__, *non_wake_batch_list, *wake_batch_list, j);
			continue;
		}

		cwmcu_batch_fifo_read(mcu_data, j);
	}

	D("%s--: batched_list = 0x%llx\n", __func__, mcu_data->batched_list);
}

static void cwmcu_check_sensor_update(struct cwmcu_data *mcu_data)
{
	int id;
	s64 temp;

	do_gettimeofday(&mcu_data->now);
	temp = (mcu_data->now.tv_sec * NS_PER_US) + mcu_data->now.tv_usec;

	for (id = 0; id < CW_SENSORS_ID_TOTAL; id++) {
		mcu_data->time_diff[id] = temp - mcu_data->sensors_time[id];

		if (mcu_data->time_diff[id] < 0) {
			I("%s: time_diff overflow, id = %d\n", __func__, id);
			mcu_data->time_diff[id] = mcu_data->report_period[id];
		}

		if ((mcu_data->time_diff[id] >= mcu_data->report_period[id])
		    && (mcu_data->enabled_list & (1LL << id))) {
			mcu_data->sensors_time[id] = temp;
			mcu_data->update_list |= (1LL << id);
		} else
			mcu_data->update_list &= ~(1LL << id);
	}
}

static void cwmcu_read(struct cwmcu_data *mcu_data, struct iio_poll_func *pf)
{
	int i = 0;
	u16 light_adc = 0;
	u32 light_lux = 0;

	if (!s_mcu_data) {
		W("%s: probe not completed\n", __func__);
		return;
	}

	if (mcu_data->enabled_list) {

		cwmcu_check_sensor_update(mcu_data);

		for (i = 0; i < CW_SENSORS_ID_FW; i++) {
			if (is_continuous_sensor(i) &&
				(mcu_data->update_list & (1LL<<i)) &&
				(mcu_data->batch_timeout[i] == 0)) {
				cwmcu_powermode_switch(mcu_data, 1);
				cwmcu_batch_fifo_read(mcu_data, 0);
				cwmcu_powermode_switch(mcu_data, 0);
				break;
			}
		}

		for (i = CW_SENSORS_ID_FW + 1; i < CW_SENSORS_ID_TOTAL; i++) {
			if (is_continuous_sensor(i) &&
				(mcu_data->update_list & (1LL<<i)) &&
				(mcu_data->batch_timeout[i] == 0)) {
				cwmcu_powermode_switch(mcu_data, 1);
				cwmcu_batch_fifo_read(mcu_data, 1);
				cwmcu_powermode_switch(mcu_data, 0);
				break;
			}
		}

			if ((mcu_data->als_polling) &&
			    (mcu_data->update_list & (1LL<<CW_LIGHT))) {

		                u8 data[REPORT_EVENT_COMMON_LEN] = {0};
				u16 data_buff[REPORT_EVENT_COMMON_LEN] = {0};

				CWMCU_i2c_read_power(mcu_data, CWSTM32_READ_Light, data,
				                                                sizeof(data));
				if (atomic_read(&mcu_data->als_first_polling_event)) {
				        atomic_set(&mcu_data->als_first_polling_event, 0);
				} else {
				        if (data[0] == 0xFF) {
				                if (DEBUG_FLAG_LIGHT_POLLING >= 3)
				                        I("light polling data[1] = 0x%X,"
				                          " data[2] = 0x%X\n ", data[1], data[2]);

						/* sizeof(light_adc) is 0xFFFF */
				                light_adc = (data[2] << 8) | data[1];
						/*
						   MAX sizeof(light_lux) can reach to 0xFFFFFFFF,
						   meaning the "als_lux_ratio"_n can reach to 0x10001
						*/
				                light_lux = light_adc * mcu_data->als_lux_ratio_n;

				                if (DEBUG_FLAG_LIGHT_POLLING >= 2)
				                        I("light polling light_adc = 0x%X,"
				                          " light_lux = 0x%X\n ", light_adc, light_lux);

				                data_buff[0] = (u16)(light_lux & 0xFFFF);
				                data_buff[1] = (u16)((light_lux >> 16) & 0xFFFF);

				                mcu_data->light_last_data[0] = data_buff[0];
				                mcu_data->light_last_data[1] = data_buff[1];

				                cw_send_event(mcu_data, CW_LIGHT, data_buff, 0);

				                if (DEBUG_FLAG_LIGHT_POLLING >= 1)
				                        I("light polling ADC is %d, LUX is %d(0x%X 0x%X), "
				                          "ls_calibration is %u\n",
				                                 light_adc, light_lux, data_buff[0],
                                				 data_buff[1], mcu_data->ls_calibrated);
					}
				}
			}
	}

}

static int cwmcu_suspend(struct device *dev)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	int i;
	u8 data;

	if (!s_mcu_data) {
		W("%s: probe not completed\n", __func__);
		return 0;
	}
	atomic_set(&mcu_data->enter_suspend, 1);
	mcu_data->suspend_jiffies = jiffies;

	D("[CWMCU] %s++\n", __func__);

	disable_irq(mcu_data->IRQ);
	if (s_mcu_data && s_mcu_data->indio_dev
	    && s_mcu_data->indio_dev->pollfunc) {
		disable_irq_nosync(s_mcu_data->indio_dev->pollfunc->irq);
		D("%s: Disable irq = %d\n", __func__,
		  s_mcu_data->indio_dev->pollfunc->irq);
	}
	cancel_work_sync(&mcu_data->one_shot_work);
	D("%s: cancel_work_sync done\n", __func__);
	cancel_delayed_work_sync(&mcu_data->work);
	D("%s: cancel_delayed_work_sync done iio_work_done is %d\n", __func__,mcu_data->iio_work_done);

	/* Inform SensorHUB that CPU is going to suspend */
	data = 0;
	cwmcu_powermode_switch(mcu_data, 1);
	CWMCU_i2c_write(mcu_data, CW_CPU_STATUS_REG, &data, 1, 0);
	cwmcu_powermode_switch(mcu_data, 0);
	/*CWMCU_i2c_write_power(mcu_data, CW_CPU_STATUS_REG, &data, 1);*/
	D("%s: write_addr = 0x%x, write_data = 0x%x\n", __func__,
	  CW_CPU_STATUS_REG, data);

	atomic_set(&mcu_data->suspended, 1);

	for (i = 0; (mcu_data->power_on_counter != 0) &&
		    (gpio_get_value(mcu_data->gpio_wake_mcu) != 1) &&
		    (i < ACTIVE_RETRY_TIMES); i++)
		usleep_range(10, 20);

	gpio_set_value(mcu_data->gpio_wake_mcu, 1);
	mcu_data->power_on_counter = 0;

	D("[CWMCU] %s--\n", __func__);
	return 0;
}


static int cwmcu_resume(struct device *dev)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 data;

	if (!s_mcu_data) {
		W("%s: probe not completed\n", __func__);
		return 0;
	}

	mcu_data->resume_jiffies = jiffies;

	D("[CWMCU] %s++\n", __func__);
	atomic_set(&mcu_data->enter_suspend, 0);
	atomic_set(&mcu_data->suspended, 0);

	/* Inform SensorHUB that CPU is going to resume */
	data = 1;

        cwmcu_powermode_switch(mcu_data, 1);
        CWMCU_i2c_write(mcu_data, CW_CPU_STATUS_REG, &data, 1, 0);
        cwmcu_powermode_switch(mcu_data, 0);
	/*CWMCU_i2c_write_power(mcu_data, CW_CPU_STATUS_REG, &data, 1);*/
	D("%s: write_addr = 0x%x, write_data = 0x%x\n", __func__,
	  CW_CPU_STATUS_REG, data);

	if (mcu_data->w_activated_i2c
	    || mcu_data->w_re_init
	    || mcu_data->w_clear_fifo
	    || mcu_data->w_flush_fifo
	    || mcu_data->w_kick_start_mcu
	    || mcu_data->w_mcu_state_change
#ifdef CONFIG_AK8789_HALLSENSOR
	    || mcu_data->w_hall_inform_mcu
#endif
	    || mcu_data->w_batch_read
	   )
		queue_work(mcu_data->mcu_wq, &mcu_data->one_shot_work);

	if (mcu_data->enabled_list & IIO_SENSORS_MASK) {
		queue_delayed_work(mcu_data->mcu_wq, &mcu_data->work,
			msecs_to_jiffies(atomic_read(&mcu_data->delay)));
	}
	if (s_mcu_data && s_mcu_data->indio_dev
	    && s_mcu_data->indio_dev->pollfunc) {
		enable_irq(s_mcu_data->indio_dev->pollfunc->irq);
		D("%s: Enable irq = %d\n", __func__,
		  s_mcu_data->indio_dev->pollfunc->irq);
	}
	enable_irq(mcu_data->IRQ);

	D("[CWMCU] %s--\n", __func__);
	return 0;
}


#ifdef MCU_WARN_MSGS
static void print_warn_msg(struct cwmcu_data *mcu_data,
			   char *buf, u32 len, u32 index)
{
	int ret;
	char *buf_start = buf;

	while ((buf - buf_start) < len) {
		ret = min((u32)WARN_MSG_BLOCK_LEN,
			  (u32)(len - (buf - buf_start)));
		ret = CWMCU_i2c_read_power(mcu_data,
				     CW_I2C_REG_WARN_MSG_BUFFER,
				     buf, ret);
		if (ret == 0) {
			break;
		} else if (ret < 0) {
			E("%s: warn i2c_read: ret = %d\n", __func__, ret);
			break;
		} else
			buf += ret;
	}
	printk(KERN_WARNING "[S_HUB][CW_MCU] Warning MSG[%d] = %.*s",
			index, (int)(buf - buf_start), buf_start);
}
#endif

void easy_access_irq_handler(struct cwmcu_data *mcu_data)
{
	int ret;
	u8 clear_intr;
	u8 sensor_id = HTC_GESTURE_MOTION;
	u8 data[6];
	u16 data_buff[REPORT_EVENT_COMMON_LEN] = {0};

	ret = CWMCU_i2c_read(mcu_data, CWSTM32_READ_Gesture_Motion, data, sizeof(data));

	if (data[0] == HTC_GESTURE_MOTION_TYPE_SWIPE_UP ||
		  data[0] == HTC_GESTURE_MOTION_TYPE_SWIPE_DOWN ||
		  data[0] == HTC_GESTURE_MOTION_TYPE_SWIPE_LEFT ||
		  data[0] == HTC_GESTURE_MOTION_TYPE_SWIPE_RIGHT ||
		  data[0] == HTC_GESTURE_MOTION_TYPE_LAUNCH_CAMERA ||
		  data[0] == HTC_GESTURE_MOTION_TYPE_DOUBLE_TAP) {
		I("%s: gesture detected = %d\n", __func__, data[0]);
		data_buff[0] = data[0];
		cw_send_event(mcu_data, sensor_id, data_buff, 0);
		mcu_data->sensors_time[sensor_id] = 0;
		mcu_data->power_key_pressed = 0;
	} else {
		E("%s: no gesture motion type = %d\n", __func__, data[0]);
	}

#if 0
	if (vib_trigger) {
		vib_trigger_event(vib_trigger, mcu_data->vibrate_ms);
	} else {
		E("%s: no vib_trigger\n", __func__);
	}
#endif

	clear_intr = CW_MCU_INT_BIT_HTC_GESTURE_MOTION;
	ret = CWMCU_i2c_write(mcu_data, CWSTM32_INT_ST4, &clear_intr, 1, 1);
}

static irqreturn_t cwmcu_irq_handler(int irq, void *handle)
{
	struct cwmcu_data *mcu_data = handle;
	s32 ret;
	u8 INT_st1 = 0, INT_st2 = 0, INT_st3 = 0, INT_st4 = 0, err_st = 0, batch_st = 0;
	u8 clear_intr;
	u16 ps_autok_thd = 0 , ps_min_adc = 0;
	u16 ps_adc = 0;
	u16 p_status = 0;
	u16 light_adc = 0;
	u32 light_lux = 0;
#ifdef SHUB_DLOAD_SUPPORT
	int mcu_status_level;
#endif //SHUB_DLOAD_SUPPORT

	if (!s_mcu_data) {
		E("%s: probe not completed\n", __func__);
		return IRQ_HANDLED;
	}

	D("[CWMCU] %s\n", __func__);

#ifdef SHUB_DLOAD_SUPPORT
	mcu_status_level = MCU2CPU_STATUS_GPIO_LEVEL(mcu_data);
	//if gpio_mcu_status_level changes
	if (MCU2CPU_STATUS_GPIO_LEVEL_DLOAD == mcu_status_level || mcu_data->gpio_mcu_status_level != mcu_status_level) {
		mcu_data->gpio_mcu_status_level = mcu_status_level;
		I("%s gpio_mcu_status_level:%d\n", __func__, mcu_data->gpio_mcu_status_level);
		mcu_data->w_mcu_state_change = true;
		queue_work(mcu_data->mcu_wq, &mcu_data->one_shot_work);
	}
#endif //SHUB_DLOAD_SUPPORT

	if (MCU_IN_DLOAD() || MCU_IN_BOOTLOADER() || mcu_data->gpio_mcu_status_level == MCU2CPU_STATUS_GPIO_LEVEL_DLOAD) {
		I("%s skip, s_mcu_state:%d, gpio_mcu_status_level:%d\n", __func__, s_mcu_state, mcu_data->gpio_mcu_status_level);
		goto EXIT;
	}

	cwmcu_powermode_switch(mcu_data, 1);

	ret = CWMCU_i2c_read(mcu_data, CWSTM32_INT_ST1, &INT_st1, 1);
	if (ret < 0)INT_st1 = 0;

	ret = CWMCU_i2c_read(mcu_data, CWSTM32_INT_ST2, &INT_st2, 1);
	if (ret < 0)INT_st2 = 0;

	ret = CWMCU_i2c_read(mcu_data, CWSTM32_INT_ST3, &INT_st3, 1);
	if (ret < 0)INT_st3 = 0;

	ret = CWMCU_i2c_read(mcu_data, CWSTM32_INT_ST4, &INT_st4, 1);
	if (ret < 0)INT_st4 = 0;

	ret = CWMCU_i2c_read(mcu_data, CWSTM32_ERR_ST, &err_st, 1);
	if (ret < 0)err_st = 0;

	ret = CWMCU_i2c_read(mcu_data, CWSTM32_BATCH_MODE_COMMAND, &batch_st, 1);
	if (ret < 0)batch_st = 0;

	I(
	  "%s: INT_st(1, 2, 3, 4) = (0x%x, 0x%x, 0x%x, 0x%x), err_st = 0x%x"
	  ", batch_st = 0x%x\n",
	  __func__, INT_st1, INT_st2, INT_st3, INT_st4, err_st, batch_st);

	/* INT_st1: bit 4 */
	if (INT_st1 & CW_MCU_INT_BIT_PROXIMITY) {
		u8 data[REPORT_EVENT_PROXIMITY_LEN] = {0};
		u16 data_buff[REPORT_EVENT_PROXIMITY_LEN] = {0};

		wake_lock_timeout(&mcu_data->ps_read_wake_lock, 2*HZ);

		if(mcu_data->enabled_list & (1<<proximity)) {
			CWMCU_i2c_read_power(mcu_data, CWSTM32_READ_Proximity, data,
									 sizeof(data));
			if(data[0] < 2) {
				mcu_data->sensors_time[proximity] =
					mcu_data->sensors_time[proximity] -
					mcu_data->report_period[proximity];

                                ps_autok_thd    = (data[6] << 8) | data[5];
                                ps_min_adc      = (data[4] << 8) | data[3];
                                ps_adc          = (data[2] << 8) | data[1];
                                p_status        = data[0];
                                data_buff[0]    = data[0];

				cw_send_event(mcu_data, CW_PROXIMITY, data_buff, 0);

                                I("Proximity interrupt occur value is %d adc is"
                                    " 0x%X, while min_adc is 0x%X, autok_thd is 0x%X,"
                                    " ps_calibration is %d\n",
                                    data[0], ps_adc, ps_min_adc, ps_autok_thd,
                                    mcu_data->ps_calibrated);
			}
		}
		else {
                        D("Proximity interrupt occur value is %d adc is"
                          " 0x%X, while min_adc is 0x%X, autok_thd is 0x%X"
                          " ps_calibration is %d (message only)\n",
                          data[0], ps_adc, ps_min_adc, ps_autok_thd,
                          mcu_data->ps_calibrated);
		}

		if(data[0] < 2) {
		clear_intr = CW_MCU_INT_BIT_PROXIMITY;
			ret = CWMCU_i2c_write(mcu_data, CWSTM32_INT_ST1, &clear_intr, 1, 1);
			CWMCU_i2c_write(mcu_data, CWSTM32_READ_Proximity, &data[0], 1, 1);
		}
	}

	/* INT_st1: bit 3 */
	if (INT_st1 & CW_MCU_INT_BIT_LIGHT) {
		u8 data[REPORT_EVENT_COMMON_LEN] = {0};
		u16 data_buff[REPORT_EVENT_COMMON_LEN] = {0};

		if (mcu_data->enabled_list & (1<<light)) {
			CWMCU_i2c_read_power(mcu_data, CWSTM32_READ_Light, data, sizeof(data));
			if (data[0] < 10 || data[0] == 0xFF) {
				mcu_data->sensors_time[light] =
						mcu_data->sensors_time[light] -
						mcu_data->report_period[light];
				if (mcu_data->als_polling) {
					/* sizeof(light_adc) is 0xFFFF */
					light_adc = (data[2] << 8) | data[1];
					/*
					   MAX sizeof(light_lux) can reach to 0xFFFFFFFF,
					   meaning the "als_lux_ratio"_n can reach to 0x10001
					*/
					light_lux = light_adc * mcu_data->als_lux_ratio_n;
					data_buff[0] = light_lux & 0xFFFF;
					data_buff[1] = (light_lux >> 16) & 0xFFFF;

					mcu_data->light_last_data[0] = data_buff[0];
					mcu_data->light_last_data[1] = data_buff[1];
				}
				else {
					light_adc = (data[2] << 8) | data[1];
					data_buff[0] = data[0];
					mcu_data->light_last_data[0] = data_buff[0];
				}

				/*
				   There are two initial state mechanism existed in this driver
					1. (ABORTED in MCU_411) Driven by driver, stored the lastest
					   light sensor data for inititial state.
					2. Driven by MCU, MCU will initiate an LS interrupt when it
					   ready itself.
				*/
				cw_send_event(mcu_data, CW_LIGHT, data_buff, 0);
				I(
				  "light interrupt occur value is %u, adc "
				  "is %x ls_calibration is %u\n",
					data[0], light_adc,
					mcu_data->ls_calibrated);
			} else {
				D(
				  "light interrupt occur value is %u, adc is"
				  " %x ls_calibration is %u (message only)\n",
					data[0], light_adc,
					mcu_data->ls_calibrated);
			}
		}
		if (data[0] < 10 || data[0] == 0xFF) {
			clear_intr = CW_MCU_INT_BIT_LIGHT;
			CWMCU_i2c_write(mcu_data, CWSTM32_INT_ST1, &clear_intr,
					1, 1);
		}
	}

	/* INT_st2: bit 5 */
	if (INT_st2 & CW_MCU_INT_BIT_SHUB_BOOTUP) {
		I("%s: CW_MCU_INT_BIT_SHUB_BOOTUP\n", __func__);

		mcu_data->mcu_bootup = true;
		mcu_data->w_mcu_state_change = true;
		queue_work(mcu_data->mcu_wq, &mcu_data->one_shot_work);

		clear_intr = CW_MCU_INT_BIT_SHUB_BOOTUP;
		ret = CWMCU_i2c_write(mcu_data, CWSTM32_INT_ST2, &clear_intr, 1, 1);
	}

	/* INT_st2: bit 6 */
	if (INT_st2 & CW_MCU_INT_BIT_LOG_AVAILABLE) {
		I("%s: CW_MCU_INT_BIT_LOG_AVAILABLE\n", __func__);

#ifdef SHUB_LOGGING_SUPPORT
		complete(&s_mcu_log_avail);
#endif //SHUB_LOGGING_SUPPORT

		clear_intr = CW_MCU_INT_BIT_LOG_AVAILABLE;
		ret = CWMCU_i2c_write(mcu_data, CWSTM32_INT_ST2, &clear_intr, 1, 1);
	}

	/* INT_st3: bit 4 */
	if (INT_st3 & CW_MCU_INT_BIT_SIGNIFICANT_MOTION) {
		if (mcu_data->enabled_list & (1LL << CW_SIGNIFICANT_MOTION)) {
			u16 data_buff[REPORT_EVENT_COMMON_LEN] = {0};
			__le64 data64[2];
			u8 *data = (u8 *)data64;

			data = data + sizeof(__le64) - sizeof(u8);

			ret = CWMCU_i2c_read(mcu_data,
					     CWSTM32_READ_SIGNIFICANT_MOTION,
					     data, sizeof(u8) + sizeof(__le64));
			if (ret >= 0) {
				u64 timestamp_event;
				__le64 *le64_timestamp = data64 + 1;

				timestamp_event = le64_to_cpu(*le64_timestamp);

				mcu_data->sensors_time[CW_SIGNIFICANT_MOTION]
						= 0;

				wake_lock_timeout(
					&mcu_data->significant_wake_lock, HZ);

				data_buff[0] = 1;
				cw_send_event(mcu_data, CW_SIGNIFICANT_MOTION,
					      data_buff, timestamp_event);

				if (DEBUG_FLAG_GSENSOR == 1) {
					I("%s: Significant timestamp = %llu\n"
					  , __func__, timestamp_event);
				}
			} else {
				E(
				  "Read CWSTM32_READ_SIGNIFICANT_MOTION fails,"
				  " ret = %d\n", ret);
			}
		}
		clear_intr = CW_MCU_INT_BIT_SIGNIFICANT_MOTION;
		CWMCU_i2c_write(mcu_data, CWSTM32_INT_ST3, &clear_intr, 1, 1);
	}

	/* INT_st3: bit 5 */
	if (INT_st3 & CW_MCU_INT_BIT_STEP_DETECTOR) {
		if (mcu_data->enabled_list & ((1ULL << CW_STEP_DETECTOR) |
					      (1ULL << CW_STEP_DETECTOR_W)))
			cwmcu_batch_read(mcu_data);

		clear_intr = CW_MCU_INT_BIT_STEP_DETECTOR;
		CWMCU_i2c_write(mcu_data, CWSTM32_INT_ST3, &clear_intr, 1, 1);
	}

	/* INT_st3: bit 6 */
	if (INT_st3 & CW_MCU_INT_BIT_STEP_COUNTER) {
		if (mcu_data->enabled_list & (1LL << CW_STEP_COUNTER_W))
			cwmcu_batch_read(mcu_data);

		if (mcu_data->enabled_list & (1LL << CW_STEP_COUNTER)) {
			__le64 data64[2];
			u8 *data = (u8 *)data64;
			__le32 step_fw;

			ret = CWMCU_i2c_read(mcu_data,
					     CWSTM32_READ_STEP_COUNTER,
					     data, 12);
			if (ret >= 0) {
				step_fw = *(__le32 *)(data + 8);
				D("%s: From Firmware, step = %u\n",
				  __func__, le32_to_cpu(step_fw));

				mcu_data->sensors_time[CW_STEP_COUNTER]
					= 0;

				report_step_counter(mcu_data,
						    le32_to_cpu(step_fw)
						    , le64_to_cpu(
							data64[0])
						    , false);

				D(
				  "%s: Step Counter INT, step = %llu"
				  ", timestamp = %llu\n"
				  , __func__
				  , mcu_data->step_counter_base
				    + le32_to_cpu(step_fw)
				  , le64_to_cpu(data64[0]));
			} else {
				E(
				  "%s: Step Counter i2c read fails, "
				  "ret = %d\n", __func__, ret);
			}
		}
		clear_intr = CW_MCU_INT_BIT_STEP_COUNTER;
		CWMCU_i2c_write(mcu_data, CWSTM32_INT_ST3, &clear_intr, 1, 1);
	}

	/* INT_st4: bit 0 */
	if (INT_st4 & CW_MCU_INT_BIT_HTC_GESTURE_MOTION) {
		wake_lock_timeout(&mcu_data->gesture_motion_wake_lock, 2 * HZ);
		easy_access_irq_handler(mcu_data);
	}

	/* INT_st4: bit 4 */
	if (INT_st4 & CW_MCU_INT_BIT_ANY_MOTION) {
		if (mcu_data->enabled_list & (1 << HTC_ANY_MOTION)) {
			s16 data_buff[REPORT_EVENT_COMMON_LEN] = {0};

			mcu_data->sensors_time[HTC_ANY_MOTION] = 0;

			wake_lock_timeout(&mcu_data->any_motion_wake_lock,
					  1 * HZ);

			data_buff[0] = 1;
			cw_send_event(mcu_data, HTC_ANY_MOTION, data_buff, 0);

			D("%s: HTC_ANY_MOTION occurs!\n", __func__);
		}
		clear_intr = CW_MCU_INT_BIT_ANY_MOTION;
		ret = CWMCU_i2c_write(mcu_data, CWSTM32_INT_ST4, &clear_intr,
				      1, 1);
	}

#ifdef MCU_WARN_MSGS
	/* err_st: bit 5 */
	if (err_st & CW_MCU_INT_BIT_ERROR_WARN_MSG) {
		u8 buf_len[WARN_MSG_BUFFER_LEN_SIZE] = {0};

		ret = CWMCU_i2c_read(mcu_data, CW_I2C_REG_WARN_MSG_BUFFER_LEN,
				     buf_len, sizeof(buf_len));
		if (ret >= 0) {
			int i;
			char buf[WARN_MSG_PER_ITEM_LEN];

			for (i = 0; i < WARN_MSG_BUFFER_LEN_SIZE; i++) {
				if (buf_len[i] <= WARN_MSG_PER_ITEM_LEN)
					print_warn_msg(mcu_data, buf,
						       buf_len[i], i);
			}
		} else {
			E("%s: Warn MSG read fails, ret = %d\n",
						__func__, ret);
		}
		clear_intr = CW_MCU_INT_BIT_ERROR_WARN_MSG;
		ret = CWMCU_i2c_write(mcu_data, CWSTM32_ERR_ST, &clear_intr, 1, 1);
	}
#endif

	/* err_st: bit 6 */
	if (err_st & CW_MCU_INT_BIT_ERROR_MCU_EXCEPTION) {
		bool reset_done;

		E("[CWMCU] MCU_EXCEPTION \n");

		mutex_lock(&s_activated_i2c_lock);
		reset_done = reset_hub(mcu_data, false);
		mutex_unlock(&s_activated_i2c_lock);

		clear_intr = CW_MCU_INT_BIT_ERROR_MCU_EXCEPTION;
		ret = CWMCU_i2c_write(mcu_data, CWSTM32_ERR_ST, &clear_intr, 1, 1);
	}

	/* err_st: bit 7 */
	if (err_st & CW_MCU_INT_BIT_ERROR_WATCHDOG_RESET) {
		u8 data[WATCHDOG_STATUS_LEN] = {0};

		E("[CWMCU] Watch Dog Reset\n");
		msleep(5);

		ret = CWMCU_i2c_read(mcu_data, CW_I2C_REG_WATCHDOG_STATUS,
				     data, WATCHDOG_STATUS_LEN);
		if (ret >= 0) {
			int i;

			for (i = 0; i < WATCHDOG_STATUS_LEN; i++) {
				E("%s: Watchdog Status[%d] = 0x%x\n",
					__func__, i, data[i]);
			}
		} else {
			E("%s: Watchdog status dump fails, ret = %d\n",
						__func__, ret);
		}

		mcu_data->w_re_init = true;
		queue_work(mcu_data->mcu_wq, &mcu_data->one_shot_work);

		clear_intr = CW_MCU_INT_BIT_ERROR_WATCHDOG_RESET;
		ret = CWMCU_i2c_write(mcu_data, CWSTM32_ERR_ST, &clear_intr, 1, 1);
	}

	/* batch_st */
	if (batch_st & CW_MCU_INT_BIT_BATCH_INT_MASK) {
		mcu_data->w_batch_read = true;
		queue_work(mcu_data->mcu_wq, &mcu_data->one_shot_work);
	}

	cwmcu_powermode_switch(mcu_data, 0);

EXIT:
	return IRQ_HANDLED;
}

/*=======iio device reg=========*/
static void iio_trigger_work(struct irq_work *work)
{
	struct cwmcu_data *mcu_data = container_of((struct irq_work *)work,
					struct cwmcu_data, iio_irq_work);

	iio_trigger_poll(mcu_data->trig);
}

static irqreturn_t cw_trigger_handler(int irq, void *p)
{
	struct iio_poll_func *pf = p;
	struct iio_dev *indio_dev = pf->indio_dev;
	struct cwmcu_data *mcu_data = iio_priv(indio_dev);

	cwmcu_read(mcu_data, pf);
	mcu_data->iio_work_done = 0;
	mutex_lock(&mcu_data->lock);
	iio_trigger_notify_done(mcu_data->indio_dev->trig);
	mutex_unlock(&mcu_data->lock);
	mcu_data->iio_work_done = 1;
	return IRQ_HANDLED;
}

static const struct iio_buffer_setup_ops cw_buffer_setup_ops = {
#if 0 // seems removed from linux 3.14
	.preenable = &iio_sw_buffer_preenable,
#endif
	.postenable = &iio_triggered_buffer_postenable,
	.predisable = &iio_triggered_buffer_predisable,
};

static int cw_pseudo_irq_enable(struct iio_dev *indio_dev)
{
	struct cwmcu_data *mcu_data = iio_priv(indio_dev);

	if (!atomic_cmpxchg(&mcu_data->pseudo_irq_enable, 0, 1)) {
		I("%s:\n", __func__);
		cancel_delayed_work_sync(&mcu_data->work);
		queue_delayed_work(mcu_data->mcu_wq, &mcu_data->work, 0);
	}

	return 0;
}

static int cw_pseudo_irq_disable(struct iio_dev *indio_dev)
{
	struct cwmcu_data *mcu_data = iio_priv(indio_dev);

	if (atomic_cmpxchg(&mcu_data->pseudo_irq_enable, 1, 0)) {
		cancel_delayed_work_sync(&mcu_data->work);
		I("%s:\n", __func__);
	}
	return 0;
}

static int cw_set_pseudo_irq(struct iio_dev *indio_dev, int enable)
{
	if (enable)
		cw_pseudo_irq_enable(indio_dev);
	else
		cw_pseudo_irq_disable(indio_dev);

	return 0;
}

static int cw_data_rdy_trigger_set_state(struct iio_trigger *trig,
		bool state)
{
	struct iio_dev *indio_dev =
			(struct iio_dev *)iio_trigger_get_drvdata(trig);
	struct cwmcu_data *mcu_data = iio_priv(indio_dev);

	mutex_lock(&mcu_data->mutex_lock);
	cw_set_pseudo_irq(indio_dev, state);
	mutex_unlock(&mcu_data->mutex_lock);

	return 0;
}

static const struct iio_trigger_ops cw_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = &cw_data_rdy_trigger_set_state,
};

static int cw_probe_trigger(struct iio_dev *iio_dev)
{
	struct cwmcu_data *mcu_data = iio_priv(iio_dev);
	int ret;

	iio_dev->pollfunc = iio_alloc_pollfunc(&iio_pollfunc_store_time,
			&cw_trigger_handler, IRQF_ONESHOT, iio_dev,
			"%s_consumer%d", iio_dev->name, iio_dev->id);
	if (iio_dev->pollfunc == NULL) {
		ret = -ENOMEM;
		goto error_ret;
	}
	mcu_data->trig = iio_trigger_alloc("%s-dev%d",
			iio_dev->name,
			iio_dev->id);
	if (!mcu_data->trig) {
		ret = -ENOMEM;
		goto error_dealloc_pollfunc;
	}
	mcu_data->trig->dev.parent = mcu_data->mcu_dev;
	mcu_data->trig->ops = &cw_trigger_ops;
	iio_trigger_set_drvdata(mcu_data->trig, iio_dev);

	ret = iio_trigger_register(mcu_data->trig);
	if (ret)
		goto error_free_trig;

	return 0;

error_free_trig:
	iio_trigger_free(mcu_data->trig);
error_dealloc_pollfunc:
	iio_dealloc_pollfunc(iio_dev->pollfunc);
error_ret:
	return ret;
}

static int cw_probe_buffer(struct iio_dev *iio_dev)
{
	int ret;
	struct iio_buffer *buffer;

	buffer = iio_kfifo_allocate(iio_dev);
	if (!buffer) {
		ret = -ENOMEM;
		goto error_ret;
	}

	buffer->scan_timestamp = true;
	iio_dev->buffer = buffer;
	iio_dev->setup_ops = &cw_buffer_setup_ops;
	iio_dev->modes |= INDIO_BUFFER_TRIGGERED;
	ret = iio_buffer_register(iio_dev, iio_dev->channels,
				  iio_dev->num_channels);
	if (ret)
		goto error_free_buf;

	iio_scan_mask_set(iio_dev, iio_dev->buffer, CW_SCAN_ID);
	iio_scan_mask_set(iio_dev, iio_dev->buffer, CW_SCAN_X);
	iio_scan_mask_set(iio_dev, iio_dev->buffer, CW_SCAN_Y);
	iio_scan_mask_set(iio_dev, iio_dev->buffer, CW_SCAN_Z);
	return 0;

error_free_buf:
	iio_kfifo_free(iio_dev->buffer);
error_ret:
	return ret;
}


static int cw_read_raw(struct iio_dev *indio_dev,
		       struct iio_chan_spec const *chan,
		       int *val,
		       int *val2,
		       long mask) {
	struct cwmcu_data *mcu_data = iio_priv(indio_dev);
	int ret = -EINVAL;

	if (chan->type != IIO_ACCEL)
		return ret;

	mutex_lock(&mcu_data->lock);
	switch (mask) {
	case 0:
		*val = mcu_data->iio_data[chan->channel2 - IIO_MOD_X];
		ret = IIO_VAL_INT;
		break;
	case IIO_CHAN_INFO_SCALE:
		/* Gain : counts / uT = 1000 [nT] */
		/* Scaling factor : 1000000 / Gain = 1000 */
		*val = 0;
		*val2 = 1000;
		ret = IIO_VAL_INT_PLUS_MICRO;
		break;
	}
	mutex_unlock(&mcu_data->lock);

	return ret;
}

#define CW_CHANNEL(axis)			\
{						\
	.type = IIO_ACCEL,			\
	.modified = 1,				\
	.channel2 = axis+1,			\
	.info_mask_separate = BIT(IIO_CHAN_INFO_SCALE),	\
	.scan_index = axis,			\
	.scan_type = {				\
			.sign = 'u',		\
			.realbits = 32,		\
			.storagebits = 32,	\
			.shift = 0,		\
			.endianness = IIO_BE,	\
		}, \
}
	/*.scan_type = IIO_ST('u', 32, 32, 0) */

static const struct iio_chan_spec cw_channels[] = {
	CW_CHANNEL(CW_SCAN_ID),
	CW_CHANNEL(CW_SCAN_X),
	CW_CHANNEL(CW_SCAN_Y),
	CW_CHANNEL(CW_SCAN_Z),
	IIO_CHAN_SOFT_TIMESTAMP(CW_SCAN_TIMESTAMP)
};

static const struct iio_info cw_info = {
	.read_raw = &cw_read_raw,
	.driver_module = THIS_MODULE,
};

static int mcu_fetch_cali_data(struct cwmcu_data *mcu_data, const char *path) {
	struct device_node *cali_data_node = NULL;
	unsigned char *cali_data = NULL;
	int cali_data_size, i = 0;

	cali_data_node = of_find_node_by_path(path);
	if (cali_data_node != NULL) {
		cali_data = (unsigned char *)of_get_property(cali_data_node,
			G_SENSOR_FLASH_DATA, &cali_data_size);
		if (cali_data != NULL) {
			for (i = 0; (i < cali_data_size) && (i < 4); i++) {
				mcu_data->gs_kvalue |= (cali_data[i] << (i * 8));
			}
		} else {
			I("cali_data not found for g-sensor\n");
		}
		I("gs_kvalue = %08x\n", mcu_data->gs_kvalue);

		cali_data = (unsigned char *)of_get_property(cali_data_node,
			GYRO_SENSOR_FLASH_DATA, &cali_data_size);
		if (cali_data != NULL) {
			for (i = 0; (i < cali_data_size) && (i < 4); i++) {
				mcu_data->gy_kvalue |= (cali_data[i] << (i * 8));
			}
			//TODO: remove it if we don't need g-sensor L/R calibration
			mcu_data->gs_kvalue_L1 = (cali_data[5] << 8) | cali_data[4];
			mcu_data->gs_kvalue_L2 = (cali_data[7] << 8) | cali_data[6];
			mcu_data->gs_kvalue_L3 = (cali_data[9] << 8) | cali_data[8];
			mcu_data->gs_kvalue_R1 = (cali_data[11] << 8) | cali_data[10];
			mcu_data->gs_kvalue_R2 = (cali_data[13] << 8) | cali_data[12];
			mcu_data->gs_kvalue_R3 = (cali_data[15] << 8) | cali_data[14];
		} else {
			I("cali_data not found for gyro\n");
		}
		I("gy_kvalue = %08x\n", mcu_data->gy_kvalue);

		cali_data = (unsigned char *)of_get_property(cali_data_node,
			LIGHT_SENSOR_FLASH_DATA, &cali_data_size);
		if (cali_data != NULL) {
			for (i = 0; (i < cali_data_size) && (i < 4); i++) {
				mcu_data->als_kvalue |= (cali_data[i] << (i * 8));
			}
		} else {
			I("cali_data not found for light sensor\n");
		}
		I("als_kvalue = %08x\n", mcu_data->als_kvalue);

		cali_data = (unsigned char*)of_get_property(cali_data_node,
			PROX_SENSOR_FLASH_DATA, &cali_data_size);
		if (cali_data != NULL) {
			for (i = 0; (i < cali_data_size) && (i < 8); i++) {
				if (i < 4)
					mcu_data->ps_kheader |= (cali_data[i] << (i * 8));
				else
					mcu_data->ps_kvalue |= (cali_data[i] << ((i - 4) * 8));
			}
		} else {
			I("cali_data not found for proximity sensor\n");
		}
		I("ps_kheader = %08x, ps_kvalue = %08x\n", mcu_data->ps_kheader, mcu_data->ps_kvalue);

		cali_data = (unsigned char *)of_get_property(cali_data_node,
			BARO_SENSOR_FLASH_DATA, &cali_data_size);
		if (cali_data != NULL) {
			for (i = 0; (i < cali_data_size) && (i < 5); i++) {
				if (i == cali_data_size - 1)
					mcu_data->bs_kheader = cali_data[i];
				else
					mcu_data->bs_kvalue |= (cali_data[i] << (i * 8));
			}
		} else {
			I("cali_data not found for barometer\n");
		}
		I("bs_kheader = %08x, bs_kvalue = %08x\n", mcu_data->bs_kheader, mcu_data->bs_kvalue);
	} else {
		E("cali_data_node offset not found\n");
		return -1;
	}
	return 0;
}

static int mcu_parse_dt(struct device *dev, struct cwmcu_data *pdata)
{
	struct property *prop = NULL;
	struct device_node *dt = dev->of_node;
	u32 buf = 0;
	int rc;

	pdata->gpio_mcu_status = of_get_named_gpio(dt, "mcu,mcu_status-gpio",
					0);
	if (!gpio_is_valid(pdata->gpio_mcu_status)) {
		E("DT:gpio_mcu_status value is not valid %d\n", pdata->gpio_mcu_status);
	}
	else
		D("DT:gpio_mcu_status=%d\n", pdata->gpio_mcu_status);

	pdata->gpio_wake_mcu = of_get_named_gpio(dt, "mcu,Cpu_wake_mcu-gpio",
					0);
	if (!gpio_is_valid(pdata->gpio_wake_mcu))
		E("DT:gpio_wake_mcu value is not valid\n");
	else
		D("DT:gpio_wake_mcu=%d\n", pdata->gpio_wake_mcu);

	pdata->gpio_mcu_irq = of_get_named_gpio(dt, "mcu,intr-gpio", 0);
	if (!gpio_is_valid(pdata->gpio_mcu_irq))
		E("DT:gpio_mcu_irq value is not valid\n");
	else
		D("DT:gpio_mcu_irq=%d\n", pdata->gpio_mcu_irq);

	pdata->gpio_reset = of_get_named_gpio(dt, "mcu,Reset-gpio", 0);
	if (!gpio_is_valid(pdata->gpio_reset))
		E("DT:gpio_reset value is not valid\n");
	else
		D("DT:gpio_reset=%d\n", pdata->gpio_reset);

	pdata->gpio_chip_mode = of_get_named_gpio(dt, "mcu,Chip_mode-gpio", 0);
	if (!gpio_is_valid(pdata->gpio_chip_mode))
		E("DT:gpio_chip_mode value is not valid\n");
	else
		D("DT:gpio_chip_mode=%d\n", pdata->gpio_chip_mode);

	prop = of_find_property(dt, "mcu,gs_chip_layout", NULL);
	if (prop) {
		of_property_read_u32(dt, "mcu,gs_chip_layout", &buf);
		pdata->gs_chip_layout = buf;
		D("%s: chip_layout = %d\n", __func__, pdata->gs_chip_layout);
	} else
		E("%s: g_sensor,chip_layout not found", __func__);

	prop = of_find_property(dt, "touch_enable", NULL);
	if (prop) {
		of_property_read_u32(dt, "touch_enable", &buf);
		pdata->touch_enable = buf;
		I("%s: touch_enable = %d\n", __func__, pdata->touch_enable);
	} else
		E("%s: touch_enable not found", __func__);

	prop = of_find_property(dt, "vibrate_ms", NULL);
	if (prop) {
		of_property_read_u32(dt, "vibrate_ms", &buf);
		pdata->vibrate_ms = buf;
		I("%s: vibrate_ms = %d\n", __func__, pdata->vibrate_ms);
	} else
		E("%s: vibrate_ms not found", __func__);

	prop = of_find_property(dt, "mcu,acceleration_axes", NULL);
	if (prop) {
		of_property_read_u32(dt, "mcu,acceleration_axes", &buf);
		pdata->acceleration_axes = buf;
		I("%s: acceleration axes = %u\n", __func__,
			pdata->acceleration_axes);
	} else
		E("%s: g_sensor axes not found", __func__);

	prop = of_find_property(dt, "mcu,magnetic_axes", NULL);
	if (prop) {
		of_property_read_u32(dt, "mcu,magnetic_axes", &buf);
		pdata->magnetic_axes = buf;
		I("%s: Compass axes = %u\n", __func__, pdata->magnetic_axes);
	} else
		E("%s: Compass axes not found", __func__);

	prop = of_find_property(dt, "mcu,gyro_axes", NULL);
	if (prop) {
		of_property_read_u32(dt, "mcu,gyro_axes", &buf);
		pdata->gyro_axes = buf;
		I("%s: gyro axes = %u\n", __func__, pdata->gyro_axes);
	} else
		E("%s: gyro axes not found", __func__);

        prop = of_find_property(dt, "mcu,als_levels", NULL);
        if (prop) {
                of_property_read_u32_array(dt, "mcu,als_levels", adc_table, 10);
                pdata->als_levels = &adc_table[0];
	} else
		E("%s: Light sensor level not found", __func__);

        prop = of_find_property(dt, "mcu,als_goldl", NULL);
        if (prop) {
                of_property_read_u32(dt, "mcu,als_goldl", &buf);
                pdata->als_goldl = buf;
                I("%s: als_goldl = 0x%x\n",__func__, pdata->als_goldl);
        } else
                E("%s: als_goldl not found", __func__);

        prop = of_find_property(dt, "mcu,als_goldh", NULL);
        if (prop) {
                of_property_read_u32(dt, "mcu,als_goldh", &buf);
                pdata->als_goldh = buf;
                I("%s: als_goldh = 0x%x\n",__func__, pdata->als_goldh);
        } else
                E("%s: als_goldh not found", __func__);

        prop = of_find_property(dt, "mcu,als_polling", NULL);
        if (prop) {
                of_property_read_u32(dt, "mcu,als_polling", &buf);
                pdata->als_polling = buf;
                I("%s: als_polling = %d\n", __func__, pdata->als_polling);

                of_property_read_u32(dt, "mcu,als_lux_ratio_n", &buf);
                pdata->als_lux_ratio_n = buf;
                I("%s: als_lux_ratio_n = %d\n", __func__, pdata->als_lux_ratio_n);

                of_property_read_u32(dt, "mcu,als_lux_ratio_d", &buf);
                pdata->als_lux_ratio_d = buf;
                I("%s: als_lux_ratio_d = %d\n", __func__, pdata->als_lux_ratio_d);
        } else {
                pdata->als_polling = 0;
		pdata->als_lux_ratio_n = 1;
		pdata->als_lux_ratio_d = 1;
                E("%s: als_polling not found", __func__);
        }

        prop = of_find_property(dt, "mcu,ps_thd_fixed", NULL);
        if (prop) {
                of_property_read_u32(dt, "mcu,ps_thd_fixed", &buf);
                pdata->ps_thd_fixed = buf;
                I("%s: ps_thd_fixed = 0x%x\n",__func__, pdata->ps_thd_fixed);
        } else
                E("%s: ps_thd_fixed not found", __func__);

        prop = of_find_property(dt, "mcu,ps_thd_add", NULL);
        if (prop) {
                of_property_read_u32(dt, "mcu,ps_thd_add", &buf);
                pdata->ps_thd_add = buf;
                I("%s: ps_thd_add = 0x%x\n",__func__, pdata->ps_thd_add);
        } else
                E("%s: ps_thd_add not found", __func__);

	pdata->v_sr_2v85 = devm_regulator_get(dev, "V_SR_2V85");
	if (IS_ERR(pdata->v_sr_2v85)) {
		pdata->v_sr_2v85 = NULL;
		E("%s: Unable to get v_sr_2v85\n", __func__);
	} else {
		rc = regulator_set_optimum_mode(pdata->v_sr_2v85, 100000);
		if (rc < 0) {
			E("Unable to set HPM of regulator v_sr_2v85\n");
			return 0;
		}

		rc = regulator_enable(pdata->v_sr_2v85);
		if (rc)
			E("Unable to enable v_sr_2v85\n");
		else
			I("%s: Set v_sr_2v85 to HPM success\n", __func__);
	}

	pdata->v_srio_1v8 = devm_regulator_get(dev, "V_SRIO_1V8");
	if (IS_ERR(pdata->v_srio_1v8)) {
		pdata->v_srio_1v8 = NULL;
		E("%s: Unable to get v_srio_1v8\n", __func__);
	} else {
		rc = regulator_set_optimum_mode(pdata->v_srio_1v8, 100000);
		if (rc < 0) {
			E("Unable to set HPM of regulator v_srio_1v8\n");
			return 0;
		}

		rc = regulator_enable(pdata->v_srio_1v8);
		if (rc)
			E("Unable to enable v_srio_1v8\n");
		else
			I("%s: Set v_srio_1v8 to HPM success\n", __func__);
	}

	return 0;
}

static ssize_t bma250_clear_powerkey_pressed(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	unsigned long value;
	int error;

	error = kstrtol(buf, 10, &value);
	if (error) {
		E("%s: kstrtol fails, error = %d\n", __func__, error);
		return error;
	}

	if (value == 1)
		mcu_data->power_key_pressed = 1;
	else if (value == 0)
		mcu_data->power_key_pressed = 0;

	return count;
}

static ssize_t bma250_get_powerkry_pressed(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);

	return snprintf(buf, PAGE_SIZE, "%d\n", mcu_data->power_key_pressed);
}
static DEVICE_ATTR(clear_powerkey_flag, S_IRUSR|S_IWUSR|S_IRGRP|S_IWGRP,
		bma250_get_powerkry_pressed, bma250_clear_powerkey_pressed);

static ssize_t set_g_sensor_user_offset(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	char *token;
	char *str_buf;
	char *running;
	long input_val[3] = {0};
	int rc, i;
	s8 temp_kvalue[3] = {0};

	str_buf = kstrndup(buf, count, GFP_KERNEL);
	if (str_buf == NULL) {
		E("%s: cannot allocate buffer\n", __func__);
		return -1;
	}
	running = str_buf;

	for (i = 0; i < 3; i++) {
		token = strsep(&running, " ");
		if (token == NULL) {
			E("%s: token = NULL, i = %d\n", __func__, i);
			break;
		}

		rc = kstrtol(token, 10, &input_val[i]);
		if (rc) {
			E("%s: kstrtol fails, rc = %d, i = %d\n",
			  __func__, rc, i);
			kfree(str_buf);
			return rc;
		}
	}
	kfree(str_buf);

	I("%s: User Calibration(x, y, z) = (%ld, %ld, %ld)\n", __func__,
	  input_val[0], input_val[1], input_val[2]);

	temp_kvalue[0] = (s8)(input_val[0] / 3);
	temp_kvalue[1] = (s8)(input_val[1] / 3);
	temp_kvalue[2] = (s8)(input_val[2] / 3);

	I("%s: temp_kvalue(x, y, z) = (0x%x, 0x%x, 0x%x)\n", __func__,
	  temp_kvalue[0] & 0xFF, temp_kvalue[1] & 0xFF, temp_kvalue[2] & 0xFF);

	mcu_data->gs_kvalue = 0x67000000;
	mcu_data->gs_kvalue |= (temp_kvalue[2] & 0xFF);
	mcu_data->gs_kvalue |= (temp_kvalue[1] & 0xFF) << 8;
	mcu_data->gs_kvalue |= (temp_kvalue[0] & 0xFF) << 16;

	I("%s: gs_kvalue = 0x%x\n", __func__, mcu_data->gs_kvalue);

	cwmcu_set_g_sensor_kvalue(mcu_data);

	return count;
}

static DEVICE_ATTR(g_sensor_user_offset, S_IWUSR|S_IWGRP, NULL,
		   set_g_sensor_user_offset);

static ssize_t p_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d\n", p_status);
}
static DEVICE_ATTR(p_status, 0444, p_status_show, NULL);

static ssize_t ps_adc_show(struct device *dev,
               struct device_attribute *attr, char *buf)
{
        u8 data[REPORT_EVENT_PROXIMITY_LEN] = {0};
        u16 ps_adc = 0;
        u16 ps_autok_thd = 0;
        u8  ps_pocket_mode = 0;
        u8  ps_enable = 0;

        CWMCU_i2c_read_power(s_mcu_data, CWSTM32_READ_Proximity, data, sizeof(data));
        ps_adc          = (data[2] << 8) | data[1];
        ps_autok_thd    = (data[6] << 8) | data[5];
        ps_pocket_mode  = data[8];

        if((s_mcu_data->enabled_list & CW_MCU_INT_BIT_PROXIMITY) == 1)
                ps_enable = 1;

        return scnprintf(buf, PAGE_SIZE, "ADC[0x%04X], ENABLE = %d, intr_pin = "
                            "%d, ps_pocket_mode = %d, model = CM36686-MCU,"
                            " ps_autok_thd = %d\n",
                            ps_adc, ps_enable, CW_MCU_INT_BIT_PROXIMITY,
                            ps_pocket_mode, ps_autok_thd);
}
static DEVICE_ATTR(ps_adc, 0444, ps_adc_show, NULL);


static ssize_t set_gesture_motion(struct device *dev,
				  struct device_attribute *attr,
				  const char *buf, size_t count)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 *data;
	unsigned long val = 0;
	int i, rc;

	rc = kstrtol(buf, 16, &val);
	if (rc) {
		E("%s: kstrtol fails, error = %d\n", __func__, rc);
		return rc;
	}

	data = (u8 *)&val;

	I("%s: setting parameter = (0x%x, 0x%x, 0x%x, 0x%x)\n",
	  __func__, data[0], data[1], data[2], data[3]);
	mcu_data->gesture_motion_param = val;

	cwmcu_powermode_switch(mcu_data, 1);
	for (i = 0; i < GESTURE_MOTION_UPDATE_ATTRIBUTE_LEN; i++) {
		CWMCU_i2c_write(mcu_data, GESTURE_MOTION_UPDATE_ATTRIBUTE, &data[i], 1, 1);
	}
	cwmcu_powermode_switch(mcu_data, 0);

	return count;
}

static ssize_t get_gesture_motion(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);
	u8 data[GESTURE_MOTION_UPDATE_ATTRIBUTE_LEN] = {0};

	if (CWMCU_i2c_read_power(mcu_data, GESTURE_MOTION_UPDATE_ATTRIBUTE, data, 4) >= 0) {
		I("%s: setting parameter = (0x%x, 0x%x, 0x%x, 0x%x)\n"
		  , __func__, data[0], data[1], data[2], data[3]);
		return snprintf(buf, PAGE_SIZE, "0x%08X\n",
				cpu_to_le32p((__u32 *)&data));
	}
	return snprintf(buf, PAGE_SIZE, "0x%08X\n", 0xFFFFFFFF);
}

static bool is_htc_dbg_flag_set(void)
{
    //check if SSD ramdump flag is set
#if 0
    if (get_radio_flag() & 0x8) {
        I("%s: true\n", __func__);
        return true;
    }
    else {
        I("%s: false\n", __func__);
        return false;
    }
#endif
	return false;
}

static ssize_t dbg_flag_show(struct device *dev,
                                 struct device_attribute *attr,
                                 char *buf)
{
    int dbg_flag_set;

    if (is_htc_dbg_flag_set())
        dbg_flag_set = 1;
    else
        dbg_flag_set = 0;

    return snprintf(buf, PAGE_SIZE, "%d\n", dbg_flag_set);
}

static ssize_t crash_count_show(struct device *dev,
                                 struct device_attribute *attr,
                                 char *buf)
{
	struct cwmcu_data *mcu_data = dev_get_drvdata(dev);

	int i;
	int rc;
	size_t buf_remaining = PAGE_SIZE;

	for (i = 0; i < CRASH_REASON_NUM; i++) {
		rc = scnprintf(buf, buf_remaining, "%d%c",
				mcu_data->crash_count[i],
				(i == CRASH_REASON_NUM - 1) ? '\n' : ' ');
		buf += rc;
		buf_remaining -= rc;
	}
	return PAGE_SIZE - buf_remaining;
}

#ifdef SHUB_LOGGING_SUPPORT
static ssize_t log_mask_show(struct device *dev, struct device_attribute *attr,char *buf);
static ssize_t log_mask_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
static ssize_t log_level_show(struct device *dev, struct device_attribute *attr,char *buf);
static ssize_t log_level_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);
#endif //SHUB_LOGGING_SUPPORT
static ssize_t sensor_placement_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count);

static struct device_attribute attributes[] = {

	__ATTR(enable, 0664, active_show, active_set),
	__ATTR(batch_enable, 0664, batch_show, batch_set),
	__ATTR(delay_ms, 0664, interval_show, interval_set),
	__ATTR(flush, 0664, flush_show, flush_set),
	__ATTR(calibrator_en, 0220, NULL, set_calibrator_en),
	__ATTR(calibrator_status_acc, 0440, show_calibrator_status_acc, NULL),
	__ATTR(calibrator_status_mag, 0440, show_calibrator_status_mag, NULL),
	__ATTR(calibrator_status_gyro, 0440, show_calibrator_status_gyro, NULL),
	__ATTR(calibrator_data_acc, 0664, get_k_value_acc_f, set_k_value_acc_f),
	__ATTR(calibrator_data_acc_rl, 0440, get_k_value_acc_rl_f, NULL),
	__ATTR(ap_calibrator_data_acc_rl, 0440, ap_get_k_value_acc_rl_f, NULL),
	__ATTR(calibrator_data_mag, 0664, get_k_value_mag_f, set_k_value_mag_f),
	__ATTR(calibrator_data_gyro, 0664, get_k_value_gyro_f, set_k_value_gyro_f),
	__ATTR(calibrator_data_light, 0664, get_k_value_light_f, set_k_value_light_f),
	__ATTR(calibrator_data_proximity, 0664, get_k_value_proximity_f, set_k_value_proximity_f),
	__ATTR(calibrator_data_barometer, 0664, get_k_value_barometer_f, set_k_value_barometer_f),
	__ATTR(gesture_motion, 0660, get_gesture_motion, set_gesture_motion),
	__ATTR(data_barometer, 0440, get_barometer, NULL),
	__ATTR(data_proximity, 0444, get_proximity, NULL),
	__ATTR(data_acc_polling, 0440, get_acceleration_polling, NULL),
	__ATTR(data_mag_polling, 0440, get_magnetic_polling, NULL),
	__ATTR(data_gyro_polling, 0440, get_gyro_polling, NULL),
	__ATTR(data_proximity_polling, 0444, get_proximity_polling, NULL),
	__ATTR(data_light_polling, 0440, get_light_polling, NULL),
	__ATTR(ls_mechanism, 0444, get_ls_mechanism, NULL),
	__ATTR(sensor_hub_rdata, 0220, NULL, read_mcu_data),
	__ATTR(ps_canc, 0664, get_ps_canc, set_ps_canc),
	__ATTR(data_light_kadc, 0440, get_light_kadc, NULL),
	__ATTR(sensor_hw_id, 0440, sensor_hw_id_show, NULL),
	__ATTR(firmware_version, 0440, get_firmware_version, NULL),
	__ATTR(firmware_info, 0440, get_firmware_info, NULL),
	__ATTR(trigger_crash, 0220, NULL, trigger_mcu_crash),
	__ATTR(sensors_self_test, 0660, sensors_self_test_show, sensors_self_test_store),
	__ATTR(mcu_wakeup, 0220, NULL, trigger_mcu_wakeup),
#ifdef SHUB_LOGGING_SUPPORT
	__ATTR(mcu_log_mask, 0660, log_mask_show, log_mask_store),
	__ATTR(mcu_log_level, 0660, log_level_show, log_level_store),
#endif //SHUB_LOGGING_SUPPORT
	__ATTR(dbg_flag, 0440, dbg_flag_show, NULL),
	__ATTR(sensor_placement, 0220, NULL, sensor_placement_store),
	__ATTR(vibrate_ms, 0220, NULL, set_vibrate_ms),
	__ATTR(crash_count, 0440, crash_count_show, NULL),
};


static int create_sysfs_interfaces(struct cwmcu_data *mcu_data)
{
	int i;
	int res;
	struct class *optical_class = NULL;
	struct device *proximity_dev = NULL;

	mcu_data->bma250_class = class_create(THIS_MODULE, "bma250_powerkey");
	if (IS_ERR(mcu_data->bma250_class)) {
		res = PTR_ERR(mcu_data->bma250_class);
		mcu_data->bma250_class = NULL;
		E("%s: could not allocate bma250_powerkey_class, res = %d\n",
		  __func__, res);
		goto error_powerkey_class;
	}

	mcu_data->bma250_dev = device_create(mcu_data->bma250_class,
					    NULL, 0, "%s", "bma250");
	if (IS_ERR(mcu_data->bma250_dev)) {
		res = PTR_ERR(mcu_data->bma250_dev);
		goto err_bma250_device_create;
	}

	dev_set_drvdata(mcu_data->bma250_dev, mcu_data);

	res = device_create_file(mcu_data->bma250_dev,
				 &dev_attr_clear_powerkey_flag);
	if (res) {
		E("%s, create bma250_device_create_file fail!\n", __func__);
		goto err_create_bma250_device_file;
	}


	mcu_data->gs_cali_class = class_create(THIS_MODULE, "htc_g_sensor");
	if (IS_ERR(mcu_data->gs_cali_class)) {
		res = PTR_ERR(mcu_data->gs_cali_class);
		mcu_data->gs_cali_class = NULL;
		E("%s: could not allocate htc_g_sensor, res = %d\n",
		  __func__, res);
		goto error_gs_cali_class;
	}

	mcu_data->gs_cali_dev = device_create(mcu_data->gs_cali_class,
					    NULL, 0, "%s", "g_sensor");
	if (IS_ERR(mcu_data->gs_cali_dev)) {
		res = PTR_ERR(mcu_data->gs_cali_dev);
		E("%s: gs_cali_dev PTR_ERR, res = %d\n", __func__, res);
		goto err_gs_cali_device_create;
	}

	dev_set_drvdata(mcu_data->gs_cali_dev, mcu_data);

	res = device_create_file(mcu_data->gs_cali_dev,
				 &dev_attr_g_sensor_user_offset);
	if (res) {
		E("%s, create g_sensor_user_offset fail!\n", __func__);
		goto err_create_g_sensor_user_offset_device_file;
	}


	optical_class = class_create(THIS_MODULE, "optical_sensors");
	if (IS_ERR(optical_class)) {
		res = PTR_ERR(optical_class);
		optical_class = NULL;
		E("%s: could not allocate optical_class, res = %d\n", __func__,
		  res);
		goto error_optical_class_create;
	}

	proximity_dev = device_create(optical_class, NULL, 0, "%s",
				      "proximity");
	res = device_create_file(proximity_dev, &dev_attr_p_status);
	if (res) {
		E("%s, create proximty_device_create_file fail!\n", __func__);
		goto err_create_proximty_device_file;
	}

        res = device_create_file(proximity_dev, &dev_attr_ps_adc);
        if (res) {
                E("%s, create proximty_device_create_file fail!\n", __func__);
                goto err_create_proximty_device_file;
        }

	mcu_data->sensor_class = class_create(THIS_MODULE, "htc_sensorhub");
	if (IS_ERR(mcu_data->sensor_class)) {
		res = PTR_ERR(mcu_data->sensor_class);
		goto err_class_create;
	}

	mcu_data->sensor_dev = device_create(mcu_data->sensor_class, NULL, 0,
					     "%s", "sensor_hub");
	if (IS_ERR(mcu_data->sensor_dev)) {
		res = PTR_ERR(mcu_data->sensor_dev);
		goto err_device_create;
	}

	dev_set_drvdata(mcu_data->sensor_dev, mcu_data);

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(mcu_data->sensor_dev, attributes + i))
			goto error;

	res = sysfs_create_link(&mcu_data->sensor_dev->kobj,
				&mcu_data->indio_dev->dev.kobj, "iio");
	if (res < 0)
		goto error;

	return 0;

error:
	while (--i >= 0)
		device_remove_file(mcu_data->sensor_dev, attributes + i);
	put_device(mcu_data->sensor_dev);
	device_unregister(mcu_data->sensor_dev);
err_device_create:
	class_destroy(mcu_data->sensor_class);
err_class_create:
	device_remove_file(proximity_dev, &dev_attr_p_status);
err_create_proximty_device_file:
	class_destroy(optical_class);
error_optical_class_create:
	device_remove_file(mcu_data->gs_cali_dev,
			   &dev_attr_g_sensor_user_offset);
err_create_g_sensor_user_offset_device_file:
	put_device(mcu_data->gs_cali_dev);
	device_unregister(mcu_data->gs_cali_dev);
err_gs_cali_device_create:
	class_destroy(mcu_data->gs_cali_class);
error_gs_cali_class:
	device_remove_file(mcu_data->bma250_dev, &dev_attr_clear_powerkey_flag);
err_create_bma250_device_file:
	put_device(mcu_data->bma250_dev);
	device_unregister(mcu_data->bma250_dev);
err_bma250_device_create:
	class_destroy(mcu_data->bma250_class);
error_powerkey_class:
	dev_err(mcu_data->mcu_dev, "%s:Unable to create interface\n",
		__func__);

	return res;
}

static void destroy_sysfs_interfaces(struct cwmcu_data *mcu_data)
{
	int i;

	sysfs_remove_link(&mcu_data->sensor_dev->kobj, "iio");
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(mcu_data->sensor_dev, attributes + i);
	put_device(mcu_data->sensor_dev);
	device_unregister(mcu_data->sensor_dev);
	class_destroy(mcu_data->sensor_class);
	device_remove_file(mcu_data->gs_cali_dev,
			   &dev_attr_g_sensor_user_offset);
	put_device(mcu_data->gs_cali_dev);
	device_unregister(mcu_data->gs_cali_dev);
	class_destroy(mcu_data->gs_cali_class);
	device_remove_file(mcu_data->bma250_dev, &dev_attr_clear_powerkey_flag);
	put_device(mcu_data->bma250_dev);
	device_unregister(mcu_data->bma250_dev);
	class_destroy(mcu_data->bma250_class);
}

static void cwmcu_remove_trigger(struct iio_dev *indio_dev)
{
	struct cwmcu_data *mcu_data = iio_priv(indio_dev);

	iio_trigger_unregister(mcu_data->trig);
	iio_trigger_free(mcu_data->trig);
	iio_dealloc_pollfunc(indio_dev->pollfunc);
}
static void cwmcu_remove_buffer(struct iio_dev *indio_dev)
{
	iio_buffer_unregister(indio_dev);
	iio_kfifo_free(indio_dev->buffer);
}

static void cwmcu_one_shot(struct work_struct *work)
{
	struct cwmcu_data *mcu_data = container_of((struct work_struct *)work,
			struct cwmcu_data, one_shot_work);
	u8 clear_intr;
	I("w_activated_i2c is %d,w_re_init is %d, w_kick_start_mcu is %d, w_mcu_state_change is %d, w_flush_fifo is %d, w_clear_fifo is %d, w_batch_read is %d\n",
	mcu_data->w_activated_i2c,mcu_data->w_re_init,mcu_data->w_kick_start_mcu,mcu_data->w_mcu_state_change,mcu_data->w_flush_fifo,mcu_data->w_clear_fifo,mcu_data->w_batch_read);
	wake_lock_timeout(&mcu_data->one_shot_wake_lock,
			  msecs_to_jiffies(60000));

	if (mcu_data->w_activated_i2c == true) {
		mcu_data->w_activated_i2c = false;

		mutex_lock(&s_activated_i2c_lock);
		if (retry_exhausted(mcu_data) &&
		    time_after(jiffies, mcu_data->i2c_jiffies +
					REACTIVATE_PERIOD)) {

			// if MCU is in DLOAD or FLASH state, it will eventually back to SHUB state, so don't need to reset_hub here
			if (MCU_IN_DLOAD() || MCU_IN_BOOTLOADER()) {
				E("%s[%d]: skip reset_hub, s_mcu_state:%x\n", __func__, __LINE__, s_mcu_state);
			} else {
				bool reset_done;
				reset_done = reset_hub(mcu_data, false);
				E("%s[%d]: reset_hub, reset_done:%d\n", __func__, __LINE__, reset_done);
			}
		}

		if (retry_exhausted(mcu_data)) {
			I("%s: i2c_total_retry = %d, i2c_latch_retry = %d\n",
			  __func__, mcu_data->i2c_total_retry,
			  mcu_data->i2c_latch_retry);
		}
		else {
			/* record the failure */
			mcu_data->i2c_total_retry++;
			mcu_data->i2c_jiffies = jiffies;
		}

		mutex_unlock(&s_activated_i2c_lock);
		D(
		  "%s--: mcu_data->i2c_total_retry = %d, "
		  "mcu_data->i2c_latch_retry = %d\n", __func__,
		  mcu_data->i2c_total_retry, mcu_data->i2c_latch_retry);
	}

	if (mcu_data->w_re_init == true) {
		mcu_data->w_re_init = false;

		cwmcu_powermode_switch(mcu_data, 1);

		cwmcu_sensor_placement(mcu_data);
		cwmcu_set_sensor_kvalue(mcu_data);
		cwmcu_set_touch_solution(mcu_data);
		cwmcu_restore_status(mcu_data);

		mcu_time_sync();

#ifdef SHUB_DLOAD_SUPPORT
		mcu_enable_disable_dload_mode(mcu_data->dload_mode_enabled);
#endif //SHUB_DLOAD_SUPPORT

#ifdef SHUB_LOGGING_SUPPORT
		mcu_set_log_mask(mcu_data->mcu_log_mask);
		mcu_set_log_level(mcu_data->mcu_log_level);
#endif //SHUB_LOGGING_SUPPORT

#ifdef CONFIG_FB
		mcu_set_display_state(mcu_data->is_display_on);
#endif //CONFIG_FB

		//mcu is initialized completely here, trigger mcu state transit to MCU_STATE_SHUB_RUN
		mcu_data->mcu_sensor_ready = true;
		mcu_data->w_mcu_state_change = true;
		queue_work(mcu_data->mcu_wq, &mcu_data->one_shot_work);

		cwmcu_powermode_switch(mcu_data, 0);
	}

	if (mcu_data->w_kick_start_mcu == true) {
		mcu_data->w_kick_start_mcu = false;
		mutex_lock(&s_activated_i2c_lock);
		reset_hub(mcu_data, true);
		mutex_unlock(&s_activated_i2c_lock);

		enable_irq(mcu_data->IRQ);
	}

	if (mcu_data->w_mcu_state_change == true) {
		mcu_data->w_mcu_state_change = false;
		do_mcu_state_transition(mcu_data);
	}

	if (mcu_data->w_flush_fifo == true) {
		int i, j;
		mcu_data->w_flush_fifo = false;
		I("w_flush_fifo is true ++\n");

		cwmcu_powermode_switch(mcu_data, 1);
		for (j = 0; j < 2; j++) {
			cwmcu_batch_fifo_read(mcu_data, j);
		}
		cwmcu_powermode_switch(mcu_data, 0);

		for (i = 0; i < num_sensors; i++) {
			while (mcu_data->pending_flush[i] > 0) {
				cwmcu_send_flush(mcu_data, i);
				mcu_data->pending_flush[i]--;
			}
		}
		I("w_flush_fifo is true --\n");
	}

	mutex_lock(&mcu_data->mutex_lock);
	if (mcu_data->w_clear_fifo == true) {
		int j;
		I("w_clear_fifo ++ \n");
		mcu_data->w_clear_fifo = false;
		mutex_unlock(&mcu_data->mutex_lock);

		cwmcu_powermode_switch(mcu_data, 1);

		for (j = 0; j < 2; j++)
			cwmcu_batch_fifo_read(mcu_data, j);

		cwmcu_powermode_switch(mcu_data, 0);

		mutex_lock(&mcu_data->mutex_lock);
		if (!mcu_data->w_clear_fifo)
			mcu_data->w_clear_fifo_running = false;
		I("w_clear_fifo -- \n");
	} else
		mcu_data->w_clear_fifo_running = false;
	mutex_unlock(&mcu_data->mutex_lock);

#ifdef CONFIG_AK8789_HALLSENSOR
	if (mcu_data->w_hall_inform_mcu == true) {
		int i, rc;

		mcu_data->w_hall_inform_mcu = false;

		for (i = 0; i < HALL_RETRY_TIMES; i++) {
			rc = CWMCU_i2c_write_power(s_mcu_data,
					   CW_I2C_REG_DOTVIEW_STATUS,
					   &s_mcu_data->dotview_st, 1);
			if (rc) {
				E(
				  "%s: CWMCU_i2c_write HALL fails, rc = %d, "
				  "i = %d\n", __func__, rc, i);
				msleep(10);
			} else
				break;
		}
	}
#endif

	if (mcu_data->w_batch_read == true) {
		I("w_batch_read ++ \n");
		mcu_data->w_batch_read = false;

		wake_lock(&mcu_data->batch_read_wake_lock);
		clear_intr = CW_MCU_INT_BIT_BATCH_INT_MASK;
		CWMCU_i2c_write_power(mcu_data, CWSTM32_BATCH_MODE_COMMAND, &clear_intr, 1);
		cwmcu_powermode_switch(mcu_data, 1);
		cwmcu_batch_read(mcu_data);
		cwmcu_powermode_switch(mcu_data, 0);

		wake_unlock(&mcu_data->batch_read_wake_lock);
		I("w_batch_read -- \n");
	}

	wake_unlock(&mcu_data->one_shot_wake_lock);
}


static void cwmcu_work_report(struct work_struct *work)
{
	struct cwmcu_data *mcu_data = container_of((struct delayed_work *)work,
			struct cwmcu_data, work);

	if (atomic_read(&mcu_data->pseudo_irq_enable)) {
		unsigned long jiff;

		jiff = msecs_to_jiffies(atomic_read(&mcu_data->delay));
		if (!jiff)
			jiff = 1;
		mcu_data->enqueue_iio_irq_work_jiffies = jiffies;
		irq_work_queue(&mcu_data->iio_irq_work);
		queue_delayed_work(mcu_data->mcu_wq, &mcu_data->work, jiff);
	}
}

#if defined(SHUB_DLOAD_SUPPORT) || defined(SHUB_FIRMWARE_UPDATE_SUPPORT)
static int mcu_i2c_rx(const struct cwmcu_bus_client *client, u8 *buf, int len)
{
    int ret;
    int i;

    if (!client) return -ENXIO;

    mutex_lock(&s_activated_i2c_lock);

    for (i = 0; i < RETRY_TIMES; i++) {
        D("%s(%d) len:%d\n", __func__, __LINE__, len);
        ret = mcu_do_rx(client, buf, len);
        if (unlikely(ret < 0)) {
            E("%s(%d) fail ret:%d\n", __func__, __LINE__, ret);
            msleep(1);
        }
        else {
            break;
        }
    }

    mutex_unlock(&s_activated_i2c_lock);
    return ret;
}

static int mcu_i2c_tx(const struct cwmcu_bus_client *client, u8 *buf, int len)
{
    int ret;
    int i;

    if (!client) return -ENXIO;

    mutex_lock(&s_activated_i2c_lock);

    for (i = 0; i < RETRY_TIMES; i++) {
        D("%s(%d) len:%d\n", __func__, __LINE__, len);
        ret = mcu_do_tx(client, buf, len);
        if (unlikely(ret < 0)) {
            E("%s(%d) fail ret:%d\n", __func__, __LINE__, ret);
            msleep(1);
        }
        else {
            break;
        }
    }

    mutex_unlock(&s_activated_i2c_lock);
    return ret;
}
#endif

#ifdef SHUB_DLOAD_SUPPORT
#define MCU_I2C_TX_CMD_LEN_MAX     32

#ifdef CONFIG_SENSORS_I2C_BUS
static int mcu_i2c_tx_cmd(const struct cwmcu_bus_client *client, u8 cmd, u8 *buf, int len)
{
    int ret, i = 0;
    u8 i2c_data[MCU_I2C_TX_CMD_LEN_MAX+3] = {0};

    D("%s: addr:0x%x cmd:0x%x len:%d\n", __func__, client->i2c_client->addr, cmd, len);

    if (!client) return -ENXIO;

    if (unlikely(len > MCU_I2C_TX_CMD_LEN_MAX)) {
        E("%s[%d]: invalid len:%d !!\n", __func__, __LINE__, len);
        return -EINVAL;
    }

    i2c_data[i++] = cmd;

    memcpy(&(i2c_data[i]), buf, len);

    ret = mcu_i2c_tx(client, i2c_data, len+i);
    if (unlikely(ret < 0)) {
        E("%s[%d]: addr:%x cmd:%x fail ret:%d!!\n", __func__, __LINE__, client->i2c_client->addr, cmd, ret);
        return ret;
    }

    return ret;
}

static int mcu_i2c_rx_cmd(const struct cwmcu_bus_client *client, u8 cmd, u8 *buf, int len)
{
    int ret, i = 0;
    u8 i2c_data[3];

    D("%s: addr:0x%x cmd:0x%x len:%d\n", __func__, client->i2c_client->addr, cmd, len);

    if (!client) return -ENXIO;

    i2c_data[i++] = cmd;

    ret = mcu_i2c_tx(client, i2c_data, i);
    if (unlikely(ret < 0)) {
        E("%s[%d]: addr:%x cmd:%x fail ret:%d!!\n", __func__, __LINE__, client->i2c_client->addr, cmd, ret);
        return ret;
    }

    ret = mcu_i2c_rx(client, buf, len);

    return ret;
}
#endif

#endif //SHUB_DLOAD_SUPPORT

#ifdef SHUB_FIRMWARE_UPDATE_SUPPORT

/*--------------------------------------
 * Internal Data Structure Definition
 *-------------------------------------*/

//STM383
#define STM32MCUF383_FLASH_START_ADDR       (0x08000000)
#define STM32MCUF383_FLASH_END_ADDR         (0x0803FFFF)
#define STM32MCUF383_PID                    (0x0432)

//STM401
#define STM32MCUF401_FLASH_START_ADDR       (0x08000000)
#define STM32MCUF401_FLASH_END_ADDR         (0x0803FFFF)
#define STM32MCUF401_PID                    (0x0423)

//STM411
#define STM32MCUF411_FLASH_START_ADDR       (0x08000000)
#define STM32MCUF411_FLASH_END_ADDR         (0x0807FFFF)
#define STM32MCUF411_PID                    (0x0431)

//HTC_PARAM
#define STM32MCUF383_HTC_PARAM_START_ADDR   (0x2000ff00)
#define STM32MCUF401_HTC_PARAM_START_ADDR   (0x2000ff00)
#define STM32MCUF411_HTC_PARAM_START_ADDR   (0x2001ff00)
#define HTC_PARAM_SIZE                      (0x00000100)

//MCU BL response code
#define MCU_BL_RESP_ACK  0x79
#define MCU_BL_RESP_NACK 0x1F
#define MCU_BL_RESP_BUSY 0x76

#define MCU_I2C_FLASH_NAME "i2c-mcu-flash"
#define MCU_FW_UPDATE_I2C_LEN_LIMIT 256
#define MCU_FW_UPDATE_TIMEOUT_SEC   300
#define MCU_FW_CHECKSUM_TIMEOUT_MSEC   (10000)


static struct cwmcu_bus_client *mcu_fw_flash_bus_client = NULL;
static u8 stm32mcu_bl_ver = 0x11;
static u16 stm32mcu_pid = STM32MCUF411_PID;
static u32 stm32mcu_flash_start_addr = STM32MCUF411_FLASH_START_ADDR;
static u32 stm32mcu_flash_end_addr = STM32MCUF411_FLASH_END_ADDR;
static u32 stm32mcu_htc_param_start_addr = STM32MCUF411_HTC_PARAM_START_ADDR;
static u8 stm32mcu_flash_readout_protect_enable = 0;
static u8 stm32mcu_flash_support_crc_checksum = 0;

#define FW_UPDATE_STATUS_INIT   0
#define FW_UPDATE_ONGOING       1
#define FW_UPDATE_DONE          2
static unsigned char  mcu_fw_flash_status = FW_UPDATE_DONE;
static unsigned char  mcu_fw_flash_progress = 0;

static unsigned long  mcu_fw_flash_start_jiffies;
static uint32_t  mcu_fw_img_size = 0;
static uint32_t  mcu_fw_img_crc_checksum = 0;

/*--------------------------------------
 * Internal Function Implementation
 *-------------------------------------*/
static int mcu_bl_wait_for_ack(int wait_ms)
{
    u8 i2c_data[1] = {0};
    unsigned long  start_jiffies = jiffies;
    int count = 0;
    unsigned int elapsed_ms;

    D("%s\n", __func__);
#ifdef CONFIG_SENSORS_SPI_BUS
	i2c_data[0] = 0;
	mcu_i2c_tx(mcu_fw_flash_bus_client, i2c_data, 1);
#endif
    do {
        mcu_i2c_rx(mcu_fw_flash_bus_client, i2c_data, 1);
        D("%s ack:0x%x\n", __func__, i2c_data[0]);
        if (i2c_data[0] == MCU_BL_RESP_ACK || i2c_data[0] == MCU_BL_RESP_NACK) {
#ifdef CONFIG_SENSORS_SPI_BUS
			i2c_data[0] = 0x79;
			mcu_i2c_tx(mcu_fw_flash_bus_client, i2c_data, 1);
#endif
            break;
        }

        if (jiffies_to_msecs(jiffies - start_jiffies) > wait_ms)
            break;

        //msleep(10);

        count++;
    } while(1);

    elapsed_ms = jiffies_to_msecs(jiffies - start_jiffies);
    if (elapsed_ms > 50) {
        I("%s done, count:%d elapsed_ms:%u ret:0x%x\n", __func__, count, elapsed_ms, i2c_data[0]);
    }

    return i2c_data[0];
}

#ifdef CONFIG_SENSORS_SPI_BUS
static int mcu_bl_synchronization(void)
{
    u8 i2c_data[1] = {0};
	int rc = 0;
	int tx_bytes;

    I("%s\n", __func__);

	i2c_data[0] = 0x5A;
	tx_bytes = 1;
	rc = mcu_i2c_tx(mcu_fw_flash_bus_client, i2c_data, tx_bytes);
	if (rc != tx_bytes) {
		E("%s[%d]: Failed to write 0x%02x, rc = %d\n", __func__, __LINE__, i2c_data[0], rc);
		return rc;
	}

	rc = mcu_bl_wait_for_ack(2000); //mog 500
	if (rc != MCU_BL_RESP_ACK) {
		E("%s[%d]: FW wait ACK fail, rc = 0x%x\n", __func__, __LINE__, rc);
		return -1;
	}

    return 0;
}
#endif

static int mcu_bl_get_version(void)
{
    u8  i2c_data[10];
    int rc = 0;
    int id_num_of_byte;
    int tx_bytes;
	int i;

//--------------------------------------------------------------------------
    I("%s[%d]: send GV cmd\n", __func__, __LINE__);
	i = 0;
#ifdef CONFIG_SENSORS_SPI_BUS
	i2c_data[i++] = 0x5A;
#endif
    i2c_data[i++] = 0x01;
    i2c_data[i++] = 0xFE;
    tx_bytes = i;
    rc = mcu_i2c_tx(mcu_fw_flash_bus_client, i2c_data, tx_bytes);
    if (rc != tx_bytes) {
        E("%s[%d]: Failed to write 0x%02x 0x%02x, rc = %d\n", __func__, __LINE__, i2c_data[i-2], i2c_data[i-1], rc);
        return rc;
    }

    rc = mcu_bl_wait_for_ack(2000); //mog 500
    if (rc != MCU_BL_RESP_ACK) {
        E("%s[%d]: FW wait ACK fail, rc = 0x%x\n", __func__, __LINE__, rc);
        return -1;
    }
#ifdef CONFIG_SENSORS_SPI_BUS
	rc = mcu_i2c_rx(mcu_fw_flash_bus_client, i2c_data, 1);
#endif
    rc = mcu_i2c_rx(mcu_fw_flash_bus_client, i2c_data, 1);
    if (rc != 1) {
        E("%s: Failed to read ver, rc = %d\n", __func__, rc);
        return rc;
    }

    stm32mcu_bl_ver = i2c_data[0];
    I("%s[%d]: stm32mcu_bl_ver:0x%x\n", __func__, __LINE__, stm32mcu_bl_ver);

    rc = mcu_bl_wait_for_ack(2000); //mog 500
    if (rc != MCU_BL_RESP_ACK) {
        E("%s[%d]: FW wait ACK fail, rc = 0x%x\n", __func__, __LINE__, rc);
        return -1;
    }

//--------------------------------------------------------------------------
    I("%s[%d]: Get ID cmd\n", __func__, __LINE__);
	i = 0;
#ifdef CONFIG_SENSORS_SPI_BUS
	i2c_data[i++] = 0x5A;
#endif
    i2c_data[i++] = 0x02;
    i2c_data[i++] = 0xFD;
    tx_bytes = i;
    rc = mcu_i2c_tx(mcu_fw_flash_bus_client, i2c_data, tx_bytes);
    if (rc != tx_bytes) {
        E("%s[%d]: Failed to write 0x%02x 0x%02x, rc = %d\n", __func__, __LINE__, i2c_data[i-2], i2c_data[i-1], rc);
        return rc;
    }

    rc = mcu_bl_wait_for_ack(2000); //mog 500
    if (rc != MCU_BL_RESP_ACK) {
        E("%s[%d]: FW wait ACK fail, rc = 0x%x\n", __func__, __LINE__, rc);
        return -1;
    }
#ifdef CONFIG_SENSORS_SPI_BUS
	rc = mcu_i2c_rx(mcu_fw_flash_bus_client, i2c_data, 1);
#endif
    rc = mcu_i2c_rx(mcu_fw_flash_bus_client, i2c_data, 3);
    if (rc != 3) {
        E("%s: Failed to read pid, rc = %d\n", __func__, rc);
        return rc;
    }

    id_num_of_byte = i2c_data[0] + 1;
    I("%s[%d]: id_num_of_byte:%x\n", __func__, __LINE__, id_num_of_byte);

    stm32mcu_pid = (i2c_data[1] << 8) | i2c_data[2];
    I("%s[%d]: Product ID:0x%04x\n", __func__, __LINE__, stm32mcu_pid);

    rc = mcu_bl_wait_for_ack(2000); // mog 500
    if (rc != MCU_BL_RESP_ACK) {
        E("%s[%d]: FW wait ACK fail, rc = 0x%x\n", __func__, __LINE__, rc);
        return -1;
    }

    // Update some variable according to different MCU Product ID
    switch (stm32mcu_pid) {
    case STM32MCUF383_PID:
        I("%s[%d]: STM32MCUF383_PID\n", __func__, __LINE__);
        stm32mcu_flash_start_addr = STM32MCUF383_FLASH_START_ADDR;
        stm32mcu_flash_end_addr = STM32MCUF383_FLASH_END_ADDR;
        stm32mcu_htc_param_start_addr = STM32MCUF383_HTC_PARAM_START_ADDR;
        stm32mcu_flash_readout_protect_enable = 1;
        stm32mcu_flash_support_crc_checksum = 0;
        break;

    case STM32MCUF401_PID:
        I("%s[%d]: STM32MCUF401_PID\n", __func__, __LINE__);
        stm32mcu_flash_start_addr = STM32MCUF401_FLASH_START_ADDR;
        stm32mcu_flash_end_addr = STM32MCUF401_FLASH_END_ADDR;
        stm32mcu_htc_param_start_addr = STM32MCUF401_HTC_PARAM_START_ADDR;
        stm32mcu_flash_readout_protect_enable = 0;
        stm32mcu_flash_support_crc_checksum = 1;
        break;

    case STM32MCUF411_PID:
    default:
        I("%s[%d]: STM32MCUF411_PID\n", __func__, __LINE__);
        stm32mcu_flash_start_addr = STM32MCUF411_FLASH_START_ADDR;
        stm32mcu_flash_end_addr = STM32MCUF411_FLASH_END_ADDR;
        stm32mcu_htc_param_start_addr = STM32MCUF411_HTC_PARAM_START_ADDR;
        stm32mcu_flash_readout_protect_enable = 1;
        stm32mcu_flash_support_crc_checksum = 1;
        break;
    }

    return 0;
}

static int mcu_bl_read_protect_enable(void)
{
    uint8_t i2c_data[3];
    int rc;
    int tx_bytes;
	int i;

	i = 0;
#ifdef CONFIG_SENSORS_SPI_BUS
	i2c_data[i++] = 0x5A;
    i2c_data[i++] = 0x82;
    i2c_data[i++] = 0x7D;
#else
    //i2c_data[i++] = (stm32mcu_bl_ver == 0x11) ? 0x83 : 0x82; //mog
    //i2c_data[i++] = (stm32mcu_bl_ver == 0x11) ? 0x7C : 0x7D;

    i2c_data[i++] = (stm32mcu_bl_ver == 0x11) ? 0x83 : 0x83;
    i2c_data[i++] = (stm32mcu_bl_ver == 0x11) ? 0x7C : 0x7C;

#endif
    I("%s: Send command = 0x%02x 0x%02x \n", __func__, i2c_data[0], i2c_data[1]);
    tx_bytes = i;
    rc = mcu_i2c_tx(mcu_fw_flash_bus_client, i2c_data, tx_bytes);
    if (rc != tx_bytes) {
        E("%s[%d]: Failed to write 0x%02x 0x%02x, rc = %d\n", __func__, __LINE__, i2c_data[i-2], i2c_data[i-1], rc);
        return rc;
    }

    I("%s[%d]: wait ack1\n", __func__, __LINE__);
    rc = mcu_bl_wait_for_ack(2000); //mog 500
    if (rc != MCU_BL_RESP_ACK) {
        E("%s[%d]: FW wait ACK fail, rc = 0x%x\n", __func__, __LINE__, rc);
        return -1;
    }

    I("%s[%d]: wait ack2\n", __func__, __LINE__);
    rc = mcu_bl_wait_for_ack(20000); //mog 5000
    if (rc != MCU_BL_RESP_ACK) {
        E("%s[%d]: FW wait ACK fail, rc = 0x%x\n", __func__, __LINE__, rc);
        return -1;
    }

    msleep(2000);	// mog 500 //need to delay 500ms here, otherwise, sensorhub cannot work after read protect enabled

#ifdef CONFIG_SENSORS_SPI_BUS
	mcu_bl_synchronization();
#endif

    return 0;
}

static int mcu_bl_read_protect_disable(void)
{
    uint8_t i2c_data[3];
    int rc;
    int tx_bytes;
	int i;

	i = 0;
#ifdef CONFIG_SENSORS_SPI_BUS
	i2c_data[i++] = 0x5A;
    i2c_data[i++] = 0x92;
    i2c_data[i++] = 0x6D;
#else
    //i2c_data[i++] = (stm32mcu_bl_ver == 0x11) ? 0x93 : 0x92;
    //i2c_data[i++] = (stm32mcu_bl_ver == 0x11) ? 0x6C : 0x6D; //mog

    i2c_data[i++] = (stm32mcu_bl_ver == 0x11) ? 0x93 : 0x93;
    i2c_data[i++] = (stm32mcu_bl_ver == 0x11) ? 0x6C : 0x6C;
#endif

    I("%s: Send command = 0x%02x 0x%02x \n", __func__, i2c_data[0], i2c_data[1]);
    tx_bytes = i;
    rc = mcu_i2c_tx(mcu_fw_flash_bus_client, i2c_data, tx_bytes);
    if (rc != tx_bytes) {
        E("%s[%d]: Failed to write 0x%02x 0x%02x, rc = %d\n", __func__, __LINE__, i2c_data[i-2], i2c_data[i-1], rc);
        return rc;
    }

    I("%s[%d]: wait ack1\n", __func__, __LINE__);
    rc = mcu_bl_wait_for_ack(2000); //mog 500
    if (rc != MCU_BL_RESP_ACK) {
        E("%s[%d]: FW wait ACK fail, rc = 0x%x\n", __func__, __LINE__, rc);
        return -1;
    }

    I("%s[%d]: wait ack2\n", __func__, __LINE__);
    rc = mcu_bl_wait_for_ack(80000); //mog 20000
    if (rc != MCU_BL_RESP_ACK) {
        E("%s[%d]: FW wait ACK fail, rc = 0x%x\n", __func__, __LINE__, rc);
        return -1;
    }

    usleep_range(41*1000, 42*1000);

#ifdef CONFIG_SENSORS_SPI_BUS
		mcu_bl_synchronization();
#endif

    return 0;
}

#if 0
static int mcu_bl_erase_flash_mem(void)
{
    u8 i2c_data[128] = {0};
    int rc;
    int tx_bytes;
	int i;

    if (!mcu_fw_flash_bus_client)
        return -ENOENT;
	i = 0;
#ifdef CONFIG_SENSORS_SPI_BUS
	i2c_data[i++] = 0x5A;
    i2c_data[i++] = 0x44;
    i2c_data[i++] = 0xBB;
#else
    i2c_data[i++] = (stm32mcu_bl_ver == 0x11) ? 0x45 : 0x44;
    i2c_data[i++] = (stm32mcu_bl_ver == 0x11) ? 0xBA : 0xBB;
#endif

    I("%s: Send command = 0x%02x 0x%02x \n", __func__, i2c_data[0], i2c_data[1]);
    tx_bytes = i;
    rc = mcu_i2c_tx(mcu_fw_flash_bus_client, i2c_data, tx_bytes);
    if (rc != tx_bytes) {
        E("%s[%d]: Failed to write 0x%02x 0x%02x, rc = %d\n", __func__, __LINE__, i2c_data[0], i2c_data[1], rc);
        return rc;
    }

    rc = mcu_bl_wait_for_ack(500);
    if (rc != MCU_BL_RESP_ACK) {
        E("%s[%d]: FW wait ACK fail, rc = 0x%x\n", __func__, __LINE__, rc);
        return -1;
    }

    I("%s[%d]: start global mass erase\n", __func__, __LINE__);
    i2c_data[0] = 0xFF;
    i2c_data[1] = 0xFF;
    i2c_data[2] = 0;
    tx_bytes = 3;
    rc = mcu_i2c_tx(mcu_fw_flash_bus_client, i2c_data, tx_bytes);
    if (rc != tx_bytes) {
        E("%s[%d]: Failed to write 0x%02x 0x%02x, rc = %d\n", __func__, __LINE__, i2c_data[0], i2c_data[1], rc);
        return rc;
    }

    I("%s: wait for erase...\n", __func__);
    rc = mcu_bl_wait_for_ack(20000);
    if (rc != MCU_BL_RESP_ACK) {
        E("%s[%d]: FW wait ACK fail, rc = 0x%x\n", __func__, __LINE__, rc);
        return -1;
    }
    I("%s: erase done\n", __func__);

    return 0;
}
#endif

static int mcu_bl_write_memory(u32 start_address,
          u8 write_buf[],
          int numberofbyte)
{
    u8 i2c_data[MCU_FW_UPDATE_I2C_LEN_LIMIT+2] = {0};
    __be32 to_i2c_command;
    int data_len, checksum;
    int i;
    int rc;
    int tx_bytes;

    if (!mcu_fw_flash_bus_client)
        return -ENOENT;

    if (numberofbyte > 256) {
        E("%s[%d]: numberofbyte(%d) > 256\n", __func__, __LINE__, numberofbyte);
        return -EINVAL;
    }
	i = 0;
#ifdef CONFIG_SENSORS_SPI_BUS
	i2c_data[i++] = 0x5A;
    i2c_data[i++] = 0x31;
    i2c_data[i++] = 0xCE;
#else
    //i2c_data[i++] = (stm32mcu_bl_ver == 0x11) ? 0x32 : 0x31;
    //i2c_data[i++] = (stm32mcu_bl_ver == 0x11) ? 0xCD : 0xCE;

    i2c_data[i++] = (stm32mcu_bl_ver == 0x11) ? 0x32 : 0x32;
    i2c_data[i++] = (stm32mcu_bl_ver == 0x11) ? 0xCD : 0xCD;
#endif
    tx_bytes = i;
    rc = mcu_i2c_tx(mcu_fw_flash_bus_client, i2c_data, tx_bytes);
    if (rc != tx_bytes) {
        E("%s[%d]: Failed to write 0x%02x 0x%02x, rc = %d\n", __func__, __LINE__, i2c_data[i-2], i2c_data[i-1], rc);
        return rc;
    }

    rc = mcu_bl_wait_for_ack(2000); //mog 500
    if (rc != MCU_BL_RESP_ACK) {
        E("%s[%d]: FW wait ACK fail, rc = 0x%x\n", __func__, __LINE__, rc);
        return -1;
    }

    to_i2c_command = cpu_to_be32(start_address);
    memcpy(i2c_data, &to_i2c_command, sizeof(__be32));
    i2c_data[4] = i2c_data[0] ^ i2c_data[1] ^ i2c_data[2] ^ i2c_data[3];
    tx_bytes = 5;
    rc = mcu_i2c_tx(mcu_fw_flash_bus_client, i2c_data, tx_bytes);
    if (rc != tx_bytes) {
        E("%s[%d]: Failed to write 0x%02x 0x%02x, rc = %d\n", __func__, __LINE__, i2c_data[0], i2c_data[1], rc);
        return rc;
    }

    rc = mcu_bl_wait_for_ack(2000); //mog 500
    if (rc != MCU_BL_RESP_ACK) {
        E("%s[%d]: FW wait ACK fail, rc = 0x%x\n", __func__, __LINE__, rc);
        return -1;
    }


    checksum = 0x0;
    data_len = numberofbyte + 2;

    i2c_data[0] = numberofbyte - 1;

    for (i = 0; i < numberofbyte; i++)
        i2c_data[i+1] = write_buf[i];

    for (i = 0; i < (data_len - 1); i++)
        checksum ^= i2c_data[i];

    i2c_data[i] = checksum;
    rc = mcu_i2c_tx(mcu_fw_flash_bus_client, i2c_data, data_len);
    if (rc != data_len) {
        E("%s[%d]: Failed to write 0x%02x 0x%02x, rc = %d\n", __func__, __LINE__, i2c_data[0], i2c_data[1], rc);
        return rc;
    }

    rc = mcu_bl_wait_for_ack(12000); //mog 3000
    if (rc != MCU_BL_RESP_ACK) {
        E("%s[%d]: FW wait ACK fail, rc = 0x%x\n", __func__, __LINE__, rc);
        return -1;
    }

    return 0;
}

static int mcu_bl_erase_htc_param(void)
{
    int ret;
    u8 buffer[HTC_PARAM_SIZE] = {0};

    ret = mcu_bl_write_memory(stm32mcu_htc_param_start_addr, buffer, HTC_PARAM_SIZE);
    if (ret) {
        E("%s(%d): mcu_bl_write_memory fails,"
          "ret = %d\n", __func__, __LINE__, ret);
        return -EIO;
    }
    else {
        I("%s: done, addr:0x%08x, size:%u\n", __func__, stm32mcu_htc_param_start_addr, HTC_PARAM_SIZE);
    }

    return ret;
}

static void mcu_bl_enter_leave(uint8_t enter)
{
    //enter mcu bootloader mode
    mutex_lock(&s_activated_i2c_lock);

    //pull boot pin
    if (enter) {
        I("%s(%d) : enter\n", __func__, __LINE__);
        s_mcu_state = MCU_STATE_BOOTLOADER;
        mcu_chip_mode_set(s_mcu_data, MCU_CHIP_MODE_BOOTLOADER);
    }
    else {
        I("%s(%d) : leave\n", __func__, __LINE__);
        mcu_state_enter_unknown(s_mcu_data);
        mcu_chip_mode_set(s_mcu_data, MCU_CHIP_MODE_APPLICATION);
    }

    I("%s[%d]: gpio_chip_mode value = %d\n", __func__, __LINE__,
      gpio_get_value_cansleep(s_mcu_data->gpio_chip_mode));

    usleep_range(10000, 15000);

    //reset hub
    reset_hub(s_mcu_data, true);
    mutex_unlock(&s_activated_i2c_lock);
}

static uint32_t mcu_get_firmware_checksum(uint32_t firmware_size)
{
    int ret = 0;
    mcu_fw_checksum_t fw_checksum = {0, 0};
    unsigned long  start_jiffies = jiffies;

    I("%s[%d]: start get checksum firmware_size:%d\n", __func__, __LINE__, firmware_size);

    ret = CWMCU_i2c_write_block_power(s_mcu_data, CW_I2C_REG_FLASH_CHECKSUM, (u8*)&firmware_size, 4);
    if(ret < 0) {
        E("%s[%d]: checksum i2c failed.\n", __func__, __LINE__);
        return fw_checksum.check_sum;
    }

    while (fw_checksum.calculate_done != 1) {
        CWMCU_i2c_read_power(s_mcu_data, CW_I2C_REG_FLASH_CHECKSUM, (u8*)&fw_checksum, sizeof(fw_checksum));

        if (fw_checksum.calculate_done != 1) {
            if (jiffies_to_msecs(jiffies - start_jiffies) > MCU_FW_CHECKSUM_TIMEOUT_MSEC) {
                E("%s[%d]: wait checksum failed !!!\n", __func__, __LINE__);
                fw_checksum.check_sum = 0;
                break;
            }

            usleep_range(10000, 15000);
        }
    }

    I("%s[%d]: done get checksum:0x%X ...\n", __func__, __LINE__, fw_checksum.check_sum);

    return fw_checksum.check_sum;
}

/*--------------------------------------
 * Device attributes
 *-------------------------------------*/
static ssize_t fw_update_status_show(struct device *dev,
                                 struct device_attribute *attr,
                                 char *buf)
{
        return snprintf(buf, PAGE_SIZE, "%d\n", mcu_fw_flash_status);
}

static ssize_t fw_update_status_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf,
                                  size_t count)
{
    unsigned long tmp;
    int error;
    error = kstrtoul(buf, 10, &tmp);
    if (error) {
        E("%s[%d]: kstrtoul fails, error = %d\n", __func__, __LINE__, error);
    }
    else {
        mcu_fw_flash_status = tmp;
    }

    return count;
}

static ssize_t fw_update_progress_show(struct device *dev,
                                 struct device_attribute *attr,
                                 char *buf)
{
        return snprintf(buf, PAGE_SIZE, "%d\n", mcu_fw_flash_progress);
}

static ssize_t fw_update_progress_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf,
                                  size_t count)
{
    unsigned long tmp;
    int error;
    error = kstrtoul(buf, 10, &tmp);
    if (error) {
        E("%s[%d]: kstrtoul fails, error = %d\n", __func__, __LINE__, error);
    }
    else {
        mcu_fw_flash_progress = tmp;
    }

    return count;
}

static ssize_t fw_update_timeout_show(struct device *dev,
                                 struct device_attribute *attr,
                                 char *buf)
{
        return snprintf(buf, PAGE_SIZE, "%d\n", MCU_FW_UPDATE_TIMEOUT_SEC);
}

static DEVICE_ATTR(fw_update_status, S_IRUSR | S_IWUSR, fw_update_status_show, fw_update_status_store);
static DEVICE_ATTR(fw_update_timeout, S_IRUSR, fw_update_timeout_show, NULL);
static DEVICE_ATTR(fw_update_progress, S_IRUSR | S_IWUSR, fw_update_progress_show, fw_update_progress_store);

/*--------------------------------------
 * SHUB FLASH Misc Device fops
 *-------------------------------------*/
static int shub_fw_flash_open(struct inode *inode, struct file *file)
{
    mcu_fw_flash_start_jiffies = jiffies;
    I("%s(%d) done\n", __func__, __LINE__);
    return 0;
}

static int shub_fw_flash_release(struct inode *inode, struct file *file)
{
    I("%s(%d) done, elapsed_ms:%u\n", __func__, __LINE__, jiffies_to_msecs(jiffies - mcu_fw_flash_start_jiffies));
    return 0;
}

static ssize_t shub_fw_flash_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    return -ENODEV;
}

static ssize_t shub_fw_flash_write(struct file *file, const char __user *buf,
        size_t count, loff_t *ppos)
{
    size_t written = 0;
    size_t to_write;
    u8 write_buf[MCU_FW_UPDATE_I2C_LEN_LIMIT];
    int ret;
    unsigned long  start_jiffies = jiffies;
    u32 fw_flash_addr = stm32mcu_flash_start_addr + *ppos;

    if ( fw_flash_addr > stm32mcu_flash_end_addr) {
        E("%s(%d) invalid fw_flash_addr:0x%08x *ppos(0x%08x)\n", __func__, __LINE__, fw_flash_addr, (unsigned int)(*ppos));
        return written;
    }

    D("%s(%d): count(%u) *ppos(0x%08x)\n", __func__, __LINE__, (unsigned int)count, (unsigned int)(*ppos));

    while (count > 0) {
        to_write = min_t(size_t, count, (size_t)MCU_FW_UPDATE_I2C_LEN_LIMIT);
        to_write = min_t(size_t, to_write, (size_t)(stm32mcu_flash_end_addr-fw_flash_addr+1));
        if (to_write == 0) {
            E("%s(%d) reach mcu flash limit, fw_flash_addr:0x%08x\n", __func__, __LINE__, fw_flash_addr);
            break;
        }

        if (unlikely(copy_from_user(write_buf, buf, to_write))) {
            E("%s(%d) Unable to copy from user space !!\n", __func__, __LINE__);
            return -EFAULT;
        }

        ret = mcu_bl_write_memory(fw_flash_addr,
                     write_buf,
                     to_write);
        if (ret) {
            E("%s(%d): mcu_bl_write_memory fails,"
              "ret = %d\n", __func__, __LINE__, ret);
            return -EIO;
        }
        else {
            D("%s: mcu_bl_write_memory done, addr:0x%08x, size:%u\n", __func__, fw_flash_addr, (unsigned int)to_write);
        }

        buf += to_write;
        count -= to_write;
        written += to_write;
        *ppos += to_write;
        mcu_fw_img_size += to_write;
        fw_flash_addr = stm32mcu_flash_start_addr + *ppos;
    }

    I("%s(%d): return written(%u), elapsed_ms:%u\n", __func__, __LINE__, (unsigned int)written, jiffies_to_msecs(jiffies - start_jiffies));

    return written;
}

static long shub_fw_flash_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = 0;

    if (!s_mcu_data) {
        E("%s(%d): s_mcu_data is NULL, return\n", __func__, __LINE__);
        return -EINVAL;
    }

    switch (cmd) {

    case SHUB_FW_IOCTL_PRE_FLASH:
        {
            I("%s(%d): SHUB_FW_IOCTL_PRE_FLASH\n", __func__, __LINE__);

            //Initialize mcu_fw_img_size
            mcu_fw_img_size = 0;

            //enter mcu bootloader mode
            mcu_bl_enter_leave(1);
#ifdef CONFIG_SENSORS_SPI_BUS
			mcu_bl_synchronization();
#endif
            //get bl version
            ret = mcu_bl_get_version();
            if (ret < 0) {
                E("%s(%d): mcu_bl_get_version fails, ret = %d\n", __func__, __LINE__, ret);
                return ret;
            }

            //erase flash by enable then disable readout protect
            if (1) {
                ret = mcu_bl_read_protect_enable();
                if (ret < 0) {
                    I("%s(%d): mcu_bl_read_protect_enable fails, ret = %d\n", __func__, __LINE__, ret);
                }

                //readout unprotect
                ret = mcu_bl_read_protect_disable();
                if (ret < 0) {
                    E("%s(%d): read_protect_disable fails, ret = %d\n", __func__, __LINE__, ret);
                    return ret;
                }
            }

            //erase htc_param to prevent shub enter dload after leave bootloader
            mcu_bl_erase_htc_param();
        }
        break;

    case SHUB_FW_IOCTL_POST_FLASH:
        {
            unsigned long  start_jiffies;
            unsigned long timeout;

            I("%s(%d): SHUB_FW_IOCTL_POST_FLASH\n", __func__, __LINE__);


            if (stm32mcu_flash_support_crc_checksum) {
                //leave mcu bootloader mode
                mcu_bl_enter_leave(0);

                //wait mcu enter SHUB state to get checksum
                timeout = wait_for_completion_timeout(&s_mcu_enter_shub_run, (msecs_to_jiffies(3000)));
                if (timeout == 0) {
                    E("%s(%d): wait_for s_mcu_enter_shub_run timeout !!!\n", __func__, __LINE__);
                } else {
                    I("%s(%d): s_mcu_enter_shub_run completely\n", __func__, __LINE__);
                }

                if(!MCU_IN_BOOTLOADER()) {
                    //get checksum
                    mcu_fw_img_crc_checksum = mcu_get_firmware_checksum(mcu_fw_img_size);
                    I("%s(%d): firmware checksum:0x%X\n", __func__, __LINE__, mcu_fw_img_crc_checksum);
                }

                //enter mcu bootloader mode
                mcu_bl_enter_leave(1);
#ifdef CONFIG_SENSORS_SPI_BUS
				mcu_bl_synchronization();
#endif

                //erase htc_param to prevent shub enter dload after leave bootloader
                mcu_bl_erase_htc_param();
            }

            //readout protect
            if (stm32mcu_flash_readout_protect_enable) {
                I("%s(%d): enable read_protect\n", __func__, __LINE__);
                ret = mcu_bl_read_protect_enable();
                if (ret < 0) {
                    E("%s(%d): read_protect_disable fails, ret = %d\n", __func__, __LINE__, ret);
                }
            }

            //leave mcu bootloader mode
            mcu_bl_enter_leave(0);

            //wait mcu enter SHUB state
            I("%s(%d): After leave bootloader, wait mcu enter SHUB state\n", __func__, __LINE__);
            start_jiffies = jiffies;
            while (!MCU_IN_SHUB() && (jiffies_to_msecs(jiffies - start_jiffies) < 3000)) {
                msleep(5);
            }

            I("%s(%d): SHUB_FW_IOCTL_POST_FLASH done, s_mcu_state:%d\n", __func__, __LINE__, s_mcu_state);
        }
        break;

    case SHUB_FW_IOCTL_GET_FW_VERSION:
        {
            u8 fw_ver_from_mcu[6] = {0};
            mcu_fw_version_t fw_version;

            I("%s(%d): SHUB_FW_IOCTL_GET_FW_VERSION\n", __func__, __LINE__);

            ret = CWMCU_i2c_read(s_mcu_data, FIRMWARE_VERSION, fw_ver_from_mcu, sizeof(fw_ver_from_mcu));
            if (ret < 0) {
                E("%s(%d): Read FIRMWARE_VERSION err:%d\n", __func__, __LINE__, ret);
            }
            else {
                memset(&fw_version, 0, sizeof(fw_version));
                fw_version.arch = fw_ver_from_mcu[0];
                fw_version.sense= fw_ver_from_mcu[1];
                fw_version.cw_lib= fw_ver_from_mcu[2];
                fw_version.water= fw_ver_from_mcu[3];
                fw_version.active_engine= fw_ver_from_mcu[4];
                fw_version.project_mapping= fw_ver_from_mcu[5];

                ret = copy_to_user((void *)arg, &fw_version, sizeof(fw_version));
                if (ret) {
                    E("%s(%d): copy_to_user failed err:%d\n", __func__, __LINE__, ret);
                }
                else {
                    I("%s(%d): fw_version, arch:%d sense:%d cw_lib:%d water:%d active:%d project:%d\n", __func__, __LINE__,
                        fw_version.arch, fw_version.sense, fw_version.cw_lib, fw_version.water, fw_version.active_engine, fw_version.project_mapping );
                }
            }
        }
        break;

    case SHUB_FW_IOCTL_GET_FW_INFO:
        {
            u8 fw_info_from_mcu[6] = {0};
            mcu_fw_info_t fw_info;

            I("%s(%d): SHUB_FW_IOCTL_GET_FW_INFO\n", __func__, __LINE__);

            ret = CWMCU_i2c_read_power(s_mcu_data, FIRMWARE_INFO, fw_info_from_mcu, sizeof(fw_info_from_mcu));
            if (ret < 0) {
                E("%s(%d): Read FIRMWARE_INFO err:%d\n", __func__, __LINE__, ret);
            }
            else {
                memset(&fw_info, 0, sizeof(fw_info));
                fw_info.jenkins_num_hi = fw_info_from_mcu[0];
                fw_info.jenkins_num_lo = fw_info_from_mcu[1];
                fw_info.build_time_hh = fw_info_from_mcu[2];
                fw_info.build_time_mm = fw_info_from_mcu[3];
                fw_info.cw_branch = fw_info_from_mcu[4];
                fw_info.cw_mcu_type = fw_info_from_mcu[5];

                ret = copy_to_user((void *)arg, &fw_info, sizeof(fw_info));
                if (ret) {
                    E("%s(%d): copy_to_user failed err:%d\n", __func__, __LINE__, ret);
                }
                else {
                    I("%s(%d): Jenkins build number %u, Build time(hh:mm) %02u:%02u, CW branch %u, CW mcu type %u\n", __func__, __LINE__,
                        (fw_info_from_mcu[0] << 8) | fw_info_from_mcu[1],
                        fw_info_from_mcu[2], fw_info_from_mcu[3],
                        fw_info_from_mcu[4], fw_info_from_mcu[5]);
                }
            }
        }
        break;

    case SHUB_FW_IOCTL_START_FW_CHECKSUM:
        {
            if(!MCU_IN_BOOTLOADER()) {
                //get checksum
                mcu_fw_img_crc_checksum = mcu_get_firmware_checksum((uint32_t)arg);
                I("%s(%d): firmware checksum:0x%X\n", __func__, __LINE__, mcu_fw_img_crc_checksum);
            }
        }
        break;

    case SHUB_FW_IOCTL_GET_FW_CHECKSUM:
        {
            I("%s(%d): SHUB_FW_IOCTL_GET_FW_CHECKSUM\n", __func__, __LINE__);

            if (put_user(mcu_fw_img_crc_checksum, (uint32_t __user *)arg)) {
                E("%s(%d): put_user mcu_fw_img_crc_checksum failed\n", __func__, __LINE__);
                return -EFAULT;
            }
        }
        break;

    default:
        E("%s(%d): INVALID param:0x%x\n", __func__, __LINE__, cmd);
        ret = -EINVAL;
        break;
    }

    return ret;
}

static const struct file_operations shub_fw_flash_fops = {
    .owner = THIS_MODULE,
    .read  = shub_fw_flash_read,
    .write = shub_fw_flash_write,
    .open  = shub_fw_flash_open,
    .release = shub_fw_flash_release,
    .unlocked_ioctl = shub_fw_flash_ioctl,
    .compat_ioctl = shub_fw_flash_ioctl
};

static struct miscdevice shub_fw_flash_miscdev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = SHUB_FIRMWARE_UPDATE_DEVICE_NAME,
    .fops = &shub_fw_flash_fops
};

static int mcu_pinctrl_init(struct cwmcu_data *mcu_data)
{
    int retval;
    int ret;
    I("mcu_pinctrl_init");
    /* Get pinctrl if target uses pinctrl */

    mcu_data->pinctrl = devm_pinctrl_get(mcu_data->mcu_dev);
    if (IS_ERR_OR_NULL(mcu_data->pinctrl)) {
        E("Target does not use pinctrl\n");
        retval = PTR_ERR(mcu_data->pinctrl);
        mcu_data->pinctrl = NULL;
        return retval;
    }

    mcu_data->gpio_state_init = pinctrl_lookup_state(mcu_data->pinctrl, "mcu_gpio_init");
    if (IS_ERR_OR_NULL(mcu_data->gpio_state_init)) {
        E("Can not get ts default pinstate\n");
        retval = PTR_ERR(mcu_data->gpio_state_init);
        mcu_data->pinctrl = NULL;
        return retval;
    }

    ret = pinctrl_select_state(mcu_data->pinctrl, mcu_data->gpio_state_init);
    if (ret) {
        E("can not init gpio\n");
        return ret;
    }

    return 0;
}
#ifdef CONFIG_SENSORS_SPI_BUS
/*--------------------------------------
 * MCU fw flash SPI
 *-------------------------------------*/
static int mcu_fw_flash_spi_probe(struct spi_device *spi)
{
    int ret;
	I("%s\n", __func__);
#if 0 //TODO: Review this if NFC request
    if (!strcmp(htc_get_bootmode(), "offmode_charging")) {
        I("%s: disabled when offmode_charging\n", __func__);
        return -EIO;
    }
#endif
    mcu_fw_flash_bus_client = s_mcu_data->mcu_client;

    /* misc register */
    ret = misc_register(&shub_fw_flash_miscdev);
    if (ret < 0) {
        E("%s: failed to register misc device for '%s'!\n", __func__, shub_fw_flash_miscdev.name);
        return ret;
    }

    /* device attribute on sysfs */
    I("%s: create device attribute %s %s\n", __func__, dev_name(shub_fw_flash_miscdev.this_device), dev_attr_fw_update_status.attr.name);
    ret = device_create_file(shub_fw_flash_miscdev.this_device, &dev_attr_fw_update_status);
    if (ret < 0) {
        E("%s: cant create device attribute %s %s\n", __func__, dev_name(shub_fw_flash_miscdev.this_device), dev_attr_fw_update_status.attr.name);
        return ret;
    }

    I("%s: create device attribute %s %s\n", __func__, dev_name(shub_fw_flash_miscdev.this_device), dev_attr_fw_update_timeout.attr.name);
    ret = device_create_file(shub_fw_flash_miscdev.this_device, &dev_attr_fw_update_timeout);
    if (ret < 0) {
        E("%s: cant create device attribute %s %s\n", __func__, dev_name(shub_fw_flash_miscdev.this_device), dev_attr_fw_update_timeout.attr.name);
        return ret;
    }

    I("%s: create device attribute %s %s\n", __func__, dev_name(shub_fw_flash_miscdev.this_device), dev_attr_fw_update_progress.attr.name);
    ret = device_create_file(shub_fw_flash_miscdev.this_device, &dev_attr_fw_update_progress);
    if (ret < 0) {
        E("%s: cant create device attribute %s %s\n", __func__, dev_name(shub_fw_flash_miscdev.this_device), dev_attr_fw_update_progress.attr.name);
        return ret;
    }

    return ret;
}

static int mcu_fw_flash_spi_remove(struct spi_device *spi)
{
    I("%s\n", __func__);

    misc_deregister(&shub_fw_flash_miscdev);

    return 0;
}

#else
/*--------------------------------------
 * MCU fw flash I2C
 *-------------------------------------*/
static int mcu_fw_flash_i2c_probe(struct i2c_client *client,
                                  const struct i2c_device_id *id)
{
    int ret;
#if 0 //TODO: Review this if NFC request
    if (!strcmp(htc_get_bootmode(), "offmode_charging")) {
        I("%s: disabled when offmode_charging\n", __func__);
        return -EIO;
    }
#endif
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        E("%s: i2c_check_functionality error\n", __func__);
        return -EIO;
    }

	mcu_fw_flash_bus_client = kmalloc(sizeof(struct cwmcu_bus_client), GFP_KERNEL);
	if(!mcu_fw_flash_bus_client){
		E("%s: mcu_fw_flash_bus_client kmalloc failed\n", __func__);
		return -ENOMEM;
	}

    mcu_fw_flash_bus_client->i2c_client= client;
    I("%s: addr:%x\n", __func__, mcu_fw_flash_bus_client->i2c_client->addr);

    /* misc register */
    ret = misc_register(&shub_fw_flash_miscdev);
    if (ret < 0) {
        E("%s: failed to register misc device for '%s'!\n", __func__, shub_fw_flash_miscdev.name);
        return ret;
    }

    /* device attribute on sysfs */
    I("%s: create device attribute %s %s\n", __func__, dev_name(shub_fw_flash_miscdev.this_device), dev_attr_fw_update_status.attr.name);
    ret = device_create_file(shub_fw_flash_miscdev.this_device, &dev_attr_fw_update_status);
    if (ret < 0) {
        E("%s: cant create device attribute %s %s\n", __func__, dev_name(shub_fw_flash_miscdev.this_device), dev_attr_fw_update_status.attr.name);
        return ret;
    }

    I("%s: create device attribute %s %s\n", __func__, dev_name(shub_fw_flash_miscdev.this_device), dev_attr_fw_update_timeout.attr.name);
    ret = device_create_file(shub_fw_flash_miscdev.this_device, &dev_attr_fw_update_timeout);
    if (ret < 0) {
        E("%s: cant create device attribute %s %s\n", __func__, dev_name(shub_fw_flash_miscdev.this_device), dev_attr_fw_update_timeout.attr.name);
        return ret;
    }

    I("%s: create device attribute %s %s\n", __func__, dev_name(shub_fw_flash_miscdev.this_device), dev_attr_fw_update_progress.attr.name);
    ret = device_create_file(shub_fw_flash_miscdev.this_device, &dev_attr_fw_update_progress);
    if (ret < 0) {
        E("%s: cant create device attribute %s %s\n", __func__, dev_name(shub_fw_flash_miscdev.this_device), dev_attr_fw_update_progress.attr.name);
        return ret;
    }

    return ret;
}

static int mcu_fw_flash_i2c_remove(struct i2c_client *client)
{
    I("%s\n", __func__);

    misc_deregister(&shub_fw_flash_miscdev);

    return 0;
}

static const struct i2c_device_id mcu_fw_flash_id[] = {
    {MCU_I2C_FLASH_NAME, 0},
    { }
};

#ifdef CONFIG_OF
static struct of_device_id mcu_fw_flash_match_table[] = {
    {.compatible = "htc_mcu_flash" },
    {},
};
#else
#define mcu_fw_flash_match_table NULL
#endif

static struct i2c_driver mcu_fw_flash_i2c_driver = {
    .driver = {
        .name = MCU_I2C_FLASH_NAME,
        .owner = THIS_MODULE,
        .of_match_table = mcu_fw_flash_match_table,
    },
    .probe    = mcu_fw_flash_i2c_probe,
    .remove   = mcu_fw_flash_i2c_remove,
    .id_table = mcu_fw_flash_id,
};
#endif
#endif //SHUB_FIRMWARE_UPDATE_SUPPORT

#ifdef SHUB_LOGGING_SUPPORT
/*--------------------------------------
 * Internal Data Structure Definition
 *-------------------------------------*/

/*--------------------------------------
 * Internal Function Implementation
 *-------------------------------------*/

static void mcu_set_log_mask(u32 log_mask)
{
    int ret;

    s_mcu_data->mcu_log_mask = log_mask;
    if (MCU_IN_SHUB()) {
        ret = CWMCU_i2c_write_block_power(s_mcu_data, CW_I2C_REG_LOG_MASK, (u8*)&log_mask, sizeof(log_mask));
        I("%s(%d): log_mask:0x%x ret:%d\n", __func__, __LINE__, log_mask, ret);
    }
    else {
        I("%s(%d): MCU not in SHUB state\n", __func__, __LINE__);
    }
}

static int mcu_get_log_mask(u32 *log_mask_ptr)
{
    int ret = -1;

    if (MCU_IN_SHUB()) {
        ret = CWMCU_i2c_read_power(s_mcu_data, CW_I2C_REG_LOG_MASK, (u8*)log_mask_ptr, sizeof(u32));
        I("%s(%d): log_mask:0x%x ret:%d\n", __func__, __LINE__, *log_mask_ptr, ret);
        s_mcu_data->mcu_log_mask = *log_mask_ptr;
    }
    else {
        I("%s(%d): MCU not in SHUB state\n", __func__, __LINE__);
    }

    return ret;
}

static ssize_t log_mask_show(struct device *dev,
                                 struct device_attribute *attr,
                                 char *buf)
{
    uint32_t log_mask = 0;
    int ret;
    ret = mcu_get_log_mask(&log_mask);
    if (ret < 0)
        log_mask = 0;

    return snprintf(buf, PAGE_SIZE, "0x%x\n", (int)(log_mask));
}

static ssize_t log_mask_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf,
                                  size_t count)
{
    uint32_t log_mask = 0;
    int error;

    error = kstrtou32(buf, 16, &log_mask);
    if (error) {
        E("%s[%d]: kstrtoul fails, error = %d\n", __func__, __LINE__, error);
    }
    else {
        I("%s(%d): 0x%x\n", __func__, __LINE__, log_mask);
        mcu_set_log_mask(log_mask);
    }

    return count;
}

static void mcu_set_log_level(u32 log_level)
{
    int ret;

    s_mcu_data->mcu_log_level = log_level;
    if (MCU_IN_SHUB()) {
        ret = CWMCU_i2c_write_block_power(s_mcu_data, CW_I2C_REG_LOG_LEVEL, (u8*)&log_level, sizeof(log_level));
        I("%s(%d): log_level:0x%x ret:%d\n", __func__, __LINE__, log_level, ret);
    }
    else {
        I("%s(%d): MCU not in SHUB state\n", __func__, __LINE__);
    }
}

static int mcu_get_log_level(u32 *log_level_ptr)
{
    int ret = -1;

    if (MCU_IN_SHUB()) {
        ret = CWMCU_i2c_read_power(s_mcu_data, CW_I2C_REG_LOG_LEVEL, (u8*)log_level_ptr, sizeof(u32));
        I("%s(%d): log_level:0x%x ret:%d\n", __func__, __LINE__, *log_level_ptr, ret);
        s_mcu_data->mcu_log_level = *log_level_ptr;
    }
    else {
        I("%s(%d): MCU not in SHUB state\n", __func__, __LINE__);
    }

    return ret;
}

static ssize_t log_level_show(struct device *dev,
                                 struct device_attribute *attr,
                                 char *buf)
{
    uint32_t log_level = 0;
    int ret;
    ret = mcu_get_log_level(&log_level);
    if (ret < 0)
        log_level = 0;

    return snprintf(buf, PAGE_SIZE, "%d\n", (int)(log_level));
}

static ssize_t log_level_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf,
                                  size_t count)
{
    uint32_t log_level = 0;
    int error;

    error = kstrtou32(buf, 10, &log_level);
    if (error) {
        E("%s[%d]: kstrtoul fails, error = %d\n", __func__, __LINE__, error);
    }
    else {
        I("%s(%d): %d\n", __func__, __LINE__, log_level);
        mcu_set_log_level(log_level);
    }

    return count;
}

static u32 mcu_get_log_size(void)
{
    int ret;
    struct log_size_struct {
        u32 log_size;
        u32 drop_count;
    } cwmcu_log_size = {0, 0};

    if (MCU_IN_SHUB()) {
        ret = CWMCU_i2c_read_power(s_mcu_data, CW_I2C_REG_LOG_SIZE, (u8*)&cwmcu_log_size, sizeof(cwmcu_log_size));
        I("%s: ret:%d, log_size = %d, drop_count = %d\n", __func__, ret, cwmcu_log_size.log_size, cwmcu_log_size.drop_count);
    }
    else {
        I("%s: MCU not in SHUB state\n", __func__);
    }

    return cwmcu_log_size.log_size;
}

/*--------------------------------------
 * SHUB LOG Misc Device fops
 *-------------------------------------*/
static int shub_log_open(struct inode *inode, struct file *file)
{
    if (!file)
        return -ENODEV;

    I("%s(%d)\n", __func__, __LINE__);

    return 0;
}

static int shub_log_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}

static ssize_t shub_log_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    int ret;
    int read_len = 0;
    int total_read_len = 0;
    unsigned long  start_jiffies = jiffies;
    D("%s(%d): count(%lu)\n", __func__, __LINE__, count);

    if (MCU_IN_SHUB()) {
        u8 i2c_log_buf[I2C_LOG_READ_LEN];

        while (count > 0) {
            if (count < I2C_LOG_READ_LEN) {
                D("%s(%d): count(%u) < I2C_LOG_READ_LEN(%u)\n", __func__, __LINE__, (unsigned int)count, I2C_LOG_READ_LEN);
                goto EXIT;
            }
            ret = CWMCU_i2c_read_power(s_mcu_data, CW_I2C_REG_LOG_DATA, i2c_log_buf, I2C_LOG_READ_LEN);

            if (ret < 0) {
                E("%s: Read LOG_DATA fails, ret = %d\n", __func__, ret);
                goto EXIT;
            } else {
                read_len = ret;
                ret = copy_to_user(buf, i2c_log_buf,
                                   (read_len <= I2C_LOG_READ_LEN)
                                   ? read_len : I2C_LOG_READ_LEN);
                if (ret) {
                    E("%s(%d): copy_to_user failed err:%d\n", __func__, __LINE__, ret);
                    goto EXIT;
                }
            }

            D("%s: Read LOG_DATA, len:%d, log:%s\n", __func__, read_len, i2c_log_buf);
            count -= read_len;
            total_read_len += read_len;
            buf += read_len;
        }
    }

EXIT:
    I("%s(%d): return total_read_len(%d), elapsed_ms:%u\n", __func__, __LINE__, total_read_len, jiffies_to_msecs(jiffies - start_jiffies));
    return total_read_len;
}

static ssize_t shub_log_write(struct file *file, const char __user * buf, size_t size, loff_t *pos)
{
    return -ENODEV;
}

static long shub_log_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = 0;

    //wait until sensor hub is running
    while (!MCU_IN_SHUB()) {
        unsigned long timeout = wait_for_completion_timeout(&s_mcu_enter_shub_run, (msecs_to_jiffies(30000)));
        if (timeout == 0) {
            E("%s(%d): wait_for s_mcu_enter_shub_run timeout !!!\n", __func__, __LINE__);
        } else {
            I("%s(%d): s_mcu_enter_shub_run completely\n", __func__, __LINE__);
            break;
        }
    }

    switch (cmd) {
    case SHUB_LOG_IOCTL_WAIT_FOR_NOTIFY:
        {
            uint32_t mcu_log_type = 0;
            ret = wait_for_completion_interruptible(&s_mcu_log_avail);
            if (ret != -ERESTARTSYS) {
                I("%s(%d): s_mcu_log_avail retval:%d, mcu_log_type:0x%x\n", __func__, __LINE__, ret, mcu_log_type);
            }
            if (!ret) {
                put_user(mcu_log_type, (unsigned int __user *)arg);
            }
            reinit_completion(&s_mcu_log_avail);
        }
        break;

    case SHUB_LOG_IOCTL_GET_LOG_SIZE:
        {
            unsigned int mcu_log_size = 0;
            mcu_log_size = mcu_get_log_size();
            put_user(mcu_log_size, (unsigned int __user *)arg);
        }
        break;


    case SHUB_LOG_IOCTL_SET_LOGMASK:
        {
            uint32_t log_mask = 0;
            if (copy_from_user(&log_mask, (unsigned int __user *)arg, sizeof(log_mask))) {
                E("%s(%d): CWMCU_IOCTL_SET_LOGMASK invalid param\n", __func__, __LINE__);
                return -EFAULT;
            }
            mcu_set_log_mask(log_mask);
        }
        break;

    case SHUB_LOG_IOCTL_GET_LOGMASK:
        {
            uint32_t log_mask = 0;
            int ret;
            ret = mcu_get_log_mask(&log_mask);
            if (ret < 0)
                log_mask = 0;
            put_user(log_mask, (unsigned int __user *)arg);
        }
        break;

    case SHUB_LOG_IOCTL_SET_LOGLEVEL:
        {
            uint32_t log_level = 0;
            if (copy_from_user(&log_level, (unsigned int __user *)arg, sizeof(log_level))) {
                E("%s(%d): CWMCU_IOCTL_SET_LOGMASK invalid param\n", __func__, __LINE__);
                return -EFAULT;
            }
            mcu_set_log_level(log_level);
        }
        break;

    case SHUB_LOG_IOCTL_GET_LOGLEVEL:
        {
            uint32_t log_level = 0;
            int ret;
            ret = mcu_get_log_level(&log_level);
            put_user(log_level, (unsigned int __user *)arg);
        }
        break;

    case SHUB_LOG_IOCTL_GET_LOG_START:
        {
        }
        break;

    case SHUB_LOG_IOCTL_GET_LOG_DONE:
        {
        }
        break;

    default:
        E("%s(%d): INVALID param:0x%x\n", __func__, __LINE__, cmd);
        ret = -EINVAL;
        break;
    }

    return ret;
}

static const struct file_operations shub_log_fops = {
    .owner = THIS_MODULE,
    .read  = shub_log_read,
    .write = shub_log_write,
    .unlocked_ioctl = shub_log_ioctl,
    .compat_ioctl = shub_log_ioctl,
    .open  = shub_log_open,
    .release = shub_log_release,
};

static struct miscdevice shub_log_miscdev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = SHUB_LOG_DEVICE_NAME,
    .fops = &shub_log_fops
};
#endif //SHUB_LOGGING_SUPPORT

#ifdef SHUB_EVENT_SUPPORT
/*--------------------------------------
 * Internal Data Structure Definition
 *-------------------------------------*/

/*--------------------------------------
 * Internal Function Implementation
 *-------------------------------------*/
static u32 mcu_get_event_size(void)
{
    int ret;
    u32 event_size = 0;

    if (MCU_IN_SHUB()) {
        ret = CWMCU_i2c_read_power(s_mcu_data, CW_I2C_REG_EVENT_SIZE, (u8*)&event_size, sizeof(event_size));
        I("%s: ret:%d, event_size = %d\n", __func__, ret, event_size);
    }
    else {
        I("%s: MCU not in SHUB state\n", __func__);
    }

    return event_size;
}


/*--------------------------------------
 * SHUB EVENT Misc Device fops
 *-------------------------------------*/
static int shub_event_open(struct inode *inode, struct file *file)
{
    if (!file)
        return -ENODEV;

    I("%s(%d)\n", __func__, __LINE__);

    return 0;
}

static int shub_event_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}

static ssize_t shub_event_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    int ret;
    unsigned int read_len = 0;
    int total_read_len = 0;
    unsigned long  start_jiffies = jiffies;
    D("%s(%d): count(%lu)\n", __func__, __LINE__, count);

    if (MCU_IN_SHUB()) {
        u8 i2c_event_buf[I2C_EVENT_READ_LEN];

        while (count > 0) {
            if (count < I2C_EVENT_READ_LEN) {
                D("%s(%d): count(%u) < I2C_EVENT_READ_LEN(%u)\n", __func__, __LINE__, (unsigned int)count, I2C_EVENT_READ_LEN);
                goto EXIT;
            }
            ret = CWMCU_i2c_read_power(s_mcu_data, CW_I2C_REG_EVENT_DATA, i2c_event_buf, I2C_EVENT_READ_LEN);

            if (ret < 0) {
                E("%s: Read EVENT_DATA fails, ret = %d\n", __func__, ret);
                goto EXIT;
            } else {
                read_len = ret;
                ret = copy_to_user(buf, i2c_event_buf,
                                   (read_len <= I2C_EVENT_READ_LEN)
                                   ? read_len : I2C_EVENT_READ_LEN);
                if (ret) {
                    E("%s(%d): copy_to_user failed err:%d\n", __func__, __LINE__, ret);
                    goto EXIT;
                }
            }

            count -= read_len;
            total_read_len += read_len;
            buf += read_len;
        }
    }

EXIT:
    I("%s(%d): return total_read_len(%d), elapsed_ms:%u\n", __func__, __LINE__, total_read_len, jiffies_to_msecs(jiffies - start_jiffies));
    return total_read_len;
}

static ssize_t shub_event_write(struct file *file, const char __user * buf, size_t size, loff_t *pos)
{
    return -ENODEV;
}

static long shub_event_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = 0;

    //wait until sensor hub is running
    while (!MCU_IN_SHUB()) {
        unsigned long timeout = wait_for_completion_timeout(&s_mcu_enter_shub_run, (msecs_to_jiffies(30000)));
        if (timeout == 0) {
            E("%s(%d): wait_for s_mcu_enter_shub_run timeout !!!\n", __func__, __LINE__);
        } else {
            I("%s(%d): s_mcu_enter_shub_run completely\n", __func__, __LINE__);
            break;
        }
    }

    switch (cmd) {
    case SHUB_EVENT_IOCTL_WAIT_FOR_NOTIFY:
        {
            uint32_t mcu_event_type = 0;
            ret = wait_for_completion_interruptible(&s_mcu_event_avail);
            if (ret != -ERESTARTSYS) {
                I("%s(%d): s_mcu_event_avail retval:%d, mcu_event_type:0x%x\n", __func__, __LINE__, ret, mcu_event_type);
            }
            if (!ret) {
                put_user(mcu_event_type, (unsigned int __user *)arg);
            }
            reinit_completion(&s_mcu_event_avail);
        }
        break;

    case SHUB_EVENT_IOCTL_GET_EVENT_SIZE:
        {
            unsigned int mcu_event_size = 0;
            mcu_event_size = mcu_get_event_size();
            put_user(mcu_event_size, (unsigned int __user *)arg);
        }
        break;

    case SHUB_EVENT_IOCTL_GET_EVENT_START:
        {
        }
        break;

    case SHUB_EVENT_IOCTL_GET_EVENT_DONE:
        {
        }
        break;

    default:
        E("%s(%d): INVALID param:0x%x\n", __func__, __LINE__, cmd);
        ret = -EINVAL;
        break;
    }

    return ret;
}

static const struct file_operations shub_event_fops = {
    .owner = THIS_MODULE,
    .read  = shub_event_read,
    .write = shub_event_write,
    .unlocked_ioctl = shub_event_ioctl,
    .compat_ioctl = shub_event_ioctl,
    .open  = shub_event_open,
    .release = shub_event_release,
};

static struct miscdevice shub_event_miscdev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = SHUB_EVENT_DEVICE_NAME,
    .fops = &shub_event_fops
};
#endif //SHUB_EVENT_SUPPORT

#ifdef SHUB_DLOAD_SUPPORT
/*--------------------------------------
 * Internal Data Structure Definition
 *-------------------------------------*/
#define MCU_I2C_DLOAD_NAME "i2c-mcu-dload"
#define STM32MCUF411_RAM_START_ADDR       (0x20000000)
#define STM32MCUF411_RAM_SIZE             (0x20000)

static struct cwmcu_bus_client *dload_bus_client = NULL;

static unsigned int shub_ramdump_size = 0;
static unsigned int ramdump_capture_inprogress = 0;


/*--------------------------------------
 * Internal Function Implementation
 *-------------------------------------*/

static int mcu_dload_i2c_write(u8 cmd, u8 *data, u8 len)
{
    int ret;

    if (!dload_bus_client) return -EIO;

#ifdef CONFIG_SENSORS_SPI_BUS
	ret = mcu_spi_tx_cmd(dload_bus_client, cmd, data, len);
#else
    ret = mcu_i2c_tx_cmd(dload_bus_client, cmd, data, len);
#endif
    if (ret < 0) {
        E("%s error, cmd = 0x%x, ret = %d\n", __func__, cmd, ret);
    }

    return ret;
}

static int mcu_dload_i2c_read(u8 cmd, u8 *data, u8 len)
{
    int ret;

    if (!dload_bus_client) return -EIO;

#ifdef CONFIG_SENSORS_SPI_BUS
	ret = mcu_spi_rx_cmd(dload_bus_client, cmd, data, len);
#else
    ret = mcu_i2c_rx_cmd(dload_bus_client, cmd, data, len);
#endif
    if (ret < 0) {
        E("%s error: cmd = 0x%x, ret = %d\n", __func__, cmd, ret);
    }

    return ret;
}

static int mcu_dload_dump_backup_registers(void)
{
    int ret = 0;
    u8 bkp_reg;
    u32 data;

    for (bkp_reg = 0; bkp_reg <= 19; bkp_reg++) {
        ret = mcu_dload_i2c_write(CW_I2C_REG_DUMP_BACKUP_REG, (u8*)&bkp_reg, sizeof(bkp_reg));
        if (ret < 0) {
            E("%s: write CW_I2C_REG_DUMP_BACKUP_REG [%d] fails, ret = %d\n",
                __func__, bkp_reg, ret);
            break;
        }

        ret = mcu_dload_i2c_read(CW_I2C_REG_DUMP_BACKUP_REG, (u8*)&data, sizeof(data));
        if (ret < 0) {
            E("%s: BackupRegister[%d] dump fails, ret = %d\n",
                __func__, bkp_reg, ret);
            break;
        } else {
            I("%s: BackupRegister[%d] = 0x%x\n",
                __func__, bkp_reg, data);
        }
    }

    return ret;
}

static int mcu_dload_dump_exception_buffer(struct cwmcu_data *mcu_data)
{
    int ret;
    u32 exception_len = 0;

    ret = mcu_dload_i2c_read(CW_I2C_REG_EXCEPTION_BUFFER_LEN, (u8*)&exception_len, sizeof(exception_len));
    if (ret >= 0) {
        u8 data[EXCEPTION_BLOCK_LEN];
        int i;

        E("%s: exception_len = %u\n", __func__, exception_len);
        if (exception_len > EXCEPTION_LEN_MAX)
            exception_len = EXCEPTION_LEN_MAX;

        for (i = 0; exception_len > 0 ; i++) {
            memset(data, 0, sizeof(data));
            ret = mcu_dload_i2c_read(CW_I2C_REG_EXCEPTION_BUFFER, data, sizeof(data));
            if (ret >= 0) {
                char buf[3*EXCEPTION_BLOCK_LEN];
                u32 print_len = ((exception_len > EXCEPTION_BLOCK_LEN) ? EXCEPTION_BLOCK_LEN : exception_len);

                print_hex_data(buf, i, data, print_len);
                exception_len -= print_len;
                if (i == 0) {
                    mcu_data->crash_reason = (data[1] << 8) | data[0];
                }
            } else {
                E("%s: i = %d, excp1 i2c_read: ret = %d\n", __func__, i, ret);
                break;
            }
        }
    } else {
        E("%s: Exception status dump fails, ret = %d\n", __func__, ret);
    }

    return ret;
}

static inline int MCU_I2C_WRITE(u8 reg_addr, u8 *data, u8 len)
{
    if (MCU_IN_DLOAD() || (MCU_IN_UNKNOWN()&&(s_mcu_data->gpio_mcu_status_level==MCU2CPU_STATUS_GPIO_LEVEL_DLOAD)))
        return mcu_dload_i2c_write(reg_addr, data, len);
    else if (MCU_IN_SHUB() || (MCU_IN_UNKNOWN()&&(s_mcu_data->gpio_mcu_status_level==MCU2CPU_STATUS_GPIO_LEVEL_SHUB)))
        return CWMCU_i2c_write_block_power(s_mcu_data, reg_addr, data, len);
    else
        return 0;
}

static inline int MCU_I2C_READ(u8 reg_addr, u8 *data, u8 len)
{
    if (MCU_IN_DLOAD() || (MCU_IN_UNKNOWN()&&(s_mcu_data->gpio_mcu_status_level==MCU2CPU_STATUS_GPIO_LEVEL_DLOAD)))
        return mcu_dload_i2c_read(reg_addr, data, len);
    else if (MCU_IN_SHUB() || (MCU_IN_UNKNOWN()&&(s_mcu_data->gpio_mcu_status_level==MCU2CPU_STATUS_GPIO_LEVEL_SHUB)))
        return CWMCU_i2c_read_power(s_mcu_data, reg_addr, data, len);
    else
        return 0;
}

static int mcu_set_reboot_state(u32 state)
{
    u32 reboot_state_read = 0xff;
    int ret;

    I("%s: 0x%x s_mcu_state:%d\n", __func__, state, s_mcu_state);

    if (state != MCU_SYS_STATUS_DLOAD && state != MCU_SYS_STATUS_SHUB) {
        I("%s: Invalid state:0x%x\n", __func__, state);
        return -EINVAL;
    }

    ret = MCU_I2C_WRITE(CW_I2C_REG_REBOOT_MODE, (u8*)&state, sizeof(state));
    if (ret < 0) {
        E("%s[%d]: failed, ret=%d\n", __func__, __LINE__, ret);
        return ret;
    }

    ret = MCU_I2C_READ(CW_I2C_REG_REBOOT_MODE, (u8*)&reboot_state_read, sizeof(reboot_state_read));
    if (ret < 0) {
        E("%s[%d]: failed, ret=%d\n", __func__, __LINE__, ret);
        return ret;
    }

    if (reboot_state_read != state) {
        E("%s: mode_read(0x%x) != mode(0x%x)\n", __func__, reboot_state_read, state);
        ret = -EIO;
    }

    return ret;
}

static void mcu_enable_disable_dload_mode(bool en)
{
    s_mcu_data->dload_mode_enabled = en;
    if (s_mcu_data->dload_mode_enabled) {
        I("%s(%d): ramdump dbg mode is enabled, set hub reboot state to MCU_STATE_DLOAD\n", __func__, __LINE__);
        mcu_set_reboot_state(MCU_SYS_STATUS_DLOAD);
    }
    else {
        I("%s(%d): ramdump dbg mode is disabled, set hub reboot state to MCU_STATE_NORMAL\n", __func__, __LINE__);
        mcu_set_reboot_state(MCU_SYS_STATUS_SHUB);
    }
}



/*--------------------------------------
 * Device attributes
 *-------------------------------------*/
static ssize_t dload_enable_show(struct device *dev,
                                 struct device_attribute *attr,
                                 char *buf)
{
    return snprintf(buf, PAGE_SIZE, "%d\n", (int)(s_mcu_data->dload_mode_enabled));
}

static ssize_t dload_enable_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf,
                                  size_t count)
{
    unsigned long tmp;
    int error;
    error = kstrtoul(buf, 10, &tmp);
    if (error) {
        E("%s[%d]: kstrtoul fails, error = %d\n", __func__, __LINE__, error);
    }
    else {
        I("%s(%d): dload_mode_enabled:%lu\n", __func__, __LINE__, tmp);
        mcu_enable_disable_dload_mode(tmp);
    }

    return count;
}

// cat /sys/class/misc/shub_dload/dload_enable
static DEVICE_ATTR(dload_enable, S_IRUSR | S_IWUSR, dload_enable_show, dload_enable_store);

/*--------------------------------------
 * SHUB DLOAD Misc Device fops
 *-------------------------------------*/
static int shub_dload_open(struct inode *inode, struct file *file)
{
    if (!file)
        return -ENODEV;

    I("%s(%d)\n", __func__, __LINE__);

    return 0;
}

static int shub_dload_release(struct inode *inode, struct file *file)
{
    file->private_data = NULL;
    return 0;
}

static ssize_t shub_dload_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
    int ret;
    int read_len = 0;
    int total_read_len = 0;
    unsigned long  start_jiffies = jiffies;
    D("%s(%d): count(%lu) ramdump_capture_inprogress(%d)\n", __func__, __LINE__, count, ramdump_capture_inprogress);

    if (ramdump_capture_inprogress ){
        u8 i2c_ramdump_buf[I2C_RAMDUMP_READ_LEN];

        while (count > 0) {
            if (count < I2C_RAMDUMP_READ_LEN) {
                E("%s(%d): count(%u) < I2C_RAMDUMP_READ_LEN(%u)\n", __func__, __LINE__, (unsigned int)count, I2C_RAMDUMP_READ_LEN);
                goto EXIT;
            }
            ret = mcu_dload_i2c_read(CW_I2C_REG_CAPTURE_RAMDUMP, i2c_ramdump_buf, I2C_RAMDUMP_READ_LEN);

            if (ret < 0) {
                E("%s: Read RAMDUMP fails, ret = %d\n", __func__, ret);
                goto EXIT;
            } else {
                read_len = ret;
                ret = copy_to_user(buf, i2c_ramdump_buf, read_len);
                if (ret) {
                    E("%s(%d): copy_to_user failed err:%d\n", __func__, __LINE__, ret);
                    goto EXIT;
                }
            }

            D("%s: Read RAMDUMP, len = %d\n", __func__, read_len);
            count -= read_len;
            total_read_len += read_len;
            buf += read_len;
        }
    }

EXIT:
    I("%s(%d): return total_read_len(%d), elapsed_ms:%u\n", __func__, __LINE__, total_read_len, jiffies_to_msecs(jiffies - start_jiffies));
    return total_read_len;
}

static ssize_t shub_dload_write(struct file *file, const char __user * buf, size_t size, loff_t *pos)
{
    return -ENODEV;
}

static long shub_dload_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    int ret = 0;
    static unsigned long  ramdump_start_jiffies = 0;

    switch (cmd) {
    case SHUB_DLOAD_IOCTL_WAIT_FOR_NOTIFY:
        {
            uint32_t mcu_dload_type = 0;
            ret = wait_for_completion_interruptible(&s_mcu_ramdump_avail);
            if (ret != -ERESTARTSYS) {
                I("%s(%d): s_mcu_ramdump_avail retval:%d mcu_dload_type:%d\n", __func__, __LINE__, ret, mcu_dload_type);
            }
            if (!ret) {
                put_user(mcu_dload_type, (unsigned int __user *)arg);
            }
            reinit_completion(&s_mcu_ramdump_avail);
        }
        break;

    case SHUB_DLOAD_IOCTL_GET_RAMDUMP_SIZE:
        {
            put_user(shub_ramdump_size, (unsigned int __user *)arg);
        }
        break;

    case SHUB_DLOAD_IOCTL_RAMDUMP_START:
        {
            struct cwmcu_ramdump_param ramdump_param;

            ramdump_capture_inprogress = 1;

            if (1) {
                ramdump_param.start_addr = STM32MCUF411_RAM_START_ADDR;
                ramdump_param.size = STM32MCUF411_RAM_SIZE;

                mcu_dload_i2c_write(CW_I2C_REG_CAPTURE_RAMDUMP, ((u8*)&ramdump_param), sizeof(struct cwmcu_ramdump_param));
                shub_ramdump_size = ramdump_param.size;
                I("%s(%d): RAMDUMP_START start_addr:0x%x, size:0x%x\n", __func__, __LINE__, ramdump_param.start_addr, ramdump_param.size);
            }
            ramdump_start_jiffies = jiffies;
        }
        break;

    case SHUB_DLOAD_IOCTL_RAMDUMP_DONE:
        {
            unsigned long timeout;
            unsigned long  elapsed_ms = jiffies_to_msecs(jiffies - ramdump_start_jiffies);
            ramdump_capture_inprogress = 0;

            I("%s(%d): RAMDUMP_DONE, elapsed_ms:%lu\n", __func__, __LINE__, elapsed_ms);

            if (MCU_IN_DLOAD()) {
                I("%s(%d): reset mcu to SHUB state\n", __func__, __LINE__);
                mcu_set_reboot_state(MCU_SYS_STATUS_SHUB);
                if (s_mcu_data) {
                    mutex_lock(&s_activated_i2c_lock);
                    reset_hub(s_mcu_data, true);
                    mutex_unlock(&s_activated_i2c_lock);
                }
                timeout = wait_for_completion_timeout(&s_mcu_enter_shub_run, (msecs_to_jiffies(5000)));
                if (timeout == 0) {
                    E("%s(%d): wait_for s_mcu_enter_shub_run timeout !!!\n", __func__, __LINE__);
                } else {
                    I("%s(%d): s_mcu_enter_shub_run completely\n", __func__, __LINE__);
                }
            }
        }
        break;

    case SHUB_DLOAD_IOCTL_ENABLE_DLOAD:
        {
            uint8_t en;
            if (copy_from_user(&(en), (unsigned int __user *)arg, sizeof(en))) {
                E("%s(%d): SHUB_DLOAD_IOCTL_ENABLE_DLOAD invalid param\n", __func__, __LINE__);
                return -EFAULT;
            }

            I("%s(%d): SHUB_DLOAD_IOCTL_ENABLE_DLOAD en:%d\n", __func__, __LINE__, en);

            mcu_enable_disable_dload_mode(en);
        }
        break;

    default:
        E("%s(%d): INVALID param:0x%x\n", __func__, __LINE__, cmd);
        ret = -EINVAL;
        break;
    }

    return ret;
}

static const struct file_operations shub_dload_fops = {
    .owner = THIS_MODULE,
    .read  = shub_dload_read,
    .write = shub_dload_write,
    .unlocked_ioctl = shub_dload_ioctl,
    .compat_ioctl = shub_dload_ioctl,
    .open  = shub_dload_open,
    .release = shub_dload_release,
};

static struct miscdevice shub_dload_miscdev = {
    .minor = MISC_DYNAMIC_MINOR,
    .name = SHUB_DLOAD_DEVICE_NAME,
    .fops = &shub_dload_fops
};
#ifdef CONFIG_SENSORS_SPI_BUS
/*--------------------------------------
 * MCU dload SPI driver
 *-------------------------------------*/

static int mcu_dload_spi_probe(struct spi_device *spi)
{
    int ret;
	I("%s\n", __func__);
#if 0 //TODO: Review this if NFC request
    if (!strcmp(htc_get_bootmode(), "offmode_charging")) {
        I("%s: disabled when offmode_charging\n", __func__);
        return -EIO;
    }
#endif
    dload_bus_client = s_mcu_data->mcu_client;

    /* misc register */
    ret = misc_register(&shub_dload_miscdev);
    if (ret < 0) {
        E("%s: failed to register misc device for '%s'!\n", __func__, shub_dload_miscdev.name);
        return ret;
    }

    /* device attribute on sysfs */
    I("%s: create device attribute %s %s\n", __func__, dev_name(shub_dload_miscdev.this_device), dev_attr_dload_enable.attr.name);
    ret = device_create_file(shub_dload_miscdev.this_device, &dev_attr_dload_enable);
    if (ret < 0) {
        E("%s: cant create device attribute %s %s\n", __func__, dev_name(shub_dload_miscdev.this_device), dev_attr_dload_enable.attr.name);
        return ret;
    }

    return 0;
}

static int mcu_dload_spi_remove(struct spi_device *spi)
{
    I("%s\n", __func__);

    misc_deregister(&shub_dload_miscdev);

    return 0;
}

#else
/*--------------------------------------
 * MCU dload I2C driver
 *-------------------------------------*/
static int mcu_dload_i2c_probe(struct i2c_client *client,
            const struct i2c_device_id *id)
{
    int ret;
#if 0 //TODO: Review this if NFC request
    if (!strcmp(htc_get_bootmode(), "offmode_charging")) {
        I("%s: disabled when offmode_charging\n", __func__);
        return -EIO;
    }
#endif
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        E("%s: i2c_check_functionality error\n", __func__);
        return -EIO;
    }

	dload_bus_client = kmalloc(sizeof(struct cwmcu_bus_client), GFP_KERNEL);
	if(!dload_bus_client){
		E("%s: dload_bus_client kmalloc failed\n", __func__);
		return -ENOMEM;
	}

    dload_bus_client->i2c_client = client;
    I("%s: addr:%x\n", __func__,dload_bus_client->i2c_client->addr);

    /* misc register */
    ret = misc_register(&shub_dload_miscdev);
    if (ret < 0) {
        E("%s: failed to register misc device for '%s'!\n", __func__, shub_dload_miscdev.name);
        return ret;
    }

    /* device attribute on sysfs */
    I("%s: create device attribute %s %s\n", __func__, dev_name(shub_dload_miscdev.this_device), dev_attr_dload_enable.attr.name);
    ret = device_create_file(shub_dload_miscdev.this_device, &dev_attr_dload_enable);
    if (ret < 0) {
        E("%s: cant create device attribute %s %s\n", __func__, dev_name(shub_dload_miscdev.this_device), dev_attr_dload_enable.attr.name);
        return ret;
    }

    return 0;
}

static int mcu_dload_i2c_remove(struct i2c_client *client)
{
    I("%s\n", __func__);

    misc_deregister(&shub_dload_miscdev);

    return 0;
}

static const struct i2c_device_id mcu_dload_id[] = {
    {MCU_I2C_DLOAD_NAME, 0},
    { }
};

#ifdef CONFIG_OF
static struct of_device_id mcu_dload_match_table[] = {
    {.compatible = "htc_mcu_dload" },
    {},
};
#else
#define mcu_dload_match_table NULL
#endif

static struct i2c_driver mcu_dload_i2c_driver = {
    .driver = {
        .name = MCU_I2C_DLOAD_NAME,
        .owner = THIS_MODULE,
        .of_match_table = mcu_dload_match_table,
    },
    .probe    = mcu_dload_i2c_probe,
    .remove   = mcu_dload_i2c_remove,
    .id_table = mcu_dload_id,
};
#endif
#endif //SHUB_DLOAD_SUPPORT

#ifdef CONFIG_FB
static int fb_notifier_callback(struct notifier_block *self,
                                 unsigned long event, void *data)
{
    struct fb_event *evdata = data;
    int *blank;

    D("%s\n", __func__);
    if (evdata && evdata->data && event == FB_EVENT_BLANK && s_mcu_data &&
                    s_mcu_data->mcu_client->i2c_client) {
        blank = evdata->data;
        switch (*blank) {
        case FB_BLANK_UNBLANK:
            D("MCU late_resume\n");
            if (!s_mcu_data->is_display_on) {
                s_mcu_data->is_display_on = true;
                mcu_time_sync();
                mcu_set_display_state(s_mcu_data->is_display_on);

                #ifdef SHUB_LOGGING_SUPPORT
                complete(&s_mcu_log_avail);
                #endif //SHUB_LOGGING_SUPPORT

                #ifdef SHUB_EVENT_SUPPORT
                complete(&s_mcu_event_avail);
                #endif //SHUB_EVENT_SUPPORT
            }
            break;

        case FB_BLANK_POWERDOWN:
        case FB_BLANK_HSYNC_SUSPEND:
        case FB_BLANK_VSYNC_SUSPEND:
        case FB_BLANK_NORMAL:
            D("MCU early_suspend\n");
            if (s_mcu_data->is_display_on) {
                s_mcu_data->is_display_on = false;
                mcu_set_display_state(s_mcu_data->is_display_on);
            }
            break;
        }
    }
    return 0;
}

static void mcu_fb_register(struct work_struct *work)
{
    int ret = 0;
    struct cwmcu_data *mcu_data = container_of((struct delayed_work *)work,
                                    struct cwmcu_data, delay_work_register_fb);
    I("%s in", __func__);

    mcu_data->fb_notif.notifier_call = fb_notifier_callback;
    ret = fb_register_client(&mcu_data->fb_notif);
    if (ret)
        E("MCU ERR:Unable to register fb_notifier: %d\n", ret);
}
#endif //CONFIG_FB

static int CWMCU_probe_init(struct cwmcu_data *mcu_data,
				       struct iio_dev *indio_dev)
{
	int error;
	int i;

	error = mcu_fetch_cali_data(mcu_data, CALIBRATION_DATA_PATH);
	if (error)
		E("%s: fetch cali data failed, ret = %d\n", __func__, error);

	if (mcu_data->mcu_dev->of_node) {
		D("Device Tree parsing.");

		error = mcu_parse_dt(mcu_data->mcu_dev, mcu_data);
		if (error) {
			dev_err(mcu_data->mcu_dev,
				"%s: mcu_parse_dt for pdata failed. err = %d\n"
					, __func__, error);
			goto exit_mcu_parse_dt_fail;
		}
	} else {
		//TODO: remove it if we don't need platform data
		if (mcu_data->mcu_dev->platform_data != NULL) {
			mcu_data->acceleration_axes =
				((struct cwmcu_platform_data *)
				 mcu_data->mcu_client->spi_client->dev.platform_data)
				->acceleration_axes;
			mcu_data->magnetic_axes =
				((struct cwmcu_platform_data *)
				 mcu_data->mcu_client->spi_client->dev.platform_data)
				->magnetic_axes;
			mcu_data->gyro_axes =
				((struct cwmcu_platform_data *)
				 mcu_data->mcu_client->spi_client->dev.platform_data)
				->gyro_axes;
			mcu_data->gpio_wake_mcu =
				((struct cwmcu_platform_data *)
				 mcu_data->mcu_client->spi_client->dev.platform_data)
				->gpio_wake_mcu;
		}
	}

	if (gpio_is_valid(mcu_data->gpio_reset)) {
		error = gpio_request(mcu_data->gpio_reset, "cwmcu_reset");
		if (error)
			E("%s : request reset gpio fail\n", __func__);
		gpio_direction_input(mcu_data->gpio_reset);
	} else {
		E("%s : reset is not valid\n", __func__);
	}

#ifdef SHUB_DLOAD_SUPPORT
	if (gpio_is_valid(mcu_data->gpio_mcu_status)) {
		error = gpio_request(mcu_data->gpio_mcu_status, "cwmcu_status");
		E("%s : request cwmcu_status\n", __func__);
		if (error)
			E("%s : request gpio_mcu_status gpio fail\n", __func__);

		gpio_direction_input(mcu_data->gpio_mcu_status);
	} else {
		E("%s : gpio_mcu_status is not valid\n", __func__);
	}
	mcu_data->gpio_mcu_status_level = MCU2CPU_STATUS_GPIO_LEVEL(mcu_data);
	I("%s : gpio_mcu_status_level:%d\n", __func__, mcu_data->gpio_mcu_status_level);
#endif //SHUB_DLOAD_SUPPORT

	if (gpio_is_valid(mcu_data->gpio_wake_mcu)) {
		error = gpio_request(mcu_data->gpio_wake_mcu, "cwmcu_CPU2MCU");
		if (error)
			E("%s : request gpio_wake_mcu gpio fail\n", __func__);
		gpio_direction_output(mcu_data->gpio_wake_mcu, 1);
	} else {
		E("%s : gpio_wake_mcu is not valid\n", __func__);
	}

	if (gpio_is_valid(mcu_data->gpio_chip_mode)) {
		error = gpio_request(mcu_data->gpio_chip_mode, "cwmcu_hub_boot_mode");
		if (error)
			E("%s : request chip_mode gpio fail\n", __func__);
		gpio_direction_output(mcu_data->gpio_chip_mode, 0);
	} else {
		E("%s : chip_mode is not valid\n", __func__);
	}

	if (gpio_is_valid(mcu_data->gpio_mcu_irq)) {
		error = gpio_request(mcu_data->gpio_mcu_irq, "cwmcu_int");
		if (error) {
			E("%s : request irq gpio fail\n", __func__);
		}
		gpio_direction_input(mcu_data->gpio_mcu_irq);
	} else {
		E("%s : irq is not valid\n", __func__);
	}
#if 0 //TODO: Review this if NFC request
	/* NOTICE: return fail due to NFC disable i2c while offmode_charging in HIMA */
	if (!strcmp(htc_get_bootmode(), "offmode_charging")) {
		I("%s: disabled when offmode_charging\n", __func__);
		goto error_free_dev;
	}
#endif
	gpio_direction_output(mcu_data->gpio_reset, 0);
	gpio_direction_output(mcu_data->gpio_wake_mcu, 0);
	mcu_chip_mode_set(mcu_data, MCU_CHIP_MODE_APPLICATION);

	mcu_pinctrl_init(mcu_data);
	mutex_init(&mcu_data->mutex_lock);
	mutex_init(&mcu_data->group_i2c_lock);
	mutex_init(&mcu_data->power_mode_lock);
	mutex_init(&mcu_data->lock);

	INIT_DELAYED_WORK(&mcu_data->work, cwmcu_work_report);
	INIT_WORK(&mcu_data->one_shot_work, cwmcu_one_shot);

	error = cw_probe_buffer(indio_dev);
	if (error) {
		E("%s: iio cw_probe_buffer failed\n", __func__);
		goto error_free_dev;
	}
	error = cw_probe_trigger(indio_dev);
	if (error) {
		E("%s: iio cw_probe_trigger failed\n", __func__);
		goto error_remove_buffer;
	}
	error = iio_device_register(indio_dev);
	if (error) {
		E("%s: iio iio_device_register failed\n", __func__);
		goto error_remove_trigger;
	}

	error = create_sysfs_interfaces(mcu_data);
	if (error)
		goto err_free_mem;

	for (i = 0; i < num_sensors; i++) {
		mcu_data->sensors_time[i] = 0;
		mcu_data->report_period[i] = DEFAULT_DELAY_US * MS_TO_PERIOD;
	}

	s_mcu_data = mcu_data;

	wake_lock_init(&mcu_data->gesture_motion_wake_lock, WAKE_LOCK_SUSPEND,
		       "gesture_motion_wake_lock");
	wake_lock_init(&mcu_data->significant_wake_lock, WAKE_LOCK_SUSPEND,
		       "significant_wake_lock");
	wake_lock_init(&mcu_data->any_motion_wake_lock, WAKE_LOCK_SUSPEND,
		       "any_motion_wake_lock");
	wake_lock_init(&mcu_data->batch_read_wake_lock, WAKE_LOCK_SUSPEND,
		       "batch_read_wake_lock");
	wake_lock_init(&mcu_data->ps_read_wake_lock, WAKE_LOCK_SUSPEND,
		       "ps_read_wake_lock");
#ifdef SHUB_DLOAD_SUPPORT
	wake_lock_init(&mcu_data->mcu_dload_wake_lock, WAKE_LOCK_SUSPEND,
		       "mcu_dload_wake_lock");
#endif //SHUB_DLOAD_SUPPORT
	wake_lock_init(&mcu_data->report_wake_lock, WAKE_LOCK_SUSPEND,
		       "report_wake_lock");
	wake_lock_init(&mcu_data->one_shot_wake_lock, WAKE_LOCK_SUSPEND,
		       "one_shot_wake_lock");

	atomic_set(&mcu_data->delay, CWMCU_MAX_DELAY);
	init_irq_work(&mcu_data->iio_irq_work, iio_trigger_work);

	mcu_data->mcu_wq = create_singlethread_workqueue("htc_mcu");

	mcu_data->IRQ = gpio_to_irq(mcu_data->gpio_mcu_irq);

	D("Requesting irq = %d\n", mcu_data->IRQ);
	error = request_threaded_irq(mcu_data->IRQ, NULL, cwmcu_irq_handler,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT, "cwmcu", mcu_data);
	if (error)
		E("[CWMCU] could not request irq %d\n", error);
	error = enable_irq_wake(mcu_data->IRQ);
	if (error < 0)
		E("[CWMCU] could not enable irq as wakeup source %d\n", error);

	atomic_set(&mcu_data->suspended, 0);
	atomic_set(&mcu_data->critical_sect, 0);

#if 0
	vib_trigger_register_simple("vibrator", &vib_trigger);
#endif

#ifdef SHUB_LOGGING_SUPPORT
	error = misc_register(&shub_log_miscdev);
	if (error < 0) {
		E("%s: failed to register misc device for '%s'!\n", __func__, shub_log_miscdev.name);
	}
#endif //SHUB_LOGGING_SUPPORT

#ifdef SHUB_EVENT_SUPPORT
	error = misc_register(&shub_event_miscdev);
	if (error < 0) {
		E("%s: failed to register misc device for '%s'!\n", __func__, shub_event_miscdev.name);
	}
#endif //SHUB_EVENT_SUPPORT
	return 0;

#ifdef CONFIG_FB
	INIT_DELAYED_WORK(&mcu_data->delay_work_register_fb, mcu_fb_register);
	queue_delayed_work(mcu_data->mcu_wq, &mcu_data->delay_work_register_fb, msecs_to_jiffies(15000));
#endif //CONFIG_FB

	I("CWMCU_probe_init success!\n");

	return 0;

err_free_mem:
	if (indio_dev)
		iio_device_unregister(indio_dev);
error_remove_trigger:
	if (indio_dev)
		cwmcu_remove_trigger(indio_dev);
error_remove_buffer:
	if (indio_dev)
		cwmcu_remove_buffer(indio_dev);
error_free_dev:
exit_mcu_parse_dt_fail:
	return error;
}

#ifdef CONFIG_SENSORS_SPI_BUS
static int CWMCU_spi_probe(struct spi_device *spi)
{
	struct cwmcu_data *mcu_data;
	struct iio_dev *indio_dev;
	int error = -1;

	I("%s++: Do not call iio_push_to_buffers when !pseudo_irq_enable\n",
	  __func__);

	D("%s: sizeof(*mcu_data) = %lu\n", __func__, sizeof(*mcu_data));

	indio_dev = iio_device_alloc(sizeof(*mcu_data));
	if (!indio_dev) {
		I("%s: iio_device_alloc failed\n", __func__);
		return -ENOMEM;
	}

	indio_dev->name = CWMCU_I2C_NAME;
	indio_dev->dev.parent = &spi->dev;
	indio_dev->info = &cw_info;
	indio_dev->channels = cw_channels;
	indio_dev->num_channels = ARRAY_SIZE(cw_channels);
	indio_dev->modes |= INDIO_BUFFER_TRIGGERED;

	mcu_data = iio_priv(indio_dev);
	mcu_data->mcu_client = kmalloc(sizeof(struct cwmcu_bus_client), GFP_KERNEL);
	if(!mcu_data->mcu_client){
		E("%s: mcu_client kmalloc failed\n", __func__);
		goto error_free_mcu_client;
	}
	mcu_data->mcu_client->spi_client = spi;
	mcu_data->indio_dev = indio_dev;
	mcu_data->mcu_dev = &spi->dev;

	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_0;
	spi->chip_select = 0;

	error = spi_setup(spi);
	if (error < 0) {
		E("%s: spi_setup failed\n", __func__);
	}

	error = CWMCU_probe_init(mcu_data, indio_dev);
	if(error < 0)
		goto error_free_dev;
	spi_set_drvdata(spi, mcu_data);
	pm_runtime_enable(&spi->dev);

	spi->irq = mcu_data->IRQ;

	I("CWMCU_spi_probe success!\n");

#ifdef SHUB_DLOAD_SUPPORT
    mcu_dload_spi_probe(spi);
#endif //SHUB_DLOAD_SUPPORT

#ifdef SHUB_FIRMWARE_UPDATE_SUPPORT
    mcu_fw_flash_spi_probe(spi);
#endif //SHUB_FIRMWARE_UPDATE_SUPPORT
	//trigger mcu state transit to MCU_STATE_SHUB_RUN
	mcu_data->w_kick_start_mcu = true;
	queue_work(mcu_data->mcu_wq, &mcu_data->one_shot_work);
	disable_irq(mcu_data->IRQ);

#ifdef CONFIG_AK8789_HALLSENSOR
	hallsensor_register_notifier(&hallsensor_status_handler);
#endif

	return 0;

error_free_dev:
	if (spi->dev.of_node &&
	    ((struct cwmcu_platform_data *)mcu_data->mcu_client->spi_client->dev.platform_data))
		kfree(mcu_data->mcu_client->spi_client->dev.platform_data);
	kfree(mcu_data->mcu_client);
error_free_mcu_client:
	if (indio_dev)
		iio_device_free(indio_dev);
	s_mcu_data = NULL;

	return error;
}

static int CWMCU_spi_remove(struct spi_device *spi)
{
	struct cwmcu_data *mcu_data = spi_get_drvdata(spi);
#ifdef CONFIG_AK8789_HALLSENSOR
	hallsensor_unregister_notifier(&hallsensor_status_handler);
#endif
	gpio_set_value(mcu_data->gpio_wake_mcu, 1);

#ifdef SHUB_LOGGING_SUPPORT
	misc_deregister(&shub_log_miscdev);
#endif //SHUB_LOGGING_SUPPORT

#ifdef SHUB_EVENT_SUPPORT
	misc_deregister(&shub_event_miscdev);
#endif //SHUB_EVENT_SUPPORT

#ifdef SHUB_DLOAD_SUPPORT
	mcu_dload_spi_remove(spi);
#endif //SHUB_DLOAD_SUPPORT

#ifdef SHUB_FIRMWARE_UPDATE_SUPPORT
	mcu_fw_flash_spi_remove(spi);
#endif //SHUB_FIRMWARE_UPDATE_SUPPORT

#ifdef CONFIG_FB
	if (fb_unregister_client(&mcu_data->fb_notif))
		E("%s: Error occurred while unregistering fb_notifier\n", __func__);
#endif //CONFIG_FB

	wake_lock_destroy(&mcu_data->gesture_motion_wake_lock);
	wake_lock_destroy(&mcu_data->significant_wake_lock);
	wake_lock_destroy(&mcu_data->any_motion_wake_lock);
	wake_lock_destroy(&mcu_data->batch_read_wake_lock);
	wake_lock_destroy(&mcu_data->ps_read_wake_lock);
#ifdef SHUB_DLOAD_SUPPORT
	wake_lock_destroy(&mcu_data->mcu_dload_wake_lock);
#endif //SHUB_DLOAD_SUPPORT

	destroy_sysfs_interfaces(mcu_data);
	kfree(mcu_data);
	s_mcu_data = NULL;
	return 0;
}

static const struct dev_pm_ops cwmcu_pm_ops = {
	.suspend = cwmcu_suspend,
	.resume = cwmcu_resume
};


static const struct spi_device_id cwmcu_id[] = {
	{CWMCU_I2C_NAME, 0},
	{ }
};
#ifdef CONFIG_OF
static struct of_device_id mcu_match_table[] = {
	{.compatible = "htc_mcu" },
	{},
};
#else
#define mcu_match_table NULL
#endif

MODULE_DEVICE_TABLE(spi, cwmcu_id);

static struct spi_driver cwmcu_driver = {
	.driver = {
		.name = CWMCU_I2C_NAME,
		.owner = THIS_MODULE,
		.bus = &spi_bus_type,
		.pm = &cwmcu_pm_ops,
		.of_match_table = mcu_match_table,
	},
	.probe    = CWMCU_spi_probe,
	.remove   = CWMCU_spi_remove,
	.id_table = cwmcu_id,
};

static int __init CWMCU_spi_init(void)
{
	return spi_register_driver(&cwmcu_driver);
}
module_init(CWMCU_spi_init);

static void __exit CWMCU_spi_exit(void)
{
    spi_unregister_driver(&cwmcu_driver);
}
module_exit(CWMCU_spi_exit);

#else
static int CWMCU_i2c_probe(struct i2c_client *client,
				       const struct i2c_device_id *id)
{
	struct cwmcu_data *mcu_data;
	struct iio_dev *indio_dev;
	int error = -1;

	I("%s++: separate fetching cali data and parsing dt\n", __func__);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		E("%s: i2c_check_functionality error\n", __func__);
		return -EIO;
	}

	D("%s: sizeof(*mcu_data) = %lu\n", __func__, sizeof(*mcu_data));

	indio_dev = iio_device_alloc(sizeof(*mcu_data));
	if (!indio_dev) {
		I("%s: iio_device_alloc failed\n", __func__);
		return -ENOMEM;
	}

	i2c_set_clientdata(client, indio_dev);

	indio_dev->name = CWMCU_I2C_NAME;
	indio_dev->dev.parent = &client->dev;
	indio_dev->info = &cw_info;
	indio_dev->channels = cw_channels;
	indio_dev->num_channels = ARRAY_SIZE(cw_channels);
	indio_dev->modes |= INDIO_BUFFER_TRIGGERED;

	mcu_data = iio_priv(indio_dev);
	mcu_data->client = client;
	mcu_data->mcu_client = kmalloc(sizeof(struct cwmcu_bus_client), GFP_KERNEL);
	if(!mcu_data->mcu_client){
		E("%s: mcu_client kmalloc failed\n", __func__);
		goto error_free_mcu_client;
	}
	mcu_data->mcu_client->i2c_client = client;
	mcu_data->indio_dev = indio_dev;
	mcu_data->mcu_dev = &client->dev;


	error = CWMCU_probe_init(mcu_data, indio_dev);
	if(error < 0)
		goto error_free_dev;
	i2c_set_clientdata(client, mcu_data);
	pm_runtime_enable(&client->dev);

	client->irq = mcu_data->IRQ;

	I("CWMCU_i2c_probe success!\n");

	//trigger mcu state transit to MCU_STATE_SHUB_RUN
	mcu_data->w_kick_start_mcu = true;
	queue_work(mcu_data->mcu_wq, &mcu_data->one_shot_work);
	disable_irq(mcu_data->IRQ);

#ifdef CONFIG_AK8789_HALLSENSOR
	hallsensor_register_notifier(&hallsensor_status_handler);
#endif

	return 0;

error_free_dev:
	if (client->dev.of_node &&
	    ((struct cwmcu_platform_data *)mcu_data->mcu_dev->platform_data))
		kfree(mcu_data->mcu_dev->platform_data);
	kfree(mcu_data->mcu_client);
error_free_mcu_client:
	if (indio_dev)
		iio_device_free(indio_dev);
	i2c_set_clientdata(client, NULL);
	s_mcu_data = NULL;

	return error;
}


static int CWMCU_i2c_remove(struct i2c_client *client)
{
	struct cwmcu_data *mcu_data = i2c_get_clientdata(client);

#ifdef CONFIG_AK8789_HALLSENSOR
	hallsensor_unregister_notifier(&hallsensor_status_handler);
#endif
	gpio_set_value(mcu_data->gpio_wake_mcu, 1);

#ifdef SHUB_LOGGING_SUPPORT
	misc_deregister(&shub_log_miscdev);
#endif //SHUB_LOGGING_SUPPORT

#ifdef SHUB_EVENT_SUPPORT
	misc_deregister(&shub_event_miscdev);
#endif //SHUB_EVENT_SUPPORT

#ifdef CONFIG_FB
	if (fb_unregister_client(&mcu_data->fb_notif))
		E("%s: Error occurred while unregistering fb_notifier\n", __func__);
#endif //CONFIG_FB

	wake_lock_destroy(&mcu_data->gesture_motion_wake_lock);
	wake_lock_destroy(&mcu_data->significant_wake_lock);
	wake_lock_destroy(&mcu_data->any_motion_wake_lock);
	wake_lock_destroy(&mcu_data->batch_read_wake_lock);
	wake_lock_destroy(&mcu_data->ps_read_wake_lock);
#ifdef SHUB_DLOAD_SUPPORT
	wake_lock_destroy(&mcu_data->mcu_dload_wake_lock);
#endif //SHUB_DLOAD_SUPPORT

	destroy_sysfs_interfaces(mcu_data);
	kfree(mcu_data);
	s_mcu_data = NULL;
	return 0;
}

static const struct dev_pm_ops cwmcu_pm_ops = {
	.suspend = cwmcu_suspend,
	.resume = cwmcu_resume
};


static const struct i2c_device_id cwmcu_id[] = {
	{CWMCU_I2C_NAME, 0},
	{ }
};
#ifdef CONFIG_OF
static struct of_device_id mcu_match_table[] = {
	{.compatible = "htc_mcu" },
	{},
};
#else
#define mcu_match_table NULL
#endif

MODULE_DEVICE_TABLE(i2c, cwmcu_id);

static struct i2c_driver cwmcu_driver = {
	.driver = {
		.name = CWMCU_I2C_NAME,
		   .owner = THIS_MODULE,
		.pm = &cwmcu_pm_ops,
		.of_match_table = mcu_match_table,
	},
	.probe    = CWMCU_i2c_probe,
	.remove   = CWMCU_i2c_remove,
	.id_table = cwmcu_id,
};

static int __init CWMCU_i2c_init(void)
{
#ifdef SHUB_DLOAD_SUPPORT
    i2c_add_driver(&mcu_dload_i2c_driver);
#endif //SHUB_DLOAD_SUPPORT

#ifdef SHUB_FIRMWARE_UPDATE_SUPPORT
    i2c_add_driver(&mcu_fw_flash_i2c_driver);
#endif //SHUB_FIRMWARE_UPDATE_SUPPORT

	return i2c_add_driver(&cwmcu_driver);
}
module_init(CWMCU_i2c_init);

static void __exit CWMCU_i2c_exit(void)
{
#ifdef SHUB_DLOAD_SUPPORT
    i2c_del_driver(&mcu_dload_i2c_driver);
#endif //SHUB_DLOAD_SUPPORT

#ifdef SHUB_FIRMWARE_UPDATE_SUPPORT
    i2c_del_driver(&mcu_fw_flash_i2c_driver);
#endif //SHUB_FIRMWARE_UPDATE_SUPPORT

    i2c_del_driver(&cwmcu_driver);
}
module_exit(CWMCU_i2c_exit);
#endif
MODULE_DESCRIPTION("CWMCU I2C Bus Driver V1.6");
MODULE_AUTHOR("CyWee Group Ltd.");
MODULE_LICENSE("GPL");
