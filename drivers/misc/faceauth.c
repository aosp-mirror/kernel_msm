/*
 * Google FaceAuth driver
 *
 * Copyright (C) 2018 Google, Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/faceauth.h>
#include <linux/faceauth_shared.h>
#include <linux/firmware.h>
#include <linux/init.h>
#include <linux/module.h>
#include <misc/faceauth_addresses.h>
#include <misc/faceauth_hypx.h>

#include <linux/mfd/abc-pcie.h>
#include <linux/mfd/abc-pcie-notifier.h>

#include <linux/miscdevice.h>
#include <linux/rwsem.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/uio.h>
#include <linux/workqueue.h>

#include <abc-pcie-dma.h>
#include "../mfd/abc-pcie-dma.h"

/* ABC FW and workload binary offsets */
#define M0_FIRMWARE_ADDR 0x20000000
#define CALIBRATION_SIZE 0x400

/* This is to be enabled for dog food only */
#define ENABLE_AIRBRUSH_DEBUG (1)

#if ENABLE_AIRBRUSH_DEBUG
#define DEBUG_DATA_BIN_SIZE (2 * 1024 * 1024)
#define DEBUG_DATA_NUM_BINS (5)
#endif

/* ABC FW and workload path */
#define M0_FIRMWARE_PATH "faceauth.fw"

/* Timeout in ms */
#define FACEAUTH_TIMEOUT_MS 3000
#define M0_POLLING_PAUSE_MS 80
/* Polling interval in us */
#define M0_POLLING_INTERVAL_US 6000
/* Expected latency for FW to switch to faceauth (in us)*/
#define CONTEXT_SWITCH_TO_FACEAUTH_US 6000
/* Timeout for context switch (in ms) */
#define CONTEXT_SWITCH_TIMEOUT_MS 40

/* Citadel */
#define MAX_CACHE_SIZE 512

struct faceauth_data {
	bool hypx_enable;
	/* This is to dynamically set the level of debugging in faceauth fw */
	uint64_t m0_verbosity_level;
#if IS_ENABLED(CONFIG_DEBUG_FS)
	struct dentry *debugfs_root;
#endif
	uint16_t session_id;
	/* This counter holds the number of interaction between driver and
	 * firmware, using which, faceauth firmware detects a missed command and
	 * returns an error
	 */
	uint32_t session_counter;
	bool can_transfer; /* Guarded by rwsem */
	uint64_t retry_count;
	atomic_t in_use;
	struct rw_semaphore rwsem;
	struct miscdevice misc_dev;
	struct device *device;
	struct delayed_work listener_init;
	struct notifier_block pcie_link_blocking_nb;
	bool is_secure_camera;
};

static int process_cache_flush_idxs(int16_t *flush_idxs, uint32_t flush_size);
static int dma_xfer(void *buf, int size, const int remote_addr,
		    enum dma_data_direction dir);
static int dma_xfer_vmalloc(void *buf, int size, const int remote_addr,
			    enum dma_data_direction dir);
static int dma_xfer_dmabuf(int fd, int size, const int remote_addr,
			   enum dma_data_direction dir);
static int dma_send_fw(struct device *device, const char *path,
		       const int remote_addr);
static int dma_read_dw(const int remote_addr, int *val);
static int dma_send_images(struct faceauth_start_data *data);
static int pio_write_qw(const int remote_addr, const uint64_t val);
#if ENABLE_AIRBRUSH_DEBUG
static int dma_gather_debug_log(struct faceauth_debug_data *data);
static int dma_gather_debug_data(void *destination_buffer,
				 uint32_t buffer_size);
static void clear_debug_data(void);
static void move_debug_data_to_tail(void);
static void enqueue_debug_data(struct faceauth_data *data, bool el2);
static int dequeue_debug_data(struct faceauth_debug_data *debug_step_data);

struct {
	int head_idx;
	int tail_idx;
	int count;
	char *data_buffer;
} debug_data_queue;
#endif /* #if ENABLE_AIRBRUSH_DEBUG */

static long faceauth_dev_ioctl_el1(struct file *file, unsigned int cmd,
				   unsigned long arg);
static long faceauth_dev_ioctl_el2(struct file *file, unsigned int cmd,
				   unsigned long arg);

/* M0 Verbosity Level Encoding

   64 bits wide allocated as follows:
   Bit  0   Errors
   Bits 1-3 Performance
   Bits 4-7 Scheduler
   Bits 8-11 IPU
   Bits 12-15 TPU
   Bits 16-19 Post Process
   Bits 20-63 Reserved

   In these slots, the debug levels are specified as follows:
   Level 0: 0b0000
   Level 1: 0b1000
   Level 2: 0b0100
   Level 3: 0b0010
   Level 4: 0b0001

   Level 0 means errors only. The other levels yield increasingly more
   information.

   To set all these levels, you must write the number in either
   unsigned hexadecimal format or unisigned decimal format
   to a certain file: /d/faceauth/m0_verbosity_level
   If using hexadecimal, you need to put "0x" in front.
   For example:
   either
   adb shell "echo 0x108248 > /d/faceauth/m0_verbosity_level"
   or
   adb shell "echo 1081928 > /d/faceauth/m0_verbosity_level"
   will result in the following settings:
   general errors level 0 (meaning ON)
   performance level 2
   scheduler level 3
   IPU level 1
   TPU level 0
   post process level 4
*/

static long faceauth_dev_ioctl(struct file *file, unsigned int cmd,
			       unsigned long arg)
{
	long ioctl_ret;
	struct faceauth_data *data = file->private_data;

	if (data->hypx_enable)
		ioctl_ret = faceauth_dev_ioctl_el2(file, cmd, arg);
	else
		ioctl_ret = faceauth_dev_ioctl_el1(file, cmd, arg);
	return ioctl_ret;
}

static long faceauth_dev_ioctl_el1(struct file *file, unsigned int cmd,
				   unsigned long arg)
{
	int err = 0;
	int polling_interval = M0_POLLING_INTERVAL_US;
	struct faceauth_init_data init_step_data = { 0 };
	struct faceauth_start_data start_step_data = { 0 };
	unsigned long stop;
	uint32_t ab_input;
	uint32_t ab_result;
	uint32_t angles;
	uint32_t dma_read_value;
	uint32_t fw_execution_status;
	uint32_t temp_data;
	struct faceauth_data *data = file->private_data;
#if ENABLE_AIRBRUSH_DEBUG
	struct faceauth_debug_data debug_step_data = { 0 };
#endif

	down_read(&data->rwsem);
	if (!data->can_transfer && cmd != FACEAUTH_DEV_IOC_DEBUG_DATA) {
		err = -EIO;
		pr_info("Cannot do transfer due to link down\n");
		goto exit;
	}

	switch (cmd) {
	case FACEAUTH_DEV_IOC_INIT:
		pr_info("faceauth init IOCTL\n");

		if (copy_from_user(&init_step_data, (const void __user *)arg,
				   sizeof(init_step_data))) {
			err = -EFAULT;
			goto exit;
		}

		data->is_secure_camera =
			init_step_data.features & SECURE_CAMERA_DATA;

		err = dma_send_fw(data->device, M0_FIRMWARE_PATH,
				  M0_FIRMWARE_ADDR);
		if (err) {
			pr_err("Error during M0 firmware transfer: %d\n", err);
			goto exit;
		}

		pio_write_qw(DYNAMIC_VERBOSITY_RAM_ADDR,
			     data->m0_verbosity_level);
		pio_write_qw(DISABLE_FEATURES_ADDR, init_step_data.features);

		data->session_counter = 0x0;
		aon_config_write(INPUT_COMMAND_ADDR, 4, COMMAND_NONE);
		aon_config_write(SYSREG_AON_IPU_REG29, 4, M0_FIRMWARE_ADDR);
		aon_config_write(SYSREG_REG_GP_INT0, 4, 1);

		/* Expected fw context switch latency < 6us, timeout after
		 * 40ms
		 * Expected latency is probably less than 6ms, if FW load is in
		 * critical path, we can reduce this latency
		 */
		stop = jiffies + msecs_to_jiffies(CONTEXT_SWITCH_TIMEOUT_MS);
		usleep_range(CONTEXT_SWITCH_TO_FACEAUTH_US,
			     CONTEXT_SWITCH_TO_FACEAUTH_US + 1);

		for (;;) {
			err = aon_config_read(ACK_TO_HOST_ADDR, 4,
					      &fw_execution_status);
			if (err) {
				pr_err("Error reading completion flag\n");
				goto exit;
			}
			if (fw_execution_status == STATUS_READY)
				break;
			if (time_before(stop, jiffies)) {
				pr_err("Faceauth FW context switch timeout!\n");
				err = -ETIME;
				goto exit;
			}
			usleep_range(100, 1000);
		}
		break;
	case FACEAUTH_DEV_IOC_START:
		pr_info("faceauth start IOCTL\n");

		if (copy_from_user(&start_step_data, (const void __user *)arg,
				   sizeof(start_step_data))) {
			err = -EFAULT;
			goto exit;
		}

		start_step_data.result = 0;
		start_step_data.error_code = 0;

		if (start_step_data.operation == COMMAND_ENROLL ||
		    start_step_data.operation == COMMAND_VALIDATE) {
			if (!start_step_data.image_dot_left_size) {
				err = -EINVAL;
				goto exit;
			}
			if (!start_step_data.image_dot_right_size) {
				err = -EINVAL;
				goto exit;
			}
			if (!start_step_data.image_flood_size) {
				err = -EINVAL;
				goto exit;
			}
			if (!start_step_data.calibration_size ||
			    start_step_data.calibration_size >
				    CALIBRATION_SIZE) {
				err = -EINVAL;
				goto exit;
			}

			err = dma_send_images(&start_step_data);
			if (err) {
				pr_err("Error in sending workload\n");
				goto exit;
			}
			if (start_step_data.calibration) {
				pr_info("Send calibration data\n");
				err = dma_xfer(start_step_data.calibration,
					       start_step_data.calibration_size,
					       CALIBRATION_DATA_ADDR,
					       DMA_TO_DEVICE);
				if (err) {
					pr_err("Error sending calibration\n");
					goto exit;
				}
			} else if (start_step_data.calibration_fd) {
				pr_info("Send calibration data from ION\n");
				err = dma_xfer_dmabuf(
					start_step_data.calibration_fd,
					start_step_data.calibration_size,
					CALIBRATION_DATA_ADDR, DMA_TO_DEVICE);
				if (err) {
					pr_err("Error sending calibration data from ION\n");
					goto exit;
				}
			}
		}

		if (start_step_data.operation == COMMAND_ENROLL_COMPLETE) {
			err = process_cache_flush_idxs(
				start_step_data.cache_flush_indexes,
				start_step_data.cache_flush_size);
			if (err)
				goto exit;
			err = dma_xfer_vmalloc(
				start_step_data.cache_flush_indexes,
				sizeof(start_step_data.cache_flush_indexes),
				CACHE_FLUSH_INDEXES_ADDR, DMA_TO_DEVICE);
			if (err) {
				pr_err("Error sending flush indexes\n");
				goto exit;
			}
		}

		if (start_step_data.citadel_token_size && (
		    start_step_data.operation == COMMAND_ENROLL_COMPLETE ||
		    start_step_data.operation == COMMAND_SET_FEATURE ||
		    start_step_data.operation == COMMAND_CLR_FEATURE ||
		    start_step_data.operation == COMMAND_RESET_LOCKOUT)) {
			err = dma_xfer(start_step_data.citadel_token,
			start_step_data.citadel_token_size,
			CITADEL_WRITE_TOKEN_ADDR, DMA_TO_DEVICE);
			if (err) {
				pr_err("Error sending token data\n");
				goto exit;
			}
		}

		err = aon_config_write(CITADEL_INPUT_DATA_ADDR, 4,
				       start_step_data.citadel_input);
		if (err) {
			pr_err("Error sending citadel input data\n");
			goto exit;
		}

		/* Set input flag */
		ab_input = ((++data->session_id & 0xFFFF) << 16) |
			   ((start_step_data.profile_id & 0xFF) << 8) |
			   ((start_step_data.operation & 0xFF));
		pr_info("Set faceauth input flag = 0x%08x\n", ab_input);
		err = aon_config_write(INPUT_FLAG_ADDR, 4, ab_input);
		if (err) {
			pr_err("Error setting faceauth FW address\n");
			goto exit;
		}
		aon_config_write(RESULT_FLAG_ADDR, 4, 0);
		aon_config_write(INPUT_COUNTER_ADDR, 4,
				 data->session_counter++);
		aon_config_write(INPUT_COMMAND_ADDR, 4,
				 start_step_data.operation);
		pr_info("Waiting for completion.\n");
		msleep(M0_POLLING_PAUSE_MS);
		stop = jiffies + msecs_to_jiffies(FACEAUTH_TIMEOUT_MS);
		for (;;) {
			err = aon_config_read(RESULT_FLAG_ADDR, 4, &ab_result);
			if (err) {
				pr_err("Error reading completion flag\n");
				goto exit;
			}

			if (ab_result != WORKLOAD_STATUS_NO_STATUS) {
				pr_info("Faceauth workflow completes.\n");

				break;
			}
			if (time_before(stop, jiffies)) {
				pr_err("Faceauth workflow timeout!\n");
				err = -ETIME;
				goto exit;
			}
			usleep_range(polling_interval, polling_interval + 1);
			polling_interval = polling_interval > 1 ?
						   polling_interval >> 1 :
						   1;
		}
#if ENABLE_AIRBRUSH_DEBUG
		enqueue_debug_data(data, false);
#endif

		start_step_data.result = ab_result;
		err = aon_config_read(CITADEL_OUTPUT_DATA1_ADDR, 4, &temp_data);
		if (err) {
			pr_err("Error reading Citadel Output Data 1\n");
			goto exit;
		}
		start_step_data.citadel_output1 = temp_data;

		err = aon_config_read(CITADEL_OUTPUT_DATA2_ADDR, 4, &temp_data);
		if (err) {
			pr_err("Error reading Citadel Output Data 2\n");
			goto exit;
		}
		start_step_data.citadel_output2 = temp_data;

		err = aon_config_read(CITADEL_LOCKOUT_EVENT_ADDR, 4,
				      &temp_data);
		if (err) {
			pr_err("Error reading Citadel Lockout Event\n");
			goto exit;
		}
		start_step_data.citadel_lockout_event = temp_data;

		err = aon_config_read(ANGLE_RESULT_FLAG_ADDR, 4, &angles);
		if (err) {
			pr_err("Error reading angle result flag\n");
			goto exit;
		}
		start_step_data.angles = angles;

		dma_read_dw(INTERNAL_STATE_ADDR +
				    offsetof(struct faceauth_airbrush_state,
					     error_code),
			    &dma_read_value);
		start_step_data.error_code = dma_read_value;

		pr_info("Faceauth error code 0x%08x\n",
		       start_step_data.error_code);
		dma_read_dw(INTERNAL_STATE_ADDR +
				    offsetof(struct faceauth_airbrush_state,
					     faceauth_version),
			    &dma_read_value);
		start_step_data.fw_version = dma_read_value;

		if (copy_to_user((void __user *)arg, &start_step_data,
				 sizeof(start_step_data)))
			err = -EFAULT;
		goto exit;
		break;
	case FACEAUTH_DEV_IOC_CLEANUP:
		aon_config_write(INPUT_COUNTER_ADDR, 4,
				 data->session_counter++);
		aon_config_write(INPUT_COMMAND_ADDR, 4, COMMAND_EXIT);
		/* TODO cleanup Airbrush DRAM */
		data->is_secure_camera = false;
		pr_info("faceauth cleanup IOCTL\n");
		break;
	case FACEAUTH_DEV_IOC_DEBUG:
#if ENABLE_AIRBRUSH_DEBUG
		pr_info("faceauth debug log IOCTL\n");
		if (copy_from_user(&debug_step_data, (const void __user *)arg,
				   sizeof(debug_step_data))) {
			err = -EFAULT;
			goto exit;
		}
		err = dma_gather_debug_log(&debug_step_data);
#else
		err = -EOPNOTSUPP;
#endif /* #if ENABLE_AIRBRUSH_DEBUG */
		break;
	case FACEAUTH_DEV_IOC_DEBUG_DATA:
#if ENABLE_AIRBRUSH_DEBUG
		pr_info("faceauth debug data IOCTL\n");

		if (copy_from_user(&debug_step_data, (const void __user *)arg,
				   sizeof(debug_step_data))) {
			err = -EFAULT;
			goto exit;
		}

		if (debug_step_data.debug_buffer_size < DEBUG_DATA_BIN_SIZE) {
			err = -EINVAL;
			goto exit;
		}

		switch (debug_step_data.flags) {
		case FACEAUTH_GET_DEBUG_DATA_FROM_FIFO:
			err = dequeue_debug_data(&debug_step_data);
			break;
		case FACEAUTH_GET_DEBUG_DATA_MOST_RECENT:
			move_debug_data_to_tail();
			err = dequeue_debug_data(&debug_step_data);
			break;
		case FACEAUTH_GET_DEBUG_DATA_FROM_AB_DRAM:
			if (!data->can_transfer) {
				pr_info("Cannot do transfer due to link down\n");
				err = -EIO;
				goto exit;
			}
			clear_debug_data();
			enqueue_debug_data(data, false);
			err = dequeue_debug_data(&debug_step_data);
			break;
		default:
			err = -EINVAL;
			break;
		}
#else
		err = -EOPNOTSUPP;
#endif /* #if ENABLE_AIRBRUSH_DEBUG */
		break;
	default:
		err = -EOPNOTSUPP;
		goto exit;
	}

exit:
	up_read(&data->rwsem);
	return err;
}

static long faceauth_dev_ioctl_el2(struct file *file, unsigned int cmd,
				   unsigned long arg)
{
	int err = 0;
	int polling_interval = M0_POLLING_INTERVAL_US;
	struct faceauth_start_data start_step_data = { 0 };
	struct faceauth_init_data init_step_data = { 0 };
	unsigned long stop;
	bool send_images_data;
	struct faceauth_data *data = file->private_data;
#if ENABLE_AIRBRUSH_DEBUG
	struct faceauth_debug_data debug_step_data;
#endif

	down_read(&data->rwsem);
	if (!data->can_transfer && cmd != FACEAUTH_DEV_IOC_DEBUG_DATA) {
		err = -EIO;
		pr_info("Cannot do transfer due to link down\n");
		goto exit;
	}

	switch (cmd) {
	case FACEAUTH_DEV_IOC_INIT:
		pr_info("el2: faceauth init IOCTL\n");

		if (copy_from_user(&init_step_data, (const void __user *)arg,
				   sizeof(init_step_data))) {
			err = -EFAULT;
			goto exit;
		}

		err = el2_faceauth_wait_pil_dma_over();
		if (err < 0)
			goto exit;

		data->is_secure_camera =
			init_step_data.features & SECURE_CAMERA_DATA;

		err = el2_faceauth_init(data->device, &init_step_data,
					data->m0_verbosity_level);
		if (err < 0)
			goto exit;
		break;
	case FACEAUTH_DEV_IOC_START:
		pr_info("el2: faceauth start IOCTL\n");

		if (copy_from_user(&start_step_data, (const void __user *)arg,
				   sizeof(start_step_data))) {
			err = -EFAULT;
			goto exit;
		}

		send_images_data =
			start_step_data.operation == COMMAND_ENROLL ||
			start_step_data.operation == COMMAND_VALIDATE;
		if (send_images_data) {
			if (!start_step_data.image_dot_left_size) {
				err = -EINVAL;
				goto exit;
			}
			if (!start_step_data.image_dot_right_size) {
				err = -EINVAL;
				goto exit;
			}
			if (!start_step_data.image_flood_size) {
				err = -EINVAL;
				goto exit;
			}
		}

		err = process_cache_flush_idxs(
			start_step_data.cache_flush_indexes,
			start_step_data.cache_flush_size);
		if (err)
			goto exit;

		err = el2_faceauth_process(data->device, &start_step_data,
					   data->is_secure_camera);
		if (err)
			goto exit;

		/* Check completion flag */
		pr_info("Waiting for completion.\n");
		msleep(M0_POLLING_PAUSE_MS);
		stop = jiffies + msecs_to_jiffies(FACEAUTH_TIMEOUT_MS);
		for (;;) {
			err = el2_faceauth_get_process_result(data->device,
							      &start_step_data);

			if (err) {
				pr_err("Failed to get results from EL2 %d\n",
				       err);
				goto exit;
			}

			if (start_step_data.result !=
			    WORKLOAD_STATUS_NO_STATUS) {
				/* We've got a non-zero status from AB executor
				 * Faceauth processing is completed
				 */
				break;
			}
			if (time_before(stop, jiffies)) {
				err = -ETIME;
				goto exit;
			}

			usleep_range(polling_interval, polling_interval + 1);
			polling_interval = polling_interval > 1 ?
						   polling_interval >> 1 :
						   1;
		}
#if ENABLE_AIRBRUSH_DEBUG
		enqueue_debug_data(data, true);
#endif
		if (copy_to_user((void __user *)arg, &start_step_data,
				 sizeof(start_step_data))) {
			err = -EFAULT;
			goto exit;
		}
		break;
	case FACEAUTH_DEV_IOC_CLEANUP:
		/* In case of EL2 cleanup happens in PIL callback */
		/* TODO cleanup Airbrush DRAM */
		pr_info("el2: faceauth cleanup IOCTL\n");
		el2_faceauth_cleanup(data->device);
		data->is_secure_camera = false;
		break;
	case FACEAUTH_DEV_IOC_DEBUG:
#if ENABLE_AIRBRUSH_DEBUG
		pr_info("el2: faceauth debug log IOCTL\n");
		if (copy_from_user(&debug_step_data, (const void __user *)arg,
				   sizeof(debug_step_data))) {
			err = -EFAULT;
			goto exit;
		}
		err = el2_faceauth_gather_debug_log(data->device,
						    &debug_step_data);
#else
		err = -EOPNOTSUPP;
#endif /* #if ENABLE_AIRBRUSH_DEBUG */
		break;

	case FACEAUTH_DEV_IOC_DEBUG_DATA:
#if ENABLE_AIRBRUSH_DEBUG
		pr_info("el2: faceauth debug data IOCTL\n");

		if (copy_from_user(&debug_step_data, (const void __user *)arg,
				   sizeof(debug_step_data))) {
			err = -EFAULT;
			goto exit;
		}

		if (debug_step_data.debug_buffer_size < DEBUG_DATA_BIN_SIZE) {
			err = -EINVAL;
			goto exit;
		}

		switch (debug_step_data.flags) {
		case FACEAUTH_GET_DEBUG_DATA_FROM_FIFO:
			err = dequeue_debug_data(&debug_step_data);
			break;
		case FACEAUTH_GET_DEBUG_DATA_MOST_RECENT:
			move_debug_data_to_tail();
			err = dequeue_debug_data(&debug_step_data);
			break;
		case FACEAUTH_GET_DEBUG_DATA_FROM_AB_DRAM:
			if (!data->can_transfer) {
				pr_info("Cannot do transfer due to link down\n");
				err = -EIO;
				goto exit;
			}
			clear_debug_data();
			enqueue_debug_data(data, true);
			err = dequeue_debug_data(&debug_step_data);
			break;
		default:
			err = -EINVAL;
			break;
		}
#else
		err = -EOPNOTSUPP;
#endif /* #if ENABLE_AIRBRUSH_DEBUG */
		break;
	default:
		err = -EOPNOTSUPP;
		goto exit;
	}

exit:
	up_read(&data->rwsem);
	return err;
}

static int faceauth_open(struct inode *inode, struct file *file)
{
	struct miscdevice *m = file->private_data;
	struct faceauth_data *data =
		container_of(m, struct faceauth_data, misc_dev);

	file->private_data = data;
	if (atomic_cmpxchg(&data->in_use, 0, 1))
		return -EBUSY;
	return 0;
}

static int faceauth_release(struct inode *inode, struct file *file)
{
	struct faceauth_data *data = file->private_data;
	atomic_set(&data->in_use, 0);
	return 0;
}

static const struct file_operations faceauth_dev_operations = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = faceauth_dev_ioctl,
	.compat_ioctl = faceauth_dev_ioctl,
	.open = faceauth_open,
	.release = faceauth_release,
};

static int process_cache_flush_idxs(int16_t *flush_idxs, uint32_t flush_size)
{
	int i;

	if (flush_size > FACEAUTH_MAX_CACHE_FLUSH_SIZE) {
		pr_err("Wrong cache flush size\n");
		return -EINVAL;
	}

	if (flush_size < FACEAUTH_MAX_CACHE_FLUSH_SIZE) {
		flush_idxs[flush_size] = -1;
	}

	for (i = 0; i < flush_size; ++i) {
		if (flush_idxs[i] < 0 || flush_idxs[i] >= MAX_CACHE_SIZE) {
			pr_err("Wrong cache flush index\n");
			return -EINVAL;
		}
	}

	return 0;
}

/**
 * Local function to transfer data between user space memory and Airbrush via
 * PCIE
 * @param[in] buf Address of user space buffer
 * @param[in] size Size of buffer
 * @param[in] remote_addr Address of Airbrush memory
 * @param[in] dir Direction of data transfer
 * @return Status, zero if succeed, non-zero if fail
 */
static int dma_xfer(void *buf, int size, const int remote_addr,
		    enum dma_data_direction dir)
{
	struct abc_pcie_dma_desc dma_desc;

	/* Transfer workload to target memory in Airbrush */
	memset(&dma_desc, 0, sizeof(dma_desc));
	dma_desc.local_buf = buf;
	dma_desc.local_buf_type = DMA_BUFFER_USER;
	dma_desc.remote_buf = remote_addr;
	dma_desc.remote_buf_type = DMA_BUFFER_USER;
	dma_desc.size = size;
	dma_desc.dir = dir;
	return abc_pcie_issue_dma_xfer(&dma_desc);
}

/**
 * Local function to transfer data between kernel vmalloc memory and Airbrush
 * via PCIE
 * @param[in] buf Address of kernel vmalloc memory buffer
 * @param[in] size Size of buffer
 * @param[in] remote_addr Address of Airbrush memory
 * @param[in] dir Direction of data transfer
 * @return Status, zero if succeed, non-zero if fail
 */
static int dma_xfer_vmalloc(void *buf, int size, const int remote_addr,
			    enum dma_data_direction dir)
{
	struct abc_pcie_dma_desc dma_desc;

	/* Transfer workload to target memory in Airbrush */
	memset(&dma_desc, 0, sizeof(dma_desc));
	dma_desc.local_buf = buf;
	dma_desc.local_buf_type = DMA_BUFFER_USER;
	dma_desc.remote_buf = remote_addr;
	dma_desc.remote_buf_type = DMA_BUFFER_USER;
	dma_desc.size = size;
	dma_desc.dir = dir;
	return abc_pcie_issue_dma_xfer_vmalloc(&dma_desc);
}

/**
 * Local function to transfer data between user space memory and Airbrush via
 * PCIE using ION handler
 * @param[in] fd Fd of user space dma buffer
 * @param[in] size Size of buffer
 * @param[in] remote_addr Address of Airbrush memory
 * @param[in] dir Direction of data transfer
 * @return Status, zero if succeed, non-zero if fail
 */
static int dma_xfer_dmabuf(int fd, int size, const int remote_addr,
			   enum dma_data_direction dir)
{
	struct abc_pcie_dma_desc dma_desc;

	/* Transfer workload to target memory in Airbrush */
	memset(&dma_desc, 0, sizeof(dma_desc));
	dma_desc.local_dma_buf_fd = fd;
	dma_desc.local_buf_type = DMA_BUFFER_DMA_BUF;
	dma_desc.remote_buf = remote_addr;
	dma_desc.remote_buf_type = DMA_BUFFER_USER;
	dma_desc.size = size;
	dma_desc.dir = dir;
	return abc_pcie_issue_dma_xfer(&dma_desc);
}

/**
 * Local function to send firmware to Airbrush memory via PCIE
 * @param[in] path Firmware
 * @param[in] remote_addr Address of Airbrush memory
 * @return Status, zero if succeed, non-zero if fail
 */
static int dma_send_fw(struct device *device, const char *path,
		       const int remote_addr)
{
	int err;
	const struct firmware *fw_entry;

	err = request_firmware(&fw_entry, path, device);
	if (err)
		return err;

	err = dma_xfer_vmalloc((void *)fw_entry->data, fw_entry->size,
			       remote_addr, DMA_TO_DEVICE);
	if (err)
		pr_err("Error from abc_pcie_issue_dma_xfer: %d\n", err);
	release_firmware(fw_entry);
	return err;
}

/**
 * Local function to read one DW to Airbrush memory via PCIE
 * @param[in] file File struct of this module
 * @param[in] remote_addr Address of Airbrush memory
 * @param[in] val Variable to store read-back DW
 * @return Status, zero if succeed, non-zero if fail
 */
static int dma_read_dw(const int remote_addr, int *val)
{
	int err = 0;

	err = dma_xfer_vmalloc(val, sizeof(*val), remote_addr, DMA_FROM_DEVICE);
	if (err) {
		pr_err("Error from abc_pcie_issue_dma_xfer: %d\n", err);
		return err;
	}
	return 0;
}

/**
 * Local function to send FaceAuth input data to Airbrush memory via PCIE
 * @param[in] data Data structure copied from user space
 * @return Status, zero if succeed, non-zero if fail
 */
static int dma_send_images(struct faceauth_start_data *data)
{
	int err = 0;

	if (!data->image_dot_left_fd) {
		pr_info("Send left dot image\n");
		err = dma_xfer(data->image_dot_left, data->image_dot_left_size,
			       DOT_LEFT_IMAGE_ADDR, DMA_TO_DEVICE);
		if (err) {
			pr_err("Error sending left dot image\n");
			return err;
		}
	} else {
		pr_info("Send left dot image from ION\n");
		err = dma_xfer_dmabuf(data->image_dot_left_fd,
				      data->image_dot_left_size,
				      DOT_LEFT_IMAGE_ADDR, DMA_TO_DEVICE);
		if (err) {
			pr_err("Error sending left dot image from ION\n");
			return err;
		}
	}
	if (!data->image_dot_right_fd) {
		pr_info("Send right dot image\n");
		err = dma_xfer(data->image_dot_right,
			       data->image_dot_right_size, DOT_RIGHT_IMAGE_ADDR,
			       DMA_TO_DEVICE);
		if (err) {
			pr_err("Error sending right dot image\n");
			return err;
		}
	} else {
		pr_info("Send right dot image from ION\n");
		err = dma_xfer_dmabuf(data->image_dot_right_fd,
				      data->image_dot_right_size,
				      DOT_RIGHT_IMAGE_ADDR, DMA_TO_DEVICE);
		if (err) {
			pr_err("Error sending right dot image from ION\n");
			return err;
		}
	}

	/* This is data to feed individual TPU stages */
	if (!data->image_flood_fd) {
		pr_info("Send flood image\n");
		err = dma_xfer(data->image_flood, data->image_flood_size,
			       FLOOD_IMAGE_ADDR, DMA_TO_DEVICE);
		if (err) {
			pr_err("Error sending flood image\n");
			return err;
		}
	} else {
		pr_info("Send flood image from ION\n");
		err = dma_xfer_dmabuf(data->image_flood_fd,
				      data->image_flood_size, FLOOD_IMAGE_ADDR,
				      DMA_TO_DEVICE);
		if (err) {
			pr_err("Error sending flood image from ION\n");
			return err;
		}
	}

	return err;
}

#if ENABLE_AIRBRUSH_DEBUG
static int dma_gather_debug_log(struct faceauth_debug_data *data)
{
	int err = 0;

	err = dma_xfer((void *)data->debug_buffer,
		       min((uint32_t)PRINTF_LOG_SIZE, data->debug_buffer_size),
		       PRINTF_LOG_ADDR, DMA_FROM_DEVICE);

	return err;
}

static void enqueue_debug_data(struct faceauth_data *data, bool el2)
{
	void *bin_addr;
	int err;

	bin_addr = debug_data_queue.data_buffer +
		   (DEBUG_DATA_BIN_SIZE * debug_data_queue.head_idx);
	if (el2)
		err = el2_gather_debug_data(data->device, bin_addr,
					    DEBUG_DATA_BIN_SIZE);
	else
		err = dma_gather_debug_data(bin_addr, DEBUG_DATA_BIN_SIZE);

	if (err) {
		pr_err("Debug data gathering failed: %d\n", err);
		return;
	}

	debug_data_queue.head_idx =
		(debug_data_queue.head_idx + 1) % DEBUG_DATA_NUM_BINS;

	if (debug_data_queue.count == DEBUG_DATA_NUM_BINS) {
		debug_data_queue.tail_idx =
			(debug_data_queue.tail_idx + 1) % DEBUG_DATA_NUM_BINS;
	} else {
		debug_data_queue.count++;
	}

	do_gettimeofday(&((struct faceauth_debug_entry *)bin_addr)->timestamp);
}

static void move_debug_data_to_tail(void)
{
	int delta;

	if (debug_data_queue.count <= 1)
		return;

	delta = debug_data_queue.count - 1;
	debug_data_queue.count -= delta;
	debug_data_queue.tail_idx =
		(debug_data_queue.tail_idx + delta) % DEBUG_DATA_NUM_BINS;
}

static void clear_debug_data(void)
{
	debug_data_queue.count = 0;
	debug_data_queue.head_idx = 0;
	debug_data_queue.tail_idx = 0;
}

static int dequeue_debug_data(struct faceauth_debug_data *debug_step_data)
{
	void *bin_addr;

	if (debug_data_queue.count == 0) {
		return -ENODATA;
	}

	if (debug_step_data->debug_buffer_size < DEBUG_DATA_BIN_SIZE) {
		return -ENOBUFS;
	}

	bin_addr = debug_data_queue.data_buffer +
		   (DEBUG_DATA_BIN_SIZE * debug_data_queue.tail_idx);
	debug_data_queue.tail_idx =
		(debug_data_queue.tail_idx + 1) % DEBUG_DATA_NUM_BINS;
	debug_data_queue.count--;

	if (copy_to_user((void __user *)(debug_step_data->debug_buffer),
			 bin_addr, DEBUG_DATA_BIN_SIZE)) {
		return -EFAULT;
	}

	return 0;
}

static int dma_gather_debug_data(void *destination_buffer, uint32_t buffer_size)
{
	int err = 0;
	uint32_t internal_state_struct_size;
	struct faceauth_debug_entry *debug_entry = destination_buffer;
	uint32_t current_offset;
	struct faceauth_buffer_list *output_buffers;
	int buffer_idx;
	int buffer_list_size;

	dma_read_dw(INTERNAL_STATE_ADDR +
			    offsetof(struct faceauth_airbrush_state,
				     internal_state_size),
		    &internal_state_struct_size);

	current_offset = offsetof(struct faceauth_debug_entry, ab_state) +
			 internal_state_struct_size;

	if (current_offset + (3 * INPUT_IMAGE_WIDTH * INPUT_IMAGE_HEIGHT) >
	    buffer_size) {
		err = -EINVAL;
		pr_err("");
		return err;
	}

	err = dma_xfer_vmalloc(&debug_entry->ab_state,
			       internal_state_struct_size, INTERNAL_STATE_ADDR,
			       DMA_FROM_DEVICE);
	if (err) {
		pr_err("failed to gather debug data, err %d\n", err);
		return err;
	}

	if (debug_entry->ab_state.command == COMMAND_ENROLL ||
	    debug_entry->ab_state.command == COMMAND_VALIDATE) {
		err = dma_xfer_vmalloc((uint8_t *)debug_entry + current_offset,
				       (INPUT_IMAGE_WIDTH * INPUT_IMAGE_HEIGHT),
				       DOT_LEFT_IMAGE_ADDR, DMA_FROM_DEVICE);
		debug_entry->left_dot.offset_to_image = current_offset;
		debug_entry->left_dot.image_size =
			INPUT_IMAGE_WIDTH * INPUT_IMAGE_HEIGHT;
		current_offset += INPUT_IMAGE_WIDTH * INPUT_IMAGE_HEIGHT;
		if (err) {
			pr_err("Error saving left dot image\n");
			return err;
		}

		err = dma_xfer_vmalloc((uint8_t *)debug_entry + current_offset,
				       INPUT_IMAGE_WIDTH * INPUT_IMAGE_HEIGHT,
				       DOT_RIGHT_IMAGE_ADDR, DMA_FROM_DEVICE);
		debug_entry->right_dot.offset_to_image = current_offset;
		debug_entry->right_dot.image_size =
			INPUT_IMAGE_WIDTH * INPUT_IMAGE_HEIGHT;
		current_offset += INPUT_IMAGE_WIDTH * INPUT_IMAGE_HEIGHT;
		if (err) {
			pr_err("Error saving right dot image\n");
			return err;
		}

		err = dma_xfer_vmalloc((uint8_t *)debug_entry + current_offset,
				       INPUT_IMAGE_WIDTH * INPUT_IMAGE_HEIGHT,
				       FLOOD_IMAGE_ADDR, DMA_FROM_DEVICE);
		debug_entry->flood.offset_to_image = current_offset;
		debug_entry->flood.image_size =
			INPUT_IMAGE_WIDTH * INPUT_IMAGE_HEIGHT;
		current_offset += INPUT_IMAGE_WIDTH * INPUT_IMAGE_HEIGHT;
		if (err) {
			pr_err("Error saving flood image\n");
			return err;
		}
	} else {
		debug_entry->left_dot.offset_to_image = 0;
		debug_entry->left_dot.image_size = 0;
		debug_entry->right_dot.offset_to_image = 0;
		debug_entry->right_dot.image_size = 0;
		debug_entry->flood.offset_to_image = 0;
		debug_entry->flood.image_size = 0;
	}

	output_buffers = &debug_entry->ab_state.output_buffers;
	if (!output_buffers) {
		err = -EMSGSIZE;
		return err;
	}
	buffer_idx = output_buffers->buffer_count - 1;
	if (buffer_idx < 0) {
		return err;
	}
	buffer_list_size =
		output_buffers->buffers[buffer_idx].offset_to_buffer +
		output_buffers->buffers[buffer_idx].size;

	if (buffer_list_size + current_offset > DEBUG_DATA_BIN_SIZE) {
		err = -EMSGSIZE;
		return err;
	}

	if (output_buffers->buffer_base != 0 && buffer_list_size > 0) {
		dma_xfer_vmalloc((uint8_t *)debug_entry + current_offset,
				 buffer_list_size, output_buffers->buffer_base,
				 DMA_FROM_DEVICE);
		output_buffers->buffer_base = current_offset;
		current_offset += buffer_list_size;
	}

	return err;
}
#endif /* #if ENABLE_AIRBRUSH_DEBUG */

/*
 * Local function to write a QW from user space memory to Airbrush via
 * PCIE by PIO.
 * @param[in] remote Airbrush physical address
 * @param[in] QW data (64 bits) to write
 * @return Status, zero if succeed, non-zero if fail
 */
static int pio_write_qw(const int remote_addr, const uint64_t val)
{
	// This has to be performed as two separate 32 bit writes because
	// Lassen's driver has a bug. See drivers/mfd/abc_pcie.c line 368
	uint32_t lower = (uint32_t)val;
	uint32_t upper = (uint32_t)(val >> 32);

	int err = 0;

	err = memory_config_write(remote_addr, 4, lower);

	if (err) {
		pr_err("Error in writing data to Airbrush at address 0x%08x\n",
		       remote_addr);
		return err;
	}

	err = memory_config_write(remote_addr + 4, 4, upper);

	if (err) {
		pr_err("Error in writing data to Airbrush at address 0x%08x\n",
		       remote_addr);
		return err;
	}

	return 0;
}

static void faceauth_link_listener_init(struct work_struct *work)
{
	struct faceauth_data *data =
		container_of(work, struct faceauth_data, listener_init.work);
	int err;

	err = abc_register_pcie_link_blocking_event(
		&data->pcie_link_blocking_nb);

	/* TODO: Use retry count to dynamiclly adjust retry timeout */
	if (err == -EAGAIN) {
		if (data->retry_count % 50 == 0)
			pr_info("Retry faceauth link init later.\n");
		data->retry_count++;
		schedule_delayed_work(&data->listener_init,
				      msecs_to_jiffies(1000));
	} else if (err)
		pr_err("CANNOT init link listened event in faceauth driver, err code %d\n",
		       err);
	else
		pr_info("Successfully register link listener for faceauth driver");
	return;
}

static int faceauth_pcie_blocking_listener(struct notifier_block *nb,
					   unsigned long action, void *data)
{
	struct faceauth_data *dev_data =
		container_of(nb, struct faceauth_data, pcie_link_blocking_nb);

	if (action & ABC_PCIE_LINK_ENTER_EL2) {
		down_read(&dev_data->rwsem);
		if (!dev_data->can_transfer)
			pr_err("ERROR: Wrong state, receive ENTER_EL2 while link down");
		up_read(&dev_data->rwsem);
		return NOTIFY_OK;
	}

	if (action & ABC_PCIE_LINK_EXIT_EL2) {
		down_read(&dev_data->rwsem);
		if (!dev_data->can_transfer)
			pr_err("ERROR: Wrong state, receive EXIT_EL2 while link down");
		up_read(&dev_data->rwsem);
		return NOTIFY_OK;
	}

	if (action & ABC_PCIE_LINK_PRE_DISABLE) {
		/* Use the writer lock to prevent any incoming reader */
		down_write(&dev_data->rwsem);
		dev_data->can_transfer = false;
		pr_info("All ongoing ioctls are finished, confirm disable");
		up_write(&dev_data->rwsem);
		return NOTIFY_OK;
	}

	if (action & ABC_PCIE_LINK_POST_ENABLE) {
		/*
		 * Under this scenerio, this is actually a reader
		 * There's no need to block any other reader since
		 * they'll bail out when they got the value.
		 */
		down_read(&dev_data->rwsem);
		dev_data->can_transfer = true;
		up_read(&dev_data->rwsem);
		return NOTIFY_OK;
	}

	return NOTIFY_DONE;
}

#if IS_ENABLED(CONFIG_DEBUG_FS)

static int faceauth_hypx_enable_set(void *ptr, u64 val)
{
	struct faceauth_data *data = ptr;

	data->hypx_enable = !!val;
	return 0;
}

static int faceauth_hypx_enable_get(void *ptr, u64 *val)
{
	struct faceauth_data *data = ptr;

	*val = data->hypx_enable;
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_hypx_enable, faceauth_hypx_enable_get,
			 faceauth_hypx_enable_set, "%llu\n");

static int faceauth_m0_verbosity_set(void *ptr, u64 val)
{
	struct faceauth_data *data = ptr;

	data->m0_verbosity_level = val;
	return 0;
}

static int faceauth_m0_verbosity_get(void *ptr, u64 *val)
{
	struct faceauth_data *data = ptr;

	*val = data->m0_verbosity_level;
	return 0;
}

DEFINE_DEBUGFS_ATTRIBUTE(fops_m0_verbosity, faceauth_m0_verbosity_get,
			 faceauth_m0_verbosity_set, "0x%016llx\n");

static void faceauth_debugfs_init(struct faceauth_data *data)
{
	struct dentry *debugfs_root;
	struct dentry *hypx, *m0_verbosity_level;
	int err = 0;

	debugfs_root = debugfs_create_dir("faceauth", NULL);
	if (IS_ERR_OR_NULL(debugfs_root)) {
		pr_err("Failed to create faceauth debugfs");
		err = -EIO;
		goto exit;
	}
	data->debugfs_root = debugfs_root;

	hypx = debugfs_create_file("hypx_enable", 0660, debugfs_root, data,
				   &fops_hypx_enable);
	if (!hypx) {
		err = -EIO;
		goto exit;
	}

	m0_verbosity_level =
		debugfs_create_file("m0_verbosity_level", 0660, debugfs_root,
				    data, &fops_m0_verbosity);
	if (!m0_verbosity_level) {
		err = -EIO;
		goto exit;
	}

	return;

exit:
	debugfs_remove_recursive(debugfs_root);
	data->debugfs_root = NULL;

	pr_err("faceauth debugfs initialization failed: %d\n", err);
}

static void faceauth_debugfs_remove(struct faceauth_data *data)
{
	if (!data->debugfs_root)
		return;

	debugfs_remove_recursive(data->debugfs_root);
}

#else /* CONFIG_DEBUG_FS */

static void faceauth_debugfs_init(struct faceauth_data *data)
{
}

static void faceauth_debugfs_remove(struct faceauth_data *data)
{
}

#endif /* CONFIG_DEBUG_FS */

static int faceauth_probe(struct platform_device *pdev)
{
	int err;
	struct faceauth_data *data;

#if ENABLE_AIRBRUSH_DEBUG
	int i;
#endif

	data = devm_kzalloc(&pdev->dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;
	platform_set_drvdata(pdev, data);

	init_rwsem(&data->rwsem);
	data->hypx_enable = true;
	data->device = &pdev->dev;
	data->can_transfer = true;
	data->retry_count = 0;
	data->misc_dev.minor = MISC_DYNAMIC_MINOR,
	data->misc_dev.name = "faceauth",
	data->misc_dev.fops = &faceauth_dev_operations;
	data->pcie_link_blocking_nb.notifier_call =
		faceauth_pcie_blocking_listener;

	INIT_DELAYED_WORK(&data->listener_init, faceauth_link_listener_init);

	schedule_delayed_work(&data->listener_init, msecs_to_jiffies(1000));

	err = misc_register(&data->misc_dev);
	if (err)
		goto exit1;

	faceauth_debugfs_init(data);

	el2_faceauth_probe(data->device);

#if ENABLE_AIRBRUSH_DEBUG
	clear_debug_data();
	debug_data_queue.data_buffer =
		vmalloc(DEBUG_DATA_BIN_SIZE * DEBUG_DATA_NUM_BINS);

	if (debug_data_queue.data_buffer == NULL) {
		err = -ENOMEM;
		goto exit3;
	}

	for (i = 0; i < DEBUG_DATA_NUM_BINS; i++) {
		memset(debug_data_queue.data_buffer + (DEBUG_DATA_BIN_SIZE * i),
		       0, sizeof(struct faceauth_debug_entry));
	}
#endif

	return 0;

exit3:
	faceauth_debugfs_remove(data);
	misc_deregister(&data->misc_dev);

exit1:
	abc_unregister_pcie_link_blocking_event(&data->pcie_link_blocking_nb);
	return err;
}

static int faceauth_remove(struct platform_device *pdev)
{
	struct faceauth_data *data = platform_get_drvdata(pdev);

	el2_faceauth_remove(data->device);
	abc_unregister_pcie_link_blocking_event(&data->pcie_link_blocking_nb);
	misc_deregister(&data->misc_dev);
	faceauth_debugfs_remove(data);

#if ENABLE_AIRBRUSH_DEBUG
	clear_debug_data();
	vfree(debug_data_queue.data_buffer);
#endif

	return 0;
}

static struct platform_driver faceauth_driver = {
	.probe = faceauth_probe,
	.remove = faceauth_remove,
	.driver = {
		.name = "faceauth",
		.owner = THIS_MODULE,
	},
};
struct platform_device *faceauth_pdev;

static int __init faceauth_init(void)
{
	int ret;

	faceauth_pdev =
		platform_device_register_simple("faceauth", -1, NULL, 0);
	if (IS_ERR(faceauth_pdev))
		return PTR_ERR(faceauth_pdev);
	arch_setup_dma_ops(&faceauth_pdev->dev, 0, U64_MAX, NULL, false);

	ret = platform_driver_register(&faceauth_driver);
	if (ret)
		platform_device_unregister(faceauth_pdev);

	return 0;
}
module_init(faceauth_init);

static void __exit faceauth_exit(void)
{
	platform_driver_unregister(&faceauth_driver);
	platform_device_unregister(faceauth_pdev);
}
module_exit(faceauth_exit);

MODULE_AUTHOR("Anatol Pomazau <anatol@google.com>, Lei Liu <leliu@google.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Google FaceAuth driver");
