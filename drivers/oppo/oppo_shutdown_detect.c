/**********************************************************************************
* Copyright (c)  2008-2020  Guangdong OPPO Mobile Comm Corp., Ltd
*
* Description:     shutdown_detect Monitor  Kernel Driver
*
* Version   : 1.2
* Date       : 2020-04-21
* Author     : Leefong@WSW.BSP.Stability
* ------------------------------ Revision History: --------------------------------
* <version>           <date>                <author>                            <desc>
* Revision 1.0        2010-01-05       wen.luo@PSW.Kernel.Stability       Created for shutdown_detect Monitor
* Revision 1.1        2019-04-28       liang.zhang@TECH.Storage.Stability modify shutdown_detect and add log back
* Revision 1.2        2020-04-21       Leefong.chen@WSW.BSP.Stability modify shutdown_detect for wear os
***********************************************************************************/

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/sched.h>
#include <linux/sched/rt.h>
#include <linux/interrupt.h>
#include <linux/mm.h>
#include <linux/fs.h>
#include <linux/mount.h>
#include <linux/kdev_t.h>
#include <linux/major.h>
#include <linux/reboot.h>
#include <linux/sysrq.h>
#include <linux/kbd_kern.h>
#include <linux/proc_fs.h>
#include <linux/nmi.h>
#include <linux/quotaops.h>
#include <linux/perf_event.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/suspend.h>
#include <linux/writeback.h>
#include <linux/swap.h>
#include <linux/spinlock.h>
#include <linux/vt_kern.h>
#include <linux/workqueue.h>
#include <linux/hrtimer.h>
#include <linux/delay.h>
#include <linux/oom.h>
#include <linux/slab.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/moduleparam.h>
#include <linux/jiffies.h>
#include <linux/syscalls.h>
#include <linux/of.h>
#include <linux/rcupdate.h>
#include <linux/kthread.h>

#include <asm/ptrace.h>
#include <asm/irq_regs.h>

#include <linux/sysrq.h>
#include <linux/clk.h>

#include <linux/kmsg_dump.h>

#include <include/oppo_project.h>

#define SEQ_printf(m, x...)     \
    do {                        \
        if (m)                  \
            seq_printf(m, x);   \
        else                    \
            pr_debug(x);        \
    } while (0)

#define OPPO_SHUTDOWN_LOG_START_BLOCK_EMMC  10240
#define OPPO_SHUTDOWN_LOG_START_BLOCK_UFS   1280
#define OPPO_SHUTDOWN_KERNEL_LOG_SIZE_BYTES 1024 * 1024
#define OPPO_SHUTDOWN_FLAG_OFFSET           0 * 1024 * 1024
#define OPPO_SHUTDOWN_KMSG_OFFSET           29 * 1024 * 1024
#define FILE_MODE_0666                      0666

#define BLOCK_SIZE_EMMC                     512
#define BLOCK_SIZE_UFS                      4096

#define SHUTDOWN_MAGIC_LEN                  8

#define ShutDownTO                          0x9B

#define TASK_INIT_COMM                      "init"

#define OPPO_PARTITION_OPPORESERVE3_LINK    "/dev/block/by-name/opporeserve3"

#define SIG_SHUTDOWN                        (SIGRTMIN + 0x12)

#define SHUTDOWN_STAGE_KERNEL               20
#define SHUTDOWN_STAGE_INIT                 30
#define SHUTDOWN_STAGE_SYSTEMSERVER         40
#define SHUTDOWN_TIMEOUNT_UMOUNT            31
#define SHUTDOWN_TIMEOUNT_VOLUME            32
#define SHUTDOWN_TIMEOUNT_SUBSYSTEM         43
#define SHUTDOWN_TIMEOUNT_RADIOS            44
#define SHUTDOWN_TIMEOUNT_PM                45
#define SHUTDOWN_TIMEOUNT_AM                46
#define SHUTDOWN_TIMEOUNT_BC                47
#define SHUTDOWN_STAGE_INIT_POFF            70
#define SHUTDOWN_RUS_MIN                    255
#define SHUTDOWN_TOTAL_TIME_MIN             60
#define SHUTDOWN_DEFAULT_NATIVE_TIME        15
#define SHUTDOWN_DEFAULT_JAVA_TIME          15
#define SHUTDOWN_DEFAULT_TOTAL_TIME         90
#define SHUTDOWN_INCREASE_TIME              5

#define KE_LOG_COLLECT_TIMEOUT              msecs_to_jiffies(10000)

static struct kmsg_dumper shutdown_kmsg_dumper;

static DECLARE_COMPLETION(shd_comp);
static DEFINE_MUTEX(shd_wf_mutex);

static unsigned int shutdown_phase;
static bool shutdown_detect_started = false;
static bool shutdown_detect_enable = true;
static bool is_shutdows = false;
static unsigned int gtimeout = 0;
static unsigned int gtotaltimeout = SHUTDOWN_DEFAULT_TOTAL_TIME;
static unsigned int gjavatimeout = SHUTDOWN_DEFAULT_JAVA_TIME;
static unsigned int gnativetimeout = SHUTDOWN_DEFAULT_NATIVE_TIME;

static struct task_struct *shutdown_task = NULL;
struct task_struct *shd_complete_monitor = NULL;

struct shd_info
{
    char magic[SHUTDOWN_MAGIC_LEN];
    int  shutdown_err;
    int  shutdown_times;
};

#define SIZEOF_STRUCT_SHD_INFO sizeof(struct shd_info)

time_t shutdown_start_time = 0;
time_t shutdown_end_time = 0;
time_t shutdown_systemserver_start_time = 0;
time_t shutdown_init_start_time = 0;
time_t shutdown_kernel_start_time = 0;

static int shutdown_kthread(void *data){
    kernel_power_off();
    return 0;
}

static int shutdown_detect_func(void *dummy);

static void shutdown_timeout_flag_write(int timeout);
static void shutdown_dump_kernel_log(void);
static int shutdown_timeout_flag_write_now(void *args);

extern int creds_change_dac(void);
extern int shutdown_kernel_log_save(void *args);
extern void shutdown_dump_android_log(void);

static ssize_t shutdown_detect_trigger(struct file *filp, const char *ubuf, size_t cnt, loff_t *data)
{
    char buf[64];
    long val = 0;
    int ret = 0;
    struct task_struct *tsk = NULL;
    unsigned int temp = SHUTDOWN_DEFAULT_TOTAL_TIME;

    if(shutdown_detect_enable == false) {
        return -EPERM;
    }

    if (cnt >= sizeof(buf)) {
        return -EINVAL;
    }

    if (copy_from_user(&buf, ubuf, cnt)) {
        return -EFAULT;
    }

    buf[cnt] = 0;

    ret = kstrtoul(buf, 0, (unsigned long *)&val);

    if (ret < 0) {
        return ret;
    }

    if (val == SHUTDOWN_STAGE_INIT_POFF) {
         is_shutdows = true;
         val = SHUTDOWN_STAGE_INIT;
    }
    gnativetimeout += SHUTDOWN_INCREASE_TIME;
    gjavatimeout += SHUTDOWN_INCREASE_TIME;
/*wen.luo@PSW.BSP.Kernel.Stability. 2019/07/11, modify for show the murderer*/
    tsk = current->group_leader;
    pr_info("%s:%d shutdown_detect, GroupLeader is %s:%d\n", current->comm, task_pid_nr(current), tsk->comm, task_pid_nr(tsk));
    //val: 0x gtotaltimeout|gjavatimeout|gnativetimeout , gnativetimeout < F, gjavatimeout < F
    if (val > SHUTDOWN_RUS_MIN) {
         gnativetimeout = val % 16;
         gjavatimeout = ((val - gnativetimeout) % 256 ) / 16;
         temp = val / 256;
         gtotaltimeout = (temp < SHUTDOWN_TOTAL_TIME_MIN) ? SHUTDOWN_TOTAL_TIME_MIN : temp;// for safe
         pr_info("shutdown_detect_trigger rus val %ld %d %d %d\n", val, gnativetimeout, gjavatimeout, gtotaltimeout);
         return cnt;
    }

    //pr_err("shutdown_detect_trigger final val %ld %d %d %d\n", val, gnativetimeout, gjavatimeout, gtotaltimeout);

    switch (val) {
    case 0:
        if (shutdown_detect_started) {
            shutdown_detect_started = false;
            shutdown_phase = 0;
        }
        shutdown_detect_enable = false;
        pr_err("shutdown_detect: abort shutdown detect\n");
        break;
    case SHUTDOWN_STAGE_KERNEL:
        shutdown_kernel_start_time = current_kernel_time().tv_sec;
        pr_info("shutdown_kernel_start_time %ld\n", shutdown_kernel_start_time);
        if((shutdown_init_start_time - shutdown_kernel_start_time) > gnativetimeout) {
            pr_err("shutdown_detect_timeout: timeout happened in reboot.cpp\n");
            shutdown_dump_kernel_log();
            shutdown_timeout_flag_write(1);
        } else {
            if(((shutdown_init_start_time - shutdown_systemserver_start_time) > gjavatimeout) && shutdown_systemserver_start_time) {
                // timeout happend in system_server stage
                shutdown_timeout_flag_write(1);
            }
        }
        shutdown_phase = val;
        pr_err("shutdown_detect_phase: shutdown  current phase systemcall\n");
        break;
    case SHUTDOWN_STAGE_INIT:
        if (!shutdown_detect_started) {
            shutdown_detect_started = true;
            shutdown_init_start_time = current_kernel_time().tv_sec;
            shutdown_start_time = shutdown_init_start_time;
            shd_complete_monitor = kthread_run(shutdown_detect_func, NULL, "shutdown_detect_thread");
            if (IS_ERR(shd_complete_monitor)) {
                ret = PTR_ERR(shd_complete_monitor);
                pr_err("shutdown_detect: cannot start thread: %d\n", ret);
            }

        } else {
            shutdown_init_start_time = current_kernel_time().tv_sec;

            if((shutdown_init_start_time - shutdown_systemserver_start_time) > gjavatimeout) {
                pr_err("shutdown_detect_timeout: timeout happened in system_server stage\n");
                shutdown_dump_android_log();
                shutdown_dump_kernel_log();
            }
        }
        //pr_err("shutdown_init_start_time %ld\n", shutdown_init_start_time);
        shutdown_phase = val;
        pr_err("shutdown_detect_phase: shutdown  current phase init\n");
        break;
    case SHUTDOWN_TIMEOUNT_UMOUNT:
        pr_err("shutdown_detect_timeout: umount timeout\n");
        break;
    case SHUTDOWN_TIMEOUNT_VOLUME:
        pr_err("shutdown_detect_timeout: volume shutdown timeout\n");
        break;
    case SHUTDOWN_STAGE_SYSTEMSERVER:
        shutdown_systemserver_start_time = current_kernel_time().tv_sec;

        //pr_err("shutdown_systemserver_start_time %ld\n", shutdown_systemserver_start_time);
        if (!shutdown_detect_started) {
            shutdown_detect_started = true;
            shutdown_start_time = shutdown_systemserver_start_time;
            shd_complete_monitor = kthread_run(shutdown_detect_func, NULL, "shutdown_detect_thread");
        }
        shutdown_phase = val;
        pr_err("shutdown_detect_phase: shutdown  current phase systemserver\n");
        break;
	case SHUTDOWN_TIMEOUNT_SUBSYSTEM:
		pr_err("shutdown_detect_timeout: ShutdownSubSystem timeout\n");
		break;
    case SHUTDOWN_TIMEOUNT_RADIOS:
        pr_err("shutdown_detect_timeout: ShutdownRadios timeout\n");
        break;
    case SHUTDOWN_TIMEOUNT_PM:
        pr_err("shutdown_detect_timeout: ShutdownPackageManager timeout\n");
        break;
    case SHUTDOWN_TIMEOUNT_AM:
        pr_err("shutdown_detect_timeout: ShutdownActivityManager timeout\n");
        break;
    case SHUTDOWN_TIMEOUNT_BC:
        pr_err("shutdown_detect_timeout: SendShutdownBroadcast timeout\n");
        break;
    default:
        break;
    }
    if(!shutdown_task && is_shutdows) {
        shutdown_task = kthread_create(shutdown_kthread, NULL,"shutdown_kthread");
        if (IS_ERR(shutdown_task)) {
            pr_err("create shutdown thread fail, will BUG()\n");
            msleep(60*1000);
            BUG();
        }
    }
    return cnt;
}

static int shutdown_detect_show(struct seq_file *m, void *v)
{
    SEQ_printf(m, "=== shutdown_detect controller ===\n");
    SEQ_printf(m, "0:   shutdown_detect abort\n");
    SEQ_printf(m, "20:   shutdown_detect systemcall reboot phase\n");
    SEQ_printf(m, "30:   shutdown_detect init reboot phase\n");
    SEQ_printf(m, "40:   shutdown_detect system server reboot phase\n");
    SEQ_printf(m, "=== shutdown_detect controller ===\n\n");
    SEQ_printf(m, "shutdown_detect: shutdown phase: %u\n", shutdown_phase);
    return 0;
}

static int shutdown_detect_open(struct inode *inode, struct file *file)
{
    return single_open(file, shutdown_detect_show, inode->i_private);
}

static const struct file_operations shutdown_detect_fops = {
    .open        = shutdown_detect_open,
    .write       = shutdown_detect_trigger,
    .read        = seq_read,
    .llseek      = seq_lseek,
    .release     = single_release,
};

static int dump_kmsg(const char * filepath, size_t offset_of_start, struct kmsg_dumper *kmsg_dumper)
{
    mm_segment_t old_fs;
    char line[1024] = {0};
    int file_fd = -1;
    size_t len = 0;
    int result = -1;
    size_t bytes_writed = 0;

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    file_fd = sys_open(filepath, O_CREAT | O_WRONLY | O_TRUNC, FILE_MODE_0666);
    if (file_fd < 0)
    {
        pr_err("sys_open %s failed, error: %d", filepath, file_fd);
        result = -1;
        goto shd_fail;
    }
    sys_lseek(file_fd, offset_of_start, SEEK_SET);
    kmsg_dumper->active = true;
    while (kmsg_dump_get_line(kmsg_dumper, true, line, sizeof(line), &len)) {
        line[len] = '\0';
        mutex_lock(&shd_wf_mutex);
        bytes_writed = sys_write(file_fd, line, len);
        if(len != bytes_writed)
        {
            mutex_unlock(&shd_wf_mutex);
            result = -1;
            goto shd_fail;
        }
        mutex_unlock(&shd_wf_mutex);
    }
    result = 0;

shd_fail:
    set_fs(old_fs);
    if (file_fd >= 0)
    {
        sys_close(file_fd);
        sys_sync();
    }
    return result;
}

int shutdown_kernel_log_save(void *args)
{
    if(0 != dump_kmsg(OPPO_PARTITION_OPPORESERVE3_LINK, OPPO_SHUTDOWN_KMSG_OFFSET, &shutdown_kmsg_dumper))
    {
        pr_err("dump kmsg failed\n");
        complete(&shd_comp);
        return -1;
    }

    complete(&shd_comp);
    return 1;
}

static int shutdown_timeout_flag_write_now(void *args)
{
    struct file *opfile;
    ssize_t size;
    loff_t offsize;
    char data_info[SIZEOF_STRUCT_SHD_INFO] = {'\0'};
    mm_segment_t old_fs;
    int rc;
    struct shd_info shutdown_flag;

    opfile = filp_open(OPPO_PARTITION_OPPORESERVE3_LINK, O_RDWR, 0600);

    if (IS_ERR(opfile)) {
        pr_err("open OPPO_PARTITION_OPPORESERVE3_LINK error: %ld\n",PTR_ERR(opfile));
        complete(&shd_comp);
        return -1;
    }

    offsize = OPPO_SHUTDOWN_FLAG_OFFSET;

    old_fs = get_fs();
    set_fs(KERNEL_DS);

    strncpy(shutdown_flag.magic, "ShutDown", SHUTDOWN_MAGIC_LEN);
    if(gtimeout) {
        shutdown_flag.shutdown_err = ShutDownTO;
    } else {
        shutdown_flag.shutdown_err = 0;
    }

    shutdown_flag.shutdown_times = (int)(shutdown_end_time - shutdown_start_time);

    memcpy(data_info, &shutdown_flag, SIZEOF_STRUCT_SHD_INFO);

    size = vfs_write(opfile, data_info, SIZEOF_STRUCT_SHD_INFO, &offsize);

    if (size < 0) {
         pr_err("vfs_write data_info %s size %d \n", data_info, size);
         set_fs(old_fs);
         filp_close(opfile,NULL);
         complete(&shd_comp);
         return -1;
    }
    rc = vfs_fsync(opfile, 1);
    if (rc)
        pr_err("sync returns %d\n", rc);

    set_fs(old_fs);
    filp_close(opfile,NULL);
    pr_info("shutdown_timeout_flag_write_now done \n");
    complete(&shd_comp);

    return 0;
}

static void task_comm_to_struct(const char * pcomm, struct task_struct ** t_result)
{
    struct task_struct *g, *t;
    rcu_read_lock();
    for_each_process_thread(g, t)
    {
        if(!strcmp(t->comm, pcomm))
        {
            *t_result = t;
            rcu_read_unlock();
            return;
        }
    }
    t_result = NULL;
    rcu_read_unlock();
}

void shutdown_dump_android_log(void)
{
    struct task_struct *sd_init;
    sd_init = NULL;
    task_comm_to_struct(TASK_INIT_COMM, &sd_init);
    if(NULL != sd_init)
    {
        pr_err("send shutdown_dump_android_log signal %d", SIG_SHUTDOWN);
        send_sig(SIG_SHUTDOWN, sd_init, 0);
        pr_err("after send shutdown_dump_android_log signal %d", SIG_SHUTDOWN);
        // wait to collect shutdown log finished
        schedule_timeout_interruptible(20 * HZ);
    }
}

static void shutdown_dump_kernel_log(void)
{
    struct task_struct *tsk;
    tsk = kthread_run(shutdown_kernel_log_save, NULL, "shd_collect_dmesg");
    if(IS_ERR(tsk))
    {
        pr_err("create kernel thread shd_collect_dmesg failed\n");
        return;
    }
    // wait max 10s to collect shutdown log finished
    if(!wait_for_completion_timeout(&shd_comp, KE_LOG_COLLECT_TIMEOUT))
    {
        pr_err("collect kernel log timeout\n");
    }
}

static void shutdown_timeout_flag_write(int timeout)
{
    struct task_struct *tsk;

    gtimeout = timeout;

    shutdown_end_time = current_kernel_time().tv_sec;

    tsk = kthread_run(shutdown_timeout_flag_write_now, NULL, "shd_to_flag");
    if(IS_ERR(tsk))
    {
        pr_err("create kernel thread shd_to_flag failed\n");
        return;
    }
    // wait max 10s to collect shutdown log finished
    if(!wait_for_completion_timeout(&shd_comp, KE_LOG_COLLECT_TIMEOUT))
    {
        pr_err("shutdown_timeout_flag_write timeout\n");
    }
}

static int shutdown_detect_func(void *dummy)
{
    //schedule_timeout_uninterruptible(gtotaltimeout * HZ);
    msleep(gtotaltimeout * 1000);

    pr_info("shutdown_detect:%s call sysrq show block and cpu thread. BUG\n", __func__);
    handle_sysrq('w');
    handle_sysrq('l');
    pr_info("shutdown_detect:%s shutdown_detect status:%u. \n", __func__, shutdown_phase);

    if(shutdown_phase >= SHUTDOWN_STAGE_INIT) {
        shutdown_dump_android_log();
    }

    shutdown_dump_kernel_log();

    shutdown_timeout_flag_write(1);// timeout happened

        if(is_shutdows){
            pr_err("shutdown_detect: shutdown or reboot? shutdown\n");
            if(shutdown_task) {
                wake_up_process(shutdown_task);
            }
        }else{
            pr_err("shutdown_detect: shutdown or reboot? reboot\n");
            BUG();
        }
    return 0;
}

static int __init init_shutdown_detect_ctrl(void)
{
    struct proc_dir_entry *pe;
    pr_info("shutdown_detect:register shutdown_detect interface\n");
    pe = proc_create("shutdown_detect", 0664, NULL, &shutdown_detect_fops);
    if (!pe) {
        pr_err("shutdown_detect:Failed to register shutdown_detect interface\n");
        return -ENOMEM;
    }
    return 0;
}

device_initcall(init_shutdown_detect_ctrl);

