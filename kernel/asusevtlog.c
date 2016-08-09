#include <linux/asusevtlog.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/list.h>
#include <linux/syscalls.h>
#include <linux/delay.h>
#include <linux/workqueue.h>
#include <linux/init.h>
#include <linux/string.h>
#include <linux/slab.h>

char messages[256];

// Add for power on/off reason +++
u16 warm_reset_value;
u8 power_on_value;
u16 power_off_value;
bool power_on_cold_boot;
bool warm_reset_sid;
bool power_on_sid;
bool power_off_sid;
bool power_on_value_unknown;
bool power_on_value_unable;
bool power_off_value_unknown;
bool power_off_value_unable;
// Add for power on/off reason ---

static struct workqueue_struct *ASUSEvtlog_workQueue;
static void do_write_event_worker(struct work_struct *work);
static DECLARE_WORK(eventLog_Work, do_write_event_worker);

extern int suspend_in_progress;
static int g_hfileEvtlog = -MAX_ERRNO;
static char g_Asus_Eventlog[ASUS_EVTLOG_MAX_ITEM][ASUS_EVTLOG_STR_MAXLEN];
static int g_Asus_Eventlog_read = 0;
static int g_Asus_Eventlog_write = 0;
static struct mutex mA;
static int g_bEventlogEnable = 1;

static void do_write_event_worker(struct work_struct *work)
{
	char buffer[512];
	char *delim = " ";
	char *pch;
	char *cmdline;
	int n = 0;
	struct rtc_time tm;
	struct timespec ts;

	while (suspend_in_progress) {
		msleep(1000);
	}

	if (IS_ERR((const void*)g_hfileEvtlog)) {
		long size;

		g_hfileEvtlog = sys_open(ASUS_EVTLOG_PATH"ASUSEvtlog.txt", O_CREAT|O_RDWR|O_SYNC, 0666);
		if (g_hfileEvtlog < 0) {
			printk("[adbg] 1. open %s failed, err:%d\n", ASUS_EVTLOG_PATH"ASUSEvtlog.txt", g_hfileEvtlog);
			return;
		}

		size = sys_lseek(g_hfileEvtlog, 0, SEEK_END);
		if (size >= SZ_2M) {
			sys_close(g_hfileEvtlog);
			sys_rmdir(ASUS_EVTLOG_PATH"ASUSEvtlog_old.txt");
			sys_rename(ASUS_EVTLOG_PATH"ASUSEvtlog.txt", ASUS_EVTLOG_PATH"ASUSEvtlog_old.txt");
			g_hfileEvtlog = sys_open(ASUS_EVTLOG_PATH"ASUSEvtlog.txt", O_CREAT|O_RDWR|O_SYNC, 0666);
			if (g_hfileEvtlog < 0)
				printk("[adbg] 1. open %s failed during renaming old one, err:%d\n", ASUS_EVTLOG_PATH"ASUSEvtlog.txt", g_hfileEvtlog);
		}
		n = sprintf(buffer, "\n\n---------------System Boot----%s---------\n", ASUS_SW_VER);

		// Add for power on/off reason +++
		if (warm_reset_value) {
			n += sprintf(buffer+n, "[Reboot] Warm reset Reason: ");
			if (warm_reset_sid) {
				n += sprintf(buffer+n, "0x%04x => ", warm_reset_value);
				if (warm_reset_value & 0x0001) {
					n += sprintf(buffer+n, "[Soft reset];");
				}
				if (warm_reset_value & 0x0002) {
					n += sprintf(buffer+n, "[Reset via PS_HOLD];");
				}
				if (warm_reset_value & 0x0004) {
					n += sprintf(buffer+n, "[PMIC Watchdog];");
				}
				if (warm_reset_value & 0x0008) {
					n += sprintf(buffer+n, "[KPD_RES1];");
				}
				if (warm_reset_value & 0x0010) {
					n += sprintf(buffer+n, "[KPD_RES2];");
				}
				if (warm_reset_value & 0x0020) {
					n += sprintf(buffer+n, "[KPDPWR+RESIN];");
				}
				if (warm_reset_value & 0x0040) {
					n += sprintf(buffer+n, "[RESIN];");
				}
				if (warm_reset_value & 0x0080) {
					n += sprintf(buffer+n, "[Power key];");
				}
				if (warm_reset_value & 0x1000) {
					n += sprintf(buffer+n, "[TFT];");
				}
				n += sprintf(buffer+n, " (last time)");
			}

			n += sprintf(buffer+n, " ###### \n###### Bootup Reason: ");

			if (power_on_value_unable) {
				n += sprintf(buffer+n, "Unable to read PON_RESASON1 reg");
			} else {
				if (power_on_value_unknown) {
					n += sprintf(buffer+n, "0x%04x => ", power_on_value);
					n += sprintf(buffer+n, "[Unknown];");
					n += sprintf(buffer+n, " (%s boot)", power_on_cold_boot ? "cold" : "warm");
				} else if (power_on_sid) {
					n += sprintf(buffer+n, "0x%04x => ", power_on_value);
					if (power_on_value & 0x01) {
						n += sprintf(buffer+n, "[Hard reset];");
					}
					if (power_on_value & 0x02) {
						n += sprintf(buffer+n, "[Suddenly Power Loss];");
					}
					if (power_on_value & 0x04) {
						n += sprintf(buffer+n, "[RTC];");
					}
					if (power_on_value & 0x08) {
						n += sprintf(buffer+n, "[DC];");
					}
					if (power_on_value & 0x10) {
						n += sprintf(buffer+n, "[USB];");
					}
					if (power_on_value & 0x20) {
						n += sprintf(buffer+n, "[PON1];");
					}
					if (power_on_value & 0x40) {
						n += sprintf(buffer+n, "[CBLPWR];");
					}
					if (power_on_value & 0x80) {
						n += sprintf(buffer+n, "[Power Key];");
					}
					n += sprintf(buffer+n, " (%s boot)", power_on_cold_boot ? "cold" : "warm");
				}
			}
			n += sprintf(buffer+n, " ######\n");
		} else {
			n += sprintf(buffer+n, "[Shutdown] Power off Reason: ");
			if (power_off_value_unable) {
				n += sprintf(buffer+n, "Unable to read POFF_RESASON regs");
			} else {
				if (power_off_value_unknown) {
					n += sprintf(buffer+n, "0x%04x => ", power_off_value);
					n += sprintf(buffer+n, "[Unknown];");
					n += sprintf(buffer+n, " (last time)");
				} else if (power_off_sid) {
					n += sprintf(buffer+n, "0x%04x => ", power_off_value);
					if (power_off_value & 0x0001) {
						n += sprintf(buffer+n, "[Soft reset];");
					}
					if (power_off_value & 0x0002) {
						n += sprintf(buffer+n, "[Reset via PS_HOLD];");
					}
					if (power_off_value & 0x0004) {
						n += sprintf(buffer+n, "[PMIC Watchdog];");
					}
					if (power_off_value & 0x0008) {
						n += sprintf(buffer+n, "[Battery lost];");
					}
					if (power_off_value & 0x0010) {
						n += sprintf(buffer+n, "[KPD_RES2];");
					}
					if (power_off_value & 0x0020) {
						n += sprintf(buffer+n, "[KPDPWR+RESIN];");
					}
					if (power_off_value & 0x0040) {
						n += sprintf(buffer+n, "[RESIN];");
					}
					if (power_off_value & 0x0080) {
						n += sprintf(buffer+n, "[Power key];");
					}
					if (power_off_value & 0x0400) {
						n += sprintf(buffer+n, "[AVDD_RB];");
					}
					if (power_off_value & 0x0800) {
						n += sprintf(buffer+n, "[Charger];");
					}
					if (power_off_value & 0x1000) {
						n += sprintf(buffer+n, "[TFT];");
					}
					if (power_off_value & 0x2000) {
						n += sprintf(buffer+n, "[UVLO];");
					}
					if (power_off_value & 0x4000) {
						n += sprintf(buffer+n, "[PMIC Overtemp];");
					}
					if (power_off_value & 0x8000) {
						n += sprintf(buffer+n, "[Fail safe reset];");
					}
					n += sprintf(buffer+n, " (last time)");
				}

				n += sprintf(buffer+n, " ###### \n###### Bootup Reason: ");

				if (power_on_value_unable) {
					n += sprintf(buffer+n, "Unable to read PON_RESASON1 reg");
				} else {
					if (power_on_value_unknown) {
						n += sprintf(buffer+n, "0x%04x => ", power_on_value);
						n += sprintf(buffer+n, "[Unknown];");
						n += sprintf(buffer+n, " (%s boot)", power_on_cold_boot ? "cold" : "warm");
					} else if (power_on_sid) {
						n += sprintf(buffer+n, "0x%04x => ", power_on_value);
						if (power_on_value & 0x01) {
							n += sprintf(buffer+n, "[Hard reset];");
						}
						if (power_on_value & 0x02) {
							n += sprintf(buffer+n, "[Suddenly Power Loss];");
						}
						if (power_on_value & 0x04) {
							n += sprintf(buffer+n, "[RTC];");
						}
						if (power_on_value & 0x08) {
							n += sprintf(buffer+n, "[DC];");
						}
						if (power_on_value & 0x10) {
							n += sprintf(buffer+n, "[USB];");
						}
						if (power_on_value & 0x20) {
							n += sprintf(buffer+n, "[PON1];");
						}
						if (power_on_value & 0x40) {
							n += sprintf(buffer+n, "[CBLPWR];");
						}
						if (power_on_value & 0x80) {
							n += sprintf(buffer+n, "[Power Key];");
						}
						n += sprintf(buffer+n, " (%s boot)", power_on_cold_boot ? "cold" : "warm");
					}
				}
				n += sprintf(buffer+n, " ######\n");
			}
		}
		// Add for power on/off reason ---

		getnstimeofday(&ts);
		ts.tv_sec -= sys_tz.tz_minuteswest * 60;
		rtc_time_to_tm(ts.tv_sec, &tm);
		get_monotonic_boottime(&ts);
		n += sprintf(buffer+n, "(%ld)%04d-%02d-%02d %02d:%02d:%02d :",ts.tv_sec,tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

		cmdline = kstrdup(saved_command_line, GFP_KERNEL);

		if (cmdline != NULL) {
			while ((pch = strsep(&cmdline,delim)) != NULL) {
				if (strnstr(pch, "androidboot.bootreason=",strlen(pch))) {
					n += sprintf(buffer+n, "[cmdline] %s\n", pch);
				}
			}
		} else {
			printk("[adbg] cmdline is NULL\n");
		}

		sys_write(g_hfileEvtlog, buffer, strlen(buffer));
		sys_close(g_hfileEvtlog);
		kfree(cmdline);

	}
	if (!IS_ERR((const void*)g_hfileEvtlog)) {
		int str_len;
		char* pchar;
		long size;

		g_hfileEvtlog = sys_open(ASUS_EVTLOG_PATH"ASUSEvtlog.txt", O_CREAT|O_RDWR|O_SYNC, 0666);
		if (g_hfileEvtlog < 0) {
			printk("[adbg] 2. open %s failed, err:%d\n", ASUS_EVTLOG_PATH"ASUSEvtlog.txt", g_hfileEvtlog);
			return;
		}

		size = sys_lseek(g_hfileEvtlog, 0, SEEK_END);
		if (size >= SZ_2M) {
			sys_close(g_hfileEvtlog);
			sys_rmdir(ASUS_EVTLOG_PATH"ASUSEvtlog_old.txt");
			sys_rename(ASUS_EVTLOG_PATH"ASUSEvtlog.txt", ASUS_EVTLOG_PATH"ASUSEvtlog_old.txt");
			g_hfileEvtlog = sys_open(ASUS_EVTLOG_PATH"ASUSEvtlog.txt", O_CREAT|O_RDWR|O_SYNC, 0666);
			if (g_hfileEvtlog < 0)
				printk("[adbg] 2. open %s failed during renaming old one, err:%d\n", ASUS_EVTLOG_PATH"ASUSEvtlog.txt", g_hfileEvtlog);
		}

		while (g_Asus_Eventlog_read != g_Asus_Eventlog_write) {
			mutex_lock(&mA);

			str_len = strlen(g_Asus_Eventlog[g_Asus_Eventlog_read]);
			pchar = g_Asus_Eventlog[g_Asus_Eventlog_read];
			g_Asus_Eventlog_read ++;
			g_Asus_Eventlog_read %= ASUS_EVTLOG_MAX_ITEM;
			mutex_unlock(&mA);

			if (pchar[str_len - 1] != '\n') {
				pchar[str_len] = '\n';
				pchar[str_len + 1] = '\0';
			}
			while (suspend_in_progress) {
				msleep(1000);
			}
			sys_write(g_hfileEvtlog, pchar, strlen(pchar));
			sys_fsync(g_hfileEvtlog);
			printk(pchar);
		}
		sys_close(g_hfileEvtlog);
	}
}

void ASUSEvtlog(const char *fmt, ...)
{

	va_list args;
	char* buffer;

	if (g_bEventlogEnable == 0)
		return;

	if (!in_interrupt() && !in_atomic() && !irqs_disabled())
		mutex_lock(&mA);//spin_lock(&spinlock_eventlog);

	buffer = g_Asus_Eventlog[g_Asus_Eventlog_write];

	g_Asus_Eventlog_write ++;
	g_Asus_Eventlog_write %= ASUS_EVTLOG_MAX_ITEM;

	if (!in_interrupt() && !in_atomic() && !irqs_disabled())
		mutex_unlock(&mA);//spin_unlock(&spinlock_eventlog);

	memset(buffer, 0, ASUS_EVTLOG_STR_MAXLEN);

	if (buffer) {
		struct rtc_time tm;
		struct timespec ts;

		getnstimeofday(&ts);
		ts.tv_sec -= sys_tz.tz_minuteswest * 60; // to get correct timezone information
		rtc_time_to_tm(ts.tv_sec, &tm);
		get_monotonic_boottime(&ts);
		sprintf(buffer, "(%ld)%04d-%02d-%02d %02d:%02d:%02d :",ts.tv_sec,tm.tm_year + 1900, tm.tm_mon + 1, tm.tm_mday, tm.tm_hour, tm.tm_min, tm.tm_sec);

		va_start(args, fmt);
		vscnprintf(buffer + strlen(buffer), ASUS_EVTLOG_STR_MAXLEN - strlen(buffer), fmt, args);
		va_end(args);

		queue_work(ASUSEvtlog_workQueue, &eventLog_Work);
	} else
		printk("[adbg] ASUSEvtlog buffer cannot be allocated\n");
}
EXPORT_SYMBOL(ASUSEvtlog);

static ssize_t evtlogswitch_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	if (strncmp(buf, "0", 1) == 0) {
		ASUSEvtlog("ASUSEvtlog disable !!");
		printk("[adbg] ASUSEvtlog disable !!\n");
		g_bEventlogEnable = 0;
	}
	if (strncmp(buf, "1", 1) == 0) {
		g_bEventlogEnable = 1;
		ASUSEvtlog("ASUSEvtlog enable !!");
		printk("[adbg] ASUSEvtlog enable !!\n");
	}

	return count;
}

static ssize_t asusevtlog_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	if (count > 256)
		count = 256;

	memset(messages, 0, sizeof(messages));
	if (copy_from_user(messages, buf, count))
		return -EFAULT;
	ASUSEvtlog(messages);

	return count;
}

static const struct file_operations proc_evtlogswitch_operations = {
	.write      = evtlogswitch_write,
};

static const struct file_operations proc_asusevtlog_operations = {
	.write      = asusevtlog_write,
};

static int __init proc_asusevtlog_init(void)
{
	proc_create("asusevtlog", S_IRWXUGO, NULL, &proc_asusevtlog_operations);
	proc_create("asusevtlog-switch", S_IRWXUGO, NULL, &proc_evtlogswitch_operations);
	mutex_init(&mA);
	ASUSEvtlog_workQueue  = create_singlethread_workqueue("ASUSEVTLOG_WORKQUEUE");
	printk("[adbg] ASUSEvtlog init\n");
	return 0;
}
module_init(proc_asusevtlog_init);
