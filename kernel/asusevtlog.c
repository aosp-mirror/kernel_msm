#include <linux/asusevtlog.h>
#include <linux/proc_fs.h>
#include <linux/fs.h>
#include <linux/time.h>
#include <linux/rtc.h>
#include <linux/list.h>
#include <linux/syscalls.h>
#include <linux/delay.h>
#include <linux/workqueue.h>

char messages[256];

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
	char buffer[256];

	while (suspend_in_progress) {
		msleep(1000);
	}

	if (IS_ERR((const void*)g_hfileEvtlog)) {
		long size;
		{
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
			sprintf(buffer, "\n\n---------------System Boot----%s---------\n", ASUS_SW_VER);

			sys_write(g_hfileEvtlog, buffer, strlen(buffer));
			sys_close(g_hfileEvtlog);
		}
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
