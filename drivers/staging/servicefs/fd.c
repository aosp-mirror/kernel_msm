#include <linux/file.h>
#include <linux/fdtable.h>
#include <linux/bitops.h>
#include <linux/interrupt.h>
#include <linux/spinlock.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>

/*
 * copied from task_get_unused_fd_flags/alloc_fd
 */
int servicefs_get_unused_fd_flags(struct task_struct *task, int flags)
{
	struct files_struct *files = task->files;
	unsigned long rlim_cur;
	unsigned long irqs;

	if (files == NULL)
		return -ESRCH;

	if (!lock_task_sighand(task, &irqs))
		return -EMFILE;

	rlim_cur = task_rlimit(task, RLIMIT_NOFILE);
	unlock_task_sighand(task, &irqs);

	return __alloc_fd(files, 0, rlim_cur, flags);
}

/*
 * copied from fd_install
 */
void servicefs_fd_install(struct task_struct *task, unsigned int fd,
		struct file *file)
{
	if (task->files)
		__fd_install(task->files, fd, file);
}

/*
 * copied from fget
 */
struct file *servicefs_fget(struct task_struct *task, unsigned int fd)
{
	struct file *file;
	struct files_struct *files = task->files;

	rcu_read_lock();
	file = fcheck_files(files, fd);
	if (file) {
		/* File object ref couldn't be taken */
		if (file->f_mode & FMODE_PATH ||
		    !atomic_long_inc_not_zero(&file->f_count))
			file = NULL;
	}
	rcu_read_unlock();

	return file;
}

