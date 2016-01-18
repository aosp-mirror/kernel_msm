#include <linux/module.h>
#include <linux/reboot.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/slab.h>
#include <linux/sched.h>

#define RESTART_TIMER_TIMEOUT 12


static void restart_timeout(unsigned long data)
{
	pr_emerg("--------------Restart timer timeout----------------\n");
	show_stack((void *) data,NULL);
	pr_emerg("### Show All Blocked State ###\n");
	show_state_filter(TASK_UNINTERRUPTIBLE);
	BUG();
}

int restart_timer_add(struct notifier_block *this,unsigned long code,void *data)
{
	static struct timer_list *restart_timer;
	struct task_struct *tsk;

	tsk = get_current();
	restart_timer = kmalloc(sizeof(struct timer_list),GFP_KERNEL);
	init_timer(restart_timer);
	restart_timer->function = restart_timeout;
	restart_timer->expires = jiffies + HZ * RESTART_TIMER_TIMEOUT;
	restart_timer->data = (unsigned long) tsk;
	add_timer(restart_timer);

	return NOTIFY_DONE;
}

static struct notifier_block restart_notifier ={
	.notifier_call = restart_timer_add,
};

static int __init timer_notifier_add(void)
{
	register_reboot_notifier(&restart_notifier);//add a notifier block in reboot notifier list

	return 0;
}
module_init(timer_notifier_add);
