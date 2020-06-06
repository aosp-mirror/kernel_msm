#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <asm/setup.h>

unsigned int oppo_ftm_mode;
unsigned int oppo_hw_id;
unsigned int oppo_board_id;
unsigned int oppo_panel_id;
unsigned int panel_present = 1;

EXPORT_SYMBOL(oppo_ftm_mode);
EXPORT_SYMBOL(oppo_hw_id);
EXPORT_SYMBOL(oppo_board_id);
EXPORT_SYMBOL(oppo_panel_id);
EXPORT_SYMBOL(panel_present);

/*
 * The pn553's signal-handling code needs to call force_sig_info to raise signals.
 * Export it here to avoid general kernel-wide access to this function.
 */
EXPORT_SYMBOL(force_sig_info);

static int __init set_ftm_mode(char *str)
{
	char *end;
	long new = simple_strtol(str, &end, 0);
	if (end == str || new > INT_MAX || new < INT_MIN)
		return -EINVAL;
	oppo_ftm_mode = new;
	return 1;
}
__setup("oppo.ftm_mode=", set_ftm_mode);

static int __init set_hw_id(char *str)
{
	char *end;
	long new = simple_strtol(str, &end, 0);
	if (end == str || new > INT_MAX || new < INT_MIN)
		return -EINVAL;
	oppo_hw_id = new;
	return 1;
}
__setup("oppo.hw_id=", set_hw_id);

static int __init set_board_id(char *str)
{
	char *end;
	long new = simple_strtol(str, &end, 0);
	if (end == str || new > INT_MAX || new < INT_MIN)
		return -EINVAL;
	oppo_board_id = new;
	return 1;
}
__setup("oppo.board_id=", set_board_id);

static int __init set_panel_id(char *str)
{
       char *end;
       long new = simple_strtol(str, &end, 16);
       if (end == str || (new & 0xFF000000))
               return -EINVAL;
       oppo_panel_id = new;
       return 1;
}
__setup("oppo.panel_id=", set_panel_id);

static int __init set_is_panel_present(char *str)
{
    panel_present = 0;
    return 1;
}
__setup("panel.null", set_is_panel_present);
