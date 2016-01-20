/*for P/L sensor common header file for each vender chip*/
#ifndef __LINUX_SENSOR_HUB_H
#define __LINUX_SENSOR_HUB_H

extern struct blocking_notifier_head double_tap_notifier_list;

extern int register_notifier_by_facedown(struct notifier_block *nb);
extern int unregister_notifier_by_facedown(struct notifier_block *nb);

#endif

