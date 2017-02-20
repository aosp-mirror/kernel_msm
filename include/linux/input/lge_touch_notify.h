#ifndef __LINUX_TOUCH_NOTIFY_H
#define __LINUX_TOUCH_NOTIFY_H

#include <linux/notifier.h>


/* the dsv on */
#define LCD_EVENT_TOUCH_LPWG_ON		0x01
#define LCD_EVENT_TOUCH_LPWG_OFF	0x02

#define LCD_EVENT_TOUCH_PWR_OFF      0XFF
/* to let lcd-driver know touch-driver's status */
#define LCD_EVENT_TOUCH_DRIVER_REGISTERED	0x03
/* For notifying proxy status to operate ENA control in lcd driver*/
#define LCD_EVENT_TOUCH_PROXY_STATUS 0X04
#define LCD_EVENT_TOUCH_SLEEP_STATUS 0X05
#define LCD_EVENT_TOUCH_SWIPE_STATUS 0X06
#define LCD_EVENT_TOUCH_PANEL_INFO_READ		0x07
#define LCD_EVENT_TOUCH_PANEL_INFO_WRITE	0x08

/* For PPlus */
#define NOTIFY_TOUCH_RESET			0x07
#define NOTIFY_CONNECTION			0x09
#define NOTIFY_WIRELEES				0x0A
#define NOTIFY_IME_STATE			0x0B
#define NOTIFY_DEBUG_TOOL			0x0C
#define NOTIFY_CALL_STATE			0x0D
#define NOTIFY_FB				0x0E
#define NOTIFY_EARJACK				0x0F
#define NOTIFY_DEBUG_OPTION			0x10
#define NOTIFY_ONHAND_STATE			0x12
#define NOTIFY_TOUCH_IRQ			0x13
#define LCD_EVENT_HW_RESET			(NOTIFY_TOUCH_RESET)
#define LCD_EVENT_LCD_MODE			0x08
#define LCD_EVENT_READ_REG			0x11

struct touch_event {
	void *data;
};

int touch_blocking_notifier_register(struct notifier_block *nb);
int touch_blocking_notifier_unregister(struct notifier_block *nb);
int touch_blocking_notifier_call(unsigned long val, void *v);

int touch_atomic_notifier_register(struct notifier_block *nb);
int touch_atomic_notifier_unregister(struct notifier_block *nb);
int touch_atomic_notifier_call(unsigned long val, void *v);

int touch_register_client(struct notifier_block *nb);
int touch_unregister_client(struct notifier_block *nb);
int touch_notifier_call_chain(unsigned long val, void *v);
#endif /* _LINUX_TOUCH_NOTIFY_H */
