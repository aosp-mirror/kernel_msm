/*
* Copyright c 2016 PixArt Imaging Inc.. All Rights Reserved.

* This program is free software; you may redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation; version 2 of the License.

* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
* BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
* ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
* CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*/
#ifndef _PIXART_PLATFORM_
#define _PIXART_PLATFORM_

#include <linux/input.h>
#include <linux/pm.h>
#include <linux/spi/spi.h>
#include <linux/module.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/delay.h>
#include <linux/types.h>

/* extern functions */
extern unsigned char OTS_Read_Reg(unsigned char addr);
extern void OTS_Write_Reg(unsigned char addr, unsigned char data);
extern void delay(int ms);

/*define downscale factor for rotation of shaft */
#define EXPECTED_COUNT_PER_ROUND 	60
#define REAL_AVG_COUNT_PER_ROUND 	60
/*base on: sensor Reg0x0d=0x44, shaft diameter=4mm, sensor-to-shaft distance=1.3mm*/

#define debug_print printk
#endif
