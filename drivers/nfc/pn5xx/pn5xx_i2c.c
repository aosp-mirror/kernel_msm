/*
 * Copyright (C) 2010 Trusted Logic S.A.
 * modifications copyright (C) 2015 NXP B.V.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
 *
 *
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>
#include "pn5xx_i2c.h"
#include <linux/of_gpio.h>
#include <linux/regulator/consumer.h>
#include <linux/of.h>
#include <asm/siginfo.h>
#include <linux/rcupdate.h>
#include <linux/sched.h>
#include <linux/signal.h>
#include <linux/clk.h>
#include <linux/wakelock.h>
#define DATA_TRANSFER_INTERVAL (2*HZ)
#define READ_DATA_TIMEOUT (100)
#define SIG_NFC 44
#define MAX_BUFFER_SIZE    512

#define MODE_OFF    0
#define MODE_RUN    1
#define MODE_FW     2

/* Only pn548, pn547 and pn544 are supported */
#define CHIP "pn544"
#define DRIVER_CARD    "PN5xx NFC"
#define DRIVER_DESC    "NFC driver for PN5xx Family"

#ifndef CONFIG_OF
#define CONFIG_OF
#endif
#define NFC_TRY_NUM (3)

struct pn5xx_dev    {
    wait_queue_head_t    read_wq;
    struct mutex        read_mutex;
    struct i2c_client    *client;
    struct miscdevice   pn5xx_device;
    int                 ven_gpio;
    int                 firm_gpio;
    int                 irq_gpio;
    int                 clkreq_gpio;
    struct regulator *pvdd_reg;
    struct regulator *vbat_reg;
    struct regulator *pmuvcc_reg;
    struct regulator *sevdd_reg;
    unsigned int        ese_pwr_gpio; /* gpio used by SPI to provide power to p61 via NFCC */
    struct mutex        p61_state_mutex; /* used to make p61_current_state flag secure */
    p61_access_state_t  p61_current_state; /* stores the current P61 state */
    bool                nfc_ven_enabled; /* stores the VEN pin state powered by Nfc */
    bool                spi_ven_enabled; /* stores the VEN pin state powered by Spi */
    bool            irq_enabled;
    spinlock_t        irq_enabled_lock;
    long                nfc_service_pid; /*used to signal the nfc the nfc service */
    struct wake_lock    wake_lock;
    struct clk         *clk;
    bool                clk_enabled;
};

static struct pn5xx_dev *pn5xx_dev;
static struct semaphore ese_access_sema;
static void release_ese_lock(p61_access_state_t  p61_current_state);
int get_ese_lock(p61_access_state_t  p61_current_state, int timeout);
static void p61_get_access_state(struct pn5xx_dev*, p61_access_state_t*);

static long  pn5xx_dev_ioctl(struct file *filp, unsigned int cmd,
                unsigned long arg);

/**********************************************************
 * Interrupt control and handler
 **********************************************************/
static void pn5xx_disable_irq(struct pn5xx_dev *pn5xx_dev)
{
    unsigned long flags;

    spin_lock_irqsave(&pn5xx_dev->irq_enabled_lock, flags);
    if (pn5xx_dev->irq_enabled) {
        disable_irq_nosync(pn5xx_dev->client->irq);
        pn5xx_dev->irq_enabled = false;
    }
    spin_unlock_irqrestore(&pn5xx_dev->irq_enabled_lock, flags);
}

static int pn5xx_clock_enable(struct pn5xx_dev *pn5xx_dev)
{
    int ret = 0;
    if (NULL == pn5xx_dev->clk)
    {
        return -1;
    }
    if (false == pn5xx_dev->clk_enabled)
    {
        ret = clk_prepare_enable(pn5xx_dev->clk);
        if (ret)
        {
            return -1;
        }
        pn5xx_dev->clk_enabled = true;
    }
    return ret;
}


static int pn5xx_clock_disable(struct pn5xx_dev *pn5xx_dev)
{
    int r = -1;
    if (pn5xx_dev->clk != NULL)
    {
        if (pn5xx_dev->clk_enabled)
        {
            clk_disable_unprepare(pn5xx_dev->clk);
            pn5xx_dev->clk_enabled = false;
        }
        return 0;
    }
    return r;
}


static irqreturn_t pn5xx_dev_irq_handler(int irq, void *dev_id)
{
    struct pn5xx_dev *pn5xx_dev = dev_id;

    pn5xx_disable_irq(pn5xx_dev);

    /* Wake up waiting readers */
    wake_up(&pn5xx_dev->read_wq);
    wake_lock_timeout(&pn5xx_dev->wake_lock, DATA_TRANSFER_INTERVAL);
    return IRQ_HANDLED;
}

static int pn5xx_suspend(struct i2c_client *client, pm_message_t mesg)
{
    pr_info("%s\n", __func__);
    return 0;
}

static int pn5xx_resume(struct i2c_client *client)
{
    return 0;
}


/**********************************************************
 * private functions
 **********************************************************/
static void p61_update_access_state(struct pn5xx_dev *pn5xx_dev, p61_access_state_t current_state, bool set)
{
    pr_info("%s: Enter current_state = %x\n", __func__, pn5xx_dev->p61_current_state);
    if (current_state)
    {
        if(set){
            if(pn5xx_dev->p61_current_state == P61_STATE_IDLE)
                pn5xx_dev->p61_current_state = P61_STATE_INVALID;
            pn5xx_dev->p61_current_state |= current_state;
        }
        else{
            pn5xx_dev->p61_current_state ^= current_state;
            if(!pn5xx_dev->p61_current_state)
                pn5xx_dev->p61_current_state = P61_STATE_IDLE;
        }
    }
    pr_info("%s: Exit current_state = %x\n", __func__, pn5xx_dev->p61_current_state);
}

static void p61_get_access_state(struct pn5xx_dev *pn5xx_dev, p61_access_state_t *current_state)
{

    if (current_state == NULL) {
        *current_state = P61_STATE_INVALID;
    } else {
        *current_state = pn5xx_dev->p61_current_state;
    }
}

static void p61_access_lock(struct pn5xx_dev *pn5xx_dev)
{
    pr_info("%s: Enter\n", __func__);
    mutex_lock(&pn5xx_dev->p61_state_mutex);
    pr_info("%s: Exit\n", __func__);
}

static void p61_access_unlock(struct pn5xx_dev *pn5xx_dev)
{
    pr_info("%s: Enter\n", __func__);
    mutex_unlock(&pn5xx_dev->p61_state_mutex);
    pr_info("%s: Exit\n", __func__);
}

static void signal_handler(p61_access_state_t state, long nfc_pid)
{
    struct siginfo sinfo;
    pid_t pid;
    struct task_struct *task;
    int sigret = 0;
    pr_info("%s: Enter\n", __func__);

    memset(&sinfo, 0, sizeof(struct siginfo));
    sinfo.si_signo = SIG_NFC;
    sinfo.si_code = SI_QUEUE;
    sinfo.si_int = state;
    pid = nfc_pid;

    task = pid_task(find_vpid(pid), PIDTYPE_PID);
    if(task)
    {
        pr_info("%s.\n", task->comm);
        sigret = force_sig_info(SIG_NFC, &sinfo, task);
        if(sigret < 0){
            pr_info("force_sig_info failed..... sigret %d.\n", sigret);
        }
    }
    else{
        pr_info("finding task from PID failed\r\n");
    }
    pr_info("%s: Exit\n", __func__);
}

static int pn5xx_enable(struct pn5xx_dev *dev, int mode)
{
    int r;
    p61_access_state_t current_state;

    /* turn on the regulators */
    /* -- if the regulators were specified, they're required */
    if(dev->pvdd_reg != NULL)
    {
        r = regulator_enable(dev->pvdd_reg);
        if (r < 0){
            pr_err("%s: not able to enable pvdd\n", __func__);
            return r;
        }
    }
    if(dev->vbat_reg != NULL)
    {
        r = regulator_enable(dev->vbat_reg);
        if (r < 0){
            pr_err("%s: not able to enable vbat\n", __func__);
            goto enable_exit0;
        }
    }
    if(dev->pmuvcc_reg != NULL)
    {
        r = regulator_enable(dev->pmuvcc_reg);
        if (r < 0){
            pr_err("%s: not able to enable pmuvcc\n", __func__);
            goto enable_exit1;
        }
    }
    if(dev->sevdd_reg != NULL)
    {
        r = regulator_enable(dev->sevdd_reg);
        if (r < 0){
            pr_err("%s: not able to enable sevdd\n", __func__);
            goto enable_exit2;
        }
    }

    current_state = P61_STATE_INVALID;
    p61_get_access_state(dev, &current_state);
    if (MODE_RUN == mode) {
        pr_info("%s power on\n", __func__);
        if (gpio_is_valid(dev->firm_gpio)) {
            if ((current_state & (P61_STATE_WIRED|P61_STATE_SPI|P61_STATE_SPI_PRIO))== 0) {
                p61_update_access_state(dev, P61_STATE_IDLE, true);
            }
            gpio_set_value(dev->firm_gpio, 0);
        }

        dev->nfc_ven_enabled = true;
        if (dev->spi_ven_enabled == false) {
            gpio_set_value(dev->ven_gpio, 1);
            msleep(100);
        }
        r = pn5xx_clock_enable(dev);
        if (r < 0)
        {
            dev_err(&dev->client->dev, "unable to enable clock\n");
        }
        msleep(20);
    }
    else if (MODE_FW == mode) {
        if (current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO))
        {
            /* NFCC fw/download should not be allowed if p61 is used
             * by SPI
             */
            pr_info("%s NFCC should not be allowed to reset/FW download \n", __func__);
            p61_access_unlock(dev);
            return -EBUSY; /* Device or resource busy */
        }
        dev->nfc_ven_enabled = true;
        if (dev->spi_ven_enabled == false)
        {
            /* power on with firmware download (requires hw reset)
             */
            pr_info("%s power on with firmware\n", __func__);
            gpio_set_value(dev->ven_gpio, 1);
            msleep(20);
            if (gpio_is_valid(dev->firm_gpio)) {
                p61_update_access_state(dev, P61_STATE_DWNLD, true);
                gpio_set_value(dev->firm_gpio, 1);
            }
            msleep(20);
            gpio_set_value(dev->ven_gpio, 0);
            msleep(100);
            gpio_set_value(dev->ven_gpio, 1);
            msleep(20);
        }
    } else {
        pr_err("%s bad arg %d\n", __func__, mode);
        p61_access_unlock(dev);
        return -EINVAL;
    }

    return 0;

enable_exit2:
    if(dev->pmuvcc_reg) regulator_disable(dev->pmuvcc_reg);
enable_exit1:
    if(dev->vbat_reg) regulator_disable(dev->vbat_reg);
enable_exit0:
    if(dev->pvdd_reg) regulator_disable(dev->pvdd_reg);

    return r;
}

static void pn5xx_disable(struct pn5xx_dev *dev)
{
    /* power off */
    int ret = 0;
    pr_info("%s power off\n", __func__);
    if (gpio_is_valid(dev->firm_gpio))
    {
        p61_access_state_t current_state = P61_STATE_INVALID;
        p61_get_access_state(dev, &current_state);
        if ((current_state & (P61_STATE_WIRED|P61_STATE_SPI|P61_STATE_SPI_PRIO))== 0) {
            p61_update_access_state(dev, P61_STATE_IDLE, true);
        }
        gpio_set_value(dev->firm_gpio, 0);
    }
    dev->nfc_ven_enabled = false;
    /* Don't change Ven state if spi made it high */
    if (dev->spi_ven_enabled == false) {
        gpio_set_value(dev->ven_gpio, 0);
        msleep(100);
    }
    ret = pn5xx_clock_disable(dev);
    if (ret < 0)
    {
        dev_err(&dev->client->dev, "unable to disable clock\n");
    }
    /* hardware dependent delay */
    msleep(100);
    if(dev->sevdd_reg) regulator_disable(dev->sevdd_reg);
    if(dev->pmuvcc_reg) regulator_disable(dev->pmuvcc_reg);
    if(dev->vbat_reg) regulator_disable(dev->vbat_reg);
    if(dev->pvdd_reg) regulator_disable(dev->pvdd_reg);

}

/**********************************************************
 * driver functions
 **********************************************************/
static ssize_t pn5xx_dev_read(struct file *filp, char __user *buf,
        size_t count, loff_t *offset)
{
    struct pn5xx_dev *pn5xx_dev = filp->private_data;
    char tmp[MAX_BUFFER_SIZE];
    int ret = 0;
    int retry = 0;
    bool isSuccess = false;

    if (count > MAX_BUFFER_SIZE)
    {
        count = MAX_BUFFER_SIZE;
    }
    pr_debug("%s : reading %zu bytes.\n", __func__, count);

    mutex_lock(&pn5xx_dev->read_mutex);

    if (!gpio_get_value(pn5xx_dev->irq_gpio)) {
        if (filp->f_flags & O_NONBLOCK) {
            ret = -EAGAIN;
            goto fail;
        }

        while (1) {
            ret = 0;
            pn5xx_dev->irq_enabled = true;
            enable_irq(pn5xx_dev->client->irq);
            /*If IRQ line is already high, which means IRQ was high
            just before enabling the interrupt, skip waiting for interrupt,
            as interrupt would have been disabled by then in the interrupt handler*/
            if (!gpio_get_value(pn5xx_dev->irq_gpio)){
            ret = wait_event_interruptible(
                    pn5xx_dev->read_wq,
                    !pn5xx_dev->irq_enabled);
            }
            pn5xx_disable_irq(pn5xx_dev);

            if (ret)
                goto fail;

            if (gpio_get_value(pn5xx_dev->irq_gpio))
                break;

            pr_warning("%s: spurious interrupt detected\n", __func__);
        }
    }
    /* Read data, we have 3 chances */
    for (retry = 0; retry < NFC_TRY_NUM; retry++)
    {
        ret = i2c_master_recv(pn5xx_dev->client, tmp, (int)count);
        if (ret == (int)count)
        {
            isSuccess = true;
            break;
        }
        else
        {
            pr_info("%s : read data try =%d returned %d\n", __func__, retry, ret);
            msleep(1);
            continue;
        }
    }
    if (false == isSuccess)
    {
        pr_err("%s : i2c_master_recv returned %d\n", __func__, ret);
        ret = -EIO;
    }

    mutex_unlock(&pn5xx_dev->read_mutex);

    /* pn5xx seems to be slow in handling I2C read requests
     * so add 1ms delay after recv operation */
    udelay(1000);
    if(0 != wake_lock_active(&pn5xx_dev->wake_lock))
    {
        wake_unlock(&pn5xx_dev->wake_lock);
    }
    if(gpio_get_value(pn5xx_dev->clkreq_gpio))
    {
        pr_err("%s: wake_lock_timeout in POS when read\n", __func__);
        wake_lock_timeout(&pn5xx_dev->wake_lock, READ_DATA_TIMEOUT);
    }
    if (ret < 0) {
        pr_err("%s: i2c_master_recv returned %d\n", __func__, ret);
        return ret;
    }
    if (ret > count) {
        pr_err("%s: received too many bytes from i2c (%d)\n",
            __func__, ret);
        return -EIO;
    }
    if (copy_to_user(buf, tmp, ret)) {
        pr_warning("%s : failed to copy to user space\n", __func__);
        return -EFAULT;
    }
    return ret;

fail:
    mutex_unlock(&pn5xx_dev->read_mutex);
    return ret;
}

static ssize_t pn5xx_dev_write(struct file *filp, const char __user *buf,
        size_t count, loff_t *offset)
{
    struct pn5xx_dev  *pn5xx_dev;
    char tmp[MAX_BUFFER_SIZE];
    int ret;
    int retry = 0;
    bool isSuccess = false;

    pn5xx_dev = filp->private_data;

    if (count > MAX_BUFFER_SIZE)
    {
        count = MAX_BUFFER_SIZE;
    }

    if (copy_from_user(tmp, buf, count)) {
        pr_err("%s : failed to copy from user space\n", __func__);
        return -EFAULT;
    }

    pr_debug("%s : writing %zu bytes.\n", __func__, count);

    /* Write data, we have 3 chances */
    for (retry = 0; retry < NFC_TRY_NUM; retry++)
    {
        ret = i2c_master_send(pn5xx_dev->client, tmp, (int)count);
        if (ret == (int)count)
        {
            isSuccess = true;
            break;
        }
        else
        {
            if (retry > 0)
            {
                pr_info("%s : send data try =%d returned %d\n", __func__, retry, ret);
            }
            msleep(1);
            continue;
         }
    }

    if (false == isSuccess)
    {
        pr_err("%s : i2c_master_send returned %d\n", __func__, ret);
        ret = -EIO;
    }

    /* pn5xx seems to be slow in handling I2C write requests
     * so add 1ms delay after I2C send oparation */
    udelay(1000);

    return ret;
}

static int pn5xx_dev_open(struct inode *inode, struct file *filp)
{
    struct pn5xx_dev *pn5xx_dev = container_of(filp->private_data,
                                               struct pn5xx_dev,
                                               pn5xx_device);

    filp->private_data = pn5xx_dev;

    pr_info("%s : %d,%d\n", __func__, imajor(inode), iminor(inode));

    // pn5xx_enable(pn5xx_dev, MODE_RUN);

    return 0;
}

static int pn5xx_dev_release(struct inode *inode, struct file *filp)
{
    // struct pn5xx_dev *pn5xx_dev = container_of(filp->private_data,
    //                                           struct pn5xx_dev,
    //                                           pn5xx_device);

    pr_info("%s : closing %d,%d\n", __func__, imajor(inode), iminor(inode));

    // pn5xx_disable(pn5xx_dev);

    return 0;
}

long  pn5xx_dev_ioctl_byp61(unsigned int cmd,
                unsigned long arg)
{
    struct file tmp;
    memset(&tmp, 0, sizeof(tmp));
    pr_info("%s, cmd=%d, arg=%lu\n", __func__, cmd, arg);
    tmp.private_data = pn5xx_dev;
    return pn5xx_dev_ioctl(&tmp,cmd,arg);
}

long  pn5xx_dev_ioctl(struct file *filp, unsigned int cmd,
                unsigned long arg)
{
    struct pn5xx_dev *pn5xx_dev = filp->private_data;

    pr_info("%s, cmd=%d, arg=%lu\n", __func__, cmd, arg);

    if (cmd == PN5XX_GET_ESE_ACCESS)
    {
        return get_ese_lock(P61_STATE_WIRED, arg);
    }
    p61_access_lock(pn5xx_dev);
    switch (cmd) {
    case PN5XX_SET_PWR:
        if (arg == 2) {
            /* power on w/FW */
            pn5xx_enable(pn5xx_dev, arg);
        } else if (arg == 1) {
            /* power on */
            pn5xx_enable(pn5xx_dev, arg);
        } else  if (arg == 0) {
            /* power off */
            pn5xx_disable(pn5xx_dev);
        } else {
            pr_err("%s bad SET_PWR arg %lu\n", __func__, arg);
            return -EINVAL;
        }
        break;
    case PN5XX_CLK_REQ:
        if(1 == arg){
            if(gpio_is_valid(pn5xx_dev->clkreq_gpio)){
                gpio_set_value(pn5xx_dev->clkreq_gpio, 1);
            }
        }
        else if(0 == arg) {
            if(gpio_is_valid(pn5xx_dev->clkreq_gpio)){
                gpio_set_value(pn5xx_dev->clkreq_gpio, 0);
            }
        } else {
            pr_err("%s bad CLK_REQ arg %lu\n", __func__, arg);
            return -EINVAL;
        }
        break;
    case P61_SET_SPI_PWR:
    {
        p61_access_state_t current_state = P61_STATE_INVALID;
        p61_get_access_state(pn5xx_dev, &current_state);
        if (arg == 1) {
            pr_info("%s : PN61_SET_SPI_PWR - power on ese\n", __func__);
            if ((current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO)) == 0) {
                p61_update_access_state(pn5xx_dev, P61_STATE_SPI, true);
                /*To handle triple mode protection signal
                NFC service when SPI session started*/
                if (current_state & P61_STATE_WIRED){
                    if(pn5xx_dev->nfc_service_pid){
                        pr_info("nfc service pid %s   ---- %ld", __func__, pn5xx_dev->nfc_service_pid);
                        signal_handler(P61_STATE_SPI, pn5xx_dev->nfc_service_pid);
                    }
                    else{
                        pr_info(" invalid nfc service pid....signalling failed%s   ---- %ld", __func__, pn5xx_dev->nfc_service_pid);
                    }
                }
                pn5xx_dev->spi_ven_enabled = true;
                if (pn5xx_dev->nfc_ven_enabled == false) {
                    /* provide power to NFCC if, NFC service not provided */
                    gpio_set_value(pn5xx_dev->ven_gpio, 1);
                    msleep(10);
                }
                /* pull the gpio to high once NFCC is power on*/
                gpio_set_value(pn5xx_dev->ese_pwr_gpio, 1);
            } else {
                pr_info("%s : PN61_SET_SPI_PWR -  power on ese failed \n", __func__);
                p61_access_unlock(pn5xx_dev);
                return -EBUSY; /* Device or resource busy */
            }
        } else if (arg == 0) {
            pr_info("%s : PN61_SET_SPI_PWR - power off ese\n", __func__);
            if(current_state & P61_STATE_SPI_PRIO){
                p61_update_access_state(pn5xx_dev, P61_STATE_SPI_PRIO, false);
                if (current_state & P61_STATE_WIRED)
                {
                    if(pn5xx_dev->nfc_service_pid){
                        pr_info("nfc service pid %s   ---- %ld", __func__, pn5xx_dev->nfc_service_pid);
                        signal_handler(P61_STATE_SPI_PRIO_END, pn5xx_dev->nfc_service_pid);
                }
                else{
                    pr_info(" invalid nfc service pid....signalling failed%s   ---- %ld", __func__, pn5xx_dev->nfc_service_pid);
                }
                }
                if (!(current_state & P61_STATE_WIRED))
                    gpio_set_value(pn5xx_dev->ese_pwr_gpio, 0);
                pn5xx_dev->spi_ven_enabled = false;
                 if (pn5xx_dev->nfc_ven_enabled == false) {
                     gpio_set_value(pn5xx_dev->ven_gpio, 0);
                     msleep(10);
                 }
              }else if(current_state & P61_STATE_SPI){
                  p61_update_access_state(pn5xx_dev, P61_STATE_SPI, false);
                  if (!(current_state & P61_STATE_WIRED)){
                      gpio_set_value(pn5xx_dev->ese_pwr_gpio, 0);
                  }
                /*To handle triple mode protection signal
                NFC service when SPI session started*/
                 if (current_state & P61_STATE_WIRED){
                    if(pn5xx_dev->nfc_service_pid){
                        pr_info("nfc service pid %s   ---- %ld", __func__, pn5xx_dev->nfc_service_pid);
                        signal_handler(P61_STATE_SPI, pn5xx_dev->nfc_service_pid);
                    }
                    else{
                        pr_info(" invalid nfc service pid....signalling failed%s   ---- %ld", __func__, pn5xx_dev->nfc_service_pid);
                    }
                  }
                  pn5xx_dev->spi_ven_enabled = false;
                  if (pn5xx_dev->nfc_ven_enabled == false) {
                      gpio_set_value(pn5xx_dev->ven_gpio, 0);
                      msleep(10);
                  }
            } else {
                pr_err("%s : PN61_SET_SPI_PWR - failed, current_state = %x \n",
                        __func__, pn5xx_dev->p61_current_state);
                p61_access_unlock(pn5xx_dev);
                return -EPERM; /* Operation not permitted */
            }
        }else if (arg == 2) {
            pr_info("%s : PN61_SET_SPI_PWR - reset\n", __func__);
            if (current_state & (P61_STATE_IDLE|P61_STATE_SPI|P61_STATE_SPI_PRIO)) {
                if (pn5xx_dev->spi_ven_enabled == false)
                {
                    pn5xx_dev->spi_ven_enabled = true;
                    if (pn5xx_dev->nfc_ven_enabled == false) {
                        /* provide power to NFCC if, NFC service not provided */
                        gpio_set_value(pn5xx_dev->ven_gpio, 1);
                        msleep(10);
                    }
                }
                gpio_set_value(pn5xx_dev->ese_pwr_gpio, 0);
                msleep(10);
                gpio_set_value(pn5xx_dev->ese_pwr_gpio, 1);
                msleep(10);
            } else {
                pr_info("%s : PN61_SET_SPI_PWR - reset  failed \n", __func__);
                p61_access_unlock(pn5xx_dev);
                return -EBUSY; /* Device or resource busy */
            }
        }else if (arg == 3) {
            pr_info("%s : PN61_SET_SPI_PWR - Prio Session Start power on ese\n", __func__);
            if ((current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO)) == 0) {
                p61_update_access_state(pn5xx_dev, P61_STATE_SPI_PRIO, true);
                if (current_state & P61_STATE_WIRED){
                    if(pn5xx_dev->nfc_service_pid){
                        pr_info("nfc service pid %s   ---- %ld", __func__, pn5xx_dev->nfc_service_pid);
                        signal_handler(P61_STATE_SPI_PRIO, pn5xx_dev->nfc_service_pid);
                    }
                    else{
                        pr_info(" invalid nfc service pid....signalling failed%s   ---- %ld", __func__, pn5xx_dev->nfc_service_pid);
                    }
                }
                pn5xx_dev->spi_ven_enabled = true;
                if (pn5xx_dev->nfc_ven_enabled == false) {
                    /* provide power to NFCC if, NFC service not provided */
                    gpio_set_value(pn5xx_dev->ven_gpio, 1);
                    msleep(10);
                }
                /* pull the gpio to high once NFCC is power on*/
                gpio_set_value(pn5xx_dev->ese_pwr_gpio, 1);
            }else {
                pr_info("%s : Prio Session Start power on ese failed \n", __func__);
                p61_access_unlock(pn5xx_dev);
                return -EBUSY; /* Device or resource busy */
            }
        }else if (arg == 4) {
            if (current_state & P61_STATE_SPI_PRIO)
            {
                pr_info("%s : PN61_SET_SPI_PWR - Prio Session Ending...\n", __func__);
                p61_update_access_state(pn5xx_dev, P61_STATE_SPI_PRIO, false);
                /*after SPI prio timeout, the state is changing from SPI prio to SPI */
                p61_update_access_state(pn5xx_dev, P61_STATE_SPI, true);
                if (current_state & P61_STATE_WIRED)
                {
                    if(pn5xx_dev->nfc_service_pid){
                        pr_info("nfc service pid %s   ---- %ld", __func__, pn5xx_dev->nfc_service_pid);
                        signal_handler(P61_STATE_SPI_PRIO_END, pn5xx_dev->nfc_service_pid);
                    }
                    else{
                        pr_info(" invalid nfc service pid....signalling failed%s   ---- %ld", __func__, pn5xx_dev->nfc_service_pid);
                    }
               }
            }
            else
            {
                pr_info("%s : PN61_SET_SPI_PWR -  Prio Session End failed \n", __func__);
                p61_access_unlock(pn5xx_dev);
                return -EBADRQC; /* Device or resource busy */
            }
        } else if(arg == 5){
            release_ese_lock(P61_STATE_SPI);
        }
        else {
            pr_info("%s bad ese pwr arg %lu\n", __func__, arg);
            p61_access_unlock(pn5xx_dev);
            return -EBADRQC; /* Invalid request code */
        }
    }
    break;
    case P61_GET_PWR_STATUS:
    {
        p61_access_state_t current_state = P61_STATE_INVALID;
        p61_get_access_state(pn5xx_dev, &current_state);
        pr_info("%s: P61_GET_PWR_STATUS  = %x",__func__, current_state);
        put_user(current_state, (int __user *)arg);
    }
    break;
    case P61_SET_WIRED_ACCESS:
    {
        p61_access_state_t current_state = P61_STATE_INVALID;
        p61_get_access_state(pn5xx_dev, &current_state);
        if (arg == 1)
        {
            if (current_state)
            {
                pr_info("%s : P61_SET_WIRED_ACCESS - enabling\n", __func__);
                p61_update_access_state(pn5xx_dev, P61_STATE_WIRED, true);
                if (current_state & P61_STATE_SPI_PRIO)
                {
                    if(pn5xx_dev->nfc_service_pid){
                        pr_info("nfc service pid %s   ---- %ld", __func__, pn5xx_dev->nfc_service_pid);
                        signal_handler(P61_STATE_SPI_PRIO, pn5xx_dev->nfc_service_pid);
                    }
                    else{
                        pr_info(" invalid nfc service pid....signalling failed%s   ---- %ld", __func__, pn5xx_dev->nfc_service_pid);
                    }
                }
                if((current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO)) == 0)
                    gpio_set_value(pn5xx_dev->ese_pwr_gpio, 1);
            } else {
                pr_info("%s : P61_SET_WIRED_ACCESS -  enabling failed \n", __func__);
                p61_access_unlock(pn5xx_dev);
                return -EBUSY; /* Device or resource busy */
            }
        } else if (arg == 0) {
            pr_info("%s : P61_SET_WIRED_ACCESS - disabling \n", __func__);
            if (current_state & P61_STATE_WIRED){
                p61_update_access_state(pn5xx_dev, P61_STATE_WIRED, false);
                if((current_state & (P61_STATE_SPI|P61_STATE_SPI_PRIO)) == 0)
                    gpio_set_value(pn5xx_dev->ese_pwr_gpio, 0);
            } else {
                pr_err("%s : P61_SET_WIRED_ACCESS - failed, current_state = %x \n",
                        __func__, pn5xx_dev->p61_current_state);
                p61_access_unlock(pn5xx_dev);
                return -EPERM; /* Operation not permitted */
            }
        }
        else if(arg == 2)
        {
             pr_info("%s : P61_ESE_GPIO_LOW  \n", __func__);
             gpio_set_value(pn5xx_dev->ese_pwr_gpio, 0);
        }
        else if(arg == 3)
        {
            pr_info("%s : P61_ESE_GPIO_HIGH  \n", __func__);
            gpio_set_value(pn5xx_dev->ese_pwr_gpio, 1);
        }
        else if(arg == 4)
        {
            release_ese_lock(P61_STATE_WIRED);
        }
        else {
            pr_info("%s P61_SET_WIRED_ACCESS - bad arg %lu\n", __func__, arg);
            p61_access_unlock(pn5xx_dev);
            return -EBADRQC; /* Invalid request code */
        }
    }
    break;
    case PN5XX_SET_NFC_SERVICE_PID:
    {
        pr_info("%s : The NFC Service PID is %ld\n", __func__, arg);
        pn5xx_dev->nfc_service_pid = arg;

    }
    break;
    default:
        pr_err("%s bad ioctl %u\n", __func__, cmd);
        p61_access_unlock(pn5xx_dev);
        return -EINVAL;
    }
    p61_access_unlock(pn5xx_dev);
    return 0;
}

EXPORT_SYMBOL(pn5xx_dev_ioctl_byp61);
int get_ese_lock(p61_access_state_t  p61_current_state, int timeout)
{
    unsigned long tempJ = msecs_to_jiffies(timeout);
    if(down_timeout(&ese_access_sema, tempJ) != 0)
    {
        printk("get_ese_lock: timeout p61_current_state = %d\n", p61_current_state);
        return -EBUSY;
    }
    return 0;
}

EXPORT_SYMBOL(get_ese_lock);

static void release_ese_lock(p61_access_state_t  p61_current_state)
{
    up(&ese_access_sema);
}

static const struct file_operations pn5xx_dev_fops = {
    .owner    = THIS_MODULE,
    .llseek    = no_llseek,
    .read    = pn5xx_dev_read,
    .write    = pn5xx_dev_write,
    .open    = pn5xx_dev_open,
    .release  = pn5xx_dev_release,
    .unlocked_ioctl  = pn5xx_dev_ioctl,
};

/*
 * Handlers for alternative sources of platform_data
 */
#ifdef CONFIG_OF
/*
 * Translate OpenFirmware node properties into platform_data
 */
static int pn5xx_get_pdata(struct device *dev,
                           struct pn5xx_i2c_platform_data *pdata)
{
    struct device_node *node;
    u32 flags;
    int val;

    /* make sure there is actually a device tree node */
    node = dev->of_node;
    if (!node)
        return -ENODEV;

    memset(pdata, 0, sizeof(*pdata));

    /* read the dev tree data */

    /* ven pin - enable's power to the chip - REQUIRED */
    val = of_get_named_gpio_flags(node, "nxp,pn5xx-ven", 0, &flags);
    if (val >= 0) {
        pdata->ven_gpio = val;
    }
    else{
        dev_err(dev, "VEN GPIO error getting from OF node\n");
        return val;
    }

    /* firm pin - controls firmware download - OPTIONAL */
    val = of_get_named_gpio_flags(node, "nxp,pn5xx-fw-dwnld", 0, &flags);
    if (val >= 0) {
        pdata->firm_gpio = val;
}
    else {
        pdata->firm_gpio = -EINVAL;
        dev_warn(dev, "FIRM GPIO <OPTIONAL> error getting from OF node\n");
    }

    /* irq pin - data available irq - REQUIRED */
    val = of_get_named_gpio_flags(node, "nxp,pn5xx-irq", 0, &flags);
    if (val >= 0) {
        pdata->irq_gpio = val;
    }
    else {
        dev_err(dev, "IRQ GPIO error getting from OF node\n");
        return val;
    }

    /* ese-pwr pin - enable's power to the ese- REQUIRED */
    val = of_get_named_gpio_flags(node, "nxp,pn5xx-ese-pwr", 0, &flags);
    if (val >= 0) {
        pdata->ese_pwr_gpio = val;
    }
    else {
        dev_err(dev, "ESE PWR GPIO error getting from OF node\n");
        return val;
    }

    /* clkreq pin - controls the clock to the PN547 - OPTIONAL */
    val = of_get_named_gpio_flags(node, "nxp,pn5xx-clkreq", 0, &flags);
    if (val >= 0) {
        pdata->clkreq_gpio = val;
    }
    else {
        pdata->clkreq_gpio = -EINVAL;
        dev_warn(dev, "CLKREQ GPIO <OPTIONAL> error getting from OF node\n");
    }

    /* handle the regulator lines - these are optional
     * PVdd - pad Vdd (544, 547)
     * Vbat - Battery (544, 547)
     * PMUVcc - UICC Power (544, 547)
     * SEVdd - SE Power (544)
     *
     * Will attempt to load a matching Regulator Resource for each
     * If no resource is provided, then the input will not be controlled
     * Example: if only PVdd is provided, it is the only one that will be
     *  turned on/off.
     */
    pdata->pvdd_reg = regulator_get(dev, "nxp,pn5xx-pvdd");
    if(IS_ERR(pdata->pvdd_reg)) {
        pr_err("%s: could not get nxp,pn5xx-pvdd, rc=%ld\n", __func__, PTR_ERR(pdata->pvdd_reg));
        pdata->pvdd_reg = NULL;
    }

    pdata->vbat_reg = regulator_get(dev, "nxp,pn5xx-vbat");
    if (IS_ERR(pdata->vbat_reg)) {
        pr_err("%s: could not get nxp,pn5xx-vbat, rc=%ld\n", __func__, PTR_ERR(pdata->vbat_reg));
        pdata->vbat_reg = NULL;
    }

    pdata->pmuvcc_reg = regulator_get(dev, "nxp,pn5xx-pmuvcc");
    if (IS_ERR(pdata->pmuvcc_reg)) {
        pr_err("%s: could not get nxp,pn5xx-pmuvcc, rc=%ld\n", __func__, PTR_ERR(pdata->pmuvcc_reg));
        pdata->pmuvcc_reg = NULL;
    }

    pdata->sevdd_reg = regulator_get(dev, "nxp,pn5xx-sevdd");
    if (IS_ERR(pdata->sevdd_reg)) {
        pr_err("%s: could not get nxp,pn5xx-sevdd, rc=%ld\n", __func__, PTR_ERR(pdata->sevdd_reg));
        pdata->sevdd_reg = NULL;
    }

    return 0;
}
#else
static int pn5xx_get_pdata(struct device *dev,
                           struct pn5xx_i2c_platform_data *pdata)
{
    pdata = dev->platform_data;
    return 0;
}
#endif


/*
 *  pn5xx_probe
 */
#ifdef KERNEL_3_4_AND_OLDER
 static int __devinit pn5xx_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
#else
static int pn5xx_probe(struct i2c_client *client,
        const struct i2c_device_id *id)
#endif
{
    int ret;
    struct pn5xx_i2c_platform_data *pdata;  // gpio values, from board file or DT
    struct pn5xx_i2c_platform_data tmp_pdata;

    pr_info("%s\n", __func__);

    /* ---- retrieve the platform data ---- */
    /* If the dev.platform_data is NULL, then */
    /* attempt to read from the device tree */
    if(!client->dev.platform_data)
    {
        ret = pn5xx_get_pdata(&(client->dev), &tmp_pdata);
        if(ret){
            return ret;
        }

        pdata = &tmp_pdata;
    }
    else
    {
        pdata = client->dev.platform_data;
    }

    if (pdata == NULL) {
        pr_err("%s : nfc probe fail\n", __func__);
        return  -ENODEV;
    }

    /* validate the the adapter has basic I2C functionality */
    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        pr_err("%s : need I2C_FUNC_I2C\n", __func__);
        return  -ENODEV;
    }

    /* reserve the GPIO pins */
    pr_info("%s: request irq_gpio %d\n", __func__, pdata->irq_gpio);
    ret = gpio_request(pdata->irq_gpio, "nfc_int");
    if (ret){
        pr_err("%s :not able to get GPIO irq_gpio\n", __func__);
        return  -ENODEV;
    }
    ret = gpio_to_irq(pdata->irq_gpio);
    if (ret < 0){
        pr_err("%s :not able to map GPIO irq_gpio to an IRQ\n", __func__);
        goto err_ven;
    }
    else{
        client->irq = ret;
    }

    pr_info("%s: request ven_gpio %d\n", __func__, pdata->ven_gpio);
    ret = gpio_request(pdata->ven_gpio, "nfc_ven");
    if (ret){
        pr_err("%s :not able to get GPIO ven_gpio\n", __func__);
        goto err_ven;
    }

    if (gpio_is_valid(pdata->firm_gpio)) {
        pr_info("%s: request firm_gpio %d\n", __func__, pdata->firm_gpio);
        ret = gpio_request(pdata->firm_gpio, "nfc_firm");
        if (ret){
            pr_err("%s :not able to get GPIO firm_gpio\n", __func__);
            goto err_firm;
        }
    }

    if (gpio_is_valid(pdata->ese_pwr_gpio)) {
        pr_info("%s: request ese_pwr_gpio %d\n", __func__, pdata->ese_pwr_gpio);
        ret = gpio_request(pdata->ese_pwr_gpio, "nfc_ese_pwr");
        if (ret){
            pr_err("%s :not able to get GPIO ese_pwr_gpio\n", __func__);
            goto err_ese_pwr;
        }
    }

    if (gpio_is_valid(pdata->clkreq_gpio)) {
        pr_info("%s: request clkreq_gpio %d\n", __func__, pdata->clkreq_gpio);
        ret = gpio_request(pdata->clkreq_gpio, "nfc_clkreq");
        if (ret){
            pr_err("%s :not able to get GPIO clkreq_gpio\n", __func__);
            goto err_clkreq;
        }
    }

    /* allocate the pn5xx driver information structure */
    pn5xx_dev = kzalloc(sizeof(*pn5xx_dev), GFP_KERNEL);
    if (pn5xx_dev == NULL) {
        dev_err(&client->dev, "failed to allocate memory for module data\n");
        ret = -ENOMEM;
        goto err_exit;
    }

    /* store the platform data in the driver info struct */
    pn5xx_dev->irq_gpio = pdata->irq_gpio;
    pn5xx_dev->ven_gpio  = pdata->ven_gpio;
    pn5xx_dev->firm_gpio  = pdata->firm_gpio;
    pn5xx_dev->ese_pwr_gpio  = pdata->ese_pwr_gpio;
    pn5xx_dev->p61_current_state = P61_STATE_IDLE;
    pn5xx_dev->nfc_ven_enabled = false;
    pn5xx_dev->spi_ven_enabled = false;
    pn5xx_dev->clkreq_gpio = pdata->clkreq_gpio;
    pn5xx_dev->pvdd_reg = pdata->pvdd_reg;
    pn5xx_dev->vbat_reg = pdata->vbat_reg;
    pn5xx_dev->pmuvcc_reg = pdata->vbat_reg;
    pn5xx_dev->sevdd_reg = pdata->sevdd_reg;
    wake_lock_init(&pn5xx_dev->wake_lock, WAKE_LOCK_SUSPEND,"pn5xx");
    pn5xx_dev->client   = client;
    pn5xx_dev->clk_enabled = false;

    pn5xx_dev->clk = clk_get(&client->dev, "bbclk2");
    if (IS_ERR(pn5xx_dev->clk)) {
        dev_err(&client->dev, "%s: unable to get clk\n", __func__);
        ret = PTR_ERR(pn5xx_dev->clk);
        goto err_exit;
    }

    /* finish configuring the I/O */
    ret = gpio_direction_input(pn5xx_dev->irq_gpio);
    if (ret < 0) {
        pr_err("%s :not able to set irq_gpio as input\n", __func__);
        goto err_exit;
    }

    ret = gpio_direction_output(pn5xx_dev->ven_gpio, 0);
    if (ret < 0) {
        pr_err("%s : not able to set ven_gpio as output\n", __func__);
        goto err_exit;
    }

    if (gpio_is_valid(pn5xx_dev->firm_gpio)) {
        ret = gpio_direction_output(pn5xx_dev->firm_gpio, 0);
        if (ret < 0) {
            pr_err("%s : not able to set firm_gpio as output\n",
                 __func__);
            goto err_exit;
        }
    }

    ret = gpio_direction_output(pn5xx_dev->ese_pwr_gpio, 0);
    if (ret < 0) {
        pr_err("%s : not able to set ese_pwr gpio as output\n", __func__);
        goto err_ese_pwr;
    }

    if (gpio_is_valid(pn5xx_dev->clkreq_gpio)) {
        ret = gpio_direction_input(pn5xx_dev->clkreq_gpio);
        if (ret < 0) {
            pr_err("%s : not able to set clkreq_gpio as output\n",
                   __func__);
            goto err_exit;
        }
    }

    /* init mutex and queues */
    init_waitqueue_head(&pn5xx_dev->read_wq);
    mutex_init(&pn5xx_dev->read_mutex);
    sema_init(&ese_access_sema, 1);
    mutex_init(&pn5xx_dev->p61_state_mutex);
    spin_lock_init(&pn5xx_dev->irq_enabled_lock);

    /* register as a misc device - character based with one entry point */
    pn5xx_dev->pn5xx_device.minor = MISC_DYNAMIC_MINOR;
    pn5xx_dev->pn5xx_device.name = CHIP;
    pn5xx_dev->pn5xx_device.fops = &pn5xx_dev_fops;
    ret = misc_register(&pn5xx_dev->pn5xx_device);
    if (ret) {
        pr_err("%s : misc_register failed\n", __FILE__);
        goto err_misc_register;
    }

    /* request irq.  the irq is set whenever the chip has data available
     * for reading.  it is cleared when all data has been read.
     */
    pr_info("%s : requesting IRQ %d\n", __func__, client->irq);
    pn5xx_dev->irq_enabled = true;
    ret = request_irq(client->irq, pn5xx_dev_irq_handler,
              IRQF_TRIGGER_HIGH, client->name, pn5xx_dev);
    if (ret) {
        dev_err(&client->dev, "request_irq failed\n");
        goto err_request_irq_failed;
    }
    enable_irq_wake(client->irq);
    pn5xx_disable_irq(pn5xx_dev);


    i2c_set_clientdata(client, pn5xx_dev);

    return 0;

err_request_irq_failed:
    misc_deregister(&pn5xx_dev->pn5xx_device);
err_misc_register:
    mutex_destroy(&pn5xx_dev->read_mutex);
    mutex_destroy(&pn5xx_dev->p61_state_mutex);
    kfree(pn5xx_dev);
err_exit:
    if (gpio_is_valid(pdata->clkreq_gpio))
        gpio_free(pdata->clkreq_gpio);
    if(pn5xx_dev->clk)
    {
        clk_put(pn5xx_dev->clk);
    }
err_ese_pwr:
    gpio_free(pdata->ese_pwr_gpio);
err_clkreq:
    if (gpio_is_valid(pdata->firm_gpio))
        gpio_free(pdata->firm_gpio);
err_firm:
    gpio_free(pdata->ven_gpio);
err_ven:
    gpio_free(pdata->irq_gpio);
    return ret;
}

#ifdef KERNEL_3_4_AND_OLDER
static int __devexit pn5xx_remove(struct i2c_client *client)
#else
static int pn5xx_remove(struct i2c_client *client)
#endif
{
    struct pn5xx_dev *pn5xx_dev;

    pr_info("%s\n", __func__);

    pn5xx_dev = i2c_get_clientdata(client);
    free_irq(client->irq, pn5xx_dev);
    misc_deregister(&pn5xx_dev->pn5xx_device);
    mutex_destroy(&pn5xx_dev->read_mutex);
    mutex_destroy(&pn5xx_dev->p61_state_mutex);
    gpio_free(pn5xx_dev->irq_gpio);
    gpio_free(pn5xx_dev->ven_gpio);
    gpio_free(pn5xx_dev->ese_pwr_gpio);
    pn5xx_dev->p61_current_state = P61_STATE_INVALID;
    pn5xx_dev->nfc_ven_enabled = false;
    pn5xx_dev->spi_ven_enabled = false;
    if(pn5xx_dev->clk)
    {
        clk_put(pn5xx_dev->clk);
    }
    if (gpio_is_valid(pn5xx_dev->firm_gpio))
        gpio_free(pn5xx_dev->firm_gpio);
    if (gpio_is_valid(pn5xx_dev->clkreq_gpio))
        gpio_free(pn5xx_dev->clkreq_gpio);
    regulator_put(pn5xx_dev->pvdd_reg);
    regulator_put(pn5xx_dev->vbat_reg);
    regulator_put(pn5xx_dev->pmuvcc_reg);
    regulator_put(pn5xx_dev->sevdd_reg);
    wake_lock_destroy(&pn5xx_dev->wake_lock);
    kfree(pn5xx_dev);

    return 0;
}

/*
 *
 */
#ifdef CONFIG_OF
static struct of_device_id pn5xx_dt_match[] = {
    { .compatible = "nxp,pn547", },
    { .compatible = "nxp,pn544", },
    {},
};
MODULE_DEVICE_TABLE(of, pn5xx_dt_match);
#endif

static const struct i2c_device_id pn5xx_id[] = {
    { "pn547", 0 },
    { },
};
MODULE_DEVICE_TABLE(i2c, pn5xx_id);

static struct i2c_driver pn5xx_driver = {
    .id_table    = pn5xx_id,
    .probe        = pn5xx_probe,
#ifdef KERNEL_3_4_AND_OLDER
    .remove        = __devexit_p(pn5xx_remove),
#else
    .remove        = pn5xx_remove,
#endif
    .driver        = {
        .owner    = THIS_MODULE,
        .name    = "pn544",
        .of_match_table = pn5xx_dt_match,
    },
    .suspend = pn5xx_suspend,
    .resume = pn5xx_resume,
};

/*
 * module load/unload record keeping
 */

static int __init pn5xx_dev_init(void)
{
    pr_info("%s\n", __func__);
    return i2c_add_driver(&pn5xx_driver);
}

static void __exit pn5xx_dev_exit(void)
{
    pr_info("%s\n", __func__);
    i2c_del_driver(&pn5xx_driver);
}

module_init(pn5xx_dev_init);
module_exit(pn5xx_dev_exit);

MODULE_AUTHOR("Sylvain Fonteneau");
MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");

