/****************************************************************************
 * boards/arm/stm32l4/nucleo-l476rg/src/stm32l4_appinit.c
 *
 *   Copyright (C) 2016, 2018 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <sys/mount.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <syslog.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/sdio.h>
#include <nuttx/mmcsd.h>

#include <stm32l4.h>
#include <stm32l4_uart.h>

#include <arch/board/board.h>
#include <nuttx/leds/userled.h>

#ifdef CONFIG_LCD_DEV
#include <lcd_dev/lcd_dev.h>
#endif

#include "nucleo-l476rg.h"

#ifdef HAVE_RTC_DRIVER
#  include <nuttx/timers/rtc.h>
#  include "stm32l4_rtc.h"
#endif

#include <nuttx/input/buttons.h>

#include <nuttx/timers/oneshot.h>
#include <nuttx/audio/tone.h>
#include "stm32l4_pwm.h"

#include <nuttx/mtd/mtd.h>
#include <nuttx/mtd/configdata.h>
#include <mc6470/mc6470_acc.h>
#include <mc6470/mc6470_mag.h>

#include "stm32l4_i2c.h"
#include "stm32l4_dfumode.h"


/****************************************************************************
 * Private Functions
 ****************************************************************************/

static void mc6470_acc_enable_int(xcpt_t irqhandler, void* arg)
{
  stm32l4_gpiosetevent(GPIO_MC6470_ACC_INT, false, true, false, irqhandler, arg);
}

/****************************************************************************
 * Private Data
 ***************************************************************************/

struct mc6470_acc_lower_dev_s g_mc6470_lower =
{
  .enable_irq = &mc6470_acc_enable_int
};

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: stm32_i2c_register
 *
 * Description:
 *   Register one I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
static void stm32_i2c_register(int bus)
{
  FAR struct i2c_master_s *i2c;
  int ret;

  i2c = stm32l4_i2cbus_initialize(bus);
  if (i2c == NULL)
  {
    syslog(LOG_ERR, "ERROR: Failed to get I2C%d interface\n", bus);
  }
  else
  {
    ret = i2c_register(i2c, bus);
    if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: Failed to register I2C%d driver: %d\n",
             bus, ret);
      stm32l4_i2cbus_uninitialize(i2c);
    }
  }
}
#endif

/****************************************************************************
 * Name: stm32_i2ctool
 *
 * Description:
 *   Register I2C drivers for the I2C tool.
 *
 ****************************************************************************/

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
static void stm32_i2ctool(void)
{
  stm32_i2c_register(1);
}
#else
#  define stm32_i2ctool()
#endif

/****************************************************************************
 * Name: board_app_initialize
 *
 * Description:
 *   Perform application specific initialization.  This function is never
 *   called directly from application code, but only indirectly via the
 *   (non-standard) boardctl() interface using the command BOARDIOC_INIT.
 *
 * Input Parameters:
 *   arg - The boardctl() argument is passed to the board_app_initialize()
 *         implementation without modification.  The argument has no
 *         meaning to NuttX; the meaning of the argument is a contract
 *         between the board-specific initialization logic and the
 *         matching application logic.  The value could be such things as a
 *         mode enumeration value, a set of DIP switch switch settings, a
 *         pointer to configuration data read from a file or serial FLASH,
 *         or whatever you would like to do with it.  Every implementation
 *         should accept zero/NULL as a default configuration.
 *
 * Returned Value:
 *   Zero (OK) is returned on success; a negated errno value is returned on
 *   any failure to indicate the nature of the failure.
 *
 ****************************************************************************/

int board_app_initialize(uintptr_t arg)
{
#ifdef HAVE_RTC_DRIVER
  FAR struct rtc_lowerhalf_s *rtclower;
  struct i2c_master_s* i2c;
#endif

#ifdef CONFIG_BUTTONS_LOWER
  btn_lower_initialize("/dev/buttons");

  /* test buttons to enter bootloader */

  if (board_buttons() & BUTTON_UL_BIT)
  {
    stm32l4_dfumode();
  }
#endif

#if defined(CONFIG_I2C) && defined(CONFIG_SYSTEM_I2CTOOL)
  stm32_i2ctool();
#endif

  int ret = OK;

#ifdef HAVE_PROC
  /* Mount the proc filesystem */

  syslog(LOG_INFO, "Mounting procfs to /proc\n");

  ret = mount(NULL, CONFIG_NSH_PROC_MOUNTPOINT, "procfs", 0, NULL);
  if (ret < 0)
    {
      syslog(LOG_ERR,
             "ERROR: Failed to mount the PROC filesystem: %d (%d)\n",
             ret, errno);
      return ret;
    }
#endif

#ifdef HAVE_RTC_DRIVER
  /* Instantiate the STM32L4 lower-half RTC driver */

  rtclower = stm32l4_rtc_lowerhalf();
  if (!rtclower)
    {
      serr("ERROR: Failed to instantiate the RTC lower-half driver\n");
      return -ENOMEM;
    }
  else
    {
      /* Bind the lower half driver and register the combined RTC driver
       * as /dev/rtc0
       */

      ret = rtc_initialize(0, rtclower);
      if (ret < 0)
        {
          serr("ERROR: Failed to bind/register the RTC driver: %d\n", ret);
          return ret;
        }
    }
#endif

#ifdef CONFIG_SENSORS_BMP280
  ret = stm32_bmp280initialize("/dev/press0");
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize BMP280, error %d\n", ret);
      return ret;
    }
#endif

#ifdef CONFIG_ADC
  /* Initialize ADC and register the ADC driver. */

  ret = stm32l4_adc_setup();
  if (ret < 0)
    {
      syslog(LOG_ERR, "ERROR: stm32l4_adc_setup failed: %d\n", ret);
    }
#endif

#ifdef CONFIG_DEV_GPIO
  ret = stm32l4_gpio_initialize();
  if (ret < 0)
    {
      syslog(LOG_ERR, "Failed to initialize GPIO Driver: %d\n", ret);
      return ret;
    }
#endif

  /* bicycle companion extra initialization */

#ifdef CONFIG_LCD
  board_lcd_initialize();
#endif

#ifdef CONFIG_LCD_DEV
  lcddev_register(0);
#endif

  struct pwm_lowerhalf_s* pwm = stm32l4_pwminitialize(4);

#ifdef CONFIG_AUDIO_TONE
  {
    struct oneshot_lowerhalf_s* oneshot = oneshot_initialize(3, 10); /* TIM3, 10us resolution */
    pwm->ops->setup(pwm);
    tone_register("/dev/buzzer", pwm, oneshot);
  }

  pwm_register("/dev/buzzer_pwm", pwm);
#endif  

  /* initialize some GPIOs */

  stm32l4_configgpio(GPIO_MOTOR);

  stm32l4_configgpio(GPIO_BAT_PG);
  stm32l4_configgpio(GPIO_BAT_STAT1);
  stm32l4_configgpio(GPIO_BAT_STAT2);

  /* initialize I2C devices */

  i2c = stm32l4_i2cbus_initialize(1);

  stm32l4_configgpio(GPIO_MC6470_ACC_INT);

  mc6470_acc_register("/dev/acc", i2c, &g_mc6470_lower);
  mc6470_mag_register("/dev/mag", i2c);

  mtdconfig_register(at24c_initialize(i2c));

  stm32l4_pulsecounter_initialize();

#ifndef CONFIG_ARCH_LEDS
  userled_lower_initialize("/dev/userleds");
#endif

  return ret;
}

#if defined(CONFIG_BOARDCTL_UNIQUEID)
int board_uniqueid(uint8_t *uniqueid)
{
  if (uniqueid == 0)
    {
      return -EINVAL;
    }

  stm32l4_get_uniqueid(uniqueid);
  return OK;
}
#endif
