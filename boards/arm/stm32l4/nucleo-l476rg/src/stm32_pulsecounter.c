/****************************************************************************
 * boards/arm/stm32l4/nucleo-l476rg/src/stm32_pulsecounter.c
 *
 *   Copyright (C) 2011 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *   Modified: Librae <librae8226@gmail.com>
 *   Modified: Matias Nitsche <mnitsche@dc.ub.ar>
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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/spi/spi.h>
#include <nuttx/lcd/lcd.h>
#include <nuttx/lcd/memlcd.h>
#include <nuttx/timers/pwm.h>

#include "stm32l4_gpio.h"
#include "stm32l4_lptim.h"
#include "nucleo-l476rg.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/
/* Configuration ************************************************************/

#define STM32L4_GPIO_LPTIM2_IN  (GPIO_LPTIM2_IN1_2 | GPIO_PULLUP)

/****************************************************************************
 * Private Data
 ****************************************************************************/

struct stm32l4_lptim_dev_s* lptim2 = NULL;

/****************************************************************************
 * Private Functions
 ****************************************************************************/


/****************************************************************************
 * Public Functions
 ****************************************************************************/

void stm32l4_pulsecounter_initialize(void)
{
  lptim2 = stm32l4_lptim_init(2);

  STM32L4_LPTIM_SETCLOCKSOURCE(lptim2, STM32L4_LPTIM_CLK_EXT);
  STM32L4_LPTIM_SETMODE(lptim2, STM32L4_LPTIM_MODE_CONTINUOUS);
  STM32L4_LPTIM_SETPERIOD(lptim2, 0xffff);
  /* TODO: if counter is non zero here, disable and re-enable should reset it */

  stm32l4_configgpio(STM32L4_GPIO_LPTIM2_IN);
}

void stm32l4_pulsecounter_deinitialize(void)
{
  if (lptim2)
    {
      stm32l4_lptim_deinit(lptim2);
      lptim2 = NULL;

      stm32l4_unconfiggpio(STM32L4_GPIO_LPTIM2_IN);
    }
}

uint32_t stm32l4_pulsecounter_getcounter(void)
{
  DEBUGASSERT(lptim2);

  return STM32L4_LPTIM_GETCOUNTER(lptim2);
}

