/****************************************************************************
 * boards/arm/stm32l4/nucleo-l476rg/src/stm32_buttons.c
 *
 *   Copyright (C) 2014-2015, 2017 Gregory Nutt. All rights reserved.
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

#include <stdint.h>
#include <errno.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <arch/board/board.h>

#include "nucleo-l476rg.h"

#ifdef CONFIG_ARCH_BUTTONS

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_button_initialize
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.
 *   After that, board_buttons() may be called to collect the current state
 *   of all buttons or board_button_irq() may be called to register button
 *   interrupt handlers.
 *
 ****************************************************************************/

uint32_t board_button_initialize(void)
{
  stm32l4_configgpio(GPIO_BTN_UL);
  stm32l4_configgpio(GPIO_BTN_UR);
  stm32l4_configgpio(GPIO_BTN_BL);
  stm32l4_configgpio(GPIO_BTN_BR);
  return NUM_BUTTONS;
}

/****************************************************************************
 * Name: board_buttons
 ****************************************************************************/

uint32_t board_buttons(void)
{
  /* Check that state of each USER button. A LOW value means that the key is
   * pressed.
   */

  uint32_t buttons = 0;
  buttons |= ((!stm32l4_gpioread(GPIO_BTN_UL)) << BOARD_BUTTON_UL);
  buttons |= ((!stm32l4_gpioread(GPIO_BTN_BL)) << BOARD_BUTTON_BL);
  buttons |= ((!stm32l4_gpioread(GPIO_BTN_UR)) << BOARD_BUTTON_UR);
  buttons |= ((!stm32l4_gpioread(GPIO_BTN_BR)) << BOARD_BUTTON_BR);
  return buttons;
}

/****************************************************************************
 * Button support.
 *
 * Description:
 *   board_button_initialize() must be called to initialize button resources.
 *   After that, board_buttons() may be called to collect the current state
 *   of all buttons or board_button_irq() may be called to register button
 *   interrupt handlers.
 *
 *   After board_button_initialize() has been called, board_buttons() may be
 *   called to collect the state of all buttons.  board_buttons() returns an
 *   32-bit bit set with each bit associated with a button.  See the
 *   BUTTON_*_BIT definitions in board.h for the meaning of each bit.
 *
 *   board_button_irq() may be called to register an interrupt handler that
 *   will be called when a button is depressed or released. The ID value is a
 *   button enumeration value that uniquely identifies a button resource. See
 *   the BUTTON_* definitions in board.h for the meaning of enumeration
 *   value.
 *
 ****************************************************************************/

#ifdef CONFIG_ARCH_IRQBUTTONS
int board_button_irq(int id, xcpt_t irqhandler, FAR void *arg)
{
  int ret = -EINVAL;

  if (id == BOARD_BUTTON_BL)
  {
    ret = stm32l4_gpiosetevent(GPIO_BTN_BL, true, true, false, irqhandler, arg);
  }
  else if (id == BOARD_BUTTON_BR)
  {
    ret = stm32l4_gpiosetevent(GPIO_BTN_BR, true, true, false, irqhandler, arg);
  }
  else if (id == BOARD_BUTTON_UR)
  {
    ret = stm32l4_gpiosetevent(GPIO_BTN_UR, true, true, false, irqhandler, arg);
  }
  else if (id == BOARD_BUTTON_UL)
  {
    ret = stm32l4_gpiosetevent(GPIO_BTN_UL, true, true, false, irqhandler, arg);
  }

  return ret;
}
#endif
#endif /* CONFIG_ARCH_BUTTONS */
