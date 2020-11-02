/****************************************************************************
 * arch/sim/src/sim/up_lcd.c
 *
 *   Copyright (C) 2011, 2015, 2020 Gregory Nutt. All rights reserved.
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
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <debug.h>

#include <nuttx/arch.h>
#include <nuttx/board.h>
#include <nuttx/lcd/lcd.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

/* Check contrast selection */

#if !defined(CONFIG_LCD_MAXCONTRAST)
#  define CONFIG_LCD_MAXCONTRAST 100
#endif

/* Check power setting */

#if !defined(CONFIG_LCD_MAXPOWER)
#  define CONFIG_LCD_MAXPOWER 100
#endif

/* Simulated LCD geometry and color format */

#ifndef CONFIG_SIM_FBWIDTH
#  define CONFIG_SIM_FBWIDTH  320 /* Framebuffer width in pixels */
#endif

#ifndef CONFIG_SIM_FBHEIGHT
#  define CONFIG_SIM_FBHEIGHT 240 /* Framebuffer height in pixels */
#endif

#ifndef CONFIG_SIM_FBBPP
#  define CONFIG_SIM_FBBPP    16  /* Framebuffer bytes per pixel (RGB) */
#endif

#define FB_STRIDE ((CONFIG_SIM_FBBPP * CONFIG_SIM_FBWIDTH + 7) >> 3)

#undef FB_FMT
#if CONFIG_SIM_FBBPP == 1
#  define FB_FMT FB_FMT_RGB1
#elif CONFIG_SIM_FBBPP == 4
#  define FB_FMT FB_FMT_RGB4
#elif CONFIG_SIM_FBBPP == 8
#  define FB_FMT FB_FMT_RGB8
#elif CONFIG_SIM_FBBPP == 16
#  define FB_FMT FB_FMT_RGB16_565
#elif CONFIG_SIM_FBBPP == 24
#  define FB_FMT FB_FMT_RGB24
#elif CONFIG_SIM_FBBPP == 32
#  define FB_FMT FB_FMT_RGB32
#else
#  error "Unsupported BPP"
#endif

/****************************************************************************
 * Private Type Definition
 ****************************************************************************/

/* This structure describes the state of this driver */

struct sim_dev_s
{
  /* Publicly visible device structure */

  struct lcd_dev_s dev;

  /* Private LCD-specific information follows */

  uint8_t power;        /* Current power setting */
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* LCD Data Transfer Methods */

static int sim_putrun(fb_coord_t row, fb_coord_t col,
                      FAR const uint8_t *buffer, size_t npixels);
static int sim_putarea(fb_coord_t row_start, fb_coord_t row_end,
                       fb_coord_t col_start, fb_coord_t col_end,
                       FAR const uint8_t *buffer);
static int sim_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                      size_t npixels);

/* LCD Configuration */

static int sim_getvideoinfo(FAR struct lcd_dev_s *dev,
                            FAR struct fb_videoinfo_s *vinfo);
static int sim_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                            FAR struct lcd_planeinfo_s *pinfo);

/* LCD RGB Mapping */

#ifdef CONFIG_FB_CMAP
#  error "RGB color mapping not supported by this driver"
#endif

/* Cursor Controls */

#ifdef CONFIG_FB_HWCURSOR
#  error "Cursor control not supported by this driver"
#endif

/* LCD Specific Controls */

static int sim_getpower(struct lcd_dev_s *dev);
static int sim_setpower(struct lcd_dev_s *dev, int power);
static int sim_getcontrast(struct lcd_dev_s *dev);
static int sim_setcontrast(struct lcd_dev_s *dev, unsigned int contrast);

/****************************************************************************
 * Private Data
 ****************************************************************************/

/* This is working memory allocated by the LCD driver for each LCD device
 * and for each color plane.  This memory will hold one raster line of data.
 * The size of the allocated run buffer must therefore be at least
 * (bpp * xres / 8).  Actual alignment of the buffer must conform to the
 * bitwidth of the underlying pixel type.
 *
 * If there are multiple planes, they may share the same working buffer
 * because different planes will not be operate on concurrently.  However,
 * if there are multiple LCD devices, they must each have unique run buffers.
 */

static uint8_t g_runbuffer[FB_STRIDE];

/* This structure describes the overall LCD video controller */

static const struct fb_videoinfo_s g_videoinfo =
{
  .fmt     = FB_FMT,                     /* Color format: RGB16-565: RRRR RGGG GGGB BBBB */
  .xres    = CONFIG_SIM_FBWIDTH,         /* Horizontal resolution in pixel columns */
  .yres    = CONFIG_SIM_FBHEIGHT,        /* Vertical resolution in pixel rows */
  .nplanes = 1,                          /* Number of color planes supported */
};

/* This is the standard, NuttX Plane information object */

static const struct lcd_planeinfo_s g_planeinfo =
{
  .putrun  = sim_putrun,                 /* Put a run into LCD memory */
  .putarea = sim_putarea,                /* Put a rectangular area to LCD */
  .getrun  = sim_getrun,                 /* Get a run from LCD memory */
  .buffer  = (FAR uint8_t *)g_runbuffer, /* Run scratch buffer */
  .display = 0,                          /* Display number */
  .bpp     = CONFIG_SIM_FBBPP,           /* Bits-per-pixel */
};

/* This is the standard, NuttX LCD driver object */

static struct sim_dev_s g_lcddev =
{
  .dev =
  {
    /* LCD Configuration */

    .getvideoinfo = sim_getvideoinfo,
    .getplaneinfo = sim_getplaneinfo,

    /* LCD RGB Mapping -- Not supported */

    /* Cursor Controls -- Not supported */

    /* LCD Specific Controls */

    .getpower     = sim_getpower,
    .setpower     = sim_setpower,
    .getcontrast  = sim_getcontrast,
    .setcontrast  = sim_setcontrast,
  },
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  sim_putrun
 *
 * Description:
 *   This method can be used to write a partial raster line to the LCD:
 *
 *   row     - Starting row to write to (range: 0 <= row < yres)
 *   col     - Starting column to write to (range: 0 <= col <= xres-npixels)
 *   buffer  - The buffer containing the run to be written to the LCD
 *   npixels - The number of pixels to write to the LCD
 *             (range: 0 < npixels <= xres-col)
 *
 ****************************************************************************/

static int sim_putrun(fb_coord_t row, fb_coord_t col,
                      FAR const uint8_t *buffer, size_t npixels)
{
  lcdinfo("row: %d col: %d npixels: %d\n", row, col, npixels);
  return OK;
}

/****************************************************************************
 * Name:  sim_putarea
 *
 * Description:
 *   This method can be used to write a partial raster line to the LCD:
 *
 *   row_start - Starting row to write to (range: 0 <= row < yres)
 *   row_end   - Ending row to write to (range: row_start <= row < yres)
 *   col_start - Starting column to write to (range: 0 <= col <= xres)
 *   col_end   - Ending column to write to
 *               (range: col_start <= col_end < xres)
 *   buffer    - The buffer containing the area to be written to the LCD
 *
 ****************************************************************************/

static int sim_putarea(fb_coord_t row_start, fb_coord_t row_end,
                       fb_coord_t col_start, fb_coord_t col_end,
                       FAR const uint8_t *buffer)
{
  fb_coord_t row;
  size_t rows;
  size_t cols;
  size_t row_size;

  lcdinfo("row_start: %d row_end: %d col_start: %d col_end: %d\n",
          row_start, row_end, col_start, col_end);

  cols = col_end - col_start + 1;
  rows = row_end - row_start + 1;
  row_size = cols * (g_planeinfo.bpp >> 3);

#ifdef CONFIG_SIM_X11FB
  if (col_start == 0 && col_end == (g_videoinfo.xres - 1) &&
      g_stride == row_size)
    {
      /* simpler case, we can just memcpy() the whole buffer */

      memcpy(&g_planeinfo.buffer[row_start * g_stride], buffer,
             rows * row_size);
    }
  else
    {
      /* We have to go row by row */

      for (row = row_start; row <= row_end; row++)
        {
          memcpy(&g_planeinfo.buffer[row * g_stride + col_start *
                                     (g_planeinfo.bpp >> 3)],
                 &buffer[(row - row_start) * row_size],
                 cols * (g_planeinfo.bpp >> 3));
        }
    }
#endif

  return OK;
}

/****************************************************************************
 * Name:  sim_getrun
 *
 * Description:
 *   This method can be used to read a partial raster line from the LCD:
 *
 *  row     - Starting row to read from (range: 0 <= row < yres)
 *  col     - Starting column to read read (range: 0 <= col <= xres-npixels)
 *  buffer  - The buffer in which to return the run read from the LCD
 *  npixels - The number of pixels to read from the LCD
 *            (range: 0 < npixels <= xres-col)
 *
 ****************************************************************************/

static int sim_getrun(fb_coord_t row, fb_coord_t col, FAR uint8_t *buffer,
                      size_t npixels)
{
  lcdinfo("row: %d col: %d npixels: %d\n", row, col, npixels);
  return -ENOSYS;
}

/****************************************************************************
 * Name:  sim_getvideoinfo
 *
 * Description:
 *   Get information about the LCD video controller configuration.
 *
 ****************************************************************************/

static int sim_getvideoinfo(FAR struct lcd_dev_s *dev,
                            FAR struct fb_videoinfo_s *vinfo)
{
  DEBUGASSERT(dev && vinfo);
  ginfo("fmt: %d xres: %d yres: %d nplanes: %d\n",
         g_videoinfo.fmt, g_videoinfo.xres, g_videoinfo.yres,
         g_videoinfo.nplanes);

  memcpy(vinfo, &g_videoinfo, sizeof(struct fb_videoinfo_s));
  return OK;
}

/****************************************************************************
 * Name:  sim_getplaneinfo
 *
 * Description:
 *   Get information about the configuration of each LCD color plane.
 *
 ****************************************************************************/

static int sim_getplaneinfo(FAR struct lcd_dev_s *dev, unsigned int planeno,
                              FAR struct lcd_planeinfo_s *pinfo)
{
  DEBUGASSERT(dev && pinfo && planeno == 0);
  ginfo("planeno: %d bpp: %d\n", planeno, g_planeinfo.bpp);
  memcpy(pinfo, &g_planeinfo, sizeof(struct lcd_planeinfo_s));
  return OK;
}

/****************************************************************************
 * Name:  sim_getpower
 *
 * Description:
 *   Get the LCD panel power status (0: full off - CONFIG_LCD_MAXPOWER:
 *   full on). On backlit LCDs, this setting may correspond to the backlight
 *   setting.
 *
 ****************************************************************************/

static int sim_getpower(struct lcd_dev_s *dev)
{
  ginfo("power: %d\n", 0);
  return g_lcddev.power;
}

/****************************************************************************
 * Name:  sim_setpower
 *
 * Description:
 *   Enable/disable LCD panel power (0: full off - CONFIG_LCD_MAXPOWER:
 *   full on). On backlit LCDs, this setting may correspond to the backlight
 *   setting.
 *
 ****************************************************************************/

static int sim_setpower(struct lcd_dev_s *dev, int power)
{
  ginfo("power: %d\n", power);
  DEBUGASSERT(power <= CONFIG_LCD_MAXPOWER);

  /* Set new power level */

  g_lcddev.power = power;
  return OK;
}

/****************************************************************************
 * Name:  sim_getcontrast
 *
 * Description:
 *   Get the current contrast setting (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int sim_getcontrast(struct lcd_dev_s *dev)
{
  ginfo("Not implemented\n");
  return -ENOSYS;
}

/****************************************************************************
 * Name:  sim_setcontrast
 *
 * Description:
 *   Set LCD panel contrast (0-CONFIG_LCD_MAXCONTRAST).
 *
 ****************************************************************************/

static int sim_setcontrast(struct lcd_dev_s *dev, unsigned int contrast)
{
  ginfo("contrast: %d\n", contrast);
  return -ENOSYS;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name:  board_lcd_initialize
 *
 * Description:
 *   Initialize the LCD video hardware.  The initial state of the LCD is
 *   fully initialized, display memory cleared, and the LCD ready to use,
 *   but with the power setting at 0 (full off).
 *
 ****************************************************************************/

int board_lcd_initialize(void)
{
  ginfo("Initializing\n");
  return OK;
}

/****************************************************************************
 * Name:  board_lcd_getdev
 *
 * Description:
 *   Return a a reference to the LCD object for the specified LCD.  This
 *   allows support for multiple LCD devices.
 *
 ****************************************************************************/

FAR struct lcd_dev_s *board_lcd_getdev(int lcddev)
{
  DEBUGASSERT(lcddev == 0);
  return &g_lcddev.dev;
}

/****************************************************************************
 * Name:  board_lcd_uninitialize
 *
 * Description:
 *   Uninitialize the LCD support
 *
 ****************************************************************************/

void board_lcd_uninitialize(void)
{
}
