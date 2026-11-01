/****************************************************************************
 * include/nuttx/lcd/st7796.h
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __INCLUDE_NUTTX_LCD_ST7796_H
#define __INCLUDE_NUTTX_LCD_ST7796_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include <nuttx/fs/ioctl.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#ifdef CONFIG_LCD_ST7796_BUS_I80

/* ioctl commands of the i80 variant, handled by /dev/lcd0 in addition to the
 * LCDDEVIO_* commands of lcd_dev.h.  They sit above the range of lcd_dev.h.
 */

#define ST7796IOC_WAITTE      _LCDIOC(0x40) /* Arg: uint32_t* in: timeout ms,
                                             *      out: microseconds waited */
#define ST7796IOC_GETSTATS    _LCDIOC(0x41) /* Arg: struct st7796_stats_s* */
#define ST7796IOC_CLRSTATS    _LCDIOC(0x42) /* Arg: none */
#define ST7796IOC_SETPCLK     _LCDIOC(0x43) /* Arg: uint32_t* in: kHz (0 for
                                             *      the default), out: kHz */
#define ST7796IOC_SETBOUNCE   _LCDIOC(0x44) /* Arg: int (0 direct, 1 bounce) */
#define ST7796IOC_SETBRIGHT   _LCDIOC(0x45) /* Arg: int, backlight 0..100 % */
#define ST7796IOC_GETMODE     _LCDIOC(0x46) /* Arg: struct st7796_mode_s* */

#endif

/****************************************************************************
 * Public Types
 ****************************************************************************/

#ifndef __ASSEMBLY__

#ifdef CONFIG_LCD_ST7796_BUS_I80

/* Bus statistics, cumulative since ST7796IOC_CLRSTATS */

struct st7796_stats_s
{
  uint32_t transfers;     /* Pixel transfers completed */
  uint32_t bytes;         /* Pixel bytes sent */
  uint32_t dma_errors;    /* DMA descriptor errors or FIFO over/underflow */
  uint32_t underruns;     /* DMA FIFO underflow events */
  uint32_t short_xfers;   /* Transfers that ended with data unread */
  uint32_t timeouts;      /* Transfers that never completed */
  uint32_t te_edges;      /* TE rising edges seen */
  uint32_t te_timeouts;   /* TE waits that timed out */
  uint32_t last_xfer_us;  /* Duration of the last pixel transfer */
  uint32_t last_te_us;    /* Duration of the last TE wait */
  uint32_t pclk_khz;      /* WR clock of the last pixel transfer */
  uint32_t last_dmaerr;   /* DMA status bits of the last error */
  uint32_t dbg[4];        /* DMA state of the last short transfer */
  uint32_t last_prime_us; /* Wait for the DMA FIFO before the last start */
};

/* Transfer mode */

struct st7796_mode_s
{
  uint32_t bounce;        /* PSRAM sources are bounced through internal RAM */
  uint32_t te_enabled;    /* The panel TE output is on */
};

/* The bus the i80 variant of the driver writes to.  The board provides it
 * (for the ESP32-S3 it wraps the LCD_CAM driver).
 */

struct st7796_i80_ops_s
{
  /* Send a command byte followed by up to 64 parameter bytes */

  CODE int (*cmd)(uint8_t cmd, FAR const uint8_t *params, size_t len);

  /* Send cmd (negative: none) and then rows of 16-bit pixels, stride bytes
   * apart.  Returns when the transfer is complete.
   */

  CODE int (*write)(int cmd, FAR const uint8_t *buf, size_t row_bytes,
                    size_t rows, size_t stride);

  /* Optional: board-specific ioctl (statistics, clock, mode, TE wait) */

  CODE int (*ioctl)(int cmd, unsigned long arg);

  /* Optional: backlight on/off, called by SETPOWER */

  CODE void (*power)(bool on);
};

#endif /* CONFIG_LCD_ST7796_BUS_I80 */

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/****************************************************************************
 * Name: st7796_fbinitialize
 *
 * Description:
 *   Initialize the ST7796 LCD controller as a framebuffer device.
 *
 *   This function initializes the ST7796 display controller and returns
 *   a framebuffer virtual table. Display parameters (resolution, color
 *   depth, orientation, frequency) are taken from CONFIG_LCD_ST7796_*
 *   Kconfig options. The driver uses CONFIG_SPI_CMDDATA to control the
 *   DC (Data/Command) pin automatically via the SPI driver.
 *
 * Input Parameters:
 *   spi - SPI device instance configured for the ST7796
 *
 * Returned Value:
 *   Pointer to framebuffer vtable on success; NULL on failure.
 *
 * Assumptions:
 *   - CONFIG_SPI_CMDDATA is enabled
 *   - DC pin has been registered with the SPI driver
 *   - CS pin has been registered with the SPI driver
 *   - RESET and backlight pins configured by board code
 *
 ****************************************************************************/

#ifdef CONFIG_LCD_ST7796_BUS_SPI
FAR struct fb_vtable_s *st7796_fbinitialize(FAR struct spi_dev_s *spi);
#endif

#ifdef CONFIG_LCD_ST7796_BUS_I80

/****************************************************************************
 * Name: st7796_i80_initialize
 *
 * Description:
 *   Initialize the ST7796 on an i80 bus and return its lcd_dev_s, for
 *   board_lcd_getdev() and lcddev_register().  The panel reset line must
 *   have been pulsed by the board before.
 *
 * Input Parameters:
 *   ops - The bus; it must persist
 *
 * Returned Value:
 *   The LCD, or NULL on failure.
 *
 ****************************************************************************/

struct lcd_dev_s;
FAR struct lcd_dev_s *st7796_i80_initialize(
                          FAR const struct st7796_i80_ops_s *ops);

#endif

#ifdef CONFIG_LCD_ST7796_BUS_SPI

/****************************************************************************
 * Name: st7796_setrotation
 *
 * Description:
 *   Set display rotation at runtime. Valid rotation values are 0, 90,
 *   180, and 270 degrees. The rotation is applied by modifying the
 *   MADCTL register.
 *
 * Input Parameters:
 *   vtable   - Reference to the framebuffer virtual table
 *   rotation - Rotation angle in degrees (0, 90, 180, or 270)
 *
 * Returned Value:
 *   OK on success; a negated errno value on failure.
 *
 ****************************************************************************/

int st7796_setrotation(FAR struct fb_vtable_s *vtable, uint16_t rotation);

/****************************************************************************
 * Name: st7796_getrotation
 *
 * Description:
 *   Get current display rotation.
 *
 * Input Parameters:
 *   vtable - Reference to the framebuffer virtual table
 *
 * Returned Value:
 *   Current rotation in degrees (0, 90, 180, or 270).
 *
 ****************************************************************************/

uint16_t st7796_getrotation(FAR struct fb_vtable_s *vtable);

/****************************************************************************
 * Name: st7796_board_power
 *
 * Description:
 *   Board-level callback to control display backlight/power.
 *   Called by the ST7796 driver when FBIOSET_POWER ioctl is
 *   received. The board must implement this function to
 *   control the backlight GPIO pin.
 *
 * Input Parameters:
 *   on - true to turn backlight on, false to turn off
 *
 ****************************************************************************/

void st7796_board_power(bool on);

#endif /* CONFIG_LCD_ST7796_BUS_SPI */

#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __INCLUDE_NUTTX_LCD_ST7796_H */
