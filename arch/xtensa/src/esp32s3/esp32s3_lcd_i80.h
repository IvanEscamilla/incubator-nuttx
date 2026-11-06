/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_lcd_i80.h
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
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

#ifndef __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_LCD_I80_H
#define __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_LCD_I80_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifndef __ASSEMBLY__

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* Transfer statistics.  Counters are cumulative since the last clear. */

struct esp32s3_lcd_i80_stats_s
{
  uint32_t transfers;     /* Pixel transfers completed */
  uint32_t bytes;         /* Pixel bytes sent */
  uint32_t dma_errors;    /* GDMA descriptor errors or FIFO over/underflow */
  uint32_t underruns;     /* GDMA TX FIFO underflow events */
  uint32_t short_xfers;   /* Transfers that ended with descriptors unread */
  uint32_t timeouts;      /* Transfers that never signalled completion */
  uint32_t te_edges;      /* TE rising edges seen */
  uint32_t te_timeouts;   /* TE waits that timed out */
  uint32_t last_xfer_us;  /* Duration of the last pixel transfer */
  uint32_t last_te_us;    /* Duration of the last TE wait */
  uint32_t pclk_khz;      /* WR clock used by the last pixel transfer */
  uint32_t last_dmaerr;   /* GDMA raw TX interrupt bits of the last error */
  uint32_t dbg[4];        /* GDMA state of the last short transfer */
  uint32_t last_prime_us; /* Time the last transfer waited for the DMA FIFO */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
#define EXTERN extern "C"
extern "C"
{
#else
#define EXTERN extern
#endif

/****************************************************************************
 * Name: esp32s3_lcd_i80_initialize
 *
 * Description:
 *   Configure the LCD_CAM peripheral as an 8-bit i80 bus, its GPIOs, the
 *   GDMA channel and the interrupt.  It is idempotent.
 *
 * Returned Value:
 *   Zero (OK) on success; a negated errno value on failure.
 *
 ****************************************************************************/

int esp32s3_lcd_i80_initialize(void);

/****************************************************************************
 * Name: esp32s3_lcd_i80_cmd
 *
 * Description:
 *   Send one command byte (D/C low) followed by len parameter bytes
 *   (D/C high).  len is at most 64.
 *
 ****************************************************************************/

int esp32s3_lcd_i80_cmd(uint8_t cmd, FAR const uint8_t *params, size_t len);

/****************************************************************************
 * Name: esp32s3_lcd_i80_write
 *
 * Description:
 *   Send a command byte and then rows of RGB565 data to the panel.  The
 *   call returns when the transfer is complete.
 *
 *   A transfer from PSRAM first writes the data cache back to PSRAM and is
 *   read directly by GDMA (one descriptor per row, using stride), unless
 *   bounce mode is selected: then rows are copied through two internal
 *   buffers.  The two 16-bit pixel bytes are swapped by the hardware, so
 *   the data is little-endian RGB565 in memory.
 *
 * Input Parameters:
 *   cmd          - Command to send first (RAMWR)
 *   continue_cmd - Command to send before later bounce chunks (RAMWRC)
 *   buf          - First byte of the first row
 *   row_bytes    - Bytes per row (even)
 *   rows         - Number of rows
 *   stride       - Distance in bytes between the starts of two rows; 0 sends
 *                  the same row rows times (a fill)
 *
 ****************************************************************************/

int esp32s3_lcd_i80_write(int cmd, int continue_cmd,
                          FAR const uint8_t *buf, size_t row_bytes,
                          size_t rows, size_t stride);

/****************************************************************************
 * Name: esp32s3_lcd_i80_set_async / esp32s3_lcd_i80_sync
 *
 * Description:
 *   In async mode a write from a direct (non-bounce) buffer returns once
 *   the transfer is started, so the caller can render the next band while
 *   it runs.  The buffer must stay untouched until the next write, command
 *   or esp32s3_lcd_i80_sync(), which wait for it.  Off by default.
 *
 ****************************************************************************/

int esp32s3_lcd_i80_set_async(bool async);
int esp32s3_lcd_i80_sync(void);

/****************************************************************************
 * Name: esp32s3_lcd_i80_set_pclk
 *
 * Description:
 *   Override the WR clock for pixel transfers.  The nearest valid clock is
 *   used; 0 restores the configured defaults.
 *
 * Returned Value:
 *   The WR clock in kHz in use (default or override).
 *
 ****************************************************************************/

uint32_t esp32s3_lcd_i80_set_pclk(uint32_t khz);

/****************************************************************************
 * Name: esp32s3_lcd_i80_set_bounce
 *
 * Description:
 *   Select bounce mode (true) or direct GDMA reads from PSRAM (false).
 *   Sources in internal RAM are always read directly.
 *
 ****************************************************************************/

void esp32s3_lcd_i80_set_bounce(bool bounce);
bool esp32s3_lcd_i80_get_bounce(void);

/****************************************************************************
 * Name: esp32s3_lcd_i80_wait_te
 *
 * Description:
 *   Wait for the next rising edge of the TE input.
 *
 * Returned Value:
 *   The time waited in microseconds, or -ETIMEDOUT, or -ENOSYS if TE is
 *   not configured.
 *
 ****************************************************************************/

int esp32s3_lcd_i80_wait_te(uint32_t timeout_ms);

/****************************************************************************
 * Name: esp32s3_lcd_i80_getstats
 ****************************************************************************/

void esp32s3_lcd_i80_getstats(FAR struct esp32s3_lcd_i80_stats_s *stats,
                              bool clear);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_XTENSA_SRC_ESP32S3_ESP32S3_LCD_I80_H */
