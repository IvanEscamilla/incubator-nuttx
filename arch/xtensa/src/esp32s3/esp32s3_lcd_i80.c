/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_lcd_i80.c
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

/* LCD_CAM in 8-bit i80 (8080) mode.  Commands and parameters are sent by
 * the LCD_CAM command/data phases (parameters through a small GDMA buffer),
 * pixels by GDMA with one descriptor per row of the area, so that a
 * rectangle of a larger frame buffer is sent without copying it.  The
 * register sequence follows ESP-IDF's esp_lcd_panel_io_i80.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <assert.h>
#include <errno.h>
#include <inttypes.h>
#include <string.h>
#include <time.h>

#include <nuttx/arch.h>
#include <nuttx/debug.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/kmalloc.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>

#include "xtensa.h"
#include "esp_clk.h"
#include "esp_gpio.h"
#include "esp_irq.h"
#include "esp32s3_dma.h"
#include "esp32s3_lcd_i80.h"

#include "hardware/esp32s3_dma.h"
#include "hardware/esp32s3_gpio_sigmap.h"
#include "hardware/esp32s3_soc.h"

#include "periph_ctrl.h"
#include "soc/lcd_cam_struct.h"
#include "hal/lcd_ll.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define LCD_I80_DEV         ((lcd_cam_dev_t *)DR_REG_LCD_CAM_BASE)

/* LCD_CLK = PLL160 / 2 = 80 MHz.  WR = LCD_CLK / prescale. */

#define LCD_I80_CLK_KHZ     80000
#define LCD_I80_CLK_DIV     2

#define LCD_I80_PIECE_MAX   4092   /* Max bytes of one descriptor, mult of 4 */
#define LCD_I80_CMDBUF      64
#define LCD_I80_TIMEOUT_MS  500

#define LCD_I80_MAXROWS     CONFIG_ESP32S3_LCD_I80_MAXROWS
#define LCD_I80_BBUF        CONFIG_ESP32S3_LCD_I80_BOUNCE_BUFSIZE
#define LCD_I80_BDESC       ((LCD_I80_BBUF + LCD_I80_PIECE_MAX - 1) / \
                             LCD_I80_PIECE_MAX)

/* GDMA TX raw interrupt bits (esp_hal_dma gdma_ll.h).  OUT_DONE and OUT_EOF
 * are asserted for the descriptor chains used here.  OUT_TOTAL_EOF is not
 * asserted in this mode, even though every descriptor has been consumed.
 */

#define LCD_I80_DMA_COMPLETE   ((1 << 0) | (1 << 1))
#define LCD_I80_DMA_UNDERRUN   ((1 << 5) | (1 << 7))
#define LCD_I80_DMA_ERRORS     ((1 << 2) | (1 << 4) | (1 << 5) | \
                                (1 << 6) | (1 << 7))

#ifdef CONFIG_ARCH_DCACHE
#  define i80_cache_clean(s, e)  up_clean_dcache((uintptr_t)(s), \
                                                 (uintptr_t)(e))
#else
#  define i80_cache_clean(s, e)
#endif

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct lcd_i80_s
{
  bool     initialized;
  bool     bounce;
  bool     async;                /* pixel writes return once started */
  bool     pending;              /* a started write has not been waited for */
  int      pending_nd;           /* descriptors of the pending write */
  uint32_t pending_start;        /* CCOUNT at its start */
  mutex_t  lock;
  sem_t    done;
  sem_t    te_sem;
  int      chan;
  uint32_t override_khz;
  uint32_t cur_prescale;
  uint32_t ccount_mhz;           /* CPU clock in MHz, for CCOUNT time */
  struct esp32s3_lcd_i80_stats_s stats;
  volatile uint32_t te_ccount;   /* CCOUNT at last TE edge */

  /* Internal-RAM DMA resources */

  struct esp32s3_dmadesc_s desc[LCD_I80_MAXROWS];
  struct esp32s3_dmadesc_s bdesc[2][LCD_I80_BDESC];
  uint8_t  bbuf[2][LCD_I80_BBUF] aligned_data(4);
  uint8_t  cmdbuf[LCD_I80_CMDBUF] aligned_data(4);
};

/****************************************************************************
 * Private Data
 ****************************************************************************/

static struct lcd_i80_s g_i80 =
{
  .lock   = NXMUTEX_INITIALIZER,
  .done   = SEM_INITIALIZER(0),
  .te_sem = SEM_INITIALIZER(0),
  .chan   = -1,
};

static const uint8_t g_data_pins[8] =
{
  CONFIG_ESP32S3_LCD_I80_D0_PIN, CONFIG_ESP32S3_LCD_I80_D1_PIN,
  CONFIG_ESP32S3_LCD_I80_D2_PIN, CONFIG_ESP32S3_LCD_I80_D3_PIN,
  CONFIG_ESP32S3_LCD_I80_D4_PIN, CONFIG_ESP32S3_LCD_I80_D5_PIN,
  CONFIG_ESP32S3_LCD_I80_D6_PIN, CONFIG_ESP32S3_LCD_I80_D7_PIN
};

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: i80_ccount / i80_us
 *
 * Description:
 *   Microsecond timestamps from the CPU cycle counter (the system tick is
 *   too coarse to time a 16 ms transfer).
 *
 ****************************************************************************/

static inline uint32_t i80_ccount(void)
{
  uint32_t ccount;

  __asm__ __volatile__("rsr %0, ccount" : "=a" (ccount));
  return ccount;
}

static inline uint32_t i80_us(uint32_t start)
{
  return (i80_ccount() - start) / g_i80.ccount_mhz;
}

/****************************************************************************
 * Name: i80_prescale
 *
 * Description:
 *   Nearest WR divider (LCD_CLK / n) for a clock in kHz.
 *
 ****************************************************************************/

static uint32_t i80_prescale(uint32_t khz)
{
  uint32_t n;

  if (khz == 0)
    {
      khz = 1;
    }

  n = (LCD_I80_CLK_KHZ + khz / 2) / khz;
  if (n < 2)
    {
      n = 2;
    }
  else if (n > 64)
    {
      n = 64;
    }

  return n;
}

static void i80_setclk(FAR struct lcd_i80_s *priv, uint32_t khz)
{
  uint32_t n = i80_prescale(khz);

  if (n != priv->cur_prescale)
    {
      lcd_ll_set_pixel_clock_prescale(LCD_I80_DEV, n);
      priv->cur_prescale = n;
    }

  priv->stats.pclk_khz = LCD_I80_CLK_KHZ / n;
}

/****************************************************************************
 * Name: i80_interrupt
 *
 * Description:
 *   LCD_CAM interrupt: the transaction (command, params, data) is done.
 *
 ****************************************************************************/

static int i80_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct lcd_i80_s *priv = arg;
  uint32_t status = lcd_ll_get_interrupt_status(LCD_I80_DEV);

  lcd_ll_clear_interrupt_status(LCD_I80_DEV, status);

  if ((status & LCD_LL_EVENT_TRANS_DONE) != 0)
    {
      nxsem_post(&priv->done);
    }

  return 0;
}

#if CONFIG_ESP32S3_LCD_I80_TE_PIN >= 0
static int i80_te_interrupt(int irq, FAR void *context, FAR void *arg)
{
  FAR struct lcd_i80_s *priv = arg;

  priv->te_ccount = i80_ccount();
  priv->stats.te_edges++;
  nxsem_post(&priv->te_sem);
  return 0;
}
#endif

/****************************************************************************
 * Name: i80_dma_check
 *
 * Description:
 *   After a transfer: GDMA error / underflow flags and unread descriptors.
 *
 ****************************************************************************/

static void i80_dma_check(FAR struct lcd_i80_s *priv,
                          FAR struct esp32s3_dmadesc_s *first, int ndesc)
{
  uint32_t raw = GET_GDMA_CH_REG(DMA_OUT_INT_RAW_CH0_REG, priv->chan);
  bool unread = false;
  int i;

  SET_GDMA_CH_REG(DMA_OUT_INT_CLR_CH0_REG, priv->chan, 0xff);

  for (i = 0; i < ndesc; i++)
    {
      if ((first[i].ctrl & ESP32S3_DMA_CTRL_OWN) != 0)
        {
          unread = true;
          break;
        }
    }

  if ((raw & LCD_I80_DMA_UNDERRUN) != 0)
    {
      priv->stats.underruns++;
    }

  if ((raw & LCD_I80_DMA_ERRORS) != 0)
    {
      priv->stats.dma_errors++;
      priv->stats.last_dmaerr = raw;
    }

  if (unread || (raw & LCD_I80_DMA_COMPLETE) != LCD_I80_DMA_COMPLETE)
    {
      int owned = 0;

      for (i = 0; i < ndesc; i++)
        {
          if ((first[i].ctrl & ESP32S3_DMA_CTRL_OWN) != 0)
            {
              owned++;
            }
        }

      priv->stats.short_xfers++;
      priv->stats.dbg[0] = raw;
      priv->stats.dbg[1] = (ndesc << 16) | owned;
      priv->stats.dbg[2] = GET_GDMA_CH_REG(DMA_OUT_DSCR_CH0_REG,
                                           priv->chan);
      priv->stats.dbg[3] = GET_GDMA_CH_REG(DMA_OUT_EOF_DES_ADDR_CH0_REG,
                                           priv->chan);
    }
}

/****************************************************************************
 * Name: i80_mkdesc
 *
 * Description:
 *   Fill a descriptor for len bytes at p, ending the chain when last.
 *
 ****************************************************************************/

static void i80_mkdesc(FAR struct esp32s3_dmadesc_s *d, FAR const uint8_t *p,
                       size_t len, bool last)
{
  d->ctrl = ESP32S3_DMA_CTRL_OWN |
            (last ? ESP32S3_DMA_CTRL_EOF : 0) |
            (len << ESP32S3_DMA_CTRL_DATALEN_S) |
            (len << ESP32S3_DMA_CTRL_BUFLEN_S);
  d->pbuf = p;
  d->next = last ? NULL : d + 1;
}

/****************************************************************************
 * Name: i80_kick
 *
 * Description:
 *   Start one LCD_CAM transaction: optional command, optional data from
 *   the descriptor list (swap: swap the bytes of each 16-bit pixel).
 *
 ****************************************************************************/

static void i80_kick(FAR struct lcd_i80_s *priv, int cmd,
                     FAR struct esp32s3_dmadesc_s *desc, bool swap)
{
  lcd_cam_dev_t *dev = LCD_I80_DEV;

  while (nxsem_trywait(&priv->done) == 0)
    {
    }

  lcd_ll_clear_interrupt_status(dev, UINT32_MAX);
  lcd_ll_enable_swizzle(dev, swap);

  if (cmd >= 0)
    {
      lcd_ll_set_command(dev, 8, cmd);
    }

  lcd_ll_set_phase_cycles(dev, cmd >= 0 ? 1 : 0, 0, desc != NULL ? 1 : 0);
  lcd_ll_set_blank_cycles(dev, 1, 1);

  if (desc != NULL)
    {
      SET_GDMA_CH_REG(DMA_OUT_INT_CLR_CH0_REG, priv->chan, 0xff);
      esp32s3_dma_load(desc, priv->chan, true);
      esp32s3_dma_enable(priv->chan, true);

      /* Let the DMA push the first data into the LCD FIFO.  The first
       * read from PSRAM takes far longer than from internal RAM (measured:
       * a start after 1 us ends the transaction at once), so wait longer.
       */

      up_udelay(esp32s3_ptr_extram(desc->pbuf) ?
                CONFIG_ESP32S3_LCD_I80_EXTSTART_US : 1);
    }

  lcd_ll_start(dev);
}

/****************************************************************************
 * Name: i80_wait
 *
 * Description:
 *   Wait for the transaction started by i80_kick.
 *
 ****************************************************************************/

static int i80_wait(FAR struct lcd_i80_s *priv)
{
  int ret;

  ret = nxsem_tickwait_uninterruptible(&priv->done,
                                       MSEC2TICK(LCD_I80_TIMEOUT_MS));
  if (ret < 0)
    {
      priv->stats.timeouts++;
      lcd_ll_stop(LCD_I80_DEV);
      lcd_ll_fifo_reset(LCD_I80_DEV);
      esp32s3_dma_reset_channel(priv->chan, true);
      lcderr("ERROR: i80 transfer timeout\n");
      return -ETIMEDOUT;
    }

  return OK;
}

/****************************************************************************
 * Name: i80_finish
 *
 * Description:
 *   Complete the pixel write that was started without waiting (async mode).
 *   Called with the lock held before anything else uses the bus, its
 *   descriptors or its clock.
 *
 ****************************************************************************/

static int i80_finish(FAR struct lcd_i80_s *priv)
{
  int ret;

  if (!priv->pending)
    {
      return OK;
    }

  priv->pending = false;
  ret = i80_wait(priv);
  i80_dma_check(priv, priv->desc, priv->pending_nd);
  priv->stats.last_xfer_us = i80_us(priv->pending_start);
  return ret;
}

/****************************************************************************
 * Name: i80_copy_rows
 *
 * Description:
 *   Copy len bytes of the packed stream of rows into a bounce buffer,
 *   continuing at (*row, *off) of the source.
 *
 ****************************************************************************/

static void i80_copy_rows(FAR uint8_t *dst, size_t len,
                          FAR const uint8_t *src, size_t row_bytes,
                          size_t stride, FAR size_t *row, FAR size_t *off)
{
  while (len > 0)
    {
      size_t n = row_bytes - *off;

      if (n > len)
        {
          n = len;
        }

      memcpy(dst, src + *row * stride + *off, n);
      dst += n;
      len -= n;
      *off += n;
      if (*off == row_bytes)
        {
          *off = 0;
          (*row)++;
        }
    }
}

/****************************************************************************
 * Name: i80_write_direct
 *
 * Description:
 *   One descriptor per row, read by GDMA straight from the caller buffer.
 *
 ****************************************************************************/

static int i80_write_direct(FAR struct lcd_i80_s *priv, int cmd,
                            FAR const uint8_t *buf, size_t row_bytes,
                            size_t rows, size_t stride)
{
  size_t per_row = (row_bytes + LCD_I80_PIECE_MAX - 1) / LCD_I80_PIECE_MAX;
  size_t chunk_rows = LCD_I80_MAXROWS / per_row;
  int ret = OK;

  /* Packed rows in internal RAM are one byte stream for the panel: describe
   * it in full pieces, so every descriptor but the last starts on a word
   * and holds a multiple of 4 bytes.  GDMA bursts corrupt a chain of
   * per-row descriptors whose rows start off a word (rows of an odd number
   * of pixels, as a software-rotated flush of an odd-height area makes).
   */

  if (stride == row_bytes && ((uintptr_t)buf & 3) == 0 &&
      !esp32s3_ptr_extram(buf))
    {
      size_t total = row_bytes * rows;
      size_t max_len = (size_t)LCD_I80_MAXROWS * LCD_I80_PIECE_MAX;

      while (total > 0 && ret == OK)
        {
          size_t len = total < max_len ? total : max_len;
          size_t left = len;
          FAR const uint8_t *p = buf;
          int nd = 0;

          while (left > 0)
            {
              size_t piece = left < LCD_I80_PIECE_MAX ?
                             left : LCD_I80_PIECE_MAX;

              left -= piece;
              i80_mkdesc(&priv->desc[nd], p, piece, left == 0);
              p += piece;
              nd++;
            }

          i80_kick(priv, cmd, priv->desc, true);
          if (priv->async && len == total)
            {
              priv->pending       = true;
              priv->pending_nd    = nd;
              priv->pending_start = i80_ccount();
              return OK;
            }

          ret = i80_wait(priv);
          i80_dma_check(priv, priv->desc, nd);

          cmd    = -1;
          buf   += len;
          total -= len;
        }

      return ret;
    }

  while (rows > 0 && ret == OK)
    {
      size_t n = rows < chunk_rows ? rows : chunk_rows;
      int nd = 0;
      size_t r;

      for (r = 0; r < n; r++)
        {
          FAR const uint8_t *row = buf + r * stride;
          size_t left = row_bytes;

          while (left > 0)
            {
              size_t piece = left < LCD_I80_PIECE_MAX ?
                             left : LCD_I80_PIECE_MAX;

              left -= piece;
              i80_mkdesc(&priv->desc[nd], row, piece,
                         r == n - 1 && left == 0);
              row += piece;
              nd++;
            }
        }

      /* PSRAM is read by GDMA, not through the CPU cache: write the cache
       * lines of the area back first.
       */

      if (esp32s3_ptr_extram(buf))
        {
          if (stride == row_bytes)
            {
              i80_cache_clean(buf, buf + n * row_bytes);
            }
          else if (stride == 0)
            {
              i80_cache_clean(buf, buf + row_bytes);
            }
          else
            {
              for (r = 0; r < n; r++)
                {
                  i80_cache_clean(buf + r * stride,
                                  buf + r * stride + row_bytes);
                }
            }
        }

      i80_kick(priv, cmd, priv->desc, true);
      if (priv->async && rows == n)
        {
          /* The last (usually the only) piece: leave it running; the next
           * bus user, or esp32s3_lcd_i80_sync(), waits for it.
           */

          priv->pending       = true;
          priv->pending_nd    = nd;
          priv->pending_start = i80_ccount();
          return OK;
        }

      ret = i80_wait(priv);
      i80_dma_check(priv, priv->desc, nd);

      cmd   = -1;
      buf  += n * stride;
      rows -= n;
    }

  return ret;
}

/****************************************************************************
 * Name: i80_write_bounce
 *
 * Description:
 *   Copy the data through two internal buffers.  The DMA of a buffer is
 *   started only when it is full; the other one is filled meanwhile.
 *
 ****************************************************************************/

static int i80_write_bounce(FAR struct lcd_i80_s *priv, int cmd,
                            int continue_cmd, FAR const uint8_t *buf,
                            size_t row_bytes, size_t rows, size_t stride)
{
  size_t max_bytes = LCD_I80_BBUF < LCD_I80_PIECE_MAX ?
                     LCD_I80_BBUF : LCD_I80_PIECE_MAX;
  size_t max_rows = max_bytes / row_bytes;
  size_t left = rows;
  size_t row = 0;
  size_t off = 0;
  size_t nrows[2];
  int cur = 0;
  int ret = OK;

  /* Keep each bounce transfer in one descriptor and on a row boundary.  The
   * panel write is split into LCD_CAM transactions, so every transaction
   * after the first starts with the panel's memory-write-continue command.
   */

  if (max_rows == 0)
    {
      return -E2BIG;
    }

  /* Prepare the first buffer */

  nrows[0] = left < max_rows ? left : max_rows;
  i80_copy_rows(priv->bbuf[0], nrows[0] * row_bytes, buf, row_bytes, stride,
                &row, &off);
  left -= nrows[0];

  while (nrows[cur] > 0)
    {
      size_t len = nrows[cur] * row_bytes;

      i80_mkdesc(&priv->bdesc[cur][0], priv->bbuf[cur], len, true);
      i80_kick(priv, cmd, priv->bdesc[cur], true);
      cmd = continue_cmd;

      /* Fill the other buffer while this one is on the bus */

      nrows[cur ^ 1] = left < max_rows ? left : max_rows;
      i80_copy_rows(priv->bbuf[cur ^ 1], nrows[cur ^ 1] * row_bytes, buf,
                    row_bytes, stride, &row, &off);
      left -= nrows[cur ^ 1];

      ret = i80_wait(priv);
      i80_dma_check(priv, priv->bdesc[cur], 1);
      if (ret < 0)
        {
          break;
        }

      cur ^= 1;
    }

  return ret;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: esp32s3_lcd_i80_initialize
 ****************************************************************************/

int esp32s3_lcd_i80_initialize(void)
{
  FAR struct lcd_i80_s *priv = &g_i80;
  lcd_cam_dev_t *dev = LCD_I80_DEV;
  int cpuint;
  int ret = OK;
  int i;

  nxmutex_lock(&priv->lock);
  if (priv->initialized)
    {
      goto out;
    }

  priv->ccount_mhz = esp_clk_cpu_freq() / 1000000;

  /* Peripheral, clock tree: PLL160 / 2 = 80 MHz LCD_CLK, WR = / prescale */

  periph_module_enable(PERIPH_LCD_CAM_MODULE);
  lcd_ll_reset(dev);
  lcd_ll_fifo_reset(dev);
  lcd_ll_enable_clock(dev, true);
  lcd_ll_select_clk_src(dev, LCD_CLK_SRC_PLL160M);
  lcd_ll_set_group_clock_coeff(dev, LCD_I80_CLK_DIV, 0, 0);

  /* i80 mode, 8-bit bus, data length controlled by the DMA */

  lcd_ll_enable_rgb_mode(dev, false);
  lcd_ll_set_dma_read_stride(dev, 8);
  lcd_ll_enable_output_always_on(dev, true);
  lcd_ll_reverse_dma_data_bit_order(dev, false);
  lcd_ll_swap_dma_data_byte_order(dev, false);
  lcd_ll_set_clock_idle_level(dev, true);
  lcd_ll_set_pixel_clock_edge(dev, false);
  lcd_ll_set_dc_level(dev, false, false, false, true);
  dev->lc_dma_int_ena.val = 0;
  lcd_ll_clear_interrupt_status(dev, UINT32_MAX);

  priv->cur_prescale = 0;
  i80_setclk(priv, CONFIG_ESP32S3_LCD_I80_PCLK_MHZ * 1000);

  /* GDMA: TX channel with burst, descriptors owner check and write-back */

  priv->chan = esp32s3_dma_request(ESP32S3_DMA_PERIPH_LCDCAM, 1, 1, true);
  if (priv->chan < 0)
    {
      lcderr("ERROR: no GDMA channel\n");
      ret = -ENODEV;
      goto out;
    }

  esp32s3_dma_set_ext_memblk(priv->chan, true,
#if CONFIG_ESP32S3_LCD_I80_EXTMEM_BLOCK == 64
                             ESP32S3_DMA_EXT_MEMBLK_64B
#elif CONFIG_ESP32S3_LCD_I80_EXTMEM_BLOCK == 32
                             ESP32S3_DMA_EXT_MEMBLK_32B
#else
                             ESP32S3_DMA_EXT_MEMBLK_16B
#endif
                            );
  SET_GDMA_CH_BITS(DMA_OUT_CONF1_CH0_REG, priv->chan,
                   DMA_OUT_CHECK_OWNER_CH0_M);
  SET_GDMA_CH_BITS(DMA_OUT_CONF0_CH0_REG, priv->chan,
                   DMA_OUT_AUTO_WRBACK_CH0_M);

  /* Interrupt */

  cpuint = esp_setup_irq(ESP32S3_PERIPH_LCD_CAM, ESP32S3_INT_PRIO_DEF,
                         ESP_IRQ_TRIGGER_LEVEL, i80_interrupt, priv);
  if (cpuint < 0)
    {
      lcderr("ERROR: no CPU interrupt\n");
      esp32s3_dma_release(priv->chan);
      ret = -ENOMEM;
      goto out;
    }

  up_enable_irq(ESP32S3_IRQ_LCD_CAM);
  dev->lc_dma_int_ena.val = LCD_LL_EVENT_TRANS_DONE;

  /* GPIOs: connect the peripheral signals through the GPIO matrix */

  for (i = 0; i < 8; i++)
    {
      esp_configgpio(g_data_pins[i], OUTPUT);
      esp_gpio_matrix_out(g_data_pins[i], LCD_DATA_OUT0_IDX + i, 0, 0);
    }

  esp_configgpio(CONFIG_ESP32S3_LCD_I80_DC_PIN, OUTPUT);
  esp_gpio_matrix_out(CONFIG_ESP32S3_LCD_I80_DC_PIN, LCD_DC_IDX, 0, 0);
  esp_configgpio(CONFIG_ESP32S3_LCD_I80_WR_PIN, OUTPUT);
  esp_gpio_matrix_out(CONFIG_ESP32S3_LCD_I80_WR_PIN, LCD_PCLK_IDX, 0, 0);

#if CONFIG_ESP32S3_LCD_I80_TE_PIN >= 0
  esp_configgpio(CONFIG_ESP32S3_LCD_I80_TE_PIN, INPUT | RISING);
  esp_gpio_irq(CONFIG_ESP32S3_LCD_I80_TE_PIN, i80_te_interrupt, priv);
  esp_gpioirqenable(CONFIG_ESP32S3_LCD_I80_TE_PIN);
#endif

#ifdef CONFIG_ESP32S3_LCD_I80_BOUNCE
  priv->bounce = true;
#endif

  priv->initialized = true;
  syslog(LOG_INFO, "i80: LCD_CAM ready, WR %" PRIu32 " kHz, GDMA ch %d\n",
         priv->stats.pclk_khz, priv->chan);

out:
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: esp32s3_lcd_i80_cmd
 ****************************************************************************/

int esp32s3_lcd_i80_cmd(uint8_t cmd, FAR const uint8_t *params, size_t len)
{
  FAR struct lcd_i80_s *priv = &g_i80;
  int ret;

  if (!priv->initialized || len > LCD_I80_CMDBUF)
    {
      return -EINVAL;
    }

  nxmutex_lock(&priv->lock);
  (void)i80_finish(priv);
  i80_setclk(priv, CONFIG_ESP32S3_LCD_I80_PCLK_MHZ * 1000);

  if (len > 0)
    {
      memcpy(priv->cmdbuf, params, len);
      i80_mkdesc(priv->desc, priv->cmdbuf, len, true);
      i80_kick(priv, cmd, priv->desc, false);
    }
  else
    {
      i80_kick(priv, cmd, NULL, false);
    }

  ret = i80_wait(priv);
  if (ret == OK && len > 0)
    {
      i80_dma_check(priv, priv->desc, 1);
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: esp32s3_lcd_i80_write
 ****************************************************************************/

int esp32s3_lcd_i80_write(int cmd, int continue_cmd,
                          FAR const uint8_t *buf, size_t row_bytes,
                          size_t rows, size_t stride)
{
  FAR struct lcd_i80_s *priv = &g_i80;
  uint32_t start;
  bool ext;
  int ret;

  if (!priv->initialized || buf == NULL || row_bytes == 0 || rows == 0 ||
      cmd < 0 || continue_cmd < 0 || (row_bytes & 1) != 0 ||
      (stride != 0 && stride < row_bytes))
    {
      return -EINVAL;
    }

  ext = esp32s3_ptr_extram(buf);

  nxmutex_lock(&priv->lock);
  (void)i80_finish(priv);

  if (priv->override_khz != 0)
    {
      i80_setclk(priv, priv->override_khz);
    }
  else if (ext && !priv->bounce)
    {
      i80_setclk(priv, CONFIG_ESP32S3_LCD_I80_PCLK_PSRAM_MHZ * 1000);
    }
  else
    {
      i80_setclk(priv, CONFIG_ESP32S3_LCD_I80_PCLK_MHZ * 1000);
    }

  start = i80_ccount();
  if (ext && priv->bounce)
    {
      ret = i80_write_bounce(priv, cmd, continue_cmd, buf, row_bytes, rows,
                             stride);
    }
  else
    {
      ret = i80_write_direct(priv, cmd, buf, row_bytes, rows, stride);
    }

  if (!priv->pending)
    {
      priv->stats.last_xfer_us = i80_us(start);
    }

  if (ret == OK)
    {
      priv->stats.transfers++;
      priv->stats.bytes += row_bytes * rows;
    }

  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: esp32s3_lcd_i80_set_pclk
 ****************************************************************************/

uint32_t esp32s3_lcd_i80_set_pclk(uint32_t khz)
{
  FAR struct lcd_i80_s *priv = &g_i80;

  nxmutex_lock(&priv->lock);
  priv->override_khz = khz;
  nxmutex_unlock(&priv->lock);

  if (khz == 0)
    {
      return CONFIG_ESP32S3_LCD_I80_PCLK_PSRAM_MHZ * 1000;
    }

  return LCD_I80_CLK_KHZ / i80_prescale(khz);
}

/****************************************************************************
 * Name: esp32s3_lcd_i80_set_async
 ****************************************************************************/

int esp32s3_lcd_i80_set_async(bool async)
{
  FAR struct lcd_i80_s *priv = &g_i80;
  int ret;

  nxmutex_lock(&priv->lock);
  ret = i80_finish(priv);
  priv->async = async;
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: esp32s3_lcd_i80_sync
 ****************************************************************************/

int esp32s3_lcd_i80_sync(void)
{
  FAR struct lcd_i80_s *priv = &g_i80;
  int ret;

  nxmutex_lock(&priv->lock);
  ret = i80_finish(priv);
  nxmutex_unlock(&priv->lock);
  return ret;
}

/****************************************************************************
 * Name: esp32s3_lcd_i80_set_bounce / get_bounce
 ****************************************************************************/

void esp32s3_lcd_i80_set_bounce(bool bounce)
{
  g_i80.bounce = bounce;
}

bool esp32s3_lcd_i80_get_bounce(void)
{
  return g_i80.bounce;
}

/****************************************************************************
 * Name: esp32s3_lcd_i80_wait_te
 ****************************************************************************/

int esp32s3_lcd_i80_wait_te(uint32_t timeout_ms)
{
#if CONFIG_ESP32S3_LCD_I80_TE_PIN >= 0
  FAR struct lcd_i80_s *priv = &g_i80;
  uint32_t start;
  int ret;

  if (!priv->initialized)
    {
      return -EINVAL;
    }

  while (nxsem_trywait(&priv->te_sem) == 0)
    {
    }

  start = i80_ccount();
  ret = nxsem_tickwait_uninterruptible(&priv->te_sem,
                                       MSEC2TICK(timeout_ms));
  priv->stats.last_te_us = i80_us(start);
  if (ret < 0)
    {
      priv->stats.te_timeouts++;
      return -ETIMEDOUT;
    }

  return (int)priv->stats.last_te_us;
#else
  return -ENOSYS;
#endif
}

/****************************************************************************
 * Name: esp32s3_lcd_i80_getstats
 ****************************************************************************/

void esp32s3_lcd_i80_getstats(FAR struct esp32s3_lcd_i80_stats_s *stats,
                              bool clear)
{
  FAR struct lcd_i80_s *priv = &g_i80;
  irqstate_t flags = enter_critical_section();

  *stats = priv->stats;
  if (clear)
    {
      uint32_t khz = priv->stats.pclk_khz;

      memset(&priv->stats, 0, sizeof(priv->stats));
      priv->stats.pclk_khz = khz;
    }

  leave_critical_section(flags);
}
