/****************************************************************************
 * drivers/wireless/lpwan/sx126x/sx126x.c
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <sys/types.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <poll.h>

#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/debug.h>
#include <nuttx/fs/fs.h>
#include <nuttx/mutex.h>
#include <nuttx/semaphore.h>
#include <nuttx/spi/spi.h>
#include <nuttx/wqueue.h>
#include <nuttx/wireless/ioctl.h>
#include <nuttx/wireless/lpwan/sx126x.h>

#include "sx126x.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Configuration ************************************************************/

#ifndef CONFIG_SCHED_HPWORK
#  error "The SX126x driver requires CONFIG_SCHED_HPWORK"
#endif

#if SX126X_NIOCTLS > SX126X_NCMDS
#  error "Increase SX126X_NCMDS in include/nuttx/wireless/ioctl.h"
#endif

#ifndef CONFIG_LPWAN_SX126X_MAX_DEVICES
#  define CONFIG_LPWAN_SX126X_MAX_DEVICES 1
#endif

#ifndef CONFIG_LPWAN_SX126X_RXFIFO_LEN
#  define CONFIG_LPWAN_SX126X_RXFIFO_LEN 4
#endif

#ifndef CONFIG_LPWAN_SX126X_NPOLLWAITERS
#  define CONFIG_LPWAN_SX126X_NPOLLWAITERS 2
#endif

#ifndef CONFIG_LPWAN_SX126X_SPIFREQ
#  define CONFIG_LPWAN_SX126X_SPIFREQ 4000000
#endif

#ifndef CONFIG_LPWAN_SX126X_RFFREQ_DEFAULT
#  define CONFIG_LPWAN_SX126X_RFFREQ_DEFAULT 915000000
#endif

#ifndef CONFIG_LPWAN_SX126X_SF_DEFAULT
#  define CONFIG_LPWAN_SX126X_SF_DEFAULT 7
#endif

#ifndef CONFIG_LPWAN_SX126X_BW_DEFAULT
#  define CONFIG_LPWAN_SX126X_BW_DEFAULT 125
#endif

#ifndef CONFIG_LPWAN_SX126X_CR_DEFAULT
#  define CONFIG_LPWAN_SX126X_CR_DEFAULT 5
#endif

#ifndef CONFIG_LPWAN_SX126X_TXPOWER_DEFAULT
#  define CONFIG_LPWAN_SX126X_TXPOWER_DEFAULT 14
#endif

#ifndef CONFIG_LPWAN_SX126X_PREAMBLE_DEFAULT
#  define CONFIG_LPWAN_SX126X_PREAMBLE_DEFAULT 8
#endif

#ifndef CONFIG_LPWAN_SX126X_SYNCWORD
#  define CONFIG_LPWAN_SX126X_SYNCWORD SX126X_LORA_SYNCWORD_PRIVATE
#endif

#ifndef CONFIG_LPWAN_SX126X_RX_TIMEOUT_DEFAULT
#  define CONFIG_LPWAN_SX126X_RX_TIMEOUT_DEFAULT 0
#endif

#ifndef CONFIG_LPWAN_SX126X_TX_TIMEOUT_MARGIN
#  define CONFIG_LPWAN_SX126X_TX_TIMEOUT_MARGIN 500
#endif

#ifdef CONFIG_LPWAN_SX126X_CRC_DEFAULT
#  define SX126X_DEFAULT_CRC true
#else
#  define SX126X_DEFAULT_CRC false
#endif

#if defined(CONFIG_LPWAN_SX126X_IDLE_RX)
#  define SX126X_DEFAULT_IDLE SX126X_IDLE_RX
#elif defined(CONFIG_LPWAN_SX126X_IDLE_SLEEP)
#  define SX126X_DEFAULT_IDLE SX126X_IDLE_SLEEP
#else
#  define SX126X_DEFAULT_IDLE SX126X_IDLE_STANDBY
#endif

#ifdef CONFIG_LPWAN_SX126X_RXBOOST
#  define SX126X_DEFAULT_RXBOOST true
#else
#  define SX126X_DEFAULT_RXBOOST false
#endif

/* BUSY handling.  Normal commands release BUSY within a few hundred us,
 * wake-up, reset and calibration take a few ms (plus the TCXO delay).
 */

#define SX126X_BUSY_TIMEOUT_US      10000
#define SX126X_BUSY_LONG_US         50000
#define SX126X_NOBUSY_DELAY_US      300    /* No BUSY pin: after commands */
#define SX126X_NOBUSY_LONG_US       6000   /* No BUSY pin: long operations */
#define SX126X_BUSY_POLL_US         10

/* Timing */

#define SX126X_SLEEP_SETTLE_US      500    /* After SetSleep */
#define SX126X_RX_GUARD_MS          100    /* Guard on top of RX timeouts */
#define SX126X_CAD_GUARD_MS         50     /* Guard on top of CAD time */
#define SX126X_RTC_STEPS_PER_MS     64     /* 15.625 us steps */
#define SX126X_TIMEOUT_MAX_STEPS    0xfffffe
#define SX126X_TIMEOUT_MAX_MS       (SX126X_TIMEOUT_MAX_STEPS / \
                                     SX126X_RTC_STEPS_PER_MS)
#define SX126X_RX_CONTINUOUS_STEPS  0xffffff
#define SX126X_LDRO_TSYM_US         16380  /* LDRO needed above this */
#define SX126X_TX_MARGIN_TICKS \
  MSEC2TICK(CONFIG_LPWAN_SX126X_TX_TIMEOUT_MARGIN)

/* Frequency limits of the SX126x */

#define SX126X_FREQ_MIN             150000000
#define SX126X_FREQ_MAX             960000000

/* TX power limits */

#define SX1262_TXPOWER_MIN          (-9)
#define SX1262_TXPOWER_MAX          22
#define SX1261_TXPOWER_MIN          (-17)
#define SX1261_TXPOWER_MAX          14

/* IRQs enabled in the chip and routed to DIO1 */

#define SX126X_IRQ_ENABLED \
  (SX126X_IRQ_TXDONE_MASK | SX126X_IRQ_RXDONE_MASK | \
   SX126X_IRQ_HEADERERR_MASK | SX126X_IRQ_CRCERR_MASK | \
   SX126X_IRQ_CADDONE_MASK | SX126X_IRQ_CADDETECTED_MASK | \
   SX126X_IRQ_TIMEOUT_MASK)

#define SX126X_IRQ_DIO1 \
  (SX126X_IRQ_TXDONE_MASK | SX126X_IRQ_RXDONE_MASK | \
   SX126X_IRQ_HEADERERR_MASK | SX126X_IRQ_CADDONE_MASK | \
   SX126X_IRQ_TIMEOUT_MASK)

/* Max rounds of IRQ status processing per DIO1 event */

#define SX126X_IRQ_MAX_ROUNDS       4

/****************************************************************************
 * Private Types
 ****************************************************************************/

struct sx126x_dev_s
{
  FAR struct spi_dev_s *spi;
  FAR const struct sx126x_lower_s *lower;

  mutex_t lock;                 /* Driver state and chip access */
  mutex_t oplock;               /* Serializes TX and CAD */
  sem_t rx_sem;                 /* Wakes blocked readers */
  sem_t tx_sem;                 /* TxDone */
  sem_t cad_sem;                /* CadDone */
  struct work_s irq_work;       /* DIO1 work */
  uint8_t crefs;                /* Number of opens */

  /* Chip state */

  uint8_t state;                /* enum sx126x_state_e */
  bool need_init;               /* Full init needed on next open */
  bool skip_reset;              /* Attach without reset pending */
  bool sleep_cold;              /* Last sleep was a cold start sleep */
  bool rx_hw_timeout;           /* Current single RX has a timeout */
  bool rx_timedout;             /* Single RX ended with a timeout */
  bool tx_busy;                 /* TX in progress */
  bool tx_done;                 /* TxDone seen */
  bool cad_done;                /* CadDone seen */
  bool cad_detected;            /* CadDetected seen */
  uint8_t cad_exit;             /* Exit mode of the running CAD */
  int8_t image_band;            /* Band of the last image calibration */
  uint8_t rx_waiters;           /* Readers blocked in read() */
  uint32_t duty_rx_us;          /* Last RX duty cycle parameters */
  uint32_t duty_sleep_us;

  /* Board configuration that ioctls can override */

  enum sx126x_device_e model;
  uint8_t hpmax;
  uint8_t padutycycle;
  uint8_t regulator;            /* enum sx126x_regulator_mode_e */
  uint8_t tcxo_voltage;         /* enum sx126x_tcxo_voltage_e */
  uint32_t tcxo_delay;          /* 15.625 us steps, 0 = no TCXO */
  bool dio2_rfsw;

  /* Radio configuration */

  uint8_t idle;                 /* SX126X_IDLE_* */
  bool lbt;
  bool rxboost;
  bool invert_iq;
  uint32_t frequency_hz;
  int8_t power;
  uint16_t preambles;
  uint8_t fixed_len;            /* Payload length in implicit header mode */
  uint8_t lora_sf;              /* enum sx126x_lora_sf_e */
  uint8_t lora_bw;              /* enum sx126x_lora_bw_e */
  uint8_t lora_cr;              /* enum sx126x_lora_cr_e */
  bool lora_crc;
  bool lora_fixed_header;
  bool lora_ldro;               /* Forced LDRO, else automatic */
  uint16_t syncword;            /* Registers 0x0740 (MSB) / 0x0741 */
  uint32_t rx_timeout_ms;       /* read() timeout, 0 = none */

  /* Status */

  uint16_t irq_latched;
  uint32_t rx_packets;
  uint32_t rx_crcerr;
  uint32_t rx_overruns;
  uint32_t rx_timeouts;
  uint32_t tx_packets;
  uint32_t tx_timeouts;

  /* RX FIFO */

  uint8_t fifo_head;
  uint8_t fifo_count;
  struct sx126x_read_hdr_s fifo[CONFIG_LPWAN_SX126X_RXFIFO_LEN];

  /* Poll waiters */

  FAR struct pollfd *fds[CONFIG_LPWAN_SX126X_NPOLLWAITERS];
};

/* LoRa bandwidth table */

struct sx126x_bw_s
{
  uint8_t  bw;                  /* enum sx126x_lora_bw_e */
  uint32_t hz;
};

/****************************************************************************
 * Private Function Prototypes
 ****************************************************************************/

/* File operations */

static int     sx126x_open(FAR struct file *filep);
static int     sx126x_close(FAR struct file *filep);
static ssize_t sx126x_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen);
static ssize_t sx126x_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen);
static int     sx126x_ioctl(FAR struct file *filep, int cmd,
                            unsigned long arg);
static int     sx126x_poll(FAR struct file *filep, FAR struct pollfd *fds,
                           bool setup);

/* Chip control */

static void sx126x_start_rx(FAR struct sx126x_dev_s *dev,
                            uint32_t timeout_ms);
static void sx126x_set_sleep(FAR struct sx126x_dev_s *dev, bool warm);
static void sx126x_hw_init(FAR struct sx126x_dev_s *dev, bool calibrate);

/****************************************************************************
 * Private Data
 ****************************************************************************/

static const struct file_operations g_sx126x_fops =
{
  sx126x_open,    /* open */
  sx126x_close,   /* close */
  sx126x_read,    /* read */
  sx126x_write,   /* write */
  NULL,           /* seek */
  sx126x_ioctl,   /* ioctl */
  NULL,           /* mmap */
  NULL,           /* truncate */
  sx126x_poll     /* poll */
};

static const struct sx126x_bw_s g_sx126x_bw[] =
{
  { SX126X_LORA_BW_7,   7810   },
  { SX126X_LORA_BW_10,  10420  },
  { SX126X_LORA_BW_15,  15630  },
  { SX126X_LORA_BW_20,  20830  },
  { SX126X_LORA_BW_31,  31250  },
  { SX126X_LORA_BW_41,  41670  },
  { SX126X_LORA_BW_62,  62500  },
  { SX126X_LORA_BW_125, 125000 },
  { SX126X_LORA_BW_250, 250000 },
  { SX126X_LORA_BW_500, 500000 }
};

#define SX126X_NBW (sizeof(g_sx126x_bw) / sizeof(g_sx126x_bw[0]))

static struct sx126x_dev_s g_sx126x_devices[CONFIG_LPWAN_SX126X_MAX_DEVICES];

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Helpers
 ****************************************************************************/

static uint32_t sx126x_bw_hz(uint8_t bw)
{
  size_t i;

  for (i = 0; i < SX126X_NBW; i++)
    {
      if (g_sx126x_bw[i].bw == bw)
        {
          return g_sx126x_bw[i].hz;
        }
    }

  return 125000;
}

static uint8_t sx126x_bw_from_hz(uint32_t hz)
{
  uint32_t best_diff = UINT32_MAX;
  uint8_t best = SX126X_LORA_BW_125;
  uint32_t diff;
  size_t i;

  for (i = 0; i < SX126X_NBW; i++)
    {
      diff = hz > g_sx126x_bw[i].hz ? hz - g_sx126x_bw[i].hz :
                                      g_sx126x_bw[i].hz - hz;
      if (diff < best_diff)
        {
          best_diff = diff;
          best = g_sx126x_bw[i].bw;
        }
    }

  return best;
}

static bool sx126x_bw_valid(uint8_t bw)
{
  size_t i;

  for (i = 0; i < SX126X_NBW; i++)
    {
      if (g_sx126x_bw[i].bw == bw)
        {
          return true;
        }
    }

  return false;
}

/* Symbol time in microseconds */

static uint32_t sx126x_tsym_us(FAR struct sx126x_dev_s *dev)
{
  return (uint32_t)(((uint64_t)1000000 << dev->lora_sf) /
                    sx126x_bw_hz(dev->lora_bw));
}

static bool sx126x_ldro(FAR struct sx126x_dev_s *dev)
{
  return dev->lora_ldro || sx126x_tsym_us(dev) >= SX126X_LDRO_TSYM_US;
}

/* Time on air, datasheet 6.1.4 */

static uint32_t sx126x_airtime_us(FAR struct sx126x_dev_s *dev,
                                  uint32_t len)
{
  uint64_t tsym_ns;
  uint32_t sf = dev->lora_sf;
  uint32_t nsym_q4;
  uint32_t den;
  int32_t  num;

  tsym_ns = ((uint64_t)1000000000 << sf) / sx126x_bw_hz(dev->lora_bw);

  num = 8 * (int32_t)len + (dev->lora_crc ? 16 : 0) - 4 * (int32_t)sf +
        (dev->lora_fixed_header ? 0 : 20);

  if (sf <= 6)
    {
      nsym_q4 = dev->preambles * 4 + 25;    /* Npreamble + 6.25 */
      den     = 4 * sf;
    }
  else
    {
      nsym_q4 = dev->preambles * 4 + 17;    /* Npreamble + 4.25 */
      num    += 8;
      den     = sx126x_ldro(dev) ? 4 * (sf - 2) : 4 * sf;
    }

  if (num < 0)
    {
      num = 0;
    }

  nsym_q4 += 8 * 4 + 4 * (((uint32_t)num + den - 1) / den) *
                     (dev->lora_cr + 4);

  return (uint32_t)((nsym_q4 * tsym_ns / 4 + 999) / 1000);
}

static uint32_t sx126x_ms_to_steps(uint32_t ms)
{
  if (ms > SX126X_TIMEOUT_MAX_MS)
    {
      ms = SX126X_TIMEOUT_MAX_MS;
    }

  return ms * SX126X_RTC_STEPS_PER_MS;
}

static uint32_t sx126x_us_to_steps(uint32_t us)
{
  uint64_t steps = ((uint64_t)us * SX126X_RTC_STEPS_PER_MS + 999) / 1000;

  if (steps == 0)
    {
      steps = 1;
    }
  else if (steps > SX126X_RX_CONTINUOUS_STEPS)
    {
      steps = SX126X_RX_CONTINUOUS_STEPS;
    }

  return (uint32_t)steps;
}

static void sx126x_put_be24(FAR uint8_t *buf, uint32_t val)
{
  buf[0] = (uint8_t)(val >> 16);
  buf[1] = (uint8_t)(val >> 8);
  buf[2] = (uint8_t)val;
}

static uint8_t sx126x_rx_len(FAR struct sx126x_dev_s *dev)
{
  if (dev->lora_fixed_header && dev->fixed_len != 0)
    {
      return dev->fixed_len;
    }

  return SX126X_MAX_PAYLOAD;
}

static void sx126x_power_limits(FAR struct sx126x_dev_s *dev,
                                FAR int8_t *min, FAR int8_t *max)
{
  if (dev->model == SX1261)
    {
      *min = SX1261_TXPOWER_MIN;
      *max = SX1261_TXPOWER_MAX;
    }
  else
    {
      *min = SX1262_TXPOWER_MIN;
      *max = SX1262_TXPOWER_MAX;
    }
}

static int8_t sx126x_clamp_power(FAR struct sx126x_dev_s *dev, int power)
{
  int8_t min;
  int8_t max;
  int8_t pwr;

  sx126x_power_limits(dev, &min, &max);
  pwr = power < min ? min : (power > max ? max : power);

  if (dev->lower->limit_tx_power != NULL)
    {
      dev->lower->limit_tx_power(&pwr);
    }

  return pwr;
}

static bool sx126x_receiving(FAR struct sx126x_dev_s *dev)
{
  return dev->state == SX126X_STATE_RX_CONT ||
         dev->state == SX126X_STATE_RX_SINGLE ||
         dev->state == SX126X_STATE_RX_DUTY;
}

/****************************************************************************
 * SPI and BUSY
 ****************************************************************************/

static int sx126x_wait_busy(FAR struct sx126x_dev_s *dev,
                            uint32_t timeout_us)
{
  if (dev->lower->busy == NULL)
    {
      up_udelay(timeout_us > SX126X_BUSY_TIMEOUT_US ?
                SX126X_NOBUSY_LONG_US : SX126X_NOBUSY_DELAY_US);
      return OK;
    }

  while (dev->lower->busy())
    {
      if (timeout_us < SX126X_BUSY_POLL_US)
        {
          wlerr("SX126x BUSY timeout\n");
          return -ETIMEDOUT;
        }

      up_udelay(SX126X_BUSY_POLL_US);
      timeout_us -= SX126X_BUSY_POLL_US;
    }

  return OK;
}

static void sx126x_spi_begin(FAR struct sx126x_dev_s *dev)
{
  SPI_LOCK(dev->spi, true);
  SPI_SETMODE(dev->spi, SPIDEV_MODE0);
  SPI_SETBITS(dev->spi, 8);
  SPI_SETFREQUENCY(dev->spi, CONFIG_LPWAN_SX126X_SPIFREQ);
  SPI_SELECT(dev->spi, SPIDEV_LPWAN(dev->lower->dev_number), true);
}

static void sx126x_spi_end(FAR struct sx126x_dev_s *dev)
{
  SPI_SELECT(dev->spi, SPIDEV_LPWAN(dev->lower->dev_number), false);
  SPI_LOCK(dev->spi, false);
}

/* Send a command.  'returns' receives the bytes clocked out while the
 * parameters (or NOPs when params is NULL) are sent.
 */

static void sx126x_command(FAR struct sx126x_dev_s *dev, uint8_t cmd,
                           FAR const uint8_t *params, size_t paramslen,
                           FAR uint8_t *returns)
{
  uint8_t val;
  size_t i;

  sx126x_wait_busy(dev, SX126X_BUSY_TIMEOUT_US);
  sx126x_spi_begin(dev);

  SPI_SEND(dev->spi, cmd);

  for (i = 0; i < paramslen; i++)
    {
      val = SPI_SEND(dev->spi, params != NULL ? params[i] : SX126X_NOP);
      if (returns != NULL)
        {
          returns[i] = val;
        }
    }

  sx126x_spi_end(dev);
}

static void sx126x_command1(FAR struct sx126x_dev_s *dev, uint8_t cmd,
                            uint8_t param)
{
  sx126x_command(dev, cmd, &param, 1, NULL);
}

static void sx126x_write_register(FAR struct sx126x_dev_s *dev,
                                  uint16_t address,
                                  FAR const uint8_t *data, size_t len)
{
  size_t i;

  sx126x_wait_busy(dev, SX126X_BUSY_TIMEOUT_US);
  sx126x_spi_begin(dev);

  SPI_SEND(dev->spi, SX126X_WRITEREGISTER);
  SPI_SEND(dev->spi, (uint8_t)(address >> 8));
  SPI_SEND(dev->spi, (uint8_t)address);

  for (i = 0; i < len; i++)
    {
      SPI_SEND(dev->spi, data[i]);
    }

  sx126x_spi_end(dev);
}

static void sx126x_read_register(FAR struct sx126x_dev_s *dev,
                                 uint16_t address,
                                 FAR uint8_t *data, size_t len)
{
  size_t i;

  sx126x_wait_busy(dev, SX126X_BUSY_TIMEOUT_US);
  sx126x_spi_begin(dev);

  SPI_SEND(dev->spi, SX126X_READREGISTER);
  SPI_SEND(dev->spi, (uint8_t)(address >> 8));
  SPI_SEND(dev->spi, (uint8_t)address);
  SPI_SEND(dev->spi, SX126X_NOP);             /* Status */

  for (i = 0; i < len; i++)
    {
      data[i] = SPI_SEND(dev->spi, SX126X_NOP);
    }

  sx126x_spi_end(dev);
}

static uint8_t sx126x_read_reg8(FAR struct sx126x_dev_s *dev,
                                uint16_t address)
{
  uint8_t val = 0;

  sx126x_read_register(dev, address, &val, 1);
  return val;
}

static void sx126x_write_reg8(FAR struct sx126x_dev_s *dev,
                              uint16_t address, uint8_t val)
{
  sx126x_write_register(dev, address, &val, 1);
}

static void sx126x_write_buffer(FAR struct sx126x_dev_s *dev,
                                uint8_t offset,
                                FAR const uint8_t *data, size_t len)
{
  size_t i;

  sx126x_wait_busy(dev, SX126X_BUSY_TIMEOUT_US);
  sx126x_spi_begin(dev);

  SPI_SEND(dev->spi, SX126X_WRITEBUFFER);
  SPI_SEND(dev->spi, offset);

  for (i = 0; i < len; i++)
    {
      SPI_SEND(dev->spi, data[i]);
    }

  sx126x_spi_end(dev);
}

static void sx126x_read_buffer(FAR struct sx126x_dev_s *dev,
                               uint8_t offset,
                               FAR uint8_t *data, size_t len)
{
  size_t i;

  sx126x_wait_busy(dev, SX126X_BUSY_TIMEOUT_US);
  sx126x_spi_begin(dev);

  SPI_SEND(dev->spi, SX126X_READBUFFER);
  SPI_SEND(dev->spi, offset);
  SPI_SEND(dev->spi, SX126X_NOP);             /* Status */

  for (i = 0; i < len; i++)
    {
      data[i] = SPI_SEND(dev->spi, SX126X_NOP);
    }

  sx126x_spi_end(dev);
}

/* Wake the chip from sleep: a NSS falling edge wakes it up, then wait
 * for BUSY to go low.  The chip ends up in STDBY_RC.  Harmless when the
 * chip is already awake.
 */

static void sx126x_wakeup(FAR struct sx126x_dev_s *dev)
{
  sx126x_spi_begin(dev);
  SPI_SEND(dev->spi, SX126X_CMD_GETSTATUS);
  SPI_SEND(dev->spi, SX126X_NOP);
  sx126x_spi_end(dev);

  sx126x_wait_busy(dev, SX126X_BUSY_LONG_US +
                   dev->tcxo_delay * 16);    /* steps -> ~us */
}

/****************************************************************************
 * Chip commands
 ****************************************************************************/

static uint8_t sx126x_get_chipmode(FAR struct sx126x_dev_s *dev)
{
  uint8_t ret = 0;

  sx126x_command(dev, SX126X_CMD_GETSTATUS, NULL, 1, &ret);
  return (ret & SX126X_STATUS_CHIPMODE_MASK) >> SX126X_STATUS_CHIPMODE_SHIFT;
}

static void sx126x_set_standby(FAR struct sx126x_dev_s *dev,
                               enum sx126x_standby_mode_e mode)
{
  sx126x_command1(dev, SX126X_SETSTANDBY, (uint8_t)mode);
}

static void sx126x_set_tx(FAR struct sx126x_dev_s *dev, uint32_t steps)
{
  uint8_t params[SX126X_SETTX_PARAMS];

  sx126x_put_be24(params, steps);
  sx126x_command(dev, SX126X_SETTX, params, sizeof(params), NULL);
}

static void sx126x_set_rx(FAR struct sx126x_dev_s *dev, uint32_t steps)
{
  uint8_t params[SX126X_SETRX_PARAMS];

  sx126x_put_be24(params, steps);
  sx126x_command(dev, SX126X_SETRX, params, sizeof(params), NULL);
}

static void sx126x_set_rx_duty_cycle(FAR struct sx126x_dev_s *dev,
                                     uint32_t rx_steps,
                                     uint32_t sleep_steps)
{
  uint8_t params[SX126X_SETRXDUTYCYCLE_PARAMS];

  sx126x_put_be24(params + SX126X_SETRXDUTYCYCLE_RXPERIOD_PARAM, rx_steps);
  sx126x_put_be24(params + SX126X_SETRXDUTYCYCLE_SLEEPPERIOD_PARAM,
                  sleep_steps);
  sx126x_command(dev, SX126X_SETRXDUTYCYCLE, params, sizeof(params), NULL);
}

static void sx126x_calibrate(FAR struct sx126x_dev_s *dev, uint8_t blocks)
{
  sx126x_command1(dev, SX126X_CALIBRATE, blocks);
  sx126x_wait_busy(dev, SX126X_BUSY_LONG_US);
}

static void sx126x_calibrate_image(FAR struct sx126x_dev_s *dev,
                                   uint32_t freq)
{
  uint8_t params[2];
  int8_t band;

  if (freq > 900000000)
    {
      band = 4;
      params[0] = 0xe1;
      params[1] = 0xe9;
    }
  else if (freq > 850000000)
    {
      band = 3;
      params[0] = 0xd7;
      params[1] = 0xdb;
    }
  else if (freq > 770000000)
    {
      band = 2;
      params[0] = 0xc1;
      params[1] = 0xc5;
    }
  else if (freq > 460000000)
    {
      band = 1;
      params[0] = 0x75;
      params[1] = 0x81;
    }
  else
    {
      band = 0;
      params[0] = 0x6b;
      params[1] = 0x6f;
    }

  if (band == dev->image_band)
    {
      return;
    }

  sx126x_command(dev, SX126X_CALIBRATEIMAGE, params, sizeof(params), NULL);
  sx126x_wait_busy(dev, SX126X_BUSY_LONG_US);
  dev->image_band = band;
}

static void sx126x_set_pa_config(FAR struct sx126x_dev_s *dev)
{
  uint8_t params[SX126X_SETPACONFIG_PARMS];

  params[SX126X_SETPACONFIG_PADUTYCYCLE_PARAM] = dev->padutycycle;
  params[SX126X_SETPACONFIG_HPMAX_PARAM]       = dev->hpmax;
  params[SX126X_SETPACONFIG_DEVICESEL_PARAM]   = (uint8_t)dev->model;
  params[SX126X_SETPACONFIG_PALUT_PARAM]       = 0x01;

  sx126x_command(dev, SX126X_SETPACONFIG, params, sizeof(params), NULL);
}

static void sx126x_set_tx_params(FAR struct sx126x_dev_s *dev)
{
  uint8_t params[SX126X_SETTXPARMS_PARAMS];

  params[SX126X_SETTXPARMS_POWER_PARAM]    = (uint8_t)dev->power;
  params[SX126X_SETTXPARMS_RAMPTIME_PARAM] = dev->lower->tx_ramp_time;

  sx126x_command(dev, SX126X_SETTXPARMS, params, sizeof(params), NULL);
}

static void sx126x_set_dio_irq_params(FAR struct sx126x_dev_s *dev)
{
  uint8_t params[SX126X_SETDIOIRQPARAMS_PARAMS];
  uint16_t irq  = SX126X_IRQ_ENABLED | dev->lower->masks.dio1_mask;
  uint16_t dio1 = SX126X_IRQ_DIO1 | dev->lower->masks.dio1_mask;
  uint16_t dio2 = dev->dio2_rfsw ? 0 : dev->lower->masks.dio2_mask;
  uint16_t dio3 = dev->tcxo_delay != 0 ? 0 : dev->lower->masks.dio3_mask;

  irq |= dio2 | dio3;

  params[0] = (uint8_t)(irq >> 8);
  params[1] = (uint8_t)irq;
  params[2] = (uint8_t)(dio1 >> 8);
  params[3] = (uint8_t)dio1;
  params[4] = (uint8_t)(dio2 >> 8);
  params[5] = (uint8_t)dio2;
  params[6] = (uint8_t)(dio3 >> 8);
  params[7] = (uint8_t)dio3;

  sx126x_command(dev, SX126X_SETDIOIRQPARAMS, params, sizeof(params), NULL);
}

static void sx126x_set_dio3_as_tcxo(FAR struct sx126x_dev_s *dev)
{
  uint8_t params[SX126X_SETDIO3TCXOCTRL_PARAMS];

  params[SX126X_SETDIO3TCXOCTRL_TCXO_V_PARAM] = dev->tcxo_voltage;
  sx126x_put_be24(params + SX126X_SETDIO3TCXOCTRL_DELAY_PARAM,
                  dev->tcxo_delay);
  sx126x_command(dev, SX126X_SETDIO3TCXOCTRL, params, sizeof(params), NULL);
}

static uint16_t sx126x_get_irq_status(FAR struct sx126x_dev_s *dev)
{
  uint8_t rets[SX126X_GETIRQSTATUS_RETURNS];

  sx126x_command(dev, SX126X_GETIRQSTATUS, NULL, sizeof(rets), rets);
  return ((uint16_t)rets[1] << 8) | rets[2];
}

static void sx126x_clear_irq_status(FAR struct sx126x_dev_s *dev,
                                    uint16_t bits)
{
  uint8_t params[SX126X_CLEARIRQSTATUS_PARAMS];

  params[0] = (uint8_t)(bits >> 8);
  params[1] = (uint8_t)bits;
  sx126x_command(dev, SX126X_CLEARIRQSTATUS, params, sizeof(params), NULL);
}

static uint16_t sx126x_get_device_errors(FAR struct sx126x_dev_s *dev)
{
  uint8_t rets[SX126X_GETDEVICEERRORS_RETURNS];

  sx126x_command(dev, SX126X_GETDEVICEERRORS, NULL, sizeof(rets), rets);
  return ((uint16_t)rets[1] << 8) | rets[2];
}

static void sx126x_clear_device_errors(FAR struct sx126x_dev_s *dev)
{
  sx126x_command(dev, SX126X_CLEARDEVICEERRORS, NULL,
                 SX126X_CLEARDEVICEERRORS_NOPS, NULL);
}

static void sx126x_set_rf_frequency(FAR struct sx126x_dev_s *dev)
{
  uint8_t params[SX126X_SETRFFREQUENCY_PARAMS];
  uint32_t steps;

  /* RfFreq = freq * 2^25 / Fxtal */

  steps = (uint32_t)(((uint64_t)dev->frequency_hz << 25) / SX126X_FXTAL);

  params[0] = (uint8_t)(steps >> 24);
  params[1] = (uint8_t)(steps >> 16);
  params[2] = (uint8_t)(steps >> 8);
  params[3] = (uint8_t)steps;

  sx126x_command(dev, SX126X_SETRFFREQUENCY, params, sizeof(params), NULL);
}

static void sx126x_set_modulation_params(FAR struct sx126x_dev_s *dev)
{
  uint8_t params[4];

  params[SX126X_MODPARAM1_LORA_SF_PARAM]              = dev->lora_sf;
  params[SX126X_MODPARAM2_LORA_BW_PARAM]              = dev->lora_bw;
  params[SX126X_MODPARAM3_LORA_CR_PARAM]              = dev->lora_cr;
  params[SX126X_MODPARAM4_LORA_LOWDATRATE_OPTI_PARAM] = sx126x_ldro(dev);

  sx126x_command(dev, SX126X_SETMODULATIONPARAMS, params, sizeof(params),
                 NULL);
}

static void sx126x_set_packet_params(FAR struct sx126x_dev_s *dev,
                                     uint8_t payload_len)
{
  uint8_t params[6];
  uint8_t reg;

  params[0] = (uint8_t)(dev->preambles >> 8);
  params[1] = (uint8_t)dev->preambles;
  params[SX126X_PKTPARAM3_LORA_HEADERTYPE_PARAM] = dev->lora_fixed_header;
  params[SX126X_PKTPARAM4_LORA_PAYLOADLEN_PARAM] = payload_len;
  params[SX126X_PKTPARAM5_LORA_CRCTYPE_PARAM]    = dev->lora_crc;
  params[SX126X_PKTPARAM6_LORA_INVERTIQ_PARAM]   = dev->invert_iq;

  sx126x_command(dev, SX126X_SETPACKETPARMS, params, sizeof(params), NULL);

  /* Errata 15.4: optimize the inverted IQ operation */

  reg = sx126x_read_reg8(dev, SX126X_REG_IQ_POLARITY);
  if (dev->invert_iq)
    {
      reg &= ~0x04;
    }
  else
    {
      reg |= 0x04;
    }

  sx126x_write_reg8(dev, SX126X_REG_IQ_POLARITY, reg);
}

static void sx126x_set_syncword(FAR struct sx126x_dev_s *dev)
{
  uint8_t sw[2];

  sw[0] = (uint8_t)(dev->syncword >> 8);
  sw[1] = (uint8_t)dev->syncword;
  sx126x_write_register(dev, SX126X_REG_LR_SYNCWORD, sw, sizeof(sw));
}

static void sx126x_get_rx_buffer_status(FAR struct sx126x_dev_s *dev,
                                        FAR uint8_t *len,
                                        FAR uint8_t *offset)
{
  uint8_t rets[SX126X_GETRXBUFFERSTATUS_RETURNS];

  sx126x_command(dev, SX126X_GETRXBUFFERSTATUS, NULL, sizeof(rets), rets);
  *len    = rets[SX126X_GETRXBUFFERSTATUS_PAYLOAD_LEN_RETURN];
  *offset = rets[SX126X_GETRXBUFFERSTATUS_RX_START_PTR_RETURN];
}

static void sx126x_get_packet_status(FAR struct sx126x_dev_s *dev,
                                     FAR struct sx126x_read_hdr_s *hdr)
{
  uint8_t rets[SX126X_GETPACKETSTATUS_RETURNS];

  sx126x_command(dev, SX126X_GETPACKETSTATUS, NULL, sizeof(rets), rets);

  hdr->rssi = -(int16_t)(rets[SX126X_GETPACKETSTATUS_RSSIPKT_RETURN] / 2);
  hdr->snr  = (int8_t)((int8_t)rets[SX126X_GETPACKETSTATUS_SNRPKT_RETURN] /
                       4);
  hdr->signal_rssi =
    -(int16_t)(rets[SX126X_GETPACKETSTATUS_SIGRSSI_RETURN] / 2);
}

static int16_t sx126x_get_rssi_inst(FAR struct sx126x_dev_s *dev)
{
  uint8_t rets[SX126X_GETRSSIINST_RETURNS];

  sx126x_command(dev, SX126X_GETRSSIINST, NULL, sizeof(rets), rets);
  return -(int16_t)(rets[SX126X_GETRSSIINST_RSSI_RETURN] / 2);
}

/****************************************************************************
 * Radio state handling (dev->lock held)
 ****************************************************************************/

/* Make sure the chip can take commands.  After a sleep the chip is woken
 * up and re-initialized (calibrated too after a cold start).
 */

static void sx126x_ensure_awake(FAR struct sx126x_dev_s *dev)
{
  uint8_t prev = dev->state;

  if (prev != SX126X_STATE_SLEEP && prev != SX126X_STATE_RX_DUTY &&
      prev != SX126X_STATE_UNKNOWN)
    {
      return;
    }

  sx126x_wakeup(dev);
  dev->state = SX126X_STATE_STANDBY;

  if (prev == SX126X_STATE_SLEEP)
    {
      sx126x_hw_init(dev, dev->sleep_cold);
    }
}

/* Go to STDBY_RC, aborting any RX/TX/CAD */

static void sx126x_standby(FAR struct sx126x_dev_s *dev)
{
  sx126x_ensure_awake(dev);
  if (dev->state != SX126X_STATE_STANDBY)
    {
      sx126x_set_standby(dev, SX126X_STDBY_RC);
      dev->state = SX126X_STATE_STANDBY;
    }
}

/* Full chip setup.  Leaves the chip in STDBY_RC */

static void sx126x_hw_init(FAR struct sx126x_dev_s *dev, bool calibrate)
{
  uint16_t err;
  uint8_t reg;

  sx126x_set_standby(dev, SX126X_STDBY_RC);
  dev->state = SX126X_STATE_STANDBY;

  sx126x_command1(dev, SX126X_SETREGULATORMODE, dev->regulator);

  if (dev->tcxo_delay != 0)
    {
      sx126x_set_dio3_as_tcxo(dev);
    }

  if (calibrate)
    {
      /* With a TCXO the power-on calibration fails because the TCXO was
       * not powered yet.  Calibrate all blocks again.
       */

      sx126x_calibrate(dev, SX126X_CALIBRATE_ALL);
      dev->image_band = -1;
    }

  sx126x_command1(dev, SX126X_SETDIO2RFSWCTRL, dev->dio2_rfsw);
  sx126x_command1(dev, SX126X_SETRXTXFALLBACKMODE, SX126X_FALLBACK_STDBY_RC);
  sx126x_command1(dev, SX126X_SETPACKETTYPE, SX126X_PACKETTYPE_LORA);

  /* TX and RX buffer base addresses 0: both use the whole 256 bytes */

  sx126x_command(dev, SX126X_SETBUFFERBASEADDRESS, NULL,
                 SX126X_SETBUFFERBASEADDRESS_PARAMS, NULL);

  sx126x_set_dio_irq_params(dev);

  /* Errata 15.2: better resistance of the SX1262 TX to antenna mismatch */

  if (dev->model == SX1262)
    {
      reg = sx126x_read_reg8(dev, SX126X_REG_TX_CLAMP);
      sx126x_write_reg8(dev, SX126X_REG_TX_CLAMP, reg | 0x1e);
    }

  sx126x_clear_irq_status(dev, 0xffff);

  err = sx126x_get_device_errors(dev);
  if ((err & ~SX126X_DEVERR_XOSC_START) != 0)
    {
      wlerr("SX126x device errors 0x%04x\n", err);
    }

  sx126x_clear_device_errors(dev);
}

/* Apply the radio configuration.  Chip in standby. */

static void sx126x_apply_config(FAR struct sx126x_dev_s *dev, bool tx)
{
  uint8_t reg;

  sx126x_calibrate_image(dev, dev->frequency_hz);
  sx126x_set_rf_frequency(dev);
  sx126x_set_modulation_params(dev);
  sx126x_set_syncword(dev);

  if (tx)
    {
      sx126x_set_pa_config(dev);
      sx126x_set_tx_params(dev);

      /* Errata 15.1: modulation quality with 500 kHz LoRa bandwidth */

      reg = sx126x_read_reg8(dev, SX126X_REG_TX_MODULATION);
      if (dev->lora_bw == SX126X_LORA_BW_500)
        {
          reg &= ~0x04;
        }
      else
        {
          reg |= 0x04;
        }

      sx126x_write_reg8(dev, SX126X_REG_TX_MODULATION, reg);
    }
  else
    {
      sx126x_write_reg8(dev, SX126X_REG_RX_GAIN,
                        dev->rxboost ? SX126X_RX_GAIN_BOOSTED :
                                       SX126X_RX_GAIN_POWER_SAVING);
    }
}

/* Start RX.  timeout_ms == 0: continuous, else single RX with timeout */

static void sx126x_start_rx(FAR struct sx126x_dev_s *dev,
                            uint32_t timeout_ms)
{
  sx126x_standby(dev);
  sx126x_apply_config(dev, false);
  sx126x_set_packet_params(dev, sx126x_rx_len(dev));
  sx126x_command1(dev, SX126X_SETLORASYMBNUMTIMEOUT, 0);
  sx126x_command1(dev, SX126X_STOPTIMERONPREAMBLE, 0);
  sx126x_clear_irq_status(dev, 0xffff);

  dev->rx_timedout = false;

  if (timeout_ms == 0)
    {
      sx126x_set_rx(dev, SX126X_RX_CONTINUOUS_STEPS);
      dev->rx_hw_timeout = false;
      dev->state = SX126X_STATE_RX_CONT;
    }
  else
    {
      sx126x_set_rx(dev, sx126x_ms_to_steps(timeout_ms));
      dev->rx_hw_timeout = true;
      dev->state = SX126X_STATE_RX_SINGLE;
    }

  wlinfo("RX %s %" PRIu32 " ms\n", timeout_ms ? "single" : "continuous",
         timeout_ms);
}

static void sx126x_start_rx_duty(FAR struct sx126x_dev_s *dev,
                                 uint32_t rx_us, uint32_t sleep_us)
{
  sx126x_standby(dev);
  sx126x_apply_config(dev, false);
  sx126x_set_packet_params(dev, sx126x_rx_len(dev));
  sx126x_command1(dev, SX126X_SETLORASYMBNUMTIMEOUT, 0);
  sx126x_clear_irq_status(dev, 0xffff);

  dev->rx_timedout   = false;
  dev->rx_hw_timeout = false;
  dev->duty_rx_us    = rx_us;
  dev->duty_sleep_us = sleep_us;

  sx126x_set_rx_duty_cycle(dev, sx126x_us_to_steps(rx_us),
                           sx126x_us_to_steps(sleep_us));
  dev->state = SX126X_STATE_RX_DUTY;
}

static void sx126x_start_tx(FAR struct sx126x_dev_s *dev,
                            FAR const uint8_t *data, size_t len)
{
  sx126x_standby(dev);
  sx126x_apply_config(dev, true);
  sx126x_set_packet_params(dev, (uint8_t)len);
  sx126x_write_buffer(dev, 0, data, len);
  sx126x_clear_irq_status(dev, 0xffff);

  dev->tx_done = false;
  sx126x_set_tx(dev, SX126X_NO_TIMEOUT);
  dev->state = SX126X_STATE_TX;
}

static void sx126x_set_sleep(FAR struct sx126x_dev_s *dev, bool warm)
{
  if (dev->state == SX126X_STATE_SLEEP)
    {
      if (warm || dev->sleep_cold)
        {
          return;
        }
    }

  sx126x_standby(dev);

  sx126x_command1(dev, SX126X_SETSLEEP,
                  warm ? SX126X_SETSLEEP_CONF_START_WARM :
                         SX126X_SETSLEEP_CONF_START_COLD);
  up_udelay(SX126X_SLEEP_SETTLE_US);

  dev->sleep_cold = !warm;
  dev->state = SX126X_STATE_SLEEP;
}

static bool sx126x_pollin_waiting(FAR struct sx126x_dev_s *dev)
{
  int i;

  for (i = 0; i < CONFIG_LPWAN_SX126X_NPOLLWAITERS; i++)
    {
      if (dev->fds[i] != NULL && (dev->fds[i]->events & POLLIN) != 0)
        {
          return true;
        }
    }

  return false;
}

/* Put the radio in its idle state after TX, single RX or CAD.  With
 * 'waiters' set, blocked readers and POLLIN pollers keep it in RX.
 */

static void sx126x_return_to_idle(FAR struct sx126x_dev_s *dev,
                                  bool waiters)
{
  if (dev->idle == SX126X_IDLE_RX ||
      (waiters && (dev->rx_waiters > 0 || sx126x_pollin_waiting(dev))))
    {
      sx126x_start_rx(dev, 0);
    }
  else if (dev->idle == SX126X_IDLE_SLEEP)
    {
      sx126x_set_sleep(dev, true);
    }
  else
    {
      sx126x_standby(dev);
    }
}

/* Restart continuous RX after a configuration change */

static void sx126x_config_changed(FAR struct sx126x_dev_s *dev)
{
  if (dev->state == SX126X_STATE_RX_CONT)
    {
      sx126x_start_rx(dev, 0);
    }
}

/****************************************************************************
 * RX FIFO (dev->lock held)
 ****************************************************************************/

static void sx126x_sem_reset(FAR sem_t *sem)
{
  while (nxsem_trywait(sem) == OK)
    {
      /* Discard stale posts */
    }
}

static void sx126x_wake_readers(FAR struct sx126x_dev_s *dev)
{
  int sval = 0;

  /* Wake every blocked reader and leave the count at most at 1 so that a
   * reader that is about to block returns at once and re-checks.
   */

  nxsem_get_value(&dev->rx_sem, &sval);
  while (sval < 1)
    {
      nxsem_post(&dev->rx_sem);
      sval++;
    }
}

static FAR struct sx126x_read_hdr_s *
sx126x_fifo_alloc(FAR struct sx126x_dev_s *dev)
{
  uint8_t idx;

  if (dev->fifo_count >= CONFIG_LPWAN_SX126X_RXFIFO_LEN)
    {
      /* Drop the oldest packet */

      dev->fifo_head = (dev->fifo_head + 1) %
                       CONFIG_LPWAN_SX126X_RXFIFO_LEN;
      dev->fifo_count--;
      dev->rx_overruns++;
      wlwarn("SX126x RX FIFO overrun\n");
    }

  idx = (dev->fifo_head + dev->fifo_count) % CONFIG_LPWAN_SX126X_RXFIFO_LEN;
  return &dev->fifo[idx];
}

static ssize_t sx126x_fifo_get(FAR struct sx126x_dev_s *dev,
                               FAR char *buffer, size_t buflen)
{
  FAR struct sx126x_read_hdr_s *hdr = &dev->fifo[dev->fifo_head];
  size_t len = SX126X_READ_DATA_HEADER_LEN + hdr->datalen;

  if (len > buflen)
    {
      len = buflen;
    }

  memcpy(buffer, hdr, len);

  dev->fifo_head = (dev->fifo_head + 1) % CONFIG_LPWAN_SX126X_RXFIFO_LEN;
  dev->fifo_count--;

  return (ssize_t)len;
}

/* Read the received packet from the chip buffer into the RX FIFO */

static bool sx126x_fetch_packet(FAR struct sx126x_dev_s *dev,
                                uint16_t irq, uint8_t flags)
{
  FAR struct sx126x_read_hdr_s *hdr;
  bool crcerr = (irq & SX126X_IRQ_CRCERR_MASK) != 0;
  uint8_t offset;
  uint8_t len;

  if (crcerr)
    {
      dev->rx_crcerr++;
#ifndef CONFIG_LPWAN_SX126X_RX_CRCERR
      wlinfo("SX126x dropped packet with CRC error\n");
      return false;
#endif
    }

  sx126x_get_rx_buffer_status(dev, &len, &offset);

  hdr = sx126x_fifo_alloc(dev);
  sx126x_get_packet_status(dev, hdr);
  sx126x_read_buffer(dev, offset, hdr->data, len);

  hdr->datalen  = len;
  hdr->flags    = flags;
  hdr->crc_err  = crcerr ? 1 : 0;
  hdr->reserved = 0;

  dev->fifo_count++;
  dev->rx_packets++;

  wlinfo("SX126x RX %u bytes rssi %d snr %d\n", len, hdr->rssi, hdr->snr);

  sx126x_wake_readers(dev);
  poll_notify(dev->fds, CONFIG_LPWAN_SX126X_NPOLLWAITERS, POLLIN);
  return true;
}

/****************************************************************************
 * IRQ processing (dev->lock held)
 ****************************************************************************/

static void sx126x_process_irq(FAR struct sx126x_dev_s *dev, uint8_t flags)
{
  uint8_t prev = dev->state;
  bool rx_ended = false;
  uint16_t irq;
  int round;

  if (prev == SX126X_STATE_SLEEP)
    {
      /* The chip cannot raise interrupts while sleeping */

      return;
    }

  /* In RX duty cycle mode the chip may be in its sleep phase: the wake-up
   * then stops the duty cycle.  After RxDone it is in STDBY_RC already.
   * Keep the previous state for the decisions below.
   */

  sx126x_ensure_awake(dev);
  dev->state = prev;

  for (round = 0; round < SX126X_IRQ_MAX_ROUNDS; round++)
    {
      irq = sx126x_get_irq_status(dev);
      if (irq == 0)
        {
          break;
        }

      sx126x_clear_irq_status(dev, irq);
      dev->irq_latched |= irq;

      wlinfo("SX126x IRQ 0x%04x state %u\n", irq, dev->state);

      if ((irq & SX126X_IRQ_TXDONE_MASK) != 0)
        {
          if (dev->state == SX126X_STATE_TX)
            {
              dev->state = SX126X_STATE_STANDBY;
            }

          dev->tx_done = true;
          dev->tx_packets++;
          nxsem_post(&dev->tx_sem);
        }

      if ((irq & SX126X_IRQ_CADDONE_MASK) != 0)
        {
          dev->cad_detected = (irq & SX126X_IRQ_CADDETECTED_MASK) != 0;
          dev->cad_done = true;

          if (dev->state == SX126X_STATE_CAD)
            {
              dev->state = dev->cad_detected &&
                           dev->cad_exit == SX126X_CAD_RX ?
                           SX126X_STATE_RX_SINGLE : SX126X_STATE_STANDBY;
            }

          nxsem_post(&dev->cad_sem);
        }

      if ((irq & SX126X_IRQ_RXDONE_MASK) != 0)
        {
          if (dev->state == SX126X_STATE_RX_SINGLE ||
              dev->state == SX126X_STATE_RX_DUTY)
            {
              /* Single RX (also after CAD_RX) and duty cycle RX end in
               * STDBY_RC after a packet.
               */

              rx_ended = true;
              dev->state = SX126X_STATE_STANDBY;
            }

          if (dev->rx_hw_timeout && dev->lora_fixed_header)
            {
              uint8_t reg;

              /* Errata 15.3: stop the RTC after RX with timeout in
               * implicit header mode, clear the pending timeout event.
               */

              sx126x_write_reg8(dev, SX126X_REG_RTC_CTRL, 0x00);
              reg = sx126x_read_reg8(dev, SX126X_REG_EVT_MASK);
              sx126x_write_reg8(dev, SX126X_REG_EVT_MASK, reg | 0x02);
            }

          dev->rx_hw_timeout = false;
          sx126x_fetch_packet(dev, irq, flags);
        }
      else if ((irq & (SX126X_IRQ_TIMEOUT_MASK |
                       SX126X_IRQ_HEADERERR_MASK)) != 0)
        {
          if (dev->state == SX126X_STATE_TX)
            {
              /* No TX timeout is programmed, should not happen */

              dev->state = SX126X_STATE_STANDBY;
              nxsem_post(&dev->tx_sem);
            }
          else if (dev->state == SX126X_STATE_RX_SINGLE)
            {
              if ((irq & SX126X_IRQ_TIMEOUT_MASK) == 0)
                {
                  /* Header error: the chip may stay in RX, stop it and
                   * report it like a timeout.
                   */

                  sx126x_set_standby(dev, SX126X_STDBY_RC);
                }

              dev->state = SX126X_STATE_STANDBY;
              dev->rx_hw_timeout = false;
              dev->rx_timedout = true;
              dev->rx_timeouts++;
              rx_ended = true;
              sx126x_wake_readers(dev);
            }
        }
    }

  if (dev->state == SX126X_STATE_RX_DUTY)
    {
      /* Woken without a packet (header error or a spurious edge): the
       * wake-up above may have stopped the duty cycle, restart it.
       */

      sx126x_start_rx_duty(dev, dev->duty_rx_us, dev->duty_sleep_us);
      return;
    }

  if (dev->state == SX126X_STATE_UNKNOWN)
    {
      /* Attach without reset: the chip was woken up and is in STDBY_RC */

      dev->state = SX126X_STATE_STANDBY;
      return;
    }

  if (rx_ended && dev->state == SX126X_STATE_STANDBY)
    {
      sx126x_return_to_idle(dev, !dev->rx_timedout);
    }
}

static void sx126x_irq_worker(FAR void *arg)
{
  FAR struct sx126x_dev_s *dev = (FAR struct sx126x_dev_s *)arg;

  DEBUGASSERT(dev != NULL);

  nxmutex_lock(&dev->lock);
  sx126x_process_irq(dev, 0);
  nxmutex_unlock(&dev->lock);
}

static int sx126x_irq0handler(int irq, FAR void *context, FAR void *arg)
{
  FAR struct sx126x_dev_s *dev = (FAR struct sx126x_dev_s *)arg;

  DEBUGASSERT(dev != NULL);

  /* SPI is needed, defer to the work queue */

  if (work_available(&dev->irq_work))
    {
      work_queue(HPWORK, &dev->irq_work, sx126x_irq_worker, dev, 0);
    }

  return OK;
}

/****************************************************************************
 * CAD (dev->oplock and dev->lock held)
 ****************************************************************************/

static int sx126x_do_cad(FAR struct sx126x_dev_s *dev,
                         FAR struct sx126x_cad_ioc_s *cad)
{
  uint8_t params[SX126X_SETCADPARAMS_PARAMS];
  uint32_t symbols = cad->symbols;
  uint32_t wait_us;
  uint8_t symcode;
  int ret;

  if (symbols == 0)
    {
      /* Heuristic based on Semtech AN1200.48: more symbols for higher SF
       * and for the wide bandwidth.
       */

      symbols = dev->lora_sf < SX126X_LORA_SF9 ? 2 : 4;
      if (dev->lora_bw == SX126X_LORA_BW_500)
        {
          symbols *= 2;
        }
    }

  switch (symbols)
    {
      case 1:
        symcode = SX126X_CAD_ON_1_SYMB;
        break;

      case 2:
        symcode = SX126X_CAD_ON_2_SYMB;
        break;

      case 4:
        symcode = SX126X_CAD_ON_4_SYMB;
        break;

      case 8:
        symcode = SX126X_CAD_ON_8_SYMB;
        break;

      case 16:
        symcode = SX126X_CAD_ON_16_SYMB;
        break;

      default:
        return -EINVAL;
    }

  if (cad->exit_mode != SX126X_CAD_ONLY && cad->exit_mode != SX126X_CAD_RX)
    {
      return -EINVAL;
    }

  sx126x_standby(dev);
  sx126x_apply_config(dev, false);
  sx126x_set_packet_params(dev, sx126x_rx_len(dev));
  sx126x_clear_irq_status(dev, 0xffff);

  params[SX126X_SETCADPARAMS_CADSYMNUM_PARAM]   = symcode;
  params[SX126X_SETCADPARAMS_CADDETPEAK_PARAM]  =
    cad->det_peak != 0 ? cad->det_peak : dev->lora_sf + 13;
  params[SX126X_SETCADPARAMS_CADDETMIN_PARAM]   =
    cad->det_min != 0 ? cad->det_min : 10;
  params[SX126X_SETCADPARAMS_CADEXITMODE_PARAM] = cad->exit_mode;
  sx126x_put_be24(params + SX126X_SETCADPARAMS_CADTIMEOUT_PARAM,
                  cad->exit_mode == SX126X_CAD_RX ?
                  sx126x_ms_to_steps(cad->rx_timeout_ms) : 0);

  sx126x_command(dev, SX126X_SETCADPARAMS, params, sizeof(params), NULL);

  sx126x_sem_reset(&dev->cad_sem);

  dev->cad_exit     = cad->exit_mode;
  dev->cad_done     = false;
  dev->cad_detected = false;
  dev->rx_hw_timeout = cad->exit_mode == SX126X_CAD_RX &&
                       cad->rx_timeout_ms != 0;
  dev->rx_timedout  = false;

  sx126x_command(dev, SX126X_SETCAD, NULL, 0, NULL);
  dev->state = SX126X_STATE_CAD;

  wait_us = (symbols + 1) * sx126x_tsym_us(dev);

  nxmutex_unlock(&dev->lock);
  ret = nxsem_tickwait_uninterruptible(&dev->cad_sem,
                                       USEC2TICK(wait_us) +
                                       MSEC2TICK(SX126X_CAD_GUARD_MS));
  nxmutex_lock(&dev->lock);

  if (!dev->cad_done)
    {
      /* Maybe the interrupt was missed */

      sx126x_process_irq(dev, 0);
    }

  if (!dev->cad_done)
    {
      wlerr("SX126x CAD timeout (%d)\n", ret);
      sx126x_standby(dev);
      return -ETIMEDOUT;
    }

  cad->busy = dev->cad_detected;
  return OK;
}

/****************************************************************************
 * File operations
 ****************************************************************************/

static int sx126x_open(FAR struct file *filep)
{
  FAR struct sx126x_dev_s *dev = filep->f_inode->i_private;
  int ret;

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (dev->crefs == UINT8_MAX)
    {
      ret = -EMFILE;
      goto out;
    }

  if (dev->crefs == 0 && dev->need_init)
    {
      if (!dev->skip_reset)
        {
          if (dev->lower->reset != NULL)
            {
              dev->lower->reset();
            }

          sx126x_wait_busy(dev, SX126X_BUSY_LONG_US);
          dev->state = SX126X_STATE_STANDBY;
          dev->image_band = -1;
        }
      else
        {
          sx126x_ensure_awake(dev);
        }

      dev->skip_reset = false;
      sx126x_hw_init(dev, true);
      dev->need_init = false;

      wlinfo("SX126x %u initialized\n", dev->lower->dev_number);
    }

  dev->crefs++;
  ret = OK;

out:
  nxmutex_unlock(&dev->lock);
  return ret;
}

static int sx126x_close(FAR struct file *filep)
{
  FAR struct sx126x_dev_s *dev = filep->f_inode->i_private;
  int ret;

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (dev->crefs > 0)
    {
      dev->crefs--;
    }

  if (dev->crefs == 0)
    {
      /* Leave sleep and RX duty cycle alone, they are deliberate low
       * power states.  Anything else goes to standby.
       */

      if (dev->state != SX126X_STATE_SLEEP &&
          dev->state != SX126X_STATE_RX_DUTY)
        {
          sx126x_standby(dev);
        }

      dev->fifo_head  = 0;
      dev->fifo_count = 0;
    }

  nxmutex_unlock(&dev->lock);
  return OK;
}

static ssize_t sx126x_read(FAR struct file *filep, FAR char *buffer,
                           size_t buflen)
{
  FAR struct sx126x_dev_s *dev = filep->f_inode->i_private;
  clock_t start = clock_systime_ticks();
  clock_t timeout_ticks = 0;
  clock_t elapsed;
  clock_t wait;
  uint32_t timeout_ms;
  ssize_t ret;

  if (buffer == NULL || buflen < SX126X_READ_DATA_HEADER_LEN)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Forget a timeout of a single RX that nobody waited for */

  if (dev->state != SX126X_STATE_RX_SINGLE)
    {
      dev->rx_timedout = false;
    }

  timeout_ms = dev->rx_timeout_ms;
  if (timeout_ms != 0)
    {
      timeout_ticks = MSEC2TICK(timeout_ms);
    }

  while (dev->fifo_count == 0)
    {
      if ((filep->f_oflags & O_NONBLOCK) != 0)
        {
          ret = -EAGAIN;
          goto out;
        }

      elapsed = clock_systime_ticks() - start;
      if (timeout_ms != 0 && elapsed >= timeout_ticks)
        {
          ret = -ETIMEDOUT;
          goto out;
        }

      /* Arm RX unless the radio is receiving or busy with TX / CAD (it
       * returns to RX for waiting readers afterwards).
       */

      if (!sx126x_receiving(dev) && dev->state != SX126X_STATE_TX &&
          dev->state != SX126X_STATE_CAD)
        {
          uint32_t remain_ms = 0;

          if (timeout_ms != 0)
            {
              remain_ms = TICK2MSEC(timeout_ticks - elapsed);
              if (remain_ms == 0)
                {
                  remain_ms = 1;
                }
            }

          sx126x_start_rx(dev, remain_ms);
        }

      dev->rx_waiters++;

      if (timeout_ms != 0)
        {
          wait = timeout_ticks - elapsed;
          if (dev->state == SX126X_STATE_RX_SINGLE && dev->rx_hw_timeout)
            {
              /* The hardware timer stops at header detection, the packet
               * can take up to a maximum packet time on air after that.
               */

              wait += USEC2TICK(sx126x_airtime_us(dev, SX126X_MAX_PAYLOAD))
                      + MSEC2TICK(SX126X_RX_GUARD_MS);
            }

          nxmutex_unlock(&dev->lock);
          ret = nxsem_tickwait(&dev->rx_sem, wait);
        }
      else
        {
          nxmutex_unlock(&dev->lock);
          ret = nxsem_wait(&dev->rx_sem);
        }

      nxmutex_lock(&dev->lock);
      dev->rx_waiters--;

      if (dev->fifo_count > 0)
        {
          break;
        }

      if (dev->rx_timedout)
        {
          dev->rx_timedout = false;
          if (timeout_ms != 0)
            {
              ret = -ETIMEDOUT;
              goto out;
            }

          /* No read timeout: a single RX armed by someone else timed out,
           * loop and re-arm continuous RX.
           */

          continue;
        }

      if (ret == -ETIMEDOUT)
        {
          if (dev->state == SX126X_STATE_RX_SINGLE)
            {
              /* The hardware timeout never came.  Check for a missed
               * interrupt before giving up.
               */

              sx126x_process_irq(dev, 0);
              if (dev->fifo_count > 0)
                {
                  break;
                }

              if (dev->state == SX126X_STATE_RX_SINGLE)
                {
                  sx126x_standby(dev);
                  sx126x_return_to_idle(dev, false);
                }
            }

          dev->rx_timeouts++;
          goto out;
        }
      else if (ret < 0)
        {
          goto out;
        }
    }

  ret = sx126x_fifo_get(dev, buffer, buflen);

out:
  nxmutex_unlock(&dev->lock);
  return ret;
}

static ssize_t sx126x_write(FAR struct file *filep, FAR const char *buffer,
                            size_t buflen)
{
  FAR struct sx126x_dev_s *dev = filep->f_inode->i_private;
  uint32_t airtime;
  ssize_t ret;
  int wret;

  if (buffer == NULL || buflen == 0 || buflen > SX126X_MAX_PAYLOAD)
    {
      return -EINVAL;
    }

  ret = nxmutex_lock(&dev->oplock);
  if (ret < 0)
    {
      return ret;
    }

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      nxmutex_unlock(&dev->oplock);
      return ret;
    }

  dev->tx_busy = true;

  if (dev->lbt)
    {
      struct sx126x_cad_ioc_s cad;

      memset(&cad, 0, sizeof(cad));
      cad.exit_mode = SX126X_CAD_ONLY;

      ret = sx126x_do_cad(dev, &cad);
      if (ret == OK && cad.busy)
        {
          wlinfo("SX126x LBT: channel busy\n");
          ret = -EBUSY;
        }

      if (ret < 0)
        {
          sx126x_return_to_idle(dev, true);
          goto out;
        }
    }

  airtime = sx126x_airtime_us(dev, buflen);

  sx126x_sem_reset(&dev->tx_sem);

  sx126x_start_tx(dev, (FAR const uint8_t *)buffer, buflen);

  wlinfo("SX126x TX %zu bytes, %" PRIu32 " us\n", buflen, airtime);

  nxmutex_unlock(&dev->lock);
  wret = nxsem_tickwait_uninterruptible(&dev->tx_sem,
                                        USEC2TICK(airtime) +
                                        SX126X_TX_MARGIN_TICKS);
  nxmutex_lock(&dev->lock);

  if (!dev->tx_done)
    {
      /* Maybe the interrupt was missed */

      sx126x_process_irq(dev, 0);
    }

  if (dev->tx_done)
    {
      ret = (ssize_t)buflen;
    }
  else
    {
      wlerr("SX126x TX timeout (%d)\n", wret);
      dev->tx_timeouts++;
      sx126x_standby(dev);
      ret = -ETIMEDOUT;
    }

  sx126x_return_to_idle(dev, true);

out:
  dev->tx_busy = false;
  poll_notify(dev->fds, CONFIG_LPWAN_SX126X_NPOLLWAITERS, POLLOUT);
  nxmutex_unlock(&dev->lock);
  nxmutex_unlock(&dev->oplock);
  return ret;
}

static int sx126x_poll(FAR struct file *filep, FAR struct pollfd *fds,
                       bool setup)
{
  FAR struct sx126x_dev_s *dev = filep->f_inode->i_private;
  pollevent_t eventset = 0;
  int ret;
  int i;

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  if (setup)
    {
      for (i = 0; i < CONFIG_LPWAN_SX126X_NPOLLWAITERS; i++)
        {
          if (dev->fds[i] == NULL)
            {
              dev->fds[i] = fds;
              fds->priv   = &dev->fds[i];
              break;
            }
        }

      if (i >= CONFIG_LPWAN_SX126X_NPOLLWAITERS)
        {
          fds->priv = NULL;
          ret = -EBUSY;
          goto out;
        }

      if (dev->fifo_count > 0)
        {
          eventset |= POLLIN;
        }
      else if ((fds->events & POLLIN) != 0 &&
               dev->state != SX126X_STATE_TX &&
               dev->state != SX126X_STATE_CAD && !sx126x_receiving(dev) &&
               dev->state != SX126X_STATE_SLEEP)
        {
          /* Somebody wants packets but the radio does not listen */

          sx126x_start_rx(dev, 0);
        }

      if (!dev->tx_busy)
        {
          eventset |= POLLOUT;
        }

      poll_notify(&fds, 1, eventset);
    }
  else if (fds->priv != NULL)
    {
      FAR struct pollfd **slot = (FAR struct pollfd **)fds->priv;

      *slot = NULL;
      fds->priv = NULL;
    }

out:
  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * ioctl
 ****************************************************************************/

static int sx126x_ioctl_lora(FAR struct sx126x_dev_s *dev, int cmd,
                             unsigned long arg)
{
  int ret = OK;

  switch (cmd)
    {
      case WLIOC_LORA_SETSF:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);

          if (ptr == NULL || *ptr < SX126X_LORA_SF5 ||
              *ptr > SX126X_LORA_SF12)
            {
              return -EINVAL;
            }

          dev->lora_sf = *ptr;
          sx126x_config_changed(dev);
          break;
        }

      case WLIOC_LORA_GETSF:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);

          if (ptr == NULL)
            {
              return -EINVAL;
            }

          *ptr = dev->lora_sf;
          break;
        }

      case WLIOC_LORA_SETBW:
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);

          if (ptr == NULL || *ptr == 0)
            {
              return -EINVAL;
            }

          dev->lora_bw = sx126x_bw_from_hz(*ptr);
          sx126x_config_changed(dev);
          break;
        }

      case WLIOC_LORA_GETBW:
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);

          if (ptr == NULL)
            {
              return -EINVAL;
            }

          *ptr = sx126x_bw_hz(dev->lora_bw);
          break;
        }

      case WLIOC_LORA_SETCR:
        {
          FAR enum wlioc_lora_cr_e *ptr =
            (FAR enum wlioc_lora_cr_e *)((uintptr_t)arg);

          if (ptr == NULL || *ptr < WLIOC_LORA_CR_4_5 ||
              *ptr > WLIOC_LORA_CR_4_8)
            {
              return -EINVAL;
            }

          dev->lora_cr = (uint8_t)*ptr;
          sx126x_config_changed(dev);
          break;
        }

      case WLIOC_LORA_GETCR:
        {
          FAR enum wlioc_lora_cr_e *ptr =
            (FAR enum wlioc_lora_cr_e *)((uintptr_t)arg);

          if (ptr == NULL)
            {
              return -EINVAL;
            }

          *ptr = (enum wlioc_lora_cr_e)dev->lora_cr;
          break;
        }

      case WLIOC_LORA_SETCRC:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);

          if (ptr == NULL)
            {
              return -EINVAL;
            }

          dev->lora_crc = *ptr != 0;
          sx126x_config_changed(dev);
          break;
        }

      case WLIOC_LORA_GETCRC:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);

          if (ptr == NULL)
            {
              return -EINVAL;
            }

          *ptr = dev->lora_crc;
          break;
        }

      case WLIOC_LORA_SETFIXEDHDR:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);

          if (ptr == NULL)
            {
              return -EINVAL;
            }

          dev->lora_fixed_header = *ptr != 0;
          sx126x_config_changed(dev);
          break;
        }

      case WLIOC_LORA_GETFIXEDHDR:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);

          if (ptr == NULL)
            {
              return -EINVAL;
            }

          *ptr = dev->lora_fixed_header;
          break;
        }

      case WLIOC_LORA_SETSYNCWORD:
        {
          FAR struct wlioc_lora_syncword_s *ptr =
            (FAR struct wlioc_lora_syncword_s *)((uintptr_t)arg);

          if (ptr == NULL || ptr->syncword == NULL)
            {
              return -EINVAL;
            }

          if (ptr->syncword_length == 1)
            {
              /* Classic one byte LoRa sync word, 0x12 -> 0x1424 */

              uint8_t sw = ptr->syncword[0];

              dev->syncword = (uint16_t)(((sw & 0xf0) | 0x04) << 8) |
                              (uint16_t)(((sw & 0x0f) << 4) | 0x04);
            }
          else if (ptr->syncword_length == 2)
            {
              dev->syncword = ((uint16_t)ptr->syncword[0] << 8) |
                              ptr->syncword[1];
            }
          else
            {
              return -EINVAL;
            }

          sx126x_config_changed(dev);
          break;
        }

      case WLIOC_LORA_GETSYNCWORD:
        {
          FAR struct wlioc_lora_syncword_s *ptr =
            (FAR struct wlioc_lora_syncword_s *)((uintptr_t)arg);

          if (ptr == NULL || ptr->syncword == NULL ||
              ptr->syncword_length == 0)
            {
              return -EINVAL;
            }

          if (ptr->syncword_length == 1)
            {
              ptr->syncword[0] = (uint8_t)((dev->syncword >> 8) & 0xf0) |
                                 (uint8_t)((dev->syncword >> 4) & 0x0f);
            }
          else
            {
              ptr->syncword[0]    = (uint8_t)(dev->syncword >> 8);
              ptr->syncword[1]    = (uint8_t)dev->syncword;
              ptr->syncword_length = 2;
            }

          break;
        }

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

static int sx126x_ioctl_sx126x(FAR struct sx126x_dev_s *dev, int cmd,
                               unsigned long arg)
{
  int ret = OK;

  switch (cmd)
    {
      case SX126XIOC_PACKETTYPESET:
        {
          FAR enum sx126x_packet_type_e *ptr =
            (FAR enum sx126x_packet_type_e *)((uintptr_t)arg);

          if (ptr == NULL)
            {
              return -EINVAL;
            }

          ret = *ptr == SX126X_PACKETTYPE_LORA ? OK : -ENOTSUP;
          break;
        }

      case SX126XIOC_LORACONFIGSET:
        {
          FAR struct sx126x_lora_config_s *ptr =
            (FAR struct sx126x_lora_config_s *)((uintptr_t)arg);

          if (ptr == NULL ||
              ptr->modulation.spreading_factor < SX126X_LORA_SF5 ||
              ptr->modulation.spreading_factor > SX126X_LORA_SF12 ||
              !sx126x_bw_valid(ptr->modulation.bandwidth) ||
              ptr->modulation.coding_rate < SX126X_LORA_CR_4_5 ||
              ptr->modulation.coding_rate > SX126X_LORA_CR_4_8 ||
              ptr->packet.preambles == 0)
            {
              return -EINVAL;
            }

          dev->lora_sf           = ptr->modulation.spreading_factor;
          dev->lora_bw           = ptr->modulation.bandwidth;
          dev->lora_cr           = ptr->modulation.coding_rate;
          dev->lora_ldro         = ptr->modulation.low_datarate_optimization;
          dev->lora_crc          = ptr->packet.crc_enable;
          dev->lora_fixed_header = ptr->packet.fixed_length_header;
          dev->fixed_len         = ptr->packet.payload_length;
          dev->invert_iq         = ptr->packet.invert_iq;
          dev->preambles         = ptr->packet.preambles;
          sx126x_config_changed(dev);
          break;
        }

      case SX126XIOC_LORACONFIGGET:
        {
          FAR struct sx126x_lora_config_s *ptr =
            (FAR struct sx126x_lora_config_s *)((uintptr_t)arg);

          if (ptr == NULL)
            {
              return -EINVAL;
            }

          ptr->modulation.spreading_factor = dev->lora_sf;
          ptr->modulation.bandwidth        = dev->lora_bw;
          ptr->modulation.coding_rate      = dev->lora_cr;
          ptr->modulation.low_datarate_optimization = sx126x_ldro(dev);
          ptr->packet.crc_enable           = dev->lora_crc;
          ptr->packet.fixed_length_header  = dev->lora_fixed_header;
          ptr->packet.payload_length       = dev->fixed_len;
          ptr->packet.invert_iq            = dev->invert_iq;
          ptr->packet.preambles            = dev->preambles;
          break;
        }

      case SX126XIOC_STANDBY:
        {
          FAR enum sx126x_standby_mode_e *ptr =
            (FAR enum sx126x_standby_mode_e *)((uintptr_t)arg);
          enum sx126x_standby_mode_e mode = SX126X_STDBY_RC;

          if (ptr != NULL)
            {
              if (*ptr != SX126X_STDBY_RC && *ptr != SX126X_STDBY_XOSC)
                {
                  return -EINVAL;
                }

              mode = *ptr;
            }

          sx126x_ensure_awake(dev);
          sx126x_set_standby(dev, mode);
          dev->state = SX126X_STATE_STANDBY;
          break;
        }

      case SX126XIOC_SLEEP:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);

          sx126x_set_sleep(dev, ptr == NULL || *ptr != 0);
          break;
        }

      case SX126XIOC_RX:
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);

          sx126x_start_rx(dev, ptr != NULL ? *ptr : 0);
          break;
        }

      case SX126XIOC_RXTIMEOUTSET:
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);

          if (ptr == NULL)
            {
              return -EINVAL;
            }

          dev->rx_timeout_ms = *ptr > SX126X_TIMEOUT_MAX_MS ?
                               SX126X_TIMEOUT_MAX_MS : *ptr;
          break;
        }

      case SX126XIOC_RXTIMEOUTGET:
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);

          if (ptr == NULL)
            {
              return -EINVAL;
            }

          *ptr = dev->rx_timeout_ms;
          break;
        }

      case SX126XIOC_PREAMBLESET:
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);

          if (ptr == NULL || *ptr == 0 || *ptr > UINT16_MAX)
            {
              return -EINVAL;
            }

          dev->preambles = (uint16_t)*ptr;
          sx126x_config_changed(dev);
          break;
        }

      case SX126XIOC_PREAMBLEGET:
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);

          if (ptr == NULL)
            {
              return -EINVAL;
            }

          *ptr = dev->preambles;
          break;
        }

      case SX126XIOC_RXDUTYCYCLE:
        {
          FAR struct sx126x_rxduty_ioc_s *ptr =
            (FAR struct sx126x_rxduty_ioc_s *)((uintptr_t)arg);

          if (ptr == NULL || ptr->rx_us == 0 || ptr->sleep_us == 0)
            {
              return -EINVAL;
            }

          sx126x_start_rx_duty(dev, ptr->rx_us, ptr->sleep_us);
          break;
        }

      case SX126XIOC_TCXOSET:
        {
          FAR struct sx126x_tcxo_ioc_s *ptr =
            (FAR struct sx126x_tcxo_ioc_s *)((uintptr_t)arg);

          if (ptr == NULL || ptr->delay_us == 0 ||
              ptr->voltage > SX126X_TCXO_3_3V)
            {
              return -EINVAL;
            }

          dev->tcxo_voltage = ptr->voltage;
          dev->tcxo_delay   = sx126x_us_to_steps(ptr->delay_us);

          sx126x_standby(dev);
          sx126x_hw_init(dev, true);
          break;
        }

      case SX126XIOC_DIO2RFSWSET:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);
          bool was_rx;

          if (ptr == NULL)
            {
              return -EINVAL;
            }

          was_rx = dev->state == SX126X_STATE_RX_CONT;
          dev->dio2_rfsw = *ptr != 0;
          sx126x_standby(dev);
          sx126x_command1(dev, SX126X_SETDIO2RFSWCTRL, dev->dio2_rfsw);
          sx126x_set_dio_irq_params(dev);
          if (was_rx)
            {
              sx126x_start_rx(dev, 0);
            }

          break;
        }

      case SX126XIOC_REGULATORSET:
        {
          FAR enum sx126x_regulator_mode_e *ptr =
            (FAR enum sx126x_regulator_mode_e *)((uintptr_t)arg);
          bool was_rx;

          if (ptr == NULL ||
              (*ptr != SX126X_LDO && *ptr != SX126X_DC_DC_LDO))
            {
              return -EINVAL;
            }

          was_rx = dev->state == SX126X_STATE_RX_CONT;
          dev->regulator = *ptr;
          sx126x_standby(dev);
          sx126x_command1(dev, SX126X_SETREGULATORMODE, dev->regulator);
          if (was_rx)
            {
              sx126x_start_rx(dev, 0);
            }

          break;
        }

      case SX126XIOC_RSSIGET:
        {
          FAR int16_t *ptr = (FAR int16_t *)((uintptr_t)arg);

          if (ptr == NULL)
            {
              return -EINVAL;
            }

          if (dev->state != SX126X_STATE_RX_CONT &&
              dev->state != SX126X_STATE_RX_SINGLE)
            {
              return -EAGAIN;
            }

          *ptr = sx126x_get_rssi_inst(dev);
          break;
        }

      case SX126XIOC_IRQSTATUSGET:
        {
          FAR struct sx126x_irqstat_ioc_s *ptr =
            (FAR struct sx126x_irqstat_ioc_s *)((uintptr_t)arg);

          if (ptr == NULL)
            {
              return -EINVAL;
            }

          memset(ptr, 0, sizeof(*ptr));
          ptr->latched  = dev->irq_latched;
          ptr->state    = dev->state;
          ptr->chipmode = 0xff;
          dev->irq_latched = 0;

          if (dev->state != SX126X_STATE_SLEEP &&
              dev->state != SX126X_STATE_RX_DUTY &&
              dev->state != SX126X_STATE_UNKNOWN)
            {
              ptr->chipmode = sx126x_get_chipmode(dev);
              ptr->pending  = sx126x_get_irq_status(dev);
              ptr->deverr   = sx126x_get_device_errors(dev);
            }

          ptr->rx_packets  = dev->rx_packets;
          ptr->rx_crcerr   = dev->rx_crcerr;
          ptr->rx_overruns = dev->rx_overruns;
          ptr->rx_timeouts = dev->rx_timeouts;
          ptr->tx_packets  = dev->tx_packets;
          ptr->tx_timeouts = dev->tx_timeouts;
          break;
        }

      case SX126XIOC_WAKEFETCH:
        {
          uint32_t before = dev->rx_packets;

          sx126x_process_irq(dev, SX126X_RXHDR_F_WAKE);
          ret = dev->rx_packets != before ? 1 : 0;
          break;
        }

      case SX126XIOC_GETCAPS:
        {
          FAR struct sx126x_caps_s *ptr =
            (FAR struct sx126x_caps_s *)((uintptr_t)arg);

          if (ptr == NULL)
            {
              return -EINVAL;
            }

          memset(ptr, 0, sizeof(*ptr));
          ptr->flags = SX126X_CAPS_CAD | SX126X_CAPS_RXDUTY |
                       SX126X_CAPS_SLEEP | SX126X_CAPS_LBT |
                       SX126X_CAPS_RXBOOST;
          if (dev->lower->dio1_wake)
            {
              ptr->flags |= SX126X_CAPS_WAKEIRQ;
            }

          if (dev->tcxo_delay != 0)
            {
              ptr->flags |= SX126X_CAPS_TCXO;
            }

          if (dev->dio2_rfsw)
            {
              ptr->flags |= SX126X_CAPS_DIO2RFSW;
            }

          if (dev->lower->busy != NULL)
            {
              ptr->flags |= SX126X_CAPS_BUSYPIN;
            }

          ptr->max_payload = SX126X_MAX_PAYLOAD;
          sx126x_power_limits(dev, &ptr->txpower_min, &ptr->txpower_max);
          ptr->device     = (uint8_t)dev->model;
          ptr->rxfifo_len = CONFIG_LPWAN_SX126X_RXFIFO_LEN;
          break;
        }

      case SX126XIOC_IDLESET:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);

          if (ptr == NULL || *ptr > SX126X_IDLE_SLEEP)
            {
              return -EINVAL;
            }

          dev->idle = *ptr;
          if (dev->state == SX126X_STATE_STANDBY)
            {
              sx126x_return_to_idle(dev, false);
            }

          break;
        }

      case SX126XIOC_IDLEGET:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);

          if (ptr == NULL)
            {
              return -EINVAL;
            }

          *ptr = dev->idle;
          break;
        }

      case SX126XIOC_LBTSET:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);

          if (ptr == NULL)
            {
              return -EINVAL;
            }

          dev->lbt = *ptr != 0;
          break;
        }

      case SX126XIOC_AIRTIMEGET:
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);

          if (ptr == NULL || *ptr > SX126X_MAX_PAYLOAD)
            {
              return -EINVAL;
            }

          *ptr = sx126x_airtime_us(dev, *ptr);
          break;
        }

      case SX126XIOC_IQINVSET:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);

          if (ptr == NULL)
            {
              return -EINVAL;
            }

          dev->invert_iq = *ptr != 0;
          sx126x_config_changed(dev);
          break;
        }

      case SX126XIOC_RXBOOSTSET:
        {
          FAR uint8_t *ptr = (FAR uint8_t *)((uintptr_t)arg);

          if (ptr == NULL)
            {
              return -EINVAL;
            }

          dev->rxboost = *ptr != 0;
          sx126x_config_changed(dev);
          break;
        }

      default:
        ret = -ENOTTY;
        break;
    }

  return ret;
}

static int sx126x_ioctl(FAR struct file *filep, int cmd, unsigned long arg)
{
  FAR struct sx126x_dev_s *dev = filep->f_inode->i_private;
  int ret;

  wlinfo("SX126x %u ioctl 0x%x\n", dev->lower->dev_number, cmd);

  /* CAD waits for the chip, it needs the operation lock first */

  if (cmd == SX126XIOC_CAD)
    {
      FAR struct sx126x_cad_ioc_s *ptr =
        (FAR struct sx126x_cad_ioc_s *)((uintptr_t)arg);

      if (ptr == NULL)
        {
          return -EINVAL;
        }

      ret = nxmutex_lock(&dev->oplock);
      if (ret < 0)
        {
          return ret;
        }

      ret = nxmutex_lock(&dev->lock);
      if (ret < 0)
        {
          nxmutex_unlock(&dev->oplock);
          return ret;
        }

      ret = sx126x_do_cad(dev, ptr);
      if (ret < 0 || !ptr->busy || ptr->exit_mode != SX126X_CAD_RX)
        {
          if (dev->state == SX126X_STATE_STANDBY)
            {
              sx126x_return_to_idle(dev, true);
            }
        }

      nxmutex_unlock(&dev->lock);
      nxmutex_unlock(&dev->oplock);
      return ret;
    }

  ret = nxmutex_lock(&dev->lock);
  if (ret < 0)
    {
      return ret;
    }

  /* Commands touching the chip must not interfere with a running TX */

  if (dev->tx_busy && cmd != WLIOC_GETRADIOFREQ &&
      cmd != WLIOC_GETTXPOWER && cmd != SX126XIOC_GETCAPS &&
      cmd != SX126XIOC_AIRTIMEGET && cmd != SX126XIOC_IRQSTATUSGET &&
      cmd != SX126XIOC_RXTIMEOUTSET && cmd != SX126XIOC_RXTIMEOUTGET &&
      cmd != SX126XIOC_LBTSET && cmd != SX126XIOC_IDLEGET)
    {
      ret = -EBUSY;
      goto out;
    }

  switch (cmd)
    {
      case WLIOC_SETRADIOFREQ:
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);

          if (ptr == NULL || *ptr < SX126X_FREQ_MIN ||
              *ptr > SX126X_FREQ_MAX)
            {
              ret = -EINVAL;
              break;
            }

          if (dev->lower->check_frequency != NULL &&
              dev->lower->check_frequency(*ptr) != 0)
            {
              wlerr("Board does not support %" PRIu32 " Hz\n", *ptr);
              ret = -EINVAL;
              break;
            }

          dev->frequency_hz = *ptr;
          sx126x_config_changed(dev);
          break;
        }

      case WLIOC_GETRADIOFREQ:
        {
          FAR uint32_t *ptr = (FAR uint32_t *)((uintptr_t)arg);

          if (ptr == NULL)
            {
              ret = -EINVAL;
              break;
            }

          *ptr = dev->frequency_hz;
          break;
        }

      case WLIOC_SETTXPOWER:
        {
          FAR int8_t *ptr = (FAR int8_t *)((uintptr_t)arg);

          if (ptr == NULL)
            {
              ret = -EINVAL;
              break;
            }

          dev->power = sx126x_clamp_power(dev, *ptr);
          break;
        }

      case WLIOC_GETTXPOWER:
        {
          FAR int8_t *ptr = (FAR int8_t *)((uintptr_t)arg);

          if (ptr == NULL)
            {
              ret = -EINVAL;
              break;
            }

          *ptr = dev->power;
          break;
        }

      default:
        ret = sx126x_ioctl_lora(dev, cmd, arg);
        if (ret == -ENOTTY)
          {
            ret = sx126x_ioctl_sx126x(dev, cmd, arg);
          }

        break;
    }

out:
  nxmutex_unlock(&dev->lock);
  return ret;
}

/****************************************************************************
 * Setup
 ****************************************************************************/

static void sx126x_set_defaults(FAR struct sx126x_dev_s *dev)
{
  FAR const struct sx126x_lower_s *lower = dev->lower;
  int cr = CONFIG_LPWAN_SX126X_CR_DEFAULT;

  /* Board settings */

  dev->model       = SX1262;
  dev->hpmax       = 0x07;
  dev->padutycycle = 0x04;

  if (lower->get_pa_values != NULL)
    {
      lower->get_pa_values(&dev->model, &dev->hpmax, &dev->padutycycle);
    }

  dev->regulator    = lower->regulator_mode;
  dev->tcxo_voltage = lower->dio3_voltage;
  dev->tcxo_delay   = lower->dio3_delay;
  dev->dio2_rfsw    = lower->use_dio2_as_rf_sw != 0;

  /* Radio settings */

  dev->idle          = SX126X_DEFAULT_IDLE;
  dev->lbt           = false;
  dev->rxboost       = SX126X_DEFAULT_RXBOOST;
  dev->invert_iq     = false;
  dev->frequency_hz  = CONFIG_LPWAN_SX126X_RFFREQ_DEFAULT;
  dev->preambles     = CONFIG_LPWAN_SX126X_PREAMBLE_DEFAULT;
  dev->fixed_len     = 0;
  dev->lora_sf       = CONFIG_LPWAN_SX126X_SF_DEFAULT;
  dev->lora_bw       = sx126x_bw_from_hz(CONFIG_LPWAN_SX126X_BW_DEFAULT *
                                         1000);
  dev->lora_cr       = cr > 4 ? cr - 4 : cr;
  dev->lora_crc      = SX126X_DEFAULT_CRC;
  dev->lora_fixed_header = false;
  dev->lora_ldro     = false;
  dev->syncword      = CONFIG_LPWAN_SX126X_SYNCWORD;
  dev->rx_timeout_ms = CONFIG_LPWAN_SX126X_RX_TIMEOUT_DEFAULT;
  dev->power         =
    sx126x_clamp_power(dev, CONFIG_LPWAN_SX126X_TXPOWER_DEFAULT);
  dev->image_band    = -1;
}

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: sx126x_register
 ****************************************************************************/

int sx126x_register(FAR struct spi_dev_s *spi,
                    FAR const struct sx126x_lower_s *lower,
                    FAR const char *path)
{
  FAR struct sx126x_dev_s *dev;
  int ret;

  if (spi == NULL || lower == NULL || path == NULL ||
      lower->irq0attach == NULL)
    {
      return -EINVAL;
    }

  if (lower->dev_number >= CONFIG_LPWAN_SX126X_MAX_DEVICES)
    {
      wlerr("SX126x dev_number %u exceeds LPWAN_SX126X_MAX_DEVICES\n",
            lower->dev_number);
      return -EINVAL;
    }

  dev = &g_sx126x_devices[lower->dev_number];
  memset(dev, 0, sizeof(*dev));

  dev->spi   = spi;
  dev->lower = lower;

  nxmutex_init(&dev->lock);
  nxmutex_init(&dev->oplock);
  nxsem_init(&dev->rx_sem, 0, 0);
  nxsem_init(&dev->tx_sem, 0, 0);
  nxsem_init(&dev->cad_sem, 0, 0);

  sx126x_set_defaults(dev);

  dev->state      = SX126X_STATE_UNKNOWN;
  dev->need_init  = true;
  dev->skip_reset = lower->no_reset;

  if (lower->no_reset)
    {
      /* Woken from deep sleep by DIO1: keep the chip as it is and move
       * the pending packet into the RX FIFO.
       */

      nxmutex_lock(&dev->lock);
      sx126x_process_irq(dev, SX126X_RXHDR_F_WAKE);
      nxmutex_unlock(&dev->lock);

      wlinfo("SX126x attached without reset, %u packet(s) fetched\n",
             dev->fifo_count);
    }

  ret = lower->irq0attach(sx126x_irq0handler, dev);
  if (ret < 0)
    {
      wlerr("SX126x IRQ attach failed: %d\n", ret);
      goto errout;
    }

  if (lower->irq0enable != NULL)
    {
      lower->irq0enable(true);
    }

  ret = register_driver(path, &g_sx126x_fops, 0666, dev);
  if (ret < 0)
    {
      wlerr("SX126x register_driver failed: %d\n", ret);
      if (lower->irq0enable != NULL)
        {
          lower->irq0enable(false);
        }

      lower->irq0attach(NULL, NULL);
      goto errout;
    }

  return OK;

errout:
  nxsem_destroy(&dev->cad_sem);
  nxsem_destroy(&dev->tx_sem);
  nxsem_destroy(&dev->rx_sem);
  nxmutex_destroy(&dev->oplock);
  nxmutex_destroy(&dev->lock);
  return ret;
}
