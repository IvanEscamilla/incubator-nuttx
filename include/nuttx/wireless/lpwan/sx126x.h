/****************************************************************************
 * include/nuttx/wireless/lpwan/sx126x.h
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

#ifndef __INCLUDE_NUTTX_WIRELESS_LPWAN_SX126X_H
#define __INCLUDE_NUTTX_WIRELESS_LPWAN_SX126X_H

/* Semtech SX1261/SX1262 LoRa character driver.
 *
 * Userspace API (only LoRa packets are supported):
 *
 *   open()   Initializes the radio on the first open (reset, TCXO,
 *            calibration, regulator, DIO2 RF switch).  Several openers
 *            share one RX FIFO.  The configuration survives close().
 *
 *   read()   Returns one packet as struct sx126x_read_hdr_s.  The return
 *            value is SX126X_READ_DATA_HEADER_LEN + datalen (truncated to
 *            buflen).  If the RX FIFO is empty and the radio is not
 *            receiving, read() arms RX: continuous when the read timeout
 *            is 0, otherwise single RX with the hardware RX timeout.
 *            O_NONBLOCK returns -EAGAIN when the FIFO is empty, and a
 *            timeout (SX126XIOC_RXTIMEOUTSET) returns -ETIMEDOUT.
 *
 *   write()  Transmits buflen (1..255) payload bytes, blocks until TxDone
 *            and returns buflen.  Afterwards the radio goes to the idle
 *            state (SX126XIOC_IDLESET), or to continuous RX when a reader
 *            or a POLLIN poller is waiting.  With LBT enabled
 *            (SX126XIOC_LBTSET) a CAD runs first and -EBUSY is returned
 *            when the channel is busy.  -ETIMEDOUT if TxDone never comes.
 *
 *   poll()   POLLIN when the RX FIFO is not empty, POLLOUT when no TX is
 *            in progress.  Polling for POLLIN while the radio is in
 *            standby arms continuous RX.
 *
 *   ioctl()  Pointer arguments unless noted "no arg":
 *     WLIOC_SETRADIOFREQ / GETRADIOFREQ    uint32_t *  Hz
 *     WLIOC_SETTXPOWER / GETTXPOWER        int8_t *    dBm (clamped)
 *     WLIOC_LORA_SETSF / GETSF             uint8_t *   5..12
 *     WLIOC_LORA_SETBW / GETBW             uint32_t *  Hz (nearest)
 *     WLIOC_LORA_SETCR / GETCR             enum wlioc_lora_cr_e *
 *     WLIOC_LORA_SETCRC / GETCRC           uint8_t *   0/1
 *     WLIOC_LORA_SETFIXEDHDR / GETFIXEDHDR uint8_t *   0/1 (implicit hdr)
 *     WLIOC_LORA_SETSYNCWORD / GETSYNCWORD struct wlioc_lora_syncword_s *
 *                                  1 byte: 0x12 style (0x12 -> 0x1424),
 *                                  2 bytes: raw registers 0x0740/0x0741
 *     SX126XIOC_PACKETTYPESET   enum sx126x_packet_type_e * (LoRa only)
 *     SX126XIOC_LORACONFIGSET   struct sx126x_lora_config_s *
 *     SX126XIOC_LORACONFIGGET   struct sx126x_lora_config_s *
 *     SX126XIOC_STANDBY         enum sx126x_standby_mode_e * (NULL = RC)
 *     SX126XIOC_SLEEP           uint8_t * warm start 0/1 (NULL = warm)
 *     SX126XIOC_RX              uint32_t * 0 = continuous, else single RX
 *                               with that timeout in ms (NULL = cont.)
 *     SX126XIOC_RXTIMEOUTSET/GET uint32_t * read() timeout ms, 0 = none
 *     SX126XIOC_PREAMBLESET/GET uint32_t * preamble symbols
 *     SX126XIOC_CAD             struct sx126x_cad_ioc_s *, blocks until
 *                               CadDone, returns busy/free
 *     SX126XIOC_RXDUTYCYCLE     struct sx126x_rxduty_ioc_s *
 *     SX126XIOC_TCXOSET         struct sx126x_tcxo_ioc_s *
 *     SX126XIOC_DIO2RFSWSET     uint8_t * 0/1
 *     SX126XIOC_REGULATORSET    enum sx126x_regulator_mode_e *
 *     SX126XIOC_RSSIGET         int16_t * instantaneous RSSI dBm (in RX)
 *     SX126XIOC_IRQSTATUSGET    struct sx126x_irqstat_ioc_s *
 *     SX126XIOC_WAKEFETCH       no arg.  Moves a packet waiting in the
 *                               chip buffer (RxDone pending) into the RX
 *                               FIFO.  Returns 1 if queued, 0 if none.
 *     SX126XIOC_GETCAPS         struct sx126x_caps_s *
 *     SX126XIOC_IDLESET/GET     uint8_t * SX126X_IDLE_*
 *     SX126XIOC_LBTSET          uint8_t * 0/1 CAD before every write()
 *     SX126XIOC_AIRTIMEGET      uint32_t * in: payload bytes, out: us
 *     SX126XIOC_IQINVSET        uint8_t * 0/1 inverted IQ
 *     SX126XIOC_RXBOOSTSET      uint8_t * 0/1 boosted RX gain
 *
 * Deep-sleep radio wake: when the board registers the driver with
 * lower->no_reset set, the chip is neither reset nor reconfigured at
 * registration.  A packet pending in the chip buffer (RxDone) is fetched
 * into the RX FIFO immediately (flag SX126X_RXHDR_F_WAKE), and the first
 * open() skips the hardware reset.
 */

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/spi/spi.h>
#include <nuttx/irq.h>
#include <nuttx/wireless/ioctl.h>

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Maximum LoRa payload of the SX126x */

#define SX126X_MAX_PAYLOAD                  255
#define SX126X_RX_PAYLOAD_SIZE              SX126X_MAX_PAYLOAD

/* IOCTL commands ***********************************************************/

/* arg: Pointer to enum sx126x_packet_type_e.  Only LoRa is supported. */

#define SX126XIOC_PACKETTYPESET             _WLCIOC(SX126X_FIRST + 0)

/* Sets / gets LoRa parameters. arg: Pointer to struct sx126x_lora_config_s */

#define SX126XIOC_LORACONFIGSET             _WLCIOC(SX126X_FIRST + 1)

/* Enter standby. arg: Pointer to enum sx126x_standby_mode_e or NULL (RC) */

#define SX126XIOC_STANDBY                   _WLCIOC(SX126X_FIRST + 2)

/* Enter sleep. arg: Pointer to uint8_t, 1 = warm start (configuration
 * retained), 0 = cold start.  NULL means warm start.
 */

#define SX126XIOC_SLEEP                     _WLCIOC(SX126X_FIRST + 3)

/* Start RX. arg: Pointer to uint32_t timeout in ms.  0 or NULL means
 * continuous RX, otherwise single RX with a hardware timeout.
 */

#define SX126XIOC_RX                        _WLCIOC(SX126X_FIRST + 4)

/* read() timeout. arg: Pointer to uint32_t ms, 0 = wait forever */

#define SX126XIOC_RXTIMEOUTSET              _WLCIOC(SX126X_FIRST + 5)
#define SX126XIOC_RXTIMEOUTGET              _WLCIOC(SX126X_FIRST + 6)

/* Preamble length in symbols. arg: Pointer to uint32_t */

#define SX126XIOC_PREAMBLESET               _WLCIOC(SX126X_FIRST + 7)
#define SX126XIOC_PREAMBLEGET               _WLCIOC(SX126X_FIRST + 8)

/* Channel activity detection, blocks until CadDone.
 * arg: Pointer to struct sx126x_cad_ioc_s
 */

#define SX126XIOC_CAD                       _WLCIOC(SX126X_FIRST + 9)

/* Start RX duty cycle (sniff mode). arg: Pointer to
 * struct sx126x_rxduty_ioc_s.  Stop it with SX126XIOC_STANDBY.
 */

#define SX126XIOC_RXDUTYCYCLE               _WLCIOC(SX126X_FIRST + 10)

/* DIO3 TCXO control. arg: Pointer to struct sx126x_tcxo_ioc_s */

#define SX126XIOC_TCXOSET                   _WLCIOC(SX126X_FIRST + 11)

/* DIO2 as RF switch control. arg: Pointer to uint8_t 0/1 */

#define SX126XIOC_DIO2RFSWSET               _WLCIOC(SX126X_FIRST + 12)

/* Regulator mode. arg: Pointer to enum sx126x_regulator_mode_e */

#define SX126XIOC_REGULATORSET              _WLCIOC(SX126X_FIRST + 13)

/* Instantaneous RSSI (radio must be in RX). arg: Pointer to int16_t dBm */

#define SX126XIOC_RSSIGET                   _WLCIOC(SX126X_FIRST + 14)

/* IRQ status and statistics. arg: Pointer to struct sx126x_irqstat_ioc_s */

#define SX126XIOC_IRQSTATUSGET              _WLCIOC(SX126X_FIRST + 15)

/* Fetch a packet pending in the chip buffer into the RX FIFO. No arg.
 * Returns 1 if a packet was queued, 0 if there was none.
 */

#define SX126XIOC_WAKEFETCH                 _WLCIOC(SX126X_FIRST + 16)

/* Capabilities. arg: Pointer to struct sx126x_caps_s */

#define SX126XIOC_GETCAPS                   _WLCIOC(SX126X_FIRST + 17)

/* Idle state after TX / single RX / CAD. arg: Pointer to uint8_t
 * SX126X_IDLE_*
 */

#define SX126XIOC_IDLESET                   _WLCIOC(SX126X_FIRST + 18)
#define SX126XIOC_IDLEGET                   _WLCIOC(SX126X_FIRST + 19)

/* Listen before talk: CAD before each write(). arg: Pointer to uint8_t */

#define SX126XIOC_LBTSET                    _WLCIOC(SX126X_FIRST + 20)

/* Time on air for the current settings. arg: Pointer to uint32_t,
 * in: payload length in bytes, out: time on air in microseconds.
 */

#define SX126XIOC_AIRTIMEGET                _WLCIOC(SX126X_FIRST + 21)

/* Inverted IQ. arg: Pointer to uint8_t 0/1 */

#define SX126XIOC_IQINVSET                  _WLCIOC(SX126X_FIRST + 22)

/* Boosted RX gain (+~2 dB sensitivity, +~0.6 mA). arg: Pointer to uint8_t */

#define SX126XIOC_RXBOOSTSET                _WLCIOC(SX126X_FIRST + 23)

/* arg: Pointer to struct sx126x_lora_config_s */

#define SX126XIOC_LORACONFIGGET             _WLCIOC(SX126X_FIRST + 24)

/* Number of commands used (SX126X_NCMDS in ioctl.h must be >= this) */

#define SX126X_NIOCTLS                      25

/* Idle states (SX126XIOC_IDLESET) */

#define SX126X_IDLE_STANDBY                 0 /* STDBY_RC */
#define SX126X_IDLE_RX                      1 /* Continuous RX */
#define SX126X_IDLE_SLEEP                   2 /* Warm sleep */

/* struct sx126x_read_hdr_s flags */

#define SX126X_RXHDR_F_WAKE                 (1 << 0) /* Fetched after wake */

/* struct sx126x_caps_s flags */

#define SX126X_CAPS_CAD                     (1 << 0)
#define SX126X_CAPS_RXDUTY                  (1 << 1)
#define SX126X_CAPS_WAKEIRQ                 (1 << 2) /* DIO1 wakes the MCU */
#define SX126X_CAPS_SLEEP                   (1 << 3)
#define SX126X_CAPS_LBT                     (1 << 4)
#define SX126X_CAPS_RXBOOST                 (1 << 5)
#define SX126X_CAPS_TCXO                    (1 << 6)
#define SX126X_CAPS_DIO2RFSW                (1 << 7)
#define SX126X_CAPS_BUSYPIN                 (1 << 8)

/* RX FIFO data *************************************************************/

#define SX126X_READ_DATA_MAX                SX126X_MAX_PAYLOAD
#define SX126X_READ_DATA_HEADER_LEN \
  (offsetof(struct sx126x_read_hdr_s, data))

/* IRQ Register bits ********************************************************/

#define SX126X_IRQ_TXDONE_MASK              (1 << 0)
#define SX126X_IRQ_RXDONE_MASK              (1 << 1)
#define SX126X_IRQ_PREAMBLEDETECTED_MASK    (1 << 2)
#define SX126X_IRQ_SYNCWORDVALID_MASK       (1 << 3)
#define SX126X_IRQ_HEADERVALID_MASK         (1 << 4)
#define SX126X_IRQ_HEADERERR_MASK           (1 << 5)
#define SX126X_IRQ_CRCERR_MASK              (1 << 6)
#define SX126X_IRQ_CADDONE_MASK             (1 << 7)
#define SX126X_IRQ_CADDETECTED_MASK         (1 << 8)
#define SX126X_IRQ_TIMEOUT_MASK             (1 << 9)
#define SX126X_IRQ_LRFHSSHOP_MASK           (1 << 14)

/* Others */

#define SX126X_NOP                   0
#define SX126X_NO_TIMEOUT            0
#define SX126X_NO_DELAY              0

/* Oscillators and PLLs */

#define SX126X_OSC_MAIN_HZ           (32000000)
#define SX126X_FXTAL                 SX126X_OSC_MAIN_HZ

/****************************************************************************
 * Public Data Types
 ****************************************************************************/

/* Standby config */

enum sx126x_standby_mode_e
{
  SX126X_STDBY_RC    = 0x00,
  SX126X_STDBY_XOSC  = 0x01
};

/* Packet Types */

enum sx126x_packet_type_e
{
  SX126X_PACKETTYPE_GFSK    = 0x00,
  SX126X_PACKETTYPE_LORA    = 0x01,
  SX126X_PACKETTYPE_LR_FHSS = 0x13
};

/* Ramp times */

enum sx126x_ramp_time_e
{
  SX126X_SET_RAMP_10U   = 0x00,
  SX126X_SET_RAMP_20U   = 0x01,
  SX126X_SET_RAMP_40U   = 0x02,
  SX126X_SET_RAMP_80U   = 0x03,
  SX126X_SET_RAMP_200U  = 0x04,
  SX126X_SET_RAMP_800U  = 0x05,
  SX126X_SET_RAMP_1700U = 0x06,
  SX126X_SET_RAMP_3400U = 0x07
};

/* GFSK Pulse shapes */

enum sx126x_gfsk_pulseshape_e
{
  SX126X_GFSK_PULSESHAPE_NONE            = 0x00,
  SX126X_GFSK_PULSESHAPE_GAUSSIAN_BT_0_3 = 0x08,
  SX126X_GFSK_PULSESHAPE_GAUSSIAN_BT_0_5 = 0x09,
  SX126X_GFSK_PULSESHAPE_GAUSSIAN_BT_0_7 = 0x0a,
  SX126X_GFSK_PULSESHAPE_GAUSSIAN_BT_1   = 0x0b
};

/* GFSK Bandwidths in Hz */

enum sx126x_gfsk_bandwidth_e
{
  SX126X_GFSK_BANDWIDTH_4800HZ   = 0x1f,
  SX126X_GFSK_BANDWIDTH_5800HZ   = 0x17,
  SX126X_GFSK_BANDWIDTH_7300HZ   = 0x0f,
  SX126X_GFSK_BANDWIDTH_9700HZ   = 0x1e,
  SX126X_GFSK_BANDWIDTH_11700HZ  = 0x16,
  SX126X_GFSK_BANDWIDTH_14600HZ  = 0x0e,
  SX126X_GFSK_BANDWIDTH_19500HZ  = 0x1d,
  SX126X_GFSK_BANDWIDTH_23400HZ  = 0x15,
  SX126X_GFSK_BANDWIDTH_29300HZ  = 0x0d,
  SX126X_GFSK_BANDWIDTH_39000HZ  = 0x1c,
  SX126X_GFSK_BANDWIDTH_46900HZ  = 0x14,
  SX126X_GFSK_BANDWIDTH_58600HZ  = 0x0c,
  SX126X_GFSK_BANDWIDTH_78200HZ  = 0x1b,
  SX126X_GFSK_BANDWIDTH_93800HZ  = 0x13,
  SX126X_GFSK_BANDWIDTH_117300HZ = 0x0b,
  SX126X_GFSK_BANDWIDTH_156200HZ = 0x1a,
  SX126X_GFSK_BANDWIDTH_187200HZ = 0x12,
  SX126X_GFSK_BANDWIDTH_234300HZ = 0x0a,
  SX126X_GFSK_BANDWIDTH_312000HZ = 0x19,
  SX126X_GFSK_BANDWIDTH_373600HZ = 0x11,
  SX126X_GFSK_BANDWIDTH_467000HZ = 0x09
};

/* LoRa Spreading Factors */

enum sx126x_lora_sf_e
{
  SX126X_LORA_SF5  = 0x05,
  SX126X_LORA_SF6  = 0x06,
  SX126X_LORA_SF7  = 0x07,
  SX126X_LORA_SF8  = 0x08,
  SX126X_LORA_SF9  = 0x09,
  SX126X_LORA_SF10 = 0x0a,
  SX126X_LORA_SF11 = 0x0b,
  SX126X_LORA_SF12 = 0x0c
};

/* LoRa Bandwidths */

enum sx126x_lora_bw_e
{
  SX126X_LORA_BW_7   = 0x00,
  SX126X_LORA_BW_10  = 0x08,
  SX126X_LORA_BW_15  = 0x01,
  SX126X_LORA_BW_20  = 0x09,
  SX126X_LORA_BW_31  = 0x02,
  SX126X_LORA_BW_41  = 0x0a,
  SX126X_LORA_BW_62  = 0x03,
  SX126X_LORA_BW_125 = 0x04,
  SX126X_LORA_BW_250 = 0x05,
  SX126X_LORA_BW_500 = 0x06
};

/* LoRa Coding Rates */

enum sx126x_lora_cr_e
{
  SX126X_LORA_CR_4_5 = 0x01,
  SX126X_LORA_CR_4_6 = 0x02,
  SX126X_LORA_CR_4_7 = 0x03,
  SX126X_LORA_CR_4_8 = 0x04
};

/* CAD Exit modes */

enum sx126x_cad_exit_mode_e
{
  SX126X_CAD_ONLY = 0x00,
  SX126X_CAD_RX   = 0x01
};

/* TCXO voltages */

enum sx126x_tcxo_voltage_e
{
  SX126X_TCXO_1_6V = 0x00,
  SX126X_TCXO_1_7V = 0x01,
  SX126X_TCXO_1_8V = 0x02,
  SX126X_TCXO_2_2V = 0x03,
  SX126X_TCXO_2_4V = 0x04,
  SX126X_TCXO_2_7V = 0x05,
  SX126X_TCXO_3_0V = 0x06,
  SX126X_TCXO_3_3V = 0x07
};

/* Fallback modes */

enum sx126x_fallback_mode_e
{
  SX126X_FALLBACK_FS          = 0x40,
  SX126X_FALLBACK_STDBY_XOSC  = 0x30,
  SX126X_FALLBACK_STDBY_RC    = 0x20
};

/* Regulator modes */

enum sx126x_regulator_mode_e
{
  SX126X_LDO         = 0x00,
  SX126X_DC_DC_LDO   = 0x01
};

/* Device */

enum sx126x_device_e
{
  SX1261 =            0x01,
  SX1262 =            0x00
};

enum sx126x_gfsk_preamble_detect_e
{
  SX126X_GFSK_PREAMBLE_DETECT_OFF,
  SX126X_GFSK_PREAMBLE_DETECT_8B,
  SX126X_GFSK_PREAMBLE_DETECT_16B,
  SX126X_GFSK_PREAMBLE_DETECT_24B,
  SX126X_GFSK_PREAMBLE_DETECT_32B
};

/* Addr comp */

enum sx126x_address_filtering_e
{
  SX126X_ADDR_FILT_DISABLED,
  SX126X_ADDR_FILT_NODE,
  SX126X_ADDR_FILT_NODE_BROADCAST
};

/* GFSK CRC types */

enum sx126x_gfsk_crc_type_e
{
  SX126X_GFSK_CRCTYPE_OFF         = 0x01,
  SX126X_GFSK_CRCTYPE_1_BYTE      = 0x00,
  SX126X_GFSK_CRCTYPE_2_BYTE      = 0x02,
  SX126X_GFSK_CRCTYPE_1_BYTE_INV  = 0x04,
  SX126X_GFSK_CRCTYPE_2_BYTE_INV  = 0x06
};

/* LoRa mod params */

struct sx126x_modparams_lora_s
{
  enum sx126x_lora_sf_e spreading_factor;
  enum sx126x_lora_bw_e bandwidth;
  enum sx126x_lora_cr_e coding_rate;
  bool low_datarate_optimization;
};

/* GFSK mod params */

struct sx126x_modparams_gfsk_s
{
  uint32_t bitrate;
  enum sx126x_gfsk_pulseshape_e pulseshape;
  enum sx126x_gfsk_bandwidth_e bandwidth;
  uint32_t frequency_deviation;
};

/* LoRa packet params */

struct sx126x_packetparams_lora_s
{
  uint16_t preambles;
  bool fixed_length_header;
  uint8_t payload_length;
  bool crc_enable;
  bool invert_iq;
};

/* GFSK packet params */

struct sx126x_packetparams_gfsk_s
{
  uint16_t preambles;
  enum sx126x_gfsk_preamble_detect_e preamble_detect;
  uint8_t syncword_length;
  enum sx126x_address_filtering_e address_filtering;
  bool include_packet_size;
  uint8_t packet_length;
  enum sx126x_gfsk_crc_type_e crc_type;
  bool whitening_enable;
};

/* Config */

struct sx126x_lora_config_s
{
  struct sx126x_modparams_lora_s modulation;
  struct sx126x_packetparams_lora_s packet;
};

/* Driver radio states (struct sx126x_irqstat_ioc_s.state) */

enum sx126x_state_e
{
  SX126X_STATE_UNKNOWN   = 0,
  SX126X_STATE_STANDBY   = 1,
  SX126X_STATE_SLEEP     = 2,
  SX126X_STATE_RX_CONT   = 3,
  SX126X_STATE_RX_SINGLE = 4,
  SX126X_STATE_RX_DUTY   = 5,
  SX126X_STATE_TX        = 6,
  SX126X_STATE_CAD       = 7
};

/* read() data.  read() returns SX126X_READ_DATA_HEADER_LEN + datalen. */

struct sx126x_read_hdr_s
{
  uint16_t datalen;                     /* Payload bytes in data[] */
  int8_t   snr;                         /* SNR in dB (SnrPkt / 4) */
  uint8_t  flags;                       /* SX126X_RXHDR_F_* */
  int16_t  rssi;                        /* RssiPkt in dBm */
  int16_t  signal_rssi;                 /* SignalRssiPkt in dBm */
  uint8_t  crc_err;                     /* 1 if the payload CRC failed */
  uint8_t  reserved;
  uint8_t  data[SX126X_READ_DATA_MAX];
};

/* SX126XIOC_CAD */

struct sx126x_cad_ioc_s
{
  uint8_t  symbols;       /* in: 1, 2, 4, 8 or 16. 0 = default for SF */
  uint8_t  det_peak;      /* in: cadDetPeak. 0 = default (SF + 13) */
  uint8_t  det_min;       /* in: cadDetMin. 0 = default (10) */
  uint8_t  exit_mode;     /* in: enum sx126x_cad_exit_mode_e */
  uint32_t rx_timeout_ms; /* in: SX126X_CAD_RX only, RX time after a
                           *     detection.  0 = single RX, no timeout */
  bool     busy;          /* out: channel activity detected */
};

/* SX126XIOC_RXDUTYCYCLE.  The transmitter preamble must last longer than
 * 2 * rx_us + sleep_us to be caught.
 */

struct sx126x_rxduty_ioc_s
{
  uint32_t rx_us;         /* RX window in microseconds */
  uint32_t sleep_us;      /* Sleep time in microseconds */
};

/* SX126XIOC_TCXOSET.  Applied immediately and on every re-initialization */

struct sx126x_tcxo_ioc_s
{
  uint8_t  voltage;       /* enum sx126x_tcxo_voltage_e */
  uint32_t delay_us;      /* TCXO start-up time, > 0 */
};

/* SX126XIOC_IRQSTATUSGET */

struct sx126x_irqstat_ioc_s
{
  uint16_t latched;       /* IRQ bits seen by the driver since the last
                           * SX126XIOC_IRQSTATUSGET (cleared by the call) */
  uint16_t pending;       /* IRQ bits set in the chip now (0 if asleep) */
  uint16_t deverr;        /* GetDeviceErrors (0 if asleep) */
  uint8_t  state;         /* enum sx126x_state_e */
  uint8_t  chipmode;      /* GetStatus chip mode, 0xff if not read */
  uint32_t rx_packets;    /* Packets queued in the RX FIFO */
  uint32_t rx_crcerr;     /* Packets with CRC errors */
  uint32_t rx_overruns;   /* Packets dropped because the FIFO was full */
  uint32_t rx_timeouts;   /* RX timeouts */
  uint32_t tx_packets;    /* Completed transmissions */
  uint32_t tx_timeouts;   /* Transmissions without TxDone */
};

/* SX126XIOC_GETCAPS */

struct sx126x_caps_s
{
  uint32_t flags;         /* SX126X_CAPS_* */
  uint16_t max_payload;   /* 255 */
  int8_t   txpower_min;   /* dBm */
  int8_t   txpower_max;   /* dBm */
  uint8_t  device;        /* enum sx126x_device_e */
  uint8_t  rxfifo_len;    /* RX FIFO depth in packets */
};

/* Lower driver *************************************************************/

struct sx126x_irq_masks
{
  uint16_t dio1_mask;     /* Extra IRQs routed to DIO1 (ORed with the IRQs
                           * the driver needs) */
  uint16_t dio2_mask;     /* Ignored when DIO2 is the RF switch */
  uint16_t dio3_mask;     /* Ignored when DIO3 controls a TCXO */
};

struct sx126x_lower_s
{
  /* Index of radio to register.
   * ex: 0 is the primary radio, 1 is the secondary.
   * Must be within the maximum configured radios.
   */

  unsigned int dev_number;

  /* Pulse NRESET (optional). The driver waits for BUSY afterwards. */

  CODE void (*reset)(void);

  /* Read the BUSY pin, true while the chip is busy (strongly recommended).
   * Without it the driver uses fixed delays.
   */

  CODE bool (*busy)(void);

  /* This controls which DIO reacts to interrupts
   * Depended on the pinout of the board / module.
   * Note that DIO 2 and DIO 3 can be already in use
   * by the module and setting them might interfere
   * with the operation or even damage them.
   */

  struct sx126x_irq_masks masks;

  /* DIO3 TCXO control.  dio3_delay is in units of 15.625 us, 0 means no
   * TCXO (crystal).
   */

  enum sx126x_tcxo_voltage_e dio3_voltage;
  uint32_t dio3_delay;
  uint8_t use_dio2_as_rf_sw;

  /* Attach the DIO1 interrupt handler (rising edge).  Required. */

  CODE int (*irq0attach)(xcpt_t handler, FAR void *arg);

  /* Enable / disable the DIO1 interrupt (optional) */

  CODE void (*irq0enable)(bool enable);

  /* Attach without reset: set when the MCU was woken from deep sleep by
   * DIO1.  The chip is not reset or reconfigured at registration, and a
   * packet pending in its buffer is moved into the RX FIFO.
   */

  bool no_reset;

  /* DIO1 is wired to a pin that can wake the MCU from deep sleep */

  bool dio1_wake;

  /* The regulator mode is board / module depended */

  enum sx126x_regulator_mode_e regulator_mode;

  /* Power amplifier control. DO NOT exceeds the
   * limits listed in SX1261-2 V2 datasheet.
   * 13.1.14 SetPaConfig
   * This can cause damage to the device.
   * Optional: NULL means SX1262, hpmax 0x07, padutycycle 0x04 (+22 dBm).
   */

  CODE int (*get_pa_values)(FAR enum sx126x_device_e *model,
                            FAR uint8_t *hpmax, FAR uint8_t *padutycycle);

  /* TX power control. Depending on the local RF regulations,
   * power might have to be limited.
   * Also depending on board or module,
   * power values have different charactersitics.
   * More info in sx1261-2 V2 datasheet 13.4.4 SetTxParams.
   * int8_t *power (dBm) is set and this function may limit it.
   * Optional.
   */

  CODE int (*limit_tx_power)(FAR int8_t *current_power);

  enum sx126x_ramp_time_e tx_ramp_time;

  /* Frequency control
   * Typically boards have a limited range of frequencies.
   * Exceeding these can damage the radio.
   * Also depending on regulations, some frequencies are restricted.
   * This must return non zero in case a frequency is denied.
   * Optional.
   */

  CODE int (*check_frequency)(uint32_t frequency);
};

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

/****************************************************************************
 * Name: sx126x_register
 *
 * Description:
 *   Register the SX126x character driver at 'path'.  With lower->no_reset
 *   the chip is left untouched and a pending received packet is moved to
 *   the RX FIFO.
 *
 * Returned Value:
 *   OK on success, a negated errno value on failure.
 *
 ****************************************************************************/

int sx126x_register(FAR struct spi_dev_s *spi,
                    FAR const struct sx126x_lower_s *lower,
                    FAR const char *path);

#endif /* __INCLUDE_NUTTX_WIRELESS_LPWAN_SX126X_H */
