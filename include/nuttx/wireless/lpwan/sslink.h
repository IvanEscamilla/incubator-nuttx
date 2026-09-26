/****************************************************************************
 * include/nuttx/wireless/lpwan/sslink.h
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

/* SSLink: encrypted LoRa star network exposed as a character driver.
 *
 * Applications open /dev/sslinkN and use:
 *
 *   read()/poll()  -> struct sslink_event_s (messages and link events)
 *   write()        <- struct sslink_tx_s    (uplink on a node, queued
 *                                            downlink on a gateway)
 *   ioctl()        <- SSLINKIOC_* below
 *
 * Message payloads (struct sslink_*_s marked "wire") are the exact bytes
 * carried over the air inside the encrypted part of a frame.  Both ends are
 * little-endian ESP32 parts, so they are packed C structs.
 */

#ifndef __INCLUDE_NUTTX_WIRELESS_LPWAN_SSLINK_H
#define __INCLUDE_NUTTX_WIRELESS_LPWAN_SSLINK_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <nuttx/compiler.h>
#include <nuttx/wireless/ioctl.h>

#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-Processor Declarations
 ****************************************************************************/

#ifdef CONFIG_ENDIAN_BIG
#  error "SSLink wire structs assume a little-endian target"
#endif

/* Protocol constants *******************************************************/

#define SSLINK_PROTO_VERSION      1

#define SSLINK_HDR_LEN            9   /* ctrl + net_id + dev_addr + fcnt16 */
#define SSLINK_MIC_LEN            8   /* AES-CCM tag */
#define SSLINK_OVERHEAD           (SSLINK_HDR_LEN + 1 + SSLINK_MIC_LEN)

#ifdef CONFIG_SSLINK_MAX_FRAME
#  define SSLINK_MAX_FRAME        CONFIG_SSLINK_MAX_FRAME
#else
#  define SSLINK_MAX_FRAME        64
#endif

#define SSLINK_MAX_PAYLOAD        (SSLINK_MAX_FRAME - SSLINK_OVERHEAD)

/* SSLINK_OTA_BLOCK_SIZE's Kconfig range (16..234) matches the maximum
 * possible SSLINK_MAX_PAYLOAD (255 - 18 overhead), but a smaller
 * SSLINK_MAX_FRAME still leaves room for a block size Kconfig accepted.
 * Catch that combination at compile time rather than at the first OTA
 * transfer.
 */

#ifdef CONFIG_SSLINK_OTA_BLOCK_SIZE
#  if (CONFIG_SSLINK_OTA_BLOCK_SIZE + 3) > SSLINK_MAX_PAYLOAD
#    error "SSLINK_OTA_BLOCK_SIZE + 3 exceeds SSLINK_MAX_PAYLOAD"
#  endif
#endif

#define SSLINK_EUI_LEN            8
#define SSLINK_ADDR_BROADCAST     0xffffffffu

/* net_id of a JOIN_REQ from a node that does not know its gateway yet.
 * Never a gateway's own net_id.
 */

#define SSLINK_NET_ID_ANY         0xffffu
#define SSLINK_OTA_PATH_MAX       64

/* Message types (first byte of the encrypted payload) **********************/

/* 0x00-0x1f: system messages */

#define SSLINK_MSG_MAC_ACK            0x00  /* down: struct sslink_ack_s, in an
                                             * SSLINK_F_ACK frame (SPEC-mac.md) */
#define SSLINK_MSG_DEVICE_INFO        0x01  /* up:   struct sslink_device_info_s */
#define SSLINK_MSG_DIAG               0x02  /* up:   struct sslink_diag_s */

#define SSLINK_CMD_SET_REPORT_POLICY  0x10  /* down: struct sslink_policy_s */
#define SSLINK_CMD_SET_RADIO          0x11  /* down: struct sslink_radio_cfg_s */
#define SSLINK_CMD_SET_WAKE_POLICY    0x12  /* down: struct sslink_wake_policy_s */
#define SSLINK_CMD_REQUEST_STATUS     0x13  /* down: no payload */
#define SSLINK_CMD_TIME_SYNC          0x14  /* down: struct sslink_time_sync_s */
#define SSLINK_CMD_REBOOT             0x15  /* down: struct sslink_reboot_s */
#define SSLINK_CMD_UNPAIR             0x16  /* down: no payload */
#define SSLINK_CMD_DEV_MODE           0x17  /* down: struct sslink_devmode_s */

#define SSLINK_OTA_BEGIN              0x18  /* down: struct sslink_ota_begin_s */
#define SSLINK_OTA_READY              0x19  /* up:   struct sslink_ota_ready_s */
#define SSLINK_OTA_BLOCK              0x1a  /* down: struct sslink_ota_block_s + data */
#define SSLINK_OTA_ACK                0x1b  /* up:   struct sslink_ota_ack_s */
#define SSLINK_OTA_DONE               0x1c  /* up:   struct sslink_ota_done_s */
#define SSLINK_OTA_ABORT              0x1d  /* both: struct sslink_ota_abort_s */

/* 0x20-0x7f: standard telemetry */

#define SSLINK_MSG_TANK_LEVEL         0x20  /* up:   struct sslink_tank_level_s */

/* 0x80-0xff: application defined, passed through untouched */

#define SSLINK_MSG_APP_FIRST          0x80

#define SSLINK_MSG_IS_SYSTEM(t)       ((t) < 0x20)
#define SSLINK_MSG_IS_APP(t)          ((t) >= SSLINK_MSG_APP_FIRST)

/* Tank level error flags (sslink_tank_level_s.err_flags).  Any newly set
 * bit forces an immediate, acknowledged report.
 */

#define SSLINK_ERR_MEAS_ERROR         (1u << 0)
#define SSLINK_ERR_SENSOR_FAULT       (1u << 1)
#define SSLINK_ERR_SENSOR_DISCONNECT  (1u << 2)
#define SSLINK_ERR_LEAK_DETECTED      (1u << 3)
#define SSLINK_ERR_BATT_FAULT         (1u << 4)
#define SSLINK_ERR_CHARGER_FAULT      (1u << 5)
#define SSLINK_ERR_BATT_OVERTEMP      (1u << 6)
#define SSLINK_ERR_SYS_OVERTEMP       (1u << 7)
#define SSLINK_ERR_STORAGE_FAULT      (1u << 8)
#define SSLINK_ERR_RADIO_FAULT        (1u << 9)

/* Tank level warning flags (sslink_tank_level_s.warn_flags) */

#define SSLINK_WARN_LOW_LEVEL         (1u << 0)
#define SSLINK_WARN_CRITICAL_LEVEL    (1u << 1)
#define SSLINK_WARN_BATT_LOW          (1u << 2)
#define SSLINK_WARN_BATT_CRITICAL     (1u << 3)
#define SSLINK_WARN_TEMP_HIGH         (1u << 4)
#define SSLINK_WARN_TEMP_LOW          (1u << 5)
#define SSLINK_WARN_WEAK_LINK         (1u << 6)
#define SSLINK_WARN_SENSOR_UNSTABLE   (1u << 7)
#define SSLINK_WARN_CALIBRATION       (1u << 8)
#define SSLINK_WARN_DEV_MODE_ACTIVE   (1u << 9)

/* write() flags (sslink_tx_s.flags) */

#define SSLINK_TXF_ACK                (1 << 0) /* Request an acknowledgement */
#define SSLINK_TXF_URGENT             (1 << 1) /* Skip the random TX backoff */
#define SSLINK_TXF_WAKE               (1 << 2) /* Gateway: send now with a
                                                * wake-on-radio preamble */
#define SSLINK_TXF_NOLBT              (1 << 3) /* Skip listen-before-talk */

/* Dev mode flags (sslink_devmode_s.flags) */

#define SSLINK_DEVMODE_KEEP_AWAKE     (1 << 0) /* Never enter deep sleep */
#define SSLINK_DEVMODE_VERBOSE_DIAG   (1 << 1) /* Send SSLINK_MSG_DIAG */
#define SSLINK_DEVMODE_CONT_RX        (1 << 2) /* Keep the receiver on */

/* Event mask bits for SSLINKIOC_SETFILTER (1 << enum sslink_evt_e) */

#define SSLINK_EVTMASK(e)             (1u << (e))
#define SSLINK_EVTMASK_ALL            0xffffffffu

/* IOCTL commands ***********************************************************/

#define _SSLINKIOC(n)                 _WLCIOC(SSLINK_FIRST + (n))

/* Common */

#define SSLINKIOC_GETINFO         _SSLINKIOC(0)  /* arg: struct sslink_info_s * */
#define SSLINKIOC_SETPHY          _SSLINKIOC(1)  /* arg: const struct sslink_radio_cfg_s * */
#define SSLINKIOC_GETPHY          _SSLINKIOC(2)  /* arg: struct sslink_radio_cfg_s * */
#define SSLINKIOC_GETSTATS        _SSLINKIOC(3)  /* arg: struct sslink_stats_s * */
#define SSLINKIOC_RESETSTATS      _SSLINKIOC(4)  /* arg: none */
#define SSLINKIOC_SETFILTER       _SSLINKIOC(5)  /* arg: const struct sslink_filter_s * */
#define SSLINKIOC_STORE_EXPORT    _SSLINKIOC(6)  /* arg: struct sslink_store_blob_s * */
#define SSLINKIOC_STORE_IMPORT    _SSLINKIOC(7)  /* arg: const struct sslink_store_blob_s * */

/* Node role */

#define SSLINKIOC_PAIR            _SSLINKIOC(16) /* arg: uint32_t timeout_ms (value).
                                                  * Blocks. 0, -ETIMEDOUT or
                                                  * -ECONNREFUSED */
#define SSLINKIOC_UNPAIR          _SSLINKIOC(17) /* arg: none */
#define SSLINKIOC_ISPAIRED        _SSLINKIOC(18) /* arg: none. Returns 1 or 0 */
#define SSLINKIOC_SET_POLICY      _SSLINKIOC(19) /* arg: const struct sslink_policy_s * */
#define SSLINKIOC_GET_POLICY      _SSLINKIOC(20) /* arg: struct sslink_policy_s * */
#define SSLINKIOC_ACTIVITY        _SSLINKIOC(21) /* arg: none. Motion/change hint */
#define SSLINKIOC_SHOULD_SEND     _SSLINKIOC(22) /* arg: const struct sslink_sample_s *.
                                                  * Returns 1 when a report is due */
#define SSLINKIOC_NEXT_REPORT_MS  _SSLINKIOC(23) /* arg: uint32_t * */
#define SSLINKIOC_SLEEP_HINT      _SSLINKIOC(24) /* arg: uint8_t * (enum sslink_sleep_e) */
#define SSLINKIOC_DEVMODE         _SSLINKIOC(25) /* arg: const struct sslink_devmode_s * */
#define SSLINKIOC_SET_WAKE_POLICY _SSLINKIOC(26) /* arg: const struct sslink_wake_policy_s * */
#define SSLINKIOC_OTA_CONFIRM     _SSLINKIOC(27) /* arg: none */

/* Gateway role */

#define SSLINKIOC_PAIRING_OPEN    _SSLINKIOC(32) /* arg: uint32_t timeout_s (value) */
#define SSLINKIOC_PAIRING_CLOSE   _SSLINKIOC(33) /* arg: none */
#define SSLINKIOC_DEV_COUNT       _SSLINKIOC(34) /* arg: none. Returns the count */
#define SSLINKIOC_DEV_GET         _SSLINKIOC(35) /* arg: struct sslink_devinfo_s *
                                                  * (dev_addr filled in) */
#define SSLINKIOC_DEV_LIST        _SSLINKIOC(36) /* arg: struct sslink_devlist_s * */
#define SSLINKIOC_DEV_REMOVE      _SSLINKIOC(37) /* arg: uint32_t dev_addr (value) */
#define SSLINKIOC_DEV_SET_POLICY  _SSLINKIOC(38) /* arg: const struct sslink_dev_policy_s * */
#define SSLINKIOC_DEV_DEVMODE     _SSLINKIOC(39) /* arg: const struct sslink_dev_devmode_s * */
#define SSLINKIOC_OTA_START       _SSLINKIOC(40) /* arg: const struct sslink_ota_start_s * */
#define SSLINKIOC_OTA_ABORT       _SSLINKIOC(41) /* arg: uint32_t dev_addr (value) */
#define SSLINKIOC_OTA_STATUS      _SSLINKIOC(42) /* arg: struct sslink_ota_status_s *
                                                  * (dev_addr filled in) */

/* Gateway pairing candidates: nodes heard in discovery that the installer
 * has not yet confirmed (SPEC-handshake.md 4).  Confirm and reject take a
 * pointer, since an EUI does not fit in an unsigned long on xtensa.
 */

#define SSLINKIOC_CAND_COUNT      _SSLINKIOC(43) /* arg: none. Returns the count */
#define SSLINKIOC_CAND_LIST       _SSLINKIOC(44) /* arg: struct sslink_candlist_s * */
#define SSLINKIOC_CAND_CONFIRM    _SSLINKIOC(45) /* arg: const struct sslink_eui_s *.
                                                  * Answers with JOIN_RSP; the
                                                  * outcome is SSLINK_EVT_PAIRED or
                                                  * SSLINK_EVT_PAIR_REJECTED */
#define SSLINKIOC_CAND_REJECT     _SSLINKIOC(46) /* arg: const struct sslink_eui_s * */

/* Size helpers for read()/write() */

#define SSLINK_EVENT_SIZE(n)      (offsetof(struct sslink_event_s, data) + (n))
#define SSLINK_TX_SIZE(n)         (offsetof(struct sslink_tx_s, data) + (n))

/****************************************************************************
 * Public Types
 ****************************************************************************/

enum sslink_role_e
{
  SSLINK_ROLE_NODE = 0,
  SSLINK_ROLE_GATEWAY,
};

/* Device types, reported in pairing and SSLINK_MSG_DEVICE_INFO */

enum sslink_dev_type_e
{
  SSLINK_DEV_UNKNOWN    = 0,
  SSLINK_DEV_GATEWAY    = 1,
  SSLINK_DEV_GAS_TANK   = 2,
  SSLINK_DEV_WATER_TANK = 3,
};

/* Radio profiles (500 kHz bandwidth, see the SSLINK_PROFILE_* Kconfig) */

enum sslink_profile_e
{
  SSLINK_PROFILE_FAST = 0,  /* SF7:  real-time reporting and OTA */
  SSLINK_PROFILE_NORMAL,    /* SF9 */
  SSLINK_PROFILE_LONG,      /* SF10 */
  SSLINK_PROFILE_COUNT
};

/* Sleep depth the node can afford until its next report */

enum sslink_sleep_e
{
  SSLINK_SLEEP_LIGHT = 0,   /* Next report is soon: light sleep only */
  SSLINK_SLEEP_DEEP,        /* Deep sleep, radio in warm sleep */
  SSLINK_SLEEP_DEEP_SNIFF,  /* Deep sleep, radio in RX duty cycle */
};

/* Event types returned by read() */

enum sslink_evt_e
{
  SSLINK_EVT_MSG = 0,         /* data: message payload of msg_type */
  SSLINK_EVT_TX_DONE,         /* data: struct sslink_txdone_s */
  SSLINK_EVT_PAIRED,          /* data: struct sslink_paired_s */
  SSLINK_EVT_PAIR_REJECTED,   /* data: struct sslink_paired_s */
  SSLINK_EVT_UNPAIRED,        /* no data */
  SSLINK_EVT_DEV_ONLINE,      /* gateway, no data */
  SSLINK_EVT_DEV_OFFLINE,     /* gateway, no data */
  SSLINK_EVT_POLICY,          /* node, data: struct sslink_policy_s */
  SSLINK_EVT_DEVMODE,         /* data: struct sslink_devmode_s */
  SSLINK_EVT_OTA,             /* data: struct sslink_ota_evt_s */
  SSLINK_EVT_OVERFLOW,        /* this reader's queue dropped events */
  SSLINK_EVT_CANDIDATE,       /* gateway, data: struct sslink_candidate_s.
                               * A candidate was added or refreshed */
};

/* Why a pairing did not complete: struct sslink_paired_s.reason.  1..7
 * are the NACK reasons on air (SPEC-handshake.md 4.1); a NACK is
 * unauthenticated, so a node treats them as advice.  From 16 on they are
 * local and never sent.
 */

enum sslink_join_reject_e
{
  SSLINK_REJECT_NONE          = 0,
  SSLINK_REJECT_NOT_OPEN      = 1,  /* gateway not in discovery */
  SSLINK_REJECT_WEAK_SIGNAL   = 2,  /* below CONFIG_SSLINK_PAIR_RSSI_MIN */
  SSLINK_REJECT_BAD_FINISHED  = 3,
  SSLINK_REJECT_UNKNOWN_DEV   = 4,  /* resumption failed */
  SSLINK_REJECT_TABLE_FULL    = 5,  /* candidate or device table */
  SSLINK_REJECT_MALFORMED     = 6,
  SSLINK_REJECT_AUTH_VERSION  = 7,  /* unknown auth key version */
  SSLINK_REJECT_TIMEOUT       = 16, /* window elapsed, no answer */
  SSLINK_REJECT_NO_ACK        = 17, /* node: JOIN_CFM never acknowledged */
  SSLINK_REJECT_BY_USER       = 18, /* gateway: installer rejected */
};

enum sslink_tank_kind_e
{
  SSLINK_TANK_GAS = 0,
  SSLINK_TANK_WATER,
};

enum sslink_tank_state_e
{
  SSLINK_TANK_UNKNOWN = 0,
  SSLINK_TANK_STEADY,
  SSLINK_TANK_FILLING,        /* refueling / filling */
  SSLINK_TANK_CONSUMING,      /* consuming / draining */
};

enum sslink_charge_state_e
{
  SSLINK_CHARGE_NONE = 0,     /* no charger fitted */
  SSLINK_CHARGE_DISCHARGING,
  SSLINK_CHARGE_CHARGING,
  SSLINK_CHARGE_FULL,
  SSLINK_CHARGE_SOLAR_NO_SUN,
  SSLINK_CHARGE_FAULT,
};

enum sslink_ota_state_e
{
  SSLINK_OTA_IDLE = 0,
  SSLINK_OTA_ANNOUNCED,       /* gateway: BEGIN queued, waiting for READY */
  SSLINK_OTA_TRANSFER,
  SSLINK_OTA_VERIFY,
  SSLINK_OTA_DONE_PENDING,    /* image written, reboot pending */
  SSLINK_OTA_CONFIRMED,
  SSLINK_OTA_FAILED,
};

/* Wire structs *************************************************************/

/* SSLINK_MSG_MAC_ACK: the body of an ACK frame (SPEC-mac.md 1) */

begin_packed_struct struct sslink_ack_s
{
  uint16_t acked_fcnt16;      /* fcnt16 of the frame acknowledged */
  int16_t  rssi;              /* as the gateway measured that frame */
  int8_t   snr;
} end_packed_struct;

/* SSLINK_MSG_TANK_LEVEL: gas and water tank sensors */

begin_packed_struct struct sslink_tank_level_s
{
  uint8_t  sensor_kind;       /* enum sslink_tank_kind_e */
  uint16_t level_centipct;    /* 0..10000 */
  uint8_t  state;             /* enum sslink_tank_state_e */
  uint16_t batt_mv;
  uint8_t  batt_pct;
  int16_t  batt_temp_dc;      /* 0.1 degC */
  int16_t  sys_temp_dc;       /* 0.1 degC */
  uint8_t  charge_state;      /* enum sslink_charge_state_e */
  uint32_t err_flags;         /* SSLINK_ERR_* */
  uint32_t warn_flags;        /* SSLINK_WARN_* */
  uint16_t report_interval_s; /* current interval, for gateway liveness */
  uint16_t seq_sample;        /* sample counter, reveals gaps */
} end_packed_struct;

/* SSLINK_MSG_DEVICE_INFO */

begin_packed_struct struct sslink_device_info_s
{
  uint8_t  dev_type;          /* enum sslink_dev_type_e */
  uint8_t  eui[SSLINK_EUI_LEN];
  uint8_t  fw_version[4];     /* major, minor, patch, build */
  uint8_t  hw_rev;
  uint8_t  image_hash[8];     /* first bytes of the running image SHA-256 */
  uint16_t boot_count;
  uint8_t  reset_reason;
  uint8_t  radio_caps;
} end_packed_struct;

/* SSLINK_MSG_DIAG (dev mode) */

begin_packed_struct struct sslink_diag_s
{
  uint32_t uptime_s;
  uint16_t tx_count;
  uint16_t tx_failed;
  uint16_t retries;
  uint16_t rx_count;
  int16_t  last_rssi;
  int8_t   last_snr;
  int8_t   tx_power;
  uint8_t  profile;
  uint8_t  wake_cause;
  uint32_t free_heap;
} end_packed_struct;

/* SSLINK_CMD_SET_REPORT_POLICY, SSLINKIOC_SET/GET_POLICY, SSLINK_EVT_POLICY */

begin_packed_struct struct sslink_policy_s
{
  uint32_t active_ms;         /* interval while activity is detected */
  uint32_t idle_min_ms;       /* first idle interval */
  uint32_t idle_max_ms;       /* idle backoff ceiling */
  uint32_t hold_ms;           /* quiet time before switching to idle */
  uint16_t change_delta;      /* level change (centi-%) that forces a send */
  uint8_t  ack_every_n;       /* request an ACK every N reports (0: never) */
  uint16_t max_interval_s;    /* liveness guarantee */
} end_packed_struct;

/* SSLINK_CMD_SET_RADIO, SSLINKIOC_SETPHY/GETPHY */

begin_packed_struct struct sslink_radio_cfg_s
{
  uint8_t  profile;           /* enum sslink_profile_e */
  int8_t   txpower_max_dbm;
  uint8_t  link_adapt;        /* 1: adapt TX power and SF from ACK SNR */
} end_packed_struct;

/* SSLINK_CMD_SET_WAKE_POLICY, SSLINKIOC_SET_WAKE_POLICY */

begin_packed_struct struct sslink_wake_policy_s
{
  uint8_t  enable;            /* RX duty cycle while in deep sleep */
  uint16_t rx_us;
  uint16_t sleep_ms;
} end_packed_struct;

/* SSLINK_CMD_TIME_SYNC */

begin_packed_struct struct sslink_time_sync_s
{
  uint32_t unix_s;
} end_packed_struct;

/* SSLINK_CMD_REBOOT */

begin_packed_struct struct sslink_reboot_s
{
  uint16_t delay_s;
} end_packed_struct;

/* SSLINK_CMD_DEV_MODE, SSLINKIOC_DEVMODE, SSLINK_EVT_DEVMODE */

begin_packed_struct struct sslink_devmode_s
{
  uint8_t  enable;
  uint16_t duration_s;        /* capped by CONFIG_SSLINK_DEVMODE_MAX_S */
  uint32_t report_interval_ms;
  uint8_t  flags;             /* SSLINK_DEVMODE_* */
  uint8_t  profile;           /* enum sslink_profile_e */
} end_packed_struct;

/* OTA: the manifest is signed (ECDSA P-256 over all fields before sig) */

begin_packed_struct struct sslink_ota_manifest_s
{
  uint32_t image_size;
  uint8_t  fw_version[4];
  uint16_t block_size;
  uint8_t  sha256[32];
  uint8_t  sig[64];           /* raw r || s */
} end_packed_struct;

begin_packed_struct struct sslink_ota_begin_s
{
  uint8_t  session;
  struct sslink_ota_manifest_s manifest;
} end_packed_struct;

begin_packed_struct struct sslink_ota_ready_s
{
  uint8_t  session;
  uint8_t  status;            /* 0: ready, else refusal reason */
  uint16_t resume_from;       /* first block the node still needs */
} end_packed_struct;

begin_packed_struct struct sslink_ota_block_s
{
  uint8_t  session;
  uint16_t index;
  /* followed by up to block_size bytes */
} end_packed_struct;

begin_packed_struct struct sslink_ota_ack_s
{
  uint8_t  session;
  uint16_t base;
  uint32_t bitmap;            /* bit n: block base+n received */
} end_packed_struct;

begin_packed_struct struct sslink_ota_done_s
{
  uint8_t  session;
  uint8_t  status;            /* 0: verified, else error */
} end_packed_struct;

begin_packed_struct struct sslink_ota_abort_s
{
  uint8_t  session;
  uint8_t  reason;
} end_packed_struct;

/* Character driver structs *************************************************/

/* read() record.  read() returns SSLINK_EVENT_SIZE(len); the buffer must
 * hold at least sizeof(struct sslink_event_s).
 */

struct sslink_event_s
{
  uint8_t  type;              /* enum sslink_evt_e */
  uint8_t  msg_type;          /* SSLINK_EVT_MSG only */
  uint16_t len;               /* bytes valid in data[] */
  uint32_t dev_addr;          /* peer (gateway: the node; node: own addr) */
  uint32_t fcnt;
  uint32_t timestamp_ms;      /* clock_systime_ticks() converted to ms */
  int16_t  rssi;
  int8_t   snr;
  uint8_t  reserved;
  uint8_t  data[SSLINK_MAX_PAYLOAD];
};

/* write() record.  Pass SSLINK_TX_SIZE(len) as the write length. */

struct sslink_tx_s
{
  uint32_t dev_addr;          /* gateway: destination; node: ignored */
  uint8_t  msg_type;
  uint8_t  flags;             /* SSLINK_TXF_* */
  uint16_t len;
  uint8_t  data[SSLINK_MAX_PAYLOAD];
};

/* SSLINK_EVT_TX_DONE */

struct sslink_txdone_s
{
  int16_t  status;            /* 0 or negated errno */
  uint8_t  acked;
  uint8_t  retries;
  int8_t   tx_power;
  int8_t   peer_snr;          /* SNR the gateway measured */
  int16_t  peer_rssi;
};

/* SSLINK_EVT_PAIRED / SSLINK_EVT_PAIR_REJECTED */

struct sslink_paired_s
{
  uint8_t  eui[SSLINK_EUI_LEN];
  uint8_t  dev_type;
  uint8_t  reason;            /* rejected only */
};

/* An EUI passed by pointer (SSLINKIOC_CAND_CONFIRM / CAND_REJECT) */

struct sslink_eui_s
{
  uint8_t  eui[SSLINK_EUI_LEN];
};

/* SSLINK_EVT_CANDIDATE and SSLINKIOC_CAND_LIST */

struct sslink_candidate_s
{
  uint8_t  eui[SSLINK_EUI_LEN];
  uint8_t  dev_type;          /* enum sslink_dev_type_e */
  uint8_t  reserved;
  int16_t  rssi;              /* of the latest JOIN_REQ */
  uint32_t first_seen_s;      /* seconds since first heard */
  uint32_t last_seen_s;       /* seconds since last heard */
};

struct sslink_candlist_s
{
  uint16_t max;               /* entries available in cands */
  uint16_t count;             /* out: entries written */
  FAR struct sslink_candidate_s *cands;
};

/* SSLINK_EVT_OTA */

struct sslink_ota_evt_s
{
  uint8_t  state;             /* enum sslink_ota_state_e */
  uint8_t  session;
  uint16_t blocks_done;
  uint16_t blocks_total;
  int16_t  error;
};

/* SSLINKIOC_SETFILTER.  Defaults: all events, all devices, all types. */

struct sslink_filter_s
{
  uint32_t evt_mask;          /* SSLINK_EVTMASK() bits */
  uint32_t dev_addr;          /* 0: any device */
  uint8_t  msg_min;           /* SSLINK_EVT_MSG accepted when */
  uint8_t  msg_max;           /* msg_min <= msg_type <= msg_max */
};

/* SSLINKIOC_GETINFO */

struct sslink_info_s
{
  uint8_t  role;              /* enum sslink_role_e */
  uint8_t  proto_version;
  uint8_t  paired;            /* node only */
  uint8_t  dev_type;
  uint8_t  eui[SSLINK_EUI_LEN];
  uint32_t dev_addr;          /* node: own address once paired */
  uint16_t net_id;
  uint16_t max_payload;
  uint8_t  radio_caps;
};

/* SSLINKIOC_GETSTATS */

struct sslink_stats_s
{
  uint32_t tx_frames;
  uint32_t tx_acked;
  uint32_t tx_failed;
  uint32_t tx_retries;
  uint32_t tx_lbt_busy;
  uint32_t rx_frames;
  uint32_t rx_foreign;        /* wrong net_id or unknown device */
  uint32_t rx_malformed;
  uint32_t rx_mic_fail;
  uint32_t rx_replay;
  uint32_t rx_duplicate;
  uint32_t evt_dropped;
};

/* SSLINKIOC_STORE_EXPORT / IMPORT (CONFIG_SSLINK_STORE_APP) */

struct sslink_store_blob_s
{
  FAR uint8_t *buf;
  size_t len;                 /* in: buffer size, out: bytes exported */
};

/* SSLINKIOC_SHOULD_SEND */

struct sslink_sample_s
{
  uint16_t level_centipct;
  uint32_t err_flags;
  uint32_t warn_flags;
};

/* SSLINKIOC_DEV_GET / SSLINKIOC_DEV_LIST */

struct sslink_devinfo_s
{
  uint32_t dev_addr;
  uint8_t  eui[SSLINK_EUI_LEN];
  uint8_t  dev_type;
  uint8_t  online;
  uint8_t  dl_pending;        /* queued downlinks */
  uint8_t  ota_state;         /* enum sslink_ota_state_e */
  int16_t  last_rssi;
  int8_t   last_snr;
  uint32_t last_seen_s;       /* seconds since the last frame */
  uint32_t fcnt_up;
  uint32_t fcnt_down;
  uint32_t rx_count;
  uint32_t lost_count;        /* from frame counter gaps */
  uint16_t report_interval_s;
};

struct sslink_devlist_s
{
  uint16_t max;               /* entries available in devs */
  uint16_t count;             /* out: entries written */
  FAR struct sslink_devinfo_s *devs;
};

struct sslink_dev_policy_s
{
  uint32_t dev_addr;
  struct sslink_policy_s policy;
};

struct sslink_dev_devmode_s
{
  uint32_t dev_addr;
  struct sslink_devmode_s devmode;
};

struct sslink_ota_start_s
{
  uint32_t dev_addr;
  char     path[SSLINK_OTA_PATH_MAX];  /* image file on the gateway */
  struct sslink_ota_manifest_s manifest;
};

struct sslink_ota_status_s
{
  uint32_t dev_addr;          /* in */
  struct sslink_ota_evt_s status;
};

/* Board glue passed to sslink_register() */

struct sslink_board_s
{
  uint8_t  eui[SSLINK_EUI_LEN];  /* unique id, e.g. from the eFuse MAC */
  uint8_t  dev_type;             /* enum sslink_dev_type_e */
  uint8_t  wake_cause;           /* enum sslink_wake_e */
};

enum sslink_wake_e
{
  SSLINK_WAKE_COLD = 0,       /* power-on or reset */
  SSLINK_WAKE_TIMER,          /* deep sleep timer */
  SSLINK_WAKE_RADIO,          /* DIO1 during deep sleep: packet waiting */
  SSLINK_WAKE_OTHER,          /* button, sensor interrupt, ... */
};

/* Wire struct sizes are part of the protocol */

static_assert(sizeof(struct sslink_ack_s) == 5, "wire size");
static_assert(sizeof(struct sslink_tank_level_s) == 24, "wire size");
static_assert(sizeof(struct sslink_device_info_s) == 26, "wire size");
static_assert(sizeof(struct sslink_diag_s) == 22, "wire size");
static_assert(sizeof(struct sslink_policy_s) == 21, "wire size");
static_assert(sizeof(struct sslink_radio_cfg_s) == 3, "wire size");
static_assert(sizeof(struct sslink_wake_policy_s) == 5, "wire size");
static_assert(sizeof(struct sslink_devmode_s) == 9, "wire size");
static_assert(sizeof(struct sslink_ota_manifest_s) == 106, "wire size");
static_assert(sizeof(struct sslink_ota_begin_s) == 107, "wire size");
static_assert(sizeof(struct sslink_ota_ack_s) == 7, "wire size");

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
 * Name: sslink_register
 *
 * Description:
 *   Register the SSLink character driver.  Call from board bringup after
 *   the radio driver named by radiopath has been registered.
 *
 * Input Parameters:
 *   devpath   - Device node to create, e.g. "/dev/sslink0"
 *   radiopath - LoRa radio device node, e.g. "/dev/sx126x"
 *   board     - Board identity and wake cause (copied)
 *
 * Returned Value:
 *   Zero on success; a negated errno value on failure.
 *
 ****************************************************************************/

int sslink_register(FAR const char *devpath, FAR const char *radiopath,
                    FAR const struct sslink_board_s *board);

#undef EXTERN
#ifdef __cplusplus
}
#endif

#endif /* __INCLUDE_NUTTX_WIRELESS_LPWAN_SSLINK_H */
