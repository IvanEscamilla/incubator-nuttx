/****************************************************************************
 * arch/xtensa/src/esp32s3/esp32s3_perf.c
 * SPDX-License-Identifier: Apache-2.0
 ****************************************************************************/

#include <nuttx/config.h>

#ifdef CONFIG_ESP32S3_PERF_SYSTIMER
#include <nuttx/arch.h>
#include <nuttx/clock.h>
#include <nuttx/irq.h>
#include <nuttx/spinlock.h>

#include "xtensa.h"
#include "hardware/esp32s3_systimer.h"

#ifdef CONFIG_PERF_OVERFLOW_CORRECTION
#  error "PERF_OVERFLOW_CORRECTION must be disabled with ESP32S3_PERF_SYSTIMER"
#endif

#define ESP32S3_PERF_HZ 16000000ul

static spinlock_t g_perf_lock;
static clock_t g_perf_last;

void up_perf_init(void *arg)
{
  (void)arg;
}

unsigned long up_perf_getfreq(void)
{
  return ESP32S3_PERF_HZ;
}

clock_t up_perf_gettime(void)
{
  irqstate_t flags;
  uint32_t hi1;
  uint32_t lo;
  int spins = 0;

  /* UPDATE latches the 52-bit value into the HI/LO registers.  The raw
   * IRQ-safe lock serializes the handshake across cores (one core's UPDATE
   * must not re-latch while the other reads) without scheduler calls or
   * notes.  The wait is bounded: a read before the SYSTIMER clock runs
   * (early boot) returns the last value instead of hanging.
   */

  flags = spin_lock_irqsave(&g_perf_lock);
  putreg32(SYSTIMER_TIMER_UNIT0_UPDATE, SYSTIMER_UNIT0_OP_REG);
  while ((getreg32(SYSTIMER_UNIT0_OP_REG) &
          SYSTIMER_TIMER_UNIT0_VALUE_VALID) == 0)
    {
      if (++spins > 1000)
        {
          clock_t last = g_perf_last;
          spin_unlock_irqrestore(&g_perf_lock, flags);
          return last;
        }
    }

  hi1 = getreg32(SYSTIMER_UNIT0_VALUE_HI_REG) & SYSTIMER_TIMER_UNIT0_VALUE_HI;
  lo = getreg32(SYSTIMER_UNIT0_VALUE_LO_REG);
  g_perf_last = ((clock_t)hi1 << 32) | lo;
  spin_unlock_irqrestore(&g_perf_lock, flags);
  return ((clock_t)hi1 << 32) | lo;
}

void up_perf_convert(clock_t elapsed, struct timespec *ts)
{
  ts->tv_sec = elapsed / ESP32S3_PERF_HZ;
  ts->tv_nsec = NSEC_PER_SEC * (elapsed % ESP32S3_PERF_HZ) / ESP32S3_PERF_HZ;
}
#endif
