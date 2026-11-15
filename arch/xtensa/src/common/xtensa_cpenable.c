/****************************************************************************
 * arch/xtensa/src/common/xtensa_cpenable.c
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

#include <nuttx/spinlock.h>
#include <nuttx/sched.h>

#include <arch/xtensa/xtensa_coproc.h>
#include <arch/chip/core-isa.h>

#include <arch/irq.h>

#include "sched/sched.h"
#include "xtensa.h"

#if XCHAL_CP_NUM > 0

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_cpenable_slot
 *
 * Description:
 *   CPENABLE belongs to the thread: it is stored in the register frame at
 *   every context save (REG_CPENABLE) and written back at every restore.
 *
 *   While a thread runs in thread context it has no valid frame
 *   (xcp.regs is NULL), and the CPENABLE register is the truth: the next
 *   save stores it.  Nothing else has to be updated.
 *
 *   When the caller runs with a valid frame of the running thread (an
 *   interrupt or a system call), the restore on return would overwrite the
 *   register with the stale slot, so the slot is updated too.
 *
 *   Called with interrupts disabled, so the running thread cannot change.
 *
 ****************************************************************************/

static void xtensa_cpenable_slot(uint32_t cpenable)
{
  struct tcb_s *tcb = running_task();

  if (tcb != NULL && tcb->xcp.regs != NULL)
    {
      tcb->xcp.regs[REG_CPENABLE] = cpenable;
    }
}

/****************************************************************************
 * Private Data
 ****************************************************************************/

static spinlock_t g_lock = SP_UNLOCKED;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: xtensa_coproc_enable
 *
 * Description:
 *   Enable a set of co-processors.
 *
 * Input Parameters:
 *   cpset   - A bit set of co-processors to be enabled.  Matches bit layout
 *             of the CPENABLE register.  Bit 0-XCHAL_CP_NUM:  0 = no change
 *             1 = enable
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void xtensa_coproc_enable(int cpset)
{
  irqstate_t flags;
  uint32_t cpenable;

  /* These operations must be atomic */

  flags = spin_lock_irqsave(&g_lock);

  if (cpset != 0)
    {
      /* Enable the co-processors */

      cpenable = xtensa_get_cpenable();
      cpenable |= cpset;
      xtensa_set_cpenable(cpenable);
      xtensa_cpenable_slot(cpenable);
    }

  spin_unlock_irqrestore(&g_lock, flags);
}

/****************************************************************************
 * Name: xtensa_coproc_disable
 *
 * Description:
 *   Enable a set of co-processors.
 *
 * Input Parameters:
 *   cpset   - A bit set of co-processors to be enabled.  Matches bit layout
 *             of the CPENABLE register.  Bit 0-XCHAL_CP_NUM:  0 = no change
 *             1 = disable
 *
 * Returned Value:
 *   None
 *
 ****************************************************************************/

void xtensa_coproc_disable(int cpset)
{
  irqstate_t flags;
  uint32_t cpenable;

  /* These operations must be atomic */

  flags = spin_lock_irqsave(&g_lock);

  if (cpset != 0)
    {
      /* Disable the co-processors */

      cpenable = xtensa_get_cpenable();
      cpenable &= ~cpset;
      xtensa_set_cpenable(cpenable);
      xtensa_cpenable_slot(cpenable);
    }

  spin_unlock_irqrestore(&g_lock, flags);
}

#endif /* XCHAL_CP_NUM */
