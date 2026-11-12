/****************************************************************************
 * include/nuttx/lcd/lcd_trace.h
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

#ifndef __INCLUDE_NUTTX_LCD_LCD_TRACE_H
#define __INCLUDE_NUTTX_LCD_LCD_TRACE_H

/* Named trace sections for LCD drivers.  They are scheduler notes with the
 * application tag, shown by "trace dump" as nested slices.  Names must be
 * static string literals.  They compile to nothing without
 * CONFIG_SCHED_INSTRUMENTATION_DUMP.
 */

#include <nuttx/config.h>

#ifdef CONFIG_SCHED_INSTRUMENTATION_DUMP
#  include <string.h>
#  include <nuttx/sched_note.h>
#  define LCD_TRACE_BEGIN(name) sched_note_beginex(NOTE_TAG_APP, name)
#  define LCD_TRACE_END()       sched_note_end(NOTE_TAG_APP)
#else
#  define LCD_TRACE_BEGIN(name) ((void)0)
#  define LCD_TRACE_END()       ((void)0)
#endif

#endif /* __INCLUDE_NUTTX_LCD_LCD_TRACE_H */
