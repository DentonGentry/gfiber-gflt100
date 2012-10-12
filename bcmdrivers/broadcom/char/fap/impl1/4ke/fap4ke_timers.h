#ifndef __FAPTMR_H_INCLUDED__
#define __FAPTMR_H_INCLUDED__

/*
<:copyright-BRCM:2009:proprietary:standard

   Copyright (c) 2009 Broadcom Corporation
   All Rights Reserved

 This program is the proprietary software of Broadcom Corporation and/or its
 licensors, and may only be used, duplicated, modified or distributed pursuant
 to the terms and conditions of a separate, written license agreement executed
 between you and Broadcom (an "Authorized License").  Except as set forth in
 an Authorized License, Broadcom grants no license (express or implied), right
 to use, or waiver of any kind with respect to the Software, and Broadcom
 expressly reserves all rights in and to the Software and all intellectual
 property rights therein.  IF YOU HAVE NO AUTHORIZED LICENSE, THEN YOU HAVE
 NO RIGHT TO USE THIS SOFTWARE IN ANY WAY, AND SHOULD IMMEDIATELY NOTIFY
 BROADCOM AND DISCONTINUE ALL USE OF THE SOFTWARE.

 Except as expressly set forth in the Authorized License,

 1. This program, including its structure, sequence and organization,
    constitutes the valuable trade secrets of Broadcom, and you shall use
    all reasonable efforts to protect the confidentiality thereof, and to
    use this information only in connection with your use of Broadcom
    integrated circuit products.

 2. TO THE MAXIMUM EXTENT PERMITTED BY LAW, THE SOFTWARE IS PROVIDED "AS IS"
    AND WITH ALL FAULTS AND BROADCOM MAKES NO PROMISES, REPRESENTATIONS OR
    WARRANTIES, EITHER EXPRESS, IMPLIED, STATUTORY, OR OTHERWISE, WITH
    RESPECT TO THE SOFTWARE.  BROADCOM SPECIFICALLY DISCLAIMS ANY AND
    ALL IMPLIED WARRANTIES OF TITLE, MERCHANTABILITY, NONINFRINGEMENT,
    FITNESS FOR A PARTICULAR PURPOSE, LACK OF VIRUSES, ACCURACY OR
    COMPLETENESS, QUIET ENJOYMENT, QUIET POSSESSION OR CORRESPONDENCE
    TO DESCRIPTION. YOU ASSUME THE ENTIRE RISK ARISING OUT OF USE OR
    PERFORMANCE OF THE SOFTWARE.

 3. TO THE MAXIMUM EXTENT PERMITTED BY LAW, IN NO EVENT SHALL BROADCOM OR
    ITS LICENSORS BE LIABLE FOR (i) CONSEQUENTIAL, INCIDENTAL, SPECIAL,
    INDIRECT, OR EXEMPLARY DAMAGES WHATSOEVER ARISING OUT OF OR IN ANY
    WAY RELATING TO YOUR USE OF OR INABILITY TO USE THE SOFTWARE EVEN
    IF BROADCOM HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES;
    OR (ii) ANY AMOUNT IN EXCESS OF THE AMOUNT ACTUALLY PAID FOR THE
    SOFTWARE ITSELF OR U.S. $1, WHICHEVER IS GREATER. THESE LIMITATIONS
    SHALL APPLY NOTWITHSTANDING ANY FAILURE OF ESSENTIAL PURPOSE OF ANY
    LIMITED REMEDY.
:>
*/

/*
 *******************************************************************************
 * File Name  : fapMips_timers.h
 *
 * Description: This file contains global definitions of the Timers
 *              implementation for the BCM6362 FAP.
 *
 *******************************************************************************
 */

#include "fap4ke_task.h"

#define CC_FAP4KE_TIMER_HIRES

/* System timer tick rate */
#define FAPTMR_HZ_LORES 10    /* 100ms */
#define FAPTMR_HZ_HIRES 4000  /* 250us */

#define FAP4KE_TIMER_JIFFIES_32(_jiffies64) ( (uint32)(_jiffies64) )

#define fap4keTmr_jiffiesLoRes FAP4KE_TIMER_JIFFIES_32(p4keDspramGbl->timers.ctrl.loRes.jiffies64)
#define fap4keTmr_jiffiesHiRes FAP4KE_TIMER_JIFFIES_32(p4keDspramGbl->timers.ctrl.hiRes.jiffies64)

#define FAP4KE_TIMER_INIT(_timer, _handler, _arg, _taskPriority)        \
    do {                                                                \
        (_timer)->taskPriority = (_taskPriority);                       \
        FAP4KE_TASK_INIT(&(_timer)->task, (_handler), (_arg));          \
    } while(0)

/* The following 4 macros are provided for comparing tick counts that correctly handle
   wraparound in the tick count. The _unknown parameter is typically fap4keTmr_Jiffies,
   and the _known parameter is the value against which you want to compare */

/* if _unknown is after _known, true; otherwise false */
#define fap4keTmr_isTimeAfter(_unknown, _known) ( (uint32)(_known) < (uint32)(_unknown) )

/* if _unknown is before _known, true; otherwise false */
#define fap4keTmr_isTimeBefore(_unknown, _known) fap4keTmr_isTimeAfter(_known, _unknown)

/* if _unknown is after than or equal to _known, true; otherwise false */
#define fap4keTmr_isTimeAfter_eq(_unknown, _known) ( (uint32)(_unknown) >= (uint32)(_known) )

/* if _unknown is before than or equal to _known, true; otherwise false */
#define fap4keTmr_isTimeBefore_eq(_unknown, _known) fap4keTmr_isTimeAfter_eq(_known, _unknown)

typedef struct {
    /* Timer Management */
    volatile int64 jiffies64;
    Dll_t list;
} fap4keTmr_CtrlInfo_t;

typedef struct {
    /* Timer Control Information */
    fap4keTmr_CtrlInfo_t loRes;
    fap4keTmr_CtrlInfo_t hiRes;
} fap4keTmr_Ctrl_t;

typedef struct {
    Dll_t node;            /* used internally to maintain linked-list of timers */
    uint32 expiration;     /* expiration time, in fap4keTmr_Jiffies */
    fap4keTsk_taskPriority_t taskPriority; /* timer task priority */
    fap4keTsk_task_t task; /* the task in which the timer handler will run */
} fap4keTmr_timer_t;

typedef enum {
    FAP4KE_TIMER_TYPE_LORES=0,
    FAP4KE_TIMER_TYPE_HIRES,
    FAP4KE_TIMER_TYPE_MAX
} fap4keTmr_type_t;

fapRet fap4keTmr_add(fap4keTmr_timer_t *timer, fap4keTmr_type_t type);
void fap4keTmr_Init(void);

#define FAP4KE_PM_CPU_HISTORY_MAX 8

typedef struct {
    uint32 cp0Count;
    uint32 busy;
    uint32 capture;
} fap4keTmr_cpuSample_t;

#define p4keCpuSample ( (&p4keDspramGbl->timers.cpu) )

typedef struct {
    uint32 index;
    uint32 busy[FAP4KE_PM_CPU_HISTORY_MAX];
} fap4keTmr_cpuHistory_t;

#define p4keCpuHistory ( (&p4kePsmGbl->timers.cpu) )

#define pHostCpuHistory(fapIdx) ( (&pHostPsmGbl(fapIdx)->timers.cpu) )

#endif  /* defined(__FAPTMR_H_INCLUDED__) */
