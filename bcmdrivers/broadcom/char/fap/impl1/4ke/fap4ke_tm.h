#ifndef __FAP4KE_TM_H_INCLUDED__
#define __FAP4KE_TM_H_INCLUDED__

/*

 Copyright (c) 2007 Broadcom Corporation
 All Rights Reserved

<:label-BRCM:2011:DUAL/GPL:standard

This program is free software; you can redistribute it and/or modify
it under the terms of the GNU General Public License, version 2, as published by
the Free Software Foundation (the "GPL").

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.


A copy of the GPL is available at http://www.broadcom.com/licenses/GPLv2.php, or by
writing to the Free Software Foundation, Inc., 59 Temple Place - Suite 330,
Boston, MA 02111-1307, USA.

:>
*/

/*
 *******************************************************************************
 *
 * File Name  : fap4ke_tm.h
 *
 * Description: This file contains the FAP Traffic Management global definitions.
 *
 *******************************************************************************
 */

#define CC_FAP4KE_TM

//#define CC_FAP4KE_TM_TIMER_DEBUG

#if defined(CC_FAP4KE_TM)

#if !defined(CC_FAP4KE_TIMER_HIRES)
#error "FAP Traffic Management: High Resolution Timer is required (CC_FAP4KE_TIMER_HIRES)"
#endif

#define FAP4KE_TM_TIMER_JIFFIES      2  /* 500 usec */

#define FAP4KE_TM_ETH_IFG            20 /* bytes */
#define FAP4KE_TM_ETH_CRC_LEN        4
#define FAP4KE_TM_ETH_OVERHEAD       (FAP4KE_TM_ETH_CRC_LEN + FAP4KE_TM_ETH_IFG) 

#define FAP4KE_TM_BUCKET_SIZE_RATIO  4

/* Ethernet packet + 2 VLAN Tags + PPPoE + Overhead */
#define FAP4KE_TM_BUCKET_SIZE_MIN    (1514 + 8 + 8 + FAP4KE_TM_ETH_OVERHEAD)

#define FAP4KE_TM_PRIORITY_MAX       4
#define FAP4KE_TM_QUEUE_MAX          3
#define FAP4KE_TM_QUEUE_LOCAL_DEPTH  4 /* entries in local memory */

#define FAP4KE_TM_QUEUE_SDRAM_DEPTH  1200 /* entries in SDRAM memory */

#if (defined(CONFIG_BCM963268) || defined(CONFIG_BCM96362))
#define FAP4KE_TM_SCHEDULER_MAX      6
#define FAP4KE_TM_SCHEDULER_MASK     0x3F
#elif defined(CONFIG_BCM96828)
#define FAP4KE_TM_SCHEDULER_MAX      7
#define FAP4KE_TM_SCHEDULER_MASK     0x7F
#else
#error "FAP Traffic Manager: Unsupported chip"
#endif

#if (FAP4KE_PKT_MAX_DEST_PORTS > 8)
#error "Only 8 schedulers are supported in portEnableMask"
#endif

#if defined(CONFIG_BCM96362)
#define p4keTmCtrl ( &p4keDspramGbl->tmCtrl )
#else
#define p4keTmCtrl ( &p4kePsmGbl->tmCtrl )
#endif
#define p4keTmStorage ( &p4kePsmGbl->tmStorage )

typedef union {
    struct {
        void *txdma; /* BcmPktDma_LocalEthTxDma */
        uint8 *pBuf;
        uint32 key;
        int param1;
        uint16 len;
        uint16 dmaStatus;
        uint8 bufSource;
    };
    uint32 u32[6];
} fap4keTm_queueEntry_t;

typedef struct {
    fap4keTm_queueEntry_t *entry_p;
    uint16 depth;
    uint16 count;
    uint16 write;
    uint16 read;
} fap4keTm_queueCtrl_t;

typedef struct {
    uint32 tokens;
    uint32 bucketSize;
    uint32 bucket;
} fap4keTm_shaper_t;

typedef enum {
    FAP4KE_TM_QUEUE_TYPE_LOCAL=0,
    FAP4KE_TM_QUEUE_TYPE_SDRAM,
    FAP4KE_TM_QUEUE_TYPE_TOTAL
} fap4keTm_queueType_t;

typedef struct {
    uint16 totalCount;
    uint16 inSdram;
    uint32 dropped;
    fap4keTm_queueCtrl_t ctrl[FAP4KE_TM_QUEUE_TYPE_TOTAL];
} fap4keTm_arbiterQueue_t;

typedef struct {
    fap4keTm_arbiterQueue_t queue[FAP4KE_TM_QUEUE_MAX];
} fap4keTm_arbiter_t;

typedef int(* fap4keTm_arbiterFunc_t)(fap4keTm_arbiter_t *arbiter_p);

typedef struct {
    fap4keTm_shaper_t shaper;
    fap4keTm_arbiterFunc_t arbiterFunc;
    fap4keTm_arbiter_t arbiter;
} fap4keTm_scheduler_t;

#if defined(CC_FAP4KE_TM_TIMER_DEBUG)
typedef struct {
    uint32 startTime;
    uint32 timerCount;
    int hwTimerCount;
    int error;
    uint32 overflow;
} fap4keTm_ctrlDebug_t;
#endif

typedef struct {
    fap4keTm_queueEntry_t queueMem[FAP4KE_TM_QUEUE_MAX][FAP4KE_TM_QUEUE_LOCAL_DEPTH];
} fap4keTm_storageScheduler_t;

typedef struct {
    fap4keTm_storageScheduler_t scheduler[FAP4KE_TM_SCHEDULER_MAX];
} fap4keTm_storage_t;

typedef struct {
    fap4keTm_queueEntry_t queueMem[FAP4KE_TM_QUEUE_MAX][FAP4KE_TM_QUEUE_SDRAM_DEPTH];
} fap4keTm_sdramScheduler_t;

typedef struct {
    fap4keTm_sdramScheduler_t scheduler[FAP4KE_TM_SCHEDULER_MAX];
} fap4keTm_sdram_t;

typedef struct {
    uint8 masterEnable;
    uint8 portEnableMask;
    uint8 portEnableMaskConfig;
    uint8 priorityToQueue[FAP4KE_TM_PRIORITY_MAX];
    uint8 portToScheduler[FAP4KE_PKT_MAX_DEST_PORTS];
#if defined(CC_FAP4KE_TM_TIMER_DEBUG)
    fap4keTm_ctrlDebug_t debug;
#endif
    fap4keTmr_timer_t shaperTimer;
    fap4keTm_scheduler_t scheduler[FAP4KE_TM_SCHEDULER_MAX];
    fap4keTm_sdram_t *tmSdram_p;
    uint8 dmaLength[FAP4KE_TM_QUEUE_LOCAL_DEPTH+1];
} fap4keTm_ctrl_t;

#if !defined(FAP_4KE)
static inline void fapTm_kbpsToTokens(int kbps, uint32 *tokens_p, uint32 *bucketSize_p)
{
    uint32 tokens;
    uint32 bucketSize;

    tokens = ((1000/8) * kbps) / (FAPTMR_HZ_HIRES / FAP4KE_TM_TIMER_JIFFIES);

    bucketSize = (tokens * FAP4KE_TM_BUCKET_SIZE_RATIO);

    if(bucketSize < FAP4KE_TM_BUCKET_SIZE_MIN)
    {
        bucketSize = FAP4KE_TM_BUCKET_SIZE_MIN;
    }

    *tokens_p = tokens;
    *bucketSize_p = bucketSize;
}
#endif

void fap4keTm_enable(uint8 masterEnable);
void fap4keTm_config(uint8 port, uint8 enable,
                     uint16 tokens, uint32 bucketSize);
void fap4keTm_stats(uint8 port);
void fap4keTm_init(void);

#endif /* CC_FAP4KE_TM */

#endif  /* defined(__FAP4KE_TM_H_INCLUDED__) */
