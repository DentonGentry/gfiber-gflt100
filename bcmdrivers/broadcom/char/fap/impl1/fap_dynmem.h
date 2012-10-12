/*
<:copyright-BRCM:2012:proprietary:standard

   Copyright (c) 2012 Broadcom Corporation
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
 * File Name  : fap_dynmem.h
 *
 * Description: This file contains the base interface into the FAP dynamic
 *              memory.
 *******************************************************************************
 */

#ifndef _FAP_DYNMEM_H_INCLUDED_
#define _FAP_DYNMEM_H_INCLUDED_

#ifndef DYN_MEM_TEST_APP
#include "fap_hw.h"
#endif


typedef enum    {
    FAP_DM_REGION_DSP = 0,
    FAP_DM_REGION_PSM,
    FAP_DM_REGION_QSM,
    FAP_DM_REGION_HP,       // high priority - may be in QSM or PSM
    FAP_DM_REGION_MAX        
} fapDm_RegionIdx;

typedef enum    {
    FAP_DM_REGION_ORDER_DSP_PSM_QSM = 0,
    FAP_DM_REGION_ORDER_DSP_PSM_QSM_HP,
    FAP_DM_REGION_ORDER_QSM_PSM,
    FAP_DM_REGION_ORDER_QSM_PSM_HP,
    FAP_DM_REGION_ORDER_QSM,
    FAP_DM_REGION_ORDER_MAX
} fapDm_RegionOrder;

typedef uint32 fapDm_BlockId;

typedef union {
    struct  {
        unsigned              regionIdx : 3;
        unsigned              blockIdx  : 13;
        unsigned              offset : 16;
    };
    fapDm_BlockId           id;
} fapDm_BlockInfo ;


/* Note: FAP_DM_INVALID_BLOCK_ID may not be 0 (as 0 is valid), and must be the same byte repeated four times, due to
   the use of memset to make all blocks invalid by default */

#define FAP_DM_INVALID_BLOCK_ID        ((fapDm_BlockId)0xFFFFFFFF)

#define FAP_DM_RSVD_HP_FLOW_CNT     12
/* The following assumes that only flow info is stored in the hp flow types -- not command lists.
   This is true for multicast.  If we want to do other types of high priority flows, this will
   need to be adjusted: */
#define FAP_DM_HP_SIZE              (FAP_DM_RSVD_HP_FLOW_CNT*sizeof(fap4kePkt_flow_t))

#endif
