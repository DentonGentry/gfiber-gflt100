#ifndef __FAPIRQ_H_INCLUDED__
#define __FAPIRQ_H_INCLUDED__

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
 * File Name  : fap4ke_irq.h
 *
 * Description: This file contains ...
 *
 *******************************************************************************
 */

#define FAP4KE_IRQ_HANDLERS_MAX 8

#define fap4keIrq_enable(irq)    _fap4keIrq_enable(irq)

#define fap4keIrq_disable(irq)   _fap4keIrq_disable(irq)

#define fap4keIrq_register(_handler, _arg, irq) \
    __fap4keIrq_register(_handler, _arg, irq, #_handler)

typedef enum {                     /* Usage (see fap_hw.h for register definitions): */
    FAP4KE_IRQ_GROUP_FAP,          /* For interrupts in the irq_4ke_status register */
    FAP4KE_IRQ_GROUP_CHIP_EXTRA2,  /* For interrupts in the extra2ChipIrqStatus register */
    FAP4KE_IRQ_GROUP_CHIP_EXTRA,   /* For interrupts in the extraChipIrqStatus register 
                                     (or fap1IrqMaskLo register for 963268) */
    FAP4KE_IRQ_GROUP_CHIP,         /* For interrupts in the chipIrqStatus register */
    FAP4KE_IRQ_GROUP_MAX
} fap4keIrq_irqGroup_t;

typedef enum {
    FAP4KE_IRQ_ENET_RX_0 = 0,
    FAP4KE_IRQ_ENET_RX_1,
    FAP4KE_IRQ_ENET_RX_2,
    FAP4KE_IRQ_ENET_RX_3,
    FAP4KE_IRQ_ENET_RX_ALL,
    FAP4KE_IRQ_DQM,
    FAP4KE_IRQ_SAR_DMA_0,
    FAP4KE_IRQ_SAR_DMA_1,
    FAP4KE_IRQ_SAR_DMA_2,
    FAP4KE_IRQ_SAR_DMA_3,
    FAP4KE_IRQ_SAR_ALL,
    FAP4KE_IRQ_GENERAL_PURPOSE_INPUT,
    FAP4KE_IRQ_TIMER_0,
    FAP4KE_IRQ_TIMER_1,
} fap4keIrqs;


typedef fapRet(*fap4keIrq_handler_t)(uint32 intStatus, uint32 arg);

typedef struct
{
    fap4keIrq_handler_t handler;
    uint32 arg;
    fap4keIrq_irqGroup_t irqGroup;
    volatile uint32 *irqAddr;
    uint32 irqMask;
    const char *name;
    uint32 count;
} fap4keIrq_handlerInfo_t;


void fap4keIrq_init(void);

fapRet __fap4keIrq_register(    fap4keIrq_handler_t handler, 
                                uint32 arg, 
                                fap4keIrqs irq, 
                                const char *name);

void _fap4keIrq_enable(fap4keIrqs irq);

void _fap4keIrq_disable(fap4keIrqs irq);

void fap4keIrq_mainHandler(void);

void printIrqStats(void);

#endif  /* defined(__FAPIRQ_H_INCLUDED__) */
