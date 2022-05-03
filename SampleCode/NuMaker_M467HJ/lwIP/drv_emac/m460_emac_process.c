/**************************************************************************//**
 * @file     m460_emac_process.c
 * @version  V3.00
 * @brief    M460 EMAC process data source file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "string.h"
#include "m460_emac.h"
#include "arch/sys_arch.h"
#include "lwip/netif.h"


extern uint32_t queue_try_put(struct pbuf *p);
extern void ethernetif_input(u16_t len, u8_t *buf, u32_t s, u32_t ns);
extern portBASE_TYPE xInsideISR;
/*----------------------------------------------------------------------------
  EMAC IRQ Handler
 *----------------------------------------------------------------------------*/
void EMAC0_IRQHandler(void)
{
    struct sk_buff *rskb = &rxbuf[0];

    xInsideISR = pdTRUE;

    if(EMAC_ReceivePkt(rskb) != 0)
    {
        ethernetif_input(rskb->len, rskb->data, 0, 0);
    }    
    
    xInsideISR = pdFALSE;
}

uint32_t EMAC_ReceivePkt(struct sk_buff *prskb)
{
    prskb->rdy = 0;
    
    synopGMAC0_intr_handler();
    
    return prskb->rdy;
}

int32_t EMAC_TransmitPkt(struct sk_buff *ptskb, uint8_t *pbuf, uint32_t len)
{
    struct sk_buff *tskb;

    if(ptskb == NULL)
    {
        tskb = &txbuf[0];

        tskb->len = len;
        memcpy((uint8_t *)((u64)(tskb->data)), pbuf, len);
        return synopGMAC_xmit_frames(tskb, 0, 0, 0);
    }
    else
    {
        ptskb->len = len;
        return synopGMAC_xmit_frames(ptskb, 0, 0, 0);
    }
}
