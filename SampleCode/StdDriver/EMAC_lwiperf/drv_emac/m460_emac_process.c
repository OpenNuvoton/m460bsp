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
#include "lwip/netif.h"

extern uint32_t queue_try_put(struct pbuf *p);


/*----------------------------------------------------------------------------
  EMAC IRQ Handler
 *----------------------------------------------------------------------------*/
void EMAC0_IRQHandler(void)
{
    struct sk_buff *rskb = &rxbuf[0];

    if(EMAC_ReceivePkt(rskb) != 0)
    {
        /* Allocate pbuf from pool (avoid using heap in interrupts) */
        struct pbuf* p = pbuf_alloc(PBUF_RAW, rskb->len, PBUF_POOL);

        if(p != NULL)
        {
            /* Copy ethernet frame into pbuf */
            pbuf_take(p, rskb->data, rskb->len);

            /* Put in a queue which is processed in main loop */
            if(!queue_try_put(p))
            {
                /* queue is full -> packet loss */
                //printf("drop\n");
                pbuf_free(p);
            } //else
            //printf("get\n");
        }
    }
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
