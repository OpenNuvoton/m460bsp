/**************************************************************************//**
 * @file     m460_emac.h
 * @version  V3.00
 * @brief    M460 EMAC driver header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef  __M460_EMAC_H__
#define  __M460_EMAC_H__

#include "NuMicro.h"
#include "synopGMAC_network_interface.h"


extern synopGMACdevice GMACdev[GMAC_CNT];
extern synopGMACdevice *g_gmacdev;
extern struct sk_buff txbuf[GMAC_CNT];
extern struct sk_buff rxbuf[GMAC_CNT];

void EMAC_ModuleInit(uint32_t intf);
void EMAC_Open(uint32_t intf, uint8_t *macaddr);
uint32_t EMAC_ReceivePkt(struct sk_buff *prskb);
int32_t  EMAC_TransmitPkt(struct sk_buff *ptskb, uint8_t *pbuf, uint32_t len);
         
#endif  /* __M460_EMAC_H__ */
