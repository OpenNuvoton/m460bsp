/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    This Ethernet sample tends to get a DHCP lease from DHCP server. 
 *           After IP address configured, this sample can reply to PING packets.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#include "net.h"
#include "synopGMAC_network_interface.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void EMAC0_IRQHandler(void);
void EMAC_SendPkt(uint8_t *pu8Data, uint32_t u32Size);
void SelectEMACPins(uint32_t group);
void SYS_Init(void);
void UART_Init(void);


/*---------------------------------------------------------------------------------------------------------*/
/* Global Interface Variables Declarations                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_au8MacAddr[6] = DEFAULT_MAC0_ADDRESS;
uint8_t volatile g_au8IpAddr[4] = {0, 0, 0, 0};
extern synopGMACdevice GMACdev[GMAC_CNT];
extern struct sk_buff txbuf[GMAC_CNT];
extern struct sk_buff rxbuf[GMAC_CNT];
extern void synopGMAC0_intr_handler(void);

volatile uint32_t gu32SysTickCnts = 0;          // Counter for SysTick_Handler


/*----------------------------------------------------------------------------
  SysTick IRQ Handler
 *----------------------------------------------------------------------------*/
void SysTick_Handler(void)
{
	gu32SysTickCnts++;                          // Increment Counter
}
    
/* Delay execution for given amount of ticks */
void plat_delay(uint32_t ticks)
{
	uint32_t tgtTicks = gu32SysTickCnts + ticks;    // target tick count to delay execution to
	while (gu32SysTickCnts == tgtTicks) {}
}


/*----------------------------------------------------------------------------
  EMAC IRQ Handler
 *----------------------------------------------------------------------------*/
void EMAC0_IRQHandler(void)
{
    struct sk_buff *rskb = &rxbuf[0];
    
    rskb->rdy = 0;
    
    synopGMAC0_intr_handler();
    
    if(rskb->rdy != 0)
        process_rx_packet((uint8_t *)((u64)(rskb->data)), rskb->len);
}

/**
  * @brief      Send an Ethernet packet
  * @param[in]  pu8Data     Pointer to a buffer holds the packet to transmit
  * @param[in]  u32Size     Packet size (without 4 byte CRC)
  * @return     None    
  */
void EMAC_SendPkt(uint8_t *pu8Data, uint32_t u32Size)
{
    struct sk_buff *tskb = &txbuf[0];

    tskb->len = u32Size;
    memcpy((uint8_t *)((u64)(tskb->data)), pu8Data, u32Size);
    synopGMAC_xmit_frames(tskb, 0, 0, 0);
}


void SelectEMACPins(uint32_t group)
{
    if(group == 0)
    {
        SET_EMAC0_RMII_MDC_PE8();
        SET_EMAC0_RMII_MDIO_PE9();
        SET_EMAC0_RMII_TXD0_PE10();
        SET_EMAC0_RMII_TXD1_PE11();
        SET_EMAC0_RMII_TXEN_PE12();    
        SET_EMAC0_RMII_REFCLK_PC8();
        SET_EMAC0_RMII_RXD0_PC7();
        SET_EMAC0_RMII_RXD1_PC6();
        SET_EMAC0_RMII_CRSDV_PA7();
        SET_EMAC0_RMII_RXERR_PA6();
        
        SET_EMAC0_PPS_PB6();
    }
    else
    {
        SET_EMAC0_RMII_MDC_PB11();
        SET_EMAC0_RMII_MDIO_PB10();
        SET_EMAC0_RMII_TXD0_PB9();
        SET_EMAC0_RMII_TXD1_PB8();
        SET_EMAC0_RMII_TXEN_PB7();    
        SET_EMAC0_RMII_REFCLK_PB5();
        SET_EMAC0_RMII_RXD0_PB4();
        SET_EMAC0_RMII_RXD1_PB3();
        SET_EMAC0_RMII_CRSDV_PB2();
        SET_EMAC0_RMII_RXERR_PB1();
        
        SET_EMAC0_PPS_PE13();
    }
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 200MHz */
    CLK_SetCoreClock(200000000);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;
    CLK->AHBCLK1 |= CLK_AHBCLK1_GPICKEN_Msk | CLK_AHBCLK1_GPJCKEN_Msk;

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));  
    
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Initialization for sample code                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable EMAC module clock */
    CLK_EnableModuleClock(EMAC0_MODULE);
    
    /* Select multi-function pins for EMAC */
    SelectEMACPins(0);
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(DEBUG_PORT, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    synopGMACdevice * gmacdev = &GMACdev[0];
    
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    SysTick_Config(SystemCoreClock / 1000);
    
    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+------------------------------+\n");
    printf("|    EMAC Tx/Rx Sample Code    |\n");
    printf("+------------------------------+\n\n");
        
    synopGMAC_open(0);
    synopGMAC_promisc_enable(gmacdev);
    synopGMAC_set_mode(0, 1);

    NVIC_EnableIRQ(EMAC0_TXRX_IRQn);

    if (dhcp_start() < 0)
    {
        // Cannot get a DHCP lease
        printf("\nDHCP failed......\n");
    }
       
    while(1) {}
}
