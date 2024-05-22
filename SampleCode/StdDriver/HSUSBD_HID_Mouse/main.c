/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Simulate an USB mouse and draws circle on the screen.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "hid_mouse.h"

static uint8_t volatile s_u8RemouteWakeup = 0;

/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{
    uint32_t volatile i;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC and HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HIRC and HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_HXTSTB_Msk);

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 200MHz */
    CLK_SetCoreClock(FREQ_200MHZ);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;
    CLK->AHBCLK1 |= CLK_AHBCLK1_GPICKEN_Msk | CLK_AHBCLK1_GPJCKEN_Msk;

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Select HSUSBD */
    SYS->USBPHY &= ~SYS_USBPHY_HSUSBROLE_Msk;

    /* Enable USB PHY */
    SYS->USBPHY = (SYS->USBPHY & ~(SYS_USBPHY_HSUSBROLE_Msk | SYS_USBPHY_HSUSBACT_Msk)) | SYS_USBPHY_HSUSBEN_Msk;
    for(i = 0; i < 0x1000; i++);   // delay > 10 us
    SYS->USBPHY |= SYS_USBPHY_HSUSBACT_Msk;

    /* Enable HSUSBD module clock */
    CLK_EnableModuleClock(HSUSBD_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Lock protected registers */
    SYS_LockReg();
}

void GPIO_Init(void)
{
    /* Enable PH0 interrupt for wakeup */

    PH->MODE = 0x3; /* PH0 be Quasi mode */
    PH->INTSRC |= 0x1;
    PH->INTEN |= 0x1 | (0x1 << 16);
    PH->DBEN |= 0x1;      // Enable key debounce
    PH->DBCTL = 0x16; // Debounce time is about 6ms
    NVIC_EnableIRQ(GPH_IRQn);
}

void GPH_IRQHandler(void)
{
    PH->INTSRC = 0x1;
    s_u8RemouteWakeup = 1;
}

void PowerDown(void)
{
    uint32_t u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Wakeup Enable */
    HSUSBD->PHYCTL |= HSUSBD_PHYCTL_VBUSWKEN_Msk | HSUSBD_PHYCTL_LINESTATEWKEN_Msk;

    CLK_PowerDown();

    g_u8Suspend = 0;
    HSUSBD_ENABLE_USB();
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!(HSUSBD->PHYCTL & HSUSBD_PHYCTL_PHYCLKSTB_Msk))
        if(--u32TimeOutCnt == 0) break;

    /* Clear PWR_DOWN_EN if it is not clear by itself */
    if(CLK->PWRCTL & CLK_PWRCTL_PDEN_Msk)
        CLK->PWRCTL ^= CLK_PWRCTL_PDEN_Msk;

    /* Note HOST to resume USB tree if it is suspended and remote wakeup enabled */
    if(g_hsusbd_RemoteWakeupEn && s_u8RemouteWakeup)
    {
        /* Generate resume */
        HSUSBD->OPER |= HSUSBD_OPER_RESUMEEN_Msk;
        s_u8RemouteWakeup = 0;
    }

    /* Lock protected registers */
    SYS_LockReg();
}

int main(void)
{
    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    GPIO_Init();

    printf("NuMicro HSUSBD HID\n");

    HSUSBD_Open(&gsHSInfo, HID_ClassRequest, NULL);

    /* Endpoint configuration */
    HID_Init();

    /* Enable HSUSBD interrupt */
    NVIC_EnableIRQ(USBD20_IRQn);

    /* Start transaction */
    HSUSBD_Start();

    while(1)
    {
        /* Enter power down when USB suspend */
        if(g_u8Suspend)
        {
            PowerDown();

            /* Waiting for key release */
            while((GPIO_GET_IN_DATA(PH) & 0x1) != 0x1);
        }

        /* Move mouse when Key pressed */
        if((GPIO_GET_IN_DATA(PH) & 0x1) == 0x0)
            HID_UpdateMouseData();
    }
}
