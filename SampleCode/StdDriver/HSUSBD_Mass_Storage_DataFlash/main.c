/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Use Data Flash as back end storage media to simulate an USB pen drive.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "massstorage.h"

#define DATA_FLASH_BASE  0x00040000

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

int main(void)
{
    uint32_t au32Config[2];

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("NuMicro HSUSBD Mass Storage\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable FMC ISP function */
    FMC_Open();

    /* Enable Read/Write flash function */
    FMC_ENABLE_AP_UPDATE();

    /* Check if Data Flash Size is 64K. If not, to re-define Data Flash size and to enable Data Flash function */
    if(FMC_ReadConfig(au32Config, 2) < 0)
        return -1;

    if(((au32Config[0] & 0x01) == 1) || (au32Config[1] != DATA_FLASH_BASE))
    {
        FMC_ENABLE_CFG_UPDATE();
        au32Config[0] &= ~0x1;
        au32Config[1] = DATA_FLASH_BASE;
        if(FMC_WriteConfig(au32Config, 2) < 0)
            return -1;

        FMC_ReadConfig(au32Config, 2);
        if(((au32Config[0] & 0x01) == 1) || (au32Config[1] != DATA_FLASH_BASE))
        {
            printf("Error: Program Config Failed!\n");

            /* Disable FMC ISP function */
            FMC_Close();
            SYS_LockReg();
            return -1;
        }

        printf("chip reset\n");

        /* Reset Chip to reload new CONFIG value */
        SYS->IPRST0 = SYS_IPRST0_CHIPRST_Msk;
    }

    HSUSBD_Open(&gsHSInfo, MSC_ClassRequest, NULL);

    /* Endpoint configuration */
    MSC_Init();

    /* Enable HSUSBD interrupt */
    NVIC_EnableIRQ(USBD20_IRQn);

    /* Start transaction */
    while(1)
    {
        if(HSUSBD_IS_ATTACHED())
        {
            HSUSBD_Start();
            break;
        }
    }

    while(1)
    {
        if(g_hsusbd_Configured)
            MSC_ProcessCmd();
    }
}
