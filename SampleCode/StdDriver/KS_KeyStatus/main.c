/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demo to use the AES in Key Store.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock */
    CLK_SetCoreClock(FREQ_200MHZ);

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable CRYPTO module clock */
    CLK_EnableModuleClock(CRPT_MODULE);

    /* Enable Key Store module clock */
    CLK_EnableModuleClock(KS_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(DEBUG_PORT, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    int32_t i, cnt;
    
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    KS_Open();

    printf("KS SRAM remain size      : %d\n", KS_GetRemainSize(KS_SRAM));
    printf("KS SRAM remain key count : %d\n", KS_GetRemainKeyCount(KS_SRAM));
    printf("KS Flash remain size     : %d\n", KS_GetRemainSize(KS_FLASH));
    printf("KS Flash remain key count: %d\n", KS_GetRemainKeyCount(KS_FLASH));
    printf("KS OTP Keys:\n");
    for(i=0;i<8;i++)
    {
        printf("OTP%d  ", i);
    }
    printf("\n");
    
    for(cnt = 0, i = 0;i < 8;i++)
    {
        printf("  %c   ", ((KS->OTPSTS&(1 << i)) == 0)?'x':'v');
        if(KS->OTPSTS & (1 << i))
        {
            cnt++;
        }
    }
    printf("\n------------------------------------------------\n");

    printf("Total %d OTP Keys in Key Store\n", cnt);
    
    if(KS->STS & KS_STS_KRVKF_Msk)
    {
        printf("WARN: OTP key revoke flag set. The MCU is in RMA stage.\n");
        printf("      Boot function has been disabled.\n");
    }
    
    while(1) {}
}
