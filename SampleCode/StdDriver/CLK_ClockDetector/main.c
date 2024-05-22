/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show the usage of clock fail detector and clock frequency monitor function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"



void CKFAIL_IRQHandler(void);
void SYS_Init(void);
void UART0_Init(void);

/*---------------------------------------------------------------------------------------------------------*/
/*  Clock Fail Detector IRQ Handler                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void CKFAIL_IRQHandler(void)
{
    uint32_t u32Reg;

    /* Unlock protected registers */
    SYS_UnlockReg();

    u32Reg = CLK->CLKDSTS;

    if(u32Reg & CLK_CLKDSTS_HXTFIF_Msk)
    {
        /* HCLK is switched to HIRC automatically if HXT clock fail interrupt is happened */
        printf("HXT Clock is stopped! HCLK is switched to HIRC.\n");

        /* Disable HXT clock fail interrupt */
        CLK->CLKDCTL &= ~(CLK_CLKDCTL_HXTFDEN_Msk | CLK_CLKDCTL_HXTFIEN_Msk);

        /* Write 1 to clear HXT Clock fail interrupt flag */
        CLK->CLKDSTS = CLK_CLKDSTS_HXTFIF_Msk;
    }

    if(u32Reg & CLK_CLKDSTS_LXTFIF_Msk)
    {
        /* LXT clock fail interrupt is happened */
        printf("LXT Clock is stopped!\n");

        /* Disable LXT clock fail interrupt */
        CLK->CLKDCTL &= ~(CLK_CLKDCTL_LXTFIEN_Msk | CLK_CLKDCTL_LXTFDEN_Msk);

        /* Write 1 to clear LXT Clock fail interrupt flag */
        CLK->CLKDSTS = CLK_CLKDSTS_LXTFIF_Msk;
    }

    if(u32Reg & CLK_CLKDSTS_HXTFQIF_Msk)
    {
        /* HCLK should be switched to HIRC if HXT clock frequency monitor interrupt is happened */
        CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));
        printf("HXT Frequency is abnormal! HCLK is switched to HIRC.\n");

        /* Disable HXT clock frequency monitor interrupt */
        CLK->CLKDCTL &= ~(CLK_CLKDCTL_HXTFQDEN_Msk | CLK_CLKDCTL_HXTFQIEN_Msk);

        /* Write 1 to clear HXT Clock frequency monitor interrupt */
        CLK->CLKDSTS = CLK_CLKDSTS_HXTFQIF_Msk;
    }

    /* Lock protected registers */
    SYS_LockReg();

}

void SYS_Init(void)
{

    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SET_XT1_OUT_PF2();
    SET_XT1_IN_PF3();

    /* Set PF multi-function pins for X32_OUT(PF.4) and X32_IN(PF.5) */
    SET_X32_OUT_PF4();
    SET_X32_IN_PF5();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC, HXT and LXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk | CLK_PWRCTL_LXTEN_Msk);

    /* Wait for HIRC, HXT and LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_HXTSTB_Msk | CLK_STATUS_LXTSTB_Msk);

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

    /* Set PC multi-function pin for CLKO(PC.13) */
    SET_CLKO_PC13();

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------------------------+\n");
    printf("|                  Clock Detector Sample Code                 |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("| 1. HXT clock fail interrupt will happen if HXT is stopped.  |\n");
    printf("|    HCLK clock source will be switched from HXT to HIRC.     |\n");
    printf("| 2. LXT clock fail interrupt will happen if LXT is stopped.  |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("\nStop HXT or LXT to test.\n\n");

    /* Enable clock output, select CLKO clock source as HCLK and set clock output frequency is HCLK/4.
       HCLK clock source will be switched to HIRC if HXT stop and HCLK clock source is from HXT.
       You can check if HCLK clock source is switched to HIRC by clock output pin output frequency.
    */

    /* Output selected clock to CKO, CKO Clock = HCLK / 2^(1 + 1) */
    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_HCLK, 1, 0);

    /* Set the HXT clock frequency monitor upper and lower boundary value.
       The upper boundary value should be more than 512*(HXT/HIRC).
       The low boundary value should be less than 512*(HXT/HIRC).
    */
    CLK->CDUPB = 516;
    CLK->CDLOWB = 508;

    /* Set clock fail detector function enabled and interrupt enabled */
    CLK->CLKDCTL = CLK_CLKDCTL_HXTFDEN_Msk |
                   CLK_CLKDCTL_HXTFIEN_Msk |
                   CLK_CLKDCTL_LXTFDEN_Msk |
                   CLK_CLKDCTL_LXTFIEN_Msk |
                   CLK_CLKDCTL_HXTFQDEN_Msk |
                   CLK_CLKDCTL_HXTFQIEN_Msk;

    /* Enable clock fail detector interrupt */
    NVIC_EnableIRQ(CKFAIL_IRQn);

    /* Wait for clock fail detector interrupt happened */
    while(1);
}
