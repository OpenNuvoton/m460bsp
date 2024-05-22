/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to use BPWM counter output waveform.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

void SYS_Init(void);
void UART0_Init(void);

void SYS_Init(void)
{

    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SET_XT1_OUT_PF2();
    SET_XT1_IN_PF3();

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
    CLK_SetCoreClock(200000000);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;
    CLK->AHBCLK1 |= CLK_AHBCLK1_GPICKEN_Msk | CLK_AHBCLK1_GPJCKEN_Msk;

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable BPWM module clock */
    CLK_EnableModuleClock(BPWM0_MODULE);
    CLK_EnableModuleClock(BPWM1_MODULE);

    /* Select BPWM module clock source */
    CLK_SetModuleClock(BPWM0_MODULE, CLK_CLKSEL2_BPWM0SEL_PCLK0, 0);
    CLK_SetModuleClock(BPWM1_MODULE, CLK_CLKSEL2_BPWM1SEL_PCLK1, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set multi-function pin for BPWM */
    SET_BPWM0_CH0_PE2();
    SET_BPWM0_CH1_PE3();
    SET_BPWM0_CH2_PE4();
    SET_BPWM0_CH3_PE5();
    SET_BPWM0_CH4_PE6();
    SET_BPWM0_CH5_PE7();
    SET_BPWM1_CH0_PB11();
    SET_BPWM1_CH1_PB10();
    SET_BPWM1_CH2_PB9();
    SET_BPWM1_CH3_PB8();
    SET_BPWM1_CH4_PB7();
    SET_BPWM1_CH5_PB6();

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}


/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Init System, IP clock and multi-function I/O
       In the end of SYS_Init() will issue SYS_LockReg()
       to lock protected register. If user want to write
       protected register, please issue SYS_UnlockReg()
       to unlock protected register if necessary */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();

    printf("\n\nCPU @ %dHz(PLL@ %dHz)\n", SystemCoreClock, PllClock);
    printf("+------------------------------------------------------------------------+\n");
    printf("|                          BPWM Driver Sample Code                       |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will output waveform with BPWM0 and BPWM1 channel 0~5.\n");
    printf("  I/O configuration:\n");
    printf("  BPWM0 channel 0: 180000 Hz, duty 90%%.\n");
    printf("  BPWM0 channel 1: 180000 Hz, duty 80%%.\n");
    printf("  BPWM0 channel 2: 180000 Hz, duty 75%%.\n");
    printf("  BPWM0 channel 3: 180000 Hz, duty 70%%.\n");
    printf("  BPWM0 channel 4: 180000 Hz, duty 60%%.\n");
    printf("  BPWM0 channel 5: 180000 Hz, duty 50%%.\n");
    printf("  BPWM1 channel 0:  60000 Hz, duty 50%%.\n");
    printf("  BPWM1 channel 1:  60000 Hz, duty 40%%.\n");
    printf("  BPWM1 channel 2:  60000 Hz, duty 30%%.\n");
    printf("  BPWM1 channel 3:  60000 Hz, duty 25%%.\n");
    printf("  BPWM1 channel 4:  60000 Hz, duty 20%%.\n");
    printf("  BPWM1 channel 5:  60000 Hz, duty 10%%.\n");
    printf("  waveform output pin: BPWM0_CH0(PE.2), BPWM0_CH1(PE.3), BPWM0_CH2(PE.4), BPWM0_CH3(PE.5), BPWM0_CH4(PE.6), BPWM0_CH5(PE.7)\n");
    printf("                       BPWM1_CH0(PB.11), BPWM1_CH1(PB.10), BPWM1_CH2(PB.9), BPWM1_CH3(PB.8), BPWM1_CH4(PB.7), BPWM1_CH5(PB.6)\n");

    printf("Press any key to stop.\n");

    /* BPWM0 and BPWM1 channel 0~5 frequency and duty configuration are as follows */
    /* Because of BPWM0 channel 0~5 share one period, so the period value of all channels need set the same. */
    BPWM_ConfigOutputChannel(BPWM0, 0, 180000, 90);
    BPWM_ConfigOutputChannel(BPWM0, 1, 180000, 80);
    BPWM_ConfigOutputChannel(BPWM0, 2, 180000, 75);
    BPWM_ConfigOutputChannel(BPWM0, 3, 180000, 70);
    BPWM_ConfigOutputChannel(BPWM0, 4, 180000, 60);
    BPWM_ConfigOutputChannel(BPWM0, 5, 180000, 50);
    /* Because of BPWM1 channel 0~5 share one period, so the period value of all channels need set the same. */
    BPWM_ConfigOutputChannel(BPWM1, 0, 60000, 50);
    BPWM_ConfigOutputChannel(BPWM1, 1, 60000, 40);
    BPWM_ConfigOutputChannel(BPWM1, 2, 60000, 30);
    BPWM_ConfigOutputChannel(BPWM1, 3, 60000, 25);
    BPWM_ConfigOutputChannel(BPWM1, 4, 60000, 20);
    BPWM_ConfigOutputChannel(BPWM1, 5, 60000, 10);

    /* Enable output of BPWM0 and BPWM1 channel 0~5 */
    BPWM_EnableOutput(BPWM0, 0x3F);
    BPWM_EnableOutput(BPWM1, 0x3F);

    /* Start BPWM0 counter */
    BPWM_Start(BPWM0, 0x3F);
    /* Start BPWM1 counter */
    BPWM_Start(BPWM1, 0x3F);

    /* Wait for user press any key to stop */
    getchar();

    /* Stop BPWM0 counter */
    BPWM_ForceStop(BPWM0, 0x3F);
    /* Stop BPWM1 counter */
    BPWM_ForceStop(BPWM1, 0x3F);

    printf("Done.");
    while(1);

}


/*** (C) COPYRIGHT 2021 Nuvoton Technology Corp. ***/
