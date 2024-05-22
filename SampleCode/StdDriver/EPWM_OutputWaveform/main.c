/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to use EPWM counter output waveform.
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

    /* Enable EPWM module clock */
    CLK_EnableModuleClock(EPWM0_MODULE);
    CLK_EnableModuleClock(EPWM1_MODULE);

    /* Select EPWM module clock source */
    CLK_SetModuleClock(EPWM0_MODULE, CLK_CLKSEL2_EPWM0SEL_PCLK0, 0);
    CLK_SetModuleClock(EPWM1_MODULE, CLK_CLKSEL2_EPWM1SEL_PCLK1, 0);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set multi-function pin for EPWM */
    SET_EPWM0_CH0_PE7();
    SET_EPWM0_CH1_PE6();
    SET_EPWM0_CH2_PE5();
    SET_EPWM0_CH3_PE4();
    SET_EPWM0_CH4_PE3();
    SET_EPWM0_CH5_PE2();

    SET_EPWM1_CH0_PC5();
    SET_EPWM1_CH1_PC4();
    SET_EPWM1_CH2_PC3();
    SET_EPWM1_CH3_PC2();
    SET_EPWM1_CH4_PC1();
    SET_EPWM1_CH5_PC0();

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
    printf("|                          EPWM Driver Sample Code                       |\n");
    printf("|                                                                        |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  This sample code will output waveform with EPWM0 and EPWM1 channel 0~5.\n");
    printf("  I/O configuration:\n");
    printf("  EPWM0 channel 0: 360000 Hz, duty 90%%.\n");
    printf("  EPWM0 channel 1: 320000 Hz, duty 80%%.\n");
    printf("  EPWM0 channel 2: 250000 Hz, duty 75%%.\n");
    printf("  EPWM0 channel 3: 180000 Hz, duty 70%%.\n");
    printf("  EPWM0 channel 4: 160000 Hz, duty 60%%.\n");
    printf("  EPWM0 channel 5: 150000 Hz, duty 50%%.\n");
    printf("  EPWM1 channel 0: 120000 Hz, duty 50%%.\n");
    printf("  EPWM1 channel 1: 100000 Hz, duty 40%%.\n");
    printf("  EPWM1 channel 2:  90000 Hz, duty 30%%.\n");
    printf("  EPWM1 channel 3:  60000 Hz, duty 25%%.\n");
    printf("  EPWM1 channel 4:  45000 Hz, duty 20%%.\n");
    printf("  EPWM1 channel 5:  30000 Hz, duty 10%%.\n");
    printf("    waveform output pin: EPWM0_CH0(PE.7), EPWM0_CH1(PE.6), EPWM0_CH2(PE.5), EPWM0_CH3(PE.4), EPWM0_CH4(PE.3), EPWM0_CH5(PE.2)\n");
    printf("                         EPWM1_CH0(PC.5), EPWM1_CH1(PC.4), EPWM1_CH2(PC.3), EPWM1_CH3(PC.2), EPWM1_CH4(PC.1), EPWM1_CH5(PC.0)\n");


    /* EPWM0 and EPWM1 channel 0~5 frequency and duty configuration are as follows */
    EPWM_ConfigOutputChannel(EPWM0, 0, 360000, 90);
    EPWM_ConfigOutputChannel(EPWM0, 1, 320000, 80);
    EPWM_ConfigOutputChannel(EPWM0, 2, 250000, 75);
    EPWM_ConfigOutputChannel(EPWM0, 3, 180000, 70);
    EPWM_ConfigOutputChannel(EPWM0, 4, 160000, 60);
    EPWM_ConfigOutputChannel(EPWM0, 5, 150000, 50);
    EPWM_ConfigOutputChannel(EPWM1, 0, 120000, 50);
    EPWM_ConfigOutputChannel(EPWM1, 1, 100000, 40);
    EPWM_ConfigOutputChannel(EPWM1, 2, 90000, 30);
    EPWM_ConfigOutputChannel(EPWM1, 3, 60000, 25);
    EPWM_ConfigOutputChannel(EPWM1, 4, 45000, 20);
    EPWM_ConfigOutputChannel(EPWM1, 5, 30000, 10);

    /* Enable output of EPWM0 and EPWM1 channel 0~5 */
    EPWM_EnableOutput(EPWM0, 0x3F);
    EPWM_EnableOutput(EPWM1, 0x3F);

    /* Start EPWM0 counter */
    EPWM_Start(EPWM0, 0x3F);
    /* Start EPWM1 counter */
    EPWM_Start(EPWM1, 0x3F);

    printf("Press any key to stop.\n");
    getchar();

    /* Stop EPWM0 counter */
    EPWM_ForceStop(EPWM0, 0x3F);
    /* Stop EPWM1 counter */
    EPWM_ForceStop(EPWM1, 0x3F);

    printf("Done.\n");
    while(1);

}

/*** (C) COPYRIGHT 2021 Nuvoton Technology Corp. ***/
