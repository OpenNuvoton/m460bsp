/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate EPWM accumulator stop mode.
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
/**
 * @brief       EPWM1 IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle EPWM1 interrupt event
 */

void EPWM1P0_IRQHandler(void);
void SYS_Init(void);
void UART0_Init(void);


void EPWM1P0_IRQHandler(void)
{
    EPWM_ClearAccInt(EPWM1, 0);
    printf("Check if output toggles 11 times then stop toggles.\n");
}

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

    /* Enable EPWM1 module clock */
    CLK_EnableModuleClock(EPWM1_MODULE);

    /* Select EPWM1 module clock source */
    CLK_SetModuleClock(EPWM1_MODULE, CLK_CLKSEL2_EPWM1SEL_PCLK1, 0);


    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set multi-function pin for EPWM */
    SET_EPWM1_CH0_PC5();

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

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
    printf("  This sample code demonstrate EPWM1 channel 0 accumulator stop mode.\n");
    printf("  When accumulator interrupt happens, the output of EPWM1 channel 0 stops.\n");
    printf("  Since interrupt accumulator count is set to 10, the output toggles 11 times then stops. \n");

    printf("\n\nPress any key to start EPWM1 channel 0.\n");
    getchar();

    /*--------------------------------------------------------------------------------------*/
    /* Set the EPWM1 Channel 0 as EPWM output function.                                     */
    /*--------------------------------------------------------------------------------------*/

    /* Set EPWM1 channel 0 output configuration */
    EPWM_ConfigOutputChannel(EPWM1, 0, 300, 30);

    /* Enable EPWM Output path for EPWM1 channel 0 */
    EPWM_EnableOutput(EPWM1, EPWM_CH_0_MASK);

    /* Enable EPWM1 channel 0 accumulator, interrupt count 10, accumulator source select to zero point */
    EPWM_EnableAcc(EPWM1, 0, 10, EPWM_IFA_ZERO_POINT);

    /* Enable EPWM1 channel 0 accumulator interrupt */
    EPWM_EnableAccInt(EPWM1, 0);

    /* Enable EPWM1 channel 0 interrupt in the NVIC interrupt controller */
    NVIC_EnableIRQ(EPWM1P0_IRQn);

    /* Enable EPWM1 channel 0 accumulator stop mode */
    EPWM_EnableAccStopMode(EPWM1, 0);

    /* Enable Timer for EPWM1 channel 0 */
    EPWM_Start(EPWM1, EPWM_CH_0_MASK);

    while(1);
}

/*** (C) COPYRIGHT 2021 Nuvoton Technology Corp. ***/
