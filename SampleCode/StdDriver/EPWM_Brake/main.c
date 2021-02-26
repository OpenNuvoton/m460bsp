/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to use EPWM brake function.
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
void BRAKE0_IRQHandler(void);
void SYS_Init(void);
void UART0_Init(void);


/**
 * @brief       EPWM0 Brake0 IRQ Handler
 *
 * @param       None
 *
 * @return      None
 *
 * @details     ISR to handle EPWM0 Brake0 interrupt event
 */
void BRAKE0_IRQHandler(void)
{
    printf("\nFault brake!\n");
    printf("Press any key to unlock brake state. (EPWM0 channel 0 output will toggle again)\n");
    getchar();

    /* Clear brake interrupt flag */
    EPWM_ClearFaultBrakeIntFlag(EPWM0, EPWM_FB_EDGE);
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

    /* Enable EPWM0 module clock */
    CLK_EnableModuleClock(EPWM0_MODULE);

    /* Select EPWM0 module clock source */
    CLK_SetModuleClock(EPWM0_MODULE, CLK_CLKSEL2_EPWM0SEL_PCLK0, 0);


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

    /* Set multi-function pin for EPWM brake pin */
    SET_EPWM0_BRAKE0_PE8();

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
/* MAIN function                                                                                           */
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

    printf("\nConnet PE.8 (EPWM0 brake pin 0) to PD.3.\n");
    printf("It will generate brake interrupt and EPWM0 channel 0 output stop toggling.\n");

    GPIO_SetMode(PD, BIT3, GPIO_MODE_OUTPUT);
    PD3 = 0;

    /* EPWM0 frequency is 100Hz, duty 30%, */
    EPWM_ConfigOutputChannel(EPWM0, 0, 100, 30);

    /* Enable output of all EPWM channels */
    EPWM_EnableOutput(EPWM0, 0x3F);

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Enable brake and interrupt */
    EPWM_EnableFaultBrake(EPWM0, EPWM_CH_0_MASK, 1, EPWM_FB_EDGE_BKP0);
    EPWM_EnableFaultBrakeInt(EPWM0, 0);
    /* Enable brake noise filter : brake pin 0, filter count=7, filter clock=HCLK/128 */
    EPWM_EnableBrakeNoiseFilter(EPWM0, 0, 7, EPWM_NF_CLK_DIV_128);
    /* Clear brake interrupt flag */
    EPWM_ClearFaultBrakeIntFlag(EPWM0, EPWM_FB_EDGE);

    NVIC_EnableIRQ(BRAKE0_IRQn);

    /* Start */
    EPWM_Start(EPWM0, 1);

    printf("\nPress any key to generate a brake event\n");
    getchar();
    PD3 = 1;

    while(1);
}

/*** (C) COPYRIGHT 2021 Nuvoton Technology Corp. ***/
