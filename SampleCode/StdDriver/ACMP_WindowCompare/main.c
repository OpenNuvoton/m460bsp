/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate the usage of ACMP window compare function
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

void ACMP01_IRQHandler(void);
void SYS_Init(void);



void ACMP01_IRQHandler(void)
{
    /* Clear interrupt flag */
    ACMP_CLR_INT_FLAG(ACMP01, 0);
    ACMP_CLR_INT_FLAG(ACMP01, 1);

    if(ACMP01->STATUS & ACMP_STATUS_ACMPWO_Msk)
    {
        printf("The input voltage is within the window\n");
    }
    else
    {
        printf("The input voltage is not within the window\n");
    }
}


void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock to 96MHz */
    CLK_SetCoreClock(96000000);


    /* Enable UART peripheral clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable ACMP01 peripheral clock */
    CLK_EnableModuleClock(ACMP01_MODULE);

    /* Update System Core Clock */
    /* User can use SystemCoreClockUpdate() to calculate SystemCoreClock and CyclesPerUs automatically. */
    SystemCoreClockUpdate();

    /* Set PA11 multi-function pin for ACMP0 positive input pin */
    SET_ACMP0_P0_PA11();

    /* Set PB4 multi-function pin for ACMP1 positive input pin */
    SET_ACMP1_P1_PB4();

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Disable digital input path of analog pin ACMP0_P0 and ACMP1_P1 to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PA, (1ul << 11));
    GPIO_DISABLE_DIGITAL_PATH(PB, (1ul << 4));
}


int main()
{

    /* Unlock protected registers */
    SYS_UnlockReg();
    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();
    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    printf("\nThis sample code demonstrates ACMP window compare function\n");
    printf("Connect the specific analog voltage source to the positive inputs\n");
    printf("of both comparators, PA11 and PB4. This sample code will monitor if the\n");
    printf("input is between the range of CRV0 and bandgap.\n");
    printf("Press any key to continue ...");
    getchar();
    printf("\n");

    /* Select VDDA as CRV source */
    ACMP_SELECT_CRV_SRC(ACMP01, ACMP_VREF_CRV0SSEL_VDDA);

    /* Select CRV level: VDDA * 9 / 63 */
    ACMP_CRV_SEL(ACMP01, 9);

    /* Enable CRV0 */
    ACMP_ENABLE_CRV0(ACMP01);

    /* Configure ACMP0. Enable ACMP0 and select CRV as the source of ACMP negative input. */
    ACMP_Open(ACMP01, 0, ACMP_CTL_NEGSEL_CRV, ACMP_CTL_HYSTERESIS_DISABLE);
    /* Configure ACMP1. Enable ACMP1 and select CRV as the source of ACMP negative input. */
    ACMP_Open(ACMP01, 1, ACMP_CTL_NEGSEL_VBG, ACMP_CTL_HYSTERESIS_DISABLE);
    /* Select P1 as ACMP1 positive input channel */
    ACMP_SELECT_P(ACMP01, 1, ACMP_CTL_POSSEL_P1);
    /* Enable window compare mode */
    ACMP_ENABLE_WINDOW_COMPARE(ACMP01, 0);
    ACMP_ENABLE_WINDOW_COMPARE(ACMP01, 1);

    /* Clear ACMP 0 and 1 interrupt flag */
    ACMP_CLR_INT_FLAG(ACMP01, 0);
    ACMP_CLR_INT_FLAG(ACMP01, 1);

    if(ACMP01->STATUS & ACMP_STATUS_ACMPWO_Msk)
    {
        printf("The input voltage in inside the window\n");
    }
    else
    {
        printf("The input voltage in outside the window\n");
    }

    /* Enable interrupt */
    ACMP_ENABLE_INT(ACMP01, 0);
    ACMP_ENABLE_INT(ACMP01, 1);
    /* Enable ACMP01 interrupt */
    NVIC_EnableIRQ(ACMP01_IRQn);

    while(1);

}
