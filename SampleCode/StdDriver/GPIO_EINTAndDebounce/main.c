/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show the usage of GPIO external interrupt function and de-bounce function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"



void EINT0_IRQHandler(void);
void EINT1_IRQHandler(void);
void SYS_Init(void);
void UART0_Init(void);

/**
 * @brief       External INT0 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The External INT0 default IRQ, declared in startup_m460.s.
 */
void EINT0_IRQHandler(void)
{

    /* To check if PA.6 external interrupt occurred */
    if(GPIO_GET_INT_FLAG(PA, BIT6))
    {
        GPIO_CLR_INT_FLAG(PA, BIT6);
        printf("PA.6 EINT0 occurred.\n");
    }

    /* To check if PB.5 external interrupt occurred */
    if(GPIO_GET_INT_FLAG(PB, BIT5))
    {
        GPIO_CLR_INT_FLAG(PB, BIT5);
        printf("PB.5 EINT0 occurred.\n");
    }

}

/**
 * @brief       External INT1 IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The External INT1 default IRQ, declared in startup_m460.s.
 */
void EINT1_IRQHandler(void)
{

    /* To check if PA.7 external interrupt occurred */
    if(GPIO_GET_INT_FLAG(PA, BIT7))
    {
        GPIO_CLR_INT_FLAG(PA, BIT7);
        printf("PA.7 EINT1 occurred.\n");
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

    /* Set multi-function pins for EINT0(PA.6), EINT0(PB.5) and EINT1(PA.7) */
    SET_INT0_PA6();
    SET_INT0_PB5();
    SET_INT1_PA7();

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
    printf("+------------------------------------------------------------+\n");
    printf("|    GPIO EINT0/EINT1 Interrupt and De-bounce Sample Code    |\n");
    printf("+------------------------------------------------------------+\n\n");

    /*-----------------------------------------------------------------------------------------------------*/
    /* GPIO External Interrupt Function Test                                                               */
    /*-----------------------------------------------------------------------------------------------------*/
    printf("EINT0(PA.6 and PB.5) and EINT1(PA.7) are used to test interrupt\n");

    /* Configure PA.6 as EINT0 pin and enable interrupt by falling edge trigger */
    GPIO_SetMode(PA, BIT6, GPIO_MODE_INPUT);
    GPIO_EnableInt(PA, 6, GPIO_INT_FALLING);

    /* Configure PB.5 as EINT0 pin and enable interrupt by rising edge trigger */
    GPIO_SetMode(PB, BIT5, GPIO_MODE_INPUT);
    GPIO_EnableInt(PB, 5, GPIO_INT_RISING);
    NVIC_EnableIRQ(EINT0_IRQn);

    /* Configure PA.7 as EINT1 pin and enable interrupt by falling and rising edge trigger */
    GPIO_SetMode(PA, BIT7, GPIO_MODE_INPUT);
    GPIO_EnableInt(PA, 7, GPIO_INT_BOTH_EDGE);
    NVIC_EnableIRQ(EINT1_IRQn);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO_SET_DEBOUNCE_TIME(PA, GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);
    GPIO_SET_DEBOUNCE_TIME(PB, GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);
    GPIO_ENABLE_DEBOUNCE(PA, BIT6);
    GPIO_ENABLE_DEBOUNCE(PB, BIT5);
    GPIO_ENABLE_DEBOUNCE(PA, BIT7);

    /* Waiting for interrupts */
    while(1);
}
