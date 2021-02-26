/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to use SC embedded timer.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void SC0_IRQHandler(void);
void SYS_Init(void);
void UART_Init(void);


/**
  * @brief  The interrupt services routine of smartcard port 0
  * @param  None
  * @retval None
  */
void SC0_IRQHandler(void)
{
    static uint32_t u32Sec = 1;

    if(SC0->INTSTS & SC_INTSTS_TMR0IF_Msk)
    {
        /* Clear interrupt flag */
        SC0->INTSTS = SC_INTSTS_TMR0IF_Msk;
        printf("%d sec\n", u32Sec++);
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


    /*---------------------------------------------------------------------------------------------------------*/
    /* Initialization for sample code                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable SC0 module clock and clock source from HIRC divide 1 */
    CLK_EnableModuleClock(SC0_MODULE);
    CLK_SetModuleClock(SC0_MODULE, CLK_CLKSEL3_SC0SEL_HIRC, CLK_CLKDIV1_SC0(1));
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(DEBUG_PORT, 115200);
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

    /* Init UART for printf */
    UART_Init();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("\nThis sample code demo how to use SC embedded timer.\n");

    /* Open smartcard interface */
    SC_Open(SC0, SC_PIN_STATE_IGNORE, SC_PIN_STATE_HIGH);

    /* Enable SC TIMER0 interrupt */
    SC_ENABLE_INT(SC0, SC_INTEN_TMR0IEN_Msk);
    NVIC_EnableIRQ(SC0_IRQn);

    /* Real ETU divider value is "11+1=12", and the duration of each ETU is 12/SC_CLK */
    SC0->ETUCTL = 11;

    /* Each 1,000,000 ETU will generate TIMER0 timeout event */
    SC_StartTimer(SC0, 0, SC_TMR_MODE_4, 1000000);  // timer counter will be reloaded.

    while(1) {}
}
