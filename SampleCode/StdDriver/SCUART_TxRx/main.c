/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate smartcard UART mode by connecting PB.4 and PB.5 pins.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
/* This is the string we used in loopback demo */
static uint8_t g_au8TxBuf[] = "Hello World!";

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
    // Print SCUART received data to UART port
    // Data length here is short, so we're not care about UART FIFO over flow.
    while(!SCUART_GET_RX_EMPTY(SC0))
        UART_WRITE(DEBUG_PORT, SCUART_READ(SC0));

    // RDA is the only interrupt enabled in this sample, this status bit
    // automatically cleared after Rx FIFO empty. So no need to clear interrupt
    // status here.
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
    /* Enable SC0 module clock and clock source from HIRC*/
    CLK_EnableModuleClock(SC0_MODULE);
    CLK_SetModuleClock(SC0_MODULE, CLK_CLKSEL3_SC0SEL_HIRC, CLK_CLKDIV1_SC0(1));

    /* Set multi-function pins for SC UART mode */
    SET_SC0_DAT_PB4();
    SET_SC0_CLK_PB5();
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
    printf("+----------------------------------------+\n");
    printf("|    Smartcard UART Mode Sample Code     |\n");
    printf("+----------------------------------------+\n\n");
    printf("# Please connect SC0 CLK pin(PB.5) with SC0 DAT pin(PB.4) first.\n");
    printf("    - PB.5 as UART Tx\n");
    printf("    - PB.4 as UART Rx\n");
    printf("# Check UART message ... Is Hello World! ?\n\n");

    /* Open smartcard interface 0 in UART mode. The line config will be 115200-8n1 */
    /* Can call SCUART_SetLineConfig() later if necessary */
    SCUART_Open(SC0, 115200);

    /* Enable receive interrupt, no need to use other interrupts in this demo */
    SCUART_ENABLE_INT(SC0, SC_INTEN_RDAIEN_Msk);
    NVIC_EnableIRQ(SC0_IRQn);

    /*
        Send the demo string out from SC0_CLK pin,
        Received data from SC0_DAT pin will be print out to UART console
    */
    SCUART_Write(SC0, g_au8TxBuf, sizeof(g_au8TxBuf));

    while(1) {}
}
