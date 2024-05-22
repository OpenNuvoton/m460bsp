/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Transmit and receive UART data in UART IrDA mode.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"



/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void IrDA_FunctionTest(void);
void IrDA_FunctionTxTest(void);
void IrDA_FunctionRxTest(void);
void SYS_Init(void);
void UART0_Init(void);
void UART1_Init(void);


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

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(UART1_MODULE);

    /* Select UART module clock source and UART module clock divider */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set PA multi-function pins for UART1 RXD(PA.2) and TXD(PA.3) */
    SET_UART1_RXD_PA2();
    SET_UART1_TXD_PA3();

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

void UART1_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART1 */
    SYS_ResetModule(UART1_RST);

    /* Configure UART1 and set UART1 Baudrate */
    UART_Open(UART1, 115200);
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

    /* Init UART1 */
    UART1_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("\nUART Sample Program\n");

    /* UART sample IrDA function */
    IrDA_FunctionTest();

    printf("\nUART Sample Program End\n");

    while(1);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  IrDA Function Test                                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
void IrDA_FunctionTest(void)
{
    uint8_t u8Item;

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     Pin Configure                                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  ______                                          _______  |\n");
    printf("| |      |                                        |       | |\n");
    printf("| |Master|--UART1_TXD(PA.3) <==> UART1_RXD(PA.2)--|Slave  | |\n");
    printf("| |      |                                        |       | |\n");
    printf("| |______|                                        |_______| |\n");
    printf("|                                                           |\n");
    printf("+-----------------------------------------------------------+\n");

    /*
        UART0 is set to debug port and connect with PC firstly.
        The IrDA sample code needs two module board to execute.
        Set the master board is IrDA TX Mode and the other is IrDA Rx mode.
        Inputting char on terminal will be sent to the UART0 of master.
        After the master receiving, the inputting char will send to UART0 again.
        At the same time, it also sends to UART1 TX pin by IrDA mode.
        Slave will print received char after UART1 send out.
        Note that IrDA mode is ONLY used when baud rate equation is selected mode 0.

    */

    printf("\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|     IrDA Function Test                                      |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|  Description :                                              |\n");
    printf("|    The sample code needs two boards. One is Master and      |\n");
    printf("|    the other is slave.  Master will send data based on      |\n");
    printf("|    terminal input and Slave will printf received data on    |\n");
    printf("|    terminal screen.                                         |\n");
    printf("|  Please select Master or Slave test                         |\n");
    printf("|  [0] Master    [1] Slave                                    |\n");
    printf("+-------------------------------------------------------------+\n");
    u8Item = (uint8_t)getchar();

    if(u8Item == '0')
        IrDA_FunctionTxTest();
    else
        IrDA_FunctionRxTest();

}

/*---------------------------------------------------------------------------------------------------------*/
/*  IrDA Function Transmit Test                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void IrDA_FunctionTxTest(void)
{
    uint8_t u8OutChar;

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     IrDA Function Mode Test (Master)                      |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| 1). Input char by UART0 terminal.                         |\n");
    printf("| 2). UART1 will send a char according to step 1.           |\n");
    printf("| 3). Return step 1. (Press '0' to exit)                    |\n");
    printf("+-----------------------------------------------------------+\n");

    printf("\nIrDA Sample Code Start. \n");

    /* Set IrDA Tx Mode, Baud Rate configuration must be used MODE0 */
    UART_SelectIrDAMode(UART1, 57600, UART_IRDA_TXEN);

    /* Wait Terminal input to send data to UART1 TX pin */
    do
    {
        u8OutChar = (uint8_t)getchar();
        printf("   Input: %c , Send %c out\n", u8OutChar, u8OutChar);
        UART_WRITE(UART1, u8OutChar);
    }
    while(u8OutChar != '0');

    printf("\nIrDA Sample Code End.\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  IrDA Function Receive Test                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void IrDA_FunctionRxTest(void)
{
    uint8_t u8InChar = 0xFF;
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     IrDA Function Mode Test (Slave)                       |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| 1). Polling RDA_Flag to check data input though UART1     |\n");
    printf("| 2). If received data is '0', the program will exit.       |\n");
    printf("|     Otherwise, print received data on terminal            |\n");
    printf("+-----------------------------------------------------------+\n\n");

    /* Set IrDA Rx Mode, Baud Rate configuration must be used MODE0 */
    UART_SelectIrDAMode(UART1, 57600, UART_IRDA_RXEN);

    /* Reset Rx FIFO */
    UART1->FIFO |= UART_FIFO_RXRST_Msk;
    while(UART1->FIFO & UART_FIFO_RXRST_Msk)
        if(--u32TimeOutCnt == 0) break;

    printf("Waiting...\n");

    /* Use polling method to wait master data */
    do
    {
        if(UART_IS_RX_READY(UART1))
        {
            u8InChar = (uint8_t)UART_READ(UART1);
            printf("   Input: %c \n", u8InChar);
        }
    }
    while(u8InChar != '0');

}
