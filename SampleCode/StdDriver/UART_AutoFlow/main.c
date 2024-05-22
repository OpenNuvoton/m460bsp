/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Transmit and receive data using auto flow control.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define RXBUFSIZE   1024

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static volatile int32_t g_i32Pointer = 0;
static uint8_t g_u8RecData[RXBUFSIZE] = {0};

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void AutoFlow_FunctionTest(void);
void AutoFlow_FunctionTxTest(void);
void AutoFlow_FunctionRxTest(void);
void SYS_Init(void);
void UART0_Init(void);
void UART1_Init(void);
void UART1_IRQHandler(void);


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

    /* Set PA multi-function pins for UART1 RXD(PA.2), TXD(PA.3), nRTS(PA.0) and nCTS(PA.1) */
    SET_UART1_RXD_PA2();
    SET_UART1_TXD_PA3();
    SET_UART1_nRTS_PA0();
    SET_UART1_nCTS_PA1();

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

    /* UART auto flow sample function */
    AutoFlow_FunctionTest();

    printf("\nUART Sample Program End\n");

    while(1);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  AutoFlow Function Test                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void AutoFlow_FunctionTest(void)
{
    uint8_t u8Item;

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     Pin Configure                                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  ______                                            _____  |\n");
    printf("| |      |                                          |     | |\n");
    printf("| |Master|--UART1_TXD(PA.3)  <==>  UART1_RXD(PA.2)--|Slave| |\n");
    printf("| |      |--UART1_nCTS(PA.1) <==> UART1_nRTS(PA.0)--|     | |\n");
    printf("| |______|                                          |_____| |\n");
    printf("|                                                           |\n");
    printf("+-----------------------------------------------------------+\n");

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|       AutoFlow Function Test                              |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code needs two boards. One is Master and    |\n");
    printf("|    the other is slave. Master will send 1k bytes data     |\n");
    printf("|    to slave. Slave will check if received data is correct |\n");
    printf("|  Please select Master or Slave test                       |\n");
    printf("|  [0] Master    [1] Slave                                  |\n");
    printf("+-----------------------------------------------------------+\n");
    u8Item = (uint8_t)getchar();

    if(u8Item == '0')
        AutoFlow_FunctionTxTest();
    else
        AutoFlow_FunctionRxTest();

}

/*---------------------------------------------------------------------------------------------------------*/
/*  AutoFlow Function Test (Master)                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void AutoFlow_FunctionTxTest(void)
{
    uint32_t u32Idx;

    /* Enable RTS and CTS autoflow control */
    UART_EnableFlowCtrl(UART1);

    /* Send 1k bytes data */
    for(u32Idx = 0; u32Idx < RXBUFSIZE; u32Idx++)
    {
        /* Send 1 byte data */
        UART_WRITE(UART1, u32Idx & 0xFF);

        /* Wait if Tx FIFO is full */
        while(UART_IS_TX_FULL(UART1));
    }

    printf("\n Transmit Done\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  AutoFlow Function Test (Slave)                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void AutoFlow_FunctionRxTest(void)
{
    uint32_t u32Idx, u32Err = 0;

    /* Enable RTS and CTS autoflow control */
    UART_EnableFlowCtrl(UART1);

    /* Set RTS Trigger Level as 8 bytes */
    UART1->FIFO = (UART1->FIFO & (~UART_FIFO_RTSTRGLV_Msk)) | UART_FIFO_RTSTRGLV_8BYTES;

    /* Set RX Trigger Level as 8 bytes */
    UART1->FIFO = (UART1->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_8BYTES;

    /* Set Timeout time 0x3E bit-time and time-out counter enable */
    UART_SetTimeoutCnt(UART1, 0x3E);

    /* Enable RDA and RTO Interrupt */
    NVIC_EnableIRQ(UART1_IRQn);
    UART_EnableInt(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RLSIEN_Msk | UART_INTEN_RXTOIEN_Msk));

    printf("\n Starting to receive data...\n");

    /* Wait for receive 1k bytes data */
    while(g_i32Pointer < RXBUFSIZE);

    /* Compare Data */
    for(u32Idx = 0; u32Idx < RXBUFSIZE; u32Idx++)
    {
        if(g_u8RecData[u32Idx] != (u32Idx & 0xFF))
        {
            u32Err = 1;
            break;
        }
    }

    if(u32Err)
        printf("Compare Data Failed\n");
    else
        printf("\n Receive OK & Check OK\n");

    /* Disable RDA and RTO Interrupt */
    NVIC_DisableIRQ(UART1_IRQn);
    UART_DisableInt(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RLSIEN_Msk | UART_INTEN_RXTOIEN_Msk));

}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 1 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_IRQHandler(void)
{
    uint8_t u8InChar = 0xFF;

    /* Rx Ready or Time-out INT */
    if(UART_GET_INT_FLAG(UART1, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))
    {
        /* Read data until RX FIFO is empty */
        while(UART_GET_RX_EMPTY(UART1) == 0)
        {
            u8InChar = (uint8_t)UART_READ(UART1);
            g_u8RecData[g_i32Pointer++] = u8InChar;
        }
    }

    /* Handle transmission error */
    if(UART1->FIFOSTS & (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk))
    {
        UART1->FIFOSTS = (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk | UART_FIFOSTS_RXOVIF_Msk);
    }

}
