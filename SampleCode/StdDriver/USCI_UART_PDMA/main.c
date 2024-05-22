/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Transmit and receive UART data with PDMA.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

//#define ENABLE_PDMA_INTERRUPT
#define PDMA_TEST_LENGTH   128

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t g_u8Tx_Buffer[PDMA_TEST_LENGTH] = {0};
uint8_t g_u8Rx_Buffer[PDMA_TEST_LENGTH] = {0};

volatile uint32_t u32IsTestOver = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_UART_PDMATest(void);

void PDMA0_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA0);

    if(status & 0x1)    /* abort */
    {
        printf("target abort interrupt !!\n");
        if(PDMA_GET_ABORT_STS(PDMA0) & 0x4)
            u32IsTestOver = 2;
        PDMA_CLR_ABORT_FLAG(PDMA0, PDMA_GET_ABORT_STS(PDMA0));
    }
    else if(status & 0x2)      /* done */
    {
        if((PDMA_GET_TD_STS(PDMA0) & (1 << 0)) && (PDMA_GET_TD_STS(PDMA0) & (1 << 1)))
        {
            u32IsTestOver = 1;
            PDMA_CLR_TD_FLAG(PDMA0, PDMA_GET_TD_STS(PDMA0));
        }
    }
    else if(status & 0x300)      /* channel 2 timeout */
    {
        printf("timeout interrupt !!\n");
        u32IsTestOver = 3;
        PDMA_CLR_TMOUT_FLAG(PDMA0, 0);
        PDMA_CLR_TMOUT_FLAG(PDMA0, 1);
    }
    else
        printf("unknown interrupt !!\n");
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

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable USCI0 module clock */
    CLK_EnableModuleClock(USCI0_MODULE);

    /* Enable PDMA0 module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Select UART module clock source and UART module clock divider */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set PA multi-function pins for USCI0_DAT0(PA.10) and USCI0_DAT1(PA.9) */
    SET_USCI0_DAT0_PA10();
    SET_USCI0_DAT1_PA9();

}

void UART0_Init()
{
    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

void USCI0_Init()
{
    /* Configure USCI0 as UART mode */
    UUART_Open(UUART0, 115200);
}

void PDMA_Init(void)
{
    /* Open PDMA Channel */
    PDMA_Open(PDMA0, 1 << 0); // Channel 0 for UART1 TX
    PDMA_Open(PDMA0, 1 << 1); // Channel 1 for UART1 RX
    // Select basic mode
    PDMA_SetTransferMode(PDMA0, 0, PDMA_USCI0_TX, 0, 0);
    PDMA_SetTransferMode(PDMA0, 1, PDMA_USCI0_RX, 0, 0);
    // Set data width and transfer count
    PDMA_SetTransferCnt(PDMA0, 0, PDMA_WIDTH_8, PDMA_TEST_LENGTH);
    PDMA_SetTransferCnt(PDMA0, 1, PDMA_WIDTH_8, PDMA_TEST_LENGTH);
    //Set PDMA Transfer Address
    PDMA_SetTransferAddr(PDMA0, 0, ((uint32_t)(&g_u8Tx_Buffer[0])), PDMA_SAR_INC, (uint32_t)(&(UUART0->TXDAT)), PDMA_DAR_FIX);
    PDMA_SetTransferAddr(PDMA0, 1, (uint32_t)(&(UUART0->RXDAT)), PDMA_SAR_FIX, ((uint32_t)(&g_u8Rx_Buffer[0])), PDMA_DAR_INC);
    //Select Single Request
    PDMA_SetBurstType(PDMA0, 0, PDMA_REQ_SINGLE, 0);
    PDMA_SetBurstType(PDMA0, 1, PDMA_REQ_SINGLE, 0);
    //Set timeout
    //PDMA_SetTimeOut(PDMA0, 0, 0, 0x5555);
    //PDMA_SetTimeOut(PDMA0, 1, 0, 0x5555);

#ifdef ENABLE_PDMA_INTERRUPT
    PDMA_EnableInt(PDMA0, 0, PDMA_INT_TRANS_DONE);
    PDMA_EnableInt(PDMA0, 1, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA0_IRQn);
    u32IsTestOver = 0;
#endif
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

    /* Init USCI0 */
    USCI0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);

    printf("\nUSCI UART Sample Program\n");

    /* USCI UART sample function */
    USCI_UART_PDMATest();

    while(1);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  USCI UART Function Test                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void USCI_UART_PDMATest(void)
{
    uint32_t i, u32TimeOutCnt;

    printf("+-----------------------------------------------------------+\n");
    printf("|  USCI UART PDMA Test                                      |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code will demo USCI_UART PDMA function      |\n");
    printf("|    Please connect USCI0_UART_TX and USCI0_UART_RX pin.    |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("Please press any key to start test. \n\n");

    getchar();

    for(i = 0; i < PDMA_TEST_LENGTH; i++)
    {
        g_u8Tx_Buffer[i] = i;
        g_u8Rx_Buffer[i] = 0xff;
    }

    /* PDMA reset */
    UUART0->PDMACTL |= UUART_PDMACTL_PDMARST_Msk;
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(UUART0->PDMACTL & UUART_PDMACTL_PDMARST_Msk)
        if(--u32TimeOutCnt == 0) break;

    PDMA_Init();

    /* Enable UART DMA Tx and Rx */
    UUART0->PDMACTL = UUART_PDMACTL_TXPDMAEN_Msk | UUART_PDMACTL_RXPDMAEN_Msk | UUART_PDMACTL_PDMAEN_Msk;

#ifdef ENABLE_PDMA_INTERRUPT
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(u32IsTestOver == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA time-out!\n");
            return;
        }
    }

    if(u32IsTestOver == 1)
        printf("test done...\n");
    else if(u32IsTestOver == 2)
        printf("target abort...\n");
    else if(u32IsTestOver == 3)
        printf("timeout...\n");
#else
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while((!(PDMA_GET_TD_STS(PDMA0)&PDMA_TDSTS_TDIF0_Msk) || (!(PDMA_GET_TD_STS(PDMA0)&PDMA_TDSTS_TDIF1_Msk))))
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA time-out!\n");
            return;
        }
    }

    PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF0_Msk | PDMA_TDSTS_TDIF1_Msk);
#endif

    for(i = 0; i < PDMA_TEST_LENGTH; i++)
    {
        if(g_u8Rx_Buffer[i] != i)
        {
            printf("\n Receive Data Compare Error !!");
            return;
        }

    }

    printf("\nUART PDMA test Pass.\n");

}
