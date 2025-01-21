/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Transmit and receive UART data with PDMA.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"



#define ENABLE_PDMA_INTERRUPT
#define PDMA_TEST_LENGTH 100
#define PDMA_TIME 0x5555

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static uint8_t g_u8Tx_Buffer[PDMA_TEST_LENGTH];
static uint8_t g_u8Rx_Buffer[PDMA_TEST_LENGTH];

volatile uint32_t u32IsTestOver = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void UART0_Init(void);
void UART1_Init(void);
void PDMA_Init(void);
void PDMA0_IRQHandler(void);
void UART_PDMATest(void);
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

    /* Enable UART and PDMA0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(UART1_MODULE);
    CLK_EnableModuleClock(PDMA0_MODULE);

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
    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

void UART1_Init(void)
{
    /* Configure UART1 and set UART1 Baudrate */
    UART_Open(UART1, 115200);
}

void PDMA_Init(void)
{
    /* Open PDMA Channel */
    PDMA_Open(PDMA0,1 << 0); // Channel 0 for UART1 TX
    PDMA_Open(PDMA0,1 << 1); // Channel 1 for UART1 RX
    // Select basic mode
    PDMA_SetTransferMode(PDMA0,0, PDMA_UART1_TX, 0, 0);
    PDMA_SetTransferMode(PDMA0,1, PDMA_UART1_RX, 0, 0);
    // Set data width and transfer count
    PDMA_SetTransferCnt(PDMA0,0, PDMA_WIDTH_8, PDMA_TEST_LENGTH);
    PDMA_SetTransferCnt(PDMA0,1, PDMA_WIDTH_8, PDMA_TEST_LENGTH);
    //Set PDMA Transfer Address
    PDMA_SetTransferAddr(PDMA0,0, ((uint32_t) (&g_u8Tx_Buffer[0])), PDMA_SAR_INC, UART1_BASE, PDMA_DAR_FIX);
    PDMA_SetTransferAddr(PDMA0,1, UART1_BASE, PDMA_SAR_FIX, ((uint32_t) (&g_u8Rx_Buffer[0])), PDMA_DAR_INC);
    //Select Single Request
    PDMA_SetBurstType(PDMA0,0, PDMA_REQ_SINGLE, 0);
    PDMA_SetBurstType(PDMA0,1, PDMA_REQ_SINGLE, 0);
    //Set timeout
    //PDMA_SetTimeOut(PDMA0, 0, 0, 0x5555);
    //PDMA_SetTimeOut(PDMA0, 1, 0, 0x5555);

#ifdef ENABLE_PDMA_INTERRUPT
    PDMA_EnableInt(PDMA0,0, PDMA_INT_TRANS_DONE);
    PDMA_EnableInt(PDMA0,1, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA0_IRQn);
    u32IsTestOver = 0;
#endif
}

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

    /* Init UART1 for test */
    UART1_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("\nUART Sample Program\n");

    UART_PDMATest();

    while(1);
}

/**
 * @brief       PDMA IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PDMA default IRQ, declared in startup_m460.s.
 */
void PDMA0_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA0);

    if (status & 0x1)   /* abort */
    {
        printf("target abort interrupt !!\n");
        if (PDMA_GET_ABORT_STS(PDMA0) & 0x4)
            u32IsTestOver = 2;
        PDMA_CLR_ABORT_FLAG(PDMA0,PDMA_GET_ABORT_STS(PDMA0));
    }
    else if (status & 0x2)     /* done */
    {
        if ( (PDMA_GET_TD_STS(PDMA0) & (1 << 0)) && (PDMA_GET_TD_STS(PDMA0) & (1 << 1)) )
        {
            u32IsTestOver = 1;
            PDMA_CLR_TD_FLAG(PDMA0,PDMA_GET_TD_STS(PDMA0));
        }
    }
    else if (status & 0x300)     /* timeout */
    {
        printf("timeout interrupt !!\n");
        u32IsTestOver = 3;

        PDMA_SetTimeOut(PDMA0,0, 0, 0);
        PDMA_CLR_TMOUT_FLAG(PDMA0,0);
        PDMA_SetTimeOut(PDMA0,0, 1, PDMA_TIME);

        PDMA_SetTimeOut(PDMA0,1, 0, 0);
        PDMA_CLR_TMOUT_FLAG(PDMA0,1);
        PDMA_SetTimeOut(PDMA0,1, 1, PDMA_TIME);
    }
    else
        printf("unknown interrupt !!\n");
}


/*---------------------------------------------------------------------------------------------------------*/
/*  UART PDMA Test                                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void UART_PDMATest(void)
{
    uint32_t i, u32TimeOutCnt;

    printf("+-----------------------------------------------------------+\n");
    printf("|  UART PDMA Test                                           |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code will demo UART1 PDMA function.         |\n");
    printf("|    Please connect UART1_TX and UART1_RX pin.              |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("Please press any key to start test. \n\n");

    getchar();

    /*
        Using UAR1 external loop back.
        This code will send data from UART1_TX and receive data from UART1_RX.
    */

    for (i=0; i<PDMA_TEST_LENGTH; i++)
    {
        g_u8Tx_Buffer[i] = i;
        g_u8Rx_Buffer[i] = 0xff;
    }

    while(1)
    {
        PDMA_Init();

        UART1->INTEN = UART_INTEN_RLSIEN_Msk; // Enable Receive Line interrupt
        NVIC_EnableIRQ(UART1_IRQn);

        UART1->INTEN |= UART_INTEN_TXPDMAEN_Msk | UART_INTEN_RXPDMAEN_Msk;

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

        if (u32IsTestOver == 1)
            printf("test done...\n");
        else if (u32IsTestOver == 2)
            printf("target abort...\n");
        else if (u32IsTestOver == 3)
            printf("timeout...\n");
#else
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while( (!(PDMA_GET_TD_STS(PDMA0)&PDMA_TDSTS_TDIF0_Msk)) || (!(PDMA_GET_TD_STS(PDMA0)&PDMA_TDSTS_TDIF1_Msk)) )
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for PDMA time-out!\n");
                return;
            }
        }

        PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF0_Msk|PDMA_TDSTS_TDIF1_Msk);
#endif

        UART1->INTEN &= ~UART_INTEN_TXPDMAEN_Msk;
        UART1->INTEN &= ~UART_INTEN_RXPDMAEN_Msk;

        for (i=0; i<PDMA_TEST_LENGTH; i++)
        {
            if(g_u8Rx_Buffer[i] != i)
            {
                printf("\n Receive Data Compare Error !!");
                return;
            }

            g_u8Rx_Buffer[i] = 0xff;
        }

        printf("\nUART PDMA test Pass.\n");
    }

}

void UART1_IRQHandler(void)
{
    uint32_t u32DAT;
    uint32_t u32IntSts = UART1->INTSTS;

    if(u32IntSts & UART_INTSTS_HWRLSIF_Msk)
    {
        if(UART1->FIFOSTS & UART_FIFOSTS_BIF_Msk)
            printf("\n BIF \n");
        if(UART1->FIFOSTS & UART_FIFOSTS_FEF_Msk)
            printf("\n FEF \n");
        if(UART1->FIFOSTS & UART_FIFOSTS_PEF_Msk)
            printf("\n PEF \n");

        u32DAT = UART1->DAT; // read out data
        printf("\n Error Data is '0x%x' \n", u32DAT);
        UART1->FIFOSTS = (UART_FIFOSTS_BIF_Msk | UART_FIFOSTS_FEF_Msk | UART_FIFOSTS_PEF_Msk);
    }
}
