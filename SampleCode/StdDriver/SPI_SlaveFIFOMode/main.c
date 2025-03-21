/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Configure SPI0 as Slave mode and demonstrate how to communicate
 *           with an off-chip SPI Master device with FIFO mode. This sample
 *           code needs to work with SPI_MasterFIFOMode sample code.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

// *** <<< Use Configuration Wizard in Context Menu >>> ***
// <o> GPIO Slew Rate Control
// <0=> Normal <1=> High <2=> Fast
#define SlewRateMode    0
// *** <<< end of configuration section >>> ***

#define TEST_COUNT      16

#define InterruptMode /* Undefine it when using polling mode */

static uint32_t s_au32SourceData[TEST_COUNT];
static uint32_t s_au32DestinationData[TEST_COUNT];
static volatile uint32_t s_u32TxDataCount;
static volatile uint32_t s_u32RxDataCount;

void SYS_Init(void);
void SPI_Init(void);
void SPI0_IRQHandler(void);

void SYS_Init(void)
{
    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SET_XT1_OUT_PF2();
    SET_XT1_IN_PF3();

    /* Set PF multi-function pins for X32_OUT(PF.4) and X32_IN(PF.5) */
    SET_X32_OUT_PF4();
    SET_X32_IN_PF5();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC, HXT and LXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk | CLK_PWRCTL_LXTEN_Msk);

    /* Wait for HIRC, HXT and LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_HXTSTB_Msk | CLK_STATUS_LXTSTB_Msk);

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 200MHz */
    CLK_SetCoreClock(FREQ_200MHZ);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;
    CLK->AHBCLK1 |= CLK_AHBCLK1_GPICKEN_Msk | CLK_AHBCLK1_GPJCKEN_Msk;

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable SPI0 module clock */
    CLK_EnableModuleClock(SPI0_MODULE);

    /* Select SPI0 module clock source as PCLK1 */
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK1, MODULE_NoMsk);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Setup SPI0 multi-function pins */
    SET_SPI0_MOSI_PA0();
    SET_SPI0_MISO_PA1();
    SET_SPI0_CLK_PA2();
    SET_SPI0_SS_PA3();

    /* Enable SPI0 clock pin (PA2) and SPI0 SS pin (PA3) schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk | GPIO_SMTEN_SMTEN3_Msk;

#if (SlewRateMode == 0)
    /* Enable SPI0 I/O normal slew rate */
    GPIO_SetSlewCtl(PA, BIT0 | BIT1 | BIT2 | BIT3, GPIO_SLEWCTL_NORMAL);
#elif (SlewRateMode == 1)
    /* Enable SPI0 I/O high slew rate */
    GPIO_SetSlewCtl(PA, BIT0 | BIT1 | BIT2 | BIT3, GPIO_SLEWCTL_HIGH);
#elif (SlewRateMode == 2)
    /* Enable SPI0 I/O fast slew rate */
    GPIO_SetSlewCtl(PA, BIT0 | BIT1 | BIT2 | BIT3, GPIO_SLEWCTL_FAST);
#endif
}

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure as a slave, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Configure SPI0 as a low level active device. */
    SPI_Open(SPI0, SPI_SLAVE, SPI_MODE_0, 32, (uint32_t)NULL);
}

void SPI0_IRQHandler(void)
{
    while(s_u32RxDataCount < TEST_COUNT)
    {
        /* Check RX EMPTY flag */
        while(SPI_GET_RX_FIFO_EMPTY_FLAG(SPI0) == 0)
        {
            /* Read RX FIFO */
            s_au32DestinationData[s_u32RxDataCount++] = SPI_READ_RX(SPI0);
        }
        /* Check TX FULL flag and TX data count */
        while((SPI_GET_TX_FIFO_FULL_FLAG(SPI0) == 0) && (s_u32TxDataCount < TEST_COUNT))
        {
            /* Write to TX FIFO */
            SPI_WRITE_TX(SPI0, s_au32SourceData[s_u32TxDataCount++]);
        }
        if(s_u32TxDataCount >= TEST_COUNT)
            SPI_DisableInt(SPI0, SPI_FIFO_TXTH_INT_MASK); /* Disable TX FIFO threshold interrupt */

        /* Check the RX FIFO time-out interrupt flag */
        if(SPI_GetIntFlag(SPI0, SPI_FIFO_RXTO_INT_MASK))
        {
            /* If RX FIFO is not empty, read RX FIFO. */
            while(SPI_GET_RX_FIFO_EMPTY_FLAG(SPI0) == 0)
                s_au32DestinationData[s_u32RxDataCount++] = SPI_READ_RX(SPI0);
        }
    }
}

/* ------------- */
/* Main function */
/* ------------- */
int main(void)
{
    uint32_t u32DataCount;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Init SPI */
    SPI_Init();

    printf("\n\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                      SPI Slave Mode Sample Code                      |\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("\n");
    printf("Configure SPI0 as a slave.\n");
    printf("Bit length of a transaction: 32\n");
    printf("The I/O connection for SPI0:\n");
    printf("    SPI0_SS(PA3)\n    SPI0_CLK(PA2)\n");
    printf("    SPI0_MISO(PA1)\n    SPI0_MOSI(PA0)\n\n");
    printf("SPI controller will enable FIFO mode and transfer %d data to a off-chip master device.\n", TEST_COUNT);
    printf("In the meanwhile the SPI controller will receive %d data from the off-chip master device.\n", TEST_COUNT);
    printf("After the transfer is done, the %d received data will be printed out.\n", TEST_COUNT);

    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        /* Write the initial value to source buffer */
        s_au32SourceData[u32DataCount] = 0x00AA0000 + u32DataCount;
        /* Clear destination buffer */
        s_au32DestinationData[u32DataCount] = 0;
    }

    s_u32TxDataCount = 0;
    s_u32RxDataCount = 0;
    printf("Press any key if the master device configuration is ready.\n");
    getchar();
    printf("\n");

    /* Set TX FIFO threshold */
    SPI_SetFIFO(SPI0, 2, 2);

#ifdef InterruptMode
    /* Enable TX FIFO threshold interrupt and RX FIFO time-out interrupt */
    SPI_EnableInt(SPI0, SPI_FIFO_TXTH_INT_MASK | SPI_FIFO_RXTO_INT_MASK);
    NVIC_EnableIRQ(SPI0_IRQn);
#else
    /* Access TX and RX FIFO */
    while(s_u32RxDataCount < TEST_COUNT)
    {
        /* Check TX FULL flag and TX data count */
        if((SPI_GET_TX_FIFO_FULL_FLAG(SPI0) == 0) && (s_u32TxDataCount < TEST_COUNT))
            SPI_WRITE_TX(SPI0, s_au32SourceData[s_u32TxDataCount++]); /* Write to TX FIFO */
        /* Check RX EMPTY flag */
        if(SPI_GET_RX_FIFO_EMPTY_FLAG(SPI0) == 0)
            s_au32DestinationData[s_u32RxDataCount++] = SPI_READ_RX(SPI0); /* Read RX FIFO */
    }
#endif

    /* Print the received data */
    printf("Received data:\n");
    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        printf("%d:\t0x%X\n", u32DataCount, s_au32DestinationData[u32DataCount]);
    }

#ifdef InterruptMode
    /* Disable TX FIFO threshold interrupt and RX FIFO time-out interrupt */
    SPI_DisableInt(SPI0, SPI_FIFO_TXTH_INT_MASK | SPI_FIFO_RXTO_INT_MASK);
    NVIC_DisableIRQ(SPI0_IRQn);
#endif

    printf("The data transfer was done.\n");

    printf("\n\nExit SPI driver sample code.\n");

    /* Reset SPI0 */
    SPI_Close(SPI0);

    while(1);
}
