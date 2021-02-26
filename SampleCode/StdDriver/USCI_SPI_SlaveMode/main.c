/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Configure USCI_SPI0 as Slave mode and demonstrate how to communicate with an off-chip SPI Master device.
 *           This sample code needs to work with USCI_SPI_MasterMode sample code.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define TEST_COUNT  16

static uint32_t s_au32SourceData[TEST_COUNT];
static uint32_t s_au32DestinationData[TEST_COUNT];

/* Function prototype declaration */
void SYS_Init(void);
void USCI_SPI_Init(void);

/* ------------- */
/* Main function */
/* ------------- */
int main()
{
    uint32_t u32TxDataCount, u32RxDataCount;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Init USCI_SPI0 */
    USCI_SPI_Init();

    printf("\n\n");
    printf("+-----------------------------------------------------+\n");
    printf("|           USCI_SPI Slave Mode Sample Code           |\n");
    printf("+-----------------------------------------------------+\n");
    printf("\n");
    printf("Configure USCI_SPI0 as a slave.\n");
    printf("Bit length of a transaction: 16\n");
    printf("The I/O connection for USCI_SPI0:\n");
    printf("    USCI_SPI0_SS (PD.4)\n    USCI_SPI0_CLK (PD.0)\n");
    printf("    USCI_SPI0_MISO (PD.2)\n    USCI_SPI0_MOSI (PD.1)\n\n");
    printf("USCI_SPI controller will transfer %d data to a off-chip master device.\n", TEST_COUNT);
    printf("In the meanwhile the USCI_SPI controller will receive %d data from the off-chip master device.\n", TEST_COUNT);
    printf("After the transfer is done, the %d received data will be printed out.\n", TEST_COUNT);

    for(u32TxDataCount = 0; u32TxDataCount < TEST_COUNT; u32TxDataCount++)
    {
        /* Write the initial value to source buffer */
        s_au32SourceData[u32TxDataCount] = 0xAA00 + u32TxDataCount;
        /* Clear destination buffer */
        s_au32DestinationData[u32TxDataCount] = 0;
    }

    u32TxDataCount = 0;
    u32RxDataCount = 0;
    printf("Press any key if the master device configuration is ready.");
    getchar();
    printf("\n");

    /* Access TX and RX Buffer */
    while(u32RxDataCount < TEST_COUNT)
    {
        /* Check TX FULL flag and TX data count */
        if((USPI_GET_TX_FULL_FLAG(USPI0) == 0) && (u32TxDataCount < TEST_COUNT))
            USPI_WRITE_TX(USPI0, s_au32SourceData[u32TxDataCount++]); /* Write to TX Buffer */
        /* Check RX EMPTY flag */
        if(USPI_GET_RX_EMPTY_FLAG(USPI0) == 0)
            s_au32DestinationData[u32RxDataCount++] = USPI_READ_RX(USPI0); /* Read RX Buffer */
    }

    /* Print the received data */
    printf("Received data:\n");
    for(u32RxDataCount = 0; u32RxDataCount < TEST_COUNT; u32RxDataCount++)
    {
        printf("%d:\t0x%X\n", u32RxDataCount, s_au32DestinationData[u32RxDataCount]);
    }
    printf("The data transfer was done.\n");

    printf("\n\nExit USCI_SPI driver sample code.\n");

    /* Close USCI_SPI0 */
    USPI_Close(USPI0);
    while(1);
}

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

    /* Enable USCI0 module clock */
    CLK_EnableModuleClock(USCI0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set USCI0_SPI multi-function pins */
    SET_USCI0_CLK_PD0();
    SET_USCI0_DAT0_PD1();
    SET_USCI0_DAT1_PD2();
    SET_USCI0_CTL0_PD4();

    /* USCI_SPI clock pin enable schmitt trigger */
    PD->SMTEN |= GPIO_SMTEN_SMTEN0_Msk;
}

void USCI_SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init USCI_SPI0                                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure USCI_SPI0 as a slave, USCI_SPI0 clock rate = f_PCLK1,
       clock idle low, 16-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    USPI_Open(USPI0, USPI_SLAVE, USPI_MODE_0, 16, (uint32_t)NULL);
    /* Configure USCI_SPI_SS pin as low-active. */
    USPI0->CTLIN0 = (USPI0->CTLIN0 & ~USPI_CTLIN0_ININV_Msk) | USPI_CTLIN0_ININV_Msk;
}
