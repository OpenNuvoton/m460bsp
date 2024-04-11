/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate SPI data transfer with PDMA.
 *           QSPI0 will be configured as Master mode and SPI1 will be configured as Slave mode.
 *           Both TX PDMA function and RX PDMA function will be enabled.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

// *** <<< Use Configuration Wizard in Context Menu >>> ***
// <o> GPIO Slew Rate Control
// <0=> Normal <1=> High <2=> Fast
#define SlewRateMode    1
// *** <<< end of configuration section >>> ***

#define SPI_MASTER_TX_DMA_CH 0
#define SPI_MASTER_RX_DMA_CH 1
#define SPI_SLAVE_TX_DMA_CH  2
#define SPI_SLAVE_RX_DMA_CH  3

#define TEST_COUNT      64

/* Function prototype declaration */
void SYS_Init(void);
void SPI_Init(void);
void SpiLoopTest_WithPDMA(void);

/* Global variable declaration */
static uint32_t s_au32MasterToSlaveTestPattern[TEST_COUNT];
static uint32_t s_au32SlaveToMasterTestPattern[TEST_COUNT];
static uint32_t s_au32MasterRxBuffer[TEST_COUNT];
static uint32_t s_au32SlaveRxBuffer[TEST_COUNT];

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

    /* Enable QSPI0 module clock */
    CLK_EnableModuleClock(QSPI0_MODULE);

    /* Enable SPI1 module clock */
    CLK_EnableModuleClock(SPI1_MODULE);

    /* Select QSPI0 and SPI1 module clock source as PCLK0 */
    CLK_SetModuleClock(QSPI0_MODULE, CLK_CLKSEL2_QSPI0SEL_PCLK0, MODULE_NoMsk);
    CLK_SetModuleClock(SPI1_MODULE, CLK_CLKSEL2_SPI1SEL_PCLK0, MODULE_NoMsk);

    /* Enable PDMA0 module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Configure QSPI0 related multi-function pins. GPA[3:0] : QSPI0_SS, QSPI0_CLK, QSPI0_MISO0, QSPI0_MOSI0. */
    SET_QSPI0_MOSI0_PA0();
    SET_QSPI0_MISO0_PA1();
    SET_QSPI0_CLK_PA2();
    SET_QSPI0_SS_PA3();

    /* Enable QSPI0 clock pin (PA2) schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

    /* Configure SPI1 related multi-function pins. GPH[7:4] : SPI1_SS, SPI1_CLK, SPI1_MOSI, SPI1_MISO. */
    SET_SPI1_MISO_PH4();
    SET_SPI1_MOSI_PH5();
    SET_SPI1_CLK_PH6();
    SET_SPI1_SS_PH7();

#if (SlewRateMode == 0)
    /* Enable QSPI0 I/O normal slew rate */
    GPIO_SetSlewCtl(PA, BIT0 | BIT1 | BIT2 | BIT3, GPIO_SLEWCTL_NORMAL);

    /* Enable SPI1 I/O normal slew rate */
    GPIO_SetSlewCtl(PH, BIT4 | BIT5 | BIT6 | BIT7, GPIO_SLEWCTL_NORMAL);
#elif (SlewRateMode == 1)
    /* Enable QSPI0 I/O high slew rate */
    GPIO_SetSlewCtl(PA, BIT0 | BIT1 | BIT2 | BIT3, GPIO_SLEWCTL_HIGH);

    /* Enable SPI1 I/O high slew rate */
    GPIO_SetSlewCtl(PH, BIT4 | BIT5 | BIT6 | BIT7, GPIO_SLEWCTL_HIGH);
#elif (SlewRateMode == 2)
    /* Enable QSPI0 I/O fast slew rate */
    GPIO_SetSlewCtl(PA, BIT0 | BIT1 | BIT2 | BIT3, GPIO_SLEWCTL_FAST);

    /* Enable SPI1 I/O fast slew rate */
    GPIO_SetSlewCtl(PH, BIT4 | BIT5 | BIT6 | BIT7, GPIO_SLEWCTL_FAST);
#endif
}

void SPI_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init SPI                                                                                                */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Configure QSPI0 */
    /* Configure QSPI0 as a master, SPI clock rate 2MHz,
       clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    QSPI_Open(QSPI0, QSPI_MASTER, QSPI_MODE_0, 32, 2000000);
    /* Enable the automatic hardware slave selection function. Select the QSPI0_SS pin and configure as low-active. */
    QSPI_EnableAutoSS(QSPI0, QSPI_SS, QSPI_SS_ACTIVE_LOW);

    /* Configure SPI1 */
    /* Configure SPI1 as a slave, clock idle low, 32-bit transaction, drive output on falling clock edge and latch input on rising edge. */
    /* Configure SPI1 as a low level active device. SPI peripheral clock rate = f_PCLK1 */
    SPI_Open(SPI1, SPI_SLAVE, SPI_MODE_0, 32, (uint32_t)NULL);
}

void SpiLoopTest_WithPDMA(void)
{
    uint32_t u32DataCount, u32TestCycle;
    uint32_t u32RegValue, u32Abort;
    int32_t i32Err;

    printf("\nQSPI0/SPI1 Loop test with PDMA ");

    /* Source data initiation */
    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
    {
        s_au32MasterToSlaveTestPattern[u32DataCount] = 0x55000000 | (u32DataCount + 1);
        s_au32SlaveToMasterTestPattern[u32DataCount] = 0xAA000000 | (u32DataCount + 1);
    }

    /* Reset PDMA module */
    SYS_ResetModule(PDMA0_RST);

    /* Enable PDMA channels */
    PDMA_Open(PDMA0, (1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH) | (1 << SPI_SLAVE_RX_DMA_CH) | (1 << SPI_SLAVE_TX_DMA_CH));

    /*=======================================================================
      SPI master PDMA TX channel configuration:
      -----------------------------------------------------------------------
        Word length = 32 bits
        Transfer Count = TEST_COUNT
        Source = s_au32MasterToSlaveTestPattern
        Source Address = Increasing
        Destination = QSPI0->TX
        Destination Address = Fixed
        Burst Type = Single Transfer
    =========================================================================*/
    /* Set transfer width (32 bits) and transfer count */
    PDMA_SetTransferCnt(PDMA0, SPI_MASTER_TX_DMA_CH, PDMA_WIDTH_32, TEST_COUNT);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA0, SPI_MASTER_TX_DMA_CH, (uint32_t)s_au32MasterToSlaveTestPattern, PDMA_SAR_INC, (uint32_t)&QSPI0->TX, PDMA_DAR_FIX);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA0, SPI_MASTER_TX_DMA_CH, PDMA_QSPI0_TX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA0, SPI_MASTER_TX_DMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA0->DSCT[SPI_MASTER_TX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;

    /*=======================================================================
      SPI master PDMA RX channel configuration:
      -----------------------------------------------------------------------
        Word length = 32 bits
        Transfer Count = TEST_COUNT
        Source = QSPI0->RX
        Source Address = Fixed
        Destination = s_au32MasterRxBuffer
        Destination Address = Increasing
        Burst Type = Single Transfer
    =========================================================================*/
    /* Set transfer width (32 bits) and transfer count */
    PDMA_SetTransferCnt(PDMA0, SPI_MASTER_RX_DMA_CH, PDMA_WIDTH_32, TEST_COUNT);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA0, SPI_MASTER_RX_DMA_CH, (uint32_t)&QSPI0->RX, PDMA_SAR_FIX, (uint32_t)s_au32MasterRxBuffer, PDMA_DAR_INC);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA0, SPI_MASTER_RX_DMA_CH, PDMA_QSPI0_RX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA0, SPI_MASTER_RX_DMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA0->DSCT[SPI_MASTER_RX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;

    /*=======================================================================
      SPI slave PDMA RX channel configuration:
      -----------------------------------------------------------------------
        Word length = 32 bits
        Transfer Count = TEST_COUNT
        Source = SPI1->RX
        Source Address = Fixed
        Destination = s_au32SlaveRxBuffer
        Destination Address = Increasing
        Burst Type = Single Transfer
    =========================================================================*/
    /* Set transfer width (32 bits) and transfer count */
    PDMA_SetTransferCnt(PDMA0, SPI_SLAVE_RX_DMA_CH, PDMA_WIDTH_32, TEST_COUNT);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA0, SPI_SLAVE_RX_DMA_CH, (uint32_t)&SPI1->RX, PDMA_SAR_FIX, (uint32_t)s_au32SlaveRxBuffer, PDMA_DAR_INC);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA0, SPI_SLAVE_RX_DMA_CH, PDMA_SPI1_RX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA0, SPI_SLAVE_RX_DMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA0->DSCT[SPI_SLAVE_RX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;

    /*=======================================================================
      SPI slave PDMA TX channel configuration:
      -----------------------------------------------------------------------
        Word length = 32 bits
        Transfer Count = TEST_COUNT
        Source = s_au32SlaveToMasterTestPattern
        Source Address = Increasing
        Destination = SPI1->TX
        Destination Address = Fixed
        Burst Type = Single Transfer
    =========================================================================*/
    /* Set transfer width (32 bits) and transfer count */
    PDMA_SetTransferCnt(PDMA0, SPI_SLAVE_TX_DMA_CH, PDMA_WIDTH_32, TEST_COUNT);
    /* Set source/destination address and attributes */
    PDMA_SetTransferAddr(PDMA0, SPI_SLAVE_TX_DMA_CH, (uint32_t)s_au32SlaveToMasterTestPattern, PDMA_SAR_INC, (uint32_t)&SPI1->TX, PDMA_DAR_FIX);
    /* Set request source; set basic mode. */
    PDMA_SetTransferMode(PDMA0, SPI_SLAVE_TX_DMA_CH, PDMA_SPI1_TX, FALSE, 0);
    /* Single request type. SPI only support PDMA single request type. */
    PDMA_SetBurstType(PDMA0, SPI_SLAVE_TX_DMA_CH, PDMA_REQ_SINGLE, 0);
    /* Disable table interrupt */
    PDMA0->DSCT[SPI_SLAVE_TX_DMA_CH].CTL |= PDMA_DSCT_CTL_TBINTDIS_Msk;

    /* Enable SPI slave PDMA function */
    SPI_TRIGGER_TX_RX_PDMA(SPI1);
    /* Enable SPI master PDMA function */
    QSPI_TRIGGER_TX_RX_PDMA(QSPI0);

    i32Err = 0;
    for(u32TestCycle = 0; u32TestCycle < 10000; u32TestCycle++)
    {
        if((u32TestCycle & 0x1FF) == 0)
            putchar('.');

        while(1)
        {
            /* Get interrupt status */
            u32RegValue = PDMA_GET_INT_STATUS(PDMA0);

            /* Check the PDMA transfer done interrupt flag */
            if(u32RegValue & PDMA_INTSTS_TDIF_Msk)
            {
                /* Check the PDMA transfer done flags */
                if((PDMA_GET_TD_STS(PDMA0) & ((1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH) | (1 << SPI_SLAVE_TX_DMA_CH) | (1 << SPI_SLAVE_RX_DMA_CH))) ==
                        ((1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH) | (1 << SPI_SLAVE_TX_DMA_CH) | (1 << SPI_SLAVE_RX_DMA_CH)))
                {
                    /* Clear the PDMA transfer done flags */
                    PDMA_CLR_TD_FLAG(PDMA0, (1 << SPI_MASTER_TX_DMA_CH) | (1 << SPI_MASTER_RX_DMA_CH) | (1 << SPI_SLAVE_TX_DMA_CH) | (1 << SPI_SLAVE_RX_DMA_CH));

                    /* Disable SPI master's PDMA transfer function */
                    QSPI_DISABLE_TX_RX_PDMA(QSPI0);

                    /* Check the transfer data */
                    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
                    {
                        if(s_au32MasterToSlaveTestPattern[u32DataCount] != s_au32SlaveRxBuffer[u32DataCount])
                        {
                            i32Err = 1;
                            break;
                        }
                        if(s_au32SlaveToMasterTestPattern[u32DataCount] != s_au32MasterRxBuffer[u32DataCount])
                        {
                            i32Err = 1;
                            break;
                        }
                    }

                    if(u32TestCycle >= 10000)
                        break;

                    /* Source data initiation */
                    for(u32DataCount = 0; u32DataCount < TEST_COUNT; u32DataCount++)
                    {
                        s_au32MasterToSlaveTestPattern[u32DataCount]++;
                        s_au32SlaveToMasterTestPattern[u32DataCount]++;
                    }
                    /* Re-trigger */
                    /* Slave PDMA TX channel configuration */
                    /* Set transfer width (32 bits) and transfer count */
                    PDMA_SetTransferCnt(PDMA0, SPI_SLAVE_TX_DMA_CH, PDMA_WIDTH_32, TEST_COUNT);
                    /* Set request source; set basic mode. */
                    PDMA_SetTransferMode(PDMA0, SPI_SLAVE_TX_DMA_CH, PDMA_SPI1_TX, FALSE, 0);

                    /* Slave PDMA RX channel configuration */
                    /* Set transfer width (32 bits) and transfer count */
                    PDMA_SetTransferCnt(PDMA0, SPI_SLAVE_RX_DMA_CH, PDMA_WIDTH_32, TEST_COUNT);
                    /* Set request source; set basic mode. */
                    PDMA_SetTransferMode(PDMA0, SPI_SLAVE_RX_DMA_CH, PDMA_SPI1_RX, FALSE, 0);

                    /* Master PDMA TX channel configuration */
                    /* Set transfer width (32 bits) and transfer count */
                    PDMA_SetTransferCnt(PDMA0, SPI_MASTER_TX_DMA_CH, PDMA_WIDTH_32, TEST_COUNT);
                    /* Set request source; set basic mode. */
                    PDMA_SetTransferMode(PDMA0, SPI_MASTER_TX_DMA_CH, PDMA_QSPI0_TX, FALSE, 0);

                    /* Master PDMA RX channel configuration */
                    /* Set transfer width (32 bits) and transfer count */
                    PDMA_SetTransferCnt(PDMA0, SPI_MASTER_RX_DMA_CH, PDMA_WIDTH_32, TEST_COUNT);
                    /* Set request source; set basic mode. */
                    PDMA_SetTransferMode(PDMA0, SPI_MASTER_RX_DMA_CH, PDMA_QSPI0_RX, FALSE, 0);

                    /* Enable master's PDMA transfer function */
                    QSPI_TRIGGER_TX_RX_PDMA(QSPI0);
                    break;
                }
            }
            /* Check the PDMA transfer abort interrupt flag */
            if(u32RegValue & PDMA_INTSTS_ABTIF_Msk)
            {
                /* Get the target abort flag */
                u32Abort = PDMA_GET_ABORT_STS(PDMA0);
                /* Clear the target abort flag */
                PDMA_CLR_ABORT_FLAG(PDMA0, u32Abort);
                i32Err = 1;
                break;
            }
            /* Check the PDMA time-out interrupt flag */
            if(u32RegValue & 0x00000300)
            {
                /* Clear the time-out flag */
                PDMA0->INTSTS = u32RegValue & 0x00000300;
                i32Err = 1;
                break;
            }
        }

        if(i32Err)
            break;
    }

    /* Disable all PDMA channels */
    PDMA_Close(PDMA0);

    if(i32Err)
    {
        printf(" [FAIL]\n");
    }
    else
    {
        printf(" [PASS]\n");
    }

    return;
}

int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /* Init SPI */
    SPI_Init();

    printf("\n\n");
    printf("+--------------------------------------------------------------+\n");
    printf("|                    SPI + PDMA Sample Code                    |\n");
    printf("+--------------------------------------------------------------+\n");
    printf("\n");
    printf("Configure QSPI0 as a master and SPI1 as a slave.\n");
    printf("Bit length of a transaction: 32\n");
    printf("The I/O connection for QSPI0/SPI1 loopback:\n");
    printf("    QSPI0_SS   (PA3) <--> SPI1_SS  (PH7)\n    QSPI0_CLK  (PA2) <--> SPI1_CLK (PH6)\n");
    printf("    QSPI0_MISO0(PA1) <--> SPI1_MISO(PH4)\n    QSPI0_MOSI0(PA0) <--> SPI1_MOSI(PH5)\n\n");
    printf("Please connect QSPI0 with SPI1, and press any key to start transmission ...");
    getchar();
    printf("\n");

    SpiLoopTest_WithPDMA();

    printf("\n\nExit SPI driver sample code.\n");

    /* Close QSPI0 */
    QSPI_Close(QSPI0);

    /* Close SPI1 */
    SPI_Close(SPI1);

    while(1);
}
