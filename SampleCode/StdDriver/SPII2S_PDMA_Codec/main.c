/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    This is an I2S demo with PDMA function connected with audio codec.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define NAU8822     1

/* Use LIN as source, undefine it if MIC is used */
//#define INPUT_IS_LIN

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define BUFF_LEN        512
#define BUFF_HALF_LEN   (BUFF_LEN/2)

typedef struct
{
    uint32_t CTL;
    uint32_t SA;
    uint32_t DA;
    uint32_t FIRST;
} DESC_TABLE_T;

static volatile uint8_t s_u8TxIdx = 0, s_u8RxIdx = 0;
static uint32_t s_au32PcmRxBuff[2][BUFF_LEN] = {{0}};
static uint32_t s_au32PcmTxBuff[2][BUFF_LEN] = {{0}};
static DESC_TABLE_T s_asDescTable_TX[2], s_asDescTable_RX[2];

#if NAU8822
void I2C_WriteNAU8822(uint8_t u8Addr, uint16_t u16Data);
void NAU8822_Setup(void);
#else
uint8_t I2C_WriteMultiByteforNAU88L25(uint8_t u8ChipAddr, uint16_t u16SubAddr, const uint8_t *p, uint32_t u32Len);
uint8_t I2C_WriteNAU88L25(uint16_t u16Addr, uint16_t u16Dat);
void NAU88L25_Reset(void);
void NAU88L25_Setup(void);
#endif
void SYS_Init(void);
void PDMA_Init(void);
void I2C2_Init(void);
void PDMA0_IRQHandler(void);

#if NAU8822

/*---------------------------------------------------------------------------------------------------------*/
/*  Write 9-bit data to 7-bit address register of NAU8822 with I2C2                                        */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_WriteNAU8822(uint8_t u8Addr, uint16_t u16Data)
{
    I2C_START(I2C2);
    I2C_WAIT_READY(I2C2);

    I2C_SET_DATA(I2C2, 0x1A << 1);
    I2C_SET_CONTROL_REG(I2C2, I2C_CTL_SI);
    I2C_WAIT_READY(I2C2);

    I2C_SET_DATA(I2C2, (uint8_t)((u8Addr << 1) | (u16Data >> 8)));
    I2C_SET_CONTROL_REG(I2C2, I2C_CTL_SI);
    I2C_WAIT_READY(I2C2);

    I2C_SET_DATA(I2C2, (uint8_t)(u16Data & 0x00FF));
    I2C_SET_CONTROL_REG(I2C2, I2C_CTL_SI);
    I2C_WAIT_READY(I2C2);

    I2C_STOP(I2C2);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  NAU8822 Settings with I2C interface                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void NAU8822_Setup(void)
{
    printf("\nConfigure NAU8822 ...");

    I2C_WriteNAU8822(0,  0x000);   /* Reset all registers */
    CLK_SysTickDelay(10000);

#ifdef INPUT_IS_LIN   /* Input source is LIN */
    I2C_WriteNAU8822(1,  0x02F);
    I2C_WriteNAU8822(2,  0x1B3);   /* Enable L/R Headphone, ADC Mix/Boost, ADC */
    I2C_WriteNAU8822(3,  0x07F);   /* Enable L/R main mixer, DAC */
    I2C_WriteNAU8822(4,  0x010);   /* 16-bit word length, I2S format, Stereo */
    I2C_WriteNAU8822(5,  0x000);   /* Companding control and loop back mode (all disable) */
    I2C_WriteNAU8822(6,  0x14D);   /* Divide by 2, 48K */
    I2C_WriteNAU8822(7,  0x000);   /* 48K for internal filter coefficients */
    I2C_WriteNAU8822(10, 0x008);   /* DAC soft mute is disabled, DAC oversampling rate is 128x */
    I2C_WriteNAU8822(14, 0x108);   /* ADC HP filter is disabled, ADC oversampling rate is 128x */
    I2C_WriteNAU8822(15, 0x1EF);   /* ADC left digital volume control */
    I2C_WriteNAU8822(16, 0x1EF);   /* ADC right digital volume control */

    I2C_WriteNAU8822(44, 0x000);   /* LLIN/RLIN is not connected to PGA */
    I2C_WriteNAU8822(47, 0x050);   /* LLIN connected, and its Gain value */
    I2C_WriteNAU8822(48, 0x050);   /* RLIN connected, and its Gain value */
    I2C_WriteNAU8822(50, 0x001);   /* Left DAC connected to LMIX */
    I2C_WriteNAU8822(51, 0x001);   /* Right DAC connected to RMIX */
#else   /* Input source is MIC */
    I2C_WriteNAU8822(1,  0x03F);
    I2C_WriteNAU8822(2,  0x1BF);   /* Enable L/R Headphone, ADC Mix/Boost, ADC */
    I2C_WriteNAU8822(3,  0x07F);   /* Enable L/R main mixer, DAC */
    I2C_WriteNAU8822(4,  0x010);   /* 16-bit word length, I2S format, Stereo */
    I2C_WriteNAU8822(5,  0x000);   /* Companding control and loop back mode (all disable) */
    I2C_WriteNAU8822(6,  0x14D);   /* Divide by 2, 48K */
    I2C_WriteNAU8822(7,  0x000);   /* 48K for internal filter coefficients */
    I2C_WriteNAU8822(10, 0x008);   /* DAC soft mute is disabled, DAC oversampling rate is 128x */
    I2C_WriteNAU8822(14, 0x108);   /* ADC HP filter is disabled, ADC oversampling rate is 128x */
    I2C_WriteNAU8822(15, 0x1EF);   /* ADC left digital volume control */
    I2C_WriteNAU8822(16, 0x1EF);   /* ADC right digital volume control */

    I2C_WriteNAU8822(44, 0x033);   /* LMICN/LMICP is connected to PGA */
    I2C_WriteNAU8822(50, 0x001);   /* Left DAC connected to LMIX */
    I2C_WriteNAU8822(51, 0x001);   /* Right DAC connected to RMIX */
#endif

    printf("[OK]\n");
}

#else   // NAU88L25

uint8_t I2C_WriteMultiByteforNAU88L25(uint8_t u8ChipAddr, uint16_t u16SubAddr, const uint8_t *p, uint32_t u32Len)
{
    (void)u32Len;

    /* Send START */
    I2C_START(I2C2);
    I2C_WAIT_READY(I2C2);

    /* Send device address */
    I2C_SET_DATA(I2C2, u8ChipAddr);
    I2C_SET_CONTROL_REG(I2C2, I2C_CTL_SI);
    I2C_WAIT_READY(I2C2);

    /* Send register number and MSB of data */
    I2C_SET_DATA(I2C2, (uint8_t)(u16SubAddr >> 8));
    I2C_SET_CONTROL_REG(I2C2, I2C_CTL_SI);
    I2C_WAIT_READY(I2C2);

    /* Send register number and MSB of data */
    I2C_SET_DATA(I2C2, (uint8_t)(u16SubAddr & 0x00FF));
    I2C_SET_CONTROL_REG(I2C2, I2C_CTL_SI);
    I2C_WAIT_READY(I2C2);

    /* Send data */
    I2C_SET_DATA(I2C2, p[0]);
    I2C_SET_CONTROL_REG(I2C2, I2C_CTL_SI);
    I2C_WAIT_READY(I2C2);

    /* Send data */
    I2C_SET_DATA(I2C2, p[1]);
    I2C_SET_CONTROL_REG(I2C2, I2C_CTL_SI);
    I2C_WAIT_READY(I2C2);

    /* Send STOP */
    I2C_STOP(I2C2);

    return  0;
}

uint8_t I2C_WriteNAU88L25(uint16_t u16Addr, uint16_t u16Dat)
{
    uint8_t u8TxData0[2];

    u8TxData0[0] = (uint8_t)(u16Dat >> 8);
    u8TxData0[1] = (uint8_t)(u16Dat & 0x00FF);

    return (I2C_WriteMultiByteforNAU88L25(0x1A << 1, u16Addr, &u8TxData0[0], 2));
}

void NAU88L25_Reset(void)
{
    I2C_WriteNAU88L25(0,  0x1);
    I2C_WriteNAU88L25(0,  0);   /* Reset all registers */
    CLK_SysTickDelay(10000);

    printf("NAU88L25 Software Reset.\n");
}

void NAU88L25_Setup(void)
{
    I2C_WriteNAU88L25(0x0003,  0x8053);
    I2C_WriteNAU88L25(0x0004,  0x0001);
    I2C_WriteNAU88L25(0x0005,  0x3126);
    I2C_WriteNAU88L25(0x0006,  0x0008);
    I2C_WriteNAU88L25(0x0007,  0x0010);
    I2C_WriteNAU88L25(0x0008,  0xC000);
    I2C_WriteNAU88L25(0x0009,  0x6000);
    I2C_WriteNAU88L25(0x000A,  0xF13C);
    I2C_WriteNAU88L25(0x000C,  0x0048);
    I2C_WriteNAU88L25(0x000D,  0x0000);
    I2C_WriteNAU88L25(0x000F,  0x0000);
    I2C_WriteNAU88L25(0x0010,  0x0000);
    I2C_WriteNAU88L25(0x0011,  0x0000);
    I2C_WriteNAU88L25(0x0012,  0xFFFF);
    I2C_WriteNAU88L25(0x0013,  0x0015);
    I2C_WriteNAU88L25(0x0014,  0x0110);
    I2C_WriteNAU88L25(0x0015,  0x0000);
    I2C_WriteNAU88L25(0x0016,  0x0000);
    I2C_WriteNAU88L25(0x0017,  0x0000);
    I2C_WriteNAU88L25(0x0018,  0x0000);
    I2C_WriteNAU88L25(0x0019,  0x0000);
    I2C_WriteNAU88L25(0x001A,  0x0000);
    I2C_WriteNAU88L25(0x001B,  0x0000);
    I2C_WriteNAU88L25(0x001C,  0x0002);
    I2C_WriteNAU88L25(0x001D,  0x301A);   /* 301A:Master, BCLK_DIV=12.288M/8=1.536M, LRC_DIV=1.536M/32=48K */
    I2C_WriteNAU88L25(0x001E,  0x0000);
    I2C_WriteNAU88L25(0x001F,  0x0000);
    I2C_WriteNAU88L25(0x0020,  0x0000);
    I2C_WriteNAU88L25(0x0021,  0x0000);
    I2C_WriteNAU88L25(0x0022,  0x0000);
    I2C_WriteNAU88L25(0x0023,  0x0000);
    I2C_WriteNAU88L25(0x0024,  0x0000);
    I2C_WriteNAU88L25(0x0025,  0x0000);
    I2C_WriteNAU88L25(0x0026,  0x0000);
    I2C_WriteNAU88L25(0x0027,  0x0000);
    I2C_WriteNAU88L25(0x0028,  0x0000);
    I2C_WriteNAU88L25(0x0029,  0x0000);
    I2C_WriteNAU88L25(0x002A,  0x0000);
    I2C_WriteNAU88L25(0x002B,  0x0012);
    I2C_WriteNAU88L25(0x002C,  0x0082);
    I2C_WriteNAU88L25(0x002D,  0x0000);
    I2C_WriteNAU88L25(0x0030,  0x00CF);
    I2C_WriteNAU88L25(0x0031,  0x0000);
    I2C_WriteNAU88L25(0x0032,  0x0000);
    I2C_WriteNAU88L25(0x0033,  0x00CF);
    I2C_WriteNAU88L25(0x0034,  0x02CF);
    I2C_WriteNAU88L25(0x0038,  0x1486);
    I2C_WriteNAU88L25(0x0039,  0x0F12);
    I2C_WriteNAU88L25(0x003A,  0x25FF);
    I2C_WriteNAU88L25(0x003B,  0x3457);
    I2C_WriteNAU88L25(0x0045,  0x1486);
    I2C_WriteNAU88L25(0x0046,  0x0F12);
    I2C_WriteNAU88L25(0x0047,  0x25F9);
    I2C_WriteNAU88L25(0x0048,  0x3457);
    I2C_WriteNAU88L25(0x004C,  0x0000);
    I2C_WriteNAU88L25(0x004D,  0x0000);
    I2C_WriteNAU88L25(0x004E,  0x0000);
    I2C_WriteNAU88L25(0x0050,  0x2007);
    I2C_WriteNAU88L25(0x0051,  0x0000);
    I2C_WriteNAU88L25(0x0053,  0xC201);
    I2C_WriteNAU88L25(0x0054,  0x0C95);
    I2C_WriteNAU88L25(0x0055,  0x0000);
    I2C_WriteNAU88L25(0x0058,  0x1A14);
    I2C_WriteNAU88L25(0x0059,  0x00FF);
    I2C_WriteNAU88L25(0x0066,  0x0060);
    I2C_WriteNAU88L25(0x0068,  0xC300);
    I2C_WriteNAU88L25(0x0069,  0x0000);
    I2C_WriteNAU88L25(0x006A,  0x0083);
    I2C_WriteNAU88L25(0x0071,  0x0011);
    I2C_WriteNAU88L25(0x0072,  0x0260);
    I2C_WriteNAU88L25(0x0073,  0x332C);
    I2C_WriteNAU88L25(0x0074,  0x4502);
    I2C_WriteNAU88L25(0x0076,  0x3140);
    I2C_WriteNAU88L25(0x0077,  0x0000);
    I2C_WriteNAU88L25(0x007F,  0x553F);
    I2C_WriteNAU88L25(0x0080,  0x0420);
    I2C_WriteNAU88L25(0x0001,  0x07D4);

    printf("NAU88L25 Configured done.\n");
}

#endif

void SYS_Init(void)
{
    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SET_XT1_OUT_PF2();
    SET_XT1_IN_PF3();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC and HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HIRC and HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_HXTSTB_Msk);

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

    /* Enable I2C2 module clock */
    CLK_EnableModuleClock(I2C2_MODULE);

    /* Enable PDMA0 module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Configure SPI0 related multi-function pins. */
    /* GPA[4:0] : SPI0_I2SMCLK (I2S0_I2SMCLK), SPI0_SS (I2S0_LRCLK), SPI0_CLK (I2S0_BCLK), SPI0_MISO (I2S0_DI), SPI0_MOSI (I2S0_DO). */
    SET_SPI0_MOSI_PA0();
    SET_SPI0_MISO_PA1();
    SET_SPI0_CLK_PA2();
    SET_SPI0_SS_PA3();
    SET_SPI0_I2SMCLK_PA4();

    /* Enable SPI0 clock pin (PA2) schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

    /* Set I2C2 multi-function pins */
    SET_I2C2_SDA_PD0();
    SET_I2C2_SCL_PD1();

    /* Enable I2C2 clock pin (PD1) schmitt trigger */
    PD->SMTEN |= GPIO_SMTEN_SMTEN1_Msk;
}

/* Configure PDMA to Scatter Gather mode */
void PDMA_Init(void)
{
    /* Tx description */
    s_asDescTable_TX[0].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    s_asDescTable_TX[0].SA = (uint32_t)&s_au32PcmTxBuff[0];
    s_asDescTable_TX[0].DA = (uint32_t)&SPI0->TX;
    s_asDescTable_TX[0].FIRST = (uint32_t)&s_asDescTable_TX[1] - (PDMA0->SCATBA);

    s_asDescTable_TX[1].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    s_asDescTable_TX[1].SA = (uint32_t)&s_au32PcmTxBuff[1];
    s_asDescTable_TX[1].DA = (uint32_t)&SPI0->TX;
    s_asDescTable_TX[1].FIRST = (uint32_t)&s_asDescTable_TX[0] - (PDMA0->SCATBA);   /* Link to first description */

    /* Rx description */
    s_asDescTable_RX[0].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    s_asDescTable_RX[0].SA = (uint32_t)&SPI0->RX;
    s_asDescTable_RX[0].DA = (uint32_t)&s_au32PcmRxBuff[0];
    s_asDescTable_RX[0].FIRST = (uint32_t)&s_asDescTable_RX[1] - (PDMA0->SCATBA);

    s_asDescTable_RX[1].CTL = ((BUFF_LEN - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_FIX | PDMA_DAR_INC | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    s_asDescTable_RX[1].SA = (uint32_t)&SPI0->RX;
    s_asDescTable_RX[1].DA = (uint32_t)&s_au32PcmRxBuff[1];
    s_asDescTable_RX[1].FIRST = (uint32_t)&s_asDescTable_RX[0] - (PDMA0->SCATBA);   /* Link to first description */

    /* Open PDMA channel 1 for SPI TX and channel 2 for SPI RX */
    PDMA_Open(PDMA0, 0x3 << 1);

    /* Configure PDMA transfer mode */
    PDMA_SetTransferMode(PDMA0, 1, PDMA_SPI0_TX, 1, (uint32_t)&s_asDescTable_TX[0]);
    PDMA_SetTransferMode(PDMA0, 2, PDMA_SPI0_RX, 1, (uint32_t)&s_asDescTable_RX[0]);

    /* Enable PDMA channel 1&2 interrupt */
    PDMA_EnableInt(PDMA0, 1, 0);
    PDMA_EnableInt(PDMA0, 2, 0);

    NVIC_EnableIRQ(PDMA0_IRQn);
}

/* Init I2C interface */
void I2C2_Init(void)
{
    /* Open I2C2 and set clock to 100k */
    I2C_Open(I2C2, 100000);

    /* Get I2C2 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C2));
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("+------------------------------------------------------------------------+\n");
    printf("|               SPII2S Driver Sample Code with audio codec               |\n");
    printf("+------------------------------------------------------------------------+\n");
    printf("  NOTE: This sample code needs to work with audio codec.\n");

    /* Init I2C2 to access codec */
    I2C2_Init();

#if (!NAU8822)
    /* Reset NAU88L25 codec */
    NAU88L25_Reset();
#endif

#ifdef INPUT_IS_LIN
    SPII2S_Open(SPI0, SPII2S_MODE_SLAVE, 48000, SPII2S_DATABIT_16, SPII2S_STEREO, SPII2S_FORMAT_I2S);
#else
    SPII2S_Open(SPI0, SPII2S_MODE_SLAVE, 48000, SPII2S_DATABIT_16, SPII2S_MONO, SPII2S_FORMAT_I2S);
#endif

    /* Set PD3 low to enable phone jack on NuMaker board. */
    SYS->GPD_MFP0 &= ~(SYS_GPD_MFP0_PD3MFP_Msk);
    GPIO_SetMode(PD, BIT3, GPIO_MODE_OUTPUT);
    PD3 = 0;

    /* Select source from HXT(12MHz) */
    CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_HXT, 0);

#if NAU8822
    /* Initialize NAU8822 codec */
    NAU8822_Setup();
#else
    SPI0->I2SCTL |= SPI_I2SCTL_ORDER_Msk;
    /* Initialize NAU88L25 codec */
    CLK_SysTickDelay(20000);
    NAU88L25_Setup();
#endif

    /* Set MCLK and enable MCLK */
    SPII2S_EnableMCLK(SPI0, 12000000);

#ifndef INPUT_IS_LIN
    /* NAU8822 will store data in left channel */
    SPII2S_SET_MONO_RX_CHANNEL(SPI0, SPII2S_MONO_LEFT);
#endif

    PDMA_Init();

    /* Enable I2S Rx function */
    SPII2S_ENABLE_RXDMA(SPI0);
    SPII2S_ENABLE_RX(SPI0);

    /* Enable I2S Tx function */
    SPII2S_ENABLE_TXDMA(SPI0);
    SPII2S_ENABLE_TX(SPI0);

    while(1);
}

void PDMA0_IRQHandler(void)
{
    uint32_t u32Status = PDMA_GET_INT_STATUS(PDMA0);

    if(u32Status & 0x1)    /* abort */
    {
        if(PDMA_GET_ABORT_STS(PDMA0) & 0x4)
            PDMA_CLR_ABORT_FLAG(PDMA0, PDMA_ABTSTS_ABTIF1_Msk);
        PDMA_CLR_ABORT_FLAG(PDMA0, PDMA_ABTSTS_ABTIF2_Msk);
    }
    else if(u32Status & 0x2)
    {
        if(PDMA_GET_TD_STS(PDMA0) & 0x4)              /* channel 2 done */
        {
            /* Copy RX data to TX buffer */
            memcpy(&s_au32PcmTxBuff[s_u8TxIdx ^ 1], &s_au32PcmRxBuff[s_u8RxIdx], BUFF_LEN * 4);
            s_u8RxIdx ^= 1;
            PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF2_Msk);
        }
        if(PDMA_GET_TD_STS(PDMA0) & 0x2)              /* channel 1 done */
        {
            s_u8TxIdx ^= 1;
            PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF1_Msk);
        }
    }
    else
        printf("unknown interrupt, status=0x%x!!\n", u32Status);
}
