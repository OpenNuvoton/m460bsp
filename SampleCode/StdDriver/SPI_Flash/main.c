/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Access SPI flash through SPI interface.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "string.h"
#include "NuMicro.h"

// *** <<< Use Configuration Wizard in Context Menu >>> ***
// <o> GPIO Slew Rate Control
// <0=> Normal <1=> High <2=> Fast
#define SlewRateMode    1
// <c1> Enable SPI Optimize
// <i> Use FIFO mechanism for QSPI RX to maximize throughput
#define ENABLE_SPI_OPTIMIZE
// </c>
// *** <<< end of configuration section >>> ***

#define TEST_NUMBER     1   /* page numbers */
#define TEST_LENGTH     256 /* length */

#define SPI_FLASH_PORT  SPI0

static uint8_t s_au8SrcArray[TEST_LENGTH];
static uint8_t s_au8DestArray[TEST_LENGTH];

uint32_t SpiFlash_ReadJedecID(void);
void SpiFlash_ChipErase(void);
uint8_t SpiFlash_ReadStatusReg(void);
void SpiFlash_WriteStatusReg(uint8_t u8Value);
int32_t SpiFlash_WaitReady(void);
void SpiFlash_NormalPageProgram(uint32_t u32StartAddress, uint8_t *u8DataBuffer);
void SpiFlash_NormalRead(uint32_t u32StartAddress, uint8_t *u8DataBuffer);
void SYS_Init(void);

__STATIC_INLINE void wait_SPI_IS_BUSY(SPI_T *spi)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while(SPI_IS_BUSY(spi))
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for SPI time-out!\n");
            break;
        }
    }
}

uint32_t SpiFlash_ReadJedecID(void)
{
    uint8_t u8RxData[4], u8IDCnt = 0;

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x9F, Read JEDEC ID
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x9F);

    // receive 32-bit
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    while(!SPI_GET_RX_FIFO_EMPTY_FLAG(SPI_FLASH_PORT))
        u8RxData[u8IDCnt++] = (uint8_t)SPI_READ_RX(SPI_FLASH_PORT);

    return (uint32_t)((u8RxData[1] << 16) | (u8RxData[2] << 8) | u8RxData[3]);
}

void SpiFlash_ChipErase(void)
{
    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x06);

    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    //////////////////////////////////////////

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0xC7, Chip Erase
    SPI_WRITE_TX(SPI_FLASH_PORT, 0xC7);

    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    SPI_ClearRxFIFO(SPI_FLASH_PORT);
}

uint8_t SpiFlash_ReadStatusReg(void)
{
    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x05, Read status register
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x05);

    // read status
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);

    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    // skip first rx data
    SPI_READ_RX(SPI_FLASH_PORT);

    return (SPI_READ_RX(SPI_FLASH_PORT) & 0xff);
}

void SpiFlash_WriteStatusReg(uint8_t u8Value)
{
    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x06);

    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    ///////////////////////////////////////

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x01, Write status register
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x01);

    // write status
    SPI_WRITE_TX(SPI_FLASH_PORT, u8Value);

    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);
}

int32_t SpiFlash_WaitReady(void)
{
    uint8_t u8ReturnValue;
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    do
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for SPI time-out!\n");
            return -1;
        }

        u8ReturnValue = SpiFlash_ReadStatusReg();
        u8ReturnValue = u8ReturnValue & 1;
    }
    while(u8ReturnValue != 0); // check the BUSY bit

    return 0;
}

void SpiFlash_NormalPageProgram(uint32_t u32StartAddress, uint8_t *u8DataBuffer)
{
    uint32_t u32Cnt = 0;

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x06);

    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);


    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x02, Page program
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x02);

    // send 24-bit start address
    SPI_WRITE_TX(SPI_FLASH_PORT, (u32StartAddress >> 16) & 0xFF);
    SPI_WRITE_TX(SPI_FLASH_PORT, (u32StartAddress >> 8)  & 0xFF);
    SPI_WRITE_TX(SPI_FLASH_PORT, u32StartAddress       & 0xFF);

    // write data
    while(1)
    {
        if(!SPI_GET_TX_FIFO_FULL_FLAG(SPI_FLASH_PORT))
        {
            SPI_WRITE_TX(SPI_FLASH_PORT, u8DataBuffer[u32Cnt++]);
            if(u32Cnt > 255) break;
        }
    }

    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);

    SPI_ClearRxFIFO(SPI_FLASH_PORT);
}

void SpiFlash_NormalRead(uint32_t u32StartAddress, uint8_t *u8DataBuffer)
{
#ifndef ENABLE_SPI_OPTIMIZE
    uint32_t u32Cnt;
#else
    uint32_t u32RxDataWord = 64;
    uint32_t u32RxTmp;
    uint32_t u32TxDataCount = 0;
    uint32_t u32RxDataCount = 0;
#endif

    // /CS: active
    SPI_SET_SS_LOW(SPI_FLASH_PORT);

    // send Command: 0x03, Read data
    SPI_WRITE_TX(SPI_FLASH_PORT, 0x03);

    // send 24-bit start address
    SPI_WRITE_TX(SPI_FLASH_PORT, (u32StartAddress >> 16) & 0xFF);
    SPI_WRITE_TX(SPI_FLASH_PORT, (u32StartAddress >> 8)  & 0xFF);
    SPI_WRITE_TX(SPI_FLASH_PORT, u32StartAddress       & 0xFF);

    wait_SPI_IS_BUSY(SPI_FLASH_PORT);
    // clear RX buffer
    SPI_ClearRxFIFO(SPI_FLASH_PORT);
#ifdef ENABLE_SPI_OPTIMIZE
    SPI_SET_DATA_WIDTH(SPI_FLASH_PORT, 32);
    SPI_ENABLE_BYTE_REORDER(SPI_FLASH_PORT);

    while(u32RxDataCount < 256)
    {
        /* Check TX FULL flag */
        if((SPI_GET_TX_FIFO_FULL_FLAG(SPI_FLASH_PORT) == 0) && (u32TxDataCount < u32RxDataWord))
        {
            SPI_WRITE_TX(SPI_FLASH_PORT, 0xFFFFFFFF);
            u32TxDataCount++;
        }
        /* Check RX EMPTY flag */
        if(SPI_GET_RX_FIFO_EMPTY_FLAG(SPI_FLASH_PORT) == 0)
        {
            u32RxTmp = QSPI_READ_RX(SPI_FLASH_PORT);
            memcpy(&u8DataBuffer[u32RxDataCount], &u32RxTmp, 4);
            u32RxDataCount+=4;
        }
    }

    SPI_DISABLE_BYTE_REORDER(SPI_FLASH_PORT);
    SPI_SET_DATA_WIDTH(SPI_FLASH_PORT, 8);
#else
    // read data
    for(u32Cnt = 0; u32Cnt < 256; u32Cnt++)
    {
        SPI_WRITE_TX(SPI_FLASH_PORT, 0x00);
        wait_SPI_IS_BUSY(SPI_FLASH_PORT);
        u8DataBuffer[u32Cnt] = (uint8_t)SPI_READ_RX(SPI_FLASH_PORT);
    }
#endif
    // wait tx finish
    wait_SPI_IS_BUSY(SPI_FLASH_PORT);

    // /CS: de-active
    SPI_SET_SS_HIGH(SPI_FLASH_PORT);
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

    /* Enable SPI0 clock pin (PA2) schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

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

/* Main */
int main(void)
{
    uint32_t u32ByteCount, u32FlashAddress, u32PageNumber;
    uint32_t u32Error = 0;
    uint32_t u32ID;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /* Configure SPI_FLASH_PORT as a master, MSB first, 8-bit transaction, SPI Mode-0 timing, clock is 2MHz */
    SPI_Open(SPI_FLASH_PORT, SPI_MASTER, SPI_MODE_0, 8, 2000000);

    /* Disable auto SS function, control SS signal manually. */
    SPI_DisableAutoSS(SPI_FLASH_PORT);

    printf("\n\n");
    printf("+-------------------------------------------------------------------------+\n");
    printf("|                        SPI Sample with SPI Flash                        |\n");
    printf("+-------------------------------------------------------------------------+\n");

    u32ID = SpiFlash_ReadJedecID();

    if(u32ID == 0xEF4014)
        printf("Flash found: W25Q80 ...\n");
    else if(u32ID == 0xEF4015)
        printf("Flash found: W25Q16 ...\n");
    else if(u32ID == 0xEF4016)
        printf("Flash found: W25Q32 ...\n");
    else if(u32ID == 0xEF4017)
        printf("Flash found: W25Q64 ...\n");
    else if(u32ID == 0xEF4018)
        printf("Flash found: W25Q128 ...\n");
    else
    {
        printf("Wrong ID, 0x%X\n", u32ID);
        goto lexit;
    }

    printf("Erase chip ...");

    /* Erase SPI flash */
    SpiFlash_ChipErase();

    /* Wait ready */
    if(SpiFlash_WaitReady() < 0) goto lexit;

    printf("[OK]\n");

    /* init source data buffer */
    for(u32ByteCount = 0; u32ByteCount < TEST_LENGTH; u32ByteCount++)
    {
        s_au8SrcArray[u32ByteCount] = (uint8_t)u32ByteCount;
    }

    printf("Start to normal write data to Flash ...");
    /* Program SPI flash */
    u32FlashAddress = 0;
    for(u32PageNumber = 0; u32PageNumber < TEST_NUMBER; u32PageNumber++)
    {
        /* page program */
        SpiFlash_NormalPageProgram(u32FlashAddress, s_au8SrcArray);
        if(SpiFlash_WaitReady() < 0) goto lexit;
        u32FlashAddress += 0x100;
    }

    printf("[OK]\n");

    /* clear destination data buffer */
    for(u32ByteCount = 0; u32ByteCount < TEST_LENGTH; u32ByteCount++)
    {
        s_au8DestArray[u32ByteCount] = 0;
    }

    printf("Normal Read & Compare ...");

    /* Read SPI flash */
    u32FlashAddress = 0;
    for(u32PageNumber = 0; u32PageNumber < TEST_NUMBER; u32PageNumber++)
    {
        /* page read */
        SpiFlash_NormalRead(u32FlashAddress, s_au8DestArray);
        u32FlashAddress += 0x100;

        for(u32ByteCount = 0; u32ByteCount < TEST_LENGTH; u32ByteCount++)
        {
            if(s_au8DestArray[u32ByteCount] != s_au8SrcArray[u32ByteCount])
                u32Error++;
        }
    }

    if(u32Error == 0)
        printf("[OK]\n");
    else
        printf("[FAIL]\n");

lexit:

    while(1);
}
