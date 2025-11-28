/*
 * This file is part of the Serial Flash Universal Driver Library.
 *
 * Copyright (c) 2016-2018, Armink, <armink.ztl@gmail.com>
 * Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
 
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * 'Software'), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED 'AS IS', WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * Function: Portable interface for each platform.
 * Created on: 2016-04-23
 */

#include "stdarg.h"
#include "NuMicro.h"
#include "sfud.h"

// *** <<< Use Configuration Wizard in Context Menu >>> ***
// <o> GPIO Slew Rate Control
// <0=> Normal <1=> High <2=> Fast
#define SlewRateMode    1
// *** <<< end of configuration section >>> ***

static char log_buf[256];
#define SPI_FLASH_PORT  SPI0
#define QSPI_FLASH_PORT  QSPI0
#define SPIM_FLASH_PORT  SPIM
#define CHAR_BIT_SIZE    8
void sfud_log_debug(const char *file, const long line, const char *format, ...);

uint8_t SpiFlash_ReadStatusReg(void);
uint8_t SpiFlash_ReadStatusReg2(void);
void SpiFlash_WriteStatusReg(uint8_t u8Value1, uint8_t u8Value2);
int32_t SpiFlash_WaitReady(void);
void SpiFlash_EnableQEBit(void);
void SpiFlash_DisableQEBit(void);
void SpiFlash_QuadFastRead(uint32_t u32StartAddress, uint8_t *u8DataBuffer);

static void Spi_Ss_Low(void* spi);
static void Spi_Ss_High(void* spi);
static void Spi_Write_Tx(void *spi, uint8_t data);
static uint8_t Spi_Read_Rx(void *spi);
bool Spi_Is_Busy(void *spi);
bool Spi_Tx_Fifo_empty(void *spi);

static void Qspi_Ss_Low(void* spi);
static void Qspi_Ss_High(void* spi);
static void Qspi_Write_Tx(void *spi, uint8_t data);
static uint8_t Qspi_Read_Rx(void *spi);
bool Qspi_Is_Busy(void *spi);
bool Qspi_Tx_Fifo_empty(void *spi);

void Spim_Write_Tx(void *spi, uint8_t data);
uint8_t Spim_Read_Rx(void *spi);
void Spim_Ss_Low(void* spi);
void Spim_Ss_High(void* spi);
bool Spim_Is_Busy(void *spi);
void Spim_Switch_Output(void *spi);
void Spim_Switch_Input(void *spi);

static void spi_lock(const sfud_spi *spi)
{
    __disable_irq();
}

static void spi_unlock(const sfud_spi *spi)
{
    __enable_irq();
}

__STATIC_INLINE void wait_Qspi_IS_Busy(QSPI_T *qspi)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    while(QSPI_IS_BUSY(qspi))
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for QSPI time-out!\n");
            break;
        }
    }
}
/**
 * SPI write data then read data
 */
static sfud_err spi_write_read(const sfud_spi *spi, const uint8_t *write_buf, size_t write_size, uint8_t *read_buf,
        size_t read_size)
{
    sfud_err result = SFUD_SUCCESS;
    uint8_t send_data, read_data;

    if (write_size)
    {
        SFUD_ASSERT(write_buf);
    }
    if (read_size)
    {
        SFUD_ASSERT(read_buf);
    }
    /**
     * add your spi write and read code
     */
    // /CS: active
    spi->ss_low(spi->user_module);

    /* Start reading and writing data */
    for (size_t i = 0, retry_times; i < write_size + read_size; i++)
    {
         /* First write the data from the buffer to the SPI bus; after writing, send a dummy (0xFF) to the SPI bus */
        if (i < write_size)
        {
            send_data = *write_buf++;
            if(spi->sel_dir_out)
            {
                spi->sel_dir_out(spi->user_module);
            }
        }
        else
        {
            send_data = SFUD_DUMMY_DATA;
            if(spi->sel_dir_in)
            {
                spi->sel_dir_in(spi->user_module);
            }
        }
        /* Send data */
        retry_times = 1000;

        while (spi->isbusy(spi->user_module))
        {
            SFUD_RETRY_PROCESS(NULL, retry_times, result);
        }
        if (result != SFUD_SUCCESS)
        {
            goto exit;
        }
        spi->tx(spi->user_module, send_data);
        /* Receive data */
        retry_times = 1000;

        while (spi->isbusy(spi->user_module))
        {
            SFUD_RETRY_PROCESS(NULL, retry_times, result);
        }
        if (result != SFUD_SUCCESS)
        {
            goto exit;
        }
        read_data = spi->rx(spi->user_module);

        /* After sending the data from the write buffer, read the data from the SPI bus into the read buffer */
        if (i >= write_size)
        {
            *read_buf++ = read_data;
        }
    }

exit:
    // /CS: de-active
    spi->ss_high(spi->user_module);

    return result;
}

#ifdef SFUD_USING_QSPI
/**
 * read flash data by QSPI
 */
static sfud_err qspi_read(const struct __sfud_spi *spi, uint32_t addr, sfud_qspi_read_cmd_format *qspi_read_cmd_format,
        uint8_t *read_buf, size_t read_size)
{
    sfud_err result = SFUD_SUCCESS;
    uint32_t u32Cnt;

    // enable quad mode
    SpiFlash_EnableQEBit();

    // /CS: active
    QSPI_SET_SS_LOW((QSPI_T*)spi->user_module);

    // Command: 0xEB, Fast Read quad data
    QSPI_WRITE_TX(QSPI_FLASH_PORT, qspi_read_cmd_format->instruction);
    wait_Qspi_IS_Busy((QSPI_T*)spi->user_module);

    // enable SPI quad IO mode and set direction
    QSPI_ENABLE_QUAD_OUTPUT_MODE((QSPI_T*)spi->user_module);

    for( u32Cnt = 1; u32Cnt <= qspi_read_cmd_format->dummy_cycles; u32Cnt++)
    {
        if( (u32Cnt * CHAR_BIT_SIZE) <= qspi_read_cmd_format->address_size )
        {
            // send start address
            QSPI_WRITE_TX((QSPI_T*)spi->user_module, (addr >> ( qspi_read_cmd_format->address_size - (u32Cnt * CHAR_BIT_SIZE))) & 0xFF);
        }
        else
        {
            // dummy byte
            QSPI_WRITE_TX((QSPI_T*)spi->user_module, 0x00);
        }
    }

    wait_Qspi_IS_Busy((QSPI_T*)spi->user_module);
    QSPI_ENABLE_QUAD_INPUT_MODE((QSPI_T*)spi->user_module);

    // clear RX buffer
    QSPI_ClearRxFIFO((QSPI_T*)spi->user_module);

    // read data
    for(u32Cnt = 0; u32Cnt < read_size; u32Cnt++)
    {
        QSPI_WRITE_TX((QSPI_T*)spi->user_module, 0x00);
        wait_Qspi_IS_Busy((QSPI_T*)spi->user_module);
        read_buf[u32Cnt] = (uint8_t)QSPI_READ_RX((QSPI_T*)spi->user_module);
    }

    // wait tx finish
    wait_Qspi_IS_Busy((QSPI_T*)spi->user_module);

    // /CS: de-active
    QSPI_SET_SS_HIGH((QSPI_T*)spi->user_module);

    QSPI_DISABLE_QUAD_MODE((QSPI_T*)spi->user_module);

    // disable quad mode
    SpiFlash_DisableQEBit();
    
    /**
     * add your qspi read flash data code
     */

    return result;
}

static sfud_err qspi_read_spim(const struct __sfud_spi *spi, uint32_t addr, sfud_qspi_read_cmd_format *qspi_read_cmd_format,
        uint8_t *read_buf, size_t read_size)
{
    sfud_err result = SFUD_SUCCESS;
    // enable quad mode
    SPIM_SetQuadEnable(1, 1UL);
#ifndef ENABLE_SPIM_DMA_READ
    SPIM_IO_Read(addr, qspi_read_cmd_format->address_size > 24, read_size, read_buf, 0xEB, 1, 4, 4, qspi_read_cmd_format->dummy_cycles - (qspi_read_cmd_format->address_size / CHAR_BIT_SIZE));
#else
    SPIM_SET_DCNUM(4);
    SPIM_DMA_Read(addr, 0, read_size, read_buf, CMD_DMA_FAST_QUAD_READ, 1);
#endif
    return result;
}
#endif /* SFUD_USING_QSPI */
/* about 100 microsecond delay */
static void retry_delay_100us(void)
{
    CLK_SysTickDelay(100);
}

sfud_err sfud_spi_port_init(sfud_flash *flash)
{
    sfud_err result = SFUD_SUCCESS;

    switch (flash->index)
    {
        case SFUD_W25Q16BV_DEVICE_INDEX:

            SYS_UnlockReg();
            /* Enable GPIOD clock */
            CLK->AHBCLK0 |= CLK_AHBCLK0_GPDCKEN_Msk;
            /* Enable SPI0 module clock */
            CLK_EnableModuleClock(SPI0_MODULE);
            /* Select SPI0 module clock source as PCLK1 */
            CLK_SetModuleClock(SPI0_MODULE, CLK_CLKSEL2_SPI0SEL_PCLK1, MODULE_NoMsk);
            /* Setup SPI0 multi-function pins */

            SET_SPI0_MOSI_PD0();
            SET_SPI0_MISO_PD1();
            SET_SPI0_CLK_PD2();
            SET_SPI0_SS_PD3();
            /* Enable SPI0 clock pin (PD2) schmitt trigger */
            PD->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

#if (SlewRateMode == 0)
            /* Enable SPI0 I/O normal slew rate */
            GPIO_SetSlewCtl(PD, BIT0 | BIT1 | BIT2 | BIT3, GPIO_SLEWCTL_NORMAL);
#elif (SlewRateMode == 1)
            /* Enable SPI0 I/O high slew rate */
            GPIO_SetSlewCtl(PD, BIT0 | BIT1 | BIT2 | BIT3, GPIO_SLEWCTL_HIGH);
#elif (SlewRateMode == 2)
            /* Enable SPI0 I/O fast slew rate */
            GPIO_SetSlewCtl(PD, BIT0 | BIT1 | BIT2 | BIT3, GPIO_SLEWCTL_FAST);
#endif

            /* Configure SPI_FLASH_PORT as a master, MSB first, 8-bit transaction, SPI Mode-0 timing, clock is 2MHz */
            SPI_Open(SPI0, SPI_MASTER, SPI_MODE_0, 8, 2000000);

            /* Disable auto SS function, control SS signal manually. */
            SPI_DisableAutoSS(SPI_FLASH_PORT);

            SYS_LockReg();

            flash->spi.wr = spi_write_read;
            flash->spi.lock = spi_lock;
            flash->spi.unlock = spi_unlock;

            flash->spi.user_module = (void*)SPI_FLASH_PORT;
            flash->spi.ss_low = Spi_Ss_Low;
            flash->spi.ss_high = Spi_Ss_High;
            flash->spi.tx = Spi_Write_Tx;
            flash->spi.rx = Spi_Read_Rx;
            flash->spi.isbusy = Spi_Is_Busy;
            flash->spi.tx_fifo_empty = Spi_Tx_Fifo_empty;
            flash->retry.delay = retry_delay_100us;
            flash->retry.times = 1000;//100ms timeout
            break;

        case SFUD_W25X16VS_DEVICE_INDEX:

            SYS_UnlockReg();
            /* Enable GPIOA clock */
            CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk;

            /* Enable QSPI0 module clock */
            CLK_EnableModuleClock(QSPI0_MODULE);

            /* Select QSPI0 module clock source as PCLK0 */
            CLK_SetModuleClock(QSPI0_MODULE, CLK_CLKSEL2_QSPI0SEL_PCLK0, MODULE_NoMsk);

            /* Setup QSPI0 multi-function pins */
            SET_QSPI0_MOSI0_PA0();
            SET_QSPI0_MISO0_PA1();
            SET_QSPI0_CLK_PA2();
            SET_QSPI0_SS_PA3();
            SET_QSPI0_MOSI1_PA4();
            SET_QSPI0_MISO1_PA5();

            /* Enable QSPI0 clock pin (PA2) schmitt trigger */
            PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk;

#if (SlewRateMode == 0)
            /* Enable QSPI0 I/O normal slew rate */
            GPIO_SetSlewCtl(PA, BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5, GPIO_SLEWCTL_NORMAL);
#elif (SlewRateMode == 1)
            /* Enable QSPI0 I/O high slew rate */
            GPIO_SetSlewCtl(PA, BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5, GPIO_SLEWCTL_HIGH);
#elif (SlewRateMode == 2)
            /* Enable QSPI0 I/O fast slew rate */
            GPIO_SetSlewCtl(PA, BIT0 | BIT1 | BIT2 | BIT3 | BIT4 | BIT5, GPIO_SLEWCTL_FAST);
#endif
            /* Configure SPI_FLASH_PORT as a master, MSB first, 8-bit transaction, QSPI Mode-0 timing, clock is 2MHz */
            QSPI_Open(QSPI_FLASH_PORT, QSPI_MASTER, QSPI_MODE_0, 8, 2000000);

            /* Disable auto SS function, control SS signal manually. */
            QSPI_DisableAutoSS(QSPI_FLASH_PORT);

            SYS_LockReg();

            /* set the interfaces and data */
            flash->spi.wr = spi_write_read;
#ifdef SFUD_USING_QSPI
            flash->spi.qspi_read = qspi_read;
#endif
            flash->spi.lock = spi_lock;
            flash->spi.unlock = spi_unlock;

            flash->spi.user_module = (void*)QSPI_FLASH_PORT;
            flash->spi.ss_low = Qspi_Ss_Low;
            flash->spi.ss_high = Qspi_Ss_High;
            flash->spi.tx = Qspi_Write_Tx;
            flash->spi.rx = Qspi_Read_Rx;
            flash->spi.isbusy = Qspi_Is_Busy;
            flash->spi.tx_fifo_empty = Qspi_Tx_Fifo_empty;
            flash->retry.delay = retry_delay_100us;
            flash->retry.times = 1000;//100ms timeout
            break;

        case SFUD_W25Q32JV_DEVICE_INDEX:

            SYS_UnlockReg();
            /* Enable GPIOI,J clock */
            CLK->AHBCLK1 |= CLK_AHBCLK1_GPICKEN_Msk | CLK_AHBCLK1_GPJCKEN_Msk;
            /* Enable SPIM module clock */
            CLK_EnableModuleClock(SPIM_MODULE);
            /* Init SPIM multi-function pins, MOSI(PJ.1), MISO(PI.13), CLK(PJ.0), SS(PI.12), D3(PI.15), and D2(PI.14) */
            SET_SPIM_MOSI_PJ1();
            SET_SPIM_MISO_PI13();
            SET_SPIM_CLK_PJ0();
            SET_SPIM_SS_PI12();
            SET_SPIM_D3_PI15();
            SET_SPIM_D2_PI14();

#if (SlewRateMode == 0)
            /* Enable SPIM I/O normal slew rate */
            GPIO_SetSlewCtl(PI, BIT12 | BIT13 | BIT14 | BIT15, GPIO_SLEWCTL_NORMAL);
            GPIO_SetSlewCtl(PJ, BIT0 | BIT1, GPIO_SLEWCTL_NORMAL);
#elif (SlewRateMode == 1)
            /* Enable SPIM I/O high slew rate */
            GPIO_SetSlewCtl(PI, BIT12 | BIT13 | BIT14 | BIT15, GPIO_SLEWCTL_HIGH);
            GPIO_SetSlewCtl(PJ, BIT0 | BIT1, GPIO_SLEWCTL_HIGH);
#elif (SlewRateMode == 2)
            /* Enable SPIM I/O fast slew rate */
            GPIO_SetSlewCtl(PI, BIT12 | BIT13 | BIT14 | BIT15, GPIO_SLEWCTL_FAST);
            GPIO_SetSlewCtl(PJ, BIT0 | BIT1, GPIO_SLEWCTL_FAST);
#endif
            SYS_LockReg();

            SPIM_SET_CLOCK_DIVIDER(15);       /* Set SPIM clock as HCLK divided by 16 */
            SPIM_SET_RXCLKDLY_RDDLYSEL(0);    /* Insert 0 delay cycle. Adjust the sampling clock of received data to latch the correct data. */
            SPIM_SET_RXCLKDLY_RDEDGE();       /* Use SPI input clock rising edge to sample received data. */
            SPIM_SET_DCNUM(8);                /* Set 8 dummy cycle. */

            /* set the interfaces and data */
            flash->spi.wr = spi_write_read;
#ifdef SFUD_USING_QSPI
            flash->spi.qspi_read = qspi_read_spim;
#endif
            flash->spi.lock = spi_lock;
            flash->spi.unlock = spi_unlock;

            flash->spi.user_module = (void*)SPIM_FLASH_PORT;
            flash->spi.ss_low = Spim_Ss_Low;
            flash->spi.ss_high = Spim_Ss_High;
            flash->spi.sel_dir_out = Spim_Switch_Output;
            flash->spi.sel_dir_in = Spim_Switch_Input;
            flash->spi.tx = Spim_Write_Tx;
            flash->spi.rx = Spim_Read_Rx;
            flash->spi.isbusy = Spim_Is_Busy;
            flash->retry.delay = retry_delay_100us;
            flash->retry.times = 1000;//100ms timeout
            break;

        default:
            result = SFUD_ERR_NOT_FOUND;
            break;
    }
    return result;
}

/**
 * This function is print debug info.
 *
 * @param file the file which has call this function
 * @param line the line number which has call this function
 * @param format output format
 * @param ... args
 */
void sfud_log_debug(const char *file, const long line, const char *format, ...)
{
    va_list args;

    /* args point to the first variable parameter */
    va_start(args, format);
    printf("[SFUD](%s:%ld) ", file, line);
    /* must use vprintf to print */
    vsnprintf(log_buf, sizeof(log_buf), format, args);
    printf("%s\n", log_buf);
    va_end(args);
}

/**
 * This function is print routine info.
 *
 * @param format output format
 * @param ... args
 */
void sfud_log_info(const char *format, ...)
{
    va_list args;

    /* args point to the first variable parameter */
    va_start(args, format);
    printf("[SFUD]");
    /* must use vprintf to print */
    vsnprintf(log_buf, sizeof(log_buf), format, args);
    printf("%s\n", log_buf);
    va_end(args);
}

uint8_t SpiFlash_ReadStatusReg(void)
{
    uint8_t u8Val;

    QSPI_ClearRxFIFO(QSPI_FLASH_PORT);

    // /CS: active
    QSPI_SET_SS_LOW(QSPI_FLASH_PORT);

    // send Command: 0x05, Read status register
    QSPI_WRITE_TX(QSPI_FLASH_PORT, 0x05);

    // read status
    QSPI_WRITE_TX(QSPI_FLASH_PORT, 0x00);

    // wait tx finish
    wait_Qspi_IS_Busy(QSPI_FLASH_PORT);

    // /CS: de-active
    QSPI_SET_SS_HIGH(QSPI_FLASH_PORT);

    // skip first rx data
    u8Val = (uint8_t)QSPI_READ_RX(QSPI_FLASH_PORT);
    u8Val = (uint8_t)QSPI_READ_RX(QSPI_FLASH_PORT);

    return u8Val;
}

uint8_t SpiFlash_ReadStatusReg2(void)
{
    uint8_t u8Val;

    QSPI_ClearRxFIFO(QSPI_FLASH_PORT);

    // /CS: active
    QSPI_SET_SS_LOW(QSPI_FLASH_PORT);

    // send Command: 0x35, Read status register
    QSPI_WRITE_TX(QSPI_FLASH_PORT, 0x35);

    // read status
    QSPI_WRITE_TX(QSPI_FLASH_PORT, 0x00);

    // wait tx finish
    wait_Qspi_IS_Busy(QSPI_FLASH_PORT);

    // /CS: de-active
    QSPI_SET_SS_HIGH(QSPI_FLASH_PORT);

    // skip first rx data
    u8Val = (uint8_t)QSPI_READ_RX(QSPI_FLASH_PORT);
    u8Val = (uint8_t)QSPI_READ_RX(QSPI_FLASH_PORT);

    return u8Val;
}

void SpiFlash_WriteStatusReg(uint8_t u8Value1, uint8_t u8Value2)
{
    // /CS: active
    QSPI_SET_SS_LOW(QSPI_FLASH_PORT);

    // send Command: 0x06, Write enable
    QSPI_WRITE_TX(QSPI_FLASH_PORT, 0x06);

    // wait tx finish
    wait_Qspi_IS_Busy(QSPI_FLASH_PORT);

    // /CS: de-active
    QSPI_SET_SS_HIGH(QSPI_FLASH_PORT);

    ///////////////////////////////////////

    // /CS: active
    QSPI_SET_SS_LOW(QSPI_FLASH_PORT);

    // send Command: 0x01, Write status register
    QSPI_WRITE_TX(QSPI_FLASH_PORT, 0x01);

    // write status
    QSPI_WRITE_TX(QSPI_FLASH_PORT, u8Value1);
    QSPI_WRITE_TX(QSPI_FLASH_PORT, u8Value2);

    // wait tx finish
    wait_Qspi_IS_Busy(QSPI_FLASH_PORT);

    // /CS: de-active
    QSPI_SET_SS_HIGH(QSPI_FLASH_PORT);
}

int32_t SpiFlash_WaitReady(void)
{
    volatile uint8_t u8ReturnValue;
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    do
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for QSPI time-out!\n");
            return -1;
        }

        u8ReturnValue = SpiFlash_ReadStatusReg();
        u8ReturnValue = u8ReturnValue & 1;
    }
    while(u8ReturnValue != 0); // check the BUSY bit

    return 0;
}

void SpiFlash_EnableQEBit(void)
{
    uint8_t u8Status1 = SpiFlash_ReadStatusReg();
    uint8_t u8Status2 = SpiFlash_ReadStatusReg2();

    u8Status2 |= 0x2;
    SpiFlash_WriteStatusReg(u8Status1, u8Status2);
    SpiFlash_WaitReady();
}

void SpiFlash_DisableQEBit(void)
{
    uint8_t u8Status1 = SpiFlash_ReadStatusReg();
    uint8_t u8Status2 = SpiFlash_ReadStatusReg2();

    u8Status2 &= ~0x2;
    SpiFlash_WriteStatusReg(u8Status1, u8Status2);
    SpiFlash_WaitReady();
}

void Spi_Ss_Low(void* spi)
{
    SPI_SET_SS_LOW((SPI_T*)spi);
}

void Spi_Ss_High(void* spi)
{
    SPI_SET_SS_HIGH((SPI_T*)spi);
}

void Spi_Write_Tx(void *spi, uint8_t data)
{
    SPI_WRITE_TX((SPI_T*)spi, data);
}

uint8_t Spi_Read_Rx(void *spi)
{
    return SPI_READ_RX((SPI_T*)spi);
}

bool Spi_Is_Busy(void *spi)
{
    return SPI_IS_BUSY((SPI_T*)spi);
}

bool Spi_Tx_Fifo_empty(void *spi)
{
    return SPI_GET_TX_FIFO_EMPTY_FLAG((SPI_T*)spi);
}

void Qspi_Ss_Low(void* spi)
{
    QSPI_SET_SS_LOW((QSPI_T*)spi);
}
void Qspi_Ss_High(void* spi)
{
    QSPI_SET_SS_HIGH((QSPI_T*)spi);
}

void Qspi_Write_Tx(void *spi, uint8_t data)
{
    QSPI_WRITE_TX((QSPI_T*)spi, data);
}

uint8_t Qspi_Read_Rx(void *spi)
{
    return QSPI_READ_RX((QSPI_T*)spi);
}

bool Qspi_Is_Busy(void *spi)
{
    return QSPI_IS_BUSY((QSPI_T*)spi);
}

bool Qspi_Tx_Fifo_empty(void *spi)
{
    return QSPI_GET_TX_FIFO_EMPTY_FLAG((QSPI_T*)spi);
}

void Spim_Write_Tx(void *spi, uint8_t data)
{
    SPIM->TX[0] = data;
    SPIM_SET_OPMODE(SPIM_CTL0_OPMODE_IO);    /* Switch to Normal mode. */
    SPIM_SET_DATA_WIDTH(1 * 8UL);
    SPIM_SET_DATA_NUM(1UL);
    SPIM_SET_GO();
    SPIM_WAIT_FREE();
}

uint8_t Spim_Read_Rx(void *spi)
{
    return SPIM->RX[0];
}

void Spim_Ss_High(void* spi)
{
    SPIM_SET_SS_EN(0);
}

void Spim_Ss_Low(void* spi)
{
    SPIM_SET_SS_EN(1);
}

bool Spim_Is_Busy(void *spi)
{
    return (bool)SPIM_IS_BUSY();
}

void Spim_Switch_Output(void *spi)
{
    SPIM_ENABLE_SING_OUTPUT_MODE();     /* 1-bit, Output. */
}

void Spim_Switch_Input(void *spi)
{
    SPIM_ENABLE_SING_INPUT_MODE();      /* 1-bit, Input.  */
}