/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate SPIM DMA read/write with cipher enabled. This sample
 *           also dumps SPI flash content via I/O mode read to prove it is encrypted cipher context.
 *
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"

// #define HCLK_192MHZ
#define MFP_SELECT                  0            /* Multi-function pin select */

#define SPIM_KEY_1                  0x391e9055   /* SPIM cipher key word 1. User defined. */
#define SPIM_KEY_2                  0xf15da090   /* SPIM cipher key word 2. User defined. */

#define FLASH_BLOCK_SIZE            (64*1024)    /* Flash block size. Depend on the physical flash. */
#define TEST_BLOCK_ADDR             0x10000      /* Test block address on SPI flash. */
#define BUFFER_SIZE                 2048

#ifdef __ICCARM__
#pragma data_alignment=4
uint8_t  g_buff[BUFFER_SIZE];
#else
uint8_t  g_buff[BUFFER_SIZE] __attribute__((aligned(4)));
#endif


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 80MHz */
    CLK_SetCoreClock(80000000);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;
    CLK->AHBCLK1 |= CLK_AHBCLK1_GPICKEN_Msk | CLK_AHBCLK1_GPJCKEN_Msk;

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable SPIM module clock */
    CLK_EnableModuleClock(SPIM_MODULE);
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

#if MFP_SELECT
    /* Init SPIM multi-function pins, MOSI(PE.2), MISO(PE.3), CLK(PE.4), SS(PE.5), D3(PE.6), and D2(PE.7) */
    SET_SPIM_MOSI_PE2();
    SET_SPIM_MISO_PE3();
    SET_SPIM_CLK_PE4();
    SET_SPIM_SS_PE5();
    SET_SPIM_D3_PE6();
    SET_SPIM_D2_PE7();

    PE->SMTEN |= GPIO_SMTEN_SMTEN4_Msk;

    /* Set SPIM I/O pins as high slew rate up to 80 MHz. */
    PE->SLEWCTL = (PE->SLEWCTL & 0xFFFF000F) |
                  (0x1<<GPIO_SLEWCTL_HSREN2_Pos) | (0x1<<GPIO_SLEWCTL_HSREN3_Pos) |
                  (0x1<<GPIO_SLEWCTL_HSREN4_Pos) | (0x1<<GPIO_SLEWCTL_HSREN5_Pos) |
                  (0x1<<GPIO_SLEWCTL_HSREN6_Pos) | (0x1<<GPIO_SLEWCTL_HSREN7_Pos);
#else
    /* Init SPIM multi-function pins, MOSI(PJ.1), MISO(PI.13), CLK(PJ.0), SS(PI.12), D3(PI.15), and D2(PI.14) */
    SET_SPIM_MOSI_PJ1();
    SET_SPIM_MISO_PI13();
    SET_SPIM_CLK_PJ0();
    SET_SPIM_SS_PI12();
    SET_SPIM_D3_PI15();
    SET_SPIM_D2_PI14();

    PJ->SMTEN |= GPIO_SMTEN_SMTEN0_Msk;

    /* Set SPIM I/O pins as high slew rate up to 80 MHz. */
    PI->SLEWCTL = (PE->SLEWCTL & 0x0CFFFFFF) |
                  (0x1<<GPIO_SLEWCTL_HSREN12_Pos) | (0x1<<GPIO_SLEWCTL_HSREN14_Pos) |
                  (0x1<<GPIO_SLEWCTL_HSREN15_Pos) ;
    PJ->SLEWCTL = (PE->SLEWCTL & 0xF3FFFFF0) |
                  (0x1<<GPIO_SLEWCTL_HSREN0_Pos) | (0x1<<GPIO_SLEWCTL_HSREN1_Pos) |
                  (0x1<<GPIO_SLEWCTL_HSREN13_Pos) ;
#endif

}

void UART0_Init(void)
{

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

void  DumpBufferHex(uint8_t *pucBuff, int nSize)
{
    int     nIdx, i;

    nIdx = 0;
    while (nSize > 0)
    {
        printf("0x%04X  ", nIdx);
        for (i = 0; i < 16; i++)
            printf("%02x ", pucBuff[nIdx + i]);
        printf("  ");
        for (i = 0; i < 16; i++)
        {
            if ((pucBuff[nIdx + i] >= 0x20) && (pucBuff[nIdx + i] < 127))
                printf("%c", pucBuff[nIdx + i]);
            else
                printf(".");
            nSize--;
        }
        nIdx += 16;
        printf("\n");
    }
    printf("\n");
}

int  dma_read_write(int is4ByteAddr, uint32_t u32RdCmd, int dc_num)
{
    uint32_t    i, offset;             /* variables */
    uint32_t    *pData;

    if (SPIM_Enable_4Bytes_Mode(is4ByteAddr, 1) != 0)
    {
        printf("\nSPIM_Enable_4Bytes_Mode failed!\n");
        return -1;
    }

    SPIM_SET_DCNUM(dc_num);

    /*
     *  Erase flash page
     */
    printf("Erase SPI flash block 0x%x...", TEST_BLOCK_ADDR);
    SPIM_EraseBlock(TEST_BLOCK_ADDR, is4ByteAddr, OPCODE_BE_64K, 1, 1);
    printf("done.\n");

    SPIM_INVALID_CACHE();

    /*
     *  Verify flash page be erased
     */
    printf("Verify SPI flash block 0x%x be erased...", TEST_BLOCK_ADDR);
    for (offset = 0; offset < FLASH_BLOCK_SIZE; offset += BUFFER_SIZE)
    {
        memset(g_buff, 0, BUFFER_SIZE);
        SPIM_IO_Read(TEST_BLOCK_ADDR+offset, is4ByteAddr, BUFFER_SIZE, g_buff, OPCODE_FAST_READ, 1, 1, 1, 1);

        pData = (uint32_t *)g_buff;
        for (i = 0; i < BUFFER_SIZE; i += 4, pData++)
        {
            if (*pData != 0xFFFFFFFF)
            {
                printf("FAILED!\n");
                printf("Flash address 0x%x, read 0x%x!\n", TEST_BLOCK_ADDR+i, *pData);
                return -1;
            }
        }
    }
    printf("done.\n");

    /*
     *  Program data to flash block
     */
    printf("Program sequential data to flash block 0x%x...", TEST_BLOCK_ADDR);
    for (offset = 0; offset < FLASH_BLOCK_SIZE; offset += BUFFER_SIZE)
    {
        pData = (uint32_t *)g_buff;
        for (i = 0; i < BUFFER_SIZE; i += 4, pData++)
            *pData = (i << 16) | (TEST_BLOCK_ADDR + offset + i);

        SPIM_DMA_Write(TEST_BLOCK_ADDR+offset, is4ByteAddr, BUFFER_SIZE, g_buff, CMD_NORMAL_PAGE_PROGRAM);
    }
    printf("done.\n");

    /*
     *  Verify flash block data with DMA read
     */
    if ((u32RdCmd == CMD_DMA_NORMAL_QUAD_READ) || (u32RdCmd == CMD_DMA_FAST_QUAD_READ))
        SPIM_SetQuadEnable(1, 1);

    printf("Verify SPI flash block 0x%x data with DMA read command 0x%x...", TEST_BLOCK_ADDR, u32RdCmd);
    for (offset = 0; offset < FLASH_BLOCK_SIZE; offset += BUFFER_SIZE)
    {
        memset(g_buff, 0, BUFFER_SIZE);
        SPIM_DMA_Read(TEST_BLOCK_ADDR+offset, is4ByteAddr, BUFFER_SIZE, g_buff, u32RdCmd, 1);
        pData = (uint32_t *)g_buff;
        for (i = 0; i < BUFFER_SIZE; i += 4, pData++)
        {
            if (*pData != ((i << 16) | (TEST_BLOCK_ADDR + offset + i)))
            {
                printf("FAILED!\n");
                printf("Flash address 0x%x, read 0x%x, expect 0x%x!\n", TEST_BLOCK_ADDR+i, *pData, (i << 16) | (TEST_BLOCK_ADDR + offset + i));
                return -1;
            }
        }

    }
    SPIM_SetQuadEnable(0, 1);
    printf("done.\n");

    memset(g_buff, 0, 64);
    printf("\n\nDump SPI flash with I/O read. It's the SPIM cypher encrypted text.\n");
    SPIM_IO_Read(TEST_BLOCK_ADDR, is4ByteAddr, BUFFER_SIZE, g_buff, OPCODE_FAST_READ, 1, 1, 1, 1);
    DumpBufferHex(g_buff, 64);

    memset(g_buff, 0, 64);
    printf("\n\nDump SPI flash with DMA read. It's the plain text. The cypher text was decrypted by SPIM while doing DMA read.\n");
    SPIM_DMA_Read(TEST_BLOCK_ADDR, is4ByteAddr, BUFFER_SIZE, g_buff, u32RdCmd, 1);
    DumpBufferHex(g_buff, 64);

    return 0;
}

int main()
{
    uint8_t     idBuf[3];

    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init();                        /* Init System, IP clock and multi-function I/O */

    UART0_Init();                      /* Initialize UART0 */

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("+------------------------------------+\n");
    printf("|    M460 SPIM cipher mode demo      |\n");
    printf("+------------------------------------+\n");

    SYS_UnlockReg();                   /* Unlock register lock protect */

#ifdef HCLK_192MHZ
    SPIM_SET_CLOCK_DIVIDER(2);        /* Set SPIM clock as HCLK divided by 4 */

    SPIM_SET_RXCLKDLY_RDDLYSEL(0);    /* Insert 0 delay cycle. Adjust the sampling clock of received data to latch the correct data. */
    SPIM_SET_RXCLKDLY_RDEDGE();       /* Use SPI input clock rising edge to sample received data. */
#else
    SPIM_SET_CLOCK_DIVIDER(1);        /* Set SPIM clock as HCLK divided by 2 */

    SPIM_SET_RXCLKDLY_RDDLYSEL(0);    /* Insert 0 delay cycle. Adjust the sampling clock of received data to latch the correct data. */
    SPIM_SET_RXCLKDLY_RDEDGE();       /* Use SPI input clock rising edge to sample received data. */
#endif

    SPIM_SET_DCNUM(8);                /* 8 is the default value. */

    if (SPIM_InitFlash(1) != 0)        /* Initialized SPI flash */
    {
        printf("SPIM flash initialize failed!\n");
        goto lexit;
    }

    SPIM_ReadJedecId(idBuf, sizeof (idBuf), 1);
    printf("SPIM get JEDEC ID=0x%02X, 0x%02X, 0x%02X\n", idBuf[0], idBuf[1], idBuf[2]);

    SPIM->KEY1 = SPIM_KEY_1;
    SPIM->KEY2 = SPIM_KEY_2;

    SPIM_ENABLE_CIPHER();

    SPIM_DISABLE_CCM();

    SPIM_ENABLE_CACHE();

    printf("\n[1] 3-bytes address mode, fast read...");
    if (dma_read_write(0, CMD_DMA_FAST_READ, 8) < 0)
    {
        printf("  FAILED!!\n");
        goto lexit;
    }
    printf("[OK].\n");

    printf("\n[2] 3-bytes address mode, dual read...");
    if (dma_read_write(0, CMD_DMA_FAST_READ_DUAL_OUTPUT, 8) < 0)
    {
        printf("  FAILED!!\n");
        goto lexit;
    }
    printf("[OK].\n");

#if 0  /* W25Q20 does not support 4-bytes address mode. */
    printf("\n[3] 4-bytes address mode, dual read...");
    if (dma_read_write(1, CMD_DMA_NORMAL_DUAL_READ, 8) < 0)
    {
        printf("  FAILED!!\n");
        goto lexit;
    }
    printf("[OK].\n");

    printf("\n[4] 4-bytes address mode, quad read...");
    if (dma_read_write(1, CMD_DMA_FAST_QUAD_READ, 4) < 0)
    {
        printf("  FAILED!!\n");
        goto lexit;
    }
    printf("[OK].\n");
#endif

    printf("\nSPIM cipher enable demo done.\n");

lexit:

    SYS_LockReg();                     /* Lock protected registers */
    while (1);
}

/*** (C) COPYRIGHT 2021 Nuvoton Technology Corp. ***/
