/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Implement CRC in CRC-32 mode with PDMA transfer.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
typedef struct dma_desc_t
{
    uint32_t u32Ctl;
    uint32_t u32Src;
    uint32_t u32Dest;
    uint32_t u32Offset;
} DMA_DESC_T;
static DMA_DESC_T g_DMA_DESC[1];

uint32_t GetFMCChecksum(uint32_t u32Address, uint32_t u32Size);
uint32_t GetPDMAChecksum(uint32_t u32Address, uint32_t u32Size);
void SYS_Init(void);
void UART_Init(void);


uint32_t GetFMCChecksum(uint32_t u32Address, uint32_t u32Size)
{
    uint32_t u32CHKS;

    FMC_ENABLE_ISP();
    u32CHKS = FMC_GetChkSum(u32Address, u32Size);

    return u32CHKS;
}

uint32_t GetPDMAChecksum(uint32_t u32Address, uint32_t u32Size)
{
    volatile uint32_t u32Loop = 0, u32INTSTS;

    /* Enable PDMA module clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_PDMA0CKEN_Msk;

    /* Give valid source address and transfer count and program PDMA memory to memory, dest => CRC_WDATA */
    PDMA0->CHCTL = (1 << 0); // use PDMA CH0
    g_DMA_DESC[0].u32Ctl =
        (1 << PDMA_DSCT_CTL_OPMODE_Pos)  | (0 << PDMA_DSCT_CTL_TXTYPE_Pos) |
        (7 << PDMA_DSCT_CTL_BURSIZE_Pos) |
        (0 << PDMA_DSCT_CTL_SAINC_Pos)   | (3 << PDMA_DSCT_CTL_DAINC_Pos) |
        (2 << PDMA_DSCT_CTL_TXWIDTH_Pos) | (((u32Size / 4) - 1) << PDMA_DSCT_CTL_TXCNT_Pos);
    g_DMA_DESC[0].u32Src    = (uint32_t)u32Address;
    g_DMA_DESC[0].u32Dest   = (uint32_t) & (CRC->DAT);
    g_DMA_DESC[0].u32Offset = 0;

    PDMA0->DSCT[0].CTL = PDMA_OP_SCATTER;
    PDMA0->DSCT[0].NEXT = (uint32_t)&g_DMA_DESC[0] - (PDMA0->SCATBA);

    u32INTSTS = PDMA0->INTSTS;
    PDMA0->INTSTS = u32INTSTS;
    PDMA0->INTEN = (1 << 0);

    /* Trigger PDMA CH0 transfer ... */
    PDMA0->SWREQ = (1 << 0);

    while(PDMA0->TRGSTS & 0x1)   // wait PDMA finish
    {
        if(u32Loop++ > (SystemCoreClock / 100))
        {
            printf("\n[PDMA transfer time-out]\n");
            return 0;
        }
    }

    return CRC->CHECKSUM;
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

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));  
    
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Initialization for sample code                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable CRC module clock */
    CLK_EnableModuleClock(CRC_MODULE);
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(DEBUG_PORT, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32Addr, u32FMCChecksum, u32CRC32Checksum, u32PDMAChecksum;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-----------------------------------------------------+\n");
    printf("|    CRC32 with PDMA Sample Code                      |\n");
    printf("|       - Get APROM first %d bytes CRC result by    |\n", (uint32_t)FMC_FLASH_PAGE_SIZE);
    printf("|          a.) FMC checksum command                   |\n");
    printf("|          b.) CPU write CRC data register directly   |\n");
    printf("|          c.) PDMA write CRC data register           |\n");
    printf("+-----------------------------------------------------+\n\n");

    /*  Case a. */
    u32FMCChecksum = GetFMCChecksum(0x0, FMC_FLASH_PAGE_SIZE);

    /*  Case b. */
    /* Configure CRC controller for CRC-CRC32 mode */
    CRC_Open(CRC_32, (CRC_WDATA_RVS | CRC_CHECKSUM_RVS | CRC_CHECKSUM_COM), 0xFFFFFFFFul, CRC_CPU_WDATA_32);
    /* Start to execute CRC-CRC32 operation */
    for(u32Addr = 0; u32Addr < FMC_FLASH_PAGE_SIZE; u32Addr += 4)
    {
        CRC_WRITE_DATA(inpw(u32Addr));
    }
    u32CRC32Checksum = CRC_GetChecksum();

    /*  Case c. */
    /* Configure CRC controller for CRC-CRC32 mode with PDMA */
    CRC_Open(CRC_32, (CRC_WDATA_RVS | CRC_CHECKSUM_RVS | CRC_CHECKSUM_COM), 0xFFFFFFFFul, CRC_CPU_WDATA_32);
    u32PDMAChecksum = GetPDMAChecksum(0x0, FMC_FLASH_PAGE_SIZE);

    printf("APROM first %d bytes checksum:\n", (uint32_t)FMC_FLASH_PAGE_SIZE);
    printf("   - by FMC command:    0x%08X\n", u32FMCChecksum);
    printf("   - by CRC CPU write:  0x%08X\n", u32CRC32Checksum);
    printf("   - by CRC PDMA write: 0x%08X\n", u32PDMAChecksum);

    if((u32FMCChecksum == u32CRC32Checksum) && (u32CRC32Checksum == u32PDMAChecksum))
    {
        if((u32FMCChecksum == 0) || (u32FMCChecksum == 0xFFFFFFFFul))
        {
            printf("\n[Get checksum ... WRONG]\n");
        }
        else
        {
            printf("\n[Compare checksum ... PASS]\n");
        }
    }
    else
    {
        printf("\n[Compare checksum ... WRONG]\n");
    }

    /* Disable CRC function */
    CLK_DisableModuleClock(CRC_MODULE);

    while(1) {}
}
