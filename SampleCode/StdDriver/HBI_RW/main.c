/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show HyperRAM read/write control via HyperBus Interface
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>

#include "NuMicro.h"

void HBI_IRQHandler(void)
{

    if((HBI->INTSTS & HBI_INTSTS_OPDONE_Msk) == HBI_INTSTS_OPDONE_Msk)
    {
        HBI->INTSTS |= HBI_INTSTS_OPDONE_Msk;
    }

    if((HBI->INTSTS & HBI_INTSTS_OPDONE_Msk) == HBI_INTSTS_OPDONE_Msk)
    {
        HBI->INTSTS |= HBI_INTSTS_OPDONE_Msk;
        printf("clear done flag fail ! \n");
    }

}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 200MHz */
    CLK_SetCoreClock(180000000);

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

    /* Enable HBI interrupt */
    HBI_ENABLE_INT;
    NVIC_EnableIRQ(HBI_IRQn);
}

void UART0_Init(void)
{

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}



static int32_t Clear4Bytes(uint32_t u32StartAddr)
{
    uint32_t u32TimeOutCnt = HBI_TIMEOUT;

    HBI->ADR = u32StartAddr;
    HBI->WDATA = 0;
    HBI->CMD = HBI_CMD_WRITE_HRAM_4_BYTE;

    while(HBI->CMD != 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for HBI time-out!\n");
            return -1;
        }
    }

    return 0;
}

static int32_t ClearHyperRAM(uint32_t u32StartAddr, uint32_t u32EndAddr)
{
    uint32_t u32Data, i;

    for(i = u32StartAddr; i < u32EndAddr; i+=4)
    {
        if( Clear4Bytes(i) < 0 )
        {
            return -1;
        }
        u32Data = HBI_Read4Byte(i);
        if(u32Data != 0)
        {
            printf("ClearHyperRAM fail!! Read address:0x%08x  data::0x%08x  expect: 0\n",  i, u32Data);
            return -1;
        }
    }

    return 0;
}

int main()
{
    uint32_t i, u32Data, u32PatCnt;
    uint32_t u32StartAddr, u32EndAddr;
    uint32_t u32BitPattern, u24BitPattern;
    uint16_t u16BitPattern;
    uint8_t u8BitPattern;
    uint32_t au32Pat[5] = {0x12345678, 0x5a5a5a5a, 0xa5a5a5a5, 0xFFFFFFFF, 0x9abcdef1};


    SYS_UnlockReg();                   /* Unlock register lock protect */

    SYS_Init();                        /* Init System, IP clock and multi-function I/O */

    UART0_Init();                      /* Initialize UART0 */

    printf("+------------------------------------------+\n");
    printf("|    M460 HyperBus Interface Sample Code   |\n");
    printf("+------------------------------------------+\n");

    /*
        HBI initialization has been implemented in SystemInit() with HBI_ENABLE option.
    */
    
    /* Memory max space 64MBits --> 8Mbytes --> 0x800000 */
    u32StartAddr = 0x0;
    u32EndAddr = u32StartAddr+0x1000;

    for(u32PatCnt = 0; u32PatCnt < 5; u32PatCnt++)
    {
        printf("======= Pattern Round[%d] Test Start! ======= \n", u32PatCnt);
        u32BitPattern = au32Pat[u32PatCnt];
        u24BitPattern = u32BitPattern >> 8;
        u16BitPattern = u24BitPattern >> 8;
        u8BitPattern  = u16BitPattern >> 8;

        printf("Test Pattern 32 bits: 0x%08x\n", u32BitPattern);
        printf("             24 bits: 0x%08x\n", u24BitPattern);
        printf("             16 bits: 0x%08x\n", u16BitPattern);
        printf("              8 bits: 0x%08x\n", u8BitPattern);

        /* Clear HyperRAM */
        if( ClearHyperRAM(u32StartAddr, u32EndAddr) < 0 )
            goto lexit;

        /* Fill 4 Byte pattern to HyperRAM */
        printf("4 Byte Write test .....\n");
        for(i = u32StartAddr; i < u32EndAddr; i+=4)
        {
            HBI_Write4Byte(i, u32BitPattern);
        }

        /* Read 4 Byte pattern to check */
        for(i = u32StartAddr; i < u32EndAddr; i+=4)
        {
            u32Data = HBI_Read4Byte(i);
            if(u32Data != u32BitPattern)
            {
                printf("line(%d) [FAIL] Read address:0x%08x  data::0x%08x  expect:0x%08x \n", __LINE__,i,  u32Data, u32BitPattern);
                goto lexit;
            }
        }
        printf("4 Byte Write test Done!!\n");

        /* Clear HyperRAM */
        if( ClearHyperRAM(u32StartAddr, u32EndAddr) < 0 )
            goto lexit;

        /* Fill 3 Byte pattern to HyperRAM */
        printf("3 Byte Write test .....\n");
        for(i = u32StartAddr; i < u32EndAddr; i+=4)
        {
            HBI_Write3Byte(i, u24BitPattern);
        }

        /* Read 3 Byte pattern to check */
        for(i = u32StartAddr; i < u32EndAddr; i+=4)
        {
            u32Data = HBI_Read4Byte(i);

            if(u32Data != u24BitPattern)
            {
                printf("line(%d) [FAIL] Read address:0x%08x  data::0x%08x  expect:0x%08x \n", __LINE__,i,  u32Data, u24BitPattern);
                goto lexit;
            }
        }
        printf("3 Byte Write test Done!!\n");

        /* Clear HyperRAM */
        if( ClearHyperRAM(u32StartAddr, u32EndAddr) < 0 )
            goto lexit;

        /* Fill 2 Byte pattern to HyperRAM */
        printf("2 Byte Write test .....\n");
        for(i = u32StartAddr; i < u32EndAddr; i+=2)
        {
            HBI_Write2Byte(i, u16BitPattern);
        }

        /* Read 2 Byte pattern to check */
        for(i = u32StartAddr; i < u32EndAddr; i+=2)
        {
            u32Data = HBI_Read2Byte(i);

            if(u32Data != u16BitPattern)
            {
                printf("line(%d) [FAIL] Read address:0x%08x  data::0x%08x  expect:0x%08x \n", __LINE__,i,  u32Data, u16BitPattern);
                return -1;
            }
        }
        printf("2 Byte Write test Done!!\n");

        /* Clear HyperRAM */
        if( ClearHyperRAM(u32StartAddr, u32EndAddr) < 0 )
            goto lexit;

        /* Fill 1 Byte pattern to HyperRAM */
        printf("1 Byte Write test .....\n");
        for(i = u32StartAddr; i < u32EndAddr; i+=1)
        {
            HBI_Write1Byte(i, u8BitPattern);
        }

        /* Read 1 Byte pattern to check */
        for(i = u32StartAddr; i < u32EndAddr; i+=2)
        {
            u32Data = HBI_Read2Byte(i);
            if(u32Data != (u8BitPattern<<8|u8BitPattern))
            {
                printf("line(%d) [FAIL] Read address:0x%08x  data::0x%08x  expect:0x%08x \n", __LINE__,i,  u32Data,  (u8BitPattern<<8|u8BitPattern));
                goto lexit;
            }
        }
        printf("1 Byte Write test Done!!\n");

        /* Clear HyperRAM */
        if( ClearHyperRAM(u32StartAddr, u32EndAddr) < 0 )
            goto lexit;

        printf("======= Pattern Round[%d] Test Pass! ======= \n", u32PatCnt);
    }

    printf("\nHyperBus Interface Sample Code Completed.\n");

lexit:

    while (1);
}
