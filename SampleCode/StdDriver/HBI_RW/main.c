/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show FMC read Flash IDs, erase, read, and write function
 *
 *
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
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
        while(1);
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
    CLK_SetCoreClock(200000000);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;
    CLK->AHBCLK1 |= CLK_AHBCLK1_GPICKEN_Msk | CLK_AHBCLK1_GPJCKEN_Msk;

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable HBI module clock */
    CLK_EnableModuleClock(HBI_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set multi-function pins for HBI */
    SET_HBI_D0_PG11();
    SET_HBI_D1_PG12();
    SET_HBI_D2_PC0();
    SET_HBI_D3_PG10();
    SET_HBI_D4_PG9();
    SET_HBI_D5_PG13();
    SET_HBI_D6_PG14();
    SET_HBI_D7_PG15();

    SET_HBI_RWDS_PC1();
    SET_HBI_nRESET_PC2();
    SET_HBI_nCS_PC3();
    SET_HBI_CK_PC4();
    SET_HBI_nCK_PC5();

    /* Enable HBI interrupt */
    HBI_ENABLE_INT;
    NVIC_EnableIRQ(HBI_IRQn);
}

void UART0_Init(void)
{

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}



static void Clear4Bytes(uint32_t u32StartAddr)
{
    HBI->ADR = u32StartAddr;
    HBI->WDATA = 0;
    HBI->CMD = HBI_CMD_WRITE_HRAM_4_BYTE;

    while(HBI->CMD != 0);
}

static void ClearHyperRAM(uint32_t u32StartAddr, uint32_t u32EndAddr)
{
    uint32_t u32Data, i;

    for(i = u32StartAddr; i < u32EndAddr; i+=4)
    {
        Clear4Bytes(i);
        u32Data = HBI_Read2Word(i);
        if(u32Data != 0)
        {
            printf("ClearHyperRAM fail!! Read address:0x%08x  data::0x%08x  expect: 0\n",  i, u32Data);
            while(1);
        }
    }
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

#ifdef _PZ
    /* For palladium */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(153600, 38400);
#endif

    printf("+------------------------------------------+\n");
    printf("|    M460 HyperBus Interface Sample Code   |\n");
    printf("+------------------------------------------+\n");

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
        ClearHyperRAM(u32StartAddr, u32EndAddr);

        /* Fill 4 Byte pattern to HyperRAM */
        printf("4 Byte Write test .....\n");
        for(i = u32StartAddr; i < u32EndAddr; i+=4)
        {
            HBI_Write4Byte(i, u32BitPattern);
        }

        /* Read 4 Byte pattern to check */
        for(i = u32StartAddr; i < u32EndAddr; i+=4)
        {
            u32Data = HBI_Read2Word(i);
            if(u32Data != u32BitPattern)
            {
                printf("line(%d) [FAIL] Read address:0x%08x  data::0x%08x  expect:0x%08x \n", __LINE__,i,  u32Data, u32BitPattern);
                return -1;
            }
        }
        printf("4 Byte Write test Done!!\n");

        /* Clear HyperRAM */
        ClearHyperRAM(u32StartAddr, u32EndAddr);

        /* Fill 3 Byte pattern to HyperRAM */
        printf("3 Byte Write test .....\n");
        for(i = u32StartAddr; i < u32EndAddr; i+=4)
        {
            HBI_Write3Byte(i, u24BitPattern);
        }

        /* Read 3 Byte pattern to check */
        for(i = u32StartAddr; i < u32EndAddr; i+=4)
        {
            u32Data = HBI_Read2Word(i);

            if(u32Data != u24BitPattern)
            {
                printf("line(%d) [FAIL] Read address:0x%08x  data::0x%08x  expect:0x%08x \n", __LINE__,i,  u32Data, u24BitPattern);
                return -1;
            }
        }
        printf("3 Byte Write test Done!!\n");

        /* Clear HyperRAM */
        ClearHyperRAM(u32StartAddr, u32EndAddr);

        /* Fill 2 Byte pattern to HyperRAM */
        printf("2 Byte Write test .....\n");
        for(i = u32StartAddr; i < u32EndAddr; i+=2)
        {
            HBI_Write2Byte(i, u16BitPattern);
        }

        /* Read 2 Byte pattern to check */
        for(i = u32StartAddr; i < u32EndAddr; i+=2)
        {
            u32Data = HBI_Read1Word(i);

            if(u32Data != u16BitPattern)
            {
                printf("line(%d) [FAIL] Read address:0x%08x  data::0x%08x  expect:0x%08x \n", __LINE__,i,  u32Data, u16BitPattern);
                return -1;
            }
        }
        printf("2 Byte Write test Done!!\n");

        /* Clear HyperRAM */
        ClearHyperRAM(u32StartAddr, u32EndAddr);

        /* Fill 1 Byte pattern to HyperRAM */
        printf("1 Byte Write test .....\n");
        for(i = u32StartAddr; i < u32EndAddr; i+=1)
        {
            HBI_Write1Byte(i, u8BitPattern);
        }

        /* Read 1 Byte pattern to check */
        for(i = u32StartAddr; i < u32EndAddr; i+=2)
        {
            u32Data = HBI_Read1Word(i);
            if(u32Data != (u8BitPattern<<8|u8BitPattern))
            {
                printf("line(%d) [FAIL] Read address:0x%08x  data::0x%08x  expect:0x%08x \n", __LINE__,i,  u32Data,  (u8BitPattern<<8|u8BitPattern));
                return -1;
            }
        }
        printf("1 Byte Write test Done!!\n");

        /* Clear HyperRAM */
        ClearHyperRAM(u32StartAddr, u32EndAddr);

        printf("======= Pattern Round[%d] Test Pass! ======= \n", u32PatCnt);

    }

    printf("\nHyperBus Interface Sample Code Completed.\n");

    while (1);
}

/*** (C) COPYRIGHT 2021 Nuvoton Technology Corp. ***/
