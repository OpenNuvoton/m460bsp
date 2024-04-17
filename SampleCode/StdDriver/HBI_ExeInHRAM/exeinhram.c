/****************************************************************************//**
 * @file     exeinsram.c
 * @version  V0.10
 * @brief    Implement a code to execute in HyperRAM.
 *
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define TEST_SIZE   (8 * 1024)
__ALIGNED(4) uint8_t g_u8Pool[TEST_SIZE];
#if !defined(__IAR_SYSTEMS_ICC__)
volatile uint32_t g_u32Ticks = 0;
void SysTick_Handler()
{
    g_u32Ticks++;
}
#else
extern volatile uint32_t g_u32Ticks;

#define TEST_PATTERN                0x5A5A5A5A

static int32_t Clear4Bytes(uint32_t u32StartAddr)
{
    outp32(u32StartAddr, 0);
    return 0;
}

static int32_t ClearHyperRAM(uint32_t u32StartAddr, uint32_t u32EndAddr)
{
    uint32_t u32Data, i;

    for (i = u32StartAddr; i < u32EndAddr; i += 4)
    {
        if (Clear4Bytes(i) < 0)
        {
            return -1;
        }
        u32Data = inp32(i);
        if (u32Data != 0)
        {
            printf("ClearHyperRAM fail!! Read address:0x%08x  data::0x%08x  expect: 0\n", i, u32Data);
            return -1;
        }
    }

    return 0;
}

int32_t FlashAccess_OnHRAM(void)
{
    uint32_t u32Data;
    uint32_t i, u32StartAddr, u32EndAddr;

    /* The HyperRAM address for erase/write/read demo */
    u32StartAddr = HYPERRAM_BASE;
    u32EndAddr = u32StartAddr + 0x1000;
    ClearHyperRAM(u32StartAddr, u32EndAddr);

    /* Clear HyperRAM */
    if (ClearHyperRAM(u32StartAddr, u32EndAddr) < 0)
        goto lexit;

    /* Fill 4 Byte pattern to HyperRAM */
    printf("4 Byte Write test to HyperRAM .....");
    for (i = u32StartAddr; i < u32EndAddr; i += 4)
    {
        outp32(i, TEST_PATTERN);
    }

    /* Read 4 Byte pattern to check */
    for (i = u32StartAddr; i < u32EndAddr; i += 4)
    {
        u32Data = inp32(i);
        if (u32Data != TEST_PATTERN)
        {
            printf("Done\n");
            printf("line(%d) [FAIL] Read address:0x%08x  data::0x%08x  expect:0x%08x \n", __LINE__, i, u32Data, TEST_PATTERN);
            goto lexit;
        }
    }
    printf("Done!!\n");

    /* Clear HyperRAM */
    if (ClearHyperRAM(u32StartAddr, u32EndAddr) < 0)
        goto lexit;

    return 0;

lexit:
    return -1;
}

int32_t CopyToHyperRAM(uint32_t u32StartAddr, uint32_t u32EndAddr)
{
    uint32_t u32Data, u32Data_HRAM, i, u32DataSize;
    uint32_t u32HRAMStartAddr;

    u32DataSize = u32EndAddr - u32StartAddr;
    u32HRAMStartAddr = HYPERRAM_BASE;

    /* Clear HyperRAM */
    if (ClearHyperRAM(u32HRAMStartAddr, (u32HRAMStartAddr + u32DataSize)) < 0)
        goto lexit;
    
    for (i = 0; i < u32DataSize; i += 4)
    {
        u32Data = inp32(u32StartAddr + i);
        outp32(u32HRAMStartAddr + i, u32Data);
    }

    /* Read 4 Byte pattern to check */
    for (i = 0; i < u32DataSize; i += 4)
    {
        u32Data = inp32(u32StartAddr + i);
        u32Data_HRAM = inp32(u32HRAMStartAddr + i);
        if (u32Data != u32Data_HRAM)
        {
            printf("line(%d) [FAIL] Read address:0x%08x  data::0x%08x  expect:0x%08x \n", __LINE__, (u32StartAddr + i), u32Data, u32Data_HRAM);
            goto lexit;
        }
    }

    return 0;

lexit:
    return -1;
}
#endif

void test()
{
    int32_t i;
    int32_t* pi32;

    memset(g_u8Pool, 0, TEST_SIZE);

    for(i = 0; i < TEST_SIZE; i++)
    {
        g_u8Pool[i] = i;
        g_u8Pool[TEST_SIZE - i - 1] = g_u8Pool[i];
        g_u8Pool[i] = g_u8Pool[TEST_SIZE - i - 1];
    }

    pi32 = (int32_t*)g_u8Pool;
    for(i = 0; i < TEST_SIZE / 4; i++)
    {
        pi32[i] = i;
        pi32[i] = pi32[TEST_SIZE / 4 - i - 1];
        pi32[TEST_SIZE / 4 - i - 1] = pi32[i];
    }

}

int32_t ExeInHRAM(void)
{
    int32_t i, j;
    
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPCCKEN_Msk;


    /* Init SysTick */
    SysTick_Config(SystemCoreClock / 1000);

    for(i = 1; i < 100; i += 10)
    {
        g_u32Ticks = 0;
        PC0 = 1;
        for(j = 0; j < i; j++)
        {
            test();
        }
        PC0 = 0;
        printf("Elapsed time: %d ms, %d loops\n", g_u32Ticks, j-1);
    }

    return 0;
}


