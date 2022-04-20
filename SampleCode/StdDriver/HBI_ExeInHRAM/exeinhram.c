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
volatile uint32_t g_u32Ticks = 0;

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

void SysTick_Handler()
{
    g_u32Ticks++;
    
}

int32_t ExeInHRAM(void)
{
    int32_t i, j;
    uint32_t ticks;
    
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


