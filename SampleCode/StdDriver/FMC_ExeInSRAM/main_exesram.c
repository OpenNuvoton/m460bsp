/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 16/10/17 2:06p $
 * @brief
 *           Implement a code and execute in SRAM to program embedded Flash.
 *           (Support IAR)
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"



#define SRAM_CODE_EXE_ADDR  0x20010000
#define SRAM_CODE_BASE      0x8000
#define SRAM_CODE_SIZE      0x2000

#define TOTAL_VECTORS   (144)                               /* Total vector numbers */
__ALIGNED(256) uint32_t g_au32Vector[TOTAL_VECTORS] = {0};  /* Vector space in SRAM */


typedef void (FUNC_PTR)(void);

extern int32_t FlashAccess_OnSRAM(void);

volatile uint32_t g_u32Ticks = 0;
void SysTick_Handler()
{
    g_u32Ticks++;

    if((g_u32Ticks % 1000) == 0)
    {
        printf("Time elapse: %d\n", g_u32Ticks / 1000);
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

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();
}

int main(void)
{
    int32_t i;
    uint32_t *au32Vectors = (uint32_t *)0x0;
    FUNC_PTR    *func;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init Vector Table to SRAM */
    for(i = 0; i < TOTAL_VECTORS; i++)
    {
        g_au32Vector[i] = au32Vectors[i];
    }
    SCB->VTOR = (uint32_t)&g_au32Vector[0];

    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    printf("\n\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|      FMC Write/Read code execute in SRAM Sample Code      |\n");
    printf("+-----------------------------------------------------------+\n");

    /*
       This sample code is used to demonstrate how to implement a code to execute in SRAM.
    */

    /* SysTick used for test interrupts in SRAM */
    SysTick_Config(SystemCoreClock / 1000);

    /* Unlock protected registers */
    SYS_UnlockReg();

    printf("Execute demo code in APROM ==>\n");

    FlashAccess_OnSRAM();

    memcpy((uint8_t *)SRAM_CODE_EXE_ADDR, (uint8_t *)SRAM_CODE_BASE, SRAM_CODE_SIZE);

    printf("Execute demo code in SRAM ==>\n");

    func = (FUNC_PTR *)(SRAM_CODE_EXE_ADDR + 1);

    func();   /* branch to exeinsram.o in SRAM  */

    printf("Execute demo code in SRAM Done!\n");
    /* Lock protected registers */
    SYS_LockReg();

    printf("\nFMC Sample Code Completed.\n");

    while(1);

}
/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
