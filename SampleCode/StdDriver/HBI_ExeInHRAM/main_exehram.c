/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 16/10/17 2:06p $
 * @brief
 *           Implement a code and execute in HyperRAM to program embedded Flash.
 *           (Support IAR)
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define HRAM_CODE_BASE      0x8000
#define HRAM_CODE_SIZE      0x2000

extern int32_t ExeInHRAM(void);
extern int32_t FlashAccess_OnHRAM(void);
extern int32_t CopyToHyperRAM(uint32_t u32StartAddr, uint32_t u32EndAddr);

typedef uint32_t (FUNC_PTR)(void);

volatile uint32_t g_u32Ticks = 0;
void SysTick_Handler()
{
    g_u32Ticks++;

    if((g_u32Ticks % 1000) == 0)
    {
        printf("Time elapse: %d\n", g_u32Ticks / 1000);
    }

}

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

    /* Enable HBI interrupt */
    HBI_ENABLE_INT;
    NVIC_EnableIRQ(HBI_IRQn);
}

int main(void)
{
    FUNC_PTR *ori_func, *new_func;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    printf("\n\n");
    printf("+------------------------------------------------+\n");
    printf("|      Code execute in HyperRAM Sample Code      |\n");
    printf("+------------------------------------------------+\n");

    /*
       This sample code is used to demonstrate how to implement a code to execute in HyperRAM.
    */

    /* SysTick used for test interrupts in HyperRAM */
    SysTick_Config(SystemCoreClock / 1000);

    /* Unlock protected registers */
    SYS_UnlockReg();

    printf("HyperRAM R/W test in APROM ==>\n");

    if(FlashAccess_OnHRAM() < 0)
    {
        printf("APROM access on HyperRAM failed.\n");
        goto lexit;
    }

    if(CopyToHyperRAM(HRAM_CODE_BASE, (HRAM_CODE_BASE + HRAM_CODE_SIZE)) < 0)
    {
        printf("Copy to HyperRAM failed.\n");
        goto lexit;
    }

    printf("Execute demo code in HyperRAM ==>\n");
    ori_func = (FUNC_PTR *)ExeInHRAM;
    new_func = (FUNC_PTR *)(HYPERRAM_BASE + ((uint32_t)ori_func - HRAM_CODE_BASE));
    printf("Will branch to address: 0x%08x\n", (uint32_t)new_func);
    new_func();

    printf("Execute demo code in HyperRAM Done!\n");
    /* Lock protected registers */
    SYS_LockReg();

lexit:
    while(1);
}
/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
