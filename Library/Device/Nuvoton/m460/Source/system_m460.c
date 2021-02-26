/**************************************************************************//**
 * @file     system_M480.c
 * @version  V1.000
 * @brief    CMSIS Cortex-M4 Core Peripheral Access Layer Source File for M480
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2017-2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/

#include "NuMicro.h"


/*----------------------------------------------------------------------------
  DEFINES
 *----------------------------------------------------------------------------*/


/*----------------------------------------------------------------------------
  Clock Variable definitions
 *----------------------------------------------------------------------------*/
uint32_t SystemCoreClock  = __SYSTEM_CLOCK;    /*!< System Clock Frequency (Core Clock)*/
uint32_t CyclesPerUs      = (__HSI / 1000000UL); /* Cycles per micro second */
uint32_t PllClock         = __HSI;             /*!< PLL Output Clock Frequency         */
uint32_t gau32ClkSrcTbl[] = {__HXT, __LXT, 0UL, __LIRC, 0UL, 0UL, 0UL, __HIRC};

/*----------------------------------------------------------------------------
  Clock functions
 *----------------------------------------------------------------------------*/
void SystemCoreClockUpdate (void)            /* Get Core Clock Frequency      */
{
    uint32_t u32Freq, u32ClkSrc;
    uint32_t u32HclkDiv;

    /* Update PLL Clock */
    PllClock = CLK_GetPLLClockFreq();

    u32ClkSrc = CLK->CLKSEL0 & CLK_CLKSEL0_HCLKSEL_Msk;

    if(u32ClkSrc == CLK_CLKSEL0_HCLKSEL_PLL)
    {
        /* Use PLL clock */
        u32Freq = PllClock;
    }
    else
    {
        /* Use the clock sources directly */
        u32Freq = gau32ClkSrcTbl[u32ClkSrc];
    }

    u32HclkDiv = (CLK->CLKDIV0 & CLK_CLKDIV0_HCLKDIV_Msk) + 1UL;

    /* Update System Core Clock */
    SystemCoreClock = u32Freq / u32HclkDiv;


    //if(SystemCoreClock == 0)
    //    __BKPT(0);

    CyclesPerUs = (SystemCoreClock + 500000UL) / 1000000UL;
}


/**
 * @brief  Initialize the System
 *
 * @param  none
 * @return none
 */
void SystemInit (void)
{

    /* Add your system initialize code here.
       Do not use global variables because this function is called before
       reaching pre-main. RW section maybe overwritten afterwards.          */

    /* FPU settings ------------------------------------------------------------*/
#if (__FPU_PRESENT == 1U) && (__FPU_USED == 1U)
    SCB->CPACR |= ((3UL << 10*2) |                 /* set CP10 Full Access */
                   (3UL << 11*2)  );               /* set CP11 Full Access */
#endif

}
