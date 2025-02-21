/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Bank Remap sample code(Bank0 App).
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "NuDB_common.h"
#include "xmodem.h"


#define PLL_CLOCK       96000000

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint32_t  s_u32DbLength;             /* dual bank program remaining length       */
static volatile uint32_t  s_u32DbAddr;               /* dual bank program current flash address  */
static volatile uint32_t  s_u32TickCnt;              /* timer ticks - 100 ticks per second       */

/*---------------------------------------------------------------------------------------------------------*/
/* Global Functions Declaration                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void ResetCPU(void);
void UART_Init(UART_T* uart, uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32BaudRate);
void SysTick_Handler(void);
void EnableSysTick(int i8TicksPerSecond);
void WDT_IRQHandler(void);
void SYS_Init(void);
uint32_t  SelfTest(void);
void StartTimer0(void);
uint32_t  GetTimer0Counter(void);
uint32_t  FuncCrc32(uint32_t u32Start, uint32_t u32Len);
/*---------------------------------------------------------------------------------------------------------*/
/* Global Functions                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t  FuncCrc32(uint32_t u32Start, uint32_t u32Len)
{
    uint32_t  u32Idx, u32Data = 0UL;

    /* WDTAT_RVS, CHECKSUM_RVS, CHECKSUM_COM */
    for(u32Idx = 0; u32Idx < u32Len; u32Idx += 4)
    {
        u32Data += *(uint32_t *)(u32Start + u32Idx);
    }
    u32Data = 0xFFFFFFFF - u32Data + 1UL;

    return u32Data;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Interrupt Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void SysTick_Handler(void)
{
    /* Increase timer tick */
    s_u32TickCnt++;

    /* Calculate CRC32 value, just to consume CPU time  */
    FuncCrc32(0x10000, 0x100);
}

void EnableSysTick(int i8TicksPerSecond)
{
    s_u32TickCnt = 0;

    /* HCLK is 64 MHz */
    SystemCoreClock = PLL_CLOCK;
    if(SysTick_Config(SystemCoreClock / (uint32_t)i8TicksPerSecond))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");
        while(1);
    }
}

void WDT_IRQHandler(void)
{
    WDT_RESET_COUNTER();

    if(WDT_GET_TIMEOUT_INT_FLAG() == 1)
    {
        /* Clear WDT time-out interrupt flag */
        WDT_CLEAR_TIMEOUT_INT_FLAG();
    }


}



void ResetCPU(void)
{
    SYS->IPRST0 |= SYS_IPRST0_CPURST_Msk;
    SYS->IPRST0 &= ~SYS_IPRST0_CPURST_Msk;
}

void UART_Init(UART_T* uart, uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32BaudRate)
{
    uint32_t u32ClkTbl[4] = {__HXT, 0, __LXT, __HIRC};

    /* UART clock source and clock divider setting */

    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_UART1SEL_Msk)) | (u32ClkSrc << CLK_CLKSEL1_UART1SEL_Pos);
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_UART1DIV_Msk)) | (u32ClkDiv << CLK_CLKDIV0_UART1DIV_Pos);


    /* Set UART line configuration */
    uart->LINE = UART_WORD_LEN_8 | UART_PARITY_NONE | UART_STOP_BIT_1;

    /* Get PLL clock frequency if UART clock source selection is PLL */
    if(u32ClkSrc == 1)
        u32ClkTbl[1] = CLK_GetPLLClockFreq();

    /* Set UART baud rate */
    if(u32BaudRate != 0)
        uart->BAUD = (UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER((u32ClkTbl[u32ClkSrc]) / (u32ClkDiv + 1), u32BaudRate));

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



uint32_t  SelfTest(void)
{
    uint32_t  i,  sum = 0;
    for(i = 0; i < 1; i++)
    {
        sum = FuncCrc32(0x10000, 0x10000);
    }
    printf("sum = 0x%x", sum);
    return 1;
}

void StartTimer0(void)
{
    /* Start TIMER0  */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_TMR0SEL_Msk)) | CLK_CLKSEL1_TMR0SEL_HXT;
    /* enable TIMER0 clock */
    CLK->APBCLK0 |= CLK_APBCLK0_TMR0CKEN_Msk;
    /* Disable timer */
    TIMER0->CTL = 0;
    /* Clear interrupt status */
    TIMER0->INTSTS = (TIMER_INTSTS_TWKF_Msk | TIMER_INTSTS_TIF_Msk);
    /* Maximum time  */
    TIMER0->CMP = 0xFFFFFE;
    /* Clear timer counter */
    TIMER0->CNT = 0;
    /* Start timer */
    TIMER0->CTL = (11 << TIMER_CTL_PSC_Pos) | TIMER_ONESHOT_MODE | TIMER_CTL_CNTEN_Msk;
}

uint32_t  GetTimer0Counter(void)
{
    return TIMER0->CNT;
}


int main()
{
    uint32_t u32ch;
    int32_t  i32Err;
    /* CPU executing in which Bank */
    uint32_t  u32ExecBank = 0;

    /* Disable register write-protection function */
    SYS_UnlockReg();

    /* Initial clocks and multi-functions */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);
    UART_Open(UART1, 115200);

    /* Set Vector Table Offset Register */
    SCB->VTOR = APP_BASE;

    /* Enable ISP and APROM update */
    FMC_ENABLE_ISP();
    FMC_ENABLE_AP_UPDATE();

    /* Unlock protected registers */
    SYS_UnlockReg();


    do
    {
        printf("\n\n");
        printf("+------------------------+\n");
        printf("|  Boot from 0x%08X  |\n", FMC_GetVECMAP());
        printf("+------------------------+\n");

        u32ExecBank = (uint32_t)((FMC->ISPSTS & FMC_ISPSTS_FBS_Msk) >> FMC_ISPSTS_FBS_Pos);

#ifdef NewApp
        printf("\n BANK%d APP processing (New Firmware!!!)\n", u32ExecBank);
#else
        printf("\n BANK%d APP processing \n", u32ExecBank);
#endif

        printf("\n Download new FW?[y/n]\n");
        u32ch = (uint32_t)getchar();
        if(u32ch == 'y')
        {
            printf("\n Bank%d processing, download data to Bank%d.\n", u32ExecBank, u32ExecBank ^ 1);

            EnableSysTick(1000);
            StartTimer0();

            /* Dual bank background program address */
            s_u32DbAddr   = FMC_BANK_SIZE * (u32ExecBank ^ 1) + APP_BASE;
            /* Dual bank background length */
            s_u32DbLength = APP_SIZE;

            EnableSysTick(1000);
            StartTimer0();

            i32Err = Xmodem(s_u32DbAddr);
            if(i32Err < 0)
            {
                printf("Xmodem transfer fail!\n");
                while(1);
            }
            else
            {
                printf("Xomdem transfer done!\n");
                printf("Total transfer size is %d\n", i32Err);
            }

            printf("\n Firmware download completed!!\n");
        }
        else
        {
            printf("\n Reset from BANK%d Loader \n", u32ExecBank);
            /* Remap to Loader */
            FMC_SetVectorPageAddr(0x0);
            ResetCPU();
        }




    }
    while(1);


}

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
