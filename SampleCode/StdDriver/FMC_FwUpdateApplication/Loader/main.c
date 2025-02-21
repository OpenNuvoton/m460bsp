/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    Bank Remap sample code(Bank0 Loader).
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "NuDB_common.h"
#include "xmodem.h"


#define CREATE_BANK1_APP   0
#define PLL_CLOCK          96000000

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint32_t  s_u32ExecBank;             /* CPU executing in which Bank              */
static volatile uint32_t  s_u32DbLength;             /* dual bank program remaining length       */
static volatile uint32_t  s_u32DbAddr;               /* dual bank program current flash address  */
static volatile uint32_t  s_u32TickCnt;              /* timer ticks - 100 ticks per second       */
/*---------------------------------------------------------------------------------------------------------*/
/* Global Functions                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void UART_Init(UART_T* uart, uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32BaudRate);
void ResetCPU(void);
void SysTick_Handler(void);
void EnableSysTick(int i8TicksPerSecond);
void WDT_IRQHandler(void);
void StartTimer0(void);
uint32_t  GetTimer0Counter(void);
void SYS_Init(void);
void Download(void);
uint32_t  FuncCrc32(uint32_t u32Start, uint32_t u32Len);


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


void ResetCPU(void)
{
    SYS->IPRST0 |= SYS_IPRST0_CPURST_Msk;
    SYS->IPRST0 &= ~SYS_IPRST0_CPURST_Msk;
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
        printf(" Set system tick error!!\n");
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

void UART_Init(UART_T* uart, uint32_t u32ClkSrc, uint32_t u32ClkDiv, uint32_t u32BaudRate)
{
    uint32_t u32ClkTbl[4] = {__HXT, 0, __LXT, __HIRC};

    /* UART clock source and clock divider setting */

    CLK->CLKSEL2 = (CLK->CLKSEL2 & (~CLK_CLKSEL1_UART1SEL_Msk)) | (u32ClkSrc << CLK_CLKSEL1_UART1SEL_Pos);
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


void Download(void)
{
    int32_t  i32Err;

    printf("\n Bank%d processing, download data to Bank%d APP Base.\n\n\n", s_u32ExecBank, s_u32ExecBank);

    EnableSysTick(1000);
    StartTimer0();

    /* Dual bank background program address */
    s_u32DbAddr   = APP_BASE;

    /* Dual bank background program length */
    s_u32DbLength = APP_SIZE;


    EnableSysTick(1000);
    StartTimer0();

    /* Use Xmodem to download firmware from PC*/
    i32Err = Xmodem(s_u32DbAddr);

    if(i32Err < 0)
    {
        printf("\nXmodem transfer fail!\n");
        while(1);
    }
    else
    {
        printf("\nXomdem transfer done!\n");
        printf("Total transfer size is %d\n", i32Err);
    }

    printf("\n Firmware download completed!!\n");
}


int main()
{
    uint8_t u8GetCh;
    uint32_t i;
    uint32_t u32Loader0ChkSum, u32Loader1ChkSum;
    uint32_t u32App0ChkSum, u32App1ChkSum;

    /* Disable register write-protection function */
    SYS_UnlockReg();

    /* Initial clocks and multi-functions */
    SYS_Init();

    /* Set Vector Table Offset Register */
    SCB->VTOR = LOADER_BASE;

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

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

        /* Check CPU run at Bank0 or Bank1*/
        s_u32ExecBank = (uint32_t)((FMC->ISPSTS & FMC_ISPSTS_FBS_Msk) >> FMC_ISPSTS_FBS_Pos);
        printf("\n BANK%d Loader processing \n\n", s_u32ExecBank);

        /* Get loader CRC */
        u32Loader0ChkSum = FMC_GetChkSum(FMC_APROM_BASE, LOADER_SIZE);
        u32Loader1ChkSum = FMC_GetChkSum(FMC_APROM_BANK0_END, LOADER_SIZE);
        printf(" Loader0 checksum: 0x%08x \n Loader1 checksum: 0x%08x\n", u32Loader0ChkSum, u32Loader1ChkSum);

        /* Get app CRC */
        u32App0ChkSum = FMC_GetChkSum(FMC_APROM_BASE + APP_BASE, APP_SIZE);
        u32App1ChkSum = FMC_GetChkSum(FMC_APROM_BANK0_END + APP_BASE, APP_SIZE);
        printf(" App0 checksum: 0x%08x \n App1 checksum: 0x%08x\n", u32App0ChkSum, u32App1ChkSum);

        /* Write firmware CRC in CRC base for following checking */
        printf("\n Firmware CRC in [0x%x] is [0x%x]\n", FW_CRC_BASE, FMC_Read(FW_CRC_BASE));
        if(FMC_Read(FW_CRC_BASE) == 0xFFFFFFFF)
        {

            FMC_Write(FW_CRC_BASE, u32App0ChkSum);
            printf("\n Update Firmware CRC in [0x%x] is [0x%x]\n", FW_CRC_BASE, FMC_Read(FW_CRC_BASE));
        }

        /* Write backup firmware CRC in backup CRC base for following checking */
        printf("\n Backup Firmware CRC in [0x%x] is [0x%x]\n", BACKUP_FW_CRC_BASE, FMC_Read(BACKUP_FW_CRC_BASE));
        if(FMC_Read(BACKUP_FW_CRC_BASE) == 0xFFFFFFFF)
        {

            FMC_Write(BACKUP_FW_CRC_BASE, u32App1ChkSum);
            printf("\n Update Firmware CRC in [0x%x] is [0x%x]\n", BACKUP_FW_CRC_BASE, FMC_Read(BACKUP_FW_CRC_BASE));
        }

        /* Create the other bank loader for executing bank remap */
        if((s_u32ExecBank == 0) && (u32Loader0ChkSum != u32Loader1ChkSum))
        {
            printf("\n Create BANK%d Loader... \n",  s_u32ExecBank ^ 1);

            /* Erase loader region */
            for(i = LOADER_BASE; i < (LOADER_BASE + LOADER_SIZE); i += FMC_FLASH_PAGE_SIZE)
            {
                FMC_Erase(FMC_BANK_SIZE * (s_u32ExecBank ^ 1) + i);
            }
            /* Create loader in the other bank */
            for(i = LOADER_BASE; i < (LOADER_BASE + LOADER_SIZE); i += 4)
            {
                FMC_Write(FMC_BANK_SIZE * (s_u32ExecBank ^ 1) + i, FMC_Read((FMC_BANK_SIZE * s_u32ExecBank) + i));
            }
            printf(" Create Bank%d Loader completed! \n", (s_u32ExecBank ^ 1));
        }

#if CREATE_BANK1_APP
        if((s_u32ExecBank == 0) && ((FMC_CheckAllOne((FMC_APROM_BANK0_END + APP_BASE), APP_SIZE)) == READ_ALLONE_YES))
        {
            printf("\n Create BANK%d App... \n", s_u32ExecBank ^ 1);

            /* Erase app region */
            for(i = APP_BASE; i < (APP_BASE + APP_SIZE); i += FMC_FLASH_PAGE_SIZE)
            {
                FMC_Erase(FMC_BANK_SIZE * (s_u32ExecBank ^ 1) + i);
            }
            /* Create app in the other bank(just for test)*/
            for(i = APP_BASE; i < (APP_BASE + APP_SIZE); i += 4)
            {
                FMC_Write(FMC_BANK_SIZE * (s_u32ExecBank ^ 1) + i, FMC_Read((FMC_BANK_SIZE * s_u32ExecBank) + i));
            }
            printf(" Create Bank%d App completed! \n", (s_u32ExecBank ^ 1));
        }
#endif
        /* To check if system has been reset by WDT time-out reset or not */
        if(WDT_GET_RESET_FLAG() == 1)
        {
            WDT_CLEAR_RESET_FLAG();
            printf("\n === System reset by WDT time-out event === \n");
            printf(" Any key to remap back to backup FW\n");
            getchar();

            /* Remap to Bank1 to execute backup firmware */
            FMC_RemapBank(1);
            s_u32ExecBank = (uint32_t)((FMC->ISPSTS & FMC_ISPSTS_FBS_Msk) >> FMC_ISPSTS_FBS_Pos);
            printf("\n BANK%d Loader after remap  \n\n", s_u32ExecBank);

            /* Remap to App */
            FMC_SetVectorPageAddr(APP_BASE);
            ResetCPU();
        }

        printf("\n Execute BANK%d APP? [y/n] \n", s_u32ExecBank);
        u8GetCh = (uint8_t)getchar();
        printf("\n User select [%c] \n", u8GetCh);

        if(u8GetCh == 'y')
        {
            /* Remap to App */
            FMC_SetVectorPageAddr(APP_BASE);
            ResetCPU();
        }
        else
        {

            printf("\n Download new firmware? [y/n] \n");
            u8GetCh = (uint8_t)getchar();
            printf("\n User select [%c] \n", u8GetCh);

            if(u8GetCh == 'y')
            {
                /* Download new firmware */
                Download();
                printf("\n Any key to execute new firmware \n");
                getchar();
                /* Remap to App */
                FMC_SetVectorPageAddr(APP_BASE);
                ResetCPU();
            }
            else
            {
                /* Remap to Loader */
                FMC_SetVectorPageAddr(LOADER_BASE);
                ResetCPU();
            }

        }

    }
    while(1);

}
