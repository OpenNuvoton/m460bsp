/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate the RTC static tamper function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Functions and variables declaration                                                                     */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint32_t g_u32IsTamper = FALSE;

void TAMPER_IRQHandler(void);
void SYS_Init(void);
void UART_Init(void);


/**
 * @brief       IRQ Handler for RTC TAMPER Interrupt
 * @param       None
 * @return      None
 * @details     The TAMPER_IRQHandler is default IRQ of RTC TAMPER, declared in startup_m460.s.
 */
void TAMPER_IRQHandler(void)
{
    uint32_t i;
    uint32_t u32FlagStatus, u32TAMPCAL, u32TAMPTIME;

    /* Tamper interrupt occurred */
    if(RTC_GET_TAMPER_INT_FLAG())
    {
        u32FlagStatus = RTC_GET_TAMPER_INT_STATUS();

        for(i = 0; i < 6; i++)
        {
            if(u32FlagStatus & (0x1UL << (i + RTC_INTSTS_TAMP0IF_Pos)))
                printf(" Tamper %d Detected!!\n", i);
        }

        u32TAMPCAL = RTC->TAMPCAL;
        u32TAMPTIME = RTC->TAMPTIME;
        printf(" Tamper detected date/time: 20%d%d/%d%d/%d%d ",
               (uint32_t)((u32TAMPCAL & RTC_TAMPCAL_TENYEAR_Msk) >> RTC_TAMPCAL_TENYEAR_Pos),
               (uint32_t)((u32TAMPCAL & RTC_TAMPCAL_YEAR_Msk) >> RTC_TAMPCAL_YEAR_Pos),
               (uint32_t)((u32TAMPCAL & RTC_TAMPCAL_TENMON_Msk) >> RTC_TAMPCAL_TENMON_Pos),
               (uint32_t)((u32TAMPCAL & RTC_TAMPCAL_MON_Msk) >> RTC_TAMPCAL_MON_Pos),
               (uint32_t)((u32TAMPCAL & RTC_TAMPCAL_TENDAY_Msk) >> RTC_TAMPCAL_TENDAY_Pos),
               (uint32_t)((u32TAMPCAL & RTC_TAMPCAL_DAY_Msk) >> RTC_TAMPCAL_DAY_Pos));
        printf("%d%d:%d%d:%d%d.\n\n",
               (uint32_t)((u32TAMPTIME & RTC_TAMPTIME_TENHR_Msk) >> RTC_TAMPTIME_TENHR_Pos),
               (uint32_t)((u32TAMPTIME & RTC_TAMPTIME_HR_Msk) >> RTC_TAMPTIME_HR_Pos),
               (uint32_t)((u32TAMPTIME & RTC_TAMPTIME_TENMIN_Msk) >> RTC_TAMPTIME_TENMIN_Pos),
               (uint32_t)((u32TAMPTIME & RTC_TAMPTIME_MIN_Msk) >> RTC_TAMPTIME_MIN_Pos),
               (uint32_t)((u32TAMPTIME & RTC_TAMPTIME_TENSEC_Msk) >> RTC_TAMPTIME_TENSEC_Pos),
               (uint32_t)((u32TAMPTIME & RTC_TAMPTIME_SEC_Msk) >> RTC_TAMPTIME_SEC_Pos));

        RTC_CLEAR_TAMPER_INT_FLAG(u32FlagStatus);
        g_u32IsTamper = TRUE;
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


    /*---------------------------------------------------------------------------------------------------------*/
    /* Initialization for sample code                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable LXT and Waiting for clock ready */
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Enable RTC module clock */
    CLK_EnableModuleClock(RTC_MODULE);
    
    /* Set multi-function pins for RTC Tamper */
    SET_TAMPER2_PF8();
    SET_TAMPER3_PF9();
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
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    S_RTC_TIME_DATA_T sInitTime, sGetTime;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+------------------------------------+\n");
    printf("|    RTC Static Tamper Sample Code   |\n");
    printf("+------------------------------------+\n\n");

    /* Set LXT as RTC clock source */
    RTC_SetClockSource(RTC_CLOCK_SOURCE_LXT);

    /* Open RTC and start counting */
    sInitTime.u32Year       = 2021;
    sInitTime.u32Month      = 10;
    sInitTime.u32Day        = 10;
    sInitTime.u32Hour       = 12;
    sInitTime.u32Minute     = 12;
    sInitTime.u32Second     = 0;
    sInitTime.u32DayOfWeek  = RTC_SUNDAY;
    sInitTime.u32TimeScale  = RTC_CLOCK_24;
    if(RTC_Open(&sInitTime) != 0)
    {
        printf("\n RTC initial fail!!");
        printf("\n Please check h/w setting!!");
        goto lexit;
    }

    RTC_GetDateAndTime(&sGetTime);
    printf("# Initial data/time is: %d/%02d/%02d %02d:%02d:%02d.\n",
           sGetTime.u32Year, sGetTime.u32Month, sGetTime.u32Day, sGetTime.u32Hour, sGetTime.u32Minute, sGetTime.u32Second);
    printf("# Please connect TAMPER2/3(PF.8/9) pins to High first.\n");
    printf("# Press any key to start test: (Pull TAMPER2/3 pin Low to generate tamper event)\n\n");
    getchar();

    printf("# Check tamper date/time when tamper event occurred:\n\n");

    RTC_CLEAR_TAMPER_INT_FLAG(RTC_INTSTS_TAMP0IF_Msk | RTC_INTSTS_TAMP1IF_Msk | RTC_INTSTS_TAMP2IF_Msk |
                              RTC_INTSTS_TAMP3IF_Msk | RTC_INTSTS_TAMP4IF_Msk | RTC_INTSTS_TAMP5IF_Msk);

    RTC_StaticTamperEnable(RTC_TAMPER2_SELECT | RTC_TAMPER3_SELECT, RTC_TAMPER_HIGH_LEVEL_DETECT,
                           RTC_TAMPER_DEBOUNCE_ENABLE);

    g_u32IsTamper = FALSE;

    /* Enable RTC Tamper Interrupt */
    RTC_EnableInt(RTC_INTEN_TAMP2IEN_Msk | RTC_INTEN_TAMP3IEN_Msk);
    NVIC_EnableIRQ(TAMPER_IRQn);

    while(1)
    {
        while(g_u32IsTamper == FALSE) {}
        g_u32IsTamper = FALSE;
    }

lexit:

    while(1);
}
