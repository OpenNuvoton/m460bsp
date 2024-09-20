/**************************************************************************//**
 * @file     PartTest.c
 * @version  V3.00
 * @brief    IRQ handlers and system initialzation
 *
 * @copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/


/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"

/* Hardware includes. */
#include "NuMicro.h"

/* Standard demo include. */
#include "partest.h"

extern int IsDebugFifoEmpty(void);

volatile uint8_t g_u8IsRTCAlarmINT = 0, g_u8IsGPIOINT = 0;
volatile uint32_t u32SubSecCntRecord = 0, u32SecCntRecord = 0;

S_RTC_TIME_DATA_T sWriteRTC, sReadRTC;
volatile uint32_t g_u32WriteRTC_Ticks, g_u32ReadRTC_Ticks;

/*-----------------------------------------------------------*/

/**
 * @brief       IRQ Handler for RTC Interrupt
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The RTC_IRQHandler is default IRQ of RTC, declared in startup_m460.s.
 */
void RTC_IRQHandler(void)
{
    /* To check if RTC alarm interrupt occurred */
    if(RTC_GET_ALARM_INT_FLAG() == 1)
    {
        /* Clear RTC alarm interrupt flag */
        RTC_CLEAR_ALARM_INT_FLAG();

        /* Read current RTC date/time/ticks */
        RTC_GetDateAndTime(&sReadRTC);
        g_u32ReadRTC_Ticks = ((RTC->TIME & RTC_TIME_HZCNT_Msk) >> RTC_TIME_HZCNT_Pos);

        printf("RTC Alarm: %d/%02d/%02d %02d:%02d:%02d.%02d\n",
               sReadRTC.u32Year, sReadRTC.u32Month, sReadRTC.u32Day, sReadRTC.u32Hour, sReadRTC.u32Minute, sReadRTC.u32Second, g_u32ReadRTC_Ticks * 100 / 128);

        g_u8IsRTCAlarmINT++;
    }

    /* To check if RTC tick interrupt occurred */
    if(RTC_GET_TICK_INT_FLAG() == 1)
    {
        /* Clear RTC tick interrupt flag */
        RTC_CLEAR_TICK_INT_FLAG();
    }
}

/**
 * @brief       GPIO PB IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PB default IRQ, declared in startup_m460.s.
 */
void GPB_IRQHandler(void)
{
    if(GPIO_GET_INT_FLAG(PB, BIT1))      /* To check if PB.1 interrupt occurred */
    {
        GPIO_CLR_INT_FLAG(PB, BIT1);
        printf("CPU woken up by external interrupt (PB.1 INT occurred).\n");

        g_u8IsGPIOINT ++;
    }
    else if(GPIO_GET_INT_FLAG(PB, BIT2)) /* To check if PB.2 interrupt occurred */
    {
        GPIO_CLR_INT_FLAG(PB, BIT2);
        printf("CPU woken up by external interrupt (PB.2 INT occurred).\n");

        g_u8IsGPIOINT ++;
    }
    else if(GPIO_GET_INT_FLAG(PB, BIT3)) /* To check if PB.3 interrupt occurred */
    {
        GPIO_CLR_INT_FLAG(PB, BIT3);
        printf("CPU woken up by external interrupt (PB.3 INT occurred).\n");

        g_u8IsGPIOINT ++;
    }
    else if(GPIO_GET_INT_FLAG(PB, BIT4)) /* To check if PB.4 interrupt occurred */
    {
        GPIO_CLR_INT_FLAG(PB, BIT4);
        printf("CPU woken up by external interrupt (PB.4 INT occurred).\n");

        g_u8IsGPIOINT ++;
    }
    else if(GPIO_GET_INT_FLAG(PB, BIT5)) /* To check if PB.5 interrupt occurred */
    {
        GPIO_CLR_INT_FLAG(PB, BIT5);
        printf("CPU woken up by external interrupt (PB.5 INT occurred).\n");

        g_u8IsGPIOINT ++;
    }
    else
    {
        /* Un-expected interrupt. Just clear all PB interrupts */
        PB->INTSRC = PB->INTSRC;
        printf("Un-expected interrupts.\n");
    }

    /* Read current RTC date/time */
    RTC_GetDateAndTime(&sReadRTC);
    g_u32ReadRTC_Ticks = ((RTC->TIME & RTC_TIME_HZCNT_Msk) >> RTC_TIME_HZCNT_Pos);

//    printf("IO wakup up time: %d/%02d/%02d %02d:%02d:%02d.%02d\n",
//            sReadRTC.u32Year, sReadRTC.u32Month, sReadRTC.u32Day, sReadRTC.u32Hour, sReadRTC.u32Minute, sReadRTC.u32Second, g_u32ReadRTC_Ticks * 100 / 128);
}

void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt;

    /* Select Power-down mode */
    CLK_SetPowerDownMode(CLK_PMUCTL_PDMSEL_LLPD);

    /* To check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(IsDebugFifoEmpty() == 0)
        if(--u32TimeOutCnt == 0) break;

    /* Enter to Power-down mode */
    CLK_PowerDown();
}

void vParTestInitialise(void)
{
    uint32_t u32Pin;

    SYS_UnlockReg();

    /* Set PF multi-function pins for X32_OUT(PF.4) and X32_IN(PF.5) */
    SET_X32_OUT_PF4();
    SET_X32_IN_PF5();

    /* Enable HIRC and LXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_LXTEN_Msk);

    /* Wait for HIRC and LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_LXTSTB_Msk);

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 200MHz */
    CLK_SetCoreClock(200000000);

    /* Enable GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART module clock source as HIRC and UART module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(DEBUG_PORT, 115200);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set PD multi-function pins for CLKO(PD.12) */
    SET_CLKO_PD12();

    /* Output selected clock to CKO, CKO Clock = HCLK / 2^(5 + 1) */
    /* CLKO could be used to monitor MCU power down or not. */
    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_HCLK, 5, 0);

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+-------------------------------------+\n");
    printf("|    FreeRTOS Tickless Sample Code    |\n");
    printf("+-------------------------------------+\n");
    printf("CLKO(PD.12) can monitor if the CPU enters Power-Down mode.\n");
    printf("Please toggle PB.1~5 to wake up the CPU.\n\n");

#if (configUSE_TICKLESS_IDLE == 1)
    /* Enable RTC module clock */
    CLK_EnableModuleClock(RTC_MODULE);

    /* Enable RTC NVIC */
    NVIC_EnableIRQ(RTC_IRQn);

    /* Open RTC and start counting */
    sWriteRTC.u32Year       = 2023;
    sWriteRTC.u32Month      = 4;
    sWriteRTC.u32Day        = 20;
    sWriteRTC.u32Hour       = 0;
    sWriteRTC.u32Minute     = 0;
    sWriteRTC.u32Second     = 0;
    sWriteRTC.u32DayOfWeek  = RTC_FRIDAY;
    sWriteRTC.u32TimeScale  = RTC_CLOCK_24;

    if(RTC_Open(&sWriteRTC) != 0)
    {
        printf("\n RTC initial fail!!");
        printf("\n Please check h/w setting!!");
        while(1);
    }

    /* Enable sub-second counter */
    RTC->CLKFMT |= RTC_CLKFMT_HZCNTEN_Msk;
#endif /* configUSE_TICKLESS_IDLE */

    /* Configure PB.1~5 as Input mode and enable interrupt by rising or falling edge trigger */
    GPIO_SetMode(PB, (BIT1 | BIT2 | BIT3 | BIT4 | BIT5), GPIO_MODE_INPUT);
    for(u32Pin = 1; u32Pin < 6; u32Pin++)
        GPIO_EnableInt(PB, u32Pin, GPIO_INT_BOTH_EDGE);
    NVIC_EnableIRQ(GPB_IRQn);

    /* LED control */
    GPIO_SetMode(PH, (BIT4), GPIO_MODE_OUTPUT);

    /* Enable interrupt de-bounce function and select de-bounce sampling cycle time is 1024 clocks of LIRC clock */
    GPIO_SET_DEBOUNCE_TIME(PB, GPIO_DBCTL_DBCLKSRC_LIRC, GPIO_DBCTL_DBCLKSEL_1024);
    GPIO_ENABLE_DEBOUNCE(PB, (BIT1 | BIT2 | BIT3 | BIT4 | BIT5));

    /* Unlock protected registers before entering Power-down mode */
    SYS_UnlockReg();
}
