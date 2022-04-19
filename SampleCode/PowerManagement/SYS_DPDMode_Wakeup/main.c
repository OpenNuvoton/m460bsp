/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to wake up system from DPD Power-down mode by Wake-up pin(PC.0),
 *           Wake-up Timer, RTC Tick, RTC Alarm or RTC Tamper 0.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*
//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
*/

/*
// <o0> LVR
//      <0=> Disable
//      <1=> Enable
*/
#define SET_LVR       0

/*
// <o0> POR
//      <0=> Disable
//      <1=> Enable
*/
#define SET_POR       0

/*
// <o0> LIRC
//      <0=> Disable
//      <1=> Enable
*/
#define SET_LIRC       0

/*
// <o0> LXT
//      <0=> Disable
//      <1=> Enable
*/
#define SET_LXT       0



void PowerDownFunction(void);
void WakeUpPinFunction(uint32_t u32PDMode, uint32_t u32EdgeType);
void WakeUpTimerFunction(uint32_t u32PDMode, uint32_t u32Interval);
void WakeUpRTCTickFunction(uint32_t u32PDMode);
void WakeUpRTCAlarmFunction(uint32_t u32PDMode);
void WakeUpRTCTamperFunction(uint32_t u32PDMode);
void CheckPowerSource(void);
void GpioPinSetting(void);
void GpioPinSettingRTC(void);
int32_t LvrSetting(void);
void PorSetting(void);
int32_t LircSetting(void);
int32_t LxtSetting(void);
void SYS_Init(void);
void UART0_Init(void);



/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt;

    /* Check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    UART_WAIT_TX_EMPTY(DEBUG_PORT)
        if(--u32TimeOutCnt == 0) break;

    /* Enter to Power-down mode */
    CLK_PowerDown();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode and Wake up source by Wake-up pin                         */
/*---------------------------------------------------------------------------------------------------------*/
void WakeUpPinFunction(uint32_t u32PDMode, uint32_t u32EdgeType)
{
    printf("Enter to DPD Power-down mode......\n");

    /* Select Power-down mode */
    CLK_SetPowerDownMode(u32PDMode);

    /* Configure GPIO as input mode */
    GPIO_SetMode(PC, BIT0, GPIO_MODE_INPUT);

    /* Set Wake-up pin trigger type at Deep Power-down mode */
    CLK_EnableDPDWKPin(u32EdgeType);

    /* Enter to Power-down mode and wait for wake-up reset happen */
    PowerDownFunction();
}

/*-----------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode and Wake up source by Wake-up Timer                         */
/*-----------------------------------------------------------------------------------------------------------*/
void WakeUpTimerFunction(uint32_t u32PDMode, uint32_t u32Interval)
{
    printf("Enable LIRC for wake-up source.\n");

    /* Enable LIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);

    /* Wait for LIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk);

    printf("Enter to DPD Power-down mode......\n");

    /* Select Power-down mode */
    CLK_SetPowerDownMode(u32PDMode);

    /* Set Wake-up Timer Time-out Interval */
    CLK_SET_WKTMR_INTERVAL(u32Interval);

    /* Enable Wake-up Timer */
    CLK_ENABLE_WKTMR();

    /* Enter to Power-down mode and wait for wake-up reset happen */
    PowerDownFunction();
}

/*-----------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode and Wake up source by RTC Tick                              */
/*-----------------------------------------------------------------------------------------------------------*/
void RTC_IRQHandler(void);
void RTC_IRQHandler(void)
{
    /* Clear RTC interrupt flag */
    uint32_t u32INTSTS = RTC->INTSTS;
    RTC->INTSTS = u32INTSTS;
}

void WakeUpRTCTickFunction(uint32_t u32PDMode)
{
    printf("Enable LXT for RTC wake-up source.\n");

    /* Enable LXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Wait for LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Enable RTC peripheral clock */
    CLK->APBCLK0 |= CLK_APBCLK0_RTCCKEN_Msk;

    /* RTC clock source select LXT */
    CLK_SetModuleClock(RTC_MODULE, RTC_LXTCTL_RTCCKSEL_LXT, (uint32_t)NULL);

    /* Open RTC and start counting */
    if( RTC_Open(NULL) < 0 )
    {
        printf("Initialize RTC module and start counting failed\n");
        return;
    }

    /* Clear tick status */
    RTC_CLEAR_TICK_INT_FLAG();

    /* Enable RTC Tick interrupt */
    RTC_EnableInt(RTC_INTEN_TICKIEN_Msk);
    NVIC_EnableIRQ(RTC_IRQn);

    /* Select Power-down mode */
    CLK_SetPowerDownMode(u32PDMode);

    /* Set RTC tick period as 1 second */
    RTC_SetTickPeriod(RTC_TICK_1_SEC);

    /* Set PF.4 ~ PF.11 I/O state by RTC control to prevent floating */
    GpioPinSettingRTC();

    /* Enable RTC wake-up */
    CLK_ENABLE_RTCWK();

    /* Enter to Power-down mode and wait for wake-up reset happen */
    printf("Enter to DPD Power-down mode......\n");
    PowerDownFunction();
}


/*-----------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode and Wake up source by RTC Alarm                             */
/*-----------------------------------------------------------------------------------------------------------*/
void  WakeUpRTCAlarmFunction(uint32_t u32PDMode)
{
    S_RTC_TIME_DATA_T sWriteRTC;

    printf("Enable LXT for RTC wake-up source.\n");

    /* Enable LXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Wait for LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Enable RTC peripheral clock */
    CLK->APBCLK0 |= CLK_APBCLK0_RTCCKEN_Msk;

    /* RTC clock source select LXT */
    CLK_SetModuleClock(RTC_MODULE, RTC_LXTCTL_RTCCKSEL_LXT, (uint32_t)NULL);

    /* Open RTC and start counting */
    sWriteRTC.u32Year       = 2021;
    sWriteRTC.u32Month      = 5;
    sWriteRTC.u32Day        = 11;
    sWriteRTC.u32DayOfWeek  = 2;
    sWriteRTC.u32Hour       = 15;
    sWriteRTC.u32Minute     = 4;
    sWriteRTC.u32Second     = 10;
    sWriteRTC.u32TimeScale  = 1;
    if( RTC_Open(&sWriteRTC) < 0 )
    {
        printf("Initialize RTC module and start counting failed\n");
        return;
    }

    /* Set RTC alarm date/time */
    sWriteRTC.u32Year       = 2021;
    sWriteRTC.u32Month      = 5;
    sWriteRTC.u32Day        = 11;
    sWriteRTC.u32DayOfWeek  = 2;
    sWriteRTC.u32Hour       = 15;
    sWriteRTC.u32Minute     = 4;
    sWriteRTC.u32Second     = 15;
    RTC_SetAlarmDateAndTime(&sWriteRTC);

    printf("# Set RTC current date/time: 2021/05/11 15:04:10.\n");
    printf("# Set RTC alarm date/time:   2021/05/11 15:04:%d.\n", sWriteRTC.u32Second);
    printf("Enter to DPD Power-down mode......\n");

    /* clear alarm status */
    RTC_CLEAR_ALARM_INT_FLAG();

    /* Enable RTC alarm interrupt */
    RTC_EnableInt(RTC_INTEN_ALMIEN_Msk);
    NVIC_EnableIRQ(RTC_IRQn);

    /* Select Power-down mode */
    CLK_SetPowerDownMode(u32PDMode);

    /* Set PF.4 ~ PF.11 I/O state by RTC control to prevent floating */
    GpioPinSettingRTC();

    /* Enable RTC wake-up */
    CLK_ENABLE_RTCWK();

    /* Enter to Power-down mode and wait for wake-up reset happen */
    PowerDownFunction();
}

/*-----------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode and Wake up source by RTC Tamper                            */
/*-----------------------------------------------------------------------------------------------------------*/
void TAMPER_IRQHandler(void);
void TAMPER_IRQHandler(void)
{
    /* Clear RTC interrupt flag */
    uint32_t u32INTSTS = RTC->INTSTS;
    RTC->INTSTS = u32INTSTS;
}

void  WakeUpRTCTamperFunction(uint32_t u32PDMode)
{
    printf("Enable LXT for RTC wake-up source.\n");

    /* Enable LXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Wait for LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

    /* Enable RTC peripheral clock */
    CLK->APBCLK0 |= CLK_APBCLK0_RTCCKEN_Msk;

    /* RTC clock source select LXT */
    CLK_SetModuleClock(RTC_MODULE, RTC_LXTCTL_RTCCKSEL_LXT, (uint32_t)NULL);

    /* Open RTC and start counting */
    if( RTC_Open(NULL) < 0 )
    {
        printf("Initialize RTC module and start counting failed\n");
        return;
    }

    /* Set RTC Tamper 0 as low level detect */
    RTC_StaticTamperEnable(RTC_TAMPER0_SELECT, RTC_TAMPER_LOW_LEVEL_DETECT, RTC_TAMPER_DEBOUNCE_DISABLE);

    /* Clear Tamper 0 status */
    RTC_CLEAR_TAMPER_INT_FLAG(RTC_INTSTS_TAMP0IF_Msk);

    /* Disable Spare Register */
    RTC->SPRCTL = RTC_SPRCTL_SPRCSTS_Msk;

    /* Enable RTC Tamper 0 */
    RTC_EnableInt(RTC_INTEN_TAMP0IEN_Msk);
    NVIC_EnableIRQ(TAMPER_IRQn);

    /* Select Power-down mode */
    CLK_SetPowerDownMode(u32PDMode);

    /* Set PF.4 ~ PF.11 I/O state by RTC control to prevent floating */
    GpioPinSettingRTC();

    /* Enable RTC wake-up */
    CLK_ENABLE_RTCWK();

    /* Enter to Power-down mode and wait for wake-up reset happen */
    printf("Enter to DPD Power-down mode......\n");
    PowerDownFunction();
}

/*-----------------------------------------------------------------------------------------------------------*/
/*  Function for Check Power Manager Status                                                                  */
/*-----------------------------------------------------------------------------------------------------------*/
void CheckPowerSource(void)
{
    uint32_t u32RegRstsrc;
    u32RegRstsrc = CLK_GetPMUWKSrc();

    printf("Power manager Power Manager Status 0x%x\n", u32RegRstsrc);

    if((u32RegRstsrc & CLK_PMUSTS_RTCWK_Msk) != 0)
        printf("Wake-up source is RTC.\n");
    if((u32RegRstsrc & CLK_PMUSTS_TMRWK_Msk) != 0)
        printf("Wake-up source is Wake-up Timer.\n");
    if((u32RegRstsrc & CLK_PMUSTS_PINWK0_Msk) != 0)
        printf("Wake-up source is Wake-up Pin.\n");
    if((u32RegRstsrc & CLK_PMUSTS_RSTWK_Msk) != 0)
        printf("Wake-up source is Pin Reset.\n");

    /* Clear all wake-up flag */
    CLK->PMUSTS |= CLK_PMUSTS_CLRWK_Msk;
}

/*-----------------------------------------------------------------------------------------------------------*/
/*  Function for GPIO, LVR, POR, LIRC and LXT setting for power consumption                                  */
/*-----------------------------------------------------------------------------------------------------------*/
void GpioPinSetting(void)
{
    /* Set all GPIOs are quasi mode */
    PA->MODE = 0xFFFFFFFF;
    PB->MODE = 0xFFFFFFFF;
    PC->MODE = 0xFFFFFFFF;
    PD->MODE = 0xFFFFFFFF;
    PE->MODE = 0xFFFFFFFF;
    PF->MODE = 0xFFFFFFFF;
    PG->MODE = 0xFFFFFFFF;
    PH->MODE = 0xFFFFFFFF;
    PI->MODE = 0xFFFFFFFF;
    PJ->MODE = 0xFFFFFFFF;

    /* Set all GPIOs are output high */
    PA->DOUT = 0x0000FFFF;
    PB->DOUT = 0x0000FFFF;
    PC->DOUT = 0x0000FFFF;
    PD->DOUT = 0x0000FFFF;
    PE->DOUT = 0x0000FFFF;
    PF->DOUT = 0x0000FFFF;
    PG->DOUT = 0x0000FFFF;
    PH->DOUT = 0x0000FFFF;
    PI->DOUT = 0x0000FFFF;
    PJ->DOUT = 0x0000FFFF;
}

void GpioPinSettingRTC(void)
{
    /* Set PF.4 ~ PF.11 as Quasi mode output high by RTC control */
    CLK->APBCLK0 |= CLK_APBCLK0_RTCCKEN_Msk;
    RTC->GPIOCTL1 = RTC_GPIOCTL1_DOUT7_Msk | (RTC_IO_MODE_QUASI<<RTC_GPIOCTL1_OPMODE7_Pos) |
                    RTC_GPIOCTL1_DOUT6_Msk | (RTC_IO_MODE_QUASI<<RTC_GPIOCTL1_OPMODE6_Pos) |
                    RTC_GPIOCTL1_DOUT5_Msk | (RTC_IO_MODE_QUASI<<RTC_GPIOCTL1_OPMODE5_Pos) |
                    RTC_GPIOCTL1_DOUT4_Msk | (RTC_IO_MODE_QUASI<<RTC_GPIOCTL1_OPMODE4_Pos);
    RTC->GPIOCTL0 = RTC_GPIOCTL0_DOUT3_Msk | (RTC_IO_MODE_QUASI<<RTC_GPIOCTL0_OPMODE3_Pos) |
                    RTC_GPIOCTL0_DOUT2_Msk | (RTC_IO_MODE_QUASI<<RTC_GPIOCTL0_OPMODE2_Pos) |
                    RTC_GPIOCTL0_DOUT1_Msk | (RTC_IO_MODE_QUASI<<RTC_GPIOCTL0_OPMODE1_Pos) |
                    RTC_GPIOCTL0_DOUT0_Msk | (RTC_IO_MODE_QUASI<<RTC_GPIOCTL0_OPMODE0_Pos);
    CLK->APBCLK0 &= ~CLK_APBCLK0_RTCCKEN_Msk;
}

int32_t LvrSetting(void)
{
    uint32_t u32TimeOutCnt;

    if(SET_LVR == 0)
    {
        SYS_DISABLE_LVR();
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while( SYS->BODCTL & SYS_BODCTL_LVRRDY_Msk )
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for LVR disable time-out!\n");
                return -1;
            }
        }
    }
    else
    {
        SYS_ENABLE_LVR();
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while( (SYS->BODCTL & SYS_BODCTL_LVRRDY_Msk) == 0 )
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for LVR enable time-out!\n");
                return -1;
            }
        }
    }

    return 0;
}

void PorSetting(void)
{
    if(SET_POR == 0)
    {
        SYS_DISABLE_POR();
    }
    else
    {
        SYS_ENABLE_POR();
    }
}

int32_t LircSetting(void)
{
    uint32_t u32TimeOutCnt;

    if(SET_LIRC == 0)
    {
        CLK_DisableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while( CLK->STATUS & CLK_STATUS_LIRCSTB_Msk )
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for LIRC disable time-out!\n");
                return -1;
            }
        }
    }
    else
    {
        CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
        if( CLK_WaitClockReady(CLK_PWRCTL_LIRCEN_Msk) == 0)
        {
            printf("Wait for LIRC enable time-out!\n");
            return -1;
        }
    }

    return 0;
}

int32_t LxtSetting(void)
{
    uint32_t u32TimeOutCnt;

    if(SET_LXT == 0)
    {
        CLK_DisableXtalRC(CLK_PWRCTL_LXTEN_Msk);
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while( CLK->STATUS & CLK_STATUS_LXTSTB_Msk )
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for LXT disable time-out!\n");
                return -1;
            }
        }
    }
    else
    {
        CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
        if( CLK_WaitClockReady(CLK_PWRCTL_LXTEN_Msk) == 0)
        {
            printf("Wait for LXT enable time-out!\n");
            return -1;
        }
    }

    return 0;
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

    /* Set PC multi-function pin for CLKO(PC.13) */
    SET_CLKO_PC13();

    /* Set PF multi-function pin for TAMPER0(PF.6) */
    SET_TAMPER0_PF6();

    /* Set PF multi-function pins for X32_OUT(PF.4) and X32_IN(PF.5) */
    SET_X32_OUT_PF4();
    SET_X32_IN_PF5();

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    uint8_t u8Item;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Unlock protected registers before setting Power-down mode */
    SYS_UnlockReg();

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_HCLK, 3, 0);

    /* Set I/O state to prevent floating */
    GpioPinSetting();

    /* Get power manager wake up source */
    CheckPowerSource();

    /* LVR setting */
    if( LvrSetting() < 0 ) return -1;

    /* POR setting */
    PorSetting();

    /* LIRC setting */
    if( LircSetting() < 0 ) return -1;

    /* LXT setting */
    if( LxtSetting() < 0 ) return -1;

    printf("+----------------------------------------------------------------+\n");
    printf("|    DPD Power-down Mode and Wake-up Sample Code.                |\n");
    printf("|    Please Select Wake up source.                               |\n");
    printf("+----------------------------------------------------------------+\n");
    printf("|[1] DPD Wake-up Pin(PC.0) trigger type is falling edge.         |\n");
    printf("|[2] DPD Wake-up TIMER time-out interval is 16384 LIRC clocks.   |\n");
    printf("|[3] DPD Wake-up by RTC Tick(1 second).                          |\n");
    printf("|[4] DPD Wake-up by RTC Alarm.                                   |\n");
    printf("|[5] DPD Wake-up by RTC Tamper0(PF.6).                           |\n");
    printf("|    Tamper pin detect voltage level is low.                     |\n");
    printf("+----------------------------------------------------------------+\n");
    u8Item = (uint8_t)getchar();

    switch(u8Item)
    {
        case '1':
            WakeUpPinFunction(CLK_PMUCTL_PDMSEL_DPD, CLK_DPDWKPIN0_FALLING);
            break;
        case '2':
            WakeUpTimerFunction(CLK_PMUCTL_PDMSEL_DPD, CLK_PMUCTL_WKTMRIS_16384);
            break;
        case '3':
            WakeUpRTCTickFunction(CLK_PMUCTL_PDMSEL_DPD);
            break;
        case '4':
            WakeUpRTCAlarmFunction(CLK_PMUCTL_PDMSEL_DPD);
            break;
        case '5':
            WakeUpRTCTamperFunction(CLK_PMUCTL_PDMSEL_DPD);
            break;
        default:
            break;
    }

    while(1);
}
