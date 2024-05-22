/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to minimize power consumption when entering power down mode.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2023 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*
//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
*/

/*
// <o0> Power-down Mode
//      <0=> PD
//      <1=> LLPD
//      <2=> FWPD
//      <4=> SPD
//      <6=> DPD
*/
#define SET_PDMSEL    0

/*
// <o0> SPD mode SRAM retention size
//      <0=> 0KB
//      <1=> 16KB
//      <2=> 32KB
//      <3=> 64KB
//      <4=> 128KB
//      <5=> 256KB
*/
#define SET_SRETSEL   0

/*
// <o0> LVR
//      <0=> Disable
//      <1=> Enable
*/
#define SET_LVR       1

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
#define SET_LIRC      0

/*
// <o0> LXT
//      <0=> Disable
//      <1=> Enable
*/
#define SET_LXT       0


#define GPIO_P0_TO_P15      0xFFFF


void PowerDownFunction(void);
void GPB_IRQHandler(void);
int32_t LvrSetting(void);
void PorSetting(void);
int32_t LircSetting(void);
int32_t LxtSetting(void);
void GpioPinSettingRTC(void);
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

    /* Select Power-down mode */
    CLK_SetPowerDownMode(SET_PDMSEL << CLK_PMUCTL_PDMSEL_Pos);

    /* Enter to Power-down mode */
    CLK_PowerDown();
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
    volatile uint32_t u32temp;

    /* To check if PB.2 interrupt occurred */
    if(GPIO_GET_INT_FLAG(PB, BIT2))
    {
        GPIO_CLR_INT_FLAG(PB, BIT2);
        printf("PB2 INT occurred.\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PB interrupts */
        u32temp = PB->INTSRC;
        PB->INTSRC = u32temp;
        printf("Un-expected interrupts.\n");
    }
}

int32_t LvrSetting(void)
{
    uint32_t u32TimeOutCnt;

    if(SET_LVR == 0)
    {
        /* Disable LVR and wait for LVR stable flag is cleared */
        SYS_DISABLE_LVR();
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while(SYS->BODCTL & SYS_BODCTL_LVRRDY_Msk)
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
        /* Enable LVR and wait for LVR stable flag is set */
        SYS_ENABLE_LVR();
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while((SYS->BODCTL & SYS_BODCTL_LVRRDY_Msk) == 0)
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
        /* Disable POR */
        SYS_DISABLE_POR();
    }
    else
    {
        /* Enable POR */
        SYS_ENABLE_POR();
    }
}

int32_t LircSetting(void)
{
    uint32_t u32TimeOutCnt;

    if(SET_LIRC == 0)
    {
        /* Disable LIRC and wait for LIRC stable flag is cleared */
        CLK_DisableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while(CLK->STATUS & CLK_STATUS_LIRCSTB_Msk)
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
        /* Enable LIRC and wait for LIRC stable flag is set */
        CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
        if(CLK_WaitClockReady(CLK_STATUS_LIRCSTB_Msk) == 0)
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
        /* Disable LXT and wait for LXT stable flag is cleared */
        CLK_DisableXtalRC(CLK_PWRCTL_LXTEN_Msk);
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while(CLK->STATUS & CLK_STATUS_LXTSTB_Msk)
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
        /* Enable LXT and wait for LXT stable flag is set */
        CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
        if(CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk) == 0)
        {
            printf("Wait for LXT enable time-out!\n");
            return -1;
        }
    }

    return 0;
}

void GpioPinSettingRTC(void)
{
    /* Set PF.4~PF.11 as Quasi mode output high by RTC control */
    RTC->GPIOCTL1 = RTC_GPIOCTL1_DOUT7_Msk | (RTC_IO_MODE_QUASI << RTC_GPIOCTL1_OPMODE7_Pos) |
                    RTC_GPIOCTL1_DOUT6_Msk | (RTC_IO_MODE_QUASI << RTC_GPIOCTL1_OPMODE6_Pos) |
                    RTC_GPIOCTL1_DOUT5_Msk | (RTC_IO_MODE_QUASI << RTC_GPIOCTL1_OPMODE5_Pos) |
                    RTC_GPIOCTL1_DOUT4_Msk | (RTC_IO_MODE_QUASI << RTC_GPIOCTL1_OPMODE4_Pos);
    RTC->GPIOCTL0 = RTC_GPIOCTL0_DOUT3_Msk | (RTC_IO_MODE_QUASI << RTC_GPIOCTL0_OPMODE3_Pos) |
                    RTC_GPIOCTL0_DOUT2_Msk | (RTC_IO_MODE_QUASI << RTC_GPIOCTL0_OPMODE2_Pos) |
                    RTC_GPIOCTL0_DOUT1_Msk | (RTC_IO_MODE_QUASI << RTC_GPIOCTL0_OPMODE1_Pos) |
                    RTC_GPIOCTL0_DOUT0_Msk | (RTC_IO_MODE_QUASI << RTC_GPIOCTL0_OPMODE0_Pos);
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
int main(void)
{
    uint32_t u32TimeOutCnt, u32PMUSTS;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Clear SPD/DPD mode wake-up status for entering SPD/DPD mode again */
    u32PMUSTS = CLK->PMUSTS;
    if(u32PMUSTS)
    {
        /* Release I/O hold status for SPD mode */
        CLK->IOPDCTL = 1;

        /* Clear SPD/DPD mode wake-up status */
        CLK->PMUSTS = CLK_PMUSTS_CLRWK_Msk;
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while(CLK->PMUSTS)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for SPD/DPD mode wake-up status is cleared time-out!\n");
                goto lexit;
            }
        }
    }

    /* Check SPD/DPD mode PB.2 falling-edge wake-up event */
    if(u32PMUSTS & (CLK_PMUSTS_PINWK2_Msk | CLK_PMUSTS_GPBWK_Msk))
    {
        printf("System waken-up done.\n\n");
        while(1);
    }

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------------------------------+\n");
    printf("|  SYS_PowerDown_MinCurrent and Wake-up by PB.2 Sample Code         |\n");
    printf("+-------------------------------------------------------------------+\n\n");

    printf("+-------------------------------------------------------------------+\n");
    printf("| Operating sequence                                                |\n");
    printf("|  1. Remove all continuous load, e.g. LED.                         |\n");
    printf("|  2. Configure all GPIO as Quasi-bidirectional Mode                |\n");
    printf("|  3. Disable analog function, e.g. POR module                      |\n");
    printf("|  4. Disable unused clock, e.g. LIRC                               |\n");
    printf("|  5. Disable SRAM retention for SPD mode                           |\n");
    printf("|  6. Enter to Power-Down                                           |\n");
    printf("|  7. Wait for PB.2 falling-edge interrupt event to wake-up the MCU |\n");
    printf("+-------------------------------------------------------------------+\n\n");

    /*
        To measure Power-down current:

        On NuMaker-M467HJ V1.0 board, remove components, e.g.
        Nu-Link2-Me,
        R79, R80, R81 for LED,
        U7 Ethernet PHY,
        U9 Audio codec,
        R73 and R75 for AUDIO_JKEN#.

        Remove R16 and then user can measure target chip power consumption by AMMETER connector.
    */

    /* Set function pin to GPIO mode except UART pin to print message */
    SYS->GPA_MFP0 = 0;
    SYS->GPA_MFP1 = 0;
    SYS->GPA_MFP2 = 0;
    SYS->GPA_MFP3 = 0;
    SYS->GPB_MFP0 = 0;
    SYS->GPB_MFP1 = 0;
    SYS->GPB_MFP2 = 0;
    SYS->GPB_MFP3 = UART0_TXD_PB13;
    SYS->GPC_MFP0 = 0;
    SYS->GPC_MFP1 = 0;
    SYS->GPC_MFP2 = 0;
    SYS->GPC_MFP3 = 0;
    SYS->GPD_MFP0 = 0;
    SYS->GPD_MFP1 = 0;
    SYS->GPD_MFP2 = 0;
    SYS->GPD_MFP3 = 0;
    SYS->GPE_MFP0 = 0;
    SYS->GPE_MFP1 = 0;
    SYS->GPE_MFP2 = 0;
    SYS->GPE_MFP3 = 0;
    SYS->GPF_MFP0 = 0;
    SYS->GPF_MFP1 = 0;
    SYS->GPF_MFP2 = 0;
    SYS->GPF_MFP3 = 0;
    SYS->GPG_MFP0 = 0;
    SYS->GPG_MFP1 = 0;
    SYS->GPG_MFP2 = 0;
    SYS->GPG_MFP3 = 0;
    SYS->GPH_MFP0 = 0;
    SYS->GPH_MFP1 = 0;
    SYS->GPH_MFP2 = 0;
    SYS->GPH_MFP3 = 0;
    SYS->GPI_MFP0 = 0;
    SYS->GPI_MFP1 = 0;
    SYS->GPI_MFP2 = 0;
    SYS->GPI_MFP3 = 0;
    SYS->GPJ_MFP0 = 0;
    SYS->GPJ_MFP1 = 0;
    SYS->GPJ_MFP2 = 0;
    SYS->GPJ_MFP3 = 0;

    /*
        Configure all GPIO as Quasi-bidirectional Mode. They are default output high.

        On NuMaker-M467HJ V1.0 board, configure GPIO as input mode pull-down if they have pull-down resistor outside:
        PA.6(RMII_RXERR),
        PB.15(FSUSB_VBUS_EN),
        PC.6(RMII_RXD1),
        PJ.13(HSUSB_VBUS_EN).
    */

    GPIO_SetMode(PA, 0xFFBF, GPIO_MODE_QUASI);
    GPIO_SetMode(PA, BIT6, GPIO_MODE_INPUT);
    GPIO_SetPullCtl(PA, BIT6, GPIO_PUSEL_PULL_DOWN);

    GPIO_SetMode(PB, 0x7FFF, GPIO_MODE_QUASI);
    GPIO_SetMode(PB, BIT15, GPIO_MODE_INPUT);
    GPIO_SetPullCtl(PB, BIT15, GPIO_PUSEL_PULL_DOWN);

    GPIO_SetMode(PC, 0xFFBF, GPIO_MODE_QUASI);
    GPIO_SetMode(PC, BIT6, GPIO_MODE_INPUT);
    GPIO_SetPullCtl(PC, BIT6, GPIO_PUSEL_PULL_DOWN);

    GPIO_SetMode(PD, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PE, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PF, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PG, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PH, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PI, GPIO_P0_TO_P15, GPIO_MODE_QUASI);

    GPIO_SetMode(PJ, 0xDFFF, GPIO_MODE_QUASI);
    GPIO_SetMode(PJ, BIT13, GPIO_MODE_INPUT);
    GPIO_SetPullCtl(PJ, BIT13, GPIO_PUSEL_PULL_DOWN);

    /* Unlock protected registers for Power-down and wake-up setting */
    SYS_UnlockReg();

    /* LVR setting */
    if(LvrSetting() < 0) goto lexit;

    /* POR setting */
    PorSetting();

    /* LIRC setting */
    if(LircSetting() < 0) goto lexit;

    /* LXT setting */
    if(LxtSetting() < 0) goto lexit;

    /* Select SPD mode SRAM retention size */
    CLK->PMUCTL = (CLK->PMUCTL & (~CLK_PMUCTL_SRETSEL_Msk)) | (SET_SRETSEL << CLK_PMUCTL_SRETSEL_Pos);

    /* Wake-up source configuration */
    if((SET_PDMSEL == CLK_PMUCTL_PDMSEL_PD) ||
            (SET_PDMSEL == CLK_PMUCTL_PDMSEL_LLPD) ||
            (SET_PDMSEL == CLK_PMUCTL_PDMSEL_FWPD))
    {
        /* Configure PB.2 as Quasi mode and enable interrupt by falling edge trigger */
        GPIO_SetMode(PB, BIT2, GPIO_MODE_QUASI);
        GPIO_EnableInt(PB, 2, GPIO_INT_FALLING);
        NVIC_EnableIRQ(GPB_IRQn);
    }
    else if(SET_PDMSEL == CLK_PMUCTL_PDMSEL_SPD)
    {
        /* Enable wake-up pin PB.2 falling edge wake-up at SPD mode */
        CLK_EnableSPDWKPin(1, 2, CLK_SPDWKPIN_FALLING, CLK_SPDWKPIN_DEBOUNCEDIS);

        /* Set PF.4~PF.11 I/O state by RTC control if RTC is enabled to prevent floating. */
        if(CLK->APBCLK0 & CLK_APBCLK0_RTCCKEN_Msk) GpioPinSettingRTC();
    }
    else if(SET_PDMSEL == CLK_PMUCTL_PDMSEL_DPD)
    {
        /* Enable wake-up pin 2 (PB.2) falling edge wake-up at DPD mode. PB.2 would be input mode floating at DPD mode. */
        CLK_EnableDPDWKPin(CLK_DPDWKPIN2_FALLING);

        /* Set PF.4~PF.11 I/O state by RTC control if RTC is enabled to prevent floating. */
        if(CLK->APBCLK0 & CLK_APBCLK0_RTCCKEN_Msk) GpioPinSettingRTC();
    }
    else
    {
        printf("Unknown Power-down mode!\n");
        goto lexit;
    }

    /* Enter to Power-down mode */
    if(SET_PDMSEL == CLK_PMUCTL_PDMSEL_PD)        printf("Enter to PD Power-Down ......\n");
    else if(SET_PDMSEL == CLK_PMUCTL_PDMSEL_LLPD) printf("Enter to LLPD Power-Down ......\n");
    else if(SET_PDMSEL == CLK_PMUCTL_PDMSEL_FWPD) printf("Enter to FWPD Power-Down ......\n");
    else if(SET_PDMSEL == CLK_PMUCTL_PDMSEL_SPD)  printf("Enter to SPD Power-Down ......\n");
    else if(SET_PDMSEL == CLK_PMUCTL_PDMSEL_DPD)  printf("Enter to DPD Power-Down ......\n");

    PowerDownFunction();

    /* Waiting for PB.2 falling-edge interrupt event */
    printf("System waken-up done.\n\n");

lexit:

    while(1);
}
