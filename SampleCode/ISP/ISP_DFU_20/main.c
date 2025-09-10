/***************************************************************************//**
 * @file     main.c
 * @brief    Demonstrate how to upgrade firmware between USB device and PC through USB DFU (Device Firmware Upgrade) class.
 *           A windows tool is also included in this sample code to connect with USB device.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "fmc_user.h"
#include "dfu_transfer.h"

#define PLL_CLOCK           200000000

int32_t g_FMC_i32ErrCode = 0;

void ProcessHardFault(void);
void SH_Return(void);
void SendChar_ToUART(void);
int32_t SYS_Init(void);

void ProcessHardFault(void) {}
void SH_Return(void) {}
void SendChar_ToUART(void) {}

/* Add implementations to fix linker warnings from the newlib-nano C library in VSCode-GCC14.3.1 */
void _close(void) {}
void _lseek(void) {}
void _read_r(void) {}
void _write_r(void) {}

uint32_t CLK_GetPLLClockFreq(void)
{
    return PLL_CLOCK;
}

int32_t SYS_Init(void)
{
    uint32_t volatile i;
    uint32_t u32TimeOutCnt;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC and HXT clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk;

    /* Wait for HIRC and HXT clock ready */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!(CLK->STATUS & (CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_HXTSTB_Msk)))
    {
        if(--u32TimeOutCnt == 0)
            return -1;
    }

    /* Set HCLK clock source as HIRC first */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;

    /* Disable PLL clock before setting PLL frequency */
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Set PLL clock as 200MHz from HIRC */
    CLK->PLLCTL = CLK_PLLCTL_200MHz_HIRC;

    /* Wait for PLL clock ready */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk))
    {
        if(--u32TimeOutCnt == 0)
            return -1;
    }

    /* Set power level by HCLK working frequency */
    SYS->PLCTL = (SYS->PLCTL & (~SYS_PLCTL_PLSEL_Msk)) | SYS_PLCTL_PLSEL_PL0;

    /* Set flash access cycle by HCLK working frequency */
    FMC->CYCCTL = (FMC->CYCCTL & (~FMC_CYCCTL_CYCLE_Msk)) | (8);

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Select HCLK clock source as PLL and HCLK source divider as 1 */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_PLL;

    /* Update System Core Clock */
    PllClock        = 200000000;
    SystemCoreClock = 200000000;
    CyclesPerUs     = SystemCoreClock / 1000000;  /* For CLK_SysTickDelay() */

    /* Select HSUSBD */
    SYS->USBPHY &= ~SYS_USBPHY_HSUSBROLE_Msk;

    /* Enable USB PHY */
    SYS->USBPHY = (SYS->USBPHY & ~(SYS_USBPHY_HSUSBROLE_Msk | SYS_USBPHY_HSUSBACT_Msk)) | SYS_USBPHY_HSUSBEN_Msk;
    for(i = 0; i < 0x1000; i++);   // delay > 10 us
    SYS->USBPHY |= SYS_USBPHY_HSUSBACT_Msk;

    /* Enable HSUSBD module clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_HSUSBDCKEN_Msk;

    /* Enable GPIO B clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPBCKEN_Msk;

    return 0;
}

void USBD20_IRQHandler(void);
/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock write-protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    if(SYS_Init() < 0)
        goto _APROM;

    /* Enable ISP */
    CLK->AHBCLK0 |= CLK_AHBCLK0_ISPCKEN_Msk;
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_APUEN_Msk | FMC_ISPCTL_ISPFF_Msk;

    /* Get APROM and Data Flash size */
    g_u32ApromSize = GetApromSize();
    GetDataFlashInfo(&g_u32DataFlashAddr, &g_u32DataFlashSize);

    if(DetectPin != 0)
    {
        goto _APROM;
    }

    /* Open HSUSBD controller */
    HSUSBD_Open(NULL, DFU_ClassRequest, NULL);

    /*Init Endpoint configuration for DFU */
    DFU_Init();

    /* Start transaction */
    HSUSBD->OPER = HSUSBD_OPER_HISPDEN_Msk;   /* high-speed */
    HSUSBD_CLR_SE0();

    /* Polling HSUSBD interrupt flag */
    while(DetectPin == 0)
    {
        USBD20_IRQHandler();
    }

_APROM:
    /* Reset system and boot from APROM */
    FMC_SetVectorPageAddr(FMC_APROM_BASE);
    NVIC_SystemReset();

    /* Trap the CPU */
    while(1);
}
