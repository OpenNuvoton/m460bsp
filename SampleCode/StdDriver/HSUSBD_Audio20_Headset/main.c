/***************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    This is an UAC2.0 sample and used to plays the sound send from PC
 *           through the USB interface
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "usbd_audio.h"

/*--------------------------------------------------------------------------*/
void SYS_Init(void)
{
    uint32_t volatile i;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC and HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HIRC and HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_HXTSTB_Msk);

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 192MHz */
    CLK_SetCoreClock(FREQ_192MHZ);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;
    CLK->AHBCLK1 |= CLK_AHBCLK1_GPICKEN_Msk | CLK_AHBCLK1_GPJCKEN_Msk;

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Select HSUSBD */
    SYS->USBPHY &= ~SYS_USBPHY_HSUSBROLE_Msk;

    /* Enable USB PHY */
    SYS->USBPHY = (SYS->USBPHY & ~(SYS_USBPHY_HSUSBROLE_Msk | SYS_USBPHY_HSUSBACT_Msk)) | SYS_USBPHY_HSUSBEN_Msk;
    for(i = 0; i < 0x1000; i++);   // delay > 10 us
    SYS->USBPHY |= SYS_USBPHY_HSUSBACT_Msk;

    /* Enable HSUSBD module clock */
    CLK_EnableModuleClock(HSUSBD_MODULE);

    /* Enable TIMER0 module clock */
    CLK_EnableModuleClock(TMR0_MODULE);

    /* Select TIMER0 module clock source as HIRC */
    CLK_SetModuleClock(TMR0_MODULE, CLK_CLKSEL1_TMR0SEL_HIRC, 0);

    /* Enable I2S0 module clock */
    CLK_EnableModuleClock(I2S0_MODULE);

    /* Enable I2C2 module clock */
    CLK_EnableModuleClock(I2C2_MODULE);

    /* Enable PDMA0 module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set multi-function pins for I2S0 */
    SET_I2S0_BCLK_PI6();
    SET_I2S0_MCLK_PI7();
    SET_I2S0_DI_PI8();
    SET_I2S0_DO_PI9();
    SET_I2S0_LRCK_PI10();

    /* Set I2C2 multi-function pins */
    SET_I2C2_SDA_PD0();
    SET_I2C2_SCL_PD1();

    PI->SMTEN |= GPIO_SMTEN_SMTEN6_Msk;
    PD->SMTEN |= GPIO_SMTEN_SMTEN1_Msk;

    /* Lock protected registers */
    SYS_LockReg();
}

/* Init I2C interface */
void I2C2_Init(void)
{
    /* Open I2C2 and set clock to 400k */
    I2C_Open(I2C2, 400000);

    /* Get I2C2 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C2));
}


int main(void)
{
    uint32_t volatile i;

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

#ifdef __HID__
    GPIO_Init();
#endif

    printf("\n");
    printf("+--------------------------------------------------------+\n");
    printf("|           NuMicro HSUSB UAC2.0 Sample Code             |\n");
    printf("+--------------------------------------------------------+\n");
    printf("HXT clock %d Hz\n", CLK_GetHXTFreq());
    printf("CPU clock %d Hz\n", CLK_GetCPUFreq());

    /* Init I2C2 to access codec */
    I2C2_Init();

    /* Select source from PLL/2 */
    CLK_SetModuleClock(I2S0_MODULE, CLK_CLKSEL3_I2S0SEL_PLL_DIV2, 0);

    /* Open I2S0 interface and set to slave mode, stereo channel, I2S format */
    I2S_Open(I2S0, I2S_MODE_SLAVE, 192000, I2S_DATABIT_16, I2S_STEREO, I2S_FORMAT_I2S);

    /* Set PD3 low to enable phone jack on NuMaker board. */
    SYS->GPD_MFP0 &= ~(SYS_GPD_MFP0_PD3MFP_Msk);
    GPIO_SetMode(PD, BIT3, GPIO_MODE_OUTPUT);
    PD3 = 0;

    /* Set MCLK and enable MCLK */
    I2S_EnableMCLK(I2S0, 12000000);

#if NAU8822
    NAU8822_Setup();
#else
    NAU88L25_Reset();
    /* Initialize NAU88L25 codec */
    CLK_SysTickDelay(10000);
    I2S0->CTL0 |= I2S_CTL0_ORDER_Msk;
    /* Initialize NAU88L25 codec */
    CLK_SysTickDelay(20000);
    NAU88L25_Setup();
#endif

    /* Configure PDMA */
    PDMA_Init();

    /* Configure TIMER0 for adjusting codec's PLL */
    TIMER_Open(TIMER0, TIMER_PERIODIC_MODE, 300);
    TIMER_EnableInt(TIMER0);
    NVIC_SetPriority(TMR0_IRQn, 3);
    NVIC_EnableIRQ(TMR0_IRQn);

    HSUSBD_Open(&gsHSInfo, UAC_ClassRequest, UAC_SetInterface);

    /* Endpoint configuration */
    UAC_Init();
    NVIC_EnableIRQ(USBD20_IRQn);
    HSUSBD_Start();

    while(1)
    {
#ifdef __HID__
        HID_UpdateHidData();
#endif
    }
}
