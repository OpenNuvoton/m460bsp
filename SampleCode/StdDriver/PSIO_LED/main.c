/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to light up the WS1812B LED array.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "WS1812B_driver_LED.h"


#define LED_NUMBER  8
#define PIN_NUMBER  2


typedef enum
{
    eCASE_GREEN_BLUE = 0,
    eCASE_RED_GREEN,
    eCASE_BLUE_RED,
    eCASE_WHITE,
} E_LED_COLOR;


void SYS_Init(void);
void UART0_Init(void);


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

    /* Enable PDMA0 module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Enable PSIO module clock */
    CLK_EnableModuleClock(PSIO_MODULE);

    /* Select PSIO module clock source as HIRC and PSIO module clock divider as 1 */
    CLK_SetModuleClock(PSIO_MODULE, CLK_CLKSEL2_PSIOSEL_HIRC, CLK_CLKDIV1_PSIO(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set PSIO multi-function pin CH0(PB.15) and CH1(PC.4) */
    SET_PSIO0_CH0_PB15();
    SET_PSIO0_CH1_PC4();

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

    /* For library internal used, this memory size should be (data length)*3 word at least, */
    /* this case is  LED_NUMBER*PIN_NUMBER*3 word.                                          */
    uint32_t   au32LedPattern[LED_NUMBER * PIN_NUMBER], au32InternalUse[LED_NUMBER * PIN_NUMBER * 3];

    S_PSIO_WS2812B_LED_CFG  sConfig;
    E_LED_COLOR             eColor = eCASE_GREEN_BLUE;
    WS2812B_LED_Pin_CFG     sPinCFG = {PSIO_PIN0, PSIO_PIN1, 0, 0, 0, 0, 0, 0}; //Enable Pin0 and Pin1

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+------------------------------------------------------+ \n");
    printf("|         Worldsemi WS2812B LED sample code            | \n");
    printf("|          Please connected PSIO_CH0(PB.15)            | \n");
    printf("|           and PSIO_CH1(PC.4) to device               | \n");
    printf("+------------------------------------------------------+ \n");

    /* Set Led configuration */
    sConfig.u8SlotCtrl      = PSIO_SC0;
    sConfig.u8PDMAChannel   = 0;
    sConfig.pu8PinCFG       = sPinCFG;
    sConfig.u8PinNumber     = 2;
    sConfig.pu32DataAddr    = au32LedPattern;
    sConfig.u32DataLength   = LED_NUMBER * PIN_NUMBER;
    sConfig.pu32InternalMemory = au32InternalUse;           /* For library internal used, the memory size should be (data length)*3 word at least. */
    /* This case is  LED_NUMBER*PIN_NUMBER*3 word. */

    /* Initialize PSIO setting for WS2812B LED */
    PSIO_WS2812B_Open(&sConfig);

    do
    {
        switch(eColor)
        {
            case eCASE_GREEN_BLUE:
                /* PIN0 */                              /* PIN1 */
                au32LedPattern[0]    = WS2812B_GREEN;
                au32LedPattern[1]    = WS2812B_BLUE;    //LED0
                au32LedPattern[2]    = WS2812B_GREEN;
                au32LedPattern[3]    = WS2812B_BLUE;    //LED1
                au32LedPattern[4]    = WS2812B_GREEN;
                au32LedPattern[5]    = WS2812B_BLUE;    //LED2
                au32LedPattern[6]    = WS2812B_GREEN;
                au32LedPattern[7]    = WS2812B_BLUE;    //LED3
                au32LedPattern[8]    = WS2812B_GREEN;
                au32LedPattern[9]    = WS2812B_BLUE;    //LED4
                au32LedPattern[10]   = WS2812B_GREEN;
                au32LedPattern[11]   = WS2812B_BLUE;    //LED5
                au32LedPattern[12]   = WS2812B_GREEN;
                au32LedPattern[13]   = WS2812B_BLUE;    //LED6
                au32LedPattern[14]   = WS2812B_GREEN;
                au32LedPattern[15]   = WS2812B_BLUE;    //LED7
                eColor = eCASE_RED_GREEN;
                break;

            case eCASE_RED_GREEN:
                /* PIN0 */                              /* PIN1 */
                au32LedPattern[0]    = WS2812B_RED;
                au32LedPattern[1]    = WS2812B_GREEN;   //LED0
                au32LedPattern[2]    = WS2812B_RED;
                au32LedPattern[3]    = WS2812B_GREEN;   //LED1
                au32LedPattern[4]    = WS2812B_RED;
                au32LedPattern[5]    = WS2812B_GREEN;   //LED2
                au32LedPattern[6]    = WS2812B_RED;
                au32LedPattern[7]    = WS2812B_GREEN;   //LED3
                au32LedPattern[8]    = WS2812B_RED;
                au32LedPattern[9]    = WS2812B_GREEN;   //LED4
                au32LedPattern[10]   = WS2812B_RED;
                au32LedPattern[11]   = WS2812B_GREEN;   //LED5
                au32LedPattern[12]   = WS2812B_RED;
                au32LedPattern[13]   = WS2812B_GREEN;   //LED6
                au32LedPattern[14]   = WS2812B_RED;
                au32LedPattern[15]   = WS2812B_GREEN;   //LED7
                eColor = eCASE_BLUE_RED;
                break;

            case eCASE_BLUE_RED:
                /* PIN0 */                              /* PIN1 */
                au32LedPattern[0]    = WS2812B_BLUE;
                au32LedPattern[1]    = WS2812B_RED;     //LED0
                au32LedPattern[2]    = WS2812B_BLUE;
                au32LedPattern[3]    = WS2812B_RED;     //LED1
                au32LedPattern[4]    = WS2812B_BLUE;
                au32LedPattern[5]    = WS2812B_RED;     //LED2
                au32LedPattern[6]    = WS2812B_BLUE;
                au32LedPattern[7]    = WS2812B_RED;     //LED3
                au32LedPattern[8]    = WS2812B_BLUE;
                au32LedPattern[9]    = WS2812B_RED;     //LED4
                au32LedPattern[10]   = WS2812B_BLUE;
                au32LedPattern[11]   = WS2812B_RED;     //LED5
                au32LedPattern[12]   = WS2812B_BLUE;
                au32LedPattern[13]   = WS2812B_RED;     //LED6
                au32LedPattern[14]   = WS2812B_BLUE;
                au32LedPattern[15]   = WS2812B_RED;     //LED7
                eColor = eCASE_WHITE;
                break;

            case eCASE_WHITE:
                /* PIN0 */                              /* PIN1 */
                au32LedPattern[0]    = WS2812B_WHITE;
                au32LedPattern[1]    = WS2812B_WHITE;   //LED0
                au32LedPattern[2]    = WS2812B_WHITE;
                au32LedPattern[3]    = WS2812B_WHITE;   //LED1
                au32LedPattern[4]    = WS2812B_WHITE;
                au32LedPattern[5]    = WS2812B_WHITE;   //LED2
                au32LedPattern[6]    = WS2812B_WHITE;
                au32LedPattern[7]    = WS2812B_WHITE;   //LED3
                au32LedPattern[8]    = WS2812B_WHITE;
                au32LedPattern[9]    = WS2812B_WHITE;   //LED4
                au32LedPattern[10]   = WS2812B_WHITE;
                au32LedPattern[11]   = WS2812B_WHITE;   //LED5
                au32LedPattern[12]   = WS2812B_WHITE;
                au32LedPattern[13]   = WS2812B_WHITE;   //LED6
                au32LedPattern[14]   = WS2812B_WHITE;
                au32LedPattern[15]   = WS2812B_WHITE;   //LED7
                eColor = eCASE_GREEN_BLUE;
                break;
        }

        /* Send LED pattern by PSIO */
        PSIO_WS2812B_Send_Pattern(&sConfig);

        /* Delay 500 ms */
        CLK_SysTickLongDelay(500000);
    }
    while(1);

}
