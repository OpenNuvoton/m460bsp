/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to implement 1-Wire protocol by PSIO.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "DS18B20_driver_thermometer.h"



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

    /* Enable PSIO module clock */
    CLK_EnableModuleClock(PSIO_MODULE);

    /* Set PCLK1 to HCLK/8 */
    CLK->PCLKDIV = (CLK->PCLKDIV & ~CLK_PCLKDIV_APB1DIV_Msk) | CLK_PCLKDIV_APB1DIV_DIV8;

    /* Select PSIO module clock source as PCLK1 and PSIO module clock divider as 250 */
    CLK_SetModuleClock(PSIO_MODULE, CLK_CLKSEL2_PSIOSEL_PCLK1, CLK_CLKDIV1_PSIO(250));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set PSIO multi-function pin CH0(PB.15) */
    SET_PSIO0_CH0_PB15();

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
    S_PSIO_DS18B20_CFG sConfig;
    uint8_t au8Data[2] = {0};

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
    printf("|      DS18B20 ONE WIRE Thermometer  Test Code         | \n");
    printf("|      Please connected PSIO_CH0(PB.15) to device      | \n");
    printf("+------------------------------------------------------+ \n");

    /* Use slot controller 0 and pin 0 */
    sConfig.u8SlotCtrl   = PSIO_SC0;
    sConfig.u8DataPin    = PSIO_PIN0;

    /* Reset PSIO */
    SYS->IPRST2 |= SYS_IPRST2_PSIORST_Msk;
    SYS->IPRST2 &= ~SYS_IPRST2_PSIORST_Msk;

    /* Initialize PSIO setting for DS18B20 */
    PSIO_DS18B20_Open(&sConfig);

    do
    {
        /* Reset DS18B20 */
        PSIO_DS18B20_Reset(&sConfig);

        /* Write command to DS18B20 */
        PSIO_DS18B20_Write_Command(&sConfig, ONEWIRE_SKIP_ROM);
        PSIO_DS18B20_Write_Command(&sConfig, ONEWIRE_CONVT);
        /* Reset DS18B20 */
        PSIO_DS18B20_Reset(&sConfig);

        /* Write command to DS18B20 */
        PSIO_DS18B20_Write_Command(&sConfig, ONEWIRE_SKIP_ROM);
        PSIO_DS18B20_Write_Command(&sConfig, ONEWIRE_RDSCRATCH_PAD);

        /* Read data from DS18B20 */
        PSIO_DS18B20_Read_Data(&sConfig, &au8Data[0]);

        printf("Temperature= %f degrees C\n", (au8Data[0] | ((au8Data[1] & 0x07) << 8)) * 0.0625);

        /* Initialize data */
        au8Data[0] = 0;
        au8Data[1] = 0;

        /* Delay 50000 us */
        CLK_SysTickDelay(50000);

    }
    while(1);
}
