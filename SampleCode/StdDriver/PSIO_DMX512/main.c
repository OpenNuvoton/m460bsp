/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to implement DMX512 protocol by PSIO.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "DMX512_driver.h"



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

    /* Select PSIO module clock source as HIRC and PSIO module clock divider as 8 */
    CLK_SetModuleClock(PSIO_MODULE, CLK_CLKSEL2_PSIOSEL_HIRC, CLK_CLKDIV1_PSIO(8));

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
    S_PSIO_DMX512_CFG sConfig;
    uint16_t au16RxBuf[5];
    uint32_t i, u32TimeOutCnt;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+--------------------------------------------------------+ \n");
    printf("|   DMX512 Protocol Test Code                            | \n");
    printf("|   Please connected PSIO_CH0(PB.15) to PSIO_CH1(PC.4)   | \n");
    printf("+--------------------------------------------------------+ \n");

    /* Reset PSIO */
    SYS->IPRST2 |= SYS_IPRST2_PSIORST_Msk;
    SYS->IPRST2 &= ~SYS_IPRST2_PSIORST_Msk;

    /* Use slot controller 0 and pin 0  for TX */
    /* Use slot controller 1 and pin 1  for RX */
    sConfig.u8TxSlotCounter      = PSIO_SC0;
    sConfig.u8RxSlotCounter      = PSIO_SC1;
    sConfig.u8TxPin              = PSIO_PIN0;
    sConfig.u8RxPin              = PSIO_PIN1;

    /* Initialize PSIO setting for DMX512 */
    PSIO_DMX512_Open(&sConfig);
    NVIC_EnableIRQ(PSIO_IRQn);

    while(1)
    {

        for(i = 1 ; i < 6 ; i++)
        {
            PSIO_DMX512_getChannelData(&sConfig, i, &au16RxBuf[0]);
            printf("press any key to continue\n");
            getchar();

            PSIO_DMX512_Tx(&sConfig, 0, eDMX512_BREAK_START);
            CLK_SysTickDelay(50);
            PSIO_DMX512_Tx(&sConfig, 0x59, eDMX512_DATA);     /* Channel 1 data */
            CLK_SysTickDelay(50);
            PSIO_DMX512_Tx(&sConfig, 0x67, eDMX512_DATA);     /* Channel 2 data */
            CLK_SysTickDelay(50);
            PSIO_DMX512_Tx(&sConfig, 0x79, eDMX512_DATA);     /* Channel 3 data */
            CLK_SysTickDelay(50);
            PSIO_DMX512_Tx(&sConfig, 0x88, eDMX512_DATA);     /* Channel 4 data */
            CLK_SysTickDelay(50);
            PSIO_DMX512_Tx(&sConfig, 0x55, eDMX512_DATA);     /* Channel 5 data */
            CLK_SysTickDelay(50);

            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
            while(!(*sConfig.pu8RcvDone))
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for data time-out!\n");
                    goto lexit;
                }
            }

            printf("%u: 0x%02X\n", i, (uint8_t)DMX512_GET_DATA(au16RxBuf[0]));
        }
    }

lexit:

    while(1);
}
