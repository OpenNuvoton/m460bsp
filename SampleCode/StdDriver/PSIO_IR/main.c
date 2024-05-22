/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to implement NEC IR protocol by PSIO.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "NEC_IR_driver.h"



void SYS_Init(void);
void UART0_Init(void);



void SYS_Init(void)
{

    /* Set PF multi-function pins for X32_OUT(PF.4) and X32_IN(PF.5) */
    SET_X32_OUT_PF4();
    SET_X32_IN_PF5();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable LXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Wait for LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

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

    /* Select PSIO module clock source as LXT and PSIO module clock divider as 2 */
    CLK_SetModuleClock(PSIO_MODULE, CLK_CLKSEL2_PSIOSEL_LXT, CLK_CLKDIV1_PSIO(2));

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
    S_PSIO_NEC_CFG sConfig;
    uint32_t u32TimeOutCnt;

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
    printf("|               NEC IR sample code                     | \n");
    printf("|      Please check waveform on PSIO_CH0(PB.15)        | \n");
    printf("+------------------------------------------------------+ \n");

    /* Use slot controller 0 and pin 0 */
    sConfig.u8SlotCtrl   = PSIO_SC0;
    sConfig.u8TxPin      = PSIO_PIN0;

    /* Initialize PSIO setting for NEC IR protocol */
    PSIO_NEC_Open(&sConfig);

    /* Send  0x1(Address), ~0x1(/Address), 0x2(Command), ~0x2(/Command) */
    PSIO_NEC_Send(&sConfig, 0x1, ~0x1, 0x2, ~0x2);

    /* Wait transfer done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(PSIO_NEC_TransferDone(&sConfig))
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for PSIO transfer done time-out!\n");
            goto lexit;
        }
    }

    /* Send  Repeat signal */
    PSIO_NEC_Repeat(&sConfig);

    /* Wait transfer done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(PSIO_NEC_TransferDone(&sConfig))
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for PSIO transfer done time-out!\n");
            goto lexit;
        }
    }

    /* Release PSIO setting */
    PSIO_NEC_Close(&sConfig);

    printf("Complete!\n");

lexit:

    while(1);
}
