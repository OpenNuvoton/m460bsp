/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to implement PS/2 host protocol by PSIO.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "PS2_Host_driver.h"



void SYS_Init(void);
void UART0_Init(void);
void PSIO_IRQHandler(void);


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
S_PSIO_PS2 g_sConfig;
volatile uint32_t g_u32TxData = 0, g_u32RxData = 0;
volatile uint32_t g_u32RxACK = 1;


void PSIO_IRQHandler(void)
{
    static uint8_t u8BitNumber = 0;
    uint8_t u8INT0Flag;
    uint32_t u32TimeOutCnt;

    /* Get INT0 interrupt flag */
    u8INT0Flag = PSIO_GET_INT_FLAG(PSIO, PSIO_INTSTS_CON0IF_Msk);

    if(u8INT0Flag)
    {
        /* Clear INT0 interrupt flag */
        PSIO_CLEAR_INT_FLAG(PSIO, PSIO_INTSTS_CON0IF_Msk);
    }
    else
    {
        printf("Unknown interrupt occur!!!\n");
    }

    if((PSIO_PS2_GET_STATUS() == eHOST_READ) || (PSIO_PS2_GET_STATUS() == eHOST_READY_TO_READ))
    {
        static uint32_t u32RxBuffer = 0;

        /* Trigger slot controller */
        PSIO_START_SC(PSIO, g_sConfig.u8DataSC);

        /* Wait input buffer full */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while(!PSIO_GET_TRANSFER_STATUS(PSIO, PSIO_TRANSTS_INFULL0_Msk << (g_sConfig.u8DataPin * 4)))
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for PSIO time-out!\n");
                return;
            }
        }

        /* Receive 11 bit */
        u32RxBuffer |= (PSIO_GET_INPUT_DATA(PSIO, g_sConfig.u8DataPin) << u8BitNumber);

        if(u8BitNumber == 10)
        {
            PSIO_PS2_SET_STATUS(eHOST_IDLE);
            u8BitNumber = 0;
            g_u32RxData = u32RxBuffer;
            u32RxBuffer = 0;
        }
        else
        {
            PSIO_PS2_SET_STATUS(eHOST_READ);
            u8BitNumber++;
        }
    }
    else if(PSIO_PS2_GET_STATUS() == eHOST_WRITE)
    {
        if(u8BitNumber == 10)
        {
            /* Trigger slot controller */
            PSIO_START_SC(PSIO, g_sConfig.u8DataSC);

            /* Wait slot controller is not busy */
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
            while(PSIO_GET_BUSY_FLAG(PSIO, g_sConfig.u8DataSC))
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for PSIO time-out!\n");
                    return;
                }
            }

            /* Read data from buffer */
            g_u32RxACK = PSIO_GET_INPUT_DATA(PSIO, g_sConfig.u8DataPin);

            /* Set pin interval status as high */
            PSIO->GNCT[g_sConfig.u8DataPin].GENCTL = (PSIO->GNCT[g_sConfig.u8DataPin].GENCTL & ~PSIO_GNCT_GENCTL_INTERVAL_Msk)
                    | (PSIO_HIGH_LEVEL << PSIO_GNCT_GENCTL_INTERVAL_Pos);
            PSIO_PS2_SET_STATUS(eHOST_IDLE);
            u8BitNumber = 0;
        }
        else if(u8BitNumber == 9)
        {

            PSIO_SET_OUTPUT_DATA(PSIO, g_sConfig.u8DataPin, g_u32TxData >> u8BitNumber);

            /* Trigger slot controller */
            PSIO_START_SC(PSIO, g_sConfig.u8DataSC);

            /* Wait slot controller is not busy */
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
            while(PSIO_GET_BUSY_FLAG(PSIO, g_sConfig.u8DataSC))
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for PSIO time-out!\n");
                    return;
                }
            }

            /* Set check point action */
            /* For more efficient, accessing register directly */
            PSIO->GNCT[g_sConfig.u8DataPin].CPCTL1   = PSIO_IN_BUFFER;     //input buffer
            u8BitNumber++;
        }
        else
        {
            /* Send 9 bit */
            PSIO_SET_OUTPUT_DATA(PSIO, g_sConfig.u8DataPin, g_u32TxData >> u8BitNumber);

            /* Trigger slot controller */
            PSIO_START_SC(PSIO, g_sConfig.u8DataSC);

            u8BitNumber++;
        }
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
    uint32_t u32RxData = 0x0;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART for printf */
    UART0_Init();

    printf("******************************************************\n");
    printf("|               PS/2 Host Sample Code                |\n");
    printf("|      Please connected PSIO_CH0(PB.15)(Clock)       |\n");
    printf("|      and PSIO_CH1(PC.4)(Data).                     |\n");
    printf("******************************************************\n");

    /* Use slot controller 0/1 and pin 0/1 */
    g_sConfig.u8ClockSC        = PSIO_SC0;
    g_sConfig.u8DataSC         = PSIO_SC1;
    g_sConfig.u8ClockPin       = PSIO_PIN0;
    g_sConfig.u8DataPin        = PSIO_PIN1;
    g_sConfig.p32ClockMFP      = &PB15;
    g_sConfig.p32DataMFP       = &PC4;

    /* Initialize PSIO setting for PS/2 host protocol */
    PSIO_PS2_Open(&g_sConfig);

    while(1)
    {
        /* Set PSIO on read signal state */
        if(PSIO_PS2_GET_STATUS() != eHOST_READY_TO_READ)
        {
            PSIO_PS2_HostRead(&g_sConfig);
        }

        /* Receiving data */
        while(PSIO_PS2_GET_STATUS() == eHOST_READ);

        /* Data was received */
        if(PSIO_PS2_GET_STATUS() == eHOST_IDLE)
        {
            PSIO_PS2_SET_STATUS(eHOST_READY_TO_READ);
            u32RxData = g_u32RxData;
            printf("[Start]0x%x, [Data]0x%x, [Parity]0x%x, [Stop]0x%x\n",
                   u32RxData & 0x1, (u32RxData >> 1) & 0xFF, (u32RxData >> 9) & 0x1, (u32RxData >> 10) & 0x1);
        }

        /* Send data */
        if(!(UART0->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk))
        {
            uint32_t u32Data;

            u32Data = UART0->DAT;
            g_u32TxData = PSIO_Encode_TxData(&u32Data);
            printf("[Host send to device]0x%x, 0x%x\n", u32Data, g_u32TxData);
            if(PSIO_PS2_HostSend(&g_sConfig) < 0)
                goto lexit;

            while(PSIO_PS2_GET_STATUS() == eHOST_WRITE);
        }
    }

lexit:

    while(1);

}
