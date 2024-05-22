/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to wake up system from Power-down mode by UART interrupt.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define RS485_ADDRESS   0xC0

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void UART_DataWakeUp(void);
void UART_CTSWakeUp(void);
void UART_RxThresholdWakeUp(void);
void UART_RS485WakeUp(void);
void UART_PowerDown_TestItem(void);
void UART_PowerDownWakeUpTest(void);
void PowerDownFunction(void);
void SYS_Init(void);
void UART0_Init(void);
void UART1_Init(void);
void UART1_IRQHandler(void);
/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(DEBUG_PORT)
    if(--u32TimeOutCnt == 0) break;

    /* Enter to Power-down mode */
    CLK_PowerDown();
}

void SYS_Init(void)
{

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

    /* Enable UART module clock */
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_EnableModuleClock(UART1_MODULE);

    /* Select UART module clock source and UART module clock divider */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set PA multi-function pins for UART1 RXD(PA.2), TXD(PA.3), nRTS(PA.0) and nCTS(PA.1) */
    SET_UART1_RXD_PA2();
    SET_UART1_TXD_PA3();
    SET_UART1_nRTS_PA0();
    SET_UART1_nCTS_PA1();

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

void UART1_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Reset UART1 */
    SYS_ResetModule(UART1_RST);

    /* Configure UART1 and set UART1 Baudrate */
    UART_Open(UART1, 9600);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init UART1 for test */
    UART1_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("UART Sample Program\n\n");

    /* UART Power-down and Wake-up sample function */
    UART_PowerDownWakeUpTest();

    printf("\nUART Sample Program End\n");

    while(1);

}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 1 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_IRQHandler(void)
{
    uint32_t u32Data, u32TimeOutCnt;

    if(UART_GET_INT_FLAG(UART1, UART_INTSTS_WKINT_Msk))     /* UART wake-up interrupt flag */
    {
        UART_ClearIntFlag(UART1, UART_INTSTS_WKINT_Msk);
        printf("UART wake-up.\n");
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        UART_WAIT_TX_EMPTY(DEBUG_PORT)
        if(--u32TimeOutCnt == 0) break;
    }
    else if(UART_GET_INT_FLAG(UART1, UART_INTSTS_RDAINT_Msk | UART_INTSTS_RXTOINT_Msk))  /* UART receive data available flag */
    {
        while(UART_GET_RX_EMPTY(UART1) == 0)
        {
            u32Data = UART_READ(UART1);
            if(u32Data & UART_DAT_PARITY_Msk)
                printf("Address: 0x%X\n", (u32Data & 0xFF));
            else
                printf("Data: 0x%X\n", u32Data);
        }
    }

}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART nCTS Wake-up Function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void UART_CTSWakeUp(void)
{
    /* Enable UART nCTS wake-up function */
    UART1->WKCTL |= UART_WKCTL_WKCTSEN_Msk;

    printf("System enter to Power-down mode.\n");
    printf("Toggle UART1 nCTS to wake-up system.\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Data Wake-up Function                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void UART_DataWakeUp(void)
{
    /* Enable UART data wake-up function */
    UART1->WKCTL |= UART_WKCTL_WKDATEN_Msk;

    /* Set UART data wake-up start bit compensation value.
       It indicates how many clock cycle selected by UART_CLK does the UART controller can get the first bit (start bit)
       when the device is wake-up from power-down mode.
       If UART_CLK is selected as HIRC(12MHz) and the HIRC stable time is about 64 clocks,
       the data wake-up start bit compensation value can be set as 64. */
    UART1->DWKCOMP = 64;

    printf("System enter to Power-down mode.\n");
    printf("Send data with baud rate 9600bps to UART1 to wake-up system.\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Rx threshold and time-out Function                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void UART_RxThresholdWakeUp(void)
{
    /* Select UART clock source as LXT */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_LXT, CLK_CLKDIV0_UART1(1));

    /* Set UART baud rate and baud rate compensation */
    UART_Open(UART1, 9600);
    UART1->BRCOMP = 0xA5;

    /* Enable UART Rx Threshold and Rx time-out wake-up function */
    UART1->WKCTL |= UART_WKCTL_WKRFRTEN_Msk | UART_WKCTL_WKTOUTEN_Msk;

    /* Set Rx FIFO interrupt trigger level */
    UART1->FIFO = (UART1->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_4BYTES;

    /* Enable UART Rx time-out function */
    UART_SetTimeoutCnt(UART1, 40);

    printf("System enter to Power-down mode.\n");
    printf("Send data with baud rate 9600bps to UART1 to wake-up system.\n\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART RS485 address match (AAD mode) function                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void UART_RS485WakeUp(void)
{
    /* Select UART clock source as LXT */
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_LXT, CLK_CLKDIV0_UART1(1));

    /* Set UART baud rate and baud rate compensation */
    UART_Open(UART1, 9600);
    UART1->BRCOMP = 0xA5;

    /* RS485 address match (AAD mode) setting */
    UART_SelectRS485Mode(UART1, UART_ALTCTL_RS485AAD_Msk, RS485_ADDRESS);

    /* Enable parity source selection function */
    UART1->LINE |= (UART_LINE_PSS_Msk | UART_LINE_PBE_Msk);

    /* Enable UART RS485 address match, Rx Threshold and Rx time-out wake-up function */
    UART1->WKCTL |= UART_WKCTL_WKRFRTEN_Msk | UART_WKCTL_WKRS485EN_Msk | UART_WKCTL_WKTOUTEN_Msk;

    /* Enable UART Rx time-out function */
    UART_SetTimeoutCnt(UART1, 40);

    printf("System enter to Power-down mode.\n");
    printf("Send RS485 address byte 0x%X with baud rate 9600bps to UART1 to wake-up system.\n\n", RS485_ADDRESS);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Power-down and Wake-up Menu                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void UART_PowerDown_TestItem(void)
{
    printf("+-----------------------------------------------------------+\n");
    printf("|  UART Power-down and wake-up test                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| [1] nCTS wake-up test                                     |\n");
    printf("| [2] Data wake-up test                                     |\n");
    printf("| [3] Rx threshold and time-out wake-up test                |\n");
    printf("| [4] RS485 wake-up test                                    |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| Quit                                           - [Others] |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("Please Select key (1~4): ");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  UART Power-down and Wake-up Test Function                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void UART_PowerDownWakeUpTest(void)
{
    uint32_t u32Item;

    printf("Due to PLL clock stable too slow.\n");
    printf("Before demo UART wake-up, this demo code will switch HCLK from PLL to HIRC.\n");

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Lock protected registers */
    SYS_LockReg();

    printf("\nCPU @ %dHz\n", SystemCoreClock);

    /* Enable UART wake-up and receive data available interrupt */
    NVIC_EnableIRQ(UART1_IRQn);
    UART_EnableInt(UART1, UART_INTEN_WKIEN_Msk | UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);

    UART_PowerDown_TestItem();
    u32Item = (uint32_t)getchar();
    printf("%c\n\n", u32Item);
    switch(u32Item)
    {
        case '1':
            UART_CTSWakeUp();
            break;
        case '2':
            UART_DataWakeUp();
            break;
        case '3':
            UART_RxThresholdWakeUp();
            break;
        case '4':
            UART_RS485WakeUp();
            break;
        default:
            return;
    }

    /* Unlock protected registers before entering Power-down mode */
    SYS_UnlockReg();

    /* Enter to Power-down mode */
    PowerDownFunction();

    /* Wait for all data transmission is finished */
    while((UART1->FIFOSTS & UART_FIFOSTS_RXIDLE_Msk) == 0);

    /* Lock protected registers after entering Power-down mode */
    SYS_LockReg();

    printf("Enter any key to end test.\n");
    getchar();

    /* Disable UART wake-up function */
    UART1->WKCTL = 0;

    /* Disable UART Interrupt */
    NVIC_DisableIRQ(UART1_IRQn);
    UART_DisableInt(UART1, UART_INTEN_WKIEN_Msk | UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk);

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Set core clock to 200MHz */
    CLK_SetCoreClock(200000000);

    /* Lock protected registers */
    SYS_LockReg();

}
