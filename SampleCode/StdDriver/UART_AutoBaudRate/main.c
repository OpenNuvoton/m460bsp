/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Show how to use auto baud rate detection function.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"



/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void AutoBaudRate_Test(void);
void AutoBaudRate_TxTest(void);
void AutoBaudRate_RxTest(void);
void SYS_Init(void);
void UART0_Init(void);
void UART1_Init(void);
uint32_t GetUartBaudrate(UART_T* uart);



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

    /* Set PA multi-function pins for UART1 RXD(PA.2) and TXD(PA.3) */
    SET_UART1_RXD_PA2();
    SET_UART1_TXD_PA3();

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
    UART_Open(UART1, 115200);
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

    /* Init UART1 */
    UART1_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("\nUART Sample Program\n");

    /* UART auto baud rate sample function */
    AutoBaudRate_Test();

    printf("\nUART Sample Program End\n");

    while(1);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Auto Baud Rate Function Test                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void AutoBaudRate_Test(void)
{

    uint32_t u32Item;

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     Pin Configure                                         |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  ______                                            _____  |\n");
    printf("| |      |                                          |     | |\n");
    printf("| |Master|--UART1_TXD(PA.3)  <==>  UART1_RXD(PA.2)--|Slave| |\n");
    printf("| |      |                                          |     | |\n");
    printf("| |______|                                          |_____| |\n");
    printf("|                                                           |\n");
    printf("+-----------------------------------------------------------+\n");

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|     Auto Baud Rate Function Test                          |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Description :                                            |\n");
    printf("|    The sample code needs two boards. One is Master and    |\n");
    printf("|    the other is slave.  Master will send input pattern    |\n");
    printf("|    0x1 with different baud rate. It can check if Slave    |\n");
    printf("|    calculates correct baud rate.                          |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Please select Master or Slave test                       |\n");
    printf("|  [0] Master    [1] Slave                                  |\n");
    printf("+-----------------------------------------------------------+\n");
    u32Item = (uint32_t)getchar();

    if(u32Item == '0')
        AutoBaudRate_TxTest();
    else
        AutoBaudRate_RxTest();

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Auto Baud Rate Function Tx Test                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void AutoBaudRate_TxTest(void)
{
    uint32_t u32Item;

    do
    {

        printf("\n");
        printf("+-----------------------------------------------------------+\n");
        printf("|     Auto Baud Rate Function Test (Master)                 |\n");
        printf("+-----------------------------------------------------------+\n");
        printf("| [1] baud rate 38400 bps                                   |\n");
        printf("| [2] baud rate 57600 bps                                   |\n");
        printf("| [3] baud rate 115200 bps                                  |\n");
        printf("|                                                           |\n");
        printf("| Select baud rate and master will send 0x1 to slave ...    |\n");
        printf("+-----------------------------------------------------------+\n");
        printf("| Quit                                              - [ESC] |\n");
        printf("+-----------------------------------------------------------+\n\n");
        u32Item = (uint32_t)getchar();
        if(u32Item == 27) break;
        printf("%c\n", u32Item);

        /* Set different baud rate */
        switch(u32Item)
        {
            case '1':
                UART1->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 38400);
                break;
            case '2':
                UART1->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 57600);
                break;
            default:
                UART1->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HXT, 115200);
                break;
        }

        /* Send input pattern 0x1 for auto baud rate detection bit length is 1-bit */
        UART_WRITE(UART1, 0x1);

    }
    while(1);

}

/*---------------------------------------------------------------------------------------------------------*/
/*  Get UART Baud Rate Function                                                                            */
/*---------------------------------------------------------------------------------------------------------*/
uint32_t GetUartBaudrate(UART_T* uart)
{
    uint8_t u8UartClkSrcSel = 0, u8UartClkDivNum = 0;
    uint32_t au32ClkTbl[4] = {__HXT, 0, __LXT, __HIRC};
    uint32_t u32BaudDiv;

    /* Get UART clock source selection and UART clock divider number */
    switch((uint32_t)uart)
    {
        case UART0_BASE:
            u8UartClkSrcSel = (CLK->CLKSEL1 & CLK_CLKSEL1_UART0SEL_Msk) >> CLK_CLKSEL1_UART0SEL_Pos;
            u8UartClkDivNum = (CLK->CLKDIV0 & CLK_CLKDIV0_UART0DIV_Msk) >> CLK_CLKDIV0_UART0DIV_Pos;
            break;
        case UART1_BASE:
            u8UartClkSrcSel = (CLK->CLKSEL1 & CLK_CLKSEL1_UART1SEL_Msk) >> CLK_CLKSEL1_UART1SEL_Pos;
            u8UartClkDivNum = (CLK->CLKDIV0 & CLK_CLKDIV0_UART1DIV_Msk) >> CLK_CLKDIV0_UART1DIV_Pos;
            break;
        case UART2_BASE:
            u8UartClkSrcSel = (CLK->CLKSEL3 & CLK_CLKSEL3_UART2SEL_Msk) >> CLK_CLKSEL3_UART2SEL_Pos;
            u8UartClkDivNum = (CLK->CLKDIV4 & CLK_CLKDIV4_UART2DIV_Msk) >> CLK_CLKDIV4_UART2DIV_Pos;
            break;
        case UART3_BASE:
            u8UartClkSrcSel = (CLK->CLKSEL3 & CLK_CLKSEL3_UART3SEL_Msk) >> CLK_CLKSEL3_UART3SEL_Pos;
            u8UartClkDivNum = (CLK->CLKDIV4 & CLK_CLKDIV4_UART3DIV_Msk) >> CLK_CLKDIV4_UART3DIV_Pos;
            break;
        case UART4_BASE:
            u8UartClkSrcSel = (CLK->CLKSEL3 & CLK_CLKSEL3_UART4SEL_Msk) >> CLK_CLKSEL3_UART4SEL_Pos;
            u8UartClkDivNum = (CLK->CLKDIV4 & CLK_CLKDIV4_UART4DIV_Msk) >> CLK_CLKDIV4_UART4DIV_Pos;
            break;
        case UART5_BASE:
            u8UartClkSrcSel = (CLK->CLKSEL3 & CLK_CLKSEL3_UART5SEL_Msk) >> CLK_CLKSEL3_UART5SEL_Pos;
            u8UartClkDivNum = (CLK->CLKDIV4 & CLK_CLKDIV4_UART5DIV_Msk) >> CLK_CLKDIV4_UART5DIV_Pos;
            break;
        case UART6_BASE:
            u8UartClkSrcSel = (CLK->CLKSEL3 & CLK_CLKSEL3_UART6SEL_Msk) >> CLK_CLKSEL3_UART6SEL_Pos;
            u8UartClkDivNum = (CLK->CLKDIV4 & CLK_CLKDIV4_UART6DIV_Msk) >> CLK_CLKDIV4_UART6DIV_Pos;
            break;
        case UART7_BASE:
            u8UartClkSrcSel = (CLK->CLKSEL3 & CLK_CLKSEL3_UART7SEL_Msk) >> CLK_CLKSEL3_UART7SEL_Pos;
            u8UartClkDivNum = (CLK->CLKDIV4 & CLK_CLKDIV4_UART7DIV_Msk) >> CLK_CLKDIV4_UART7DIV_Pos;
            break;
        case UART8_BASE:
            u8UartClkSrcSel = (CLK->CLKSEL2 & CLK_CLKSEL2_UART8SEL_Msk) >> CLK_CLKSEL2_UART8SEL_Pos;
            u8UartClkDivNum = (CLK->CLKDIV5 & CLK_CLKDIV5_UART8DIV_Msk) >> CLK_CLKDIV5_UART8DIV_Pos;
            break;
        case UART9_BASE:
            u8UartClkSrcSel = (CLK->CLKSEL2 & CLK_CLKSEL2_UART9SEL_Msk) >> CLK_CLKSEL2_UART9SEL_Pos;
            u8UartClkDivNum = (CLK->CLKDIV5 & CLK_CLKDIV5_UART9DIV_Msk) >> CLK_CLKDIV5_UART9DIV_Pos;
            break;
        default:
            return 0;
    }

    /* Get PLL/2 clock frequency if UART clock source selection is PLL/2 */
    if(u8UartClkSrcSel == 1)
        au32ClkTbl[u8UartClkSrcSel] = CLK_GetPLLClockFreq() >> 1;

    /* Get UART baud rate divider */
    u32BaudDiv = (uart->BAUD & UART_BAUD_BRD_Msk) >> UART_BAUD_BRD_Pos;

    /* Calculate UART baud rate if baud rate is set in MODE 0 */
    if((uart->BAUD & (UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk)) == UART_BAUD_MODE0)
        return ((au32ClkTbl[u8UartClkSrcSel]) / (u8UartClkDivNum + 1) / (u32BaudDiv + 2)) >> 4;

    /* Calculate UART baud rate if baud rate is set in MODE 2 */
    else if((uart->BAUD & (UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk)) == UART_BAUD_MODE2)
        return ((au32ClkTbl[u8UartClkSrcSel]) / (u8UartClkDivNum + 1) / (u32BaudDiv + 2));

    /* Calculate UART baud rate if baud rate is set in MODE 1 */
    else if((uart->BAUD & (UART_BAUD_BAUDM1_Msk | UART_BAUD_BAUDM0_Msk)) == UART_BAUD_BAUDM1_Msk)
        return ((au32ClkTbl[u8UartClkSrcSel]) / (u8UartClkDivNum + 1) / (u32BaudDiv + 2)) / (((uart->BAUD & UART_BAUD_EDIVM1_Msk) >> UART_BAUD_EDIVM1_Pos) + 1);

    /* Unsupported baud rate setting */
    else
        return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Auto Baud Rate Function Rx Test                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
void AutoBaudRate_RxTest(void)
{
    /* Enable auto baud rate detect function */
    UART1->ALTCTL |= UART_ALTCTL_ABRDEN_Msk;

    printf("\nreceiving input pattern... ");

    /* Forces a write of all user-space buffered data for the given output */
    fflush(stdout);

    /* Wait until auto baud rate detect finished or time-out */
    while((UART1->ALTCTL & UART_ALTCTL_ABRIF_Msk) == 0);

    if(UART1->FIFOSTS & UART_FIFOSTS_ABRDIF_Msk)
    {
        /* Clear auto baud rate detect finished flag */
        UART1->FIFOSTS = UART_FIFOSTS_ABRDIF_Msk;
        printf("Baud rate is %dbps.\n", GetUartBaudrate(UART1));
    }
    else if(UART1->FIFOSTS & UART_FIFOSTS_ABRDTOIF_Msk)
    {
        /* Clear auto baud rate detect time-out flag */
        UART1->FIFOSTS = UART_FIFOSTS_ABRDTOIF_Msk;
        printf("Time-out!\n");
    }

}
