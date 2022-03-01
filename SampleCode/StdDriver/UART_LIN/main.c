/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Transmit LIN frame including header and response in UART LIN mode.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include "NuMicro.h"




#define LIN_ID      0x30

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static volatile int32_t g_i32pointer;
static volatile int32_t g_i32RxCounter;
static uint8_t g_u8SendData[12];
static uint8_t g_u8ReceiveData[9];

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void UART_FunctionTest(void);
int32_t LIN_Tx_FunctionTest(void);
void LIN_Rx_FunctionTest(void);
void UART1_IRQHandler(void);
void SYS_Init(void);
void UART0_Init(void);
void UART1_Init(void);
uint8_t ComputeChksumValue(uint8_t *pu8Buf, uint32_t u32ByteCnt);
void TestItem(void);
uint8_t GetParityValue(uint32_t u32id);


void UART1_IRQHandler(void)
{
    volatile uint32_t u32IntSts = UART1->INTSTS;
    uint32_t u32Data;

    if(u32IntSts & UART_INTSTS_LININT_Msk)
    {
        if(UART1->LINSTS & UART_LINSTS_SLVHDETF_Msk)
        {
            // Clear LIN slave header detection flag
            UART1->LINSTS = UART_LINSTS_SLVHDETF_Msk;
            printf("\n LIN Slave Header detected ");
        }

        if(UART1->LINSTS & (UART_LINSTS_SLVHEF_Msk | UART_LINSTS_SLVIDPEF_Msk | UART_LINSTS_BITEF_Msk))
        {
            // Clear LIN error flag
            UART1->LINSTS = (UART_LINSTS_SLVHEF_Msk | UART_LINSTS_SLVIDPEF_Msk | UART_LINSTS_BITEF_Msk);
            printf("\n LIN error detected ");
        }
    }

    if(u32IntSts & UART_INTSTS_RDAINT_Msk)
    {
        u32Data = UART1->DAT;
        g_u8ReceiveData[g_i32RxCounter] = (uint8_t)u32Data;
        g_i32RxCounter++;
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
int32_t main(void)
{
    (void)g_u8SendData;

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

    /* UART sample LIN function */
    UART_FunctionTest();

    while(1);

}


/*---------------------------------------------------------------------------------------------------------*/
/* Compute Parity Value                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t GetParityValue(uint32_t u32id)
{
    uint32_t u32Res = 0, ID[6], p_Bit[2], mask = 0;

    for(mask = 0; mask < 6; mask++)
        ID[mask] = (u32id & (1 << mask)) >> mask;

    p_Bit[0] = (ID[0] + ID[1] + ID[2] + ID[4]) % 2;
    p_Bit[1] = (!((ID[1] + ID[3] + ID[4] + ID[5]) % 2));

    u32Res = u32id + (p_Bit[0] << 6) + (p_Bit[1] << 7);
    return (uint8_t)u32Res;
}

/*---------------------------------------------------------------------------------------------------------*/
/* Compute CheckSum Value                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
uint8_t ComputeChksumValue(uint8_t *pu8Buf, uint32_t u32ByteCnt)
{
    uint32_t i, CheckSum = 0;

    for(i = 0 ; i < u32ByteCnt; i++)
    {
        CheckSum += pu8Buf[i];
        if(CheckSum >= 256)
            CheckSum -= 255;
    }
    return (uint8_t)(255 - CheckSum);
}


/*---------------------------------------------------------------------------------------------------------*/
/*  LIN Function Test                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void TestItem(void)
{
    printf("+--------------------------------------------------------------------------------+\n");
    printf("|  UART LIN Demo Function Select                                                 |\n");
    printf("+--------------------------------------------------------------------------------+\n");
    printf("|  1 : Demo LIN Tx Function                                                      |\n");
    printf("|  2 : Demo LIN Rx Function                                                      |\n");
    printf("+--------------------------------------------------------------------------------+\n");
    printf("| Quit                                                                   - [ESC] |\n");
    printf("+--------------------------------------------------------------------------------+\n\n");
    printf("Please Select key (1~2): ");

    /* Forces a write of all user-space buffered data for the given output */
    fflush(stdout);
}

void UART_FunctionTest(void)
{
    uint32_t u32Item;

    do
    {
        TestItem();
        u32Item = (uint32_t)getchar();
        printf("%c\n", u32Item);

        switch(u32Item)
        {
            case '1':
                if( LIN_Tx_FunctionTest() <0 )
                    u32Item = 27;
                break;

            case '2':
                LIN_Rx_FunctionTest();
                break;

            default:
                break;
        }
    }
    while(u32Item != 27);

    printf("\nUART Demo Program End\n");

}

int32_t LIN_Tx_FunctionTest(void)
{
    uint32_t u32TimeOutCnt;

    //The sample code will send a LIN header with ID is 0x30 and response field.
    //The response field with 8 data bytes and checksum without including ID.

    uint8_t au8TestPattern[9] = {0x1, 0x2, 0x3, 0x4, 0x5, 0x6, 0x7, 0x8, 0x0}; // 8 data byte + 1 byte checksum

    printf("+--------------------------------------------------------------------------------+\n");
    printf("|  UART LIN Tx Function Test                                                     |\n");
    printf("+--------------------------------------------------------------------------------+\n");
    printf("|  Description :                                                                 |\n");
    printf("|    The sample code will send a LIN header with ID is 0x30 and response field   |\n");
    printf("+--------------------------------------------------------------------------------+\n");

    /* Send break+sync+ID */
    g_i32pointer = 0 ;

    /* Set UART Configuration, LIN Max Speed is 20K */
    UART_SetLineConfig(UART1, 9600, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);

    /* Switch back to LIN Function */
    UART1->FUNCSEL = UART_FUNCSEL_LIN;

    // Send Header
    UART1->LINCTL = UART_LINCTL_PID(LIN_ID) | UART_LINCTL_HSEL_BREAK_SYNC_ID |
                    UART_LINCTL_BSL(1) | UART_LINCTL_BRKFL(12) | UART_LINCTL_IDPEN_Msk;
    /* LIN TX Send Header Enable */
    UART1->LINCTL |= UART_LINCTL_SENDH_Msk;
    /* Wait until break field, sync field and ID field transfer completed */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while((UART1->LINCTL & UART_LINCTL_SENDH_Msk) == UART_LINCTL_SENDH_Msk)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for UART LIN header transfer completed time-out!\n");
            return -1;
        }
    }

    /* Compute checksum without ID and fill checksum value to au8TestPattern[8] */
    au8TestPattern[8] = ComputeChksumValue(&au8TestPattern[0], 8);
    UART_Write(UART1, &au8TestPattern[0], 9);

    printf("\n UART LIN Tx Function Demo End !!\n\n");

    return 0;
}

void LIN_Rx_FunctionTest(void)
{
    //The sample code will detect LIN header(break+sync+ID) and receive response field.
    //The response field with 8 data bytes and checksum without including ID.

    printf("+--------------------------------------------------------------------------------+\n");
    printf("|  UART LIN Rx Function Test                                                     |\n");
    printf("+--------------------------------------------------------------------------------+\n");
    printf("|  Description :                                                                 |\n");
    printf("|    The sample code will receive a LIN response field data                      |\n");
    printf("+--------------------------------------------------------------------------------+\n");

    g_i32RxCounter = 0;

    /* Set UART Configuration, LIN Max Speed is 20K */
    UART_SetLineConfig(UART1, 9600, UART_WORD_LEN_8, UART_PARITY_NONE, UART_STOP_BIT_1);

    /* Switch back to LIN Function */
    UART1->FUNCSEL = UART_FUNCSEL_LIN;

    /* Enable RDA\Time-out\LIN Interrupt */
    UART_ENABLE_INT(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RXTOIEN_Msk | UART_INTEN_LINIEN_Msk));
    NVIC_EnableIRQ(UART1_IRQn);

    // Receive Header: break+sync+ID
    UART1->LINCTL = UART_LINCTL_PID(LIN_ID) | UART_LINCTL_HSEL_BREAK_SYNC_ID | UART_LINCTL_SLVHDEN_Msk |
                    UART_LINCTL_IDPEN_Msk | UART_LINCTL_MUTE_Msk | UART_LINCTL_SLVEN_Msk;

    while(g_i32RxCounter < 9);

    printf("\n Receive Data:\n [ 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x, 0x%x 0x%x ] \n",
           g_u8ReceiveData[0], g_u8ReceiveData[1], g_u8ReceiveData[2], g_u8ReceiveData[3],
           g_u8ReceiveData[4], g_u8ReceiveData[5], g_u8ReceiveData[6], g_u8ReceiveData[7],
           g_u8ReceiveData[8]);

    NVIC_DisableIRQ(UART1_IRQn);

    printf("\n UART LIN Rx Function Demo End !!\n\n");
}
