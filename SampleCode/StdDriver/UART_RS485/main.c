/****************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Transmit and receive data in UART RS485 mode.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"



#define IS_USE_RS485NMM     0    /* 1:Select NMM_Mode , 0:Select AAD_Mode */
#define MATCH_ADDRSS1       0xC0
#define MATCH_ADDRSS2       0xA2
#define UNMATCH_ADDRSS1     0xB1
#define UNMATCH_ADDRSS2     0xD3

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static uint8_t g_u8SendDataGroup1[10] = {0};
static uint8_t g_u8SendDataGroup2[10] = {0};
static uint8_t g_u8SendDataGroup3[10] = {0};
static uint8_t g_u8SendDataGroup4[10] = {0};


/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
extern char GetChar(void);
void RS485_SendAddressByte(uint8_t u8data);
void RS485_SendDataByte(uint8_t *pu8TxBuf, uint32_t u32WriteBytes);
void RS485_9bitModeMaster(void);
void RS485_FunctionTest(void);
void RS485_HANDLE(void);
void UART1_IRQHandler(void);
void SYS_Init(void);
void UART0_Init(void);
void UART1_Init(void);
void RS485_9bitModeSlave(void);

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle UART Channel 1 interrupt event                                                            */
/*---------------------------------------------------------------------------------------------------------*/
void UART1_IRQHandler(void)
{
    RS485_HANDLE();
}

/*---------------------------------------------------------------------------------------------------------*/
/* RS485 Callback function                                                                                 */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_HANDLE(void)
{
    volatile uint32_t u32Addr = 0;
    volatile uint32_t u32IntSts = UART1->INTSTS;

    if((u32IntSts & UART_INTSTS_RLSINT_Msk) && (u32IntSts & UART_INTSTS_RDAINT_Msk))    /* RLS INT & RDA INT */
    {
        if(UART_RS485_GET_ADDR_FLAG(UART1))         /* ADD_IF, RS485 mode */
        {
            u32Addr = UART_READ(UART1);
            UART_RS485_CLEAR_ADDR_FLAG(UART1);      /* clear ADD_IF flag */
            printf("\nAddr=0x%x,Get:", u32Addr);

#if(IS_USE_RS485NMM ==1) /* RS485 NMM Mode */

            /* if address match, enable RX to receive data, otherwise to disable RX. */
            /* In NMM mode, user can decide multi-address filter. In AAD mode, only one address can set */
            if((u32Addr == MATCH_ADDRSS1) || (u32Addr == MATCH_ADDRSS2))
            {
                UART1->FIFO &= ~UART_FIFO_RXOFF_Msk;   /* Enable RS485 RX */
            }
            else
            {
                UART1->FIFO |= UART_FIFO_RXOFF_Msk;    /* Disable RS485 RX */
                UART1->FIFO |= UART_FIFO_RXRST_Msk;    /* Clear data from RX FIFO */
            }
#endif
        }
    }
    else if(u32IntSts & UART_INTSTS_RDAINT_Msk)        /* Rx Ready INT */
    {
        /* Handle received data */
        printf("%d,", UART1->DAT);

        /* Forces a write of all user-space buffered data for the given output */
        fflush(stdout);
    }

    else if(u32IntSts & UART_INTSTS_BUFERRINT_Msk)       /* Buffer Error INT */
    {
        printf("\nBuffer Error...\n");
        UART_ClearIntFlag(UART1, UART_INTSTS_BUFERRINT_Msk);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Transmit Control  (Address Byte: Parity Bit =1 , Data Byte:Parity Bit =0)                        */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_SendAddressByte(uint8_t u8data)
{
    /* Set UART parity as MARK and skip baud rate setting */
    UART_SetLineConfig(UART1, 0, UART_WORD_LEN_8, UART_PARITY_MARK, UART_STOP_BIT_1);

    /* Send data */
    UART_WRITE(UART1, u8data);
}

void RS485_SendDataByte(uint8_t *pu8TxBuf, uint32_t u32WriteBytes)
{
    /* Set UART parity as SPACE and skip baud rate setting */
    UART_SetLineConfig(UART1, 0, UART_WORD_LEN_8, UART_PARITY_SPACE, UART_STOP_BIT_1);

    /* Send data */
    UART_Write(UART1, pu8TxBuf, u32WriteBytes);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Transmit Test                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_9bitModeMaster()
{
    int32_t i32;

    printf("+-----------------------------------------------------------+\n");
    printf("|               RS485 9-bit Master Test                     |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| The function will send different address with 10 data     |\n");
    printf("| bytes to test RS485 9-bit mode. Please connect TX/RX to   |\n");
    printf("| another board and wait its ready to receive.              |\n");
    printf("| Press any key to start...                                 |\n");
    printf("+-----------------------------------------------------------+\n\n");
    GetChar();

    /* Set RS485-Master as AUD mode */
    /* Enable AUD mode to HW control RTS pin automatically */
    /* You also can use GPIO to control RTS pin for replacing AUD mode */
    UART_SelectRS485Mode(UART1, UART_ALTCTL_RS485AUD_Msk, 0);

    /* Set RTS pin active level as high level active */
    UART1->MODEM = (UART1->MODEM & (~UART_MODEM_RTSACTLV_Msk)) | UART_RTS_IS_HIGH_LEV_ACTIVE;

    /* Prepare Data to transmit */
    for(i32 = 0; i32 < 10; i32++)
    {
        g_u8SendDataGroup1[i32] = (uint8_t)i32;
        g_u8SendDataGroup2[i32] = (uint8_t)i32 + 10;
        g_u8SendDataGroup3[i32] = (uint8_t)i32 + 20;
        g_u8SendDataGroup4[i32] = (uint8_t)i32 + 30;
    }

    /* Send different address and data for test */
    printf("Send Address %x and data 0~9\n", MATCH_ADDRSS1);
    RS485_SendAddressByte(MATCH_ADDRSS1);
    RS485_SendDataByte(g_u8SendDataGroup1, 10);

    printf("Send Address %x and data 10~19\n", UNMATCH_ADDRSS1);
    RS485_SendAddressByte(UNMATCH_ADDRSS1);
    RS485_SendDataByte(g_u8SendDataGroup2, 10);

    printf("Send Address %x and data 20~29\n", MATCH_ADDRSS2);
    RS485_SendAddressByte(MATCH_ADDRSS2);
    RS485_SendDataByte(g_u8SendDataGroup3, 10);

    printf("Send Address %x and data 30~39\n", UNMATCH_ADDRSS2);
    RS485_SendAddressByte(UNMATCH_ADDRSS2);
    RS485_SendDataByte(g_u8SendDataGroup4, 10);

    printf("Transfer Done\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Receive Test  (IS_USE_RS485NMM: 0:AAD  1:NMM)                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_9bitModeSlave(void)
{
    uint32_t u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */

    /* Set Data Format, only need parity enable whatever parity ODD/EVEN */
    UART_SetLineConfig(UART1, 0, UART_WORD_LEN_8, UART_PARITY_EVEN, UART_STOP_BIT_1);

    /* Set RX Trigger Level as 1 byte */
    UART1->FIFO = (UART1->FIFO & (~UART_FIFO_RFITL_Msk)) | UART_FIFO_RFITL_1BYTE;

    /* Set RTS pin active level as high level active */
    UART1->MODEM = (UART1->MODEM & (~UART_MODEM_RTSACTLV_Msk)) | UART_RTS_IS_HIGH_LEV_ACTIVE;

#if (IS_USE_RS485NMM == 1)

    printf("+-----------------------------------------------------------+\n");
    printf("|    Normal Multidrop Operation Mode                        |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| The function is used to test 9-bit slave mode.            |\n");
    printf("| Only Address %x and %x data can receive                   |\n", MATCH_ADDRSS1, MATCH_ADDRSS2);
    printf("+-----------------------------------------------------------+\n");

    /* Set Receiver disabled before set RS485-NMM mode */
    UART1->FIFO |= UART_FIFO_RXOFF_Msk;

    /* Set RS485-NMM Mode */
    UART_SelectRS485Mode(UART1, UART_ALTCTL_RS485NMM_Msk | UART_ALTCTL_RS485AUD_Msk, 0);

#else
    printf("Auto Address Match Operation Mode\n");
    printf("+-----------------------------------------------------------+\n");
    printf("| The function is used to test 9-bit slave mode.            |\n");
    printf("|    Auto Address Match Operation Mode                      |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|Only Address %x,data can receive                           |\n", MATCH_ADDRSS1);
    printf("+-----------------------------------------------------------+\n");

    /* Set RS485-AAD Mode and address match is 0xC0 */
    UART_SelectRS485Mode(UART1, UART_ALTCTL_RS485AAD_Msk | UART_ALTCTL_RS485AUD_Msk, MATCH_ADDRSS1);

#endif

    /* Set RS485 address detection enable */
    UART1->ALTCTL |= UART_ALTCTL_ADDRDEN_Msk;

    /* Enable RDA\RLS Interrupt */
    UART_ENABLE_INT(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RLSIEN_Msk));

    /* Enable UART1 interrupt */
    NVIC_EnableIRQ(UART1_IRQn);

    printf("Ready to receive data...(Press any key to stop test)\n");
    GetChar();

    /* Flush FIFO */
    while(UART_GET_RX_EMPTY(UART1) == 0)
    {
        UART_READ(UART1);
        if(--u32TimeOutCnt == 0) break;
    }

    /* Disable RDA/RLS interrupt */
    UART_DISABLE_INT(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_RLSIEN_Msk));
    NVIC_DisableIRQ(UART1_IRQn);

    /* Set UART Function */
    UART_Open(UART1, 115200);

    printf("\n\nEnd test\n");
}


/*---------------------------------------------------------------------------------------------------------*/
/*  RS485 Function Test                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void RS485_FunctionTest()
{
    uint32_t u32Item;

    printf("\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|      RS485 Function Test IO Setting                       |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  ______                                            _____  |\n");
    printf("| |      |                                          |     | |\n");
    printf("| |Master|--UART1_TXD(PA.3)        UART1_RXD(PA.2)--|Slave| |\n");
    printf("| |      |--UART1_nRTS(PA.0)      UART1_nRTS(PA.0)--|     | |\n");
    printf("| |______|                                          |_____| |\n");
    printf("|                                                           |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|       RS485 Function Test                                 |\n");
    printf("+-----------------------------------------------------------+\n");
    printf("|  Please select Master or Slave test                       |\n");
    printf("|  [0] Master    [1] Slave                                  |\n");
    printf("+-----------------------------------------------------------+\n\n");
    u32Item = (uint32_t)getchar();

    /*
        The sample code is used to test RS485 9-bit mode and needs
        two Module test board to complete the test.

        Master:
            1.Set AUD mode and HW will control RTS pin. RTSACTLV is set to '0'.
            2.Master will send four different address with 10 bytes data to test Slave.
            3.Address bytes : the parity bit should be '1'. (Set UART_LINE = 0x2B)
            4.Data bytes : the parity bit should be '0'. (Set UART_LINE = 0x3B)
            5.RTS pin is low in idle state. When master is sending, RTS pin will be pull high.

        Slave:
            1.Set AAD and AUD mode firstly. RTSACTLV is set to '0'.
            2.The received byte, parity bit is '1' , is considered "ADDRESS".
            3.The received byte, parity bit is '0' , is considered "DATA".  (Default)
            4.AAD: The slave will ignore any data until ADDRESS match address match value.
              When RLS and RDA interrupt is happened,it means the ADDRESS is received.
              Check if RS485 address byte detect flag is set and read RX FIFO data to clear ADDRESS stored in RX FIFO.

              NMM: The slave will ignore data byte until RXOFF is disabled.
              When RLS and RDA interrupt is happened,it means the ADDRESS is received.
              Check the ADDRESS is match or not by user in UART_IRQHandler.
              If the ADDRESS is match, clear RXOFF bit to receive data byte.
              If the ADDRESS is not match, set RXOFF bit to avoid data byte stored in FIFO.

        Note: User can measure transmitted data waveform on TXD and RXD pin.
              RTS pin is used for RS485 transceiver to control transmission direction.
              RTS pin is low in idle state. When master is sending data, RTS pin will be pull high.
              The connection to RS485 transceiver is as following figure for reference.
               __________     ___________      ___________      __________
              |          |   |           |    |           |    |          |
              |Master    |   |RS485      |    |RS485      |    |Slave     |
              | UART_TXD |---|Transceiver|<==>|Transceiver|----| UART_RXD |
              | UART_nRTS|---|           |    |           |----| UART_nRTS|
              |__________|   |___________|    |___________|    |__________|
    */

    if(u32Item == '0')
        RS485_9bitModeMaster();
    else
        RS485_9bitModeSlave();

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

    /* Set PA multi-function pins for UART1 RXD(PA.2), TXD(PA.3) and nRTS(PA.0) */
    SET_UART1_RXD_PA2();
    SET_UART1_TXD_PA3();
    SET_UART1_nRTS_PA0();

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

    /* UART RS485 sample function */
    RS485_FunctionTest();

    printf("\nUART Sample Program End\n");

    while(1);

}
