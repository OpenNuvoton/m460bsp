/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    An example of interrupt control using CAN bus communication.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "stdio.h"
#include "string.h"
#include "NuMicro.h"

/*
//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
*/

/*
// <o0> CANFD Module Selection
//      <0=> CANFD Module 0
//      <1=> CANFD Module 1
//      <2=> CANFD Module 2
//      <3=> CANFD Module 3
//      <i> Specify a CANFD module to transmit or receive messages by selected in the Master or Slave selection menu.
*/
#define CANFD_MODULE       0


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
CANFD_T * g_pCanfd = ((CANFD_MODULE == 0) ? CANFD0 : (CANFD_MODULE == 1) ? CANFD1 : (CANFD_MODULE == 2) ? CANFD2 : CANFD3);
CANFD_FD_MSG_T      g_sRxMsgFrame;
CANFD_FD_MSG_T      g_sTxMsgFrame;
volatile uint8_t   g_u8RxFIFO0CompleteFlag = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void CAN_ShowRecvMessage(void);
void CAN_RxTest(void);
void CAN_TxTest(void);
void CAN_TxRxINTTest(void);
void CAN_SendMessage(CANFD_FD_MSG_T *psTxMsg, E_CANFD_ID_TYPE eIdType, uint32_t u32Id, uint8_t u8Len);


/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle CAN Line 0 interrupt event                                                                */
/*---------------------------------------------------------------------------------------------------------*/
#if (CANFD_MODULE == 0)
void CANFD00_IRQHandler(void)
#elif (CANFD_MODULE == 1)
void CANFD10_IRQHandler(void)
#elif (CANFD_MODULE == 2)
void CANFD20_IRQHandler(void)
#elif (CANFD_MODULE == 3)
void CANFD30_IRQHandler(void)
#else
void CANFD30_IRQHandler(void)
#endif
{
    printf("IR =0x%08X \n", g_pCanfd->IR);
    /* Clear the Interrupt flag */
    CANFD_ClearStatusFlag(g_pCanfd, CANFD_IR_TOO_Msk | CANFD_IR_RF1N_Msk);
    /* Receive the Rx FIFO0 buffer */
    CANFD_ReadRxFifoMsg(g_pCanfd, 0, &g_sRxMsgFrame);
    g_u8RxFIFO0CompleteFlag = 1;
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

#if (CANFD_MODULE == 0)
    /* Select CAN FD0 clock source is HCLK */
    CLK_SetModuleClock(CANFD0_MODULE, CLK_CLKSEL0_CANFD0SEL_HCLK, CLK_CLKDIV5_CANFD0(1));

    /* Enable CAN FD0 peripheral clock */
    CLK_EnableModuleClock(CANFD0_MODULE);
#elif (CANFD_MODULE == 1)
    /* Select CAN FD1 clock source is HCLK */
    CLK_SetModuleClock(CANFD1_MODULE, CLK_CLKSEL0_CANFD1SEL_HCLK, CLK_CLKDIV5_CANFD1(1));

    /* Enable CAN FD1 peripheral clock */
    CLK_EnableModuleClock(CANFD1_MODULE);
#elif (CANFD_MODULE == 2)
    /* Select CAN FD2 clock source is HCLK */
    CLK_SetModuleClock(CANFD2_MODULE, CLK_CLKSEL0_CANFD2SEL_HCLK, CLK_CLKDIV5_CANFD2(1));

    /* Enable CAN FD2 peripheral clock */
    CLK_EnableModuleClock(CANFD2_MODULE);
#elif (CANFD_MODULE == 3)
    /* Select CAN FD3 clock source is HCLK */
    CLK_SetModuleClock(CANFD3_MODULE, CLK_CLKSEL0_CANFD3SEL_HCLK, CLK_CLKDIV5_CANFD3(1));

    /* Enable CAN FD3 peripheral clock */
    CLK_EnableModuleClock(CANFD3_MODULE);
#else
    /* Select CAN FD3 clock source is HCLK */
    CLK_SetModuleClock(CANFD3_MODULE, CLK_CLKSEL0_CANFD3SEL_HCLK, CLK_CLKDIV5_CANFD3(1));

    /* Enable CAN FD3 peripheral clock */
    CLK_EnableModuleClock(CANFD3_MODULE);
#endif

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

#if (CANFD_MODULE == 0)
    /* Set PC multi-function pins for CAN FD0 RXD and TXD */
    SET_CAN0_RXD_PC4();
    SET_CAN0_TXD_PC5();
#elif (CANFD_MODULE == 1)
    /* Set PC multi-function pins for CAN FD1 RXD and TXD */
    SET_CAN1_RXD_PC2();
    SET_CAN1_TXD_PC3();
#elif (CANFD_MODULE == 2)
    /* Set PC multi-function pins for CAN FD2 RXD and TXD */
    SET_CAN2_RXD_PC0();
    SET_CAN2_TXD_PC1();
#elif (CANFD_MODULE == 3)
    /* Set PC multi-function pins for CAN FD3 RXD and TXD */
    SET_CAN3_RXD_PC6();
    SET_CAN3_TXD_PC7();
#else
    /* Set PC multi-function pins for CAN FD3 RXD and TXD */
    SET_CAN3_RXD_PC6();
    SET_CAN3_TXD_PC7();
#endif
}


/*---------------------------------------------------------------------------------------------------------*/
/* CAN Function Test Menu (Master)                                                                         */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_TestItem(void)
{
    printf("\n");
    printf("+------------------------------------------------------------+\n");
    printf("|                CAN Tx Function Test (Master)               |\n");
    printf("+------------------------------------------------------------+\n");
    printf("| [1] Standard ID = 0x111            ( Data length 8 bytes ) |\n");
    printf("| [2] Standard ID = 0x22F            ( Data length 8 bytes ) |\n");
    printf("| [3] Standard ID = 0x333            ( Data length 8 bytes ) |\n");
    printf("| [4] Extended ID = 0x221            ( Data length 8 bytes ) |\n");
    printf("| [5] Extended ID = 0x3333           ( Data length 8 bytes ) |\n");
    printf("| [6] Extended ID = 0x44444          ( Data length 8 bytes ) |\n");
    printf("| Select ID number and master will send message to slave ... |\n");
    printf("+------------------------------------------------------------+\n");
    printf("| Quit                                               - [ESC] |\n");
    printf("+------------------------------------------------------------+\n\n");
}


/*---------------------------------------------------------------------------------------------------------*/
/* CAN Function Tx Test                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_TxTest(void)
{
    uint8_t u8Item;

    do
    {
        CAN_TestItem();
        u8Item = getchar();

        switch(u8Item)
        {

            case '1':
                /* Standard ID = 0x111, Data length 8 bytes */
                CAN_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x111, 8);
                break;

            case '2':
                /* Standard ID = 0x22F, Data length 8 bytes */
                CAN_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x22F, 8);
                break;

            case '3':
                /* Standard ID = 0x333, Data length 8 bytes */
                CAN_SendMessage(&g_sTxMsgFrame, eCANFD_SID, 0x333, 8);
                break;

            case '4':
                /* Extend ID = 0x221, Data length 8 bytes */
                CAN_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x221, 8);
                break;

            case '5':
                /* Extend ID = 0x3333, Data length 8 bytes */
                CAN_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x3333, 8);
                break;

            case '6':
                /* Extend ID = 0x44444, Data length 8 bytes */
                CAN_SendMessage(&g_sTxMsgFrame, eCANFD_XID, 0x44444, 8);
                break;

            default:
                break;

        }
    }
    while(u8Item != 27);
}


/*---------------------------------------------------------------------------------------------------------*/
/* Send CAN Message Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_SendMessage(CANFD_FD_MSG_T *psTxMsg, E_CANFD_ID_TYPE eIdType, uint32_t u32Id, uint8_t u8Len)
{
    uint8_t u8Cnt;

    /* Set the ID number */
    psTxMsg->u32Id = u32Id;
    /* Set the ID type */
    psTxMsg->eIdType = eIdType;
    /* Set the frame type */
    psTxMsg->eFrmType = eCANFD_DATA_FRM;
    /* Set the bitrate switch attribute */
    psTxMsg->bBitRateSwitch = 0;
    /* Set data length */
    psTxMsg->u32DLC = u8Len;

    for(u8Cnt = 0; u8Cnt < psTxMsg->u32DLC; u8Cnt++) psTxMsg->au8Data[u8Cnt] = u8Cnt;

    g_u8RxFIFO0CompleteFlag = 0;

    /* Use message buffer 0 */
    if(eIdType == eCANFD_SID)
        printf("Send to transmit message 0x%08x (11-bit)\n", psTxMsg->u32Id);
    else
        printf("Send to transmit message 0x%08x (29-bit)\n", psTxMsg->u32Id);

    if(CANFD_TransmitTxMsg(g_pCanfd, 0, psTxMsg) != eCANFD_TRANSMIT_SUCCESS)
    {
        printf("Failed to transmit message\n");
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/* CAN Function Rx Test                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_RxTest(void)
{
    uint8_t u8Cnt = 0;

    printf("Start CAN bus reception :\n");

    do
    {
        while(!g_u8RxFIFO0CompleteFlag);
        CAN_ShowRecvMessage();
        g_u8RxFIFO0CompleteFlag = 0;
        memset(&g_sRxMsgFrame, 0, sizeof(g_sRxMsgFrame));
        u8Cnt++;
    }
    while(u8Cnt < 6);
}


/*---------------------------------------------------------------------------------------------------------*/
/* Show the CAN Message Function                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_ShowRecvMessage(void)
{
    uint8_t u8Cnt;

    if(g_sRxMsgFrame.eIdType == eCANFD_SID)
        printf("Rx FIFO0(Standard ID) ID = 0x%08X\n", g_sRxMsgFrame.u32Id);
    else
        printf("Rx FIFO0(Extended ID) ID = 0x%08X\n", g_sRxMsgFrame.u32Id);

    printf("Message Data(%02d bytes) : ", g_sRxMsgFrame.u32DLC);

    for(u8Cnt = 0; u8Cnt <  g_sRxMsgFrame.u32DLC; u8Cnt++)
    {
        printf("%02d ,", g_sRxMsgFrame.au8Data[u8Cnt]);
    }

    printf("\n\n");
}


/*---------------------------------------------------------------------------------------------------------*/
/* Init CAN                                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_Init(void)
{
    CANFD_FD_T sCANFD_Config;

    printf("+-------------------------------------------------------------+\n");
    printf("|                        Pin Configure                        |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|      CAN_TXD                            CAN_TXD(Any board)  |\n");
    printf("|      CAN_RXD                            CAN_RXD(Any board)  |\n");
    printf("|         |-----------|  CANBUS  |-----------|                |\n");
    printf("|  ------>|           |<-------->|           |<------         |\n");
    printf("|   CAN_TX|    CAN    |  CAN_H   |    CAN    |CAN_TX          |\n");
    printf("|         |Transceiver|          |Transceiver|                |\n");
    printf("|  <------|           |<-------->|           |------>         |\n");
    printf("|   CAN_RX|           |  CAN_L   |           |CAN_RX          |\n");
    printf("|         |-----------|          |-----------|                |\n");
    printf("+-------------------------------------------------------------+\n\n");

    /* Get the CAN configuration value */
    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_MODE);
    sCANFD_Config.sBtConfig.sNormBitRate.u32BitRate = 1000000;
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = 0;
    /* Open the CAN feature */
    CANFD_Open(g_pCanfd, &sCANFD_Config);

#if (CANFD_MODULE == 0)
    NVIC_EnableIRQ(CANFD00_IRQn);
#elif (CANFD_MODULE == 1)
    NVIC_EnableIRQ(CANFD10_IRQn);
#elif (CANFD_MODULE == 2)
    NVIC_EnableIRQ(CANFD20_IRQn);
#elif (CANFD_MODULE == 3)
    NVIC_EnableIRQ(CANFD30_IRQn);
#else
    NVIC_EnableIRQ(CANFD30_IRQn);
#endif

    /* Receive 0x110~0x11F in CAN rx FIFO0 buffer by setting mask 0 */
    CANFD_SetSIDFltr(g_pCanfd, 0, CANFD_RX_FIFO0_STD_MASK(0x110, 0x7F0));
    /* Receive 0x22F in CAN rx FIFO0 buffer by setting mask 1 */
    CANFD_SetSIDFltr(g_pCanfd, 1, CANFD_RX_FIFO0_STD_MASK(0x22F, 0x7FF));
    /* Receive 0x333 in CAN rx FIFO0 buffer by setting mask 2 */
    CANFD_SetSIDFltr(g_pCanfd, 2, CANFD_RX_FIFO0_STD_MASK(0x333, 0x7FF));

    /* Receive 0x220~0x22F (29-bit id) in CAN rx FIFO0 buffer by setting mask 0 */
    CANFD_SetXIDFltr(g_pCanfd, 0, CANFD_RX_FIFO0_EXT_MASK_LOW(0x220), CANFD_RX_FIFO0_EXT_MASK_HIGH(0x1FFFFFF0));
    /* Receive 0x3333 (29-bit id) in CAN rx FIFO0 buffer by setting mask 1 */
    CANFD_SetXIDFltr(g_pCanfd, 1, CANFD_RX_FIFO0_EXT_MASK_LOW(0x3333), CANFD_RX_FIFO0_EXT_MASK_HIGH(0x1FFFFFFF));
    /* Receive 0x44444 (29-bit id) in CAN rx FIFO0 buffer by setting mask 2 */
    CANFD_SetXIDFltr(g_pCanfd, 2, CANFD_RX_FIFO0_EXT_MASK_LOW(0x44444), CANFD_RX_FIFO0_EXT_MASK_HIGH(0x1FFFFFFF));
    /* Reject Non-Matching Standard ID and Extended ID Filter(RX FIFO0) */
    CANFD_SetGFC(g_pCanfd, eCANFD_REJ_NON_MATCH_FRM, eCANFD_REJ_NON_MATCH_FRM, 1, 1);
    /* Enable RX FIFO0 new message interrupt using interrupt line 0 */
    CANFD_EnableInt(g_pCanfd, (CANFD_IE_TOOE_Msk | CANFD_IE_RF0NE_Msk), 0, 0, 0);
    /* CAN Run to Normal mode */
    CANFD_RunToNormal(g_pCanfd, TRUE);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Fini CAN                                                                                                */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_Fini(void)
{

#if (CANFD_MODULE == 0)
    NVIC_DisableIRQ(CANFD00_IRQn);
#elif (CANFD_MODULE == 1)
    NVIC_DisableIRQ(CANFD10_IRQn);
#elif (CANFD_MODULE == 2)
    NVIC_DisableIRQ(CANFD20_IRQn);
#elif (CANFD_MODULE == 3)
    NVIC_DisableIRQ(CANFD30_IRQn);
#else
    NVIC_DisableIRQ(CANFD30_IRQn);
#endif
    CANFD_Close(g_pCanfd);
}

/*---------------------------------------------------------------------------------------------------------*/
/* CAN Tx Rx Interrupt Function Test                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_TxRxINTTest(void)
{
    uint8_t u8Item;

    /* CAN interface initialization */
    CAN_Init();

    printf("+----------------------------------------------------------------------------+\n");
    printf("|                              CAN Function Test                             |\n");
    printf("+----------------------------------------------------------------------------+\n");
    printf("|  Description :                                                             |\n");
    printf("|    The sample code needs two boards. One is master(CAN transmitter) and    |\n");
    printf("|    the other is slave(CAN receiver). Master will send 6 messages with      |\n");
    printf("|    different sizes of data and ID to the slave. Slave will check if        |\n");
    printf("|    received data is correct after getting 6 messages data.                 |\n");
    printf("|  Please select Master or Slave test                                        |\n");
    printf("|  [0] Master(CAN transmitter)    [1] Slave(CAN receiver)                    |\n");
    printf("+----------------------------------------------------------------------------+\n\n");

    u8Item = getchar();

    if(u8Item == '0')
    {
        CAN_TxTest();
    }
    else
    {
        CAN_RxTest();
    }

    /* CAN interface finalization */
    CAN_Fini();

    printf("CAN Sample Code End.\n");
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*                                         Main Function                                                   */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();

    printf("\n CANFD%d CAN mode transmission test\r\n", ((CANFD_MODULE == 0) ? 0 : (CANFD_MODULE == 1) ? 1 : (CANFD_MODULE == 2) ? 2 : 3));

    CAN_TxRxINTTest();

    while(1) {}
}
