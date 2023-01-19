/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Use CAN FD mode function to do internal loopback test.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
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
//      <i> Specify a CANFD module to do the internal loopback self-test.
*/
#define CANFD_MODULE       0


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
CANFD_T * g_pCanfd = ((CANFD_MODULE == 0)?CANFD0:(CANFD_MODULE == 1)?CANFD1:(CANFD_MODULE == 2)?CANFD2:CANFD3);
CANFD_FD_MSG_T      g_sRxMsgFrame;
CANFD_FD_MSG_T      g_sTxMsgFrame;
volatile uint8_t   g_u8RxFIFO1CompleteFlag = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
void SYS_Init(void);
void CANFD_TEST_HANDLE(void);
#if (CANFD_MODULE == 0)
void CANFD00_IRQHandler(void);
#elif (CANFD_MODULE == 1)
void CANFD10_IRQHandler(void);
#elif (CANFD_MODULE == 2)
void CANFD20_IRQHandler(void);
#elif (CANFD_MODULE == 3)
void CANFD30_IRQHandler(void);
#else
void CANFD30_IRQHandler(void);
#endif
/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle CAN FD0 Line0 interrupt event                                                             */
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
    CANFD_TEST_HANDLE();
}
/*---------------------------------------------------------------------------------------------------------*/
/* CAN FD0 Callback function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_TEST_HANDLE(void)
{
    printf("IR =0x%08X \n", g_pCanfd->IR);
    /*Clear the Interrupt flag */
    CANFD_ClearStatusFlag(g_pCanfd, CANFD_IR_TOO_Msk | CANFD_IR_RF1N_Msk);
    /*Receive the Rx Fifo1 buffer */
    CANFD_ReadRxFifoMsg(g_pCanfd, 1, &g_sRxMsgFrame);
    g_u8RxFIFO1CompleteFlag = 1;
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

}

/*---------------------------------------------------------------------------------------------------------*/
/*                           CAN FD Tx Rx Function Test                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_CANFD_TxRx_Test(CANFD_FD_MSG_T *psTxMsg, E_CANFD_ID_TYPE eFrameIdType, uint32_t u32Id, uint8_t u8LenType)
{
    uint8_t u8Cnt;
    uint32_t u32TimeOutCnt = CANFD_TIMEOUT;

    /*Set the ID Number*/
    psTxMsg->u32Id = u32Id;
    /*Set the frame type*/
    psTxMsg->eIdType = eFrameIdType;
    /*Set FD frame format attribute */
    psTxMsg->bFDFormat = 1;
    /*Set the bitrate switch attribute*/
    psTxMsg->bBitRateSwitch = 1;

    /*Set data length*/
    if (u8LenType == 0)      psTxMsg->u32DLC = 8;
    else if (u8LenType == 1) psTxMsg->u32DLC = 12;
    else if (u8LenType == 2) psTxMsg->u32DLC = 16;
    else if (u8LenType == 3) psTxMsg->u32DLC = 20;
    else if (u8LenType == 4) psTxMsg->u32DLC = 24;
    else if (u8LenType == 5) psTxMsg->u32DLC = 32;
    else if (u8LenType == 6) psTxMsg->u32DLC = 48;
    else if (u8LenType == 7) psTxMsg->u32DLC = 64;

    for (u8Cnt = 0; u8Cnt < psTxMsg->u32DLC; u8Cnt++) psTxMsg->au8Data[u8Cnt] = u8Cnt;

    g_u8RxFIFO1CompleteFlag = 0;

    /* use message buffer 0 */
    if (eFrameIdType == eCANFD_SID)
        printf("Send to transmit message 0x%08x (11-bit)\n", psTxMsg->u32Id);
    else
        printf("Send to transmit message 0x%08x (29-bit)\n", psTxMsg->u32Id);

    if (CANFD_TransmitTxMsg(g_pCanfd, 0, psTxMsg) != 1)
    {
        printf("Failed to transmit message\n");
    }

    /* Wait the Rx FIFO1 received message */
    while (!g_u8RxFIFO1CompleteFlag)
    {
        if (--u32TimeOutCnt == 0)
        {
            printf("Wait for CANFD Rx FIFO1 received message time-out!\n");
            return;
        }
    }

    printf("Rx FIFO1 : Received message 0x%08X\n", g_sRxMsgFrame.u32Id);
    printf("Message Data : ");

    for (u8Cnt = 0; u8Cnt <  g_sRxMsgFrame.u32DLC; u8Cnt++)
    {
        printf("%02d ,", g_sRxMsgFrame.au8Data[u8Cnt]);
    }

    printf("\n\n");
    memset(&g_sRxMsgFrame, 0, sizeof(g_sRxMsgFrame));
}

/*---------------------------------------------------------------------------------------------------------*/
/*  CAN FD Function Test                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_CANFD_Loopback(void)
{
    uint8_t u8Loop;
    CANFD_FD_T sCANFD_Config;

    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_FD_MODE);
    sCANFD_Config.sBtConfig.bEnableLoopBack = TRUE;
    sCANFD_Config.sBtConfig.sNormBitRate.u32BitRate = 1000000;
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = 4000000;
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

    /* receive 0x110~0x11F in CAN FD0 rx fifo1 buffer by setting mask 0 */
    CANFD_SetSIDFltr(g_pCanfd, 0, CANFD_RX_FIFO1_STD_MASK(0x110, 0x7F0));
    /* receive 0x220 in CAN FD0 rx fifo1 buffer by setting mask 1 */
    CANFD_SetSIDFltr(g_pCanfd, 1, CANFD_RX_FIFO1_STD_MASK(0x22F, 0x7FF));
    /* receive 0x333 in CAN FD0 rx fifo1 buffer by setting mask 2 */
    CANFD_SetSIDFltr(g_pCanfd, 2, CANFD_RX_FIFO1_STD_MASK(0x333, 0x7FF));

    /* receive 0x220~0x22f (29-bit id) in CAN FD0 rx fifo1 buffer by setting mask 0 */
    CANFD_SetXIDFltr(g_pCanfd, 0, CANFD_RX_FIFO1_EXT_MASK_LOW(0x220), CANFD_RX_FIFO1_EXT_MASK_HIGH(0x1FFFFFF0));
    /* receive 0x3333 (29-bit id) in CAN FD0 rx message buffer by setting mask 1 */
    CANFD_SetXIDFltr(g_pCanfd, 1, CANFD_RX_FIFO1_EXT_MASK_LOW(0x3333), CANFD_RX_FIFO1_EXT_MASK_HIGH(0x1FFFFFFF));
    /* receive 0x44444 (29-bit id) in CAN FD0 rx fifo buffer by setting mask 2 */
    CANFD_SetXIDFltr(g_pCanfd, 2, CANFD_RX_FIFO1_EXT_MASK_LOW(0x44444), CANFD_RX_FIFO1_EXT_MASK_HIGH(0x1FFFFFFF));
    /* Reject Non-Matching Standard ID and Extended ID Filter(RX fifo1)*/
    CANFD_SetGFC(g_pCanfd, eCANFD_ACC_NON_MATCH_FRM_RX_FIFO1, eCANFD_ACC_NON_MATCH_FRM_RX_FIFO1, 1, 1);
    /* Enable RX fifo1 new message interrupt using interrupt line 0. */
    CANFD_EnableInt(g_pCanfd, (CANFD_IE_TOOE_Msk | CANFD_IE_RF1NE_Msk), 0, 0, 0);
    /* CAN FD0 Run to Normal mode  */
    CANFD_RunToNormal(g_pCanfd, TRUE);

    for (u8Loop = 0  ; u8Loop < 8; u8Loop++)
    {
        CANFD_CANFD_TxRx_Test(&g_sTxMsgFrame, eCANFD_SID, 0x110 + u8Loop, u8Loop);
    }

    CANFD_CANFD_TxRx_Test(&g_sTxMsgFrame, eCANFD_SID, 0x2FF, 7);
    CANFD_CANFD_TxRx_Test(&g_sTxMsgFrame, eCANFD_SID, 0x333, 7);

    for (u8Loop = 0 ; u8Loop < 8; u8Loop++)
    {
        CANFD_CANFD_TxRx_Test(&g_sTxMsgFrame, eCANFD_XID, 0x220 + u8Loop, u8Loop);
    }

    CANFD_CANFD_TxRx_Test(&g_sTxMsgFrame, eCANFD_XID, 0x3333, 7);
    CANFD_CANFD_TxRx_Test(&g_sTxMsgFrame, eCANFD_XID, 0x44444, 7);

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
int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART to 115200-8n1 for print message */
    UART0_Init();

    printf("\n CANFD%d FD Mode Loopback example\r\n", ((CANFD_MODULE == 0)?0:(CANFD_MODULE == 1)?1:(CANFD_MODULE == 2)?2:3));
    /* CAN FD Loopback Test */
    CANFD_CANFD_Loopback();
    printf("\n CANFD%d FD Mode Loopback Test Done\r\n", ((CANFD_MODULE == 0)?0:(CANFD_MODULE == 1)?1:(CANFD_MODULE == 2)?2:3));

    while (1) {}
}
