/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Use CAN FD Monitor mode to listen to CAN bus communication test.
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
//      <i> Specify a CANFD module to listen to CAN bus communication.
*/
#define CANFD_MODULE       0


/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
CANFD_T * g_pCanfd = ((CANFD_MODULE == 0) ? CANFD0 : (CANFD_MODULE == 1) ? CANFD1 : (CANFD_MODULE == 2) ? CANFD2 : CANFD3);
CANFD_FD_MSG_T   g_sRxFIFO0MsgFrame[3];
CANFD_FD_MSG_T   g_sRxFIFO1MsgFrame[3];
uint8_t g_u8RxFIFO0RcvOk = 0;
uint8_t g_u8RxFIFO1RcvOk = 0;
uint8_t g_u8RxFIFO0MsgIndex = 0;
uint8_t g_u8RxFIFO1MsgIndex = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void CANFD_MonitorMode_Init(uint32_t u32NormBitRate, uint32_t u32DataBitRate);
uint32_t Get_CANFD_NominalBitRate(CANFD_T *psCanfd);
uint32_t Get_CANFD_DataBitRate(CANFD_T *psCanfd);
void CANFD_ShowMsg(CANFD_FD_MSG_T *sRxMsg);
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
/* ISR to handle CAN FD Line 0 interrupt event                                                             */
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
    /* Rx FIFO 0 New Message Interrupt */
    if(g_pCanfd->IR & CANFD_IR_RF0N_Msk)
    {
        g_u8RxFIFO0RcvOk = 1;
        CANFD_ClearStatusFlag(g_pCanfd, CANFD_IR_RF0N_Msk);
    }

    /* Rx FIFO 1 New Message Interrupt */
    if(g_pCanfd->IR & CANFD_IR_RF1N_Msk)
    {
        g_u8RxFIFO1RcvOk = 1;
        CANFD_ClearStatusFlag(g_pCanfd, CANFD_IR_RF1N_Msk);
    }

    /* Rx FIFO 0 Message Lost Interrupt */
    if(g_pCanfd->IR & CANFD_IR_RF0L_Msk)
    {
        printf("Rx FIFO 0 Message Lost(Standard ID)\n");
        CANFD_ClearStatusFlag(g_pCanfd, CANFD_IR_RF0L_Msk);
    }

    /* Rx FIFO 1 Message Lost Interrupt */
    if(g_pCanfd->IR & CANFD_IR_RF1L_Msk)
    {
        printf("Rx FIFO 1 Message Lost(Extended ID)\n");
        CANFD_ClearStatusFlag(g_pCanfd, CANFD_IR_RF1L_Msk);
    }
}

/*---------------------------------------------------------------------------*/
/* Show Message Function                                                     */
/*---------------------------------------------------------------------------*/
void CANFD_ShowMsg(CANFD_FD_MSG_T *sRxMsg)
{
    uint8_t u8Cnt;

    /* Show the message information */
    if(sRxMsg->eIdType == eCANFD_SID)
        printf("Rx buf 0: ID = 0x%08X(11-bit), DLC = %d\n", sRxMsg->u32Id, sRxMsg->u32DLC);
    else
        printf("Rx buf 1: ID = 0x%08X(29-bit), DLC = %d\n", sRxMsg->u32Id, sRxMsg->u32DLC);

    printf("Message Data : ");

    for(u8Cnt = 0; u8Cnt < sRxMsg->u32DLC; u8Cnt++)
    {
        printf("%02u ,", sRxMsg->au8Data[u8Cnt]);
    }

    printf("\n\n");
}

/*---------------------------------------------------------------------------*/
/* Get the CAN FD interface Nominal bit rate Function                        */
/*---------------------------------------------------------------------------*/
uint32_t Get_CANFD_NominalBitRate(CANFD_T *psCanfd)
{
    uint32_t u32BitRate = 0;
    uint32_t u32CanClk  = 0;
    uint32_t u32CanDiv  = 0;
    uint8_t  u8Tq = 0;
    uint8_t  u8NtSeg1 = 0;
    uint8_t  u8NtSeg2 = 0;

#if (CANFD_MODULE == 0)
    if(CLK_GetModuleClockSource(CANFD0_MODULE) == (CLK_CLKSEL0_CANFD0SEL_HCLK >> CLK_CLKSEL0_CANFD0SEL_Pos))
#elif (CANFD_MODULE == 1)
    if(CLK_GetModuleClockSource(CANFD1_MODULE) == (CLK_CLKSEL0_CANFD1SEL_HCLK >> CLK_CLKSEL0_CANFD1SEL_Pos))
#elif (CANFD_MODULE == 2)
    if(CLK_GetModuleClockSource(CANFD2_MODULE) == (CLK_CLKSEL0_CANFD2SEL_HCLK >> CLK_CLKSEL0_CANFD2SEL_Pos))
#elif (CANFD_MODULE == 3)
    if(CLK_GetModuleClockSource(CANFD3_MODULE) == (CLK_CLKSEL0_CANFD3SEL_HCLK >> CLK_CLKSEL0_CANFD3SEL_Pos))
#else
    if(CLK_GetModuleClockSource(CANFD3_MODULE) == (CLK_CLKSEL0_CANFD3SEL_HCLK >> CLK_CLKSEL0_CANFD3SEL_Pos))
#endif
        u32CanClk = CLK_GetHCLKFreq();
    else
        u32CanClk = CLK_GetHXTFreq();

#if (CANFD_MODULE == 0)
    u32CanDiv = ((CLK->CLKDIV5 & CLK_CLKDIV5_CANFD0DIV_Msk) >> CLK_CLKDIV5_CANFD0DIV_Pos) + 1;
#elif (CANFD_MODULE == 1)
    u32CanDiv = ((CLK->CLKDIV5 & CLK_CLKDIV5_CANFD1DIV_Msk) >> CLK_CLKDIV5_CANFD1DIV_Pos) + 1;
#elif (CANFD_MODULE == 2)
    u32CanDiv = ((CLK->CLKDIV5 & CLK_CLKDIV5_CANFD2DIV_Msk) >> CLK_CLKDIV5_CANFD2DIV_Pos) + 1;
#elif (CANFD_MODULE == 3)
    u32CanDiv = ((CLK->CLKDIV5 & CLK_CLKDIV5_CANFD3DIV_Msk) >> CLK_CLKDIV5_CANFD3DIV_Pos) + 1;
#else
    u32CanDiv = ((CLK->CLKDIV5 & CLK_CLKDIV5_CANFD3DIV_Msk) >> CLK_CLKDIV5_CANFD3DIV_Pos) + 1;
#endif
    u32CanClk = u32CanClk / u32CanDiv;
    u8Tq = ((psCanfd->NBTP & CANFD_NBTP_NBRP_Msk) >> CANFD_NBTP_NBRP_Pos) + 1 ;
    u8NtSeg1 = ((psCanfd->NBTP & CANFD_NBTP_NTSEG1_Msk) >> CANFD_NBTP_NTSEG1_Pos);
    u8NtSeg2 = ((psCanfd->NBTP & CANFD_NBTP_NTSEG2_Msk) >> CANFD_NBTP_NTSEG2_Pos);
    u32BitRate = u32CanClk / u8Tq / (u8NtSeg1 + u8NtSeg2 + 3);

    return u32BitRate;
}

/*---------------------------------------------------------------------------*/
/* Get the CAN FD interface Data bit rate Function                           */
/*---------------------------------------------------------------------------*/
uint32_t Get_CANFD_DataBitRate(CANFD_T *psCanfd)
{
    uint32_t u32BitRate = 0;
    uint32_t u32CanClk  = 0;
    uint32_t u32CanDiv  = 0;
    uint8_t  u8Tq = 0;
    uint8_t  u8NtSeg1 = 0;
    uint8_t  u8NtSeg2 = 0;

#if (CANFD_MODULE == 0)
    if(CLK_GetModuleClockSource(CANFD0_MODULE) == (CLK_CLKSEL0_CANFD0SEL_HCLK >> CLK_CLKSEL0_CANFD0SEL_Pos))
#elif (CANFD_MODULE == 1)
    if(CLK_GetModuleClockSource(CANFD1_MODULE) == (CLK_CLKSEL0_CANFD1SEL_HCLK >> CLK_CLKSEL0_CANFD1SEL_Pos))
#elif (CANFD_MODULE == 2)
    if(CLK_GetModuleClockSource(CANFD2_MODULE) == (CLK_CLKSEL0_CANFD2SEL_HCLK >> CLK_CLKSEL0_CANFD2SEL_Pos))
#elif (CANFD_MODULE == 3)
    if(CLK_GetModuleClockSource(CANFD3_MODULE) == (CLK_CLKSEL0_CANFD3SEL_HCLK >> CLK_CLKSEL0_CANFD3SEL_Pos))
#else
    if(CLK_GetModuleClockSource(CANFD3_MODULE) == (CLK_CLKSEL0_CANFD3SEL_HCLK >> CLK_CLKSEL0_CANFD3SEL_Pos))
#endif
        u32CanClk = CLK_GetHCLKFreq();
    else
        u32CanClk = CLK_GetHXTFreq();

#if (CANFD_MODULE == 0)
    u32CanDiv = ((CLK->CLKDIV5 & CLK_CLKDIV5_CANFD0DIV_Msk) >> CLK_CLKDIV5_CANFD0DIV_Pos) + 1;
#elif (CANFD_MODULE == 1)
    u32CanDiv = ((CLK->CLKDIV5 & CLK_CLKDIV5_CANFD1DIV_Msk) >> CLK_CLKDIV5_CANFD1DIV_Pos) + 1;
#elif (CANFD_MODULE == 2)
    u32CanDiv = ((CLK->CLKDIV5 & CLK_CLKDIV5_CANFD2DIV_Msk) >> CLK_CLKDIV5_CANFD2DIV_Pos) + 1;
#elif (CANFD_MODULE == 3)
    u32CanDiv = ((CLK->CLKDIV5 & CLK_CLKDIV5_CANFD3DIV_Msk) >> CLK_CLKDIV5_CANFD3DIV_Pos) + 1;
#else
    u32CanDiv = ((CLK->CLKDIV5 & CLK_CLKDIV5_CANFD3DIV_Msk) >> CLK_CLKDIV5_CANFD3DIV_Pos) + 1;
#endif
    u32CanClk = u32CanClk / u32CanDiv;
    u8Tq = ((psCanfd->DBTP & CANFD_DBTP_DBRP_Msk) >> CANFD_DBTP_DBRP_Pos) + 1 ;
    u8NtSeg1 = ((psCanfd->DBTP & CANFD_DBTP_DTSEG1_Msk) >> CANFD_DBTP_DTSEG1_Pos);
    u8NtSeg2 = ((psCanfd->DBTP & CANFD_DBTP_DTSEG2_Msk) >> CANFD_DBTP_DTSEG2_Pos);
    u32BitRate = u32CanClk / u8Tq / (u8NtSeg1 + u8NtSeg2 + 3);

    return u32BitRate;
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
/* Init CAN FD                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_MonitorMode_Init(uint32_t u32NormBitRate, uint32_t u32DataBitRate)
{
    CANFD_FD_T sCANFD_Config;

    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_FD_MODE);
    sCANFD_Config.sBtConfig.sNormBitRate.u32BitRate = u32NormBitRate;
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = u32DataBitRate;
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

    printf("CAN FD monitoring Nominal baud rate(bps): %d\n", Get_CANFD_NominalBitRate(g_pCanfd));
    printf("CAN FD monitoring Data baud rate(bps): %d\n", Get_CANFD_DataBitRate(g_pCanfd));
    /* Enable the Bus Monitoring Mode */
    g_pCanfd->CCCR |= CANFD_CCCR_MON_Msk;

    /* Non-matching Frames with Extended ID and Standard ID are stored in Rx FIFO0 or Rx FIFO1, reject all remote frames with 11-bit standard IDs and 29-bit extended IDs */
    CANFD_SetGFC(g_pCanfd, eCANFD_ACC_NON_MATCH_FRM_RX_FIFO0, eCANFD_ACC_NON_MATCH_FRM_RX_FIFO1, 1, 1);
    /* Enable RX FIFO New message, Message lost interrupt using interrupt line 0 */
    CANFD_EnableInt(g_pCanfd, (CANFD_IE_RF0NE_Msk | CANFD_IE_RF0LE_Msk | CANFD_IE_RF1NE_Msk | CANFD_IE_RF1LE_Msk), 0, 0, 0);
    /* CAN FD Run to Normal mode */
    CANFD_RunToNormal(g_pCanfd, TRUE);
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

    /*---------------------------------------------------------------------------------------------------------*/
    /*                                                 SAMPLE CODE                                             */
    /*---------------------------------------------------------------------------------------------------------*/
    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+-----------------------------------------------------------------------------+\n");
    printf("|                       CANFD%d Monitor Mode Sample Code                       |\n", ((CANFD_MODULE == 0) ? 0 : (CANFD_MODULE == 1) ? 1 : (CANFD_MODULE == 2) ? 2 : 3));
    printf("+-----------------------------------------------------------------------------+\n");
    printf("|  Description :                                                              |\n");
    printf("|    The sample code needs three boards. Use CAN FD Monitor mode to listen to |\n");
    printf("|    CAN Bus communication test. The sample code must be set up on the node   |\n");
    printf("|    of CAN communication transmission. Users can use one of the sample codes |\n");
    printf("|    ' CANFD_CANFD_TxRx ' or ' CANFD_CANFD_TxRxINT ' as the CAN communication |\n");
    printf("|    transmission network.                                                    |\n");
    printf("|    Note: Users need to confirm whether the transmission rate matches.       |\n");
    printf("+-----------------------------------------------------------------------------+\n\n");

    printf("+-----------------------------------------------------------------------------+\n");
    printf("|                                Pin Configure                                |\n");
    printf("+-----------------------------------------------------------------------------+\n");
    printf("|                                                         CAN_TXD(Any board)  |\n");
    printf("|                                CANBUS                   CAN_RXD(Any board)  |\n");
    printf("|                                  ||    CAN_H   |-----------|                |\n");
    printf("|                                  || <--------->|           |<------         |\n");
    printf("|                                  ||            |    CAN    |CAN_TX          |\n");
    printf("|      CAN_TXD                     ||    CAN_L   |Transceiver|                |\n");
    printf("|      CAN_RXD                     || <--------->|           |------>         |\n");
    printf("|         |-----------|   CAN_H    ||            |           |CAN_RX          |\n");
    printf("|  ------>|           |<---------> ||            |-----------|                |\n");
    printf("|   CAN_TX|   CAN     |            ||                                         |\n");
    printf("|         |Transceiver|            ||                     CAN_TXD(Any board)  |\n");
    printf("|  <------|           |   CAN_L    ||                     CAN_RXD(Any board)  |\n");
    printf("|   CAN_RX|           |<---------> ||    CAN_H   |-----------|                |\n");
    printf("|         |-----------|            || <--------->|           |<------         |\n");
    printf("|                                  ||            |    CAN    |CAN_TX          |\n");
    printf("|                                  ||    CAN_L   |Transceiver|                |\n");
    printf("|                                  || <--------->|           |------>         |\n");
    printf("|                                  ||            |           |CAN_RX          |\n");
    printf("|                                  ||            |-----------|                |\n");
    printf("+-----------------------------------------------------------------------------+\n\n");

    /* CANFD interface initialization */
    CANFD_MonitorMode_Init(1000000, 4000000);

    while(1)
    {
        if(g_u8RxFIFO0RcvOk == 1)
        {
            if(g_u8RxFIFO0MsgIndex > 2)
                g_u8RxFIFO0MsgIndex = 0;

            /* Receive the Rx FIFO0 message(Standard ID) */
            CANFD_ReadRxFifoMsg(g_pCanfd, 0, &g_sRxFIFO0MsgFrame[g_u8RxFIFO0MsgIndex]);
            g_u8RxFIFO0MsgIndex++;
            g_u8RxFIFO0RcvOk = 0;
        }

        if(g_u8RxFIFO0RcvOk == 0 && g_u8RxFIFO0MsgIndex != 0)
        {
            CANFD_ShowMsg(&g_sRxFIFO0MsgFrame[g_u8RxFIFO0MsgIndex - 1]);
            g_u8RxFIFO0MsgIndex--;
        }

        if(g_u8RxFIFO1RcvOk == 1)
        {
            if(g_u8RxFIFO1MsgIndex > 2)
                g_u8RxFIFO1MsgIndex = 0;

            /* Receive the Rx FIFO1 message(Extended ID) */
            CANFD_ReadRxFifoMsg(g_pCanfd, 1, &g_sRxFIFO1MsgFrame[g_u8RxFIFO1MsgIndex]);
            g_u8RxFIFO1MsgIndex++;
            g_u8RxFIFO1RcvOk = 0;
        }

        if(g_u8RxFIFO1RcvOk == 0 && g_u8RxFIFO1MsgIndex != 0)
        {
            CANFD_ShowMsg(&g_sRxFIFO1MsgFrame[g_u8RxFIFO1MsgIndex - 1]);
            g_u8RxFIFO1MsgIndex--;
        }
    }
}
