/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Transmit and receive CAN FD message through CAN interface.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
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

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void SYS_Init(void);
void CANFD_Init(void);
void CANFD_TxRxTest(void);



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
void CANFD_Init(void)
{
    CANFD_FD_T sCANFD_Config;

    printf("+-------------------------------------------------------------+\n");
    printf("|                        Pin Configure                        |\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|      CAN_TXD                            CAN_TXD(Any board)  |\n");
    printf("|      CAN_RXD                            CAN_RXD(Any board)  |\n");
    printf("|         |-----------|  CANBUS  |-----------|                |\n");
    printf("|  ------>|           |<-------->|           |<------         |\n");
    printf("|   CAN_TX|   CANFD   |  CAN_H   |   CANFD   |CAN_TX          |\n");
    printf("|         |Transceiver|          |Transceiver|                |\n");
    printf("|  <------|           |<-------->|           |------>         |\n");
    printf("|   CAN_RX|           |  CAN_L   |           |CAN_RX          |\n");
    printf("|         |-----------|          |-----------|                |\n");
    printf("+-------------------------------------------------------------+\n\n");

    /* Get the CAN FD configuration value */
    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_FD_MODE);
    sCANFD_Config.sBtConfig.sNormBitRate.u32BitRate = 1000000;
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = 4000000;
    /* Open the CAN FD feature */
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

    /* Receive 0x111 (11-bit id) in CAN FD rx message buffer 0 by setting mask 0 */
    CANFD_SetSIDFltr(g_pCanfd, 0, CANFD_RX_BUFFER_STD(0x111, 0));
    /* Receive 0x22F (11-bit id) in CAN FD rx message buffer 0 by setting mask 1 */
    CANFD_SetSIDFltr(g_pCanfd, 1, CANFD_RX_BUFFER_STD(0x22F, 0));
    /* Receive 0x333 (11-bit id) in CAN FD rx message buffer 0 by setting mask 2 */
    CANFD_SetSIDFltr(g_pCanfd, 2, CANFD_RX_BUFFER_STD(0x333, 0));

    /* Receive 0x222 (29-bit id) in CAN FD rx message buffer 1 by setting mask 3 */
    CANFD_SetXIDFltr(g_pCanfd, 0, CANFD_RX_BUFFER_EXT_LOW(0x222, 1), CANFD_RX_BUFFER_EXT_HIGH(0x222, 1));
    /* Receive 0x3333 (29-bit id) in CAN FD rx message buffer 1 by setting mask 3 */
    CANFD_SetXIDFltr(g_pCanfd, 1, CANFD_RX_BUFFER_EXT_LOW(0x3333, 1), CANFD_RX_BUFFER_EXT_HIGH(0x3333, 1));
    /* Receive 0x44444 (29-bit id) in CAN FD rx message buffer 1 by setting mask 3 */
    CANFD_SetXIDFltr(g_pCanfd, 2, CANFD_RX_BUFFER_EXT_LOW(0x44444, 1), CANFD_RX_BUFFER_EXT_HIGH(0x44444, 1));
    /* CAN FD Run to Normal mode */
    CANFD_RunToNormal(g_pCanfd, TRUE);
}

/*---------------------------------------------------------------------------------------------------------*/
/* Fini CAN FD                                                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_Fini(void)
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
/* CAN FD Function Test                                                                                    */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_TxRxTest(void)
{
    uint8_t u8Item;
    uint8_t u8Cnt;
    uint8_t u8ErrFlag = 0;
    uint8_t u8TxTestNum = 0;
    uint8_t u8RxTestNum = 0;
    uint8_t u8RxTempLen = 0;
    CANFD_FD_MSG_T      sRxMsgFrame;
    CANFD_FD_MSG_T      sTxMsgFrame;

    /* CAN FD interface initialization */
    CANFD_Init();

    printf("+--------------------------------------------------------------------------+\n");
    printf("|                           CAN FD Function Test                           |\n");
    printf("+--------------------------------------------------------------------------+\n");
    printf("|  Description :                                                           |\n");
    printf("|    The sample code needs two boards. One is master(CAN FD transmitter)   |\n");
    printf("|    and the other is slave(CAN FD receiver). Master will send 6 messages  |\n");
    printf("|    with different sizes of data and ID to the slave. Slave will check if |\n");
    printf("|    received data is correct after getting 6 messages data.               |\n");
    printf("|  Please select Master or Slave test                                      |\n");
    printf("|  [0] Master(CAN FD transmitter)    [1] Slave(CAN FD receiver)            |\n");
    printf("+--------------------------------------------------------------------------+\n\n");

    u8Item = getchar();

    if(u8Item == '0')
    {
        /* Send 6 messages with different ID and data size */
        for(u8TxTestNum = 0; u8TxTestNum < 6 ; u8TxTestNum++)
        {
            printf("Start to CAN FD Bus Transmitter :\n");

            /* Set the ID Number */
            if(u8TxTestNum == 0)      sTxMsgFrame.u32Id = 0x111;
            else if(u8TxTestNum == 1) sTxMsgFrame.u32Id = 0x22F;
            else if(u8TxTestNum == 2) sTxMsgFrame.u32Id = 0x333;
            else if(u8TxTestNum == 3) sTxMsgFrame.u32Id = 0x222;
            else if(u8TxTestNum == 4) sTxMsgFrame.u32Id = 0x3333;
            else if(u8TxTestNum == 5) sTxMsgFrame.u32Id = 0x44444;

            /* Set the ID type */
            if(u8TxTestNum < 3)
                sTxMsgFrame.eIdType = eCANFD_SID;
            else
                sTxMsgFrame.eIdType = eCANFD_XID;

            /* Set the frame type */
            sTxMsgFrame.eFrmType = eCANFD_DATA_FRM;
            /* Set CAN FD frame format */
            sTxMsgFrame.bFDFormat = 1;
            /* Set the bitrate switch */
            sTxMsgFrame.bBitRateSwitch = 1;

            /* Set the data length */
            if(u8TxTestNum == 0  ||  u8TxTestNum == 3)     sTxMsgFrame.u32DLC = 16;
            else if(u8TxTestNum == 1 || u8TxTestNum == 4)  sTxMsgFrame.u32DLC = 32;
            else if(u8TxTestNum == 2 || u8TxTestNum == 5)  sTxMsgFrame.u32DLC = 64;

            if(u8TxTestNum < 3)
                printf("Send to transmit message 0x%08x (11-bit)\n", sTxMsgFrame.u32Id);
            else
                printf("Send to transmit message 0x%08x (29-bit)\n", sTxMsgFrame.u32Id);

            printf("Data Message : ");

            for(u8Cnt = 0; u8Cnt < sTxMsgFrame.u32DLC; u8Cnt++)
            {
                sTxMsgFrame.au8Data[u8Cnt] = u8Cnt + u8TxTestNum;
                printf("%02d,", sTxMsgFrame.au8Data[u8Cnt]);
            }

            printf("\n\n");

            /* Use message buffer 0 */
            if(CANFD_TransmitTxMsg(g_pCanfd, 0, &sTxMsgFrame) != eCANFD_TRANSMIT_SUCCESS)
            {
                printf("Failed to transmit message\n");
            }

        }

        printf("\n Transmit Done\n");
    }
    else
    {
        printf("Start to CAN FD Bus Receiver :\n");

        /* Receive 6 messages with different ID and data size */
        do
        {
            /* Check for any received messages on CAN FD message buffer 0 */
            if(CANFD_ReadRxBufMsg(g_pCanfd, 0, &sRxMsgFrame) == eCANFD_RECEIVE_SUCCESS)
            {
                printf("Rx buf 0: Received message 0x%08X (11-bit)\r\n", sRxMsgFrame.u32Id);
                printf("Message Data : ");

                for(u8Cnt = 0; u8Cnt < sRxMsgFrame.u32DLC; u8Cnt++)
                {
                    printf("%02d ,", sRxMsgFrame.au8Data[u8Cnt]);

                    if(sRxMsgFrame.au8Data[u8Cnt] != u8Cnt + u8RxTestNum)
                    {
                        u8ErrFlag = 1;
                    }
                }

                printf(" \n\n");

                /* Check Standard ID number */
                if((sRxMsgFrame.u32Id != 0x111) && (sRxMsgFrame.u32Id != 0x22F) && (sRxMsgFrame.u32Id != 0x333))
                {
                    u8ErrFlag = 1;
                }

                if(u8RxTestNum == 0)      u8RxTempLen = 16;
                else if(u8RxTestNum == 1) u8RxTempLen = 32;
                else if(u8RxTestNum == 2) u8RxTempLen = 64;

                /* Check Data length */
                if((u8RxTempLen != sRxMsgFrame.u32DLC) || (sRxMsgFrame.eIdType != eCANFD_SID))
                {
                    u8ErrFlag = 1;
                }

                if(u8ErrFlag == 1)
                {
                    printf("CAN FD STD ID or Data Error \n");
                    getchar();
                }

                u8RxTestNum++;
            }

            /* Check for any received messages on CAN FD message buffer 1 */
            if(CANFD_ReadRxBufMsg(g_pCanfd, 1, &sRxMsgFrame) == eCANFD_RECEIVE_SUCCESS)
            {

                printf("Rx buf 1: Received message 0x%08X (29-bit)\r\n", sRxMsgFrame.u32Id);
                printf("Message Data : ");

                for(u8Cnt = 0; u8Cnt < sRxMsgFrame.u32DLC; u8Cnt++)
                {
                    printf("%02d ,", sRxMsgFrame.au8Data[u8Cnt]);

                    if(sRxMsgFrame.au8Data[u8Cnt] != u8Cnt + u8RxTestNum)
                    {
                        u8ErrFlag = 1;
                    }
                }

                printf(" \n\n");

                /* Check Extend ID number */
                if((sRxMsgFrame.u32Id  != 0x222) && (sRxMsgFrame.u32Id  != 0x3333) && (sRxMsgFrame.u32Id != 0x44444))
                {
                    u8ErrFlag = 1;
                }

                if(u8RxTestNum == 3)      u8RxTempLen = 16;
                else if(u8RxTestNum == 4) u8RxTempLen = 32;
                else if(u8RxTestNum == 5) u8RxTempLen = 64;

                /* Check Data length */
                if((u8RxTempLen != sRxMsgFrame.u32DLC) || (sRxMsgFrame.eIdType != eCANFD_XID))
                {
                    u8ErrFlag = 1;
                }

                if(u8ErrFlag == 1)
                {
                    printf("CAN FD EXD ID or Data Error \n");
                    getchar();
                }

                u8RxTestNum++;

            }
        }
        while(u8RxTestNum < 6);

        printf("\n Receive OK & Check OK\n");
    }

    /* CAN FD interface finalization */
    CANFD_Fini();
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
int main()
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
    printf("+----------------------------------------+\n");
    printf("|    CANFD%d FD mode transmission test    |\n", ((CANFD_MODULE == 0) ? 0 : (CANFD_MODULE == 1) ? 1 : (CANFD_MODULE == 2) ? 2 : 3));
    printf("+----------------------------------------+\n");

    /* CANFD sample function */
    CANFD_TxRxTest();

    while(1) {}
}
