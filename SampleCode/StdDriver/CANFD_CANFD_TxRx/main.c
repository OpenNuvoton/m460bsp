/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Transmit and receive CAN FD message through CAN interface.
 *
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include "stdio.h"
#include "string.h"
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void);
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

    /* Select CAN FD0 clock source is HCLK */
    CLK_SetModuleClock(CANFD0_MODULE, CLK_CLKSEL0_CANFD0SEL_HCLK, CLK_CLKDIV5_CANFD0(1));

    /* Enable CAN FD0 peripheral clock */
    CLK_EnableModuleClock(CANFD0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set PC multi-function pins for CAN FD0 RXD and TXD */
    SET_CAN0_RXD_PC4();
    SET_CAN0_TXD_PC5();

}


/*---------------------------------------------------------------------------------------------------------*/
/*                                    Init CAN FD0                                                          */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD_Init(void)
{
    CANFD_FD_T sCANFD_Config;

    printf("+---------------------------------------------------------------+\n");
    printf("|     Pin Configure                                             |\n");
    printf("+---------------------------------------------------------------+\n");
    printf("|  CAN0_TXD(PC5)                         CAN_TXD(Any board)     |\n");
    printf("|  CAN0_RXD(PC4)                         CAN_RXD(Any board      |\n");
    printf("|          |-----------| CANBUS  |-----------|                  |\n");
    printf("|  ------> |           |<------->|           |<------           |\n");
    printf("|   CAN0_TX|   CANFD   |  CAN_H  |   CANFD   |CAN_TX            |\n");
    printf("|          |Transceiver|         |Transceiver|                  |\n");
    printf("|  <------ |           |<------->|           |------>           |\n");
    printf("|   CAN0_RX|           |  CAN_L  |           |CAN_RX            |\n");
    printf("|          |-----------|         |-----------|                  |\n");
    printf("|                                                               |\n");
    printf("+---------------------------------------------------------------+\n\n");

    /*Get the CAN FD0 configuration value*/
    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_FD_MODE);
    sCANFD_Config.sBtConfig.sNormBitRate.u32BitRate = 1000000;
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = 4000000;
    /*Open the CAN FD0 feature*/
    CANFD_Open(CANFD0, &sCANFD_Config);

    /* receive 0x110 in CAN FD0 rx message buffer 0 by setting mask 0 */
    CANFD_SetSIDFltr(CANFD0, 0, CANFD_RX_BUFFER_STD(0x111, 0));
    /* receive 0x220 in CAN FD0 rx message buffer 0 by setting mask 1 */
    CANFD_SetSIDFltr(CANFD0, 1, CANFD_RX_BUFFER_STD(0x22F, 0));
    /* receive 0x333 in CAN FD0 rx message buffer 0 by setting mask 2 */
    CANFD_SetSIDFltr(CANFD0, 2, CANFD_RX_BUFFER_STD(0x333, 0));

    /* receive 0x222 (29-bit id) in CAN FD0 rx message buffer 1 by setting mask 3 */
    CANFD_SetXIDFltr(CANFD0, 0, CANFD_RX_BUFFER_EXT_LOW(0x222, 1), CANFD_RX_BUFFER_EXT_HIGH(0x222, 1));
    /* receive 0x3333 (29-bit id) in CAN FD0 rx message buffer 1 by setting mask 3 */
    CANFD_SetXIDFltr(CANFD0, 1, CANFD_RX_BUFFER_EXT_LOW(0x3333, 1), CANFD_RX_BUFFER_EXT_HIGH(0x3333, 1));
    /* receive 0x44444 (29-bit id) in CAN FD0 rx message buffer 1 by setting mask 3 */
    CANFD_SetXIDFltr(CANFD0, 2, CANFD_RX_BUFFER_EXT_LOW(0x44444, 1), CANFD_RX_BUFFER_EXT_HIGH(0x44444, 1));
    /* CAN FD0 Run to Normal mode  */
    CANFD_RunToNormal(CANFD0, TRUE);
}


/*---------------------------------------------------------------------------------------------------------*/
/*  CAN FD Function Test                                                                                    */
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

    /* CAN FD interface initialization*/
    CANFD_Init();

    printf("+--------------------------------------------------------------------------+\n");
    printf("|                       CAN FD Function Test                               |\n");
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

    if (u8Item == '0')
    {
        /* Send 6 messages with different ID and data size */
        for (u8TxTestNum = 0; u8TxTestNum < 6 ; u8TxTestNum++)
        {
            printf("Start to CAN FD Bus Transmitter :\n");

            /* Set the ID Number */
            if (u8TxTestNum == 0)      sTxMsgFrame.u32Id = 0x111;
            else if (u8TxTestNum == 1) sTxMsgFrame.u32Id = 0x22F;
            else if (u8TxTestNum == 2) sTxMsgFrame.u32Id = 0x333;
            else if (u8TxTestNum == 3) sTxMsgFrame.u32Id = 0x222;
            else if (u8TxTestNum == 4) sTxMsgFrame.u32Id = 0x3333;
            else if (u8TxTestNum == 5) sTxMsgFrame.u32Id = 0x44444;

            /*Set the ID type*/
            if (u8TxTestNum < 3)
                sTxMsgFrame.eIdType = eCANFD_SID;
            else
                sTxMsgFrame.eIdType = eCANFD_XID;

            /*Set the frame type*/
            sTxMsgFrame.eFrmType = eCANFD_DATA_FRM;
            /*Set CAN FD frame format */
            sTxMsgFrame.bFDFormat = 1;
            /*Set the bitrate switch */
            sTxMsgFrame.bBitRateSwitch = 1;

            /*Set the data lenght */
            if (u8TxTestNum == 0  ||  u8TxTestNum == 3)     sTxMsgFrame.u32DLC = 16;
            else if (u8TxTestNum == 1 || u8TxTestNum == 4)  sTxMsgFrame.u32DLC = 32;
            else if (u8TxTestNum == 2 || u8TxTestNum == 5)  sTxMsgFrame.u32DLC = 64;

            if (u8TxTestNum < 3)
                printf("Send to transmit message 0x%08x (11-bit)\n", sTxMsgFrame.u32Id);
            else
                printf("Send to transmit message 0x%08x (29-bit)\n", sTxMsgFrame.u32Id);

            printf("Data Message : ");

            for (u8Cnt = 0; u8Cnt < sTxMsgFrame.u32DLC; u8Cnt++)
            {
                sTxMsgFrame.au8Data[u8Cnt] = u8Cnt + u8TxTestNum;
                printf("%02d,", sTxMsgFrame.au8Data[u8Cnt]);
            }

            printf("\n\n");

            /* use message buffer 0 */
            if (CANFD_TransmitTxMsg(CANFD0, 0, &sTxMsgFrame) != 1)
            {
                printf("Failed to transmit message\n");
            }

        }

        printf("\n Transmit Done\n");
    }
    else
    {
        printf("Start to CAN FD Bus Receiver :\n");

        /* Receive  6 messages with different ID and data size */
        do
        {
            /* check for any received messages on CAN FD0 message buffer 0 */
            if (CANFD_ReadRxBufMsg(CANFD0, 0, &sRxMsgFrame) == 1)
            {
                printf("Rx buf 0: Received message 0x%08X\n", sRxMsgFrame.u32Id);
                printf("Message Data : ");

                for (u8Cnt = 0; u8Cnt < sRxMsgFrame.u32DLC; u8Cnt++)
                {
                    printf("%02d ,", sRxMsgFrame.au8Data[u8Cnt]);

                    if (sRxMsgFrame.au8Data[u8Cnt] != u8Cnt + u8RxTestNum)
                    {
                        u8ErrFlag = 1;
                    }
                }

                printf(" \n\n");

                /* Check Standard ID number */
                if ((sRxMsgFrame.u32Id != 0x111) && (sRxMsgFrame.u32Id != 0x22F) && (sRxMsgFrame.u32Id != 0x333))
                {
                    u8ErrFlag = 1;
                }

                if (u8RxTestNum == 0)      u8RxTempLen = 16;
                else if (u8RxTestNum == 1) u8RxTempLen = 32;
                else if (u8RxTestNum == 2) u8RxTempLen = 64;

                /* Check Data lenght */
                if ((u8RxTempLen != sRxMsgFrame.u32DLC) || (sRxMsgFrame.eIdType != eCANFD_SID))
                {
                    u8ErrFlag = 1;
                }

                if (u8ErrFlag == 1)
                {
                    printf("CAN FD STD ID or Data Error \n");
                    getchar();
                }

                u8RxTestNum++;
            }

            /* check for any received messages on CAN FD0 message buffer 1 */
            if (CANFD_ReadRxBufMsg(CANFD0, 1, &sRxMsgFrame) == 1)
            {

                printf("Rx buf 1: Received message 0x%08X (29-bit)\r\n", sRxMsgFrame.u32Id);
                printf("Message Data : ");

                for (u8Cnt = 0; u8Cnt < sRxMsgFrame.u32DLC; u8Cnt++)
                {
                    printf("%02d ,", sRxMsgFrame.au8Data[u8Cnt]);

                    if (sRxMsgFrame.au8Data[u8Cnt] != u8Cnt + u8RxTestNum)
                    {
                        u8ErrFlag = 1;
                    }
                }

                printf(" \n\n");

                /* Check Extend ID number */
                if ((sRxMsgFrame.u32Id  != 0x222) && (sRxMsgFrame.u32Id  != 0x3333) && (sRxMsgFrame.u32Id != 0x44444))
                {
                    u8ErrFlag = 1;
                }

                if (u8RxTestNum == 3)      u8RxTempLen = 16;
                else if (u8RxTestNum == 4) u8RxTempLen = 32;
                else if (u8RxTestNum == 5) u8RxTempLen = 64;

                /* Check Data lenght */
                if ((u8RxTempLen != sRxMsgFrame.u32DLC) || (sRxMsgFrame.eIdType != eCANFD_XID))
                {
                    u8ErrFlag = 1;
                }

                if (u8ErrFlag == 1)
                {
                    printf("CAN FD EXD ID or Data Error \n");
                    getchar();
                }

                u8RxTestNum++;

            }
        } while (u8RxTestNum < 6);

        printf("\n Receive OK & Check OK\n");
    }
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

    /*---------------------------------------------------------------------------------------------------------*/
    /*                                                 SAMPLE CODE                                             */
    /*---------------------------------------------------------------------------------------------------------*/
    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+----------------------------------------+\n");
    printf("|      CAN FD mode transmission test     |\n");
    printf("+----------------------------------------+\n");

    /* CANFD sample function */
    CANFD_TxRxTest();

    while (1) {}
}
