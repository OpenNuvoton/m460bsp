/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demo to use the AES in Key Store.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"


#define BUFF_SIZE           1024

static volatile int g_AES_done = 0, g_AESERR_done = 0;


#pragma pack(push)
#pragma pack(4)
static uint8_t g_au8In[] =
{
    0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77,
    0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff
};
#pragma pack(pop)

#pragma pack(push)
#pragma pack(4)
static uint8_t g_au8Out[BUFF_SIZE];
#pragma pack(pop)

#pragma pack(push)
#pragma pack(4)
static uint8_t g_au8Out2[BUFF_SIZE];
#pragma pack(pop)


int AES_Test(CRPT_T *crpt, KS_MEM_Type mem, int32_t keyIdx);
void CRPT_IRQHandler(void);
void DumpBuff(uint8_t *pucBuff, int nBytes);
void SYS_Init(void);
void UART_Init(void);


void CRPT_IRQHandler(void)
{
    CRPT_T *crpt;

    crpt = CRPT;


    if(crpt->INTSTS & CRPT_INTSTS_AESIF_Msk)
    {
        g_AES_done = 1;
        crpt->INTSTS = CRPT_INTSTS_AESIF_Msk;
    }

    if(crpt->INTSTS & CRPT_INTSTS_AESEIF_Msk)
    {
        g_AESERR_done = 1;
        crpt->INTSTS = CRPT_INTSTS_AESEIF_Msk;
    }

    ECC_DriverISR(crpt);
}

void DumpBuff(uint8_t *pucBuff, int nBytes)
{
    int nIdx, i, j;

    nIdx = 0;
    while(nBytes > 0)
    {
        j = nBytes;
        if(j > 16)
        {
            j = 16;
        }
        printf("0x%04X  ", nIdx);
        for(i = 0; i < j; i++)
            printf("%02x ", pucBuff[nIdx + i]);
        for(; i < 16; i++)
            printf("   ");
        printf("  ");
        for(i = 0; i < j; i++)
        {
            if((pucBuff[nIdx + i] >= 0x20) && (pucBuff[nIdx + i] < 127))
                printf("%c", pucBuff[nIdx + i]);
            else
                printf(".");
            nBytes--;
        }


        nIdx += j;
        printf("\n");
    }
    printf("\n");
}

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock */
    CLK_SetCoreClock(200000000);

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable CRYPTO module clock */
    CLK_EnableModuleClock(CRPT_MODULE);

    /* Enable Key Store module clock */
    CLK_EnableModuleClock(KS_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Update System Core Clock */
    SystemCoreClockUpdate();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART module */
    SYS_ResetModule(UART0_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(DEBUG_PORT, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    int32_t i32KeyIdx;

    //Key = 0x7A29E38E063FF08A2F7A7F2A93484D6F
    uint32_t au32Key[4] = {0x93484D6F, 0x2F7A7F2A, 0x063FF08A, 0x7A29E38E};

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    KS_Open();
    //KS_EraseAll(KS_SRAM);

    i32KeyIdx = KS_Write(KS_SRAM, KS_META_128 | KS_META_AES, au32Key);
    if(i32KeyIdx < 0)
    {
        printf("Fail to write key to Key Store!\n");
        printf("KS SRAM remaind size = %d\n", KS_GetRemainSize(KS_SRAM));

        goto lexit;
    }

    printf("KS SRAM remind size: %d\n", KS_GetRemainSize(KS_SRAM));


    printf("The key in key store:\n");
    DumpBuff((uint8_t *)&au32Key[0], 16);

    AES_Test(CRPT, KS_SRAM, i32KeyIdx);

    printf("Done!\n");

lexit:

    while(1) {}
}


int AES_Test(CRPT_T *crpt, KS_MEM_Type mem, int32_t keyIdx)
{
    int32_t len;
    // IV = 0xF8C44B6FBDF96B835547FF45DE1FFC92
    uint32_t au32IV[4] = {0xDE1FFC92, 0x5547FF45, 0xBDF96B83, 0xF8C44B6F };
    uint32_t u32TimeOutCnt;

    (void)mem;

    /* Enable Crypto interrupt */
    NVIC_EnableIRQ(CRPT_IRQn);

    /* Original data */
    printf("The input data:\n");
    DumpBuff(g_au8In, sizeof(g_au8In));

    /*---------------------------------------
     *  AES-128 ECB mode encrypt
     *---------------------------------------*/
    AES_Open(crpt, 0, 1, AES_MODE_CFB, AES_KEY_SIZE_128, AES_IN_OUT_SWAP);

    /* Use key in key store */
    AES_SetKey_KS(crpt, KS_SRAM, keyIdx);

    /* Provide the IV key */
    AES_SetInitVect(crpt, 0, au32IV);

    /* Prepare the source data and output buffer */
    len = sizeof(g_au8In);
    if(len & 0xf)
    {
        printf("Alignment Error!\n");
        printf("For AES CFB mode, Input data must be 16 bytes (128 bits) alignment.\n");
        return -1;
    }

    AES_SetDMATransfer(crpt, 0, (uint32_t)g_au8In, (uint32_t)g_au8Out, (uint32_t)len);

    AES_ENABLE_INT(crpt);
    g_AES_done = 0;
    g_AESERR_done = 0;

    /* Start AES encode */
    AES_Start(crpt, 0, CRYPTO_DMA_ONE_SHOT);
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!g_AES_done)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for AES encode time-out!\n");
            return -1;
        }
    }
    if(g_AESERR_done)
    {
        printf("AES Encode Fail!\n");
        return -1;
    }

    printf("AES encrypt done. The output data:\n");
    DumpBuff(g_au8Out, 16);

    /*---------------------------------------
     *  AES-128 ECB mode decrypt
     *---------------------------------------*/
    AES_Open(crpt, 0, 0, AES_MODE_CFB, AES_KEY_SIZE_128, AES_IN_OUT_SWAP);

    /* Use key in key store */
    AES_SetKey_KS(crpt, KS_SRAM, keyIdx);

    /* Provide the IV key */
    AES_SetInitVect(crpt, 0, au32IV);

    AES_SetDMATransfer(crpt, 0, (uint32_t)g_au8Out, (uint32_t)g_au8Out2, (uint32_t)len);

    AES_ENABLE_INT(crpt);
    g_AES_done = 0;
    g_AESERR_done = 0;

    /* Start AES decode */
    AES_Start(crpt, 0, CRYPTO_DMA_ONE_SHOT);
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!g_AES_done)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for AES decode time-out!\n");
            return -1;
        }
    }

    if(g_AESERR_done)
    {
        printf("AES Decode Fail!\n");
        return -1;
    }

    printf("AES decrypt done. Decode Data:\n");
    DumpBuff(g_au8Out2, len);

    if(memcmp((char *)g_au8In, (char *)g_au8Out2, (uint32_t)len))
    {
        printf("[FAILED]\n");
        return -1;
    }


    return 0;
}
