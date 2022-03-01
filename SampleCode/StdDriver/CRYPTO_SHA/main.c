/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show Crypto IP SHA function
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NuMicro.h"
#include "vector_parser.h"

extern void OpenTestVector(void);
extern int  GetNextPattern(void);


static int32_t  g_i32DigestLength = 0;

static volatile int g_SHA_done;


void CRPT_IRQHandler(void);
int  do_compare(uint8_t *output, uint8_t *expect, int cmp_len);
int32_t RunSHA(void);
void SYS_Init(void);
void DEBUG_PORT_Init(void);


void CRPT_IRQHandler(void)
{
    if(SHA_GET_INT_FLAG(CRPT))
    {
        g_SHA_done = 1;
        SHA_CLR_INT_FLAG(CRPT);
    }
}


int  do_compare(uint8_t *output, uint8_t *expect, int cmp_len)
{
    int   i;

    if(memcmp(expect, output, (size_t)cmp_len))
    {
        printf("\nMismatch!! - %d\n", cmp_len);
        for(i = 0; i < cmp_len; i++)
            printf("0x%02x    0x%02x\n", expect[i], output[i]);
        return -1;
    }
    return 0;
}


int32_t RunSHA(void)
{
    uint32_t  au32OutputDigest[8];
    uint32_t u32TimeOutCnt;

    SHA_Open(CRPT, SHA_MODE_SHA1, SHA_IN_SWAP, 0);

    SHA_SetDMATransfer(CRPT, (uint32_t)&g_au8ShaData[0], (uint32_t)g_i32DataLen / 8);

    printf("Key len= %d bits\n", g_i32DataLen);

    g_SHA_done = 0;
    /* Start SHA calculation */
    SHA_Start(CRPT, CRYPTO_DMA_ONE_SHOT);

    /* Waiting for SHA calcuation done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!g_SHA_done)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for SHA calcuation done time-out!\n");
            return (-1);
        }
    }

    /* Read SHA calculation result */
    SHA_Read(CRPT, au32OutputDigest);

    /* Compare calculation result with golden pattern */
    if(do_compare((uint8_t *)&au32OutputDigest[0], &g_au8ShaDigest[0], g_i32DigestLength) < 0)
    {
        printf("Compare error!\n");
        return (-1);
    }
    return 0;
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


void DEBUG_PORT_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    DEBUG_PORT->LINE = UART_PARITY_NONE | UART_STOP_BIT_1 | UART_WORD_LEN_8;
    DEBUG_PORT->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);

}


/*-----------------------------------------------------------------------------*/
int main(void)
{

    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    DEBUG_PORT_Init();

    printf("+-----------------------------------+\n");
    printf("|       Crypto SHA Sample Demo      |\n");
    printf("+-----------------------------------+\n");

    NVIC_EnableIRQ(CRPT_IRQn);
    SHA_ENABLE_INT(CRPT);

    /* Load test vector data base */
    OpenTestVector();

    while(1)
    {
        /* Get data from test vector to calcualte and
           compre the result with golden pattern */
        if(GetNextPattern() < 0)
            break;

        if(RunSHA() < 0)
            break;
    }

    printf("SHA test done.\n");

    while(1);
}
