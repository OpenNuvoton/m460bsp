/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 19/11/22 2:06p $
 * @brief    Show how mbedTLS SHA256 function works.
 * @note
 * SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "mbedtls/rsa.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "NuMicro.h"

#define MBEDTLS_EXIT_SUCCESS    0
#define MBEDTLS_EXIT_FAILURE    -1

extern int mbedtls_sha256_self_test(int verbose);

volatile uint32_t g_u32Ticks = 0;


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

    /* Set UART Clock */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

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

void SysTick_Handler()
{
    g_u32Ticks++;
}


int main(void)
{
    int  i32Ret = MBEDTLS_EXIT_SUCCESS;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Init debug message */
    UART0_Init();

    SysTick_Config(SystemCoreClock / 1000);

    printf("MBEDTLS SHA256 self test ...\n");

#ifdef MBEDTLS_SHA256_ALT
    printf("Hardware Accellerator Enabled.\n");
#else
    printf("Pure software crypto running.\n");
#endif

    g_u32Ticks = 0;
    i32Ret = mbedtls_sha256_self_test(1);
    printf("Total elapsed time is %d ms\n", g_u32Ticks);

    if(i32Ret < 0)
    {
        printf("Test fail!\n");
    }
    printf("Test Done!\n");
    while(1);

}


void show(void)
{
    int i, j;
    int n = 128;
    uint8_t *pu8;

    printf("\n");
    for(i = 0; i < 3; i++)
    {
        printf("SADDR[%d]", i);
        pu8 = (uint8_t *)CRPT->RSA_SADDR[i];
        for(j = 0; j < n; j++)
        {
            if((j & 0xf) == 0)
                printf("\n");
            printf("%02x ", pu8[j]);
        }
        printf("\n");
    }

    printf("DADDR");
    pu8 = (uint8_t *)CRPT->RSA_DADDR;
    for(j = 0; j < n; j++)
    {
        if((j & 0xf) == 0)
            printf("\n");
        printf("%02x ", pu8[j]);
    }
    printf("\n");

}

void dump(uint8_t *p, uint32_t size)
{
    int i;

    for(i = 0; i < size; i++)
    {
        if((i & 0xf) == 0)
            printf("\n");
        printf("%02x ", p[i]);
    }
    printf("\n");


}



int mbedtls_platform_entropy_poll(void *data,
                                  unsigned char *output, size_t len, size_t *olen)
{
    return 0;
}
