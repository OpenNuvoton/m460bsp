/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * $Revision: 3 $
 * $Date: 19/11/22 2:06p $
 * @brief    Show how mbedTLS AES function works.
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "common.h"
#include "mbedtls/aes.h"
//*** <<< Use Configuration Wizard in Context Menu >>> ***
// <c0> Enable AES Test
#define TEST_AES
// </c>
// <c0> Enable CCM Test
#define TEST_CCM
// </c>
// <c0> Enable GCM Test
#define TEST_GCM
// </c>


//*** <<< end of configuration section >>>    ***



#define MBEDTLS_EXIT_SUCCESS    0
#define MBEDTLS_EXIT_FAILURE    -1

extern int mbedtls_aes_self_test(int verbose);
extern int mbedtls_gcm_self_test(int verbose);
extern int mbedtls_ccm_self_test(int verbose);
void SYS_Init(void);


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

    printf("MBEDTLS AES self test ...\n");

#ifdef MBEDTLS_AES_ALT
    printf("Hardware Accellerator Enabled.\n");
#else
    printf("Pure software crypto running.\n");
#endif

#ifdef TEST_AES
    g_u32Ticks = 0;
    i32Ret = mbedtls_aes_self_test(1);
    printf("Total elapsed time is %d ms\n", g_u32Ticks);
#endif

#ifdef TEST_GCM
    g_u32Ticks = 0;
    i32Ret |= mbedtls_gcm_self_test(1);
    printf("Total elapsed time is %d ms\n", g_u32Ticks);
#endif

#ifdef TEST_CCM
    g_u32Ticks = 0;
    i32Ret |= mbedtls_ccm_self_test(1);
    printf("Total elapsed time is %d ms\n", g_u32Ticks);
#endif

    if(i32Ret < 0)
    {
        printf("Test fail!\n");
    }
    printf("Test Done!\n");
    while(1);

}
