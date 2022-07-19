/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demo to use ECC ECDH with Key Store.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

void CRPT_IRQHandler(void);
void DumpBuff(uint8_t *pucBuff, int nBytes);
void SYS_Init(void);
void UART_Init(void);


void CRPT_IRQHandler(void)
{
    CRPT_T *crpt;

    crpt = CRPT;


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
    int32_t i;
    int32_t err;
    int32_t i32KeyIdx_d, i32ShareKeyIdx;
    uint32_t time;
    uint32_t au32ECC_N[18] = {0};

    /* Public Key of B sdie. Gen by private key = 74C57C8F23BE25F0EF591CEF81E89D1DE08CFDD7E3CADC8670A757D07A961DF0 */
    char Qx[] = "43A5FC3C3347670FADC59972E0DE55E3FBC9055DA48F49DBA1A29E3CC602A2F6";
    char Qy[] = "6232E772198811E1EB541640B872DC137ABF1D01FB8F3AE60F3432D26091772F";

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    printf("CPU @ %dHz\n", SystemCoreClock);

    /* Init Key Store */
    KS_Open();

    /* Enable CRPT interrupt */
    NVIC_EnableIRQ(CRPT_IRQn);
    ECC_ENABLE_INT(CRPT);
    SHA_ENABLE_INT(CRPT);

    for(i = 0; i < 18; i++)
        au32ECC_N[i] = 0;
    au32ECC_N[7] = 0xFFFFFFFFul;
    au32ECC_N[6] = 0x00000001ul;
    au32ECC_N[5] = 0x00000000ul;
    au32ECC_N[4] = 0x00000000ul;
    au32ECC_N[3] = 0x00000000ul;
    au32ECC_N[2] = 0xFFFFFFFFul;
    au32ECC_N[1] = 0xFFFFFFFFul;
    au32ECC_N[0] = 0xFFFFFFFFul;

    /* We need to prepare N first for RNG to generate a private key */
    err = RNG_ECDH_Init(PRNG_KEY_SIZE_256, au32ECC_N);
    if(err)
    {
        printf("PRNG ECDH Inital failed\n");
        goto lexit;
    }


    /* Reset SysTick to measure time */
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    /*------------------------------------------------------------------------------*/
    /* Generate private key to Key Store */
    /* NOTE: The private key must be generated form RNG hardware when using ECDH with key store */
    i32KeyIdx_d = RNG_ECDH(PRNG_KEY_SIZE_256);
    if(i32KeyIdx_d < 0)
    {
        printf("[FAILED]\n");
        printf("Fail to write k to KS SRAM\n");
        goto lexit;
    }

    /*-----------------------------------------------------------------------------------------------*/
    /* Calcualte Share Key by private key A and publick key B */
    SysTick->VAL = 0;
    if((i32ShareKeyIdx = ECC_GenerateSecretZ_KS(CRPT, CURVE_P_256, KS_SRAM, i32KeyIdx_d, Qx, Qy)) < 0)
    {
        printf("ECC ECDH share key calculation fail\n");
        goto lexit;
    }
    printf("Share Key Idx for A Side = %d, remain size = %d\n", i32ShareKeyIdx, KS_GetRemainSize(KS_SRAM));

    time = 0xffffff - SysTick->VAL;

    printf("Elapsed time: %d.%d ms\n", time / CyclesPerUs / 1000, time / CyclesPerUs % 1000);

    printf("Done!\n");

lexit:

    while(1) {}
}
