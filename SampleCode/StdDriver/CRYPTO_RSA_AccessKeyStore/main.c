/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Shows how to use Crypto RSA engine accesses key from key store
 *            to sign and verify signatures.
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define RSA_BIT_LEN      2048

__ALIGNED(16) static char    N[RSA_KBUF_HLEN] = "bad47a84c1782e4dbdd913f2a261fc8b65838412c6e45a2068ed6d7f16e9cdf4462b39119563cafb74b9cbf25cfd544bdae23bff0ebe7f6441042b7e109b9a8afaa056821ef8efaab219d21d6763484785622d918d395a2a31f2ece8385a8131e5ff143314a82e21afd713bae817cc0ee3514d4839007ccb55d68409c97a18ab62fa6f9f89b3f94a2777c47d6136775a56a9a0127f682470bef831fbec4bcd7b5095a7823fd70745d37d1bf72b63c4b1b4a3d0581e74bf9ade93cc46148617553931a79d92e9e488ef47223ee6f6c061884b13c9065b591139de13c1ea2927491ed00fb793cd68f463f5f64baa53916b46c818ab99706557a1c2d50d232577d1";
__ALIGNED(16) static char    E[RSA_KBUF_HLEN] = "00000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000010001";
__ALIGNED(16) static char    d[RSA_KBUF_HLEN] = "40d60f24b61d76783d3bb1dc00b55f96a2a686f59b3750fdb15c40251c370c65cada222673811bc6b305ed7c90ffcb3abdddc8336612ff13b42a75cb7c88fb936291b523d80acce5a0842c724ed85a1393faf3d470bda8083fa84dc5f31499844f0c7c1e93fb1f734a5a29fb31a35c8a0822455f1c850a49e8629714ec6a2657efe75ec1ca6e62f9a3756c9b20b4855bdc9a3ab58c43d8af85b837a7fd15aa1149c119cfe960c05a9d4cea69c9fb6a897145674882bf57241d77c054dc4c94e8349d376296137eb421686159cb878d15d171eda8692834afc871988f203fc822c5dcee7f6c48df663ea3dc755e7dc06aebd41d05f1ca2891e2679783244d068f";
__ALIGNED(16) static char    Msg[RSA_KBUF_HLEN] = "70992c9d95a4908d2a94b3ab9fa1cd643f120e326f9d7808af50cac42c4b0b4eeb7f0d4df303a568fbfb82b0f58300d25357645721bb71861caf81b27a56082c80a146499fb4eab5bde4493f5d00f1a437bbc360dfcd8056fe6be10e608adb30b6c2f7652428b8d32d362945982a46585d2102ef7995a8ba6e8ad8fd16bd7ae8f53c3d7fcfba290b57ce7f8f09c828d6f2d3ce56f131bd9461e5667e5b73edac77f504dac4f202a9570eb4515b2bf516407db831518db8a2083ec701e8fd387c430bb1a72deca5b49d429cf9deb09cc4518dc5f57c089aa2d3420e567e732102c2c92b88a07c69d70917140ab3823c63f312d3f11fa87ba29da3c7224b4fb4bc";

__ALIGNED(16) static RSA_BUF_KS_T s_sRSABuf;
static uint32_t g_au32TmpBuf[RSA_KBUF_HLEN];
static int32_t g_i32PrivateKeyNum, g_i32PublicKeyNum;

static volatile int g_RSA_done;
static volatile int g_RSA_error;

void CRPT_IRQHandler(void);
void RSA_Hex2Reg(char *input, uint32_t *reg);
void SYS_Init(void);
void DEBUG_PORT_Init(void);
void EraseAllFlashKey(void);
int32_t PrepareKeys(void);


void CRPT_IRQHandler(void)
{
    if(PRNG_GET_INT_FLAG(CRPT))
    {
        PRNG_CLR_INT_FLAG(CRPT);
    }
    if(SHA_GET_INT_FLAG(CRPT))
    {
        SHA_CLR_INT_FLAG(CRPT);
    }
    if(RSA_GET_INT_FLAG(CRPT))
    {
        g_RSA_done = 1;
        if(RSA_GET_INT_FLAG(CRPT)&CRPT_INTSTS_RSAEIF_Msk)
        {
            g_RSA_error = 1;
            printf("RSA error flag is set!!\n");
        }
        RSA_CLR_INT_FLAG(CRPT);
    }
}

void RSA_Hex2Reg(char *input, uint32_t *reg)
{
    int       i, si;
    uint32_t  val32;

    si = (int)strlen(input) - 1;
    while(si >= 0)
    {
        val32 = 0;
        for(i = 0; (i < 8) && (si >= 0); i++)
        {
            if(input[si] <= '9')
                val32 |= (uint32_t)((input[si] - '0') << (i * 4));
            else if((input[si] <= 'z') && (input[si] >= 'a'))
                val32 |= (uint32_t)((input[si] - 'a' + 10) << (i * 4));
            else
                val32 |= (uint32_t)((input[si] - 'A' + 10) << (i * 4));
            si--;
        }
        *reg++ = val32;
    }
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

    /* Enable CRPT module clock */
    CLK_EnableModuleClock(CRPT_MODULE);

    /* Enable KS module clock */
    CLK_EnableModuleClock(KS_MODULE);

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

void EraseAllFlashKey(void)
{
    int32_t i32Ret;
    printf("\nPress any key to erase all keys in FLASH of key store.\n");
    getchar();

    /* Erase all keys in FLASH of key store */
    i32Ret = KS_EraseAll(KS_FLASH);
    if(i32Ret < 0)
    {
        printf("\nErase all keys in FLASH of key store is failed!\n");
    }
    printf("\nErase completed.\n");
}

int32_t PrepareKeys(void)
{
    /* Create and read the private key number */
    RSA_Hex2Reg(d, (uint32_t *)&g_au32TmpBuf[0]);
    g_i32PrivateKeyNum = KS_Write(KS_FLASH, KS_META_RSA_EXP | KS_META_2048 | KS_META_READABLE, (uint32_t *)&g_au32TmpBuf[0]);
    if(g_i32PrivateKeyNum == -1)
        return -1;

    /* Create and read the public key number */
    RSA_Hex2Reg(E, (uint32_t *)&g_au32TmpBuf[0]);
    g_i32PublicKeyNum = KS_Write(KS_FLASH, KS_META_RSA_EXP | KS_META_2048 | KS_META_READABLE, (uint32_t *)&g_au32TmpBuf[0]);
    if(g_i32PublicKeyNum == -1)
        return -1;

    printf("\n[The number of created keys in FLASH of key store] \n");
    printf("  Private key : %d\n", g_i32PrivateKeyNum);
    printf("  Public key  : %d\n", g_i32PublicKeyNum);
    printf("\n");

    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    char    OutputResult[RSA_KBUF_HLEN];
    uint32_t u32TimeOutCnt;

    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    DEBUG_PORT_Init();

    printf("\n\n+-------------------------------------------------+\n");
    printf("|   Crypto RSA access key from key store sample   |\n");
    printf("+-------------------------------------------------+\n");

    /* Open key store */
    KS_Open();

    /* Prepare the keys in key store */
    if(PrepareKeys() == -1)
    {
        printf("\nCreate keys is failed!!\n");
        goto lexit;
    }

    NVIC_EnableIRQ(CRPT_IRQn);

    RSA_ENABLE_INT(CRPT);

    g_RSA_done = 0;
    g_RSA_error = 0;

    printf("Private key (N,d) -\n");
    printf("    N = %s\n", N);
    printf("    d = %s\n", d);
    printf("Public key (N,e) -\n");
    printf("    E = %s\n", E);

    /*---------------------------------------
     *  RSA sign
     *---------------------------------------*/

    /* Configure RSA operation mode and key length */
    RSA_Open(CRPT, RSA_MODE_NORMAL, RSA_KEY_SIZE_2048, &s_sRSABuf, sizeof(s_sRSABuf), 1);
    /* Set RSA private key is read from FLASH of key store */
    RSA_SetKey_KS(CRPT, (uint32_t)g_i32PrivateKeyNum, KS_FLASH, 0);
    RSA_SetDMATransfer_KS(CRPT, Msg, N, 0, 0, 0, 0, 0, 0, 0, 0);
    RSA_Start(CRPT);

    /* Waiting for RSA operation done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!g_RSA_done)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for RSA operation done time-out!\n");
            goto lexit;
        }
    }

    /* Check error flag */
    if(g_RSA_error)
    {
        printf("\nRSA has error!!\n");
        goto lexit;
    }

    /* Get RSA output result */
    RSA_Read(CRPT, OutputResult);
    printf("\nRSA sign: %s\n", OutputResult);

    /*---------------------------------------
     *  RSA verify
     *---------------------------------------*/
    g_RSA_done = 0;
    g_RSA_error = 0;

    /* Configure RSA operation mode and key length */
    RSA_Open(CRPT, RSA_MODE_NORMAL, RSA_KEY_SIZE_2048, &s_sRSABuf, sizeof(s_sRSABuf), 1);
    /* Set RSA public key is read from FLASH of key store */
    RSA_SetKey_KS(CRPT, (uint32_t)g_i32PublicKeyNum, KS_FLASH, 0);
    RSA_SetDMATransfer_KS(CRPT, OutputResult, N, 0, 0, 0, 0, 0, 0, 0, 0);
    RSA_Start(CRPT);

    /* Waiting for RSA operation done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!g_RSA_done)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for RSA operation done time-out!\n");
            goto lexit;
        }
    }

    /* Check error flag */
    if(g_RSA_error)
    {
        printf("\nRSA has error!!\n");
        goto lexit;
    }

    /* Get RSA output result */
    RSA_Read(CRPT, OutputResult);
    printf("\nRSA Output: %s\n", OutputResult);

    /* Verify the message */
    if(strcasecmp(OutputResult, Msg) == 0)
        printf("\nRSA signature verify OK.\n");
    else
    {
        printf("\nRSA signature verify failed!!\n");
        goto lexit;
    }
    printf("\nDone.\n");

lexit:

    /* Erase all keys in FLASH of key store */
    EraseAllFlashKey();

    while(1);
}
