/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Shows how to use Crypto RSA engine CRT/CRT bypass mode accesses
 *           key from key store to sign and verify signatures.
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define RSA_BIT_LEN      2048

__ALIGNED(16) static char    N[RSA_KBUF_HLEN] = "bad47a84c1782e4dbdd913f2a261fc8b65838412c6e45a2068ed6d7f16e9cdf4462b39119563cafb74b9cbf25cfd544bdae23bff0ebe7f6441042b7e109b9a8afaa056821ef8efaab219d21d6763484785622d918d395a2a31f2ece8385a8131e5ff143314a82e21afd713bae817cc0ee3514d4839007ccb55d68409c97a18ab62fa6f9f89b3f94a2777c47d6136775a56a9a0127f682470bef831fbec4bcd7b5095a7823fd70745d37d1bf72b63c4b1b4a3d0581e74bf9ade93cc46148617553931a79d92e9e488ef47223ee6f6c061884b13c9065b591139de13c1ea2927491ed00fb793cd68f463f5f64baa53916b46c818ab99706557a1c2d50d232577d1";
__ALIGNED(16) static char    d[RSA_KBUF_HLEN] = "40d60f24b61d76783d3bb1dc00b55f96a2a686f59b3750fdb15c40251c370c65cada222673811bc6b305ed7c90ffcb3abdddc8336612ff13b42a75cb7c88fb936291b523d80acce5a0842c724ed85a1393faf3d470bda8083fa84dc5f31499844f0c7c1e93fb1f734a5a29fb31a35c8a0822455f1c850a49e8629714ec6a2657efe75ec1ca6e62f9a3756c9b20b4855bdc9a3ab58c43d8af85b837a7fd15aa1149c119cfe960c05a9d4cea69c9fb6a897145674882bf57241d77c054dc4c94e8349d376296137eb421686159cb878d15d171eda8692834afc871988f203fc822c5dcee7f6c48df663ea3dc755e7dc06aebd41d05f1ca2891e2679783244d068f";
__ALIGNED(16) static char    P[RSA_KBUF_HLEN] = "e7c9e4b3e5d7ac9e83be08328105356dfeefe222f26c95378effd2150fadf7ba23f5b4705d82e4f1bc45057067c7def73e2100f756ee6d547965fa4f24b85d68867f03d7c886d1dbcca4c589745701b362a1f1417f471d8475b6b7a16a4c48ef1f556edc3f0ff6ba13d365d6e82751f207d91101c8eea1013ccdd9e1de4c387f";
__ALIGNED(16) static char    Q[RSA_KBUF_HLEN] = "ce58602e051f0f4647c4ec57f682e5737fc482a8a1ffac9043bba4fba3387d7dd2154507af1e28bd81b61fcdfe35f9734e0d9b53682ec785f1f6e6224f63d10bf78484b83a4254f333d0fb3f3e9e1834bede52e3078ac279a862fb90af266d7591c81f20b718d07d51bfc221b66a25403b4ac1a68d673fdd959b01ecf3d0a7af";
__ALIGNED(16) static char    Msg[RSA_KBUF_HLEN] = "70992c9d95a4908d2a94b3ab9fa1cd643f120e326f9d7808af50cac42c4b0b4eeb7f0d4df303a568fbfb82b0f58300d25357645721bb71861caf81b27a56082c80a146499fb4eab5bde4493f5d00f1a437bbc360dfcd8056fe6be10e608adb30b6c2f7652428b8d32d362945982a46585d2102ef7995a8ba6e8ad8fd16bd7ae8f53c3d7fcfba290b57ce7f8f09c828d6f2d3ce56f131bd9461e5667e5b73edac77f504dac4f202a9570eb4515b2bf516407db831518db8a2083ec701e8fd387c430bb1a72deca5b49d429cf9deb09cc4518dc5f57c089aa2d3420e567e732102c2c92b88a07c69d70917140ab3823c63f312d3f11fa87ba29da3c7224b4fb4bc";
__ALIGNED(16) static char    Sign[RSA_KBUF_HLEN] = "7e65b998a05f626b028c75dc3fbf98963dce66d0f4c3ae4237cff304d84d8836cb6bad9ac86f9d1b8a28dd70404788b869d2429f1ec0663e51b753f7451c6b4645d99126e457c1dac49551d86a8a974a3131e9b371d5c214cc9ff240c299bd0e62dbc7a9a2dad9fa5404adb00632d36332d5be6106e9e6ec81cac45cd339cc87abbe7f89430800e16e032a66210b25e926eda243d9f09955496ddbc77ef74f17fee41c4435e78b46965b713d72ce8a31af641538add387fedfd88bb22a42eb3bda40f72ecad941dbffdd47b3e77737da741553a45b630d070bcc5205804bf80ee2d51612875dbc4796960052f1687e0074007e6a33ab8b2085c033f9892b6f74";
__ALIGNED(16) static char    Msg2[RSA_KBUF_HLEN] = "2b7ce910993b660fccfb6bf22ec4e4251fa39470bea2e0f747209b7e844a84b81bf09f89917d2a73fcb4e9a5a429f5891234d3f1374acf20c584d44336a6dee61891d5ead50035d3df8df03f319d57a77eff90459def4dbdc7b04f2bb94996c0c1c87990d43df3fa6e7480a9588c863670936e0152cd2461bfc10f7fb203299bbd8d91ec79f3ad56304b89af70a784925e07bbf0ffe5629beb0438bc6ca65629f8d6c0b671ab4a0e9345e843322c591dddbbe8ee8f7d72a47fabe53d40286dcfaefc05d8954e43573f17421afb5c849e5fbaff312d64bb81794bc9fce3a809dcfef87696b7993d3c39a66d27f01c6cd32e043c7c2dd07487c0c4001b49bb2555";
__ALIGNED(16) static char    Sign2[RSA_KBUF_HLEN] = "11111111a05f626b028c75dc3fbf98963dce66d0f4c3ae4237cff304d84d8836cb6bad9ac86f9d1b8a28dd70404788b869d2429f1ec0663e51b753f7451c6b4645d99126e457c1dac49551d86a8a974a3131e9b371d5c214cc9ff240c299bd0e62dbc7a9a2dad9fa5404adb00632d36332d5be6106e9e6ec81cac45cd339cc87abbe7f89430800e16e032a66210b25e926eda243d9f09955496ddbc77ef74f17fee41c4435e78b46965b713d72ce8a31af641538add387fedfd88bb22a42eb3bda40f72ecad941dbffdd47b3e77737da741553a45b630d070bcc5205804bf80ee2d51612875dbc4796960052f1687e0074007e6a33ab8b2085c033f922222222";

__ALIGNED(16) static RSA_BUF_KS_T s_sRSABuf;
static uint32_t g_au32TmpBuf[RSA_KBUF_HLEN];

static int32_t g_i32PrivateKeyNum = 0, g_i32PNum = 0, g_i32QNum = 0, g_i32CpNum = 0, g_i32CqNum = 0;
static int32_t g_i32DpNum = 0, g_i32DqNum = 0, g_i32RpNum = 0, g_i32RqNum = 0;

static volatile int g_RSA_done;
static volatile int g_RSA_error;

void CRPT_IRQHandler(void);
void RSA_Hex2Reg(char *input, uint32_t *reg);
void SYS_Init(void);
void DEBUG_PORT_Init(void);
void EraseAllSramKey(void);
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

void EraseAllSramKey(void)
{
    int32_t i32Ret;
    printf("\nPress any key to erase all keys in SRAM of key store.\n");
    getchar();

    /* Erase all keys in SRAM of key store */
    i32Ret = KS_EraseAll(KS_SRAM);
    if(i32Ret == -1)
    {
        printf("\nErase all keys in SRAM of key store is failed!\n");
    }
    printf("\nErase completed.\n");
}

int32_t PrepareKeys(void)
{
    /* Create and read the private key number */
    RSA_Hex2Reg(d, (uint32_t *)&g_au32TmpBuf[0]);
    g_i32PrivateKeyNum = KS_Write(KS_SRAM, KS_META_RSA_EXP | KS_META_2048 | KS_META_READABLE, (uint32_t *)&g_au32TmpBuf[0]);
    if(g_i32PrivateKeyNum == -1)
        return -1;

    /* Create and read the P key number. P is equal to half of private/public key length. */
    RSA_Hex2Reg(P, (uint32_t *)&g_au32TmpBuf[0]);
    g_i32PNum = KS_Write(KS_SRAM, KS_META_RSA_EXP | KS_META_1024 | KS_META_READABLE, (uint32_t *)&g_au32TmpBuf[0]);
    if(g_i32PNum == -1)
        return -1;

    /* Create and read the Q key number. Q is equal to half of private/public key length. */
    RSA_Hex2Reg(Q, (uint32_t *)&g_au32TmpBuf[0]);
    g_i32QNum = KS_Write(KS_SRAM, KS_META_RSA_EXP | KS_META_1024 | KS_META_READABLE, (uint32_t *)&g_au32TmpBuf[0]);
    if(g_i32QNum == -1)
        return -1;

    /* Create and read the Cp key number. Cp is equal to private/public key length. */
    /* The Cp key is for Temporary use, it can be any value. */
    g_i32CpNum = KS_Write(KS_SRAM, KS_META_RSA_EXP | KS_META_2048 | KS_META_READABLE, (uint32_t *)&g_au32TmpBuf[0]);
    if(g_i32CpNum == -1)
        return -1;

    /* Create and read the Cq key number. Cq is equal to private/public key length. */
    /* The Cq key is for Temporary use, it can be any value. */
    g_i32CqNum = KS_Write(KS_SRAM, KS_META_RSA_EXP | KS_META_2048 | KS_META_READABLE, (uint32_t *)&g_au32TmpBuf[0]);
    if(g_i32CqNum == -1)
        return -1;

    /* Create and read the Dp key number. Dp is equal to half of private/public key length. */
    /* The Dp key is for Temporary use, it can be any value. */
    g_i32DpNum = KS_Write(KS_SRAM, KS_META_RSA_EXP | KS_META_1024 | KS_META_READABLE, (uint32_t *)&g_au32TmpBuf[0]);
    if(g_i32DpNum == -1)
        return -1;

    /* Create and read the Dq key number. Dq is equal to half of private/public key length. */
    /* The Rq key is for Temporary use, it can be any value. */
    g_i32DqNum = KS_Write(KS_SRAM, KS_META_RSA_EXP | KS_META_1024 | KS_META_READABLE, (uint32_t *)&g_au32TmpBuf[0]);
    if(g_i32DqNum == -1)
        return -1;

    /* Create and read the Rp key number. Rp is equal to private/public key length. */
    /* The Rp key is for Temporary use, it can be any value. */
    g_i32RpNum = KS_Write(KS_SRAM, KS_META_RSA_EXP | KS_META_2048 | KS_META_READABLE, (uint32_t *)&g_au32TmpBuf[0]);
    if(g_i32RpNum == -1)
        return -1;

    /* Create and read the Rq key number. Rq is equal to private/public key length. */
    /* The Rq key is for Temporary use, it can be any value. */
    g_i32RqNum = KS_Write(KS_SRAM, KS_META_RSA_EXP | KS_META_2048 | KS_META_READABLE, (uint32_t *)&g_au32TmpBuf[0]);
    if(g_i32RqNum == -1)
        return -1;

    printf("\n[The number of created keys in SRAM of key store] \n");
    printf("  Private key : %d\n", g_i32PrivateKeyNum);
    printf("  P           : %d\n", g_i32PNum);
    printf("  Q           : %d\n", g_i32QNum);
    printf("  Cp          : %d\n", g_i32CpNum);
    printf("  Cq          : %d\n", g_i32CqNum);
    printf("  Dp          : %d\n", g_i32DpNum);
    printf("  Cq          : %d\n", g_i32DqNum);
    printf("  Rp          : %d\n", g_i32RpNum);
    printf("  Rq          : %d\n", g_i32RqNum);
    printf("\n");

    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    char    OutputResult[RSA_KBUF_HLEN];
    uint32_t u32TimeOutCnt;

    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    DEBUG_PORT_Init();

    printf("\n\n+---------------------------------------------------------------------+\n");
    printf("|   Crypto RSA CRT/CRT bypass mode access key from key store sample   |\n");
    printf("+---------------------------------------------------------------------+\n");

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

    /*---------------------------------------
     *  RSA first sign with CRT mode
     *---------------------------------------*/

    /* Configure RSA operation mode and key length */
    if(RSA_Open(CRPT, RSA_MODE_CRT, RSA_KEY_SIZE_2048, &s_sRSABuf, sizeof(s_sRSABuf), 1) != 0)
    {
        printf("\nRSA buffer size is incorrect!!\n");
        goto lexit;
    }
    /* Set RSA private key is read from SRAM of key store */
    RSA_SetKey_KS(CRPT, (uint32_t)g_i32PrivateKeyNum, KS_SRAM, 0);
    RSA_SetDMATransfer_KS(CRPT, Msg, N, (uint32_t)g_i32PNum, (uint32_t)g_i32QNum, (uint32_t)g_i32CpNum, (uint32_t)g_i32CqNum,
                          (uint32_t)g_i32DpNum, (uint32_t)g_i32DqNum, (uint32_t)g_i32RpNum, (uint32_t)g_i32RqNum);
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
    printf("\nRSA sign 1: %s\n", OutputResult);

    /* Verify the signature */
    if(strcasecmp(OutputResult, Sign) == 0)
        printf("\nRSA signature 1 verify OK.\n\n");
    else
    {
        printf("\nRSA signature 1 verify failed!!\n\n");
        goto lexit;
    }

    /*--------------------------------------------------------------------------------
     *  RSA second sign with CRT Bypass mode to speed up performance(use the same key)
     *--------------------------------------------------------------------------------*/
    g_RSA_done = 0;
    g_RSA_error = 0;

    /* Configure RSA operation mode and key length */
    if(RSA_Open(CRPT, RSA_MODE_CRTBYPASS, RSA_KEY_SIZE_2048, &s_sRSABuf, sizeof(s_sRSABuf), 1) != 0)
    {
        printf("\nRSA buffer size is incorrect!!\n");
        goto lexit;
    }
    /* Use the same key, and change Msg */
    RSA_SetKey_KS(CRPT, (uint32_t)g_i32PrivateKeyNum, KS_SRAM, 0);
    RSA_SetDMATransfer_KS(CRPT, Msg2, N, (uint32_t)g_i32PNum, (uint32_t)g_i32QNum, (uint32_t)g_i32CpNum, (uint32_t)g_i32CqNum,
                          (uint32_t)g_i32DpNum, (uint32_t)g_i32DqNum, (uint32_t)g_i32RpNum, (uint32_t)g_i32RqNum);
    RSA_Start(CRPT);

    /* Waiting for RSA operation done */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!g_RSA_done)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for operation done  RSA time-out!\n");
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
    printf("\nRSA sign 2: %s\n", OutputResult);

    /* Verify the message */
    if(strcasecmp(OutputResult, Sign2) == 0)
        printf("\nRSA signature 2 verify OK.\n\n");
    else
    {
        printf("\nRSA signature 2 verify failed!!\n\n");
        goto lexit;
    }
    printf("\nDone.\n");

lexit:

    /* Erase all keys in SRAM of key store */
    EraseAllSramKey();

    while(1);
}
