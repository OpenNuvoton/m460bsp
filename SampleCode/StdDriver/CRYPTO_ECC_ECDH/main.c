/**************************************************************************//**
 * @file     main.c
 * @version  V1.10
 * @brief    Show Crypto IP ECC CDH secret Z generation.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define B2C(c)    (((uint8_t)(c)<10)?((uint8_t)(c)+'0'):((uint8_t)(c)-10+'a'))


#define ECC_CURVE_SEL  P_256     // Select the curve

// Available Curve for Selection
#define P_192 0
#define P_224 1
#define P_256 2
#define P_384 3
#define P_521 4
#define K_163 5
#define K_233 6
#define K_283 7
#define K_409 8
#define K_571 9
#define B_163 10
#define B_233 11
#define B_283 12
#define B_409 13
#define B_571 14

#if   (ECC_CURVE_SEL == P_192)
#define ECC_CURVE_TYPE  CURVE_P_192
#define ECC_KEY_SIZE    192
#elif (ECC_CURVE_SEL == P_224)
#define ECC_CURVE_TYPE  CURVE_P_224
# define ECC_KEY_SIZE   224
#elif (ECC_CURVE_SEL == P_256)
#define ECC_CURVE_TYPE  CURVE_P_256
#define ECC_KEY_SIZE    256
#elif(ECC_CURVE_SEL == P_384)
#define ECC_CURVE_TYPE  CURVE_P_384
#define ECC_KEY_SIZE    384
#elif(ECC_CURVE_SEL == P_521)
#define ECC_CURVE_TYPE  CURVE_P_521
#define ECC_KEY_SIZE    521
#elif(ECC_CURVE_SEL == K_163)
#define ECC_CURVE_TYPE  CURVE_K_163
#define ECC_KEY_SIZE    163
#elif(ECC_CURVE_SEL == K_233)
#define ECC_CURVE_TYPE  CURVE_K_233
#define ECC_KEY_SIZE    233
#elif(ECC_CURVE_SEL == K_283)
#define ECC_CURVE_TYPE  CURVE_K_283
#define ECC_KEY_SIZE    283
#elif(ECC_CURVE_SEL == K_409)
#define ECC_CURVE_TYPE  CURVE_K_409
#define ECC_KEY_SIZE    409
#elif(ECC_CURVE_SEL == K_571)
#define ECC_CURVE_TYPE  CURVE_K_571
#define ECC_KEY_SIZE    571
#elif(ECC_CURVE_SEL == B_163)
#define ECC_CURVE_TYPE  CURVE_B_163
#define ECC_KEY_SIZE    163
#elif(ECC_CURVE_SEL == B_233)
#define ECC_CURVE_TYPE  CURVE_B_233
#define ECC_KEY_SIZE    233
#elif(ECC_CURVE_SEL == B_283)
#define ECC_CURVE_TYPE  CURVE_B_283
#define ECC_KEY_SIZE    283
#elif(ECC_CURVE_SEL == B_409)
#define ECC_CURVE_TYPE  CURVE_B_409
#define ECC_KEY_SIZE    409
#elif(ECC_CURVE_SEL == B_571)
#define ECC_CURVE_TYPE  CURVE_B_571
#define ECC_KEY_SIZE    571
#endif

void CRPT_IRQHandler(void);
void SYS_Init(void);
void DEBUG_PORT_Init(void);
void GenPrivateKey(char *d, uint32_t u32NBits);

void CRPT_IRQHandler()
{
    ECC_DriverISR(CRPT);
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


void DEBUG_PORT_Init()
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    DEBUG_PORT->LINE = UART_PARITY_NONE | UART_STOP_BIT_1 | UART_WORD_LEN_8;
    DEBUG_PORT->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);

}


void GenPrivateKey(char *d, uint32_t u32NBits)
{
    uint32_t au32r[(ECC_KEY_SIZE + 31) / 32];
    uint8_t *u8r;
    int32_t i, j;
    uint32_t u32Idx;

    u8r = (uint8_t *)&au32r[0];
    do
    {
        /* Generate random number for private key */
        RNG_Random(au32r, (u32NBits + 31) / 32);

        for(u32Idx = 0, j = 0; u32Idx < (u32NBits + 7) / 8; u32Idx++)
        {
            d[j++] = B2C(u8r[u32Idx] & 0xf);
            d[j++] = B2C(u8r[u32Idx] >> 4);
        }
        d[(u32NBits + 3) / 4] = 0;

        /* Check if the private key valid */
        if(ECC_IsPrivateKeyValid(CRPT, ECC_CURVE_TYPE, d))
        {
            break;
        }
        else
        {
            /* Decrease 1 bit and try again */
            d[(u32NBits + 2) / 4] = 0;
            if(ECC_IsPrivateKeyValid(CRPT, ECC_CURVE_TYPE, d))
            {
                if(((u32NBits & 0x3) != 0) && (((u32NBits - 1) & 0x3) == 0))
                {
                    /* Need padding 1 nibble back */
                    j = (u32NBits + 2) / 4;
                    for(i = j; i > 0; i--)
                        d[i] = d[i - 1];
                    d[i + 1] = 0;
                    d[0] = '0';
                }

                break;
            }
            else
            {
                /* Invalid key */
                printf("Current private key is not valid. Need a new one.\n");
            }
        }
    }
    while(1);

}

static char d[168];                          /* private key */
static char d2[168];                         /* private key */
static char Qx[168], Qy[168];                /* temporary buffer used to keep output public keys */
static char Qx2[168], Qy2[168];              /* temporary buffer used to keep output public keys */
static char k[168];                          /* random integer k form [1, n-1]                */
static char k2[168];                         /* random integer k form [1, n-1]                */
//static char msg[] = "78E5B4BFFDFB6798EFF1E5A761E8C60FF64B9CED835910E3D266BD01A2E24CC0";
//static char R[168], S[168];                  /* temporary buffer used to keep digital signature (R,S) pair */

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    uint32_t u32Time;

    /* Lock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    DEBUG_PORT_Init();

    printf("+---------------------------------------------+\n");
    printf("|              Crypto ECC ECDH Demo           |\n");
    printf("+---------------------------------------------+\n");

    NVIC_EnableIRQ(CRPT_IRQn);
    ECC_ENABLE_INT(CRPT);

    /* Initial TRNG */
    RNG_Open();

    /*------------------------------------------------------------------------*/
    /* Generate a private key A */
    GenPrivateKey(d, ECC_KEY_SIZE);
    printf("Private key A = %s\n", d);


    /* Generate public Key A */
    /* Reset SysTick to measure calculation time */
    SysTick->VAL = 0;
    if(ECC_GeneratePublicKey(CRPT, ECC_CURVE_TYPE, d, Qx, Qy) < 0)
    {
        printf("ECC key generation failed!!\n");
        goto lexit;
    }
    u32Time = 0xffffff - SysTick->VAL;

    printf("Pub Key    Ax = %s\n", Qx);
    printf("Pub Key    Ay = %s\n", Qy);
    printf("Elapsed time: %d.%d ms\n", u32Time / CyclesPerUs / 1000, u32Time / CyclesPerUs % 1000);


    /*------------------------------------------------------------------------*/
    /* Generate a private key B */
    GenPrivateKey(d2, ECC_KEY_SIZE);
    printf("Private key  B = %s\n", d2);

    /* Generate public Key B */
    /* Reset SysTick to measure calculation time */
    SysTick->VAL = 0;
    if(ECC_GeneratePublicKey(CRPT, ECC_CURVE_TYPE, d2, Qx2, Qy2) < 0)
    {
        printf("ECC key generation failed!!\n");
        goto lexit;
    }
    u32Time = 0xffffff - SysTick->VAL;

    printf("Pub Key     Bx = %s\n", Qx2);
    printf("Pub Key     By = %s\n", Qy2);
    printf("Elapsed time: %d.%d ms\n", u32Time / CyclesPerUs / 1000, u32Time / CyclesPerUs % 1000);

    /* Calcualte Share Key by private key A and publick key B */
    if(ECC_GenerateSecretZ(CRPT, ECC_CURVE_TYPE, d, Qx2, Qy2, k) < 0)
    {
        printf("ECC ECDH share key calculation fail\n");
        goto lexit;
    }

    printf("Share key calculated by A = %s\n", k);

    /* Calcualte Share Key by private key B and publick key A */
    if(ECC_GenerateSecretZ(CRPT, ECC_CURVE_TYPE, d2, Qx, Qy, k2) < 0)
    {
        printf("ECC ECDH share key calculation fail\n");
        goto lexit;
    }

    printf("Share key calculated by B = %s\n", k2);

    if(strcmp(k, k2) != 0)
    {
        printf("Share key mismatch!!\n");
    }
    else
    {
        printf("Share key match. Test OK\n");
    }

lexit:

    while(1);
}
