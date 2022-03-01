/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Show whole ECC flow. Including private key/public key/Signature generation and
 *           Signature verification.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "NuMicro.h"

#define KEY_LENGTH          192  /* Select ECC P-192 curve, 192-bits key length */
#define PRNG_KEY_SIZE       PRNG_KEY_SIZE_256
#define CURVE_P_SIZE        CURVE_P_192

static char d[168];                         /* private key */
static char Qx[168], Qy[168];               /* temporary buffer used to keep output public keys */
static char k[168];                         /* random integer k form [1, n-1]                */
static char msg[] = "This is a message. It could be encypted.";
static char R[168], S[168];                 /* temporary buffer used to keep digital signature (R,S) pair */




#define ENDIAN(x)   ((((x)>>24)&0xff) | (((x)>>8)&0xff00) | (((x)<<8)&0xff0000) | ((x)<<24))

uint8_t Byte2Char(uint8_t c);
void CRPT_IRQHandler(void);
void  dump_buff_hex(uint8_t *pucBuff, int nBytes);
void SYS_Init(void);
void DEBUG_PORT_Init(void);

uint8_t Byte2Char(uint8_t c)
{
    if(c < 10)
        return (c + '0');
    if(c < 16)
        return (c - 10 + 'a');

    return 0;
}


void CRPT_IRQHandler()
{
    ECC_DriverISR(CRPT);
}


void  dump_buff_hex(uint8_t *pucBuff, int nBytes)
{
    int     nIdx, i;

    nIdx = 0;
    while(nBytes > 0)
    {
        printf("0x%04X  ", nIdx);
        for(i = 0; i < 16; i++)
            printf("%02x ", pucBuff[nIdx + i]);
        printf("  ");
        for(i = 0; i < 16; i++)
        {
            if((pucBuff[nIdx + i] >= 0x20) && (pucBuff[nIdx + i] < 127))
                printf("%c", pucBuff[nIdx + i]);
            else
                printf(".");
            nBytes--;
        }
        nIdx += 16;
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

    /* Enable CRPT module clock */
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

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int32_t main(void)
{
    int32_t j, m, err;
    uint32_t time, i, nbits;
    uint32_t au32r[(KEY_LENGTH + 31) / 32];
    uint8_t *au8r;


    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    DEBUG_PORT_Init();

    printf("+---------------------------------------------+\n");
    printf("|   Crypto ECC Public Key Generation Demo     |\n");
    printf("+---------------------------------------------+\n");

    NVIC_EnableIRQ(CRPT_IRQn);
    ECC_ENABLE_INT(CRPT);

    nbits = KEY_LENGTH;

    /* Initial TRNG */
    RNG_Open();

    /* Init Timer */
    SysTick->LOAD  = 0xfffffful;                                              /* set reload register */
    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;    /* Enable SysTick IRQ and SysTick Timer */
    
    au8r = (uint8_t *)&au32r[0];
    do
    {

        /* Generate random number for private key */
        RNG_Random(au32r, (nbits + 31) / 32);

        for(i = 0, j = 0; i < nbits / 8; i++)
        {
            d[j++] = Byte2Char(au8r[i] & 0xf);
            d[j++] = Byte2Char(au8r[i] >> 4);
        }
        d[j] = 0; // NULL end


        printf("Private key = %s\n", d);

        /* Check if the private key valid */
        if(ECC_IsPrivateKeyValid(CRPT, CURVE_P_SIZE, d))
        {
            break;
        }
        else
        {
            /* Invalid key */
            printf("Current private key is not valid. Need a new one.\n");
        }

    }
    while(1);

    /* Reset SysTick to measure time */
    SysTick->VAL = 0;
    if(ECC_GeneratePublicKey(CRPT, CURVE_P_SIZE, d, Qx, Qy) < 0)
    {
        printf("ECC key generation failed!!\n");
        while(1);
    }
    time = 0xffffff - SysTick->VAL;

    printf("Public Qx is %s\n", Qx);
    printf("Public Qy is %s\n", Qy);
    printf("Elapsed time: %d.%d ms\n", time / CyclesPerUs / 1000, time / CyclesPerUs % 1000);

    /*
        Try to generate signature serveral times with private key and verificate them with the same
        public key.

    */
    for(m = 0; m < 3; m++)
    {
        printf("//-------------------------------------------------------------------------//\n");

        /* Generate random number k */
        RNG_Random(au32r, (nbits + 31) / 32);

        for(i = 0, j = 0; i < nbits / 8; i++)
        {
            k[j++] = Byte2Char(au8r[i] & 0xf);
            k[j++] = Byte2Char(au8r[i] >> 4);
        }
        k[j] = 0; // NULL End

        printf("  k = %s\n", k);

        if(ECC_IsPrivateKeyValid(CRPT, CURVE_P_SIZE, k))
        {
            /* Private key check ok */
        }
        else
        {
            /* Invalid key */
            printf("Current k is not valid\n");
            return -1;

        }

        SysTick->VAL = 0;
        if(ECC_GenerateSignature(CRPT, CURVE_P_SIZE, msg, d, k, R, S) < 0)
        {
            printf("ECC signature generation failed!!\n");
            return -1;
        }
        time = 0xffffff - SysTick->VAL;

        printf("  R  = %s\n", R);
        printf("  S  = %s\n", S);
        printf("  msg= %s\n", msg);
        printf("Elapsed time: %d.%d ms\n", time / CyclesPerUs / 1000, time / CyclesPerUs % 1000);

        SysTick->VAL = 0;
        err = ECC_VerifySignature(CRPT, CURVE_P_SIZE, msg, Qx, Qy, R, S);
        time = 0xffffff - SysTick->VAL;
        if(err < 0)
        {
            printf("ECC signature verification failed!!\n");
            return -1;
        }
        else
        {
            printf("ECC digital signature verification OK.\n");
        }
        printf("Elapsed time: %d.%d ms\n", time / CyclesPerUs / 1000, time / CyclesPerUs % 1000);
    }

    printf("Demo Done.\n");

    while(1);
}
