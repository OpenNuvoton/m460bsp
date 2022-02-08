/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to program, read, and lock OTP.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>

#include "NuMicro.h"


/* To enable/disable OTP to be programed (Note: Generally, programing OTP is not allowed)  */
#define PROGRAM_OTP     0

#if PROGRAM_OTP
static volatile uint8_t g_u8ProgAbortFlag;
#endif


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

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();
}

void UART0_Init(void)
{
    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

int main()
{
    uint32_t    u32i, u32OtpHw, u32OtpLw;
#if PROGRAM_OTP
    int32_t     i32Ret, i32GetCh;
#endif
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    printf("+-----------------------------------+\n");
    printf("|   M460 FMC OTP Sample Demo        |\n");
    printf("+-----------------------------------+\n");

    SYS_UnlockReg();                   /* Unlock protected registers */

    FMC_Open();                        /* Enable FMC ISP function */



    for(u32i = 0; u32i < FMC_OTP_ENTRY_CNT; u32i++)
    {
        if(FMC_ReadOTP(u32i, &u32OtpLw, &u32OtpHw) != 0)
        {
            printf("Read OTP%d failed!\n", u32i);
            goto lexit;
        }

        if((u32OtpLw == 0xFFFFFFFF) && (u32OtpHw == 0xFFFFFFFF))
        {
            printf("OTP%d is 0xFFFFFFFF-0xFFFFFFFF. It should be a free entry.\n", u32i);
            break;
        }
    }

    if(u32i == FMC_OTP_ENTRY_CNT)
    {
        printf("All OTP entries are used.\n");
        goto lexit;
    }

#if PROGRAM_OTP
    g_u8ProgAbortFlag = 0;
    printf("\nOTP%d is a free entry?   \n", u32i);
    printf("Program OTP%d?  (y/n) \n", u32i);

    i32GetCh = getchar();

    if(i32GetCh == 'y')
    {
        printf("\n\nWarnning: \n");
        printf("          If OTP is programed, the original OTP data could be destroyed.\n");
        printf("          If OTP is programed, each bit only can be programed from 1 to 0  \n");
        printf("          If OTP is programed, it could not be erased(even mass erase)     \n");
        printf("\n\nReally program OTP?  (y/n)   \n");
        i32GetCh = getchar();
        if(i32GetCh == 'y')
        {
            printf("Program OTP%d with 0x%x-0x%x...\n", u32i, 0x5A5A0000 | u32i, 0x00005A5A | u32i);

            if(FMC_WriteOTP(u32i, 0x5A5A0000 | u32i, 0x00005A5A | u32i) != 0)
            {
                printf("Failed to program OTP%d!\n", u32i);
                goto lexit;
            }
        }
        else
        {
            printf("Abort programing OTP\n");
            g_u8ProgAbortFlag = 1;
        }
    }
    else
    {
        printf("Abort programing OTP\n");
        g_u8ProgAbortFlag = 1;
    }
#endif

    if(FMC_ReadOTP(u32i, &u32OtpLw, &u32OtpHw) != 0)
    {
        printf("Read OTP%d failed after programmed!\n", u32i);
        goto lexit;
    }

    printf("Read back OTP%d: 0x%x-0x%x.\n", u32i, u32OtpLw, u32OtpHw);

#if PROGRAM_OTP
    if(g_u8ProgAbortFlag == 0)
    {
        if((u32OtpLw != (0x5A5A0000 | u32i)) || (u32OtpHw != (0x00005A5A | u32i)))
        {
            printf("OTP%d value is not matched with programmed value!\n", u32i);
            goto lexit;
        }
    }

    i32Ret =    FMC_IsOTPLocked(u32i);

    if(i32Ret == 0)
    {
        printf("\nOTP%d is still not locked   \n", u32i);
        printf("Lock OTP%d?  (y/n)   \n", u32i);


        i32GetCh = getchar();


        if((i32GetCh == 'y') && (g_u8ProgAbortFlag == 0))
        {
            printf("\n\nWarnning: \n");
            printf("          If OTP lock is programed, each bit only can be programed from 1 to 0  \n");
            printf("          If OTP is locked, it could not be erased(even mass erase)     \n");
            printf("\n\nReally lock OTP?  (y/n)   \n");
            i32GetCh = getchar();
            if(i32GetCh == 'y')
            {

                printf("Lock OTP%d...\n", u32i);

                if(FMC_LockOTP(u32i) != 0)
                {
                    printf("Failed to lock OTP%d!\n", u32i);
                    goto lexit;
                }
            }
            else
            {
                printf("Abort OTP lock\n");
                g_u8ProgAbortFlag = 1;
            }
        }
        else
        {
            printf("Abort OTP lock\n");
            g_u8ProgAbortFlag = 1;
        }
    }
    else if(i32Ret == 1)
    {
        printf("\nOTP%d is is already locked   \n", u32i);
    }
    else
    {
        printf("\nRead OTP%d lock failed!\n", u32i);
    }

#endif

    if(FMC_ReadOTP(u32i, &u32OtpLw, &u32OtpHw) != 0)
    {
        printf("Read OTP%d failed after programmed!\n", u32i);
        goto lexit;
    }

    printf("Read OTP%d locked: 0x%x-0x%x.\n", u32i, u32OtpLw, u32OtpHw);

#if PROGRAM_OTP
    if(g_u8ProgAbortFlag == 0)
    {
        if((u32OtpLw != (0x5A5A0000 | u32i)) || (u32OtpHw != (0x00005A5A | u32i)))
        {
            printf("OTP%d value is incorrect after locked!\n", u32i);
            goto lexit;
        }
    }
    g_u8ProgAbortFlag = 0;
#endif

    printf("OTP demo done.\n");

lexit:
    FMC_Close();                       /* Disable FMC ISP function */
    SYS_LockReg();                     /* Lock protected registers */

    while(1);
}

/*** (C) COPYRIGHT 2021 Nuvoton Technology Corp. ***/
