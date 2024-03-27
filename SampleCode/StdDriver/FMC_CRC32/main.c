/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to use FMC CRC32 ISP command to calculate the
 *           CRC32 checksum of APROM and LDROM.
 *
 *
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>

#include "NuMicro.h"


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
    uint32_t    u32Data, u32ChkSum;    /* temporary data */

    SYS_UnlockReg();                   /* Unlock protected registers */

    SYS_Init();                        /* Init System, IP clock and multi-function I/O */

    UART0_Init();                      /* Initialize UART0 */

    printf("+------------------------------------+\n");
    printf("|   M460 FMC CRC32 Sample Demo       |\n");
    printf("+------------------------------------+\n");

    FMC_Open();                        /* Enable FMC ISP function */

    u32Data = FMC_ReadCID();           /* Read company ID. Should be 0x530000DA. */
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_ReadCID failed!\n");
        goto lexit;
    }

    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    u32Data = FMC_ReadPID();           /* Read product ID. */
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_ReadPID failed!\n");
        goto lexit;
    }
    printf("  Product ID ............................ [0x%08x]\n", u32Data);

    /* Read User Configuration CONFIG0 */
    printf("  User Config 0 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE));
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_Read(FMC_CONFIG_BASE) failed!\n");
        goto lexit;
    }

    /* Read User Configuration CONFIG1 */
    printf("  User Config 1 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE+4));
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_Read(FMC_CONFIG_BASE+4) failed!\n");
        goto lexit;
    }

    /* Read Data Flash base address */
    printf("  Data Flash Base Address ............... [0x%08x]\n", FMC_ReadDataFlashBaseAddr());

    printf("\nLDROM (0xF100000 ~ 0xF102000) CRC32 checksum =>  ");

    /*
     *  Request FMC hardware to run CRC32 calculation on LDROM.
     */
    u32ChkSum = FMC_GetChkSum(FMC_LDROM_BASE, FMC_LDROM_SIZE);
    if (u32ChkSum == 0xFFFFFFFF)
    {
        printf("Failed on calculating LDROM CRC32 checksum!\n");
        goto lexit;                    /* failed */
    }
    printf("0x%x\n", u32ChkSum);       /* print out LDROM CRC32 check sum value */


    printf("\nAPROM bank0 (0x0 ~ 0x80000) CRC32 checksum =>  ");

    /*
     *  Request FMC hardware to run CRC32 calculation on APROM bank 0.
     *  Note that FMC CRC32 checksum calculation area must not cross bank boundary.
     */
    u32ChkSum = FMC_GetChkSum(FMC_APROM_BASE, 0x80000);
    if (u32ChkSum == 0xFFFFFFFF)
    {
        printf("Failed on calculating APROM bank0 CRC32 checksum!\n");
        goto lexit;
    }
    printf("0x%x\n", u32ChkSum);       /* print out APROM CRC32 check sum value */

    /*
     *  Request FMC hardware to run CRC32 calculation on APROM bank 1. (M460HD only)
     */
    if ((SYS->CSERVER & SYS_CSERVER_VERSION_Msk) == 0x2)
    {
        printf("\nAPROM bank1 (0x80000 ~ 0x100000) CRC32 checksum =>  ");
        u32ChkSum = FMC_GetChkSum(FMC_APROM_BASE+0x80000, 0x80000);
        if (u32ChkSum == 0xFFFFFFFF)
        {
            printf("Failed on calculating APROM bank1 CRC32 checksum!\n");
            goto lexit;
        }
        printf("0x%x\n", u32ChkSum);       /* print out APROM CRC32 check sum value */
    }

    printf("\nFMC CRC32 checksum test done.\n");

lexit:
    FMC_Close();                       /* Disable FMC ISP function */
    SYS_LockReg();                     /* Lock protected registers */

    while (1);
}

/*** (C) COPYRIGHT 2021 Nuvoton Technology Corp. ***/
