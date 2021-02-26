/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Demonstrate how to use FMC Read-All-One ISP command to verify
 *           APROM/LDROM pages are all 0xFFFFFFFF or not.
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
    int         ret;                   /* return value */
    uint32_t    u32Data;

    SYS_UnlockReg();                   /* Unlock protected registers */

    SYS_Init();                        /* Init System, IP clock and multi-function I/O */

    UART0_Init();                      /* Initialize UART0 */

#ifdef _PZ
    /* For palladium */
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(153600, 38400);
#endif

    printf("+------------------------------------------+\n");
    printf("|     M460 FMC_ReadAllOne Sample Demo      |\n");
    printf("+------------------------------------------+\n");

    FMC_Open();                        /* Enable FMC ISP function */

    u32Data = FMC_ReadCID();           /* Read company ID. Should be 0xDA. */
    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    u32Data = FMC_ReadPID();           /* Read product ID. */
    printf("  Product ID ............................ [0x%08x]\n", u32Data);

    /* Read User Configuration CONFIG0 */
    printf("  User Config 0 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE));
    /* Read User Configuration CONFIG1 */
    printf("  User Config 1 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE+4));

    /* Read Data Flash base address */
    printf("  Data Flash Base Address ............... [0x%08x]\n", FMC_ReadDataFlashBaseAddr());

    FMC_ENABLE_LD_UPDATE();            /* Enable LDROM update. */

    FMC_Erase(FMC_LDROM_BASE);         /* Erase LDROM page 0. */

    /* Run and check flash contents are all 0xFFFFFFFF. */
    ret = FMC_CheckAllOne(FMC_LDROM_BASE, FMC_FLASH_PAGE_SIZE);
    if (ret == READ_ALLONE_YES)                  /* return value READ_ALLONE_YES means all flash contents are 0xFFFFFFFF */
        printf("READ_ALLONE_YES success.\n");    /* FMC_CheckAllOne() READ_ALLONE_YES passed on LDROM page 0. */
    else
        printf("READ_ALLONE_YES failed!\n");     /* FMC_CheckAllOne() READ_ALLONE_YES failed on LDROM page 0. */

    FMC_Write(FMC_LDROM_BASE, 0);      /* program a 0 to LDROM to make it not all 0xFFFFFFFF. */

    /* Run and check flash contents are not all 0xFFFFFFFF. */
    ret = FMC_CheckAllOne(FMC_LDROM_BASE, FMC_FLASH_PAGE_SIZE);
    if (ret == READ_ALLONE_NOT)
        printf("READ_ALLONE_NOT success.\n");   /* FMC_CheckAllOne() READ_ALLONE_NOT passed on LDROM page 0. */
    else
        printf("READ_ALLONE_NOT failed!\n");    /* FMC_CheckAllOne() READ_ALLONE_NOT failed on LDROM page 0. */

    printf("\nFMC Read-All-One test done.\n");

    FMC_Close();                       /* Disable FMC ISP function */

    SYS_LockReg();                     /* Lock protected registers */

    while (1);
}

/*** (C) COPYRIGHT 2021 Nuvoton Technology Corp. ***/
