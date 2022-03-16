/****************************************************************************//**
 * @file     main.c
 * @version  V0.10
 * @brief    Implement a code to execute in HyperRAM.
 *
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"

#define HBI_MFP_SELECT   0

extern int32_t ExeInHRAM(void);

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

    /* Enable HBI module clock */
    CLK_EnableModuleClock(HBI_MODULE);
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set multi-function pins for HBI */
#if HBI_MFP_SELECT

    SET_HBI_D0_PG11();
    SET_HBI_D1_PG12();
    SET_HBI_D2_PC0();
    SET_HBI_D3_PG10();
    SET_HBI_D4_PG9();
    SET_HBI_D5_PG13();
    SET_HBI_D6_PG14();
    SET_HBI_D7_PG15();

    SET_HBI_RWDS_PC1();
    SET_HBI_nRESET_PC2();
    SET_HBI_nCS_PC3();
    SET_HBI_CK_PC4();
    SET_HBI_nCK_PC5();

#else

    SET_HBI_D0_PJ6();
    SET_HBI_D1_PJ5();
    SET_HBI_D2_PJ4();
    SET_HBI_D3_PJ3();
    SET_HBI_D4_PH15();
    SET_HBI_D5_PD7();
    SET_HBI_D6_PD6();
    SET_HBI_D7_PD5();

    SET_HBI_RWDS_PH14();
    SET_HBI_nRESET_PJ2();
    SET_HBI_nCS_PJ7();
    SET_HBI_CK_PH13();
    SET_HBI_nCK_PH12();

#endif

}

extern uint32_t Load$$HRAM$$RO$$Base;
extern uint32_t Load$$HRAM$$RO$$Limit;
extern uint32_t Load$$HRAM$$RO$$Length;
extern uint32_t Image$$HRAM$$RO$$Base;

int32_t main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    printf("\n\n");
    printf("+------------------------------------------------+\n");
    printf("|      Code execute in HyperRAM Sample Code      |\n");
    printf("+------------------------------------------------+\n");

    /*
       This sample code demonstrates how to make a sub-routine code executed in SRAM.
    */

    printf("Will branch to address: 0x%08x\n", (uint32_t)ExeInHRAM);

    /* Copy code from ROM to HyperRAM manually */
    memcpy(&Image$$HRAM$$RO$$Base, &Load$$HRAM$$RO$$Base, (uint32_t) &Load$$HRAM$$RO$$Length);
    ExeInHRAM();

    printf("\nReturn to execute in APROM \n");
    printf("\nHyperRAM execute code test done \n");

    while (1);

}

/*** (C) COPYRIGHT 2022 Nuvoton Technology Corp. ***/
