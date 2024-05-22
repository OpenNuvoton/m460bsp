/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Implement a multi-boot system to boot from different applications in APROM.
 *           A LDROM code and 4 APROM code are implemented in this sample code.
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>

#include "NuMicro.h"


#if !defined(__ICCARM__) && !defined(__GNUC__)
extern uint32_t Image$$RO$$Base;
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

int main(void)
{
    uint8_t ch;
    uint32_t u32VECMAP;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /*
        This sample code shows how to boot with different firmware images in APROM.
        In the code, VECMAP is used to implement multi-boot function. Software set VECMAP
        to remap page of VECMAP to 0x0~0x1ff and set VTOR to remap page of all vector table.

        To use this sample code, please:
        1. Build all targets and download to device individually.
           For Keil and IAR project, the targets are:
               FMC_MultiBoot, RO=0x0
               FMC_Boot0, RO=0x4000
               FMC_Boot1, RO=0x8000
               FMC_Boot2, RO=0xC000
               FMC_Boot3, RO=0x10000
               FMC_BootLD, RO=0xF100000
           For GCC project, the targets are:
               FMC_MultiBoot, RO=0x0
               FMC_Boot1, RO=0x8000
               FMC_Boot3, RO=0x10000
        2. Reset MCU to execute FMC_MultiBoot.

    */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Get current vector mapping address */
    u32VECMAP = FMC_GetVECMAP();

    /* Set Vector Table Offset Register */
    if(u32VECMAP == 0x100000)
        SCB->VTOR = FMC_LDROM_BASE;
    else
        SCB->VTOR = u32VECMAP;

    printf("\n\n");
    printf("+----------------------------------------------+\n");
    printf("|     Multi-Boot Sample Code(0x%08X)       |\n", u32VECMAP);
    printf("+----------------------------------------------+\n");

    /* Enable FMC ISP function */
    FMC_Open();

#if defined(__ICCARM__) || defined(__GNUC__)
    printf("VECMAP = 0x%x\n", FMC_GetVECMAP());
#else
#ifndef BootLD
    printf("Current RO Base = 0x%x, VECMAP = 0x%x\n", (uint32_t)&Image$$RO$$Base, FMC_GetVECMAP());
#else
    printf("VECMAP = 0x%x\n", FMC_GetVECMAP());
#endif
#endif

    printf("Select one boot image: \n");
#if defined(__ARMCC_VERSION) || defined(__ICCARM__)
    printf("[0] Boot 0, base = 0x4000\n");
    printf("[1] Boot 1, base = 0x8000\n");
    printf("[2] Boot 2, base = 0xC000\n");
    printf("[3] Boot 3, base = 0x10000\n");
    printf("[4] Boot 4, base = 0xF100000\n");
#else
    printf("[1] Boot 1, base = 0x8000\n");
    printf("[3] Boot 3, base = 0x10000\n");
#endif
    printf("[Others] Boot, base = 0x0\n");

    ch = getchar();
    switch(ch)
    {
#if defined(__ARMCC_VERSION) || defined(__ICCARM__)
        case '0':
            FMC_SetVectorPageAddr(0x4000);
            break;
        case '1':
            FMC_SetVectorPageAddr(0x8000);
            break;
        case '2':
            FMC_SetVectorPageAddr(0xC000);
            break;
        case '3':
            FMC_SetVectorPageAddr(0x10000);
            break;
        case '4':
            FMC_SetVectorPageAddr(0xF100000);
            break;
#else
        case '1':
            FMC_SetVectorPageAddr(0x8000);
            break;
        case '3':
            FMC_SetVectorPageAddr(0x10000);
            break;
#endif
        default:
            FMC_SetVectorPageAddr(0x0);
            break;
    }

    if(g_FMC_i32ErrCode != 0)
    {
        printf("FMC_SetVectorPageAddr failed!\n");
        goto lexit;
    }

    /* Reset CPU only to reset to new vector page */
    SYS_ResetCPU();

    /* Reset System to reset to new vector page. */
    // NVIC_SystemReset();

lexit:

    while(1);
}

/*** (C) COPYRIGHT 2021 Nuvoton Technology Corp. ***/
