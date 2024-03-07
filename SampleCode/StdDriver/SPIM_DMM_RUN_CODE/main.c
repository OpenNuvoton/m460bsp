/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    This sample shows how to make an application booting from APROM
 *           with a sub-routine resided on SPIM flash.
 *
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>

#include "NuMicro.h"

#define USE_4_BYTES_MODE            0            /* W25Q20 does not support 4-bytes address mode. */
#define MFP_SELECT                  0            /* Multi-function pin select                     */
#define SPIM_CIPHER_ON              0


void spim_routine(void);


void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 80MHz */
    CLK_SetCoreClock(80000000);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;
    CLK->AHBCLK1 |= CLK_AHBCLK1_GPICKEN_Msk | CLK_AHBCLK1_GPJCKEN_Msk;

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable SPIM module clock */
    CLK_EnableModuleClock(SPIM_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

#if MFP_SELECT
    /* Init SPIM multi-function pins, MOSI(PE.2), MISO(PE.3), CLK(PE.4), SS(PE.5), D3(PE.6), and D2(PE.7) */
    SET_SPIM_MOSI_PE2();
    SET_SPIM_MISO_PE3();
    SET_SPIM_CLK_PE4();
    SET_SPIM_SS_PE5();
    SET_SPIM_D3_PE6();
    SET_SPIM_D2_PE7();

    PE->SMTEN |= GPIO_SMTEN_SMTEN4_Msk;

    /* Set SPIM I/O pins as high slew rate up to 80 MHz. */
    PE->SLEWCTL = (PE->SLEWCTL & 0xFFFF000F) |
                  (0x1<<GPIO_SLEWCTL_HSREN2_Pos) | (0x1<<GPIO_SLEWCTL_HSREN3_Pos) |
                  (0x1<<GPIO_SLEWCTL_HSREN4_Pos) | (0x1<<GPIO_SLEWCTL_HSREN5_Pos) |
                  (0x1<<GPIO_SLEWCTL_HSREN6_Pos) | (0x1<<GPIO_SLEWCTL_HSREN7_Pos);
#else
    /* Init SPIM multi-function pins, MOSI(PJ.1), MISO(PI.13), CLK(PJ.0), SS(PI.12), D3(PI.15), and D2(PI.14) */
    SET_SPIM_MOSI_PJ1();
    SET_SPIM_MISO_PI13();
    SET_SPIM_CLK_PJ0();
    SET_SPIM_SS_PI12();
    SET_SPIM_D3_PI15();
    SET_SPIM_D2_PI14();

    PJ->SMTEN |= GPIO_SMTEN_SMTEN0_Msk;

    /* Set SPIM I/O pins as high slew rate up to 80 MHz. */
    PI->SLEWCTL = (PI->SLEWCTL & 0x0CFFFFFF) |
                  (0x1<<GPIO_SLEWCTL_HSREN12_Pos) | (0x1<<GPIO_SLEWCTL_HSREN14_Pos) |
                  (0x1<<GPIO_SLEWCTL_HSREN15_Pos) ;
    PJ->SLEWCTL = (PJ->SLEWCTL & 0xF3FFFFF0) |
                  (0x1<<GPIO_SLEWCTL_HSREN0_Pos) | (0x1<<GPIO_SLEWCTL_HSREN1_Pos) |
                  (0x1<<GPIO_SLEWCTL_HSREN13_Pos) ;
#endif

}

void UART0_Init(void)
{
    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

int main()
{
    uint8_t     idBuf[3];

    SYS_UnlockReg();                   /* Unlock protected registers */

    SYS_Init();                        /* Init System, IP clock and multi-function I/O    */

    UART0_Init();                      /* Initialize UART0                                */

    printf("+--------------------------------------------------+\n");
    printf("|    M460 SPIM DMM mode running program on flash   |\n");
    printf("+--------------------------------------------------+\n");

    SYS_UnlockReg();                   /* Unlock protected registers                      */

    SPIM_SET_CLOCK_DIVIDER(1);        /* Set SPIM clock as HCLK divided by 2 */

    SPIM_SET_RXCLKDLY_RDDLYSEL(0);    /* Insert 0 delay cycle. Adjust the sampling clock of received data to latch the correct data. */
    SPIM_SET_RXCLKDLY_RDEDGE();       /* Use SPI input clock rising edge to sample received data. */

    SPIM_SET_DCNUM(8);                /* 8 is the default value. */

    if (SPIM_InitFlash(1) != 0)        /* Initialized SPI flash */
    {
        printf("SPIM flash initialize failed!\n");
        goto lexit;
    }

    SPIM_ReadJedecId(idBuf, sizeof (idBuf), 1);
    printf("SPIM get JEDEC ID=0x%02X, 0x%02X, 0x%02X\n", idBuf[0], idBuf[1], idBuf[2]);

    SPIM_DISABLE_CCM();

    //SPIM_DISABLE_CACHE();
    SPIM_ENABLE_CACHE();

    if (SPIM_CIPHER_ON)
        SPIM_ENABLE_CIPHER();
    else
        SPIM_DISABLE_CIPHER();

    if (SPIM_Enable_4Bytes_Mode(USE_4_BYTES_MODE, 1) != 0)
    {
        printf("SPIM_Enable_4Bytes_Mode failed!\n");
        goto lexit;
    }

    SPIM->CTL1 |= SPIM_CTL1_CDINVAL_Msk;        // invalid cache

    SPIM_EnterDirectMapMode(USE_4_BYTES_MODE, CMD_DMA_FAST_READ, 0);

    while (1)
    {
        printf("\n\nProgram is currently running on APROM flash.\n");
        printf("Press any key to branch to sub-routine on SPIM flash...\n");

        getchar();

        spim_routine();
    }

lexit:

    while(1);
}

/*** (C) COPYRIGHT 2021 Nuvoton Technology Corp. ***/
