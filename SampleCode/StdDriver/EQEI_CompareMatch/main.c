/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Show the usage of EQEI compare function.
 *
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


#define EQEI0A   PA0
#define EQEI0B   PA1


void EQEI0_IRQHandler(void)
{
    if(EQEI_GET_INT_FLAG(EQEI0, EQEI_STATUS_CMPF_Msk))     /* Compare-match flag */
    {
        printf("Compare-match INT!\n\n");
        EQEI_CLR_INT_FLAG(EQEI0, EQEI_STATUS_CMPF_Msk);
    }

    if(EQEI_GET_INT_FLAG(EQEI0, EQEI_STATUS_OVUNF_Msk))    /* Counter Overflow or underflow flag */
    {
        printf("Overflow INT!\n\n");
        EQEI_CLR_INT_FLAG(EQEI0, EQEI_STATUS_OVUNF_Msk);
    }
}

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

    /* Enable EQEI0 module clock */
    CLK_EnableModuleClock(EQEI0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set PA multi-function pins for EQEI0_A, EQEI0_B, EQEI0_INDEX */
    SET_EQEI0_A_PA4();
    SET_EQEI0_B_PA3();
    SET_EQEI0_INDEX_PA5();
}

void UART0_Init()
{

    /* Configure UART0 and set UART0 Baudrate */
    UART_Open(UART0, 115200);
}

int main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    printf("+--------------------------------------+\n");
    printf("|     M460 EQEI Driver Sample Code      |\n");
    printf("+--------------------------------------+\n\n");
    printf("  >> Please connect PA.0 and PA.4 << \n");
    printf("  >> Please connect PA.1 and PA.3 << \n");
    printf("     Press any key to start test\n\n");
    getchar();

    /* Configure PA.0 and PA.1 as output mode */
    GPIO_SetMode(PA, BIT0, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PA, BIT1, GPIO_MODE_OUTPUT);

    EQEI0A = 0;
    EQEI0B = 0;

    /* Set EQEI counting mode as X4 Compare-counting mode,
       set maximum counter value and enable IDX, QEA and QEB input */
    EQEI_Open(EQEI0, EQEI_CTL_X4_COMPARE_COUNTING_MODE, 0x20000);

    /* Set counter compare value */
    EQEI_SET_CNT_CMP(EQEI0, 0x10000);

    /* Enable compare function */
    EQEI_ENABLE_CNT_CMP(EQEI0);

    /* Enable EQEI interrupt */
    EQEI_EnableInt(EQEI0, EQEI_CTL_CMPIEN_Msk | EQEI_CTL_OVUNIEN_Msk);

    /* Start EQEI function */
    EQEI_Start(EQEI0);

    /* Wait compare-match and overflow interrupt happened */
    while(1)
    {
        EQEI0A = 1;
        CLK_SysTickDelay(16);
        EQEI0B = 1;
        CLK_SysTickDelay(16);
        EQEI0A = 0;
        CLK_SysTickDelay(16);
        EQEI0B = 0;
        CLK_SysTickDelay(16);
    }

}
