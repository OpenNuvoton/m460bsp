/**************************************************************************//**
 * @file     LDROM_iap.c
 * @version  V1.00
 * @brief    FMC LDROM IAP sample program run on LDROM.
 *
 *
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>

#include "NuMicro.h"

typedef void (FUNC_PTR)(void);
void ProcessHardFault(void) { while(1); /* Halt here if hard fault occurs. */ }

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


/**
 * @brief       Routine to send a char
 * @param[in]   ch Character to send to debug port.
 * @returns     Send value from UART debug port
 * @details     Send a target char to UART debug port .
 */
static void SendChar_ToUART(int ch)
{
    while (UART0->FIFOSTS & UART_FIFOSTS_TXFULL_Msk);

    UART0->DAT = ch;
    if(ch == '\n')
    {
        while (UART0->FIFOSTS & UART_FIFOSTS_TXFULL_Msk);
        UART0->DAT = '\r';
    }
}

/**
 * @brief    Routine to get a char
 * @param    None
 * @returns  Get value from UART debug port or semihost
 * @details  Wait UART debug port or semihost to input a char.
 */
static char GetChar(void)
{
    while(1)
    {
        if ((UART0->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk) == 0)
        {
            return (UART0->DAT);
        }
    }
}

static void PutString(char *str)
{
    while (*str != '\0')
    {
        SendChar_ToUART(*str++);
    }
}

#ifdef __GNUC__                        /* for GNU C compiler */
/**
 * @brief       Hard fault handler
 * @param[in]   stack pointer points to the dumped registers in SRAM
 * @return      None
 * @details     Replace while(1) at the end of this function with chip reset if WDT is not enabled for end product
 */
void Hard_Fault_Handler(uint32_t stack[])
{
    PutString("In Hard Fault Handler\n");
    while(1);
}
#endif

int main()
{
    uint32_t    u32TimeOutCnt;         /* time-out counter */

    SYS_UnlockReg();                   /* Unlock protected registers */

    SYS_Init();                        /* Init System, IP clock and multi-function I/O */

    SCB->VTOR = FMC_LDROM_BASE;        /* Set Vector Table Offset Register */

    UART_Open(UART0, 115200);          /* Init UART to 115200-8n1 for print message */

    PutString("\n\n");
    PutString("+-------------------------------------+\n");
    PutString("|     M460 FMC IAP Sample Code        |\n");
    PutString("|          [LDROM code]               |\n");
    PutString("+-------------------------------------+\n");

    FMC_Open();                        /* Enable FMC ISP function */

    PutString("\n\nPress any key to branch to APROM...\n");
    GetChar();                         /* block on waiting for any one character input from UART0 */

    PutString("\n\nChange VECMAP and branch to APROM...\n");
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while (!(UART0->FIFOSTS & UART_FIFOSTS_TXEMPTY_Msk))        /* wait until UART0 TX FIFO is empty */
        if(--u32TimeOutCnt == 0) break;

    /*  NOTE!
     *     Before change VECMAP, user MUST disable all interrupts.
     */
    FMC_SetVectorPageAddr(FMC_APROM_BASE);        /* Vector remap APROM page 0 to address 0. */
    if (g_FMC_i32ErrCode != 0)
    {
        PutString("FMC_SetVectorPageAddr(FMC_APROM_BASE) failed!\n");
        goto lexit;
    }

    SYS_LockReg();                                /* Lock protected registers */

    /* Software reset to boot to APROM */
    NVIC_SystemReset();

lexit:

    while (1);
}

/*** (C) COPYRIGHT 2021 Nuvoton Technology Corp. ***/
