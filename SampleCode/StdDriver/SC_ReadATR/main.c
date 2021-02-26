/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Read the smartcard ATR from smartcard 0 interface.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "sclib.h"

#define SC_INTF         0 // Smartcard interface 0

/*---------------------------------------------------------------------------------------------------------*/
/* Functions declaration                                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void SC0_IRQHandler(void);
void SYS_Init(void);
void UART_Init(void);


/**
  * @brief  The interrupt services routine of smartcard port 0
  * @param  None
  * @retval None
  */
void SC0_IRQHandler(void)
{
    /* Please don't remove any of the function calls below */

    // Card insert/remove event occurred, no need to check other event...
    if(SCLIB_CheckCDEvent(SC_INTF))
        return;

    // Check if there's any timeout event occurs. If so, it usually indicates an error
    SCLIB_CheckTimeOutEvent(SC_INTF);

    // Check transmit and receive interrupt, all data transmission take place in this function
    SCLIB_CheckTxRxEvent(SC_INTF);

    /*
        Check if there's any transmission error occurred (e.g. parity error, frame error...)
        These errors will induce SCLIB to deactivation smartcard eventually.
    */
    SCLIB_CheckErrorEvent(SC_INTF);

    return;
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
    
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();


    /*---------------------------------------------------------------------------------------------------------*/
    /* Initialization for sample code                                                                          */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Enable SC0 module clock and clock source from HIRC divide 3, 4MHz*/
    CLK_EnableModuleClock(SC0_MODULE);
    CLK_SetModuleClock(SC0_MODULE, CLK_CLKSEL3_SC0SEL_HIRC, CLK_CLKDIV1_SC0(3));

    /* Set SC0 multi-function pin */
    SET_SC0_PWR_PB2();
    SET_SC0_RST_PB3();
    SET_SC0_DAT_PB4();
    SET_SC0_CLK_PB5();
    SET_SC0_nCD_PC12();
}

void UART_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(DEBUG_PORT, 115200);
}


/*---------------------------------------------------------------------------------------------------------*/
/*  MAIN function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    SCLIB_CARD_INFO_T sCardInfo;
    int32_t i32Ret;
    uint32_t i;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART for printf */
    UART_Init();

    /* Lock protected registers */
    SYS_LockReg();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);
    printf("+--------------------------------------+\n");
    printf("|    Read Smartcard ATR Sample Code    |\n");
    printf("+--------------------------------------+\n\n");
    printf("# I/O configuration:\n");
    printf("    SC0PWR (PB.2)  <--> smart card slot power pin\n");
    printf("    SC0RST (PB.3)  <--> smart card slot reset pin\n");
    printf("    SC0CLK (PB.5)  <--> smart card slot clock pin\n");
    printf("    SC0DAT (PB.4)  <--> smart card slot data pin\n");
    printf("    SC0CD  (PC.12) <--> smart card slot card detect pin\n");
    printf("\nThis sample code reads ATR from smartcard...\n");

    /*
        Open smartcard interface 0. CD pin state low indicates card insert and PWR pin low raise VCC pin to card
        The second and third parameter needs to be set according to the board design
    */
    SC_Open(SC0, SC_PIN_STATE_LOW, SC_PIN_STATE_HIGH);
    NVIC_EnableIRQ(SC0_IRQn);

    // Wait 'til card insert
    while(SC_IsCardInserted(SC0) == FALSE);

    /*
        Activate slot 0, and disable EMV2000 check during card activation
        EMV is a technical standard for smart payment cards and for payment terminals and automated teller
        machines that can accept them. It has a stricter checking rule than ISO 7816-3. If the second
        parameter set as TRUE, SCLIB will report activation failure for cards comply with ISO 7816 but not EMV2000
    */
    i32Ret = SCLIB_Activate(SC_INTF, FALSE);
    if(i32Ret == SCLIB_SUCCESS)
    {
        /*
            Use SCLIB_GetCardInfo to get information about the card, which includes ATR.

            An Answer To Reset (ATR) is a message output by a contact Smart Card conforming to
            ISO/IEC 7816 standards, following electrical reset of the card's chip by a card reader.
            The ATR conveys information about the communication parameters proposed by the card,
            and the card's nature and state.                                --Wikipedia
        */
        SCLIB_GetCardInfo(SC_INTF, &sCardInfo);
        printf("\nATR: ");
        for(i = 0; i < sCardInfo.ATR_Len; i++)
            printf("%02x ", sCardInfo.ATR_Buf[i]);
        printf("\n");
    }
    else
        printf("\nSmartcard activate failed\n");

    while(1) {}
}
