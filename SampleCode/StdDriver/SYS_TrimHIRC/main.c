/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to use LXT to trim HIRC.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"




/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void TrimHIRC(void);
void IRC_IRQHandler(void);
void SYS_Init(void);
void UART0_Init(void);

/*--------------------------------------------------------------------------------------------------------*/
/*  IRCTrim IRQ Handler                                                                                   */
/*--------------------------------------------------------------------------------------------------------*/
void IRC_IRQHandler(void)
{
    if(SYS->IRCTISTS & SYS_IRCTISTS_TFAILIF_Msk)   /* Get Trim Failure Interrupt */
    {
        /* Display HIRC trim status */
        printf("HIRC Trim Failure Interrupt\n");
        /* Clear Trim Failure Interrupt */
        SYS->IRCTISTS = SYS_IRCTISTS_TFAILIF_Msk;
    }
    if(SYS->IRCTISTS & SYS_IRCTISTS_CLKERRIF_Msk)   /* Get LXT Clock Error Interrupt */
    {
        /* Display HIRC trim status */
        printf("LXT Clock Error Interrupt\n");
        /* Clear LXT Clock Error Interrupt */
        SYS->IRCTISTS = SYS_IRCTISTS_CLKERRIF_Msk;
    }

}


void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable LXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);

    /* Wait for LXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_LXTSTB_Msk);

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

    /* Set PC multi-function pin for CLKO(PC.13) */
    SET_CLKO_PC13();
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART0 */
    SYS_ResetModule(UART0_RST);

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    printf("\n\nCPU @ %dHz\n", SystemCoreClock);

    /* Enable Interrupt */
    NVIC_EnableIRQ(IRC_IRQn);

    /* Trim HIRC to 12MHz */
    TrimHIRC();

    /* Disable IRC Trim */
    SYS->IRCTCTL = 0;
    printf("Disable IRC Trim\n");

    while(1);

}

void TrimHIRC(void)
{
    uint32_t u32TimeOutCnt;

    /*  Enable IRC Trim, set HIRC clock and enable interrupt */
    SYS->IRCTIEN |= (SYS_IRCTIEN_CLKEIEN_Msk | SYS_IRCTIEN_TFAILIEN_Msk);
    SYS->IRCTCTL = (SYS->IRCTCTL & ~SYS_IRCTCTL_FREQSEL_Msk) | 0x1;

    CLK_SysTickDelay(2000); /* Waiting for HIRC Frequency Lock */

    /* Get HIRC Frequency Lock */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while((SYS->IRCTISTS & SYS_IRCTISTS_FREQLOCK_Msk) == 0)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("HIRC Trim failed\n");
            return;
        }
    }

    printf("HIRC Frequency Lock\n");

    /* Clear Trim Lock */
    SYS->IRCTISTS = SYS_IRCTISTS_FREQLOCK_Msk;

    /* Enable CLKO and output frequency = HIRC */
    CLK_EnableCKO(CLK_CLKSEL1_CLKOSEL_HIRC, 0, 1);
}
