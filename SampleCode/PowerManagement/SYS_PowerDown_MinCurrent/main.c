/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to minimize power consumption when entering power down mode.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*
//-------- <<< Use Configuration Wizard in Context Menu >>> -----------------
*/

/*
// <o0> Power-down Mode
//      <0=> PD
//      <1=> LLPD
//      <2=> FWPD
//      <4=> SPD
//      <6=> DPD
*/
#define SET_PDMSEL       0

/*
// <o0> SPD mode SRAM retention size
//      <0=> 0KB
//      <1=> 16KB
//      <2=> 32KB
//      <3=> 64KB
//      <4=> 128KB
//      <5=> 256KB
*/
#define SET_SRETSEL       0

/*
// <o0> LVR
//      <0=> Disable
//      <1=> Enable
*/
#define SET_LVR       0

/*
// <o0> POR
//      <0=> Disable
//      <1=> Enable
*/
#define SET_POR       0

/*
// <o0> LIRC
//      <0=> Disable
//      <1=> Enable
*/
#define SET_LIRC       0

/*
// <o0> LXT
//      <0=> Disable
//      <1=> Enable
*/
#define SET_LXT       0



#define GPIO_P0_TO_P15      0xFFFF

void SYS_Disable_AnalogPORCircuit(void);
void PowerDownFunction(void);
void GPB_IRQHandler(void);
int32_t LvrSetting(void);
void PorSetting(void);
int32_t LircSetting(void);
int32_t LxtSetting(void);
void SYS_Init(void);
void UART0_Init(void);


/*---------------------------------------------------------------------------------------------------------*/
/*  Function for System Entry to Power Down Mode                                                           */
/*---------------------------------------------------------------------------------------------------------*/
void PowerDownFunction(void)
{
    uint32_t u32TimeOutCnt;

    /* Check if all the debug messages are finished */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    UART_WAIT_TX_EMPTY(DEBUG_PORT)
        if(--u32TimeOutCnt == 0) break;

    /* Select Power-down mode */
    CLK_SetPowerDownMode(SET_PDMSEL<<CLK_PMUCTL_PDMSEL_Pos);

    /* Enter to Power-down mode */
    CLK_PowerDown();
}

/**
 * @brief       GPIO PB IRQ
 *
 * @param       None
 *
 * @return      None
 *
 * @details     The PB default IRQ, declared in startup_m460.s.
 */
void GPB_IRQHandler(void)
{
    volatile uint32_t u32temp;

    /* To check if PB.2 interrupt occurred */
    if(GPIO_GET_INT_FLAG(PB, BIT2))
    {
        GPIO_CLR_INT_FLAG(PB, BIT2);
        printf("PB2 INT occurred.\n");
    }
    else
    {
        /* Un-expected interrupt. Just clear all PB interrupts */
        u32temp = PB->INTSRC;
        PB->INTSRC = u32temp;
        printf("Un-expected interrupts.\n");
    }
}

int32_t LvrSetting(void)
{
    uint32_t u32TimeOutCnt;

    if(SET_LVR == 0)
    {
        SYS_DISABLE_LVR();
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while( SYS->BODCTL & SYS_BODCTL_LVRRDY_Msk )
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for LVR disable time-out!\n");
                return -1;
            }
        }
    }
    else
    {
        SYS_ENABLE_LVR();
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while( (SYS->BODCTL & SYS_BODCTL_LVRRDY_Msk) == 0 )
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for LVR enable time-out!\n");
                return -1;
            }
        }
    }

    return 0;
}

void PorSetting(void)
{
    if(SET_POR == 0)
    {
        SYS_DISABLE_POR();
    }
    else
    {
        SYS_ENABLE_POR();
    }
}

int32_t LircSetting(void)
{
    uint32_t u32TimeOutCnt;

    if(SET_LIRC == 0)
    {
        CLK_DisableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while( CLK->STATUS & CLK_STATUS_LIRCSTB_Msk )
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for LIRC disable time-out!\n");
                return -1;
            }
        }
    }
    else
    {
        CLK_EnableXtalRC(CLK_PWRCTL_LIRCEN_Msk);
        if( CLK_WaitClockReady(CLK_PWRCTL_LIRCEN_Msk) == 0)
        {
            printf("Wait for LIRC enable time-out!\n");
            return -1;
        }
    }

    return 0;
}

int32_t LxtSetting(void)
{
    uint32_t u32TimeOutCnt;

    if(SET_LXT == 0)
    {
        CLK_DisableXtalRC(CLK_PWRCTL_LXTEN_Msk);
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while( CLK->STATUS & CLK_STATUS_LXTSTB_Msk )
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for LXT disable time-out!\n");
                return -1;
            }
        }
    }
    else
    {
        CLK_EnableXtalRC(CLK_PWRCTL_LXTEN_Msk);
        if( CLK_WaitClockReady(CLK_PWRCTL_LXTEN_Msk) == 0)
        {
            printf("Wait for LXT enable time-out!\n");
            return -1;
        }
    }

    return 0;
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
int32_t main(void)
{
    uint32_t u32TimeOutCnt, u32PMUSTS;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Clear SPD/DPD mode wake-up status for enterning SPD/DPD mode again */
    u32PMUSTS = CLK->PMUSTS;
    if( CLK->PMUSTS )
    {
        /* Release I/O hold status for SPD mode */
        CLK->IOPDCTL = 1;

        /* Clear SPD/DPD mode wake-up status */
        CLK->PMUSTS = CLK_PMUSTS_CLRWK_Msk;
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while(CLK->PMUSTS)
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for SPD/DPD mode wake-up status is cleared time-out!\n");
                return -1;
            }
        }
    }

    /* PB.2 falling-edge wake-up event */
    if( u32PMUSTS & (CLK_PMUSTS_PINWK2_Msk|CLK_PMUSTS_GPBWK_Msk) )
    {
        printf("System waken-up done.\n\n");
        while(1);
    }

    printf("\n\nCPU @ %d Hz\n", SystemCoreClock);
    printf("+-------------------------------------------------------------------+\n");
    printf("|  SYS_PowerDown_MinCurrent and Wake-up by PB.2 Sample Code         |\n");
    printf("+-------------------------------------------------------------------+\n\n");

    printf("+-------------------------------------------------------------------+\n");
    printf("| Operating sequence                                                |\n");
    printf("|  1. Remove all continuous load, e.g. LED.                         |\n");
    printf("|  2. Configure all GPIO as Quasi-bidirectional Mode                |\n");
    printf("|  3. Disable LVR                                                   |\n");
    printf("|  4. Disable analog function, e.g. POR module                      |\n");
    printf("|  5. Disable unused clock, e.g. LIRC                               |\n");
    printf("|  6. Disable SRAM retention for SPD mode                           |\n");
    printf("|  7. Enter to Power-Down                                           |\n");
    printf("|  8. Wait for PB.2 falling-edge interrupt event to wake-up the MCU |\n");
    printf("+-------------------------------------------------------------------+\n\n");

    /* Check if all the debug messages are finished */
    UART_WAIT_TX_EMPTY(DEBUG_PORT);

    /* Set function pin to GPIO mode expect UART pin to print message */
    SYS->GPA_MFP0 = 0;
    SYS->GPA_MFP1 = 0;
    SYS->GPA_MFP2 = 0;
    SYS->GPA_MFP3 = 0;
    SYS->GPB_MFP0 = 0;
    SYS->GPB_MFP1 = 0;
    SYS->GPB_MFP2 = 0;
    SYS->GPB_MFP3 = (UART0_RXD_PB12 | UART0_TXD_PB13);
    SYS->GPC_MFP0 = 0;
    SYS->GPC_MFP1 = 0;
    SYS->GPC_MFP2 = 0;
    SYS->GPC_MFP3 = 0;
    SYS->GPD_MFP0 = 0;
    SYS->GPD_MFP1 = 0;
    SYS->GPD_MFP2 = 0;
    SYS->GPD_MFP3 = 0;
    SYS->GPE_MFP0 = 0;
    SYS->GPE_MFP1 = 0;
    SYS->GPE_MFP2 = 0;
    SYS->GPE_MFP3 = 0;
    SYS->GPF_MFP0 = 0;
    SYS->GPF_MFP1 = 0;
    SYS->GPF_MFP2 = 0;
    SYS->GPF_MFP3 = 0;
    SYS->GPG_MFP0 = 0;
    SYS->GPG_MFP1 = 0;
    SYS->GPG_MFP2 = 0;
    SYS->GPG_MFP3 = 0;
    SYS->GPH_MFP0 = 0;
    SYS->GPH_MFP1 = 0;
    SYS->GPH_MFP2 = 0;
    SYS->GPH_MFP3 = 0;
    SYS->GPI_MFP0 = 0;
    SYS->GPI_MFP1 = 0;
    SYS->GPI_MFP2 = 0;
    SYS->GPI_MFP3 = 0;
    SYS->GPJ_MFP0 = 0;
    SYS->GPJ_MFP1 = 0;
    SYS->GPJ_MFP2 = 0;
    SYS->GPJ_MFP3 = 0;

    /* Configure all GPIO as Quasi-bidirectional Mode */
    GPIO_SetMode(PA, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PB, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PC, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PD, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PE, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PF, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PG, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PH, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PI, GPIO_P0_TO_P15, GPIO_MODE_QUASI);
    GPIO_SetMode(PJ, GPIO_P0_TO_P15, GPIO_MODE_QUASI);

    /* Unlock protected registers for Power-down and wake-up setting */
    SYS_UnlockReg();

    /* LVR setting */
    if( LvrSetting() < 0 ) return -1;

    /* POR setting */
    PorSetting();

    /* LIRC setting */
    if( LircSetting() < 0 ) return -1;

    /* LXT setting */
    if( LxtSetting() < 0 ) return -1;

    /* Select SPD mode SRAM retention size */
    CLK->PMUCTL = (CLK->PMUCTL & (~CLK_PMUCTL_SRETSEL_Msk)) | (SET_SRETSEL<<CLK_PMUCTL_SRETSEL_Pos);

    /* Wake-up source configuraion */
    if( ( SET_PDMSEL == CLK_PMUCTL_PDMSEL_PD ) ||
        ( SET_PDMSEL == CLK_PMUCTL_PDMSEL_LLPD ) ||
        ( SET_PDMSEL == CLK_PMUCTL_PDMSEL_FWPD ) )
    {
        /* Configure PB.2 as Quasi mode and enable interrupt by falling edge trigger */
        GPIO_SetMode(PB, BIT2, GPIO_MODE_QUASI);
        GPIO_EnableInt(PB, 2, GPIO_INT_FALLING);
        NVIC_EnableIRQ(GPB_IRQn);
    }
    else if( SET_PDMSEL == CLK_PMUCTL_PDMSEL_SPD )
    {
        CLK_EnableSPDWKPin(1, 2, CLK_SPDWKPIN_FALLING, CLK_SPDWKPIN_DEBOUNCEDIS);
    }
    else if( SET_PDMSEL == CLK_PMUCTL_PDMSEL_DPD )
    {
        CLK_EnableDPDWKPin(CLK_DPDWKPIN2_FALLING);
    }
    else
    {
        printf("Unknown Power-down mode!\n");
        return -1;
    }

    /* Enter to Power-down mode */
    if( SET_PDMSEL == CLK_PMUCTL_PDMSEL_PD )        printf("Enter to PD Power-Down ......\n");
    else if( SET_PDMSEL == CLK_PMUCTL_PDMSEL_LLPD ) printf("Enter to LLPD Power-Down ......\n");
    else if( SET_PDMSEL == CLK_PMUCTL_PDMSEL_FWPD ) printf("Enter to FWPD Power-Down ......\n");
    else if( SET_PDMSEL == CLK_PMUCTL_PDMSEL_SPD )  printf("Enter to SPD Power-Down ......\n");
    else if( SET_PDMSEL == CLK_PMUCTL_PDMSEL_DPD )  printf("Enter to DPD Power-Down ......\n");

    PowerDownFunction();

    /* Waiting for PB.2 falling-edge interrupt event */
    printf("System waken-up done.\n\n");

    while(1);

}
