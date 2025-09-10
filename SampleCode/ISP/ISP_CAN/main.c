/***************************************************************************//**
 * @file     main.c
 * @brief    ISP tool main function
 * @version  0x32
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PLL_CLOCK                         200000000

#define GPIO_SETMODE(port, pin, u32Mode)  ((port)->MODE = ((port)->MODE & ~(0x3ul << ((pin) << 1))) | ((u32Mode) << ((pin) << 1)))

#define CAN_BAUD_RATE                     500000
#define Master_ISP_ID                     0x487
#define Device0_ISP_ID                    0x784
#define CAN_ISP_DtatLength                0x08
#define CAN_RETRY_COUNTS                  0x1fffffff

#define CMD_READ_CONFIG                   0xA2000000
#define CMD_RUN_APROM                     0xAB000000
#define CMD_GET_DEVICEID                  0xB1000000

/*---------------------------------------------------------------------------*/
/*  Function Declare                                                         */
/*---------------------------------------------------------------------------*/
/* Declare a CAN message structure */
typedef struct
{
    uint32_t  Address;
    uint32_t  Data;
} STR_CANMSG_ISP_T;

static volatile CANFD_FD_MSG_T g_sRxMsgFrame;
static volatile uint8_t s_u8CANPackageFlag = 0, s_u8CANAckFlag = 0;

void CANFD00_IRQHandler(void);
int32_t SYS_Init(void);
void CAN_Package_ACK(CANFD_T *psCanfd);
void CAN_Init(void);
void ProcessHardFault(void);
void SH_Return(void);
void SendChar_ToUART(int ch);
uint32_t CAN_Parsing_MSG(uint8_t *u8pMsg);

/* Add implementations to fix linker warnings from the newlib-nano C library in VSCode-GCC14.3.1 */
void _close(void) {}
void _lseek(void) {}
void _read_r(void) {}
void _write_r(void) {}

uint32_t CLK_GetPLLClockFreq(void)
{
    return PLL_CLOCK;
}

void CLK_EnableModuleClock(uint32_t u32ModuleIdx)
{
    uint32_t u32TmpVal = 0UL, u32TmpAddr = 0UL;

    /* Index, 0x0:AHBCLK0, 0x1:APBCLK0, 0x2:APBCLK1, 0x3:APBCLK2, 0x4:AHBCLK1 */
    uint32_t au32ClkEnTbl[5] = {0x0UL, 0x4UL, 0x8UL, 0x34UL, 0x54UL};

    u32TmpVal = (1UL << MODULE_IP_EN_Pos(u32ModuleIdx));
    u32TmpAddr = (uint32_t)&CLK->AHBCLK0 + au32ClkEnTbl[MODULE_APBCLK(u32ModuleIdx)];

    *(volatile uint32_t *)u32TmpAddr |= u32TmpVal;
}

void CLK_DisableModuleClock(uint32_t u32ModuleIdx)
{
    uint32_t u32TmpVal = 0UL, u32TmpAddr = 0UL;

    /* Index, 0x0:AHBCLK0, 0x1:APBCLK0, 0x2:APBCLK1, 0x3:APBCLK2, 0x4:AHBCLK1 */
    uint32_t au32ClkEnTbl[5] = {0x0UL, 0x4UL, 0x8UL, 0x34UL, 0x54UL};

    u32TmpVal = ~(1UL << MODULE_IP_EN_Pos(u32ModuleIdx));
    u32TmpAddr = (uint32_t)&CLK->AHBCLK0 + au32ClkEnTbl[MODULE_APBCLK(u32ModuleIdx)];

    *(uint32_t *)u32TmpAddr &= u32TmpVal;
}

uint32_t CLK_GetModuleClockSource(uint32_t u32ModuleIdx)
{
    uint32_t u32TmpVal = 0UL, u32TmpAddr = 0UL;
    uint32_t au32SelTbl[5] = {0x0UL, 0x4UL, 0x8UL, 0xCUL, 0x4CUL}; /* CLK_CLKSEL0~4 */
    uint32_t u32RTCCKEN = CLK->APBCLK0 & CLK_APBCLK0_RTCCKEN_Msk;

    /* Get clock source selection setting */
    if(u32ModuleIdx == RTC_MODULE)
    {
        if(u32RTCCKEN == 0UL)
        {
            /* Enable RTC clock to get LXT clock source */
            CLK->APBCLK0 |= CLK_APBCLK0_RTCCKEN_Msk;
        }

        u32TmpVal = ((RTC->LXTCTL & RTC_LXTCTL_RTCCKSEL_Msk) >> RTC_LXTCTL_RTCCKSEL_Pos);

        if(u32RTCCKEN == 0UL)
        {
            /* Disable RTC clock if it is disabled before */
            CLK->APBCLK0 &= (~CLK_APBCLK0_RTCCKEN_Msk);
        }

    }
    else if(MODULE_CLKSEL_Msk(u32ModuleIdx) != MODULE_NoMsk)
    {
        /* Get clock select control register address */
        u32TmpAddr = (uint32_t)&CLK->CLKSEL0 + (au32SelTbl[MODULE_CLKSEL(u32ModuleIdx)]);

        /* Get clock source selection setting */
        u32TmpVal = ((inpw((uint32_t *)u32TmpAddr) & (MODULE_CLKSEL_Msk(u32ModuleIdx) << MODULE_CLKSEL_Pos(u32ModuleIdx))) >> MODULE_CLKSEL_Pos(u32ModuleIdx));
    }

    return u32TmpVal;
}

uint32_t CLK_GetHCLKFreq(void)
{
    SystemCoreClockUpdate();
    return SystemCoreClock;
}

/*---------------------------------------------------------------------------------------------------------*/
/* ISR to handle CAN FD0 Line0 interrupt event                                                             */
/*---------------------------------------------------------------------------------------------------------*/
void CANFD00_IRQHandler(void)
{
    uint32_t u32IIDRstatus;

    u32IIDRstatus = CANFD0->IR;

    /**************************/
    /* Status Change interrupt*/
    /**************************/
    if((u32IIDRstatus & CANFD_IR_RF0N_Msk) == CANFD_IR_RF0N_Msk)
    {
        /*Clear the Interrupt flag */
        CANFD_ClearStatusFlag(CANFD0, CANFD_IR_TOO_Msk | CANFD_IR_RF0N_Msk);
        CANFD_ReadRxFifoMsg(CANFD0, 0, (CANFD_FD_MSG_T *)&g_sRxMsgFrame);
        s_u8CANPackageFlag = 1;
    }
}

int32_t SYS_Init(void)
{
    uint32_t u32TimeOutCnt;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK->PWRCTL |= CLK_PWRCTL_HIRCEN_Msk;

    /* Wait for HIRC clock ready */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!(CLK->STATUS & CLK_STATUS_HIRCSTB_Msk))
    {
        if(--u32TimeOutCnt == 0)
            return -1;
    }

    /* Set HCLK clock source as HIRC first */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_HIRC;

    /* Disable PLL clock before setting PLL frequency */
    CLK->PLLCTL |= CLK_PLLCTL_PD_Msk;

    /* Set PLL clock as 200MHz from HIRC */
    CLK->PLLCTL = CLK_PLLCTL_200MHz_HIRC;

    /* Wait for PLL clock ready */
    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!(CLK->STATUS & CLK_STATUS_PLLSTB_Msk))
    {
        if(--u32TimeOutCnt == 0)
            return -1;
    }

    /* Set power level by HCLK working frequency */
    SYS->PLCTL = (SYS->PLCTL & (~SYS_PLCTL_PLSEL_Msk)) | SYS_PLCTL_PLSEL_PL0;

    /* Set flash access cycle by HCLK working frequency */
    FMC->CYCCTL = (FMC->CYCCTL & (~FMC_CYCCTL_CYCLE_Msk)) | (8);

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Select HCLK clock source as PLL and HCLK source divider as 1 */
    CLK->CLKDIV0 = (CLK->CLKDIV0 & (~CLK_CLKDIV0_HCLKDIV_Msk)) | CLK_CLKDIV0_HCLK(1);
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_HCLKSEL_Msk)) | CLK_CLKSEL0_HCLKSEL_PLL;

    /* Update System Core Clock */
    PllClock        = 200000000;
    SystemCoreClock = 200000000;
    CyclesPerUs     = SystemCoreClock / 1000000;  /* For CLK_SysTickDelay() */

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;
    CLK->AHBCLK1 |= CLK_AHBCLK1_GPICKEN_Msk | CLK_AHBCLK1_GPJCKEN_Msk;

    /* Select CAN FD0 clock source is HCLK */
    CLK->CLKDIV5 = (CLK->CLKDIV5 & (~CLK_CLKDIV5_CANFD0DIV_Msk)) | CLK_CLKDIV5_CANFD0(1);
    CLK->CLKSEL0 = (CLK->CLKSEL0 & (~CLK_CLKSEL0_CANFD0SEL_Msk)) | CLK_CLKSEL0_CANFD0SEL_HCLK;

    /* Enable CAN FD0 peripheral clock */
    CLK->AHBCLK1 |= CLK_AHBCLK1_CANFD0CKEN_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set PC multi-function pins for CAN FD0 RXD and TXD */
    SET_CAN0_RXD_PC4();
    SET_CAN0_TXD_PC5();

    return 0;
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Tx Msg by Normal Mode Function (With Message RAM)                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void CAN_Package_ACK(CANFD_T *psCanfd)
{
    CANFD_FD_MSG_T  sTxMsgFrame;
    s_u8CANAckFlag = 1;
    /* Send a 11-bit Standard Identifier message */
    sTxMsgFrame.eFrmType = eCANFD_DATA_FRM;
    sTxMsgFrame.eIdType  = eCANFD_SID;
    sTxMsgFrame.u32Id    = Device0_ISP_ID;
    sTxMsgFrame.u32DLC   = CAN_ISP_DtatLength;

    sTxMsgFrame.au32Data[0] = g_sRxMsgFrame.au32Data[0];
    sTxMsgFrame.au32Data[1] = g_sRxMsgFrame.au32Data[1];

    if(CANFD_TransmitTxMsg(psCanfd, 0, &sTxMsgFrame) == FALSE)     // Configure Msg RAM and send the Msg in the RAM
    {
        return;
    }

}

void CAN_Init(void)
{
    CANFD_FD_T sCANFD_Config;

    /* Enable CAN FD0 peripheral clock */
    CLK->AHBCLK1 |= CLK_AHBCLK1_CANFD0CKEN_Msk;

    /* Set PC multi-function pins for CAN FD0 RXD and TXD */
    SET_CAN0_RXD_PC4();
    SET_CAN0_TXD_PC5();

    /* Set CAN transceiver to high speed mode */
    GPIO_SETMODE(PC, 11, GPIO_MODE_OUTPUT);
    PC11 = 0;

    /* Get the CAN configuration value */
    CANFD_GetDefaultConfig(&sCANFD_Config, CANFD_OP_CAN_MODE);
    sCANFD_Config.sBtConfig.sNormBitRate.u32BitRate = CAN_BAUD_RATE;
    sCANFD_Config.sBtConfig.sDataBitRate.u32BitRate = 0;

    /* Open the CAN feature */
    CANFD_Open(CANFD0, &sCANFD_Config);
    NVIC_EnableIRQ(CANFD00_IRQn);

    /* Set CAN reveive message */
    CANFD_SetSIDFltr(CANFD0, 0, CANFD_RX_FIFO0_STD_MASK(Master_ISP_ID, 0x7FF));

    /* Enable Standard ID and  Extended ID Filter as RX FOFI0*/
    CANFD_SetGFC(CANFD0, eCANFD_ACC_NON_MATCH_FRM_RX_FIFO0, eCANFD_ACC_NON_MATCH_FRM_RX_FIFO0, 1, 1);
    /* Enable RX fifo0 new message interrupt using interrupt line 0. */
    CANFD_EnableInt(CANFD0, (CANFD_IE_TOOE_Msk | CANFD_IE_RF0NE_Msk | CANFD_IE_TCE_Msk), 0, 0, 0);

    /* CAN FD0 Run to Normal mode  */
    CANFD_RunToNormal(CANFD0, TRUE);

}

/*---------------------------------------------------------------------------------------------------------*/
/* MAIN function                                                                                           */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    if(SYS_Init() < 0)
        goto _APROM;

    /* Enable FMC ISP AP CFG function & clear ISPFF */
    FMC->ISPCTL |= FMC_ISPCTL_ISPEN_Msk | FMC_ISPCTL_APUEN_Msk | FMC_ISPCTL_CFGUEN_Msk | FMC_ISPCTL_ISPFF_Msk;

    SCB->VTOR = FMC_LDROM_BASE;
    /* Init CAN port */
    CAN_Init();

    SysTick->LOAD = 300000 * CyclesPerUs;
    SysTick->VAL   = (0x00);
    SysTick->CTRL = SysTick->CTRL | SysTick_CTRL_CLKSOURCE_Msk | SysTick_CTRL_ENABLE_Msk;

    while(1)
    {
        if(s_u8CANPackageFlag == 1)
        {
            break;
        }

        if(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)
        {
            goto _APROM;
        }
    }

    /* state of update program */
    while(1)
    {
        if(s_u8CANPackageFlag)
        {
            volatile STR_CANMSG_ISP_T *psISPCanMsg = (STR_CANMSG_ISP_T *)&g_sRxMsgFrame.au32Data[0];
            s_u8CANPackageFlag = 0;

            if(psISPCanMsg->Address == CMD_GET_DEVICEID)
            {
                psISPCanMsg->Data = SYS->PDID;
            }
            else if(psISPCanMsg->Address == CMD_READ_CONFIG)
            {
                psISPCanMsg->Data = FMC_Read(psISPCanMsg->Data);
            }
            else if(psISPCanMsg->Address == CMD_RUN_APROM)
            {
                break;
            }
            else
            {
                PD2 = 0;
                if((psISPCanMsg->Address % FMC_FLASH_PAGE_SIZE) == 0)
                {
                    FMC_Erase(psISPCanMsg->Address);
                }

                FMC_Write(psISPCanMsg->Address, psISPCanMsg->Data);
                psISPCanMsg->Data = FMC_Read(psISPCanMsg->Address);
                PD2 = 1;
            }

            /* send CAN FD ISP Package (ACK) */
            CAN_Package_ACK(CANFD0);
        }
    }

_APROM:
    /* Reset system and boot from APROM */
    FMC_SetVectorPageAddr(FMC_APROM_BASE);
    NVIC_SystemReset();

    /* Trap the CPU */
    while(1);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Empty functions for reduce code size to fit  into LDROM & solve the functions are not be defined.      */
/*---------------------------------------------------------------------------------------------------------*/
void ProcessHardFault()
{}

void SH_Return()
{}

void SendChar_ToUART(int ch)
{
    (void)ch;
}
