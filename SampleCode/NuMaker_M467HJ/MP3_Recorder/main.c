/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    MP3 recorder sample records sound to MP3 files stored on SD memory
 *           card and press button to play it.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "config.h"
#include "diskio.h"
#include "ff.h"
#include "l3.h"

FATFS FatFs[FF_VOLUMES];               /* File system object for logical drive */

#ifdef __ICCARM__
#pragma data_alignment=32
BYTE Buff[16] ;                   /* Working buffer */
DMA_DESC_T DMA_DESC[2];
#else
BYTE Buff[16] __attribute__((aligned(32)));       /* Working buffer */
DMA_DESC_T DMA_DESC[2] __attribute__((aligned(32)));
#endif

uint8_t bAudioPlaying = 0;
extern signed int aPCMBuffer[2][PCM_BUFFER_SIZE];

volatile uint32_t u32SW2 = 0xF, u32SW3 = 0xF;
volatile uint32_t u32StartRecord = 1, u32StopRecord = 0, u32RecordDone = 0;

/*---------------------------------------------------------*/
/* User Provided RTC Function for FatFs module             */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support an RTC.                     */
/* This function is not required in read-only cfg.         */

unsigned long get_fattime(void)
{
    unsigned long tmr;

    tmr = 0x00000;

    return tmr;
}

void SDH0_IRQHandler(void)
{
    unsigned int volatile isr;
    unsigned int volatile ier;

    // FMI data abort interrupt
    if(SDH0->GINTSTS & SDH_GINTSTS_DTAIF_Msk)
    {
        /* ResetAllEngine() */
        SDH0->GCTL |= SDH_GCTL_GCTLRST_Msk;
    }

    //----- SD interrupt status
    isr = SDH0->INTSTS;
    if(isr & SDH_INTSTS_BLKDIF_Msk)
    {
        // block down
        SD0.DataReadyFlag = TRUE;
        SDH0->INTSTS = SDH_INTSTS_BLKDIF_Msk;
    }

    if(isr & SDH_INTSTS_CDIF_Msk)    // port 0 card detect
    {
        //----- SD interrupt status
        // it is work to delay 50 times for SD_CLK = 200KHz
        {
            int volatile i;         // delay 30 fail, 50 OK
            for(i = 0; i < 0x500; i++); // delay to make sure got updated value from REG_SDISR.
            isr = SDH0->INTSTS;
        }

        if(isr & SDH_INTSTS_CDSTS_Msk)
        {
            printf("\n***** card remove !\n");
            SD0.IsCardInsert = FALSE;   // SDISR_CD_Card = 1 means card remove for GPIO mode
            memset(&SD0, 0, sizeof(SDH_INFO_T));
        }
        else
        {
            printf("***** card insert !\n");
            SDH_Open(SDH0, CardDetect_From_GPIO);
            SDH_Probe(SDH0);
        }

        SDH0->INTSTS = SDH_INTSTS_CDIF_Msk;
    }

    // CRC error interrupt
    if(isr & SDH_INTSTS_CRCIF_Msk)
    {
        if(!(isr & SDH_INTSTS_CRC16_Msk))
        {
            //printf("***** ISR sdioIntHandler(): CRC_16 error !\n");
            // handle CRC error
        }
        else if(!(isr & SDH_INTSTS_CRC7_Msk))
        {
            if(!SD0.R3Flag)
            {
                //printf("***** ISR sdioIntHandler(): CRC_7 error !\n");
                // handle CRC error
            }
        }
        SDH0->INTSTS = SDH_INTSTS_CRCIF_Msk;      // clear interrupt flag
    }

    if(isr & SDH_INTSTS_DITOIF_Msk)
    {
        printf("***** ISR: data in timeout !\n");
        SDH0->INTSTS |= SDH_INTSTS_DITOIF_Msk;
    }

    // Response in timeout interrupt
    if(isr & SDH_INTSTS_RTOIF_Msk)
    {
        printf("***** ISR: response in timeout !\n");
        SDH0->INTSTS |= SDH_INTSTS_RTOIF_Msk;
    }
}

void SD_Inits(void)
{
    /* Select multi-function pins */
    SET_SD0_DAT0_PE2();
    SET_SD0_DAT1_PE3();
    SET_SD0_DAT2_PE4();
    SET_SD0_DAT3_PE5();
    SET_SD0_CLK_PE6();
    SET_SD0_CMD_PE7();
    SET_SD0_nCD_PD13();

    /* Select IP clock source */
    CLK_SetModuleClock(SDH0_MODULE, CLK_CLKSEL0_SDH0SEL_PLL_DIV2, CLK_CLKDIV0_SDH0(5));
    /* Enable IP clock */
    CLK_EnableModuleClock(SDH0_MODULE);
}

void SYS_Init(void)
{
    /* Set PF multi-function pins for XT1_OUT(PF.2) and XT1_IN(PF.3) */
    SET_XT1_OUT_PF2();
    SET_XT1_IN_PF3();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC and HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HIRC and HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_HXTSTB_Msk);

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 180MHz */
    CLK_SetCoreClock(FREQ_180MHZ);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;
    CLK->AHBCLK1 |= CLK_AHBCLK1_GPICKEN_Msk | CLK_AHBCLK1_GPJCKEN_Msk;

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable I2S0 module clock */
    CLK_EnableModuleClock(I2S0_MODULE);

    /* Enable I2C2 module clock */
    CLK_EnableModuleClock(I2C2_MODULE);

    /* Enable PDMA0 module clock */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /* Enable HBI module clock */
    CLK_EnableModuleClock(HBI_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set multi-function pins for I2S0 */
    SET_I2S0_BCLK_PI6();
    SET_I2S0_MCLK_PI7();
    SET_I2S0_DI_PI8();
    SET_I2S0_DO_PI9();
    SET_I2S0_LRCK_PI10();

    /* Enable I2S0 clock pin (PI6) schmitt trigger */
    PI->SMTEN |= GPIO_SMTEN_SMTEN6_Msk;

    /* Set I2C2 multi-function pins */
    SET_I2C2_SDA_PD0();
    SET_I2C2_SCL_PD1();

    /* Enable I2C2 clock pin (PD1) schmitt trigger */
    PD->SMTEN |= GPIO_SMTEN_SMTEN1_Msk;

    /* Set multi-function pins for HBI */
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
}

void I2C2_Init(void)
{
    /* Open I2C2 and set clock to 100k */
    I2C_Open(I2C2, 100000);
}

/* Configure PDMA to Scatter Gather mode */
void PDMA_Init(void)
{
    DMA_DESC[0].ctl = ((PCM_BUFFER_SIZE - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    DMA_DESC[0].src = (uint32_t)&aPCMBuffer[0][0];
    DMA_DESC[0].dest = (uint32_t)&I2S0->TXFIFO;
    DMA_DESC[0].offset = (uint32_t)&DMA_DESC[1] - (PDMA0->SCATBA);

    DMA_DESC[1].ctl = ((PCM_BUFFER_SIZE - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    DMA_DESC[1].src = (uint32_t)&aPCMBuffer[1][0];
    DMA_DESC[1].dest = (uint32_t)&I2S0->TXFIFO;
    DMA_DESC[1].offset = (uint32_t)&DMA_DESC[0] - (PDMA0->SCATBA);

    PDMA_Open(PDMA0, 1 << 2);
    PDMA_SetTransferMode(PDMA0, 2, PDMA_I2S0_TX, 1, (uint32_t)&DMA_DESC[0]);

    PDMA_EnableInt(PDMA0, 2, 0);
    NVIC_EnableIRQ(PDMA0_IRQn);
}

void GPH_IRQHandler(void)
{
    volatile uint32_t u32Temp;

    /* To check if PH.0 interrupt occurred */
    if(GPIO_GET_INT_FLAG(PH, BIT0))
    {
        GPIO_CLR_INT_FLAG(PH, BIT0);

        u32SW2 = 0;
        u32RecordDone = 1;
        I2S_DISABLE_RX(I2S0);
    }
    else
    {
        /* Un-expected interrupt. Just clear all PH interrupts */
        u32Temp = PH->INTSRC;
        PH->INTSRC = u32Temp;
        printf("Un-expected interrupts.\n");
    }
}

static int32_t Clear4Bytes(uint32_t u32StartAddr)
{
    outp32(u32StartAddr, 0);
    return 0;
}

static int32_t ClearHyperRAM(uint32_t u32StartAddr, uint32_t u32EndAddr)
{
    uint32_t u32Data, i;

    for(i = u32StartAddr; i < u32EndAddr; i += 4)
    {
        if(Clear4Bytes(i) < 0)
        {
            return -1;
        }
        u32Data = inp32(i);
        if(u32Data != 0)
        {
            printf("ClearHyperRAM fail!! Read address:0x%08x  data::0x%08x  expect: 0\n",  i, u32Data);
            return -1;
        }
    }
    return 0;
}

int32_t main(void)
{
    TCHAR sd_path[] = { '0', ':', 0 };    /* SD drive started from 0 */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init SD */
    SD_Inits();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("+-----------------------------------------------------------------------+\n");
    printf("|                  MP3 Recorder Sample with audio codec                 |\n");
    printf("+-----------------------------------------------------------------------+\n");
    printf(" Please insert SD card \n");
    printf(" Press SW2 button to start recording and press again to stop recording \n");
    printf(" Press SW3 button to play MP3 files on SD card \n");

    /* Configure PH.0 and PH.1 as Output mode */
    GPIO_SetMode(PH, BIT0, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PH, BIT1, GPIO_MODE_OUTPUT);

    /* Configure FATFS */
    SDH_Open_Disk(SDH0, CardDetect_From_GPIO);
    f_chdrive(sd_path);          /* Set default path */

    /* Init I2C2 to access codec */
    I2C2_Init();

    /* Select source from HXT(12MHz) */
    CLK_SetModuleClock(I2S0_MODULE, CLK_CLKSEL3_I2S0SEL_HXT, 0);

    while(1)
    {
        u32SW2 = (PH->PIN & (1 << 0)) ? 0 : 1;
        u32SW3 = (PH->PIN & (1 << 1)) ? 0 : 1;

        if(SD0.IsCardInsert == TRUE)
        {
            if((u32SW2 == 1) && (u32StartRecord == 1) && (u32RecordDone == 0))
            {
                GPIO_EnableInt(PH, 0, GPIO_INT_FALLING);
                NVIC_EnableIRQ(GPH_IRQn);
                NVIC_SetPriority(GPH_IRQn, 3);

                if(u32StartRecord == 1)
                {
                    /* Clear HyperRAM */
                    if(ClearHyperRAM(HYPER_RAM_MEM_MAP, HYPER_RAM_MEM_MAP + 0x800000) < 0)
                        return -1;
                    Recorder_Init();
                    u32StartRecord = 0;
                }
                printf("Start recording ...\n\n");
                /* Record MP3 */
                MP3Recorder();
            }
            else
            {
                if(u32RecordDone == 1)
                {
                    GPIO_DisableInt(PH, 0);
                    u32StartRecord = 1;
                    u32StopRecord = 1;
                    Recorder_Uninit();
                    u32RecordDone = 0;
                }

                /* Play MP3 */
                if(u32SW3 == 1)
                {
                    MP3Player();
                }
            }
        }
    }
}
