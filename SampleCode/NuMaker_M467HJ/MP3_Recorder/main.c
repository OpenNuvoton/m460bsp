/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    MP3 recorder sample encodes sound to MP3 format and stores it to
 *           a microSD card, and this MP3 file can also be played.
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

/*---------------------------------------------------------------------------*/
/* Global variables                                                          */
/*---------------------------------------------------------------------------*/
#ifdef __ICCARM__
#pragma data_alignment=32
DMA_DESC_T DMA_DESC[2] @0x20003000;
#else
DMA_DESC_T DMA_DESC[2] __attribute__((aligned(32)));
#endif

volatile uint32_t u32BTN0 = 0xF, u32BTN1 = 0xF;
volatile uint32_t g_u32RecordStart = 0, g_u32RecordDone = 0;
volatile uint32_t g_u32WriteSDToggle = 0;

extern shine_config_t config;
extern shine_t        s;
extern int32_t        samples_per_pass;
extern FIL            mp3FileObject;

/*---------------------------------------------------------------------------*/
/* Functions                                                                 */
/*---------------------------------------------------------------------------*/
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
    uint32_t volatile u32Isr;

    // FMI data abort interrupt
    if(SDH0->GINTSTS & SDH_GINTSTS_DTAIF_Msk)
    {
        /* ResetAllEngine() */
        SDH0->GCTL |= SDH_GCTL_GCTLRST_Msk;
    }

    //----- SD interrupt status
    u32Isr = SDH0->INTSTS;

    if(u32Isr & SDH_INTSTS_BLKDIF_Msk)
    {
        // block down
        SD0.DataReadyFlag = TRUE;
        SDH0->INTSTS = SDH_INTSTS_BLKDIF_Msk;
    }

    if(u32Isr & SDH_INTSTS_CDIF_Msk)    // port 0 card detect
    {
        //----- SD interrupt status
        // it is work to delay 50 times for SD_CLK = 200KHz
        {
            int volatile i;         // delay 30 fail, 50 OK

            for(i = 0; i < 0x500; i++);  // delay to make sure got updated value from REG_SDISR.

            u32Isr = SDH0->INTSTS;
        }

        if(u32Isr & SDH_INTSTS_CDSTS_Msk)
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
    if(u32Isr & SDH_INTSTS_CRCIF_Msk)
    {
        if(!(u32Isr & SDH_INTSTS_CRC16_Msk))
        {
            //printf("***** ISR sdioIntHandler(): CRC_16 error !\n");
            // handle CRC error
        }
        else if(!(u32Isr & SDH_INTSTS_CRC7_Msk))
        {
            if(!SD0.R3Flag)
            {
                //printf("***** ISR sdioIntHandler(): CRC_7 error !\n");
                // handle CRC error
            }
        }

        SDH0->INTSTS = SDH_INTSTS_CRCIF_Msk;      // clear interrupt flag
    }

    if(u32Isr & SDH_INTSTS_DITOIF_Msk)
    {
        printf("***** ISR: data in timeout !\n");
        SDH0->INTSTS |= SDH_INTSTS_DITOIF_Msk;
    }

    // Response in timeout interrupt
    if(u32Isr & SDH_INTSTS_RTOIF_Msk)
    {
        printf("***** ISR: response in timeout !\n");
        SDH0->INTSTS |= SDH_INTSTS_RTOIF_Msk;
    }
}

void SD_Init(void)
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
    CLK_SetModuleClock(SDH0_MODULE, CLK_CLKSEL0_SDH0SEL_PLL_DIV2, CLK_CLKDIV0_SDH0(2));
    /* Enable IP clock */
    CLK_EnableModuleClock(SDH0_MODULE);

    NVIC_SetPriority(SDH0_IRQn, 3);
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
    DMA_DESC[0].src = (uint32_t)&g_ai32PCMBuffer[0][0];
    DMA_DESC[0].dest = (uint32_t)&I2S0->TXFIFO;
    DMA_DESC[0].offset = (uint32_t)&DMA_DESC[1] - (PDMA0->SCATBA);

    DMA_DESC[1].ctl = ((PCM_BUFFER_SIZE - 1) << PDMA_DSCT_CTL_TXCNT_Pos) | PDMA_WIDTH_32 | PDMA_SAR_INC | PDMA_DAR_FIX | PDMA_REQ_SINGLE | PDMA_OP_SCATTER;
    DMA_DESC[1].src = (uint32_t)&g_ai32PCMBuffer[1][0];
    DMA_DESC[1].dest = (uint32_t)&I2S0->TXFIFO;
    DMA_DESC[1].offset = (uint32_t)&DMA_DESC[0] - (PDMA0->SCATBA);

    PDMA_Open(PDMA0, 1 << 2);
    PDMA_SetTransferMode(PDMA0, 2, PDMA_I2S0_TX, 1, (uint32_t)&DMA_DESC[0]);

    PDMA_EnableInt(PDMA0, 2, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA0_IRQn);
}

void GPH_IRQHandler(void)
{
    volatile uint32_t u32Temp;

    /* To check if PH.1 interrupt occurred */
    if(GPIO_GET_INT_FLAG(PH, BIT1))
    {
        GPIO_CLR_INT_FLAG(PH, BIT1);

        if(g_u32RecordStart == 1)
            g_u32RecordDone = 1;
    }
    else
    {
        /* Un-expected interrupt. Just clear all PH interrupts */
        u32Temp = PH->INTSRC;
        PH->INTSRC = u32Temp;
        printf("Un-expected interrupts.\n");
    }
}

#ifndef REC_IN_RT

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

#endif

int main(void)
{
    int32_t i32Written;
    uint8_t *pu8Data;
    uint32_t u32PrintFlag = 1;

    TCHAR sd_path[] = { '0', ':', 0 };    /* SD drive started from 0 */

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init SD */
    SD_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    printf("+-----------------------------------------------------------------------+\n");
    printf("|                  MP3 Recorder Sample with Audio Codec                 |\n");
    printf("+-----------------------------------------------------------------------+\n");
    printf(" Please insert a microSD card\n");
    printf(" Press BTN0 button to start recording and press BTN1 button to stop recording\n");
    printf(" Press BTN1 button can also play MP3 file from microSD card\n");

    /* Configure PH.0 and PH.1 as Output mode */
    GPIO_SetMode(PH, BIT0, GPIO_MODE_OUTPUT);
    GPIO_SetMode(PH, BIT1, GPIO_MODE_OUTPUT);

    /* Enable PH.1 interrupt by falling edge trigger */
    GPIO_EnableInt(PH, 1, GPIO_INT_FALLING);
    NVIC_EnableIRQ(GPH_IRQn);
    NVIC_SetPriority(GPH_IRQn, (1 << __NVIC_PRIO_BITS) - 2);

    /* Configure FATFS */
    SDH_Open_Disk(SDH0, CardDetect_From_GPIO);
    f_chdrive(sd_path);          /* Set default path */

    /* Init I2C2 to access audio codec */
    I2C2_Init();

    /* Select source from HXT(12MHz) */
    CLK_SetModuleClock(I2S0_MODULE, CLK_CLKSEL3_I2S0SEL_HXT, 0);

    while(1)
    {
        /* Read pin state of PH.0 and PH.1 */
        u32BTN0 = (PH->PIN & (1 << 0)) ? 0 : 1;
        u32BTN1 = (PH->PIN & (1 << 1)) ? 0 : 1;

        if(SD0.IsCardInsert == TRUE)
        {
#ifdef REC_IN_RT

            /* Inform users about microSD card usage */
            if((u32PrintFlag == 1) && (g_u32ErrorFlag != 0))
            {
                printf("\n\nSome sounds have been lost due to poor microSD card performance.\nPlease replace !!!\n\n");

                u32PrintFlag = 0;
                g_u32ErrorFlag = 0;
            }

            /* Encode sound data to MP3 format and store in microSD card */
            if(g_u32RecordStart == 1)
            {
                if((g_u32WriteSDToggle == 0) && (g_u32BuffPos1 > 0) && (g_u32BuffPos1 == ((samples_per_pass * config.wave.channels) >> 1)))
                {
                    pu8Data = shine_encode_buffer_interleaved(s, (int16_t *)(&g_au32PcmBuff1), (int *)&i32Written);

                    if(Write_MP3(i32Written, pu8Data, &config) != i32Written)
                    {
                        printf("shineenc: write error\n");
                    }

                    g_u32BuffPos1 = 0;
                    g_u32WriteSDToggle = 1;
                }
                else if((g_u32WriteSDToggle == 1) && (g_u32BuffPos2 > 0) && (g_u32BuffPos2 == ((samples_per_pass * config.wave.channels) >> 1)))
                {
                    pu8Data = shine_encode_buffer_interleaved(s, (int16_t *)(&g_au32PcmBuff2), (int *)&i32Written);

                    if(Write_MP3(i32Written, pu8Data, &config) != i32Written)
                    {
                        printf("shineenc: write error\n");
                    }

                    g_u32BuffPos2 = 0;
                    g_u32WriteSDToggle = 0;
                }
            }

#endif

            if((u32BTN0 == 1) && (g_u32RecordStart == 0) && (g_u32RecordDone == 0))
            {
#ifndef REC_IN_RT

                /* Clear HyperRAM */
                if(ClearHyperRAM(HYPERRAM_BASE, HYPERRAM_BASE + 0x800000) < 0)
                    return -1;

#endif

                /* Configure recording condition, init I2S, audio codec and encoder */
                Recorder_Init();

#ifdef REC_IN_RT

                printf("Start recording ... (online)\n");

#else

                printf("Start recording ... (offline)\n");

#endif

                /* Enable I2S RX function to receive sound data */
                I2S_ENABLE_RX(I2S0);

                g_u32RecordStart = 1;

#ifdef REC_IN_RT

                u32PrintFlag = 1;

                printf("Encode and write out the MP3 file ");

#endif
            }

            /* Play MP3 */
            if((u32BTN1 == 1) && (g_u32RecordStart == 0))
            {
                MP3Player();
            }

            if(g_u32RecordDone == 1)
            {
                /* Disable I2S RX function */
                I2S_DISABLE_RX(I2S0);

#ifndef REC_IN_RT

                /* Encode sound data to MP3 format and store in microSD card */
                MP3Recorder();

#endif

                /* Close encoder */
                shine_close(s);

                f_close(&mp3FileObject);

                printf(" Done !\n\n");

                g_u32RecordStart = 0;
                g_u32RecordDone = 0;

                /* Play MP3 */
                MP3Player();
            }
        }
    }
}
