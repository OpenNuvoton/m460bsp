/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to trigger EADC by EPWM and transfer conversion data by PDMA.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint32_t g_u32AdcIntFlag;
static volatile uint32_t g_u32IsTestOver = 0;
static int16_t  g_ai16ConversionData[6] = {0};
static uint32_t g_u32SampleModuleNum = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_FunctionTest(void);
void EADC00_IRQHandler(void);
void SYS_Init(void);
void UART0_Init(void);
void EPWM0_Init(void);
void PDMA_Init(void);
void ReloadPDMA(void);
void PDMA0_IRQHandler(void);
/*---------------------------------------------------------------------------------------------------------*/
/* EADC interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void EADC00_IRQHandler(void)
{
    g_u32AdcIntFlag = 1;
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);      /* Clear the A/D ADINT0 interrupt flag */
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

    /* Enable EADC0 module clock */
    CLK_EnableModuleClock(EADC0_MODULE);

    /* Set EADC0 clock divider as 12 */
    CLK_SetModuleClock(EADC0_MODULE, CLK_CLKSEL0_EADC0SEL_PLL_DIV2, CLK_CLKDIV0_EADC0(12));

    /* Enable EPWM0 module clock */
    CLK_EnableModuleClock(EPWM0_MODULE);

    /* Select EPWM0 module clock source as PCLK0 */
    CLK_SetModuleClock(EPWM0_MODULE, CLK_CLKSEL2_EPWM0SEL_PCLK0, 0);

    /* Enable PDMA0 clock source */
    CLK_EnableModuleClock(PDMA0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set multi-function pins for EADC0 channels. */
    SET_EADC0_CH0_PB0();
    SET_EADC0_CH1_PB1();
    SET_EADC0_CH2_PB2();
    SET_EADC0_CH3_PB3();

    /* Disable digital input path of EADC analog pin to prevent leakage */
    GPIO_DISABLE_DIGITAL_PATH(PB, BIT3 | BIT2 | BIT1 | BIT0);

    /* Set multi-function pins for EPWM */
    SET_EPWM0_CH0_PE7();

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

void EPWM0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init EPWM0                                                                                              */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset EPWM0 module */
    SYS_ResetModule(EPWM0_RST);

    /* Set EPWM0 timer clock prescaler */
    EPWM_SET_PRESCALER(EPWM0, 0, 0);

    /* Set up counter type */
    EPWM0->CTL1 &= ~EPWM_CTL1_CNTTYPE0_Msk;

    /* Set EPWM0 timer duty */
    EPWM_SET_CMR(EPWM0, 0, 108);

    /* Set EPWM0 timer period */
    EPWM_SET_CNR(EPWM0, 0, 216);

    /* EPWM period point trigger ADC enable */
    EPWM_EnableADCTrigger(EPWM0, 0, EPWM_TRG_ADC_EVEN_PERIOD);

    /* Set output level at zero, compare up, period(center) and compare down of specified channel */
    EPWM_SET_OUTPUT_LEVEL(EPWM0, BIT0, EPWM_OUTPUT_HIGH, EPWM_OUTPUT_LOW, EPWM_OUTPUT_NOTHING, EPWM_OUTPUT_NOTHING);

    /* Enable output of EPWM0 channel 0 */
    EPWM_EnableOutput(EPWM0, BIT0);
}

void PDMA_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init PDMA                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset PDMA module */
    SYS_ResetModule(PDMA0_RST);

    /* Configure PDMA peripheral mode form EADC to memory */
    /* Open Channel 2 */
    PDMA_Open(PDMA0, BIT2);

    /* transfer width is half word(16 bit) and transfer count is 6 */
    PDMA_SetTransferCnt(PDMA0, 2, PDMA_WIDTH_16, 6);

    /* Set source address as EADC data register(no increment) and destination address as g_ai16ConversionData array(increment) */
    PDMA_SetTransferAddr(PDMA0, 2, (uint32_t)&EADC0->DAT[g_u32SampleModuleNum], PDMA_SAR_FIX, (uint32_t)g_ai16ConversionData, PDMA_DAR_INC);

    /* Select PDMA request source as ADC RX */
    PDMA_SetTransferMode(PDMA0, 2, PDMA_EADC0_RX, FALSE, 0);

    /* Set PDMA as single request type for EADC */
    PDMA_SetBurstType(PDMA0, 2, PDMA_REQ_SINGLE, PDMA_BURST_4);

    PDMA_EnableInt(PDMA0, 2, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA0_IRQn);
}

void ReloadPDMA(void)
{
    /* Transfer width is half word(16 bit) and transfer count is 6 */
    PDMA_SetTransferCnt(PDMA0, 2, PDMA_WIDTH_16, 6);

    /* Select PDMA request source as ADC RX */
    PDMA_SetTransferMode(PDMA0, 2, PDMA_EADC0_RX, FALSE, 0);
}

/*---------------------------------------------------------------------------------------------------------*/
/* EADC function test                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_FunctionTest(void)
{
    uint8_t  u8Option;
    uint8_t  u8Index = 0;
    uint32_t u32TimeOutCnt = 0;
    int32_t  i32Err;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|     EPWM trigger mode and transfer conversion data by PDMA test      |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("\nIn this test, software will get 6 conversion result from the specified channel.\n");

    while(1)
    {
        /* Reload PDMA configuration for next transmission */
        ReloadPDMA();

        printf("Select input mode:\n");
        printf("  [1] Single end input (channel 2 only)\n");
        printf("  [2] Differential input (channel pair 1 only(channel 2 and 3))\n");
        printf("  Other keys: exit single mode test\n");
        u8Option = (uint8_t)getchar();
        if(u8Option == '1')
        {
            /* Set input mode as single-end and enable the A/D converter */
            i32Err = EADC_Open(EADC0, EADC_CTL_DIFFEN_SINGLE_END);

            /* Check EADC global error code. */
            if(i32Err != 0)
            {
                if(i32Err == EADC_CAL_ERR)
                {
                    printf("EADC has calibration error.\n");
                    return;
                }
                else if(i32Err == EADC_CLKDIV_ERR)
                {
                    printf("EADC clock frequency is configured error.\n");
                    return;
                }
                else
                {
                    printf("EADC has operation error.\n");
                    return;
                }
            }

            /* Configure the sample module 0 for analog input channel 2 and enable EPWM0 trigger source */
            EADC_ConfigSampleModule(EADC0, g_u32SampleModuleNum, EADC_EPWM0TG0_TRIGGER, 2);
            EADC_ENABLE_SAMPLE_MODULE_PDMA(EADC0, 1 << g_u32SampleModuleNum);

            printf("Conversion result of channel 2:\n");

            /* Enable EPWM0 channel 0 counter */
            EPWM_Start(EPWM0, BIT0); /* EPWM0 channel 0 counter start running. */

            /* Wait PDMA interrupt (g_u32IsTestOver will be set at IRQ_Handler function) */
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
            while(g_u32IsTestOver == 0)
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for PDMA interrupt time-out!\n");
                    return;
                }
            }
            g_u32IsTestOver = 0;

            /* Disable EPWM0 channel 0 counter */
            EPWM_ForceStop(EPWM0, BIT0); /* EPWM0 counter stop running. */

            for(u8Index = 0; (u8Index) < 6; u8Index++)
                printf("                                0x%X (%d)\n", g_ai16ConversionData[u8Index], g_ai16ConversionData[u8Index]);

        }
        else if(u8Option == '2')
        {
            /* Set input mode as differential and enable the A/D converter */
            i32Err = EADC_Open(EADC0, EADC_CTL_DIFFEN_DIFFERENTIAL);

            /* Check EADC global error code. */
            if(i32Err != 0)
            {
                if(i32Err == EADC_CAL_ERR)
                {
                    printf("EADC has calibration error.\n");
                    return;
                }
                else if(i32Err == EADC_CLKDIV_ERR)
                {
                    printf("EADC clock frequency is configured error.\n");
                    return;
                }
                else
                {
                    printf("EADC has operation error.\n");
                    return;
                }
            }

            /* Configure the sample module 0 for analog input channel 2 and software trigger source.*/
            EADC_ConfigSampleModule(EADC0, g_u32SampleModuleNum, EADC_EPWM0TG0_TRIGGER, 2);
            EADC_ENABLE_SAMPLE_MODULE_PDMA(EADC0, 1 << g_u32SampleModuleNum);

            printf("Conversion result of channel 2:\n");

            /* Enable EPWM0 channel 0 counter */
            EPWM_Start(EPWM0, BIT0); /* EPWM0 channel 0 counter start running. */

            /* Wait PDMA interrupt (g_u32IsTestOver will be set at IRQ_Handler function) */
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
            while(g_u32IsTestOver == 0)
            {
                if(--u32TimeOutCnt == 0)
                {
                    printf("Wait for PDMA interrupt time-out!\n");
                    return;
                }
            }
            g_u32IsTestOver = 0;

            /* Disable EPWM0 channel 0 counter */
            EPWM_ForceStop(EPWM0, BIT0); /* EPWM0 counter stop running. */

            for(u8Index = 0; (u8Index) < 6; u8Index++)
                printf("                                0x%X (%d)\n", g_ai16ConversionData[u8Index], g_ai16ConversionData[u8Index]);

        }
        else
            return ;

        EADC_Close(EADC0);
        /* Reset EADC module */
        SYS_ResetModule(EADC0_RST);
    }
}


/*---------------------------------------------------------------------------------------------------------*/
/* PDMA interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void PDMA0_IRQHandler(void)
{
    uint32_t status = PDMA_GET_INT_STATUS(PDMA0);

    if(status & PDMA_INTSTS_ABTIF_Msk)    /* abort */
    {
        if(PDMA_GET_ABORT_STS(PDMA0) & PDMA_ABTSTS_ABTIF2_Msk)
            g_u32IsTestOver = 2;
        PDMA_CLR_ABORT_FLAG(PDMA0, PDMA_ABTSTS_ABTIF2_Msk);
    }
    else if(status & PDMA_INTSTS_TDIF_Msk)      /* done */
    {
        if(PDMA_GET_TD_STS(PDMA0) & PDMA_TDSTS_TDIF2_Msk)
            g_u32IsTestOver = 1;
        PDMA_CLR_TD_FLAG(PDMA0, PDMA_TDSTS_TDIF2_Msk);
    }
    else
        printf("unknown interrupt !!\n");
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init EPWM for EADC */
    EPWM0_Init();

    /* Init PDMA for EADC */
    PDMA_Init();

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* Reset EADC module */
    SYS_ResetModule(EADC0_RST);

    /* EADC function test */
    EADC_FunctionTest();

    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC0_MODULE);

    /* Disable EPWM0 IP clock */
    CLK_DisableModuleClock(EPWM0_MODULE);

    /* Disable PDMA clock source */
    CLK_DisableModuleClock(PDMA0_MODULE);

    /* Disable PDMA Interrupt */
    NVIC_DisableIRQ(PDMA0_IRQn);

    printf("Exit EADC sample code\n");

    while(1);
}


/*** (C) COPYRIGHT 2021 Nuvoton Technology Corp. ***/
