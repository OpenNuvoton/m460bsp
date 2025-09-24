/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to trigger EADC by EPWM.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"


/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint32_t g_u32AdcIntFlag, g_u32COVNUMFlag = 0;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_FunctionTest(void);
void SYS_Init(void);
void UART0_Init(void);
void EPWM0_Init(void);
void EADC00_IRQHandler(void);



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

    /* Set EPWM0 timer clock prescaler */
    EPWM_SET_PRESCALER(EPWM0, 0, 10);

    /* Set up counter type */
    EPWM0->CTL1 &= ~EPWM_CTL1_CNTTYPE0_Msk;

    /* Set EPWM0 timer duty */
    EPWM_SET_CMR(EPWM0, 0, 1000);

    /* Set EPWM0 timer period */
    EPWM_SET_CNR(EPWM0, 0, 2000);

    /* EPWM period point trigger ADC enable */
    EPWM_EnableADCTrigger(EPWM0, 0, EPWM_TRG_ADC_EVEN_PERIOD);

    /* Set output level at zero, compare up, period(center) and compare down of specified channel */
    EPWM_SET_OUTPUT_LEVEL(EPWM0, BIT0, EPWM_OUTPUT_HIGH, EPWM_OUTPUT_LOW, EPWM_OUTPUT_NOTHING, EPWM_OUTPUT_NOTHING);

    /* Enable output of EPWM0 channel 0 */
    EPWM_EnableOutput(EPWM0, BIT0);

}

/*---------------------------------------------------------------------------------------------------------*/
/* EADC function test                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_FunctionTest(void)
{
    uint8_t  u8Option;
    int32_t  ai32ConversionData[6] = {0};
    uint32_t u32COVNUMFlag = 0;
    uint8_t u8Index = 0;
    uint32_t u32TimeOutCnt = 0;
    int32_t i32Err;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                       EPWM trigger mode test                         |\n");
    printf("+----------------------------------------------------------------------+\n");

    printf("\nIn this test, software will get 6 conversion result from the specified channel.\n");

    while(1)
    {
        printf("Select input mode:\n");
        printf("  [1] Single end input (channel 2 only)\n");
        printf("  [2] Differential input (channel pair 1 only)\n");
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
            EADC_ConfigSampleModule(EADC0, 0, EADC_EPWM0TG0_TRIGGER, 2);

            /* Clear the A/D ADINT0 interrupt flag for safe */
            EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);

            /* Enable the sample module 0 interrupt */
            EADC_ENABLE_INT(EADC0, BIT0);//Enable sample module A/D ADINT0 interrupt.
            EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, 0, BIT0);//Enable sample module 0 interrupt.
            NVIC_EnableIRQ(EADC00_IRQn);

            printf("Conversion result of channel 2:\n");

            /* Reset the EADC indicator and enable EPWM0 channel 0 counter */
            g_u32AdcIntFlag = 0;
            g_u32COVNUMFlag = 0;
            EPWM_Start(EPWM0, BIT0); //EPWM0 channel 0 counter start running.

            while(1)
            {
                /* Wait ADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
                u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
                while(g_u32AdcIntFlag == 0)
                {
                    if(--u32TimeOutCnt == 0)
                    {
                        printf("Wait for EADC interrupt time-out!\n");
                        return;
                    }
                }

                /* Reset the ADC interrupt indicator */
                g_u32AdcIntFlag = 0;

                /* Get the conversion result of the sample module 0 */
                u32COVNUMFlag = g_u32COVNUMFlag - 1;
                ai32ConversionData[u32COVNUMFlag] = EADC_GET_CONV_DATA(EADC0, 0);

                if(g_u32COVNUMFlag > 6)
                    break;
            }

            /* Disable EPWM0 channel 0 counter */
            EPWM_ForceStop(EPWM0, BIT0); //EPWM0 counter stop running.

            /* Disable sample module 0 interrupt */
            EADC_DISABLE_SAMPLE_MODULE_INT(EADC0, 0, BIT0);

            for(u8Index = 0; (u8Index) < 6; u8Index++)
                printf("                                0x%X (%d)\n", ai32ConversionData[u8Index], ai32ConversionData[u8Index]);

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
            EADC_ConfigSampleModule(EADC0, 0, EADC_EPWM0TG0_TRIGGER, 2);

            /* Clear the A/D ADINT0 interrupt flag for safe */
            EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);

            /* Enable sample module A/D ADINT0 interrupt. */
            EADC_ENABLE_INT(EADC0, BIT0);
            /* Enable sample module 0 interrupt. */
            EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, 0, BIT0);
            NVIC_EnableIRQ(EADC00_IRQn);

            printf("Conversion result of channel 2:\n");

            /* Reset the EADC indicator and enable EPWM0 channel 0 counter */
            g_u32AdcIntFlag = 0;
            g_u32COVNUMFlag = 0;
            /* EPWM0 channel 0 counter start running. */
            EPWM_Start(EPWM0, BIT0);

            while(1)
            {
                /* Wait ADC interrupt (g_u32AdcIntFlag will be set at IRQ_Handler function) */
                u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
                while(g_u32AdcIntFlag == 0)
                {
                    if(--u32TimeOutCnt == 0)
                    {
                        printf("Wait for EADC interrupt time-out!\n");
                        return;
                    }
                }

                /* Reset the ADC interrupt indicator */
                g_u32AdcIntFlag = 0;

                /* Get the conversion result of the sample module 0   */
                u32COVNUMFlag = g_u32COVNUMFlag - 1;
                ai32ConversionData[u32COVNUMFlag] = EADC_GET_CONV_DATA(EADC0, 0);

                if(g_u32COVNUMFlag > 6)
                    break;
            }

            /* Disable EPWM0 channel 0 counter */
            EPWM_ForceStop(EPWM0, BIT0); //EPWM0 counter stop running.

            /* Disable sample module 0 interrupt */
            EADC_DISABLE_SAMPLE_MODULE_INT(EADC0, 0, 0x1);

            for(u8Index = 0; (u8Index) < 6; u8Index++)
                printf("                                0x%X (%d)\n", ai32ConversionData[u8Index], ai32ConversionData[u8Index]);

        }
        else
            return ;

    }
}



/*---------------------------------------------------------------------------------------------------------*/
/* EADC interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void EADC00_IRQHandler(void)
{
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);      /* Clear the A/D ADINT0 interrupt flag */
    g_u32AdcIntFlag = 1;
    g_u32COVNUMFlag++;
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

    /* Init EPWM for EADC */
    EPWM0_Init();

    /*---------------------------------------------------------------------------------------------------------*/
    /* SAMPLE CODE                                                                                             */
    /*---------------------------------------------------------------------------------------------------------*/

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* EADC function test */
    EADC_FunctionTest();

    /* Reset EADC module */
    SYS_ResetModule(EADC0_RST);

    /* Reset EPWM0 module */
    SYS_ResetModule(EPWM0_RST);

    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC0_MODULE);

    /* Disable EPWM0 IP clock */
    CLK_DisableModuleClock(EPWM0_MODULE);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(EADC00_IRQn);

    printf("Exit EADC sample code\n");

    while(1);

}


/*** (C) COPYRIGHT 2021 Nuvoton Technology Corp. ***/
