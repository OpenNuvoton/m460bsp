/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to trigger multiple sample modules and got conversion results in order of priority.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Define global variables and constants                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint32_t g_u32EadcInt0Flag, g_u32EadcInt1Flag, g_u32EadcInt2Flag, g_u32EadcInt3Flag;

static uint32_t g_au32IntModule[4];    /* save the sample module number for ADINT0~3 */
static volatile uint32_t g_au32IntSequence[4];  /* save the interrupt sequence for ADINT0~3 */
static volatile uint32_t g_u32IntSequenceIndex;

/*---------------------------------------------------------------------------------------------------------*/
/* Define functions prototype                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_FunctionTest(void);
void SYS_Init(void);
void UART0_Init(void);
void EADC00_IRQHandler(void);
void EADC01_IRQHandler(void);
void EADC02_IRQHandler(void);
void EADC03_IRQHandler(void);


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

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/*---------------------------------------------------------------------------------------------------------*/
/* EADC function test                                                                                      */
/*---------------------------------------------------------------------------------------------------------*/
void EADC_FunctionTest(void)
{
    uint8_t  u8Option;
    int32_t  i32ConversionData, i;
    uint32_t u32TimeOutCnt;
    int32_t  i32Err;

    printf("\n");
    printf("+----------------------------------------------------------------------+\n");
    printf("|                      EADC Pending Priority sample code               |\n");
    printf("+----------------------------------------------------------------------+\n");

    /* Set the EADC and enable the A/D converter */
    i32Err = EADC_Open(EADC0, 0);

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

    while(1)
    {
        printf("Select test items:\n");
        printf("  [1] Assign interrupt ADINT0~3 to Sample Module 0~3\n");
        printf("  [2] Assign interrupt ADINT3~0 to Sample Module 0~3\n");
        printf("  Other keys: exit EADC test\n");
        u8Option = (uint8_t)getchar();

        if(u8Option == '1')
        {
            g_au32IntModule[0] = 0;  /* Assign ADINT0 to Sample module 0 */
            g_au32IntModule[1] = 1;  /* Assign ADINT1 to Sample module 1 */
            g_au32IntModule[2] = 2;  /* Assign ADINT2 to Sample module 2 */
            g_au32IntModule[3] = 3;  /* Assign ADINT3 to Sample module 3 */
        }
        else if(u8Option == '2')
        {
            g_au32IntModule[0] = 3;  /* Assign ADINT0 to Sample module 3 */
            g_au32IntModule[1] = 2;  /* Assign ADINT1 to Sample module 2 */
            g_au32IntModule[2] = 1;  /* Assign ADINT2 to Sample module 1 */
            g_au32IntModule[3] = 0;  /* Assign ADINT3 to Sample module 0 */
        }
        else
            break;  /* exit while loop */

        /* Configure the sample module for analog input channel and software trigger source. */
        EADC_ConfigSampleModule(EADC0, 0, EADC_SOFTWARE_TRIGGER, 6);
        EADC_ConfigSampleModule(EADC0, 1, EADC_SOFTWARE_TRIGGER, 7);
        EADC_ConfigSampleModule(EADC0, 2, EADC_SOFTWARE_TRIGGER, 8);
        EADC_ConfigSampleModule(EADC0, 3, EADC_SOFTWARE_TRIGGER, 9);

        /* Clear the A/D ADINTx interrupt flag for safe */
        EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk | EADC_STATUS2_ADIF1_Msk | EADC_STATUS2_ADIF2_Msk | EADC_STATUS2_ADIF3_Msk);

        /* Enable the sample module interrupt.  */
        EADC_ENABLE_INT(EADC0, BIT0 | BIT1 | BIT2 | BIT3);

        EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, 0, BIT0 << g_au32IntModule[0]);
        EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, 1, BIT0 << g_au32IntModule[1]);
        EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, 2, BIT0 << g_au32IntModule[2]);
        EADC_ENABLE_SAMPLE_MODULE_INT(EADC0, 3, BIT0 << g_au32IntModule[3]);

        NVIC_EnableIRQ(EADC00_IRQn);
        NVIC_EnableIRQ(EADC01_IRQn);
        NVIC_EnableIRQ(EADC02_IRQn);
        NVIC_EnableIRQ(EADC03_IRQn);

        /* Reset the EADC interrupt indicator and trigger sample module to start A/D conversion */
        g_u32IntSequenceIndex = 0;
        g_u32EadcInt0Flag = 0;
        g_u32EadcInt1Flag = 0;
        g_u32EadcInt2Flag = 0;
        g_u32EadcInt3Flag = 0;

        /* Start EADC conversion for sample module 0 ~ 3 at the same time */
        EADC_START_CONV(EADC0, BIT0 | BIT1 | BIT2 | BIT3);

        /* Wait all EADC interrupt (g_u32EadcIntxFlag will be set at EADC_INTx_IRQHandler() function) */
        u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
        while((g_u32EadcInt0Flag == 0) || (g_u32EadcInt1Flag == 0) ||
                (g_u32EadcInt2Flag == 0) || (g_u32EadcInt3Flag == 0))
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Wait for EADC interrupt time-out!\n");
                return;
            }
        }

        /* Get the conversion result of the sample module */
        printf("The ADINTx interrupt sequence is:\n");

        for(i = 0; i < 4; i++)
        {
            i32ConversionData = EADC_GET_CONV_DATA(EADC0, g_au32IntModule[i]);
            printf("ADINT%d: #%d, Module %d, Conversion result: 0x%X (%d)\n", i, g_au32IntSequence[i], g_au32IntModule[i], i32ConversionData, i32ConversionData);
        }

        printf("\n");

        /* Disable the ADINTx interrupt */
        EADC_DISABLE_INT(EADC0, BIT0 | BIT1 | BIT2 | BIT3);

        EADC_DISABLE_SAMPLE_MODULE_INT(EADC0, 0, BIT0 << g_au32IntModule[0]);
        EADC_DISABLE_SAMPLE_MODULE_INT(EADC0, 1, BIT0 << g_au32IntModule[1]);
        EADC_DISABLE_SAMPLE_MODULE_INT(EADC0, 2, BIT0 << g_au32IntModule[2]);
        EADC_DISABLE_SAMPLE_MODULE_INT(EADC0, 3, BIT0 << g_au32IntModule[3]);

        NVIC_DisableIRQ(EADC00_IRQn);
        NVIC_DisableIRQ(EADC01_IRQn);
        NVIC_DisableIRQ(EADC02_IRQn);
        NVIC_DisableIRQ(EADC03_IRQn);
    }   /* End of while(1) */

    /* Disable the A/D converter */
    EADC_Close(EADC0);
}



/*---------------------------------------------------------------------------------------------------------*/
/* EADC interrupt handler                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void EADC00_IRQHandler(void)
{
    g_u32EadcInt0Flag = 1;
    /* Clear the A/D ADINT0 interrupt flag */
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF0_Msk);

    /* Save the interrupt sequence about ADINT0 */
    g_au32IntSequence[0] = g_u32IntSequenceIndex++;
}

void EADC01_IRQHandler(void)
{
    g_u32EadcInt1Flag = 1;
    /* Clear the A/D ADINT1 interrupt flag */
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF1_Msk);

    /* Save the interrupt sequence about ADINT1 */
    g_au32IntSequence[1] = g_u32IntSequenceIndex++;
}

void EADC02_IRQHandler(void)
{
    g_u32EadcInt2Flag = 1;
    /* Clear the A/D ADINT2 interrupt flag */
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF2_Msk);

    /* Save the interrupt sequence about ADINT2 */
    g_au32IntSequence[2] = g_u32IntSequenceIndex++;
}

void EADC03_IRQHandler(void)
{
    g_u32EadcInt3Flag = 1;
    /* Clear the A/D ADINT3 interrupt flag */
    EADC_CLR_INT_FLAG(EADC0, EADC_STATUS2_ADIF3_Msk);

    /* Save the interrupt sequence about ADINT3 */
    g_au32IntSequence[3] = g_u32IntSequenceIndex++;
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

    printf("\nSystem clock rate: %d Hz", SystemCoreClock);

    /* EADC function test */
    EADC_FunctionTest();

    /* Disable EADC IP clock */
    CLK_DisableModuleClock(EADC0_MODULE);

    /* Reset EADC module */
    SYS_ResetModule(EADC0_RST);

    /* Disable External Interrupt */
    NVIC_DisableIRQ(EADC00_IRQn);
    NVIC_DisableIRQ(EADC01_IRQn);
    NVIC_DisableIRQ(EADC02_IRQn);
    NVIC_DisableIRQ(EADC03_IRQn);

    printf("Exit EADC sample code\n");

    while(1);

}

/*** (C) COPYRIGHT 2021 Nuvoton Technology Corp. ***/
