/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Use packet format (all the luma and chroma data interleaved) to
 *           store captured image from NT99141 sensor to SRAM and
 *           encode the image to jpeg.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "sensor.h"



void CCAPInterruptHandler(void);
void CCAP_IRQHandler(void);
void CCAPSetFreq(uint32_t u32ModFreqKHz, uint32_t u32SensorFreq);
int32_t PacketFormatDownScale(void);
void UART0_Init(void);
void SYS_Init(void);
extern void JpegEncode(unsigned char* image, unsigned char* jBuf, unsigned long *jSize, int width, int height);



/*------------------------------------------------------------------------------------------*/
/* To run CCAPInterrupt Handler, when CCAP frame end interrupt                              */
/*------------------------------------------------------------------------------------------*/
volatile uint32_t u32FramePass = 0;

void CCAPInterruptHandler(void)
{
    u32FramePass++;
}

void CCAP_IRQHandler(void)
{
    uint32_t u32CapInt;
    u32CapInt = CCAP->INT;
    if((u32CapInt & (CCAP_INT_VIEN_Msk | CCAP_INT_VINTF_Msk)) == (CCAP_INT_VIEN_Msk | CCAP_INT_VINTF_Msk))
    {
        CCAPInterruptHandler();
        CCAP->INT |= CCAP_INT_VINTF_Msk;     /* Clear Frame end interrupt */
    }

    if((u32CapInt & (CCAP_INT_ADDRMIEN_Msk | CCAP_INT_ADDRMINTF_Msk)) == (CCAP_INT_ADDRMIEN_Msk | CCAP_INT_ADDRMINTF_Msk))
    {
        CCAP->INT |= CCAP_INT_ADDRMINTF_Msk; /* Clear Address match interrupt */
    }

    if((u32CapInt & (CCAP_INT_MEIEN_Msk | CCAP_INT_MEINTF_Msk)) == (CCAP_INT_MEIEN_Msk | CCAP_INT_MEINTF_Msk))
    {
        CCAP->INT |= CCAP_INT_MEINTF_Msk;    /* Clear Memory error interrupt */
    }
    CCAP->CTL = CCAP->CTL | CCAP_CTL_UPDATE;
}

void CCAPSetFreq(uint32_t u32ModFreqKHz, uint32_t u32SensorFreq)
{
    int32_t i32Div;

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Enable CAP Clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_CCAPCKEN_Msk;

    /* Enable Sensor Clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_SENCKEN_Msk;

    /* Reset IP */
    SYS_ResetModule(CCAP_RST);

    /* Specified sensor clock */
    CLK->CLKSEL0 = (CLK->CLKSEL0 & ~CLK_CLKSEL0_CCAPSEL_Msk) | CLK_CLKSEL0_CCAPSEL_HCLK ;
    i32Div = CLK_GetHCLKFreq() / u32SensorFreq - 1;
    if(i32Div < 0) i32Div = 0;
    CLK->CLKDIV3 = (CLK->CLKDIV3 & ~CLK_CLKDIV3_VSENSEDIV_Msk) | i32Div << CLK_CLKDIV3_VSENSEDIV_Pos;

    /* Lock protected registers */
    SYS_LockReg();
}

#define SENSOR_IN_WIDTH             640
#define SENSOR_IN_HEIGHT            480
#define SYSTEM_WIDTH                160
#define SYSTEM_HEIGHT               120
uint8_t u8FrameBuffer[SYSTEM_WIDTH * SYSTEM_HEIGHT + SYSTEM_WIDTH];

#define DataFormatAndOrder (CCAP_PAR_INDATORD_YUYV | CCAP_PAR_INFMT_YUV422 | CCAP_PAR_OUTFMT_ONLY_Y)

int32_t PacketFormatDownScale(void)
{
    uint32_t u32Frame;

    /* Initialize NT99141 sensor and set NT99141 output YUV422 format */
    if(InitNT99141_VGA_YUV422() == FALSE) return -1;

    /* Enable External CCAP Interrupt */
    NVIC_EnableIRQ(CCAP_IRQn);

    /* Enable External CCAP Interrupt */
    CCAP_EnableInt(CCAP_INT_VIEN_Msk);

    /* Set Vsync polarity, Hsync polarity, pixel clock polarity, Sensor Format and Order */
    CCAP_Open(NT99141SensorPolarity | DataFormatAndOrder, CCAP_CTL_PKTEN);

    /* Set Cropping Window Vertical/Horizontal Starting Address and Cropping Window Size */
    CCAP_SetCroppingWindow(0, 0, SENSOR_IN_HEIGHT, SENSOR_IN_WIDTH);

    /* Set System Memory Packet Base Address Register */
    CCAP_SetPacketBuf((uint32_t)u8FrameBuffer);

    /* Set Packet Scaling Vertical/Horizontal Factor Register */
    CCAP_SetPacketScaling(SYSTEM_HEIGHT, SENSOR_IN_HEIGHT, SYSTEM_WIDTH, SENSOR_IN_WIDTH);

    /* Set Packet Frame Output Pixel Stride Width */
    CCAP_SetPacketStride(SYSTEM_WIDTH);

    /* Start Image Capture Interface */
    CCAP_Start();

    u32Frame = u32FramePass;
    while(1)
    {
        if(u32FramePass != (u32Frame))
        {
            u32Frame = u32FramePass;
            printf("u32FramePass=%d\n", u32FramePass);
        }
        if(u32FramePass > 10)
            break;
    }

    /* Stop capturing images */
    CCAP_Stop(TRUE);
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

    /* Set multi-function pins for CCAP */
    SET_CCAP_DATA7_PG2();
    SET_CCAP_DATA6_PG3();
    SET_CCAP_DATA5_PG4();
    SET_CCAP_DATA4_PF11();
    SET_CCAP_DATA3_PF10();
    SET_CCAP_DATA2_PF9();
    SET_CCAP_DATA1_PF8();
    SET_CCAP_DATA0_PF7();
    SET_CCAP_PIXCLK_PG9();
    SET_CCAP_SCLK_PG10();
    SET_CCAP_VSYNC_PG12();
    SET_CCAP_HSYNC_PG13();

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
uint8_t u8JpegBuffer[SYSTEM_WIDTH * SYSTEM_HEIGHT];

int main(void)
{
    unsigned long u32JpegSize = sizeof(u8JpegBuffer);

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Lock protected registers */
    SYS_LockReg();

    /* Init UART0 for printf */
    UART0_Init();

    /* Init CCAP clock and Sensor clock */
    CCAPSetFreq(12000000, 12000000);

    /* Using Packet format to Image down scale */
    if(PacketFormatDownScale() >= 0)
    {
        /* jpeg encode */
        JpegEncode(u8FrameBuffer, u8JpegBuffer, &u32JpegSize, SYSTEM_WIDTH, SYSTEM_HEIGHT);
    }

    /* Forces a write of all user-space buffered data for the given output */
    fflush(stdout);

    while(1);
}
