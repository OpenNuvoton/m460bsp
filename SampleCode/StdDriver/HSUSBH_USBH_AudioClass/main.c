/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrates how to use USBH Audio Class driver. It shows the
 *           mute, volume, auto-gain, channel, and sampling rate control.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"

#include "usbh_lib.h"
#include "usbh_uac.h"

#define AUDIO_IN_BUFSIZ             8192

#ifdef DEBUG_ENABLE_SEMIHOST
#error This sample cannot execute with semihost enabled
#endif

static volatile int s_i8AuInCnt, s_i8AuOutCnt;

static uint16_t s_u16VolMax, s_u16VolMin, s_u16VolRes, s_u16VolCur;

extern int kbhit(void);                        /* function in retarget.c                 */

static volatile uint32_t s_u32TickCnt;

void SysTick_Handler(void);
void enable_sys_tick(int ticks_per_second);
void SYS_Init(void);
void UART0_Init(void);
int audio_in_callback(UAC_DEV_T *dev, uint8_t *pu8Data, int i8Len);
int audio_out_callback(UAC_DEV_T *dev, uint8_t *pu8Data, int i8Len);
void uac_control_example(UAC_DEV_T *uac_dev);

void SysTick_Handler(void)
{
    s_u32TickCnt++;
}

void enable_sys_tick(int ticks_per_second)
{
    s_u32TickCnt = 0;
    if(SysTick_Config(SystemCoreClock / (uint32_t)ticks_per_second))
    {
        /* Setup SysTick Timer for 1 second interrupts  */
        printf("Set system tick error!!\n");
        while(1);
    }
}

uint32_t get_ticks(void)
{
    return s_u32TickCnt;
}

/*
 *  This function is necessary for USB Host library.
 */
void delay_us(int usec)
{
    /*
     *  Configure Timer0, clock source from XTL_12M. Prescale 12
     */
    /* TIMER0 clock from HXT */
    CLK->CLKSEL1 = (CLK->CLKSEL1 & (~CLK_CLKSEL1_TMR0SEL_Msk)) | CLK_CLKSEL1_TMR0SEL_HXT;
    CLK->APBCLK0 |= CLK_APBCLK0_TMR0CKEN_Msk;
    TIMER0->CTL = 0;        /* disable timer */
    TIMER0->INTSTS = (TIMER_INTSTS_TIF_Msk | TIMER_INTSTS_TWKF_Msk);   /* write 1 to clear for safety */
    TIMER0->CMP = (uint32_t)usec;
    TIMER0->CTL = (11 << TIMER_CTL_PSC_Pos) | TIMER_ONESHOT_MODE | TIMER_CTL_CNTEN_Msk;

    while(!TIMER0->INTSTS);
}

void SYS_Init(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC and HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk | CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HIRC and HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk | CLK_STATUS_HXTSTB_Msk);

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 192MHz */
    CLK_SetCoreClock(FREQ_192MHZ);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;
    CLK->AHBCLK1 |= CLK_AHBCLK1_GPICKEN_Msk | CLK_AHBCLK1_GPJCKEN_Msk;

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Enable USBH module clock */
    CLK_EnableModuleClock(USBH_MODULE);

    /* USB Host desired input clock is 48 MHz. */
    /* Select USB module clock source as PLL/2 and USB module clock divider as 2 */
    CLK_SetModuleClock(USBH_MODULE, CLK_CLKSEL0_USBSEL_PLL_DIV2, CLK_CLKDIV0_USB(2));

    /* Set OTG as USB Host role */
    SYS->USBPHY = SYS_USBPHY_HSUSBEN_Msk | (0x1 << SYS_USBPHY_HSUSBROLE_Pos) | SYS_USBPHY_USBEN_Msk | SYS_USBPHY_SBO_Msk | (0x1 << SYS_USBPHY_USBROLE_Pos);
    delay_us(20);
    SYS->USBPHY |= SYS_USBPHY_HSUSBACT_Msk;

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* USB_VBUS_EN (USB 1.1 VBUS power enable pin) multi-function pin - PB.15     */
    SET_USB_VBUS_EN_PB15();

    /* USB_VBUS_ST (USB 1.1 over-current detect pin) multi-function pin - PC.14   */
    SET_USB_VBUS_ST_PC14();

    /* HSUSB_VBUS_EN (USB 2.0 VBUS power enable pin) multi-function pin - PJ.13   */
    SET_HSUSB_VBUS_EN_PJ13();

    /* HSUSB_VBUS_ST (USB 2.0 over-current detect pin) multi-function pin - PJ.12 */
    SET_HSUSB_VBUS_ST_PJ12();

    /* USB 1.1 port multi-function pin VBUS, D+, D-, and ID pins */
    SET_USB_VBUS_PA12();
    SET_USB_D_N_PA13();
    SET_USB_D_P_PA14();
    SET_USB_OTG_ID_PA15();

    /* Lock protected registers */
    SYS_LockReg();
}

void UART0_Init(void)
{
    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

/**
 *  @brief  Audio-in data callback function.
 *          UAC driver notify user that audio-in data has been moved into user audio-in buffer,
 *          which is provided by user application via UAC_InstallIsoInCbFun().
 *  @param[in] dev    Audio Class device
 *  @param[in] pu8Data   Available audio-in data, which is located in user audio-in buffer.
 *  @param[in] i8Len    Length of available audio-in data started from <pu8Data>.
 *  @return   UAC driver does not check this return value.
 */
int audio_in_callback(UAC_DEV_T *dev, uint8_t *pu8Data, int i8Len)
{
    (void)dev;
    (void)pu8Data;

    s_i8AuInCnt += i8Len;
    //printf("I %x,%x\n", (int)pu8Data & 0xffff, i8Len);   // UART send too many will cause ISO transfer time overrun

    // Add your code here to get audio-in data ...
    // For example, memcpy(audio_record_buffer, pu8Data, i8Len);
    // . . .

    return 0;
}

/**
 *  @brief  Audio-out data callback function.
 *          UAC driver requests user to move audio-out data into the specified address. The audio-out
 *          data will then be send to UAC device via isochronous-out pipe.
 *  @param[in] dev    Audio Class device
 *  @param[in] pu8Data   Application should move audio-out data into this buffer.
 *  @param[in] i8Len    Maximum length of audio-out data can be moved.
 *  @return   Actual length of audio data moved.
 */
int audio_out_callback(UAC_DEV_T *dev, uint8_t *pu8Data, int i8Len)
{
    (void)dev;
    (void)pu8Data;

    s_i8AuOutCnt += i8Len;
    //printf("O %x,%x\n", (int)pu8Data & 0xffff, i8Len);   // UART send too many will cause ISO transfer time overrun

    // Add your code here to put audio-out data ...
    // For example, memcpy(pu8Data, playback_buffer, actual_len);
    //              return actual_len;
    // . . .

    return 192;   // for 48000 stereo Hz
}

void uac_control_example(UAC_DEV_T *uac_dev)
{
    uint16_t u16Val;
    uint32_t au32SRate[4];
    uint8_t u8Val;
    uint8_t au8Data[8];
    int i8Cnt, i8Ret;
    uint32_t u32Val;

    s_u16VolMax = s_u16VolMin = s_u16VolRes = 0;

    printf("\nGet channel information ===>\n");

    /*-------------------------------------------------------------*/
    /*  Get channel number information                             */
    /*-------------------------------------------------------------*/
    i8Ret = usbh_uac_get_channel_number(uac_dev, UAC_SPEAKER);
    if(i8Ret < 0)
        printf("    Failed to get speaker's channel number.\n");
    else
        printf("    Speaker: %d\n", i8Ret);

    i8Ret = usbh_uac_get_channel_number(uac_dev, UAC_MICROPHONE);
    if(i8Ret < 0)
        printf("    Failed to get microphone's channel number.\n");
    else
    {
        printf("    Microphone: %d\n", i8Ret);
    }

    printf("\nGet subframe bit resolution ===>\n");

    /*-------------------------------------------------------------*/
    /*  Get audio subframe bit resolution information              */
    /*-------------------------------------------------------------*/
    i8Ret = usbh_uac_get_bit_resolution(uac_dev, UAC_SPEAKER, &u8Val);
    if(i8Ret < 0)
        printf("    Failed to get speaker's bit resoltion.\n");
    else
    {
        printf("    Speaker audio subframe size: %d bytes\n", u8Val);
        printf("    Speaker subframe bit resolution: %d\n", i8Ret);
    }

    i8Ret = usbh_uac_get_bit_resolution(uac_dev, UAC_MICROPHONE, &u8Val);
    if(i8Ret < 0)
        printf("    Failed to get microphone's bit resoltion.\n");
    else
    {
        printf("    Microphone audio subframe size: %d bytes\n", u8Val);
        printf("    Microphone subframe bit resolution: %d\n", i8Ret);
    }

    printf("\nGet sampling rate list ===>\n");

    /*-------------------------------------------------------------*/
    /*  Get audio subframe bit resolution information              */
    /*-------------------------------------------------------------*/
    i8Ret = usbh_uac_get_sampling_rate(uac_dev, UAC_SPEAKER, (uint32_t *)&au32SRate[0], 4, &u8Val);
    if(i8Ret < 0)
        printf("    Failed to get speaker's sampling rate.\n");
    else
    {
        if(u8Val == 0)
            printf("    Speaker sampling rate range: %d ~ %d Hz\n", au32SRate[0], au32SRate[1]);
        else
        {
            for(i8Cnt = 0; i8Cnt < u8Val; i8Cnt++)
                printf("    Speaker sampling rate: %d\n", au32SRate[i8Cnt]);
        }
    }

    i8Ret = usbh_uac_get_sampling_rate(uac_dev, UAC_MICROPHONE, (uint32_t *)&au32SRate[0], 4, &u8Val);
    if(i8Ret < 0)
        printf("    Failed to get microphone's sampling rate.\n");
    else
    {
        if(u8Val == 0)
            printf("    Microphone sampling rate range: %d ~ %d Hz\n", au32SRate[0], au32SRate[1]);
        else
        {
            for(i8Cnt = 0; i8Cnt < u8Val; i8Cnt++)
                printf("    Microphone sampling rate: %d\n", au32SRate[i8Cnt]);
        }
    }

    printf("\nSpeaker mute control ===>\n");

    /*-------------------------------------------------------------*/
    /*  Get current mute value of UAC device's speaker.            */
    /*-------------------------------------------------------------*/
    if(usbh_uac_mute_control(uac_dev, UAC_SPEAKER, UAC_GET_CUR, UAC_CH_MASTER, au8Data) == UAC_RET_OK)
    {
        printf("    Speaker mute state is %d.\n", au8Data[0]);
    }
    else
        printf("    Failed to get speaker mute state!\n");

    printf("\nSpeaker L(F) volume control ===>\n");

#if 0
    /*--------------------------------------------------------------------------*/
    /*  Get current volume value of UAC device's speaker left channel.          */
    /*--------------------------------------------------------------------------*/
    if(usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_CUR, UAC_CH_LEFT_FRONT, &u16Val) == UAC_RET_OK)
        printf("    Speaker L(F) volume is 0x%x.\n", u16Val);
    else
        printf("    Failed to get seaker L(F) volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get minimum volume value of UAC device's speaker left channel.          */
    /*--------------------------------------------------------------------------*/
    if(usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_MIN, UAC_CH_LEFT_FRONT, &u16Val) == UAC_RET_OK)
        printf("    Speaker L(F) minimum volume is 0x%x.\n", u16Val);
    else
        printf("    Failed to get speaker L(F) minimum volume!\n");
    /*--------------------------------------------------------------------------*/
    /*  Get maximum volume value of UAC device's speaker left channel.          */
    /*--------------------------------------------------------------------------*/
    if(usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_MAX, UAC_CH_LEFT_FRONT, &u16Val) == UAC_RET_OK)
        printf("    Speaker L(F) maximum volume is 0x%x.\n", u16Val);
    else
        printf("    Failed to get speaker L(F) maximum volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get volume resolution of UAC device's speaker left channel.             */
    /*--------------------------------------------------------------------------*/
    if(usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_RES, UAC_CH_LEFT_FRONT, &u16Val) == UAC_RET_OK)
        printf("    Speaker L(F) volume resolution is 0x%x.\n", u16Val);
    else
        printf("    Failed to get speaker L(F) volume resolution!\n");

    printf("\nSpeaker R(F) volume control ===>\n");

    /*--------------------------------------------------------------------------*/
    /*  Get current volume value of UAC device's speaker right channel.         */
    /*--------------------------------------------------------------------------*/
    if(usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_CUR, UAC_CH_RIGHT_FRONT, &u16Val) == UAC_RET_OK)
        printf("    Speaker R(F) volume is 0x%x.\n", u16Val);
    else
        printf("    Failed to get speaker R(F) volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get minimum volume value of UAC device's speaker right channel.         */
    /*--------------------------------------------------------------------------*/
    if(usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_MIN, UAC_CH_RIGHT_FRONT, &u16Val) == UAC_RET_OK)
        printf("    Speaker R(F) minimum volume is 0x%x.\n", u16Val);
    else
        printf("    Failed to get speaker R(F) minimum volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get maximum volume value of UAC device's speaker right channel.         */
    /*--------------------------------------------------------------------------*/
    if(usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_MAX, UAC_CH_RIGHT_FRONT, &u16Val) == UAC_RET_OK)
        printf("    Speaker R(F) maximum volume is 0x%x.\n", u16Val);
    else
        printf("    Failed to get speaker R(F) maximum volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get volume resolution of UAC device's speaker right channel.            */
    /*--------------------------------------------------------------------------*/
    if(usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_RES, UAC_CH_RIGHT_FRONT, &u16Val) == UAC_RET_OK)
        printf("    Speaker R(F) volume resolution is 0x%x.\n", u16Val);
    else
        printf("    Failed to get speaker R(F) volume resolution!\n");
#endif

    printf("\nSpeaker master volume control ===>\n");

    /*--------------------------------------------------------------------------*/
    /*  Get minimum volume value of UAC device's speaker master channel.        */
    /*--------------------------------------------------------------------------*/
    if(usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_MIN, UAC_CH_MASTER, &u16Val) == UAC_RET_OK)
        printf("    Speaker minimum master volume is 0x%x.\n", u16Val);
    else
        printf("    Failed to get speaker master minimum volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get maximum volume value of UAC device's speaker master channel.        */
    /*--------------------------------------------------------------------------*/
    if(usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_MAX, UAC_CH_MASTER, &u16Val) == UAC_RET_OK)
        printf("    Speaker maximum master volume is 0x%x.\n", u16Val);
    else
        printf("    Failed to get speaker maximum master volume!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get volume resolution of UAC device's speaker master channel.           */
    /*--------------------------------------------------------------------------*/
    if(usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_RES, UAC_CH_MASTER, &u16Val) == UAC_RET_OK)
        printf("    Speaker master volume resolution is 0x%x.\n", u16Val);
    else
        printf("    Failed to get speaker master volume resolution!\n");

    /*--------------------------------------------------------------------------*/
    /*  Get current volume value of UAC device's speaker master channel.        */
    /*--------------------------------------------------------------------------*/
    if(usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_CUR, UAC_CH_MASTER, &u16Val) == UAC_RET_OK)
        printf("    Speaker master volume is 0x%x.\n", u16Val);
    else
        printf("    Failed to get speaker master volume!\n");

#if 0
    printf("\nMixer master volume control ===>\n");

    /*-------------------------------------------------------------*/
    /*  Get current mute value of UAC device's microphone.         */
    /*-------------------------------------------------------------*/
    printf("\nMicrophone mute control ===>\n");
    if(usbh_uac_mute_control(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, UAC_CH_MASTER, au8Data) == UAC_RET_OK)
        printf("    Microphone mute state is %d.\n", au8Data[0]);
    else
        printf("    Failed to get microphone mute state!\n");
#endif

    printf("\nMicrophone volume control ===>\n");

    /*-------------------------------------------------------------*/
    /*  Get current volume value of UAC device's microphone.       */
    /*-------------------------------------------------------------*/
    if(usbh_uac_vol_control(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, UAC_CH_MASTER, &s_u16VolCur) == UAC_RET_OK)
        printf("    Microphone current volume is 0x%x.\n", s_u16VolCur);
    else
        printf("    Failed to get microphone current volume!\n");

    /*-------------------------------------------------------------*/
    /*  Get minimum volume value of UAC device's microphone.       */
    /*-------------------------------------------------------------*/
    if(usbh_uac_vol_control(uac_dev, UAC_MICROPHONE, UAC_GET_MIN, UAC_CH_MASTER, &s_u16VolMin) == UAC_RET_OK)
        printf("    Microphone minimum volume is 0x%x.\n", s_u16VolMin);
    else
        printf("    Failed to get microphone minimum volume!\n");

    /*-------------------------------------------------------------*/
    /*  Get maximum volume value of UAC device's microphone.       */
    /*-------------------------------------------------------------*/
    if(usbh_uac_vol_control(uac_dev, UAC_MICROPHONE, UAC_GET_MAX, UAC_CH_MASTER, &s_u16VolMax) == UAC_RET_OK)
        printf("    Microphone maximum volume is 0x%x.\n", s_u16VolMax);
    else
        printf("    Failed to get microphone maximum volume!\n");

    /*-------------------------------------------------------------*/
    /*  Get resolution of UAC device's microphone volume value.    */
    /*-------------------------------------------------------------*/
    if(usbh_uac_vol_control(uac_dev, UAC_MICROPHONE, UAC_GET_RES, UAC_CH_MASTER, &s_u16VolRes) == UAC_RET_OK)
        printf("    Microphone volume resolution is 0x%x.\n", s_u16VolRes);
    else
        printf("    Failed to get microphone volume resolution!\n");

#if 0
    /*-------------------------------------------------------------*/
    /*  Get current auto-gain setting of UAC device's microphone.  */
    /*-------------------------------------------------------------*/
    printf("\nMicrophone automatic gain control ===>\n");
    if(UAC_AutoGainControl(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, UAC_CH_MASTER, au8Data) == UAC_RET_OK)
        printf("    Microphone auto gain is %s.\n", au8Data[0] ? "ON" : "OFF");
    else
        printf("    Failed to get microphone auto-gain state!\n");
#endif

    printf("\nSampling rate control ===>\n");

    /*-------------------------------------------------------------*/
    /*  Get current sampling rate value of UAC device's speaker.   */
    /*-------------------------------------------------------------*/
    if(usbh_uac_sampling_rate_control(uac_dev, UAC_SPEAKER, UAC_GET_CUR, &u32Val) == UAC_RET_OK)
        printf("    Speaker's current sampling rate is %d.\n", u32Val);
    else
        printf("    Failed to get speaker's current sampling rate!\n");

    /*-------------------------------------------------------------*/
    /*  Set new sampling rate value of UAC device's speaker.       */
    /*-------------------------------------------------------------*/
    u32Val = 48000;
    if(usbh_uac_sampling_rate_control(uac_dev, UAC_SPEAKER, UAC_SET_CUR, &u32Val) != UAC_RET_OK)
        printf("    Failed to set Speaker's current sampling rate %d.\n", u32Val);

    if(usbh_uac_sampling_rate_control(uac_dev, UAC_SPEAKER, UAC_GET_CUR, &u32Val) == UAC_RET_OK)
        printf("    Speaker's current sampling rate is %d.\n", u32Val);
    else
        printf("    Failed to get speaker's current sampling rate!\n");

    /*-------------------------------------------------------------*/
    /*  Get current sampling rate value of UAC device's microphone.*/
    /*-------------------------------------------------------------*/
    if(usbh_uac_sampling_rate_control(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, &u32Val) == UAC_RET_OK)
        printf("    Microphone's current sampling rate is %d.\n", u32Val);
    else
        printf("    Failed to get microphone's current sampling rate!\n");

    /*-------------------------------------------------------------*/
    /*  Set new sampling rate value of UAC device's microphone.    */
    /*-------------------------------------------------------------*/
    u32Val = 48000;
    if(usbh_uac_sampling_rate_control(uac_dev, UAC_MICROPHONE, UAC_SET_CUR, &u32Val) != UAC_RET_OK)
        printf("    Failed to set microphone's current sampling rate!\n");

    if(usbh_uac_sampling_rate_control(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, &u32Val) == UAC_RET_OK)
        printf("    Microphone's current sampling rate is %d.\n", u32Val);
    else
        printf("    Failed to get microphone's current sampling rate!\n");
}

int main(void)
{
    UAC_DEV_T *uac_dev = NULL;
    int i8Ch;
    uint16_t u16Val;

    SYS_Init();                        /* Init System, IP clock and multi-function I/O */

    UART0_Init();                      /* Initialize UART0 */

    printf("\n\n");
    printf("+--------------------------------------------+\n");
    printf("|                                            |\n");
    printf("|    USB Host Audio Class sample program     |\n");
    printf("|                                            |\n");
    printf("+--------------------------------------------+\n");

    enable_sys_tick(100);

    usbh_core_init();
    usbh_uac_init();
    usbh_memory_used();

    while(1)
    {
        if(usbh_pooling_hubs())               /* USB Host port detect polling and management */
        {
            /*
             *  Has hub port event.
             */

            uac_dev = usbh_uac_get_device_list();
            if(uac_dev == NULL)
                continue;

            if(uac_dev != NULL)               /* should be newly connected UAC device */
            {
                usbh_uac_open(uac_dev);

                uac_control_example(uac_dev);

                usbh_uac_start_audio_out(uac_dev, audio_out_callback);

                usbh_uac_start_audio_in(uac_dev, audio_in_callback);
            }
        }

        if(uac_dev == NULL)
        {
            s_i8AuInCnt = 0;
            s_i8AuOutCnt = 0;

            if(!kbhit())
            {
                i8Ch = getchar();
                usbh_memory_used();
            }

            continue;
        }

        if(!kbhit())
        {
            i8Ch = getchar();

            if((i8Ch == '+') && (s_u16VolCur + s_u16VolRes <= s_u16VolMax))
            {
                printf("+");
                u16Val = s_u16VolCur + s_u16VolRes;
                if(usbh_uac_vol_control(uac_dev, UAC_MICROPHONE, UAC_SET_CUR, UAC_CH_MASTER, &u16Val) == UAC_RET_OK)
                {
                    printf("    Microphone set volume 0x%x success.\n", u16Val);
                    s_u16VolCur = u16Val;
                }
                else
                    printf("    Failed to set microphone volume 0x%x!\n", u16Val);
            }
            else if((i8Ch == '-') && (s_u16VolCur - s_u16VolRes >= s_u16VolMin))
            {
                printf("-");
                u16Val = s_u16VolCur - s_u16VolRes;
                if(usbh_uac_vol_control(uac_dev, UAC_MICROPHONE, UAC_SET_CUR, UAC_CH_MASTER, &u16Val) == UAC_RET_OK)
                {
                    printf("    Microphone set volume 0x%x success.\n", u16Val);
                    s_u16VolCur = u16Val;
                }
                else
                    printf("    Failed to set microphone volume 0x%x!\n", u16Val);
            }
            else if((i8Ch == '0') && (s_u16VolCur - s_u16VolRes >= s_u16VolMin))
            {
                if(usbh_uac_vol_control(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, UAC_CH_MASTER, &s_u16VolCur) == UAC_RET_OK)
                    printf("    Microphone current volume is 0x%x.\n", s_u16VolCur);
                else
                    printf("    Failed to get microphone current volume!\n");
            }
            else
            {
                printf("IN: %d, OUT: %d\n", s_i8AuInCnt, s_i8AuOutCnt);
                usbh_memory_used();
            }

        }  /* end of kbhit() */
    }
}
