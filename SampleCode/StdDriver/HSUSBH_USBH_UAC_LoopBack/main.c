/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    The sample receives audio data from UAC device, and immediately send
 *           back to that UAC device.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"
#include "usbh_lib.h"
#include "usbh_uac.h"

#ifdef DEBUG_ENABLE_SEMIHOST
#error This sample cannot execute with semihost enabled
#endif

//#define UAC_MUTE_CONTROL
//#define UAC_GAIN_CONTROL


static uint16_t g_u16RecVolMax, g_u16RecVolMin, g_u16RecVolRes, g_u16RecVolCur;    
static uint16_t g_u16PlayVolMax, g_u16PlayVolMin, g_u16PlayVolRes, g_u16PlayVolCur;  
uint8_t u8SpkAlt, u8MicAlt;

extern int kbhit(void);                        /* function in retarget.c                 */

extern volatile int8_t g_i8MicIsMono;
extern volatile uint32_t g_u32UacRecCnt;       /* Counter of UAC record data             */
extern volatile uint32_t g_u32UacPlayCnt;      /* Counter of UAC playback data           */

extern void ResetAudioLoopBack(void);
extern int audio_in_callback(UAC_DEV_T *dev, uint8_t *pu8Data, int i8Len);
extern int audio_out_callback(UAC_DEV_T *dev, uint8_t *pu8Data, int i8Len);

static volatile uint32_t s_u32TickCnt;

void SysTick_Handler(void);
void enable_sys_tick(int ticks_per_second);
void dump_buff_hex(uint8_t *pu8Buff, int i8Bytes);
void uac_control_example(UAC_DEV_T *uac_dev);
void SYS_Init(void);
void UART0_Init(void);

void SysTick_Handler(void)
{
    s_u32TickCnt++;
}

void enable_sys_tick(int ticks_per_second)
{
    s_u32TickCnt = 0;
    if(SysTick_Config(SystemCoreClock / (uint32_t)ticks_per_second))
    {
        /* Setup SysTick Timer for 1 second interrupts */
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

void dump_buff_hex(uint8_t *pu8Buff, int i8Bytes)
{
    int i8Idx, i8Cnt;

    i8Idx = 0;
    while(i8Bytes > 0)
    {
        printf("0x%04X  ", i8Idx);
        for(i8Cnt = 0; (i8Cnt < 16) && (i8Bytes > 0); i8Cnt++)
        {
            printf("%02x ", pu8Buff[i8Idx + i8Cnt]);
            i8Bytes--;
        }
        i8Idx += 16;
        printf("\n");
    }
    printf("\n");
}


void uac_control_example(UAC_DEV_T *uac_dev)
{
    uint32_t au32SRate[32];
    uint32_t au32SpeakChannel[32];            
    uint32_t au32MicChannel[32];              
    uint32_t au32SpeakBit[32];            
    uint32_t au32MicBit[32];
    uint8_t u8SpkCh, u8MicCh;               
    uint8_t u8SpkBit, u8MicBit;       
    uint8_t u8Val;
    uint8_t u8Item;
    int i8Cnt, i8Ret;
    uint32_t u32Val;

    printf("\nGet channel & Sub frame Size information ===>\n");
    /*-------------------------------------------------------------*/
    /*  Get information                                            */
    /*-------------------------------------------------------------*/
    if(uac_dev->acif.speaker_id)
    {
        i8Ret = usbh_uac_get_channel_number(uac_dev, UAC_SPEAKER,(uint32_t *)&au32SpeakChannel[0], 4, &u8SpkCh);
        if(i8Ret < 0)
            printf("    Failed to get Speaker's Channel number.\n");
        else
        {
            i8Ret = usbh_uac_get_bit_resolution(uac_dev, UAC_SPEAKER,(uint32_t *)&au32SpeakBit[0], 4, &u8SpkBit);
        
            if(i8Ret < 0)
                printf("    Failed to get Speaker's Sub frame Size.\n");
            else
            {
                for(i8Cnt = 0; i8Cnt < u8SpkCh; i8Cnt++)
                    printf("    Speaker alt %d : %d Channels  %d bit resolution\n", i8Cnt, au32SpeakChannel[i8Cnt], au32SpeakBit[i8Cnt]);

                printf(" * Please select Speaker alt:\n");  
                u8Item = getchar() - '0';
                if(u8Item > 0 && u8Item <uac_dev->asif_out.iface->num_alt )
                    u8SpkAlt = u8Item;
                else
                    u8SpkAlt = 1;
                printf(" -> Select Speaker alt %d : %d Channels  %d bytes per sample\n", u8SpkAlt, au32SpeakChannel[u8SpkAlt], au32SpeakBit[u8SpkAlt]);
            }
        }
    }
    if(uac_dev->acif.mic_id)
    {
        i8Ret = usbh_uac_get_channel_number(uac_dev, UAC_MICROPHONE,(uint32_t *)&au32MicChannel[0], 4, &u8MicCh);
        if(i8Ret < 0)
            printf("    Failed to get Microphone's Channel number.\n");
        else
        {
            i8Ret = usbh_uac_get_bit_resolution(uac_dev, UAC_MICROPHONE,(uint32_t *)&au32MicBit[0], 4, &u8MicBit);
       
            if(i8Ret < 0)
                printf("    Failed to get Microphone's Sub frame Size.\n");
            else
            {
                for(i8Cnt = 0; i8Cnt < u8MicBit; i8Cnt++)
                    printf("    Microphone alt %d : %d Channels %d bytes per sample\n", i8Cnt,  au32MicChannel[i8Cnt], au32MicBit[i8Cnt]);

                printf(" * Please select Microphone alt:\n");  
                u8Item = getchar() - '0';
                if(u8Item > 0 && u8Item <uac_dev->asif_in.iface->num_alt )
                    u8MicAlt = u8Item;
                else
                    u8MicAlt = 1;   
                printf(" -> Select Microphone alt %d : %d Channels %d bytes per sample\n", u8MicAlt,  au32MicChannel[u8MicAlt], au32MicBit[u8MicAlt]);
            }
        }
    }

    printf("\nSampling Rate Control ===>\n");

    if(uac_dev->acif.speaker_id)
    {
        printf(" * Get Speaker Sampling rate list\n");
        i8Ret = usbh_uac_get_sampling_rate(uac_dev, UAC_SPEAKER, (uint32_t *)&au32SRate[0], 4, &u8Val);
        if(i8Ret < 0)
            printf("    Failed to get Speaker's Sampling rate.\n");
        else
        {
            if(u8Val == 0)
            {
                printf("    Speaker Sampling rate range: %d ~ %d Hz\n", au32SRate[0], au32SRate[1]);
                u32Val = au32SRate[0];
            }
            else
            {
                for(i8Cnt = 0; i8Cnt < u8Val; i8Cnt++)
                    printf("    Speaker Sampling rate index %d : %d\n", i8Cnt, au32SRate[i8Cnt]);
                printf(" * Please select Speaker Sampling rate:\n");
                u8Item = getchar() - '0';
                if(u8Item > 0 && u8Item <u8Val)
                    u32Val = au32SRate[u8Item];
                else
                    u32Val = au32SRate[0];
            }
            printf(" * Set Speaker Sampling rate to %d\n", u32Val);

            if(usbh_uac_sampling_rate_control(uac_dev, UAC_SPEAKER, UAC_SET_CUR, &u32Val) != UAC_RET_OK)
                printf("    Failed to set Speaker's Sampling rate!\n");
            else
                printf("    Set Speaker's Sampling rate to %d!\n",u32Val);

            printf(" * Get Speaker Current Sampling rate\n");

            if(usbh_uac_sampling_rate_control(uac_dev, UAC_SPEAKER, UAC_GET_CUR, &u32Val) == UAC_RET_OK)
                printf("    Speaker's current Sampling rate is %d.\n", u32Val);
            else
                printf("    Failed to get Speaker's current Sampling rate!\n");
        }
        printf(" -> Set Speaker Sampling rate to %d\n", u32Val);
    }
    if(uac_dev->acif.mic_id)
    {
        printf(" * Get Microphone Sampling rate list\n");
        i8Ret = usbh_uac_get_sampling_rate(uac_dev, UAC_MICROPHONE, (uint32_t *)&au32SRate[0], 4, &u8Val);
        if(i8Ret < 0)
            printf("    Failed to get Microphone's Sampling rate.\n");
        else
        {
            if(u8Val == 0)
            {
                printf("    Microphone Sampling rate range: %d ~ %d Hz\n", au32SRate[0], au32SRate[1]);
                u32Val = au32SRate[0];
            }
            else
            {
                for(i8Cnt = 0; i8Cnt < u8Val; i8Cnt++)
                    printf("    Microphone Sampling rate index %d : %d\n", i8Cnt, au32SRate[i8Cnt]);
                printf(" * Please select Microphone Sampling rate:\n");
                u8Item = getchar() - '0';
                if(u8Item > 0 && u8Item <u8Val)
                    u32Val = au32SRate[u8Item];
                else
                    u32Val = au32SRate[0];
            }
        }
        if(usbh_uac_sampling_rate_control(uac_dev, UAC_MICROPHONE, UAC_SET_CUR, &u32Val) != UAC_RET_OK)
            printf("    Failed to set Microphone's Sampling rate!\n");       
        else
            printf(" -> Set Microphone's Sampling rate to %d!\n",u32Val);

        printf(" * Get Microphone Current Sampling rate\n");

        if(usbh_uac_sampling_rate_control(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, &u32Val) == UAC_RET_OK)
            printf("    Microphone's current Sampling rate is %d.\n", u32Val);
        else
            printf("    Failed to get Microphone's current Sampling rate!\n");
    }
#ifdef UAC_MUTE_CONTROL
    printf("\nMute Control ===>\n");
    if(uac_dev->acif.speaker_id)
    {
        if(usbh_uac_mute_control(uac_dev, UAC_SPEAKER, UAC_GET_CUR, UAC_CH_MASTER, au8Data) == UAC_RET_OK)
        {
            printf("    Get Speaker mute state : %d.\n", au8Data[0]);
        }
        else
            printf("    Failed to get speaker mute state!\n");

        au8Data[0] ^= 1;
        if(usbh_uac_mute_control(uac_dev, UAC_SPEAKER, UAC_SET_CUR, UAC_CH_MASTER, au8Data) == UAC_RET_OK)
        {
            printf("    Set Speaker mute state to %d.\n", au8Data[0]);
        }
        else
            printf("    Failed to set speaker mute state!\n");

        if(usbh_uac_mute_control(uac_dev, UAC_SPEAKER, UAC_GET_CUR, UAC_CH_MASTER, au8Data) == UAC_RET_OK)
        {
            printf("    Get Speaker mute state : %d.\n", au8Data[0]);
        }
        else
            printf("    Failed to get speaker mute state!\n");
    }
    if(uac_dev->acif.mic_id)
    {
        if(usbh_uac_mute_control(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, UAC_CH_MASTER, au8Data) == UAC_RET_OK)
        {
            printf("    Get Mircophone mute state : %d.\n", au8Data[0]);
        }
        else
            printf("    Failed to get Mircophone mute state!\n");

        au8Data[0] ^= 1;
        if(usbh_uac_mute_control(uac_dev, UAC_MICROPHONE, UAC_SET_CUR, UAC_CH_MASTER, au8Data) == UAC_RET_OK)
        {
            printf("    Set Mircophone mute state to %d.\n", au8Data[0]);
        }
        else
            printf("    Failed to get Mircophone mute state!\n");

        if(usbh_uac_mute_control(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, UAC_CH_MASTER, au8Data) == UAC_RET_OK)
        {
            printf("    Get Mircophone mute state : %d.\n", au8Data[0]);
        }
        else
            printf("    Failed to get Mircophone mute state!\n");
    }
    if(uac_dev->acif.speaker_id)
    {
        printf("\nSpeaker master volume control ===>\n");
        /*--------------------------------------------------------------------------*/
        /*  Get minimum volume value of UAC device's speaker master channel.        */
        /*--------------------------------------------------------------------------*/
        if(usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_MIN, UAC_CH_MASTER, &g_u16PlayVolMin) == UAC_RET_OK)
            printf("    Speaker minimum master volume is 0x%x.\n", g_u16PlayVolMin);
        else
            printf("    Failed to get speaker master minimum volume!\n");

        /*--------------------------------------------------------------------------*/
        /*  Get maximum volume value of UAC device's speaker master channel.        */
        /*--------------------------------------------------------------------------*/
        if(usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_MAX, UAC_CH_MASTER, &g_u16PlayVolMax) == UAC_RET_OK)
            printf("    Speaker maximum master volume is 0x%x.\n", g_u16PlayVolMax);
        else
            printf("    Failed to get speaker maximum master volume!\n");

        /*--------------------------------------------------------------------------*/
        /*  Get volume resolution of UAC device's speaker master channel.           */
        /*--------------------------------------------------------------------------*/
        if(usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_RES, UAC_CH_MASTER, &g_u16PlayVolRes) == UAC_RET_OK)
            printf("    Speaker master volume resolution is 0x%x.\n", g_u16PlayVolRes);
        else
            printf("    Failed to get speaker master volume resolution!\n");

        /*--------------------------------------------------------------------------*/
        /*  Get current volume value of UAC device's speaker master channel.        */
        /*--------------------------------------------------------------------------*/
        if(usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_CUR, UAC_CH_MASTER, &g_u16PlayVolCur) == UAC_RET_OK)
            printf("    Speaker master volume is 0x%x.\n", g_u16PlayVolCur);
        else
            printf("    Failed to get speaker master volume!\n");
    }

    if(uac_dev->acif.mic_id)
    {
        printf("\nMixer master volume control ===>\n");

        /*-------------------------------------------------------------*/
        /*  Get current mute value of UAC device's microphone.         */
        /*-------------------------------------------------------------*/
        printf("\nMicrophone mute control ===>\n");
        if(usbh_uac_mute_control(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, UAC_CH_MASTER, au8Data) == UAC_RET_OK)
            printf("    Microphone mute state is %d.\n", au8Data[0]);
        else
            printf("    Failed to get microphone mute state!\n");
    }
#endif
    if(uac_dev->acif.mic_id)
    {
        printf("\nMicrophone volume control ===>\n");

        /*-------------------------------------------------------------*/
        /*  Get current volume value of UAC device's microphone.       */
        /*-------------------------------------------------------------*/
        if(usbh_uac_vol_control(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, UAC_CH_MASTER, &g_u16RecVolCur) == UAC_RET_OK)
            printf("    Microphone current volume is 0x%x.\n", g_u16RecVolCur);
        else
            printf("    Failed to get microphone current volume!\n");

        /*-------------------------------------------------------------*/
        /*  Get minimum volume value of UAC device's microphone.       */
        /*-------------------------------------------------------------*/
        if(usbh_uac_vol_control(uac_dev, UAC_MICROPHONE, UAC_GET_MIN, UAC_CH_MASTER, &g_u16RecVolMin) == UAC_RET_OK)
            printf("    Microphone minimum volume is 0x%x.\n", g_u16RecVolMin);
        else
            printf("    Failed to get microphone minimum volume!\n");

        /*-------------------------------------------------------------*/
        /*  Get maximum volume value of UAC device's microphone.       */
        /*-------------------------------------------------------------*/
        if(usbh_uac_vol_control(uac_dev, UAC_MICROPHONE, UAC_GET_MAX, UAC_CH_MASTER, &g_u16RecVolMax) == UAC_RET_OK)
            printf("    Microphone maximum volume is 0x%x.\n", g_u16RecVolMax);
        else
            printf("    Failed to get microphone maximum volume!\n");

        /*-------------------------------------------------------------*/
        /*  Get resolution of UAC device's microphone volume value.    */
        /*-------------------------------------------------------------*/
        if(usbh_uac_vol_control(uac_dev, UAC_MICROPHONE, UAC_GET_RES, UAC_CH_MASTER, &g_u16RecVolRes) == UAC_RET_OK)
            printf("    Microphone volume resolution is 0x%x.\n", g_u16RecVolRes);
        else
            printf("    Failed to get microphone volume resolution!\n");
    }
#ifdef UAC_GAIN_CONTROL
    printf("\nSpeaker automatic gain control ===>\n");
    if(usbh_uac_auto_gain_control(uac_dev, UAC_SPEAKER, UAC_GET_CUR, UAC_CH_MASTER, au8Data) == UAC_RET_OK)
        printf("    Get Speaker auto gain : %s.\n", au8Data[0] ? "ON" : "OFF");
    else
        printf("    Failed to get speaker auto-gain state!\n");

    au8Data[0] ^= 1;
    if(usbh_uac_auto_gain_control(uac_dev, UAC_SPEAKER, UAC_SET_CUR, UAC_CH_MASTER, au8Data) == UAC_RET_OK)
        printf("    Set Speaker auto gain to %s.\n", au8Data[0] ? "ON" : "OFF");
    else
        printf("    Failed to get speaker auto-gain state!\n");

    if(usbh_uac_auto_gain_control(uac_dev, UAC_SPEAKER, UAC_GET_CUR, UAC_CH_MASTER, au8Data) == UAC_RET_OK)
        printf("    Get Speaker auto gain : %s.\n", au8Data[0] ? "ON" : "OFF");
    else
        printf("    Failed to get speaker auto-gain state!\n");
    /*-------------------------------------------------------------*/
    /*  Get current auto-gain setting of UAC device's microphone.  */
    /*-------------------------------------------------------------*/
    printf("\nMicrophone automatic gain control ===>\n");
    if(usbh_uac_auto_gain_control(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, UAC_CH_MASTER, au8Data) == UAC_RET_OK)
        printf("    Get Microphone auto gain : %s.\n", au8Data[0] ? "ON" : "OFF");
    else
        printf("    Failed to get microphone auto-gain state!\n");

    au8Data[0] ^= 1;
    if(usbh_uac_auto_gain_control(uac_dev, UAC_MICROPHONE, UAC_SET_CUR, UAC_CH_MASTER, au8Data) == UAC_RET_OK)
        printf("    Set Microphone auto gain to %s.\n", au8Data[0] ? "ON" : "OFF");
    else
        printf("    Failed to get microphone auto-gain state!\n");

    if(usbh_uac_auto_gain_control(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, UAC_CH_MASTER, au8Data) == UAC_RET_OK)
        printf("    Get Microphone auto gain : %s.\n", au8Data[0] ? "ON" : "OFF");
    else
        printf("    Failed to get microphone auto-gain state!\n");
#endif
}

int main(void)
{
    UAC_DEV_T *uac_dev = NULL;
    int i8Ch;
    uint16_t u16Val;

    SYS_Init();                        /* Init System, IP clock and multi-function I/O */

    UART0_Init();                      /* Initialize UART0 */

    enable_sys_tick(100);

    printf("\n\n");
    printf("+--------------------------------------------+\n");
    printf("|                                            |\n");
    printf("|        USB Host UAC loop back demo         |\n");
    printf("|                                            |\n");
    printf("+--------------------------------------------+\n");

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

                ResetAudioLoopBack();

                if(uac_dev->acif.mic_id)
                    usbh_uac_start_audio_in(uac_dev, u8MicAlt, audio_in_callback);
                if(uac_dev->acif.speaker_id)
                    usbh_uac_start_audio_out(uac_dev, u8SpkAlt, audio_out_callback);
            }
        }

        if(uac_dev == NULL)
        {
            ResetAudioLoopBack();
            if(!kbhit())
            {
                i8Ch = getchar();
                usbh_memory_used();
            }
            continue;
        }


        if(kbhit())
        {
            i8Ch = getchar();
            if(uac_dev->acif.mic_id)
            {
                if((i8Ch == '+') && (g_u16RecVolCur + g_u16RecVolRes <= g_u16RecVolMax))
                {
                    printf("+");
                    u16Val = g_u16RecVolCur + g_u16RecVolRes;
                    if(usbh_uac_vol_control(uac_dev, UAC_MICROPHONE, UAC_SET_CUR, UAC_CH_MASTER, &u16Val) == UAC_RET_OK)
                    {
                        printf("    Microphone set volume 0x%x success.\n", u16Val);
                        g_u16RecVolCur = u16Val;
                    }
                    else
                        printf("    Failed to set microphone volume 0x%x!\n", u16Val);
                }
                else if((i8Ch == '-') && (g_u16RecVolCur - g_u16RecVolRes >= g_u16RecVolMin))
                {
                    printf("-");
                    u16Val = g_u16RecVolCur - g_u16RecVolRes;
                    if(usbh_uac_vol_control(uac_dev, UAC_MICROPHONE, UAC_SET_CUR, UAC_CH_MASTER, &u16Val) == UAC_RET_OK)
                    {
                        printf("    Microphone set volume 0x%x success.\n", u16Val);
                        g_u16RecVolCur = u16Val;
                    }
                    else
                        printf("    Failed to set microphone volume 0x%x!\n", u16Val);
                }
                else if(i8Ch == '0')
                {
                    if(g_u16RecVolCur - g_u16RecVolRes >= g_u16RecVolMin)
                    {
                        if(usbh_uac_vol_control(uac_dev, UAC_MICROPHONE, UAC_GET_CUR, UAC_CH_MASTER, &g_u16RecVolCur) == UAC_RET_OK)
                            printf("    Microphone current volume is 0x%x.\n", g_u16RecVolCur);
                        else
                            printf("    Failed to get microphone current volume!\n");
                    }
                }
            }

            if(uac_dev->acif.speaker_id)
            {
                if((i8Ch == '+') && (g_u16PlayVolCur + g_u16PlayVolRes <= g_u16PlayVolMax))
                {
                    printf("+");
                    u16Val = g_u16PlayVolCur + g_u16PlayVolRes;
                    if(usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_SET_CUR, UAC_CH_MASTER, &u16Val) == UAC_RET_OK)
                    {
                        printf("    Speaker set volume 0x%x success.\n", u16Val);
                        g_u16PlayVolCur = u16Val;
                    }
                    else
                        printf("    Failed to set speaker volume 0x%x!\n", u16Val);

                }
                else if((i8Ch == '-') && (g_u16PlayVolCur - g_u16PlayVolRes >= g_u16PlayVolMin))
                {
                    printf("-");

                    u16Val = g_u16PlayVolCur - g_u16PlayVolRes;
                    if(usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_SET_CUR, UAC_CH_MASTER, &u16Val) == UAC_RET_OK)
                    {
                        printf("    Speaker set volume 0x%x success.\n", u16Val);
                        g_u16PlayVolCur = u16Val;
                    }
                    else
                        printf("    Failed to set speaker volume 0x%x!\n", u16Val);
                }
                else if(i8Ch == '0')
                {
                    if(g_u16PlayVolCur - g_u16PlayVolRes >= g_u16PlayVolMin)
                    {
                        if(usbh_uac_vol_control(uac_dev, UAC_SPEAKER, UAC_GET_CUR, UAC_CH_MASTER, &g_u16PlayVolCur) == UAC_RET_OK)
                            printf("    Speaker current volume is 0x%x.\n", g_u16PlayVolCur);
                        else
                            printf("    Failed to get speaker current volume!\n");
                    }
                }
            }
            printf("IN: %d, OUT: %d\n", g_u32UacRecCnt, g_u32UacPlayCnt);
            usbh_memory_used();

        }  /* end of kbhit() */
    }
}
