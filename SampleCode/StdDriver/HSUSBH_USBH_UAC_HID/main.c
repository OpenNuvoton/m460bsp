/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrates how to use USBH Audio Class driver and HID driver
 *           at the same time. It shows the mute, volume, auto-gain, channel,
 *           and sampling rate control.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"

#include "usbh_lib.h"
#include "usbh_uac.h"
#include "usbh_hid.h"

//#define UAC_MUTE_CONTROL
//#define UAC_GAIN_CONTROL

#define AUDIO_IN_BUFSIZ             8192

#ifdef DEBUG_ENABLE_SEMIHOST
#error This sample cannot execute with semihost enabled
#endif

#ifdef __ICCARM__
#pragma data_alignment=32
static uint8_t s_au8BuffPool[1024];
#else
static uint8_t s_au8BuffPool[1024] __attribute__((aligned(32)));
#endif

static HID_DEV_T *s_hid_list[CONFIG_HID_MAX_DEV];

static volatile int s_i8AuInCnt, s_i8AuOutCnt;

static uint16_t g_u16RecVolMax, g_u16RecVolMin, g_u16RecVolRes, g_u16RecVolCur;    
static uint16_t g_u16PlayVolMax, g_u16PlayVolMin, g_u16PlayVolRes, g_u16PlayVolCur;  
uint8_t u8SpkAlt, u8MicAlt;

extern int kbhit(void);                        /* function in retarget.c                 */

static volatile uint32_t s_u32TickCnt;

void SysTick_Handler(void);
void enable_sys_tick(int ticks_per_second);
void SYS_Init(void);
void UART0_Init(void);
void dump_buff_hex(uint8_t *pu8Buff, int i8Bytes);
int is_a_new_hid_device(HID_DEV_T *hdev);
void update_hid_device_list(HID_DEV_T *hdev);
void int_read_callback(HID_DEV_T *hdev, uint16_t u16EpAddr, int i8Status, uint8_t *pu8RData, uint32_t u32DataLen);
int init_hid_device(HID_DEV_T *hdev);
int audio_in_callback(UAC_DEV_T *dev, uint8_t *pu8Data, int i8Len);
int audio_out_callback(UAC_DEV_T *dev, uint8_t *pu8Data, int i8Len);
void uac10_control_example(UAC_DEV_T *uac_dev);                         
void uac20_control_example(UAC_DEV_T *uac_dev);
       

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

int is_a_new_hid_device(HID_DEV_T *hdev)
{
    int i8Cnt;

    for(i8Cnt = 0; i8Cnt < CONFIG_HID_MAX_DEV; i8Cnt++)
    {
        if((s_hid_list[i8Cnt] != NULL) && (s_hid_list[i8Cnt] == hdev) &&
                (s_hid_list[i8Cnt]->uid == hdev->uid))
            return 0;
    }
    return 1;
}

void update_hid_device_list(HID_DEV_T *hdev)
{
    int i8Cnt = 0;

    memset(s_hid_list, 0, sizeof(s_hid_list));
    while((i8Cnt < CONFIG_HID_MAX_DEV) && (hdev != NULL))
    {
        s_hid_list[i8Cnt++] = hdev;
        hdev = hdev->next;
    }
}

void int_read_callback(HID_DEV_T *hdev, uint16_t u16EpAddr, int i8Status, uint8_t *pu8RData, uint32_t u32DataLen)
{
    /*
     *  USB host HID driver notify user the transfer status via <i8Status> parameter. If the
     *  If <i8Status> is 0, the USB transfer is fine. If <i8Status> is not zero, this interrupt in
     *  transfer failed and HID driver will stop this pipe. It can be caused by USB transfer error
     *  or device disconnected.
     */
    if(i8Status < 0)
    {
        printf("Interrupt in transfer failed! status: %d\n", i8Status);
        return;
    }
    printf("Device [0x%x,0x%x] ep 0x%x, %d bytes received =>\n",
           hdev->idVendor, hdev->idProduct, u16EpAddr, u32DataLen);
    dump_buff_hex(pu8RData, (int)u32DataLen);
}

int init_hid_device(HID_DEV_T *hdev)
{
    uint8_t *pu8DataBuff;
    int i8Ret;

    pu8DataBuff = (uint8_t *)((uint32_t)s_au8BuffPool);

    printf("\n\n==================================\n");
    printf("  Init HID device : 0x%x\n", (int)hdev);
    printf("  VID: 0x%x, PID: 0x%x\n\n", hdev->idVendor, hdev->idProduct);

    i8Ret = usbh_hid_get_report_descriptor(hdev, pu8DataBuff, 1024);
    if(i8Ret > 0)
    {
        printf("\nDump report descriptor =>\n");
        dump_buff_hex(pu8DataBuff, i8Ret);
    }

    printf("\nUSBH_HidStartIntReadPipe...\n");
    i8Ret = usbh_hid_start_int_read(hdev, 0, int_read_callback);
    if(i8Ret != HID_RET_OK)
        printf("usbh_hid_start_int_read failed! %d\n", i8Ret);
    else
        printf("Interrupt in transfer started...\n");

    return 0;
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
    s_i8AuInCnt += i8Len;
//    printf("I %x,%d bytes\n", (int)pu8Data & 0xffff, i8Len);   // UART send too many will cause ISO transfer time overrun

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
//    static uint32_t u32Data = 0;
//    uint32_t *pu32Data = (uint32_t *) pu8Data;
    (void)dev;
    (void)pu8Data;

    s_i8AuOutCnt += i8Len;
    
//    printf("O %x\n", i8Len);   // UART send too many will cause ISO transfer time overrun

    // Add your code here to put audio-out data ...
    // For example, memcpy(pu8Data, playback_buffer, actual_len);
    //              return actual_len;
    // . . .

    return i8Len;   // for 48000 stereo Hz
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
    HID_DEV_T *hdev, *hdev_list;
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
    usbh_hid_init();
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
                if(uac_dev->acif.mic_id)
                    usbh_uac_start_audio_in(uac_dev, u8MicAlt, audio_in_callback);
                if(uac_dev->acif.speaker_id)
                    usbh_uac_start_audio_out(uac_dev, u8SpkAlt, audio_out_callback);
            }

            hdev_list = usbh_hid_get_device_list();
            hdev = hdev_list;
            while(hdev != NULL)
            {
                if(is_a_new_hid_device(hdev))
                {
                    init_hid_device(hdev);
                }
                hdev = hdev->next;
            }
            update_hid_device_list(hdev_list);
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
            printf("IN: %d, OUT: %d\n", s_i8AuInCnt, s_i8AuOutCnt);
            usbh_memory_used();

        }  /* end of kbhit() */
    }
}
