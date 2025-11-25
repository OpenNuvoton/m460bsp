/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    This example demonstrates how to use the USBH library to
 *           connect to a USB Video Class Device, capture the MJPEG bit
 *           stream, and save it to an SD card as a JPG file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"
#include "usbh_lib.h"
#include "usbh_uvc.h"
#include "diskio.h"
#include "ff.h"

/*----------------------------------------------------------------------
 * USB UVC
 */
#define SELECT_RES_WIDTH        (320)
#define SELECT_RES_HEIGHT       (240)

#define IMAGE_MAX_SIZE          (SELECT_RES_WIDTH * SELECT_RES_HEIGHT * 2)
#define IMAGE_BUFF_CNT          4

FIL file;

enum
{
    IMAGE_BUFF_FREE,
    IMAGE_BUFF_USB,
    IMAGE_BUFF_READY,
    IMAGE_BUFF_POST
};

struct img_buff_t
{
    uint8_t   *buff;
    int       len;
    int       state;
};


/* MJEPG Format */
struct img_buff_t  _imgs[IMAGE_BUFF_CNT];
uint8_t  (*image_buff_pool)[IMAGE_BUFF_CNT][IMAGE_MAX_SIZE] __attribute__((aligned(32))) = (uint8_t (*)[IMAGE_BUFF_CNT][IMAGE_MAX_SIZE])HYPERRAM_BASE;
volatile int   _idx_usb = 0, _idx_post = 0;
volatile int   ignore_img = 0;

extern int kbhit(void);                        /* function in retarget.c                 */

static volatile uint32_t s_u32TickCnt;
volatile uint32_t g_u32SaveStart = 0;

void SysTick_Handler(void);
void enable_sys_tick(int ticks_per_second);
void SYS_Init(void);
void UART0_Init(void);

/*---------------------------------------------------------*/
/* User Provided RTC Function for FatFs module             */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support an RTC.                     */
/* This function is not required in read-only cfg.         */

unsigned long get_fattime(void)
{
    unsigned long g_u64Tmr;

    g_u64Tmr = 0x00000;

    return g_u64Tmr;
}

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

void SDH0_IRQHandler(void)
{
    unsigned int volatile isr;

    /* FMI data abort interrupt */
    if(SDH0->GINTSTS & SDH_GINTSTS_DTAIF_Msk)
    {
        /* ResetAllEngine() */
        SDH0->GCTL |= SDH_GCTL_GCTLRST_Msk;
    }

    /* ----- SD interrupt status */
    isr = SDH0->INTSTS;
    if(isr & SDH_INTSTS_BLKDIF_Msk)
    {
        /* block down */
        SD0.DataReadyFlag = TRUE;
        SDH0->INTSTS = SDH_INTSTS_BLKDIF_Msk;
    }

    if(isr & SDH_INTSTS_CDIF_Msk)    /* port 0 card detect */
    {
        /* ----- SD interrupt status */
        /* it is work to delay 50 times for SD_CLK = 200KHz */
        {
            int volatile i;         /* delay 30 fail, 50 OK */
            for(i = 0; i < 0x500; i++); /* delay to make sure got updated value from REG_SDISR. */
            isr = SDH0->INTSTS;
        }

        if(isr & SDH_INTSTS_CDSTS_Msk)
        {
            printf("\n***** card remove !\n");
            SD0.IsCardInsert = FALSE;   /* SDISR_CD_Card = 1 means card remove for GPIO mode */
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

    /* CRC error interrupt */
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
        SDH0->INTSTS = SDH_INTSTS_CRCIF_Msk;      /* clear interrupt flag */
    }

    if(isr & SDH_INTSTS_DITOIF_Msk)
    {
        printf("***** ISR: data in timeout !\n");
        SDH0->INTSTS |= SDH_INTSTS_DITOIF_Msk;
    }

    /* Response in timeout interrupt */
    if(isr & SDH_INTSTS_RTOIF_Msk)
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

static int32_t Clear4Bytes(uint32_t u32StartAddr)
{
    outp32(u32StartAddr, 0);
    return 0;
}

static int32_t ClearHyperRAM(uint32_t u32StartAddr, uint32_t u32EndAddr)
{
    uint32_t u32Data, i;

    for(i = u32StartAddr; i < u32EndAddr; i+=4)
    {
        if( Clear4Bytes(i) < 0 )
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

void  init_image_buffers(void)
{
    int   i;

    for(i = 0; i < IMAGE_BUFF_CNT; i++)
    {
        _imgs[i].buff = (uint8_t *)image_buff_pool[i];
        _imgs[i].len = 0;
        _imgs[i].state = IMAGE_BUFF_FREE;
    }

    _idx_usb = 0;
    _idx_post = 0;
}

int  uvc_rx_callbak(UVC_DEV_T *vdev, uint8_t *data, int len)
{
    int next_idx;

    (void)(data);

    next_idx = (_idx_usb + 1) % IMAGE_BUFF_CNT;

    if(_imgs[next_idx].state != IMAGE_BUFF_FREE)
    {
        /*
         *  Next image buffer is in used.
         *  Just drop this newly received image and reuse the same image buffer.
         */
        usbh_uvc_set_video_buffer(vdev, _imgs[_idx_usb].buff, IMAGE_MAX_SIZE);
    }
    else
    {
        _imgs[_idx_usb].state = IMAGE_BUFF_READY;   /* mark the current buffer as ready for decode/display */
        _imgs[_idx_usb].len   = len;                /* length of this newly received image   */

        /* proceed to the next image buffer */
        _idx_usb = next_idx;
        _imgs[_idx_usb].state = IMAGE_BUFF_USB;     /* mark the next image as used by USB    */

        /* assign the next image buffer to receive next image from USB */
        usbh_uvc_set_video_buffer(vdev, _imgs[_idx_usb].buff, IMAGE_MAX_SIZE);
    }

    return 0;
}

UVC_DEV_T *uvc_conn_check(UVC_DEV_T *cur_vdev)
{
    UVC_DEV_T *vdev = cur_vdev;
    IMAGE_FORMAT_E  format;
    int width, height;
    int i, ret;

    if(usbh_pooling_hubs())       /* USB Host port detect polling and management */
    {
        /*
         *  Has hub port event.
         */
        vdev = usbh_uvc_get_device_list();

        if(vdev == NULL)
        {
            cur_vdev = NULL;
            printf("\n[No UVC device connected]\n\n");
            return NULL;
        }

        if(cur_vdev == vdev)
        {
            printf("\n\n\nWaiting for UVC device connected...\n");
            return vdev;
        }

        if(vdev->next != NULL)
        {
            printf("\n\nWarning!! Multiple UVC device is not supported!!\n\n");
            getchar();
            return cur_vdev;
        }

        /*----------------------------------------------------------------------------*/
        /*  New UVC device connected.                                                 */
        /*----------------------------------------------------------------------------*/
        cur_vdev = vdev;
        printf("\n\n----------------------------------------------------------\n");
        printf("[Video format list]\n");

        for(i = 0; ; i++)
        {
            ret = usbh_get_video_format(cur_vdev, i, &format, &width, &height);

            if(ret != 0)
                break;

            if(format == UVC_FORMAT_MJPEG)
                printf("MJPEG, %d x %d\n", width, height);
        }

        printf("----------------------------------------------------------\n\n");
        ret = usbh_set_video_format(cur_vdev, UVC_FORMAT_MJPEG, SELECT_RES_WIDTH, SELECT_RES_HEIGHT);

        if(ret != 0)
            printf("usbh_set_video_format failed! - 0x%x\n", ret);
        else
            printf("MJPEG resolution is set to %d x %d now!\n", SELECT_RES_WIDTH, SELECT_RES_HEIGHT);

        init_image_buffers();

        delay_us(200); /* wait UVC Device command effective. */

        /* assign the first image buffer to receive the image from USB */
        usbh_uvc_set_video_buffer(vdev, _imgs[_idx_usb].buff, IMAGE_MAX_SIZE);
        _imgs[_idx_usb].state = IMAGE_BUFF_USB;

        ret = usbh_uvc_start_streaming(cur_vdev, uvc_rx_callbak);

        if(ret != 0)
        {
            printf("usbh_uvc_start_streaming failed! - %d\n", ret);
            printf("Please re-connect UVC device...\n");
        }
        else
            usbh_memory_used();
    }

    return cur_vdev;
}

int find_jpeg_frame(uint8_t *buf, int len, int *start, int *end)
{
    for(int i = 0; i < len - 1; i++)
    {
        if(buf[i] == 0xFF && buf[i+1] == 0xD8)
        {
            *start = i;
        }
        if(buf[i] == 0xFF && buf[i+1] == 0xD9)
        {
            *end = i + 2;
            return 1;
        }
    }

    return 0;
}

void save_jpeg_to_sd(uint8_t *jpeg_data, int size, int frame_num)
{
    FRESULT res;
    char filename[64];
    UINT bw;

    snprintf(filename, sizeof(filename), "0:/F%04d.JPG", frame_num);
    printf("%s\n", filename);

    res = f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS);
    if(res != FR_OK)
    {
        printf("Open file error! %d\n", res);
        while(1);
    }
    f_write(&file, jpeg_data, size, &bw);
    f_close(&file);
}

void process_mjpeg_stream()
{
    int start = 0, end = 0, frame = 0;

    do
    {
        /* Scanning JPEG frame in the buffer */
        while(find_jpeg_frame(_imgs[_idx_post].buff, IMAGE_MAX_SIZE, &start, &end))
        {
            /* Ignore the first three images */
            if(ignore_img >= 3)
                save_jpeg_to_sd(&_imgs[_idx_post].buff[start], end - start, frame++);

            _imgs[_idx_post].state = IMAGE_BUFF_FREE;
            _idx_post = (_idx_post + 1) % IMAGE_BUFF_CNT;

            ignore_img++;
        }
    } while(_imgs[_idx_post].state == IMAGE_BUFF_READY);
}

int main(void)
{
    UVC_DEV_T       *vdev = NULL;
    TCHAR           sd_path[] = { '0', ':', 0 };    /* SD drive started from 0 */

    SYS_Init();                        /* Init System, IP clock and multi-function I/O */

    UART0_Init();                      /* Initialize UART0 */

    /* Init SD */
    SD_Init();

    enable_sys_tick(100);

    printf("\n\n");
    printf("+------------------------------------------------------------+\n");
    printf("|                                                            |\n");
    printf("|       USB Host UVC Demo MJPEG Format Video Streaming       |\n");
    printf("|                                                            |\n");
    printf("+------------------------------------------------------------+\n");
    printf(" Please insert a microSD card.\n");
    printf(" Press any key to start saving the JPG file.\n\n");

    /* Configure FATFS */
    if(SDH_Open_Disk(SDH0, CardDetect_From_GPIO) < 0)
        while(1);
    f_chdrive(sd_path);          /* Set default path */

    /* USB host init */
    usbh_core_init();
    /* init UVC Class Driver */
    usbh_uvc_init();
    usbh_memory_used();
    /* waiting 2 Sec for UVC Class Device Stable */
    delay_us(2000000);

    /* Clear HyperRAM */
    ClearHyperRAM(HYPERRAM_BASE, HYPERRAM_BASE + 0x800000);

    while(1)
    {
        vdev = uvc_conn_check(vdev);

        if(!kbhit())
        {
            getchar();

            printf("Start vdev->is_streaming:%d\n", vdev->is_streaming);

            g_u32SaveStart = 1;
        }

        if(g_u32SaveStart == 1)
        {
            if(_imgs[_idx_post].state == IMAGE_BUFF_READY)
                process_mjpeg_stream();
        }
    }
}
