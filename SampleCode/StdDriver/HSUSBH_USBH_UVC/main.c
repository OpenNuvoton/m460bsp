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

#define SELECT_STILL_RES_WIDTH  (640)
#define SELECT_STILL_RES_HEIGHT (480)
//#define SELECT_STILL_RES_WIDTH  (320)
//#define SELECT_STILL_RES_HEIGHT (240)

#define IMAGE_MAX_SIZE          (SELECT_STILL_RES_WIDTH * SELECT_STILL_RES_HEIGHT * 2)
#define IMAGE_BUFF_CNT          4

#define MAX_SAVE_FRAMES        300    /* automatically stop after saving this many frames */

#define START_CAPTURE           0x01
#define END_CAPTURE             0x00

#define DEBUG_UVC               0      /* set to 0 to disable diagnostic messages */
#define SAVE_TO_SD              1      /* set to 0 to capture without saving to SD card */

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
struct img_buff_t _imgs[IMAGE_BUFF_CNT];
uint8_t (*image_buff_pool)[IMAGE_BUFF_CNT][IMAGE_MAX_SIZE] __attribute__((aligned(32))) = (uint8_t (*)[IMAGE_BUFF_CNT][IMAGE_MAX_SIZE])HYPERRAM_BASE;
volatile int _idx_usb = 0, _idx_post = 0;
volatile int ignore_img = 0;
int frame = 0;
volatile uint32_t g_u32CbTotal   = 0;    /* total callback invocations (for drop-rate stats) */
volatile uint32_t g_u32CbDropped = 0;    /* dropped frames (next buffer not free)             */
#if DEBUG_UVC
volatile uint32_t g_u32RxCallbackCnt = 0;   /* debug: count uvc_rx_callbak invocations */
#endif

#if DEBUG_UVC
/* Lightweight ring buffer for callback diagnostics (ISR-safe, no printf) */
#define CB_LOG_SIZE  16
struct cb_log_entry {
    uint32_t seq;           /* callback sequence number */
    int      len;           /* received image length    */
    int      idx_usb;       /* _idx_usb at entry        */
    int      idx_post;      /* _idx_post at entry       */
    uint8_t  is_streaming;  /* vdev->is_streaming       */
    uint8_t  dropped;       /* 1 = frame was dropped (next buf not free) */
};
static volatile struct cb_log_entry s_cbLog[CB_LOG_SIZE];
static volatile int s_cbLogHead = 0;   /* written by ISR  */
static volatile int s_cbLogTail = 0;   /* read by main    */

/* Call from main loop to print any pending callback log entries */
void drain_cb_log(void)
{
    while(s_cbLogTail != s_cbLogHead)
    {
        volatile struct cb_log_entry *e = &s_cbLog[s_cbLogTail];
        printf("[CB#%u] len=%d, _idx_usb=%d, _idx_post=%d, is_streaming=%d%s\n",
               e->seq, e->len, e->idx_usb, e->idx_post, e->is_streaming,
               e->dropped ? " (DROPPED)" : "");
        s_cbLogTail = (s_cbLogTail + 1) % CB_LOG_SIZE;
    }
}
#endif /* DEBUG_UVC */

extern int kbhit(void);                        /* function in retarget.c                 */
extern int usbh_uvc_still_image_trigger_control(UVC_DEV_T *vdev, uint8_t capture);
extern void compare_still_probe_commit_lengths(UVC_DEV_T *vdev);
extern int uvc_supports_still_image(void);
extern int usbh_get_video_still_format(struct uvc_dev_t *vdev, int index, IMAGE_FORMAT_E *format, int *width, int *height);
extern int  usbh_set_video_still_format(UVC_DEV_T *vdev, IMAGE_FORMAT_E format, int width, int height);
extern int  usbh_uvc_select_alt_interface(UVC_DEV_T *vdev);
extern int  usbh_uvc_select_still_alt_interface(UVC_DEV_T *vdev);
extern int usbh_uvc_start_still_streaming(UVC_DEV_T *vdev, UVC_CB_FUNC *func);

static volatile uint32_t s_u32TickCnt;
volatile uint32_t g_u32SaveStart = 0;
volatile uint32_t g_u32TimerTrigger = 0;
volatile uint32_t g_u32Only = 0;
volatile uint8_t  g_u8TimerCnt = 0;   /* tick counter for TMR1, reset before each TIMER_Start */

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

    NVIC_SetPriority(SDH0_IRQn, 4);
}

void TMR1_IRQHandler(void)
{
    if(TIMER_GetIntFlag(TIMER1) == 1)
    {
        /* Clear Timer1 time-out interrupt flag */
        TIMER_ClearIntFlag(TIMER1);
        g_u8TimerCnt++;

        if(g_u8TimerCnt >= 3)
        {
            g_u32TimerTrigger = 1;
            g_u8TimerCnt = 0;
            // printf("TIMER1 -> still capture triggered!\n");
        }
    }
}

void TMR1_Init(void)
{
    /* Select TMR1 module clock source as PCLK0 */
    CLK_SetModuleClock(TMR1_MODULE, CLK_CLKSEL1_TMR1SEL_PCLK0, 0);
    /* Enable TMR1 module clock */
    CLK_EnableModuleClock(TMR1_MODULE);

    /* Open Timer1 in periodic mode, enable interrupt and 1 interrupt tick per second */
    TIMER_Open(TIMER1, TIMER_PERIODIC_MODE, 1);
    TIMER_EnableInt(TIMER1);

    /* Enable Timer1 NVIC */
    NVIC_EnableIRQ(TMR1_IRQn);

    NVIC_SetPriority(TMR1_IRQn, 3);
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

void init_image_buffers(void)
{
    int i;

    for(i = 0; i < IMAGE_BUFF_CNT; i++)
    {
        _imgs[i].buff = (uint8_t *)image_buff_pool[i];
        _imgs[i].len = 0;
        _imgs[i].state = IMAGE_BUFF_FREE;
    }

    _idx_usb = 0;
    _idx_post = 0;
}

int uvc_rx_callbak(UVC_DEV_T *vdev, uint8_t *data, int len)
{
    int next_idx;

    (void)(data);

    g_u32CbTotal++;

#if DEBUG_UVC
    g_u32RxCallbackCnt++;

    /* Log to ring buffer (ISR-safe, no printf here!) */
    {
        int head = s_cbLogHead;
        volatile struct cb_log_entry *e = &s_cbLog[head];
        e->seq          = g_u32RxCallbackCnt;
        e->len          = len;
        e->idx_usb      = _idx_usb;
        e->idx_post     = _idx_post;
        e->is_streaming = vdev->is_streaming;
        e->dropped      = 0;  /* updated below if dropped */
        s_cbLogHead = (head + 1) % CB_LOG_SIZE;
    }
#endif

    next_idx = (_idx_usb + 1) % IMAGE_BUFF_CNT;

    if(_imgs[next_idx].state != IMAGE_BUFF_FREE)
    {
        /*
         *  Next image buffer is in used.
         *  Just drop this newly received image and reuse the same image buffer.
         */
        g_u32CbDropped++;
#if DEBUG_UVC
        s_cbLog[(s_cbLogHead + CB_LOG_SIZE - 1) % CB_LOG_SIZE].dropped = 1;
#endif
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
    /* Track USB device number of the last known device.
     * usbh_pooling_hubs() may process a disconnect+reconnect in a single call;
     * the allocator can then reuse the same UVC_DEV_T memory for the new device,
     * making cur_vdev == vdev even though it is a different USB device.
     * Comparing dev_num (monotonically increasing USB address) detects this. */
    static uint8_t s_last_dev_num = 0;

    if(usbh_pooling_hubs())       /* USB Host port detect polling and management */
    {
        /*
         *  Has hub port event.
         */
        vdev = usbh_uvc_get_device_list();

        if(vdev == NULL)
        {
            cur_vdev = NULL;
            s_last_dev_num = 0;
            printf("\n[No UVC device connected]\n\n");
            return NULL;
        }

        /* Pointer-reuse guard: same pointer but different USB address means
         * a disconnect+reconnect was silently processed inside usbh_pooling_hubs().
         * Force the new-device enumeration path. */
        if(cur_vdev == vdev && vdev->udev->dev_num != s_last_dev_num)
            cur_vdev = NULL;

        if(cur_vdev == vdev)
        {
            printf("\n\n\nWaiting for UVC device connected...\n");
            /* If a timer restart is pending and the device is already streaming,
             * start the timer now (handles reconnect after still-capture cycle). */
            if(g_u32Only == 1 && vdev != NULL && vdev->is_streaming)
            {
                printf("Camera streaming resumed. Timer begins now.\n");
                g_u32Only = 0;
                g_u8TimerCnt = 0;      /* reset tick counter before starting */
                TIMER_Start(TIMER1);
            }
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
        s_last_dev_num = vdev->udev->dev_num;   /* record device address for future reuse detection */
#if DEBUG_UVC
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

        printf("----------------------------------------------------------\n");
#endif
        ret = usbh_set_video_format(cur_vdev, UVC_FORMAT_MJPEG, SELECT_RES_WIDTH, SELECT_RES_HEIGHT);

        if(ret != 0)
            printf("usbh_set_video_format failed! - 0x%x\n", ret);
        else
            printf("MJPEG resolution is set to %d x %d now!\n\n", SELECT_RES_WIDTH, SELECT_RES_HEIGHT);


STREAMING:
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
        {
            usbh_memory_used();
            /* Start still-image timer only after streaming is confirmed active */
            if(g_u32Only == 1)
            {
                /* First time: timer had not been started yet */
                printf("Camera streaming started. Timer begins now.\n");
                g_u32Only = 0;
                g_u8TimerCnt = 0;          /* reset tick counter before starting */
                TIMER_Start(TIMER1);
            }
            else if(g_u32SaveStart == 0)
            {
                /* Reconnect during mode-1 cycle: timer was already running but
                 * accumulated ticks during re-enumeration. Stop and restart clean
                 * so the next still capture is triggered after a full 10 seconds. */
                printf("Camera streaming resumed. Timer reset.\n");
                TIMER_Stop(TIMER1);
                g_u8TimerCnt = 0;          /* reset tick counter before restarting */
                TIMER_Start(TIMER1);
            }
        }
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

void save_jpeg_to_sd(uint8_t *jpeg_data, int size, int frame_num, int is_still)
{
    FRESULT res;
    char filename[64];
    UINT bw;

    if(is_still)
        snprintf(filename, sizeof(filename), "0:/F%04dS.JPG", frame_num);
    else
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
    int start = 0, end = 0/*, frame = 0*/;
    static uint32_t s_lastStatsTick = 0;

    do
    {
        /* Scanning JPEG frame in the buffer */
        while(find_jpeg_frame(_imgs[_idx_post].buff, IMAGE_MAX_SIZE, &start, &end))
        {
            /* ---- Drop-rate report every 10 s (1000 ticks @ 100 Hz) ---- */
            {
                uint32_t now = get_ticks();
                if(s_lastStatsTick == 0)
                    s_lastStatsTick = now;          /* initialise on first iteration */
                if(now - s_lastStatsTick >= 1000)   /* 1000 ticks = 10 s */
                {
                    uint32_t total   = g_u32CbTotal;
                    uint32_t dropped = g_u32CbDropped;
                    /* NOTE: Use higher speed SD Card could reduce drop rate */
                    if(total > 0)
                        printf("[DropStats] %u/%u dropped in last 10 s (%u%%)\n",
                               dropped, total, dropped * 100 / total);
                    else
                        printf("[DropStats] no callbacks in last 10 s\n");
                    g_u32CbTotal   = 0;
                    g_u32CbDropped = 0;
                    s_lastStatsTick = now;
                }
            }

            if((g_u32TimerTrigger == 0) && (g_u32SaveStart == 0))
                break;

#if DEBUG_UVC
            printf("_idx_post %d; state %d; Timer %d\n", _idx_post, _imgs[_idx_post].state, g_u32TimerTrigger);
#endif

#if SAVE_TO_SD
                save_jpeg_to_sd(&_imgs[_idx_post].buff[start], end - start, frame++, 0);
#else
                frame++;  /* increment frame counter even when not saving */
#endif

            _imgs[_idx_post].state = IMAGE_BUFF_FREE;
            _idx_post = (_idx_post + 1) % IMAGE_BUFF_CNT;

            ignore_img++;

            /* For still image mode: clear trigger after saving one image so the
             * next inner-while iteration hits the (TimerTrigger==0 && SaveStart==0)
             * break and the function exits cleanly. */
            if(g_u32SaveStart == 0)
                g_u32TimerTrigger = 0;

            /* Stop automatically after MAX_SAVE_FRAMES */
            if(frame >= MAX_SAVE_FRAMES)
            {
                printf("\n*** Saved %d frames. Auto-stop. ***\n", frame);
                g_u32TimerTrigger = 0;
                g_u32SaveStart    = 0;
                return;
            }
        }
    } while(((_imgs[_idx_post].state == IMAGE_BUFF_READY) && (g_u32SaveStart == 1)) || (g_u32TimerTrigger == 1));
}

int main(void)
{
    UVC_DEV_T       *vdev = NULL;
    TCHAR           sd_path[] = { '0', ':', 0 };    /* SD drive started from 0 */
    static uint32_t u32Item;

    IMAGE_FORMAT_E  format;
    int width, height;
    int i, ret;

    SYS_Init();                        /* Init System, IP clock and multi-function I/O */

    UART0_Init();                      /* Initialize UART0 */

#if SAVE_TO_SD
    /* Init SD */
    SD_Init();
#endif

    /* Init TMR1 */
    TMR1_Init();

    enable_sys_tick(100);

    printf("\n\n");
    printf("+------------------------------------------------------------+\n");
    printf("|                                                            |\n");
    printf("|       USB Host UVC Demo MJPEG Format Video Streaming       |\n");
    printf("|                                                            |\n");
    printf("+------------------------------------------------------------+\n");
    printf(" Please insert a microSD card.\n\n");
    printf(" Please choose demo item and press any key to start saving the JPG file:\n");
    printf(" [1] Still image capture\n");
    printf(" [2] Video streaming capture\n\n");

#if SAVE_TO_SD
    /* Configure FATFS */
    if(SDH_Open_Disk(SDH0, CardDetect_From_GPIO) < 0)
        while(1);
    f_chdrive(sd_path);          /* Set default path */
#endif

    /* USB host init */
    usbh_core_init();
    /* init UVC Class Driver */
    usbh_uvc_init();
    usbh_memory_used();
    /* waiting 2 Sec for UVC Class Device Stable */
    delay_us(2000000);

    /* Clear HyperRAM */
    ClearHyperRAM(HYPERRAM_BASE, HYPERRAM_BASE + 0x800000);

    u32Item = getchar();

    printf("\nInput item [%c]\n\n", u32Item);

    switch(u32Item)
    {
        case '1':
            /* Mark that timer should start once camera begins streaming */
            g_u32Only = 1;
            break;

        case '2':
            g_u32SaveStart = 1;
            break;
    }

    while(1)
    {
        /* ---- Drop-rate report every 10 s (1000 ticks @ 100 Hz) ---- */
        {
            static uint32_t s_lastStatsTick = 0;
            uint32_t now = get_ticks();
            if(s_lastStatsTick == 0)
                s_lastStatsTick = now;          /* initialise on first iteration */
            if(now - s_lastStatsTick >= 1000)   /* 1000 ticks = 10 s */
            {
                uint32_t total   = g_u32CbTotal;
                uint32_t dropped = g_u32CbDropped;
                /* NOTE: Use higher speed SD Card could reduce drop rate */
                if(total > 0)
                    printf("[DropStats] %u/%u dropped in last 10 s (%u%%)\n",
                           dropped, total, dropped * 100 / total);
                else
                    printf("[DropStats] no callbacks in last 10 s\n");
                g_u32CbTotal   = 0;
                g_u32CbDropped = 0;
                s_lastStatsTick = now;
            }
        }

        vdev = uvc_conn_check(vdev);

        /* Stop the timer as soon as the camera is gone to prevent it from
         * firing during re-enumeration. It will be re-armed by uvc_conn_check
         * once the new camera is confirmed streaming (g_u32Only mechanism). */
        if(vdev == NULL)
        {
            TIMER_Stop(TIMER1);
            if(g_u32SaveStart == 0)  /* mode 1 only: re-arm timer on reconnect */
                g_u32Only = 1;
        }

        if(g_u32TimerTrigger == 1)
        {
            TIMER_Stop(TIMER1);

#if 1

        if(vdev == NULL || !vdev->is_streaming)
        {
            /* Camera not ready yet, reset trigger and wait for next cycle */
            printf("[Timer] Camera not streaming, waiting...\n");
            g_u32TimerTrigger = 0;
            g_u32Only = 1;  /* re-arm: start timer once camera connects */
            continue;
        }

        usbh_uvc_stop_streaming(vdev);
        while(vdev->is_streaming);
#if DEBUG_UVC
        printf("\n\n----------------------------------------------------------\n");
        printf("[Video still format list]\n");

        for(i = 0; ; i++)
        {
            ret = usbh_get_video_still_format(vdev, i, &format, &width, &height);

            if(ret != 0)
                break;

            if(format == UVC_FORMAT_MJPEG)
                printf("MJPEG, %d x %d\n", width, height);
        }

        printf("----------------------------------------------------------\n");
#endif
        ret = usbh_set_video_still_format(vdev, UVC_FORMAT_MJPEG, SELECT_STILL_RES_WIDTH, SELECT_STILL_RES_HEIGHT);

        if(ret != 0)
            printf("usbh_set_video_still_format failed! - 0x%x\n", ret);
        else
            printf("MJPEG still resolution is set to %d x %d now!\n\n", SELECT_STILL_RES_WIDTH, SELECT_STILL_RES_HEIGHT);

//        compare_still_probe_commit_lengths(vdev);
#endif

            /* Re-initialize image buffers before still streaming.
             * Reference program always calls init_image_buffers() +
             * usbh_uvc_set_video_buffer() before every usbh_uvc_start_streaming().
             * Without this, _idx_usb/_idx_post are misaligned: the still frame
             * lands in _imgs[_idx_usb].buff but process_mjpeg_stream() waits on
             * a different _idx_post, and the driver may write into a dirty buffer. */
            init_image_buffers();
            delay_us(200);
            usbh_uvc_set_video_buffer(vdev, _imgs[_idx_usb].buff, IMAGE_MAX_SIZE);
            _imgs[_idx_usb].state = IMAGE_BUFF_USB;

#if DEBUG_UVC
            drain_cb_log();            /* flush stale entries from previous video streaming */
            s_cbLogHead = s_cbLogTail = 0;  /* reset ring buffer for clean still-capture diagnostics */
            printf("\n[Still-Diag] Before start_still_streaming:\n");
            printf("  is_streaming=%d, img_buff=%p, img_buff_size=%d, img_size=%d\n",
                   vdev->is_streaming, vdev->img_buff, vdev->img_buff_size, vdev->img_size);
            printf("  ep_iso_in=%p", vdev->ep_iso_in);
            if(vdev->ep_iso_in)
                printf(" (bEndpointAddress=0x%02x, wMaxPacketSize=%d)",
                       vdev->ep_iso_in->bEndpointAddress, vdev->ep_iso_in->wMaxPacketSize);
            printf("\n");
            printf("  _idx_usb=%d, _idx_post=%d\n", _idx_usb, _idx_post);
            for(int bi = 0; bi < IMAGE_BUFF_CNT; bi++)
                printf("  _imgs[%d]: state=%d, len=%d, buff=%p\n", bi, _imgs[bi].state, _imgs[bi].len, _imgs[bi].buff);

            g_u32RxCallbackCnt = 0;  /* reset callback counter before still streaming */
#endif
            ret = usbh_uvc_start_still_streaming(vdev, uvc_rx_callbak);
#if DEBUG_UVC
            printf("[Still-Diag] usbh_uvc_start_still_streaming returned %d\n", ret);
            printf("  is_streaming=%d after start_still_streaming\n", vdev->is_streaming);
            if(ret != 0)
                printf("[Still-Diag] ERROR: start_still_streaming failed!\n");
#endif
            if(!vdev->is_streaming)
            {
#if DEBUG_UVC
                printf("[Still-Diag] ERROR: is_streaming=0 after start_still_streaming!\n");
#endif
                break;
            }

            /* CRITICAL: Allow EHCI ISO pipe to fully activate before sending
             * trigger.  After start_still_streaming selects the alt interface
             * and submits ISO URBs, the EHCI controller needs several
             * micro-frames (~125 us each) to schedule the periodic transfers.
             * Without this delay the camera receives the trigger and sends the
             * still frame before the host ISO pipe is ready, causing the data
             * to be silently lost and a timeout in the wait loop below.
             * 20 ms is generous; testing showed ~10 ms is the minimum. */
            delay_us(20000);

            ret = usbh_uvc_still_image_trigger_control(vdev, START_CAPTURE);
#if DEBUG_UVC
            printf("Issuing a VS_STILL_IMAGE_TRIGGER_CONTROL, ret=%d\n", ret);
            if(ret != 0)
                printf("[Still-Diag] ERROR: still_image_trigger_control(START) failed! ret=%d\n", ret);
            printf("[Still-Diag] After trigger: is_streaming=%d, img_size=%d, cb_count=%u\n",
                   vdev->is_streaming, vdev->img_size, g_u32RxCallbackCnt);
#endif

            /* Wait for still frame following the reference-program pattern:
             * - First confirm IMAGE_BUFF_READY (never scan a buffer before it is ready)
             * - Use _imgs[].len (actual received size) for find_jpeg_frame scan range
             * - Transition state READY -> FREE after processing (same as reference) */
            {
                uint32_t t0 = get_ticks();
#if DEBUG_UVC
                uint32_t last_print = t0;
                printf("[Still-Diag] Waiting for still frame... (t0=%u)\n", t0);
#endif
                while(_imgs[_idx_post].state != IMAGE_BUFF_READY)
                {
                    uint32_t now = get_ticks();
#if DEBUG_UVC
                    drain_cb_log();   /* print any pending callback log entries */
                    /* Print periodic status every ~500ms (50 ticks at 100 Hz) */
                    if(now - last_print >= 50)
                    {
                        printf("[Still-Wait] elapsed=%ums, cb_count=%u, is_streaming=%d, img_size=%d\n",
                               (now - t0) * 10, g_u32RxCallbackCnt, vdev->is_streaming, vdev->img_size);
                        printf("  _idx_usb=%d, _idx_post=%d\n", _idx_usb, _idx_post);
                        for(int bi = 0; bi < IMAGE_BUFF_CNT; bi++)
                            printf("  _imgs[%d]: state=%d, len=%d\n", bi, _imgs[bi].state, _imgs[bi].len);
                        if(vdev->ep_iso_in)
                            printf("  ep_iso_in: addr=0x%02x, wMaxPacketSize=%d, hw_pipe=0x%x\n",
                                   vdev->ep_iso_in->bEndpointAddress,
                                   vdev->ep_iso_in->wMaxPacketSize,
                                   (unsigned int)(uintptr_t)vdev->ep_iso_in->hw_pipe);
                        last_print = now;
                    }
#endif
                    if(now - t0 > 300)   /* 3-second timeout (SysTick = 100 Hz) */
                    {
                        printf("\n[Still] *** TIMEOUT waiting for still frame! ***\n");
#if DEBUG_UVC
                        printf("[Timeout-Dump] Total elapsed: %ums\n", (now - t0) * 10);
                        printf("[Timeout-Dump] Callback invocations: %u\n", g_u32RxCallbackCnt);
                        printf("[Timeout-Dump] is_streaming=%d, img_size=%d\n",
                               vdev->is_streaming, vdev->img_size);
                        printf("[Timeout-Dump] img_buff=%p, img_buff_size=%d\n",
                               vdev->img_buff, vdev->img_buff_size);
                        printf("[Timeout-Dump] _idx_usb=%d, _idx_post=%d\n", _idx_usb, _idx_post);
                        for(int bi = 0; bi < IMAGE_BUFF_CNT; bi++)
                            printf("[Timeout-Dump] _imgs[%d]: state=%d, len=%d, buff=%p\n",
                                   bi, _imgs[bi].state, _imgs[bi].len, _imgs[bi].buff);
                        if(vdev->ep_iso_in)
                            printf("[Timeout-Dump] ep_iso_in: addr=0x%02x, wMaxPacketSize=%d, hw_pipe=0x%x\n",
                                   vdev->ep_iso_in->bEndpointAddress,
                                   vdev->ep_iso_in->wMaxPacketSize,
                                   (unsigned int)(uintptr_t)vdev->ep_iso_in->hw_pipe);
                        else
                            printf("[Timeout-Dump] ep_iso_in is NULL!\n");
                        /* Check first 16 bytes of the image buffer for any partial data */
                        printf("[Timeout-Dump] img_buff first 16 bytes: ");
                        for(int di = 0; di < 16 && di < vdev->img_buff_size; di++)
                            printf("%02x ", vdev->img_buff[di]);
                        printf("\n");
#endif
                        break;
                    }
                }
#if DEBUG_UVC
                drain_cb_log();  /* print the still-capture callback entry */
#endif
                if(_imgs[_idx_post].state == IMAGE_BUFF_READY)
                {
                    int jpeg_start = 0, jpeg_end = 0;
#if DEBUG_UVC
                    printf("[Still-Diag] Frame received! _idx_post=%d, len=%d, cb_count=%u\n",
                           _idx_post, _imgs[_idx_post].len, g_u32RxCallbackCnt);
#endif
                    if(find_jpeg_frame(_imgs[_idx_post].buff, _imgs[_idx_post].len,
                                       &jpeg_start, &jpeg_end))
                    {
#if SAVE_TO_SD
                        save_jpeg_to_sd(&_imgs[_idx_post].buff[jpeg_start],
                                        jpeg_end - jpeg_start, frame++, 1);
#else
                        frame++;  /* increment frame counter even when not saving */
#endif
                    }
                    else
                        printf("[Still] No valid JPEG found in still frame buffer!\n");
                    _imgs[_idx_post].state = IMAGE_BUFF_FREE;
                    _idx_post = (_idx_post + 1) % IMAGE_BUFF_CNT;
                }
            }

            /* Send END_CAPTURE to release camera from still image mode */
            usbh_uvc_still_image_trigger_control(vdev, END_CAPTURE);
            delay_us(200000); /* 200ms: let camera process END_CAPTURE */

            /* Stop still streaming and resume normal video streaming */
            usbh_uvc_stop_streaming(vdev);
            while(vdev->is_streaming);

            /* Check if auto-stop limit reached */
            if(frame >= MAX_SAVE_FRAMES)
                break;

            /* Re-initialize image buffers before resuming video streaming.
             * Reference program always resets buffers before every streaming start. */
            init_image_buffers();
            delay_us(200);
            usbh_uvc_set_video_buffer(vdev, _imgs[_idx_usb].buff, IMAGE_MAX_SIZE);
            _imgs[_idx_usb].state = IMAGE_BUFF_USB;
            usbh_uvc_start_streaming(vdev, uvc_rx_callbak);

            g_u32TimerTrigger = 0;
            g_u8TimerCnt = 0;          /* reset tick counter before restarting */
            TIMER_Start(TIMER1);
        }

        if(g_u32SaveStart == 1)
        {
            if(vdev == NULL || !vdev->is_streaming)
                break;
#if DEBUG_UVC
            drain_cb_log();   /* print any pending callback log entries */
#endif
            if(_imgs[_idx_post].state == IMAGE_BUFF_READY)
            {
                process_mjpeg_stream();
                if(frame >= MAX_SAVE_FRAMES)
                    break;
            }
        }

        /* Mode 1: also save video frames while waiting for still capture timer.
         * The camera is streaming (Alt 2) between still captures; save those
         * frames to SD card just like mode 2 does. */
        if(g_u32SaveStart == 0 && g_u32TimerTrigger == 0 &&
           vdev != NULL && vdev->is_streaming)
        {
#if DEBUG_UVC
            drain_cb_log();   /* print any pending callback log entries */
#endif
            if(_imgs[_idx_post].state == IMAGE_BUFF_READY)
            {
                int jpeg_start = 0, jpeg_end = 0;

                if(find_jpeg_frame(_imgs[_idx_post].buff, IMAGE_MAX_SIZE,
                                   &jpeg_start, &jpeg_end))
                {
#if SAVE_TO_SD
                    save_jpeg_to_sd(&_imgs[_idx_post].buff[jpeg_start],
                                    jpeg_end - jpeg_start, frame++, 0);
#else
                    frame++;  /* increment frame counter even when not saving */
#endif
                }

                _imgs[_idx_post].state = IMAGE_BUFF_FREE;
                _idx_post = (_idx_post + 1) % IMAGE_BUFF_CNT;

                if(frame >= MAX_SAVE_FRAMES)
                    break;
            }
        }
    }

    /* Finished - stop everything */
    if(vdev != NULL && vdev->is_streaming)
        usbh_uvc_stop_streaming(vdev);
    TIMER_Stop(TIMER1);
    printf("\n+------------------------------------------------------------+\n");
#if SAVE_TO_SD
    printf("|  Capture complete: %4d frames saved to SD card.           |\n", frame);
#else
    printf("|  Capture complete: %4d frames processed.                   |\n", frame);
#endif
    printf("+------------------------------------------------------------+\n");

    while(1);  /* Halt */
}
