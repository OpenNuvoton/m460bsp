/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    Use USB Host core driver and HID driver. This sample demonstrates how
 *           to submit HID class request and how to read data from interrupt pipe.
 *           This sample supports dynamic device plug/un-plug and multiple HID devices.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"

#include "usbh_lib.h"
#include "usbh_hid.h"

// #define HAVE_INT_OUT

#ifdef __ICCARM__
#pragma data_alignment=32
static uint8_t s_au8BuffPool[1024];
#else
static uint8_t s_au8BuffPool[1024] __attribute__((aligned(32)));
#endif

static HID_DEV_T *s_hid_list[CONFIG_HID_MAX_DEV];

extern int kbhit(void);                        /* function in retarget.c                 */

static volatile uint32_t s_u32TickCnt;

static volatile uint32_t s_u32IntCnt = 0;

void SysTick_Handler(void);
void enable_sys_tick(int ticks_per_second);
void dump_buff_hex(uint8_t *pu8Buff, int i8Bytes);
int is_a_new_hid_device(HID_DEV_T *hdev);
void update_hid_device_list(HID_DEV_T *hdev);
void int_read_callback(HID_DEV_T *hdev, uint16_t u16EpAddr, int i8Status, uint8_t *pu8RData, uint32_t u32DataLen);
#ifdef HAVE_INT_OUT
void int_write_callback(HID_DEV_T *hdev, uint16_t u16EpAddr, int i8Status, uint8_t *pu8WBuff, uint32_t *pu32DataLen);
#endif
int init_hid_device(HID_DEV_T *hdev);
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
    s_u32IntCnt++;
}

#ifdef HAVE_INT_OUT
void int_write_callback(HID_DEV_T *hdev, uint16_t u16EpAddr, int i8Status, uint8_t *pu8WBuff, uint32_t *pu32DataLen)
{
    int i8MaxLen = (int) * pu32DataLen;

    (void)i8Status;

    printf("Device [0x%x,0x%x] ep 0x%x, ask user to fill data buffer and length.\n",
           hdev->idVendor, hdev->idProduct, u16EpAddr);

    memset(pu8WBuff, 0, (uint32_t)i8MaxLen);         /* Fill data to be sent via interrupt out pipe     */

    *pu32DataLen = (uint32_t)i8MaxLen;               /* Tell HID driver transfer length of this time    */
}
#endif

int init_hid_device(HID_DEV_T *hdev)
{
    uint8_t *pu8DataBuff;
    int i8Cnt, i8Ret;

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

    /*
     *  Example: GET_PROTOCOL request.
     */
    i8Ret = usbh_hid_get_protocol(hdev, pu8DataBuff);
    printf("[GET_PROTOCOL] ret = %d, protocol = %d\n", i8Ret, pu8DataBuff[0]);

    /*
     *  Example: SET_PROTOCOL request.
     */
    i8Ret = usbh_hid_set_protocol(hdev, pu8DataBuff[0]);
    printf("[SET_PROTOCOL] ret = %d, protocol = %d\n", i8Ret, pu8DataBuff[0]);

    /*
     *  Example: GET_REPORT request on report ID 0x1, report type FEATURE.
     */
    i8Ret = usbh_hid_get_report(hdev, RT_FEATURE, 0x1, pu8DataBuff, 64);
    if(i8Ret > 0)
    {
        printf("[GET_REPORT] Data => ");
        for(i8Cnt = 0; i8Cnt < i8Ret; i8Cnt++)
            printf("%02x ", pu8DataBuff[i8Cnt]);
        printf("\n");
    }

    printf("\nUSBH_HidStartIntReadPipe...\n");
    for(i8Cnt = 0; i8Cnt < 2; i8Cnt++)
    {
        /* use ping-pong buffer transfer */
        i8Ret = usbh_hid_start_int_read(hdev, 0, int_read_callback);
        if(i8Ret != HID_RET_OK)
        {
            printf("usbh_hid_start_int_read failed! %d\n", i8Ret);
            while(1);
        }
        else
            printf("Interrupt in transfer started...\n");
    }

#ifdef HAVE_INT_OUT
    i8Ret = usbh_hid_start_int_write(hdev, 0, int_write_callback);
    if((i8Ret != HID_RET_OK) && (i8Ret != HID_RET_XFER_IS_RUNNING))
        printf("usbh_hid_start_int_write failed!\n");
    else
        printf("Interrupt out transfer started...\n");
#endif

    return 0;
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

int main(void)
{
    HID_DEV_T *hdev, *hdev_list;
    uint32_t u32T0;

    SYS_Init();                        /* Init System, IP clock and multi-function I/O */

    UART0_Init();                      /* Initialize UART0 */

    enable_sys_tick(100);

    printf("\n\n");
    printf("+--------------------------------------------+\n");
    printf("|                                            |\n");
    printf("|       USB Host HID class sample demo       |\n");
    printf("|                                            |\n");
    printf("+--------------------------------------------+\n");

    usbh_core_init();
    usbh_hid_init();
    usbh_memory_used();

    memset(s_hid_list, 0, sizeof(s_hid_list));
    u32T0 = s_u32TickCnt;

    while(1)
    {
        if(usbh_pooling_hubs())              /* USB Host port detect polling and management */
        {
            usbh_memory_used();              /* print out USB memory allocating information */

            printf("\n Has hub events.\n");
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
            usbh_memory_used();
        }

        if(s_u32TickCnt - u32T0 >= 100)
        {
            u32T0 = s_u32TickCnt;
            printf("%d \n", s_u32IntCnt);
            s_u32IntCnt = 0;
        }

#ifndef DEBUG_ENABLE_SEMIHOST
        if(!kbhit())
        {
            getchar();
            usbh_memory_used();
        }
#endif
    }
}
