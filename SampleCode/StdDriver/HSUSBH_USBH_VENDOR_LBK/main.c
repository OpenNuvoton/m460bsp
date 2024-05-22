/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    This sample shows how to do transfers on a known device with
 *           a vendor driver. This sample requires a USB device running
 *           sample HSUSBD_VENDOR_LBK or USBD_VENDOR_LBK to be connected.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include <string.h>

#include "NuMicro.h"

#include "usb.h"
#include "hub.h"
#include "lbk_driver.h"

#ifdef DEBUG_ENABLE_SEMIHOST
#error This sample cannot execute with semihost enabled
#endif

extern int kbhit(void);    /* in retarget.c */

static volatile int s_i8HasError;
static volatile int s_i8IntInCnt, s_i8IntOutCnt;
static volatile int s_i8IsoInCnt, s_i8IsoOutCnt;
static volatile uint32_t s_u32TickCnt;

void SysTick_Handler(void);
void enable_sys_tick(int ticks_per_second);
void SYS_Init(void);
void UART0_Init(void);
void connect_func(UDEV_T *udev, int param);
void disconnect_func(UDEV_T *udev, int param);
void demo_ctrl_xfer(void);
void demo_bulk_xfer(void);
int int_in_callback(int status, uint8_t *rdata, int data_len);
int int_out_callback(int status, uint8_t *rdata, int data_len);
void demo_interrupt_xfer(void);
int iso_in_callback(uint8_t *rdata, int data_len);
int iso_out_callback(uint8_t *rdata, int data_len);
void demo_isochronous_xfer(void);
void vendor_lbk_demo(void);

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

/*
 *  USB device connect callback function.
 *  User invokes usbh_pooling_hubs() to let USB core able to scan and handle events of
 *  HSUSBH port, USBH port, and USB hub device ports. Once a new device connected, it
 *  will be detected and enumerated in the call to usbh_pooling_hubs(). This callback
 *  will be invoked from USB core once a newly connected device was successfully enumerated.
 */
void connect_func(UDEV_T *udev, int param)
{
    struct hub_dev_t *parent;
    int i;

    (void)param;

    parent = udev->parent;

    printf("Device [0x%x,0x%x] was connected.\n",
           udev->descriptor.idVendor, udev->descriptor.idProduct);
    printf("    Speed:    %s-speed\n", (udev->speed == SPEED_HIGH) ? "high" : ((udev->speed == SPEED_FULL) ? "full" : "low"));
    printf("    Location: ");

    if(parent == NULL)
    {
        if(udev->port_num == 1)
            printf("USB 2.0 port\n");
        else
            printf("USB 1.1 port\n");
    }
    else
    {
        if(parent->pos_id[0] == '1')
            printf("USB 2.0 port");
        else
            printf("USB 1.1 port");

        for(i = 1; parent->pos_id[i] != 0; i++)
        {
            printf(" => Hub port %c", parent->pos_id[i]);
        }

        printf(" => Hub port %d\n", udev->port_num);

        printf("\n");
    }
    printf("\n");
}

/*
 *  USB device disconnect callback function.
 *  User invokes usbh_pooling_hubs() to let USB core able to scan and handle events of
 *  HSUSBH port, USBH port, and USB hub device ports. Once a device was disconnected, it
 *  will be detected and removed in the call to usbh_pooling_hubs(). This callback
 *  will be invoked from USB core prior to remove that device.
 */
void disconnect_func(UDEV_T *udev, int param)
{
    (void)param;

    printf("Device [0x%x,0x%x] was disconnected.\n",
           udev->descriptor.idVendor, udev->descriptor.idProduct);
}

void demo_ctrl_xfer(void)
{
    uint32_t loop, msg_tick;
    uint8_t buff_out[64], buff_in[64];

    printf("\nPress 'x' to stop loop...\n\n");
    msg_tick = get_ticks();

    for(loop = 1; ; loop++)
    {
        if(!kbhit())
        {
            if(getchar() == 'x')
                return;
        }

        memset(buff_out, (loop & 0xff), sizeof(buff_out));

        if(!lbk_device_is_connected())
            return;

        if(lbk_vendor_set_data(buff_out) != 0)
        {
            printf("Control-out transfer failed. Stop control transfer loop.\n");
            return;
        }

        if(!lbk_device_is_connected())
            return;

        if(lbk_vendor_get_data(buff_in) != 0)
        {
            printf("Control-in transfer failed. Stop control transfer loop.\n");
            return;
        }

        if(memcmp(buff_out, buff_in, 64) != 0)
        {
            printf("CTRL data compare error!\n");
        }

        if(get_ticks() - msg_tick >= 100)
        {
            printf("Control transfer loop count %d.     \r", loop);
            msg_tick = get_ticks();
        }
    }
}

void demo_bulk_xfer(void)
{
    uint32_t loop, msg_tick, xfer_len;
    uint8_t buff_out[512], buff_in[512];

    printf("\nPress 'x' to stop loop...\n\n");
    msg_tick = get_ticks();

    if(lbk_device_is_high_speed())
        xfer_len = 512;
    else
        xfer_len = 64;

    for(loop = 1; ; loop++)
    {
        if(!kbhit())
        {
            if(getchar() == 'x')
                return;
        }

        memset(buff_out, (loop & 0xff), xfer_len);

        if(!lbk_device_is_connected())
            return;

        if(lbk_bulk_write(buff_out, (int)xfer_len, 100) != 0)
        {
            printf("Bulk-out transfer failed. Stop bulk transfer loop.\n");
            return;
        }

        if(!lbk_device_is_connected())
            return;

        if(lbk_bulk_read(buff_in, (int)xfer_len, 100) != 0)
        {
            printf("Bulk-in transfer failed. Stop bulk transfer loop.\n");
            return;
        }

        if(get_ticks() - msg_tick >= 100)
        {
            printf("Bulk transfer loop count %d.    \r", loop);
            msg_tick = get_ticks();
        }
    }
}

int int_in_callback(int status, uint8_t *rdata, int data_len)
{
    (void)rdata;
    (void)data_len;

    if(status < 0)
    {
        printf("Interrupt-in trnasfer error %d!\n", status);
        s_i8HasError = 1;
        return 0;
    }
    s_i8IntInCnt++;
    return 0;
}

int int_out_callback(int status, uint8_t *rdata, int data_len)
{
    if(status < 0)
    {
        printf("interrupt out transfer error.\n");
        s_i8HasError = 1;
        return 0;
    }

    /* add code here to send data to device */
    /* ... */
    memset(rdata, (s_i8IntOutCnt & 0xff), (uint32_t)data_len);
    s_i8IntOutCnt++;
    return data_len;
}

void demo_interrupt_xfer(void)
{
    uint32_t loop, msg_tick;

    printf("\nPress 'x' to stop loop...\n\n");
    msg_tick = get_ticks();

    s_i8IntInCnt = 0;
    s_i8IntOutCnt = 0;
    s_i8HasError = 0;

    lbk_interrupt_in_start(int_in_callback);
    lbk_interrupt_out_start(int_out_callback);

    for(loop = 1; ; loop++)
    {
        if(!kbhit())
        {
            if(getchar() == 'x')
            {
                lbk_interrupt_in_stop();
                lbk_interrupt_out_stop();
                return;
            }
        }

        if(!lbk_device_is_connected() || s_i8HasError)
            return;

        if(get_ticks() - msg_tick >= 100)
        {
            printf("Interrupt transfer loop RX: %d, TX: %d    \r", s_i8IntInCnt, s_i8IntOutCnt);
            msg_tick = get_ticks();
        }
    }
}

/*
 *  rdata    - isochronous in data buffer
 *  data_len - length of data received
 *  return value - not used
 */
int iso_in_callback(uint8_t *rdata, int data_len)
{
    (void)rdata;
    (void)data_len;

    /* add code here to collect data recevied from device */
    /* ... */
    s_i8IsoInCnt++;
    return 0;
}

/*
 *  rdata    - isochronous out data buffer
 *  data_len - length of data received
 *  return value - the length of data to be send, must be <= data_len
 */
int iso_out_callback(uint8_t *rdata, int data_len)
{
    /* add code here to send data to device */
    /* ... */
    memset(rdata, (s_i8IsoOutCnt & 0xff), (uint32_t)data_len);
    s_i8IsoOutCnt++;
    return data_len;
}

void demo_isochronous_xfer(void)
{
    uint32_t loop, msg_tick;

    printf("\nPress 'x' to stop loop...\n\n");
    msg_tick = get_ticks();

    s_i8IsoInCnt = 0;
    s_i8IsoOutCnt = 0;
    s_i8HasError = 0;

    lbk_isochronous_in_start(iso_in_callback);
    lbk_isochronous_out_start(iso_out_callback);

    for(loop = 1; ; loop++)
    {
        if(!kbhit())
        {
            if(getchar() == 'x')
            {
                lbk_isochronous_in_stop();
                lbk_isochronous_out_stop();
                return;
            }
        }

        if(!lbk_device_is_connected())
            return;

        if(get_ticks() - msg_tick >= 100)
        {
            printf("Isochronous transfer loop RX: %d, TX: %d    \r", s_i8IsoInCnt, s_i8IsoOutCnt);
            msg_tick = get_ticks();
        }
    }
}

void vendor_lbk_demo(void)
{
    int item;

    usbh_pooling_hubs();
    if(!lbk_device_is_connected())
        return;

    while(1)
    {
        printf("\n");
        printf("+------------------------------------------+\n");
        printf("|  USB Host transfer demo                  |\n");
        printf("+------------------------------------------+\n");
        printf("| [1] Control transfer demo                |\n");
        printf("| [2] Bulk transfer demo                   |\n");
        printf("| [3] Interrupt transfer demo              |\n");
        printf("| [4] Isochronous transfer demo            |\n");
        printf("+------------------------------------------+\n");

        usbh_memory_used();
        printf("\nSelect [1~9;A~S]: \n");

        item = getchar();

        usbh_pooling_hubs();
        if(!lbk_device_is_connected())
            return;

        switch(item)
        {
            case '1':
                demo_ctrl_xfer();
                break;

            case '2':
                demo_bulk_xfer();
                break;

            case '3':
                demo_interrupt_xfer();
                break;

            case '4':
                demo_isochronous_xfer();
                break;
        }

        usbh_pooling_hubs();
        if(!lbk_device_is_connected())
            return;
    }
}

int main(void)
{
    SYS_Init();                        /* Init System, IP clock and multi-function I/O */

    UART0_Init();                      /* Initialize UART0 */

    enable_sys_tick(100);

    printf("\n\n");
    printf("+---------------------------------------+\n");
    printf("|                                       |\n");
    printf("|     USB Host Vendor Loopback Demo     |\n");
    printf("|                                       |\n");
    printf("+---------------------------------------+\n");

    usbh_core_init();
    usbh_lbk_init();
    usbh_install_conn_callback(connect_func, disconnect_func);
    usbh_pooling_hubs();

    if(!lbk_device_is_connected())
        printf("Waiting for Vendor Loopback device to be connected...\n");

    while(1)
    {
        usbh_pooling_hubs();

        if(!lbk_device_is_connected())
            continue;

        /* do not return unless LBK device disconnected */
        vendor_lbk_demo();

        printf("\n\nWaiting for Vendor Loopback device to be connected...\n");
        usbh_memory_used();
    }
}
