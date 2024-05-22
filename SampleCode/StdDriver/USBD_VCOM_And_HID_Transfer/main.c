/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to implement a composite device (VCOM and HID Transfer).
 *           Transfer data between USB device and PC through USB HID interface.
 *           A windows tool is also included in this sample code to connect with a USB device.
 *
 *           Windows tool: User need to input the specific PID for the USB HID device connected to PC.
 *                         PID format with hexadecimal.
 *
 *           -> PID is 0xDC00 in this sample.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "vcom_and_hid_transfer.h"

#define CRYSTAL_LESS        1
#define TRIM_INIT           (SYS_BASE+0x10C)

/*--------------------------------------------------------------------------*/
STR_VCOM_LINE_CODING g_LineCoding = {115200, 0, 0, 8};   /* Baud rate : 115200    */
/* Stop bit     */
/* parity       */
/* data bits    */
uint16_t g_u16CtrlSignal = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */

/*--------------------------------------------------------------------------*/
#define RXBUFSIZE           512 /* RX buffer size */
#define TXBUFSIZE           512 /* RX buffer size */

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
/* UART0 */
static volatile uint8_t s_au8ComRbuf[RXBUFSIZE];
volatile uint16_t g_u16ComRbytes = 0;
volatile uint16_t g_u16ComRhead = 0;
volatile uint16_t g_u16ComRtail = 0;

static volatile uint8_t s_au8ComTbuf[TXBUFSIZE];
volatile uint16_t g_u16ComTbytes = 0;
volatile uint16_t g_u16ComThead = 0;
volatile uint16_t g_u16ComTtail = 0;

static uint8_t s_au8RxBuf[64] = {0};
uint8_t *g_pu8RxBuf = 0;
uint32_t g_u32RxSize = 0;
uint32_t g_u32TxSize = 0;

volatile int8_t g_i8BulkOutReady = 0;

void SYS_Init(void);
void UART0_Init(void);
void UART0_IRQHandler(void);
void PowerDown(void);
/*--------------------------------------------------------------------------*/

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Select HCLK clock source as HIRC and HCLK clock divider as 1 */
    CLK_SetHCLK(CLK_CLKSEL0_HCLKSEL_HIRC, CLK_CLKDIV0_HCLK(1));

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Enable all GPIO clock */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPACKEN_Msk | CLK_AHBCLK0_GPBCKEN_Msk | CLK_AHBCLK0_GPCCKEN_Msk | CLK_AHBCLK0_GPDCKEN_Msk |
                    CLK_AHBCLK0_GPECKEN_Msk | CLK_AHBCLK0_GPFCKEN_Msk | CLK_AHBCLK0_GPGCKEN_Msk | CLK_AHBCLK0_GPHCKEN_Msk;
    CLK->AHBCLK1 |= CLK_AHBCLK1_GPICKEN_Msk | CLK_AHBCLK1_GPJCKEN_Msk;

#if (!CRYSTAL_LESS)
    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set core clock to 192MHz */
    CLK_SetCoreClock(FREQ_192MHZ);

    /* Select USB clock source as PLL/2 and USB clock divider as 2 */
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL0_USBSEL_PLL_DIV2, CLK_CLKDIV0_USB(2));
#else
    /* Enable HIRC48M clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRC48MEN_Msk);

    /* Waiting for HIRC48M clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRC48MSTB_Msk);

    /* Set core clock to 192MHz */
    CLK_SetCoreClock(FREQ_192MHZ);

    /* Select USB clock source as HIRC48M and USB clock divider as 1 */
    CLK_SetModuleClock(USBD_MODULE, CLK_CLKSEL0_USBSEL_HIRC48M, CLK_CLKDIV0_USB(1));
#endif

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Select USBD */
    SYS->USBPHY = (SYS->USBPHY & ~SYS_USBPHY_USBROLE_Msk) | SYS_USBPHY_USBEN_Msk | SYS_USBPHY_SBO_Msk;

    /* Enable USBD module clock */
    CLK_EnableModuleClock(USBD_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* USBD multi-function pins for VBUS, D+, D-, and ID pins */
    SET_USB_VBUS_PA12();
    SET_USB_D_N_PA13();
    SET_USB_D_P_PA14();
    SET_USB_OTG_ID_PA15();
}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART */
    SYS_ResetModule(UART0_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(UART0, 115200);

    /* Enable Interrupt and install the call back function */
    UART_ENABLE_INT(UART0, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));
}

/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void UART0_IRQHandler(void)
{
    uint8_t u8InChar;
    int32_t i32Size;
    uint32_t u32IntStatus;

    u32IntStatus = UART0->INTSTS;

    if((u32IntStatus & UART_INTSTS_RDAIF_Msk) || (u32IntStatus & UART_INTSTS_RXTOIF_Msk))
    {
        /* Receiver FIFO threshold level is reached or Rx time out */

        /* Get all the input characters */
        while(!(UART0->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk))
        {
            /* Get the character from UART Buffer */
            u8InChar = (uint8_t)UART0->DAT;

            /* Check if buffer full */
            if(g_u16ComRbytes < RXBUFSIZE)
            {
                /* Enqueue the character */
                s_au8ComRbuf[g_u16ComRtail++] = u8InChar;
                if(g_u16ComRtail >= RXBUFSIZE)
                    g_u16ComRtail = 0;
                g_u16ComRbytes++;
            }
            else
            {
                /* FIFO over run */
            }
        }
    }

    if(u32IntStatus & UART_INTSTS_THREIF_Msk)
    {

        if(g_u16ComTbytes && (UART0->INTEN & UART_INTEN_THREIEN_Msk))
        {
            /* Fill the Tx FIFO */
            i32Size = g_u16ComTbytes;
            if(i32Size >= UART0_FIFO_SIZE)
            {
                i32Size = UART0_FIFO_SIZE;
            }

            while(i32Size)
            {
                u8InChar = s_au8ComTbuf[g_u16ComThead++];
                UART0->DAT = u8InChar;
                if(g_u16ComThead >= TXBUFSIZE)
                    g_u16ComThead = 0;
                g_u16ComTbytes--;
                i32Size--;
            }
        }
        else
        {
            /* No more data, just stop Tx (Stop work) */
            UART0->INTEN &= ~UART_INTEN_THREIEN_Msk;
        }
    }
}

void VCOM_TransferData(void)
{
    uint32_t i, u32Len;

    /* Check whether USB is ready for next packet or not */
    if(g_u32TxSize == 0)
    {
        /* Check whether we have new COM Rx data to send to USB or not */
        if(g_u16ComRbytes)
        {
            u32Len = g_u16ComRbytes;
            if(u32Len > EP2_MAX_PKT_SIZE)
                u32Len = EP2_MAX_PKT_SIZE;

            for(i = 0; i < u32Len; i++)
            {
                s_au8RxBuf[i] = s_au8ComRbuf[g_u16ComRhead++];
                if(g_u16ComRhead >= RXBUFSIZE)
                    g_u16ComRhead = 0;
            }

            __set_PRIMASK(1);
            g_u16ComRbytes -= u32Len;
            __set_PRIMASK(0);

            g_u32TxSize = u32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), (uint8_t *)s_au8RxBuf, u32Len);
            USBD_SET_PAYLOAD_LEN(EP2, u32Len);
        }
        else
        {
            /* Prepare a zero packet if previous packet size is EP2_MAX_PKT_SIZE and
               no more data to send at this moment to note Host the transfer has been done */
            u32Len = USBD_GET_PAYLOAD_LEN(EP2);
            if(u32Len == EP2_MAX_PKT_SIZE)
                USBD_SET_PAYLOAD_LEN(EP2, 0);
        }
    }

    /* Process the Bulk out data when bulk out data is ready. */
    if(g_i8BulkOutReady && (g_u32RxSize <= TXBUFSIZE - g_u16ComTbytes))
    {
        for(i = 0; i < g_u32RxSize; i++)
        {
            s_au8ComTbuf[g_u16ComTtail++] = g_pu8RxBuf[i];
            if(g_u16ComTtail >= TXBUFSIZE)
                g_u16ComTtail = 0;
        }

        __set_PRIMASK(1);
        g_u16ComTbytes += g_u32RxSize;
        __set_PRIMASK(0);

        g_u32RxSize = 0;
        g_i8BulkOutReady = 0; /* Clear bulk out ready flag */

        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    }

    /* Process the software Tx FIFO */
    if(g_u16ComTbytes)
    {
        /* Check if Tx is working */
        if((UART0->INTEN & UART_INTEN_THREIEN_Msk) == 0)
        {
            /* Send one bytes out */
            UART0->DAT = s_au8ComTbuf[g_u16ComThead++];
            if(g_u16ComThead >= TXBUFSIZE)
                g_u16ComThead = 0;

            g_u16ComTbytes--;

            /* Enable Tx Empty Interrupt. (Trigger first one) */
            UART0->INTEN |= UART_INTEN_THREIEN_Msk;
        }
    }
}

void PowerDown(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Wakeup Enable */
    USBD_ENABLE_INT(USBD_INTEN_WKEN_Msk);

    CLK_PowerDown();

    /* Clear PWR_DOWN_EN if it is not clear by itself */
    if(CLK->PWRCTL & CLK_PWRCTL_PDEN_Msk)
        CLK->PWRCTL ^= CLK_PWRCTL_PDEN_Msk;

    /* Lock protected registers */
    SYS_LockReg();
}

/*---------------------------------------------------------------------------------------------------------*/
/*  Main Function                                                                                          */
/*---------------------------------------------------------------------------------------------------------*/
int main(void)
{
#if CRYSTAL_LESS
    uint32_t u32TrimInit;
#endif

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, peripheral clock and multi-function I/O */
    SYS_Init();

    /* Init UART */
    UART0_Init();

    printf("\n");
    printf("+--------------------------------------------------------------+\n");
    printf("|     NuMicro USB Virtual COM and HID Transfer Sample Code     |\n");
    printf("+--------------------------------------------------------------+\n");

    USBD_Open(&gsInfo, HID_ClassRequest, NULL);

    /* Endpoint configuration */
    HID_Init();
    USBD_Start();

    NVIC_EnableIRQ(UART0_IRQn);

    NVIC_EnableIRQ(USBD_IRQn);

#if CRYSTAL_LESS
    /* Backup default trim */
    u32TrimInit = M32(TRIM_INIT);
#endif

    /* Clear SOF */
    USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

    while(1)
    {
#if CRYSTAL_LESS
        /* Start USB trim if it is not enabled. */
        if((SYS->HIRCTCTL & SYS_HIRCTCTL_FREQSEL_Msk) != 1)
        {
            /* Start USB trim only when SOF */
            if(USBD->INTSTS & USBD_INTSTS_SOFIF_Msk)
            {
                /* Clear SOF */
                USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;

                /* Re-enable crystal-less */
                SYS->HIRCTCTL = 0x01;
                SYS->HIRCTCTL |= SYS_HIRCTCTL_REFCKSEL_Msk | SYS_HIRCTCTL_BOUNDEN_Msk | (8 << SYS_HIRCTCTL_BOUNDARY_Pos);
            }
        }

        /* Disable USB Trim when error */
        if(SYS->HIRCTISTS & (SYS_HIRCTISTS_CLKERRIF_Msk | SYS_HIRCTISTS_TFAILIF_Msk))
        {
            /* Init TRIM */
            M32(TRIM_INIT) = u32TrimInit;

            /* Disable crystal-less */
            SYS->HIRCTCTL = 0;

            /* Clear error flags */
            SYS->HIRCTISTS = SYS_HIRCTISTS_CLKERRIF_Msk | SYS_HIRCTISTS_TFAILIF_Msk;

            /* Clear SOF */
            USBD->INTSTS = USBD_INTSTS_SOFIF_Msk;
        }
#endif

        /* Enter power down when USB suspend */
        if(g_u8Suspend)
            PowerDown();

        VCOM_TransferData();
    }
}
