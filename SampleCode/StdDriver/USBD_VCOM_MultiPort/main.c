/******************************************************************************
 * @file     main.c
 * @version  V3.00
 * @brief    Demonstrate how to implement a USB multi virtual COM port device.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "vcom_serial.h"

#define CRYSTAL_LESS        1
#define TRIM_INIT           (SYS_BASE+0x10C)

/*--------------------------------------------------------------------------*/
STR_VCOM_LINE_CODING g_LineCoding0 = {115200, 0, 0, 8};   /* Baud rate : 115200    */
STR_VCOM_LINE_CODING g_LineCoding1 = {115200, 0, 0, 8};   /* Baud rate : 115200    */
STR_VCOM_LINE_CODING g_LineCoding2 = {115200, 0, 0, 8};   /* Baud rate : 115200    */
STR_VCOM_LINE_CODING g_LineCoding3 = {115200, 0, 0, 8};   /* Baud rate : 115200    */
STR_VCOM_LINE_CODING g_LineCoding4 = {115200, 0, 0, 8};   /* Baud rate : 115200    */
STR_VCOM_LINE_CODING g_LineCoding5 = {115200, 0, 0, 8};   /* Baud rate : 115200    */
STR_VCOM_LINE_CODING g_LineCoding6 = {115200, 0, 0, 8};   /* Baud rate : 115200    */
/* Stop bit     */
/* parity       */
/* data bits    */
uint16_t g_u16CtrlSignal0 = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */
uint16_t g_u16CtrlSignal1 = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */

uint16_t g_u16CtrlSignal2 = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */
uint16_t g_u16CtrlSignal3 = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */

uint16_t g_u16CtrlSignal4 = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */
uint16_t g_u16CtrlSignal5 = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */

uint16_t g_u16CtrlSignal6 = 0;     /* BIT0: DTR(Data Terminal Ready) , BIT1: RTS(Request To Send) */

/*--------------------------------------------------------------------------*/
#define RXBUFSIZE           512 /* RX buffer size */
#define TXBUFSIZE           512 /* RX buffer size */

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#if (VCOM_CNT >= 1)
/* UART0 */
static volatile uint8_t s_au8ComRbuf0[RXBUFSIZE];
volatile uint16_t g_u16ComRbytes0 = 0;
volatile uint16_t g_u16ComRhead0 = 0;
volatile uint16_t g_u16ComRtail0 = 0;

static volatile uint8_t s_au8ComTbuf0[TXBUFSIZE];
volatile uint16_t g_u16ComTbytes0 = 0;
volatile uint16_t g_u16ComThead0 = 0;
volatile uint16_t g_u16ComTtail0 = 0;

static uint8_t s_au8RxBuf0[64] = {0};
uint8_t *g_pu8RxBuf0 = 0;
uint32_t g_u32RxSize0 = 0;
uint32_t g_u32TxSize0 = 0;

volatile int8_t g_i8BulkOutReady0 = 0;
#endif

#if (VCOM_CNT >= 2)
/* UART1 */
static volatile uint8_t s_au8ComRbuf1[RXBUFSIZE];
volatile uint16_t g_u16ComRbytes1 = 0;
volatile uint16_t g_u16ComRhead1 = 0;
volatile uint16_t g_u16ComRtail1 = 0;

static volatile uint8_t s_au8ComTbuf1[TXBUFSIZE];
volatile uint16_t g_u16ComTbytes1 = 0;
volatile uint16_t g_u16ComThead1 = 0;
volatile uint16_t g_u16ComTtail1 = 0;

static uint8_t s_au8RxBuf1[64] = {0};
uint8_t *g_pu8RxBuf1 = 0;
uint32_t g_u32RxSize1 = 0;
uint32_t g_u32TxSize1 = 0;

volatile int8_t g_i8BulkOutReady1 = 0;
#endif

#if (VCOM_CNT >= 3)
/* UART2 */
static volatile uint8_t s_au8ComRbuf2[RXBUFSIZE];
volatile uint16_t g_u16ComRbytes2 = 0;
volatile uint16_t g_u16ComRhead2 = 0;
volatile uint16_t g_u16ComRtail2 = 0;

static volatile uint8_t s_au8ComTbuf2[TXBUFSIZE];
volatile uint16_t g_u16ComTbytes2 = 0;
volatile uint16_t g_u16ComThead2 = 0;
volatile uint16_t g_u16ComTtail2 = 0;

static uint8_t s_au8RxBuf2[64] = {0};
uint8_t *g_pu8RxBuf2 = 0;
uint32_t g_u32RxSize2 = 0;
uint32_t g_u32TxSize2 = 0;

volatile int8_t g_i8BulkOutReady2 = 0;
#endif

#if (VCOM_CNT >= 4)
/* UART3 */
static volatile uint8_t s_au8ComRbuf3[RXBUFSIZE];
volatile uint16_t g_u16ComRbytes3 = 0;
volatile uint16_t g_u16ComRhead3 = 0;
volatile uint16_t g_u16ComRtail3 = 0;

static volatile uint8_t s_au8ComTbuf3[TXBUFSIZE];
volatile uint16_t g_u16ComTbytes3 = 0;
volatile uint16_t g_u16ComThead3 = 0;
volatile uint16_t g_u16ComTtail3 = 0;

static uint8_t s_au8RxBuf3[64] = {0};
uint8_t *g_pu8RxBuf3 = 0;
uint32_t g_u32RxSize3 = 0;
uint32_t g_u32TxSize3 = 0;

volatile int8_t g_i8BulkOutReady3 = 0;
#endif

#if (VCOM_CNT >= 5)
/* UART4 */
static volatile uint8_t s_au8ComRbuf4[RXBUFSIZE];
volatile uint16_t g_u16ComRbytes4 = 0;
volatile uint16_t g_u16ComRhead4 = 0;
volatile uint16_t g_u16ComRtail4 = 0;

static volatile uint8_t s_au8ComTbuf4[TXBUFSIZE];
volatile uint16_t g_u16ComTbytes4 = 0;
volatile uint16_t g_u16ComThead4 = 0;
volatile uint16_t g_u16ComTtail4 = 0;

static uint8_t s_au8RxBuf4[64] = {0};
uint8_t *g_pu8RxBuf4 = 0;
uint32_t g_u32RxSize4 = 0;
uint32_t g_u32TxSize4 = 0;

volatile int8_t g_i8BulkOutReady4 = 0;
#endif

#if (VCOM_CNT >= 6)
/* UART5 */
static volatile uint8_t s_au8ComRbuf5[RXBUFSIZE];
volatile uint16_t g_u16ComRbytes5 = 0;
volatile uint16_t g_u16ComRhead5 = 0;
volatile uint16_t g_u16ComRtail5 = 0;

static volatile uint8_t s_au8ComTbuf5[TXBUFSIZE];
volatile uint16_t g_u16ComTbytes5 = 0;
volatile uint16_t g_u16ComThead5 = 0;
volatile uint16_t g_u16ComTtail5 = 0;

static uint8_t s_au8RxBuf5[64] = {0};
uint8_t *g_pu8RxBuf5 = 0;
uint32_t g_u32RxSize5 = 0;
uint32_t g_u32TxSize5 = 0;

volatile int8_t g_i8BulkOutReady5 = 0;
#endif

#if (VCOM_CNT >= 7)
/* UART6 */
static volatile uint8_t s_au8ComRbuf6[RXBUFSIZE];
volatile uint16_t g_u16ComRbytes6 = 0;
volatile uint16_t g_u16ComRhead6 = 0;
volatile uint16_t g_u16ComRtail6 = 0;

static volatile uint8_t s_au8ComTbuf6[TXBUFSIZE];
volatile uint16_t g_u16ComTbytes6 = 0;
volatile uint16_t g_u16ComThead6 = 0;
volatile uint16_t g_u16ComTtail6 = 0;

static uint8_t s_au8RxBuf6[64] = {0};
uint8_t *g_pu8RxBuf6 = 0;
uint32_t g_u32RxSize6 = 0;
uint32_t g_u32TxSize6 = 0;

volatile int8_t g_i8BulkOutReady6 = 0;
#endif

void SYS_Init(void);
void UART0_Init(void);
void UART1_Init(void);
void UART2_Init(void);
void UART3_Init(void);
void UART4_Init(void);
void UART5_Init(void);
void UART6_Init(void);
void UART0_IRQHandler(void);
void UART1_IRQHandler(void);
void UART2_IRQHandler(void);
void UART3_IRQHandler(void);
void UART4_IRQHandler(void);
void UART5_IRQHandler(void);
void UART6_IRQHandler(void);
void PowerDown(void);
/*---------------------------------------------------------------------------------------------------------*/

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

    /* Enable UART module clock and select UART0 module clock source */
#if (VCOM_CNT >= 1)
    CLK_EnableModuleClock(UART0_MODULE);
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));
#endif

#if (VCOM_CNT >= 2)
    CLK_EnableModuleClock(UART1_MODULE);
    CLK_SetModuleClock(UART1_MODULE, CLK_CLKSEL1_UART1SEL_HIRC, CLK_CLKDIV0_UART1(1));
#endif

#if (VCOM_CNT >= 3)
    CLK_EnableModuleClock(UART2_MODULE);
    CLK_SetModuleClock(UART2_MODULE, CLK_CLKSEL3_UART2SEL_HIRC, CLK_CLKDIV4_UART2(1));
#endif

#if (VCOM_CNT >= 4)
    CLK_EnableModuleClock(UART3_MODULE);
    CLK_SetModuleClock(UART3_MODULE, CLK_CLKSEL3_UART3SEL_HIRC, CLK_CLKDIV4_UART3(1));
#endif

#if (VCOM_CNT >= 5)
    CLK_EnableModuleClock(UART4_MODULE);
    CLK_SetModuleClock(UART4_MODULE, CLK_CLKSEL3_UART4SEL_HIRC, CLK_CLKDIV4_UART4(1));
#endif

#if (VCOM_CNT >= 6)
    CLK_EnableModuleClock(UART5_MODULE);
    CLK_SetModuleClock(UART5_MODULE, CLK_CLKSEL3_UART5SEL_HIRC, CLK_CLKDIV4_UART5(1));
#endif

#if (VCOM_CNT >= 7)
    CLK_EnableModuleClock(UART6_MODULE);
    CLK_SetModuleClock(UART6_MODULE, CLK_CLKSEL3_UART6SEL_HIRC, CLK_CLKDIV4_UART6(1));
#endif

    /* Select USBD */
    SYS->USBPHY = (SYS->USBPHY & ~SYS_USBPHY_USBROLE_Msk) | SYS_USBPHY_USBEN_Msk | SYS_USBPHY_SBO_Msk;

    /* Enable USBD module clock */
    CLK_EnableModuleClock(USBD_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART RXD and TXD */
#if (VCOM_CNT >= 1)
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();
#endif

#if (VCOM_CNT >= 2)
    SET_UART1_RXD_PA2();
    SET_UART1_TXD_PA3();
#endif

#if (VCOM_CNT >= 3)
    SET_UART2_RXD_PC4();
    SET_UART2_TXD_PC5();
#endif

#if (VCOM_CNT >= 4)
    SET_UART3_RXD_PE0();
    SET_UART3_TXD_PE1();
#endif

#if (VCOM_CNT >= 5)
    SET_UART4_RXD_PB10();
    SET_UART4_TXD_PB11();
#endif

#if (VCOM_CNT >= 6)
    SET_UART5_RXD_PE6();
    SET_UART5_TXD_PE7();
#endif

#if (VCOM_CNT >= 7)
    SET_UART6_RXD_PA10();
    SET_UART6_TXD_PA11();
#endif

    /* USBD multi-function pins for VBUS, D+, D-, and ID pins */
    SET_USB_VBUS_PA12();
    SET_USB_D_N_PA13();
    SET_USB_D_P_PA14();
    SET_USB_OTG_ID_PA15();
}

#if (VCOM_CNT >= 1)
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
#endif

#if (VCOM_CNT >= 2)
void UART1_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART */
    SYS_ResetModule(UART1_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(UART1, 115200);

    /* Enable Interrupt and install the call back function */
    UART_ENABLE_INT(UART1, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));
}
#endif

#if (VCOM_CNT >= 3)
void UART2_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART */
    SYS_ResetModule(UART2_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(UART2, 115200);

    /* Enable Interrupt and install the call back function */
    UART_ENABLE_INT(UART2, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));
}
#endif

#if (VCOM_CNT >= 4)
void UART3_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART */
    SYS_ResetModule(UART3_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(UART3, 115200);

    /* Enable Interrupt and install the call back function */
    UART_ENABLE_INT(UART3, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));
}
#endif

#if (VCOM_CNT >= 5)
void UART4_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART */
    SYS_ResetModule(UART4_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(UART4, 115200);

    /* Enable Interrupt and install the call back function */
    UART_ENABLE_INT(UART4, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));
}
#endif

#if (VCOM_CNT >= 6)
void UART5_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART */
    SYS_ResetModule(UART5_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(UART5, 115200);

    /* Enable Interrupt and install the call back function */
    UART_ENABLE_INT(UART5, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));
}
#endif

#if (VCOM_CNT >= 7)
void UART6_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    /* Reset UART */
    SYS_ResetModule(UART6_RST);

    /* Configure UART and set UART Baudrate */
    UART_Open(UART6, 115200);

    /* Enable Interrupt and install the call back function */
    UART_ENABLE_INT(UART6, (UART_INTEN_RDAIEN_Msk | UART_INTEN_THREIEN_Msk | UART_INTEN_RXTOIEN_Msk));
}
#endif

/*---------------------------------------------------------------------------------------------------------*/
/* UART Callback function                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
#if (VCOM_CNT >= 1)
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
            if(g_u16ComRbytes0 < RXBUFSIZE)
            {
                /* Enqueue the character */
                s_au8ComRbuf0[g_u16ComRtail0++] = u8InChar;
                if(g_u16ComRtail0 >= RXBUFSIZE)
                {
                    g_u16ComRtail0 = 0;
                }
                g_u16ComRbytes0++;
            }
            else
            {
                /* FIFO over run */
            }
        }
    }

    if(u32IntStatus & UART_INTSTS_THREIF_Msk)
    {

        if(g_u16ComTbytes0 && (UART0->INTEN & UART_INTEN_THREIEN_Msk))
        {
            /* Fill the Tx FIFO */
            i32Size = g_u16ComTbytes0;
            if(i32Size >= UART0_FIFO_SIZE)
            {
                i32Size = UART0_FIFO_SIZE;
            }

            while(i32Size)
            {
                u8InChar = s_au8ComTbuf0[g_u16ComThead0++];
                UART0->DAT = u8InChar;
                if(g_u16ComThead0 >= TXBUFSIZE)
                {
                    g_u16ComThead0 = 0;
                }
                g_u16ComTbytes0--;
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
#endif

#if (VCOM_CNT >= 2)
void UART1_IRQHandler(void)
{
    uint8_t u8InChar;
    int32_t i32Size;
    uint32_t u32IntStatus;

    u32IntStatus = UART1->INTSTS;

    if((u32IntStatus & UART_INTSTS_RDAIF_Msk) || (u32IntStatus & UART_INTSTS_RXTOIF_Msk))
    {
        /* Receiver FIFO threshold level is reached or Rx time out */

        /* Get all the input characters */
        while(!(UART1->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk))
        {
            /* Get the character from UART Buffer */
            u8InChar = (uint8_t)UART1->DAT;

            /* Check if buffer full */
            if(g_u16ComRbytes1 < RXBUFSIZE)
            {
                /* Enqueue the character */
                s_au8ComRbuf1[g_u16ComRtail1++] = u8InChar;
                if(g_u16ComRtail1 >= RXBUFSIZE)
                {
                    g_u16ComRtail1 = 0;
                }
                g_u16ComRbytes1++;
            }
            else
            {
                /* FIFO over run */
            }
        }
    }

    if(u32IntStatus & UART_INTSTS_THREIF_Msk)
    {

        if(g_u16ComTbytes1 && (UART1->INTEN & UART_INTEN_THREIEN_Msk))
        {
            /* Fill the Tx FIFO */
            i32Size = g_u16ComTbytes1;
            if(i32Size >= UART1_FIFO_SIZE)
            {
                i32Size = UART1_FIFO_SIZE;
            }

            while(i32Size)
            {
                u8InChar = s_au8ComTbuf1[g_u16ComThead1++];
                UART1->DAT = u8InChar;
                if(g_u16ComThead1 >= TXBUFSIZE)
                    g_u16ComThead1 = 0;
                g_u16ComTbytes1--;
                i32Size--;
            }
        }
        else
        {
            /* No more data, just stop Tx (Stop work) */
            UART1->INTEN &= ~UART_INTEN_THREIEN_Msk;
        }
    }
}
#endif

#if (VCOM_CNT >= 3)
void UART2_IRQHandler(void)
{
    uint8_t u8InChar;
    int32_t i32Size;
    uint32_t u32IntStatus;

    u32IntStatus = UART2->INTSTS;

    if((u32IntStatus & UART_INTSTS_RDAIF_Msk) || (u32IntStatus & UART_INTSTS_RXTOIF_Msk))
    {
        /* Receiver FIFO threshold level is reached or Rx time out */

        /* Get all the input characters */
        while(!(UART2->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk))
        {
            /* Get the character from UART Buffer */
            u8InChar = (uint8_t)UART2->DAT;

            /* Check if buffer full */
            if(g_u16ComRbytes2 < RXBUFSIZE)
            {
                /* Enqueue the character */
                s_au8ComRbuf2[g_u16ComRtail2++] = u8InChar;
                if(g_u16ComRtail2 >= RXBUFSIZE)
                {
                    g_u16ComRtail2 = 0;
                }
                g_u16ComRbytes2++;
            }
            else
            {
                /* FIFO over run */
            }
        }
    }

    if(u32IntStatus & UART_INTSTS_THREIF_Msk)
    {

        if(g_u16ComTbytes2 && (UART2->INTEN & UART_INTEN_THREIEN_Msk))
        {
            /* Fill the Tx FIFO */
            i32Size = g_u16ComTbytes2;
            if(i32Size >= UART2_FIFO_SIZE)
            {
                i32Size = UART2_FIFO_SIZE;
            }

            while(i32Size)
            {
                u8InChar = s_au8ComTbuf2[g_u16ComThead2++];
                UART2->DAT = u8InChar;
                if(g_u16ComThead2 >= TXBUFSIZE)
                {
                    g_u16ComThead2 = 0;
                }
                g_u16ComTbytes2--;
                i32Size--;
            }
        }
        else
        {
            /* No more data, just stop Tx (Stop work) */
            UART2->INTEN &= ~UART_INTEN_THREIEN_Msk;
        }
    }
}
#endif

#if (VCOM_CNT >= 4)
void UART3_IRQHandler(void)
{
    uint8_t u8InChar;
    int32_t i32Size;
    uint32_t u32IntStatus;

    u32IntStatus = UART3->INTSTS;

    if((u32IntStatus & UART_INTSTS_RDAIF_Msk) || (u32IntStatus & UART_INTSTS_RXTOIF_Msk))
    {
        /* Receiver FIFO threshold level is reached or Rx time out */

        /* Get all the input characters */
        while(!(UART3->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk))
        {
            /* Get the character from UART Buffer */
            u8InChar = (uint8_t)UART3->DAT;

            /* Check if buffer full */
            if(g_u16ComRbytes3 < RXBUFSIZE)
            {
                /* Enqueue the character */
                s_au8ComRbuf3[g_u16ComRtail3++] = u8InChar;
                if(g_u16ComRtail3 >= RXBUFSIZE)
                {
                    g_u16ComRtail3 = 0;
                }
                g_u16ComRbytes3++;
            }
            else
            {
                /* FIFO over run */
            }
        }
    }

    if(u32IntStatus & UART_INTSTS_THREIF_Msk)
    {

        if(g_u16ComTbytes3 && (UART3->INTEN & UART_INTEN_THREIEN_Msk))
        {
            /* Fill the Tx FIFO */
            i32Size = g_u16ComTbytes3;
            if(i32Size >= UART3_FIFO_SIZE)
            {
                i32Size = UART3_FIFO_SIZE;
            }

            while(i32Size)
            {
                u8InChar = s_au8ComTbuf3[g_u16ComThead3++];
                UART3->DAT = u8InChar;
                if(g_u16ComThead3 >= TXBUFSIZE)
                {
                    g_u16ComThead3 = 0;
                }
                g_u16ComTbytes3--;
                i32Size--;
            }
        }
        else
        {
            /* No more data, just stop Tx (Stop work) */
            UART3->INTEN &= ~UART_INTEN_THREIEN_Msk;
        }
    }
}
#endif

#if (VCOM_CNT >= 5)
void UART4_IRQHandler(void)
{
    uint8_t u8InChar;
    int32_t i32Size;
    uint32_t u32IntStatus;

    u32IntStatus = UART4->INTSTS;

    if((u32IntStatus & UART_INTSTS_RDAIF_Msk) || (u32IntStatus & UART_INTSTS_RXTOIF_Msk))
    {
        /* Receiver FIFO threshold level is reached or Rx time out */

        /* Get all the input characters */
        while(!(UART4->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk))
        {
            /* Get the character from UART Buffer */
            u8InChar = (uint8_t)UART4->DAT;

            /* Check if buffer full */
            if(g_u16ComRbytes4 < RXBUFSIZE)
            {
                /* Enqueue the character */
                s_au8ComRbuf4[g_u16ComRtail4++] = u8InChar;
                if(g_u16ComRtail4 >= RXBUFSIZE)
                {
                    g_u16ComRtail4 = 0;
                }
                g_u16ComRbytes4++;
            }
            else
            {
                /* FIFO over run */
            }
        }
    }

    if(u32IntStatus & UART_INTSTS_THREIF_Msk)
    {

        if(g_u16ComTbytes4 && (UART4->INTEN & UART_INTEN_THREIEN_Msk))
        {
            /* Fill the Tx FIFO */
            i32Size = g_u16ComTbytes4;
            if(i32Size >= UART4_FIFO_SIZE)
            {
                i32Size = UART4_FIFO_SIZE;
            }

            while(i32Size)
            {
                u8InChar = s_au8ComTbuf4[g_u16ComThead4++];
                UART4->DAT = u8InChar;
                if(g_u16ComThead4 >= TXBUFSIZE)
                {
                    g_u16ComThead4 = 0;
                }
                g_u16ComTbytes4--;
                i32Size--;
            }
        }
        else
        {
            /* No more data, just stop Tx (Stop work) */
            UART4->INTEN &= ~UART_INTEN_THREIEN_Msk;
        }
    }
}
#endif

#if (VCOM_CNT >= 6)
void UART5_IRQHandler(void)
{
    uint8_t u8InChar;
    int32_t i32Size;
    uint32_t u32IntStatus;

    u32IntStatus = UART5->INTSTS;

    if((u32IntStatus & UART_INTSTS_RDAIF_Msk) || (u32IntStatus & UART_INTSTS_RXTOIF_Msk))
    {
        /* Receiver FIFO threshold level is reached or Rx time out */

        /* Get all the input characters */
        while(!(UART5->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk))
        {
            /* Get the character from UART Buffer */
            u8InChar = (uint8_t)UART5->DAT;

            /* Check if buffer full */
            if(g_u16ComRbytes5 < RXBUFSIZE)
            {
                /* Enqueue the character */
                s_au8ComRbuf5[g_u16ComRtail5++] = u8InChar;
                if(g_u16ComRtail5 >= RXBUFSIZE)
                {
                    g_u16ComRtail5 = 0;
                }
                g_u16ComRbytes5++;
            }
            else
            {
                /* FIFO over run */
            }
        }
    }

    if(u32IntStatus & UART_INTSTS_THREIF_Msk)
    {

        if(g_u16ComTbytes5 && (UART5->INTEN & UART_INTEN_THREIEN_Msk))
        {
            /* Fill the Tx FIFO */
            i32Size = g_u16ComTbytes5;
            if(i32Size >= UART5_FIFO_SIZE)
            {
                i32Size = UART5_FIFO_SIZE;
            }

            while(i32Size)
            {
                u8InChar = s_au8ComTbuf5[g_u16ComThead5++];
                UART5->DAT = u8InChar;
                if(g_u16ComThead5 >= TXBUFSIZE)
                {
                    g_u16ComThead5 = 0;
                }
                g_u16ComTbytes5--;
                i32Size--;
            }
        }
        else
        {
            /* No more data, just stop Tx (Stop work) */
            UART5->INTEN &= ~UART_INTEN_THREIEN_Msk;
        }
    }
}
#endif

#if (VCOM_CNT >= 7)
void UART6_IRQHandler(void)
{
    uint8_t u8InChar;
    int32_t i32Size;
    uint32_t u32IntStatus;

    u32IntStatus = UART6->INTSTS;

    if((u32IntStatus & UART_INTSTS_RDAIF_Msk) || (u32IntStatus & UART_INTSTS_RXTOIF_Msk))
    {
        /* Receiver FIFO threshold level is reached or Rx time out */

        /* Get all the input characters */
        while(!(UART6->FIFOSTS & UART_FIFOSTS_RXEMPTY_Msk))
        {
            /* Get the character from UART Buffer */
            u8InChar = (uint8_t)UART6->DAT;

            /* Check if buffer full */
            if(g_u16ComRbytes6 < RXBUFSIZE)
            {
                /* Enqueue the character */
                s_au8ComRbuf6[g_u16ComRtail6++] = u8InChar;
                if(g_u16ComRtail6 >= RXBUFSIZE)
                {
                    g_u16ComRtail6 = 0;
                }
                g_u16ComRbytes6++;
            }
            else
            {
                /* FIFO over run */
            }
        }
    }

    if(u32IntStatus & UART_INTSTS_THREIF_Msk)
    {

        if(g_u16ComTbytes6 && (UART6->INTEN & UART_INTEN_THREIEN_Msk))
        {
            /* Fill the Tx FIFO */
            i32Size = g_u16ComTbytes6;
            if(i32Size >= UART6_FIFO_SIZE)
            {
                i32Size = UART6_FIFO_SIZE;
            }

            while(i32Size)
            {
                u8InChar = s_au8ComTbuf6[g_u16ComThead6++];
                UART6->DAT = u8InChar;
                if(g_u16ComThead6 >= TXBUFSIZE)
                {
                    g_u16ComThead6 = 0;
                }
                g_u16ComTbytes6--;
                i32Size--;
            }
        }
        else
        {
            /* No more data, just stop Tx (Stop work) */
            UART6->INTEN &= ~UART_INTEN_THREIEN_Msk;
        }
    }
}
#endif
void VCOM_TransferData(void)
{
    uint32_t i, u32Len;

    /* Check whether USB is ready for next packet or not */
#if (VCOM_CNT >= 1)
    if(g_u32TxSize0 == 0)
    {
        /* Check whether we have new COM Rx data to send to USB or not */
        if(g_u16ComRbytes0)
        {
            u32Len = g_u16ComRbytes0;
            if(u32Len > EP2_MAX_PKT_SIZE)
            {
                u32Len = EP2_MAX_PKT_SIZE;
            }

            for(i = 0; i < u32Len; i++)
            {
                s_au8RxBuf0[i] = s_au8ComRbuf0[g_u16ComRhead0++];
                if(g_u16ComRhead0 >= RXBUFSIZE)
                {
                    g_u16ComRhead0 = 0;
                }
            }

            __set_PRIMASK(1);
            g_u16ComRbytes0 -= u32Len;
            __set_PRIMASK(0);

            g_u32TxSize0 = u32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP2)), (uint8_t *)s_au8RxBuf0, u32Len);
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
#endif

#if (VCOM_CNT >= 2)
    if(g_u32TxSize1 == 0)
    {
        /* Check whether we have new COM Rx data to send to USB or not */
        if(g_u16ComRbytes1)
        {
            u32Len = g_u16ComRbytes1;
            if(u32Len > EP7_MAX_PKT_SIZE)
            {
                u32Len = EP7_MAX_PKT_SIZE;
            }

            for(i = 0; i < u32Len; i++)
            {
                s_au8RxBuf1[i] = s_au8ComRbuf1[g_u16ComRhead1++];
                if(g_u16ComRhead1 >= RXBUFSIZE)
                {
                    g_u16ComRhead1 = 0;
                }
            }

            __set_PRIMASK(1);
            g_u16ComRbytes1 -= u32Len;
            __set_PRIMASK(0);

            g_u32TxSize1 = u32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP7)), (uint8_t *)s_au8RxBuf1, u32Len);
            USBD_SET_PAYLOAD_LEN(EP7, u32Len);
        }
        else
        {
            /* Prepare a zero packet if previous packet size is EP7_MAX_PKT_SIZE and
               no more data to send at this moment to note Host the transfer has been done */
            u32Len = USBD_GET_PAYLOAD_LEN(EP7);
            if(u32Len == EP7_MAX_PKT_SIZE)
                USBD_SET_PAYLOAD_LEN(EP7, 0);
        }
    }
#endif

#if (VCOM_CNT >= 3)
    if(g_u32TxSize2 == 0)
    {
        /* Check whether we have new COM Rx data to send to USB or not */
        if(g_u16ComRbytes2)
        {
            u32Len = g_u16ComRbytes2;
            if(u32Len > EP10_MAX_PKT_SIZE)
            {
                u32Len = EP10_MAX_PKT_SIZE;
            }

            for(i = 0; i < u32Len; i++)
            {
                s_au8RxBuf2[i] = s_au8ComRbuf2[g_u16ComRhead2++];
                if(g_u16ComRhead2 >= RXBUFSIZE)
                {
                    g_u16ComRhead2 = 0;
                }
            }

            __set_PRIMASK(1);
            g_u16ComRbytes2 -= u32Len;
            __set_PRIMASK(0);

            g_u32TxSize2 = u32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP10)), (uint8_t *)s_au8RxBuf2, u32Len);
            USBD_SET_PAYLOAD_LEN(EP10, u32Len);
        }
        else
        {
            /* Prepare a zero packet if previous packet size is EP7_MAX_PKT_SIZE and
               no more data to send at this moment to note Host the transfer has been done */
            u32Len = USBD_GET_PAYLOAD_LEN(EP10);
            if(u32Len == EP10_MAX_PKT_SIZE)
                USBD_SET_PAYLOAD_LEN(EP10, 0);
        }
    }
#endif

#if (VCOM_CNT >= 4)
    if(g_u32TxSize3 == 0)
    {
        /* Check whether we have new COM Rx data to send to USB or not */
        if(g_u16ComRbytes3)
        {
            u32Len = g_u16ComRbytes3;
            if(u32Len > EP13_MAX_PKT_SIZE)
            {
                u32Len = EP13_MAX_PKT_SIZE;
            }

            for(i = 0; i < u32Len; i++)
            {
                s_au8RxBuf3[i] = s_au8ComRbuf3[g_u16ComRhead3++];
                if(g_u16ComRhead3 >= RXBUFSIZE)
                {
                    g_u16ComRhead3 = 0;
                }
            }

            __set_PRIMASK(1);
            g_u16ComRbytes3 -= u32Len;
            __set_PRIMASK(0);

            g_u32TxSize3 = u32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP13)), (uint8_t *)s_au8RxBuf3, u32Len);
            USBD_SET_PAYLOAD_LEN(EP13, u32Len);
        }
        else
        {
            /* Prepare a zero packet if previous packet size is EP7_MAX_PKT_SIZE and
               no more data to send at this moment to note Host the transfer has been done */
            u32Len = USBD_GET_PAYLOAD_LEN(EP13);
            if(u32Len == EP13_MAX_PKT_SIZE)
                USBD_SET_PAYLOAD_LEN(EP13, 0);
        }
    }
#endif

#if (VCOM_CNT >= 5)
    if(g_u32TxSize4 == 0)
    {
        /* Check whether we have new COM Rx data to send to USB or not */
        if(g_u16ComRbytes4)
        {
            u32Len = g_u16ComRbytes4;
            if(u32Len > EP16_MAX_PKT_SIZE)
            {
                u32Len = EP16_MAX_PKT_SIZE;
            }

            for(i = 0; i < u32Len; i++)
            {
                s_au8RxBuf4[i] = s_au8ComRbuf4[g_u16ComRhead4++];
                if(g_u16ComRhead4 >= RXBUFSIZE)
                {
                    g_u16ComRhead4 = 0;
                }
            }

            __set_PRIMASK(1);
            g_u16ComRbytes4 -= u32Len;
            __set_PRIMASK(0);

            g_u32TxSize4 = u32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP16)), (uint8_t *)s_au8RxBuf4, u32Len);
            USBD_SET_PAYLOAD_LEN(EP16, u32Len);
        }
        else
        {
            /* Prepare a zero packet if previous packet size is EP7_MAX_PKT_SIZE and
               no more data to send at this moment to note Host the transfer has been done */
            u32Len = USBD_GET_PAYLOAD_LEN(EP16);
            if(u32Len == EP16_MAX_PKT_SIZE)
                USBD_SET_PAYLOAD_LEN(EP16, 0);
        }
    }
#endif

#if (VCOM_CNT >= 6)
    if(g_u32TxSize5 == 0)
    {
        /* Check whether we have new COM Rx data to send to USB or not */
        if(g_u16ComRbytes5)
        {
            u32Len = g_u16ComRbytes5;
            if(u32Len > EP19_MAX_PKT_SIZE)
            {
                u32Len = EP19_MAX_PKT_SIZE;
            }

            for(i = 0; i < u32Len; i++)
            {
                s_au8RxBuf5[i] = s_au8ComRbuf5[g_u16ComRhead5++];
                if(g_u16ComRhead5 >= RXBUFSIZE)
                {
                    g_u16ComRhead5 = 0;
                }
            }

            __set_PRIMASK(1);
            g_u16ComRbytes5 -= u32Len;
            __set_PRIMASK(0);

            g_u32TxSize5 = u32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP19)), (uint8_t *)s_au8RxBuf5, u32Len);
            USBD_SET_PAYLOAD_LEN(EP19, u32Len);
        }
        else
        {
            /* Prepare a zero packet if previous packet size is EP7_MAX_PKT_SIZE and
               no more data to send at this moment to note Host the transfer has been done */
            u32Len = USBD_GET_PAYLOAD_LEN(EP19);
            if(u32Len == EP19_MAX_PKT_SIZE)
                USBD_SET_PAYLOAD_LEN(EP19, 0);
        }
    }
#endif

#if (VCOM_CNT >= 7)
    if(g_u32TxSize6 == 0)
    {
        /* Check whether we have new COM Rx data to send to USB or not */
        if(g_u16ComRbytes6)
        {
            u32Len = g_u16ComRbytes6;
            if(u32Len > EP22_MAX_PKT_SIZE)
            {
                u32Len = EP22_MAX_PKT_SIZE;
            }

            for(i = 0; i < u32Len; i++)
            {
                s_au8RxBuf6[i] = s_au8ComRbuf6[g_u16ComRhead6++];
                if(g_u16ComRhead6 >= RXBUFSIZE)
                {
                    g_u16ComRhead6 = 0;
                }
            }

            __set_PRIMASK(1);
            g_u16ComRbytes6 -= u32Len;
            __set_PRIMASK(0);

            g_u32TxSize6 = u32Len;
            USBD_MemCopy((uint8_t *)(USBD_BUF_BASE + USBD_GET_EP_BUF_ADDR(EP22)), (uint8_t *)s_au8RxBuf6, u32Len);
            USBD_SET_PAYLOAD_LEN(EP22, u32Len);
        }
        else
        {
            /* Prepare a zero packet if previous packet size is EP7_MAX_PKT_SIZE and
               no more data to send at this moment to note Host the transfer has been done */
            u32Len = USBD_GET_PAYLOAD_LEN(EP22);
            if(u32Len == EP22_MAX_PKT_SIZE)
                USBD_SET_PAYLOAD_LEN(EP22, 0);
        }
    }
#endif

#if (VCOM_CNT >= 1)
    /* Process the Bulk out data when bulk out data is ready. */
    if(g_i8BulkOutReady0 && (g_u32RxSize0 <= TXBUFSIZE - g_u16ComTbytes0))
    {
        for(i = 0; i < g_u32RxSize0; i++)
        {
            s_au8ComTbuf0[g_u16ComTtail0++] = g_pu8RxBuf0[i];
            if(g_u16ComTtail0 >= TXBUFSIZE)
            {
                g_u16ComTtail0 = 0;
            }
        }

        __set_PRIMASK(1);
        g_u16ComTbytes0 += g_u32RxSize0;
        __set_PRIMASK(0);

        g_u32RxSize0 = 0;
        g_i8BulkOutReady0 = 0; /* Clear bulk out ready flag */

        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP3, EP3_MAX_PKT_SIZE);
    }
#endif

#if (VCOM_CNT >= 2)
    if(g_i8BulkOutReady1 && (g_u32RxSize1 <= TXBUFSIZE - g_u16ComTbytes1))
    {
        for(i = 0; i < g_u32RxSize1; i++)
        {
            s_au8ComTbuf1[g_u16ComTtail1++] = g_pu8RxBuf1[i];
            if(g_u16ComTtail1 >= TXBUFSIZE)
            {
                g_u16ComTtail1 = 0;
            }
        }

        __set_PRIMASK(1);
        g_u16ComTbytes1 += g_u32RxSize1;
        __set_PRIMASK(0);

        g_u32RxSize1 = 0;
        g_i8BulkOutReady1 = 0; /* Clear bulk out ready flag */

        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP6, EP6_MAX_PKT_SIZE);
    }
#endif

#if (VCOM_CNT >= 3)
    if(g_i8BulkOutReady2 && (g_u32RxSize2 <= TXBUFSIZE - g_u16ComTbytes2))
    {
        for(i = 0; i < g_u32RxSize2; i++)
        {
            s_au8ComTbuf2[g_u16ComTtail2++] = g_pu8RxBuf2[i];
            if(g_u16ComTtail2 >= TXBUFSIZE)
            {
                g_u16ComTtail2 = 0;
            }
        }

        __set_PRIMASK(1);
        g_u16ComTbytes2 += g_u32RxSize2;
        __set_PRIMASK(0);

        g_u32RxSize2 = 0;
        g_i8BulkOutReady2 = 0; /* Clear bulk out ready flag */

        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP9, EP9_MAX_PKT_SIZE);
    }
#endif

#if (VCOM_CNT >= 4)

    if(g_i8BulkOutReady3 && (g_u32RxSize3 <= TXBUFSIZE - g_u16ComTbytes3))
    {
        for(i = 0; i < g_u32RxSize3; i++)
        {
            s_au8ComTbuf3[g_u16ComTtail3++] = g_pu8RxBuf3[i];
            if(g_u16ComTtail3 >= TXBUFSIZE)
            {
                g_u16ComTtail3 = 0;
            }
        }

        __set_PRIMASK(1);
        g_u16ComTbytes3 += g_u32RxSize3;
        __set_PRIMASK(0);

        g_u32RxSize3 = 0;
        g_i8BulkOutReady3 = 0; /* Clear bulk out ready flag */

        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP12, EP12_MAX_PKT_SIZE);
    }
#endif

#if (VCOM_CNT >= 5)
    if(g_i8BulkOutReady4 && (g_u32RxSize4 <= TXBUFSIZE - g_u16ComTbytes4))
    {
        for(i = 0; i < g_u32RxSize4; i++)
        {
            s_au8ComTbuf4[g_u16ComTtail4++] = g_pu8RxBuf4[i];
            if(g_u16ComTtail4 >= TXBUFSIZE)
            {
                g_u16ComTtail4 = 0;
            }
        }

        __set_PRIMASK(1);
        g_u16ComTbytes4 += g_u32RxSize4;
        __set_PRIMASK(0);

        g_u32RxSize4 = 0;
        g_i8BulkOutReady4 = 0; /* Clear bulk out ready flag */

        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP15, EP15_MAX_PKT_SIZE);
    }
#endif

#if (VCOM_CNT >= 6)
    if(g_i8BulkOutReady5 && (g_u32RxSize5 <= TXBUFSIZE - g_u16ComTbytes5))
    {
        for(i = 0; i < g_u32RxSize5; i++)
        {
            s_au8ComTbuf5[g_u16ComTtail5++] = g_pu8RxBuf5[i];
            if(g_u16ComTtail5 >= TXBUFSIZE)
            {
                g_u16ComTtail5 = 0;
            }
        }

        __set_PRIMASK(1);
        g_u16ComTbytes5 += g_u32RxSize5;
        __set_PRIMASK(0);

        g_u32RxSize5 = 0;
        g_i8BulkOutReady5 = 0; /* Clear bulk out ready flag */

        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP18, EP18_MAX_PKT_SIZE);
    }

#endif

#if (VCOM_CNT >= 7)
    if(g_i8BulkOutReady6 && (g_u32RxSize6 <= TXBUFSIZE - g_u16ComTbytes6))
    {
        for(i = 0; i < g_u32RxSize6; i++)
        {
            s_au8ComTbuf6[g_u16ComTtail6++] = g_pu8RxBuf6[i];
            if(g_u16ComTtail6 >= TXBUFSIZE)
            {
                g_u16ComTtail6 = 0;
            }
        }

        __set_PRIMASK(1);
        g_u16ComTbytes6 += g_u32RxSize6;
        __set_PRIMASK(0);

        g_u32RxSize6 = 0;
        g_i8BulkOutReady6 = 0; /* Clear bulk out ready flag */

        /* Ready to get next BULK out */
        USBD_SET_PAYLOAD_LEN(EP21, EP21_MAX_PKT_SIZE);
    }

#endif
    /* Process the software Tx FIFO */
#if (VCOM_CNT >= 1)
    if(g_u16ComTbytes0)
    {
        /* Check if Tx is working */
        if((UART0->INTEN & UART_INTEN_THREIEN_Msk) == 0)
        {
            /* Send one bytes out */
            UART0->DAT = s_au8ComTbuf0[g_u16ComThead0++];
            if(g_u16ComThead0 >= TXBUFSIZE)
            {
                g_u16ComThead0 = 0;
            }

            g_u16ComTbytes0--;

            /* Enable Tx Empty Interrupt. (Trigger first one) */
            UART0->INTEN |= UART_INTEN_THREIEN_Msk;
        }
    }
#endif

#if (VCOM_CNT >= 2)
    if(g_u16ComTbytes1)
    {
        /* Check if Tx is working */
        if((UART1->INTEN & UART_INTEN_THREIEN_Msk) == 0)
        {
            /* Send one bytes out */
            UART1->DAT = s_au8ComTbuf1[g_u16ComThead1++];
            if(g_u16ComThead1 >= TXBUFSIZE)
            {
                g_u16ComThead1 = 0;
            }

            g_u16ComTbytes1--;

            /* Enable Tx Empty Interrupt. (Trigger first one) */
            UART1->INTEN |= UART_INTEN_THREIEN_Msk;
        }
    }
#endif

#if (VCOM_CNT >= 3)
    if(g_u16ComTbytes2)
    {
        /* Check if Tx is working */
        if((UART2->INTEN & UART_INTEN_THREIEN_Msk) == 0)
        {
            /* Send one bytes out */
            UART2->DAT = s_au8ComTbuf2[g_u16ComThead2++];
            if(g_u16ComThead2 >= TXBUFSIZE)
            {
                g_u16ComThead2 = 0;
            }

            g_u16ComTbytes2--;

            /* Enable Tx Empty Interrupt. (Trigger first one) */
            UART2->INTEN |= UART_INTEN_THREIEN_Msk;
        }
    }
#endif

#if (VCOM_CNT >= 4)

    if(g_u16ComTbytes3)
    {
        /* Check if Tx is working */
        if((UART3->INTEN & UART_INTEN_THREIEN_Msk) == 0)
        {
            /* Send one bytes out */
            UART3->DAT = s_au8ComTbuf3[g_u16ComThead3++];
            if(g_u16ComThead3 >= TXBUFSIZE)
            {
                g_u16ComThead3 = 0;
            }

            g_u16ComTbytes3--;

            /* Enable Tx Empty Interrupt. (Trigger first one) */
            UART3->INTEN |= UART_INTEN_THREIEN_Msk;
        }
    }
#endif

#if (VCOM_CNT >= 5)
    if(g_u16ComTbytes4)
    {
        /* Check if Tx is working */
        if((UART4->INTEN & UART_INTEN_THREIEN_Msk) == 0)
        {
            /* Send one bytes out */
            UART4->DAT = s_au8ComTbuf4[g_u16ComThead4++];
            if(g_u16ComThead4 >= TXBUFSIZE)
            {
                g_u16ComThead4 = 0;
            }

            g_u16ComTbytes4--;

            /* Enable Tx Empty Interrupt. (Trigger first one) */
            UART4->INTEN |= UART_INTEN_THREIEN_Msk;
        }
    }
#endif

#if (VCOM_CNT >= 6)
    if(g_u16ComTbytes5)
    {
        /* Check if Tx is working */
        if((UART5->INTEN & UART_INTEN_THREIEN_Msk) == 0)
        {
            /* Send one bytes out */
            UART5->DAT = s_au8ComTbuf5[g_u16ComThead5++];
            if(g_u16ComThead5 >= TXBUFSIZE)
            {
                g_u16ComThead5 = 0;
            }

            g_u16ComTbytes5--;

            /* Enable Tx Empty Interrupt. (Trigger first one) */
            UART5->INTEN |= UART_INTEN_THREIEN_Msk;
        }
    }
#endif

#if (VCOM_CNT >= 7)
    if(g_u16ComTbytes6)
    {
        /* Check if Tx is working */
        if((UART6->INTEN & UART_INTEN_THREIEN_Msk) == 0)
        {
            /* Send one bytes out */
            UART6->DAT = s_au8ComTbuf6[g_u16ComThead6++];
            if(g_u16ComThead6 >= TXBUFSIZE)
            {
                g_u16ComThead6 = 0;
            }

            g_u16ComTbytes6--;

            /* Enable Tx Empty Interrupt. (Trigger first one) */
            UART6->INTEN |= UART_INTEN_THREIEN_Msk;
        }
    }
#endif
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
    {
        CLK->PWRCTL ^= CLK_PWRCTL_PDEN_Msk;
    }

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
#if (VCOM_CNT >= 1)
    UART0_Init();
#endif

#if (VCOM_CNT >= 2)
    UART1_Init();
#endif

#if (VCOM_CNT >= 3)
    UART2_Init();
#endif

#if (VCOM_CNT >= 4)
    UART3_Init();
#endif

#if (VCOM_CNT >= 5)
    UART4_Init();
#endif

#if (VCOM_CNT >= 6)
    UART5_Init();
#endif

#if (VCOM_CNT >= 7)
    UART6_Init();
#endif

    printf("\n");
    printf("+-------------------------------------------------------------+\n");
    printf("|        NuMicro USB Virtual COM Multi-Port Sample Code       |\n");
    printf("+-------------------------------------------------------------+\n");

    USBD_Open(&gsInfo, VCOM_ClassRequest, NULL);

    /* Endpoint configuration */
    VCOM_Init();
    USBD_Start();

#if (VCOM_CNT >= 1)
    NVIC_EnableIRQ(UART0_IRQn);
#endif

#if (VCOM_CNT >= 2)
    NVIC_EnableIRQ(UART1_IRQn);
#endif

#if (VCOM_CNT >= 3)
    NVIC_EnableIRQ(UART2_IRQn);
#endif

#if (VCOM_CNT >= 4)
    NVIC_EnableIRQ(UART3_IRQn);
#endif

#if (VCOM_CNT >= 5)
    NVIC_EnableIRQ(UART4_IRQn);
#endif

#if (VCOM_CNT >= 6)
    NVIC_EnableIRQ(UART5_IRQn);
#endif

#if (VCOM_CNT >= 7)
    NVIC_EnableIRQ(UART6_IRQn);
#endif

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
        {
            PowerDown();
        }

        VCOM_TransferData();
    }
}
