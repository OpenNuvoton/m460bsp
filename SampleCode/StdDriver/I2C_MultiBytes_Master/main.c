/**************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief
 *           Show how to set I2C use Multi bytes API Read and Write data to Slave.
 *           Needs to work with I2C_Slave sample code.
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define PLL_CLOCK       192000000

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
volatile uint8_t g_u8DeviceAddr;
volatile uint8_t g_au8MstTxData[3];
volatile uint8_t g_u8MstRxData;
volatile uint8_t g_u8MstDataLen;
volatile uint8_t g_u8MstEndFlag = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);
volatile static I2C_FUNC s_I2C0HandlerFn = NULL;


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

    /* Enable I2C0 peripheral clock */
    CLK_EnableModuleClock(I2C0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();;

    /* Set I2C0 multi-function pins */
    SET_I2C0_SDA_PA4();
    SET_I2C0_SCL_PA5();

    /* I2C pin enable schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN4_Msk | GPIO_SMTEN_SMTEN5_Msk;
}

void I2C0_Init(void)
{
    /* Open I2C module and set bus clock */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));
}

void I2C0_Close(void)
{
    /* Disable I2C0 interrupt and clear corresponding NVIC bit */
    I2C_DisableInt(I2C0);
    NVIC_DisableIRQ(I2C0_IRQn);

    /* Disable I2C0 and close I2C0 clock */
    I2C_Close(I2C0);
    CLK_DisableModuleClock(I2C0_MODULE);

}


int main(void)
{
    uint32_t i;
    uint8_t txbuf[256] = {0}, rDataBuf[256] = {0};

    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O. */
    SYS_Init();

    /* Configure UART0: 115200, 8-bit word, no parity bit, 1 stop bit. */
    UART_Open(UART0, 115200);

    /*
        This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Multi Bytes Write
        and Multi Bytes Read operations, and check if the read data is equal to the programmed data.
    */
    printf("+--------------------------------------------------------+\n");
    printf("| I2C Driver Sample Code for Multi Bytes Read/Write Test |\n");
    printf("| Needs to work with I2C_Slave sample code               |\n");
    printf("|                                                        |\n");
    printf("| I2C Master (I2C0) <---> I2C Slave(I2C0)                |\n");
    printf("| !! This sample code requires two borads to test !!     |\n");
    printf("+--------------------------------------------------------+\n");

    printf("\n");

    /* Init I2C0 */
    I2C0_Init();

    /* Slave address */
    g_u8DeviceAddr = 0x15;

    /* Prepare data for transmission */
    for(i = 0; i < 256; i++)
    {
        txbuf[i] = (uint8_t) i + 3;
    }

    for(i = 0; i < 256; i += 32)
    {
        /* Write 32 bytes data to Slave */
        while(I2C_WriteMultiBytesTwoRegs(I2C0, g_u8DeviceAddr, i, &txbuf[i], 32) < 32);
    }

    printf("Multi bytes Write access Pass.....\n");

    printf("\n");

    /* Use Multi Bytes Read from Slave (Two Registers) */
    while(I2C_ReadMultiBytesTwoRegs(I2C0, g_u8DeviceAddr, 0x0000, rDataBuf, 256) < 256);

    /* Compare TX data and RX data */
    for(i = 0; i < 256; i++)
    {
        if(txbuf[i] != rDataBuf[i])
        {
            printf("Data compare fail... R[%d] Data: 0x%X\n", i, rDataBuf[i]);
            goto failExit;
        }
    }
    printf("Multi bytes Read access Pass.....\n");
    while(1);

failExit:
    printf("Multi bytes Read access Fail.....\n");
    while(1);
}
/*** (C) COPYRIGHT 2021 Nuvoton Technology Corp. ***/
