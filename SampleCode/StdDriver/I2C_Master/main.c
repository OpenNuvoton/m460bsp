/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * @brief    M460 I2C Driver Sample Code
 *           This is a I2C master mode demo and need to be tested with a slave device.
 *
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

// *** <<< Use Configuration Wizard in Context Menu >>> ***
// <e> I2C addressing mode transfer
#define I2C_10Bit_MODE 0
//  <o> Addressing Mode Interface
//  <0=> 7-bit <1=> 10-bit
// </e>
//*** <<< end of configuration section >>> ***

#if (I2C_10Bit_MODE)
#define SLV_10BIT_ADDR (0x1E<<2)
#endif
/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static volatile uint8_t g_u8DeviceAddr;
#if (I2C_10Bit_MODE)
static volatile uint8_t g_u8DeviceLAddr;
static volatile uint8_t g_au8MstTxData[4];
#else
static volatile uint8_t g_au8MstTxData[3];
#endif
static volatile uint8_t g_u8MstDataLen;
static volatile uint8_t g_u8MstEndFlag = 0;
static volatile uint8_t g_u8MstTxAbortFlag = 0;
static volatile uint8_t g_u8MstRxAbortFlag = 0;
static volatile uint8_t g_u8MstReStartFlag = 0;
static volatile uint8_t g_u8TimeoutFlag = 0;
static  uint8_t g_u8MstRxData;

typedef void (*I2C_FUNC)(uint32_t u32Status);

static volatile I2C_FUNC s_I2C0HandlerFn = NULL;

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 IRQ Handler                                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C0);

    if(I2C_GET_TIMEOUT_FLAG(I2C0))
    {
        /* Clear I2C0 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C0);
        g_u8TimeoutFlag = 1;
    }
    else
    {
        if(s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Rx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterRx(uint32_t u32Status)
{
    uint32_t u32TimeOutCnt;

    if(u32Status == 0x08)                       /* START has been transmitted and prepare SLA+W */
    {
        I2C_SET_DATA(I2C0, (g_u8DeviceAddr << 1)); /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA | I2C_CTL_STO | I2C_CTL_SI);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
#if (I2C_10Bit_MODE)
        if(g_u8MstDataLen != 3)
#else
        if(g_u8MstDataLen != 2)
#endif
        {
            I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA | I2C_CTL_SI);
        }
    }
    else if(u32Status == 0x10)                  /* Repeat START has been transmitted and prepare SLA+R */
    {
        I2C_SET_DATA(I2C0, (g_u8DeviceAddr << 1) | 0x01);  /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x40)                  /* SLA+R has been transmitted and ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x58)                  /* DATA has been received and NACK has been returned */
    {
        g_u8MstRxData = I2C_GET_DATA(I2C0);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO | I2C_CTL_SI);
        g_u8MstEndFlag = 1;
    }
    else
    {
        /* Error condition process */
        printf("[MasterRx] Status [0x%x] Unexpected abort!! Anykey to re-start\n", u32Status);
        if(u32Status == 0x38)                 /* Master arbitration lost, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if(u32Status == 0x30)            /* Master transmit data NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if(u32Status == 0x48)            /* Master receive address NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if(u32Status == 0x00)            /* Master bus error, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        /*Setting MasterRx abort flag for re-start mechanism*/
        g_u8MstRxAbortFlag = 1;
        getchar();
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        u32TimeOutCnt = I2C_TIMEOUT;
        while(I2C0->CTL0 & I2C_CTL0_SI_Msk)
            if(--u32TimeOutCnt == 0) break;
    }
    I2C_WAIT_SI_CLEAR(I2C0);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C Tx Callback Function                                                                               */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_MasterTx(uint32_t u32Status)
{
    uint32_t u32TimeOutCnt;

    if(u32Status == 0x08)                       /* START has been transmitted */
    {
        I2C_SET_DATA(I2C0, g_u8DeviceAddr << 1);  /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA | I2C_CTL_STO | I2C_CTL_SI);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
#if (I2C_10Bit_MODE)
        if(g_u8MstDataLen != 4)
#else
        if(g_u8MstDataLen != 3)
#endif
        {
            I2C_SET_DATA(I2C0, g_au8MstTxData[g_u8MstDataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO | I2C_CTL_SI);
            g_u8MstEndFlag = 1;
        }
    }
    else
    {
        /* Error condition process */
        printf("[MasterTx] Status [0x%x] Unexpected abort!! Anykey to re-start\n", u32Status);

        if(u32Status == 0x38)                   /* Master arbitration lost, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if(u32Status == 0x00)              /* Master bus error, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if(u32Status == 0x30)              /* Master transmit data NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if(u32Status == 0x48)              /* Master receive address NACK, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else if(u32Status == 0x10)              /* Master repeat start, clear SI */
        {
            I2C_SET_DATA(I2C0, (uint32_t)((g_u8DeviceAddr << 1) | 0x01));   /* Write SLA+R to Register I2CDAT */
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        /*Setting MasterTRx abort flag for re-start mechanism*/
        g_u8MstTxAbortFlag = 1;
        getchar();
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        u32TimeOutCnt = I2C_TIMEOUT;
        while(I2C0->CTL0 & I2C_CTL0_SI_Msk)
            if(--u32TimeOutCnt == 0) break;
    }
    I2C_WAIT_SI_CLEAR(I2C0);
}

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

    /* Enable I2C0 clock */
    CLK_EnableModuleClock(I2C0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set I2C0 multi-function pins */
    SET_I2C0_SDA_PA4();
    SET_I2C0_SCL_PA5();

    /* I2C pin enable schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN4_Msk | GPIO_SMTEN_SMTEN5_Msk;
}

void I2C0_Init(void)
{
#if (I2C_10Bit_MODE)
    /* Enable I2C0 10-bit address mode */
    I2C0->CTL1 |= I2C_CTL1_ADDR10EN_Msk;
#endif
    /* Open I2C0 and set clock to 100k */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    printf("I2C clock %d Hz\n", I2C_GetBusClockFreq(I2C0));

    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
}

int32_t Read_Write_SLAVE(uint32_t slvaddr)
{
    uint32_t i;

    do
    {
        /* Enable I2C timeout */
        I2C_EnableTimeout(I2C0, 0);
        g_u8MstReStartFlag = 0;
#if (I2C_10Bit_MODE)
        g_u8DeviceAddr = (slvaddr >> 8) | SLV_10BIT_ADDR;
        g_u8DeviceLAddr = slvaddr & 0xFF;
#else
        g_u8DeviceAddr = slvaddr;
#endif
        g_u8TimeoutFlag = 0;

        for(i = 0; i < 0x100; i++)
        {
#if (I2C_10Bit_MODE)
            g_au8MstTxData[0] = (uint8_t)g_u8DeviceLAddr;
            g_au8MstTxData[1] = (uint8_t)((i & 0xFF00) >> 8);
            g_au8MstTxData[2] = (uint8_t)(i & 0x00FF);
            g_au8MstTxData[3] = (uint8_t)(g_au8MstTxData[1] + 3);
#else
            g_au8MstTxData[0] = (uint8_t)((i & 0xFF00) >> 8);
            g_au8MstTxData[1] = (uint8_t)(i & 0x00FF);
            g_au8MstTxData[2] = (uint8_t)(g_au8MstTxData[1] + 3);
#endif

            g_u8MstDataLen = 0;
            g_u8MstEndFlag = 0;

            /* I2C function to write data to slave */
            s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterTx;

            /* I2C as master sends START signal */
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

            /* Wait I2C Tx Finish or Unexpected Abort*/
            do
            {
                if(g_u8TimeoutFlag)
                {
                    printf(" MasterTx time out, any to reset IP\n");
                    getchar();
                    SYS->IPRST1 |= SYS_IPRST1_I2C0RST_Msk;
                    SYS->IPRST1 = 0;
                    I2C0_Init();
                    /* Set MasterTx abort flag*/
                    g_u8MstTxAbortFlag = 1;
                }
            }
            while(g_u8MstEndFlag == 0 && g_u8MstTxAbortFlag == 0);

            g_u8MstEndFlag = 0;

            if(g_u8MstTxAbortFlag)
            {
                /* Clear MasterTx abort flag*/
                g_u8MstTxAbortFlag = 0;
                /* Set Master re-start flag*/
                g_u8MstReStartFlag = 1;
                break;
            }

            /* I2C function to read data from slave */
            s_I2C0HandlerFn = (I2C_FUNC)I2C_MasterRx;

            g_u8MstDataLen = 0;
#if (I2C_10Bit_MODE)
            g_u8DeviceAddr = (slvaddr >> 8) | SLV_10BIT_ADDR;
#else
            g_u8DeviceAddr = slvaddr;
#endif

            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA);

            /* Wait I2C Rx Finish or Unexpected Abort*/
            do
            {
                if(g_u8TimeoutFlag)
                {
                    /* When I2C timeout, reset IP*/
                    printf(" MasterRx time out, any to reset IP\n");
                    getchar();
                    SYS->IPRST1 |= SYS_IPRST1_I2C0RST_Msk;
                    SYS->IPRST1 = 0;
                    I2C0_Init();
                    /* Set MasterRx abort flag*/
                    g_u8MstRxAbortFlag = 1;
                }
            }
            while(g_u8MstEndFlag == 0 && g_u8MstRxAbortFlag == 0);

            g_u8MstEndFlag = 0;

            if(g_u8MstRxAbortFlag)
            {
                /* Clear MasterRx abort flag*/
                g_u8MstRxAbortFlag = 0;
                /* Set Master re-start flag*/
                g_u8MstReStartFlag = 1;
                break;
            }

            /* Compare data */
#if (I2C_10Bit_MODE)
            if(g_u8MstRxData != g_au8MstTxData[3])
#else
            if(g_u8MstRxData != g_au8MstTxData[2])
#endif
            {
                printf("I2C Byte Write/Read Failed, Data 0x%x\n", g_u8MstRxData);
                return -1;
            }
        }
    }
    while(g_u8MstReStartFlag);   /*If unexpected abort happens, re-start the transmition*/

    printf("Master Access Slave (0x%X) Test OK\n", slvaddr);
    return 0;
}

int main(void)
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    /* Init System, IP clock and multi-function I/O */
    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /*
        This sample code sets I2C bus clock to 100kHz. Then, Master accesses Slave with Byte Write
        and Byte Read operations, and check if the read data is equal to the programmed data.
    */

    printf("+-------------------------------------------------------+\n");
#if (I2C_10Bit_MODE)
    printf("| I2C Driver 10bit Sample Code(Master) for access Slave |\n");
#else
    printf("|       I2C Driver Sample Code(Master) for access Slave |\n");
#endif
    printf("+-------------------------------------------------------+\n");

    /* Init I2C0 */
    I2C0_Init();

    /* Access Slave with no address mask */
    printf("\n");
    printf(" == No Mask Address ==\n");
#if (I2C_10Bit_MODE)
    Read_Write_SLAVE(0x116);
    Read_Write_SLAVE(0x136);
    Read_Write_SLAVE(0x156);
    Read_Write_SLAVE(0x176);
#else
    Read_Write_SLAVE(0x15);
    Read_Write_SLAVE(0x35);
    Read_Write_SLAVE(0x55);
    Read_Write_SLAVE(0x75);
#endif
    printf("SLAVE Address test OK.\n");

    /* Access Slave with address mask */
    printf("\n");
    printf(" == Mask Address ==\n");
#if (I2C_10Bit_MODE)
    Read_Write_SLAVE(0x116 & ~0x04);
    Read_Write_SLAVE(0x136 & ~0x02);
    Read_Write_SLAVE(0x156 & ~0x04);
    Read_Write_SLAVE(0x176 & ~0x02);
#else
    Read_Write_SLAVE(0x15 & ~0x01);
    Read_Write_SLAVE(0x35 & ~0x04);
    Read_Write_SLAVE(0x55 & ~0x01);
    Read_Write_SLAVE(0x75 & ~0x04);
#endif
    printf("SLAVE Address Mask test OK.\n");

    while(1);
}
