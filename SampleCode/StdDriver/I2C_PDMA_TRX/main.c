/**************************************************************************//**
 * @file     main.c
 * @version  V2.00
 * @brief    Demonstrate I2C PDMA mode and need to connect I2C0(Master) and I2C1(Slave).
 *
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 *
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"

#define I2C0_PDMA_TX_CH     0
#define I2C1_PDMA_RX_CH     1
#define I2C0_PDMA_RX_CH     2
#define I2C1_PDMA_TX_CH     3
#define PDMA_TEST_LENGTH    4

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
static uint8_t g_u8MasterTx_Buffer[PDMA_TEST_LENGTH];
static uint8_t g_u8MasterRx_Buffer[PDMA_TEST_LENGTH];
static uint8_t g_u8SlaveTx_Buffer[PDMA_TEST_LENGTH];
static uint8_t g_u8SlaveRx_Buffer[PDMA_TEST_LENGTH];
volatile uint32_t PDMA_DONE = 0;

volatile uint8_t g_u8DeviceAddr = 0x16;
volatile uint8_t g_u8EndFlag = 0;
volatile uint8_t g_u8MasterDataLen = 0;
volatile uint8_t g_u8SlaveDataLen = 0;
volatile uint16_t g_u8SlaveBufferAddr = 0;

typedef void (*I2C_FUNC)(uint32_t u32Status);

static I2C_FUNC s_I2C0HandlerFn = NULL;
static I2C_FUNC s_I2C1HandlerFn = NULL;


void PDMA0_IRQHandler(void)
{
    uint32_t u32Status = PDMA0->TDSTS;

    //Master TX
    if(u32Status & (0x1 << I2C0_PDMA_TX_CH))
    {
        printf("\n I2C0 Tx done  ");
        PDMA0->TDSTS = 0x1;
    }

    //Master RX
    if(u32Status & (0x1 << I2C0_PDMA_RX_CH))
    {
        printf("\n I2C0 Rx done  ");
        PDMA0->TDSTS = 0x1 << I2C0_PDMA_RX_CH;
        PDMA_DONE = 1;
    }

    //Slave RX
    if(u32Status & (0x1 << I2C1_PDMA_RX_CH))
    {
        printf("\n I2C1 Rx done  ");
        PDMA0->TDSTS = 0x1 << I2C1_PDMA_RX_CH;
        PDMA_DONE = 1;
    }

    //Slave TX
    if(u32Status & (0x1 << I2C1_PDMA_TX_CH))
    {
        printf("\n I2C1 Tx done  ");
        PDMA0->TDSTS = 0x1 << I2C1_PDMA_TX_CH;
    }
}


void I2C0_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C0);

    if(I2C_GET_TIMEOUT_FLAG(I2C0))
    {
        /* Clear I2C0 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C0);
    }
    else
    {
        if(s_I2C0HandlerFn != NULL)
            s_I2C0HandlerFn(u32Status);
    }
}


void I2C1_IRQHandler(void)
{
    uint32_t u32Status;

    u32Status = I2C_GET_STATUS(I2C1);

    if(I2C_GET_TIMEOUT_FLAG(I2C1))
    {
        /* Clear I2C1 Timeout Flag */
        I2C_ClearTimeoutFlag(I2C1);
    }
    else
    {
        if(s_I2C1HandlerFn != NULL)
            s_I2C1HandlerFn(u32Status);
    }
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C1 Slave TRx Callback Function                                                                       */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_PDMA_SlaveRx(uint32_t u32Status)
{
    if(u32Status == 0x60)                       /* Own SLA+W has been receive; ACK has been return */
    {
        /*
            Note:
            During PDMA operation, I2C controller will not occur receive Address ACK interrupt
        */
    }
    else if(u32Status == 0x80)                 /* Previously address with own SLA address
                                                  Data has been received; ACK has been returned*/
    {
        /*
            Note:
            During PDMA operation, I2C controller will not occur receive Data ACK interrupt
        */
    }
    else if(u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
        if(u32Status == 0x68)               /* Slave receive arbitration lost, clear SI */
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
        }
        else if(u32Status == 0xB0)          /* Address transmit arbitration lost, clear SI  */
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
        }
        else                                /* Slave bus error, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        }
    }
    I2C_WAIT_SI_CLEAR(I2C1);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C1 PDMA Slave Tx Callback Function                                                                   */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_PDMA_SlaveTx(uint32_t u32Status)
{
    uint8_t u8data;

    if(u32Status == 0x60)                        /* Own SLA+W has been receive; ACK has been return */
    {
        g_u8SlaveDataLen = 0;
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0x80)                 /* Previously address with own SLA address
                                                   Data has been received; ACK has been returned*/
    {
        u8data = (unsigned char) I2C_GET_DATA(I2C1);
        g_u8SlaveRx_Buffer[g_u8SlaveDataLen++] = u8data;
        g_u8SlaveBufferAddr = (g_u8SlaveRx_Buffer[0] << 8) + g_u8SlaveRx_Buffer[1];

        if(g_u8SlaveDataLen == 2)
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
        }
    }
    else if(u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0xA0)                 /* A STOP or repeated START has been received while still
                                                   addressed as Slave/Receiver*/
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);

        /* Enable I2C1 Slave TX */
        I2C1->CTL1 = I2C_CTL1_TXPDMAEN_Msk;
    }
    else if(u32Status == 0xA8)                   /* Own SLA+R has been receive; ACK has been return */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur START interrupt
        */
    }
    else if(u32Status == 0xB8)                   /* Data byte in I2CDAT has been transmitted ACK has been received */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur START interrupt
        */
    }
    else if(u32Status == 0xC0)                 /* Data byte or last data in I2CDAT has been transmitted
                                                   Not ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else if(u32Status == 0x88)                 /* Previously addressed with own SLA address; NOT ACK has
                                                   been returned */
    {
        I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
        if(u32Status == 0x68)               /* Slave receive arbitration lost, clear SI */
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
        }
        else if(u32Status == 0xB0)          /* Address transmit arbitration lost, clear SI  */
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI_AA);
        }
        else                                /* Slave bus error, stop I2C and clear SI */
        {
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI);
        }
    }
    I2C_WAIT_SI_CLEAR(I2C1);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C0 PDMA Master Tx Callback Function                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_PDMA_MasterTx(uint32_t u32Status)
{
    if(u32Status == 0x08)                       /* START has been transmitted */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur START interrupt
        */
    }
    else if(u32Status == 0x10)                  /* Repeat START has been transmitted */
    {

    }
    else if(u32Status == 0x18)                  /* SLA+W has been transmitted and ACK has been received */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur address ACK interrupt
        */
    }
    else if(u32Status == 0x20)                  /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if(u32Status == 0x28)                  /* DATA has been transmitted and ACK has been received */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur data ACK interrupt
        */
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
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
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO_SI);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
    }
    I2C_WAIT_SI_CLEAR(I2C0);
}

/*---------------------------------------------------------------------------------------------------------*/
/*  I2C1 PDMA Master Rx Callback Function                                                                  */
/*---------------------------------------------------------------------------------------------------------*/
void I2C_PDMA_MasterRx(uint32_t u32Status)
{
    if(u32Status == 0x08)                          /* START has been transmitted and prepare SLA+W */
    {
        I2C_SET_DATA(I2C0, (g_u8DeviceAddr << 1) | 0x00);     /* Write SLA+W to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x18)                     /* SLA+W has been transmitted and ACK has been received */
    {
        g_u8MasterDataLen = 1;
        I2C_SET_DATA(I2C0, g_u8MasterTx_Buffer[g_u8MasterDataLen++]);
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x20)                     /* SLA+W has been transmitted and NACK has been received */
    {
        I2C_STOP(I2C0);
        I2C_START(I2C0);
    }
    else if(u32Status == 0x28)                     /* DATA has been transmitted and ACK has been received */
    {
        if(g_u8MasterDataLen <= 2)
        {
            I2C_SET_DATA(I2C0, g_u8MasterTx_Buffer[g_u8MasterDataLen++]);
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
        }
        else
        {
            I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STA_SI);
        }
    }
    else if(u32Status == 0x10)                    /* Repeat START has been transmitted and prepare SLA+R */
    {
        I2C_SET_DATA(I2C0, (g_u8DeviceAddr << 1) | 0x01);   /* Write SLA+R to Register I2CDAT */
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI);
    }
    else if(u32Status == 0x40)                    /* SLA+R has been transmitted and ACK has been received */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_SI_AA);

        /* Enable I2C0 Master RX */
        I2C0->CTL1 = I2C_CTL1_RXPDMAEN_Msk;
    }
    else if(u32Status == 0x50)                    /* DATA has been received and ACK has been returned */
    {
        /*
           Note:
           During PDMA operation, I2C controller will not occur receive data ACK interrupt
        */
    }
    else if(u32Status == 0x58)                    /* DATA has been received and NACK has been returned */
    {
        I2C_SET_CONTROL_REG(I2C0, I2C_CTL_STO | I2C_CTL_SI);
    }
    else
    {
        /* TO DO */
        printf("Status 0x%x is NOT processed\n", u32Status);
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

    /* Enable I2C0, I2C1, and PDMA0 clock */
    CLK_EnableModuleClock(I2C0_MODULE);
    CLK_EnableModuleClock(I2C1_MODULE);
    CLK_EnableModuleClock(PDMA0_MODULE);

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();

    /* Set I2C0 multi-function pins */
    SET_I2C0_SDA_PA4();
    SET_I2C0_SCL_PA5();

    /* Set I2C1 multi-function pins */
    SET_I2C1_SDA_PA2();
    SET_I2C1_SCL_PA3();

    /* I2C pin enable schmitt trigger */
    PA->SMTEN |= GPIO_SMTEN_SMTEN4_Msk | GPIO_SMTEN_SMTEN5_Msk;
    PA->SMTEN |= GPIO_SMTEN_SMTEN2_Msk | GPIO_SMTEN_SMTEN3_Msk;
}

void I2C0_Init(void)
{
    /* Open I2C0 module and set bus clock */
    I2C_Open(I2C0, 100000);

    /* Get I2C0 Bus Clock */
    printf("I2C0 clock %d Hz\n", I2C_GetBusClockFreq(I2C0));

    /* Enable I2C0 interrupt */
    I2C_EnableInt(I2C0);
    NVIC_EnableIRQ(I2C0_IRQn);
}

void I2C1_Init(void)
{
    /* Open I2C1 module and set bus clock */
    I2C_Open(I2C1, 100000);

    /* Get I2C1 Bus Clock */
    printf("I2C1 clock %d Hz\n", I2C_GetBusClockFreq(I2C1));

    /* Set I2C1 4 Slave Addresses */
    I2C_SetSlaveAddr(I2C1, 0, 0x16, 0);   /* Slave Address : 0x16 */
    I2C_SetSlaveAddr(I2C1, 1, 0x36, 0);   /* Slave Address : 0x36 */
    I2C_SetSlaveAddr(I2C1, 2, 0x56, 0);   /* Slave Address : 0x56 */
    I2C_SetSlaveAddr(I2C1, 3, 0x76, 0);   /* Slave Address : 0x76 */

    /* Set I2C1 4 Slave Addresses Mask */
    I2C_SetSlaveAddrMask(I2C1, 0, 0x04);
    I2C_SetSlaveAddrMask(I2C1, 1, 0x02);
    I2C_SetSlaveAddrMask(I2C1, 2, 0x04);
    I2C_SetSlaveAddrMask(I2C1, 3, 0x02);

    /* Enable I2C1 interrupt */
    I2C_EnableInt(I2C1);
    NVIC_EnableIRQ(I2C1_IRQn);
}

void PDMA_Init(void)
{
    /* Open PDMA Channel */
    PDMA_Open(PDMA0, 1 << I2C0_PDMA_TX_CH); // Channel 0 for I2C0 TX
    PDMA_Open(PDMA0, 1 << I2C1_PDMA_RX_CH); // Channel 1 for I2C1 RX
    PDMA_Open(PDMA0, 1 << I2C0_PDMA_RX_CH); // Channel 2 for I2C0 RX
    PDMA_Open(PDMA0, 1 << I2C1_PDMA_TX_CH); // Channel 3 for I2C1 TX
    // Select basic mode
    PDMA_SetTransferMode(PDMA0, I2C0_PDMA_TX_CH, PDMA_I2C0_TX, 0, 0);
    PDMA_SetTransferMode(PDMA0, I2C1_PDMA_RX_CH, PDMA_I2C1_RX, 0, 0);
    PDMA_SetTransferMode(PDMA0, I2C0_PDMA_RX_CH, PDMA_I2C0_RX, 0, 0);
    PDMA_SetTransferMode(PDMA0, I2C1_PDMA_TX_CH, PDMA_I2C1_TX, 0, 0);
    // Set data width and transfer count
    PDMA_SetTransferCnt(PDMA0, I2C0_PDMA_TX_CH, PDMA_WIDTH_8, PDMA_TEST_LENGTH);
    PDMA_SetTransferCnt(PDMA0, I2C1_PDMA_RX_CH, PDMA_WIDTH_8, PDMA_TEST_LENGTH);
    PDMA_SetTransferCnt(PDMA0, I2C0_PDMA_RX_CH, PDMA_WIDTH_8, PDMA_TEST_LENGTH - 3); // except Slave Address and two bytes Data Address
    PDMA_SetTransferCnt(PDMA0, I2C1_PDMA_TX_CH, PDMA_WIDTH_8, PDMA_TEST_LENGTH - 3); // except Slave Address and two bytes Data Address
    //Set PDMA Transfer Address
    PDMA_SetTransferAddr(PDMA0, I2C0_PDMA_TX_CH, ((uint32_t)(&g_u8MasterTx_Buffer[0])), PDMA_SAR_INC, (uint32_t)(&(I2C0->DAT)), PDMA_DAR_FIX);
    PDMA_SetTransferAddr(PDMA0, I2C1_PDMA_RX_CH, (uint32_t)(&(I2C1->DAT)), PDMA_SAR_FIX, ((uint32_t)(&g_u8SlaveRx_Buffer[0])), PDMA_DAR_INC);
    PDMA_SetTransferAddr(PDMA0, I2C0_PDMA_RX_CH, (uint32_t)(&(I2C0->DAT)), PDMA_SAR_FIX, ((uint32_t)(&g_u8MasterRx_Buffer[0])), PDMA_DAR_INC);
    PDMA_SetTransferAddr(PDMA0, I2C1_PDMA_TX_CH, ((uint32_t)(&g_u8SlaveTx_Buffer[0])), PDMA_SAR_INC, (uint32_t)(&(I2C1->DAT)), PDMA_DAR_FIX);
    //Select Single Request
    PDMA_SetBurstType(PDMA0, I2C0_PDMA_TX_CH, PDMA_REQ_SINGLE, 0);
    PDMA_SetBurstType(PDMA0, I2C1_PDMA_RX_CH, PDMA_REQ_SINGLE, 0);
    PDMA_SetBurstType(PDMA0, I2C0_PDMA_RX_CH, PDMA_REQ_SINGLE, 0);
    PDMA_SetBurstType(PDMA0, I2C1_PDMA_TX_CH, PDMA_REQ_SINGLE, 0);

    PDMA_EnableInt(PDMA0, I2C0_PDMA_TX_CH, PDMA_INT_TRANS_DONE);
    PDMA_EnableInt(PDMA0, I2C1_PDMA_RX_CH, PDMA_INT_TRANS_DONE);
    PDMA_EnableInt(PDMA0, I2C0_PDMA_RX_CH, PDMA_INT_TRANS_DONE);
    PDMA_EnableInt(PDMA0, I2C1_PDMA_TX_CH, PDMA_INT_TRANS_DONE);
    NVIC_EnableIRQ(PDMA0_IRQn);
}
void I2C_PDMA(void)
{
    uint32_t i, u32TimeOutCnt;

    // I2C0
    GPIO_SetMode(PA, BIT4, GPIO_MODE_OPEN_DRAIN);
    GPIO_SetMode(PA, BIT5, GPIO_MODE_OPEN_DRAIN);
    PA4 = 1;
    PA5 = 1;

    // I2C1
    GPIO_SetMode(PA, BIT2, GPIO_MODE_OPEN_DRAIN);
    GPIO_SetMode(PA, BIT3, GPIO_MODE_OPEN_DRAIN);
    PA2 = 1;
    PA3 = 1;

    for(i = 0; i < PDMA_TEST_LENGTH; i++)
    {
        g_u8MasterTx_Buffer[i] = i;
        g_u8SlaveRx_Buffer[i] = 0xff;
    }

    g_u8MasterTx_Buffer[0] = ((g_u8DeviceAddr << 1) | 0x00);   //1 byte SLV + W
    g_u8MasterTx_Buffer[1] = 0x00;                             //2 bytes Data address
    g_u8MasterTx_Buffer[2] = 0x00;

    PDMA_Init();

    /* I2C enter no address SLV mode */
    I2C_SET_CONTROL_REG(I2C1, I2C_CTL_SI | I2C_CTL_AA);
    /* Enable I2C1 PDMA RX mode */
    I2C1->CTL1 = I2C_CTL1_RXPDMAEN_Msk;

    /* I2C1 function to Slave receive/transmit data */
    s_I2C1HandlerFn = I2C_PDMA_SlaveRx;

    PDMA_DONE = 0;

    /* Enable I2C TX */
    I2C0->CTL1 = I2C_CTL1_TXPDMAEN_Msk;

    s_I2C0HandlerFn = (I2C_FUNC)I2C_PDMA_MasterTx;

    /* Send START condition, start the PDMA data transmit */
    I2C_START(I2C0);

    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!PDMA_DONE)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA time-out!\n");
            return;
        }
    }

    /* Disable I2C0 PDMA TX mode */
    I2C0->CTL1 &= ~I2C_CTL1_TXPDMAEN_Msk;
    /* Disable I2C1 PDMA RX mode */
    I2C1->CTL1 &= ~I2C_CTL1_RXPDMAEN_Msk;

    for(i = 0; i < PDMA_TEST_LENGTH; i++)
    {
        if(g_u8SlaveRx_Buffer[i] != g_u8MasterTx_Buffer[i])
        {
            printf("\n Receive Data Compare Error !!");
            return;
        }
        else
        {
            if(i > 2)
                g_u8SlaveTx_Buffer[i - 3] = g_u8MasterTx_Buffer[i];
        }
    }

    /* Test Master RX and Slave TX with PDMA function */

    /* I2C0 function to Master receive data */
    s_I2C0HandlerFn = (I2C_FUNC)I2C_PDMA_MasterRx;
    /* I2C1 function to Slave transmit data */
    s_I2C1HandlerFn = I2C_PDMA_SlaveTx;

    PDMA_DONE = 0;

    /* Send START condition */
    I2C_START(I2C0);

    u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
    while(!PDMA_DONE)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Wait for PDMA transfer done time-out!\n");
            return;
        }
    }

    /* Disable I2C0 PDMA RX mode */
    I2C0->CTL1 &= ~I2C_CTL1_RXPDMAEN_Msk;
    /* Disable I2C1 PDMA TX mode */
    I2C1->CTL1 &= ~I2C_CTL1_TXPDMAEN_Msk;

    for(i = 0; i < PDMA_TEST_LENGTH - 3; i++)
    {
        if(g_u8MasterRx_Buffer[i] != g_u8MasterTx_Buffer[i + 3])
        {
            printf("\n Slave Receive Data Compare Error !!");
            return;
        }
    }

    printf("\nI2C PDMA test Pass.\n");
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
    printf("|       I2C Driver Sample Code for PDMA                 |\n");
    printf("|                                                       |\n");
    printf("| I2C Master (I2C0) <---> I2C Slave(I2C1)               |\n");
    printf("+-------------------------------------------------------+\n");

    /* Init I2C0 */
    I2C0_Init();
    /* Init I2C1 */
    I2C1_Init();

    I2C_PDMA();

    while(1);
}
