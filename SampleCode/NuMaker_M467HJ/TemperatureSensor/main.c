/******************************************************************************
 * @file     main.c
 * @version  V1.00
 * @brief    A simple demo for NuMaker-M467HJ board to show message from UART0 to ICE VCOM and show LED
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "clk.h"
#include "uart.h"
#include "gpio.h"
#include "sys.h"

#define SENSOR_ADDR     (0x48)


#define I2C_GO(i2c, cmd)   {\
    I2C_SET_CONTROL_REG(i2c, cmd);\
    while(!(i2c->CTL0 & I2C_CTL0_SI_Msk));\
}


#define LED_INIT()  (PH->MODE = ((PH->MODE &(~(0x3ful << 4*2))) | (0x15ul << 4 *2)))
#define LED_RED     PH4
#define LED_YELLOW  PH5
#define LED_GREEN   PH6

void SYS_Init(void);
void UART0_Init(void);

void SYS_Init(void)
{

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HIRC clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HIRCEN_Msk);

    /* Wait for HIRC clock ready */
    CLK_WaitClockReady(CLK_STATUS_HIRCSTB_Msk);

    /* Set core clock to 192MHz */
    CLK_SetCoreClock(192000000);

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Enable I2C module clock */
    CLK_EnableModuleClock(I2C2_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();


    /* I2C2 MFP for temperature sensor */
    SET_I2C2_SDA_PD0();
    SET_I2C2_SCL_PD1();

    /* Enable pull-up and schmitt for I2C */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPDCKEN_Msk;
    GPIO_SetPullCtl(PD, BIT0 | BIT1, GPIO_PUSEL_PULL_UP);
    PD->SMTEN |= BIT0 | BIT1;

}

void UART0_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init UART                                                                                               */
    /*---------------------------------------------------------------------------------------------------------*/
    UART0->LINE = UART_PARITY_NONE | UART_STOP_BIT_1 | UART_WORD_LEN_8;
    UART0->BAUD = UART_BAUD_MODE2 | UART_BAUD_MODE2_DIVIDER(__HIRC, 115200);

}

int32_t SENSOR_Read(int32_t idx)
{
    I2C_T* master;
    int32_t err = 0;
    uint32_t s;
    uint8_t u8Data;

    master = I2C2;

    /* Master: START */
    I2C_GO(master, I2C_CTL_STA_SI);

    /* Master: SLA + W */
    I2C_SET_DATA(master, SENSOR_ADDR << 1 | 0);
    I2C_GO(master, I2C_CTL_SI);

    s = I2C_GET_STATUS(master);
    if(s == 0x18)
    {
        /* Master: SLA + W ACK */

        I2C_SET_DATA(master, idx);
        I2C_GO(master, I2C_CTL_SI);

        s = I2C_GET_STATUS(master);
        if(s == 0x28)
        {
            /* Master: sensor register index ACK */

            /* repeat start */
            I2C_GO(master, I2C_CTL_STA_SI);

            s = I2C_GET_STATUS(master);
            if(s == 0x10)
            {
                /* Repeat Start OK */
                
                /* Master: SLA + R */
                I2C_SET_DATA(master, SENSOR_ADDR << 1 | 1);
                I2C_GO(master, I2C_CTL_SI);

                s = I2C_GET_STATUS(master);
                if(s == 0x40)
                {
                    /* Master: SLA + R ACK */

                    /* Master: Receive data from slave */
                    I2C_GO(master, I2C_CTL_SI_AA);
                    u8Data = I2C_GET_DATA(master);

                    err = u8Data;
                    goto lexit;

                }
                else if(s == 0x48)
                {
                    /* Master: SLA + R NAK */

                    printf("SLA + R NAK\n");
                    err = -1;
                    goto lexit;
                }
                else
                {
                    /* Master: Unknown */
                    err = -1;
                    printf("Master STATUS=%02x\n", s);
                    goto lexit;
                }
            }
            else
            {
                /* Master: Unknown */
                err = -1;
                printf("Master STATUS=%02x\n", s);
                goto lexit;
            }
        }
        else if(s == 0x30)
        {
            /* Master: sensor register index NAK */

            printf("Master send data NAK\n");
            err = -1;
            goto lexit;

        }
        else
        {
            /* Master: Unknown */
            err = -1;
            printf("Master STATUS=%02x\n", s);
            goto lexit;
        }


    }
    else if(s == 0x20)
    {
        /* Master: SLA + W + NAK */

        printf("SLA+W NAK. Device not existed\n");
        err = -1;
        goto lexit;
    }
    else
    {
        /* Master: Unknown */
        err = -1;
        printf("Master STATUS=%02x\n", s);
        goto lexit;
    }


lexit:

    /* Master: STOP */
    I2C_SET_CONTROL_REG(master, I2C_CTL_STO_SI);


    return err;
}


int32_t SENSOR_Write(int32_t idx, uint8_t u8Data)
{
    I2C_T* master;
    int32_t err = 0;
    int32_t s;

    master = I2C2;

    /* Master: START */
    I2C_GO(master, I2C_CTL_STA_SI);

    /* Master: SLA + W */
    I2C_SET_DATA(master, SENSOR_ADDR << 1 | 0);
    I2C_GO(master, I2C_CTL_SI);

    if(I2C_GET_STATUS(master) == 0x18)
    {
        /* Master: SLA + W ACK */

        /* Master: Write register index */
        I2C_SET_DATA(master, idx);
        I2C_GO(master, I2C_CTL_SI);

        s = I2C_GET_STATUS(master);
        if(s == 0x28)
        {
            /* Master: register index ACK */

            /* Master: Write data */
            I2C_SET_DATA(master, u8Data);
            I2C_GO(master, I2C_CTL_SI);

            s = I2C_GET_STATUS(master);
            if(s == 0x28)
            {
                /* Master: write data ACK */
                goto lexit;

            }
            else if(s == 0x30)
            {
                /* Master: sensor register index NAK */

                printf("Master write data NAK\n");
                err = -1;
                goto lexit;

            }
            else
            {
                /* Master: Unknown */
                err = -1;
                printf("Master STATUS=%02x\n", s);
                goto lexit;
            }

        }
        else if(s == 0x30)
        {
            /* Master: sensor register index NAK */

            printf("Master write register idx NAK\n");
            err = -1;
            goto lexit;

        }
        else
        {
            /* Master: Unknown */
            err = -1;
            printf("Master STATUS=%02x\n", s);
            goto lexit;
        }


    }
    else if(I2C_GET_STATUS(master) == 0x20)
    {
        /* Master: SLA + W + NAK */

        printf("SLA+W NAK. Device not existed\n");
        err = -1;
        goto lexit;
    }
    else
    {
        /* Master: Unknown */
        err = -1;
        printf("Master STATUS=%02x\n", I2C_GET_STATUS(master));
    }


lexit:

    /* Master: STOP */
    I2C_SET_CONTROL_REG(master, I2C_CTL_STO_SI);


    return err;
}



int main()
{
    int32_t i32Data;
    
    SYS_UnlockReg();

    SYS_Init();

    UART0_Init();
    
    I2C_Open(I2C2, 100000);

    /* Enable GPIO PH to control LED */
    CLK->AHBCLK0 |= CLK_AHBCLK0_GPHCKEN_Msk;
    
    LED_INIT();

    printf("\n");
    printf("+------------------------------------------------------------------+\n");
    printf("|                     Temperature Sensor Demo                      |\n");
    printf("+------------------------------------------------------------------+\n");

    i32Data = SENSOR_Read(0xfd);
    printf("CID = 0x%02x\n", i32Data);
    i32Data = SENSOR_Read(0xfe);
    printf("VID = 0x%02x\n", i32Data);
    i32Data = SENSOR_Read(0xff);
    printf("DID = 0x%02x\n", i32Data);
    
    while(1)
    {
        /* The local temperature(on - die) data with 8 - bit 2`s complement format. */
        i32Data = (int8_t)SENSOR_Read(0x0);

        printf("Temperature = %d degrees Celsius  \r", i32Data);
        CLK_SysTickLongDelay(1000000);
        
        LED_GREEN ^= 1;
    }


}
