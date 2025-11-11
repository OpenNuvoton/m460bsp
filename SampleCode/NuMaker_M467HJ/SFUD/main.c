/*************************************************************************//**
 * @file     main.c
 * @version  V3.00
 * @brief    A project template for M460 MCU.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include "stdio.h"
#include "NuMicro.h"
#include "sfud.h"

#define SFUD_DEMO_TEST_BUFFER_SIZE                     1024
static void SFUD_Demo(uint32_t u32Addr, uint32_t u32Size, uint8_t *pu8Data, uint8_t u8Device);
static uint8_t s_au8SfudDemoBuf[SFUD_DEMO_TEST_BUFFER_SIZE] __attribute__((aligned(4)));//4-byte alignment required by SPIM DMA controller.

void SYS_Init(void)
{
    /*---------------------------------------------------------------------------------------------------------*/
    /* Init System Clock                                                                                       */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Enable HXT clock */
    CLK_EnableXtalRC(CLK_PWRCTL_HXTEN_Msk);

    /* Wait for HXT clock ready */
    CLK_WaitClockReady(CLK_STATUS_HXTSTB_Msk);

    /* Set PCLK0 and PCLK1 to HCLK/2 */
    CLK->PCLKDIV = (CLK_PCLKDIV_APB0DIV_DIV2 | CLK_PCLKDIV_APB1DIV_DIV2);

    /* Set core clock to 200MHz */
    CLK_SetCoreClock(200000000);

    /* Enable UART0 module clock */
    CLK_EnableModuleClock(UART0_MODULE);

    /* Select UART0 module clock source as HIRC and UART0 module clock divider as 1 */
    CLK_SetModuleClock(UART0_MODULE, CLK_CLKSEL1_UART0SEL_HIRC, CLK_CLKDIV0_UART0(1));

    /* Set GPB multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();
}

/**
 * SFUD demo for the first flash device test.
 *
 * @param addr flash start address
 * @param size test flash size
 * @param size test flash data buffer
 */
static void SFUD_Demo(uint32_t u32Addr, uint32_t u32Size, uint8_t *pu8Data, uint8_t u8Device)
{
    sfud_err result = SFUD_SUCCESS;
    const sfud_flash *sFlash = sfud_get_device(u8Device);
    size_t i;
    /* Prepare write data */
    for (i = 0; i < u32Size; i++)
    {
        pu8Data[i] = i;
    }
    /* Erase test */
    result = sfud_erase(sFlash, u32Addr, u32Size);
    if (result == SFUD_SUCCESS)
    {
        printf("Erase the %s flash data finish. Start from 0x%08X, size is %d.\r\n", sFlash->name, u32Addr, u32Size);
    }
    else
    {
        printf("Erase the %s flash data failed.\r\n", sFlash->name);
        return;
    }
    /* write test */
    result = sfud_write(sFlash, u32Addr, u32Size, pu8Data);
    if (result == SFUD_SUCCESS)
    {
        printf("Write the %s flash data finish. Start from 0x%08X, size is %d.\r\n", sFlash->name, u32Addr, u32Size);
    }
    else
    {
        printf("Write the %s flash data failed.\r\n", sFlash->name);
        return;
    }
    /* Read test */
    for (i = 0; i < u32Size; i++)// Clear Buffer
    {
        pu8Data[i] = 0;
    }
    result = sfud_read(sFlash, u32Addr, u32Size, pu8Data);
    if (result == SFUD_SUCCESS)
    {
        printf("Read the %s flash data success. Start from 0x%08X, size is %d. The data is:\r\n", sFlash->name, u32Addr, u32Size);
        printf("Offset (h) 00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\r\n");
        for (i = 0; i < u32Size; i++)
        {
            if (i % 16 == 0)
            {
                printf("[%08X] ", u32Addr + i);
            }
            printf("%02X ", pu8Data[i]);
            if (((i + 1) % 16 == 0) || i == u32Size - 1)
            {
                printf("\r\n");
            }
        }
        printf("\r\n");
    }
    else
    {
        printf("Read the %s flash data failed.\r\n", sFlash->name);
    }
    /* data check */
    for (i = 0; i < u32Size; i++)
    {
        if (pu8Data[i] != i % 256)
        {
            printf("Read and check write data has an error. Write the %s flash data failed.\r\n", sFlash->name);
            break;
        }
    }
    if (i == u32Size)
    {
        printf("The %s flash test is success.\r\n\r\n", sFlash->name);
    }
}
/*
 * This is a template project for M460 series MCU. Users could based on this project to create their
 * own application without worry about the IAR/Keil project settings.
 *
 * This template application uses external crystal as HCLK source and configures UART0 to print out
 * "Hello World", users may need to do extra system configuration based on their system design.
 */

int main()
{
    /* Unlock protected registers */
    SYS_UnlockReg();

    SYS_Init();

    /* Init UART to 115200-8n1 for print message */
    UART_Open(UART0, 115200);

    /* Connect UART to PC, and open a terminal tool to receive following message */
    printf("\n\n");
    printf("+-------------------------------------------------------------------------+\n");
    printf("|                       SFUD with Flash Sample Code                       |\n");
    printf("+-------------------------------------------------------------------------+\n");

    /* SFUD initialize */
    sfud_init();
    for (uint8_t u8Idx = 0; u8Idx < sfud_get_device_num(); u8Idx++)
    {
        sfud_flash *sFlash = sfud_get_device(u8Idx);
        if(sFlash->init_ok == true)
        {
            printf("\r\nThe %s flash demo start.\r\n", sFlash->name);
#ifdef SFUD_USING_QSPI
            /* Enable qspi fast read mode, set four data lines width. */
            sfud_qspi_fast_read_enable(sfud_get_device(u8Idx), 4);
#endif
            SFUD_Demo(0, sizeof(s_au8SfudDemoBuf), s_au8SfudDemoBuf, u8Idx);
        }
    }

    /* Got no where to go, just loop forever */
    while(1);
}
