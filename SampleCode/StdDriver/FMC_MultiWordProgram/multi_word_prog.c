/**************************************************************************//**
 * @file     multi_word_prog.c
 * @version  V1.00
 * @brief    This sample run on SRAM to show FMC multi word program function.
 *
 *
 * @copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>

#include "NuMicro.h"

#define USE_DRIVER_API


uint32_t    page_buff[FMC_FLASH_PAGE_SIZE / 4];


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

    /*---------------------------------------------------------------------------------------------------------*/
    /* Init I/O Multi-function                                                                                 */
    /*---------------------------------------------------------------------------------------------------------*/

    /* Set multi-function pins for UART0 RXD and TXD */
    SET_UART0_RXD_PB12();
    SET_UART0_TXD_PB13();
}

void UART0_Init(void)
{
    /* Configure UART0 and set UART0 baud rate */
    UART_Open(UART0, 115200);
}

int  multi_word_program(uint32_t start_addr)
{
    uint32_t    i, u32TimeOutCnt;

    printf("    program address 0x%x\n", start_addr);

    FMC->ISPADDR = start_addr;
    FMC->MPDAT0  = page_buff[0];
    FMC->MPDAT1  = page_buff[1];
    FMC->MPDAT2  = page_buff[2];
    FMC->MPDAT3  = page_buff[3];
    FMC->ISPCMD  = FMC_ISPCMD_PROGRAM_MUL;
    FMC->ISPTRG  = FMC_ISPTRG_ISPGO_Msk;

    for(i = 4; i < FMC_MULTI_WORD_PROG_LEN / 4;)
    {
        u32TimeOutCnt = FMC_TIMEOUT_WRITE;
        while(FMC->MPSTS & (FMC_MPSTS_D0_Msk | FMC_MPSTS_D1_Msk))
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Time-out occurred on waiting D0/D1!\n");
                return -1;
            }
        }

        if(!(FMC->MPSTS & FMC_MPSTS_MPBUSY_Msk))
        {
            printf("    [WARNING] busy cleared after D0D1 cleared!\n");
            i += 2;
            break;
        }

        FMC->MPDAT0 = page_buff[i++];
        FMC->MPDAT1 = page_buff[i++];

        if(i == FMC_MULTI_WORD_PROG_LEN / 4)
            return 0;           // done

        u32TimeOutCnt = FMC_TIMEOUT_WRITE;
        while(FMC->MPSTS & (FMC_MPSTS_D2_Msk | FMC_MPSTS_D3_Msk))
        {
            if(--u32TimeOutCnt == 0)
            {
                printf("Time-out occurred on waiting D2/D3!\n");
                return -1;
            }
        }

        if(!(FMC->MPSTS & FMC_MPSTS_MPBUSY_Msk))
        {
            printf("    [WARNING] busy cleared after D2D3 cleared!\n");
            i += 2;
            break;
        }

        FMC->MPDAT2 = page_buff[i++];
        FMC->MPDAT3 = page_buff[i++];
    }

    if(i != FMC_MULTI_WORD_PROG_LEN / 4)
    {
        printf("    [WARNING] Multi-word program interrupted at 0x%x !!\n", i);
        return -1;
    }

    u32TimeOutCnt = FMC_TIMEOUT_WRITE;
    while(FMC->MPSTS & FMC_MPSTS_MPBUSY_Msk)
    {
        if(--u32TimeOutCnt == 0)
        {
            printf("Time-out occurred on waiting MPBUSY cleared!\n");
            return -1;
        }
    }

    return 0;
}

int main()
{
    uint32_t  i, addr, maddr, done = 0; /* temporary variables */

    SYS_UnlockReg();                   /* Unlock protected registers */

    SYS_Init();                        /* Init System, IP clock and multi-function I/O */

    UART0_Init();                      /* Initialize UART0 */

    printf("\n\n");
    printf("+-------------------------------------+\n");
    printf("|    M460 Multi-word Program Sample   |\n");
    printf("+-------------------------------------+\n");

    FMC_Open();                        /* Enable FMC ISP function */

    FMC_ENABLE_AP_UPDATE();            /* Enable APROM erase/program */

    for(addr = 0x4000; addr < 0x20000; addr += FMC_FLASH_PAGE_SIZE)
    {
        printf("Multiword program APROM page 0x%x =>\n", addr);

        if(FMC_Erase(addr) < 0)
        {
            printf("    Erase failed!!\n");
            goto err_out;
        }

        printf("    Program...\n");

        for(maddr = addr; maddr < addr + FMC_FLASH_PAGE_SIZE; maddr += FMC_MULTI_WORD_PROG_LEN)
        {
            /* Prepare test pattern */
            for(i = 0; i < FMC_MULTI_WORD_PROG_LEN; i += 4)
                page_buff[i / 4] = maddr + i;

#ifdef USE_DRIVER_API
            i = FMC_WriteMultiple(maddr, page_buff, FMC_MULTI_WORD_PROG_LEN);
            if(i <= 0)
            {
                printf("FMC_WriteMultiple failed: %d\n", i);
                goto err_out;
            }
            printf("programmed length = %d\n", i);
#else
            /* execute multi-word program */
            if(multi_word_program(maddr) < 0)
                goto err_out;
#endif
        }
        printf("    [OK]\n");

        printf("    Verify...");

        for(i = 0; i < FMC_FLASH_PAGE_SIZE; i += 4)
            page_buff[i / 4] = addr + i;

        for(i = 0; i < FMC_FLASH_PAGE_SIZE; i += 4)
        {
            if(FMC_Read(addr + i) != page_buff[i / 4])
            {
                printf("\n[FAILED] Data mismatch at address 0x%x, expect: 0x%x, read: 0x%x!\n", addr + i, page_buff[i / 4], FMC_Read(addr + i));
                goto err_out;
            }
            if(g_FMC_i32ErrCode != 0)
            {
                printf("FMC_Read address 0x%x failed!\n", addr + i);
                goto err_out;
            }
        }
        printf("[OK]\n");
    }

    printf("\n\nMulti-word program demo done.\n");
    done = 1;

err_out:

    if(done)
    {
        printf("\n\nMulti-word program demo done.\n");
    }
    else
    {
        printf("\n\nERROR!\n");
    }
    while(1);
}
