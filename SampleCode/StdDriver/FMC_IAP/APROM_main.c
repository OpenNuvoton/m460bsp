/**************************************************************************//**
 * @file     APROM_main.c
 * @version  V1.00
 * @brief    FMC APROM IAP sample program run on APROM.
 *
 *
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>

#include "NuMicro.h"


extern uint32_t  loaderImage1Base, loaderImage1Limit;   /* symbol of image start and end */


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

/**
  * @brief    Load an image to specified flash address. The flash area must have been enabled by
  *           caller. For example, if caller want to program an image to LDROM, FMC_ENABLE_LD_UPDATE()
  *           must be called prior to calling this function.
  * @return   Image is successfully programmed or not.
  * @retval   0   Success.
  * @retval   -1  Program/verify failed.
  */
static int  load_image_to_flash(uint32_t image_base, uint32_t image_limit, uint32_t flash_addr, uint32_t max_size)
{
    uint32_t   i, j, u32Data, u32ImageSize, *pu32Loader;

    u32ImageSize = max_size;           /* Give the maximum size of programmable flash area. */

    printf("Program image to flash address 0x%x...", flash_addr);    /* information message */

    /*
     * program the whole image to specified flash area
     */
    pu32Loader = (uint32_t *)image_base;
    for (i = 0; i < u32ImageSize; i += FMC_FLASH_PAGE_SIZE)
    {

        FMC_Erase(flash_addr + i);     /* erase a flash page */
        for (j = 0; j < FMC_FLASH_PAGE_SIZE; j += 4)                 /* program image to this flash page */
        {
            FMC_Write(flash_addr + i + j, pu32Loader[(i + j) / 4]);
        }
    }
    printf("OK.\nVerify ...");

    /* Verify loader */
    for (i = 0; i < u32ImageSize; i += FMC_FLASH_PAGE_SIZE)
    {
        for (j = 0; j < FMC_FLASH_PAGE_SIZE; j += 4)
        {
            u32Data = FMC_Read(flash_addr + i + j);        /* read a word from flash memory */

            if (u32Data != pu32Loader[(i+j)/4])            /* check if the word read from flash be matched with original image */
            {
                printf("data mismatch on 0x%x, [0x%x], [0x%x]\n", flash_addr + i + j, u32Data, pu32Loader[(i+j)/4]);
                return -1;             /* image program failed */
            }

            if (i + j >= u32ImageSize) /* check if it reach the end of image */
                break;
        }
    }
    printf("OK.\n");
    return 0;                          /* success */
}


int main()
{
    uint8_t     u8Item;                /* menu item */
    uint32_t    u32Data;               /* temporary data word */
    uint32_t    u32TimeOutCnt;         /* time-out counter */

    SYS_UnlockReg();                   /* Unlock register lock protect */

    SYS_Init();                        /* Init System, IP clock and multi-function I/O */

    SCB->VTOR = FMC_APROM_BASE;        /* Set Vector Table Offset Register */

    UART0_Init();                      /* Initialize UART0 */

    printf("\n\n");
    printf("+----------------------------------------+\n");
    printf("|     M460 FMC_IAP Sample Code           |\n");
    printf("|           [APROM code]                 |\n");
    printf("+----------------------------------------+\n");

    FMC_Open();                        /* Enable FMC ISP function */

    /* Read BS */
    printf("  Boot Mode ............................. ");
    if (FMC_GetBootSource() == 0)      /* Get boot source */
        printf("[APROM]\n");           /* Is booting from APROM */
    else
    {
        printf("[LDROM]\n");           /* Is booting from LDROM */
        printf("  WARNING: The sample code must execute in APROM!\n");
        goto lexit;                    /* This sample program must execute in APROM. Program aborted. */
    }

    u32Data = FMC_ReadCID();           /* get company ID */
    printf("  Company ID ............................ [0x%08x]\n", u32Data);

    u32Data = FMC_ReadPID();           /* get product ID */
    printf("  Product ID ............................ [0x%08x]\n", u32Data);

    /* Read User Configuration CONFIG0 */
    printf("  User Config 0 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE));
    /* Read User Configuration CONFIG1 */
    printf("  User Config 1 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE+4));

    do
    {
        printf("\n\n\n");
        printf("+----------------------------------------+\n");
        printf("|               Select                   |\n");
        printf("+----------------------------------------+\n");
        printf("| [0] Load IAP code to LDROM             |\n");
        printf("| [1] Run IAP program (in LDROM)         |\n");
        printf("+----------------------------------------+\n");
        printf("Please select...");
        u8Item = getchar();            /* block waiting to receive any one character from UART0 */
        printf("%c\n", u8Item);        /* print out the selected item */

        switch (u8Item)
        {
        case '0':
            FMC_ENABLE_LD_UPDATE();    /* Enable LDROM update capability */
            /*
             *  The binary image of LDROM code is embedded in this sample.
             *  load_image_to_flash() will program this LDROM code to LDROM.
             */
            if (load_image_to_flash((uint32_t)&loaderImage1Base, (uint32_t)&loaderImage1Limit,
                                    FMC_LDROM_BASE, FMC_LDROM_SIZE) != 0)
            {
                printf("Load image to LDROM failed!\n");
                goto lexit;            /* Load LDROM code failed. Program aborted. */
            }
            FMC_DISABLE_LD_UPDATE();   /* Disable LDROM update capability */
            break;

        case '1':
            printf("\n\nChange VECMAP and branch to LDROM...\n");
            printf("LDROM code SP = 0x%x\n", *(uint32_t *)(FMC_LDROM_BASE));
            printf("LDROM code ResetHandler = 0x%x\n", *(uint32_t *)(FMC_LDROM_BASE+4));
            u32TimeOutCnt = SystemCoreClock; /* 1 second time-out */
            while (!(UART0->FIFOSTS & UART_FIFOSTS_TXEMPTY_Msk))       /* Wait for UART0 TX FIFO cleared */
                if(--u32TimeOutCnt == 0) break;

            /*  NOTE!
             *     Before change VECMAP, user MUST disable all interrupts.
             *     The following code CANNOT locate in address 0x0 ~ 0x200.
             */

            FMC_SetVectorPageAddr(FMC_LDROM_BASE);
            if (g_FMC_i32ErrCode != 0)
            {
                printf("FMC_SetVectorPageAddr(FMC_LDROM_BASE) failed!\n");
                goto lexit;
            }

            /* Software reset to boot to LDROM */
            NVIC_SystemReset();

            break;

        default :
            continue;                  /* invalid selection */
        }
    }
    while (1);


lexit:                                 /* program exit */

    FMC_Close();                       /* Disable FMC ISP function */

    SYS_LockReg();                     /* Lock protected registers */

    printf("\nFMC Sample Code Completed.\n");

    while (1);
}

/*** (C) COPYRIGHT 2021 Nuvoton Technology Corp. ***/
