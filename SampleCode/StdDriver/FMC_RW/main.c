/**************************************************************************//**
 * @file     main.c
 * @version  V1.00
 * @brief    Show FMC read Flash IDs, erase, read, and write function
 *
 *
 * @copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>

#include "NuMicro.h"


#define APROM_TEST_BASE             0x20000      /* APROM test start address                 */
#define DATA_FLASH_TEST_BASE        0x30000      /* Data Flash test start address            */
#define DATA_FLASH_TEST_END         0x40000      /* Data Flash test end address              */
#define TEST_PATTERN                0x5A5A5A5A   /* Test pattern                             */


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

static int  set_data_flash_base(uint32_t u32DFBA)
{
    uint32_t   au32Config[2];          /* User Configuration */

    /* Read User Configuration 0 & 1 */
    if (FMC_ReadConfig(au32Config, 2) < 0)
    {
        printf("\nRead User Config failed!\n");       /* Error message */
        return -1;                     /* failed to read User Configuration */
    }

    /* Check if Data Flash is enabled and is expected address. */
    if ((!(au32Config[0] & 0x1)) && (au32Config[1] == u32DFBA))
        return 0;                      /* no need to modify User Configuration */

    FMC_ENABLE_CFG_UPDATE();           /* Enable User Configuration update. */

    au32Config[0] &= ~0x1;             /* Clear CONFIG0 bit 0 to enable Data Flash */
    au32Config[1] = u32DFBA;           /* Give Data Flash base address  */

    /* Update User Configuration settings. */
    if (FMC_WriteConfig(au32Config, 2) < 0)
        return -1;                     /* failed to write user configuration */

    printf("\nSet Data Flash base as 0x%x.\n", DATA_FLASH_TEST_BASE);  /* debug message */

    /* Perform chip reset to make new User Config take effect. */
    SYS->IPRST0 = SYS_IPRST0_CHIPRST_Msk;
    return 0;                          /* success */
}


int32_t fill_data_pattern(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t u32Addr;                  /* flash address */

    /* Fill flash range from u32StartAddr to u32EndAddr with data word u32Pattern. */
    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        if (FMC_Write(u32Addr, u32Pattern) != 0)          /* Program flash */
        {
            printf("FMC_Write address 0x%x failed!\n", u32Addr);
            return -1;
        }
    }
    return 0;
}


int32_t  verify_data(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t    u32Addr;               /* flash address */
    uint32_t    u32data;               /* flash data    */

    /* Verify if each data word from flash u32StartAddr to u32EndAddr be u32Pattern.  */
    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += 4)
    {
        u32data = FMC_Read(u32Addr);   /* Read a flash word from address u32Addr. */

        if (g_FMC_i32ErrCode != 0)
        {
            printf("FMC_Read address 0x%x failed!\n", u32Addr);
            return -1;
        }

        if (u32data != u32Pattern)     /* Verify if data matched. */
        {
            printf("\nFMC_Read data verify failed at address 0x%x, read=0x%x, expect=0x%x\n", u32Addr, u32data, u32Pattern);
            return -1;                 /* data verify failed */
        }
    }
    return 0;                          /* data verify OK */
}


int32_t  flash_test(uint32_t u32StartAddr, uint32_t u32EndAddr, uint32_t u32Pattern)
{
    uint32_t    u32Addr;               /* flash address */

    for (u32Addr = u32StartAddr; u32Addr < u32EndAddr; u32Addr += FMC_FLASH_PAGE_SIZE)
    {
        printf("    Flash test address: 0x%x    \r", u32Addr);       /* information message */

        if (FMC_Erase(u32Addr) != 0)            /* Erase page */
        {
            printf("FMC_Erase address 0x%x failed!\n", u32Addr);
            return -1;
        }

        /* Verify if page contents are all 0xFFFFFFFF */
        if (verify_data(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, 0xFFFFFFFF) < 0)
        {
            printf("\nPage 0x%x erase verify failed!\n", u32Addr);   /* error message */
            return -1;                 /* Erase verify failed */
        }

        /* Write test pattern to fill the whole page. */
        if (fill_data_pattern(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, u32Pattern) != 0)
            return -1;

        /* Verify if page contents are all equal to test pattern */
        if (verify_data(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, u32Pattern) < 0)
        {
            printf("\nData verify failed!\n ");                      /* error message */
            return -1;                 /* Program verify failed */
        }

        /* Erase page */
        if (FMC_Erase(u32Addr) != 0)
        {
            printf("FMC_Erase address 0x%x failed!\n", u32Addr);
            return -1;
        }

        /* Verify if page contents are all 0xFFFFFFFF after erased. */
        if (verify_data(u32Addr, u32Addr + FMC_FLASH_PAGE_SIZE, 0xFFFFFFFF) < 0)
        {
            printf("\nPage 0x%x erase verify failed!\n", u32Addr);   /* error message */
            return -1;                 /* Erase verify failed */
        }
    }
    printf("\r    Flash Test Passed.          \n");                  /* information message */
    return 0;      /* flash test passed */
}


int main()
{
    uint32_t    i, u32Data;            /* variables */

    SYS_UnlockReg();                   /* Unlock register lock protect */

    SYS_Init();                        /* Init System, IP clock and multi-function I/O */

    UART0_Init();                      /* Initialize UART0 */

    printf("+------------------------------------------+\n");
    printf("|    M460 FMC Read/Write Sample Demo       |\n");
    printf("+------------------------------------------+\n");

    FMC_Open();                        /* Enable FMC ISP function */

    /* Enable Data Flash and set base address. */
    if (set_data_flash_base(DATA_FLASH_TEST_BASE) < 0)
    {
        printf("Failed to set Data Flash base address!\n");          /* error message */
        goto lexit;                    /* failed to configure Data Flash, aborted */
    }

    /* Get booting source (APROM/LDROM) */
    printf("  Boot Mode ............................. ");
    if (FMC_GetBootSource() == 0)
        printf("[APROM]\n");           /* debug message */
    else
    {
        printf("[LDROM]\n");           /* debug message */
        printf("  WARNING: The driver sample code must execute in AP mode!\n");
        goto lexit;                    /* failed to get boot source */
    }

    u32Data = FMC_ReadCID();           /* Get company ID, should be 0xDA. */
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_ReadCID failed!\n");
        goto lexit;
    }
    printf("  Company ID ............................ [0x%08x]\n", u32Data);   /* information message */

    u32Data = FMC_ReadPID();           /* Get product ID. */
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_ReadPID failed!\n");
        goto lexit;
    }
    printf("  Product ID ............................ [0x%08x]\n", u32Data);   /* information message */

    for (i = 0; i < 3; i++)            /* Get 96-bits UID. */
    {
        u32Data = FMC_ReadUID(i);
        if (g_FMC_i32ErrCode != 0)
        {
            printf("FMC_ReadUID %d failed!\n", i);
            goto lexit;
        }
        printf("  Unique ID %d ........................... [0x%08x]\n", i, u32Data);  /* information message */
    }

    for (i = 0; i < 4; i++)            /* Get 4 words UCID. */
    {
        u32Data = FMC_ReadUCID(i);
        if (g_FMC_i32ErrCode != 0)
        {
            printf("FMC_ReadUCID %d failed!\n", i);
            goto lexit;
        }
        printf("  Unique Customer ID %d .................. [0x%08x]\n", i, u32Data);  /* information message */
    }

    /* Read User Configuration CONFIG0 */
    printf("  User Config 0 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE));
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_Read(FMC_CONFIG_BASE) failed!\n");
        goto lexit;
    }

    /* Read User Configuration CONFIG1 */
    printf("  User Config 1 ......................... [0x%08x]\n", FMC_Read(FMC_CONFIG_BASE+4));
    if (g_FMC_i32ErrCode != 0)
    {
        printf("FMC_Read(FMC_CONFIG_BASE+4) failed!\n");
        goto lexit;
    }

    /* Read Data Flash base address */
    u32Data = FMC_ReadDataFlashBaseAddr();
    printf("  Data Flash Base Address ............... [0x%08x]\n", u32Data);   /* information message */

    printf("\n\nLDROM test =>\n");     /* information message */

    FMC_ENABLE_LD_UPDATE();            /* Enable LDROM update. */
    /* Execute flash program/verify test on LDROM. */
    if (flash_test(FMC_LDROM_BASE, FMC_LDROM_END, TEST_PATTERN) < 0)
    {
        printf("\n\nLDROM test failed!\n");        /* error message */
        goto lexit;                    /* LDROM test failed. Program aborted. */
    }

    FMC_DISABLE_LD_UPDATE();           /* Disable LDROM update. */

    printf("\n\nAPROM test =>\n");     /* information message */

    FMC_ENABLE_AP_UPDATE();            /* Enable APROM update. */

    /* Execute flash program/verify test on APROM. */
    if (flash_test(APROM_TEST_BASE, DATA_FLASH_TEST_BASE, TEST_PATTERN) < 0)
    {
        printf("\n\nAPROM test failed!\n");        /* debug message */
        goto lexit;                                /* APROM test failed. Program aborted. */
    }

    printf("\n\nData Flash test =>\n");            /* information message */

    /* Execute flash program/verify test on Data Flash. */
    if (flash_test(DATA_FLASH_TEST_BASE, DATA_FLASH_TEST_END, TEST_PATTERN) < 0)
    {
        printf("\n\nData flash read/write test failed!\n");
        goto lexit;                    /* flash test failed */
    }

    FMC_DISABLE_AP_UPDATE();           /* Disable APROM update. */

lexit:

    FMC_Close();                       /* Disable FMC ISP function */

    SYS_LockReg();                     /* Lock protected registers */

    printf("\nFMC Sample Code Completed.\n");

    while (1);
}

/*** (C) COPYRIGHT 2021 Nuvoton Technology Corp. ***/
