/***************************************************************************//**
 * @file     fmc_user.c
 * @brief    FMC driver source file
 * @version  2.0.0
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <stdio.h>
#include "NuMicro.h"
#include "fmc_user.h"

int FMC_Proc(uint32_t u32Cmd, uint32_t addr_start, uint32_t addr_end, uint32_t *data);

int FMC_Proc(uint32_t u32Cmd, uint32_t addr_start, uint32_t addr_end, uint32_t *data)
{
    uint32_t u32Addr, Reg;
    uint32_t u32TimeOutCount = SystemCoreClock;

    for(u32Addr = addr_start; u32Addr < addr_end; data++)
    {
        FMC->ISPCMD = u32Cmd;
        FMC->ISPADDR = u32Addr;

        if(u32Cmd == FMC_ISPCMD_PROGRAM)
        {
            FMC->ISPDAT = *data;
        }

        FMC->ISPTRG = 0x1;
        __ISB();

        /* Wait ISP cmd complete */
        while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
        {
            if(--u32TimeOutCount == 0) /* 1 second time-out */
                return -1;
        }

        Reg = FMC->ISPCTL;

        if(Reg & FMC_ISPCTL_ISPFF_Msk)
        {
            FMC->ISPCTL = Reg;
            return -1;
        }

        if(u32Cmd == FMC_ISPCMD_READ)
        {
            *data = FMC->ISPDAT;
        }

        if(u32Cmd == FMC_ISPCMD_PAGE_ERASE)
        {
            u32Addr += FMC_FLASH_PAGE_SIZE;
        }
        else
        {
            u32Addr += 4;
        }
    }

    return 0;
}

/**
 * @brief      Program 32-bit data into specified address of flash
 *
 * @param[in]  u32addr  Flash address include APROM, LDROM, Data Flash, and CONFIG
 * @param[in]  u32data  32-bit Data to program
 *
 * @details    To program word data into Flash include APROM, LDROM, Data Flash, and CONFIG.
 *             The corresponding functions in CONFIG are listed in FMC section of TRM.
 *
 * @note
 *             Please make sure that Register Write-Protection Function has been disabled
 *             before using this function.
 */
int FMC_Write_User(uint32_t u32Addr, uint32_t u32Data)
{
    return FMC_Proc(FMC_ISPCMD_PROGRAM, u32Addr, u32Addr + 4, &u32Data);
}

/**
 * @brief       Read 32-bit Data from specified address of flash
 *
 * @param[in]   u32addr  Flash address include APROM, LDROM, Data Flash, and CONFIG
 *
 * @return      The data of specified address
 *
 * @details     To read word data from Flash include APROM, LDROM, Data Flash, and CONFIG.
 *
 * @note
 *              Please make sure that Register Write-Protection Function has been disabled
 *              before using this function.
 */
int FMC_Read_User(uint32_t u32Addr, uint32_t *data)
{
    return FMC_Proc(FMC_ISPCMD_READ, u32Addr, u32Addr + 4, data);
}

/**
 * @brief      Flash page erase
 *
 * @param[in]  u32addr  Flash address including APROM, LDROM, Data Flash, and CONFIG
 *
 * @details    To do flash page erase. The target address could be APROM, LDROM, Data Flash, or CONFIG.
 *             The page size is 512 bytes.
 *
 * @note
 *             Please make sure that Register Write-Protection Function has been disabled
 *             before using this function.
 */
int FMC_Erase_User(uint32_t u32Addr)
{
    return FMC_Proc(FMC_ISPCMD_PAGE_ERASE, u32Addr, u32Addr + 4, 0);
}

void ReadData(uint32_t addr_start, uint32_t addr_end, uint32_t *data)    // Read data from flash
{
    FMC_Proc(FMC_ISPCMD_READ, addr_start, addr_end, data);
    return;
}

void WriteData(uint32_t addr_start, uint32_t addr_end, uint32_t *data)  // Write data into flash
{
    FMC_Proc(FMC_ISPCMD_PROGRAM, addr_start, addr_end, data);
    return;
}


int EraseAP(uint32_t addr_start, uint32_t size)
{
    uint32_t u32Addr, u32Cmd, u32Size;
    int32_t i32Size;
    uint32_t u32TimeOutCount = FMC_TIMEOUT_ERASE;

    u32Addr = addr_start;
    i32Size = (int32_t)size;

    while(i32Size > 0)
    {
        if((size >= FMC_BANK_SIZE) && !(u32Addr & (FMC_BANK_SIZE - 1)))
        {
            u32Cmd = FMC_ISPCMD_BANK_ERASE;
            u32Size = FMC_BANK_SIZE;
        }
        else
        {
            u32Cmd = FMC_ISPCMD_PAGE_ERASE;
            u32Size = FMC_FLASH_PAGE_SIZE;
        }

        FMC->ISPCMD = u32Cmd;
        FMC->ISPADDR = u32Addr;
        FMC->ISPTRG = FMC_ISPTRG_ISPGO_Msk;
        __ISB();

        while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)    /* Wait for ISP command done. */
        {
            if(--u32TimeOutCount == 0)
                return -1;
        }

        if(FMC->ISPCTL & FMC_ISPCTL_ISPFF_Msk)
        {
            FMC->ISPCTL |= FMC_ISPCTL_ISPFF_Msk;
            return -1;
        }

        u32Addr += u32Size;
        size -= u32Size;
        i32Size = (int32_t)size;
    }

    return 0;
}

void UpdateConfig(uint32_t *data, uint32_t *res)
{
    uint32_t u32Size = 16;
    FMC_ENABLE_CFG_UPDATE();
    FMC_Proc(FMC_ISPCMD_PAGE_ERASE, Config0, Config0 + 8, 0);
    FMC_Proc(FMC_ISPCMD_PROGRAM, Config0, Config0 + u32Size, data);

    if(res)
    {
        FMC_Proc(FMC_ISPCMD_READ, Config0, Config0 + u32Size, res);
    }

    FMC_DISABLE_CFG_UPDATE();
}
