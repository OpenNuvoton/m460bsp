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

#define FMC_BLOCK_SIZE           (FMC_FLASH_PAGE_SIZE * 4UL)

int FMC_Proc(uint32_t u32Cmd, uint32_t addr_start, uint32_t addr_end, uint32_t *data);

int FMC_Proc(uint32_t u32Cmd, uint32_t addr_start, uint32_t addr_end, uint32_t *data)
{
    uint32_t u32Addr, Reg;
    uint32_t u32TimeOutCount;

    for(u32Addr = addr_start; u32Addr < addr_end; data++, u32Addr += 4)
    {
        FMC->ISPADDR = u32Addr;

        if((u32Addr & (FMC_FLASH_PAGE_SIZE - 1)) == 0 && u32Cmd == FMC_ISPCMD_PROGRAM)
        {
            FMC->ISPCMD = FMC_ISPCMD_PAGE_ERASE;
            FMC->ISPTRG = 0x1;

            u32TimeOutCount = SystemCoreClock; /* 1 second time-out */
            while(FMC->ISPTRG & 0x1)
            {
                if(--u32TimeOutCount == 0)
                    return -1;
            }
        }

        FMC->ISPCMD = u32Cmd;

        if(u32Cmd == FMC_ISPCMD_PROGRAM)
        {
            FMC->ISPDAT = *data;
        }

        FMC->ISPTRG = 0x1;
        //__ISB();

        /* Wait ISP cmd complete */
        u32TimeOutCount = SystemCoreClock; /* 1 second time-out */
        while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)
        {
            if(--u32TimeOutCount == 0)
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
    }

    return 0;
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
        else if((size >= FMC_BLOCK_SIZE) && !(u32Addr & (FMC_BLOCK_SIZE - 1)))
        {
            u32Cmd = FMC_ISPCMD_BLOCK_ERASE;
            u32Size = FMC_BLOCK_SIZE;
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

        while(FMC->ISPTRG & FMC_ISPTRG_ISPGO_Msk)   /* Wait for ISP command done. */
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
