/***************************************************************************//**
 * @file     targetdev.c
 * @brief    ISP support function source file
 * @version  0x32
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "targetdev.h"
#include "isp_user.h"


/* Supports maximum 1M (APROM) */
uint32_t GetApromSize()
{
    /* The smallest of APROM size is 2K. */
    uint32_t size = 0x800, data;
    int result;

    do
    {
        result = FMC_Read_User(size, &data);

        if(result < 0)
        {
            return size;
        }
        else
        {
            size *= 2;
        }
    }
    while(1);
}

/* Data Flash is shared with APROM.
   The size and start address are defined in CONFIG1. */
void GetDataFlashInfo(uint32_t *pu32Addr, uint32_t *pu32Size)
{
    uint32_t u32Data;
    *pu32Size = 0;
    FMC_Read_User(Config0, &u32Data);

    if((u32Data & 0x01) == 0)    /* DFEN enable */
    {
        FMC_Read_User(Config1, &u32Data);

        /* Filter the reserved bits in CONFIG1 */
        u32Data &= 0x000FFFFF;

        if(u32Data > g_u32ApromSize || u32Data & (FMC_FLASH_PAGE_SIZE - 1))    /* Avoid config1 value from error */
        {
            u32Data = g_u32ApromSize;
        }

        *pu32Addr = u32Data;
        *pu32Size = g_u32ApromSize - u32Data;
    }
    else
    {
        *pu32Addr = g_u32ApromSize;
        *pu32Size = 0;
    }
}
