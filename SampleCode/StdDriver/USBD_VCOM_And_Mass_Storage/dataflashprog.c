/**************************************************************************//**
 * @file     dataflashprog.c
 * @version  V3.00
 * @brief    Data Flash programming sample file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/


/*---------------------------------------------------------------------------------------------------------*/
/* Includes of system headers                                                                              */
/*---------------------------------------------------------------------------------------------------------*/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"
#include "dataflashprog.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Macro, type and constant definitions                                                                    */
/*---------------------------------------------------------------------------------------------------------*/

uint32_t g_sectorBuf[FLASH_PAGE_SIZE / 4];

void DataFlashRead(uint32_t u32Addr, uint32_t u32Size, uint32_t u32Buffer)
{
    /* This is low level read function of USB Mass Storage */
    int32_t len;
    uint32_t i;
    uint32_t * pu32Buf = (uint32_t *)u32Buffer;

    /* Modify the address to MASS_STORAGE_OFFSET */
    u32Addr += MASS_STORAGE_OFFSET;

    len = (int32_t)u32Size;

    while(len >= BUFFER_PAGE_SIZE)
    {
        //FMC_ReadPage(u32Addr, (uint32_t *)u32Buffer);
        for(i = 0; i < BUFFER_PAGE_SIZE / 4; i++)
            pu32Buf[i] = FMC_Read(u32Addr + i * 4);
        u32Addr   += BUFFER_PAGE_SIZE;
        u32Buffer += BUFFER_PAGE_SIZE;
        len  -= BUFFER_PAGE_SIZE;
        pu32Buf = (uint32_t *)u32Buffer;
    }
}

void DataFlashReadPage(uint32_t u32Addr, uint32_t u32Buffer)
{
    uint32_t i;
    uint32_t * pu32Buf = (uint32_t *)u32Buffer;

    /* Modify the address to MASS_STORAGE_OFFSET */
    u32Addr += MASS_STORAGE_OFFSET;

    for(i = 0; i < FLASH_PAGE_SIZE / 4; i++)
        pu32Buf[i] = FMC_Read(u32Addr + i * 4);
}

uint32_t DataFlashProgramPage(uint32_t u32StartAddr, uint32_t * u32Buf)
{
    uint32_t i;

    for(i = 0; i < FLASH_PAGE_SIZE / 4; i++)
    {
        FMC_Write(u32StartAddr + i * 4, u32Buf[i]);
    }

    return 0;
}


void DataFlashWrite(uint32_t u32Addr, uint32_t u32Size, uint32_t u32Buffer)
{
    /* This is low level write function of USB Mass Storage */
    int32_t len, i, offset;
    uint32_t *pu32;
    uint32_t alignAddr;

    /* Modify the address to MASS_STORAGE_OFFSET */
    u32Addr += MASS_STORAGE_OFFSET;

    len = (int32_t)u32Size;

    if((len == FLASH_PAGE_SIZE) && ((u32Addr & (FLASH_PAGE_SIZE - 1)) == 0))
    {
        /* Page erase */
        FMC_Erase(u32Addr);

        while(len >= FLASH_PAGE_SIZE)
        {
            DataFlashProgramPage(u32Addr, (uint32_t *) u32Buffer);
            len    -= FLASH_PAGE_SIZE;
            u32Buffer += FLASH_PAGE_SIZE;
            u32Addr   += FLASH_PAGE_SIZE;
        }
    }
    else
    {
        do
        {
            alignAddr = u32Addr & 0xFFF000;

            /* Get the sector offset*/
            offset = (u32Addr & (FLASH_PAGE_SIZE - 1));

            if(offset || (u32Size < FLASH_PAGE_SIZE))
            {
                /* Not 4096-byte alignment. Read the destination page for modification. Note: It needs to avoid adding MASS_STORAGE_OFFSET twice. */
                DataFlashReadPage(alignAddr - MASS_STORAGE_OFFSET, /*FLASH_PAGE_SIZE,*/ (uint32_t)&g_sectorBuf[0]);

            }

            /* Source u32Buffer */
            pu32 = (uint32_t *)u32Buffer;
            /* Get the update length */
            len = FLASH_PAGE_SIZE - offset;
            if(u32Size < len)
                len = u32Size;
            /* Update the destination u32Buffer */
            for(i = 0; i < len / 4; i++)
            {
                g_sectorBuf[offset / 4 + i] = pu32[i];
            }

            /* Page erase */
            FMC_Erase(alignAddr);
            /* Write to the destination page */
            DataFlashProgramPage(alignAddr, (uint32_t *) g_sectorBuf);

            u32Size -= len;
            u32Addr += len;
            u32Buffer += len;

        }
        while(u32Size > 0);
    }
}
