/****************************************************************************//**
 * @file     exeinsram.c
 * @version  V0.10
 * @brief    Implement a code to execute in HyperRAM.
 *
 * @copyright (C) 2022 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/
#include <stdio.h>
#include <string.h>
#include "NuMicro.h"


int32_t ExeInHRAM(void)
{

    printf("\nNow execute in HyperRAM @ 0x%08x\n", (uint32_t)ExeInHRAM);
    printf("\nAny key to continue\n");
    getchar();

    return 0;
}


