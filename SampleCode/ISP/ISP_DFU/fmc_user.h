/***************************************************************************//**
 * @file     fmc_user.h
 * @brief    FMC driver header file
 * @version  2.0.0
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef FMC_USER_H
#define FMC_USER_H

#include "m460.h"

/*---------------------------------------------------------------------------------------------------------*/
/* Define parameter                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define ISPGO           0x01

/*---------------------------------------------------------------------------------------------------------*/
/*  FMC Macro Definitions                                                                                  */
/*---------------------------------------------------------------------------------------------------------*/

int FMC_Read_User(uint32_t u32Addr, uint32_t *data);
void ReadData(uint32_t addr_start, uint32_t addr_end, uint32_t *data);
void WriteData(uint32_t addr_start, uint32_t addr_end, uint32_t *data);

#endif  /* FMC_USER_H */
