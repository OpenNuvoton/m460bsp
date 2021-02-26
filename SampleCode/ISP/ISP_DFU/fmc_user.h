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

int FMC_Read_User(unsigned int u32Addr, unsigned int *data);
void ReadData(unsigned int addr_start, unsigned int addr_end, unsigned int *data);
void WriteData(unsigned int addr_start, unsigned int addr_end, unsigned int *data);

#endif  /* FMC_USER_H */
