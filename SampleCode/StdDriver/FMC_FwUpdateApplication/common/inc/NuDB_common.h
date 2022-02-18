/**************************************************************************//**
 * @file     NuDB_common.h
 * @version  V0.00
 * @brief    NuDB common header file
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2020 Nuvoton Technology Corp. All rights reserved.
 *****************************************************************************/
#ifndef __NUDB_COMMON_H__
#define __NUDB_COMMON_H__

#ifdef __cplusplus
extern "C"
{
#endif


#define LOADER_BASE 0x0
#define LOADER_SIZE 0x8000
#define APP_BASE    0x8000
#define APP_SIZE    0x8000

#define FW_CRC_BASE        0x7F800
#define NEW_FW_CRC_BASE    0x7F804
#define BACKUP_FW_CRC_BASE 0x7F808


#ifdef __cplusplus
}
#endif

#endif /* __NUDB_COMMON_H__ */

/*** (C) COPYRIGHT 2020 Nuvoton Technology Corp. ***/
