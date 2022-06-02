/******************************************************************************
 * @file     xomlib.c
 * @version  V1.00
 * @brief    Function pointer for XOM APIs.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include "NuMicro.h"

#if defined(__ARMCC_VERSION)
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wunused-variable"
#pragma clang diagnostic ignored "-Wmissing-variable-declarations"
#endif


int32_t (*XOM_Add)(int32_t a, int32_t b) = (int32_t (*)(int32_t a, int32_t b))(0x00010001);
int32_t (*XOM_Div)(int32_t a, int32_t b) = (int32_t (*)(int32_t a, int32_t b))(0x00010015);
int32_t (*XOM_Mul)(int32_t a, int32_t b) = (int32_t (*)(int32_t a, int32_t b))(0x0001002d);
int32_t (*XOM_Sub)(int32_t a, int32_t b) = (int32_t (*)(int32_t a, int32_t b))(0x00010041);
int32_t (*XOM_Sum)(int32_t *pbuf, int32_t n) = (int32_t (*)(int32_t *pbuf, int32_t n))(0x00010055);

#if defined(__ARMCC_VERSION)
#pragma clang diagnostic pop
#endif
