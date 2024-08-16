;/**************************************************************************//**
; * @file     ap_image.S
; * @version  V1.00
; * @brief    Assembly code include AP image.
; *
; * SPDX-License-Identifier: Apache-2.0
; * @copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
;*****************************************************************************/
    .syntax unified
    .arch   armv7-m

    .globl  loaderImage1Base
    .globl  loaderImage1Limit

    .align  4

    .text

loaderImage1Base:
#ifndef BIN_FILE
# ifndef _ARM_VSCODE_
    .incbin  "../../LDROM_iap.bin"
# else
    .incbin   "./LDROM_iap.bin"
# endif
#else
    .incbin   BIN_FILE
#endif // #ifndef BIN_FILE

loaderImage1Limit:
    .space   4

    .end
