;/**************************************************************************//**
; * @file     ap_image.S
; * @version  V1.00
; * @brief    Assembly code include AP image.
; *
; *
; * @copyright (C) 2016 Nuvoton Technology Corp. All rights reserved.
;*****************************************************************************/
    .syntax unified
    .arch   armv7-m

    .globl  loaderImage1Base
    .globl  loaderImage1Limit

    .align  4

    .text

loaderImage1Base:
#ifndef BIN_FILE
    .incbin  "../../LDROM_iap.bin"
#else
    .incbin   BIN_FILE
#endif
loaderImage1Limit:
    .space   4

    .end
