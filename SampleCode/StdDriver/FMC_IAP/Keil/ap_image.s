;/**************************************************************************//**
; * @file     ap_image.S
; * @version  V1.00
; * @brief    Assembly code include AP image.
; *
; *
; * @copyright (C) 2024 Nuvoton Technology Corp. All rights reserved.
;*****************************************************************************/
    .syntax unified
    .arch armv7-m


    .global  loaderImage1Base
    .global  loaderImage1Limit

    .align   4

    .text

loaderImage1Base:
#ifndef BIN_FILE
    .incbin "./fmc_ld_code.bin"
#else
    .incbin BIN_FILE
#endif
loaderImage1Limit:
    .space  4
    .end