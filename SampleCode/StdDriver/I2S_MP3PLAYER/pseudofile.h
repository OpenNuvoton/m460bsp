/**************************************************************************//**
 * @file     pseudofile.h
 * @version  V3.00
 * @brief    Pseudo file system implementation.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef PSEUDO_FFILE_H
#define PSEUDO_FFILE_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* File access mode and open method flags (3rd argument of f_open) */
#define	FA_READ				0x01
#define	FA_WRITE			0x02
#define	FA_OPEN_EXISTING	0x00
#define	FA_CREATE_NEW		0x04
#define	FA_CREATE_ALWAYS	0x08
#define	FA_OPEN_ALWAYS		0x10
#define	FA_OPEN_APPEND		0x30


/* ====== Type definitions ====== */
typedef enum {
    FR_OK = 0,          /* Succeeded */
    FR_DISK_ERR,        /* Low-level error */
    FR_INT_ERR,         /* Assertion failed */
    FR_NOT_READY,       /* Device not ready */
    FR_NO_FILE,         /* File not found */
    FR_DENIED,          /* Access denied or file full */
    FR_INVALID_OBJECT,  /* Invalid file object */
    FR_INVALID_PARAMETER
} FRESULT;

typedef unsigned int   UINT;     /* FatFs uses UINT for counts */
typedef uint32_t       FSIZE_t;  /* File size type */

/* Our in-memory file object */
typedef struct {
    const uint8_t *data; /* Pointer to backing storage */
    size_t size;         /* File size */
    size_t pos;          /* Current file pointer */
    int writable;        /* Writable flag */
    int eof;             /* End-of-file flag */
} FIL;

/* ====== API ====== */

/* Open or create a file (here always opens same fixed array) */
FRESULT f_open (FIL* fp, const char* path, uint8_t mode);

/* Close an open file object */
FRESULT f_close (FIL* fp);

/* Read data from the file */
FRESULT f_read (FIL* fp, void* buff, UINT btr, UINT* br);

/* Write data to the file */
FRESULT f_write (FIL* fp, const void* buff, UINT btw, UINT* bw);

/* Move file pointer of the file object */
FRESULT f_lseek (FIL* fp, FSIZE_t ofs);

/* Test end-of-file */
int f_eof (FIL* fp);

#ifdef __cplusplus
}
#endif

#endif /* PSEUDO_FFILE_H */
