/**************************************************************************//**
 * @file     pseudofile.c
 * @version  V3.00
 * @brief    Pseudo file system implementation.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (c) 2025 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#include <string.h>
#include <stdlib.h>
#include "pseudofile.h"

/* ====== Fixed mp3 bit stream content ====== */
static const uint8_t gFileData[] = {
    #include "mp3.dat"
};
static const size_t gFileSize = sizeof(gFileData) - 1; // exclude '\0'


FRESULT f_open (FIL* fp, const char* path, uint8_t mode) {
    (void)path; // not used, single fixed file
    if (!fp) return FR_INVALID_PARAMETER;

    fp->data = gFileData;
    fp->size = gFileSize;
    fp->pos  = 0;
    fp->eof  = 0;
    fp->writable = (mode & FA_WRITE); // if LSB indicates writable
    return FR_OK;
}

FRESULT f_close (FIL* fp) {
    if (!fp) return FR_INVALID_PARAMETER;
    // Nothing to release, static backing
    return FR_OK;
}

FRESULT f_read (FIL* fp, void* buff, UINT btr, UINT* br) {
    if (!fp || !buff) return FR_INVALID_PARAMETER;
    if (fp->pos > fp->size) return FR_INVALID_OBJECT;

    size_t remain = fp->size - fp->pos;
    size_t ncopy  = (btr < remain) ? btr : remain;

    memcpy(buff, fp->data + fp->pos, ncopy);
    fp->pos += ncopy;
    fp->eof = (fp->pos >= fp->size);

    if (br) *br = (UINT)ncopy;
    return FR_OK;
}

FRESULT f_write (FIL* fp, const void* buff, UINT btw, UINT* bw) {
    if (!fp || !buff) return FR_INVALID_PARAMETER;
    if (!fp->writable) return FR_DENIED;
    if (fp->pos > fp->size) return FR_INVALID_OBJECT;

    size_t remain = fp->size - fp->pos;
    size_t ncopy  = (btw < remain) ? btw : remain;

    memcpy((uint8_t*)fp->data + fp->pos, buff, ncopy);
    fp->pos += ncopy;
    fp->eof = (fp->pos >= fp->size);

    if (bw) *bw = (UINT)ncopy;
    if (ncopy < btw) return FR_DENIED; // no space for full write
    return FR_OK;
}

FRESULT f_lseek (FIL* fp, FSIZE_t ofs) {
    if (!fp) return FR_INVALID_PARAMETER;
    if (ofs > fp->size) ofs = (FSIZE_t)fp->size; // clamp

    fp->pos = (size_t)ofs;
    fp->eof = (fp->pos >= fp->size);
    return FR_OK;
}

int f_eof (FIL* fp) {
    if (!fp) return 1;
    return fp->eof;
}