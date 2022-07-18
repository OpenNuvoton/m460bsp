/**
 * \file gcm.h
 *
 * \brief This file contains GCM definitions and functions.
 *
 * The Galois/Counter Mode (GCM) for 128-bit block ciphers is defined
 * in <em>D. McGrew, J. Viega, The Galois/Counter Mode of Operation
 * (GCM), Natl. Inst. Stand. Technol.</em>
 *
 * For more information on GCM, see <em>NIST SP 800-38D: Recommendation for
 * Block Cipher Modes of Operation: Galois/Counter Mode (GCM) and GMAC</em>.
 *
 */
/*
 *  Copyright The Mbed TLS Contributors
 *  SPDX-License-Identifier: Apache-2.0
 *
 *  Licensed under the Apache License, Version 2.0 (the "License"); you may
 *  not use this file except in compliance with the License.
 *  You may obtain a copy of the License at
 *
 *  http://www.apache.org/licenses/LICENSE-2.0
 *
 *  Unless required by applicable law or agreed to in writing, software
 *  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 *  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *  See the License for the specific language governing permissions and
 *  limitations under the License.
 */

#ifndef MBEDTLS_GCM_ALT_H
#define MBEDTLS_GCM_ALT_H

#if defined(MBEDTLS_GCM_ALT)

#define MAX_GCM_BUF      256
#define GCM_PBLOCK_SIZE  MAX_GCM_BUF     /* NOTE: This value must be 16 bytes alignment. This value must > size of A */


/**
 * \brief          The GCM context structure.
 */
typedef struct mbedtls_gcm_context
{
    mbedtls_cipher_context_t MBEDTLS_PRIVATE(cipher_ctx);  /*!< The cipher context used. */
    uint64_t MBEDTLS_PRIVATE(HL)[16];                      /*!< Precalculated HTable low. */
    uint64_t MBEDTLS_PRIVATE(HH)[16];                      /*!< Precalculated HTable high. */
    uint64_t MBEDTLS_PRIVATE(len);                         /*!< The total length of the encrypted data. */
    uint64_t MBEDTLS_PRIVATE(add_len);                     /*!< The total length of the additional data. */
    unsigned char MBEDTLS_PRIVATE(base_ectr)[16];          /*!< The first ECTR for tag. */
    unsigned char MBEDTLS_PRIVATE(y)[16];                  /*!< The Y working value. */
    unsigned char MBEDTLS_PRIVATE(buf)[16];                /*!< The buf working value. */
    int MBEDTLS_PRIVATE(mode);                             /*!< The operation to perform:
                                               #MBEDTLS_GCM_ENCRYPT or
                                               #MBEDTLS_GCM_DECRYPT. */


    uint32_t keySize;       /* Key size: 128/192/256 */
    uint32_t encDec;        /* 0: decrypt, 1: encrypt */
    uint32_t opMode;        /* AES_MODE_ECB/CBC/CFB */
    uint32_t iv[4];         /* IV for next block cipher */
    uint32_t keys[8];       /* Cipher key */
    uint32_t basicOpt;      /* Basic option of AES controller */
    uint8_t  gcm_buf[MAX_GCM_BUF]; /* buffer for GCM DMA input */
    uint8_t  out_buf[MAX_GCM_BUF+16]; /* buffer for GCM DMA output */
    uint8_t  fb_buf[72];    /* feedback buffer for GCM DMA */
    uint8_t  fb_buf2[72];   /* feedback buffer 2 for GCM DMA */
    uint8_t  tag[16];       /* Tag */
    uint32_t gcm_buf_bytes; /* Bytes in gcm_buf */
    uint32_t firstFlag;     /* A flag for the first data block */
    uint32_t endFlag;       /* final block is done */

    uint8_t *add;
    size_t addlen;


}
mbedtls_gcm_context;

#endif  /* MBEDTLS_GCM_ALT */

#endif /* gcm.h */
