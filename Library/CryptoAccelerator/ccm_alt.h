/**
 * \file ccm_alt.h
 *
 * \brief This file provides an API for the CCM authenticated encryption
 *        mode for block ciphers.
 *
 * CCM combines Counter mode encryption with CBC-MAC authentication
 * for 128-bit block ciphers.
 *
 * Input to CCM includes the following elements:
 * <ul><li>Payload - data that is both authenticated and encrypted.</li>
 * <li>Associated data (Adata) - data that is authenticated but not
 * encrypted, For example, a header.</li>
 * <li>Nonce - A unique value that is assigned to the payload and the
 * associated data.</li></ul>
 *
 * Definition of CCM:
 * http://csrc.nist.gov/publications/nistpubs/800-38C/SP800-38C_updated-July20_2007.pdf
 * RFC 3610 "Counter with CBC-MAC (CCM)"
 *
 * Related:
 * RFC 5116 "An Interface and Algorithms for Authenticated Encryption"
 *
 * Definition of CCM*:
 * IEEE 802.15.4 - IEEE Standard for Local and metropolitan area networks
 * Integer representation is fixed most-significant-octet-first order and
 * the representation of octets is most-significant-bit-first order. This is
 * consistent with RFC 3610.
 */
/*
 *  Copyright The Mbed TLS Contributors
 *  Copyright (c) 2023, Nuvoton Technology Corporation
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

#ifndef MBEDTLS_CCM_ALT_H
#define MBEDTLS_CCM_ALT_H

#include "mbedtls/private_access.h"

#include "mbedtls/build_info.h"

#include "mbedtls/cipher.h"

#include "NuMicro.h"

#define MAX_CCM_BUF     256

#define MBEDTLS_CCM_DECRYPT       0
#define MBEDTLS_CCM_ENCRYPT       1
#define MBEDTLS_CCM_STAR_DECRYPT  2
#define MBEDTLS_CCM_STAR_ENCRYPT  3

/** Bad input parameters to the function. */
#define MBEDTLS_ERR_CCM_BAD_INPUT       -0x000D
/** Authenticated decryption failed. */
#define MBEDTLS_ERR_CCM_AUTH_FAILED     -0x000F

#ifdef __cplusplus
extern "C" {
#endif


// Regular implementation
//

/**
 * \brief    The CCM context-type definition. The CCM context is passed
 *           to the APIs called.
 */
typedef struct mbedtls_ccm_context
{
    unsigned char MBEDTLS_PRIVATE(y)[16];    /*!< The Y working buffer */
    unsigned char MBEDTLS_PRIVATE(ctr)[16];  /*!< The counter buffer */
    mbedtls_cipher_context_t MBEDTLS_PRIVATE(cipher_ctx);    /*!< The cipher context used. */
    size_t MBEDTLS_PRIVATE(plaintext_len);   /*!< Total plaintext length */
    size_t MBEDTLS_PRIVATE(add_len);         /*!< Total authentication data length */
    size_t MBEDTLS_PRIVATE(tag_len);         /*!< Total tag length */
    size_t MBEDTLS_PRIVATE(processed);       /*!< Track how many bytes of input data
                                                  were processed (chunked input).
                                                  Used independently for both auth data
                                                  and plaintext/ciphertext.
                                                  This variable is set to zero after
                                                  auth data input is finished. */
    unsigned char MBEDTLS_PRIVATE(q);        /*!< The Q working value */
    unsigned char MBEDTLS_PRIVATE(mode);     /*!< The operation to perform:
                                                  #MBEDTLS_CCM_ENCRYPT or
                                                  #MBEDTLS_CCM_DECRYPT or
                                                  #MBEDTLS_CCM_STAR_ENCRYPT or
                                                  #MBEDTLS_CCM_STAR_DECRYPT. */
    int MBEDTLS_PRIVATE(state);              /*!< Working value holding context's
                                                  state. Used for chunked data
                                                  input */


    /* -------------------------------------- */
    __ALIGNED(4) uint8_t MBEDTLS_PRIVATE(ccm_buf)[MAX_CCM_BUF];
    __ALIGNED(4) uint8_t MBEDTLS_PRIVATE(out_buf)[MAX_CCM_BUF + 16];
    uint8_t MBEDTLS_PRIVATE(key)[32];
    uint32_t MBEDTLS_PRIVATE(keybits);
    uint32_t MBEDTLS_PRIVATE(basicOp);
    
}
mbedtls_ccm_context;

#ifdef __cplusplus
}
#endif

#endif /* MBEDTLS_CCM_ALT_H */
