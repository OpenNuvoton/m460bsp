set(CRPT_SRC
    aes_alt.c
    ccm_alt.c
    ecdh_alt.c
    ecdsa_alt.c
    ecp_internal_alt.c
    gcm_alt.c
    rsa_alt.c
    sha256_alt.c
    trng_api.c
    platform_alt.c
    mbedtls_config.h
)

add_library(crypto_accellerator ${CRPT_SRC})



