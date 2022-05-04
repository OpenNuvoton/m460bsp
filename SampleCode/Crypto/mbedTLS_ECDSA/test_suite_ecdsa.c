/*
 * *** THIS FILE HAS BEEN MACHINE GENERATED ***
 *
 * This file has been machine generated using the script:
 * generate_test_code.py
 *
 * Test file      : .\test_suite_ecdsa.c
 *
 * The following files were used to create this file.
 *
 *      Main code file      : suites/main_test.function
 *      Platform code file  : suites/host_test.function
 *      Helper file         : suites/helpers.function
 *      Test suite file     : suites/test_suite_ecdsa.function
 *      Test suite data     : suites/test_suite_ecdsa.data
 *
 */

#if defined(__unix__) || (defined(__APPLE__) && defined(__MACH__))
#if !defined(_POSIX_C_SOURCE)
#define _POSIX_C_SOURCE 200112L // for fileno() from <stdio.h>
#endif
#endif

#include "mbedtls/build_info.h"

/* Test code may use deprecated identifiers only if the preprocessor symbol
 * MBEDTLS_TEST_DEPRECATED is defined. When building tests, set
 * MBEDTLS_TEST_DEPRECATED explicitly if MBEDTLS_DEPRECATED_WARNING is
 * enabled but the corresponding warnings are not treated as errors.
 */
#if !defined(MBEDTLS_DEPRECATED_REMOVED) && !defined(MBEDTLS_DEPRECATED_WARNING)
#define MBEDTLS_TEST_DEPRECATED
#endif

/*----------------------------------------------------------------------------*/
/* Common helper code */

/*----------------------------------------------------------------------------*/
/* Headers */

#include <test/helpers.h>
#include <test/macros.h>
#include <test/random.h>
#include <test/psa_crypto_helpers.h>

#include <stdlib.h>

#include "NuMicro.h"

#if defined (MBEDTLS_ERROR_C)
#include "mbedtls/error.h"
#endif
#if defined(MBEDTLS_PLATFORM_C)
#include "mbedtls/platform.h"
#else
#include <stdio.h>
#define mbedtls_fprintf    fprintf
#define mbedtls_snprintf   snprintf
#define mbedtls_calloc     calloc
#define mbedtls_free       free
#define mbedtls_exit       exit
#define mbedtls_time       time
#define mbedtls_time_t     time_t
#define MBEDTLS_EXIT_SUCCESS EXIT_SUCCESS
#define MBEDTLS_EXIT_FAILURE EXIT_FAILURE
#endif

#if defined(MBEDTLS_MEMORY_BUFFER_ALLOC_C)
#include "mbedtls/memory_buffer_alloc.h"
#endif

#ifdef _MSC_VER
#include <basetsd.h>
typedef UINT8 uint8_t;
typedef INT32 int32_t;
typedef UINT32 uint32_t;
#define strncasecmp _strnicmp
#define strcasecmp _stricmp
#else
#include <stdint.h>
#endif

#include <string.h>

#if defined(__unix__) || (defined(__APPLE__) && defined(__MACH__))
#include <unistd.h>
#include <strings.h>
#endif

/* Type for Hex parameters */
typedef struct data_tag
{
    uint8_t *   x;
    uint32_t    len;
} data_t;

/*----------------------------------------------------------------------------*/
/* Status and error constants */

#define DEPENDENCY_SUPPORTED            0   /* Dependency supported by build */
#define KEY_VALUE_MAPPING_FOUND         0   /* Integer expression found */
#define DISPATCH_TEST_SUCCESS           0   /* Test dispatch successful */

#define KEY_VALUE_MAPPING_NOT_FOUND     -1  /* Integer expression not found */
#define DEPENDENCY_NOT_SUPPORTED        -2  /* Dependency not supported */
#define DISPATCH_TEST_FN_NOT_FOUND      -3  /* Test function not found */
#define DISPATCH_INVALID_TEST_DATA      -4  /* Invalid test parameter type.
                                               Only int, string, binary data
                                               and integer expressions are
                                               allowed */
#define DISPATCH_UNSUPPORTED_SUITE      -5  /* Test suite not supported by the
                                               build */

/*----------------------------------------------------------------------------*/
/* Global variables */

/*----------------------------------------------------------------------------*/
/* Helper flags for complex dependencies */

/* Indicates whether we expect mbedtls_entropy_init
 * to initialize some strong entropy source. */
#if !defined(MBEDTLS_NO_DEFAULT_ENTROPY_SOURCES) && \
    ( !defined(MBEDTLS_NO_PLATFORM_ENTROPY) ||      \
        defined(MBEDTLS_ENTROPY_HARDWARE_ALT) ||    \
        defined(ENTROPY_NV_SEED) )
#define ENTROPY_HAVE_STRONG
#endif


 /*----------------------------------------------------------------------------*/
 /* test patterns */
#include "dat.h"

typedef struct {
    const char* base;
    const char* limit;
    unsigned int ofs;
} MYFILE;

MYFILE g_myfile;

char* myfgets(char* s, int n, FILE* f)
{
    MYFILE* myfile;
    int32_t i;
    int32_t ofs;
    const char* pu8Base, * pu8Limit;

    myfile = (MYFILE*)f;

    pu8Base = myfile->base;
    pu8Limit = myfile->limit;
    ofs = myfile->ofs;

    /* End of file check */
    if(pu8Base + ofs >= pu8Limit)
        return 0;

    /* Seek to offset */
    pu8Base += ofs;

    /* Check length to read */
    if(pu8Base + n > pu8Limit)
    {
        /* touch the eof */
        n = (int)(pu8Limit - pu8Base);
    }

    for(i = 0; i < n; i++)
    {
        s[i] = pu8Base[i];

        /* Check new line */
        if((pu8Base[i] == 0x0d) && (pu8Base[i + 1] == 0x0a))
        {
            s[i] = 0x0a;
            i++;

            /* ahead one byte due to CRLF --> LF */
            myfile->ofs += 1;
            break;
        }

        if(pu8Base[i] == 0x0a)
        {
            i++;
            break;
        }
    }

    /* update file offset */
    myfile->ofs += i;

    /* append 0 */
    if(i < n)
        s[i] = 0;

    return s;
}

int myfeof(FILE* f)
{
    MYFILE* myfile;
    myfile = (MYFILE*)f;
    if(myfile->base + myfile->ofs >= myfile->limit)
        return 1;
    return 0;
}

void myfclose(FILE* f)
{
    MYFILE* myfile;

    myfile = (MYFILE*)f;

    myfile->ofs = 0;
}

FILE* myfopen(char const* fname, char const* mode)
{
    g_myfile.base = _dat;
    g_myfile.limit = _dat + sizeof(_dat);
    g_myfile.ofs = 0;
    return (FILE*)&g_myfile;
}



/*----------------------------------------------------------------------------*/
/* Helper Functions */

#if defined(MBEDTLS_PSA_CRYPTO_C)
/** Check that no PSA Crypto key slots are in use.
 *
 * If any slots are in use, mark the current test as failed.
 *
 * \return 0 if the key store is empty, 1 otherwise.
 */
int test_fail_if_psa_leaking( int line_no, const char *filename )
{
    const char *msg = mbedtls_test_helper_is_psa_leaking( );
    if( msg == NULL )
        return 0;
    else
    {
        mbedtls_test_fail( msg, line_no, filename );
        return 1;
    }
}
#endif /* defined(MBEDTLS_PSA_CRYPTO_C) */

#if defined(__unix__) || (defined(__APPLE__) && defined(__MACH__))
static int redirect_output( FILE* out_stream, const char* path )
{
    int out_fd, dup_fd;
    FILE* path_stream;

    out_fd = fileno( out_stream );
    dup_fd = dup( out_fd );

    if( dup_fd == -1 )
    {
        return( -1 );
    }

    path_stream = fopen( path, "w" );
    if( path_stream == NULL )
    {
        close( dup_fd );
        return( -1 );
    }

    fflush( out_stream );
    if( dup2( fileno( path_stream ), out_fd ) == -1 )
    {
        close( dup_fd );
        fclose( path_stream );
        return( -1 );
    }

    fclose( path_stream );
    return( dup_fd );
}

static int restore_output( FILE* out_stream, int dup_fd )
{
    int out_fd = fileno( out_stream );

    fflush( out_stream );
    if( dup2( dup_fd, out_fd ) == -1 )
    {
        close( out_fd );
        close( dup_fd );
        return( -1 );
    }

    close( dup_fd );
    return( 0 );
}
#endif /* __unix__ || __APPLE__ __MACH__ */



/*----------------------------------------------------------------------------*/
/* Test Suite Code */


#define TEST_SUITE_ACTIVE

#if defined(MBEDTLS_ECDSA_C)
#include "mbedtls/ecdsa.h"
void test_ecdsa_prim_zero( int id )
{
    mbedtls_ecp_group grp;
    mbedtls_ecp_point Q;
    mbedtls_mpi d, r, s;
    mbedtls_test_rnd_pseudo_info rnd_info;
    unsigned char buf[MBEDTLS_MD_MAX_SIZE];

    mbedtls_ecp_group_init( &grp );
    mbedtls_ecp_point_init( &Q );
    mbedtls_mpi_init( &d ); mbedtls_mpi_init( &r ); mbedtls_mpi_init( &s );
    memset( &rnd_info, 0x00, sizeof( mbedtls_test_rnd_pseudo_info ) );
    memset( buf, 0, sizeof( buf ) );

    TEST_ASSERT( mbedtls_ecp_group_load( &grp, id ) == 0 );
    TEST_ASSERT( mbedtls_ecp_gen_keypair( &grp, &d, &Q,
                                          &mbedtls_test_rnd_pseudo_rand,
                                          &rnd_info ) == 0 );

    TEST_ASSERT( mbedtls_ecdsa_sign( &grp, &r, &s, &d, buf, sizeof( buf ),
                                     &mbedtls_test_rnd_pseudo_rand,
                                     &rnd_info ) == 0 );
    TEST_ASSERT( mbedtls_ecdsa_verify( &grp, buf, sizeof( buf ), &Q, &r, &s ) == 0 );

exit:
    mbedtls_ecp_group_free( &grp );
    mbedtls_ecp_point_free( &Q );
    mbedtls_mpi_free( &d ); mbedtls_mpi_free( &r ); mbedtls_mpi_free( &s );
}

void test_ecdsa_prim_zero_wrapper( void ** params )
{

    test_ecdsa_prim_zero( *( (int *) params[0] ) );
}
void test_ecdsa_prim_random( int id )
{
    mbedtls_ecp_group grp;
    mbedtls_ecp_point Q;
    mbedtls_mpi d, r, s;
    mbedtls_test_rnd_pseudo_info rnd_info;
    unsigned char buf[MBEDTLS_MD_MAX_SIZE];

    mbedtls_ecp_group_init( &grp );
    mbedtls_ecp_point_init( &Q );
    mbedtls_mpi_init( &d ); mbedtls_mpi_init( &r ); mbedtls_mpi_init( &s );
    memset( &rnd_info, 0x00, sizeof( mbedtls_test_rnd_pseudo_info ) );
    memset( buf, 0, sizeof( buf ) );

    /* prepare material for signature */
    TEST_ASSERT( mbedtls_test_rnd_pseudo_rand( &rnd_info,
                                               buf, sizeof( buf ) ) == 0 );
    TEST_ASSERT( mbedtls_ecp_group_load( &grp, id ) == 0 );
    TEST_ASSERT( mbedtls_ecp_gen_keypair( &grp, &d, &Q,
                                          &mbedtls_test_rnd_pseudo_rand,
                                          &rnd_info ) == 0 );

    TEST_ASSERT( mbedtls_ecdsa_sign( &grp, &r, &s, &d, buf, sizeof( buf ),
                                     &mbedtls_test_rnd_pseudo_rand,
                                     &rnd_info ) == 0 );
    TEST_ASSERT( mbedtls_ecdsa_verify( &grp, buf, sizeof( buf ), &Q, &r, &s ) == 0 );

exit:
    mbedtls_ecp_group_free( &grp );
    mbedtls_ecp_point_free( &Q );
    mbedtls_mpi_free( &d ); mbedtls_mpi_free( &r ); mbedtls_mpi_free( &s );
}

void test_ecdsa_prim_random_wrapper( void ** params )
{

    test_ecdsa_prim_random( *( (int *) params[0] ) );
}

void test_ecdsa_prim_test_vectors( int id, char * d_str, char * xQ_str,
                              char * yQ_str, data_t * rnd_buf,
                              data_t * hash, char * r_str, char * s_str,
                              int result )
{
    mbedtls_ecp_group grp;
    mbedtls_ecp_point Q;
    mbedtls_mpi d, r, s, r_check, s_check;
    mbedtls_test_rnd_buf_info rnd_info;

    mbedtls_ecp_group_init( &grp );
    mbedtls_ecp_point_init( &Q );
    mbedtls_mpi_init( &d ); mbedtls_mpi_init( &r ); mbedtls_mpi_init( &s );
    mbedtls_mpi_init( &r_check ); mbedtls_mpi_init( &s_check );

    TEST_ASSERT( mbedtls_ecp_group_load( &grp, id ) == 0 );
    TEST_ASSERT( mbedtls_ecp_point_read_string( &Q, 16, xQ_str, yQ_str ) == 0 );
    TEST_ASSERT( mbedtls_test_read_mpi( &d, 16, d_str ) == 0 );
    TEST_ASSERT( mbedtls_test_read_mpi( &r_check, 16, r_str ) == 0 );
    TEST_ASSERT( mbedtls_test_read_mpi( &s_check, 16, s_str ) == 0 );
    rnd_info.fallback_f_rng = mbedtls_test_rnd_std_rand;
    rnd_info.fallback_p_rng = NULL;
    rnd_info.buf = rnd_buf->x;
    rnd_info.length = rnd_buf->len;

    /* Fix rnd_buf->x by shifting it left if necessary */
    if( grp.nbits % 8 != 0 )
    {
        unsigned char shift = 8 - ( grp.nbits % 8 );
        size_t i;

        for( i = 0; i < rnd_info.length - 1; i++ )
            rnd_buf->x[i] = rnd_buf->x[i] << shift | rnd_buf->x[i+1] >> ( 8 - shift );

        rnd_buf->x[rnd_info.length-1] <<= shift;
    }

    TEST_ASSERT( mbedtls_ecdsa_sign( &grp, &r, &s, &d, hash->x, hash->len,
                 mbedtls_test_rnd_buffer_rand, &rnd_info ) == result );

    if ( result == 0)
    {
        TEST_ASSERT( mbedtls_mpi_cmp_mpi( &r, &r_check ) == 0 );
        TEST_ASSERT( mbedtls_mpi_cmp_mpi( &s, &s_check ) == 0 );

        TEST_ASSERT( mbedtls_ecdsa_verify( &grp, hash->x, hash->len, &Q, &r_check, &s_check ) == 0 );

        TEST_ASSERT( mbedtls_mpi_sub_int( &r, &r, 1 ) == 0 );
        TEST_ASSERT( mbedtls_mpi_add_int( &s, &s, 1 ) == 0 );

        TEST_ASSERT( mbedtls_ecdsa_verify( &grp, hash->x, hash->len,
                     &Q, &r, &s_check ) == MBEDTLS_ERR_ECP_VERIFY_FAILED );
        TEST_ASSERT( mbedtls_ecdsa_verify( &grp, hash->x, hash->len,
                     &Q, &r_check, &s ) == MBEDTLS_ERR_ECP_VERIFY_FAILED );
        TEST_ASSERT( mbedtls_ecdsa_verify( &grp, hash->x, hash->len,
                     &grp.G, &r_check, &s_check ) == MBEDTLS_ERR_ECP_VERIFY_FAILED );
    }

exit:
    mbedtls_ecp_group_free( &grp );
    mbedtls_ecp_point_free( &Q );
    mbedtls_mpi_free( &d ); mbedtls_mpi_free( &r ); mbedtls_mpi_free( &s );
    mbedtls_mpi_free( &r_check ); mbedtls_mpi_free( &s_check );
}

void test_ecdsa_prim_test_vectors_wrapper( void ** params )
{
    data_t data4 = {(uint8_t *) params[4], *( (uint32_t *) params[5] )};
    data_t data6 = {(uint8_t *) params[6], *( (uint32_t *) params[7] )};

    test_ecdsa_prim_test_vectors( *( (int *) params[0] ), (char *) params[1], (char *) params[2], (char *) params[3], &data4, &data6, (char *) params[8], (char *) params[9], *( (int *) params[10] ) );
}
#if defined(MBEDTLS_ECDSA_DETERMINISTIC)
void test_ecdsa_det_test_vectors( int id, char * d_str, int md_alg, char * msg,
                             char * r_str, char * s_str )
{
    mbedtls_ecp_group grp;
    mbedtls_mpi d, r, s, r_check, s_check;
    unsigned char hash[MBEDTLS_MD_MAX_SIZE];
    size_t hlen;
    const mbedtls_md_info_t *md_info;

    mbedtls_ecp_group_init( &grp );
    mbedtls_mpi_init( &d ); mbedtls_mpi_init( &r ); mbedtls_mpi_init( &s );
    mbedtls_mpi_init( &r_check ); mbedtls_mpi_init( &s_check );
    memset( hash, 0, sizeof( hash ) );

    TEST_ASSERT( mbedtls_ecp_group_load( &grp, id ) == 0 );
    TEST_ASSERT( mbedtls_test_read_mpi( &d, 16, d_str ) == 0 );
    TEST_ASSERT( mbedtls_test_read_mpi( &r_check, 16, r_str ) == 0 );
    TEST_ASSERT( mbedtls_test_read_mpi( &s_check, 16, s_str ) == 0 );

    md_info = mbedtls_md_info_from_type( md_alg );
    TEST_ASSERT( md_info != NULL );
    hlen = mbedtls_md_get_size( md_info );
    TEST_ASSERT( mbedtls_md( md_info, (const unsigned char *) msg,
                 strlen( msg ), hash ) == 0 );

    TEST_ASSERT(
                mbedtls_ecdsa_sign_det_ext( &grp, &r, &s, &d, hash, hlen,
                                            md_alg, mbedtls_test_rnd_std_rand,
                                            NULL )
                == 0 );

    TEST_ASSERT( mbedtls_mpi_cmp_mpi( &r, &r_check ) == 0 );
    TEST_ASSERT( mbedtls_mpi_cmp_mpi( &s, &s_check ) == 0 );

exit:
    mbedtls_ecp_group_free( &grp );
    mbedtls_mpi_free( &d ); mbedtls_mpi_free( &r ); mbedtls_mpi_free( &s );
    mbedtls_mpi_free( &r_check ); mbedtls_mpi_free( &s_check );
}

void test_ecdsa_det_test_vectors_wrapper( void ** params )
{

    test_ecdsa_det_test_vectors( *( (int *) params[0] ), (char *) params[1], *( (int *) params[2] ), (char *) params[3], (char *) params[4], (char *) params[5] );
}
#endif /* MBEDTLS_ECDSA_DETERMINISTIC */
#if defined(MBEDTLS_SHA256_C)
void test_ecdsa_write_read_zero( int id )
{
    mbedtls_ecdsa_context ctx;
    mbedtls_test_rnd_pseudo_info rnd_info;
    unsigned char hash[32];
    unsigned char sig[200];
    size_t sig_len, i;

    mbedtls_ecdsa_init( &ctx );
    memset( &rnd_info, 0x00, sizeof( mbedtls_test_rnd_pseudo_info ) );
    memset( hash, 0, sizeof( hash ) );
    memset( sig, 0x2a, sizeof( sig ) );

    /* generate signing key */
    TEST_ASSERT( mbedtls_ecdsa_genkey( &ctx, id,
                                       &mbedtls_test_rnd_pseudo_rand,
                                       &rnd_info ) == 0 );

    /* generate and write signature, then read and verify it */
    TEST_ASSERT( mbedtls_ecdsa_write_signature( &ctx, MBEDTLS_MD_SHA256,
                 hash, sizeof( hash ),
                 sig, sizeof( sig ), &sig_len, &mbedtls_test_rnd_pseudo_rand,
                 &rnd_info ) == 0 );
    TEST_ASSERT( mbedtls_ecdsa_read_signature( &ctx, hash, sizeof( hash ),
                 sig, sig_len ) == 0 );

    /* check we didn't write past the announced length */
    for( i = sig_len; i < sizeof( sig ); i++ )
        TEST_ASSERT( sig[i] == 0x2a );

    /* try verification with invalid length */
    TEST_ASSERT( mbedtls_ecdsa_read_signature( &ctx, hash, sizeof( hash ),
                 sig, sig_len - 1 ) != 0 );
    TEST_ASSERT( mbedtls_ecdsa_read_signature( &ctx, hash, sizeof( hash ),
                 sig, sig_len + 1 ) != 0 );

    /* try invalid sequence tag */
    sig[0]++;
    TEST_ASSERT( mbedtls_ecdsa_read_signature( &ctx, hash, sizeof( hash ),
                 sig, sig_len ) != 0 );
    sig[0]--;

    /* try modifying r */
    sig[10]++;
    TEST_ASSERT( mbedtls_ecdsa_read_signature( &ctx, hash, sizeof( hash ),
                 sig, sig_len ) == MBEDTLS_ERR_ECP_VERIFY_FAILED );
    sig[10]--;

    /* try modifying s */
    sig[sig_len - 1]++;
    TEST_ASSERT( mbedtls_ecdsa_read_signature( &ctx, hash, sizeof( hash ),
                 sig, sig_len ) == MBEDTLS_ERR_ECP_VERIFY_FAILED );
    sig[sig_len - 1]--;

exit:
    mbedtls_ecdsa_free( &ctx );
}

void test_ecdsa_write_read_zero_wrapper( void ** params )
{

    test_ecdsa_write_read_zero( *( (int *) params[0] ) );
}
#endif /* MBEDTLS_SHA256_C */
#if defined(MBEDTLS_SHA256_C)
void test_ecdsa_write_read_random( int id )
{
    mbedtls_ecdsa_context ctx;
    mbedtls_test_rnd_pseudo_info rnd_info;
    unsigned char hash[32];
    unsigned char sig[200];
    size_t sig_len, i;

    mbedtls_ecdsa_init( &ctx );
    memset( &rnd_info, 0x00, sizeof( mbedtls_test_rnd_pseudo_info ) );
    memset( hash, 0, sizeof( hash ) );
    memset( sig, 0x2a, sizeof( sig ) );

    /* prepare material for signature */
    TEST_ASSERT( mbedtls_test_rnd_pseudo_rand( &rnd_info,
                                               hash, sizeof( hash ) ) == 0 );

    /* generate signing key */
    TEST_ASSERT( mbedtls_ecdsa_genkey( &ctx, id,
                                       &mbedtls_test_rnd_pseudo_rand,
                                       &rnd_info ) == 0 );

    /* generate and write signature, then read and verify it */
    TEST_ASSERT( mbedtls_ecdsa_write_signature( &ctx, MBEDTLS_MD_SHA256,
                 hash, sizeof( hash ),
                 sig, sizeof( sig ), &sig_len, &mbedtls_test_rnd_pseudo_rand,
                 &rnd_info ) == 0 );
    TEST_ASSERT( mbedtls_ecdsa_read_signature( &ctx, hash, sizeof( hash ),
                 sig, sig_len ) == 0 );

    /* check we didn't write past the announced length */
    for( i = sig_len; i < sizeof( sig ); i++ )
        TEST_ASSERT( sig[i] == 0x2a );

    /* try verification with invalid length */
    TEST_ASSERT( mbedtls_ecdsa_read_signature( &ctx, hash, sizeof( hash ),
                 sig, sig_len - 1 ) != 0 );
    TEST_ASSERT( mbedtls_ecdsa_read_signature( &ctx, hash, sizeof( hash ),
                 sig, sig_len + 1 ) != 0 );

    /* try invalid sequence tag */
    sig[0]++;
    TEST_ASSERT( mbedtls_ecdsa_read_signature( &ctx, hash, sizeof( hash ),
                 sig, sig_len ) != 0 );
    sig[0]--;

    /* try modifying r */
    sig[10]++;
    TEST_ASSERT( mbedtls_ecdsa_read_signature( &ctx, hash, sizeof( hash ),
                 sig, sig_len ) == MBEDTLS_ERR_ECP_VERIFY_FAILED );
    sig[10]--;

    /* try modifying s */
    sig[sig_len - 1]++;
    TEST_ASSERT( mbedtls_ecdsa_read_signature( &ctx, hash, sizeof( hash ),
                 sig, sig_len ) == MBEDTLS_ERR_ECP_VERIFY_FAILED );
    sig[sig_len - 1]--;

exit:
    mbedtls_ecdsa_free( &ctx );
}

void test_ecdsa_write_read_random_wrapper( void ** params )
{

    test_ecdsa_write_read_random( *( (int *) params[0] ) );
}
#endif /* MBEDTLS_SHA256_C */
#if defined(MBEDTLS_ECP_RESTARTABLE)
void test_ecdsa_read_restart( int id, data_t *pk, data_t *hash, data_t *sig,
                         int max_ops, int min_restart, int max_restart )
{
    mbedtls_ecdsa_context ctx;
    mbedtls_ecdsa_restart_ctx rs_ctx;
    int ret, cnt_restart;

    mbedtls_ecdsa_init( &ctx );
    mbedtls_ecdsa_restart_init( &rs_ctx );

    TEST_ASSERT( mbedtls_ecp_group_load( &ctx.grp, id ) == 0 );
    TEST_ASSERT( mbedtls_ecp_point_read_binary( &ctx.grp, &ctx.Q,
                                                pk->x, pk->len ) == 0 );

    mbedtls_ecp_set_max_ops( max_ops );

    cnt_restart = 0;
    do {
        ret = mbedtls_ecdsa_read_signature_restartable( &ctx,
                            hash->x, hash->len, sig->x, sig->len, &rs_ctx );
    } while( ret == MBEDTLS_ERR_ECP_IN_PROGRESS && ++cnt_restart );

    TEST_ASSERT( ret == 0 );
    TEST_ASSERT( cnt_restart >= min_restart );
    TEST_ASSERT( cnt_restart <= max_restart );

    /* try modifying r */

    TEST_ASSERT( sig->len > 10 );
    sig->x[10]++;
    do {
        ret = mbedtls_ecdsa_read_signature_restartable( &ctx,
                            hash->x, hash->len, sig->x, sig->len, &rs_ctx );
    } while( ret == MBEDTLS_ERR_ECP_IN_PROGRESS );
    TEST_ASSERT( ret == MBEDTLS_ERR_ECP_VERIFY_FAILED );
    sig->x[10]--;

    /* try modifying s */
    sig->x[sig->len - 1]++;
    do {
        ret = mbedtls_ecdsa_read_signature_restartable( &ctx,
                            hash->x, hash->len, sig->x, sig->len, &rs_ctx );
    } while( ret == MBEDTLS_ERR_ECP_IN_PROGRESS );
    TEST_ASSERT( ret == MBEDTLS_ERR_ECP_VERIFY_FAILED );
    sig->x[sig->len - 1]--;

    /* Do we leak memory when aborting an operation?
     * This test only makes sense when we actually restart */
    if( min_restart > 0 )
    {
        ret = mbedtls_ecdsa_read_signature_restartable( &ctx,
                            hash->x, hash->len, sig->x, sig->len, &rs_ctx );
        TEST_ASSERT( ret == MBEDTLS_ERR_ECP_IN_PROGRESS );
    }

exit:
    mbedtls_ecdsa_free( &ctx );
    mbedtls_ecdsa_restart_free( &rs_ctx );
}

void test_ecdsa_read_restart_wrapper( void ** params )
{
    data_t data1 = {(uint8_t *) params[1], *( (uint32_t *) params[2] )};
    data_t data3 = {(uint8_t *) params[3], *( (uint32_t *) params[4] )};
    data_t data5 = {(uint8_t *) params[5], *( (uint32_t *) params[6] )};

    test_ecdsa_read_restart( *( (int *) params[0] ), &data1, &data3, &data5, *( (int *) params[7] ), *( (int *) params[8] ), *( (int *) params[9] ) );
}
#endif /* MBEDTLS_ECP_RESTARTABLE */
#if defined(MBEDTLS_ECP_RESTARTABLE)
#if defined(MBEDTLS_ECDSA_DETERMINISTIC)
void test_ecdsa_write_restart( int id, char *d_str, int md_alg,
                          char *msg, data_t *sig_check,
                          int max_ops, int min_restart, int max_restart )
{
    int ret, cnt_restart;
    mbedtls_ecdsa_restart_ctx rs_ctx;
    mbedtls_ecdsa_context ctx;
    unsigned char hash[MBEDTLS_MD_MAX_SIZE];
    unsigned char sig[MBEDTLS_ECDSA_MAX_LEN];
    size_t hlen, slen;
    const mbedtls_md_info_t *md_info;

    mbedtls_ecdsa_restart_init( &rs_ctx );
    mbedtls_ecdsa_init( &ctx );
    memset( hash, 0, sizeof( hash ) );
    memset( sig, 0, sizeof( sig ) );

    TEST_ASSERT( mbedtls_ecp_group_load( &ctx.grp, id ) == 0 );
    TEST_ASSERT( mbedtls_test_read_mpi( &ctx.d, 16, d_str ) == 0 );

    md_info = mbedtls_md_info_from_type( md_alg );
    TEST_ASSERT( md_info != NULL );

    hlen = mbedtls_md_get_size( md_info );
    TEST_ASSERT( mbedtls_md( md_info,
                             (const unsigned char *) msg, strlen( msg ),
                             hash ) == 0 );

    mbedtls_ecp_set_max_ops( max_ops );

    slen = sizeof( sig );
    cnt_restart = 0;
    do {
        ret = mbedtls_ecdsa_write_signature_restartable( &ctx,
                md_alg, hash, hlen, sig, sizeof( sig ), &slen,
                mbedtls_test_rnd_std_rand, NULL, &rs_ctx );
    } while( ret == MBEDTLS_ERR_ECP_IN_PROGRESS && ++cnt_restart );

    TEST_ASSERT( ret == 0 );
    TEST_ASSERT( slen == sig_check->len );
    TEST_ASSERT( memcmp( sig, sig_check->x, slen ) == 0 );

    TEST_ASSERT( cnt_restart >= min_restart );
    TEST_ASSERT( cnt_restart <= max_restart );

    /* Do we leak memory when aborting an operation?
     * This test only makes sense when we actually restart */
    if( min_restart > 0 )
    {
        ret = mbedtls_ecdsa_write_signature_restartable( &ctx,
                md_alg, hash, hlen, sig, sizeof( sig ), &slen,
                mbedtls_test_rnd_std_rand, NULL, &rs_ctx );
        TEST_ASSERT( ret == MBEDTLS_ERR_ECP_IN_PROGRESS );
    }

exit:
    mbedtls_ecdsa_restart_free( &rs_ctx );
    mbedtls_ecdsa_free( &ctx );
}

void test_ecdsa_write_restart_wrapper( void ** params )
{
    data_t data4 = {(uint8_t *) params[4], *( (uint32_t *) params[5] )};

    test_ecdsa_write_restart( *( (int *) params[0] ), (char *) params[1], *( (int *) params[2] ), (char *) params[3], &data4, *( (int *) params[6] ), *( (int *) params[7] ), *( (int *) params[8] ) );
}
#endif /* MBEDTLS_ECDSA_DETERMINISTIC */
#endif /* MBEDTLS_ECP_RESTARTABLE */
#endif /* MBEDTLS_ECDSA_C */


/*----------------------------------------------------------------------------*/
/* Test dispatch code */


/**
 * \brief       Evaluates an expression/macro into its literal integer value.
 *              For optimizing space for embedded targets each expression/macro
 *              is identified by a unique identifier instead of string literals.
 *              Identifiers and evaluation code is generated by script:
 *              generate_test_code.py
 *
 * \param exp_id    Expression identifier.
 * \param out_value Pointer to int to hold the integer.
 *
 * \return       0 if exp_id is found. 1 otherwise.
 */
int get_expression( int32_t exp_id, int32_t * out_value )
{
    int ret = KEY_VALUE_MAPPING_FOUND;

    (void) exp_id;
    (void) out_value;

    switch( exp_id )
    {

#if defined(MBEDTLS_ECDSA_C)

        case 0:
            {
                *out_value = MBEDTLS_ECP_DP_SECP192R1;
            }
            break;
        case 1:
            {
                *out_value = MBEDTLS_ECP_DP_SECP224R1;
            }
            break;
        case 2:
            {
                *out_value = MBEDTLS_ECP_DP_SECP256R1;
            }
            break;
        case 3:
            {
                *out_value = MBEDTLS_ECP_DP_SECP384R1;
            }
            break;
        case 4:
            {
                *out_value = MBEDTLS_ECP_DP_SECP521R1;
            }
            break;
        case 5:
            {
                *out_value = MBEDTLS_MD_SHA1;
            }
            break;
        case 6:
            {
                *out_value = MBEDTLS_MD_SHA224;
            }
            break;
        case 7:
            {
                *out_value = MBEDTLS_MD_SHA256;
            }
            break;
        case 8:
            {
                *out_value = MBEDTLS_MD_SHA384;
            }
            break;
        case 9:
            {
                *out_value = MBEDTLS_MD_SHA512;
            }
            break;
        case 10:
            {
                *out_value = MBEDTLS_ERR_ECP_INVALID_KEY;
            }
            break;
#endif

        default:
           {
                ret = KEY_VALUE_MAPPING_NOT_FOUND;
           }
           break;
    }
    return( ret );
}


/**
 * \brief       Checks if the dependency i.e. the compile flag is set.
 *              For optimizing space for embedded targets each dependency
 *              is identified by a unique identifier instead of string literals.
 *              Identifiers and check code is generated by script:
 *              generate_test_code.py
 *
 * \param dep_id    Dependency identifier.
 *
 * \return       DEPENDENCY_SUPPORTED if set else DEPENDENCY_NOT_SUPPORTED
 */
int dep_check( int dep_id )
{
    int ret = DEPENDENCY_NOT_SUPPORTED;

    (void) dep_id;

    switch( dep_id )
    {

#if defined(MBEDTLS_ECDSA_C)

        case 0:
            {
#if defined(MBEDTLS_ECP_DP_SECP192R1_ENABLED)
                ret = DEPENDENCY_SUPPORTED;
#else
                ret = DEPENDENCY_NOT_SUPPORTED;
#endif
            }
            break;
        case 1:
            {
#if defined(MBEDTLS_ECP_DP_SECP224R1_ENABLED)
                ret = DEPENDENCY_SUPPORTED;
#else
                ret = DEPENDENCY_NOT_SUPPORTED;
#endif
            }
            break;
        case 2:
            {
#if defined(MBEDTLS_ECP_DP_SECP256R1_ENABLED)
                ret = DEPENDENCY_SUPPORTED;
#else
                ret = DEPENDENCY_NOT_SUPPORTED;
#endif
            }
            break;
        case 3:
            {
#if defined(MBEDTLS_ECP_DP_SECP384R1_ENABLED)
                ret = DEPENDENCY_SUPPORTED;
#else
                ret = DEPENDENCY_NOT_SUPPORTED;
#endif
            }
            break;
        case 4:
            {
#if defined(MBEDTLS_ECP_DP_SECP521R1_ENABLED)
                ret = DEPENDENCY_SUPPORTED;
#else
                ret = DEPENDENCY_NOT_SUPPORTED;
#endif
            }
            break;
        case 5:
            {
#if defined(MBEDTLS_SHA1_C)
                ret = DEPENDENCY_SUPPORTED;
#else
                ret = DEPENDENCY_NOT_SUPPORTED;
#endif
            }
            break;
        case 6:
            {
#if defined(MBEDTLS_SHA224_C)
                ret = DEPENDENCY_SUPPORTED;
#else
                ret = DEPENDENCY_NOT_SUPPORTED;
#endif
            }
            break;
        case 7:
            {
#if defined(MBEDTLS_SHA256_C)
                ret = DEPENDENCY_SUPPORTED;
#else
                ret = DEPENDENCY_NOT_SUPPORTED;
#endif
            }
            break;
        case 8:
            {
#if defined(MBEDTLS_SHA384_C)
                ret = DEPENDENCY_SUPPORTED;
#else
                ret = DEPENDENCY_NOT_SUPPORTED;
#endif
            }
            break;
        case 9:
            {
#if defined(MBEDTLS_SHA512_C)
                ret = DEPENDENCY_SUPPORTED;
#else
                ret = DEPENDENCY_NOT_SUPPORTED;
#endif
            }
            break;
#endif

        default:
            break;
    }
    return( ret );
}


/**
 * \brief       Function pointer type for test function wrappers.
 *
 * A test function wrapper decodes the parameters and passes them to the
 * underlying test function. Both the wrapper and the underlying function
 * return void. Test wrappers assume that they are passed a suitable
 * parameter array and do not perform any error detection.
 *
 * \param param_array   The array of parameters. Each element is a `void *`
 *                      which the wrapper casts to the correct type and
 *                      dereferences. Each wrapper function hard-codes the
 *                      number and types of the parameters.
 */
typedef void (*TestWrapper_t)( void **param_array );


/**
 * \brief       Table of test function wrappers. Used by dispatch_test().
 *              This table is populated by script:
 *              generate_test_code.py
 *
 */
TestWrapper_t test_funcs[] =
{
/* Function Id: 0 */

#if defined(MBEDTLS_ECDSA_C)
    test_ecdsa_prim_zero_wrapper,
#else
    NULL,
#endif
/* Function Id: 1 */

#if defined(MBEDTLS_ECDSA_C)
    test_ecdsa_prim_random_wrapper,
#else
    NULL,
#endif
/* Function Id: 2 */

#if defined(MBEDTLS_ECDSA_C)
    test_ecdsa_prim_test_vectors_wrapper,
#else
    NULL,
#endif
/* Function Id: 3 */

#if defined(MBEDTLS_ECDSA_C) && defined(MBEDTLS_ECDSA_DETERMINISTIC)
    test_ecdsa_det_test_vectors_wrapper,
#else
    NULL,
#endif
/* Function Id: 4 */

#if defined(MBEDTLS_ECDSA_C) && defined(MBEDTLS_SHA256_C)
    test_ecdsa_write_read_zero_wrapper,
#else
    NULL,
#endif
/* Function Id: 5 */

#if defined(MBEDTLS_ECDSA_C) && defined(MBEDTLS_SHA256_C)
    test_ecdsa_write_read_random_wrapper,
#else
    NULL,
#endif
/* Function Id: 6 */

#if defined(MBEDTLS_ECDSA_C) && defined(MBEDTLS_ECP_RESTARTABLE)
    test_ecdsa_read_restart_wrapper,
#else
    NULL,
#endif
/* Function Id: 7 */

#if defined(MBEDTLS_ECDSA_C) && defined(MBEDTLS_ECP_RESTARTABLE) && defined(MBEDTLS_ECDSA_DETERMINISTIC)
    test_ecdsa_write_restart_wrapper,
#else
    NULL,
#endif

};

/**
 * \brief        Dispatches test functions based on function index.
 *
 * \param func_idx    Test function index.
 * \param params      The array of parameters to pass to the test function.
 *                    It will be decoded by the #TestWrapper_t wrapper function.
 *
 * \return       DISPATCH_TEST_SUCCESS if found
 *               DISPATCH_TEST_FN_NOT_FOUND if not found
 *               DISPATCH_UNSUPPORTED_SUITE if not compile time enabled.
 */
int dispatch_test( size_t func_idx, void ** params )
{
    int ret = DISPATCH_TEST_SUCCESS;
    TestWrapper_t fp = NULL;

    if ( func_idx < (int)( sizeof( test_funcs ) / sizeof( TestWrapper_t ) ) )
    {
        fp = test_funcs[func_idx];
        if ( fp )
        {
            #if defined(MBEDTLS_PSA_CRYPTO_EXTERNAL_RNG)
                mbedtls_test_enable_insecure_external_rng( );
            #endif

                fp( params );

            #if defined(MBEDTLS_TEST_MUTEX_USAGE)
                mbedtls_test_mutex_usage_check( );
            #endif /* MBEDTLS_TEST_MUTEX_USAGE */
        }
        else
            ret = DISPATCH_UNSUPPORTED_SUITE;
    }
    else
    {
        ret = DISPATCH_TEST_FN_NOT_FOUND;
    }

    return( ret );
}


/**
 * \brief       Checks if test function is supported in this build-time
 *              configuration.
 *
 * \param func_idx    Test function index.
 *
 * \return       DISPATCH_TEST_SUCCESS if found
 *               DISPATCH_TEST_FN_NOT_FOUND if not found
 *               DISPATCH_UNSUPPORTED_SUITE if not compile time enabled.
 */
int check_test( size_t func_idx )
{
    int ret = DISPATCH_TEST_SUCCESS;
    TestWrapper_t fp = NULL;

    if ( func_idx < (int)( sizeof(test_funcs)/sizeof( TestWrapper_t ) ) )
    {
        fp = test_funcs[func_idx];
        if ( fp == NULL )
            ret = DISPATCH_UNSUPPORTED_SUITE;
    }
    else
    {
        ret = DISPATCH_TEST_FN_NOT_FOUND;
    }

    return( ret );
}


/**
 * \brief       Verifies that string is in string parameter format i.e. "<str>"
 *              It also strips enclosing '"' from the input string.
 *
 * \param str   String parameter.
 *
 * \return      0 if success else 1
 */
int verify_string( char **str )
{
    if( ( *str )[0] != '"' ||
        ( *str )[strlen( *str ) - 1] != '"' )
    {
        mbedtls_fprintf( stderr,
            "Expected string (with \"\") for parameter and got: %s\n", *str );
        return( -1 );
    }

    ( *str )++;
    ( *str )[strlen( *str ) - 1] = '\0';

    return( 0 );
}

/**
 * \brief       Verifies that string is an integer. Also gives the converted
 *              integer value.
 *
 * \param str   Input string.
 * \param value Pointer to int for output value.
 *
 * \return      0 if success else 1
 */
int verify_int( char *str, int32_t *value )
{
    size_t i;
    int minus = 0;
    int digits = 1;
    int hex = 0;

    for( i = 0; i < strlen( str ); i++ )
    {
        if( i == 0 && str[i] == '-' )
        {
            minus = 1;
            continue;
        }

        if( ( ( minus && i == 2 ) || ( !minus && i == 1 ) ) &&
            str[i - 1] == '0' && ( str[i] == 'x' || str[i] == 'X' ) )
        {
            hex = 1;
            continue;
        }

        if( ! ( ( str[i] >= '0' && str[i] <= '9' ) ||
                ( hex && ( ( str[i] >= 'a' && str[i] <= 'f' ) ||
                           ( str[i] >= 'A' && str[i] <= 'F' ) ) ) ) )
        {
            digits = 0;
            break;
        }
    }

    if( digits )
    {
        if( hex )
            *value = strtol( str, NULL, 16 );
        else
            *value = strtol( str, NULL, 10 );

        return( 0 );
    }

    mbedtls_fprintf( stderr,
                    "Expected integer for parameter and got: %s\n", str );
    return( KEY_VALUE_MAPPING_NOT_FOUND );
}


/**
 * \brief       Usage string.
 *
 */
#define USAGE \
    "Usage: %s [OPTIONS] files...\n\n" \
    "   Command line arguments:\n" \
    "     files...          One or more test data files. If no file is\n" \
    "                       specified the following default test case\n" \
    "                       file is used:\n" \
    "                           %s\n\n" \
    "   Options:\n" \
    "     -v | --verbose    Display full information about each test\n" \
    "     -h | --help       Display this information\n\n", \
    argv[0], \
    "TESTCASE_FILENAME"


/**
 * \brief       Read a line from the passed file pointer.
 *
 * \param f     FILE pointer
 * \param buf   Pointer to memory to hold read line.
 * \param len   Length of the buf.
 *
 * \return      0 if success else -1
 */
int get_line( FILE *f, char *buf, size_t len )
{
    char *ret;
    int i = 0, str_len = 0, has_string = 0;

    /* Read until we get a valid line */
    do
    {
        ret = myfgets( buf, len, f );
        if( ret == NULL )
            return( -1 );

        str_len = strlen( buf );

        /* Skip empty line and comment */
        if ( str_len == 0 || buf[0] == '#' )
            continue;
        has_string = 0;
        for ( i = 0; i < str_len; i++ )
        {
            char c = buf[i];
            if ( c != ' ' && c != '\t' && c != '\n' &&
                 c != '\v' && c != '\f' && c != '\r' )
            {
                has_string = 1;
                break;
            }
        }
    } while( !has_string );

    /* Strip new line and carriage return */
    ret = buf + strlen( buf );
    if( ret-- > buf && *ret == '\n' )
        *ret = '\0';
    if( ret-- > buf && *ret == '\r' )
        *ret = '\0';

    return( 0 );
}

/**
 * \brief       Splits string delimited by ':'. Ignores '\:'.
 *
 * \param buf           Input string
 * \param len           Input string length
 * \param params        Out params found
 * \param params_len    Out params array len
 *
 * \return      Count of strings found.
 */
static int parse_arguments( char *buf, size_t len, char **params,
                            size_t params_len )
{
    size_t cnt = 0, i;
    char *cur = buf;
    char *p = buf, *q;

    params[cnt++] = cur;

    while( *p != '\0' && p < ( buf + len ) )
    {
        if( *p == '\\' )
        {
            p++;
            p++;
            continue;
        }
        if( *p == ':' )
        {
            if( p + 1 < buf + len )
            {
                cur = p + 1;
                TEST_HELPER_ASSERT( cnt < params_len );
                params[cnt++] = cur;
            }
            *p = '\0';
        }

        p++;
    }

    /* Replace newlines, question marks and colons in strings */
    for( i = 0; i < cnt; i++ )
    {
        p = params[i];
        q = params[i];

        while( *p != '\0' )
        {
            if( *p == '\\' && *( p + 1 ) == 'n' )
            {
                p += 2;
                *( q++ ) = '\n';
            }
            else if( *p == '\\' && *( p + 1 ) == ':' )
            {
                p += 2;
                *( q++ ) = ':';
            }
            else if( *p == '\\' && *( p + 1 ) == '?' )
            {
                p += 2;
                *( q++ ) = '?';
            }
            else
                *( q++ ) = *( p++ );
        }
        *q = '\0';
    }

    return( cnt );
}

/**
 * \brief       Converts parameters into test function consumable parameters.
 *              Example: Input:  {"int", "0", "char*", "Hello",
 *                                "hex", "abef", "exp", "1"}
 *                      Output:  {
 *                                0,                // Verified int
 *                                "Hello",          // Verified string
 *                                2, { 0xab, 0xef },// Converted len,hex pair
 *                                9600              // Evaluated expression
 *                               }
 *
 *
 * \param cnt               Parameter array count.
 * \param params            Out array of found parameters.
 * \param int_params_store  Memory for storing processed integer parameters.
 *
 * \return      0 for success else 1
 */
static int convert_params( size_t cnt , char ** params , int32_t * int_params_store )
{
    char ** cur = params;
    char ** out = params;
    int ret = DISPATCH_TEST_SUCCESS;

    while ( cur < params + cnt )
    {
        char * type = *cur++;
        char * val = *cur++;

        if ( strcmp( type, "char*" ) == 0 )
        {
            if ( verify_string( &val ) == 0 )
            {
              *out++ = val;
            }
            else
            {
                ret = ( DISPATCH_INVALID_TEST_DATA );
                break;
            }
        }
        else if ( strcmp( type, "int" ) == 0 )
        {
            if ( verify_int( val, int_params_store ) == 0 )
            {
              *out++ = (char *) int_params_store++;
            }
            else
            {
                ret = ( DISPATCH_INVALID_TEST_DATA );
                break;
            }
        }
        else if ( strcmp( type, "hex" ) == 0 )
        {
            if ( verify_string( &val ) == 0 )
            {
                size_t len;

                TEST_HELPER_ASSERT(
                  mbedtls_test_unhexify( (unsigned char *) val, strlen( val ),
                                         val, &len ) == 0 );

                *int_params_store = len;
                *out++ = val;
                *out++ = (char *)(int_params_store++);
            }
            else
            {
                ret = ( DISPATCH_INVALID_TEST_DATA );
                break;
            }
        }
        else if ( strcmp( type, "exp" ) == 0 )
        {
            int exp_id = strtol( val, NULL, 10 );
            if ( get_expression ( exp_id, int_params_store ) == 0 )
            {
              *out++ = (char *)int_params_store++;
            }
            else
            {
              ret = ( DISPATCH_INVALID_TEST_DATA );
              break;
            }
        }
        else
        {
          ret = ( DISPATCH_INVALID_TEST_DATA );
          break;
        }
    }
    return( ret );
}

/**
 * \brief       Tests snprintf implementation with test input.
 *
 * \note
 * At high optimization levels (e.g. gcc -O3), this function may be
 * inlined in run_test_snprintf. This can trigger a spurious warning about
 * potential misuse of snprintf from gcc -Wformat-truncation (observed with
 * gcc 7.2). This warning makes tests in run_test_snprintf redundant on gcc
 * only. They are still valid for other compilers. Avoid this warning by
 * forbidding inlining of this function by gcc.
 *
 * \param n         Buffer test length.
 * \param ref_buf   Expected buffer.
 * \param ref_ret   Expected snprintf return value.
 *
 * \return      0 for success else 1
 */
#if defined(__GNUC__)
__attribute__((__noinline__))
#endif
static int test_snprintf( size_t n, const char *ref_buf, int ref_ret )
{
    int ret;
    char buf[10] = "xxxxxxxxx";
    const char ref[10] = "xxxxxxxxx";

    if( n >= sizeof( buf ) )
        return( -1 );
    ret = mbedtls_snprintf( buf, n, "%s", "123" );
    if( ret < 0 || (size_t) ret >= n )
        ret = -1;

    if( strncmp( ref_buf, buf, sizeof( buf ) ) != 0 ||
        ref_ret != ret ||
        memcmp( buf + n, ref + n, sizeof( buf ) - n ) != 0 )
    {
        return( 1 );
    }

    return( 0 );
}

/**
 * \brief       Tests snprintf implementation.
 *
 * \return      0 for success else 1
 */
static int run_test_snprintf( void )
{
    return( test_snprintf( 0, "xxxxxxxxx",  -1 ) != 0 ||
            test_snprintf( 1, "",           -1 ) != 0 ||
            test_snprintf( 2, "1",          -1 ) != 0 ||
            test_snprintf( 3, "12",         -1 ) != 0 ||
            test_snprintf( 4, "123",         3 ) != 0 ||
            test_snprintf( 5, "123",         3 ) != 0 );
}

/** \brief Write the description of the test case to the outcome CSV file.
 *
 * \param outcome_file  The file to write to.
 *                      If this is \c NULL, this function does nothing.
 * \param argv0         The test suite name.
 * \param test_case     The test case description.
 */
static void write_outcome_entry( FILE *outcome_file,
                                 const char *argv0,
                                 const char *test_case )
{
    /* The non-varying fields are initialized on first use. */
    static const char *platform = NULL;
    static const char *configuration = NULL;
    static const char *test_suite = NULL;

    if( outcome_file == NULL )
        return;

    if( platform == NULL )
    {
        platform = getenv( "MBEDTLS_TEST_PLATFORM" );
        if( platform == NULL )
            platform = "unknown";
    }
    if( configuration == NULL )
    {
        configuration = getenv( "MBEDTLS_TEST_CONFIGURATION" );
        if( configuration == NULL )
            configuration = "unknown";
    }
    if( test_suite == NULL )
    {
        test_suite = strrchr( argv0, '/' );
        if( test_suite != NULL )
            test_suite += 1; // skip the '/'
        else
            test_suite = argv0;
    }

    /* Write the beginning of the outcome line.
     * Ignore errors: writing the outcome file is on a best-effort basis. */
    mbedtls_fprintf( outcome_file, "%s;%s;%s;%s;",
                     platform, configuration, test_suite, test_case );
}

/** \brief Write the result of the test case to the outcome CSV file.
 *
 * \param outcome_file  The file to write to.
 *                      If this is \c NULL, this function does nothing.
 * \param unmet_dep_count            The number of unmet dependencies.
 * \param unmet_dependencies         The array of unmet dependencies.
 * \param missing_unmet_dependencies Non-zero if there was a problem tracking
 *                                   all unmet dependencies, 0 otherwise.
 * \param ret                        The test dispatch status (DISPATCH_xxx).
 * \param info                       A pointer to the test info structure.
 */
static void write_outcome_result( FILE *outcome_file,
                                  size_t unmet_dep_count,
                                  int unmet_dependencies[],
                                  int missing_unmet_dependencies,
                                  int ret,
                                  const mbedtls_test_info_t *info )
{
    if( outcome_file == NULL )
        return;

    /* Write the end of the outcome line.
     * Ignore errors: writing the outcome file is on a best-effort basis. */
    switch( ret )
    {
        case DISPATCH_TEST_SUCCESS:
            if( unmet_dep_count > 0 )
            {
                size_t i;
                mbedtls_fprintf( outcome_file, "SKIP" );
                for( i = 0; i < unmet_dep_count; i++ )
                {
                    mbedtls_fprintf( outcome_file, "%c%d",
                                     i == 0 ? ';' : ':',
                                     unmet_dependencies[i] );
                }
                if( missing_unmet_dependencies )
                    mbedtls_fprintf( outcome_file, ":..." );
                break;
            }
            switch( info->result )
            {
                case MBEDTLS_TEST_RESULT_SUCCESS:
                    mbedtls_fprintf( outcome_file, "PASS;" );
                    break;
                case MBEDTLS_TEST_RESULT_SKIPPED:
                    mbedtls_fprintf( outcome_file, "SKIP;Runtime skip" );
                    break;
                default:
                    mbedtls_fprintf( outcome_file, "FAIL;%s:%d:%s",
                                     info->filename, info->line_no,
                                     info->test );
                    break;
            }
            break;
        case DISPATCH_TEST_FN_NOT_FOUND:
            mbedtls_fprintf( outcome_file, "FAIL;Test function not found" );
            break;
        case DISPATCH_INVALID_TEST_DATA:
            mbedtls_fprintf( outcome_file, "FAIL;Invalid test data" );
            break;
        case DISPATCH_UNSUPPORTED_SUITE:
            mbedtls_fprintf( outcome_file, "SKIP;Unsupported suite" );
            break;
        default:
            mbedtls_fprintf( outcome_file, "FAIL;Unknown cause" );
            break;
    }
    mbedtls_fprintf( outcome_file, "\n" );
    fflush( outcome_file );
}

/**
 * \brief       Desktop implementation of execute_tests().
 *              Parses command line and executes tests from
 *              supplied or default data file.
 *
 * \param argc  Command line argument count.
 * \param argv  Argument array.
 *
 * \return      Program exit status.
 */
int execute_tests( int argc , const char ** argv )
{
    /* Local Configurations and options */
    const char *default_filename = ".\\test_suite_ecdsa.datax";
    const char *test_filename = NULL;
    const char **test_files = NULL;
    size_t testfile_count = 0;
    int option_verbose = 0;
    size_t function_id = 0;

    /* Other Local variables */
    int arg_index = 1;
    const char *next_arg;
    size_t testfile_index, i, cnt;
    int ret;
    unsigned total_errors = 0, total_tests = 0, total_skipped = 0;
    FILE *file;
    char buf[5000];
    char *params[50];
    /* Store for proccessed integer params. */
    int32_t int_params[50];
    void *pointer;
#if defined(__unix__) || (defined(__APPLE__) && defined(__MACH__))
    int stdout_fd = -1;
#endif /* __unix__ || __APPLE__ __MACH__ */
    const char *outcome_file_name = getenv( "MBEDTLS_TEST_OUTCOME_FILE" );
    FILE *outcome_file = NULL;

#if defined(MBEDTLS_MEMORY_BUFFER_ALLOC_C) && \
    !defined(TEST_SUITE_MEMORY_BUFFER_ALLOC)
    unsigned char alloc_buf[1000000];
    mbedtls_memory_buffer_alloc_init( alloc_buf, sizeof( alloc_buf ) );
#endif

#if defined(MBEDTLS_TEST_MUTEX_USAGE)
    mbedtls_test_mutex_usage_init( );
#endif

    /*
     * The C standard doesn't guarantee that all-bits-0 is the representation
     * of a NULL pointer. We do however use that in our code for initializing
     * structures, which should work on every modern platform. Let's be sure.
     */
    memset( &pointer, 0, sizeof( void * ) );
    if( pointer != NULL )
    {
        mbedtls_fprintf( stderr, "all-bits-zero is not a NULL pointer\n" );
        return( 1 );
    }

    /*
     * Make sure we have a snprintf that correctly zero-terminates
     */
    if( run_test_snprintf() != 0 )
    {
        mbedtls_fprintf( stderr, "the snprintf implementation is broken\n" );
        return( 1 );
    }

    if( outcome_file_name != NULL && *outcome_file_name != '\0' )
    {
        outcome_file = fopen( outcome_file_name, "a" );
        if( outcome_file == NULL )
        {
            mbedtls_fprintf( stderr, "Unable to open outcome file. Continuing anyway.\n" );
        }
    }

    while( arg_index < argc )
    {
        next_arg = argv[arg_index];

        if( strcmp( next_arg, "--verbose" ) == 0 ||
                 strcmp( next_arg, "-v" ) == 0 )
        {
            option_verbose = 1;
        }
        else if( strcmp(next_arg, "--help" ) == 0 ||
                 strcmp(next_arg, "-h" ) == 0 )
        {
            mbedtls_fprintf( stdout, USAGE );
            mbedtls_exit( EXIT_SUCCESS );
        }
        else
        {
            /* Not an option, therefore treat all further arguments as the file
             * list.
             */
            test_files = &argv[ arg_index ];
            testfile_count = argc - arg_index;
        }

        arg_index++;
    }

    /* If no files were specified, assume a default */
    if ( test_files == NULL || testfile_count == 0 )
    {
        test_files = &default_filename;
        testfile_count = 1;
    }

    /* Initialize the struct that holds information about the last test */
    mbedtls_test_info_reset( );

    /* Now begin to execute the tests in the testfiles */
    for ( testfile_index = 0;
          testfile_index < testfile_count;
          testfile_index++ )
    {
        size_t unmet_dep_count = 0;
        int unmet_dependencies[20];
        int missing_unmet_dependencies = 0;

        test_filename = test_files[ testfile_index ];

        file = myfopen( test_filename, "r" );
        if( file == NULL )
        {
            mbedtls_fprintf( stderr, "Failed to open test file: %s\n",
                             test_filename );
            if( outcome_file != NULL )
                fclose( outcome_file );
            return( 1 );
        }

        while( !myfeof( file ) )
        {
            if( unmet_dep_count > 0 )
            {
                mbedtls_fprintf( stderr,
                    "FATAL: Dep count larger than zero at start of loop\n" );
                mbedtls_exit( MBEDTLS_EXIT_FAILURE );
            }
            unmet_dep_count = 0;
            missing_unmet_dependencies = 0;

            if( ( ret = get_line( file, buf, sizeof(buf) ) ) != 0 )
                break;
            mbedtls_fprintf( stdout, "%s%.66s",
                    mbedtls_test_info.result == MBEDTLS_TEST_RESULT_FAILED ?
                    "\n" : "", buf );
            mbedtls_fprintf( stdout, " " );
            for( i = strlen( buf ) + 1; i < 67; i++ )
                mbedtls_fprintf( stdout, "." );
            mbedtls_fprintf( stdout, " " );
            fflush( stdout );
            write_outcome_entry( outcome_file, argv[0], buf );

            total_tests++;

            if( ( ret = get_line( file, buf, sizeof( buf ) ) ) != 0 )
                break;
            cnt = parse_arguments( buf, strlen( buf ), params,
                                   sizeof( params ) / sizeof( params[0] ) );

            if( strcmp( params[0], "depends_on" ) == 0 )
            {
                for( i = 1; i < cnt; i++ )
                {
                    int dep_id = strtol( params[i], NULL, 10 );
                    if( dep_check( dep_id ) != DEPENDENCY_SUPPORTED )
                    {
                        if( unmet_dep_count <
                            ARRAY_LENGTH( unmet_dependencies ) )
                        {
                            unmet_dependencies[unmet_dep_count] = dep_id;
                            unmet_dep_count++;
                        }
                        else
                        {
                            missing_unmet_dependencies = 1;
                        }
                    }
                }

                if( ( ret = get_line( file, buf, sizeof( buf ) ) ) != 0 )
                    break;
                cnt = parse_arguments( buf, strlen( buf ), params,
                                       sizeof( params ) / sizeof( params[0] ) );
            }

            // If there are no unmet dependencies execute the test
            if( unmet_dep_count == 0 )
            {
                mbedtls_test_info_reset( );

#if defined(__unix__) || (defined(__APPLE__) && defined(__MACH__))
                /* Suppress all output from the library unless we're verbose
                 * mode
                 */
                if( !option_verbose )
                {
                    stdout_fd = redirect_output( stdout, "/dev/null" );
                    if( stdout_fd == -1 )
                    {
                        /* Redirection has failed with no stdout so exit */
                        exit( 1 );
                    }
                }
#endif /* __unix__ || __APPLE__ __MACH__ */

                function_id = strtoul( params[0], NULL, 10 );
                if ( (ret = check_test( function_id )) == DISPATCH_TEST_SUCCESS )
                {
                    ret = convert_params( cnt - 1, params + 1, int_params );
                    if ( DISPATCH_TEST_SUCCESS == ret )
                    {
                        ret = dispatch_test( function_id, (void **)( params + 1 ) );
                    }
                }

#if defined(__unix__) || (defined(__APPLE__) && defined(__MACH__))
                if( !option_verbose && restore_output( stdout, stdout_fd ) )
                {
                        /* Redirection has failed with no stdout so exit */
                        exit( 1 );
                }
#endif /* __unix__ || __APPLE__ __MACH__ */

            }

            write_outcome_result( outcome_file,
                                  unmet_dep_count, unmet_dependencies,
                                  missing_unmet_dependencies,
                                  ret, &mbedtls_test_info );
            if( unmet_dep_count > 0 || ret == DISPATCH_UNSUPPORTED_SUITE )
            {
                total_skipped++;
                mbedtls_fprintf( stdout, "----" );

                if( 1 == option_verbose && ret == DISPATCH_UNSUPPORTED_SUITE )
                {
                    mbedtls_fprintf( stdout, "\n   Test Suite not enabled" );
                }

                if( 1 == option_verbose && unmet_dep_count > 0 )
                {
                    mbedtls_fprintf( stdout, "\n   Unmet dependencies: " );
                    for( i = 0; i < unmet_dep_count; i++ )
                    {
                        mbedtls_fprintf( stdout, "%d ",
                                        unmet_dependencies[i] );
                    }
                    if( missing_unmet_dependencies )
                        mbedtls_fprintf( stdout, "..." );
                }
                mbedtls_fprintf( stdout, "\n" );
                fflush( stdout );

                unmet_dep_count = 0;
                missing_unmet_dependencies = 0;
            }
            else if( ret == DISPATCH_TEST_SUCCESS )
            {
                if( mbedtls_test_info.result == MBEDTLS_TEST_RESULT_SUCCESS )
                {
                    mbedtls_fprintf( stdout, "PASS\n" );
                }
                else if( mbedtls_test_info.result == MBEDTLS_TEST_RESULT_SKIPPED )
                {
                    mbedtls_fprintf( stdout, "----\n" );
                    total_skipped++;
                }
                else
                {
                    total_errors++;
                    mbedtls_fprintf( stdout, "FAILED\n" );
                    mbedtls_fprintf( stdout, "  %s\n  at ",
                                     mbedtls_test_info.test );
                    if( mbedtls_test_info.step != (unsigned long)( -1 ) )
                    {
                        mbedtls_fprintf( stdout, "step %lu, ",
                                         mbedtls_test_info.step );
                    }
                    mbedtls_fprintf( stdout, "line %d, %s",
                                     mbedtls_test_info.line_no,
                                     mbedtls_test_info.filename );
                    if( mbedtls_test_info.line1[0] != 0 )
                        mbedtls_fprintf( stdout, "\n  %s",
                                         mbedtls_test_info.line1 );
                    if( mbedtls_test_info.line2[0] != 0 )
                        mbedtls_fprintf( stdout, "\n  %s",
                                         mbedtls_test_info.line2 );
                }
                fflush( stdout );
            }
            else if( ret == DISPATCH_INVALID_TEST_DATA )
            {
                mbedtls_fprintf( stderr, "FAILED: FATAL PARSE ERROR\n" );
                myfclose( file );
                mbedtls_exit( 2 );
            }
            else if( ret == DISPATCH_TEST_FN_NOT_FOUND )
            {
                mbedtls_fprintf( stderr, "FAILED: FATAL TEST FUNCTION NOT FOUND\n" );
                myfclose( file );
                mbedtls_exit( 2 );
            }
            else
                total_errors++;
        }
        myfclose( file );
    }

    if( outcome_file != NULL )
        fclose( outcome_file );

    mbedtls_fprintf( stdout, "\n----------------------------------------------------------------------------\n\n");
    if( total_errors == 0 )
        mbedtls_fprintf( stdout, "PASSED" );
    else
        mbedtls_fprintf( stdout, "FAILED" );

    mbedtls_fprintf( stdout, " (%u / %u tests (%u skipped))\n",
                     total_tests - total_errors, total_tests, total_skipped );

#if defined(MBEDTLS_MEMORY_BUFFER_ALLOC_C) && \
    !defined(TEST_SUITE_MEMORY_BUFFER_ALLOC)
#if defined(MBEDTLS_MEMORY_DEBUG)
    mbedtls_memory_buffer_alloc_status();
#endif
    mbedtls_memory_buffer_alloc_free();
#endif

    return( total_errors != 0 );
}


/*----------------------------------------------------------------------------*/
/* Main Test code */

volatile uint32_t g_u32Ticks = 0;
void SysTick_Handler()
{
    g_u32Ticks++;
}



/**
 * \brief       Program main. Invokes platform specific execute_tests().
 *
 * \param argc      Command line arguments count.
 * \param argv      Array of command line arguments.
 *
 * \return       Exit code.
 */
int main()
{
    int argc = 0;
    const char* argv[] = {0};

#if defined(MBEDTLS_TEST_HOOKS)
    extern void (*mbedtls_test_hook_test_fail)( const char * test, int line, const char * file );
    mbedtls_test_hook_test_fail = &mbedtls_test_fail;
#if defined(MBEDTLS_ERROR_C)
    mbedtls_test_hook_error_add = &mbedtls_test_err_add_check;
#endif
#endif

    int ret = mbedtls_test_platform_setup();
    if( ret != 0 )
    {
        mbedtls_fprintf( stderr,
                         "FATAL: Failed to initialize platform - error %d\n",
                         ret );
        return( -1 );
    }

#if (defined(MBEDTLS_ECDSA_VERIFY_ALT) || defined(MBEDTLS_ECDSA_SIGN_ALT))
    printf("Hardware Accellerator Enabled.\n");
#else
    printf("Pure software crypto running.\n");
#endif

    SysTick_Config(SystemCoreClock / 1000);

    g_u32Ticks = 0;
    ret = execute_tests( argc, argv );
    printf("Total elapsed time is %d ms\n", g_u32Ticks);

    mbedtls_test_platform_teardown();

    for(;;) {}

}
