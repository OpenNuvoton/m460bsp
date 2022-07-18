

set(PRJ mbedTLS_ECDH)

add_executable(${PRJ} test_suite_ecdh.c)

target_link_libraries(${PRJ} system_startup standard_driver mbedcrypto crypto_accelerator test_lib)
target_include_directories(${PRJ}
    PUBLIC
    ${BSP_DIR}/ThirdParty/mbedtls-3.1.0/tests/include

)
target_link_options(${PRJ}
   PUBLIC
    --cpu=${LINK_MCU_CORE}
    --library_type=microlib
    --ro-base 0x00000000
    --rw-base 0x20000000
    --entry Reset_Handler
    --first __Vectors
    --strict
    --map
    --datacompressor=off
    --info=inline
    --entry Reset_Handler
    --summary_stderr
    --info totals
    --map
    --load_addr_map_info
    --xref
    --callgraph
    --symbols
)

add_custom_command(TARGET ${PRJ}
   COMMAND ${ARM_FROMELF} --bin --output ${PRJ}.bin ${PRJ}.elf
   VERBATIM
)
