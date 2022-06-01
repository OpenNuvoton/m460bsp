

set(PRJ HBI_RW_MemMap)
add_executable(${PRJ} main.c)
target_link_libraries(${PRJ} system_startup standard_driver)
target_compile_definitions(system_startup
  PUBLIC
    HBI_ENABLE=2
)

target_link_options(${PRJ}
   PUBLIC
    --cpu=Cortex-M4.fp.sp
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

# Convert to binary and copy to target device
add_custom_command(TARGET ${PRJ} POST_BUILD
    COMMAND ${ARM_FROMELF} --bin --output ${PRJ}.bin ${PRJ}.elf
    VERBATIM
)

