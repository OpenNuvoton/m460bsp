

set(PRJ FMC_FwUpdateApplication_Loader)
add_executable(${PRJ} main.c ${CMAKE_CURRENT_LIST_DIR}/../../../../Library/NuMaker/common/xmodem.c)
target_link_libraries(${PRJ} system_startup standard_driver)
target_include_directories(${PRJ}
    PUBLIC
        ${CMAKE_CURRENT_LIST_DIR}/../common/inc
        ${CMAKE_CURRENT_LIST_DIR}/../../../../Library/NuMaker/common
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

add_custom_command(TARGET ${PRJ} POST_BUILD
    COMMAND ${ARM_FROMELF} --bin --output ${PRJ}.bin ${PRJ}.elf
    VERBATIM
)

