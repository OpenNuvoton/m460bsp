

set(PRJ HBI_ExeInHRAM)
add_executable(${PRJ} main.c exeinhram.c)
target_link_libraries(${PRJ} system_startup standard_driver)
target_compile_definitions(system_startup
  PUBLIC
    HBI_ENABLE=2
)

target_link_options(${PRJ}
   PUBLIC
    --cpu=Cortex-M4.fp.sp
    --library_type=microlib
    --entry Reset_Handler
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
    --scatter ${CMAKE_CURRENT_LIST_DIR}/Keil/HBI_ExeInHRAM.sct
)

add_custom_command(TARGET ${PRJ} POST_BUILD
    COMMAND ${ARM_FROMELF} --bin --output ${PRJ}.bin ${PRJ}.elf
    VERBATIM
)
