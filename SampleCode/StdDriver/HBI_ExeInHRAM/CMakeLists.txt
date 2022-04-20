

set(PRJ HBI_ExeInHRAM)
add_executable(${PRJ} main.c exeinhram.c)
target_link_libraries(${PRJ} m460_startup m460_driver)
target_compile_definitions(m460_startup
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

set(DOWNLOAD_BOARD FALSE)

# Convert to binary and copy to target device
if(${DOWNLOAD_BOARD})

    add_custom_command(TARGET ${PRJ} POST_BUILD
       COMMAND ${ARM_FROMELF} --bin --output y:/${PRJ}.bin ${PRJ}.elf
       VERBATIM
    )

endif()
