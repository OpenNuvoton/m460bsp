enable_language(C ASM)

set(PRJ FMC_IAP_LDROM_iap)
add_executable(${PRJ} LDROM_iap.c)
target_link_libraries(${PRJ} system_startup standard_driver)

if(C_COMPILER_ID STREQUAL ARMCLANG)
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
        COMMAND ${ARM_FROMELF} --bin --output fmc_ld_code.bin ${PRJ}.elf
        VERBATIM
    )
endif()

if(C_COMPILER_ID STREQUAL GCC)
    target_link_options(${PRJ}
        PUBLIC
        -T ${BSP_DIR}/Library/Device/Nuvoton/m460/Source/GCC/gcc_arm.ld
        -Xlinker
        --gc-sections
        -Wl,-Map=${PRJ}.map
        -gdwarf-3
    )

    add_custom_command(TARGET ${PRJ} POST_BUILD
        COMMAND ${ARM_OBJCOPY} -O binary ${PRJ} fmc_ld_code.bin
        VERBATIM
    )
endif()

# #####################################################################################################
set(PRJ FMC_IAP)

if(C_COMPILER_ID STREQUAL ARMCLANG)
    add_executable(${PRJ} APROM_main.c ${CMAKE_CURRENT_LIST_DIR}/GCC/ap_image.S)
    target_link_libraries(${PRJ} system_startup standard_driver)

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
endif()

if(C_COMPILER_ID STREQUAL GCC)
    add_executable(${PRJ} APROM_main.c ${CMAKE_CURRENT_LIST_DIR}/GCC/ap_image.S)
    target_link_libraries(${PRJ} system_startup standard_driver)
    target_compile_definitions(${PRJ} PUBLIC _ARM_VSCODE_)

    target_link_options(${PRJ}
        PUBLIC
        -T ${BSP_DIR}/Library/Device/Nuvoton/m460/Source/GCC/gcc_arm.ld
        -Xlinker
        --gc-sections
        -Wl,-Map=${PRJ}.map
        -gdwarf-3
    )

    add_custom_command(TARGET ${PRJ} POST_BUILD
        COMMAND ${ARM_OBJCOPY} -O binary ${PRJ} ${PRJ}.bin
        VERBATIM
    )
endif()

target_compile_definitions(${PRJ} PUBLIC BIN_FILE="${CMAKE_BINARY_DIR}/SampleCode/StdDriver/FMC_IAP/fmc_ld_code.bin")

add_dependencies(FMC_IAP FMC_IAP_LDROM_iap)