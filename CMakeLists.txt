cmake_minimum_required(VERSION 3.21)
cmake_policy(SET CMP0123 NEW)

# Create a symbolic link from ${base_name} in the binary directory
# to the corresponding path in the source directory.
function(link_to_source base_name)
    # Get OS dependent path to use in `execute_process`
    if(CMAKE_HOST_WIN32)
        # mklink is an internal command of cmd.exe it can only work with \
        string(REPLACE "/" "\\" link "${CMAKE_CURRENT_BINARY_DIR}/${base_name}")
        string(REPLACE "/" "\\" target "${CMAKE_CURRENT_SOURCE_DIR}/${base_name}")
    else()
        set(link "${CMAKE_CURRENT_BINARY_DIR}/${base_name}")
        set(target "${CMAKE_CURRENT_SOURCE_DIR}/${base_name}")
    endif()

    if(NOT EXISTS ${link})
        if(CMAKE_HOST_UNIX)
            set(command ln -s ${target} ${link})
        else()
            if(IS_DIRECTORY ${target})
                set(command cmd.exe /c mklink /j ${link} ${target})
            else()
                set(command cmd.exe /c mklink /h ${link} ${target})
            endif()
        endif()

        execute_process(COMMAND ${command}
            RESULT_VARIABLE result
            ERROR_VARIABLE output)

        if(NOT ${result} EQUAL 0)
            message(FATAL_ERROR "Could not create symbolic link for: ${target} --> ${output}")
        endif()
    endif()
endfunction(link_to_source)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION "5")
set(CMAKE_SYSTEM_PROCESSOR ARM)

if(C_COMPILER_ID STREQUAL ARMCLANG)

    find_program(ARM_CC NAMES armclang REQUIRED)
    find_program(ARM_CXX NAMES armclang REQUIRED)
    find_program(ARM_ASM NAMES armclang REQUIRED)
    find_program(ARM_AR NAMES armar REQUIRED)
    find_program(ARM_LINK NAMES armlink REQUIRED)
    find_program(ARM_FROMELF NAMES fromelf REQUIRED)

    set(CMAKE_C_COMPILER ${ARM_CC} CACHE STRING "")
    set(CMAKE_CXX_COMPILER ${ARM_CXX} CACHE STRING "")
    set(CMAKE_ASM_COMPILER ${ARM_ASM} CACHE STRING "")
    set(CMAKE_LINKER ${ARM_LINK} CACHE STRING "")

    set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
    set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
    set(CMAKE_FIND_ROOT_PATH ${TOOLCHAIN_PATH})

    set(LINK_MCU_CORE "Cortex-M4.fp.sp")

    set(C_FLAGS "--target=arm-arm-none-eabi -mcpu=cortex-m4+fp -mfloat-abi=hard -c -fno-rtti -funsigned-char -fshort-enums -fshort-wchar -D__MICROLIB -gdwarf-3 -O2 -fno-function-sections -Wno-packed -Wno-missing-variable-declarations -Wno-missing-prototypes -Wno-missing-noreturn -Wno-sign-conversion -Wno-nonportable-include-path -Wno-reserved-id-macro -Wno-unused-macros -Wno-documentation-unknown-command -Wno-documentation -Wno-license-management -Wno-parentheses-equality -D__MICROLIB -DMBEDTLS_CONFIG_FILE=mbedtls_config.h -Ic:/Keil_v5/ARM/ARMCLANG/include")
    set(CMAKE_C_FLAGS "-xc -std=c99  ${C_FLAGS}")
    set(CMAKE_CXX_FLAGS ${C_FLAGS})
    set(CMAKE_ASM_FLAGS "--target=arm-arm-none-eabi -mcpu=cortex-m4  -mfpu=fpv4-sp-d16 ")
    set(CMAKE_EXECUTABLE_SUFFIX ".axf")
endif()

# ---------------------------------------------------------------------
if(C_COMPILER_ID STREQUAL GCC)
    # Prevent CMake from testing the toolchain
    set(CMAKE_C_COMPILER_FORCED 1)
    set(CMAKE_CXX_COMPILER_FORCED 1)

    find_program(ARM_CC NAMES arm-none-eabi-gcc REQUIRED)
    find_program(ARM_CXX NAMES arm-none-eabi-gcc REQUIRED)
    find_program(ARM_OBJCOPY NAMES arm-none-eabi-objcopy REQUIRED)

    set(CMAKE_C_COMPILER ${ARM_CC} CACHE STRING "")
    set(CMAKE_CXX_COMPILER ${ARM_CXX} CACHE STRING "")

    set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
    set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
    set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)
    set(CMAKE_FIND_ROOT_PATH ${TOOLCHAIN_PATH})

    set(LINK_MCU_CORE "Cortex-M4.fp.sp")

    set(STACK_SIZE 0x4000)
    set(HEAP_SIZE 0x20000)

    set(C_FLAGS "-mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard --specs=nano.specs -Wall -Wno-format -Wno-return-type -Wno-unused-but-set-variable -fdata-sections -ffunction-sections -fno-builtin -fshort-enums -funsigned-char -mthumb -std=c99 -DMBEDTLS_CONFIG_FILE=mbedtls_config.h -I\"C:/Program Files (x86)/GNU Tools ARM Embedded/6 2017-q1-update/arm-none-eabi/include\"")
    set(CMAKE_C_FLAGS ${C_FLAGS})
    set(CMAKE_CXX_FLAGS ${C_FLAGS})
endif()

project(m460bsp)

function(create_project PRJ)
    add_executable(${PRJ} ${ARGN})
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
            COMMAND ${ARM_FROMELF} --bin --output ${PRJ}.bin ${PRJ}.elf
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
            COMMAND ${ARM_OBJCOPY} -O binary ${PRJ} ${PRJ}.bin
            VERBATIM
        )
    endif()
endfunction()

set(BSP_DIR ${CMAKE_CURRENT_LIST_DIR})

include_directories(${TOOLCHAIN_PATH}/include)
include_directories(${BSP_DIR}/Library/CMSIS/Include)
include_directories(${BSP_DIR}/Library/Device/Nuvoton/m460/Include)
include_directories(${BSP_DIR}/Library/StdDriver/inc)
include_directories(${BSP_DIR}/Library/CryptoAccelerator)
include_directories(${BSP_DIR}/Library/NuMaker/xmodem)
include_directories(${BSP_DIR}/ThirdParty/mbedtls-3.1.0/include)
include_directories(${BSP_DIR}/ThirdParty/mbedtls-3.1.0/library)
include_directories(${BSP_DIR}/ThirdParty/FatFs/source)
include_directories(${BSP_DIR}/ThirdParty/LibMAD/inc)

# Build library
add_subdirectory(${BSP_DIR}/Library/StdDriver/src)
add_subdirectory(${BSP_DIR}/Library/Device/Nuvoton/m460/Source)
add_subdirectory(${BSP_DIR}/Library/CryptoAccelerator)
add_subdirectory(${BSP_DIR}/Library/NuMaker/xmodem)
add_subdirectory(${BSP_DIR}/ThirdParty/mbedtls-3.1.0/library)
add_subdirectory(${BSP_DIR}/ThirdParty/FatFs/source)
add_subdirectory(${BSP_DIR}/ThirdParty/LibMAD)

target_include_directories(mbedcrypto
    PUBLIC
    ${TOOLCHAIN_PATH}/include
)

# Library for mbedtls test
add_library(test_lib
    ${BSP_DIR}/ThirdParty/mbedtls-3.1.0/tests/src/helpers.c
    ${BSP_DIR}/ThirdParty/mbedtls-3.1.0/tests/src/random.c
    ${BSP_DIR}/ThirdParty/mbedtls-3.1.0/tests/src/psa_crypto_helpers.c
)
target_include_directories(test_lib
    PUBLIC
    ${BSP_DIR}/ThirdParty/mbedtls-3.1.0/tests/include
)

# Build sample code
add_subdirectory(${BSP_DIR}/SampleCode/CortexM4)
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver)
add_subdirectory(${BSP_DIR}/SampleCode/ISP)
add_subdirectory(${BSP_DIR}/SampleCode/PowerManagement)
add_subdirectory(${BSP_DIR}/SampleCode/NuMaker_M467HJ)
create_project(Hard_Fault_Sample ${BSP_DIR}/SampleCode/Hard_Fault_Sample/main.c)
create_project(Template ${BSP_DIR}/SampleCode/Template/main.c)
