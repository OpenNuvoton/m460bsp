cmake_minimum_required(VERSION 3.21)
cmake_policy(SET CMP0123 NEW)


# Create a symbolic link from ${base_name} in the binary directory
# to the corresponding path in the source directory.
function(link_to_source base_name)
    # Get OS dependent path to use in `execute_process`
    if (CMAKE_HOST_WIN32)
        #mklink is an internal command of cmd.exe it can only work with \
        string(REPLACE "/" "\\" link "${CMAKE_CURRENT_BINARY_DIR}/${base_name}")
        string(REPLACE "/" "\\" target "${CMAKE_CURRENT_SOURCE_DIR}/${base_name}")
    else()
        set(link "${CMAKE_CURRENT_BINARY_DIR}/${base_name}")
        set(target "${CMAKE_CURRENT_SOURCE_DIR}/${base_name}")
    endif()

    if (NOT EXISTS ${link})
        if (CMAKE_HOST_UNIX)
            set(command ln -s ${target} ${link})
        else()
            if (IS_DIRECTORY ${target})
                set(command cmd.exe /c mklink /j ${link} ${target})
            else()
                set(command cmd.exe /c mklink /h ${link} ${target})
            endif()
        endif()

        execute_process(COMMAND ${command}
            RESULT_VARIABLE result
            ERROR_VARIABLE output)

        if (NOT ${result} EQUAL 0)
            message(FATAL_ERROR "Could not create symbolic link for: ${target} --> ${output}")
        endif()
    endif()
endfunction(link_to_source)

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION "5")
set(CMAKE_SYSTEM_PROCESSOR ARM)

set(TOOLCHAIN_PATH C:/Keil_v5/ARM/ARMCLANG)
find_program(ARM_CC NAMES armclang  PATHS ${TOOLCHAIN_PATH}/bin REQUIRED NO_DEFAULT_PATH)
find_program(ARM_CXX NAMES armclang  PATHS ${TOOLCHAIN_PATH}/bin REQUIRED NO_DEFAULT_PATH)
find_program(ARM_ASM NAMES armasm PATHS ${TOOLCHAIN_PATH}/bin REQUIRED NO_DEFAULT_PATH)
find_program(ARM_AR NAMES armar PATHS ${TOOLCHAIN_PATH}/bin REQUIRED NO_DEFAULT_PATH)
find_program(ARM_LINK NAMES armlink PATHS ${TOOLCHAIN_PATH}/bin REQUIRED NO_DEFAULT_PATH)
find_program(ARM_FROMELF NAMES fromelf PATHS ${TOOLCHAIN_PATH}/bin REQUIRED NO_DEFAULT_PATH)

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

set(STACK_SIZE 0x4000)
set(HEAP_SIZE 0x20000)

#SET(CMAKE_C_FLAGS    "-mcpu=cortex-m4 -Wno-ignored-optimization-argument -Wno-unused-command-line-argument -Wall    -Wno-error=cpp    -c    -fdata-sections    -ffunction-sections        -fshort-enums    -fshort-wchar -funsigned-char -masm=auto -nostdlib -mfpu=none -g -std=c99 -D__MICROLIB")
#SET(CMAKE_CXX_FLAGS  "-mcpu=cortex-m4 -Wno-ignored-optimization-argument -Wno-unused-command-line-argument -Wall    -Wno-error=cpp    -c    -fdata-sections    -ffunction-sections        -fshort-enums    -fshort-wchar -funsigned-char -masm=auto -nostdlib -mfpu=none -g -D__MICROLIB")
set(C_FLAGS "--target=arm-arm-none-eabi -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -c -fno-rtti -funsigned-char -fshort-enums -fshort-wchar -D__MICROLIB -gdwarf-3 -O2 -fno-function-sections -Wno-packed -Wno-missing-variable-declarations -Wno-missing-prototypes -Wno-missing-noreturn -Wno-sign-conversion -Wno-nonportable-include-path -Wno-reserved-id-macro -Wno-unused-macros -Wno-documentation-unknown-command -Wno-documentation -Wno-license-management -Wno-parentheses-equality -D__MICROLIB -DMBEDTLS_CONFIG_FILE=mbedtls_config.h -Ic:/Keil_v5/ARM/ARMCLANG/include")
set(CMAKE_C_FLAGS "-xc -std=c99  ${C_FLAGS}")
set(CMAKE_CXX_FLAGS ${C_FLAGS})
set(CMAKE_ASM_FLAGS "--cpu Cortex-M4.fp.sp -g --16 --apcs=interwork --pd \"__MICROLIB SETA 1\" --pd \"Stack_Size SETA ${STACK_SIZE}\" --pd \"Heap_Size SETA ${HEAP_SIZE}\"" )
set(CMAKE_EXECUTABLE_SUFFIX ".axf")


project(m460bsp)

set(BSP_DIR  ${CMAKE_CURRENT_LIST_DIR})

include_directories(${TOOLCHAIN_PATH}/include)
include_directories(${BSP_DIR}/Library/CMSIS/Include)
include_directories(${BSP_DIR}/Library/Device/Nuvoton/m460/Include)
include_directories(${BSP_DIR}/Library/StdDriver/inc)
include_directories(${BSP_DIR}/Library/CryptoAccelerator)
include_directories(${BSP_DIR}/ThirdParty/mbedtls-3.1.0/include)
include_directories(${BSP_DIR}/ThirdParty/mbedtls-3.1.0/library)

# Build library
add_subdirectory(${BSP_DIR}/Library/StdDriver/src)
add_subdirectory(${BSP_DIR}/Library/Device/Nuvoton/m460/Source)
add_subdirectory(${BSP_DIR}/Library/CryptoAccelerator)
add_subdirectory(${BSP_DIR}/ThirdParty/mbedtls-3.1.0/library)

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

#add_subdirectory(${BSP_DIR}/SampleCode/Crypto/mbedTLS_SHA256)
#add_subdirectory(${BSP_DIR}/SampleCode/Crypto/mbedTLS_RSA)
#add_subdirectory(${BSP_DIR}/SampleCode/Crypto/mbedTLS_ECDSA)
#add_subdirectory(${BSP_DIR}/SampleCode/Crypto/mbedTLS_ECDH)
#add_subdirectory(${BSP_DIR}/SampleCode/Crypto/mbedTLS_AES)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/TIMER_Periodic)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/ACMP_CompareDAC)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/ACMP_CompareVBG)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/ACMP_Wakeup)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/ACMP_WindowCompare)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/KS_ECDH)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/KS_ECDSA)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/RNG_Random)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/RNG_EntropyPoll)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/EADC_Accumulate)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/EADC_ADINT_Trigger)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/EADC_Average)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/EADC_BandGap)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/EADC_EPWM_Trigger)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/EADC_PDMA_EPWM_Trigger)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/EADC_Pending_Priority)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/EADC_ResultMonitor)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/EADC_SWTRG_Trigger)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/EADC_TempSensor)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/EADC_Timer_Trigger)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/EADC_VBat)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/DAC_EPWMTrigger)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/DAC_ExtPinTrigger)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/DAC_GroupMode)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/DAC_PDMA_EPWMTrigger)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/DAC_PDMA_TimerTrigger)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/DAC_SoftwareTrigger)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/DAC_TimerTrigger)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/HBI_ExeInHRAM)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/HBI_RW)
#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/HBI_RW_MemMap)
#add_subdirectory(${BSP_DIR}/SampleCode/NuMaker_M467HJ/TemperatureSensor)

#add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/CRYPTO_ECC_Demo)
add_subdirectory(${BSP_DIR}/SampleCode/Crypto/mbedTLS_AES)






