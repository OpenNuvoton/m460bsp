cmake_minimum_required(VERSION 3.21)
cmake_policy(SET CMP0123 NEW)


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

#SET(CMAKE_C_FLAGS    "-mcpu=cortex-m4 -Wno-ignored-optimization-argument -Wno-unused-command-line-argument -Wall    -Wno-error=cpp    -c    -fdata-sections    -ffunction-sections        -fshort-enums    -fshort-wchar -funsigned-char -masm=auto -nostdlib -mfpu=none -g -std=c99 -D__MICROLIB")
#SET(CMAKE_CXX_FLAGS  "-mcpu=cortex-m4 -Wno-ignored-optimization-argument -Wno-unused-command-line-argument -Wall    -Wno-error=cpp    -c    -fdata-sections    -ffunction-sections        -fshort-enums    -fshort-wchar -funsigned-char -masm=auto -nostdlib -mfpu=none -g -D__MICROLIB")
set(C_FLAGS "--target=arm-arm-none-eabi -mcpu=cortex-m4 -mfpu=fpv4-sp-d16 -mfloat-abi=hard -c -fno-rtti -funsigned-char -fshort-enums -fshort-wchar -D__MICROLIB -gdwarf-3 -O2 -fno-function-sections -Wno-packed -Wno-missing-variable-declarations -Wno-missing-prototypes -Wno-missing-noreturn -Wno-sign-conversion -Wno-nonportable-include-path -Wno-reserved-id-macro -Wno-unused-macros -Wno-documentation-unknown-command -Wno-documentation -Wno-license-management -Wno-parentheses-equality -D__MICROLIB")
set(CMAKE_C_FLAGS "-xc -std=c99  ${C_FLAGS}")
set(CMAKE_CXX_FLAGS ${C_FLAGS})
set(CMAKE_ASM_FLAGS "--cpu Cortex-M4.fp.sp -g --16 --apcs=interwork --pd \"__MICROLIB SETA 1\"" )
set(CMAKE_EXECUTABLE_SUFFIX ".axf")


project(m460bsp)

set(BSP_DIR  ${CMAKE_CURRENT_LIST_DIR})

include_directories(${TOOLCHAIN_PATH}/include)
include_directories(${BSP_DIR}/Library/CMSIS/Include)
include_directories(${BSP_DIR}/Library/Device/Nuvoton/m460/Include)
include_directories(${BSP_DIR}/Library/StdDriver/inc)

# Build library 
add_subdirectory(${BSP_DIR}/Library/StdDriver/src)
add_subdirectory(${BSP_DIR}/Library/Device/Nuvoton/m460/Source)

# Download to board
set(DOWNLOAD_BOARD FALSE)

# Build sample code
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/TIMER_Periodic)
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/ACMP_CompareDAC)
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/ACMP_CompareVBG)
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/ACMP_Wakeup)
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/ACMP_WindowCompare)
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/KS_ECDH)
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/KS_ECDSA)
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/RNG_Random)
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/RNG_EntropyPoll)
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/EADC_Accumulate)
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/EADC_ADINT_Trigger)
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/EADC_Average)
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/EADC_BandGap)
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/EADC_EPWM_Trigger)
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/EADC_PDMA_EPWM_Trigger)
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/EADC_Pending_Priority)
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/EADC_ResultMonitor)
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/EADC_SWTRG_Trigger)
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/EADC_TempSensor)
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/EADC_Timer_Trigger)
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/EADC_VBat)
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/DAC_EPWMTrigger)
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/DAC_ExtPinTrigger)
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/DAC_GroupMode)
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/DAC_PDMA_EPWMTrigger)
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/DAC_PDMA_TimerTrigger)
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/DAC_SoftwareTrigger)
add_subdirectory(${BSP_DIR}/SampleCode/StdDriver/DAC_TimerTrigger)





