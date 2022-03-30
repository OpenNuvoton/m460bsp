
#set_property(SOURCE ARM/startup_m460.s PROPERTY LANGUAGE ASM)

set(can_use_assembler TRUE)
enable_language(C ASM)
add_library(m460_startup 
    system_m460.c 
    ARM/startup_m460.s
)

target_include_directories(m460_startup
    PUBLIC
    ${BSP_DIR}/Library/CMSIS/Include
    ${BSP_DIR}/Library/Device/Nuvoton/m460/Include
    ${BSP_DIR}/Library/StdDriver/inc
)