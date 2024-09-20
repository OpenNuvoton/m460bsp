enable_language(C ASM)
add_library(system_startup
    system_m460.c
    $<$<STREQUAL:${C_COMPILER_ID},ARMCLANG>:ARM/startup_m460.S>
    $<$<STREQUAL:${C_COMPILER_ID},GCC>:GCC/_syscalls.c>
    $<$<STREQUAL:${C_COMPILER_ID},GCC>:GCC/startup_m460.S>
)
