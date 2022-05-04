enable_language(C ASM)
add_library(system_startup
    system_m460.c
    ARM/startup_m460.s
)
