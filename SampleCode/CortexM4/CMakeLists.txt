create_project(BitBand ${CMAKE_CURRENT_LIST_DIR}/BitBand/main.c)

create_project(DSP_FFT ${CMAKE_CURRENT_LIST_DIR}/DSP_FFT/main.c ${CMAKE_CURRENT_LIST_DIR}/DSP_FFT/arm_fft_bin_data.c)

if(C_COMPILER_ID STREQUAL GCC)
    target_link_libraries(DSP_FFT ${BSP_DIR}/Library/CMSIS/Lib/GCC/libarm_cortexM4lf_math.a)
else()
    target_link_libraries(DSP_FFT ${BSP_DIR}/Library/CMSIS/Lib/ARM/arm_cortexM4lf_math.lib)
endif()

target_compile_definitions(DSP_FFT PRIVATE -DARM_MATH_CM4)

create_project(MPU ${CMAKE_CURRENT_LIST_DIR}/MPU/main.c)