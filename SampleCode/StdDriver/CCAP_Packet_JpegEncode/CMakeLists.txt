

set(PRJ CCAP_Packet_JpegEncode)
set(LIBJPEG_SRC
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jdapimin.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jaricom.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jcapistd.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jcarith.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jccoefct.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jccolor.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jcdctmgr.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jchuff.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jcinit.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jcmainct.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jcmarker.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jcmaster.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jcomapi.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jcparam.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jcprepct.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jcsample.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jctrans.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jcapimin.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jdapistd.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jdarith.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jdatadst.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jdatasrc.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jdcoefct.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jdcolor.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jddctmgr.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jdhuff.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jdinput.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jdmainct.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jdmarker.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jdmaster.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jdmerge.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jdpostct.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jdsample.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jdtrans.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jfdctflt.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jfdctfst.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jfdctint.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jidctflt.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jidctfst.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jidctint.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jmemansi.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jquant1.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jquant2.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jutils.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/rdbmp.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/rdcolmap.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/rdgif.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/rdppm.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/rdrle.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/rdswitch.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/rdtarga.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/transupp.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/wrbmp.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/wrgif.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/wrppm.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/wrrle.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jerror.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/wrtarga.c
    ${CMAKE_CURRENT_LIST_DIR}/../../../ThirdParty/libjpeg/jmemmgr.c
)

add_executable(${PRJ} main.c i2c_gpio.c jpeg.c Sensor_nt99141.c ${LIBJPEG_SRC})

target_link_libraries(${PRJ} system_startup standard_driver)
target_include_directories(${PRJ} 
    PUBLIC
        .
)

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

