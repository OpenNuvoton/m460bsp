set(DRV_SRC
    src/bit.c
    src/decoder.c
    src/fixed.c
    src/frame.c
    src/huffman.c
    src/layer12.c
    src/layer3.c
    src/MP3Func.c
    src/stream.c
    src/synth.c
    src/timer.c
    src/version.c
)

add_library(libmad ${DRV_SRC})


target_compile_definitions(libmad PUBLIC __WINS__)