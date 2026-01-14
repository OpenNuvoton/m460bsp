#include "sfud.h"

// *** <<< Use Configuration Wizard in Context Menu >>> ***
// <c1> Enable SPIM DMA Read
// <i> Enable SPIM DMA Read
//#define ENABLE_SPIM_DMA_READ
// </c>
// <c1> ENABLE_THROUGHPUT_OPTIMIZE
// <i> Use FIFO mechanism and enable PDMA for QSPI RX to maximize throughput
#define ENABLE_THROUGHPUT_OPTIMIZE
// </c>
// *** <<< end of configuration section >>> ***
#pragma once

enum
{
    SFUD_W25Q16BV_DEVICE_INDEX = 0,
    SFUD_W25X16VS_DEVICE_INDEX = 1,
    SFUD_W25Q32JV_DEVICE_INDEX = 2,
};

#define SFUD_FLASH_DEVICE_TABLE                                                \
{                                                                              \
    [SFUD_W25Q16BV_DEVICE_INDEX] = {.name = "W25Q16BVSSIG", .spi.name = "SPI0"},            \
    [SFUD_W25X16VS_DEVICE_INDEX] = {.name = "W25X16JVSSIG", .spi.name = "QSPI0"},           \
    [SFUD_W25Q32JV_DEVICE_INDEX] = {.name = "W25Q32JVSSIQ", .spi.name = "SPIM"},            \
}
