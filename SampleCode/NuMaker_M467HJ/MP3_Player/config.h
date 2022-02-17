/**************************************************************************//**
 * @file     config.h
 * @version  V3.00
 * @brief    I2S mp3 player sample configuration header file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef PROJECT_CONFIG_H
#define PROJECT_CONFIG_H

#define NAU8822     1

/*---------------------------------------------------------------------------------------------------------*/
/* Global variables                                                                                        */
/*---------------------------------------------------------------------------------------------------------*/
#define USE_SDH
//#define USE_USBH

#define PCM_BUFFER_SIZE        2304
#define FILE_IO_BUFFER_SIZE    4096

struct mp3Header
{
    unsigned int sync : 11;
    unsigned int version : 2;
    unsigned int layer : 2;
    unsigned int protect : 1;
    unsigned int bitrate : 4;
    unsigned int samfreq : 2;
    unsigned int padding : 1;
    unsigned int private : 1;
    unsigned int channel : 2;
    unsigned int mode : 2;
    unsigned int copy : 1;
    unsigned int original : 1;
    unsigned int emphasis : 2;
};

struct AudioInfoObject
{
    unsigned int playFileSize;
    unsigned int mp3FileEndFlag;
    unsigned int mp3SampleRate;
    unsigned int mp3BitRate;
    unsigned int mp3Channel;
    unsigned int mp3PlayTime;
    unsigned int mp3Playing;
};

typedef struct dma_desc_t
{
    uint32_t ctl;
    uint32_t src;
    uint32_t dest;
    uint32_t offset;
} DMA_DESC_T;


void PDMA_Reset_SCTable(uint8_t id);
void NAU8822_ConfigSampleRate(uint32_t u32SampleRate);
void NAU88L25_ConfigSampleRate(uint32_t u32SampleRate);

int mp3CountV1L3Headers(unsigned char *pBytes, size_t size);
extern void PDMA_Init(void);
extern void NAU8822_Setup(void);
extern void NAU88L25_Setup(void);
extern void NAU88L25_Reset(void);
extern void MP3Player(void);
extern volatile uint8_t u8PCMBuffer_Playing;

#endif
