/***************************************************************************//**
 * @file     usbd_audio.h
 * @version  V3.00
 * @brief    HSUSBD audio codec header file.
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __USBD_UAC_H__
#define __USBD_UAC_H__

#include "config.h"

#define NAU8822     1
//#define __FEEDBACK__

/* Use LIN as source, undefine it if MIC is used */
//#define INPUT_IS_LIN

/* Define the vendor id and product id */
#define USBD_VID        0x0416

#ifndef __HID__
#ifdef __FEEDBACK__
#define USBD_PID        0xC120
#else
#define USBD_PID        0xC020
#endif
#else
#ifdef __JOYSTICK__
#ifdef __FEEDBACK__
#define USBD_PID        0xC121
#else
#define USBD_PID        0xC021
#endif
#elif defined __MEDIAKEY__
#ifdef __FEEDBACK__
#define USBD_PID        0xC122
#else
#define USBD_PID        0xC022
#endif
#endif
#endif

#define AUDIO_RATE  AUDIO_RATE_48K

#define AUDIO_RATE_8K      8000
#define AUDIO_RATE_16K    16000
#define AUDIO_RATE_24K    24000
#define AUDIO_RATE_32K    32000
#define AUDIO_RATE_48K    48000       /* The audo play sampling rate. The setting is 48KHz */
#define AUDIO_RATE_96K    96000       /* The audo play sampling rate. The setting is 96KHz */
#define AUDIO_RATE_441K   44100       /* The audo play sampling rate. The setting is 44.1KHz */

/*!<Define Audio information */
#define PLAY_RATE       AUDIO_RATE   /* The play sampling rate */
#define PLAY_CHANNELS   2
#define PLAY_BIT_RATE   0x10    /* 16-bit data rate */

#define REC_RATE        AUDIO_RATE   /* The record sampling rate */
#define REC_CHANNELS    2
#define REC_BIT_RATE    0x10    /* 16-bit data rate */

#define REC_FEATURE_UNITID      0x05
#define PLAY_FEATURE_UNITID     0x06

#define BUFF_LEN    800

/* Define Descriptor information */
#if(PLAY_CHANNELS == 1)
#define PLAY_CH_CFG     1
#endif
#if(PLAY_CHANNELS == 2)
#define PLAY_CH_CFG     3
#endif

#if(REC_CHANNELS == 1)
#define REC_CH_CFG     1
#endif
#if(REC_CHANNELS == 2)
#define REC_CH_CFG     3
#endif

/********************************************/
/* Audio Class Current State                */
/********************************************/
/*!<Define Audio Class Current State */
#define UAC_STOP_AUDIO_RECORD           0
#define UAC_START_AUDIO_RECORD          1
#define UAC_PROCESSING_AUDIO_RECORD     2
#define UAC_BUSY_AUDIO_RECORD           3

/***************************************************/
/*      Audio Class-Specific Request Codes         */
/***************************************************/
/*!<Define Audio Class Specific Request */
#define UAC_REQUEST_CODE_UNDEFINED  0x00
#define UAC_SET_CUR                 0x01
#define UAC_GET_CUR                 0x81
#define UAC_SET_MIN                 0x02
#define UAC_GET_MIN                 0x82
#define UAC_SET_MAX                 0x03
#define UAC_GET_MAX                 0x83
#define UAC_SET_RES                 0x04
#define UAC_GET_RES                 0x84
#define UAC_SET_MEM                 0x05
#define UAC_GET_MEM                 0x85
#define UAC_GET_STAT                0xFF

#define MUTE_CONTROL                0x01
#define VOLUME_CONTROL              0x02

/*!<Define HID Class Specific Request */
#define GET_REPORT              0x01
#define GET_IDLE                0x02
#define GET_PROTOCOL            0x03
#define SET_REPORT              0x09
#define SET_IDLE                0x0A
#define SET_PROTOCOL            0x0B

#ifdef __HID__
#ifdef __MEDIAKEY__
/* Byte 0 */
#define HID_CTRL_MUTE        0x01
#define HID_CTRL_VOLUME_INC  0x02
#define HID_CTRL_VOLUME_DEC  0x04
/* Byte 1 */
#define HID_CTRL_PLAY        0x01
#define HID_CTRL_STOP        0x02
#define HID_CTRL_PAUSE       0x04
#define HID_CTRL_NEXT        0x08
#define HID_CTRL_PREVIOUS    0x10
#define HID_CTRL_RECORD      0x20
#define HID_CTRL_REWIND      0x40
#define HID_CTRL_FF          0x80
#endif
#endif

/*-------------------------------------------------------------*/
/* Define EP maximum packet size */
#define CEP_MAX_PKT_SIZE        64
#define CEP_OTHER_MAX_PKT_SIZE  64

/* Maximum Packet Size for Recprd Endpoint */
#define EPA_MAX_PKT_SIZE        (REC_RATE * REC_CHANNELS * 2 / 1000)
#define EPA_OTHER_MAX_PKT_SIZE  (REC_RATE * REC_CHANNELS * 2 / 1000)
#ifdef __FEEDBACK__
/* Maximum Packet Size for Play Endpoint */
#define EPB_MAX_PKT_SIZE        ((AUDIO_RATE_96K * PLAY_CHANNELS * 2 / 1000) + 64)
#define EPB_OTHER_MAX_PKT_SIZE  ((AUDIO_RATE_96K * PLAY_CHANNELS * 2 / 1000) + 64)
#else
#define EPB_MAX_PKT_SIZE        (AUDIO_RATE_96K * PLAY_CHANNELS * 2 / 1000)
#define EPB_OTHER_MAX_PKT_SIZE  (AUDIO_RATE_96K * PLAY_CHANNELS * 2 / 1000)
#endif
/* Maximum Packet Size for Feedback Endpoint */
#define EPC_MAX_PKT_SIZE        8
#define EPC_OTHER_MAX_PKT_SIZE  8
/* Maximum Packet Size for HID Endpoint */
#define EPD_MAX_PKT_SIZE        8
#define EPD_OTHER_MAX_PKT_SIZE  8
/* Maximum Packet Size for HID Endpoint */
#define EPE_MAX_PKT_SIZE        8
#define EPE_OTHER_MAX_PKT_SIZE  8


#define CEP_BUF_BASE    0
#define CEP_BUF_LEN     CEP_MAX_PKT_SIZE
#define EPA_BUF_BASE    0x100
#define EPA_BUF_LEN     EPA_MAX_PKT_SIZE
#define EPB_BUF_BASE    0x300
#define EPB_BUF_LEN     EPB_MAX_PKT_SIZE
#define EPC_BUF_BASE    0x500
#define EPC_BUF_LEN     EPC_MAX_PKT_SIZE
#define EPD_BUF_BASE    0x600
#define EPD_BUF_LEN     EPD_MAX_PKT_SIZE
#define EPE_BUF_BASE    0x700
#define EPE_BUF_LEN     EPE_MAX_PKT_SIZE

/* Define the interrupt In EP number */
#define ISO_IN_EP_NUM       0x01
#define ISO_OUT_EP_NUM      0x02
#define ISO_IN_FB_EP_NUM    0x03
#define HID_IN_EP_NUM       0x04
#define HID_OUT_EP_NUM      0x05

#define PDMA_TXBUFFER_CNT     7
#define PDMA_RXBUFFER_CNT     8

#define PDMA_I2S_TX_CH  1
#define PDMA_I2S_RX_CH  2

typedef enum
{
    E_RS_REC_CH0,          /* Record Channel 0 */
    E_RS_REC_CH1,          /* Record Channel 1 */
    E_RS_PLAY_CH0,         /* Play Channel 0 */
    E_RS_PLAY_CH1          /* Play Channel 1 */
} RESAMPLE_MODE_T;


typedef struct
{
    int r;
    int s;
    int d;
    int count;
} SR_T;

/* For I2C transfer */
typedef enum
{
    E_RS_NONE,          // no re-sampling
    E_RS_UP,            // up sampling
    E_RS_DOWN           // down sampling
} RESAMPLE_STATE_T;


/*-------------------------------------------------------------*/
extern volatile uint32_t u32BuffLen, u32RxBuffLen;
extern volatile uint32_t g_usbd_UsbAudioState;
extern volatile uint8_t u8AudioPlaying;
extern volatile uint32_t g_usbd_SampleRate;
extern volatile uint32_t g_usbd_PlaySampleRate;
extern volatile uint32_t g_usbd_RecSampleRate;
extern volatile uint32_t g_PrePlaySampleRate;
extern volatile uint32_t g_PreRecSampleRate;

extern uint32_t PcmPlayBuff[PDMA_TXBUFFER_CNT][BUFF_LEN];
extern uint8_t PcmRecBuff[PDMA_RXBUFFER_CNT][BUFF_LEN];
extern uint8_t u8PcmRxBufFull[PDMA_RXBUFFER_CNT];
extern volatile uint8_t u8TxDataCntInBuffer;
extern volatile uint8_t u8RxDataCntInBuffer;
extern volatile uint8_t u8PDMATxIdx;
extern volatile uint8_t u8PDMARxIdx;

extern volatile uint32_t g_play_max_packet_size;
extern volatile uint32_t g_rec_max_packet_size;
extern volatile uint32_t g_resample_play_s_idx;
extern volatile uint32_t g_resample_rec_s_idx;
extern const SR_T srt[];
extern volatile uint8_t g_u8EPDReady;


void UAC_DeviceEnable(uint32_t bIsPlay);
void UAC_DeviceDisable(uint32_t bIsPlay);
void UAC_GetPlayData(void);
void UAC_SendRecData(void);

void AudioStartPlay(uint32_t u32SampleRate);
void AudioStartRecord(uint32_t u32SampleRate);
/*-------------------------------------------------------------*/
void UAC_Init(void);
void UAC_ClassRequest(void);
void UAC_SetInterface(uint32_t u32AltInterface);

void EPA_Handler(void);
void EPB_Handler(void);
void EPC_Handler(void);
void EPD_Handler(void);
void EPE_Handler(void);
void AdjustCodecPll(RESAMPLE_STATE_T r);
#if NAU8822
void NAU8822_Setup(void);
void NAU8822_ConfigSampleRate(uint32_t u32SampleRate);
#else
void NAU88L25_Reset(void);
void NAU88L25_Setup(void);
void NAU88L25_ConfigSampleRate(uint32_t u32SampleRate);
#endif

typedef struct dma_desc_t
{
    uint32_t ctl;
    uint32_t src;
    uint32_t dest;
    uint32_t offset;
} DMA_DESC_T;

extern volatile uint8_t g_usbd_txflag;
extern volatile uint8_t g_usbd_rxflag;
extern void PDMA_Init(void);
extern void PDMA_WriteTxSGTable(void);
extern void PDMA_ResetRxSGTable(uint8_t id);
extern void PDMA_WriteRxSGTable(void);

extern int Resamples(RESAMPLE_MODE_T mode, short *x, int ch_num, int samples, short *y, int s_idx);
extern void HID_UpdateHidData(void);
extern void GPIO_Init(void);

#endif  /* __USBD_UAC_H_ */
