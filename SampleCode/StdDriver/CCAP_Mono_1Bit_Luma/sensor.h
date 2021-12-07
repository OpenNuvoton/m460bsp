/**************************************************************************//**
 * @file     sensor.h
 * @version  V3.00
 * @brief    Sensor driver
 *
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2021 Nuvoton Technology Corp. All rights reserved.
 ******************************************************************************/
#ifndef __CCAP_SENSOR_H_
#define __CCAP_SENSOR_H_

#include "NuMicro.h"

int InitOV7725_VGA_YUV422(void);
#define OV7725SensorPolarity         (CCAP_PAR_VSP_HIGH | CCAP_PAR_HSP_LOW  | CCAP_PAR_PCLKP_HIGH)
#define OV7725DataFormatAndOrder (CCAP_PAR_INDATORD_UYVY | CCAP_PAR_INFMT_YUV422 | CCAP_PAR_OUTFMT_YUV422)

int InitNT99141_VGA_YUV422(void);
#define NT99141SensorPolarity         (CCAP_PAR_VSP_LOW | CCAP_PAR_HSP_LOW  | CCAP_PAR_PCLKP_HIGH)
#define NT99141DataFormatAndOrder (CCAP_PAR_INDATORD_YUYV | CCAP_PAR_INFMT_YUV422 | CCAP_PAR_OUTFMT_YUV422)

int InitHM01B0_4BIT_YUV422(void);
#define HM01B0SensorPolarity         (CCAP_PAR_VSP_LOW | CCAP_PAR_HSP_LOW  | CCAP_PAR_PCLKP_HIGH)
#define HM01B0DataFormatAndOrder (CCAP_PAR_INDATORD_VYUY | CCAP_PAR_INFMT_YUV422 | CCAP_PAR_OUTFMT_ONLY_Y)

#endif
