/**************************************************************************//**
 * @file     uvc_core.c
 * @version  V1.00
 * @brief    USB Host Video Class driver
 *
 * @note
 * @copyright SPDX-License-Identifier: Apache-2.0
 * @copyright Copyright (C) 2025 Nuvoton Technology Corp. All rights reserved.
*****************************************************************************/

#include <stdio.h>
#include <string.h>

#include "usbh_lib.h"
#include "usbh_uvc.h"

/** @addtogroup LIBRARY Library
  @{
*/

/** @addtogroup USBH_Library USBH Library
  @{
*/

/** @addtogroup USBH_EXPORTED_FUNCTIONS USBH Exported Functions
  @{
*/

/// @cond HIDDEN_SYMBOLS

static void  dump_parameter_block(UVC_CTRL_PARAM_T *param)
{
#ifdef UVC_DEBUG
    UVC_DBGMSG("\n\nbmHint          = 0x%x\n", param->bmHint);
    UVC_DBGMSG("bFormatIndex    = %d\n", param->bFormatIndex);
    UVC_DBGMSG("bFrameIndex     = %d\n", param->bFrameIndex);
    UVC_DBGMSG("dwFrameInterval = %d\n", param->dwFrameInterval);
    UVC_DBGMSG("wKeyFrameRate   = %d\n", param->wKeyFrameRate);
    UVC_DBGMSG("wPFrameRate     = %d\n", param->wPFrameRate);
    UVC_DBGMSG("dwMaxVideoFrameSize      = %d\n", param->dwMaxVideoFrameSize);
    UVC_DBGMSG("dwMaxPayloadTransferSize = %d\n", param->dwMaxPayloadTransferSize);
    UVC_DBGMSG("bmFramingInfo   = 0x%x\n", param->bmFramingInfo);
    UVC_DBGMSG("bUsage          = 0x%x\n", param->bUsage);
    UVC_DBGMSG("bmSettings      = 0x%x\n", param->bmSettings);
#else
    (void)(param);
#endif
}

static void  dump_still_parameter_block(UVC_STILL_CTRL_PARAM_T *param)
{
#ifdef UVC_DEBUG
    UVC_DBGMSG("\n\nbFormatIndex          = 0x%x\n", param->bFormatIndex);
    UVC_DBGMSG("bFrameIndex       = %d\n", param->bFrameIndex);
    UVC_DBGMSG("bCompressionIndex = %d\n", param->bCompressionIndex);
    UVC_DBGMSG("dwMaxVideoFrameSize      = %d\n", param->dwMaxVideoFrameSize);
    UVC_DBGMSG("dwMaxPayloadTransferSize = %d\n", param->dwMaxPayloadTransferSize);
#else
    (void)(param);
#endif
}

/**
 *  @brief  Video Class Request - Get Video Probe Control
 *  @param[in]  vdev   UVC device
 *  @param[in]  req    Control request.
 *                     UVC_SET_CUR
 *                     UVC_GET_CUR
 *                     UVC_GET_MIN
 *                     UVC_GET_MAX
 *                     UVC_GET_RES
 *                     UVC_GET_LEN
 *                     UVC_GET_INFO
 *                     UVC_GET_DEF
 *  @param[out] param  A set of shadow parameters from the UVC device.
 *  @return   Success or failed.
 *  @retval   0        Success
 *  @retval   Otheriwse  Error occurred
 */
int usbh_uvc_probe_control(UVC_DEV_T *vdev, uint8_t req, UVC_CTRL_PARAM_T *param)
{
    uint8_t     bmRequestType;
    uint32_t    xfer_len;
    int         ret;

    if(req & 0x80)
        bmRequestType = REQ_TYPE_IN | REQ_TYPE_CLASS_DEV | REQ_TYPE_TO_IFACE;
    else
        bmRequestType = REQ_TYPE_OUT | REQ_TYPE_CLASS_DEV | REQ_TYPE_TO_IFACE;

    ret = usbh_ctrl_xfer(vdev->udev, bmRequestType, req,
                         (VS_PROBE_CONTROL << 8),    /* wValue - Control Selector (CS)    */
                         vdev->iface_stream->if_num, /* wIndex - Zero and Interface       */
                         sizeof(UVC_CTRL_PARAM_T),   /* wLength - Length of parameter block */
                         (uint8_t *)param,           /* parameter block                   */
                         &xfer_len, UVC_REQ_TIMEOUT);

    if(ret != 0)
    {
        UVC_DBGMSG("usbh_uvc_probe_control incorrect transfer length! %d %d\n", ret, xfer_len);
    }
    else if(req == UVC_GET_CUR)
    {
        dump_parameter_block(param);
    }

    return ret;
}

/**
 *  @brief  Video Class Request - Set Video Commit Control
 *  @param[in]  vdev   UVC device
 *  @param[out] param  A set of shadow parameters from the UVC device.
 *  @return   Success or failed.
 *  @retval   0        Success
 *  @retval   Otheriwse  Error occurred
 */
static int usbh_uvc_commit_control(UVC_DEV_T *vdev, UVC_CTRL_PARAM_T *param)
{
    uint8_t     bmRequestType;
    uint32_t    xfer_len;

    bmRequestType = REQ_TYPE_OUT | REQ_TYPE_CLASS_DEV | REQ_TYPE_TO_IFACE;

    return usbh_ctrl_xfer(vdev->udev, bmRequestType, UVC_SET_CUR,
                          (VS_COMMIT_CONTROL << 8),   /* wValue - Control Selector (CS)    */
                          vdev->iface_stream->if_num, /* wIndex - Zero and Interface       */
                          sizeof(UVC_CTRL_PARAM_T),   /* wLength - Length of parameter block */
                          (uint8_t *)param,           /* parameter block                   */
                          &xfer_len, UVC_REQ_TIMEOUT);
}

/**
 *  @brief  Video Class Request - Get Video Still Probe Control
 *  @param[in]  vdev   UVC device
 *  @param[in]  req    Control request.
 *                     UVC_SET_CUR
 *                     UVC_GET_CUR
 *                     UVC_GET_MIN
 *                     UVC_GET_MAX
 *                     UVC_GET_LEN
 *                     UVC_GET_INFO
 *                     UVC_GET_DEF
 *  @param[out] param  A set of shadow parameters from the UVC device.
 *  @return   Success or failed.
 *  @retval   0        Success
 *  @retval   Otheriwse  Error occurred
 */
int usbh_uvc_still_probe_control(UVC_DEV_T *vdev, uint8_t req, UVC_STILL_CTRL_PARAM_T *param)
{
    uint8_t     bmRequestType;
    uint32_t    xfer_len;
    int         ret;

    if(req & 0x80)
        bmRequestType = REQ_TYPE_IN | REQ_TYPE_CLASS_DEV | REQ_TYPE_TO_IFACE;
    else
        bmRequestType = REQ_TYPE_OUT | REQ_TYPE_CLASS_DEV | REQ_TYPE_TO_IFACE;

    ret = usbh_ctrl_xfer(vdev->udev, bmRequestType, req,
                         (VS_STILL_PROBE_CONTROL << 8), /* wValue - Control Selector (CS)    */
                         vdev->iface_stream->if_num,    /* wIndex - Zero and Interface       */
                         sizeof(UVC_STILL_CTRL_PARAM_T),/* wLength - Length of parameter block */
                         (uint8_t *)param,              /* parameter block                   */
                         &xfer_len, UVC_REQ_TIMEOUT);

    if(ret != 0)
    {
        UVC_DBGMSG("usbh_uvc_still_probe_control incorrect transfer length! %d %d\n", ret, xfer_len);
    }
    else if(req == UVC_GET_CUR)
    {
        dump_still_parameter_block(param);
    }

    return ret;
}

/**
 *  @brief  Video Class Request - Set Video Still Commit Control
 *  @param[in]  vdev   UVC device
 *  @param[out] param  A set of shadow parameters from the UVC device.
 *  @return   Success or failed.
 *  @retval   0        Success
 *  @retval   Otheriwse  Error occurred
 */
static int usbh_uvc_still_commit_control(UVC_DEV_T *vdev, UVC_STILL_CTRL_PARAM_T *param)
{
    uint8_t     bmRequestType;
    uint32_t    xfer_len;

    bmRequestType = REQ_TYPE_OUT | REQ_TYPE_CLASS_DEV | REQ_TYPE_TO_IFACE;

    return usbh_ctrl_xfer(vdev->udev, bmRequestType, UVC_SET_CUR,
                          (VS_STILL_COMMIT_CONTROL << 8), /* wValue - Control Selector (CS)    */
                          vdev->iface_stream->if_num,     /* wIndex - Zero and Interface       */
                          sizeof(UVC_STILL_CTRL_PARAM_T), /* wLength - Length of parameter block */
                          (uint8_t *)param,               /* parameter block                   */
                          &xfer_len, UVC_REQ_TIMEOUT);
}

int usbh_uvc_still_image_trigger_control(UVC_DEV_T *vdev, uint8_t capture)
{
    uint8_t     bmRequestType;
    uint32_t    xfer_len;
    uint8_t     *bptr;
    int         ret;

    bmRequestType = REQ_TYPE_OUT | REQ_TYPE_CLASS_DEV | REQ_TYPE_TO_IFACE;

    if(vdev == NULL)
        return UVC_RET_DEV_NOT_FOUND;

    bptr = usbh_alloc_mem(4);
    if(bptr == NULL)
        return USBH_ERR_MEMORY_OUT;

    *bptr = capture;

    ret = usbh_ctrl_xfer(vdev->udev, bmRequestType, UVC_SET_CUR,
                          (VS_STILL_IMAGE_TRIGGER_CONTROL << 8),   /* wValue - Control Selector (CS)    */
                          vdev->iface_stream->if_num,              /* wIndex - Zero and Interface       */
                          1,                                       /* wLength - Length of the data phase*/
                          bptr,                                    /* Trigger capture                   */
                          &xfer_len, UVC_REQ_TIMEOUT);

    usbh_free_mem(bptr, 4);
    return ret;
}

/**
 * @brief  Check if the UVC device supports still image capture.
 * @param[in]  vdev    Pointer to the UVC device structure.
 * @return     1 if supported, 0 if not supported.
 */
int uvc_supports_still_image(void)
{
    /* Checking for still image support requires a valid UVC device context
     * (bTriggerSupport field from the parsed VS_INPUT_HEADER descriptor).
     * The device context is not available here; return -1 (not supported).
     */
    printf("Device does not support still image capture.\n");
    return -1; /* Not supported / not implemented */
}

/**
 * @brief  Get the length of Still Probe/Commit data block using GET_LEN request.
 * @param[in]  vdev    Pointer to the UVC device structure.
 * @param[in]  iface   Video Streaming interface number.
 * @param[in]  control_selector  Control selector (e.g., VS_PROBE_CONTROL or VS_COMMIT_CONTROL).
 * @return     Length of the data block on success, negative value on error.
 */
int uvc_get_still_data_length(UVC_DEV_T *vdev, uint8_t control_selector)
{
    uint8_t bmRequestType = 0xA1; // Device-to-Host, Class, Interface
    uint8_t bRequest = UVC_GET_LEN; // GET_LEN request
    uint16_t wValue = (control_selector << 8); // Control selector in high byte
    uint16_t wIndex = vdev->iface_stream->if_num; // Interface number
    uint16_t wLength = 2; // Length of the data phase (2 bytes)
    uint8_t *data; // Buffer to store the returned length
    int ret;
    uint32_t xfer_len;

    data = usbh_alloc_mem(4);
    if(data == NULL) return USBH_ERR_MEMORY_OUT;

    // Send control transfer
    ret = usbh_ctrl_xfer(vdev->udev, bmRequestType, bRequest, wValue, wIndex, wLength, data, &xfer_len, UVC_REQ_TIMEOUT);

    if (ret == 0) // Success
    {
        // length is a 16-bit value in little-endian format
        ret = data[0] | (data[1] << 8);
    }

    usbh_free_mem(data, 4);

    return ret;
}

/**
 * @brief  Compare Still Probe and Commit data block lengths.
 * @param[in]  vdev    Pointer to the UVC device structure.
 * @param[in]  iface   Video Streaming interface number.
 */
void compare_still_probe_commit_lengths(UVC_DEV_T *vdev)
{
    int probe_length = uvc_get_still_data_length(vdev, VS_STILL_PROBE_CONTROL);
    int commit_length = uvc_get_still_data_length(vdev, VS_STILL_COMMIT_CONTROL);

    if (probe_length < 0 || commit_length < 0)
    {
        printf("Error reading lengths.\n");
        return;
    }

    if (probe_length == commit_length)
    {
        printf("Still Probe and Commit data block lengths are the same: %d bytes\n", probe_length);
    }
    else
    {
        printf("Still Probe length: %d bytes, Commit length: %d bytes\n", probe_length, commit_length);
    }
}

/*
 *  Based on the current parameter block information, select the best-fit alternative interface of
 *  UVC streaming interface.
 */
/*static */int  usbh_uvc_select_alt_interface(UVC_DEV_T *vdev)
{
    IFACE_T      *iface;
    UVC_STRM_T   *vs = &vdev->vs;
    uint32_t     payload_size = vdev->param.dwMaxPayloadTransferSize;
    int          i, ret, best = -1;

    /*------------------------------------------------------------------------------------*/
    /*  Find the streaming interface                                                      */
    /*------------------------------------------------------------------------------------*/
    iface = vdev->udev->iface_list;

    while(iface != NULL)
    {
        if(iface->if_num == vdev->iface_stream->if_num)
            break;

        iface = iface->next;
    }

    if(iface == NULL)
    {
        UVC_DBGMSG("Can't find UVC streaming interface!\n");
        return UVC_RET_NOT_SUPPORT;
    }

    /*------------------------------------------------------------------------------------*/
    /*  Find the the best alternative interface                                           */
    /*------------------------------------------------------------------------------------*/
    if(payload_size > 3072)
        payload_size = 3072;

    /*
     *  Find the largest one of those "wMaxPacketSize <= 3072" settings
     */
    for(i = 0; i < vs->num_of_alt; i++)
    {
        if(vs->max_pktsz[i] <= 3072)
        {
            if(best == -1)
                best = i;
            else
            {
                if(vs->max_pktsz[i] > vs->max_pktsz[best])
                    best = i;
            }
        }
    }

    for(i = 0; i < vs->num_of_alt; i++)
    {
        UVC_DBGMSG("i=%d, best=%d, %d, %d\n", i, best, vs->max_pktsz[i], payload_size);

        if((vs->max_pktsz[i] >= payload_size) && (vs->max_pktsz[i] <= 3072))
        {
            if(best == -1)
                best = i;
            else
            {
                if(vs->max_pktsz[i] < vs->max_pktsz[best])
                    best = i;
            }
        }
    }

    if(best == -1)
    {
        UVC_DBGMSG("Bandwidth - Cannot find available alternative interface! %d\n", payload_size);
        return UVC_RET_NOT_SUPPORT;
    }

    UVC_DBGMSG("Select UVC streaming interface %d alternative setting %d.\n", iface->if_num, vs->alt_no[best]);
    ret = usbh_set_interface(iface, vs->alt_no[best]);

    if(ret != 0)
    {
        UVC_DBGMSG("Fail to set UVC streaming interface! %d, %d\n", iface->if_num, vs->alt_no[best]);
        return ret;
    }

    vdev->ep_iso_in = usbh_iface_find_ep(iface, 0, EP_ADDR_DIR_IN | EP_ATTR_TT_ISO);

    if(vdev->ep_iso_in == NULL)
    {
        UVC_DBGMSG("Can't find iso-in enpoint in the selected streaming interface! %d, %d\n", iface->if_num, best);
        return UVC_RET_NOT_SUPPORT;
    }

    return 0;
}

int  usbh_uvc_select_still_alt_interface(UVC_DEV_T *vdev)
{
    IFACE_T      *iface;
    UVC_STRM_T   *vs = &vdev->vs;
    uint32_t     payload_size = vdev->still_param.dwMaxPayloadTransferSize;
    int          i, ret, best = -1;

    /*------------------------------------------------------------------------------------*/
    /*  Find the streaming interface                                                      */
    /*------------------------------------------------------------------------------------*/
    iface = vdev->udev->iface_list;

    while(iface != NULL)
    {
        if(iface->if_num == vdev->iface_stream->if_num)
            break;

        iface = iface->next;
    }

    if(iface == NULL)
    {
        UVC_DBGMSG("Can't find UVC streaming interface!\n");
        return UVC_RET_NOT_SUPPORT;
    }

    /*------------------------------------------------------------------------------------*/
    /*  Find the the best alternative interface                                           */
    /*------------------------------------------------------------------------------------*/
    if(payload_size > 3072)
        payload_size = 3072;

    /*
     *  Find the largest one of those "wMaxPacketSize <= 3072" settings
     */
    for(i = 0; i < vs->num_of_alt; i++)
    {
        if(vs->max_pktsz[i] <= 3072)
        {
            if(best == -1)
                best = i;
            else
            {
                if(vs->max_pktsz[i] > vs->max_pktsz[best])
                    best = i;
            }
        }
    }

    for(i = 0; i < vs->num_of_alt; i++)
    {
        UVC_DBGMSG("i=%d, best=%d, %d, %d\n", i, best, vs->max_pktsz[i], payload_size);

        if((vs->max_pktsz[i] >= payload_size) && (vs->max_pktsz[i] <= 3072))
        {
            if(best == -1)
                best = i;
            else
            {
                if(vs->max_pktsz[i] < vs->max_pktsz[best])
                    best = i;
            }
        }
    }

    if(best == -1)
    {
        UVC_DBGMSG("Bandwidth - Cannot find available alternative interface! %d\n", payload_size);
        return UVC_RET_NOT_SUPPORT;
    }

    UVC_DBGMSG("Select UVC streaming interface %d alternative setting %d.\n", iface->if_num, vs->alt_no[best]);
    ret = usbh_set_interface(iface, vs->alt_no[best]);

    if(ret != 0)
    {
        UVC_DBGMSG("Fail to set UVC streaming interface! %d, %d\n", iface->if_num, vs->alt_no[best]);
        return ret;
    }

    vdev->ep_iso_in = usbh_iface_find_ep(iface, 0, EP_ADDR_DIR_IN | EP_ATTR_TT_ISO);

    if(vdev->ep_iso_in == NULL)
    {
        UVC_DBGMSG("Can't find iso-in enpoint in the selected streaming interface! %d, %d\n", iface->if_num, best);
        return UVC_RET_NOT_SUPPORT;
    }

    return 0;
}

/// @endcond HIDDEN_SYMBOLS

/**
 *  @brief  Get a video format from support list of the UVC device
 *  @param[in]  vdev    UVC device
 *  @param[in]  index   Index probe to supportd image format
 *  @param[out] format  If success, return the image format.
 *  @param[out] width   If success, return the image width.
 *  @param[out] height  If success, return the image height.
 *  @return   Success or failed.
 *  @retval   0          Success
 *  @retval   -1         Nonexistent
 *  @retval   Otheriwse  Error occurred
 */
int  usbh_get_video_format(UVC_DEV_T *vdev, int index, IMAGE_FORMAT_E *format, int *width, int *height)
{
    UVC_CTRL_T  *vc;

    if(vdev == NULL)
        return UVC_RET_DEV_NOT_FOUND;

    vc = &vdev->vc;

    if(index >= vc->num_of_frames)
        return -1;

    *format = vc->frame_format[index];
    *width  = vc->width[index];
    *height = vc->height[index];
    return 0;
}

/**
 *  @brief  Set video format
 *  @param[in]  vdev    UVC device
 *  @param[out] format  Image format
 *
 *  @param[out] width   Image width
 *  @param[out] height  Image height
 *  @return   Success or failed.
 *  @retval   0          Success
 *  @retval   Otheriwse  Error occurred
 */
int  usbh_set_video_format(UVC_DEV_T *vdev, IMAGE_FORMAT_E format, int width, int height)
{
    UVC_CTRL_T       *vc;
    UVC_CTRL_PARAM_T *param;
    int    format_index = -1, frame_index = -1;
    int    i, ret;

    if(vdev == NULL)
        return UVC_RET_DEV_NOT_FOUND;

    vc = &vdev->vc;
    param = &vdev->param;

    /*------------------------------------------------------------------------------------*/
    /*  Find video format index                                                           */
    /*------------------------------------------------------------------------------------*/
    for(i = 0; i < vc->num_of_formats; i++)
    {
        if(vc->format[i] == format)
        {
            format_index = vc->format_idx[i];
            break;
        }
    }

    if(format_index == -1)
    {
        UVC_DBGMSG("Video format 0x%x not supported!\n", format);
        return UVC_RET_NOT_SUPPORT;
    }

    /*------------------------------------------------------------------------------------*/
    /*  Find video frame index                                                            */
    /*------------------------------------------------------------------------------------*/
    for(i = 0; i < vc->num_of_frames; i++)
    {
        if(vc->frame_format[i] != format)
            continue;

        if((vc->width[i] == width) && (vc->height[i] == height))
        {
            frame_index = vc->frame_idx[i];
            break;
        }
    }

    if(frame_index == -1)
    {
        UVC_DBGMSG("Video size %d x %d not supported!\n", width, height);
        return UVC_RET_NOT_SUPPORT;
    }

    UVC_DBGMSG("Video format found, bFormatIndex=%d, bFrameIndex=%d\n", format_index, frame_index);

    /*------------------------------------------------------------------------------------*/
    /*  Get Video Probe Control                                                           */
    /*------------------------------------------------------------------------------------*/
    ret = usbh_uvc_probe_control(vdev, UVC_GET_CUR, param);

    if(ret < 0)
    {
        UVC_DBGMSG("Get Video Probe Control failed! %d\n", ret);
        return ret;
    }

    if((param->bFormatIndex == format_index) && (param->bFrameIndex == frame_index))
    {
        goto commit;
    }

    /*------------------------------------------------------------------------------------*/
    /*  Set Video Probe Control                                                           */
    /*------------------------------------------------------------------------------------*/

    param->bFormatIndex = format_index;
    param->bFrameIndex = frame_index;

    ret = usbh_uvc_probe_control(vdev, UVC_SET_CUR, param);

    if(ret < 0)
    {
        UVC_DBGMSG("Set Video Probe Control failed! %d\n", ret);
        return ret;
    }

    ret = usbh_uvc_probe_control(vdev, UVC_GET_CUR, param);

    if(ret < 0)
    {
        UVC_DBGMSG("Get Video Probe Control failed! %d\n", ret);
        return ret;
    }

    if((param->bFormatIndex != format_index) || (param->bFrameIndex != frame_index))
    {
        return UVC_RET_NOT_SUPPORT;
    }

commit:
    /*------------------------------------------------------------------------------------*/
    /*  Set Video Commit Control                                                          */
    /*------------------------------------------------------------------------------------*/
    ret = usbh_uvc_commit_control(vdev, param);

    if(ret < 0)
    {
        UVC_DBGMSG("Set Video Commit Control failed! %d\n", ret);
        return ret;
    }

    return ret;
}


/// @cond HIDDEN_SYMBOLS


int  usbh_get_video_still_format(UVC_DEV_T *vdev, int index, IMAGE_FORMAT_E *format, int *width, int *height)
{
    UVC_CTRL_T  *vc;

    if(vdev == NULL)
        return UVC_RET_DEV_NOT_FOUND;

    vc = &vdev->vc;

    if(index >= vc->num_of_frames)
        return -1;

    *format = vc->frame_format[index];
    *width  = vc->still_image_width[index];
    *height = vc->still_image_height[index];
    return 0;
}

/**
 *  @brief  Set video still format
 *  @param[in]  vdev    UVC device
 *  @param[out] format  Image format
 *
 *  @param[out] width   Still image width
 *  @param[out] height  Still image height
 *  @return   Success or failed.
 *  @retval   0          Success
 *  @retval   Otheriwse  Error occurred
 */
int  usbh_set_video_still_format(UVC_DEV_T *vdev, IMAGE_FORMAT_E format, int width, int height)
{
    UVC_CTRL_T       *vc;
    UVC_STILL_CTRL_PARAM_T *still_param;
    int    format_index = -1, frame_index = -1;
    int    i, ret;

    if(vdev == NULL)
        return UVC_RET_DEV_NOT_FOUND;

    vc = &vdev->vc;
    still_param = &vdev->still_param;

    /*------------------------------------------------------------------------------------*/
    /*  Find video format index                                                           */
    /*------------------------------------------------------------------------------------*/
    for(i = 0; i < vc->num_of_formats; i++)
    {
        if(vc->format[i] == format)
        {
            format_index = vc->format_idx[i];
            break;
        }
    }

    if(format_index == -1)
    {
        UVC_DBGMSG("Video still format 0x%x not supported!\n", format);
        return UVC_RET_NOT_SUPPORT;
    }

    /*------------------------------------------------------------------------------------*/
    /*  Find video frame index                                                            */
    /*------------------------------------------------------------------------------------*/
    for(i = 0; i < vc->num_of_frames; i++)
    {
        if(vc->frame_format[i] != format)
            continue;

        if((vc->still_image_width[i] == width) && (vc->still_image_height[i] == height))
        {
            frame_index = vc->frame_idx[i];
            break;
        }
    }

    if(frame_index == -1)
    {
        UVC_DBGMSG("Video size %d x %d not supported!\n", width, height);
        return UVC_RET_NOT_SUPPORT;
    }

    UVC_DBGMSG("Video format found, bFormatIndex=%d, bFrameIndex=%d\n", format_index, frame_index);

    /*------------------------------------------------------------------------------------*/
    /*  Get Video Still Probe Control                                                           */
    /*------------------------------------------------------------------------------------*/
    ret = usbh_uvc_still_probe_control(vdev, UVC_GET_CUR, still_param);

    if(ret < 0)
    {
        UVC_DBGMSG("Get Video Still Probe Control failed! %d\n", ret);
        return ret;
    }

    if((still_param->bFormatIndex == format_index) && (still_param->bFrameIndex == frame_index))
    {
        goto commit;
    }

    /*------------------------------------------------------------------------------------*/
    /*  Set Video Still Probe Control                                                           */
    /*------------------------------------------------------------------------------------*/

    still_param->bFormatIndex = format_index;
    still_param->bFrameIndex = frame_index;

    ret = usbh_uvc_still_probe_control(vdev, UVC_SET_CUR, still_param);

    if(ret < 0)
    {
        UVC_DBGMSG("Set Video Still Probe Control failed! %d\n", ret);
        return ret;
    }

    ret = usbh_uvc_still_probe_control(vdev, UVC_GET_CUR, still_param);

    if(ret < 0)
    {
        UVC_DBGMSG("Get Video Still Probe Control failed! %d\n", ret);
        return ret;
    }

    if((still_param->bFormatIndex != format_index) || (still_param->bFrameIndex != frame_index))
    {
        return UVC_RET_NOT_SUPPORT;
    }

commit:
    /*------------------------------------------------------------------------------------*/
    /*  Set Video Still Commit Control                                                          */
    /*------------------------------------------------------------------------------------*/
    ret = usbh_uvc_still_commit_control(vdev, still_param);

    if(ret < 0)
    {
        UVC_DBGMSG("Set Video Still Commit Control failed! %d\n", ret);
        return ret;
    }

    return ret;
}

/**
 * @brief  Check if the UVC device supports still image capture.
 * @param[in]  vdev    Pointer to the UVC device structure.
 * @return     1 if supported, 0 if not supported.
 */
//int uvc_supports_still_image(UVC_DEV_T *vdev)
//{
//    uint8_t *desc = vdev->vc_desc; // Pointer to Video Control descriptor
//    uint32_t desc_len = vdev->vc_desc_len; // Length of the descriptor
//    uint32_t offset = 0;

//    // Parse the descriptor
//    while (offset < desc_len)
//    {
//        uint8_t length = desc[offset];
//        uint8_t type = desc[offset + 1];

//        // Check for Class-Specific VS Interface Descriptor
//        if (type == CS_INTERFACE)
//        {
//            uint8_t subtype = desc[offset + 2];

//            // Check for VS_STILL_IMAGE_TRIGGER_CONTROL
//            if (subtype == VS_STILL_IMAGE_TRIGGER_CONTROL)
//            {
//                printf("Device supports still image capture.\n");
//                return 1; // Supported
//            }
//        }

//        // Move to the next descriptor
//        offset += length;
//    }

//    printf("Device does not support still image capture.\n");
//    return 0; // Not supported
//}


/// @cond HIDDEN_SYMBOLS


void  uvc_parse_streaming_data(UVC_DEV_T *vdev, uint8_t *buff, int pkt_len)
{
    UVC_STRM_T   *vs = &vdev->vs;
    //UVC_CTRL_T   *vc = &vdev->vc;
    int          data_len;

    if(pkt_len < 2)
        return;                             /* invalid packet                             */

    if(pkt_len < buff[0])
        return;                             /* unlikely pakcet length error               */

    data_len = pkt_len - buff[0];

    if(vs->current_frame_error)
    {
        if(buff[1] & UVC_PL_EOF)           /* error cleared only if EOF met              */
        {
            vs->current_frame_error = 0;
            vdev->img_size = 0;
        }

        return;
    }

    if(vdev->img_size == 0)                /* Start of a new image                       */
    {
        // Check for Still Image Frame flag (bit 4 of buff[1])
        if(buff[1] & UVC_PL_STI)
        {
            // Still Image Frame detected
            //printf("Still Image Frame detected\n");

            vs->current_frame_toggle = buff[1] & UVC_PL_FID;

            if(data_len > 0)
            {
                memcpy(vdev->img_buff, buff + buff[0], data_len);
                vdev->img_size = data_len;

                /* Single-packet still image: EOF may be set in this same packet */
                if(buff[1] & UVC_PL_EOF)
                {
                    if(vdev->func_rx && (vdev->img_size > 0))
                        vdev->func_rx(vdev, vdev->img_buff, vdev->img_size);
                    vdev->img_size = 0;
                }
                return;
            }
        }
        else
        {
            // Handle other types of frames (e.g., video frames)
            vs->current_frame_toggle = buff[1] & UVC_PL_FID;

            if(data_len > 0)
            {
                memcpy(vdev->img_buff, buff + buff[0], data_len);
                vdev->img_size = data_len;

                /* Single-packet video frame: EOF may be set in this same packet */
                if(buff[1] & UVC_PL_EOF)
                {
                    if(vdev->func_rx && (vdev->img_size > 0))
                        vdev->func_rx(vdev, vdev->img_buff, vdev->img_size);
                    vdev->img_size = 0;
                }
                return;
            }
        }
    }
    else
    {
        if((buff[1] & UVC_PL_FID) != vs->current_frame_toggle)
        {
            UVC_DBGMSG("FID toggle error!\n");
            vs->current_frame_toggle = buff[1] & UVC_PL_FID;
            vdev->img_size = 0;

            // Optionally, you can start accumulating new frame data here
            if(data_len > 0)
            {
                memcpy(vdev->img_buff, buff + buff[0], data_len);
                vdev->img_size = data_len;
            }

            return;
        }

        if(buff[1] & UVC_PL_ERR)
        {
            UVC_DBGMSG("Payload ERR bit error!\n");
            vs->current_frame_error = 1;
            return;
        }

        if((buff[1] & UVC_PL_RES) && (buff[1] & UVC_PL_PTS))
        {
            if(vdev->func_rx && (vdev->img_size > 0))
                vdev->func_rx(vdev, vdev->img_buff, vdev->img_size);

            vdev->img_size = 0;
            return;
        }

        if(vdev->img_size + data_len > vdev->img_buff_size)
        {
            UVC_DBGMSG("Image data overrun!\n");
            vs->current_frame_error = 1;
            return;
        }

        if(data_len > 0)
        {
            memcpy(vdev->img_buff + vdev->img_size, buff + buff[0], data_len);
            vdev->img_size += data_len;
        }

        if(buff[1] & UVC_PL_EOF)
        {
            if(vdev->func_rx && (vdev->img_size > 0))
                vdev->func_rx(vdev, vdev->img_buff, vdev->img_size);

            vdev->img_size = 0;
        }
    }
}


static void iso_in_irq(UTR_T *utr)
{
    UVC_DEV_T   *vdev = (UVC_DEV_T *)utr->context;
    int         i, ret;

    /* We don't want to do anything if we are about to be removed! */
    if(!vdev || !vdev->udev)
        return;

    if(vdev->is_streaming == 0)
    {
        UVC_DBGMSG("iso_in_irq stop utr 0x%x\n", (int)utr);
        utr->status = USBH_ERR_ABORT;
        return;
    }

    utr->bIsoNewSched = 0;

    for(i = 0; i < IF_PER_UTR; i++)
    {
        if(utr->iso_status[i] == 0)
        {
            uvc_parse_streaming_data(vdev, utr->iso_buff[i], utr->iso_xlen[i]);
        }
        else
        {
            if((utr->iso_status[i] == USBH_ERR_NOT_ACCESS0) || (utr->iso_status[i] == USBH_ERR_NOT_ACCESS1))
                utr->bIsoNewSched = 1;
        }

        utr->iso_xlen[i] = utr->ep->wMaxPacketSize;
    }

    /* schedule the following isochronous transfers */
    ret = usbh_iso_xfer(utr);

    if(ret < 0)
    {
        UVC_DBGMSG("usbh_iso_xfer failed!\n");
        utr->status = USBH_ERR_ABORT;
    }
}

/// @endcond HIDDEN_SYMBOLS

/**
 *  @brief  Give the image buffer where the next received image will be written to.
 *  @param[in] vdev           Video Class device
 *  @param[in] image_buff     The image buffer.
 *  @param[in] img_buff_size  Size of the image buffer.
 *  @return    None.
 */
void usbh_uvc_set_video_buffer(UVC_DEV_T *vdev, uint8_t *image_buff, int img_buff_size)
{
    vdev->img_buff = image_buff;
    vdev->img_buff_size = img_buff_size;
    vdev->img_size = 0;
}


/**
 *  @brief  Start to receive video data from UVC device.
 *  @param[in] vdev       Video Class device
 *  @param[in] func       Video in callback function.
 *  @return   Success or not.
 *  @retval    0          Success
 *  @retval    Otherwise  Failed
 */
int usbh_uvc_start_streaming(UVC_DEV_T *vdev, UVC_CB_FUNC *func)
{
    UDEV_T       *udev = vdev->udev;
    EP_INFO_T    *ep;
    UTR_T        *utr;
    int          i, j, ret;

    if((vdev == NULL) || (func == NULL))
        return UVC_RET_INVALID;

    if(vdev->is_streaming)
        return UVC_RET_IS_STREAMING;

    /*
     *  Select the best alternative streaming interface and also determine the endpoint.
     */
    ret = usbh_uvc_select_alt_interface(vdev);

    if(ret != 0)
    {
        UVC_ERRMSG("Failed to select UVC alternative interface!\n");
        return ret;
    }

    ep = vdev->ep_iso_in;

    vdev->func_rx = func;

#ifdef UVC_DEBUG
    UVC_DBGMSG("Actived isochronous-in endpoint =>");
    usbh_dump_ep_info(ep);
#endif

    /*------------------------------------------------------------------------------------*/
    /*  Allocate isochronous in UTRs and assign transfer buffer                           */
    /*------------------------------------------------------------------------------------*/
    for(i = 0; i < UVC_UTR_PER_STREAM; i++)
    {
        if(vdev->utr_rx[i] != NULL)
        {
            vdev->utr_rx[i]->status = 0;
            /*
             *  The UTR is reused from a previous streaming session (e.g. still Alt4→video Alt2).
             *  iso_xlen[j] must be updated to the NEW endpoint's wMaxPacketSize.
             *  If left at the old value, the EHCI programs iTD slots with the wrong size:
             *  - Too small (512 vs 1536): camera packets cause overflow → iso_status error
             *    → iso_in_irq discards data silently, JPEG header lost → timeout.
             *  - Too large (1536 vs 512): DMA buffer over-request (less critical but wrong).
             *  iso_in_irq corrects iso_xlen only AFTER the first IRQ fires; the first batch
             *  of transfers must already have the correct size.
             */
            for(j = 0; j < IF_PER_UTR; j++)
                vdev->utr_rx[i]->iso_xlen[j] = ep->wMaxPacketSize;
            continue;
        }

        utr = alloc_utr(udev);              /* allocate UTR                               */

        if(utr == NULL)
        {
            ret = USBH_ERR_MEMORY_OUT;      /* memory allocate failed                     */
            goto err_2;                     /* abort                                      */
        }

        vdev->utr_rx[i] = utr;
        utr->buff = vdev->in_buff + i * UVC_UTR_INBUF_SIZE;
        utr->data_len = UVC_UTR_INBUF_SIZE;

        for(j = 0; j < IF_PER_UTR; j++)
        {
            utr->iso_buff[j] = utr->buff + j * 3072;
            utr->iso_xlen[j] = ep->wMaxPacketSize;
        }
    }

    /*------------------------------------------------------------------------------------*/
    /*  Start UTRs                                                                        */
    /*------------------------------------------------------------------------------------*/

    vdev->utr_rx[0]->bIsoNewSched = 1;
    vdev->is_streaming = 1;

    for(i = 0; i < UVC_UTR_PER_STREAM; i++)
    {
        utr = vdev->utr_rx[i];
        utr->context = vdev;
        utr->ep = ep;
        utr->func = iso_in_irq;
        ret = usbh_iso_xfer(utr);

        if(ret < 0)
        {
            UVC_DBGMSG("Error - failed to start UTR %d isochronous-in transfer (%d)", i, ret);
            goto err_1;
        }
    }

    return UVC_RET_OK;

err_1:

    for(i = 0; i < UVC_UTR_PER_STREAM; i++)
    {
        usbh_quit_utr(vdev->utr_rx[i]);         /* quit all UTRs                          */
    }

err_2:

    for(i = 0; i < UVC_UTR_PER_STREAM; i++)    /* free all UTRs                          */
    {
        if(vdev->utr_rx[i] != NULL)
        {
            free_utr(vdev->utr_rx[i]);
            vdev->utr_rx[i] = NULL;
        }
    }

    return ret;
}

int usbh_uvc_start_still_streaming(UVC_DEV_T *vdev, UVC_CB_FUNC *func)
{
    UDEV_T       *udev = vdev->udev;
    EP_INFO_T    *ep;
    UTR_T        *utr;
    int          i, j, ret;

    if((vdev == NULL) || (func == NULL))
        return UVC_RET_INVALID;

    if(vdev->is_streaming)
        return UVC_RET_IS_STREAMING;

    /*
     *  Select the best alternative streaming interface and also determine the endpoint.
     */
    ret = usbh_uvc_select_still_alt_interface(vdev);

    if(ret != 0)
    {
        UVC_ERRMSG("Failed to select UVC alternative interface!\n");
        return ret;
    }

    ep = vdev->ep_iso_in;

    vdev->func_rx = func;

#ifdef UVC_DEBUG
    UVC_DBGMSG("Actived still isochronous-in endpoint =>");
    usbh_dump_ep_info(ep);
#endif

    /*------------------------------------------------------------------------------------*/
    /*  Allocate isochronous in UTRs and assign transfer buffer                           */
    /*------------------------------------------------------------------------------------*/
    for(i = 0; i < UVC_UTR_PER_STREAM; i++)
    {
        if(vdev->utr_rx[i] != NULL)
        {
            vdev->utr_rx[i]->status = 0;
            /*
             *  The UTR is reused from a previous streaming session (e.g. video Alt2→still Alt4).
             *  CRITICAL: iso_xlen[j] MUST be updated to the new endpoint's wMaxPacketSize.
             *  If still-Alt4 needs 1536 bytes/packet but iso_xlen[j] = 512 (from video-Alt2),
             *  the EHCI programs iTD slots requesting only 512 bytes. When the camera sends
             *  1536-byte packets, the EHCI reports overflow (iso_status[i] != 0).
             *  iso_in_irq skips uvc_parse_streaming_data() for error slots → ALL still image
             *  data is silently discarded, including the JPEG SOI (FF D8) in the first
             *  packet → find_jpeg_frame() never finds a valid frame → timeout.
             *  iso_in_irq corrects iso_xlen only after the first IRQ; the first batch
             *  of isochronous transfers must already carry the correct size.
             */
            for(j = 0; j < IF_PER_UTR; j++)
                vdev->utr_rx[i]->iso_xlen[j] = ep->wMaxPacketSize;
            continue;
        }

        utr = alloc_utr(udev);              /* allocate UTR                               */

        if(utr == NULL)
        {
            ret = USBH_ERR_MEMORY_OUT;      /* memory allocate failed                     */
            goto err_2;                     /* abort                                      */
        }

        vdev->utr_rx[i] = utr;
        utr->buff = vdev->in_buff + i * UVC_UTR_INBUF_SIZE;
        utr->data_len = UVC_UTR_INBUF_SIZE;

        for(j = 0; j < IF_PER_UTR; j++)
        {
            utr->iso_buff[j] = utr->buff + j * 3072;
            utr->iso_xlen[j] = ep->wMaxPacketSize;
        }
    }

    /*------------------------------------------------------------------------------------*/
    /*  Start UTRs                                                                        */
    /*------------------------------------------------------------------------------------*/

    vdev->utr_rx[0]->bIsoNewSched = 1;
    vdev->is_streaming = 1;

    for(i = 0; i < UVC_UTR_PER_STREAM; i++)
    {
        utr = vdev->utr_rx[i];
        utr->context = vdev;
        utr->ep = ep;
        utr->func = iso_in_irq;
        ret = usbh_iso_xfer(utr);

        if(ret < 0)
        {
            UVC_DBGMSG("Error - failed to start UTR %d isochronous-in transfer (%d)", i, ret);
            goto err_1;
        }
    }

    return UVC_RET_OK;

err_1:

    for(i = 0; i < UVC_UTR_PER_STREAM; i++)
    {
        usbh_quit_utr(vdev->utr_rx[i]);         /* quit all UTRs                          */
    }

err_2:

    for(i = 0; i < UVC_UTR_PER_STREAM; i++)    /* free all UTRs                          */
    {
        if(vdev->utr_rx[i] != NULL)
        {
            free_utr(vdev->utr_rx[i]);
            vdev->utr_rx[i] = NULL;
        }
    }

    return ret;
}

/**
 *  @brief  Pause the video straming input.
 *  @param[in] vdev       Video Class device
 *  @return   Success or not.
 *  @retval    0          Success
 *  @retval    Otherwise  Failed
 */
int usbh_uvc_stop_streaming(UVC_DEV_T *vdev)
{
    IFACE_T   *iface;
    int       i, ret;

    if(vdev == NULL)
        return UVC_RET_INVALID;

    if(!vdev->is_streaming)
        return UVC_RET_OK;                  /* UVC is currently not streaming, do nothing */

    vdev->is_streaming = 0;

    /*------------------------------------------------------------------------------------*/
    /*  Explicitly abort all isochronous UTRs before the interface switch.               */
    /*  This removes their ITDs from the EHCI frame list so SET_INTERFACE(alt=0)         */
    /*  is not rejected by the camera due to in-flight isochronous activity.             */
    /*------------------------------------------------------------------------------------*/
    for(i = 0; i < UVC_UTR_PER_STREAM; i++)
    {
        if(vdev->utr_rx[i] != NULL)
        {
            usbh_quit_utr(vdev->utr_rx[i]);

            /*  Free the UTR completely so that the next call to start_streaming() or
             *  start_still_streaming() always allocates a FRESH UTR with clean state.
             *
             *  Root cause of still-capture timeout (observed symptom: iso_in_irq never
             *  fires during the 3-second wait after start_still_streaming()):
             *
             *  After usbh_quit_utr(), the UTR struct is alive but its associated iTDs
             *  may still be known to the EHCI hardware (residual state in the periodic
             *  frame list).  When usbh_iso_xfer() is called again with the same UTR
             *  object (reuse path), the EHCI treats it as "already scheduled" and does
             *  not insert new iTDs.  usbh_iso_xfer() returns 0 (success) but no
             *  isochronous transfers ever start, iso_in_irq never fires, and no data
             *  arrives  → permanent timeout.
             *
             *  free_utr() is safe here: usbh_quit_utr() blocks until the final
             *  iso_in_irq callback fires ("iso_in_irq stop utr ..."), confirming the
             *  UTR is fully detached from EHCI before this function returns.
             *  The same quit→free pattern is already used in the error paths of
             *  start_streaming() / start_still_streaming().                           */
            free_utr(vdev->utr_rx[i]);
            vdev->utr_rx[i] = NULL;
        }
    }

    /*------------------------------------------------------------------------------------*/
    /*  Find the streaming interface                                                      */
    /*------------------------------------------------------------------------------------*/
    iface = vdev->udev->iface_list;

    while(iface != NULL)
    {
        if(iface->if_num == vdev->iface_stream->if_num)
            break;

        iface = iface->next;
    }

    if(iface == NULL)
    {
        UVC_DBGMSG("Can't find UVC streaming interface!\n");
        return UVC_RET_NOT_SUPPORT;
    }

    ret = usbh_set_interface(iface, 0);     /* select alternative setting (0 bandwidth)   */

    if(ret != 0)
    {
        UVC_DBGMSG("Fail to select UVC streaming interface alt. 0! %d\n", iface->if_num);
        /* UTRs are already cancelled; streaming is stopped at software level.
         * Do not propagate the error so the caller can proceed with start_streaming. */
    }

    return 0;
}

/*! @}*/ /* end of group USBH_EXPORTED_FUNCTIONS */

/*! @}*/ /* end of group USBH_Library */

/*! @}*/ /* end of group LIBRARY */
