set(USBH_CORE_SRC
    src_core/ehci.c
    src_core/ehci_iso.c
    src_core/hub.c
    src_core/mem_alloc.c
    src_core/ohci.c
    src_core/usb_core.c
)

add_library(usbh_core_lib ${USBH_CORE_SRC})
target_include_directories(usbh_core_lib PUBLIC ${CMAKE_CURRENT_LIST_DIR}/inc)

set(USBH_UVC_SRC
    src_uvc/uvc_core.c
    src_uvc/uvc_driver.c
    src_uvc/uvc_parser.c
)

add_library(usbh_uvc_lib ${USBH_UVC_SRC})
target_include_directories(usbh_uvc_lib PUBLIC ${CMAKE_CURRENT_LIST_DIR}/inc)
