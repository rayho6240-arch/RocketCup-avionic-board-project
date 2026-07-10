/**
  ******************************************************************************
  * @file    usb_device.c
  * @brief   USB Device 初始化（手動移植）。整檔 #if FEATURE_USB_CDC：飛行版不編入，
  *          故主/備航電不啟動 USB（不列舉、不進 USB 中斷、不佔 CPU）。
  ******************************************************************************
  */
#include "board_config.h"
#if FEATURE_USB_CDC

#include "usb_device.h"
#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"
#include "main.h"   /* Error_Handler */

USBD_HandleTypeDef hUsbDeviceFS;

void MX_USB_DEVICE_Init(void)
{
    if (USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS) != USBD_OK) {
        Error_Handler();
    }
    if (USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC) != USBD_OK) {
        Error_Handler();
    }
    if (USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS) != USBD_OK) {
        Error_Handler();
    }
    if (USBD_Start(&hUsbDeviceFS) != USBD_OK) {
        Error_Handler();
    }
}

#endif /* FEATURE_USB_CDC */
