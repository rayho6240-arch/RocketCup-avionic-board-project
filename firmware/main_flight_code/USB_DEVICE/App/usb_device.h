/**
  ******************************************************************************
  * @file    usb_device.h
  * @brief   USB Device 初始化（手動移植）。MX_USB_DEVICE_Init 由 main.c 在
  *          FEATURE_USB_CDC（即地面站）時呼叫。
  ******************************************************************************
  */
#ifndef __USB_DEVICE__H__
#define __USB_DEVICE__H__

#ifdef __cplusplus
extern "C" {
#endif

#include "usbd_def.h"

extern USBD_HandleTypeDef hUsbDeviceFS;

/** @brief 初始化 USB Device + 註冊 CDC 類別 + 啟動。僅地面站呼叫。 */
void MX_USB_DEVICE_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __USB_DEVICE__H__ */
