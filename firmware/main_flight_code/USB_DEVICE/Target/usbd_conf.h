/**
  ******************************************************************************
  * @file    usbd_conf.h
  * @brief   USB Device 設定（手動移植，非 CubeMX 產生）。F407 USB_OTG_FS、全速、單一 CDC。
  *          記憶體採靜態配置（USBD_static_malloc）。本檔僅提供巨集/型別，恆被編入；
  *          USB 是否實際啟用由各 .c 以 #if FEATURE_USB_CDC 控制（見 board_config.h）。
  ******************************************************************************
  */
#ifndef __USBD_CONF__H__
#define __USBD_CONF__H__

#ifdef __cplusplus
extern "C" {
#endif

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f4xx.h"
#include "stm32f4xx_hal.h"

/* CDC 單實例、全速設定 */
#define USBD_MAX_NUM_INTERFACES        1U
#define USBD_MAX_NUM_CONFIGURATION     1U
#define USBD_MAX_STR_DESC_SIZ          512U
#define USBD_DEBUG_LEVEL               0U
#define USBD_LPM_ENABLED               0U
#define USBD_SELF_POWERED              1U
#define USBD_CLASS_USER_STRING_DESC    0U
#define USBD_SUPPORT_USER_STRING_DESC  0U
#define USBD_CLASS_BOS_ENABLED         0U
#define DEVICE_FS                      0

/* 記憶體管理（靜態配置） */
#define USBD_malloc         (void *)USBD_static_malloc
#define USBD_free           USBD_static_free
#define USBD_memset         memset
#define USBD_memcpy         memcpy
#define USBD_Delay          HAL_Delay

/* DEBUG 巨集（USBD_DEBUG_LEVEL=0 → 全部 no-op） */
#if (USBD_DEBUG_LEVEL > 0U)
#define USBD_UsrLog(...)  do { printf(__VA_ARGS__); printf("\n"); } while (0)
#else
#define USBD_UsrLog(...)  do {} while (0)
#endif
#if (USBD_DEBUG_LEVEL > 1U)
#define USBD_ErrLog(...)  do { printf("ERROR: "); printf(__VA_ARGS__); printf("\n"); } while (0)
#else
#define USBD_ErrLog(...)  do {} while (0)
#endif
#if (USBD_DEBUG_LEVEL > 2U)
#define USBD_DbgLog(...)  do { printf("DEBUG : "); printf(__VA_ARGS__); printf("\n"); } while (0)
#else
#define USBD_DbgLog(...)  do {} while (0)
#endif

void *USBD_static_malloc(uint32_t size);
void  USBD_static_free(void *p);

#ifdef __cplusplus
}
#endif

#endif /* __USBD_CONF__H__ */
