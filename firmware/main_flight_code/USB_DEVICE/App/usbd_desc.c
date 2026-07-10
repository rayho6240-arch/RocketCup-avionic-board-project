/**
  ******************************************************************************
  * @file    usbd_desc.c
  * @brief   USB Device 描述符（手動移植，F407 FS CDC，VID 0x0483 / PID 0x5740）。
  *          序號由 MCU 96-bit 唯一 ID 產生。整檔 #if FEATURE_USB_CDC。
  ******************************************************************************
  */
#include "board_config.h"
#if FEATURE_USB_CDC

#include "usbd_core.h"
#include "usbd_desc.h"
#include "usbd_conf.h"

#define USBD_VID                      0x0483
#define USBD_PID_FS                   0x5740
#define USBD_LANGID_STRING            1033
#define USBD_MANUFACTURER_STRING      "RocketCom"
#define USBD_PRODUCT_STRING_FS        "RocketCom GroundStation"
#define USBD_CONFIGURATION_STRING_FS  "CDC Config"
#define USBD_INTERFACE_STRING_FS      "CDC Interface"

#define USB_SIZ_STRING_SERIAL         0x1AU

static void Get_SerialNum(void);
static void IntToUnicode(uint32_t value, uint8_t *pbuf, uint8_t len);

static uint8_t *USBD_FS_DeviceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);
static uint8_t *USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length);

USBD_DescriptorsTypeDef FS_Desc = {
    USBD_FS_DeviceStrDescriptor,
    USBD_FS_LangIDStrDescriptor,
    USBD_FS_ManufacturerStrDescriptor,
    USBD_FS_ProductStrDescriptor,
    USBD_FS_SerialStrDescriptor,
    USBD_FS_ConfigStrDescriptor,
    USBD_FS_InterfaceStrDescriptor
};

/* USB 標準裝置描述符（18 bytes） */
#if defined ( __ICCARM__ )
#pragma data_alignment=4
#endif
__ALIGN_BEGIN static uint8_t USBD_FS_DeviceDesc[USB_LEN_DEV_DESC] __ALIGN_END = {
    0x12,                       /* bLength */
    USB_DESC_TYPE_DEVICE,       /* bDescriptorType */
    0x00, 0x02,                 /* bcdUSB 2.00 */
    0x02,                       /* bDeviceClass: CDC */
    0x02,                       /* bDeviceSubClass */
    0x00,                       /* bDeviceProtocol */
    USB_MAX_EP0_SIZE,           /* bMaxPacketSize0 */
    LOBYTE(USBD_VID), HIBYTE(USBD_VID),
    LOBYTE(USBD_PID_FS), HIBYTE(USBD_PID_FS),
    0x00, 0x02,                 /* bcdDevice rel. 2.00 */
    USBD_IDX_MFC_STR,
    USBD_IDX_PRODUCT_STR,
    USBD_IDX_SERIAL_STR,
    USBD_MAX_NUM_CONFIGURATION
};

#if defined ( __ICCARM__ )
#pragma data_alignment=4
#endif
__ALIGN_BEGIN static uint8_t USBD_LangIDDesc[USB_LEN_LANGID_STR_DESC] __ALIGN_END = {
    USB_LEN_LANGID_STR_DESC,
    USB_DESC_TYPE_STRING,
    LOBYTE(USBD_LANGID_STRING), HIBYTE(USBD_LANGID_STRING)
};

#if defined ( __ICCARM__ )
#pragma data_alignment=4
#endif
__ALIGN_BEGIN static uint8_t USBD_StringSerial[USB_SIZ_STRING_SERIAL] __ALIGN_END = {
    USB_SIZ_STRING_SERIAL,
    USB_DESC_TYPE_STRING,
};

#if defined ( __ICCARM__ )
#pragma data_alignment=4
#endif
__ALIGN_BEGIN static uint8_t USBD_StrDesc[USBD_MAX_STR_DESC_SIZ] __ALIGN_END;

static uint8_t *USBD_FS_DeviceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = sizeof(USBD_FS_DeviceDesc);
    return USBD_FS_DeviceDesc;
}
static uint8_t *USBD_FS_LangIDStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = sizeof(USBD_LangIDDesc);
    return USBD_LangIDDesc;
}
static uint8_t *USBD_FS_ProductStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_PRODUCT_STRING_FS, USBD_StrDesc, length);
    return USBD_StrDesc;
}
static uint8_t *USBD_FS_ManufacturerStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
    return USBD_StrDesc;
}
static uint8_t *USBD_FS_SerialStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    *length = USB_SIZ_STRING_SERIAL;
    Get_SerialNum();
    return USBD_StringSerial;
}
static uint8_t *USBD_FS_ConfigStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_CONFIGURATION_STRING_FS, USBD_StrDesc, length);
    return USBD_StrDesc;
}
static uint8_t *USBD_FS_InterfaceStrDescriptor(USBD_SpeedTypeDef speed, uint16_t *length)
{
    (void)speed;
    USBD_GetString((uint8_t *)USBD_INTERFACE_STRING_FS, USBD_StrDesc, length);
    return USBD_StrDesc;
}

/* 由 MCU 96-bit 唯一 ID 產生序號字串 */
static void Get_SerialNum(void)
{
    uint32_t deviceserial0 = *(uint32_t *)(UID_BASE);
    uint32_t deviceserial1 = *(uint32_t *)(UID_BASE + 0x4);
    uint32_t deviceserial2 = *(uint32_t *)(UID_BASE + 0x8);

    deviceserial0 += deviceserial2;
    if (deviceserial0 != 0U) {
        IntToUnicode(deviceserial0, &USBD_StringSerial[2], 8);
        IntToUnicode(deviceserial1, &USBD_StringSerial[18], 4);
    }
}

static void IntToUnicode(uint32_t value, uint8_t *pbuf, uint8_t len)
{
    uint8_t idx;
    for (idx = 0; idx < len; idx++) {
        if (((value >> 28)) < 0xA) {
            pbuf[2 * idx] = (uint8_t)((value >> 28) + '0');
        } else {
            pbuf[2 * idx] = (uint8_t)((value >> 28) + 'A' - 10);
        }
        value = value << 4;
        pbuf[2 * idx + 1] = 0;
    }
}

#endif /* FEATURE_USB_CDC */
