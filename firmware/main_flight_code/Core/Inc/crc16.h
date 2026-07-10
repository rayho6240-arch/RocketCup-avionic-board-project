/*
 * crc16.h — CRC-16/CCITT-FALSE 單一實作（P1，取代 w25qxx/telemetry 兩份重複）
 * ===========================================================================
 * 參數：poly=0x1021, init=0xFFFF, 無反射, xorout=0x0000。
 * 黃金向量："123456789"（9 bytes ASCII）→ 0x29B1（tests/test_telemetry.c 驗證）。
 * 使用者：w25qxx.c ring_crc16（Flash ring/SysFlags）、telemetry.c telem_crc16
 *（下行遙測，與地面站 GroundStation/telemetry_decoder.py 同參數）。
 * 純 header-only，host 測試直接 include。
 */
#ifndef CRC16_H
#define CRC16_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

static inline uint16_t crc16_ccitt_false(const uint8_t *data, uint16_t len)
{
    uint16_t crc = 0xFFFFu;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u)
                                  : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

#ifdef __cplusplus
}
#endif

#endif /* CRC16_H */
