/*
 * flash_ring_math.h — Flash 環形緩衝區幾何數學（P1，header-only，host 可測）
 * ===========================================================================
 * 自 w25qxx.c 抽出的純位址計算：池量、寫入/擦除指標推進、迴繞、熱啟動回讀
 * 位址。不依賴 HAL / SPI（僅 stdint），由 tests/test_flash_ring.c 驗證。
 * w25qxx.c 僅保留實際 SPI 讀寫/擦除與狀態變數。
 *
 * 本次抽離同時修復迴繞歧義 bug（P1 flash ring 加固項）：
 *   舊版 erased_end 採「可停在 FLASH_RINGBUF_END+1」的 exclusive-end 語意
 *   （滾動擦除以 `> END+1` 才迴繞、PreEraseOne 以 `> END` 補正規化，兩處
 *   已不一致）。當擦除推進到環尾(erased_end=END+1)且寫入指標隨後也迴繞回
 *   BASE 時，pool = erased_end − write = 整個環的假池量 —— 實際上 BASE 起
 *   的扇區是最舊資料、根本未擦。之後整圈寫入全部落在未擦區（NOR flash
 *   只能 1→0，覆寫=資料損毀），且因 W25QXX_WriteData 不回讀驗證而silent。
 *   長時間 bench 浸泡（30min ≈ 2.9MB）累積數次即會繞滿 15.9MB 觸發。
 *
 *   新版：erased_end 一律正規化 —— 推進後 ≥ END+1 即迴繞回 BASE，環上恆有
 *   erased_end ∈ [BASE, END]。語意：pool = write→erased_end 的環向距離，
 *   erased_end == write ⇒ pool = 0（空池）。「全環已擦」與「空池」的兩針
 *   歧義由池上限消除：FLASH_RING_PREERASE_TARGET(64 sectors=256KB) 遠小於
 *   環容量 15.9MB，pool 永不合法地達到整環。
 */
#ifndef FLASH_RING_MATH_H
#define FLASH_RING_MATH_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* === 環形緩衝區幾何（自 w25qxx.h 移入；單一真相來源） === */
#define FLASH_RINGBUF_ADDR       0x010000UL   /* Ring Buffer 起始（64KB 對齊） */
#define FLASH_RINGBUF_END        0xFFFFFFUL   /* Ring Buffer 結束（inclusive） */
#define FLASH_RINGBUF_SIZE       0xFF0000UL   /* ~15.9 MB */
#define FLASH_RING_PACKET_SIZE   80UL         /* 每筆封包大小 80 bytes */
#define FLASH_RING_PREERASE_N    10           /* 開機預擦 Sector 數量 */

/* P0-E：PAD 期背景預擦目標池。64 sectors × 51 封包 ÷ 50Hz ≈ 65s 飛行容量（>2× 全程）。
 * 飛行態（BOOST..MAIN_DEPLOY）禁止同步滾動擦除（最壞 ~400ms 阻塞主迴圈，FSM 停擺、
 * EKF 斷饋且持 SPI3 mutex）—— 池耗盡時丟棄該筆並計數（遙測可觀測）。
 * ⚠️ 飛行時間 >60s 的任務需加大此值，並於發射檢核表確認 [FLASH] pool 達標後才起飛。 */
#define FLASH_RING_PREERASE_TARGET 64U        /* PAD 期背景預擦目標（sectors） */

/* 擦除粒度。必須等於 W25QXX_SECTOR_SIZE（w25qxx.h 以 _Static_assert 鎖定）。 */
#define FLASH_RING_SECTOR_SIZE   4096UL

/* 幾何不變量：封包與 sector 都恰好鋪滿整環（無封包跨環尾、環尾 sector 對齊）。
 * 0xFF0000 / 80 = 208896 整；0xFF0000 / 4096 = 4080 整。
 * ring_last_packet_addr() 的迴繞回讀與 erased_end 正規化皆依賴此性質。 */
#if (FLASH_RINGBUF_SIZE % FLASH_RING_PACKET_SIZE) != 0
#error "FLASH_RINGBUF_SIZE 必須是 FLASH_RING_PACKET_SIZE 的整數倍"
#endif
#if (FLASH_RINGBUF_SIZE % FLASH_RING_SECTOR_SIZE) != 0
#error "FLASH_RINGBUF_SIZE 必須是 FLASH_RING_SECTOR_SIZE 的整數倍"
#endif

/* 池量（bytes）：write → erased_end 的環向距離。兩針相等 ⇒ 0（空池）。 */
static inline uint32_t ring_pool_bytes_calc(uint32_t write_addr, uint32_t erased_end)
{
    if (erased_end >= write_addr) {
        return erased_end - write_addr;
    }
    return (FLASH_RINGBUF_END + 1UL - write_addr) + (erased_end - FLASH_RINGBUF_ADDR);
}

/* 寫入指標推進一筆封包（含迴繞）。因封包恰好鋪滿整環，推進後恰落 END+1 即迴繞。 */
static inline uint32_t ring_write_advance(uint32_t write_addr)
{
    write_addr += FLASH_RING_PACKET_SIZE;
    if (write_addr + FLASH_RING_PACKET_SIZE > FLASH_RINGBUF_END + 1UL) {
        write_addr = FLASH_RINGBUF_ADDR;
    }
    return write_addr;
}

/* 下一個待擦 sector 的位址（= 正規化後的 erased_end 本身；防衛性收斂）。 */
static inline uint32_t ring_erase_target(uint32_t erased_end)
{
    return (erased_end > FLASH_RINGBUF_END) ? FLASH_RINGBUF_ADDR : erased_end;
}

/* 擦除指標推進一個 sector（正規化迴繞：≥ END+1 即回 BASE，恆 ∈ [BASE, END]）。 */
static inline uint32_t ring_erase_advance(uint32_t erased_end)
{
    erased_end = ring_erase_target(erased_end) + FLASH_RING_SECTOR_SIZE;
    if (erased_end >= FLASH_RINGBUF_END + 1UL) {
        erased_end = FLASH_RINGBUF_ADDR;
    }
    return erased_end;
}

/* 最後一筆已寫封包的位址（熱啟動回讀）。write==BASE 時為環尾最後一格。 */
static inline uint32_t ring_last_packet_addr(uint32_t write_addr)
{
    if (write_addr == FLASH_RINGBUF_ADDR) {
        return FLASH_RINGBUF_END + 1UL - FLASH_RING_PACKET_SIZE;
    }
    return write_addr - FLASH_RING_PACKET_SIZE;
}

/* 給定封包位址的前一筆封包位址（環向後退一格）。 */
static inline uint32_t ring_prev_packet_addr(uint32_t pkt_addr)
{
    if (pkt_addr == FLASH_RINGBUF_ADDR) {
        return FLASH_RINGBUF_END + 1UL - FLASH_RING_PACKET_SIZE;
    }
    return pkt_addr - FLASH_RING_PACKET_SIZE;
}

/* 區間 [addr, addr+len) 是否完整落在已擦池 [write, erased_end) 環區間內。
 * 寫入路徑的安全不變量（host 測試用；firmware 寫入前置檢查亦可呼叫）。 */
static inline uint8_t ring_span_in_pool(uint32_t write_addr, uint32_t erased_end,
                                        uint32_t addr, uint32_t len)
{
    uint32_t pool = ring_pool_bytes_calc(write_addr, erased_end);
    /* addr 相對 write 的環向偏移 */
    uint32_t off = (addr >= write_addr)
                   ? (addr - write_addr)
                   : (FLASH_RINGBUF_END + 1UL - write_addr) + (addr - FLASH_RINGBUF_ADDR);
    return (off + len <= pool) ? 1U : 0U;
}

#ifdef __cplusplus
}
#endif

#endif /* FLASH_RING_MATH_H */
