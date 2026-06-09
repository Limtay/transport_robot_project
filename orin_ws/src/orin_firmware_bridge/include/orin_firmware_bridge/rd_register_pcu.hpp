#ifndef ORIN_FIRMWARE_BRIDGE__RD_REGISTER_PCU_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_REGISTER_PCU_HPP_

#include "orin_firmware_bridge/rd_common.hpp"

namespace orin_bridge::pcu {

/* ===== Region offset / size (bytes) ===== */
constexpr uint16_t REG_TOTAL_SIZE        = 256;

constexpr uint16_t REG_DEFINE_OFFSET     =   0;
constexpr uint16_t REG_DEFINE_SIZE       =  16;

constexpr uint16_t REG_SYS_OFFSET        =  46;
constexpr uint16_t REG_SYS_SIZE          =  16;

constexpr uint16_t REG_DATA_POWER_OFFSET =  96;
constexpr uint16_t REG_DATA_POWER_SIZE   =   8;

constexpr uint16_t REG_CMD_OFFSET        = 128;
constexpr uint16_t REG_CMD_SIZE          =   4;

constexpr uint16_t REG_DIAG_OFFSET       = 224;
constexpr uint16_t REG_DIAG_SIZE         =  32;

/* ===== [DEFINE] addr 0~15 (16 bytes) ===== */
typedef struct __attribute__((packed)) {
    /* addr  0 */ uint8_t sys_write_mode;
    /* addr  1 */ uint8_t err_timeout;
    /* addr  2 */ uint8_t fatal_timeout;
    /* addr  3 */ uint8_t err_cnt;
    /* addr  4 */ uint8_t fatal_cnt;
    /* addr  5 */ uint8_t hw_reset;
    /* addr  6 */ uint8_t reserved[10];
} DEFINE_t;

/* ===== [SYSTEM] addr 46~61 (16 bytes) ===== */
typedef struct __attribute__((packed)) {
    /* addr 46 */ uint8_t  sys_state;   // 0=INIT / 1=IDLE / 2=ACTIVE / 3=FAULT
    /* addr 47 */ uint8_t  hw_error;
    /* addr 48 */ uint8_t  reserved[14];
} DATA_SYS_t;

/* ===== [POWER] addr 96~103 (8 bytes) — RX from PCU ===== */
typedef struct __attribute__((packed)) {
    /* addr  96 */ uint32_t battery_voltage; // [mV]
    /* addr 100 */ int16_t  battery_current; // [mA], 음수=방전
    /* addr 102 */ uint8_t  reserved[2];
} DATA_POWER_t;

/* ===== [CMD] addr 128~131 (4 bytes) — TX to PCU ===== */
typedef struct __attribute__((packed)) {
    /* addr 128 */ uint8_t cmd_power;   // 0=off / 1=on
    /* addr 129 */ uint8_t reserved[3];
} CMD_t;

/* ===== [DIAG] addr 224~255 (32 bytes) ===== */
typedef struct __attribute__((packed)) {
    /* addr 224 */ uint32_t cmd_write_tick;
    /* addr 228 */ uint8_t  reserved[28];
} DIAG_t;

/* ===== 전체 레지스터 맵 — Total 256 bytes ===== */
typedef struct __attribute__((packed)) {
    /* addr   0 */ DEFINE_t     reg_df;
    /* addr  16 */ uint8_t      reserved0[30];
    /* addr  46 */ DATA_SYS_t   sys;
    /* addr  62 */ uint8_t      reserved1[34];
    /* addr  96 */ DATA_POWER_t power;
    /* addr 104 */ uint8_t      reserved2[24];
    /* addr 128 */ CMD_t        cmd;
    /* addr 132 */ uint8_t      reserved3[92];
    /* addr 224 */ DIAG_t       diag;
} REGISTER_t;

static_assert(sizeof(DEFINE_t)     == REG_DEFINE_SIZE,      "DEFINE_t size mismatch");
static_assert(sizeof(DATA_SYS_t)   == REG_SYS_SIZE,         "DATA_SYS_t size mismatch");
static_assert(sizeof(DATA_POWER_t) == REG_DATA_POWER_SIZE,  "DATA_POWER_t size mismatch");
static_assert(sizeof(CMD_t)        == REG_CMD_SIZE,          "CMD_t size mismatch");
static_assert(sizeof(DIAG_t)       == REG_DIAG_SIZE,         "DIAG_t size mismatch");
static_assert(sizeof(REGISTER_t)   == REG_TOTAL_SIZE,        "REGISTER_t total size mismatch");

} // namespace orin_bridge::pcu

#endif
