#ifndef ORIN_FIRMWARE_BRIDGE__RD_COMMON_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_COMMON_HPP_

#include <cstdint>

namespace orin_bridge {

typedef enum {
    RD_OK      = 0x00U,
    RD_TIMEOUT = 0x01U,
    RD_ERROR   = 0x02U,
    RD_FATAL   = 0x03U,
    RD_CLOSED  = 0x04U
} RD_RET;

// ===== STATE_t: stm_ws/ECU_V3/Core/Inc/rd_common.h 와 동일 =====
// lifecycle[3:0]: LS_INIT(0) / LS_READY(1) / LS_RUNNING(2) / LS_DEGRADED(3) / LS_RECOVERING(4) / LS_OFFLINE(15)
// health[7:4]  : HC_OK(0) ... HC_FATAL(15)
typedef union {
    uint8_t raw;
    struct {
        uint8_t lifecycle : 4;
        uint8_t health    : 4;
    } bits;
} STATE_t;

// Lifecycle constants
#define LS_INIT         0
#define LS_READY        1
#define LS_RUNNING      2
#define LS_DEGRADED     3
#define LS_RECOVERING   4
#define LS_OFFLINE      15

// Health code constants
#define HC_OK               0
#define HC_INFO             1
#define HC_TIMEOUT          2
#define HC_CRC_ERR          3
#define HC_FRAMING_ERR      4
#define HC_OVERRUN          5
#define HC_DATA_RANGE       6
#define HC_PROTOCOL_ERR     7
#define HC_ACK_FAIL         8
#define HC_PARAM_ERR        9
#define HC_HW_FAULT         10
#define HC_BUS_WARNING      11
#define HC_BUS_PASSIVE      12
#define HC_BUS_OFF          13
#define HC_UNRECOVERABLE    14
#define HC_FATAL            15

// STATE_t 조작 매크로
#define CH_GET_LIFECYCLE(raw)   ((raw) & 0x0F)
#define CH_GET_HEALTH(raw)      (((raw) >> 4) & 0x0F)
#define CH_PACK(lc, hc)         (((hc) & 0x0F) << 4 | ((lc) & 0x0F))

// hw_error / hw_fatal 비트 정의 (DATA_SYSTEM_t.hw_error / hw_fatal)
// 비트 순서는 STM 이 정한다 — 03 §3.1 의 채널 순서와 같다 (bit2 는 **uart6**, IMU).
// 구 이름이 `HW_BIT_UART4` 였다: 슬롯이 uart4 → uart6 으로 옮겨졌는데 매크로만 남았다.
// ⚠ 이 매크로군은 정의만 있고 **사용처가 없다.** 그래서 이름이 틀린 채로 오래 남았고,
//   앞으로도 테스트가 잡아 주지 못한다 — 쓰기 시작할 때 이 주석을 지울 것.
#define HW_BIT_UART1    (1u << 0)
#define HW_BIT_UART2    (1u << 1)
#define HW_BIT_UART6    (1u << 2)
#define HW_BIT_CAN1     (1u << 3)
#define HW_BIT_I2C1     (1u << 4)

} // namespace orin_bridge

#endif