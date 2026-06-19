/*
 * rd_common.h
 *
 *  Created on: Aug 12, 2025
 *      Author: abc01
 */

#ifndef INC_RD_COMMON_H_
#define INC_RD_COMMON_H_

/* Private includes ----------------------------------------------------------*/
#include <stdint.h>

/* Exported types ------------------------------------------------------------*/
typedef uint32_t RD_RET;
/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/
#define RET_OK ((RD_RET)0x00)
#define RET_NOK ((RD_RET)0x01)
#define RET_WAIT ((RD_RET)0x02)

/* ===== Lifecycle State =====
 * 4-bit field (raw[3:0]). packed into STATE_t.bits.lifecycle.
 *
 * 단순화된 채널 lifecycle 모델 (6 상태):
 *   - 상위 FSM (robot_state, SYSTEM_STATE_e) 가 "왜 시스템이 멈췄나" 를 표현하고,
 *     채널 lifecycle 은 "통신/채널 자체가 살아있나" 만 표현한다.
 *   - 따라서 IDLE / PAUSED / SUSPENDED 같은 "외부 요청 정지" 상태는 두지 않는다 —
 *     모두 상위 FSM (ESTOP_SW, MANUAL 등) 의 책임 영역.
 *   - UNINIT 폐기: memset 0 == LS_INIT 으로 자연 초기화. 미구현 모듈 (예: IMU) 은
 *     LS_OFFLINE 으로 명시 발행.
 *
 *      | 값  | 상수          | 설명
 * ----+-----+--------------+----------------------------------------- */
#define LS_INIT         0   /* 부팅 / RECOVERY 후 초기화 진행 중 (memset 0 = 이 값) */
#define LS_READY        1   /* 초기화 완료, 첫 패킷 수신 대기 (핸드셰이크 단계) */
#define LS_RUNNING      2   /* 정상 송수신 중 */
#define LS_DEGRADED     3   /* 통신 동작 중이나 최근 에러율 높음 (decay 카운터 임계 초과) */
#define LS_RECOVERING   4   /* 상위 명령으로 복구 시퀀스 실행 중 (이후 LS_INIT 으로 전이) */
/* 5~14 reserved */
#define LS_OFFLINE      15  /* 복구 불가 / 채널 영구 비활성 (미구현 모듈 포함) */

/* ===== Degraded Counter (LS_DEGRADED 산출용) =====
 * Decay 카운터 기반 "최근 에러 빈도" 추정. 채널마다 uint16 누적기 1개 운용.
 *
 *   매 에러:           cnt = min(cnt + K_<rate>, DEGRADED_CNT_MAX)
 *   매 Checker tick:   if (cnt > 0) cnt -= DEGRADED_TICK_DECAY
 *   진입:              LS_RUNNING  && cnt > DEGRADED_THRESHOLD_HIGH → LS_DEGRADED
 *   복귀:              LS_DEGRADED && cnt < DEGRADED_THRESHOLD_LOW  → LS_RUNNING
 *                      (4× 히스테리시스 갭으로 flapping 방지)
 *
 * 설계 기준 (Checker @ 100Hz):
 *   - 경계 손실률 = 5%   (이 이하는 cnt 자연 감쇠로 0 수렴, DEGRADED 진입 안 함)
 *   - 10% 손실           → ~2.0 sec 후 LS_DEGRADED 진입
 *   - 100% 손실 (단절)   → ~0.10 sec 진입, ~0.5 sec CNT_MAX 포화
 *   - 에러 정지 후 복귀:  cnt=200 → 1.5 sec, cnt=1000 → 9.5 sec
 *
 * 채널 통신 주파수별 K (식: K = 2000 / F_comm — 어느 F_comm 이든 5% 손실 = 중립):
 *   - 100Hz 채널 (예: RS485 master query)  → DEGRADED_K_100HZ  = 20
 *   - 200Hz 채널 (예: I2C AS5600 polling)  → DEGRADED_K_200HZ  = 10
 *   - 250Hz 채널 (예: CAN AK motor TX)     → DEGRADED_K_250HZ  =  8
 *
 *   THRESHOLD / DECAY / CNT_MAX 는 K 와 무관한 공통 상수 —
 *   같은 손실% 면 채널 주파수 무관하게 동일 시간에 DEGRADED 로 들어간다.
 * ------------------------------------------------------------------- */
#define DEGRADED_K_100HZ          20    /* 에러 1건당 +20  (100Hz 통신) */
#define DEGRADED_K_200HZ          10    /* 에러 1건당 +10  (200Hz 통신) */
#define DEGRADED_K_250HZ           8    /* 에러 1건당 +8   (250Hz 통신) */
#define DEGRADED_TICK_DECAY        1    /* Checker tick(20ms) 마다 -2 */
#define DEGRADED_THRESHOLD_HIGH  200    /* RUNNING  → DEGRADED 진입 */
#define DEGRADED_THRESHOLD_LOW    50    /* DEGRADED → RUNNING  복귀 */
#define DEGRADED_CNT_MAX        1000    /* uint16 포화 상한 (복구 latency ≤ 10 sec) */

/* ===== Health Code =====
 * 4-bit field (raw[7:4]). packed into STATE_t.bits.health
 * 심각도 그룹: 정상(0) | 정보(1) | 경고(2~6) | 에러(7~10) | 심각(11~13) | 치명(14~15)
 * -----------------------------------------------------------------------
 *      | 값                | 상수 |    설명                            | 심각도
 * ----+----------------+-----+-------------------------------+---------*/
#define HC_OK               0   /* 결함 없음                       | 정상*/                      
#define HC_INFO             1   /* 통계적 이벤트 (재시도 1회 성공 등)   | 정상*/ 
/*-----+----정상 ( 2~ 6)---+---+----------------------------------+------*/
#define HC_TIMEOUT          2   /* 응답 타임아웃 (재시도로 회복)       | 경고 */
#define HC_CRC_ERR          3   /* 체크섬/CRC 불일치                | 경고*/
#define HC_FRAMING_ERR      4   /* UART framing, CAN form error  | 경고    */
#define HC_OVERRUN          5   /* RX 버퍼/FIFO 오버런              | 경고 */
#define HC_DATA_RANGE       6   /* 디코딩은 됐지만 값이 범위 밖        | 경고 */
/*-----+----에러 ( 7~10)---+---+----------------------------------+------*/
#define HC_PROTOCOL_ERR     7   /* 알 수 없는 명령/잘못된 시퀀스       | 에러 */
#define HC_ACK_FAIL         8   /* I2C NACK, RS485 응답 없음        | 에러 */
#define HC_PARAM_ERR        9   /* 설정값 부정합 (자가진단 실패)        | 에러 */
#define HC_HW_FAULT         10  /* 페리페럴 자체 결함 (HAL_ERROR)     | 에러 */
/*-----+----심각 (11~12)---+---+----------------------------------+------*/
#define HC_BUS_WARNING      11  /* CAN error counter >= 96 등      | 심각 */
#define HC_BUS_PASSIVE      12  /* CAN error passive (>= 128)      | 심각 */
/*-----+----치명 (13~15)---+---+----------------------------------+------*/
#define HC_BUS_OFF          13  /* 물리적 단절, 자동/수동 복구 필요      | 치명 */
#define HC_UNRECOVERABLE    14  /* 해당 채널 완전 실패                 | 치명 */
#define HC_FATAL            15  /* 시스템 차원 결함, WDT reset 권장    | 치명 */

/* ===== Severity Thresholds ===== */
#define HC_THRESHOLD_INFO       1   /* >= 1: 정보 이벤트 */
#define HC_THRESHOLD_WARN       2   /* >= 2: 경고 */
#define HC_THRESHOLD_ERROR      7   /* >= 7: 에러 */
#define HC_THRESHOLD_CRITICAL   11  /* >= 11: 심각 (CAN bus 이상) */
#define HC_THRESHOLD_FATAL      13  /* >= 13: 치명 */

#define CH_GET_LIFECYCLE(raw)   ((raw) & 0x0F)
#define CH_GET_HEALTH(raw)      (((raw) >> 4) & 0x0F)
#define CH_PACK(lc, hc)         (((hc) & 0x0F) << 4 | ((lc) & 0x0F))

typedef union {
    uint8_t raw;
    struct {
        uint8_t lifecycle : 4;
        uint8_t health    : 4;
    } bits;
} STATE_t;

typedef struct {
    STATE_t   state;         /**< lifecycle(bits[3:0]) + health(bits[7:4]) — LS_*|HC_* 참조.
                              *  lifecycle 전이 ownership:
                              *    INIT     → LS_READY
                              *    Checker  → LS_INIT/LS_READY → LS_RUNNING 첫 승격
                              *               LS_RUNNING ↔ LS_DEGRADED  (degraded_cnt 히스테리시스)
                              *               LS_OFFLINE 강제 (HC_FATAL 또는 rx_error_cnt 임계)
                              *    RECOVERY → LS_RECOVERING (진입 표시, 성공 시 INIT 가 LS_READY 로 reset,
                                         	 	 	  실패 시 LS_OFFLINE 으로 강제 전이)
                              *  IDLEHandler/Transmit/RX_Apply 등은 lifecycle 직접 변경 금지. */
    uint32_t  isr_err_code;  /**< HAL_*_ErrorCallback 에서 캡처한 raw HAL 에러코드 */
    uint16_t  rx_error_cnt;  /**< 연속 HAL/packet 에러 누적 카운터 (FATAL OFFLINE 판정용, 단조 증가) */
    uint16_t  tx_error_cnt;  /**< TX 에러 누적 카운터 (디버그용)        */
    uint16_t  degraded_cnt;  /**< LS_DEGRADED 산출용 decay 카운터.
                                       매 에러시 +degraded_k (Checker 인자) / 무에러 tick 시 -DEGRADED_TICK_DECAY.
                                  DEGRADED_THRESHOLD_HIGH 초과 → LS_DEGRADED 진입,
                                  DEGRADED_THRESHOLD_LOW 미만 → LS_RUNNING 복귀 (히스테리시스 4× 갭).
                                  DEGRADED_CNT_MAX 포화. */
} ERROR_STATUS_t;

/* health 상위 nibble이므로 raw 비교 = health 우선, lifecycle 차순. */
static inline STATE_t STATE_WORSE(STATE_t a, STATE_t b) { return (a.raw > b.raw) ? a : b; }

/* M-8: ISR 에러 플래그 atomic read-clear (LDREX/STREX 루프).
 *       read 와 clear 사이에 ISR 가 새 비트를 OR 해도 손실 없음. */
static inline uint32_t isr_err_take(volatile uint32_t *p) {
    return __atomic_exchange_n(p, 0, __ATOMIC_RELAXED);
}

#endif /* INC_RD_COMMON_H_ */
