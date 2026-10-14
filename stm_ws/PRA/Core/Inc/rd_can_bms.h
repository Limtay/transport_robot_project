/*
 * rd_can_bms.h
 *
 *  PRA BMS CAN(CAN1) 드라이버 — 요청→응답 폴링, 단순 로깅 전용.
 *  설계/Step: pra_config_wiki/pra_bms_can.md · 프로토콜: pra_config_wiki/BMS_protocol_can.md
 *
 *  🔒 격리 원칙: 본 모듈은 PRA 동작·전원 FSM·시스템 리붓에 절대 관여하지 않는다.
 *     수신값을 레지스터맵(SENSOR/BMS)에 갱신하는 것만 수행하고, 통신 단절 시
 *     CAN 서브시스템만 자체 재초기화(재시도)한다. (ECU_V3 CAN_RECOVERY 개념)
 *
 *  파일 소유 범위(.c): CAN 필터/Start/Notification, HAL RX/Error weak 콜백,
 *     요청 빌드·RX pop/apply, 디코드→reg, Checker/Recovery, RD_TASK_CAN().
 *  인터럽트(2026-10-10): CAN1 RX0/SCE 는 **.ioc 에서 활성** → IRQ 핸들러=stm32f4xx_it.c,
 *     NVIC enable=HAL_CAN_MspInit 가 소유. 본 파일은 핸들러/NVIC 를 정의하지 않는다.
 *
 *  Created on: 2026-10-10
 *      Author: swarm
 *
 *  === 현재 구현 범위: Step 8-4 (디코드 → 레지스터 발행) ===
 *    Step 8-1: INIT(필터/Start/Notification) + HAL 콜백 + RD_TASK_CAN.
 *    Step 8-2: BMS_Handle_t(ID별 raw 캐시) + CAN_BMS_REQUEST_ALL(3요청 메일박스 직송)
 *              + RX pop/apply(데이터ID 분기 적재 + last_rx_tick latch) + ~1s 요청 폴링.
 *    Step 8-3: RD_CAN_BMS_RECOVERY(CAN 서브시스템만 재초기화) + RD_TASK_CAN 에서
 *              (응답 타임아웃 || HAL 에러) 자기완결 복구 트리거. 시스템/전원 무전파(격리).
 *    Step 8-4: RD_CAN_BMS_UPDATE — 핸들 raw(big-endian) → reg.sensor_bms 발행
 *              (voltage/current[int16,-30000]/soc/max_cell_temp[40offset 보존]/error_lsb·hsb).
 *    Step 8-5: RD_CAN_BMS_CHECKER — 채널 STATE_t(addr61) 발행(로깅 전용, 시스템 무전파).
 *    Step 8-6: delta_tick(addr60) 발행(CHECKER 내). hw_reset(addr5)=미구현(자동복구라 불요).
 *    Orin 연동 실테스트=8-7 / 리비전 보드·실 BMS·과열 체크=8-8.
 */

#ifndef INC_RD_CAN_BMS_H_
#define INC_RD_CAN_BMS_H_

#include "stm32f4xx_hal.h"
#include "rd_common.h"   /* RD_RET */
#include <stdint.h>

/* ── BMS 데이터 ID (요청/응답 대상) ───────────────────────────────────────── */
#define BMS_ID_VIC      0x90u   /* 전압/전류/SOC */
#define BMS_ID_TEMP     0x92u   /* 최고/최저 셀 온도 */
#define BMS_ID_FAULT    0x98u   /* 배터리 고장상태 8B */

/* ── 수신 확인용 디버그 스냅샷 (Step 8-1 — PCAN 더미패킷 FIFO 인입 검증) ───────── */
typedef struct {
    uint32_t ext_id;     /* 수신 프레임 29-bit ExtId (예: 0x18904001)             */
    uint8_t  data_id;    /* (ext_id >> 16) & 0xFF — 0x90 / 0x92 / 0x98            */
    uint8_t  dlc;        /* Data Length Code                                     */
    uint8_t  data[8];    /* 페이로드 원본                                          */
} BMS_RxDebug_t;

/* 디버거로 관찰: 더미패킷 송신 시 BMS_rx_count 증가 + BMS_last_rx 갱신 확인 */
extern volatile uint32_t      BMS_rx_count;       /* FIFO0 수신 프레임 누적        */
extern volatile uint32_t      BMS_err_count;      /* CAN HAL 에러 콜백 누적        */
extern volatile uint32_t      BMS_isr_err_code;   /* 최근 HAL_CAN_ERROR_* OR 캡처  */
extern volatile BMS_RxDebug_t BMS_last_rx;        /* 최근 수신 프레임 스냅샷        */
extern volatile STATE_t       BMS_ch_state;       /* CAN1 채널 상태(lifecycle+health) */

/* ── BMS 핸들 (ID별 응답 raw 캐시) ────────────────────────────────────────── */
typedef struct {
    volatile uint8_t data[8];   /* 최근 응답 페이로드 원본 (ISR write)               */
    volatile uint8_t valid;     /* 1회 이상 수신 플래그                              */
} BMS_Msg_t;

typedef struct {
    BMS_Msg_t         msg_vic;      /* 0x90 전압/전류/SOC                           */
    BMS_Msg_t         msg_temp;     /* 0x92 최고/최저 온도                           */
    BMS_Msg_t         msg_fault;    /* 0x98 고장상태 8B                             */
    volatile uint32_t last_rx_tick; /* 최근(임의 3종) 수신 시각 [HW_NowTick] — 신선도/타임아웃 */
} BMS_Handle_t;

/* RX FIFO 에서 꺼낸 1 프레임 */
typedef struct {
    uint32_t ext_id;    /* 29-bit ExtId                                          */
    uint8_t  data_id;   /* (ext_id >> 16) & 0xFF                                 */
    uint8_t  dlc;
    uint8_t  data[8];
} BMS_RxFrame_t;

extern BMS_Handle_t BMS;   /* 단일 BMS 핸들 (rd_can_bms.c 소유) */

/* delta_tick 체계용 free-run 시계 포트 (현재 HAL_GetTick ms, TIM5 부재) */
uint32_t HW_NowTick(void);

/* CAN1 필터(0x18xx4001 mask)·Start·Notification 설정 + BMS 핸들 clear. 성공 RET_OK */
RD_RET RD_CAN_BMS_INIT(CAN_HandleTypeDef *hcan);

/* CAN 서브시스템만 재초기화(Stop→DeInit→Init→INIT). 통신 단절 시 자기복구용.
 * 🔒 시스템/전원 FSM 에 전파하지 않는다(격리). 성공 RET_OK. */
RD_RET RD_CAN_BMS_RECOVERY(CAN_HandleTypeDef *hcan);

/* BMS 핸들 raw → 디코드(big-endian·오프셋/스케일) → reg.sensor_bms 발행.
 * current 만 오프셋 제거(int16, raw-30000), max_cell_temp 는 40 offset 보존(uint8). RD_TASK_CAN 이 주기 호출. */
void RD_CAN_BMS_UPDATE(void);

/* 채널 상태(STATE_t, lifecycle+health) + delta_tick 계산 → reg.sensor_bms.state(addr61)/
 * delta_tick(addr60) 발행. 🔒 로깅 전용 — 시스템/전원에 전파하지 않는다. RD_TASK_CAN 주기 호출. */
void RD_CAN_BMS_CHECKER(void);

/* 요청 3종(0x90/0x92/0x98, `0x18[id]0140`, 8×0x00)을 TX 메일박스 3개에 직접 적재.
 * 모두 적재 성공 RET_OK / 일부 실패 RET_WAIT / 전부 실패 RET_NOK. */
RD_RET CAN_BMS_REQUEST_ALL(CAN_HandleTypeDef *hcan);

/* RX FIFO0 에서 1 프레임 pop → BMS_RxFrame_t. 성공 1 / 없음·오류 0. (ISR 컨텍스트) */
uint8_t CAN_BMS_RX_POP(CAN_HandleTypeDef *hcan, BMS_RxFrame_t *frame);

/* frame.data_id 가 0x90/0x92/0x98 이면 해당 캐시에 raw 저장 + last_rx_tick latch.
 * 매칭 1 / 비대상 0. (ISR 컨텍스트) */
uint8_t CAN_BMS_RX_APPLY(BMS_Handle_t *h, const BMS_RxFrame_t *frame);

/* can1Task 진입점 — main `Start_can1` 에 유저가 수동으로 `RD_TASK_CAN();` 추가.
 * INIT(실패 시 CAN 만 재시도) → ~1s 주기 REQUEST_ALL + (타임아웃 || HAL 에러) 시 RECOVERY. */
void RD_TASK_CAN(void);

#endif /* INC_RD_CAN_BMS_H_ */
