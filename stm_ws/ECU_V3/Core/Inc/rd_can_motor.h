/**
 ******************************************************************************
 * @file    rd_can_motor.h
 * @author  Kyeongtae
 * @date    2026-05-28
 * @brief   CAN AK 모터 드라이버 래퍼 — rd_peripheral_ecu 분리 레이어.
 *
 *  reg.motor_data 와 1:1 mapping 되는 DATA_MOTOR_t 를 직접 갱신한다.
 *  MARSHAL_PUBLISH 는 data_mtr → reg.motor_data 단순 memcpy 로 완성.
 *
 *  4 함수 인터페이스:
 *    INIT     : CAN 페리페럴 + ECU_AK[] 초기화
 *    UPDATE   : ECU_AK[].state → data_mtr 의 position/velocity/current/temp 복사
 *    TRANSMIT : CMD_MOTOR_t 스냅샷 → ECU_AK[].cmd → CAN_AK_WRITE
 *    CHECKER  : CAN_AK_CHECKER + DEGRADED decay + data_mtr.error_code/comm_err/state 갱신
 *
 *  소유 규칙 (Threading):
 *    ECU_AK[4] 는 이 모듈이 단독 정의.
 *    UPDATE / TRANSMIT / CHECKER 는 controlTask / systemTask 단독 호출.
 *    ISR ↔ task 공유 필드 (state/error) 는 volatile + 단일 store 패턴.
 ******************************************************************************
 */

#ifndef INC_RD_CAN_MOTOR_H_
#define INC_RD_CAN_MOTOR_H_

#include "rd_peripheral_ecu.h"

/* 활성 모터 피드백 신선도 임계 (H1) — AK 모터는 명령 무관 100Hz 상시 피드백 송신 (사용자 설정).
 * ALL_READY 게이트(motor_on 전제)와 주행 중 통신 상실 판정(→ESTOP_SW)이 공용. */
#define MOTOR_COMM_FAULT_MS  100

/* INIT — CAN_Init + 4 모터 CAN_AK_INIT (CAN_ID = 1~4) + err.can 상태머신 초기화 */
RD_RET RD_CAN_MOTOR_INIT(CAN_HandleTypeDef *hcan, volatile PERIPHERAL_ERROR_t *err);

/* UPDATE — ECU_AK[].state → DATA_MOTOR_t 의 raw 채널 데이터 복사.
 *           error_code / comm_err / state 는 CHECKER 가 담당. */
RD_RET RD_CAN_MOTOR_UPDATE(volatile DATA_MOTOR_t *data);

/* TRANSMIT — CMD_MOTOR_t 스냅샷 → ECU_AK[].cmd → CAN_AK_WRITE.
 *            taskENTER_CRITICAL 로 cmd 스냅샷을 떠서 race 회피.
 *            실제 TX 대상 = motor_mask & READY_MASK (A, 2026-08-03) —
 *            응답이 없는 모터에는 프레임을 내지 않는다 (빈 버스 ACK 폭주 원천 차단).
 *            드라이버 불변식으로 두어 ESTOP 등 어떤 상위 경로도 우회할 수 없게 한다. */
RD_RET RD_CAN_MOTOR_TRANSMIT(const CMD_MOTOR_t *cmd, uint8_t motor_mask);

/* READY_MASK — mask 된 모터 중 피드백이 MOTOR_COMM_FAULT_MS 이내로 신선한 모터 비트필드.
 *              TX 대상 산출(A) + ALL_READY 판정의 공용 기반. */
uint8_t RD_CAN_MOTOR_READY_MASK(uint8_t motor_mask);

/* ALL_READY — mask 된 "전" 모터의 상시 피드백이 MOTOR_COMM_FAULT_MS 이내로 신선한가.
 *             ① motor_on 전제조건 (존재 게이트: 모터 전원 전 TX 미개시 — 전원 순서 무해화)
 *             ② 구동 중 !ALL_READY → motor_fault 로 ESTOP_SW (자동복귀형, systemTask)
 *             채널 escalation (DEGRADED/OFFLINE) 과 분리된 per-motor 경로. */
uint8_t RD_CAN_MOTOR_ALL_READY(uint8_t motor_mask);

/* CHECKER — per-motor CAN_AK_CHECKER 호출 + DEGRADED decay + fatal_cnt hysteresis.
 *  쓰는 필드:
 *    data->error_code   pack4(AK_State_t.error_code) [4bit×4]
 *    data->comm_err     pack4_comm(AK_COMM_ERR_t)    [2bit×4]
 *    data->state        worst-of-4 STATE_t (err->can.state mirror)
 *    err->can.{state, isr_err_code, rx_error_cnt, degraded_cnt}
 *    err->can_rx_cnt[i] / can_tx_cnt[i]  raw 카운터 캐시 (DIAG 발행용)
 *
 *  motor_mask 제외 모터는 타임아웃/에러/worst 집계 제외 (발행 필드 0).
 *  per-motor RX 타임아웃은 채널 health/degraded 에 반영하지 않음 (H1) —
 *  채널 escalation 은 HAL/버스 에러 전용, 모터 무응답 정지는 ALL_READY 가 담당.
 *
 *  ever_seen 래치 (C, 2026-08-03): mask 된 모터 중 한 번이라도 응답을 본 적이 있어야
 *  (err->motor_seen) 채널 escalation(RUNNING 승격 / 즉시 OFFLINE)을 허용한다.
 *  "아직 안 켜짐(부팅 순서·늦은 응답)" 과 "보이다가 사라짐(진짜 이상)" 을 의미로 구분 —
 *  전자는 health 만 보고하며 조용히 READY 대기, 주행 차단은 ALL_READY 게이트가 담당.
 *
 * @param  data     DATA_MOTOR_t (in/out)
 * @param  err      PERIPHERAL_ERROR_t (in/out)
 * @param  motor_mask 활성 모터 비트필드 (reg addr 192 스냅샷)
 */
RD_RET RD_CAN_MOTOR_CHECKER(volatile DATA_MOTOR_t *data, volatile PERIPHERAL_ERROR_t *err, uint8_t motor_mask);

/* RECOVERY — CAN_RECOVERY + 4 모터 카운터/타임스탬프 리셋 (상위 Checker 가 RET_NOK 시 호출) */
RD_RET RD_CAN_MOTOR_RECOVERY(PERIPHERAL_t *peripheral, volatile PERIPHERAL_ERROR_t *err);

#endif /* INC_RD_CAN_MOTOR_H_ */
