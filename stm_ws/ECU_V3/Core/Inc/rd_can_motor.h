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

/* INIT — CAN_Init + 4 모터 CAN_AK_INIT (CAN_ID = 1~4) */
RD_RET RD_CAN_MOTOR_INIT(CAN_HandleTypeDef *hcan);

/* UPDATE — ECU_AK[].state → DATA_MOTOR_t 의 raw 채널 데이터 복사.
 *           error_code / comm_err / state 는 CHECKER 가 담당. */
RD_RET RD_CAN_MOTOR_UPDATE(volatile DATA_MOTOR_t *data);

/* TRANSMIT — CMD_MOTOR_t 스냅샷 → ECU_AK[].cmd → CAN_AK_WRITE.
 *            taskENTER_CRITICAL 로 cmd 스냅샷을 떠서 race 회피. */
RD_RET RD_CAN_MOTOR_TRANSMIT(const volatile CMD_MOTOR_t *cmd);

/* CHECKER — per-motor CAN_AK_CHECKER 호출 + DEGRADED decay + fatal_cnt hysteresis.
 *  쓰는 필드:
 *    data->error_code   pack4(AK_State_t.error_code) [4bit×4]
 *    data->comm_err     pack4_comm(AK_COMM_ERR_t)    [2bit×4]
 *    data->state        worst-of-4 STATE_t (err->can.state mirror)
 *    err->can.{state, isr_err_code, rx_error_cnt, degraded_cnt}
 *    err->can_rx_cnt[i] / can_tx_cnt[i]  raw 카운터 캐시 (DIAG 발행용)
 *
 * @param  data     DATA_MOTOR_t (in/out)
 * @param  err      PERIPHERAL_ERROR_t (in/out)
 */
RD_RET RD_CAN_MOTOR_CHECKER(volatile DATA_MOTOR_t *data, volatile PERIPHERAL_ERROR_t *err);

#endif /* INC_RD_CAN_MOTOR_H_ */
