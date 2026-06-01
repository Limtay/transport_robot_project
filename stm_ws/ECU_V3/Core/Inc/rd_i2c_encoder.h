/**
 ******************************************************************************
 * @file    rd_i2c_encoder.h
 * @author  Kyeongtae
 * @date    2026-05-28
 * @brief   I2C AS5600 엔코더 드라이버 래퍼 — DATA_ENCODER_t 직접 갱신.
 *
 *  reg.encoder 와 1:1 mapping 되는 DATA_ENCODER_t 를 직접 갱신.
 *  MARSHAL_PUBLISH 는 data_ecd → reg.encoder 단순 memcpy 로 완성.
 *
 *  3 함수 인터페이스:
 *    INIT    : I2C 핸들 + AS5600_Enc[] 초기화
 *    UPDATE  : AS5600_UPDATE × 5 + offset 보정 → data->encoder[5]
 *    CHECKER : MUX/채널 에러 + ISR HAL 에러 + DEGRADED decay → data->state, err->i2c.state
 ******************************************************************************
 */

#ifndef INC_RD_I2C_ENCODER_H_
#define INC_RD_I2C_ENCODER_H_

#include "rd_peripheral_ecu.h"

RD_RET RD_I2C_ENCODER_INIT(I2C_HandleTypeDef *hi2c);

/* UPDATE — 5ms 타임가드 초과 시 RET_NOK (MUX 장애 가능성, CHECKER 가 처리) */
RD_RET RD_I2C_ENCODER_UPDATE(volatile DATA_ENCODER_t *data, volatile PERIPHERAL_ERROR_t *err);

/* CHECKER — encoder 채널 에러 + MUX + ISR HAL 에러 종합.
 *  쓰는 필드:
 *    data->state              worst-of-N STATE_t (err->i2c.state mirror)
 *    err->i2c.{state, isr_err_code, rx_error_cnt, degraded_cnt}
 *    err->i2c_rx_cnt[NUM_ENCODERS]  raw 카운터 캐시
 *    err->mux_rx_cnt          MUX 실패 카운터 캐시
 */
RD_RET RD_I2C_ENCODER_CHECKER(volatile DATA_ENCODER_t *data, volatile PERIPHERAL_ERROR_t *err);

#endif /* INC_RD_I2C_ENCODER_H_ */
