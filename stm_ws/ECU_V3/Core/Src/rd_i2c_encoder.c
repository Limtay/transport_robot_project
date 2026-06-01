/**
 ******************************************************************************
 * @file    rd_i2c_encoder.c
 * @author  Kyeongtae
 * @date    2026-05-28
 * @brief   I2C AS5600 엔코더 드라이버 래퍼 — DATA_ENCODER_t 직접 갱신.
 ******************************************************************************
 */

#include "rd_i2c_encoder.h"
#include <string.h>

/* ── 전역 — AS5600_Enc 단독 정의 ────────────────────────────────────────── */
AS5600_Handle_t AS5600_Enc[NUM_ENCODERS];

static uint8_t any_enc_err;
static uint8_t any_running;
/* ── 모듈 전용 상수 및 상태 ─────────────────────────────────────────────── */
static const uint16_t Enc_init_value[NUM_ENCODERS] = {3521, 881, 3846, 2312, 3840};

/* ── 공개 함수 ─────────────────────────────────────────────────────────── */

RD_RET RD_I2C_ENCODER_INIT(I2C_HandleTypeDef *hi2c)
{
    if (hi2c == NULL) return RET_NOK;
    for (int i = 0; i < NUM_ENCODERS; i++) {
        AS5600_INIT(&AS5600_Enc[i], hi2c, i);   /* MUX 채널 0~4 */
    }
    any_running = 0;
    return RET_OK;
}

RD_RET RD_I2C_ENCODER_UPDATE(volatile DATA_ENCODER_t *data, volatile PERIPHERAL_ERROR_t *err)
{
    if (data == NULL) return RET_NOK;

    any_enc_err = 0;
    for (int i = 0; i < NUM_ENCODERS; i++) {
        AS5600_Status_e status = AS5600_UPDATE(&AS5600_Enc[i]);
        if (status == AS5600_ERR_MUX) {
        	err->mux_rx_cnt++;
        	any_enc_err++;
        }else if (status == AS5600_ERR_ENC) {
        	err->mux_rx_cnt = 0;
        	err->i2c_rx_cnt[i]++;
        	any_enc_err++;
        } else {
        	err->mux_rx_cnt = 0;
        	err->i2c_rx_cnt[i] = 0;
        	any_running = 1;
        }
    }

    if(any_enc_err == NUM_ENCODERS) return RET_NOK;
    for (int i = 0; i < NUM_ENCODERS; i++) {
        data->encoder[i] =
            (uint16_t)((ENC_RAW_MAX + AS5600_Enc[i].raw_angle - Enc_init_value[i]) % ENC_RAW_MAX);
    }
    return (any_enc_err == 0) ? RET_OK : RET_WAIT;
}

/* ── CHECKER ──────────────────────────────────────────────────────────── */

RD_RET RD_I2C_ENCODER_CHECKER(volatile DATA_ENCODER_t *data, volatile PERIPHERAL_ERROR_t *err)
{
    if (data == NULL || err == NULL) return RET_NOK;

    uint8_t health    = HC_OK;
    uint8_t lifecycle = err->i2c.state.bits.lifecycle;

    /* RECOVERING / OFFLINE 일 때는 상위가 RECOVERY 호출 전까지 검사 skip
     * (rd_uart.c CHECKER 와 동일 패턴) */
    if (lifecycle == LS_RECOVERING) return RET_WAIT;
    if (lifecycle == LS_OFFLINE)    return RET_NOK;

    /* 1. ISR 캡처 HAL 에러 — atomic read-clear */
    uint32_t hal_err = isr_err_take(&err->i2c.isr_err_code);
    if (hal_err != 0) {
        switch (hal_err) {
			case HAL_I2C_ERROR_AF:    health = HC_ACK_FAIL;
			case HAL_I2C_ERROR_BERR:  health = HC_BUS_WARNING;
			case HAL_I2C_ERROR_OVR:   health = HC_OVERRUN;
			default : 				  health = HC_PROTOCOL_ERR;
        }
        err->i2c.rx_error_cnt++;
    } else {
        err->i2c.rx_error_cnt = 0;
    }

    /* 3. health 가중치 */
    if (health != HC_OK) {
		if (any_enc_err == 5) 	  health = HC_HW_FAULT;
		else if (any_enc_err > 0) health = HC_TIMEOUT;
    }

    /* 4. degraded counter — 100Hz i2cTask 기준 K=20 */
    if (health != HC_OK) {
        uint32_t next = (uint32_t)err->i2c.degraded_cnt + DEGRADED_K_100HZ;
        err->i2c.degraded_cnt = (next > DEGRADED_CNT_MAX) ? DEGRADED_CNT_MAX : (uint16_t)next;
    } else if (err->i2c.degraded_cnt > 0) {
        err->i2c.degraded_cnt = (err->i2c.degraded_cnt > DEGRADED_TICK_DECAY)
                                ? (err->i2c.degraded_cnt - DEGRADED_TICK_DECAY) : 0;
    }

    /* 5. Lifecycle 전이 */
    if (health == HC_OK && lifecycle == LS_READY && any_running) lifecycle = LS_RUNNING;
    if (lifecycle == LS_DEGRADED && err->i2c.degraded_cnt == DEGRADED_CNT_MAX)
    	lifecycle = LS_OFFLINE;
    else if (lifecycle == LS_RUNNING  && err->i2c.degraded_cnt >= DEGRADED_THRESHOLD_HIGH)
        lifecycle = LS_DEGRADED;
    else if (lifecycle == LS_DEGRADED && err->i2c.degraded_cnt <= DEGRADED_THRESHOLD_LOW)
        lifecycle = LS_RUNNING;

    /* 6. 즉시 OFFLINE 트리거 — 치명 단발 OR HAL 에러 누적 임계 초과 */
    if (health >= HC_THRESHOLD_FATAL || err->i2c.rx_error_cnt > HAL_FATAL_CNT_TH) {
        lifecycle = LS_OFFLINE;
        if (health < HC_FATAL) health = HC_FATAL;
    }

    /* 7. state 갱신 — ERROR_STATUS_t.i2c.state + data->state mirror */
    err->i2c.state.bits.health    = health;
    err->i2c.state.bits.lifecycle = lifecycle;
    data->state                   = err->i2c.state;

    if (lifecycle == LS_OFFLINE)        return RET_NOK;   /* 복구 필요 */
    if (health   >= HC_THRESHOLD_WARN)  return RET_WAIT;  /* 경고/에러 — 상위 레이어가 모니터링 */
    return RET_OK;
}
