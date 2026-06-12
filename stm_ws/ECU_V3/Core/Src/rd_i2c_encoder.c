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

/* SCL/SDA 토글용 짧은 지연 (~5us @84MHz) — bus-clear half-clock. */
static inline void i2c_busclr_delay(void) { for (volatile uint32_t d = 0; d < 500U; d++) { __NOP(); } }

RD_RET RD_I2C_ENCODER_RECOVERY(I2C_HandleTypeDef *hi2c, volatile PERIPHERAL_ERROR_t *err)
{
    if (hi2c == NULL || err == NULL) return RET_NOK;

    /* 진입 표시 — CHECKER 는 LS_RECOVERING 을 보호(검사 skip). */
    err->i2c.state.bits.lifecycle = LS_RECOVERING;

    /* 1) 페리페럴 정지 (MspDeInit 가 PB8/PB9 의 AF 를 해제) */
    HAL_I2C_DeInit(hi2c);

    /* 2) 버스 클리어 — SCL(PB8)/SDA(PB9) 를 open-drain GPIO 로 두고 SCL 9클럭 토글 +
     *    STOP 으로, SDA 를 잡고 멈춘(clock-stretch/hang) 슬레이브를 강제 해제. */
    __HAL_RCC_GPIOB_CLK_ENABLE();
    GPIO_InitTypeDef gpio = {0};
    gpio.Pin   = GPIO_PIN_8 | GPIO_PIN_9;   /* PB8=SCL, PB9=SDA */
    gpio.Mode  = GPIO_MODE_OUTPUT_OD;
    gpio.Pull  = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOB, &gpio);

    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);   /* SDA 해제(릴리즈) */
    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET); /* SCL low  */
        i2c_busclr_delay();
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);   /* SCL high */
        i2c_busclr_delay();
        if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9) == GPIO_PIN_SET) break; /* SDA 풀림 */
    }
    /* STOP: SCL high 상태에서 SDA low→high */
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
    i2c_busclr_delay();
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
    i2c_busclr_delay();

    /* 3) 페리페럴 재구성 (MspInit 가 PB8/PB9 를 AF4 로 복원) */
    if (HAL_I2C_Init(hi2c) != HAL_OK) {
        err->i2c.state.bits.lifecycle = LS_OFFLINE;   /* 무한 재시도 방지 */
        err->i2c.state.bits.health    = HC_FATAL;
        return RET_NOK;
    }

    /* 4) 엔코더 핸들 + 상태머신 완전 리셋 */
    for (int i = 0; i < NUM_ENCODERS; i++) AS5600_INIT(&AS5600_Enc[i], hi2c, i);
    err->i2c.rx_error_cnt = 0;
    err->i2c.degraded_cnt = 0;
    err->i2c.isr_err_code = 0;
    err->mux_rx_cnt       = 0;
    for (int i = 0; i < NUM_ENCODERS; i++) err->i2c_rx_cnt[i] = 0;
    any_enc_err = 0;
    any_running = 0;
    err->i2c.state.bits.lifecycle = LS_READY;
    err->i2c.state.bits.health    = HC_OK;
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

    /* 1. ISR 캡처 HAL 에러 — atomic read-clear.
     *    HAL 에러코드는 비트마스크이므로 & 로 검사 (우선순위 if-chain).
     *    ※ AS5600 는 블로킹 I2C 라 보통 ErrorCallback 이 안 불려 hal_err==0 이지만,
     *       향후 IT/DMA 전환 대비해 분류는 유지한다. */
    uint32_t hal_err = isr_err_take(&err->i2c.isr_err_code);
    if (hal_err != 0) {
        if      (hal_err & HAL_I2C_ERROR_BERR) health = HC_BUS_WARNING;
        else if (hal_err & HAL_I2C_ERROR_OVR)  health = HC_OVERRUN;
        else if (hal_err & HAL_I2C_ERROR_AF)   health = HC_ACK_FAIL;
        else if (hal_err & HAL_I2C_ERROR_DMA)  health = HC_HW_FAULT;
        else                                   health = HC_PROTOCOL_ERR;
        err->i2c.rx_error_cnt++;
    } else {
        err->i2c.rx_error_cnt = 0;
    }

    /* 2. 블로킹 폴링 read 실패가 주 검출 경로 — any_enc_err 로 health 직접 산출
     *    (hal_err 유무와 무관해야 함! 기존엔 hal_err!=0 안에 갇혀 인코더 실패가
     *     영영 감지 안 됐음). 전 채널 실패 = MUX/버스 단위 장애로 간주. */
    if (health == HC_OK) {
        if      (any_enc_err >= NUM_ENCODERS) health = HC_HW_FAULT;  /* 전 채널/MUX 실패 */
        else if (any_enc_err > 0)             health = HC_TIMEOUT;   /* 일부 채널 실패 */
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
    /* READY → RUNNING 승격:
     *   - health OK + 수신 이력(any_running) : 정상 승격
     *   - health != OK : 에러 = 버스가 동작은 했다는 뜻 → RUNNING 으로 승격해
     *     RUNNING→DEGRADED→OFFLINE→RECOVERY escalation 경로를 타게 한다.
     *     (READY 에 머문 채 health 만 에러로 굳어 복구가 영영 안 걸리는 freeze 방지) */
    if (lifecycle == LS_READY && (any_running || health != HC_OK)) lifecycle = LS_RUNNING;
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
