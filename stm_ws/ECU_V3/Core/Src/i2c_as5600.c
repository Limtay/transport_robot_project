/**
 ******************************************************************************
 * @file    i2c_as5600.c
 * @author  Kyeongtae
 * @date    2026-02-14
 * @brief   AS5600 + TCA9548A I2C MUX 드라이버 구현부.
 *
 *  파일 구성
 *  -----------------------------------------------------------------------
 *    [Static helpers]  MUX_SelectChannel / AS5600_ReadAngle
 *    [Lifecycle]       AS5600_INIT
 *    [Update]          AS5600_UPDATE — 핵심 진입점, AS5600_Status_e 반환
 *
 *  타이밍 메모
 *  -----------------------------------------------------------------------
 *    HAL_I2C_* 호출의 timeout = 2 ms. 5채널을 폴링해도 worst-case 10 ms 이내.
 *    상위 태스크 10ms 주기 안에서 안전.
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "i2c_as5600.h"

/* Static function prototypes ------------------------------------------------*/
static HAL_StatusTypeDef MUX_SelectChannel(I2C_HandleTypeDef *hi2c, uint8_t channel);
static HAL_StatusTypeDef AS5600_ReadAngle (I2C_HandleTypeDef *hi2c, uint8_t reg, uint16_t *angle_count);

/* ============================================================================
 *                            Static helpers
 * ========================================================================== */

/**
 * @brief  TCA9548A 1바이트 컨트롤 레지스터에 비트마스크를 써서 채널 선택.
 *         channel >= 8 이면 HAL_ERROR (호출자 잘못).
 */
static HAL_StatusTypeDef MUX_SelectChannel(I2C_HandleTypeDef *hi2c, uint8_t channel) {
    if (channel >= 8) return HAL_ERROR;
    uint8_t data = (uint8_t)(1U << channel);
    return HAL_I2C_Master_Transmit(hi2c, I2C_MUX_ADDR, &data, 1, 2);
}

/**
 * @brief  AS5600에서 12bit 각도 2byte를 빅엔디안으로 읽어 정수로 변환.
 *         실패 시 *angle_count 는 갱신하지 않음.
 */
static HAL_StatusTypeDef AS5600_ReadAngle(I2C_HandleTypeDef *hi2c, uint8_t reg, uint16_t *angle_count) {
    uint8_t buf[2] = {0, 0};
    HAL_StatusTypeDef status =
        HAL_I2C_Mem_Read(hi2c, AS5600_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT, buf, 2, 2);

    if (status == HAL_OK) {
        *angle_count = ((uint16_t)buf[0] << 8) | buf[1];
    }
    return status;
}

/* ============================================================================
 *                              Lifecycle
 * ========================================================================== */

void AS5600_INIT(AS5600_Handle_t *pEncoder, I2C_HandleTypeDef *hi2c, uint8_t mux_channel) {
    pEncoder->hi2c        = hi2c;
    pEncoder->mux_channel = mux_channel;
    pEncoder->raw_angle   = 0;
    pEncoder->err_cnt     = 0;
}

/* ============================================================================
 *                                Update
 * ========================================================================== */

/**
 * @brief  3단계로 동작:
 *           1) MUX 채널 선택 → 실패 시 즉시 AS5600_ERR_MUX 반환 (상위로 위임).
 *           2) AS5600_REG_ANGLE 에서 각도 read → 실패 시 err_cnt++ → ERR_ENC.
 *           3) 성공 시 raw_angle 갱신, err_cnt = 0, AS5600_OK.
 *
 *  err_cnt 는 모듈이 자동 관리하지만 fault 판정 임계(ENC_ERR_THRESHOLD)는
 *  헤더에 정의되어 있고 비교는 상위 레이어가 수행.
 */
AS5600_Status_e AS5600_UPDATE(AS5600_Handle_t *pEncoder) {
    /* 1) MUX 선택 실패는 상위 레이어 책임 — 모듈 카운터 안 건드림. */
    if (MUX_SelectChannel(pEncoder->hi2c, pEncoder->mux_channel) != HAL_OK) {
        return AS5600_ERR_MUX;
    }

    /* 2) 엔코더 read 실패 — 일시적일 수 있어 카운터만 증가시키고 반환. */
    uint16_t temp_angle = 0;
    if (AS5600_ReadAngle(pEncoder->hi2c, AS5600_REG_ANGLE, &temp_angle) != HAL_OK) {
        if (pEncoder->err_cnt < 0xFFFF) pEncoder->err_cnt++;
        return AS5600_ERR_ENC;
    }
    /* 3) 성공 — raw 갱신 + 카운터 리셋. */
    pEncoder->raw_angle = temp_angle;
    pEncoder->err_cnt   = 0;
    return AS5600_OK;
}
