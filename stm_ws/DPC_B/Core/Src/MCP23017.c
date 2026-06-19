/*
 * MCP23017.c
 *
 *  Created on: Jan 29, 2026
 *      Author: abc01
 */

#include "MCP23017.h"

/* 내부 통신 함수 */
static void EXIO_WriteReg(I2C_HandleTypeDef *hi2c, uint8_t reg, uint8_t data) {
    HAL_I2C_Mem_Write(hi2c, EXIO_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 2);
}

static uint8_t EXIO_ReadReg(I2C_HandleTypeDef *hi2c, uint8_t reg) {
    uint8_t data = 0;
    HAL_I2C_Mem_Read(hi2c, EXIO_ADDR, reg, I2C_MEMADD_SIZE_8BIT, &data, 1, 2);
    return data;
}

/* Input 설정: 방향 1, 풀업 상시 활성화 */
void EXIO_Set_INPUT(I2C_HandleTypeDef *hi2c, uint8_t pin) {
    uint8_t regDir = (pin < 8) ? EXIO_IODIRA : EXIO_IODIRB;
    uint8_t regPu  = (pin < 8) ? EXIO_GPPUA  : EXIO_GPPUB;
    uint8_t bitPos = (pin < 8) ? pin : (pin - 8);

    uint8_t dirVal = EXIO_ReadReg(hi2c, regDir);
    EXIO_WriteReg(hi2c, regDir, dirVal | (1 << bitPos));

    uint8_t puVal = EXIO_ReadReg(hi2c, regPu);
    EXIO_WriteReg(hi2c, regPu, puVal | (1 << bitPos));
}

/* Output 설정: 방향 0, 기본 출력값 0 */
void EXIO_Set_OUTPUT(I2C_HandleTypeDef *hi2c, uint8_t pin) {
    uint8_t regDir = (pin < 8) ? EXIO_IODIRA : EXIO_IODIRB;
    uint8_t regLat = (pin < 8) ? EXIO_OLATA  : EXIO_OLATB;
    uint8_t bitPos = (pin < 8) ? pin : (pin - 8);

    // 기본값 0 세팅
    uint8_t latVal = EXIO_ReadReg(hi2c, regLat);
    EXIO_WriteReg(hi2c, regLat, latVal & ~(1 << bitPos));

    // 방향 설정
    uint8_t dirVal = EXIO_ReadReg(hi2c, regDir);
    EXIO_WriteReg(hi2c, regDir, dirVal & ~(1 << bitPos));
}

/* 핀 제어 */
void EXIO_WritePin(I2C_HandleTypeDef *hi2c, uint8_t pin, uint8_t state) {
    uint8_t regOut = (pin < 8) ? EXIO_OLATA : EXIO_OLATB;
    uint8_t bitPos = (pin < 8) ? pin : (pin - 8);

    uint8_t currentVal = EXIO_ReadReg(hi2c, regOut);
    if(state) currentVal |= (1 << bitPos);
    else      currentVal &= ~(1 << bitPos);

    EXIO_WriteReg(hi2c, regOut, currentVal);
}

/* 핀 읽기 */
uint8_t EXIO_ReadPin(I2C_HandleTypeDef *hi2c, uint8_t pin) {
    uint8_t regIn = (pin < 8) ? EXIO_GPIOA : EXIO_GPIOB;
    uint8_t bitPos = (pin < 8) ? pin : (pin - 8);

    return (EXIO_ReadReg(hi2c, regIn) >> bitPos) & 0x01;
}
