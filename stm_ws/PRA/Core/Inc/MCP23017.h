/*
 * MCP23017.h
 *
 *  Created on: Jan 29, 2026
 *      Author: abc01
 */

#ifndef INC_MCP23017_H_
#define INC_MCP23017_H_

#include "stm32f4xx_hal.h" // 사용하시는 STM32 시리즈에 맞게 변경 (예: stm32l4xx_hal.h)

/* MCP23017 Slave Address (A0,A1,A2 -> GND) */
#define EXIO_ID           0x20
#define EXIO_ADDR         (EXIO_ID << 1)

/* Register Addresses (BANK 0) - 충돌 방지를 위해 EXIO_ 접두사 추가 */
#define EXIO_IODIRA       0x00
#define EXIO_IODIRB       0x01
#define EXIO_GPPUA        0x0C
#define EXIO_GPPUB        0x0D
#define EXIO_GPIOA        0x12
#define EXIO_GPIOB        0x13
#define EXIO_OLATA        0x14
#define EXIO_OLATB        0x15

/* Function Prototypes */
void EXIO_Set_INPUT(I2C_HandleTypeDef *hi2c, uint8_t pin);
void EXIO_Set_OUTPUT(I2C_HandleTypeDef *hi2c, uint8_t pin);
void EXIO_WritePin(I2C_HandleTypeDef *hi2c, uint8_t pin, uint8_t state);
uint8_t EXIO_ReadPin(I2C_HandleTypeDef *hi2c, uint8_t pin);

#endif /* INC_MCP23017_H_ */
