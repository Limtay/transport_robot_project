/*
 * rd_peripheral.h
 *
 *  Created on: 2026. 2. 24.
 *      Author: Lenovo
 */

#ifndef INC_RD_PERIPHERAL_H_
#define INC_RD_PERIPHERAL_H_

/* Private includes ----------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
/* Exported types ------------------------------------------------------------*/

// GPIO Setting -------------------------------
typedef struct {
	GPIO_TypeDef* per_GPIOx;
	uint16_t per_GPIO_Pin;
	uint32_t* per_pCCR;
} GPIO_IO_t;

typedef struct {
	GPIO_IO_t IND_IO;
	GPIO_IO_t MODE_IO;
	GPIO_IO_t ESTOP_IO;
} GPIO_IO_ALL_t;

/* GPIO 는 EXTI 가 아니라 태스크 폴링(RD_PERIPHERAL_READ)으로 취득 —
 * ISR 접근이 없는 task↔task 공유라 volatile 불필요 (2026-07-17 정리). */
typedef struct {
	uint8_t IND;
	uint8_t MODE;
	uint8_t ESTOP;

	uint8_t IND_cnt;

	GPIO_IO_ALL_t IO;
} GPIO_t;

#endif /* INC_RD_PERIPHERAL_H_ */
