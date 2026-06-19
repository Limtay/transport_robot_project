/*
 * rd_peripheral.h
 *
 *  Created on: Mar 10, 2026
 *      Author: swarm
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
	volatile uint32_t* per_pCCR;
} GPIO_IO_t;

// typedef struct {
// 	GPIO_IO_t IND_IO;
// 	GPIO_IO_t MODE_IO;
// 	GPIO_IO_t ESTOP_IO;
// } GPIO_IO_ALL_t;

// typedef struct {
// 	volatile uint8_t IND;
// 	volatile uint8_t MODE;
// 	volatile uint8_t ESTOP;

// 	volatile uint8_t IND_cnt;

// 	GPIO_IO_ALL_t IO;
// } GPIO_t;



#endif /* INC_RD_PERIPHERAL_H_ */
