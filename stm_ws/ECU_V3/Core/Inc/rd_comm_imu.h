/*
 * rd_comm_imu.h
 *
 *  Created on: Jun 10, 2026
 *      Author: swarm
 */

#ifndef INC_RD_COMM_IMU_H_
#define INC_RD_COMM_IMU_H_

/* Private includes ----------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "rd_common.h"
#include "rd_define.h"
#include "rd_uart.h"

/* Exported macro ------------------------------------------------------------*/
// 헤더 정의

/* Exported types ------------------------------------------------------------*/

typedef struct __attribute__((packed)){
	// [5:12] Data (8 Bytes)
	uint16_t Data[14];

} IMU_comm_s_t;

typedef struct {
	IMU_comm_s_t packet;
} IMU_comm_t;
/* Exported constants --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
RD_RET RD_IMU_INIT(IMU_comm_t *imu_obj);
RD_RET RD_IMU_READ(UART_Ring_t *uart_obj, IMU_comm_t *imu_obj);

#endif /* INC_RD_COMM_IMU_H_ */
