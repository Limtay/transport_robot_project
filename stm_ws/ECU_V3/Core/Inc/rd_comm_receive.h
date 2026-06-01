/*
 * rd_comm_recive.h
 *
 *  Created on: 2026. 2. 14.
 *      Author: Kyeongtae
 */

#ifndef INC_RD_COMM_RECEIVE_H_
#define INC_RD_COMM_RECEIVE_H_

/* Private includes ----------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "rd_common.h"
#include "rd_define.h"
#include "rd_uart.h"

/* Exported macro ------------------------------------------------------------*/
// 헤더 정의
#define RECEIVE_HEADER1  0x20
#define RECEIVE_HEADER2  0x40

#define RECEIVE_PACKET_SIZE 32
#define RECEIVE_DATA_CHANNELS 14
/* Exported types ------------------------------------------------------------*/

typedef struct __attribute__((packed)){
	// [0:1] Header (2 Bytes)
	uint16_t Header;       // 0x40
	// [5:12] Data (8 Bytes)
	uint16_t Data[14];
	// [13] Checksum (1 Byte)
	uint16_t Checksum;
} RECEIVE_comm_s_t;

typedef struct {
	RECEIVE_comm_s_t packet;
	volatile int thrr1;
	volatile int diff1;
	volatile int thrr2;
	volatile int diff2;
	
	volatile uint8_t receive_flag;
	volatile uint8_t mode_flag;

	volatile uint8_t selector[2];
} RECEIVE_comm_t;
/* Exported constants --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
RD_RET RD_RECEIVE_INIT(RECEIVE_comm_t *receive_obj);
RD_RET RD_RECEIVE_READ(UART_Ring_t *uart_obj, RECEIVE_comm_t *receive_obj);

#endif /* INC_RD_COMM_RECEIVE_H_ */

