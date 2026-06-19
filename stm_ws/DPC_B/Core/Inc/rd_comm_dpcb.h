/*
 * rd_comm_dpcb.h
 *
 *  Created on: Jan 26, 2026
 *      Author: abc01
 */

#ifndef INC_RD_COMM_DPCB_H_
#define INC_RD_COMM_DPCB_H_

/* Private includes ----------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "rd_common.h"
#include "rd_define.h"
#include "rd_uart.h"

/* Exported macro ------------------------------------------------------------*/

#define PACKET_HEADER  0x40
#define PACKET_LENGTH 4
#define DATA_LENGTH	2

#define COMM_TIMEOUT 1000 //ms

/* Exported types ------------------------------------------------------------*/




typedef struct {
	uint8_t Header;
	uint8_t Data[DATA_LENGTH];
	uint8_t Checksum;
} PACKET_s_t; //4byte simple packet for DPC_A <=> DPC_B

typedef struct {
	PACKET_s_t tx;
	PACKET_s_t rx;
} PACKET_comm_t; //4byte simple packet for DPC_A <=> DPC_B


/* Exported constants --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
RD_RET RD_PACKET_INIT(PACKET_comm_t *packet_obj);
RD_RET RD_PACKET_READ(UART_Ring_t *uart_obj, PACKET_comm_t *packet_obj);
RD_RET RD_PACKET_WRITE(UART_Ring_t *uart_obj, PACKET_comm_t *packet_obj);


#endif /* INC_RD_COMM_DPCB_H_ */
