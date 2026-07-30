/*
 * rd_comm_dyn.h
 *  Dynamixel Protocol 2.0 – 저수준 패킷 빌더 / 파서
 *
 *  Created on: Mar 10, 2026
 *      Author: swarm
 */

#ifndef INC_RD_COMM_DYN_H_
#define INC_RD_COMM_DYN_H_

/* Private includes ----------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "rd_common.h"
#include "rd_define.h"
#include "rd_uart.h"

/* ── Protocol 2.0 고정 바이트 (raw magic, enum 부적합) ──────────────────────*/
#define DYN_HEADER1         0xFF
#define DYN_HEADER2         0xFF
#define DYN_HEADER3         0xFD
#define DYN_RESERVED        0x00

#define DYN_BROADCAST_ID    0xFE

/* ── 버퍼/블록 크기 상수 ─────────────────────────────────────────────────────*/
#define DYN_ID_IDX           4
#define DYN_HEADER_SIZE      7 // HEADER(3) + RESERVED(1) + ID(1) + LENGTH(2)
#define DYN_DATA_BUF_SIZE    64

typedef struct __attribute__((packed)){
	uint8_t  TargetID;  
	uint16_t Length;  
	uint8_t  Instruction;   
	uint8_t  Data[DYN_DATA_BUF_SIZE];
    /*-----------------------------*/
    uint16_t  data_len; // DATA Length w/ CRC(2)
} DYN_PACKET_t; //16byte MODBUS packet

typedef struct {
	DYN_PACKET_t tx;
	DYN_PACKET_t rx;
} DYN_comm_t; //16byte MODBUS packet

/* Exported functions prototypes ---------------------------------------------*/
RD_RET RD_DYNPACK_READ(RS485_t *rs485_obj, DYN_comm_t *packet_obj);
RD_RET RD_DYNPACK_WRITE(RS485_t *rs485_obj, DYN_comm_t *packet_obj);

#endif /* INC_RD_COMM_DYN_H_ */
