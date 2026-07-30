/*
 * rd_lasf_sc.h
 *
 *  Created on: Feb 24, 2026
 *      Author: abc01
 */

#ifndef INC_RD_LASF_SC_H_
#define INC_RD_LASF_SC_H_

/* Private includes ----------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "rd_common.h"
#include "rd_define.h"
#include "rd_uart.h"

/* Exported macro ------------------------------------------------------------*/

#define PACKET_HEADER1  0x55
#define PACKET_HEADER2  0xAA

#define DATA_LENGTH_TX	0x0D
#define PACKET_TX_LEN	18
#define TX_SUM_IDX1		2
#define TX_SUM_IDX2		17

#define DATA_LENGTH_RX	0x0F
#define PACKET_RX_LEN	20
#define RX_SUM_IDX1		2
#define RX_SUM_IDX2		19

#define CMD_WR			0x32
#define IDX_MODE		0x0025
#define IDX_POS			0x0029
#define IDX_FDB			0x0025

#define LASF_NUM		9

//#define RX_TIMEOUT 1000 //ms

/* Exported types ------------------------------------------------------------*/

typedef struct __attribute__((packed)){
	uint8_t 	HeaderH;		// 0x55
	uint8_t 	HeaderL;		// 0xAA
	uint8_t 	DAL;			// 0x05, (include CMD~POS_DATA_HSB) change. fixed 0D
	uint8_t 	ID;				// Target Device ID
	uint8_t 	CMD;			// 0x32 : CMD WR RESISTER
	uint16_t 	REG_IDX;		// 0x0025 : start register of mode setting
	uint16_t 	SET_MODE;		// 2byte MODE, 1:servo, 2:speed, 3:force
	int16_t 	CTL_VOLTAGE;	// 2byte voltage data. always 0 becuase ill not use this
	int16_t 	CTL_FORCE;		// 2byte force data 0~2000
	uint16_t 	CTL_SPEED;		// 2byte speed data 0~2000
	int16_t 	CTL_POS;		// 2byte position data 0~2000
	uint8_t 	Checksum;		// checksum without Header
} LASF_TX_PACKET_t;

typedef struct __attribute__((packed)){
	uint8_t 	HeaderL;	// 0xAA
	uint8_t 	HeaderH;	// 0x55
	uint8_t 	DAL;		// 0x0F, 15byte
	uint8_t 	ID;			// Target Device ID
	uint8_t 	CMD;		// 0x32 : CMD WR RESISTER
	uint16_t 	REG_IDX;	// 0x0025 : State register
	int16_t 	TAR_POS;	// 2byte target position data 	(step)
	int16_t 	ACT_POS;	// 2byte actual position data	(step)
	uint16_t 	CURRENT;	// 2byte actual current 		(mA)
	int16_t 	FT_VAL;		// 2byte Force sensor value 	(gram)
	uint16_t 	FT_ADC;		// 2byte ADC value 				(gram)
	int8_t 		TEMP;		// 1byte temperature			(deg)
	uint8_t 	Error_C;	// 1byte Error code for debug
	uint8_t 	Checksum;	// checksum without Header
} LASF_RX_PACKET_t; 		//4byte simple packet for DPC_A <=> DPC_B

typedef struct {
	uint8_t 	ID;			// Device ID

	uint16_t	CTL_MODE;	// 2byte MODE, 1:servo, 2:speed, 3:force
	int16_t 	CTL_POS;	// 2byte position data 0~2000 	(step)
	int16_t 	CTL_FORCE;	// 2byte position data 0~2000 	(step)
	uint16_t 	CTL_SPEED;	// 2byte position data 0~2000 	(step)

	int16_t 	ACT_POS;	// 2byte actual position data	(step)
	int16_t 	FT_VAL;		// 2byte Force sensor value 	(gram)
	int8_t 		TEMP;		// 1byte temperature			(deg)
	uint16_t 	CURRENT;	// 2byte actual current 		(mA)
	uint8_t 	Error_C;	// 1byte Error code for debug
	int16_t 	TAR_POS;	// 2byte actual position data	(step)
	uint16_t	FT_ADC;		// 2byte force ADC data			(step)
	int16_t		FT_OFFSET;	// 2byte force offset			(gram)

} LASF_DATA_t;

typedef struct {
	LASF_RX_PACKET_t 	RX;
	LASF_TX_PACKET_t 	TX;
	LASF_DATA_t			LASF;
} LASF_COMM_t;	//define single LASF actuator

/* Exported constants --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
RD_RET RD_LASF_INIT(LASF_COMM_t *lasf_obj);
RD_RET RD_LASF_READ(UART_Ring_t *uart_obj, LASF_COMM_t *packet_obj);
RD_RET RD_LASF_WRITE(UART_Ring_t *uart_obj, LASF_COMM_t *packet_obj);
RD_RET RD_LASF_UPDATE(LASF_COMM_t *packet_obj);


#endif /* INC_RD_LASF_SC_H_ */
