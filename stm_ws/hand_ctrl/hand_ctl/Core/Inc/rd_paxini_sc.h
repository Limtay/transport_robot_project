/*
 * rd_paxini_sc.h
 *
 *  Created on: Feb 25, 2026
 *      Author: abc01
 */

#ifndef INC_RD_PAXINI_SC_H_
#define INC_RD_PAXINI_SC_H_

/* Private includes ----------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "rd_common.h"
#include "rd_define.h"
#include "main.h"
#include <string.h>

/* Exported macro ------------------------------------------------------------*/
extern SPI_HandleTypeDef hspi2; // CubeMX에서 설정한 SPI 핸들
#define PAX_SPI &hspi2

/* CS pin define */
#define PAX_CS1_PORT SPI2_CS1_GPIO_Port
#define PAX_CS1_PIN  SPI2_CS1_Pin
#define PAX_CS2_PORT SPI2_CS2_GPIO_Port
#define PAX_CS2_PIN  SPI2_CS2_Pin
#define PAX_CS3_PORT SPI2_CS3_GPIO_Port
#define PAX_CS3_PIN  SPI2_CS3_Pin

/* function defines */
#define HEAD_LEN 5
#define SPI_BUFF_SIZE 400	// max 381 byte when receive all node's force data
#define PAXINI_NUM 127		//M2528 have 127 contact node
#define Force_THRESHOLD 1  // meanable contact threadhold value : 0.1N Z axis

/* Exported types ------------------------------------------------------------*/
typedef struct __attribute__((packed)){
	float 		Fx;	// 0x55
	float 		Fy;
	float		Fz;
	float		Mx;
	float		My;
	float		Mz;
} FORCE_3AX_t;

typedef struct __attribute__((packed)){
	uint8_t state;
	FORCE_3AX_t GET_SUM;
	uint8_t contact_num;
} PAXINI_PACKET_t;

typedef struct __attribute__((packed)){
	uint8_t state;
	FORCE_3AX_t GET_DATA[PAXINI_NUM];
} PAXINI_FULL_PACKET_t;


/* Exported constants --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Paxini_Init(void);
void process_tactile_data(uint8_t* pData, PAXINI_PACKET_t* tactile);
RD_RET Paxini_Read(uint8_t node_idx, uint8_t func_code, uint16_t addr, uint16_t len, uint8_t *p_out_data);
void Paxini_Write(uint8_t finger_idx, uint8_t func_code, uint16_t addr, uint16_t len, uint8_t *pdata);
void Paxini_Test_Example(void);

#endif /* INC_RD_PAXINI_SC_H_ */
