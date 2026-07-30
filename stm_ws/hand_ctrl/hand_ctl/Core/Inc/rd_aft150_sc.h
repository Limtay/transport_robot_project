/*
 * rd_aft150_sc.h
 *
 *  Created on: 2026. 4. 28.
 *      Author: SHJ
 */

#ifndef INC_RD_AFT150_SC_H_
#define INC_RD_AFT150_SC_H_

/* Private includes ----------------------------------------------------------*/

#include "stm32f4xx_hal.h"
#include "rd_common.h"
#include "rd_define.h"
#include "main.h"

/* Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/
typedef struct __attribute__((packed)){
	int16_t FT_Low[6];
	int16_t FT_Offset[6];
	float FT_Data[6];
} AFT150_DATA_t;

/* Exported constants --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
void Aidin_Sensor_Start(void);
void Aidin_Sensor_Cali(void);


#endif /* INC_RD_AFT150_SC_H_ */
