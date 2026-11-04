/*
 * incremental_enc.h
 *
 *  Created on: 2026. 11. 4.
 *      Author: SHJ
 */

#ifndef INC_INCREMENTAL_ENC_H_
#define INC_INCREMENTAL_ENC_H_

/* Private includes ----------------------------------------------------------*/
#include "stm32f1xx_hal.h"

/* Exported macro ------------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

typedef struct {
    uint16_t current_cnt;
    uint16_t last_cnt;
    int16_t  delta_cnt;
    int32_t  accumulated_count;
} LinearEncoder_t;

/* Exported variables --------------------------------------------------------*/


/* Exported functions prototypes ---------------------------------------------*/
void ENC_INIT(void);
void ENC_UPDATE(LinearEncoder_t *pEnc, TIM_HandleTypeDef *htim);
void ENC_SET_ZERO(LinearEncoder_t *pEnc, TIM_HandleTypeDef *htim);
void Delay_Cycles(uint32_t cycles);
#endif /* INC_INCREMENTAL_ENC_H_ */
