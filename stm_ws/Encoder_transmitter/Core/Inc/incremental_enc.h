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

#define ADC_WIN_SAMPLES      106
#define ADC_DMA_HALF_LEN     (NUM_LOADCELLS * ADC_WIN_SAMPLES)   /* half 하나의 uint16_t 개수 */
#define ADC_DMA_LEN          (ADC_DMA_HALF_LEN * 2)              /* Start_DMA 에 넘길 총 전송 개수 */

/* CHECKER 임계값 */
#define ADC_STALE_TIMEOUT_MS 20        /* 콜백 정체 = DMA/변환 정지 판정 (창 주기 ~5.5ms 대비 여유) */
#define ADC_RAIL_LOW         8         /* 이 이하로 붙으면 단선/접지문제 의심 (0~4095 중) */
#define ADC_RAIL_HIGH        4088      /* 이 이상으로 붙으면 포화/전원문제 의심 */


/* Exported types ------------------------------------------------------------*/

/* 채널별 보정값 (각 로드셀마다 다름). offset 은 raw ADC 카운트 단위의 영점(tare). */
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
