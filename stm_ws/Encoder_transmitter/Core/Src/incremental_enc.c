/*
 * incremental_enc.c
 *
 *  Created on: 2026. 11. 4.
 *      Author: SHJ
 */

#include "incremental_enc.h"

/* Macro ---------------------------------------------------------------------*/
//typedef uint32_t RD_RET;

//#define RET_OK ((RD_RET)0x00)
//#define RET_NOK ((RD_RET)0x01)
//#define RET_WAIT ((RD_RET)0x02)

/* Exported variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;

/* Private variables ---------------------------------------------------------*/
LinearEncoder_t ENC[2];

/* Private function ----------------------------------------------------------*/
/* base 오프셋부터 시작하는 half 창을 채널별 박스카 평균 → loadcell.raw_avg 갱신 후 cal+IIR 적용. */

void ENC_INIT(){
    /* 1. 엔코더 하드웨어 카운트 시작 */
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    /* 2. 첫 델타 계산 시 튀는 현상을 막기 위해 초기 CNT 값을 last_cnt에 동기화 */
    ENC[0].last_cnt = __HAL_TIM_GET_COUNTER(&htim3);
    ENC[1].last_cnt = __HAL_TIM_GET_COUNTER(&htim4);

    /* 3. 누적치 및 거리 변수 초기화 */
    ENC[0].accumulated_count = 0;
    ENC[1].accumulated_count = 0;

    /* 4. 타이머 마스크 활성화 */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // 추적 인터페이스 활성화
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;           // 사이클 카운터 활성화
}

void ENC_UPDATE(LinearEncoder_t *pEnc, TIM_HandleTypeDef *htim)
{
    pEnc->current_cnt = __HAL_TIM_GET_COUNTER(htim);
    pEnc->delta_cnt = (int16_t)(pEnc->current_cnt - pEnc->last_cnt);
    pEnc->accumulated_count += pEnc->delta_cnt;
    pEnc->last_cnt = pEnc->current_cnt;
}

void ENC_SET_ZERO(LinearEncoder_t *pEnc, TIM_HandleTypeDef *htim)
{
	__HAL_TIM_SET_COUNTER(htim, 0);
	pEnc->accumulated_count = 0;
	pEnc->last_cnt 			= 0;
	pEnc->current_cnt 		= 0;
	pEnc->delta_cnt 		= 0;

}

void Delay_Cycles(uint32_t cycles)
{
    uint32_t start = DWT->CYCCNT;
    while ((DWT->CYCCNT - start) < cycles);
}
