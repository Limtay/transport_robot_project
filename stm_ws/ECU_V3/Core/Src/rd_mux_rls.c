/*
 * rd_mux_rls.c
 *
 *  74HC138 + MAX3491 ×6 RLS 엔코더 MUX — 채널 매핑/사용 순서는 rd_mux_rls.h 참고.
 *  드라이버는 진단(RET_*, ok/miss 카운트)만, 복구·상태전이는 상위 태스크 소유.
 *
 *  Created on: Sep 28, 2026
 *      Author: limtay
 */

/* Includes ------------------------------------------------------------------*/
#include "rd_mux_rls.h"

/* Exported user code ----------------------------------------------------------*/

RD_RET RD_MUX_RLS_INIT(MUX_RLS_t *mux_obj, UART_Ring_t *uart_obj, RLS_Model_e model)
{
    if (mux_obj == NULL || uart_obj == NULL) return RET_NOK;

    mux_obj->uart_obj = uart_obj;
    for (uint8_t i = 0; i < RLS_NUM; i++) {
        RD_RLS_INIT(&mux_obj->enc[i], model);
        mux_obj->ok_cnt[i]   = 0;
        mux_obj->miss_cnt[i] = 0;
    }
    return RD_MUX_RLS_SELECT(mux_obj, RLS_MUX_NONE);
}

RD_RET RD_MUX_RLS_SELECT(MUX_RLS_t *mux_obj, uint8_t idx)
{
    if (mux_obj == NULL) return RET_NOK;

    uint8_t y;
    if      (idx == RLS_MUX_NONE) y = RLS_MUX_Y_IDLE;
    else if (idx <  RLS_NUM)      y = (uint8_t)(idx + RLS_MUX_Y_OFFSET);
    else return RET_NOK;

    /* 74HC138 입력: A = bit0, B = bit1, C = bit2 */
    for (uint8_t b = 0; b < 3u; b++) {
        HAL_GPIO_WritePin(mux_obj->SEL[b].per_GPIOx, mux_obj->SEL[b].per_GPIO_Pin,
                          ((y >> b) & 0x01u) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    }
    mux_obj->mux_ch = idx;
    return RET_OK;
}

RD_RET RD_MUX_RLS_WRITE(MUX_RLS_t *mux_obj, uint8_t idx, uint8_t cmd)
{
    if (mux_obj == NULL || mux_obj->uart_obj == NULL || idx >= RLS_NUM) return RET_NOK;

    if (RD_MUX_RLS_SELECT(mux_obj, idx) != RET_OK) return RET_NOK;

    /* 이전 채널이 타임아웃 뒤 늦게 보낸(또는 전환으로 잘린) 프레임이 남아 있으면
     * 이번 채널 응답으로 파싱되므로 송신 전에 버린다. */
    mux_obj->uart_obj->rx_new = 0;

    return RD_RLS_WRITE(mux_obj->uart_obj, &mux_obj->enc[idx], cmd);
}

RD_RET RD_MUX_RLS_READ(MUX_RLS_t *mux_obj, uint8_t idx)
{
    if (mux_obj == NULL || mux_obj->uart_obj == NULL || idx >= RLS_NUM) return RET_NOK;
    if (mux_obj->mux_ch != idx) return RET_NOK;   /* 선택 안 된 채널의 응답일 수 없음 */

    RD_RET ret = RD_RLS_READ(mux_obj->uart_obj, &mux_obj->enc[idx]);
    if (ret == RET_OK) mux_obj->ok_cnt[idx]++;
    else               mux_obj->miss_cnt[idx]++;
    return ret;
}
