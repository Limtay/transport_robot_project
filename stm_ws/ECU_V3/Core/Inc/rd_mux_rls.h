/*
 * rd_mux_rls.h
 *
 *  RLS 엔코더 최대 6채널을 UART4 1개로 시분할 폴링하는 MUX 레이어.
 *  74HC138(3→8 디코더) 출력 Y0~Y7(active-low)을 NOT 게이트로 반전해
 *  선택 채널만 H, 나머지는 L 로 MAX3491 ×6 의 enable 을 하나만 열어 준다.
 *  패킷 빌드/파싱은 rd_comm_rls 그대로 — 여기서는 "어느 트랜시버를 열지" 만 담당.
 *
 *  ── 채널 매핑 ────────────────────────────────────────────────────────────
 *   엔코더 idx 0..5  →  74HC138 Y(idx + RLS_MUX_Y_OFFSET)  = Y0..Y5
 *   RLS_MUX_NONE     →  Y(RLS_MUX_Y_IDLE) = Y7 (미배선 → 전 트랜시버 해제)
 *   MUXA/B/C = y 의 bit0/1/2.
 *   주의: 리셋 직후 MX_GPIO_Init 이 셋 다 LOW → Y0(엔코더 0)이 선택된 상태로 부팅.
 *         RD_MUX_RLS_INIT 에서 Y7 로 해제한다.
 *
 *  ── 사용 순서 (요청 1개 → 응답 1개, rd_comm_rls 와 동일한 폴링 전제) ─────
 *   RD_MUX_RLS_WRITE(mux, i, cmd)   : 채널 i 선택 + stale 수신 폐기 + cmd 송신
 *   (UART IDLE 플래그 대기)
 *   RD_MUX_RLS_READ(mux, i)         : 채널 i 응답 파싱 → mux->enc[i]
 *   → 응답을 다 받기 전에 채널을 바꾸면 안 된다 (응답이 잘려 다음 채널 소관으로 섞임).
 *
 *  Created on: Sep 28, 2026
 *      Author: limtay
 */

#ifndef INC_RD_MUX_RLS_H_
#define INC_RD_MUX_RLS_H_

/* Private includes ----------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "rd_common.h"
#include "rd_define.h"
#include "rd_uart.h"
#include "rd_comm_rls.h"
#include "rd_peripheral.h"

/* Exported constants --------------------------------------------------------*/
#define RLS_NUM            6u     /**< 실장 엔코더 수 (MAX3491 개수)                      */
#define RLS_MUX_Y_OFFSET   0u     /**< 엔코더 idx 0 이 물린 74HC138 출력 번호 (배선 기준) */
#define RLS_MUX_Y_IDLE     7u     /**< 미배선 출력 — 선택 시 전 트랜시버 해제             */
#define RLS_MUX_NONE       0xFFu  /**< RD_MUX_RLS_SELECT 인자: 채널 해제                   */

/* Exported types ------------------------------------------------------------*/
typedef struct {
    UART_Ring_t      *uart_obj;          /**< 공유 UART (의존성 주입)                          */
    GPIO_IO_t         SEL[3];            /**< MUXA/B/C — 객체 생성 시 .SEL = {...} 로 주입      */
    volatile uint8_t  mux_ch;            /**< 현재 선택된 엔코더 idx (RLS_MUX_NONE = 해제)      */

    RLS_comm_t        enc[RLS_NUM];      /**< 채널별 파싱 결과                                  */
    uint32_t          ok_cnt[RLS_NUM];   /**< 파싱 성공 누적 — 워치창 채널 건전성 확인용        */
    uint32_t          miss_cnt[RLS_NUM]; /**< 무응답/길이·헤더 불일치 누적                      */
} MUX_RLS_t;

/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  uart_obj 주입 + 채널별 RD_RLS_INIT(model) + 채널 해제. 부팅 시 1회.
 *         SEL 은 건드리지 않는다 (정적 초기화로 주입). 채널별로 기종이 다르면
 *         호출 후 enc[i] 를 RD_RLS_INIT 으로 다시 초기화할 것.
 */
RD_RET RD_MUX_RLS_INIT(MUX_RLS_t *mux_obj, UART_Ring_t *uart_obj, RLS_Model_e model);

/**
 * @brief  엔코더 idx 채널의 트랜시버만 연다. RLS_MUX_NONE 이면 전부 해제.
 *         3핀을 순차로 쓰므로 수십 ns 동안 중간 코드가 거쳐가지만, 트랜잭션 사이
 *         (버스 idle)에서만 호출하므로 무해하다.
 * @retval RET_NOK  NULL 또는 idx 범위 밖
 */
RD_RET RD_MUX_RLS_SELECT(MUX_RLS_t *mux_obj, uint8_t idx);

/**
 * @brief  채널 선택 → 이전 채널의 늦은/잘린 수신 폐기 → cmd 1바이트 송신.
 * @retval RD_RLS_WRITE 반환값 그대로 (RET_NOK: 인자 오류)
 */
RD_RET RD_MUX_RLS_WRITE(MUX_RLS_t *mux_obj, uint8_t idx, uint8_t cmd);

/**
 * @brief  현재 선택 채널(idx)의 응답을 enc[idx] 로 파싱하고 ok/miss 카운트.
 *         무응답도 miss 로 센다 → 플래그 대기(타임아웃) 직후 1회만 호출할 것.
 * @retval RD_RLS_READ 반환값 그대로 / RET_NOK: 선택 채널 불일치
 */
RD_RET RD_MUX_RLS_READ(MUX_RLS_t *mux_obj, uint8_t idx);

#endif /* INC_RD_MUX_RLS_H_ */
