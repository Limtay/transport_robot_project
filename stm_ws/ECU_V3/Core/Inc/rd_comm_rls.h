/*
 * rd_comm_rls.h
 *
 *  RLS 엔코더(Orbis / AksIM-2) 비동기 시리얼 커맨드-응답 레이어.
 *  두 기종은 명령 뼈대가 같고(ASCII 1바이트 요청 → 헤더+위치+부가 응답)
 *  패킷 길이·분해능·부가 데이터 포맷만 다르다 → 모델별 static 파서로 분리.
 *
 *  ── 모델 대조 ────────────────────────────────────────────────────────────
 *              Orbis (BR10/20/30)        AksIM-2 (MB029)
 *   분해능      14bit 고정                17 / 18bit (INIT 후 bits 로 지정)
 *   위치 필드   2B(14B) / 4B(14M)         3B(싱글턴) / 5B(멀티턴)
 *   'd' 상태    1B                        2B
 *   't' 온도    2B, °C×10 (signed)        1B, °C 정수 (signed)
 *   'v'         7B (시리얼 6자)           59B (ID 7자 + 시리얼 8자 + 파트넘버…)
 *   전용 명령   —                         'j'(영속 상태) 's'(속도 RPM)
 *                                         'a'(신호 레벨) 'i'(자가캘 상태, 위치 없음)
 *
 *  위치는 두 기종 모두 **좌측 정렬 + MSB first**, 최하위 2비트가
 *  b1=Error / b0=Warning (active low, 0 이면 이상). 그 사이는 0 패딩.
 *  → 필드폭(Orbis 16bit / AksIM 24bit)과 분해능만 알면 동일 수식으로 파싱된다.
 *
 *  출처: DOC/SENSOR/orbis_uart_rs422_stm32_guide.md,
 *        DOC/SENSOR/Aksim2 uart rs422 stm32 guide.md
 *
 *  Created on: Sep 2, 2026
 *      Author: limtay
 */

#ifndef INC_RD_COMM_RLS_H_
#define INC_RD_COMM_RLS_H_

/* Private includes ------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "rd_common.h"
#include "rd_define.h"
#include "rd_uart.h"
#include <math.h>

/* Exported types -----------------------------------------------------------------*/

/** @brief 대상 하드웨어. INIT 에서 1회 지정하며 파서 분기 키가 된다. */
typedef enum {
    RLS_ORBIS = 0,   /**< Orbis BR10/20/30 — 14bit 고정 */
    RLS_AKSIM,       /**< AksIM-2 MB029    — 17/18bit   */
} RLS_Model_e;

/**
 * @brief  Orbis 명령. 값 = 실제 전송되는 ASCII 코드.
 *         Orbis UART 는 이 5개가 전부다 ('j','s','a','i' 없음).
 */
typedef enum {
    ORBIS_POS_HDR = 0x31,   /* '1' 헤더 + 위치                  */
    ORBIS_POS     = 0x33,   /* '3' 위치만 (헤더 없음, 폴링용)   */
    ORBIS_STATUS  = 0x64,   /* 'd' 헤더 + 위치 + 상세상태 1B    */
    ORBIS_TEMP    = 0x74,   /* 't' 헤더 + 위치 + 온도 2B(×10)   */
    ORBIS_SERIAL  = 0x76,   /* 'v' 헤더 + 시리얼 6B             */
} RLS_CMD_ORBIS_e;

/**
 * @brief  AksIM-2 명령. 값 = ASCII 코드. 데이터시트(MBD01_15 p.36)의 9개 전부.
 *         잘못된 명령에는 헤더 1B 만 회신 → 길이 불일치로 framing 에러 처리된다.
 */
typedef enum {
    AKSIM_POS_HDR    = 0x31,  /* '1' 헤더 + 위치                     */
    AKSIM_POS        = 0x33,  /* '3' 위치만 (헤더 없음, 폴링용)      */
    AKSIM_SIGNAL     = 0x61,  /* 'a' 헤더 + 위치 + 신호 레벨 2B      */
    AKSIM_STATUS     = 0x64,  /* 'd' 헤더 + 위치 + 상세상태 2B       */
    AKSIM_SELFCAL    = 0x69,  /* 'i' 헤더 + 자가캘 상태 7B (위치 없음) */
    AKSIM_STATUS_PER = 0x6A,  /* 'j' 헤더 + 위치 + 영속 상세상태 2B  */
    AKSIM_SPEED      = 0x73,  /* 's' 헤더 + 위치 + 속도 3B (RPM)     */
    AKSIM_TEMP       = 0x74,  /* 't' 헤더 + 위치 + 온도 1B           */
    AKSIM_SERIAL     = 0x76,  /* 'v' 59B (ID + 시리얼 + 파트넘버…)   */
} RLS_CMD_AKSIM_e;

/** @brief 파싱 결과. 명령을 바꿔가며 워치창에서 필드별로 확인하는 용도. */
typedef struct {
    RLS_Model_e model;        /**< INIT 에서 지정                                */
    uint8_t  bits;            /**< 위치 분해능. INIT 이 모델별 기본값(ORBIS 14 /
                                   AKSIM 18) 세팅 — 17bit AksIM 이면 INIT 후 17 로 덮어쓸 것 */

    uint32_t position;        /**< 0 .. (1<<bits)-1                              */
    uint16_t multiturn;       /**< 멀티턴 응답에서만 갱신 (싱글턴 응답은 값 유지) */
    uint8_t  error;           /**< 1 = 위치 무효 (원본 active-low 를 반전 저장)   */
    uint8_t  warning;         /**< 1 = 동작 한계 근접                            */

    uint16_t status;          /**< 'd' 상세상태 raw (Orbis 1B / AksIM 2B)        */
    uint16_t status_per;      /**< 'j' 영속 상세상태 raw — AksIM 전용            */
    int32_t  speed_rpm;       /**< 's' 회전 속도 [RPM] — AksIM 전용              */
    float    temp_c;          /**< 't' 온도 [°C]                                 */
    char     serial[17];      /**< 'v' 시리얼 (Orbis 6자 / AksIM 8자) + NUL      */
    char     part_number[17]; /**< 'v' 파트넘버 16자 + NUL — AksIM 전용.
                                   분해능(17B/18B/17M/18M)·보드레이트 변형 확인용   */

    /* ── AksIM 전용 부가 데이터 ('d','j' 상세상태 비트는 active-HIGH — 위치 E/W 와 극성 반대) */
    uint16_t signal_level;    /**< 'a' 신호 레벨 raw → RD_RLS_AksimRideHeightUm() */
    uint8_t  cal_status;      /**< 'i' 자가캘 상태 (코드는 APP10)                 */
    uint16_t ecc_um;          /**< 'i' 링 편심 [um] — 200 초과 시 재조립 권고     */
    uint16_t ecc_phase_deg;   /**< 'i' 링 편심 위상 [deg]                         */
    int16_t  radial_um;       /**< 'i' 리드헤드 반경방향 변위 [um] (signed)       */

    uint8_t  last_cmd;        /**< WRITE 에서 저장한 ASCII 명령 — READ 파싱 분기 키 */
    volatile uint32_t ts_stamp; /**< 파싱 성공 시각 [rd_now_tick, x0.1ms]        */

    /* WRITE 직전 → READ 직후 TIM5 틱 차 [x0.1ms]. TIM5 가 10kHz(1틱=100us)라
     * 실측 왕복 ~85us 구간에서는 0/1 만 나온다 — 정밀 측정용이 아니라
     * "왕복이 1틱 안에 끝나는가" 를 보는 상시 건전성 지표. */
    uint32_t tim5_delta;
} RLS_comm_t;

/* Exported functions prototypes ---------------------------------------------------*/

/**
 * @brief  0 초기화 + 모델/기본 분해능 설정. 부팅 시 1회.
 *         AksIM 17bit 장비면 호출 직후 obj->bits = 17 로 덮어쓸 것.
 */
RD_RET RD_RLS_INIT(RLS_comm_t *rls_obj, RLS_Model_e model);

/**
 * @brief  cmd 1바이트를 DMA TX 로 전송하고 last_cmd 에 기록(READ 파싱 분기용).
 *         cmd 는 모델에 맞는 열거형(RLS_CMD_ORBIS_e / RLS_CMD_AKSIM_e) 값을 넣는다.
 * @retval RET_OK/RET_WAIT/RET_NOK — RD_UART_TRANSMIT 그대로 전달
 */
RD_RET RD_RLS_WRITE(UART_Ring_t *uart_obj, RLS_comm_t *rls_obj, uint8_t cmd);

/**
 * @brief  rx_new 확인 후 model + last_cmd 조합에 맞게 응답을 파싱.
 *         멀티턴/싱글턴은 응답 총 길이로 프레임마다 자동 판별한다.
 * @retval RET_OK   파싱 성공
 * @retval RET_WAIT 신규 데이터 없음, 또는 길이/헤더 불일치
 *                  (불일치 시 COMM_ERR_FRAMING_BIT 통보)
 * @retval RET_NOK  NULL 포인터
 */
RD_RET RD_RLS_READ(UART_Ring_t *uart_obj, RLS_comm_t *rls_obj);

/** @brief 현재 position 을 각도로 변환 [deg]. 분해능(bits)에 맞춰 자동 스케일. */
static inline float RD_RLS_AngleDeg(const RLS_comm_t *rls_obj)
{
    return (float)rls_obj->position * 360.0f / (float)(1UL << rls_obj->bits);
}

/**
 * @brief  'a' 신호 레벨 → 라이드 하이트 [um] (MBD01 p.23, 사이즈 022/029 계수).
 *         목표 50~350um, 공차 ±20um. signal_level 0(미수신)이면 NAN.
 */
static inline float RD_RLS_AksimRideHeightUm(const RLS_comm_t *rls_obj)
{
    if (rls_obj->signal_level == 0u) return NAN;
    return -95.49f * logf((float)rls_obj->signal_level) + 977.0f;
}

#endif /* INC_RD_COMM_RLS_H_ */
