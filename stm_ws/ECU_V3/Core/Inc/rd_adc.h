/*
 * rd_adc.h
 *
 *  Created on: Jul 3, 2026
 *      Author: limtay
 *
 *  로드셀 ADC (ADC1 + DMA2_Stream0 circular).
 *  kHz 오버샘플링 → 창별 박스카 평균 → 캘리브레이션 → (약한 IIR) → 200Hz 제어 피드백.
 *  안전 최소셋(옵션1): CHECKER = staleness + 레일값 진단 / RECOVERY = OVR·DMA정지 시 Stop→Start 재기동.
 */

#ifndef INC_RD_ADC_H_
#define INC_RD_ADC_H_

/* Private includes ----------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "rd_common.h"
#include "rd_define.h"

/* Exported macro ------------------------------------------------------------*/
/* 채널 수. CubeMX MX_ADC1_Init 의 NbrOfConversion 및 Rank 순서와 반드시 일치. 현재 배선: IN8(PB0)/IN9(PB1). */
#define NUM_LOADCELLS        2

/* 오버샘플링: 채널당 창(half-buffer) 샘플 수. DMA 는 [half][full] 더블버퍼(circular).
 * 1변환 = (480+12)cyc ≈ 23.4µs (ADC 21MHz = APB2 84MHz/4, SYSCLK 168MHz) 기준,
 * 2ch × 106 = 212변환 ≈ 4.97ms → half 창이 제어주기 5ms 에 정렬.
 * 콜백(half/full 교대)이 ~5ms 마다 adcTask 를 깨움. 채널/샘플/클럭 바꾸면 이 값 재계산.
 * (구 53 은 SYSCLK 84MHz/ADC 10.5MHz 시절 값 — 2026-07 클럭 2배 상향으로 재계산) */
#define ADC_WIN_SAMPLES      106
#define ADC_DMA_HALF_LEN     (NUM_LOADCELLS * ADC_WIN_SAMPLES)   /* half 하나의 uint16_t 개수 */
#define ADC_DMA_LEN          (ADC_DMA_HALF_LEN * 2)              /* Start_DMA 에 넘길 총 전송 개수 */

/* CHECKER 임계값 */
#define ADC_STALE_TIMEOUT_MS 20        /* 콜백 정체 = DMA/변환 정지 판정 (창 주기 ~5.5ms 대비 여유) */
#define ADC_RAIL_LOW         8         /* 이 이하로 붙으면 단선/접지문제 의심 (0~4095 중) */
#define ADC_RAIL_HIGH        4088      /* 이 이상으로 붙으면 포화/전원문제 의심 */

/* 트림 평균: 창 양끝에서 각각 K개(스파이크 후보)를 버리고 평균. 실측 spike_cnt(주로 2, 최대 3) 기반.
 * 반드시 ADC_WIN_SAMPLES > 2*ADC_TRIM_K. 스파이크가 한쪽에 몰리면 그쪽은 최대 K개까지만 제거됨(넘으면 K↑). */
#define ADC_TRIM_K           2

/* 후단 필터 */
#define ADC_IIR_ALPHA        0.5f      /* 약하게(크게) 시작. 잔여 떨림만 다듬음 → 위상지연 최소 */

/* 진단(임시): 노이즈 프로파일러 — 창당 스파이크 개수 판정 */
#define ADC_SPIKE_K          3.0f      /* |v - 트림평균| > K·σ 이면 스파이크로 카운트 */
#define ADC_SPIKE_FLOOR      3.0f      /* σ 매우 작을 때(깨끗) 오검출 방지용 임계 하한(카운트) */

/* 변환 상수 (튜닝/진단용) */
#define ADC_VREF             3.3f
#define ADC_FULL_SCALE       4095.0f

/* Exported types ------------------------------------------------------------*/

/* 채널별 보정값 (각 로드셀마다 다름). offset 은 raw ADC 카운트 단위의 영점(tare). */
typedef struct {
    float    scale;    /* (raw - offset) → 무게(kg) 변환 계수 */
    uint16_t offset;   /* 영점(tare) raw 값 */
} LoadCellCal;

/* 런타임 상태 (단일 소유자: ISR 는 volatile 필드만 기록, adcTask 가 읽기/가공) */
typedef struct {
    volatile uint32_t last_isr_tick;              /* 마지막 ConvCplt/half 콜백 tick (staleness 판정용) */
    volatile uint8_t  half_ready;                 /* ISR set: 앞쪽 half 창 채워짐 */
    volatile uint8_t  full_ready;                 /* ISR set: 뒤쪽 half 창 채워짐 */

    volatile uint32_t ts_stamp;                   /* 마지막 창 완료(half/full 콜백) 시각 [rd_now_tick, ×0.1ms] — delta_tick 용 */

    uint16_t raw_avg[NUM_LOADCELLS];              /* 최근 창 박스카 평균 (raw) */
    float    weight[NUM_LOADCELLS];               /* cal + IIR 적용 후 무게(kg) */

    uint8_t  rail_flag;                           /* 채널별 레일 의심 비트마스크 (bit i = 채널 i) */
    uint8_t  is_stale;                            /* 최근 CHECKER 에서 staleness 감지됨 */
    uint32_t recover_cnt;                         /* RECOVERY 수행 횟수 (진단) */
    STATE_t  state;                               /* 종합 상태 (lifecycle+health) — CHECKER 가 갱신, reg 로 발행 */

    /* --- 진단(임시): 5ms 창당 콜백 빈도 측정. 버퍼 사이징 판단 후 제거 가능 --- */
    volatile uint32_t cb_half_cnt;                /* half 콜백 누적 (ISR 증가) */
    volatile uint32_t cb_full_cnt;                /* full 콜백 누적 (ISR 증가) */
    uint32_t cb_half_per_tick;                    /* 최근 5ms 창의 half 콜백 수 (watch 대상) */
    uint32_t cb_full_per_tick;                    /* 최근 5ms 창의 full 콜백 수 (watch 대상) */
    uint32_t cb_max_per_tick;                     /* 관측된 (half+full) 최댓값 (버퍼 사이징 근거) */

    /* full 콜백 간격 측정 (us). htim5 기반 → full 한 바퀴 실제 소요시간 */
    uint64_t cb_full_last_us;                     /* 직전 full 콜백 시각 (us) */
    uint32_t cb_full_dt_us;                       /* 직전 full 콜백 간격 (us, watch 대상) */
    uint32_t cb_full_dt_min_us;                   /* 관측된 최소 간격 (us) */
    uint32_t cb_full_dt_max_us;                   /* 관측된 최대 간격 (us) */

    /* --- 진단(임시): 창별 노이즈 프로파일 (트림 평균/편차 기준). 필터 방식 결정 근거 --- */
    float    prof_sigma[NUM_LOADCELLS];           /* 트림 표준편차 (노이즈 크기) */
    uint16_t prof_range[NUM_LOADCELLS];           /* max - min (스파이크 진폭) */
    uint8_t  prof_spike_cnt[NUM_LOADCELLS];       /* 창당 |v-μ|>K·σ 샘플 수 (스파이크 개수) */
} ADC_Loadcell_t;

/* Exported variables --------------------------------------------------------*/
extern LoadCellCal     cal[NUM_LOADCELLS];
extern ADC_Loadcell_t  loadcell;

/* Exported functions prototypes ---------------------------------------------*/
RD_RET   RD_ADC_INIT(ADC_HandleTypeDef *hadc);      /* DMA circular 기동 */
RD_RET   RD_ADC_PROCESS(void);                      /* 준비된 창을 박스카 평균 + cal + IIR (adcTask) */
RD_RET   RD_ADC_CHECKER(void);                      /* staleness/레일 진단만. RET_OK/RET_NOK */
RD_RET   RD_ADC_RECOVERY(ADC_HandleTypeDef *hadc);  /* Stop_DMA → Start_DMA 재기동 (OVR/정지 복구) */

float    RD_ADC_GetWeight(uint8_t idx);             /* 가공된 무게(kg) */
uint16_t RD_ADC_GetRaw(uint8_t idx);                /* 최근 창 raw 평균 (노이즈 실측/튜닝용) */
void     RD_ADC_Tare(void);                         /* 현재 raw 평균을 각 채널 영점으로 설정 */

void     RD_ADC_DIAG_TICK(void);                    /* 진단(임시): 5ms 태스크마다 콜백 카운트 스냅샷+리셋 */

#endif /* INC_RD_ADC_H_ */
