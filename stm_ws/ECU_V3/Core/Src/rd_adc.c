/*
 * rd_adc.c
 *
 *  Created on: Jul 3, 2026
 *      Author: limtay
 *
 *  로드셀 ADC 파이프라인 (안전 최소셋 / 옵션1).
 *  - DMA circular 로 kHz raw 수집 → half/complete 콜백으로 더블버퍼 (tearing 방지).
 *  - adcTask 가 준비된 창을 박스카 평균(오버샘플링) → cal(scale/offset) → 약한 IIR → weight[].
 *  - CHECKER: 콜백 정체(staleness) + 레일값(단선/포화) 진단만.
 *  - RECOVERY: OVR·DMA정지 시 Stop_DMA → Start_DMA 재기동 (HAL 자동복구 없음).
 *
 *  소유권: ISR 은 loadcell 의 volatile 필드(last_isr_tick, half/full_ready)만 기록.
 *          가공/진단/복구는 adcTask 단독. 뮤텍스 없음(단일 생산자-소비자).
 */

#include "rd_adc.h"
#include "cmsis_os.h"
#include "rd_uart.h"                 /* RTOS_IS_AVAILABLE */
#include <math.h>                    /* sqrtf — 진단 프로파일러 */

extern osThreadId_t adcTaskHandle;   /* main.c 정의 — 콜백이 adcTask 를 깨움 */
extern uint64_t Get_Time_us(void);   /* rd_system.c — 진단용 us 시각 (태스크 문맥에서 호출) */

/* Exported variables --------------------------------------------------------*/
/* 채널별 보정값 — 헤더가 아닌 여기서 1회 정의 (다중 include 시 ODR 위반 방지). */
LoadCellCal cal[NUM_LOADCELLS] = {
    {0.0123f, 0},   /* 로드셀0 (IN8, PB0) */
    {0.0121f, 0},   /* 로드셀1 (IN9, PB1) */
};

ADC_Loadcell_t loadcell = {0};

/* Private variables ---------------------------------------------------------*/
/* DMA 대상 버퍼: [ half(ADC_DMA_HALF_LEN) | full(ADC_DMA_HALF_LEN) ], 각 창은 scan 이 채널 순서로 반복. */
static uint16_t adc_raw[ADC_DMA_LEN];

/* Private function ----------------------------------------------------------*/
/* base 오프셋부터 시작하는 half 창을 채널별 박스카 평균 → loadcell.raw_avg 갱신 후 cal+IIR 적용. */
static void RD_ADC_ACCUMULATE(uint16_t base)
{
    uint32_t sum[NUM_LOADCELLS] = {0};
    uint16_t low[NUM_LOADCELLS][ADC_TRIM_K];    /* 채널별 최소 K개 (스파이크 후보) */
    uint16_t high[NUM_LOADCELLS][ADC_TRIM_K];   /* 채널별 최대 K개 */
    for (uint8_t ch = 0; ch < NUM_LOADCELLS; ch++)
        for (uint8_t k = 0; k < ADC_TRIM_K; k++) { low[ch][k] = 0xFFFF; high[ch][k] = 0; }

    for (uint16_t s = 0; s < ADC_WIN_SAMPLES; s++) {
        uint16_t off = base + s * NUM_LOADCELLS;
        for (uint8_t ch = 0; ch < NUM_LOADCELLS; ch++) {
            uint16_t v = adc_raw[off + ch];
            sum[ch] += v;

            /* 최소 K개 유지: 현재 low[] 중 가장 큰 것보다 작으면 그 자리를 교체 */
            uint8_t im = 0;
            for (uint8_t k = 1; k < ADC_TRIM_K; k++) if (low[ch][k] > low[ch][im]) im = k;
            if (v < low[ch][im]) low[ch][im] = v;

            /* 최대 K개 유지: 현재 high[] 중 가장 작은 것보다 크면 그 자리를 교체 */
            uint8_t iM = 0;
            for (uint8_t k = 1; k < ADC_TRIM_K; k++) if (high[ch][k] < high[ch][iM]) iM = k;
            if (v > high[ch][iM]) high[ch][iM] = v;
        }
    }

    for (uint8_t ch = 0; ch < NUM_LOADCELLS; ch++) {
        /* 트림 평균: 양끝 K개(임펄스 노이즈)를 버리고 나머지 평균 → IIR 앞단에서 스파이크 제거. */
        uint32_t trim = 0;
        for (uint8_t k = 0; k < ADC_TRIM_K; k++) trim += low[ch][k] + high[ch][k];
        uint16_t avg = (uint16_t)((sum[ch] - trim) / (ADC_WIN_SAMPLES - 2 * ADC_TRIM_K));
        loadcell.raw_avg[ch] = avg;

        /* cal: (raw - 영점) * scale → 무게. IIR: 잔여 떨림만 1단 평활. */
        float w = ((float)avg - (float)cal[ch].offset) * cal[ch].scale;
        loadcell.weight[ch] += ADC_IIR_ALPHA * (w - loadcell.weight[ch]);
    }
}

/* 진단(임시): 창별 노이즈 프로파일. 트림(min·max 1개 제거) 평균/편차를 기준으로 잡아 σ 가
 * 스파이크에 오염되지 않게 한 뒤, |v-μ|>K·σ 샘플 수를 세어 "창당 스파이크 개수"를 구한다.
 * → 트림 K / 미디언 전환 결정 근거. 제거 시: 이 함수 + PROCESS 호출 2줄 + prof_* 필드/매크로 삭제. */
static void RD_ADC_PROFILE(uint16_t base)
{
    for (uint8_t ch = 0; ch < NUM_LOADCELLS; ch++) {
        uint32_t sum = 0;
        uint64_t sum_sq = 0;
        uint16_t vmin = 0xFFFF, vmax = 0;

        for (uint16_t s = 0; s < ADC_WIN_SAMPLES; s++) {
            uint16_t v = adc_raw[base + s * NUM_LOADCELLS + ch];
            sum    += v;
            sum_sq += (uint32_t)v * v;
            if (v < vmin) vmin = v;
            if (v > vmax) vmax = v;
        }

        uint32_t n = ADC_WIN_SAMPLES - 2;                 /* 트림 후 표본 수 */
        float mean = (float)(sum - vmin - vmax) / (float)n;
        float var  = (float)(sum_sq - (uint32_t)vmin*vmin - (uint32_t)vmax*vmax) / (float)n - mean*mean;
        if (var < 0.0f) var = 0.0f;                       /* 반올림 오차 방어 */
        float sigma = sqrtf(var);

        float thr = ADC_SPIKE_K * sigma;
        if (thr < ADC_SPIKE_FLOOR) thr = ADC_SPIKE_FLOOR; /* 깨끗할 때 노이즈플로어 오검출 방지 */

        uint8_t cnt = 0;
        for (uint16_t s = 0; s < ADC_WIN_SAMPLES; s++) {
            float d = (float)adc_raw[base + s * NUM_LOADCELLS + ch] - mean;
            if (d < 0.0f) d = -d;
            if (d > thr) cnt++;
        }

        loadcell.prof_sigma[ch]     = sigma;
        loadcell.prof_range[ch]     = vmax - vmin;
        loadcell.prof_spike_cnt[ch] = cnt;
    }
}

/* Exported functions --------------------------------------------------------*/
RD_RET RD_ADC_INIT(ADC_HandleTypeDef *hadc)
{
    loadcell.half_ready = 0;
    loadcell.full_ready = 0;
    loadcell.last_isr_tick = osKernelGetTickCount();
    loadcell.ts_stamp      = TS_INVALID;   /* 첫 창 완료 전 delta_tick = 0xFF 보장 */

    if (HAL_ADC_Start_DMA(hadc, (uint32_t*)adc_raw, ADC_DMA_LEN) != HAL_OK) {
        return RET_NOK;
    }
    return RET_OK;
}

/* 준비된 창을 가공. 두 half 가 모두 준비됐으면 순서대로 처리해 weight 를 최신(full)으로 마감. */
RD_RET RD_ADC_PROCESS(void)
{
    uint8_t processed = 0;

    if (loadcell.half_ready) {
        loadcell.half_ready = 0;
        RD_ADC_ACCUMULATE(0);
        RD_ADC_PROFILE(0);                 /* 진단(임시) */
        processed = 1;
    }
    if (loadcell.full_ready) {
        loadcell.full_ready = 0;
        RD_ADC_ACCUMULATE(ADC_DMA_HALF_LEN);
        RD_ADC_PROFILE(ADC_DMA_HALF_LEN);  /* 진단(임시) */
        processed = 1;

        /* 진단(임시): full 창 처리 간격(us). 태스크 wake 기준이라 스케줄링 지터 포함. */
        uint64_t now = Get_Time_us();
        if (loadcell.cb_full_last_us != 0) {
            uint32_t dt = (uint32_t)(now - loadcell.cb_full_last_us);
            loadcell.cb_full_dt_us = dt;
            if (loadcell.cb_full_dt_min_us == 0 || dt < loadcell.cb_full_dt_min_us)
                loadcell.cb_full_dt_min_us = dt;
            if (dt > loadcell.cb_full_dt_max_us)
                loadcell.cb_full_dt_max_us = dt;
        }
        loadcell.cb_full_last_us = now;
    }
    return processed ? RET_OK : RET_WAIT;
}

/* 진단만 수행(직접 복구 없음). staleness = 복구 대상(RET_NOK), 레일 = HW 문제라 보고만. */
RD_RET RD_ADC_CHECKER(void)
{
    /* 1) 레일값: 채널별 단선(LOW)/포화(HIGH) 고착 표시. SW 로는 못 고치므로 플래그만. */
    uint8_t rail = 0;
    for (uint8_t ch = 0; ch < NUM_LOADCELLS; ch++) {
        if (loadcell.raw_avg[ch] <= ADC_RAIL_LOW || loadcell.raw_avg[ch] >= ADC_RAIL_HIGH) {
            rail |= (uint8_t)(1u << ch);
        }
    }
    loadcell.rail_flag = rail;

    /* 2) staleness: 콜백이 멈추면 값이 얼어붙음 → 제어 피드백 위험. 복구 필요. */
    uint32_t now = osKernelGetTickCount();
    if ((now - loadcell.last_isr_tick) > ADC_STALE_TIMEOUT_MS) {
        loadcell.is_stale = 1;
        /* 콜백 정체 = DMA 정지 → systemTask 가 RECOVERY 수행 중. */
        loadcell.state.raw = CH_PACK(LS_RECOVERING, HC_HW_FAULT);
        return RET_NOK;
    }
    loadcell.is_stale = 0;
    /* 정상 구동. 레일(단선/포화) 있으면 값 범위 밖 경고로 표시(복구 대상 아님). */
    loadcell.state.raw = CH_PACK(LS_RUNNING, rail ? HC_DATA_RANGE : HC_OK);
    return RET_OK;
}

/* OVR/DMA정지 복구: 반드시 Stop → Start 재기동해야 변환이 재개된다(HAL 자동복구 없음). */
RD_RET RD_ADC_RECOVERY(ADC_HandleTypeDef *hadc)
{
    HAL_ADC_Stop_DMA(hadc);
    loadcell.half_ready = 0;
    loadcell.full_ready = 0;

    if (HAL_ADC_Start_DMA(hadc, (uint32_t*)adc_raw, ADC_DMA_LEN) != HAL_OK) {
        return RET_NOK;
    }
    loadcell.last_isr_tick = osKernelGetTickCount();
    loadcell.recover_cnt++;
    return RET_OK;
}

float RD_ADC_GetWeight(uint8_t idx)
{
    if (idx >= NUM_LOADCELLS) return 0.0f;
    return loadcell.weight[idx];
}

uint16_t RD_ADC_GetRaw(uint8_t idx)
{
    if (idx >= NUM_LOADCELLS) return 0;
    return loadcell.raw_avg[idx];
}

void RD_ADC_Tare(void)
{
    for (uint8_t ch = 0; ch < NUM_LOADCELLS; ch++) {
        cal[ch].offset = loadcell.raw_avg[ch];
        loadcell.weight[ch] = 0.0f;
    }
}

/*--------------------------------------------------------------*/
/* ISR: volatile 필드만 기록 + 타임스탬프. 가공/진단은 adcTask.                 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) {
        loadcell.half_ready = 1;
        loadcell.last_isr_tick = osKernelGetTickCount();
        loadcell.ts_stamp      = rd_now_tick();    /* 창 완료 시각 latch (delta_tick 용) */
        loadcell.cb_half_cnt++;                    /* 진단(임시) */
#ifdef RTOS_IS_AVAILABLE
        osThreadFlagsSet(adcTaskHandle, 0x0001);   /* adcTask 기상 */
#endif
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    if (hadc->Instance == ADC1) {
        loadcell.full_ready = 1;
        loadcell.last_isr_tick = osKernelGetTickCount();
        loadcell.ts_stamp      = rd_now_tick();    /* 창 완료 시각 latch (delta_tick 용) */
        loadcell.cb_full_cnt++;                    /* 진단(임시) */
#ifdef RTOS_IS_AVAILABLE
        osThreadFlagsSet(adcTaskHandle, 0x0001);   /* adcTask 기상 */
#endif
    }
}

/* 진단(임시): 5ms 태스크마다 호출. 지난 창의 콜백 수를 스냅샷하고 카운터를 0으로.
 * 카운터는 ISR 에서만 증가하므로, 읽기+리셋 사이 경합을 막기 위해 짧게 IRQ 만 가린다. */
void RD_ADC_DIAG_TICK(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    uint32_t h = loadcell.cb_half_cnt;
    uint32_t f = loadcell.cb_full_cnt;
    loadcell.cb_half_cnt = 0;
    loadcell.cb_full_cnt = 0;
    __set_PRIMASK(primask);

    loadcell.cb_half_per_tick = h;
    loadcell.cb_full_per_tick = f;
    if ((h + f) > loadcell.cb_max_per_tick) {
        loadcell.cb_max_per_tick = h + f;          /* 최악값 = 버퍼 여유 사이징 근거 */
    }
}
