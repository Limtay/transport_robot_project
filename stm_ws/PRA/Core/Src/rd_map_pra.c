/**
 ******************************************************************************
 * @file    rd_map_pra.c
 * @author  swarm
 * @date    2026-09-17
 * @brief   PRA 레지스터 맵 디스패치 + 마샬 레이어 구현부.
 *
 *  DPC_B rd_map_dpcb.c 를 PRA 256B 맵(rd_register_pra.h)에 맞게 이식.
 *
 *  Dispatch: RD_ORIN_HANDLE → DISPATCH_WRITE/READ → find_region(LUT) → reg
 *  Marshal : PUBLISH/CONSUME 은 Step 7 본체 예정 (현재 void 스텁)
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "rd_map_pra.h"
#include "rd_system.h"   /* hw, PRA_uart6, PRA_CONTROL, tim_cnt (SYSTEM 영역 발행) */
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"
#include <string.h>

/* Exported variables --------------------------------------------------------*/
REGISTER_t reg;   /* 256B 레지스터 맵 단일 인스턴스 */

/* Private types -------------------------------------------------------------*/
#define REG_ACC_R   0x01
#define REG_ACC_W   0x02
#define REG_ACC_RW  (REG_ACC_R | REG_ACC_W)

typedef struct {
    uint16_t offset;
    uint16_t size;
    uint8_t  access;
    uint8_t  needs_unlock;
} Region_t;

/* ── 영역 LUT (offset 오름차순, 빈틈 없음) ─────────────────────────────────────
 * addr 0 (sys_write_mode) 은 unlock 키이므로 항상 R/W — 별도 영역 분리.
 * 잠금 대상은 DEFINE(1~15) 만 (시스템 설정 파라미터 오기입 방지).
 * CMD_POWER(65~67) 는 Orin/로컬이 상시 쓰는 전원 제어 영역 → unlock 불필요.
 *   ※ 모드 기반 write 권한(opmode §4/§6) 은 미구현 — 현재 lock 게이팅 영역 없음. */
static const Region_t s_regions[] = {
    /* SYS_WRITE_MODE addr 0 (unlock 키, 항상 R/W) */
    { 0,                        1,                       REG_ACC_RW, 0 },
    /* DEFINE         addr 1~15 (UNLOCK 필요) */
    { 1,                        REG_DEFINE_SIZE - 1,     REG_ACC_RW, 1 },
    /* SYSTEM         addr 16~28 */
    { REG_SYS_OFFSET,           REG_SYS_SIZE,            REG_ACC_R,  0 },
    /* RSVD0          addr 29~37 */
    { REG_RSVD0_OFFSET,         REG_RSVD0_SIZE,          REG_ACC_R,  0 },
    /* SENSOR/ADC     addr 38~47 */
    { REG_SENSOR_ADC_OFFSET,    REG_SENSOR_ADC_SIZE,     REG_ACC_R,  0 },
    /* SENSOR/BMS     addr 48~64 */
    { REG_SENSOR_BMS_OFFSET,    REG_SENSOR_BMS_SIZE,     REG_ACC_R,  0 },
    /* CMD/POWER      addr 65~67 (상시 R/W) */
    { REG_CMD_POWER_OFFSET,     REG_CMD_POWER_SIZE,      REG_ACC_RW, 0 },
    /* RSVD1          addr 68~79 */
    { REG_RSVD1_OFFSET,         REG_RSVD1_SIZE,          REG_ACC_R,  0 },
    /* DIAG           addr 80~99 */
    { REG_DIAG_OFFSET,          REG_DIAG_SIZE,           REG_ACC_R,  0 },
    /* RSVD2          addr 100~255 */
    { REG_RSVD2_OFFSET,         REG_RSVD2_SIZE,          REG_ACC_R,  0 },
};
#define NUM_REGIONS (sizeof(s_regions) / sizeof(s_regions[0]))

/* Private helpers -----------------------------------------------------------*/

static const Region_t *find_region(uint16_t addr)
{
    for (uint32_t i = 0; i < NUM_REGIONS; i++) {
        const Region_t *r = &s_regions[i];
        if (addr < (uint16_t)(r->offset + r->size)) return r;
    }
    return NULL;
}

/**
 * @brief  [addr, addr+len) 에 걸치는 모든 영역을 검사.
 *         access_mask 가 모든 영역에서 set 이어야 OK.
 *         WRITE 시 needs_unlock 영역은 sys_write_mode == SYS_WRITE_UNLOCK 이어야 OK.
 */
static uint8_t check_region_range(uint16_t addr, uint16_t len,
                                   uint8_t access_mask, uint8_t is_write)
{
    if (len == 0)                              return ORIN_ERR_DATA_LEN;
    if ((uint32_t)addr + len > REG_TOTAL_SIZE) return ORIN_ERR_DATA_LEN;

    uint16_t cur = addr;
    uint16_t end = (uint16_t)(addr + len);
    while (cur < end) {
        const Region_t *r = find_region(cur);
        if (r == NULL)                              return ORIN_ERR_ACCESS;
        if ((r->access & access_mask) == 0)         return ORIN_ERR_ACCESS;
        if (is_write && r->needs_unlock &&
            reg.reg_df.sys_write_mode != SYS_WRITE_UNLOCK)
                                                    return ORIN_ERR_ACCESS;
        cur = (uint16_t)(r->offset + r->size);
    }
    return ORIN_ERR_NONE;
}

/* Exported functions --------------------------------------------------------*/

RD_RET RD_MAP_INIT(void)
{
    memset(&reg, 0, sizeof(reg));

    reg.reg_df.sys_write_mode = SYS_WRITE_LOCK;
    reg.reg_df.err_timeout    = DEF_ERR_TIMEOUT;
    reg.reg_df.fatal_timeout  = DEF_FATAL_TIMEOUT;
    reg.reg_df.err_cnt        = DEF_ERR_CNT;
    reg.reg_df.fatal_cnt      = DEF_FATAL_CNT;

    return RET_OK;
}

uint8_t RD_MAP_DISPATCH_WRITE(uint16_t addr, uint16_t len, const uint8_t *src, uint8_t lock)
{
    if (src == NULL) return ORIN_ERR_DATA_LEN;

    uint8_t e = check_region_range(addr, len, REG_ACC_W, 1);
    if (e != ORIN_ERR_NONE) return e;

    /* CMD_POWER 영역(65~67) 에 걸치는 쓰기 판정 (cmd_write_tick 갱신 + 향후 lock 게이팅). */
    uint8_t is_cmd_power = (addr <= REG_CMD_POWER_E_OFFSET) &&
                           ((addr + len - 1) >= REG_CMD_POWER_S_OFFSET);
    /* PRA 는 현재 잠금(모드 기반 권한) 영역 미정의 → lock 무영향. (opmode §4/§6 TODO) */
    if (!is_cmd_power) lock = 0;
    if (lock) return ORIN_ERR_ACCESS;

    taskENTER_CRITICAL();
    if (is_cmd_power) reg.diag.cmd_write_tick = osKernelGetTickCount();
    memcpy((uint8_t*)&reg + addr, src, len);
    taskEXIT_CRITICAL();

    return ORIN_ERR_NONE;
}

uint8_t RD_MAP_DISPATCH_READ(uint16_t addr, uint16_t len, uint8_t *dst)
{
    if (dst == NULL) return ORIN_ERR_DATA_LEN;

    uint8_t e = check_region_range(addr, len, REG_ACC_R, 0);
    if (e != ORIN_ERR_NONE) return e;

    taskENTER_CRITICAL();
    memcpy(dst, (uint8_t*)&reg + addr, len);
    taskEXIT_CRITICAL();

    return ORIN_ERR_NONE;
}

/* ── degraded% 변환 (raw 0~1000 → 0~100%) ────────────────────────────────── */
static inline uint8_t deg_pct(uint16_t raw)
{
    uint32_t v = ((uint32_t)raw * 26u) >> 8;
    return (v > 100u) ? 100u : (uint8_t)v;
}

/**
 * @brief  내부 상태 → reg R/O 영역 발행. **Step6 = SYSTEM 영역만** 구현
 *         (Checker 산출 hw./degraded/sys_state/tick). SENSOR_ADC/BMS 는 Step7
 *         (PERIPHERAL_t/ADC/CAN 정의 후) 본체 채움.
 */
void RD_MAP_MARSHAL_PUBLISH(void)
{
    taskENTER_CRITICAL();
    /* SYSTEM (addr 16~28) — USART6(Orin/uart2 슬롯) 채널 + 시스템 상태 */
    reg.sys.hw_error        = hw.error.raw;
    reg.sys.hw_fatal        = hw.fatal.raw;
    reg.sys.hw_reset        = hw.reset.raw;
    reg.sys.sys_state       = (uint8_t)PRA_CONTROL.STATE;   /* 운용 전원 FSM 미러 (payload_state 폐기, 2026-10-04) */
    reg.sys.degraded_cnt[1] = deg_pct(PRA_uart6.error.degraded_cnt);  /* [1]=uart2(Orin) */
    reg.sys.realtime_tick   = tim_cnt;
    /* degraded_cnt[0]=CAN1 / [2~3]=RSVD : 미구현 0 유지. rs485_proc_delta : 후속 */
    taskEXIT_CRITICAL();

    /* SENSOR/ADC 발행 (의미 기준 매핑). 2026-10-13: current/voltage = int16 ×0.1 단위(×10 스케일),
     * NTC 는 int8 ×1°C 유지. (sensor_bms 는 rd_can_bms 가 직접 발행) */
    for (int i=0; i<3; i++) {
        reg.sensor_adc.ntc[i]		= (int8_t)PRA_PERIPHERAL.NTC[i];                        /* ×1 [°C] */
    }
    reg.sensor_adc.voltage_input 	= (int16_t)(PRA_PERIPHERAL.SENSOR_Voltage_IN  * 10.0f); /* ×0.1 [V] */
    reg.sensor_adc.voltage_output	= (int16_t)(PRA_PERIPHERAL.SENSOR_Voltage_OUT * 10.0f); /* ×0.1 [V] */
    reg.sensor_adc.current_input	= (int16_t)(PRA_PERIPHERAL.SENSOR_Current_BUS * 10.0f); /* ×0.1 [A] */
    //reg.sensor_adc.state			= 0; //TODO : nothing to check (only can check hal_ok or nok)

}

void RD_MAP_MARSHAL_CONSUME(void)
{
    /* TODO(Step7): reg.cmd_power(ctr_mode[3]) → 전원 제어 적용 */
}
