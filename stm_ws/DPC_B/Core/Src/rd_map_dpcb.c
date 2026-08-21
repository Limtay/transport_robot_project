/**
 ******************************************************************************
 * @file    rd_map_dpcb.c
 * @author  swarm
 * @date    2026-06-25
 * @brief   DPC_B 레지스터 맵 디스패치 + 마샬 레이어 구현부.
 *
 *  Dispatch 흐름:
 *      RD_ORIN_HANDLE → DISPATCH_WRITE / DISPATCH_READ → find_region (LUT) → reg
 *
 *  peri 흐름:
 *      systemTask  → dpca_PUBLISH : PERIPHERAL 상태 → reg R/O 영역 갱신 (구현 완료)
 *      controlTask → dpca_CONSUME : reg.cmd_* → PERIPHERAL 적용 (Step 7 예정)
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "rd_map_dpcb.h"
#include "rd_system.h"   /* DPC_CTL, tim_cnt, hw, DPCB_uart2/4/6 */
#include "cmsis_os.h"
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

/* ── 영역 LUT (offset 오름차순, 빈틈 없음) ───────────────────────────────── */
/* addr 0 (sys_write_mode) 은 unlock 키이므로 항상 R/W — 별도 영역으로 분리.
 * 잠금 대상은 DEFINE(1~15) 만 — 시스템 설정 파라미터 오기입 방지.
 * CMD_* 영역 (120~142) 은 Orin 이 상시 쓰는 제어 영역이라 unlock 불필요.
 * CMD_MOT(128~142) 는 lock 파라미터로 AUTO 모드 시에만 허용 (DISPATCH_WRITE 로직). */
static const Region_t s_regions[] = {
    /* SYS_WRITE_MODE addr 0 (unlock 키, 항상 R/W) */
    { 0,                       1,                        REG_ACC_RW, 0 },
    /* DEFINE         addr 1~15 (UNLOCK 필요) */
    { 1,                       REG_DEFINE_SIZE - 1,      REG_ACC_RW, 1 },
    /* RSVD0          addr 16~45 */
    { REG_RSVD0_OFFSET,        REG_RSVD0_SIZE,           REG_ACC_R,  0 },
    /* SYSTEM         addr 46~61 */
    { REG_SYS_OFFSET,          REG_SYS_SIZE,             REG_ACC_R,  0 },
    /* SENSOR/DPCA    addr 62~64 */
    { REG_SENSOR_DPCA_OFFSET,  REG_SENSOR_DPCA_SIZE,     REG_ACC_R,  0 },
    /* UART2/data     addr 65 */
    { REG_UART2_OFFSET,        REG_UART2_SIZE,           REG_ACC_R,  0 },
    /* GPIO/data      addr 66~73 */
    { REG_SENSOR_DPCB_OFFSET,  REG_SENSOR_DPCB_SIZE,     REG_ACC_R,  0 },
    /* MOTOR/data     addr 74~110 */
    { REG_MOTOR_DATA_OFFSET,   REG_MOTOR_DATA_SIZE,      REG_ACC_R,  0 },
    /* RSVD1          addr 111~119 */
    { REG_RSVD1_OFFSET,        REG_RSVD1_SIZE,           REG_ACC_R,  0 },
    /* CMD/DPCA       addr 120~121 */
    { REG_CMD_DPCA_OFFSET,     REG_CMD_DPCA_SIZE,        REG_ACC_RW, 0 },
    /* CMD/DPCB       addr 122~127 */
    { REG_CMD_DPCB_OFFSET,     REG_CMD_DPCB_SIZE,        REG_ACC_RW, 0 },
    /* CMD/MOT        addr 128~142 — AUTO 시에만 lock=0 허용 (DISPATCH_WRITE 에서 판단) */
    { REG_CMD_MOT_OFFSET,      REG_CMD_MOT_SIZE,         REG_ACC_RW, 0 },
    /* RSVD2          addr 143~174 */
    { REG_RSVD2_OFFSET,        REG_RSVD2_SIZE,           REG_ACC_R,  0 },
    /* DIAG           addr 175~206 */
    { REG_DIAG_OFFSET,         REG_DIAG_SIZE,            REG_ACC_R,  0 },
    /* RSVD3          addr 207~255 */
    { REG_RSVD3_OFFSET,        REG_RSVD3_SIZE,           REG_ACC_R,  0 },
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
 *         access_mask (REG_ACC_R / REG_ACC_W) 가 모든 영역에서 set 이어야 OK.
 *         WRITE 시 needs_unlock 영역은 sys_write_mode == SYS_WRITE_UNLOCK 이어야 OK.
 * @retval ORIN_ERR_NONE / ORIN_ERR_ACCESS / ORIN_ERR_DATA_LEN
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

    /* 기본 전류 제한 설정 (DEF_GOAL_CURRENT × 2.69mA ≈ 2.0A) */
    for (int i = 0; i < 3; i++) {
        reg.cmd_mot.goal_current[i] = DEF_GOAL_CURRENT;
    }
    return RET_OK;
}

uint8_t RD_MAP_DISPATCH_WRITE(uint16_t addr, uint16_t len, const uint8_t *src, uint8_t lock)
{
    if (src == NULL) return ORIN_ERR_DATA_LEN;

    uint8_t e = check_region_range(addr, len, REG_ACC_W, 1);
    if (e != ORIN_ERR_NONE) return e;

    /* CMD_MOT 영역(128~142) 쓰기 시 lock 판단:
     * AUTO 모드가 아닌 경우(lock=1) 모터 직접 제어 차단.
     * CMD_DPCA(120~121), CMD_DPCB(122~127) 는 lock 무관 항상 허용. */
    uint8_t is_cmd_mot = (addr <= REG_CMD_MOT_E_OFFSET) &&
                         ((addr + len - 1) >= REG_CMD_MOT_S_OFFSET);
    if (!is_cmd_mot) lock = 0;
    if (lock) return ORIN_ERR_ACCESS;

    taskENTER_CRITICAL();
    if (is_cmd_mot) reg.diag.cmd_write_tick = osKernelGetTickCount();
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

/* ── dpca_PUBLISH : PERIPHERAL → reg R/O 영역 (systemTask 10ms) ──────────── */

void RD_MAP_MARSHAL_PUBLISH(const PERIPHERAL_t *p)
{
    /* ── SYSTEM 영역 스냅샷 (CRITICAL 진입 전) ─────────────────────────────── */
    /* sys_state(addr 57): 운용 상태 DPCB_STATE_e 값을 DPC_CTL.STATE 에서 단방향 복사.
     * (구: payload_state/SYSTEM_STATE_e — 현재 payload_state 는 내부 FAULT 처리 전용) */
    uint8_t  s_state    = (uint8_t)DPC_CTL.STATE;
    uint32_t s_tick     = tim_cnt;
    uint8_t  s_hw_reset = hw.reset.raw;
    uint8_t  s_hw_fatal = hw.fatal.raw;
    uint8_t  s_hw_error = hw.error.raw;
    uint8_t  s_deg[8]   = {0};

    s_deg[0] = deg_pct(DPCB_uart2.error.degraded_cnt);  /* idx0: uart2 (Orin RS485) */
    s_deg[1] = deg_pct(DPCA_uart4.error.degraded_cnt);  /* idx1: uart4 (DPC-A)      */
    s_deg[2] = deg_pct(DPCB_uart6.error.degraded_cnt);  /* idx2: uart6 (Dynamixel)  */
    /* TODO: idx3 = i2c(MCP23017) 채널 오염도 — Checker 미구현, 0 유지 */

    /* ── 채널 종합 상태 스냅샷 (STATE_t) ────────────────────────────────────── */
    STATE_t s_uart2 = DPCB_uart2.error.state;   /* Orin RS485 (USART2) */
    STATE_t s_uart4 = DPCA_uart4.error.state;   /* DPC-A     (UART4)   */
    STATE_t s_uart6 = DPCB_uart6.error.state;   /* Dynamixel (USART6)  */

    /* ── MOTOR/센서/패널 스냅샷 (PERIPHERAL) ────────────────────────────────── */
    int32_t s_pos[DYN_NUM_MOTORS]  = {0}, s_vel[DYN_NUM_MOTORS] = {0};
    int16_t s_cur[DYN_NUM_MOTORS]  = {0};
    uint8_t s_temp[DYN_NUM_MOTORS] = {0}, s_hwerr[DYN_NUM_MOTORS] = {0};
    uint8_t s_prox = 0, s_alock = 0, s_block = 0;
    //uint8_t s_mode = 0;
    //uint8_t s_state = 0;
    uint8_t s_sw[6] = {0};

    if (p != NULL) {
        for (int i = 0; i < DYN_NUM_MOTORS; i++) {
            const DYN_STATE_t *st = &p->MOT[i].dyn_ctrl.ram.state;
            s_pos[i]   = st->present_position;              /* int32 직접 복사        */
            s_vel[i]   = st->present_velocity;              /* int32 직접 복사        */
            s_cur[i]   = st->present_current;               /* int16 직접 복사        */
            s_temp[i]  = st->present_temperature;           /* u8 직접 복사           */
            s_hwerr[i] = p->MOT[i].dyn_ctrl.hardware_error; /* ram.state 아닌 ctrl 필드 */
        }
        s_prox  = p->A_PROX_DATA;    /* DPC-A 지면 근접 3bit  */
        s_alock = p->A_CON_DATA;     /* DPC-A 집게 접점 4bit  */
        s_block = p->CON_DATA;       /* DPC-B 고정 접점 4bit  */

        //s_mode = DPC_CTL.MODE;
        s_state = DPC_CTL.STATE;

        s_sw[0] = p->PANEL.SW1_state;
        s_sw[1] = p->PANEL.SW2_state;
        s_sw[2] = p->PANEL.SW3_state;
        s_sw[3] = p->PANEL.SW4_state;
        s_sw[4] = p->PANEL.SW5_state;
        s_sw[5] = p->PANEL.SW6_state;
    }

    /* ── 일괄 기입 (CRITICAL) ───────────────────────────────────────────────── */
    taskENTER_CRITICAL();
    /* SYSTEM */
    for (int i = 0; i < 8; i++) reg.sys.degraded_cnt[i] = s_deg[i];
    reg.sys.hw_reset      = s_hw_reset;
    reg.sys.hw_fatal      = s_hw_fatal;
    reg.sys.hw_error      = s_hw_error;
    reg.sys.sys_state     = s_state;
    reg.sys.realtime_tick = s_tick;
    /* 채널 종합 상태 */
    reg.uart2.state             = s_uart2;
    reg.sensor_dpca.uart4_state = s_uart4;
    reg.motor_data.uart6_state  = s_uart6;

    if (p != NULL) {
        /* MOTOR 피드백 (ID=2/3/4 = MOT[0/1/2] 순서) */
        for (int i = 0; i < DYN_NUM_MOTORS; i++) {
            reg.motor_data.present_position[i]    = s_pos[i];
            reg.motor_data.present_velocity[i]    = s_vel[i];
            reg.motor_data.present_current[i]     = s_cur[i];
            reg.motor_data.present_temperature[i] = s_temp[i];
            reg.motor_data.hardware_error[i]      = s_hwerr[i];
        }
        /* 센서 (DPC-A / DPC-B) */
        reg.sensor_dpca.prox_contact = s_prox;
        reg.sensor_dpca.lock_contact = s_alock;
        reg.sensor_dpcb.lock_contact = s_block;

        //reg.cmd_dpcb.mode = s_mode;
        reg.sys.sys_state = s_state;
        /* 패널 스위치 (SPST: 0=OFF/1=ON, SPDT: 0=IDLE/1=UP/2=DOWN) */
        for (int i = 0; i < 6; i++) reg.sensor_dpcb.ex_sw[i] = s_sw[i];
        /* TODO: reg.sensor_dpcb.panel_state — i2c(MCP23017) 채널 상태 미구현, 미기입(0 유지) */
    }
    taskEXIT_CRITICAL();
}

void RD_MAP_MARSHAL_CONSUME(PERIPHERAL_t *p)
{
    //(void)p;
    if(reg.cmd_dpcb.sys_state_target != 0xff){
    	DPC_CTL.STATE = reg.cmd_dpcb.sys_state_target;
    	reg.cmd_dpcb.sys_state_target = 0xff;
    }
    if(reg.cmd_dpcb.mode != 0xff){
    	DPC_CTL.MODE = reg.cmd_dpcb.mode;
    	reg.cmd_dpcb.mode = 0xff;
    }

    if (DPC_CTL.MODE == 1) {
    	DPCB_PERIPHERAL.SERVO_EN = reg.cmd_dpcb.servo_cmd;
    	if(DPC_CTL.STATE < 2){
    		DPCB_PERIPHERAL.LIGHT_EN = reg.cmd_dpcb.light_en;
    	}
    }
    DPCB_PERIPHERAL.B_EN_BOOT = reg.cmd_dpcb.boot_en;
    DPCB_PERIPHERAL.A_EN_BOOT = reg.cmd_dpcb.boot_en;
    /* TODO Step 7:
     * 적용 항목 (CRITICAL 안에서 스냅샷 → CRITICAL 밖에서 분배):
     *
     * 1) reg.cmd_dpca → DPCA_PACKET.tx.Data (dpcaTask 가 전송)
     *    reg.cmd_dpca.locker_en → DPCA_PACKET.tx.Data[0]
     *    reg.cmd_dpca.boot_en   → DPCA_PACKET.tx.Data[1]
     *
     * 2) reg.cmd_dpcb → DPCB_PERIPHERAL.*
     *    reg.cmd_dpcb.locker_en  → DPCB_PERIPHERAL.EN_ALL (혹은 CON_DATA)
     *    reg.cmd_dpcb.boot_en    → DPCB_PERIPHERAL.EN_BOOT											check
     *    reg.cmd_dpcb.light_en   → DPCB_PERIPHERAL.LIGHT_EN										check
     *    reg.cmd_dpcb.servo_cmd  → DPCB_PERIPHERAL.SERVO_EN										check
     *    reg.cmd_dpcb.mode             → DPC_CTL (mode 전환 트리거, DPCB_MODE_e)
     *    reg.cmd_dpcb.sys_state_target → DPC_CTL.STATE (목표 상태 지정, DPCB_STATE_e / 입력 mask 적용)
     *
     * 3) reg.cmd_mot → DPCB_PERIPHERAL.MOT[i].dyn_ctrl.*
     *    reg.cmd_mot.torque_en[i]     → (토크 제어)
     *    reg.cmd_mot.goal_position[i] → dyn_ctrl.ram.cmd.goal_position
     *    reg.cmd_mot.goal_current[i]  → dyn_ctrl.ram.cmd.goal_current
     */
}
