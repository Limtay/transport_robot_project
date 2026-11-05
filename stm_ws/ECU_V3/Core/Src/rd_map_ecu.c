/**
 ******************************************************************************
 * @file    rd_map_ecu.c
 * @author  Kyeongtae
 * @date    2026-05-28
 * @brief   레지스터 맵 디스패치 + 마샬 레이어 구현부.
 *
 *  Dispatch 흐름:
 *      RD_PACKET_HANDLE → DISPATCH_WRITE/READ → find_region (LUT) → memcpy ↔ reg
 *
 *  Marshal 흐름:
 *      rs485Task   → MARSHAL_PUBLISH  : data_mtr/data_ecd → reg (memcpy)
 *                                       + sys/uart2/rc state 발행
 *                                       (요청 직전 발행 + 10ms timeout 기상 — 요청 시점 스냅샷)
 *      controlTask → MARSHAL_CONSUME  : reg.cmd_motor → cmd_mtr (memcpy)
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "rd_map_ecu.h"
#include "rd_system.h"       /* robot_state, ECU_uart1/2/6, ECU_imu, rd_now_tick */
#include "rd_adc.h"          /* loadcell (로드셀 ADC 발행 + ts_stamp) */
#include "rd_i2c_encoder.h"  /* AS5600_Enc[] (채널별 ts_stamp) */
#include "rd_origin.h"       /* ORIGIN[] (POS_FB 원점 필드) */
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
/* addr 0 (sys_write_mode) 은 unlock 키 자체이므로 항상 R/W — 별도 영역으로 분리.
 * 잠금 대상은 DEFINE(1~15) 뿐 — 시스템 설정 파라미터 오기입 방지용.
 * MOTOR/cmd·SYSTEM/cmd 는 Orin 이 상시 쓰는 제어 영역이라 unlock 불필요. */
static const Region_t s_regions[] = {
    { 0,                     1,                   REG_ACC_RW, 0 },  /* SYS_WRITE_MODE  0  (unlock 키, 항상 R/W) */
    { 1,                     REG_DEFINE_SIZE - 1, REG_ACC_RW, 1 },  /* DEFINE       1~15  (UNLOCK 필요)        */
    { REG_SYS_OFFSET,        REG_SYS_SIZE,        REG_ACC_R,  0 },  /* SYSTEM      16~32  */
    { REG_RSVD0_OFFSET,      REG_RSVD0_SIZE,      REG_ACC_R,  0 },  /* RSVD0       33~41  */
    { REG_LOADCELL_OFFSET,   REG_LOADCELL_SIZE,   REG_ACC_R,  0 },  /* LOADCELL    42~47  */
    { REG_IMU_OFFSET,        REG_IMU_SIZE,        REG_ACC_R,  0 },  /* IMU         48~69  */
    { REG_ENCODER_OFFSET,    REG_ENCODER_SIZE,    REG_ACC_R,  0 },  /* ENCODER     70~85  */
    { REG_UART2_OFFSET,      REG_UART2_SIZE,      REG_ACC_R,  0 },  /* UART2          86  */
    { REG_SENSOR_RC_OFFSET,  REG_SENSOR_RC_SIZE,  REG_ACC_R,  0 },  /* SENSOR/RC      87  */
    { REG_MOTOR_DATA_OFFSET, REG_MOTOR_DATA_SIZE, REG_ACC_R,  0 },  /* MOTOR/data  88~127 */
    { REG_CMD_MOTOR_OFFSET,  REG_CMD_MOTOR_SIZE,  REG_ACC_RW, 0 },  /* MOTOR/cmd  128~179 */
    { REG_CMD_SYSTEM_OFFSET, REG_CMD_SYSTEM_SIZE, REG_ACC_RW, 0 },  /* SYSTEM/cmd 180~192 */
    { REG_RSVD1_OFFSET,      REG_RSVD1_SIZE,      REG_ACC_R,  0 },  /* RSVD1      193~223 */
    { REG_DIAG_OFFSET,       REG_DIAG_SIZE,       REG_ACC_R,  0 },  /* DIAG       224~255 */
    { REG_POS_FB_OFFSET,     REG_POS_FB_SIZE,     REG_ACC_R,  0 },  /* POS_FB     256~343 */
};
#define NUM_REGIONS (sizeof(s_regions) / sizeof(s_regions[0]))

/* Private helpers -----------------------------------------------------------*/

/** @brief addr 가 속한 첫 Region_t 를 반환 (LUT 오름차순 + 빈틈 없음 가정).
 *         addr >= r->offset 비교는 생략 — 작은 인덱스부터 검사하므로 첫 매칭이 정답. */
static const Region_t *find_region(uint16_t addr)
{
    for (uint32_t i = 0; i < NUM_REGIONS; i++) {
        const Region_t *r = &s_regions[i];
        if (addr < (uint16_t)(r->offset + r->size)) return r;
    }
    return NULL;
}

/**
 * @brief  [addr, addr+len) 가 걸치는 모든 영역을 검사.
 *         access_mask (REG_ACC_R / REG_ACC_W) 모든 영역에서 set 이어야 OK.
 *         WRITE 시 needs_unlock 영역은 sys_write_mode == UNLOCK 이어야 OK.
 * @retval PACKET_ERR_NONE / PACKET_ERR_ACCESS / PACKET_ERR_DATA_LEN
 */
static uint8_t check_region_range(uint16_t addr, uint16_t len, uint8_t access_mask, uint8_t is_write)
{
    if (len == 0)                              return PACKET_ERR_DATA_LEN;
    if ((uint32_t)addr + len > REG_TOTAL_SIZE) return PACKET_ERR_DATA_LEN;

    uint16_t cur = addr;
    uint16_t end = (uint16_t)(addr + len);
    while (cur < end) {
        const Region_t *r = find_region(cur);
        if (r == NULL)                                      return PACKET_ERR_ACCESS;
        if ((r->access & access_mask) == 0)                 return PACKET_ERR_ACCESS;

        /* [버그 수정] 기존 `!r->needs_unlock` 은 조건 반전 — unlock 이 필요 없는
         * 영역이 LOCK 에 막히고, 정작 CMD 영역은 무조건 통과했었다.
         * needs_unlock 영역은 sys_write_mode == UNLOCK 일 때만 쓰기 허용. */
        if (is_write)
        	if (r->needs_unlock && reg.reg_df.sys_write_mode != SYS_WRITE_UNLOCK)
        		return PACKET_ERR_ACCESS;
        cur = (uint16_t)(r->offset + r->size);
    }
    return PACKET_ERR_NONE;
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

    /* Orin soft ESTOP — 기본 해제 상태 (1). 0 이면 AUTO 모드에서 CAN_AK_ESTOP 제동 */
    reg.cmd_system.soft_estop = SOFT_ESTOP_RELEASE;

    /* cmd_velocity LPF — 기본 사용 (memset 0 이면 부팅 직후 LPF 꺼짐 상태가 되므로 명시 세팅).
     * auto_mode 는 memset 0 == KINEMATIC 이 자연 default. */
    reg.cmd_system.use_lpf = 1;

    /* 활성 모터 마스크 — 기본 4개 전체. 단일 트랙 테스트 시 Orin/RS485 로 0x01 등 write */
    reg.cmd_system.motor_mask = MOTOR_MASK_ALL;
//    reg.cmd_system.motor_mask = 2;

    for (int i = 0; i < NUM_AK_MOTORS; i++) {
        reg.cmd_motor.ctr_mode[i] = DEF_CTR_MODE;
    }
    return RET_OK;
}

uint8_t RD_MAP_DISPATCH_WRITE(uint16_t addr, uint16_t len, const uint8_t *src, uint8_t lock)
{
    if (src == NULL) return PACKET_ERR_DATA_LEN;
    uint8_t e = check_region_range(addr, len, REG_ACC_W, 1);
    if (e != PACKET_ERR_NONE) return e;

    /* TODO: cmd_vel 영역 WRITE tick 을 저장해 상위단 워치독 구현 필요.
     *             AUTO 모드에서 20ms 이상 cmd_vel 갱신이 없으면 강제 모터 정지.
     *  (REG_CMD_VEL_S_OFFSET ~ REG_CMD_VEL_E_OFFSET 범위 겹침 판정 후 osKernelGetTickCount() 기록)
     *  TODO: AUTO(lock을 판단) 모드가 아닌 경우에서는 모터 영역에는 쓰기 금지 */
    uint8_t is_cmd_vel = (addr <= REG_CMD_VEL_E_OFFSET) &&
                         (addr + len - 1 >= REG_CMD_VEL_S_OFFSET);
    if (!is_cmd_vel) lock = 0;
    if (lock) return PACKET_ERR_ACCESS;
    taskENTER_CRITICAL();
    if (is_cmd_vel) reg.diag.cmd_write_tick = osKernelGetTickCount();
    memcpy((uint8_t*)&reg + addr, src, len);
    taskEXIT_CRITICAL();

    return PACKET_ERR_NONE;
}

uint8_t RD_MAP_DISPATCH_READ(uint16_t addr, uint16_t len, uint8_t *dst)
{
    if (dst == NULL) return PACKET_ERR_DATA_LEN;
    uint8_t e = check_region_range(addr, len, REG_ACC_R, 0);
    if (e != PACKET_ERR_NONE) return e;

    taskENTER_CRITICAL();
    memcpy(dst, (uint8_t*)&reg + addr, len);
    taskEXIT_CRITICAL();

    return PACKET_ERR_NONE;
}

/* ── degraded% 변환 (raw 0~1000 → 0~100%) — (raw*26)>>8 ≈ raw/10 ─────────── */
static inline uint8_t deg_pct(uint16_t raw) {
    uint32_t v = ((uint32_t)raw * 26u) >> 8;
    return (v > 100u) ? 100u : (uint8_t)v;
}

void RD_MAP_MARSHAL_PUBLISH(const PERIPHERAL_t *p)
{
    if (p == NULL) return;

    /* 1) CRIT 밖에서 캐시 — 발행 시점 now 1회로 realtime_tick + 전체 delta 를
     *    일관 계산 (Orin 은 realtime_tick − delta 로 각 취득시각 복원, memo_260716 §6~7) */
    uint32_t now  = rd_now_tick();
    uint8_t  st_e = (uint8_t)robot_state;

    STATE_t u2 = ECU_uart2.error.state;
    STATE_t u1 = ECU_uart1.error.state;
    STATE_t u6 = ECU_uart6.error.state;
    STATE_t u4 = ECU_uart4.error.state;

    uint8_t deg[8] = {0};
    deg[0] = deg_pct(ECU_uart1.error.degraded_cnt);   /* uart1 RC */
    deg[1] = deg_pct(ECU_uart2.error.degraded_cnt);   /* uart2 RS485 */
    deg[2] = deg_pct(ECU_uart6.error.degraded_cnt);   /* uart6 IMU (구 uart4 슬롯) */
    deg[3] = deg_pct(p->err.can.degraded_cnt);        /* can1 */
    deg[4] = deg_pct(p->err.i2c.degraded_cnt);        /* i2c1 */
    deg[6] = deg_pct(ECU_uart4.error.degraded_cnt);   /* uart4 RLS — idx5 는 로드셀 예약 (Orin 03 §3.1) */

    /* delta_tick 계산 (드라이버 핸들의 latch ts → uint8 ×0.1ms, 0xFF=stale) */
    uint8_t lc_delta  = rd_delta_tick(now, loadcell.ts_stamp);
    uint8_t imu_delta = rd_delta_tick(now, ECU_imu.ts_stamp);
    uint8_t enc_delta[NUM_ENCODERS];
    for (uint8_t i = 0; i < NUM_ENCODERS; i++) {
#if RD_USE_I2C_ENCODER
        enc_delta[i] = rd_delta_tick(now, AS5600_Enc[i].ts_stamp);
#else
        enc_delta[i] = DELTA_STALE;
#endif
    }
    uint8_t mtr_delta[NUM_AK_MOTORS], cmd_delta[NUM_AK_MOTORS];
    for (uint8_t i = 0; i < NUM_AK_MOTORS; i++) {
        mtr_delta[i] = rd_delta_tick(now, ECU_AK[i].ts_fb);
        cmd_delta[i] = rd_delta_tick(now, ECU_AK[i].ts_cmd);
    }

    /* 2) CRIT 안에서 reg 갱신 (memcpy 위주 — delta 는 memcpy 뒤에 덮어써야 함:
     *    data_mtr/data_ecd 미러에는 delta 필드가 무의미한 값이기 때문) */
    taskENTER_CRITICAL();
    memcpy((void *)&reg.motor_data,   (const void *)&p->data_mtr, sizeof(DATA_MOTOR_t));
    memcpy((void *)&reg.encoder,      (const void *)&p->data_ecd, sizeof(DATA_ENCODER_t));
    memcpy((void *)&reg.sys.hw_error, (const void *)&hw.error, 	  sizeof(HW_ERROR_FLAG_t));
    memcpy(reg.sys.degraded_cnt, deg, sizeof(deg));

    /* IMU raw 발행 — IMU_comm_s_t(20B) 는 DATA_IMU_t 데이터부와 1:1 (delta/state 미포함). */
    memcpy((void *)&reg.imu,          (const void *)&ECU_imu.packet, sizeof(IMU_comm_s_t));

    reg.sys.sys_state     = st_e;
    reg.sys.realtime_tick = now;     /* delta 와 같은 now — 시간축 정합의 기준값 */

    reg.uart2.state = u2;
    reg.rc.state    = u1;
    reg.imu.state   = u6;

    /* 로드셀 ADC (ADC1) — raw 트림평균 ch0~1 + 종합 STATE_t (RD_ADC_CHECKER 가 state 갱신). */
    for (uint8_t i = 0; i < 2 && i < NUM_LOADCELLS; i++)
        reg.loadcell.avg[i] = loadcell.raw_avg[i];
    reg.loadcell.state = loadcell.state;

    /* delta_tick 발행 (memcpy 이후 덮어쓰기) */
    reg.loadcell.delta_tick = lc_delta;
    reg.imu.delta_tick      = imu_delta;
    for (uint8_t i = 0; i < NUM_ENCODERS; i++)
        reg.encoder.delta_tick[i] = enc_delta[i];
    for (uint8_t i = 0; i < NUM_AK_MOTORS; i++) {
        reg.motor_data.delta_tick[i]     = mtr_delta[i];
        reg.motor_data.cmd_delta_tick[i] = cmd_delta[i];
    }

    /* POS_FB (INST_POS 고정 응답) — 모터 값은 100Hz 미러(data_mtr)가 아닌 ECU_AK 최신값. */
    PAGE_POS_FB_t *fb = &reg.pos_fb;
    fb->realtime_tick    = now;
    fb->rs485_proc_delta = reg.sys.rs485_proc_delta;
    fb->sys_state        = st_e;
    fb->hw_reset         = hw.reset.raw;
    memcpy(fb->degraded_cnt, deg, sizeof(deg));

    uint8_t origin_flag = 0;
    for (uint8_t i = 0; i < NUM_AK_MOTORS; i++) {
        fb->mtr_position[i]    = ECU_AK[i].state.position;
        fb->mtr_velocity[i]    = ECU_AK[i].state.velocity;
        fb->mtr_current[i]     = ECU_AK[i].state.current;
        fb->pre_origin_pose[i] = ORIGIN[i].S_pub;
        fb->origin_seq[i]      = ORIGIN[i].seq;
        if (ORIGIN[i].pending)    origin_flag |= (uint8_t)(1u << i);
        if (ORIGIN[i].frame_lost) origin_flag |= (uint8_t)(0x10u << i);
    }
    fb->origin_flag = origin_flag;

    for (uint8_t i = 0; i < 2 && i < NUM_LOADCELLS; i++)
        fb->loadcell_avg[i] = loadcell.raw_avg[i];
    fb->loadcell_state = loadcell.state;
    fb->motor_state    = p->err.can.state;
    fb->rls_state      = u4;

    uint8_t rls_ew = 0;
    for (uint8_t i = 0; i < RLS_NUM; i++) {
        const RLS_comm_t *e = &ECU_rls_mux.enc[i];
        rls_ew |= (uint8_t)(((e->error ? 2u : 0u) | (e->warning ? 1u : 0u)) << (i * 2u));
    }
    fb->rls_ew = rls_ew;

    const RLS_comm_t *lin = &ECU_rls_mux.enc[RLS_CH_LINEAR];
    fb->rls_psi1        = ECU_rls_mux.enc[RLS_CH_PSI1].position;
    fb->rls_psi2        = ECU_rls_mux.enc[RLS_CH_PSI2].position;
    fb->rls_theta3      = (uint16_t)ECU_rls_mux.enc[RLS_CH_THETA3].position;
    fb->lin_zero_result = lin->zero_result;
    fb->lin_zero_seq    = lin->zero_seq;
    fb->lin_x           = lin->lin_x;
    fb->lin_y           = lin->lin_y;

    fb->odiag_motor    = ORIGIN_DIAG.motor;
    fb->odiag_dt       = ORIGIN_DIAG.dt;
    fb->odiag_e_track  = ORIGIN_DIAG.e_track;
    fb->odiag_residual = ORIGIN_DIAG.residual;
    fb->odiag_vel      = ORIGIN_DIAG.vel;
    reg.diag.origin_jump_rej = ORIGIN_jump_rej;
    reg.diag.overspeed_cnt   = ORIGIN_overspeed_cnt;

    /* reg.cmd_system.mode 는 모드 단일 진실원천 (GPIO 토글·Orin write 가 갱신).
     * 여기서 GPIO 값으로 덮어쓰지 않는다 — 덮어쓰면 Orin 원격 write 가 즉시 무효화됨. */
    taskEXIT_CRITICAL();
}

void RD_MAP_MARSHAL_CONSUME(PERIPHERAL_t *p)
{
    if (p == NULL) return;
    taskENTER_CRITICAL();
    memcpy(&p->cmd_mtr, &reg.cmd_motor, sizeof(CMD_MOTOR_t));
    taskEXIT_CRITICAL();
}
