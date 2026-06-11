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
 *      systemTask  → MARSHAL_PUBLISH  : data_mtr/data_ecd → reg (memcpy)
 *                                       + sys/uart2/rc state 발행
 *      systemTask  → MARSHAL_CONSUME  : reg.cmd_motor → cmd_mtr (memcpy)
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "rd_map_ecu.h"
#include "rd_system.h"   /* robot_state, tim_cnt, ECU_uart1/2 */
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
static const Region_t s_regions[] = {
    { REG_DEFINE_OFFSET,     REG_DEFINE_SIZE,     REG_ACC_RW, 0 },  /* DEFINE       0~15  */
    { REG_RSVD0_OFFSET,      REG_RSVD0_SIZE,      REG_ACC_R,  0 },  /* RSVD0       16~45  */
    { REG_SYS_OFFSET,        REG_SYS_SIZE,        REG_ACC_R,  0 },  /* SYSTEM      46~61  */
    { REG_IMU_OFFSET,        REG_IMU_SIZE,        REG_ACC_R,  0 },  /* IMU         62~82  */
    { REG_ENCODER_OFFSET,    REG_ENCODER_SIZE,    REG_ACC_R,  0 },  /* ENCODER     83~93  */
    { REG_UART2_OFFSET,      REG_UART2_SIZE,      REG_ACC_R,  0 },  /* UART2          94  */
    { REG_SENSOR_RC_OFFSET,  REG_SENSOR_RC_SIZE,  REG_ACC_R,  0 },  /* SENSOR/RC      95  */
    { REG_MOTOR_DATA_OFFSET, REG_MOTOR_DATA_SIZE, REG_ACC_R,  0 },  /* MOTOR/data  96~127 */
    { REG_CMD_MOTOR_OFFSET,  REG_CMD_MOTOR_SIZE,  REG_ACC_RW, 1 },  /* MOTOR/cmd  128~179 */
    { REG_CMD_SYSTEM_OFFSET, REG_CMD_SYSTEM_SIZE, REG_ACC_RW, 1 },  /* SYSTEM/cmd 180~191 */
    { REG_RSVD1_OFFSET,      REG_RSVD1_SIZE,      REG_ACC_R,  0 },  /* RSVD1      192~223 */
    { REG_DIAG_OFFSET,       REG_DIAG_SIZE,       REG_ACC_R,  0 },  /* DIAG       224~255 */
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

        if (is_write)
        	if (!r->needs_unlock && reg.reg_df.sys_write_mode != SYS_WRITE_UNLOCK)
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

    /* 1) CRIT 밖에서 캐시 */
    uint8_t  st_e  = (uint8_t)robot_state;
    uint32_t tk    = tim_cnt;

    STATE_t u2 = ECU_uart2.error.state;
    STATE_t u1 = ECU_uart1.error.state;
    STATE_t u6 = ECU_uart6.error.state;

    uint8_t deg[8] = {0};
    deg[0] = deg_pct(ECU_uart1.error.degraded_cnt);   /* uart1 RC */
    deg[1] = deg_pct(ECU_uart2.error.degraded_cnt);   /* uart2 RS485 */
    deg[2] = deg_pct(ECU_uart6.error.degraded_cnt);   /* uart6 IMU (구 uart4 슬롯) */
    deg[3] = deg_pct(p->err.can.degraded_cnt);        /* can1 */
    deg[4] = deg_pct(p->err.i2c.degraded_cnt);        /* i2c1 */

    /* 2) CRIT 안에서 reg 갱신 (memcpy 위주) */
    taskENTER_CRITICAL();
    memcpy((void *)&reg.motor_data,   (const void *)&p->data_mtr, sizeof(DATA_MOTOR_t));
    memcpy((void *)&reg.encoder,      (const void *)&p->data_ecd, sizeof(DATA_ENCODER_t));
    memcpy((void *)&reg.sys.hw_reset, (const void *)&hw.reset, 	  sizeof(HW_ERROR_FLAG_t));
    memcpy(reg.sys.degraded_cnt, deg, sizeof(deg));

    /* IMU raw 발행 — IMU_comm_s_t 멤버 순서(quat z,y,x,w / gyro / acc) = DATA_IMU_t 데이터부와 1:1.
     * 물리값 변환 가중치/단위는 rd_register_ecu.h DATA_IMU_t 주석 참조. */
    memcpy((void *)&reg.imu,          (const void *)&ECU_imu.packet, sizeof(IMU_comm_s_t));

    reg.sys.sys_state     = st_e;
    reg.sys.realtime_tick = tk;

    reg.uart2.state = u2;
    reg.rc.state    = u1;
    reg.imu.state   = u6;

    /* cmd_system.mode 는 GPIO MODE 핀 mirror (Phase 1: GPIO master) */
    reg.cmd_system.mode = p->data.MODE;
    taskEXIT_CRITICAL();
}

void RD_MAP_MARSHAL_CONSUME(PERIPHERAL_t *p)
{
    if (p == NULL) return;
    taskENTER_CRITICAL();
    memcpy((void *)&p->cmd_mtr, (const void *)&reg.cmd_motor, sizeof(CMD_MOTOR_t));
    taskEXIT_CRITICAL();
}
