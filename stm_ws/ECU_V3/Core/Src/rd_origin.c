/*
 * rd_origin.c
 *
 *  자율 원점 이동 FSM — 동작·안전 규칙은 rd_origin.h, redesign/11_ecu_position_261030.md §5 참고.
 *
 *  Created on: Nov 5, 2026
 */

/* Includes ------------------------------------------------------------------*/
#include "rd_origin.h"
#include "rd_system.h"
#include "rd_can_motor.h"
#include <math.h>
#include <string.h>

/* Exported variables --------------------------------------------------------*/
ORIGIN_t      ORIGIN[NUM_AK_MOTORS];
ORIGIN_DIAG_t ORIGIN_DIAG;
uint32_t      ORIGIN_jump_rej;

/* Private variables ---------------------------------------------------------*/
static volatile uint32_t s_inst_tick;

/* Private user code ---------------------------------------------------------*/

/* NaN 도 점프로 본다 (비교가 false 라 반대로 쓰면 그대로 통과한다) */
static inline uint8_t is_jump(float a, float b)
{
    return (fabsf(a - b) <= ORIGIN_JUMP_TH_DEG) ? 0u : 1u;
}

static inline int32_t abs_raw(int16_t v) { return (v < 0) ? -(int32_t)v : (int32_t)v; }

/* [deg] → ×0.1° int16 포화 */
static int16_t deg_to_raw(float deg)
{
    float r = deg * 10.0f;
    if (r >  32767.0f) return  32767;
    if (r < -32768.0f) return -32768;
    return (int16_t)r;
}

/* 새 좌표 fb 로 원점 적용 여부 확정. 성공해야만 S·seq 를 발행한다 — Orin offset 은 틀어지지 않는다. */
static void origin_confirm(ORIGIN_t *o, uint8_t i, int16_t fb, uint32_t ts, uint32_t now)
{
    if ((int32_t)(ts - (o->t_origin + ORIGIN_CONFIRM_MARGIN)) > 0) {
        if (abs_raw(fb) < ORIGIN_SHIFT_OK_RAW) {
            /* residual = S 오차 실측: v·(t_new − t_old) − fb_new  (S=fb_old) */
            float v   = (float)o->vel_old * AK_VEL_RAW_TO_DEGPS;
            float res = v * (float)(ts - o->t_fb_old) * 1e-4f - (float)fb * 0.1f;

            taskENTER_CRITICAL();
            o->S_pub      = o->S_raw;
            o->seq++;
            o->shifted    = 1u;
            o->confirming = 0u;
            ORIGIN_DIAG.motor    = i;
            ORIGIN_DIAG.dt       = o->d_dt;
            ORIGIN_DIAG.e_track  = o->d_e_track;
            ORIGIN_DIAG.residual = deg_to_raw(res);
            ORIGIN_DIAG.vel      = o->vel_old;
            taskEXIT_CRITICAL();
        } else {
            o->frame_lost = 1u;
            o->confirming = 0u;
        }
    } else if ((int32_t)(now - o->t_origin) > (int32_t)(ORIGIN_CONFIRM_TO_MS * 10u)) {
        o->frame_lost = 1u;
        o->confirming = 0u;
    }
}

/* Exported user code --------------------------------------------------------*/

void RD_ORIGIN_INIT(void)
{
    memset(ORIGIN, 0, sizeof(ORIGIN));
    memset(&ORIGIN_DIAG, 0, sizeof(ORIGIN_DIAG));
    ORIGIN_DIAG.motor = 0xFFu;
    ORIGIN_jump_rej   = 0u;
    /* 부팅 직후 now≈0 이어도 "수신 없음" 이 되도록 과거 시각으로 (TS_INVALID 와 같은 방식) */
    s_inst_tick = (uint32_t)0u - (ORIGIN_INST_TO_MS * 2u);
}

void RD_ORIGIN_NOTE_INST(void)
{
    s_inst_tick = osKernelGetTickCount();
}

uint8_t RD_ORIGIN_UPDATE(CMD_MOTOR_t *cmd, uint8_t motor_mask)
{
    if (cmd == NULL) return 0u;

    uint8_t  origin_mask = 0u;
    uint32_t now   = rd_now_tick();
    uint8_t  ready = RD_CAN_MOTOR_READY_MASK(motor_mask);
    uint8_t  drive = (ECU_PERIPHERAL.data.motor_on && !ECU_PERIPHERAL.data.ESTOP_override) ? 1u : 0u;

    /* FIFO 설정이 아니면 POSITION 이 SET_ORIGIN 을 앞질러 큰 점프가 난다 (§5.4) → FSM 을 켜지 않는다.
     * CubeMX 재생성으로 설정이 되돌아가도 원점 이동이 위험하게 돌지 않게 하는 방어선. */
    uint8_t  inst  = ((osKernelGetTickCount() - s_inst_tick) <= ORIGIN_INST_TO_MS &&
                      hcan1.Init.TransmitFifoPriority == ENABLE) ? 1u : 0u;

    for (uint8_t i = 0; i < NUM_AK_MOTORS; i++) {
        ORIGIN_t *o   = &ORIGIN[i];
        uint8_t   bit = (uint8_t)(1u << i);
        uint8_t   is_pos = (cmd->ctr_mode[i] == MODE_POSITION) ? 1u : 0u;

        taskENTER_CRITICAL();
        int16_t  fb  = ECU_AK[i].state.position;
        int16_t  vel = ECU_AK[i].state.velocity;
        uint32_t ts  = ECU_AK[i].ts_fb;
        taskEXIT_CRITICAL();

        /* 확인은 활성 여부와 무관 — SET_ORIGIN 이 이미 나갔으므로 중간에 멈춰도 결과를 확정한다 */
        if (o->confirming) origin_confirm(o, i, fb, ts, now);

        /* POSITION 이탈 = Orin 재무장 */
        if (!is_pos) {
            o->frame_lost = 0u;
            o->shifted    = 0u;
        }
        /* 원점을 옮긴 뒤 통신 상실 = 전원 재기동으로 temporary 원점이 사라졌을 수 있다 */
        if (o->shifted && (motor_mask & bit) && !(ready & bit)) o->frame_lost = 1u;

        if (o->frame_lost) {
            /* TX skip 이 아닌 제동: AK 는 마지막 위치 목표를 계속 따라가므로 멈추지 않는다 */
            o->armed      = 0u;
            o->pending    = 0u;
            o->confirming = 0u;
            taskENTER_CRITICAL();
            cmd->ctr_mode[i]    = MODE_CURRENT_BRAKE;
            cmd->cmd_current[i] = BREAK_CURRENT_SW;
            taskEXIT_CRITICAL();
            continue;
        }

        /* 확인 중 재진입: fb·cmd 가 어느 좌표인지 모른다 → 확정될 때까지(수 ms) 이 모터 TX 만 쉰다 */
        if (o->confirming && !o->armed && is_pos) {
            taskENTER_CRITICAL();
            cmd->ctr_mode[i] = MODE_ESTOP;
            taskEXIT_CRITICAL();
            continue;
        }

        if (!(is_pos && inst && drive && (ready & bit))) {
            o->armed   = 0u;
            o->pending = 0u;
            continue;                  /* cmd 는 손대지 않음 — RW 경로 그대로 통과 */
        }

        float c      = cmd->cmd_position[i];
        float fb_deg = (float)fb * 0.1f;

        if (!o->armed) {
            if (is_jump(c, fb_deg)) {  /* 진입 첫 cmd 점프 방지 */
                c = fb_deg;
                ORIGIN_jump_rej++;
            }
            o->armed = 1u;
        } else {
            if (o->pending) {
                if (is_jump(c, o->last_tx)) c -= (float)o->S_raw * 0.1f;  /* 옛 좌표 cmd → 현재 좌표 */
                else                        o->pending = 0u;              /* Orin 이 새 좌표로 넘어옴 */
            }
            if (is_jump(c, o->last_tx)) {
                c = o->last_tx;        /* 보정 후에도 점프면 직전 위치 유지 */
                ORIGIN_jump_rej++;
            }
        }

        if (!o->pending && !o->confirming && abs_raw(fb) >= ORIGIN_TH_RAW) {
            o->S_raw      = fb;
            o->vel_old    = vel;
            o->t_fb_old   = ts;
            o->t_origin   = now;
            o->d_dt       = rd_delta_tick(now, ts);
            o->d_e_track  = deg_to_raw(c - fb_deg);
            o->pending    = 1u;
            o->confirming = 1u;
            origin_mask  |= bit;
            c -= (float)o->S_raw * 0.1f;   /* 이번 tick TX 부터 새 좌표 */
        }

        o->last_tx = c;
        taskENTER_CRITICAL();
        cmd->cmd_position[i] = c;
        taskEXIT_CRITICAL();
    }
    return origin_mask;
}
