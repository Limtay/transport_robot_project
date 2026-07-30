/*
 * rd_control.c
 *
 *  LPF + RC 입력 매핑 (Phase 1).
 *  향후 skid-steer kinematics 등 제어 알고리즘은 RD_CONTROL_UPDATE 내부 확장.
 *
 *  Created on: 2026. 5. 14.
 *      Author: Kyeongtae
 */

#include "rd_control.h"
#include "cmsis_os.h"
#include "can_ak.h"           /* AK_Control_Mode_t */
#include <string.h>

/* Private state -------------------------------------------------------------*/
static float    s_filtered_vel[NUM_AK_MOTORS];

static const float LPF_ALPHA = 0.05f;

/* Public --------------------------------------------------------------------*/
void RD_CONTROL_INIT(void)
{
    memset(s_filtered_vel, 0, sizeof(s_filtered_vel));
}

void RD_CONTROL_RESET_FILTER(void)
{
    memset(s_filtered_vel, 0, sizeof(s_filtered_vel));
}

/* 잔류 명령 일괄 청소 — reg.cmd_motor(cur/vel) + reg.cmd_system(lin/ang) + cmd_mtr(cur/vel).
 * 호출: systemTask 비구동 훅 (!motor_on || ESTOP_override, RD_TASK_SYSTEM 참조).
 * reg 를 0 으로 만들면 무분기 control 파이프라인이 자동으로 0 을 계산 —
 * "정지" 가 코드 분기가 아니라 데이터로 표현된다.
 * ctr_mode 는 건드리지 않음 (모드 전환 글리치 회피 — 기존 원칙).
 * LPF 는 제외 (controlTask 소유) — 전이 리셋은 RD_TASK_CONTROL 이 담당. */
void RD_CONTROL_CMD_CLEAR(void)
{
    taskENTER_CRITICAL();
    for (int i = 0; i < NUM_AK_MOTORS; i++) {
        reg.cmd_motor.cmd_current[i]           = 0.0f;
        reg.cmd_motor.cmd_velocity[i]          = 0.0f;
        ECU_PERIPHERAL.cmd_mtr.cmd_current[i]  = 0.0f;
        ECU_PERIPHERAL.cmd_mtr.cmd_velocity[i] = 0.0f;
    }
    reg.cmd_system.cmd_lin_vel = 0.0f;
    reg.cmd_system.cmd_ang_vel = 0.0f;
    taskEXIT_CRITICAL();
}

void RD_CONTROL_RC_TO_REGISTER(const RECEIVE_comm_t *rc, CMD_MOTOR_t *cm, CMD_SYSTEM_t *cs)
{
    if (rc == NULL || cm == NULL || cs == NULL) return;

    /* receive_flag 하강(조종기 신호 끊김/플래그 OFF): stale 스틱값 매핑 skip.
     * 잔류 명령 청소는 비구동 훅(motor_on=0 → CMD_CLEAR, RD_TASK_SYSTEM)이 담당 —
     * 청소 경로 단일화. 여기선 reg 에 쓰지 않는 것만 보장. */
    if (!rc->receive_flag) return;

    /* 1) selector[0] → weight 매핑 (0=정지/1=×0.15/2=×0.50/3=×1.00) */
    uint8_t weight = rc->selector[0] & 0x03;

    /* 2) selector[1] == 3 → CURRENT 모드, 그 외 → VELOCITY 모드 */
    uint8_t ctrl_mode = (rc->selector[1] == 3) ? MODE_CURRENT : MODE_VELOCITY;

    /* 3) throttle scale: 같은 weight 라도 mode 에 따라 단위가 다름 */
    float scale;
    switch (weight) {
        case 1:  scale = 0.15f; break;
        case 2:  scale = 0.5f;  break;
        case 3:  scale = 1.0f;  break;
        default: scale = 0.0f;  break;
    }
    scale *= (ctrl_mode == MODE_CURRENT) ? 0.006f : 10.0f;

    /* 4) thrr/diff → 4 모터 분배 (mode_flag: 0=2-throttle / 1=1-throttle 레이아웃) */
    float t[4];
    if (rc->mode_flag) {
        /* 1-throttle */
        t[0] =  rc->thrr1 - rc->diff1;
        t[1] =  t[0];
        t[2] = -rc->thrr1 - rc->diff1;
        t[3] =  t[2];
    } else {
        /* 2-throttle */
        t[0] =  rc->thrr1 - rc->diff1;
        t[1] =  rc->thrr2 - rc->diff2;
        t[2] = -rc->thrr2 - rc->diff2;
        t[3] = -rc->thrr1 - rc->diff1;
    }
    for (int i = 0; i < NUM_AK_MOTORS; i++) t[i] *= scale;

    /* 5) reg.cmd_motor 쓰기 (CRIT 보호) — weight 는 scale 계산용 로컬 (reg 발행 폐기, addr 188 은 auto_mode) */
    taskENTER_CRITICAL();
    for (int i = 0; i < NUM_AK_MOTORS; i++) cm->ctr_mode[i] = ctrl_mode;
    if (ctrl_mode == MODE_CURRENT) {
        cs->use_lpf = 0;
        for (int i = 0; i < NUM_AK_MOTORS; i++) {
            cm->cmd_current[i]  = t[i];
            cm->cmd_velocity[i] = 0.0f;
        }
    } else {
        cs->use_lpf = 1;
        for (int i = 0; i < NUM_AK_MOTORS; i++) {
            cm->cmd_velocity[i] = t[i];
            cm->cmd_current[i]  = 0.0f;
        }
    }
    taskEXIT_CRITICAL();
}

void RD_CONTROL_KINEMATICS(float lin_vel, float ang_vel, float rpm_out[NUM_AK_MOTORS])
{
    /* M-6: 입력 클램핑 — LPF saturation 후 transient overshoot 방지 */
#define KIN_MAX_LIN_MPS  10.0f
#define KIN_MAX_ANG_RPS   5.0f
    if (lin_vel >  KIN_MAX_LIN_MPS) lin_vel =  KIN_MAX_LIN_MPS;
    if (lin_vel < -KIN_MAX_LIN_MPS) lin_vel = -KIN_MAX_LIN_MPS;
    if (ang_vel >  KIN_MAX_ANG_RPS) ang_vel =  KIN_MAX_ANG_RPS;
    if (ang_vel < -KIN_MAX_ANG_RPS) ang_vel = -KIN_MAX_ANG_RPS;

    /* MPS_TO_RPM = 60 / (2π × R): m/s → RPM 변환 계수
     * 168 = Gear ratio 8 x Pole pair 21 */
    static const float MPS_TO_RPM = 168.0f * 60.0f / (2.0f * 3.14159265f * ROBOT_WHEEL_RADIUS_M);
    const float half_w = ROBOT_TRACK_WIDTH_M * 0.5f;

    float v_right = lin_vel + ang_vel * half_w;   /* 우측 선속도 [m/s] */
    float v_left  = lin_vel - ang_vel * half_w;   /* 좌측 선속도 [m/s] */

    rpm_out[0] =  v_right * MPS_TO_RPM;  /* M0: RIGHT */
    rpm_out[1] =  v_right * MPS_TO_RPM;  /* M1: RIGHT */
    rpm_out[2] = -v_left  * MPS_TO_RPM;  /* M2: LEFT (반전 장착) */
    rpm_out[3] = -v_left  * MPS_TO_RPM;  /* M3: LEFT (반전 장착) */
}

/* 순수 제어 파이프라인 (무분기 정책): LPF 만.
 * AUTO 경로 분기(auto_mode: KINEMATIC/CURRENT/DIRECT/CONTROL)는 전부
 * systemTask(ACTION_STATE_AUTO, 100Hz)가 reg 에 스테이징 완료 — 여기는 상태 판단 0.
 * 정지/차단의 상황 판단도 전부 상위(systemTask)가 소유 —
 *   reg 청소   : 비구동 훅 CMD_CLEAR (!motor_on || ESTOP_override)
 *   TX 게이트  : PERIPHERAL_WRITE (motor_on skip / override 제동)
 *   스테일 워치독 : AUTO_TIMEOUT (rd_system.h) → motor_on=0 + 훅 청소
 * cmd 는 CONSUME 완료된 cmd_mtr, use_lpf 는 호출부(RD_TASK_CONTROL)가 CRIT 로
 * 읽은 스냅샷 — 이 함수는 전역 reg/robot_state 를 참조하지 않는다. */
void RD_CONTROL_UPDATE(CMD_MOTOR_t *cmd, uint8_t use_lpf)
{
    if (cmd == NULL) return;

    /* cmd_velocity LPF in-place (cmd_current 는 즉각 반응을 위해 LPF 없음).
     * 비구동 구간엔 reg=0(CMD_CLEAR) 입력으로 자연 감쇠, 전이/use_lpf 상승엣지의
     * 0 재시작은 RD_TASK_CONTROL 의 전이 감지 RESET_FILTER 가 보장. */
    if (use_lpf) {
        for (int i = 0; i < NUM_AK_MOTORS; i++) {
            s_filtered_vel[i] += LPF_ALPHA * (cmd->cmd_velocity[i] - s_filtered_vel[i]);
            cmd->cmd_velocity[i] = s_filtered_vel[i];
        }
    }
}
