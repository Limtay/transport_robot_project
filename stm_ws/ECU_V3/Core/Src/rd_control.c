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

void RD_CONTROL_RC_TO_REGISTER(const RECEIVE_comm_t *rc, CMD_MOTOR_t *cm, CMD_SYSTEM_t *cs)
{
    if (rc == NULL || cm == NULL || cs == NULL) return;
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
        t[0] = -rc->thrr1 - rc->diff1;
        t[1] = -rc->thrr2 - rc->diff2;
        t[2] =  rc->thrr2 - rc->diff2;
        t[3] =  rc->thrr1 - rc->diff1;
    }
    for (int i = 0; i < NUM_AK_MOTORS; i++) t[i] *= scale;

    /* 5) reg.cmd_motor 쓰기 (CRIT 보호) */
    taskENTER_CRITICAL();
    cs->weight = weight;
    for (int i = 0; i < NUM_AK_MOTORS; i++) cm->ctr_mode[i] = ctrl_mode;
    if (ctrl_mode == MODE_CURRENT) {
        for (int i = 0; i < NUM_AK_MOTORS; i++) {
            cm->cmd_current[i]  = t[i];
            cm->cmd_velocity[i] = 0.0f;
        }
    } else {
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

    /* MPS_TO_RPM = 60 / (2π × R): m/s → RPM 변환 계수 */
    static const float MPS_TO_RPM = 60.0f / (2.0f * 3.14159265f * ROBOT_WHEEL_RADIUS_M);
    const float half_w = ROBOT_TRACK_WIDTH_M * 0.5f;

    float v_right = lin_vel + ang_vel * half_w;   /* 우측 선속도 [m/s] */
    float v_left  = lin_vel - ang_vel * half_w;   /* 좌측 선속도 [m/s] */

    rpm_out[0] =  v_right * MPS_TO_RPM;  /* M0: RIGHT */
    rpm_out[1] =  v_right * MPS_TO_RPM;  /* M1: RIGHT */
    rpm_out[2] = -v_left  * MPS_TO_RPM;  /* M2: LEFT (반전 장착) */
    rpm_out[3] = -v_left  * MPS_TO_RPM;  /* M3: LEFT (반전 장착) */
}

void RD_CONTROL_UPDATE(volatile CMD_MOTOR_t *cmd, SYSTEM_STATE_e s)
{
    if (cmd == NULL) return;

    /* 비상정지/페일 모드: LPF 우회 (cmd 는 호출자가 결정, 필터만 리셋) */
    if (s != SYS_STATE_MANUAL && s != SYS_STATE_AUTO) {
        RD_CONTROL_RESET_FILTER();
        return;
    }
	// REGISTER -> PERIPHERAL
    RD_MAP_MARSHAL_CONSUME(&ECU_PERIPHERAL);

    taskENTER_CRITICAL();
    uint8_t use_kin = reg.cmd_system.ctr_flag;
    taskEXIT_CRITICAL();
	if (robot_state == SYS_STATE_AUTO && use_kin) {
		/* kinematics mode (ctr_flag=1): lin/ang_vel → cmd_mtr.cmd_velocity[] 덮어쓰기 */
		float lin, ang;
		taskENTER_CRITICAL();
		lin     = reg.cmd_system.cmd_lin_vel;
		ang     = reg.cmd_system.cmd_ang_vel;
		taskEXIT_CRITICAL();

		float rpm_out[NUM_AK_MOTORS];
		RD_CONTROL_KINEMATICS(lin, ang, rpm_out);
		taskENTER_CRITICAL();
		for (int i = 0; i < NUM_AK_MOTORS; i++) {
			ECU_PERIPHERAL.cmd_mtr.cmd_velocity[i] = rpm_out[i];
			ECU_PERIPHERAL.cmd_mtr.ctr_mode[i]     = MODE_VELOCITY;
		}
		taskEXIT_CRITICAL();

	}
    /* MANUAL / AUTO: cmd_velocity에 LPF in-place
     * cmd_current는 즉각 반응을 위해 LPF 삭제 */
    for (int i = 0; i < NUM_AK_MOTORS; i++) {
        s_filtered_vel[i] += LPF_ALPHA * (cmd->cmd_velocity[i] - s_filtered_vel[i]);
        cmd->cmd_velocity[i] = s_filtered_vel[i];
    }
}
