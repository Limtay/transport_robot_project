/*
 * rd_control.h
 *
 *  Control layer (Layer 3):
 *  - RC 수신기 입력 → reg.cmd_motor (단일 source-of-truth)
 *  - reg.cmd_motor 기반 LPF → PERIPHERAL.cmd 출력
 *  - 향후 cmd_lin/ang_vel → skid-steer kinematics 확장 예정 (Phase 2)
 *
 *  Created on: 2026. 5. 14.
 *      Author: Kyeongtae
 */

#ifndef INC_RD_CONTROL_H_
#define INC_RD_CONTROL_H_

#include "stm32f4xx_hal.h"
#include "rd_common.h"
#include "rd_register_ecu.h"
#include "rd_peripheral_ecu.h"
#include "rd_comm_receive.h"
#include "rd_system.h"   /* SYSTEM_STATE_e */

/* ── Skid-steer kinematics 물리 파라미터 (수정 시 여기만 변경) ──────────── */
#define ROBOT_WHEEL_RADIUS_M   0.10f   /* [m] 바퀴 반지름 */
#define ROBOT_TRACK_WIDTH_M    1.10f   /* [m] 좌우 바퀴 중심 간격 */

/* Init / reset */
void RD_CONTROL_INIT(void);
void RD_CONTROL_RESET_FILTER(void);

/* RC 수신기 1프레임 → reg.cmd_motor 매핑 (CRIT 보호).
 * MANUAL 모드일 때 controlTask 가 매 tick 호출. */
void RD_CONTROL_RC_TO_REGISTER(const RECEIVE_comm_t *rc, CMD_MOTOR_t *cm);

/* LPF 적용: cmd->cmd_velocity / cmd->cmd_current 를 in-place 필터링.
 * INIT/FAULT/ESTOP 모드에서는 필터 리셋 (브레이크 명령은 PERIPHERAL_WRITE 가 LPF 우회). */
void RD_CONTROL_UPDATE(volatile CMD_MOTOR_t *cmd, SYSTEM_STATE_e s);

/* Skid-steer kinematics: cmd_lin_vel [m/s] / cmd_ang_vel [rad/s] → rpm_out[4] [RPM].
 * Motor layout: M0,M1=RIGHT(+RPM=forward) / M2,M3=LEFT(-RPM=forward, mirrored mount).
 * ang_vel > 0 = 좌회전 (CCW from above). ctr_flag=1 일 때 systemTask 가 호출. */
void RD_CONTROL_KINEMATICS(float lin_vel, float ang_vel, float rpm_out[NUM_AK_MOTORS]);

#endif /* INC_RD_CONTROL_H_ */
