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
#define ROBOT_WHEEL_RADIUS_M   0.105f   /* [m] 바퀴 반지름 */
#define ROBOT_TRACK_WIDTH_M    1.07f   /* [m] 좌우 바퀴 중심 간격 */

/* ── 레이어 원칙 (2026-07-14 리팩토링, 2026-07-17 auto_mode 재분리) ────────
 * systemTask  = 정책/안전 + reg 스테이징: 모든 에러·스테일(AUTO_TIMEOUT)·estop 을
 *               motor_on/ESTOP_override 로 일원화 + 비구동 훅에서 CMD_CLEAR.
 *               AUTO 경로 분기(auto_mode: KINEMATIC/CURRENT/DIRECT/CONTROL)도
 *               ACTION_STATE_AUTO(100Hz) 가 reg.cmd_motor 에 스테이징 —
 *               MANUAL 의 RC_TO_REGISTER 와 대칭.
 * controlTask = 순수 연산(200Hz): CONSUME → LPF → 출력. 상황 판단 분기 없음 —
 *               reg 가 안전값(0)이면 출력도 안전값. 필요한 값은 태스크 루프가
 *               CRIT 스냅샷으로 읽어 인자로 전달 (함수 내 전역 참조 최소화). */

/* Init / reset */
void RD_CONTROL_INIT(void);
void RD_CONTROL_RESET_FILTER(void);

/* 잔류 명령 일괄 청소: reg.cmd_motor(cur/vel) + reg.cmd_system(lin/ang) + cmd_mtr(cur/vel).
 * systemTask 비구동 훅(!motor_on || ESTOP_override) 이 매 tick 호출 (CRIT 보호).
 * LPF 는 미포함 (controlTask 소유) — 전이 리셋은 RD_TASK_CONTROL 담당. */
void RD_CONTROL_CMD_CLEAR(void);

/* RC 수신기 1프레임 → reg.cmd_motor 매핑 (CRIT 보호).
 * MANUAL 모드일 때 systemTask(ACTION_STATE_MANUAL)가 매 tick 호출.
 * receive_flag==0 이면 매핑 skip — 청소는 비구동 훅이 담당. */
void RD_CONTROL_RC_TO_REGISTER(const RECEIVE_comm_t *rc, CMD_MOTOR_t *cm, CMD_SYSTEM_t *cs);

/* 순수 제어 파이프라인: cmd_velocity LPF (cmd 는 CONSUME 완료된 cmd_mtr).
 * use_lpf 는 호출부가 CRIT 스냅샷으로 읽어 전달 — 함수 내 reg/robot_state 참조 없음.
 * 상태 불문 매 tick 실행 (무분기) — 정지는 reg=0 + TX 게이트가 표현. */
void RD_CONTROL_UPDATE(CMD_MOTOR_t *cmd, uint8_t use_lpf);

/* Skid-steer kinematics: cmd_lin_vel [m/s] / cmd_ang_vel [rad/s] → rpm_out[4] [RPM].
 * Motor layout: M0,M1=RIGHT(+RPM=forward) / M2,M3=LEFT(-RPM=forward, mirrored mount).
 * ang_vel > 0 = 좌회전 (CCW from above). AUTO 모드에서 controlTask 가 호출. */
void RD_CONTROL_KINEMATICS(float lin_vel, float ang_vel, float rpm_out[NUM_AK_MOTORS]);

#endif /* INC_RD_CONTROL_H_ */
