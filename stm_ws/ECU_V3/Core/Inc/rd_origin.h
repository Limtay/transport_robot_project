/*
 * rd_origin.h
 *
 *  INST_POS 위치 제어의 자율 원점 이동. AK 피드백은 ±3200° 까지만 읽히므로 3000° 에서
 *  모터 0점을 현재 위치로 옮기고, Orin 이 알기 전까지 들어오는 옛 좌표 cmd 를 대신 보정한다.
 *  Orin 은 origin_seq 가 바뀌면 offset += pre_origin_pose 만 하면 된다.
 *  상세: redesign/11_ecu_position_261030.md §5
 *
 *  소유: 상태는 controlTask 단독. MARSHAL_PUBLISH 는 CRIT 안에서 읽기만 한다.
 *  INST_POS 를 받는 동안에만 cmd 를 건드린다 — RW 로 보내는 기존 POSITION 은 그대로 통과.
 *
 *  Created on: Nov 5, 2026
 */

#ifndef INC_RD_ORIGIN_H_
#define INC_RD_ORIGIN_H_

/* Private includes ----------------------------------------------------------*/
#include "rd_common.h"
#include "rd_define.h"
#include "rd_register_ecu.h"
#include "rd_peripheral_ecu.h"

/* Exported constants --------------------------------------------------------*/
#if RD_ORIGIN_TEST
/* 시험: 기준을 Live Expressions 로 바꾼다 (재플래시 없이 S 를 올리려고). 판정값은 기준의 절반.
 * ⚠ 브리지를 띄우기 전에 |현재 fb| 보다 크게 둘 것 — 작으면 띄우자마자 SET_ORIGIN 이 나간다. */
extern volatile int32_t ORIGIN_th_raw;
#define ORIGIN_TH_RAW          ORIGIN_th_raw
#define ORIGIN_JUMP_TH_DEG     ((float)ORIGIN_th_raw * 0.05f)
#else
#define ORIGIN_TH_RAW          30000    /* |fb| ≥ 3000° 에서 원점 트리거 (fb 한계 3200°) */
#define ORIGIN_JUMP_TH_DEG     1000.0f  /* 옛 좌표 / 점프 판정 (S ≥ 3000° 라 충분히 분리됨) */
#endif
#define ORIGIN_CONFIRM_MARGIN  20u      /* [×0.1ms] 원점 TX 후 이 시간 뒤에 받은 fb 만 새 좌표로 인정 */
#define ORIGIN_CONFIRM_TO_MS   50u      /* 이 안에 새 좌표 fb 를 못 보면 원점 실패 */
#if RD_ORIGIN_TEST
#define ORIGIN_SHIFT_OK_RAW    (ORIGIN_th_raw / 2)
#else
#define ORIGIN_SHIFT_OK_RAW    15000    /* 확인 fb 가 |fb| < 1500° 면 원점 적용됨 */
#endif
#define ORIGIN_INST_TO_MS      100u     /* 마지막 INST_POS 후 이 시간 안일 때만 동작 */

/* velocity raw → °/s. residual 진단 전용. raw×10 = ERPM, 출력축 rpm = ERPM/168 (rd_control.c)
 * ⚠ fb 위치가 출력축 기준이라는 가정 — T1 실측 전 잠정값. 제어에는 쓰지 않는다. */
#define AK_VEL_RAW_TO_DEGPS    (10.0f * 6.0f / 168.0f)

/* POSITION 과속 제동 (임시 보호). 실측 60°/s = raw 167, 환산은 위 AK_VEL_RAW_TO_DEGPS.
 * 180°/s(raw 504) 가 20ms(controlTask 4 tick) 이어지면 FRAME_LOST 로 래치하고 제동한다. */
#if RD_ORIGIN_TEST
/* 시험: 원점 직후 스스로 회복하는지 보려고 완화하고 Live Expressions 로 바꾼다.
 * 기본 360°/s, 40ms, 5A (운영값 180°/s, 20ms, 10A). */
extern volatile int32_t  ORIGIN_over_raw;
extern volatile uint8_t  ORIGIN_over_ticks;
extern volatile float    ORIGIN_over_brake_a;
#define POS_OVERSPEED_RAW        ORIGIN_over_raw
#define POS_OVERSPEED_TICKS      ORIGIN_over_ticks
#define POS_OVERSPEED_BRAKE_A    ORIGIN_over_brake_a
#else
#define POS_OVERSPEED_RAW        504
#define POS_OVERSPEED_TICKS      4u
#define POS_OVERSPEED_BRAKE_A    BREAK_CURRENT_HW   /* 10A — 달리는 모터를 세우므로 SW(3A) 보다 강하게 */
#endif

/* Exported types ------------------------------------------------------------*/
typedef struct {
    uint8_t  armed;       /* last_tx 유효 */
    float    last_tx;     /* 직전에 모터로 나간 cmd [deg, 현재 모터 좌표] */
    uint8_t  pending;     /* ORIGIN_PEND: 옛 좌표 cmd 보정 중 */
    uint8_t  confirming;  /* 원점 TX 후 새 좌표 fb 대기 */
    uint8_t  frame_lost;  /* FRAME_LOST 래치 — POSITION 이탈까지 제동 */
    uint8_t  shifted;     /* 이번 POSITION 세션에서 temporary 원점이 적용됨 */
    uint8_t  over_ticks;  /* 과속 연속 tick */
    uint8_t  overspeed;   /* 과속으로 래치됨 — 제동 전류를 고른다 */
    uint8_t  gap;         /* M1: SET_ORIGIN 뒤 POSITION 을 쉬는 중 */
    uint32_t gap_end;     /* M1: 쉬기 끝 시각 [TIM5] */
    float    dbg_in;      /* 시험: 보정 직전 cmd [deg] (Orin 이 보낸 값) */
    float    dbg_out;     /* 시험: 보정 후 모터로 나가는 cmd [deg] */
    int16_t  S_raw;       /* 진행 중 이벤트의 S (×0.1°) */
    int16_t  S_pub;       /* 확정·발행된 S → pre_origin_pose */
    uint8_t  seq;         /* 확정 카운터 → origin_seq */
    uint32_t t_origin;    /* 원점 TX 시각 [TIM5] */
    uint32_t t_fb_old;    /* fb_old 취득 시각 [TIM5] */
    int16_t  vel_old;     /* 원점 시점 속도 raw */
    uint8_t  d_dt;        /* 진단: 원점 TX − fb_old [×0.1ms] */
    int16_t  d_e_track;   /* 진단: 원점 tick 의 cmd − fb_old [×0.1°] */
} ORIGIN_t;

/** @brief 마지막 확정 원점 이벤트 진단 → POS_FB odiag_* */
typedef struct {
    uint8_t motor;        /* 0xFF = 없음 */
    uint8_t dt;
    int16_t e_track;
    int16_t residual;
    int16_t vel;
} ORIGIN_DIAG_t;

/* Exported variables --------------------------------------------------------*/
extern ORIGIN_t      ORIGIN[NUM_AK_MOTORS];
extern ORIGIN_DIAG_t ORIGIN_DIAG;
extern uint32_t      ORIGIN_jump_rej;
extern uint32_t      ORIGIN_overspeed_cnt;
/* 타이밍 시험 방법 — Live Expressions 에서 IDLE 중에 바꾼다.
 * 0 = M0: SET_ORIGIN + 같은 tick POSITION(c−S) / 1 = M1: SET_ORIGIN 후 ORIGIN_gap_ms 동안 POSITION 쉼 */
extern volatile uint8_t  ORIGIN_method;
extern volatile uint16_t ORIGIN_gap_ms;

/* Exported functions prototypes ---------------------------------------------*/
void RD_ORIGIN_INIT(void);

/** @brief INST_POS write 성공 시 호출 (rs485Task). */
void RD_ORIGIN_NOTE_INST(void);

/**
 * @brief  매 tick, CONSUME/LPF 뒤 PERIPHERAL_WRITE 직전 (controlTask).
 *         POSITION + INST_POS 수신 중인 모터의 cmd 를 현재 모터 좌표로 보정한다.
 * @param  cmd        cmd_mtr (in/out)
 * @param  motor_mask reg addr 192 스냅샷
 * @retval 이번 tick 에 SET_ORIGIN 을 보낼 모터 비트필드
 */
uint8_t RD_ORIGIN_UPDATE(CMD_MOTOR_t *cmd, uint8_t motor_mask);

#endif /* INC_RD_ORIGIN_H_ */
