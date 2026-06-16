/*
 * rd_peripheral_ecu.h
 *
 *  PERIPHERAL_t — 드라이버 종합 컨테이너. reg 와 1:1 mapping 되는 sub-struct 를
 *  멤버로 보유 (data_ecd / data_mtr / cmd_mtr) 하여 MARSHAL_PUBLISH/CONSUME 이
 *  단순 memcpy 한 방으로 완성되도록 한다.
 *
 *  Created on: Jan 22, 2026
 *      Author: abc01
 */

#ifndef INC_RD_PERIPHERAL_ECU_H_
#define INC_RD_PERIPHERAL_ECU_H_

/* Private includes ----------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"

#include "rd_register_ecu.h"
#include "rd_peripheral.h"
#include "rd_common.h"
#include "rd_define.h"
/* 드라이버 헤더 ----------------------------------------------------------- */
#include "i2c_as5600.h"
#include "can_ak.h"

/* Exported macro ------------------------------------------------------------*/
#define NUM_ENCODERS 5
#define NUM_AK_MOTORS  4

#define RESET_TIME 		 5000 //[ms]
#define MODE_CHANGE_TIME  100 //[ms]

#define HAL_FATAL_CNT_TH    5   /**< 연속 HAL 에러 누적 임계 → LS_OFFLINE        */

/* Exported variables ---------------------------------------------------------*/
extern AS5600_Handle_t AS5600_Enc[NUM_ENCODERS];
extern CAN_Ak_Handle_t ECU_AK[NUM_AK_MOTORS];

/* Exported types ------------------------------------------------------------*/

/* ── GPIO / motor_on / ESTOP override 등 시스템 단위 cmd flag ─────────────
 *  motor_on / ESTOP_override 는 systemTask 의 ACTION_STATE_* 가 set,
 *  controlTask 의 PERIPHERAL_WRITE 가 read — single-writer 패턴.
 *  GPIO 읽기 결과(MODE/ESTOP/MODE_DONE) 와 같은 구조체에 두어 한 번에 다룸. */
typedef struct {
	/* GPIO 입력 (systemTask 의 RD_PERIPHERAL_READ 가 갱신) */
	uint8_t MODE;          /* 실제 모드 mirror (reg.cmd_system.mode → IND LED 표시용). AUTO=1 / MANUAL=0 */
	uint8_t MODE_DONE;     /* MODE 토글 완료 플래그 */
	uint8_t MODE_TOGGLE;   /* GPIO 스위치 토글 요청 1회성 이벤트 (systemTask 가 reg.cmd_system.mode 반전 후 클리어) */
	uint8_t ESTOP;         /* 물리 ESTOP 스위치 (GPIO) */
	uint32_t MODE_tick;

	/* 상위 명령 플래그 (systemTask ACTION_STATE_* 가 결정) */
	uint8_t motor_on;        /* 1 = CAN TX 활성 / 0 = TX skip          */
	uint8_t ESTOP_override;  /* 1 = ESTOP_HW/SW 모드, controlTask 가 cmd_mtr 무시하고 break current 적용 */
	float   estop_current;   /* ESTOP_override == 1 일 때 controlTask 가 4 모터 모두 이 값으로 BRAKE */
} PERIPHERAL_DATA_t;

/* ── 에러/상태 집계 ────────────────────────────────────────────────────────
 *  ERROR_STATUS_t can / i2c 의 `state` 필드가 채널 worst-of-N STATE_t 역할.
 *    can.state = 4 모터 worst-of-4 (CAN_MOTOR_CHECKER 산출)
 *    i2c.state = 5 enc + MUX worst-of-N (I2C_ENCODER_CHECKER 산출)
 *  isr_err_code / rx_error_cnt / degraded_cnt 는 ERROR_STATUS_t 내부 멤버 사용.
 *  EVALUATE_STATE 가 can.state + i2c.state + UART state → hw_*_bits 비트맵으로 집계. */
typedef struct {

	/* DIAG 발행용 raw 카운터 캐시 (CHECKER 가 ECU_AK[].error / AS5600_Enc[].err_cnt 에서 복사) */
	uint8_t  can_tx_cnt[NUM_AK_MOTORS];
	uint8_t  can_rx_cnt[NUM_AK_MOTORS];
	uint8_t  i2c_rx_cnt[NUM_ENCODERS];
	uint8_t  mux_rx_cnt;

	/* 통신 단위 종합 — state(lifecycle+health) + ISR HAL 에러 캡처 + decay 카운터 */
	ERROR_STATUS_t can;   /* CAN 4 모터 종합 (RD_CAN_MOTOR_CHECKER 가 갱신)        */
	ERROR_STATUS_t i2c;   /* I2C 5 enc + MUX 종합 (RD_I2C_ENCODER_CHECKER 가 갱신) */
} PERIPHERAL_ERROR_t;

/* ── PERIPHERAL 종합 컨테이너 ──────────────────────────────────────────────
 *  data_ecd / data_mtr / cmd_mtr 는 reg.encoder / reg.motor_data / reg.cmd_motor
 *  와 1:1 mapping. MARSHAL_PUBLISH 는 data_* → reg 단순 memcpy,
 *  MARSHAL_CONSUME 는 reg.cmd_motor → cmd_mtr 단순 memcpy 로 완성.
 *  드라이버 UPDATE 함수가 data_* 에 직접 쓰고, TRANSMIT 함수가 cmd_mtr 에서 직접 읽는다. */
typedef struct {
	CAN_HandleTypeDef *hcan;
	I2C_HandleTypeDef *hi2c;
	GPIO_t gpio;

	volatile DATA_ENCODER_t    data_ecd;   /* reg.encoder    mirror — RD_I2C_ENCODER_UPDATE 가 쓴다 */
	volatile DATA_MOTOR_t      data_mtr;   /* reg.motor_data mirror — RD_CAN_MOTOR_UPDATE 가 쓴다  */
	volatile CMD_MOTOR_t       cmd_mtr;    /* reg.cmd_motor  mirror — MARSHAL_CONSUME 가 쓴다       */
	volatile PERIPHERAL_DATA_t data;       /* GPIO + motor_on + ESTOP override (systemTask 가 갱신) */

	volatile PERIPHERAL_ERROR_t err;
} PERIPHERAL_t;

/* Exported functions prototypes ---------------------------------------------*/

/** 상위가 호출. CHECKER 위임 결과 (motor_state) 기반 RET_NOK/WAIT/OK. */
RD_RET RD_PERIPHERAL_CHECKER(PERIPHERAL_t* peripheral_obj, const DEFINE_t *thresh);

RD_RET RD_PERIPHERAL_INIT(PERIPHERAL_t* peripheral_obj, CAN_HandleTypeDef* hcan, I2C_HandleTypeDef* hi2c);

RD_RET RD_PERIPHERAL_WRITE(PERIPHERAL_t* peripheral_obj);  /* GPIO + CAN_MOTOR_TRANSMIT (controlTask) */
RD_RET RD_PERIPHERAL_READ(PERIPHERAL_t* peripheral_obj);   /* GPIO READ + CAN_MOTOR_UPDATE (systemTask) */
RD_RET RD_PERIPHERAL_I2C(PERIPHERAL_t* peripheral_obj); /* I2C_ENCODER_UPDATE (i2cTask)             */
#endif /* INC_RD_PERIPHERAL_ECU_H_ */
