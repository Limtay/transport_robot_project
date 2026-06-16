/*
 * rd_peripheral_ecu.c
 *
 *  GPIO 전담 + drivers wrap. CAN AK → rd_can_motor / I2C → rd_i2c_encoder.
 *
 *  데이터 흐름:
 *    READ   : GPIO_READ → data.MODE/ESTOP + CAN_MOTOR_UPDATE → data_mtr (systemTask)
 *    UPDATE : I2C_ENCODER_UPDATE → data_ecd                  (i2cTask)
 *    WRITE  : GPIO_WRITE + ESTOP override 또는 cmd_mtr 그대로 CAN_MOTOR_TRANSMIT (controlTask)
 *    CHECKER: CAN_MOTOR_CHECKER → err.can.state             (systemTask)
 *    EVALUATE_STATE: I2C_ENCODER_CHECKER → err.i2c.state    (i2cTask)
 *
 *  Created on: Jan 22, 2026
 *      Author: abc01
 */

#include "rd_peripheral_ecu.h"

#include "rd_can_motor.h"
#include "rd_i2c_encoder.h"

#include <string.h>

/* Private prototypes --------------------------------------------------------*/
static RD_RET RD_GPIO_INIT(GPIO_t* GPIO);
static RD_RET RD_GPIO_WRITE(GPIO_t* GPIO, volatile PERIPHERAL_DATA_t *data);
static RD_RET RD_GPIO_READ(GPIO_t* GPIO);

/* Private helper functions --------------------------------------------------*/
static RD_RET RD_GPIO_INIT(GPIO_t* GPIO) {
	if (GPIO == NULL) return RET_NOK;

	GPIO->IO.IND_IO.per_GPIO_Pin   = INDICATOR1_Pin;
	GPIO->IO.IND_IO.per_GPIOx      = INDICATOR1_GPIO_Port;
	GPIO->IO.MODE_IO.per_GPIO_Pin  = MODE1_Pin;
	GPIO->IO.MODE_IO.per_GPIOx     = MODE1_GPIO_Port;
	GPIO->IO.ESTOP_IO.per_GPIO_Pin = MODE2_Pin;
	GPIO->IO.ESTOP_IO.per_GPIOx    = MODE2_GPIO_Port;

	GPIO->IND   = 0;
	GPIO->MODE  = 0;
	GPIO->ESTOP = 0;
	return RET_OK;
}

static RD_RET RD_GPIO_WRITE(GPIO_t* GPIO, volatile PERIPHERAL_DATA_t *data) {
	if (GPIO == NULL) return RET_NOK;
	if (data->MODE == 1) {
		GPIO->IND_cnt = 0;
		HAL_GPIO_WritePin(GPIO->IO.IND_IO.per_GPIOx, GPIO->IO.IND_IO.per_GPIO_Pin, SET);
	} else {
		GPIO->IND_cnt++;
		if (GPIO->IND_cnt > 20) {
			GPIO->IND_cnt = 0;
			HAL_GPIO_TogglePin(GPIO->IO.IND_IO.per_GPIOx, GPIO->IO.IND_IO.per_GPIO_Pin);
		}
	}
	return RET_OK;
}

static RD_RET RD_GPIO_READ(GPIO_t* GPIO) {
	if (GPIO == NULL) return RET_NOK;
	GPIO->MODE  = HAL_GPIO_ReadPin(GPIO->IO.MODE_IO.per_GPIOx,  GPIO->IO.MODE_IO.per_GPIO_Pin);
	GPIO->ESTOP = HAL_GPIO_ReadPin(GPIO->IO.ESTOP_IO.per_GPIOx, GPIO->IO.ESTOP_IO.per_GPIO_Pin);
	return RET_OK;
}

/* Public functions ----------------------------------------------------------*/

RD_RET RD_PERIPHERAL_INIT(PERIPHERAL_t* peripheral_obj, CAN_HandleTypeDef* hcan, I2C_HandleTypeDef* hi2c) {
    if (peripheral_obj == NULL) return RET_NOK;
    memset(peripheral_obj, 0, sizeof(PERIPHERAL_t));

    peripheral_obj->hcan = hcan;
    peripheral_obj->hi2c = hi2c;
    peripheral_obj->data.MODE_tick = HAL_GetTick();

    /* CHECKER 가 LS_READY → LS_RUNNING 승격 가능하도록 명시 세팅 (memset 0 = LS_INIT) */
    peripheral_obj->err.can.state.bits.lifecycle = LS_READY;
    peripheral_obj->err.i2c.state.bits.lifecycle = LS_READY;

    if (RD_GPIO_INIT(&peripheral_obj->gpio)       != RET_OK) return RET_NOK;
    if (RD_I2C_ENCODER_INIT(peripheral_obj->hi2c) != RET_OK) return RET_NOK;
    if (RD_CAN_MOTOR_INIT(peripheral_obj->hcan, &peripheral_obj->err) != RET_OK) return RET_NOK;
    return RET_OK;
}


/**
 * controlTask 가 호출. GPIO 출력 + motor_on 체크 후 CAN TX.
 *  - ESTOP_override == 1 : cmd_mtr 무시, 4 모터 모두 BRAKE 명령으로 덮어써 전송.
 *  - 그 외                : cmd_mtr 그대로 전송.
 */
RD_RET RD_PERIPHERAL_WRITE(PERIPHERAL_t* peripheral_obj) {
	if (peripheral_obj == NULL) return RET_NOK;
	if (RD_GPIO_WRITE(&peripheral_obj->gpio, &peripheral_obj->data) != RET_OK) return RET_NOK;

	if (peripheral_obj->data.motor_on != 1) return RET_OK;  /* TX skip */

	if (peripheral_obj->data.ESTOP_override) {
		CMD_MOTOR_t estop;
		memset(&estop, 0, sizeof(estop));
		for (int i = 0; i < NUM_AK_MOTORS; i++) {
			estop.ctr_mode[i]    = MODE_CURRENT_BRAKE;
			estop.cmd_current[i] = peripheral_obj->data.estop_current;
		}
		return RD_CAN_MOTOR_TRANSMIT(&estop);
	}
	return RD_CAN_MOTOR_TRANSMIT(&peripheral_obj->cmd_mtr);
}

/**
 * systemTask 가 호출. GPIO READ + MODE 토글 처리 + CAN 모터 raw 데이터 갱신.
 */
RD_RET RD_PERIPHERAL_READ(PERIPHERAL_t* peripheral_obj) {
	if (peripheral_obj == NULL) return RET_NOK;
	if (RD_GPIO_READ(&peripheral_obj->gpio) != RET_OK) return RET_NOK;

	peripheral_obj->data.ESTOP = peripheral_obj->gpio.ESTOP;

	if (peripheral_obj->gpio.MODE == 1) {
		peripheral_obj->data.MODE_tick = HAL_GetTick();
		peripheral_obj->data.MODE_DONE = 0;
	} else {
		uint32_t mode_time = HAL_GetTick() - peripheral_obj->data.MODE_tick;
		/* >5s 홀드 → reboot 요청. 직접 NVIC_SystemReset() 대신 RET_WAIT 로 상위에 위임
		 * (systemTask 가 CAN TX abort 후 안전 리셋하는 RD_REBOOT_HANDLE() 호출 — H-2 대응). */
		if (mode_time > RESET_TIME) return RET_WAIT;
		else if (mode_time > MODE_CHANGE_TIME && peripheral_obj->data.MODE_DONE == 0) {
			/* 모드 소유는 reg.cmd_system.mode 로 이전 — 여기선 토글 "요청" 1회만 발행.
			 * 실제 반전은 systemTask(RD_SYSTEM_UPDATE_STATE)가 수행. */
			peripheral_obj->data.MODE_TOGGLE = 1;
			peripheral_obj->data.MODE_DONE   = 1;
		}
	}
	RD_CAN_MOTOR_UPDATE(&peripheral_obj->data_mtr);
	return RET_OK;
}

/** i2cTask 가 호출. AS5600 × 5 UPDATE → data_ecd 갱신. */
/** i2cTask 가 호출. encoder CHECKER → err.i2c.state + data_ecd.state 갱신. */
RD_RET RD_PERIPHERAL_I2C(PERIPHERAL_t* peripheral_obj) {
	if (peripheral_obj == NULL) return RET_NOK;
	RD_I2C_ENCODER_UPDATE(&peripheral_obj->data_ecd, &peripheral_obj->err);
	return RD_I2C_ENCODER_CHECKER(&peripheral_obj->data_ecd, &peripheral_obj->err);
}

