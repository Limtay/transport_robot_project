/*
 * rd_control.h
 *
 *  Created on: Feb 11, 2026
 *      Author: abc01
 */

#ifndef INC_RD_CONTROL_H_
#define INC_RD_CONTROL_H_

/* Private includes ----------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
#include "rd_common.h"
#include "rd_define.h"

#include "rd_peripheral_dpcb.h"

/* Exported macro ------------------------------------------------------------*/
#define ERROR_STATE		10

#define TIMEOUT_1		500		// almost 5s		INIT STATE
#define TIMEOUT_2		250			// almost 5s		DESCEND 1 STATE
#define TIMEOUT_3		1000		// almost 10s		DESCEND 1 STATE
#define TIMEOUT_4		500			// almost 5s		WAIT STATE
#define TIMEOUT_5		500			// almost 5s		ASCEND 1 STATE
#define TIMEOUT_6		1000		// almost 10s		ASCEND 2 STATE
#define TIMEOUT_7		500			// almost 5s		FINISH STATE

#define MAX_POS			-16384		// 500mm, 4 rotate

#define TIMEOUT_SOL		1000		// Solenoid maximum enable time

#define CLP_M			1000
#define CURR_LIM		100			// INIT limit current 270mA 2.69mA per unit
#define SPEED_LIM		10			// INIT limit speed

#define ASC_CURR_LIM	1000		//2.7A

/* Exported types ------------------------------------------------------------*/
typedef struct {
	//volatile float MOT_INIT_POS;	//Initial position for offset
	volatile float MOT_SPEED_CTL;	// actual control (float type)
	float MOT_P_gain;				// P gain
	float MOT_I_gain;				// I gain
	volatile float MOT_I_temp;		// integrate buffer
	float MOT_I_max;				// integrate max
} PID_t;


typedef struct {
	volatile uint8_t STATE;			// 0 is manual and idle, 1 is AUTO INIT, 2 is AUTO DOWN, 3 is AUTO WAIT, 4 is AUTO UP, return 0
	volatile uint32_t LOOP_CNT;		// counter of loop for timeout

	PID_t MOT_SPEED_PID;
	PID_t MOT_PITCH_PID;
	PID_t MOT_ROLL_PID;
} CONTROL_DPC_t;

/* Exported constants --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
RD_RET RD_CONTROL_INIT(CONTROL_DPC_t *CTL);
RD_RET RD_CONTROL_LOOP(CONTROL_DPC_t *CTL, PERIPHERAL_t* GPIO);

#endif /* INC_RD_CONTROL_H_ */
