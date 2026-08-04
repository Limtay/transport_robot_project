/*
 * rd_control.c
 *
 *  Created on: Feb 11, 2026
 *      Author: abc01
 */


/* Includes ------------------------------------------------------------------*/
#include "rd_control.h"
#include <string.h>


/* Exported includes ----------------------------------------------------------*/

/* Exported typedef -----------------------------------------------------------*/

/* Exported define ------------------------------------------------------------*/
#define CLAMP(val, min, max) ({           \
    __typeof__ (val) _val = (val);        \
    __typeof__ (min) _min = (min);        \
    __typeof__ (max) _max = (max);        \
    _val < _min ? _min : (_val > _max ? _max : _val); \
})

#define ABS(x) ((x) < 0 ? -(x) : (x))

/* Exported variables ---------------------------------------------------------*/
//int16_t sum = 0;
/* Exported function prototypes -----------------------------------------------*/
RD_RET RD_CONTROL_INIT(CONTROL_DPC_t *CTL);
RD_RET RD_CONTROL_LOOP(CONTROL_DPC_t *CTL, PERIPHERAL_t* GPIO);

//mode = 0
RD_RET RD_CONTROL_CASE_IDLE			(CONTROL_DPC_t *CTL, PERIPHERAL_t* GPIO);

//mode = 1
RD_RET RD_CONTROL_CASE_CTRL			(CONTROL_DPC_t *CTL, PERIPHERAL_t* GPIO);		//CASE 0
RD_RET RD_CONTROL_CASE_HOLD			(CONTROL_DPC_t *CTL, PERIPHERAL_t* GPIO);		//CASE 1

RD_RET RD_CONTROL_CASE_INIT			(CONTROL_DPC_t *CTL, PERIPHERAL_t* GPIO);		//CASE 2
RD_RET RD_CONTROL_CASE_DESCEND_1	(CONTROL_DPC_t *CTL, PERIPHERAL_t* GPIO);		//CASE 3
RD_RET RD_CONTROL_CASE_DESCEND_2	(CONTROL_DPC_t *CTL, PERIPHERAL_t* GPIO);		//CASE 4
RD_RET RD_CONTROL_CASE_WAIT			(CONTROL_DPC_t *CTL, PERIPHERAL_t* GPIO);		//CASE 5
RD_RET RD_CONTROL_CASE_ASCEND_1		(CONTROL_DPC_t *CTL, PERIPHERAL_t* GPIO);		//CASE 6
RD_RET RD_CONTROL_CASE_ASCEND_2		(CONTROL_DPC_t *CTL, PERIPHERAL_t* GPIO);		//CASE 7
RD_RET RD_CONTROL_CASE_FINISH		(CONTROL_DPC_t *CTL, PERIPHERAL_t* GPIO);		//CASE 8

RD_RET RD_CONTROL_CASE_ERROR		(CONTROL_DPC_t *CTL, PERIPHERAL_t* GPIO);		//ERROR CASE 10

//RD_RET RD_CONTROL_CTL_BALACE		(float TAR_VEL, CONTROL_DPC_t *CTL, PERIPHERAL_t* GPIO);		//Winch balancing control

int32_t MATH_Clamp(int32_t val, int32_t min, int32_t max);
/* Private user code ---------------------------------------------------------*/

RD_RET RD_CONTROL_INIT(CONTROL_DPC_t *CTL){
	CTL->STATE							= 0;			// Main state, 0 is manual ctl
	CTL->MODE							= 0;			//
	/*
	//CTL->MOT_SPEED_PID.MOT_INIT_POS 	= 0.0f; 		// actually, have to set
	CTL->MOT_SPEED_PID.MOT_SPEED_CTL 	= 0.0f;
	CTL->MOT_SPEED_PID.MOT_P_gain 		= 2.0f;
	CTL->MOT_SPEED_PID.MOT_I_gain 		= 0.0f;
	CTL->MOT_SPEED_PID.MOT_I_temp 		= 0.0f;
	CTL->MOT_SPEED_PID.MOT_I_max 		= 500.0f;		// gain init speed A

	//CTL->MOT_SPEED_PID.MOT_INIT_POS 	= 0.0f;
	CTL->MOT_PITCH_PID.MOT_SPEED_CTL 	= 0.0f;
	CTL->MOT_PITCH_PID.MOT_P_gain 		= 2.0f;
	CTL->MOT_PITCH_PID.MOT_I_gain 		= 0.0f;
	CTL->MOT_PITCH_PID.MOT_I_temp 		= 0.0f;
	CTL->MOT_PITCH_PID.MOT_I_max 		= 500.0f;		// gain init pitch pid

	//CTL->MOT_SPEED_PID.MOT_INIT_POS 	= 0.0f;
	CTL->MOT_ROLL_PID.MOT_SPEED_CTL 	= 0.0f;
	CTL->MOT_ROLL_PID.MOT_P_gain 		= 2.0f;
	CTL->MOT_ROLL_PID.MOT_I_gain 		= 0.0f;
	CTL->MOT_ROLL_PID.MOT_I_temp 		= 0.0f;
	CTL->MOT_ROLL_PID.MOT_I_max 		= 500.0f;		// gain init roll pid
	*/
	return RET_OK;
}

RD_RET RD_CONTROL_LOOP(CONTROL_DPC_t *CTL, PERIPHERAL_t* GPIO){
	CTL->LOOP_CNT++;
	/*
	for(int i=0; i < DYN_NUM_MOTORS; i++){
		if(GPIO->MOT[i].dyn_ctrl.error.status_error != 0){
			CTL->STATE = ERROR_STATE;
			break;
		}
	}
	*/

	if (CTL->MODE == 0){
		/*____________________  MANUAL 	____________________*/
		// SW1이 모드변경. 짧게누르면(SW_LOW~SW_HIGH) 모스스위칭, 길게 누르면(MODE_CNT>SW_HIGH) 시스템 리부팅(현재 리부팅 미구현)
		// SW2를 A,B locker제어기로 사용.
		RD_CONTROL_CASE_IDLE(CTL, GPIO);

		if (GPIO->PANEL.SW1_state == 0){ 						//뗐을때
			uint32_t delta_tick = HAL_GetTick() - CTL->MODE_SW_LAST;
			if (delta_tick > SW_LOW && delta_tick < SW_HIGH){
				CTL->MODE = 1;
				CTL->STATE = DPCB_STATE_CTRL;
				CTL->FSM_SW_LAST = HAL_GetTick();
			}
			//CTL->MODE_SW_LAST = HAL_GetTick(); 									//갱신
		}
	}

	else if (CTL->MODE == 1){
		/*____________________  AUTO 	____________________*/
		if (GPIO->PANEL.SW1_state == 0){ //뗐을때
			uint32_t delta_tick = HAL_GetTick() - CTL->MODE_SW_LAST;
			if (delta_tick > SW_LOW && delta_tick < SW_HIGH){
				CTL->MODE = 0;
			}
			//CTL->MODE_SW_LAST = HAL_GetTick(); //갱신
		}

		switch (CTL->STATE) {													//ACTION SWITCH
				/*____________________  CTRL 	____________________*/
				// : SW2를 천이용 제어로 사용.
				case 0: {		//CTRL
					if (GPIO->PANEL.SW2_state == 0){ // SW2 뗐을때
						uint32_t delta_tick = HAL_GetTick() - CTL->FSM_SW_LAST;
						if (delta_tick > SW_LOW && delta_tick < SW_HIGH){
							CTL->STATE = DPCB_STATE_HOLD;						//HOLD 모드로 전이
						}else if (delta_tick > SW_HIGH){
							CTL->STATE = DPCB_STATE_INIT;						//전개로 전이
						}

						//CTL->FSM_SW_LAST = HAL_GetTick(); 						//갱신
					}

					//TODO : CTRL 구현

					CTL->LOOP_CNT = 0;											// 루프카운트 초기화

					break;
				}
				case 1:{		//HOLD
					if (GPIO->PANEL.SW2_state == 0){ // SW2 뗐을때
						uint32_t delta_tick = HAL_GetTick() - CTL->FSM_SW_LAST;
						if (delta_tick > SW_LOW && delta_tick < SW_HIGH){
							CTL->STATE = DPCB_STATE_CTRL;						//CTRL로 전이
						}
						/*else if (delta_tick > SW_HIGH){
							CTL->STATE = DPCB_STATE_INIT;						//전개로 전이
						}*/
						//CTL->FSM_SW_LAST = HAL_GetTick(); //갱신
					}

					//TODO : HOLD 구현

					CTL->LOOP_CNT = 0;											// 루프카운트 초기화
					break;
				}
				/*____________________  AUTO 	____________________*/
				case 2:{		//INIT
					RD_CONTROL_CASE_INIT(CTL, GPIO);


					// NORMAL transition
					int32_t sum = 0;
					for (int i=0; i < DYN_NUM_MOTORS; i++){
						sum += ABS(GPIO->MOT[i].dyn_ctrl.ram.state.present_velocity);
					}

					if (sum < SPEED_LIM && CTL->LOOP_CNT > 100){
						CTL->STATE = DPCB_STATE_DESCEND_1; 						// 속도합이 5 이하로 떨어질경우 천이
						CTL->LOOP_CNT = 0;										// 루프카운트 초기화
					}

					// ERROR transition
					if (CTL->LOOP_CNT > TIMEOUT_1){
						CTL->STATE = DPCB_STATE_ERROR;
					}
					break;
				}
				case 3:{		//DESCEND 1
					RD_CONTROL_CASE_DESCEND_1(CTL, GPIO);

					// NORMAL transition
					if (GPIO->CON_DATA == CONT_ALL_UNLOCK){
						CTL->STATE = DPCB_STATE_DESCEND_2;						//0b 0000xxxx일때 천이. 걸렸을때 1임. 즉 싹다 풀렸을때만 천이하는거임
						CTL->LOOP_CNT = 0;										// 루프카운트 초기화
					}
					// ERROR transition
					if (CTL->LOOP_CNT > TIMEOUT_2){ 							//2.5sec
						CTL->STATE = DPCB_STATE_ERROR;
					}
					break;
				}
				case 4:{		//DESCEND 2
					RD_CONTROL_CASE_DESCEND_2(CTL, GPIO);

					// NORMAL transition
					int32_t avr_pos = 0;
					for (int i=0; i < DYN_NUM_MOTORS; i++){
						avr_pos += GPIO->MOT[i].dyn_ctrl.ram.state.present_position - GPIO->MOT[i].INIT_POS;
					}
					avr_pos = ABS(avr_pos/3);
					if (GPIO->A_PROX_DATA == A_PROX_ALL_ON && CTL->LOOP_CNT > 100){
						CTL->STATE = DPCB_STATE_WAIT;
						CTL->FSM_SW_LAST = HAL_GetTick(); //갱신
						CTL->LOOP_CNT = 0;										// 루프카운트 초기화
					}
					// ERROR transition
					if (CTL->LOOP_CNT > TIMEOUT_3 || avr_pos > MAX_POS){ 		//max 500mm descend
						CTL->STATE = DPCB_STATE_ERROR;
					}

					break;
				}
				case 5:{		//WAIT
					RD_CONTROL_CASE_WAIT(CTL, GPIO);
					CTL->LOOP_CNT = 0;

					if (GPIO->PANEL.SW2_state == 0){ // SW2 뗐을때
						uint32_t delta_tick = HAL_GetTick() - CTL->FSM_SW_LAST;
						if (delta_tick > SW_HIGH){									// 5초이상 꾹누르면 전이
							CTL->STATE = DPCB_STATE_ASCEND_1;						// 기본모드로 전이
						}
						/*
						if (delta_tick > SW_LOW && delta_tick < SW_HIGH){
							CTL->STATE = DPCB_STATE_ASCEND_1;					// ASCEND로 전이
						}else if (delta_tick > SW_HIGH){						// 5초이상
							CTL->STATE = DPCB_STATE_CTRL;						// 기본모드로 전이
						}
						*/
						//CTL->FSM_SW_LAST = HAL_GetTick(); //갱신
					}

					//if (GPIO->PANEL.SW1_state == 1) CTL->STATE++;
					break;
				}
				/*____________________ 	ASCEND 	____________________*/
				case 6:{		//ASCEND 1
					RD_CONTROL_CASE_ASCEND_1(CTL, GPIO);

					// NORMAL transition
					if (GPIO->A_PROX_DATA == A_PROX_ALL_OFF){					//바닥에서 다 떨어졌는지 확인
						CTL->STATE = DPCB_STATE_ASCEND_2;						// 근접센서 다 떨어졌을때 모드전환
						CTL->LOOP_CNT = 0;										// 루프카운트 초기화
					}
					// ERROR transition
					if (CTL->LOOP_CNT > TIMEOUT_5){
						CTL->STATE = DPCB_STATE_ERROR;
					}

					break;
				}
				case 7:{		//ASCEND 2
					RD_CONTROL_CASE_ASCEND_2(CTL, GPIO);

					int16_t max_current = 0;
					for (int i=0; i < DYN_NUM_MOTORS; i++){
						int16_t temp_curr = ABS(GPIO->MOT[i].LPF_CURRENT);
						if (temp_curr > max_current){
							max_current = temp_curr; 		//refresh max current
						}
						//TODO : current 오버시 에러 확인.
					}

					// NORMAL transition
					if (GPIO->CON_DATA == CONT_ALL_LOCK){			//라커 다 걸렸을때
						CTL->STATE = DPCB_STATE_FINISH;
						CTL->LOOP_CNT = 0;					// 루프카운트 초기화
					}
					/*
					uint8_t over_pos_flag = 0;
					for (int i=0; i < DYN_NUM_MOTORS; i++){
						if (GPIO->MOT[i].dyn_ctrl.ram.state.present_position < GPIO->MOT[i].INIT_POS;
					}
					*/
					// ERROR transition
					if (CTL->LOOP_CNT > TIMEOUT_6){ //상승 위치 제한 필요함  || max_current > ASC_CURR_LIM
						CTL->STATE = DPCB_STATE_ERROR;
					}
					break;
				}
				case 8:{		//FINISH
					RD_CONTROL_CASE_FINISH(CTL, GPIO);

					//normal transition
					if (GPIO->PANEL.SW2_state == 0){ // SW2 뗐을때
						uint32_t delta_tick = HAL_GetTick() - CTL->FSM_SW_LAST;
						if (delta_tick > SW_HIGH){									// 5초이상 꾹누르면 전이
							CTL->STATE = DPCB_STATE_CTRL;							// 기본모드로 전이
						}
					}
					//CTL->STATE = DPCB_STATE_CTRL;							// back to idle mode
					//CTL->FSM_SW_LAST = HAL_GetTick(); //갱신
					break;
				}
					/*____________________ 	ERROR 	____________________*/
				default:{
					//RD_CONTROL_CASE_ERROR(CTL, GPIO);

					//normal transition

					// ERROR transition

					break;
				}
	}



	}


	if (GPIO->PANEL.SW1_state == 0){
		CTL->MODE_SW_LAST = HAL_GetTick();
	}

	if (GPIO->PANEL.SW2_state == 0){
		CTL->FSM_SW_LAST = HAL_GetTick();
	}

	return RET_OK;
}

/*============================== LOOP CASE ACTION ==============================*/
RD_RET RD_CONTROL_CASE_CTRL	(CONTROL_DPC_t *CTL, PERIPHERAL_t* GPIO){
	//TODO : CTRL 구현. orin이 수동제어
	return RET_OK;
}


RD_RET RD_CONTROL_CASE_HOLD	(CONTROL_DPC_t *CTL, PERIPHERAL_t* GPIO){
	//TODO : HOLD 구현.
	return RET_OK;
}

RD_RET RD_CONTROL_CASE_IDLE(CONTROL_DPC_t *CTL, PERIPHERAL_t* GPIO){
	/*=======================================
	* SW2 (SPST+LED)	: LED btn, 1 to 1
	* SW6 (DPST)		: servo locker, 0:IDLE  1:LOCK  2:UNLOCK
	* SW5 (DPST)		: winch motor,  0:IDLE  1:ASCE  2:DESC
	* SW4 (DPST)		: winch motor,  0:IDLE  1:ASCE  2:DESC
	* SW3 (DPST)		: winch motor,  0:IDLE  1:ASCE  2:DESC
	*/

	if (GPIO->PANEL.SW2_state == 1){ //A,B locker EN. Need Timeout
		if(HAL_GetTick() - GPIO->LAST_EN_TICK < TIMEOUT_SOL){
			GPIO->EN_ALL = 1;
			GPIO->A_EN_ALL = 1; //add
			//GPIO->PANEL.LED2_state = 1;
		}else{
			GPIO->EN_ALL = 0;
			GPIO->A_EN_ALL = 0; //add
			//GPIO->PANEL.LED2_state = 0;
		}
	}else{
		GPIO->EN_ALL = 0;
		GPIO->A_EN_ALL = 0; //add
		//GPIO->PANEL.LED2_state = 0;
		GPIO->LAST_EN_TICK = HAL_GetTick();
	}
	switch (GPIO->PANEL.SW6_state){
	case 0:
		GPIO->SERVO_EN = 0;
		break;
	case 1:									//lock
		GPIO->SERVO_EN = 1;
		break;
	case 2:									//unlock
		GPIO->SERVO_EN = 2;
		break;
	default:
		GPIO->SERVO_EN = 0;
		break;
	}




	switch (GPIO->PANEL.SW5_state){ //motor A switch
	case 0:
		GPIO->MOT[0].TARGET_POS = GPIO->MOT[0].dyn_ctrl.ram.state.present_position; //refresh current pos
		RD_MOT_FORCE_DRIVE(&GPIO->MOT[0], 0);
		break;
	case 1:									//Ascending
		RD_MOT_FORCE_DRIVE(&GPIO->MOT[0], +50);
		break;
	case 2:									//Descending
		RD_MOT_FORCE_DRIVE(&GPIO->MOT[0], -50);
		break;
	default:
		GPIO->MOT[0].TARGET_POS = GPIO->MOT[0].dyn_ctrl.ram.state.present_position; //refresh current pos
		RD_MOT_FORCE_DRIVE(&GPIO->MOT[0], 0);
		break;
	}

	switch (GPIO->PANEL.SW4_state){ //motor B switch
	case 0:
		GPIO->MOT[1].TARGET_POS = GPIO->MOT[1].dyn_ctrl.ram.state.present_position;
		RD_MOT_FORCE_DRIVE(&GPIO->MOT[1], 0);
		break;
	case 1:
		RD_MOT_FORCE_DRIVE(&GPIO->MOT[1], +50);
		break;
	case 2:
		RD_MOT_FORCE_DRIVE(&GPIO->MOT[1], -50);
		break;
	default:
		GPIO->MOT[1].TARGET_POS = GPIO->MOT[1].dyn_ctrl.ram.state.present_position;
		RD_MOT_FORCE_DRIVE(&GPIO->MOT[1], 0);
		break;
	}

	switch (GPIO->PANEL.SW3_state){ //motor C switch
	case 0:
		GPIO->MOT[2].TARGET_POS = GPIO->MOT[2].dyn_ctrl.ram.state.present_position;
		RD_MOT_FORCE_DRIVE(&GPIO->MOT[2], 0);
		break;
	case 1:
		RD_MOT_FORCE_DRIVE(&GPIO->MOT[2], +50);
		break;
	case 2:
		RD_MOT_FORCE_DRIVE(&GPIO->MOT[2], -50);
		break;
	default:
		GPIO->MOT[2].TARGET_POS = GPIO->MOT[2].dyn_ctrl.ram.state.present_position;
		RD_MOT_FORCE_DRIVE(&GPIO->MOT[2], 0);
		break;
	}

	return RET_OK;
}


/*============================== CASE ACTION LIST ==============================*/
// INIT		: CASE-1
RD_RET RD_CONTROL_CASE_INIT(CONTROL_DPC_t *CTL, PERIPHERAL_t* GPIO){
	// INIT 				: winch_ctl +50, refresh init pos

	for (int i=0; i < DYN_NUM_MOTORS; i++){
		//전류 쓸거면 래치로 고쳐야됨
		//if(ABS(GPIO->MOT[i].dyn_ctrl.ram.state.present_current) < CURR_LIM && CTL->LOOP_CNT){
		if(ABS(GPIO->MOT[i].dyn_ctrl.ram.state.present_velocity) < SPEED_LIM && CTL->LOOP_CNT > 100){
			RD_MOT_DRIVE(&GPIO->MOT[i], 0); //stop.
		}else{
			RD_MOT_DRIVE(&GPIO->MOT[i], -50);
			GPIO->MOT[i].INIT_POS = GPIO->MOT[i].dyn_ctrl.ram.state.present_position;
			GPIO->MOT[i].TARGET_POS = GPIO->MOT[i].INIT_POS;
		}
	}

	/*
	RD_MOT_DRIVE(&GPIO->MOT[0], -50);
	RD_MOT_DRIVE(&GPIO->MOT[1], -50);
	RD_MOT_DRIVE(&GPIO->MOT[2], -50);

	//refresh INIT pos
	GPIO->MOT[0].INIT_POS = GPIO->MOT[1].dyn_ctrl.ram.state.present_position;
	GPIO->MOT[1].INIT_POS = GPIO->MOT[1].dyn_ctrl.ram.state.present_position;
	GPIO->MOT[2].INIT_POS = GPIO->MOT[2].dyn_ctrl.ram.state.present_position;
	GPIO->MOT[0].TARGET_POS = GPIO->MOT[1].INIT_POS;
	GPIO->MOT[1].TARGET_POS = GPIO->MOT[1].INIT_POS;
	GPIO->MOT[2].TARGET_POS = GPIO->MOT[2].INIT_POS;
	*/

	GPIO->A_EN_ALL = 0;				//lockA off
	GPIO->EN_ALL = 0;				//lockB off
	return RET_OK;
}

// DECEND_1	: CASE-2
RD_RET RD_CONTROL_CASE_DESCEND_1(CONTROL_DPC_t *CTL, PERIPHERAL_t* GPIO){
	// DECEND_1				: winch_ctl -200, lockB on

	RD_MOT_FORCE_DRIVE(&GPIO->MOT[0], +20); // slow desc
	RD_MOT_FORCE_DRIVE(&GPIO->MOT[1], +20);
	RD_MOT_FORCE_DRIVE(&GPIO->MOT[2], +20);

	GPIO->A_EN_ALL = 0;				//lockA off
	GPIO->EN_ALL = 1;				//lockB on
	return RET_OK;
}

// DECEND_2	: CASE-3
RD_RET RD_CONTROL_CASE_DESCEND_2(CONTROL_DPC_t *CTL, PERIPHERAL_t* GPIO){
	// DECEND_2				: winch_ctl -500, lockB off
	RD_MOT_FORCE_DRIVE(&GPIO->MOT[0], +50);
	RD_MOT_FORCE_DRIVE(&GPIO->MOT[1], +50);
	RD_MOT_FORCE_DRIVE(&GPIO->MOT[2], +50);

	GPIO->A_EN_ALL = 0;				//lockA off
	GPIO->EN_ALL = 0;				//re_lock (current issue)
	return RET_OK;
}

// WAIT	   : CASE-4
RD_RET RD_CONTROL_CASE_WAIT(CONTROL_DPC_t *CTL, PERIPHERAL_t* GPIO){
	// WAIT					: winch_ctl 0(brk), lockA on
	RD_MOT_FORCE_DRIVE(&GPIO->MOT[0], 0);
	RD_MOT_FORCE_DRIVE(&GPIO->MOT[1], 0);
	RD_MOT_FORCE_DRIVE(&GPIO->MOT[2], 0);

	GPIO->A_EN_ALL = 1;				//lockA on
	GPIO->EN_ALL = 0;				//lockB off
	GPIO->LIGHT_EN = 1;				//Turn on Light
	return RET_OK;
}

// ASCEND_1	: CASE-5
RD_RET RD_CONTROL_CASE_ASCEND_1(CONTROL_DPC_t *CTL, PERIPHERAL_t* GPIO){
	// ASCEND_1				: winch_ctl 200
	RD_MOT_FORCE_DRIVE(&GPIO->MOT[0], -20);
	RD_MOT_FORCE_DRIVE(&GPIO->MOT[1], -20);
	RD_MOT_FORCE_DRIVE(&GPIO->MOT[2], -20);

	GPIO->A_EN_ALL = 1;				//lockA on
	GPIO->EN_ALL = 0;				//lockB off
	GPIO->LIGHT_EN = 0;				//Turn off Light

	return RET_OK;
}

// ASCEND_2	: CASE-6
RD_RET RD_CONTROL_CASE_ASCEND_2(CONTROL_DPC_t *CTL, PERIPHERAL_t* GPIO){
	// ASCEND_2				: winch_ctl 500, lockA on
	RD_MOT_FORCE_DRIVE(&GPIO->MOT[0], -50);
	RD_MOT_FORCE_DRIVE(&GPIO->MOT[1], -50);
	RD_MOT_FORCE_DRIVE(&GPIO->MOT[2], -50);

	GPIO->A_EN_ALL = 1;				//lockA on
	GPIO->EN_ALL = 0;				//lockB off

	return RET_OK;
}

// FINISH	: CASE-7
RD_RET RD_CONTROL_CASE_FINISH(CONTROL_DPC_t *CTL, PERIPHERAL_t* GPIO){
	// FINISH				: winch_ctl 0(brk), lockA off
	//RD_MOT_FORCE_DRIVE(&GPIO->MOT[0], 0);
	//RD_MOT_FORCE_DRIVE(&GPIO->MOT[1], 0);
	//RD_MOT_FORCE_DRIVE(&GPIO->MOT[2], 0);
	//back to init
	GPIO->MOT[0].dyn_ctrl.ram.cmd.goal_position = GPIO->MOT[0].INIT_POS;
	GPIO->MOT[1].dyn_ctrl.ram.cmd.goal_position = GPIO->MOT[1].INIT_POS;
	GPIO->MOT[2].dyn_ctrl.ram.cmd.goal_position = GPIO->MOT[2].INIT_POS;

	GPIO->A_EN_ALL = 0;				//lockA off
	GPIO->EN_ALL = 0;				//lockB off

	return RET_OK;
}

// ERROR	: CASE-10, default
RD_RET RD_CONTROL_CASE_ERROR(CONTROL_DPC_t *CTL, PERIPHERAL_t* GPIO){
	// ERROR				: winch_ctl 0(brk), lockA off, lockB off
	/*
	RD_MOT_FORCE_DRIVE(&GPIO->MOT[0], 0);
	RD_MOT_FORCE_DRIVE(&GPIO->MOT[1], 0);
	RD_MOT_FORCE_DRIVE(&GPIO->MOT[2], 0);
	*/
	//turn off torque
	GPIO->A_EN_ALL = 0;				//lockA off
	GPIO->EN_ALL = 0;				//lockB off
	//CTL->STATE = DPCB_STATE_CTRL; //back to init state

	return RET_OK;
}


