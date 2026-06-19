/*
 * rd_peripheral_dpcb.c
 *
 *  Created on: Jan 27, 2026
 *      Author: abc01
 */


/* Includes ------------------------------------------------------------------*/
#include "rd_peripheral_dpcb.h"
#include <string.h>


/* Exported includes ----------------------------------------------------------*/

/* Exported typedef -----------------------------------------------------------*/

/* Exported define ------------------------------------------------------------*/


/* Exported variables ---------------------------------------------------------*/
extern PERIPHERAL_t DPCB_PERIPHERAL;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim3;
extern I2C_HandleTypeDef hi2c1;
/* Exported function prototypes -----------------------------------------------*/
RD_RET RD_PERIPHERAL_INIT(PERIPHERAL_t* GPIO);
//RD_RET RD_ENC_INIT(peripheral_enc_t* enc);
RD_RET RD_EXIO_INIT(peripheral_exio_t* exio);

RD_RET RD_PERIPHERAL_WRITE(PERIPHERAL_t* GPIO); //Drive IO need
RD_RET RD_PERIPHERAL_READ(PERIPHERAL_t* GPIO); //Read IO state what we need

RD_RET RD_EXIO_UPDATE(peripheral_exio_t* exio);
RD_RET RD_DPCA_UPDATE(PERIPHERAL_t* GPIO, PACKET_comm_t* PACKET);

void Update_Continuous_Angle(float current, float prev, volatile float* accumulated, uint8_t rev);
//void Drive_Motor(PERIPHERAL_MOT_IO_t* MOT_IO, volatile int32_t CTL, volatile uint8_t STP);

//RD_RET RD_PWM_Capture(peripheral_enc_t* enc, GPIO_IO_t* enc_io, volatile uint32_t now_tick);

/* Private user code ---------------------------------------------------------*/

/*==================== EXIO PVC ===================*/
RD_RET RD_EXIO_INIT(peripheral_exio_t* exio){
	//Port A:0~7, Port B:8:15
	// Output 설정
	EXIO_Set_OUTPUT(exio->hi2c, EXIO_LED1);
	EXIO_Set_OUTPUT(exio->hi2c, EXIO_LED2);

	// Input 설정
	EXIO_Set_INPUT(exio->hi2c, EXIO_SW1);
	EXIO_Set_INPUT(exio->hi2c, EXIO_SW2);
	EXIO_Set_INPUT(exio->hi2c, EXIO_SW3A);
	EXIO_Set_INPUT(exio->hi2c, EXIO_SW3B);
	EXIO_Set_INPUT(exio->hi2c, EXIO_SW4A);
	EXIO_Set_INPUT(exio->hi2c, EXIO_SW4B);
	EXIO_Set_INPUT(exio->hi2c, EXIO_SW5A);
	EXIO_Set_INPUT(exio->hi2c, EXIO_SW5B);
	EXIO_Set_INPUT(exio->hi2c, EXIO_SW6A);
	EXIO_Set_INPUT(exio->hi2c, EXIO_SW6B);

	HAL_GPIO_WritePin(EXIO_RST_GPIO_Port, EXIO_RST_Pin, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(EXIO_RST_GPIO_Port, EXIO_RST_Pin, GPIO_PIN_SET);

	return RET_OK;
}

RD_RET RD_EXIO_UPDATE(peripheral_exio_t* exio){
	//READ
	exio->SW1_state = EXIO_ReadPin(exio->hi2c, EXIO_SW1);
	exio->SW2_state = EXIO_ReadPin(exio->hi2c, EXIO_SW2);

	uint8_t temp_swa, temp_swb;

	temp_swa = EXIO_ReadPin(exio->hi2c, EXIO_SW3A);
	temp_swb = EXIO_ReadPin(exio->hi2c, EXIO_SW3B);
	exio->SW3_state = (temp_swa << 1) | temp_swb;

	temp_swa = EXIO_ReadPin(exio->hi2c, EXIO_SW4A);
	temp_swb = EXIO_ReadPin(exio->hi2c, EXIO_SW4B);
	exio->SW4_state = (temp_swa << 1) | temp_swb;

	temp_swa = EXIO_ReadPin(exio->hi2c, EXIO_SW5A);
	temp_swb = EXIO_ReadPin(exio->hi2c, EXIO_SW5B);
	exio->SW5_state = (temp_swa << 1) | temp_swb;

	temp_swa = EXIO_ReadPin(exio->hi2c, EXIO_SW6A);
	temp_swb = EXIO_ReadPin(exio->hi2c, EXIO_SW6B);
	exio->SW6_state = (temp_swa << 1) | temp_swb;

	//WRITE
	EXIO_WritePin(exio->hi2c, EXIO_LED1, (uint8_t)exio->LED1_state);
	EXIO_WritePin(exio->hi2c, EXIO_LED2, (uint8_t)exio->LED2_state);

	return RET_OK;
}

/*==================== DPC_A communication ===================*/
RD_RET RD_DPCA_UPDATE(PERIPHERAL_t* GPIO, PACKET_comm_t* PACKET){
	GPIO->A_CON_DATA = PACKET->rx.Data[0];
	GPIO->A_PROX_DATA = PACKET->rx.Data[1];

	if (GPIO->A_EN_ALL != 0){
		PACKET->tx.Data[0] = 1;
	}else{
		PACKET->tx.Data[0] = 0;
	}
	if (GPIO->A_EN_BOOT != 0){
		PACKET->tx.Data[1] = 1;
	}else{
		PACKET->tx.Data[1] = 0;
	}

	return RET_OK;
}

/*==================== MOTOR PVC ===================*/
RD_RET RD_MOT_DRIVE(PERIPHERAL_MOT_t* MOT, int32_t speed){
	uint16_t now_tick = MOT->dyn_ctrl.ram.state.realtime_tick;
	MOT->delta_tick = now_tick-MOT->dyn_present_tick;
	MOT->dyn_present_tick = now_tick;
	int32_t curr_pos = MOT->dyn_ctrl.ram.state.present_position;
	int32_t targ_pos = curr_pos + (int32_t)((float)speed * 0.3908f);

	MOT->dyn_ctrl.ram.cmd.goal_position = targ_pos;

	return RET_OK;
}

RD_RET RD_MOT_FORCE_DRIVE(PERIPHERAL_MOT_t* MOT, int32_t speed){
	MOT->TARGET_POS = MOT->TARGET_POS + (int32_t)((float)speed * 0.3908f);

	MOT->dyn_ctrl.ram.cmd.goal_position = MOT->TARGET_POS;

	return RET_OK;
}
/*==================== GENERAL ===================*/

RD_RET RD_PERIPHERAL_INIT(PERIPHERAL_t* GPIO)
{
	if (GPIO == NULL) return RET_NOK;

	/*============================== PIN INIT ==============================*/

	//INIT BOOT pin
	GPIO->IO.BOOT_IO_1.per_GPIO_Pin = BOOT_IO_1_Pin;
	GPIO->IO.BOOT_IO_1.per_GPIOx = BOOT_IO_1_GPIO_Port;

	GPIO->IO.BOOT_IO_2.per_GPIO_Pin = BOOT_IO_2_Pin;
	GPIO->IO.BOOT_IO_2.per_GPIOx = BOOT_IO_2_GPIO_Port;

	//INIT SOL pin
	GPIO->IO.EN_IO.per_GPIO_Pin = SOL_EN_Pin;				//solenoid all EN
	GPIO->IO.EN_IO.per_GPIOx = SOL_EN_GPIO_Port;

	GPIO->IO.CON_A_IO.per_GPIO_Pin = CON_A_Pin;				//solenoid CON A
	GPIO->IO.CON_A_IO.per_GPIOx = CON_A_GPIO_Port;

	GPIO->IO.CON_B_IO.per_GPIO_Pin = CON_B_Pin;				//solenoid CON B
	GPIO->IO.CON_B_IO.per_GPIOx = CON_B_GPIO_Port;

	GPIO->IO.CON_C_IO.per_GPIO_Pin = CON_C_Pin;				//solenoid CON C
	GPIO->IO.CON_C_IO.per_GPIOx = CON_C_GPIO_Port;

	GPIO->IO.CON_D_IO.per_GPIO_Pin = CON_D_Pin;				//solenoid CON D
	GPIO->IO.CON_D_IO.per_GPIOx = CON_D_GPIO_Port;

	GPIO->IO.LIGHT_IO.per_GPIO_Pin = LIGHT_IO_Pin;			//LIGHT EN
	GPIO->IO.LIGHT_IO.per_GPIOx = LIGHT_IO_GPIO_Port;

	GPIO->IO.SERVO_IO.per_GPIO_Pin = SERVO_IO_Pin;			//SERVO
	GPIO->IO.SERVO_IO.per_GPIOx = SERVO_IO_GPIO_Port;
	GPIO->IO.SERVO_IO.per_pCCR = &(htim3.Instance->CCR1);

	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);				//start pwm

	//INIT DYN
	GPIO->MOT[0].DYN_IDS = 2;
	GPIO->MOT[1].DYN_IDS = 3;
	GPIO->MOT[2].DYN_IDS = 4;


	/*============================== STATE INIT ==============================*/

	//INIT SOL state
	GPIO->EN_ALL = 0;
	GPIO->LAST_EN_TICK = 0; //reset tick (ms)
	GPIO->EN_BOOT = 0;

	GPIO->CON_DATA = 0;

	//EXIO
	GPIO->PANEL.hi2c = &hi2c1;
	RD_EXIO_INIT(&(GPIO->PANEL));


	return RET_OK;
}


RD_RET RD_PERIPHERAL_WRITE(PERIPHERAL_t* GPIO)
{
	if (GPIO == NULL) return RET_NOK;

	/* ==================== WRITE MOT ====================*/
	if (GPIO->EN_ALL == 1)			HAL_GPIO_WritePin(GPIO->IO.EN_IO.per_GPIOx, GPIO->IO.EN_IO.per_GPIO_Pin, GPIO_PIN_SET);
	else if (GPIO->EN_ALL == 0)		HAL_GPIO_WritePin(GPIO->IO.EN_IO.per_GPIOx, GPIO->IO.EN_IO.per_GPIO_Pin, GPIO_PIN_RESET);

	if (GPIO->LIGHT_EN == 1)		HAL_GPIO_WritePin(GPIO->IO.LIGHT_IO.per_GPIOx, GPIO->IO.LIGHT_IO.per_GPIO_Pin, GPIO_PIN_SET);
	else if (GPIO->LIGHT_EN == 0)	HAL_GPIO_WritePin(GPIO->IO.LIGHT_IO.per_GPIOx, GPIO->IO.LIGHT_IO.per_GPIO_Pin, GPIO_PIN_RESET);

	if (GPIO->SERVO_EN == 0)		*(GPIO->IO.SERVO_IO.per_pCCR) = 0;		// idle
	else if (GPIO->SERVO_EN == 1)	*(GPIO->IO.SERVO_IO.per_pCCR) = 950; 	// unlock
	else if (GPIO->SERVO_EN == 2)	*(GPIO->IO.SERVO_IO.per_pCCR) = 1950; 	// lock

	if (GPIO->EN_BOOT == 1)			HAL_GPIO_TogglePin(GPIO->IO.BOOT_IO_1.per_GPIOx, GPIO->IO.BOOT_IO_1.per_GPIO_Pin); 	//inverse for UART dummy format
	else if (GPIO->EN_BOOT == 0)	HAL_GPIO_WritePin(GPIO->IO.BOOT_IO_1.per_GPIOx, GPIO->IO.BOOT_IO_1.per_GPIO_Pin, GPIO_PIN_SET);		//so it is pull-up pin

	return RET_OK;
}

RD_RET RD_PERIPHERAL_READ(PERIPHERAL_t* GPIO)
{
	if (GPIO == NULL) return RET_NOK;
	/* ==================== READ EXIO ====================*/
	RD_EXIO_UPDATE(&(GPIO->PANEL));

	/* ==================== READ GPIO ====================*/

	GPIO->CON_DATA = (HAL_GPIO_ReadPin(GPIO->IO.CON_A_IO.per_GPIOx, GPIO->IO.CON_A_IO.per_GPIO_Pin) << 7) |
	                 (HAL_GPIO_ReadPin(GPIO->IO.CON_B_IO.per_GPIOx, GPIO->IO.CON_B_IO.per_GPIO_Pin) << 6) |
	                 (HAL_GPIO_ReadPin(GPIO->IO.CON_C_IO.per_GPIOx, GPIO->IO.CON_C_IO.per_GPIO_Pin) << 5) |
	                 (HAL_GPIO_ReadPin(GPIO->IO.CON_D_IO.per_GPIOx, GPIO->IO.CON_D_IO.per_GPIO_Pin) << 4);


	/* ==================== READ MOT ====================*/

	return RET_OK;
}

/* ==================== READ ME ====================*/
// 0. RD_PERIPHERAL_INIT은 "실제 핀의 배치를 추상화하는 함수"임. 실제로 포트바뀌면 여기서 다 수정해야되고, 기본적으로 main.h에서 1차적으로 정의되있긴함.
// 1. RD_PERIPHERAL_WRITE는 실제 핀에 값을 write 하는곳.
// 2. RD_PERIPHERAL_READ는 실제 핀의 로직값이나 센싱값을 스트럭쳐에 갱신하는곳
