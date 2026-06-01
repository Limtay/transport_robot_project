/*
 * rd_system.c
 *
 *  Created on: 2026. 2. 24.
 *      Author: Lenovo
 */

/* Includes ------------------------------------------------------------------*/
#include "rd_system.h"
#include "rd_control.h"
#include "rd_i2c_encoder.h"
#include <string.h>

/* Exported variables ---------------------------------------------------------*/
static uint8_t rs485_error_cnt = 0;
uint8_t 	   can_fatal_cnt   = 0;
static uint8_t can_stable_cnt  = 0;

uint32_t tim_cnt = 0;


volatile uint8_t  fatal_uart1_cnt = 0;
volatile uint8_t  fatal_rs485_cnt = 0;
volatile uint8_t  fatal_can1_cnt = 0;

static volatile uint8_t  reboot_flag = 0;

#ifdef USE_RTOS_CAN_QUEUE
extern osMessageQueueId_t canTxQueueHandle;
#endif
#define TX_WARN_SOFT_RESET_MAX  5
#define MODE_STATE() (ECU_PERIPHERAL.data.MODE ? SYS_STATE_AUTO : SYS_STATE_MANUAL)

/*-----------CLASS Object ---------- */
LED_STATE_e LED_G_state = LED_BLINK_500;
LED_STATE_e LED_R_state = LED_RESET;

SYSTEM_STATE_e robot_state = SYS_STATE_INIT;

HW_ERROR_FLAG_t hw = {0};
/*========== UART1 (RC 수신기) ==========*/
UART_Ring_t ECU_uart1;
/*========== UART2 (RS485) ==========*/
UART_Ring_t ECU_uart2;
RS485_t 	ECU_rs485;

PACKET_comm_t ECU_PACKET;
RECEIVE_comm_t ECU_receive;
PERIPHERAL_t ECU_PERIPHERAL;

/* Exported function prototypes -----------------------------------------------*/
static RD_RET RD_SYSTEM_CHECKER(RD_RET state, uint8_t* hw_state, uint8_t* err_cnt);
static void RD_LED_BLINK(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, LED_STATE_e led_state, uint32_t* last_tick);
static void CAN_AK_ESTOP(float break_current);

static void ACTION_STATE_INIT(void);
static void ACTION_STATE_AUTO(void);
static void ACTION_STATE_ESTOP_HW(void);
static void ACTION_STATE_ESTOP_SW(void);
static void ACTION_STATE_FAULT(void);
static void ACTION_STATE_MANUAL(void);
static void RD_TX_WARN_Handler(void);

static RD_RET RD_SYSTEM_Update_Peripheral(void);
static void   RD_SYSTEM_Update_State(RD_RET periph_state);
static void   RD_SYSTEM_EVALUATE_STATE(PERIPHERAL_t *p);

/* Private Function code ------------------------------------------------------*/
static void RD_LED_BLINK(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, LED_STATE_e led_state, uint32_t* last_tick) {
	if (led_state == LED_RESET || led_state == LED_SET) {
		HAL_GPIO_WritePin(GPIOx, GPIO_Pin, led_state);
		*last_tick = osKernelGetTickCount();
	} else {
		uint16_t blink_interval = led_state;
		if (osKernelGetTickCount() - *last_tick >= blink_interval) {
			HAL_GPIO_TogglePin(GPIOx, GPIO_Pin);
			*last_tick = osKernelGetTickCount();
		}
	}
}

/* ESTOP: cmd_mtr 직접 만지지 않고 ESTOP_override + estop_current 만 set.
 * controlTask 의 PERIPHERAL_WRITE 가 ESTOP_override 보고 BRAKE 명령 생성 후 TX. */
static void CAN_AK_ESTOP(float break_current) {
	ECU_PERIPHERAL.data.motor_on       = 1;
	ECU_PERIPHERAL.data.ESTOP_override = 1;
	ECU_PERIPHERAL.data.estop_current  = break_current;
}

static void ACTION_STATE_INIT(void) {
	ECU_PERIPHERAL.data.motor_on       = 0;
	ECU_PERIPHERAL.data.ESTOP_override = 0;
}

static void ACTION_STATE_MANUAL(void) {
	ECU_PERIPHERAL.data.ESTOP_override = 0;
	/* RC 채널이 RUNNING/DEGRADED 일 때만 motor_on. OFFLINE 시 motor_on=0 */
	uint8_t lc = ECU_uart1.error.state.bits.lifecycle;
	uint8_t rc_ok = (lc == LS_RUNNING || lc == LS_DEGRADED) ? 1 : 0;
	ECU_PERIPHERAL.data.motor_on = (rc_ok && ECU_receive.receive_flag) ? 1 : 0;
}

static void ACTION_STATE_AUTO(void) {
	ECU_PERIPHERAL.data.ESTOP_override = 0;
	uint8_t any_nonzero = 0;
	if (reg.cmd_system.ctr_flag) {
		any_nonzero = (reg.cmd_system.cmd_lin_vel != 0.0f ||
		               reg.cmd_system.cmd_ang_vel != 0.0f) ? 1 : 0;
	} else {
		for (int i = 0; i < NUM_AK_MOTORS; i++) {
			if (reg.cmd_motor.cmd_velocity[i] != 0.0f) { any_nonzero = 1; break; }
		}
	}
	ECU_PERIPHERAL.data.motor_on = any_nonzero;
}

static void ACTION_STATE_ESTOP_HW(void) { CAN_AK_ESTOP(BREAK_CURRENT_HW); }
static void ACTION_STATE_ESTOP_SW(void) { CAN_AK_ESTOP(BREAK_CURRENT_SW); }

static void ACTION_STATE_FAULT(void) {
	ECU_PERIPHERAL.data.motor_on       = 0;
	ECU_PERIPHERAL.data.ESTOP_override = 0;

#ifdef RS485_TEST_ON
#else
	if (hw.fatal.bit.uart2) Error_Handler();
#endif
	if (hw.fatal.bit.can) {
		if (++can_fatal_cnt > FATAL_MAX) RD_ERROR_HANDLE();
		if (CAN_RECOVERY(&hcan1) == HAL_OK) {
			for (int i = 0; i < NUM_AK_MOTORS; i++) {
				ECU_AK[i].error.tx_err_cnt = 0;
				ECU_AK[i].error.rx_err_cnt = 0;
				ECU_AK[i].error.last_rx_tick = HAL_GetTick();
			}
			hw.fatal.bit.can = 0;
			robot_state = SYS_STATE_INIT;
		}
	}
}

static void RD_SYSTEM_UPDATE_STATE(STATE_t state) {
	if (robot_state == SYS_STATE_FAULT) return;
	RD_PERIPHERAL_READ(&ECU_PERIPHERAL);
	if (ECU_PERIPHERAL.data.ESTOP || ECU_PERIPHERAL.data.MODE_DONE) {
		robot_state = SYS_STATE_ESTOP_HW;
	} else if (robot_state == SYS_STATE_ESTOP_HW) {
		robot_state = (state.bits.health == HC_HW_FAULT) ? SYS_STATE_ESTOP_SW : MODE_STATE();
	} else if (robot_state == SYS_STATE_ESTOP_SW) {
		if (state < HC_THRESHOLD_WARN) robot_state = MODE_STATE();
	} else {
		robot_state = MODE_STATE();
	}
}

/**
 * @brief  채널별 STATE_t → hw_error_bits / hw_fatal_bits 집계.
 *         MARSHAL_PUBLISH 가 reg.sys.hw_error / hw_fatal 로 발행.
 */
static void RD_SYSTEM_EVALUATE_STATE(void)
{
    STATE_t s;
	s = ECU_PERIPHERAL.err.can.state;
    if (s.bits.health    >= HC_THRESHOLD_WARN) hw.error.bit.can = 1;
    if (s.bits.lifecycle == LS_OFFLINE)        hw.fatal.bit.can = 1;

    s = ECU_PERIPHERAL.err.i2c.state;
    if (s.bits.health    >= HC_THRESHOLD_WARN) hw.error.bit.i2c = 1;
    if (s.bits.lifecycle == LS_OFFLINE)        hw.fatal.bit.i2c = 1;

    s = ECU_uart1.error.state;
    if (s.bits.health    >= HC_THRESHOLD_WARN) hw.error.bit.uart1 = 1;
    if (s.bits.lifecycle == LS_OFFLINE)        hw.fatal.bit.i2c = 1;

    s = ECU_uart2.error.state;
    if (s.bits.health    >= HC_THRESHOLD_WARN) hw.error.bit.uart2 = 1;
    if (s.bits.lifecycle == LS_OFFLINE)        hw.fatal.bit.i2c = 1;
}

/* Private Function code ------------------------------------------------------*/
void RD_SYSTEM_INIT(void) {
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
  HAL_Delay(1000);
  /*==========UART INIT==========*/
  ECU_uart1.error.state.raw           = LS_INIT;
  ECU_rs485.uart_obj->error.state.raw = LS_INIT;
  /*==========COMM INIT==========*/
  RD_RECEIVE_INIT(&ECU_receive);
  RD_PACKET_INIT(&ECU_PACKET);
  /*==========MAP INIT===========*/
  RD_MAP_INIT();
  /*========Control INIT=========*/
  RD_CONTROL_INIT();
  /*=======Peripheral INIT=======*/
  RD_PERIPHERAL_INIT(&ECU_PERIPHERAL, &hcan1, &hi2c1);
  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
}
/*  RTOS TASK  --------------------------------------------------------*/
void RD_TASK_DEFAULT(void) {
  static uint32_t last_r_tick = osKernelGetTickCount();
  static uint32_t last_g_tick = osKernelGetTickCount();
  for(;;)
  {
	RD_LED_BLINK(LED_R_GPIO_Port, LED_R_Pin, LED_R_state, &last_r_tick);
	RD_LED_BLINK(LED_G_GPIO_Port, LED_G_Pin, LED_G_state, &last_g_tick);
	osDelay(50);
  }
}

static void fatal_cnt_plus(uint8_t *cnt) { *cnt =(*cnt+FATAL_K > FATAL_MAX) ? FATAL_MAX : *cnt+FATAL_K;}
static void fatal_cnt_minu(uint8_t *cnt) { if(*cnt > 0) *cnt--;}

void RD_SYSTEM_CHECKER(void) {
  uint8_t lc = ECU_PERIPHERAL.err.can.state.bits.lifecycle;
  if (lc != LS_RECOVERING) {
	  if (RD_CAN_MOTOR_CHECKER(ECU_PERIPHERAL.data_mtr, ECU_PERIPHERAL.err) == RET_NOK){
		  fatal_cnt_plus(fatal_can1_cnt);
		  if (RD_CAN_MOTOR_RECOVERY(&ECU_PERIPHERAL) == RET_NOK)
			  fatal_cnt_plus(fatal_can1_cnt);
	  } else fatal_cnt_minu(fatal_can1_cnt);
  }

  uint8_t lc = ECU_rs485->uart_obj->error.state.bits.lifecycle;
  if (lc != LS_RECOVERING) {
	  if (RD_RS485_CHECKER(&ECU_rs485, DEGRADED_K_100HZ) == RET_NOK){
		  fatal_cnt_plus(fatal_rs485_cnt);
		  if (RD_RS485_RECOVERY(&ECU_rs485, &huart2) == RET_NOK)
			  fatal_cnt_plus(fatal_rs485_cnt);
	  } else fatal_cnt_minu(fatal_rs485_cnt);
  }

  uint8_t lc = ECU_uart1.error.state.bits.lifecycle;
  if (lc != LS_RECOVERING) {
	  if (RD_UART_CHECKER(&ECU_uart1, DEGRADED_K_100HZ) == RET_NOK){
		  fatal_cnt_plus(fatal_uart1_cnt);
		  if (RD_UART_RECOVERY(&ECU_uart1, &huart1) == RET_NOK)
			  fatal_cnt_plus(fatal_uart1_cnt);
	  } else fatal_cnt_minu(fatal_uart1_cnt);
  }
}

void RD_TASK_SYSTEM(void) {
  static uint32_t tick = osKernelGetTickCount();
  for(;;)
  {
	RD_SYSTEM_CHECKER();
	RD_SYSTEM_EVALUATE_STATE(&ECU_PERIPHERAL);
	RD_SYSTEM_UPDATE_STATE(ECU_PERIPHERAL.err.can.state);

	switch (robot_state) {
		case SYS_STATE_FAULT:    LED_G_state = LED_BLINK_100; break;
		case SYS_STATE_ESTOP_SW: LED_G_state = LED_BLINK_100; break;
		default:                 LED_G_state = LED_BLINK_500; break;
	}
	switch (robot_state) {
		case SYS_STATE_INIT:     ACTION_STATE_INIT();     break;
		case SYS_STATE_MANUAL:   ACTION_STATE_MANUAL();   break;
		case SYS_STATE_AUTO:     ACTION_STATE_AUTO();     break;
		case SYS_STATE_ESTOP_HW: ACTION_STATE_ESTOP_HW(); break;
		case SYS_STATE_ESTOP_SW: ACTION_STATE_ESTOP_SW(); break;
		case SYS_STATE_FAULT:    ACTION_STATE_FAULT();    break;
	}

	RD_MAP_MARSHAL_PUBLISH(&ECU_PERIPHERAL);
	if (robot_state == SYS_STATE_MANUAL || robot_state == SYS_STATE_AUTO) {
		RD_MAP_MARSHAL_CONSUME(&ECU_PERIPHERAL);
	}
	if (robot_state == SYS_STATE_AUTO) {
		/* kinematics mode (ctr_flag=1): lin/ang_vel → cmd_mtr.cmd_velocity[] 덮어쓰기 */
		float lin, ang;
		uint8_t use_kin;
		taskENTER_CRITICAL();
		lin     = reg.cmd_system.cmd_lin_vel;
		ang     = reg.cmd_system.cmd_ang_vel;
		use_kin = reg.cmd_system.ctr_flag;
		taskEXIT_CRITICAL();
		if (use_kin) {
			float rpm_out[NUM_AK_MOTORS];
			RD_CONTROL_KINEMATICS(lin, ang, rpm_out);
			taskENTER_CRITICAL();
			for (int i = 0; i < NUM_AK_MOTORS; i++) {
				ECU_PERIPHERAL.cmd_mtr.cmd_velocity[i] = rpm_out[i];
				ECU_PERIPHERAL.cmd_mtr.ctr_mode[i]     = MODE_VELOCITY;
			}
			taskEXIT_CRITICAL();
		}
	}

	tick += 10;
	osDelayUntil(tick);
  }
}

void RD_TASK_CONTROL(void) {
  uint32_t tick = osKernelGetTickCount();
  static SYSTEM_STATE_e prev_state = SYS_STATE_INIT;
  for(;;)
  {
	if (robot_state != prev_state) {
		RD_CONTROL_RESET_FILTER();
		prev_state = robot_state;
	}
	RD_CONTROL_UPDATE(&ECU_PERIPHERAL.cmd_mtr, robot_state);
	if (RD_PERIPHERAL_WRITE(&ECU_PERIPHERAL) != RET_OK) {
		robot_state = SYS_STATE_FAULT;
	}
	tick += RD_TASK_CONTROL_200Hz;
	osDelayUntil(tick);
  }
}

void RD_TASK_CAN1(void) {
  for(;;)
  {
#ifdef USE_RTOS_CAN_QUEUE
  CAN_AK_TX_TASK_HANDLER();
#else
  osDelay(10);
#endif
  }
}

void RD_TASK_RS485(void) {
  if (RD_RS485_INIT(&ECU_rs485) != RET_OK) RD_REBOOT_HANDLE();
  RD_RET packet_state = RET_OK;
  for(;;)
  {
#ifdef RTOS_IS_AVAILABLE
	osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
#else
	osDelay(1);
#endif
	packet_state = RD_PACKET_READ(&ECU_rs485, &ECU_PACKET);
	if (packet_state == RET_OK){
		LED_R_state = LED_RESET;
		RD_PACKET_HANDLE(&ECU_PACKET);
		RD_RET wr = RD_PACKET_WRITE(&ECU_rs485, &ECU_PACKET);
		if (ECU_PACKET.reboot_pending) {
			if (wr == RET_OK) reboot_flag = 1;
			ECU_PACKET.reboot_pending = 0;
		}
	}else if(packet_state == RET_NOK) {
		LED_R_state = LED_BLINK_100;
	}
  }
}

void RD_TASK_RC(void) {
  if (RD_UART_INIT(&ECU_uart1) != RET_OK) RD_REBOOT_HANDLE();
  for(;;)
  {
#ifdef RTOS_IS_AVAILABLE
	osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
#else
	osDelay(1);
#endif
//	  if (RD_RECEIVE_READ(&ECU_uart1, &ECU_receive) == RET_OK)
	RD_RECEIVE_READ(&ECU_uart1, &ECU_receive);
  }
}

void RD_TASK_I2C1(void) {
	static uint32_t tick = osKernelGetTickCount();
	for(;;)
	{
		RD_PERIPHERAL_I2C(&ECU_PERIPHERAL);
		tick += 10;
		osDelayUntil(tick);
	}
}
/*  Callback & Error Handler  --------------------------------------------------------*/
void RD_TIM_CALLBACK(void) { tim_cnt++; }

void RD_REBOOT_HANDLE(void) {
	HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0 | CAN_TX_MAILBOX1 | CAN_TX_MAILBOX2);
#ifdef USE_RTOS_CAN_QUEUE
	osMessageQueueReset(canTxQueueHandle);
#endif
	NVIC_SystemReset();
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
	if (hcan->Instance == CAN1) {
		AK_RxFrame_t frame;
		if (CAN_AK_RX_POP(hcan, &frame)) {
			for (int i = 0; i < NUM_AK_MOTORS; i++) {
				if (CAN_AK_RX_APPLY(&ECU_AK[i], &frame)) break;
			}
		}
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* ISR — lifecycle 직접 변경 금지. raw HAL 에러코드만 캡처 → CHECKER 가 매핑. */
    if (huart->Instance == USART2) ECU_uart2.error.isr_err_code = HAL_UART_GetError(huart);
    if (huart->Instance == USART1) ECU_uart1.error.isr_err_code = HAL_UART_GetError(huart);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    	ECU_PERIPHERAL.err.i2c.isr_err_code = HAL_I2C_GetError(hi2c);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1) {
    	ECU_PERIPHERAL.err.can.isr_err_code = HAL_CAN_GetError(hcan);
        HAL_CAN_ResetError(hcan);
    }
}
