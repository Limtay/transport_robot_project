/*
 * rd_system.h
 *
 *  Created on: 2026. 2. 24.
 *      Author: Lenovo
 */
/* Private includes ----------------------------------------------------------*/
#ifndef INC_RD_SYSTEM_H_
#define INC_RD_SYSTEM_H_

#include "stm32f4xx_hal.h"
#include "main.h"
#include "cmsis_os.h"

#include "rd_common.h"
#include "rd_define.h"

#include "rd_uart.h"
#include "rd_comm_ecu.h"
#include "rd_comm_receive.h"
#include "rd_comm_imu.h"
#include "rd_peripheral_ecu.h"
#include "rd_map_ecu.h"
#include "rd_register_ecu.h"
/* Exported macro ------------------------------------------------------------*/


#define RS485_TEST_ON // TEST ON

#define AUTO_TIMEOUT 100 // [ms]
// System_Checker() 호출 횟수 기준 [count]
#define FATAL_MAX 200
#define FATAL_K   20

#define CAN_STABLE_MIN 10

#define BREAK_CURRENT_HW 10 // [A]
#define BREAK_CURRENT_SW  3 // [A]


/* Exported types ------------------------------------------------------------*/
typedef enum {
	LED_RESET = GPIO_PIN_RESET,
	LED_SET	  = GPIO_PIN_SET,
	LED_BLINK_100 = 100,
	LED_BLINK_500 = 500
} LED_STATE_e;

typedef enum {
    SYS_STATE_INIT = 0,   // 초기화 및 부팅 상태
    SYS_STATE_MANUAL,     // 수동 조종 모드 (수신기 기반)
    SYS_STATE_AUTO,       // 자율 주행 모드 (Orin AGX 기반)
    SYS_STATE_ESTOP_SW,   // 소프트 ESTOP (과열/에러 - 조건 해소 시 자동 복귀)
    SYS_STATE_ESTOP_HW,   // 물리 ESTOP 스위치 (스위치 해제 시 복귀)
    SYS_STATE_FAULT       // 하드웨어 고장 (CAN bus-off, UART fatal 등)
} SYSTEM_STATE_e;

typedef union {
	uint8_t raw;
	struct {
		uint8_t uart1 : 1;
		uint8_t uart2 : 1;
		uint8_t uart6 : 1;   /* IMU (구 uart4 슬롯 재사용 — reg.sys 비트 위치 불변) */
		uint8_t can   : 1;
		uint8_t i2c   : 1;
		uint8_t RSVD  : 3;
	} bit;
} HARDWARE_STATUS_t;

typedef struct __attribute__((packed)) {
	HARDWARE_STATUS_t reset;
	HARDWARE_STATUS_t fatal;
	HARDWARE_STATUS_t error;
} HW_ERROR_FLAG_t;
/* Exported variables --------------------------------------------------------*/

/* Exported HandlerType ------------------------------------------------------*/
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart6;
extern CAN_HandleTypeDef  hcan1;
extern I2C_HandleTypeDef  hi2c1;
extern TIM_HandleTypeDef  htim5;

/* Exported ObjectType ---------------------------------------------------------*/
extern volatile SYSTEM_STATE_e robot_state;
extern HW_ERROR_FLAG_t hw;
extern uint8_t  can_fatal_cnt;   // CAN FAULT 재시도 카운터 (FAULT 상태 진입 후 CAN_RECOVERY 시도 횟수)
extern uint32_t tim_cnt;         // TIM5 1kHz 카운터 (ECU Alive time 계산용)

extern UART_Ring_t ECU_uart1;
extern UART_Ring_t ECU_uart2;
extern UART_Ring_t ECU_uart6;
extern RS485_t ECU_rs485;

extern PACKET_comm_t ECU_PACKET;
extern RECEIVE_comm_t ECU_receive;
extern IMU_comm_t ECU_imu;
extern PERIPHERAL_t ECU_PERIPHERAL;

/* Exported constants --------------------------------------------------------*/

/* Control task 주기 — 정수배 권장 (200/250/500/1000 Hz), 최대 1000Hz */
#define RD_TASK_CONTROL_200Hz 5
#define RD_TASK_CONTROL_250Hz 4
#define RD_TASK_CONTROL_500Hz 2

/* Exported functions prototypes ---------------------------------------------*/
void RD_SYSTEM_INIT(void);

void RD_TASK_DEFAULT(void);  /* 50ms (20Hz)  — LED 상태 표시 */
void RD_TASK_SYSTEM(void);   /* 10ms (100Hz) — FSM + GPIO + EVALUATE_STATE + MARSHAL */
void RD_TASK_CONTROL(void);  /* 1000/RD_TASK_CONTROL_HZ ms — LPF + CAN TX */
void RD_TASK_RS485(void);    /* flag + 20ms checker — PACKET + RS485_CHECKER */
void RD_TASK_IMU(void);
void RD_TASK_RC(void);       /* 1ms poll + 20ms checker — RC RECEIVE + UART_CHECKER */
void RD_TASK_CAN1(void);     /* queue drain — CAN_AK_TxTask_Handler */
void RD_TASK_I2C1(void);     /* 10ms (100Hz) — I2C_ENCODER_UPDATE + ENCODER_CHECKER */

uint64_t Get_Time_us(void);

void RD_TIM_CALLBACK(void);
void RD_REBOOT_HANDLE(void);
#endif /* INC_RD_SYSTEM_H_ */
