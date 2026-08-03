/*
 * rd_system.h
 *
 *  Created on: 2026-06-19
 *      Author: swarm
 */

#ifndef INC_RD_SYSTEM_H_
#define INC_RD_SYSTEM_H_

/* Private includes ----------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
#include "cmsis_os.h"

#include "rd_common.h"
#include "rd_define.h"

#include "rd_uart.h"
#include "rd_comm_dpcb.h"          /* DPC-A 4-byte 단순 패킷 */
#include "rd_peripheral_dpcb.h"   /* rd_map_dyn.h / rd_comm_dyn.h 포함 */
#include "rd_control.h"
#include "rd_map_dpcb.h"          /* reg, DISPATCH/MARSHAL, ORIN_Error_e */
#include "rd_comm_orin.h"          /* ORIN_COMM_t, RD_ORIN_* — Orin RS485 패킷 레이어 */

/* Exported macro ------------------------------------------------------------*/


#define RS485_TEST_ON // TEST ON

#define AUTO_TIMEOUT 100 // [ms]
// System_Checker() 호출 횟수 기준 [count]
#define FATAL_MAX 200
#define FATAL_K   20

/* Exported types ------------------------------------------------------------*/
typedef enum {
	LED_RESET = GPIO_PIN_RESET,
	LED_SET	  = GPIO_PIN_SET,
	LED_BLINK_100 = 100,
	LED_BLINK_500 = 500
} LED_STATE_e;

typedef enum {
    SYS_STATE_INIT = 0,   // 초기화 및 부팅 상태
    SYS_STATE_MANUAL,     // 패널 조종 모드
    SYS_STATE_AUTO,       // 자율 전개모드
    SYS_STATE_ESTOP_SW,   // 소프트 ESTOP (과열/에러 - 조건 해소 시 자동 복귀)
    SYS_STATE_ESTOP_HW,   // 물리 ESTOP 스위치 (스위치 해제 시 복귀) < 삭제할듯
    SYS_STATE_FAULT       // 하드웨어 고장 (CAN bus-off, UART fatal 등)
} SYSTEM_STATE_e;

typedef union {
	uint8_t raw;
	struct {
		uint8_t uart2 : 1;
		uint8_t uart4 : 1;
		uint8_t uart6 : 1;   /* IMU (구 uart4 슬롯 재사용 — reg.sys 비트 위치 불변) */
		uint8_t i2c   : 1;
		uint8_t RSVD  : 4;
	} bit;
} HARDWARE_STATUS_t;

typedef struct __attribute__((packed)) {
	HARDWARE_STATUS_t reset;
	HARDWARE_STATUS_t fatal;
	HARDWARE_STATUS_t error;
} HW_ERROR_FLAG_t;

/* Exported HandlerType ------------------------------------------------------*/
extern UART_HandleTypeDef huart2;   /* Orin RS485 (USART2) */
extern UART_HandleTypeDef huart4;   /* DPCA (UART4) */
extern UART_HandleTypeDef huart6;   /* Dynamixel RS485 (USART6) */
extern TIM_HandleTypeDef  htim5;    /* 32-bit us 타이머 */

/* RTOS task handles — rd_system.c 에서 wake_task 주입 시 사용 */
extern osThreadId_t periTaskHandle; /* Dynamixel(USART6) 태스크 */

/* Exported ObjectType -------------------------------------------------------*/
extern UART_Ring_t   DPCA_uart4;
extern PACKET_comm_t DPCA_PACKET;

extern PERIPHERAL_t  DPCB_PERIPHERAL;
extern CONTROL_DPC_t DPC_CTL;

extern UART_Ring_t   DPCB_uart6;    /* Dynamixel RS485 backing UART */
extern RS485_t       DPCB_dyn;      /* Dynamixel RS485 핸들 */

extern UART_Ring_t   DPCB_uart2;    /* Orin RS485 backing UART */
extern RS485_t       DPCB_rs485;    /* Orin RS485 핸들 */
extern ORIN_COMM_t   ORIN_PACKET;   /* Orin RS485 패킷 채널 핸들 */

extern volatile uint8_t rs485_init_fail_cnt;  /* USART2 INIT 재시도 횟수 (부팅 진단, 0 이 정상) */

extern volatile SYSTEM_STATE_e payload_state;
extern uint32_t      tim_cnt;
extern uint32_t      Diff_tick;
extern HW_ERROR_FLAG_t hw;

/* Exported functions prototypes ---------------------------------------------*/
void RD_SYSTEM_INIT(void);
void RD_TASK_SYSTEM(void);

void RD_TASK_DEFAULT(void);  /* 1000ms — LED 상태 표시 */
void RD_TASK_CONTROL(void);  /* 10ms   — 적재전개 FSM 컨트롤 */
void RD_TASK_RS485(void);    /* 이벤트 — Dynamixel 모터 제어 */
void RD_TASK_I2C(void);      /* 10ms   — I2C 패널(MCP23017) 갱신 */
void RD_TASK_DPCA(void);     /* ~10ms  — DPCA 송수신 + 변수 매칭 */
void RD_TASK_PERI(void);     /* 10ms   — GPIO 읽기/쓰기 */

#endif /* INC_RD_SYSTEM_H_ */
