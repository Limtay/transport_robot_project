/*
 * rd_system.h
 *
 *  Created on: 2026-09-17
 *      Author: swarm
 *
 *  PRA 전역 객체/시스템 상태/태스크 진입점. DPC_B rd_system.h 계열 이식.
 *  Step 4·5 = Orin RS485 통신(RD_TASK_RS485). Step 6 = RS485(USART6) 채널 Checker + systemTask.
 *  ⚠ I2C1(패널)·CAN1(BMS) Checker 는 해당 페리페럴 추가 후 확장 (현재 uart2 슬롯만).
 */

#ifndef INC_RD_SYSTEM_H_
#define INC_RD_SYSTEM_H_

/* Private includes ----------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
#include "cmsis_os2.h"

#include "rd_common.h"
#include "rd_define.h"
#include "rd_uart.h"
#include "rd_comm_orin.h"   /* ORIN_COMM_t, RD_ORIN_* — Orin RS485 패킷 레이어 */
#include "rd_map_pra.h"     /* reg, DISPATCH/MARSHAL, ORIN_Error_e */
#include "rd_peripheral_pra.h"

#include "FreeRTOS.h"   /* taskENTER_CRITICAL / taskEXIT_CRITICAL */
#include "task.h"
#include <string.h>

/* Exported macro ------------------------------------------------------------*/
#define RS485_TEST_ON        /* 정의 시 FAULT 진입해도 자동 재부팅 차단 (브링업). 실운용 시 제거 */
#define FATAL_MAX  		200        /* fatal_cnt 임계 → SYS_STATE_FAULT */
#define FATAL_K    		20         /* Checker RET_NOK 1회당 fatal_cnt 증가 */

#define UVLO_TH			40.0f		/* UVLO threshold*/
#define OVLO_TH			60.0f		/* OVLO threshold*/
#define OCLO_TH			500.0f		/* OCLO but do not use yet. large value*/
#define OTLO_TH			80.0f		/* over temperature cutoff 80 */
#define UTLO_TH		   -20.0f		/* under temperature cut off -20 */

#define BOOT_TICK		100			/* 1s when boot AUX1 */
#define AUX1_OFF_TICK	500			/* 5s off when reboot seq */

#define RELAY_DELAY_TICK	 50		/* Relay on time, 0.5s */
#define PC_VOLTAGE_DIFF		0.5f	/* input, output voltage error gap*/
#define RUN_VOLTAGE_TH		1.0f	/* normal threadhold */
#define RUN_TRANS_TICK		100		/* off, on trans error check point */
#define BOOT_SEQ_TIMEOUT	500		/* Relay on time : 5s*/


#define SW_LOW			100			// more then 0.1s. based on Tick
#define SW_HIGH			3000		// less then 3s

/* Exported types ------------------------------------------------------------*/
/* sys_state(addr 23) 값 — rd_register_pra.h 주석과 일치. */
typedef enum {
    SYS_STATE_INIT 		= 0,   		/* 초기화 및 부팅 상태 */
    SYS_STATE_RUN_OFF,     			/* 정상 구동상태, 대전류 off */
    SYS_STATE_RUN_TRN,   			/* 정상 구동상태, 대전류 천이중 */
    SYS_STATE_RUN_ON,   			/* 정상 구동상태, 대전류 on */
    SYS_STATE_FAULT		= 255       /* 하드웨어 고장 (UART fatal 등) */
} SYSTEM_STATE_e;

typedef enum {
	BOOT_FSM_OFF 		= 0,   		/* 완전 종료상태 */
	BOOT_FSM_1_LSR_ON,				/* 로우사이드 릴레이 온 */
	BOOT_FSM_2_PC_ON,				/* 프리차저 온 */
	BOOT_FSM_3_HSR_ON,				/* 하이사이드 릴레이 온 */
	BOOT_FSM_4_PC_OFF,				/* 프리차저 오프 */
	BOOT_FSM_5_ON,					/* 완전 부팅 상태 */
	BOOT_FSM_6_PC_ON,				/* 프리차저 온. 아크 방지 */
	BOOT_FSM_7_HSR_OFF,				/* 하이사이드 릴레이 오프 */
	BOOT_FSM_8_PC_OFF,				/* 프리차저 오프 */
	BOOT_FSM_9_LSR_OFF,				/* 로우사이드 릴레이 오프 */
	BOOT_FSM_FAULT 		= 255		/* 전이 실패 상태. 모든 로직 강제 off */
} BOOT_FSM_e;

/* HW 채널 비트 — rd_register_pra.h HW_BIT_* 순서와 일치: bit0=CAN1 / bit1=uart2(Orin) / bit2=I2C1 */
typedef union {
    uint8_t raw;
    struct {
        uint8_t can1  : 1;   /* bit0 — BMS CAN (미구현) */
        uint8_t uart2 : 1;   /* bit1 — Orin RS485 (USART6) */
        uint8_t i2c   : 1;   /* bit2 — MCP23017 (미구현) */
        uint8_t RSVD  : 5;
    } bit;
} HARDWARE_STATUS_t;

typedef struct __attribute__((packed)) {
    HARDWARE_STATUS_t reset;   /* Checker 가 세운 "리셋 필요" 통보 (→ reg.sys.hw_reset addr22) */
    HARDWARE_STATUS_t fatal;   /* lifecycle==OFFLINE (→ reg.sys.hw_fatal addr21) */
    HARDWARE_STATUS_t error;   /* health>=WARN     (→ reg.sys.hw_error addr20) */
} HW_ERROR_FLAG_t;

typedef struct {
	volatile uint8_t BOOT_FSM;			// BOOT FSM : 0(off state), 1(lowside relay on), 2(precharging), 3(highside relay on), 4(precharging off),
										// 5(on state), 6(precharging on), 7(highside relay off), 8(precharger off), 9(lowside relay off), 255 is FSM fail.
	volatile uint8_t STATE;				//
	volatile uint32_t LOOP_CNT;			// counter of loop for timeout

	volatile uint32_t AUX_SW_LAST;		// counter for mode selection
	volatile uint32_t MAIN_SW_LAST;		// counter for mode selection

} CONTROL_PRA_t;
/* Exported HandlerType ------------------------------------------------------*/
extern UART_HandleTypeDef huart6;   /* Orin RS485 (USART6) */

/* Exported ObjectType -------------------------------------------------------*/
extern UART_Ring_t   PRA_uart6;     /* Orin RS485 backing UART (USART6) */
extern RS485_t       PRA_rs485;     /* Orin RS485 핸들 (.DIR=PC8) */
extern ORIN_COMM_t   ORIN_PACKET;   /* Orin RS485 패킷 채널 핸들 */

extern PERIPHERAL_t  PRA_PERIPHERAL; /* 전원GPIO+AUX+ADC+EXIO */
extern CONTROL_PRA_t PRA_CONTROL;    /* 전원 제어 FSM — .STATE 가 sys_state(addr23) 발행원 */

extern volatile uint8_t     rs485_init_fail_cnt;  /* USART6 INIT 재시도 (부팅 진단, 0=정상) */
extern volatile uint8_t     fatal_rs485_cnt;      /* USART6 Checker fatal 누적 */
extern HW_ERROR_FLAG_t hw;                        /* hw.error/fatal/reset 집계 */
extern uint32_t        tim_cnt;                   /* alive tick (현재 HAL_GetTick[ms], TIM5 부재) */

/* Exported functions prototypes ---------------------------------------------*/
void RD_SYSTEM_INIT(void);   	/* main() USER CODE BEGIN 2 에서 1회 (스케줄러 시작 전) */
void RD_TASK_RS485(void);    	/* rs485Task 진입점 — 내부 init + 이벤트 for(;;) */
void RD_TASK_SYSTEM(void);   	/* systemTask 진입점 — 10ms Checker/EVALUATE/PUBLISH */
void RD_TASK_CONTROL(void);		/* controlTask 진입점 — systemloop */
void RD_TASK_DEFAULT(void);

#endif /* INC_RD_SYSTEM_H_ */
