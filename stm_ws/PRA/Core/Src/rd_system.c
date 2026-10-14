/*
 * rd_system.c
 *
 *  Created on: 2026-09-17
 *      Author: swarm
 *
 *  PRA 전역 객체 + 시스템 초기화 + Orin RS485 태스크 + systemTask Checker.
 *  DPC_B rd_system.c 이식. Checker 는 **USART6(Orin/uart2 슬롯) 채널만** 구현 —
 *  CAN1(BMS)·I2C1(패널) 은 해당 페리페럴 추가 후 확장(EVALUATE/CHECKER 에 훅만 주석).
 */

/* Includes ------------------------------------------------------------------*/
#include "rd_system.h"
#include <math.h>

/* Exported variables (전역 객체) --------------------------------------------*/
UART_Ring_t   PRA_uart6;     	/* Orin RS485 backing UART (USART6) */
RS485_t       PRA_rs485;     	/* Orin RS485 핸들 (.DIR=PC8) */
ORIN_COMM_t   ORIN_PACKET;   	/* Orin RS485 패킷 채널 핸들 */

PERIPHERAL_t  PRA_PERIPHERAL; 	/* PRA Peripheral 패킷 */
CONTROL_PRA_t PRA_CONTROL;		/* PRA 제어 스트럭쳐 */

volatile uint8_t     rs485_init_fail_cnt = 0;
volatile uint8_t     fatal_rs485_cnt     = 0;
HW_ERROR_FLAG_t hw      = {0};
uint32_t        tim_cnt = 0;

/* Private helpers -----------------------------------------------------------*/
static void fatal_cnt_plus(volatile uint8_t *cnt) { *cnt = (*cnt + FATAL_K > FATAL_MAX) ? FATAL_MAX : *cnt + FATAL_K; }
static void fatal_cnt_minu(volatile uint8_t *cnt) { if (*cnt > 0) (*cnt)--; }

/* ISR: HAL UART 에러 raw 코드 캡처 (Checker 가 isr_err_take 로 소비) */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6) PRA_uart6.error.isr_err_code |= HAL_UART_GetError(huart);
}

/**
 * @brief  Orin 요청 하드웨어 리셋 처리. reg.reg_df.hw_reset(addr5) 비트가 올라오면
 *         해당 채널 RECOVERY. (현재 uart2=USART6 만 구현, can1/i2c 는 페리페럴 추가 후)
 */
static void RD_SYSTEM_HW_RESET_HANDLE(void)
{
    HARDWARE_STATUS_t req;
    taskENTER_CRITICAL();
    req.raw = reg.reg_df.hw_reset;
    taskEXIT_CRITICAL();
    if (req.raw == 0) return;

    if (req.bit.uart2) { RD_RS485_RECOVERY(&PRA_rs485); fatal_rs485_cnt = 0; }
    /* if (req.bit.can1) { ... }   // TODO: CAN1 페리페럴 추가 후 */
    /* if (req.bit.i2c)  { ... }   // TODO: I2C1 페리페럴 추가 후 */

    taskENTER_CRITICAL();
    reg.reg_df.hw_reset &= (uint8_t)~req.raw;   /* addr5 Orin 요청 플래그 클리어 */
    taskEXIT_CRITICAL();
}

/**
 * @brief  채널별 Checker + Recovery (10ms/100Hz).
 *         USART6(Orin/uart2): fatal 누적 시 UART RECOVERY + hw.reset telltale 만.
 *         ⚠ 통신 FAULT ↔ 전원 FSM 의도적 미연계 (2026-10-04): Orin 통신 단절로 메인 전원을
 *            내리지 않는다. 전원 차단·self-reboot 없음. 채널 health 는 hw_error/hw_fatal 로 노출.
 */
static void RD_SYSTEM_CHECKER(void)
{
    /* ── PRA_rs485 (USART6 / Orin RS485, 레지스터 uart2 슬롯) ─────────────── */
    uint8_t lc = PRA_rs485.uart_obj->error.state.bits.lifecycle;
    if (lc != LS_RECOVERING) {
        if (RD_RS485_CHECKER(&PRA_rs485, DEGRADED_K_100HZ) == RET_NOK) {
            fatal_cnt_plus(&fatal_rs485_cnt);
            if (RD_RS485_RECOVERY(&PRA_rs485) == RET_NOK)
                fatal_cnt_plus(&fatal_rs485_cnt);
            if (fatal_rs485_cnt >= FATAL_MAX) {
                hw.reset.bit.uart2 = 1;   /* Orin 가시용 telltale (addr22). 로컬 전원 액션 없음 */
            }
        } else {
            fatal_cnt_minu(&fatal_rs485_cnt);
        }
    }

    /* ── CAN1(BMS) / I2C1(패널) : 페리페럴 추가 후 확장 (Step9/10) ─────────── */
}

/* ACTION_STATE_FAULT 제거(2026-10-04, #5): 통신 fatal 시 self-reboot/전원차단 안 함.
 * 전원 세이프티 정리는 RD_CONTROL_LOOP 의 FAULT(else) 분기가 담당(출력 off). */

/**
 * @brief  채널 STATE_t → hw.error/hw.fatal 집계 (레벨 트리거, 매 tick 재계산).
 */
static void RD_SYSTEM_EVALUATE_STATE(void)
{
    hw.error.raw = 0;
    hw.fatal.raw = 0;

    STATE_t s;
    s = PRA_uart6.error.state;      /* Orin RS485 (uart2 슬롯) */
    if (s.bits.health    >= HC_THRESHOLD_WARN) hw.error.bit.uart2 = 1;
    if (s.bits.lifecycle == LS_OFFLINE)        hw.fatal.bit.uart2 = 1;

    /* CAN1/I2C1 채널 상태 집계는 페리페럴 추가 후 */
}

/* System init ---------------------------------------------------------------*/
/**
 * @brief  스케줄러 시작 전(main USER CODE BEGIN 2) 1회 호출.
 *         RS485 DIR/uart 링크 주입 + 레지스터 맵/패킷 채널 초기화.
 * @note   실제 RS485(HAL_UART_Init+DMA)는 RD_TASK_RS485 진입부에서 RD_RS485_INIT 로 수행.
 */
void RD_SYSTEM_INIT(void)
{
    /*==========RS485 핸들 링크 + DIR 핀 주입 (DIP)==========*/
    PRA_rs485.uart_obj          = &PRA_uart6;
    PRA_rs485.DIR.per_GPIOx     = RS485_DIR_GPIO_Port;   /* PC8 (Orin RS485 DIR) */
    PRA_rs485.DIR.per_GPIO_Pin  = RS485_DIR_Pin;

    /*==========레지스터 맵 / 패킷 채널 INIT==========*/
    RD_MAP_INIT();
    RD_ORIN_INIT(&ORIN_PACKET);
}

RD_RET RD_CONTROL_INIT(CONTROL_PRA_t *CTL){
	CTL->AUX_SW_LAST 		= HAL_GetTick();
	CTL->MAIN_SW_LAST 		= HAL_GetTick();

	CTL->LOOP_CNT 			= 0;

	CTL->BOOT_FSM 			= 0;
	CTL->STATE 				= 0;
	return RET_OK;
}
/*
RD_RET RD_PROTECT_CHECKER(CONTROL_PRA_t *CTL, PERIPHERAL_t *PERI){
	if ((PERI->SENSOR_Voltage_IN < UVLO_TH) || (PERI->SENSOR_Voltage_IN < OVLO_TH)){
		CTL->STATE = 0; //do not ready boot or work.
	}
}
*/

/* Task implementations ------------------------------------------------------*/
void RD_CONTROL_LOOP(CONTROL_PRA_t *CTL, PERIPHERAL_t *PERI){
	CTL->LOOP_CNT++;

	/* ========== STATE FSM ==========  */
	if(CTL->STATE == SYS_STATE_INIT){
		//TODO : CHECK
		/* PERI write */
		PERI->EN_AUX1 		= 0;
		PERI->EN_AUX2 		= 0;
		PERI->EN_SSP 		= 0;
		if(CTL->LOOP_CNT > 10){ //만약 릴레이켜진상태에서 init들어갈때 SSP 손상 방지용으로 SSP부터끄고 시간차줌
			PERI->EN_RelayN 	= 0;
			PERI->EN_RelayP 	= 0;
		}

		/* LOGIC write */
		CTL->BOOT_FSM = 0; //hold boot FSM, to do not move

		/* Transition */
		/* LOGIC BASE transition */
		if ((PERI->SENSOR_Voltage_IN < UVLO_TH) || (PERI->SENSOR_Voltage_IN > OVLO_TH)){
			CTL->LOOP_CNT = 0; //do not ready, cnt is 0
		}
		if(CTL->LOOP_CNT > BOOT_TICK) {
			CTL->STATE = SYS_STATE_RUN_OFF; 	//normal + 1s after transition state.
			CTL->LOOP_CNT = 0; 	//init loop cnt
			CTL->MAIN_SW_LAST = HAL_GetTick();
		}
		/* SW BASE transition */


	}else if(CTL->STATE >= SYS_STATE_RUN_OFF && CTL->STATE <= SYS_STATE_RUN_ON){
		// RUN STATE
		/* PERI write */
		PERI->EN_AUX1		= 1; //boot embed

		//TODO : FSM BOOT SEQ

		//CTL->BOOT_FSM이 넘겨줌. 따로 뭘 하진않음
		if (PERI->EXIO.SW1_state == 1){ // SW1 뗐을때
			uint32_t delta_tick = HAL_GetTick() - CTL->MAIN_SW_LAST;
			if (delta_tick > SW_LOW && delta_tick < SW_HIGH){ 			//짧은틱
				//do not use
			}else if (delta_tick > SW_HIGH){							//긴틱

				if(CTL->BOOT_FSM == BOOT_FSM_5_ON) {
					CTL->BOOT_FSM = BOOT_FSM_6_PC_ON;
					CTL->STATE = SYS_STATE_RUN_TRN;
				}
				if(CTL->BOOT_FSM == BOOT_FSM_OFF) {
					CTL->BOOT_FSM = BOOT_FSM_1_LSR_ON;
					CTL->STATE = SYS_STATE_RUN_TRN;
				}
				CTL->LOOP_CNT = 0;
			}
			CTL->MAIN_SW_LAST = HAL_GetTick();
			//TODO : 경험적으로 그냥 키면, 부팅될때 자동전이가 발생함. init에서 틱 갱신을 해줬긴한데, 그사이 시간이 100ms보다 작을지는 실험해봐야앎.
		}

		/*else if (CTL->STATE == SYS_STATE_RUN_TRN){
			if(CTL->BOOT_FSM == BOOT_FSM_OFF){
				CTL->BOOT_FSM = BOOT_FSM_1_LSR_ON; // start on
				CTL->MAIN_SW_LAST = HAL_GetTick();
				CTL->LOOP_CNT = 0;
			}else if(CTL->BOOT_FSM == BOOT_FSM_5_ON){
				CTL->BOOT_FSM = BOOT_FSM_6_PC_ON; // start off
				CTL->MAIN_SW_LAST = HAL_GetTick();
				CTL->LOOP_CNT = 0;
			}

		}
		*/
		/* LOGIC write */

		/* Transition */

		/* LOGIC BASE transition */
		for (int i=0; i<3; i++){ //temperature lockout
			if((PERI->NTC[i] > OTLO_TH) || (PERI->NTC[i] < UTLO_TH)){
				CTL->BOOT_FSM = BOOT_FSM_FAULT;
				CTL->STATE = SYS_STATE_FAULT; //transition fault state
			}
		}
		if ((PERI->SENSOR_Voltage_IN < UVLO_TH) || (PERI->SENSOR_Voltage_IN > OVLO_TH)){
			CTL->BOOT_FSM = BOOT_FSM_FAULT;
			CTL->STATE = SYS_STATE_FAULT; //transition fault state
		}
		/* SW BASE transition */



		/*
	}else if(CTL->STATE == 2){
		//TODO : REBOOT
		//PERI write
		PERI->EN_AUX1		= 0; //boot embed

		//LOGIC write

		//Transition
		if(CTL->LOOP_CNT > AUX1_OFF_TICK){
			CTL->STATE = 1; //back to boot
		}
		*/
	}else{ //FAULT state
		//TODO : something is very wrong
		PERI->EN_AUX1 = 0;
		PERI->EN_AUX2 = 0;
		PERI->EN_SSP = 0;
		if(CTL->LOOP_CNT > 100){
			PERI->EN_RelayP = 0;
			PERI->EN_RelayN = 0; //부하가 다 릴레이로 가게
		}
	}

	float delta_voltage;

	/* ========== BOOT FSM ==========  */
	switch (CTL->BOOT_FSM) {

		/* OFF */
		case BOOT_FSM_OFF:
			/* PERI */
			PERI->EN_SSP 	= 0;
			PERI->EN_RelayP = 0;
			PERI->EN_RelayN = 0;

			/* Transition */
			CTL->STATE = SYS_STATE_RUN_OFF; //no transition, just state change

			if(CTL->LOOP_CNT > RUN_TRANS_TICK && PERI->SENSOR_Voltage_OUT > RUN_VOLTAGE_TH) CTL->BOOT_FSM = BOOT_FSM_FAULT; //trans tick만큼 지났을때 전압 안떨어지면 fault
			break;

		/* ON TRANS */
		case BOOT_FSM_1_LSR_ON:
			/* PERI */
			PERI->EN_SSP 	= 0;
			PERI->EN_RelayP = 0;
			PERI->EN_RelayN = 1;

			/* Transition */
			//normal
			if(CTL->LOOP_CNT > RELAY_DELAY_TICK){
				CTL->BOOT_FSM = BOOT_FSM_2_PC_ON;
				CTL->LOOP_CNT = 0;
			}
			//fault
			if(CTL->LOOP_CNT > BOOT_SEQ_TIMEOUT) CTL->BOOT_FSM = BOOT_FSM_FAULT;
			break;

		case BOOT_FSM_2_PC_ON:
			/* PERI */
			PERI->EN_SSP 	= 1;
			PERI->EN_RelayP = 0;
			PERI->EN_RelayN = 1;

			/* Transition */
			//normal
			delta_voltage = fabsf(PERI->SENSOR_Voltage_IN - PERI->SENSOR_Voltage_OUT); //48 - 47.8 = positive 0.2
			if(CTL->LOOP_CNT > RELAY_DELAY_TICK && delta_voltage < PC_VOLTAGE_DIFF){
				CTL->BOOT_FSM = BOOT_FSM_3_HSR_ON;
				CTL->LOOP_CNT = 0;
			}
			//fault
			if(CTL->LOOP_CNT > BOOT_SEQ_TIMEOUT) CTL->BOOT_FSM = BOOT_FSM_FAULT;
			break;

		case BOOT_FSM_3_HSR_ON:
			/* PERI */
			PERI->EN_SSP 	= 1;
			PERI->EN_RelayP = 1;
			PERI->EN_RelayN = 1;

			/* Transition */
			//normal
			if(CTL->LOOP_CNT > RELAY_DELAY_TICK) {
				CTL->BOOT_FSM = BOOT_FSM_4_PC_OFF;
				CTL->LOOP_CNT = 0;
			}
			//fault
			if(CTL->LOOP_CNT > BOOT_SEQ_TIMEOUT) CTL->BOOT_FSM = BOOT_FSM_FAULT;
			break;

		case BOOT_FSM_4_PC_OFF:
			/* PERI */
			PERI->EN_SSP 	= 0; //it have to be 0, but circuit issue, remain 1
			PERI->EN_RelayP = 1;
			PERI->EN_RelayN = 1;

			/* Transition */
			//normal
			CTL->BOOT_FSM = BOOT_FSM_5_ON; //direct transition
			CTL->LOOP_CNT = 0;
			break;

		/* ON */
		case BOOT_FSM_5_ON:
			/* PERI */
			PERI->EN_SSP 	= 0; //fix too
			PERI->EN_RelayP = 1;
			PERI->EN_RelayN = 1;

			/* Transition */
			CTL->STATE = SYS_STATE_RUN_ON; //no transition, just state change
			//falt check
			//delta_voltage = fabsf(PERI->SENSOR_Voltage_IN - PERI->SENSOR_Voltage_OUT); //48 - 47.8 = positive 0.2
			delta_voltage = 0; //DNP. because diode block voltage barrier
			if(CTL->LOOP_CNT > RUN_TRANS_TICK && delta_voltage > PC_VOLTAGE_DIFF) CTL->BOOT_FSM = BOOT_FSM_FAULT; //trans tick만큼 지났을때 전압 안근접하면 fault

			break;

		/* OFF TRANS */
		case BOOT_FSM_6_PC_ON:
			/* PERI */
			PERI->EN_SSP 	= 1;
			PERI->EN_RelayP = 1;
			PERI->EN_RelayN = 1;

			/* Transition */
			//normal
			delta_voltage = fabsf(PERI->SENSOR_Voltage_IN - PERI->SENSOR_Voltage_OUT); //48 - 47.8 = positive 0.2
			if(CTL->LOOP_CNT > RELAY_DELAY_TICK && delta_voltage < PC_VOLTAGE_DIFF) { //이건 정상작동. ssp켜져서 전압잡힘
				CTL->BOOT_FSM = BOOT_FSM_7_HSR_OFF;
				CTL->LOOP_CNT = 0;
			}
			//fault
			if(CTL->LOOP_CNT > BOOT_SEQ_TIMEOUT) CTL->BOOT_FSM = BOOT_FSM_FAULT;
			break;

		case BOOT_FSM_7_HSR_OFF:
			/* PERI */
			PERI->EN_SSP 	= 1;
			PERI->EN_RelayP = 0;
			PERI->EN_RelayN = 1;

			/* Transition */
			//normal
			if(CTL->LOOP_CNT > RELAY_DELAY_TICK) {
				CTL->BOOT_FSM = BOOT_FSM_8_PC_OFF;
				CTL->LOOP_CNT = 0;
			}
			//fault
			if(CTL->LOOP_CNT > BOOT_SEQ_TIMEOUT) CTL->BOOT_FSM = BOOT_FSM_FAULT;
			break;
		case BOOT_FSM_8_PC_OFF:
			/* PERI */
			PERI->EN_SSP 	= 0;
			PERI->EN_RelayP = 0;
			PERI->EN_RelayN = 1;

			/* Transition */
			//normal
			if(CTL->LOOP_CNT > RELAY_DELAY_TICK) {
				CTL->BOOT_FSM = BOOT_FSM_9_LSR_OFF;
				CTL->LOOP_CNT = 0;
			}
			//fault
			if(CTL->LOOP_CNT > BOOT_SEQ_TIMEOUT) CTL->BOOT_FSM = BOOT_FSM_FAULT;
			break;

		case BOOT_FSM_9_LSR_OFF:
			/* PERI */
			PERI->EN_SSP 	= 0;
			PERI->EN_RelayP = 0;
			PERI->EN_RelayN = 0;

			/* Transition */
			//normal
			if(CTL->LOOP_CNT > RELAY_DELAY_TICK) {
				CTL->BOOT_FSM = BOOT_FSM_OFF;
				CTL->LOOP_CNT = 0;
			}
			//fault
			if(CTL->LOOP_CNT > BOOT_SEQ_TIMEOUT) CTL->BOOT_FSM = BOOT_FSM_FAULT;
			break;

		default:
			//TODO : FAULT
			PERI->EN_SSP 	= 0;
			PERI->EN_RelayP = 0;
			PERI->EN_RelayN = 0;
			//fault진입시, precharge 기능 전체를 마비시킴. 다른곳에서 인위적으로 on이나 off로 풀어주지 않으면 중지됨.
			break;
	}

	/* ========== AUX2 FSM ==========  */

	if(CTL->STATE >= SYS_STATE_RUN_OFF && CTL->STATE <= SYS_STATE_RUN_ON){ //when run state

		if (PERI->EXIO.SW2_state == 1){ // SW2 뗐을때
			uint32_t delta_tick = HAL_GetTick() - CTL->AUX_SW_LAST;
			if (delta_tick > SW_LOW && delta_tick < SW_HIGH){ 			//짧은틱
				//do not use
			}else if (delta_tick > SW_HIGH){							//긴틱
				if (PERI->EN_AUX2 == 1) 		PERI->EN_AUX2 = 0;		//오린파워 : 토글 (off)
				else if (PERI->EN_AUX2 == 0) 	PERI->EN_AUX2 = 1;		//오린파워 : 토글 (o n)
			}
			CTL->AUX_SW_LAST = HAL_GetTick();
			//TODO : 경험적으로 그냥 키면, 부팅될때 자동전이가 발생함. init에서 틱 갱신을 해줬긴한데, 그사이 시간이 100ms보다 작을지는 실험해봐야앎.
		}
	}


}
/**
 * @brief  rs485Task 진입점 — Orin RS485 modbus 서비스. main.c Start_rs485 는 본 함수만 호출.
 */
void RD_TASK_RS485(void)
{
    /* ── 초기화 (task 컨텍스트) ──────────────────
     * Error_Handler(보드 프리즈) 대신 재시도 카운터 + 반복 실패 시 리셋. */
    while (RD_RS485_INIT(&PRA_rs485, &huart6) != RET_OK) {
        rs485_init_fail_cnt++;
        if (rs485_init_fail_cnt >= 10) NVIC_SystemReset();
        osDelay(100);
    }
    PRA_uart6.wake_task = rs485TaskHandle;   /* IDLE ISR → rs485Task 기상 */

    /* ── 이벤트 루프 ─────────────────────────────*/
    for (;;)
    {
        osThreadFlagsWait(0x0001, osFlagsWaitAny, 10);   /* IDLE ISR set 또는 10ms 폴링 */

        RD_MAP_MARSHAL_PUBLISH();   /* 요청 직전 발행 (request-synchronous, SYSTEM 영역) */

        if (RD_ORIN_READ(&PRA_rs485, &ORIN_PACKET) != RET_OK) continue;

        RD_ORIN_HANDLE(&ORIN_PACKET, 0);   /* lock=0 (모드 기반 권한 TODO) */
        RD_RET wr = RD_ORIN_WRITE(&PRA_rs485, &ORIN_PACKET);

        if (ORIN_PACKET.reboot_pending) {
            ORIN_PACKET.reboot_pending = 0;
            if (wr == RET_OK) {
                uint32_t t0 = osKernelGetTickCount();
                while (PRA_rs485.uart_obj->huart->gState != HAL_UART_STATE_READY &&
                       (osKernelGetTickCount() - t0) < 50) {
                    osDelay(1);
                }
                osDelay(2);
                NVIC_SystemReset();
            }
        }
    }
}

void RD_TASK_CONTROL(void)
{
	/* RD_PERIPHERAL_INIT 내부에서 IO 핀 주입 + RD_EXIO_INIT + RD_ADC_INIT(DMA 기동) 수행 */
	RD_PERIPHERAL_INIT(&PRA_PERIPHERAL);
	RD_CONTROL_INIT(&PRA_CONTROL);

	for (;;)
	{
		RD_PERIPHERAL_READ(&PRA_PERIPHERAL);    /* ADC 갱신(RD_UPDATE_ADC) */
		RD_EXIO_UPDATE(&(PRA_PERIPHERAL.EXIO)); /* MCP23017 SW read + LED/BUZ write */
		RD_PERIPHERAL_WRITE(&PRA_PERIPHERAL);   /* 전원 GPIO(Relay/SSP/AUX) write */

		RD_CONTROL_LOOP(&PRA_CONTROL, &PRA_PERIPHERAL);
		osDelay(10);
	}
}

/**
 * @brief  systemTask 진입점 — 10ms Checker/EVALUATE/발행. main.c StartSystem 은 본 함수만 호출.
 */
void RD_TASK_SYSTEM(void)
{
    uint32_t wake = osKernelGetTickCount();

    for (;;)
    {
        wake += 10;
        osDelayUntil(wake);

        tim_cnt = HAL_GetTick();   /* alive tick [ms] — TIM5 부재, 스케일 후속 조정(register 는 ×0.1ms) */

        RD_SYSTEM_HW_RESET_HANDLE();
        RD_SYSTEM_CHECKER();
        RD_SYSTEM_EVALUATE_STATE();

        RD_MAP_MARSHAL_PUBLISH();   /* SYSTEM 영역 발행 (hw.degraded/sys_state=PRA_CONTROL.STATE) */
        /* RD_MAP_MARSHAL_CONSUME();  // Step7 */
    }
}

void RD_TASK_DEFAULT(void){
	//max 1s
	uint8_t IND_tick = 0;
	for (;;)
	{
		IND_tick++;

		//TODO : BMS communication success + no fault -> green light on, no fault no bus green blink
		//TODO : fault only turn on red light

		/* AUX INDICATOR */
		if (PRA_PERIPHERAL.EN_AUX2 == 1){ //blink is on, on is off
			if(IND_tick%10 < 5){
				PRA_PERIPHERAL.EXIO.LED2_state = 1;
			}else{
				PRA_PERIPHERAL.EXIO.LED2_state = 0;
			}
		}else{
			PRA_PERIPHERAL.EXIO.LED2_state = 1;
		}

		/* MAIN INDICATOR */
		if (PRA_CONTROL.STATE == SYS_STATE_RUN_OFF){
			PRA_PERIPHERAL.EXIO.LED1_state = 0;
		}else if (PRA_CONTROL.STATE == SYS_STATE_RUN_ON){
			PRA_PERIPHERAL.EXIO.LED1_state = 1;
		}else if (PRA_CONTROL.STATE == SYS_STATE_RUN_TRN){
			if(IND_tick%2 == 1){
				PRA_PERIPHERAL.EXIO.LED1_state = 0;
			}else{
				PRA_PERIPHERAL.EXIO.LED1_state = 1;
			}
		}

		osDelay(100);// 0.1s
	}
}






