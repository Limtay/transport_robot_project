/*
 * rd_system.c
 *
 *  Created on: 2026-06-19
 *      Author: swarm
 */

/* Includes ------------------------------------------------------------------*/
#include "rd_system.h"

/* Private includes ----------------------------------------------------------*/

/* Exported variables ---------------------------------------------------------*/
uint32_t tim_cnt = 0;

volatile uint8_t  fatal_uart4_cnt = 0;
volatile uint8_t  fatal_rs485ex_cnt = 0;
volatile uint8_t  fatal_rs485_cnt = 0;

/* 부팅 진단 (Live Watch 전용, 2026-08-03).
 * INIT 실패를 Error_Handler(=__disable_irq + while(1)) 로 처리하면 보드 전체가 얼어
 * "펌웨어가 죽었다" 와 "배선/트랜시버 문제다" 를 구분할 수 없다. 카운터로 남긴다. */
volatile uint8_t  rs485_init_fail_cnt = 0;   /* USART2(Orin) INIT 재시도 횟수 — 0 이 정상 */

/* Exported ObjectType -------------------------------------------------------*/

/*-----------CLASS Object ---------- */
LED_STATE_e LED_G_state = LED_BLINK_500;
LED_STATE_e LED_R_state = LED_RESET;

volatile SYSTEM_STATE_e payload_state = SYS_STATE_INIT;  /* systemTask + controlTask 공유 → volatile */

HW_ERROR_FLAG_t hw = {0};


CONTROL_DPC_t DPC_CTL;					// FSM state
PERIPHERAL_t  DPCB_PERIPHERAL;			// 페리페럴

/*==========수신용 usart4번 (DPCA)==========*/
/* huart 는 RD_UART_INIT(obj, huart) 에서 주입 */
UART_Ring_t   DPCA_uart4 = {0};
PACKET_comm_t DPCA_PACKET;

/*==========USART6 RS485 (Dynamixel)==========*/
/* huart 는 RD_RS485_INIT 에서. DIR 핀은 여기서 정적 주입. */
UART_Ring_t   DPCB_uart6 = {0};
RS485_t       DPCB_dyn = {
    .uart_obj = &DPCB_uart6,
    .DIR = { .per_GPIOx = RS485_EX_DIR_GPIO_Port, .per_GPIO_Pin = RS485_EX_DIR_Pin }
};

/*==========USART2 RS485 (Orin)==========*/
UART_Ring_t   DPCB_uart2 = {0};
RS485_t       DPCB_rs485 = {
    .uart_obj = &DPCB_uart2,
    .DIR = { .per_GPIOx = RS485_DIR_GPIO_Port, .per_GPIO_Pin = RS485_DIR_Pin }
};
ORIN_COMM_t   ORIN_PACKET;          /* Orin RS485 패킷 채널 핸들 */



/*==========Dynamixel 루프 타이머==========*/
uint32_t      Diff_tick; //주기성 체크용이니 무시해도됨


/* Private function prototypes -----------------------------------------------*/
static void RD_SYSTEM_CHECKER(void);
static void ACTION_STATE_FAULT(void);

/* Private Function code ------------------------------------------------------*/
/* ── IWDG (독립 워치독) — HAL 모듈 미포함이라 레지스터 직접 제어 ──────────────
 *  LSI 32kHz / prescaler 64 = 500Hz (2ms/tick), reload 250 → ~500ms 타임아웃.
 *  start 후에는 정지 불가 → systemTask 가 heartbeat 조건 만족 시에만 refresh. */
/*
static void RD_IWDG_START(void) {
	__HAL_DBGMCU_FREEZE_IWDG();
	IWDG->KR  = 0x0000CCCCU;   // IWDG enable (LSI 자동 기동)
	IWDG->KR  = 0x00005555U;   // PR/RLR 쓰기 허용
	IWDG->PR  = 0x04U;         // prescaler /64
	IWDG->RLR = 250U;          // 250 × 2ms ≈ 500ms
	// PVU/RVU 갱신 완료 대기 (LSI 안정화 전 무한 spin 방지 위해 bound)
	for (volatile uint32_t t = 0; (IWDG->SR != 0U) && (t < 100000U); t++) { }
	IWDG->KR  = 0x0000AAAAU;   //초기 refresh
}
*/

static void fatal_cnt_plus(volatile uint8_t *cnt) { *cnt = (*cnt + FATAL_K > FATAL_MAX) ? FATAL_MAX : *cnt + FATAL_K; }
static void fatal_cnt_minu(volatile uint8_t *cnt) { if (*cnt > 0) (*cnt)--; }

/**
 * @brief  Orin 요청 하드웨어 리셋 처리 (Code_modify.md — STM/ECU).
 *         reg.reg_df.hw_reset (addr 5) 비트가 올라오면 해당 채널을 직접 RECOVERY 하고
 *         addr 54 (hw.reset → reg.sys.hw_reset) / addr 5 플래그를 모두 내린다.
 *         흐름: Checker 가 hw.reset 비트로 리셋 필요 통보 (addr 54)
 *               → Orin 이 RCLCPP_ERROR 확인 후 addr 5 에 해당 비트 WRITE
 *               → 여기서 실제 리셋 수행 + 양쪽 플래그 클리어.
 * @note   FAULT escalation 채널(can/uart2)도 리셋은 수행하지만 robot_state 는
 *         건드리지 않음 — FAULT 탈출은 기존 정책대로 Orin REBOOT 명령 사용.
 */

static void RD_SYSTEM_HW_RESET_HANDLE(void) {
	HARDWARE_STATUS_t req;
	taskENTER_CRITICAL();
	req.raw = reg.reg_df.hw_reset;
	taskEXIT_CRITICAL();
	if (req.raw == 0) return;

	if (req.bit.uart2) { RD_RS485_RECOVERY(&DPCB_rs485);  fatal_rs485_cnt = 0; }
	if (req.bit.uart4) { RD_UART_RECOVERY(&DPCA_uart4); fatal_uart4_cnt = 0; }
	if (req.bit.uart6) { RD_UART_RECOVERY(&DPCB_uart6);  fatal_rs485ex_cnt = 0; }
	//if (req.bit.i2c)   { RD_I2C_ENCODER_RECOVERY(&hi2c1, &ECU_PERIPHERAL.err); }

	taskENTER_CRITICAL();
	//hw.reset.raw        &= (uint8_t)~req.raw;  // addr 54 (MARSHAL_PUBLISH 가 발행)
	//reg.reg_df.hw_reset &= (uint8_t)~req.raw;  // addr 5  (Orin 요청 플래그)
	taskEXIT_CRITICAL();
}



/**
 * @brief  채널별 Checker + Recovery 디스패치 + FAULT 에스컬레이션.
 *         10ms 주기 (100Hz) 호출 기준.
 *
 *  채널별 정책:
 *   DPCB_rs485 (uart2) : ECU_V3 동일 — fatal_rs485_cnt ≥ FATAL_MAX → SYS_STATE_FAULT + reboot
 *   DPCA_uart4  (uart4): FAULT 없음 — fatal_uart4_cnt ≥ FATAL_MAX → hw.reset.bit.uart4 + LS_RECOVERING
 *   DPCB_dyn    (uart6): FAULT 없음, hw.reset 없음 — 버스 상태만 EVALUATE_STATE 로 집계
 */
static void RD_SYSTEM_CHECKER(void)
{
    uint8_t lc;

    /* ── DPCB_rs485 (USART2 / Orin RS485) ─────────────────────────────────── */
    lc = DPCB_rs485.uart_obj->error.state.bits.lifecycle;
    if (lc != LS_RECOVERING) {
        if (RD_RS485_CHECKER(&DPCB_rs485, DEGRADED_K_100HZ) == RET_NOK) {
            fatal_cnt_plus(&fatal_rs485_cnt);
            if (RD_RS485_RECOVERY(&DPCB_rs485) == RET_NOK)
                fatal_cnt_plus(&fatal_rs485_cnt);
            if (fatal_rs485_cnt >= FATAL_MAX) {
                hw.reset.bit.uart2 = 1;
                payload_state = SYS_STATE_FAULT;
            }
        } else {
            fatal_cnt_minu(&fatal_rs485_cnt);
        }
    }

    /* ── DPCA_uart4 (UART4 / DPC-A) ───────────────────────────────────────── */
    lc = DPCA_uart4.error.state.bits.lifecycle;
    if (lc != LS_RECOVERING) {
        if (RD_UART_CHECKER(&DPCA_uart4, DEGRADED_K_100HZ) == RET_NOK) {
            fatal_cnt_plus(&fatal_uart4_cnt);
            if (RD_UART_RECOVERY(&DPCA_uart4) == RET_NOK)
                fatal_cnt_plus(&fatal_uart4_cnt);
            if (fatal_uart4_cnt >= FATAL_MAX) {
                hw.reset.bit.uart4 = 1;
                DPCA_uart4.error.state.bits.lifecycle = LS_RECOVERING;
            }
        } else {
            fatal_cnt_minu(&fatal_uart4_cnt);
        }
    }

    /* ── DPCB_dyn (USART6 / Dynamixel RS485) ──────────────────────────────── */
    /* FAULT 에스컬레이션 없음 — 버스 상태는 EVALUATE_STATE 가 hw.error/hw.fatal 로 집계.
     * 개별 모터 정상 여부는 dyn_ctrl.comm_flag 기반으로 상위에서 별도 처리. */
    lc = DPCB_dyn.uart_obj->error.state.bits.lifecycle;
    if (lc != LS_RECOVERING) {
        if (RD_RS485_CHECKER(&DPCB_dyn, DEGRADED_K_100HZ) == RET_NOK) {
            fatal_cnt_plus(&fatal_rs485ex_cnt);
            if (RD_RS485_RECOVERY(&DPCB_dyn) == RET_NOK)
                fatal_cnt_plus(&fatal_rs485ex_cnt);
        } else {
            fatal_cnt_minu(&fatal_rs485ex_cnt);
        }
    }
}

/**
 * @brief  SYS_STATE_FAULT 진입 시 안전 처리.
 *         전체 모터 토크 OFF 후 RS485_TEST_ON 비활성 빌드에서만 MCU 재부팅.
 */
static void ACTION_STATE_FAULT(void)
{
    for (int i = 0; i < DYN_NUM_MOTORS; i++) {
        RD_DYN_TORQUE_ON(&DPCB_PERIPHERAL.MOT[i].dyn_ctrl, 0);
        RD_DYN_LOOP(&DPCB_dyn, &DPCB_PERIPHERAL.MOT[i].dyn_ctrl);
    }
#ifndef RS485_TEST_ON
    if (hw.reset.bit.uart2) NVIC_SystemReset();
#endif
}

/**
 * @brief  채널별 STATE_t → hw_error_bits / hw_fatal_bits 집계.
 *         MARSHAL_PUBLISH 가 reg.sys.hw_error / hw_fatal 로 발행.
 */
static void RD_SYSTEM_EVALUATE_STATE(void)
{
    /* 레벨 트리거: 매 tick error/fatal 비트를 0 으로 리셋 후 현재 채널 상태로 재계산.
     * → 채널이 RUNNING 으로 자동 복구되면 hw_error/hw_fatal 비트도 자동 해제 (sticky 방지).
     *   (hw.reset 비트는 별도 용도이므로 여기서 건드리지 않음) */
    hw.error.raw = 0;
    hw.fatal.raw = 0;

    STATE_t s;

    //s = PERIPHERAL.err.i2c.state;
    //if (s.bits.health    >= HC_THRESHOLD_WARN) hw.error.bit.i2c = 1;
    //if (s.bits.lifecycle == LS_OFFLINE)        hw.fatal.bit.i2c = 1;

    s = DPCB_uart2.error.state; 			// dpc
    if (s.bits.health    >= HC_THRESHOLD_WARN) hw.error.bit.uart2 = 1;
    if (s.bits.lifecycle == LS_OFFLINE)        hw.fatal.bit.uart2 = 1;

    s = DPCA_uart4.error.state;				//
    if (s.bits.health    >= HC_THRESHOLD_WARN) hw.error.bit.uart4 = 1;
    if (s.bits.lifecycle == LS_OFFLINE)        hw.fatal.bit.uart4 = 1;

    s = DPCB_uart6.error.state; 	//
    if (s.bits.health    >= HC_THRESHOLD_WARN) hw.error.bit.uart6 = 1;
    if (s.bits.lifecycle == LS_OFFLINE)        hw.fatal.bit.uart6 = 1;
}



/* RD_SYSTEM_INIT ------------------------------------------------------------*/

void RD_SYSTEM_INIT(void)
{
    /*==========TIM5 시작 (32-bit us 타이머)==========*/
    HAL_TIM_Base_Start(&htim5);

    /*==========COMM INIT==========*/
    if (RD_UART_INIT(&DPCA_uart4, &huart4) != RET_OK) Error_Handler();
    RD_PACKET_INIT(&DPCA_PACKET);

    /*==========GPIO/PERI INIT==========*/
    RD_PERIPHERAL_INIT(&DPCB_PERIPHERAL);

    /*==========레지스터 맵 INIT==========*/
    RD_MAP_INIT();
    RD_ORIN_INIT(&ORIN_PACKET);

    HAL_Delay(1000);
    HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
}


/* Task implementations ------------------------------------------------------*/

void RD_TASK_DEFAULT(void)
{
    //uint32_t id_num = 0;
    static uint8_t cnt = 0;
    //sw1 is mode, sw2 is lock&state trans

    for (;;)
    {
    	//LED1
    	switch (DPC_CTL.MODE) {
    	case 0:
    		if (DPCB_PERIPHERAL.EN_ALL == 1) {DPCB_PERIPHERAL.PANEL.LED2_state = 1;}
    		else{DPCB_PERIPHERAL.PANEL.LED2_state = 0;}


    		if (cnt == 0 || cnt == 2) {DPCB_PERIPHERAL.PANEL.LED1_state = 1;} //blink
    		else {DPCB_PERIPHERAL.PANEL.LED1_state = 0;}
    		break;
    	case 1:
    		switch (DPC_CTL.STATE) {
    		case 0:
    			DPCB_PERIPHERAL.PANEL.LED2_state = 0;
    			break;
    		case 1:
    			DPCB_PERIPHERAL.PANEL.LED2_state = 1;
    			break;
    		case 5:
    			if (cnt == 0 || cnt == 2) {DPCB_PERIPHERAL.PANEL.LED2_state = 1;} //blink
    			else {DPCB_PERIPHERAL.PANEL.LED2_state = 0;}
    			break;
    		default :
    			if (cnt == 0) {DPCB_PERIPHERAL.PANEL.LED2_state = 1;} //blink
    			else {DPCB_PERIPHERAL.PANEL.LED2_state = 0;}
    			break;
    		}

    		DPCB_PERIPHERAL.PANEL.LED1_state = 1; //on
    		break;
    	default :
    		DPCB_PERIPHERAL.PANEL.LED1_state = 0;
    	}


    	cnt++;
    	if(cnt == 10){
    		cnt = 0;
    	}
    	osDelay(100);


    	/*
        switch (DPC_CTL.STATE) {
            case 0:
                id_num = 0;
                HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
                break;
            case 4:
                id_num = 3;
                break;
            case 10:
                id_num = 0;
                HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
                break;
            default:
                id_num = 5;
                break;
        }

        for (int i = 0; i < (int)id_num; i++) {
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
            osDelay(100);
            HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_RESET);
            osDelay(100);
        }
        osDelay(1000 - (2 * id_num * 100));
        */
    }

}

void RD_TASK_SYSTEM(void) {
    uint32_t wake = osKernelGetTickCount();

    for (;;)
    {
        wake += 10;
        osDelayUntil(wake);

        tim_cnt = __HAL_TIM_GET_COUNTER(&htim5);

        RD_SYSTEM_HW_RESET_HANDLE();
        RD_SYSTEM_CHECKER();
        RD_SYSTEM_EVALUATE_STATE();

        if (payload_state == SYS_STATE_FAULT) ACTION_STATE_FAULT();

        RD_MAP_MARSHAL_PUBLISH(&DPCB_PERIPHERAL);
        //RD_MAP_MARSHAL_CONSUME(&DPCB_PERIPHERAL); //주석
    }
}

void RD_TASK_CONTROL(void)
{
    for (;;)
    {
        RD_CONTROL_LOOP(&DPC_CTL, &DPCB_PERIPHERAL);
        osDelay(10);
    }
}

void RD_TASK_RS485(void)
{
    /* ── 초기화 ──────────────────────────────────
     * Error_Handler() 는 __disable_irq() + while(1) 이라 **보드 전체가 언다** — 그러면
     * 펌웨어 결함과 배선/트랜시버 문제가 겉으로 구분되지 않는다. 치명 처리는 유지하되
     * 관측 가능한 형태로 바꾼다: 재시도 횟수를 남기고(Live Watch), 반복 실패 시 리셋. */
    while (RD_RS485_INIT(&DPCB_rs485, &huart2) != RET_OK) {
        rs485_init_fail_cnt++;
        if (rs485_init_fail_cnt >= 10) NVIC_SystemReset();
        osDelay(100);
    }
    DPCB_uart2.wake_task = rs485TaskHandle;  /* IDLE ISR → rs485Task 깨우기 */

    /* ── 이벤트 루프 ─────────────────────────────*/
    for (;;)
    {
        /* USART2 IDLE ISR 가 0x0001 플래그 set → 여기서 깨어남.
         * timeout 10ms: 패킷 이벤트가 없어도(Orin 미사용, enable_dpc_read=false) 주기 기상해
         * reg 발행을 유지하고, 기상 경로가 한 번 끊겨도 폴링으로 자기치유한다.
         * rx_new 없으면 RD_ORIN_READ 가 즉시 RET_WAIT 라 폴링 비용은 무시 수준. */
        osThreadFlagsWait(0x0001, osFlagsWaitAny, 10);

        /* 요청 직전 발행 (request-synchronous snapshot): 응답이 항상 요청 시점 스냅샷이
         * 되어 발행 주기 vs 요청 주기의 비트(beat)로 생기던 중복/스테일 샘플이 없어진다.
         * systemTask 의 주기 발행은 그대로 둔다 — 패널/DPC-A 등 내부 소비자가 따로 있다. */
        RD_MAP_MARSHAL_PUBLISH(&DPCB_PERIPHERAL);

        /* 파싱 실패(헤더/CRC/ID 불일치) 시 다음 패킷 대기 */
        if (RD_ORIN_READ(&DPCB_rs485, &ORIN_PACKET) != RET_OK) continue;

        /* CMD_MOT 영역 쓰기 잠금: AUTO 모드일 때만 해제 */
        uint8_t mtr_lock;
        taskENTER_CRITICAL();
        mtr_lock = (payload_state == SYS_STATE_AUTO) ? 0u : 1u;
        taskEXIT_CRITICAL();

        RD_ORIN_HANDLE(&ORIN_PACKET, mtr_lock);
        RD_RET wr = RD_ORIN_WRITE(&DPCB_rs485, &ORIN_PACKET);

        /* REBOOT 명령: 응답 DMA TX 가 실제로 나간 뒤 리셋 (응답 유실 방지).
         * gState == READY = DMA 전송 완료, +2ms 는 마지막 바이트 shift-out 여유. */
        if (ORIN_PACKET.reboot_pending) {
            ORIN_PACKET.reboot_pending = 0;
            if (wr == RET_OK) {
                uint32_t t0 = osKernelGetTickCount();
                while (DPCB_rs485.uart_obj->huart->gState != HAL_UART_STATE_READY &&
                       (osKernelGetTickCount() - t0) < 50) {
                    osDelay(1);
                }
                osDelay(2);
                NVIC_SystemReset();
            }
        }
    }
}

void RD_TASK_I2C(void)
{
    for (;;)
    {
        RD_EXIO_UPDATE(&DPCB_PERIPHERAL.PANEL);
        osDelay(10);
    }
}

void RD_TASK_DPCA(void)
{
    for (;;)
    {
        RD_PACKET_WRITE(&DPCA_uart4, &DPCA_PACKET);

        RD_RET comm_state = RET_WAIT;
        uint32_t comm_cnt = 0;
        while (comm_state != RET_OK && comm_cnt <= 10) {
            osDelay(1);
            comm_state = RD_PACKET_READ(&DPCA_uart4, &DPCA_PACKET);
            if (comm_state == RET_OK) {
                HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
            } else if (comm_state == RET_NOK) {
                HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);
            }
            comm_cnt++;
        }

        RD_DPCA_UPDATE(&DPCB_PERIPHERAL, &DPCA_PACKET);

        osDelay(10);
    }
}

void RD_TASK_PERI(void)
{
    /* ── 초기화 ──────────────────────────────────*/
    if (RD_RS485_INIT(&DPCB_dyn, &huart6) != RET_OK) Error_Handler();
    DPCB_uart6.wake_task = periTaskHandle;   /* IDLE ISR → periTask 깨우기 */
    osDelay(10);

    for (int i = 0; i < DYN_NUM_MOTORS; i++) {
        if (RD_DYN_INIT(&DPCB_PERIPHERAL.MOT[i].dyn_ctrl, DPCB_PERIPHERAL.MOT[i].DYN_IDS) != RET_OK) Error_Handler();
        for (int j = 0; j < DYN_NUM_MOTORS; j++)
            if (RD_DYN_INIT_SET(&DPCB_dyn, &DPCB_PERIPHERAL.MOT[i].dyn_ctrl) != RET_WAIT) break;
    }

    /*─────────Torque disable─────────*/
    for (int i = 0; i < DYN_NUM_MOTORS; i++) {
        RD_DYN_TORQUE_ON(&DPCB_PERIPHERAL.MOT[i].dyn_ctrl, 0);
        RD_DYN_LOOP(&DPCB_dyn, &DPCB_PERIPHERAL.MOT[i].dyn_ctrl);
    }

    /*──────Write Target current──────*/
    for (int i = 0; i < DYN_NUM_MOTORS; i++) {
        DPCB_PERIPHERAL.MOT[i].dyn_ctrl.inst           = INST_WRITE;
        DPCB_PERIPHERAL.MOT[i].dyn_ctrl.addr.start     = DYN_ADDR_GOAL_CURRENT;
        DPCB_PERIPHERAL.MOT[i].dyn_ctrl.addr.size      = DYN_SIZE_GOAL_CURRENT;
        DPCB_PERIPHERAL.MOT[i].dyn_ctrl.ram.cmd.goal_current = 750; /* 2.69 [mA/U] */

        RD_DYN_LOOP(&DPCB_dyn, &DPCB_PERIPHERAL.MOT[i].dyn_ctrl);
        DPCB_PERIPHERAL.MOT[i].dyn_present_tick =
        DPCB_PERIPHERAL.MOT[i].dyn_ctrl.ram.state.realtime_tick;

        osDelay(10);
    }

    /*─────────Torque active─────────*/
    for (int i = 0; i < DYN_NUM_MOTORS; i++) {
        RD_DYN_TORQUE_ON(&DPCB_PERIPHERAL.MOT[i].dyn_ctrl, 1);
        RD_DYN_LOOP(&DPCB_dyn, &DPCB_PERIPHERAL.MOT[i].dyn_ctrl);
    }

    /* ── 무한 루프 ─────────────────────────────────*/
    for (;;)
    {
        uint32_t start_tick = osKernelGetTickCount();

        for (int i = 0; i < DYN_NUM_MOTORS; i++)
        {
            DYN_Ctrl_t *dyn = &DPCB_PERIPHERAL.MOT[i].dyn_ctrl;

            /* 1) Present 상태 읽기 (realtime_tick ~ present_temperature, 27B) */
            RD_DYN_UPDATE_STATE(dyn);
            RD_DYN_LOOP(&DPCB_dyn, dyn);

            /* 2) Hardware Error Status(addr 70) 읽기 */
            RD_DYN_UPDATE_HWERROR(dyn);
            RD_DYN_LOOP(&DPCB_dyn, dyn);

            /* 3) Goal 명령(full block, GOAL_CURRENT~GOAL_POSITION 18B) 쓰기 */
            RD_DYN_UPDATE_CMD(dyn, DYN_MODE_CUR_POSITION);
            RD_DYN_LOOP(&DPCB_dyn, dyn);

            /* 4) Operating Mode 설정 (모드 변경 시에만 실제 전송) */
            RD_DYN_OPERATE_ON(dyn, DYN_MODE_CUR_POSITION);
            RD_DYN_LOOP(&DPCB_dyn, dyn);

            /* 5) Torque ON (꺼져 있을 때만 실제 전송) */
            RD_DYN_TORQUE_ON(dyn, 1);
            RD_DYN_LOOP(&DPCB_dyn, dyn);

            DPCB_PERIPHERAL.MOT[i].LPF_CURRENT =
                DPCB_PERIPHERAL.MOT[i].LPF_CURRENT * 0.95f +
                dyn->ram.state.present_current * 0.05f;
        }

        Diff_tick = osKernelGetTickCount() - start_tick;

        uint32_t nowTick = HAL_GetTick();
        DPCB_PERIPHERAL.deltaTick = nowTick - DPCB_PERIPHERAL.oldTick;
        DPCB_PERIPHERAL.oldTick   = nowTick;
        RD_PERIPHERAL_READ(&DPCB_PERIPHERAL);
        RD_PERIPHERAL_WRITE(&DPCB_PERIPHERAL);

        osDelay(1);
    }
}


/* HAL Callbacks -------------------------------------------------------------*/

/**
 * @brief HAL UART 에러 콜백 — ISR 컨텍스트.
 *        raw HAL 에러코드만 |= 누적 캡처.
 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART4)   DPCA_uart4.error.isr_err_code  |= HAL_UART_GetError(huart);
    if (huart->Instance == USART6)  DPCB_uart6.error.isr_err_code  |= HAL_UART_GetError(huart);
    if (huart->Instance == USART2)  DPCB_uart2.error.isr_err_code  |= HAL_UART_GetError(huart);
}
