/*
 * rd_system.c
 *
 *  Created on: 2026. 2. 24.
 *      Author: Lenovo
 */

/* Includes ------------------------------------------------------------------*/
#include "rd_system.h"
#include "rd_control.h"
#include "rd_can_motor.h"
#include "rd_i2c_encoder.h"
#include <string.h>

/* Exported variables ---------------------------------------------------------*/
uint32_t tim_cnt = 0;

volatile uint8_t  fatal_uart1_cnt = 0;
volatile uint8_t  fatal_uart6_cnt = 0;
volatile uint8_t  fatal_rs485_cnt = 0;
volatile uint8_t  fatal_can1_cnt = 0;

/* IWDG heartbeat — controlTask 가 매 루프 증가. systemTask 가 이 값이 진행했을 때만 IWDG refresh.
 * controlTask(모터 TX) 나 systemTask(감시) 둘 중 하나라도 hang → refresh 중단 → ~500ms 후 IWDG 리셋. */
volatile uint32_t hb_control = 0;

/* 모드 단일 진실원천 = reg.cmd_system.mode (GPIO 토글·Orin write 양쪽이 갱신). 1=AUTO / 0=MANUAL */
#define MODE_STATE() (reg.cmd_system.mode ? SYS_STATE_AUTO : SYS_STATE_MANUAL)

/*-----------CLASS Object ---------- */
LED_STATE_e LED_G_state = LED_BLINK_500;
LED_STATE_e LED_R_state = LED_RESET;

volatile SYSTEM_STATE_e robot_state = SYS_STATE_INIT;  /* systemTask + controlTask 공유 → volatile */

HW_ERROR_FLAG_t hw = {0};
/*========== UART1 (RC 수신기) ==========*/
UART_Ring_t ECU_uart1;
/*========== UART6 (IMU) ==========*/
UART_Ring_t ECU_uart6;

/*========== UART2 (RS485) ==========*/
UART_Ring_t ECU_uart2;
RS485_t 	ECU_rs485;

IMU_comm_t     ECU_imu;
PACKET_comm_t  ECU_PACKET;
RECEIVE_comm_t ECU_receive;
PERIPHERAL_t ECU_PERIPHERAL;

/* Exported function prototypes -----------------------------------------------*/
static void RD_LED_BLINK(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin, LED_STATE_e led_state, uint32_t* last_tick);
static void CAN_AK_ESTOP(float break_current);

static uint8_t RD_MOTOR_FAULT_ACTIVE(void);
static uint8_t RD_CAN_LINK_DOWN(void);

static void fatal_cnt_plus(volatile uint8_t *cnt);
static void fatal_cnt_minu(volatile uint8_t *cnt);

static void ACTION_STATE_INIT(void);
static void ACTION_STATE_AUTO(void);
static void ACTION_STATE_ESTOP_HW(void);
static void ACTION_STATE_ESTOP_SW(void);
static void ACTION_STATE_FAULT(void);
static void ACTION_STATE_MANUAL(void);

static void RD_SYSTEM_CHECKER(void);
static void RD_SYSTEM_HW_RESET_HANDLE(void);
static void RD_SYSTEM_UPDATE_STATE(STATE_t state);
static void RD_SYSTEM_EVALUATE_STATE(void);

static void RD_IWDG_START(void);
static inline void RD_IWDG_REFRESH(void) { IWDG->KR = 0x0000AAAAU; }

/* Private Function code ------------------------------------------------------*/
/* ── IWDG (독립 워치독) — HAL 모듈 미포함이라 레지스터 직접 제어 ──────────────
 *  LSI 32kHz / prescaler 64 = 500Hz (2ms/tick), reload 250 → ~500ms 타임아웃.
 *  start 후에는 정지 불가 → systemTask 가 heartbeat 조건 만족 시에만 refresh. */
static void RD_IWDG_START(void) {
	__HAL_DBGMCU_FREEZE_IWDG();
	IWDG->KR  = 0x0000CCCCU;   /* IWDG enable (LSI 자동 기동) */
	IWDG->KR  = 0x00005555U;   /* PR/RLR 쓰기 허용 */
	IWDG->PR  = 0x04U;         /* prescaler /64 */
	IWDG->RLR = 250U;          /* 250 × 2ms ≈ 500ms */
	/* PVU/RVU 갱신 완료 대기 (LSI 안정화 전 무한 spin 방지 위해 bound) */
	for (volatile uint32_t t = 0; (IWDG->SR != 0U) && (t < 100000U); t++) { }
	IWDG->KR  = 0x0000AAAAU;   /* 초기 refresh */
}

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

/* 모터 자체 fault: 어느 모터든 error_code != 0 (과열/과전류/락업 등) 또는 temp >= AK_TEMP_WARN.
 * data_mtr 는 systemTask tick 시작부(CHECKER/PERIPHERAL_READ)에서 이미 갱신됨. */
static uint8_t RD_MOTOR_FAULT_ACTIVE(void) {
	if (ECU_PERIPHERAL.data_mtr.error_code != 0) return 1;
	for (int i = 0; i < NUM_AK_MOTORS; i++) {
		if (ECU_PERIPHERAL.data_mtr.temp[i] >= AK_TEMP_WARN) return 1;
	}
	return 0;
}

/* CAN 링크 단절: 4 모터 종합 lifecycle == LS_OFFLINE. */
static uint8_t RD_CAN_LINK_DOWN(void) {
	return (ECU_PERIPHERAL.err.can.state.bits.lifecycle == LS_OFFLINE) ? 1 : 0;
}

static void fatal_cnt_plus(volatile uint8_t *cnt) { *cnt = (*cnt + FATAL_K > FATAL_MAX) ? FATAL_MAX : *cnt + FATAL_K; }
static void fatal_cnt_minu(volatile uint8_t *cnt) { if (*cnt > 0) (*cnt)--; }

static void ACTION_STATE_INIT(void) {
	ECU_PERIPHERAL.data.motor_on       = 0;
	ECU_PERIPHERAL.data.ESTOP_override = 0;
}

static void ACTION_STATE_MANUAL(void) {
	ECU_PERIPHERAL.data.ESTOP_override = 0;
	/* RC 채널이 RUNNING/DEGRADED 이고 RX 가 stale 이 아닐 때만 motor_on.
	 * health==HC_TIMEOUT(무수신 500ms 초과)이면 receive_flag/thrr/diff 가 stale 이므로
	 * lifecycle 이 아직 OFFLINE 이 아니더라도 motor_on=0 으로 강제 (stale 명령 구동 방지).
	 * CAN 링크 단절(OFFLINE) 시에도 motor_on=0 (TX 무의미 + 안전). */
	STATE_t st = ECU_uart1.error.state;
	uint8_t lc = st.bits.lifecycle;
	uint8_t rc_ok = ((lc == LS_RUNNING || lc == LS_DEGRADED) &&
	                 st.bits.health != HC_TIMEOUT) ? 1 : 0;

	ECU_PERIPHERAL.data.motor_on =
		(rc_ok && ECU_receive.receive_flag && !RD_CAN_LINK_DOWN()) ? 1 : 0;

	/* MANUAL: RC 스틱 입력(thrr/diff/selector) → reg.cmd_motor 매핑 후 CONSUME.
	 * reg 를 단일 source 로 유지하고 reg.cmd_motor → cmd_mtr 순서를 보장. */
	RD_CONTROL_RC_TO_REGISTER(&ECU_receive, &reg.cmd_motor, &reg.cmd_system);
}

static void ACTION_STATE_AUTO(void) {
	/* Orin soft ESTOP (addr 189): ACTIVE(0) 면 FSM 전이 없이 AUTO 상태 안에서
	 * CAN_AK_ESTOP 소프트 제동 수행 (ESTOP_SW 와 동일한 BREAK_CURRENT_SW).
	 * 해제(1) 시 아래 정상 경로의 ESTOP_override=0 으로 자동 복귀. */
	taskENTER_CRITICAL();
	uint8_t soft_estop = reg.cmd_system.soft_estop;
	taskEXIT_CRITICAL();
	if (soft_estop == SOFT_ESTOP_ACTIVE) {
		CAN_AK_ESTOP(BREAK_CURRENT_SW);
		/* 제동 중 잔여 cmd_vel 클리어 — 해제 직후 LPF 가 0 부터 시작해 튐 방지 */
		taskENTER_CRITICAL();
		reg.cmd_system.cmd_lin_vel = 0.0f;
		reg.cmd_system.cmd_ang_vel = 0.0f;
		taskEXIT_CRITICAL();
		return;
	}

	ECU_PERIPHERAL.data.ESTOP_override = 0;

	STATE_t st = ECU_uart2.error.state;
	uint8_t lc = st.bits.lifecycle;
	uint8_t rc_ok = ((lc == LS_RUNNING || lc == LS_DEGRADED) &&
	                 st.bits.health != HC_TIMEOUT) ? 1 : 0;

	taskENTER_CRITICAL();
	if (osKernelGetTickCount() - reg.diag.cmd_write_tick > AUTO_TIMEOUT) rc_ok = 0;
	taskEXIT_CRITICAL();

	ECU_PERIPHERAL.data.motor_on = (rc_ok && !RD_CAN_LINK_DOWN()) ? 1 : 0;
}

static void ACTION_STATE_ESTOP_HW(void) { CAN_AK_ESTOP(BREAK_CURRENT_HW); }
static void ACTION_STATE_ESTOP_SW(void) { CAN_AK_ESTOP(BREAK_CURRENT_SW); }

static void ACTION_STATE_FAULT(void) {
	ECU_PERIPHERAL.data.motor_on       = 0;
	ECU_PERIPHERAL.data.ESTOP_override = 0;

#ifdef RS485_TEST_ON
#else
	if (hw.reset.bit.uart2) RD_REBOOT_HANDLE();
#endif
	if (hw.reset.bit.can) {
		// 상위 단( ORIN에서 REBOOT 할 때까지 FAULT 상태 유지
	}
}

static void RD_SYSTEM_CHECKER(void) {
  uint8_t lc;
  /* ── CAN1 (AK 모터) ──
   * FAULT 상태에서는 ACTION_STATE_FAULT 가 CAN 복구를 단독 소유 → 여기선 skip (이중 복구 방지).
   * 그 외에는 매 tick auto-recovery 수행. 지속 복구 실패(fatal_can1_cnt>FATAL_MAX) → FAULT escalation. */
  lc = ECU_PERIPHERAL.err.can.state.bits.lifecycle;
  if (robot_state != SYS_STATE_FAULT && lc != LS_RECOVERING) {
	  if (RD_CAN_MOTOR_CHECKER(&ECU_PERIPHERAL.data_mtr, &ECU_PERIPHERAL.err) == RET_NOK){
		  fatal_cnt_plus(&fatal_can1_cnt);
		  if (RD_CAN_MOTOR_RECOVERY(&ECU_PERIPHERAL, &ECU_PERIPHERAL.err) == RET_NOK)
			  fatal_cnt_plus(&fatal_can1_cnt);
		  if (fatal_can1_cnt >= FATAL_MAX) {
			  hw.reset.bit.can = 1;
			  robot_state = SYS_STATE_FAULT;
		  }
	  } else fatal_cnt_minu(&fatal_can1_cnt);
  }

  /* ── UART2 (RS485) ── */
  lc = ECU_rs485.uart_obj->error.state.bits.lifecycle;
  if (lc != LS_RECOVERING) {
	  if (RD_RS485_CHECKER(&ECU_rs485, DEGRADED_K_100HZ) == RET_NOK){
		  fatal_cnt_plus(&fatal_rs485_cnt);
		  if (RD_RS485_RECOVERY(&ECU_rs485) == RET_NOK)
			  fatal_cnt_plus(&fatal_rs485_cnt);
		  if (fatal_rs485_cnt >= FATAL_MAX) {
			  hw.reset.bit.uart2 = 1;
			  robot_state = SYS_STATE_FAULT;
		  }
	  } else fatal_cnt_minu(&fatal_rs485_cnt);
  }

  /* ── UART1 (RC) ── */
  lc = ECU_uart1.error.state.bits.lifecycle;
  if (lc != LS_RECOVERING) {
	  if (RD_UART_CHECKER(&ECU_uart1, DEGRADED_K_100HZ) == RET_NOK){
		  fatal_cnt_plus(&fatal_uart1_cnt);
		  if (RD_UART_RECOVERY(&ECU_uart1) == RET_NOK)
			  fatal_cnt_plus(&fatal_uart1_cnt);
		  if (fatal_uart1_cnt >= FATAL_MAX) {
			  /* 상위 단에 Need Reset 요청 (addr54 발행) — Orin 이 addr5 WRITE 시
			   * RD_SYSTEM_HW_RESET_HANDLE 이 실제 복구 + 양쪽 플래그 클리어 */
			  hw.reset.bit.uart1 = 1;
			  ECU_uart1.error.state.bits.lifecycle = LS_RECOVERING;
		  }
	  } else fatal_cnt_minu(&fatal_uart1_cnt);
  }

  /* ── UART6 (IMU) — UART1 과 동일 규칙 (텔레메트리 채널: FAULT escalation 없이 reset 요청만) ── */
  lc = ECU_uart6.error.state.bits.lifecycle;
  if (lc != LS_RECOVERING) {
	  if (RD_UART_CHECKER(&ECU_uart6, DEGRADED_K_100HZ) == RET_NOK){
		  fatal_cnt_plus(&fatal_uart6_cnt);
		  if (RD_UART_RECOVERY(&ECU_uart6) == RET_NOK)
			  fatal_cnt_plus(&fatal_uart6_cnt);
		  if (fatal_uart6_cnt >= FATAL_MAX) {
			  hw.reset.bit.uart6 = 1; // Checker는 금지 상위 단에 Need Reset 요청
			  ECU_uart6.error.state.bits.lifecycle = LS_RECOVERING;
		  }
	  } else fatal_cnt_minu(&fatal_uart6_cnt);
  }
}

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

	if (req.bit.uart1) { RD_UART_RECOVERY(&ECU_uart1);  fatal_uart1_cnt = 0; }
	if (req.bit.uart2) { RD_RS485_RECOVERY(&ECU_rs485); fatal_rs485_cnt = 0; }
	if (req.bit.uart6) { RD_UART_RECOVERY(&ECU_uart6);  fatal_uart6_cnt = 0; }
	if (req.bit.can)   { RD_CAN_MOTOR_RECOVERY(&ECU_PERIPHERAL, &ECU_PERIPHERAL.err); fatal_can1_cnt = 0; }
	if (req.bit.i2c)   { RD_I2C_ENCODER_RECOVERY(&hi2c1, &ECU_PERIPHERAL.err); }

	taskENTER_CRITICAL();
	hw.reset.raw        &= (uint8_t)~req.raw;  /* addr 54 (MARSHAL_PUBLISH 가 발행) */
	reg.reg_df.hw_reset &= (uint8_t)~req.raw;  /* addr 5  (Orin 요청 플래그)        */
	taskEXIT_CRITICAL();
}

static void RD_SYSTEM_UPDATE_STATE(STATE_t state) {
	if (robot_state == SYS_STATE_FAULT) return;

	/* GPIO MODE 스위치 >5s 홀드 → RET_WAIT 반환. CAN abort 후 안전 리셋. */
	if (RD_PERIPHERAL_READ(&ECU_PERIPHERAL) == RET_WAIT) RD_REBOOT_HANDLE();

	/* GPIO 스위치 토글 요청 → reg.cmd_system.mode 반전 (현재 모드의 반대로 change). */
	if (ECU_PERIPHERAL.data.MODE_TOGGLE) {
		ECU_PERIPHERAL.data.MODE_TOGGLE = 0;
		taskENTER_CRITICAL();
		reg.cmd_system.mode = reg.cmd_system.mode ? 0 : 1;
		taskEXIT_CRITICAL();
	}
	/* IND LED 표시용 mirror — 실제 모드(reg) 를 data.MODE 로 반영. */
	ECU_PERIPHERAL.data.MODE = reg.cmd_system.mode;

	/* 모터 자체 fault(과열/과전류/락업/temp>=warn) → 소프트 ESTOP. 해소 시 자동 복귀. */
	uint8_t motor_fault = RD_MOTOR_FAULT_ACTIVE();

	if (ECU_PERIPHERAL.data.ESTOP || ECU_PERIPHERAL.data.MODE_DONE) {
		robot_state = SYS_STATE_ESTOP_HW;
	} else if (robot_state == SYS_STATE_ESTOP_HW) {
		robot_state = (state.bits.health == HC_HW_FAULT || motor_fault) ? SYS_STATE_ESTOP_SW : MODE_STATE();
	} else if (robot_state == SYS_STATE_ESTOP_SW) {
		/* 모터 fault 가 해소되고 CAN health 도 경고 미만일 때만 정상 모드 복귀 (자동 recovery). */
		if (!motor_fault && state.bits.health < HC_THRESHOLD_WARN) robot_state = MODE_STATE();
	} else {
		robot_state = motor_fault ? SYS_STATE_ESTOP_SW : MODE_STATE();
	}
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
	s = ECU_PERIPHERAL.err.can.state;
    if (s.bits.health    >= HC_THRESHOLD_WARN) hw.error.bit.can = 1;
    if (s.bits.lifecycle == LS_OFFLINE)        hw.fatal.bit.can = 1;

    s = ECU_PERIPHERAL.err.i2c.state;
    if (s.bits.health    >= HC_THRESHOLD_WARN) hw.error.bit.i2c = 1;
    if (s.bits.lifecycle == LS_OFFLINE)        hw.fatal.bit.i2c = 1;

    s = ECU_uart1.error.state;
    if (s.bits.health    >= HC_THRESHOLD_WARN) hw.error.bit.uart1 = 1;
    if (s.bits.lifecycle == LS_OFFLINE)        hw.fatal.bit.uart1 = 1;

    s = ECU_uart2.error.state;
    if (s.bits.health    >= HC_THRESHOLD_WARN) hw.error.bit.uart2 = 1;
    if (s.bits.lifecycle == LS_OFFLINE)        hw.fatal.bit.uart2 = 1;

    s = ECU_uart6.error.state;
    if (s.bits.health    >= HC_THRESHOLD_WARN) hw.error.bit.uart6 = 1;
    if (s.bits.lifecycle == LS_OFFLINE)        hw.fatal.bit.uart6 = 1;
}

/* Private Function code ------------------------------------------------------*/
void RD_SYSTEM_INIT(void) {
  HAL_TIM_Base_Start_IT(&htim5);
  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
  HAL_Delay(1000);
  /*==========UART INIT==========*/
  /* RS485 핸들에 backing UART 링버퍼 연결 — 이 연결이 없으면 uart_obj == NULL 로 아래에서 HardFault */
  ECU_rs485.uart_obj = &ECU_uart2;
  ECU_uart1.error.state.raw           = LS_INIT;
  ECU_uart6.error.state.raw           = LS_INIT;
  ECU_rs485.uart_obj->error.state.raw = LS_INIT;
  /*==========COMM INIT==========*/
  RD_RECEIVE_INIT(&ECU_receive);
  RD_IMU_INIT(&ECU_imu);
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
  uint32_t last_r_tick = osKernelGetTickCount();
  uint32_t last_g_tick = osKernelGetTickCount();
  for(;;)
  {
	RD_LED_BLINK(LED_R_GPIO_Port, LED_R_Pin, LED_R_state, &last_r_tick);
	RD_LED_BLINK(LED_G_GPIO_Port, LED_G_Pin, LED_G_state, &last_g_tick);
	osDelay(50);
  }
}

void RD_TASK_SYSTEM(void) {
  uint32_t tick = osKernelGetTickCount();
  uint32_t hb_control_last = 0;
  RD_IWDG_START();   /* 스케줄러 시작 후 기동 — RD_SYSTEM_INIT 의 HAL_Delay 로 인한 오리셋 회피 */
  for(;;)
  {
	RD_SYSTEM_HW_RESET_HANDLE();   /* Orin addr5 리셋 요청 우선 처리 (처리 후 Checker 가 재평가) */
	RD_SYSTEM_CHECKER();
	RD_SYSTEM_EVALUATE_STATE();
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
	/* IWDG refresh — controlTask 가 직전 tick 이후 진행했을 때만(=살아있을 때만).
	 * controlTask hang → hb 정체 → refresh 중단 → IWDG 리셋. systemTask 자신은 이 루프가
	 * 도는 것 자체가 liveness 이므로 별도 검사 불필요. */
	uint32_t hc = hb_control;
	if (hc != hb_control_last) {
		RD_IWDG_REFRESH();
		hb_control_last = hc;
	}

	tick += 10;
	osDelayUntil(tick);
  }
}

void RD_TASK_CONTROL(void) {
  // RESISTER -> PERIPHERAL
  uint32_t tick = osKernelGetTickCount();
  static SYSTEM_STATE_e prev_state = SYS_STATE_INIT;
  for(;;)
  {
	hb_control++;   /* IWDG heartbeat — systemTask 가 liveness 확인용 */
	if (robot_state != prev_state) {
		RD_CONTROL_RESET_FILTER();
		prev_state = robot_state;
	}
	/* motor_on=0(소스 비활성) 시 RD_CONTROL_UPDATE 가 LPF·명령을 0 으로 리셋 →
	 * 재기동 시 직전 명령 잔류로 튀는 것 방지. TX skip 은 RD_PERIPHERAL_WRITE 가 담당. */
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
  if (RD_RS485_INIT(&ECU_rs485, &huart2) != RET_OK) RD_REBOOT_HANDLE();
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

		uint8_t mtr_lock = 1;
		taskENTER_CRITICAL();
		if (robot_state == SYS_STATE_AUTO) mtr_lock = 0;
		taskEXIT_CRITICAL();
		RD_PACKET_HANDLE(&ECU_PACKET, mtr_lock);

		RD_RET wr = RD_PACKET_WRITE(&ECU_rs485, &ECU_PACKET);
		if (ECU_PACKET.reboot_pending) {
			ECU_PACKET.reboot_pending = 0;
			if (wr == RET_OK) {
				/* REBOOT 응답 DMA TX 가 실제로 나간 뒤 리셋 (응답 유실 방지).
				 * gState==READY = DMA 전송 완료, +2ms 는 마지막 바이트 shift-out 여유. */
				uint32_t t0 = osKernelGetTickCount();
				while (ECU_rs485.uart_obj->huart->gState != HAL_UART_STATE_READY &&
				       (osKernelGetTickCount() - t0) < 50) {
					osDelay(1);
				}
				osDelay(2);
				RD_REBOOT_HANDLE();
			}
		}
	}else if(packet_state == RET_NOK) {
		LED_R_state = LED_BLINK_100;
	}
  }
}

void RD_TASK_IMU(void) {
  if (RD_UART_INIT(&ECU_uart6, &huart6) != RET_OK) RD_REBOOT_HANDLE();
  for(;;)
  {
#ifdef RTOS_IS_AVAILABLE
	osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);
#else
	osDelay(1);
#endif
	RD_IMU_READ(&ECU_uart6, &ECU_imu);
  }
}

void RD_TASK_RC(void) {
  if (RD_UART_INIT(&ECU_uart1, &huart1) != RET_OK) RD_REBOOT_HANDLE();
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
	uint32_t tick = osKernelGetTickCount();
	for(;;)
	{
		/* CHECKER 결과가 OFFLINE(RET_NOK)이면 자동 복구(버스 락업 포함).
		 * 인코더는 Orin 텔레메트리용이라 ESTOP 연동 불필요 — auto-recovery 만 수행. */
		if (RD_PERIPHERAL_I2C(&ECU_PERIPHERAL) == RET_NOK) {
			RD_I2C_ENCODER_RECOVERY(&hi2c1, &ECU_PERIPHERAL.err);
		}
		tick += 10;
		osDelayUntil(tick);
	}
}

uint64_t Get_Time_us(void)
{
    uint32_t current_ms;
    uint32_t current_us;
    taskENTER_CRITICAL();
    current_ms = tim_cnt;
    current_us = __HAL_TIM_GET_COUNTER(&htim5);
    if (__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_UPDATE) != RESET) {
        current_us = __HAL_TIM_GET_COUNTER(&htim5);
        current_ms++;
    }
    taskEXIT_CRITICAL();
    return ((uint64_t)current_ms * 1000) + current_us;
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
    /* ISR — lifecycle 직접 변경 금지. raw HAL 에러코드만 누적 캡처(|=) → CHECKER 가 매핑/클리어. */
    if (huart->Instance == USART2) ECU_uart2.error.isr_err_code |= HAL_UART_GetError(huart);
    if (huart->Instance == USART1) ECU_uart1.error.isr_err_code |= HAL_UART_GetError(huart);
    if (huart->Instance == USART6) ECU_uart6.error.isr_err_code |= HAL_UART_GetError(huart);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c)
{
    if (hi2c->Instance == I2C1)
    	ECU_PERIPHERAL.err.i2c.isr_err_code |= HAL_I2C_GetError(hi2c);
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance == CAN1) {
    	/* ISR — lifecycle 직접 변경 금지. HAL 누적 에러코드 캡처 → CHECKER 가 매핑. */
    	ECU_PERIPHERAL.err.can.isr_err_code |= HAL_CAN_GetError(hcan);
        HAL_CAN_ResetError(hcan);
        /* ★ 에러 IT 폭주(특히 LEC: 버스 단선/노이즈 시 매 에러프레임마다 ERRI 재발) 차단.
         *    여기서 끄지 않으면 IRQ(prio 5)가 systemTask 를 기아시켜 CHECKER/복구가 못 돌고
         *    CAN 이 멈춘다. RD_CAN_MOTOR_CHECKER 가 매 tick 재무장(ActivateNotification)하고,
         *    OFFLINE 복구 시엔 CAN_Init 이 재등록한다. RX/TX 알림은 건드리지 않아 통신은 유지. */
        HAL_CAN_DeactivateNotification(hcan,
            CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE | CAN_IT_BUSOFF |
            CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR);
    }
}
