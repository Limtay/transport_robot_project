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
#include "rd_adc.h"
#include <string.h>

/* Exported variables ---------------------------------------------------------*/
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

/* 이번 systemTask tick 의 모터 존재 판정 스냅샷 (2026-08-03).
 * 구: ACTION_MANUAL / ACTION_AUTO / UPDATE_STATE 가 각자 RD_CAN_MOTOR_ALL_READY 를 재호출 —
 * 같은 tick 안에서 세 값이 미세하게 어긋날 수 있었고, "모터 존재 판정" 이라는 개념에
 * 이름이 없어 정책이 세 군데로 흩어져 보였다. CHECKER 직후 1회 산출 → 소비자는 읽기만.
 * systemTask 단독 소유(생산·소비 모두)라 volatile 불필요.
 * TRANSMIT 의 per-motor TX 필터(READY_MASK)는 controlTask 200Hz 소유 + 다른 입도라
 * 여기에 합치지 않는다 — 드라이버 불변식으로 남겨야 상위 경로가 우회할 수 없다. */
static uint8_t motor_ready = 0;

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
 * controlTask 의 PERIPHERAL_WRITE 가 ESTOP_override 보고 BRAKE 명령 생성 후 TX.
 * motor_on=1 은 존재 게이트(ALL_READY)를 거치지 않는다 — 제동은 "구동 가능할 때만"이
 * 아니라 항상 시도해야 하기 때문. 부재 모터로 프레임이 새는 문제는 TRANSMIT 의
 * 실효 마스크(A, RD_CAN_MOTOR_READY_MASK)가 per-motor 로 막는다. */
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

	/* 존재 게이트 (H1 개정): mask 된 전 모터의 상시 피드백이 신선할 때만 구동 (motor_ready).
	 * 모터 전원이 아직 없으면 TX 미개시 → 빈 버스 ACK 폭주→FAULT (전원 순서) 원천 차단.
	 * 늦게 켜진 모터는 피드백이 보이는 즉시 자동 합류.
	 * 일부 모터만 살아있는 상태의 주행은 금지 — 차동구동에서 한 바퀴가 빠지면 직진 명령에
	 * 로봇이 돌기 때문에, TX 필터(per-motor)와 달리 여기는 all-or-nothing 이어야 한다. */
	ECU_PERIPHERAL.data.motor_on =
		(rc_ok && ECU_receive.receive_flag && !RD_CAN_LINK_DOWN() && motor_ready) ? 1 : 0;

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
	uint8_t auto_mode  = reg.cmd_system.auto_mode;
	taskEXIT_CRITICAL();
	if (soft_estop == SOFT_ESTOP_ACTIVE) {
		CAN_AK_ESTOP(BREAK_CURRENT_SW);
		/* ESTOP_override=1 세팅 → 이번 tick 의 비구동 훅이 CMD_CLEAR 수행 (해제 직후
		 * 잔여 명령 방지). 제동 TX 는 PERIPHERAL_WRITE 가 override 보고 생성. */
		return;
	}

	ECU_PERIPHERAL.data.ESTOP_override = 0;

	STATE_t st = ECU_uart2.error.state;
	uint8_t lc = st.bits.lifecycle;
	uint8_t rc_ok = ((lc == LS_RUNNING || lc == LS_DEGRADED) &&
	                 st.bits.health != HC_TIMEOUT) ? 1 : 0;

	/* AUTO 경로 분기는 전부 여기(100Hz 정책 레이어)서 끝내고 reg 에 스테이징 —
	 * controlTask 는 CONSUME→LPF→출력만 (MANUAL 의 RC_TO_REGISTER 와 대칭 구조). */
	switch (auto_mode) {
		case 0: { /* KINEMATIC: cmd_lin/ang_vel → 4모터 속도 명령 */
			float lin, ang;
			taskENTER_CRITICAL();
			lin = reg.cmd_system.cmd_lin_vel;
			ang = reg.cmd_system.cmd_ang_vel;
			taskEXIT_CRITICAL();

			float rpm_out[NUM_AK_MOTORS];
			RD_CONTROL_KINEMATICS(lin, ang, rpm_out);
			taskENTER_CRITICAL();
			for (int i = 0; i < NUM_AK_MOTORS; i++) {
				reg.cmd_motor.cmd_velocity[i] = rpm_out[i];
				reg.cmd_motor.ctr_mode[i]     = MODE_VELOCITY;
			}
			taskEXIT_CRITICAL();
			break;
		}
		case 1: { /* CURRENT: Orin cmd_current 그대로, 모드만 강제 */
			taskENTER_CRITICAL();
			for (int i = 0; i < NUM_AK_MOTORS; i++) {
				reg.cmd_motor.ctr_mode[i] = MODE_CURRENT;
			}
			taskEXIT_CRITICAL();
			break;
		}
		case 2: /* DIRECT: reg 값 무가공 통과 (주의요망) — AUTO_TIMEOUT 워치독만 적용 */
			break;
		case 4: { /* VELOCITY (2026-07-27 신규): Orin cmd_velocity 그대로, 모드만 강제.
		           * CURRENT(1) 과 대칭 — bridge write 범위는 148:16 (redesign/01 §3) */
			taskENTER_CRITICAL();
			for (int i = 0; i < NUM_AK_MOTORS; i++) {
				reg.cmd_motor.ctr_mode[i] = MODE_VELOCITY;
			}
			taskEXIT_CRITICAL();
			break;
		}
		case 5: { /* POSITION (2026-07-27 신규): Orin cmd_position 그대로, 모드만 강제.
		           * bridge write 범위는 132:16 (redesign/01 §3) */
			taskENTER_CRITICAL();
			for (int i = 0; i < NUM_AK_MOTORS; i++) {
				reg.cmd_motor.ctr_mode[i] = MODE_POSITION;
			}
			taskEXIT_CRITICAL();
			break;
		}
		case 3: /* CONTROL: TODO 미래 확장 (MPC 하위 제어 루프 등, CONTROL_UPDATE 에서 채울 예정).
		         * 현재는 안전용으로 motor off. */
			rc_ok = 0;
			break;
		default:
			rc_ok = 0;
			break;
	}

	taskENTER_CRITICAL();
	if (osKernelGetTickCount() - reg.diag.cmd_write_tick > AUTO_TIMEOUT) rc_ok = 0;
	taskEXIT_CRITICAL();

	/* 존재 게이트 (H1 개정) — MANUAL 과 동일: mask 전 모터 피드백 신선 시에만 구동 */
	ECU_PERIPHERAL.data.motor_on = (rc_ok && !RD_CAN_LINK_DOWN() && motor_ready) ? 1 : 0;
}

static void ACTION_STATE_ESTOP_HW(void) { CAN_AK_ESTOP(BREAK_CURRENT_HW); }
static void ACTION_STATE_ESTOP_SW(void) { CAN_AK_ESTOP(BREAK_CURRENT_SW); }

static void ACTION_STATE_FAULT(void) {
	ECU_PERIPHERAL.data.motor_on       = 0;
	ECU_PERIPHERAL.data.ESTOP_override = 0;

#ifdef RS485_TEST_ON
#else
	/* uart2 fatal (AUTO): 즉시 리붓 → UART2_REBOOT_DELAY_MS 유예 후 리붓 (H2).
	 * 순간 오판/과도 상태에서의 즉시 SystemReset 을 피하고, 유예 중 플래그가
	 * 해제(Orin addr5 등)되면 리붓 취소. */
	static uint32_t uart2_fault_tick = 0;
	if (hw.reset.bit.uart2) {
		if (uart2_fault_tick == 0) {
			uart2_fault_tick = osKernelGetTickCount();
		} else if (osKernelGetTickCount() - uart2_fault_tick >= UART2_REBOOT_DELAY_MS) {
			RD_REBOOT_HANDLE();
		}
	} else {
		uart2_fault_tick = 0;
	}
#endif
	if (hw.reset.bit.can) {
		// 상위 단( ORIN에서 REBOOT 할 때까지 FAULT 상태 유지
	}
}

static void RD_SYSTEM_CHECKER(void) {
  uint8_t lc;
  uint8_t motor_mask = reg.cmd_system.motor_mask;   /* 단일 byte 원자 read (스냅샷) */
  /* ── CAN1 (AK 모터) ──
   * FAULT 상태에서는 ACTION_STATE_FAULT 가 CAN 복구를 단독 소유 → 여기선 skip (이중 복구 방지).
   * 그 외에는 매 tick auto-recovery 수행. 지속 복구 실패(fatal_can1_cnt>FATAL_MAX) → FAULT escalation. */
  lc = ECU_PERIPHERAL.err.can.state.bits.lifecycle;
  if (robot_state != SYS_STATE_FAULT && lc != LS_RECOVERING) {
	  if (RD_CAN_MOTOR_CHECKER(&ECU_PERIPHERAL.data_mtr, &ECU_PERIPHERAL.err, motor_mask) == RET_NOK){
		  fatal_cnt_plus(&fatal_can1_cnt);
		  if (RD_CAN_MOTOR_RECOVERY(&ECU_PERIPHERAL, &ECU_PERIPHERAL.err) == RET_NOK)
			  fatal_cnt_plus(&fatal_can1_cnt);
		  if (fatal_can1_cnt >= FATAL_MAX) {
			  hw.reset.bit.can = 1;
			  robot_state = SYS_STATE_FAULT;
		  }
	  } else fatal_cnt_minu(&fatal_can1_cnt);
  }

  /* ── UART2 (RS485) ── (H2, failsafe_analysis_260717.md §8-P2)
   * fatal 도달 시 모드별 분기:
   *   AUTO   → FAULT (ACTION_STATE_FAULT 가 UART2_REBOOT_DELAY_MS 후 SystemReset)
   *   MANUAL → FAULT 미진입 — RC 채널과 동일하게 RECOVERING 동결 + addr54 리셋 요청만.
   *            RC 주행은 유지되고, 물리 원인 제거 후 Orin addr5 write 또는 GPIO long-hold
   *            리붓으로 해제. (동결 중 checker 는 WAIT 반환 → 재트리거 없음) */
  lc = ECU_rs485.uart_obj->error.state.bits.lifecycle;
  if (lc != LS_RECOVERING) {
	  if (RD_RS485_CHECKER(&ECU_rs485, DEGRADED_K_100HZ) == RET_NOK){
		  fatal_cnt_plus(&fatal_rs485_cnt);
		  if (RD_RS485_RECOVERY(&ECU_rs485) == RET_NOK)
			  fatal_cnt_plus(&fatal_rs485_cnt);
		  if (fatal_rs485_cnt >= FATAL_MAX) {
			  hw.reset.bit.uart2 = 1;
			  if (MODE_STATE() == SYS_STATE_AUTO) {
				  robot_state = SYS_STATE_FAULT;
			  } else {
				  ECU_rs485.uart_obj->error.state.bits.lifecycle = LS_RECOVERING;
			  }
		  }
	  } else fatal_cnt_minu(&fatal_rs485_cnt);
  } else if (hw.reset.bit.uart2 && MODE_STATE() == SYS_STATE_AUTO) {
	  /* MANUAL 동결 상태에서 AUTO 로 전환 시도 → RS485 없이는 AUTO 불가이므로
	   * FAULT 경로 합류 (3초 후 리붓으로 복구 시도). rc_ok=0 이라 그 사이에도 안전. */
	  robot_state = SYS_STATE_FAULT;
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

  /* ── 지휘 채널 전멸 판정 (2026-08-03) ──
   * MANUAL 에서 uart2 fatal 은 단독으로는 FAULT 가 아니다 (위 분기: RC 주행은 유지).
   * 그런데 그 RC(uart1)마저 살아있지 않으면 로봇을 지휘할 채널이 하나도 없는 상태 —
   * 이때는 FAULT 로 올려 ACTION_STATE_FAULT 의 uart2 경로(3초 유예 후 SystemReset)로
   * 재기동을 시도한다. 레벨 트리거라 uart2 동결 이후에 RC 가 죽는 순서도 잡힌다.
   * lc 는 UART1 블록 진입 시점(332행) 값 — 그 블록이 이번 tick 에 LS_RECOVERING 으로
   * 동결시킨 경우 반영은 다음 tick 이지만, 3초 유예 대비 10ms 지연이라 무해하다.
   * 부팅 오발동 없음: uart2 는 첫 수신 전 LS_READY 에 머물러 (rd_uart.c 4a + 타임아웃
   * 판정의 lifecycle>=LS_RUNNING 조건) fatal 로 가지 않아 hw.reset.bit.uart2 가 서지 않는다. */
  if (hw.reset.bit.uart2 && MODE_STATE() == SYS_STATE_MANUAL &&
      lc != LS_RUNNING && lc != LS_DEGRADED) {
	  robot_state = SYS_STATE_FAULT;
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

  /* ── ADC (로드셀) ── 제어 피드백 안전등급 → i2c(자가복구)와 달리 systemTask 소유(정석 패턴).
   *    CHECKER = staleness/레일 진단만. staleness(콜백 정체=DMA정지) 시 RECOVERY = Stop→Start 재기동.
   *    adcTask 는 콜백 대기로 블로킹만 하므로, 정지 감시는 여기(독립 100Hz)가 담당한다. */
  if (RD_ADC_CHECKER() == RET_NOK) {
	  RD_ADC_RECOVERY(&hadc1);
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

	/* E1 (2026-08-03) — FAULT 탈출. FAULT 를 유발하는 채널은 can / uart2 둘뿐이므로,
	 * 요청 처리 후 두 리셋 플래그가 모두 내려갔으면 정상 모드로 복귀시킨다.
	 * 구 코드는 리셋만 수행하고 robot_state 는 건드리지 않아 FAULT 탈출 수단이 리붓뿐이었다
	 * (ACTION_STATE_FAULT 의 can 분기는 비어 있고, RD_SYSTEM_CHECKER 는 FAULT 중 CAN 검사를
	 *  통째로 skip 하므로 자가 복귀 경로가 없다).
	 * 원인이 남아 있으면 다음 tick 의 CHECKER 가 다시 escalation 하므로 위험하지 않고,
	 * 주행 재개는 ALL_READY / rc_ok 게이트가 여전히 독립적으로 막는다.
	 * 자율 복귀(E2)는 채택하지 않음 — 여기까지 온 건 이미 FATAL_MAX 연속 실패라
	 * 타임아웃성이 아닌 HW 이상으로 보고, 복귀 시점은 Orin 이 판단한다. */
	if (robot_state == SYS_STATE_FAULT && !hw.reset.bit.can && !hw.reset.bit.uart2) {
		robot_state = MODE_STATE();
	}
}

static void RD_SYSTEM_UPDATE_STATE(STATE_t state) {
	/* GPIO 읽기는 FAULT 에서도 수행 (F1): MODE 스위치 >5s 홀드 리붓이 FAULT 탈출의
	 * 최후 수단 (uart2 FAULT 면 Orin REBOOT 명령도 불가) — early return 앞에 둔다.
	 * data_mtr 갱신도 함께 유지되어 FAULT 중 텔레메트리 동결 방지. */
	if (RD_PERIPHERAL_READ(&ECU_PERIPHERAL) == RET_WAIT) RD_REBOOT_HANDLE();

	if (robot_state == SYS_STATE_FAULT) return;

	/* GPIO 스위치 토글 요청 → reg.cmd_system.mode 반전 (현재 모드의 반대로 change). */
	if (ECU_PERIPHERAL.data.MODE_TOGGLE) {
		ECU_PERIPHERAL.data.MODE_TOGGLE = 0;
		taskENTER_CRITICAL();
		reg.cmd_system.mode = reg.cmd_system.mode ? 0 : 1;
		taskEXIT_CRITICAL();
	}
	/* IND LED 표시용 mirror — 실제 모드(reg) 를 data.MODE 로 반영. */
	ECU_PERIPHERAL.data.MODE = reg.cmd_system.mode;

	/* 모터 자체 fault(과열/과전류/락업/temp>=warn) → 소프트 ESTOP. 해소 시 자동 복귀.
	 * + 구동 중(motor_on) 활성(mask) 모터 피드백 상실도 동일 경로 (H1) —
	 *   전체 구성에서 모터 1개 커넥터 탈락 시 전체 제동, 통신 복구 시 자동 복귀.
	 *   비구동 시 미접촉 모터는 ACTION 의 존재 게이트가 motor_on 자체를 막아
	 *   (TX 미개시, 조용한 대기) 여기서는 fault 로 잡지 않는다.
	 *   → motor_on 조건이 "구동 중 상실"과 "아직 미도착"을 가르는 지점이므로 유지한다.
	 *   motor_ready 는 이번 tick 스냅샷 (ACTION_* 와 동일 값). */
	uint8_t motor_fault = RD_MOTOR_FAULT_ACTIVE();
	if (ECU_PERIPHERAL.data.motor_on && !motor_ready) motor_fault = 1;

	if (/*ECU_PERIPHERAL.data.ESTOP ||*/ECU_PERIPHERAL.data.MODE_DONE) {
		robot_state = SYS_STATE_ESTOP_HW;
	} else if (robot_state == SYS_STATE_ESTOP_HW) {
		robot_state = (state.bits.health == HC_HW_FAULT || motor_fault) ? SYS_STATE_ESTOP_SW : MODE_STATE();
	} else if (robot_state == SYS_STATE_ESTOP_SW) {
		/* 모터 fault 가 해소되고 CAN health 도 경고 미만일 때만 정상 모드 복귀 (자동 recovery). */
		if (!motor_fault && state.bits.health < HC_THRESHOLD_WARN) robot_state = MODE_STATE();
	} else {
		robot_state = motor_fault ? SYS_STATE_ESTOP_SW : MODE_STATE();
	}

	/* Orin 의 mode write 도 GPIO 토글과 대칭으로 ESTOP 경유 (F5): 주행 중 모드가 바뀌면
	 * 이번 tick 을 ESTOP_SW 로 강제 — 제동 1펄스 + 비구동 훅 청소 + LPF 리셋 후
	 * 다음 tick 에 정상 복귀 조건으로 새 모드 진입. GPIO 경로는 위 FSM 의 MODE_DONE 이
	 * ESTOP_HW 를 강제하므로 (robot_state 가 MANUAL/AUTO 아님) 여기서 중복 발동 없음. */
	static uint8_t prev_mode = 0;
	uint8_t cur_mode = ECU_PERIPHERAL.data.MODE;   /* 위에서 reg 미러된 값 */
	if (cur_mode != prev_mode && (robot_state == SYS_STATE_MANUAL || robot_state == SYS_STATE_AUTO)) {
		robot_state = SYS_STATE_ESTOP_SW;
	}
	prev_mode = cur_mode;
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
  HAL_TIM_Base_Start(&htim5);   /* free-run 카운터 전용 — update IRQ 불필요 (구 _IT 폐기) */
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

	/* 모터 존재 판정 1회 산출 — 이후 UPDATE_STATE / ACTION_* 가 이 값만 읽는다.
	 * motor_mask 는 uint8 단일 필드라 원자 read (rd_system.c 기존 관례와 동일). */
	motor_ready = RD_CAN_MOTOR_ALL_READY(reg.cmd_system.motor_mask);

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

	/* 비구동 훅: 구동 불가(motor_on=0) 또는 제동 중(ESTOP_override=1)이면 잔류 명령 청소.
	 * override 는 ESTOP_HW/SW·soft_estop 에서 1 — 모든 비구동 상황이 이 한 줄로 커버된다.
	 * 레벨 트리거(매 tick)라 청소 후 유입된 스테일 명령도 10ms 내 재청소.
	 * reg=0 이면 무분기 control 파이프라인이 자동으로 0 을 계산 (LPF 자연 감쇠). */
	if (!ECU_PERIPHERAL.data.motor_on || ECU_PERIPHERAL.data.ESTOP_override) {
		RD_CONTROL_CMD_CLEAR();
	}

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
  uint32_t tick = osKernelGetTickCount();
  /* LPF 전이 리셋 신호 4종 — 무분기 UPDATE 보완:
   * 비구동 구간엔 reg=0(CMD_CLEAR) 입력으로 자연 감쇠하지만, 짧은 정지(감쇠 미완) 후
   * 재개 시 잔류 속도에서 재가속하지 않도록 전이 순간엔 항상 0 에서 재시작.
   * use_lpf 0→1 상승엣지 포함 — OFF 동안 동결된 옛 필터값에서 재개하는 점프 방지
   * (MANUAL 내 전류↔속도 selector 토글은 robot_state/motor_on 이 안 변해 기존 3신호로 안 잡힘). */
  static SYSTEM_STATE_e prev_state   = SYS_STATE_INIT;
  static uint8_t        prev_estop   = SOFT_ESTOP_RELEASE;
  static uint8_t        prev_mtr_on  = 0;
  static uint8_t        prev_use_lpf = 1;
  for(;;)
  {
	hb_control++;   /* IWDG heartbeat — systemTask 가 liveness 확인용 */

	SYSTEM_STATE_e cur_state = robot_state;
	taskENTER_CRITICAL();
	uint8_t cur_estop   = reg.cmd_system.soft_estop;
	uint8_t cur_use_lpf = reg.cmd_system.use_lpf;
	uint8_t motor_mask  = reg.cmd_system.motor_mask;
	taskEXIT_CRITICAL();
	uint8_t cur_mtr_on = ECU_PERIPHERAL.data.motor_on;
	if (cur_state != prev_state || cur_estop != prev_estop ||
	    (cur_mtr_on && !prev_mtr_on) || (cur_use_lpf && !prev_use_lpf)) {
		RD_CONTROL_RESET_FILTER();
	}
	prev_state   = cur_state;
	prev_estop   = cur_estop;
	prev_mtr_on  = cur_mtr_on;
	prev_use_lpf = cur_use_lpf;

	/* 순수 제어 파이프라인 — 정지/차단은 reg(CMD_CLEAR)와 게이트(motor_on/override)가
	 * 표현하므로 상태 불문 매 tick 실행. TX skip/제동은 RD_PERIPHERAL_WRITE 담당.
	 * 데이터플로우를 태스크에서 그대로 드러냄: reg → cmd_mtr(CONSUME) → LPF → TX.
	 * use_lpf 는 위에서 CRIT 로 읽은 스냅샷을 전달 — UPDATE 내부 reg 참조 제거. */
	RD_MAP_MARSHAL_CONSUME(&ECU_PERIPHERAL);
	RD_CONTROL_UPDATE(&ECU_PERIPHERAL.cmd_mtr, cur_use_lpf);
	/* 반환값 미사용 (F4): NOK 는 NULL 인자뿐(도달 불가)이라 구 FAULT 전이는 죽은 경로였고,
	 * robot_state 쓰기는 정책 레이어(systemTask) 단독 소유 원칙 위반이라 제거. */
	RD_PERIPHERAL_WRITE(&ECU_PERIPHERAL, motor_mask);
	/* reg 발행은 rs485Task 의 요청 직전 MARSHAL_PUBLISH 로 일원화 —
	 * 구 PUBLISH_FAST(200Hz 주기 발행, 2026-07-07 bag 중복 샘플 대응)는 폐기. */
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
	/* timeout 10ms: 패킷 이벤트가 없어도(수동 모드, Orin 미사용) 주기 기상해
	 * reg 발행을 유지 — Live watch 동결 방지 + 미래의 내부 reg 소비자 대비.
	 * rx_new 없으면 RD_PACKET_READ 가 즉시 RET_WAIT 라 폴링 비용은 무시 수준. */
	osThreadFlagsWait(0x0001, osFlagsWaitAny, 10);
#else
	osDelay(1);
#endif
	/* 요청 직전 발행 (request-synchronous snapshot): 발행 주기 vs 요청 주기의
	 * 비트(beat)로 생기던 중복/스테일 샘플 문제의 일반해. 기상 사유(유효 패킷/
	 * timeout/타 노드 트래픽)와 무관하게 무조건 발행 — 유효 요청이면 발행→HANDLE
	 * 간격이 us 수준이라 응답은 항상 요청 시점 데이터가 된다.
	 * (소요 ~10us 수준 실측 diag 는 제거 — 200us 예산 대비 무해 판정) */
	RD_MAP_MARSHAL_PUBLISH(&ECU_PERIPHERAL);
	packet_state = RD_PACKET_READ(&ECU_rs485, &ECU_PACKET);
	if (packet_state == RET_OK){
		LED_R_state = LED_RESET;

		uint8_t mtr_lock = 1;
		taskENTER_CRITICAL();
		if (robot_state == SYS_STATE_AUTO) mtr_lock = 0;
		taskEXIT_CRITICAL();
		RD_PACKET_HANDLE(&ECU_PACKET, mtr_lock);

		/* 응답 TX 직전 latch: publish(realtime_tick)→TX 시작 처리시간을 diag 에 기록.
		 * 이번 응답의 reg 스냅샷은 이미 HANDLE 에서 소비됐으므로 이 값은 다음
		 * 트랜잭션 응답에 실린다 — Orin 이 소급 매칭 (testbed_spec.md §2.5). */
		reg.sys.rs485_proc_delta = rd_delta_tick(rd_now_tick(), reg.sys.realtime_tick); /* 2026-07-27 diag→sys 이동 (addr 228→32) */

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
	/* 자가 복구 지수 백오프 (H6): 라인이 계속 죽어 있으면 매 10ms 버스클리어(9클럭+수 ms)를
	 * 무한 반복하게 되므로, 연속 실패 시 재시도 간격을 100→200→400→800→1000ms(cap) 로 늘린다.
	 * UPDATE 가 정상(RET_NOK 아님)으로 돌아오면 즉시 리셋 — 복구 반응성은 유지. */
	uint32_t recovery_next_tick = 0;
	uint32_t recovery_interval  = I2C_RECOVERY_BASE_MS;
	for(;;)
	{
		/* CHECKER 결과가 OFFLINE(RET_NOK)이면 자동 복구(버스 락업 포함).
		 * 인코더는 Orin 텔레메트리용이라 ESTOP 연동 불필요 — auto-recovery 만 수행.
		 * 백오프 리셋은 완전 정상(RET_OK)에서만 (F3): 인코더 일부 탈락 시
		 * WAIT(부분실패)→~500ms 후 OFFLINE→복구가 순환하는데, WAIT 에서 리셋하면
		 * 백오프가 무력화되어 0.5s 마다 버스클리어 반복 — 5개 세트 상시 사용 전제라
		 * 복구 시도는 계속하되 간격만 1s 로 수렴시킨다. */
		RD_RET i2c_state = RD_PERIPHERAL_I2C(&ECU_PERIPHERAL);
		if (i2c_state == RET_NOK) {
			uint32_t now = osKernelGetTickCount();
			if ((int32_t)(now - recovery_next_tick) >= 0) {
				RD_I2C_ENCODER_RECOVERY(&hi2c1, &ECU_PERIPHERAL.err);
				recovery_next_tick = now + recovery_interval;
				recovery_interval  = (recovery_interval * 2 > I2C_RECOVERY_MAX_MS)
				                   ? I2C_RECOVERY_MAX_MS : recovery_interval * 2;
			}
		} else if (i2c_state == RET_OK) {
			recovery_interval  = I2C_RECOVERY_BASE_MS;
			recovery_next_tick = 0;
		}
		tick += 10;
		osDelayUntil(tick);
	}
}

void RD_TASK_ADC1(void) {
	/* DMA circular 기동. 콜백 정체(DMA정지) 감시·복구는 systemTask(RD_SYSTEM_CHECKER) 소유. */
	RD_ADC_INIT(&hadc1);

	for(;;)
	{
#ifdef RTOS_IS_AVAILABLE
		osThreadFlagsWait(0x0001, osFlagsWaitAny, osWaitForever);   /* 창(half/full) 완성 시 콜백이 기상 */
#else
		osDelay(1);
#endif
		/* 완성된 창을 박스카 평균 + cal + IIR → weight[] */
		RD_ADC_PROCESS();
	}
}

/* TIM5 free-run 100us tick — CNT 단일 read 라 ISR/태스크 어디서든 원자적 (CRIT 불필요).
 * 센서 timestamp latch(rd_common.h delta_tick 체계)의 단일 시각 소스. */
uint32_t rd_now_tick(void)
{
    return __HAL_TIM_GET_COUNTER(&htim5);
}

/* 드라이버 공용 시계 포트(HW_NowTick) 오버라이드 — can_ak / i2c_as5600 을 TIM5 에 바인딩.
 * 드라이버는 자기 헤더가 선언한 HW_NowTick 만 알고 rd_now_tick / rd_common 은 모른 채 → standalone 유지.
 * (weak 기본값은 각 드라이버 .c(can_ak/i2c_as5600)에 있고, 이 strong 정의가 링크 시 우선한다.) */
uint32_t HW_NowTick(void) { return rd_now_tick(); }

/* 진단용 절대 시각 [us] — 분해능 100us (구현이 rd_now_tick 기반으로 단순화됨). */
uint64_t Get_Time_us(void)
{
    return (uint64_t)rd_now_tick() * 100u;
}
/*  Callback & Error Handler  --------------------------------------------------------*/

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
