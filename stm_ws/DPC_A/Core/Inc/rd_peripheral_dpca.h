/*
 * rd_peripheral_dpca.h
 *
 *  Created on: Jan 22, 2026
 *      Author: abc01
 */

#ifndef INC_RD_PERIPHERAL_DPCA_H_
#define INC_RD_PERIPHERAL_DPCA_H_

/* Private includes ----------------------------------------------------------*/
#include "main.h"
#include "rd_common.h"
#include "rd_define.h"
#include "stm32f4xx_hal.h"

/* Exported macro ------------------------------------------------------------*/
#define CONTROL_TASK_PERIOD_MS 10 // 제어 Task가 도는 주기 (ms)
#define LPF_ALPHA 0.045f          // lpf_sum(n)=1−(1−α)^n
#define LPF_THRESHOLD_HIGH 0.9f
#define LPF_THRESHOLD_LOW 0.1f
// 0.045f, 0.9f, 0.1f → 500ms 반응속도

/* Exported types ------------------------------------------------------------*/

// 핀 1개 지정
typedef struct {
  GPIO_TypeDef *per_GPIOx; // 어느 알파벳 포트인가? (예: GPIOA, GPIOB 등)
  uint16_t per_GPIO_Pin;   // 몇 번 핀인가? (예: GPIO_PIN_0, GPIO_PIN_5 등)
  uint32_t *per_pCCR; // (추후 확장용) PWM 제어를 위한 타이머 레지스터 포인터
} PERIPHERAL_IO_t;

// 핀 묶음
typedef struct {
  PERIPHERAL_IO_t BOOT_IO_1; // 부팅 제어용 핀 1
  PERIPHERAL_IO_t BOOT_IO_2; // 부팅 제어용 핀 2

  PERIPHERAL_IO_t EN_IO;    // 솔레노이드 밸브 전체 전원 On/Off 핀
  PERIPHERAL_IO_t CON_A_IO; // 솔레노이드 A 읽기 핀
  PERIPHERAL_IO_t CON_B_IO; // 솔레노이드 B 읽기 핀
  PERIPHERAL_IO_t CON_C_IO; // 솔레노이드 C 읽기 핀
  PERIPHERAL_IO_t CON_D_IO; // 솔레노이드 D 읽기 핀

  PERIPHERAL_IO_t PXS_A_IO; // 근접 센서(Proximity) A 입력 핀
  PERIPHERAL_IO_t PXS_B_IO; // 근접 센서 B 입력 핀
  PERIPHERAL_IO_t PXS_C_IO; // 근접 센서 C 입력 핀

} PERIPHERAL_IO_ALL_t;

typedef struct {

  /* 1. 출력(명령) 변수들 */
  volatile uint8_t EN_ALL; // 솔레노이드 전체 전원 제어 명령 (1: On, 0: Off)
  volatile uint32_t last_en_tick; // 솔레노이드가 켜진 시간 기록
  volatile uint8_t EN_BOOT;       // 부팅 시퀀스 제어 명령

  /* 2. 각 솔레노이드 피드백 (1: On, 0: Off) */
  volatile uint8_t CON_A;
  volatile uint8_t CON_B;
  volatile uint8_t CON_C;
  volatile uint8_t CON_D;

  /* 3. 각 근접 센서(Proximity) 입력 상태 (1: 감지됨, 0: 감지 안 됨) */
  volatile uint8_t PXS_A;
  volatile uint8_t PXS_B;
  volatile uint8_t PXS_C;

  /* 4. LPF 변수  */
  volatile float lpf_sum_CON_A;
  volatile float lpf_sum_CON_B;
  volatile float lpf_sum_CON_C;
  volatile float lpf_sum_CON_D;
  volatile float lpf_sum_PXS_A;
  volatile float lpf_sum_PXS_B;
  volatile float lpf_sum_PXS_C;

  /* 5. 물리적 핀 주소록 */
  PERIPHERAL_IO_ALL_t IO;

} PERIPHERAL_t;

/* Exported constants --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
RD_RET RD_PERIPHERAL_INIT(PERIPHERAL_t *peripheral_obj);
RD_RET RD_PERIPHERAL_WRITE(PERIPHERAL_t *peripheral_obj);
RD_RET RD_PERIPHERAL_READ(PERIPHERAL_t *peripheral_obj);

#endif /* INC_RD_PERIPHERAL_DPCA_H_ */
