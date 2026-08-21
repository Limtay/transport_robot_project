/*
 * rd_peripheral_dpca.c
 *
 *  Created on: Jan 22, 2026
 *      Author: abc01
 */

/* Includes ------------------------------------------------------------------*/
#include "rd_peripheral_dpca.h"
#include "stm32f4xx_hal_gpio.h"
#include <string.h>

/* Exported includes
 * ----------------------------------------------------------*/

/* Exported typedef
 * -----------------------------------------------------------*/

/* Exported define
 * ------------------------------------------------------------*/

/* Exported variables
 * ---------------------------------------------------------*/

/* Exported function prototypes
 * -----------------------------------------------*/
RD_RET RD_PERIPHERAL_INIT(PERIPHERAL_t *peripheral_obj);
RD_RET RD_PERIPHERAL_WRITE(PERIPHERAL_t *peripheral_obj); // Drive IO need
RD_RET
RD_PERIPHERAL_READ(PERIPHERAL_t *peripheral_obj); // Read IO state what we need

/* Private user code ---------------------------------------------------------*/

#define LPF_READ(state, lpf_sum, port, pin)                                    \
  do {                                                                         \
    float current_val =                                                        \
        (HAL_GPIO_ReadPin((port), (pin)) == GPIO_PIN_SET) ? 1.0f : 0.0f;       \
    (lpf_sum) = (1.0f - LPF_ALPHA) * (lpf_sum) + LPF_ALPHA * current_val;      \
    if ((lpf_sum) > LPF_THRESHOLD_HIGH)                                        \
      (state) = 1;                                                             \
    else if ((lpf_sum) < LPF_THRESHOLD_LOW)                                    \
      (state) = 0;                                                             \
  } while (0)

RD_RET RD_PERIPHERAL_INIT(PERIPHERAL_t *peripheral_obj) {
  if (peripheral_obj == NULL)
    return RET_NOK;

  // INIT PROX pin
  peripheral_obj->IO.PXS_A_IO.per_GPIO_Pin = PXS_A_Pin; // proximity A
  peripheral_obj->IO.PXS_A_IO.per_GPIOx = PXS_A_GPIO_Port;

  peripheral_obj->IO.PXS_B_IO.per_GPIO_Pin = PXS_B_Pin; // proximity B
  peripheral_obj->IO.PXS_B_IO.per_GPIOx = PXS_B_GPIO_Port;

  peripheral_obj->IO.PXS_C_IO.per_GPIO_Pin = PXS_C_Pin; // proximity C
  peripheral_obj->IO.PXS_C_IO.per_GPIOx = PXS_C_GPIO_Port;

  // INIT BOOT pin
  peripheral_obj->IO.BOOT_IO_1.per_GPIO_Pin = BOOT_IO_1_Pin;
  peripheral_obj->IO.BOOT_IO_1.per_GPIOx = BOOT_IO_1_GPIO_Port;

  peripheral_obj->IO.BOOT_IO_2.per_GPIO_Pin = BOOT_IO_2_Pin;
  peripheral_obj->IO.BOOT_IO_2.per_GPIOx = BOOT_IO_2_GPIO_Port;

  // INIT SOL pin
  peripheral_obj->IO.EN_IO.per_GPIO_Pin = SOL_EN_Pin; // solenoid all EN
  peripheral_obj->IO.EN_IO.per_GPIOx = SOL_EN_GPIO_Port;

  peripheral_obj->IO.CON_A_IO.per_GPIO_Pin = CON_A_Pin; // solenoid CON A
  peripheral_obj->IO.CON_A_IO.per_GPIOx = CON_A_GPIO_Port;

  peripheral_obj->IO.CON_B_IO.per_GPIO_Pin = CON_B_Pin; // solenoid CON B
  peripheral_obj->IO.CON_B_IO.per_GPIOx = CON_B_GPIO_Port;

  peripheral_obj->IO.CON_C_IO.per_GPIO_Pin = CON_C_Pin; // solenoid CON C
  peripheral_obj->IO.CON_C_IO.per_GPIOx = CON_C_GPIO_Port;

  peripheral_obj->IO.CON_D_IO.per_GPIO_Pin = CON_D_Pin; // solenoid CON D
  peripheral_obj->IO.CON_D_IO.per_GPIOx = CON_D_GPIO_Port;

  // INIT SOL state
  peripheral_obj->PXS_A = 0;
  peripheral_obj->PXS_B = 0;
  peripheral_obj->PXS_C = 0;

  peripheral_obj->EN_ALL = 0;
  peripheral_obj->last_en_tick = 0; // reset tick (ms)
  peripheral_obj->EN_BOOT = 0;

  peripheral_obj->CON_A = 0;
  peripheral_obj->CON_B = 0;
  peripheral_obj->CON_C = 0;
  peripheral_obj->CON_D = 0;

  // Init LPF sum
  peripheral_obj->lpf_sum_CON_A = 0.0f;
  peripheral_obj->lpf_sum_CON_B = 0.0f;
  peripheral_obj->lpf_sum_CON_C = 0.0f;
  peripheral_obj->lpf_sum_CON_D = 0.0f;
  peripheral_obj->lpf_sum_PXS_A = 0.0f;
  peripheral_obj->lpf_sum_PXS_B = 0.0f;
  peripheral_obj->lpf_sum_PXS_C = 0.0f;

  return RET_OK;
}

RD_RET RD_PERIPHERAL_WRITE(PERIPHERAL_t *peripheral_obj) {
  if (peripheral_obj == NULL)
    return RET_NOK;

  if (peripheral_obj->EN_ALL == 1)
    HAL_GPIO_WritePin(peripheral_obj->IO.EN_IO.per_GPIOx,
                      peripheral_obj->IO.EN_IO.per_GPIO_Pin, GPIO_PIN_SET);
  else if (peripheral_obj->EN_ALL == 0)
    HAL_GPIO_WritePin(peripheral_obj->IO.EN_IO.per_GPIOx,
                      peripheral_obj->IO.EN_IO.per_GPIO_Pin, GPIO_PIN_RESET);

  if (peripheral_obj->EN_BOOT == 1)
    HAL_GPIO_TogglePin(peripheral_obj->IO.BOOT_IO_1.per_GPIOx,
                       peripheral_obj->IO.BOOT_IO_1
                           .per_GPIO_Pin); // inverse for UART dummy format
  else if (peripheral_obj->EN_BOOT == 0)
    HAL_GPIO_WritePin(peripheral_obj->IO.BOOT_IO_1.per_GPIOx,
                      peripheral_obj->IO.BOOT_IO_1.per_GPIO_Pin,
                      GPIO_PIN_SET); // so it is pull-up pin

  return RET_OK;
}

RD_RET RD_PERIPHERAL_READ(PERIPHERAL_t *peripheral_obj) {
  if (peripheral_obj == NULL)
    return RET_NOK;

  LPF_READ(peripheral_obj->CON_A, peripheral_obj->lpf_sum_CON_A,
           peripheral_obj->IO.CON_A_IO.per_GPIOx,
           peripheral_obj->IO.CON_A_IO.per_GPIO_Pin);
  LPF_READ(peripheral_obj->CON_B, peripheral_obj->lpf_sum_CON_B,
           peripheral_obj->IO.CON_B_IO.per_GPIOx,
           peripheral_obj->IO.CON_B_IO.per_GPIO_Pin);
  LPF_READ(peripheral_obj->CON_C, peripheral_obj->lpf_sum_CON_C,
           peripheral_obj->IO.CON_C_IO.per_GPIOx,
           peripheral_obj->IO.CON_C_IO.per_GPIO_Pin);
  LPF_READ(peripheral_obj->CON_D, peripheral_obj->lpf_sum_CON_D,
           peripheral_obj->IO.CON_D_IO.per_GPIOx,
           peripheral_obj->IO.CON_D_IO.per_GPIO_Pin);

  LPF_READ(peripheral_obj->PXS_A, peripheral_obj->lpf_sum_PXS_A,
           peripheral_obj->IO.PXS_A_IO.per_GPIOx,
           peripheral_obj->IO.PXS_A_IO.per_GPIO_Pin);
  LPF_READ(peripheral_obj->PXS_B, peripheral_obj->lpf_sum_PXS_B,
           peripheral_obj->IO.PXS_B_IO.per_GPIOx,
           peripheral_obj->IO.PXS_B_IO.per_GPIO_Pin);
  LPF_READ(peripheral_obj->PXS_C, peripheral_obj->lpf_sum_PXS_C,
           peripheral_obj->IO.PXS_C_IO.per_GPIOx,
           peripheral_obj->IO.PXS_C_IO.per_GPIO_Pin);

  return RET_OK;
}

/* ==================== READ ME ====================*/
// 0. RD_PERIPHERAL_INIT은 "실제 핀의 배치를 추상화하는 함수"임. 실제로
// 포트바뀌면 여기서 다 수정해야되고, 기본적으로 main.h에서 1차적으로
// 정의되있긴함.
// 1. RD_PERIPHERAL_WRITE는 페리페럴값을 기반으로 EN_ALL, EN_BOOT 핀들을 갱신함.
// 2. RD_PERIPHERAL_READ는 센서, 솔레노이드 피드백 핀들을 갱신함.
