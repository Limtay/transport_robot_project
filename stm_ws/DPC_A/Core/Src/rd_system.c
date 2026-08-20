#include "rd_system.h"
#include "cmsis_os.h"
#include "rd_map_dpca.h"
#include "rd_peripheral_dpca.h"
#include "rd_uart.h"

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;

/*==========수신용 패킷정의==========*/
PACKET_comm_t DPCA_PACKET;

/*==========페리페럴 초기화==========*/
PERIPHERAL_t DPCA_PERIPHERAL;

/*==========수신용 usart4번==========*/
UART_Ring_t DPCA_uart4 = {.rx_buffer = {0}, // 배열 초기화
                          .head = 0,
                          .tail = 0,
                          .huart = &huart4, // 미리 선언된 UART_HandleTypeDef

                          .temp_buffer = {0},
                          .rx_new = 0,
                          .last_rx_tick = 0};

void RD_SYSTEM_INIT(void) {
  /*==========COMM INIT==========*/
  if (RD_UART_INIT(&DPCA_uart4) != RET_OK)
    Error_Handler();
  RD_PACKET_INIT(&DPCA_PACKET);

  /*==========GPIO INIT==========*/
  RD_PERIPHERAL_INIT(&DPCA_PERIPHERAL);

  HAL_Delay(1000);

  HAL_GPIO_WritePin(LED_G_GPIO_Port, LED_G_Pin, GPIO_PIN_SET);
}

/*10ms 마다 호출*/
void RD_TASK_CONTROL(void) {
  uint32_t tick = osKernelGetTickCount();

  for (;;) {
    RD_PERIPHERAL_READ(&DPCA_PERIPHERAL); // 현재 핀 값 읽기
    RD_MAP_WRITE(&DPCA_PERIPHERAL, &DPCA_PACKET);

    /* [테스트 코드] 센서A → 솔레노이드 직접 제어, 평소에는 주석처리 */
    // DPCA_PERIPHERAL.EN_ALL = DPCA_PERIPHERAL.PXS_A;
    /*=======================================================*/

    // 패킷의 tx에 읽어온 핀 값 쓰기
    RD_PERIPHERAL_WRITE(&DPCA_PERIPHERAL); // EN값으로 핀 제어

    tick += CONTROL_TASK_PERIOD_MS;
    osDelayUntil(tick);
  }
}

/*인터럽트 발생 시 호출, 20ms동안 호출되지 않으면 UART CHECKER만 자동으로 호출*/
void RD_TASK_UART(void) {
  for (;;) {
    uint32_t flags = osThreadFlagsWait(0x01, osFlagsWaitAny, 20);

    if (RD_UART_CHECKER(&DPCA_uart4, 10) == RET_NOK) {
      RD_UART_RECOVERY(&DPCA_uart4);
    }

    if (flags == 0x01) {
      RD_RET comm_state = RD_PACKET_READ(&DPCA_uart4, &DPCA_PACKET);
      // uart통신으로 들어온 데이터를 패킷의 rx에 작성

      if (comm_state == RET_OK) {
        RD_PACKET_WRITE(&DPCA_uart4, &DPCA_PACKET);
        // 패킷의 tx를 uart통신으로 내보내기

        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_RESET);
      } else if (comm_state == RET_NOK)
        HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, GPIO_PIN_SET);

      RD_MAP_READ(&DPCA_PERIPHERAL, &DPCA_PACKET); // 패킷의 rx값으로 EN값 작성
    }
  }
}