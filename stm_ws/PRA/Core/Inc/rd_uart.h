/**
 ******************************************************************************
 * @file    rd_uart.h
 * @author  abc01
 * @date    2025-08-12 (PRA 이식 2026-09-17, Step6 — ECU/DPC_B advanced 드라이버)
 * @brief   DMA+IDLE 인터럽트 기반 UART 링버퍼 드라이버 (UART / RS485).
 *
 *  ECU_V3/DPC_B 의 advanced rd_uart 를 PRA 로 이식(단순형 대체). STATE_t error /
 *  degraded_cnt / lifecycle(LS_*) / Checker+Recovery 분리 모델. DIR·huart 는 주입식.
 *
 *  에러 처리 정책
 *    - `state.bits.lifecycle == LS_OFFLINE` 이면 Checker 가 `RET_NOK` 반환.
 *    - 상위 레이어(systemTask)가 `RET_NOK` 를 감지하고 직접 `RD_*_RECOVERY` 호출.
 *    - Checker 는 절대 INIT/RECOVERY 를 내부 호출하지 않음.
 *
 *  ISR 배선 (stm32f4xx_it.c USART6_IRQHandler):
 *    - TC  플래그 → RD_RS485_IRQ_HANDLER(&PRA_rs485)  (HAL_UART_IRQHandler 앞)
 *    - IDLE 플래그 → RD_UART_IDLE_HANDLER(PRA_rs485.uart_obj)  (뒤)
 ******************************************************************************
 */

#ifndef INC_RD_UART_H_
#define INC_RD_UART_H_

/* Private includes ----------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "rd_common.h"
#include "rd_define.h"

/* RS485 -> GPIO_t */
#if __has_include("rd_peripheral.h")
    #include "rd_peripheral.h"
    #define RS485_AVAILABLE
#endif

/* RTOS 전용 TASK */
#if __has_include("cmsis_os2.h")
    #include "cmsis_os2.h"
    extern osThreadId_t rs485TaskHandle;   /**< Orin RS485(USART6) 태스크 */
    #define RTOS_IS_AVAILABLE  /**< UART_Ring_t.wake_task 로 태스크 깨우기 활성화 */
#endif

/* Exported macro ------------------------------------------------------------*/
/* RS485 는 멀티드롭 공유 버스 — DMA 링에 타 노드 트래픽까지 유입되므로 버스 최대 프레임의
 * 2배 여유. 구 128B 시 오버플로우 → rx_length 쓰레기 → FRAMING 폭주 (DPC_B history §20).
 * RAM: rx(256)+temp(256)+tx(272)=784B (F446 128KB 대비 무시 가능). */
#define RX_BUFFER_SIZE  256  /**< DMA 링버퍼 / temp 버퍼 크기 (bytes) */
#define TX_BUFFER_SIZE  272  /**< DMA 송신 버퍼 크기 (bytes) — payload 256 + 프레임 8 + 여유 */

#define TX_TIMEOUT          10   /**< RS485 송신 모드 강제 복귀 임계 (ms)         */
#define UART_RX_TIMEOUT_MS  100   /**< RX 무수신 타임아웃 → HC_TIMEOUT (ms)        */
#define UART_FATAL_CNT_TH   10   /**< 연속 HAL 에러 누적 임계 → LS_OFFLINE        */

/* --- 범용 packet 에러 비트 (UART_Ring_t.comm_err_flag 에 set) ----------------
 *  packet layer(rd_comm_orin) 가 OR set, RD_UART_CHECKER 가 HC_* 매핑 후 즉시 clear. */
#define COMM_ERR_FRAMING_BIT (1u << 0)  /**< packet 구조 깨짐 (header/length 등)   */
#define COMM_ERR_CRC_BIT     (1u << 1)  /**< packet 무결성 깨짐 (CRC/checksum)     */

/* Exported types ------------------------------------------------------------*/

typedef struct {
    uint8_t              rx_buffer[RX_BUFFER_SIZE]; /**< DMA 링버퍼 (DMA 가 직접 씀)          */
    volatile uint16_t    head;                       /**< 직전 IDLE 때 처리가 끝난 위치         */
    volatile uint16_t    tail;                       /**< 현재 DMA write 위치                   */
    UART_HandleTypeDef  *huart;                      /**< HAL UART 핸들 (의존성 주입)           */
    volatile uint16_t    rx_length;                  /**< 최근 IDLE 에서 수신된 바이트 수       */

    uint8_t              temp_buffer[RX_BUFFER_SIZE]; /**< 선형화된 수신 데이터 저장            */
    volatile uint8_t     rx_new;                     /**< 신규 수신 데이터 플래그 (1=있음)     */
    volatile uint32_t    last_rx_tick;               /**< 마지막 수신 시각 (0=아직 미수신)     */

    uint8_t              tx_buffer[TX_BUFFER_SIZE];  /**< DMA 송신 버퍼                         */
    volatile uint16_t    tx_length;                  /**< 송신 예정 바이트 수                   */

    volatile ERROR_STATUS_t error;
    volatile uint8_t     comm_err_flag;              /**< 범용 packet 에러 비트 (COMM_ERR_*).  */

#ifdef RTOS_IS_AVAILABLE
    osThreadId_t         wake_task;                 /**< IDLE ISR 에서 깨울 태스크. INIT 후 상위에서 주입. */
#endif

} UART_Ring_t;

#ifdef RS485_AVAILABLE
typedef struct {
    UART_Ring_t         *uart_obj;                   /**< 내부 UART 링버퍼 핸들 (의존성 주입)  */
    GPIO_IO_t            DIR;                        /**< RS485 방향 제어 GPIO (주입식)         */
    volatile uint8_t     tx_mode;                    /**< 1 = 송신 모드, 0 = 수신 모드          */
    volatile uint32_t    last_tx_tick;               /**< 마지막 송신 시각 (TX_TIMEOUT 감지용)  */
} RS485_t;
#endif


/* Exported functions prototypes ---------------------------------------------*/
RD_RET RD_UART_INIT(UART_Ring_t *uart_obj, UART_HandleTypeDef *huart);
RD_RET RD_UART_RECOVERY(UART_Ring_t *uart_obj);
RD_RET RD_UART_IDLE_HANDLER(UART_Ring_t *uart_obj);
RD_RET RD_UART_TRANSMIT(UART_Ring_t *uart_obj);
RD_RET RD_UART_CHECKER(UART_Ring_t *uart_obj, uint16_t degraded_k);

#ifdef RS485_AVAILABLE
RD_RET RD_RS485_INIT(RS485_t *rs485_obj, UART_HandleTypeDef *huart);
RD_RET RD_RS485_RECOVERY(RS485_t *rs485_obj);
RD_RET RD_RS485_TRANSMIT(RS485_t *rs485_obj);
RD_RET RD_RS485_CHECKER(RS485_t *rs485_obj, uint16_t degraded_k);
RD_RET RD_RS485_IRQ_HANDLER(RS485_t *rs485_obj);
#endif

#endif /* INC_RD_UART_H_ */
