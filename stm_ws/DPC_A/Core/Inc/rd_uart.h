/*
 * RD_UART.h
 *
 *  Created on: Aug 12, 2025
 *      Author: abc01
 */

#ifndef INC_RD_UART_H_
#define INC_RD_UART_H_

/* Private includes ----------------------------------------------------------*/
#include "rd_common.h"
#include "rd_define.h"
#include "stm32f4xx_hal.h"

/* Exported macro ------------------------------------------------------------*/
#define RX_BUFFER_SIZE 32
#define TX_BUFFER_SIZE 32

#define TX_TIMEOUT 10 /**< RS485 송신 모드 강제 복귀 임계 (ms)         */
#define UART_RX_TIMEOUT_MS                                                     \
  100                        /**< RX 무수신 타임아웃 → HC_TIMEOUT (ms)        */
#define UART_FATAL_CNT_TH 10 /**< 연속 HAL 에러 누적 임계 → LS_OFFLINE */

/* --- 범용 packet 에러 비트 (UART_Ring_t.comm_err_flag 에 set) ----------------
 *  어떤 packet protocol (RS485 Dyn 2.0, RC 수신기 등) 이든 같은 의미.
 *  packet layer (rd_comm_ecu, rd_comm_receive) 가 OR 로 set,
 *  RD_UART_CHECKER 가 우선순위에 따라 HC_CRC_ERR / HC_FRAMING_ERR 매핑 후 즉시
 * clear. */
#define COMM_ERR_FRAMING_BIT                                                   \
  (1u << 0)                        /**< packet 구조 깨짐 (header/length 등)   */
#define COMM_ERR_CRC_BIT (1u << 1) /**< packet 무결성 깨짐 (CRC/checksum) */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief UART 링버퍼 핸들. DMA RX + DMA TX + 에러 상태를 모두 포함.
 *
 *  volatile: ISR(IDLE, DMA TC)과 태스크가 동시에 접근하는 필드에 적용.
 */
typedef struct {
  uint8_t rx_buffer[RX_BUFFER_SIZE]; /**< DMA 링버퍼 (DMA 가 직접 씀) */
  volatile uint16_t head;      /**< 직전 IDLE 때 처리가 끝난 위치         */
  volatile uint16_t tail;      /**< 현재 DMA write 위치                   */
  UART_HandleTypeDef *huart;   /**< HAL UART 핸들 (의존성 주입)           */
  volatile uint16_t rx_length; /**< 최근 IDLE 에서 수신된 바이트 수       */

  uint8_t temp_buffer[RX_BUFFER_SIZE]; /**< 선형화된 수신 데이터 저장 */
  volatile uint8_t rx_new; /**< 신규 수신 데이터 플래그 (1=있음)     */
  volatile uint32_t last_rx_tick; /**< 마지막 수신 시각 (HAL_GetTick 기준). INIT
                                     시 0 으로 세팅. 0 = 아직 미수신 (LS_READY →
                                     LS_RUNNING 승격 불가 조건). */

  uint8_t tx_buffer[TX_BUFFER_SIZE]; /**< DMA 송신 버퍼 */
  volatile uint16_t tx_length; /**< 송신 예정 바이트 수                   */

  volatile ERROR_STATUS_t error;
  volatile uint8_t
      comm_err_flag; /**< 범용 packet 에러 비트 (COMM_ERR_FRAMING_BIT /
                      COMM_ERR_CRC_BIT). packet layer 가 OR set, Checker 가 HC_*
                      매핑 후 즉시 clear. */

} UART_Ring_t;

/* Exported constants --------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/
RD_RET RD_UART_INIT(UART_Ring_t *uart_obj);
RD_RET RD_UART_RECOVERY(UART_Ring_t *uart_obj);
RD_RET RD_UART_IDLEHandler(UART_Ring_t *uart_obj);
RD_RET RD_UART_Transmit(UART_Ring_t *uart_obj);
RD_RET RD_UART_CHECKER(UART_Ring_t *uart_obj, uint16_t degraded_k);

#endif /* INC_RD_UART_H_ */
