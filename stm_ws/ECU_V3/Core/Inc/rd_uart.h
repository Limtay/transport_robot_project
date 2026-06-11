/**
 ******************************************************************************
 * @file    rd_uart.h
 * @author  abc01
 * @date    2025-08-12
 * @brief   DMA+IDLE 인터럽트 기반 UART 링버퍼 드라이버 (UART / RS485).
 *
 *  모듈 개요
 *  -----------------------------------------------------------------------
 *    - DMA 수신 + IDLE 인터럽트로 가변 길이 프레임을 끊어 수신.
 *    - DMA 송신으로 블로킹 없이 패킷 송신.
 *    - RS485 DIR 핀 제어 (TX 완료 후 자동 RX 복귀는 TC 인터럽트에서 처리).
 *
 *  에러 처리 정책
 *  -----------------------------------------------------------------------
 *    - `state.bits.lifecycle == LS_OFFLINE` 이면 Checker 가 `RET_NOK` 반환.
 *    - 상위 레이어가 `RET_NOK` 를 감지하고 직접 `RD_UART_RECOVERY` 호출.
 *    - Checker 는 절대 INIT/RECOVERY 를 내부 호출하지 않음.
 *
 *  사용 순서
 *  -----------------------------------------------------------------------
 *    1) `RD_UART_INIT(&uart_obj)` 또는 `RD_RS485_INIT(&rs485_obj)` — 1회.
 *    2) stm32f4xx_it.c USART IRQ 핸들러 내부:
 *         @code
 *         // TC 플래그 — RS485 DIR 복귀 (RS485 전용)
 *         if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_TC)) {
 *             HAL_GPIO_WritePin(rs485.DIR.per_GPIOx, rs485.DIR.per_GPIO_Pin, GPIO_PIN_RESET);
 *             rs485.tx_mode = 0;
 *         }
 *         HAL_UART_IRQHandler(&huart2);
 *         // IDLE 플래그 — 수신 완료
 *         if (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_IDLE)) {
 *             __HAL_UART_CLEAR_IDLEFLAG(&huart2);
 *             RD_UART_IDLE_HANDLER(rs485.uart_obj);
 *         }
 *         @endcode
 *    3) 주기 태스크(또는 RTOS 태스크 깨운 뒤):
 *         @code
 *         if (RD_UART_CHECKER(&uart_obj, DEGRADED_K_100HZ) == RET_NOK)
 *             RD_UART_RECOVERY(&uart_obj);   // 상위 레이어가 판단 후 호출
 *         @endcode
 *
 *  RTOS 지원
 *  -----------------------------------------------------------------------
 *    cmsis_os2.h 가 존재하면 IDLE ISR 에서 rs485TaskHandle 스레드를 깨움.
 *    필요 없으면 `RTOS_IS_AVAILABLE` 정의를 주석 처리.
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
	extern osThreadId_t rcTaskHandle;
    extern osThreadId_t rs485TaskHandle;
    extern osThreadId_t imuTaskHandle;
    #define RTOS_IS_AVAILABLE  /**< RS485 Thread 깨우기용 플래그 — 미사용시 주석 처리 */
#endif

/* Exported macro ------------------------------------------------------------*/
#define RX_BUFFER_SIZE  64  /**< DMA 링버퍼 / temp 버퍼 크기 (bytes)*/
#define TX_BUFFER_SIZE  128  /**< DMA 송신 버퍼 크기 (bytes)*/

#define TX_TIMEOUT          10   /**< RS485 송신 모드 강제 복귀 임계 (ms)         */
#define UART_RX_TIMEOUT_MS  100   /**< RX 무수신 타임아웃 → HC_TIMEOUT (ms)        */
#define UART_FATAL_CNT_TH   10   /**< 연속 HAL 에러 누적 임계 → LS_OFFLINE        */

/* --- 범용 packet 에러 비트 (UART_Ring_t.comm_err_flag 에 set) ----------------
 *  어떤 packet protocol (RS485 Dyn 2.0, RC 수신기 등) 이든 같은 의미.
 *  packet layer (rd_comm_ecu, rd_comm_receive) 가 OR 로 set,
 *  RD_UART_CHECKER 가 우선순위에 따라 HC_CRC_ERR / HC_FRAMING_ERR 매핑 후 즉시 clear. */
#define COMM_ERR_FRAMING_BIT (1u << 0)  /**< packet 구조 깨짐 (header/length 등)   */
#define COMM_ERR_CRC_BIT     (1u << 1)  /**< packet 무결성 깨짐 (CRC/checksum)     */

/* Exported types ------------------------------------------------------------*/

/**
 * @brief UART 링버퍼 핸들. DMA RX + DMA TX + 에러 상태를 모두 포함.
 *
 *  volatile: ISR(IDLE, DMA TC)과 태스크가 동시에 접근하는 필드에 적용.
 */
typedef struct {
    uint8_t              rx_buffer[RX_BUFFER_SIZE]; /**< DMA 링버퍼 (DMA 가 직접 씀)          */
    volatile uint16_t    head;                       /**< 직전 IDLE 때 처리가 끝난 위치         */
    volatile uint16_t    tail;                       /**< 현재 DMA write 위치                   */
    UART_HandleTypeDef  *huart;                      /**< HAL UART 핸들 (의존성 주입)           */
    volatile uint16_t    rx_length;                  /**< 최근 IDLE 에서 수신된 바이트 수       */

    uint8_t              temp_buffer[RX_BUFFER_SIZE]; /**< 선형화된 수신 데이터 저장            */
    volatile uint8_t     rx_new;                     /**< 신규 수신 데이터 플래그 (1=있음)     */
    volatile uint32_t    last_rx_tick;               /**< 마지막 수신 시각 (HAL_GetTick 기준). INIT 시 0 으로 세팅.
                                                          0 = 아직 미수신 (LS_READY → LS_RUNNING 승격 불가 조건). */

    uint8_t              tx_buffer[TX_BUFFER_SIZE];  /**< DMA 송신 버퍼                         */
    volatile uint16_t    tx_length;                  /**< 송신 예정 바이트 수                   */

    volatile ERROR_STATUS_t error;
	volatile uint8_t     comm_err_flag;              /**< 범용 packet 에러 비트 (COMM_ERR_FRAMING_BIT / COMM_ERR_CRC_BIT).
                                                          packet layer 가 OR set, Checker 가 HC_* 매핑 후 즉시 clear. */

} UART_Ring_t;

#ifdef RS485_AVAILABLE
/**
 * @brief RS485 핸들. UART 핸들을 포함하고 DIR 핀과 송신 타임아웃을 관리.
 */
typedef struct {
    UART_Ring_t         *uart_obj;                   /**< 내부 UART 링버퍼 핸들 (의존성 주입)  */
    GPIO_IO_t            DIR;                        /**< RS485 방향 제어 GPIO                  */
    volatile uint8_t     tx_mode;                    /**< 1 = 송신 모드, 0 = 수신 모드          */
    volatile uint32_t    last_tx_tick;               /**< 마지막 송신 시각 (TX_TIMEOUT 감지용)  */
} RS485_t;
#endif


/* Exported functions prototypes ---------------------------------------------*/

/**
 * @brief  버퍼·DMA·카운터·IDLE 인터럽트 초기화. 하드웨어는 준비된 상태를 가정.
 *         최초 부팅 시 직접 호출하거나, RD_UART_RECOVERY 에서 내부 호출.
 * @param  uart_obj  대상 핸들
 * @param  huart     주입할 HAL UART 핸들 (INIT 시 1회 주입, 이후 uart_obj->huart 로 재사용)
 * @retval RET_OK   초기화 성공
 * @retval RET_NOK  DMA 시작 실패 또는 NULL 포인터
 */
RD_RET RD_UART_INIT(UART_Ring_t *uart_obj, UART_HandleTypeDef *huart);

/**
 * @brief  하드웨어 완전 재초기화 (Abort → DeInit → Init) 후 RD_UART_INIT 호출.
 *         상위 레이어가 Checker 에서 RET_NOK 를 받은 뒤 직접 호출.
 *         huart 는 INIT 시 주입된 uart_obj->huart 를 재사용 (별도 인자 불필요).
 * @param  uart_obj  대상 핸들
 * @retval RET_OK   복구 성공
 * @retval RET_NOK  HAL 재초기화 실패 또는 NULL 포인터
 */
RD_RET RD_UART_RECOVERY(UART_Ring_t *uart_obj);

/**
 * @brief  IDLE 인터럽트 핸들러에서 호출. DMA 현재 위치로 링버퍼를 처리하여
 *         temp_buffer 에 선형화하고 rx_new = 1 로 설정.
 * @param  uart_obj  대상 핸들
 * @retval RET_OK   새 데이터 수신
 * @retval RET_WAIT 수신 데이터 없음 (길이 0)
 * @retval RET_NOK  NULL 포인터 또는 HAL 에러 코드 존재
 */
RD_RET RD_UART_IDLE_HANDLER(UART_Ring_t *uart_obj);

/**
 * @brief  DMA TX 를 시작. gState 가 READY 가 아니면 RET_WAIT 반환.
 * @param  uart_obj  대상 핸들 (tx_buffer / tx_length 를 미리 채운 뒤 호출)
 * @retval RET_OK   송신 시작 성공
 * @retval RET_WAIT UART 가 아직 이전 송신 중
 * @retval RET_NOK  NULL 포인터, tx_length = 0, 또는 HAL 송신 실패
 */
RD_RET RD_UART_TRANSMIT(UART_Ring_t *uart_obj);

/**
 * @brief  isr_err_code / packet 에러 / 타임아웃을 검사하여 state.bits.health + lifecycle 을 갱신.
 *         rx_error_cnt 는 HAL 하드웨어 에러 전용 연속 카운터 (packet/timeout 에러는 건드리지 않음).
 *         HAL 에러 없는 틱에서 rx_error_cnt = 0 리셋 — IDLE 인터럽트가 아닌 Checker 가 관리.
 *         degraded_cnt 는 ALL 에러 (HAL + packet + timeout) 에 +degraded_k, 무에러 틱에 -DECAY.
 *         RUNNING ↔ DEGRADED 전이는 `DEGRADED_THRESHOLD_HIGH/LOW` 히스테리시스로 결정.
 *         lifecycle == LS_OFFLINE 이면 RET_NOK 반환 → 상위가 RD_UART_RECOVERY 직접 호출.
 * @param  uart_obj    대상 핸들
 * @param  degraded_k  채널 통신 주파수에 맞는 가중치 (rd_common.h: DEGRADED_K_100HZ / _200HZ / _250HZ).
 *                     상위 레이어가 채널 특성에 따라 결정해 주입 (DIP — 드라이버는 정책 모름).
 * @retval RET_OK   정상
 * @retval RET_WAIT 경고/에러 수준 (상위 레이어가 모니터링)
 * @retval RET_NOK  LS_OFFLINE (복구 필요)
 */
RD_RET RD_UART_CHECKER(UART_Ring_t *uart_obj, uint16_t degraded_k);

#ifdef RS485_AVAILABLE
/**
 * @brief  DIR 핀/필드 설정 후 RD_UART_INIT 호출. 최초 부팅 시 1회 호출.
 * @param  rs485_obj  대상 핸들 (uart_obj 포인터를 미리 연결해야 함)
 * @param  huart      주입할 HAL UART 핸들 (INIT 시 1회 주입)
 * @retval RET_OK   초기화 성공
 * @retval RET_NOK  NULL 포인터 또는 UART 초기화 실패
 */
RD_RET RD_RS485_INIT(RS485_t *rs485_obj, UART_HandleTypeDef *huart);

/**
 * @brief  DIR 핀 RX 복귀 후 RD_UART_RECOVERY 호출.
 *         상위 레이어가 Checker 에서 RET_NOK 를 받은 뒤 직접 호출.
 *         huart 는 INIT 시 주입된 uart_obj->huart 를 재사용 (별도 인자 불필요).
 * @param  rs485_obj  대상 핸들
 * @retval RET_OK   복구 성공
 * @retval RET_NOK  NULL 포인터 또는 UART 복구 실패
 */
RD_RET RD_RS485_RECOVERY(RS485_t *rs485_obj);

/**
 * @brief  RS485 송신 시작. DIR 핀을 TX 모드(SET)로 전환 후 DMA TX 시작.
 *         송신 실패 시 DIR 핀을 즉시 RX 모드로 복귀.
 *         TC 인터럽트(stm32f4xx_it.c)에서 송신 완료 후 DIR 핀을 RESET 해야 함.
 * @param  rs485_obj  대상 핸들 (uart_obj->tx_buffer / tx_length 미리 채울 것)
 * @retval RET_OK   송신 시작 성공
 * @retval RET_WAIT UART 가 이전 송신 중
 * @retval RET_NOK  NULL 포인터 또는 HAL 송신 실패
 */
RD_RET RD_RS485_TRANSMIT(RS485_t *rs485_obj);

/**
 * @brief  RD_UART_CHECKER 호출 + TX 타임아웃 시 DIR 핀 강제 복귀.
 *         lifecycle == LS_OFFLINE 이면 RET_NOK 반환.
 *         상위 레이어가 RET_NOK 를 받으면 RD_RS485_RECOVERY 를 직접 호출.
 * @param  rs485_obj   대상 핸들
 * @param  degraded_k  내부 RD_UART_CHECKER 에 그대로 전달 (DEGRADED_K_100HZ 등)
 * @retval RET_OK   정상
 * @retval RET_WAIT 경고/에러 수준 (상위 레이어가 모니터링)
 * @retval RET_NOK  LS_OFFLINE 상태 (복구 필요)
 */
RD_RET RD_RS485_CHECKER(RS485_t *rs485_obj, uint16_t degraded_k);

RD_RET RD_RS485_IRQ_HANDLER(RS485_t *rs485_obj);
#endif

#endif /* INC_RD_UART_H_ */
