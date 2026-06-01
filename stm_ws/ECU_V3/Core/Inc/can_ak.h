/**
 ******************************************************************************
 * @file    can_ak.h
 * @author  Kyeongtae
 * @date    2026-05-21
 * @brief   CubeMars AK 시리즈 모터(AK10-9 등) Servo Mode CAN 드라이버.
 *
 *  - 통신: CAN 2.0B Extended ID (29-bit) @ 1 Mbps.
 *    ExtId 구성: [bit28..8] = control mode (datasheet 4.2),
 *                [bit7..0]  = motor driver CAN_ID.
 *  - RTOS(FreeRTOS+CMSIS-RTOS2) 환경에서는 송신을 Queue로 비동기 처리하고,
 *    Bare-metal 환경에서는 메일박스로 즉시 송신. (USE_RTOS_CAN_QUEUE 자동 감지)
 *
 *  사용 순서 (대표 시나리오)
 *  -----------------------------------------------------------------------
 *    1) CAN_Init(&hcan1);                          // 필터/인터럽트/Start
 *    2) CAN_AK_INIT(&ECU_AK[i], &hcan1, can_id);   // 각 모터 핸들 초기화
 *    3) 제어 루프(주기 태스크) 안에서:
 *         ECU_AK[i].cmd.mode = MODE_VELOCITY;
 *         ECU_AK[i].cmd.rpm  = 1500.0f;
 *         CAN_AK_WRITE(&ECU_AK[i]);                // 송신
 *         CAN_AK_CHECKER(&ECU_AK[i], HAL_GetTick());// RX 타임아웃 감시
 *    4) HAL_CAN_RxFifo0MsgPendingCallback 안에서:
 *         AK_RxFrame_t f;
 *         if (CAN_AK_RX_POP(hcan, &f))
 *             for (i=0..N) if (CAN_AK_RX_APPLY(&ECU_AK[i], &f)) break;
 *
 *  @par Threading / 호출 컨텍스트 (race 방지를 위한 owner 규칙)
 *  -----------------------------------------------------------------------
 *    함수별로 호출이 허용되는 컨텍스트가 다르다. 단일 모터 핸들을 여러
 *    task 에서 동시에 만지지 말 것 — 본 드라이버는 critical section /
 *    atomic intrinsic 을 사용하지 않고 "단일 owner task" 원칙으로 race
 *    를 피한다. ISR ↔ task 공유 필드는 volatile + 단일 store 패턴으로 보호.
 *
 *      CAN_Init / CAN_RECOVERY / CAN_AK_INIT        : init phase (single ctx)
 *      CAN_AK_WRITE                                 : single TASK (controlTask 등)
 *      CAN_AK_CHECKER                               : single TASK (controlTask 등)
 *      CAN_AK_RX_POP  / CAN_AK_RX_APPLY             : ISR only (HAL_CAN_RxFifo0MsgPendingCallback)
 *      CAN_AK_TX_TASK_HANDLER                        : single TASK (can1Task)
 *
 *    ISR 와 task 가 공유하는 핸들 필드 (volatile + 단일 store):
 *      state.position/velocity/current/temp_motor/error_code  : ISR write, task read
 *      error.last_rx_tick                                     : ISR write(=tick), task write(=tick) — 둘 다 새 tick 으로 덮어쓰므로 안전
 *      error.is_running                                       : ISR write(=1, RX_Apply), task write(=0, CHECKER timeout)
 *      error.rx_err_cnt                                       : ISR write(=0, RX_Apply), task write(++, CHECKER) — 1 tick 짜리 false positive 가능, 다음 tick 회복
 *      error.tx_err_cnt                                       : task write only
 *      error.comm_err                                         : task write only (CHECKER 의 단일 store)
 *
 *  단위 / 스케일 요약
 *  -----------------------------------------------------------------------
 *    cmd.current        [A]    Current/Brake 모드에서 ±1000배로 송신
 *    cmd.rpm            [RPM]  Velocity 모드에서 정수 그대로 송신
 *    cmd.pos            [deg]  Position 모드에서 ×10000 정수로 송신
 *    state.position     int16  ×0.1f → [deg]
 *    state.velocity     int16  ×10.0f → [RPM]
 *    state.current      int16  ×0.01f → [A]
 *    state.temp_motor   int8   [°C]
 ******************************************************************************
 */

#ifndef INC_CAN_AK_H_
#define INC_CAN_AK_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include <stdint.h>

/* Exported defines ----------------------------------------------------------*/
/* --- 모터 입력 한계값 (Datasheet Servo Mode 기준) -------------------------- */
#define AK_MAX_RPM   50000.0f   /**< Velocity 모드 최대 RPM (절대값)             */
#define AK_MAX_POS   36000.0f   /**< Position 모드 최대 각도 [deg] (절대값)       */
#define AK_MAX_CUR   60.0f      /**< Current / Current Brake 최대 전류 [A]        */
#define AK_MAX_VEL   327670.0f  /**< Pos-Vel 루프 속도 한계 [ERPM] (int16 ×10)    */
#define AK_MAX_ACC   327670.0f  /**< Pos-Vel 루프 가속도 한계 [ERPM/s] (int16 ×10)*/

/* --- 통신 타임아웃 기준 ---------------------------------------------------- */
#define CAN_RX_TIMEOUT_MS   100  /**< [ms] 이 시간 내 RX 없으면 1회 카운트 증가     */

/* --- 액션 트리거용 임계 (comm_err 산출과는 무관 — 상위 레이어 reset/ESTOP 판정용) --- */
#define AK_TX_TIMEOUT_ERR   5   /**< [count] 연속 TX 실패 임계 — rd_system TX_WARN reset 판단용 */
#define AK_RX_TIMEOUT_ERR   10  /**< [count] 연속 RX 타임아웃 임계 — 상위 레이어 ESCALATION 판단용 */
#define AK_TEMP_WARN        75  /**< [°C]    모터 과열 경고 임계 — rd_system ESTOP_SW 트리거용 */

/* --- comm_err 비트 매핑 (즉시 상태 반영 — 임계값 무관) -------------------- */
#define AK_COMM_RX_BIT      (1u << 0)   /**< bit 0 (값 1): rx_err_cnt > 0 이면 set */
#define AK_COMM_TX_BIT      (1u << 1)   /**< bit 1 (값 2): tx_err_cnt > 0 이면 set */

/* --- CAN 하드웨어 매핑 ----------------------------------------------------- */
#define CAN_AK_RX_FIFO      CAN_RX_FIFO0     /**< 사용할 RX FIFO 번호           */
#define CAN_AK_FILTER_FIFO  CAN_FILTER_FIFO0 /**< RX 필터 → FIFO 할당           */

/* --- 컴파일 옵션 ----------------------------------------------------------- */
/** 정의 시 IDMASK 필터로 AK 모터 ID만 통과. 주석 시 전부 통과(FreePass). */
#define USE_CAN_IDMASK_FILTER

/* RTOS 헤더 존재 시 자동으로 Queue 송신 경로 활성화.
 * Bare-metal로 빌드하려면 cmsis_os2.h를 include path에서 제외하면 됨. */
#if __has_include("cmsis_os2.h")
    #include "cmsis_os2.h"
    #define USE_RTOS_CAN_QUEUE          /**< Queue + TX task 송신 사용         */
    #define MIN_QUEUE 1                 /**< Queue 최소 여유 공간 (포화 방지)   */
#endif

/* Exported types ------------------------------------------------------------*/

/**
 * @brief 모터 드라이버가 보고하는 에러 코드 (datasheet 4.1 RX 8th byte).
 *        AK_State_t.error_code 에 저장.
 */
typedef enum {
    AK_ERROR_NONE = 0,         /**< 정상                      */
    AK_ERROR_MOTOR_OVR_TEMP,   /**< 모터 과열                 */
    AK_ERROR_OVR_CURRENT,      /**< 과전류                    */
    AK_ERROR_OVR_VOLT,         /**< 과전압                    */
    AK_ERROR_UND_VOLT,         /**< 저전압                    */
    AK_ERROR_ENCODER,          /**< 엔코더 이상                */
    AK_ERROR_MOSFET_OVR_TEMP,  /**< MOSFET 과열              */
    AK_ERROR_MOTOR_LCK         /**< 모터 락업 (회전 불가)      */
} AK_ERROR_CODE_e;

/**
 * @brief 제어 모드 (datasheet 4.2 Servo Mode).
 *        값이 ExtId의 상위 바이트로 그대로 들어가므로 순서/값 변경 금지.
 *        MODE_ESTOP은 본 드라이버 전용 — CAN_AK_WRITE에서 송신을 skip하는 의미.
 */
typedef enum {
//  MODE_DUTY      = 0,       // Duty cycle mode (미사용)
    MODE_ESTOP = 0,           /**< 송신 안 함 (소프트 정지 상태)              */
    MODE_CURRENT,             /**< Current loop  : cmd.current [-60~60 A]    */
    MODE_CURRENT_BRAKE,       /**< Current brake : cmd.current [0~60 A]      */
    MODE_VELOCITY,            /**< Velocity      : cmd.rpm [-50k~50k RPM]    */
    MODE_POSITION,            /**< Position      : cmd.pos [-36000~36000 deg]*/
    MODE_SET_ORIGIN,          /**< Set origin    : cmd.set_origin_mode       */
    MODE_POS_VEL_LOOP,        /**< Pos+Vel loop  : cmd.pos/rpm/acc           */
    MODE_MIT                  /**< Force control (미구현)                    */
} AK_Control_Mode_t;

/**
 * @brief 모터로부터 수신된 raw 상태값 (CAN RX 8byte → 파싱 결과).
 *
 *  필드는 datasheet 스케일 그대로 보존 (정수). 물리량이 필요할 때 곱셈으로 환산:
 *    position [deg] = position * 0.1f
 *    velocity [RPM] = velocity * 10.0f
 *    current  [A]   = current  * 0.01f
 *    temp_motor[°C] = temp_motor (그대로)
 */
typedef struct {
    int16_t position;             /**< 위치   raw, ×0.1f → [deg]              */
    int16_t velocity;             /**< 속도   raw, ×10.0f → [RPM]             */
    int16_t current;              /**< 전류   raw, ×0.01f → [A]               */
    int8_t  temp_motor;           /**< 온도   [-20 ~ 127°C]                   */
    AK_ERROR_CODE_e error_code;   /**< 드라이버 자체 에러 코드 (RX byte[7])   */
} AK_State_t;

/**
 * @brief 사용자가 채워서 송신을 트리거하는 명령 묶음.
 *
 *  사용 패턴:
 *    pMotor->cmd.mode = MODE_xxx;       // 어떤 필드를 쓸지 결정
 *    pMotor->cmd.<해당필드> = 값;
 *    CAN_AK_WRITE(pMotor);
 *
 *  mode 별 사용 필드는 AK_Control_Mode_t 주석 참조.
 *  범위 초과 입력은 CAN_AK_WRITE 내부에서 자동으로 clamp됨.
 */
typedef struct {
    AK_Control_Mode_t mode;       /**< 어떤 모드로 송신할지 (필수)            */
    float   current;              /**< [A]      MODE_CURRENT*에서 사용         */
    float   rpm;                  /**< [RPM]    MODE_VELOCITY / POS_VEL_LOOP   */
    float   pos;                  /**< [degree] MODE_POSITION / POS_VEL_LOOP   */
    float   acc;                  /**< [ERPM/s] MODE_POS_VEL_LOOP에서 사용     */
    uint8_t set_origin_mode;      /**< MODE_SET_ORIGIN 인자 (datasheet 4.2.7)  */
} AK_CMD_t;

/**
 * @brief 통신 헬스 카운터. CAN_AK_CHECKER / RX_Apply / Transmit에서 갱신.
 *        외부에서 헬스 판정 시:
 *          - rx_err_cnt 가 일정 임계 이상 → 수신 단절
 *          - tx_err_cnt 가 일정 임계 이상 → 송신 단절
 *          - state.error_code != 0       → 드라이버 자체 에러
 */
typedef struct {
    uint8_t  tx_err_cnt;          /**< 연속 송신 실패 횟수                    */
    uint8_t  rx_err_cnt;          /**< 연속 수신 타임아웃 횟수                 */
    uint32_t last_rx_tick;        /**< 마지막 RX 시각 (HAL_GetTick 단위)      */
} AK_Error_t;

/**
 * @brief AK 모터 한 대를 표현하는 객체 핸들.
 *
 *  배열로 선언하여 여러 모터 관리:
 *    CAN_Ak_Handle_t ECU_AK[NUM_AK_MOTORS];
 *  CAN 버스가 여러 개면 .hcan을 모터마다 다르게 주입할 수 있음.
 *
 *  volatile: cmd/state/error 는 RX 콜백(ISR) 및 TX task와 공유되므로 필수.
 */
typedef struct {
    CAN_HandleTypeDef *hcan;         /**< 사용할 CAN 핸들 (의존성 주입)        */
    uint8_t            CAN_ID;       /**< 모터 드라이버 ID (1~254)             */
    volatile AK_CMD_t   cmd;         /**< 송신 명령 버퍼 (사용자가 갱신)        */
    volatile AK_State_t state;       /**< 수신 상태 (RX 콜백이 갱신)            */
    volatile AK_Error_t error;       /**< 통신 헬스 카운터                      */
} CAN_Ak_Handle_t;

/**
 * @brief CAN TX 큐 엔트리. CAN_AK_Transmit이 이 형태로 빌드해서
 *        Queue에 넣거나(RTOS) 즉시 송신(Bare-metal)함.
 */
typedef struct {
    CAN_HandleTypeDef  *hcan;        /**< 송신에 사용할 CAN 핸들               */
    CAN_TxHeaderTypeDef TxHeader;    /**< StdId/ExtId/DLC 등                   */
    uint8_t             Data[8];     /**< 페이로드 (DLC 만큼만 유효)            */
    AK_Error_t         *pError;      /**< tx_err_cnt 갱신용 (최소 결합)         */
} CAN_Tx_Packet_t;

/**
 * @brief RX FIFO에서 한 번 꺼낸 결과 프레임.
 *        모듈 외부에서 모터 lookup용으로 driver_id를 사용.
 */
typedef struct {
    uint8_t driver_id;               /**< ExtId 하위 8bit (모터 CAN_ID)        */
    uint8_t data[8];                 /**< 페이로드 8byte 원본                  */
} AK_RxFrame_t;

/* Exported functions prototypes ---------------------------------------------*/

/* ============================================================================
 * RX callback 등록 예 (사용자 코드 측)
 * ----------------------------------------------------------------------------
 *  void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
 *      if (hcan->Instance == CAN1) {
 *          AK_RxFrame_t frame;
 *          if (CAN_AK_RX_POP(hcan, &frame)) {
 *              for (int i = 0; i < NUM_AK_MOTORS; i++) {
 *                  if (CAN_AK_RX_APPLY(&ECU_AK[i], &frame)) break;
 *              }
 *          }
 *      }
 *  }
 * ========================================================================== */

/* --- CAN 페리페럴 초기화/복구 --------------------------------------------- */

/**
 * @brief  CAN 페리페럴을 시작 (필터 + 인터럽트 + HAL_CAN_Start).
 *         모든 모터 CAN_AK_INIT 호출 전에 1회 호출.
 * @param  hcan  대상 CAN 핸들 (예: &hcan1)
 * @retval HAL_OK / HAL_ERROR
 */
HAL_StatusTypeDef CAN_Init(CAN_HandleTypeDef *hcan);

/**
 * @brief  Bus-off 등 치명적 에러 후 CAN을 재초기화.
 *         Stop → DeInit → Init → CAN_Init 순서, Queue도 reset.
 * @param  hcan  대상 CAN 핸들
 * @retval HAL_OK / HAL_ERROR
 */
HAL_StatusTypeDef CAN_RECOVERY(CAN_HandleTypeDef *hcan);

/* --- 모터 핸들 초기화/송신/감시 ------------------------------------------- */

/**
 * @brief  모터 핸들 1개 초기화. cmd/state/error 를 모두 0으로 클리어하고,
 *         cmd.mode = MODE_ESTOP, last_rx_tick = HAL_GetTick() 로 세팅.
 * @param  pMotor  대상 모터 핸들
 * @param  hcan    이 모터가 매달릴 CAN 핸들
 * @param  can_id  모터 드라이버 CAN ID (1~254)
 */
void CAN_AK_INIT(CAN_Ak_Handle_t *pMotor, CAN_HandleTypeDef *hcan, uint8_t can_id);

/**
 * @brief  pMotor->cmd 에 채워진 값을 바탕으로 한 프레임을 송신.
 *
 *  - cmd.mode 에 따라 사용할 필드와 페이로드 포맷이 정해짐 (AK_Control_Mode_t 참조).
 *  - 입력 범위는 내부에서 자동 clamp (AK_MAX_* 사용).
 *  - MODE_ESTOP / 미지원 mode는 송신 skip (반환만).
 *  - 실제 송신 경로는 USE_RTOS_CAN_QUEUE 매크로에 따라 Queue 또는 즉시 송신.
 *
 * @param pMotor  명령 채워진 모터 핸들
 */
void CAN_AK_WRITE(CAN_Ak_Handle_t *pMotor);

/**
 * @brief  CAN RX 타임아웃 감시. 마지막 RX로부터 CAN_RX_TIMEOUT ms 이상
 *         경과했으면 rx_err_cnt 1 증가 + is_running = 0.
 *         호출자는 controlTask 등 주기 태스크에서 모터별로 호출.
 *
 * @param pMotor        대상 모터
 * @param current_tick  현재 시각 (보통 HAL_GetTick())
 */
void CAN_AK_CHECKER(CAN_Ak_Handle_t *pMotor, uint32_t current_tick);

/* --- RX 처리 (split: Pop → Apply) ----------------------------------------- */

/**
 * @brief  CAN RX FIFO에서 한 프레임을 꺼내 AK_RxFrame_t 로 변환.
 *         RX 콜백 안에서 한 번 호출하고, 반환된 frame을 모터별로 Apply.
 * @param  hcan   인터럽트가 발생한 CAN 핸들
 * @param  frame  결과를 받을 버퍼 (NULL 금지)
 * @retval 1  프레임 추출 성공
 * @retval 0  메시지 없음 / 인자 오류
 */
uint8_t CAN_AK_RX_POP(CAN_HandleTypeDef *hcan, AK_RxFrame_t *frame);

/**
 * @brief  frame.driver_id 가 pMotor->CAN_ID 와 일치하면 state/error 갱신.
 *         일치하지 않으면 아무것도 하지 않음.
 *         호출자가 자기 컬렉션(배열/맵/단일)을 순회해서 일치하는 첫 모터에 적용.
 * @param  pMotor  적용 대상 후보 모터
 * @param  frame   RX_Pop 결과 (NULL 금지)
 * @retval 1  일치 → state 업데이트됨 (caller는 루프 break 가능)
 * @retval 0  불일치 / 인자 오류
 */
uint8_t CAN_AK_RX_APPLY(CAN_Ak_Handle_t *pMotor, const AK_RxFrame_t *frame);

/* --- (RTOS only) TX 큐 처리 태스크 ---------------------------------------- */

/**
 * @brief  Queue에 쌓인 TX 패킷을 메일박스로 흘려보내는 태스크 본체.
 *         RTOS 태스크에서 무한 루프로 호출:
 *
 *           void Start_can1(void *argument) {
 *               for (;;) CAN_AK_TX_TASK_HANDLER();
 *           }
 *
 *         USE_RTOS_CAN_QUEUE 정의 시에만 구현되어 있음.
 */
void CAN_AK_TX_TASK_HANDLER(void);

#endif /* INC_CAN_AK_H_ */
