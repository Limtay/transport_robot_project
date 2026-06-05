/**
 ******************************************************************************
 * @file    can_ak.c
 * @author  Kyeongtae
 * @date    2026-05-21
 * @brief   CubeMars AK 모터 Servo Mode CAN 드라이버 구현부.
 *
 *  파일 구성
 *  -----------------------------------------------------------------------
 *    [Static helpers]    클램프 / 빅엔디안 직렬화 / 필터 설정
 *    [TX path]           Transmit (Queue or Direct) → Direct
 *    [RX path]           RX_Pop (FIFO → frame)
 *                        RX_Apply (frame → motor state)
 *    [Lifecycle]         CAN_Init / CAN_RECOVERY / CAN_AK_INIT
 *    [Control entry]     CAN_AK_WRITE (mode switch)
 *    [Health monitor]    CAN_AK_CHECKER (timeout 감지)
 *    [RTOS-only]         CAN_AK_TX_TASK_HANDLER
 *
 *  공유 변수 / 동시성 메모
 *  -----------------------------------------------------------------------
 *    - pMotor->state, error : RX 콜백(ISR)에서 갱신 → volatile 필수
 *    - pMotor->cmd          : 사용자 태스크에서 갱신, WRITE에서 read-only 소비
 *    - canTxQueueHandle     : RTOS 모드에서 ISR/태스크 양쪽에서 사용 (osMessageQueue가 동기화)
 *
 *  와이어 포맷
 *  -----------------------------------------------------------------------
 *    ExtId (29 bit) = (mode_id << 8) | CAN_ID
 *    Data           = 빅엔디안 직렬화, DLC = 실제 페이로드 길이
 *      mode_id == MODE_VELOCITY  → int32_t  rpm                        (4B)
 *      mode_id == MODE_CURRENT*  → int32_t (current[A] * 1000)         (4B)
 *      mode_id == MODE_POSITION  → int32_t (pos[deg] * 10000)          (4B)
 *      mode_id == MODE_POS_VEL_LOOP → int32 pos×10000 | int16 spd/10
 *                                     | int16 acc/10                   (8B)
 *      mode_id == MODE_SET_ORIGIN → uint8_t set_origin_mode            (1B)
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "can_ak.h"
#include <string.h>

/* Static function prototypes ------------------------------------------------*/
static float clampf(float v, float lo, float hi);
static void  buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index);
static void  buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index);

static void  CAN_Filter_Config(CAN_HandleTypeDef *hcan);
static void  CAN_AK_Transmit (CAN_Ak_Handle_t *pMotor, uint8_t mode_id, uint8_t *data, uint8_t len);
__attribute__((unused))
static void  CAN_AK_Direct   (CAN_Ak_Handle_t *pMotor, CAN_Tx_Packet_t tx_packet);

/* ============================================================================
 *                            Static helpers
 * ========================================================================== */

/** @brief [lo, hi] 범위로 float 값을 자름. */
static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/** @brief int32를 빅엔디안으로 buffer[*index]에 4byte 직렬화, index 4 증가. */
static void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
    buffer[(*index)++] = (uint8_t)(number >> 24);
    buffer[(*index)++] = (uint8_t)(number >> 16);
    buffer[(*index)++] = (uint8_t)(number >> 8);
    buffer[(*index)++] = (uint8_t)(number);
}

/** @brief int16을 빅엔디안으로 buffer[*index]에 2byte 직렬화, index 2 증가. */
static void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index) {
    buffer[(*index)++] = (uint8_t)(number >> 8);
    buffer[(*index)++] = (uint8_t)(number);
}

/**
 * @brief  CAN 필터 1뱅크 등록.
 *         USE_CAN_IDMASK_FILTER 정의 시 AK10-9 mode 0x29 / driver 1~F 만 통과.
 *         (filter_id 0x2901 + mask 0x1FFFFFF0  → 상위 모드는 0x29 고정,
 *          하위 driver_id 4bit는 임의).
 *         실패 시 Error_Handler() 호출.
 */
static void CAN_Filter_Config(CAN_HandleTypeDef *hcan)
{
    CAN_FilterTypeDef canFilter;

    canFilter.FilterBank           = 0;
    canFilter.FilterMode           = CAN_FILTERMODE_IDMASK;   /* ID MASK 모드   */
    canFilter.FilterScale          = CAN_FILTERSCALE_32BIT;   /* Extended ID    */
    canFilter.FilterFIFOAssignment = CAN_AK_FILTER_FIFO;      /* FIFO 할당      */
    canFilter.FilterActivation     = ENABLE;

#ifdef USE_CAN_IDMASK_FILTER
    /* AK10-9 mode 0x29 + driver_id 하위 nibble 임의 통과.
     * 32bit 레지스터 정렬: ExtId는 <<3 한 뒤 IDE/RTR 플래그 + 4(IDE bit). */
    uint32_t filter_id   = (0x2901u     << 3) | 4;
    uint32_t filter_mask = (0x1FFFFFF0u << 3) | 4;
    canFilter.FilterIdHigh     = (filter_id   >> 16) & 0xFFFF;
    canFilter.FilterIdLow      = (filter_id        ) & 0xFFFF;
    canFilter.FilterMaskIdHigh = (filter_mask >> 16) & 0xFFFF;
    canFilter.FilterMaskIdLow  = (filter_mask      ) & 0xFFFF;
#else
    /* FreePass: 모든 ID 통과 */
    canFilter.FilterIdHigh     = 0x0000;
    canFilter.FilterIdLow      = 0x0000;
    canFilter.FilterMaskIdHigh = 0x0000;
    canFilter.FilterMaskIdLow  = 0x0000;
#endif
    if (HAL_CAN_ConfigFilter(hcan, &canFilter) != HAL_OK) {
        Error_Handler();
    }
}

/* ============================================================================
 *                                TX path
 * ========================================================================== */

/**
 * @brief  ExtId/DLC/Data를 빌드해서 송신 경로(Queue or Direct)로 전달.
 *         len > 8 인 입력은 8로 잘림 (안전장치).
 *
 *  - RTOS 모드 : canTxQueue 에 put. 여유공간이 MIN_QUEUE 미만이면 가장
 *    오래된 패킷을 1개 버리고(tx_err_cnt++) 새 패킷을 넣음.
 *  - Bare-metal: 메일박스로 즉시 송신 (CAN_AK_Direct).
 */
static void CAN_AK_Transmit(CAN_Ak_Handle_t *pMotor, uint8_t mode_id, uint8_t *data, uint8_t len) {
    if (len > 8) len = 8;

    CAN_Tx_Packet_t tx_packet;
    tx_packet.hcan                       = pMotor->hcan;
    tx_packet.TxHeader.DLC               = len;
    tx_packet.TxHeader.StdId             = 0x00;
    tx_packet.TxHeader.IDE               = CAN_ID_EXT;
    tx_packet.TxHeader.RTR               = CAN_RTR_DATA;
    tx_packet.TxHeader.TransmitGlobalTime = DISABLE;
    tx_packet.TxHeader.ExtId             = ((uint32_t)mode_id << 8) | pMotor->CAN_ID;
    memcpy(tx_packet.Data, data, len);
    tx_packet.pError                     = &pMotor->error;  /* 카운터 갱신용  */

#ifdef USE_RTOS_CAN_QUEUE
    /* Queue 포화 시 오래된 패킷부터 drop (실시간 제어에서 stale 명령 회피). */
    if (osMessageQueueGetSpace(canTxQueueHandle) < MIN_QUEUE) {
        CAN_Tx_Packet_t trash_packet;
        osMessageQueueGet(canTxQueueHandle, &trash_packet, NULL, 0);
        pMotor->error.tx_err_cnt++;
    }
    if (osMessageQueuePut(canTxQueueHandle, &tx_packet, 0, 0) != osOK) {
        pMotor->error.tx_err_cnt++; /* enqueue 실패, 리셋은 canTask에서 한 곳에서 진행 */
    }
#else
    /* RTOS 미사용 시: 메일박스로 곧장 송신 */
    CAN_AK_Direct(pMotor, tx_packet);
#endif
}

/**
 * @brief  메일박스 여유가 있으면 한 프레임을 즉시 송신.
 *         실패 시 tx_err_cnt++, 성공 시 0으로 리셋.
 */
static void CAN_AK_Direct(CAN_Ak_Handle_t *pMotor, CAN_Tx_Packet_t tx_packet) {
    if (HAL_CAN_GetTxMailboxesFreeLevel(tx_packet.hcan) > 0) {
        uint32_t TxMailbox;
        if (HAL_CAN_AddTxMessage(tx_packet.hcan, &tx_packet.TxHeader,
                                 tx_packet.Data, &TxMailbox) != HAL_OK) {
            pMotor->error.tx_err_cnt++;       /* AddTxMessage 자체 실패        */
        } else {
            pMotor->error.tx_err_cnt = 0;     /* 정상 송신                     */
        }
    } else {
        pMotor->error.tx_err_cnt++;           /* 메일박스 가득 → 송신 실패     */
    }
}

/* ============================================================================
 *                                RX path
 * ========================================================================== */

/**
 * @brief  FIFO에서 한 프레임을 꺼내 AK_RxFrame_t 로 변환. (헤더 doc 참조)
 *         이 함수는 모터 컬렉션을 모름 → 호출자가 driver_id 로 디스패치.
 */
uint8_t CAN_AK_RX_POP(CAN_HandleTypeDef *hcan, AK_RxFrame_t *frame) {
    if (frame == NULL) return 0;

    CAN_RxHeaderTypeDef RxHeader;
    if (HAL_CAN_GetRxMessage(hcan, CAN_AK_RX_FIFO, &RxHeader, frame->data) != HAL_OK) {
        return 0;
    }
    /* ExtId = (mode_id << 8) | driver_id → 하위 8bit가 driver_id */
    frame->driver_id = (uint8_t)(RxHeader.ExtId & 0xFF);
    return 1;
}

/**
 * @brief  frame이 이 모터에 해당하면 state/error 를 갱신.
 *
 *  RX 페이로드 포맷 (datasheet 4.1):
 *    [0..1] position  int16 BE   (×0.1f → deg)
 *    [2..3] velocity  int16 BE   (×10.0f → RPM)
 *    [4..5] current   int16 BE   (×0.01f → A)
 *    [6]    temp      int8       (°C)
 *    [7]    err_code  uint8      (AK_ERROR_CODE_e)
 */
uint8_t CAN_AK_RX_APPLY(CAN_Ak_Handle_t *pMotor, const AK_RxFrame_t *frame) {
    if (pMotor == NULL || frame == NULL) return 0;
    if (frame->driver_id != pMotor->CAN_ID) return 0;

    const uint8_t *d = frame->data;
    /* C-3: 5개 필드를 로컬 tmp 에 먼저 빌드 후 memcpy 단일 store.
     *       태스크가 struct copy 하는 도중 ISR 가 개별 필드를 덮는 torn-read 방지. */
    AK_State_t tmp;
    tmp.position   = (int16_t)((d[0] << 8) | d[1]);
    tmp.velocity   = (int16_t)((d[2] << 8) | d[3]);
    tmp.current    = (int16_t)((d[4] << 8) | d[5]);
    tmp.temp_motor = (int8_t) d[6];
    tmp.error_code = (AK_ERROR_CODE_e)d[7];   /* L-13: uint8 → enum 명시 캐스팅 */
    memcpy((void*)&pMotor->state, &tmp, sizeof(AK_State_t));

    pMotor->error.last_rx_tick = HAL_GetTick();
    pMotor->error.rx_err_cnt   = 0;
    return 1;
}

/* ============================================================================
 *                              Lifecycle
 * ========================================================================== */

/**
 * @brief  모터 핸들 초기화. cmd.mode 기본값은 MODE_ESTOP(송신 skip).
 *         last_rx_tick = 0 (미수신 표시 — 첫 RX 전까지 LS_RUNNING 승격 방지).
 */
void CAN_AK_INIT(CAN_Ak_Handle_t *pMotor, CAN_HandleTypeDef *hcan, uint8_t can_id) {
    pMotor->hcan   = hcan;
    pMotor->CAN_ID = can_id;
    memset((void*)&pMotor->cmd,   0, sizeof(AK_CMD_t));
    memset((void*)&pMotor->state, 0, sizeof(AK_State_t));
    memset((void*)&pMotor->error, 0, sizeof(AK_Error_t));
    pMotor->cmd.mode           = MODE_ESTOP;
    pMotor->error.last_rx_tick = 0;
}

/**
 * @brief  CAN 페리페럴을 사용 가능 상태로 만들기:
 *         필터 등록 → HAL_CAN_Start → RX/에러 인터럽트 활성화.
 */
HAL_StatusTypeDef CAN_Init(CAN_HandleTypeDef *hcan)
{
    CAN_Filter_Config(hcan);
    if (HAL_CAN_Start(hcan) != HAL_OK) return HAL_ERROR;

    HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
    HAL_CAN_ActivateNotification(hcan,
        CAN_IT_ERROR_WARNING | CAN_IT_ERROR_PASSIVE |
        CAN_IT_BUSOFF        | CAN_IT_LAST_ERROR_CODE | CAN_IT_ERROR);
    return HAL_OK;
}

/**
 * @brief  Bus-off 등 복구가 필요할 때 CAN 페리페럴 전체를 재초기화.
 *         Stop → DeInit → HAL_CAN_Init → CAN_Init(필터/인터럽트 재등록),
 *         RTOS 모드에서는 TX Queue도 reset.
 */
HAL_StatusTypeDef CAN_RECOVERY(CAN_HandleTypeDef *hcan)
{
    if (hcan == NULL) return HAL_ERROR;

    HAL_CAN_Stop(hcan);
    HAL_CAN_DeInit(hcan);
    if (HAL_CAN_Init(hcan) != HAL_OK) return HAL_ERROR;

#ifdef USE_RTOS_CAN_QUEUE
    osMessageQueueReset(canTxQueueHandle);
#endif
    return CAN_Init(hcan);
}

/* ============================================================================
 *                            Control entry
 * ========================================================================== */

/**
 * @brief  pMotor->cmd 의 mode/필드를 읽어 한 프레임을 송신.
 *
 *  모드별 사용 필드 / 범위 / 스케일:
 *    MODE_CURRENT        cmd.current [-AK_MAX_CUR, +AK_MAX_CUR]  ×1000 → int32
 *    MODE_CURRENT_BRAKE  cmd.current [0,           +AK_MAX_CUR]  ×1000 → int32
 *    MODE_VELOCITY       cmd.rpm     [-AK_MAX_RPM, +AK_MAX_RPM]         int32
 *    MODE_POSITION       cmd.pos     [-AK_MAX_POS, +AK_MAX_POS]  ×10000→ int32
 *    MODE_POS_VEL_LOOP   pos(×10000) | spd/10 (int16) | acc/10 (int16)
 *    MODE_SET_ORIGIN     cmd.set_origin_mode (uint8 1byte)
 *    MODE_ESTOP / 그 외  송신 안 함 (return)
 *
 *  범위 초과 입력은 내부 clampf 로 자동 보정. 호출자는 클램프 책임 없음.
 */
void CAN_AK_WRITE(CAN_Ak_Handle_t *pMotor) {
    uint8_t buffer[8];
    int32_t idx = 0;
    uint8_t mode_id = pMotor->cmd.mode;

    switch (mode_id) {
        case MODE_CURRENT: {
            float c = clampf(pMotor->cmd.current, -AK_MAX_CUR, AK_MAX_CUR);
            buffer_append_int32(buffer, (int32_t)(c * 1000.0f), &idx);
            break;
        }
        case MODE_CURRENT_BRAKE: {
            float c = clampf(pMotor->cmd.current, 0.0f, AK_MAX_CUR);
            buffer_append_int32(buffer, (int32_t)(c * 1000.0f), &idx);
            break;
        }
        case MODE_VELOCITY: {
            float rpm = clampf(pMotor->cmd.rpm, -AK_MAX_RPM, AK_MAX_RPM);
            buffer_append_int32(buffer, (int32_t)rpm, &idx);
            break;
        }
        case MODE_POSITION: {
            float pos = clampf(pMotor->cmd.pos, -AK_MAX_POS, AK_MAX_POS);
            buffer_append_int32(buffer, (int32_t)(pos * 10000.0f), &idx);
            break;
        }
        case MODE_POS_VEL_LOOP: {
            float pos = clampf(pMotor->cmd.pos, -AK_MAX_POS, AK_MAX_POS);
            float spd = clampf(pMotor->cmd.rpm, -AK_MAX_VEL, AK_MAX_VEL);
            float acc = clampf(pMotor->cmd.acc, 0.0f,        AK_MAX_ACC);
            buffer_append_int32(buffer, (int32_t)(pos * 10000.0f), &idx);
            /* int16에 맞추려고 ÷10. 와이어 단위는 ERPM, ERPM/s. */
            buffer_append_int16(buffer, (int16_t)(spd / 10.0f), &idx);
            buffer_append_int16(buffer, (int16_t)(acc / 10.0f), &idx);
            break;
        }
        case MODE_SET_ORIGIN: {
            buffer[idx++] = pMotor->cmd.set_origin_mode;
            break;
        }
        case MODE_ESTOP:
        default:
            return;   /* 송신 skip — 소프트 정지/미지원 모드 */
    }

    CAN_AK_Transmit(pMotor, mode_id, buffer, (uint8_t)idx);
}

/* ============================================================================
 *                           Health monitor
 * ========================================================================== */

/**
 * @brief  주기 호출용 헬스 체커 (per-motor).
 *         RX 타임아웃 감지: 마지막 RX 이후 CAN_RX_TIMEOUT_MS 이상 경과 시 rx_err_cnt 1 증가
 *         (0xFF saturate). last_rx_tick 은 옮기지 않으므로 단절 지속 시 매 tick 증가 →
 *         상위 RD_CAN_MOTOR_CHECKER 가 rx_err_cnt>0 을 AK_COMM_RX_BIT 으로, tx_err_cnt>0 을
 *         AK_COMM_TX_BIT 으로 묶어 data->comm_err / degraded_cnt 에 반영한다.
 *
 *         모터 자체 에러(과열/과전류 등) 는 state.error_code 채널이 별도 보관.
 *         외부 마스터는 comm_err(통신) + state.error_code(모터 fault) 를 함께 봐야 함.
 */
void CAN_AK_CHECKER(CAN_Ak_Handle_t *pMotor, uint32_t current_tick)
{
    if (pMotor == NULL) return;
    /* 1) RX 타임아웃 감지 */
    if ((current_tick - pMotor->error.last_rx_tick) > CAN_RX_TIMEOUT_MS) {
        if (pMotor->error.rx_err_cnt < 0xFF) pMotor->error.rx_err_cnt++;
    }
}

/* ============================================================================
 *                          RTOS-only TX task
 * ========================================================================== */

#ifdef USE_RTOS_CAN_QUEUE
/**
 * @brief  TX Queue 1개를 꺼내 메일박스로 흘려보냄. 메일박스가 가득이면
 *         최대 3 tick까지 osDelay(1) 으로 양보하면서 대기.
 *         호출자(RTOS 태스크)는 무한 루프에서 이 함수를 반복 호출.
 *
 *         실패/성공에 따라 dequeued_packet.pError->tx_err_cnt 갱신
 *         (pError NULL이면 무시).
 */
void CAN_AK_TX_TASK_HANDLER(void) {
    CAN_Tx_Packet_t dequeued_packet;

    /* 1) 큐에서 TX 패킷 1개 꺼내기 (없으면 무한 대기) */
    if (osMessageQueueGet(canTxQueueHandle, &dequeued_packet, NULL, osWaitForever) != osOK) {
        return;
    }

    /* 2) 메일박스 여유 생길 때까지 짧게 대기 (최대 3 tick) */
    uint8_t wait_timeout = 0;
    while (HAL_CAN_GetTxMailboxesFreeLevel(dequeued_packet.hcan) == 0) {
       osDelay(1);
       if (++wait_timeout > 2) {
    	   if (dequeued_packet.pError != NULL)
    		   dequeued_packet.pError->tx_err_cnt++;
    	   return;
        }   /* 너무 길게 막히면 포기 */
    }

    /* 3) 송신 + 에러 카운트 갱신 */
    uint32_t TxMailbox;
    if (HAL_CAN_AddTxMessage(dequeued_packet.hcan, &dequeued_packet.TxHeader,
                             dequeued_packet.Data, &TxMailbox) != HAL_OK) {
        if (dequeued_packet.pError != NULL)
            dequeued_packet.pError->tx_err_cnt++;
    } else {
        if (dequeued_packet.pError != NULL)
            dequeued_packet.pError->tx_err_cnt = 0;
    }
}
#endif /* USE_RTOS_CAN_QUEUE */
