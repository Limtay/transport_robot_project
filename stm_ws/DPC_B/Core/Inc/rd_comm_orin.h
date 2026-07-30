/**
 ******************************************************************************
 * @file    rd_comm_orin.h
 * @author  swarm
 * @date    2026-06-25
 * @brief   Orin RS485 (USART2) Dynamixel Protocol 2.0-like 패킷 빌더/파서 — DPC_B용.
 *
 *  와이어 포맷:
 *      [0:1]  0xAA 0x55  Header
 *      [2]    ID         요청 = ORIN_MY_ID(0xE2) / 응답 = ORIN_MASTER_ID(0x01)
 *      [3:4]  Length     L|H = Instruction(1) + Parameter(N) + CRC(2) = N + 3
 *      [5]    Instruction  READ=0x02 / WRITE=0x03
 *      [6..N+5] Parameter  가변
 *      [N+6:N+7] CRC      CRC-16/IBM, little-endian
 *
 *  ECU_V3 의 rd_comm_ecu.h 와 동일한 와이어 포맷을 따른다.
 *  단, 기존 rd_comm_dpcb.h (DPCA 4-byte 패킷) 과의 타입명 충돌 방지를 위해
 *  모든 식별자를 ORIN_* 접두사로 통일한다.
 ******************************************************************************
 */

#ifndef INC_RD_COMM_ORIN_H_
#define INC_RD_COMM_ORIN_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "rd_common.h"
#include "rd_define.h"
#include "rd_uart.h"

/* ── Header magic ────────────────────────────────────────────────────────────*/
#define ORIN_HEADER1                0xAA
#define ORIN_HEADER2                0x55

/* ── 노드 ID ─────────────────────────────────────────────────────────────────
 *  ORIN_MY_ID     : DPC_B RS485 노드 주소 (ECU_V3 = 0xE1, DPC_B = 0xE2).
 *  ORIN_MASTER_ID : 마스터(Orin AGX) 주소. 응답 패킷의 TargetID 고정값.
 * ──────────────────────────────────────────────────────────────────────────*/
#define ORIN_MY_ID                  0xE2
#define ORIN_MASTER_ID              0x01

/* ── 버퍼 / 오프셋 상수 ──────────────────────────────────────────────────────
 *  ORIN_ID_IDX       : raw 버퍼에서 ID 필드 바이트 인덱스 (Header 2B 다음).
 *  ORIN_HEADER_SIZE  : Header(2)+ID(1)+Length(2) = 5B.
 *  ORIN_DATA_BUF_SIZE: Parameter 페이로드 최대 크기.
 *                      READ 응답 시 Data[0] 이 err 바이트로 예약되므로
 *                      요청 가능한 최대 rlen = ORIN_DATA_BUF_SIZE - 1 = 89B.
 * ──────────────────────────────────────────────────────────────────────────*/
#define ORIN_ID_IDX                 2
#define ORIN_HEADER_SIZE            5
#define ORIN_DATA_BUF_SIZE          90

/* ── Instruction 코드 ────────────────────────────────────────────────────────*/
#define ORIN_INST_PING              0x01
#define ORIN_INST_READ              0x02
#define ORIN_INST_WRITE             0x03
#define ORIN_INST_REBOOT            0x08

/* ── 패킷 구조체 ─────────────────────────────────────────────────────────────
 *  ORIN_PKT_t 는 와이어 포맷 [2..N+5] 에 1:1 대응.
 *  data_len : 소프트웨어 전용 (와이어 미포함) = Parameter 바이트 수 = Length - 3
 * ──────────────────────────────────────────────────────────────────────────*/
typedef struct __attribute__((packed)) {
    uint8_t  TargetID;
    uint16_t Length;                       /* 와이어 값: data_len + 3 */
    uint8_t  Instruction;
    uint8_t  Data[ORIN_DATA_BUF_SIZE];    /* Parameter 페이로드 */
    /* --- 소프트웨어 전용 (와이어 미포함) --- */
    uint16_t data_len;
} ORIN_PKT_t;

/* ── 통신 채널 핸들 ──────────────────────────────────────────────────────────*/
typedef struct {
    ORIN_PKT_t tx;
    ORIN_PKT_t rx;
    uint8_t    reboot_pending;   /* REBOOT 응답 송신 후 NVIC_SystemReset 대기 플래그 */
} ORIN_COMM_t;

/* Exported functions --------------------------------------------------------*/

/** @brief ORIN_COMM_t tx/rx 버퍼를 0 으로 초기화. */
RD_RET RD_ORIN_INIT(ORIN_COMM_t *comm);

/**
 * @brief  RS485 수신 버퍼에서 패킷 1개를 파싱하여 comm->rx 에 저장.
 * @retval RET_OK   파싱 성공
 * @retval RET_WAIT 수신 없음 / 유효하지 않은 패킷 (재시도 불필요)
 * @retval RET_NOK  인자 오류
 */
RD_RET RD_ORIN_READ(RS485_t *rs485_obj, ORIN_COMM_t *comm);

/**
 * @brief  rx.Instruction 분기 → 레지스터 맵 Dispatch → tx 응답 구성.
 *         lock=1 이면 CMD_MOT 영역 WRITE 를 거부 (ORIN_ERR_ACCESS).
 */
RD_RET RD_ORIN_HANDLE(ORIN_COMM_t *comm, uint8_t lock);

/**
 * @brief  comm->tx 를 직렬화하여 RS485 로 송신.
 * @retval RD_RS485_TRANSMIT 반환값 전파.
 */
RD_RET RD_ORIN_WRITE(RS485_t *rs485_obj, ORIN_COMM_t *comm);

#endif /* INC_RD_COMM_ORIN_H_ */
