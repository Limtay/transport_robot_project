/**
 ******************************************************************************
 * @file    rd_comm_ecu.h
 * @author  swarm
 * @date    2026-03-10
 * @brief   Dynamixel Protocol 2.0 모방 패킷 빌더 / 파서 — 공개 인터페이스.
 *
 *  와이어 포맷 요약:
 *      [0:1]  0xAA 0x55  Header
 *      [2]    ID         요청 = PACKET_MY_ID(0xE1) / 응답 = PACKET_MASTER_ID(0x01)
 *      [3:4]  Length     L|H = Instruction(1) + Parameter(N) + CRC(2) = N + 3
 *      [5]    Instruction  READ=0x02 / WRITE=0x03  (응답 시 원 요청 Instruction 복사)
 *      [6..N+5] Parameter  가변
 *      [N+6:N+7] CRC      CRC-16/IBM, little-endian
 *
 *  ※ 실제 Dynamixel 과 프레임 구조만 참고했으며 프로토콜 레벨 호환 없음.
 *     식별자 접두사 PACKET_* 사용
 ******************************************************************************
 */

#ifndef INC_RD_COMM_ECU_H_
#define INC_RD_COMM_ECU_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "rd_common.h"
#include "rd_define.h"
#include "rd_uart.h"

/* ── Header magic (raw 값, enum 부적합) ──────────────────────────────────────*/
#define PACKET_HEADER1              0xAA
#define PACKET_HEADER2              0x55

/* ── 노드 ID ─────────────────────────────────────────────────────────────────
 *  PACKET_MY_ID     : 이 ECU 의 RS485 주소. 수신 시 ID 필터링에 사용.
 *  PACKET_MASTER_ID : 마스터(Orin) 주소. 응답 패킷의 TargetID 고정값.
 * ──────────────────────────────────────────────────────────────────────────*/
#define PACKET_MY_ID                0xE1
#define PACKET_MASTER_ID            0x01

/* ── 버퍼 / 오프셋 상수 ──────────────────────────────────────────────────────
 *  PACKET_ID_IDX    : raw 버퍼에서 ID 필드의 바이트 인덱스 (Header 2B 다음).
 *  PACKET_HEADER_SIZE : Header(2)+ID(1)+Length(2) = 5B, Length 필드 이전까지.
 *  PACKET_DATA_BUF_SIZE : Parameter 페이로드 최대 크기.
 *                         READ 응답 시 Data[0] 이 err 바이트로 예약되므로
 *                         요청 가능한 최대 rlen = PACKET_DATA_BUF_SIZE - 1.
 * ──────────────────────────────────────────────────────────────────────────*/
#define PACKET_ID_IDX               2
#define PACKET_HEADER_SIZE          5
#define PACKET_DATA_BUF_SIZE        90

/* ── Instruction 코드 ────────────────────────────────────────────────────────*/
#define PACKET_INST_PING            0x01  /* alive check: Param 없음, Err(1) 응답      */
#define PACKET_INST_READ            0x02  /* reg 읽기: Param = AddrL|H + LenL|H (4B)  */
#define PACKET_INST_WRITE           0x03  /* reg 쓰기: Param = AddrL|H + Data[N]      */
#define PACKET_INST_REBOOT          0x08  /* NVIC_SystemReset: 응답 송신 후 리셋       */

/* ── 패킷 구조체 ─────────────────────────────────────────────────────────────
 *  PACKET_PACKET_t 는 와이어 포맷 [2..N+5] 구간에 1:1 대응한다.
 *  단, data_len 은 소프트웨어 전용 메타 필드로 와이어에 포함되지 않는다.
 *      data_len = Parameter 바이트 수 (Instruction / CRC 제외)
 *               = Length - 3
 *  memcpy 시 복사 크기 = 4 + data_len  (ID 1B + Length 2B + Instruction 1B + Param)
 * ──────────────────────────────────────────────────────────────────────────*/
typedef struct __attribute__((packed)) {
    uint8_t  TargetID;
    uint16_t Length;                          /* 와이어 값: data_len + 3 */
    uint8_t  Instruction;
    uint8_t  Data[PACKET_DATA_BUF_SIZE];      /* Parameter 페이로드 */
    /* --- 소프트웨어 전용 (와이어 미포함) --- */
    uint16_t data_len;
} PACKET_PACKET_t;

/* ── 통신 채널 핸들 ──────────────────────────────────────────────────────────
 * ──────────────────────────────────────────────────────────────────────────*/
typedef struct {
    PACKET_PACKET_t tx;
    PACKET_PACKET_t rx;
    uint8_t reboot_pending;    /* REBOOT 응답 송신 후 NVIC_SystemReset 대기 플래그 */
} PACKET_comm_t;

/* Exported functions --------------------------------------------------------*/

/** @brief tx/rx 버퍼와 에러 카운터를 0 으로 초기화. */
RD_RET RD_PACKET_INIT(PACKET_comm_t *packet_obj);

/**
 * @brief  RS485 수신 버퍼에서 패킷 1개를 파싱하여 packet_obj->rx 에 저장.
 * @retval RET_OK   파싱 성공 — RD_PACKET_HANDLE 호출 가능
 * @retval RET_WAIT 수신 데이터 없음 또는 유효하지 않은 패킷
 * @retval RET_NOK  인자 오류
 */
RD_RET RD_PACKET_READ(RS485_t *rs485_obj, PACKET_comm_t *packet_obj);

/**
 * @brief  rx.Instruction 을 분기하여 레지스터 맵 Dispatch 후 tx 응답을 구성.
 *         성공/실패 무관하게 항상 RET_OK 반환 — 에러는 tx.Data[0] (PACKET_Error_e) 으로 전달.
 */
RD_RET RD_PACKET_HANDLE(PACKET_comm_t *packet_obj, uint8_t lock);

/**
 * @brief  packet_obj->tx 를 직렬화하여 RS485 로 송신.
 * @retval RD_RS485_TRANSMIT 반환값 그대로 전파 (RET_OK / RET_WAIT / RET_NOK).
 */
RD_RET RD_PACKET_WRITE(RS485_t *rs485_obj, PACKET_comm_t *packet_obj);

#endif /* INC_RD_COMM_ECU_H_ */
