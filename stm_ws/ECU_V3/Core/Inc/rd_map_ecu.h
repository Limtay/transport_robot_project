/**
 ******************************************************************************
 * @file    rd_map_ecu.h
 * @author  Kyeongtae
 * @date    2026-01-22
 * @brief   레지스터 맵 디스패치 + 마샬 레이어 — 공개 인터페이스.
 *
 *  이 레이어는 두 가지 책임을 가진다:
 *
 *  1) Dispatch (외부 RS485 마스터 ↔ reg):
 *       RD_PACKET_HANDLE 이 READ/WRITE Instruction 을 받으면
 *       Dispatch_Read / Dispatch_Write 를 통해 256B reg 에 직접 접근.
 *       영역 경계·접근 권한·UNLOCK 조건을 s_regions[] LUT 로 일괄 검증.
 *
 *  2) Marshal (reg ↔ PERIPHERAL_t):
 *       Marshal_Publish  : PERIPHERAL.data + error 상태 → reg R/O 영역 발행 (updateTask)
 *       Marshal_Consume  : reg.cmd_motor → PERIPHERAL.cmd 적용 (controlTask)
 *       두 함수 모두 단위 변환·패킹은 taskENTER_CRITICAL 밖에서 수행하고
 *       reg 쓰기만 CRITICAL 구간 안에서 처리한다.
 *
 *  단일 source-of-truth: `REGISTER_t reg` (rd_map_ecu.c 에 정의된 전역 변수).
 *  다른 모듈은 이 레이어를 통하지 않고 reg 를 직접 수정해서는 안 된다.
 ******************************************************************************
 */

#ifndef INC_RD_MAP_ECU_H_
#define INC_RD_MAP_ECU_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
#include "rd_common.h"
#include "rd_define.h"
#include "rd_register_ecu.h"
#include "rd_peripheral_ecu.h"
#include "rd_comm_ecu.h"

/* ── 응답 에러 코드 ──────────────────────────────────────────────────────────
 *  Dynamixel Protocol 2.0 STATUS 패킷의 Error 필드에 해당.
 *  RD_PACKET_HANDLE 이 Dispatch 반환값을 그대로 tx->Data[0] 에 실어 마스터로 전달.
 *  실제 Dynamixel 과 혼동 방지 위해 PACKET_ 접두사 사용 (DYN_* 아님).
 * ──────────────────────────────────────────────────────────────────────────*/
typedef enum {
    PACKET_ERR_NONE        = 0x00,  /* 정상 처리 */
    PACKET_ERR_RESULT_FAIL = 0x01,  /* Instruction 처리 실패 */
    PACKET_ERR_INST        = 0x02,  /* 정의되지 않은 Instruction */
    PACKET_ERR_CRC         = 0x03,  /* CRC 불일치 */
    PACKET_ERR_DATA_RANGE  = 0x04,  /* 데이터 값이 min/max 범위 초과 */
    PACKET_ERR_DATA_LEN    = 0x05,  /* 데이터 길이 부족 또는 주소 범위 초과 */
    PACKET_ERR_DATA_LIMIT  = 0x06,  /* 데이터 값이 Limit 초과 */
    PACKET_ERR_ACCESS      = 0x07,  /* R/O 영역 쓰기 / 미정의 주소 / UNLOCK 미설정 */
} PACKET_Error_e;

/* ── 전역 레지스터 맵 ────────────────────────────────────────────────────────
 *  256 byte packed 구조체. 외부 마스터는 바이트 주소(0~255)로 접근하고
 *  내부 코드는 필드명으로 접근한다 (struct overlay).
 *  reg 는 rd_map_ecu.c 에 단 하나만 존재하며 이 extern 선언으로 공유된다.
 * ──────────────────────────────────────────────────────────────────────────*/
extern REGISTER_t reg;

/* Exported functions --------------------------------------------------------*/

/** @brief reg 전체를 0 으로 초기화 후 DEFINE 영역 기본값과 ctr_mode 기본값 설정. */
RD_RET  RD_MAP_INIT(void);

/**
 * @brief  주소 addr 부터 len 바이트를 src 에서 reg 로 쓴다.
 * @retval PACKET_Error_e (0x00 = 성공)
 * @note   영역 횡단·R/O 영역·UNLOCK 미설정 시 PACKET_ERR_ACCESS 반환.
 *         addr+len 이 256B 초과 시 PACKET_ERR_DATA_LEN 반환.
 *         성공 시 taskENTER_CRITICAL 로 보호된 memcpy 로 reg 를 갱신한다.
 */
uint8_t RD_MAP_DISPATCH_WRITE(uint16_t addr, uint16_t len, const uint8_t *src);

/**
 * @brief  주소 addr 부터 len 바이트를 reg 에서 dst 로 읽는다.
 * @retval PACKET_Error_e (0x00 = 성공)
 * @note   영역 횡단·W/O 영역 시 PACKET_ERR_ACCESS 반환.
 *         성공 시 taskENTER_CRITICAL 로 보호된 memcpy 로 스냅샷을 가져온다.
 */
uint8_t RD_MAP_DISPATCH_READ(uint16_t addr, uint16_t len, uint8_t *dst);

/**
 * @brief  PERIPHERAL.data + error 상태를 reg R/O 영역에 발행한다 (updateTask 호출).
 * @note   발행 항목: motor_data(position/velocity/current/temp/error_code/comm_err/state),
 *                    encoder.state, sys(hw_error/hw_fatal/sys_state/realtime_tick),
 *                    uart2.state, rc.state, diag(rx/tx/pkt_*_err_cnt).
 *         단위 변환·패킹은 CRITICAL 밖에서 로컬 변수에 계산 후 reg 쓰기만 CRITICAL 안.
 */
void    RD_MAP_MARSHAL_PUBLISH(const PERIPHERAL_t *p);

/**
 * @brief  reg.cmd_motor 스냅샷을 PERIPHERAL.cmd 에 적용한다 (controlTask 호출).
 * @note   스냅샷 전체를 CRITICAL 안에서 일괄 복사 후 CRITICAL 밖에서 필드를 분배.
 *         현재 ctr_mode[0] 을 4 모터 공통 대표값으로 사용 (Known Limitation).
 */
void    RD_MAP_MARSHAL_CONSUME(PERIPHERAL_t *p);

#endif /* INC_RD_MAP_ECU_H_ */
