/**
 ******************************************************************************
 * @file    rd_map_dpcb.h
 * @author  swarm
 * @date    2026-06-25
 * @brief   DPC_B 레지스터 맵 디스패치 + 마샬 레이어 — 공개 인터페이스.
 *
 *  이 레이어는 두 가지 책임을 가진다:
 *
 *  1) Dispatch (외부 RS485 마스터 ↔ reg):
 *       RD_ORIN_HANDLE 이 READ/WRITE Instruction 을 받으면
 *       RD_MAP_DISPATCH_READ / WRITE 를 통해 256B reg 에 직접 접근.
 *       영역 경계·접근 권한·UNLOCK 조건을 s_regions[] LUT 로 검증.
 *
 *  2) Marshal (reg ↔ PERIPHERAL_t):
 *       RD_MAP_MARSHAL_PUBLISH  : PERIPHERAL 상태 → reg R/O 영역 발행 (Step 6)
 *       RD_MAP_MARSHAL_CONSUME  : reg.cmd_* → PERIPHERAL 적용 (Step 6)
 *       두 함수는 Step 5 에서 stub 만 생성, Step 6 에서 내용 채움.
 *
 *  단일 source-of-truth: `REGISTER_t reg` (rd_map_dpcb.c 에 정의된 전역 변수).
 ******************************************************************************
 */

#ifndef INC_RD_MAP_DPCB_H_
#define INC_RD_MAP_DPCB_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
#include "rd_common.h"
#include "rd_define.h"
#include "rd_register_dpcb.h"
#include "rd_peripheral_dpcb.h"
#include "rd_comm_orin.h"

/* ── 응답 에러 코드 ──────────────────────────────────────────────────────────
 *  RD_ORIN_HANDLE 이 Dispatch 반환값을 그대로 tx->Data[0] 에 실어 마스터로 전달.
 * ──────────────────────────────────────────────────────────────────────────*/
typedef enum {
    ORIN_ERR_NONE        = 0x00,   /* 정상 처리 */
    ORIN_ERR_RESULT_FAIL = 0x01,   /* Instruction 처리 실패 */
    ORIN_ERR_INST        = 0x02,   /* 정의되지 않은 Instruction */
    ORIN_ERR_CRC         = 0x03,   /* CRC 불일치 */
    ORIN_ERR_DATA_RANGE  = 0x04,   /* 데이터 값이 min/max 범위 초과 */
    ORIN_ERR_DATA_LEN    = 0x05,   /* 데이터 길이 부족 또는 주소 범위 초과 */
    ORIN_ERR_DATA_LIMIT  = 0x06,   /* 데이터 값이 Limit 초과 */
    ORIN_ERR_ACCESS      = 0x07,   /* R/O 영역 쓰기 / 미정의 주소 / UNLOCK 미설정 / 모터 LOCK */
} ORIN_Error_e;

/* ── 전역 레지스터 맵 ────────────────────────────────────────────────────────
 *  256 byte packed 구조체. 외부 마스터는 바이트 주소(0~255)로 접근.
 *  내부 코드는 필드명으로 접근한다 (struct overlay).
 * ──────────────────────────────────────────────────────────────────────────*/
extern REGISTER_t reg;

/* Exported functions --------------------------------------------------------*/

/** @brief reg 전체 0 초기화 후 DEFINE 기본값 + CMD_MOT 기본 전류 설정. */
RD_RET  RD_MAP_INIT(void);

/**
 * @brief  주소 addr 부터 len 바이트를 src 에서 reg 로 쓴다.
 * @retval ORIN_Error_e (0x00 = 성공)
 * @note   영역 횡단·R/O·UNLOCK 미설정 → ORIN_ERR_ACCESS.
 *         lock=1 이고 CMD_MOT 영역(128~142) 에 걸치는 경우 → ORIN_ERR_ACCESS.
 *         addr+len > 256 → ORIN_ERR_DATA_LEN.
 */
uint8_t RD_MAP_DISPATCH_WRITE(uint16_t addr, uint16_t len, const uint8_t *src, uint8_t lock);

/**
 * @brief  주소 addr 부터 len 바이트를 reg 에서 dst 로 읽는다.
 * @retval ORIN_Error_e (0x00 = 성공)
 */
uint8_t RD_MAP_DISPATCH_READ(uint16_t addr, uint16_t len, uint8_t *dst);

/**
 * @brief  PERIPHERAL 상태를 reg R/O 영역에 발행한다 (Step 6 구현 예정).
 * @note   발행 항목: motor_data, sensor_dpca, sensor_dpcb, uart2/4/6 state,
 *                    sys (hw_error/hw_fatal/sys_state/realtime_tick/degraded_cnt).
 */
void    RD_MAP_MARSHAL_PUBLISH(const PERIPHERAL_t *p);

/**
 * @brief  reg.cmd_* 를 PERIPHERAL 에 적용한다 (Step 7 구현 예정).
 * @note   적용 항목: cmd_dpca, cmd_dpcb (mode/sys_state_target 포함), cmd_mot.
 */
void    RD_MAP_MARSHAL_CONSUME(PERIPHERAL_t *p);

#endif /* INC_RD_MAP_DPCB_H_ */
