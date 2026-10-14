/**
 ******************************************************************************
 * @file    rd_map_pra.h
 * @author  swarm
 * @date    2026-09-17
 * @brief   PRA 레지스터 맵 디스패치 + 마샬 레이어 — 공개 인터페이스.
 *
 *  DPC_B rd_map_dpcb.h 를 PRA 에 이식.
 *
 *  1) Dispatch (외부 RS485 마스터(Orin) ↔ reg):
 *       RD_ORIN_HANDLE → RD_MAP_DISPATCH_READ / WRITE → 256B reg 직접 접근.
 *       영역 경계·접근 권한·UNLOCK 조건을 s_regions[] LUT 로 검증.
 *
 *  2) Marshal (reg ↔ PERIPHERAL):
 *       RD_MAP_MARSHAL_PUBLISH : 내부 상태 → reg R/O 영역 발행 (**Step 7 본체**)
 *       RD_MAP_MARSHAL_CONSUME : reg.cmd_* → 내부 적용            (**Step 7 본체**)
 *       현재는 void 스텁 — PRA PERIPHERAL_t 미정의(Step 8/10) 이므로 인자 없이 둔다.
 *
 *  단일 source-of-truth: `REGISTER_t reg` (rd_map_pra.c 정의).
 ******************************************************************************
 */

#ifndef INC_RD_MAP_PRA_H_
#define INC_RD_MAP_PRA_H_

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"
#include "rd_common.h"
#include "rd_define.h"
#include "rd_register_pra.h"
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
    ORIN_ERR_ACCESS      = 0x07,   /* R/O 영역 쓰기 / 미정의 주소 / UNLOCK 미설정 / LOCK */
} ORIN_Error_e;

/* ── 전역 레지스터 맵 (256 byte, 바이트 주소 0~255 로 외부 접근) ─────────────── */
extern REGISTER_t reg;

/* Exported functions --------------------------------------------------------*/

/** @brief reg 전체 0 초기화 후 DEFINE 기본값 설정. */
RD_RET  RD_MAP_INIT(void);

/**
 * @brief  주소 addr 부터 len 바이트를 src 에서 reg 로 쓴다.
 * @retval ORIN_Error_e (0x00 = 성공). 영역 횡단·R/O·UNLOCK 미설정 → ORIN_ERR_ACCESS,
 *         addr+len > 256 → ORIN_ERR_DATA_LEN.
 * @note   lock 파라미터는 잠금 영역 write 게이팅용 (PRA 현재 잠금 영역 없음 → 무영향, 모드 기반 권한은 TODO).
 */
uint8_t RD_MAP_DISPATCH_WRITE(uint16_t addr, uint16_t len, const uint8_t *src, uint8_t lock);

/** @brief 주소 addr 부터 len 바이트를 reg 에서 dst 로 읽는다. @retval ORIN_Error_e */
uint8_t RD_MAP_DISPATCH_READ(uint16_t addr, uint16_t len, uint8_t *dst);

/** @brief 내부 상태 → reg R/O 영역 발행 (Step 7 본체 — 현재 스텁). */
void    RD_MAP_MARSHAL_PUBLISH(void);

/** @brief reg.cmd_* → 내부 적용 (Step 7 본체 — 현재 스텁). */
void    RD_MAP_MARSHAL_CONSUME(void);

#endif /* INC_RD_MAP_PRA_H_ */
