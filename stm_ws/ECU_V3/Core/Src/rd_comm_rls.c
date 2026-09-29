/*
 * rd_comm_rls.c
 *
 *  RLS Orbis / AksIM-2 커맨드-응답 파싱 — 포맷/명령셋은 rd_comm_rls.h 참고.
 *  규칙은 rd_comm_imu.c 와 동일: 검증 실패는 comm_err_flag 통보 후 RET_WAIT,
 *  lifecycle 전이는 Checker 소유. WRITE/READ 는 last_cmd 로 짝을 맞춘다
 *  (요청 1개 → 응답 1개, 동시에 여러 명령을 걸지 않는 폴링 전제).
 *
 *  Created on: Sep 2, 2026
 *      Author: limtay
 */

/* Includes ------------------------------------------------------------------*/
#include "rd_comm_rls.h"
#include <string.h>

/* Private function prototypes -------------------------------------------------*/
static void   RLS_ParseAligned(uint32_t raw, uint8_t field_bits, RLS_comm_t *o);
static void   RLS_CmdShape(RLS_Model_e model, uint8_t cmd, uint8_t *hdr, uint8_t *extra);
static RD_RET RLS_ReadOrbis(RLS_comm_t *o, const uint8_t *b, uint16_t len, uint8_t cmd);
static RD_RET RLS_ReadAksim(RLS_comm_t *o, const uint8_t *b, uint16_t len, uint8_t cmd);

/* Private user code -----------------------------------------------------------*/

/**
 * @brief  좌측 정렬 위치 필드 공통 파싱. 두 기종 모두 최상위부터 위치 bits 개,
 *         최하위 b1=Error / b0=Warning(active low), 그 사이는 0 패딩이라
 *         필드폭(Orbis 16 / AksIM 24)만 다르고 수식은 같다.
 */
static void RLS_ParseAligned(uint32_t raw, uint8_t field_bits, RLS_comm_t *o)
{
    o->position = (raw >> (field_bits - o->bits)) & ((1UL << o->bits) - 1UL);
    o->error    = ((raw >> 1) & 1u) ? 0u : 1u;   /* active low 반전 */
    o->warning  = ((raw >> 0) & 1u) ? 0u : 1u;
}

/**
 * @brief  명령별 응답 형태 — 헤더 1B 유무 + 위치 뒤 부가 바이트 수.
 *         이걸로 기대 길이를 (헤더 + 위치 + 부가) 로 계산해 싱글턴/멀티턴을 판별한다.
 *         'v', 'i' 는 위치가 없어 호출부에서 별도 처리.
 */
static void RLS_CmdShape(RLS_Model_e model, uint8_t cmd, uint8_t *hdr, uint8_t *extra)
{
    *hdr = (cmd == (uint8_t)ORBIS_POS) ? 0u : 1u;   /* '3' 만 헤더를 회신하지 않음 */

    switch (cmd) {
    case (uint8_t)ORBIS_STATUS:                     /* 'd' 상세상태 */
        *extra = (model == RLS_ORBIS) ? 1u : 2u; break;
    case (uint8_t)ORBIS_TEMP:                       /* 't' 온도 */
        *extra = (model == RLS_ORBIS) ? 2u : 1u; break;
    case (uint8_t)AKSIM_STATUS_PER: *extra = 2u; break;  /* 'j' AksIM 전용 */
    case (uint8_t)AKSIM_SIGNAL:     *extra = 2u; break;  /* 'a' AksIM 전용 */
    case (uint8_t)AKSIM_SPEED:      *extra = 3u; break;  /* 's' AksIM 전용 */
    default:                        *extra = 0u; break;  /* '1', '3' */
    }
}

/* ── Orbis: 위치 필드 2B(14B) / 4B(14M), 16bit 좌측정렬 ──────────────────── */
static RD_RET RLS_ReadOrbis(RLS_comm_t *o, const uint8_t *b, uint16_t len, uint8_t cmd)
{
    if (cmd == (uint8_t)ORBIS_SERIAL) {              /* 'v' : 헤더 + 시리얼 6B 고정 */
        if (len != 7u || b[0] != cmd) return RET_NOK;
        memcpy(o->serial, &b[1], 6);
        o->serial[6] = '\0';
        return RET_OK;
    }

    uint8_t hdr, extra;
    RLS_CmdShape(RLS_ORBIS, cmd, &hdr, &extra);
    if (hdr && b[0] != cmd) return RET_NOK;

    uint8_t pos_len;
    if      (len == (uint16_t)(hdr + 2u + extra)) pos_len = 2u;   /* 14B 싱글턴 */
    else if (len == (uint16_t)(hdr + 4u + extra)) pos_len = 4u;   /* 14M 멀티턴 */
    else return RET_NOK;

    const uint8_t *p = &b[hdr];
    if (pos_len == 4u) {
        o->multiturn = (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
        p += 2;
    }
    RLS_ParseAligned(((uint32_t)p[0] << 8) | p[1], 16u, o);

    const uint8_t *x = &b[hdr + pos_len];            /* 위치 뒤 부가 데이터 */
    if (cmd == (uint8_t)ORBIS_STATUS) {
        o->status = x[0];
    } else if (cmd == (uint8_t)ORBIS_TEMP) {         /* 2B signed, °C×10 */
        o->temp_c = (float)(int16_t)(((uint16_t)x[0] << 8) | x[1]) / 10.0f;
    }
    return RET_OK;
}

/* ── AksIM-2: 위치 필드 3B(싱글턴) / 5B(멀티턴), 24bit 좌측정렬 ──────────── */
static RD_RET RLS_ReadAksim(RLS_comm_t *o, const uint8_t *b, uint16_t len, uint8_t cmd)
{
    if (cmd == (uint8_t)AKSIM_SERIAL) {              /* 'v' : 59B (데이터시트 B1 = b[0]) */
        if (len != 59u || b[0] != cmd) return RET_NOK;
        memcpy(o->serial,      &b[9],  8);           /* B10~B17 시리얼   */
        o->serial[8] = '\0';
        memcpy(o->part_number, &b[18], 16);          /* B19~B34 파트넘버 */
        o->part_number[16] = '\0';
        return RET_OK;
    }
    if (cmd == (uint8_t)AKSIM_SELFCAL) {             /* 'i' : 8B, 위치 필드 없음 */
        if (len != 8u || b[0] != cmd) return RET_NOK;
        o->cal_status    = b[1];
        o->ecc_um        = (uint16_t)(((uint16_t)b[2] << 8) | b[3]);
        o->ecc_phase_deg = (uint16_t)(((uint16_t)b[4] << 8) | b[5]);
        o->radial_um     = (int16_t)(((uint16_t)b[6] << 8) | b[7]);
        return RET_OK;
    }

    uint8_t hdr, extra;
    RLS_CmdShape(RLS_AKSIM, cmd, &hdr, &extra);
    if (hdr && b[0] != cmd) return RET_NOK;

    uint8_t pos_len;
    if      (len == (uint16_t)(hdr + 3u + extra)) pos_len = 3u;   /* 싱글턴 */
    else if (len == (uint16_t)(hdr + 5u + extra)) pos_len = 5u;   /* 멀티턴 */
    else return RET_NOK;

    const uint8_t *p = &b[hdr];
    if (pos_len == 5u) {
        o->multiturn = (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
        p += 2;
    }
    RLS_ParseAligned(((uint32_t)p[0] << 16) | ((uint32_t)p[1] << 8) | p[2], 24u, o);

    const uint8_t *x = &b[hdr + pos_len];            /* 위치 뒤 부가 데이터 */
    switch (cmd) {
    case (uint8_t)AKSIM_STATUS:
        o->status = (uint16_t)(((uint16_t)x[0] << 8) | x[1]);
        break;
    case (uint8_t)AKSIM_STATUS_PER:
        o->status_per = (uint16_t)(((uint16_t)x[0] << 8) | x[1]);
        break;
    case (uint8_t)AKSIM_SIGNAL:                      /* 2B unsigned */
        o->signal_level = (uint16_t)(((uint16_t)x[0] << 8) | x[1]);
        break;
    case (uint8_t)AKSIM_TEMP:
        o->temp_c = (float)(int8_t)x[0];             /* 1B signed, °C 정수 */
        break;
    case (uint8_t)AKSIM_SPEED: {                     /* 3B signed → 24→32bit 부호확장 */
        int32_t v = ((int32_t)x[0] << 16) | ((int32_t)x[1] << 8) | (int32_t)x[2];
        if (v & 0x00800000) { v |= (int32_t)0xFF000000; }
        o->speed_rpm = v;
        break;
    }
    default:
        break;                                       /* '1', '3' 은 부가 데이터 없음 */
    }
    return RET_OK;
}

/* Exported user code ----------------------------------------------------------*/

RD_RET RD_RLS_INIT(RLS_comm_t *rls_obj, RLS_Model_e model)
{
    if (rls_obj == NULL) return RET_NOK;

    memset(rls_obj, 0, sizeof(*rls_obj));
    rls_obj->model    = model;
    rls_obj->bits     = (model == RLS_ORBIS) ? 14u : 18u;  /* 17bit AksIM 이면 호출부에서 덮어쓸 것 */
    rls_obj->last_cmd = (model == RLS_ORBIS) ? (uint8_t)ORBIS_POS : (uint8_t)AKSIM_POS;
    rls_obj->ts_stamp = TS_INVALID;                        /* 첫 프레임 전 delta_tick = 0xFF 보장 */
    return RET_OK;
}

RD_RET RD_RLS_WRITE(UART_Ring_t *uart_obj, RLS_comm_t *rls_obj, uint8_t cmd)
{
    if (uart_obj == NULL || rls_obj == NULL) return RET_NOK;

    uart_obj->tx_buffer[0] = cmd;
    uart_obj->tx_length    = 1;

    RD_RET ret = RD_UART_TRANSMIT(uart_obj);
    if (ret == RET_OK) {
        /* 실제로 나간 명령만 기록 — RET_WAIT(이전 송신 중)이면 진행중인 응답은
         * 여전히 이전 last_cmd 소관이므로 덮어쓰지 않는다. */
        rls_obj->last_cmd = cmd;
    }
    return ret;
}

RD_RET RD_RLS_READ(UART_Ring_t *uart_obj, RLS_comm_t *rls_obj)
{
    if (uart_obj == NULL || rls_obj == NULL) return RET_NOK;
    if (uart_obj->rx_new == 0) return RET_WAIT;

    const uint8_t *pBuf = uart_obj->temp_buffer;
    uint16_t       len  = uart_obj->rx_length;
    uint8_t        cmd  = rls_obj->last_cmd;
    uart_obj->rx_new = 0;

    RD_RET ret = (rls_obj->model == RLS_ORBIS) ? RLS_ReadOrbis(rls_obj, pBuf, len, cmd)
                                               : RLS_ReadAksim(rls_obj, pBuf, len, cmd);
    if (ret != RET_OK) {
        uart_obj->comm_err_flag |= COMM_ERR_FRAMING_BIT;  /* CHECKER 에 framing 에러 통보 */
        return RET_WAIT;
    }

    /* 취득 시각 = IDLE(프레임 수신 완료) 시각 채택 (delta_tick 용) */
    rls_obj->ts_stamp = uart_obj->rx_stamp;
    return RET_OK;
}
