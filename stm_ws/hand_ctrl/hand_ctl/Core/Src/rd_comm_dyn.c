/*
 * rd_comm_dyn.c
 *  Dynamixel Protocol 2.0 – 저수준 패킷 빌더 / 파서
 *
 *  Created on: Mar 10, 2026
 *      Author: swarm
 */

/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "rd_comm_dyn.h"
#include <string.h>

/* Private defines -----------------------------------------------------------*/

/* ── CRC-16 테이블 (Dynamixel Protocol 2.0 공식 테이블) ─────────────────────
 *  CRC 계산 범위: 패킷 첫 바이트(0xFF)부터 CRC 필드 직전까지
 *  Header(FF FF FD 00) 포함하여 전체 패킷에 대해 계산합니다.
 * ──────────────────────────────────────────────────────────────────────────*/
static const uint16_t s_crc_table[256] = {
    0x0000, 0x8005, 0x800F, 0x000A, 0x801B, 0x001E, 0x0014, 0x8011,
    0x8033, 0x0036, 0x003C, 0x8039, 0x0028, 0x802D, 0x8027, 0x0022,
    0x8063, 0x0066, 0x006C, 0x8069, 0x0078, 0x807D, 0x8077, 0x0072,
    0x0050, 0x8055, 0x805F, 0x005A, 0x804B, 0x004E, 0x0044, 0x8041,
    0x80C3, 0x00C6, 0x00CC, 0x80C9, 0x00D8, 0x80DD, 0x80D7, 0x00D2,
    0x00F0, 0x80F5, 0x80FF, 0x00FA, 0x80EB, 0x00EE, 0x00E4, 0x80E1,
    0x00A0, 0x80A5, 0x80AF, 0x00AA, 0x80BB, 0x00BE, 0x00B4, 0x80B1,
    0x8093, 0x0096, 0x009C, 0x8099, 0x0088, 0x808D, 0x8087, 0x0082,
    0x8183, 0x0186, 0x018C, 0x8189, 0x0198, 0x819D, 0x8197, 0x0192,
    0x01B0, 0x81B5, 0x81BF, 0x01BA, 0x81AB, 0x01AE, 0x01A4, 0x81A1,
    0x01E0, 0x81E5, 0x81EF, 0x01EA, 0x81FB, 0x01FE, 0x01F4, 0x81F1,
    0x81D3, 0x01D6, 0x01DC, 0x81D9, 0x01C8, 0x81CD, 0x81C7, 0x01C2,
    0x0140, 0x8145, 0x814F, 0x014A, 0x815B, 0x015E, 0x0154, 0x8151,
    0x8173, 0x0176, 0x017C, 0x8179, 0x0168, 0x816D, 0x8167, 0x0162,
    0x8123, 0x0126, 0x012C, 0x8129, 0x0138, 0x813D, 0x8137, 0x0132,
    0x0110, 0x8115, 0x811F, 0x011A, 0x810B, 0x010E, 0x0104, 0x8101,
    0x8303, 0x0306, 0x030C, 0x8309, 0x0318, 0x831D, 0x8317, 0x0312,
    0x0330, 0x8335, 0x833F, 0x033A, 0x832B, 0x032E, 0x0324, 0x8321,
    0x0360, 0x8365, 0x836F, 0x036A, 0x837B, 0x037E, 0x0374, 0x8371,
    0x8353, 0x0356, 0x035C, 0x8359, 0x0348, 0x834D, 0x8347, 0x0342,
    0x03C0, 0x83C5, 0x83CF, 0x03CA, 0x83DB, 0x03DE, 0x03D4, 0x83D1,
    0x83F3, 0x03F6, 0x03FC, 0x83F9, 0x03E8, 0x83ED, 0x83E7, 0x03E2,
    0x83A3, 0x03A6, 0x03AC, 0x83A9, 0x03B8, 0x83BD, 0x83B7, 0x03B2,
    0x0390, 0x8395, 0x839F, 0x039A, 0x838B, 0x038E, 0x0384, 0x8381,
    0x0280, 0x8285, 0x828F, 0x028A, 0x829B, 0x029E, 0x0294, 0x8291,
    0x82B3, 0x02B6, 0x02BC, 0x82B9, 0x02A8, 0x82AD, 0x82A7, 0x02A2,
    0x82E3, 0x02E6, 0x02EC, 0x82E9, 0x02F8, 0x82FD, 0x82F7, 0x02F2,
    0x02D0, 0x82D5, 0x82DF, 0x02DA, 0x82CB, 0x02CE, 0x02C4, 0x82C1,
    0x8243, 0x0246, 0x024C, 0x8249, 0x0258, 0x825D, 0x8257, 0x0252,
    0x0270, 0x8275, 0x827F, 0x027A, 0x826B, 0x026E, 0x0264, 0x8261,
    0x0220, 0x8225, 0x822F, 0x022A, 0x823B, 0x023E, 0x0234, 0x8231,
    0x8213, 0x0216, 0x021C, 0x8219, 0x0208, 0x820D, 0x8207, 0x0202
};

/* Private user code ---------------------------------------------------------*/

/**
 * @brief CRC-16 계산 (패킷 첫 바이트부터 CRC 필드 직전까지)
 * @param pBuf   : 패킷 버퍼 포인터
 * @param length : 전체 패킷 크기 (CRC 2바이트 포함)
 */
static uint16_t CalculateChecksum(const uint8_t *pBuf, uint16_t length)
{
    uint16_t crc = 0;
    for (uint16_t j = 0; j < length - 2; j++) {
        uint16_t i = ((crc >> 8) ^ pBuf[j]) & 0xFF;
        crc = (crc << 8) ^ s_crc_table[i];
    }
    return crc;
}

/**
 * @brief Byte Stuffing 적용 (TX 전용)
 *        검사 범위 내 "FF FF FD" 패턴 → "FF FF FD FD" 로 변환
 * @param pBuf    : 패킷 버퍼
 * @param start   : 검사 시작 인덱스 (Instruction 필드)
 * @param count   : 검사 바이트 수 (Instruction + Parameter, CRC 제외)
 * @param buf_max : 버퍼 최대 크기 (오버플로우 방지)
 * @return        : 삽입된 stuffing 바이트 수
 */
static uint16_t ApplyByteStuffing(uint8_t *pBuf, uint16_t start, uint16_t count, uint16_t buf_max)
{
    uint16_t stuffed = 0;
    uint16_t i   = start;
    uint16_t end = start + count; // 슬라이딩 끝 인덱스 (삽입 시 증가)

    while (i + 2 < end) {
        if (pBuf[i] == 0xFF && pBuf[i+1] == 0xFF && pBuf[i+2] == 0xFD) {
            if (end + 1 > buf_max) break; // 버퍼 초과 방지
            memmove(&pBuf[i+4], &pBuf[i+3], end - (i + 3));
            pBuf[i+3] = 0xFD;
            end++;
            stuffed++;
            i += 4; // 삽입된 0xFD 이후부터 재검사
        } else {
            i++;
        }
    }
    return stuffed;
}

/**
 * @brief Byte Stuffing 제거 (RX 전용)
 *        검사 범위 내 "FF FF FD FD" 패턴 → "FF FF FD" 로 복원
 * @param pBuf  : 패킷 버퍼
 * @param start : 검사 시작 인덱스 (Instruction 필드)
 * @param count : 검사 바이트 수 (Stuffing 포함 Instruction + Parameter)
 * @return      : 제거된 stuffing 바이트 수
 */
static uint16_t RemoveByteStuffing(uint8_t *pBuf, uint16_t start, uint16_t count)
{
    uint16_t removed = 0;
    uint16_t i   = start;
    uint16_t end = start + count; // 슬라이딩 끝 인덱스 (제거 시 감소)

    while (i + 3 < end) {
        if (pBuf[i] == 0xFF && pBuf[i+1] == 0xFF && pBuf[i+2] == 0xFD && pBuf[i+3] == 0xFD) {
            memmove(&pBuf[i+3], &pBuf[i+4], end - (i + 4));
            end--;
            removed++;
            i += 3; // "FF FF FD" 직후부터 재검사
        } else {
            i++;
        }
    }
    return removed;
}

/* Exported user code --------------------------------------------------------*/
RD_RET RD_DYNPACK_READ(RS485_t *rs485_obj, DYN_comm_t *packet_obj)
{
    if (rs485_obj->uart_obj == NULL || packet_obj == NULL) return RET_NOK;
    UART_Ring_t *uart_obj = rs485_obj->uart_obj;

    if (!uart_obj->rx_new) {
        return RET_WAIT;
    }

    uint8_t  *pBuf      = uart_obj->temp_buffer;
    uint16_t  packet_len = uart_obj->rx_length;

    /* --- [Step 1] Header check --- */
    if (pBuf[0] != DYN_HEADER1 || pBuf[1] != DYN_HEADER2 ||
        pBuf[2] != DYN_HEADER3 || pBuf[3] != DYN_RESERVED)
    {
        uart_obj->rx_new = 0;
        return RET_WAIT;
    }

    /* --- [Step 2] Length check --- */
    uint16_t length_field = (uint16_t)(pBuf[5] | (pBuf[6] << 8));
    if (packet_len != length_field + DYN_HEADER_SIZE)
    {
        uart_obj->rx_new = 0;
        return RET_WAIT;
    }

    /* --- [Step 3] CRC check (Byte Stuffing 포함 상태에서 계산) --- */
    uint16_t received_crc = (uint16_t)(pBuf[packet_len - 2] | (pBuf[packet_len - 1] << 8));
    if (CalculateChecksum(pBuf, packet_len) != received_crc)
    {
        uart_obj->rx_new = 0;
        return RET_WAIT;
    }

    /* --- [Step 4] Byte Stuffing 제거 --- */
    // 검사 범위: pBuf[DYN_HEADER_SIZE] (Instruction) ~ Parameter 끝 (CRC 제외)
    // stuffed_payload_len = Inst(1) + Param + stuffing바이트 = length_field - CRC(2)
    uint16_t stuffed_payload_len = length_field - 2;
    uint16_t removed = RemoveByteStuffing(pBuf, DYN_HEADER_SIZE, stuffed_payload_len);

    /* --- [Step 5] DATA copy (Parsing) --- */
    // data_len = 실제 Parameter 바이트 수 (Instruction 제외)
    packet_obj->rx.data_len = stuffed_payload_len - removed - 1;
    // Length 필드를 destuffed 값으로 갱신 후 구조체에 반영
    uint16_t new_length = length_field - removed;
    pBuf[5] = new_length & 0xFF;
    pBuf[6] = (new_length >> 8) & 0xFF;
    // 복사 크기: ID(1) + Length(2) + Instruction(1) + Parameters = 4 + data_len
    memcpy(&packet_obj->rx, &pBuf[DYN_ID_IDX], 4 + packet_obj->rx.data_len);

    uart_obj->is_running = 1;
    uart_obj->rx_new = 0;
    return RET_OK;
}

RD_RET RD_DYNPACK_WRITE(RS485_t *rs485_obj, DYN_comm_t *packet_obj)
{
    if (rs485_obj->uart_obj == NULL || packet_obj == NULL) return RET_NOK;

    uint8_t  *pBuf    = rs485_obj->uart_obj->tx_buffer;
    uint16_t  data_len = packet_obj->tx.data_len; // Parameter 바이트 수

    /* --- [Step 1] 기본 패킷 구성 (Byte Stuffing 적용 전) --- */
    pBuf[0] = DYN_HEADER1;
    pBuf[1] = DYN_HEADER2;
    pBuf[2] = DYN_HEADER3;
    pBuf[3] = DYN_RESERVED;

    // LENGTH = Inst(1) + Param + CRC(2)
    packet_obj->tx.Length = data_len + 3;

    // 복사 크기: ID(1) + Length(2) + Instruction(1) + Parameters = 4 + data_len
    memcpy(&pBuf[DYN_ID_IDX], &packet_obj->tx, 4 + data_len);

    /* --- [Step 2] Byte Stuffing 적용 --- */
    // 검사 범위: pBuf[DYN_HEADER_SIZE] (Instruction) ~ Parameter 끝, CRC 슬롯 제외
    // count = Inst(1) + Param
    uint16_t stuffed = ApplyByteStuffing(pBuf, DYN_HEADER_SIZE, data_len + 1, TX_BUFFER_SIZE);

    /* --- [Step 3] Length 필드 업데이트 (stuffed 바이트 반영) --- */
    uint16_t new_length = data_len + 3 + stuffed; // Inst(1) + Param + stuffed + CRC(2)
    pBuf[5] = new_length & 0xFF;
    pBuf[6] = (new_length >> 8) & 0xFF;
    uint16_t packet_len = DYN_HEADER_SIZE + new_length;

    /* --- [Step 4] CRC 계산 및 삽입 (Byte Stuffing 적용된 상태에서) --- */
    uint16_t checksum = CalculateChecksum(pBuf, packet_len);
    pBuf[packet_len - 2] = checksum & 0xFF;
    pBuf[packet_len - 1] = (checksum >> 8) & 0xFF;

    rs485_obj->uart_obj->tx_length = packet_len;

    return RD_RS485_Transmit(rs485_obj);
}
