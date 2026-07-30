/**
 ******************************************************************************
 * @file    rd_comm_orin.c
 * @author  swarm
 * @date    2026-06-25
 * @brief   Orin RS485 (USART2) Dynamixel Protocol 2.0-like 패킷 구현부.
 *
 *  ECU_V3 의 rd_comm_ecu.c 를 DPC_B 에 맞게 이식.
 *  - 타입명: PACKET_* → ORIN_*
 *  - 디스패치: RD_MAP_DISPATCH_WRITE / READ (rd_map_dpcb.h)
 *
 *  호출 흐름 (rs485Task):
 *      RD_ORIN_READ → RD_ORIN_HANDLE → RD_ORIN_WRITE
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "rd_comm_orin.h"
#include "rd_map_dpcb.h"
#include <string.h>

/* ── CRC-16/IBM 룩업 테이블 ──────────────────────────────────────────────────
 *  다항식: x^16 + x^15 + x^2 + 1 (0x8005, LSB-first).
 *  계산 범위: Header[0] ~ CRC 필드 직전.
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

/* Private functions ---------------------------------------------------------*/

static uint16_t CalculateChecksum(const uint8_t *pBuf, uint16_t length)
{
    uint16_t crc = 0;
    for (uint16_t j = 0; j < length - 2; j++) {
        uint16_t i = ((crc >> 8) ^ pBuf[j]) & 0xFF;
        crc = (crc << 8) ^ s_crc_table[i];
    }
    return crc;
}

/* Exported functions --------------------------------------------------------*/

RD_RET RD_ORIN_INIT(ORIN_COMM_t *comm)
{
    if (comm == NULL) return RET_NOK;
    memset(comm, 0, sizeof(*comm));
    return RET_OK;
}

/**
 * @brief  RS485 수신 버퍼에서 패킷 1개를 파싱하여 comm->rx 에 저장.
 * @note   ID 불일치 (자기 ID 아닌 패킷) 는 에러 없이 묵시적 폐기 — 버스 공유 환경 정상 동작.
 */
RD_RET RD_ORIN_READ(RS485_t *rs485_obj, ORIN_COMM_t *comm)
{
    if (rs485_obj->uart_obj == NULL || comm == NULL) return RET_NOK;
    UART_Ring_t *uart_obj = rs485_obj->uart_obj;

    if (!uart_obj->rx_new) return RET_WAIT;

    uint8_t  *pBuf      = uart_obj->temp_buffer;
    uint16_t  packet_len = uart_obj->rx_length;

    /* [Step 1] Header */
    if (pBuf[0] != ORIN_HEADER1 || pBuf[1] != ORIN_HEADER2) {
        uart_obj->comm_err_flag |= COMM_ERR_FRAMING_BIT;
        uart_obj->rx_new = 0;
        return RET_WAIT;
    }

    /* [Step 1b] ID — 자기 ID 아닌 패킷은 에러 없이 폐기 */
    if (pBuf[ORIN_ID_IDX] != ORIN_MY_ID) {
        uart_obj->rx_new = 0;
        return RET_WAIT;
    }

    /* [Step 2] Length */
    uint16_t length_field = (uint16_t)(pBuf[3] | (pBuf[4] << 8));
    if (packet_len != length_field + ORIN_HEADER_SIZE) {
        uart_obj->comm_err_flag |= COMM_ERR_FRAMING_BIT;
        uart_obj->rx_new = 0;
        return RET_WAIT;
    }

    /* [Step 3] CRC */
    uint16_t received_crc = (uint16_t)(pBuf[packet_len - 2] | (pBuf[packet_len - 1] << 8));
    if (CalculateChecksum(pBuf, packet_len) != received_crc) {
        uart_obj->comm_err_flag |= COMM_ERR_CRC_BIT;
        uart_obj->rx_new = 0;
        return RET_WAIT;
    }

    /* [Step 4] 파싱 — data_len = Length - Inst(1) - CRC(2) */
    comm->rx.data_len = length_field - 3;
    memcpy(&comm->rx, &pBuf[ORIN_ID_IDX], 4 + comm->rx.data_len);

    uart_obj->rx_new = 0;
    return RET_OK;
}

/**
 * @brief  rx.Instruction 분기 → Dispatch → tx 응답 구성.
 * @note   WRITE: lock=1 이면 CMD_MOT 영역(128~142) 에 걸치는 쓰기를 ORIN_ERR_ACCESS 로 거부.
 *         CMD_DPCA(120~121), CMD_DPCB(122~127) 는 lock 무관 항상 허용.
 */
RD_RET RD_ORIN_HANDLE(ORIN_COMM_t *comm, uint8_t lock)
{
    if (comm == NULL) return RET_NOK;

    ORIN_PKT_t *rx  = &comm->rx;
    ORIN_PKT_t *tx  = &comm->tx;
    uint8_t     err = ORIN_ERR_NONE;

    switch (rx->Instruction) {
        case ORIN_INST_PING: {
            tx->Data[0] = ORIN_ERR_NONE;
            tx->data_len = 1;
            break;
        }
        case ORIN_INST_WRITE: {
            if (rx->data_len < 3) { err = ORIN_ERR_DATA_LEN; break; }
            uint16_t addr = (uint16_t)(rx->Data[0]) | ((uint16_t)rx->Data[1] << 8);
            uint16_t wlen = rx->data_len - 2;
            err = RD_MAP_DISPATCH_WRITE(addr, wlen, &rx->Data[2], lock);
            tx->Data[0] = err;
            tx->data_len = 1;
            break;
        }
        case ORIN_INST_READ: {
            if (rx->data_len != 4) {
                err = ORIN_ERR_DATA_LEN;
                tx->Data[0] = err;
                tx->data_len = 1;
                break;
            }
            uint16_t addr = (uint16_t)(rx->Data[0]) | ((uint16_t)rx->Data[1] << 8);
            uint16_t rlen = (uint16_t)(rx->Data[2]) | ((uint16_t)rx->Data[3] << 8);
            if (rlen > (ORIN_DATA_BUF_SIZE - 1)) {
                err = ORIN_ERR_DATA_LEN;
                tx->Data[0] = err;
                tx->data_len = 1;
                break;
            }
            /* Data[0] = err 바이트, Data[1..] = reg 데이터 */
            err = RD_MAP_DISPATCH_READ(addr, rlen, &tx->Data[1]);
            tx->Data[0] = err;
            tx->data_len = (err == ORIN_ERR_NONE) ? (1 + rlen) : 1;
            break;
        }
        case ORIN_INST_REBOOT: {
            /* 응답 먼저 빌드 — 실제 리셋은 rs485Task 가 WRITE 완료 후 수행 */
            tx->Data[0] = ORIN_ERR_NONE;
            tx->data_len = 1;
            comm->reboot_pending = 1;
            break;
        }
        default:
            err = ORIN_ERR_INST;
            tx->Data[0] = err;
            tx->data_len = 1;
            break;
    }

    tx->TargetID    = ORIN_MASTER_ID;
    tx->Instruction = rx->Instruction;
    return RET_OK;
}

/**
 * @brief  comm->tx 를 직렬화하여 RS485 로 송신.
 */
RD_RET RD_ORIN_WRITE(RS485_t *rs485_obj, ORIN_COMM_t *comm)
{
    if (rs485_obj->uart_obj == NULL || comm == NULL) return RET_NOK;

    uint8_t  *pBuf    = rs485_obj->uart_obj->tx_buffer;
    uint16_t  data_len = comm->tx.data_len;

    pBuf[0] = ORIN_HEADER1;
    pBuf[1] = ORIN_HEADER2;

    comm->tx.Length = data_len + 3;   /* Length = Inst(1) + Param(N) + CRC(2) */

    /* ID(1)+Length(2)+Instruction(1)+Parameter(N) = 4+data_len 복사 */
    memcpy(&pBuf[ORIN_ID_IDX], &comm->tx, 4 + data_len);

    uint16_t packet_len = ORIN_HEADER_SIZE + data_len + 3;
    uint16_t checksum   = CalculateChecksum(pBuf, packet_len);
    pBuf[packet_len - 2] = checksum & 0xFF;
    pBuf[packet_len - 1] = (checksum >> 8) & 0xFF;

    rs485_obj->uart_obj->tx_length = packet_len;

    return RD_RS485_TRANSMIT(rs485_obj);
}
