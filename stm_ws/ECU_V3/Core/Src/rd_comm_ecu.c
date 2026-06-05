/**
 ******************************************************************************
 * @file    rd_comm_ecu.c
 * @author  swarm
 * @date    2026-03-10
 * @brief   Dynamixel Protocol 2.0 모방 패킷 빌더 / 파서 구현부.
 *
 *  와이어 포맷 (6개 필드, 가변 길이):
 *      [0:1]  Header  0xAA 0x55
 *      [2]    ID      요청: PACKET_MY_ID(0xE1) / 응답: PACKET_MASTER_ID(0x01)
 *      [3:4]  Length  L|H = Instruction(1) + Parameter(N) + CRC(2) = N + 3
 *      [5]    Instruction  READ=0x02 / WRITE=0x03 / ...  (응답: 원 요청 복사)
 *      [6..N+5] Parameter  가변 (Instruction 별 포맷 상이)
 *      [N+6:N+7] CRC  CRC-16/IBM, little-endian
 *
 *  호출 흐름 (rs485Task):
 *      RD_PACKET_READ → RD_PACKET_HANDLE → RD_PACKET_WRITE
 *
 *  에러 처리 정책:
 *      comm_err_flag 는 RD_UART_CHECKER 가 읽어 STATE_t health 로 승격.
 *      lifecycle 전이는 이 파일에서 절대 하지 않음 — Checker 가 단독 소유.
 ******************************************************************************
 */

/* Includes ------------------------------------------------------------------*/
#include "rd_comm_ecu.h"
#include "rd_map_ecu.h"
#include <string.h>

/* ── CRC-16/IBM 룩업 테이블 ──────────────────────────────────────────────────
 *  다항식: x^16 + x^15 + x^2 + 1 (0x8005, LSB-first).
 *  계산 범위: 패킷 첫 바이트(Header[0]) ~ CRC 필드 직전까지.
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

/**
 * @brief  CRC-16/IBM 계산.
 * @param  pBuf   패킷 버퍼 시작 포인터 (Header[0] 부터)
 * @param  length 전체 패킷 바이트 수 (CRC 2 byte 포함)
 * @note   length - 2 로 CRC 필드 자체를 계산에서 제외.
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

/* Exported functions --------------------------------------------------------*/

/**
 * @brief  PACKET_comm_t 구조체 초기화 (tx/rx 버퍼 + 에러 카운터 클리어).
 */
RD_RET RD_PACKET_INIT(PACKET_comm_t *packet_obj)
{
    if (packet_obj == NULL) return RET_NOK;
    memset(packet_obj, 0, sizeof(*packet_obj));
    return RET_OK;
}

/**
 * @brief  RS485 수신 버퍼에서 패킷 1개를 파싱하여 packet_obj->rx 에 저장.
 * @retval RET_OK   파싱 성공 — Handle 호출 가능 상태
 * @retval RET_WAIT 수신 데이터 없음 또는 유효하지 않은 패킷 (재시도 불필요)
 * @retval RET_NOK  인자 오류
 *
 * @note   ID 불일치 시 에러 처리 없이 묵시적 폐기.
 *           RS485 버스에는 다른 노드의 트래픽도 공존하므로 정상 동작.
 */
RD_RET RD_PACKET_READ(RS485_t *rs485_obj, PACKET_comm_t *packet_obj)
{
    if (rs485_obj->uart_obj == NULL || packet_obj == NULL) return RET_NOK;
    UART_Ring_t *uart_obj = rs485_obj->uart_obj;

    if (!uart_obj->rx_new) return RET_WAIT;

    uint8_t  *pBuf      = uart_obj->temp_buffer;
    uint16_t  packet_len = uart_obj->rx_length;

    /* [Step 1] Header */
    if (pBuf[0] != PACKET_HEADER1 || pBuf[1] != PACKET_HEADER2)
    {
        uart_obj->comm_err_flag |= COMM_ERR_FRAMING_BIT;  /* M-2: CHECKER 에 framing 에러 통보 */
    	uart_obj->rx_new = 0;
    	return RET_WAIT;
    }

    /* [Step 1b] ID — 자기 ID 아닌 패킷은 에러 없이 폐기 (버스 공유 환경) */
    if (pBuf[PACKET_ID_IDX] != PACKET_MY_ID)
    {
        uart_obj->rx_new = 0;
        return RET_WAIT;
    }

    /* [Step 2] Length — 와이어 기준 pBuf[3:4] (Header 2B + ID 1B 다음) */
    uint16_t length_field = (uint16_t)(pBuf[3] | (pBuf[4] << 8));
    if (packet_len != length_field + PACKET_HEADER_SIZE)
    {
        uart_obj->comm_err_flag |= COMM_ERR_FRAMING_BIT;  /* M-2: CHECKER 에 framing 에러 통보 */
    	uart_obj->rx_new = 0;
    	return RET_WAIT;
    }

    /* [Step 3] CRC */
    uint16_t received_crc = (uint16_t)(pBuf[packet_len - 2] | (pBuf[packet_len - 1] << 8));
    if (CalculateChecksum(pBuf, packet_len) != received_crc)
    {
        uart_obj->comm_err_flag |= COMM_ERR_CRC_BIT;
        uart_obj->rx_new = 0;
        return RET_WAIT;
    }

    /* [Step 4] 파싱 — ID(1)+Length(2)+Instruction(1)+Parameter 를 rx 구조체로 복사.
     *  data_len = Parameter 바이트 수 = Length - Inst(1) - CRC(2) */
    packet_obj->rx.data_len = length_field - 3;
    memcpy(&packet_obj->rx, &pBuf[PACKET_ID_IDX], 4 + packet_obj->rx.data_len);

    /* lifecycle 는 Checker 가 소유 — rx_new 클리어만 수행 */
    uart_obj->rx_new = 0;
    return RET_OK;
}

/**
 * @brief  수신 Instruction 을 분기하여 레지스터 맵 Dispatch 후 tx 응답을 구성.
 * @note   응답 포맷: TargetID = PACKET_MASTER_ID(0x01), Instruction = rx 복사.
 *         Parameter[0] = PACKET_Error_e (8종), READ 성공 시 [1..] 에 reg 데이터.
 * @note   WRITE 최소 data_len = 3: Addr(2B) + Data(1B 이상).
 *         READ  최대 rlen = PACKET_DATA_BUF_SIZE - 1: Data[0] 가 err 바이트로 예약됨.
 */
RD_RET RD_PACKET_HANDLE(PACKET_comm_t *packet_obj, uint8_t lock)
{
    if (packet_obj == NULL) return RET_NOK;

    PACKET_PACKET_t *rx = &packet_obj->rx;
    PACKET_PACKET_t *tx = &packet_obj->tx;
    uint8_t err = PACKET_ERR_NONE;

    switch (rx->Instruction) {
        case PACKET_INST_PING: {
            /* Parameter 없음. Err(1) = PACKET_ERR_NONE 만 응답. */
            tx->Data[0] = PACKET_ERR_NONE;
            tx->data_len = 1;
            break;
        }
        case PACKET_INST_WRITE: {
            /* Parameter: [AddrLo, AddrHi, Data...], 최소 3 byte */
            if (rx->data_len < 3) { err = PACKET_ERR_DATA_LEN; break; }
            uint16_t addr = (uint16_t)(rx->Data[0]) | ((uint16_t)rx->Data[1] << 8);
            uint16_t wlen = rx->data_len - 2;   /* Data 부분 길이 (Addr 2B 제외) */
            err = RD_MAP_DISPATCH_WRITE(addr, wlen, &rx->Data[2], lock);
            tx->Data[0] = err;
            tx->data_len = 1;
            break;
        }
        case PACKET_INST_READ: {
            /* Parameter: [AddrLo, AddrHi, LenLo, LenHi], 항상 4 byte */
            if (rx->data_len != 4) { err = PACKET_ERR_DATA_LEN; tx->Data[0] = err; tx->data_len = 1; break; }
            uint16_t addr = (uint16_t)(rx->Data[0]) | ((uint16_t)rx->Data[1] << 8);
            uint16_t rlen = (uint16_t)(rx->Data[2]) | ((uint16_t)rx->Data[3] << 8);
            /* Data[0] 이 err 바이트로 예약되므로 실제 데이터는 Data[1] 부터 */
            if (rlen > (PACKET_DATA_BUF_SIZE - 1)) {
                err = PACKET_ERR_DATA_LEN;
                tx->Data[0] = err;
                tx->data_len = 1;
                break;
            }
            err = RD_MAP_DISPATCH_READ(addr, rlen, &tx->Data[1]);
            tx->Data[0] = err;
            tx->data_len = (err == PACKET_ERR_NONE) ? (1 + rlen) : 1;
            break;
        }
        case PACKET_INST_REBOOT: {
            /* 응답 먼저 빌드 — 실제 리셋은 rs485Task 가 WRITE 완료 후 수행. */
            tx->Data[0] = PACKET_ERR_NONE;
            tx->data_len = 1;
            packet_obj->reboot_pending = 1;
            break;
        }
        default:
            err = PACKET_ERR_INST;
            tx->Data[0] = err;
            tx->data_len = 1;
            break;
    }

    tx->TargetID    = PACKET_MASTER_ID;    /* 응답 수신자 = 마스터(Orin) */
    tx->Instruction = rx->Instruction;     /* 어떤 요청에 대한 응답인지 식별용 */
    return RET_OK;
}

/**
 * @brief  packet_obj->tx 를 직렬화하여 RS485 로 송신.
 * @note   packet_len = PACKET_HEADER_SIZE(5) + data_len + 3
 *         여기서 +3 은 Length 필드가 Instruction(1)+Parameter(N)+CRC(2) 를 포함하기 때문.
 *         CRC 는 pBuf 의 마지막 2 byte 에 little-endian 으로 삽입.
 */
RD_RET RD_PACKET_WRITE(RS485_t *rs485_obj, PACKET_comm_t *packet_obj)
{
    if (rs485_obj->uart_obj == NULL || packet_obj == NULL) return RET_NOK;

    uint8_t  *pBuf    = rs485_obj->uart_obj->tx_buffer;
    uint16_t  data_len = packet_obj->tx.data_len;

    pBuf[0] = PACKET_HEADER1;
    pBuf[1] = PACKET_HEADER2;

    packet_obj->tx.Length = data_len + 3;  /* Length = Inst(1) + Param(N) + CRC(2) */

    /* ID(1)+Length(2)+Instruction(1)+Parameter(N) = 4+data_len 바이트 복사 */
    memcpy(&pBuf[PACKET_ID_IDX], &packet_obj->tx, 4 + data_len);

    uint16_t packet_len = PACKET_HEADER_SIZE + data_len + 3;
    uint16_t checksum   = CalculateChecksum(pBuf, packet_len);
    pBuf[packet_len - 2] = checksum & 0xFF;
    pBuf[packet_len - 1] = (checksum >> 8) & 0xFF;

    rs485_obj->uart_obj->tx_length = packet_len;

    return RD_RS485_TRANSMIT(rs485_obj);
}
