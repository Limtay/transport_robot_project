#include "orin_firmware_bridge/rd_comm.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <cstring>

namespace orin_bridge {

RdComm::RdComm(RdUart* ptr) : uart_(ptr) {
    if (uart_ == nullptr) {
        throw std::runtime_error("[COMM Fatal] RdComm initialized with null RdUart pointer!");
    }
}

RD_RET RdComm::Init(PACKET_comm_t* packet_obj) {
    if (!packet_obj || !uart_) {
        std::cerr << "[COMM Fatal] RdComm::Init() - Invalid UART or PACKET pointer!" << std::endl;
        return RD_FATAL;
    }

    RD_RET ret = uart_->Init();
    if (ret != RD_OK) return ret;

    std::memset(packet_obj, 0, sizeof(PACKET_comm_t));
    comm_error_counter_ = 0;

    std::cout << "[COMM Info] COMM Initialized Successfully" << std::endl;
    return RD_OK;
}

RD_RET RdComm::Stop() {
    if (!uart_) return RD_FATAL;
    return uart_->Stop();
}

// 주의: 버퍼 위생(flush)은 이 함수가 아니라 호출자 책임이다.
// 호출자(RdSchedule::ExecuteTask)가 매 트랜잭션 Write 직전에 comm_->Clear() 를 호출하므로,
// 어떤 에러로 빠져나가든(sync/length/body/CRC) 남은 잔여 바이트는 다음 사이클 시작 시 비워진다.
// (실패한 Read 와 다음 Write 사이에 포트를 읽는 코드가 없다는 전제 — 마스터 req→resp 구조)
RD_RET RdComm::Read(PACKET_comm_t* packet_obj, size_t header_timeout_ms, size_t body_timeout_ms) {
    if (!uart_ || !packet_obj) {
        std::cerr << "[COMM Fatal] Invalid UART or Packet Pointer!" << std::endl;
        return RD_FATAL;
    }

    // ---- Stage 1: 헤더 5바이트 읽기 ----
    uint8_t header_buf[HEADER_PART_LEN];
    RD_RET ret = uart_->Read(header_buf, HEADER_PART_LEN, header_timeout_ms);
    if (ret != RD_OK) return ret;

    // Header(2) + ID(1) 통합 검증.
    // Orin 이 마스터이므로 들어오는 모든 패킷은 반드시 ORIN ID(0x01).
    // 셋 중 하나라도 어긋나면 라인 동기화가 깨진 것 → 에러 처리 (flush 는 다음 Write 직전에).
    if (header_buf[0] != HEADER1 || header_buf[1] != HEADER2 || header_buf[2] != MY_ID) {
        return HandleCommError("Sync mismatch: " + PacketToString(header_buf, HEADER_PART_LEN));
    }

    // Length 파싱 (Little Endian)
    uint16_t length = static_cast<uint16_t>(header_buf[3]) |
                     (static_cast<uint16_t>(header_buf[4]) << 8);

    // Length sanity check
    if (length < MIN_LENGTH_FIELD || length > (MAX_DATA_LEN + 3)) {
        return HandleCommError("Invalid length field: " + std::to_string(length));
    }

    // ---- Stage 2: 나머지 Length 바이트 읽기 ----
    uint8_t body_buf[MAX_PACKET_LEN];
    ret = uart_->Read(body_buf, length, body_timeout_ms);
    if (ret != RD_OK) return ret; // 부분 수신 — 다음 Write 직전 flush 가 복구

    // ---- CRC 검증 (조립 전) ----
    uint16_t recv_crc = static_cast<uint16_t>(body_buf[length - 2]) |
                       (static_cast<uint16_t>(body_buf[length - 1]) << 8);
    uint16_t calc_crc = CalculateCRC16(header_buf, HEADER_PART_LEN);
    calc_crc = CalculateCRC16(body_buf, length - TAIL_LEN, calc_crc);
    if (recv_crc != calc_crc) {
        return HandleCommError("CRC Error: recv=0x" + ToHex(recv_crc) +
                               " calc=0x" + ToHex(calc_crc));
    }

    // ---- 구조체에 직접 파싱 ----
    // header_buf[2..4] = ID + Length(2) → pack 앞 3바이트와 일치
    std::memcpy(&packet_obj->rx.pack, &header_buf[2], HEADER_PART_LEN - 2);
    // body_buf[0..length-3] = Inst + Data → pack.Inst 이후와 일치 (CRC 2바이트 제외)
    std::memcpy(&packet_obj->rx.pack.Inst, body_buf, length - TAIL_LEN);
    packet_obj->rx.data_len = static_cast<uint16_t>(length - 3);

    comm_error_counter_ = 0;
    return RD_OK;
}

RD_RET RdComm::Write(PACKET_comm_t* packet_obj, size_t data_len) {
    if (!uart_ || !packet_obj) {
        std::cerr << "[COMM Fatal] Invalid UART or Packet Pointer!" << std::endl;
        return RD_FATAL;
    }
    if (data_len > MAX_DATA_LEN) {
        std::cerr << "[COMM Fatal] data_len exceeds MAX_DATA_LEN: " << data_len << std::endl;
        return RD_FATAL;
    }

    // Length = Inst(1) + Data(N) + CRC(2)
    packet_obj->tx.pack.Length = static_cast<uint16_t>(data_len + 3);
    packet_obj->tx.data_len    = static_cast<uint16_t>(data_len);

    // local buf 에 wire 패킷 구성: [H1][H2][ID+Length+Inst+Data][CRC]
    uint8_t buf[MAX_PACKET_LEN];
    buf[0] = HEADER1;
    buf[1] = HEADER2;
    // pack 앞 4바이트(ID+Length+Inst) + Data = data_len+4 바이트
    std::memcpy(&buf[2], &packet_obj->tx.pack, data_len + 4);

    size_t crc_offset = HEADER_PART_LEN + 1 + data_len;  // H1+H2+ID+Len+Inst+Data 끝
    uint16_t crc = CalculateCRC16(buf, crc_offset);
    buf[crc_offset]     = static_cast<uint8_t>(crc & 0xFF);
    buf[crc_offset + 1] = static_cast<uint8_t>((crc >> 8) & 0xFF);

    return uart_->Write(buf, crc_offset + TAIL_LEN);
}

void RdComm::Clear() {
    uart_->Flush();
}

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

uint16_t RdComm::CalculateCRC16(const uint8_t* data, size_t len, uint16_t init_crc) {
    uint16_t crc = init_crc;
    for (size_t j = 0; j < len; ++j) {
        uint16_t i = ((crc >> 8) ^ data[j]) & 0xFF;
        crc = (crc << 8) ^ s_crc_table[i];
    }
    return crc;
}

std::string RdComm::ToHex(uint16_t value) {
    std::stringstream ss;
    ss << std::hex << std::setw(4) << std::setfill('0') << value;
    return ss.str();
}

std::string RdComm::PacketToString(const uint8_t* pBuf, size_t len) {
    std::stringstream ss;
    ss << "[ ";
    for (size_t i = 0; i < len; ++i) {
        ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(pBuf[i]) << " ";
    }
    ss << "]";
    return ss.str();
}

RD_RET RdComm::HandleCommError(const std::string& msg) {
    comm_error_counter_++;
    if (comm_error_counter_ < COMM_ERR_IGNORE) {
        return RD_TIMEOUT;
    } else if (comm_error_counter_ < COMM_ERR_WARN) {
        std::cerr << "[COMM Warning] " << msg << " (Count: " << comm_error_counter_ << ")" << std::endl;
        return RD_ERROR;
    } else {
        std::cerr << "[COMM Fatal] Protocol Integrity Failed! " << msg << std::endl;
        return RD_FATAL;
    }
}

} // namespace orin_bridge
