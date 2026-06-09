#ifndef ORIN_FIRMWARE_BRIDGE__RD_COMM_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_COMM_HPP_

#include <string>
#include "orin_firmware_bridge/rd_uart.hpp"

namespace orin_bridge {

// ===== 패킷 크기 상수 =====
constexpr size_t HEADER_PART_LEN  = 5;    // Header1, Header2, ID, Length[2]
constexpr size_t TAIL_LEN         = 2;    // CRC16
constexpr size_t MAX_PACKET_LEN   = 256;
constexpr size_t MAX_DATA_LEN     = 248;  // MAX_PACKET_LEN - HEADER_PART_LEN - 1(Inst) - TAIL_LEN
constexpr size_t MIN_LENGTH_FIELD = 3;    // Inst(1) + CRC(2), Data 0바이트 케이스

// ===== 패킷 마커 =====
constexpr uint8_t HEADER1 = 0xAA;
constexpr uint8_t HEADER2 = 0x55;
constexpr uint8_t MY_ID   = 0x01; // ORIN ID (수신 필터링용)

// ===== ID / Inst 정의 =====
enum class PacketID : uint8_t {
    ORIN  = 0x01,
    DPC_A = 0xD1,
    DPC_B = 0xD2,
    ECU   = 0xE1,
    PCU   = 0xA1,
};

enum class PacketInst : uint8_t {
    PING     = 0x01,
    READ     = 0x02,
    WRITE    = 0x03,
    REBOOT   = 0x08,
};

// ===== STM32 응답 Data[0] 에러 코드 =====
enum class PacketError : uint8_t {
    NONE        = 0x00,
    RESULT_FAIL = 0x01,
    INST        = 0x02,
    CRC         = 0x03,
    DATA_RANGE  = 0x04,
    DATA_LEN    = 0x05,
    DATA_LIMIT  = 0x06,
    ACCESS      = 0x07,
};

inline const char* PacketErrorStr(PacketError e) {
    switch (e) {
        case PacketError::NONE:        return "NONE";
        case PacketError::RESULT_FAIL: return "Result Fail";
        case PacketError::INST:        return "Instruction Error";
        case PacketError::CRC:         return "CRC Error";
        case PacketError::DATA_RANGE:  return "Data Range Error";
        case PacketError::DATA_LEN:    return "Data Length Error";
        case PacketError::DATA_LIMIT:  return "Data Limit Error";
        case PacketError::ACCESS:      return "Access Error";
        default:                       return "Unknown Error";
    }
}

// ===== 패킷 구조체 =====
typedef struct __attribute__((packed)) {
    uint8_t  ID;
    uint16_t Length;         // LE: Inst(1) + Data(N) + CRC(2) = N+3
    uint8_t  Inst;
    uint8_t  Data[MAX_DATA_LEN];
} PACKET_fields_t;

typedef struct {
    PACKET_fields_t pack;
    uint16_t        data_len;  // Data[] 유효 바이트 수 (CRC/헤더 제외)
} PACKET_s_t;

typedef struct {
    PACKET_s_t rx;
    PACKET_s_t tx;
} PACKET_comm_t;

// ===== RdComm Class =====
class RdComm {
public:
    explicit RdComm(RdUart* ptr);

    RD_RET Init(PACKET_comm_t* packet_obj);

    // 하위 UART 포트를 닫아 Uninitialized 상태로 되돌린다.
    // 치명 오류 복구 시 Init() 전에 호출해 깨진 fd/카운터를 강제로 리셋한다.
    RD_RET Stop();

    // [RX] Two-stage 가변 길이 패킷 수신
    // header_timeout_ms: 헤더 5바이트 대기 (STM 응답 시작 대기)
    // body_timeout_ms  : 바디 N+3바이트 대기 (헤더 수신 후 바디 전송)
    // 두 stage 합이 5ms 주기를 넘지 않도록 호출 (권장 2 + 2)
    RD_RET Read(PACKET_comm_t* packet_obj, size_t header_timeout_ms, size_t body_timeout_ms);

    // [TX] 가변 길이 패킷 송신
    // data_len: tx.f.Data 영역의 유효 바이트 수
    RD_RET Write(PACKET_comm_t* packet_obj, size_t data_len);

    void Clear();

private:
    RdUart* uart_;

    uint16_t CalculateCRC16(const uint8_t* data, size_t len, uint16_t init_crc = 0);
    static std::string ToHex(uint16_t value);
    std::string PacketToString(const uint8_t* pBuf, size_t len);

    int comm_error_counter_ = 0;
    const int COMM_ERR_IGNORE = 5;
    const int COMM_ERR_WARN   = 20;
    RD_RET HandleCommError(const std::string& msg);
};

} // namespace orin_bridge

#endif
