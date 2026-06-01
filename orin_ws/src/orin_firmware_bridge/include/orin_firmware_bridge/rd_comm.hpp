#ifndef ORIN_FIRMWARE_BRIDGE__RD_COMM_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_COMM_HPP_

#include "orin_firmware_bridge/rd_uart.hpp"

namespace orin_bridge {

// --- Packet Definition (STM32와 100% 동일) ---
#pragma pack(push, 1)
typedef struct {
    uint8_t Header1;       // 0x40
    uint8_t Header2;       // 0x20
    uint8_t TargetID;
    uint8_t FunctionCode;
    uint8_t Index;
    uint8_t Data[8];
    uint8_t Checksum;
    uint8_t CR;            // 0x0D
    uint8_t LF;            // 0x0A
} PACKET_s_t; // 16 bytes
#pragma pack(pop)

// STM32의 PACKET_comm_t와 대응
typedef struct {
    PACKET_s_t tx;
    PACKET_s_t rx;
} PACKET_comm_t;

// --- Class Definition ---
class RdComm {
public:
    // 생성자: RdUart 포인터를 인자로 받음
    RdComm(RdUart* ptr);
    // 초기화
    RD_RET Init(PACKET_comm_t* packet_obj);

    // [RX] 패킷 수신 (Blocking Mode)
    // timeout_ms 동안 패킷을 기다림. 성공 시 packet_obj->rx에 데이터 채움
    RD_RET Read(PACKET_comm_t* packet_obj, size_t timeout_ms);

    // [TX] 패킷 송신
    // packet_obj->tx의 내용을 조립하여 전송
    RD_RET Write(PACKET_comm_t* packet_obj);

    void Clear();
private:
    RdUart* uart_; // 하위 레이어 포인터
    
    // Utility: Checksum 계산
    uint8_t CalculateChecksum(const PACKET_s_t* packet);

    // 상수 정의
    static const uint8_t HEADER1 = 0x40;
    static const uint8_t HEADER2 = 0x20;
    static const uint8_t TAIL_CR = 0x0D;
    static const uint8_t TAIL_LF = 0x0A;
    static const size_t  PACKET_LEN = 16;

    // Error Handling
    int comm_error_counter_ = 0; 
    const int COMM_ERR_IGNORE = 10; 
    const int COMM_ERR_WARN   = 30;
    RD_RET HandleCommError(const std::string& msg);

    std::string PacketToString(const uint8_t* pBuf, size_t len);
};

} // namespace orin_bridge

#endif