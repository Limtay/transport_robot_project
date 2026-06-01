#include "orin_firmware_bridge/rd_comm.hpp"
#include <iostream>
#include <cassert> // assert 사용
#include <iomanip>
#include <cstddef> // offsetof를 쓰기 위해 필요
#include <sstream> // stringstream 사용

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

    packet_obj->tx.Header1 = HEADER1;
    packet_obj->tx.Header2 = HEADER2;
    packet_obj->tx.CR = TAIL_CR;
    packet_obj->tx.LF = TAIL_LF;

    std::cout << "[COMM Info] COMM Initialized Successfully" << std::endl;
    return RD_OK;
}

RD_RET RdComm::Read(PACKET_comm_t* packet_obj, size_t timeout_ms) {
    if (!uart_ || !packet_obj) {
        std::cerr << "[COMM Fatal] Invalid UART or Packet Pointer!" << std::endl;
        return RD_FATAL;
    }
    uint8_t pBuf[PACKET_LEN];

    RD_RET ret = uart_->Read(pBuf, PACKET_LEN, timeout_ms);
    if (ret != RD_OK) return ret; // UART level error
    std::memcpy(&packet_obj->rx, &pBuf, PACKET_LEN); // 수신 데이터 복사
    
    // Header/Tail Check
    if (pBuf[0]  != HEADER1 || pBuf[1]  != HEADER2 ||
        pBuf[14] != TAIL_CR || pBuf[15] != TAIL_LF) {
        return HandleCommError("Header/Tail Mismatch " + PacketToString(pBuf, PACKET_LEN));
    }

    // Chcksum check
    uint8_t cal_checksum = CalculateChecksum(&(packet_obj->rx));
    if (packet_obj->rx.Checksum != cal_checksum) {
        return HandleCommError("Checksum Error " + PacketToString(pBuf, PACKET_LEN));
    }
    comm_error_counter_ = 0;
    return RD_OK; // 성공
}

RD_RET RdComm::Write(PACKET_comm_t* packet_obj) {
    if (!uart_ || !packet_obj) {
        std::cerr << "[COMM Fatal] Invalid UART or Packet Pointer!" << std::endl;
        return RD_FATAL;
    }
    
    packet_obj->tx.Checksum = CalculateChecksum(&(packet_obj->tx));

    return uart_->Write((uint8_t*)&(packet_obj->tx), PACKET_LEN);
}

void RdComm::Clear() {
    uart_->ClearFlash();
}

uint8_t RdComm::CalculateChecksum(const PACKET_s_t* packet) {
    const uint8_t* pBuf = (const uint8_t*)packet;
    uint32_t sum = 0;
    for (size_t i = 0; i < offsetof(PACKET_s_t, Checksum); i++) {
        sum += pBuf[i];
    }
    return (uint8_t)(sum & 0xFF);
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
        // 1st : ignore
        return RD_TIMEOUT;
    } 
    else if (comm_error_counter_ < COMM_ERR_WARN) {
        // 2nd : warning
        std::cerr << "[COMM Warning] " << msg << " (Count: " << comm_error_counter_ << ")" << std::endl;
        return RD_ERROR;
    } 
    else {
        // 3rd : fatal
        std::cerr << "[COMM Fatal] Protocol Integrity Failed!" << std::endl;
        return RD_FATAL; 
    }
}

} // namespace orin_bridge
