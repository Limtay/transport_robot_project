#include "orin_firmware_bridge/rd_uart.hpp"
#include <iostream>
// #include <algorithm>
#include <cstdio> // printf 사용을 위해 추가
#include <cstring>     // strerror, memcpy
#include <cerrno>      // errno
#include <chrono>      // steady_clock deadline
#include <poll.h>      // poll() 기반 수신
#include <unistd.h>    // read()

namespace orin_bridge {

RdUart::RdUart(const std::string& port_name_) : is_initialized_(false), port_name(port_name_) {
    rx_buffer.reserve(RX_BUFFER_SIZE);
    tx_buffer.reserve(TX_BUFFER_SIZE);
}

RdUart::~RdUart() {
    Stop();
}

RD_RET RdUart::Init() {
    std::lock_guard<std::mutex> lock(port_mutex_);
    if (is_initialized_) return RD_OK;  // 이미 초기화된 경우 무시
    std::cout << "[UART Info] Initializing UART on port: " << port_name << std::endl;
    try {
        std::cout << "[UART debug #0] Creating new SerialPort object..." << std::endl;
        serial_port_ = std::make_unique<LibSerial::SerialPort>();
        std::cout << "[UART debug #1] SerialPort object created." << std::endl;
        serial_port_->Open(port_name);
        std::cout << "[UART debug #2] SerialPort opened." << std::endl;
        serial_port_->SetBaudRate(LibSerial::BaudRate::BAUD_921600);
        serial_port_->SetCharacterSize(LibSerial::CharacterSize::CHAR_SIZE_8);
        serial_port_->SetStopBits(LibSerial::StopBits::STOP_BITS_1);
        serial_port_->SetParity(LibSerial::Parity::PARITY_NONE);
        serial_port_->SetVMin(0);
        serial_port_->SetVTime(0);
    } catch (const std::exception& e) { // [수정] 구체적인 에러 잡기
        std::cerr << "[UART Fatal] Open Failed: " << e.what() << std::endl;
        return RD_FATAL;
    } catch (...) { // [수정] 그 외 알 수 없는 에러
        std::cerr << "[UART Fatal] Unknown Exception!" << std::endl;
        return RD_FATAL;
    }

    rx_error_counter_ = 0;
    tx_error_counter_ = 0;
    is_initialized_ = true;

    std::cout << "[UART Info] UART Initialized Successfully on port: " << port_name << std::endl;
    return RD_OK;
}

RD_RET RdUart::Stop() {
    std::lock_guard<std::mutex> lock(port_mutex_);

    is_initialized_ = false;
    rx_error_counter_ = 0;
    tx_error_counter_ = 0;

    if (serial_port_ && serial_port_->IsOpen()) {
    try {
        serial_port_->FlushInputBuffer();
        serial_port_->Close();
    } catch (const std::exception& e) {
        std::cerr << "[UART Fatal] Close Failed during Stop(): " << e.what() << std::endl;
        return RD_FATAL;
    } catch (...) {
        std::cerr << "[UART Fatal] Unknown Exception during Stop()" << std::endl;
        return RD_FATAL;
    }
    }
    return RD_OK;
}

void RdUart::Flush() {
    std::lock_guard<std::mutex> lock(port_mutex_);
    if (serial_port_ && serial_port_->IsOpen()) {
        serial_port_->FlushInputBuffer();
    } else {
        std::cerr << "[UART Warning] Flush called but Serial Port not open!" << std::endl;
    }
}

RD_RET RdUart::Write(uint8_t* pBuf, size_t length) {
    if (!pBuf || !is_initialized_) { // NULL pointer or not initialized
        std::cerr << "[UART Fatal] UART TX Not Initialized!" << std::endl;
        return RD_FATAL;
    }

    std::lock_guard<std::mutex> lock(port_mutex_);
    if (!serial_port_ || !serial_port_->IsOpen()) { // Not open UART Port
        std::cout << "[UART Fatal] Serial Port Not Open!" << std::endl;
        return RD_FATAL;
    }
    try {
        tx_buffer.assign(pBuf, pBuf + length);
        serial_port_->Write(tx_buffer);
        // [필수] 반이중(RS485)에서는 송신이 물리적으로 끝날 때까지 블록해야 한다.
        // Write() 는 커널/USB 버퍼에 큐잉만 하고 즉시 리턴하므로, Drain 없이 바로 Read 하면
        // 송신·턴어라운드 구간과 수신이 겹쳐 0x00 프레이밍 에러가 쏟아지고 STM 은
        // 깨끗한 요청을 못 받아 응답하지 않는다(Rx 0). tcdrain 으로 마지막 stop bit 까지 비운다.
        serial_port_->DrainWriteBuffer();
    } catch (const std::exception& e) { // Standard Exception
        return HandleErrorState(tx_error_counter_, "TX Error: " + std::string(e.what()));
    } catch (...) { // Unknown Exception
        return HandleErrorState(tx_error_counter_, "TX Unknown Exception");
    }
    tx_buffer.clear();
    tx_error_counter_ = 0;
    return RD_OK;
}


RD_RET RdUart::Read(uint8_t* pBuf, size_t length, const size_t timeout_ms_) {
    if (!pBuf || !is_initialized_) { // NULL pointer or not initialized
        std::cerr << "[UART Fatal] UART RX Not Initialized!" << std::endl;
        return RD_FATAL;
    }

    std::lock_guard<std::mutex> io_lock(port_mutex_);
    if (!serial_port_ || !serial_port_->IsOpen()) { // Not open UART Port
        std::cerr << "[UART Fatal] Serial Port Not Open!" << std::endl;
        return RD_FATAL;
    }
    const int fd = serial_port_->GetFileDescriptor();
    if (fd < 0) return HandleErrorState(rx_error_counter_, "RX Invalid FD");

    // poll() 기반 수신 — LibSerial 의 busy-poll(스핀+벽시계 비교) 대신 커널 블로킹 사용.
    // 커널이 '데이터 도착' 시점에 깨우므로, 읽기 도중 스레드가 선점(preempt)당해도
    // 벽시계 기준 오탐 타임아웃이 나지 않는다. (← 노드 내부 손실의 직접 원인 제거)
    // VMIN=0/VTIME=0 으로 열려 있어 POLLIN 후 read() 는 가용 바이트를 즉시 반환한다.
    const auto deadline = std::chrono::steady_clock::now() +
                          std::chrono::milliseconds(timeout_ms_);
    size_t got = 0;
    while (got < length) {
        const auto now = std::chrono::steady_clock::now();
        if (now >= deadline) break; // timeout
        const auto remain_us =
            std::chrono::duration_cast<std::chrono::microseconds>(deadline - now).count();
        const int remain_ms = static_cast<int>((remain_us + 999) / 1000); // 올림, 최소 1ms

        struct pollfd pfd { fd, POLLIN, 0 };
        const int pr = ::poll(&pfd, 1, remain_ms);
        if (pr < 0) {
            if (errno == EINTR) continue; // 시그널 — 재시도
            return HandleErrorState(rx_error_counter_,
                std::string("RX poll error: ") + std::strerror(errno));
        }
        if (pr == 0) break; // poll timeout — 데이터 없음
        if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) {
            return HandleErrorState(rx_error_counter_, "RX poll HUP/ERR");
        }
        if (pfd.revents & POLLIN) {
            const ssize_t n = ::read(fd, pBuf + got, length - got);
            if (n > 0) {
                got += static_cast<size_t>(n);
            } else if (n == 0) {
                break; // EOF (시리얼에선 드묾)
            } else if (errno != EINTR && errno != EAGAIN && errno != EWOULDBLOCK) {
                return HandleErrorState(rx_error_counter_,
                    std::string("RX read error: ") + std::strerror(errno));
            }
        }
    }

    if (got < length) { // 타임아웃 / 부분수신 — 다음 Write 직전 flush 가 복구
        return HandleErrorState(rx_error_counter_,
            "RX Timeout (got " + std::to_string(got) + "/" + std::to_string(length) + ")");
    }

    rx_error_counter_ = 0;
    return RD_OK;
}

RD_RET RdUart::HandleErrorState(int& counter, const std::string& msg) {
    counter++; // error counter increment
    if (counter < ERR_LIMIT_IGNORE) {
        // 1st : ignore
        return RD_TIMEOUT;
    } 
    else if (counter < ERR_LIMIT_WARN) {
        // 2nd : warning
        std::cerr << "[UART Warning] " << msg << " (Count: " << counter << ")" << std::endl;
        return RD_ERROR;
    } 
    else {
        // 3rd : fatal
        std::cerr << "[UART FATAL] " << msg << " Limit Exceeded! (" << counter << ")" << std::endl;
        return RD_FATAL;
    }
}

}// namespace orin_bridge