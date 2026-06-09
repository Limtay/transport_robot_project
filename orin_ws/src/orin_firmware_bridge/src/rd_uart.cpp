#include "orin_firmware_bridge/rd_uart.hpp"
#include <iostream>
// #include <algorithm>
#include <cstdio> // printf 사용을 위해 추가

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

    {   // Scoped Lock
        std::lock_guard<std::mutex> io_lock(port_mutex_);
        if (!serial_port_ || !serial_port_->IsOpen()) { // Not open UART Port
            std::cerr << "[UART Fatal] Serial Port Not Open!" << std::endl;
            return RD_FATAL;
        }
        try {
            serial_port_->Read(rx_buffer, length, timeout_ms_);
        } catch (const LibSerial::ReadTimeout&) { // Timeout Error
            return HandleErrorState(rx_error_counter_, "RX Timeout");
        } catch (const std::runtime_error& e) { // Runtime Error
            std::string error_msg = e.what();
            if (error_msg.find("Success") != std::string::npos ) return RD_TIMEOUT;
            return HandleErrorState(rx_error_counter_, "RX Runtime Error: " + error_msg);
        } catch (const std::exception& e) {// Standard Exception
            return HandleErrorState(rx_error_counter_, "RX Std Error: " + std::string(e.what()));
        } catch (...) { // Unknown Exception
            return HandleErrorState(rx_error_counter_, "RX Unknown Exception");
        }

    } 
    if (rx_buffer.size() >= length) { // Data Available & Copy
        std::memcpy(pBuf, rx_buffer.data(), length);
    }
    rx_buffer.clear();
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