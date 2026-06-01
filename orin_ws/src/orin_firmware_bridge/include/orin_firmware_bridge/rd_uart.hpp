#ifndef ORIN_FIRMWARE_BRIDGE__RD_UART_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_UART_HPP_

#include <cstdint>
#include <vector>
#include <cstring>
#include <mutex>
#include <memory>
#include <atomic>
#include <condition_variable>
#include <libserial/SerialPort.h>
#include "orin_firmware_bridge/rd_common.hpp"

namespace orin_bridge {

#define RX_BUFFER_SIZE 32 
#define TX_BUFFER_SIZE 32  

class RdUart {
public:
    RdUart(const std::string& port_name_);
    ~RdUart();
    /**
     * @brief 시리얼 포트를 열고 링버퍼를 초기화합니다.
     * @note StopRx() 이후 재사용하려면 반드시 이 함수를 다시 호출해야 합니다.
     */
    RD_RET Init();

    /**
     * @brief 포트를 닫습니다.
     * @note 호출 후 객체는 Uninitialized 상태가 됩니다.
     */
    RD_RET Stop();

    void ClearFlash();
    /**
     * @brief 데이터를 전송합니다.
     * @warning Thread-Safe하지만, 내부 Mutex로 인해 RX Timeout(최대 10ms)만큼 지연될 수 있음.
     */
    RD_RET Write(uint8_t* pBuf, size_t length);

    // 링버퍼에서 데이터 꺼내기 (Pop)
    RD_RET Read(uint8_t* pBuf, size_t length, const size_t timeout_ms_);

private:
    // 송수신 버퍼
    std::vector<uint8_t> rx_buffer; 
    std::vector<uint8_t> tx_buffer;

    // error 카운터
    int rx_error_counter_;
    int tx_error_counter_;

    // 에러 임계값 상수
    const int ERR_LIMIT_IGNORE = 10; // 10번까진 무시 (Timeout 처리)
    const int ERR_LIMIT_WARN   = 30; // 30번까진 경고 (Error 처리)
    RD_RET HandleErrorState(int& counter, const std::string& msg);
    
    // 초기화 상태 플래그
    std::atomic<bool> is_initialized_;
    // RX 스레드 비정상 종료 감지용 플래그
    std::atomic<bool> is_rx_error_; 

    std::unique_ptr<LibSerial::SerialPort> serial_port_;
    
    std::mutex rx_mutex_;           
    std::condition_variable rx_cv_; 

    std::mutex port_mutex_;

    const std::string port_name;
};

} // namespace orin_bridge

#endif