#ifndef ORIN_FIRMWARE_BRIDGE__RD_COMMON_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_COMMON_HPP_

#include <cstdint>
//#include <vector>

namespace orin_bridge {

/**
 * @brief Return Status for Orin Bridge functions
 * STM32의 rd_common.h와 호환성을 유지하되, 
 * 리눅스 스레드 환경을 고려해 RD_CLOSED 상태를 추가함.
 */
typedef enum {
    RD_OK      = 0x00U, // 성공
    RD_TIMEOUT = 0x01U, // 일시적 에러 ( < 10)  ->  시간 초과
    RD_ERROR   = 0x02U, // 지속적 에러 ( > 10)  ->  경고 로그 출력
    RD_FATAL   = 0x03U, // 치명적 에러 ( > 30)  ->  에러 누적 및 연결 끊김 ->   프로그램 종료 필요
    RD_CLOSED  = 0x04U  // 스레드가 정상 종료됨   ->  Stop() 호출
} RD_RET;


} // namespace orin_bridge

#endif