#ifndef ORIN_FIRMWARE_BRIDGE__RD_MAP_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_MAP_HPP_

#include <cstdint>
#include <mutex>
#include "orin_firmware_bridge/rd_comm.hpp"
#include "orin_firmware_bridge/rd_common.hpp"
#include "orin_firmware_bridge/rd_register_ecu.hpp"
#include "orin_firmware_bridge/rd_register_dpc.hpp"
#include "orin_firmware_bridge/rd_register_pcu.hpp"

namespace orin_bridge {

// ===== Target ID =====
namespace TARGET {
    constexpr uint8_t ORIN = static_cast<uint8_t>(PacketID::ORIN);
    constexpr uint8_t ECU  = static_cast<uint8_t>(PacketID::ECU);
    constexpr uint8_t DPC  = static_cast<uint8_t>(PacketID::DPC_B);
    constexpr uint8_t PCU  = static_cast<uint8_t>(PacketID::PCU);
}

// ===== TaskConfig: 스케줄러 슬롯 정의 =====
struct TaskConfig_t {
    uint8_t    target_id;
    PacketInst inst;        // READ or WRITE
    uint16_t   start_addr;  // 레지스터 시작 주소
    uint16_t   data_len;    // READ: 읽을 바이트 수 / WRITE: 쓸 바이트 수 (start_addr 제외)

    TaskConfig_t() = default;
    TaskConfig_t(uint8_t tid, PacketInst i, uint16_t addr, size_t len)
        : target_id(tid), inst(i), start_addr(addr), data_len(static_cast<uint16_t>(len)) {}
};

// ===== 통신 health =====
struct CommHealth_t {
    uint16_t timeout_cnt  = 0;
    bool     is_connected = false;
};

// ===== ECU 상태 =====
struct EcuState_t {
    CommHealth_t    comm;
    ecu::REGISTER_t reg;
};

// ===== DPC =====
struct DpcState_t {
    CommHealth_t    comm;
    dpc::REGISTER_t reg;
};

// ===== PCU =====
struct PcuState_t {
    CommHealth_t    comm;
    pcu::REGISTER_t reg;
};

// ===== 통합 로봇 상태 =====
struct RobotState_t {
    EcuState_t ecu;
    DpcState_t dpc;
    PcuState_t pcu;
    mutable std::mutex state_mutex;
};

// ===== RdMap =====
class RdMap {
public:
    RdMap() = default;
    ~RdMap() = default;

    // Encode: config에 따라 패킷 Data 영역을 채우고 송신할 data_len 반환
    RD_RET Encode(const TaskConfig_t& config, RobotState_t* state,
                  PACKET_comm_t* pkt, size_t* out_data_len);

    // Decode: sent_config(마지막 송신 task)를 참조해 응답 데이터를 state에 반영
    // (STM 응답 inst 는 송신 inst 를 그대로 echo)
    RD_RET Decode(PACKET_comm_t* pkt, const TaskConfig_t& sent_config, RobotState_t* state);

private:
    static constexpr uint16_t TIMEOUT_MAX = 3;

    // 노드별 Encode/Decode 는 동작이 100% 동일하고, 차이는 (1) 상태 타입과
    // (2) 레지스터 총 크기(total_size)뿐이라 템플릿 하나로 통합한다.
    // State 는 {CommHealth_t comm; <REGISTER_t> reg;} 레이아웃을 만족하는 노드 상태 타입.
    // total_size 는 그 노드의 REG_TOTAL_SIZE (노드마다 달라도 무방).
    template <typename State>
    RD_RET EncodeNode(const TaskConfig_t& config, State* node, uint16_t total_size,
                      PACKET_comm_t* pkt, size_t* out_data_len);
    template <typename State>
    RD_RET DecodeNode(PACKET_comm_t* pkt, const TaskConfig_t& sent_config,
                      State* node, uint16_t total_size);

    // 레지스터 블록을 평평한 바이트 배열로 보고 start_addr 위치의 포인터/잔여 크기를 돌려준다.
    template <typename State>
    static uint8_t* GetRegionPtr(State* node, uint16_t start_addr,
                                 uint16_t total_size, size_t* region_size);
};

} // namespace orin_bridge

#endif
