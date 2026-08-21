#ifndef ORIN_FIRMWARE_BRIDGE__RD_MAP_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_MAP_HPP_

#include <cstdint>
#include <initializer_list>
#include <mutex>
#include "orin_firmware_bridge/core/rd_comm.hpp"
#include "orin_firmware_bridge/core/rd_common.hpp"
#include "orin_firmware_bridge/core/rd_register_ecu.hpp"
#include "orin_firmware_bridge/core/rd_register_dpc.hpp"
#include "orin_firmware_bridge/core/rd_register_pcu.hpp"
#include "orin_firmware_bridge/core/rd_read_age.hpp"

namespace orin_bridge {

// ===== Target ID =====
namespace TARGET {
    constexpr uint8_t ORIN = static_cast<uint8_t>(PacketID::ORIN);
    constexpr uint8_t ECU  = static_cast<uint8_t>(PacketID::ECU);
    constexpr uint8_t DPC  = static_cast<uint8_t>(PacketID::DPC_B);
    constexpr uint8_t PCU  = static_cast<uint8_t>(PacketID::PCU);
}

// ===== READ 세그먼트 (멀티세그먼트 READ 용) =====
// STM READ(0x02) 확장: Param = [AddrL|H + LenL|H] × n — 비연속 레지스터 구간을
// 한 트랜잭션으로 읽는다. 응답 Data = Err(1) + seg0 | seg1 | ... 연접.
struct Segment_t {
    uint16_t addr;
    uint16_t len;
};

// ===== TaskConfig: 스케줄러 슬롯 정의 =====
struct TaskConfig_t {
    uint8_t    target_id;
    PacketInst inst;        // READ or WRITE
    uint16_t   start_addr;  // 레지스터 시작 주소
    uint16_t   data_len;    // READ: 읽을 바이트 수 / WRITE: 쓸 바이트 수 (start_addr 제외)

    // READ 전용 멀티세그먼트. seg_count==0 이면 기존 단일 구간(start_addr/data_len) 동작.
    static constexpr int MAX_READ_SEGS = 8;
    Segment_t  segs[MAX_READ_SEGS] = {};
    uint8_t    seg_count = 0;

    TaskConfig_t() = default;
    TaskConfig_t(uint8_t tid, PacketInst i, uint16_t addr, size_t len)
        : target_id(tid), inst(i), start_addr(addr), data_len(static_cast<uint16_t>(len)) {}
    // 멀티세그 READ 생성자 — start_addr/data_len 은 seg[0] 로 채워 로그/기존 코드와 호환 유지
    TaskConfig_t(uint8_t tid, PacketInst i, std::initializer_list<Segment_t> list)
        : target_id(tid), inst(i), start_addr(0), data_len(0) {
        for (const auto& s : list) {
            if (seg_count >= MAX_READ_SEGS) break;
            segs[seg_count++] = s;
        }
        if (seg_count > 0) { start_addr = segs[0].addr; data_len = segs[0].len; }
    }
    // RW 생성자 — start_addr/data_len = write 구간(shadow reg 에서 복사), segs = read 세그
    TaskConfig_t(uint8_t tid, PacketInst i, uint16_t waddr, size_t wlen,
                 std::initializer_list<Segment_t> read_segs)
        : target_id(tid), inst(i), start_addr(waddr), data_len(static_cast<uint16_t>(wlen)) {
        for (const auto& s : read_segs) {
            if (seg_count >= MAX_READ_SEGS) break;
            segs[seg_count++] = s;
        }
    }
};

// 보드별 레지스터 섀도의 시작 주소와 크기. **여기 하나만 둔다** — 종전에는
// rd_command.cpp 의 static 함수였고, 다른 곳에서 필요해지는 순간 복사될 자리였다.
// (호출자가 state_mutex 를 잡아야 한다. 이 함수는 포인터만 돌려준다.)
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
    // 섀도의 **동반 메타데이터** — 각 바이트를 마지막으로 읽은 시각 (09 §5.4 ①, U8).
    // 섀도와 같은 `state_mutex` 로 보호된다. 별도 객체로 빼서 생성자로 돌리지 않은 이유가
    // 이것이다 — 값과 그 값의 나이는 **같은 락 아래에서 같이** 갱신돼야 서로 어긋나지 않는다.
    ReadAgeMap read_age;
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

    // 응답의 read 연접 데이터(Data[1..])를 각 세그먼트 제 주소로 scatter — READ/RW 공용
    template <typename State>
    RD_RET ScatterReadSegs(PACKET_comm_t* pkt, const TaskConfig_t& sent_config,
                           State* node, uint16_t total_size);

    // RW write 거부(모드 lock 등) 로그 스팸 방지용 카운터 (200Hz 스트림 대비 간헐 출력).
    // write 성공 시 0 으로 리셋되므로 값 자체가 곧 "연속 거부 tick 수" —
    // 제어 FSM(§2)의 LOCKED 래치 판정에 그대로 쓰인다.
    uint64_t rw_write_err_cnt_ = 0;
    uint64_t rw_write_err_total_ = 0;   // 누적 (리셋 없음) — 프로파일 구간 집계용
    uint8_t  last_rw_err_ = 0;          // 직전 RW 응답의 에러 바이트 (read_err | write_err<<4)

public:
    // 연속 RW write 거부 tick 수 (0 = 정상). RdControl::NoteWriteErrStreak 소비용.
    uint64_t RwWriteErrStreak() const { return rw_write_err_cnt_; }
    // 누적 RW write 거부 횟수 (리셋 없음) — action result 의 write_err_cnt 는
    // 구간 차분(종료 시점 − 시작 시점)으로 구한다.
    uint64_t RwWriteErrTotal() const { return rw_write_err_total_; }
    // 직전 RW 응답의 에러 니블 — ControlFeedback.rw_err (§3.4) 로 그대로 나간다.
    uint8_t  LastRwErr() const { return last_rw_err_; }
};


// 보드별 레지스터 섀도의 시작 주소와 크기. **여기 하나만 둔다** — 종전에는 rd_command.cpp
// 의 static 함수였고, 다른 곳에서 필요해지는 순간 복사될 자리였다 (실제로 07 §2 Tab3 에서
// 필요해졌다). 포인터만 돌려주며, 호출자가 state_mutex 를 잡아야 한다.
inline uint8_t* ShadowBase(RobotState_t* st, uint8_t target_id, uint16_t* total_size) {
    switch (target_id) {
        case TARGET::ECU: *total_size = ecu::REG_TOTAL_SIZE; return reinterpret_cast<uint8_t*>(&st->ecu.reg);
        case TARGET::DPC: *total_size = dpc::REG_TOTAL_SIZE; return reinterpret_cast<uint8_t*>(&st->dpc.reg);
        case TARGET::PCU: *total_size = pcu::REG_TOTAL_SIZE; return reinterpret_cast<uint8_t*>(&st->pcu.reg);
        default:          *total_size = 0; return nullptr;
    }
}

} // namespace orin_bridge

#endif
