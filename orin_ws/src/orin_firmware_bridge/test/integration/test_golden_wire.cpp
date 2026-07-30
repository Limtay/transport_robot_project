// 골든 바이트 테스트 — RS485 와이어 계약을 바이트 단위로 고정한다 (redesign/06 §2.2).
//
// 왜 이 테스트가 필요한가:
//   기존 42개 테스트는 전부 `rd_test_fixture.hpp` 를 통해 실제 `RdNode` 노드를 띄운다.
//   그런데 재설계(02 A2)는 그 `RdNode` 를 4개로 쪼갠다 — 즉 **검증 대상 자체가 사라진다.**
//   리팩터링하면서 테스트도 같이 고치면, 그 테스트는 더 이상 "안 바뀌었음" 을 증명하지 못한다.
//
//   그래서 분할 대상이 아닌 **L1(`RdMap`)의 출력 바이트열**을 고정한다.
//   구조를 어떻게 바꾸든 ECU 로 나가는 바이트는 같아야 한다 — 그것이 "동작 불변" 의
//   가장 엄밀한 정의다. rclcpp 도 실기도 필요 없다.
//
// 갱신 방법 (의도적으로 동작을 바꿀 때만):
//   RD_GOLDEN_UPDATE=1 로 실행하면 골든 파일을 다시 쓴다.
//   ⚠ 06 §5.2 — 골든 불일치는 **예외 없이 롤백**이 원칙이다. 기대값을 고치는 것은
//     "동작을 의도적으로 바꾼다" 는 별도 결정일 때만이며, 그건 재설계가 아니라 기능 변경이다.

#include <gtest/gtest.h>

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "orin_firmware_bridge/core/rd_map.hpp"
#include "orin_firmware_bridge/core/rd_read_preset.hpp"   // 프리셋 표에서 세그를 가져온다

namespace {

using orin_bridge::PacketInst;
using orin_bridge::RdMap;
using orin_bridge::RobotState_t;
using orin_bridge::Segment_t;
using orin_bridge::TaskConfig_t;
namespace ecu = orin_bridge::ecu;

// 골든 파일 경로는 CMake 가 -D 로 넘긴다 (빌드 디렉터리가 아니라 소스 트리를 가리켜야 한다).
#ifndef RD_GOLDEN_PATH
#define RD_GOLDEN_PATH "golden_wire.txt"
#endif

// shadow 를 결정론적 패턴으로 채운다. 0 으로 두면 write 구간이 전부 0 이라
// "어느 바이트가 어디서 왔는지" 를 구분하지 못해 오프셋 실수를 못 잡는다.
void FillDeterministic(RobotState_t* st) {
    auto* raw = reinterpret_cast<uint8_t*>(&st->ecu.reg);
    for (size_t i = 0; i < sizeof(ecu::REGISTER_t); i++) {
        raw[i] = static_cast<uint8_t>((i * 7u + 13u) & 0xFFu);
    }
}

std::string ToHex(const uint8_t* p, size_t n) {
    static const char* kHex = "0123456789ABCDEF";
    std::string out;
    out.reserve(n * 3);
    for (size_t i = 0; i < n; i++) {
        if (i) out.push_back(' ');
        out.push_back(kHex[p[i] >> 4]);
        out.push_back(kHex[p[i] & 0x0F]);
    }
    return out;
}

// 한 케이스 = 이름 + TaskConfig. Encode 결과를 "이름 | data_len | hex" 한 줄로 만든다.
struct Case {
    std::string  name;     // 프리셋 이름을 붙여 만들므로 소유해야 한다
    TaskConfig_t cfg;
};

std::string EncodeLine(const Case& c) {
    RobotState_t st;
    FillDeterministic(&st);

    RdMap                   map;
    orin_bridge::PACKET_comm_t pkt{};
    size_t                  len = 0;

    const auto ret = map.Encode(c.cfg, &st, &pkt, &len);
    std::ostringstream os;
    os << c.name << " | ret=" << static_cast<int>(ret) << " | len=" << len << " | "
       << ToHex(pkt.tx.pack.Data, len);
    return os.str();
}

// 06 §2.2 의 케이스 목록. 프리셋·auto_mode·INST 조합을 덮는다.
std::vector<Case> BuildCases() {
    std::vector<Case> v;

    // ── 단일 구간 READ (영역별) ────────────────────────────────
    v.push_back({"read_sys",
                 TaskConfig_t(orin_bridge::TARGET::ECU, PacketInst::READ,
                              ecu::REG_SYS_OFFSET, ecu::REG_SYS_SIZE)});
    v.push_back({"read_motor_data",
                 TaskConfig_t(orin_bridge::TARGET::ECU, PacketInst::READ,
                              ecu::REG_MOTOR_DATA_OFFSET, ecu::REG_MOTOR_DATA_SIZE)});
    v.push_back({"read_diag",
                 TaskConfig_t(orin_bridge::TARGET::ECU, PacketInst::READ,
                              ecu::REG_DIAG_OFFSET, ecu::REG_DIAG_SIZE)});
    v.push_back({"read_full_scan",
                 TaskConfig_t(orin_bridge::TARGET::ECU, PacketInst::READ, 0,
                              ecu::REG_TOTAL_SIZE)});

    // ── 멀티세그 READ — **살아 있는 프리셋에서 세그를 가져온다** ─────────────
    //
    // ⚠ 종전에는 세그 목록을 여기 손으로 적어 뒀다. 그래서 2026-07-30 에 control·
    //    control_test 프리셋 배치가 §2.3 대로 바뀌었을 때 **골든이 아무 말도 하지 않았다** —
    //    골든이 고정하고 있던 것은 실제로 나가는 wire 가 아니라 "예전에 나갔던 wire" 였다.
    //    (같은 구멍을 Q3 때 project 에서도 봤다.)
    //
    //    기대값(바이트)은 여전히 골든 파일이 소유한다 — 프리셋을 고치면 diff 가 뜬다.
    //    입력을 표에서 가져오는 것과 기대값을 표에서 계산하는 것은 다르다.
    for (uint8_t i = 0; i < ecu::kPresetCount; i++) {
        const auto* pr = ecu::kPresets[i];
        TaskConfig_t t(orin_bridge::TARGET::ECU, PacketInst::READ, 0, 0);
        for (uint8_t k = 0; k < pr->count; k++)
            t.segs[t.seg_count++] = Segment_t{pr->spans[k].addr, pr->spans[k].len};
        t.start_addr = t.segs[0].addr;
        t.data_len   = t.segs[0].len;
        v.push_back({std::string("read_preset_") + pr->name, t});
    }

    // ── WRITE (out-of-span 계열: motor_mask / mode / auto_mode) ─
    v.push_back({"write_motor_mask",
                 TaskConfig_t(orin_bridge::TARGET::ECU, PacketInst::WRITE, 192, 1)});
    v.push_back({"write_ecu_mode",
                 TaskConfig_t(orin_bridge::TARGET::ECU, PacketInst::WRITE, 190, 1)});
    v.push_back({"write_auto_mode",
                 TaskConfig_t(orin_bridge::TARGET::ECU, PacketInst::WRITE, 188, 1)});
    v.push_back({"write_cmd_system_8",
                 TaskConfig_t(orin_bridge::TARGET::ECU, PacketInst::WRITE,
                              ecu::REG_CMD_SYSTEM_OFFSET, 8)});

    // ── RW: auto_mode 별 write 범위 (01 §3) ────────────────────
    // 읽기 세그는 **기본 프리셋(control)에서 가져온다** — 스케줄러가 실제로 그렇게 만든다
    // (rd_schedule.cpp 의 task_control_ 굽기). 손으로 적으면 프리셋과 갈라진다.
    std::vector<Segment_t> ctrl_segs;
    for (uint8_t k = 0; k < ecu::kPresetControl.count; k++)
        ctrl_segs.push_back({ecu::kPresetControl.spans[k].addr, ecu::kPresetControl.spans[k].len});
    auto rw = [&ctrl_segs](uint16_t waddr, uint16_t wlen) {
        TaskConfig_t t(orin_bridge::TARGET::ECU, PacketInst::RW, waddr, wlen);
        for (const auto& g : ctrl_segs) t.segs[t.seg_count++] = g;
        return t;
    };

    v.push_back({"rw_current_164_16",   rw(164, 16)});                    // auto_mode=1 CURRENT
    v.push_back({"rw_direct_128_52",                                       // =2 DIRECT (P8 조합)
                 rw(ecu::REG_CMD_MOTOR_OFFSET, ecu::REG_CMD_MOTOR_SIZE)});
    v.push_back({"rw_velocity_148_16", rw(148, 16)});                      // =4 VELOCITY
    v.push_back({"rw_position_132_16", rw(132, 16)});                      // =5 POSITION
    v.push_back({"rw_kinematic_180_8", rw(ecu::REG_CMD_SYSTEM_OFFSET, 8)});// =0 KINEMATIC

    // ── Q3 project RW — 종전 project 의 세 갈래(센서 READ 100Hz / cmd_vel WRITE 50Hz /
    //    sys READ 10Hz)를 대체한 **새 wire 모양**이다. 이 케이스가 없으면 project 의
    //    바이트가 바뀌어도 골든이 아무 말을 하지 않는다 (재편 당시 실제로 그랬다).
    // project 세그도 표(kPresetProject)에서 가져온다 — 여기 손으로 적으면 같은 구멍이 다시 생긴다.
    std::vector<Segment_t> proj_segs;
    for (uint8_t k = 0; k < ecu::kPresetProject.count; k++)
        proj_segs.push_back({ecu::kPresetProject.spans[k].addr, ecu::kPresetProject.spans[k].len});

    TaskConfig_t proj_rw(orin_bridge::TARGET::ECU, PacketInst::RW, ecu::REG_CMD_SYSTEM_OFFSET, 8);
    for (const auto& g : proj_segs) proj_rw.segs[proj_rw.seg_count++] = g;
    v.push_back({"rw_project_cmdvel_180_8", proj_rw});

    // manual 은 같은 구간을 **읽기만** 한다 — 쓰기가 섞이면 여기서 바이트가 달라진다.
    TaskConfig_t proj_rd(orin_bridge::TARGET::ECU, PacketInst::READ, 0, 0);
    for (const auto& g : proj_segs) proj_rd.segs[proj_rd.seg_count++] = g;
    proj_rd.start_addr = proj_rd.segs[0].addr;
    proj_rd.data_len   = proj_rd.segs[0].len;
    v.push_back({"read_project_preset", proj_rd});

    // ── 다른 보드 (DPC / PCU) ─────────────────────────────────
    v.push_back({"read_dpc_sys",
                 TaskConfig_t(orin_bridge::TARGET::DPC, PacketInst::READ, 0, 16)});
    v.push_back({"read_pcu_sys",
                 TaskConfig_t(orin_bridge::TARGET::PCU, PacketInst::READ, 0, 16)});

    return v;
}

std::vector<std::string> ReadGolden(const std::string& path) {
    std::vector<std::string> lines;
    std::ifstream            f(path);
    std::string              line;
    while (std::getline(f, line)) {
        if (!line.empty() && line[0] != '#') lines.push_back(line);
    }
    return lines;
}

TEST(GoldenWire, EncodeMatchesGolden) {
    const auto cases = BuildCases();

    std::vector<std::string> actual;
    actual.reserve(cases.size());
    for (const auto& c : cases) actual.push_back(EncodeLine(c));

    const std::string path = RD_GOLDEN_PATH;

    if (std::getenv("RD_GOLDEN_UPDATE") != nullptr) {
        std::ofstream f(path);
        ASSERT_TRUE(f.is_open()) << "골든 파일을 쓸 수 없다: " << path;
        f << "# 골든 바이트 — RdMap::Encode 출력 (redesign/06 §2.2)\n"
             "# 형식: <케이스> | ret=<RD_RET> | len=<data_len> | <Data[] hex>\n"
             "# shadow 는 raw[i] = (i*7+13) & 0xFF 패턴으로 채운다.\n"
             "# ⚠ 이 파일을 손으로 고치지 말 것. 재생성은 RD_GOLDEN_UPDATE=1 이며,\n"
             "#   그것은 '동작을 의도적으로 바꾼다' 는 결정일 때만이다 (06 §5.2).\n";
        for (const auto& l : actual) f << l << "\n";
        GTEST_SKIP() << "골든 파일 재생성: " << path;
    }

    const auto expected = ReadGolden(path);
    ASSERT_FALSE(expected.empty())
        << "골든 파일이 없거나 비어 있다: " << path
        << "\n최초 생성: RD_GOLDEN_UPDATE=1 <이 테스트 실행파일>";

    ASSERT_EQ(expected.size(), actual.size())
        << "케이스 수가 다르다 — 케이스를 추가/삭제했다면 골든을 재생성해야 한다";

    for (size_t i = 0; i < actual.size(); i++) {
        EXPECT_EQ(expected[i], actual[i])
            << "\n와이어 바이트가 바뀌었다 (케이스 " << i << ")."
            << "\n  기대: " << expected[i] << "\n  실제: " << actual[i]
            << "\n구조 개편이 ECU 로 나가는 바이트를 바꿨다는 뜻이다 — 06 §5.2 에 따라 롤백한다.";
    }
}

// Encode→Decode 왕복: 응답을 만들어 되돌렸을 때 shadow 가 원래 값으로 복원되는가.
// Encode 만 고정하면 '읽어들이는 쪽' 의 오프셋 실수를 못 잡는다.
TEST(GoldenWire, DecodeRoundTripRestoresShadow) {
    // 견인 프리셋으로 READ 를 보냈다고 가정하고, 그 세그먼트들의 실제 바이트를 응답에 싣는다.
    const TaskConfig_t cfg(orin_bridge::TARGET::ECU, PacketInst::READ,
                           {{27, 5},
                            {ecu::REG_LOADCELL_OFFSET, ecu::REG_LOADCELL_SIZE},
                            {ecu::REG_MOTOR_DATA_OFFSET, 36},
                            {ecu::REG_PROC_DELTA_OFFSET, 1}});

    RobotState_t src;
    FillDeterministic(&src);

    // 응답 Data = err(1) + 세그먼트 연접
    orin_bridge::PACKET_comm_t pkt{};
    const auto* src_raw = reinterpret_cast<const uint8_t*>(&src.ecu.reg);
    size_t      n       = 0;
    pkt.rx.pack.Data[n++] = 0;  // err = OK
    for (uint8_t s = 0; s < cfg.seg_count; s++) {
        std::memcpy(&pkt.rx.pack.Data[n], src_raw + cfg.segs[s].addr, cfg.segs[s].len);
        n += cfg.segs[s].len;
    }
    // 응답 패킷의 ID 는 **수신자(ORIN)** 다 — 요청의 대상 ID 와 다르다.
    pkt.rx.pack.ID   = orin_bridge::TARGET::ORIN;
    pkt.rx.pack.Inst = static_cast<uint8_t>(PacketInst::READ);
    pkt.rx.data_len  = static_cast<uint16_t>(n);

    RobotState_t dst;  // 0 으로 시작
    std::memset(&dst.ecu.reg, 0, sizeof(ecu::REGISTER_t));

    RdMap map;
    ASSERT_EQ(map.Decode(&pkt, cfg, &dst), orin_bridge::RD_RET::RD_OK);

    // 읽어온 세그먼트 구간만 원본과 같아야 한다 (그 밖은 0 그대로)
    const auto* dst_raw = reinterpret_cast<const uint8_t*>(&dst.ecu.reg);
    for (uint8_t s = 0; s < cfg.seg_count; s++) {
        for (uint16_t k = 0; k < cfg.segs[s].len; k++) {
            const uint16_t a = cfg.segs[s].addr + k;
            EXPECT_EQ(dst_raw[a], src_raw[a])
                << "addr " << a << " (세그 " << static_cast<int>(s) << ") 복원 실패";
        }
    }

    // 특히 2026-07-27 에 옮긴 proc_delta(32) 가 제자리에 들어갔는가
    EXPECT_EQ(dst.ecu.reg.sys.rs485_proc_delta, src.ecu.reg.sys.rs485_proc_delta);
    EXPECT_EQ(dst.ecu.reg.sys.sys_state,        src.ecu.reg.sys.sys_state);
    EXPECT_EQ(dst.ecu.reg.sys.realtime_tick,    src.ecu.reg.sys.realtime_tick);
}

}  // namespace
