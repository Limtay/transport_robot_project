// 표 → TaskConfig_t 굽기 (`BakeFrame`, rd_slot_table.hpp)
//
// ## 왜 이 테스트가 없었고, 없어서 무엇을 놓쳤나
//
// 굽기는 `RdSchedule` 생성자 안의 람다였고 결과(`project_task_[]`)는 private 이었다.
// 골든 바이트 테스트(test_golden_wire)는 `TaskConfig_t` 를 **손으로 만들어** 쓰므로 이
// 변환을 지나가지 않는다. 즉 표와 wire 사이의 이 한 칸이 **아무 테스트도 안 보는 구간**이었다.
//
// 그 사이에 결함이 있었다 (2026-08-05 발견):
//
//   `TaskConfig_t::start_addr/data_len` 의 의미가 inst 마다 다르다 (rd_map.hpp)
//     READ — seg[0] 의 사본. 로그 호환용이고 wire 에는 안 쓰인다
//     RW   — **write 구간.** EncodeNode 가 이 주소로 섀도를 복사해 보낸다
//
//   구 `fill_segs` 는 READ 관례를 RW 에도 적용했다. 그래서 project 프레임의 ECU RW 는
//   **cmd_vel 을 180:8 이 아니라 16:17(SYS, 읽기 전용 구간)에 쓰려고 했다.**
//
// 실기에서 안 드러난 이유는 06 §9.9 에 있다 — Q3(RW 도입, 2026-07-29)는
// *"실기 타이밍 확인, **주행 검증은 미실시**"* 였다. 주행을 시켰다면 첫 tick 에 드러났다.

#include <gtest/gtest.h>

#include "orin_firmware_bridge/sched/rd_slot_table.hpp"

namespace {

using orin_bridge::BakeFrame;
using orin_bridge::PacketInst;
using orin_bridge::SlotId;
using orin_bridge::TaskConfig_t;
using orin_bridge::kMaxFrameTicks;
namespace ecu = orin_bridge::ecu;
namespace frames = orin_bridge::frames;

// ★ 이 테스트가 이 파일의 이유다.
//
// **0 이 아닌 값으로 판별한다** — 기대 180, 결함 시 16. 둘 다 0 이 아니고 서로 다르므로
// "안 채워졌다" 와 "잘못 채워졌다" 가 구분된다. 기대값이 0 이었다면 판별이 안 됐다.
TEST(FrameBake, ProjectRwKeepsWriteSpanAndDoesNotInheritReadSeg0) {
    TaskConfig_t task[kMaxFrameTicks]{}, read_only[kMaxFrameTicks]{};
    BakeFrame(frames::kProject, task, read_only);

    int rw_seen = 0;
    for (uint8_t t = 0; t < frames::kProject.ticks; t++) {
        if (frames::kProject.slots[t].id != SlotId::ECU) continue;
        ASSERT_EQ(task[t].inst, PacketInst::RW) << "tick " << int(t);
        rw_seen++;

        // write 구간 — 표가 적은 그대로여야 한다 (cmd_system 180:8 = cmd_vel).
        EXPECT_EQ(task[t].start_addr, ecu::REG_CMD_SYSTEM_OFFSET)
            << "tick " << int(t)
            << ": RW 의 start_addr 가 write 주소가 아니다. read seg[0](=SYS 16)로 덮였다면 "
               "cmd_vel 이 읽기 전용 구간으로 나간다";
        EXPECT_EQ(task[t].data_len, 8u) << "tick " << int(t);

        // 읽기 구간 — project 프리셋 그대로.
        ASSERT_EQ(task[t].seg_count, ecu::kPresetProject.count) << "tick " << int(t);
        for (uint8_t i = 0; i < ecu::kPresetProject.count; i++) {
            EXPECT_EQ(task[t].segs[i].addr, ecu::kPresetProject.spans[i].addr);
            EXPECT_EQ(task[t].segs[i].len,  ecu::kPresetProject.spans[i].len);
        }
    }
    EXPECT_EQ(rw_seen, 5) << "project 의 ECU RW 는 5칸 (Q3)";
}

// 구 결함을 **여기에 박제한다.** 이 규칙을 다시 도입하면 어떤 값이 나오는지를 적어 두면,
// 다음에 누가 "READ 랑 RW 랑 같이 처리하면 되지 않나" 라고 할 때 답이 코드 안에 있다.
TEST(FrameBake, OldRuleWouldHaveClobberedTheWriteAddress) {
    TaskConfig_t t(orin_bridge::TARGET::ECU, PacketInst::RW,
                   ecu::REG_CMD_SYSTEM_OFFSET, 8);
    for (uint8_t i = 0; i < ecu::kPresetProject.count; i++)
        t.segs[t.seg_count++] = orin_bridge::Segment_t{ecu::kPresetProject.spans[i].addr,
                                                       ecu::kPresetProject.spans[i].len};
    ASSERT_EQ(t.start_addr, ecu::REG_CMD_SYSTEM_OFFSET);   // 여기까지는 옳다

    // 구 fill_segs 의 마지막 두 줄 — READ 관례를 무조건 적용했다.
    t.start_addr = t.segs[0].addr;
    t.data_len   = t.segs[0].len;

    EXPECT_EQ(t.start_addr, ecu::REG_SYS_OFFSET)
        << "구 규칙의 결과: write 주소가 SYS(16) 가 된다 — 이것이 결함의 정체다";
    EXPECT_EQ(t.data_len, ecu::REG_SYS_SIZE);
    EXPECT_NE(t.start_addr, ecu::REG_CMD_SYSTEM_OFFSET);
}

// READ 슬롯에서는 같은 관례가 **옳다** — seg[0] 사본이 로그·구 코드 호환용이고
// wire 에는 안 쓰인다. RW 만 예외라는 것을 못박는다.
TEST(FrameBake, ReadSlotsStillMirrorSeg0) {
    TaskConfig_t task[kMaxFrameTicks]{};
    BakeFrame(frames::kManual, task, nullptr);

    int rd_seen = 0;
    for (uint8_t t = 0; t < frames::kManual.ticks; t++) {
        if (frames::kManual.slots[t].id != SlotId::ECU) continue;
        ASSERT_EQ(task[t].inst, PacketInst::READ) << "manual 의 ECU 는 READ 다 (01 §4.1)";
        rd_seen++;
        ASSERT_GT(task[t].seg_count, 0);
        EXPECT_EQ(task[t].start_addr, task[t].segs[0].addr);
        EXPECT_EQ(task[t].data_len,   task[t].segs[0].len);
    }
    EXPECT_EQ(rd_seen, 5);
}

// DPC·PCU 슬롯도 표가 적은 구간을 그대로 들고 나가야 한다.
TEST(FrameBake, DpcAndPcuSlotsCarryTheirOwnSpans) {
    TaskConfig_t task[kMaxFrameTicks]{}, read_only[kMaxFrameTicks]{};
    BakeFrame(frames::kProject, task, read_only);

    bool dpc = false, pcu = false;
    for (uint8_t t = 0; t < frames::kProject.ticks; t++) {
        const auto id = frames::kProject.slots[t].id;
        if (id == SlotId::DPC) {
            dpc = true;
            EXPECT_EQ(task[t].target_id, orin_bridge::TARGET::DPC);
            EXPECT_EQ(task[t].inst, PacketInst::READ);
            EXPECT_EQ(task[t].seg_count, orin_bridge::spans::kDpcProject20Hz.count);
        } else if (id == SlotId::PCU) {
            pcu = true;
            EXPECT_EQ(task[t].target_id, orin_bridge::TARGET::PCU);
            EXPECT_EQ(task[t].inst, PacketInst::READ);
        }
    }
    EXPECT_TRUE(dpc) << "project 에 DPC 슬롯이 없다";
    EXPECT_TRUE(pcu) << "project 에 PCU 슬롯이 없다";
}

// RW 의 읽기 폴백(cmd_vel 일시정지용)은 **같은 읽기 구간**이어야 한다 —
// 다르면 정지 중과 주행 중의 시계열 필드 구성이 갈라진다.
TEST(FrameBake, RwReadFallbackHasSameReadSegs) {
    TaskConfig_t task[kMaxFrameTicks]{}, read_only[kMaxFrameTicks]{};
    BakeFrame(frames::kProject, task, read_only);

    for (uint8_t t = 0; t < frames::kProject.ticks; t++) {
        if (frames::kProject.slots[t].id != SlotId::ECU) continue;
        ASSERT_EQ(read_only[t].inst, PacketInst::READ) << "tick " << int(t);
        ASSERT_EQ(read_only[t].seg_count, task[t].seg_count);
        for (uint8_t i = 0; i < task[t].seg_count; i++) {
            EXPECT_EQ(read_only[t].segs[i].addr, task[t].segs[i].addr);
            EXPECT_EQ(read_only[t].segs[i].len,  task[t].segs[i].len);
        }
    }
}

}  // namespace
