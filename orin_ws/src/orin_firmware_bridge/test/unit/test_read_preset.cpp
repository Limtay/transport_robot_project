// 읽기 프리셋 테이블 (redesign/04 §2.3, §2.4.2)
//
// 막는 것: 프리셋을 바꾼 뒤 **어떤 섀도 필드가 더 이상 갱신되지 않는지 모르는 것**.
// 그걸 모르고 발행하면 낡은 값이 신선한 값처럼 나간다 — 이 프로젝트가 반복해서
// 지워 온 결함 형태다 (06 §4.11, §4.12).
//
// wire 예산은 static_assert 가 컴파일 타임에 이미 봤다 (rd_read_preset.hpp).
// 여기서는 **Covers() 의 의미**와 기본 프리셋의 불변식을 고정한다.

#include <gtest/gtest.h>

#include "orin_firmware_bridge/core/rd_read_preset.hpp"

namespace {

using orin_bridge::ReadPreset;
namespace ecu = orin_bridge::ecu;

// id 0 의 배치를 못박는다. **2026-07-30 04 §2.3 적용으로 바뀌었다** —
// 종전 `{27,5},{42,6},{88,36},{164,16},{32,1},{128,4}` (6세그 68B) 에서
// `{16,17},{42,86},{128,4},{164,16}` (4세그 123B) 으로.
// 세그가 줄고 payload 는 늘었다: SYS 전체 + 42~127 연속을 확보한 결과다.
TEST(ReadPresetTable, DefaultPresetIsControlWithTheSection23Layout) {
    ASSERT_EQ(ecu::kPresets[0], &ecu::kPresetControl);
    EXPECT_STREQ(ecu::kPresetControl.name, "control");
    EXPECT_EQ(ecu::kPresetControl.count, 4);
    EXPECT_EQ(ecu::kPresetControl.RespPayload(), 17 + 86 + 4 + 16);   // = 123
}

TEST(ReadPresetTable, RespPayloadIsDerivedNotHandWritten) {
    // 04 §2.2.3 — 개수·크기에서 파생시킨다. 사람이 더해 적으면 프리셋을 추가할 때마다
    // 틀린다 (04 §2.5 가 지적한 "43 을 손으로 옮겨 적어 payload 와 wire 를 섞은" 사고).
    for (uint8_t i = 0; i < ecu::kPresetCount; i++) {
        uint16_t sum = 0;
        for (uint8_t s = 0; s < ecu::kPresets[i]->count; s++)
            sum = static_cast<uint16_t>(sum + ecu::kPresets[i]->spans[s].len);
        EXPECT_EQ(ecu::kPresets[i]->RespPayload(), sum) << ecu::kPresets[i]->name;
        EXPECT_LE(ecu::kPresets[i]->RespPayload(), orin_bridge::kMaxRespPayload);
    }
}

// ★ Covers() 의 의미 — 부분 포함은 false 다.
//
// ⚠ 종전에는 이 성질을 `kPresetControl` 로 검사했다. 그래서 §2.3 적용으로 control 이
//    42~127 을 연속으로 덮게 된 순간 **Covers() 는 멀쩡한데 테스트가 깨졌다.**
//    Covers() 의 의미를 보는 테스트가 특정 프리셋의 배치에 묶여 있으면 안 된다 —
//    합성 프리셋으로 본다.
TEST(ReadPresetTable, PartialOverlapIsNotCovered) {
    // 두 세그먼트에 걸친 블록을 "읽었다" 고 하면 그 안의 어떤 필드가 낡았는지 알 수 없다.
    constexpr ReadPreset split = {"split", {{88, 36}, {124, 4}}, 2};

    EXPECT_TRUE (split.Covers(88, 36));
    EXPECT_FALSE(split.Covers(88, 40)) << "두 세그에 걸쳐 있으면 덮은 것이 아니다";
    EXPECT_FALSE(split.Covers(88 + 30, 10)) << "시작은 세그 안, 끝은 밖";
    EXPECT_FALSE(split.Covers(70, 4))       << "완전히 밖";
    // 인접한 두 세그를 합치면 덮는다 — "연속으로 읽어야 한 블록으로 취급된다".
    constexpr ReadPreset merged = {"merged", {{88, 40}}, 1};
    EXPECT_TRUE(merged.Covers(88, 40));
}

// ★★ 이 테스트는 **뒤집혔다** (2026-07-30).
//
// 종전 이름은 `ControlDoesNotReadSysBlock` 이었고, *"hw_error 가 계속 미판독인 것은 버그가
// 아니라 프리셋이 그 구간을 안 읽는 것"* 이라고 적어 두고 그 상태를 **기대값으로 고정**했다.
// 설명은 맞았지만 **그걸 고정한 것이 잘못이었다** — 그 결과 엔코더 버스가 100% degraded 인
// 것을 몇 달 못 봤다 (06 §10 B1). 04 §2.3 은 그래서 전 프리셋이 SYS 를 읽도록 정했다.
//
// 관측 불가 상태를 "의도된 것" 으로 테스트에 박아 두면, 그 테스트는 결함을 보호한다.
TEST(ReadPresetTable, EveryPresetReadsTheWholeSysBlock) {
    for (uint8_t i = 0; i < ecu::kPresetCount; i++) {
        EXPECT_TRUE(ecu::kPresets[i]->Covers(ecu::REG_SYS_OFFSET, ecu::REG_SYS_SIZE))
            << ecu::kPresets[i]->name
            << " 가 SYS 를 온전히 안 읽는다 — degraded_cnt·hw_error·hw_fatal 이 미판독이 되고,"
               " 그건 '정상' 과 구분되지 않는다 (04 §2.3 변경 1)";
    }
    // 프레임 전용 프리셋도 같은 규칙을 따른다 (Q3 가 sys 슬롯을 없앤 근거).
    EXPECT_TRUE(ecu::kPresetProject.Covers(ecu::REG_SYS_OFFSET, ecu::REG_SYS_SIZE));
}

// 04 §2.3 변경 2 — control 은 진단 채널 6개의 STATE_t 를 **전부** 확보한다 (03 §3.1).
// static_assert 가 이미 보지만, 실패 메시지에 어느 채널인지 남기려면 여기가 낫다.
TEST(ReadPresetTable, ControlCoversAllSixDiagnosticStateBytes) {
    struct Ch { const char* name; uint16_t addr; };
    const Ch chs[] = {
        {"idx0 RC(uart1)",   ecu::REG_SENSOR_RC_OFFSET},                              // 87
        {"idx1 RS485(uart2)",ecu::REG_UART2_OFFSET},                                   // 86
        {"idx2 IMU(uart6)",  static_cast<uint16_t>(ecu::REG_IMU_OFFSET + ecu::REG_IMU_SIZE - 1)},          // 69
        {"idx3 모터(can1)",  static_cast<uint16_t>(ecu::REG_MOTOR_DATA_OFFSET + ecu::REG_MOTOR_DATA_SIZE - 1)}, // 127
        {"idx4 엔코더(i2c1)",static_cast<uint16_t>(ecu::REG_ENCODER_OFFSET + ecu::REG_ENCODER_SIZE - 1)},  // 85
        {"idx5 로드셀(adc)", static_cast<uint16_t>(ecu::REG_LOADCELL_OFFSET + ecu::REG_LOADCELL_SIZE - 1)},// 47
    };
    for (const auto& c : chs)
        EXPECT_TRUE(ecu::kPresetControl.Covers(c.addr, 1))
            << c.name << " STATE_t(addr " << c.addr << ") 가 세그 밖 — lc/hs 가 미판독이 된다";
}

// 06 §9.9 의 공백 회복 — Q3 로 엔코더를 읽는 프리셋이 하나도 없어졌던 것.
TEST(ReadPresetTable, SomePresetStillReadsTheEncoderBlock) {
    bool any = ecu::kPresetProject.Covers(ecu::REG_ENCODER_OFFSET, ecu::REG_ENCODER_SIZE);
    for (uint8_t i = 0; i < ecu::kPresetCount; i++)
        any = any || ecu::kPresets[i]->Covers(ecu::REG_ENCODER_OFFSET, ecu::REG_ENCODER_SIZE);
    EXPECT_TRUE(any) << "엔코더(70:16)를 읽는 프리셋이 하나도 없다 — link_angle 이 전 모드 NaN 이다";
    EXPECT_TRUE(ecu::kPresetControl.Covers(ecu::REG_ENCODER_OFFSET, ecu::REG_ENCODER_SIZE))
        << "control 이 그 역할을 맡는다 (04 §2.3: project 는 링크 각을 쓰지 않는다)";
}

TEST(ReadPresetTable, DiagDropsMotorBlockOnPurpose) {
    // diag 로 갈아끼우면 fb_position/velocity/current 는 갱신되지 않는다.
    // 텔레메트리가 그 자리를 NaN 으로 내보내야 한다 (rd_telemetry.cpp).
    EXPECT_FALSE(ecu::kPresetDiag.Covers(ecu::REG_MOTOR_DATA_OFFSET, 36));
    EXPECT_FALSE(ecu::kPresetDiag.Covers(ecu::REG_LOADCELL_OFFSET, ecu::REG_LOADCELL_SIZE));
}

// ★ **쓰기를 하는** 프리셋은 ctr_mode read-back 을 유지해야 한다.
//
// SET_CTR_MODE 검증이 이 read-back 에 의존한다. 빠진 프리셋으로 갈아끼우면 ctr_mode
// 설정이 "검증 시간초과" 로 실패한다 — 둘 다 IDLE 전용이라 실제로 겹치는 조합이다.
//
// `control_test` 는 예외다: `auto_mode: none` 전용이라 **검증할 write 자체가 없다.**
// 그래서 read-back 을 빼고, 그 덕에 구 traction 배치와 wire 가 바이트 단위로 같다.
TEST(ReadPresetTable, WritingPresetsKeepCtrModeReadback) {
    for (uint8_t i = 0; i < ecu::kPresetCount; i++) {
        const auto* p = ecu::kPresets[i];
        if (p == &ecu::kPresetControlTest) continue;   // READ 전용
        EXPECT_TRUE(p->Covers(ecu::REG_CMD_MOTOR_OFFSET, 4))
            << p->name << " 가 ctr_mode read-back 을 잃었다";
    }
}

// 이름으로 고를 수 있어야 한다 (01 §4.2 단어형 기동 파라미터).
TEST(ReadPresetTable, ParsesEveryNameAndRejectsTypos) {
    uint8_t id = 0xFF;
    for (uint8_t i = 0; i < ecu::kPresetCount; i++) {
        ASSERT_TRUE(ecu::ParseReadPreset(ecu::kPresets[i]->name, &id)) << ecu::kPresets[i]->name;
        EXPECT_EQ(id, i);
    }
    EXPECT_FALSE(ecu::ParseReadPreset("contorl", &id));
    EXPECT_FALSE(ecu::ParseReadPreset("", &id));
}

TEST(ReadPresetTable, SegmentCountWithinTaskLimit) {
    for (uint8_t i = 0; i < ecu::kPresetCount; i++)
        EXPECT_LE(ecu::kPresets[i]->count, orin_bridge::kMaxReadSegs);
}

}  // namespace

// ★ 2026-07-29 실기 회귀 — 기동 직후 "안 읽는 블록" 이 읽은 것처럼 보이던 문제.
//
// 구 코드의 Reads() 는 프리셋 포인터가 널이면 **true** 를 돌려줬다 ("미설정이면 기존 동작").
// 그런데 기동 시엔 SetReadPreset() 이 한 번도 안 불려 계속 널이었고, 그래서
// **읽지도 않은 SYS 블록의 낡은 섀도 값 0 이 "정상" 으로 발행됐다.**
// 실제 hw_error 는 16(미연결 엔코더)이었다 — 프리셋을 diag 로 바꾸고서야 드러났다.
//
// 널일 때 true 는 "안 읽는 것까지 읽었다고 주장" 하는 쪽이라, 이 함수가 막으려던 일을
// 그대로 하게 된다. 기본값은 **실제 기본 프리셋**이어야 한다.
TEST(ReadPresetTable, DefaultIsARealPresetNotPermissive) {
    // ⚠ 종전 이 테스트의 첫 단정은 *"control 은 SYS 를 읽지 않는다"* 였다. 그것은 이 회귀의
    //    본질이 아니라 **당시 control 배치의 우연**이었고, §2.3 적용으로 뒤집혔다.
    //    회귀의 본질은 "널/빈 프리셋이 전부 true 를 돌려주는 것" 이다 — 그것만 본다.
    //    (control 이 SYS 를 읽는다는 사실은 EveryPresetReadsTheWholeSysBlock 이 지킨다.)

    // "아무것도 모르는 프리셋" 은 아무것도 덮지 않아야 한다 —
    // 빈 프리셋이 전부 true 를 돌려주면 그것이 곧 구 버그다.
    constexpr ReadPreset empty = {"empty", {}, 0};
    EXPECT_FALSE(empty.Covers(ecu::REG_SYS_OFFSET, ecu::REG_SYS_SIZE));
    EXPECT_FALSE(empty.Covers(ecu::REG_MOTOR_DATA_OFFSET, 36));
    EXPECT_EQ(empty.RespPayload(), 0);
}
