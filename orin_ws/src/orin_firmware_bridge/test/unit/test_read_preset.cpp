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

// id 0 의 배치를 못박는다. 세 번 바뀌었고 그 이력이 곧 이 프리셋의 설계 근거다:
//
//   구  `{27,5},{42,6},{88,36},{164,16},{32,1},{128,4}`  6세그  68B
//   →   `{16,17},{42,86},{128,4},{164,16}`               4세그 123B  (2026-07-30, 04 §2.3)
//   →   `{16,17},{48,80}`                                2세그  97B  (2026-08-04, 09 §1.1)
//
// 마지막 변경에서 빠진 것은 셋이다 — **어느 것도 "그냥 줄인" 것이 아니다** (09 §1.2):
//   `{42,6}`   로드셀 → control_test 의 관심사. control 에서는 미판독으로 나간다
//   `{128,4}`  ctr_mode read-back → 섀도가 곧 write 버퍼라 검증이 무조건 통과했다.
//              그래서 `DoInSpanCtrMode` 의 검증 자체를 없앴다
//   `{164,16}` cmd_current read-back → `ControlFeedback.cmd` 의 의미가
//              "ECU 가 받은 값" → "브리지가 보낸 값" 으로 바뀐다
TEST(ReadPresetTable, DefaultPresetIsControlWithTheSection91Layout) {
    ASSERT_EQ(ecu::kPresets[0], &ecu::kPresetControl);
    EXPECT_STREQ(ecu::kPresetControl.name, "control");
    EXPECT_EQ(ecu::kPresetControl.count, 2);
    EXPECT_EQ(ecu::kPresetControl.RespPayload(), 17 + 80);   // = 97
}

// diag 배치 (09 §1.1). `{188,4}` 가 신규다 — TAB3 의 ECU 버튼(auto_mode/soft_estop/
// mode/use_lpf)이 실제 레지스터를 따라갈 수 있는 **유일한** 경로다.
//
// memo_260731 은 첫 세그를 `{0,16}` 이라 적었지만 `{0,33}` 으로 간다. `{0,16}` 이면
// SYS(16:17)가 빠지고, 그건 아래 EveryPresetReadsTheWholeSysBlock 이 막는 바로 그것이다.
TEST(ReadPresetTable, DiagLayoutIncludesCmdSystemReadback) {
    EXPECT_EQ(ecu::kPresetDiag.count, 4);
    EXPECT_EQ(ecu::kPresetDiag.RespPayload(), 33 + 4 + 4 + 16);   // = 57
    EXPECT_TRUE(ecu::kPresetDiag.Covers(ecu::REG_AUTO_MODE_OFFSET, 4))
        << "188:4 (auto_mode/soft_estop/mode/use_lpf) 가 빠졌다 — TAB3 리드백이 사라진다";
    // 네 필드가 정말 연속인가. 헤더가 재배치되면 여기가 먼저 깨져야 한다.
    EXPECT_EQ(ecu::REG_SOFT_ESTOP_OFFSET, ecu::REG_AUTO_MODE_OFFSET + 1);
    EXPECT_EQ(ecu::REG_MODE_OFFSET,       ecu::REG_AUTO_MODE_OFFSET + 2);
    EXPECT_EQ(ecu::REG_USE_LPF_OFFSET,    ecu::REG_AUTO_MODE_OFFSET + 3);
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

// 04 §2.3 변경 2 — control 은 진단 채널의 STATE_t 를 확보한다 (03 §3.1).
// static_assert 가 이미 보지만, 실패 메시지에 어느 채널인지 남기려면 여기가 낫다.
//
// ⚠ **6개 중 5개다** (09 §1.1). 로드셀(idx5, addr 47)은 `{42,6}` 과 함께 빠졌고
//    control_test 가 맡는다 — 아래 LoadcellStateMovedToControlTest 가 그것을 지킨다.
//    "6개 전부" 를 여기에 남겨 두면 로드셀이 어디로 갔는지 아무도 모르게 된다.
TEST(ReadPresetTable, ControlCoversFiveDiagnosticStateBytes) {
    struct Ch { const char* name; uint16_t addr; };
    const Ch chs[] = {
        {"idx0 RC(uart1)",   ecu::REG_SENSOR_RC_OFFSET},                              // 87
        {"idx1 RS485(uart2)",ecu::REG_UART2_OFFSET},                                   // 86
        {"idx2 IMU(uart6)",  static_cast<uint16_t>(ecu::REG_IMU_OFFSET + ecu::REG_IMU_SIZE - 1)},          // 69
        {"idx3 모터(can1)",  static_cast<uint16_t>(ecu::REG_MOTOR_DATA_OFFSET + ecu::REG_MOTOR_DATA_SIZE - 1)}, // 127
        {"idx4 엔코더(i2c1)",static_cast<uint16_t>(ecu::REG_ENCODER_OFFSET + ecu::REG_ENCODER_SIZE - 1)},  // 85
    };
    for (const auto& c : chs)
        EXPECT_TRUE(ecu::kPresetControl.Covers(c.addr, 1))
            << c.name << " STATE_t(addr " << c.addr << ") 가 세그 밖 — lc/hs 가 미판독이 된다";
}

// 로드셀 STATE_t(47) 를 읽는 프리셋이 **하나는 있어야 한다.** control 에서 뺀 것이
// "아무도 안 읽는다" 가 되면 06 §9.9 (엔코더가 통째로 사라졌던 것)와 같은 공백이 된다.
TEST(ReadPresetTable, LoadcellStateMovedToControlTest) {
    constexpr uint16_t kLcState = ecu::REG_LOADCELL_OFFSET + ecu::REG_LOADCELL_SIZE - 1;  // 47
    EXPECT_FALSE(ecu::kPresetControl.Covers(kLcState, 1))
        << "control 이 로드셀을 다시 읽는다 — 09 §1.1 에서 뺀 것이다";
    EXPECT_TRUE(ecu::kPresetControlTest.Covers(kLcState, 1))
        << "control_test 마저 로드셀 STATE_t(47) 를 잃으면 어느 프리셋도 안 읽는다";
    bool any = false;
    for (uint8_t i = 0; i < ecu::kPresetCount; i++)
        any = any || ecu::kPresets[i]->Covers(ecu::REG_LOADCELL_OFFSET, ecu::REG_LOADCELL_SIZE);
    EXPECT_TRUE(any) << "로드셀(42:6)을 읽는 프리셋이 하나도 없다";
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

// ★★ 이 테스트도 **뒤집혔다** (2026-08-04, 09 §1.2).
//
// 종전 이름은 `WritingPresetsKeepCtrModeReadback` 이었고 *"SET_CTR_MODE 검증이 이
// read-back 에 의존하므로 쓰기 프리셋은 반드시 유지해야 한다"* 를 고정했다.
// **그 전제가 틀렸다.** 섀도는 read-back 의 목적지인 동시에 **write 버퍼**다
// (`RdControl::PrepareWrite` 가 같은 구조체에 쓴다). 그래서 read-back 이 있든 없든
// 검증 루프는 브리지 자신이 쓴 값을 읽고 있었고 — 있을 때는 ECU 값으로 덮이기를
// 기다렸을 뿐 — 없으면 **1 tick 만에 무조건 통과**한다.
//
// 무조건 통과하는 검증은 거짓 보증이라 `DoInSpanCtrMode` 의 검증을 없앴고, control 에서
// `{128,4}` 를 뺐다. 관측이 필요하면 **diag 로 갈아끼운다** — 그래서 diag 는 유지한다.
TEST(ReadPresetTable, ControlDropsCtrModeReadbackAndDiagKeepsIt) {
    EXPECT_FALSE(ecu::kPresetControl.Covers(ecu::REG_CMD_MOTOR_OFFSET, 4))
        << "control 에 ctr_mode read-back 이 다시 들어왔다 — 20B 가 늘 뿐 검증은 여전히 "
           "브리지가 쓴 값을 읽는다 (09 §1.2 ①)";
    EXPECT_FALSE(ecu::kPresetControl.Covers(ecu::REG_CMD_CURRENT_OFFSET, 16))
        << "control 에 cmd_current read-back 이 다시 들어왔다 — ControlFeedback.cmd 의 "
           "의미가 조용히 바뀐다 (09 §1.2 ②)";
    EXPECT_TRUE(ecu::kPresetDiag.Covers(ecu::REG_CMD_MOTOR_OFFSET, 4))
        << "diag 가 ctr_mode 를 잃었다 — control 에서 뺀 뒤로 여기가 유일한 관측 경로다";
}

// `ControlFeedback.cmd` 의 의미가 프리셋마다 다르다는 것을 표로 고정한다 (09 §1.2 ②).
// 이 비대칭이 결정의 대가이고, 모르면 bag 분석에서 두 가지를 같은 값으로 섞게 된다.
TEST(ReadPresetTable, CmdFieldMeaningDependsOnPreset) {
    // control_test: auto_mode:none 이라 브리지가 안 쓴다 + 164:16 을 읽는다 → ECU 실값
    EXPECT_TRUE(ecu::kPresetControlTest.Covers(ecu::REG_CMD_CURRENT_OFFSET, 16))
        << "control_test 의 cmd 는 STM RC 램프가 만든 ECU 실값이다 — 견인 실험의 본체";
    // control/diag: 읽지 않는다 → 브리지가 보낸 값
    EXPECT_FALSE(ecu::kPresetControl.Covers(ecu::REG_CMD_CURRENT_OFFSET, 16));
    EXPECT_FALSE(ecu::kPresetDiag.Covers(ecu::REG_CMD_CURRENT_OFFSET, 16));
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
