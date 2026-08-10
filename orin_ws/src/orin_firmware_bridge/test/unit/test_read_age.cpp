// 섀도 바이트별 읽기 시각 (redesign/09 §5.4 ①, U8)
//
// ## 무엇을 고정하는가
//
// `OP_GET_REGISTERS` 가 종전에 준 것은 `fresh`(신선 구간 목록) 하나였고, 그건 **2값**이라
//   · 5ms 전 프리셋 값
//   · 30초 전 기동 스냅샷
//   · 한 번도 안 읽은 자리
// 셋 중 뒤의 둘을 같은 회색으로 그렸다. 조작자가 화면의 0 을 보고 "레지스터가 0" 인지
// "안 읽어서 0" 인지 판별할 수 없다는 뜻이다.
//
// 여기서 못박는 것은 그 셋이 **서로 다른 값으로 구분된다**는 것이고, 특히
//   ① 겹치는 구간에서 **나중 읽기가 이긴다** (kAll 42:151 안에 프리셋 48:80 이 들어 있다)
//   ② 그래서 구간이 **쪼개진다** — 이 쪼개짐이 곧 "일부만 실시간" 이라는 사실의 표현이다
// 두 가지다. ①이 깨지면 낡은 값이 신선해 보이고, ②가 깨지면 신선한 값이 낡아 보인다.

#include <gtest/gtest.h>

#include "orin_firmware_bridge/core/rd_map.hpp"      // TARGET::*
#include "orin_firmware_bridge/core/rd_read_age.hpp"

using namespace orin_bridge;

namespace {

using Clock = ReadAgeMap::Clock;

// 테스트는 벽시계를 기다리지 않는다 — 시각을 **직접 만든다**. `sleep` 으로 나이를 만들면
// 테스트가 느려지고, 느린 CI 에서 경계값이 흔들려 간헐 실패가 된다.
Clock::time_point T0() { return Clock::time_point{} + std::chrono::hours(1); }
Clock::time_point At(double s) {
    return T0() + std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>(s));
}

}  // namespace

// ── 기본: 안 읽은 자리는 목록에 없다 ────────────────────────────────────────
// "미판독" 을 age=-1 이나 null 같은 특수값으로 표현하지 않는다 — 목록에 없다는 것이
// 곧 미판독이다. 특수값을 두면 소비자가 그것을 숫자로 읽는 경로가 반드시 생긴다.
TEST(ReadAge, UnreadBytesAreAbsentNotZeroAge) {
    ReadAgeMap m;
    EXPECT_FALSE(m.AnyRead(TARGET::ECU));
    EXPECT_TRUE(m.Snapshot(TARGET::ECU, At(1.0)).empty());
}

TEST(ReadAge, MarkedSpanComesBackWithItsAge) {
    ReadAgeMap m;
    m.Mark(TARGET::ECU, 16, 17, ReadOrigin::PRESET, T0());

    const auto s = m.Snapshot(TARGET::ECU, At(0.005));
    ASSERT_EQ(s.size(), 1u);
    EXPECT_EQ(s[0].addr, 16);
    EXPECT_EQ(s[0].len, 17);
    EXPECT_EQ(s[0].src, ReadOrigin::PRESET);
    EXPECT_NEAR(s[0].age_s, 0.005, 1e-6);
}

// 떨어진 두 세그는 **합쳐지지 않는다.** 합치면 사이의 미판독 구간이 읽힌 것으로 보인다.
TEST(ReadAge, DisjointSegsStaySeparate) {
    ReadAgeMap m;
    const auto now = T0();
    m.Mark(TARGET::ECU, 16, 17, ReadOrigin::PRESET, now);
    m.Mark(TARGET::ECU, 48, 80, ReadOrigin::PRESET, now);

    const auto s = m.Snapshot(TARGET::ECU, now);
    ASSERT_EQ(s.size(), 2u);
    EXPECT_EQ(s[0].addr, 16);  EXPECT_EQ(s[0].len, 17);
    EXPECT_EQ(s[1].addr, 48);  EXPECT_EQ(s[1].len, 80);
}

// 맞닿은 두 세그가 **같은 시각·같은 출처**면 하나로 합쳐진다 — 한 트랜잭션으로 읽힌
// 연속 구간을 두 칸으로 보고할 이유가 없다.
TEST(ReadAge, AdjacentSameStampMergesIntoOne) {
    ReadAgeMap m;
    const auto now = T0();
    m.Mark(TARGET::ECU, 0, 16, ReadOrigin::INIT, now);
    m.Mark(TARGET::ECU, 16, 17, ReadOrigin::INIT, now);

    const auto s = m.Snapshot(TARGET::ECU, now);
    ASSERT_EQ(s.size(), 1u);
    EXPECT_EQ(s[0].addr, 0);
    EXPECT_EQ(s[0].len, 33);
}

// ── 핵심 ① — 겹칠 때 나중 읽기가 이긴다 ────────────────────────────────────
// 이게 뒤집히면 기동 스냅샷(30초 전)이 주기 프리셋(5ms 전)을 덮어써서, 실시간으로
// 갱신되는 구간이 화면에 "30초 된 값" 으로 뜬다.
TEST(ReadAge, LaterReadWinsOnOverlap) {
    ReadAgeMap m;
    m.Mark(TARGET::ECU, 42, 151, ReadOrigin::INIT,   T0());          // 기동 전체읽기
    m.Mark(TARGET::ECU, 48, 80,  ReadOrigin::PRESET, At(30.0));      // 30초 뒤 주기 프리셋

    const auto s = m.Snapshot(TARGET::ECU, At(30.005));
    // 42:6 (init, 30초) / 48:80 (preset, 5ms) / 128:65 (init, 30초)
    ASSERT_EQ(s.size(), 3u);

    EXPECT_EQ(s[0].addr, 42);  EXPECT_EQ(s[0].len, 6);
    EXPECT_EQ(s[0].src, ReadOrigin::INIT);
    EXPECT_NEAR(s[0].age_s, 30.005, 1e-6);

    EXPECT_EQ(s[1].addr, 48);  EXPECT_EQ(s[1].len, 80);
    EXPECT_EQ(s[1].src, ReadOrigin::PRESET);
    EXPECT_NEAR(s[1].age_s, 0.005, 1e-6);

    EXPECT_EQ(s[2].addr, 128); EXPECT_EQ(s[2].len, 65);
    EXPECT_EQ(s[2].src, ReadOrigin::INIT);
    EXPECT_NEAR(s[2].age_s, 30.005, 1e-6);
}

// ── 핵심 ② — 같은 구간을 다시 읽으면 나이가 리셋된다 ───────────────────────
TEST(ReadAge, RereadingResetsAge) {
    ReadAgeMap m;
    m.Mark(TARGET::ECU, 16, 17, ReadOrigin::PRESET, T0());
    m.Mark(TARGET::ECU, 16, 17, ReadOrigin::PRESET, At(10.0));

    const auto s = m.Snapshot(TARGET::ECU, At(10.001));
    ASSERT_EQ(s.size(), 1u);
    EXPECT_NEAR(s[0].age_s, 0.001, 1e-6);
}

// 보드마다 독립이다. 공유하면 DPC 를 읽은 것이 ECU 를 신선하게 만든다.
TEST(ReadAge, TargetsAreIndependent) {
    ReadAgeMap m;
    m.Mark(TARGET::DPC, 46, 65, ReadOrigin::PRESET, T0());

    EXPECT_TRUE(m.AnyRead(TARGET::DPC));
    EXPECT_FALSE(m.AnyRead(TARGET::ECU));
    EXPECT_FALSE(m.AnyRead(TARGET::PCU));
    EXPECT_EQ(m.Snapshot(TARGET::DPC, T0()).size(), 1u);
    EXPECT_TRUE(m.Snapshot(TARGET::ECU, T0()).empty());
}

// 모르는 target_id 는 조용히 무시한다 (표시용 메타데이터라 여기서 죽일 이유가 없다).
TEST(ReadAge, UnknownTargetIsIgnored) {
    ReadAgeMap m;
    m.Mark(0x77, 0, 16, ReadOrigin::PRESET, T0());
    EXPECT_TRUE(m.Snapshot(0x77, T0()).empty());
}

// 256B 를 넘어가는 구간은 **잘린다.** 넘치면 옆 보드의 배열을 침범한다.
TEST(ReadAge, OutOfRangeIsClamped) {
    ReadAgeMap m;
    m.Mark(TARGET::ECU, 250, 100, ReadOrigin::SLOT, T0());

    const auto s = m.Snapshot(TARGET::ECU, T0());
    ASSERT_EQ(s.size(), 1u);
    EXPECT_EQ(s[0].addr, 250);
    EXPECT_EQ(s[0].len, 6);       // 250..255
    // 침범이 없었는지 — 옆 칸(DPC)이 여전히 백지여야 한다.
    EXPECT_FALSE(m.AnyRead(TARGET::DPC));
}

// NEVER 로는 도장을 못 찍는다 — "안 읽음" 을 기록하는 것은 의미가 없고, 그걸 허용하면
// 스냅샷에 age 를 가진 미판독 구간이 생긴다.
TEST(ReadAge, MarkingWithNeverIsANoOp) {
    ReadAgeMap m;
    m.Mark(TARGET::ECU, 0, 16, ReadOrigin::NEVER, T0());
    EXPECT_FALSE(m.AnyRead(TARGET::ECU));
}

// 출처가 다르면 시각이 같아도 안 합친다 — 화면이 "왜 낡았는가" 를 설명하는 근거다.
TEST(ReadAge, DifferentSourcesDoNotMerge) {
    ReadAgeMap m;
    const auto now = T0();
    m.Mark(TARGET::ECU, 0, 16, ReadOrigin::INIT, now);
    m.Mark(TARGET::ECU, 16, 17, ReadOrigin::SLOT, now);

    const auto s = m.Snapshot(TARGET::ECU, now);
    ASSERT_EQ(s.size(), 2u);
    EXPECT_EQ(s[0].src, ReadOrigin::INIT);
    EXPECT_EQ(s[1].src, ReadOrigin::SLOT);
}

// ── JSON 스키마 ─────────────────────────────────────────────────────────────
// 키 이름·타입·소수 자릿수를 고정한다. 종전 `fresh` 는 서비스 콜백 한가운데서 조립돼
// 어떤 테스트도 닿지 못했고, 그래서 형식이 바뀌어도 웹을 띄우기 전엔 아무도 몰랐다.
TEST(ReadAgeJson, SchemaIsFixed) {
    ReadAgeMap m;
    m.Mark(TARGET::ECU, 16, 17, ReadOrigin::PRESET, T0());
    const auto js = ReadAgeSpansToJson(m.Snapshot(TARGET::ECU, At(0.005)));
    EXPECT_EQ(js, "[{\"addr\":16,\"len\":17,\"age_s\":0.005,\"src\":\"preset\"}]");
}

TEST(ReadAgeJson, EmptySnapshotIsEmptyArray) {
    EXPECT_EQ(ReadAgeSpansToJson({}), "[]");
}

TEST(ReadAgeJson, MultipleSpansAreCommaSeparated) {
    ReadAgeMap m;
    m.Mark(TARGET::ECU, 0, 16, ReadOrigin::INIT, T0());
    m.Mark(TARGET::ECU, 48, 80, ReadOrigin::PRESET, At(30.0));
    const auto js = ReadAgeSpansToJson(m.Snapshot(TARGET::ECU, At(30.0)));
    EXPECT_EQ(js,
        "[{\"addr\":0,\"len\":16,\"age_s\":30.000,\"src\":\"init\"},"
         "{\"addr\":48,\"len\":80,\"age_s\":0.000,\"src\":\"preset\"}]");
}

// 로케일이 소수점을 `,` 로 찍으면 JSON 이 깨진다 (rd_status.cpp 가 같은 처방을 쓴다).
// 로케일을 실제로 바꿀 수는 없는 환경이 있으므로, **`,` 가 소수점 자리에 없다**는
// 형태로 고정한다.
TEST(ReadAgeJson, DecimalPointIsADotNotALocaleComma) {
    ReadAgeMap m;
    m.Mark(TARGET::ECU, 0, 4, ReadOrigin::OOS, T0());
    const auto js = ReadAgeSpansToJson(m.Snapshot(TARGET::ECU, At(1.5)));
    EXPECT_NE(js.find("\"age_s\":1.500"), std::string::npos) << js;
    EXPECT_EQ(js.find("1,500"), std::string::npos) << js;
}
