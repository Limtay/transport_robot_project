// jeongae 시퀀스의 tick 주기 ↔ 대기 상한 (U12, 2026-08-06)
//
// ## 무엇이 틀어졌었나
//
// `kWaitTicksMax = 150` 옆에 *"5Hz tick 기준, 기본 30초"* 라고 적혀 있었다. **둘 다 틀렸다.**
// `TickAutoSequence()` 를 부르는 곳(`rd_schedule.cpp` 의 project/manual 분기)에는 레이트
// 분주가 없어서 **200Hz 루프의 매 tick 마다** 불린다. 실효 상한이 150 × 5ms ≈ 0.75초였고,
// 전개를 걸면 `DPC_WAIT_CAMERA` 에서 곧바로 Abort 했다.
//
// **값만 보는 테스트로는 절대 못 잡는다.** `wait_ticks > max` 는 어느 주기에서든 똑같이
// 참이 되고, 로그에는 "대기 초과" 만 남지 얼마 만에 걸렸는지는 안 남는다. 실제로 이 결함은
// U12 가 `wait_ticks` 를 `GET_STATUS` 로 꺼내 화면에 숫자를 띄우고 나서야 보였다.
//
// ## 그래서 소스를 읽는다
//
// 지켜야 하는 것은 **"상수가 가정한 주기"와 "실제 호출 주기"가 같다** 는 것이다. 후자는
// 코드의 형태(분주가 있는가)로만 확인할 수 있다. `test_counter_producers`·`test_no_dead_code`
// 가 같은 이유로 같은 방법을 쓴다.

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <regex>
#include <sstream>
#include <string>

#include "orin_firmware_bridge/policy/rd_sequence.hpp"

namespace fs = std::filesystem;
using orin_bridge::RdSequence;

namespace {

std::string Slurp(const fs::path& p) {
    std::ifstream f(p);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

std::string Schedule() {
    return Slurp(fs::path(RD_SRC_DIR) / "src" / "sched" / "rd_schedule.cpp");
}

}  // namespace

// 상한이 **초 단위로 표현**돼 있는가. 맨 숫자로 돌아가면 같은 사고가 반복된다 —
// 읽는 사람이 "이게 몇 초지" 를 매번 tick 주기로 나눠 봐야 하고, 그 나눗셈을 한 번
// 틀린 것이 이 결함이었다.
TEST(SeqTickRate, TimeoutIsExpressedInSecondsTimesTickRate) {
    EXPECT_EQ(RdSequence::kWaitTicksMax, RdSequence::kTickHz * RdSequence::kWaitTimeoutS);
    // 60초 = 실제 전개 ≈40초(사용자 실측) + 여유 20초.
    // ⚠ 이 상한은 **국면당**이다 — `DPC_WAIT_CAMERA` 하나가 DPC 의 2→3→4→5 전 구간을
    //   덮는다. "한 상태당" 으로 읽고 10초를 넣었다가 실제 전개가 무조건 실패하는
    //   값이 됐었다 (rd_sequence.hpp 주석 참조).
    EXPECT_EQ(RdSequence::kWaitTimeoutS, 60u) << "실제 전개 ≈40초 + 여유";
    EXPECT_EQ(RdSequence::kTickHz, 200u) << "스케줄 루프는 5ms 주기다";
    EXPECT_EQ(RdSequence::kWaitTicksMax, 12000u);
}

// **핵심** — 호출부에 분주가 없어야 `kTickHz = 200` 이 맞다.
//
// 누군가 `if (tick_count_ % 40 == 0) command_->TickAutoSequence();` 로 바꾸면 상한이
// 조용히 40배가 된다(10초 → 400초). 컴파일도 되고 테스트도 통과하며, 증상은 "전개가
// 영원히 안 끝난다" 로 나타난다. 그래서 형태를 고정한다.
TEST(SeqTickRate, TickAutoSequenceHasNoRateDividerAtTheCallSite) {
    const std::string src = Schedule();
    const size_t at = src.find("TickAutoSequence()");
    ASSERT_NE(at, std::string::npos) << "호출부를 못 찾았다 — 이름이 바뀌었으면 이 테스트도 갱신할 것";
    EXPECT_EQ(src.find("TickAutoSequence()", at + 1), std::string::npos)
        << "호출부가 둘 이상이다 — 주기가 두 배가 된다";

    // 같은 줄, 그리고 바로 앞 줄에 조건이 붙어 있지 않은가.
    const size_t line_begin = src.rfind('\n', at) + 1;
    const std::string line = src.substr(line_begin, src.find('\n', at) - line_begin);
    EXPECT_EQ(line.find("if"), std::string::npos)
        << "호출이 조건문과 같은 줄에 있다 — 분주일 수 있다:\n" << line;
    EXPECT_EQ(line.find('%'), std::string::npos)
        << "호출 줄에 나머지 연산이 있다 — 분주다:\n" << line;

    const size_t prev_begin = src.rfind('\n', line_begin - 2) + 1;
    const std::string prev = src.substr(prev_begin, line_begin - prev_begin);
    const bool prev_is_guard =
        std::regex_search(prev, std::regex(R"(if\s*\(.*(%|tick_count_).*\))"));
    EXPECT_FALSE(prev_is_guard)
        << "바로 앞 줄이 분주 조건으로 보인다 — kTickHz 가 더 이상 200 이 아니다:\n" << prev;
}

// 스케줄 루프가 정말 5ms 인가 (kTickHz=200 의 근거).
TEST(SeqTickRate, ScheduleLoopPeriodIsFiveMilliseconds) {
    const std::string src = Schedule();
    EXPECT_NE(src.find("period        = 5ms"), std::string::npos)
        << "루프 주기가 5ms 가 아니면 kTickHz 를 같이 고쳐야 한다";
}

// 낡은 "5Hz" 설명이 시퀀스 쪽에 되살아나지 않는가. 이 오해가 결함의 원인이었다.
TEST(SeqTickRate, TheStaleFiveHertzClaimIsGone) {
    for (const char* rel : {"include/orin_firmware_bridge/policy/rd_sequence.hpp",
                            "src/policy/rd_sequence.cpp"}) {
        const std::string src = Slurp(fs::path(RD_SRC_DIR) / rel);
        // "5Hz" 를 **주기 설명으로** 쓴 자리만 문제다. 결함 이력을 적어 둔 문장에는
        // 그 낱말이 들어갈 수밖에 없으므로, 호출 주기를 단언하는 형태만 막는다.
        EXPECT_EQ(src.find("5Hz 로 스케줄러가 호출"), std::string::npos)
            << rel << " 에 낡은 5Hz 주장이 되살아났다";
        EXPECT_EQ(src.find("5Hz tick 기준"), std::string::npos) << rel;
    }
}

// 2026-08-07 — cmd_vel 0 수렴 스킵의 **기본값과 상한**. 안전 파라미터라 소스로 고정한다.
//
// 경사에서 정지 유지 중 쓰기가 끊기면 ECU 가 100ms(`AUTO_TIMEOUT`) 뒤 명령을 무효화한다.
// 그래서 ① 스킵은 **기본 off** 이고 ② 켜더라도 상한이 3초처럼 짧으면 안 된다.
TEST(SeqTickRate, CmdVelZeroSkipIsOffByDefaultWithALongTimeout) {
    const std::string cfg = Slurp(fs::path(RD_SRC_DIR) /
                                  "include" / "orin_firmware_bridge" / "rd_config.hpp");
    EXPECT_NE(cfg.find("cmd_vel_zero_skip_default = false"), std::string::npos)
        << "0 수렴 스킵의 기본값이 false 가 아니다 — 경사에서 위험한 쪽이 기본이 된다";
    EXPECT_NE(cfg.find("cmd_vel_zero_timeout  = 30.0"), std::string::npos)
        << "0 수렴 상한이 30초가 아니다 (구 3초는 조작 중 정적 구간에서 쉽게 걸린다)";
    EXPECT_EQ(cfg.find("cmd_vel_zero_timeout  = 3.0"), std::string::npos)
        << "구 3초 값이 남아 있다";
}
