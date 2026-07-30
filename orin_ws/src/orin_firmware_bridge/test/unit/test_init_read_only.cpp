// 읽기 전용 control(`auto_mode: none`)도 **조작 입구를 연다** (2026-07-30 실기)
//
// ## 무엇이 있었나
//
// `rd_schedule.cpp` 의 `read_only_control` 분기가 `MarkInitDone()` 도 `on_init_done_()` 도
// 부르지 않았다. ECU 에 아무것도 쓰지 않으므로 검증할 write 가 없는 것은 맞지만,
// **FSM 이 INIT 에 머물면 `AcceptsConfig` 가 조작을 전부 거부한다.**
//
// `./run.sh traction` 이 정확히 이 구성이고, 그래서 견인 실험 내내:
//   · `cli status` 가 아예 안 됐다 (config 서비스가 만들어지지 않는다)
//   · 웹 Tab3(레지스터 맵)이 죽어 있었다 (OP_GET_REGISTERS 가 그 서비스를 탄다)
//   · bag 의 모든 메시지가 `control_state=INIT`, `auto_mode=CURRENT` 로 기록됐다.
//     **쓰지 않는 런이 "전류 명령을 쓴 런" 으로 남았다.**
//
// 바로 아래 `manual` 분기에는 같은 문제를 인지한 주석이 있다 — *"INIT 에 머물면 config
// service 가 통째로 막힌다"*. **manual 만 고쳐졌고 read-only 는 빠져 있었다.**
//
// ## 왜 이 테스트가 소스를 읽는가
//
// 이 분기는 **시리얼 포트가 열려야** 도달한다 (`Initialize()` → `RunLoop()`). 노드만 띄워서는
// 재현되지 않고, 실기에서만 드러났다. 그래서 실기가 없어도 도는 형태로 고정한다 —
// 분기 안에 필요한 호출이 실제로 적혀 있는가.
//
// 값을 보는 테스트로 대신할 수 없는 것도 같은 이유다: `control_state` 를 확인하려면
// 그 코드 경로를 돌려야 하고, 그러려면 ECU 가 있어야 한다.

#include <gtest/gtest.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>

namespace {

std::string Src(const char* rel) {
    std::ifstream f(std::filesystem::path(RD_SRC_DIR) / rel);
    EXPECT_TRUE(f.good()) << "파일을 못 읽었다: " << rel;
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

std::string StripLineComments(const std::string& src) {
    std::string out;
    out.reserve(src.size());
    bool in_comment = false;
    for (size_t i = 0; i < src.size(); i++) {
        if (!in_comment && src[i] == '/' && i + 1 < src.size() && src[i + 1] == '/') in_comment = true;
        if (src[i] == '\n') in_comment = false;
        if (!in_comment) out += src[i];
    }
    return out;
}

// `if (read_only_control) {` 블록의 본문 (다음 `} else if` 직전까지).
std::string ReadOnlyBranch() {
    const auto src = StripLineComments(Src("src/sched/rd_schedule.cpp"));
    const std::string head = "if (read_only_control) {";
    const size_t b = src.find(head);
    EXPECT_NE(b, std::string::npos) << "read_only_control 분기를 못 찾았다 — 개명됐다면 이 테스트도 고칠 것";
    if (b == std::string::npos) return "";
    const size_t e = src.find("} else if", b);
    return src.substr(b, (e == std::string::npos ? src.size() : e) - b);
}

}  // namespace

// ★ 핵심 — FSM 을 IDLE 로 보내고 L3 에 알린다.
TEST(InitReadOnly, ReadOnlyControlLeavesInitAndOpensOperatorEntry) {
    const auto branch = ReadOnlyBranch();

    EXPECT_NE(branch.find("MarkInitDone"), std::string::npos)
        << "읽기 전용 control 이 FSM 을 INIT 에 두고 있다.\n"
        << "  INIT 이면 AcceptsConfig 가 전부 거부해 **조작 입구가 통째로 막힌다** —\n"
        << "  cli status 불가, 웹 Tab3 사망, 프리셋 교체 불가.\n"
        << "  (manual 분기가 같은 이유로 이미 MarkInitDone 을 부른다.)";

    EXPECT_NE(branch.find("on_init_done_"), std::string::npos)
        << "읽기 전용 control 이 INIT 완료를 L3 에 알리지 않는다 — config 서비스가 안 열린다.";
}

// 재생 서버는 **열지 않는다.** write 가 없는데 goal 을 수락하면
// "재생됐다는데 아무것도 안 움직인다" 가 되고, 그건 거부보다 나쁘다.
TEST(InitReadOnly, ReadOnlyControlDoesNotOpenTheProfileServer) {
    EXPECT_NE(ReadOnlyBranch().find("on_init_done_(false)"), std::string::npos)
        << "읽기 전용인데 with_profiles=true 로 알리고 있다 — 재생 서버가 열린다.\n"
        << "  RdControlApi 에는 읽기 전용 거부 로직이 없으므로 goal 이 그대로 수락된다.";
}

// auto_mode 를 none 으로 확정해야 한다 — 이 값이 피드백과 result.json 에 남는다.
TEST(InitReadOnly, ReadOnlyControlRecordsAutoModeAsNone) {
    EXPECT_NE(ReadOnlyBranch().find("SetAutoMode(AUTO_MODE_NONE)"), std::string::npos)
        << "읽기 전용 런이 기본값(CURRENT)을 그대로 발행한다.\n"
        << "  **쓰지 않은 런이 '전류 명령을 쓴 런' 으로 기록된다** — 분석이 그것을 구분할 수 없다.";
}

// 활성 프리셋도 알려야 한다. 기본값에 기대면 틀린 채로 조용히 돈다
// (control 분기가 같은 이유로 SetReadPreset 을 명시 호출한다 — 2026-07-29 실기).
TEST(InitReadOnly, ReadOnlyControlAnnouncesItsReadPreset) {
    EXPECT_NE(ReadOnlyBranch().find("SetReadPreset"), std::string::npos)
        << "활성 프리셋을 텔레메트리에 안 알린다 — Reads() 가 기본 프리셋으로 판정해\n"
        << "  안 읽는 구간을 신선한 값처럼 발행할 수 있다.";
}

// 두 서버가 **따로** 열리는가 — 하나로 묶여 있으면 read-only 는 둘 다 잃는다.
TEST(InitReadOnly, ConfigServiceAndProfileServerAreSeparatelyOpenable) {
    const auto hdr = Src("include/orin_firmware_bridge/ros/rd_control_api.hpp");
    EXPECT_NE(hdr.find("void StartConfigService()"), std::string::npos)
        << "config 서비스를 단독으로 열 방법이 없다 — 종전처럼 하나로 묶여 있으면\n"
        << "  '재생은 못 하지만 상태는 봐야 하는' 구성(견인 실험)이 표현 불가능하다.";

    // 조작 입구 없이 재생 서버만 열리는 조합은 만들지 않는다.
    const auto src = StripLineComments(Src("src/ros/rd_control_api.cpp"));
    const size_t p = src.find("void RdControlApi::StartProfileServer()");
    ASSERT_NE(p, std::string::npos);
    EXPECT_NE(src.find("StartConfigService", p), std::string::npos)
        << "StartProfileServer 가 config 서비스를 보장하지 않는다 — "
           "재생 중 상태를 못 읽는 구성이 생긴다.";
}
