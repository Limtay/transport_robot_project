// 쓰기 결과 반영 — 리드백을 뺀 자리 (redesign/09 §1.2 ③, U2)
//
// U1 에서 control 프리셋의 read-back 20B 를 뺐다. 그전까지 섀도는 "ECU 가 가진 값"
// 이었지만 지금은 "브리지가 보내려는 값" 이다. 거부된 쓰기까지 섀도에 남으면
// `ControlFeedback.cmd` 가 **ECU 가 받지도 않은 값**을 신선한 값처럼 발행한다.
//
// ⚠ 계획서(09 §1.2 ③)의 "보내기 직전 스냅샷을 되돌린다" 는 **성립하지 않는다** —
//   그 스냅샷은 이미 새 의도값이라 되돌려도 no-op 다. 되돌릴 값은 **직전에 수락된 값**
//   이고 그것을 들고 있는 것이 `WriteEcho` 다. 아래 EchoIsNotTheSentSnapshot 가 그 차이를
//   테스트로 고정한다 — 다음에 누가 계획서만 보고 "스냅샷이면 되잖아" 로 되돌리는 것을 막는다.

#include <gtest/gtest.h>

#include <cstring>

#include "orin_firmware_bridge/sched/rd_write_echo.hpp"

namespace {

using orin_bridge::ClassifyWrite;
using orin_bridge::WriteEcho;
using orin_bridge::WriteOutcome;

// ── ClassifyWrite — 조합 전수 ───────────────────────────────────────────────

TEST(WriteEchoClassify, NonWriteInstIsAlwaysNone) {
    // READ/PING 은 쓰기가 없다. err 가 있어도 쓰기 상태가 아니다.
    EXPECT_EQ(ClassifyWrite(false, false, true,  true,  0x00), WriteOutcome::NONE);
    EXPECT_EQ(ClassifyWrite(false, false, true,  true,  0x77), WriteOutcome::NONE);
    EXPECT_EQ(ClassifyWrite(false, true,  false, false, 0x30), WriteOutcome::NONE);
}

TEST(WriteEchoClassify, NoResponseIsUnknownNotRejected) {
    // ★ 이 구분이 이 파일의 핵심이다. 무응답은 "보드가 못 받았다" 가 아니라
    //    **"받았는지 모른다"** 이다 — 요청은 나갔고 응답만 유실됐을 수 있다.
    //    UNKNOWN 을 REJECTED 로 다루면 실제로 적용된 값을 섀도에서 지운다.
    EXPECT_EQ(ClassifyWrite(true, false, /*comm_ok*/false, /*resp_ok*/false, 0x00),
              WriteOutcome::UNKNOWN);
    EXPECT_EQ(ClassifyWrite(true, true,  false, false, 0x00), WriteOutcome::UNKNOWN);
}

TEST(WriteEchoClassify, MalformedResponseIsUnknown) {
    // ID/Inst 가 안 맞으면 err 바이트를 믿을 근거가 없다 — 0 이든 아니든 UNKNOWN.
    EXPECT_EQ(ClassifyWrite(true, false, /*comm_ok*/true, /*resp_ok*/false, 0x00),
              WriteOutcome::UNKNOWN);
    EXPECT_EQ(ClassifyWrite(true, false, true, false, 0x07), WriteOutcome::UNKNOWN);
}

TEST(WriteEchoClassify, PlainWriteUsesWholeErrByte) {
    EXPECT_EQ(ClassifyWrite(true, /*is_rw*/false, true, true, 0x00), WriteOutcome::ACCEPTED);
    EXPECT_EQ(ClassifyWrite(true, false, true, true, 0x07), WriteOutcome::REJECTED);
    // WRITE 는 니블 분리가 아니다 — 상위 니블만 세워도 거부다.
    EXPECT_EQ(ClassifyWrite(true, false, true, true, 0x30), WriteOutcome::REJECTED);
}

TEST(WriteEchoClassify, RwSplitsNibblesAndOnlyLooksAtWriteHalf) {
    // RW 응답 err = read_err | write_err<<4 (rd_map.cpp DecodeNode).
    EXPECT_EQ(ClassifyWrite(true, /*is_rw*/true, true, true, 0x00), WriteOutcome::ACCEPTED);
    // ★ 읽기만 실패 — **쓰기는 수락됐다.** 여기서 REJECTED 로 보면 멀쩡히 적용된 값을
    //   되돌려 섀도가 ECU 와 갈라진다.
    EXPECT_EQ(ClassifyWrite(true, true, true, true, 0x07), WriteOutcome::ACCEPTED);
    // 쓰기만 실패 — 읽기는 정상.
    EXPECT_EQ(ClassifyWrite(true, true, true, true, 0x30), WriteOutcome::REJECTED);
    // 둘 다 실패.
    EXPECT_EQ(ClassifyWrite(true, true, true, true, 0x37), WriteOutcome::REJECTED);
}

// ── WriteEcho — 되돌림 ──────────────────────────────────────────────────────

TEST(WriteEchoRollback, DoesNothingBeforeAnyAcceptedWrite) {
    // ★ 기동 직후. 거울은 0 으로 초기화돼 있지만 그 0 은 **관측된 적 없는 값**이다.
    //   0 을 섀도에 쓰면 "ECU 가 0 을 갖고 있다" 는 거짓말을 새로 만든다.
    WriteEcho echo;
    uint8_t shadow[256];
    std::memset(shadow, 0xAB, sizeof(shadow));

    EXPECT_EQ(echo.Rollback(164, 16, shadow, sizeof(shadow)), 0u);
    for (int i = 164; i < 180; i++)
        EXPECT_EQ(shadow[i], 0xAB) << "addr " << i << " — 수락 이력이 없는데 건드렸다";
}

TEST(WriteEchoRollback, RestoresLastAcceptedNotTheSentValue) {
    WriteEcho echo;
    uint8_t shadow[256] = {};

    // ① 5.0A 를 보내 수락됨
    const uint8_t accepted[4] = {0x11, 0x22, 0x33, 0x44};
    std::memcpy(&shadow[164], accepted, 4);
    echo.NoteAccepted(164, accepted, 4);

    // ② 새 의도값 9.0A 가 섀도에 들어가고 전송됨 → 거부됨
    const uint8_t attempted[4] = {0x99, 0x88, 0x77, 0x66};
    std::memcpy(&shadow[164], attempted, 4);

    EXPECT_EQ(echo.Rollback(164, 4, shadow, sizeof(shadow)), 4u);
    EXPECT_EQ(std::memcmp(&shadow[164], accepted, 4), 0)
        << "직전 수락값(①)으로 돌아가야 한다 — 전송값(②)으로 되돌리면 no-op 이다";
}

// ★★ 계획서의 메커니즘이 왜 안 되는지를 코드로 박제한다.
TEST(WriteEchoRollback, EchoIsNotTheSentSnapshot) {
    uint8_t shadow[256] = {};
    const uint8_t prev[2] = {0x11, 0x22};
    std::memcpy(&shadow[190], prev, 2);

    // "보내기 직전 스냅샷" — Encode 시점의 섀도는 **이미 새 값**이다.
    const uint8_t next[2] = {0xEE, 0xFF};
    std::memcpy(&shadow[190], next, 2);
    uint8_t snapshot_at_send[2];
    std::memcpy(snapshot_at_send, &shadow[190], 2);

    // 거부됐다 치고 그 스냅샷으로 되돌리면…
    std::memcpy(&shadow[190], snapshot_at_send, 2);
    EXPECT_EQ(std::memcmp(&shadow[190], next, 2), 0)
        << "전송 시점 스냅샷으로 되돌리는 것은 no-op 이다 — 09 §1.2 ③ 의 원안이 틀린 이유";
    EXPECT_NE(std::memcmp(&shadow[190], prev, 2), 0);
}

TEST(WriteEchoRollback, OnlyTouchesBytesWithAcceptHistory) {
    // 구간의 일부만 수락 이력이 있는 경우 — 나머지는 손대지 않는다.
    WriteEcho echo;
    uint8_t shadow[256];
    std::memset(shadow, 0x5A, sizeof(shadow));

    const uint8_t two[2] = {0x01, 0x02};
    echo.NoteAccepted(128, two, 2);          // 128,129 만 이력이 있다
    std::memset(&shadow[128], 0xFF, 4);      // 128~131 을 새 값으로 덮음

    EXPECT_EQ(echo.Rollback(128, 4, shadow, sizeof(shadow)), 2u);
    EXPECT_EQ(shadow[128], 0x01);
    EXPECT_EQ(shadow[129], 0x02);
    EXPECT_EQ(shadow[130], 0xFF) << "이력 없는 바이트를 건드렸다";
    EXPECT_EQ(shadow[131], 0xFF) << "이력 없는 바이트를 건드렸다";
}

TEST(WriteEchoRollback, ReturnsZeroWhenShadowAlreadyMatches) {
    // 되돌릴 것이 없으면 0 — 로그가 "0B 되돌림" 을 찍어 무의미한 경고를 내지 않게.
    WriteEcho echo;
    uint8_t shadow[256] = {};
    const uint8_t v[3] = {7, 8, 9};
    std::memcpy(&shadow[180], v, 3);
    echo.NoteAccepted(180, v, 3);

    EXPECT_EQ(echo.Rollback(180, 3, shadow, sizeof(shadow)), 0u);
    EXPECT_EQ(std::memcmp(&shadow[180], v, 3), 0);
}

TEST(WriteEchoRollback, LaterAcceptOverwritesTheEcho) {
    WriteEcho echo;
    uint8_t shadow[256] = {};
    const uint8_t a[2] = {0xA0, 0xA1};
    const uint8_t b[2] = {0xB0, 0xB1};
    echo.NoteAccepted(200, a, 2);
    echo.NoteAccepted(200, b, 2);            // 두 번째 수락이 기준점이 된다

    std::memset(&shadow[200], 0xCC, 2);
    EXPECT_EQ(echo.Rollback(200, 2, shadow, sizeof(shadow)), 2u);
    EXPECT_EQ(std::memcmp(&shadow[200], b, 2), 0) << "가장 최근 수락값이 기준이다";
}

TEST(WriteEchoRollback, ClampsAtRegisterEnd) {
    // 경계 밖으로 나가면 잘라낸다 — 섀도 밖을 쓰면 인접 멤버를 깨뜨린다.
    WriteEcho echo;
    uint8_t shadow[256];
    std::memset(shadow, 0x11, sizeof(shadow));
    const uint8_t v[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    echo.NoteAccepted(252, v, 8);            // 252~255 만 들어간다
    std::memset(&shadow[252], 0xFF, 4);

    EXPECT_EQ(echo.Rollback(252, 8, shadow, sizeof(shadow)), 4u);
    for (int i = 0; i < 4; i++) EXPECT_EQ(shadow[252 + i], v[i]);
}

// 노드가 섞이지 않는가 — 브리지는 ECU/DPC/PCU 각각에 거울을 하나씩 둔다.
// 한 인스턴스가 다른 노드의 주소를 오염시키면 DPC 거부가 ECU 섀도를 되돌린다.
TEST(WriteEchoRollback, InstancesAreIndependent) {
    WriteEcho ecu, dpc;
    uint8_t sh_ecu[256] = {}, sh_dpc[256] = {};
    const uint8_t e[1] = {0xE1};
    ecu.NoteAccepted(164, e, 1);

    EXPECT_TRUE(ecu.IsAccepted(164));
    EXPECT_FALSE(dpc.IsAccepted(164)) << "다른 노드의 이력이 새어 들어왔다";

    sh_dpc[164] = 0x77;
    EXPECT_EQ(dpc.Rollback(164, 1, sh_dpc, sizeof(sh_dpc)), 0u);
    EXPECT_EQ(sh_dpc[164], 0x77);
    (void)sh_ecu;
}

}  // namespace
