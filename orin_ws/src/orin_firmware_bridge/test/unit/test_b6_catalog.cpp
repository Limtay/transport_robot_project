// B6 — 의미 단위 명령 카탈로그 + 게이트 (redesign/00 B6, 03 §7, 07 §2)
//
// **이 테스트가 지키는 것은 하나다: 어떤 조합이 허용되고 어떤 조합이 막히는가.**
// 그 판정이 카탈로그의 순수 함수 하나에 모여 있어야 호출부마다 갈라지지 않는다.
//
// 회귀 관점: 이 파일 이전에는 `CommandSet` 이 raw 주소만 받았고 **모든 명령이 manual
// 전용**이었다 — control 에서는 레지스터를 읽을 길이 아예 없었다. 아래 READ 허용 검사는
// 그 상태에 대고 돌리면 전부 실패한다.

#include <gtest/gtest.h>

#include "orin_firmware_bridge/policy/rd_command_catalog.hpp"

namespace {

namespace cc = orin_bridge::cmdcat;
namespace ecu = orin_bridge::ecu;

constexpr bool kManual = true, kControl = false;
constexpr bool kStopped = true, kMoving = false;

// ── 게이트 ────────────────────────────────────────────────────────────────
// ★ 핵심 — **읽기는 아무것도 바꾸지 않으므로 모드를 가리지 않는다.**
TEST(B6Gate, ReadsAreAllowedInEveryModeWithoutSafeStop) {
    for (uint8_t cmd : {cc::CMD_READ_SYS, cc::CMD_READ_MOTOR, cc::CMD_READ_SENSOR,
                        cc::CMD_READ_DIAG, cc::CMD_READ_ALL}) {
        EXPECT_TRUE(cc::Gate(cmd, kControl, kMoving, 0).ok)
            << cc::CmdName(cmd) << " 가 control/이동중에 막혔다 — 읽기는 막을 이유가 없다";
        EXPECT_TRUE(cc::Gate(cmd, kManual, kMoving, 0).ok) << cc::CmdName(cmd);
    }
}

TEST(B6Gate, RawIsManualOnly) {
    EXPECT_FALSE(cc::Gate(cc::CMD_RAW_READ,  kControl, kStopped, 0).ok);
    EXPECT_FALSE(cc::Gate(cc::CMD_RAW_WRITE, kControl, kStopped, 0).ok);
    EXPECT_TRUE (cc::Gate(cc::CMD_RAW_READ,  kManual,  kStopped, 0).ok);
    EXPECT_TRUE (cc::Gate(cc::CMD_RAW_WRITE, kManual,  kStopped, 0).ok);

    // 거부 사유가 **무엇을 해야 하는지** 알려 주는가 — "거부됨" 만으로는 다음 수가 없다.
    const auto g = cc::Gate(cc::CMD_RAW_READ, kControl, kStopped, 0);
    ASSERT_NE(g.why, nullptr);
    EXPECT_NE(std::string(g.why).find("manual"), std::string::npos);
}

TEST(B6Gate, WritesNeedSafeStop) {
    EXPECT_FALSE(cc::Gate(cc::CMD_SET_USE_LPF, kManual, kMoving,  1).ok);
    EXPECT_TRUE (cc::Gate(cc::CMD_SET_USE_LPF, kManual, kStopped, 1).ok);
    EXPECT_FALSE(cc::Gate(cc::CMD_REBOOT,      kManual, kMoving,  0).ok);
    EXPECT_TRUE (cc::Gate(cc::CMD_REBOOT,      kManual, kStopped, 0).ok);
    // raw_write 는 두 조건을 **모두** 넘어야 한다.
    EXPECT_FALSE(cc::Gate(cc::CMD_RAW_WRITE, kManual,  kMoving,  0).ok);
    EXPECT_FALSE(cc::Gate(cc::CMD_RAW_WRITE, kControl, kStopped, 0).ok);
}

// ★ 멈추라는 명령을 "안 멈춰 있어서" 거부하면 게이트의 목적과 정반대다.
TEST(B6Gate, SoftEstopEngageIsAlwaysAllowedButReleaseIsNot) {
    EXPECT_TRUE(cc::Gate(cc::CMD_SET_SOFT_ESTOP, kControl, kMoving,
                         ecu::SOFT_ESTOP_ACTIVE).ok)
        << "ESTOP 작동은 감속 방향이라 이동 중에도 허용해야 한다";
    EXPECT_FALSE(cc::Gate(cc::CMD_SET_SOFT_ESTOP, kControl, kMoving,
                          ecu::SOFT_ESTOP_RELEASE).ok)
        << "ESTOP 해제는 가속 방향이다 — safe_stop 이 필요하다";
    EXPECT_TRUE(cc::Gate(cc::CMD_SET_SOFT_ESTOP, kControl, kStopped,
                         ecu::SOFT_ESTOP_RELEASE).ok);
}

TEST(B6Gate, UnknownCommandIsRejected) {
    for (uint8_t cmd : {uint8_t(5), uint8_t(12), uint8_t(99), uint8_t(255)}) {
        EXPECT_FALSE(cc::Gate(cmd, kManual, kStopped, 0).ok) << int(cmd);
    }
}

// 03 §7.3 에는 있었으나 **의도적으로 두지 않은** 것들. ControlConfig 가 소유한다.
// 여기 되살아나면 auto_mode 를 바꾸는 길이 둘이 되고 게이트가 갈라진다.
TEST(B6Catalog, ConfigOwnedCommandsAreAbsent) {
    for (uint8_t cmd : {uint8_t(10), uint8_t(11), uint8_t(12), uint8_t(15)}) {
        EXPECT_EQ(cc::Find(cmd), nullptr)
            << "cmd " << int(cmd) << " 가 카탈로그에 생겼다 — ControlConfig 와 이중 경로다";
    }
}

// ── 구간 ──────────────────────────────────────────────────────────────────
// 예산은 static_assert 가 이미 봤다. 여기서는 **무엇을 덮는지**를 고정한다.
TEST(B6Catalog, ReadSpansCoverWhatTheirNameClaims) {
    EXPECT_TRUE(cc::spans::kSys.Covers(ecu::REG_SYS_OFFSET, ecu::REG_SYS_SIZE));
    EXPECT_TRUE(cc::spans::kMotor.Covers(ecu::REG_MOTOR_DATA_OFFSET, ecu::REG_MOTOR_DATA_SIZE));
    EXPECT_TRUE(cc::spans::kSensor.Covers(ecu::REG_LOADCELL_OFFSET, ecu::REG_LOADCELL_SIZE));
    EXPECT_TRUE(cc::spans::kSensor.Covers(ecu::REG_IMU_OFFSET, ecu::REG_IMU_SIZE));
    EXPECT_TRUE(cc::spans::kSensor.Covers(ecu::REG_ENCODER_OFFSET, ecu::REG_ENCODER_SIZE));
    EXPECT_TRUE(cc::spans::kDiag.Covers(ecu::REG_DIAG_OFFSET, ecu::REG_DIAG_SIZE));
    EXPECT_TRUE(cc::spans::kDiag.Covers(ecu::REG_SYS_OFFSET, ecu::REG_SYS_SIZE));
}

// ★ read_all 이 한 트랜잭션에 들어가는 것이 Tab3 테이블의 전제다.
TEST(B6Catalog, ReadAllFitsOneTransactionAndSkipsReserved) {
    EXPECT_EQ(cc::spans::kAll.RespPayload(), 216);
    EXPECT_LE(cc::spans::kAll.RespPayload(), orin_bridge::kMaxRespPayload);
    EXPECT_LE(cc::spans::kAll.count, orin_bridge::kMaxReadSegs);

    // 예약 구간은 빠진다 — 그것이 216B 로 줄어든 이유다.
    EXPECT_FALSE(cc::spans::kAll.Covers(ecu::REG_RSVD0_OFFSET, 1));
    EXPECT_FALSE(cc::spans::kAll.Covers(ecu::REG_RSVD1_OFFSET, 1));

    // 표시해야 하는 블록은 전부 들어 있다.
    for (auto blk : {std::pair<uint16_t, uint16_t>{ecu::REG_DEFINE_OFFSET, ecu::REG_DEFINE_SIZE},
                     {ecu::REG_SYS_OFFSET, ecu::REG_SYS_SIZE},
                     {ecu::REG_LOADCELL_OFFSET, ecu::REG_LOADCELL_SIZE},
                     {ecu::REG_IMU_OFFSET, ecu::REG_IMU_SIZE},
                     {ecu::REG_ENCODER_OFFSET, ecu::REG_ENCODER_SIZE},
                     {ecu::REG_MOTOR_DATA_OFFSET, ecu::REG_MOTOR_DATA_SIZE},
                     {ecu::REG_CMD_MOTOR_OFFSET, ecu::REG_CMD_MOTOR_SIZE},
                     {ecu::REG_CMD_SYSTEM_OFFSET, ecu::REG_CMD_SYSTEM_SIZE},
                     {ecu::REG_DIAG_OFFSET, ecu::REG_DIAG_SIZE}}) {
        EXPECT_TRUE(cc::spans::kAll.Covers(blk.first, blk.second))
            << "read_all 이 " << blk.first << ":" << blk.second << " 를 놓쳤다";
    }
}

// ★ 예산 상수가 STM 버퍼 90 시절의 88 로 남아 있었다 — 파생값이어야 한다.
TEST(B6Catalog, RespBudgetIsDerivedNotHardcoded) {
    EXPECT_EQ(orin_bridge::kMaxRespPayload, orin_bridge::MAX_DATA_LEN - 1);
    EXPECT_GT(orin_bridge::kMaxRespPayload, 88)
        << "88 은 STM PACKET_DATA_BUF_SIZE 가 90 이던 시절의 값이다";
}

TEST(B6Catalog, EveryEntryHasNameAndConsistentKind) {
    for (uint8_t i = 0; i < cc::kCatalogCount; i++) {
        const auto& d = cc::kCatalog[i];
        ASSERT_NE(d.name, nullptr);
        EXPECT_EQ(cc::Find(d.cmd), &d) << d.name << " 를 cmd 로 못 찾는다";
        if (d.kind == cc::CmdKind::READ) {
            EXPECT_NE(d.read, nullptr) << d.name << " 가 READ 인데 구간이 없다";
            EXPECT_FALSE(d.needs_safe_stop) << d.name << " — 읽기에 safe_stop 을 요구한다";
        } else {
            EXPECT_EQ(d.read, nullptr) << d.name << " 가 READ 가 아닌데 구간을 들고 있다";
        }
        if (d.kind == cc::CmdKind::WRITE1) EXPECT_NE(d.waddr, 0) << d.name;
    }
}

}  // namespace
