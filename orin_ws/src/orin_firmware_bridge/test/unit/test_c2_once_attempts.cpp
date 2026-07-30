// C2 — `once` 만료를 **시간이 아니라 시도 횟수**로 (redesign/00 C2)
//
// 무엇이 문제였나: REBOOT 성공 시 해당 보드에 **3초 blackout** 이 걸리는데,
// `once` 명령의 만료는 **벽시계 2초** 였다. blackout 중에는 `GetSlotTask` 가 early-return
// 하므로 명령이 **한 번도 시도되지 않는데** 시계는 계속 흘렀다.
// → REBOOT 직후 예약한 명령은 **시도 0회로 죽었다.**
//
// 시간이 아니라 시도를 세면 blackout 이 예산을 갉아먹지 않는다.

#include <gtest/gtest.h>

#include <chrono>
#include <cstring>
#include <memory>
#include <thread>

#include "orin_firmware_bridge/policy/rd_command.hpp"

namespace {

using orin_bridge::RdCommand;
using orin_bridge::RobotState_t;
using orin_bridge::CommandRequest_t;
using orin_bridge::TaskConfig_t;

class C2Once : public ::testing::Test {
protected:
    void SetUp() override {
        st_ = std::make_unique<RobotState_t>();
        std::memset(&st_->ecu.reg, 0, sizeof(st_->ecu.reg));
        cmd_ = std::make_unique<RdCommand>(st_.get());
    }

    // once READ 하나를 슬롯에 넣는다.
    bool Post() {
        CommandRequest_t r;
        r.slot       = 0;
        r.action     = 1;                       // SET
        r.target_id  = orin_bridge::TARGET::ECU;
        // B6 이후 raw 읽기는 CMD_RAW_READ 다 (주소를 직접 주는 유일한 경로).
        r.cmd        = orin_bridge::cmdcat::CMD_RAW_READ;
        r.start_addr = 16;
        r.data_len   = 4;
        r.duration   = orin_bridge::CMD_DURATION_ONCE;
        std::string msg;
        return cmd_->HandleRequest(r, &msg);
    }

    // 실패를 반복해 시도를 소진시킨다. 실제 발사된 횟수를 돌려준다.
    int DrainAttempts(int max_iter) {
        int fired = 0;
        for (int i = 0; i < max_iter; i++) {
            TaskConfig_t t;
            if (!cmd_->GetSlotTask(0, &t)) break;
            fired++;
            cmd_->ReportResult(0, orin_bridge::RD_ERROR);    // 계속 실패 → RET_OK 까지 재시도
        }
        return fired;
    }

    std::unique_ptr<RobotState_t> st_;
    std::unique_ptr<RdCommand>    cmd_;
};

// ★ 만료 기준이 시도 횟수다 — 정확히 kOnceMaxAttempts 번 발사되고 포기한다.
TEST_F(C2Once, ExpiresAfterAttemptCountNotWallClock) {
    ASSERT_TRUE(Post());
    const int fired = DrainAttempts(500);
    EXPECT_EQ(fired, static_cast<int>(orin_bridge::kOnceMaxAttempts))
        << "시도 횟수가 만료 기준이 아니다";
    // 소진 후에는 슬롯이 비어 더 안 나간다.
    TaskConfig_t t;
    EXPECT_FALSE(cmd_->GetSlotTask(0, &t));
}

// ★ 이것이 C2 가 고치려던 버그다 — **시간이 흘러도 시도를 안 했으면 죽지 않는다.**
//   구 코드(2s 벽시계)에서는 여기서 시도 0회로 포기했다.
TEST_F(C2Once, SurvivesLongWaitWithoutAttempts) {
    ASSERT_TRUE(Post());
    // blackout 이 걸린 상황을 시간 경과로 흉내낸다 (구 timeout 2s 를 넘긴다).
    std::this_thread::sleep_for(std::chrono::milliseconds(2200));

    TaskConfig_t t;
    EXPECT_TRUE(cmd_->GetSlotTask(0, &t))
        << "시도를 한 번도 안 했는데 벽시계만으로 포기했다 (C2 가 고치려던 버그)";
    EXPECT_EQ(t.start_addr, 16);
}

// 예산은 **실제 발사에만** 쓰인다 — 기다린 시간은 소모하지 않는다.
TEST_F(C2Once, WaitingDoesNotConsumeBudget) {
    ASSERT_TRUE(Post());
    std::this_thread::sleep_for(std::chrono::milliseconds(2200));
    EXPECT_EQ(DrainAttempts(500), static_cast<int>(orin_bridge::kOnceMaxAttempts));
}

// 성공하면 즉시 해제된다 (once 의 의미) — 시도 예산과 무관하다.
TEST_F(C2Once, SucceedsAndReleasesImmediately) {
    ASSERT_TRUE(Post());
    TaskConfig_t t;
    ASSERT_TRUE(cmd_->GetSlotTask(0, &t));
    cmd_->ReportResult(0, orin_bridge::RD_OK);
    EXPECT_FALSE(cmd_->GetSlotTask(0, &t)) << "성공했는데 슬롯이 남아 있다";
}

}  // namespace
