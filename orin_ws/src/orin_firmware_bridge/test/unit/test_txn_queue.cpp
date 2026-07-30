// 큐 포화 테스트 (redesign/06 §2.3)
//
// 왜 따로 필요한가: 4단계는 **wire 를 안 바꾼다.** 그래서 골든 바이트 테스트가 통과해도
// "200Hz 발행이 막히는지" 는 전혀 모른다. 골든이 못 보는 구멍을 이 테스트가 메운다.
//
// 확인하는 계약 (02 §6.3):
//   Q2 생산자는 큐가 가득 차도 **블록하지 않는다** — 시간으로 잰다
//   Q2 가득 차면 **새 샘플을 드롭**한다 (오래된 것을 버리지 않는다 = 시계열 순서 보존)
//   Q3 드롭은 **조용히 사라지지 않는다** — drop_cnt 로 셀 수 있다

#include <gtest/gtest.h>

#include <atomic>
#include <chrono>
#include <thread>

#include "orin_firmware_bridge/sched/rd_txn_queue.hpp"

using orin_bridge::RdTxnQueue;

namespace {

TEST(TxnQueue, PushPopFifo) {
    RdTxnQueue<int, 8> q;
    for (int i = 0; i < 7; i++) EXPECT_TRUE(q.TryPush(i)) << "용량-1 까지는 들어가야 한다";
    int v = -1;
    for (int i = 0; i < 7; i++) {
        ASSERT_TRUE(q.Pop(&v));
        EXPECT_EQ(v, i) << "FIFO 순서가 깨지면 시계열 분석이 무의미해진다";
    }
    EXPECT_FALSE(q.Pop(&v));
    EXPECT_EQ(q.DropCount(), 0u);
}

// Q2 — 가득 차면 새 것을 버린다. 오래된 것을 밀어내지 않는다.
TEST(TxnQueue, FullDropsNewestNotOldest) {
    RdTxnQueue<int, 4> q;                       // 실제 용량 3
    EXPECT_TRUE(q.TryPush(10));
    EXPECT_TRUE(q.TryPush(11));
    EXPECT_TRUE(q.TryPush(12));
    EXPECT_FALSE(q.TryPush(13)) << "가득 찬 큐는 false 를 돌려줘야 한다";
    EXPECT_FALSE(q.TryPush(14));
    EXPECT_EQ(q.DropCount(), 2u) << "Q3: 드롭이 조용히 사라지면 분석이 구멍을 못 본다";

    int v = -1;
    ASSERT_TRUE(q.Pop(&v)); EXPECT_EQ(v, 10) << "가장 오래된 것이 살아 있어야 한다";
    ASSERT_TRUE(q.Pop(&v)); EXPECT_EQ(v, 11);
    ASSERT_TRUE(q.Pop(&v)); EXPECT_EQ(v, 12);
}

// Q2 의 핵심 — **소비자가 멈춰도 생산자는 블록되지 않는다.**
// 이것이 깨지면 DDS 가 막힐 때 200Hz tick 이 그대로 밀린다 (4단계의 존재 이유).
TEST(TxnQueue, ProducerNeverBlocksWhenConsumerStalls) {
    RdTxnQueue<int, 256> q;
    constexpr int kPush = 200000;               // 큐 용량의 780배 — 대부분 드롭된다

    const auto t0 = std::chrono::steady_clock::now();
    for (int i = 0; i < kPush; i++) q.TryPush(i);   // 소비자 없음 = 영구 포화
    const auto dt = std::chrono::steady_clock::now() - t0;

    const double ms = std::chrono::duration<double, std::milli>(dt).count();
    // 200Hz tick 예산은 5ms. 20만 회 TryPush 가 그 한 tick 안에 끝나야 한다는
    // 뜻은 아니지만, **블록이 없다면** 이 정도는 수 ms 다. 넉넉히 100ms 로 잡는다.
    EXPECT_LT(ms, 100.0) << "생산자가 " << ms << "ms 걸렸다 — 어딘가에서 대기하고 있다";
    EXPECT_EQ(q.DropCount(), static_cast<uint64_t>(kPush) - 255u);
}

// SPSC 계약 — 생산자/소비자 각 1스레드에서 손실·중복·순서 뒤바뀜이 없어야 한다.
TEST(TxnQueue, SpscNoLossWhenConsumerKeepsUp) {
    RdTxnQueue<int, 256> q;
    constexpr int kN = 50000;
    std::atomic<bool> done{false};
    int received = 0, expect = 0;
    bool ordered = true;

    std::thread consumer([&] {
        int v;
        while (!done.load() || !q.Empty()) {
            if (q.Pop(&v)) {
                if (v != expect++) ordered = false;
                received++;
            }
        }
    });

    // ⚠ 실제 생산자는 **재시도하지 않는다** (Q2). 여기서만 재시도해 손실 없는 스트림을
    //    만들고 순서·개수를 본다. 재시도한 TryPush 실패도 설계상 드롭으로 집계되므로
    //    이 테스트에서 DropCount 는 의미가 없다 — 그건 위 포화 테스트가 본다.
    for (int i = 0; i < kN; i++) {
        while (!q.TryPush(i)) std::this_thread::yield();
    }
    done.store(true);
    consumer.join();

    EXPECT_TRUE(ordered);
    EXPECT_EQ(received, kN) << "재시도로 밀어넣은 만큼 정확히 나와야 한다 (손실·중복 없음)";
}

}  // namespace
