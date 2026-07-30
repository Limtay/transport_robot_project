#ifndef ORIN_FIRMWARE_BRIDGE__RD_TXN_QUEUE_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_TXN_QUEUE_HPP_

// RdTxnQueue — 락프리 SPSC 링버퍼 (redesign/02 §6.3)
//
// 왜: DDS 발행이 블록되면 200Hz tick 이 그대로 밀린다. 큐로 끊는다.
//
//   [스케줄 스레드]  TryPush(r) → 즉시 반환 (**절대 블록하지 않는다**)
//   [발행 스레드]    Pop(&r)    → 메시지 변환 → publish (여기서 블록돼도 무방)
//
// 설계 규칙 (02 §6.3):
//   Q1 SPSC — 생산자 1(스케줄) / 소비자 1(발행). atomic head/tail + release/acquire
//   Q2 생산자는 절대 대기하지 않는다. 가득 차면 **새 샘플을 드롭**하고 drop_cnt++
//      (오래된 것을 버리면 시계열 순서가 꼬인다)
//   Q3 드롭이 있었으면 다음 성공 샘플에 누적 drop_cnt 를 실어 보낸다 —
//      **분석이 구멍의 위치와 크기를 안다.** 조용한 손실을 만들지 않는 것이 핵심이다
//   Q4 256 슬롯 ≈ 1.28초 분량. 이걸 넘겼다면 소비자가 1초 이상 멈춘 것 = 이미 비정상

#include <atomic>
#include <cstddef>
#include <cstdint>

namespace orin_bridge {

template <typename T, size_t N = 256>
class RdTxnQueue {
    static_assert((N & (N - 1)) == 0, "N 은 2의 거듭제곱이어야 한다 (& 마스크로 감싸기 위함)");

public:
    // [생산자] 성공 시 true. 가득 차면 드롭하고 false — **블록하지 않는다** (Q2).
    bool TryPush(const T& v) {
        const size_t head = head_.load(std::memory_order_relaxed);
        const size_t next = (head + 1) & (N - 1);
        if (next == tail_.load(std::memory_order_acquire)) {
            drop_cnt_.fetch_add(1, std::memory_order_relaxed);
            return false;
        }
        buf_[head] = v;
        head_.store(next, std::memory_order_release);
        return true;
    }

    // [소비자] 꺼낼 것이 있으면 true.
    bool Pop(T* out) {
        const size_t tail = tail_.load(std::memory_order_relaxed);
        if (tail == head_.load(std::memory_order_acquire)) return false;
        *out = buf_[tail];
        tail_.store((tail + 1) & (N - 1), std::memory_order_release);
        return true;
    }

    // 누적 드롭 수 (리셋 없음) — Q3. 0 이면 한 번도 안 막혔다는 뜻이다.
    uint64_t DropCount() const { return drop_cnt_.load(std::memory_order_relaxed); }

    bool Empty() const {
        return tail_.load(std::memory_order_acquire) == head_.load(std::memory_order_acquire);
    }

private:
    T buf_[N];
    std::atomic<size_t>   head_{0};   // 생산자만 쓴다
    std::atomic<size_t>   tail_{0};   // 소비자만 쓴다
    std::atomic<uint64_t> drop_cnt_{0};
};

}  // namespace orin_bridge

#endif  // ORIN_FIRMWARE_BRIDGE__RD_TXN_QUEUE_HPP_
