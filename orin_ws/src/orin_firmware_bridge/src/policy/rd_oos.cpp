#include "orin_firmware_bridge/policy/rd_oos.hpp"

#include <chrono>

namespace orin_bridge {

bool RdOos::TakeStep(OosStep_t* step) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!in_flight_) return false;
    if (step_.phase != OosPhase::WRITE && step_.phase != OosPhase::READ) return false;
    *step = step_;
    return true;
}

// WRITE 실패 시 재시도·READ 없이 즉시 종료한다 (01 §6.2-3) —
// 재시도는 service 재호출(호출자 몫)이라 여기서 tick 을 더 훔치지 않는다.
void RdOos::ReportResult(bool tx_ok, uint8_t readback) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!in_flight_) return;

        if (step_.phase == OosPhase::WRITE) {
            write_ok_ = tx_ok;
            // 성공했을 때만 검증 READ 로 한 tick 더 (§6.2-4: 최대 2 tick)
            step_.phase = tx_ok ? OosPhase::READ : OosPhase::DONE;
        } else if (step_.phase == OosPhase::READ) {
            read_ok_    = tx_ok;
            readback_   = readback;
            step_.phase = OosPhase::DONE;
        }
    }
    cv_.notify_all();
}

bool RdOos::Request(uint16_t addr, uint8_t value, std::string* msg) {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        // §6.2-1 동시 1건 — 큐잉하지 않고 즉시 거부한다 (대기시켰다가 뒤늦게 반영되면
        // 호출자가 언제 적용됐는지 알 수 없어 실험 기록이 흐려진다)
        if (in_flight_) {
            *msg = "다른 단발 설정이 진행 중 — 잠시 후 재시도 (동시 1건만 허용)";
            return false;
        }
        step_      = OosStep_t{OosPhase::WRITE, addr, value};
        in_flight_ = true;
        write_ok_  = false;
        read_ok_   = false;
        readback_  = 0;
    }

    bool write_ok, read_ok; uint8_t readback;
    {
        std::unique_lock<std::mutex> lock(mutex_);
        // §2 검증 규칙: 10 tick(50ms) 내 미확인 → 실패 응답
        const bool done = cv_.wait_for(lock, std::chrono::milliseconds(kVerifyTimeoutMs),
                                       [this]{ return step_.phase == OosPhase::DONE; });
        write_ok = write_ok_;
        read_ok  = read_ok_;
        readback = readback_;
        in_flight_  = false;         // 타임아웃이어도 반드시 해제 — 안 하면 영구 블록
        step_.phase = OosPhase::NONE;
        if (!done) {
            *msg = "검증 시간초과 — " + std::to_string(kVerifyTicks) + " tick(" +
                   std::to_string(kVerifyTimeoutMs) + "ms) 내 read-back 미확인 "
                   "(스케줄 루프 정지 또는 통신 두절 의심)";
            return false;
        }
    }

    if (!write_ok) { *msg = "addr" + std::to_string(addr) + " WRITE 실패 (재시도는 재호출)"; return false; }
    if (!read_ok)  { *msg = "addr" + std::to_string(addr) + " 검증 READ 실패"; return false; }
    if (readback != value) {
        *msg = "read-back 불일치 — addr" + std::to_string(addr) +
               " 기대 " + std::to_string(value) + ", 실제 " + std::to_string(readback);
        return false;
    }
    *msg = "addr" + std::to_string(addr) + " = " + std::to_string(value) + " 검증 OK";
    return true;
}

}  // namespace orin_bridge
