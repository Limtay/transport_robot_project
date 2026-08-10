#ifndef ORIN_FIRMWARE_BRIDGE__RD_WRITE_ECHO_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_WRITE_ECHO_HPP_

// 쓰기 결과 반영 — 리드백을 뺀 자리를 메운다 (redesign/09 §1.2 ③, U2)
//
// ## 배경
//
// 09 §1.1 에서 control 프리셋의 read-back 20B 를 뺐다. 그전까지 섀도는 **ECU 가 실제로
// 갖고 있는 값**이었지만(응답이 덮어썼다), 지금은 **브리지가 보내려는 값**이다.
// 둘의 차이는 쓰기가 거부됐을 때 드러난다 — ECU 는 안 받았는데 섀도는 받은 것처럼 남고,
// 그 값이 그대로 `ControlFeedback.cmd` 로 나간다.
//
// ## ⚠ 09 §1.2 ③ 의 메커니즘은 성립하지 않는다 (2026-08-05 구현 중 발견)
//
// 계획서는 *"wire 로 내기 직전에 그 구간을 스냅샷하고, err ≠ 0 이면 되돌린다"* 고 적었다.
// **그 스냅샷은 되돌릴 값이 아니다.** 순서가 이렇기 때문이다:
//
//     ① 누군가(PrepareWrite / 서비스 콜백)가 섀도에 **새 의도값 V** 를 쓴다
//     ② Encode 가 섀도에서 V 를 읽어 패킷에 담는다      ← 계획서의 "직전 스냅샷" = V
//     ③ ECU 가 거부한다
//     ④ 되돌려야 할 값은 **직전에 ECU 가 받아들인 값 V_prev** 인데, ②의 스냅샷은 V 다
//
// 즉 계획서대로 하면 V 를 V 로 덮는 **no-op** 이다. 되돌리려면 V_prev 를 따로 들고 있어야
// 하고, 그것이 이 파일이다: **마지막으로 수락된 바이트의 거울(echo).**
//
// ## 왜 바이트별 valid 인가
//
// 기동 직후에는 수락된 값이 없다. 그때 거부가 나면 되돌릴 대상이 **0 으로 초기화된 거울**
// 인데, 그 0 을 섀도에 쓰면 "ECU 가 0 을 갖고 있다" 는 **거짓말을 새로 만든다.**
// 그래서 "이 주소를 성공적으로 쓴 적이 있는가" 를 바이트마다 들고, 없으면 손대지 않는다.
//
// 구간 단위(마지막 성공 구간 하나)로 두면 더 작지만, ECU 에는 200Hz 제어 RW 와 커맨드
// 슬롯의 raw write 가 **번갈아** 나가므로 서로 상대의 구간을 무효화한다. 바이트별이면
// 그 간섭이 없다 (256B × 2 = 512B / 노드).

#include <cstdint>
#include <cstring>

namespace orin_bridge {

// 한 트랜잭션의 **쓰기** 결과. 읽기 성패와 분리한다 — RW 는 에러 니블이 둘이라
// "읽기는 됐는데 쓰기는 거부" 가 정상적으로 존재한다.
enum class WriteOutcome : uint8_t {
    NONE,      // 이 트랜잭션에는 쓰기가 없다 (READ/PING)
    ACCEPTED,  // 보드가 받았다
    REJECTED,  // 보드가 **명시적으로 거부**했다 (err 니블 ≠ 0)
    UNKNOWN,   // 무응답·형식 오류 — 받았는지 알 수 없다
};

inline const char* WriteOutcomeName(WriteOutcome o) {
    switch (o) {
        case WriteOutcome::NONE:     return "none";
        case WriteOutcome::ACCEPTED: return "accepted";
        case WriteOutcome::REJECTED: return "rejected";
        case WriteOutcome::UNKNOWN:  return "unknown";
    }
    return "?";
}

// 응답에서 쓰기 결과를 읽어낸다. **순수 함수** — 패킷 구조체도 ROS 도 모른다.
//
//   has_write   이 inst 가 쓰기를 포함하는가 (WRITE / RW)
//   is_rw       RW 면 err 바이트가 니블 분리다 (read_err | write_err<<4)
//   comm_ok     응답을 받았는가 (timeout 아님)
//   resp_ok     응답의 ID/Inst 가 기대와 맞는가 — **틀리면 err 바이트를 믿으면 안 된다**
//   err_byte    응답 Data[0]
//
// ⚠ **UNKNOWN 에서는 되돌리지 않는다.** 무응답은 "보드가 못 받았다" 가 아니라 "받았는지
//    모른다" 이다 — 요청이 나간 뒤 응답만 유실됐을 수 있다. 모르는 것을 안다고 가정해
//    되돌리면, 실제로는 적용된 값을 섀도에서 지워 다음 비교가 전부 어긋난다.
constexpr WriteOutcome ClassifyWrite(bool has_write, bool is_rw,
                                     bool comm_ok, bool resp_ok, uint8_t err_byte) {
    if (!has_write) return WriteOutcome::NONE;
    if (!comm_ok || !resp_ok) return WriteOutcome::UNKNOWN;
    const uint8_t werr = is_rw ? static_cast<uint8_t>((err_byte >> 4) & 0x0F) : err_byte;
    return werr == 0 ? WriteOutcome::ACCEPTED : WriteOutcome::REJECTED;
}

// 노드 하나(ECU/DPC/PCU)의 "마지막으로 수락된 쓰기 바이트" 거울.
class WriteEcho {
public:
    static constexpr uint16_t kSize = 256;   // 세 보드 모두 레지스터 맵 256B

    // 수락됨 — 보낸 바이트를 거울에 새긴다.
    void NoteAccepted(uint16_t addr, const uint8_t* sent, uint16_t len) {
        if (!sent) return;
        for (uint16_t i = 0; i < len && (addr + i) < kSize; i++) {
            accepted_[addr + i] = sent[i];
            valid_[addr + i]    = true;
        }
    }

    // 거부됨 — 섀도를 거울로 되돌린다. **수락된 적 있는 바이트만** 건드린다.
    // 되돌린 바이트 수를 돌려준다 (0 = 되돌릴 근거가 없었다 = 기동 직후 등).
    uint16_t Rollback(uint16_t addr, uint16_t len, uint8_t* shadow, uint16_t shadow_size) const {
        if (!shadow) return 0;
        uint16_t n = 0;
        for (uint16_t i = 0; i < len; i++) {
            const uint16_t a = static_cast<uint16_t>(addr + i);
            if (a >= kSize || a >= shadow_size) break;
            if (!valid_[a]) continue;              // 수락된 적 없다 — 지어내지 않는다
            if (shadow[a] == accepted_[a]) continue;   // 이미 같다
            shadow[a] = accepted_[a];
            n++;
        }
        return n;
    }

    // 진단용 — 이 주소가 한 번이라도 수락된 적 있는가.
    bool IsAccepted(uint16_t addr) const { return addr < kSize && valid_[addr]; }
    uint8_t AcceptedByte(uint16_t addr) const { return addr < kSize ? accepted_[addr] : 0; }

    void Reset() {
        std::memset(accepted_, 0, sizeof(accepted_));
        std::memset(valid_, 0, sizeof(valid_));
    }

private:
    uint8_t accepted_[kSize] = {};
    bool    valid_[kSize]    = {};
};

}  // namespace orin_bridge

#endif  // ORIN_FIRMWARE_BRIDGE__RD_WRITE_ECHO_HPP_
