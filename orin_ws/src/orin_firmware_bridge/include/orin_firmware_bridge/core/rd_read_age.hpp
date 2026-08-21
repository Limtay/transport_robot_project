#ifndef ORIN_FIRMWARE_BRIDGE__RD_READ_AGE_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_READ_AGE_HPP_

// ReadAgeMap — **섀도의 각 바이트를 마지막으로 읽은 시각** (redesign/09 §5.4 ①, U8)
//
// ## 왜 필요한가
//
// `OP_GET_REGISTERS` 는 섀도를 덤프할 뿐 버스를 건드리지 않는다. 그래서 화면에 뜬 256B 는
// **전부 같은 시점의 값이 아니다** — 주기 프리셋이 덮는 구간은 5ms 전 값이고, 기동 전체읽기
// 로만 채운 구간은 30초 전 값이며, 한 번도 안 읽은 자리는 아예 0 이다.
//
// 종전 표현은 `fresh`(신선한 구간 목록) 하나였다. 이건 **2값 분류**라 위 셋 중 둘을 구분하지
// 못한다: 기동 스냅샷은 "읽긴 읽었는데 낡았다" 인데 `fresh` 밖이므로 "안 읽음" 과 같은
// 회색으로 그려진다. 조작자가 화면의 0 을 보고 "레지스터가 0" 인지 "안 읽어서 0" 인지
// 판별할 수 없다 — 이 프로젝트에서 이미 여러 번 사람을 속인 부류의 모호함이다.
//
// → 구간마다 **경과시간(age)** 을 준다. 2값이 아니라 실수 하나이므로 위 셋이 전부 구분된다.
//
// ## 왜 바이트 단위인가
//
// 구간 목록을 들고 병합하려면 "겹치는 구간 둘이 시각이 다를 때 어떻게 쪼개는가" 를 풀어야
// 한다 (실제로 겹친다 — `kAll` 42:151 안에 프리셋 48:80 이 들어 있다). 바이트마다 시각을
// 두면 그 문제가 사라지고, 스냅샷이 **같은 시각·같은 출처인 연속 바이트를 묶어** 구간을
// 되만들어 준다. 결과는 실제로 읽힌 세그와 정확히 일치한다.
//
// 비용은 3보드 × 256B × 9B ≈ 7KB 다. 200Hz 경로에서 하는 일은 세그당 memset 한 번이다.
//
// ⚠ **여기 있는 것은 "읽은 시각" 이지 "값이 맞다" 가 아니다.** 쓰기만 하고 리드백이 없는
//   주소(DPC 126 등)는 영원히 `NEVER` 로 남는다 — 그것이 정직한 표현이다.

#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
#include <vector>

#include "orin_firmware_bridge/core/rd_comm.hpp"

namespace orin_bridge {

// 무엇이 이 바이트를 읽었는가. 화면이 "왜 이렇게 낡았는가" 를 설명할 때 쓴다 —
// age 만 있으면 조작자는 "30초" 를 보고 통신이 끊긴 줄 안다. 출처가 `init` 이면
// 그건 고장이 아니라 **아무도 주기적으로 안 읽는 구간**이라는 뜻이다.
//
// ⚠ 이름이 `ReadSrc` 가 아닌 이유: 슬롯 표(`rd_slot_table.hpp`)에 이미 같은 이름의 다른
//   enum 이 있다. 그쪽은 "이 슬롯의 읽기 구간을 어디서 가져오는가"(FIXED/PRESET)이고
//   여기는 "이 바이트를 무엇이 읽었는가" 다 — 둘 다 `PRESET` 을 갖고 있어서 한 이름을
//   쓰면 겹치는 순간 컴파일러가 아니라 사람이 헷갈린다.
enum class ReadOrigin : uint8_t {
    NEVER = 0,   // 한 번도 안 읽음 — 섀도에 값이 있어도 그건 초기값이지 관측이 아니다
    INIT,        // 기동 전체읽기 / INIT 의 검증 READ
    PRESET,      // 주기 슬롯 (control 프리셋 · project/manual 프레임)
    SLOT,        // 커맨드 슬롯 READ (CMD_READ_* / raw_read)
    OOS,         // out-of-span 단발 검증 READ
};

inline const char* ReadOriginName(ReadOrigin s) {
    switch (s) {
        case ReadOrigin::NEVER:  return "never";
        case ReadOrigin::INIT:   return "init";
        case ReadOrigin::PRESET: return "preset";
        case ReadOrigin::SLOT:   return "slot";
        case ReadOrigin::OOS:    return "oos";
    }
    return "unknown";
}

class ReadAgeMap {
public:
    using Clock     = std::chrono::steady_clock;
    using TimePoint = Clock::time_point;

    // 스냅샷 한 칸 — 같은 시각·같은 출처로 읽힌 연속 구간.
    struct Span {
        uint16_t addr  = 0;
        uint16_t len   = 0;
        double   age_s = 0.0;
        ReadOrigin src = ReadOrigin::NEVER;
    };

    static constexpr int kTargets = 3;    // ECU / DPC / PCU
    static constexpr int kBytes   = 256;  // 세 보드 모두 REG_TOTAL_SIZE = 256

    // 구간 하나를 "방금 읽었다" 로 도장 찍는다. 범위를 벗어나는 부분은 조용히 잘린다 —
    // 여기서 기동을 막을 이유가 없다 (표시용 메타데이터다).
    // ⚠ 호출자가 `RobotState_t::state_mutex` 를 잡은 상태여야 한다 (섀도와 같은 락).
    void Mark(uint8_t target_id, uint16_t addr, uint16_t len, ReadOrigin src, TimePoint now) {
        const int t = Index(target_id);
        if (t < 0 || src == ReadOrigin::NEVER) return;
        const int end = static_cast<int>(addr) + static_cast<int>(len);
        const int hi  = end < kBytes ? end : kBytes;
        for (int i = addr; i < hi; i++) {
            stamp_[t][i] = now;
            src_[t][i]   = src;
        }
    }

    // 같은 (시각, 출처) 를 가진 연속 바이트를 묶어 돌려준다. 안 읽은 바이트는 **빠진다** —
    // 목록에 없다는 것이 곧 "미판독" 이다 (`age_s: null` 같은 특수값을 만들지 않는다).
    std::vector<Span> Snapshot(uint8_t target_id, TimePoint now) const {
        std::vector<Span> out;
        const int t = Index(target_id);
        if (t < 0) return out;
        int i = 0;
        while (i < kBytes) {
            if (src_[t][i] == ReadOrigin::NEVER) { i++; continue; }
            const TimePoint st = stamp_[t][i];
            const ReadOrigin sc = src_[t][i];
            int j = i;
            while (j < kBytes && src_[t][j] == sc && stamp_[t][j] == st) j++;
            Span s;
            s.addr  = static_cast<uint16_t>(i);
            s.len   = static_cast<uint16_t>(j - i);
            s.age_s = std::chrono::duration<double>(now - st).count();
            s.src   = sc;
            out.push_back(s);
            i = j;
        }
        return out;
    }

    // 한 번이라도 읽힌 적이 있는가 (테스트·로그용).
    bool AnyRead(uint8_t target_id) const {
        const int t = Index(target_id);
        if (t < 0) return false;
        for (int i = 0; i < kBytes; i++) if (src_[t][i] != ReadOrigin::NEVER) return true;
        return false;
    }

private:
    static int Index(uint8_t target_id) {
        switch (target_id) {
            case static_cast<uint8_t>(PacketID::ECU):   return 0;
            case static_cast<uint8_t>(PacketID::DPC_B): return 1;
            case static_cast<uint8_t>(PacketID::PCU):   return 2;
            default:                                    return -1;
        }
    }

    TimePoint stamp_[kTargets][kBytes] = {};
    ReadOrigin src_[kTargets][kBytes]   = {};
};

// 스냅샷을 JSON 배열 문자열로. **`OP_GET_REGISTERS` 응답에 그대로 박힌다.**
//
// 여기 따로 뺀 이유: 종전 `fresh` 배열은 서비스 콜백 한가운데서 `ostringstream` 으로
// 조립돼 어떤 유닛 테스트도 닿지 못했다. 그래서 키 이름이나 소수점 형식이 바뀌어도
// 웹을 띄우기 전에는 아무도 모른다.
//
// ⚠ 로케일이 소수점을 `,` 로 찍으면 JSON 이 깨진다 — `snprintf("%.3f")` 로 고정한다
//   (rd_status.cpp 가 같은 이유로 같은 처방을 쓴다).
inline std::string ReadAgeSpansToJson(const std::vector<ReadAgeMap::Span>& spans) {
    std::string out = "[";
    for (size_t i = 0; i < spans.size(); i++) {
        char b[128];
        std::snprintf(b, sizeof(b),
                      "%s{\"addr\":%u,\"len\":%u,\"age_s\":%.3f,\"src\":\"%s\"}",
                      i ? "," : "",
                      static_cast<unsigned>(spans[i].addr),
                      static_cast<unsigned>(spans[i].len),
                      spans[i].age_s,
                      ReadOriginName(spans[i].src));
        out += b;
    }
    out += "]";
    return out;
}

}  // namespace orin_bridge

#endif  // ORIN_FIRMWARE_BRIDGE__RD_READ_AGE_HPP_
