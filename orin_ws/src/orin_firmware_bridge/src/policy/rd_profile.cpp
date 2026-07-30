#include "orin_firmware_bridge/policy/rd_profile.hpp"

#include <cmath>

#include "orin_firmware_bridge/core/rd_register_ecu.hpp"

#include <yaml-cpp/yaml.h>
#include <cmath>

#include "orin_firmware_bridge/core/rd_register_ecu.hpp"
#include <random>
#include <algorithm>
#include <sstream>
#include <chrono>

namespace orin_bridge {

namespace {

// YAML 노드에서 필수 스칼라를 꺼낸다 — 없으면 위치를 담은 사유를 만들어 실패시킨다.
// "어느 모터 몇 번째 세그먼트의 무슨 키" 까지 나와야 사용자가 YAML 을 고칠 수 있다.
bool Req(const YAML::Node& n, const char* key, double* out,
         int motor_no, int seg_idx, const char* type, std::string* err) {
    if (!n[key]) {
        std::ostringstream os;
        os << "m" << motor_no << " 세그먼트[" << seg_idx << "] (type=" << type
           << "): 필수 키 '" << key << "' 누락";
        *err = os.str();
        return false;
    }
    try {
        *out = n[key].as<double>();
    } catch (const YAML::Exception&) {
        std::ostringstream os;
        os << "m" << motor_no << " 세그먼트[" << seg_idx << "] (type=" << type
           << "): '" << key << "' 를 숫자로 읽을 수 없음";
        *err = os.str();
        return false;
    }
    return true;
}

double Opt(const YAML::Node& n, const char* key, double fallback) {
    if (!n[key]) return fallback;
    try { return n[key].as<double>(); } catch (const YAML::Exception&) { return fallback; }
}

} // namespace

// 세그먼트 1개를 tick 배열로 펼친다 (§4.2 9종).
// 실패 시 false + err. 성공 시 out 뒤에 append.
// `was_noise` / 클램프 한계를 받는 이유는 E3 때문이다 (05 §3.4):
//   - noise 구간은 slew 검사에서 제외해야 한다 → 어느 tick 이 noise 인지 호출자가 알아야 한다
//   - 대신 진폭 쪽 안전장치(mean ± 4σ)를 여기서 건다 → 실효 클램프 범위를 알아야 한다
static bool ExpandSegment(const YAML::Node& seg, int motor_no, int seg_idx,
                          std::mt19937_64& rng, std::vector<float>* out, std::string* err,
                          double lim_lo, double lim_hi, bool* was_noise) {
    if (was_noise) *was_noise = false;
    if (!seg["type"]) {
        std::ostringstream os;
        os << "m" << motor_no << " 세그먼트[" << seg_idx << "]: 'type' 키 누락";
        *err = os.str();
        return false;
    }
    const std::string type = seg["type"].as<std::string>("");

    // custom 은 duration 대신 samples[] 길이가 구간을 정한다 — 먼저 처리.
    if (type == "custom") {
        if (!seg["samples"] || !seg["samples"].IsSequence()) {
            std::ostringstream os;
            os << "m" << motor_no << " 세그먼트[" << seg_idx
               << "] (type=custom): 'samples' 배열 누락";
            *err = os.str();
            return false;
        }
        const double rate = Opt(seg, "rate", static_cast<double>(RdProfile::kTickHz));
        // 05 §3.3 — 200Hz 초과 샘플은 **물리적으로 재생할 방법이 없다.** 브리지가 조용히
        // 다운샘플하면 "재생된 것과 기록된 것이 달라진다" (§4.3-4 가 금지하는 바로 그것).
        // 변환은 내보내는 쪽(웹·스크립트) 책임으로 두고, 사유를 밝히며 거부한다.
        if (!(rate >= 1.0 && rate <= RdProfile::kTickHz)) {
            std::ostringstream os;
            os << "m" << motor_no << " 세그먼트[" << seg_idx << "] (type=custom): rate " << rate
               << " 는 1~200 범위 밖 — 내보내기 단계에서 200Hz 로 다운샘플(평균) 후 제출할 것";
            *err = os.str();
            return false;
        }
        std::vector<double> src;
        for (const auto& sm : seg["samples"]) src.push_back(sm.as<double>(0.0));
        if (src.empty() || src.size() > 100000) {
            std::ostringstream os;
            os << "m" << motor_no << " 세그먼트[" << seg_idx << "] (type=custom): samples 길이 "
               << src.size() << " (1~100,000 이어야 함)";
            *err = os.str();
            return false;
        }
        // 05 §4.3 — **기본 보간을 선형으로.** floor(최근접)는 rate 가 낮을수록 큰 계단
        // 불연속을 만든다. 50Hz 로 그린 매끄러운 곡선이 200Hz 재생에서 4 tick 마다 튀는
        // 계단이 되고, 전류 지령이면 그대로 토크 충격이다. slew_rate 와 함께 쓰면 그 계단이
        // 위반으로 잡혀 **사용자가 그리지도 않은 이유로 reject** 된다.
        // rate==200 이면 두 방식의 결과가 완전히 같으므로 기존 기록에 영향이 없다.
        const std::string interp = seg["interp"] ? seg["interp"].as<std::string>("linear") : "linear";
        if (interp != "linear" && interp != "nearest") {
            std::ostringstream os;
            os << "m" << motor_no << " 세그먼트[" << seg_idx
               << "] (type=custom): interp 는 linear|nearest";
            *err = os.str();
            return false;
        }
        const size_t n = static_cast<size_t>(std::llround(src.size() / rate * RdProfile::kTickHz));
        out->reserve(out->size() + n);
        for (size_t i = 0; i < n; i++) {
            const double t = i * RdProfile::kDt;
            const double x = t * rate;                       // 소스 인덱스 (실수)
            double v;
            if (interp == "nearest" || src.size() == 1) {
                size_t idx = static_cast<size_t>(x);
                if (idx >= src.size()) idx = src.size() - 1;
                v = src[idx];
            } else {
                const size_t i0 = static_cast<size_t>(x);
                if (i0 + 1 >= src.size()) {
                    v = src.back();                          // 마지막 샘플을 넘으면 유지 (§4.3)
                } else {
                    const double f = x - static_cast<double>(i0);
                    v = src[i0] * (1.0 - f) + src[i0 + 1] * f;
                }
            }
            out->push_back(static_cast<float>(v));
        }
        return true;
    }

    // stair 는 values[]×step_duration 으로 구간이 정해진다.
    if (type == "stair") {
        if (!seg["values"] || !seg["values"].IsSequence()) {
            std::ostringstream os;
            os << "m" << motor_no << " 세그먼트[" << seg_idx
               << "] (type=stair): 'values' 배열 누락";
            *err = os.str();
            return false;
        }
        double step_dur = 0.0;
        if (!Req(seg, "step_duration", &step_dur, motor_no, seg_idx, "stair", err)) return false;
        if (step_dur <= 0.0) {
            std::ostringstream os;
            os << "m" << motor_no << " 세그먼트[" << seg_idx << "] (type=stair): step_duration 은 양수여야 함";
            *err = os.str();
            return false;
        }
        const size_t nvals = seg["values"].size();
        if (nvals < 1 || nvals > 1000) {
            std::ostringstream os;
            os << "m" << motor_no << " 세그먼트[" << seg_idx << "] (type=stair): values 길이 "
               << nvals << " (1~1,000 이어야 함)";
            *err = os.str();
            return false;
        }
        const size_t per = static_cast<size_t>(std::llround(step_dur * RdProfile::kTickHz));
        // 05 §3.2 — **펼치기 전에** 총 tick 을 검사한다. 상한을 넘는 프로파일은
        // 메모리를 한 바이트도 쓰기 전에 거부된다.
        if (per * nvals > RdProfile::kMaxTicks) {
            std::ostringstream os;
            os << "m" << motor_no << " 세그먼트[" << seg_idx << "] (type=stair): "
               << (per * nvals) << " tick 은 상한 " << RdProfile::kMaxTicks << " 초과";
            *err = os.str();
            return false;
        }
        out->reserve(out->size() + per * nvals);
        for (const auto& v : seg["values"]) {
            const float val = v.as<float>(0.0f);
            for (size_t i = 0; i < per; i++) out->push_back(val);
        }
        return true;
    }

    // 나머지 7종은 duration 기반.
    double duration = 0.0;
    if (!Req(seg, "duration", &duration, motor_no, seg_idx, type.c_str(), err)) return false;
    // 05 §3.2 — 확장 전 tick 수 검사. 모든 타입이 산술식으로 구할 수 있다.
    if (duration > 0.0) {
        const size_t n_pre = static_cast<size_t>(std::llround(duration * RdProfile::kTickHz));
        if (n_pre > RdProfile::kMaxTicks) {
            std::ostringstream os;
            os << "m" << motor_no << " 세그먼트[" << seg_idx << "] (type=" << type << "): "
               << n_pre << " tick 은 상한 " << RdProfile::kMaxTicks << " 초과 ("
               << RdProfile::kMaxDuration << "s)";
            *err = os.str();
            return false;
        }
        out->reserve(out->size() + n_pre);
    }
    if (duration <= 0.0) {
        std::ostringstream os;
        os << "m" << motor_no << " 세그먼트[" << seg_idx << "] (type=" << type
           << "): duration 은 양수여야 함";
        *err = os.str();
        return false;
    }
    const size_t n = static_cast<size_t>(std::llround(duration * RdProfile::kTickHz));

    if (type == "hold") {
        double v = 0.0;
        if (!Req(seg, "value", &v, motor_no, seg_idx, "hold", err)) return false;
        for (size_t i = 0; i < n; i++) out->push_back(static_cast<float>(v));

    } else if (type == "ramp") {
        double from = 0.0, to = 0.0;
        if (!Req(seg, "from", &from, motor_no, seg_idx, "ramp", err)) return false;
        if (!Req(seg, "to",   &to,   motor_no, seg_idx, "ramp", err)) return false;
        for (size_t i = 0; i < n; i++) {
            // 마지막 tick 이 정확히 to 가 되도록 (n-1) 로 나눈다 — 히스테리시스 실험에서
            // 상승 끝값과 하강 시작값이 어긋나면 안 된다.
            const double a = (n > 1) ? static_cast<double>(i) / (n - 1) : 1.0;
            out->push_back(static_cast<float>(from + (to - from) * a));
        }

    } else if (type == "step") {
        double from = 0.0, to = 0.0, t_step = 0.0;
        if (!Req(seg, "from",   &from,   motor_no, seg_idx, "step", err)) return false;
        if (!Req(seg, "to",     &to,     motor_no, seg_idx, "step", err)) return false;
        if (!Req(seg, "t_step", &t_step, motor_no, seg_idx, "step", err)) return false;
        // 05 §3.3 — t_step 이 구간 밖이면 step 이 아니라 hold 다. 조용히 그렇게 되면
        // 사용자는 계단을 그렸다고 믿는다.
        if (!(t_step > 0.0 && t_step < duration)) {
            std::ostringstream os;
            os << "m" << motor_no << " 세그먼트[" << seg_idx << "] (type=step): t_step "
               << t_step << " 은 0 < t_step < duration(" << duration << ") 이어야 함";
            *err = os.str();
            return false;
        }
        const size_t k = static_cast<size_t>(std::llround(t_step * RdProfile::kTickHz));
        for (size_t i = 0; i < n; i++) {
            out->push_back(static_cast<float>(i < k ? from : to));
        }

    } else if (type == "sine") {
        double amp = 0.0, freq = 0.0;
        if (!Req(seg, "amp",  &amp,  motor_no, seg_idx, "sine", err)) return false;
        if (!Req(seg, "freq", &freq, motor_no, seg_idx, "sine", err)) return false;
        // 05 §3.3 — 25Hz = 주기당 8샘플(200Hz 체인). 그 이상은 재생되는 계단파가 의도한
        // 정현파와 다르고, 100Hz(Nyquist)에 다가갈수록 앨리어싱으로 **완전히 다른 저주파
        // 파형**이 나온다. 실제 기구 대역은 훨씬 낮으므로 5Hz 초과는 경고다.
        if (!(freq > 0.0 && freq <= 25.0)) {
            std::ostringstream os;
            os << "m" << motor_no << " 세그먼트[" << seg_idx << "] (type=sine): freq " << freq
               << " Hz 는 0 < freq <= 25 여야 함 (200Hz 체인에서 주기당 8샘플이 하한)";
            *err = os.str();
            return false;
        }
        const double offset = Opt(seg, "offset", 0.0);
        for (size_t i = 0; i < n; i++) {
            const double t = i * RdProfile::kDt;
            out->push_back(static_cast<float>(offset + amp * std::sin(2.0 * M_PI * freq * t)));
        }

    } else if (type == "chirp") {
        double amp = 0.0, f0 = 0.0, f1 = 0.0;
        if (!Req(seg, "amp", &amp, motor_no, seg_idx, "chirp", err)) return false;
        if (!Req(seg, "f0",  &f0,  motor_no, seg_idx, "chirp", err)) return false;
        if (!Req(seg, "f1",  &f1,  motor_no, seg_idx, "chirp", err)) return false;
        if (!(f0 >= 0.0 && f0 <= 25.0) || !(f1 >= 0.0 && f1 <= 25.0)) {
            std::ostringstream os;
            os << "m" << motor_no << " 세그먼트[" << seg_idx << "] (type=chirp): f0/f1 은 0~25 Hz";
            *err = os.str();
            return false;
        }
        const double offset = Opt(seg, "offset", 0.0);
        for (size_t i = 0; i < n; i++) {
            const double t = i * RdProfile::kDt;
            // 선형 스윕의 순간위상 = 2π(f0·t + (f1-f0)/(2T)·t²)
            const double phase = 2.0 * M_PI * (f0 * t + (f1 - f0) * t * t / (2.0 * duration));
            out->push_back(static_cast<float>(offset + amp * std::sin(phase)));
        }

    } else if (type == "prbs") {
        double low = 0.0, high = 0.0, bit_dur = 0.0;
        if (!Req(seg, "low",          &low,     motor_no, seg_idx, "prbs", err)) return false;
        if (!Req(seg, "high",         &high,    motor_no, seg_idx, "prbs", err)) return false;
        if (!Req(seg, "bit_duration", &bit_dur, motor_no, seg_idx, "prbs", err)) return false;
        // 05 §3.3 — 1 tick(5ms) 미만이면 비트가 표현되지 않는다. low==high 면 hold 다.
        if (bit_dur < RdProfile::kDt) {
            std::ostringstream os;
            os << "m" << motor_no << " 세그먼트[" << seg_idx << "] (type=prbs): bit_duration "
               << bit_dur << " s 는 1 tick(" << RdProfile::kDt << "s) 이상이어야 함";
            *err = os.str();
            return false;
        }
        if (low == high) {
            std::ostringstream os;
            os << "m" << motor_no << " 세그먼트[" << seg_idx
               << "] (type=prbs): low 와 high 가 같다 — hold 를 쓸 것";
            *err = os.str();
            return false;
        }
        if (bit_dur <= 0.0) {
            std::ostringstream os;
            os << "m" << motor_no << " 세그먼트[" << seg_idx << "] (type=prbs): bit_duration 은 양수여야 함";
            *err = os.str();
            return false;
        }
        const size_t per = std::max<size_t>(1, static_cast<size_t>(std::llround(bit_dur * RdProfile::kTickHz)));
        std::bernoulli_distribution coin(0.5);
        float cur = coin(rng) ? static_cast<float>(high) : static_cast<float>(low);
        for (size_t i = 0; i < n; i++) {
            if (i % per == 0) cur = coin(rng) ? static_cast<float>(high) : static_cast<float>(low);
            out->push_back(cur);
        }

    } else if (type == "noise") {
        double mean = 0.0, stddev = 0.0;
        if (!Req(seg, "mean", &mean,   motor_no, seg_idx, "noise", err)) return false;
        if (!Req(seg, "std",  &stddev, motor_no, seg_idx, "noise", err)) return false;
        // 05 §3.3 — std<=0 이면 noise 가 아니라 hold(mean) 다. 조용히 두면 실험 기록에
        // "noise 세그먼트" 로 남는데 실제로는 상수였다는 뜻이 된다.
        if (!(stddev > 0.0)) {
            std::ostringstream os;
            os << "m" << motor_no << " 세그먼트[" << seg_idx
               << "] (type=noise): std 는 양수여야 함 (0 이면 hold 를 쓸 것)";
            *err = os.str();
            return false;
        }
        // E3-3 진폭 안전장치 (05 §3.4): mean ± 4σ 가 실효 클램프를 벗어나면 **거부**한다.
        //
        // 왜 클램프에 맡기지 않는가 — 클램프가 분포의 꼬리를 잘라내면 **실제 재생된 분포가
        // YAML 이 선언한 정규분포와 달라진다.** 시스템 식별 입력이 선언과 다르면 동정 결과가
        // 틀리는데, 그 사실이 기록 어디에도 남지 않는다 (clamp_cnt 는 숫자일 뿐 분포가
        // 바뀌었다는 뜻을 담지 못한다).
        //
        // 4σ 인 이유: 720,000 tick(상한)에서 기대 초과 횟수가 2·Φ(-4)·7.2e5 ≈ 45 회 —
        // 실무상 "거의 안 잘린다" 는 뜻이다.
        if (lim_lo < lim_hi) {
            const double lo4 = mean - 4.0 * stddev, hi4 = mean + 4.0 * stddev;
            if (lo4 < lim_lo || hi4 > lim_hi) {
                std::ostringstream os;
                os << "m" << motor_no << " 세그먼트[" << seg_idx
                   << "] (type=noise): mean±4σ = [" << lo4 << ", " << hi4
                   << "] 가 실효 한계 [" << lim_lo << ", " << lim_hi << "] 를 벗어난다 — "
                   << "클램프가 분포의 꼬리를 잘라내면 재생된 잡음이 선언한 정규분포와 "
                   << "달라진다 (05 §3.4 E3-3). std 를 줄이거나 limits 를 넓힐 것";
                *err = os.str();
                return false;
            }
        }
        if (was_noise) *was_noise = true;
        std::normal_distribution<double> gauss(mean, stddev);
        for (size_t i = 0; i < n; i++) out->push_back(static_cast<float>(gauss(rng)));

    } else {
        std::ostringstream os;
        os << "m" << motor_no << " 세그먼트[" << seg_idx << "]: 알 수 없는 type '" << type
           << "' (지원: hold/ramp/stair/step/sine/chirp/prbs/noise/custom)";
        *err = os.str();
        return false;
    }
    return true;
}

const char* RdProfile::ModeStr(Mode m) {
    switch (m) {
        case Mode::CURRENT:  return "current";
        case Mode::VELOCITY: return "velocity";
        case Mode::POSITION: return "position";
    }
    return "?";
}

// 05 §2.3 — mode <-> auto_mode <-> ctr_mode 2단 검사.
// 이것이 01 §7 의 "가드 판정 기준을 ctr_mode 에서 auto_mode 로 한 단계 올린다" 를 구현하는 자리다.
// DIRECT 일 때만 ctr_mode 까지 내려간다 — 그때는 ctr_mode 가 bridge 소유(write 128:52)라
// shadow 값이 신뢰 가능하다.
bool RdProfile::AcceptsAutoMode(uint8_t auto_mode, const uint8_t ctr_mode[4],
                                uint8_t active_mask, std::string* err) const {
    const char* need = nullptr;
    uint8_t     native = 0;
    switch (mode_) {
        case Mode::CURRENT:  native = ecu::AUTO_MODE_CURRENT;  need = "current";  break;
        case Mode::VELOCITY: native = ecu::AUTO_MODE_VELOCITY; need = "velocity"; break;
        case Mode::POSITION: native = ecu::AUTO_MODE_POSITION; need = "position"; break;
    }
    if (auto_mode != native && auto_mode != ecu::AUTO_MODE_DIRECT) {
        // 사유에 **현재 값과 해야 할 일**을 함께 적는다 — 원인 불명 거부를 만들지 않는다.
        *err = std::string("mode: ") + need + " 프로파일 — 현재 auto_mode=" +
               std::to_string(auto_mode) + ". `control_cli config auto_mode " +
               std::to_string(native) + "` 후 재시도 (또는 DIRECT)";
        return false;
    }
    if (auto_mode != ecu::AUTO_MODE_DIRECT) return true;

    // DIRECT — 활성 모터 전부의 ctr_mode 를 본다
    for (int i = 0; i < 4; i++) {
        if (!((active_mask >> i) & 0x01)) continue;
        const uint8_t cm = ctr_mode[i];
        bool ok = false;
        switch (mode_) {
            case Mode::CURRENT:
                ok = (cm == ecu::CTR_MODE_CURRENT || cm == 2 /* CURRENT_BRAKE */); break;
            case Mode::VELOCITY: ok = (cm == ecu::CTR_MODE_VELOCITY); break;
            case Mode::POSITION: ok = (cm == ecu::CTR_MODE_POSITION); break;
        }
        if (!ok) {
            *err = std::string("mode: ") + need + " 인데 M" + std::to_string(i + 1) +
                   " 의 ctr_mode=" + std::to_string(cm) +
                   " (DIRECT 에서는 모터별 ctr_mode 가 단위를 정한다 — "
                   "`control_cli config ctr_mode " + std::to_string(i + 1) + " <값>`)";
            return false;
        }
    }
    return true;
}

bool RdProfile::LoadFromYaml(const std::string& yaml_text, uint8_t active_mask,
                             float global_max_current, std::string* err) {
    name_.clear();
    tick_count_ = 0;
    clamp_cnt_  = 0;
    for (int i = 0; i < 4; i++) samples_[i].clear();
    segment_index_.clear();

    YAML::Node root;
    try {
        root = YAML::Load(yaml_text);
    } catch (const YAML::Exception& e) {
        *err = std::string("YAML 파싱 실패: ") + e.what();   // yaml-cpp 가 line/col 을 포함해 준다
        return false;
    }
    if (!root.IsMap()) {
        *err = "YAML 최상위가 맵이 아님 (name/motors 키를 가진 맵이어야 함)";
        return false;
    }

    name_ = root["name"] ? root["name"].as<std::string>("") : "";

    if (!root["motors"] || !root["motors"].IsMap()) {
        *err = "'motors' 맵 누락 — 최소 1개 모터의 세그먼트 목록이 필요";
        return false;
    }

    // mode: (05 §2.2) — 없으면 current. 기존 프로파일이 그대로 돈다.
    mode_ = Mode::CURRENT;
    if (root["mode"]) {
        const std::string ms = root["mode"].as<std::string>("current");
        if      (ms == "current")  mode_ = Mode::CURRENT;
        else if (ms == "velocity") mode_ = Mode::VELOCITY;
        else if (ms == "position") mode_ = Mode::POSITION;
        else { *err = "mode: '" + ms + "' 는 current|velocity|position 중 하나여야 한다"; return false; }
    }

    // limits — **모드 의존** (05 §2.4). velocity/position 은 전역 기본값이 없으므로 필수다.
    //   velocity/position 용 전역 파라미터를 새로 만들지 않는 이유: 이 모드로 돌린 실험이
    //   0건이라 안전한 기본값을 모른다. **모르는 값에 기본값을 주는 것보다 프로파일이 매번
    //   명시하게 강제하는 쪽이 안전하다.**
    float limit_lo = -global_max_current;
    float limit_hi =  global_max_current;
    bool  has_slew = false;
    double slew_rate = 0.0;
    {
        const auto lim = root["limits"];
        const bool has_abs   = lim && lim["max_abs"];
        const bool has_range = lim && lim["range"];
        const bool has_maxc  = lim && lim["max_current"];

        if (has_range) {
            const auto r = lim["range"];
            if (!r.IsSequence() || r.size() != 2) {
                *err = "limits.range 는 [lo, hi] 두 값이어야 한다"; return false;
            }
            limit_lo = r[0].as<float>();
            limit_hi = r[1].as<float>();
            if (!(limit_lo < limit_hi)) { *err = "limits.range 는 lo < hi 여야 한다"; return false; }
        } else if (has_abs) {
            const float a = std::fabs(lim["max_abs"].as<float>());
            limit_lo = -a; limit_hi = a;
        } else if (has_maxc) {
            // deprecated 별칭 — mode: current 에서만 유효 (05 §2.4)
            if (mode_ != Mode::CURRENT) {
                *err = "limits.max_current 는 mode: current 전용 (deprecated) — "
                       "velocity/position 은 max_abs 또는 range 를 쓸 것";
                return false;
            }
            const float a = std::fabs(lim["max_current"].as<float>());
            limit_lo = -a; limit_hi = a;
        }

        // 모드별 필수 검사
        if (mode_ == Mode::VELOCITY && !(has_abs || has_range)) {
            *err = "mode: velocity 는 limits.max_abs 필수 — 전역 기본값이 없다"; return false;
        }
        if (mode_ == Mode::POSITION && !has_range) {
            *err = "mode: position 은 limits.range 필수 — 관절 가동범위는 비대칭이라 "
                   "max_abs 로 표현할 수 없다";
            return false;
        }
        // current 는 전역 클램프와 **더 좁은 쪽**을 취한다 (§4.3 규칙 3, 현행 유지)
        if (mode_ == Mode::CURRENT) {
            limit_lo = std::max(limit_lo, -global_max_current);
            limit_hi = std::min(limit_hi,  global_max_current);
        }

        if (lim && lim["slew_rate"]) {
            slew_rate = lim["slew_rate"].as<double>(0.0);
            has_slew  = slew_rate > 0.0;
        }
    }
    limit_lo_ = limit_lo;
    limit_hi_ = limit_hi;
    // 기존 대칭 클램프 경로가 쓰는 값 (아래 코드 호환)

    // seed 미지정 시 시각 기반 (재현성을 위해 result 에 기록되어야 함 — C-4b action result)
    const uint64_t seed = root["seed"]
        ? root["seed"].as<uint64_t>(42)
        : static_cast<uint64_t>(std::chrono::steady_clock::now().time_since_epoch().count());
    seed_ = seed;   // **반드시 남긴다** — 시각 기반 시드는 여기서 안 붙잡으면 복원 불가능하다

    // 모터별 사전 샘플링
    std::vector<uint16_t> seg_of_motor[4];
    std::vector<bool>     noise_of_motor[4];   // E3 — noise 구간 표식 (slew 면제 대상)
    int    ref_motor   = -1;    // 세그먼트 수가 가장 많은 모터 (피드백 segment_index 기준)
    size_t ref_segs    = 0;

    for (const auto& kv : root["motors"]) {
        const std::string key = kv.first.as<std::string>("");
        // 키 형식 "m<번호>" — 오타가 조용히 무시되면 안 되므로 엄격히 본다.
        if (key.size() < 2 || key[0] != 'm') {
            *err = "motors 키 '" + key + "' 형식 오류 — 'm1'~'m4' 여야 함";
            return false;
        }
        int motor_no = 0;
        try {
            motor_no = std::stoi(key.substr(1));
        } catch (const std::exception&) {
            *err = "motors 키 '" + key + "' 의 모터 번호를 읽을 수 없음";
            return false;
        }
        if (motor_no < 1 || motor_no > 4) {
            *err = "motors 키 '" + key + "': 모터 번호는 1~4";
            return false;
        }
        // §4.3 규칙 1 — 지정 모터 ⊆ active_motors
        if ((active_mask & (1u << (motor_no - 1))) == 0) {
            *err = "m" + std::to_string(motor_no) + " 는 active_motors 에 없음 — goal 거부 (§4.3-1)";
            return false;
        }
        if (!kv.second.IsSequence()) {
            *err = "m" + std::to_string(motor_no) + " 의 값이 세그먼트 배열이 아님";
            return false;
        }

        std::mt19937_64 rng(seed + static_cast<uint64_t>(motor_no));  // 모터별 독립 스트림, 시드 재현성 유지
        std::vector<float>    samples;
        std::vector<uint16_t> segs;
        std::vector<bool>     noise_mask;   // E3 — slew 검사에서 건너뛸 tick
        int seg_idx = 0;
        for (const auto& seg : kv.second) {
            const size_t before = samples.size();
            bool seg_is_noise = false;
            if (!ExpandSegment(seg, motor_no, seg_idx, rng, &samples, err,
                               limit_lo_, limit_hi_, &seg_is_noise)) return false;
            segs.resize(samples.size(), static_cast<uint16_t>(seg_idx));
            noise_mask.resize(samples.size(), seg_is_noise);
            for (size_t t = before; t < samples.size(); t++) noise_mask[t] = seg_is_noise;
            if (samples.size() == before) {
                *err = "m" + std::to_string(motor_no) + " 세그먼트[" + std::to_string(seg_idx) +
                       "]: 길이가 0 tick — duration 이 너무 짧음 (최소 " +
                       std::to_string(1.0 / kTickHz) + "s)";
                return false;
            }
            seg_idx++;
        }
        if (samples.empty()) {
            *err = "m" + std::to_string(motor_no) + ": 세그먼트가 하나도 없음";
            return false;
        }
        if (samples.size() * kDt > kMaxDuration) {
            *err = "m" + std::to_string(motor_no) + ": 총 길이가 상한 " +
                   std::to_string(static_cast<int>(kMaxDuration)) + "s 초과";
            return false;
        }

        const int mi = motor_no - 1;
        samples_[mi]     = std::move(samples);
        seg_of_motor[mi] = std::move(segs);
        noise_of_motor[mi] = std::move(noise_mask);
        if (static_cast<size_t>(seg_idx) > ref_segs) {   // 동수면 먼저 온(번호 작은) 모터 유지
            ref_segs  = static_cast<size_t>(seg_idx);
            ref_motor = mi;
        }
    }

    // §4.3 규칙 2 — 전체 길이 = 가장 긴 모터. 짧은 쪽은 0 으로 패딩,
    // 미지정 활성 모터는 전 구간 0. (미지정 = "끄고 싶다" 가 아니라 "0 을 유지" 라는 뜻)
    for (int i = 0; i < 4; i++) tick_count_ = std::max(tick_count_, samples_[i].size());
    if (tick_count_ == 0) {
        *err = "프로파일 길이가 0 — 유효한 세그먼트가 없음";
        return false;
    }
    for (int i = 0; i < 4; i++) {
        if ((active_mask & (1u << i)) == 0) { samples_[i].assign(tick_count_, 0.0f); continue; }
        samples_[i].resize(tick_count_, 0.0f);
    }

    // 피드백 segment_index — 기준 모터의 것을 쓰고, 패딩 구간은 마지막 세그 번호를 유지.
    segment_index_.assign(tick_count_, 0);
    if (ref_motor >= 0) {
        const auto& src = seg_of_motor[ref_motor];
        for (size_t t = 0; t < tick_count_; t++) {
            segment_index_[t] = (t < src.size()) ? src[t]
                                                 : (src.empty() ? 0 : src.back());
        }
    }

    // §4.3 규칙 3 — 전 샘플 클램프 (횟수 기록)
    for (int i = 0; i < 4; i++) {
        for (float& v : samples_[i]) {
            // **비대칭 클램프** (05 §2.4). position 의 관절 가동범위는 대칭이 아니다 —
            // ±max_abs 로 자르면 한쪽이 실제 한계보다 넓거나 좁아진다.
            if (v > limit_hi_) { v = limit_hi_; clamp_cnt_++; }
            if (v < limit_lo_) { v = limit_lo_; clamp_cnt_++; }
        }
    }

    // §4.3 규칙 4 — slew_rate 위반은 '성형' 이 아니라 reject.
    // 프로파일이 곧 실험 기록이므로 몰래 고쳐 재생하면 기록과 실제가 어긋난다.
    if (has_slew) {
        const double max_delta = slew_rate * kDt;
        for (int i = 0; i < 4; i++) {
            const auto& nz = noise_of_motor[i];
            for (size_t t = 1; t < samples_[i].size(); t++) {
                // E3 (05 §3.4) — **noise 구간은 건너뛴다.**
                // 백색 가우시안 잡음은 정의상 tick 간 델타가 무제한이라, std 가 아무리 작아도
                // 긴 프로파일 어딘가는 반드시 한계를 넘는다. 두 규칙을 함께 지키면
                // **noise 세그먼트는 사실상 항상 거부된다** — 2026-07-29 실기에서 실제로 그랬다.
                // `noise` 를 쓴다는 것 자체가 "이 구간은 레이트 무제한" 이라는 선언이다.
                // 경계 tick(직전이 noise)도 제외한다 — 그 델타 역시 잡음이 만든 것이다.
                // 진폭 쪽은 위 4σ 가드가 대신 지킨다.
                const bool exempt = (t < nz.size() && (nz[t] || nz[t - 1]));
                if (exempt) { slew_exempt_ticks_++; continue; }
                const double d = std::fabs(static_cast<double>(samples_[i][t]) - samples_[i][t - 1]);
                if (d > max_delta + 1e-9) {
                    std::ostringstream os;
                    os << "m" << (i + 1) << " tick " << t << " (t=" << (t * kDt)
                       << "s): slew_rate 위반 — " << (d / kDt) << " A/s > 한계 " << slew_rate
                       << " A/s (§4.3-4: 자동 성형 없이 거부)";
                    *err = os.str();
                    return false;
                }
            }
        }
    }

    return true;
}

bool RdProfile::SampleAt(size_t tick, float out_current[4], uint16_t* out_segment_index) const {
    if (tick >= tick_count_) return false;
    for (int i = 0; i < 4; i++) out_current[i] = samples_[i][tick];
    if (out_segment_index) *out_segment_index = segment_index_[tick];
    return true;
}

} // namespace orin_bridge
