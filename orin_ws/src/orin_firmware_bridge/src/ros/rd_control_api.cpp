#include "orin_firmware_bridge/ros/rd_control_api.hpp"

#include "orin_firmware_bridge/policy/rd_status.hpp"

#include <chrono>
#include <cmath>

namespace orin_bridge {

RdControlApi::RdControlApi(rclcpp::Node* node, RobotState_t* state, BridgeConfig& cfg,
                           RdControl* control, RdProfilePlayer* player, RdOos* oos,
                           std::function<RunCounters_t()> counters,
                           std::function<bool(uint8_t, std::string*)> set_read_preset)
    : node_(node), state_(state), cfg_(cfg), control_(control), player_(player), oos_(oos),
      counters_(std::move(counters)), set_read_preset_(std::move(set_read_preset)) {
    // 구독은 **항상** 연다. arm=off 면 받아서 버린다 (01 §6.1.3) — 구독 자체를 늦게 열면
    // arm 을 켠 직후 첫 메시지를 놓쳐 STREAM 진입이 한 박자 늦는다.
    sub_cmd_motor_ = node_->create_subscription<mgs_tp_msgs::msg::CmdMotor>(
        "/carrier/control/cmd_motor", 10,
        std::bind(&RdControlApi::CallbackCmdMotor, this, std::placeholders::_1));
}

// 스테일 판정 기준은 **메시지의 header.stamp** 가 아니라 수신 시각이다.
// 발행자 시계가 틀어져 있으면(웹 브라우저·다른 호스트) stamp 기반 판정이 통째로 무의미해진다.
// "언제 만들어졌나" 가 아니라 "우리가 언제 받았나" 가 워치독의 질문이다.
void RdControlApi::CallbackCmdMotor(const mgs_tp_msgs::msg::CmdMotor::SharedPtr msg) {
    StreamCmd_t c;
    // CmdMotor.ctr_mode 는 **예약 필드다 — 여기서 담기만 하고 아무도 읽지 않는다.**
    // 발행자가 그걸 모르고 채워 보내면 "모드를 지정했는데 안 먹는다" 가 되므로 한 번은 말해 준다.
    // (읽게 만들지 않는 이유는 CmdMotor.msg 주석 참조: 200Hz 스트림이 제어 모드를 바꾸면
    //  회전 중 모드 전환이 가능해져 safe_stop 게이트가 무의미해진다.)
    for (int i = 0; i < 4 && !warned_ctr_mode_ignored_; i++) {
        if (msg->ctr_mode[i] == 0) continue;
        warned_ctr_mode_ignored_ = true;
        RCLCPP_WARN(node_->get_logger(),
            "CmdMotor.ctr_mode 는 예약 필드로 무시된다 — 제어 모드는 "
            "`control_cli config ctr_mode <모터> <값>` (IDLE 전용) 으로 설정할 것. "
            "이 경고는 1회만 표시한다");
    }
    for (int i = 0; i < 4; i++) {
        c.ctr_mode[i] = msg->ctr_mode[i];
        c.position[i] = msg->position[i];
        c.velocity[i] = msg->velocity[i];
        c.current[i]  = msg->current[i];
    }
    c.stamp = node_->now().seconds();
    control_->PushStreamCommand(c);
}

bool RdControlApi::DoSetAutoMode(uint8_t mode, std::string* msg) {
    const uint8_t cur = control_->AutoMode();
    if (cur == mode) {
        *msg = "이미 auto_mode=" + std::to_string(mode) + "(" + ecu::AutoModeName(mode) +
               "), write " + ecu::AutoModeWriteSpan(mode);
        return true;
    }

    if (mode == ecu::AUTO_MODE_DIRECT) {
        // ① shadow 소독 — DIRECT 에서는 ECU 가 손대지 않으므로 bridge 가 쓴 값이 곧 명령이 된다.
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            for (int i = 0; i < 4; i++) control_->SetCtrMode(i, ecu::CTR_MODE_CURRENT);
        }
        {
            std::lock_guard<std::mutex> lock(state_->state_mutex);
            auto& cm = state_->ecu.reg.cmd_motor;
            for (int i = 0; i < 4; i++) {
                cm.ctr_mode[i]     = ecu::CTR_MODE_CURRENT;
                cm.cmd_position[i] = 0.0f;
                cm.cmd_velocity[i] = 0.0f;
            }
        }
        // ② WRITE+검증
        if (!DoOutOfSpanWrite(ecu::REG_AUTO_MODE_OFFSET, mode, msg)) return false;
        // ③ 범위 확장 (검증 성공 후에만)
        control_->SetAutoMode(mode);
        *msg += " / auto_mode=2(DIRECT), write 128:52 (shadow 소독 완료)";
        RCLCPP_WARN(node_->get_logger(),
            "auto_mode 1(CURRENT) -> 2(DIRECT): shadow 소독 후 write 범위 128:52 로 확장 — "
            "이제 ctr_mode 는 bridge 소유 (ECU 미개입)");
        return true;
    }

    // DIRECT 이외로: 먼저 ECU 모드를 바꾼 뒤 범위를 줄인다.
    // (CURRENT 뿐 아니라 VELOCITY(4)/POSITION(5) 도 이 경로다 — 전부 ECU 가 ctr_mode 를 강제한다.)
    if (!DoOutOfSpanWrite(ecu::REG_AUTO_MODE_OFFSET, mode, msg)) return false;
    control_->SetAutoMode(mode);
    {   // ECU 가 강제하는 ctr_mode 로 섀도를 맞춘다 — 모드마다 다르다 (read-back 혼선 방지)
        const uint8_t forced = ecu::AutoModeForcedCtrMode(mode);
        std::lock_guard<std::mutex> lock(data_mutex_);
        for (int i = 0; i < 4; i++) control_->SetCtrMode(i, forced);
    }
    *msg += std::string(" / auto_mode=") + std::to_string(mode) + "(" + ecu::AutoModeName(mode) +
            "), write " + ecu::AutoModeWriteSpan(mode);
    RCLCPP_WARN(node_->get_logger(),
        "auto_mode %u(%s) -> %u(%s): write 범위 %s — ctr_mode 는 ECU 가 %u 로 강제",
        cur, ecu::AutoModeName(cur), mode, ecu::AutoModeName(mode),
        ecu::AutoModeWriteSpan(mode), ecu::AutoModeForcedCtrMode(mode));
    return true;
}

// in-span(ctr_mode 128~131): 섀도만 바꾸고 다음 RW tick 이 자연 반영 — 패킷 삽입 없음 (§2 표).
// 검증은 같은 트랜잭션의 read 세그({128,4})로 돌아온 값을 본다.
bool RdControlApi::DoInSpanCtrMode(const std::vector<uint8_t>& motors, uint8_t mode, std::string* msg) {
    {
        std::lock_guard<std::mutex> lock(data_mutex_);
        for (uint8_t m : motors) control_->SetCtrMode(m - 1, mode);
    }
    // 다음 tick 이 write 하고 그 응답의 read 세그가 섀도를 덮을 때까지 기다린다.
    for (int i = 0; i < RdOos::kVerifyTicks; i++) {
        std::this_thread::sleep_for(std::chrono::milliseconds(5));   // 1 tick
        bool all_ok = true;
        {
            std::lock_guard<std::mutex> lock(state_->state_mutex);
            for (uint8_t m : motors) {
                if (state_->ecu.reg.cmd_motor.ctr_mode[m - 1] != mode) { all_ok = false; break; }
            }
        }
        if (all_ok) {
            *msg = "ctr_mode = " + std::to_string(mode) + " 검증 OK (" +
                   std::to_string(i + 1) + " tick)";
            return true;
        }
    }
    *msg = "ctr_mode 검증 시간초과 — " + std::to_string(RdOos::kVerifyTicks) + " tick 내 read-back 미확인";
    return false;
}

// ===== §3.3 config service =====

void RdControlApi::CallbackControlConfig(
    const std::shared_ptr<mgs_tp_msgs::srv::ControlConfig::Request> req,
    std::shared_ptr<mgs_tp_msgs::srv::ControlConfig::Response> res) {
    using Req = mgs_tp_msgs::srv::ControlConfig::Request;

    const ControlState st = control_->State();

    // GET_STATUS 는 부작용이 없으므로 상태와 무관하게 허용한다 —
    // RUNNING 중에도 CLI/웹이 상태를 물을 수 있어야 한다 (§2 표의 '허용 입력'은
    // 상태를 바꾸는 입력에 대한 규정으로 해석).
    if (req->op == Req::OP_GET_STATUS) {
        // 04 §4 (C3) — **정형 JSON.** 종전에는 사람이 읽는 한국어 문장을 돌려줬고,
        // control_web 이 그걸 정규식으로 뜯어 쓰고 있었다 (06 §9.1). 스키마가 계약이다.
        const RunCounters_t c = counters_();
        StatusSnapshot_t snap;

        snap.bridge_mode   = BridgeModeName(cfg_.bridge_mode);
        snap.control_state = ControlStateStr(st);
        snap.read_preset   = c.read_preset_name;

        const uint8_t am = control_->AutoMode();
        snap.auto_mode  = AutoModeJsonName(am);
        snap.write_span = ecu::AutoModeWriteSpan(am);

        const ControlWrite_t w = control_->SelectWrite();
        snap.write_source = WriteSourceName(st == ControlState::RUNNING ? 2u
                                          : st == ControlState::STREAM  ? 3u : 0u);
        snap.goal_id      = w.goal_id;
        snap.profile_time = w.profile_time;

        {
            std::lock_guard<std::mutex> lock(state_->state_mutex);
            snap.ecu_sys_state = state_->ecu.reg.sys.sys_state;
            snap.ecu_mode      = EcuModeName(state_->ecu.reg.cmd_system.mode);
        }

        // **요청한 집합**이다 (기동 파라미터). ECU 실값이 아니다 —
        // 섀도의 cmd_system.motor_mask(192)는 **어느 읽기 프리셋도 읽지 않아** write 흔적만
        // 남는 자리라, 그걸 "실값" 이라고 내보내면 manual(브리지가 안 씀)에서 0 이 나오고
        // 그건 "모른다" 가 아니라 "마스크가 0" 으로 읽힌다.
        // control 에서는 INIT 이 write+검증하므로 요청값 = 실값이다.
        snap.motor_mask = cfg_.active_motor_mask;
        for (int i = 0; i < 4; i++)
            if ((cfg_.active_motor_mask >> i) & 0x01) snap.active_motors.push_back(i + 1);
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            for (int i = 0; i < 4; i++) snap.ctr_mode.emplace_back(CtrModeName(control_->CtrMode(i)));
        }

        // safe_stop 은 **사유까지** 준다. 웹이 origin 버튼을 왜 못 누르는지 보여주려면
        // 참/거짓만으로는 부족하다 (04 §4: safe_stop=false 일 때만 detail).
        std::string why;
        snap.safe_stop = control_->SafeStop(&why);
        if (!snap.safe_stop) snap.safe_stop_detail = why;

        // 미수렴이면 header.stamp 가 fallback 이라 품질·드리프트는 **의미가 없다** → null.
        snap.stamp_valid = c.clock_converged;
        if (c.clock_converged) {
            snap.stamp_quality_ms = c.stamp_quality_ms * 1000.0;
            snap.drift_ppm        = c.drift_ppm;
        }
        if (c.rtt_ms > 0.0) snap.rtt_ms = c.rtt_ms * 1000.0;
        snap.rw_err   = c.rw_err;
        snap.drop_cnt = static_cast<uint32_t>(c.drop_cnt);

        if (st == ControlState::LOCKED) snap.lock_reason = control_->LockReason();

        if (slot_snapshot_) {
            int idx = 0;
            for (const auto& v : slot_snapshot_()) {
                StatusSlot_t sl;
                sl.index  = idx++;
                sl.active = v.active;
                if (v.active) {
                    sl.cmd      = CmdInstName(v.inst);
                    sl.target   = TargetName(v.target_id);
                    sl.duration = v.duration;
                    sl.attempts = v.attempts;
                }
                snap.slots.push_back(std::move(sl));
            }
        }

        res->ok = true;
        res->message = StatusToJson(snap);
        return;
    }

    // 로케일 소수점(",")이 JSON 을 깨뜨리므로 snprintf 로 고정한다 (rd_status.cpp 와 같은 이유).
    auto Fmt3 = [](double v) {
        char b[32]; std::snprintf(b, sizeof(b), "%.3f", v); return std::string(b);
    };
    if (req->op == Req::OP_GET_REGISTERS) {
        // 07 §2 Tab3 — 섀도 덤프 + **신선도**. 부작용이 없으므로 GET_STATUS 와 같이
        // FSM 게이트 앞에 둔다 (RUNNING 중에도 표를 볼 수 있어야 한다).
        const uint8_t tid = req->value ? static_cast<uint8_t>(req->value) : TARGET::ECU;
        uint16_t total = 0;
        uint8_t* base = ShadowBase(state_, tid, &total);
        if (!base) {
            res->ok = false;
            res->message = "알 수 없는 target_id";
            return;
        }
        std::vector<uint8_t> buf(total);
        {
            std::lock_guard<std::mutex> lock(state_->state_mutex);
            std::memcpy(buf.data(), base, total);
        }

        // ── 어느 바이트가 신선한가 ────────────────────────────────────────
        // 두 출처의 합집합이다:
        //   ① 200Hz 루프가 매 tick 읽는 **현재 프리셋** (ECU 만)
        //   ② 마지막으로 성공한 **슬롯 READ** (CMD_READ_* / raw_read)
        // 여기 안 들어가는 자리는 섀도에 값이 있어도 **읽은 적이 없거나 낡은 것**이다.
        std::ostringstream fresh;
        bool first = true;
        auto emit = [&](uint16_t a, uint16_t l) {
            if (!l) return;
            if (!first) fresh << ",";
            fresh << "[" << a << "," << l << "]";
            first = false;
        };
        double age = -1.0;
        if (tid == TARGET::ECU && preset_ptr_) {
            const ReadPreset* p = preset_ptr_();
            if (p) for (uint8_t i = 0; i < p->count; i++) emit(p->spans[i].addr, p->spans[i].len);
        }
        if (last_read_) {
            const auto lr = last_read_();
            if (lr.valid && lr.target_id == tid) {
                for (uint8_t i = 0; i < lr.count; i++) emit(lr.spans[i].addr, lr.spans[i].len);
                age = lr.age_s;
            }
        }

        std::ostringstream hex;
        hex << std::hex << std::setfill('0');
        for (uint8_t b : buf) hex << std::setw(2) << static_cast<int>(b);

        std::ostringstream js;
        js << "{\"target\":\"" << TargetName(tid) << "\",\"total\":" << total
           << ",\"fresh\":[" << fresh.str() << "]"
           << ",\"read_age_s\":" << (age < 0 ? std::string("null") : Fmt3(age))
           << ",\"bytes\":\"" << hex.str() << "\"}";
        res->ok = true;
        res->message = js.str();
        return;
    }

    // REARM 은 LOCKED 전용 — 나머지는 IDLE 전용 (§2 표)
    const bool is_rearm = (req->op == Req::OP_REARM);

    // ⚠ **감속 방향 조작은 FSM 게이트를 통과시킨다** (01 §6.2).
    //   arm off = 웹의 stop 버튼이다. STREAM 에서 config 를 통째로 막으면 **stop 이 안 눌린다** —
    //   달리는 중에 멈추는 수단을 잠그는 셈이라 게이트의 목적과 정반대가 된다.
    //   (arm **on** 은 설정 변경이므로 아래 게이트를 그대로 받는다.)
    const bool is_disarm = (req->op == Req::OP_SET_STREAM_ARM && req->value == 0);

    if (!is_disarm && !control_->AcceptsConfig(is_rearm)) {
        res->ok = false;
        res->message = std::string("거부 — FSM 이 ") + ControlStateStr(st) +
                       (st == ControlState::RUNNING
                            ? " (RUNNING 중 설정 변경 불가, 중단은 action cancel)"
                        : st == ControlState::LOCKED
                            ? " (REARM 만 허용)"
                            : " (IDLE 에서만 허용)");
        return;
    }

    if (is_rearm) {
        res->ok = control_->Rearm();
        res->message = res->ok ? "REARM 완료 — IDLE" : "REARM 불가 (LOCKED 아님)";
        return;
    }

    switch (req->op) {
        case Req::OP_SET_ACTIVE_MOTORS: {
            uint8_t mask = 0;
            for (uint8_t m : req->motors) {
                if (m < 1 || m > 4) {
                    res->ok = false;
                    res->message = "모터 번호 " + std::to_string(m) + " 는 범위 밖 (1~4)";
                    return;
                }
                mask |= static_cast<uint8_t>(1u << (m - 1));
            }
            if (mask == 0) {
                res->ok = false;
                res->message = "활성 모터가 비어 있음 — 최소 1개 필요";
                return;
            }
            std::string msg;
            res->ok = DoOutOfSpanWrite(ecu::REG_MOTOR_MASK_OFFSET, mask, &msg);
            res->message = msg;
            if (res->ok) {
                cfg_.active_motor_mask = mask;   // 이후 프로파일 검증(§4.3-1)이 이 값을 쓴다
                RCLCPP_WARN(node_->get_logger(), "active_motors 변경 — mask=0x%02X", mask);
            }
            return;
        }
        case Req::OP_SET_CTR_MODE: {
            // §2.6: auto_mode=1(CURRENT)에서 ctr_mode 는 ECU 소유다 — bridge write 범위(164:16)
            // 밖이라 wire 로 나가지도 않고, ECU 가 100Hz 로 CURRENT 를 강제한다.
            // 조용히 10 tick 타임아웃으로 실패하느니 이유를 분명히 밝히고 거부한다.
            if (control_->AutoMode() != ecu::AUTO_MODE_DIRECT) {
                res->ok = false;
                res->message = "SET_CTR_MODE 는 auto_mode=2(DIRECT) 에서만 유효 — "
                               "현재 1(CURRENT) 에서는 ECU 가 ctr_mode 를 CURRENT 로 강제한다 "
                               "(먼저 SET_AUTO_MODE=2 필요)";
                return;
            }
            const int mode = req->value;
            // §3.3: 0=ESTOP / 1=CURRENT / 2=CURRENT_BRAKE / 3=VELOCITY / 4=POSITION.
            // 5~7(SET_ORIGIN/POS_VEL/MIT)은 거부 — 제어에서 쓸 일이 없고 오조작 위험.
            if (mode < 0 || mode > 4) {
                res->ok = false;
                res->message = "ctr_mode " + std::to_string(mode) +
                               " 거부 — 허용 0=ESTOP/1=CURRENT/2=CURRENT_BRAKE/3=VELOCITY/4=POSITION";
                return;
            }
            if (req->motors.empty()) {
                res->ok = false;
                res->message = "대상 모터가 비어 있음";
                return;
            }
            std::vector<uint8_t> motors;
            for (uint8_t m : req->motors) {
                if (m < 1 || m > 4) {
                    res->ok = false;
                    res->message = "모터 번호 " + std::to_string(m) + " 는 범위 밖 (1~4)";
                    return;
                }
                // 비활성 모터 지정 거부 — 메모의 "비활성 모터는 체크 불가" 규칙 (§3.3)
                if ((cfg_.active_motor_mask & (1u << (m - 1))) == 0) {
                    res->ok = false;
                    res->message = "m" + std::to_string(m) + " 는 비활성 모터 — 거부";
                    return;
                }
                motors.push_back(m);
            }
            std::string msg;
            res->ok = DoInSpanCtrMode(motors, static_cast<uint8_t>(mode), &msg);
            res->message = msg;
            return;
        }
        case Req::OP_SET_MODE: {
            if (req->value != 0 && req->value != 1) {
                res->ok = false;
                res->message = "mode 는 0(MANUAL) 또는 1(AUTO)";
                return;
            }
            std::string msg;
            res->ok = DoOutOfSpanWrite(ecu::REG_MODE_OFFSET,
                                       static_cast<uint8_t>(req->value), &msg);
            res->message = msg;
            return;
        }
        case Req::OP_SET_AUTO_MODE: {
            // 01 부록: 거부는 0(KINEMATIC — ECU 가 ctr_mode 덮어씀)과 3(CONTROL — 미구현) 뿐.
            //   4/5 는 2026-07-28 실기 확인 (06 §4.7).
            const bool am_ok = (req->value == ecu::AUTO_MODE_CURRENT  ||
                                req->value == ecu::AUTO_MODE_DIRECT   ||
                                req->value == ecu::AUTO_MODE_VELOCITY ||
                                req->value == ecu::AUTO_MODE_POSITION);
            if (!am_ok) {
                res->ok = false;
                res->message = "auto_mode " + std::to_string(req->value) +
                    " 거부 — 1(CURRENT)/2(DIRECT)/4(VELOCITY)/5(POSITION) 만 허용 " +
                    (req->value == ecu::AUTO_MODE_KINEMATIC
                        ? "(KINEMATIC 은 ECU 가 ctr_mode 를 VELOCITY 로 덮어써 전류 실험 불가)"
                        : "(CONTROL 은 ECU 미구현 — motor off)");
                return;
            }
            std::string msg;
            res->ok = DoSetAutoMode(static_cast<uint8_t>(req->value), &msg);
            res->message = msg;
            return;
        }
        case Req::OP_SET_READ_PRESET: {
            // 04 §2.4.2 — "정기 센서 변경 = 읽기 프리셋 교체". 슬롯을 쓰지 않는 경로다
            // (tick 소모 0). control 은 커맨드 슬롯이 0개라 이것이 유일한 수단이다.
            //
            // IDLE 판정은 위 게이트가 이미 했다. RUNNING 중 교체를 막는 이유는 안전이 아니라
            // **데이터**다 — 발행 메시지의 필드 구성이 중간에 바뀌면 실험 시계열이 갈라진다.
            if (!set_read_preset_) {
                res->ok = false;
                res->message = "프리셋 교체 경로가 연결되지 않았다 (control 모드로 기동할 것)";
                return;
            }
            std::string why;
            res->ok = set_read_preset_(static_cast<uint8_t>(req->value), &why);
            res->message = why;
            if (res->ok) {
                RCLCPP_WARN(node_->get_logger(),
                    "읽기 프리셋 교체 — %s. **이 프리셋이 안 읽는 필드는 미판독(0xFF/NaN)으로 "
                    "발행된다** (낡은 값을 신선한 값처럼 내보내지 않기 위함)", why.c_str());
            }
            return;
        }
        case Req::OP_SET_ORIGIN: {
            // 01 §6.3 C-1 — 레벨 레지스터(ctr_mode)에 실린 엣지 명령. 1 tick 만 나가고 원복된다.
            // 자동화하지 않는다: auto_mode=POSITION 진입 시 자동 원점은 기각됐다 (실험마다
            // 기준이 달라져 기록 간 비교가 깨진다). 사람이 pose 를 보고 누르는 동작이다.
            std::string why;
            res->ok = control_->RequestOriginPulse(&why);
            res->message = why;
            return;
        }
        case Req::OP_SET_STREAM_ARM: {
            // 01 §6.1.3 — 웹의 run/stop 버튼. STREAM 진입을 **명시적 조작**으로 만든다.
            const bool on = (req->value != 0);
            std::string why;
            res->ok = control_->SetStreamArm(on, &why);
            res->message = why;
            return;
        }
        default:
            res->ok = false;
            res->message = "알 수 없는 op " + std::to_string(req->op);
            return;
    }
}

// ===== §3.2 run_profile action =====

// 조작 입구. **control 구성이면 무조건 열린다** — 쓰든 안 쓰든 상태는 읽어야 한다.
// 두 번 불려도 안전하다 (재생성하지 않는다).
void RdControlApi::StartConfigService() {
    if (srv_control_config_) return;
    // config service 는 검증 대기(최대 50ms) 동안 블록되므로 전용 콜백 그룹에 둔다 —
    // 기본 그룹(MutuallyExclusive)에 두면 그 사이 action 피드백·취소가 막힌다.
    cb_group_config_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    srv_control_config_ = node_->create_service<mgs_tp_msgs::srv::ControlConfig>(
        "/carrier/control/config",
        std::bind(&RdControlApi::CallbackControlConfig, this, std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default, cb_group_config_);
    RCLCPP_INFO(node_->get_logger(), "config service 준비됨: /carrier/control/config");
}

void RdControlApi::StartProfileServer() {
    // 조작 입구가 먼저 있어야 한다 — 재생 중 상태를 못 읽는 구성은 만들지 않는다.
    StartConfigService();
    if (action_server_) return;
    action_server_ = rclcpp_action::create_server<RunProfile>(
        node_, "/carrier/control/run_profile",
        std::bind(&RdControlApi::HandleProfileGoal, this, std::placeholders::_1, std::placeholders::_2),
        std::bind(&RdControlApi::HandleProfileCancel, this, std::placeholders::_1),
        std::bind(&RdControlApi::HandleProfileAccepted, this, std::placeholders::_1));

    RCLCPP_INFO(node_->get_logger(),
        "run_profile action 준비됨: /carrier/control/run_profile");
}

// goal 수락 판정: 파싱 → 검증 → 사전 샘플링까지 여기서 끝낸다 (§3.2).
// 실패는 전부 reject — RUNNING 에 들어간 뒤 죽는 것보다 낫다.
rclcpp_action::GoalResponse RdControlApi::HandleProfileGoal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const RunProfile::Goal> goal) {
    (void)uuid;

    const ControlState st = control_->State();
    if (st != ControlState::IDLE) {
        // RUNNING 중 새 goal 거부 = 기존 실험 오염 방지. LOCKED 면 REARM 이 먼저다.
        RCLCPP_WARN(node_->get_logger(),
            "run_profile 거부 — FSM 이 %s (IDLE 에서만 수락)", ControlStateStr(st));
        return rclcpp_action::GoalResponse::REJECT;
    }

    // 05 §2.3 수락 규칙 — **판정 기준을 ctr_mode 에서 auto_mode 로 한 단계 올린다** (01 §7).
    //
    // 구 가드는 "활성 모터 전부 ctr_mode==CURRENT" 를 요구했다. v1 player 가 전류 전용이던
    // 시절의 규칙이며, mode: velocity/position 프로파일을 통째로 막는다.
    // 새 규칙: auto_mode 가 프로파일 mode 의 native 값이거나 DIRECT 면 통과.
    //   DIRECT 일 때만 ctr_mode 까지 내려간다 — 그때는 ctr_mode 가 bridge 소유라 신뢰 가능하다.
    //
    // ⚠ 순서 주의: **프로파일을 먼저 파싱**해야 mode 를 알 수 있으므로 Load 가 이 검사보다
    //   앞선다. 구 코드는 반대였고, 그래서 프로파일 내용과 무관한 고정 규칙일 수밖에 없었다.
    std::string err;
    if (!player_->Load(goal->profile_yaml, cfg_.active_motor_mask, &err)) {
        RCLCPP_ERROR(node_->get_logger(), "run_profile 거부 — 프로파일 오류: %s", err.c_str());
        return rclcpp_action::GoalResponse::REJECT;
    }
    {
        uint8_t ctr[4];
        {
            std::lock_guard<std::mutex> lock(state_->state_mutex);
            for (int i = 0; i < 4; i++) ctr[i] = state_->ecu.reg.cmd_motor.ctr_mode[i];
        }
        std::string why;
        if (!player_->AcceptsAutoMode(control_->AutoMode(), ctr, cfg_.active_motor_mask, &why)) {
            RCLCPP_ERROR(node_->get_logger(), "run_profile 거부 — %s", why.c_str());
            return rclcpp_action::GoalResponse::REJECT;
        }
        float lo = 0.0f, hi = 0.0f;
        player_->EffectiveLimits(&lo, &hi);
        // 재생 중 wire 직전 클램프를 **프로파일이 정한 실효 한계**로 바꾼다 (05 §2.4).
        // 구 코드는 무조건 cmd_current_max([A])를 썼다 — position 프로파일이 각도째 잘렸다.
        control_->SetProfileLimits(lo, hi);
        RCLCPP_INFO(node_->get_logger(),
            "run_profile 수락 — mode=%s limits=[%.3f, %.3f] auto_mode=%u",
            player_->ModeStr(), lo, hi, control_->AutoMode());
    }

    RCLCPP_INFO(node_->get_logger(),
        "run_profile 수락 — name='%s' 길이 %.2fs (%zu tick), 클램프 %u 회",
        goal->name.empty() ? player_->Name().c_str() : goal->name.c_str(),
        player_->DurationSec(), player_->TickCount(), player_->ClampCount());
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse RdControlApi::HandleProfileCancel(const std::shared_ptr<GoalHandle> gh) {
    (void)gh;
    RCLCPP_WARN(node_->get_logger(), "run_profile 취소 요청 — 즉시 0A 후 IDLE");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void RdControlApi::HandleProfileAccepted(const std::shared_ptr<GoalHandle> gh) {
    // 실행은 별도 스레드 — 여기서 블록하면 executor 가 막혀 cancel 도 못 받는다.
    std::thread{std::bind(&RdControlApi::ExecuteProfile, this, gh)}.detach();
}

void RdControlApi::ExecuteProfile(const std::shared_ptr<GoalHandle> gh) {
    auto result   = std::make_shared<RunProfile::Result>();
    auto feedback = std::make_shared<RunProfile::Feedback>();

    uint32_t goal_id = 0;
    size_t   total_ticks = 0;
    const RunCounters_t c_at_start = counters_();

    {
        goal_id           = player_->Begin();
        total_ticks       = player_->TickCount();
        result->clamp_cnt = player_->ClampCount();
    }

    // FSM 전이는 여기서 — 실패하면(그 사이 LOCKED 등) 재생하지 않는다.
    if (!control_->BeginProfile(goal_id)) {
        result->success = false;
        result->message = "FSM 이 IDLE 이 아니라 시작 불가 (현재: " +
                          std::string(ControlStateStr(control_->State())) + ")";
        result->goal_id = goal_id;
        gh->abort(result);
        return;
    }
    {
        player_->Activate();   // 이 시점부터 스케줄 tick 이 샘플을 소비한다
    }

    // 5Hz 피드백 루프 — 재생 자체는 스케줄 스레드(TickProfile)가 200Hz 로 돌린다.
    rclcpp::Rate rate(5.0);
    bool canceled = false;
    while (rclcpp::ok()) {
        size_t tick; bool done;
        {
            player_->Progress(&tick, &done);
        }

        if (gh->is_canceling()) { canceled = true; break; }
        if (done || tick >= total_ticks) break;
        // 재생 중 LOCKED 로 떨어지면(write 연속 거부 등) 더 돌릴 이유가 없다.
        if (control_->State() != ControlState::RUNNING) break;

        feedback->t              = static_cast<float>(tick * RdProfile::kDt);
        feedback->segment_index  = 0;
        feedback->progress       = total_ticks ? static_cast<float>(tick) / total_ticks : 0.0f;
        {
            float c[4]; uint16_t seg = 0;
            if (player_->SampleAt(tick, c, &seg)) feedback->segment_index = seg;
        }
        gh->publish_feedback(feedback);
        rate.sleep();
    }

    // 정리: 재생 중지 → FSM IDLE 복귀 (write 소스가 0A 로 돌아간다)
    size_t executed;
    const ControlState end_state = control_->State();
    {
        executed = player_->End();
    }
    control_->EndProfile();   // LOCKED 였다면 무시된다 (RUNNING 일 때만 전이)

    // 05 §5.3 — 카운터는 **런 구간의 증분**으로 낸다. 노드 기동 이후 누적을 그대로 실으면
    // 두 번째 런부터 이전 런의 사고가 섞여 들어와 그 런을 잘못 판정하게 된다.
    const RunCounters_t c_end = counters_();
    result->goal_id        = goal_id;
    result->ticks_executed = static_cast<uint32_t>(executed);
    result->write_err_cnt  = static_cast<uint32_t>(c_end.write_err_total - c_at_start.write_err_total);
    result->irregular_tick_cnt =
        static_cast<uint32_t>(c_end.irregular_tick_cnt - c_at_start.irregular_tick_cnt);
    result->late_tick_cnt =
        static_cast<uint32_t>(c_end.late_tick_cnt - c_at_start.late_tick_cnt);
    result->drop_cnt  = static_cast<uint32_t>(c_end.drop_cnt - c_at_start.drop_cnt);
    result->mode      = player_->ModeNum();
    result->seed      = player_->Seed();
    result->slew_exempt_ticks = player_->SlewExemptTicks();
    result->run_dir   = "";        // 폴더는 CLI 소유다 — 브리지는 경로를 모른다
    // **재생 내내** 수렴 상태였어야 true 다. 시작과 끝 어느 한쪽이라도 미수렴이면
    // 그 런의 header.stamp 는 등급이 다르다 (05 §5.2 B1).
    result->clock_converged = c_at_start.clock_converged && c_end.clock_converged;
    result->drift_ppm       = c_end.drift_ppm;

    if (canceled) {
        result->success = false;
        result->message = "canceled";
        gh->canceled(result);
        RCLCPP_WARN(node_->get_logger(), "run_profile 취소됨 (goal_id=%u, %zu/%zu tick)",
                    goal_id, executed, total_ticks);
    } else if (end_state != ControlState::RUNNING) {
        result->success = false;
        result->message = "재생 중 FSM 이 " + std::string(ControlStateStr(end_state)) +
                          " 로 전이 — 중단 (사유: " + control_->LockReason() + ")";
        gh->abort(result);
        RCLCPP_ERROR(node_->get_logger(), "run_profile 중단 (goal_id=%u): %s",
                     goal_id, result->message.c_str());
    } else {
        result->success = true;
        result->message = "완료";
        gh->succeed(result);
        RCLCPP_INFO(node_->get_logger(),
            "run_profile 완료 (goal_id=%u, %zu tick, write 거부 %u 회)",
            goal_id, executed, result->write_err_cnt);
    }
}

// 200Hz — 스케줄 루프가 PrepareControlCommand 직전에 호출.
// 사전 샘플링 배열에서 한 tick 을 꺼내 FSM 에 넣는 것이 전부다 (연산 없음, §4.3-5).
// TickProfile 구현은 rd_profile_player.cpp (L2) 로 이동 — 02 A2

}  // namespace orin_bridge
