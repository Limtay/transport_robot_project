#include "orin_firmware_bridge/sched/rd_schedule.hpp"

#include "orin_firmware_bridge/sched/rd_slot_table.hpp"
#include <iostream>
#include <pthread.h>
#include <sched.h>

using namespace std::chrono_literals;

namespace orin_bridge {

RdSchedule::RdSchedule(RdComm* comm, RdMap* map, RobotState_t* state,
                       ITelemetrySink* sink, RdCommand* command,
                       const BridgeConfig& cfg, RdOos* oos, RdProfilePlayer* player,
                       RdControl* control,
                       std::function<void(bool)> on_init_done,
                       std::function<bool()> skip_cmd_write,
                       ILogger* log, IClock* clock, IRunGate* gate)
    : comm_(comm), map_(map), robot_state_(state), sink_(sink),
      command_(command), cfg_(cfg), oos_(oos), player_(player), control_(control),
      on_init_done_(std::move(on_init_done)), skip_cmd_write_(std::move(skip_cmd_write)),
      log_(log ? log : DefaultLogger()),
      clock_(clock ? clock : DefaultClock()),
      gate_(gate ? gate : DefaultRunGate()),
      tick_count_(0), is_running_(false)
{
    // (아래 표 굽기)
    // 04 §2.2.1 — **구간은 표가 소유한다.** 표를 tick 순서대로 펼쳐 TaskConfig_t 로 굽는다.
    // 종전에는 여기서 task_100hz_ 등 5개를 손으로 만들었고, 그래서 표를 봐도 각 슬롯이
    // 무엇을 읽는지 알 수 없었다 (그리고 DPC·PCU 구간은 표에 적을 자리가 없었다).
    //
    // ⚠ wire 는 바뀌지 않는다: 단일 세그 READ 는 seg_count==1 이든 start_addr/data_len
    //    이든 Data 4바이트가 동일하다 (rd_map.cpp EncodeNode — "n=1 은 기존과 동일").
    //    골든 바이트 테스트가 그것을 확인한다.
    // 굽기는 프레임 두 개(project·manual)에 같은 규칙으로 돈다 — 규칙을 두 벌 적으면
    // 한쪽만 고치는 사고가 난다 (이 세션에서만 같은 부류를 세 번 봤다).
    auto bake = [](const FrameDef& frame, TaskConfig_t* out, TaskConfig_t* out_read) {
        for (uint8_t t = 0; t < frame.ticks; t++) {
            const SlotDef& s = frame.slots[t];
            const uint8_t tgt = s.id == SlotId::PCU ? TARGET::PCU
                              : s.id == SlotId::DPC ? TARGET::DPC
                                                    : TARGET::ECU;
            // 표가 가리키는 고정 구간을 세그로 펼친다 (READ / RW 공통).
            auto fill_segs = [&s](TaskConfig_t* task) {
                if (s.read.src == ReadSrc::FIXED && s.read.fixed) {
                    for (uint8_t i = 0; i < s.read.fixed->count; i++)
                        task->segs[task->seg_count++] =
                            Segment_t{s.read.fixed->spans[i].addr, s.read.fixed->spans[i].len};
                }
                // start_addr/data_len 은 seg[0] 로 채운다 — 로그·기존 코드와의 호환용이며
                // wire 에는 영향이 없다 (READ 는 seg_count>=1 이면 segs 만 쓴다).
                if (task->seg_count > 0) {
                    task->start_addr = task->segs[0].addr;
                    task->data_len   = task->segs[0].len;
                }
            };
            TaskConfig_t task{}, read_only{};
            switch (s.inst) {
                case SlotInst::READ: {
                    task = TaskConfig_t(tgt, PacketInst::READ, 0, 0);
                    fill_segs(&task);
                    read_only = task;   // 이미 읽기 전용이다
                    break;
                }
                case SlotInst::WRITE:
                    task = TaskConfig_t(tgt, PacketInst::WRITE, s.write.addr, s.write.len);
                    // 쓰기 전용 슬롯의 폴백은 "아무것도 안 함" 이다 (읽을 구간이 없다).
                    break;
                case SlotInst::RW: {
                    // Q3 — 읽기 구간은 표가 소유하고(FIXED), 쓰기는 표가 적은 고정 범위다.
                    // control 의 RW 와 달리 런타임 프리셋·auto_mode 를 타지 않는다.
                    task = TaskConfig_t(tgt, PacketInst::RW, s.write.addr, s.write.len);
                    fill_segs(&task);
                    // 쓰기를 뺀 같은 읽기 — cmd_vel 일시정지 시 이 tick 이 통째로 사라지면
                    // 센서 시계열에 구멍이 나므로, 읽기만이라도 나가게 한다.
                    read_only = TaskConfig_t(tgt, PacketInst::READ, 0, 0);
                    fill_segs(&read_only);
                    break;
                }
                default:
                    break;   // EMPTY / COMMAND — 늦은 바인딩
            }
            out[t] = task;
            if (out_read) out_read[t] = read_only;
        }
    };
    bake(frames::kProject, project_task_, project_task_read_);
    bake(frames::kManual,  manual_task_,  nullptr);

    // 제어: 200Hz RW — write 범위는 auto_mode 에서 파생된다 (§2.6).
    // read 배치는 **읽기 프리셋**이 정한다 (04 §2.3, rd_read_preset.hpp).
    //
    // (프리셋 × auto_mode) 조합을 **기동 시 전부 만들어 둔다.** 200Hz 스레드는 atomic
    // 인덱스 두 개로 고르기만 한다 — 교체 때 태스크를 다시 조립하면 루프가 읽고 있는
    // 데이터를 건드리게 된다 (04 §2.4.2 "런타임에 조립하지 않는다").
    //
    // ── auto_mode=1(CURRENT): ECU 가 ctr_mode 를 CURRENT 로 강제하므로 bridge 는 전류만 쓴다.
    //    write 164:16 → 요청 1+24+2+16 = 43B / 응답 1+68 = 69B.
    //    워치독: 164~179 는 is_cmd_vel 판정 범위(132~187)와 겹쳐 cmd_write_tick 갱신 유지됨
    //    (rd_map_ecu.c:146 — addr<=187 && addr+len-1>=132). 축소해도 AUTO_TIMEOUT 안 걸린다.
    // ── auto_mode=2(DIRECT): ECU 미개입이라 bridge 가 ctr_mode 까지 소유해야 한다.
    //    write 128:52 → 요청 1+24+2+52 = 79B(와이어 87B) / 응답 69B.
    //    ⚠ 구 STM RX 버퍼 64B 로는 87B 요청이 오버런했다(P8). 2026-07-27 RX 256 으로 확장 — 01 §9.2.
    // ── auto_mode=4(VELOCITY) / 5(POSITION): CURRENT 와 대칭. write 범위만 다르다 (01 §3).
    //    ECU 가 ctr_mode 를 자가치유하므로 bridge 는 값만 쓴다 (2026-07-28 실기 확인).
    for (uint8_t pi = 0; pi < ecu::kPresetCount; pi++) {
        const ReadPreset& p = *ecu::kPresets[pi];
        auto build = [&](uint16_t waddr, uint16_t wlen) {
            TaskConfig_t t(TARGET::ECU, PacketInst::RW, waddr, wlen);
            for (uint8_t i = 0; i < p.count; i++)
                t.segs[t.seg_count++] = Segment_t{p.spans[i].addr, p.spans[i].len};
            return t;
        };
        task_control_[pi][kIdxCurrent]  = build(164, 16);
        task_control_[pi][kIdxDirect]   = build(ecu::REG_CMD_MOTOR_OFFSET, ecu::REG_CMD_MOTOR_SIZE);
        task_control_[pi][kIdxVelocity] = build(ecu::REG_CMD_VELOCITY_OFFSET, ecu::REG_CMD_VALUE_SIZE);
        task_control_[pi][kIdxPosition] = build(ecu::REG_CMD_POSITION_OFFSET, ecu::REG_CMD_VALUE_SIZE);

        // auto_mode: none — **쓰지 않는다.** 같은 read 세그를 READ 로 낸다.
        // 구 task_traction_ 이 하던 일이며, control_test 프리셋이면 wire 가 바이트 단위로 같다.
        TaskConfig_t r(TARGET::ECU, PacketInst::READ, 0, 0);
        for (uint8_t i = 0; i < p.count; i++)
            r.segs[r.seg_count++] = Segment_t{p.spans[i].addr, p.spans[i].len};
        task_control_read_[pi] = r;
    }
    // 기동 파라미터가 고른 프리셋으로 시작한다 (01 §4.2).
    read_preset_.store(cfg_.read_preset_param, std::memory_order_relaxed);
}

RdSchedule::~RdSchedule() {
    Stop();
}

int RdSchedule::MainLoopStart() {
    // 파라미터 오류는 하드웨어와 무관하므로 USB 대기 전에 즉시 판정한다 —
    // 오타 하나 때문에 "Waiting for USB..." 만 찍히며 매달려 있는 상황을 만들지 않기 위함.
    // 모드·프리셋 오타는 **모드와 무관하게** 막는다. 기본값으로 떨어뜨리면 조작자가
    // 의도하지 않은 모드로 뜨는데, 그건 모터가 도는 모드인지 아닌지가 갈리는 문제다.
    if (!cfg_.bridge_mode_valid || !cfg_.read_preset_valid) {
        RD_FATAL(log_, "RdSchedule",
            "기동 파라미터가 유효하지 않다 (%s) — 통신 시도 없이 종료 (§3.1)",
            !cfg_.bridge_mode_valid ? "bridge_mode" : "read_preset");
        return 1;
    }
    if (cfg_.IsControl() &&
        (!cfg_.active_motors_valid || !cfg_.auto_mode_valid)) {
        RD_FATAL(log_, "RdSchedule",
            "기동 파라미터가 유효하지 않다 (%s) — 통신 시도 없이 종료 (§3.1)",
            !cfg_.active_motors_valid ? "active_motors" : "auto_mode");
        return 1;
    }

    is_running_ = true;
    ApplyRtScheduling(pthread_self());  // 메인 스레드에서 루프를 돌리므로 현재 스레드에 적용
    RD_INFO(log_, "RdSchedule", "Scheduler MainLoop Started.");
    SupervisorLoop();
    return init_fatal_ ? 1 : 0;   // §3.1: INIT 검증 실패는 exit≠0
}

// SCHED_FIFO 우선순위 + CPU 코어 고정을 주어진 스레드에 적용.
// 권한(rtprio/CAP_SYS_NICE) 없으면 WARN 후 일반 우선순위로 계속.
void RdSchedule::ApplyRtScheduling(pthread_t thread) {
    sched_param sp;
    sp.sched_priority = kRtPriority;
    if (pthread_setschedparam(thread, SCHED_FIFO, &sp) != 0) {
        RD_WARN(log_, "RdSchedule",
            "SCHED_FIFO 설정 실패 (권한 부족) — limits.conf rtprio 설정 또는 sudo 실행 필요. "
            "일반 우선순위로 계속 (주기 밀림 발생 가능).");
    } else {
        RD_INFO(log_, "RdSchedule",
            "SCHED_FIFO pri=%d 적용됨.", kRtPriority);
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(kCpuCore, &cpuset);
    if (pthread_setaffinity_np(thread, sizeof(cpu_set_t), &cpuset) != 0) {
        RD_WARN(log_, "RdSchedule", "CPU affinity(core %d) 설정 실패.", kCpuCore);
    } else {
        RD_INFO(log_, "RdSchedule", "CPU affinity core %d 고정됨.", kCpuCore);
    }
}

// 루프는 **메인 스레드에서만** 돈다 (`MainLoopStart`). 별도 스레드로 돌리는 경로는 없다 —
// 200Hz 주기를 지키려면 SCHED_FIFO 를 그 스레드에 걸어야 하는데, 메인에 걸면
// `main()` 이 곧 실시간 스레드라 조립·종료 순서를 한 곳에서 볼 수 있다.
void RdSchedule::Stop() { is_running_ = false; }

void RdSchedule::SupervisorLoop() {
    while (is_running_ && gate_->Ok()) {
        if (RunLoop() == RD_OK) break;
        // INIT 검증 실패는 설정 오류 — 재접속을 반복해도 같은 결과이므로 즉시 종료한다.
        if (init_fatal_) {
            is_running_ = false;
            break;
        }
        {
            std::lock_guard<std::mutex> lock(robot_state_->state_mutex);
            robot_state_->ecu.comm.is_connected = false;
            robot_state_->dpc.comm.is_connected = false;
            robot_state_->pcu.comm.is_connected = false;
        }
        RD_WARN(log_, "RdSchedule", "Hardware Error! Restarting in 1 second...");
        std::this_thread::sleep_for(1s);
    }
}

uint8_t* RdSchedule::RegBytePtr(uint16_t addr) {
    auto& cs = robot_state_->ecu.reg.cmd_system;
    switch (addr) {
        case ecu::REG_MODE_OFFSET:       return &cs.mode;
        case ecu::REG_MOTOR_MASK_OFFSET: return &cs.motor_mask;
        case ecu::REG_AUTO_MODE_OFFSET:  return &cs.auto_mode;
        default:                         return nullptr;
    }
}

// 1바이트 WRITE + 같은 주소 READ 로 read-back 검증 (§3.1).
// 매 시도마다 섀도를 기대값으로 되돌린다 — 직전 시도의 READ decode 가 섀도를 ECU 실값으로
// 덮었을 수 있어서, 이걸 빠뜨리면 2회차부터 엉뚱한 값을 쓰게 된다.
bool RdSchedule::WriteVerifyByte(uint16_t addr, uint8_t* shadow_field,
                                 uint8_t expect, const char* label) {
    const TaskConfig_t wr_cfg{TARGET::ECU, PacketInst::WRITE, addr, 1};
    const TaskConfig_t rd_cfg{TARGET::ECU, PacketInst::READ,  addr, 1};

    for (int attempt = 1; attempt <= kInitMaxRetry; attempt++) {
        {
            std::lock_guard<std::mutex> lock(robot_state_->state_mutex);
            *shadow_field = expect;
        }
        RD_RET tx_wr = RD_ERROR;
        ExecuteTask(wr_cfg, &tx_wr);

        if (tx_wr == RD_OK) {
            RD_RET tx_rd = RD_ERROR;
            ExecuteTask(rd_cfg, &tx_rd);
            if (tx_rd == RD_OK) {
                uint8_t actual;
                {
                    std::lock_guard<std::mutex> lock(robot_state_->state_mutex);
                    actual = *shadow_field;
                }
                if (actual == expect) {
                    RD_INFO(log_, "RdSchedule",
                        "INIT %s: addr%u = 0x%02X 검증 OK (%d/%d 회차)",
                        label, addr, expect, attempt, kInitMaxRetry);
                    return true;
                }
                RD_WARN(log_, "RdSchedule",
                    "INIT %s: read-back 불일치 (기대 0x%02X, 실제 0x%02X) — %d/%d 회차",
                    label, expect, actual, attempt, kInitMaxRetry);
            } else {
                RD_WARN(log_, "RdSchedule",
                    "INIT %s: 검증 READ 실패 — %d/%d 회차", label, attempt, kInitMaxRetry);
            }
        } else {
            RD_WARN(log_, "RdSchedule",
                "INIT %s: WRITE 실패 — %d/%d 회차", label, attempt, kInitMaxRetry);
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kInitRetryIntervalMs));
    }
    RD_FATAL(log_, "RdSchedule",
        "INIT %s: %d 회 재시도 전부 실패 (addr%u = 0x%02X) — 노드를 종료한다",
        label, kInitMaxRetry, addr, expect);
    return false;
}

// §3.1 INIT 플로우: motor_mask(192) → auto_mode(188) → mode(190)=AUTO.
// **순서가 안전의 핵심이다** (§2.6):
//   - 마스크를 AUTO 전에 확정 → 의도하지 않은 트랙이 한 tick 도 돌지 않는다.
//   - auto_mode 를 AUTO 전에 확정 → KINEMATIC(기본값 0)이 활성인 순간을 한 tick 도 거치지
//     않는다. KINEMATIC 이 살아 있으면 ECU 가 100Hz 로 ctr_mode 를 VELOCITY 로 덮어써
//     bridge 의 CURRENT write 와 경쟁하고, controlTask 가 tick 마다 1/3 을 오가며 읽는다.
RD_RET RdSchedule::InitControl() {
    if (!cfg_.active_motors_valid) {
        RD_FATAL(log_, "RdSchedule",
            "active_motors 파라미터가 유효하지 않다 — INIT 중단 (노드 종료)");
        return RD_FATAL;
    }
    if (!cfg_.auto_mode_valid) {
        RD_FATAL(log_, "RdSchedule",
            "auto_mode 파라미터가 유효하지 않다 — INIT 중단 (노드 종료)");
        return RD_FATAL;
    }

    auto& reg = robot_state_->ecu.reg;

    // ① motor_mask(192) — 활성 트랙 확정
    if (!WriteVerifyByte(ecu::REG_MOTOR_MASK_OFFSET, &reg.cmd_system.motor_mask,
                         cfg_.active_motor_mask, "motor_mask")) {
        return RD_FATAL;
    }
    // ② auto_mode(188) — AUTO 경로 확정 (KINEMATIC 덮어쓰기 차단). AUTO 진입 전이어야 한다.
    const uint8_t am = cfg_.auto_mode_param;
    const std::string am_label = std::string("auto_mode(") + ecu::AutoModeName(am) + ")";
    if (!WriteVerifyByte(ecu::REG_AUTO_MODE_OFFSET, &reg.cmd_system.auto_mode,
                         am, am_label.c_str())) {
        return RD_FATAL;
    }
    control_->SetAutoMode(am);   // write 범위 셀렉터 확정
    // 비-DIRECT 면 ECU 가 ctr_mode 를 강제한다 — 섀도를 그 값으로 맞춰 둔다.
    // (DIRECT 는 bridge 소유이므로 건드리지 않는다.) 이걸 빼면 기동 직후 status 가
    // auto_mode=5(POSITION) 인데 ctr_mode=[1,1,1,1](CURRENT) 로 보고한다.
    if (am != ecu::AUTO_MODE_DIRECT) {
        const uint8_t forced = ecu::AutoModeForcedCtrMode(am);
        for (int i = 0; i < 4; i++) control_->SetCtrMode(i, forced);
    }
    // ③ mode(190)=AUTO — RC 없이 레지스터 write 로 진입 (reg.cmd_system.mode 가 모드의 단일 진실원천)
    if (!WriteVerifyByte(ecu::REG_MODE_OFFSET, &reg.cmd_system.mode,
                         ecu::MODE_AUTO, "mode(AUTO)")) {
        return RD_FATAL;
    }

    RD_INFO(log_, "RdSchedule",
        "INIT 완료 — mask=0x%02X, auto_mode=%u(%s), write 범위 %s",
        cfg_.active_motor_mask, am, ecu::AutoModeName(am), ecu::AutoModeWriteSpan(am));
    // 활성 프리셋을 텔레메트리에 **명시적으로** 알린다. 기본값에 기대지 않는 이유는
    // 기본값이 틀렸을 때 조용히 틀린 채로 돌기 때문이다 (2026-07-29 실기에서 겪었다).
    if (sink_) sink_->SetReadPreset(ecu::kPresets[read_preset_.load(std::memory_order_relaxed)]);

    control_->MarkInitDone();   // INIT -> IDLE (write 0A 고정)
    return RD_OK;
}

RD_RET RdSchedule::RunLoop() {
    auto period        = 5ms;
    auto initial_delay = 1s;

    if (Initialize() != RD_OK) return RD_FATAL;

    // 견인 테스트 / MPC 제어 모드 — 기동 시 파라미터로 고정 (memo_260606.md P1/P2)
    // control 은 한 축이고, 그 안에서 **쓰는가/읽기만 하는가**를 auto_mode 가 정한다.
    const bool read_only_control = cfg_.IsReadOnlyControl();   // 구 traction_test_mode
    const bool control_mode      = cfg_.IsControl();
    if (read_only_control) {
        RD_WARN(log_, "RdSchedule",
                    "Main Control Loop Started [CONTROL / auto_mode=none] (200Hz 배치 READ 전용 — WRITE 없음, 서브슬롯 정지). 구 traction_test_mode 와 같은 배치다");
        // ⚠ **여기가 비어 있었다** (2026-07-30 실기에서 발견). ECU 에 아무것도 쓰지 않으므로
        //   INIT 검증할 write 가 없는데, 그렇다고 INIT 에 머물러도 되는 것은 아니었다:
        //   FSM 이 INIT 이면 `AcceptsConfig` 가 전부 거부해 **조작 입구가 통째로 막힌다.**
        //   견인 실험(`./run.sh traction`)이 정확히 이 구성이고, 그래서 그동안
        //     · `cli status` 가 아예 안 됐고
        //     · 웹 Tab3(레지스터 맵)이 죽어 있었고
        //     · bag 의 모든 메시지가 control_state=INIT / auto_mode=CURRENT(틀린 값)로 기록됐다
        //   (manual 분기는 같은 문제를 이미 인지하고 고쳐 뒀는데 이쪽이 빠져 있었다.)
        //
        //   `SetAutoMode(none)` 을 함께 하는 이유: 이 값이 피드백의 `auto_mode` 로 나가고
        //   result.json 에 남는다. 기본값(CURRENT)을 그대로 두면 **쓰지 않는 런이
        //   "전류 명령을 쓴 런" 으로 기록된다.**
        control_->SetAutoMode(AUTO_MODE_NONE);
        control_->MarkInitDone();
        if (sink_) sink_->SetReadPreset(ecu::kPresets[read_preset_.load(std::memory_order_relaxed)]);
        // with_profiles=false — write 가 없으므로 재생 서버는 열지 않는다.
        if (on_init_done_) on_init_done_(false);
    } else if (control_mode) {
        RD_WARN(log_, "RdSchedule",
                    "Main Control Loop Started [CONTROL] (200Hz RW: cmd_motor WRITE + 배치 READ — 서브슬롯 정지)");
        // §3.1 INIT 플로우 — 성공 시에만 FSM 이 IDLE 로 간다. 실패는 exit≠0.
        if (InitControl() != RD_OK) {
            init_fatal_ = true;
            return RD_FATAL;
        }
        // IDLE 도달 후에야 goal 을 받을 수 있다 — INIT 전에 열면 검증 안 된 상태로 재생될 수 있다.
        if (on_init_done_) on_init_done_(true);   // 명령을 쓰는 구성 → 재생 서버까지 연다
    } else if (cfg_.IsManual()) {
        // 01 §4.1 (R1·R2) — **자동 설정을 하나도 하지 않는다.** motor_mask·auto_mode·
        // mode(AUTO) write 를 전부 건너뛴다. 기동은 범위 스캔(진실 원천 확보)까지이고
        // 그 다음은 조작자가 슬롯 명령으로 직접 쌓는다.
        //
        // "안전장치가 없는 모드" 가 아니라 "자동 설정이 없는 모드" 다 —
        // safe_stop 게이트·LOCKED 래치·active_motors 불변식은 그대로 산다.
        RD_WARN(log_, "RdSchedule",
                    "Main Control Loop Started [MANUAL] — **ECU 자동 설정 없음.** "
                    "motor_mask/auto_mode/mode(AUTO) write 를 건너뛴다. "
                    "모터를 돌리려면 슬롯 명령으로 직접 쌓을 것 (01 §4.1)");
        // FSM 은 IDLE 로 보낸다 — INIT 에 머물면 config service 가 통째로 막힌다
        // (AcceptsConfig 가 INIT 을 거부한다). manual 의 요점은 조작자가 직접 쌓는 것이므로
        // 입구를 잠그면 모드 자체가 성립하지 않는다.
        control_->MarkInitDone();
        // manual 은 `WritesCommands()` 가 false 다 (control 이 아니므로) — 슬롯 명령으로
        // 직접 쌓는 모드이고 프레임에 write 슬롯이 없다. 재생 서버를 열면 goal 은 수락되는데
        // 아무것도 안 움직인다. 조작 입구만 연다.
        if (on_init_done_) on_init_done_(false);
    } else {
        RD_INFO(log_, "RdSchedule",
                    "Main Control Loop Started (200Hz tick / 100Hz RD / 50Hz WR / 10Hz sys / 5Hz cmd x4)");
    }

    auto next_cycle = clock_->NowSteady() + initial_delay;
    RD_RET ret_val  = RD_OK;

    while (is_running_ && gate_->Ok()) {
        next_cycle += period;
        std::this_thread::sleep_until(next_cycle);

        // [계측] 깨어난 시점 — next_cycle 대비 오버슛이 순수 wake latency(스케줄 지연).
        auto wake_time = clock_->NowSteady();
        int64_t wake_us = std::chrono::duration_cast<std::chrono::microseconds>(
                              wake_time - next_cycle).count();
        if (wake_us > stat_wake_max_) stat_wake_max_ = wake_us;

        ret_val = RD_OK;
        // 07 §3.2 — control 프레임의 **양보 tick**. control 은 커맨드 전용 슬롯이 0개라
        // (04 §2.4.1) 이 한 칸이 사용자 명령(레지스터 READ/WRITE·REBOOT)의 유일한 경로다.
        //
        // 표는 "어느 칸을 양보할 수 있는가" 만 말하고(frames::kControl.user_slot_mask),
        // "지금 양보해도 되는가" 는 여기 게이트가 정한다.
        //
        // 막는 것은 **진행 중인 실험**이다 — RUNNING/STREAM 에서 tick 을 훔치면 시계열에
        // 구멍이 생긴다. INIT·IDLE·LOCKED 는 막지 않는다.
        //   ⚠ INIT 을 막으면 `auto_mode:none`(구 traction) 에서 양보가 **영영 열리지 않는다** —
        //     그 경로는 InitControl() 를 타지 않아 FSM 이 INIT 에 머물기 때문이다.
        //     "IDLE 에서만" 으로 적었다가 이 구멍을 확인하고 고쳤다.
        //
        // State() 는 **양보 tick 에서만** 묻는다 (단축 평가) — 200Hz RT 루프에서 매 tick
        // FSM 락을 잡을 이유가 없다.
        //
        // ⚠ 마스크는 **활성 프레임**에서 읽는다. 종전에는 `frames::kControl` 로 못박아 두고
        //   `control_mode &&` 로 걸렀는데, Q3 로 project(3칸)·manual(10칸)에도 마스크가
        //   생기면서 그 하드코딩이 "표에 적혀 있는데 안 열리는 칸" 을 만들게 됐다.
        const FrameDef& active_frame =
            control_mode ? (read_only_control ? frames::kControlRead : frames::kControl)
          : cfg_.AutoConfiguresEcu() ? frames::kProject
                                     : frames::kManual;
        const bool user_slot_open =
            active_frame.UserSlotAt(tick_count_) &&
            control_->State() != ControlState::RUNNING &&
            control_->State() != ControlState::STREAM;

        if (user_slot_open && RunUserSlot(&ret_val)) {
            // 이 tick 은 제어 트랜잭션을 하지 않았다 → **피드백을 발행하지 않는다.**
            // 없는 샘플을 지어내지 않는다 (§6.2-2 와 같은 이유).
            // 발행하지 않았다는 사실 자체를 센다 — 안 그러면 시계열의 구멍이 무기명이 된다.
            if (sink_) sink_->OnIrregularTick();
        } else if (read_only_control) {
            // ===== 견인 테스트: 매 tick(200Hz) 배치 READ + Decode 성공 시 즉시 발행 =====
            // WRITE 없음(전류 명령은 STM RC 램프가 생성) / jeongae FSM·서브슬롯 정지.
            RD_RET tx_res = RD_ERROR;
            // 프리셋에서 파생된 READ 태스크 — 구 task_traction_ 은 control_test 프리셋과
            // 같은 세그라 별도 정의를 두지 않는다 (진실이 둘이면 갈라진다).
            ret_val = ExecuteTask(task_control_read_[read_preset_.load(std::memory_order_relaxed)],
                                  &tx_res);
            if (tx_res == RD_OK) {
                // 순서 주의: `OnTxn` 이 시계 추정기를 갱신하고, 그 결과 캐시를
                // `ControlFeedback` 의 rtt/clock_offset 요약 필드가 읽는다.
                // 뒤집으면 한 tick 늦은 값이 실린다.
                sink_->OnTxn(last_txn_);   // §2.5 시계 동기·지연 계측
                sink_->OnFeedback();        // §3.4 (traction 모드도 발행)
            }
        } else if (control_mode) {
            // ===== 제어: 매 tick(200Hz) RW — cmd_motor WRITE + 배치 READ + 발행 =====
            // write 값은 §2 FSM 이 결정 (IDLE=0A) + STM 20ms 워치독 (이중 안전).
            // §2 out-of-span 단발: 이 tick 을 일반 WRITE/READ 패킷으로 대체한다.
            // 대체된 tick 은 read 스냅샷이 없으므로 피드백을 발행하지 않는다 (§6.2-2,
            // 200Hz 스트림에 1샘플 갭 허용 — 보간하면 없는 값을 지어내는 것이라 금지).
            OosStep_t oos;
            if (oos_->TakeStep(&oos)) {
                const bool is_write = (oos.phase == OosPhase::WRITE);
                if (is_write) {
                    std::lock_guard<std::mutex> lock(robot_state_->state_mutex);
                    uint8_t* p = RegBytePtr(oos.addr);
                    if (p) *p = oos.value;
                }
                const TaskConfig_t cfg{TARGET::ECU,
                                       is_write ? PacketInst::WRITE : PacketInst::READ,
                                       oos.addr, 1};
                RD_RET tx = RD_ERROR;
                ExecuteTask(cfg, &tx);
                uint8_t readback = 0;
                if (!is_write) {
                    std::lock_guard<std::mutex> lock(robot_state_->state_mutex);
                    uint8_t* p = RegBytePtr(oos.addr);
                    if (p) readback = *p;
                }
                oos_->ReportResult(tx == RD_OK, readback);
                if (sink_) sink_->OnIrregularTick();   // 이 tick 의 read 스냅샷은 없다
                continue;   // §6.2-3: 성패와 무관하게 다음 tick 은 정상 RW 로 복귀
            }

            // 프로파일 재생: 사전 샘플링 배열에서 이번 tick 값을 꺼내 FSM 에 넣는다.
            // (RUNNING 이 아니면 no-op — write 소스 결정은 FSM 이 한다)
            sink_->OnWriteErrTotal(map_->RwWriteErrTotal());
            player_->Tick(control_);
            control_->PrepareWrite();
            RD_RET tx_res = RD_ERROR;
            // §2.6 제약 1: 구조체를 바꾸는 게 아니라 미리 만든 둘 중 하나를 **고른다**.
            // §2.6 auto_mode 파생 write 범위. **런타임에 TaskConfig_t 를 수정하지 않는다** —
            // 미리 만들어 둔 것 중에서 고르기만 한다 (200Hz 스레드와의 레이스 방지).
            const TaskConfig_t& cfg = SelectControlTask(control_->AutoMode());
            ret_val = ExecuteTask(cfg, &tx_res);
            if (tx_res == RD_OK) {
                sink_->OnRwErr(map_->LastRwErr());    // §3.4 rw_err
                sink_->OnTxn(last_txn_);   // §2.5 시계 동기·지연 계측
                sink_->OnFeedback();        // §3.4
                // §2 LOCKED 래치: write 가 연속 거부되면 명령이 안 먹는 채로 실험이
                // 진행되는 상황이라 0A 래치 후 명시적 REARM 을 요구한다.
                control_->NoteWriteErrStreak(map_->RwWriteErrStreak());
            }
        } else {
            // jeongae 자동 시퀀스 FSM (통신 없음, 상태 전이만)
            command_->TickAutoSequence();

            // 04 §2.2 (C1) — **표에서 찾아본다.** 종전에는 tick%2 / odd_idx%2 / (odd_idx-1)/2
            // 산술로 매번 되짚었다. 스케줄을 바꾸려면 그 식을 읽어내야 했고, 주기가 몇 Hz 인지는
            // 세어 보기 전엔 알 수 없었다. 표는 그 둘을 한눈에 준다 (rd_slot_table.hpp).
            //
            // **게이트는 표가 아니라 여기 남는다** — blackout·cmd_vel guard·manual 은
            // "이번에 무엇을 할 차례인가" 가 아니라 "지금 해도 되는가" 의 문제라 층이 다르다.
            const uint8_t  tick_in_frame = tick_count_ % active_frame.ticks;
            const SlotDef& slot          = active_frame.slots[tick_in_frame];
            const bool     is_manual     = (&active_frame == &frames::kManual);
            const TaskConfig_t* baked    = is_manual ? manual_task_ : project_task_;

            if (slot.id == SlotId::COMMAND) {
                // 늦은 바인딩 — 대상·명령을 커맨드 슬롯이 런타임에 정한다.
                TaskConfig_t task;
                if (command_->GetSlotTask(slot.cmd_index, &task)) {
                    RD_RET tx_res = RD_ERROR;
                    ret_val = ExecuteTask(task, &tx_res);
                    command_->ReportResult(slot.cmd_index, tx_res);
                }
            } else if (slot.id != SlotId::EMPTY) {
                // 게이트: "지금 해도 되는가". 대상(blackout)은 모든 슬롯 공통이고,
                // cmd_vel WRITE 만 추가 조건을 갖는다.
                //   skip: manual(자동 write 금지) / jeongae soft-ESTOP / cmd_vel 안전장치.
                //   (CMD 영역은 unlock 불필요)
                bool skip = command_->IsTargetBlackedOut(
                                slot.id == SlotId::PCU ? TARGET::PCU
                              : slot.id == SlotId::DPC ? TARGET::DPC
                                                       : TARGET::ECU);
                if (slot.id == SlotId::PCU) skip = skip || !cfg_.enable_pcu_read;  // 04 §6 — 레지스터 미확정
                if (slot.id == SlotId::DPC) skip = skip || !cfg_.enable_dpc_read;  // 04 §6 — 기본 off (보드 미장착 대비)

                // cmd_vel 쓰기를 막아야 하는가. manual 은 프레임 자체가 READ 라 해당 없다
                // (01 §4.1 을 게이트가 아니라 표가 표현한다 — 04 §2.4).
                const bool block_write =
                    slot.id == SlotId::ECU && slot.write.Has() &&
                    (command_->IsCmdVelPaused() || (skip_cmd_write_ && skip_cmd_write_()));

                // 판정은 표가 소유한 순수 함수가 한다 (rd_slot_table.hpp ActionFor) —
                // 실기 heartbeat 로는 폴백이 걸렸는지 안 걸렸는지 구분되지 않으므로
                // (어느 쪽이든 트랜잭션 1건) 단위 테스트가 닿는 곳에 둔다.
                if (!skip) {
                    switch (ActionFor(slot, block_write)) {
                        case SlotAction::NORMAL:
                            ret_val = ExecuteTask(baked[tick_in_frame]);
                            break;
                        case SlotAction::READ_FALLBACK:
                            ret_val = ExecuteTask(project_task_read_[tick_in_frame]);
                            break;
                        case SlotAction::SKIP:
                            break;
                    }
                }
            }
        }
        tick_count_++;

        if (ret_val == RD_FATAL) {
            RD_ERROR(log_, "RdSchedule", "Hardware Disconnected! Returning to Supervisor.");
            comm_->Clear();
            return RD_FATAL;
        }

        // time_elapsed = wake latency(sleep_until 오버슛) + 이번 tick 처리시간.
        auto time_elapsed = clock_->NowSteady() - next_cycle;
        int64_t elapsed_us = std::chrono::duration_cast<std::chrono::microseconds>(time_elapsed).count();

        // 매 tick 구간 통계 누적 (평균/최대/초과 횟수).
        stat_sum_us_ += (elapsed_us > 0 ? static_cast<uint64_t>(elapsed_us) : 0);
        stat_cnt_++;
        if (elapsed_us > stat_max_us_) stat_max_us_ = elapsed_us;
        // 처리시간 = 전체 - wake. 스파이크가 wake(스케줄) 인지 proc(I/O) 인지 분리.
        int64_t proc_us = elapsed_us - wake_us;
        if (proc_us > stat_proc_max_) stat_proc_max_ = proc_us;

        if (time_elapsed > 5 * period) {
            RD_FATAL(log_, "RdSchedule",
                         "Processing time exceeded 5x period! (%ld us)", elapsed_us);
            return RD_FATAL;
        } else if (time_elapsed > period) {
            next_cycle = clock_->NowSteady() + period;
            // RT 루프 안에서는 카운트만 — per-tick 동기 콘솔 로깅이 그 자체로 지터를
            // 유발(다음 사이클까지 밀림)하므로 제거하고 헤더비트에서 요약 출력한다.
            exceeded_cnt_++;
            // exceeded_cnt_ 는 헤더비트마다 0 으로 리셋되는 **구간** 통계라 런 단위로 쓸 수
            // 없다. 리셋되지 않는 누적본을 따로 텔레메트리에 남겨 action result 가
            // 시작/끝 차분을 뜬다 (late_tick_cnt).
            if (sink_) sink_->OnLateTick();
        }

        if (tick_count_ % 400 == 55) {
            uint64_t loss   = (tx_count > rx_count) ? (tx_count - rx_count) : 0;
            double   avg_us = stat_cnt_ ? static_cast<double>(stat_sum_us_) / stat_cnt_ : 0.0;
            double   over_pct = stat_cnt_ ? 100.0 * exceeded_cnt_ / stat_cnt_ : 0.0;
            bool ecu_on, dpc_on, pcu_on;
            {
                std::lock_guard<std::mutex> lock(robot_state_->state_mutex);
                ecu_on = robot_state_->ecu.comm.is_connected;
                dpc_on = robot_state_->dpc.comm.is_connected;
                pcu_on = robot_state_->pcu.comm.is_connected;
            }
            const char* ecu = ecu_on ? "ON" : "OFF";
            const char* dpc = dpc_on ? "ON" : "OFF";
            const char* pcu = pcu_on ? "ON" : "OFF";
            std::printf("\n====== [RdSchedule Heartbeat] ======\n");
            std::printf(" Ticks : %lu\n", tick_count_);
            std::printf(" Timing: avg %.0f us / max %ld us | over-period %lu/%lu (%.1f%%)\n",
                        avg_us, stat_max_us_, exceeded_cnt_, stat_cnt_, over_pct);
            std::printf(" Spike : wake_max %ld us / proc_max %ld us  <- 스파이크 출처\n",
                        stat_wake_max_, stat_proc_max_);
            std::printf(" I/O   : clear_max %ld / write_max %ld / read_max %ld us\n",
                        stat_clear_max_, stat_write_max_, stat_read_max_);
            std::printf(" Comm  : Tx %lu / Rx %lu (Loss: %lu)\n", tx_count, rx_count, loss);
            std::printf(" Nodes : ECU[%s]  DPC[%s]  PCU[%s]\n", ecu, dpc, pcu);
            std::printf("====================================\n");

            // 평균 처리시간이 예산(4ms)을 넘으면 tail spike 가 아니라 전형적 케이스가
            // 무거운 것 → 구조적 조치 필요. 구간당 1회만 경고(로깅 지터 최소화).
            if (avg_us > static_cast<double>(kBudgetUs)) {
                RD_WARN(log_, "RdSchedule",
                    "평균 처리시간 %.0f us > 예산 %ld us — 주기 예산 초과(전형 케이스). "
                    "Read 타임아웃/슬롯 부하/코어 격리 점검 필요.", avg_us, kBudgetUs);
            }

            stat_sum_us_ = 0; stat_cnt_ = 0; stat_max_us_ = 0; exceeded_cnt_ = 0;
            stat_wake_max_ = 0; stat_proc_max_ = 0;
            stat_clear_max_ = 0; stat_write_max_ = 0; stat_read_max_ = 0;
        }
    }
    return RD_OK;
}

// 서브 슬롯 패턴 (200ms 1회전): [E10, PCU, DPC, C1, C2, E10, PCU, DPC, C3, C4]
RD_RET RdSchedule::Initialize() {
    RD_INFO(log_, "RdSchedule", "RdNode Comm Node Starting");
    // 복구 재진입 시 깨진 포트/에러 카운터를 강제로 닫아 Init() 이 실제 재오픈하도록 한다.
    // (최초 기동 시엔 포트가 닫혀 있어 사실상 no-op)
    comm_->Stop();
    while (gate_->Ok()) {
        if (comm_->Init(&packet_obj_) == RD_OK) break;
        RD_WARN(log_, "RdSchedule", "Waiting for USB...");
        std::this_thread::sleep_for(1s);
    }
    if (!gate_->Ok()) {
        std::cerr << "[RdSchedule Fatal] ROS shutdown during init" << std::endl;
        return RD_FATAL;
    }
    // 잠금은 DEFINE(1~15) 영역만 — CMD 영역 쓰기는 unlock 불필요.
    // DEFINE 수정이 필요할 땐 CLI 'macro unlock on' → write → 'macro unlock off'.
    return RD_OK;
}

// auto_mode -> write 범위 (01 §3 표). 미지원 값은 CURRENT 로 떨어뜨린다 —
// 기동 검증이 이미 막지만, 런타임 config service 가 바꿀 수 있으므로 여기서도 안전측으로 둔다.
const TaskConfig_t& RdSchedule::SelectControlTask(uint8_t auto_mode) const {
    const uint8_t pi = read_preset_.load(std::memory_order_relaxed);
    switch (auto_mode) {
        case ecu::AUTO_MODE_DIRECT:   return task_control_[pi][kIdxDirect];
        case ecu::AUTO_MODE_VELOCITY: return task_control_[pi][kIdxVelocity];
        case ecu::AUTO_MODE_POSITION: return task_control_[pi][kIdxPosition];
        default:                      return task_control_[pi][kIdxCurrent];
    }
}

// 07 §3.2 — control 프레임의 양보 tick 에서 대기 중인 커맨드 하나를 실행한다.
//
// 실행할 것이 없으면 **false 를 돌려주고 이 tick 은 평소대로 제어에 쓰인다** — 즉 커맨드가
// 없을 때의 동작은 종전과 바이트 단위로 같다. 훔친 tick 만 제어 트랜잭션이 빠진다.
//
// 슬롯 번호가 낮을수록 우선순위가 높다는 규칙(04 §2.2.2)을 그대로 따르며, 프레임당 한 칸이므로
// 여러 슬롯이 차 있으면 낮은 번호부터 한 프레임에 하나씩 나간다.
bool RdSchedule::RunUserSlot(RD_RET* ret_val) {
    for (int i = 0; i < CMD_NUM_SLOTS; i++) {
        TaskConfig_t task;
        if (!command_->GetSlotTask(i, &task)) continue;
        RD_RET tx_res = RD_ERROR;
        *ret_val = ExecuteTask(task, &tx_res);
        command_->ReportResult(i, tx_res);
        return true;
    }
    return false;
}

// 04 §2.4.2 — 교체는 **IDLE 에서만**. 그 판정은 호출자(config 서비스)가 이미 했다.
// 여기서는 id 범위만 본다: 없는 프리셋으로 바꿨다고 응답하는 일이 없어야 한다.
bool RdSchedule::SetReadPreset(uint8_t id, std::string* why) {
    if (id >= ecu::kPresetCount) {
        if (why) *why = "프리셋 id " + std::to_string(id) + " 없음 (0~" +
                        std::to_string(ecu::kPresetCount - 1) + ")";
        return false;
    }
    read_preset_.store(id, std::memory_order_relaxed);
    // 텔레메트리에 **어느 블록이 더 이상 갱신되지 않는지** 알린다. 이걸 빼면 낡은 섀도
    // 값이 신선한 값처럼 계속 발행된다 (rd_read_preset.hpp 의 Covers() 설명 참조).
    if (sink_) sink_->SetReadPreset(ecu::kPresets[id]);
    if (why) *why = std::string("읽기 프리셋 = ") + ecu::kPresets[id]->name +
                    " (응답 " + std::to_string(ecu::kPresets[id]->RespPayload()) + "B)";
    return true;
}

RD_RET RdSchedule::ExecuteTask(const TaskConfig_t& config, RD_RET* tx_result) {
    if (tx_result) *tx_result = RD_ERROR;

    size_t data_len = 0;
    RD_RET ret = map_->Encode(config, robot_state_, &packet_obj_, &data_len);
    if (ret != RD_OK) return ret;

    // [핵심·유일한 flush 지점] Write 직전 RX 버퍼 flush.
    // Orin 마스터 req→resp 구조에서, 이전 사이클의 늦은(stale) 응답이나
    // 잔여 바이트를 이번 요청과 섞이지 않게 매번 깨끗이 비운다.
    // 정상 사이클에선 비울 게 없어 사실상 no-op, 에러(sync/length/body/CRC) 후엔
    // 여기서 1사이클 내 자가복구. → RdComm::Read 내부엔 별도 Flush 를 두지 않는다.
    auto _t0 = clock_->NowSteady();
    comm_->Clear();
    auto _t1 = clock_->NowSteady();

    // [§2.5 계측] t_req: write 직전 epoch 시각 (ROS time 과 동일 시계, use_sim_time 제외)
    last_txn_ = TxnTiming_t{};
    last_txn_.t_req = clock_->NowEpoch();
    last_txn_.req_bytes = data_len + kWireOverheadBytes;

    ret = comm_->Write(&packet_obj_, data_len);
    auto _t2 = clock_->NowSteady();
    if (ret == RD_OK) tx_count++;
    else if (ret == RD_FATAL) return ret;

    // 헤더 2ms + 바디 2ms = 최악 4ms < 5ms 주기 (1ms 마진)
    ret = comm_->Read(&packet_obj_, 2, 2);
    auto _t3 = clock_->NowSteady();

    // [§2.5 계측] t_resp: 응답 수신 완료 epoch 시각 (성공 여부 무관하게 기록, valid 는 아래서)
    last_txn_.t_resp = clock_->NowEpoch();
    {
        using us = std::chrono::microseconds;
        int64_t c = std::chrono::duration_cast<us>(_t1 - _t0).count();
        int64_t w = std::chrono::duration_cast<us>(_t2 - _t1).count();
        int64_t r = std::chrono::duration_cast<us>(_t3 - _t2).count();
        if (c > stat_clear_max_) stat_clear_max_ = c;
        if (w > stat_write_max_) stat_write_max_ = w;
        if (r > stat_read_max_)  stat_read_max_  = r;
    }
    if (ret == RD_OK) {
        rx_count++;
        last_txn_.resp_bytes = packet_obj_.rx.data_len + kWireOverheadBytes;
        ret = map_->Decode(&packet_obj_, config, robot_state_);
        if (ret != RD_OK) return ret;  // 패킷 에러 (Data[0] != NONE 포함)
        last_txn_.valid = true;        // [§2.5] 응답+Decode 성공 — 시계 동기 샘플로 사용 가능
        if (tx_result) *tx_result = RD_OK;
    } else if (ret == RD_FATAL) {
        return ret;
    } else {
        // RD_ERROR / RD_TIMEOUT: Read 내부에서 이미 flush 완료, 다음 사이클 Write 전 flush 가 재보장
        if (tx_result) *tx_result = ret;
    }

    return RD_OK;
}

} // namespace orin_bridge
