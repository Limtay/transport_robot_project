#ifndef ORIN_FIRMWARE_BRIDGE__RD_READ_PRESET_HPP_
#define ORIN_FIRMWARE_BRIDGE__RD_READ_PRESET_HPP_

// 읽기 프리셋 테이블 (redesign/04 §2.3, §2.4.2)
//
// "IDLE 에서 다른 센서를 계속 보고 싶다" 를 **커맨드 슬롯으로 푸는 것은 층이 틀렸다.**
// 그건 프레임에 트랜잭션을 끼우는 일이 아니라 **같은 슬롯이 읽는 구간을 바꾸는 일**이다.
// 슬롯을 쓰지 않으므로 tick 소모가 0 이고, control(슬롯 0개)에서도 가능하다.
//
// ## 규칙 (04 §2.4.2)
//
// - 프리셋은 **미리 만들어 둔 것 중에서 고른다.** 200Hz 스레드가 읽는 데이터를
//   런타임에 조립하지 않는다 (write 범위 셀렉터와 같은 방식).
// - **교체는 IDLE 에서만.** RUNNING 중 교체는 발행 메시지의 필드 구성을 바꿔
//   실험 시계열을 중간에 갈라놓는다.
// - wire 예산은 `static_assert` 가 컴파일 타임에 검증한다 — 예산을 넘는 프리셋은
//   애초에 고를 수 없다.
//
// ## 왜 Covers() 가 이 파일의 핵심인가
//
// 프리셋을 바꾸면 **어떤 섀도 필드는 더 이상 갱신되지 않는다.** 그걸 모르고 발행하면
// 낡은 값이 신선한 값처럼 나간다 — 이 프로젝트가 반복해서 지워 온 결함 형태다.
// 그래서 텔레메트리가 매 발행마다 "이 블록을 지금 읽고 있나" 를 물을 수 있어야 하고,
// 안 읽는 자리는 **0 이 아니라 미판독 센티넬**(uint8 0xFF / float NaN)로 나간다.

#include <cstdint>
#include <string>

#include "orin_firmware_bridge/core/rd_comm.hpp"
#include "orin_firmware_bridge/core/rd_register_ecu.hpp"

namespace orin_bridge {

struct ReadSpanEntry { uint16_t addr; uint16_t len; };

// 응답 payload 상한. 응답 Data = err(1) + payload 이므로 payload ≤ Data 크기 - 1.
//
// ⚠ **파생시킨다. 손으로 적지 않는다.** 종전에는 `88` 이 박혀 있었는데, 그것은 STM
//    `PACKET_DATA_BUF_SIZE` 가 90 이던 시절의 값이다. 2026-07-27 에 STM 이 90→256 으로
//    커졌는데(P8) 이 숫자는 따라오지 않아, 실제로는 들어가는 읽기 구간이 컴파일 단계에서
//    거부되고 있었다 — 04 §2.5 가 지적한 "사람이 옮겨 적은 수" 사고와 같은 형태다.
//
// 지금의 병목은 STM(256)이 아니라 **Orin 의 `MAX_DATA_LEN`(248)** 이다. 둘 중 작은 쪽이
// 계약이며, `MAX_DATA_LEN` 이 바뀌면 이 값도 저절로 따라간다.
constexpr uint16_t kMaxRespPayload = static_cast<uint16_t>(MAX_DATA_LEN - 1);   // = 247
constexpr uint8_t  kMaxReadSegs    = 6;   // TaskConfig_t 의 세그먼트 상한과 같다

struct ReadPreset {
    const char*   name;
    ReadSpanEntry spans[kMaxReadSegs];
    uint8_t       count;

    // 응답 payload 합. 세그먼트 개수·크기에서 파생시킨다 (04 §2.2.3) —
    // 프리셋을 추가할 때마다 사람이 더해 적는 일을 없앤다.
    constexpr uint16_t RespPayload() const {
        uint16_t n = 0;
        for (uint8_t i = 0; i < count; i++) n = static_cast<uint16_t>(n + spans[i].len);
        return n;
    }

    // [addr, addr+len) 이 **어느 한 세그먼트에 온전히 담기는가.**
    // 두 세그먼트에 걸쳐 있으면 false 다 — 부분적으로 읽힌 블록을 "읽었다" 고 하면
    // 그 안의 어떤 필드가 낡았는지 알 수 없다.
    constexpr bool Covers(uint16_t addr, uint16_t len) const {
        for (uint8_t i = 0; i < count; i++) {
            if (addr >= spans[i].addr &&
                (addr + len) <= (spans[i].addr + spans[i].len)) return true;
        }
        return false;
    }
};

namespace ecu {

// id 0 — 기본. **09 §1 적용 (2026-08-04)**: SYS 전체 + 센서/모터 연속 두 세그만.
//
// 이 프리셋은 세 번 바뀌었다. 어느 것이 왜 빠졌는지가 전부 다르므로 셋 다 남긴다.
//
//   ① `{27,5}`(구 SYS 일부) → `{16,17}`. `degraded_cnt`·`hw_error`·`hw_fatal` 이 미판독이라
//      엔코더 버스가 100% degraded 인 것을 몇 주 동안 못 봤다 (06 §10 B1). 04 §2.3 변경 1.
//   ② 진단 채널 6개의 `STATE_t` 는 각 센서 블록 **끝바이트**에 흩어져 있다
//      (03 §3.1: RC 87 / RS485 86 / IMU 69 / 모터 127 / 엔코더 85 / 로드셀 47).
//      `{48,80}` 한 세그가 그중 **다섯 개**를 연속으로 덮는다.
//   ③ **로드셀(42:6)과 read-back 20B 를 뺐다 (09 §1, 2026-08-04).** 아래 참조.
//
// ⚠ **로드셀(42:6)은 이제 control 에서 안 읽는다.** 로드셀은 견인 실험의 관심사이고
//    그 구성은 `control_test` 다 — 거기서는 `{42,6}` 을 읽는다. 그래서 로드셀 `STATE_t`(47)
//    를 요구하는 static_assert 도 control_test 쪽으로 옮겼다. `dt_loadcell`·`loadcell_raw`
//    는 control 에서 **미판독(NaN)** 으로 나간다 — 0 으로 두면 "하중 없음" 으로 읽힌다.
//
// ⚠ **read-back 20B(`{128,4}`·`{164,16}`)를 뺐다.** 종전 주석은 "빼면 조용히 망가진다" 고
//    적었는데, 그 둘이 무엇을 보장했는지 다시 보면:
//      `{128,4}`  — SET_CTR_MODE 검증이 이 섀도를 읽었다. **섀도가 곧 write 버퍼**라
//                   (rd_control.cpp `PrepareWrite`) read-back 이 없으면 브리지가 자기가 쓴
//                   값을 다시 읽어 **검증이 무조건 통과한다.** 통과하는 검증은 거짓 보증이라
//                   `DoInSpanCtrMode` 의 검증 자체를 없앴다 (09 §1.2 ①).
//      `{164,16}` — `ControlFeedback.cmd` 가 여기서 나온다. read-back 이 없으면 같은 필드가
//                   자동으로 **"브리지가 보낸 값"** 이 된다 (09 §1.2 ②). 코드는 안 바뀌고
//                   의미가 바뀐다 — `rd_telemetry.cpp` 의 `m.cmd` 주석 참조.
//    쓰기 실패는 read-back 대신 **응답 err → 섀도 롤백**으로 잡는다 (09 §1.2 ③ / U2).
constexpr ReadPreset kPresetControl = {
    "control",
    {{REG_SYS_OFFSET, REG_SYS_SIZE},                   // 16:17 — SYS 전체 (04 §2.3 변경 1)
     {REG_IMU_OFFSET, 80}},                            // 48:80 — IMU~모터 (48~127) 연속
    2
};

// id 1 — 진단. **04 §2.3 + 09 §1.1 적용**: DEFINE+SYS · ctr_mode · CMD_SYSTEM · DIAG 앞부분.
//
// `{0,33}` — memo_260731 은 `{0,16}` 이라 적었지만 **SYS(16:17)를 놓칠 수 없다.**
// "모든 프리셋이 SYS 전체를 읽는다"(04 §2.3 변경 1)가 `NodeStatus` 매핑(03 §5.3)이 보드·
// 모드 분기 없이 성립하는 근거이고, 어긴 결과가 06 §10 B1 의 사고다. 두 구간이 인접하므로
// 합쳐도 세그먼트가 늘지 않고, 커맨드 카탈로그의 `spans::kDiag`(0:33 + 224:32)와도 같은
// 모양이 되어 둘이 갈라지지 않는다.
//
// `{128,4}` — **control 에서는 빠졌지만 여기서는 남는다.** ctr_mode read-back 을 볼 수
// 있는 유일한 프리셋이 되므로, "지금 ECU 가 어떤 ctr_mode 를 갖고 있나" 를 확인하려면
// diag 로 갈아끼운다. (검증 자동화는 09 §1.2 ① 에서 없앴다 — 여기 있는 것은 **관측**이다.)
//
// `{188,4}` — **신규 (09 §1.1).** auto_mode(188) · soft_estop(189) · mode(190) · use_lpf(191).
// TAB3 의 ECU 버튼 3종이 실제 레지스터를 따라갈 수 있는 유일한 경로다. project 프리셋에는
// 없으므로 거기서는 낙관적 스위칭이고, 그 사실을 화면에 표시한다 (09 §5.3 ③).
//
// `{224,16}` — DIAG 32B 중 앞 16B. DIAG 는 `cmd_write_tick`(u32) + 28B 예약이라
// **의미 있는 것은 앞 4B 뿐**이고, 뒤 16B 를 안 읽어도 잃는 정보가 없다.
//
// 모터 블록(88:40)이 빠지므로 **fb_position/velocity/current 는 갱신되지 않는다.**
// 그 자리는 Covers() 를 통해 미판독으로 나간다.
constexpr ReadPreset kPresetDiag = {
    "diag",
    {{0, 33},                                          // 0:33   DEFINE + SYS + proc_delta
     {REG_CMD_MOTOR_OFFSET, 4},                        // 128:4  ctr_mode 관측
     {REG_AUTO_MODE_OFFSET, 4},                        // 188:4  auto_mode/estop/mode/lpf
     {REG_DIAG_OFFSET, 16}},                           // 224:16 cmd_write_tick (+ 예약 일부)
    4
};

// id 2 — 견인 실험. **04 §2.3 적용**: `{16,17}` 로 시작 + 모터 블록 `{88,36}` → `{88,40}`.
//
// `{88,40}` 으로 4바이트 늘린 이유는 모터 `STATE_t`(127)가 그 안에 있어서다 — 종전 36B 는
// 그것을 잘라내 `lc[3]`/`hs[3]`(can1/모터)이 미판독이었다. 견인 실험에서 CAN 상태를 못 보는
// 것은 곤란하다 (2026-07-21 CAN framing err 이력이 있다).
//
// ⚠ **종전의 "traction wire 와 바이트 단위로 같다" 는 불변식은 여기서 끝난다.**
//    그 불변식은 `traction_test_mode` 를 `control`+`auto_mode:none`+`control_test` 로 접을 때
//    회귀를 막기 위한 것이었고, 접기는 이미 끝났다. 골든은 새 배치로 재생성했다.
//
// `{164,16}` 을 남긴다 — 04 §2.3 스케치에는 없지만 **견인 실험의 `cmd` 가 이것이다.**
// auto_mode:none 이라 브리지는 아무것도 쓰지 않고, 전류 명령은 STM RC 램프가 만든다.
// 그 값을 아는 유일한 경로가 이 read-back 이다.
//
// `{128,4}` 는 여전히 **뺀다** — 쓰는 것이 없으니 검증할 read-back 도 없다 (아래 static_assert).
constexpr ReadPreset kPresetControlTest = {
    "control_test",
    {{REG_SYS_OFFSET, REG_SYS_SIZE},                   // 16:17
     {REG_LOADCELL_OFFSET, REG_LOADCELL_SIZE},         // 42:6
     {REG_MOTOR_DATA_OFFSET, REG_MOTOR_DATA_SIZE},     // 88:40 (STATE_t 127 포함)
     {REG_CMD_CURRENT_OFFSET, 16}},                    // 164:16 — 견인의 cmd
    4
};

// project 프레임 전용 (04 §2.3, Q3). ECU RW 슬롯 5칸이 **이 구간을 읽으면서 cmd_vel 을
// 쓴다** — 종전에 센서 READ(100Hz)와 cmd_vel WRITE(50Hz)를 따로 내던 것을 한 트랜잭션으로
// 합친 것이 Q3 의 요점이다.
//
// `{16,17}` 로 시작하는 이유: 04 §2.3 이 **모든 프리셋의 공통 규칙**으로 정했다. SYS 블록을
// 상시 확보해야 NodeStatus 매핑(03 §5.3)이 보드·모드 분기 없이 성립한다. 종전 project 는
// sys 를 10Hz 짜리 별도 슬롯으로 읽었는데, RW 로 합치면서 **매 tick 딸려 온다** —
// 슬롯 하나가 통째로 없어지고 sys 신선도는 10Hz → 100Hz 로 올라간다.
//
// 엔코더(70:16)를 뺀 것은 04 §2.3 의 결정이다 — project 는 링크 각을 쓰지 않는다.
// 그만큼 응답이 작아져(97→81B) RW 로 합쳐도 예산에 여유가 남는다.
constexpr ReadPreset kPresetProject = {
    "project",
    {{REG_SYS_OFFSET, REG_SYS_SIZE},                   // 16:17
     {REG_IMU_OFFSET, REG_IMU_SIZE},                   // 48:22
     {86, 42}},                                        // 86:42 uart2 + RC + 모터 데이터
    3
};

// ⚠ `kPresets[]` 는 **조작자가 런타임에 고를 수 있는 것**들이다 (read_preset 파라미터 /
//    SET_READ_PRESET). project 프리셋은 여기 넣지 않는다 — project 프레임이 자기 구간을
//    FIXED 로 소유하며(rd_slot_table.hpp), 축이 다르다. control 실험용 프리셋을 고르는
//    자리에 프레임 소유 구간이 섞이면 "고를 수 있는 것" 의 정의가 흐려진다 (04 §2.4.2).
constexpr const ReadPreset* kPresets[] = { &kPresetControl, &kPresetDiag, &kPresetControlTest };
constexpr uint8_t kPresetCount = sizeof(kPresets) / sizeof(kPresets[0]);

// 예산 검증은 여기서 끝난다 — 런타임에 다시 볼 필요가 없다 (04 §2.4.2).
static_assert(kPresetControl.RespPayload() <= kMaxRespPayload, "control 프리셋 응답 예산 초과");
static_assert(kPresetDiag.RespPayload()    <= kMaxRespPayload, "diag 프리셋 응답 예산 초과");
static_assert(kPresetControl.count <= kMaxReadSegs, "control 프리셋 세그먼트 수 초과");
static_assert(kPresetDiag.count    <= kMaxReadSegs, "diag 프리셋 세그먼트 수 초과");

// 배치가 바뀌면 여기가 먼저 깨진다 (09 §1.1 의 표). `RespPayload()` 는 파생값이므로
// 이 줄은 "합이 얼마인가" 가 아니라 **"내가 의도한 구간을 넣었는가"** 를 묻는다.
static_assert(kPresetControl.RespPayload()     ==  97, "control     = 17 + 80");
static_assert(kPresetDiag.RespPayload()        ==  57, "diag        = 33 + 4 + 4 + 16");
static_assert(kPresetControlTest.RespPayload() ==  79, "control_test= 17 + 6 + 40 + 16");

// ── 04 §2.3 변경 1: **모든 프리셋이 SYS 전체를 읽는다** ─────────────────────
// 이것이 §2.3 의 첫 번째 결정이고, 지켜지지 않으면 `NodeStatus` 매핑(03 §5.3)이 보드·모드
// 분기 없이 성립하지 않는다. `{27,5}` 처럼 일부만 읽으면 degraded_cnt·hw_error 가 미판독이라
// **고장이 안 보인다** — 엔코더 버스가 100% degraded 인 것을 여태 못 본 이유다 (06 §10).
static_assert(kPresetControl.Covers(REG_SYS_OFFSET, REG_SYS_SIZE),
              "control 이 SYS 전체를 안 읽는다 (04 §2.3 변경 1)");
static_assert(kPresetDiag.Covers(REG_SYS_OFFSET, REG_SYS_SIZE),
              "diag 가 SYS 전체를 안 읽는다");
static_assert(kPresetControlTest.Covers(REG_SYS_OFFSET, REG_SYS_SIZE),
              "control_test 가 SYS 전체를 안 읽는다 (04 §2.3 변경 1)");
static_assert(kPresetProject.Covers(REG_SYS_OFFSET, REG_SYS_SIZE),
              "project 가 SYS 를 놓쳤다 — 그러면 Q3 의 sys 슬롯 제거가 성립하지 않는다");
// proc_delta(32) 는 SYS 안에 있으므로 위가 곧 이것을 보장한다 — 따로 세그를 둘 필요가 없다.
static_assert(REG_PROC_DELTA_OFFSET >= REG_SYS_OFFSET &&
              REG_PROC_DELTA_OFFSET < REG_SYS_OFFSET + REG_SYS_SIZE,
              "proc_delta 가 SYS 밖으로 나갔다 — 프리셋에 별도 세그가 필요해진다");

// ── 04 §2.3 변경 2: control 은 진단 채널의 STATE_t 를 확보한다 ─────────────────
// 03 §3.1 — 각 센서 블록 **끝바이트**에 흩어져 있다. 하나라도 세그 밖이면 그 채널의
// lc/hs 가 미판독으로 나가고, 그건 "정상" 과 구분되지 않는다.
//
// ⚠ **6개 중 5개다** (09 §1.1). 로드셀 STATE_t(47) 는 `{42,6}` 을 읽는 control_test 로
//    옮겼다 — 아래 control_test 절 참조. control 에서 로드셀 lc/hs 는 미판독으로 나간다.
static_assert(kPresetControl.Covers(REG_IMU_OFFSET + REG_IMU_SIZE - 1, 1),
              "control: IMU STATE_t(69) 를 놓쳤다 — idx 2");
static_assert(kPresetControl.Covers(REG_ENCODER_OFFSET + REG_ENCODER_SIZE - 1, 1),
              "control: 엔코더 STATE_t(85) 를 놓쳤다 — idx 4");
static_assert(kPresetControl.Covers(REG_UART2_OFFSET, 1),
              "control: RS485 STATE_t(86) 를 놓쳤다 — idx 1");
static_assert(kPresetControl.Covers(REG_SENSOR_RC_OFFSET, 1),
              "control: RC STATE_t(87) 를 놓쳤다 — idx 0");
static_assert(kPresetControl.Covers(REG_MOTOR_DATA_OFFSET + REG_MOTOR_DATA_SIZE - 1, 1),
              "control: 모터 STATE_t(127) 를 놓쳤다 — idx 3");
// 06 §9.9 의 공백 — Q3 로 엔코더를 읽는 프리셋이 하나도 없어졌던 것을 여기서 되찾는다.
static_assert(kPresetControl.Covers(REG_ENCODER_OFFSET, REG_ENCODER_SIZE),
              "엔코더(70:16)를 읽는 프리셋이 없다 — link_angle 이 전 모드 NaN 이 된다");

static_assert(kPresetControl.Covers(REG_MOTOR_DATA_OFFSET, REG_MOTOR_DATA_SIZE),
              "control 이 모터 블록을 놓쳤다 — fb_position/velocity/current 가 전부 NaN 이 된다");

// ── 09 §1.2: control 은 read-back 을 **하지 않는다** ────────────────────────────
// 되돌아오는 것을 막기 위해 부정형으로 못 박는다. 다시 넣으면 20B 가 늘 뿐 아니라
// `ControlFeedback.cmd` 의 의미가 조용히 "브리지가 보낸 값" → "ECU 가 받은 값" 으로
// 되바뀌고, 그건 필드 소비자가 알 수 없는 변화다.
static_assert(!kPresetControl.Covers(REG_CMD_MOTOR_OFFSET, 4),
              "control 에 ctr_mode read-back 이 다시 들어왔다 (09 §1.2 ①) — "
              "쓰기 검증은 응답 err + 섀도 롤백으로 한다");
static_assert(!kPresetControl.Covers(REG_CMD_CURRENT_OFFSET, 16),
              "control 에 cmd_current read-back 이 다시 들어왔다 (09 §1.2 ②) — "
              "ControlFeedback.cmd 의 의미가 바뀐다");
static_assert(!kPresetControl.Covers(REG_LOADCELL_OFFSET, REG_LOADCELL_SIZE),
              "control 에 로드셀이 다시 들어왔다 — 로드셀은 control_test 의 관심사다");

// diag 는 관측용이므로 반대다 — 여기가 ctr_mode·CMD_SYSTEM 을 볼 수 있는 유일한 자리다.
static_assert(kPresetDiag.Covers(REG_CMD_MOTOR_OFFSET, 4),
              "diag 가 ctr_mode 를 놓쳤다 — control 에서 빠진 뒤로 여기가 유일한 관측 경로다");
static_assert(kPresetDiag.Covers(REG_AUTO_MODE_OFFSET, 4),
              "diag 가 CMD_SYSTEM(188~191)을 놓쳤다 — TAB3 의 ECU 버튼 리드백이 사라진다");
static_assert(REG_AUTO_MODE_OFFSET + 3 == REG_USE_LPF_OFFSET,
              "188:4 가 auto_mode/soft_estop/mode/use_lpf 를 덮는다는 전제가 깨졌다");

// project (Q3) — RW 로 합치므로 요청(쓰기 8B)까지 얹어도 예산 안에 들어야 한다.
static_assert(kPresetProject.RespPayload() <= kMaxRespPayload, "project 프리셋 응답 예산 초과");
static_assert(kPresetProject.count <= kMaxReadSegs, "project 프리셋 세그먼트 수 초과");
static_assert(kPresetProject.RespPayload() == 81, "project = 17 + 22 + 42");
// **sys 를 놓치면 Q3 가 성립하지 않는다** — 종전의 10Hz sys 슬롯을 없앤 근거가 이것이다.
static_assert(kPresetProject.Covers(REG_SYS_OFFSET, REG_SYS_SIZE),
              "project 가 SYS 를 놓쳤다 — 그러면 sys 전용 슬롯을 없앨 수 없다");
static_assert(kPresetProject.Covers(REG_MOTOR_DATA_OFFSET, REG_MOTOR_DATA_SIZE),
              "project 가 모터 데이터를 놓쳤다");
// 엔코더를 뺀 것이 의도임을 못 박는다 (04 §2.3). 실수로 들어오면 응답이 커진다.
static_assert(!kPresetProject.Covers(REG_ENCODER_OFFSET, REG_ENCODER_SIZE),
              "project 는 엔코더를 읽지 않는다 (04 §2.3)");
static_assert(kPresetControlTest.RespPayload() <= kMaxRespPayload, "control_test 응답 예산 초과");
static_assert(kPresetControlTest.Covers(REG_LOADCELL_OFFSET, REG_LOADCELL_SIZE),
              "control_test 가 로드셀을 놓쳤다 — 견인 실험의 본체다");
// control 에서 옮겨 온 요구 (09 §1.1). 로드셀 STATE_t(47)를 읽는 프리셋은 **여기뿐**이다.
static_assert(kPresetControlTest.Covers(REG_LOADCELL_OFFSET + REG_LOADCELL_SIZE - 1, 1),
              "control_test: 로드셀 STATE_t(47) 를 놓쳤다 — idx 5. "
              "control 에서 옮겨 왔으므로 여기서 빠지면 어느 프리셋도 안 읽는다");
static_assert(kPresetControlTest.Covers(REG_MOTOR_DATA_OFFSET, 36),
              "control_test 가 모터 블록을 놓쳤다");
// control_test 는 read-back 이 없다 — auto_mode:none 전용이라는 뜻이다.
static_assert(!kPresetControlTest.Covers(REG_CMD_MOTOR_OFFSET, 4),
              "control_test 에 ctr_mode read-back 이 들어왔다 — 그러면 traction wire 가 바뀐다");

// 이름 -> id. 기동 파라미터·config service 가 쓴다 (01 §4.2 단어형).
inline bool ParseReadPreset(const std::string& s, uint8_t* out) {
    for (uint8_t i = 0; i < kPresetCount; i++) {
        if (s == kPresets[i]->name) { *out = i; return true; }
    }
    return false;
}

}  // namespace ecu
}  // namespace orin_bridge

#endif  // ORIN_FIRMWARE_BRIDGE__RD_READ_PRESET_HPP_
