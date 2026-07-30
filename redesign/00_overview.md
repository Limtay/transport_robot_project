# 00 — orin_ws 재설계 개요 · 용어 · As-Is 진단

> 작성 2026-07-26. 출발점: [HANDOFF_260724.md](../HANDOFF_260724.md) (사용자 아이디어 노트).
> 이 문서는 **"지금 시스템이 무엇인가"를 이해하는 문서**다. 설계 결정은 01~06에서 한다.
> 기존 스펙 진실 원천: [testbed_spec.md](../testbed_spec.md) — 이 재설계는 그 위에 얹는 구조 개편이다.

---

## 1. 이 문서 묶음의 목적

현재 `orin_firmware_bridge` 는 AI가 사용자 요청을 받아 빠르게 확장해 온 코드다. 기능은 실기 검증까지
끝났지만(testbed_spec §6 #0~#9 완료), **사용자가 직접 유지보수·확장하기 어려운 구조**가 되었다.
재설계의 목표는 기능 추가가 아니라 **소유권의 명확화**다.

| 목표 | 구체 기준 |
|------|-----------|
| G1 | 각 layer가 **단일 목적**을 갖는다. "이 기능을 고치려면 어느 파일을 여는가"에 1초 안에 답할 수 있다 |
| G2 | 토픽·서비스·액션이 **모드별로 정의**된다. 미사용 발행/태스크를 제거해 CPU 점유를 낮춘다 |
| G3 | `command_cli` 를 없애고 기능을 통합한다 (현재 CLI가 2개로 갈라져 있다) |
| G4 | 커스텀 메시지를 **프로젝트 전용(mgs01_base_msgs)** 과 **플랫폼 공용(mgs_tp_msgs)** 으로 분리한다 |
| G5 | **웹 기반 제어의 초석** — 웹이 붙을 자리(액션/서비스 계약)를 먼저 확정한다 |

**비목표(이번에 하지 않는 것)**: 제어 알고리즘 변경, ECU 펌웨어 기능 추가(레지스터 순서 조정 제외),
실험 캠페인 진행. 재설계는 **"동작 불변, 구조만"** 을 원칙으로 한다.

### 문서 로드맵

**재설계의 진실 원천은 `00~06` 일곱 문서다.** 이 문서(00)가 전 라운드 결정의 색인이며,
각 결정의 근거와 상세는 해당 번호 문서에 있다. 진행 현황표는 §7.

| # | 문서 | 다루는 것 |
|---|------|-----------|
| 00 | 이 문서 | 용어, As-Is 현황, 문제 진단, To-Be 큰 그림, **결정 색인 Q/R/A/B/C/E** |
| 01 | [01_modes.md](01_modes.md) | 모드 정의·통합 FSM·슬롯 프레임·명령 삽입·shadow 권위 모델·개명표·STM 작업 |
| 02 | [02_layers.md](02_layers.md) | Layer 구조·의존 방향(A1)·파일 배치·데이터 계약·스레드/락 소유권 |
| 03 | [03_interfaces.md](03_interfaces.md) | 시간축 통일 · 진단 배열 · 모드별 topic/service/action · 메시지 패키지 |
| 04 | [04_scheduler.md](04_scheduler.md) | 슬롯 테이블 데이터 형식 · `duration` 규칙 · `GET_STATUS` JSON · CLI 체계 · tick 루프 |
| 05 | [05_profile_record.md](05_profile_record.md) | 프로파일 `mode:` 키, 세그먼트 검증, 웹 드로잉 변환, `result.json` v2, 기록 폴더 |
| 06 | [06_migration.md](06_migration.md) | 작업 분해, 순서, 검증 방법(골든 바이트 + 실기 관문), 롤백 기준, 리스크 |

---

## 2. 용어 사전 — 먼저 이것부터

### 2.1 "모드" 라는 단어의 4중 충돌 ⚠ 가장 중요

현재 코드·문서에서 "모드"는 **4개의 서로 다른 개념**을 가리킨다. 이것이 이해를 막는 1순위 원인이다.

| # | 이름 | 사는 곳 | 값 | 누가 바꾸나 | 결정하는 것 |
|---|------|---------|-----|-------------|-------------|
| 1 | `mode` | ECU 레지스터 **addr 190** | 0=MANUAL / 1=AUTO | RC 스위치(GPIO) **또는** Orin write | ECU가 RC를 듣나, Orin을 듣나 |
| 2 | `auto_mode` | ECU 레지스터 **addr 188** | 0=KINEMATIC / 1=CURRENT / 2=DIRECT / 3=CONTROL | Orin write | AUTO일 때 ECU가 명령을 **어떻게 해석**하나 |
| 3 | `control_mode` / `traction_test_mode` | **브리지 기동 파라미터** | bool | 노드 실행 인자 | 브리지 **스케줄 프레임** 선택 |
| 4 | `TestbedState` | **브리지 내부 FSM** | INIT/IDLE/RUNNING/STREAM/LOCKED | 브리지 자신 | 매 tick **write 값의 출처** |

**핵심 관계**:
- 1번(`mode`)이 AUTO여야 ECU가 Orin 명령을 받는다. MANUAL이면 Orin이 뭘 써도 무시된다.
- 2번(`auto_mode`)이 **브리지의 write 범위를 파생**시킨다 → "ECU가 건드리지 않는 영역만 브리지가 쓴다"
  (testbed_spec §2.6). KINEMATIC이면 ECU가 100Hz로 `ctr_mode`를 덮어써서 브리지와 싸운다 → 금지.
- 3번은 순수하게 Orin 쪽 사정 — 어떤 레지스터를 얼마나 자주 읽고 쓸지의 문제.
- 4번은 **안전 장치** — 값이 아니라 "값의 출처"를 결정한다. RW 트랜잭션 자체는 어느 상태에서도 계속 돈다.

> **재설계 과제 (01에서 확정)**: 이 4개 중 3번(브리지 기동 파라미터)이 bool 2개로 흩어져 있는 것이 문제.
> `bridge_mode: {project | traction | control}` 같은 **단일 enum**으로 접는 것이 HANDOFF의 방향이다.


### 2.2 통신 프리미티브

| 용어 | 뜻 |
|------|-----|
| **레지스터 맵** | ECU의 256바이트 메모리 이미지. 주소로 읽고 쓴다. `rd_register_ecu.hpp` 가 C++ 미러 |
| **shadow (섀도)** | Orin이 들고 있는 레지스터 사본(`RobotState_t`). WRITE는 여기서 퍼가고, READ는 여기에 채운다 |
| **INST** | 패킷 명령어. `PING(1)` / `READ(2)` / `WRITE(3)` / **`RW(4)`** / `REBOOT(8)` |
| **RW** | Write와 Read를 **한 트랜잭션**으로 처리. 200Hz 제어 루프의 핵심 — 명령 반영과 상태 스냅샷이 같은 시점 |
| **멀티세그 READ** | 비연속 구간을 한 번에 읽기. `{27,5}, {42,6}, {88,36}...` 처럼 세그 리스트를 보낸다 |
| **in-span / out-of-span** | 레지스터가 RW의 write 범위 **안/밖**인가. 안이면 shadow만 고치면 다음 tick에 자연 반영. 밖이면(`motor_mask 192`, `mode 190`) **RW 1 tick을 일반 WRITE 패킷으로 대체**해야 한다 |
| **Single Writer 원칙** | control_mode에서 wire(RS485)에 접근하는 주체는 200Hz RW 루프 **하나뿐**. 모든 외부 입력은 shadow만 건드린다 (testbed_spec D6) |

### 2.3 스케줄러 프리미티브

| 용어 | 뜻 |
|------|-----|
| **tick** | 스케줄 루프 1주기. 현재 **5ms (200Hz)** |
| **frame** | tick의 반복 단위. 현재 40 tick = 200ms (5Hz) |
| **슬롯 (slot)** | 프레임 안의 지정석. "이 자리에서는 이 태스크를 쏜다". 현재 서브슬롯 10칸 순환 |
| **커맨드 슬롯** | 사용자가 임의 명령(READ/WRITE/REBOOT)을 꽂는 4칸. `duration`으로 once/N초/forever |
| **duration** | `0=forever` / `1=once`(RET_OK까지 재시도, 2s 타임아웃) / `2~100`=초 |
| **blackout** | REBOOT 후 3초간 해당 보드 접근 차단 |

### 2.4 Testbed — "테스트베드"가 뭔가

**물리적으로**: 로봇 본체가 아니라 **모터·로드셀을 고정한 시험대**. 전류 명령을 넣고 견인력(로드셀)을
측정해서 `current → traction` 매핑을 구하는 장치다 (analysis/traction 파이프라인의 입력원).

**소프트웨어적으로**: "테스트베드"는 브리지 안의 **실험 실행 계약** 전체를 가리킨다:
- 입력 2경로: **Action**(프로파일 재생, 장기·취소가능) + **Service**(단발 설정, 응답 필요)
- 상태: `TestbedState` FSM (§2.1 4번)
- 관측: `/carrier/testbed/feedback` 200Hz 단일 토픽 — **1 RW 트랜잭션 = 1 메시지**
- 기록: 실험 1회 = 폴더 1개 (bag + profile.yaml + result.json)

> **오해 주의**: "testbed"는 traction 실험 전용 이름처럼 보이지만, 실제로는 **200Hz 결정론적 제어 +
> 실험 기록**이라는 일반 메커니즘이다. 재설계에서 이름을 바꿀 후보다 (01에서 결정).

### 2.5 Profile — "프로파일 방식"이 뭔가

**한 줄 정의**: 시간에 따른 모터 명령 파형을 **YAML 파일로 기술**하고, 브리지가 그것을 재생하는 방식.

**왜 이렇게 하나 (testbed_spec D1)**:
외부 노드가 200Hz로 토픽을 쏘면 DDS 지터 때문에 명령 타이밍이 흔들리고, 놓친 샘플이 생기며,
실험 재현성이 깨진다. 그래서 **파형 자체를 브리지 안으로 밀어 넣는다**.

**동작 흐름**:
```
[사용자] hysteresis_ramp.yaml 작성
   │
   │  testbed_cli run hysteresis_ramp.yaml --record
   ▼
[Action goal]  profile_yaml 전문(全文)을 문자열로 전송
   ▼
[브리지] ① YAML 파싱
        ② 검증  — 지정 모터 ⊆ active_motors? 전류 ≤ 클램프? slew_rate 위반?
                   위반 시 goal 즉시 reject (몰래 수정하지 않는다 — 프로파일 = 실험 기록)
        ③ 사전 샘플링 — 전 구간을 200Hz 배열로 **미리** 펼침 (float[4][N])
        ④ FSM: IDLE → RUNNING
   ▼
[200Hz tick] samples_[m][tick] 을 **배열 인덱싱만** 해서 shadow에 씀
             ※ 재생 중에는 파싱도 수식 연산도 하지 않는다 (5ms 예산 보호)
   ▼
[종료] tick 소진 → IDLE 복귀 → Action result (ticks_executed, clamp_cnt, write_err_cnt)
```

**YAML 세그먼트 타입** (9종):
`hold` / `ramp` / `stair` / `step` / `sine` / `chirp` / `prbs` / `noise` / `custom`

```yaml
name: hysteresis_ramp_v1
limits: {max_current: 25.0, slew_rate: 50.0}
motors:
  m2:
    - {type: hold, duration: 3.0, value: 0}        # 무부하 영점 (분석 tare 구간)
    - {type: ramp, duration: 20.0, from: 0, to: 10}
    - {type: hold, duration: 3.0, value: 10}
    - {type: ramp, duration: 20.0, from: 10, to: 0}
  m3:
    - {type: hold, duration: 46.0, value: 0}
```

- **`custom`** 이 웹 UI의 목적지다: 웹에서 그래프를 마우스로 그리면 → 샘플 배열 → `custom` 세그 →
  YAML → 액션 goal. 즉 **"바 형태 제어"와 "프로파일 제작"은 같은 파이프의 두 입구**다 (HANDOFF 요구사항).
- **현재 제약**: v1 player는 **CURRENT[A] 전용**. → **E1 로 해소**: `mode:` 키를 신설해
  `current|velocity|position` 을 선언하고, 브리지는 `auto_mode` 와 맞는지 **검사만** 한다 (05 §2).

### 2.6 기록 (goal_id · 폴더 규격)

- `goal_id`: 세션 내 단조증가 seq. 피드백 토픽에 태그되어 **분석이 bag을 실험 단위로 자동 분할**하는 키.
- 노드 재시작하면 1부터 다시 매겨진다 → **실험의 전역 키 = (폴더명, goal_id) 쌍**. bag 간 goal_id 값 자체를 비교 금지.
- 폴더 규격 (CLI·웹 공통 계약) — **아래는 코드 실제 동작**. 스펙 문서와 어긋나 있던 것을 E5/E6 이
  코드 쪽으로 정본화했다 (05 §5.1, §6.2):
  ```
  data/rosbags/<name>_<MM-DD_HH-MM>/          # 연·초는 result.json 의 started_at 이 갖는다
    bag/          # 개명 후 /carrier/control/feedback 1개 (--diag 시 comm_diag 추가)
    profile.yaml  # 제출한 원본 사본
    result.json   # schema v2 — 05 §5.2
    console.log   # 선택
  ```

---

## 3. As-Is 현황 지도

### 3.1 패키지 구성

| 패키지 | 언어 | 실행 파일 | 역할 |
|--------|------|-----------|------|
| `orin_firmware_bridge` | C++ | `comm_test_node` | **본체** — RS485 브리지 + 스케줄러 + ROS 인터페이스 |
| " | C++ | `command_cli` | 대화형 REPL. `/carrier/command_set` 호출 (312줄) |
| `mgs01_base_msgs` | IDL | — | msg 4 + srv 2 + action 1 |
| `testbed_cli` | Python | `testbed_cli` | 원샷 CLI. testbed config/action 호출 + bag 기록 |
| `carrier_teleop` | Python | `keyboard_teleop` | 키보드 `cmd_vel` |

### 3.2 브리지 파일별 역할·규모

| 파일 | 줄 | 역할 | 진단 |
|------|-----|------|------|
| `rd_uart.cpp` | 205 | 시리얼 포트 open/read/write | ✅ 단일 목적 |
| `rd_comm.cpp` | 203 | 패킷 프레이밍 + CRC16 + 에러 카운터 | ✅ 단일 목적 |
| `rd_map.cpp` | 267 | Encode/Decode — 레지스터 ↔ 패킷 | ✅ 단일 목적 |
| `rd_clock_sync.cpp` | 90 | ECU tick ↔ ROS time offset/drift 추정 | ✅ 단일 목적 |
| `rd_profile.cpp` | 388 | YAML 파싱 → 검증 → 사전 샘플링 | ✅ 단일 목적 |
| `rd_testbed.cpp` | 134 | FSM (INIT/IDLE/RUNNING/STREAM/LOCKED) | ✅ 단일 목적 |
| `rd_command.cpp` | 394 | 커맨드 슬롯 4칸 + jeongae 자동 시퀀스 FSM | ⚠ **2개 목적 혼재** |
| `rd_schedule.cpp` | 564 | tick 루프 + 슬롯 디스패치 + INIT 플로우 + RT 스케줄링 | ⚠ 모드 3중 분기 |
| **`rd_bridge.cpp`** | **1162** | ROS 노드 전부: 47 pub / 2 sub / 3 srv / 1 action / 타이머 2개 / 프로파일 실행 스레드 / out-of-span 상태기계 | ❌ **God object** |
| `command_cli.cpp` | 312 | 별도 REPL 노드 | ❌ testbed_cli와 중복 |

### 3.3 현재 발행 토픽 — 총 47개

| 그룹 | 개수 | 예 | 주기 |
|------|------|-----|------|
| 연결/상태 | 5 | `/carrier/status`, `/carrier/{ecu,dpc,pcu}/connected`, `/carrier/battery/soc` | 10Hz |
| ECU 시스템 | 2 | `/carrier/ecu/fsm`, `/carrier/ecu/alive_time` | 10Hz |
| 모터 피드백 | 6 | `/carrier/ecu/motor/{current/raw, current/filtered, pose, speed, temp, error}` | 100Hz |
| 센서 | 2 | `/carrier/ecu/imu`, `/carrier/ecu/sensor/linkage_angle` | 100Hz |
| 에러 채널별 | 20 | `/carrier/ecu/error/{degraded_cnt,hw_reset,hw_error,hw_fatal}/{uart1,uart2,uart6,can,i2c}` | 10Hz |
| lc/hs 분리 | 6 | `/carrier/ecu/{lc,hs}/{motor,encoder,rc}` | 10Hz |
| 모터 comm_err | 4 | `/carrier/ecu/motor/comm_err/{1..4}` | 10Hz |
| 테스트베드 | 2 | `/carrier/testbed/feedback`, `/carrier/testbed/comm_latency` | 200Hz |

**진단**: 에러 채널 20개 + lc/hs 6개 + comm_err 4개 = **30개가 스칼라 1개짜리 토픽**이다.
HANDOFF의 `uint8 lc[8]` / `uint8 hs[8]` 배열 제안이 이걸 겨냥한 것 — **30개 → 1개 status 메시지**로 접힌다.

### 3.4 구독 / 서비스 / 액션

| 종류 | 이름 | 타입 | 소비처 |
|------|------|------|--------|
| Sub | `/carrier_cmd_vel` | `geometry_msgs/Twist` | 50Hz WRITE 180~187 |
| Sub | `/jeongae` | `mgs01_base_msgs/JeonGae` | 자동 전개 시퀀스 트리거 |
| Srv | `/carrier/command_set` | `CommandSet` | 커맨드 슬롯 SET/RESET |
| Srv | `/carrier/jeongae_lock` | `std_srvs/SetBool` | 전개 잠금 |
| Srv | `/carrier/testbed/config` | `TestbedConfig` | 6개 op (motors/ctr_mode/mode/rearm/status/auto_mode) → 재설계 후 7 op (03 §6.4) |
| Act | `/carrier/testbed/run_profile` | `RunProfile` | 프로파일 재생 |

### 3.5 기동 파라미터

| 파라미터 | 기본 | 설명 |
|----------|------|------|
| `control_mode` | false | 200Hz RW 프레임 |
| `traction_test_mode` | false | 200Hz 배치 READ 프레임 — **HANDOFF에서 삭제 대상** |
| `auto_mode` | 1 (CURRENT) | 0·3은 기동 거부 |
| `active_motors` | [1,2,3,4] | **실제 연결된 모터와 정확히 일치해야 함** (불일치 시 FAULT) |
| `cmd_current_max` | 30.0 | 전역 전류 클램프 [A] |
| `cmd_vel_guard_enable` | true | 100ms 미수신 / 3초 0 수렴 시 WRITE skip |
| `cmd_vel_topic_timeout` / `cmd_vel_zero_timeout` | 0.1 / 3.0 | 위 가드 임계 |
| `imu_frame_id` | "imu_link" | — |

### 3.6 현재 스케줄 프레임 — 3가지가 공존

**(A) 일반 모드** (`control_mode=false`, `traction_test_mode=false`) — 40 tick 프레임
```
짝수 tick        : 100Hz  ECU READ  48~127 (80B, IMU+ENC+UART2+RC+MOTOR)
홀수 tick (짝)   :  50Hz  ECU WRITE 180~187 (8B, cmd_lin_vel/cmd_ang_vel)
홀수 tick (홀)   : 서브슬롯 10칸 순환 (200ms 1회전)
                   [E10, PCU, DPC, C1, C2, E10, PCU, DPC, C3, C4]
                   E10 = ECU READ 16~31 (sys 영역)  ※ 코드 주석의 "46~61" 은 stale
                   PCU/DPC = 레지스터 미정 → enable 플래그로 기본 OFF
                   C1~C4 = 커맨드 슬롯 (각 5Hz)
```

**(B) traction 모드** — 매 tick 200Hz 멀티세그 READ (5세그, 응답 65B)
`{27,5} {42,6} {88,36} {164,16} {228,1}`

**(C) control 모드** — 매 tick 200Hz **RW** (write 범위는 `auto_mode` 파생)
```
auto_mode=1 CURRENT : write 164:16  → 요청 43B / 응답 69B
auto_mode=2 DIRECT  : write 128:52  → 요청 79B / 응답 69B
read 6세그 공통     : {27,5} {42,6} {88,36} {164,16} {228,1} {128,4}
```

**진단**: (B)는 (C)의 write 없는 버전에 불과하다 → HANDOFF의 "traction_test_mode 삭제" 판단이 맞다.
그리고 (A)와 (C)는 **읽는 구간이 거의 겹치는데 완전히 다른 코드 경로**를 탄다. 이것이 04에서
"슬롯 테이블 하나로 통합"해야 하는 이유다.

---

## 4. 문제 진단 — 왜 손대기 어려운가

| # | 문제 | 증상 | 근본 원인 |
|---|------|------|-----------|
| P1 | **God object** `rd_bridge` 1162줄 | 토픽 하나 추가하려면 이 파일을 연다. 프로파일 실행 스레드·out-of-span 상태기계·IMU 단위변환·EMA 필터가 한 클래스 안에 있다 | ROS 노드 = 모든 ROS 인터페이스의 집합, 이라는 잘못된 등식 |
| P2 | **모드 4중 충돌** (§2.1) | 문서를 읽어도 "어떤 모드"인지 매번 되짚어야 함 | 이름 공간 분리 실패 |
| P3 | **CLI 이중화** | `command_cli`(C++ REPL, 레지스터 디버깅) vs `testbed_cli`(Python 원샷, 실험). 기능이 겹치기 시작함 | 용도가 다르다고 판단해 분리했으나, 실제로는 "브리지에 명령 보내기"라는 같은 일 |
| P4 | **토픽 난립 47개** | `ros2 topic list` 가 읽기 어렵고, 30개가 스칼라 1개짜리. DDS 발행 오버헤드 | 레지스터 필드 하나 = 토픽 하나 라는 매핑 |
| P5 | **모드별 코드 3중 분기** | `RunLoop()` 안에 traction/control/일반 if-else. 새 모드 추가 = 분기 추가 | 슬롯 테이블이 데이터가 아니라 코드로 존재 |
| P6 | **진단 정보 stale** | control 모드에서 `hw_error`·`degraded_cnt` 가 배치 READ 범위 밖이라 **거짓 정상** 표시 | 읽기 범위가 모드마다 하드코딩 |
| P7 | **메시지 패키지 혼재** | `mgs01_base_msgs` 에 프로젝트 전용(`JeonGae`)과 플랫폼 공용(`TestbedFeedback`, `CommLatency`)이 섞임 | 분류 기준 부재 |
| P8 | **DIRECT 전환 크래시** (이월 버그) | `config auto_mode 2` → 128:52 RW가 RD_FATAL → 재시작 → SEGFAULT (재현 2/2) | 미조사. STM RS485 52B RX 수용 여부 + 재시작 경합 의심 |

---

## 5. To-Be 큰 그림

### 5.1 핵심 원칙 (재설계 전체를 관통)

| # | 원칙 | 의미 |
|---|------|------|
| **R1** | **모드는 설정(데이터)이지 코드 분기가 아니다** | 모드 = "어떤 슬롯 테이블을 로드하는가". 새 모드 추가 = 테이블 추가, `if` 추가 아님 |
| **R2** | **Single Writer 유지** | wire 접근은 스케줄 스레드 단독. 외부 입력은 shadow만 수정 |
| **R3** | **shadow 소유권** | write 범위 **안**만 브리지가 쓴다. 범위 밖 shadow 자리에는 ECU 실값(read)이 들어 있고, 그것이 검증의 유일한 진실 원천 |
| **R4** | **ROS 인터페이스는 코어를 모른다** | 코어(스케줄·맵·통신)는 rclcpp 의존 없이 단위 테스트 가능해야 한다 |
| **R5** | **1 트랜잭션 = 1 메시지** | 200Hz 피드백은 같은 RW에서 나온 값만 담는다. 보간·합성 금지 |
| **R6** | **동작 불변, 구조만** | 리팩터링 단계마다 `colcon test` 65케이스 통과로 증명 |

### 5.2 레이어 스케치 (상세는 02에서)

```
┌─────────────────────────────────────────────────────────────┐
│ L4  클라이언트   testbed_cli (통합) / testbed_web / AI 스크립트│  ← 프로세스 밖
└───────────────────────────┬─────────────────────────────────┘
                            │ Action / Service / Topic  (계약: 03)
┌───────────────────────────▼─────────────────────────────────┐
│ L3  ROS 인터페이스 계층                                        │
│     rd_node(셸) · rd_telemetry(발행) · rd_testbed_api(실험)    │
│     · rd_carrier_api(프로젝트 태스크)                          │
│     ※ rclcpp 를 아는 유일한 층. 상태를 소유하지 않는다           │
└───────────────────────────┬─────────────────────────────────┘
                            │ 순수 C++ 인터페이스 (rclcpp 없음)
┌───────────────────────────▼─────────────────────────────────┐
│ L2  응용/정책 계층                                            │
│     rd_testbed(FSM) · rd_profile(재생) · rd_command(슬롯)     │
│     · rd_sequence(jeongae 전개 — rd_command 에서 분리)        │
└───────────────────────────┬─────────────────────────────────┘
┌───────────────────────────▼─────────────────────────────────┐
│ L1  스케줄 계층   rd_schedule (200Hz tick, 슬롯 테이블 실행)   │
│                   + rd_slot_table (모드별 데이터)             │
└───────────────────────────┬─────────────────────────────────┘
┌───────────────────────────▼─────────────────────────────────┐
│ L0  전송 계층     rd_map(Encode/Decode) · rd_comm(패킷)       │
│                   · rd_uart(시리얼) · rd_clock_sync           │
└─────────────────────────────────────────────────────────────┘
```

**의존 방향은 위 → 아래 단방향.** 현재 `rd_schedule` 이 `rd_bridge`(L3)를 직접 참조하는 역방향
의존이 있는데(`bridge_node_->IsControlMode()`, `PublishTestbedFeedback()`), 이것이 P1·P5의 구조적 뿌리다.
→ **콜백 인터페이스로 뒤집는다** (02에서 설계).

### 5.3 모드 taxonomy 초안 (01에서 확정)

| 모드 | 목적 | tick | ECU auto_mode | 주 인터페이스 |
|------|------|------|---------------|---------------|
| `project` | 실차 운용 (주행 + 전개) | 5ms, 슬롯 순환 | KINEMATIC | `/carrier_cmd_vel`, `/jeongae` |
| `control` | 200Hz 결정론 제어 (실험·MPC) | 5ms, 매 tick RW | CURRENT / DIRECT | `run_profile` action, `testbed/config` |
| `manual` | 진단·수동 (RC 주행 관찰) | 5ms, READ 위주 | — | 커맨드 슬롯 |

`traction` 은 `control` + `auto_mode=CURRENT` + 프로파일로 **완전히 대체**된다 → 모드에서 삭제.

---

## 6. 기반 결정 Q1~Q8 — 확정 (2026-07-26)

상세는 [01_modes.md](01_modes.md) §1.

| # | 질문 | 확정 |
|---|------|------|
| Q1 | 브리지 모드 표현 | **단일 enum** `bridge_mode: project\|control\|manual`. `traction_test_mode` 삭제 |
| Q2 | 런타임 모드 전환 | **기동 시 고정** (변경 = 재시작. ECU `AUTO_TIMEOUT` 이 자동 정지시키므로 이미 안전 경로) |
| Q3 | 슬롯 10칸의 내역 | **ECU RW×5 + DPC×1 + PCU×1 + Command×3** (50ms 프레임 → ECU 100Hz, 나머지 20Hz) |
| Q4 | `command_cli` 통합 | **삭제**. raw `reg` 서브커맨드는 만들지 않음 — 명령은 상위에서 의미 단위로 조립 |
| Q5 | 메시지 패키지 | `mgs01_base_msgs`=`JeonGae` 만 / `mgs_tp_msgs`=나머지 전부 (`CmdMotor` 포함) |
| Q6 | "testbed" 이름 | **`control` 로 개명** (01 §7 개명표) |
| Q7 | 웹 착수 | 계약(03) 먼저, 구현은 리팩터링 후 |
| Q8 | STM 레지스터 재배치 | 채택 — SYS 영역 16→**17B**, `rs485_proc_delta` → **addr 32** (01 §8) |

**Q8 정정**: HANDOFF 노트는 `rs485_proc_delta` 를 addr 29로 적었으나 `realtime_tick`(4B)이
addr 28~31을 점유하므로 겹친다. 올바른 주소는 **addr 32**이며, 이때 Control task 읽기 세그
`{26,7}` 과 `{16,17}` 이 정확히 맞아떨어진다 — HANDOFF의 의도가 이 배치임이 역산으로 확인됐다.

**Q3 추가 요구사항 (사용자)**: CLI가 **어떤 `bridge_mode` 에서도 중간에 사용자 명령을 넣을 수** 있어야 한다.
단 WRITE는 안전 정지 확인 후. → 01 §6.2(`safe_stop` 술어) / §6.3(명령 삽입 3종 분류)에서 설계 완료.

### 2차 라운드 R1~R6 (2026-07-26)

| # | 결정 |
|---|------|
| R1 | `developer_mode` 파라미터 **삭제** → `manual` 모드에 흡수 (축 하나 감소) |
| R2 | `manual` 재정의: write 전면 거부 → **"자동 설정이 전무한 고급 수동 모드"** |
| R3 | 기동 조합 거부 규칙 **6개 → 2개** (write 범위 파생 원칙이 겹침을 구조적으로 배제) |
| R4 | 읽기 프리셋에 read-back 세그 **넣지 않는다** → **shadow 권위 모델**로 대체 (01 §7) |
| R5 | STM 버퍼 **90 → 256** (+RX 64→256, TX 128→272) — **P8 근본 수정** |
| R6 | 정기 READ = **IDLE 전용**. RUNNING 중에는 1회성 READ만 |

**P8(DIRECT 전환 크래시) 원인 규명 (2026-07-26)**: `RX_BUFFER_SIZE=64` DMA 링버퍼에 DIRECT RW
요청 87B가 들어오면 `rx_length = (tail-head+64) % 64` 가 **23으로 계산**되어 패킷이 잘린다.
CURRENT 요청은 51B라 통과한다 — 재현 조건과 정확히 일치. 브리지 경합이 아니라 **STM 버퍼 문제**.
상세: 01 §9.2.

---

### 3차 라운드 A1~A5 (2026-07-26) — 레이어

| # | 결정 |
|---|------|
| A1 | 역방향 의존 22곳을 **성격별 3종 처방**으로 뒤집는다 (설정=주입 / 통지=`ITelemetrySink` / 정책=L2 직접 참조) |
| A2 | L3 4분할 (`rd_node`/`rd_telemetry`/`rd_control_api`/`rd_carrier_api`) + L2 신설 2개 (`rd_profile_player`/`rd_oos`) |
| A3 | `rd_command` → 슬롯 관리 + `rd_sequence`(jeongae) 분리 |
| A4 | 프로파일 액션 스레드 → 타이머 흡수. **200Hz 발행을 락프리 큐 + 전용 스레드로 분리** (DDS 블로킹 차단) |
| A5 | `ILogger` + `IClock` 추상화 → L0/L1/L2 를 rclcpp 없이 유닛 테스트 |

**핵심 강제 수단**: CMake 를 `rd_core_lib`(rclcpp 링크 **안 함**) / `rd_ros_lib` 두 개로 나눠
계층 위반을 **링크 에러로 차단**한다. 문서 규약은 잊히지만 빌드 실패는 잊히지 않는다. → 02 §4.1

---

### 4차 라운드 B1~B6 (2026-07-26) — 인터페이스

| # | 결정 |
|---|------|
| B1 | `CommLatency` 흡수 + **목적 재정의** — 지연 계산이 아니라 **시간축 통일**. `header.stamp` 를 "ECU 취득 기준시각을 Orin 시간축으로 변환한 값"으로 바꾸고, 센서별 `dt_*` 를 함께 발행 |
| B2 | 진단 인덱스 규약 통일(`0=uart1/RC … 5=adc/로드셀`) → **토픽 33개 → `NodeStatus` 1개** |
| B3 | `project` = HANDOFF 계약명 준수(47→6) / `control` = 단일 200Hz custom(49→1) |
| B4 | `mgs01_base_msgs`=`JeonGae` / `mgs_tp_msgs`=나머지 8종 |
| B5 | `SETPOINT` → 서비스 아님, `cmd_motor` 토픽에 **통합** → `write_source` 4→3개 |
| B6 | 커맨드 슬롯 = **의미 단위 명령**, raw 주소는 `manual` 전용 |

**P4(토픽 난립) 해소 규모**: 47개 → `project` 6개 / `control` 1개.

---

### 5차 라운드 C1~C5 (2026-07-26) — 스케줄러

| # | 결정 |
|---|------|
| C1 | 슬롯 테이블 = `constexpr` 데이터. 슬롯 = **ID(대상) × INST(명령) + 읽기·쓰기 구간**. 파생값은 저장하지 않고, 조합 유효성·wire 예산을 **`static_assert`** 로 검증 (2026-07-27 검토 반영) |
| C2 | `once` 만료를 **시간 → 시도 횟수(40회)** 로 변경 — blackout(3s) > once timeout(2s) 이라 REBOOT 직후 명령이 한 번도 시도되지 못하고 죽던 버그 수정 |
| C3 | `GET_STATUS` 정형 JSON — 모든 키 항상 존재, 값 없으면 null, enum 은 문자열 |
| C4 | `control_cli` 가 `command_cli` 흡수. 의미 단위 `cmd` + `manual` 전용 `raw` |
| C5 | DPC/PCU 도 **ECU 와 동일한 SYS 레이아웃(16~32)** 채택 제안 — 📋 **미합의 TODO** (04 §6) |

**2026-07-27 검토에서 함께 확정된 것** (04 §2 전면 개정):

| 항목 | 내용 |
|---|---|
| 슬롯 구조 | `SlotId` × `SlotInst` + **읽기·쓰기 구간을 슬롯이 소유** (현행 `TaskConfig_t` 와 같은 형태) |
| 검사 범위 | `ECU`/`DPC`/`PCU` 는 세 INST 전부 개방. `static_assert` 는 **구조만** 보고 펌웨어 지원 여부는 런타임 거부 |
| 파생값 | `SegCount()`/`RespPayload()`/`CmdCapacity()` — 저장하지 않는다 (어긋날 수 없게) |
| 대체 불가 프레임 | `user_slot_mask == 0` ⇒ 1 tick 대체(OOS)만. `control` 을 특수 케이스로 두지 않는다 |
| 정기 센서 변경 | 커맨드 슬롯이 아니라 **읽기 프리셋 교체** (`SET_READ_PRESET` op 6, IDLE 전용) |
| 읽기 프리셋 | **전부 `{16,17}`(SYS 전체) 로 시작**. `control` 은 `{47,81}` 로 1B 확장 → `lc`/`hs` 6채널 확보 |
| `ControlFeedback` | `lc[8]`·`hs[8]`·`degraded_cnt[8]`·`hw_*` 추가 (246B/200Hz). 미판독 채널은 `0xFF` |

**POSITION 제어 시나리오 검토에서 확정된 것** (2026-07-27, 01 §3.3·§6.1.2·§6.1.3·§6.3):

`auto_mode` 에 POSITION 이 생기면서 **"안전값 = 0" 이라는 CURRENT 전용 가정**이 세 군데서 깨졌다.
POSITION 에서 0 은 "원점으로 가라" 는 명령이다.

| 항목 | 확정 |
|---|---|
| IDLE·LOCKED 안전값 | `auto_mode` 별 표로 확장. **POSITION = `fb_position` 매 tick 재시드**(bumpless transfer) |
| shadow 소독 | `→DIRECT` 전용이던 규칙을 **전 전환에 일반화**. POSITION 진입 소독은 IDLE 재시드가 겸한다 |
| `STREAM` 진입 | 메시지 도착만으로 RUNNING 이 되던 것을 **명시적 arm(웹 run/stop, op 7)** 요구로 변경 |
| `STREAM` 스테일 | `LOCKED` → **`IDLE` + arm off**. (초안은 01 과 03 이 서로 어긋나 있었다) |
| `SET_ORIGIN` | **`in-span 1회성 WRITE`(펄스)** 분류 신설. `manual` 전용 제약 해제. **자동화하지 않는다** |
| 정지 워치독 | **4층으로 정리** — 브리지 FSM / Orin 50ms / ECU `AUTO_TIMEOUT` 100ms / **AK 모터 내부 200ms**. 마지막 층은 사용자가 모터에 직접 설정한 값으로 코드·문서 어디에도 없었다 |
| 웹 범위 | 자세 표시 → (선택) 원점 → 슬라이더 정렬 → **run/stop** → 조작. **그 이상 넣지 않는다.** 진입 변위 검사도 하지 않고 조작자 책임으로 둔다 (01 §6.1.3) |

---

### 6차 라운드 E1~E6 (2026-07-27) — 프로파일 · 기록

| # | 결정 |
|---|------|
| E1 | 프로파일 **`mode:` 키 신설** (`current\|velocity\|position`). 브리지는 수락 시 **검사만** 하고 `auto_mode` 를 대신 바꾸지 않는다 |
| E2 | 세그먼트 길이 검사를 **확장 전으로 이동** + 타입별 상·하한 확정 (현행 파서 구멍 6건 차단) |
| E3 | `noise` 는 slew 검사 **면제**. `testbed_spec §4.2` 의 "`noise` 는 `slew_rate` 필수" 문구 **철회** — 두 규칙을 함께 지키면 백색잡음이 정의상 항상 거부된다 |
| E4 | 웹 드로잉 → **`ramp`/`hold` 세그먼트 열** (`custom` 아님). `custom` 보간 기본 **linear** |
| E5 | `result.json` **schema v2** — 코드 쪽 키 이름 정본 + 11개 추가. `RunProfile.action` Result 확장이 선행 조건 |
| E6 | 기록 폴더 구조·폴더명 **현행 유지**(자산 호환). 녹화 토픽 1개로 개명, `verify_run_dir` 에 sha256 대조 추가 |

**E1 이 드러낸 것**: `rd_bridge.cpp:604~609` 의 wire 직전 클램프가 **단위를 모른다** — 무조건
`cmd_current_max_`(30.0)로 자르므로 velocity 프로파일이면 RPM 지령이 30 으로 잘린다. 검증과
클램프가 서로 다른 단위계를 쓰는데 둘 다 통과해 에러도 나지 않는다. 상세: 05 §2.1.

---

### 7차 라운드 F1~F6 (2026-07-27) — 마이그레이션

| # | 결정 |
|---|------|
| F1 | 증명 수단 = `colcon test` + **골든 바이트 테스트**(신규, 0단계에서 생성). 실기는 **최종 1회** |
| F2 | 05 스키마 변경은 **7번(메시지 패키지)에 포함** — 정의만 하고 값 채우기는 생산 단계에 붙인다 |
| F3 | 개명은 **한 커밋**. 별칭 기간 없음 (두 이름 공존 기간의 조합을 검증할 필요가 없다) |
| F4 | 단계 = 커밋 1개, 롤백 = `git revert` 1회. **골든 불일치는 예외 없이 롤백** |
| F5 | STM 4건은 **병행**. 실기 세션 안에서 **G1(신STM+구브리지) → G2(통합) → G3(신규기능)** 으로 실패 도메인 분리 |
| F6 | 구 분석 분기는 G2 통과 즉시 폐기. `analysis-legacy-format` **태그로 보존** |

**F1 이 드러낸 것**: 현행 테스트는 `test/rd_test_fixture.hpp` 가 **실제 `RdBridge` 노드를 띄운다.**
그런데 이번 재설계는 그 `RdBridge` 를 4개로 쪼개는 일이라 **테스트가 붙어 있는 대상 자체가
사라진다.** 리팩터링 도중 테스트를 같이 고치면 그 테스트는 "안 바뀌었음" 을 증명하지 못한다.
→ 분할 대상이 아닌 **L1(`rd_map`)의 wire 바이트열**을 고정하는 골든 테스트가 필요하다. 06 §2.2.

---

## 7. 문서 진행 현황

| # | 문서 | 내용 | 상태 |
|---|---|---|---|
| 00 | **이 문서** | 용어 사전 · As-Is · 문제 진단 P1~P8 · To-Be 원칙 · **전 라운드 결정 색인** | ✅ |
| 01 | [01_modes.md](01_modes.md) | `bridge_mode` 정의 · 통합 FSM · 슬롯 프레임 · 명령 삽입 규칙 · shadow 권위 모델 · STM 작업 | ✅ |
| 02 | [02_layers.md](02_layers.md) | 계층 L0~L3 · 의존 방향 뒤집기 · CMake 강제 · 데이터 계약 · 스레드/락 소유권 | ✅ |
| 03 | [03_interfaces.md](03_interfaces.md) | 시간축 통일 · 진단 인덱스 규약 · 모드별 토픽 · 메시지 전체 정의 | ✅ |
| 04 | [04_scheduler.md](04_scheduler.md) | 슬롯 테이블 · `duration` 규칙 · `GET_STATUS` JSON · `control_cli` 체계 · tick 루프 | ✅ (검토 중) |
| 05 | [05_profile_record.md](05_profile_record.md) | 프로파일 `mode:` 키 · 세그먼트 검증 · 웹 드로잉 변환 · `result.json` v2 · 기록 폴더 | ✅ |
| 06 | [06_migration.md](06_migration.md) | 골든 바이트 테스트 · 10단계 순서 · STM 병행 · 실기 관문 G1~G3 · 롤백 기준 | ✅ |

**이 묶음(00~06)이 재설계의 유일한 진실 원천이다.** 기존 스펙
[testbed_spec.md](../testbed_spec.md) 위에 얹는 구조 개편이며, 두 문서가 어긋나는 곳은
각 문서가 명시적으로 어느 쪽이 정본인지 밝힌다 (예: 05 §5.1 기록 규격, 05 §3.4 `noise` 규칙).

> 이 문서들은 **설계 단계**의 산출물이다. 실제 코드 변경 이력은 코드를 고칠 때
> [Code_modify.md](../Code_modify.md) 에 기록한다 — 문서 작성은 그 로그의 대상이 아니다.
