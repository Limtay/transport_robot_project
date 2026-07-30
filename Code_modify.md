## 수정 목표

- Orin 
  - orin_firmware_bridge
    - 스케줄러 수정 및 Command  수신, 명령 시퀀스 제작
      - 미정인 부분은 TODO로 메모 (DPC, PRA 레지스터 미확정)
    - ECU 54번 레지스터 (reg.sys.hw_reset) 플래그가 올라오면 RCLCPP_ERR 로 띄워서 요청
    - cmd_vel 안정장치 설계
      - jeongae를 포함한 topic이 들어오지 않을 때는 (< 100 ms) 커맨드 전송 skip
      - cmd_vel 의 값이 0에 수렴한지 (> 3 sec) 일 때 커맨드 전송 skip
        - 이 안전장치는 쓸 수도 있고 안쓸 수도 있으니 (on/off ) 로 할 수 있게
  - Command 입력 node 제작
  - 
- STM
  - ECU
    - Register 수정
      - 0 : SYS_WRITE_MODE 는 R/W 가능하고 Unlock으로 따로 빼기
    - reg.def.hw_reset (5번 레지스터) 플래그 올라 오면 해당 하드웨어 reset 후 54번, 5번 둘다 플래그 내리기

## 1. 스케줄러 종류

### 100 Hz loop

- ECU | READ | 62 - 127 (66 byte)

### 50 Hz loop

- ECU | WRITE | 180 - 187 (8 byte)

### 10 Hz loop

- ECU | READ | 46 - 61 (16 byte) 
- PCU | READ | 미정 (SOC, SOH 등)
- DPC | READ | 미정 ( 전개 FSM state 등)

### 5 Hz loop

- command 1
- command 2
- command 3
- command 4
(1,2) (3,4) 합쳐서 10Hz 가능
(1,2,3,4) 합쳐서 20Hz 가능

## 2. Command

### 2.1. command INST

- Set
  - command 명령 세팅
- Reset
  - 공란으로 만들기

### 2.2.  command set

- command # 선택
  - RESET
  - SET
    - 지속 시간 선택 
      - Forever (0)
      - 1회 적용 (1)
        - RET_OK 반환 될 때까지 반복
        - 2 sec 넘어 갈 시 Time out 후 포기
      - 지속 시간 선택 [sec] (범위 2~100 sec)
    - Target id 선택
    - Inst 
      - Write
        - 1번 기능: 현재 입력된 reg 값 전송
          - 시작 주소, 길이 입력
        - 2번 기능: 사용자 데이터 입력
          - 시작 주소, 데이터 입력
            - 데이터 입력시 자동 변환 옵션 있으면 좋을 듯
            Ex) {(float)20, (uint32_t)15 } --> 자동으로 8바이트 입력으로 변환, 전송
      - Read
        - 시작 주소, 길이 입력
        - reg 업데이트 후 해당되는 reg 값 teminal로 띄우기 (RCLCPP_INFO)
      - Reboot
        - 해당 ID 3초간 접근 X ,  보드 상태 off 로 띄우기

### 2.3. 우선 순위 규칙

- 숫자가 낮을 수록 필수 위치
- 스케줄러에서 사용되다가 모든 커맨드가 사용되고 있을 경우 낮은 커맨드가 일시 정지, 명령 처리 후 재개 
- 예시
  - 토픽 명령 Jeongae 가 들어옴
  - 모든 명령이 차있음
  - command 6 명령 일시 정지 후 이 공간 활용하여 진행
  - RET_OK 반환 될 때까지 지속 (timeout 2 sec) 
    - 성공 시 -->(RCLCPP_INFO)
    - 실패 시--> (RCLCPP_ERR)
      - 패킷 에러
      - timeout

### 2.4 MACRO

자주 쓰는 커맨드 매크로로 미리 설정

- 종류
  - ECU hw_reset : 해당 번호 or 통신 포트 입력시 해당 비트 1 플래그 올려서 입력
    - ECU | WRITE | 5   (UART1 | UART2 | UART6 | CAN1 | I2C1 | RSVD)
  - DPC 전방 LED on/off
  - PCU relay off / on / reboot 
  - DPC 공벽 1번칸 2번칸 open/close/default 
  - jeongae lock on/off (orin 기본 변수, default 0)

## 3. Auto command

### 3.1 규칙

- Robot 상태에 따라 자동 할당 되는 커맨드 종류
- 기본 command는 비워진 칸에서 가장 상단에 배치
- 모두 막혀 있는 경우 우선순위 낮은 칸 비우고 세팅 ( 명령 처리 후 이전 명령 복귀)

### 3.2 종류

- Jeongae topic (1) 수신 
  - 전개 시퀀스 
    - Jeongae Unlock check
    - [REG WRITE, Command] ECU soft ESTOP 요청 
      - ECU | WRITE | 189 -> value : 0
        - 성공 시  50 Hz loop 명령 정지
    - [REG READ, Default]  DPC state READ (현재 완전한 상태 인지 check)
    - [REG WRITE, Command] DPC 전개 요청 ( 공벽하고 밑에 따로하는지 필요 )
      - 따로 시 공벽 1번칸 (1) 2번칸 (2), 전개판 (3)
    - [REG READ, Default] DPC state READ 대기 (카메라 위치 까지, LED on)
    - [ROS2 Action]deploy cammera Action
    - ( 4과제 요청오면 더 진행할지 무시하고 진행할지 결정필요)
    - [REG WRITE, Command] DPC 회수 요청
    - [REG READ, Default] DPC state READ 대기 (회수 완료 state)
    - [REG WRITE, Command] ECU soft ESTOP 해제
      - ECU | WRITE | 189 -> value : 1
        - 성공 시  50 Hz loop 명령 시작
    - Jeongae Locking
  - 매 시퀀스 종료시 RCLCPP_INFO, ERR 등 알리기 
  - 각 state 실패 시 어떻게 해야할지 생각이 필요.
  - 이후 들어오는 jeongae topic은 무시, Jeongae unlock 필요

## 4. 명령 간 규칙

- 미정

---

# 수정 메모 (작업 로그)

## 2026-06-11 — 1차 구현 완료 (Claude Code)

### 결정 사항 (사용자 확인)

- Command 입력: **ROS2 Service + CLI 노드** (`/carrier/command_set`) — 분산 환경(Orin=bridge, 노트북=CLI)에서도 토픽과 동일하게 DDS 로 동작 확인
- 커맨드 슬롯: **5Hz × 4슬롯** (문서 기본안)
- cmd_vel 안전장치: **기본 ON**, `cmd_vel_guard_enable` 파라미터로 런타임 토글 가능
- Auto command(jeongae): **FSM 골격까지 구현**, DPC/카메라 단계는 TODO skip

### Orin (orin_firmware_bridge) 변경

- **스케줄러 재구성** (`rd_schedule.cpp`): 200Hz tick, 200ms 프레임
  - 짝수 tick: 100Hz ECU READ 62~127 (66B)
  - 홀수 tick 절반: 50Hz ECU WRITE 180~187 (8B)
  - 나머지 홀수 tick 10개 순환: [E10, PCU, DPC, C1, C2, E10, PCU, DPC, C3, C4]
  → ECU sys READ 46~61 10Hz / PCU·DPC READ 10Hz (**레지스터 미정 — enable 플래그로 기본 OFF, TODO**) / 커맨드 슬롯 4개 각 5Hz
- **RdCommand 신규** (`rd_command.cpp`): 슬롯 SET/RESET, duration(forever/once/2~100s), once는 RET_OK까지 5Hz 재시도+2s timeout, REBOOT 시 3초 blackout+보드 off 표시, 우선순위 규칙(§2.3: 꽉 차면 최하위 슬롯 일시정지→복귀), jeongae 전개 시퀀스 FSM(§3)
- **cmd_vel 안전장치** (`rd_bridge.cpp::ShouldSkipCmdWrite`): ①jeongae 포함 토픽 100ms 미수신 ②cmd_vel 0 수렴 3초 → 50Hz WRITE skip. 파라미터: `cmd_vel_guard_enable`(런타임 토글), `cmd_vel_topic_timeout`, `cmd_vel_zero_timeout`
- **reg54 hw_reset 감지**: PublishStatus(10Hz)에서 비트 감지 시 RCLCPP_ERROR(1s throttle)로 채널명 + 조치(macro hw_reset) 안내
- **CLI 노드 신규** (`command_cli`): set/reset/macro 파싱, `(f)20.5 (u32)15` 자동 형변환(little-endian). 매크로: `hw_reset <ch>`, `jeongae_lock on/off` 구현 / `dpc_led`, `pcu_relay`, `gongbyeok` 은 레지스터 미정 TODO
- **RdMap**: REBOOT/PING inst encode/decode 지원 추가
- **unlock 자동화**: 스케줄러 Init 시 ECU addr0=UNLOCK 전송 + 50Hz WRITE 연속 10회 실패 시 unlock 재전송 (ECU 재부팅 후 LOCK 복귀 자가복구)
- `mgs01_base_msgs`: `srv/CommandSet.srv` 추가
- colcon 빌드 통과 (경고 0)

### STM (ECU_V3) 변경 — ⚠ STM32CubeIDE 빌드 검증 필요

- `**rd_map_ecu.c` unlock 버그 수정**: `check_region_range()` 의 `!r->needs_unlock` 조건이 반전되어 있었음 — 기존엔 unlock 불필요 영역이 LOCK에 막히고 CMD 영역(128~191)은 무조건 쓰기 허용되던 상태. `r->needs_unlock && LOCK → ACCESS 거부` 로 수정
- **SYS_WRITE_MODE 분리**: region LUT 에서 addr0 을 단독 영역(항상 R/W)으로 분리, DEFINE 1~~15 는 needs_unlock=1 로 변경 → **이제 CMD 영역 및 DEFINE(1~~15) 쓰기는 UNLOCK 선행 필수** (Orin 쪽 자동 unlock 으로 대응)
- `**rd_system.c` addr5 처리 신규**: `RD_SYSTEM_HW_RESET_HANDLE()` — systemTask(100Hz) 에서 reg.def.hw_reset(addr5) 비트 검사 → 채널별 RECOVERY 호출(uart1/uart2/uart6/can/i2c) + fatal 카운터 리셋 → addr54(hw.reset)/addr5 플래그 동시 클리어

### 미확정 / TODO (다음 작업)

- PCU READ 레지스터 확정 (SOC, SOH 등) → `task_10hz_pcu`_ 주소 갱신 + `enable_pcu_read_` 활성화
- DPC READ 레지스터 확정 (전개 FSM state) → `task_10hz_dpc_` + jeongae 시퀀스 DPC 단계 구현
- jeongae 시퀀스: DPC 전개/회수 요청 레지스터, 공벽(1)(2)/전개판(3) 구분, deploy camera Action, 4과제 정책
- 각 시퀀스 state 실패 시 정책 (현재: ESTOP 해제 시도 후 jeongae lock 걸고 중단)
- soft ESTOP 용 addr189(ctr_flag) 의 ECU 측 의미 확인 — 문서상 189=0 요청이나 현재 ECU 코드에서 189는 direct/kinematics 선택 플래그
- dpc_led / pcu_relay / gongbyeok 매크로 (레지스터 미정)
- STM 빌드: CubeIDE 에서 컴파일 + 실기 검증 (이 노트북엔 arm 툴체인 없음)

### 참고 (동작 변경)

- 에러 토픽 채널명 `uart4` → `uart6` 변경 (STM idx2 가 IMU/uart6 인 것과 정렬): `/carrier/ecu/error/*/uart6`
- 기존 스케줄의 slot9 커스텀 커맨드(`PushCustomCommand`)는 커맨드 슬롯 4개 체계로 대체·제거
- CLI 실행: `ros2 run orin_firmware_bridge command_cli` (노트북에서 실행 시 같은 ROS_DOMAIN_ID 필요)

## 2026-06-11 — 2차 수정 (사용자 피드백 반영)

### 1. 잠금 범위 축소: DEFINE 영역만 잠금

- STM `rd_map_ecu.c`: CMD_MOTOR(128~~179)·CMD_SYSTEM(180~~191) needs_unlock → **0** (unlock 불필요).
잠금은 **DEFINE 1~15 만** 유지, addr0(sys_write_mode)은 항상 R/W (unlock 키)
- Orin 스케줄러의 자동 unlock(Init 시 전송 + 50Hz 실패 시 재전송) **제거** — cmd_vel 쓰기에 더 이상 불필요
- DEFINE 쓰기가 필요한 매크로 대응:
  - CLI `macro unlock <on|off>` 신규 (reg0 write)
  - CLI `macro hw_reset <ch>` 는 unlock(reg0=1) → reg5 write → lock(reg0=0) 3단계 자동 체인 (0.5s 간격)

### 2. addr189(구 ctr_flag) → Orin 용 soft ESTOP 재정의

- `ctr_flag` 는 코드상 사용처가 주석 처리되어 있던 미사용 필드 → `**soft_estop`** 으로 개명
  - 값: **0 = ESTOP 작동** (AUTO 모드 모터 정지) / **1 = 해제 (default)** — 문서 §3.2 프로토콜과 일치
  - 상수: `SOFT_ESTOP_ACTIVE(0)` / `SOFT_ESTOP_RELEASE(1)` (STM `rd_register_ecu.h` ↔ Orin `rd_register_ecu.hpp` 동일)
- STM 동작 (`rd_system.c ACTION_STATE_AUTO`): soft_estop==ACTIVE 면 **FSM 전이 없이 `CAN_AK_ESTOP(BREAK_CURRENT_SW)` 능동 제동**
(ESTOP_SW 와 동일한 3A 제동). 제동 중 reg 의 cmd_lin/ang_vel 을 0 으로 클리어해 해제 직후 잔여 명령으로 튀는 것 방지.
해제(1) 시 정상 경로의 ESTOP_override=0 으로 자동 복귀
- `rd_control.c RD_CONTROL_UPDATE`: soft_estop 작동 중 AUTO 분기의 CONSUME/kinematics/LPF 가
cmd_mtr 를 덮어써 해제 순간 잔여 명령이 TX 되는 문제 차단 — motor_on=0 과 동일하게 LPF·명령 0 리셋 후 return

## 2026-06-12 — 실기 통신 테스트 (데스크톱 ↔ ECU, /dev/ttyUSB0)

STM 빌드 플래시 완료 상태에서 bridge 실기 테스트 수행. **전 항목 통과.**


| 테스트               | 결과                                                                                                 |
| ----------------- | -------------------------------------------------------------------------------------------------- |
| 통신 안정성            | ECU[ON], 손실 2(기동 직후뿐), exec ~2.3ms < 5ms 주기. Tx 220회/2s = 100Hz READ + 10Hz sys (이론값 일치)           |
| cmd_vel 안전장치      | 토픽 없음 → 50Hz WRITE skip (Tx 수로 확인). 토픽 발행(20Hz) → WRITE 재개. 정지 후 재차단                               |
| mtr_lock (기존 보호)  | FAULT 상태에서 cmd_vel WRITE 는 STM 이 ACCESS 거부 — AUTO 모드에서만 수락 (설계 의도)                                 |
| 커맨드 READ (once)   | slot auto 배정 → 37ms 내 발사 → RET_OK → hex 덤프 출력 ✓                                                    |
| DEFINE 잠금         | lock 상태 addr1 WRITE → Access Error 재시도 → 2s TIMEOUT 포기 ✓                                           |
| unlock 체인         | addr0=1 → addr1 write 성공 → read 검증 → addr0=0 ✓                                                     |
| CMD 영역 잠금 해제      | lock 상태에서 addr189 쓰기 성공 (DEFINE 만 잠금 확인) ✓                                                         |
| soft_estop        | 부팅 기본값 1(해제) 확인, 0/1 토글 왕복 ✓ (AUTO 제동 동작은 모터 연결 후 확인 필요)                                           |
| reg54 hw_reset 감지 | CAN 모터 미연결 → reg54=0x08 → RCLCPP_ERROR 1초 간격 출력 ✓                                                  |
| hw_reset 매크로      | CLI `macro hw_reset can1` → unlock→reg5=0x08→lock 체인 성공 → STM 이 CAN 복구 + addr54/5 클리어 → ERROR 중단 ✓ |
| REBOOT 커맨드        | alive 476s → REBOOT OK → 3s blackout(connected=false) → 자동 재연결, alive 7.2s (재부팅 확인) ✓              |


미검증(모터/실주행 필요): AUTO 모드 cmd_vel 주행, soft_estop CAN_AK_ESTOP 제동, jeongae 시퀀스 E2E.
참고: 포트 권한은 `sudo usermod -aG dialout $USER` 후 재로그인으로 영구 해결 권장 (현재 chmod 666 임시).

## 2026-06-12 — IMU 토픽 발행 추가 (단위 변환)

- STM `rd_comm_imu.h` raw scale 주석 기준으로 `sensor_msgs/msg/Imu` 발행 (`/carrier/ecu/imu`, 100Hz, PublishMotorFeedback 내)
  - quat z,y,x,w: raw ×0.0001 → orientation (무단위)
  - gyro x,y,z: raw ×0.1 [deg/s] → ×π/180 → angular_velocity [rad/s]
  - acc x,y,z: raw ×0.001 [g] → ×9.81 → linear_acceleration [m/s²] (soa2 = 중력 제거 Local)
  - covariance: 미추정 0 행렬, IMU OFFLINE 시 orientation_covariance[0]=-1 (REP-145)
  - frame_id: `imu_frame_id` 파라미터 (default "imu_link")
- 실기 검증: orientation 정규화 확인(‖q‖≈1), 100Hz 발행 확인 ✓ (IMU 센서 연결 상태)
- STM `RD_MAP_INIT`: 기본값 `SOFT_ESTOP_RELEASE` (해제 상태로 부팅)
- Orin: 섀도 기본값도 RELEASE 로 설정 (main.cpp) — WRITE_REG 로 인한 의도치 않은 ESTOP 방지.
jeongae 시퀀스는 `REG_SOFT_ESTOP_OFFSET`/상수 사용으로 변경
- AUTO 모드는 kinematics 고정 (제어 경로 선택 기능 폐기, `rd_control.c` 주석 정리)
- colcon 재빌드 통과 (경고 0) / STM 은 CubeIDE 빌드 검증 필요

## 2026-07-06 — 견인력 매핑 실험 인프라 (memo_260606.md Step 1~4, Claude Code)

### 결정 사항 (사용자 확인)

- 멀티세그 READ: **방안2 — 기존 READ(0x02) 통합** (`data_len % 4 == 0`, n=1 하위호환)
- 램프: **모터 idx 0 (CAN ID 1)** 단일 / **30A 홀드** / 기울기 **0.5/1.0/2.0 A/s** (selector[0] 1/2/3)
- 발행: **Decode 성공 직후 스케줄 루프에서 직접 publish** (200Hz timer 아님, 1 배치 = 1 msg)
- tare/캘리브레이션: 이번 범위 제외 — raw 로깅 → 오프라인 캘리브레이션

### STM (ECU_V3) 변경 — ⚠ STM32CubeIDE 빌드 검증 필요

- `rd_comm_ecu.c` READ case 멀티세그먼트 확장: Param `[AddrL|H+LenL|H]×n`,
  응답 `Data[0]=err + seg 연접`, 누적 길이 90B 가드. 세그 간 CRIT 원자성 없음(주석 명기)
- `rd_control.c/h` 전류 램프 모드 신규: MANUAL + `selector[1]==2` 에서 thrr1 부호 방향
  선형 램프 (dt=10ms). `RAMP_MOTOR_IDX(0)` 만 인가, 나머지 0A + 전 모터 MODE_CURRENT.
  스틱 중립=즉시 0 / 방향 반전=0 재시작 / 30A 도달=홀드.
  리셋 경로 3중: receive_flag 하강 / 램프 모드 이탈 / `RD_CONTROL_RESET_FILTER`(FSM 전이)

### Orin (orin_firmware_bridge / mgs01_base_msgs) 변경

- `rd_register_ecu.hpp`: **LOADCELL 영역(41~45) 미러 누락 수정** — RSVD0 30→25B,
  `DATA_LOADCELL_t` 추가 (STM rd_register_ecu.h 와 동기화)
- `rd_map.hpp`: `Segment_t{addr,len}` + `TaskConfig_t` 멀티세그 지원
  (`MAX_READ_SEGS=8`, initializer_list 생성자, seg_count==0 = 기존 단일 구간)
- `rd_map.cpp`: Encode READ 4×n 파라미터 / Decode READ 세그별 제 주소 scatter (경로 통일)
- `mgs01_base_msgs/msg/TractionTest.msg` 신규: header + ecu_tick_ms + sys_state +
  cmd_current[4] + fb_current[4] + fb_velocity[4] + fb_position[4] + loadcell_raw[2]
- `rd_bridge`: `traction_test_mode` 파라미터(기동 시 고정) + `PublishTractionTest()`
  (`/carrier/ecu/traction_test`, depth 100)
- `rd_schedule`: traction 모드 분기 — 매 tick 배치 READ `{57:5, 41:5, 96:24, 164:16}`
  (응답 51B ≤ 89B) → Decode 성공 시 즉시 발행. WRITE/서브슬롯/jeongae FSM 정지

### 검증

- colcon 빌드 통과 (mgs01_base_msgs, orin_firmware_bridge — 경고 0)
- 오프라인 왕복 테스트 (rd_map.cpp 단독 컴파일 + STM 핸들러 로직 재현): 4-seg 배치
  scatter 왕복 / n=1 하위호환 / 길이 불일치 거부 / 90B 초과 거부 — 전부 PASS
- 미검증 (실기 필요): STM CubeIDE 빌드, RS485 실통신 멀티세그, 램프 실동작, bag 로깅

### 드라이런 절차 (실기)

1. STM: CubeIDE 빌드 → 플래시
2. Orin: `ros2 run orin_firmware_bridge comm_test_node --ros-args -p traction_test_mode:=true`
3. `ros2 topic hz /carrier/ecu/traction_test` → **200Hz** 확인
4. `ros2 topic echo` 로 sys_state=1(MANUAL), ecu_tick_ms 5ms 등증가, loadcell_raw 정상범위 확인
5. RC: selector[1]=중간(2) → selector[0] 기울기 선택 → thrr1 전/후 → cmd_current[0] 램프 확인
6. `ros2 bag record /carrier/ecu/traction_test` — 시작 후 수 초 무부하 구간(오프라인 영점용) 포함
7. 헤더비트 로그에서 over-period % / loss 확인 (200Hz 예산 내 동작 검증)


## 2026-07-07 — TEST0 샘플 분석 → STM 개선 + 분석 파이프라인 (Claude Code)

### TEST0 샘플 bag 분석 결과 (p0/p1, w20, 정적)

- 통신 품질 양호: 200Hz 발행, 드랍 거의 없음 (148s 중 tick 갭 20ms 1회). fb=cmd 정확 추종
- **중복 샘플 49.9%**: 발행은 200Hz 인데 레지스터 갱신(MARSHAL_PUBLISH)이 systemTask
  100Hz → 실효 데이터 100Hz. 로드셀 ADC 자체는 창 주기 ~5.5ms(~180Hz)라 병목 아님
- **선형성**: ~8A 데드밴드 후 선형. 모델 `F = a·max(I−I0, 0)` 로 피팅해야 함
  (전 구간 단순 선형은 R² 0.85~0.97 로 왜곡)
- **반복성**: p0 slope CV 1.7~3.1% (양호) / p1 CV 11%+, 데드밴드 3~7A 산포,
  3차 램프 후 베이스라인 미복귀 ~300cnt → p1 접촉점은 픽스처/트랙 밀림 의심
- **lc[0] 무부하 baseline ≈ 1.6cnt (ADC 바닥)**: 음방향 클리핑 + `ADC_RAIL_LOW(8)`
  단선 진단 오검출 상태. 영점조절 시 무부하점을 수백 cnt 이상으로 올릴 것 (하드웨어)

### 결정 사항 (사용자 확인)

- 메타데이터: **index CSV 1장** (`analysis/traction/test_index.csv`)
- 피팅: **데드밴드+선형** (slope a, deadband I0 둘 다 조건별 결과로 추출)
- 샘플링: **STM 200Hz 수정 후 재테스트**
- 램프 조작: **thrr1>0 상승 / 중립 홀드 / thrr1<0 하강** (하강 곡선 = 히스테리시스 측정).
  후진(음전류) 램프 폐기, 비상정지는 ESTOP/receive_flag 로 충분

### STM (ECU_V3) 변경 — ⚠ STM32CubeIDE 빌드 검증 필요

- `rd_control.c` 램프 모드: 중립=즉시0 → **중립=홀드**, thrr1<0 = 하강(하한 0 클램프).
  방향반전 리셋 로직 제거. receive_flag/모드이탈/FSM 전이 리셋 경로는 기존 유지
- `rd_map_ecu.c/h` `RD_MAP_MARSHAL_PUBLISH_FAST()` 신규: motor_data(32B) + loadcell.avg +
  realtime_tick 만 발행하는 고속 경로. 100Hz 본 발행과 항목 겹침 무해 (동일 소스)
- `rd_system.c` RD_TASK_CONTROL(200Hz): PERIPHERAL_WRITE 후 PUBLISH_FAST 호출 추가.
  cmd_current 램프 적분은 systemTask 100Hz 유지 (x축은 fb_current 라 무영향)

### 분석 파이프라인 신규 (`analysis/traction/`)

- `traction_analysis.py` — numpy+matplotlib 만 사용 (ROS 불필요, 수동 CDR 디코딩)
  - index CSV 일괄 처리, tick 중복 자동 제거 (STM 수정 전/후 bag 모두 처리 가능)
  - 비교 대상 고정: `MOTOR_IDX=1` × `LC_CH=0` (스크립트 상단 상수 — 셋업 변경 시 수정)
  - 자동: tare(첫 램프 전 무부하 창) / 램프 구간 분리 /
    rise·hold·fall 위상 라벨 (하강 램프 대응 준비됨)
  - 피팅: I0 그리드 + a 폐형해. 노이즈 3층 분리 — 센서(무부하 std) /
    역학(이동평균 잔차 = stick-slip) / 반복성(램프 간 slope CV%, 빈 mean±σ 리본)
  - 베이스라인 미복귀 자동 플래그 (>50cnt) — 픽스처 밀림 감지
  - 산출: bag별 timeseries/scatter_fit/residuals.png + summary.csv + compare.png
- `test_index.csv` 스키마: bag,payload_kg,velocity_mps,body_angle_deg,contact_point,ramp_slope_aps,note
- 실행: `python3 analysis/traction/traction_analysis.py` → `analysis/traction/out/`

### 검증

- 파이프라인: TEST0 bag 2개 엔드투엔드 통과 (램프 3회×2bag 검출, p1 미복귀 4건 플래그)
- 미검증 (실기 필요): STM CubeIDE 빌드, 200Hz 실효 데이터 확인(중복 0% 기대),
  홀드/하강 램프 실동작

## 2026-07-19 — 시간 동기·지연 계측 (testbed_spec.md §2.5 / §6 #0, Claude Code)

### 목적

ECU tick(TIM5 10kHz) ↔ Orin(ROS time) 시계 매핑(offset+drift) 상시 추정 +
명령/피드백 경로 지연 분해 계측. 이후 MPC 지연 보상과 분석 시간축(ECU tick 마스터)의 기반.

### STM (ECU_V3) 변경 — ⚠ STM32CubeIDE 빌드 검증 필요

- `rd_register_ecu.h` DIAG_t: addr 228 `rs485_proc_delta`(uint8, ×0.1ms) 신설, reserved 28→27
- `rd_system.c` RD_TASK_RS485: HANDLE 후 응답 TX 직전 `rd_delta_tick(rd_now_tick(),
  reg.sys.realtime_tick)` latch → diag 기록. **직전 트랜잭션 값 보고 방식** (스냅샷이 처리
  전에 찍히므로 다음 응답에 실림 — Orin 소급 매칭). 프로토콜 변경 없음

### Orin (orin_firmware_bridge / mgs01_base_msgs) 변경

- `mgs01_base_msgs/msg/CommLatency.msg` 신설: t_req/t_resp/ecu_tick/rtt/wire_up·down/
  proc_delta_prev/quality/clock_offset/drift_ppm/offset_valid/cmd_delta_tick[4]
- `rd_clock_sync.{hpp,cpp}` 신설 (ROS 비의존): NTP 유사 offset 구간 추정
  — lb=t_req+wire_up, ub=t_resp−wire_down−proc (wire 는 921600bps 결정적 계산, 대칭 가정
  불필요) → quality(ub−lb) 선별 + 20s 창 선형 피팅 → offset/drift(ppm). tick 32bit unwrap,
  불연속 가드(ECU 리부트/NTP 스텝 시 창 리셋), 폐기 tick 캐시 보고
- `rd_schedule`: ExecuteTask 에 t_req(write 직전)/t_resp(수신 완료) epoch 계측 +
  wire 바이트 수 기록 (`TxnTiming_t last_txn_`) / traction·control 배치 READ 세그 확장
  {88:24}→{88:36} (delta_tick×4 + cmd_delta_tick×4 포함) + {228:1} 추가 (5세그) —
  RW 요청 75B/응답 65B ≤ 90B 확인
- `rd_bridge`: `PublishCommLatency()` — 추정기 갱신 + `/carrier/testbed/comm_latency`
  발행 (traction/control 모드 한정, 200Hz). 최초 수렴 시 INFO 1회
- `rd_register_ecu.hpp` 미러 동기화 (DIAG rs485_proc_delta) + `REG_PROC_DELTA_OFFSET`

### 검증

- STM: DIAG region R/O LUT 기존 커버 확인, rs485Task 단일 소유(락 불필요) 확인
- Orin: 세그 예산(요청 75B/응답 65B ≤ 90B), scatter 경로(flat 256B) addr228 커버 확인
- **미검증**: colcon 빌드 (이 머신 ROS 없음 — 개발머신에서 빌드 필요), STM CubeIDE 빌드,
  실기 특성화 실험 (spec §2.5: RTT 분포 / offset 수렴·drift / cmd 경로 E2E)
- 특성화 bag 분석 스크립트(`analysis/latency/`)는 실기 bag 확보 후 작성 예정

## 2026-07-13 — P2: RW Inst + MPC 제어 프레임 (memo_260606.md §8, Claude Code)

### 결정 사항 (사용자 확인)

- Inst 이름/코드: **PACKET_INST_RW = 0x04**
- 응답 에러: **니블 분리** `Data[0] = read_err | (write_err<<4)` — write 거부에도 read 스트림 유지
- MPC-ready 제어 프레임 스켈레톤 지금 구현
- 센서별 us 타임스탬프(§9)는 **P3 이월** (절대 타이머 uint32 ms + uint16 10us, 센서별 uint16 델타 — 레지스터 개편 필요)

### STM (ECU_V3) 변경 — ⚠ STM32CubeIDE 빌드 검증 필요

- `rd_comm_ecu.c/h` RW(0x04) 핸들러 신규: `[R]+read세그×R+[waddr]+데이터`,
  WRITE 적용 → READ 스냅샷(read-back 겸용), read 는 write 결과 무관 항상 수행
- `rd_control.c/h` AUTO 직접 전류 경로: `ctr_mode[0]==MODE_CURRENT` 면 kinematics 덮어쓰기 skip.
  `RD_CTRL_CMD_TIMEOUT_MS(20ms)` 스테일 워치독 — cmd_write_tick 갱신 끊기면 전류 강제 0
  (rd_map_ecu.c 의 기존 TODO 워치독을 direct-current 경로에 구현)

### Orin (orin_firmware_bridge) 변경

- `PacketInst::RW` + `TaskConfig_t` RW 생성자 (start_addr/data_len=write 구간, segs=read)
- `rd_map.cpp`: Encode RW / Decode RW(니블 파싱, write 거부 간헐 로그 + 복구 알림),
  read scatter 를 `ScatterReadSegs()` 로 분리 (READ/RW 공용)
- `rd_bridge`: `control_mode`·`cmd_current_max`(30A)·`cmd_current_timeout`(50ms) 파라미터,
  `/carrier/cmd_current`(Float32MultiArray[4]) 구독 + 클램프, `PrepareControlCommand()` —
  스테일 시 0A. traction 과 동시 지정 시 traction 우선(ERROR 로그)
- `rd_schedule`: `task_control_` = RW{write 128:52 + read 4세그(견인과 동일)},
  control_mode 분기 — 매 tick Prepare → RW → TractionTest 발행

### 검증

- 오프라인 RW 왕복 테스트 (rd_map.cpp 단독 + STM RW 핸들러 재현): ① write 수락 왕복
  + read-back ② lock 거부 시 read 지속(니블) ③ 포맷 오류 양니블 DATA_LEN ④ read 에러 RD_ERROR — 전부 PASS
- colcon 빌드 통과 (경고 0)
- 미검증 (실기): STM CubeIDE 빌드, RW 실통신, AUTO 직접 전류 실동작, 20ms 워치독 실측

### 실기 검증 절차 (제어 경로)

1. STM: CubeIDE 빌드 → 플래시
2. `ros2 run orin_firmware_bridge comm_test_node --ros-args -p control_mode:=true`
3. 로봇 AUTO 모드 + RC on (motor_on 게이트) — cmd_current 토픽 없으면 0A 유지 확인
4. `ros2 topic pub -r 50 /carrier/cmd_current std_msgs/msg/Float32MultiArray "{data:[1.0,0,0,0]}"` → M0 전류 인가 확인 (TractionTest read-back 으로 검증)
5. 토픽 중단 → 50ms 내 0A 복귀(ORIN 가드) / bridge kill → STM 20ms 워치독 동작 확인
6. MANUAL 전환 → RW write 거부(ACCESS) 로그 + read 스트림 지속 확인

## 2026-07-14 — MARSHAL_PUBLISH 를 rs485Task 로 이관 (요청 직전 발행, Claude Code)

### 배경/결정 (memo_260608.md 토론)

- reg 데이터 영역의 목적 = Orin 전달 → 발행 지점을 전달 담당 태스크(rs485Task)로 이동,
  레이어를 해치던 `PUBLISH_FAST`(controlTask 200Hz) 폐기
- 우려 1 (송신 지연): PUBLISH 본문 = memcpy ~75B + 스칼라 → 예측 ~10us ≪ 200us 예산.
  CRIT 길이는 기존과 동일(호출 위치만 이동) → ISR 블로킹 프로파일 불변.
  `diag_publish_us_last/max` (rd_system.c, 임시) 로 실측 후 제거 예정
- 우려 2 (수동 모드 미기상): grep 검증 — reg 데이터 영역의 펌웨어 내부 소비자 없음(Orin 전용).
  그래도 `osThreadFlagsWait(0x0001, osFlagsWaitAny, 10)` timeout 기상으로 발행 유지
  (Live watch 동결 방지 + 미래 내부 소비자 대비)

### STM (ECU_V3) 변경 — ⚠ STM32CubeIDE 빌드 검증 필요

- `rd_system.c RD_TASK_RS485`: wait timeout ∞→10ms, 루프 최상단에서 기상 사유 무관
  무조건 `MARSHAL_PUBLISH` (RET_OK 분기 안이 아님 — timeout 경로도 커버).
  유효 요청 시 발행→HANDLE 간격 us 수준 = 요청 시점 스냅샷
- `rd_system.c RD_TASK_SYSTEM`: PUBLISH 호출 제거 (Checker 등 '생산'은 그대로 소유)
- `rd_system.c RD_TASK_CONTROL`: PUBLISH_FAST 호출 제거
- `rd_map_ecu.c/h`: `RD_MAP_MARSHAL_PUBLISH_FAST` 함수/선언 삭제, 주석 갱신
- `CLAUDE.md`: 태스크 표(rs485Task 이벤트+10ms) + 데이터 흐름 갱신

### 효과

- 응답 데이터 신선도 = 센서 갱신 나이로 수렴 (발행 200Hz/100Hz 주기와 READ 주기의
  beat 로 인한 중복 샘플 문제의 일반해 — 2026-07-07 bag 중복 이슈의 근본 해결)
- P2 RW(MPC) 응답의 read 세그도 자동으로 요청 시점 스냅샷

### 실기 확인 항목

1. Live Expressions: `diag_publish_us_max` ≤ 200us 확인 (예측 ~10us) → 이후 diag 제거
2. traction_test_mode bag 재검증: 로드셀/모터fb 중복 샘플률 0% 근접 확인
3. 수동 모드(Orin 미접속)에서 reg live watch 갱신되는지 확인 (10ms timeout 동작)

## 2026-07-14 — 램프 모드 삭제 + 직접 전류 2단 워치독 정리 (STM, Claude Code)

### 배경 (memo_260608.md)

- 견인 실험용 RC 전류 램프 모드(selector[1]==2) 폐기 — 이후 전류 명령은 Orin 200Hz RW 로 일원화
- STM 범위만 우선 진행 (ORIN 측 traction 인프라는 유지)

### STM (ECU_V3) 변경 — ⚠ STM32CubeIDE 빌드 검증 필요

- `rd_control.c/h` 램프 전체 삭제: selector[1]==2 분기, s_ramp_current 적분기,
  RAMP_SLOPE_APS 테이블, RAMP_* 매크로 4종, RESET_FILTER/receive_flag 리셋 연동.
  (selector[1]==3 RC 직접비례 CURRENT 모드는 기존 기능이라 유지)
- 직접 전류 **2단 스테일 워치독** 확립 (rd_control.h 주석에 설계 명문화):
  - **1단** (controlTask 200Hz, `RD_CTRL_CMD_TIMEOUT_MS=20ms`): cmd_write_tick 스테일 →
    전류 강제 0, **TX 유지** (0A 능동 전송으로 즉시 토크 제거).
    개선: reg.cmd_motor 도 함께 0 — Orin read-back/로그가 실제 인가 전류와 일치
  - **2단** (systemTask 100Hz, `AUTO_TIMEOUT=100ms`, 사용자 구현): motor_on=0 →
    TX 중단 최종 게이트. kinematics 경로 포함 AUTO 전체 커버
    (이전 세션에서 지적한 "kinematics 경로 워치독 부재" 구멍이 이것으로 해소됨)

### 실기 확인 항목

1. AUTO + control_mode 에서 cmd_current 토픽 중단 → 20~25ms 내 fb 전류 0 수렴 (1단)
2. bridge kill → 100ms 후 motor_on=0 (2단, CAN TX 정지 확인)
3. MANUAL RC 주행 회귀 확인 (selector[1]==3 전류 / 기본 속도 모드)

## 2026-07-14 — CAN TX 잔류 명령(급발진) 차단 (STM, Claude Code)

### 배경 (사용자 우려)

- 버스 일시 정지(error passive/메일박스 full 등, RECOVERY 문턱 밑) 중 쌓인 명령이
  갑작스런 회복 순간 한꺼번에 발사 → 급발진 우려
- 분석 결과: RTOS 큐(depth16)는 drop-oldest 로 항상 최근 ~20ms 치만 유지되어 위험 낮음.
  **실제 벡터는 HW TX 메일박스 3개** — AddTxMessage 된 프레임은 자동 만료가 없어
  수 초 묵은 명령(예: +20A)이 회복 순간 최우선 송신됨. 완전 bus-off 는
  CAN_RECOVERY(DeInit+큐 reset)가 이미 커버, 그 문턱 밑의 일시 정지가 구멍.

### STM (ECU_V3) 변경 — ⚠ STM32CubeIDE 빌드 검증 필요

- `can_ak.h`: `CAN_TX_STALE_MS(20ms)` 신설, `CAN_Tx_Packet_t.enq_tick` 필드 추가
- `can_ak.c CAN_AK_Transmit`: enqueue 시 `enq_tick = osKernelGetTickCount()` 스탬프
- `can_ak.c CAN_AK_TX_TASK_HANDLER`:
  - **신선도 검사**: dequeue 시 20ms 초과 프레임은 송신하지 않고 폐기 (tx_err_cnt++)
    — motor_on=0 등 유입 중단 후 큐 잔류(기존 최대 ~50ms 창)도 20ms 로 조임
  - **메일박스 Abort**: 3 tick 대기에도 안 비면 `HAL_CAN_AbortTxRequest`(전 메일박스)
    — 이 버스(ECU+모터4, 부하 ~15%)에서 3ms full = 비정상이므로 유효 트래픽 손실 없음

→ 큐(20ms)·HW 메일박스(3ms+abort) 양쪽에 시간 상한 — "회복 순간 묵은 명령 발사" 클래스 차단

### 실기 확인 항목

1. 정상 주행 중 tx_err_cnt 가 평시 0 유지 (신선도/abort 오탐 없음)
2. CAN 커넥터 순간 분리→재연결: 재연결 순간 모터 거동 관찰 (묵은 토크 펄스 없어야 함)
3. AK_TX_TIMEOUT_ERR(5) 연동 — abort 경로의 tx_err_cnt 증가가 기존 에러 승격과 정합

## 2026-07-14 — 잔류 명령 청소 로직 일원화: RD_CONTROL_CMD_CLEAR (STM, Claude Code)

### 배경 (사용자 제안)

- RD_CONTROL_UPDATE 안에 "잔류 명령 0 초기화" 블록이 3곳(모드 이탈/motor_on=0/soft_estop)
  인라인 반복돼 불필요하게 복잡 → motor_on 의 결정권자인 systemTask 가 청소까지 소유하도록 분리

### STM (ECU_V3) 변경 — ⚠ STM32CubeIDE 빌드 검증 필요

- `rd_control.c/h` `RD_CONTROL_CMD_CLEAR()` 신설: reg.cmd_motor(cur/vel) +
  reg.cmd_system(lin/ang) + cmd_mtr(cur/vel) 일괄 0 (CRIT). **ctr_mode 는 유지** (기존 원칙).
  **LPF 는 제외** — s_filtered_vel 은 controlTask 소유 static 이라 systemTask 에서 건드리면
  크로스태스크 레이스 → 필터 리셋은 RD_CONTROL_UPDATE 분기(controlTask 문맥)에 존치
- `rd_system.c RD_TASK_SYSTEM`: ACTION switch 직후 `if (!motor_on) RD_CONTROL_CMD_CLEAR()`
  — 레벨 트리거(매 tick), 청소 후 유입된 스테일 명령도 다음 tick 재청소
- `rd_system.c ACTION_STATE_AUTO` soft_estop: 인라인 lin/ang 클리어 → CMD_CLEAR 로 대체
  (reg.cmd_motor + cmd_mtr 까지 청소 범위 확대)
- `rd_control.c RD_CONTROL_UPDATE` 단순화:
  - motor_on==0 / soft_estop 분기: 인라인 0 루프 제거 → RESET_FILTER + return 만
  - 1단 워치독(20ms): 인라인 0 루프 → CMD_CLEAR 재사용
- 동작 등가성: motor_on==0 시 TX 는 PERIPHERAL_WRITE 가 skip 하므로 청소가 100Hz(system)로
  옮겨져도 잔류 명령이 TX 될 창 없음 / soft_estop 중 TX 는 ESTOP_override(제동)가 담당

### 실기 확인 항목

1. RC receive_flag 하강 → 재상승 시 0 에서 재기동 (잔여 명령 튐 없음, 기존 동작 유지)
2. soft_estop 토글 왕복 시 해제 직후 잔여 명령 TX 없음
3. control_mode 스트림 중단 → 20ms 1단(0A 전송) → 100ms 2단(motor_on=0) 순차 동작

### (후속 수정, 같은 날) CMD_CLEAR 를 controlTask 로 재배치 — 사용자 피드백

- systemTask 훅 방식 대신 **RD_CONTROL_UPDATE 비구동 분기 안에서 직접 호출**로 변경:
  - 호출자가 controlTask 단일이 되면서 크로스태스크 문제 소멸 → **LPF 리셋까지 CMD_CLEAR 에 포함**
    (한 함수가 잔류 상태 전체 청소를 완결), 함수는 rd_control.c **static** 으로 강등 (공개 API 아님)
  - 200Hz 레벨 트리거 → 청소 지연 10ms→5ms, "미청소 창" 문제도 해소
  - 소유권 재정리: systemTask = motor_on **결정**만 / controlTask = **집행**(청소+필터)
- RD_CONTROL_UPDATE 비구동 3분기(비 MANUAL·AUTO 상태 / motor_on==0 / soft_estop) 전부
  `CMD_CLEAR + return` 으로 통일 — INIT/ESTOP/FAULT 상태에서도 잔류 명령 청소 (기존보다 강화)
- rd_system.c: motor_on 훅 제거, ACTION_STATE_AUTO soft_estop 의 CMD_CLEAR 호출 제거 (주석으로 위임 명시)

## 2026-07-14 — 레이어 리팩토링: 정책(SYSTEM) / 연산(CONTROL) 완전 분리 (STM, Claude Code)

### 설계 원칙 (사용자 방향 + 토론 확정)

- **systemTask = 정책/안전**: 모든 에러·스테일·estop 을 motor_on/ESTOP_override 로 일원화,
  비구동이면 reg 청소. **controlTask = 순수 연산**: CONSUME → 경로 변환 → LPF, 상황 판단 분기 0.
- "정지" 가 코드 분기가 아니라 **데이터(reg=0) + 게이트(motor_on/override)** 로 표현됨 —
  control 은 sys_state 가 무엇이든 같은 파이프라인을 돈다.

### 결정 사항 (사용자 확인)

- 스테일 워치독: **AUTO_TIMEOUT(100ms) 단일** — 1단(20ms, control 내) 삭제.
  발동 시 motor_on=0 + (훅 경유) CMD_CLEAR 로 마지막 전류 잔류 차단
- AUTO 경로 선택(direct 전류/kinematics): control 이 ctr_mode 직접 읽기 유지 (수식 선택으로 간주)
- LPF: 자연 감쇠 + **전이 감지 리셋** (robot_state / soft_estop / motor_on 0→1)
- RC receive_flag==0 청소: RC_TO_REGISTER 자체 청소 삭제 → 비구동 훅으로 일원화

### STM (ECU_V3) 변경 — ⚠ STM32CubeIDE 빌드 검증 필요

- `rd_system.c RD_TASK_SYSTEM`: **통합 비구동 훅** 신설 —
  `if (!motor_on || ESTOP_override) RD_CONTROL_CMD_CLEAR();`
  override 는 ESTOP_HW/SW·soft_estop 에서 1 이므로 모든 비구동 상황이 한 줄로 커버
- `rd_system.c RD_TASK_CONTROL`: 전이 감지 LPF 리셋 3신호(robot_state/soft_estop/motor_on 상승)
- `rd_control.c RD_CONTROL_UPDATE`: **무분기화** — 비구동 3분기 + 1단 워치독 삭제,
  CONSUME → AUTO 경로 변환 → LPF 만. 시그니처 `(cmd, s)` → `(cmd)` (상태 파라미터 제거)
- `rd_control.c RD_CONTROL_RC_TO_REGISTER`: receive_flag==0 → 매핑 skip 만 (청소 루프 삭제)
- `rd_control.c/h`: CMD_CLEAR public 복귀 (systemTask 호출, LPF 제외),
  RD_CTRL_CMD_TIMEOUT_MS 삭제, 레이어 원칙 헤더 주석 신설
- `rd_system.h`: AUTO_TIMEOUT 주석 — 단일 워치독 + Orin 측 가드(50ms)가 1차임을 명시

### 안전 등가성 검증 (설계 시점)

- soft_estop 중 Orin 이 계속 write 해서 reg 에 최대 10ms 비영값 존재 가능 →
  TX 는 override 제동이 대체하므로 버스로 나가는 건 항상 brake ✓
- receive_flag 하강: 같은 systemTask tick 안에서 motor_on=0 계산 → 훅 청소 (지연 0) ✓
- MPC 스트림 사망: Orin 가드(50ms, 0A) 1차 → STM 100ms motor_on=0+청소 2차.
  bridge 자체 사망 시 0~100ms 마지막 전류 TX 지속은 수용(사용자 결정) ✓

### 실기 확인 항목

1. MANUAL RC 주행 / 플래그 하강→재상승 0 재기동
2. AUTO MPC: 스트림 중단 → 100ms 내 motor_on=0 + reg 0 (TractionTest read-back 으로 확인)
3. soft_estop 토글: 제동 → 해제 시 LPF 0 재시작 (전이 리셋)
4. ESTOP_SW/HW 진입·해제 왕복

## 2026-07-16 — Timestamp/delta_tick 통합 (memo_260716.md 전체 구현, Claude Code)

### 사전 발견 (중요)

- **TIM5 는 이미 10kHz free-run(PSC=8399, ARR=max)으로 MX 설정돼 있었으나 코드가 구 1kHz
  가정 그대로** — tim_cnt 0 고정(update IRQ 119h/회) → realtime_tick 0 발행, Get_Time_us ×100 오염
  상태였음. 이번 작업으로 정합화.

### 확정 사항 (memo §A~G)

- [TODO-A] ts 저장 = **드라이버 핸들 분산** (단일 소유자 선례 준수, 중앙 구조체 없음)
- [TODO-B] latch: ADC half/full 콜백 / CAN_AK_RX_APPLY / AddTxMessage 성공 직후(pTsCmd 포인터)
  / UART IDLE 핸들러(rx_stamp, uart 공통) / 인코더 폴링 성공 분기
- 레이아웃 = 시트 역산안 확정 (SYS 16~31, LOADCELL 42~47, IMU 48~69, ENC 70~85, MOTOR 88~127,
  **CMD/RSVD1/DIAG 불변** → lock 범위·워치독·Orin 쓰기 경로 무변경)
- publish 실측 diag 제거 / Orin 동기화 동일 세션 진행

### STM (ECU_V3) 변경 — ⚠ STM32CubeIDE 빌드 검증 필요

- `rd_common.h`: `rd_now_tick()` 선언 + `DELTA_STALE(0xFF)` + `TS_INVALID(-256)` + `rd_delta_tick()`
  (TS_INVALID 로 부팅 직후 stale 오탐을 산술적으로 해소 — valid 플래그 불필요)
- `rd_system.c/h`: tim_cnt·RD_TIM_CALLBACK 폐기, `rd_now_tick()` 정의(CNT 단일 read, ISR-safe),
  `Get_Time_us()`=tick×100 재작성, TIM5 Start_IT→Start, publish diag 제거
- `main.c`: PeriodElapsedCallback 의 TIM5 분기 제거
- `rd_register_ecu.h`: delta_tick 포함 전면 재배치 (합 256B)
- latch 5종: `rd_adc`(ts_stamp+콜백), `can_ak`(ts_fb/ts_cmd + CAN_Tx_Packet_t.pTsCmd),
  `rd_uart`(rx_stamp), `rd_comm_imu`(ts_stamp=IDLE 시각 채택), `i2c_as5600`+`rd_i2c_encoder`(채널별)
- `rd_map_ecu.c`: LUT 재배치 순서 갱신, MARSHAL_PUBLISH 에 delta 일괄 계산 통합
  (발행 시점 now 1개 → realtime_tick + 전 delta 정합 — Orin 이 now−delta 로 취득시각 복원)

### Orin (orin_firmware_bridge / mgs01_base_msgs) 변경

- `rd_register_ecu.hpp` 전면 재작성 (미러 + static_assert 256B 컴파일 검증 통과)
- `rd_schedule`: 배치 세그 {57:5}→{27:5}, {41:5}→{42:6}, {96:24}→{88:24} (traction/control 공통),
  100Hz READ 는 매크로 기반이라 자동 (48~127, 80B)
- `rd_bridge`: alive_time /1000→/10000
- `TractionTest.msg`: `ecu_tick_ms`→`ecu_tick` [×0.1ms] 개명 (analysis 스크립트 미사용 확인)

### 검증

- colcon 빌드 통과 (msgs+bridge, 경고 0) — hpp static_assert 로 256B 레이아웃 컴파일 검증
- 오프라인 테스트 (test_layout): ① offsetof 전수 = 시트 주소 일치 ② 새 4세그 배치(52B) 왕복
  ③ delta 산술 — wrap(memo §9 단위테스트 포함)·미갱신 stale·부팅 직후 TS_INVALID — 전부 PASS
- 미검증 (실기): CubeIDE 빌드, delta_tick 분포 (memo §9: 모터 0~100, IMU 0~40, 0xFF 빈도)

### 남은 항목

- 시트(Google Sheet) 표기 정정: realtime_tick "1kHz"→"10kHz" (사용자 수동)
- TractionTest.msg 에 delta_tick 필드 추가 (후속 결정)
- DPCB 맵 동일 패턴 적용 (별도 계획)


## 2026-07-17 — auto_mode 4-경로 분리 + SYSTEM/CONTROL 재분리 (STM+Orin, Claude Code)

### 설계 (사용자 방향 + 확정)

- **CMD_SYSTEM_t 개편**: addr 188 `weight` → `auto_mode` (0=KINEMATIC/1=CURRENT/2=DIRECT/3=CONTROL(TODO)),
  addr 191 reserved → `use_lpf` (default 1). weight 는 RC scale 계산용 로컬로 강등 (reg 발행 폐기)
- **AUTO 분기 전부 SYSTEM(100Hz) 소유**: ACTION_STATE_AUTO 가 auto_mode 별로 reg.cmd_motor 에
  스테이징 (KINEMATIC=kinematics→cmd_velocity+MODE_VELOCITY / CURRENT=ctr_mode 강제 /
  DIRECT=무가공 통과 / CONTROL·default=motor_on 차단). MANUAL 의 RC_TO_REGISTER 와 대칭 구조 완성
- **CONTROL(200Hz) = CONSUME → LPF 만**: RD_CONTROL_UPDATE 의 AUTO 분기(robot_state/ctr_mode 판단) 삭제
- **use_lpf 소유권**: MANUAL 은 ECU (속도모드=1/전류모드=0, RC_TO_REGISTER) / AUTO 는 Orin write

### 확정 사항 (사용자 선택)

- LPF 재개 글리치: **use_lpf 0→1 상승엣지를 전이 리셋 신호에 추가** (4번째 신호) —
  OFF 동안 동결된 옛 필터값에서 재개하는 점프 방지 (MANUAL 내 전류↔속도 selector 토글은
  robot_state/motor_on 불변이라 기존 3신호로 안 잡히던 구멍)
- DIRECT 가드: **워치독만** (AUTO_TIMEOUT + soft_estop + motor_on 게이트로 충분, 추가 제한 없음)
- 함수 인자: **스냅샷 인자 방식** — reg/ECU_PERIPHERAL 싱글톤은 전역 유지, 함수는 조작 대상
  포인터 + CRIT 스냅샷 값을 인자로. CONSUME 호출을 RD_TASK_CONTROL 루프로 이동 (데이터플로우 가시화)

### STM (ECU_V3) 변경 — ⚠ STM32CubeIDE 빌드 검증 필요

- `rd_register_ecu.h`: 구 CMD_SYSTEM_t(weight 판) 중복 typedef 삭제 (컴파일 에러였음),
  auto_mode 주석 정정 (default 0=KINEMATIC — memset 0 자연 default, "COTROL" 오타),
  use_lpf 주석을 실제 정책(MANUAL=ECU 소유 속도1/전류0, AUTO=Orin 소유)으로
- `rd_map_ecu.c RD_MAP_INIT`: `use_lpf = 1` 명시 세팅 추가 (memset 0 → 부팅 직후 LPF OFF 버그)
- `rd_system.c ACTION_STATE_AUTO`: switch 문법 수정 (case 브레이스, default 콜론 누락)
- `rd_system.c RD_TASK_CONTROL`: 전이 리셋 4신호화 (use_lpf 상승엣지), CONSUME 호출 이동,
  use_lpf CRIT 스냅샷 → RD_CONTROL_UPDATE 인자 전달
- `rd_control.c/h`: RD_CONTROL_UPDATE(cmd) → (cmd, use_lpf) — AUTO 분기·CONSUME·전역 reg/robot_state
  참조 전부 제거 (순수 LPF), RC_TO_REGISTER 의 `cs->weight` 쓰기 삭제, 레이어 원칙 주석 갱신

### Orin 변경

- `rd_register_ecu.hpp`: CMD_SYSTEM_t 미러 동기화 (weight→auto_mode, reserved→use_lpf) —
  bridge 코드에서 weight 참조 없음 확인

### 실기 확인 항목

1. 부팅 직후 use_lpf==1 확인 (addr 191 read-back)
2. MANUAL selector 전류→속도 토글: LPF 0 재시작 (점프 없음)
3. AUTO auto_mode=0: Orin lin/ang write → 속도 주행 / =1: 직접 전류 / =3: motor_on=0
4. AUTO 중 cmd write 중단 → 100ms 워치독 motor_on=0 (기존 경로 회귀 확인)

## 2026-07-17 — USER 점검 요청 처리: ADC 창 재계산 + volatile 정리 (STM, Claude Code)

### 점검 결과 (아래 "USER 수정 사항 정리" 섹션 대응)

- **can_ak ts_fb (POP 시점 캡처)**: 이상 없음 ✓ — RX_POP(FIFO pending ISR 컨텍스트)에서
  GetRxMessage 직후 latch, APPLY 가 복사. 초기값 AK_TS_INVALID 유지
- **i2c_as5600 ts_stamp**: 이상 없음 ✓ — UPDATE 성공 분기에서 즉시 latch, 실패 시 latch 없음
- **ADC sample 106**: 판단 정확 → 수정 (아래)
- **volatile 점검**: 남발은 아니나 task↔task 4곳 불필요 → 정리 (아래)
- **AUTO 중 LS_DEGRADED**: 현행 유지 권장(수정 안 함) — 손실 위험은 AUTO_TIMEOUT(100ms)·
  CRC·HC_TIMEOUT 이 이미 상한을 걸고, degraded_cnt% 가 reg 로 발행되므로 정지/감속 정책은
  임무 맥락을 아는 Orin 이 soft_estop 으로 결정 (정책은 상위 소유 원칙). 보수적으로 가려면
  rc_ok=0 대신 DEGRADED 시 속도 클램프가 중간 선택지

### 수정 1 — ADC 창 재계산 (클럭 168MHz 상향 대응)

- `.ioc` diff 확인: SYSCLK 84→168MHz (PLLN 84→168). APB1 DIV2→DIV4 보상으로 CAN 1Mbps·
  UART2/6 baud·TIM5 10kHz(APB1 Tim 84MHz) 전부 무변경. **APB2 만 42→84MHz** →
  ADC 10.5→21MHz → 1변환 46.9→23.4µs → 구 53 샘플이면 half 창 2.48ms 로 5ms 정렬 붕괴
- `rd_adc.h`: `ADC_WIN_SAMPLES` 53→**106** (2ch×106=212변환 ≈ 4.97ms 정렬 복원) + 주석 재계산.
  DMA 버퍼 424→848B (문제없음), ADC_STALE_TIMEOUT_MS 20 유효
- `CLAUDE.md`: "84 MHz" → 168MHz (APB 구성 부기)

### 수정 2 — volatile 정리 (원칙: ISR 가 직접 쓰는 필드만 volatile)

- 유지(ISR 공유): can_ak state/error/ts_*, rd_uart ring/error, rd_adc *_ready/ts/cb_*,
  hb_control, robot_state, PERIPHERAL_DATA_t/PERIPHERAL_ERROR_t(집계 경로 보수적 유지)
- 제거(task↔task, CRIT/단일코어로 충분):
  `rd_comm_receive.h` thrr/diff/flag/selector (rcTask 쓰기 · systemTask 읽기),
  `rd_peripheral.h` GPIO_t IND/MODE/ESTOP/IND_cnt (EXTI 아님 — 태스크 폴링),
  `can_ak.h` cmd (controlTask 단일 소유, 큐에는 복사본),
  `rd_peripheral_ecu.h` cmd_mtr (전 접근 CRIT 보호)
- 시그니처 전파: RD_CAN_MOTOR_TRANSMIT `(const volatile → const)`,
  RD_CONTROL_UPDATE `(volatile 제거)`, CONSUME/TRANSMIT/INIT 의 volatile 캐스팅 정리
- 근거 메모: RX_APPLY·CONSUME 의 memcpy 캐스팅이 volatile 을 이미 버리고 있었음 —
  실제 보호는 단일 store 패턴 + CRIT 이 담당 (volatile 은 보조)

### 실기 확인 항목

1. CubeIDE 빌드 (클럭 168MHz 재생성 코드 포함)
2. 로드셀: adcTask 콜백 주기 ~5ms 복원 (cb_full_dt_us ≈ 9.9ms, half/full 교대 5ms)
3. delta_tick 분포 재확인 (loadcell delta ≈ 0~50 [×0.1ms])

## 2026-07-17 USER 수정 사항 정리
### 직접 수정 사항 (점검 필요)
- can_ak.c 에서 ts_fb 취득 시간은 POP하는 시점에서 캡처하는 것이 가장 가까운 시점
- i2c_as5600 에서도 같은 맥락으로 update가 되는 시점에 즉시 rd_now_tick update를 하는 방식으로 수정
- ADC에서 sample 수 106으로 수정이 필요할 것 같음.

### 점검 요청 사항
- volatile이 남발 되는지 나중에 전체적으로 점검이 필요할 것 같음. 
- ACTION_STATE_AUTO에서 LS_DEGRADED일 때도 모터를 운전하는게 맞는 선택인가 위험할 거 같아서 이 때에는 rc_ok == 0이지 않을까?



## 2026-07-17 — Fail-safe 개선 P1~P4 구현 (failsafe_analysis_260717.md §8, Claude Code)

### P4 — 문서 정정 (H7)

- `CLAUDE.md`: UART_RX_TIMEOUT 500→100ms, UART_FATAL_CNT_TH 20→10, HAL_FATAL_CNT_TH/AUTO_TIMEOUT
  행 추가, 미사용 AK_TX/RX_TIMEOUT_ERR 각주 처리, **전원 인가 순서 수칙** 신설
  (모터 전원 → ECU, CAN 은 TX 주도라 READY 대기 예외)
- `rd_common.h`: DEGRADED_TICK_DECAY 주석 "20ms −2" → "10ms 틱 −1"

### P3 — I2C 자가 복구 지수 백오프 (H6)

- `rd_system.h`: I2C_RECOVERY_BASE_MS(100)/MAX_MS(1000) 신설
- `rd_system.c RD_TASK_I2C1`: 연속 실패 시 재시도 간격 100→200→400→800→1000ms(cap),
  UPDATE 정상 복귀 시 즉시 리셋 — 라인 사망 시 매 10ms 버스클리어 폭주 제거

### P2 — uart2 fatal 모드별 처리 (H2)

- `rd_system.c RD_SYSTEM_CHECKER`: FATAL_MAX 도달 시 AUTO=FAULT / **MANUAL=RECOVERING 동결
  + addr54 리셋 요청만 (RC 주행 유지)** — RC 채널과 대칭 복원. 동결 중 AUTO 전환 시 FAULT 합류
- `rd_system.c ACTION_STATE_FAULT`: uart2 리붓 즉시 → **UART2_REBOOT_DELAY_MS(3s) 유예 후**
  SystemReset, 유예 중 플래그 해제(Orin addr5) 시 리붓 취소. (현 빌드는 RS485_TEST_ON 이라
  리붓 경로 비활성 — 플래그 해제 시 활성)

### P1 — motor_mask + 활성 모터 무응답 ESTOP_SW + 채널 escalation 분리 (H1)

- `rd_register_ecu.h`: CMD_SYSTEM_t addr 192 `motor_mask` 추가 (bit0~3=M1~4, default 0x0F,
  MOTOR_MASK_ALL) — RSVD1 에서 1B 인출 (CMD_SYSTEM 13B, RSVD1 193~223 31B). LUT 매크로 기반 자동
- `rd_map_ecu.c RD_MAP_INIT`: motor_mask = MOTOR_MASK_ALL
- `rd_can_motor.c/h`:
  - TRANSMIT(cmd, mask) — 마스크 제외 모터 TX skip (단일 트랙 테스트 지원)
  - CHECKER(data, err, mask) — 제외 모터 집계 제외 + **per-motor RX 타임아웃을 채널
    health/degraded 에서 분리** (H1 원인이던 0.5s 주기 DEGRADED→OFFLINE→RECOVERY 순환 소멸,
    채널 escalation 은 HAL/버스 에러 전용)
  - `RD_CAN_MOTOR_COMM_LOST(mask, motor_on)` 신설 — 구동 중 활성 모터 500ms 무응답 판정
    (MOTOR_COMM_FAULT_MS, motor_on 상승 후 500ms 기동 유예로 부팅/전원순서 오탐 방지)
- `rd_system.c RD_SYSTEM_UPDATE_STATE`: motor_fault |= COMM_LOST → **ESTOP_SW 자동복귀형**
  (전체 구성에서 모터 1개 탈락 시 전체 제동, 통신 복구 시 자동 복귀)
- `rd_peripheral_ecu.c/h`: PERIPHERAL_WRITE(obj, mask) — 스냅샷 인자 전파 (정상/ESTOP 경로 공통)
- Orin `rd_register_ecu.hpp`: CMD_SYSTEM 13B/RSVD1 31B 동기화 — g++ static_assert 256B 통과.
  bridge 는 cmd_lin/ang(180~187)만 write 라 무영향 확인

### 검증

- Orin hpp: g++ -fsyntax-only + CMD_SYSTEM_t==13 / REGISTER_t==256 static_assert 통과
- 전 호출부 시그니처 정합 grep 확인. STM CubeIDE 빌드는 실기 검증 필요

### 실기 확인 항목

1. 부팅: addr 192 read-back == 0x0F, 모터 전원 늦어도 READY 대기 (enable off)
2. 단일 트랙: mask=0x01 write → M2~4 TX 없음 + 0.5s RECOVERY 순환 사라짐 (delta_tick 유지 확인)
3. 전체 구성 주행 중 모터 1개 커넥터 분리 → 500ms 내 ESTOP_SW 제동 → 재연결 시 자동 복귀
4. I2C 커넥터 분리 → 복구 재시도 간격 1s 로 수렴, 재연결 시 즉시 복구
5. (RS485_TEST_ON 해제 빌드) uart2 노이즈 주입: MANUAL 주행 유지 + addr54 요청 / AUTO 3s 후 리붓

## 2026-07-17 — H1 개정: 상시 피드백 전제 정정 → ALL_READY 존재 게이트 (STM, Claude Code)

### 전제 정정 (사용자)

- AK 모터는 **명령 무관 100Hz 상시 피드백** 송신하도록 설정돼 있음 — 구 COMM_LOST 설계의
  "피드백=명령 응답 → 비구동 구간 stale 갭" 전제가 무효

### 토론 확정

- mask 는 **의도 선언** 유지 (Orin/RS485 만 write, ECU 자동 갱신 없음 — 단일 소유 원칙).
  "tick==0 자동 감지로 레지스터 대체" 안은 부팅 시 커넥터 빠진 모터를 조용히 제외한 채
  3륜 주행하는 안전 구멍 + H1 원 결정("하나라도 무응답이면 전체 정지")과 모순이라 기각
- `MOTOR_COMM_FAULT_MS` 500→200ms (사용자 조정, 상시 100Hz 피드백이라 충분)

### 변경

- `rd_can_motor.c/h`: `RD_CAN_MOTOR_COMM_LOST`(on_since 기동 유예 + last==0 분기) 삭제 →
  **`RD_CAN_MOTOR_ALL_READY(mask)`** 신설 — "mask 전 모터 피드백 200ms 이내" 신선도 단일 판정.
  tick==0 은 now-0 이 항상 임계 초과라 자연히 not-ready (분기 불필요, 사용자 지적 반영)
- `rd_system.c ACTION_STATE_MANUAL/AUTO`: motor_on 전제조건에 ALL_READY **존재 게이트** 추가 —
  모터 전원 전 TX 미개시 → 빈 버스 ACK 폭주→FAULT (전원 인가 순서 문제) 코드로 원천 해소,
  늦게 켜진 모터 자동 합류. 비구동 미접촉은 "조용한 대기"(fault 아님)
- `rd_system.c RD_SYSTEM_UPDATE_STATE`: 구동 중(motor_on) !ALL_READY → motor_fault → ESTOP_SW
  (자동복귀형 유지). 주차 중 뽑힌 모터는 게이트가 motor_on 을 안 올려 헛 ESTOP 없음
- `CLAUDE.md`: 전원 인가 순서 운용 수칙 → 게이트로 무해화됨을 반영

### 실기 확인 항목 (기존 5항에 추가/대체)

1. ECU 먼저 부팅 + RC enable on + 모터 전원 off → TX 없음·에러 없음·조용한 대기 확인
2. 모터 전원 인가 → ~200ms 내 motor_on 자동 상승 (자동 합류)
3. 주행 중 모터 1개 분리 → 200ms 내 ESTOP_SW → 재연결 시 자동 복귀
4. 단일 트랙: default 0x0F 로는 구동 불가(게이트) → mask=0x01 write 후 정상 구동

## 2026-07-17 — 전체 로직 점검 (F1~F7) 및 수정 (STM, Claude Code)

### 점검 범위

rd_system.c 7개 태스크 + ISR 5종 전수 추적 (SYSTEM/CONTROL 분리·fail-safe·Orin write 종료
감지·auto_mode TODO·잔류 명령 제거·delta_tick 정합). 양호 확인: 루프 순서 정합, 워치독 3중화,
비구동 훅 커버리지 전수, pTsCmd 큐 경로 latch, hw 구조체/reg 발행 순서 일치, isr_err_take 원자성.

### 수정 (사용자 확정)

- **F1**: `RD_SYSTEM_UPDATE_STATE` — `RD_PERIPHERAL_READ`(GPIO+long-hold 리붓 감지)를 FAULT
  early-return **앞으로 이동**. 구조상 FAULT 에서 GPIO 리붓이 전면 차단되어 (uart2 FAULT 시
  Orin REBOOT 도 불가) 전원 재시작만 남던 모순 해소. FAULT 중 data_mtr 텔레메트리 동결도 해소
- **F3**: I2C 백오프 리셋 조건을 RET_OK(완전 정상)로 축소 — 인코더 일부 탈락 시
  WAIT→OFFLINE→복구 순환에서 백오프가 매번 리셋되어 무력화되던 문제 수정 (0.5s→1s 수렴).
  채널 분리는 하지 않음 (사용자 결정: 엔코더는 상시 5개 세트 사용 전제 — 부분 데이터 무의미,
  복구 시도 지속이 맞고 간격만 완화)
- **F4**: controlTask 의 `RD_PERIPHERAL_WRITE != OK → robot_state=FAULT` 제거 — NOK 는 NULL
  인자뿐인 죽은 경로 + 정책 변수 쓰기는 systemTask 단독 소유 원칙 위반
- **F5**: Orin mode write 도 GPIO 토글과 대칭으로 **1-tick ESTOP_SW 경유** — UPDATE_STATE
  FSM 뒤에 prev_mode 엣지 감지 (주행 상태에서만 발동, GPIO 경로는 MODE_DONE→ESTOP_HW 가
  우선이라 중복 없음). 제동 1펄스 + 훅 청소 + LPF 리셋 후 새 모드 진입
- **F7**: rd_uart.c — CR1 비원자 RMW 3곳 (checker soft-rearm 의 IDLE IT off/on,
  RS485_TRANSMIT 의 RE clear+IDLE off) 을 PRIMASK 가드(uart_crit_enter/exit)로 원자화.
  FreeRTOS 비의존 (드라이버 standalone 유지). HAL 내부 RMW 잔여 창은 수용 (자기치유)
- 죽은 변수 `any_comm_err` 제거 (H1 분리 후 잔재, (void) 처리돼 있던 것)

### 수정 안 함 (사용자 확정)

- **F2**: 주행 중 REBOOT 시 모터 정지 없음 — AK 모터 자체 CAN 무수신 타임아웃 설정 확인됨 → 무해
- **F6**: DIRECT 모드에서 ctr_mode(128~131)만 쓰는 write 는 워치독(132~187) 사각 — 오탐(안전측)
  방향이라 현행 유지. 범위를 128 로 넓히면 역으로 "ctr_mode 만 살아있고 값 스트림 죽은" 위험
  케이스에서 워치독이 안 걸리므로 현행이 안전측 (아래 F6 상세 참조)
- TIM5_IRQHandler 잔존 — 사용자가 CubeMX 에서 직접 제거 예정

### 실기 확인 항목

1. FAULT 상태에서 GPIO long-hold 리붓 동작 확인 (F1)
2. AUTO 주행 중 Orin 이 mode=0 write → 1-tick 제동 후 MANUAL 진입, LPF 0 재시작 (F5)
3. 인코더 1개 분리 → 복구 시도 간격 1s 수렴 + 나머지 4채널 읽기 유지 시간 증가 (F3)

## 2026-07-19 — §2.5 시계 동기·지연 계측 코드 빌드 검증 (개발머신, Claude Code)

### 검증 (HANDOFF_260719.md 작업 A)

- **A-1 변경 파일 존재 확인 — 통과**. 지시서 표 8개 항목 전수 확인, sync 누락 없음.
  `CommLatency.msg`(+CMakeLists 등록) / `rd_clock_sync.hpp`,`.cpp`(+`rd_bridge_lib` 소스 등록) /
  `rd_schedule.cpp` 5세그 배치(seg2 `88:36`, seg4 `228:1`)·`last_txn_`·`kWireOverheadBytes` /
  `rd_bridge.cpp` `PublishCommLatency`+`/carrier/testbed/comm_latency` 퍼블리셔 /
  `rd_register_ecu.hpp` `REG_PROC_DELTA_OFFSET=228`+DIAG_t 필드 /
  STM `rd_register_ecu.h` `rs485_proc_delta`+`reserved[27]` /
  STM `rd_system.c:625` `rd_delta_tick()` — `RD_PACKET_WRITE` 직전 위치 정확.
- **A-2 colcon 빌드 — 통과 (코드 수정 0건)**. `mgs01_base_msgs` 7.4s, `orin_firmware_bridge` 22.3s,
  양쪽 stderr 출력 없음 (경고 0, 기존 관례 유지). §2.5 코드는 초회 빌드에서 무수정 통과.
- **환경 블로커 1건 (코드 무관)**: 이 개발머신에 `libserial-dev` 미설치로 CMake
  `pkg_check_modules(LIBSERIAL REQUIRED libserial)` 실패 → 사용자가 `apt install libserial-dev`
  (1.0.0-7) 설치 후 재빌드 통과. 신규 머신 세팅 시 선행 의존성으로 기록해 둘 것.
- **A-3 STM CubeIDE 빌드 — 불가**. 이 머신에 ARM 툴체인(`arm-none-eabi-gcc`)·CubeIDE 모두 없음.
  STM 2파일 변경분은 **⚠ STM32CubeIDE 빌드 검증 필요 (별도 머신)** 상태로 남음.

### 미실행

- 작업 B(실기 드라이런): ECU 하드웨어 미연결 + STM 플래시 미완 → 착수 불가.

## 2026-07-19 — testbed_spec §6 #1: 테스트베드 인터페이스 4종 정의 (개발머신, Claude Code)

### 변경

- `mgs01_base_msgs/msg/TestbedFeedback.msg` (신규) — spec §3.4 필드 정의 그대로. 200Hz 통합 피드백.
  `goal_id`(분석 자동 분할 키)·`testbed_state`·`rtt`·`clock_offset`(+valid) 포함. `TractionTest.msg`
  의 후속이나 **구 msg 는 이번에 제거하지 않음** (§6 D3·D4 폐기 시점 = C-6, 분석 파이프라인 구버전
  bag 분기 유지 필요).
- `mgs01_base_msgs/msg/CmdMotor.msg` (신규) — spec §3.5. Task 3 예약, **인터페이스 정의만** (STREAM
  소비 로직은 이번 범위 밖).
- `mgs01_base_msgs/srv/TestbedConfig.srv` (신규) — spec §3.3. op 코드는 매직넘버 대신 `OP_*` 상수로
  선언 (0~4 값은 스펙과 동일) — 클라이언트/CLI 가 숫자 리터럴 없이 쓰도록.
- `mgs01_base_msgs/action/RunProfile.action` (신규) — spec §3.2 Goal/Result/Feedback 3부 그대로.
- `CMakeLists.txt` / `package.xml` — 위 4종 등록 + `action_msgs` 의존 추가 (action goal/result 래퍼).

### 결정

- `loadcell_raw` 는 스펙대로 `int32[2]` 채택 (구 `TractionTest.msg` 는 `uint16[2]`). 스펙 우선 규칙
  적용 — 원시 ADC 부호/범위 확장 여지를 남기는 방향이라 타당.

### 검증

- `colcon build --packages-select mgs01_base_msgs` 통과. 초회 빌드의 stderr 경고는 WSL 파일시스템
  clock skew 였고, `build/`·`install/` 삭제 후 **클린 재빌드에서 경고 0** 확인 (코드 무관 확정).
- `ros2 interface list` 로 4종 생성 확인 + `ros2 interface show` 로 세 인터페이스 필드·타입이
  스펙과 1:1 일치함을 실물 확인.
- 회귀: `orin_firmware_bridge` 재빌드 통과 (기존 `TractionTest`/`CommLatency` 소비 코드 영향 없음).
- ⚠ 런타임(실기) 검증은 미실시 — ECU 하드웨어 미연결.

## 2026-07-19 — testbed_spec §6 #2: 테스트베드 FSM + write 소스 선택 + LOCKED 래치 (개발머신, Claude Code)

### 변경

- `rd_testbed.hpp/.cpp` (신규) — spec §2 FSM 을 독립 모듈로 분리. `TestbedState`(INIT/IDLE/
  RUNNING/STREAM/LOCKED) + 전이 API(`MarkInitDone`/`BeginProfile`/`EndProfile`/`Lock`/`Rearm`)
  + write 소스 선택(`SelectWrite`) + config 수락 판정(`AcceptsConfig`). 상태 변수는 클래스 밖으로
  노출하지 않고 전 메서드가 자체 mutex 보호 — 200Hz 스케줄 루프와 spin 스레드(service/action)
  동시 접근 전제.
- `rd_bridge.cpp` `PrepareControlCommand` — write 소스를 **FSM 단독 결정**으로 전환 (D6 Single
  Writer). FSM 값에도 `cmd_current_max_` 클램프를 wire 직전 재적용 (프로파일 검증 통과값의 최종 방어).
- `rd_map.hpp` — `RwWriteErrStreak()` getter 추가. 기존 `rw_write_err_cnt_` 는 write 성공 시 0
  리셋이라 값이 곧 연속 거부 tick 수 → LOCKED 판정에 그대로 사용 (신규 카운터 도입 불필요).
- `rd_schedule.cpp` — control 브랜치에서 매 tick `NoteWriteErrStreak()` 호출 (연속 50 tick =
  0.25s 거부 시 LOCKED). 루프 진입 시 `MarkInitDone()` 호출.

### 결정·주의

- **`/carrier/cmd_current` 는 이 시점부터 wire 에 반영되지 않는다** (D3 폐기 경로). 구독자 제거는
  C-6 이지만 write 소스가 FSM 으로 넘어간 이상 무음 무시가 되므로, 콜백에서 5s 스로틀 WARN 을
  남기도록 했다 (기동 로그에도 명기). 기존 MPC 사용자가 "명령을 보냈는데 안 도는" 상황을 겪지
  않게 하려는 조치.
- **⚠ 잠정 코드 1건**: `MarkInitDone()` 을 루프 진입 시 무조건 호출 — C-3(§3.1)에서 motor_mask(192)/
  mode(190) WRITE+검증 플로우로 **대체되어야 하며**, 검증 실패 시 IDLE 대신 노드 종료(exit≠0)가
  되어야 한다. 해당 위치에 ⚠ 주석 명기.
- STREAM 은 Task 3 예약이라 진입 경로 없음 — `SelectWrite` 에서 0A(안전측) 반환만 정의.

### 검증

- `colcon build --packages-select orin_firmware_bridge` 통과, 경고 0.
- **FSM 단위 검증 하네스** 작성·실행 (실기 불가 대체): spec §2 표의 상태별 write 값·진입·이탈·
  허용입력을 5개 그룹 27개 단언으로 대조 → **ALL PASS**. 확인 항목: IDLE 0A 고정(C-2 완료기준),
  RUNNING 중 새 goal 거부, RUNNING/LOCKED config 거부·REARM 만 허용, LOCKED 가 RUNNING 잔류
  명령을 무시하고 0A 래치, 연쇄 래치 시 최초 사유 보존, 스트릭 0 복귀해도 자동 해제 안 됨(명시
  REARM 요구), EndProfile 후 goal_id/샘플 잔류 제거.
  - 하네스는 스크래치패드에 둠 (레포에 테스트 관례 없음) — 상시 회귀로 승격할지는 사용자 판단.
- ⚠ 실기 검증 미실시 — ECU 하드웨어 미연결. 실제 0A wire 반영·write 거부 스트릭 발동은 미확인.

## 2026-07-19 — testbed_spec §6 #3: active_motors INIT 플로우 (개발머신, Claude Code)

### 변경

- `rd_bridge.cpp` — `active_motors` 파라미터(기본 `[1,2,3,4]`) 선언·검증 → `motor_mask` 비트필드
  계산. 범위 밖(1~4 아님)·빈 리스트·타입 오류는 전부 `active_motors_valid_=false` 로 표시.
- `rd_schedule.cpp` `InitTestbed()` (신규) — §3.1 플로우: ① motor_mask(192) WRITE+검증
  → ② mode(190)=1(AUTO) WRITE+검증 → 성공 시 `MarkInitDone()`(FSM IDLE). 각 단계
  0.2s 간격 10회 재시도, 전부 실패 시 `RD_FATAL`.
- `rd_schedule.cpp` `WriteVerifyByte()` (신규) — 1B WRITE 후 같은 주소 READ 로 read-back 검증.
  **매 시도마다 섀도를 기대값으로 재설정** — 직전 시도의 READ decode 가 섀도를 ECU 실값으로
  덮으므로, 빠뜨리면 2회차부터 엉뚱한 값을 쓰게 된다 (구현 시 실제로 걸리기 쉬운 함정).
- `MainLoopStart()` 반환형 `void`→`int`, `main.cpp` 가 그 값을 프로세스 종료 코드로 사용.
  `SupervisorLoop` 은 `init_fatal_` 일 때 재접속 재시도를 건너뛰고 즉시 종료 (설정 오류는
  재시도해도 같은 결과).
- `rd_register_ecu.hpp` — `REG_MODE_OFFSET(190)`/`MODE_MANUAL`/`MODE_AUTO`/
  `REG_MOTOR_MASK_OFFSET(192)` 상수 추가 (매직넘버 제거).
- C-2 의 ⚠ 잠정 `MarkInitDone()` 무조건 호출 **제거됨** — 부채 해소.

### 결정

- **파라미터 오류는 USB 대기 전에 판정** (`MainLoopStart` 선두). 스펙에 명시된 순서는 아니지만,
  오타 하나로 "Waiting for USB..." 만 반복하며 매달리는 상황을 막기 위함. 통신 시도 없이 exit=1.
- INIT 순서(mask → mode)는 스펙 그대로 유지 — AUTO 진입 전에 마스크를 확정해야 의도치 않은
  트랙이 한 tick 도 돌지 않는다.

### 수정 (부수 발견 — 기존 버그)

- `RdBridge::Start()` 의 spin 스레드가 **종료 경합 시 SIGABRT** 로 죽던 문제 수정. 기동 직후
  종료하면 `rclcpp::shutdown()` 이 spin 진입보다 먼저 실행돼 무효 컨텍스트에서 guard condition
  생성이 `RCLError` 를 던지고 프로세스가 코어를 덤프했다 (실측 exit=134 재현). `ok()` 검사 +
  `RCLError` 흡수로 조용히 빠져나오게 함. 기존 코드의 잠재 버그였고 C-3 의 조기 종료 경로가
  드러냈다 — 정상 장기 실행 경로에선 창이 좁아 안 보이던 것.

### 검증 (실기 없이 가능한 범위 전부)

- `colcon build` 통과, 경고 0.
- 파라미터 검증 실측: 기본값 → `mask=0x0F` / `[2,3]` → `mask=0x06` (부분 트랙) /
  `[1,5]`·`[0]` → 범위밖 거부 후 **exit=1** (3회 반복 재현) / 유효값 → USB 대기 정상 진입.
- ⚠ **미검증 (ECU 필요)**: WRITE+검증 재시도 루프의 실제 동작, read-back 불일치 처리,
  10회 실패 시 exit≠0 경로, AUTO 진입. `Initialize()`(USB 개통)가 선행이라 HW 없이는
  이 경로에 도달 자체가 불가 — 작업 B 드라이런 시 최우선 확인 항목.

## 2026-07-19 — testbed_spec §6 #4: 프로파일 player + run_profile action (개발머신, Claude Code)

### 변경 (C-4a: 파싱·검증·사전 샘플링)

- `rd_profile.hpp/.cpp` (신규) — §4 스키마 전체 구현. 역할은 "파싱 → 검증 → 사전 샘플링" 까지로
  한정하고, 재생 중에는 배열 인덱싱만 한다 (§4.3-5, 200Hz 예산 보호).
  - 세그먼트 9종 전부: hold/ramp/stair/step/sine/chirp/prbs/noise/custom.
  - `ramp` 는 마지막 tick 이 정확히 `to` 가 되도록 (n-1) 로 나눈다 — 히스테리시스 실험에서
    상승 끝값과 하강 시작값이 어긋나면 안 되기 때문.
  - `custom` 은 rate≠200Hz 를 최근접 이웃으로 200Hz 리샘플 (웹 드로잉 경로가 임의 rate 를 낸다).
  - `prbs`/`noise` 는 seed 기반 `mt19937_64`, 모터별 독립 스트림(seed+motor_no)으로 재현성 유지.
  - 검증 5규칙 전부: ⊆active_motors reject / 짧은 쪽 0A 패딩 / min(profile,전역) 클램프+횟수 기록 /
    slew 위반은 **성형이 아니라 reject** / 사전 샘플링 완료 후에만 RUNNING.
  - 오류 사유에 **모터·세그먼트 번호·키 이름**을 담는다 (YAML 문법 오류는 yaml-cpp 의 line/col).
  - 길이 상한 3600s (사전 샘플링 메모리 방어).
- `CMakeLists.txt`/`package.xml` — `yaml-cpp` 의존 추가.

### 변경 (C-4b: action 서버 + tick 재생)

- `rd_bridge.cpp` — `/carrier/testbed/run_profile` action 서버. goal 콜백에서 파싱·검증·사전
  샘플링까지 끝내고 실패는 전부 reject (RUNNING 진입 후 죽는 것보다 낫다). 실행은 별도 스레드
  (executor 블로킹 방지), 5Hz 피드백, cancel → 0A → IDLE.
- `RdBridge::TickProfile()` — 스케줄 루프가 매 tick 호출, 사전 샘플 1개를 FSM 에 주입.
- `RdMap::RwWriteErrTotal()` — 누적 write 거부 카운터 추가 (기존 스트릭 카운터는 리셋되므로
  result 의 `write_err_cnt` 는 구간 차분으로 구한다).
- action 서버는 **INIT 성공 후에만 오픈** — 검증 안 된 상태로 재생되는 창을 없애기 위함.

### 스펙 의문 (§5 기록 대상)

- **`segment_index` 의 정의가 모터별 세그 목록이 다를 때 불명확**. §3.4 는 단일 필드인데 모터마다
  세그먼트 수·경계가 다를 수 있다. 구현은 **기준 모터(세그먼트 수 최다, 동수면 번호 작은 쪽)**의
  인덱스를 쓰고 패딩 구간은 마지막 세그 번호를 유지하도록 했다 — 경계가 가장 많이 드러나는 쪽이
  분석에 유리하다는 판단. 노트북 세션이 스펙을 확정해 주면 그에 맞춘다.
- §4.1 "샘플 단위 = 모터의 ctr_mode 를 따름" 은 현재 write 경로가 CURRENT 고정이라 전부 [A] 로
  다뤘다. SET_CTR_MODE(C-5) 이후 모드가 갈리면 그때 확장 필요.

### 검증 (실기 없이 가능한 범위 전부)

- `colcon build` 통과, 경고 0.
- **C-4a 하네스 60 단언 ALL PASS**: §4.1 예시 프로파일(46s/9200tick, ramp 끝값·중간값·세그 전이),
  9종 세그먼트 파형 정확성(stair 계단값/step 경계/sine 1·3분기/chirp 진폭/prbs 이진성·시드
  재현성/custom 리샘플), 검증 5규칙, 오류 12종의 사유 문자열에 위치가 실제로 담기는지.
- **C-4b 하네스 28 단언 ALL PASS** (시리얼 없이 노드 구성 + 스케줄 tick 모사): 정상 재생
  (ticks_executed=400 정확), goal_id 단조증가, 검증 실패 3종 reject 후 IDLE 유지, 클램프 수락
  +clamp_cnt, **cancel → success=false/"canceled"/0A·goal_id=0 복귀**, RUNNING 중 동시 goal
  reject, LOCKED 에서 goal reject → REARM 후 정상 재생.
- ⚠ **미검증 (ECU 필요)**: 실제 wire 반영, 200Hz 실시간 재생 시 tick 지터, write 거부 발생 시
  result 의 write_err_cnt 집계. 작업 B 이후 확인 필요.

## 2026-07-19 — testbed_spec §6 #5: config service (개발머신, Claude Code)

### 선행: 스펙 확정 2건 반영 (노트북 회신 §6.1)

- `segment_index` 기준 모터 방식 **승인 — 구현 유지**. 주석을 "권위 소스는 YAML+profile_time,
  이 필드는 보조 지표"로 갱신 (추가 공수 금지 지침 반영).
- 샘플 단위 **v1 = CURRENT[A] 전용 확정**. `rd_profile.hpp` 주석에서 "그때 확장" → "TEST4 확장
  예약, 구현하지 않음"으로 갱신. RUNNING 재생은 상태 무관 항상 CURRENT.

### 변경

- `rd_bridge` — `/carrier/testbed/config` 서비스 (`TestbedConfig`) 5 op 구현.
- **out-of-span 경로** (`DoOutOfSpanWrite`, mask192/mode190): service 가 요청을 걸고 스케줄러가
  RW 1 tick 을 일반 WRITE 패킷으로 대체 → 다음 tick READ 로 read-back → cv 로 응답.
  회신 §6.2 제약을 코드 구조로 강제:
  ① `oos_in_flight_` 단일 슬롯 — in-flight 중 새 요청은 큐잉 없이 즉시 거부
  ② 대체 tick 은 `continue` 로 빠져 **피드백/CommLatency 발행 자체를 건너뜀** (보간 없음)
  ③ WRITE 실패 시 phase 를 READ 가 아닌 DONE 으로 — READ tick 을 더 훔치지 않고 즉시 복귀
  ④ 최대 2 tick (WRITE+READ), 50ms/10tick 타임아웃
- **in-span 경로** (`DoInSpanCtrMode`, ctr_mode 128~131): 섀도만 수정 → 다음 RW tick 자연 반영.
  검증을 위해 `task_control_` read 세그에 `{128,4}` 추가 (요청 79B / 응답 69B, STM 버퍼 90B 이내).
- `RdSchedule::RegBytePtr()` — out-of-span 대상 주소를 화이트리스트로 제한 (엉뚱한 주소 쓰기 차단).
- `PrepareControlCommand` — ctr_mode 를 섀도(`ctr_mode_[]`)에서 가져오되 **RUNNING 중에는 항상
  CURRENT** (spec §7-0). IDLE 에서만 SET_CTR_MODE 값이 유효.
- `RdBridge::Start()` — `rclcpp::spin` → **MultiThreadedExecutor**. config service 가 read-back
  검증으로 최대 50ms 블록되는 동안 action 피드백·취소가 멈추면 안 되기 때문. 기본 콜백 그룹은
  MutuallyExclusive 라 **기존 콜백들 간 직렬성은 그대로** — 전용 그룹의 config service 만 병렬.

### 결정

- **GET_STATUS 는 상태 무관 허용**. §2 표대로면 RUNNING/LOCKED 에서 거부지만, 부작용이 없고
  RUNNING 중 상태 조회를 막으면 CLI(§5.1 `status`)·웹 대시보드가 무력화된다. 표의 '허용 입력'을
  **상태를 바꾸는 입력에 대한 규정**으로 해석했다. 이견 있으면 알려줄 것.

### 검증

- `colcon build` 통과, 경고 0.
- **하네스 42 단언 ALL PASS** (C-4b 방식 + 가짜 ECU 로 대체 tick 처리, 회신 §6.2-5 지침):
  GET_STATUS / SET_ACTIVE_MOTORS 가 **정확히 2 tick** 소모 / SET_MODE 0·1·거부 /
  **WRITE 실패 시 1 tick 만 소모하고 복귀** / read-back 불일치 사유 / **50ms 타임아웃 후
  in-flight 해제되어 재요청 성공** / SET_CTR_MODE 비활성·범위 거부 + 성공 경로(VELOCITY·ESTOP) /
  **RUNNING 중 4종 전부 거부, GET_STATUS 만 허용** / LOCKED→REARM→정상 복귀.
- ⚠ **미검증 (ECU 필요)**: 실제 tick 대체 시 200Hz 주기 영향(2 tick 갭), ECU 가 실제로 mask/mode
  를 수용하는지, in-span read 세그 확장(69B) 후 STM 응답 정상 여부.

## 2026-07-19 — testbed_spec §6 #6: 피드백 개편 + 구 인터페이스 완전 폐기 (개발머신, Claude Code)

### 변경 (노트북 회신 §6.2b "완전 폐기" 지침대로)

- `TractionTest.msg` **파일 삭제** + `mgs01_base_msgs/CMakeLists.txt` 등록 제거.
- `/carrier/ecu/traction_test` + `PublishTractionTest()` **삭제** → `/carrier/testbed/feedback`
  + `PublishTestbedFeedback()` (`TestbedFeedback`, §3.4)로 대체. **traction_test_mode 에서도 발행**
  (해당 모드는 FSM 이 INIT 이라 goal_id=0, testbed_state 는 그대로 실린다).
- `/carrier/cmd_current` 구독 + `CallbackCmdCurrent` + `cmd_current_timeout` 파라미터 **삭제** (D3).
  `cmd_current_max` 는 지침대로 **유지** — player §4.3-3 전역 클램프로 계속 쓰인다.
- `RdMap::LastRwErr()` 추가 — RW 응답의 에러 니블(read|write<<4)을 보관해 `rw_err` 필드로 전달.
- CommLatency 토픽/발행은 **유지** (계측 원자료). TestbedFeedback 의 rtt/clock_offset 은 요약 필드로
  역할 분리 — §6.2b 그대로.
- `rosbag_test.sh` / `README.txt` 의 구 토픽명을 신 토픽명으로 갱신.
- `analysis/traction/` 은 지침대로 **불변** (#8 은 별도 작업).

### 결정·주의

- **시계 추정기 재호출 금지**: `TestbedFeedback` 의 rtt/clock_offset 을 채우려고
  `clock_sync_` 를 다시 Update 하면 **같은 트랜잭션을 두 번 먹여 추정이 오염된다**.
  `PublishCommLatency` 가 유일한 Update 지점이고, 그 결과를 멤버에 캐시해 피드백이 읽도록 했다.
  이에 맞춰 스케줄러의 발행 순서를 **CommLatency → TestbedFeedback** 으로 정렬 (캐시 최신화 선행).
- `grep -r TractionTest orin_ws/src` 잔존 2건은 **`IsTractionTestMode()` — 모드 이름**이며
  msg 타입이 아니다. §6.2b 가 traction_test_mode 자체는 유지(그 모드에서도 발행)를 전제하므로
  의도된 잔존. `ros2 interface list` 에서 `mgs01_base_msgs/msg/TractionTest` 는 완전히 사라졌다.

### 검증

- `mgs01_base_msgs` 클린 재빌드 + `orin_firmware_bridge` 빌드 통과, 경고 0.
- **신규 하네스 24 단언 ALL PASS**: 섀도에 심은 값으로 **필드 매핑·스케일 전수 확인**
  (fb_current ×0.01 / fb_velocity ×10 / fb_position ×0.1 / loadcell·tick 원본),
  IDLE goal_id=0 → RUNNING goal_id=1·profile_time 증가 → 취소 후 0 복귀 (D4 분석 분할 키),
  **구 토픽 발행자 0 / 신 토픽 발행자 1 / cmd_current 구독자 0**, tick 당 1건 발행(누락 없음).
- **기존 하네스 4종 전부 회귀 PASS** (fsm 27 / profile 60 / action 28 / config 42).
- ⚠ **미검증 (ECU 필요)**: 실기 200Hz 발행률, rw_err 실제 값, 응답 69B 후 STM 정상성.

## 2026-07-19 — testbed_spec §2.6 (C-8): auto_mode ↔ write 범위 연동 + KINEMATIC 덮어쓰기 버그 수정

### 사전 확인 (ECU 코드로 회신 주장 재검증)

- `rd_system.c:195` KINEMATIC(case 0)이 100Hz 로 `ctr_mode[i]=MODE_VELOCITY` 덮어씀 — **확인**.
  case 1(CURRENT)은 `ctr_mode=MODE_CURRENT` 강제(자가치유), case 2(DIRECT)는 무가공 통과 — 확인.
- 워치독: `rd_map_ecu.c:146` `is_cmd_vel = (addr<=187) && (addr+len-1>=132)`.
  164:16 → 164<=187 ✓, 179>=132 ✓ → `cmd_write_tick` 갱신 유지 — **축소해도 AUTO_TIMEOUT 안 걸림 확인**.

### 변경

- `rd_register_ecu.hpp` — `REG_AUTO_MODE_OFFSET(188)` + `AUTO_MODE_KINEMATIC/CURRENT/DIRECT/CONTROL`.
- **INIT 3단계로 변경**: motor_mask(192) → **auto_mode(188)** → mode(190)=AUTO.
  auto_mode 를 AUTO 진입 **전에** 확정해 KINEMATIC 활성 구간을 한 tick 도 거치지 않는다.
- 기동 파라미터 `auto_mode`(기본 1). 0·3 은 기동 거부 + **USB 대기 전 fail-fast**(exit=1) —
  C-3 의 active_motors 와 동일한 처리.
- **write 범위 = auto_mode 파생** (§2.6 제약 1): `task_control_current_`(164:16) /
  `task_control_direct_`(128:52) 를 **생성자에서 미리 만들고** 매 tick `AutoMode()` atomic 으로
  **고른다**. 런타임 구조체 변경 없음 (200Hz 스레드 레이스 원천 차단).
- `SET_AUTO_MODE`(op=5) 신설 — 1·2 만 허용. **1→2 는 shadow 소독(ctr_mode=CURRENT/pos=0/vel=0)
  → WRITE+검증 → 범위 확장** 순서, 2→1 은 WRITE+검증 → 축소 (§2.6 제약 2). 응답에 전환 후
  write 범위 명시.
- **프로파일 재생 가드**(§2.6 제약 3): goal 수락 시 활성 모터 전부 `ctr_mode==CURRENT` 아니면 reject.
- `GET_STATUS` 에 auto_mode + write 범위 노출.

### 수정 (하네스가 드러낸 내 구현의 버그)

- **`PrepareControlCommand` 가 §2.6 원칙을 위반하고 있었다**. auto_mode=1 이면 write 범위가
  164:16 이라 ctr_mode 는 wire 로 나가지 않는데, 매 tick 섀도의 `ctr_mode/pos/vel` 을 덮어써
  **read 세그 {128,4} 로 들어온 ECU 실값을 지우고 있었다**. §2.6-3 가드가 그 섀도를 읽으므로
  ECU 가 VELOCITY 여도 bridge 자신이 쓴 낙관적 CURRENT 를 보고 **통과해버리는** 실패가 가능했다
  (하네스에서 산발 재현 — 5회 중 1회). 수정: **섀도에는 bridge 가 소유한 것만 쓴다** —
  CURRENT 모드는 cmd_current 만, DIRECT 모드에서만 ctr_mode/pos/vel 까지. 수정 후 5회 연속 통과.
- 파급: auto_mode=1 에서 `SET_CTR_MODE` 는 무의미(ECU 가 CURRENT 강제 + wire 미전송)해졌으므로
  **사유를 명시해 거부**하도록 변경 (조용히 10 tick 타임아웃되는 것보다 낫다).
  → 스펙 §3.3 은 SET_CTR_MODE 의 auto_mode 의존을 명시하지 않음. §5 에 기록.

### 검증

- 빌드 통과, 경고 0.
- **하네스 30 단언, 5회 연속 ALL PASS**: ⓐ 주소·상수 계약 + INIT 호출 순서(소스 검증:
  mask→auto_mode→mode) ⓑ 셀렉터 전환 + 응답의 범위 명시 ⓒ **WRITE 시점 shadow 소독 완료 관측**
  + WRITE→READ 2 tick trace ⓓ 활성 모터 VELOCITY/POSITION 시 goal reject·비활성 모터는 무관
  ⓓ2 SET_CTR_MODE 의 DIRECT 전용 계약 ⓔ 0·3·9 거부 + 거부 후 셀렉터 불변.
- `auto_mode:=0/3` → **exit=1 실측**, `1/2` → 정상 USB 대기 진입.
- ⚠ **미검증 (ECU 필요)**: 실제 INIT 3단계 실행, 범위 축소 후 AUTO_TIMEOUT 미발동, DIRECT 전환의
  실제 wire 동작. 작업 B 확인 항목.

### ⚠ 하네스 유실 사고 (기록)

- C-1~C-6 하네스 5종(fsm/profile/action/config/feedback, 157 단언)이 **스크래치패드에 있던 탓에
  세션 간 삭제됨** — C-8 회귀 실행 불가. C-2 시점에 "상시 회귀로 승격할지는 사용자 판단"으로
  남겨둔 리스크가 현실화된 것. **레포 내 이관 권고** (예: `orin_ws/test/` + colcon test).

## 2026-07-19 — C-7.5: 하네스 레포 이관 (개발머신, Claude Code)

### 변경

- `orin_firmware_bridge/test/` 신설, `ament_cmake_gtest` 로 `colcon test` 등록 (6종 = 6 실행파일).
  파일당 실행파일이라 DDS/노드 상태가 테스트 간 섞이지 않는다.
  - `rd_test_common.hpp` — rclcpp 전역 환경(실행파일당 1회 init) + `WaitFor()` 폴링 유틸.
    고정 sleep 은 느린 머신에서 깨지고 빠른 머신에서 시간을 버리므로 조건 폴링으로 통일.
  - `rd_test_fixture.hpp` — 노드 + 가짜 ECU + tick 모사 픽스처. 스케줄 루프의 실제 순서
    (out-of-span 대체 → TickProfile → PrepareControlCommand → read 세그 echo)를 재현.
    시리얼만 가짜이고 그 위 로직은 전부 실제 코드가 돈다.
  - `test_testbed_fsm`(C-2) / `test_profile_parser`(C-4a) / `test_run_profile_action`(C-4b) /
    `test_config_service`(C-5) / `test_auto_mode`(C-8) / `test_active_motors`(C-3).
  - 각 파일 헤더에 **"막는 것"** 1~2줄 명시 (지시 §6.2e-2).
- C-3 은 계약이 프로세스 **종료 코드**라 in-process 가 아니라 `$<TARGET_FILE:comm_test_node>` 를
  서브프로세스로 띄워 확인 (유효 → USB 대기 timeout 124 / 무효 → 1).
- `package.xml` `ament_cmake_gtest` test_depend 추가. 기존 `ament_lint_auto` 블록은
  `if(FALSE)` 로 비활성 유지 (원래 스킵 설정이었고, 이번 범위에서 린트 도입은 별건).

### 수정 (이관 중 발견)

- 픽스처 초기 섀도 `ctr_mode=0(ESTOP)` 때문에 `ConfigServiceTest.RunningRejectsEverythingButStatus`
  가 산발 실패. 실제로는 INIT(auto_mode=1) 직후 ECU 가 CURRENT 를 보고하므로 픽스처도 그 상태에서
  시작하도록 수정 — 테스트 타이밍 의존 제거.

### 검증 (§6.2e-3 C-8 회귀 — 통과)

- **`colcon test` 42 케이스 / 226 단언, 0 failures — 3회 연속 동일**.
  `PrepareControlCommand` 수정이 직접 건드린 C-4b 재생 경로(7 케이스)와 C-5 검증 경로(8 케이스)
  모두 통과 → **C-8 회귀 증거 확보**.
- **회귀 테스트가 실제로 버그를 잡는지 역검증**: `PrepareControlCommand` 의 소유권 분기를
  버그 상태(`direct = true`, 항상 전부 덮어쓰기)로 되돌려 재빌드 → `test_auto_mode` **1 failure**
  발생 확인 → 수정 원복 후 재통과. "통과하는 테스트"가 아니라 "실패를 잡는 테스트"임을 확인.
  - 이 과정에서 초기 작성한 §2.6 제약 0 테스트가 **버그를 못 잡는 형태**였음을 발견해 교체:
    tick 루프가 도는 중에는 read 세그 모사가 덮어쓴 값을 되돌려놔 가려진다. 루프를 멈추고
    `PrepareControlCommand()` 를 직접 호출해 계약을 찌르는 방식으로 변경.
- ⚠ 실기 무관 (전부 ECU 없이 도는 테스트). 실기 검증 항목은 작업 B 로 계속 이월.

## 2026-07-19 — testbed_spec §6 #7 (C-7): testbed_cli (개발머신, Claude Code)

### 변경

- **신규 패키지 `testbed_cli`** (ament_python, entry point `testbed_cli`).
  - `record.py` — **ROS 비의존** 폴더 규격 모듈. 폴더 계약은 CLI 의 핵심 산출물이고 ECU 없이
    검증 가능해야 하므로 ROS 호출과 분리했다. `create_run_dir`(중복 시 _2,_3 — 덮어쓰면 앞
    실험이 사라진다) / `copy_profile`(원문 그대로) / `write_result` / `verify_run_dir`(누락 항목
    리스트) / `profile_label`(라벨 우선순위: --name > YAML name > 파일명).
  - `cli.py` — `status` / `config motors|ctr_mode|mode|auto_mode` / `rearm` / `run [--record]` /
    `abort`. 스펙 §5.1 커맨드 전부 + `auto_mode`(§3.3 op5 신설분) 추가.
- 폴더 규격: `data/rosbags/<name>_<MM-DD_HH-MM>/` = `bag/` + `profile.yaml` + `result.json`.
  실패한 실험도 기록한다 (왜 실패했는지가 데이터).
- `abort` 는 UUID·stamp 0 인 CancelGoal 요청 = action 규약상 '전부 취소'. 별도 프로세스인 CLI 는
  goal handle 이 없으므로 이 경로를 쓴다.
- bag 종료는 **SIGINT** — SIGKILL 하면 metadata.yaml 이 안 써져 bag 이 열리지 않는다.
- run 종료 시 CLI 가 스스로 `verify_run_dir` 로 규격을 확인하고 미충족 시 WARN (조용히 넘기면
  분석 단계에서야 발견된다).

### 수정 (전체 테스트 실행에서 드러난 것)

- **DDS 도메인 격리 추가** (양 패키지). CLI e2e 의 가짜 bridge 는 실제 bridge 와 **같은 토픽·서비스
  이름**을 쓴다. `colcon test` 가 패키지를 병렬로 돌리자 C++ 테스트와 서로를 발견해 요청이 엉뚱한
  상대에게 가면서 **6~7건 실패**가 났다 (패키지별 단독 실행에서는 안 보였음). 프로세스마다
  PID 기반 `ROS_DOMAIN_ID` + `ROS_LOCALHOST_ONLY=1` 로 독립 그래프를 갖게 해 해결.
  → 같은 LAN 의 실제 로봇과도 격리되므로 실기 환경에서 테스트를 돌려도 안전하다.
- **기존 결함 수정**: `mgs01_base_msgs/package.xml` 의 `test_depend` 가 `member_of_group` 뒤에
  있어 package format3 스키마 위반 (`xmllint` 테스트 실패). **내 변경 이전부터 있던 문제**임을
  `git show HEAD:` 원본으로 확인 후 순서 교정.

### 검증

- **`colcon test` 전체 65 케이스, 0 failures — 3회 연속** (bridge gtest 42 + CLI pytest 19 + lint 4).
- CLI pytest 19: 폴더 규격 단위(경로 탈출 차단·중복 방지·원문 보존·실패 기록·누락 검출) +
  인자 파싱(스펙 커맨드 형태) + **e2e 5종**(가짜 bridge 상대로 CLI 를 실제 프로세스로 실행 —
  op 코드·모터 인자 배선, YAML 원문 전달, `--record` 폴더 규격, 거부 시 exit≠0 + 기록 유지).
- **실기 대체 수동 확인**: 가짜 bridge 상대로 `run --record` 1회 실행 → 폴더에 `bag/metadata.yaml`,
  `bag/bag_0.db3`, `profile.yaml`, `result.json` 생성 확인. **`ros2 bag record` 연동이 실제로
  동작함**을 확인했다 (테스트는 bag 부재도 허용하지만 이 환경에서는 실제 생성됨).
- ⚠ **완료 기준의 '실기 run 1회'는 미충족** — ECU 없이는 불가. 다만 폴더 규격·CLI 배선·bag 연동은
  위와 같이 검증됨. 실기에서 확인할 잔여: 실제 200Hz 스트림이 담긴 bag 의 내용·크기, 장시간 run.

## 실기 검증 세션 (2026-07-20, HANDOFF_VERIFY_260719 수행)

> 개발머신(Legion, x86_64) + ECU(/dev/ttyUSB0) 실기. STM 플래시 최신본 3델타(07-16/17/19) 사용자 확인.
> 선행: 빌드가 stale(07-07)여서 clean colcon build 재수행(testbed_cli 미설치였음 → 4패키지 빌드 OK).

- **V0 새니티 PASS** — comm_test_node(일반). 헤더비트 ECU[ON], over-period 0/400(0.0%) 지속,
  avg exec ~1.0~1.4ms(스펙 ~2.3ms 이하 만족), Loss=2 고정(startup only, Rx=Tx-2로 정체 → 정상성장 0).
  `/carrier/ecu/fsm`=1(MANUAL, RC無 정상), `/carrier/ecu/imu`=100.00Hz(std 0.09ms) → 레지스터맵 정합 간접확인.
  참고: dev머신이라 SCHED_FIFO 실패 WARN(rtprio 미설정) — §1-4 판정완화 대상, 타이밍 영향 無(0% over-period).

- **V1 시간동기·지연 PASS (drift 크기 1건 단서 있음)** — traction_test_mode.
  - 200Hz: bag 13.3분(796.8s)에 comm_latency 159,336건 = **199.96Hz 지속**, feedback 동수(200Hz) → 응답 65B 확장 부하 영향 無.
  - rtt 1.87~2.26ms ✓ / wire_down 0.79ms ✓ / **wire_up 0.30ms** (문서 기대 ≈0.9ms보다 낮음 —
    wire=bytes×kByteTime 이므로 요청 바이트수가 문서 가정보다 작은 것. 내부 정합 O, 결함 아님).
  - **proc_delta_prev 0.1ms 비영** → 07-19 rs485_proc_delta(228) 플래시 확인됨.
  - offset_valid: 기동 **~6s** 에 "시계 동기 수렴" INFO 1회 (기준 ≤30s) ✓, 이후 4분간 계속 true.
  - ⚠ **drift_ppm ≈ -19,800 ppm (-1.98%)** — 문서 기대치 "수십 ppm" 과 3자릿수 차이.
    17샘플/4분 관측: -19,527~-20,118 ppm (±300), **급변·점프 없음**. clock_offset 은
    1784541169.03→1784541164.31 로 4.72s/238s 감소 = -19,830 ppm 으로 **drift_ppm 과 정확히 일치** →
    추정기가 실제 클럭차를 올바로 추종·보정 중 (자기정합 확인).
    원인: ECU tick 은 TIM5 10kHz free-run(×0.1ms)이고 rd_clock_sync.cpp `UnwrapTickSec` 의 1e-4 스케일은
    정상(스케일이 10배 틀렸다면 drift 가 ±90만 ppm 로 나왔을 것). 즉 **STM 타임베이스가 crystal 이 아니라
    RC 급(HSI ±1~2%)** 이라는 하드웨어 특성으로 판단. 시간동기 기능 자체는 정상 → V1 통과로 판정하되
    §5 에 의문 기록. **분석 시 raw ecu_tick 을 10kHz 정확으로 가정하면 안 되고 보정된 clock_offset 을 써야 함.**
  - bag: `data/rosbags/V1_commlatency_07-20_19-08/` (46.2MiB, 796.8s, 318,672건). data/rosbags 신규 생성(동기본에 없었음).

- **V2 INIT — BLOCKED (ECU 가 AUTO 진입 직후 FAULT, 원인 미규명)**
  - INIT 3단계 자체는 **전부 성공**: motor_mask addr192=0x0F 검증 OK(3/10회차, WRITE 2회 실패 후 성공) →
    auto_mode addr188=0x01 OK → mode(AUTO) addr190=0x01 OK → `INIT 완료` → `FSM: INIT -> IDLE`.
    즉 **레지스터만으로 AUTO 지시는 전달됨** (C-2 INIT 시퀀스 자체는 실기 동작 확인).
  - 그러나 직후 RW write 가 전부 `[Access Error]` → 50 tick 연속 거부 → 테스트베드 FSM **LOCKED** 래치.
    `rw_err=112(0x70)` = write 니블 **7(ACCESS)** — §4-C 표의 "ECU robot_state ≠ AUTO (mtr_lock)" 과 일치.
  - 실제 ECU 상태: **`sys_state = 5 = SYS_STATE_FAULT`** (`/carrier/ecu/fsm`, TestbedFeedback 동일).
    V0·V1 동안은 계속 1(MANUAL) 정상이었으므로 **AUTO 전환 시점에 FAULT 진입**한 것.
    FAULT 는 sticky — `rd_system.c:375` 가 EVALUATE_STATE 를 early return 시켜 리셋 전까지 복귀 불가.
  - **§4 플레이북으로 좁힌 결과 — 알려진 FAULT 경로가 전부 배제됨**:
    - 펌웨어 내 `robot_state = SYS_STATE_FAULT` 는 3곳뿐(rd_system.c:270 CAN fatal / 290 RS485 fatal /
      299 uart2reset+AUTO). **세 경로 모두 `hw.reset` 비트를 set** 하고, 그 비트는 rd_map_ecu.c:215 가
      `reg.sys.hw_reset` 로 미러 → Orin REBOOT 전까지 유지된다.
    - 측정: `hw_reset/hw_error/hw_fatal` **5채널 전부 false**, `degraded_cnt` **5채널 전부 0**,
      모터 comm_err 4개 전부 0, alive_time 단조증가(리부트 없음).
      FAULT 진입 시 checker 가 동결(263행)되므로 카운터는 진입 시점 값을 보존 → CAN/RS485 escalation 이었다면
      degraded_cnt 가 포화여야 하는데 0. ⇒ **CAN·RS485 fatal 경로 모두 배제.**
    - 구 `controlTask RD_PERIPHERAL_WRITE NOK → FAULT`(플래그 미설정 경로)는 2ccd39c(07-18)에서 제거됨.
      플래시 최신 여부 검증: proc_delta(228)은 e891f09(07-20)에서 추가된 것인데 V1 에서 **비영값** 관측
      ⇒ 07-20 커밋이 플래시돼 있음 ⇒ 07-18 의 제거도 포함 ⇒ **구펌웨어 가설도 배제.**
  - ⇒ **현재 소스 트리로 설명 불가능한 FAULT.** 정적 분석 한계 — 재현 실험 필요(전원 재투입 후
    sys_state 를 200Hz 로 관측해 전이 tick 과 그 순간의 플래그를 포착).
  - 유력 가설(미검증): **모터 무전원 상태에서 AUTO 진입** 자체가 트리거. 설계 주석은 ALL_READY 게이트가
    무전원을 무해화한다고 하지만, 그 게이트는 `motor_on`/TX 를 막을 뿐이고 mask 된 부재 모터에 대한
    상위 escalation 을 막는다는 보장은 코드상 확인되지 않았다. 사실이면 **지시서 §1-5 의
    "V0~V4 는 전부 무전원으로 진행 가능" 전제가 틀린 것**이 된다 (V2 부터 모터 전원 필요).
  - ⚠ 조치 필요: FAULT sticky → **ECU 전원 재투입(또는 리셋) 없이는 V2 이후 진행 불가.**

- **V2 근본원인 규명 — 격리 실험으로 확정 (위 "설명 불가" 기록을 정정)**
  - 방법: 평상(MANUAL) 노드를 계속 띄워 RS485 트래픽·진단을 살려 둔 채 **`addr190=1` 1바이트만** write.
    INIT 의 mask/auto_mode write 와 200Hz cmd write 를 전부 배제하고 AUTO 전환만 시험.
  - 결과 — 전이 경로 포착: **fsm 1(MANUAL) → 3(ESTOP_SW) → 5(FAULT)** (분포 27/8/91 샘플).
    동시에 평상 노드가 `ECU hw_reset 요청 (reg54=0x08): [can]` 을 출력 → **`hw.reset.bit.can`(bit3) 세트**.
  - ⇒ **FAULT 는 rd_system.c:268-271 의 CAN fatal 경로가 맞다** (최초 가설이 옳았고, 아래 이유로 오배제됐음).
  - **메커니즘**: mode write → 모드전환 안전장치(rd_system.c:413-414)가 **ESTOP_SW 강제 경유** →
    `ACTION_STATE_ESTOP_SW()` 가 `CAN_AK_ESTOP(BREAK_CURRENT_SW)` 로 **무조건 모터에 CAN 제동 프레임 송신** →
    모터 무전원이라 ACK 없음 → CAN 에러 누적 → `fatal_can1_cnt >= FATAL_MAX` → `hw.reset.bit.can=1` + FAULT.
    ALL_READY 게이트는 `motor_on`/정상 주행 TX 만 막고 **ESTOP 경로의 TX 는 막지 않는다** — 설계 주석의
    "무전원 무해화" 가 ESTOP 경유에는 성립하지 않는다.
  - ⇒ **무전원에서는 AUTO 진입이 구조적으로 불가능. 지시서 §1-5 "V0~V4 전부 무전원 진행 가능" 은 오류이며,
    V2 부터 모터(CAN) 전원이 필요하다.**

- **[버그] control/testbed 모드에서 ECU 진단 토픽이 stale — §4 플레이북을 무력화**
  - 증상: 동일한 ECU FAULT 상태인데 **control_mode 에서는** `hw_reset/hw_error/hw_fatal` 5채널 전부 false,
    `degraded_cnt` 전부 0, `/carrier/ecu/fsm` 이 0(INIT) 을 섞어 발행. **평상 모드에서는** 같은 상태가
    `reg54=0x08(can)` 으로 정확히 보고됨.
  - 원인: 이 토픽들은 전부 `ecu_reg = state_->ecu.reg` 256B 스냅샷(rd_bridge.cpp:324) 파생인데,
    control/testbed 모드의 배치 READ 는 sys 세그(addr16~31)를 갱신하지 않아 `ecu_reg.sys` 가 0/stale 로 남는다.
    (TestbedFeedback 은 별도 경로 `reg`(rd_bridge.cpp:516) 라 `sys_state=5` 를 정확히 보고 — 두 미러의 불일치.)
  - 영향: **§4-A/§4-B/§4-D 가 지시하는 진단 토픽이 정작 control_mode 디버깅에서 거짓 정상(0/false)을 준다.**
    본 세션에서 실제로 CAN 원인을 오배제하는 원인이 됐다.
  - 권고: control 모드 배치에 sys 세그를 포함시키거나, `ecu_reg` 미갱신 구간에서는 해당 토픽 발행을
    보류(또는 stale 표시)할 것. **판정용 권위 소스는 TestbedFeedback 의 `sys_state`.**
    (스펙 §2 Single Writer 등 원칙 변경이 아니라 발행 게이팅 문제라 수정 가능하나, 본 세션은 V2 블로커
     해소가 우선이고 §4 공통원칙(원인 확정 후 최소 수정)에 따라 사용자 판단 대기.)

---

## 2026-07-21 실기 검증 재개 (CAN 안정화 후 · 모터 2번만 연결·전원 ON)

전제 변화: 사용자가 CAN framing err(168MHz 승격 부작용)를 **SJW=3** 으로 해소, CAN 안정화.
STM 재플래시(motor_mask 기본 ALL). **모터 2번(ECU_AK[1]) 1개만 연결 + 전원 ON.** 지시서 §1-5 정정
(V2 부터 전원 필요)에 따라 모터 전원 인가 상태로 재개.

- **[핵심] V2 블로커 해소 — mask 를 실연결 모터에 일치시키면 무FAULT AUTO 진입 성공.**
  - `active_motors:="[2]"` (mask 0x02) 로 기동 → INIT 3단계 검증 OK(mask 0x02→auto_mode 0x01→mode AUTO)
    → **ecu/fsm=2(AUTO)**, TestbedFeedback **sys_state=2**, testbed_state=1(IDLE), rw_err=0, FAULT/ESTOP 무.
    fb_position[1]=-80.3 (모터 2 CAN 피드백 생존 확인).
  - 원리: 07-20 규명대로 mode write 는 ESTOP_SW 를 강제 경유하고 `CAN_AK_ESTOP` 이 mask 된 모터에 제동
    프레임 TX 한다. 단, TX 는 `RD_PERIPHERAL_WRITE(…, motor_mask)`→`RD_CAN_MOTOR_TRANSMIT` 이
    **masked-out 모터를 skip**(rd_can_motor.c:100). 따라서 **mask={실제 연결 모터}이면 제동 TX 가 그
    모터에서 ACK 받아 CAN fatal 이 안 난다.** ⇒ 07-20 의 FAULT 는 mask=0x0F 인데 모터 1/3/4 가
    부재했기 때문 (부재 모터로 제동 TX → ACK 실패 → CAN OFFLINE → checker NOK → FAULT).
  - ⇒ **무전원 진행 불가는 여전히 맞지만, "전원+mask 일치"면 단일 모터로도 V2 통과.** 지시서 V2 의
    `active_motors:="[1,2,3,4]"` 는 4모터 전제 — 실연결 구성에 맞춰 mask 를 줄여야 한다.

- **[버그·재현] auto_mode DIRECT(2) 전환 시 `128:52` RW write 가 RD_FATAL → 브리지 슈퍼바이저 재시작,
  2회차엔 SEGFAULT 로 노드 크래시.** (2/2 재현)
  - 경로: `config auto_mode 2` → DoSetAutoMode DIRECT(shadow 소독+auto_mode write 성공) → 다음 200Hz
    tick 이 `task_control_direct_`(128:52) 로 RW → `ExecuteTask` 가 **RD_FATAL** 반환 →
    rd_schedule.cpp:383 "Hardware Disconnected! Returning to Supervisor." → comm 재초기화·재INIT
    (auto_mode 1 로 리셋). 2회차 재현 시 재INIT 직후 **Segmentation fault (exit 245)** 로 노드 사망.
  - 이 DIRECT(128:52) write 는 **실기 최초 실행**(V2 블로킹으로 그간 도달 못 함). 브리지측 버퍼는
    여유(RX/TX 512, MAX_PACKET 256, RW≈57R+52W+프레이밍<256)라 오버플로 아님 → ECU 가 52B write 에
    무응답(시리얼 read timeout)이 유력. SEGFAULT 는 슈퍼바이저 재시작이 in-flight config 서비스 콜과
    경합하는 수명(lifetime) 버그로 추정(별건).
  - 영향: **트랙션 실험은 CURRENT(164:16) 경로만 사용 → 크리티컬 패스 아님.** DIRECT 는 옵션 경로.
  - 조치: 본 세션에서 미수정(ECU 펌웨어는 이 노트북서 빌드/플래시 불가; SEGFAULT 수정은 경합 분석 필요).
    ⇒ **STM 담당: ECU RS485 RX 가 52B RW write 를 수용하는지 확인. 브리지 담당: 슈퍼바이저 재시작
    경로의 config 서비스 수명 경합(SEGFAULT) 수정.**

- **[발견] 단일 모터 벤치에서 out-of-span `config motors` 로 부재 모터를 mask 에 추가하면 ESTOP_SW→LOCKED.**
  - `config motors 2 3`(mask 0x06, 모터 3 부재) → 즉시 **testbed LOCKED**(rw_err=0x70 ACCESS, sys_state=3 ESTOP_SW).
  - 원인(정상 FW 동작): AUTO/IDLE 에서 모터 2가 ready+RS485 running 이라 이미 `motor_on=1`. mask 에
    부재 모터 3 추가 → `RD_CAN_MOTOR_ALL_READY(0x06)` false 인데 motor_on=1 → rd_system.c:393-394 의
    **"주행 중 통신 상실"(H1) → motor_fault=1 → ESTOP_SW** → mtr_lock → RW 연속 거부 50tick → LOCKED.
  - 회복: LOCKED 는 REARM 만 허용하나 mask 0x06 그대로면 REARM→IDLE→즉시 재LOCKED(데드락).
    **노드 재시작(INIT 재실행 → mask 0x02 재기록)** 으로만 탈출 (INIT write 는 CMD_SYSTEM 영역이라
    mtr_lock 무관하게 통과). §4 공통원칙 "재부팅 후 노드 재시작" 과 동일.
  - ⇒ 지시서 V3 out-of-span 예시(`config motors 2 3`, read-back 0x06)는 **4모터 전제**. 단일 모터
    벤치에선 부재 모터를 mask 에 넣지 말 것(=addr192 write 테스트는 `config motors 2` 로 대체 수행함).

- **V4 무전원 드라이런 대체 — 모터 2 전원 ON 상태라 0A 프로파일로 액션/FSM/피드백 플러밍만 검증(무토크).**
  - `hold 0A 3s + hold 0A 2s`(m2) run → goal 수락→RUNNING→완주→**success**, ticks_executed=**1000**
    (=5s×200 정확), write_err=0, clamp=0. 재생 중 goal_id=1(≠0)·testbed_state=2(RUNNING)·profile_time
    0→4.925 단조증가·rw_err=0(1454/1454). 완주 후 IDLE 복귀. ⇒ **C-7 액션/프로파일 경로 실기 통과**
    (실전류 램프·fb_current 비영 확인은 V5 로 이월 — 모터 전원 인가 상태라 램프=실구동=§2 안전 게이트 대상).
  - V3 RUNNING 중 config 거부 ✓("RUNNING 중 설정 변경 불가, 중단은 action cancel").

- **V5 실구동 PASS (사용자 승인·입회, 모터2 실전류 구동).**
  - ① 3A hold(2s 램프→8s hold): **fb_current[m2] ≈ cmd** (cmd 3.0 → fb 2.98~3.03), 램프 추종·실회전 확인.
    read-back 비영값 = C-7 "V5 에서만 비영 확인" 충족. 완주 후 cmd·fb 모두 0.
  - ② `--record --name V5_first` 소전류 램프(hold0 3s→ramp0→6A 8s→hold6A 3s→ramp6→0A 8s, 22s):
    goal_id=4·**4400 tick**·write_err=0·clamp=0. **§5.2 폴더 규격 충족**:
    `data/rosbags/V5_first_07-21_15-37/` = bag(feedback+comm_latency 각 4594msg / 23.04s ≈ **199.4Hz** 실데이터)
    + result.json(clamp_cnt/finished_at/goal_id/message/name/profile_source/started_at/**success=true**/
    ticks_executed=4400/write_err_cnt=0 전수) + profile.yaml 사본. (bag 는 repo-root data/rosbags 로 이동 —
    CLI 기본 bag-dir 가 CWD 상대라 orin_ws 서 실행 시 orin_ws/data 로 감. V1 과 동일 위치로 정렬.)
  - ③ abort 실기: 4A 구동(fb 3.99) 중 `testbed_cli abort` → **즉시 fb_current=0** → IDLE,
    result "canceled"(goal_id=5, 1320 tick).
  - ⚠ 로드셀 raw 응답이 3A 에서 Δ~27~29 counts 로 작음 — 트랙 픽스처 체결/로드셀 영점(Stage1) 점검 필요.
    (V5 판정 자체와 무관 — fb_current 추종이 판정 기준.)
  - CLI 인자: 지시서 `--label` 아님 **`--name`** (record 라벨). 지시서 §5.1 예시 갱신 필요.
- **검증 요약(07-21) 최종:** V0 재확인 PASS · V1 PASS(07-20) · **V2 PASS(블로커 해소)** · V3 부분통과
  (status/out-of-span/in-span거부/RUNNING거부 ✓, **DIRECT 전환 FAIL**) · V4 0A 플러밍 PASS ·
  **V5 실구동+기록+abort PASS**. ⇒ **V2 블로커 해소 + C-7 실기 완료.** ⚠ STM 코드 수정분은
  STM32CubeIDE 빌드 검증 필요(이 노트북 빌드 불가). 이월: DIRECT 크래시(브리지 SEGFAULT/ECU 52B RW),
  control 모드 진단토픽 stale(07-20), 로드셀 신호 크기 점검(Stage1).


## 2026-07-21 (노트북/설계) 실기 보고 검토 + #9 분석 파이프라인 + 문서 정합화

실기 세션의 V0~V6 보고(HANDOFF_VERIFY §5)를 설계 소유자로서 검토·확정. STM 코드는 건드리지 않음(빌드 불가).

- **#9 `analysis/latency/latency_analysis.py` 신규 작성 + V1 bag 실행 검증** (ROS 비의존, sqlite3 + 수동 CDR):
  - CommLatency.msg 디코더(정렬 규칙 기반, frame_id 길이 무관) → RTT/구간분해/cmd_delta/offset·drift/루프건전성.
  - V1(`data/rosbags/V1_commlatency_07-20_19-08`, 159,336샘플/796.8s) 실측: **200.00Hz**, over-period(>7.5ms)
    0.03%(48건), RTT mean 1.96ms·p99 2.30·jitter 0.13ms. 구간분해 wire_up 0.30 + wire_down 0.79 +
    proc_delta 0.05 + **USB잔차 0.82** ≈ RTT 1.96ms(잔차 고립 성공). 07-20 보고 수치와 정합.
  - **drift 확정**: 추정기 -19,579ppm vs 독립 회귀 교차검증(`t_resp=a·ecu_tick+b`) -19,211ppm 일치 → 실재하는
    **-1.96%** HSI 특성(추정기 정상). 산출물: summary.json + rtt_breakdown/clock_drift/loop_health png.
  - 플롯 라벨은 ASCII(개발머신 폰트 불확실 대비), 리포트·JSON·주석은 한글.
- **drift 원인 규명 (실기가 STM 에 넘긴 의문 해소)**: `main.c` RCC `PLLSource=HSI` 확인 → HSI 내부 RC(±1~2%)가
  -1.96% drift 의 근거. 하드웨어 특성 확정, 소프트웨어 보정(clock_offset)으로 충분. testbed_spec §2.5 반영.
- **문서 정합화**: testbed_spec §3.1(active_motors=실연결 마스크 불변식, V2 블로커)·§2.5(drift 실측 %스케일)·
  §5.1(CLI 실측 정수코드 인자 + `--json`/단어형/`--timeout` 미구현 명시). HANDOFF_VERIFY §1-5(무전원 전제
  오류 정정)·§3-V1(drift 기대치)·V2 예시(active_motors [1,2,3,4]→[2]). HANDOFF_260719 §6.4(회신·판단 4건).
- **설계 판단 4건 확정**(HANDOFF §6.4): ① ESTOP TX ALL_READY 게이팅 → 현행 유지(마스크 불변식이 정답)
  ② TIM5=HSI 확정 ③ control 배치 READ 에 sys 세그 포함 → 승인하되 #11/다음 브리지 수정 때 ④ #9 완료.
- **다음**: test_plan Stage 1(로드셀 영점 — V5 Δ~27counts 게이트). AI 자동화 착수 전 `testbed_cli --json` 선행.

## 2026-07-21 (노트북/설계) #9b traction 디코더 + Stage 1 실험 셋업

실기 세션이 실험 세팅(무게추 3개: 10.46/11.03/10.25kg) 하는 동안 분석·프로파일·러너 셋업.

- **#9b `analysis/traction/traction_analysis.py` 신포맷 대응**: `_decode_testbed()`(TestbedFeedback.msg,
  tick ×0.1ms, loadcell int32, goal_id) + `_detect_format()`(topics 테이블로 구/신 자동판별, feedback 토픽만
  선별) + `split_by_goal()`(goal_id>0 연속구간 실험 자동분할). 구 `_decode`/구 bag 경로 로직 불변(회귀 안전).
  V5 bag 검증: dur 23.48s(1e-4 스케일 정확)·goal_ids[0,4]·cmd0-6/fb0-6·lc 1015-1042·램프1·분할1. 전 파이프라인
  end-to-end 완주(신포맷 소비 입증). 구 TEST2 bag 은 sync 사본에 없어 로직불변으로 갈음.
- **`analysis/traction/loadcell_calib.py` 신규** (Stage 1): `--zero`(무부하 baseline/노이즈/레일마진/드리프트)
  + `--calib --loads`(안정 plateau 자동검출 → cnt↔N 선형피팅, 채널별). V5 검증: **로드셀 영점 이미 충분**
  (ch0 1028/ch1 1034cnt, 레일마진~1007, 드리프트~0) → Stage 1 핵심은 영점 아니라 cnt↔N 캘리브레이션.
- **`analysis/traction/run_campaign.py` 신규**: testbed_cli 배치 러너(반복 N회 + 재장착 프롬프트). 종료코드
  +result.json 으로 판정, 이상 시 즉시 중단(§3 안전경계). session_summary.json 산출. (dev머신 실기 검증 대상.)
- **`data/profiles/` 4종 신규**: loadcell_zero(5분 0A) / std_ramp_cycle(0-15A rise-hold-fall) /
  deadband_stair(4-12A 0.5A계단 왕복) / step_probe(8-12-18A 스텝). 파서(rd_profile.cpp) 파라미터명 대조 확인
  (hold/ramp/stair/step ✓). 단일 모터 m2, 안전상한 반영(램프천장은 첫 실기 후 상향 파라미터).
- **`test_plan_stage1_260721.md` 신규**: 세션 A(Stage1 캘리브레이션, 모터 불필요: 영점재확인→cnt↔N 누적적재
  →캘리브 반복성) + 세션 B(모터: 첫구동→반복성 게이트 5+2회 CV<5%→데드밴드/스텝→payload 스윕). 무게추 스택
  0/10.46/21.49/31.74kg=0/102.6/210.8/311.3N. ⚠ loading 방향(수평 견인 vs 수직 payload) 세션 개시 전 확정.
- **다음**: Stage1 cnt↔N 확정 후 traction_analysis 에 raw→[N] 상수 반영. AI 자동화 편의로 testbed_cli --json.
- **loading 방향 확정(사용자)**: 무게추 = **캐리어 수직 payload(법선 하중)**. ⇒ 무게추로 로드셀 cnt↔N
  캘리브레이션 불가(수평 기지력 필요) → 절대 [N] 은 스프링저울 확보 후로 지연. **반복성 게이트는 slope CV
  스케일 불변이라 raw cnt 로 지금 진행**. payload 스윕(0/10.46/21.49/31.74kg)이 본 매핑 실험. test_plan_stage1 갱신.
- **로드셀 캘리 방식 확정(사용자)**: 도르래/수평 인장 스킵. **중력-저울 방식**(무게추를 로드셀 감지축에
  중력방향 적재, F=m·g)으로 지금 cnt→N 캘리, 결과를 [N] 으로. 수평 재캘리는 정밀 셋업 때. 배선:
  `loadcell_calib.py --calib` 성공 시 `analysis/traction/loadcell_cal.json`(정준) 기록 →
  `traction_analysis.py` 가 읽어 로드셀 자동 [N] 환산(단위·플롯·BASELINE_WARN 임계 전부 N, 파일 없으면
  cnt 하위호환). 합성 상수(0.12N/cnt)로 cnt↔N 전환·전 파이프라인 N모드 무크래시 검증. test_plan_stage1 §0·§1 갱신.

## 2026-07-21 (실기·노트북 통합 세션) Stage 1 로드셀 cnt→N 캘리브레이션 실기 완료

**이 세션이 하드웨어에 직접 접근**(ROS Humble + /dev/ttyUSB0 + 빌드된 orin_ws)해 캘리 실행·분석까지 수행.
브리지는 `traction_test_mode`(READ 전용·무모터, rd_bridge.cpp:86/509 확인)로 기동 — 안전.

- **절차**: 무게추를 로드셀(ch0)에 중력방향 누적 적재하며 feedback bag 기록
  (`data/rosbags/s1_cal_07-21_17-24/`, 98,316 msg). 사용자 페이스 핸드셰이크로 단계별 적재.
- **실측 (ch0)**: 0kg→212cnt / 10.46(102.6N)→1505 / 21.49(210.8N)→2867 / 31.74(311.3N)→3236.
- **캘리 결과**: **선형역 3점(0/10.46/21.49) 피팅 R²=1.00000, 잔차 <1cnt → k=12.601 cnt/N,
  N_per_count=0.07936 N/cnt**. `analysis/traction/loadcell_cal.json`(정준) 기록 →
  traction_analysis.py 가 이제 로드셀을 자동 [N] 출력(BASELINE_WARN=3.97N 등).
- **[발견] 고하중 롤오프**: 31.74kg(311N) 점이 선형 예상(4134cnt) 대비 3236cnt로 **~22% 부족**
  (마지막 구간 감도 12.6→3.7 cnt/N 급락, 안정값·크리핑 아님). 원인 후보: 로드셀 정격 근접 비선형 OR
  중력-캘리 셋업의 고하중 적재 쏠림/바인딩(수평 견인에선 다를 수 있음). **신뢰 힘범위 ≤~211N 로 제한**,
  수평 재캘리 또는 고하중 물리 점검 대상. std_ramp_cycle 15A 는 데드밴드(~8A) 제하면 견인력이 이 범위 내 예상.
- **채널**: 사용자 확정 ch0 사용(ch1 백업 기록만). 시스템 종료·유휴 확인.
- **다음(세션 B)**: 모터 전원+픽스처 체결+입회 하에 반복성 게이트(std_ramp_cycle 5+2, CV<5%) → payload 스윕.

## 2026-07-21 Stage 1 정밀 재캘리 (2 cycle, up-down) — 통합 결과

작은 무게추(1.53×2, 1.87×2) + 10.46/11.03 로 0~28kg up-down 2사이클(사용자 페이스, 각 단계 ≥5초).
브리지는 traction_test_mode(READ 전용). 각 plateau std ~1.2cnt (매우 안정).

- **통합 캘리 (21점, 0~229N)**: **k=12.6298 cnt/N, N_per_count=0.07918 N/cnt, offset=213.5, R²=0.999992**,
  잔차 std 2.59cnt(0.205N). loadcell_cal.json 갱신 → traction_analysis 자동 [N].
- **반복성**: 10.46kg 3회·21.49kg 3회(2사이클 교차) 측정, 최대 범위 3.3cnt=**0.26N**. 히스테리시스 최대 **0.17N**.
- **[핵심] 하드 포화 확정**: ch0 는 **~3234cnt 에서 포화**, **~247N(25kg)** 도달 시. 25/26.76/28.29kg 이 모두
  ~3234 로 병합(사이클1 top plateau 111s), 지난 31.74kg=3236 도 동일 → 젠틀 롤오프 아닌 하드 천장.
  **선형 유효 0~229N. 견인력 실험은 ≤229N(약 19A@20.7N/A·데드밴드8A) 유지 필수** — 초과 시 로드셀 무응답.
- 산출: analysis/traction/calib_out/{linearity_c1,calib_combined}.png. bag: s1_calwide_c1/c2_07-21.

## 2026-07-21 Stage 1 저범위 보강 (사이클 3, 1세트×3반복) — 최종 캘리 확정

작은 추(1.53×2,1.87×2)로 0~6.8kg up-down, 사용자가 재적재 위치 변동 우려로 **1세트를 3회 반복**(bag s1_callow_c3).
- **최종 통합 (c1+c2+c3, 46점 0~229N)**: **k=12.6270 cnt/N, N_per_count=0.07920 N/cnt, offset=214.6,
  R²=0.999988, 잔차 0.24N**. loadcell_cal.json 확정.
- **위치 반복성 (재적재 편차)**: 하중따라 증가 — 0N 0.08N → 67N **0.89N**(작은추 5단 스택 최상단). 반면 큰추
  구간(102~229N)은 0.02~0.26N 로 훨씬 안정. ⇒ 작은추 톨 스택의 off-center 가 주원인(사용자 직관 검증). 캘리
  절차상 편차이지 센서 결함 아님. 실험 견인력은 자연 힘이라 무관.
- **경미한 오목성**: 저범위(≤67N) 국소 k=12.73 vs 전역 12.63 (~0.8%). 단일 직선 잔차 0.24N 로 실용상 무시.
- 산출: analysis/traction/calib_out/calib_final.png. 시스템 종료·유휴.

## 2026-07-21 문서 전면 정리 (HANDOFF 세대교체)

실기 검증·Stage 0/1 완료를 반영해 지시서 체계를 정리. 스펙 문서들은 절대 지시서로서 현행화.

- **삭제**: HANDOFF_260719 / HANDOFF_VERIFY_260719 / SESSION_HANDOFF_260719 — 임무 완료(왕복 이력은 git +
  이 로그에 보존). SESSION_HANDOFF 의 "실기 실행 불가" 전제는 이 세션에서 무효화됨(ROS+ttyUSB0 직접 접근).
- **신규 [HANDOFF_260721.md](HANDOFF_260721.md)**: 역할 승계(설계 소유자+실험 실행자) + 상태 스냅샷 +
  하드 제약 5건(전류≤19A/mask 일치/DIRECT 금지/clock_offset 보정/CLI 미구현 의존 금지) + 다음 작업
  (①Stage 2 반복성 게이트 ②#12 CLI --json → #11 리팩터링 → #10 웹 ③STM SJW 커밋·수평 재캘리).
- **testbed_spec.md**: §6 진행표 전면 갱신(#0~#9 ✅ 상태·실기 근거 표기, #10/#11/#12 잔여, 이월 버그 3건 명시).
- **test_plan_traction.md**: 헤더에 하드 제약 3건 승격 / Stage 0·1 완료 실측 기록 / Stage 2 조건을 현 벤치
  (m2, payload 스택)로 갱신 / §2 프로파일 표를 data/profiles/ 실물 기준(**25A→15A 하향, 절대 19A**) /
  §3 안전경계·§4 체크리스트 현행화.
- **TASKS.md**: §2 전면 갱신(인프라 ✅·Stage 진행·잔여 TODO), §1.3 토픽 표 신 인터페이스로 교체
  (cmd_current 폐기 완료, testbed/feedback·comm_latency·config·run_profile), 진단 stale 경고 부기.
- **_index.md / CLAUDE.md**: 파일 지도·핵심 문서·환경 전제(실기 실행 가능) 현행화.

## 2026-07-21 (세션) — TEST3 Stage 2 실기 실행 + 40kg 자율 배터리 셋업

- **실기 브링업**: 브리지 control_mode + `active_motors:="[2]"` 기동 → INIT(mask→auto_mode1→AUTO)→IDLE.
  ⚠ **latency_timer=1 필수** — 16ms 기본값이면 FTDI 버퍼 지연으로 RW read-back 이 트랜잭션 타임아웃 안에
  안 들어와 INIT motor_mask WRITE 10/10 실패→FATAL. 1 로 바꾸자 즉시 INIT 통과. (build.sh 3행에 설정 있음.)
- **B-1 firstlight_3a (신규 프로파일)**: 3A 저전류 first-light. 명령경로·200Hz·write_err0·baseline복귀 ✅.
  단 3A<데드밴드(~6.5A)라 견인력 무반응(0.04N) — 결함 아님, 예상.
- **B-1b std_ramp_cycle 15A(payload 미상)**: 피크 209N(tared), rise slope **23.9 N/A**(이론밴드 18–25 안),
  deadband ~6.5A. **큰 히스테리시스**(fall 10.3 N/A) + 종료직후 잔류 20N→15~20s 이완 후 ~4.5N(캡스턴/스틱션).
- **b2_pre_w40 안전게이트(40kg,15A)**: 절대 캘리 피크 **214.6N** → 선형상한 229N 여유 14.5N(얇음).
  → **Stage 2 배터리는 14A 로 확정**(피크~190N, 여유~40N). 매핑은 rise 곡선 기준(하강은 스틱션 오염).
- **신규 프로파일 8종** `data/profiles/camp_*.yaml`: ramp_slow/med/fast · deadband_stair · step · sine_lo ·
  chirp · prbs(seed42). 전부 14A cap. `analysis/traction/lint_profiles.py`(파서 규칙 미러 린터)로 전량 통과.
- **신규 오케스트레이터** `analysis/traction/run_battery.py`(+.sh): 40kg 단일 세팅 무인 배터리(12런, ~20분).
  프리플라이트 IDLE / result.json 검증 / 런 사이 settle 20s + 상태 재확인(비-IDLE=ESTOP/FAULT 시 자동 중단).
  세션 요약 `data/rosbags/battery_session_*.json`. 계획 문서: [test_plan_stage2_campaign_260721.md](test_plan_stage2_campaign_260721.md).
- ⚠ 분석 미수행(설계 방침: 전량 수집 후 별도 세션 일괄). traction_analysis.py 는 램프 중심 —
  sine/chirp/prbs/stair 전용 분석기는 분석 세션에서 확장 필요.

## 2026-07-21 (세션) — 히스테리시스 특성화 배터리 (Stage 2b) 셋업

- **camp_* 배터리 12런(40kg/14A) 완주** — write_err/clamp 0, 이상 없음. 세션 `battery_session_07-21_21-05.json`.
- **히스테리시스 계통 특성화** 설계: 캡스턴/스틱션 이력을 FORC·하강속도·마이너루프·작동점밴드·커스텀변곡·
  중첩반전 6패밀리로 훑음. 신규 프로파일 17종 `data/profiles/hys_*.yaml`(생성기 scratchpad/gen_hys.py):
  - A(forc_rev14/12/10/8) 반전점, C(desc_slow/med/fast 0.3/0.6/1.2A/s) 하강속도 = **핵심 ×2**.
  - B(minor_f10/07/04) D(band_hi/mid/lo) E(glide1/2) F(nest1/2) = ×1.
  - 표준 램프 **1.0 A/s**(0.375→상향, 시간 단축; 준정적성은 C가 판별). sine 정수주기+ramp 브릿지로
    세그경계 연속(급단차 ≤0.01A). 전 프로파일 lint 통과([0,14]A, clamp0).
- **오케스트레이터 매니페스트 구동으로 리팩터링**: `run_battery.py <manifest.json>` + `run_battery.sh <manifest>`.
  블록 쿨다운(8런마다 90s, 모터 열 완화 — control 모드는 온도 토픽 미발행이라 SW 가드 불가, 시간기반+물리감시 대체).
  매니페스트 `hysteresis_w40_manifest.json`: 24런, settle15s, ~30분(핵심 A/C×2, 나머지×1).
- **린터 확장** `lint_profiles.py`: 세그먼트 경계 급단차(모터 저크) 검사 추가.
- 계획: [test_plan_hysteresis_260721.md](test_plan_hysteresis_260721.md). ⚠ 이력 전용 분석기(FORC/루프폭/기억)는
  분석 세션에서 확장 필요(traction_analysis.py 는 램프 중심).

## 2026-07-22 (세션) — 히스테리시스 배터리 3세트 실기 완주 + 분석 인계

- **AUTO 재브링업**: ECU 재부팅 후 latency_timer=1 유지 확인 → 브리지 재기동 → INIT 통과(mask/auto_mode/mode
  검증 OK) → IDLE. (foreground sleep 이 환경에서 차단되는 특성 재확인 — 폴링은 python 루프로 대체.)
- **hysteresis_w40 배터리 3세트 연속 실행**(문답 없이, Sonnet): 세트당 24런 ×3 = **72런**, 전량
  write_err=0/clamp=0/ESTOP·FAULT 없음. 세션 `battery_hysteresis_w40_07-22_{12-56,13-38,14-37}.json`.
  세트1(camp_*) 12런 포함 **누적 84런** 무결 수집.
- **분석 인계 문서 신규** [analysis/result/HANDOFF_analysis_260722.md](analysis/result/HANDOFF_analysis_260722.md):
  수집물 지도·캘리브레이션 주의(오프셋 미차감 함정)·6개 패밀리 분석 질문·분석기 갭(stair/sine/chirp/prbs/FORC
  미지원, traction_analysis.py 는 램프 중심) · 다음 액션(Fable 세션이 분석+재계획 문서 작성, 실행은 보류).
- 물리 실행 단계 종료 — 다음은 Fable 모델 분석 세션.

## 2026-07-22 (세션, Fable) — 히스테리시스 분석 완료: Preisach 회귀 모델 확정

- **분석 스크립트 신규** `analysis/traction/hys_{cache,analysis1,analysis2,model,crosscheck}.py`:
  84런 npz 캐시 → 반복성/율의존성/FORC → 마이너루프/밴드/중첩 → 이산 Preisach(NNLS)+PI 비교 → 교차검증.
- **핵심 결론**: current→traction 은 율무관 이력계(하강 0.3~1.2A/s 완전중첩). **İ 항 불필요.**
  이산 Preisach(릴레이 231, NNLS 선형회귀) 훈련 3.5N / **미학습 파형 검증 4.9~6.3N (~3% FS)** —
  **러닝 불필요, 수학적 회귀로 충분**. 고전 PI 는 루프폭의 작동점 비례(1.5+1.03·Ic N) 못 잡아 기각(2.4배 오차).
  반환점 기억 <1% FS, wiping-out 성립, 스텝 τ63≈0.15s, 크리프 1~4N/8s, 상승기울기만 약한 rate 의존(~10%/5배).
- **산출물**: `analysis/result/hysteresis_analysis_260722.md`(결과 문서) + hys1/2/3 png 3장 +
  `hys_main_curves_w40.csv`(주곡선 수치) + `hys_cache/preisach_model.npz`(모델 가중치).
- **한계**: PRBS(0.5s 비트) 예측 실패(지연 보정 후에도 17N) — 빠른 명령 제어에는 동특성 ID(P2) 필요.
  payload 40kg 단일점 — F(I,W) 는 P1 스윕 필요. 다음 실험 제안 5건은 결과 문서 §다음 실험 참조.
- test_index.csv 84행 추가는 보류 — hys 런은 램프 전용 traction_analysis 파이프라인에 안 맞음,
  세션 JSON(battery_*.json) + hys_cache/index.json 이 해당 캠페인의 인덱스 역할.
- **사용자 결정(2026-07-22)**: payload 는 이론상 변수 아님 → 스윕 제외(무게추=캘리용). 제어 대역 미정 →
  P2 보류. 차기 우선순위 = **V1 제어형 검증**(slew≤1A/s 장기 궤적, 예측오차 <5N 목표) → P3 크리프 → P4 기준런.
  결과 문서 §다음 실험 갱신됨.

## 2026-07-23 (세션, Fable) — Stage 3 설계: 트랙 위치 외란 → 제어 강건성 가이드

- **목표 전환**(사용자): 고무 트랙의 parking set·발열·온도 민감성 때문에 트랙 접촉 지점은 변수가 아닌
  **외란**. F 는 I 와 Time(크리프/열/일간 변화)에 의존 → 분산 성분 분해로 **제어기 강건성 요구를 정량화**.
- **계획 문서 신규** [test_plan_robust_260723.md](test_plan_robust_260723.md): 30분 복합 세션
  = 5 트랙위치 블록 × [rb_forc14(기준 이력) + rb_creepminor(P3 크리프+마이너루프) + rb_ctrl_s<k>(V1
  slew≤1A/s 무작위 궤적, seed 블록고정)]. 다일 반복 + 라틴방진 순서(열-위치 교락 해소), parking 메타데이터.
- 구현 사양 포함(Opus 작업 대기): 프로파일 생성기 gen_robust.py / run_battery.py `--suffix`(위치 기록)·
  `--note`(정차이력) / 블록 매니페스트 5개 / rb_analysis.py 분산분해 + robust_guide 산출.
- 기존 84런은 위치 무통제 기준일 데이터로 재분류.
- **크리프 승격(2026-07-23 사용자)**: 크리프를 외란→**모델 변수**로. 확장 모델
  F=Preisach[I]+Σz_j, ż_j=(−z_j+b_j·ΔH)/τ_j, τ={0.5,2,8,30}s — b_j 는 hold 선형회귀(러닝 불필요 유지).
  계획 갱신: rb_creepminor → **rb_creepA/B 격일 교대**(일내 동일→분산분해 성립, 격일→레벨 커버리지:
  A={rise12,fall6,relax0} B={rise8,fall10,relax0}, hold30s+0A이완25s), ctrl hold 1~8s 로 확장(크리프
  in-context 검증), 분석에 크리프 동정 절 추가 → 최종 산출 preisach_creep_model.npz.

## 2026-07-24 (세션) — orin_ws 구 rosbag 데이터 삭제

- **삭제(사용자 확인 후, 152MB)**: `orin_ws/TEST0_*` `TEST1_*` `TEST2_*`(15개, 07-07~07-10 정적 캠페인)
  + `orin_ws/Loadcell_07-10_19-22`. 전부 `.gitignore`(`orin_ws/TEST*_*/`) 대상이라 git 미추적 —
  **복구 불가 영구 삭제**. `test_index.csv` 가 `../../data/rosbags/TEST2_*` 로 참조했으나 실제 파일은
  `data/rosbags/` 로 이관된 적 없이 orin_ws 루트에만 존재했음(이미 죽은 참조 상태였음).
  → `test_index.csv` 를 헤더만 남기고 정리(해당 15행 제거).
- **오삭제 복구**: `orin_ws/rosbag_test.sh`(bag record 헬퍼 스크립트, 데이터 아님)가 `rm -rf TEST*` 패턴에
  같이 걸려 삭제됐다가 git(`b9fd1a4`)에서 즉시 복원함 — 실질 영향 없음.
