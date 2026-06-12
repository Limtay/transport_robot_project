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
				-  해당 ID 3초간 접근 X ,  보드 상태 off 로 띄우기
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
			-  ECU | WRITE | 189 -> value : 0
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
			-  ECU | WRITE | 189 -> value : 1
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
- **`rd_map_ecu.c` unlock 버그 수정**: `check_region_range()` 의 `!r->needs_unlock` 조건이 반전되어 있었음 — 기존엔 unlock 불필요 영역이 LOCK에 막히고 CMD 영역(128~191)은 무조건 쓰기 허용되던 상태. `r->needs_unlock && LOCK → ACCESS 거부` 로 수정
- **SYS_WRITE_MODE 분리**: region LUT 에서 addr0 을 단독 영역(항상 R/W)으로 분리, DEFINE 1~15 는 needs_unlock=1 로 변경 → **이제 CMD 영역 및 DEFINE(1~15) 쓰기는 UNLOCK 선행 필수** (Orin 쪽 자동 unlock 으로 대응)
- **`rd_system.c` addr5 처리 신규**: `RD_SYSTEM_HW_RESET_HANDLE()` — systemTask(100Hz) 에서 reg.def.hw_reset(addr5) 비트 검사 → 채널별 RECOVERY 호출(uart1/uart2/uart6/can/i2c) + fatal 카운터 리셋 → addr54(hw.reset)/addr5 플래그 동시 클리어

### 미확정 / TODO (다음 작업)
- [ ] PCU READ 레지스터 확정 (SOC, SOH 등) → `task_10hz_pcu_` 주소 갱신 + `enable_pcu_read_` 활성화
- [ ] DPC READ 레지스터 확정 (전개 FSM state) → `task_10hz_dpc_` + jeongae 시퀀스 DPC 단계 구현
- [ ] jeongae 시퀀스: DPC 전개/회수 요청 레지스터, 공벽(1)(2)/전개판(3) 구분, deploy camera Action, 4과제 정책
- [ ] 각 시퀀스 state 실패 시 정책 (현재: ESTOP 해제 시도 후 jeongae lock 걸고 중단)
- [ ] soft ESTOP 용 addr189(ctr_flag) 의 ECU 측 의미 확인 — 문서상 189=0 요청이나 현재 ECU 코드에서 189는 direct/kinematics 선택 플래그
- [ ] dpc_led / pcu_relay / gongbyeok 매크로 (레지스터 미정)
- [ ] STM 빌드: CubeIDE 에서 컴파일 + 실기 검증 (이 노트북엔 arm 툴체인 없음)

### 참고 (동작 변경)
- 에러 토픽 채널명 `uart4` → `uart6` 변경 (STM idx2 가 IMU/uart6 인 것과 정렬): `/carrier/ecu/error/*/uart6`
- 기존 스케줄의 slot9 커스텀 커맨드(`PushCustomCommand`)는 커맨드 슬롯 4개 체계로 대체·제거
- CLI 실행: `ros2 run orin_firmware_bridge command_cli` (노트북에서 실행 시 같은 ROS_DOMAIN_ID 필요)

## 2026-06-11 — 2차 수정 (사용자 피드백 반영)

### 1. 잠금 범위 축소: DEFINE 영역만 잠금
- STM `rd_map_ecu.c`: CMD_MOTOR(128~179)·CMD_SYSTEM(180~191) needs_unlock → **0** (unlock 불필요).
  잠금은 **DEFINE 1~15 만** 유지, addr0(sys_write_mode)은 항상 R/W (unlock 키)
- Orin 스케줄러의 자동 unlock(Init 시 전송 + 50Hz 실패 시 재전송) **제거** — cmd_vel 쓰기에 더 이상 불필요
- DEFINE 쓰기가 필요한 매크로 대응:
	- CLI `macro unlock <on|off>` 신규 (reg0 write)
	- CLI `macro hw_reset <ch>` 는 unlock(reg0=1) → reg5 write → lock(reg0=0) 3단계 자동 체인 (0.5s 간격)

### 2. addr189(구 ctr_flag) → Orin 용 soft ESTOP 재정의
- `ctr_flag` 는 코드상 사용처가 주석 처리되어 있던 미사용 필드 → **`soft_estop`** 으로 개명
	- 값: **0 = ESTOP 작동** (AUTO 모드 모터 정지) / **1 = 해제 (default)** — 문서 §3.2 프로토콜과 일치
	- 상수: `SOFT_ESTOP_ACTIVE(0)` / `SOFT_ESTOP_RELEASE(1)` (STM `rd_register_ecu.h` ↔ Orin `rd_register_ecu.hpp` 동일)
- STM 동작 (`rd_system.c ACTION_STATE_AUTO`): soft_estop==ACTIVE 면 **FSM 전이 없이 `CAN_AK_ESTOP(BREAK_CURRENT_SW)` 능동 제동**
  (ESTOP_SW 와 동일한 3A 제동). 제동 중 reg 의 cmd_lin/ang_vel 을 0 으로 클리어해 해제 직후 잔여 명령으로 튀는 것 방지.
  해제(1) 시 정상 경로의 ESTOP_override=0 으로 자동 복귀
- `rd_control.c RD_CONTROL_UPDATE`: soft_estop 작동 중 AUTO 분기의 CONSUME/kinematics/LPF 가
  cmd_mtr 를 덮어써 해제 순간 잔여 명령이 TX 되는 문제 차단 — motor_on=0 과 동일하게 LPF·명령 0 리셋 후 return

## 2026-06-12 — 실기 통신 테스트 (데스크톱 ↔ ECU, /dev/ttyUSB0)

STM 빌드 플래시 완료 상태에서 bridge 실기 테스트 수행. **전 항목 통과.**

| 테스트 | 결과 |
|---|---|
| 통신 안정성 | ECU[ON], 손실 2(기동 직후뿐), exec ~2.3ms < 5ms 주기. Tx 220회/2s = 100Hz READ + 10Hz sys (이론값 일치) |
| cmd_vel 안전장치 | 토픽 없음 → 50Hz WRITE skip (Tx 수로 확인). 토픽 발행(20Hz) → WRITE 재개. 정지 후 재차단 |
| mtr_lock (기존 보호) | FAULT 상태에서 cmd_vel WRITE 는 STM 이 ACCESS 거부 — AUTO 모드에서만 수락 (설계 의도) |
| 커맨드 READ (once) | slot auto 배정 → 37ms 내 발사 → RET_OK → hex 덤프 출력 ✓ |
| DEFINE 잠금 | lock 상태 addr1 WRITE → Access Error 재시도 → 2s TIMEOUT 포기 ✓ |
| unlock 체인 | addr0=1 → addr1 write 성공 → read 검증 → addr0=0 ✓ |
| CMD 영역 잠금 해제 | lock 상태에서 addr189 쓰기 성공 (DEFINE 만 잠금 확인) ✓ |
| soft_estop | 부팅 기본값 1(해제) 확인, 0/1 토글 왕복 ✓ (AUTO 제동 동작은 모터 연결 후 확인 필요) |
| reg54 hw_reset 감지 | CAN 모터 미연결 → reg54=0x08 → RCLCPP_ERROR 1초 간격 출력 ✓ |
| hw_reset 매크로 | CLI `macro hw_reset can1` → unlock→reg5=0x08→lock 체인 성공 → STM 이 CAN 복구 + addr54/5 클리어 → ERROR 중단 ✓ |
| REBOOT 커맨드 | alive 476s → REBOOT OK → 3s blackout(connected=false) → 자동 재연결, alive 7.2s (재부팅 확인) ✓ |

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