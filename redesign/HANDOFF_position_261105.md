# 핸드오프 — Position Control 작업 (2026-11-05)

> 새 환경에서 이어받는 사람(또는 AI 세션)의 진입점. 브랜치 `feature/ecu_ctr`, **main 은 건드리지 않는다.**

## 1. 무엇을 만드는 중인가

플랜의 위치 궤적을 200Hz 로 넣는 전용 경로. AK 피드백이 ±3200° 까지만 읽히므로 **ECU 가 3000° 에서 스스로 원점을 옮기고**, Orin 은 cmd 만 보내다가 `origin_seq` 가 바뀌면 offset 을 더한다. 실험은 AI 가 CLI 로 주도할 예정이라 CLI·웹도 같이 고친다.

- 원 요구: `research/modeling/sim/planning/User/261030_SW_수정계획.md` (이 git 리포 밖, 워크스페이스 안)
- ECU 설계: [11_ecu_position_261030.md](11_ecu_position_261030.md)
- Orin 설계: [12_orin_position_261105.md](12_orin_position_261105.md) — **§1 부터 읽을 것**

## 2. 현재 상태

| 단계 | 상태 |
|---|---|
| ECU 구체안 (11) | 완료 |
| ECU 구현 (`stm_ws/ECU_V3`) | **코드 완료, CubeIDE 빌드·플래시 완료 (2026-11-05, 사용자). 동작 시험(11 §8) 미실시.** `.ioc` 재생성 반영: CAN1 TXFP=ENABLE, 클럭 소스 HSI→HSE |
| Orin 구체안 (12) | **확정** (§4 결정 반영) |
| Orin 구현 | **코드·가짜 ECU 테스트 완료 (2026-11-05), 실기 미검증.** 12 §10 1~7단계 |
| CLI 구현 | **완료, 실기 미검증** (`preflight`·`lin-zero`·`run --json`, `run.sh position`) |
| 웹 구체안 (13) | 미작성. 계약만 12 §8 |

ECU 구현 범위: POS_FB 페이지(addr 256~343), `PACKET_INST_POS`(0x05), `rd_origin.c/h`(자율 원점 FSM), CAN `origin_mask`, RLS 4채널(커스텀 linear 포함), UART4 Checker, `RD_USE_I2C_ENCODER`, `lin_zero_req`(addr 193).

## 3. 다음에 할 일

**사용자 몫**
1. ~~`ECU_V3.ioc` CAN1 Transmit FIFO Priority = ENABLE~~ **완료.** ~~CubeIDE 빌드~~ **완료.**
2. 첫 연결 때 HSE 확인: 보드 수정(crystal)이 16MHz 여야 한다 (`HSE_VALUE`·`.ioc` 가정). 다르면 CAN·UART·TIM5 가 전부 어긋난다. HSE 가 안 뜨면 부팅 중 `Error_Handler` 에 멈춰 RS485 무응답이 된다.
3. 커스텀 보드 펌웨어: 잘못된 요청 응답을 int32 `0x7FFFFFFF`×2 (와이어 `FF FF FF 7F FF FF FF 7F`) 로 맞추기. 변경 가능성 있음 — ECU 쪽은 `CUST_BAD_REQ_VALUE` 하나만 바꾸면 된다.

**AI 몫**
1. 13 웹 구체안 → 웹 구현 (계약은 12 §8). ⚠ 웹의 `record.start_bag`·`result_from_action` 호출은 아직 `pos=True`/`position_inst=True` 를 넘기지 않아 position 런의 웹 기록에 `pos_feedback` 이 빠진다.
2. 플래시 후 11 §8 T0~T11. 와이어 호환은 그때 증명된다. 가짜 ECU(`test_pos_inst`)는 우리가 이해한 ECU 다.
3. 미검증: FRAME_LOST REARM 절차(CURRENT→30ms→POSITION)는 서비스 경로로 돌려 본 적이 없다.

## 4. 사용자 결정이 남은 것

1. ~~position 런에서 IMU 기록?~~ **넣지 않는다 (2026-11-05).** POS_FB 88B 유지, IMU·AS5600 은 NaN.
2. ~~실기 구동 권한.~~ **런마다 사용자 승인 (2026-11-05).** 사용자가 ESTOP 옆에 있을 때만 실행.
3. 플랜 → YAML 변환 도구: 트랙 변위 → 모터 각도 스케일, 모터별 부호, 활성 모터 2개 지정이 필요. T1 실측 후.
4. `AK_VEL_RAW_TO_DEGPS` 는 잠정값 (fb 가 출력축 기준이라는 가정). 진단 전용이라 제어에는 영향 없음. T1 실측으로 확정.

## 5. 확정된 결정 (다시 논의하지 말 것)

- 원점은 ECU 자율. old-frame 판정은 fb 가 아니라 `last_tx`. Orin offset 트리거는 flag 가 아니라 **`origin_seq` 변화**.
- S(`pre_origin_pose`) = fb_old. 확인 후에만 seq 발행. 오차는 `odiag_*` 로 실측 후 외삽 여부 판단.
- 원점 FSM 은 **INST_POS 수신 중에만** 동작 — RW 로 보내는 기존 POSITION 은 그대로 보존.
- FRAME_LOST 는 TX 차단이 아니라 **제동** (AK 는 마지막 목표를 계속 따라간다).
- linear 패킷: `'3'` → int32 x,y LE 8B / `'i'` → 성공 0x00×8, 실패 0x11×8 / MUX 4번 채널 `enc[3]`, 다른 RLS 와 **같은 드라이버 구조** (task 에 별도 코드 금지 — 사용자 지시).
- 진단 인덱스: 5 = 로드셀 예약, **6 = uart4** (Orin 규약 03 §3.1 과 일치).
- Orin: INST_POS 는 기동 시 고정, 새 토픽 `pos_feedback`, `ControlFeedback` 정의 불변 (옛 bag 호환).

## 6. 함정

- **구 펌웨어 + 새 Orin**: `position_inst` 는 기동 프로브가 막는다. 일반 control 은 그대로 동작한다.
- position 기동의 `MotorStatus.temp`·`error_code` 는 기동 스냅샷이다 (int8 라 NaN 불가, POS_FB 에 없음). 실행 중 판단에 쓰지 말고 LOCKED 후 `read_all` 로 읽을 것.
- `.ioc` 재생성 후 `main.c` 의 `TransmitFifoPriority` 가 ENABLE 인지 확인. 사용자 코드 블록 밖 수정이 날아가지 않았는지도 diff 로 볼 것.
- 첫 원점 시험은 **저속·무부하**. SET_ORIGIN 유실 시 판정까지 최대 ~12ms 동안 모터가 잘못된 목표로 움직일 수 있다. AK 가 위치 제어 중 SET_ORIGIN 을 받을 때의 거동은 데이터시트로 확정되지 않았다 (11 §8 T3).

## 7. 환경

- **작업 트리는 `~/tp_ws` (ext4, LF).** Google Drive 사본(`track_control/fw/transport_robot_project`)은 작업 트리가 CRLF 라 `build.sh` 가 안 돌고, 거기서 커밋하면 줄바꿈 diff 약 80만 줄이 생긴다 — 코드를 고치거나 커밋하지 말 것.
- RLS 배선 확정 (2026-11-05, 사용자): 1·2번 AksIM, 3번 Orbis, 4번 커스텀 = 코드의 `enc[0..3]`, 11 §6 과 일치. 10월 채널별 기종 배열안(ch0=Orbis)은 쓰지 않으며 `git stash` 의 `rls_enc_model_261022` 에 남아 있다.
- 이 머신: ROS 2·colcon·ssh·STM32CubeIDE 있음 (`ECU_V3/Debug` 가 `~/tp_ws` 에 생성된다). `PATH` 에 ARM 툴체인·`gh` 없음.
- Orin 빌드: `orin_ws` 루트에서 `./build.sh --test`. 코드 수정 규칙은 [../CLAUDE_ECU.md](../CLAUDE_ECU.md).
- ⚠ `build.sh --test`(패키지 병렬)에서 `control_cli` e2e 1건·`test_run_profile_action` 이 간헐 실패했다 (같은 커밋, 1회 재현). `--executor sequential` 로는 전부 통과. 원인 미확정 — `control_cli/test/conftest.py` 가 도메인을 PID 로 분리하므로 도메인 혼선으로 단정할 수 없다. 판정은 `colcon test --packages-select control_cli orin_firmware_bridge --executor sequential` 로 할 것.

## 8. 쓰임새 요약 (ECU 레지스터)

`POS_FB`(256~343) 표는 11 §2, 원점 순서는 11 §5.6, Orin 이 지킬 계약은 11 §9.
