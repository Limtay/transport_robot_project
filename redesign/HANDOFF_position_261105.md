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
| ECU 구현 (`stm_ws/ECU_V3`) | **코드 완료, 빌드·실기 미검증.** 오프셋 재계산, 괄호·`#if` 짝, 호출부 확인만 스크립트로 통과 |
| Orin 구체안 (12) | 작성 완료, **사용자 승인 대기** (§4 질문 2개) |
| Orin 구현 | **시작 안 함** |
| CLI 구현 | **시작 안 함** (설계는 12 §7) |
| 웹 구체안 (13) | 미작성. 계약만 12 §8 |

ECU 구현 범위: POS_FB 페이지(addr 256~343), `PACKET_INST_POS`(0x05), `rd_origin.c/h`(자율 원점 FSM), CAN `origin_mask`, RLS 4채널(커스텀 linear 포함), UART4 Checker, `RD_USE_I2C_ENCODER`, `lin_zero_req`(addr 193).

## 3. 다음에 할 일

**사용자 몫**
1. `ECU_V3.ioc` — CAN1 **Transmit FIFO Priority = ENABLE** 후 재생성. 안 하면 `rd_origin` 이 자동 원점을 켜지 않는다 (의도된 방어선, 11 §5.4).
2. STM32CubeIDE Refresh(F5) 후 빌드 — 신규 `rd_origin.c` 포함 확인. **지금은 STM 환경이 없어 뒤로 미룬 상태.**
3. 커스텀 보드 펌웨어: 잘못된 요청 응답을 int32 `0x7FFFFFFF`×2 (와이어 `FF FF FF 7F FF FF FF 7F`) 로 맞추기. 변경 가능성 있음 — ECU 쪽은 `CUST_BAD_REQ_VALUE` 하나만 바꾸면 된다.

**AI 몫**
1. §4 질문 답을 받아 12 를 확정한다 (IMU 를 넣게 되면 POS_FB 크기·11 §2 부터 수정).
2. 12 §10 순서로 Orin·CLI 구현 → 13 웹 구체안 → 웹 구현.
3. 실물 ECU 없이 가짜 ECU 테스트로 먼저 검증 (12 §9). 와이어 호환은 플래시 후에야 증명된다.

## 4. 사용자 결정이 남은 것

1. **position 런에서 IMU 기록?** POS_FB 88B 에 IMU·AS5600 이 없어 NaN 으로 나간다. 몸체 pitch 가 필요하면 POS_FB 에 추가해야 한다. 플래시 전이라 지금이 가장 싸다. (AI 추천: 넣기)
2. **실기 구동 권한.** `CLAUDE_ECU.md` 의 "실기 기동은 사용자 확인 후" 를 AI 실험에서 어떻게 할지. (AI 추천: 유지 — 사용자가 ESTOP 옆에서 런마다 승인)
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

- **`test_stm_mirror` 는 지금 깨진다** — ECU 헤더만 344B 로 바뀌고 Orin 미러는 아직 256B. 12 §2 에서 고친다.
- **구 펌웨어 + 새 Orin 조합**: 12 §3 의 기동 프로브가 막을 예정. 구현 전에는 주의.
- `.ioc` 재생성 후 `main.c` 의 `TransmitFifoPriority` 가 ENABLE 인지 확인. 사용자 코드 블록 밖 수정이 날아가지 않았는지도 diff 로 볼 것.
- 첫 원점 시험은 **저속·무부하**. SET_ORIGIN 유실 시 판정까지 최대 ~12ms 동안 모터가 잘못된 목표로 움직일 수 있다. AK 가 위치 제어 중 SET_ORIGIN 을 받을 때의 거동은 데이터시트로 확정되지 않았다 (11 §8 T3).

## 7. 환경

- 직전 환경(Windows)에는 ROS 2·컴파일러·`gh`·SSH 키가 없었다. 새 환경에서 무엇이 되는지 먼저 확인할 것.
- Orin 빌드: `orin_ws` 루트에서 `./build.sh --test`. 코드 수정 규칙은 [../CLAUDE_ECU.md](../CLAUDE_ECU.md).
- 작업 트리의 `analysis/` 수정 4건은 이 작업과 무관한 사용자 변경이라 커밋에서 뺐다.

## 8. 쓰임새 요약 (ECU 레지스터)

`POS_FB`(256~343) 표는 11 §2, 원점 순서는 11 §5.6, Orin 이 지킬 계약은 11 §9.
