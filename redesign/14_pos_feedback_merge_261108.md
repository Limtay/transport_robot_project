# 14. position_inst 토픽 단일화 (2026-11-08)

> position_inst 기동에서 나가는 ROS 토픽을 `/carrier/control/pos_feedback` 하나로 줄인다. ECU 변경 없음.

## 1. 결정 (사용자, 2026-11-08)

| # | 결정 | 이유 |
|---|---|---|
| D1 | 새 메시지 `PosInstFeedback` 을 만들어 토픽 `/carrier/control/pos_feedback` 으로 낸다 | 구 `PosFeedback` 정의를 바꾸면 그 정의로 받은 bag(서스펜션·로드셀 캘리)을 못 읽는다. sqlite3 bag 에는 메시지 정의가 없다 |
| D2 | position_inst 에서 `/carrier/control/feedback`(ControlFeedback)을 내지 않는다 | 12 Q3 "ControlFeedback 유지" 를 뒤집는다. 같은 값이 두 토픽에 나갔다 |
| D3 | position_inst 에서 `/carrier_imu`·`/carrier_battery`·`/carrier/{ecu,dpc,pcu}/status`·`/carrier/ecu/motor` 를 내지 않는다 | POS_FB 에 없는 블록이라 섀도의 낡은 값이 나갔다. 저장소 안에 구독자가 없다 |

다른 기동(current·velocity·control_test·project)은 바뀌지 않는다. `comm_diag` 는 그대로 파라미터로 켠다.

## 2. `PosInstFeedback`

정의: `orin_ws/src/mgs_tp_msgs/msg/PosInstFeedback.msg`.

- **POS_FB 88B 전체** (addr 256~343) — 원값 그대로. 모터 위치·속도·전류, 로드셀 2ch, RLS 3ch, 리니어 x·y, 원점 진단(odiag), degraded_cnt, 상태 바이트.
- **위치 명령** — `cmd`(ECU 에 쓴 cmd_position, 모터 좌표), `cmd_abs`·`fb_abs`·`offset`(브리지 절대 좌표).
- **브리지 상태** — control_state·write_source·auto_mode·motor_mask·goal_id·profile_time·segment_index.
- **환산값** — `fb_position`[deg]·`fb_velocity`[RPM]·`fb_current`[A]·`rls_deg`. ControlFeedback 과 같은 이름이라 웹 상태 패널이 그대로 쓴다.
- **RW 결과** — rw_err·drop_cnt·irregular_tick_cnt·late_tick_cnt, stamp_valid.

POS_FB 에 없는 값: `hw_error`·`hw_fatal`(웹은 255 = 미판독으로 표시), IMU, 링크 각, 센서별 취득 지연.

## 3. 바뀐 곳

| 파일 | 변경 |
|---|---|
| `mgs_tp_msgs` | `PosInstFeedback.msg` 추가. `PosFeedback.msg` 는 구 bag 용으로 유지 (주석만) |
| `rd_telemetry.{hpp,cpp}` | position_inst 에서 상태·IMU·ControlFeedback 발행자를 만들지 않음. pos_feedback 을 새 타입으로 채움 |
| `test_interface_inventory.cpp` | position_inst 에 pos_feedback 만 있는지 검사 |
| `control_cli/record.py` | `record_topics(pos=True)` = pos_feedback 하나 |
| `control_web/server.py` | pos_feedback 하나로 상태 표시·슬라이더 추종 |
| `rec.sh` | 요약을 새 메시지에서 읽음 |

## 4. 검증

- 브리지 gtest 334 중 실패 7. 6 개는 `test_active_motors` (FTDI 어댑터가 꽂힌 채 ECU 전원이 꺼져 있어 "USB 대기" 대신 INIT 실패로 끝남 — 이 변경과 무관, USB 를 빼면 통과 전제). 나머지 1 은 같은 묶음의 CTest 집계.
- control_cli 39, control_web 121 통과 (새 테스트 2).
- `rec.sh` 요약: 합성 bag 으로 확인.
- ⚠ **실기 미확인**: ECU 연결 상태에서 토픽 목록, 200Hz, 필드 값(특히 `cmd`·로드셀·리니어), 웹 상태 패널.
