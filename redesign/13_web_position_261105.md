# 13 — 웹 Position Control 구체안 (2026-11-05)

> 범위: `orin_ws/src/control_web`. 계약은 [12_orin_position_261105.md](12_orin_position_261105.md) §8, 웹 구조는 [07_web.md](07_web.md). 브리지·ECU 는 고치지 않는다.
> 웹은 사용자 조작용이다. 판정은 브리지가 하고, 웹은 보여 주고 전달한다. 단, 절대 좌표 때문에 **웹이 직접 위험을 만드는 곳**은 웹에서 막는다 (§3).

## 1. 기동

- control 패널에 `position_inst` 체크박스. 값은 그대로 기동 파라미터로 나간다.
- 조합 검증은 브리지 기동 게이트가 한다 (`bridge_mode:=control` + `auto_mode:=position` 필수). 웹이 다시 적지 않는다.
- 터미널에서 `./run.sh position` 으로 띄운 브리지에 붙어도 같게 동작하도록, 판정은 기동 파라미터가 아니라 `GET_STATUS.position_inst` 로 한다.

## 2. 표시

- `pos_feedback` 를 구독한다. `snapshot` 에 `position_inst`, `pos`, `last_reject`, `pos_fb` 를 넣는다.
- `pos_fb` = `fb_abs`·`cmd_abs`·`offset`·`origin_seq`·`origin_flag`·`fb_abs_held`. 1초 넘게 안 오면 빠진다.
- **NaN 은 null 로 바꾼다.** `cmd_abs` 는 좌표 무효면 NaN 인데, `json.dumps` 의 `NaN` 은 브라우저 `JSON.parse` 를 통째로 깨서 화면이 "웹 노드 응답 없음" 이 된다.
- Tab1 ① 표: `position_inst` 면 position·cmd 칸이 절대값이다 (헤더도 바뀐다).
- 기동 패널 안의 위치 제어 칸 (Tab1·Tab2 공유): 모터별 abs·cmd·offset·seq, 원점 진행(PEND), **FRAME_LOST·LOCKED 경고**, 마지막 거부 사유.

## 3. Tab1 슬라이더 (STREAM = 절대값)

| 위험 | 대응 |
|---|---|
| 안 만진 슬라이더의 초기값이 모터 좌표면 원점 이동 뒤 약 3000° 어긋남 | 초기값은 `fb_position` 이 아니라 `fb_abs` (12 §8) |
| 범위가 ±180° 고정이면 965° 에서 슬라이더를 건드리는 순간 170° 로 간다 | 범위를 **현재 setpoint 기준 ±30°** 로 옮긴다. 손을 떼면 다시 가운데로 온다. 트랙이 걸린 상태라 한 번에 크게 못 가게 한 값이다 |
| 첫 `pos_feedback` 전에 run 을 누르면 setpoint 0 = "절대 0° 로 가라" | 상태 수신 전, `pos_fb` 없음, 활성 모터 `fb_abs` 무효, FRAME_LOST, PEND 면 **arm 거부** |
| 비활성 모터에 모터 좌표를 넣으면 절대값과 섞인다 | `position_inst` 면 `fb_abs` 를 넣는다 |

원점 잡기 버튼은 `auto_mode=direct` 조건으로 이미 꺼져 있고, 브리지도 거부한다.

## 4. REARM

과속 제동·추종 오차로 LOCKED 가 되면 웹에서 복구할 길이 없었다. LOCKED 일 때만 켜지는 REARM 버튼을 두고, 기존 `/api/config` 로 `OP_REARM` 을 보낸다. 제동이 풀리므로 확인 창을 띄운다.

## 5. Tab2

- `position_ref` 선택, `limits.max_track_err` 입력. `build_yaml` 은 그대로 넘긴다. 없어서 거부되는 판정은 브리지 몫이다.
- 기록: `record.start_bag(pos=True)`, `result_from_action(position_inst=True)` — CLI 와 같은 함수.
- 결과에 원점 횟수, 시작·끝 절대 위치, 최대 추종 오차를 표시한다.

## 6. Tab4

`regmap.json` 은 이미 POS_FB 를 포함한 344B 이고 `regmap.js` 는 맵을 읽어 그리므로 코드 변경이 없다. POS_FB 의 타입을 디코더가 다 아는지 테스트로만 잠근다.

## 7. 넣지 않은 것

- `lin-zero`·`preflight` 버튼: AI 실험은 CLI 가 주도하고, 웹에 넣을 요구가 아직 없다.
- 슬라이더 값 자체의 한계: 브리지는 STREAM 절대값을 자르지 않는다. ±30° 는 UI 조작 범위이고 안전 한계가 아니다.

## 8. 검증

- 단위: `test_position_web.py` (setpoint·arm 게이트·NaN·범위·발행값·기록), `test_profile_build.py`, `test_tabs.py`.
- ⚠ 브라우저 조작은 실기에서 확인해야 한다. 런마다 사용자 승인.
