# TEST3 Stage 2 — 자율 프로파일 배터리 (40kg 단일 세팅, 무인 수집)

> 2026-07-21 설계. **한 물리 세팅(payload 40kg 고정, 재장착·교체 없음)에서 무인으로 수집 가능한
> 프로파일만** 순차 실행 → 데이터만 모으고 **분석은 전량 수집 후 별도 세션에서 일괄** 수행.
> 실행 주체: `analysis/traction/run_battery.py` (오케스트레이터). 문답 없이 자율 진행.

## 고정 조건 (하드)

- 모터 **m2** 단독, `active_motors:="[2]"`, auto_mode=**CURRENT**, **전류 상한 14.0A**.
- **payload 40kg** (안전게이트 `b2_pre_w40` 에서 15A 피크 **214.6N** 실측 → 선형상한 229N 여유 14.5N.
  14A 로 낮춰 피크 ~190N·여유 ~40N 확보. 15A 로 올리지 말 것 — 반복 중 포화 위험).
- 데드밴드 ~6.5A, rise slope ~24 N/A (이론밴드 18–25 안). 큰 이력(캡스턴/스틱션) + 종료직후 잔류
  20N→15~20s 이완 후 ~4.5N. → 런 사이 **settle 20s**(쿨다운+이완). 매핑은 **rise 곡선** 기준.

## 배터리 (12런, ~20분) — 실행 순서 = `run_battery.py:BATTERY`

| # | 프로파일 | 반복 | 성격 | 길이 |
|---|----------|------|------|------|
| 1 | camp_ramp_slow (0.20 A/s) | ×1 | 준정적 램프(이력 기준선) | 153s |
| 2 | camp_ramp_med (0.375 A/s) | ×4 | **반복성 게이트**(in-place rise-slope CV) | 88s |
| 3 | camp_ramp_fast (0.70 A/s) | ×2 | 율의존 이력 | 53s |
| 4 | camp_deadband_stair | ×1 | 계단 상승/하강(데드밴드·레벨별 이력) | 100s |
| 5 | camp_step (0→7→14→7→0) | ×1 | 스텝 동특성 | 45s |
| 6 | camp_sine_lo (0.1Hz, 0–14A) | ×1 | 준정적 이력 루프 | 70s |
| 7 | camp_chirp (0.05→1.0Hz) | ×1 | 주파수 응답 | 70s |
| 8 | camp_prbs (3↔13A, bit0.5s, seed42) | ×1 | 광대역 시스템 ID | 70s |

- 전 프로파일 `lint_profiles.py 2 14.0` 통과: reject 없음, 전류 [0,14]A, 클램프·음수 0.
- 라벨 규격: `c_<stem>_w40_r<NN>` → `data/rosbags/<label>_<일시>/`(bag+profile.yaml+result.json).

## 실행 방법 (다음 세션)

```bash
# 사전: 브리지가 control_mode + active_motors:=[2] 로 IDLE (이미 기동됨).
#       latency_timer=1 필수 (echo 1 | sudo tee .../ttyUSB0/latency_timer). payload 40kg 적재.
bash analysis/traction/run_battery.sh   # 백그라운드 권장, ~20분. 진행 로그 = stdout.
```

- 오케스트레이터 자동 안전장치: 프리플라이트 IDLE 확인 / 런마다 result.json success·write_err 검증 /
  런 사이 settle 후 **상태 재확인(IDLE 아니면 = HW_ESTOP/FAULT → 즉시 중단·abort)** / 이상 시 배터리 정지.
- HW_ESTOP 은 사용자 담당 — 누르면 다음 런 전 상태확인에서 걸려 배터리가 자동 중단됨.
- 세션 요약: `data/rosbags/battery_session_<일시>.json` (모든 런의 폴더·result·slope·payload).

## 분석 (전량 수집 후 별도 세션)

- 소스: `battery_session_*.json` → 각 run_dir. rise 곡선 기준 slope/deadband, 반복성 CV(med×4),
  율의존 이력(slow/med/fast), stair 레벨별 이력, step 라이즈타임, sine/chirp/prbs 시스템 ID.
- ⚠ 현 `traction_analysis.py` 는 램프(find_ramps) 중심 — sine/chirp/prbs/stair 전용 분석기는
  이 분석 세션에서 확장 필요. bag(feedback)은 프로파일 종류와 무관하게 전량 기록됨.
- `test_index.csv` 행은 `battery_session_*.json` 에서 생성(payload=40, slope=표대로).
- 로드셀 [N] 자동 환산은 `loadcell_cal.json`(F=(cnt−214.6)/12.627) 이 담당.

## 이 세션에서 이미 수집된 선행 런 (배터리 아님, 참고)

| 라벨 | 프로파일 | 결과 |
|------|----------|------|
| b1_firstlight | firstlight_3a (3A) | 명령경로✅, 3A<데드밴드라 힘 무반응 |
| b1b_std15 | std_ramp_cycle (15A, payload 미상) | 피크 209N(tared), slope 23.9 N/A, deadband 6.5A, 큰 이력 |
| b2_pre_w40 | std_ramp_cycle (15A, **40kg**) | 안전게이트: 절대피크 **214.6N** → 14A 결정 근거 |
