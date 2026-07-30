# 실행 계획 — 반복성 게이트 + payload 스윕 (2026-07-21 세션)

> **진행 현황**: 세션 A (§1, 로드셀 영점+캘리) **✅ 완료** — 확정 캘리는
> [analysis/result/Stage1_loadcell_calibration_260721.md](analysis/result/Stage1_loadcell_calibration_260721.md)
> (`F=(cnt−214.6)/12.627`, 선형 0~229N, **포화 ~247N → 전류 ≤19A 하드 제약**). **다음 = 세션 B (§2)**.

> [test_plan_traction.md](test_plan_traction.md) Stage 1~4 를 **현 셋업(단일 모터 m2, 수직 payload 무게추 3개)**
> 에 맞춰 실행 배치로 구체화. 인프라: [testbed_spec.md](testbed_spec.md). 프로파일: `data/profiles/`.
> 분석: `analysis/traction/{loadcell_calib,traction_analysis,run_campaign}.py` (2026-07-21 신규/갱신).

## 0. 셋업·핵심 결정

- **무게추 = 캐리어 수직 payload(법선 하중)** (2026-07-21 사용자 확정). 누적 스택:
  **0 / 10.46 / 21.49 / 31.74 kg** = 4 payload 레벨. → 무게추는 **payload 스윕용**이지 로드셀 힘 기준이 아님.
- **모터**: 2번만 연결·전원. bridge `active_motors:="[2]"` (mask=실연결, testbed_spec §3.1 불변식).
- **로드셀 영점 이미 충분**(V5 실측 baseline ch0 1028 / ch1 1034 cnt, 레일마진~1007, 드리프트~0).

### 결정 (2026-07-21 사용자) — 중력-저울 방식으로 지금 cnt→N 캘리, 수평 캘리는 후속

도르래/수평 인장은 **스킵**. 이번 실험은 **저울처럼 로드셀 감지축에 무게추를 중력 방향으로 얹어**
그때 발생하는 힘(F = m·g)으로 cnt↔N 을 잡고, **분석 결과를 [N] 으로** 낸다. 더 정확한 셋업에서
수평방향 캘리를 다시 하며, 지금은 그 방향성 오차는 무시(근사 수용).

절차: 무게추를 로드셀에 누적으로 얹어(0/10.46/21.49/31.74 kg = 0/102.6/210.8/311.3 N) 기록 →
`loadcell_calib.py --calib` 가 plateau 자동검출·선형피팅 후 **`analysis/traction/loadcell_cal.json`**
(정준 캘리 파일)을 기록. 이후 `traction_analysis.py` 가 이 파일을 읽어 로드셀을 자동으로 [N] 으로 환산
(단위·플롯·임계 전부 N). 캘리 파일이 없으면 raw cnt 로 동작(하위호환). ※ 반복성 게이트 slope CV 는
스케일 불변이라 캘리 유무와 무관하게 성립.

## 1. 세션 A — 로드셀 영점 + 중력-저울 cnt→N 캘리 (모터 불필요)

**A-1. 영점 재확인** (5분 무부하):
```bash
testbed_cli run data/profiles/loadcell_zero.yaml --record --name s1_zero
python3 analysis/traction/loadcell_calib.py --zero data/rosbags/s1_zero_<일시>/bag
```
판정: 두 채널 mean ≥300cnt / std(노이즈층 기준값) / 레일마진>0 / 5분 드리프트. (이미 통과 예상 — 확인용.)

**A-2. cnt→N 캘리브레이션** (`loadcell_zero` 5분 hold 재생 중, 무게추를 **로드셀 감지축에 중력방향으로**
각 30s 마다 누적 적재 → 30s 무부하 → 10.46 → +11.03(21.49) → +10.25(31.74) → 유지 → 역순 제거):
```bash
python3 analysis/traction/loadcell_calib.py \
  --calib data/rosbags/s1_zero_<일시>/bag --loads 0 10.46 21.49 31.74 --channel 0
```
- 각 하중 변경 후 **정지·안정 대기**(얹는 순간 충격은 plateau 검출이 자동 배제).
- 판정: `R² ≥ 0.999`, 잔차 std. 성공 시 **`loadcell_cal.json` 자동 기록** → 이후 전 분석이 [N].
- plateau 개수 ≠ 하중 개수 경고 시: 안정 대기 늘리거나 `--loads` 재확인.
- ⚠ 이건 **중력(수직) 방향 캘리** — 견인(수평) 방향과 감도차가 있을 수 있으나 이번 실험은 근사 수용,
  정밀 셋업 때 수평 재캘리(위 결정).

## 2. 세션 B — 모터 견인력 (모터 전원·픽스처 체결·입회)

⚠ **안전수칙** (실기 검증 세션에서 확립): ① 첫 구동은 **3A 이하 hold**, 트랙 픽스처가 힘을 받을 상태인지 사용자 확인 후. ② 이상 시 즉시 `testbed_cli abort` → 그래도 돌면 ECU 전원 차단. LOCKED 전이는 정상 동작 — 원인 파악 전 REARM 금지. ③ bridge 노드 Ctrl+C 시 RW 단절 → ECU AUTO_TIMEOUT(100ms)이 모터를 내림(최후 수단). ④ **전류 ≤19A** (로드셀 포화 229N).
payload 는 무게추를 캐리어에 얹어 설정하며, **payload 변경은 반드시 모터 정지(IDLE)·전원 안전 상태에서**.

**B-1. 첫 실구동 검증** (`std_ramp_cycle` 1회, payload 1개 예: 31.74kg):
```bash
testbed_cli run data/profiles/std_ramp_cycle.yaml --record --name b1_firstlight
python3 analysis/traction/traction_analysis.py <임시 test_index>
```
확인: 전류↑에 **로드셀 Δ 유의 증가**(V5 Δ~27cnt 는 무부하 자유회전 탓 — 픽스처 체결+payload 시 실견인력).
데드밴드(~8A) 이후 선형 상승? 안 보이면 픽스처·장력·`fb_current` 스케일 점검(test_plan §1.6 ⚠ 선결검증).

**B-2. 반복성 게이트 ★ 캠페인의 관문** (`std_ramp_cycle` 5회 + 재장착 2회, payload=31.74kg 고정):
```bash
python3 analysis/traction/run_campaign.py \
  --profile data/profiles/std_ramp_cycle.yaml --label b2_rep_w31 \
  --repeats 5 --reseat-after 5 --extra-after-reseat 2 --settle 8
```
판정(rise-phase, **raw cnt 로 가능 — 스케일 불변**): **slope CV < 5% → 통과** / 잔차 RMS 의 A당량
(TEST2 ≈0.32~0.36A) 기록 / 베이스라인 미복귀 <50cnt(분석기 자동 검출). **불충분 시 캠페인 중단·픽스처 진단**
(강성/장력/로드셀 마운트) → 2회 연속 실패면 "제어 적용 불가" 판정 회의.

**B-3. payload 스윕 ★ 본 매핑 실험** (payload 4레벨 × `std_ramp_cycle` 2회):
```bash
# payload 블록마다 (무게추 얹고/빼고 — 모터 정지 상태에서 교체):
python3 analysis/traction/run_campaign.py --profile data/profiles/std_ramp_cycle.yaml \
  --label b3_sweep_w00 --repeats 2 --settle 8      # 무부하
#   → w10(10.46) → w21(21.49) → w31(31.74) 반복
```
분석: payload → slope/deadband/견인력 관계. 법선하중이 데드밴드(프리로드)·가용 견인력에 미치는 영향.
반복 2회로 각 payload 의 재현성도 확보(B-2 의 축소 반복).

**B-4. 데드밴드·동특성** (기준 payload, 옵션 — B-2 통과 후):
```bash
testbed_cli run data/profiles/deadband_stair.yaml --record --name b4_stair_w31
testbed_cli run data/profiles/step_probe.yaml     --record --name b4_step_w31
```

## 3. 분석·기록 규약

- 모든 run = `data/rosbags/<label>_<일시>/` 자기완결 폴더(bag+profile.yaml+result.json). run_campaign 은
  `session_<label>_<일시>.json` 배치 요약도 남긴다.
- 새 bag → `analysis/traction/test_index.csv` 에 행 추가:
  `bag,payload_kg,velocity_mps,body_angle_deg,contact_point,ramp_slope_aps,note`
  (예: `../../data/rosbags/b3_sweep_w21_r01_07-2x_..,21.49,0,0,p1,0.375,payload 스윕`) → `traction_analysis.py` 재실행.
- 신 bag 은 `TestbedFeedback.msg` 포맷 — 분석기 자동판별(goal_id 분할, 구 TractionTest bag 호환 유지).
- **결과 단위는 [N]** — A-2 캘리 후 `loadcell_cal.json` 이 있으면 자동 환산. (수평 재캘리 시 상수만 갱신하면
  기존 bag 도 재분석으로 소급 반영 — 선형이라 값이 재계산됨.)

## 4. AI 자동화 경계 (test_plan §3 준수)

- 허용: 계획된 YAML 배치 실행 + 즉석 분석 + 분석 결과 따른 **파라미터** 조정(램프 천장·stair 범위·반복 추가).
- 금지: `limits.max_current` 25A 초과 / 신규 세그먼트 **타입** 신설 / 합의 밖 motor_mask·payload 변경.
- 즉시 abort+보고: 베이스라인 미복귀>50cnt / 모터 온도 / `rw_err` 반복 / 예상 밖 힘 / LOCKED.
- **미구현 의존 금지**: `testbed_cli --json` 아직 없음 — 러너는 종료코드+result.json 으로 판정(그것으로 충분).
