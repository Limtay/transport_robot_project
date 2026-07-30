# TEST3 실험 계획 — Current–Traction Mapping 캠페인 (정적)

> Phase 3 산출물 (2026-07-19, **2026-07-21 실측 반영 갱신**). 실행 인프라: [testbed_spec.md](testbed_spec.md) / 태스크 맥락: [TASKS.md](TASKS.md) §2.
> 선행 캠페인 결론: [wiki/experiments/2026-07_traction-current-mapping.md](../wiki/experiments/2026-07_traction-current-mapping.md) (TEST2).
> **전제 충족됨**: testbed_spec §6 #0~#9 구현·실기 검증 완료, Stage 0·1 완료. 다음 실행: **Stage 2** (세부 배치: [test_plan_stage1_260721.md](test_plan_stage1_260721.md) 세션 B). 완료된 스테이지는 `/log-experiment` 로 wiki 컴파일.
>
> **⚠ 전 스테이지 공통 하드 제약 (2026-07-21 Stage 1 실측)**:
> 1. **로드셀 힘 ≤ ~229 N** — ch0 는 ~247 N(25 kg)에서 하드 포화(~3234 cnt, 무응답). 이론 slope 기준 **전류 상한 ≈ 19 A**. 프로파일 `limits.max_current` 는 15 A 표준, 절대 19 A 초과 금지.
> 2. **`active_motors` = 실연결·전원 모터와 정확 일치** (testbed_spec §3.1 불변식) — 위반 시 AUTO 진입에서 CAN fatal→FAULT. 현 벤치: 모터 2번만 → `active_motors:="[2]"`.
> 3. **auto_mode 는 CURRENT(1) 만** — DIRECT(2) 전환은 브리지 크래시 이월 버그 (testbed_spec §6).

## 0. 캠페인 목표 — 판정할 질문 3개

1. **반복성 판정** (Stage 2 게이트): 영점 조정 + 결정적 프로파일 조건에서 traction 맵의
   반복성이 제어 연구에 충분한가? — **불충분 시 이 트랙은 제어 적용 불가** (memo_260710 기준), 캠페인 중단·하드웨어 재설계.
2. **p3/p4 미스터리** (Stage 3): 위치별 2× 성능 차이의 원인이 스프로킷 위상인가? — fb_position 로깅으로 분리.
3. **절대력 맵 + 불확도** (Stage 4~5): MPC에 넘길 최종 산출 —
   `(위치·위상·방향) → traction[N] ↔ current[A]` 맵과 그 잔차(불확도, MPC 안전마진 입력).

범위 제외: **트랙 회전 속도 ≠ 0** (kinetic 영역) → TEST4 후속 캠페인 (§6). pitch angle 은 Stage 4 옵션 (픽스처 각도 조정 가능 시).

## 1. Stage 구조 (의존성 순 — 각 스테이지는 이전 통과가 전제)

### Stage 0 — 통신 특성화 ✅ 완료 (2026-07-20~21)

- 실측 (V1 bag 796.8s, `analysis/latency/latency_analysis.py`): **200.00 Hz** 유지, over-period 0.03%,
  RTT mean 1.96 ms·p99 2.30 ms, offset 수렴 ≤6 s.
- **drift 실측 ≈ -19,600 ppm (-1.96%)** — "수십 ppm" 기대는 오류였음. ECU TIM5 가 HSI(내부 RC) 기반인
  하드웨어 특성 (testbed_spec §2.5). 기능 정상 — **분석·MPC 는 raw ecu_tick 을 10kHz 로 가정 금지,
  보정된 `clock_offset` 사용** (CommLatency.msg 규칙).
- 산출: `analysis/latency/out/V1_commlatency_07-20_19-08/` (summary.json + 3 png).

### Stage 1 — 로드셀 캘리브레이션 ✅ 완료 (2026-07-21)

- **cnt→N 캘리 확정** (중력-저울 방식, 3사이클 46점, 결과 문서: `analysis/result/Stage1_loadcell_calibration_260721.md`):
  **`F[N] = (cnt − 214.6) / 12.627`** (0.07920 N/cnt), R²=0.999988, 잔차 ±0.24 N, 히스테리시스 0.17 N.
  정준 파일 `analysis/traction/loadcell_cal.json` → 분석 파이프라인이 자동 [N] 출력.
- **하드 포화 발견**: ~247 N(25 kg)에서 ~3234 cnt 천장, 그 위 무응답 → **선형 유효 0~229 N** (상단 하드 제약).
- 잔여 (후속, 비블로킹): **수평(견인) 방향 재캘리** — 현 캘리는 수직 근사. 선형이라 상수만 갱신하면 기존 bag 소급 재분석.
- 무부하 baseline 은 마운트 상태에 따라 ~210 또는 ~1030 cnt 로 관측됨 — **각 실험 세션 시작 시 무부하값 확인**
  후 진행 (분석은 tare 차감이라 절대값 무관, 단 음방향 헤드룸 확인용).

### Stage 2 — 반복성 게이트 ★ 캠페인의 관문 (다음 실행 — 배치: [test_plan_stage1_260721.md](test_plan_stage1_260721.md) 세션 B)

- 조건: 현 벤치 구성 — **단일 모터 m2, payload 31.74 kg**(가용 무게추 스택), 표준 램프 사이클 (§2 `std_ramp_cycle`).
- 실행: 동일 YAML **5 run** (run 간 트랙 재정렬 없이 연속, `analysis/traction/run_campaign.py` 사용) + 트랙 위치 재설정 후 **추가 2 run** (재장착 재현성).
- 판정 기준 (rise-phase 기준):
  - **slope CV < 5% → 통과** (TEST2 p1/p2 실측 1.7~3.1% 재현 확인 수준)
  - 동시 기록: 잔차 RMS 의 **A 당량 환산** (TEST2: ≈0.32~0.36 A) → MPC 불확도 파라미터로 보고
  - 베이스라인 미복귀 < 50cnt (픽스처 밀림 플래그 기준, 기존 분석기 자동 검출)
- 실패 시: 캠페인 중단 → 분기 진단 (픽스처 강성 / 트랙 장력 / 로드셀 마운트) 후 재시도. 2회 연속 실패 시 "제어 적용 불가" 판정 회의.

### Stage 3 — p3/p4 진단 (스프로킷 위상 분리)

- 가설: 위치 효과의 실체는 스프로킷 각 위상 (rubber lug ↔ 로드셀 접촉 기하).
- 설계: 동일 트랙 위치에서 **스프로킷 1회전을 위상 그리드 6~8점**으로 나눠, 각 위상에서 표준 램프 사이클 2 run.
  - 위상 조정: config service 로 저전류 미세 이송 → `fb_position` 으로 위상 확인 후 정지 (테스트베드 인프라로 가능해진 부분).
  - 모든 bag 에 fb_position 이 200Hz 로 기록되므로 사후 위상 라벨링 자동.
- 분석: 위상 vs slope/deadband/잔차 상관 → 위상 효과가 유의하면 맵에 위상 축 추가, 무의미하면 p3/p4 는 다른 원인 (픽스처 기하) 재조사.

### Stage 4 — 변수 스윕 (정적)

- 축: **position (p1~p4) × payload (가용 무게추 확정: 0 / 10.46 / 21.49 / 31.74 kg 누적 스택) × 방향 (rise/fall)**. pitch 는 픽스처 허용 시 추가.
- 조건당 프로파일 3종 (§2): `std_ramp_cycle` (맵 본체) + `deadband_stair` (데드밴드 정밀) + `step_probe` (lag τ 식별).
- run 수: 조건당 ramp 2 + stair 1 + step 1 = 4 bag. 전체 규모는 Stage 3 결과(위상 축 필요 여부)에 따라 확정.
- hold-out: 조건 그리드의 ~20% 는 피팅에서 제외 (Stage 5 검증 전용) — 실행 시점에 무작위 지정.

### Stage 5 — 모델 확정 (오프라인)

- 후보: ① per-phase(rise/fall) deadband+linear (+first-order lag) — TEST2 방식 확장 ② 히스테리시스 연산자 (Bouc-Wen 급) — step/PRBS 데이터로 판단.
- 판정: hold-out RMS [N] 및 A 당량. 목표: 전 조건에서 TEST2 p1/p2 수준 (≤ ~0.4 A 당량) 달성.
- 산출: 런타임 맵 함수 + 조건별 불확도 표 → `/log-experiment` 로 wiki 컴파일 + MPC 설계 입력 (research/plan 연동).

## 1.6 이론 앵커 — 전류→견인력 이론 기울기·오프셋 (Stage 1·5 기준선)

모터 AKE90-8 스펙([doc/AKE90-8 spec.png](doc/AKE90-8%20spec.png)) + 스프로킷 반지름으로 이론 slope 를
계산해, 절대력 캘리브레이션(Stage 1) 후 실측 slope 의 기준선으로 쓴다.

**파라미터**: Kt = 0.272 Nm/A (로터축), 감속비 N = 8, r = 0.105 m (스프로킷축→힘 작용점).

**정적 견인력 모델**: `F = η·Kt·N·(I − I0) / r`

- **이론 기울기** `a = Kt·N/r = 0.272×8/0.105 ≈ 20.7 N/A` (η=1).
- Kt 는 로터축 기준 확인: 출력축 기준이면 정격토크가 5.7Nm 여야 하나 스펙 55Nm → 감속 전 값.
- 스펙 교차검증 (출력 토크상수 역산, 데이터시트 값 불완전 일관):
  Kt 공칭 2.18 → **20.7** / 피크점 170Nm÷72A=2.36 → 22.5 / 정격점 55Nm÷21A=2.62 → 24.9 N/A.
- **이론 밴드 ≈ 20.7~24.9 N/A**, 유성기어 η≈0.9 반영 시 하한 ~18.6 → **중심 ~20-21, 실측 예상 18~25 N/A**.

**오프셋(deadband I0)의 물리적 의미**: I=I0 에서 F=0 →
출력토크 η·Kt·N·I0 이 전부 마찰/프리로드로 소모. TEST2 I0≈8A 환산:
`T_friction^out = a·I0·r ≈ 17.4 Nm` (출력축), 견인력 당량 `a·I0 ≈ 165 N`.
- 성분 분리: 위치 무관(기어·베어링 Coulomb) + 위치 의존(고무트랙 slack·변형 take-up·정적 접촉마찰).
- p3/p4 데드밴드 산포 3~7A → **~60~145 N 위치 의존 프리로드 변동** → Stage 3 가설(접촉 기하)과 정량 정합.

**분석 활용**:
- Stage 1 후 실측 slope[N/A] 를 이 밴드와 대조 → 힘 전달 온전성/효율 추정.
- 실측 ≪ 이론이면: **전류 과보고(CubeMars AK 알려진 이슈)** / 고무 컴플라이언스 힘 손실 / 효율 저하 분해.
- ⚠ **선결 검증**: reg `fb_current` 가 모터 Iq(토크전류)인지 확인 — AK 서보 프로토콜은 보통 Iq[A]지만
  펌웨어 스케일 보고 사례 존재. 실측·이론 slope 괴리 시 1순위 용의자.

## 2. 표준 프로파일 라이브러리 (파일화 완료: **`data/profiles/`**, 2026-07-21)

| 이름 | 구성 (파일 실측) | 용도 |
|------|------|------|
| `std_ramp_cycle.yaml` | hold 5s(영점) → ramp 0→**15A** 40s → hold 3s → ramp 15→0A 40s → hold 5s (93s, 0.375A/s) | 맵 본체 (rise+fall). 반복은 `run_campaign.py` 가 N회 |
| `deadband_stair.yaml` | hold 5s → stair 4~12A 0.5A 단 × 12s 홀드 (상승 후 하강 왕복) | 데드밴드·정상상태 포인트 정밀 |
| `step_probe.yaml` | hold 5s → {8→12, 12→8, 12→18, 18→12 A} 스텝 각 20s 홀드 | first-order lag τ 식별 |
| `loadcell_zero.yaml` | 0A hold 5분 (max_current 0.1) | 로드셀 baseline/노이즈/드리프트 (무토크) |
| `sysid_prbs` (옵션, 미작성) | 데드밴드 위 동작점 ±2A PRBS, bit 0.5s, 120s, seed 고정 | Stage 5 히스테리시스/동특성 판별 보강 |

공통 규칙: 프로파일 첫 hold 는 분석기 tare 창 / **limits.max_current 15A 표준, 절대 상한 19A (로드셀 포화 229N — 상단 하드 제약 1)** / 전류는 대상 모터 1개(현 벤치 m2), 나머지 0A.
(구 계획의 25A 상한은 2026-07-21 로드셀 포화 실측으로 **하향 확정** — 25A ≈ 350N 은 측정 불가.)

## 3. AI 자동화 프로토콜 (결정: 세션 내 적응형)

**세션 = 사용자가 하드웨어·안전 확인 후 개시를 선언한 연속 실험 구간.** 세션 내에서 Claude 는:

- **허용**: 계획된 YAML 배치 실행(`run --record`) / 즉석 분석 / 분석 결과에 따른 **다음 프로파일 즉석 수정** (예: 데드밴드 실측치 근방으로 stair 범위 재조정, 반복 추가).
- **안전 경계** (수정 자유의 한계 — testbed_spec §4.3 검증이 최종 방어선):
  1. `limits.max_current` **19A 초과 프로파일 생성 금지** (로드셀 포화 — 상단 하드 제약 1. 표준은 15A).
  2. 계획에 없는 프로파일 **타입** 신설 금지 (§2 라이브러리의 파라미터 변경만).
  3. 이상 징후 시 **즉시 abort + 사용자 보고 후 대기**: 베이스라인 미복귀 >4N(≈50cnt) / 모터 온도 경고 / `rw_err` 반복 / 로드셀 포화역(>229N) 진입 / 예상 범위 밖 힘.
  4. 모터 대상·motor_mask 변경은 세션 개시 시 합의된 것만 (현 벤치: m2 만).
  5. **미구현 CLI 기능(`--json` 등) 의존 금지** — 자동화 판정은 종료코드 + result.json (testbed_spec §5.1 TODO 참조).
- **기록**: 실험 1회 = `data/rosbags/<name>_<일시>/` (bag + YAML 사본 + result) — 즉석 수정된 YAML 도 동일 규격으로 남아 사후 재현 가능.
- **세션 종료 시**: 배치 요약 (run 목록, 판정 지표, 이상 이력) 보고 → 다음 세션 계획은 사용자와 합의.

## 4. 실행 전 체크리스트 (선행 조건 — 2026-07-21 현황)

- [x] colcon 빌드 + STM 플래시 + 실기 검증 V0~V6 (2026-07-20~21 완주)
- [x] testbed_spec §6 #1~#8 구현·실기 검증 (msg/FSM/player/config/CLI)
- [x] motor_mask 실기 확인 — 단일 모터 벤치 검증 + **§3.1 불변식 확립** (mask=실연결 일치 필수)
- [x] 로드셀 캘리브레이션 (Stage 1) — cnt→N 확정, 포화 한계 규명
- [x] 분석 파이프라인: TestbedFeedback 디코더 + goal_id 분할 + [N] 단위 (`analysis/traction/`, 2026-07-21)
      — 위상 라벨링(fb_position 기반)은 Stage 3 착수 시 추가
- [ ] **Stage 2 세션 개시 조건**: 모터 2번 전원 ON + 트랙 픽스처 체결 + 사용자 입회 + 무부하 로드셀 값 확인

## 5. TEST4 예고 (범위 밖 — 셋업 검토 항목만)

- 트랙 회전 속도 ≠0 (kinetic 영역): 현 정적 픽스처에서 회전 중 로드셀 부하가 유지 가능한지 검토 필요 (트랙-지면 슬립 vs 픽스처 견인). 불가 시 드럼/벨트 대항 셋업 설계.
- 속도 실험이 가능해지면 `std_ramp_cycle` 을 속도 레벨별 반복 + velocity 모드(ctr_mode=3) 프로파일 사용.
