# TEST3 Stage 2b — 히스테리시스 계통 특성화 배터리 (40kg, ~30분)

> 2026-07-21 설계(Opus). 캡스턴/스틱션 이력을 계통적으로 훑는다. **핵심 A(반전점)·C(하강속도) ×2,
> 나머지 ×1**, 표준 램프 **1.0 A/s**(시간 단축), 총 24런 ~30분. 실행: `run_battery.py`(무인).

## 고정 조건 (하드)
- m2 단독, `active_motors:="[2]"`, auto_mode=CURRENT, **전류 ≤14A**(40kg 포화 여유 ~40N). rise 기준 분석.
- 표준 램프 **1.0 A/s**(준정적성은 C 계열이 판별). 런 사이 settle 15s, 8런마다 쿨다운 90s.
- ⚠ **모터 온도 SW 미가독**(control 모드=텔레메트리 서브슬롯 정지). 블록 쿨다운 + **사용자 물리 감시(ESTOP)**
  로 대체. 뜨거우면 ESTOP → 오케스트레이터가 다음 런 전 비-IDLE 감지해 자동 중단.

## 실험 패밀리 (17 프로파일 → 24런)

| 패밀리 | 프로파일 | 반복 | 질문 / 분석 산출 |
|--------|----------|------|------------------|
| **A. FORC 반전점** | forc_rev14/12/10/8 | **×2** | 반전점별 하강 분기족 → FORC 다이어그램 |
| **C. 하강 속도** | desc_slow/med/fast (0.3/0.6/1.2 A/s) | **×2** | 속도무관(캡스턴) vs 점성 판별 → 모델 형태 |
| B. 부분 마이너루프 | minor_f10/f07/f04 | ×1 | 루프 닫힘·합동성·복귀점 기억 |
| D. 작동점 사인밴드 | band_hi/mid/lo (9-14/6-11/3-8) | ×1 | 루프폭 vs 작동점(장력 의존) |
| E. 커스텀 변곡 | glide1(드리프트)/glide2(진폭변조) | ×1 | 모델 피팅/검증 데이터 |
| F. 중첩 반전 | nest1/nest2 | ×1 | 기억 소거(wiping-out) 규칙 |

- 전 프로파일 `lint_profiles.py 2 14.0` 통과: reject 없음, [0,14]A, 클램프·음수 0, 세그경계 급단차 ≤0.01A(연속).
- 실행 순서·시간: `hysteresis_w40_manifest.json` (A→C→B→D→E→F, 핵심 앞쪽 배치).

## 실행 (다음 세션, 문답 없이)
```bash
# 사전: 브리지 control_mode+active_motors:=[2] IDLE(기동됨), latency_timer=1, payload 40kg.
bash analysis/traction/run_battery.sh data/profiles/hysteresis_w40_manifest.json   # 백그라운드, ~30분
```
- 자동 안전장치: 프리플라이트 IDLE / 런마다 result.json 검증 / 런 사이 상태 재확인(비-IDLE=중단) / 이상 시 abort.
- 세션 요약: `data/rosbags/battery_hysteresis_w40_<일시>.json` (모든 런 폴더·result·tag).

## 분석 (전량 수집 후, Fable 세션)
- 소스: `battery_hysteresis_w40_*.json` → 각 run_dir. rise 곡선 기준.
- FORC 다이어그램(A) / 하강곡선 중첩=율의존 판별(C) / 마이너루프 닫힘·합동성(B) / 루프폭 vs 작동점(D) /
  임의궤적 예측오차(E) / 기억 소거 검증(F). ⚠ traction_analysis.py 는 램프 중심 — 이력 전용 분석기 확장 필요.
- 발견된 규칙 → 검증 실험/새 가설 반복 테스트로 재계획(사용자 판단).
