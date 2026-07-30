# 분석 세션 인계 — TEST3 Stage 2 데이터 (2026-07-22, Fable 세션용)

> 수집 완료. **이 세션은 분석만 수행** — 실기 명령 없음. 아래 데이터를 바탕으로 결과 정리 +
> 다음 실험 재계획(발견된 규칙 → 검증/반복 테스트 제안)을 작성해 사용자가 판단하도록 한다.

## 수집물 지도

| 세트 | 위치 | 런 수 | 캠페인 문서 |
|------|------|-------|-------------|
| camp_* (기울기별·반복·데드밴드·동특성) | `data/rosbags/c_*_w40_r*/` | 12 | [test_plan_stage2_campaign_260721.md](../../test_plan_stage2_campaign_260721.md) |
| hysteresis_w40 ×3세트 | `data/rosbags/h_*_w40_r*/` | 72(24×3) | [test_plan_hysteresis_260721.md](../../test_plan_hysteresis_260721.md) |
| 세션 요약(JSON, 폴더·result·tag 목록) | `data/rosbags/battery_session_07-21_21-05.json`<br>`data/rosbags/battery_hysteresis_w40_07-22_{12-56,13-38,14-37}.json` | — | — |
| 선행 단발 런(배터리 아님) | `b1_firstlight_*` `b1b_std15_*` `b2_pre_w40_*` | 3 | HANDOFF_260721.md §참고 |

전 런: payload **40kg**, motor **m2**, `active_motors=[2]`, auto_mode=CURRENT, 전류 상한 **14A**(단, b1b/b2_pre 는 15A). 이상 0건(write_err/clamp/ESTOP/FAULT 전무).

## 캘리브레이션 (전 bag 공통)
- `analysis/traction/loadcell_cal.json`: **F[N] = (cnt − 214.6) / 12.627**, R²=0.999988, 선형 0–229N, 포화개시 ~247N.
- `traction_analysis.load_bag()` 이 이 파일을 자동 적용(오프셋 미차감 곱셈만 하므로, **절대 힘이 필요하면
  offset_cnt 를 직접 빼야 함** — b2_pre_w40 분석에서 실수했던 지점, 이번 세션 스크립트 참고).

## 이미 확인된 물리 특성 (재확인용 기준값)
- 데드밴드 ~6.5A, rise slope ~24 N/A (이론밴드 18–25 안).
- **큰 히스테리시스**: fall slope ~10 N/A (rise 대비 절반 이하) — 캡스턴/스틱션 계열로 추정(하강속도 계열이 검증 대상).
- 잔류 이완: 정지 직후 피크 부근 힘이 **15~20초**에 걸쳐 완화 후 잔류(~4.5N @40N 스케일에서). settle 15~20s 는 이 시정수 기준.
- 40kg·15A 절대 피크 214.6N → 14A 배터리는 여유 확보 목적(포화 없음 확인됨, clamp=0).

## 분석 과제 (패밀리별 질문 — 계획 문서에 상세)

| 패밀리 | 데이터 | 핵심 질문 |
|--------|--------|-----------|
| A FORC (forc_rev14/12/10/8, ×2×3세트=24bag) | 반전점별 하강 분기 | FORC 다이어그램 — 반전점에 따라 하강경로가 어떻게 갈라지는가 |
| C 하강속도 (desc_0.3/0.6/1.2, ×2×3=18bag) | 하강 rate 스윕 | **세 하강곡선이 겹치는가(속도무관=캡스턴) vs 벌어지는가(점성)** — 모델형태 결정 |
| B 마이너루프 (minor_f10/07/04, ×1×3=9bag) | 부분하강+재상승 | 루프 닫힘·복귀점 기억(Preisach 합동성) |
| D 사인밴드 (band_hi/mid/lo, ×1×3=9bag) | 작동점별 소진폭 사인 | 루프 폭이 작동점(=캡스턴 장력)에 비례하는가 |
| E glide (glide1/2, ×1×3=6bag) | 부드러운 임의궤적 | 위 패밀리로 세운 모델의 예측오차(검증셋) |
| F nest (nest1/2, ×1×3=6bag) | 중첩 반전 | 안쪽 루프가 바깥 루프 닫힘 시 지워지는가(wiping-out) |
| camp_ramp_slow/med/fast | 기울기 0.2/0.375/0.7A/s | 세션1 기준값(참고용, C 계열과 별개 조건이니 직접 비교 주의) |

## ⚠ 분석기 갭 (이 세션에서 확장 필요)
`analysis/traction/traction_analysis.py` 는 **단일 램프(find_ramps) 중심** — 아래는 미지원, 신규 작성 필요:
- stair/sine/chirp/prbs 세그먼트 분해 (segment_index 필드로 구간 식별은 가능, load_bag 은 그대로 재사용)
- FORC 다이어그램 생성, 하강곡선 중첩도 정량화(예: DTW 또는 구간별 RMS 차)
- 마이너루프 닫힘 판정, 사인루프 폭 추출, 커스텀궤적 예측오차

`load_bag(bag_dir)` (같은 파일) 재사용 권장 — goal_id/segment_index 로 세그먼트 구간 자동 분할 가능(단발 bag 이므로 goal_id 는 항상 동일값 1개).

## 다음 액션 (Fable 세션이 만들 것)
1. 위 분석기 갭을 메워 6개 질문에 답하는 스크립트/결과.
2. **결론 문서** `analysis/result/` 에 신규(옵시디언 `![[img.png]]` 규칙 준수).
3. 발견된 규칙(예: "특정 전류대에서 이력 폭이 급변한다") → **검증 실험 또는 새 가설 반복 테스트 계획**을
   test_plan 형식으로 작성(실행은 하지 말 것 — 사용자가 검토 후 별도 세션에서 승인).
4. `test_index.csv` 에 84개 런 행 추가(payload=40, slope=해당 프로파일 값).
