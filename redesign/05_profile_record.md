# 05 — 프로파일 스키마 · 기록 규격

> 04 §8 의 **D1~D5** 에 대한 답. 이 문서가 정하는 것은 **`control` 모드의 실험 산출물 계약**이다.
> 재생기(`rd_profile`)·CLI 기록기(`record.py`)·린터(`lint_profiles.py`)·분석
> (`traction_analysis.py`, `run_campaign.py`) 넷이 같은 규격을 봐야 하므로, 여기가 그 한 곳이다.
>
> 기존 진실 원천은 [testbed_spec.md §4~§5](../testbed_spec.md) 이고, 이 문서는 그것을
> **정밀화 + 개명 반영 + 발견된 구멍 메움**으로 대체한다.

---

## 1. 결정 요약 (E1~E6)

| # | 결정 |
|---|---|
| E1 | 프로파일에 **`mode:` 최상위 키 신설** (`current`\|`velocity`\|`position`). 브리지는 **수락 시 검사만** 하고 `auto_mode` 를 대신 바꿔주지 않는다 |
| E2 | 세그먼트 검증을 **확장 *전* 으로 이동** + 타입별 상·하한 확정. 현행 파서의 구멍 **6건** 을 막는다 |
| E3 | `noise` 는 §4.3-4(slew reject) 의 **예외**. 스펙의 "`noise` 는 `slew_rate` 필수" 문구는 **철회** — 구현 불가능한 규칙이었다 |
| E4 | 웹 드로잉의 1순위 출력은 **`ramp` 세그먼트 열**. `custom` 은 자유곡선·외부 CSV 전용. `custom` 보간은 **선형 기본** |
| E5 | `result.json` **schema v2** — 코드 쪽 키 이름을 정본으로 채택하고 스펙이 요구한 4개 + 신규 7개를 추가. `RunProfile.action` Result 확장이 선행 조건 |
| E6 | 기록 폴더 구조·폴더명은 **현행 코드 유지**(자산 호환). 녹화 토픽만 개명, `verify_run_dir` 에 스키마 검사 추가 |

---

## 2. E1 — 프로파일 `mode:` 키 (D1)

### 2.1 문제 — 같은 YAML 이 모드에 따라 다른 물리량이 된다

현행 재생기는 **CURRENT[A] 전용**이라고 선언되어 있다 (`rd_profile.hpp:15`).
01 §3.2 가 `auto_mode` 에 `VELOCITY=4` / `POSITION=5` 를 추가하면 이 전제가 깨진다.

```yaml
- {type: ramp, duration: 20.0, from: 0, to: 10}
```

이 한 줄이 `auto_mode=CURRENT` 에서는 **0→10 A**, `VELOCITY` 에서는 **0→10 RPM**,
`POSITION` 에서는 **0→10 deg** 다. YAML 어디에도 어느 쪽인지 적혀 있지 않다.

"프로파일이 곧 실험 기록"(§4.3-4 의 근거)이라는 원칙에 정면으로 어긋난다 — 기록만 보고
무슨 실험이었는지 복원할 수 없고, 잘못된 `auto_mode` 에서 재생하면 **조용히 다른 실험**이 된다.

> **더 위험한 것**: `rd_bridge.cpp:604~609` 의 wire 직전 최종 클램프는 **단위를 모른다.**
> ```cpp
> const float lim = static_cast<float>(cmd_current_max_);   // 기본 30.0
> if (v > lim) v = lim;
> ```
> `mode: velocity` 프로파일을 재생하면 **RPM 지령이 전류 상한 30 으로 잘린다.** 검증
> (`LoadFromYaml`)과 이 클램프가 **서로 다른 단위계를 쓰는데 둘 다 통과**하므로 에러가 나지 않는다.
> 이 한 곳 때문에라도 모드는 명시되어야 한다.

### 2.2 결정

**최상위 `mode:` 키를 신설한다. 값은 물리량 이름이지 `auto_mode` 번호가 아니다.**

```yaml
name: hysteresis_ramp_v1
mode: current                   # current | velocity | position   (없으면 current + WARN)
limits:
  max_abs: 25.0                 # 단위는 mode 가 결정 (A / RPM / deg)
motors:
  m2:
    - {type: ramp, duration: 20.0, from: 0, to: 10}
```

세 가지 대안 중 이것을 고른 이유:

| 안 | 내용 | 판정 |
|---|---|---|
| (a) 키 없음 — `auto_mode` 상속 | 현행 | ✗ 위 §2.1 그대로. 기록이 자기완결적이지 않다 |
| (b) **`mode:` 선언 + 수락 시 검사** | 불일치면 goal reject | **✔ 채택** |
| (c) `mode:` 선언 + 브리지가 `auto_mode` 자동 전환 | 편하다 | ✗ 아래 |

**(c) 를 기각하는 이유**: `auto_mode` 변경은 out-of-span WRITE 이고 `IDLE` 에서만,
`→DIRECT` 는 shadow 소독을 선행해야 한다 (01 §3.3). 그 절차를 goal 수락 경로에 숨기면
① `RUNNING` 직전에 ECU 제어 모드가 바뀌는 순간이 생기고, ② YAML 오타 하나가 **ECU 를 말없이
재구성**한다. 모드 전환은 사람이 `control_cli config auto_mode` 로 명시적으로 하는 일이다.

### 2.3 수락 규칙 — `mode` ↔ `auto_mode` ↔ `ctr_mode`

`DIRECT` 에서는 단위를 모터별 `ctr_mode` 가 정하므로 (01 §3 표), 검사는 2단이다.

| `mode:` | 허용 `auto_mode` | `DIRECT` 일 때 추가 검사 (활성 모터 전부) |
|---|---|---|
| `current` | `CURRENT(1)` 또는 `DIRECT(2)` | `ctr_mode ∈ {1 CURRENT, 2 CURRENT_BRAKE}` |
| `velocity` | `VELOCITY(4)` 또는 `DIRECT(2)` | `ctr_mode == 3 VELOCITY` |
| `position` | `POSITION(5)` 또는 `DIRECT(2)` | `ctr_mode == 4 POSITION` |

- 그 외 조합(`KINEMATIC(0)`·`CONTROL(3)`·`NONE`)은 **전부 reject**. 사유 메시지에 현재
  `auto_mode` 와 필요한 값을 함께 적는다:
  `"mode: velocity 프로파일 — 현재 auto_mode=current. control_cli config auto_mode velocity 후 재시도"`
- 판정 입력은 **shadow 의 `auto_mode`** (01 §7 권위 모델). read-back 세그를 새로 만들지 않는다.
- 판정 시점은 **goal 수락 시 1회**. 재생 중 `auto_mode` 는 `IDLE` 에서만 바뀌므로 재검사 불필요다.

> 이 규칙이 01 §7 의 "프로파일 가드 판정 기준을 `ctr_mode` 에서 `auto_mode` 로 한 단계 올린다"
> 를 실제로 구현하는 자리다. `DIRECT` 일 때만 `ctr_mode` 까지 내려가고, 그때는 `ctr_mode` 가
> bridge 소유(write 범위 128:52 안)라 shadow 값이 신뢰 가능하다.

### 2.4 `limits` 의 모드 의존 — 무제한을 금지한다

`limits.max_current` 는 단위가 박힌 이름이라 확장되지 않는다. 다음과 같이 바꾼다.

| 키 | 의미 | 비고 |
|---|---|---|
| `limits.max_abs` | 대칭 클램프 `abs(v) ≤ max_abs` | `current`·`velocity` 의 기본형 |
| `limits.range: [lo, hi]` | 비대칭 클램프 | `position` 은 관절 가동범위가 비대칭 → **필수** |
| `limits.slew_rate` | 최대 변화율 [단위/s] | 단위는 `mode` 를 따름. 위반은 reject (§4.3-4 유지) |
| `limits.max_current` | **deprecated 별칭** | `mode: current` 에서만 유효, `max_abs` 와 동의. WARN |

**전역 클램프와의 합성**:

| `mode` | 전역 파라미터 | 프로파일 `limits` 없을 때 |
|---|---|---|
| `current` | `cmd_current_max` (기본 30.0) | 전역값 적용 (현행 유지) |
| `velocity` | **없음** | **reject** — `"mode: velocity 는 limits.max_abs 필수"` |
| `position` | **없음** | **reject** — `"mode: position 은 limits.range 필수"` |

velocity/position 용 전역 파라미터를 새로 만들지 **않는** 이유: 아직 이 모드로 돌린 실험이
0건이고, 안전한 기본값을 모른다. 모르는 값에 기본값을 주는 것보다 **프로파일이 매번 명시하게
강제**하는 쪽이 안전하다. 실험이 쌓여 상식적 상한이 생기면 그때 파라미터로 승격한다.

**wire 직전 클램프(`rd_bridge.cpp:604`)도 같이 고쳐야 한다** — 현재 무조건 `cmd_current_max_` 를
쓴다. 재생 중 유효한 클램프 값(`mode` 로 결정된 실효 limit)을 FSM 이 들고 있다가 그것으로 자른다.
`mode: current` 일 때의 동작은 지금과 완전히 동일하다.

### 2.5 하위호환 — 기존 자산은 그대로 돈다

TEST3 캠페인의 기존 YAML 에는 `mode:` 가 없다.

- `mode:` 누락 → **`current` 로 해석 + WARN 로그 1줄**. reject 하지 않는다.
- `result.json` 에는 **해석된 실효 모드**를 기록한다(§6.2 `mode` 키) — 기록에는 항상 명시된다.
- 린터(`lint_profiles.py`)는 누락을 경고로 보고하고 `--fix` 로 `mode: current` 를 삽입할 수 있게 한다.

---

## 3. E2 — 세그먼트 검증 규칙 정밀화 (D2)

### 3.1 현행 파서에서 찾은 구멍 6건 (전부 코드 근거 있음)

`rd_profile.cpp` / `ExpandSegment()` 기준.

| # | 구멍 | 위치 | 증상 |
|---|---|---|---|
| 1 | **길이 검사가 확장 *뒤***에 있다 | `:314` (`samples.size()*kDt > kMaxDuration`) | `duration: 1e9` → `:126` 에서 `n=2e11` → **push_back 루프가 먼저 OOM/행**. 상한 검사에 도달하지 못한다 |
| 2 | `custom` `rate` 하한 없음 | `:66` (`rate <= 0.0` 만 검사) | `rate: 0.0001`, samples 10개 → `n = 10/0.0001*200 = 2e7` → 모터당 80MB. 위 #1 과 같은 경로 |
| 3 | `step` 의 `t_step` 범위 무검사 | `:149` | `t_step > duration` → 전 구간 `from` 유지, **스텝이 없는 스텝 세그먼트**가 조용히 통과 |
| 4 | `sine`/`chirp` 주파수 무검사 | `:157`, `:167` | `freq: 300` (200Hz 샘플링) → 앨리어싱. 의도와 전혀 다른 파형이 에러 없이 재생된다 |
| 5 | `noise` 의 `std` 음수 무검사 | `:200` (`std::normal_distribution<double> gauss(mean, stddev)`) | 음수 stddev 는 **표준 정의상 UB**. 실험 장비를 UB 로 몰지 않는다 |
| 6 | `stair` `values` 개수 무제한 | `:109~112` | `step_duration` × `values` 곱이 폭발해도 #1 과 같은 경로로 죽는다 |

`stair`·`prbs` 의 `step_duration`/`bit_duration` 양수 검사(`:102`, `:182`)와 "길이 0 tick"
검사(`:302`)는 이미 있다 — 그건 유지한다.

### 3.2 선행 검사 규칙 (구멍 #1·#2·#6 의 공통 처방)

**세그먼트를 펼치기 전에 tick 수를 계산해 검사한다.**

```
kMaxTicks = kMaxDuration * kTickHz = 3600 * 200 = 720,000 tick
```

1. 각 세그먼트의 예상 tick 수 `n` 을 **확장 전에** 산출한다 (모든 타입이 산술식으로 구할 수 있다).
2. `n > kMaxTicks` → 즉시 reject.
3. 누적 `Σn > kMaxTicks` → 즉시 reject (모터별).
4. 통과한 뒤에만 `out->reserve(n)` + 확장.

이렇게 하면 "상한을 넘는 프로파일"은 **메모리를 한 바이트도 쓰기 전에** 거부된다.
`:314` 의 사후 검사는 제거해도 되지만, 방어선 이중화 비용이 0 이므로 남긴다.

**YAML 전문 상한**: goal 의 `profile_yaml` 은 문자열 필드라 DDS 를 통과한다.
**1 MB** 를 상한으로 두고 초과 시 CLI 단계에서 거부한다 (브리지 도달 전).

### 3.3 타입별 파라미터 제약 (확정표)

`✔` = 필수, `○` = 선택. **이 표가 `rd_profile.cpp` 와 `lint_profiles.py` 양쪽의 정본이다.**

| type | 파라미터 | 제약 |
|---|---|---|
| `hold` | `duration`✔ `value`✔ | `duration > 0` |
| `ramp` | `duration`✔ `from`✔ `to`✔ | `duration > 0` |
| `step` | `duration`✔ `from`✔ `to`✔ `t_step`✔ | `duration > 0`, **`0 < t_step < duration`** (신규) |
| `stair` | `values[]`✔ `step_duration`✔ | `step_duration > 0`, **`1 ≤ len(values) ≤ 1000`** (신규) |
| `sine` | `duration`✔ `amp`✔ `freq`✔ `offset`○ | **`0 < freq ≤ 25`** (신규), `freq > 5` 는 WARN |
| `chirp` | `duration`✔ `amp`✔ `f0`✔ `f1`✔ `offset`○ | **`0 ≤ f0,f1 ≤ 25`**, `f0 ≠ f1` 아니면 WARN("sine 을 쓰라") |
| `prbs` | `duration`✔ `low`✔ `high`✔ `bit_duration`✔ | `bit_duration ≥ 0.005` (= 1 tick), `low ≠ high` |
| `noise` | `duration`✔ `mean`✔ `std`✔ | **`std > 0`** (신규), `mean ± 4σ` 가 클램프 범위 밖이면 reject (§3.4) |
| `custom` | `samples[]`✔ `rate`○ `interp`○ | **`1 ≤ rate ≤ 200`**, **`1 ≤ len(samples) ≤ 100,000`** (신규) — §5 |

**주파수 상한 25Hz 의 근거**: 지령 체인이 200Hz 이므로 25Hz 는 **주기당 8 샘플**이다. 그 이상은
재생되는 계단파가 의도한 정현파와 다르고, 100Hz(Nyquist)에 다가갈수록 앨리어싱으로 **완전히 다른
저주파 파형**이 나온다. 실제 액추에이터·기구 대역은 이보다 훨씬 낮으므로 5Hz 초과는 경고로 알린다.

**`custom` `rate ≤ 200` 의 근거**: 200Hz 초과 샘플은 물리적으로 재생할 방법이 없다.
브리지가 안티에일리어싱 필터를 넣어 조용히 다운샘플하면 **재생된 것과 기록된 것이 달라진다**
(§4.3-4 가 금지하는 바로 그것). 변환은 내보내는 쪽(웹·스크립트)의 책임으로 두고, 브리지는
사유를 밝히며 거부한다: `"rate 500 > 200Hz — 내보내기 단계에서 200Hz 로 다운샘플(평균) 후 제출"`.

### 3.4 E3 — `noise` 와 `slew_rate` 의 모순 (D2 의 숨은 지뢰)

`testbed_spec §4.2` 는 `noise` 행에 **"(slew_rate 필수)"** 라고 적었다. 그런데:

- §4.3-4 는 `slew_rate` 위반을 **자동 성형이 아니라 reject** 로 정했다.
- 백색 가우시안 잡음은 정의상 tick 간 델타가 무제한이다. `std` 가 아무리 작아도
  720,000 tick 중 어딘가는 `slew_rate·dt` 를 넘는다.

**즉 "`noise` 는 `slew_rate` 필수" 규칙을 지키면 `noise` 세그먼트는 사실상 항상 reject 된다.**
현행 코드는 이 문구를 구현하지 않았기 때문에(`:196~201` 에 `slew_rate` 관련 코드 없음) 지금까지
문제가 드러나지 않았다 — 스펙과 코드가 어긋난 채 코드 쪽이 옳게 동작하고 있었던 경우다.

**결정 (E3)**:

1. 스펙 §4.2 의 "`slew_rate` 필수" 문구를 **철회**한다.
2. `slew_rate` 검사는 **`noise` 세그먼트 구간을 건너뛴다.** `noise` 를 쓴다는 것이 곧
   "이 구간은 레이트 무제한 백색잡음"이라는 선언이다. 건너뛴 구간은 `result.json` 에
   기록된다(§6.2 `slew_exempt_ticks`).
3. 대신 **진폭 쪽 안전장치**를 둔다: `mean ± 4σ` 가 실효 클램프 범위를 벗어나면 reject.
   근거 — 클램프가 분포의 꼬리를 잘라내면 **실제 재생된 분포가 YAML 이 선언한 정규분포와 달라진다.**
   시스템 식별에 쓰는 입력이 선언과 다르면 동정 결과가 틀린다. 4σ 는 720,000 tick 에서 기대
   초과 횟수 약 45회 (2·Φ(-4)·7.2e5 ≈ 45) 로, 실무상 "거의 안 잘린다" 는 뜻이다.
4. 레이트가 제한된 랜덤 입력이 필요하면 그건 `noise` 가 아니라 **필터링된 잡음**이므로,
   현재는 웹/스크립트에서 미리 만들어 `custom` 으로 제출한다. (전용 세그먼트 타입 추가는
   06 이후 필요할 때.)

### 3.5 이중 구현(린터)의 동기화 — 드리프트를 테스트로 막는다

`analysis/traction/lint_profiles.py` 는 `rd_profile.cpp` 의 파서를 **파이썬으로 다시 구현**한
것이다 (자기 docstring 이 "파서 규칙 미러링" 이라고 밝힌다). 런 낭비를 막는 값어치가 크므로
없애지 않는다. 문제는 **둘이 조용히 갈라지는 것**이다 — 실제로 지금 `REQ` 표(`:10~20`)에는
§3.3 의 신규 제약이 하나도 없다.

**처방**: 공유 YAML 코퍼스로 양쪽 판정을 고정한다.

```
test/profiles/          # 판정이 기대값과 함께 박힌 코퍼스
  valid/*.yaml          # 전부 accept 되어야 함
  reject/*.yaml         # 파일명이 곧 기대 사유 태그 (예: step_tstep_over_duration.yaml)
```

- C++ 쪽: `colcon test` 에서 `RdProfile::LoadFromYaml` 이 `valid/` 전부 통과 · `reject/` 전부 거부.
- 파이썬 쪽: `pytest` 에서 린터가 같은 판정.
- **`reject/` 파일 하나 = §3.3 표의 제약 하나**. 표에 줄을 추가하면 코퍼스에도 파일이 늘어난다.

`custom` 보간(§5.3) 같은 수치 규칙은 코퍼스만으로 안 잡히므로, 대표 파형 3개의 샘플 배열을
`.npy` 로 박아두고 양쪽이 비트 단위로 일치하는지 본다.

---

## 4. E4 — 웹 그래프 드로잉 → 세그먼트 변환 (D3)

### 4.1 폴리라인은 `custom` 이 아니라 `ramp` 열이다

`testbed_spec §5.3` 은 "그래프 드로잉 → custom 샘플" 로 적혀 있다. 이 경로를 **1순위에서 내린다.**

사용자가 캔버스에서 점을 찍어 만든 꺾은선은 **정확히 `ramp` 세그먼트의 열과 동치**다.
점 `(t₀,v₀), (t₁,v₁), …, (tₙ,vₙ)` 는 손실 없이 이렇게 변환된다:

```yaml
- {type: ramp, duration: 20.0, from: 0,  to: 10}
- {type: ramp, duration: 3.0,  from: 10, to: 10}    # 수평 구간은 hold 로
```

| 경로 | 20점 폴리라인의 YAML | 사람이 읽을 수 있나 | 편집 가능한가 |
|---|---|---|---|
| `custom` 샘플 | **9,200개 숫자** (46초 @200Hz) | ✗ | ✗ |
| `ramp` 열 | **19줄** | ✔ | ✔ (숫자를 직접 고칠 수 있다) |

"프로파일 = 실험 기록"인데 기록이 9,200개 숫자면 나중에 그 실험이 무엇이었는지 아무도 모른다.
게다가 `custom` 은 §5 의 최근접 리샘플을 타면서 **계단 왜곡**까지 얻는다 — 원본은 직선이었는데.

**결정**: 웹 편집기는 드로잉 결과를 **세그먼트 열로 내보낸다.**
- 수평(|Δv| < ε) 구간 → `hold`
- 그 외 → `ramp`
- 인접 세그먼트의 기울기가 같으면 병합 (`from`/`to` 연결)
- 스냅 격자: 시간 0.05s(=10 tick), 값은 `mode` 별 유효자릿수(current 0.1A / velocity 1RPM / position 0.1deg)

### 4.2 `custom` 이 남는 자리

지워야 할 타입은 아니다. 이 셋은 `ramp` 열로 못 만든다.

1. **외부 CSV 임포트** — 다른 시스템의 측정 궤적을 그대로 재생 (V1 제어형 검증의 무작위 궤적)
2. **자유곡선(freehand)** — 마우스로 그린 매끄러운 곡선. 점이 수백 개라 세그먼트 열이 오히려 크다
3. **스크립트 생성 파형** — `gen_robust.py` 같은 생성기가 만든 것

세 경우 모두 **사람이 손으로 읽을 물건이 아님이 명백**하다는 공통점이 있다. `custom` 은 그때 쓴다.

### 4.3 리샘플 · 보간 규격

현행 `:81~87` 은 **최근접 이웃**이다.

```cpp
const size_t n = llround(src.size() / rate * kTickHz);
size_t idx = (size_t)(t * rate);          // 최근접(실제로는 floor) — 계단이 생긴다
```

**결정: 기본 보간을 선형으로 바꾸고, `interp: nearest` 로 현행 동작을 남긴다.**

| `interp` | 동작 | 언제 |
|---|---|---|
| `linear` (**기본**) | 선형 보간 | 연속 파형 (거의 전부) |
| `nearest` | 최근접 (현행) | 의도적 계단 — `position` 목표점 열, 이산 레벨 |

세 가지 근거:
1. floor 방식은 rate 가 낮을수록 큰 **계단 불연속**을 만든다. 50Hz 로 그린 매끄러운 곡선이
   200Hz 재생에서 4 tick 마다 튀는 계단이 된다 — 전류 지령이면 그대로 토크 충격이다.
2. `slew_rate` 를 함께 쓰면 그 계단이 **위반으로 잡혀 reject** 된다. 사용자는 매끄러운 곡선을
   그렸는데 "slew 위반" 을 보게 된다. 원인이 자기 파형이 아니라 브리지의 리샘플 방식이다.
3. **기존 기록에 영향이 없다**: `rate == 200` 이면 `idx = floor(i/200·200) = i` 로 두 방식의
   결과가 완전히 같다. 지금까지의 `custom` 은 전부 기본 rate(=200) 였으므로 재생 결과가 바뀌지 않는다.

경계 규칙: `t` 가 마지막 샘플을 넘으면 마지막 값을 유지 (현행 `:85` 와 동일).

### 4.4 상한 (§3.3 재확인)

| 항목 | 상한 | 근거 |
|---|---|---|
| 드로잉 점 개수 | 2,000 | 세그먼트 열로 나가므로 YAML 2,000줄 — 그 이상은 `custom` 을 쓸 상황 |
| `custom` `samples` | 100,000 | 200Hz 로 500초. 1MB YAML 상한과 정합 (샘플당 ~7자 × 10만 ≈ 700KB) |
| `custom` `rate` | [1, 200] | §3.3 |
| YAML 전문 | 1 MB | action goal 문자열 |

---

## 5. E5 — `result.json` 스키마 v2 (D4)

### 5.1 먼저: 스펙과 코드가 다르다

| 항목 | `testbed_spec §5.2` | `record.py` 실제 | 분석이 쓰는 쪽 |
|---|---|---|---|
| 실험 이름 키 | `label` | **`name`** (`:87`) | `name` |
| 종료 시각 키 | `ended_at` | **`finished_at`** (`:96`) | — |
| 프로파일 경로 | — | **`profile_source`** (`:88`) | — |
| `seed` | 필수 | **없음** | — |
| `profile_sha256` | 필수 | **없음** | — |
| `node_params` | 필수 | **없음** | — |
| `bag_dir` | 필수 | **없음** | — |
| 폴더명 | `<label>_<YYYY-MM-DD_HHMMSS>` | **`<name>_<MM-DD_HH-MM>`** (`:40`) | `run_campaign.py:41` 이 이 형식으로 glob |

**결정: 이름은 코드 쪽을 정본으로 삼는다** (`name`/`finished_at`/`profile_source`).
스펙 쪽 이름에는 사용처가 없는 반면 코드 쪽은 이미 84런의 기록 자산에 박혀 있고
`run_campaign.py` 가 읽는다. 스펙 §5.2 문구를 코드에 맞춰 고친다.
**누락된 5개(`seed`·`profile_sha256`·`node_params`·`bag_dir`)는 스펙이 옳다 — 추가한다.**

### 5.2 스키마 v2

```json
{
  "schema_version": 2,

  "name": "hysteresis_ramp_v1",
  "mode": "current",
  "goal_id": 7,
  "success": true,
  "message": "",

  "started_at": "2026-07-27T14:03:11",
  "finished_at": "2026-07-27T14:03:57",

  "profile_source": "/home/limtay/tp_ws/data/profiles/hys_ramp.yaml",
  "profile_sha256": "3f2a…",
  "seed": 42,

  "ticks_executed": 9200,
  "write_err_cnt": 0,
  "clamp_cnt": 0,
  "irregular_tick_cnt": 0,
  "drop_cnt": 0,
  "slew_exempt_ticks": 0,

  "clock_converged": true,
  "drift_ppm": -19600.0,

  "node_params": {
    "bridge_mode": "control",
    "active_motors": [2, 3],
    "auto_mode": "current",
    "cmd_current_max": 30.0,
    "effective_limit": 25.0
  },

  "bag_dir": "/home/limtay/tp_ws/data/rosbags/hysteresis_ramp_v1_07-27_14-03/bag"
}
```

키별 근거 (신규분):

| 키 | 왜 필요한가 |
|---|---|
| `schema_version` | v1 기록 84런이 이미 있다. 분석이 분기할 수 있어야 한다 |
| `mode` | E1. 누락 프로파일도 **해석된 실효 모드**가 여기 남는다 (§2.5) |
| `profile_sha256` | `profile.yaml` 사본이 재생된 것과 같음을 증명. 사본이 유실돼도 동일성 판정 가능 |
| `seed` | `noise`/`prbs` 재현. 미지정 시 시각 기반이라(`rd_profile.cpp:257~259`) **기록하지 않으면 영구 소실** |
| `irregular_tick_cnt` | 1회성 READ 가 write 를 유지한 채 read 세그를 바꾼 tick 수 (01 §6.3-B). 그 tick 은 피드백 필드가 결손이므로 분석이 감안해야 한다 |
| `drop_cnt` | 발행 큐 드롭 누적 (02 §6.3). **시계열에 구멍이 있었는지**를 사후에 아는 유일한 수단 |
| `slew_exempt_ticks` | E3. slew 검사를 건너뛴 `noise` 구간의 길이 |
| `clock_converged`·`drift_ppm` | B1. 클럭 추정기가 미수렴이면 `header.stamp` 가 fallback(Orin 수신 시각)이라 **시간축 정밀도 등급이 다르다.** 분석이 그 런을 섞으면 안 된다 |
| `node_params.effective_limit` | `min(전역, 프로파일 limits)` 의 실제 적용값. clamp_cnt 해석에 필요 |
| `bag_dir` | 폴더를 옮겨도 원 위치를 안다 |

**실패한 런도 반드시 기록한다** (현행 `record.py:85` 주석의 원칙 유지 — "왜 실패했는지가 데이터다").
reject 로 끝난 경우 `ticks_executed=0`, `message` 에 사유, `bag_dir` 은 만들어진 만큼.

### 5.3 선행 조건 — `RunProfile.action` Result 확장

`irregular_tick_cnt`·`drop_cnt`·`seed`·`slew_exempt_ticks` 는 **브리지만 아는 값**이다.
CLI 가 채울 수 없으므로 action Result 에 실어야 한다.

```
# Result  (mgs_tp_msgs/RunProfile — 03 §5.2)
bool   success
string message
uint32 goal_id
uint32 ticks_executed
uint32 write_err_cnt
uint32 clamp_cnt
uint32 irregular_tick_cnt    # 신규
uint32 drop_cnt              # 신규
uint32 slew_exempt_ticks     # 신규
uint64 seed                  # 신규 — 실효 시드 (미지정 시 브리지가 정한 값)
uint8  mode                  # 신규 — 해석된 실효 mode (0=current 1=velocity 2=position)
float32 drift_ppm            # 신규
bool   clock_converged       # 신규
```

Goal·Feedback 은 그대로 둔다. `node_params` 는 CLI 가 `control_cli status --json` 으로 받아
채운다 (브리지가 두 번 말할 필요 없다).

---

## 6. E6 — 기록 폴더 규격 (D5)

### 6.1 구조는 유지한다

```
data/rosbags/<name>_<MM-DD_HH-MM>/
  bag/            # ros2 bag (metadata.yaml + *.db3)
  profile.yaml    # 제출한 YAML 원문 사본
  result.json     # §5.2 schema v2
  console.log     # CLI stdout 사본
```

"실험 1회 = 폴더 1개 = 자기완결 기록" 원칙과 4개 항목 구성은 검증된 규격이므로 바꾸지 않는다.

### 6.2 폴더명 — 코드 형식(`MM-DD_HH-MM`)을 정본으로

스펙은 `<label>_<YYYY-MM-DD_HHMMSS>` 를 적었지만 채택하지 않는다.

| | 스펙안 | 현행 코드 | 판정 |
|---|---|---|---|
| 연도 | 있음 | 없음 | 스펙 우세 |
| 초 단위 | 있음 | 없음 (분 + `_2` 접미) | 스펙 우세 |
| 기존 자산 호환 | ✗ | **✔** (`TEST2_*`·`V*_*`·`s1_*` 전부 이 형식) | **코드 우세** |
| `run_campaign.py:41` glob | 깨진다 | **✔** | **코드 우세** |

- 연도·초는 `result.json` 의 `started_at`(ISO8601)이 이미 정확히 갖고 있다 — 폴더명은 **사람이
  고르기 위한 라벨**이지 유일 키가 아니다. 전역 유일 키는 `(폴더명, goal_id)` 쌍이라고
  `record.py:11~12` 가 이미 정의했다.
- 같은 분 충돌 시 `_2`, `_3` 접미 (`create_run_dir:52~55`) 유지.

**스펙 §5.2 문구를 코드에 맞춰 고친다.**

`name` 우선순위도 코드 쪽으로 확정: `--name` 인자 > YAML `name:` > 프로파일 파일명
(`record.profile_label` + `cli.py:170`). 스펙의 "> `run`" 은 `sanitize_name` 의 빈 문자열
fallback 으로만 남는다.

### 6.3 개명 반영

| | 현행 | 개명 후 |
|---|---|---|
| 패키지 | `testbed_cli` | **`control_cli`** (Q6) |
| 녹화 토픽 | `/carrier/testbed/feedback`, `/carrier/testbed/comm_latency` (2개) | **`/carrier/control/feedback` 1개** |
| `--diag` 시 | — | `+ /carrier/control/comm_diag` |
| 기본 `--bag-dir` | `data/rosbags` | 유지 |

`comm_latency` 가 빠지는 것은 손실이 아니다 — B1 로 시간축 정보가 `ControlFeedback` 안에
흡수되었다 (03 §2.1). 지연 분해 원자료가 필요한 디버그 세션만 `--diag` 를 붙인다.

> **분석 파이프라인 영향**: `analysis/latency/latency_analysis.py` 는 `comm_latency` 토픽을
> 전제로 하고 없으면 예외를 던진다(`:112`). 구 bag 분석용으로 그대로 두고, 신 포맷용
> 경로는 `ControlFeedback` 을 읽도록 추가한다. 06 의 검증 단계에 넣는다.

### 6.4 `verify_run_dir` 강화

현행(`record.py:103~118`)은 **파일 존재만** 본다. `result.json` 이 비어 있거나 키가 빠져도 통과한다.

추가할 검사:
1. `result.json` 이 §5.2 **필수 키를 전부 갖는지** (값이 null 이어도 키는 있어야 한다 — C3 와 같은 규약)
2. `schema_version` 이 아는 값인지
3. `bag/` 안에 `metadata.yaml` 과 `*.db3` 이 **실제로 있는지** (폴더만 생기고 rosbag2 가 실패한 경우 검출)
4. `profile.yaml` 의 sha256 이 `result.json.profile_sha256` 과 **일치**하는지

4번이 핵심이다 — "재생된 것과 기록된 것이 같다"는 §4.3-4 의 원칙을 **런 종료 시점에 실제로 검증**하는
유일한 지점이다. 불일치는 경고가 아니라 **exit 코드로 알린다** (기록이 거짓이면 그 런은 못 쓴다).

---

## 7. 프로파일 자산 마이그레이션

기존 YAML 은 `data/profiles/` 와 각 기록 폴더 안 사본에 있다 (이 레포에는 없고 실행 머신에 있음).

| 단계 | 작업 |
|---|---|
| 1 | 린터에 §3.3 표 + `mode:` 검사 반영 |
| 2 | `data/profiles/*.yaml` 전수 린트 → 신규 제약 위반 목록 확보 |
| 3 | 위반이 있으면 **고치지 말고 먼저 보고** — 이미 실행된 실험이 새 규칙에서 거부된다면 그 실험 데이터의 유효성 자체를 재검토해야 한다 (예: `sine freq > 25` 가 있었다면 그 런은 앨리어싱된 파형으로 돌았다) |
| 4 | `mode: current` 일괄 삽입 (린터 `--fix`) |
| 5 | 기록 폴더 안 `profile.yaml` 사본은 **건드리지 않는다** — 그건 그때 재생된 원문이고, 고치면 `profile_sha256` 이 증명하려는 것이 무너진다 |

---

## 8. 06 에서 결정할 것

| # | 질문 |
|---|------|
| F1 | 마이그레이션 8단계(§_HANDOFF §5)의 각 단계마다 "동작 불변"을 **무엇으로 증명**하는가 — `colcon test` 통과만으로 충분한가, 실기 회귀런(같은 프로파일 재생 후 bag 비교)이 필요한가 |
| F2 | 05 의 스키마 변경(action Result·`result.json` v2)은 8단계 중 **어디에 넣는가** — 7번(메시지 패키지 분리)과 함께인가, 별도 단계인가 |
| F3 | 개명(`testbed`→`control`)의 **원자성** — 토픽·패키지·CLI 를 한 커밋에 바꾸는가, 별칭 기간을 두는가 |
| F4 | 각 단계의 **롤백 기준** — 실기 검증에서 무엇이 나오면 되돌리는가 |
| F5 | STM 작업 4건(§_HANDOFF §4)을 **언제** 넣는가 — 브리지 재설계와 동시에 하면 실패 원인 분리가 안 된다 |
| F6 | 구 bag 84런의 분석 파이프라인 **지원 기간** — 구/신 분기를 언제까지 유지하는가 |

---

## 부록: 결정 요약 카드

```
mode: 키 (E1)
  current | velocity | position — 최상위 필수(누락 시 current + WARN)
  브리지는 검사만, auto_mode 를 대신 바꾸지 않는다 (out-of-span WRITE 를 goal 에 숨기지 않는다)
  DIRECT 에서는 활성 모터 ctr_mode 까지 2단 검사
  limits: max_abs (대칭) / range:[lo,hi] (position 필수) / slew_rate. max_current 는 별칭(deprecated)
  velocity·position 은 전역 기본 클램프 없음 → limits 미지정이면 reject
  ⚠ rd_bridge.cpp:604 의 wire 직전 클램프가 단위를 모른다 — 같이 고쳐야 함

세그먼트 검증 (E2)
  길이 검사를 **확장 전**으로 (현재는 확장 후 → duration:1e9 가 OOM 을 먼저 만든다)
  kMaxTicks = 720,000 / YAML 전문 ≤ 1MB
  신규 제약: 0<t_step<duration / freq ≤ 25Hz(주기당 8샘플) / std > 0 / values ≤ 1000
             custom rate ∈ [1,200], samples ≤ 100,000
  린터(lint_profiles.py)는 유지 — 공유 YAML 코퍼스 + 양쪽 테스트로 드리프트 차단

noise (E3)  slew_rate 검사에서 **면제**. "noise 는 slew_rate 필수" 스펙 문구는 철회(구현 불가능)
            대신 mean±4σ 가 클램프 밖이면 reject (분포가 잘리면 선언과 다른 입력이 된다)

웹 드로잉 (E4)
  폴리라인 → **ramp/hold 세그먼트 열** (custom 아님). 20점 = 19줄 vs 9,200개 숫자
  custom 은 CSV 임포트·자유곡선·스크립트 생성 전용
  보간 기본 = linear (현행 nearest 는 계단 → slew 오탐). rate==200 이면 두 방식 동일 → 기존 기록 무영향

result.json v2 (E5)
  이름은 코드 정본: name / finished_at / profile_source   (스펙의 label·ended_at 폐기)
  추가: schema_version mode profile_sha256 seed irregular_tick_cnt drop_cnt
        slew_exempt_ticks clock_converged drift_ppm node_params bag_dir
  선행조건: RunProfile.action Result 에 7개 필드 확장 (브리지만 아는 값)

기록 폴더 (E6)
  구조 4항목 유지 / 폴더명 <name>_<MM-DD_HH-MM> 유지 (자산·run_campaign glob 호환)
  녹화 토픽 = /carrier/control/feedback 1개 (--diag 시 comm_diag 추가)
  verify_run_dir 강화: 필수 키 · bag 실내용 · **profile.yaml sha256 일치** (불일치는 exit 코드)
```
