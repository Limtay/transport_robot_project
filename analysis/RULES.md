# analysis/ 작업 규칙

> **살아있는 문서다.** 2026-08-21 에 전면 재구성하면서 만들었고, 실험을 돌리면서 겪은 것을
> 그때그때 여기에 적어 다듬는다. 규칙이 현실과 어긋나면 **현실이 아니라 이 문서를 고친다.**

---

## 0. 이 디렉터리가 하는 일

`orin_ws` / `stm_ws` 는 **로봇을 돌리는 코드**, `analysis/` 는 **그 로봇이 낸 데이터를 읽는 코드**다.
분석 작업에서 `orin_ws` · `stm_ws` 를 고치지 않는다 — 읽기만 한다. 브리지·펌웨어에 손댈 일이
생기면 그건 별개 작업으로 분리한다.

```
analysis/
  RULES.md              ← 이 문서
  loadcell_cal.json     ← **정준 캘리 상수** (여기 하나뿐이다)
  lib/                  ← 재사용 모듈. 도구가 import 한다
    bagio.py              rosbag2 → numpy (ControlFeedback 전용)
    calib.py              cnt ↔ N
    plotstyle.py          플롯 스타일 + 한글 폰트
    ref_curve_w40_m2.csv  참고 F(I) 곡선 (payload 40 kg·m2, 2026-07-22)
  tools/                ← 실행하는 것들
    profile_preview.py    프로파일 YAML → 승인용 그래프   ★ §2
    run_block.py          셀(=N반복) 실행                ★ §3
    ramp_analysis.py      램프 사이클 bag → 지표·플롯      ★ §4
    loadcell_recal_quick.py  로드셀 간이 재캘리
  profiles/             ← 프로파일 YAML (**추적되는 곳**)
    preview/              승인 대기 그래프 — 런이 끝나면 지운다
  result/               ← 결과 문서·그림 (남기는 것)
  tests/                ← 회귀 테스트
  _old/                 ← 2026-08-21 이전 레거시 (§7)
```

**프로파일을 `data/profiles/` 에 두지 않는다.** `.gitignore` 가 `data/` 를 통째로 제외해서
2026-08-21 에 표준 프로파일 라이브러리가 통째로 사라진 적이 있다 (`std_ramp_cycle.yaml` 등).
`analysis/profiles/` 는 추적된다.

---

## 1. 환경

```bash
source /opt/ros/humble/setup.bash
source ~/tp_ws/orin_ws/install/setup.bash     # mgs_tp_msgs 가 필요하다
```

`bagio` 는 rosidl 역직렬화를 쓰므로 **ROS 환경이 필수**다. 그 대가로 메시지 정의가 바뀌어도
조용히 틀리는 대신 **거부한다** (§7.1).

---

## 2. ★ 실험 승인 절차 — 전류를 넣기 전에 그림으로 합의한다

**전류를 넣는 모든 실험은 이 절차를 거친다.** 예외 없다.

### ① 프로파일을 쓴다

`analysis/profiles/<이름>.yaml`. 계획 문서(`test_plan_*.md`)에 근거가 있어야 한다.

### ② 미리보기 그래프를 만든다

```bash
python3 tools/profile_preview.py profiles/t4_ramp_cycle.yaml
# -> analysis/profiles/preview/t4_ramp_cycle.png
```

그래프에는 **두 가지가 같이** 나온다:

| 위 | cmd current vs 시간 — 세그먼트 경계, `limits` 선, 피크 전류 |
| 아래 | **예측 힘** vs 시간 — abort 선(190 N), 검증 상한(200.7 N), 예측 피크 |

세그먼트 전개는 브리지(`rd_profile.cpp`)와 **같은 200 Hz 격자**로 재현한다. 미리보기가
실제 재생과 다르면 승인이 무의미하다.

### ③ 사용자에게 보여주고 **승인을 받는다**

- 승인 → ④ 로.
- 수정 요청 → YAML 을 고치고 ② 부터 다시. **승인 없이 진행하지 않는다.**

### ④ 실행한다 (§3)

### ⑤ **한 세팅이 끝날 때까지 중간 보고를 하지 않는다**

한 세팅(= 같은 위치·하중의 프로브 + 반복 전부)이 **다 끝나기 전에는 실행만 한다.**
런마다 지표를 늘어놓지 않는다 — 토큰이 그만큼 낭비되고, 어차피 판정은 셀 단위로 한다.
세팅이 끝나면 **한 번에 보고**하고 다음 세팅을 간결하게 묻는다.

예외는 **안전**뿐이다: abort 초과, 슬립, `write_err`/`clamp`, 예상 밖 힘이 나오면
그 자리에서 멈추고 즉시 보고한다.

### ⑥ 끝나면 미리보기를 지우고, 다음을 묻는다

```bash
rm analysis/profiles/preview/<이름>.png
```

`preview/` 는 **승인 대기 중인 것만** 담는다. 남아 있는 그림 = 아직 안 돌린 실험.
런이 끝났는데 그림이 남아 있으면 그건 정리가 안 된 것이다.

> **왜 지우나**: 이 폴더가 "지금 무엇을 승인해야 하는가" 를 나타내는 상태판이기 때문이다.
> 끝난 실험의 그림이 쌓이면 상태판이 아니라 창고가 된다. 남길 가치가 있는 그림은
> `result/` 로 간다.

---

## 3. 실행

```bash
python3 tools/run_block.py profiles/t4_ramp_cycle.yaml --label t4_p1_w40 --repeats 3
```

- `control_cli run <profile> --record --name <label>` 을 반복 호출한다.
  (구 `run_campaign.py` 는 없어진 `testbed_cli` 를 불렀다 — `_old/` 에 있다.)
- 런마다 `result.json` 을 확인하고 `success=false` / `write_err_cnt` / `clamp_cnt` /
  `drop_cnt` / `irregular_tick_cnt` 가 걸리면 **즉시 중단**한다.
- 런 사이 **settle 20 s**. 종료 잔류가 15~20 s 에 걸쳐 이완하므로 그 전에 다음 런을 시작하면
  영점이 오염된다.
- 기록: `data/rosbags/<label>_r<NN>_<일시>/` (bag + profile.yaml 사본 + result.json).

### 실기 조작은 사용자 몫

브리지 기동(`orin_ws/run.sh`), 무게추, 트랙 위치, ESTOP 은 사용자가 한다.
**물어보고 진행한다** — 자동으로 띄우지 않는다.

---

## 4. 분석

```bash
python3 tools/ramp_analysis.py data/rosbags/t4_p1_w40_r01_* ... --label t4_p1_w40
```

지표: `peak_N` `slope_N_per_A`(8~13 A) `deadband_I0_A` `gap_at_7A_N`(이력)
`baseline_return_N` `slip_max_rpm`. 반복이 2개 이상이면 CV 를 같이 낸다.

- **상행/하행 분리는 `segment_index` 로** 한다. 전류 미분 부호로 나누면 홀드 구간 노이즈에서
  부호가 흔들린다.
- **tare 는 첫 hold 세그먼트**. 절대력이 필요하면 `calib.to_N()`, tare 후 상대력이면
  `calib.scale_N()` — 섞어 쓰면 offset 만큼 틀린다 (§7.2).

### 회귀 테스트

```bash
python3 tests/test_pipeline.py
```

정답을 아는 합성 bag 을 통과시킨다. **분석 코드를 고쳤으면 이걸 돌리고 나서 커밋한다.**

---

## 5. 결과 규약

- 결과 문서·그림은 `result/` 에. 파일명에 **날짜(YYMMDD)** 를 넣는다.
- 옵시디언 이미지 링크는 `![[파일명.png]]`.
- 결론 문서에는 **무엇을 확인했는지와 함께 무엇을 확인하지 못했는지**를 적는다.
  (예: "포화 천장은 이번에 측정하지 못했다 — 22.4 kg 이상 필요")
- 원시 데이터가 크면 `.gz` 로. bag 자체는 `data/` 라 추적되지 않는다 — **없어질 수 있다고
  가정하고**, 결론에 필요한 추출값은 `result/` 에 CSV/JSON 으로 남긴다.

---

## 6. 안전선 (2026-08-21 기준)

| 항목 | 값 | 출처 |
|------|-----|------|
| 로드셀 환산 | `F = (cnt − 10.08)/14.6934` | `loadcell_cal.json` |
| 검증 선형 상한 | **200.7 N** | Stage1b 재캘리 |
| **abort 선** | **190 N** (2802 cnt) | 위 아래 10 N 마진 |
| 포화 천장 | **미검증** (추정 ~219 N) | 22.4 kg 이상 필요 |
| 음방향 여유 | 0.69 N — 사실상 없음 | offset 10 cnt. **0 cnt 에 눌리면 클리핑이지 "힘 0" 이 아니다** |
| 전류 절대 상한 | 18 A | 단, 실측상 40 kg·m2 에서 **15.5 A 면 이미 200 N** |

---

## 7. 하지 말 것 — 실제로 당한 것들

### 7.1 토픽을 **이름**으로 고르지 말 것

구 `traction_analysis._detect_format()` 은 `name.endswith('/feedback')` 으로 토픽을 골랐다.
토픽이 `/carrier/testbed/feedback` → `/carrier/control/feedback` 으로 개명되고 메시지가
`TestbedFeedback` → `ControlFeedback` 으로 바뀐 뒤에도 **계속 매칭돼** 옛 바이트 오프셋으로
새 메시지를 읽었다. 크래시하지 않았다. 조용히 `lc = −0.1 N` 상수, `cmd = NaN` 을 냈다.

**`bagio` 는 타입으로 고르고, 안 맞으면 `BagFormatError` 로 거부한다.** 새 도구를 쓸 때도
같은 원칙: **모르는 입력은 조용히 처리하지 말고 거부한다.**

### 7.2 tare 와 절대력을 섞지 말 것

구 파이프라인은 `N_per_count` 를 곱하기만 하고 `offset_cnt` 를 안 뺐다. tare 후 상대력은
그래도 맞지만 **절대력은 offset 만큼 틀린다.** `calib.to_N` / `calib.scale_N` 로 갈라 뒀다.

### 7.3 `data/` 에 없어지면 안 되는 것을 두지 말 것

`.gitignore` 47행이 `data/` 를 통째로 제외한다. 2026-08-21 시점에 `data/profiles/` 전체와
07-21/22 캠페인 bag 84런이 사라져 있었다. 남은 것은 `_old/traction/hys_cache/` 의 추출본뿐이다.

### 7.5 런 시작 전에 **로드셀이 실제로 물려 있는지** 확인할 것

2026-08-21 P1/W1 첫 시도에서 상승 곡선이 **6.8 A 까지 힘 0** 인 하드 데드밴드로 나왔다.
센서·기구 특성으로 보였지만 원인은 **로드셀이 시작 시점에 붙어 있지 않았던 것**이다 —
그 구간은 슬랙을 당기는 데 쓰였을 뿐 힘이 안 걸렸다.

**증상이 물리처럼 보인다는 게 함정이다.** 데드밴드는 이 장비에서 실제로 존재하는 현상이라
(TEST2 I0≈8 A) 그대로 믿기 쉬웠다. 구분법:

- 무부하 baseline 이 **레일 바닥(≈0 cnt)** 에 붙어 있으면 접촉이 없을 수 있다.
  물려 있으면 보통 약간의 프리로드가 잡힌다.
- 저전류 구간에서 힘이 **정확히 0** (노이즈조차 안 움직임)이면 접촉 의심.

런 전에 사용자에게 **체결 상태를 확인**받는다.

### 7.4 없어진 도구를 부르지 말 것

재설계로 `testbed_cli` → `control_cli`, `command_cli` → `control_cli` 로 흡수됐고
`traction_test_mode` 불리언은 폐지됐다 (`bridge_mode:=control auto_mode:=none
read_preset:=control_test`). 구 문서의 명령줄을 그대로 쓰면 기동 게이트에서 거부되거나
빈 bag 이 된다.

---

## 8. `_old/` 에 무엇이 있나

2026-08-21 재구성 때 옮긴 것. **지우지 않은 이유가 있는 것들만** 남겼다.

| 경로 | 왜 남겼나 |
|------|-----------|
| `_old/traction/hys_cache/` (11 MB) | 07-21/22 캠페인 84런의 **유일하게 남은 추출본**. bag 은 없어졌다 |
| `_old/traction/traction_analysis.py` 외 | 구 포맷 bag 을 읽어야 할 때의 참조 구현 |
| `_old/result/` | 07-21 캘리·07-22 히스테리시스 결과. **결론은 여전히 유효**하고 인용된다 |
| `_old/latency/` | Stage 0 통신 특성화 결과 (구 CommLatency 토픽 기반) |

새 코드는 `_old/` 를 import 하지 않는다. 참조만 한다.

---

## 9. 개정 이력

| 날짜 | 내용 |
|------|------|
| 2026-08-21 | 신설. 전면 재구성(레거시 `_old/` 이관, `lib`/`tools`/`tests` 신설), §2 승인 절차 도입 |
| 2026-08-21 | §2⑤ 추가 — 한 세팅이 끝날 때까지 중간 보고 금지 (토큰 절약). §7.5 로드셀 접촉 확인 추가. §4 슬립 지표를 위치 기준으로 교체 |
