# CAMERA_ACTION — jeongae 전개 시 dpy_camera 연동

2026-08-12 작업 기록 (2026-08-13 전 구간 검증 추가). jeongae 전개 시퀀스의 `CAMERA_ACTION`
단계에서 `dpy_camera` 노드로 사진을 찍게 만든 작업의 설계·조작법·실기 검증 결과다.

- 대상 코드: `src/orin_firmware_bridge/src/policy/rd_sequence.cpp:248` (`case Seq::CAMERA_ACTION`)
- 상태: **구현 완료 / jeongae 전 구간 실기 완주 검증 완료** (2026-08-13, ECU+DPC+카메라 실장, §11)
- 관련 문서: `Code_modify.md`(시퀀스 요구사항), `../CLAUDE.md`(워크스페이스 개요)

---

## 1. 왜 Action 이 아니라 Service 인가

`dpy_camera` 에 이미 `std_srvs/Trigger` 서비스(`/dpy_camera/capture`)가 있고, 그것이
"num_shots 장 찍고 응답" 이라는 우리가 필요한 일을 정확히 한다. Action 을 새로 만들면
`dpy_camera` 패키지에 서버를 추가해야 하는데, **그 패키지를 건드리지 않는 것**이 이번
작업의 제약이었다. 진행률 피드백이나 중간 취소가 필요해지면 그때 Action 으로 올린다.

취소가 필요 없다고 판단한 근거: 촬영 중 DPC 는 `WAIT(5)` 에서 정지해 있어 시간을 끌어도
안전하고, 응답 대기에는 이미 60초 상한(`kWaitTicksMax`)이 걸려 있다.

## 2. 데이터 흐름

레이어 규칙(`RdCommand` 는 rclcpp 를 모른다)을 지키기 위해 **서비스 클라이언트는 L3 가
들고, L2 는 콜백만 위임받는다.**

```
RdSequence::Tick()  [200Hz RT 스레드]           rd_sequence.cpp:248
   │  case Seq::CAMERA_ACTION
   │   ├ host_->TriggerCameraCapture()    ← 요청 발사 (논블로킹)
   │   └ host_->CameraCaptureDone(&ok)    ← 매 tick 폴링 (논블로킹)
   ▼  ISlotHost 가상함수                        rd_sequence.hpp
RdCommand  (L2, rclcpp 미의존)                  rd_command.hpp:105
   │   std::function 위임 (SetCameraHost 로 주입)
   ▼                                            rd_node.hpp:51 에서 배선
RdCarrierApi (L3, rclcpp 소유)                  rd_carrier_api.cpp:153
   ├ AsyncParametersClient("dpy_camera")
   │     → num_shots / shot_interval 를 촬영 직전에 push
   └ Client<Trigger>("/dpy_camera/capture")
         → async_send_request, 콜백은 executor 스레드에서 완료 플래그만 세팅
   ▼
dpy_camera (파이썬 노드)                        scripts/capture_node.py
   └ ~/image_raw, ~/image_raw_1.._N (TRANSIENT_LOCAL 래치) + 선택적 파일 저장
```

**RT 스레드에서 절대 블로킹하지 않는다.** `PostAutoWriteTo`/`AutoCommandDone` 과 같은
"발사 후 폴링" 형태이며, 응답 수신은 executor 스레드에서 뮤텍스로 보호된 플래그로만
건너온다 (`camera_mutex_`).

## 3. 시퀀스 동작 (rd_sequence.cpp:248~)

```
DPC_WAIT_CAMERA  →  sys_state == WAIT(5) 도달
                    camera_attempts_ = 0, camera_attempt_pending_ = true
CAMERA_ACTION    →  ① TriggerCameraCapture()
                       성공 → 응답 폴링으로
                       실패(서비스 미준비) → 5초 후 재시도
                    ② CameraCaptureDone(&ok) 폴링 (상한 60초)
                       ok=true  → DPC 127 = ASCEND_1(6) 회수
                       ok=false → 5초 후 재시도
                    ③ 시도 5회 초과 → 포기하고 촬영 없이 회수
DPC_RETRACT      →  ...
```

**실패해도 곧바로 회수하지 않는다** (2026-08-12 사용자 결정). DPC 는 `WAIT(5)` 에 가만히
서 있을 뿐이라 여기서 시간을 끌어도 안전하고, 사진 없이 회수하는 쪽이 "카메라가 왜 안
찍혔지" 를 나중에 찾게 만드는 것보다 나쁘다고 판단했다. 단 무한 재시도는 하지 않는다 —
카메라가 영영 안 살아나는 경우까지 DPC 를 세워 두면 그게 더 나쁘다.

관련 상수 (`rd_sequence.hpp:177~182`):

| 상수 | 값 | 의미 |
|---|---|---|
| `kCameraRetrySec` / `kCameraRetryTicks` | 5초 / 1000 tick | 재시도 간격 |
| `kCameraMaxAttempts` | 5회 | 초과 시 촬영 없이 회수 (`GiveUpCameraLocked`) |
| `kWaitTicksMax` | 12000 tick (60초) | 응답 대기 절대 상한 |

응답 대기에 `kCameraRetryTicks`(5초)가 아니라 `kWaitTicksMax`(60초)를 쓰는 이유: 장수와
간격이 커지면 정상 촬영에도 수십 초가 걸린다. 이 상한은 **요청이 유실돼 응답이 영영 안
오는 경우**를 잡기 위한 것이지 정상 촬영 시간을 자르려는 게 아니다.

## 4. 변경 파일

| 파일 | 변경 내용 |
|---|---|
| `src/policy/rd_sequence.cpp:248~` | `CAMERA_ACTION` 본체 (트리거→폴링→재시도→포기), `GiveUpCameraLocked` |
| `include/policy/rd_sequence.hpp` | `ISlotHost` 에 가상함수 2개, 상수 3개, 상태변수 2개 |
| `include/policy/rd_command.hpp:105` | `SetCameraHost` — `std::function` 위임 (rclcpp 미의존 유지) |
| `include/ros/rd_node.hpp:51` | 콜백 배선 3줄 |
| `include/ros/rd_carrier_api.hpp` | Trigger 클라이언트 / 파라미터 클라이언트 / 완료 플래그 선언 |
| `src/ros/rd_carrier_api.cpp:153~` | `TriggerCameraCapture` / `SendCaptureRequest` / `CameraCaptureDone` |
| `test/unit/test_sequence.cpp` | Fake 에 카메라 2단계 반영, 전 구간 테스트 갱신 |
| `src/dpy_camera/config/dpy_camera_params.yaml` | 기본값 3장 / 2.0초 (아래 §5 ④) |
| `src/dpy_camera/config/dpy_camera.rviz` (신규) | 촬영 결과 확인용 RViz2 설정 (아래 §6/§17) |

`src/dpy_camera/scripts/capture_node.py` 는 이 시점(2026-08-12)에는 **변경하지 않았다.**
2026-08-13 에 두 번 수정했다 — 저장 경로·보관 정책(§6.1: `max_saved_shots`, `_prune()`,
저장 결과를 서비스 응답에 포함)과 사진 토픽 **1-base 번호**(§18).

같은 날 확인용 도구도 정리됐다: `src/dpy_camera/config/dpy_camera.rviz` (신규, §17 —
한 장씩 보기), `record_dpy_shots.sh` (ros2 bag, **롤백해 삭제** — §16).

## 5. N(시간) / M(장수) 조정

**① 운영 중 즉시 반영** — 촬영 직전에 매번 읽어 `dpy_camera` 로 push 하므로 재기동 불필요:

```bash
ros2 param set /firmware_bridge_node jeongae_camera_num_shots 3
ros2 param set /firmware_bridge_node jeongae_camera_shot_interval 2.0
```

**② 기동 시 지정**

```bash
./run.sh project -p jeongae_camera_num_shots:=3 -p jeongae_camera_shot_interval:=2.0
```

**③ 코드 기본값** — `src/orin_firmware_bridge/src/ros/rd_carrier_api.cpp:27-28` (재빌드 필요).
현재 `3` / `2.0`.

**④ dpy_camera 자체 기본값** — `src/dpy_camera/config/dpy_camera_params.yaml` 의
`num_shots` / `shot_interval`. 노드 재기동만 하면 되고 빌드는 불필요. 여기에는
`warmup_sec`, `video_device`, `image_width/height`, `save_dir` 도 있다.

> **jeongae 전개 중에는 ① 이 ④ 를 덮는다.** ④ 는 CLI 로 `/dpy_camera/capture` 를 직접
> 부를 때의 기본값이다. 두 값이 어긋나면 "웹에서는 3장인데 CLI 로는 1장" 같은 혼란이
> 생겨서, 현재는 양쪽 모두 3장 / 2.0초로 맞춰 두었다.

### 실제 걸리는 시간

`shot_interval` 은 **프레임 간 추가 대기**이지 촬영 주기가 아니다. 1920x1080 MJPEG 은
한 장 읽는 데 **~0.41초**가 더 붙는다 (실측). 여기에 워밍업(`warmup_sec`, 기본 1.0초)이
앞에 한 번 붙는다.

```
총 촬영 시간 ≈ warmup_sec + M × (0.41 + shot_interval)
목표 N초 안에 M장 → shot_interval ≈ (N - warmup_sec)/M - 0.41
```

| 설정 | 실측 총 시간 | 실측 장 간격 |
|---|---|---|
| 3장 / 0.5초 | 3.45 ~ 3.54초 | — |
| 5장 / 0.2초 | 4.08초 | — |
| **3장 / 2.0초 (현재 기본값)** | **6.50초** | **2.40 ~ 2.42초** |

## 6. 촬영 결과 확인 방법

사진은 `/dpy_camera/image_raw_1`, `_2`, ... 로 발행된다 (**1-base**, §18). `~/image_raw` 는
depth=1 이라 **마지막 장만** 남으므로, 버스트 전체를 보려면 번호 토픽을 봐야 한다. 장수가
줄면 큰 번호의 게시자는 다음 촬영 때 정리된다 (낡은 사진이 새 버스트의 일부처럼 남지 않도록).

### ⚠ QoS 함정

번호 토픽은 **TRANSIENT_LOCAL(래치)** 로 발행된다. 촬영이 끝난 뒤에 붙는 구독자도 사진을
받게 하려는 것인데, 이때 **구독자도 TRANSIENT_LOCAL 로 붙어야 한다.** 기본값(VOLATILE)
으로 붙으면 연결은 되지만 이미 발행된 사진은 한 장도 오지 않는다.

실측 확인 (촬영 완료 후에 붙은 구독자):

| 구독자 durability | 수신 |
|---|---|
| `TRANSIENT_LOCAL` | **3장** (전부) |
| `VOLATILE` (기본값) | **0장** |

`rqt_image_view` 에는 durability 를 바꾸는 옵션이 없다 — **촬영 후에 열면 빈 화면이다.**

> ⚠ **2026-08-13 정정:** 종전 이 문서는 "rqt 를 촬영 **전에** 미리 띄워 두면 된다" 고
> 적었는데 **그것도 안 된다.** rqt 는 durability(VOLATILE)뿐 아니라
> reliability 도 **BEST_EFFORT** 로 붙어서, 장당 1회만 발행하는 우리 방식과 맞지 않는다.
> 실측·재현 결과는 §14.

### 확인 수단

| 방법 | 명령 | 비고 |
|---|---|---|
| **RViz2** | `rviz2 -d src/dpy_camera/config/dpy_camera.rviz` | **1순위.** Image 디스플레이 하나로, **토픽 끝 숫자를 `_1 → _2 → _3`** 으로 바꿔 가며 장별로 본다. 촬영 후에 띄워도 나온다. §17 |
| **저장 파일 확인** | `eog dpy_camera_shots/` (또는 파일 관리자) | **노드가 꺼져 있어도** 된다. 촬영 한참 뒤에도, ROS 없이도 확인 가능 — 노드를 재시작하면 래치가 날아가므로 이쪽이 유일한 사후 확인 수단이다 |
| 서비스 응답 | `ros2 service call /dpy_camera/capture std_srvs/srv/Trigger {}` | 응답 message 에 **저장 장수·경로·정리 결과**가 들어온다 (§6.1) |
| topic echo | `ros2 topic echo /dpy_camera/image_raw_1 --field header` | Humble 의 echo 는 발행자 QoS 를 자동 감지해 래치 사진이 나온다 |
| ~~rqt~~ | ~~`rqt_image_view /dpy_camera/image_raw_1`~~ | ❌ **쓸 수 없다 — QoS 변경이 불가하다.** §14 |
| 파일 브라우저/CLI | `ls -1t dpy_camera_shots/` | 최신순. `eog dpy_camera_shots/$(ls -t dpy_camera_shots \| head -1)` 로 최신 1장 열기 |

> **확인 수단은 RViz2 + 저장 jpg 둘로 정리했다** (2026-08-13, §19). 전용 뷰어 스크립트
> `view_dpy_shots.py` 는 삭제했다 — RViz 가 같은 일을 더 잘 한다.

### 6.1 저장 경로와 보관 개수 (2026-08-13)

사진은 **`orin_ws/dpy_camera_shots/`** 에 `shot_<날짜>_<시각>_<N>.jpg` 로 쌓인다
(`dpy_camera_params.yaml` 의 `save_dir`, 절대경로). `N` 은 토픽 번호와 같은 **1-base**
장 번호다 (§18).

**보관 정책** — `max_saved_shots` (기본 **15**). 촬영이 끝날 때마다 **최근 N장만 남기고
오래된 파일을 지운다.** 0 이면 무제한.

```bash
ros2 param set /dpy_camera max_saved_shots 30   # 운영 중 즉시 반영 (다음 촬영부터)
```

- **버스트가 아니라 장(파일) 단위**다. 3장씩 찍는 현재 설정에서 `15` = **최근 5회분**.
  회차 경계를 맞추려면 `num_shots` 의 배수로 준다.
- 삭제 대상은 이 노드가 쓴 **`shot_*.jpg` 패턴뿐**이다 — 같은 폴더에 사람이 둔 다른
  파일은 건드리지 않는다 (경로를 잘못 줬을 때의 사고 방지).
- 정리는 **버스트가 끝난 뒤 한 번만** 돈다. 장마다 돌리면 촬영 도중 같은 버스트의 앞
  장을 지우게 되어, 실패한 장이 섞일 때 남는 장수가 들쭉날쭉해진다.

서비스 응답이 결과를 그대로 알려주므로 따로 확인할 필요가 적다:

```
3/3장 캡처 완료 | 저장 3장 → /home/bridge/tp_ws/orin_ws/dpy_camera_shots (오래된 3장 삭제, 최근 15장 유지)
```

실기 검증 (2026-08-13): 6회 연속 촬영 → 파일 수 `3→6→9→12→15→15` 로 상한에서 멈췄고,
남은 15장은 **최신 5개 버스트**였다(첫 버스트 3장 삭제 확인). 저장 jpg 15장 전부 디코딩
정상(1280x720). `ros2 param set max_saved_shots 6` 후 다음 촬영에서 즉시 6장으로 정리됐다.

저장 결과 예 (3장 / 2.0초 — 파일명 끝 숫자가 장 번호이고, 시각이 장 간격 2.4초를 보여준다):

```
$ ls -1t dpy_camera_shots/
shot_20260813_182944_914948_3.jpg
shot_20260813_182942_495145_2.jpg
shot_20260813_182940_051811_1.jpg
```

## 7. 실기 검증 결과 (2026-08-12)

`RdCarrierApi::TriggerCameraCapture` / `CameraCaptureDone` **실제 프로덕션 코드를 그대로
링크한 하네스**로, `RdSequence` 와 동일하게 200Hz 폴링하며 살아 있는 카메라에 호출했다.
(하네스는 scratchpad 에만 있고 레포에는 들어가지 않았다.)

| 항목 | 결과 |
|---|---|
| 파라미터 push | dpy_camera 가 기본값(1장/0.0초)인 상태에서 브리지 값 3/2.0 push → **적용 확인** |
| 촬영 | `3/3장 캡처 완료`, `ok=true`, 6.50초 |
| 장별 토픽 | `image_raw_0.._2` 래치 발행, 1920x1080 bgr8, 간격 2.40~2.42초 |
| 이미지 실물 | 저장 jpg 육안 확인 — 정상 |
| 다른 설정 | 5장/0.2초 → `5/5장`, `image_raw_0.._4` |
| **실패 분기** | dpy_camera 정지 후 호출 → `TriggerCameraCapture()==false` + 에러 로그 (5초 재시도 경로 진입) |
| 런타임 파라미터 | 기동 중인 `/firmware_bridge_node` 에 `ros2 param set` 성공 |
| 유닛테스트 | `test_sequence` **21/21 통과** |

### 검증 중 발견해 처리한 것

- `/dpy_camera` 노드가 **2개** 떠 있었고 둘 다 카메라를 못 여는 상태였다(같은 `/dev/video2`
  를 새 인스턴스는 정상 개방). 이름이 겹쳐 서비스 라우팅도 불확정이었다 — 정리하고 1개만
  재기동했다. **`run_dpy_camera.sh` 를 두 번 띄우지 않도록 주의.**

## 8. ~~미검증 — ECU 무응답 (블로커)~~ → **해소됨 (2026-08-13)**

> 이 절은 2026-08-12 당시 기록이다. **ECU 는 2026-08-13 정상 응답을 확인했고
> (기동 전체읽기 `ecu 216B` OK, `motor_mask` 1회차 검증 OK), 전 구간 완주도 검증했다 —
> §11 참조.** 아래는 당시 증상 기록으로만 남긴다.

- ECU 가 RS485 에 전혀 응답하지 않는다 (`motor_mask` WRITE 10회 전부 실패 → `project`/
  `control` 모드는 노드가 기동을 거부하고 종료).
- **DPC 는 정상**이다 (기동 전체읽기 `dpc 136B` OK).
- 시퀀스 첫 단계가 ECU soft ESTOP 쓰기(`189=0`)이고 실패 시 `AbortLocked` 로 즉시 중단
  되므로, ECU 없이는 DPC 단계까지 가지 못한다.

전체 테스트 290건 중 실패 6건은 전부 `test_active_motors` 이며, 원인은 모두 이 ECU
무응답(노드가 뜨지 못함)이다 — 카메라 변경과 무관하다.

**ECU 전원을 넣으면** 웹의 "jeongae 자동 전개" 패널에서 전 구간(전개 → WAIT(5) → 촬영 →
ASCEND_1(6) → FINISH(8))을 바로 확인할 수 있다.

## 9. ⚠ 웹에서 `sys_state_target=2` 를 직접 써도 촬영되지 않는다

레지스터를 커맨드 패널로 직접 쓰는 것과 jeongae 시퀀스는 **다른 경로**다.

- 직접 쓰기: DPC 가 INIT→DESCEND_1→DESCEND_2→**WAIT(5)** 까지 스스로 가서 **멈춘다.**
  Orin 은 구경만 한다. 회수(6)도 손으로 다시 써야 한다.
- 촬영은 `RdSequence` 안에 있고, 이 FSM 은 `Tick()` 의 `IDLE` 에서 **`/jeongae` 토픽
  트리거로만** 시작된다 (`rd_sequence.cpp:141`). 레지스터를 밖에서 써도 시퀀스는 `IDLE`
  그대로다.

또한 직접 쓸 때는 **`mode(126)=1(AUTO)` 를 먼저 써야 한다.** 안 그러면 DPC 의 FSM switch
자체가 안 도는데, `sys_state` 는 쓴 값을 그대로 되비추므로 **"전개 중" 으로 보이면서
실제로는 아무 일도 일어나지 않는다.**

전 구간을 보려면 웹의 **"jeongae 자동 전개"** 패널(`control_web/www/index.html:435`)의
전개 버튼을 쓴다.

## 10. 남은 TODO

- [x] ~~**ECU 복구 후 jeongae 전 구간 완주 검증**~~ — 2026-08-13 완료 (§11)
- [ ] 재시도/포기 정책(`kCameraMaxAttempts=5`) 이 현장에서 적절한지 실운용으로 확인
      (전 구간 검증 2회 모두 **1/5 시도에서 성공** — 재시도 경로는 실기에서 아직 안 밟혔다)
- [ ] 촬영 결과를 시퀀스가 **어디에 남길지** 미정 — 현재는 토픽 래치 + 선택적 파일 저장
      뿐이고, 어느 전개 회차의 사진인지 시퀀스가 표시하지 않는다
- [ ] `dpy_camera` 는 현재 git untracked (`src/dpy_camera/`) — 커밋 범위 결정 필요
- [x] ~~시퀀스 시작마다 뜨는 `ecu addr180:8 쓰기 거부 [Access Error]` 1회~~ — 원인 규명 완료
      (§12). 시퀀스 결함이 아니라 **CAN 무응답 → ECU 가 ESTOP_SW 에 영구 latch** 된 상태다.

---

## 11. 전 구간 완주 검증 (2026-08-13)

ECU·DPC·카메라를 모두 실장한 상태에서 `/jeongae` 토픽 트리거로 **전 구간을 2회 완주**했다.
2회 모두 동일하게 통과 — 재현성 확인.

기동: `./run.sh project -p enable_dpc_read:=true`
트리거: `ros2 topic pub --once /jeongae mgs01_base_msgs/msg/JeonGae "{open: true}"`

### 단계 전이 (2회차 로그, `t=0` 은 시퀀스 시작)

| t | 단계 | 로그 |
|---|---|---|
| +0.02s | `ESTOP_SET` | ECU `addr189` slot 성공 → `soft ESTOP OK — 50Hz cmd_vel 정지` |
| +0.02s | `DPC_STATE_CHECK` | `DPC 확인 OK (sys_state=0 CTRL)` |
| +0.04s | `DPC_SET_AUTO` | DPC `addr126` 성공 (`mode=AUTO`) |
| +0.07s | `DPC_DEPLOY` | DPC `addr127=2(INIT)` 성공 |
| ~+1.7s | `DPC_WAIT_CAMERA` | `카메라 WAIT 도달 (sys_state=5 WAIT)` |
| ~+1.7s | `CAMERA_ACTION` | `카메라 캡처 요청 (1/5)` |
| +8.4s | ↳ 응답 | `카메라 캡처 응답: 성공 — 3/3장 캡처 완료` |
| +8.4s | `DPC_RETRACT` | `카메라 캡처 완료 — 회수 요청`, `addr127=6(ASCEND_1)` 성공 |
| +14.4s | `DPC_WAIT_RETRACT` | `회수 완료 도달 (sys_state=0 CTRL)` |
| +14.4s | `ESTOP_RELEASE` | `soft ESTOP 해제 OK — 50Hz cmd_vel 재개` |
| +14.4s | `IDLE` | `시퀀스 종료 — jeongae lock ON` |

촬영 사진(2회차): `image_raw_0.._2`, 1280x720 bgr8, stamp 357.840 / 360.022 / 362.210
(간격 2.18 / 2.19초) — 캡처 창 안에 정확히 들어온다. `test_sequence` 유닛테스트 21/21 통과.

### ⚠ `enable_dpc_read:=true` 없이는 시퀀스가 못 간다

기본값이 `false` 라 그냥 `./run.sh project` 로 띄우면 `/carrier/dpc/status` 가
`connected:false` / 전 필드 255 이고, `DPC_STATE_CHECK` 가 `DpcSysState()` 를 영영 못 읽어
**60초 뒤 `AbortLocked("DPC state 미판독…")`** 로 죽는다. 그리고 이 파라미터는 **기동 시에만
읽는다** (`rd_node.cpp:50` → `config_` 로 복사, 런타임 `ros2 param set` 은 스케줄러에 안
먹는다) — 반드시 기동 인자로 줘야 한다.

### 관찰된 것 (시퀀스와는 별개)

- **시퀀스 시작마다 `쓰기 거부 — ecu addr180:8 … [Access Error]` 가 정확히 1회** 뜬다
  (2회차 모두 soft ESTOP 쓰기 직전). 연속 1회로 끝나고 재발하지 않으며 시퀀스는 그대로
  통과한다. cmd_vel(180:8) 경로라 촬영과 무관하지만 원인 미확인.
- `view_dpy_shots.py` 가 **3번 중 2번 "토픽이 없다"** 로 오진했다 (사진은 정상 래치).
  갓 만든 노드가 DDS 디스커버리 전에 그래프를 한 번만 조회해서 생긴 레이스 —
  보일 때까지 재조회하도록 고쳤다 (`DISCOVERY_TIMEOUT`, 이후 6/6 성공).
- 사진 해상도가 **1280x720** 으로 나왔다. §7(2026-08-12)은 1920x1080 이었고, yaml 도
  실행 중인 노드도 **1920x1080 으로 설정돼 있다**(`ros2 param get /dpy_camera image_width`
  → 1920). 즉 **요청 해상도가 적용되지 않고 V4L2 가 조용히 720p 로 떨어졌다.**
  촬영 자체는 정상이라 시퀀스 검증에는 영향 없지만, 해상도가 중요하면 별도 확인 필요.

---

## 12. `ecu addr180:8 쓰기 거부 [Access Error]` 원인 (2026-08-13)

**결론: 시퀀스 결함이 아니다. CAN 에 응답하는 모터가 하나도 없어 ECU 가
`SYS_STATE_ESTOP_SW` 에 영구히 latch 된 상태이고, 그래서 cmd_vel 구간 쓰기가 전부 거부된다.**
전개 시퀀스는 이 구간을 쓰지 않으므로 §11 완주 검증은 영향받지 않는다.

### 인과 사슬

```
CAN 버스에 응답 노드 없음 (M3 포함 전 모터 무응답)
   └ ECU CAN 컨트롤러 TEC ≥ 128 → error passive
        health = HC_BUS_PASSIVE(12)          rd_common.h:95
        ↓
Orin INIT 이 mode(190)=AUTO 를 쓴다 (0→1 전이)
   └ F5 규칙: 모드 변경 시 이번 tick 을 ESTOP_SW 로 강제   rd_system.c:470
        원래는 "1 tick 제동 후 다음 tick 복귀" 설계
        ↓
ESTOP_SW 복귀 조건: !motor_fault && health < HC_THRESHOLD_WARN(2)   rd_system.c:459
   └ health=12 ≥ 2 → **복귀 조건 영원히 불성립 → 영구 latch**
        ↓
mtr_lock = (robot_state != SYS_STATE_AUTO) = 1                     rd_system.c:680
        ↓
cmd_vel 영역(132~187) WRITE → PACKET_ERR_ACCESS                    rd_map_ecu.c:147
   └ addr180:8 이 이 안에 있다 (REG_CMD_VEL_S/E_OFFSET = 132/187)
```

> ⚠ `motor_fault` 는 **0 이다.** 모터가 미접촉이면 `motor_on` 이 0 이라 존재 게이트가
> fault 를 안 만든다(H1 설계, `rd_system.c:451`). 즉 latch 에 **들어간** 이유는 모터가
> 아니라 **mode 전이(F5)** 이고, **못 나오는** 이유가 CAN health 다. 둘 다 있어야 성립한다.

### 근거 (실측)

| 확인 | 값 | 의미 |
|---|---|---|
| `/carrier/ecu/status` `fsm` | **3** | `SYS_STATE_ESTOP_SW` — AUTO 가 아니다 |
| `hs[3]` (can1/모터) | **12** | `HC_BUS_PASSIVE` (심각) |
| `degraded_cnt[3]` | **100 고정** | 감쇠 전혀 없음 = 에러가 지금도 계속 발생 중 |
| `lc[3]` | **1** = `LS_READY` | "첫 패킷 수신 대기" — **기동 후 CAN 패킷을 한 번도 못 받았다** |
| `/carrier/ecu/motor` `comm_err` | **16** = bit4 | M3 의 `AK_COMM_RX_BIT` — **3번 모터도 무응답** |
| 동 `position/velocity/temp` | 전부 0 | 어떤 모터 피드백도 없음 |
| 동 `error_code` | 0 | 모터 자체 fault 는 없다 (→ `motor_fault=0` 확증) |

**cmd_vel 을 계속 흘리면 100% 거부된다** — 0 속도로 `/carrier_cmd_vel` 을 20Hz 발행하니
`연속 201 → 401 → … → 1001회` 로 증가했다. 평소 1회만 보이던 것은 거부가 드물어서가 아니라
**평소엔 쓰기 시도 자체가 없기 때문**이다: `ShouldSkipCmdWrite()` 가 "명령 토픽 100ms 무입력"
이면 쓰기를 건너뛰는데(`rd_carrier_api.cpp:125`), `/jeongae` 도 그 "명령 토픽"에 포함돼
**전개 트리거 순간 게이트가 잠깐 열려 1회 시도 → 거부** 되고, 25ms 뒤 시퀀스가
`SetCmdVelPaused(true)` 로 다시 닫는다. 그래서 전개마다 정확히 1회다.

### `active_motors:=[3]` 로는 해결되지 않는다 (실측)

`-p active_motors:="[3]"` 로 재기동해 `motor_mask=0x04` 적용을 확인했지만
(`INIT motor_mask: addr192 = 0x04 검증 OK`), `fsm=3` / `hs[3]=12` / `degraded_cnt[3]=100`
그대로였다. 마스크는 미접촉 모터를 체커·TX 에서 빼줄 뿐이고, **남은 M3 조차 응답하지
않으므로** CAN 은 여전히 error passive 다.

### 해야 할 일 — CAN 물리 계층

`lc[3]=LS_READY` 는 ECU 가 CAN 프레임을 **단 한 개도 수신한 적 없다**는 뜻이다.
"3번 모터 1개 연결" 이 전기적으로는 맞아도 **CAN 통신은 성립하지 않고 있다.** 확인 순서:

1. **종단 저항** — 노드가 적으면 120Ω 종단이 양 끝에 있는지. error passive 의 최빈 원인.
2. **CAN_H/CAN_L 결선·극성**, 커넥터 접촉
3. **모터 전원** — 로직만 살고 CAN 트랜시버가 안 깨어난 경우
4. **비트레이트 / 모터 CAN ID** 가 ECU 설정(`can_ak.h`)과 일치하는지 — ID 불일치면
   ECU TX 를 아무도 ACK 하지 않아 똑같이 error passive 가 된다

CAN 이 살아 `hs[3] < 2` 가 되면 ECU 는 **자동으로** ESTOP_SW 를 빠져나와 AUTO 로 복귀하고
(`rd_system.c:459`) cmd_vel 쓰기도 즉시 수락된다 — 펌웨어 수정은 불필요하다.

### 곁가지 — i2c 엔코더도 에러 중

`hs[4]=10`(`HC_HW_FAULT`) / `hw_error` bit4 로 AS5600 채널도 에러다. 다만 `degraded_cnt[4]`
는 감쇠 중이고(100→36), `robot_state` 게이트는 **CAN state 만** 본다
(`RD_SYSTEM_UPDATE_STATE(ECU_PERIPHERAL.err.can.state)`, `rd_system.c:559`) 므로 이번
Access Error 와는 무관하다. 별건으로 확인 필요.

### 설계상 짚어둘 점

미접촉 벤치에서 `mode=AUTO` 를 쓰는 순간 **주행이 영구 봉인된다.** F5 의 "1 tick 제동"
의도가, CAN health 가 WARN 이상이면 복귀 불가와 맞물려 영구 latch 로 바뀌기 때문이다.
증상이 `Access Error` 하나뿐이라 원인에서 멀다 — ECU 가 ESTOP_SW 에 머무는 이유를
Orin 이 진단 로그로 한 번 짚어주면 추적이 빨라진다.

---

## 13. 카메라 장치 고정 — 대체 시도 금지 (2026-08-13)

### 발단 — 6주간 엉뚱한 카메라로 찍고 있었다

`§11` 의 전 구간 검증에서 찍힌 사진이 전부 **1280x720** 이었다. yaml 도 노드 파라미터도
`image_width=1920` 인데 왜 720p 인지 한동안 "V4L2 가 조용히 폴백한다" 로 정리해 뒀는데,
실제 원인은 **`video_device: "/dev/video2"` 가 Arducam 이 아니라 노트북 내장 웹캠**
이었기 때문이다.

| 장치 | 정체 | 1920x1080 요청 시 |
|---|---|---|
| `/dev/video0`, `video1` | **Arducam IMX477 HQ** | 1920x1080 ✅ |
| `/dev/video2`, `video3` | Azurewave 내장 웹캠 | 1280x720 (하드웨어 한계) |

`video_device` 를 `/dev/video0` 으로 바꾸니 **1920x1080** 이 나온다. `§11` 의 시퀀스
로직 검증(요청→응답→회수)은 그대로 유효하지만, **그때 찍힌 사진은 Arducam 이 아니다.**

### 대응 ① — 지정한 장치만 연다

`_resolve_device()` (`capture_node.py`) 를 추가해, **yaml 이 지정한 장치 하나만** 열고
실패하면 촬영을 중단한다. 다른 `/dev/video*` 를 뒤지지 않는다.

- **정수 인덱스 경로 제거** — 종전엔 `"0"` 같은 숫자를 `int` 로 바꿔
  `cv2.VideoCapture(0)` 을 불렀다. 정수 인덱스는 **장치 선택을 OpenCV 열거에 맡기는**
  방식이라 어떤 노드가 열릴지 통제할 수 없다. 이제 숫자는 `/dev/video<N>` **문자열
  경로**로 바꿔 넘긴다 — 열거가 끼어들 여지가 없다.
- **열기 전에 검사** — 존재 / 문자장치 / 권한을 미리 갈라 원인별로 다른 메시지를 준다.
  `cv2.VideoCapture` 는 실패해도 예외 없이 `isOpened()==False` 만 주기 때문에, 이걸
  안 하면 "없는 경로 / 권한 없음 / 이미 점유 중" 이 전부 같은 증상으로 보인다.

### 대응 ② — 어떤 카메라를 열었는지 매번 로그로 남긴다

번호는 USB 열거 순서에 따라 바뀌므로 번호만으로는 나중에 "무엇이 찍혔는지" 를 알 수 없다.
`/sys/class/video4linux/<node>/name` 을 읽어 모델명을 찍는다:

```
[INFO] [dpy_camera]: 카메라 장치 확정: /dev/video0  [Arducam IMX477 HQ Camera: Arduc]
```

**이 한 줄이 있었으면 이번 건은 첫날 잡혔다.**

### 실기 검증

| 케이스 | 결과 |
|---|---|
| 정상 (`/dev/video0`) | `3/3장`, **1920x1080**, 로그에 `[Arducam IMX477 …]` |
| 없는 경로 (`/dev/video9`) | `success=False` — "존재하지 않는다 … 다른 번호로 대체 시도하지 않는다" |
| 숫자 문자열 (`"9"`) | `/dev/video9` 로 해석 후 동일 실패 — **열거로 다른 카메라를 열지 않는다** |
| 점유 중 (다른 프로세스가 video0 hold) | `success=False` — "다른 프로세스가 사용 중 … 다른 장치로 대체하지 않는다" (video2 로 안 넘어감) |
| 점유 해제 후 | `3/3장` 정상 복구 |

실패 케이스에서 사진 파일이 하나도 생기지 않는 것도 확인했다.

### 권장 — by-id 경로

번호 고정이 필요하면 yaml 에 by-id 를 쓴다 (USB 열거 순서와 무관):

```yaml
video_device: "/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_IMX477_HQ_Camera_UC517-video-index0"
```

심볼릭 링크는 `realpath` 로 풀어 실제 노드까지 로그에 남긴다.

---

## 14. rqt 로는 사진을 볼 수 없다 — 검증 (2026-08-13)

**결론: `rqt_image_view` 로 이 사진들을 보는 것은 불가능하다.** 사용자 설정 실수가 아니고,
"촬영 전에 미리 띄우기" 로도 해결되지 않는다. **독립된 이유가 둘 있고 둘 다 실측했다.**

### rqt 가 실제로 붙는 QoS

`ros2 run rqt_image_view rqt_image_view /dpy_camera/image_raw_0` 를 띄우고
`ros2 topic info --verbose` 로 구독자 엔드포인트를 직접 확인했다:

```
Endpoint type: SUBSCRIPTION
  Reliability: BEST_EFFORT      ← 우리 발행자는 RELIABLE
  Durability:  VOLATILE         ← 우리 발행자는 TRANSIENT_LOCAL
```

QoS 비호환(`incompatible_qos`) 이벤트는 뜨지 않는다 — **매칭은 된다.** 그래서 rqt 는
토픽을 정상으로 인식하고, 화면만 비어 있다. 이게 원인 추적을 어렵게 만든다.

### 이유 ① VOLATILE — 이미 발행된 사진을 못 받는다

촬영이 끝난 뒤 새로 붙는 구독자:

| 구독 durability | 수신 |
|---|---|
| `TRANSIENT_LOCAL` | **1장** |
| `VOLATILE` (rqt) | **0장** |

→ **촬영 후에 rqt 를 열면 영원히 빈 화면이다.**

### 이유 ② BEST_EFFORT + 장당 1회 발행 — 미리 띄워도 못 받는다

rqt 와 **똑같은 QoS**(BEST_EFFORT+VOLATILE)로 **촬영 전에** 붙어서 촬영을 트리거했다:

| 조건 | 수신 |
|---|---|
| BEST_EFFORT + VOLATILE (rqt 와 동일), 1920x1080 | **0/3장** (3회 반복 모두 0) |
| BEST_EFFORT + VOLATILE, 640x480 로 낮춰도 | **0장** |
| BEST_EFFORT + VOLATILE, 160x120 까지 낮춰도 | **0장** ← 크기 문제가 아니다 |
| **RELIABLE** + VOLATILE, 1920x1080 | **3/3장** ✅ |

크기를 1/50 로 줄여도 안 되고 reliability 만 바꾸면 되므로, **원인은 메시지 크기가 아니라
BEST_EFFORT** 다. 최소 재현 케이스로도 확인했다 (dpy_camera 무관, 별도 프로세스 2개):

| 발행 방식 (RELIABLE+TRANSIENT_LOCAL, 640x480) | BEST_EFFORT 구독자 수신 |
|---|---|
| **1회만** 발행 | **0건** (3회 반복 모두 0) |
| 0.5초마다 10회 반복 발행 | **9건** |

즉 **BEST_EFFORT 는 재전송이 없어 "한 번만 쏜" 샘플을 흘리면 그걸로 끝**이다. 우리는
장당 정확히 1회 발행하므로 rqt 가 그 한 번을 놓치면 복구 수단이 없다.
(같은 프로세스 안에서는 6MB 도 잘 가므로, 프로세스 경계를 넘을 때만 생기는 문제다.)

### 그래서 무엇을 쓰나

| 방법 | 촬영 후에도 되나 | 비고 |
|---|---|---|
| **RViz2 (`dpy_camera.rviz`)** | ✅ | **권장.** Display 별 QoS 를 TRANSIENT_LOCAL 로 지정할 수 있다. §17 |
| **저장 jpg (`dpy_camera_shots/`)** | ✅ | ROS 도 필요 없다. 노드를 재시작한 뒤에도 남는 유일한 수단 |
| `ros2 topic echo … --field header` | ✅ | Humble echo 는 발행자 QoS 를 따라간다 |
| `rqt_image_view` | ❌ | 위 두 이유로 불가 |

### rqt 를 꼭 써야 한다면

**반복 발행하는 릴레이**가 필요하다 — 위 표의 "0.5초마다 반복 → 9건" 이 그 근거다.
`image_raw_N` 을 RELIABLE+TRANSIENT_LOCAL 로 구독해서, 같은 프레임을 1~2Hz 로
**계속** 재발행하는 토픽(VOLATILE)을 따로 만들면 rqt 가 언제 붙어도 그림이 뜬다.
(대신 유휴 CPU 가 붙으므로, 구독자가 있을 때만 돌리는 게 좋다 —
`get_subscription_count() > 0` 게이트.)

---

## 15. rqt 대응 — 반복 발행 (2026-08-13)

§14 에서 "rqt 로는 볼 수 없다" 고 결론냈는데, **미리 켜 두는 경우는 해결했다.**
`capture_node` 가 같은 프레임을 **여러 번 발행**하도록 했다 (방안 B).

```yaml
publish_repeat: 4        # 같은 프레임을 몇 번 쏠지
publish_repeat_gap: 0.5  # 반복 사이 간격(초)
```

### 왜 되는가

rqt 는 BEST_EFFORT 로 구독해 **재전송이 없다.** 1회만 쏘면 놓치는 순간 끝이다.
두어 번 더 쏘면 그중 하나가 도달한다.

### ⚠ 간격이 핵심이다 — 짧으면 아무 효과가 없다

실제 rqt 화면을 스크린샷으로 확인한 결과:

| 설정 | rqt 화면 |
|---|---|
| `repeat=1` (종전) | 빈 회색 화면 |
| `repeat=3` / `gap=0.05` | **빈 회색 화면** ← 반복해도 무효 |
| `repeat=4` / `gap=0.5` | **사진 정상 표시** ✅ |

6MB 프레임이 나가는 데 시간이 걸려서, 간격이 짧으면 반복분이 같은 전송 배치에 묶여
"다시 쏘는" 효과가 사라진다. **`gap` 을 0.5 아래로 내리지 말 것.**

### 비용 — 촬영이 4.5초 길어진다

`(repeat-1) x gap x num_shots` 만큼 늘어난다. 현재 설정(4회/0.5초/3장)에서 **+4.5초**,
실측 총 촬영 시간 **약 14초** (종전 약 8초).

- jeongae 전개의 응답 대기 상한은 `kWaitTicksMax` = **60초**라 여유가 충분하다.
- **rqt 를 안 쓴다면 `publish_repeat: 1`** 로 두면 종전 속도로 돌아간다.
  저장 파일과 `view_dpy_shots.py` 는 반복과 **무관**하다 (RELIABLE 구독이라 1회로 충분).

### 여전히 안 되는 것

**촬영이 끝난 뒤 rqt 를 여는 것은 이걸로 안 풀린다.** 그건 durability(VOLATILE) 문제라
반복 발행과 무관하다 (§14 이유 ①). 촬영 후 확인은 RViz2 나 저장 jpg 를 쓴다 (§19).

> §11/§15 에 남아 있는 `view_dpy_shots.py` 언급은 **당시 기록**이다 — 그 스크립트는
> 2026-08-13 에 삭제됐다 (§19).

| 상황 | 결과 |
|---|---|
| rqt 를 **미리** 켜 두고 촬영 | ✅ 표시됨 (이번 변경) |
| 촬영 후 rqt 열기 | ❌ 여전히 빈 화면 |

---

## 16. ros2 bag 시도 — **롤백함** (2026-08-13)

한때 사진을 `ros2 bag` 으로 받아 두려고 `record_dpy_shots.sh` 를 만들었다.
**되돌렸다** — 확인 수단은 RViz2(§17)와 저장 jpg(`dpy_camera_shots/`)로 충분하고,
bag 은 그 위에 얹히는 군더더기였다:

- 1920x1080 raw 는 **장당 약 6MB**, 3장 버스트가 **약 18MB**. 압축이 없어 폴더가 금방 큰다.
- 사진은 어차피 `dpy_camera_shots/` 에 jpg 로도 남는다 — bag 은 같은 것을 20배 크기로 한 번 더 담는다.
- 녹화를 **SIGINT 로 끊어야** `metadata.yaml` 이 기록된다. SIGKILL 이면 `.db3` 만 남고
  `ros2 bag info` 가 "Could not find metadata" 로 실패한다 (실제로 겪었다) — 확인용 도구로는
  손이 너무 간다.

### 지운 것

- `orin_ws/record_dpy_shots.sh`
- `orin_ws/dpy_camera_bags/` 와 `.gitignore` 의 해당 줄

되살리고 싶으면 `ros2 bag record -e '/dpy_camera/image_raw_[0-9]+'` 한 줄이면 된다 —
**rosbag2 는 발행자 QoS 에 자동으로 맞춰 붙어서** 래치된 사진이 그냥 들어온다 (아래 표).
스크립트가 하던 일은 그 위의 편의(폴더 이름·자동 종료)뿐이었다.

### 왜 rqt 만 못 했나 — 도구별 정리 (이건 유효하다)

| 도구 | 구독 QoS | 촬영 후에 열어도 | 비고 |
|---|---|---|---|
| **RViz2** | **Display 별로 직접 설정** | ✅ | `Durability Policy: Transient Local` 로 두면 끝. §17 |
| 저장 jpg 직접 열기 | (ROS 무관) | ✅ | 노드가 꺼져 있어도 되는 유일한 수단 |
| `ros2 bag` | 발행자 QoS 에 자동 적응 | ✅ | 되지만 **안 쓴다** (위 참조) |
| `rqt_image_view` | **BEST_EFFORT+VOLATILE 고정, 변경 불가** | ❌ | QoS 오버라이드 인자도 무시된다 (실측) |

사진은 노드가 살아 있는 동안 **발행자 안에 계속 남아 있다.** 실측: 마지막 촬영 299초 뒤에
물어봐도 `TRANSIENT_LOCAL` 구독은 1920x1080 을 받아오고, `VOLATILE` 구독은 아무것도 못 받는다.
**데이터가 없는 게 아니라 rqt 가 "과거 건 필요 없다" 고 선언하고 붙는 것**이다.

### §15(rqt 반복 발행)도 이미 되돌린 상태다

`publish_repeat` / `publish_repeat_gap` 파라미터와 반복 발행 루프를 제거했다.
촬영 시간이 **14초 → 7.0초**로 복귀했다. rqt 가 다시 필요해지면 §15 의 값
(4회 / 0.5초, 그 아래로는 무효)으로 되살리면 된다.

---

## 17. RViz2 설정 파일 — **한 장씩, 토픽 번호만 바꿔서** (2026-08-13 개정)

```bash
rviz2 -d src/dpy_camera/config/dpy_camera.rviz
```

Image 디스플레이는 **하나뿐이다.** 왼쪽 Displays 패널에서

```
Shot > Topic > Value :  /dpy_camera/image_raw_1
                                              ↑ 이 숫자만 2, 3, ... 으로 바꾼다
```

바꾸는 즉시 그 장이 뜬다. 래치라 **촬영이 한참 지난 뒤에도** 그대로 남아 있고, 번호를
왔다 갔다 해도 매번 다시 받아온다. `num_shots=3` 이면 `_1`, `_2`, `_3` 이 곧 1·2·3번째 장이다.

### 왜 3장을 한 화면에 쌓지 않는가 (종전 설정에서 바꾼 점)

전에는 Image 디스플레이 3개를 세로로 쌓아 버스트를 한눈에 보게 했는데, 1920x1080 이
장당 세로 1/3 로 눌려서 **정작 사진이 안 보였다.** 한 장을 크게 띄우고 숫자만 바꾸는
쪽으로 되돌렸다. 여러 장을 한눈에 늘어놓고 비교하려면 저장된 jpg 를 파일 관리자나
`eog dpy_camera_shots/` 로 여는 편이 낫다 — 이미지 뷰어가 원래 그 용도의 도구다.

### 핵심은 딱 한 줄

```yaml
Topic:
  Durability Policy: Transient Local   # ← 이게 전부다
  Reliability Policy: Reliable
  Value: /dpy_camera/image_raw_1
```

**RViz2 는 Display 마다 QoS 를 지정할 수 있다.** rqt 가 못 하는 게 바로 이것이고
(§14), 그래서 RViz 는 촬영이 한참 지난 뒤에 띄워도 래치된 사진을 그대로 받아온다.
**토픽 이름을 바꿔도 이 QoS 설정은 그 디스플레이에 그대로 남는다** — 숫자만 고치면 된다.

### 알아 둘 것

- **`Global Status` 경고는 무시해도 된다.** TF 발행자가 없어서 `Fixed Frame` 경고가
  뜨는데, 이 설정은 3D 뷰를 쓰지 않으므로 이미지 표시에는 아무 영향이 없다.
- **없는 번호를 넣으면 빈 화면일 뿐 오류는 아니다.** `num_shots` 보다 큰 번호의 토픽은
  다음 촬영 때 아예 정리되므로(§18), `_4` 가 안 보이면 3장짜리 버스트였다는 뜻이다.
- 사진을 더 크게 보려면 왼쪽 도크 경계를 끌어 좁힌 뒤 `File > Save Config As` 로 덮어쓴다.

---

## 18. 사진 토픽 번호를 **1-base** 로 변경 (2026-08-13)

`image_raw_0.._N-1` → **`image_raw_1.._N`**.

§17 처럼 "토픽 끝 숫자만 바꿔 가며" 보는 방식에서는 0-base 가 계속 걸린다 —
`num_shots=3` 인데 `_0`, `_1`, `_2` 를 보게 되고, `_3` 을 쳐 보면 빈 화면이다.
**"몇 장 찍었나" 와 "몇 번을 보면 되나" 가 어긋나지 않게** 1부터 세도록 맞췄다.

| | 종전 | 지금 |
|---|---|---|
| 토픽 | `image_raw_0` ~ `image_raw_2` | `image_raw_1` ~ `image_raw_3` |
| `frame_id` | `dpy_camera_0` ~ `_2` | `dpy_camera_1` ~ `_3` |
| 저장 파일 | `shot_<시각>_0.jpg` ~ `_2.jpg` | `shot_<시각>_1.jpg` ~ `_3.jpg` |

- 게시자 정리 경계도 함께 바뀌었다 — 남기는 범위가 `1..num_shots` 이므로 조건이
  `idx >= num_shots` 에서 **`shot_no > num_shots`** 다. (0-base 조건을 그대로 두면
  마지막 장의 토픽이 매 촬영마다 지워졌다 다시 생긴다.)
- ⚠ **10장 이상 찍으면 `ros2 topic list` 의 사전순 정렬이 촬영 순서와 다르다**
  (`image_raw_10` 이 `image_raw_2` 앞에 온다). 순서가 중요하면 이름이 아니라
  `header.stamp` 나 저장 파일 시각을 보라.

> ⚠ **이 문서의 §7 / §11 / §14 / §15 로그에 나오는 `image_raw_0..` 은 변경 전 기록이다.**
> 당시 실측값을 그대로 두었을 뿐이니, 지금 동작과 다르다고 혼동하지 말 것.

---

## 19. 확인 수단 정리 — `view_dpy_shots.py` 삭제 (2026-08-13)

RViz2 로 사진이 뜨는 것을 눈으로 확인하고(아래 실측), **`orin_ws/view_dpy_shots.py` 를
삭제했다.** rqt 가 QoS 때문에 못 하는 일을 대신하려고 만든 스크립트였는데, §17 설정이
같은 일을 더 잘 한다 — 확인 수단은 **둘**로 정리됐다:

| | 쓰는 때 |
|---|---|
| **RViz2** (`dpy_camera.rviz`) | 노드가 살아 있을 때. 토픽 번호만 바꿔 가며 장별로 크게 본다 |
| **저장 jpg** (`dpy_camera_shots/`) | 그 외 전부. 노드가 꺼졌거나 재시작한 뒤 — **래치는 노드와 함께 사라지므로 이때는 파일뿐이다** |

`--files` / `--grid` / `--watch` 가 하던 일(목록·타일 비교·새 촬영 자동 감지)은 저장
폴더를 이미지 뷰어나 파일 관리자로 열면 그대로 된다. `eog dpy_camera_shots/` 은
좌우 키로 넘기고, 파일 관리자 썸네일은 `--grid` 와 같은 그림이다.

### RViz2 실측 (2026-08-13, 이 결정의 근거)

- Image 디스플레이가 `RELIABLE / TRANSIENT_LOCAL` 로 붙는 것을 `ros2 topic info
  --verbose` 로 확인했고, **촬영이 끝난 뒤 띄운 RViz 에 래치된 사진이 그대로 떴다**
  (`Status: Ok`). 같은 사진이 `dpy_camera_shots/` 의 jpg 와 일치하는 것도 확인.
- `Global Status: Warn` 은 TF 발행자가 없어서 뜨는 `Fixed Frame` 경고다 — 무시해도 된다.

### 곁가지 — 도크 배치를 설정에 박아 넣었다

처음 띄웠을 때 **Shot 패널이 250px 짜리로 뜨고, 쓰지도 않는 3D 뷰가 화면 대부분을
차지했다.** 사진을 보려고 여는 설정인데 사진이 제일 작았다. RViz 가 도크 배치를 저장하는
`Window Geometry > QMainWindow State` (Qt `saveState()` 의 hex) 를 채워 넣어, 이제 창을
열면 **사진이 창 대부분을 차지하고 그 아래에 Displays 패널**이 온다. 배치를 바꾸고 싶으면
RViz 에서 끌어 맞춘 뒤 `File > Save Config` 로 덮어쓰면 된다.


