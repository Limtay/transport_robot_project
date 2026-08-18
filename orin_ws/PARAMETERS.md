# PARAMETERS — 조정 가능한 값들의 코드상 위치

2026-08-13 작성. "이 값 어디서 바꾸지" 를 찾는 데 드는 시간을 줄이기 위한 지도다.

**분류 기준은 "어떻게 바꾸는가"** 다. 값의 의미보다 이게 훨씬 자주 발목을 잡는다 —
런타임에 바꿀 수 있는 줄 알고 `ros2 param set` 했는데 기동 시에만 읽는 값이라 아무 일도
안 일어나는 경우가 실제로 있었다 (`enable_dpc_read`, 2026-08-13).

| 등급 | 바꾸는 법 | 반영 시점 |
|---|---|---|
| **A. 런타임** | `ros2 param set` | 즉시 (다음 사용부터) |
| **B. 기동 인자 / YAML** | 노드 재기동 | 재기동 후 — **빌드 불필요** |
| **C. C++ 상수** | `colcon build` + 재기동 | 재빌드 후 |
| **D. STM 펌웨어** | STM32CubeIDE 빌드 + 플래시 | 플래시 후 |

---

## A. 런타임 변경 가능 (`ros2 param set`)

### A-1. 카메라 — `/dpy_camera`

`capture_node.py` 의 `on_capture()` 가 **호출 때마다 전부 다시 읽는다**
(`src/dpy_camera/scripts/capture_node.py:97~`). 그래서 전부 런타임 반영이다.

| 파라미터 | 기본값 | 선언 위치 | 의미 |
|---|---|---|---|
| `num_shots` | 3 | `capture_node.py:52` | 한 번에 찍을 장수 |
| `shot_interval` | 2.0 | `:58` | 장 사이 **추가** 대기 [s] |
| `warmup_sec` | 1.0 | `:56` | AE/AWB/AF 수렴 대기 [s] |
| `save_dir` | (아래 YAML) | `:62` | 저장 경로. 빈 문자열이면 저장 안 함 |
| `max_saved_shots` | 15 | `:71` | **최근 N장만 보관**, 0=무제한 |
| `image_width/height` | 1920/1080 | `:49-50` | ⚠ 실제로는 1280x720 으로 떨어진다 (§확인된 이슈) |
| `video_device` | `/dev/video2` | `:47` | |
| `use_mjpg` | true | `:60` | |
| `publish_image` | true | `:73` | 토픽 발행 여부 |
| `frame_id` | `dpy_camera` | `:48` | |

```bash
ros2 param set /dpy_camera max_saved_shots 30
ros2 param set /dpy_camera num_shots 5
```

> **기본값의 정본은 YAML 이다** (`src/dpy_camera/config/dpy_camera_params.yaml`, 등급 B).
> `capture_node.py` 의 `declare_parameter` 두 번째 인자는 YAML 없이 띄웠을 때의 폴백이라
> 둘이 다르다 (예: `num_shots` 코드 1 / YAML 3). **평소 쓰는 값은 YAML 쪽**이다.

### A-2. 브리지 — `/firmware_bridge_node`

| 파라미터 | 기본값 | 선언 위치 | 비고 |
|---|---|---|---|
| `jeongae_camera_num_shots` | 3 | `src/ros/rd_carrier_api.cpp:27` | 촬영 직전 읽어 dpy_camera 로 push (`:165`) |
| `jeongae_camera_shot_interval` | 2.0 | `rd_carrier_api.cpp:28` | 〃 (`:166`) |
| `cmd_vel_guard_enable` | true | `src/ros/rd_node.cpp:11` | 콜백 있음 (`rd_node.cpp:256~`) |
| `cmd_vel_zero_skip` | false | `rd_node.cpp:16` | 콜백 있음. 정본 입구는 서비스 `/carrier/cmd_vel_zero_skip` |

> ⚠ **jeongae 전개 중에는 A-2 가 A-1 을 덮는다.** 전개 경로에서는 브리지가 자기
> `jeongae_camera_*` 를 dpy_camera 로 밀어 넣으므로, `/dpy_camera` 쪽 `num_shots` 를
> 바꿔도 전개 촬영에는 반영되지 않는다. CLI 로 `/dpy_camera/capture` 를 직접 부를 때만
> A-1 이 쓰인다. (혼선을 줄이려고 현재 양쪽 모두 3장 / 2.0초로 맞춰 뒀다.)

### A-3. 런타임 서비스 (파라미터는 아니지만 같이 쓰는 입구)

| 서비스 | 타입 | 용도 |
|---|---|---|
| `/carrier/jeongae_lock` | `SetBool` | 전개 lock on/off — **시퀀스는 끝날 때마다 자동 lock** 되므로 재전개 전 `false` 필요 |
| `/carrier/cmd_vel_zero_skip` | `SetBool` | 0 수렴 스킵 |
| `/carrier/command_set` | `CommandSet` | 레지스터 직접 READ/WRITE |
| `/dpy_camera/capture` | `Trigger` | 촬영 (응답에 저장 장수·경로·정리 결과 포함) |

---

## B. 기동 시에만 읽는 값 (재기동 필요, 빌드 불필요)

### B-1. 브리지 기동 파라미터 — `src/orin_firmware_bridge/src/ros/rd_node.cpp`

전부 `config_`(`RdConfig`) **구조체로 복사된 뒤 그 사본만 쓰인다.** 그래서 런타임
`ros2 param set` 은 **먹지 않는다** — 반드시 기동 인자로 줘야 한다.

| 파라미터 | 기본값 | 선언 위치 |
|---|---|---|
| `bridge_mode` | `project` | `rd_node.cpp:26` — project/control/manual |
| `auto_mode` | `current` | `rd_node.cpp:167` — none/kinematic/current/direct/velocity/position |
| `read_preset` | `control` | `rd_node.cpp:39` |
| `active_motors` | `[1,2,3,4]` | `rd_node.cpp:89` — 빈칸은 `""` (`[]` 아님) |
| **`enable_dpc_read`** | **false** | `rd_node.cpp:50` — ⚠ **전개 시퀀스에 필수** (아래) |
| `enable_pcu_read` | false | `rd_node.cpp:51` |
| `enable_ecu_read` | true | `rd_node.cpp:53` |
| `comm_diag_enable` | false | `rd_node.cpp:47` |
| `stream_timeout` | 0.1 | `rd_node.cpp:60` |
| `cmd_current_max` | 30.0 | `rd_node.cpp:61` |
| `cmd_vel_topic_timeout` | 0.1 | `rd_node.cpp:12` — 이 시간 무입력이면 cmd_vel 쓰기 스킵 |
| `cmd_vel_zero_timeout` | 30.0 | `rd_node.cpp:14` |
| `imu_frame_id` | `imu_link` | `rd_node.cpp:17` |

> ⚠ **`enable_dpc_read:=true` 없이는 jeongae 전개가 안 된다.** 기본값이 false 라
> `/carrier/dpc/status` 가 `connected:false`/전 필드 255 로 나오고, `DPC_STATE_CHECK` 가
> DPC 상태를 못 읽어 **60초 뒤 abort** 한다.
> ```bash
> ./run.sh project -p enable_dpc_read:=true
> ```

기본값 정의 자체는 `include/orin_firmware_bridge/rd_config.hpp` 에도 있다(등급 C) —
`rd_node.cpp` 의 `declare_parameter` 두 번째 인자가 실질 기본값이므로 **보통 여기만 보면 된다.**

### B-2. 카메라 YAML — `src/dpy_camera/config/dpy_camera_params.yaml`

**카메라 기본값의 정본.** 노드 재기동만 하면 되고 빌드는 불필요하다
(`run_dpy_camera.sh` 가 `src/` 의 원본을 직접 실행한다 — install 로 복사되지 않는다).

현재: `num_shots 3` / `shot_interval 2.0` / `warmup_sec 1.0` / `video_device /dev/video2`
/ `save_dir /home/bridge/tp_ws/orin_ws/dpy_camera_shots` / `max_saved_shots 15`

### B-3. 기동 프리셋 — `run.sh`

`project` / `manual` / `traction` / `control <auto_mode>` 조합이 여기 하드코딩돼 있다
(`run.sh:27~55` 의 `case` 블록). 그 외 `-p key:=value` 는 그대로 통과된다.

---

## C. C++ 상수 (재빌드 필요)

### C-1. 전개 시퀀스 튜닝 — `include/orin_firmware_bridge/policy/rd_sequence.hpp:170~182`

| 상수 | 값 | 의미 |
|---|---|---|
| `kTickHz` | 200 | 시퀀스 tick 주파수 |
| `kWaitTimeoutS` | 60 | 각 단계 대기 절대 상한 [s] (실측 전개 ≈40초 + 여유 20초) |
| `kWaitTicksMax` | 12000 | = kTickHz × kWaitTimeoutS |
| `kCameraRetrySec` | 5 | 촬영 실패 시 재시도 간격 [s] |
| `kCameraMaxAttempts` | 5 | 초과 시 촬영 없이 회수 |

### C-2. 레지스터 주소 · 상태값

Orin 쪽 정의는 **STM 펌웨어의 거울**이다. 바꾸려면 D 와 **반드시 같이** 바꿔야 한다.

| 파일 | 내용 |
|---|---|
| `include/.../core/rd_register_ecu.hpp` | ECU 주소·상수. `REG_SOFT_ESTOP_OFFSET=189`(`:70`), `REG_MODE_OFFSET=190`(`:149`), `MODE_AUTO=1`(`:151`) |
| `include/.../core/rd_register_dpc.hpp` | DPC 주소·상태. `REG_MODE_OFFSET=126`(`:96`), `MODE_AUTO=1`(`:125`), `STATE_WAIT=5`(`:145`), `STATE_ASCEND_1=6`(`:146`) |
| `include/.../rd_config.hpp` | `RdConfig` 구조체 기본값 (`active_motor_mask=0x0F` `:107` 등) |

### C-3. 카메라 기본값 (브리지 쪽)

`src/ros/rd_carrier_api.cpp:27-28` — `jeongae_camera_num_shots` / `_shot_interval` 의
코드 기본값. 평소엔 A-2 로 런타임 조정하면 되므로 여기까지 갈 일은 드물다.

---

## D. STM 펌웨어 (CubeIDE 빌드 + 플래시)

| 파일 | 내용 |
|---|---|
| `stm_ws/ECU_V3/Core/Inc/rd_register_ecu.h` | ECU 레지스터 맵 정본. cmd_vel 잠금 구간 `REG_CMD_VEL_S/E_OFFSET = 132/187`(`:93-94`) |
| `stm_ws/ECU_V3/Core/Inc/rd_common.h` | `HC_*` health 코드와 임계값 (`HC_THRESHOLD_WARN=2` `:104`) |
| `stm_ws/ECU_V3/Core/Src/rd_system.c` | 상태 FSM. `MODE_STATE()`(`:27`), ESTOP_SW 복귀 조건(`:459`), `mtr_lock`(`:680`) |
| `stm_ws/DPC_B/Core/Inc/rd_register_dpcb.h` | DPC 레지스터 맵 정본 |
| `*.ioc` | 핀맵/페리페럴 — CubeMX 로 재생성 |

> 이 환경에는 ARM 툴체인이 없다 — 펌웨어 수정 후 컴파일은 STM32CubeIDE 에서 해야 한다.

---

## 확인된 이슈 (파라미터로는 못 고치는 것)

- ~~**해상도** — 1920 설정인데 1280x720 으로 나온다~~ → **해결 (2026-08-13)**.
  V4L2 폴백이 아니라 **`video_device` 가 엉뚱한 카메라를 가리키고 있었다.**
  `/dev/video2` = 노트북 내장 웹캠(1280x720 한계), `/dev/video0` = Arducam IMX477.
  `video_device` 를 `/dev/video0` 으로 바꾸니 1920x1080 정상. 이제 노드가 촬영마다
  **어떤 카메라를 열었는지 로그로 남긴다** (`카메라 장치 확정: … [Arducam IMX477 …]`).
- **`shot_interval` 은 촬영 주기가 아니라 프레임 간 추가 대기다.** 한 장 읽는 데
  ~0.4초가 더 붙으므로 `2.0` → 실제 간격 ≈2.2~2.4초.
  `총 시간 ≈ warmup_sec + M × (0.4 + shot_interval)`
- **ECU 가 `FAULT`/`ESTOP_SW` 면 cmd_vel(180:8) 쓰기가 전부 거부된다.** 파라미터로 못
  푼다 — CAN 물리 계층 문제다. 상세는 `CAMERA_ACTION.md` §12.

## 관련 문서

- `CAMERA_ACTION.md` — 카메라 연동 설계·조작·검증 (§5 장수/간격, §6.1 저장·보관)
- `ORIN_SET_GUIDE.md` — Orin 실시간 튜닝 (rtprio, latency_timer 등 OS 레벨)
- `Code_modify.md` — 시퀀스 요구사항과 레지스터 의미
- `../CLAUDE.md` — 워크스페이스 전체 개요
