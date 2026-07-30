# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Workspace Overview

`transport_robot_project` 는 자율 운반 로봇 한 대를 구성하는 **여러 보드/노드의 모노레포**다. 두 개의 워크스페이스로 나뉜다.

- **`orin_ws/`** — Jetson Orin AGX 에서 도는 ROS2(colcon) 워크스페이스. 로봇의 **마스터(ID 0x01)**. RS485/USB-시리얼(`/dev/ttyUSB0`, 921600 8N1)로 하위 STM32 보드들에 req→resp 한다.
- **`stm_ws/`** — STM32CubeIDE 기반 STM32F446RET6 펌웨어 프로젝트들. 각 보드가 독립 Eclipse managed-build 프로젝트.

보드들은 모두 Dynamixel 2.0-like 패킷 프로토콜로 Orin 과 통신하고, 펌웨어끼리 **공통 드라이버 계층(`rd_*` 접두사)** 과 컨벤션을 공유한다 (한 프로젝트에서 완성한 `rd_uart`/`rd_comm_*` 를 다른 프로젝트로 복사·수정해 재사용하는 패턴).

### 서브프로젝트 지도

| 경로 | 역할 | 상태 |
|------|------|------|
| `orin_ws/src/orin_firmware_bridge` | Orin 마스터 브리지: 200Hz 스케줄러, 레지스터 READ/WRITE, 커맨드 슬롯, jeongae 전개 FSM | 활성 |
| `orin_ws/src/carrier_teleop` | 키보드 teleop → `/carrier_cmd_vel` 가상 조이스틱 | 활성 |
| `orin_ws/src/mgs01_base_msgs` | 공유 msg/srv 정의 (`CommandSet.srv` 등) | 활성 |
| `stm_ws/ECU_V3` | 모바일 베이스 ECU 펌웨어 (CAN 모터×4, AS5600×5, IMU). **완성된 레퍼런스** | 완성 |
| `stm_ws/DPC_B` | DPC 보드 B 펌웨어. Dynamixel(XM430) 모터 제어. **현재 능동 작업 대상** | 작업 중 |
| `stm_ws/hand_ctrl/hand_ctl` | 핸드 컨트롤러 펌웨어. **Dynamixel(`rd_*_dyn`) 코드의 완성본 — DPC_B 작업 시 참조 소스** | 완성 (레퍼런스) |

> **`stm_ws/ECU_V3/CLAUDE.md`** 가 별도로 존재하며 ECU_V3 내부(태스크 FSM, 레지스터 맵, Degraded Counter, IWDG 등)를 상세히 다룬다. ECU_V3 를 만질 때는 그 파일을 우선 참조하라. 이 루트 문서는 워크스페이스 전체와 STM 펌웨어 공통 규약을 다룬다.

## 현재 작업 컨텍스트 (DPC_B Dynamixel)

DPC_B 의 Dynamixel 통신 계층(`rd_comm_dyn.*`, `rd_map_dyn.*`)을 수정 중이다. **`hand_ctrl/hand_ctl` 에 동일 파일의 완성본이 있으니 그쪽 dyn 관련 구현을 참조해 이식·수정**한다. ECU_V3 의 `rd_uart.*` 등 공통 드라이버도 함께 활용한다.

- 두 프로젝트의 dyn 파일은 동명(`Core/Src/rd_comm_dyn.c`, `rd_map_dyn.c` / `Core/Inc/rd_comm_dyn.h`, `rd_map_dyn.h`)이지만 내용이 다르다 — `diff` 로 차이를 확인하며 작업하라 (현재 DPC_B `rd_map_dyn.c` 222줄 vs hand_ctl 271줄).
- hand_ctl 은 dyn 외에 센서 드라이버(`rd_aft150_sc`, `rd_lasf_sc`, `rd_paxini_sc`)와 EtherCAT(`SOES/`)을 포함하므로, **dyn 관련 부분만 골라 참조**한다.
- DPC_B 고유 코드: `rd_comm_dpcb.*`(DPC_A↔DPC_B 4바이트 simple packet), `rd_peripheral_dpcb.*`, `rd_control.c`(`CONTROL_DPC_t`).

## Dynamixel 통신 계층 아키텍처 (STM 공통)

2계층으로 분리되어 있다 (의존 방향: `rd_map_dyn` → `rd_comm_dyn` → `rd_uart`).

```
RD_DYN_LOOP(rs485, ctrl)          ← rd_map_dyn.c, 태스크가 모터별로 호출
  ├ RD_DYN_READ   : Status 패킷 수신 → DYN_Ctrl_t.ram.state 갱신
  ├ RD_DYN_WRITE  : inst/addr 설정대로 Write/Read 패킷 송신
  └ RD_DYN_CHECK  : comm_flag 확인, Write 실패 시 롤백
        │
        ▼  (저수준 패킷 빌더/파서, CRC-16 포함)
  RD_DYNPACK_WRITE / RD_DYNPACK_READ   ← rd_comm_dyn.c
        │
        ▼  (시리얼 + RS485 방향제어)
  RS485_t (rd_uart.h)  →  USART6 + DIR 핀(PB15)
```

- **`DYN_Ctrl_t`** (`rd_map_dyn.h`) 가 모터 1대의 단일 상태: `id`, `mode`/`pre_mode`, `inst`, `addr`(start/size), `ram`(cmd RW + state RO), `comm`(tx/rx 패킷), `error`. 컨트롤 테이블 주소/크기 상수는 `DYN_xm430_w350.h`.
- 사용 패턴(예시는 `DPC_B/Core/Src/main.c` `Startrs485`): `inst`/`addr.start`/`addr.size`/`ram.cmd.*` 를 세팅한 뒤 `RD_DYN_LOOP` 호출. 모드 진입은 `RD_DYN_OPERATE_ON`(Torque ON → mode 설정 순서 보장) → `RD_DYN_UPDATE_CMD`.
- DPC_B 는 `DYN_NUM_MOTORS=3` (`rd_peripheral_dpcb.h`), 모터별 `DPCB_PERIPHERAL.MOT[i].dyn_ctrl`.

## STM 펌웨어 공통 규약

- **반환 코드 `RD_RET`** (`rd_common.h`): `RET_OK(0)` / `RET_NOK(1)` / `RET_WAIT(2)`. 다단계 초기화·논블로킹 핸드셰이크는 `RET_WAIT` 로 재호출을 유도한다.
- **드라이버 객체 1회 INIT**: `RD_UART_INIT` / `RD_RS485_INIT(obj, huart)` 가 struct 를 memset 하고 HAL 핸들을 주입한다 — 정적 초기화 시 huart 를 미리 채우지 말 것. RS485 DIR 핀은 객체 생성 시 `.DIR = {...}` 로 주입(드라이버가 하드코딩하지 않음).
- **RS485 방향 제어**: TX 시 DIR SET, TC 인터럽트로 자동 RX 복귀 (ECU_V3 와 동일 패턴).
- **Checker/Recovery 분리**: 드라이버는 진단(`RET_*` 반환)만, 복구·상태전이는 상위 태스크에서. (ECU_V3 의 핵심 설계 원칙, 다른 보드도 따른다.)
- **네이밍/주석**: `RD_` 접두사, 대문자 함수명, 한국어 주석. 새 코드는 주변 코드의 주석 밀도와 ISR/태스크 소유권 규칙을 그대로 따른다.
- **태스크 구조**: `main.c` 의 `StartXxx` 진입점(CubeMX 생성)이 무한 루프를 돌며 `RD_*` 호출. FreeRTOS(CMSIS-RTOS v2). DPC_B 태스크: `defaultTask`/`comm`/`peri`/`ctrl`/`rs485` + `ResmapMutex`.

## Build & Run

### STM 펌웨어 (`stm_ws/*`)

- IDE: **STM32CubeIDE** (Eclipse managed builder). **독립 Makefile/CLI 빌드 하네스 없음** — `Debug/` 의 `makefile`/`*.mk` 는 IDE 가 생성·관리하는 산출물이며 손으로 호출하지 않는다.
- 빌드: STM32CubeIDE 에서 Project → Build (`Ctrl+B`). 플래시/디버그: Run → Debug (ST-Link).
- 핀맵/페리페럴 변경: 해당 프로젝트의 `*.ioc` 를 STM32CubeMX 로 열어 재생성.
- **이 환경(Windows 개발 노트북)에는 ARM 툴체인이 없을 수 있다** — 코드 수정 후 컴파일 검증은 사용자가 STM32CubeIDE 에서 수행한다고 가정하고, 그 점을 명시하라.

### Orin ROS2 (`orin_ws/`)

```bash
cd ~/orin_ws        # (Orin/리눅스 기준 경로)
./build.sh          # realsense + mgs01_base_msgs + orin_firmware_bridge 빌드
./run.sh            # firmware_bridge_node (comm_test_node) 실행
./cli.sh            # 커맨드 CLI
# 코드만 빠르게:
colcon build --packages-select orin_firmware_bridge && source install/setup.bash
```

`mgs01_base_msgs` 변경 시 메시지 의존 패키지 재빌드 필요. Orin 실시간 튜닝(latency_timer, rtprio, SCHED_FIFO 등)은 **`ORIN_SET_GUIDE.md`** 참조 — 통신 손실/주기 밀림 트러블슈팅의 1차 출처다.

## Git 추적 범위 (주의)

`.gitignore` 가 빌드 산출물(`Debug/`, `*.o/*.elf/*.bin/*.su`, colcon `build|install|log/`)과 에디터 파일을 제외한다. 단,

- `stm_ws/DPC_B` 는 **추적 중**(소스 + 일부 `Debug/*.mk` 가 status 에 잡힘).
- `stm_ws/hand_ctrl/` 는 현재 **untracked**.
- `.gitignore` 주석은 "ECU_V3 만 추적"이라 적혀 있으나 실제 규칙은 ECU_V3 의 `Debug/`/`Release/` 만 제외하므로 다른 STM 프로젝트 소스도 커밋 가능. 커밋 전 `git status` 로 의도한 파일만 들어가는지 확인하라.

## 작업 로그 / 설계 문서

- **`Code_modify.md`** — Orin 스케줄러·커맨드 시스템·jeongae 전개 시퀀스의 요구사항 + 날짜별 구현/테스트 로그. Orin↔STM 레지스터 의미(soft_estop=addr189, hw_reset=addr5/54, DEFINE 잠금 등)와 미확정 TODO 가 여기 있다.
- **`ORIN_SET_GUIDE.md`** — Orin 시스템 세팅·실시간 튜닝·트러블슈팅 표.
- **`stm_ws/ECU_V3/CLAUDE.md`** — ECU_V3 펌웨어 내부 상세.
