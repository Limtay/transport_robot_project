# CLAUDE_ECU — ECU·Orin 실행 계층 (재설계 이후)

> **[CLAUDE.md](CLAUDE.md) 가 모노레포 전체 지도**이고, 이 문서는 그 중 **ECU_V3 + orin_ws
> 실행 계층**을 다룬다. DPC_B·hand_ctrl 은 루트 CLAUDE.md 를 볼 것.
>
> 2026-07-30 orin_ws 전면 재설계 이후의 구조·규약을 적는다.

## 구조

| 위치                   | 내용                                                                                       | 상세 문서                                                                                 |
| -------------------- | ---------------------------------------------------------------------------------------- | ------------------------------------------------------------------------------------- |
| `stm_ws/ECU_V3/`     | STM32F446 ECU 펌웨어 (FreeRTOS, CAN AK모터×4, AS5600×5, RS485)                                | **[ECU_V3/CLAUDE.md](stm_ws/ECU_V3/CLAUDE.md)** — 태스크·레지스터맵·에러모델                      |
| `stm_ws/DPC_B/`      | DPC-B(전개 제어) 보드 펌웨어 (STM32F446). Orin 이 부르는 유일한 DPC — DPC-A 는 여기에 물려 중계된다              | 레지스터 맵 미러: `orin_ws/.../core/rd_register_dpc.hpp`                                     |
| `orin_ws/src/`       | ROS 2 Humble 패키지: `orin_firmware_bridge`(C++ 브리지) / `control_cli` / `control_web` / `mgs_tp_msgs` / `mgs01_base_msgs` / `carrier_teleop` | [redesign/](redesign/) 가 설계 진실 원천                                                    |

> ⚠ **`analysis/` · `data/` · `test_plan_*.md` 는 `feature/ecu_ctr` 브랜치에만 있다.**
> 견인력 매핑 분석 파이프라인과 실험 캠페인 계획은 main 에 두지 않는다 (2026-07-30 정리).

## 핵심 문서

- **[DOC/](DOC/)** — **운용 문서 (사용자용)**. 빌드·세팅 / 웹 / CLI / 실험 기록 / 트러블슈팅.
  사용자가 직접 돌릴 때 진입점 → [DOC/README.md](DOC/README.md)
- **[redesign/08_audit_260730.md](redesign/08_audit_260730.md)** — **남은 일 (전수 대조 감사, 최신)**.
  새 세션 진입점. 진행 이력은 [06_migration.md](redesign/06_migration.md).
- **[redesign/](redesign/)** — 설계 진실 원천. 00 개요 / 01 모드·FSM / 02 계층 / 03 인터페이스 /
  04 스케줄러 / 05 프로파일·기록 / 06 이행 / 07 웹 / 08 감사. **코드 주석 68개 파일이 이 문서들을 인용한다.**
- **[testbed_spec.md](testbed_spec.md)** — 하드웨어·레지스터·시간동기 스펙 (§3.1 mask 불변식 / §2.5 drift).
- **[ORIN_SET_GUIDE.md](ORIN_SET_GUIDE.md)** — Orin 200Hz RT 세팅 절차 (latency_timer=1, SCHED_FIFO).
  신규 Orin 이관 시 이 문서대로. 요약본은 [DOC/01_setup.md](DOC/01_setup.md).
- **[Code_modify.md](Code_modify.md)** — 구 수정 스펙 + 작업 로그. **재설계로 대체됐다** —
  새 작업 로그는 `redesign/06`·`08` 에 쓴다. 코드 주석이 인용하고 있어 남겨 둔다.
- **[failsafe_analysis_260717.md](failsafe_analysis_260717.md)** — ECU 페일세이프 분석.
  `rd_system.c`·`rd_can_motor.c` 등 펌웨어 주석이 인용한다.

## 작업 규칙·제약

- **이 환경은 실기 실행 가능**: ROS 2 Humble + `/dev/ttyUSB0`(ECU/DPC) + 빌드된 `orin_ws/install`
  — 브리지 기동·bag 기록을 세션이 직접 수행한다. **단, 실기 기동과 `git commit` 은 사용자 확인 후에 한다.**
- **STM 빌드 불가**: ARM 툴체인 없음(CubeIDE 별도 머신). STM 코드 수정 시 "⚠ STM32CubeIDE 빌드 검증 필요"를 로그에 명기.
- 빌드: `orin_ws/build.sh` (`--test` 로 단위 테스트). **반드시 `orin_ws` 루트에서** — `src/` 안에서 돌리면
  유령 install 이 생겨 고친 코드가 아닌 옛 코드가 돈다.
- Orin 배포: `orin_ws/deploy.sh` (rsync, `--build` 로 원격 빌드까지). 커맨드 치트시트는 `orin_ws/README.txt`.
- 코드 컨벤션: `rd_` 접두사, Checker(진단)/Recovery(복구) 분리, ISR은 volatile 기록만 — ECU_V3/CLAUDE.md 참조.
- **미판독과 정상 0 을 절대 섞지 않는다** (03 §5.3): uint8 미판독 = 255, float = NaN.
  섀도는 0으로 초기화되므로 "안 읽은 블록" 이 정상값처럼 보인다 — 판정은 `delta_tick`·`Reads()` 로 한다.
